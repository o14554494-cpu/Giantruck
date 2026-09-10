"""OpenMV vision node for the intelligent collection car.

The camera detects red/yellow blocks, estimates robot-relative coordinates,
and sends complete target frames to the STM32 over UART3.

OpenMV UART3 wiring:
    P4 / TX -> STM32 PC11 / UART4_RX
    P5 / RX <- STM32 PC10 / UART4_TX
    GND      -> STM32 GND

Packet format (ASCII, XOR checksum):
    $F,sequence,target_count*CS
    $T,sequence,index,color,lateral_mm,forward_mm,quality,pixels*CS
    $E,sequence*CS
    $W,hit*CS                      wall state (deprecated: main code no longer
                                   uses it; default disabled)
    $B,seen,cx,cy,w,h,area,coverage*CS   black zone: main area center & size

STM32 commands accepted by this script:
    $C,S,0*CS       scan all configured colours
    $C,T,color*CS   track only the requested colour (1=red, 2=yellow)
    $C,P,0*CS       pause target transmission

The distance estimate is a starting point. CAMERA_FOCAL_LENGTH_PX and the
physical object widths must be calibrated on the real vehicle before relying
on millimetre coordinates.

v6.1-openmv-far (2026-09-03):
    Far-range optimisation on top of the protocol-compatible script:
    * keep the original 150 px gate for near/mid targets;
    * accept much smaller far blobs, but require fill/aspect/colour checks
      so random noise does not turn into phantom targets;
    * optional VGA mode doubles the pixels on a far block (check FPS >= 5).
    Output protocol and quality formula are unchanged.
"""

import sensor
import time
import math
from pyb import UART, LED


# Camera and communication configuration.
HIGH_RES_ENABLE = False      # True=640x480 doubles far-block pixels, lower FPS
IMAGE_WIDTH = 640 if HIGH_RES_ENABLE else 320
IMAGE_HEIGHT = 480 if HIGH_RES_ENABLE else 240
CAMERA_CENTER_X = IMAGE_WIDTH // 2
UART_BAUDRATE = 115200
UART_READ_BUFFER = 512
FRAME_PERIOD_MS = 200
MAX_TARGETS = 8

# Approximate focal length from the 70.8 degree horizontal field of view.
# Recalibrate this value with the final lens and mounting position.
# 225 px is the QVGA value; VGA keeps the same FOV so the focal doubles.
CAMERA_FOCAL_LENGTH_PX = 450.0 if HIGH_RES_ENABLE else 225.0

# Physical target widths used by the pinhole distance estimate. Change these
# after measuring the competition objects. Both default to 25 mm deliberately.
RED_OBJECT_WIDTH_MM = 25.0
YELLOW_OBJECT_WIDTH_MM = 25.0
MIN_FORWARD_MM = 60
MAX_FORWARD_MM = 2500

# 曝光控制：固定曝光降低画面亮度，避免亮光下自动曝光过曝。
# 画面还偏亮就把 FIXED_EXPOSURE_US 调小（8000 / 6000），
# 偏暗就调大（15000 / 20000）。
FIXED_EXPOSURE_ENABLE = True
FIXED_EXPOSURE_US = 10000
FIXED_GAIN_DB = 0.0


# Detection configuration.
COLOR_RED = 1
COLOR_YELLOW = 2

# 红色识别范围（在“能识别”与“少噪点”之间取折中）：
# L 下限 8：极暗的杂点不进，暗红仍可识别；
# A 下限 10：黄/灰/肤色类噪点被压掉大部分；
# B 上限 115：暖光下发橙的红仍可进，但排除高黄成分的假红。
# 若红色又变不敏感：先把 A 下限降回 8；若噪点仍多：把 A 下限升到 12~15。
RED_THRESHOLD = (8, 100, 10, 127, 0, 115)

# 黄色识别条件（比原始更严格）：
# 初筛要求 B 下限 22、A 范围 -45~45、亮度下限 15，
# 深黄（暗一点）和浅黄（淡一点）都能进，明显偏红/偏橙的排除。
YELLOW_THRESHOLD = (15, 100, -45, 45, 22, 127)

# 黄色纯度校验：候选 blob 的平均色还必须满足下面条件（可调）。
YELLOW_MIN_B_MEAN = 25.0
YELLOW_MAX_ABS_A_MEAN = 45.0

# 调试开关：打印每个黄色候选的平均 LAB，方便按实际数值调参。
DEBUG_PRINT_YELLOW = False

# Tuple fields: color_id, name, threshold, drawing_color, physical_width_mm.
COLOR_CONFIGS = (
    (COLOR_RED, "RED", RED_THRESHOLD, (255, 0, 0), RED_OBJECT_WIDTH_MM),
    (COLOR_YELLOW, "YELLOW", YELLOW_THRESHOLD, (255, 255, 0), YELLOW_OBJECT_WIDTH_MM),
)

# ---------- 远距离识别参数（v6.1-openmv-far 新增） ----------
# 原版 MIN_PIXELS=150 / MIN_AREA=150 会把远处只有十几~几十像素的物块全部
# 漏掉。现在把检测门槛分成两档：
#   近/中目标（>=MIN_PIXELS_NEAR）走原逻辑，不额外设限；
#   远目标（MIN_PIXELS_FAR~MIN_PIXELS_NEAR 之间）降低门槛进入，但用
#   填充率、长宽比和颜色纯度做二次校验，避免噪声被当成物块。
FAR_DETECT_ENABLE = True

# 近/中目标像素门槛，保持和原版一致，优先保证原有识别表现不退化。
MIN_PIXELS_NEAR = 150
MIN_AREA_NEAR = 150

# 远目标最低像素/面积：10 起跳，能识别远处小物块，又不会像 8 那样
# 把零星色点全收进来。误报多就调到 12/16，远处还漏就降到 8。
MIN_PIXELS_FAR = 10
MIN_AREA_FAR = 10

# 远目标最小宽/高（像素）：低于 2 像素的候选基本是噪点。
MIN_BLOB_SIZE_PX = 2

# 远目标长宽比上限：物块接近方形，细长反光/拖影大多超过 3 倍。
MAX_ASPECT_RATIO = 3.0

# 远目标最小填充率（0~100）：真实物块中心是实心的，填充率通常 >50%；
# 噪点/反光点形状散，填充率低。40 是较稳的折中值。
MIN_FILL_PERCENT = 40

# 远距离小红块的附加平均色校验（只作用于小于 MIN_PIXELS_NEAR 的候选）。
# 默认关闭：当前目标是“红色方块必须能识别”，不优先考虑噪点；
# 若误报变多再改为 True，并把下面数值调严。
FAR_COLOR_PURITY_CHECK = False
RED_FAR_MIN_L_MEAN = 10.0
RED_FAR_MIN_A_MEAN = 12.0
RED_FAR_MAX_B_MEAN = 95.0

# 打印每个进入二次校验的小候选，方便在 IDE 里观察远目标实际像素。
DEBUG_PRINT_FAR = False

# 打印每个红色候选的像素数和平均 LAB，方便定位“阈值没罩住”还是
# “被过滤条件拦下”。排查完可改回 False。
DEBUG_PRINT_RED = False

# 运行状态心跳灯（红灯，每秒翻转一次）：看到红灯持续闪烁就说明
# OpenMV 确实在跑这份 main.py；不需要时改成 False。
LED_HEARTBEAT_ENABLE = True
LED_HEARTBEAT_PERIOD_MS = 1000

# ---------- 防闪烁（跨帧确认）参数（v6.1-openmv-far2 新增） ----------
# 远距离低门槛会把个别噪点/反光当成目标，表现为小色块忽隐忽现。
# 本过滤器按毫秒计时，不受 OpenMV 实际帧率影响：
#   目标需在 CONFIRM_WINDOW_MS 内被检测到 MIN_DETECTIONS 次才确认；
#   已确认目标短暂消失 REPORT_HOLD_MS 内仍补发上一帧坐标（输出不闪断）；
#   消失超过 HOLD_MS 才删除轨迹。
ANTI_FLICKER_ENABLE = True
ANTI_FLICKER_MIN_DETECTIONS = 3           # 窗口内最少检测次数
ANTI_FLICKER_CONFIRM_WINDOW_MS = 800      # 确认窗口（毫秒）
ANTI_FLICKER_REPORT_HOLD_MS = 400         # 失联后补发上一帧坐标的时间
ANTI_FLICKER_HOLD_MS = 2000               # 失联超过该时间才删除轨迹
ANTI_FLICKER_CENTER_MATCH_PX = 160 if HIGH_RES_ENABLE else 80
ANTI_FLICKER_FORWARD_MATCH_RATIO = 0.35   # 前后帧前向距离允许变化 35%
ANTI_FLICKER_SIZE_MATCH_RATIO = 0.35      # 前后帧像素数比例下限，防误匹配
DEBUG_PRINT_STABILIZE = False

# 跨颜色互斥：
# 原逻辑：某色 blob 的中心落在另一色 blob 框内就丢弃，用于滤掉反光/偏色。
# 当前关闭：红色阈值放宽后，红方块可能同时落入黄/红两路初筛，互斥反而会
# 把真红块一起丢掉（表现为“红色方块无法识别”）。若之后发现红黄重复上报，
# 可先收窄 RED_THRESHOLD 再重新打开。
CROSS_COLOR_EXCLUSION = False

# ---------- 蓝色墙壁/撞墙检测（已停用，代码保留备查） ----------
# 项目主代码的避障已解决，不再需要墙体数据；墙壁识别本身也不精确，因此
# 默认关闭：不占用帧时间、不画黄框、不发 $W。若以后要恢复，把
# WALL_DETECT_ENABLE 改回 True，并对着实墙重新标定 BLUE_WALL_THRESHOLD。
WALL_DETECT_ENABLE = False
WALL_HIT_COVERAGE_PERCENT = 100.0   # 蓝色像素占整帧 ≥该值判撞墙（原 90）
WALL_BLOB_MIN_PIXELS = 20          # 小于该面积的蓝色碎片不参与统计
BLUE_WALL_THRESHOLD = (0, 100, -60, 127, -128, -1)

DEBUG_PRINT_WALL = False  # 墙数据不再使用，终端打印默认关闭


# ---------- 黑色卸货区检测（v7 新增，独立信号 $B，识别优先） ----------
# 语义：黑色卸货区作为“一块颜色目标”识别，与红/黄物块走同一套逻辑：
#   find_blobs 按用户给定的 LAB 规则筛像素 → 宽 margin 合并 → 取最大块。
#   墙数据已废弃：黑区识别完全独立，画面中黑色区域与墙/墙影重叠时不作
#   任何“区分墙体”的处理（蓝墙等非中性暗色本就不满足下面 LAB 判据）。
#   严格阈值会把整块黑区切成很多小碎片，所以扫描时把小碎片合并回一块，
#   不再做形状/统计二次把关（见下方参数）。
# 识别标志（seen 字段）：
#   1 = 识别到黑色区域   0 = 未识别到
# 串口报文：$B,seen,cx,cy,w,h,area,coverage*CS
#   seen=1 识别到；cx,cy = 主黑区中心相对画面中心的归一化坐标（-1~1，
#   右/下为正，用于导航）；w,h = 主黑区合并框宽高（像素）；area = 合并框
#   面积 w×h；coverage = 合并框占整帧面积比例（%）。OpenMV 不判“到达”，
#   阈值由 STM32 定。
#   该报文与红/黄目标帧、$W 撞墙报文相互独立。
BLACK_ZONE_DETECT_ENABLE = True    # 黑色区域检测总开关

# 黑色判据（用户定稿）：只把 L<20 且 |A|<10、|B|<10 的像素识别为黑色。
# find_blobs 阈值格式为 (L_min, L_max, A_min, A_max, B_min, B_max)，所以
# L 取 0..19，A/B 取 -9..9。强偏色的深色仍进不了候选。
BLACK_ZONE_THRESHOLD = (0, 19, -9, 9, -9, 9)

# 扫描与合并（当前策略：不做形状把关，直接靠“低门槛+宽 margin 合并”拼整块）：
#   黑区整块平均 LAB 虽达标，但逐像素会被噪点/纹理切成很多小碎点；若扫描
#   门槛太高，小碎点在合并前就被丢弃，拼不回整块。当前门槛设为 20、合并
#   间距设为 5（约跨 10px 间隙）：噪点压制优先，只合并几乎贴在一起的碎片。
#   若出现孤立闪烁小框，再单独把 BLACK_SCAN_MIN_PIXELS 调高试
#   （同时中心可能又拼不回来）。
BLACK_SCAN_MIN_PIXELS = 20         # find_blobs 扫描门槛（低于该值的碎点不参与合并）
BLACK_ZONE_MERGE = True            # True=合并间距内的碎片为整块
BLACK_ZONE_MERGE_MARGIN_PX = 5     # 合并间距（像素）：两碎片外接框相距 ≤2×margin
                                   # 即会合并。5=可跨过约 10px 宽的孔洞/噪声缝隙

# —— 形状/统计把关（当前全部取消，函数保留备查）——
# 下面这些参数原来在 _black_blob_passes 里对候选做“实心、不细长、真黑”的
# 二次过滤；目前 update_black_zone 不再调用它，全部不生效。
BLACK_ZONE_MIN_PIXELS = 30
BLACK_MIN_W_PX = 3
BLACK_MIN_H_PX = 3
BLACK_MIN_FILL_PERCENT = 40
BLACK_MAX_ASPECT_RATIO = 5.0

# —— 整块二次闸门（默认全关）——
# 黑色已由上面的逐像素 LAB 阈值精确定义（L<20、|A|、|B|<10），凡是能通过
# 该阈值的候选必然满足这里的分位条件，因此不再需要整块二次把关。开关
# 保留：若以后把扫描阈值放宽导致误报，可再单独打开做整块分位/均值校验。
BLACK_REQUIRE_HIST_SHAPE = False
BLACK_MEDIAN_L_MAX = 28.0    # L 中位数上限（旧“真黑直方图形状”闸门）
BLACK_UQ_L_MAX = 40.0        # L 上四分位上限
BLACK_AB_MEDIAN_MAX = 15.0   # |A|/|B| 中位数上限
BLACK_AB_UQ_MAX = 25.0       # |A|/|B| 上四分位上限

# 整块平均亮度/平均色度把关（默认关）。
BLACK_REQUIRE_MEAN_CHECK = False
BLACK_MEAN_L_MAX = 20.0      # 整块平均亮度上限
BLACK_MEAN_AB_MAX = 30.0     # |A|/|B| 平均值上限

# 可选对比度闸门（默认关）：需要时要求黑区明显暗于周围地面。
BLACK_REQUIRE_CONTRAST = False
BLACK_CONTRAST_MIN_L_DIFF = 12.0
BLACK_CONTRAST_MARGIN_PX = 10

# 黑色区域可能在画面任何高度出现（远处时更高），与红/黄物块一致采用整帧
# 扫描。墙已不再参与决策，无需为墙保留任何区域限制；若日后确因上方其他
# 深色物体误报，可再把该值调大到 0.20~0.35 仅扫描画面下方。
BLACK_ZONE_ROI_TOP_RATIO = 0.0    # 0 = 全帧；>0 时仅扫描该行以下的条带

# IDE 显示开关：OpenMV IDE 画面上用白框/十字框住黑区、绿字显示面积占比，
# 方便直接观察识别效果；本脚本不向终端打印任何黑色区调试信息。
DEBUG_DRAW_BLACK = True


# Runtime state.
MODE_SCAN = "S"
MODE_TRACK = "T"
MODE_PAUSE = "P"
vision_mode = MODE_SCAN
tracked_color = 0
rx_line = ""
frame_sequence = 0
last_frame_tick = 0
status_led = None
led_heartbeat_state = False
led_heartbeat_tick = 0
vision_tracks = []
wall_hit = False
wall_debug_tick = 0

# 黑色区域运行时状态（不要在别处改动，只由 update_black_zone 更新）。
black_seen = False          # 本帧是否识别到黑色区域
black_coverage = 0.0        # 主黑区合并框面积占整帧画面的比例（%）
black_boxes = []            # 本帧通过过滤的黑色区域矩形，供 draw 使用
black_area = 0              # 主黑区合并框面积 w×h（像素），供串口上报
black_main = None           # 主黑区 (x, y, w, h)（合并框面积最大），供串口/显示


def checksum_xor(payload):
    value = 0
    for character in payload:
        value ^= ord(character)
    return value


def encode_packet(payload):
    return "$%s*%02X\r\n" % (payload, checksum_xor(payload))


def decode_packet(line):
    line = line.strip()
    if len(line) < 6 or line[0] != "$":
        return None

    separator = line.rfind("*")
    if separator < 2 or (separator + 2) >= len(line):
        return None

    payload = line[1:separator]
    try:
        received_checksum = int(line[separator + 1:separator + 3], 16)
    except ValueError:
        return None

    if checksum_xor(payload) != received_checksum:
        return None
    return payload.split(",")


def handle_control_packet(fields):
    global vision_mode, tracked_color, vision_tracks

    if len(fields) != 3 or fields[0] != "C":
        return

    mode = fields[1]
    try:
        colour = int(fields[2])
    except ValueError:
        colour = 0

    mode_changed = False
    if mode == MODE_SCAN:
        if vision_mode != MODE_SCAN:
            vision_mode = MODE_SCAN
            tracked_color = 0
            mode_changed = True
    elif mode == MODE_TRACK and colour in (COLOR_RED, COLOR_YELLOW):
        if vision_mode != MODE_TRACK or tracked_color != colour:
            vision_mode = MODE_TRACK
            tracked_color = colour
            mode_changed = True
    elif mode == MODE_PAUSE:
        if vision_mode != MODE_PAUSE:
            vision_mode = MODE_PAUSE
            tracked_color = 0
            mode_changed = True

    # 模式/颜色切换后旧轨迹不再有效，避免跨模式串目标。
    if mode_changed:
        vision_tracks = []


def service_uart_rx(uart):
    """Consume STM32 command packets without blocking image processing."""
    global rx_line

    available = uart.any()
    if not available:
        return

    data = uart.read(available)
    if data is None:
        return

    for value in data:
        character = chr(value) if isinstance(value, int) else value

        if character == "\n":
            fields = decode_packet(rx_line)
            if fields is not None:
                handle_control_packet(fields)
            rx_line = ""
        elif character != "\r":
            if len(rx_line) < 96:
                rx_line += character
            else:
                rx_line = ""


def estimate_coordinates(blob, object_width_mm):
    """Return lateral-right and forward distance in millimetres."""
    apparent_width = max(1, blob.w())
    forward_mm = object_width_mm * CAMERA_FOCAL_LENGTH_PX / apparent_width
    forward_mm = max(MIN_FORWARD_MM, min(MAX_FORWARD_MM, forward_mm))

    error_x = blob.cx() - CAMERA_CENTER_X
    lateral_mm = error_x * forward_mm / CAMERA_FOCAL_LENGTH_PX
    return int(round(lateral_mm)), int(round(forward_mm))


def calculate_quality(blob):
    bounding_area = max(1, blob.w() * blob.h())
    fill_percent = min(100, (blob.pixels() * 100) // bounding_area)
    size_percent = min(100, blob.pixels() // 8)
    return int((fill_percent * 3 + size_percent * 2) // 5)


def yellow_passes_purity(stats):
    """黄色纯度校验：平均色 B 要高、|A| 要小，且 B 明显大于 |A|。"""
    a_mean = stats.a_mean()
    b_mean = stats.b_mean()

    if b_mean < YELLOW_MIN_B_MEAN:
        return False
    if abs(a_mean) > YELLOW_MAX_ABS_A_MEAN:
        return False
    # 橙色防护：真黄的 B 明显大于 |A|（1.2 倍），橙色两者接近。
    if b_mean <= abs(a_mean) * 1.2:
        return False
    return True


def red_passes_far(stats):
    """远处小红块的附加校验：不能太暗，A 要够高，B 不能压过红。"""
    if stats.l_mean() < RED_FAR_MIN_L_MEAN:
        return False
    if stats.a_mean() < RED_FAR_MIN_A_MEAN:
        return False
    if stats.b_mean() > RED_FAR_MAX_B_MEAN:
        return False
    return True


def passes_minimum_shape(blob):
    """近/中目标沿用原门槛；小目标必须有实心、不细长的形状。"""
    pixels = blob.pixels()
    if pixels >= MIN_PIXELS_NEAR:
        return True
    if not FAR_DETECT_ENABLE or pixels < MIN_PIXELS_FAR:
        return False

    width = blob.w()
    height = blob.h()
    if width < MIN_BLOB_SIZE_PX or height < MIN_BLOB_SIZE_PX:
        return False

    bounding_area = width * height
    if bounding_area <= 0:
        return False
    fill_percent = (pixels * 100) // bounding_area
    if fill_percent < MIN_FILL_PERCENT:
        return False

    if width >= height:
        aspect = float(width) / float(height) if height else 999.0
    else:
        aspect = float(height) / float(width) if width else 999.0
    if aspect > MAX_ASPECT_RATIO:
        return False
    return True


def centroid_inside_any(blob, other_blobs):
    """blob 的中心是否落在任一 other_blob 的矩形框内。"""
    cx = blob.cx()
    cy = blob.cy()

    for other in other_blobs:
        if (other.x() <= cx <= other.x() + other.w() and
                other.y() <= cy <= other.y() + other.h()):
            return True
    return False


def find_candidates(img):
    """单帧候选目标（尚未做跨帧确认），每个元素是一个 dict。"""
    blobs_by_color = {}
    candidates = []

    for color_id, name, threshold, draw_color, object_width_mm in COLOR_CONFIGS:
        if vision_mode == MODE_TRACK and color_id != tracked_color:
            continue

        blobs_by_color[color_id] = (
            name,
            draw_color,
            object_width_mm,
            img.find_blobs(
                [threshold],
                # 用远目标低门槛扫描，之后用 passes_minimum_shape 分级过滤。
                pixels_threshold=MIN_PIXELS_FAR,
                area_threshold=MIN_AREA_FAR,
                # Keep adjacent blocks separate so the planner receives
                # individual targets. Increase thresholds instead of
                # merging noisy regions.
                merge=False,
            ),
        )

    for color_id, (name, draw_color, object_width_mm, blobs) in \
            blobs_by_color.items():
        other_blobs = []
        for other_id, (_, _, _, other_list) in blobs_by_color.items():
            if other_id != color_id:
                other_blobs.extend(other_list)

        for blob in blobs:
            if not passes_minimum_shape(blob):
                continue

            if DEBUG_PRINT_FAR and blob.pixels() < MIN_PIXELS_NEAR:
                print("FAR %s px=%d box=%dx%d fill=%d%%"
                      % (name, blob.pixels(), blob.w(), blob.h(),
                         (blob.pixels() * 100) // max(1, blob.w() * blob.h())))

            # 新增：中心落在另一色框内的高光/偏色区域直接丢弃。
            if CROSS_COLOR_EXCLUSION and centroid_inside_any(blob, other_blobs):
                continue

            # 黄色额外做纯度校验，限制“不够黄”的目标进入。
            if color_id == COLOR_YELLOW:
                stats = img.get_statistics(roi=blob.rect())
                if DEBUG_PRINT_YELLOW:
                    print("YELLOW CAND LAB=(%d,%d,%d) px=%d"
                          % (stats.l_mean(), stats.a_mean(),
                             stats.b_mean(), blob.pixels()))
                if not yellow_passes_purity(stats):
                    continue
            elif color_id == COLOR_RED:
                # 打印红色候选的平均 LAB，便于在 IDE 里核对阈值范围。
                stats = img.get_statistics(roi=blob.rect())
                if DEBUG_PRINT_RED:
                    print("RED CAND px=%d LAB=(%d,%d,%d)"
                          % (blob.pixels(), stats.l_mean(),
                             stats.a_mean(), stats.b_mean()))
                if (FAR_COLOR_PURITY_CHECK and
                        blob.pixels() < MIN_PIXELS_NEAR and
                        not red_passes_far(stats)):
                    continue

            lateral_mm, forward_mm = estimate_coordinates(blob, object_width_mm)
            candidates.append({
                'color': color_id,
                'name': name,
                'draw': draw_color,
                'x': blob.x(),
                'y': blob.y(),
                'w': blob.w(),
                'h': blob.h(),
                'cx': blob.cx(),
                'cy': blob.cy(),
                'pixels': blob.pixels(),
                'lateral': lateral_mm,
                'forward': forward_mm,
                'quality': calculate_quality(blob),
            })

    # Prefer reliable, large observations when more than MAX_TARGETS are seen.
    candidates.sort(
        key=lambda candidate: candidate['quality'] * 100000 + candidate['pixels'],
        reverse=True)
    return candidates[:MAX_TARGETS]


def _find_best_track(candidate, tracks):
    """在当前轨迹中找与候选同色、位置/距离/大小相近的最佳轨迹。"""
    best = None
    best_error = None

    for track in tracks:
        if track['used']:
            continue
        if track['color'] != candidate['color']:
            continue

        # 纯旋转扫描时前向距离基本不变，用它做第一道匹配门槛。
        fwd_old = float(track['forward'])
        fwd_new = float(candidate['forward'])
        if fwd_old > 0.0 and fwd_new > 0.0:
            larger = fwd_old if fwd_old > fwd_new else fwd_new
            if (abs(fwd_old - fwd_new) / larger) > ANTI_FLICKER_FORWARD_MATCH_RATIO:
                continue

        size_old = track['pixels']
        size_new = candidate['pixels']
        if size_old <= 0 or size_new <= 0:
            continue
        if size_old < size_new:
            size_ratio = float(size_old) / size_new
        else:
            size_ratio = float(size_new) / size_old
        if size_ratio < ANTI_FLICKER_SIZE_MATCH_RATIO:
            continue

        # 用车头旋转/平移的速度外推下一帧位置，再按像素距离匹配。
        pred_x = track['cx'] + (track['cx'] - track['prev_cx'])
        pred_y = track['cy'] + (track['cy'] - track['prev_cy'])
        dx = candidate['cx'] - pred_x
        dy = candidate['cy'] - pred_y
        error = math.sqrt(dx * dx + dy * dy)
        if (error <= ANTI_FLICKER_CENTER_MATCH_PX and
                (best_error is None or error < best_error)):
            best_error = error
            best = track
    return best


def stabilize_targets(candidates):
    """跨帧确认/保持：滤掉单帧噪点，抑制小目标闪烁（按毫秒计时）。"""
    global vision_tracks

    if not ANTI_FLICKER_ENABLE:
        return candidates[:MAX_TARGETS]

    now_ms = time.ticks_ms()
    for track in vision_tracks:
        track['used'] = 0

    # 第一遍：把本帧候选匹配到已有轨迹或建立新轨迹。
    for candidate in candidates:
        track = _find_best_track(candidate, vision_tracks)
        if track is None:
            vision_tracks.append({
                'color': candidate['color'],
                'cx': candidate['cx'],
                'cy': candidate['cy'],
                'prev_cx': candidate['cx'],
                'prev_cy': candidate['cy'],
                'pixels': candidate['pixels'],
                'forward': candidate['forward'],
                'hits': 1,
                'last_seen_ms': now_ms,
                'used': 1,
                'last': candidate,
            })
        else:
            track['used'] = 1
            gap_ms = time.ticks_diff(now_ms, track['last_seen_ms'])
            if gap_ms > ANTI_FLICKER_CONFIRM_WINDOW_MS:
                track['hits'] = 1
            else:
                track['hits'] = min(track['hits'] + 1,
                                    ANTI_FLICKER_MIN_DETECTIONS + 2)
            track['prev_cx'] = track['cx']
            track['prev_cy'] = track['cy']
            track['cx'] = candidate['cx']
            track['cy'] = candidate['cy']
            track['pixels'] = candidate['pixels']
            track['forward'] = candidate['forward']
            track['last_seen_ms'] = now_ms
            track['last'] = candidate

    # 第二遍：按失联毫秒数决定哪些轨迹可上报/保留。
    reported = []
    kept = []
    for track in vision_tracks:
        if track['used'] == 0:
            miss_ms = time.ticks_diff(now_ms, track['last_seen_ms'])
            if track['hits'] < ANTI_FLICKER_MIN_DETECTIONS:
                # 还没确认就长时间没再出现：说明是单帧噪点，直接丢弃。
                if miss_ms > ANTI_FLICKER_CONFIRM_WINDOW_MS:
                    continue
            elif miss_ms > ANTI_FLICKER_HOLD_MS:
                if DEBUG_PRINT_STABILIZE:
                    print("STABLE LOST color=%d px=%d"
                          % (track['color'], track['pixels']))
                continue
            kept.append(track)
            if (track['hits'] >= ANTI_FLICKER_MIN_DETECTIONS and
                    miss_ms <= ANTI_FLICKER_REPORT_HOLD_MS):
                last = track['last']
                if last is not None:
                    reported.append(last)
        else:
            kept.append(track)
            if track['hits'] >= ANTI_FLICKER_MIN_DETECTIONS:
                last = track['last']
                if last is not None:
                    reported.append(last)

    vision_tracks = kept
    reported.sort(
        key=lambda target: target['quality'] * 100000 + target['pixels'],
        reverse=True)
    return reported[:MAX_TARGETS]


def draw_targets(img, targets):
    for target in targets:
        x = target['x']
        y = target['y']
        color = target['draw']
        img.draw_rectangle((x, y, target['w'], target['h']),
                           color=color, thickness=2)
        img.draw_cross(target['cx'], target['cy'], color=color, size=10)
        label = "%s %d/%d Q%d" % (target['name'], target['lateral'],
                                  target['forward'], target['quality'])
        img.draw_string(x, max(0, y - 12), label, color=color)


def send_target_frame(uart, targets):
    global frame_sequence

    frame_sequence = (frame_sequence + 1) & 0xFFFF
    uart.write(encode_packet("F,%d,%d" % (frame_sequence, len(targets))))

    for index, target in enumerate(targets):
        payload = "T,%d,%d,%d,%d,%d,%d,%d" % (
            frame_sequence,
            index,
            target['color'],
            target['lateral'],
            target['forward'],
            target['quality'],
            target['pixels'],
        )
        uart.write(encode_packet(payload))

    uart.write(encode_packet("E,%d" % frame_sequence))


def update_wall_hit(img):
    """统计整帧蓝色像素占比（默认停用）。

    IDE 画面已不再绘制任何墙体框/文字（相关显示代码已删除），
    $W 仅在 WALL_DETECT_ENABLE=True 时由 send_wall_state 发送。
    """
    global wall_hit, wall_debug_tick

    if not WALL_DETECT_ENABLE:
        wall_hit = False
        return False

    blobs = img.find_blobs(
        [BLUE_WALL_THRESHOLD],
        pixels_threshold=WALL_BLOB_MIN_PIXELS,
        area_threshold=WALL_BLOB_MIN_PIXELS,
        merge=True,
    )
    matched_pixels = 0
    for blob in blobs:
        matched_pixels += blob.pixels()

    frame_pixels = IMAGE_WIDTH * IMAGE_HEIGHT
    coverage = ((matched_pixels * 100.0) / frame_pixels
                if frame_pixels > 0 else 0.0)
    wall_hit = coverage >= WALL_HIT_COVERAGE_PERCENT

    # 调试输出：每 200ms 打印一次占比（仅终端，默认关，不画画面）。
    now_ms = time.ticks_ms()
    if DEBUG_PRINT_WALL and time.ticks_diff(now_ms, wall_debug_tick) >= 200:
        wall_debug_tick = now_ms
        print("WALL %s blue=%.1f%% blobs=%d"
              % ("HIT" if wall_hit else "CLEAR",
                 coverage, len(blobs)))
    return wall_hit


def send_wall_state(uart):
    """发送 $W,1（撞墙）或 $W,0（脱离墙面）；墙数据停用时不再发送。"""
    if not WALL_DETECT_ENABLE:
        return
    uart.write(encode_packet("W,%d" % (1 if wall_hit else 0)))


def black_zone_roi_rect():
    """黑色区域检测区：画面下方 BLACK_ZONE_ROI_TOP_RATIO 以下的全宽条带。"""
    top = int(round(IMAGE_HEIGHT * BLACK_ZONE_ROI_TOP_RATIO))
    if top >= IMAGE_HEIGHT:
        top = IMAGE_HEIGHT - 1
    return (0, top, IMAGE_WIDTH, IMAGE_HEIGHT - top)


def _black_blob_mean_ok(stats):
    """候选黑区整块平均 LAB 是否够暗且接近中性（过滤墙影/偏色暗区）。"""
    if not BLACK_REQUIRE_MEAN_CHECK:
        return True
    if stats is None:
        return True
    return (stats.l_mean() <= BLACK_MEAN_L_MAX and
            abs(stats.a_mean()) <= BLACK_MEAN_AB_MAX and
            abs(stats.b_mean()) <= BLACK_MEAN_AB_MAX)


def _black_blob_hist_ok(stats):
    """“真黑直方图形状”闸门：模拟 RGB 三通道“最左端尖峰、右侧无内容”。

    OpenMV 的 get_statistics() 对 RGB565 只给 LAB 分位统计，但这个形状
    在 LAB 里等价于：
      * L 中位数很低     -> RGB 主峰贴在最左端；
      * L 上四分位也低   -> 直方图右侧没有内容（无亮尾/渐变灰影）；
      * |A|、|B| 中位/上四分位都接近 0 -> R≈G≈B，颜色中性（深蓝墙排除）。
    stats 为 None 时保守放行（阈值本身仍把住“很暗”这一关）。
    """
    if not BLACK_REQUIRE_HIST_SHAPE:
        return True
    if stats is None:
        return True
    if stats.l_median() > BLACK_MEDIAN_L_MAX:
        return False
    if stats.l_uq() > BLACK_UQ_L_MAX:
        return False
    if (abs(stats.a_median()) > BLACK_AB_MEDIAN_MAX or
            abs(stats.b_median()) > BLACK_AB_MEDIAN_MAX):
        return False
    if (abs(stats.a_uq()) > BLACK_AB_UQ_MAX or
            abs(stats.b_uq()) > BLACK_AB_UQ_MAX):
        return False
    return True


def _black_blob_contrast_ok(img, blob, blob_stats):
    """候选黑区是否明显暗于周围地面（过滤墙影/大面积灰暗区）。"""
    if not BLACK_REQUIRE_CONTRAST:
        return True

    margin = BLACK_CONTRAST_MARGIN_PX
    x0 = max(0, blob.x() - margin)
    y0 = max(0, blob.y() - margin)
    x1 = min(IMAGE_WIDTH, blob.x() + blob.w() + margin)
    y1 = min(IMAGE_HEIGHT, blob.y() + blob.h() + margin)

    blob_area = max(1, blob.pixels())
    outer_area = max(1, (x1 - x0) * (y1 - y0))
    surround_area = outer_area - blob_area

    # 采样环太窄（黑区几乎占满扩展框，比如贴近画面边缘）时无法可靠取周围
    # 亮度，跳过对比度校验，交给均值/颜色校验把关。
    if surround_area < blob_area * 0.3:
        return True

    outer_stats = img.get_statistics(roi=(x0, y0, x1 - x0, y1 - y0))
    if blob_stats is None or outer_stats is None:
        return True

    l_blob = float(blob_stats.l_mean())
    l_outer = float(outer_stats.l_mean())
    l_surround = ((l_outer * outer_area - l_blob * blob_area) /
                  max(1.0, surround_area))
    return (l_surround - l_blob) >= BLACK_CONTRAST_MIN_L_DIFF


def _black_blob_passes(img, blob, stats=None):
    """黑色候选过滤：先形状把关，再按 LAB 分位确认“真黑”（仿物块）。"""
    pixels = blob.pixels()
    if pixels < BLACK_SCAN_MIN_PIXELS:
        return False

    width = blob.w()
    height = blob.h()
    if width >= height:
        aspect = float(width) / float(height) if height else 999.0
    else:
        aspect = float(height) / float(width) if width else 999.0
    if aspect > BLACK_MAX_ASPECT_RATIO:
        return False

    # 填充率把关：纯色黑区的外接框几乎全被暗像素填满（fill 高）；
    # 稀疏噪声/墙影边缘 fill 低，予以排除。小块要求更严。
    if width < BLACK_MIN_W_PX or height < BLACK_MIN_H_PX:
        return False
    bounding_area = width * height
    if bounding_area <= 0:
        return False
    fill_percent = (pixels * 100) // bounding_area
    if pixels < BLACK_ZONE_MIN_PIXELS:
        if fill_percent < BLACK_MIN_FILL_PERCENT:
            return False
    elif fill_percent < 15:
        return False

    # 颜色分位闸门：默认开“直方图形状”这一道，均值/对比度按需补充。
    if (BLACK_REQUIRE_HIST_SHAPE or BLACK_REQUIRE_MEAN_CHECK or
            BLACK_REQUIRE_CONTRAST):
        if stats is None:
            stats = img.get_statistics(roi=blob.rect())
        if not _black_blob_hist_ok(stats):
            return False
        if not _black_blob_mean_ok(stats):
            return False
        return _black_blob_contrast_ok(img, blob, stats)
    return True


def update_black_zone(img):
    """识别黑色区域并计算面积占比（独立于红/黄目标与 $W 墙检测）。

    只负责：找到黑色区域、给出 black_seen 与 black_coverage；
    不做任何“到达/触发”判断，阈值由 STM32 自行决定。
    """
    global black_seen, black_coverage
    global black_boxes, black_area, black_main

    if not BLACK_ZONE_DETECT_ENABLE:
        black_seen = False
        black_coverage = 0.0
        black_boxes = []
        black_area = 0
        black_main = None
        return

    roi = black_zone_roi_rect()
    blobs = img.find_blobs(
        [BLACK_ZONE_THRESHOLD],
        roi=roi,
        pixels_threshold=BLACK_SCAN_MIN_PIXELS,
        area_threshold=BLACK_SCAN_MIN_PIXELS,
        merge=BLACK_ZONE_MERGE,
        margin=BLACK_ZONE_MERGE_MARGIN_PX,
    )
    # 形状/统计把关已取消：合并后的候选全部保留，取面积最大的作为主黑区。
    black_boxes = [(blob.x(), blob.y(), blob.w(), blob.h())
                   for blob in blobs]
    black_area = 0

    # black_main 取“合并后外接框面积 w×h”最大的主黑区；coverage/area
    # 均按合并框面积计算（不再是严格黑色像素数）。
    black_main = None
    main_box_area = 0
    for blob in blobs:
        box_area = blob.w() * blob.h()
        if box_area > main_box_area:
            main_box_area = box_area
            black_main = (blob.x(), blob.y(), blob.w(), blob.h())
    black_area = main_box_area

    roi_area = max(1, roi[2] * roi[3])
    black_coverage = min(100.0, main_box_area * 100.0 / roi_area)
    black_seen = len(blobs) > 0


def draw_black_zone(img):
    """IDE 显示：黑色区域用白色边框框住（类似物块），文字为绿色占比。"""
    if not BLACK_ZONE_DETECT_ENABLE or not DEBUG_DRAW_BLACK:
        return

    if not black_boxes:
        return

    # 每个通过识别的黑色区域都用白色粗框标出（同物块框的视觉效果）；
    # 对面积最大的那个再叠加白色中心十字，方便看偏移。
    largest = black_boxes[0]
    largest_area = black_boxes[0][2] * black_boxes[0][3]
    for box in black_boxes:
        img.draw_rectangle(box, color=(255, 255, 255), thickness=2)
        area = box[2] * box[3]
        if area > largest_area:
            largest_area = area
            largest = box
    img.draw_cross(largest[0] + largest[2] // 2,
                   largest[1] + largest[3] // 2,
                   color=(255, 255, 255), size=10)

    # 文字只显示黑色区域占比（绿色字体，叠在识别框上缘外侧避开白框；
    # 保留 1 位小数，避免远处小黑区 0.3% 被显示成 0% 造成“没识别到”）。
    img.draw_string(
        largest[0], max(1, largest[1] - 12),
        "BLACK %.1f%%" % black_coverage,
        color=(0, 255, 0))


def send_black_zone_state(uart):
    """发送 $B,seen,cx,cy,w,h,area,coverage*CS。

    cx/cy：主黑区中心相对画面中心的归一化坐标（-1~1，右/下为正）；
    w/h/area：主黑区合并框宽高与面积（w×h）；coverage：合并框占整帧百分比。
    """
    if black_main is None:
        uart.write(encode_packet("B,0,0,0,0,0,0,0.0"))
        return

    x, y, w, h = black_main
    cx = (x + w / 2.0 - CAMERA_CENTER_X) / (IMAGE_WIDTH / 2.0)
    cy = (y + h / 2.0 - IMAGE_HEIGHT / 2.0) / (IMAGE_HEIGHT / 2.0)
    area = black_area
    uart.write(encode_packet(
        "B,%d,%.3f,%.3f,%d,%d,%d,%.1f"
        % (1 if black_seen else 0, cx, cy, w, h, area, black_coverage)))


sensor.reset()
sensor.set_pixformat(sensor.RGB565)
if HIGH_RES_ENABLE:
    sensor.set_framesize(sensor.VGA)
else:
    sensor.set_framesize(sensor.QVGA)

# 固定曝光降低画面亮度（自动曝光在亮光下容易过曝）。
if FIXED_EXPOSURE_ENABLE:
    try:
        sensor.set_auto_exposure(False)
        sensor.set_exposure_microseconds(FIXED_EXPOSURE_US)
    except Exception:
        print("WARN: cannot set fixed exposure, keep auto exposure")
else:
    sensor.set_auto_exposure(True)

sensor.skip_frames(time=1000)
sensor.set_auto_whitebal(False)
try:
    sensor.set_auto_gain(False)
    sensor.set_gain_db(FIXED_GAIN_DB)
except Exception:
    sensor.set_auto_gain(False)
sensor.skip_frames(time=500)

uart = UART(
    3,
    UART_BAUDRATE,
    bits=8,
    parity=None,
    stop=1,
    timeout=20,
    timeout_char=2,
    read_buf_len=UART_READ_BUFFER,
)

# 上电自检提示：启动时红灯亮 0.3 秒；运行期间红灯按心跳周期持续闪烁。
try:
    status_led = LED(1)
    status_led.on()
    time.sleep_ms(300)
    status_led.off()
except Exception:
    status_led = None
print("visual.py BOOT OK: UART3 @%d, frames every %d ms"
      % (UART_BAUDRATE, FRAME_PERIOD_MS))

if status_led is not None:
    led_heartbeat_state = False
    led_heartbeat_tick = time.ticks_ms()

clock = time.clock()

while True:
    clock.tick()
    service_uart_rx(uart)

    img = sensor.snapshot()
    img.gaussian(1)

    update_wall_hit(img)
    update_black_zone(img)

    if vision_mode == MODE_PAUSE:
        vision_tracks = []
        targets = []
    else:
        targets = stabilize_targets(find_candidates(img))
    draw_targets(img, targets)
    draw_black_zone(img)

    now = time.ticks_ms()
    if (LED_HEARTBEAT_ENABLE and status_led is not None and
            time.ticks_diff(now, led_heartbeat_tick) >= LED_HEARTBEAT_PERIOD_MS):
        led_heartbeat_tick = now
        if led_heartbeat_state:
            status_led.off()
            led_heartbeat_state = False
        else:
            status_led.on()
            led_heartbeat_state = True
    if vision_mode != MODE_PAUSE and time.ticks_diff(now, last_frame_tick) >= FRAME_PERIOD_MS:
        send_target_frame(uart, targets)
        send_wall_state(uart)
        send_black_zone_state(uart)
        last_frame_tick = now
        print(
            "VISION mode=%s frame=%d targets=%d fps=%.1f"
            % (vision_mode, frame_sequence, len(targets), clock.fps())
        )
