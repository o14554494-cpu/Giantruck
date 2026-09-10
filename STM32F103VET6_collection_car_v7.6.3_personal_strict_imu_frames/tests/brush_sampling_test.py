"""Check the actual hardware reader against all IDR states; not an IRQ timing test."""
from pathlib import Path
import os
import re
import shlex
import subprocess
import tempfile

root = Path(__file__).resolve().parents[1]
source = (root / 'Core/Src/brush_feedback_hw.c').read_text()
reader = re.search(r'static uint8_t BrushEncoder_ReadAB\(void\)\s*\{.*?\n\}', source, re.S).group()
assert reader.count('->IDR') == 1 and 'HAL_GPIO_ReadPin' not in reader
program = r'''
#include <assert.h>
#include <stdint.h>
typedef struct { volatile uint32_t IDR; } Port;
static Port port;
#define GPIOB (&port)
#define GPIO_PIN_12 (1U<<12)
#define GPIO_PIN_15 (1U<<15)
#include "brush_feedback_config.h"
''' + reader + r'''
int main(void) {
  unsigned state;
  for(state=0;state<4;state++) {
    port.IDR = (state&2 ? GPIO_PIN_12:0) | (state&1 ? GPIO_PIN_15:0);
    assert(BrushEncoder_ReadAB()==state);
    port.IDR |= 0x6FFFU; /* unrelated GPIOB bits */
    assert(BrushEncoder_ReadAB()==state);
  }
  return 0;
}
'''
with tempfile.TemporaryDirectory(prefix='brush-sampling-') as tmp:
    path = Path(tmp)
    (path/'test.c').write_text(program)
    subprocess.run(shlex.split(os.environ.get('CC','cc')) + ['-std=c11','-Wall','-Wextra','-Werror',
        '-I'+str(root/'Core/Inc'),str(path/'test.c'),'-o',str(path/'test')], check=True)
    subprocess.run([str(path/'test')], check=True)
print('PASS: actual A/B reader uses one IDR read, correct phase mapping, ignores unrelated pins')
