CC ?= cc
PYTHON ?= python3
CFLAGS ?= -std=c11 -Wall -Wextra -Werror -O2

BUILD_DIR := build
TEST_BIN := $(BUILD_DIR)/host_test
CORE_INC := ../Core/Inc
CORE_SRC := ../Core/Src
SOURCES := $(CORE_SRC)/vision_protocol.c \
	$(CORE_SRC)/target_map.c \
	$(CORE_SRC)/path_planner.c \
	$(CORE_SRC)/mission_extension.c \
	$(CORE_SRC)/combat_strategy.c \
	host_test.c

.PHONY: all test clean

all: test

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

$(TEST_BIN): $(BUILD_DIR) $(SOURCES)
	$(CC) $(CFLAGS) -I$(CORE_INC) $(SOURCES) -o $(TEST_BIN)

$(BUILD_DIR)/brush_feedback_test: brush_feedback_test.c $(CORE_SRC)/brush_feedback.c $(CORE_INC)/brush_feedback.h $(CORE_INC)/brush_feedback_config.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(CORE_INC) brush_feedback_test.c $(CORE_SRC)/brush_feedback.c -o $@

$(BUILD_DIR)/brush_current_test: brush_feedback_test.c $(CORE_SRC)/brush_feedback.c $(CORE_INC)/brush_feedback.h $(CORE_INC)/brush_feedback_config.h | $(BUILD_DIR)
	$(CC) $(CFLAGS) -I$(CORE_INC) -DBRUSH_CURRENT_ENABLE=1 -DBRUSH_CURRENT_ZERO_MV=1650 -DBRUSH_CURRENT_MV_PER_AMP=100 -DBRUSH_CURRENT_STALL_MA=1000 brush_feedback_test.c $(CORE_SRC)/brush_feedback.c -o $@

test: $(TEST_BIN) $(BUILD_DIR)/brush_feedback_test $(BUILD_DIR)/brush_current_test
	./$(TEST_BIN)
	./$(BUILD_DIR)/brush_feedback_test
	./$(BUILD_DIR)/brush_current_test
	$(PYTHON) control_regression.py

clean:
	rm -rf $(BUILD_DIR)
