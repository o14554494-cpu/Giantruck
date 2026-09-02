CC ?= cc
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

test: $(TEST_BIN)
	./$(TEST_BIN)

clean:
	rm -rf $(BUILD_DIR)
