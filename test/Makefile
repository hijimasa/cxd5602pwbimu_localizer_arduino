# Host-side unit tests for the localizer algorithm.
#   make        build and run
#   make build  build only
#   make clean
# Optional: make run ARGS=<substring> to run a subset of tests.

CXX ?= g++
CC ?= gcc
SRC_DIR := ../SubCore1/src
BUILD := out

CXXFLAGS := -std=c++17 -O2 -g -Wall -Wextra -Wno-missing-field-initializers -I$(SRC_DIR) -I.. -I.
CFLAGS := -std=c11 -O2 -g -Wall -Wextra -I$(SRC_DIR)/Fusion

ALGO_SRCS := $(wildcard $(SRC_DIR)/localizer/*.cpp)
FUSION_SRCS := $(wildcard $(SRC_DIR)/Fusion/*.c)
TEST_SRCS := $(wildcard test_*.cpp)

ALGO_OBJS := $(patsubst $(SRC_DIR)/localizer/%.cpp,$(BUILD)/localizer_%.o,$(ALGO_SRCS))
FUSION_OBJS := $(patsubst $(SRC_DIR)/Fusion/%.c,$(BUILD)/fusion_%.o,$(FUSION_SRCS))
TEST_OBJS := $(patsubst %.cpp,$(BUILD)/%.o,$(TEST_SRCS))

TARGET := $(BUILD)/run_tests

.PHONY: all build run clean
all: run

build: $(TARGET)

run: $(TARGET)
	./$(TARGET) $(ARGS)

$(TARGET): $(ALGO_OBJS) $(FUSION_OBJS) $(TEST_OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^ -lm

$(BUILD)/localizer_%.o: $(SRC_DIR)/localizer/%.cpp | $(BUILD)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILD)/fusion_%.o: $(SRC_DIR)/Fusion/%.c | $(BUILD)
	$(CC) $(CFLAGS) -c -o $@ $<

$(BUILD)/%.o: %.cpp $(wildcard *.h) $(wildcard $(SRC_DIR)/localizer/*.h) ../shared_types.h | $(BUILD)
	$(CXX) $(CXXFLAGS) -c -o $@ $<

$(BUILD):
	mkdir -p $(BUILD)

clean:
	rm -rf $(BUILD)
