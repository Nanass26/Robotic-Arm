# Tests natifs du firmware ORION-6 (g++ ou clang++, aucune carte nécessaire)
CXX ?= g++
CXXFLAGS ?= -std=c++17 -O2 -Wall -Wextra -Wno-unused-parameter
BUILD := build

all: run

$(BUILD)/test_core: test_core.cpp ../orion_fw/src/core/robot.cpp ../orion_fw/src/core/robot.h ../orion_fw/src/core/hal.h ../orion_fw/src/orion_config.h
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -o $@ test_core.cpp ../orion_fw/src/core/robot.cpp -lm

$(BUILD)/test_mit: test_mit.cpp ../mit_bridge/src/mit_core.cpp ../mit_bridge/src/mit_core.h ../mit_bridge/src/orion_config.h
	@mkdir -p $(BUILD)
	$(CXX) $(CXXFLAGS) -o $@ test_mit.cpp ../mit_bridge/src/mit_core.cpp -lm

run: $(BUILD)/test_core $(BUILD)/test_mit
	./$(BUILD)/test_core
	./$(BUILD)/test_mit

clean:
	rm -rf $(BUILD)

.PHONY: all run clean
