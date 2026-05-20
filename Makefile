.PHONY: all build configure test format lint quality clean

BUILD_DIR := build/debug

SRCS := \
	homework_06/src/main.cpp \
	homework_06/src/ballistics.cpp \
	homework_06/tests/ballistics_tests.cpp

HEADERS := \
	homework_06/include/ballistics.hpp

all: build

configure:
	cmake --preset debug

build: configure
	cmake --build --preset debug

test: build
	ctest --test-dir $(BUILD_DIR)/homework_06 --output-on-failure

format:
	cmake-format -i --config-file=/.cmake-format.json homework_06/CMakeLists.txt

lint: configure
	clang-tidy $(SRCS) $(HEADERS) -p $(BUILD_DIR)

quality: format lint test

clean:
	rm -rf build