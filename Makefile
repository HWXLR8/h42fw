PICOTOOL_PATH := ./picotool
PICO_SDK_PATH := ./pico-sdk
BIN := build/h42fw.uf2

all: build

$(PICO_SDK_PATH):
	git clone --depth 1 --recurse-submodules https://github.com/raspberrypi/pico-sdk.git

config: $(PICO_SDK_PATH)
	cmake -S . -B build -DPICO_SDK_PATH=$(PICO_SDK_PATH)

build: config
	cmake --build build -j

flash:
	$(PICOTOOL_PATH) load -v -x $(BIN)

clean:
	rm -rf build
