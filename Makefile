# Build disting NT custom algorithm plugins against the distingNT_API submodule.
#
# Targets:
#   build    - cross-compile every *.cpp in this directory to plugins/*.o
#   hardware - alias for build (all plugin builds already target the module's ARM hardware)
#   push     - build, then transfer the built .o files to a connected disting NT
#              over MIDI SysEx (same mechanism nt_helper uses), no SD card removal needed
#   clean    - remove build output

NT_API_PATH := distingNT_API
INCLUDE_PATH := $(NT_API_PATH)/include
THIRD_PARTY_PATH := third_party/mi_drums

CXX := arm-none-eabi-c++
CXXFLAGS := -std=c++11 -mcpu=cortex-m7 -mfpu=fpv5-d16 -mfloat-abi=hard -mthumb \
            -fno-rtti -fno-exceptions -Os -fPIC -Wall -I$(INCLUDE_PATH) -I$(THIRD_PARTY_PATH)

PYTHON := python3

# Override on the command line, e.g. `make push MIDI_PORT="disting NT"`
MIDI_PORT ?=
SYSEX_ID ?= 0
DEST_DIR ?= /programs/plug-ins

sources := $(wildcard *.cpp)
outputs := $(patsubst %.cpp,plugins/%.o,$(sources))

.PHONY: build hardware push clean check-api

check-api:
	@if [ ! -f "$(INCLUDE_PATH)/distingnt/api.h" ]; then \
		echo "error: $(INCLUDE_PATH)/distingnt/api.h not found."; \
		echo "Run: git submodule update --init"; \
		exit 1; \
	fi

build: check-api $(outputs)

hardware: build

plugins/%.o: %.cpp
	mkdir -p $(@D)
	$(CXX) $(CXXFLAGS) -c -o $@ $^

push: build
	$(PYTHON) tools/nt_push.py \
		$(if $(MIDI_PORT),--port "$(MIDI_PORT)") \
		--sysex-id $(SYSEX_ID) \
		--dest-dir "$(DEST_DIR)" \
		$(outputs)

clean:
	rm -f $(outputs)
