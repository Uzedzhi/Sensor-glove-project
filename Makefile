SHELL := /bin/bash

ESP_IDF_VERSION ?= v6.1
IDF_PATH ?= $(HOME)/esp/esp-idf-$(ESP_IDF_VERSION)
IDF_PYTHON ?= /opt/homebrew/bin/python3
PORT ?= auto
BAUD ?= 460800

PORT_ARG := $(if $(filter auto,$(PORT)),,-p "$(PORT)")
IDF := test -f "$(IDF_PATH)/export.sh" || { echo "ESP-IDF not found at $(IDF_PATH). Run: make sdk" >&2; exit 1; }; test -x "$(IDF_PYTHON)" || { echo "Python not found at $(IDF_PYTHON)" >&2; exit 1; }; PATH="$(dir $(IDF_PYTHON)):$$PATH"; . "$(IDF_PATH)/export.sh" >/dev/null && idf.py -D IDF_TARGET=esp32c3

.PHONY: help sdk build menuconfig flash monitor flash-monitor erase size clean fullclean doctor

help:
	@printf '%s\n' \
	  'Seeed Studio XIAO ESP32-C3 + BNO055 (native ESP-IDF)' \
	  '' \
	  '  make sdk                         install ESP-IDF v6.1 for ESP32-C3' \
	  '  make build                       compile firmware' \
	  '  make menuconfig                  configure GPIO and ESP-IDF options' \
	  '  make flash [PORT=/dev/cu.usb...] upload firmware' \
	  '  make monitor [PORT=/dev/cu.usb...] open serial monitor' \
	  '  make flash-monitor [PORT=...]     upload, then open monitor' \
	  '  make erase [PORT=...]             erase chip flash' \
	  '  make size                          print firmware size report' \
	  '  make clean | make fullclean        remove build output/config' \
	  '  make doctor                        show tool and serial-port info' \
	  '' \
	  'PORT defaults to auto-detection; BAUD defaults to 460800.'

sdk:
	@if [ ! -d "$(IDF_PATH)/.git" ]; then \
		mkdir -p "$(dir $(IDF_PATH))"; \
		git -c http.version=HTTP/1.1 clone --branch "$(ESP_IDF_VERSION)" --depth 1 --filter=blob:none --no-recurse-submodules https://github.com/espressif/esp-idf.git "$(IDF_PATH)"; \
	fi
	@git -C "$(IDF_PATH)" -c http.version=HTTP/1.1 submodule update --init --recursive --depth 1
	@cd "$(IDF_PATH)" && PATH="$(dir $(IDF_PYTHON)):$$PATH" ./install.sh esp32c3
	@printf '\nInstalled. Run `make build` in this project.\n'

build:
	@$(IDF) build

menuconfig:
	@$(IDF) menuconfig

flash:
	@$(IDF) $(PORT_ARG) -b "$(BAUD)" flash

monitor:
	@$(IDF) $(PORT_ARG) monitor

flash-monitor:
	@$(IDF) $(PORT_ARG) -b "$(BAUD)" flash monitor

erase:
	@$(IDF) $(PORT_ARG) erase-flash

size:
	@$(IDF) size

clean:
	@$(IDF) clean

fullclean:
	@$(IDF) fullclean

doctor:
	@$(IDF) --version
	@printf '%s\n' 'Supported targets:'
	@$(IDF) --list-targets
	@printf '%s\n' 'Serial devices:'
	@/bin/ls /dev/cu.* 2>/dev/null || true
