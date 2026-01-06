# SPDX-License-Identifier: MIT
# SPDX-FileCopyrightText: Copyright 2026 Sam Blenny

.PHONY: help bundle sync tty clean

# Name of top level folder in project bundle zip file should match repo name
PROJECT_DIR = $(shell basename `git rev-parse --show-toplevel`)

# This is for use by .github/workflows/buildbundle.yml GitHub Actions workflow
# To use this on Debian, you might need to apt install curl and zip.
bundle:
	@mkdir -p build
	python3 bundle_builder.py

# Sync current code and libraries to a CIRCUITPY drive on macOS.
sync: bundle
	@if [ -d /Volumes/CIRCUITPY ]; then \
		xattr -cr build; \
		rsync -rcvO 'build/${PROJECT_DIR}/CircuitPython 10.x/' /Volumes/CIRCUITPY; \
		sync; fi

# Open serial monitor on macOS assuming that there is only one serial device
# plugged in that will match the /dev/tty.usbmodem* device file glob
tty:
	@if [ -e /dev/tty.usbmodem* ]; then \
		screen -h 9999 -fn /dev/tty.usbmodem* 115200; fi

clean:
	rm -rf build
