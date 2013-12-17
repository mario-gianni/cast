#!/bin/sh

if [ $# -ne 1 ]; then
	echo "Usage: `basename $0` OUTPUT-FILENAME" >&2
	exit 1
fi

OUTPUT_FILE="$1"

echo "Will record audio and dump it to \`$OUTPUT_FILE'." >&2

echo "Recording..." >&2
# Use 44.1 kHz, signed 16 big-endian, 2 channels
parec --rate=44100 --format=s16be --channels=2 \
		| lame -r -s 44.1 -m s --bitwidth 16 --signed --big-endian - "$OUTPUT_FILE"
