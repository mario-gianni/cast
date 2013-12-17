#!/bin/bash

#------------------------------------------------------------------------
# Settings.

RUNS_CONFIG_FILE=~/Desktop/RUNS-CONFIG.sh
ARCHIVE_DIR="$HOME/ARCHIVE"
RUNS_DIR="$ARCHIVE_DIR/runs"

#------------------------------------------------------------------------
# Misc.

print_pwd() {
	echo "Current WD is \``pwd`'." >&2
}

#------------------------------------------------------------------------
# Set up the environment.

echo "Sourcing \`$RUNS_CONFIG_FILE'." >&2
. $RUNS_CONFIG_FILE

echo "Will be using ROS_MASTER_URI=$ROS_MASTER_URI" >&2
echo "Will be using CAST instantiation file \`$CAST_INST_FILE'" >&2

#------------------------------------------------------------------------
# Prepare the archive dir.


THIS="`date '+%s'`"
THIS_DIR="$RUNS_DIR/$THIS"

echo "Will use \`$THIS_DIR' for storing this run." >&2
mkdir -p "$THIS_DIR"

LATEST="`pwd`/latest-archive"

rm -fv "$LATEST"
ln -sv "$THIS_DIR" "$LATEST"

#------------------------------------------------------------------------
# Start the stuff.

# register a SIGINT trap that shuts down all background jobs
trap 'kill $(jobs -p)' SIGINT

echo "Going to start everything now." >&2

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Start MARY.

echo "Starting Mary..." >&2
print_pwd
gnome-terminal --geometry=80x6 -x tools/mary/bin/maryserver
sleep 8s
echo "Assuming Mary is ready now." >&2

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Start the abducer.

echo "Starting the abducer..." >&2
print_pwd
gnome-terminal --geometry=80x8 -x tools/abducer/start-abducer-server
sleep 2s
echo "Assuming that the abducer is ready now." >&2

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Start audio recording.

#print_pwd
#gnome-terminal --geometry=80x4 -x ./record-audio.sh "$LATEST/audio-in.mp3"

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Start CAST server.

echo "Starting the CAST server..." >&2
print_pwd
gnome-terminal -x sh -c 'output/bin/cast-server-start ; sh'
sleep 2s
echo "Assuming CAST server is ready now" >&2

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# Start CAST client.

echo "Starting the CAST client with inst file \`$CAST_INST_FILE'..." >&2
print_pwd
gnome-terminal -x sh -c "output/bin/cast-client-start $CAST_INST_FILE ; sh"

# - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
# That's it.

echo "Things should be up and running now!" >&2
