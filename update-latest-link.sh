#!/bin/sh

if [ $# -ne 1 ]; then
	echo "Usage: `basename $0` TARGET"
	echo "Will put a symlink to TARGET named 'latest' to TARGET's parent directory."
	exit 1
fi

TARGET="$1"

TARGET_RELOC="`basename \"$TARGET\"`"

PARENT_DIR="`dirname \"$TARGET\"`"
SYMLINK_NAME="latest"

SYMLINK_FULLPATH="$PARENT_DIR/$SYMLINK_NAME"

echo "Will put the symlink to '$PARENT_DIR'." >&2
echo "Will call the symlink '$SYMLINK_NAME'." >&2
echo "Will make the symlink refer to '$TARGET_RELOC'." >&2

if [ -h "$SYMLINK_FULLPATH" -o \! -e "$SYMLINK_FULLPATH" ]; then
	rm -vf "$SYMLINK_FULLPATH"
	ln -vs "$TARGET_RELOC" "$SYMLINK_FULLPATH"

else
	echo "A file called '$SYMLINK_FULLPATH' exists and is not a symlink:" >&2
	ls -l "$SYMLINK_FULLPATH"
	echo " --> aborting" >&2
	exit 1
fi

