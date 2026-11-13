#!/bin/sh
set -eu

config_dir=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
source_file="$config_dir/rear_brush_guard.py"
klipper_extras="/home/pi/klipper/klippy/extras"
link="$klipper_extras/rear_brush_guard.py"

if [ -L "$link" ] && [ "$(readlink "$link")" = "$source_file" ]; then
    exit 0
fi
if [ -e "$link" ] || [ -L "$link" ]; then
    echo "Refusing to replace existing $link" >&2
    exit 1
fi
ln -s "$source_file" "$link"
