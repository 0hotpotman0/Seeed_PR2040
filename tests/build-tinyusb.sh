#!/usr/bin/env bash

cache_dir=$(mktemp -d)

source "$TRAVIS_BUILD_DIR"/tests/common.sh

install_arduino nodebug

# Replace the skip function to only build TinyUSB tests
function skip_ino()
{
    local ino=$1
    local skiplist=""
    echo $ino | grep -q Adafruit_TinyUSB_Arduino
    if [ $? -eq 0 ]; then
        # Add items to the following list with "\n" netween them to skip running.  No spaces, tabs, etc. allowed
        read -d '' skiplist << EOL || true
/mouse_external_flash/
/hid_keyboard/
/msc_external_flash/
/msc_external_flash_sdcard/
/msc_sdfat/
/msc_sd/
/midi_pizza_box_dj/
EOL
        echo $ino | grep -q -F "$skiplist"
        echo $(( 1 - $? ))
    else
        echo 1
    fi
}
build_sketches_with_arduino 1 0 "--usbstack tinyusb"

rm -rf "$cache_dir"

