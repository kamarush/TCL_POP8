#!/bin/bash
if [ -z ${TARGET_PRODUCT} ]; then TARGET_PRODUCT=$3; fi
if [ -z ${TARGET_PRODUCT} ]; then
    echo "*** TARGET_PRODUCT is not set. stop"
    exit
fi


# export variables to shell environments
eval `TARGET_PRODUCT=${TARGET_PRODUCT} _prefix_=$1 _postfix_=$2 make --no-print-directory -i -f $1/mediatek/build/libs/shell.mk`

# for legacy "Download folder". Will be removed once nobody use it.
function make_legacy_download_folder() {
  if [ ${OUT_DIR:0:1}x == '/x' ]; then
    legacy_download_path=${OUT_DIR}/Download
  else
    legacy_download_path=${TO_ROOT}/out/Download
  fi
  if [ ! -d ${legacy_download_path} ]; then
    mkdir -p ${legacy_download_path}/sdcard
    mkdir -p ${legacy_download_path}/flash
  fi
  ln -sf ${legacy_download_path} Download
  if [ ${OUT_DIR:0:1}x == '/x' ]; then
    legacy_download_path=${OUT_DIR}/Download/$1
  else
    legacy_download_path=${TO_ROOT}/out/Download/$1
  fi
}

function copy_to_legacy_download_folder() {
  for item in $@; do
    if [ -e $item ]; then
      chmod u+w $item
      cp -f $item $legacy_download_path/
    fi
  done
}

function copy_to_legacy_download_flash_folder() {
  make_legacy_download_folder flash
  copy_to_legacy_download_folder $@
}

function copy_to_legacy_download_sdcard_folder() {
  make_legacy_download_folder sdcard
  copy_to_legacy_download_folder $@
}

