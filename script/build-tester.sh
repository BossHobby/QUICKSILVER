#! /bin/bash
set -e

MODE="release"
DEFINES=(
  "BRUSHLESS_TARGET"
  "BRUSHED_TARGET"
  "RX_UNIFIED_SERIAL"
  "RX_SBUS"
  "RX_CRSF"
  "RX_IBUS"
  "RX_FPORT"
  "RX_DSMX_2048"
  "RX_DSM2_1024"
  "RX_NRF24_BAYANG_TELEMETRY"
  "RX_BAYANG_PROTOCOL_BLE_BEACON"
  "RX_BAYANG_PROTOCOL_TELEMETRY_AUTOBIND"
  "RX_FRSKY_D8"
  "RX_FRSKY_D16"
  "RX_REDPINE"
  "RX_EXPRESS_LRS"
)
OUTPUT_FOLDER="output"
SCRIPT_FOLDER="$(dirname "$0")"
SOURCE_FOLDER="$SCRIPT_FOLDER/.."
BUILD_FOLDER="$SOURCE_FOLDER/.pio/build"

CONFIG_FILE="$SOURCE_FOLDER/src/main/config/config.h"
TARGETS_FILE="$SCRIPT_FOLDER/targets.json"

function resetConfig() {
  for DEFINE in "${DEFINES[@]}"; do
    sed -i "s/^#define \($DEFINE.*\)$/\/\/#define \1/" $CONFIG_FILE
  done
}

function setConfig() {
  for variable in ${1}; do 
    var_get() {
      echo ${variable} | base64 --decode | jq -r "${1}"
    }
    KEY=$(var_get '.key')
    VAL=$(var_get '.value')

    sed -i "s/^.*#define \($KEY.*\)$/#define $KEY $VAL/" $CONFIG_FILE
  done
}

rm -rf $OUTPUT_FOLDER
mkdir $OUTPUT_FOLDER

for target in $(jq -r '.[] | @base64' $TARGETS_FILE); do
  target_get() {
    echo ${target} | base64 --decode | jq -r "${1}"
  }

  TARGET_NAME="$(target_get '.name')"
  for config in $(target_get '.configurations[] | @base64'); do
    config_get() {
      echo ${config} | base64 --decode | jq -r "${1}"
    }
    CONFIG_NAME="$(config_get '.name')"
    BUILD_NAME="quicksilver.$TARGET_NAME.$CONFIG_NAME"

    resetConfig
    setConfig "$(config_get '.defines | to_entries[] | @base64')"

    if pio run -e $TARGET_NAME; then 
      cp "$BUILD_FOLDER/$TARGET_NAME/firmware.hex" "$OUTPUT_FOLDER/$BUILD_NAME.hex"
      echo -e "\e[32mSuccessfully\e[39m built target $BUILD_NAME"
    else
      echo -e "\e[31mError\e[39m building target $BUILD_NAME"
      exit 1
    fi
  done
done