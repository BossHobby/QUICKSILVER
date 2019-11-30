#! /bin/bash
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
  "RX_FRSKY"
)
SCRIPT_FOLDER="$(dirname "$0")"
OUTPUT_FOLDER="output"

INDEX_PAGE="$OUTPUT_FOLDER/quicksilver.html"
CONFIG_FILE="src/main/config/config.h"
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

echo '<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <meta http-equiv="X-UA-Compatible" content="ie=edge">
  <title>QUICKSILVER Builds</title>
</head><body>' > $INDEX_PAGE

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

    if make -j32 MODE="$MODE" $TARGET_NAME &> /dev/null; then 
      cp "build/$MODE/quicksilver.$TARGET_NAME.$MODE.hex" "$OUTPUT_FOLDER/$BUILD_NAME.hex"
      upload "$OUTPUT_FOLDER/$BUILD_NAME.hex" &> /dev/null
      echo "<div><a target=\"_blank\" href=\"$BUILD_NAME.hex\">$BUILD_NAME</a></div>" >> $INDEX_PAGE

      echo -e "\e[32mSuccessfully\e[39m built target $BUILD_NAME"
    else
      echo -e "\e[31mError\e[39m building target $BUILD_NAME"
    fi
  done
done

echo '</body></html>' >> $INDEX_PAGE
upload $INDEX_PAGE &> /dev/null