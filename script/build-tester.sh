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
  "RX_DSMX"
  "RX_DSM2"
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

BRANCH=$DRONE_BRANCH
if [ -z "$BRANCH" ]; then
  BRANCH="$(git rev-parse --abbrev-ref HEAD)"
fi

BUILD_PREFIX="quicksilver"
if [ ! -z "$BRANCH" ] && [ "$BRANCH" != "master" ] && [ "$BRANCH" != "develop" ]; then
  BUILD_PREFIX="$BUILD_PREFIX.$BRANCH"
fi

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

jq 'empty' ./script/targets.json

rm -rf $OUTPUT_FOLDER
mkdir $OUTPUT_FOLDER

cat <<-EOF > $OUTPUT_FOLDER/index.html
<!doctype html>
<html lang="en">

<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link href="https://cdn.jsdelivr.net/npm/bootstrap@5.1.3/dist/css/bootstrap.min.css" rel="stylesheet"
    integrity="sha384-1BmE4kWBq78iYhFldvKuhfTAU6auU8tT94WrHftjDbrCEXSU1oBoqyl2QvZ6jIW3" crossorigin="anonymous">
  <title>Quicksilver Develop</title>
</head>

<body>
  <main>
    <div class="px-4 py-5 my-3 text-center">
      <h1 class="display-5 mb-4 fw-bold">Quicksilver Develop</h1>
      <div class="col-lg-6 mx-auto">
        <p class="lead mb-4">
          Commit <a href="https://github.com/BossHobby/QUICKSILVER/commit/$DRONE_COMMIT">$DRONE_COMMIT</a>
        </p>
        <div class="list-group">
EOF

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
    BUILD_NAME="$BUILD_PREFIX.$TARGET_NAME.$CONFIG_NAME"

    resetConfig
    setConfig "$(config_get '.defines | to_entries[] | @base64')"

    if pio run -e $TARGET_NAME; then 
      cp "$BUILD_FOLDER/$TARGET_NAME/firmware.hex" "$OUTPUT_FOLDER/$BUILD_NAME.hex"
      echo -e "\e[32mSuccessfully\e[39m built target $BUILD_NAME"
      echo "<a class=\"list-group-item list-group-item-action\" href=\"$BUILD_NAME.hex\" download target=\"_blank\">$BUILD_NAME</a>" >> $OUTPUT_FOLDER/index.html
    else
      echo -e "\e[31mError\e[39m building target $BUILD_NAME"
      exit 1
    fi
  done
done

cat <<-EOF >> $OUTPUT_FOLDER/index.html
        </div>
      </div>
    </div>
  </main>
</body>

</html>
EOF