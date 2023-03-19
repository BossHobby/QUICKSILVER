# pylint: disable=missing-docstring,redefined-outer-name

import re
import glob
import os
import yaml


class IndetDumper(yaml.Dumper):
    def increase_indent(self, flow=False, indentless=False):
        return super(IndetDumper, self).increase_indent(flow, False)


gyro_orientation = {
    "GYRO_ROTATE_NONE": 0x0,
    "GYRO_ROTATE_45_CCW": 0x1,
    "GYRO_ROTATE_45_CW": 0x2,
    "GYRO_ROTATE_90_CW": 0x4,
    "GYRO_ROTATE_90_CCW": 0x8,
    "GYRO_ROTATE_180": 0x10,
    "GYRO_FLIP_180": 0x20,
}


def generate_gyro_orientation(val: str):
    for k, v in gyro_orientation.items():
        val = val.replace(k, str(v))
    return eval(val)


def generate_pin(values: dict, name: str):
    if name not in values:
        return None
    matches = re.search(r"PIN_(\w\d+)", values[name])
    if matches is None:
        return None
    return "P" + matches.group(1)


def generate_spi(values: dict, name: str):
    port = None
    if f"{name}_SPI_PORT" in values:
        port = values[f"{name}_SPI_PORT"]

    nss = None
    if f"{name}_NSS" in values:
        nss = f"{name}_NSS"

    if f"{name}_NSS_PIN" in values:
        nss = f"{name}_NSS_PIN"

    if port is None or nss is None:
        return None

    return {
        "port": int(re.search(r"SPI_PORT(\d+)", port).group(1)),
        "nss": generate_pin(values, nss),
    }


def remove_none(obj):
    if isinstance(obj, dict):
        return {k: remove_none(v) for k, v in obj.items() if v is not None}
    else:
        return obj


def generate_target(name: str, values: dict):
    target = {
        "name": name,
        "mcu": target_mcus[name],
        "brushless": True,
    }

    target["leds"] = []
    for index in range(int(values["LED_NUMBER"])):
        target["leds"].append(
            {
                "pin": generate_pin(values, f"LED{index+1}PIN"),
                "invert": f"LED{index+1}_INVERT" in values,
            }
        )

    target["spi_ports"] = []
    for port in values["SPI_PORTS"]:
        parts = re.search(r"SPI(\d+)_P(\w\d+)P(\w\d+)P(\w\d+)", port)
        target["spi_ports"].append(
            {
                "index": int(parts.group(1)),
                "sck": "P" + parts.group(2),
                "miso": "P" + parts.group(3),
                "mosi": "P" + parts.group(4),
            }
        )

    target["serial_ports"] = []
    for port in values["USART_PORTS"]:
        if port.startswith("SOFT_SERIAL_PORT"):
            continue
        parts = re.search(r"USART(\d+)_P(\w\d+)P(\w\d+)", port)
        index = int(parts.group(1))
        serial = {
            "index": index,
            "rx": "P" + parts.group(2),
            "tx": "P" + parts.group(3),
        }
        if f"USART{index}_INVERTER_PIN" in values:
            serial["inverter"] = generate_pin(values, f"USART{index}_INVERTER_PIN")
        target["serial_ports"].append(serial)

    target["serial_soft_ports"] = []
    for port in values["USART_PORTS"]:
        if port.startswith("SOFT_SERIAL_PORT") is False:
            continue
        parts = re.search(r"SOFT_SERIAL_PORT\((\d+), PIN_(\w\d+), PIN_(\w\d+)\)", port)
        index = int(parts.group(1))
        serial = {
            "index": index,
            "rx": "P" + parts.group(2),
            "tx": "P" + parts.group(3),
        }
        target["serial_soft_ports"].append(serial)

    if len(target["serial_soft_ports"]) == 0:
        target["serial_soft_ports"] = None

    if "GYRO_ORIENTATION" in values:
        target["gyro_orientation"] = generate_gyro_orientation(
            values["GYRO_ORIENTATION"]
        )
    target["gyro"] = generate_spi(values, "GYRO")
    target["osd"] = generate_spi(values, "MAX7456")
    target["flash"] = generate_spi(values, "M25P16")
    target["sdcard"] = generate_spi(values, "SDCARD")

    rx_spi = generate_spi(values, "A7105")
    if rx_spi is not None:
        target["rx_spi"] = rx_spi
        target["rx_spi"]["exti"] = generate_pin(values, "A7105_GIO1_PIN")

    rx_spi = generate_spi(values, "CC2500")
    if rx_spi is not None:
        target["rx_spi"] = rx_spi
        target["rx_spi"]["exti"] = generate_pin(values, "CC2500_GDO0_PIN")
        target["rx_spi"]["ant_sel"] = generate_pin(values, "CC2500_ANT_SEL_PIN")
        target["rx_spi"]["lna_en"] = generate_pin(values, "CC2500_LNA_EN_PIN")
        target["rx_spi"]["tx_en"] = generate_pin(values, "CC2500_TX_EN_PIN")

    rx_spi = generate_spi(values, "SX12XX")
    if rx_spi is not None:
        target["rx_spi"] = rx_spi
        target["rx_spi"]["exti"] = generate_pin(values, "SX12XX_DIO0_PIN")
        target["rx_spi"]["busy"] = generate_pin(values, "SX12XX_BUSY_PIN")
        target["rx_spi"]["busy_exti"] = "USE_SX128X_BUSY_EXTI" in values
        target["rx_spi"]["reset"] = generate_pin(values, "SX12XX_RESET_PIN")

    target["usb_detect"] = generate_pin(values, "USB_DETECT_PIN")
    target["fpv"] = generate_pin(values, "FPV_PIN")
    target["vbat"] = generate_pin(values, "VBAT_PIN")
    target["ibat"] = generate_pin(values, "IBAT_PIN")

    if "BUZZER_PIN" in values:
        target["buzzer"] = {
            "pin": generate_pin(values, "BUZZER_PIN"),
            "invert": "BUZZER_INVERT" in values,
        }

    if "SDCARD_DETECT_PIN" in values:
        target["sdcard_detect"] = {
            "pin": generate_pin(values, "SDCARD_DETECT_PIN"),
            "invert": "SDCARD_DETECT_INVERT" in values,
        }

    target["motor_pins"] = []
    for index in range(4):
        parts = re.search(r"MOTOR_PIN_P(\w\d+)", values[f"MOTOR_PIN{index}"])
        target["motor_pins"].append("P" + parts.group(1))

    return remove_none(target)


target_mcus = {}

with open("platformio.ini") as file:
    target = None
    for line in file.readlines():
        parts = re.search(r"\[env:(\w+)\]", line)
        if parts is not None:
            target = parts.group(1)

        if target is not None and line.startswith("extends"):
            target_mcus[target] = line.split("=")[1].strip()

for f in glob.glob("src/target/*/target.h", recursive=True):
    folder = os.path.dirname(f)
    name = os.path.basename(folder)

    print(f"processing {name}...")
    with open(f) as file:
        content = file.read()

    content = re.sub("\\\s*\n\s*", "#", content)
    content = re.sub("//.*", "", content)
    content = re.sub(r"^\s*", "", content, flags=re.MULTILINE)

    values = {}

    for line in content.splitlines():
        if not line.startswith("#define"):
            continue

        parts = line.split(maxsplit=2)

        if len(parts) > 2:
            args = map(str.strip, parts[2].split("#"))
            args = list(filter(None, args))
            if len(args) > 1:
                values[parts[1]] = args
            else:
                values[parts[1]] = args[0]
        else:
            values[parts[1]] = True

    res = generate_target(name, values)
    with open(os.path.join(folder, "target.yaml"), "w") as f:
        yaml.dump(res, f, Dumper=IndetDumper, sort_keys=False, default_flow_style=False)
