import glob
import hashlib
import re

import modm_devices

from pathlib import Path


class DevicesCache(dict):
    """
    Building the device enumeration from modm-device is quite expensive,
    so we cache the results in `ext/modm-devices.cache`
    The text file contains two maps:
      1. partname -> file-name.xml
         We use this to populate the `:target` option, but we only
         actually parse the device file and build the device on the first
         access of the value.
      2. file-name.xml -> MD5 hash
         This is used to check if any files have changed their contents.
         No additional checks are done, if files have moved, this may fail.
    """

    def __init__(self):
        dict.__init__(self)
        self.device_to_file = {}

    def parse_all(self):
        mapping = {}
        device_file_names = glob.glob(
            modm_devices.pkg.get_filename("modm_devices", "resources/devices/**/*.xml")
        )

        # roughly filter to supported devices
        supported = ["stm32f4", "stm32g4", "stm32f7", "stm32h7"]
        device_file_names = [
            dfn for dfn in device_file_names if any(s in dfn for s in supported)
        ]

        # Parse the files and build the :target enumeration
        parser = modm_devices.parser.DeviceParser()
        for device_file_name in device_file_names:
            device_file = parser.parse(device_file_name)
            for device in device_file.get_devices():
                self[device.partname] = device
                mapping[device.partname] = device_file_name

        return mapping

    def build(self):
        cache = Path(".pio/modm-devices.cache")
        repos = {
            p: None
            for p in [
                ".",
                modm_devices.pkg.get_filename("modm_devices", "resources/devices"),
            ]
        }
        recompute_required = not cache.exists()

        if cache.exists():
            # Read cache file and populate :target
            for line in cache.read_text().splitlines():
                line = line.split(" ")
                if line[0].startswith("/"):
                    file = line[0][1:]

                    # Store repo shas
                    if file in repos.keys():
                        repos[file] = line[1]
                        continue

                    # Check normal files
                    file = Path(file)
                    if not file.exists() or (
                        line[1] != hashlib.md5(file.read_bytes()).hexdigest()
                    ):
                        recompute_required = True
                        break
                else:
                    # Store None as device file value
                    self.device_to_file[line[0]] = line[1]
                    self[line[0]] = None

        if recompute_required:
            content = self.parse_all()
            files = [
                "{} {}".format(Path(f), hashlib.md5(Path(f).read_bytes()).hexdigest())
                for f in set(content.values())
            ]
            content = ["{} {}".format(d, Path(f)) for (d, f) in content.items()]
            repos = ["{} {}".format(path, sha) for path, sha in repos.items()]
            content = sorted(content) + sorted(files + repos)
            cache.write_text("\n".join(content))

    def __getitem__(self, item) -> modm_devices.device.Device:
        value = dict.__getitem__(self, item)
        if value is None:
            # Parse the device file and build its devices
            parser = modm_devices.parser.DeviceParser()
            device_file = parser.parse(self.device_to_file[item])
            for device in device_file.get_devices():
                self[device.partname] = device
            return self[item]
        return value


def map_signal(s):
    if s is None:
        return None

    if s["driver"] == "tim" and s["name"].startswith("ch"):
        return {
            "func": "timer",
            "af": int(s.get("af", "-1")),
            "instance": int(s["instance"]),
            "name": s["name"],
        }
    elif s["driver"] == "spi" and (
        s["name"] == "sck"
        or s["name"] == "mosi"
        or s["name"] == "miso"
        or s["name"] == "tx"
        or s["name"] == "rx"
    ):
        return {
            "func": "spi",
            "af": int(s.get("af", "-1")),
            "instance": int(s["instance"]),
            "name": {"tx": "mosi", "rx": "miso"}.get(s["name"], s["name"]),
        }
    elif (s["driver"] == "uart" or s["driver"] == "usart") and (
        s["name"] == "rx" or s["name"] == "tx"
    ):
        return {
            "func": "serial",
            "af": int(s.get("af", "-1")),
            "instance": int(s["instance"]),
            "name": s["name"],
        }
    elif s["driver"] == "adc" and re.match(r"in\d+", s.get("name", "in0")):
        return {
            "func": "adc",
            "af": -1,
            "instance": int(s["instance"]),
            "name": s.get("name", "in0xff")[2:],
        }
    else:
        return None


def map_tag(f):
    if f is None:
        return None

    if f["func"] == "timer":
        return f"TIMER_TAG(TIMER{f['instance']}, TIMER_{f['name']})".upper()
    elif f["func"] == "spi":
        return f"SPI_TAG(SPI_PORT{f['instance']}, RES_SPI_{f['name']})".upper()
    elif f["func"] == "serial":
        return f"SERIAL_TAG(SERIAL_PORT{f['instance']}, RES_SERIAL_{f['name']})".upper()
    elif f["func"] == "adc":
        return f"ADC_TAG(ADC_DEVICE{f['instance']}, {f['name']})".upper()
    else:
        return None


devices = [
    "stm32f405rg",
    "stm32f411re",
    "stm32g473ceu6",
    "stm32f722re",
    "stm32f745vg",
    "stm32f765vi",
    "stm32h743vi",
]

caches = DevicesCache()
caches.build()

for device in devices:
    pins = {}
    key = next((x for x in caches.keys() if x.startswith(device)), None)

    with open(f"src/system/{device[:9]}/gpio_pins.in", "w") as file:
        for driver in caches[key].get_all_drivers("gpio"):
            for pin in driver["gpio"]:
                file.write(f"GPIO_PIN({pin['port']}, {pin['pin']})\n".upper())
                if "signal" not in pin:
                    continue

                for s in pin["signal"]:
                    f = map_signal(s)
                    s = map_tag(f)
                    if s is None:
                        continue
                    file.write(
                        f"GPIO_AF(PIN_{pin['port']}{pin['pin']}, {f['af']}, {s})\n".upper()
                    )

    with open(f"src/system/{device[:9]}/dma.in", "w") as file:
        for driver in caches[key].get_all_drivers("dma"):
            if "streams" in driver:
                for dma in driver["streams"]:
                    for stream in dma["stream"]:
                        for channel in stream["channel"]:
                            funcs = [
                                r
                                for s in channel["signal"]
                                if (r := map_tag(map_signal(s))) is not None
                            ]
                            for func in funcs:
                                entry = ", ".join(
                                    [
                                        f".tag = {func}",
                                        f".port_index = {dma['instance']}",
                                        f".stream_index = {stream['position']}",
                                        f".channel = LL_DMA_CHANNEL_{channel['position']}",
                                    ]
                                )
                                file.write("{ " + entry + " },\n")
            if "requests" in driver:
                for dma in driver["requests"]:
                    for request in dma["request"]:
                        funcs = [
                            r
                            for s in request["signal"]
                            if (r := map_tag(map_signal(s))) is not None
                        ]
                        for func in funcs:
                            entry = ", ".join(
                                [
                                    f".tag = {func}",
                                    f".port_index = -1",
                                    f".stream_index = -1",
                                    f".request = {request['position']}",
                                ]
                            )
                            file.write("{ " + entry + " },\n")
