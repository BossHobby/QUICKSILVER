import glob
import hashlib

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
        supported = ["stm32f4", "stm32f7", "stm32h7"]
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


devices = [
    "stm32f405rg",
    "stm32f411re",
    "stm32f722re",
    "stm32f745vg",
    "stm32f765vi",
    "stm32h743vi",
]

caches = DevicesCache()
caches.build()

for device in devices:
    key = next((x for x in caches.keys() if x.startswith(device)), None)
    cache = caches[key]

    print("#ifdef %s" % (device[:-2].upper()))
    for driver in cache.get_all_drivers("gpio"):
        for pin in driver["gpio"]:
            name = (pin["port"] + pin["pin"]).upper()

            timer = []
            spi = []
            if "signal" in pin:
                for s in pin["signal"]:
                    if s["driver"] == "tim" and s["name"].startswith("ch"):
                        timer.append(
                            (
                                "TIM(%s, %s, %s)" % (s["instance"], s["name"], s["af"])
                            ).upper()
                        )
                    if s["driver"] == "spi":
                        spi.append(("SPI(%s, %s)" % (s["instance"], s["af"])).upper())

            print(
                "GPIO_PIN(%s, %s, (%s), (%s))"
                % (pin["port"].upper(), pin["pin"], ", ".join(timer), ", ".join(spi))
            )
    print("#endif")
    print()
