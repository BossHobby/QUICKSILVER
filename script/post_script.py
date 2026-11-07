# pylint: disable=missing-docstring,redefined-outer-name,unspecified-encoding

import os
import time

import serial
import serial.tools.list_ports

Import("env")

try:
    from target_inject import target_hash, process_elf_action, fast_code_report_action, inject_target, copy_hex
except ImportError:
    env.Execute("$PYTHONEXE -m pip install pyelftools pyyaml cbor2")
    from target_inject import target_hash, process_elf_action, fast_code_report_action, inject_target, copy_hex


def before_upload(source, target, env):
    # 'R' reboots into the STM32/AT32 DFU or the RP2350 ROM bootloader picotool expects.
    usb_ids = {
        "dfu": "(0483|2E3C):5740",
        "picotool": "2E8A:000A",
    }
    if env["UPLOAD_PROTOCOL"] not in usb_ids:
        return
    for port in serial.tools.list_ports.grep("USB VID:PID=" + usb_ids[env["UPLOAD_PROTOCOL"]]):
        with serial.Serial(port.device) as ser:
            # Betaflight cancels a pending reboot if another byte arrives within 100 ms.
            ser.write(b"R")
            time.sleep(2)
        if env["UPLOAD_PROTOCOL"] == "picotool":
            env.Replace(UPLOAD_PORT=port.device)
    # picotool finds BOOTSEL devices itself and ignores the port, while PlatformIO's
    # autodetect would scan every board definition and choke on the AT32 ones.
    if env["UPLOAD_PROTOCOL"] == "picotool" and not env.subst("$UPLOAD_PORT"):
        env.Replace(UPLOAD_PORT="bootsel")


remove_flags = ["gcc", "nosys", "--specs=nosys.specs"]
for scope in ("ASFLAGS", "CCFLAGS", "LINKFLAGS", "LIBS"):
    for option in remove_flags:
        while option in env[scope]:
            env[scope].remove(option)

env.Append(
    CPPDEFINES=[("TARGET_HASH", target_hash(env))],
)

objcopy_args = [
    "$OBJCOPY",
    "-S",
    "-O",
    "ihex",
]

image_actions = [
    env.VerboseAction(
        " ".join(objcopy_args + ["$BUILD_DIR/${PROGNAME}.elf", "$BUILD_DIR/${PROGNAME}.hex"]),
        "Building $BUILD_DIR/${PROGNAME}.hex",
    ),
]
if env["PIOPLATFORM"] == "raspberrypi":
    # The bootrom flashes UF2. The platform builds one before the target
    # config is injected, so rebuild it from the injected ELF.
    image_actions.append(
        env.VerboseAction(
            "picotool uf2 convert -t elf $BUILD_DIR/${PROGNAME}.elf $BUILD_DIR/${PROGNAME}.uf2",
            "Building $BUILD_DIR/${PROGNAME}.uf2",
        )
    )

env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    [
        env.VerboseAction(
            process_elf_action,
            "Injecting target config into $BUILD_DIR/${PROGNAME}.elf",
        ),
        env.VerboseAction(fast_code_report_action, "Checking ITCM fast code usage"),
    ]
    + image_actions,
)

env.AddPreAction("upload", before_upload)
env.AddTarget(
    "inject",
    os.path.join("$PROJECT_BUILD_DIR", "$BOARD_MCU", "${PROGNAME}.elf"),
    [
        env.VerboseAction(
            inject_target,
            "Injecting target config into $BUILD_DIR/${PROGNAME}.elf",
        ),
    ]
    + image_actions
    + [
        env.VerboseAction(
            copy_hex,
            "Copying $BUILD_DIR/${PROGNAME}.hex",
        ),
    ],
)
