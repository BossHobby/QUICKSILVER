# pylint: disable=missing-docstring,redefined-outer-name,unspecified-encoding

import os
import time

import serial
import serial.tools.list_ports

Import("env")

try:
    from target_inject import target_hash, process_elf_action, inject_target, copy_hex
except ImportError:
    env.Execute("$PYTHONEXE -m pip install pyelftools pyyaml cbor2")
    from target_inject import target_hash, process_elf_action, inject_target, copy_hex


def before_upload(source, target, env):
    if env["UPLOAD_PROTOCOL"] != "dfu":
        return
    for port in serial.tools.list_ports.grep("USB VID:PID=(0483|2E3C):5740"):
        with serial.Serial(port.device) as ser:
            ser.write(b"R\r\n")
            time.sleep(2)


remove_flags = ["gcc", "stdc++", "nosys", "--specs=nosys.specs"]
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

env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    [
        env.VerboseAction(
            process_elf_action,
            "Injecting target config into $BUILD_DIR/${PROGNAME}.elf",
        ),
        env.VerboseAction(
            " ".join(
                objcopy_args
                + [
                    "$BUILD_DIR/${PROGNAME}.elf",
                    "$BUILD_DIR/${PROGNAME}.hex",
                ]
            ),
            "Building $BUILD_DIR/${PROGNAME}.hex",
        ),
    ],
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
        env.VerboseAction(
            " ".join(
                objcopy_args
                + [
                    "$BUILD_DIR/${PROGNAME}.elf",
                    "$BUILD_DIR/${PROGNAME}.hex",
                ]
            ),
            "Building $BUILD_DIR/${PROGNAME}.hex",
        ),
        env.VerboseAction(
            copy_hex,
            "Copying $BUILD_DIR/${PROGNAME}.hex",
        ),
    ],
)
