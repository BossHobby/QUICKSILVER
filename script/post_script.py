import os
import time
import subprocess

import serial
import serial.tools.list_ports

Import("env", "projenv")

try:
    import yaml
    import cbor2
    from elftools.elf.elffile import ELFFile
except ImportError:
    env.Execute("$PYTHONEXE -m pip install pyelftools pyyaml cbor2")
    import yaml
    import cbor2
    from elftools.elf.elffile import ELFFile


git_version = "unknown"

if 'DRONE_TAG' in os.environ:
    git_version = os.environ.get('DRONE_TAG')
elif 'DRONE_COMMIT' in os.environ:
    git_version = os.environ.get('DRONE_COMMIT')
elif 'GITHUB_VERSION' in os.environ:
    git_version = os.environ.get('GITHUB_VERSION')
else:
    try:
        ret = subprocess.run(['git', 'rev-parse', 'HEAD'],
                             stdout=subprocess.PIPE, text=True)
        full_hash = ret.stdout.strip()
        git_version = full_hash[:7]
    except Exception:
        pass

print("GIT_VERSION: " + git_version)

projenv.Append(CPPDEFINES=[
    ("TARGET",  env["BOARD_MCU"]),
    ("GIT_VERSION", git_version)
])

remove_flags = ["-lgcc", "-lstdc++"]

for scope in ("ASFLAGS", "CCFLAGS", "LINKFLAGS"):
    for option in remove_flags:
        while option in env[scope]:
            env[scope].remove(option)


def before_upload(source, target, env):
    for port in serial.tools.list_ports.grep("USB VID:PID=0483:5740"):
        with serial.Serial(port.device) as ser:
            ser.write(b'R\r\n')
            time.sleep(2)


env.AddPreAction("upload", before_upload)


def process_elf(source, target, env):
    magic = 0x12AA0001
    section_name = ".config_flash"

    target_yaml = env.subst("src/target/$PIOENV/target.yaml")
    with open(target_yaml, 'r') as f:
        target_config = yaml.load(f, Loader=yaml.Loader)

    elf_filename = env.subst("$BUILD_DIR/${PROGNAME}.elf")

    with open(elf_filename, 'r+b') as f:
        elf = ELFFile(f)
        elf_section = elf.get_section_by_name(section_name)

        section_start = elf_section['sh_offset']
        section_end = section_start + elf_section['sh_size']

        f.seek(section_start)
        f.write(magic.to_bytes(4, 'little'))
        cbor2.dump(target_config, f)

        assert(f.tell() <= section_end)


env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    [
        process_elf,
        env.VerboseAction(" ".join([
            "$OBJCOPY", "-O", "ihex",
            "\"$BUILD_DIR/${PROGNAME}.elf\"", "\"$BUILD_DIR/${PROGNAME}.hex\""
        ]), "Building $BUILD_DIR/${PROGNAME}.hex")
    ]
)
