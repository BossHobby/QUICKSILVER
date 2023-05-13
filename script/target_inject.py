# pylint: disable=missing-docstring,redefined-outer-name,unspecified-encoding

import os
import shutil
import cbor2
import yaml

from elftools.elf.elffile import ELFFile


def process_elf(target_yaml: str, target_elf: str):
    magic = 0x12AA0001
    section_name = ".config_flash"

    with open(target_yaml, "r") as f:
        target_config = yaml.load(f, Loader=yaml.Loader)

    with open(target_elf, "r+b") as f:
        elf = ELFFile(f)
        elf_section = elf.get_section_by_name(section_name)
        if elf_section is None:
            raise KeyError("config flash section not found")

        section_start = elf_section["sh_offset"]
        section_end = section_start + elf_section["sh_size"]

        f.seek(section_start)
        f.write(magic.to_bytes(4, "little"))
        cbor2.dump(target_config, f)

        assert f.tell() <= section_end


def process_elf_action(source, target, env):
    target_yaml = env.subst("targets/${PIOENV}.yaml")
    if not os.path.isfile(target_yaml):
        print(f"Skipping injecting {target_yaml}")
        return

    target_elf = env.subst("$BUILD_DIR/${PROGNAME}.elf")
    process_elf(target_yaml, target_elf)


def inject_target(source, target, env):
    target_yaml = env.subst("targets/${PIOENV}.yaml")
    if not os.path.isfile(target_yaml):
        print(f"Skipping injecting {target_yaml}")
        return

    mcu_elf = env.subst(
        os.path.join(
            "$PROJECT_BUILD_DIR",
            "$BOARD_MCU",
            "${PROGNAME}.elf",
        )
    )
    target_elf = env.subst("$BUILD_DIR/${PROGNAME}.elf")
    shutil.copy(mcu_elf, target_elf)
    process_elf(target_yaml, target_elf)


def copy_hex(source, target, env):
    hex_name = "quicksilver."
    git_branch = env.get("GIT_BRANCH")
    if git_branch != "":
        hex_name += git_branch + "."
    hex_name += "${PIOENV}.hex"

    os.makedirs("output", exist_ok=True)
    shutil.copy(
        env.subst("$BUILD_DIR/${PROGNAME}.hex"),
        env.subst(os.path.join("output", hex_name)),
    )
