"""Build the pinned PlatformIO dependency with exactly one FreeRTOS port."""

from pathlib import Path

Import("env", "projenv")

kernel = Path(env.subst("$PROJECT_LIBDEPS_DIR/$PIOENV/FreeRTOS-Kernel"))
if not (kernel / "tasks.c").is_file():
    raise RuntimeError("FreeRTOS-Kernel dependency missing; run pio pkg install")

platform = env.subst("$PIOPLATFORM")
mcu = env.BoardConfig().get("build.mcu", "")
if platform == "native":
    port = "portable/ThirdParty/GCC/Posix"
    port_sources = [f"+<{port}/utils/wait_for_event.c>"]
    env.Append(CCFLAGS=["-pthread"], LINKFLAGS=["-pthread"])
elif mcu.startswith("stm32f7"):
    # Covers the F7 r0p1 core and its BASEPRI erratum workaround.
    port = "portable/GCC/ARM_CM7/r0p1"
    port_sources = []
else:
    # M4F (F4/G4/AT32), and later M7 revisions (H743), per upstream port.c.
    port = "portable/GCC/ARM_CM4F"
    port_sources = []

env.Append(CPPPATH=[str(kernel / "include"), str(kernel / port)])
projenv.Append(CPPPATH=[str(kernel / "include"), str(kernel / port)])
# Naked handlers reference kernel symbols in assembly; retain those boundaries.
kernel_env = env.Clone()
kernel_env.Append(CCFLAGS=["-fno-lto"])
kernel_library = kernel_env.BuildLibrary(
    env.subst("$BUILD_DIR/FreeRTOS-Kernel"),
    str(kernel),
    src_filter=["-<*>", "+<tasks.c>", "+<list.c>", f"+<{port}/port.c>"] + port_sources,
)
env.Prepend(LIBS=[kernel_library])
