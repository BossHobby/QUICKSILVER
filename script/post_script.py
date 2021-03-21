import os

Import("env", "projenv")

env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "$OBJCOPY", "-O", "ihex",
        "$BUILD_DIR/${PROGNAME}.elf", "$BUILD_DIR/${PROGNAME}.hex"
    ]), "Building $BUILD_DIR/${PROGNAME}.hex")
)

git_version = "unknown"

if 'DRONE_COMMIT' in os.environ:
  git_version = os.environ.get('DRONE_COMMIT')
else:
  try:
    with open('.git/refs/heads/master', 'r') as file:
      git_version = file.read().rstrip()
  except FileNotFoundError:
    pass

projenv.Append(CPPDEFINES=[
  ("TARGET", env["PIOENV"]),
  ("GIT_VERSION", git_version)
])

remove_flags = ["-lgcc", "-lstdc++"]

for scope in ("ASFLAGS", "CCFLAGS", "LINKFLAGS"):
  for option in remove_flags:
    while option in env[scope]:
      env[scope].remove(option)