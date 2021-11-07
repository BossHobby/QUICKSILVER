import os
import time

import serial
import serial.tools.list_ports

Import("env", "projenv")

def before_upload(source, target, env):
  for port in serial.tools.list_ports.grep("USB VID:PID=0483:5740"):
    with serial.Serial(port.device) as ser:
      ser.write(b'R\r\n')
      time.sleep(2)

env.AddPreAction("upload", before_upload)

env.AddPostAction(
    "$BUILD_DIR/${PROGNAME}.elf",
    env.VerboseAction(" ".join([
        "$OBJCOPY", "-O", "ihex",
        "$BUILD_DIR/${PROGNAME}.elf", "$BUILD_DIR/${PROGNAME}.hex"
    ]), "Building $BUILD_DIR/${PROGNAME}.hex")
)

git_version = "unknown"

if 'DRONE_TAG' in os.environ:
  git_version = os.environ.get('DRONE_TAG')
elif 'DRONE_COMMIT' in os.environ:
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