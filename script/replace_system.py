import os

Import("env")

exclude = [
  "startup_stm32f40xx.S",
  "system_stm32f4xx.c",
  "stm32f4xx_fmc.c",
  "stm32f4xx_fsmc.c"
]

def replace_system(node):
  if node.name in exclude:
    print("dropping ", node)
    return None

  return node

env.AddBuildMiddleware(
  replace_system,
  "*/framework-spl/*"
)


git_version = "unknown"

if 'DRONE_COMMIT' in os.environ:
  git_version = os.environ.get('DRONE_COMMIT')
else:
  try:
    with open('.git/refs/heads/master' 'r') as file:
      git_version = file.read()
  except FileNotFoundError:
    pass

env.Append(CPPDEFINES=[
  ("TARGET", env["PIOENV"]),
  ("GIT_VERSION", git_version)
])