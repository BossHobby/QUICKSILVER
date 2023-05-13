# pylint: disable=missing-docstring,redefined-outer-name,unspecified-encoding

import os

Import("env")

try:
    from dulwich import porcelain
except ImportError:
    env.Execute("$PYTHONEXE -m pip install dulwich")
    from dulwich import porcelain


optimze_flags = [s for s in env.GetProjectOption("system_flags", "").splitlines() if s]

linker_flags = []

common_flags = [
    "-Wdouble-promotion",
    "-Wunsafe-loop-optimizations",
    "-fsingle-precision-constant",
    "-fno-exceptions",
    "-fno-strict-aliasing",
    "-fstack-usage",
    "-fno-stack-protector",
    "-fomit-frame-pointer",
    "-fno-unwind-tables",
    "-fno-asynchronous-unwind-tables",
    "-fno-math-errno",
    "-fmerge-all-constants",
    "-funsafe-loop-optimizations",
]

if env.GetBuildType() == "release":
    common_flags.insert(0, "-flto")
    common_flags.insert(0, "-O3")
    common_flags.insert(0, "-s")
else:
    common_flags.insert(0, "-O1")


git_version = porcelain.describe(".")
if git_version[0] == "g":
    git_version = git_version[1:]
print("Git Version", git_version)
try:
    git_branch = porcelain.active_branch(".").decode("utf-8")
    if git_branch == "master":
        git_branch = ""
except:
    git_branch = "unknown"
    pass
print("Git Branch", git_branch)

env.Append(
    GIT_VERSION=git_version,
    GIT_BRANCH=git_branch,
    BUILD_FLAGS=["-std=gnu11"],
    BUILD_UNFLAGS=["-Og", "-Os"],
    ASFLAGS=optimze_flags + common_flags,
    CCFLAGS=linker_flags + optimze_flags + common_flags,
    CPPDEFINES=[("TARGET_MCU", env["BOARD_MCU"]), ("GIT_VERSION", git_version)],
    LINKFLAGS=linker_flags + optimze_flags + common_flags,
)


def rewrite_source(localenv, node):
    dst = node.get_path()
    dst = dst.replace(localenv["PIOENV"], localenv["BOARD_MCU"])
    dst = os.path.splitext(dst)[0]
    return localenv.Object(source=node, target=dst)


env.AddBuildMiddleware(rewrite_source)
