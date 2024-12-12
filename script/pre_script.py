# pylint: disable=missing-docstring,redefined-outer-name,unspecified-encoding

import os
import threading

Import("env")

try:
    from dulwich import porcelain
except ImportError:
    env.Execute("$PYTHONEXE -m pip install dulwich")
    from dulwich import porcelain

system_flags = [s for s in env.GetProjectOption("system_flags", "").splitlines() if s]

common_flags = [
    "-std=gnu11",
    "-Wdouble-promotion",
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
]

if env.GetBuildType() == "release":
    common_flags.insert(0, "-flto=auto")
    common_flags.insert(0, "-Ofast")

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
    BUILD_UNFLAGS=["-Og", "-Os", "-DDEBUG"],
    ASFLAGS=system_flags + common_flags,
    CCFLAGS=system_flags + common_flags,
    CPPDEFINES=[("GIT_VERSION", git_version)],
    LINKFLAGS=system_flags + common_flags,
)


def rewrite_source(localenv, node):
    dst = node.get_path()
    dst = dst.replace(localenv["PIOENV"], localenv["BOARD_MCU"])
    dst = os.path.splitext(dst)[0]
    return localenv.Object(source=node, target=dst)


env.AddBuildMiddleware(rewrite_source)


def touch(fname, times=None):
    with open(fname, "a"):
        os.utime(fname, times)


def fetch_thread():
    target_remote = "https://github.com/BossHobby/Targets.git"
    target_dir = os.path.join(env["PROJECT_DIR"], "targets")

    if not os.path.exists(target_dir):
        porcelain.clone(target_remote, target=target_dir, branch="targets")
        touch(os.path.join(env["PROJECT_DIR"], "platformio.ini"))
    else:
        target_repo = porcelain.open_repo(target_dir)
        target_ref_before = porcelain.describe(target_repo)
        update = porcelain.fetch(target_repo, target_remote, b"refs/heads/targets")
        porcelain.checkout_branch(target_repo, update[b"refs/heads/targets"])
        target_ref_after = porcelain.describe(target_repo)

        if target_ref_before != target_ref_after:
            touch(os.path.join(env["PROJECT_DIR"], "platformio.ini"))


t = threading.Thread(target=fetch_thread)
t.start()
