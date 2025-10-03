# Inject git hash into build defines for KC1
# PlatformIO extra script.
# Adds -DKC1_GIT_HASH=\"<short_hash>\" automatically if git available.
# Falls back to 'nogit' if repository or git is unavailable.

import subprocess
from SCons.Script import ARGUMENTS, DefaultEnvironment  # type: ignore

env = DefaultEnvironment()


def short_git_hash():
    try:
        # --short ensures compact; strip to be safe
        h = (
            subprocess.check_output(
                ["git", "rev-parse", "--short", "HEAD"], stderr=subprocess.DEVNULL
            )
            .decode()
            .strip()
        )
        # Basic sanitization: ensure only hex-ish chars
        if not h:
            return "nogit"
        return h[:16]
    except Exception:
        return "nogit"


hash_val = short_git_hash()
print("KC1: injecting git hash:", hash_val)

# Append define only if not already provided externally
if not any("KC1_GIT_HASH" in f for f in env.get("CPPDEFINES", [])):
    env.Append(CPPDEFINES=[("KC1_GIT_HASH", '"' + hash_val + '"')])
