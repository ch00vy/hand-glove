"""Run the installed SDK Wi-Fi CLI, supplying its getpass prompt over stdin.

The SDK is left untouched; credentials are never passed as command arguments.
"""

import getpass
import runpy
import sys
from pathlib import Path


def read_password(prompt="Password: ", stream=None):
    value = sys.stdin.readline()
    if not value:
        raise RuntimeError("Password tidak diterima dari GUI")
    return value.rstrip("\r\n")


def main():
    script = Path(sys.argv[1]).resolve()
    sys.argv = [str(script), *sys.argv[2:]]
    sys.path.insert(0, str(script.parent))
    sys.dont_write_bytecode = True
    getpass.getpass = read_password
    runpy.run_path(str(script), run_name="__main__")


if __name__ == "__main__":
    main()
