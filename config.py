"""Defaults and the external Rokoko SDK location (no SDK files are copied)."""

import os
from pathlib import Path

DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 12277
CONNECT_TIMEOUT = 3.0
READ_TIMEOUT = 0.25
NO_DATA_TIMEOUT = 3.0

SDK_HOME = Path(
    os.environ.get("ROKOKO_SDK_HOME")
    or Path(os.environ.get("LOCALAPPDATA", str(Path.home() / "AppData/Local")))
    / "Rokoko Device SDK"
).expanduser()
SDK_PYTHON = SDK_HOME / "examples" / "python"
