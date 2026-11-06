"""Threaded, receive-only adapter for the external official RGMP v2 parser."""

from __future__ import annotations

import importlib
import ipaddress
import queue
import re
import socket
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Callable

import config


def validate_endpoint(host: str, port: str | int) -> tuple[str, int]:
    host = host.strip()
    if not host:
        raise ValueError("Host tidak boleh kosong.")
    try:
        ipaddress.ip_address(host)
    except ValueError:
        try:
            ascii_host = host.encode("idna").decode("ascii").rstrip(".")
        except UnicodeError as exc:
            raise ValueError("Host/IP tidak valid.") from exc
        labels = ascii_host.split(".")
        if (len(ascii_host) > 253 or all(c in "0123456789." for c in ascii_host)
                or any(not re.fullmatch(r"[A-Za-z0-9](?:[A-Za-z0-9-]{0,61}[A-Za-z0-9])?", s)
                       for s in labels)):
            raise ValueError("Gunakan alamat IP atau hostname, tanpa URL atau spasi.")
        host = ascii_host
    if not re.fullmatch(r"[0-9]{1,5}", str(port).strip()):
        raise ValueError("Port harus berupa bilangan 1–65535.")
    number = int(port)
    if not 1 <= number <= 65535:
        raise ValueError("Port harus berada dalam rentang 1–65535.")
    return host, number


def load_sdk():
    """Import only from the configured SDK, without writing SDK bytecode."""
    required = ("client.py", "frame.py", "layout.py", "definition.py")
    if not all((config.SDK_PYTHON / "rgmp_core" / name).is_file() for name in required):
        raise RuntimeError(
            f"Parser rgmp_core tidak ditemukan di {config.SDK_PYTHON}. "
            "Set ROKOKO_SDK_HOME ke folder instalasi SDK."
        )
    sdk_path = str(config.SDK_PYTHON)
    sys.path.insert(0, sdk_path)
    previous = sys.dont_write_bytecode
    sys.dont_write_bytecode = True
    try:
        client = importlib.import_module("rgmp_core.client")
        frame = importlib.import_module("rgmp_core.frame")
        layout = importlib.import_module("rgmp_core.layout")
        for module in (client, frame, layout):
            if not Path(module.__file__).resolve().is_relative_to(
                config.SDK_PYTHON.resolve()
            ):
                raise RuntimeError("rgmp_core sudah dimuat dari lokasi SDK berbeda; restart aplikasi.")
        return client, frame, layout
    finally:
        sys.dont_write_bytecode = previous
        sys.path.remove(sdk_path)


class RGMPClient:
    """One connection per instance. Events are consumed by the Tk main thread.

    Optional on_data(message) receives every decoded frame on the worker thread
    for future retargeting. It must return promptly and must not call Tkinter.
    GUI telemetry is sampled; the data counter still counts every valid frame.
    """

    def __init__(self, on_data: Callable[[dict], None] | None = None):
        self.events: queue.Queue = queue.Queue(maxsize=256)
        self.on_data = on_data
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._state = "Disconnected"
        self._count = 0
        self._last_data = 0.0
        self._last_publish = 0.0
        self._last_log = 0.0
        self._latest: dict | None = None

    @property
    def alive(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def connect(self, host: str, port: str | int) -> None:
        if self._thread is not None:
            raise RuntimeError("Buat client baru untuk koneksi berikutnya.")
        host, port = validate_endpoint(host, port)
        self._thread = threading.Thread(target=self._run, args=(host, port), daemon=True)
        self._thread.start()

    def disconnect(self) -> None:
        # Only signals our reader. All socket operations stay on the worker.
        self._stop.set()

    def _emit(self, kind: str, value) -> None:
        try:
            self.events.put_nowait((kind, value))
        except queue.Full:
            # Bound memory even if the UI is temporarily busy.
            try:
                self.events.get_nowait()
            except queue.Empty:
                pass
            self.events.put_nowait((kind, value))

    def _status(self, state: str) -> None:
        if state != self._state:
            self._state = state
            self._emit("status", state)

    def _tick(self) -> None:
        now = time.monotonic()
        if now - self._last_data >= config.NO_DATA_TIMEOUT:
            self._status("Connected — No Data")
        if self._latest is not None and now - self._last_publish >= 0.1:
            self._emit("data", self._latest)
            self._last_publish = now

    def _run(self, host: str, port: int) -> None:
        reader = None
        self._status("Connecting")
        self._emit("log", f"Menghubungkan ke {host}:{port} …")
        try:
            sdk, framing, decoding = load_sdk()
            owner = self

            class Reader(sdk.RgmpClient):
                def _recv_exact(self, n: int) -> bytes:
                    # SDK read_frame calls this for header/payload. Keep partial
                    # bytes across idle timeouts; never restart a partial frame.
                    buf = bytearray()
                    while len(buf) < n:
                        if owner._stop.is_set():
                            raise ConnectionError("Reader stopped")
                        owner._tick()
                        try:
                            chunk = self._socket.recv(n - len(buf))
                        except socket.timeout:
                            continue
                        if not chunk:
                            if buf:
                                raise framing.ProtocolError("Server menutup koneksi di tengah frame RGMP.")
                            raise ConnectionError("Server closed connection")
                        buf.extend(chunk)
                    return bytes(buf)

            reader = Reader(host, port, timeout=config.CONNECT_TIMEOUT)

            def definition_received(definition, layout):
                self._emit("log", f"Definition: device={definition['device_id']}, "
                           f"groups={len(layout.groups)}")

            def data_received(device_id, group_id, timestamp_us, payload, layout):
                slots = layout.groups.get(group_id)
                if not slots:
                    raise framing.ProtocolError(f"Grup RGMP tidak dikenal/kosong: {group_id}")
                expected = sum(slot.byte_size for slot in slots)
                if len(payload) != expected:
                    raise framing.ProtocolError(
                        f"Ukuran data grup {group_id}: {len(payload)}, seharusnya {expected}."
                    )
                streams = [dict(metadata=dict(slot.entry or {}), values=decoding.read_slot(slot, payload))
                           for slot in slots]
                now = time.monotonic()
                self._count += 1
                self._last_data = now
                message = dict(count=self._count, received_at=datetime.now().astimezone().isoformat(timespec="milliseconds"),
                               device_id=device_id, group_id=group_id,
                               group_name=layout.group_names[group_id], timestamp_us=timestamp_us,
                               streams=streams)
                self._latest = message
                self._status("Streaming")
                self._tick()
                if now - self._last_log >= 1.0:
                    first = streams[0]
                    label = first['metadata'].get('target_frame') or first['metadata'].get('custom_label', '?')
                    self._emit("log", f"RGMP #{self._count}: device={device_id}, group={group_id}, "
                               f"ts={timestamp_us} µs, {label}={first['values'][:4]}")
                    self._last_log = now
                if self.on_data is not None:
                    self.on_data(message)

            def parser_error(error):
                raise error

            reader.on_definition = definition_received
            reader.on_data = data_received
            reader.on_error = parser_error
            reader.on_disconnect = lambda dev: self._emit("log", f"Perangkat {dev} terputus dari server.")
            reader.on_unknown = lambda prefix, payload: self._emit("log", f"Frame RGMP tipe {prefix} dilewati.")
            reader.on_undecodable = lambda definition, error: self._emit(
                "log", f"Device {definition['device_id']} tidak dapat didekode: {error}"
            )
            if self._stop.is_set():
                return
            with reader:
                reader._socket.settimeout(config.READ_TIMEOUT)
                if self._stop.is_set():
                    return
                self._last_data = time.monotonic()
                self._status("Connected")
                reader.run()
                if not self._stop.is_set():
                    raise ConnectionError("Koneksi ke server RGMP terputus.")
        except Exception as exc:
            if not self._stop.is_set():
                self._status("Error")
                self._emit("log", f"{type(exc).__name__}: {exc}")
        finally:
            if reader is not None:
                reader.close()
            if self._latest is not None:
                self._emit("data", self._latest)
            if self._stop.is_set():
                self._status("Disconnected")
                self._emit("log", "Koneksi pembaca GUI ditutup.")
            self._emit("done", None)
