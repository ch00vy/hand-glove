"""Smartgloves Connector — Python 3.12 / Tkinter desktop entry point."""

import queue
import tkinter as tk
from datetime import datetime
from tkinter import ttk
from tkinter.scrolledtext import ScrolledText

from config import DEFAULT_HOST, DEFAULT_PORT
from rgmp_client import RGMPClient, validate_endpoint


class SmartglovesConnector(ttk.Frame):
    def __init__(self, root: tk.Tk):
        super().__init__(root, padding=20)
        self.root = root
        self.client = None
        self.closing = False
        self.stopping = False
        root.title("Smartgloves Connector")
        root.geometry("780x500")
        root.minsize(660, 400)
        self.pack(fill="both", expand=True)
        self.columnconfigure(0, weight=1)
        self.rowconfigure(4, weight=1)
        ttk.Label(self, text="Smartgloves Connector", font=("Segoe UI", 19, "bold")).grid(
            row=0, column=0, sticky="w", pady=(0, 18))
        controls = ttk.Frame(self)
        controls.grid(row=1, column=0, sticky="ew")
        controls.columnconfigure(1, weight=1)
        ttk.Label(controls, text="Host").grid(row=0, column=0, padx=(0, 8))
        self.host = ttk.Entry(controls)
        self.host.insert(0, DEFAULT_HOST)
        self.host.grid(row=0, column=1, sticky="ew")
        ttk.Label(controls, text="Port").grid(row=0, column=2, padx=(14, 8))
        self.port = ttk.Entry(controls, width=8)
        self.port.insert(0, str(DEFAULT_PORT))
        self.port.grid(row=0, column=3)
        self.connect_button = ttk.Button(controls, text="Connect", command=self.connect)
        self.connect_button.grid(row=0, column=4, padx=(14, 8))
        self.disconnect_button = ttk.Button(controls, text="Disconnect", command=self.disconnect, state="disabled")
        self.disconnect_button.grid(row=0, column=5)
        self.status = tk.StringVar(value="Disconnected")
        self.status_label = ttk.Label(self, textvariable=self.status, font=("Segoe UI", 11, "bold"))
        self.status_label.grid(row=2, column=0, sticky="w", pady=(16, 8))
        ttk.Label(self, text="Connection Log").grid(row=3, column=0, sticky="w", pady=(4, 6))
        self.log = ScrolledText(self, height=12, state="disabled", wrap="word", font=("Consolas", 10))
        self.log.grid(row=4, column=0, sticky="nsew")
        self.counter = tk.StringVar(value="Pesan data RGMP: 0")
        self.timestamp = tk.StringVar(value="Terakhir diterima: — | Timestamp RGMP: —")
        ttk.Label(self, textvariable=self.counter).grid(row=5, column=0, sticky="w", pady=(12, 4))
        ttk.Label(self, textvariable=self.timestamp).grid(row=6, column=0, sticky="w")
        self.append_log("Siap. Jalankan SDK Driver / Hand Solver di terminal terpisah.")
        root.protocol("WM_DELETE_WINDOW", self.close)
        root.after(50, self.poll)

    def append_log(self, message):
        self.log.configure(state="normal")
        self.log.insert("end", f"[{datetime.now():%H:%M:%S}] {message}\n")
        if int(self.log.index("end-1c").split(".")[0]) > 500:
            self.log.delete("1.0", "2.0")
        self.log.see("end")
        self.log.configure(state="disabled")

    def set_status(self, state):
        self.status.set(state)
        color = {"Streaming": "#167344", "Error": "#b42318", "Connecting": "#966500",
                 "Connected — No Data": "#966500", "Connected": "#1769aa"}.get(state, "#555555")
        self.status_label.configure(foreground=color)

    def connect(self):
        if self.client is not None and self.client.alive:
            return
        try:
            host, port = validate_endpoint(self.host.get(), self.port.get())
        except ValueError as exc:
            self.set_status("Error")
            self.append_log(str(exc))
            return
        self.stopping = False
        self.counter.set("Pesan data RGMP: 0")
        self.timestamp.set("Terakhir diterima: — | Timestamp RGMP: —")
        self.client = RGMPClient()
        self.host.configure(state="disabled")
        self.port.configure(state="disabled")
        self.connect_button.configure(state="disabled")
        self.disconnect_button.configure(state="normal")
        self.set_status("Connecting")
        self.client.connect(host, port)

    def disconnect(self):
        if self.client is not None:
            self.stopping = True
            self.client.disconnect()
            self.disconnect_button.configure(state="disabled")
            self.append_log("Menutup pembaca GUI …")

    def poll(self):
        if self.client is not None:
            for _ in range(256):
                try:
                    kind, value = self.client.events.get_nowait()
                except queue.Empty:
                    break
                if kind == "status":
                    if not self.stopping or value == "Disconnected":
                        self.set_status(value)
                        self.append_log(value)
                elif kind == "log":
                    self.append_log(value)
                elif kind == "data":
                    self.counter.set(f"Pesan data RGMP: {value['count']}")
                    self.timestamp.set(f"Terakhir diterima: {value['received_at']} | "
                                       f"Timestamp RGMP: {value['timestamp_us']} µs")
            if not self.client.alive:
                self.host.configure(state="normal")
                self.port.configure(state="normal")
                self.connect_button.configure(state="normal")
                self.disconnect_button.configure(state="disabled")
        if self.closing and (self.client is None or not self.client.alive):
            self.root.destroy()
            return
        self.root.after(50, self.poll)

    def close(self):
        if self.closing:
            return
        self.closing = True
        self.connect_button.configure(state="disabled")
        self.disconnect()
        self.append_log("Menunggu worker selesai sebelum menutup aplikasi …")


if __name__ == "__main__":
    root = tk.Tk()
    SmartglovesConnector(root)
    root.mainloop()
