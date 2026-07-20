"""Manual headless-Chrome smoke test for Fleet Manager corridor rendering."""

from __future__ import annotations

import base64
import json
import os
import socket
import struct
import subprocess
import sys
import tempfile
import time
import urllib.request
from urllib.parse import urlparse


class CdpSocket:
    def __init__(self, url: str) -> None:
        parsed = urlparse(url)
        self.socket = socket.create_connection((parsed.hostname, parsed.port), timeout=10)
        self.socket.settimeout(60)
        key = base64.b64encode(os.urandom(16)).decode("ascii")
        request = (
            f"GET {parsed.path} HTTP/1.1\r\n"
            f"Host: {parsed.hostname}:{parsed.port}\r\n"
            "Upgrade: websocket\r\n"
            "Connection: Upgrade\r\n"
            f"Sec-WebSocket-Key: {key}\r\n"
            "Sec-WebSocket-Version: 13\r\n\r\n"
        )
        self.socket.sendall(request.encode("ascii"))
        response = b""
        while b"\r\n\r\n" not in response:
            response += self.socket.recv(4096)
        if b" 101 " not in response.split(b"\r\n", 1)[0]:
            raise RuntimeError(response.decode("utf-8", errors="replace"))
        self.sequence = 0

    def call(self, method: str, params: dict[str, object] | None = None) -> dict[str, object]:
        self.sequence += 1
        message_id = self.sequence
        self._send_json({"id": message_id, "method": method, "params": params or {}})
        while True:
            message = self._receive_json()
            if message.get("id") == message_id:
                if "error" in message:
                    raise RuntimeError(str(message["error"]))
                return dict(message.get("result") or {})

    def _send_json(self, payload: dict[str, object]) -> None:
        data = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        mask = os.urandom(4)
        length = len(data)
        header = bytearray([0x81])
        if length < 126:
            header.append(0x80 | length)
        elif length < 65536:
            header.append(0x80 | 126)
            header.extend(struct.pack("!H", length))
        else:
            header.append(0x80 | 127)
            header.extend(struct.pack("!Q", length))
        header.extend(mask)
        header.extend(byte ^ mask[index % 4] for index, byte in enumerate(data))
        self.socket.sendall(header)

    def _receive_json(self) -> dict[str, object]:
        fragments = bytearray()
        while True:
            first, second = self._read_exact(2)
            final = bool(first & 0x80)
            opcode = first & 0x0F
            length = second & 0x7F
            if length == 126:
                length = struct.unpack("!H", self._read_exact(2))[0]
            elif length == 127:
                length = struct.unpack("!Q", self._read_exact(8))[0]
            if second & 0x80:
                mask = self._read_exact(4)
            else:
                mask = b""
            payload = self._read_exact(length)
            if mask:
                payload = bytes(
                    byte ^ mask[index % 4]
                    for index, byte in enumerate(payload)
                )
            if opcode == 0x9:
                continue
            if opcode == 0x8:
                raise RuntimeError("Chrome closed the DevTools socket")
            if opcode == 0x1:
                fragments = bytearray(payload)
            elif opcode == 0x0 and fragments:
                fragments.extend(payload)
            else:
                continue
            if final:
                return dict(json.loads(bytes(fragments).decode("utf-8")))

    def _read_exact(self, length: int) -> bytes:
        chunks = bytearray()
        while len(chunks) < length:
            chunk = self.socket.recv(length - len(chunks))
            if not chunk:
                raise RuntimeError("DevTools socket closed")
            chunks.extend(chunk)
        return bytes(chunks)


def evaluate(
    cdp: CdpSocket,
    expression: str,
    *,
    await_promise: bool = False,
) -> object:
    result = cdp.call(
        "Runtime.evaluate",
        {
            "expression": expression,
            "awaitPromise": await_promise,
            "returnByValue": True,
        },
    )
    if result.get("exceptionDetails"):
        raise RuntimeError(str(result["exceptionDetails"]))
    remote = dict(result.get("result") or {})
    if remote.get("subtype") == "error":
        raise RuntimeError(str(remote.get("description") or remote))
    return remote.get("value")


def main() -> int:
    port = 9334
    profile = tempfile.mkdtemp(prefix="fleet-browser-smoke-")
    chrome = subprocess.Popen(
        [
            "google-chrome",
            "--headless=new",
            "--no-sandbox",
            "--disable-dev-shm-usage",
            "--use-angle=swiftshader",
            "--enable-webgl",
            "--ignore-gpu-blocklist",
            "--remote-allow-origins=*",
            f"--remote-debugging-port={port}",
            f"--user-data-dir={profile}",
            "about:blank",
        ],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        targets: list[dict[str, object]] = []
        for _ in range(100):
            try:
                targets = json.load(
                    urllib.request.urlopen(
                        f"http://127.0.0.1:{port}/json/list",
                        timeout=1,
                    )
                )
                if targets:
                    break
            except OSError:
                time.sleep(0.1)
        if not targets:
            raise RuntimeError("Chrome DevTools did not start")
        target = next(
            (
                item
                for item in targets
                if item.get("type") == "page"
                and str(item.get("url") or "") == "about:blank"
            ),
            targets[0],
        )
        cdp = CdpSocket(str(target["webSocketDebuggerUrl"]))
        cdp.call("Page.enable")
        cdp.call("Runtime.enable")
        cdp.call("Page.navigate", {"url": "http://127.0.0.1:8780/robot"})
        time.sleep(3)
        evaluate(
            cdp,
            """
            localStorage.setItem("operator:selectedRobotId", "__fleet_manager_sim__");
            localStorage.setItem("operator:lastFleetManagerId", "__fleet_manager_sim__");
            location.href = "/map_editor";
            true;
            """,
        )
        time.sleep(12)
        state = evaluate(
            cdp,
            """
            (() => {
              const scene = document.getElementById("operatorScene3d");
              return {
                path: location.pathname,
                managerId: scene?.dataset?.managerId || "",
                mapName: scene?.dataset?.mapName || "",
                canvasCount: scene?.querySelectorAll("canvas").length || 0,
                mapEditorActive: document.getElementById("mapEditorNavButton")?.classList.contains("primary") || false,
                selectedWorkspace: document.querySelector(".robot-workspace-name")?.textContent?.trim() || "",
              };
            })()
            """,
        )
        print(json.dumps(state, ensure_ascii=False, sort_keys=True))
        expected = {
            "path": "/map_editor",
            "managerId": "__fleet_manager_sim__",
            "mapName": "smart_kiva_large_w_mode",
            "mapEditorActive": True,
        }
        for key, value in expected.items():
            if not isinstance(state, dict) or state.get(key) != value:
                raise AssertionError(f"{key}: expected {value!r}, got {state!r}")
        if int(state.get("canvasCount", 0)) < 1:
            raise AssertionError(f"Babylon canvas is missing: {state!r}")
        return 0
    finally:
        chrome.terminate()
        try:
            chrome.wait(timeout=5)
        except subprocess.TimeoutExpired:
            chrome.kill()


if __name__ == "__main__":
    sys.exit(main())
