from __future__ import annotations

import sys
from pathlib import Path


def main() -> None:
    package_root = Path(__file__).resolve().parent / "robot" / "ws" / "src" / "robot_http_api"
    sys.path.insert(0, str(package_root))
    from robot_http_api.stub_server import main as stub_main

    stub_main()


if __name__ == "__main__":
    main()
