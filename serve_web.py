#!/usr/bin/env python3
import sys

from operator_app.server import main


if __name__ == "__main__":
    if not any(arg == "--port" or arg.startswith("--port=") for arg in sys.argv[1:]):
        sys.argv.extend(["--port", "8090"])
    main()
