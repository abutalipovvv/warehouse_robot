from __future__ import annotations

import json
import shutil
from pathlib import Path

from .models import DemoPayload


class RouteDemoSiteBuilder:
    def __init__(self, template_dir: Path | None = None) -> None:
        self.template_dir = template_dir or (Path(__file__).resolve().parent / "web")

    def build(
        self,
        payload: DemoPayload,
        map_dir: Path,
        output: Path | None = None,
    ) -> Path:
        output_dir = self._resolve_output_dir(map_dir=map_dir, output=output)
        output_dir.mkdir(parents=True, exist_ok=True)

        for asset_name in ("index.html", "styles.css", "app.js"):
            shutil.copyfile(self.template_dir / asset_name, output_dir / asset_name)

        payload_json = json.dumps(payload.to_dict(), ensure_ascii=False).replace(
            "</script>",
            "<\\/script>",
        )
        (output_dir / "demo-data.js").write_text(
            f"window.ROUTE_DEMO_DATA = {payload_json};\n",
            encoding="utf-8",
        )
        return output_dir / "index.html"

    def _resolve_output_dir(self, map_dir: Path, output: Path | None) -> Path:
        if output is None:
            return map_dir / "route_demo_web"

        resolved = output.resolve()
        if resolved.suffix:
            return resolved.parent / resolved.stem
        return resolved
