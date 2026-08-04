import ast
from pathlib import Path

from operator_app.core.config import DEFAULT_CONFIG_PATH, DEFAULT_STATIC_DIR
from operator_app.core.map_cache import default_maps_cache_root
from operator_app.core.workspace import default_operator_data_root


ROOT = Path(__file__).resolve().parents[1]
OPERATOR_ROOT = ROOT / "operator_app"
STATIC_ROOT = OPERATOR_ROOT / "web" / "static"


def test_operator_app_has_compact_source_and_runtime_layout() -> None:
    assert DEFAULT_CONFIG_PATH == OPERATOR_ROOT / "config" / "config.yaml"
    assert DEFAULT_STATIC_DIR == STATIC_ROOT
    assert default_maps_cache_root() == ROOT / "var" / "operator_app" / "map_cache"
    assert default_operator_data_root() == ROOT / "var" / "operator_app" / "workspaces"

    for legacy_name in ("http", "services", "robot_grpc_api", "static", "tools"):
        assert not (OPERATOR_ROOT / legacy_name).exists()

    init_files = sorted(OPERATOR_ROOT.rglob("__init__.py"))
    assert init_files
    violations: list[tuple[Path, int, str]] = []
    for path in init_files:
        module = ast.parse(
            path.read_text(encoding="utf-8"),
            filename=str(path),
        )
        for statement in module.body:
            is_docstring = (
                isinstance(statement, ast.Expr)
                and isinstance(statement.value, ast.Constant)
                and isinstance(statement.value.value, str)
            )
            if not is_docstring:
                violations.append(
                    (
                        path.relative_to(OPERATOR_ROOT),
                        statement.lineno,
                        type(statement).__name__,
                    )
                )
    assert violations == []

    export_violations: list[tuple[Path, int, str]] = []
    for path in OPERATOR_ROOT.rglob("*.py"):
        module = ast.parse(
            path.read_text(encoding="utf-8"),
            filename=str(path),
        )
        for node in ast.walk(module):
            if isinstance(node, ast.ImportFrom) and any(
                alias.name == "*" for alias in node.names
            ):
                export_violations.append(
                    (path.relative_to(OPERATOR_ROOT), node.lineno, "*")
                )
            if isinstance(node, (ast.Assign, ast.AnnAssign)):
                targets = (
                    node.targets
                    if isinstance(node, ast.Assign)
                    else [node.target]
                )
                if any(
                    isinstance(target, ast.Name) and target.id == "__all__"
                    for target in targets
                ):
                    export_violations.append(
                        (
                            path.relative_to(OPERATOR_ROOT),
                            node.lineno,
                            "__all__",
                        )
                    )
    assert export_violations == []


def test_operator_frontend_is_modular_and_offline() -> None:
    app_js = (STATIC_ROOT / "app.js").read_text(encoding="utf-8")
    assert len(app_js.splitlines()) < 100

    module_root = STATIC_ROOT / "js" / "app"
    expected_modules = {
        "constants.js",
        "operator-actions.js",
        "operator-base.js",
        "operator-fleet.js",
        "operator-map-editor.js",
        "operator-map.js",
        "operator-realtime.js",
        "operator-scene.js",
        "robot-model-editor.js",
    }
    assert {path.name for path in module_root.glob("*.js")} == expected_modules
    scene_module = (module_root / "operator-scene.js").read_text(encoding="utf-8")
    assert 'import("../../scene3d.js")' in scene_module

    scene_js = (STATIC_ROOT / "scene3d.js").read_text(encoding="utf-8")
    assert 'new URL("./vendor/babylon-9.16.2.js", import.meta.url)' in scene_js
    assert "cdn.jsdelivr.net" not in scene_js
    assert (STATIC_ROOT / "vendor" / "babylon-9.16.2.js").stat().st_size > 1_000_000
    assert (STATIC_ROOT / "vendor" / "BABYLON-LICENSE.md").is_file()
