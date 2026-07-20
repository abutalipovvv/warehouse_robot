from pathlib import Path


STATIC = Path(__file__).resolve().parents[1] / "operator_app" / "web" / "static"


def read(relative: str) -> str:
    return (STATIC / relative).read_text(encoding="utf-8")


def test_corridor_tool_is_a_previewed_rectangular_graph_selection() -> None:
    fleet_editor = read("js/app/operator-map-editor.js")
    standalone_editor = read("map-editor.js")
    scene_bridge = read("js/app/operator-scene.js")
    scene = read("scene3d.js")

    assert "beginFleetCorridorPointer(hit)" in fleet_editor
    assert "moveFleetCorridorPointer(hit)" in fleet_editor
    assert "endFleetCorridorPointer(hit)" in fleet_editor
    assert "markControlledCorridorArea(" in fleet_editor
    assert 'this.setFleetEditorAreaPreview("corridor"' in fleet_editor
    assert "handleFleetCorridorLm" not in fleet_editor
    assert "fleetCorridorStartLm" not in fleet_editor
    assert "areaPreview: this.fleetEditorAreaPreview" in scene_bridge

    assert "beginCorridorPointer(hit)" in standalone_editor
    assert "moveCorridorPointer(hit)" in standalone_editor
    assert "endCorridorPointer(hit)" in standalone_editor
    assert 'type: "corridor_rectangle"' in standalone_editor
    assert "markControlledCorridorArea(" in standalone_editor
    assert "handleCorridorToolClick" not in standalone_editor
    assert "corridorStartLm" not in standalone_editor
    assert "editor-area-preview" in scene


def test_corridor_area_uses_midpoints_and_splits_branched_graphs_into_chains() -> None:
    graph_tools = read("js/editor/graph-tools.js")

    assert "function edgeMidpoint(" in graph_tools
    assert "return pointInsideArea(edgeMidpoint(edge, byName), area);" in graph_tools
    assert "function selectedCorridorChains(" in graph_tools
    assert "selectedNeighbors > 2" in graph_tools
    assert "corridorChainId(chain.names" in graph_tools


def test_pencil_is_square_and_rectangle_has_live_preview_in_both_editors() -> None:
    occupancy = read("js/editor/occupancy-grid.js")
    fleet_editor = read("js/app/operator-map-editor.js")
    standalone_editor = read("map-editor.js")
    fleet_html = read("index.html")
    standalone_html = read("map-editor.html")

    assert "paintSquareLine(patch, from, to, size, value)" in occupancy
    assert "paintSquare(patch, centerX, centerY, size, value)" in occupancy
    assert "paintSquareLine(patch, point, point, size, value)" in fleet_editor
    assert "paintSquareLine(patch, point, point, size, value)" in standalone_editor
    assert 'this.setFleetEditorAreaPreview("rectangle"' in fleet_editor
    assert 'type: "raster_rectangle"' in standalone_editor
    assert '["raster_rectangle", "corridor_rectangle"].includes' in standalone_editor
    assert 'kind: this.dragState.type === "corridor_rectangle"' in standalone_editor
    for html in (fleet_html, standalone_html):
        assert "square pencil" in html
        assert "drag to preview" in html
        assert "Drag a rectangle around a narrow controlled corridor" in html


def test_undo_redo_history_covers_graph_and_raster_gestures() -> None:
    base = read("js/app/operator-base.js")
    fleet_editor = read("js/app/operator-map-editor.js")
    standalone_editor = read("map-editor.js")

    assert "fleetGraphSnapshot()" in fleet_editor
    assert "commitFleetGraphHistory(before" in fleet_editor
    assert 'undo: () => this.restoreFleetGraphSnapshot(before)' in fleet_editor
    assert 'redo: () => this.restoreFleetGraphSnapshot(after)' in fleet_editor
    assert "this.fleetRasterHistory.push(command)" in fleet_editor
    assert "event.ctrlKey || event.metaKey" in base
    assert 'key === "z" || key === "y"' in base

    assert "graphSnapshot()" in standalone_editor
    assert "commitGraphHistory(before" in standalone_editor
    assert 'undo: () => this.restoreGraphSnapshot(before)' in standalone_editor
    assert 'redo: () => this.restoreGraphSnapshot(after)' in standalone_editor
    assert "this.rasterHistory.push(command)" in standalone_editor
    assert "this.afterHistoryMutation(\"Map edit undone.\")" in standalone_editor
    assert "this.afterHistoryMutation(\"Map edit restored.\")" in standalone_editor
    assert '"corridor_rectangle",' in standalone_editor


def test_standalone_graph_mutations_are_recorded_as_atomic_commands() -> None:
    editor = read("map-editor.js")

    for label in (
        "Added landmark",
        "Moved landmark",
        "Updated curve",
        "Added graph edge chain.",
        "Removed landmark",
        "Removed edge",
        "Updated landmark",
        "Updated edge",
        "controlled corridor zone",
    ):
        assert label in editor
    assert "this.rasterHistory.clear();" in editor


def test_saved_corridors_remain_visually_highlighted() -> None:
    map_view = read("js/app/operator-map.js")
    scene = read("scene3d.js")
    standalone = read("map-editor.js")
    styles = read("styles.css")

    assert 'edge.properties?.controlled_region ? "controlled"' in map_view
    assert "editor-corridor-overlay" in scene
    assert 'controlled ? "#d97706"' in standalone
    assert ".graph-edge.editable.controlled" in styles
