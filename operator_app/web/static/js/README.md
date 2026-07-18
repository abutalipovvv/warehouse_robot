# Operator web modules

The browser code is split by responsibility:

- `api/http-client.js` owns JSON transport and normalized HTTP errors.
- `state/preferences.js` owns persistent UI preferences and their key prefix.
- `shared/json.js` contains serialization-safe helpers shared by both pages.
- `editor/command-stack.js` owns bounded undo/redo history.
- `editor/graph-tools.js` owns reusable graph-path and controlled-corridor edits.
- `editor/occupancy-grid.js` owns raster decoding, cell edits, dirty-region
  rendering and the `gray8` save contract.
- `app/operator-base.js` owns application startup, routing and DOM binding.
- `app/operator-realtime.js` owns WebSocket streams and live robot animation.
- `app/operator-fleet.js` owns Fleet Manager state and fleet workspace rendering.
- `app/operator-map.js` owns the 2D map projection and interaction layers.
- `app/operator-map-editor.js` owns raster, LM and graph editing.
- `app/operator-scene.js` owns Babylon view composition and navigation actions.
- `app/operator-actions.js` owns parameters, teleop, map transfer and dialogs.
- `app/constants.js` and `app/robot-model-editor.js` hold their focused UI models.

`app.js` is only the application composition root for the operator workspace.
`map-editor.js` composes Babylon picking, graph/LM editing and the modules
above. Rendering primitives stay in `scene3d.js`. Babylon.js is vendored under
`vendor/`, so starting and using the browser UI does not require internet access.

New features should put reusable state or algorithms under `js/` and keep DOM
binding in the page composition roots. API calls should go through
`http-client.js`; local-storage keys should go through `preferences.js`.
