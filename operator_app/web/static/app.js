import { OperatorAppBase } from "./js/app/operator-base.js";
import { withRealtime } from "./js/app/operator-realtime.js";
import { withFleetUi } from "./js/app/operator-fleet.js";
import { withMapView } from "./js/app/operator-map.js";
import { withMapEditor } from "./js/app/operator-map-editor.js";
import { withSceneNavigation } from "./js/app/operator-scene.js";
import { withActions } from "./js/app/operator-actions.js";

export const OperatorApp = withActions(
  withSceneNavigation(
    withMapEditor(
      withMapView(
        withFleetUi(
          withRealtime(OperatorAppBase),
        ),
      ),
    ),
  ),
);

window.addEventListener("DOMContentLoaded", () => {
  const app = new OperatorApp();
  app.init().catch((error) => {
    window.alert(error.message || String(error));
  });
});
