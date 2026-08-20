import { occupancyWallRectanglesFromImageData } from "./occupancy-walls.js";

self.addEventListener("message", (event) => {
  const payload = event.data || {};
  try {
    const pixels = new Uint8ClampedArray(payload.pixels);
    const walls = occupancyWallRectanglesFromImageData(
      {
        width: Number(payload.width || 0),
        height: Number(payload.height || 0),
        data: pixels,
      },
      Number(payload.resolution || 1),
      Number(payload.wallHeight || 1.8),
    );
    self.postMessage({ ok: true, walls });
  } catch (error) {
    self.postMessage({
      ok: false,
      error: String(error?.message || error || "occupancy wall generation failed"),
    });
  }
});
