export const OCCUPANCY_WALL_LUMINANCE_MAX = 96;
export const OCCUPANCY_WALL_MAX_INSTANCES = 1500;
export const OCCUPANCY_WALL_TARGET_CELLS = 180000;

const WALL_STRIDE_STEPS = [1, 2, 3, 4, 6, 8, 12, 16, 24, 32];

function strideCandidates(width, height, targetCells) {
  const safeTarget = Math.max(1, Math.floor(Number(targetCells || 1)));
  const initial = Math.max(1, Math.ceil(Math.sqrt((width * height) / safeTarget)));
  return [
    initial,
    ...WALL_STRIDE_STEPS.filter((stride) => stride > initial),
  ];
}

function rectanglesForStride(
  imageData,
  stride,
  resolution,
  wallHeight,
  luminanceMax,
) {
  const width = imageData.width;
  const height = imageData.height;
  const pixels = imageData.data;
  const gridWidth = Math.ceil(width / stride);
  const gridHeight = Math.ceil(height / stride);
  const occupied = new Uint8Array(gridWidth * gridHeight);
  const luminanceSumMax = luminanceMax * 3;

  // Collapse the PGM into the target grid in one linear pass. Any occupied
  // source pixel marks its visual cell occupied, so thin walls do not vanish
  // when the map is downsampled.
  for (let y = 0; y < height; y += 1) {
    const targetRow = Math.floor(y / stride) * gridWidth;
    let sourceIndex = y * width * 4;
    for (let x = 0; x < width; x += 1, sourceIndex += 4) {
      if (
        pixels[sourceIndex + 3] >= 16
        && pixels[sourceIndex] + pixels[sourceIndex + 1] + pixels[sourceIndex + 2] <= luminanceSumMax
      ) {
        occupied[targetRow + Math.floor(x / stride)] = 1;
      }
    }
  }

  let active = new Map();
  const rectangles = [];
  for (let cellY = 0; cellY < gridHeight; cellY += 1) {
    const rowOffset = cellY * gridWidth;
    const nextActive = new Map();
    let runStart = -1;

    const acceptRun = (endX) => {
      if (runStart < 0) {
        return;
      }
      const runWidth = endX - runStart;
      const key = (runStart * (gridWidth + 1)) + runWidth;
      const rectangle = active.get(key) || {
        x: runStart,
        y: cellY,
        width: runWidth,
        height: 0,
      };
      rectangle.height += 1;
      nextActive.set(key, rectangle);
      active.delete(key);
      runStart = -1;
    };

    for (let cellX = 0; cellX < gridWidth; cellX += 1) {
      if (occupied[rowOffset + cellX]) {
        if (runStart < 0) {
          runStart = cellX;
        }
      } else {
        acceptRun(cellX);
      }
    }
    acceptRun(gridWidth);
    rectangles.push(...active.values());
    active = nextActive;
  }
  rectangles.push(...active.values());

  return rectangles.map((rectangle) => {
    const pixelX = rectangle.x * stride;
    const pixelY = rectangle.y * stride;
    const pixelWidth = Math.min(width - pixelX, rectangle.width * stride);
    const pixelHeight = Math.min(height - pixelY, rectangle.height * stride);
    return {
      x: (pixelX + (pixelWidth / 2)) * resolution,
      z: (pixelY + (pixelHeight / 2)) * resolution,
      width: pixelWidth * resolution,
      depth: pixelHeight * resolution,
      height: wallHeight,
      stride,
    };
  });
}

export function occupancyWallRectanglesFromImageData(
  imageData,
  resolution = 1,
  wallHeight = 1.8,
  options = {},
) {
  const width = Math.max(0, Math.floor(Number(imageData?.width || 0)));
  const height = Math.max(0, Math.floor(Number(imageData?.height || 0)));
  const pixels = imageData?.data;
  if (!width || !height || !pixels || pixels.length < width * height * 4) {
    return [];
  }

  const mapResolution = Math.max(0.000001, Number(resolution || 1));
  const verticalHeight = Math.max(0.05, Number(wallHeight || 1.8));
  const maximumInstances = Math.max(
    1,
    Math.floor(Number(options.maximumInstances || OCCUPANCY_WALL_MAX_INSTANCES)),
  );
  const targetCells = Math.max(
    1,
    Math.floor(Number(options.targetCells || OCCUPANCY_WALL_TARGET_CELLS)),
  );
  const luminanceMax = Math.max(
    0,
    Math.min(255, Number(options.luminanceMax ?? OCCUPANCY_WALL_LUMINANCE_MAX)),
  );
  const strides = strideCandidates(width, height, targetCells);

  for (let index = 0; index < strides.length; index += 1) {
    const rectangles = rectanglesForStride(
      { width, height, data: pixels },
      strides[index],
      mapResolution,
      verticalHeight,
      luminanceMax,
    );
    if (rectangles.length <= maximumInstances || index === strides.length - 1) {
      return rectangles;
    }
  }
  return [];
}
