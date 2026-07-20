const PIXEL_FREE = 254;
const PIXEL_UNKNOWN = 205;
const PIXEL_OCCUPIED = 0;

function clamp(value, minimum, maximum) {
  return Math.max(minimum, Math.min(maximum, value));
}

function bytesToBase64(bytes) {
  const chunkSize = 0x8000;
  let binary = "";
  for (let offset = 0; offset < bytes.length; offset += chunkSize) {
    binary += String.fromCharCode(...bytes.subarray(offset, offset + chunkSize));
  }
  return window.btoa(binary);
}

export const OCCUPANCY_VALUES = Object.freeze({
  occupied: PIXEL_OCCUPIED,
  unknown: PIXEL_UNKNOWN,
  free: PIXEL_FREE,
});

export class OccupancyGrid {
  constructor(width, height, pixels) {
    this.width = Math.max(1, Math.floor(Number(width) || 1));
    this.height = Math.max(1, Math.floor(Number(height) || 1));
    const expected = this.width * this.height;
    if (!(pixels instanceof Uint8Array) || pixels.length !== expected) {
      throw new Error(`Occupancy raster must contain ${expected} pixels.`);
    }
    this.pixels = pixels;
    this.canvas = document.createElement("canvas");
    this.canvas.width = this.width;
    this.canvas.height = this.height;
    this.context = this.canvas.getContext("2d", { alpha: false, willReadFrequently: false });
    if (!this.context) {
      throw new Error("Canvas 2D is unavailable.");
    }
    this.imageData = this.context.createImageData(this.width, this.height);
    this.renderAll();
  }

  static async fromImageDataUrl(imageDataUrl, width, height) {
    const image = await new Promise((resolve, reject) => {
      const element = new Image();
      element.onload = () => resolve(element);
      element.onerror = () => reject(new Error("Unable to decode the occupancy map image."));
      element.src = String(imageDataUrl || "");
    });
    const rasterWidth = Math.max(1, Math.floor(Number(width) || image.naturalWidth || 1));
    const rasterHeight = Math.max(1, Math.floor(Number(height) || image.naturalHeight || 1));
    const canvas = document.createElement("canvas");
    canvas.width = rasterWidth;
    canvas.height = rasterHeight;
    const context = canvas.getContext("2d", { alpha: false, willReadFrequently: true });
    if (!context) {
      throw new Error("Canvas 2D is unavailable.");
    }
    context.imageSmoothingEnabled = false;
    context.drawImage(image, 0, 0, rasterWidth, rasterHeight);
    const rgba = context.getImageData(0, 0, rasterWidth, rasterHeight).data;
    const pixels = new Uint8Array(rasterWidth * rasterHeight);
    for (let index = 0; index < pixels.length; index += 1) {
      const offset = index * 4;
      pixels[index] = Math.round(
        (rgba[offset] * 0.299)
        + (rgba[offset + 1] * 0.587)
        + (rgba[offset + 2] * 0.114),
      );
    }
    return new OccupancyGrid(rasterWidth, rasterHeight, pixels);
  }

  beginPatch(label) {
    return {
      label: String(label || "Raster edit"),
      before: new Map(),
      after: new Map(),
      dirty: new Set(),
      bulkIndices: null,
      bulkBeforeValue: null,
      bulkAfterValue: null,
    };
  }

  paintLine(patch, from, to, radius, value) {
    const start = this.normalizePoint(from);
    const goal = this.normalizePoint(to);
    const brushRadius = clamp(Math.floor(Number(radius) || 1), 1, 64);
    const pixelValue = clamp(Math.round(Number(value) || 0), 0, 255);
    let x = start.x;
    let y = start.y;
    const dx = Math.abs(goal.x - start.x);
    const sx = start.x < goal.x ? 1 : -1;
    const dy = -Math.abs(goal.y - start.y);
    const sy = start.y < goal.y ? 1 : -1;
    let error = dx + dy;
    while (true) {
      this.paintDisk(patch, x, y, brushRadius, pixelValue);
      if (x === goal.x && y === goal.y) {
        break;
      }
      const error2 = 2 * error;
      if (error2 >= dy) {
        error += dy;
        x += sx;
      }
      if (error2 <= dx) {
        error += dx;
        y += sy;
      }
    }
    this.renderPatch(patch);
  }

  paintSquareLine(patch, from, to, size, value) {
    const start = this.normalizePoint(from);
    const goal = this.normalizePoint(to);
    const brushSize = clamp(Math.floor(Number(size) || 1), 1, 64);
    const pixelValue = clamp(Math.round(Number(value) || 0), 0, 255);
    let x = start.x;
    let y = start.y;
    const dx = Math.abs(goal.x - start.x);
    const sx = start.x < goal.x ? 1 : -1;
    const dy = -Math.abs(goal.y - start.y);
    const sy = start.y < goal.y ? 1 : -1;
    let error = dx + dy;
    while (true) {
      this.paintSquare(patch, x, y, brushSize, pixelValue);
      if (x === goal.x && y === goal.y) {
        break;
      }
      const error2 = 2 * error;
      if (error2 >= dy) {
        error += dy;
        x += sx;
      }
      if (error2 <= dx) {
        error += dx;
        y += sy;
      }
    }
    this.renderPatch(patch);
  }

  paintRectangle(patch, from, to, value) {
    const start = this.normalizePoint(from);
    const goal = this.normalizePoint(to);
    const left = Math.min(start.x, goal.x);
    const right = Math.max(start.x, goal.x);
    const top = Math.min(start.y, goal.y);
    const bottom = Math.max(start.y, goal.y);
    const pixelValue = clamp(Math.round(Number(value) || 0), 0, 255);
    for (let y = top; y <= bottom; y += 1) {
      for (let x = left; x <= right; x += 1) {
        this.setPixel(patch, x, y, pixelValue);
      }
    }
    this.renderPatch(patch);
  }

  floodFill(patch, point, value) {
    const start = this.normalizePoint(point);
    const replacement = clamp(Math.round(Number(value) || 0), 0, 255);
    const target = this.pixels[(start.y * this.width) + start.x];
    if (target === replacement) {
      return;
    }
    const queue = new Int32Array(this.width * this.height);
    const changed = [];
    let read = 0;
    let write = 0;
    const startIndex = (start.y * this.width) + start.x;
    this.pixels[startIndex] = replacement;
    changed.push(startIndex);
    queue[write++] = startIndex;
    while (read < write) {
      const index = queue[read++];
      const x = index % this.width;
      const y = Math.floor(index / this.width);
      if (x > 0 && this.pixels[index - 1] === target) {
        this.pixels[index - 1] = replacement;
        changed.push(index - 1);
        queue[write++] = index - 1;
      }
      if (x + 1 < this.width && this.pixels[index + 1] === target) {
        this.pixels[index + 1] = replacement;
        changed.push(index + 1);
        queue[write++] = index + 1;
      }
      if (y > 0 && this.pixels[index - this.width] === target) {
        this.pixels[index - this.width] = replacement;
        changed.push(index - this.width);
        queue[write++] = index - this.width;
      }
      if (y + 1 < this.height && this.pixels[index + this.width] === target) {
        this.pixels[index + this.width] = replacement;
        changed.push(index + this.width);
        queue[write++] = index + this.width;
      }
    }
    patch.bulkIndices = Int32Array.from(changed);
    patch.bulkBeforeValue = target;
    patch.bulkAfterValue = replacement;
    this.renderIndices(patch.bulkIndices);
  }

  commandForPatch(patch) {
    if (patch?.bulkIndices?.length) {
      const indices = patch.bulkIndices;
      const apply = (value) => {
        for (let offset = 0; offset < indices.length; offset += 1) {
          this.pixels[indices[offset]] = value;
        }
        this.renderIndices(indices);
      };
      return {
        label: patch.label,
        undo: () => apply(patch.bulkBeforeValue),
        redo: () => apply(patch.bulkAfterValue),
        pixelCount: indices.length,
      };
    }
    const indices = Array.from(patch?.before?.keys?.() || []);
    if (!indices.length) {
      return null;
    }
    const before = new Uint8Array(indices.length);
    const after = new Uint8Array(indices.length);
    indices.forEach((index, offset) => {
      before[offset] = patch.before.get(index);
      after[offset] = patch.after.get(index);
    });
    const apply = (values) => {
      indices.forEach((index, offset) => {
        this.pixels[index] = values[offset];
      });
      this.renderIndices(indices);
    };
    return {
      label: patch.label,
      undo: () => apply(before),
      redo: () => apply(after),
      pixelCount: indices.length,
    };
  }

  toDataUrl() {
    return this.canvas.toDataURL("image/png");
  }

  toPayload() {
    return {
      encoding: "base64",
      format: "gray8",
      width: this.width,
      height: this.height,
      pixelsBase64: bytesToBase64(this.pixels),
    };
  }

  normalizePoint(point) {
    return {
      x: clamp(Math.floor(Number(point?.x) || 0), 0, this.width - 1),
      y: clamp(Math.floor(Number(point?.y) || 0), 0, this.height - 1),
    };
  }

  paintDisk(patch, centerX, centerY, radius, value) {
    const radiusSquared = radius * radius;
    for (let y = centerY - radius; y <= centerY + radius; y += 1) {
      if (y < 0 || y >= this.height) continue;
      for (let x = centerX - radius; x <= centerX + radius; x += 1) {
        if (x < 0 || x >= this.width) continue;
        const dx = x - centerX;
        const dy = y - centerY;
        if ((dx * dx) + (dy * dy) <= radiusSquared) {
          this.setPixel(patch, x, y, value);
        }
      }
    }
  }

  paintSquare(patch, centerX, centerY, size, value) {
    const brushSize = clamp(Math.floor(Number(size) || 1), 1, 64);
    const left = centerX - Math.floor((brushSize - 1) / 2);
    const top = centerY - Math.floor((brushSize - 1) / 2);
    for (let y = top; y < top + brushSize; y += 1) {
      if (y < 0 || y >= this.height) continue;
      for (let x = left; x < left + brushSize; x += 1) {
        if (x < 0 || x >= this.width) continue;
        this.setPixel(patch, x, y, value);
      }
    }
  }

  setPixel(patch, x, y, value) {
    const index = (y * this.width) + x;
    const previous = this.pixels[index];
    if (previous === value) {
      return;
    }
    if (!patch.before.has(index)) {
      patch.before.set(index, previous);
    }
    patch.after.set(index, value);
    patch.dirty.add(index);
    this.pixels[index] = value;
  }

  renderAll() {
    const rgba = this.imageData.data;
    for (let index = 0; index < this.pixels.length; index += 1) {
      const value = this.pixels[index];
      const offset = index * 4;
      rgba[offset] = value;
      rgba[offset + 1] = value;
      rgba[offset + 2] = value;
      rgba[offset + 3] = 255;
    }
    this.context.putImageData(this.imageData, 0, 0);
  }

  renderPatch(patch) {
    const indices = Array.from(patch?.dirty || []);
    this.renderIndices(indices);
    patch?.dirty?.clear?.();
  }

  renderIndices(indices) {
    if (!indices.length) {
      return;
    }
    let minX = this.width - 1;
    let minY = this.height - 1;
    let maxX = 0;
    let maxY = 0;
    for (const index of indices) {
      const value = this.pixels[index];
      const offset = index * 4;
      this.imageData.data[offset] = value;
      this.imageData.data[offset + 1] = value;
      this.imageData.data[offset + 2] = value;
      this.imageData.data[offset + 3] = 255;
      const x = index % this.width;
      const y = Math.floor(index / this.width);
      minX = Math.min(minX, x);
      minY = Math.min(minY, y);
      maxX = Math.max(maxX, x);
      maxY = Math.max(maxY, y);
    }
    const width = Math.max(1, maxX - minX + 1);
    const height = Math.max(1, maxY - minY + 1);
    this.context.putImageData(this.imageData, 0, 0, minX, minY, width, height);
  }
}
