function normalizedArea(start, goal) {
  const firstX = Number(start?.x);
  const firstY = Number(start?.y);
  const secondX = Number(goal?.x);
  const secondY = Number(goal?.y);
  if (![firstX, firstY, secondX, secondY].every(Number.isFinite)) {
    return null;
  }
  return {
    minX: Math.min(firstX, secondX),
    maxX: Math.max(firstX, secondX),
    minY: Math.min(firstY, secondY),
    maxY: Math.max(firstY, secondY),
  };
}

function pointInsideArea(point, area) {
  const x = Number(point?.x);
  const y = Number(point?.y);
  return (
    Number.isFinite(x)
    && Number.isFinite(y)
    && x >= area.minX
    && x <= area.maxX
    && y >= area.minY
    && y <= area.maxY
  );
}

function edgeWorldPoints(edge, byName) {
  if (Array.isArray(edge?.world_points) && edge.world_points.length >= 2) {
    return edge.world_points;
  }
  if (Array.isArray(edge?.control_points) && edge.control_points.length >= 2) {
    return edge.control_points;
  }
  const start = byName.get(String(edge?.from || ""));
  const goal = byName.get(String(edge?.to || ""));
  return start && goal ? [start, goal] : [];
}

function edgeMidpoint(edge, byName) {
  if (Array.isArray(edge?.control_points) && edge.control_points.length === 4) {
    const [first, second, third, fourth] = edge.control_points;
    return {
      x: (
        Number(first.x || 0)
        + (3 * Number(second.x || 0))
        + (3 * Number(third.x || 0))
        + Number(fourth.x || 0)
      ) / 8,
      y: (
        Number(first.y || 0)
        + (3 * Number(second.y || 0))
        + (3 * Number(third.y || 0))
        + Number(fourth.y || 0)
      ) / 8,
    };
  }
  const points = edgeWorldPoints(edge, byName);
  if (points.length < 2) {
    return null;
  }
  const lengths = [];
  let total = 0;
  for (let index = 0; index + 1 < points.length; index += 1) {
    const length = Math.hypot(
      Number(points[index + 1].x || 0) - Number(points[index].x || 0),
      Number(points[index + 1].y || 0) - Number(points[index].y || 0),
    );
    lengths.push(length);
    total += length;
  }
  let remaining = total / 2;
  for (let index = 0; index < lengths.length; index += 1) {
    if (remaining > lengths[index] && index + 1 < lengths.length) {
      remaining -= lengths[index];
      continue;
    }
    const ratio = lengths[index] > 1e-12 ? remaining / lengths[index] : 0;
    return {
      x: Number(points[index].x || 0)
        + ((Number(points[index + 1].x || 0) - Number(points[index].x || 0)) * ratio),
      y: Number(points[index].y || 0)
        + ((Number(points[index + 1].y || 0) - Number(points[index].y || 0)) * ratio),
    };
  }
  return points[points.length - 1];
}

function edgeIntersectsArea(edge, area, byName) {
  // Selecting by midpoint deliberately excludes perpendicular connector edges
  // which only touch a corridor rectangle at an endpoint.  Including those
  // edges creates a branched, oversized mutex region and can gridlock an
  // otherwise independent neighboring aisle.
  return pointInsideArea(edgeMidpoint(edge, byName), area);
}

/**
 * Store a geometric corridor policy.  Graph membership, non-waiting internal
 * LMs and outside stop lines are compiled by Fleet Manager Core when the map
 * is loaded.  The editor deliberately does not bake policy into individual
 * edges, so the same feature works for every map and survives graph edits.
 */
export function markControlledCorridorArea(mapPayload, start, goal) {
  const area = normalizedArea(start, goal);
  const edges = Array.isArray(mapPayload?.edges) ? mapPayload.edges : [];
  const landmarks = Array.isArray(mapPayload?.lms) ? mapPayload.lms : [];
  if (
    !area
    || Math.abs(area.maxX - area.minX) < 0.001
    || Math.abs(area.maxY - area.minY) < 0.001
  ) {
    return null;
  }
  const byName = new Map(landmarks.map((landmark) => [String(landmark.name || ""), landmark]));
  const selectedEdges = edges.filter((edge) => edgeIntersectsArea(edge, area, byName));
  const zones = Array.isArray(mapPayload?.trafficZones)
    ? mapPayload.trafficZones
    : [];
  mapPayload.trafficZones = zones;
  const coordinateKey = [
    area.minX,
    area.minY,
    area.maxX,
    area.maxY,
  ].map((value) => Number(value).toFixed(3).replace(/[^0-9]/g, "")).join("-");
  const baseId = `corridor:zone:${coordinateKey}`;
  let regionId = baseId;
  let suffix = 2;
  const ids = new Set(zones.map((zone) => String(zone?.id || "")));
  while (ids.has(regionId)) {
    regionId = `${baseId}:${suffix}`;
    suffix += 1;
  }
  zones.push({
    id: regionId,
    kind: "controlled_corridor",
    shape: "rectangle",
    bounds: { ...area },
    capacity: 1,
    properties: {},
  });
  return {
    area,
    regions: [{ regionId }],
    edgeCount: selectedEdges.length,
    lmCount: landmarks.filter((landmark) => pointInsideArea(landmark, area)).length,
  };
}
