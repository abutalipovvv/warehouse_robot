export function shortestEdgePath(edges, startLm, goalLm) {
  const distances = new Map([[startLm, 0]]);
  const previous = new Map();
  const queue = [startLm];
  while (queue.length) {
    queue.sort((first, second) => (
      (distances.get(first) ?? Number.POSITIVE_INFINITY)
      - (distances.get(second) ?? Number.POSITIVE_INFINITY)
    ));
    const current = queue.shift();
    if (current === goalLm) {
      break;
    }
    for (const edge of edges || []) {
      if (edge.from !== current) {
        continue;
      }
      const distance = (distances.get(current) || 0)
        + Math.max(0.000001, Number(edge.length || 1));
      if (distance >= (distances.get(edge.to) ?? Number.POSITIVE_INFINITY)) {
        continue;
      }
      distances.set(edge.to, distance);
      previous.set(edge.to, current);
      if (!queue.includes(edge.to)) {
        queue.push(edge.to);
      }
    }
  }
  if (!distances.has(goalLm)) {
    return [];
  }
  const path = [goalLm];
  while (path[0] !== startLm) {
    const parent = previous.get(path[0]);
    if (!parent) {
      return [];
    }
    path.unshift(parent);
  }
  return path;
}

export function markControlledCorridor(mapPayload, startLm, goalLm) {
  const edges = Array.isArray(mapPayload?.edges) ? mapPayload.edges : [];
  const landmarks = Array.isArray(mapPayload?.lms) ? mapPayload.lms : [];
  const path = shortestEdgePath(edges, startLm, goalLm);
  if (path.length < 2) {
    return null;
  }
  const regionId = `corridor:${[startLm, goalLm].sort().join("<=>")}`;
  for (let index = 0; index + 1 < path.length; index += 1) {
    const from = path[index];
    const to = path[index + 1];
    for (const edge of edges) {
      if (
        (edge.from === from && edge.to === to)
        || (edge.from === to && edge.to === from)
      ) {
        edge.properties = edge.properties && typeof edge.properties === "object"
          ? edge.properties
          : {};
        edge.properties.controlled_region = regionId;
      }
    }
  }
  const byName = new Map(landmarks.map((landmark) => [landmark.name, landmark]));
  path.forEach((name, index) => {
    const landmark = byName.get(name);
    if (!landmark) {
      return;
    }
    landmark.properties = landmark.properties && typeof landmark.properties === "object"
      ? landmark.properties
      : {};
    if (index === 0 || index === path.length - 1) {
      landmark.properties.can_wait = true;
      landmark.properties.holding_point = true;
      delete landmark.properties.controlled_region;
    } else {
      landmark.properties.can_wait = false;
      landmark.properties.controlled_region = regionId;
    }
  });
  return { regionId, path };
}
