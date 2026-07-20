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

function corridorAreaId(area, componentIndex) {
  const value = (number) => Number(number).toFixed(3).replace(/-0\.000/, "0.000");
  return [
    "corridor:zone",
    `${value(area.minX)},${value(area.minY)}`,
    `${value(area.maxX)},${value(area.maxY)}`,
    componentIndex + 1,
  ].join(":");
}

function undirectedEdgeKey(first, second) {
  return [String(first || ""), String(second || "")].sort().join("\u0000");
}

function corridorChainId(names, area, componentIndex) {
  if (names.length >= 2 && names[0] !== names[names.length - 1]) {
    return `corridor:${[names[0], names[names.length - 1]].sort().join("<=>")}`;
  }
  return corridorAreaId(area, componentIndex);
}

function selectedCorridorChains(selectedEdges) {
  const pairs = new Map();
  for (const edge of selectedEdges) {
    const from = String(edge.from || "");
    const to = String(edge.to || "");
    if (!from || !to || from === to) {
      continue;
    }
    const key = undirectedEdgeKey(from, to);
    if (!pairs.has(key)) {
      pairs.set(key, { key, first: from, second: to, edges: [] });
    }
    pairs.get(key).edges.push(edge);
  }
  const adjacency = new Map();
  const append = (name, key) => {
    if (!adjacency.has(name)) {
      adjacency.set(name, new Set());
    }
    adjacency.get(name).add(key);
  };
  for (const pair of pairs.values()) {
    append(pair.first, pair.key);
    append(pair.second, pair.key);
  }
  const otherName = (pair, name) => (pair.first === name ? pair.second : pair.first);
  const visited = new Set();
  const chains = [];
  const walk = (startName, firstKey) => {
    const pairKeys = [];
    const names = [startName];
    let currentName = startName;
    let currentKey = firstKey;
    while (currentKey && !visited.has(currentKey)) {
      visited.add(currentKey);
      pairKeys.push(currentKey);
      const pair = pairs.get(currentKey);
      currentName = otherName(pair, currentName);
      names.push(currentName);
      const candidates = [...(adjacency.get(currentName) || [])]
        .filter((key) => !visited.has(key));
      if ((adjacency.get(currentName)?.size || 0) !== 2 || candidates.length !== 1) {
        break;
      }
      [currentKey] = candidates;
    }
    chains.push({
      names,
      pairKeys,
      edges: pairKeys.flatMap((key) => pairs.get(key)?.edges || []),
    });
  };

  const terminals = [...adjacency.keys()]
    .filter((name) => (adjacency.get(name)?.size || 0) !== 2)
    .sort();
  for (const name of terminals) {
    for (const key of [...(adjacency.get(name) || [])].sort()) {
      if (!visited.has(key)) {
        walk(name, key);
      }
    }
  }
  for (const key of [...pairs.keys()].sort()) {
    if (!visited.has(key)) {
      walk(pairs.get(key).first, key);
    }
  }
  return { adjacency, chains };
}

/**
 * Mark every maximal non-branching graph chain whose midpoint intersects a
 * dragged rectangular editor area as its own controlled corridor.
 *
 * Directed reverse edges share the same region.  Graph nodes at the boundary
 * of the selected component remain holding points, while strictly internal
 * nodes become non-waiting corridor resources.  Splitting disconnected
 * chains is important: a single rectangle may cross several warehouse aisles
 * or a junction and those branches must not unnecessarily serialize one
 * another.
 */
export function markControlledCorridorArea(mapPayload, start, goal) {
  const area = normalizedArea(start, goal);
  const edges = Array.isArray(mapPayload?.edges) ? mapPayload.edges : [];
  const landmarks = Array.isArray(mapPayload?.lms) ? mapPayload.lms : [];
  if (!area || !edges.length || !landmarks.length) {
    return null;
  }
  const byName = new Map(landmarks.map((landmark) => [String(landmark.name || ""), landmark]));
  const selectedEdges = edges.filter((edge) => edgeIntersectsArea(edge, area, byName));
  if (!selectedEdges.length) {
    return null;
  }

  const { adjacency, chains } = selectedCorridorChains(selectedEdges);
  const selectedEdgeSet = new Set(selectedEdges);
  const regions = [];
  chains.forEach((chain, componentIndex) => {
    const names = new Set(chain.names);
    const componentEdges = chain.edges;
    if (!componentEdges.length) {
      return;
    }
    const regionId = corridorChainId(chain.names, area, componentIndex);
    for (const edge of componentEdges) {
      edge.properties = edge.properties && typeof edge.properties === "object"
        ? edge.properties
        : {};
      edge.properties.controlled_region = regionId;
    }

    const boundaryNames = new Set();
    for (const name of names) {
      const selectedNeighbors = adjacency.get(name)?.size || 0;
      const hasOutsideEdge = edges.some((edge) => (
        !selectedEdgeSet.has(edge)
        && (String(edge.from || "") === name || String(edge.to || "") === name)
      ));
      const isChainEndpoint = name === chain.names[0] || name === chain.names[chain.names.length - 1];
      if (isChainEndpoint || selectedNeighbors > 2 || hasOutsideEdge) {
        boundaryNames.add(name);
      }
    }
    for (const name of names) {
      const landmark = byName.get(name);
      if (!landmark) {
        continue;
      }
      landmark.properties = landmark.properties && typeof landmark.properties === "object"
        ? landmark.properties
        : {};
      if (boundaryNames.has(name)) {
        landmark.properties.can_wait = true;
        landmark.properties.holding_point = true;
        delete landmark.properties.controlled_region;
      } else {
        landmark.properties.can_wait = false;
        delete landmark.properties.holding_point;
        landmark.properties.controlled_region = regionId;
      }
    }
    regions.push({
      regionId,
      edgeCount: componentEdges.length,
      lmCount: names.size,
      holdingLms: [...boundaryNames].sort(),
    });
  });
  return {
    area,
    regions,
    edgeCount: selectedEdges.length,
    lmCount: new Set(selectedEdges.flatMap((edge) => [edge.from, edge.to])).size,
  };
}
