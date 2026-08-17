const graph = window.FLOW_GRAPH;
const mobile = window.matchMedia('(max-width: 760px)').matches;
const mobilePositions = {
  raw_db: [60, 70], pose_db: [200, 70], map_builder: [340, 70], rviz2: [480, 70],
  wrappers: [60, 205], server: [200, 205], fiducial: [340, 205], covis_db: [480, 205],
  score_db: [60, 340], fused_db: [200, 340], queue: [340, 340], decision: [480, 340],
  loop_detector: [60, 475], verifier: [200, 475], pose_graph: [340, 475], optimizer: [480, 475]
};

const elements = [
  ...graph.nodes.map(([id, label, x, y, description, category]) => ({
    data: {
      id,
      label,
      description,
      category,
      color: graph.categories[category].color,
      pulseColor: graph.categories[category].active
    },
    position: mobile
      ? { x: mobilePositions[id][0], y: mobilePositions[id][1] }
      : { x, y }
  })),
  ...graph.edges.map(([id, source, target, label, category, description, offset]) => ({
    data: {
      id,
      source,
      target,
      label,
      category,
      description,
      offset,
      color: graph.categories[category].color,
      activeColor: graph.categories[category].active
    }
  }))
];

const cy = cytoscape({
  container: document.getElementById('cy'),
  elements,
  layout: { name: 'preset', fit: true, padding: mobile ? 54 : 60 },
  minZoom: 0.16,
  maxZoom: 2.4,
  wheelSensitivity: 0.18,
  style: [
    { selector: 'node', style: {
      'shape': 'round-rectangle',
      'width': mobile ? 112 : 154,
      'height': mobile ? 42 : 50,
      'background-color': '#ffffff',
      'border-width': 2,
      'border-color': 'data(color)',
      'label': 'data(label)',
      'font-family': 'Inter, ui-sans-serif, system-ui',
      'font-size': mobile ? 9 : 11,
      'font-weight': 650,
      'color': '#17202a',
      'text-wrap': 'wrap',
      'text-max-width': mobile ? 102 : 140,
      'text-valign': 'center',
      'text-halign': 'center',
      'overlay-opacity': 0,
      'transition-property': 'border-color, border-width, background-color, shadow-blur, shadow-opacity',
      'transition-duration': '110ms'
    }},
    { selector: 'edge', style: {
      'width': 1.8,
      'line-color': 'data(color)',
      'target-arrow-color': 'data(color)',
      'target-arrow-shape': 'triangle',
      'curve-style': 'unbundled-bezier',
      'control-point-distances': 'data(offset)',
      'control-point-weights': 0.5,
      'arrow-scale': 0.72,
      'label': 'data(label)',
      'font-family': 'Inter, ui-sans-serif, system-ui',
      'font-size': mobile ? 7 : 8,
      'font-weight': 620,
      'color': '#4b5863',
      'text-background-color': '#f8f9fa',
      'text-background-opacity': 0.92,
      'text-background-padding': 2,
      'text-rotation': 'autorotate',
      'min-zoomed-font-size': 7,
      'overlay-opacity': 0,
      'transition-property': 'line-color, target-arrow-color, width, opacity',
      'transition-duration': '100ms'
    }},
    { selector: 'node.active', style: {
      'background-color': '#ffffff',
      'border-color': 'data(pulseColor)',
      'border-width': 4,
      'shadow-color': 'data(pulseColor)',
      'shadow-opacity': 0.34,
      'shadow-blur': 18,
      'z-index': 20
    }},
    { selector: 'edge.active', style: {
      'line-color': 'data(activeColor)',
      'target-arrow-color': 'data(activeColor)',
      'width': 5,
      'opacity': 1,
      'z-index': 18
    }},
    { selector: 'edge.error', style: {
      'line-color': '#cf3e3e',
      'target-arrow-color': '#cf3e3e',
      'width': 5
    }}
  ]
});

const tooltip = document.getElementById('tooltip');
const eventList = document.getElementById('event-list');
const eventCount = document.getElementById('event-count');
const lastSequence = document.getElementById('last-sequence');
const lastDetail = document.getElementById('last-detail');
const connectionDot = document.getElementById('connection-dot');
const connectionLabel = document.getElementById('connection-label');
const eventQueue = [];
const activeTokens = new Map();
const edgeActivity = new Map();
const nodeActivity = new Map();
const taskStages = new Map();
let totalEvents = 0;

function showTooltip(event) {
  const item = event.target;
  tooltip.textContent = item.isEdge()
    ? `${item.data('label')}\n${item.data('description')}`
    : item.data('description');
  tooltip.classList.add('visible');
  moveTooltip(event.originalEvent);
}

function moveTooltip(event) {
  if (!event) return;
  const width = tooltip.offsetWidth || 300;
  const left = Math.min(window.innerWidth - width - 18, event.clientX + 16);
  const top = Math.min(window.innerHeight - 100, event.clientY + 16);
  tooltip.style.left = `${Math.max(12, left)}px`;
  tooltip.style.top = `${Math.max(72, top)}px`;
}

cy.on('mouseover', 'node, edge', showTooltip);
cy.on('mousemove', 'node, edge', event => moveTooltip(event.originalEvent));
cy.on('mouseout', 'node, edge', () => tooltip.classList.remove('visible'));

function updateNode(node, delta, color) {
  const id = node.id();
  const next = Math.max(0, (nodeActivity.get(id) || 0) + delta);
  if (next === 0) {
    nodeActivity.delete(id);
    node.removeClass('active');
    node.data('pulseColor', graph.categories[node.data('category')].active);
    return;
  }
  nodeActivity.set(id, next);
  node.data('pulseColor', color);
  node.addClass('active');
}

function deactivateToken(token) {
  const active = activeTokens.get(token);
  if (!active) return;
  if (active.timer) clearTimeout(active.timer);
  const edge = cy.getElementById(active.edgeId);
  const next = Math.max(0, (edgeActivity.get(active.edgeId) || 0) - 1);
  if (next === 0) {
    edgeActivity.delete(active.edgeId);
    edge.removeClass('active error');
  } else {
    edgeActivity.set(active.edgeId, next);
  }
  updateNode(edge.source(), -1, active.color);
  updateNode(edge.target(), -1, active.color);
  activeTokens.delete(token);
}

function activateEdge(event, persistent) {
  const edge = cy.getElementById(event.edge);
  if (!edge.length) return;
  const token = `event-${event.seq}`;
  const color = edge.data('activeColor');
  edgeActivity.set(event.edge, (edgeActivity.get(event.edge) || 0) + 1);
  edge.addClass(event.phase === 'error' ? 'error' : 'active');
  updateNode(edge.source(), 1, color);
  updateNode(edge.target(), 1, color);

  const active = { edgeId: event.edge, color, timer: null };
  activeTokens.set(token, active);
  if (!persistent) {
    active.timer = setTimeout(() => deactivateToken(token), 520);
  }
  return token;
}

function clearTaskStage(taskId) {
  const previous = taskStages.get(taskId);
  if (previous) deactivateToken(previous);
  taskStages.delete(taskId);
}

function pulse(event) {
  const taskId = Number(event.task_id || 0);
  const stagePhase = ['start', 'candidates', 'result'].includes(event.phase);
  const terminalPhase = ['commit', 'complete', 'error'].includes(event.phase);
  if (event.phase === 'start' && event.edge.startsWith('secondary_queue_')) {
    [...taskStages.keys()].forEach(clearTaskStage);
  }
  if (taskId > 0) clearTaskStage(taskId);
  const token = activateEdge(event, taskId > 0 && stagePhase);
  if (taskId > 0 && token && stagePhase) taskStages.set(taskId, token);
  if (taskId > 0 && terminalPhase) taskStages.delete(taskId);
}

function appendActivity(event) {
  const item = document.createElement('li');
  const edge = cy.getElementById(event.edge);
  const title = edge.length
    ? `${edge.data('label')}: ${edge.source().data('label')} → ${edge.target().data('label')}`
    : event.edge;
  const task = event.task_id ? ` · tarea ${event.task_id}` : '';
  const count = event.count ? ` · ${event.count}` : '';
  item.innerHTML = '<strong></strong><span></span>';
  item.querySelector('strong').textContent = title;
  item.querySelector('span').textContent = `${event.detail || event.phase}${count}${task}`;
  eventList.prepend(item);
  while (eventList.children.length > 16) eventList.lastChild.remove();
}

function renderEvent(event) {
  totalEvents += 1;
  eventCount.textContent = totalEvents.toLocaleString('es-ES');
  lastSequence.textContent = `#${event.seq}`;
  lastDetail.textContent = event.detail || event.phase;
  pulse(event);
  appendActivity(event);
}

setInterval(() => {
  const event = eventQueue.shift();
  if (event) renderEvent(event);
}, 110);

function connect() {
  const stream = new EventSource('/events');
  stream.onopen = () => {
    connectionDot.classList.add('online');
    connectionLabel.textContent = 'En vivo';
  };
  stream.onerror = () => {
    connectionDot.classList.remove('online');
    connectionLabel.textContent = 'Reconectando';
  };
  stream.onmessage = message => {
    try {
      const event = JSON.parse(message.data);
      eventQueue.push(event);
      if (eventQueue.length > 400) eventQueue.splice(0, eventQueue.length - 400);
    } catch (_error) {
      connectionLabel.textContent = 'Evento inválido';
    }
  };
}

const legend = document.getElementById('flow-legend');
Object.entries(graph.categories).forEach(([id, category]) => {
  const item = document.createElement('span');
  item.innerHTML = '<i></i><b></b>';
  item.querySelector('i').style.backgroundColor = category.active;
  item.querySelector('b').textContent = category.label;
  item.dataset.category = id;
  legend.appendChild(item);
});

document.getElementById('zoom-in').addEventListener('click', () => cy.zoom({
  level: Math.min(cy.maxZoom(), cy.zoom() * 1.18),
  renderedPosition: { x: cy.width() / 2, y: cy.height() / 2 }
}));
document.getElementById('zoom-out').addEventListener('click', () => cy.zoom({
  level: Math.max(cy.minZoom(), cy.zoom() / 1.18),
  renderedPosition: { x: cy.width() / 2, y: cy.height() / 2 }
}));
document.getElementById('fit').addEventListener('click', () => cy.fit(undefined, mobile ? 54 : 60));
window.addEventListener('resize', () => cy.resize());
setTimeout(() => {
  cy.resize();
  cy.fit(undefined, mobile ? 54 : 60);
}, 180);

connect();
