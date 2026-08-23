const graph = window.FLOW_GRAPH;
const mobile = window.matchMedia('(max-width: 760px)').matches;
const desktopEdgeStyles = mobile ? [] : graph.edges
  .filter(edge => edge.desktopRoute)
  .map(edge => ({
    selector: `edge#${edge.id}`,
    style: edge.desktopRoute
  }));

const elements = [
  ...graph.nodes.map(node => ({
    data: {
      id: node.id,
      label: node.label,
      description: node.description,
      category: node.category,
      color: graph.categories[node.category].color,
      pulseColor: graph.categories[node.category].active
    },
    position: mobile ? node.mobilePosition : node.position
  })),
  ...graph.edges.map(edge => ({
    data: {
      id: edge.id,
      source: edge.source,
      target: edge.target,
      label: edge.label,
      description: edge.description,
      category: edge.category,
      color: graph.categories[edge.category].color,
      activeColor: graph.categories[edge.category].active
    }
  }))
];

const cy = cytoscape({
  container: document.getElementById('cy'),
  elements,
  layout: { name: 'preset', fit: true, padding: mobile ? 56 : 62 },
  minZoom: 0.3,
  maxZoom: 2.4,
  wheelSensitivity: 0.18,
  style: [
    { selector: 'node', style: {
      'shape': 'round-rectangle',
      'width': mobile ? 150 : 172,
      'height': mobile ? 54 : 58,
      'background-color': '#ffffff',
      'border-width': 3,
      'border-color': 'data(color)',
      'label': 'data(label)',
      'font-family': 'Inter, ui-sans-serif, system-ui',
      'font-size': mobile ? 11 : 13,
      'font-weight': 650,
      'color': '#17202a',
      'text-wrap': 'wrap',
      'text-max-width': mobile ? 136 : 156,
      'text-valign': 'center',
      'text-halign': 'center',
      'overlay-opacity': 0,
      'transition-property': 'border-color, border-width, shadow-blur, shadow-opacity',
      'transition-duration': '90ms',
      'z-index-compare': 'manual',
      'z-index': 20
    }},
    { selector: 'edge', style: {
      'width': 2.5,
      'line-color': 'data(color)',
      'target-arrow-color': 'data(color)',
      'target-arrow-shape': 'triangle',
      'curve-style': 'bezier',
      'label': 'data(label)',
      'font-family': 'Inter, ui-sans-serif, system-ui',
      'font-size': 10,
      'color': '#4b5863',
      'text-background-color': '#f8f9fa',
      'text-background-opacity': 0.94,
      'text-background-padding': 3,
      'text-background-shape': 'roundrectangle',
      'text-border-color': '#d7dde2',
      'text-border-width': 1,
      'text-border-opacity': 0.8,
      'text-rotation': 'autorotate',
      'overlay-opacity': 0,
      'z-index-compare': 'manual',
      'z-index': 2
    }},
    { selector: 'node.active, node.task-active', style: {
      'border-color': 'data(pulseColor)',
      'border-width': 5,
      'shadow-color': 'data(pulseColor)',
      'shadow-opacity': 0.34,
      'shadow-blur': 18
    }},
    { selector: 'edge.active, edge.task-active', style: {
      'line-color': 'data(activeColor)',
      'target-arrow-color': 'data(activeColor)',
      'width': 5,
      'z-index': 12
    }},
    { selector: 'edge.error', style: {
      'line-color': '#c93737',
      'target-arrow-color': '#c93737'
    }},
    ...desktopEdgeStyles
  ]
});

const tooltip = document.getElementById('tooltip');
const eventList = document.getElementById('event-list');
const eventCount = document.getElementById('event-count');
const gapCount = document.getElementById('gap-count');
const lastSequence = document.getElementById('last-sequence');
const lastDetail = document.getElementById('last-detail');
const connectionDot = document.getElementById('connection-dot');
const connectionLabel = document.getElementById('connection-label');
const pendingEvents = [];
const activeTimers = new Map();
const secondaryTasks = new Map();
const secondaryReleaseTimers = new Map();
const secondaryEdgeOwners = new Map();
const secondaryNodeOwners = new Map();
const SECONDARY_DONE_HOLD_MS = 420;
let renderScheduled = false;
let totalEvents = 0;
let totalGaps = 0;

lastDetail.textContent = `${graph.phase} · esperando deltas ORB-SLAM3`;
eventList.querySelector('.empty-state').textContent =
  `Esperando eventos del flujo ${graph.phase}`;

function showTooltip(event) {
  const item = event.target;
  tooltip.textContent = item.data('description');
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

function clearActivity() {
  activeTimers.forEach(timer => clearTimeout(timer));
  activeTimers.clear();
  secondaryReleaseTimers.forEach(timer => clearTimeout(timer));
  secondaryReleaseTimers.clear();
  secondaryTasks.clear();
  secondaryEdgeOwners.clear();
  secondaryNodeOwners.clear();
  cy.elements().removeClass('active task-active error');
}

function addOwner(owners, id, flowId, element) {
  const current = owners.get(id) || new Set();
  current.add(flowId);
  owners.set(id, current);
  element.addClass('task-active');
}

function removeOwner(owners, id, flowId, element) {
  const current = owners.get(id);
  if (!current) return;
  current.delete(flowId);
  if (current.size) return;
  owners.delete(id);
  element.removeClass('task-active');
}

function latchSecondaryEdge(flowId, edge) {
  const task = secondaryTasks.get(flowId) || { edges: new Set() };
  if (task.edges.has(edge.id())) return;
  task.edges.add(edge.id());
  secondaryTasks.set(flowId, task);
  addOwner(secondaryEdgeOwners, edge.id(), flowId, edge);
  addOwner(secondaryNodeOwners, edge.source().id(), flowId, edge.source());
  addOwner(secondaryNodeOwners, edge.target().id(), flowId, edge.target());
}

function releaseSecondaryTask(flowId) {
  const task = secondaryTasks.get(flowId);
  if (!task) return;
  task.edges.forEach(edgeId => {
    const edge = cy.getElementById(edgeId);
    if (!edge.length) return;
    removeOwner(secondaryEdgeOwners, edgeId, flowId, edge);
    removeOwner(secondaryNodeOwners, edge.source().id(), flowId, edge.source());
    removeOwner(secondaryNodeOwners, edge.target().id(), flowId, edge.target());
  });
  secondaryTasks.delete(flowId);
  secondaryReleaseTimers.delete(flowId);
}

function renderSecondaryLifecycle(event) {
  const flowId = event.flow_id;
  if (!flowId) return;
  if (event.task_state === 'start') {
    const previous = secondaryReleaseTimers.get(flowId);
    if (previous) clearTimeout(previous);
    secondaryReleaseTimers.delete(flowId);
    secondaryTasks.set(flowId, secondaryTasks.get(flowId) || { edges: new Set() });
    return;
  }
  if (event.task_state === 'done') {
    const previous = secondaryReleaseTimers.get(flowId);
    if (previous) clearTimeout(previous);
    secondaryReleaseTimers.set(flowId, setTimeout(
      () => releaseSecondaryTask(flowId), SECONDARY_DONE_HOLD_MS));
  }
}

function appendActivity(event, edge) {
  const empty = eventList.querySelector('.empty-state');
  if (empty) empty.remove();
  const item = document.createElement('li');
  item.innerHTML = '<strong></strong><span></span>';
  item.querySelector('strong').textContent =
    `${edge.source().data('label')} -> ${edge.target().data('label')}`;
  item.querySelector('span').textContent =
    event.detail || event.payload_kind || event.phase || edge.data('label');
  eventList.prepend(item);
  while (eventList.children.length > 16) eventList.lastChild.remove();
}

function renderEvent(event) {
  if (event.kind === 'state_reset') {
    clearActivity();
    totalGaps += 1;
    gapCount.textContent = totalGaps.toLocaleString('es-ES');
    lastDetail.textContent = 'Estado live restablecido';
    return;
  }

  if (event.kind === 'secondary_task_lifecycle') {
    totalEvents += 1;
    eventCount.textContent = totalEvents.toLocaleString('es-ES');
    lastSequence.textContent = `#${event.bridge_seq || event.seq || totalEvents}`;
    lastDetail.textContent = event.detail || event.task_state;
    renderSecondaryLifecycle(event);
    return;
  }

  const edgeId = event.edge_id || event.edge;
  const edge = edgeId ? cy.getElementById(edgeId) : cy.collection();
  if (!edge.length || !edge.isEdge()) {
    totalGaps += 1;
    gapCount.textContent = totalGaps.toLocaleString('es-ES');
    lastDetail.textContent = 'Evento sin arista declarada';
    return;
  }

  totalEvents += 1;
  eventCount.textContent = totalEvents.toLocaleString('es-ES');
  lastSequence.textContent = `#${event.bridge_seq || event.seq || totalEvents}`;
  lastDetail.textContent = event.detail || event.payload_kind || edge.data('label');

  const secondaryFlow = typeof event.flow_id === 'string' &&
    event.flow_id.startsWith('secondary:');
  if (secondaryFlow) {
    latchSecondaryEdge(event.flow_id, edge);
  } else {
    const previousTimer = activeTimers.get(edgeId);
    if (previousTimer) clearTimeout(previousTimer);
    edge.addClass(event.stage === 'hard_failure' ? 'error active' : 'active');
    edge.source().addClass('active');
    edge.target().addClass('active');
    activeTimers.set(edgeId, setTimeout(() => {
      edge.removeClass('active error');
      edge.source().removeClass('active');
      edge.target().removeClass('active');
      activeTimers.delete(edgeId);
    }, 240));
  }
  appendActivity(event, edge);
}

function drainEvents() {
  renderScheduled = false;
  const batch = pendingEvents.splice(0, pendingEvents.length);
  batch.forEach(renderEvent);
}

function scheduleEvent(event) {
  pendingEvents.push(event);
  if (pendingEvents.length > 256) {
    pendingEvents.splice(0, pendingEvents.length - 256);
    totalGaps += 1;
    gapCount.textContent = totalGaps.toLocaleString('es-ES');
  }
  if (!renderScheduled) {
    renderScheduled = true;
    window.requestAnimationFrame(drainEvents);
  }
}

function connect() {
  const stream = new EventSource('/events');
  stream.onopen = () => {
    connectionDot.classList.add('online');
    connectionLabel.textContent = 'SSE conectado';
  };
  stream.onerror = () => {
    connectionDot.classList.remove('online');
    connectionLabel.textContent = 'Reconectando';
  };
  stream.addEventListener('state_reset', message => {
    try {
      scheduleEvent(JSON.parse(message.data));
    } catch (_error) {
      connectionLabel.textContent = 'Reset invalido';
    }
  });
  stream.onmessage = message => {
    try {
      const event = JSON.parse(message.data);
      event.bridge_seq = Number(message.lastEventId || event.seq || 0);
      scheduleEvent(event);
    } catch (_error) {
      connectionLabel.textContent = 'Evento invalido';
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
document.getElementById('fit').addEventListener('click', () =>
  cy.fit(undefined, mobile ? 56 : 62));
window.addEventListener('resize', () => cy.resize());

window.__PIPELINE_FLOW_DEBUG__ = {
  getState: () => ({
    phase: graph.phase,
    nodeCount: cy.nodes().length,
    edgeCount: cy.edges().length,
    pendingCount: pendingEvents.length,
    renderedEvents: totalEvents,
    gaps: totalGaps,
    activeSecondaryTasks: secondaryTasks.size
  })
};

setTimeout(() => {
  cy.resize();
  cy.fit(undefined, mobile ? 56 : 62);
}, 120);

connect();
