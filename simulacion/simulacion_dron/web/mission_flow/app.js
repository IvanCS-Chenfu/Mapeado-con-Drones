const graph = window.FLOW_GRAPH;
const elements = [
  ...graph.nodes.map(node => ({data: {...node, color: graph.categories[node.category].color, activeColor: graph.categories[node.category].active}, position: node.position})),
  ...graph.edges.map(edge => ({data: {...edge, color: graph.categories[edge.category].color, activeColor: graph.categories[edge.category].active}}))
];
const cy = cytoscape({
  container: document.getElementById('cy'), elements,
  layout: {name: 'preset', fit: true, padding: 55}, wheelSensitivity: 0.18,
  style: [
    {selector: 'node', style: {'shape': 'round-rectangle', 'width': 170, 'height': 56, 'background-color': '#fff', 'border-width': 3, 'border-color': 'data(color)', 'label': 'data(label)', 'font-size': 13, 'text-wrap': 'wrap', 'text-max-width': 150, 'text-valign': 'center', 'color': '#17202a'}},
    {selector: 'edge', style: {'width': 2.5, 'line-color': 'data(color)', 'target-arrow-color': 'data(color)', 'target-arrow-shape': 'triangle', 'curve-style': 'bezier', 'label': 'data(label)', 'font-size': 10, 'text-background-color': '#f8f9fa', 'text-background-opacity': 0.95, 'text-background-padding': 3, 'color': '#44515c'}},
    {selector: '.active', style: {'border-width': 6, 'border-color': 'data(activeColor)', 'line-color': 'data(activeColor)', 'target-arrow-color': 'data(activeColor)'}}
  ]
});

const tooltip = document.getElementById('tooltip');
cy.on('mouseover', 'node, edge', event => {
  tooltip.textContent = event.target.data('description') || event.target.data('label');
  tooltip.classList.add('visible');
});
cy.on('mouseout', 'node, edge', () => tooltip.classList.remove('visible'));

const levelSelect = document.getElementById('level-select');
const canvas = document.getElementById('level-canvas');
const ctx = canvas.getContext('2d');
let regions = [];

function drawLevel() {
  ctx.clearRect(0, 0, canvas.width, canvas.height);
  const selected = Number(levelSelect.value || 0);
  const current = regions.filter(region => region.level === selected);
  if (!current.length) return;
  const xmin = Math.min(...current.map(region => region.min[0]));
  const ymin = Math.min(...current.map(region => region.min[1]));
  const xmax = Math.max(...current.map(region => region.max[0]));
  const ymax = Math.max(...current.map(region => region.max[1]));
  const colors = {AB: '#1b9e77', BC: '#d95f02', CD: '#7570b3', DA: '#e6ab02'};
  current.forEach(region => {
    const x = 36 + (region.min[0] - xmin) / (xmax - xmin) * 448;
    const y = 28 + (ymax - region.max[1]) / (ymax - ymin) * 350;
    const w = (region.max[0] - region.min[0]) / (xmax - xmin) * 448;
    const h = (region.max[1] - region.min[1]) / (ymax - ymin) * 350;
    ctx.fillStyle = `${colors[region.side]}55`;
    ctx.strokeStyle = colors[region.side];
    ctx.lineWidth = 3;
    ctx.fillRect(x, y, w, h);
    ctx.strokeRect(x, y, w, h);
    ctx.fillStyle = '#eef3f7';
    ctx.font = '600 16px system-ui';
    ctx.fillText(region.side, x + 10, y + 23);
  });
}
levelSelect.addEventListener('change', drawLevel);

function updateGeometry(payload) {
  if (!Array.isArray(payload.regions)) return;
  regions = payload.regions;
  const levels = [...new Set(regions.map(region => region.level))].sort((a, b) => a - b);
  levelSelect.replaceChildren(...levels.map(level => {
    const option = document.createElement('option');
    option.value = level;
    option.textContent = `Nivel ${level}`;
    return option;
  }));
  document.getElementById('geometry-state').textContent = `${regions.length} regiones reales, sin asignar`;
  drawLevel();
}

let count = 0;
function receive(payload) {
  const edge = cy.getElementById(payload.edge_id || '');
  if (edge.length) {
    edge.addClass('active'); edge.source().addClass('active'); edge.target().addClass('active');
    setTimeout(() => {edge.removeClass('active'); edge.source().removeClass('active'); edge.target().removeClass('active');}, 500);
  }
  updateGeometry(payload);
  count += 1;
  document.getElementById('event-count').textContent = count.toLocaleString('es-ES');
  document.getElementById('last-detail').textContent = payload.detail || payload.event || 'Evento de misión';
  const list = document.getElementById('event-list');
  const empty = list.querySelector('.empty-state'); if (empty) empty.remove();
  const item = document.createElement('li'); item.textContent = payload.detail || payload.event;
  list.prepend(item); while (list.children.length > 12) list.lastChild.remove();
}

const stream = new EventSource('/events');
stream.onopen = () => {document.getElementById('connection-dot').classList.add('online'); document.getElementById('connection-label').textContent = 'SSE conectado';};
stream.onerror = () => {document.getElementById('connection-dot').classList.remove('online'); document.getElementById('connection-label').textContent = 'Reconectando';};
stream.onmessage = message => {try {receive(JSON.parse(message.data));} catch (_error) {document.getElementById('last-detail').textContent = 'Evento inválido';}};
