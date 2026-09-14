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

let count = 0;
function receive(payload) {
  const edge = cy.getElementById(payload.edge_id || '');
  if (edge.length) {
    edge.addClass('active'); edge.source().addClass('active'); edge.target().addClass('active');
    setTimeout(() => {edge.removeClass('active'); edge.source().removeClass('active'); edge.target().removeClass('active');}, 500);
  }
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
