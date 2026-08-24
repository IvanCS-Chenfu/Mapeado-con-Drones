(() => {
  const graph = window.SYSTEM_ARCHITECTURE;
  const layout = window.SYSTEM_ARCHITECTURE_LAYOUT || { positions: {} };
  const metadata = window.SYSTEM_ARCHITECTURE_METADATA || { nodes: {}, edges: {} };
  graph.nodes.forEach(node => {
    Object.assign(node.data, metadata.nodes[node.data.id] || {});
    if (layout.positions[node.data.id]) node.position = layout.positions[node.data.id];
  });
  graph.edges.forEach(edge => Object.assign(
    edge.data, metadata.edges[edge.data.id] || {}));
  const status = document.getElementById('connection-status');
  const tooltip = document.getElementById('tooltip');
  const activeTimers = new Map();

  const colors = { dron: '#2d8a68', servidor: '#c34e3f', simulacion: '#2478a6' };
  const cy = cytoscape({
    container: document.getElementById('cy'),
    elements: [...graph.nodes, ...graph.edges],
    layout: { name: 'preset', fit: true, padding: 32 },
    minZoom: 0.18,
    maxZoom: 2.4,
    wheelSensitivity: 0.16,
    style: [
      { selector: 'node[kind = "group"]', style: {
        'label': 'data(label)', 'text-valign': 'top', 'text-halign': 'center',
        'font-size': 16, 'font-weight': 700, 'color': '#17212b',
        'background-opacity': 0.06, 'border-width': 2,
        'border-color': ele => colors[ele.data('group')], 'padding': 34,
        'shape': 'roundrectangle' } },
      { selector: 'node[kind = "package"]', style: {
        'width': 172, 'height': 54, 'shape': 'roundrectangle',
        'background-color': ele => colors[ele.data('group')], 'border-width': 1,
        'border-color': '#ffffff', 'label': 'data(label)', 'font-size': 13,
        'font-weight': 600, 'color': '#ffffff', 'text-valign': 'center',
        'text-halign': 'center', 'text-outline-color': '#17212b',
        'text-outline-width': 1, 'text-wrap': 'wrap', 'text-max-width': 156 } },
      { selector: 'edge', style: {
        'curve-style': 'bezier', 'width': 2, 'line-color': '#86939a',
        'target-arrow-color': '#86939a', 'target-arrow-shape': 'triangle',
        'arrow-scale': 0.8, 'label': 'data(label)', 'font-size': 10,
        'color': '#45535c', 'text-background-color': '#f4f6f7',
        'text-background-opacity': 0.92, 'text-background-padding': 2,
        'text-rotation': 'autorotate' } },
      { selector: 'edge.active', style: {
        'width': 5, 'line-color': '#e3a008',
        'target-arrow-color': '#e3a008', 'z-index': 20 } },
      { selector: '.layer-hidden', style: { 'display': 'none' } },
      { selector: ':selected', style: { 'border-width': 4, 'border-color': '#e3a008' } }
    ]
  });

  function showTooltip(event) {
    const element = event.target;
    const fields = [
      ['rol', element.data('role')],
      ['ruta', element.data('path')],
      ['nombre ROS', element.data('ros_name')],
      ['ejecutables', element.data('executables')],
      ['YAML propietarios', element.data('owned_yaml')],
      ['dependencias', element.data('dependencies')],
      ['relaciones cross-group', element.data('cross_group')],
      ['capa', element.data('layer')],
      ['interfaz', element.data('interface')],
      ['tipo', element.data('interface_kind')],
      ['tipo ROS', element.data('message_type')],
      ['namespace', element.data('namespace')],
      ['QoS', element.data('qos')],
      ['datos', element.data('data_transferred')],
      ['productor', element.data('producer')],
      ['consumidor', element.data('consumer')],
      ['estado', element.data('status')],
      ['actividad', element.data('last_activity')],
      ['dron/instancia', element.data('last_drone')],
      ['interfaz activa', element.data('last_interface')],
      ['contador', element.data('activity_count')],
    ].filter(([, value]) => value !== undefined && value !== null && value !== '');
    tooltip.innerHTML = `<strong>${element.data('label') || element.id()}</strong>` +
      fields.map(([name, value]) => `<span>${name}: ${value}</span>`).join('');
    tooltip.hidden = false;
    tooltip.style.left = `${Math.min(event.originalEvent.clientX + 14, window.innerWidth - 390)}px`;
    tooltip.style.top = `${Math.min(event.originalEvent.clientY + 14, window.innerHeight - 130)}px`;
  }

  cy.on('mouseover', 'node, edge', showTooltip);
  cy.on('mouseout', 'node, edge', () => { tooltip.hidden = true; });
  document.getElementById('fit').addEventListener('click', () => cy.fit(undefined, 32));

  let resizeTimer;
  window.addEventListener('resize', () => {
    clearTimeout(resizeTimer);
    resizeTimer = setTimeout(() => {
      cy.resize();
      cy.fit(undefined, window.innerWidth <= 860 ? 18 : 32);
    }, 80);
  });

  document.querySelectorAll('[data-layer]').forEach(input => {
    input.addEventListener('change', () => {
      cy.edges(`[layer = "${input.dataset.layer}"]`).toggleClass(
        'layer-hidden', !input.checked);
    });
  });

  function pulse(event) {
    const edge = cy.getElementById(event.edge_id);
    if (edge.empty() || edge.data('layer') !== 'runtime' ||
        edge.data('activity_mode') !== 'direct') return;
    edge.data('last_activity', new Date().toLocaleTimeString());
    edge.data('last_drone', event.drone_id ?? event.namespace ?? event.source ?? 'n/a');
    edge.data('last_interface', event.interface || edge.data('interface'));
    edge.data('activity_count', event.count || 1);
    edge.addClass('active');
    clearTimeout(activeTimers.get(event.edge_id));
    const ttl = Number(edge.data('ttl_ms') || 800);
    activeTimers.set(event.edge_id, setTimeout(() => edge.removeClass('active'), ttl));
  }

  async function connectTelemetry() {
    try {
      const healthResponse = await fetch('/health', { cache: 'no-store' });
      const health = await healthResponse.json();
      if (health.mode !== 'live') {
        status.textContent = 'estatico';
        status.className = 'status';
        return;
      }
      const events = new EventSource('/events');
      events.onopen = () => {
        status.textContent = 'live';
        status.className = 'status live';
      };
      events.onmessage = message => {
        try {
          const event = JSON.parse(message.data);
          if (event.kind === 'architecture_activity') pulse(event);
        } catch (_error) {
          status.textContent = 'evento invalido';
        }
      };
      events.onerror = () => {
        status.textContent = 'sin telemetria';
        status.className = 'status disconnected';
      };
    } catch (_error) {
      status.textContent = 'estatico';
    }
  }

  connectTelemetry();
})();
