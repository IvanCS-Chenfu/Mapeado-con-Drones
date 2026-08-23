(() => {
  const graph = window.SYSTEM_ARCHITECTURE;
  const status = document.getElementById('connection-status');
  const tooltip = document.getElementById('tooltip');
  const activeTimers = new Map();

  const colors = {
    dron: '#2d8a68',
    servidor: '#c34e3f',
    simulacion: '#2478a6'
  };

  const cy = cytoscape({
    container: document.getElementById('cy'),
    elements: [...graph.nodes, ...graph.edges],
    layout: { name: 'preset', fit: true, padding: 32 },
    minZoom: 0.18,
    maxZoom: 2.4,
    wheelSensitivity: 0.16,
    style: [
      {
        selector: 'node[kind = "group"]',
        style: {
          'label': 'data(label)',
          'text-valign': 'top',
          'text-halign': 'center',
          'font-size': 16,
          'font-weight': 700,
          'color': '#17212b',
          'background-opacity': 0.06,
          'border-width': 2,
          'border-color': ele => colors[ele.data('group')],
          'padding': 34,
          'shape': 'roundrectangle'
        }
      },
      {
        selector: 'node[kind = "package"]',
        style: {
          'width': 172,
          'height': 54,
          'shape': 'roundrectangle',
          'background-color': ele => colors[ele.data('group')],
          'border-width': 1,
          'border-color': '#ffffff',
          'label': 'data(label)',
          'font-size': 13,
          'font-weight': 600,
          'color': '#ffffff',
          'text-valign': 'center',
          'text-halign': 'center',
          'text-outline-color': '#17212b',
          'text-outline-width': 1,
          'text-wrap': 'wrap',
          'text-max-width': 156
        }
      },
      {
        selector: 'edge',
        style: {
          'curve-style': 'bezier',
          'width': 2,
          'line-color': '#86939a',
          'target-arrow-color': '#86939a',
          'target-arrow-shape': 'triangle',
          'arrow-scale': 0.8,
          'label': 'data(label)',
          'font-size': 10,
          'color': '#45535c',
          'text-background-color': '#f4f6f7',
          'text-background-opacity': 0.92,
          'text-background-padding': 2,
          'text-rotation': 'autorotate'
        }
      },
      {
        selector: 'edge.active',
        style: {
          'width': 5,
          'line-color': '#e3a008',
          'target-arrow-color': '#e3a008',
          'z-index': 20
        }
      },
      { selector: '.layer-hidden', style: { 'display': 'none' } },
      { selector: ':selected', style: { 'border-width': 4, 'border-color': '#e3a008' } }
    ]
  });

  function showTooltip(event) {
    const element = event.target;
    const role = element.data('role');
    const interfaceName = element.data('interface');
    tooltip.innerHTML = `<strong>${element.data('label') || element.id()}</strong>` +
      (role ? `<span>${role}</span>` : '') +
      (interfaceName ? `<code>${interfaceName}</code>` : '');
    tooltip.hidden = false;
    tooltip.style.left = `${Math.min(event.originalEvent.clientX + 14, window.innerWidth - 370)}px`;
    tooltip.style.top = `${Math.min(event.originalEvent.clientY + 14, window.innerHeight - 100)}px`;
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
      const edges = cy.edges(`[layer = "${input.dataset.layer}"]`);
      edges.toggleClass('layer-hidden', !input.checked);
    });
  });

  function pulse(edgeId) {
    const edge = cy.getElementById(edgeId);
    if (edge.empty()) return;
    edge.addClass('active');
    clearTimeout(activeTimers.get(edgeId));
    activeTimers.set(edgeId, setTimeout(() => edge.removeClass('active'), 460));
  }

  try {
    const events = new EventSource('/events');
    events.onopen = () => {
      status.textContent = 'live';
      status.className = 'status live';
    };
    events.onmessage = message => {
      try {
        const event = JSON.parse(message.data);
        if (event.kind === 'architecture_activity') pulse(event.edge_id);
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
})();
