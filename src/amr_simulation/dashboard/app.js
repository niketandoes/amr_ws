/**
 * AMR Fleet Command Live Dashboard Client Application
 * Features:
 *   - 2D Canvas rendering of 12m x 8m warehouse arena, obstacles, and 1.15m bottleneck
 *   - Real-time robot pose, heading angle, and trajectory breadcrumb visualization
 *   - Server-Sent Events (SSE) consumer streaming 10 Hz JSON telemetry from ROS 2
 *   - Telemetry card updates for AMR 1, AMR 2, and AMR 3 (speed, battery SOC, state)
 *   - Dynamic Contract Net Protocol (CNP) event logger (auctions, bids, awards)
 *   - REST API dispatch triggers for opposing choke test and synthetic blockage
 */
(function() {
  const canvas = document.getElementById('warehouseCanvas');
  const ctx = canvas.getContext('2d');

  // Warehouse World Mapping Constants
  // World space: x in [-6.0, 6.0], y in [-4.5, 4.5]
  const WORLD_WIDTH = 12.0;  // meters
  const WORLD_HEIGHT = 8.0;  // meters
  const PADDING = 40;

  let scale = (canvas.width - PADDING * 2) / WORLD_WIDTH;
  const originX = canvas.width / 2;
  const originY = canvas.height / 2;

  // Convert ROS world coordinates (meters, x-right, y-up) to Canvas pixels (x-right, y-down)
  function worldToCanvas(wx, wy) {
    return {
      x: originX + (wx * scale),
      y: originY - (wy * scale)
    };
  }

  // Fleet State
  const fleet = {
    amr1: { x: -4.0, y: 0.0, yaw: 0.0, vx: 0.0, vy: 0.0, speed: 0.0, battery: 98.0, voltage: 25.1, state: 'NORMAL_NAV', trail: [] },
    amr2: { x: 4.0, y: 0.0, yaw: Math.PI, vx: 0.0, vy: 0.0, speed: 0.0, battery: 86.0, voltage: 24.6, state: 'NORMAL_NAV', trail: [] },
    amr3: { x: 1.0, y: -4.0, yaw: Math.PI / 2, vx: 0.0, vy: 0.0, speed: 0.0, battery: 92.0, voltage: 24.9, state: 'IDLE', trail: [] }
  };

  const robotColors = {
    amr1: '#3b82f6',
    amr2: '#ec4899',
    amr3: '#10b981'
  };

  // Static Warehouse Obstacles (Warehouse Racks & Bottleneck)
  const racks = [
    // Top Racks
    { x: -3.5, y: 2.2, w: 2.5, h: 0.8 },
    { x: 3.5, y: 2.2, w: 2.5, h: 0.8 },
    // Bottom Racks
    { x: -3.5, y: -2.2, w: 2.5, h: 0.8 },
    { x: 3.5, y: -2.2, w: 2.5, h: 0.8 },
    // Choke Point Enclosure Walls (forming the 1.15m narrow channel at x in [-1.0, 1.0])
    { x: 0.0, y: 2.0, w: 2.2, h: 1.8 },   // North constriction wall
    { x: 0.0, y: -2.0, w: 2.2, h: 1.8 }   // South constriction wall
  ];

  function drawGrid() {
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.03)';
    ctx.lineWidth = 1;

    for (let x = -6; x <= 6; x += 1) {
      const p1 = worldToCanvas(x, -4);
      const p2 = worldToCanvas(x, 4);
      ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.stroke();
    }

    for (let y = -4; y <= 4; y += 1) {
      const p1 = worldToCanvas(-6, y);
      const p2 = worldToCanvas(6, y);
      ctx.beginPath();
      ctx.moveTo(p1.x, p1.y);
      ctx.lineTo(p2.x, p2.y);
      ctx.stroke();
    }

    // Origin crosshair
    const center = worldToCanvas(0, 0);
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
    ctx.beginPath();
    ctx.moveTo(center.x - 10, center.y);
    ctx.lineTo(center.x + 10, center.y);
    ctx.moveTo(center.x, center.y - 10);
    ctx.lineTo(center.x, center.y + 10);
    ctx.stroke();
  }

  function drawChokeZone() {
    // Bottleneck boundary zone: x in [-1.5, 1.5], y in [-0.6, 0.6]
    const pTopLeft = worldToCanvas(-1.5, 0.6);
    const pBottomRight = worldToCanvas(1.5, -0.6);
    const w = pBottomRight.x - pTopLeft.x;
    const h = pBottomRight.y - pTopLeft.y;

    ctx.fillStyle = 'rgba(239, 68, 68, 0.08)';
    ctx.fillRect(pTopLeft.x, pTopLeft.y, w, h);

    ctx.strokeStyle = 'rgba(239, 68, 68, 0.4)';
    ctx.lineWidth = 1.5;
    ctx.setLineDash([5, 4]);
    ctx.strokeRect(pTopLeft.x, pTopLeft.y, w, h);
    ctx.setLineDash([]);

    // Label
    ctx.fillStyle = 'rgba(239, 68, 68, 0.7)';
    ctx.font = '10px JetBrains Mono';
    ctx.textAlign = 'center';
    ctx.fillText('1.15m CHOKE POINT', originX, originY - 14);
  }

  function drawObstacles() {
    racks.forEach(r => {
      const pTopLeft = worldToCanvas(r.x - r.w / 2, r.y + r.h / 2);
      const pBottomRight = worldToCanvas(r.x + r.w / 2, r.y - r.h / 2);
      const w = pBottomRight.x - pTopLeft.x;
      const h = pBottomRight.y - pTopLeft.y;

      // Obstacle body
      ctx.fillStyle = '#1e293b';
      ctx.fillRect(pTopLeft.x, pTopLeft.y, w, h);

      ctx.strokeStyle = 'rgba(255, 255, 255, 0.12)';
      ctx.lineWidth = 1.5;
      ctx.strokeRect(pTopLeft.x, pTopLeft.y, w, h);

      // Hatching effect
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.04)';
      ctx.lineWidth = 1;
      for (let offset = 0; offset < w + h; offset += 12) {
        ctx.beginPath();
        ctx.moveTo(pTopLeft.x + offset, pTopLeft.y);
        ctx.lineTo(pTopLeft.x + offset - h, pTopLeft.y + h);
        ctx.stroke();
      }
    });
  }

  function drawRobot(name, r) {
    const p = worldToCanvas(r.x, r.y);
    const color = robotColors[name] || '#ffffff';

    // Draw trajectory trail
    if (r.trail && r.trail.length > 1) {
      ctx.beginPath();
      ctx.strokeStyle = color;
      ctx.lineWidth = 1.5;
      ctx.globalAlpha = 0.3;
      r.trail.forEach((tp, idx) => {
        const cp = worldToCanvas(tp.x, tp.y);
        if (idx === 0) ctx.moveTo(cp.x, cp.y);
        else ctx.lineTo(cp.x, cp.y);
      });
      ctx.stroke();
      ctx.globalAlpha = 1.0;
    }

    // Robot body (radius = 0.3m -> in pixels)
    const radiusPx = 0.3 * scale;

    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.rotate(-r.yaw); // Canvas rotates clockwise for positive angles

    // Glow aura
    ctx.beginPath();
    ctx.arc(0, 0, radiusPx + 4, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.globalAlpha = 0.15;
    ctx.fill();
    ctx.globalAlpha = 1.0;

    // Outer Circle
    ctx.beginPath();
    ctx.arc(0, 0, radiusPx, 0, Math.PI * 2);
    ctx.fillStyle = '#0f172a';
    ctx.fill();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2.5;
    ctx.stroke();

    // Orientation arrow
    ctx.beginPath();
    ctx.moveTo(radiusPx * 0.8, 0);
    ctx.lineTo(0, -radiusPx * 0.45);
    ctx.lineTo(radiusPx * 0.2, 0);
    ctx.lineTo(0, radiusPx * 0.45);
    ctx.closePath();
    ctx.fillStyle = color;
    ctx.fill();

    ctx.restore();

    // Robot Name Label
    ctx.fillStyle = '#ffffff';
    ctx.font = '11px JetBrains Mono';
    ctx.textAlign = 'center';
    ctx.fillText(name.toUpperCase(), p.x, p.y - radiusPx - 8);
  }

  function render() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    drawGrid();
    drawChokeZone();
    drawObstacles();

    ['amr1', 'amr2', 'amr3'].forEach(name => {
      drawRobot(name, fleet[name]);
    });

    requestAnimationFrame(render);
  }

  // Update UI Card Values
  function updateTelemetryCards() {
    ['amr1', 'amr2', 'amr3'].forEach(id => {
      const r = fleet[id];
      const chokeDist = Math.hypot(r.x, r.y);

      const posEl = document.getElementById(`pos-${id}`);
      const velEl = document.getElementById(`vel-${id}`);
      const yawEl = document.getElementById(`yaw-${id}`);
      const distEl = document.getElementById(`dist-${id}`);
      const stateEl = document.getElementById(`state-${id}`);
      const battTextEl = document.getElementById(`batt-text-${id}`);
      const battFillEl = document.getElementById(`batt-fill-${id}`);

      if (posEl) posEl.textContent = `${r.x.toFixed(2)}, ${r.y.toFixed(2)} m`;
      if (velEl) velEl.textContent = `${r.speed.toFixed(2)} m/s`;
      if (yawEl) yawEl.textContent = `${(r.yaw * 180 / Math.PI).toFixed(1)}°`;
      if (distEl) distEl.textContent = `${chokeDist.toFixed(2)} m`;

      if (stateEl) {
        stateEl.textContent = r.state;
        stateEl.className = 'robot-state-tag';
        if (r.state === 'YIELDING') stateEl.classList.add('yielding');
        else if (r.state === 'BLOCKED') stateEl.classList.add('blocked');
        else if (r.speed > 0.05) stateEl.classList.add('navigating');
      }

      if (battTextEl) battTextEl.textContent = `${r.battery.toFixed(1)}% (${r.voltage.toFixed(1)} V)`;
      if (battFillEl) {
        battFillEl.style.width = `${Math.min(100, Math.max(0, r.battery))}%`;
        battFillEl.className = 'battery-fill';
        if (r.battery < 25) battFillEl.classList.add('danger');
        else if (r.battery < 50) battFillEl.classList.add('warning');
      }
    });
  }

  // Event Feed Helper
  function logEvent(type, timeStr, message) {
    const feed = document.getElementById('feedContainer');
    if (!feed) return;

    const item = document.createElement('div');
    item.className = `feed-item event-${type}`;
    item.innerHTML = `
      <span class="event-time">${timeStr}</span>
      <span class="event-msg">${message}</span>
    `;
    feed.insertBefore(item, feed.firstChild);

    // Trim old events if feed exceeds 40 items
    while (feed.children.length > 40) {
      feed.removeChild(feed.lastChild);
    }
  }

  // Connect to Telemetry EventSource (SSE)
  function setupEventStream() {
    const statusEl = document.getElementById('connStatus');
    const evtSource = new EventSource('/events');

    evtSource.onopen = function() {
      if (statusEl) statusEl.textContent = 'TELEMETRY STREAM ACTIVE';
    };

    evtSource.onerror = function() {
      if (statusEl) statusEl.textContent = 'RECONNECTING STREAM...';
    };

    evtSource.addEventListener('telemetry', function(e) {
      try {
        const data = JSON.parse(e.data);
        // Telemetry contains { amr1: {...}, amr2: {...}, amr3: {...} }
        for (const [id, rdata] of Object.entries(data)) {
          if (fleet[id]) {
            fleet[id].x = rdata.x;
            fleet[id].y = rdata.y;
            fleet[id].yaw = rdata.yaw;
            fleet[id].speed = rdata.speed;
            fleet[id].battery = rdata.battery;
            fleet[id].voltage = rdata.voltage;
            fleet[id].state = rdata.state;

            // Maintain trail
            fleet[id].trail.push({ x: rdata.x, y: rdata.y });
            if (fleet[id].trail.length > 50) fleet[id].trail.shift();
          }
        }
        updateTelemetryCards();
      } catch (err) {
        console.error('Failed to parse telemetry', err);
      }
    });

    evtSource.addEventListener('cnp_event', function(e) {
      try {
        const ev = JSON.parse(e.data);
        logEvent(ev.type, ev.time, ev.message);
      } catch (err) {
        console.error('Failed to parse CNP event', err);
      }
    });
  }

  // Live Clock
  setInterval(() => {
    const now = new Date();
    const clockEl = document.getElementById('liveClock');
    if (clockEl) {
      clockEl.textContent = now.toTimeString().split(' ')[0] + ' UTC';
    }
  }, 1000);

  // Dispatch Action Handlers
  document.getElementById('btnRunOpposing')?.addEventListener('click', () => {
    fetch('/api/dispatch_opposing', { method: 'POST' })
      .then(res => res.json())
      .then(data => logEvent('info', 'COMMAND', data.status || 'Opposing choke goals dispatched'))
      .catch(err => logEvent('auction', 'ERR', 'Failed to dispatch: ' + err));
  });

  document.getElementById('btnTriggerBlockage')?.addEventListener('click', () => {
    fetch('/api/trigger_blockage', { method: 'POST' })
      .then(res => res.json())
      .then(data => logEvent('auction', 'AUCTION', 'Synthetic blockage triggered on AMR 1 &bull; CNP Auction initiated'))
      .catch(err => logEvent('auction', 'ERR', 'Failed to trigger blockage: ' + err));
  });

  document.getElementById('btnRecharge')?.addEventListener('click', () => {
    ['amr1', 'amr2', 'amr3'].forEach(id => { fleet[id].trail = []; });
    logEvent('info', 'RESET', 'Fleet breadcrumb trails cleared');
  });

  document.getElementById('btnClearFeed')?.addEventListener('click', () => {
    const feed = document.getElementById('feedContainer');
    if (feed) feed.innerHTML = '';
  });

  // Start Animation Loop & Stream
  render();
  setupEventStream();
})();
