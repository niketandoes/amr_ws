/**
 * AMR Fleet Command Live Dashboard Client Application
 * Fully Dynamic: Auto-discovers robots and renders map from ROS 2 PGM/YAML.
 */
(function() {
  const canvas = document.getElementById('warehouseCanvas');
  const ctx = canvas.getContext('2d');
  const PADDING = 40;

  // Map state
  let mapImage = null;
  let mapOriginX = -6.0;
  let mapOriginY = -6.0;
  let mapRes = 0.05;
  let mapWidth = 240;
  let mapHeight = 240;
  let scale = 1;
  let drawOriginX = 0;
  let drawOriginY = 0;

  // Fleet state
  const fleet = {}; 
  const colorPalette = ['#3b82f6', '#ec4899', '#10b981', '#f59e0b', '#8b5cf6', '#ef4444'];
  let colorIndex = 0;

  async function loadMap() {
    try {
      // 1. Fetch YAML
      const yamlRes = await fetch('/api/map.yaml');
      if (yamlRes.ok) {
        const yamlText = await yamlRes.text();
        const resMatch = yamlText.match(/resolution:\s*([0-9.]+)/);
        if (resMatch) mapRes = parseFloat(resMatch[1]);
        const originMatch = yamlText.match(/origin:\s*\[([-\d.]+),\s*([-\d.]+)/);
        if (originMatch) {
            mapOriginX = parseFloat(originMatch[1]);
            mapOriginY = parseFloat(originMatch[2]);
        }
      }
      
      // 2. Fetch PGM
      const pgmRes = await fetch('/api/map.pgm');
      if (pgmRes.ok) {
        const pgmText = await pgmRes.text();
        const lines = pgmText.split('\n').map(l => l.trim()).filter(l => l.length > 0 && !l.startsWith('#'));
        if (lines[0] === 'P2') {
          const dims = lines[1].split(/\s+/);
          mapWidth = parseInt(dims[0]);
          mapHeight = parseInt(dims[1]);
          const maxVal = parseInt(lines[2]);
          
          const pixels = [];
          for (let i = 3; i < lines.length; i++) {
             const vals = lines[i].split(/\s+/);
             for(let v of vals) {
                if(v) pixels.push(parseInt(v));
             }
          }
          
          const imgData = new ImageData(mapWidth, mapHeight);
          for(let i = 0; i < pixels.length; i++) {
             const val = pixels[i];
             // 255 = free (dark slate), 0 = occupied (lighter slate)
             const r = val === 255 ? 15 : 71;
             const g = val === 255 ? 23 : 85;
             const b = val === 255 ? 42 : 105;
             imgData.data[i*4] = r;
             imgData.data[i*4+1] = g;
             imgData.data[i*4+2] = b;
             imgData.data[i*4+3] = 255;
          }
          
          const offCanvas = document.createElement('canvas');
          offCanvas.width = mapWidth;
          offCanvas.height = mapHeight;
          const offCtx = offCanvas.getContext('2d');
          offCtx.putImageData(imgData, 0, 0);
          mapImage = offCanvas;
        }
      }

      updateScale();
    } catch(err) {
      console.error("Failed to load map:", err);
    }
  }

  function updateScale() {
     const worldW = mapWidth * mapRes;
     const worldH = mapHeight * mapRes;
     scale = Math.min(
       (canvas.width - PADDING * 2) / worldW,
       (canvas.height - PADDING * 2) / worldH
     );
     drawOriginX = (canvas.width - worldW * scale) / 2;
     drawOriginY = (canvas.height - worldH * scale) / 2;
  }
  
  function worldToCanvas(wx, wy) {
     const mapX = wx - mapOriginX;
     const mapY = wy - mapOriginY;
     return {
        x: drawOriginX + (mapX * scale),
        y: canvas.height - drawOriginY - (mapY * scale) // flip Y for canvas
     }
  }

  function createRobotCard(id, color) {
    const container = document.getElementById('robotCardsContainer');
    if (!container) return;
    
    const div = document.createElement('div');
    div.className = 'robot-card';
    div.id = `card-${id}`;
    div.innerHTML = `
      <div class="robot-card-top">
        <div class="robot-meta">
          <span class="robot-badge" style="background-color: ${color}20; color: ${color}; border: 1px solid ${color}40;">${id.toUpperCase()}</span>
          <span class="robot-role">Dynamic Fleet Unit</span>
        </div>
        <span class="robot-state-tag" id="state-${id}">UNKNOWN</span>
      </div>
      <div class="robot-metrics-grid">
        <div class="metric-item">
          <label>POSITION</label>
          <div class="metric-val" id="pos-${id}">0.00, 0.00 m</div>
        </div>
        <div class="metric-item">
          <label>VELOCITY</label>
          <div class="metric-val" id="vel-${id}">0.00 m/s</div>
        </div>
        <div class="metric-item">
          <label>HEADING</label>
          <div class="metric-val" id="yaw-${id}">0.0&deg;</div>
        </div>
        <div class="metric-item">
          <label>CHOKE DIST</label>
          <div class="metric-val" id="dist-${id}">0.00 m</div>
        </div>
      </div>
      <div class="battery-bar-container">
        <div class="battery-label-row">
          <span>BATTERY SOC</span>
          <span id="batt-text-${id}">100.0% (24.0 V)</span>
        </div>
        <div class="battery-track">
          <div class="battery-fill" id="batt-fill-${id}" style="width: 100%; background-color: ${color};"></div>
        </div>
      </div>
    `;
    container.appendChild(div);
  }

  function drawGrid() {
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.03)';
    ctx.lineWidth = 1;
    for (let x = Math.ceil(mapOriginX); x <= mapOriginX + (mapWidth*mapRes); x += 1) {
      const p1 = worldToCanvas(x, mapOriginY);
      const p2 = worldToCanvas(x, mapOriginY + (mapHeight*mapRes));
      ctx.beginPath(); ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y); ctx.stroke();
    }
    for (let y = Math.ceil(mapOriginY); y <= mapOriginY + (mapHeight*mapRes); y += 1) {
      const p1 = worldToCanvas(mapOriginX, y);
      const p2 = worldToCanvas(mapOriginX + (mapWidth*mapRes), y);
      ctx.beginPath(); ctx.moveTo(p1.x, p1.y); ctx.lineTo(p2.x, p2.y); ctx.stroke();
    }
    const center = worldToCanvas(0, 0);
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)';
    ctx.beginPath();
    ctx.moveTo(center.x - 10, center.y); ctx.lineTo(center.x + 10, center.y);
    ctx.moveTo(center.x, center.y - 10); ctx.lineTo(center.x, center.y + 10);
    ctx.stroke();
  }

  function drawRobot(name, r) {
    const p = worldToCanvas(r.x, r.y);
    const color = r.color;

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

    const radiusPx = 0.3 * scale;
    ctx.save();
    ctx.translate(p.x, p.y);
    ctx.rotate(-r.yaw); // canvas Y is flipped, so flip rotation

    ctx.beginPath();
    ctx.arc(0, 0, radiusPx + 4, 0, Math.PI * 2);
    ctx.fillStyle = color;
    ctx.globalAlpha = 0.15;
    ctx.fill();
    ctx.globalAlpha = 1.0;

    ctx.beginPath();
    ctx.arc(0, 0, radiusPx, 0, Math.PI * 2);
    ctx.fillStyle = '#0f172a';
    ctx.fill();
    ctx.strokeStyle = color;
    ctx.lineWidth = 2.5;
    ctx.stroke();

    ctx.beginPath();
    ctx.moveTo(radiusPx * 0.8, 0);
    ctx.lineTo(0, -radiusPx * 0.45);
    ctx.lineTo(radiusPx * 0.2, 0);
    ctx.lineTo(0, radiusPx * 0.45);
    ctx.closePath();
    ctx.fillStyle = color;
    ctx.fill();
    ctx.restore();

    ctx.fillStyle = '#ffffff';
    ctx.font = '11px JetBrains Mono';
    ctx.textAlign = 'center';
    ctx.fillText(name.toUpperCase(), p.x, p.y - radiusPx - 8);
  }

  function render() {
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    if (mapImage) {
       // Draw the parsed PGM image
       const worldW = mapWidth * mapRes;
       const worldH = mapHeight * mapRes;
       // We draw the image filling the area, but because canvas Y is down, 
       // the image (which has y=0 at top) must be drawn normally?
       // Wait, PGM row 0 is y_max (top). So we draw it as is!
       ctx.drawImage(mapImage, drawOriginX, drawOriginY, worldW * scale, worldH * scale);
    }

    drawGrid();

    Object.keys(fleet).forEach(name => {
      drawRobot(name, fleet[name]);
    });

    requestAnimationFrame(render);
  }

  function updateTelemetryCards() {
    Object.keys(fleet).forEach(id => {
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
      }
    });
  }

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
    while (feed.children.length > 40) feed.removeChild(feed.lastChild);
  }

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
        
        // 1. Identify active IDs from Server
        const activeIds = Object.keys(data);
        
        // 2. Remove any robots in our frontend fleet that are NOT in activeIds
        for (const knownId of Object.keys(fleet)) {
           if (!activeIds.includes(knownId)) {
               console.log("Robot disappeared from telemetry, purging:", knownId);
               const card = document.getElementById(`card-${knownId}`);
               if (card) {
                   card.remove();
               }
               delete fleet[knownId];
           }
        }

        // 3. Update existing or dynamically register new robots
        for (const [id, rdata] of Object.entries(data)) {
          if (!fleet[id]) {
            // Dynamically register new robot!
            const color = colorPalette[colorIndex % colorPalette.length];
            colorIndex++;
            fleet[id] = { color: color, trail: [] };
            createRobotCard(id, color);
          }
          fleet[id].x = rdata.x;
          fleet[id].y = rdata.y;
          fleet[id].yaw = rdata.yaw;
          fleet[id].speed = rdata.speed;
          fleet[id].battery = rdata.battery;
          fleet[id].voltage = rdata.voltage;
          fleet[id].state = rdata.state;

          fleet[id].trail.push({ x: rdata.x, y: rdata.y });
          if (fleet[id].trail.length > 50) fleet[id].trail.shift();
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

  setInterval(() => {
    const now = new Date();
    const clockEl = document.getElementById('liveClock');
    if (clockEl) clockEl.textContent = now.toTimeString().split(' ')[0] + ' UTC';
  }, 1000);

  document.getElementById('btnRunOpposing')?.addEventListener('click', () => {
    fetch('/api/dispatch_opposing', { method: 'POST' })
      .then(res => res.json())
      .then(data => logEvent('info', 'COMMAND', data.status))
      .catch(err => logEvent('auction', 'ERR', 'Failed to dispatch: ' + err));
  });

  document.getElementById('btnTriggerBlockage')?.addEventListener('click', () => {
    fetch('/api/trigger_blockage', { method: 'POST' })
      .then(res => res.json())
      .then(data => logEvent('auction', 'AUCTION', 'Synthetic blockage triggered on AMR 1 &bull; CNP Auction initiated'))
      .catch(err => logEvent('auction', 'ERR', 'Failed to trigger blockage: ' + err));
  });

  document.getElementById('btnRecharge')?.addEventListener('click', () => {
    fetch('/api/reset_poses', { method: 'POST' })
      .then(res => res.json())
      .then(data => {
        Object.keys(fleet).forEach(id => { fleet[id].trail = []; });
        logEvent('info', 'RESET', data.status || 'Fleet poses reset to initial spawn positions');
      })
      .catch(err => logEvent('auction', 'ERR', 'Failed to reset poses: ' + err));
  });

  document.getElementById('btnClearFeed')?.addEventListener('click', () => {
    const feed = document.getElementById('feedContainer');
    if (feed) feed.innerHTML = '';
  });

  // Init
  loadMap().then(() => {
      render();
      setupEventStream();
  });
})();
