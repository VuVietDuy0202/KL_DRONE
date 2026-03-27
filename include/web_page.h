#ifndef WEB_PAGE_H
#define WEB_PAGE_H

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <title>Drone PID Tuning</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <meta charset="UTF-8">
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body { 
      font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, 'Helvetica Neue', Arial, sans-serif;
      background: #f5f5f5;
      padding: 15px;
      max-width: 600px;
      margin: 0 auto;
    }
    h1 { font-size: 28px; color: #333; margin-bottom: 5px; }
    .subtitle { font-size: 13px; color: #999; margin-bottom: 20px; }
    
    .section {
      background: white;
      border-radius: 10px;
      padding: 15px;
      margin-bottom: 15px;
      box-shadow: 0 2px 8px rgba(0,0,0,0.08);
    }
    .section-title {
      font-size: 14px;
      font-weight: 600;
      color: #333;
      margin-bottom: 12px;
      border-bottom: 2px solid #007BFF;
      padding-bottom: 8px;
    }
    
    .pid-row {
      display: flex;
      align-items: center;
      margin-bottom: 10px;
      gap: 10px;
    }
    .pid-label { width: 100px; font-size: 13px; font-weight: 500; color: #555; }
    .pid-value { 
      width: 60px;
      padding: 5px 8px;
      background: #f9f9f9;
      border: 1px solid #ddd;
      border-radius: 4px;
      text-align: center;
      font-size: 12px;
      font-weight: 600;
      color: #007BFF;
    }
    .pid-input {
      flex: 0.5;
      padding: 6px 8px;
      border: 1px solid #ddd;
      border-radius: 4px;
      font-size: 13px;
      font-weight: 500;
    }
    
    .button-group {
      display: flex;
      gap: 10px;
      margin-top: 15px;
    }
    .btn {
      flex: 1;
      padding: 12px;
      border: none;
      border-radius: 6px;
      font-size: 14px;
      font-weight: 600;
      cursor: pointer;
      transition: all 0.3s;
    }
    .btn-primary {
      background: #28a745;
      color: white;
    }
    .btn-primary:hover { background: #218838; }
    .btn-primary:active { transform: scale(0.98); }
    
    .btn-danger {
      background: #dc3545;
      color: white;
    }
    .btn-danger:hover { background: #c82333; }
    .btn-danger:active { transform: scale(0.98); }
    
    .status-msg {
      margin-top: 10px;
      padding: 8px;
      border-radius: 4px;
      font-size: 12px;
      font-weight: 500;
      display: none;
    }
    .status-msg.show { display: block; }
    .status-msg.success { background: #d4edda; color: #155724; border: 1px solid #c3e6cb; }
    .status-msg.error { background: #f8d7da; color: #721c24; border: 1px solid #f5c6cb; }
    
    .data-box {
      background: #f9f9f9;
      border-radius: 8px;
      padding: 12px;
      font-size: 13px;
      line-height: 1.8;
    }
    .data-row {
      display: flex;
      justify-content: space-between;
      align-items: center;
      margin-bottom: 8px;
    }
    .data-label { font-weight: 500; color: #555; }
    .data-value { font-weight: 600; color: #007BFF; }
    
    .status-indicator {
      display: inline-block;
      width: 10px;
      height: 10px;
      border-radius: 50%;
      margin-right: 5px;
    }
    
    .motor-display {
      display: grid;
      grid-template-columns: 1fr 1fr;
      gap: 10px;
      margin-top: 10px;
    }
    .motor-item {
      background: #f0f0f0;
      padding: 8px;
      border-radius: 6px;
      text-align: center;
      font-size: 12px;
    }
    .motor-item-label { font-weight: 600; color: #333; margin-bottom: 4px; }
    .motor-item-value { font-size: 18px; font-weight: 700; color: #007BFF; }
  </style>
  <script>
    let pidValues = {};

    function setVal(name) {
      const input = document.getElementById(name);
      if (!input) return;
      const val = input.value;
      if (val === '') return;
      fetch('/get?' + name + '=' + val)
        .then(r => r.text())
        .then(() => showStatus(name + ' = ' + val, 'success'))
        .catch(e => showStatus('Error: ' + e, 'error'));
    }

    function loadPID() {
      showStatus('Loading PID...', 'success');
      fetch('/loadpid')
        .then(r => r.text())
        .then(t => {
          showStatus('PID loaded! Disarming...', 'success');
          setTimeout(() => location.reload(), 800);
        })
        .catch(e => showStatus('Error: ' + e, 'error'));
    }

    function disarm() {
      showStatus('Disarming...', 'success');
      setTimeout(() => location.reload(), 500);
    }

    function showStatus(msg, type) {
      const status = document.getElementById('status');
      status.textContent = msg;
      status.className = 'status-msg show ' + type;
      setTimeout(() => status.classList.remove('show'), 3000);
    }

    function updateData() {
      fetch('/data')
        .then(r => r.json())
        .then(d => {
          document.getElementById('m1').textContent = d.m1;
          document.getElementById('m2').textContent = d.m2;
          document.getElementById('m3').textContent = d.m3;
          document.getElementById('m4').textContent = d.m4;
          document.getElementById('roll').textContent  = d.roll.toFixed(1);
          document.getElementById('pitch').textContent = d.pitch.toFixed(1);
          
          const fs = document.getElementById('fs');
          fs.textContent = d.fs ? 'FAILSAFE' : 'OK';
          fs.parentElement.style.color = d.fs ? '#dc3545' : '#28a745';
          
          const arm = document.getElementById('arm');
          arm.textContent = d.arm ? 'ARMED' : 'DISARMED';
          arm.parentElement.style.color = d.arm ? '#28a745' : '#999';
        })
        .catch(e => console.log('Data fetch error:', e));
    }

    function loadCurrentPID() {
      fetch('/status')
        .then(r => r.json())
        .then(d => {
          pidValues = d;
          document.getElementById('pGain').value = parseFloat(d.pGain) || 0.625;
          document.getElementById('iGain').value = parseFloat(d.iGain) || 0;
          document.getElementById('dGain').value = parseFloat(d.dGain) || 0.008;
          
          document.getElementById('pAGain').value = parseFloat(d.pAGain) || 2.0;
          document.getElementById('iAGain').value = parseFloat(d.iAGain) || 0;
          document.getElementById('dAGain').value = parseFloat(d.dAGain) || 0.007;
          
          document.getElementById('pYaw').value = parseFloat(d.pYaw) || 4.0;
          document.getElementById('iYaw').value = parseFloat(d.iYaw) || 0;
          document.getElementById('dYaw').value = parseFloat(d.dYaw) || 0;
          
          document.getElementById('tc').value = parseFloat(d.tc) || 0.004;
        })
        .catch(e => console.log('Status fetch error:', e));
    }

    setInterval(updateData, 200);
    window.onload = () => {
      loadCurrentPID();
      updateData();
    };
  </script>
</head><body>
  <h1>🚁 Drone PID Controller</h1>
  <p class="subtitle">Real-time PID Parameter Adjustment</p>

  <div class="section">
    <div class="section-title">⚙️ Rate PID</div>
    <div class="pid-row">
      <span class="pid-label">P Rate</span>
      <input type="number" id="pGain" step="any" class="pid-input" placeholder="0.625">
      <button class="btn btn-primary" onclick="setVal('pGain')" style="flex: 0.25; padding: 6px 12px;">Set</button>
    </div>
    <div class="pid-row">
      <span class="pid-label">I Rate</span>
      <input type="number" id="iGain" step="any" class="pid-input" placeholder="0">
      <button class="btn btn-primary" onclick="setVal('iGain')" style="flex: 0.25; padding: 6px 12px;">Set</button>
    </div>
    <div class="pid-row">
      <span class="pid-label">D Rate</span>
      <input type="number" id="dGain" step="any" class="pid-input" placeholder="0.008">
      <button class="btn btn-primary" onclick="setVal('dGain')" style="flex: 0.25; padding: 6px 12px;">Set</button>
    </div>
  </div>

  <div class="section">
    <div class="section-title">📐 Angle PID</div>
    <div class="pid-row">
      <span class="pid-label">P Angle</span>
      <input type="number" id="pAGain" step="any" class="pid-input" placeholder="2.0">
      <button class="btn btn-primary" onclick="setVal('pAGain')" style="flex: 0.25; padding: 6px 12px;">Set</button>
    </div>
    <div class="pid-row">
      <span class="pid-label">I Angle</span>
      <input type="number" id="iAGain" step="any" class="pid-input" placeholder="0">
      <button class="btn btn-primary" onclick="setVal('iAGain')" style="flex: 0.25; padding: 6px 12px;">Set</button>
    </div>
    <div class="pid-row">
      <span class="pid-label">D Angle</span>
      <input type="number" id="dAGain" step="any" class="pid-input" placeholder="0.007">
      <button class="btn btn-primary" onclick="setVal('dAGain')" style="flex: 0.25; padding: 6px 12px;">Set</button>
    </div>
  </div>

  <div class="section">
    <div class="section-title">🔄 Yaw Control</div>
    <div class="pid-row">
      <span class="pid-label">P Yaw</span>
      <input type="number" id="pYaw" step="any" class="pid-input" placeholder="4.0">
      <button class="btn btn-primary" onclick="setVal('pYaw')" style="flex: 0.25; padding: 6px 12px;">Set</button>
    </div>
    <div class="pid-row">
      <span class="pid-label">I Yaw</span>
      <input type="number" id="iYaw" step="any" class="pid-input" placeholder="0">
      <button class="btn btn-primary" onclick="setVal('iYaw')" style="flex: 0.25; padding: 6px 12px;">Set</button>
    </div>
    <div class="pid-row">
      <span class="pid-label">D Yaw</span>
      <input type="number" id="dYaw" step="any" class="pid-input" placeholder="0">
      <button class="btn btn-primary" onclick="setVal('dYaw')" style="flex: 0.25; padding: 6px 12px;">Set</button>
    </div>
    <div class="pid-row">
      <span class="pid-label">Cycle (s)</span>
      <input type="number" id="tc" step="any" class="pid-input" placeholder="0.004">
      <button class="btn btn-primary" onclick="setVal('tc')" style="flex: 0.25; padding: 6px 12px;">Set</button>
    </div>
  </div>

  <div class="section">
    <div class="button-group">
      <button class="btn btn-primary" onclick="loadPID()">✅ Load PID</button>
      <button class="btn btn-danger" onclick="disarm()">🛑 Disarm</button>
    </div>
    <div class="status-msg" id="status"></div>
  </div>

  <div class="section">
    <div class="section-title">📊 Live Status</div>
    <div class="motor-display">
      <div class="motor-item">
        <div class="motor-item-label">M1 (µs)</div>
        <div class="motor-item-value" id="m1">---</div>
      </div>
      <div class="motor-item">
        <div class="motor-item-label">M2 (µs)</div>
        <div class="motor-item-value" id="m2">---</div>
      </div>
      <div class="motor-item">
        <div class="motor-item-label">M3 (µs)</div>
        <div class="motor-item-value" id="m3">---</div>
      </div>
      <div class="motor-item">
        <div class="motor-item-label">M4 (µs)</div>
        <div class="motor-item-value" id="m4">---</div>
      </div>
    </div>
    <div class="data-box" style="margin-top: 10px;">
      <div class="data-row">
        <span class="data-label">Roll Angle:</span>
        <span class="data-value"><span id="roll">---</span>°</span>
      </div>
      <div class="data-row">
        <span class="data-label">Pitch Angle:</span>
        <span class="data-value"><span id="pitch">---</span>°</span>
      </div>
      <div class="data-row">
        <span class="data-label">System Status:</span>
        <span class="data-value" id="fs">---</span>
      </div>
      <div class="data-row">
        <span class="data-label">Arm Status:</span>
        <span class="data-value" id="arm">---</span>
      </div>
    </div>
  </div>
</body></html>)rawliteral";

#endif // WEB_PAGE_H
