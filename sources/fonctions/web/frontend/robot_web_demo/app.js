const wsUrl = `ws://${window.location.host}/ws`;

const ui = {
    wsBadge: document.getElementById('wsBadge'),
    modeBadge: document.getElementById('modeBadge'),
    missionBadge: document.getElementById('missionBadge'),
    overlayBadge: document.getElementById('overlayBadge'),
    statusText: document.getElementById('statusText'),
    autoState: document.getElementById('autoState'),
    targetVal: document.getElementById('targetVal'),
    fpsVal: document.getElementById('fpsVal'),
    batteryVal: document.getElementById('batteryVal'),
    headingVal: document.getElementById('headingVal'),
    northVal: document.getElementById('northVal'),
    rollVal: document.getElementById('rollVal'),
    pitchVal: document.getElementById('pitchVal'),
    yawVal: document.getElementById('yawVal'),
    gyroVecVal: document.getElementById('gyroVecVal'),
    leftThruster: document.getElementById('leftThruster'),
    rightThruster: document.getElementById('rightThruster'),
    conveyor: document.getElementById('conveyor'),
    leftThrusterValue: document.getElementById('leftThrusterValue'),
    rightThrusterValue: document.getElementById('rightThrusterValue'),
    conveyorValue: document.getElementById('conveyorValue'),
    camPanVal: document.getElementById('camPanVal'),
    camTiltVal: document.getElementById('camTiltVal'),
    expRange: document.getElementById('expRange'),
    brightRange: document.getElementById('brightRange'),
    contrastRange: document.getElementById('contrastRange'),
    manualLockBanner: document.getElementById('manualLockBanner'),
    logBox: document.getElementById('logBox'),
    detectionsBox: document.getElementById('detectionsBox'),
    videoFeed: document.getElementById('videoFeed'),
    compassCanvas: document.getElementById('compassCanvas'),
    gyroCanvas: document.getElementById('gyroCanvas')
};

let ws = null;
let currentMode = 'manual';
let manualLocked = false;

function logLine(msg) {
    const line = document.createElement('div');
    line.className = 'logLine';
    const ts = new Date().toLocaleTimeString();
    line.textContent = `[${ts}] ${msg}`;
    ui.logBox.prepend(line);
    while (ui.logBox.childElementCount > 60) {
        ui.logBox.removeChild(ui.logBox.lastChild);
    }
}

function connectWs() {
    ws = new WebSocket(wsUrl);

    ws.addEventListener('open', () => {
        ui.wsBadge.textContent = 'WS connected';
        ui.wsBadge.classList.remove('badge-off');
        ui.wsBadge.classList.add('badge-on');
        logLine('WebSocket connecté.');
    });

    ws.addEventListener('close', () => {
        ui.wsBadge.textContent = 'WS disconnected';
        ui.wsBadge.classList.remove('badge-on');
        ui.wsBadge.classList.add('badge-off');
        logLine('WebSocket fermé. Reconnexion...');
        setTimeout(connectWs, 1200);
    });

    ws.addEventListener('message', (ev) => {
        try {
            const data = JSON.parse(ev.data);
            if (data.type === 'telemetry' && data.payload) {
                applyTelemetry(data.payload);
            } else if (data.type === 'echo') {
                logLine(`Echo: ${JSON.stringify(data.payload)}`);
            } else if (data.type === 'info') {
                logLine(data.message || 'info');
            }
        } catch (err) {
            console.warn('Message WS illisible', err, ev.data);
        }
    });
}

function sendWs(obj) {
    if (!ws || ws.readyState !== WebSocket.OPEN) {
        logLine('Commande ignorée: WebSocket non connecté.');
        return;
    }
    ws.send(JSON.stringify(obj));
}

function sendIfManual(payload) {
    if (manualLocked) {
        logLine('Commande manuelle ignorée en mode AUTO.');
        return;
    }
    sendWs(payload);
}

function applyTelemetry(t) {
    currentMode = t.mode || 'manual';
    manualLocked = !!t.drive_controls_locked;

    ui.modeBadge.textContent = `MODE ${currentMode.toUpperCase()}`;
    ui.modeBadge.classList.toggle('badge-manual', currentMode === 'manual');
    ui.modeBadge.classList.toggle('badge-auto', currentMode === 'auto');

    ui.missionBadge.textContent = t.mission_enabled ? 'MISSION ON' : 'MISSION OFF';
    ui.missionBadge.classList.toggle('badge-on', !!t.mission_enabled);
    ui.missionBadge.classList.toggle('badge-off', !t.mission_enabled);

    ui.overlayBadge.textContent = t.overlay_enabled ? 'Overlay ON' : 'Overlay OFF';
    ui.overlayBadge.classList.toggle('badge-on', !!t.overlay_enabled);
    ui.overlayBadge.classList.toggle('badge-off', !t.overlay_enabled);

    ui.statusText.textContent = `status: ${t.status_text || '--'}`;
    ui.autoState.textContent = `auto state: ${t.auto_state || '--'}`;
    ui.targetVal.textContent = `Target: ${(t.vision?.target || '--')} (${((t.vision?.confidence || 0) * 100).toFixed(0)}%)`;
    ui.fpsVal.textContent = `FPS: ${(t.vision?.fps || 0).toFixed(1)}`;
    ui.batteryVal.textContent = `Battery: ${(t.battery_v || 0).toFixed(2)} V`;

    ui.headingVal.textContent = `${(t.compass?.heading_deg || 0).toFixed(1)}°`;
    ui.northVal.textContent = `${(t.compass?.mag_north_deg || 0).toFixed(1)}°`;
    ui.rollVal.textContent = `${(t.imu?.roll_deg || 0).toFixed(1)}°`;
    ui.pitchVal.textContent = `${(t.imu?.pitch_deg || 0).toFixed(1)}°`;
    ui.yawVal.textContent = `${(t.imu?.yaw_rate_dps || 0).toFixed(1)} °/s`;
    ui.gyroVecVal.textContent = `${(t.imu?.gyro_x_dps || 0).toFixed(1)} / ${(t.imu?.gyro_y_dps || 0).toFixed(1)} / ${(t.imu?.gyro_z_dps || 0).toFixed(1)}`;

    ui.leftThruster.value = Math.round(t.motors?.left_pct || 0);
    ui.rightThruster.value = Math.round(t.motors?.right_pct || 0);
    ui.conveyor.value = Math.round(t.motors?.conveyor_pct || 0);
    ui.camPanVal.textContent = `Pan ${Math.round(t.motors?.cam_pan_deg || 0)}°`;
    ui.camTiltVal.textContent = `Tilt ${Math.round(t.motors?.cam_tilt_deg || 0)}°`;

    ui.expRange.value = Math.round(t.camera?.exposure || 0);
    ui.brightRange.value = Math.round(t.camera?.brightness || 50);
    ui.contrastRange.value = Math.round(t.camera?.contrast || 50);

    refreshDisplayedValues();
    updateManualLockUi();
    renderDetections(t.vision?.detections || []);
    drawCompass(t.compass?.heading_deg || 0);
    drawGyro(t.imu?.roll_deg || 0, t.imu?.pitch_deg || 0, t.imu?.yaw_rate_dps || 0);
}

function updateManualLockUi() {
    ui.manualLockBanner.classList.toggle('hidden', !manualLocked);
    for (const elem of document.querySelectorAll('[data-manual], #leftThruster, #rightThruster, #conveyor, [data-cam-step], #btnCamCenter, #btnForward, #btnBackward, #btnRotateLeft, #btnRotateRight, #btnStopAll')) {
        elem.disabled = manualLocked;
    }
}

function refreshDisplayedValues() {
    ui.leftThrusterValue.textContent = `${ui.leftThruster.value} %`;
    ui.rightThrusterValue.textContent = `${ui.rightThruster.value} %`;
    ui.conveyorValue.textContent = `${ui.conveyor.value} %`;
}

function renderDetections(detections) {
    ui.detectionsBox.innerHTML = '';
    if (!detections.length) {
        const empty = document.createElement('div');
        empty.className = 'detItem empty';
        empty.textContent = 'Aucune détection.';
        ui.detectionsBox.appendChild(empty);
        return;
    }
    detections.forEach((det, idx) => {
        const div = document.createElement('div');
        div.className = `detItem ${det.primary ? 'primary' : ''}`;
        div.innerHTML = `
            <strong>${idx + 1}. ${det.label}</strong>
            <span>${(det.confidence * 100).toFixed(0)}%</span>
            <small>x=${det.x}, y=${det.y}, w=${det.w}, h=${det.h}</small>
        `;
        ui.detectionsBox.appendChild(div);
    });
}

function sendThrusters() {
    sendIfManual({
        type: 'command',
        action: 'thrusters',
        left_pct: Number(ui.leftThruster.value),
        right_pct: Number(ui.rightThruster.value)
    });
}

function sendConveyor() {
    sendIfManual({
        type: 'command',
        action: 'conveyor',
        pct: Number(ui.conveyor.value)
    });
}

function sendCameraSetting(name, value) {
    sendWs({
        type: 'command',
        action: 'camera_setting',
        setting: name,
        value: Number(value)
    });
}

function drawCompass(heading) {
    const ctx = ui.compassCanvas.getContext('2d');
    const w = ui.compassCanvas.width;
    const h = ui.compassCanvas.height;
    const cx = w / 2;
    const cy = h / 2;
    const r = Math.min(w, h) * 0.40;

    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = '#0f172a';
    ctx.fillRect(0, 0, w, h);

    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.stroke();

    ctx.fillStyle = '#cbd5e1';
    ctx.font = '16px Arial';
    ctx.textAlign = 'center';
    ctx.fillText('N', cx, cy - r - 10);
    ctx.fillText('S', cx, cy + r + 22);
    ctx.fillText('E', cx + r + 18, cy + 6);
    ctx.fillText('W', cx - r - 18, cy + 6);

    for (let i = 0; i < 360; i += 30) {
        const rad = (i - 90) * Math.PI / 180;
        const x1 = cx + Math.cos(rad) * (r - 10);
        const y1 = cy + Math.sin(rad) * (r - 10);
        const x2 = cx + Math.cos(rad) * r;
        const y2 = cy + Math.sin(rad) * r;
        ctx.strokeStyle = '#64748b';
        ctx.beginPath();
        ctx.moveTo(x1, y1);
        ctx.lineTo(x2, y2);
        ctx.stroke();
    }

    const rad = (heading - 90) * Math.PI / 180;
    ctx.strokeStyle = '#22c55e';
    ctx.lineWidth = 4;
    ctx.beginPath();
    ctx.moveTo(cx, cy);
    ctx.lineTo(cx + Math.cos(rad) * (r - 16), cy + Math.sin(rad) * (r - 16));
    ctx.stroke();

    ctx.fillStyle = '#f8fafc';
    ctx.font = '700 22px Arial';
    ctx.fillText(`${heading.toFixed(1)}°`, cx, cy + 8);
}

function drawGyro(roll, pitch, yawRate) {
    const ctx = ui.gyroCanvas.getContext('2d');
    const w = ui.gyroCanvas.width;
    const h = ui.gyroCanvas.height;
    ctx.clearRect(0, 0, w, h);
    ctx.fillStyle = '#0f172a';
    ctx.fillRect(0, 0, w, h);

    const centerY = h / 2;
    const centerX = w / 2;
    const rollRad = roll * Math.PI / 180;
    const skyOffset = pitch * 2.5;

    ctx.save();
    ctx.translate(centerX, centerY);
    ctx.rotate(-rollRad);
    ctx.fillStyle = '#0ea5e9';
    ctx.fillRect(-220, -240 + skyOffset, 440, 220);
    ctx.fillStyle = '#854d0e';
    ctx.fillRect(-220, -20 + skyOffset, 440, 220);
    ctx.strokeStyle = '#ffffff';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(-220, skyOffset);
    ctx.lineTo(220, skyOffset);
    ctx.stroke();
    ctx.restore();

    ctx.strokeStyle = '#f8fafc';
    ctx.lineWidth = 3;
    ctx.beginPath();
    ctx.moveTo(centerX - 70, centerY);
    ctx.lineTo(centerX + 70, centerY);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(centerX, centerY - 20);
    ctx.lineTo(centerX, centerY + 20);
    ctx.stroke();

    ctx.fillStyle = '#e2e8f0';
    ctx.font = '16px Arial';
    ctx.fillText(`Roll: ${roll.toFixed(1)}°`, 18, 28);
    ctx.fillText(`Pitch: ${pitch.toFixed(1)}°`, 18, 52);
    ctx.fillText(`Yaw rate: ${yawRate.toFixed(1)} °/s`, 18, 76);
}

ui.leftThruster.addEventListener('input', () => { refreshDisplayedValues(); sendThrusters(); });
ui.rightThruster.addEventListener('input', () => { refreshDisplayedValues(); sendThrusters(); });
ui.conveyor.addEventListener('input', () => { refreshDisplayedValues(); sendConveyor(); });

for (const elem of [ui.expRange, ui.brightRange, ui.contrastRange]) {
    elem.addEventListener('input', () => sendCameraSetting(elem.id, elem.value));
}

for (const btn of document.querySelectorAll('[data-cam-step]')) {
    btn.addEventListener('click', () => {
        sendIfManual({
            type: 'command',
            action: 'camera_step',
            axis: btn.dataset.axis,
            steps: Number(btn.dataset.steps)
        });
    });
}

document.getElementById('btnCamCenter').addEventListener('click', () => {
    sendIfManual({ type: 'command', action: 'camera_center' });
});

document.getElementById('btnMissionStart').addEventListener('click', () => {
    sendWs({ type: 'command', action: 'mission', cmd: 'start' });
});
document.getElementById('btnMissionStop').addEventListener('click', () => {
    sendWs({ type: 'command', action: 'mission', cmd: 'stop' });
});
document.getElementById('btnStopAll').addEventListener('click', () => {
    ui.leftThruster.value = 0;
    ui.rightThruster.value = 0;
    ui.conveyor.value = 0;
    refreshDisplayedValues();
    sendIfManual({ type: 'command', action: 'stop_all' });
});

document.getElementById('btnManualMode').addEventListener('click', () => {
    sendWs({ type: 'command', action: 'mode_set', mode: 'manual' });
});
document.getElementById('btnAutoMode').addEventListener('click', () => {
    sendWs({ type: 'command', action: 'mode_set', mode: 'auto' });
});
document.getElementById('btnOverlayOn').addEventListener('click', () => {
    sendWs({ type: 'command', action: 'overlay_toggle', enabled: true });
});
document.getElementById('btnOverlayOff').addEventListener('click', () => {
    sendWs({ type: 'command', action: 'overlay_toggle', enabled: false });
});

document.getElementById('btnForward').addEventListener('click', () => {
    ui.leftThruster.value = 60;
    ui.rightThruster.value = 60;
    refreshDisplayedValues();
    sendThrusters();
});
document.getElementById('btnBackward').addEventListener('click', () => {
    ui.leftThruster.value = -45;
    ui.rightThruster.value = -45;
    refreshDisplayedValues();
    sendThrusters();
});
document.getElementById('btnRotateLeft').addEventListener('click', () => {
    ui.leftThruster.value = -35;
    ui.rightThruster.value = 35;
    refreshDisplayedValues();
    sendThrusters();
});
document.getElementById('btnRotateRight').addEventListener('click', () => {
    ui.leftThruster.value = 35;
    ui.rightThruster.value = -35;
    refreshDisplayedValues();
    sendThrusters();
});

ui.videoFeed.addEventListener('error', () => {
    logLine('Le flux MJPEG ne répond pas encore.');
});

refreshDisplayedValues();
drawCompass(0);
drawGyro(0, 0, 0);
connectWs();
