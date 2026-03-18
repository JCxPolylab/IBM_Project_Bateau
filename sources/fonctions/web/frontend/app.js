const wsBadge = document.getElementById('wsBadge');
const fpsBadge = document.getElementById('fpsBadge');
const logBox = document.getElementById('logBox');
const videoFeed = document.getElementById('videoFeed');

const leftThruster = document.getElementById('leftThruster');
const rightThruster = document.getElementById('rightThruster');
const conveyor = document.getElementById('conveyor');

const compassCanvas = document.getElementById('compassCanvas');
const gyroCanvas = document.getElementById('gyroCanvas');

const ui = {
    headingVal: document.getElementById('headingVal'),
    northVal: document.getElementById('northVal'),
    rollVal: document.getElementById('rollVal'),
    pitchVal: document.getElementById('pitchVal'),
    yawVal: document.getElementById('yawVal'),
    targetVal: document.getElementById('targetVal'),
    leftThrusterValue: document.getElementById('leftThrusterValue'),
    rightThrusterValue: document.getElementById('rightThrusterValue'),
    conveyorValue: document.getElementById('conveyorValue')
};

let ws = null;
let latestTelemetry = null;
let reconnectTimer = null;

function logLine(text) {
    const line = document.createElement('div');
    line.className = 'logLine';
    line.textContent = `[${new Date().toLocaleTimeString()}] ${text}`;
    logBox.prepend(line);
}

function setWsState(connected) {
    wsBadge.textContent = connected ? 'WS connected' : 'WS disconnected';
    wsBadge.className = connected ? 'badge badge-on' : 'badge badge-off';
}

function wsUrl() {
    const proto = location.protocol === 'https:' ? 'wss' : 'ws';
    return `${proto}://${location.host}/ws`;
}

function sendWs(obj) {
    if (!ws || ws.readyState !== WebSocket.OPEN) return;
    ws.send(JSON.stringify(obj));
}

function connectWs() {
    clearTimeout(reconnectTimer);
    ws = new WebSocket(wsUrl());

    ws.addEventListener('open', () => {
        setWsState(true);
        logLine('WebSocket connected');
    });

    ws.addEventListener('message', (ev) => {
        try {
            const msg = JSON.parse(ev.data);
            if (msg.type === 'telemetry' && msg.payload) {
                latestTelemetry = msg.payload;
                updateTelemetry(msg.payload);
            } else {
                logLine(`WS RX ${ev.data}`);
            }
        } catch (err) {
            logLine(`WS parse error: ${err}`);
        }
    });

    ws.addEventListener('close', () => {
        setWsState(false);
        logLine('WebSocket closed, reconnecting...');
        reconnectTimer = setTimeout(connectWs, 1200);
    });

    ws.addEventListener('error', () => {
        setWsState(false);
    });
}

function updateTelemetry(t) {
    const heading = Number(t.heading_deg || 0);
    const north = Number(t.mag_north_deg || 0);
    const roll = Number(t.gyro?.roll_deg || 0);
    const pitch = Number(t.gyro?.pitch_deg || 0);
    const yawRate = Number(t.gyro?.yaw_rate_dps || 0);
    const target = t.vision?.target || 'none';
    const confidence = Number(t.vision?.confidence || 0);
    const fps = Number(t.vision?.fps || 0);

    ui.headingVal.textContent = `${heading.toFixed(1)}°`;
    ui.northVal.textContent = `${north.toFixed(1)}°`;
    ui.rollVal.textContent = `${roll.toFixed(1)}°`;
    ui.pitchVal.textContent = `${pitch.toFixed(1)}°`;
    ui.yawVal.textContent = `${yawRate.toFixed(1)} °/s`;
    ui.targetVal.textContent = `${target} (${(confidence * 100).toFixed(0)}%)`;
    fpsBadge.textContent = `FPS ${fps.toFixed(1)}`;

    if (t.motors) {
        leftThruster.value = Math.round(t.motors.left_pct || 0);
        rightThruster.value = Math.round(t.motors.right_pct || 0);
        conveyor.value = Math.round(t.motors.conveyor_pct || 0);
        refreshDisplayedValues();
    }

    drawCompass(heading);
    drawGyro(roll, pitch, yawRate);
}

function refreshDisplayedValues() {
    ui.leftThrusterValue.textContent = `${leftThruster.value} %`;
    ui.rightThrusterValue.textContent = `${rightThruster.value} %`;
    ui.conveyorValue.textContent = `${conveyor.value} %`;
}

function sendThrusters() {
    sendWs({
        type: 'command',
        action: 'thrusters',
        left_pct: Number(leftThruster.value),
        right_pct: Number(rightThruster.value)
    });
}

function sendConveyor() {
    sendWs({
        type: 'command',
        action: 'conveyor',
        pct: Number(conveyor.value)
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
    const ctx = compassCanvas.getContext('2d');
    const w = compassCanvas.width;
    const h = compassCanvas.height;
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
    const ctx = gyroCanvas.getContext('2d');
    const w = gyroCanvas.width;
    const h = gyroCanvas.height;
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

leftThruster.addEventListener('input', () => { refreshDisplayedValues(); sendThrusters(); });
rightThruster.addEventListener('input', () => { refreshDisplayedValues(); sendThrusters(); });
conveyor.addEventListener('input', () => { refreshDisplayedValues(); sendConveyor(); });

for (const elem of [
    document.getElementById('expRange'),
    document.getElementById('brightRange'),
    document.getElementById('contrastRange')
]) {
    elem.addEventListener('input', () => sendCameraSetting(elem.id, elem.value));
}

for (const btn of document.querySelectorAll('[data-cam-step]')) {
    btn.addEventListener('click', () => {
        sendWs({
            type: 'command',
            action: 'camera_step',
            axis: btn.dataset.axis,
            steps: Number(btn.dataset.steps)
        });
    });
}

document.getElementById('btnCamCenter').addEventListener('click', () => {
    sendWs({ type: 'command', action: 'camera_center' });
});

document.getElementById('btnMissionStart').addEventListener('click', () => {
    sendWs({ type: 'command', action: 'mission', cmd: 'start' });
});
document.getElementById('btnMissionStop').addEventListener('click', () => {
    sendWs({ type: 'command', action: 'mission', cmd: 'stop' });
});

document.getElementById('btnForward').addEventListener('click', () => {
    leftThruster.value = 60;
    rightThruster.value = 60;
    refreshDisplayedValues();
    sendThrusters();
});

document.getElementById('btnBackward').addEventListener('click', () => {
    leftThruster.value = -45;
    rightThruster.value = -45;
    refreshDisplayedValues();
    sendThrusters();
});

document.getElementById('btnRotateLeft').addEventListener('click', () => {
    leftThruster.value = -35;
    rightThruster.value = 35;
    refreshDisplayedValues();
    sendThrusters();
});

document.getElementById('btnRotateRight').addEventListener('click', () => {
    leftThruster.value = 35;
    rightThruster.value = -35;
    refreshDisplayedValues();
    sendThrusters();
});

document.getElementById('btnStopAll').addEventListener('click', () => {
    leftThruster.value = 0;
    rightThruster.value = 0;
    conveyor.value = 0;
    refreshDisplayedValues();
    sendThrusters();
    sendConveyor();
});

videoFeed.addEventListener('error', () => {
    logLine('Le flux MJPEG ne répond pas encore.');
});

refreshDisplayedValues();
drawCompass(0);
drawGyro(0, 0, 0);
connectWs();
