// AVROS Console - Joystick WebSocket Client

const WS_URL = `wss://${location.host}/ws`;

// Cache DOM elements
const statusEl = document.getElementById('status');
const valuesEl = document.getElementById('values');
const estopBtn = document.getElementById('estop');
const autoBtn = document.getElementById('autobtn');
const joystickZone = document.getElementById('joystick-zone');
const modeButtons = document.querySelectorAll('.modes button');

let ws = null;
let estop = false;
let mode = 'D';
let autonomous = false;
let joystickX = 0;
let joystickY = 0;
let sendInterval = null;

function clamp(value, min, max) {
    return Math.max(min, Math.min(max, value));
}

// ===== WEBSOCKET =====
function connect() {
    ws = new WebSocket(WS_URL);

    ws.onopen = () => {
        statusEl.textContent = 'Connected';
        statusEl.className = '';
        send({ type: 'mode', value: mode });
        send({ type: 'estop', value: estop });
        send({ type: 'autonomous', value: autonomous });
        startSending();
    };

    ws.onclose = () => {
        statusEl.textContent = 'Disconnected';
        statusEl.className = 'disconnected';
        stopSending();
        setTimeout(connect, 1000);
    };

    ws.onmessage = (e) => {
        const data = JSON.parse(e.data);
        if (data.error) {
            statusEl.textContent = data.error;
            statusEl.className = 'disconnected';
            stopSending();
            return;
        }
        valuesEl.textContent = `T: ${data.t.toFixed(2)} | S: ${data.s.toFixed(2)} | B: ${data.b.toFixed(2)}`;
    };
}

function send(data) {
    if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(data));
    }
}

function startSending() {
    if (sendInterval) return;
    sendInterval = setInterval(() => {
        if (autonomous) {
            // Hand control to Nav2: stop asserting actuator_command so it goes
            // stale and /cmd_vel takes over. Keepalive keeps telemetry flowing.
            send({ type: 'keepalive' });
        } else {
            send({ type: 'control', x: joystickX, y: joystickY });
        }
    }, 50);
}

function stopSending() {
    if (sendInterval) {
        clearInterval(sendInterval);
        sendInterval = null;
    }
}

// ===== JOYSTICK =====
function resetJoystick() {
    joystickX = 0;
    joystickY = 0;
}

function createNippleJoystick() {
    const joystick = nipplejs.create({
        zone: joystickZone,
        mode: 'static',
        position: { left: '50%', top: '50%' },
        color: '#1976d2',
        size: 150,
        restOpacity: 0.7
    });

    joystick.on('move', (evt, data) => {
        if (!data || !data.vector) return;
        const maxDist = 75;
        joystickX = clamp(data.vector.x * data.distance / maxDist, -1, 1);
        joystickY = clamp(data.vector.y * data.distance / maxDist, -1, 1);
    });

    joystick.on('end', resetJoystick);
}

function createFallbackJoystick() {
    const maxDist = 75;
    const handle = document.createElement('div');
    handle.style.cssText = [
        'position:absolute',
        'left:50%',
        'top:50%',
        'width:90px',
        'height:90px',
        'margin-left:-45px',
        'margin-top:-45px',
        'border-radius:50%',
        'background:#1976d2',
        'box-shadow:0 0 24px rgba(25,118,210,0.6)',
        'pointer-events:none',
        'touch-action:none'
    ].join(';');
    joystickZone.appendChild(handle);

    let activePointerId = null;

    function updateHandle() {
        handle.style.transform = `translate(${joystickX * maxDist}px, ${-joystickY * maxDist}px)`;
    }

    function updateFromPointer(event) {
        const rect = joystickZone.getBoundingClientRect();
        const centerX = rect.left + rect.width / 2;
        const centerY = rect.top + rect.height / 2;
        joystickX = clamp((event.clientX - centerX) / maxDist, -1, 1);
        joystickY = clamp((centerY - event.clientY) / maxDist, -1, 1);
        updateHandle();
    }

    joystickZone.addEventListener('pointerdown', (event) => {
        activePointerId = event.pointerId;
        joystickZone.setPointerCapture(event.pointerId);
        updateFromPointer(event);
    });

    joystickZone.addEventListener('pointermove', (event) => {
        if (event.pointerId !== activePointerId) return;
        updateFromPointer(event);
    });

    function releaseFallbackJoystick() {
        activePointerId = null;
        resetJoystick();
        updateHandle();
    }

    joystickZone.addEventListener('pointerup', releaseFallbackJoystick);
    joystickZone.addEventListener('pointercancel', releaseFallbackJoystick);
    joystickZone.addEventListener('lostpointercapture', releaseFallbackJoystick);
}

if (window.nipplejs) {
    createNippleJoystick();
} else {
    createFallbackJoystick();
}

// ===== E-STOP =====
estopBtn.addEventListener('click', () => {
    estop = !estop;
    estopBtn.classList.toggle('active', estop);
    estopBtn.textContent = estop ? 'E-STOP ACTIVE' : 'E-STOP';
    send({ type: 'estop', value: estop });
    // E-stop is not autonomous — drop the AUTO toggle (server forces it too).
    if (estop && autonomous) {
        autonomous = false;
        updateAutoBtn();
    }
});

// ===== AUTONOMOUS TOGGLE (IGVC §I.2 safety light) =====
function updateAutoBtn() {
    autoBtn.classList.toggle('active', autonomous);
    autoBtn.textContent = autonomous ? 'AUTO ON' : 'AUTO';
}

autoBtn.addEventListener('click', () => {
    autonomous = !autonomous;
    updateAutoBtn();
    send({ type: 'autonomous', value: autonomous });
});

// ===== MODE BUTTONS =====
modeButtons.forEach(btn => {
    btn.addEventListener('click', () => {
        modeButtons.forEach(b => b.classList.remove('active'));
        btn.classList.add('active');
        mode = btn.dataset.mode;
        send({ type: 'mode', value: mode });
    });
});

// ===== START =====
connect();
