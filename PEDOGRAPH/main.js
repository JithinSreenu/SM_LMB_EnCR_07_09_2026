const { app, BrowserWindow, Menu, ipcMain, session } = require('electron');
const path = require('path');

let SerialPortCtor = null;
let ReadlineParserCtor = null;
try {
  ({ SerialPort: SerialPortCtor } = require('serialport'));
} catch (e) {
  console.warn('[serial] serialport module not available:', e.message);
}
try {
  ({ ReadlineParser: ReadlineParserCtor } = require('@serialport/parser-readline'));
} catch (e) {
  console.warn('[serial] parser-readline not available:', e.message);
}

let espPort = null;
let espParser = null;
let espPath = null;

function closeEspPort() {
  return new Promise((resolve) => {
    try {
      if (espParser) { try { espParser.removeAllListeners(); } catch (_) {} espParser = null; }
      if (espPort) {
        const p = espPort;
        espPort = null;
        espPath = null;
        if (p.isOpen) {
          p.close((err) => resolve(!err));
        } else {
          try { p.destroy(); } catch (_) {}
          resolve(true);
        }
      } else {
        resolve(true);
      }
    } catch (_) {
      espPort = null; espParser = null; espPath = null;
      resolve(false);
    }
  });
}

function setupSerialIpc(mainWindow) {
  ipcMain.handle('serial:list', async () => {
    if (!SerialPortCtor) return { ok: false, error: 'serialport module not installed', ports: [] };
    try {
      const ports = await SerialPortCtor.list();
      // Sort: USB serial first, then by path
      ports.sort((a, b) => {
        const score = (p) => {
          const s = ((p.manufacturer || '') + ' ' + (p.friendlyName || '') + ' ' + (p.pnpId || '')).toLowerCase();
          if (/ch340|cp210| prolific|ftdi|usb.serial|usb-serial|arduino|esp32|silabs|wch/.test(s)) return 0;
          if (/bluetooth/.test(s)) return 2;
          return 1;
        };
        return score(a) - score(b) || String(a.path).localeCompare(String(b.path));
      });
      return { ok: true, ports };
    } catch (e) {
      return { ok: false, error: e.message, ports: [] };
    }
  });

  ipcMain.handle('serial:connect', async (event, { path: portPath, baudRate }) => {
    if (!SerialPortCtor) return { ok: false, error: 'serialport module not installed' };
    if (!portPath) return { ok: false, error: 'No port selected' };
    const baud = parseInt(baudRate, 10) || 115200;
    await closeEspPort();
    try {
      espPort = new SerialPortCtor({ path: portPath, baudRate: baud, autoOpen: false });
      espPath = portPath;
      await new Promise((resolve, reject) => {
        espPort.open((err) => (err ? reject(err) : resolve()));
      });
      // NOTE: firmware compact monitor lines terminate with CR only ('...\r',
      // no LF), while CSV/status lines use LF. Split on both so nothing sticks.
      const emitLine = (line) => {
        const s = String(line).trim();
        if (!s) return;
        try {
          const win = BrowserWindow.fromWebContents(event.sender);
          (win || mainWindow).webContents.send('serial:data', s);
        } catch (_) {}
      };
      let rxBuf = '';
      const pushChunk = (chunk) => {
        rxBuf += chunk.toString('utf8');
        // split on any run of CR/LF, keep the remainder buffered
        const parts = rxBuf.split(/[\r\n]+/);
        rxBuf = parts.pop();
        for (const p of parts) emitLine(p);
        // guard against a stuck partial line growing forever
        if (rxBuf.length > 4096) { emitLine(rxBuf); rxBuf = ''; }
      };
      if (ReadlineParserCtor) {
        // Feed the parser raw chunks ourselves so CR-only lines are handled too.
        espParser = null;
        espPort.on('data', pushChunk);
      } else {
        espPort.on('data', pushChunk);
      }
      espPort.on('close', () => {
        try {
          const win = BrowserWindow.fromWebContents(event.sender);
          (win || mainWindow).webContents.send('serial:closed', { path: portPath });
        } catch (_) {}
      });
      espPort.on('error', (err) => {
        try {
          const win = BrowserWindow.fromWebContents(event.sender);
          (win || mainWindow).webContents.send('serial:data', 'ERR: ' + err.message);
        } catch (_) {}
      });
      return { ok: true, path: portPath, baudRate: baud };
    } catch (e) {
      await closeEspPort();
      return { ok: false, error: e.message || String(e) };
    }
  });

  ipcMain.handle('serial:disconnect', async () => {
    await closeEspPort();
    return { ok: true };
  });

  ipcMain.handle('serial:send', async (event, text) => {
    if (!espPort || !espPort.isOpen) return { ok: false, error: 'Not connected' };
    const line = String(text == null ? '' : text);
    try {
      await new Promise((resolve, reject) => {
        espPort.write(line + '\n', (err) => (err ? reject(err) : resolve()));
      });
      espPort.drain(() => {});
      return { ok: true };
    } catch (e) {
      return { ok: false, error: e.message || String(e) };
    }
  });
}

function createWindow() {
  const mainWindow = new BrowserWindow({
    width: 1280,
    height: 800,
    minWidth: 900,
    minHeight: 600,
    title: 'Pedograph v2.0',
    backgroundColor: '#050510',
    autoHideMenuBar: true,
    webPreferences: {
      nodeIntegration: false,
      contextIsolation: true,
      enableWebSerial: true,
      preload: path.join(__dirname, 'preload.js')
    },
    icon: null,
    titleBarStyle: 'default',
    show: false
  });

  // --- Web Serial permission support (fallback path) ---
  try {
    const ses = session.defaultSession;
    ses.setPermissionCheckHandler((webContents, permission) => {
      if (permission === 'serial') return true;
      return false;
    });
    if (typeof ses.setDevicePermissionHandler === 'function') {
      ses.setDevicePermissionHandler((details) => {
        if (details.deviceType === 'serial') return true;
        return false;
      });
    }
    // Auto-approve first available port so requestPort() never hangs with no dialog.
    // (Primary path is Node serialport via IPC, so this is only a fallback.)
    ses.on('select-serial-port', (event, portList, callback) => {
      event.preventDefault();
      if (portList && portList.length > 0) callback(portList[0].portId);
      else callback('');
    });
  } catch (e) {
    console.warn('[serial] session permission setup failed:', e.message);
  }

  setupSerialIpc(mainWindow);

  mainWindow.loadFile('index.html');

  mainWindow.once('ready-to-show', () => {
    mainWindow.show();
  });

  // Custom menu with keyboard shortcuts
  const menu = Menu.buildFromTemplate([
    {
      label: 'Control',
      submenu: [
        { label: 'Start Output', accelerator: 'Space', click: () => mainWindow.webContents.executeJavaScript('toggleOutput()') },
        { label: 'Stop Output', accelerator: 'Escape', click: () => mainWindow.webContents.executeJavaScript('stopOutput()') },
        { type: 'separator' },
        { label: 'Reset Counters', accelerator: 'Ctrl+R', click: () => mainWindow.webContents.executeJavaScript('resetCounters()') },
        { type: 'separator' },
        { label: 'Exit', accelerator: 'Alt+F4', click: () => app.quit() }
      ]
    },
    {
      label: 'Presets',
      submenu: [
        { label: 'Walk', accelerator: 'Ctrl+1', click: () => mainWindow.webContents.executeJavaScript("applyPreset('walk')") },
        { label: 'Run', accelerator: 'Ctrl+2', click: () => mainWindow.webContents.executeJavaScript("applyPreset('run')") },
        { label: 'Jog', accelerator: 'Ctrl+3', click: () => mainWindow.webContents.executeJavaScript("applyPreset('jog')") },
        { label: 'Limp', accelerator: 'Ctrl+4', click: () => mainWindow.webContents.executeJavaScript("applyPreset('limp')") },
        { label: 'Stair', accelerator: 'Ctrl+5', click: () => mainWindow.webContents.executeJavaScript("applyPreset('stair')") },
        { label: 'Climb', accelerator: 'Ctrl+6', click: () => mainWindow.webContents.executeJavaScript("applyPreset('climb')") },
        { label: 'Stand', accelerator: 'Ctrl+7', click: () => mainWindow.webContents.executeJavaScript("applyPreset('stand')") },
        { label: 'Sit', accelerator: 'Ctrl+8', click: () => mainWindow.webContents.executeJavaScript("applyPreset('sit')") },
        { label: 'Test', accelerator: 'Ctrl+9', click: () => mainWindow.webContents.executeJavaScript("applyPreset('test')") }
      ]
    },
    {
      label: 'View',
      submenu: [
        { label: 'Dual Wave', accelerator: 'Ctrl+D', click: () => mainWindow.webContents.executeJavaScript("setView('dual')") },
        { label: 'CH1 Only', click: () => mainWindow.webContents.executeJavaScript("setView('ch1')") },
        { label: 'CH2 Only', click: () => mainWindow.webContents.executeJavaScript("setView('ch2')") },
        { label: 'Gait Phases', accelerator: 'Ctrl+G', click: () => mainWindow.webContents.executeJavaScript("setView('gait')") },
        { type: 'separator' },
        { label: 'Export CSV', accelerator: 'Ctrl+E', click: () => mainWindow.webContents.executeJavaScript('exportCSV()') }
      ]
    },
    {
      label: 'Help',
      submenu: [
        { label: 'About', click: () => mainWindow.webContents.executeJavaScript('showHelp()') }
      ]
    }
  ]);
  Menu.setApplicationMenu(menu);

  mainWindow.on('closed', () => { closeEspPort(); });
}

app.whenReady().then(createWindow);

app.on('window-all-closed', () => {
  closeEspPort().finally(() => app.quit());
});

app.on('activate', () => {
  if (BrowserWindow.getAllWindows().length === 0) {
    createWindow();
  }
});
