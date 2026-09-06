// Preload bridge: safe serial API for the renderer (contextIsolation: true).
const { contextBridge, ipcRenderer } = require('electron');

contextBridge.exposeInMainWorld('espSerial', {
  isElectron: true,
  list: () => ipcRenderer.invoke('serial:list'),
  connect: (portPath, baudRate) => ipcRenderer.invoke('serial:connect', { path: portPath, baudRate }),
  disconnect: () => ipcRenderer.invoke('serial:disconnect'),
  send: (text) => ipcRenderer.invoke('serial:send', text),
  onData: (cb) => {
    const handler = (_event, line) => { try { cb(line); } catch (_) {} };
    ipcRenderer.on('serial:data', handler);
    return () => ipcRenderer.removeListener('serial:data', handler);
  },
  onClosed: (cb) => {
    const handler = (_event, info) => { try { cb(info); } catch (_) {} };
    ipcRenderer.on('serial:closed', handler);
    return () => ipcRenderer.removeListener('serial:closed', handler);
  }
});
