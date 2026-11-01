(function (root, factory) {
  const api = factory(typeof module === 'object' && module.exports ? require('./core.js') : root.MotorCore);
  if (typeof module === 'object' && module.exports) module.exports = api;
  else root.MotorSerial = api;
})(globalThis, function (Core) {
  'use strict';
  class SerialLink {
    constructor({ onFrame, onStatus, onError, onLegacy }) {
      Object.assign(this, { onFrame, onStatus, onError, onLegacy });
      this.connected = false; this.failed = false; this.generation = 0; this.emergencyLatched = false;
      this.queue = Promise.resolve(); this.writer = null; this.reader = null;
    }
    async connect() {
      if (!('serial' in navigator)) throw new Error('请在 Chrome 或 Edge 中打开本地地址，使用 USB 串口连接');
      this.port = await navigator.serial.requestPort();
      await this.port.open({ baudRate: 115200 });
      this.writer = this.port.writable.getWriter();
      this.connected = true; this.failed = false; this.legacyNotified = false;
      this.generation++; this.queue = Promise.resolve();
      this.onStatus(); this.readTask = this.readLoop();
    }
    send(line) {
      const isTestCommand = line.startsWith('A,') || line.startsWith('D,') || line.startsWith('F,') || line.startsWith('V,');
      const cancellable = isTestCommand || line.startsWith('B,');
      if (line === 'G' || line === 'S') this.generation++;
      if (line === 'S') this.emergencyLatched = true;
      if (line === 'G' || isTestCommand) this.emergencyLatched = false;
      const generation = this.generation, writer = this.writer;
      if (!this.connected || !writer || this.failed) return Promise.resolve(false);
      const task = this.queue.then(async () => {
        if (!this.connected || writer !== this.writer) return false;
        if (cancellable && generation !== this.generation) return false;
        await writer.write(new TextEncoder().encode(line + '\n'));
        return true;
      });
      this.queue = task.catch(error => {
        this.failed = true; this.onError(error); this.onStatus(); return false;
      });
      return this.queue;
    }
    async readLoop() {
      const reader = this.port.readable.getReader(); this.reader = reader;
      const decoder = new TextDecoder();
      const lines = new Core.LineReader(line => {
        const frame = Core.parseTelemetry(line);
        if (frame) {
          if (frame.mode === 0) this.emergencyLatched = true;
          this.onFrame(frame);
        }
        else if (line.startsWith('T,') && !this.legacyNotified) {
          this.legacyNotified = true; this.onLegacy();
        }
      });
      try {
        while (this.connected) {
          const { value, done } = await reader.read();
          if (done) break;
          lines.push(decoder.decode(value, { stream: true }));
        }
      } catch (error) {
        if (this.connected) this.onError(error);
      } finally {
        reader.releaseLock(); this.reader = null;
        if (this.connected) {
          this.connected = false; this.generation++;
          await this.closeResources(); this.onStatus();
        }
      }
    }
    async disconnect() {
      if (!this.connected) return;
      // 正常断开仅撤销增量，保留固件中的持续重力补偿。
      if (!this.emergencyLatched) await this.send('G');
      this.connected = false; this.generation++;
      if (this.reader) await this.reader.cancel().catch(() => {});
      if (this.readTask) await this.readTask;
      await this.closeResources(); this.onStatus();
    }
    async closeResources() {
      await this.queue;
      if (this.writer) { this.writer.releaseLock(); this.writer = null; }
      if (this.port) { await this.port.close().catch(() => {}); this.port = null; }
    }
  }
  return { SerialLink };
});
