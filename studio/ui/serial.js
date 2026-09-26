// ORION Studio — liaison avec le robot réel (Web Serial, Chrome/Edge sur ordinateur).
// Protocole ORION-ASCII (voir core/protocol.js et docs/11-firmware-protocole.md).

import { h, btn, toast } from './dom.js';
import { encodeCommand, parseLine, streamSetpoint, packMitCommand, hex, MIT_FRAMES } from '../core/protocol.js';
import { toFirmwareSetCommands } from '../core/export.js';

const R2D = 180 / Math.PI;

export class RobotLink {
  constructor(app) {
    this.app = app;
    this.port = null;
    this.writer = null;
    this.reader = null;
    this.connected = false;
    this.streaming = false;
    this.mirror = false;
    this.seq = 0;
    this.status = null;
    this.version = '';
    this.listeners = [];
  }

  get supported() { return typeof navigator !== 'undefined' && 'serial' in navigator; }

  async connect() {
    if (!this.supported) { toast('Web Serial indisponible : utilisez Chrome ou Edge sur ordinateur, avec le logiciel ouvert en local.'); return; }
    try {
      this.port = await navigator.serial.requestPort();
      await this.port.open({ baudRate: 115200 });
    } catch (e) {
      toast(`Connexion annulée : ${e.message}`);
      return;
    }
    this.connected = true;
    const enc = new TextEncoderStream();
    enc.readable.pipeTo(this.port.writable).catch(() => {});
    this.writer = enc.writable.getWriter();
    this.readLoop();
    this.app.log('Port série ouvert', 'ok');
    this.send(encodeCommand('PING', [], true));
    this.send(encodeCommand('STREAM', [50], true));
    this.emit();
  }

  async readLoop() {
    const dec = new TextDecoderStream();
    this.port.readable.pipeTo(dec.writable).catch(() => {});
    this.reader = dec.readable.getReader();
    let buf = '';
    try {
      for (;;) {
        const { value, done } = await this.reader.read();
        if (done) break;
        buf += value;
        let i;
        while ((i = buf.indexOf('\n')) >= 0) {
          const line = buf.slice(0, i).replace(/\r$/, '');
          buf = buf.slice(i + 1);
          if (line) this.onLine(line);
        }
      }
    } catch (e) {
      this.app.log(`Lecture série interrompue : ${e.message}`, 'warn');
    }
    this.connected = false;
    this.streaming = false;
    this.emit();
  }

  async disconnect() {
    this.streaming = false;
    try { await this.reader?.cancel(); } catch { /* */ }
    try { await this.writer?.close(); } catch { /* */ }
    try { await this.port?.close(); } catch { /* */ }
    this.connected = false;
    this.app.log('Port série fermé');
    this.emit();
  }

  send(line) {
    if (!this.connected || !this.writer) return false;
    this.writer.write(line.endsWith('\n') ? line : `${line}\n`).catch(() => {});
    if (!line.startsWith('SP ')) this.app.monitor?.(line.trim(), 'tx');
    return true;
  }

  cmd(name, args = []) { return this.send(encodeCommand(name, args, true)); }

  onLine(line) {
    const msg = parseLine(line);
    if (!msg) return;
    if (msg.type === 'st') {
      this.status = msg;
      if (this.mirror) this.app.showMeasured(msg.q.map((d) => d / R2D));
    } else {
      this.app.monitor?.(line, msg.type === 'err' ? 'error' : 'rx');
      if (msg.type === 'ok' && msg.args[0] === 'PONG') { this.version = msg.args.slice(1).join(' '); this.emit(); }
      if (msg.type === 'err') this.app.log(`Robot : erreur ${msg.code} ${msg.message}`, 'error');
      if (msg.type === 'evt') this.app.log(`Robot : ${msg.name} ${msg.args.join(' ')}`, msg.name === 'FAULT' ? 'error' : 'ok');
    }
  }

  /** Envoie la consigne courante du simulateur (jumeau numérique). */
  tick() {
    if (!this.connected || !this.streaming) return;
    const d = this.app.sim.des;
    this.send(streamSetpoint(this.seq++, d.q.map((v) => +(v * R2D).toFixed(4)), d.qd.map((v) => +(v * R2D).toFixed(3))));
  }

  sendConfig() {
    const cmds = toFirmwareSetCommands(this.app.params);
    cmds.forEach((c) => this.send(`${c}\n`));
    this.send(encodeCommand('SAVE', [], true));
    toast(`${cmds.length} paramètres envoyés au firmware`);
  }

  onChange(fn) { this.listeners.push(fn); }
  emit() { this.listeners.forEach((f) => f(this)); }
}

/** Panneau « Robot réel ». */
export class RobotPanel {
  constructor(container, app) {
    this.app = app;
    this.c = container;
    this.link = app.link;
    this.build();
    this.link.onChange(() => this.update());
  }

  build() {
    const L = this.link;
    this.c.replaceChildren();
    this.chip = h('span.chip', 'Déconnecté');
    this.connectBtn = btn('Connecter (USB)', () => (L.connected ? L.disconnect() : L.connect()), { cls: 'btn--sm btn--primary', icon: 'usb' });
    this.streamBtn = btn('Jumeau numérique', () => { L.streaming = !L.streaming; this.update(); if (L.streaming) toast('Le robot réel suit maintenant la consigne du simulateur'); }, { cls: 'btn--sm', pressed: false, title: 'Diffuse en continu la consigne articulaire du simulateur vers le robot' });
    this.mirrorBtn = btn('Miroir', () => { L.mirror = !L.mirror; this.update(); if (!L.mirror) this.app.showMeasured(null); }, { cls: 'btn--sm', pressed: false, title: 'Affiche la position mesurée du robot réel (fantôme)' });
    const note = L.supported
      ? 'Branchez la Teensy 4.1 en USB, puis « Connecter ». Faites la prise d’origine avant tout mouvement.'
      : 'Web Serial n’est pas disponible ici. Pour piloter le robot réel, ouvrez ORION Studio en local (fichier ORION-Studio.html ou npm start) dans Chrome ou Edge sur ordinateur.';
    this.c.append(h('div.sect',
      h('div.sect__head', h('h3.sect__title', 'Robot réel'), this.chip),
      h('p.sect__note', note),
      h('div.row', this.connectBtn, this.streamBtn, this.mirrorBtn),
      h('div.row',
        btn('Activer', () => L.cmd('EN', [1]), { cls: 'btn--sm' }),
        btn('Désactiver', () => L.cmd('EN', [0]), { cls: 'btn--sm' }),
        btn('Prise d’origine', () => L.cmd('HOME'), { cls: 'btn--sm' }),
        btn('Stop', () => L.cmd('STOP'), { cls: 'btn--sm btn--danger' }),
        btn('Acquitter', () => L.cmd('CLR'), { cls: 'btn--sm' }),
        btn('Envoyer la config', () => L.sendConfig(), { cls: 'btn--sm', title: 'Envoie les paramètres (pas/°, butées, vitesses, prise d’origine) puis SAVE' }))));
    const raw = h('input.field.grow#serial-raw', { type: 'text', placeholder: 'Commande brute (ex. STAT, GET vmax 2)…', 'aria-label': 'Commande série' });
    raw.addEventListener('keydown', (e) => { if (e.key === 'Enter' && raw.value.trim()) { const t = raw.value.trim().split(/\s+/); L.cmd(t[0].toUpperCase(), t.slice(1)); raw.value = ''; } });
    this.c.append(h('div.row.row--nowrap', raw));
    // Aperçu des trames CAN mode MIT
    this.mitBox = h('pre', { style: { margin: 0, font: '11.5px/1.5 var(--font-mono)', overflowX: 'auto' } });
    this.c.append(h('div.sect',
      h('div.sect__head', h('h3.sect__title', 'Trames CAN mode MIT')),
      h('p.sect__note', `Consignes MIT (p*, v*, Kp, Kd, τff) de la consigne courante, encodées sur 8 octets pour chaque moteur (ID CAN). Activation : ${hex(MIT_FRAMES.enable)} · zéro : ${hex(MIT_FRAMES.setZero)}.`),
      this.mitBox));
    this.update();
  }

  update() {
    const L = this.link;
    this.chip.textContent = L.connected ? `Connecté${L.version ? ` · ${L.version}` : ''}` : 'Déconnecté';
    this.chip.className = `chip ${L.connected ? 'chip--good' : ''}`;
    this.connectBtn.querySelector('span:last-child').textContent = L.connected ? 'Déconnecter' : 'Connecter (USB)';
    this.connectBtn.disabled = !L.supported;
    this.streamBtn.setAttribute('aria-pressed', String(L.streaming));
    this.mirrorBtn.setAttribute('aria-pressed', String(L.mirror));
  }

  refresh() {
    if (!this.c.offsetParent) return;
    const app = this.app, sim = app.sim;
    const lines = [];
    for (let i = 0; i < sim.n; i++) {
      const a = app.params.actuators.joints[i];
      const c = app.params.control.joints[i];
      const ff = sim.ctrlOut?.ff?.[i] ?? 0;
      const f = packMitCommand({ p: sim.des.q[i], v: sim.des.qd[i], kp: c.mitKp, kd: c.mitKd, t: ff }, a);
      lines.push(`ID 0x${a.canId.toString(16).padStart(3, '0')}  ${hex(f)}   p*=${(sim.des.q[i] * R2D).toFixed(2)}°  τff=${ff.toFixed(2)} N·m`);
    }
    this.mitBox.textContent = lines.join('\n');
  }
}
