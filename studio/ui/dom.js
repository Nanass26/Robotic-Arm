// ORION Studio — utilitaires d’interface (création d’éléments, icônes, notifications, fichiers).

/** Crée un élément : h('button.btn.btn--sm', {onclick, title}, 'Texte', enfant…) */
export function h(tag, props = {}, ...children) {
  const m = /^([a-z0-9]+)((?:[.#][\w-]+)*)$/i.exec(tag);
  const el = document.createElement(m ? m[1] : tag);
  if (m && m[2]) {
    for (const part of m[2].match(/[.#][\w-]+/g)) {
      if (part[0] === '.') el.classList.add(part.slice(1));
      else el.id = part.slice(1);
    }
  }
  if (props && (typeof props !== 'object' || props instanceof Node || Array.isArray(props))) {
    children.unshift(props);
    props = {};
  }
  for (const [k, v] of Object.entries(props || {})) {
    if (v === undefined || v === null || v === false) continue;
    if (k.startsWith('on') && typeof v === 'function') el.addEventListener(k.slice(2), v);
    else if (k === 'style' && typeof v === 'object') Object.assign(el.style, v);
    else if (k === 'dataset') Object.assign(el.dataset, v);
    else if (k === 'html') el.innerHTML = v;
    else if (k in el && typeof v !== 'string') el[k] = v;
    else el.setAttribute(k, v === true ? '' : v);
  }
  for (const c of children.flat(Infinity)) {
    if (c === null || c === undefined || c === false) continue;
    el.append(c instanceof Node ? c : document.createTextNode(String(c)));
  }
  return el;
}

// Icônes (tracés SVG 24×24, trait 2 px)
const ICONS = {
  play: '<path d="M7 5v14l11-7z" fill="currentColor" stroke="none"/>',
  pause: '<path d="M8 5v14M16 5v14"/>',
  stop: '<rect x="6" y="6" width="12" height="12" rx="1" fill="currentColor" stroke="none"/>',
  step: '<path d="M6 5v14l9-7z" fill="currentColor" stroke="none"/><path d="M18 5v14"/>',
  home: '<path d="M4 11l8-7 8 7"/><path d="M6 10v10h12V10"/>',
  power: '<path d="M12 3v9"/><path d="M6.3 6.3a8 8 0 1 0 11.4 0"/>',
  plus: '<path d="M12 5v14M5 12h14"/>',
  minus: '<path d="M5 12h14"/>',
  mic: '<rect x="9" y="3" width="6" height="11" rx="3"/><path d="M5 11a7 7 0 0 0 14 0M12 18v3"/>',
  send: '<path d="M4 12l16-8-6 16-3-7z"/>',
  download: '<path d="M12 4v11M7 10l5 5 5-5M5 20h14"/>',
  upload: '<path d="M12 20V9M7 14l5-5 5 5M5 4h14"/>',
  copy: '<rect x="8" y="8" width="12" height="12" rx="2"/><path d="M16 8V5a1 1 0 0 0-1-1H5a1 1 0 0 0-1 1v10a1 1 0 0 0 1 1h3"/>',
  reset: '<path d="M4 12a8 8 0 1 0 3-6.2"/><path d="M4 4v5h5"/>',
  sun: '<circle cx="12" cy="12" r="4"/><path d="M12 2v2M12 20v2M2 12h2M20 12h2M4.9 4.9l1.4 1.4M17.7 17.7l1.4 1.4M4.9 19.1l1.4-1.4M17.7 6.3l1.4-1.4"/>',
  moon: '<path d="M20 14.5A8 8 0 0 1 9.5 4a8 8 0 1 0 10.5 10.5z"/>',
  left: '<rect x="3" y="4" width="18" height="16" rx="2"/><path d="M9 4v16"/>',
  right: '<rect x="3" y="4" width="18" height="16" rx="2"/><path d="M15 4v16"/>',
  bottom: '<rect x="3" y="4" width="18" height="16" rx="2"/><path d="M3 15h18"/>',
  cube: '<path d="M12 3l8 4.5v9L12 21l-8-4.5v-9z"/><path d="M12 12l8-4.5M12 12v9M12 12L4 7.5"/>',
  axes: '<path d="M5 19V5M5 19h14M5 19l10-10"/>',
  path: '<path d="M4 18c3-9 6 3 9-6s4-4 7-6"/>',
  cloud: '<circle cx="7" cy="14" r="1.2" fill="currentColor"/><circle cx="12" cy="9" r="1.2" fill="currentColor"/><circle cx="16" cy="15" r="1.2" fill="currentColor"/><circle cx="18" cy="8" r="1.2" fill="currentColor"/><circle cx="9" cy="18" r="1.2" fill="currentColor"/>',
  target: '<circle cx="12" cy="12" r="7"/><circle cx="12" cy="12" r="2" fill="currentColor"/><path d="M12 2v3M12 19v3M2 12h3M19 12h3"/>',
  rotate: '<path d="M20 12a8 8 0 1 1-2.3-5.7"/><path d="M20 4v5h-5"/>',
  move: '<path d="M12 3v18M3 12h18M12 3l-3 3M12 3l3 3M12 21l-3-3M12 21l3-3M3 12l3-3M3 12l3 3M21 12l-3-3M21 12l-3 3"/>',
  eye: '<path d="M2 12s4-7 10-7 10 7 10 7-4 7-10 7S2 12 2 12z"/><circle cx="12" cy="12" r="3"/>',
  capsule: '<rect x="8" y="3" width="8" height="18" rx="4"/>',
  usb: '<path d="M12 3v14M12 3l-2 3h4zM8 9v3l4 2M16 7v4l-4 2"/><circle cx="12" cy="19" r="2"/>',
  gauge: '<path d="M4 18a8 8 0 1 1 16 0"/><path d="M12 18l4-6"/>',
  shadow: '<circle cx="12" cy="9" r="5"/><ellipse cx="12" cy="19" rx="7" ry="2" fill="currentColor" stroke="none" opacity=".35"/>',
  info: '<circle cx="12" cy="12" r="9"/><path d="M12 11v6M12 7.5v.5"/>',
  grip: '<path d="M8 4v7a4 4 0 0 0 8 0V4M8 4H5M16 4h3M12 15v5"/>',
};
export function icon(name) {
  const span = document.createElement('span');
  span.innerHTML = `<svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">${ICONS[name] || ''}</svg>`;
  return span.firstChild;
}

export function btn(label, onClick, opts = {}) {
  const b = h(`button.btn${opts.cls ? '.' + opts.cls.split(' ').join('.') : ''}`, { type: 'button', title: opts.title || '', onclick: onClick });
  if (opts.id) b.id = opts.id;
  if (opts.icon) b.append(icon(opts.icon));
  if (label) b.append(h('span', label));
  if (opts.pressed !== undefined) b.setAttribute('aria-pressed', String(!!opts.pressed));
  if (!label && opts.title) b.setAttribute('aria-label', opts.title);
  return b;
}

// ------------------------------------------------------------ notifications
let toastHost = null;
export function toast(msg, ms = 2600) {
  if (!toastHost) { toastHost = h('div.toast-host', { role: 'status', 'aria-live': 'polite' }); document.body.append(toastHost); }
  const t = h('div.toast', msg);
  toastHost.append(t);
  setTimeout(() => t.remove(), ms);
}

// ------------------------------------------------------------ infobulles d’aide
let tip = null;
export function attachHelp(el, text) {
  if (!text) return el;
  const show = () => {
    if (!tip) { tip = h('div.tooltip', { role: 'tooltip' }); document.body.append(tip); }
    tip.textContent = text;
    tip.hidden = false;
    const r = el.getBoundingClientRect();
    const w = Math.min(320, window.innerWidth - 20);
    tip.style.maxWidth = `${w}px`;
    let x = r.left, y = r.bottom + 6;
    tip.style.left = '0px'; tip.style.top = '0px';
    const tr = tip.getBoundingClientRect();
    if (x + tr.width > window.innerWidth - 10) x = window.innerWidth - 10 - tr.width;
    if (y + tr.height > window.innerHeight - 10) y = r.top - tr.height - 6;
    tip.style.left = `${Math.max(10, x)}px`;
    tip.style.top = `${Math.max(10, y)}px`;
  };
  const hide = () => { if (tip) tip.hidden = true; };
  el.addEventListener('mouseenter', show);
  el.addEventListener('mouseleave', hide);
  el.addEventListener('focus', show);
  el.addEventListener('blur', hide);
  return el;
}

export function helpDot(text) {
  const b = h('button.help-dot', { type: 'button', 'aria-label': 'Aide' }, '?');
  return attachHelp(b, text);
}

// ------------------------------------------------------------ fenêtre modale
export function modal(title, body, actions = []) {
  const back = h('div.modal-back', { role: 'dialog', 'aria-modal': 'true', 'aria-label': title });
  const close = () => back.remove();
  const m = h('div.modal',
    h('div.modal__head', h('div.modal__title', title), ...actions, btn('Fermer', close, { cls: 'btn--sm' })),
    h('div.modal__body', body));
  back.append(m);
  back.addEventListener('click', (e) => { if (e.target === back) close(); });
  back.addEventListener('keydown', (e) => { if (e.key === 'Escape') close(); });
  document.body.append(back);
  setTimeout(() => m.querySelector('button')?.focus(), 0);
  return { close, el: m };
}

// ------------------------------------------------------------ fichiers
/** Enregistre un fichier (téléchargement) ; si l’environnement le bloque, affiche le contenu à copier. */
export function saveFile(name, content, mime = 'text/plain') {
  const isText = typeof content === 'string';
  try {
    const blob = content instanceof Blob ? content : new Blob([content], { type: mime });
    const url = URL.createObjectURL(blob);
    const a = h('a', { href: url, download: name });
    document.body.append(a);
    a.click();
    a.remove();
    setTimeout(() => URL.revokeObjectURL(url), 4000);
  } catch (e) {
    // ignoré : repli ci-dessous
  }
  if (isText && window.ORION_EMBEDDED) {
    // Dans une vue intégrée (artefact), les téléchargements peuvent être bloqués : on propose la copie.
    showText(name, content);
  }
}

export function showText(title, text) {
  const pre = h('pre', text);
  modal(title, pre, [btn('Copier', () => copyText(text), { cls: 'btn--sm btn--primary', icon: 'copy' })]);
}

export async function copyText(text) {
  try {
    await navigator.clipboard.writeText(text);
    toast('Copié dans le presse-papiers');
  } catch {
    toast('Copie impossible : sélectionnez le texte manuellement');
  }
}

export function pickFile(accept) {
  return new Promise((resolve) => {
    const inp = h('input', { type: 'file', accept, style: { display: 'none' } });
    inp.addEventListener('change', () => {
      const f = inp.files?.[0];
      inp.remove();
      if (!f) return resolve(null);
      const r = new FileReader();
      r.onload = () => resolve({ name: f.name, text: r.result });
      r.readAsText(f);
    });
    document.body.append(inp);
    inp.click();
  });
}

// ------------------------------------------------------------ stockage local (tolérant)
export const store = {
  get(key, def = null) {
    try { const v = localStorage.getItem(`orion.${key}`); return v === null ? def : JSON.parse(v); } catch { return def; }
  },
  set(key, v) {
    try { localStorage.setItem(`orion.${key}`, JSON.stringify(v)); } catch { /* stockage indisponible */ }
  },
  del(key) { try { localStorage.removeItem(`orion.${key}`); } catch { /* */ } },
};

export function debounce(fn, ms) {
  let t = null;
  return (...a) => { clearTimeout(t); t = setTimeout(() => fn(...a), ms); };
}

export const cssVar = (name, el = document.documentElement) => getComputedStyle(el).getPropertyValue(name).trim();

export const fmtNum = (v, d = 2) => (Number.isFinite(v) ? v.toFixed(d) : '—');
