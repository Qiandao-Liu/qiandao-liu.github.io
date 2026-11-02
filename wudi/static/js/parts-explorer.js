// Interactive component figure (Mechanical Interface section).
// Hovering a part of either hand greys out the rest of the figure, lights up that part, and opens
// its mechanism detail below. Clicking remains available for touch and keyboard interaction.
// Outlines come from parts-explorer-data.js; the detail cards live in index.html.
(function () {
  const SVGNS = 'http://www.w3.org/2000/svg';
  const ORDER = ['exo', 'linkage', 'passive'];
  const LABEL_PAD = 14;   // px around a label's text box (figure pixels)
  const HOVER_GRACE = 140; // ms before a hover ends, so skimming a small gap doesn't flicker

  function el(tag, attrs, parent) {
    const node = document.createElementNS(SVGNS, tag);
    Object.entries(attrs || {}).forEach(([k, v]) => node.setAttribute(k, v));
    if (parent) parent.appendChild(node);
    return node;
  }

  function labelBox(b, pad) {
    return { x: b[0] - pad, y: b[1] - pad, width: b[2] - b[0] + 2 * pad, height: b[3] - b[1] + 2 * pad };
  }

  function init(root) {
    const DATA = window.WUDI_PARTS;
    const stage = root.querySelector('.px-figure');
    const fallback = stage && stage.querySelector('img');
    if (!DATA || !stage || !fallback) return;

    const src = fallback.getAttribute('src');
    const dimSrc = root.dataset.dimSrc;
    const litSrc = root.dataset.litSrc || src;  // same figure with the arrows painted out
    const css = getComputedStyle(root);
    const color = (k) => css.getPropertyValue(`--px-${k}`).trim() || '#002fa7';
    const touchish = window.matchMedia && matchMedia('(hover: none), (pointer: coarse)').matches;
    const { w, h } = DATA;

    // ---------- build the SVG ----------
    const svg = el('svg', {
      viewBox: `0 0 ${w} ${h}`, class: 'px-svg', role: 'img', 'aria-label': fallback.getAttribute('alt') || '',
    });
    const defs = el('defs', {}, svg);
    el('image', { href: src, width: w, height: h }, svg);
    el('image', { href: dimSrc, width: w, height: h, class: 'px-dim' }, svg);

    ORDER.forEach((k) => {
      const clip = el('clipPath', { id: `px-clip-${k}` }, defs);
      el('path', { d: DATA.hl[k] }, clip);
      const labelClip = el('clipPath', { id: `px-clip-${k}-label` }, defs);
      el('rect', labelBox(DATA.labels[k], LABEL_PAD), labelClip);
      const glow = el('filter', { id: `px-glow-${k}`, x: '-10%', y: '-10%', width: '120%', height: '120%' }, defs);
      el('feDropShadow', { dx: 0, dy: 0, stdDeviation: 7, 'flood-color': color(k), 'flood-opacity': 0.6 }, glow);

      // the lit-up copy: this part's pixels (with a soft halo), its label, and its arrows redrawn as vectors,
      // so arrows belonging to other parts don't show through
      const layer = el('g', { class: 'px-layer', 'data-part': k }, svg);
      const lit = el('g', { filter: `url(#px-glow-${k})` }, layer);
      el('image', { href: litSrc, width: w, height: h, 'clip-path': `url(#px-clip-${k})` }, lit);
      el('image', { href: litSrc, width: w, height: h, 'clip-path': `url(#px-clip-${k}-label)` }, layer);
      DATA.arrows[k].forEach((a) => {
        const len = Math.hypot(a.tip[0] - a.tail[0], a.tip[1] - a.tail[1]);
        el('line', {
          x1: a.tail[0], y1: a.tail[1], x2: a.tip[0], y2: a.tip[1], class: 'px-arrow',
          'stroke-width': DATA.stroke * 1.35, style: `--len:${len.toFixed(1)}`,
        }, layer);
        el('polygon', { points: a.head.map((p) => p.join(',')).join(' '), class: 'px-arrowhead' }, layer);
        el('circle', { cx: a.tail[0], cy: a.tail[1], r: 9, class: 'px-anchor', style: `--c:${color(k)}` }, layer);
      });
    });

    // hit areas: arrows first so the parts they cross still win, then parts, then labels
    const hits = el('g', { class: 'px-hits', 'aria-hidden': 'true' }, svg);
    ORDER.forEach((k) => DATA.arrows[k].forEach((a) => el('line', {
      x1: a.tail[0], y1: a.tail[1], x2: a.tip[0], y2: a.tip[1], class: 'px-hit px-hit-line', 'data-part': k,
    }, hits)));
    ORDER.forEach((k) => el('path', { d: DATA.hit[k], class: 'px-hit', 'data-part': k }, hits));
    ORDER.forEach((k) => el('rect', { ...labelBox(DATA.labels[k], LABEL_PAD + 6), class: 'px-hit', 'data-part': k }, hits));

    stage.appendChild(svg);
    stage.classList.add('is-ready');

    // Interaction nudge, anchored on the left hand's exoskeleton; gone after the first interaction
    const hintAt = DATA.arrows.exo[0].tail;
    const hint = document.createElement('div');
    hint.className = 'px-hint';
    hint.setAttribute('aria-hidden', 'true');
    hint.style.left = `${(hintAt[0] / w) * 100}%`;
    hint.style.top = `${(hintAt[1] / h) * 100}%`;
    hint.innerHTML = `<span class="px-hint-icon"><svg viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round"><path d="M4 3l6.5 16 2.3-6.9L19.5 9.5z"/></svg></span>${touchish ? 'Tap' : 'Hover'} a part`;
    stage.appendChild(hint);
    let hinted = true;
    function dismissHint() {
      if (!hinted) return;
      hinted = false;
      hint.classList.add('is-gone');
      setTimeout(() => hint.remove(), 400);
    }

    // ---------- tooltip (mouse only) ----------
    const tip = document.createElement('div');
    tip.className = 'px-tip';
    tip.setAttribute('aria-hidden', 'true');
    document.body.appendChild(tip);
    function showTip(k) {
      const card = root.querySelector(`.px-card[data-part="${k}"]`);
      tip.style.setProperty('--c', color(k));
      tip.innerHTML = `<span class="px-tip-title">${card.dataset.name}</span>` +
        `<span class="px-tip-body">${card.dataset.blurb}</span>`;
      tip.classList.add('is-on');
    }
    function moveTip(e) {
      const pad = 12, tw = tip.offsetWidth, th = tip.offsetHeight;
      let x = e.clientX + 18, y = e.clientY + 18;
      if (x + tw > innerWidth - pad) x = e.clientX - tw - 18;
      if (y + th > innerHeight - pad) y = e.clientY - th - 18;
      tip.style.transform = `translate(${Math.max(pad, x)}px, ${Math.max(pad, y)}px)`;
    }
    function hideTip() { tip.classList.remove('is-on'); }

    // ---------- state ----------
    const buttons = Array.from(root.querySelectorAll('.px-tab'));
    const cards = Array.from(root.querySelectorAll('.px-card'));
    const detail = root.querySelector('.px-detail');
    let selected = null, hovered = null, hoverTimer = 0;

    function paint() {
      const k = hovered || selected;
      if (k) svg.setAttribute('data-active', k); else svg.removeAttribute('data-active');
    }
    function setHover(k) {
      clearTimeout(hoverTimer);
      if (k === hovered) return;
      hovered = k;
      if (k) dismissHint();
      if (k && selected !== k) select(k, false);
      paint();
    }
    function endHover() {
      clearTimeout(hoverTimer);
      hoverTimer = setTimeout(() => { hovered = null; paint(); hideTip(); }, HOVER_GRACE);
    }
    function select(k, reveal) {
      selected = k;
      buttons.forEach((b) => b.setAttribute('aria-expanded', String(b.dataset.part === k)));
      cards.forEach((c) => { c.hidden = c.dataset.part !== k; });
      root.classList.toggle('is-open', !!k);
      if (k) dismissHint();
      paint();
      if (hovered && tip.classList.contains('is-on')) showTip(hovered);
      if (k && reveal) setTimeout(revealDetail, 380);
    }
    // after opening from the figure, make sure the card is actually on screen
    function revealDetail() {
      const r = detail.getBoundingClientRect();
      if (r.bottom <= innerHeight - 8) return;
      const by = Math.min(r.bottom - innerHeight + 24, r.top - 72);
      if (by > 0) window.scrollBy({ top: by, behavior: 'smooth' });
    }

    // ---------- events ----------
    const partAt = (target) => {
      const hit = target && target.closest ? target.closest('.px-hit') : null;
      return hit ? hit.dataset.part : null;
    };
    svg.addEventListener('pointermove', (e) => {
      if (e.pointerType === 'touch') return;
      const k = partAt(e.target);
      if (k) {
        if (k !== hovered) showTip(k);
        setHover(k);
        moveTip(e);
      } else if (hovered) {
        endHover();
      }
    });
    svg.addEventListener('pointerleave', () => { clearTimeout(hoverTimer); hovered = null; paint(); hideTip(); });
    svg.addEventListener('click', (e) => {
      const k = partAt(e.target);
      if (k) select(touchish && k === selected ? null : k, true);
    });

    buttons.forEach((b) => {
      b.addEventListener('click', () => select(touchish && b.dataset.part === selected ? null : b.dataset.part, false));
      b.addEventListener('pointerenter', (e) => { if (e.pointerType !== 'touch') setHover(b.dataset.part); });
      b.addEventListener('pointerleave', () => { clearTimeout(hoverTimer); hovered = null; paint(); });
    });

    document.addEventListener('keydown', (e) => {
      if (e.key === 'Escape' && selected) select(null, false);
    });
    // scrolling moves the figure out from under a resting cursor without a pointermove
    window.addEventListener('scroll', () => {
      if (!hovered) return;
      clearTimeout(hoverTimer); hovered = null; paint(); hideTip();
    }, { passive: true });
  }

  document.addEventListener('DOMContentLoaded', () => {
    document.querySelectorAll('.parts-explorer').forEach(init);
  });
})();
