document.addEventListener('DOMContentLoaded', () => {
  const root = document.querySelector('[data-model-carousel]');
  if (!root) return;

  const models = [
    { name: 'LEAP Hand', model: './static/models/wudi_leap.glb', render: './static/images/extended/leap.webp', verification: 'Real-world verified', real: true },
    { name: 'WUJI Hand v1', model: './static/models/extended/wudi-wuji.glb?v=1', render: './static/images/extended/wuji.webp', verification: 'Real-world verified', real: true },
    { name: 'OYHand', model: './static/models/extended/wudi-oyhand.glb?v=2', render: './static/images/extended/oyhand.webp', verification: 'Simulation verified' },
    { name: 'Sharpa Hand', model: './static/models/extended/wudi-sharpa.glb?v=2', render: './static/images/extended/sharpa.webp', verification: 'Simulation verified' },
    { name: 'XHand', model: './static/models/extended/wudi-xhand.glb?v=2', render: './static/images/extended/xhand.webp', verification: 'Simulation verified' },
    { name: 'Schunk Hand', model: './static/models/extended/wudi-schunk.glb?v=2', render: './static/images/extended/schunk.webp', verification: 'Simulation verified' },
  ];

  const viewer = root.querySelector('.model-viewer');
  const stage = root.querySelector('.mv-stage');
  const name = root.querySelector('[data-model-name]');
  const verification = root.querySelector('[data-model-verification]');
  const render = root.querySelector('[data-model-render]');
  const count = root.querySelector('[data-model-count]');
  const dots = root.querySelector('[data-model-dots]');
  let active = 0;

  models.forEach(() => dots.appendChild(document.createElement('i')));

  const show = (next) => {
    active = (next + models.length) % models.length;
    const item = models[active];
    name.textContent = item.name;
    verification.textContent = item.verification;
    verification.classList.toggle('is-real', Boolean(item.real));
    render.src = item.render;
    render.alt = `Rendered WUDI configuration for ${item.name}`;
    count.textContent = `${active + 1} / ${models.length}`;
    Array.from(dots.children).forEach((dot, index) => dot.classList.toggle('is-active', index === active));
    stage.setAttribute('aria-label', `Interactive 3D model of WUDI configured for ${item.name}`);
    viewer.dispatchEvent(new CustomEvent('modelchange', { detail: { src: item.model } }));
  };

  root.querySelector('[data-model-direction="previous"]').addEventListener('click', () => show(active - 1));
  root.querySelector('[data-model-direction="next"]').addEventListener('click', () => show(active + 1));
  root.addEventListener('keydown', (event) => {
    if (event.key === 'ArrowLeft') show(active - 1);
    if (event.key === 'ArrowRight') show(active + 1);
  });
  show(0);
});
