document.addEventListener('DOMContentLoaded', () => {
  const layout = document.querySelector('.method-layout');
  const figure = layout?.querySelector('.method-figure');
  const copy = layout?.querySelector('.method-copy');
  const abstract = layout?.querySelector('[data-abstract-preview]');
  const toggle = abstract?.querySelector('.abstract-toggle');

  if (!layout || !figure || !copy || !abstract || !toggle) return;

  const desktop = window.matchMedia('(min-width: 901px)');

  const syncPreviewHeight = () => {
    abstract.style.removeProperty('--abstract-preview-height');
    if (!desktop.matches || abstract.dataset.expanded === 'true') return;

    const figureHeight = figure.getBoundingClientRect().height;
    const abstractOffset = abstract.getBoundingClientRect().top - copy.getBoundingClientRect().top;
    const available = figureHeight - abstractOffset - toggle.getBoundingClientRect().height - 2;
    abstract.style.setProperty('--abstract-preview-height', `${Math.max(80, available)}px`);
  };

  toggle.addEventListener('click', () => {
    const expanded = abstract.dataset.expanded === 'true';
    abstract.dataset.expanded = String(!expanded);
    toggle.setAttribute('aria-expanded', String(!expanded));
    toggle.textContent = expanded ? 'Extend' : 'Collapse';
    if (expanded) requestAnimationFrame(syncPreviewHeight);
  });

  window.addEventListener('resize', syncPreviewHeight);
  desktop.addEventListener('change', syncPreviewHeight);
  new ResizeObserver(syncPreviewHeight).observe(figure);
  requestAnimationFrame(syncPreviewHeight);
});
