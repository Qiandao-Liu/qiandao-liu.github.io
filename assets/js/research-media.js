// GIF-like playback while visible, without downloading every video on page load.
const videos = [...document.querySelectorAll('video:not([data-manual-video])')];
const visible = new Set();
const pausedByVisitor = new Set();
const play = video => {
  if (!document.hidden && !pausedByVisitor.has(video)) {
    video.autoplay = true;
    video.play().catch(() => { video.controls = true; });
  }
};
const observer = new IntersectionObserver(entries => {
  entries.forEach(({target: video, isIntersecting}) => {
    if (isIntersecting) {
      visible.add(video);
      play(video);
    } else {
      visible.delete(video);
      video.autoplay = false;
      video.pause();
    }
  });
}, {threshold: 0.05});
videos.forEach(video => {
  const playbackRate = Number(video.dataset.playbackRate || 1);
  video.defaultPlaybackRate = playbackRate;
  video.playbackRate = playbackRate;
  video.muted = true;
  video.defaultMuted = true;
  video.loop = true;
  video.playsInline = true;
  if (!video.controls) {
    video.tabIndex = 0;
    video.setAttribute('role', 'button');
    const label = video.getAttribute('aria-label') || 'Video preview';
    video.setAttribute('aria-label', label + '. Click or press Space to pause or resume.');
    const toggle = () => {
      if (video.paused) { pausedByVisitor.delete(video); play(video); }
      else { pausedByVisitor.add(video); video.pause(); }
    };
    video.addEventListener('click', toggle);
    video.addEventListener('keydown', event => {
      if (event.key === ' ' || event.key === 'Enter') { event.preventDefault(); toggle(); }
    });
  }
  observer.observe(video);
});
document.addEventListener('visibilitychange', () => {
  videos.forEach(video => {
    if (document.hidden) video.pause();
    else if (visible.has(video)) play(video);
  });
});

// Keep homepage publication author lists to one line, using the longest prefix
// that still leaves room for "et al." when the full list would wrap.
const publicationAuthorLists = [...document.querySelectorAll(
  '.home-page .research-list--publications .research-entry__authors p'
)];
publicationAuthorLists.forEach(list => {
  list.dataset.fullAuthors = list.innerHTML;
  list.dataset.fullAuthorsText = list.textContent.trim();
});

const fitsOnOneLine = element => {
  const lineHeight = Number.parseFloat(getComputedStyle(element).lineHeight);
  return element.getBoundingClientRect().height <= lineHeight * 1.25;
};

const foldPublicationAuthors = () => {
  publicationAuthorLists.forEach(list => {
    list.innerHTML = list.dataset.fullAuthors;
    list.classList.remove('research-entry__authors--folded');
    list.removeAttribute('aria-label');
    list.removeAttribute('title');
    if (fitsOnOneLine(list)) return;

    const authors = list.dataset.fullAuthors.split(/,\s+/);
    for (let visibleAuthors = authors.length - 1; visibleAuthors >= 1; visibleAuthors -= 1) {
      list.innerHTML = `${authors.slice(0, visibleAuthors).join(', ')}, <span class="research-entry__et-al">et al.</span>`;
      if (fitsOnOneLine(list) || visibleAuthors === 1) break;
    }
    list.classList.add('research-entry__authors--folded');
    list.setAttribute('aria-label', list.dataset.fullAuthorsText);
    list.setAttribute('title', list.dataset.fullAuthorsText);
  });
};

let authorFoldFrame;
const scheduleAuthorFold = () => {
  cancelAnimationFrame(authorFoldFrame);
  authorFoldFrame = requestAnimationFrame(foldPublicationAuthors);
};
foldPublicationAuthors();
window.addEventListener('resize', scheduleAuthorFold);
if (document.fonts) document.fonts.ready.then(scheduleAuthorFold);
