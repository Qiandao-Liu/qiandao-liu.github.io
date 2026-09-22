// GIF-like playback while visible, without downloading every video on page load.
const videos = [...document.querySelectorAll('video')];
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
