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

// Manual videos with an audio track should start audibly after a visitor action.
// Some browsers restore a previously muted media state even when the HTML does
// not include `muted`, so provide an explicit, accessible sound control as well.
const audioVideos = [...document.querySelectorAll('video[data-audio-video]')];
audioVideos.forEach(video => {
  const button = document.querySelector(`[data-audio-control="${video.id}"]`);
  if (!button) return;

  video.defaultMuted = false;
  let restoredOnFirstPlay = false;

  const isAudible = () => !video.muted && video.volume > 0;
  const updateButton = () => {
    const audible = isAudible();
    button.setAttribute('aria-pressed', String(audible));
    button.innerHTML = audible && !video.paused
      ? '<span aria-hidden="true">🔇</span> Mute'
      : '<span aria-hidden="true">🔊</span> Play with sound';
  };

  const restoreSound = () => {
    video.muted = false;
    if (video.volume === 0) video.volume = 1;
  };

  video.addEventListener('play', () => {
    if (!restoredOnFirstPlay) {
      restoredOnFirstPlay = true;
      restoreSound();
    }
    updateButton();
  });
  video.addEventListener('pause', updateButton);
  video.addEventListener('volumechange', updateButton);

  button.addEventListener('click', () => {
    if (!video.paused && isAudible()) {
      video.muted = true;
      updateButton();
      return;
    }

    restoredOnFirstPlay = true;
    restoreSound();
    video.play().catch(() => {});
    updateButton();
  });

  updateButton();
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
const expandedAuthorLists = new WeakSet();

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
    if (expandedAuthorLists.has(list)) {
      list.classList.add('research-entry__authors--expanded');
      return;
    }
    if (fitsOnOneLine(list)) return;

    const authors = list.dataset.fullAuthors.split(/,\s+/);
    for (let visibleAuthors = authors.length - 1; visibleAuthors >= 1; visibleAuthors -= 1) {
      list.innerHTML = `${authors.slice(0, visibleAuthors).join(', ')}, <button type="button" class="research-entry__et-al" aria-label="Show the full author list">et al.</button>`;
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

document.addEventListener('click', event => {
  const expandButton = event.target.closest('.research-entry__et-al');
  if (!expandButton) return;
  const list = expandButton.closest('.research-entry__authors p');
  expandedAuthorLists.add(list);
  list.innerHTML = list.dataset.fullAuthors;
  list.classList.remove('research-entry__authors--folded');
  list.classList.add('research-entry__authors--expanded');
  list.removeAttribute('aria-label');
  list.removeAttribute('title');
});
