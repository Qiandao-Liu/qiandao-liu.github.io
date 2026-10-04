---
layout: single
permalink: /mulberry/
title: "Mulberry Now Lives with a World Memory Master"
author_profile: false
mathjax: false
mermaid: false
---

<div class="mulberry-page-content">
<figure class="mulberry-video">
  <div class="mulberry-video__frame">
    <video controls playsinline preload="metadata" data-manual-video aria-label="Video of Mulberry">
      <source src="{{ '/images/mulberry/mulberry.mp4' | relative_url }}" type="video/mp4">
      Your browser does not support the video tag.
    </video>
  </div>
  <figcaption>This content may be disturbing. Viewer discretion is advised.</figcaption>
</figure>

<div class="mulberry-stickers">
{% for photo in (1..30) %}
  {% assign tilt_value = forloop.index | times: 17 | modulo: 13 | minus: 6 %}
  {% assign scale_step = forloop.index | times: 7 | modulo: 9 %}
  {% assign scale_value = scale_step | times: 0.01 | plus: 0.92 %}
  {% assign sticker_order = forloop.index0 | times: 17 | modulo: 30 %}
  <figure class="mulberry-sticker" style="--sticker-tilt: {{ tilt_value }}deg; --sticker-scale: {{ scale_value }}; --sticker-order: {{ sticker_order }};">
    <img loading="lazy" decoding="async" src="{{ '/images/mulberry/cat' | append: forloop.index | append: '.webp' | relative_url }}" alt="Mulberry photo {{ forloop.index }}">
  </figure>
{% endfor %}
</div>
</div>
