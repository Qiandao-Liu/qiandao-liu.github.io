// Interactive 3D viewer for the WUDI + LEAP Hand model at the end of the page.
// three.js (resolved through the import map in index.html) and the model are only fetched once the
// viewer is about to scroll into view, and the render loop sleeps whenever nothing is moving.
const touchish = window.matchMedia && matchMedia('(hover: none), (pointer: coarse)').matches;

function init(root) {
  const stage = root.querySelector('.mv-stage');
  const status = root.querySelector('.mv-status');
  const bar = root.querySelector('.mv-bar i');
  const note = root.querySelector('.mv-note');
  const hint = root.querySelector('.mv-hint');
  const btnReset = root.querySelector('[data-mv="reset"]');
  const btnSpin = root.querySelector('[data-mv="spin"]');
  if (hint) hint.textContent = touchish
    ? 'Drag to rotate · Pinch to zoom · Two fingers to pan'
    : 'Drag to rotate · Right-drag to pan · Click, then scroll to zoom';

  const setStatus = (text, pct) => {
    status.firstChild.textContent = text;
    if (bar && pct != null) bar.style.width = `${pct}%`;
  };

  let started = false;
  let loadCurrentModel = null;
  root.addEventListener('modelchange', ({ detail }) => {
    if (!detail?.src) return;
    root.dataset.src = detail.src;
    if (loadCurrentModel) loadCurrentModel(detail.src);
  });
  const near = new IntersectionObserver((entries) => {
    if (started || !entries.some((e) => e.isIntersecting)) return;
    started = true;
    near.disconnect();
    start().catch((err) => {
      console.error(err);
      setStatus('Could not load the 3D model.', null);
      root.classList.add('is-error');
    });
  }, { rootMargin: '600px 0px' });
  near.observe(root);

  async function start() {
    const [THREE, { GLTFLoader }, { OrbitControls }, { MeshoptDecoder }, { RoomEnvironment }] = await Promise.all([
      import('three'),
      import('three/addons/loaders/GLTFLoader.js'),
      import('three/addons/controls/OrbitControls.js'),
      import('three/addons/libs/meshopt_decoder.module.js'),
      import('three/addons/environments/RoomEnvironment.js'),
    ]);

    let renderer;
    try {
      renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    } catch (e) {
      setStatus('3D view needs WebGL, which this browser has turned off.', null);
      root.classList.add('is-error');
      return;
    }
    renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.NeutralToneMapping;
    stage.appendChild(renderer.domElement);

    const scene = new THREE.Scene();
    const pmrem = new THREE.PMREMGenerator(renderer);
    scene.environment = pmrem.fromScene(new RoomEnvironment(), 0.04).texture;
    scene.environmentIntensity = 0.8;
    const key = new THREE.DirectionalLight(0xffffff, 0.75);
    key.position.set(0.6, 1, 0.8);
    scene.add(key);

    const camera = new THREE.PerspectiveCamera(32, 1, 0.005, 20);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.autoRotate = !window.matchMedia('(prefers-reduced-motion: reduce)').matches;
    controls.autoRotateSpeed = 1.1;
    controls.enableZoom = touchish;   // desktop: wheel zoom only once the viewer is engaged (see below)
    syncSpin();

    // ---------- render loop that sleeps when idle ----------
    let running = false, visible = true, dragging = false;
    function frame() {
      const moved = controls.update();
      renderer.render(scene, camera);
      if (!controls.autoRotate && !moved && !dragging) stop();
    }
    function wake() {
      if (running || !visible) return;
      running = true;
      renderer.setAnimationLoop(frame);
    }
    function stop() {
      running = false;
      renderer.setAnimationLoop(null);
    }
    controls.addEventListener('start', () => { dragging = true; setSpin(false); wake(); });
    controls.addEventListener('end', () => { dragging = false; wake(); });
    controls.addEventListener('change', wake);
    new IntersectionObserver(([e]) => { visible = e.isIntersecting; if (visible) wake(); else stop(); }).observe(stage);

    function resize() {
      const w = stage.clientWidth, h = stage.clientHeight;
      if (!w || !h) return;
      renderer.setSize(w, h, false);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      renderer.render(scene, camera);
    }
    new ResizeObserver(resize).observe(stage);

    // ---------- wheel zoom without hijacking page scroll ----------
    let engaged = false, noteTimer = 0;
    stage.addEventListener('pointerdown', () => { engaged = true; controls.enableZoom = true; });
    stage.addEventListener('pointerleave', () => { engaged = false; if (!touchish) controls.enableZoom = false; });
    stage.addEventListener('wheel', (e) => {
      if (touchish) return;
      controls.enableZoom = engaged || e.ctrlKey;   // ctrl+wheel is a trackpad pinch
      if (!controls.enableZoom && note) {
        note.classList.add('is-on');
        clearTimeout(noteTimer);
        noteTimer = setTimeout(() => note.classList.remove('is-on'), 1400);
      }
    }, { capture: true, passive: true });

    // ---------- buttons ----------
    function setSpin(on) { controls.autoRotate = on; syncSpin(); if (on) wake(); }
    function syncSpin() {
      if (!btnSpin) return;
      btnSpin.setAttribute('aria-pressed', String(controls.autoRotate));
      btnSpin.title = controls.autoRotate ? 'Pause rotation' : 'Auto-rotate';
      btnSpin.setAttribute('aria-label', btnSpin.title);
    }
    btnSpin && btnSpin.addEventListener('click', () => setSpin(!controls.autoRotate));

    // ---------- model ----------
    const loader = new GLTFLoader().setMeshoptDecoder(MeshoptDecoder);
    let model = null;
    let radius = 1;
    let loadToken = 0;
    const homeDir = new THREE.Vector3(0.62, 0.34, 0.71).normalize();
    // fit the bounding sphere to the narrower of the two fields of view (portrait phones are width-bound)
    function homeDist() {
      const v = THREE.MathUtils.degToRad(camera.fov / 2);
      const h = Math.atan(Math.tan(v) * camera.aspect);
      return (radius / Math.sin(Math.min(v, h))) * 0.8;
    }
    controls.minDistance = radius * 0.6;
    controls.maxDistance = homeDist() * 3;
    function reset() {
      camera.position.copy(homeDir).multiplyScalar(homeDist());
      controls.target.set(0, 0, 0);
      controls.update();
      wake();
    }
    btnReset && btnReset.addEventListener('click', reset);
    // until someone moves the camera, keep the model framed as the viewer resizes
    let moved = false;
    controls.addEventListener('start', () => { moved = true; });
    new ResizeObserver(() => { if (!moved) reset(); }).observe(stage);

    const disposeModel = (object) => {
      object.traverse((child) => {
        child.geometry?.dispose();
        const materials = Array.isArray(child.material) ? child.material : [child.material];
        materials.filter(Boolean).forEach((material) => material.dispose());
      });
      scene.remove(object);
    };

    loadCurrentModel = async (src) => {
      const token = ++loadToken;
      root.classList.remove('is-ready', 'is-error');
      setStatus('Loading 3D model…', 0);
      try {
        const gltf = await new Promise((resolve, reject) => loader.load(src, resolve, (e) => {
          if (token === loadToken && e.lengthComputable) {
            setStatus('Loading 3D model…', Math.round((e.loaded / e.total) * 100));
          }
        }, reject));
        if (token !== loadToken) {
          disposeModel(gltf.scene);
          return;
        }
        if (model) disposeModel(model);
        model = gltf.scene;
        model.rotation.x = -Math.PI / 2;   // the CAD exports are Z-up; three.js is Y-up
        model.updateMatrixWorld(true);
        const box = new THREE.Box3().setFromObject(model);
        model.position.sub(box.getCenter(new THREE.Vector3()));
        scene.add(model);
        radius = box.getBoundingSphere(new THREE.Sphere()).radius;
        controls.minDistance = radius * 0.6;
        controls.maxDistance = homeDist() * 3;
        moved = false;
        resize();
        reset();
        root.classList.add('is-ready');
        wake();
      } catch (error) {
        if (token !== loadToken) return;
        console.error(error);
        setStatus('Could not load the 3D model.', null);
        root.classList.add('is-error');
      }
    };

    resize();
    await loadCurrentModel(root.dataset.src);
  }
}

document.querySelectorAll('.model-viewer').forEach(init);
