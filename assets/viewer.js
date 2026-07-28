import * as THREE from './vendor/three.module.min.js';
import { STLLoader } from './vendor/STLLoader.js';
import { OrbitControls } from './vendor/OrbitControls.js';

const MODEL_URLS = {
  pressure: 'assets/models/rk-200-d.stl',
  efficiency: 'assets/models/rk-200-e.stl'
};

function isWebGLAvailable() {
  try {
    const test = document.createElement('canvas');
    return !!(test.getContext('webgl2') || test.getContext('webgl'));
  } catch {
    return false;
  }
}

class EngineeringViewer {
  constructor(root) {
    this.root = root;
    this.canvas = root.querySelector('canvas');
    this.fallback = root.querySelector('.viewer-fallback');
    this.loading = root.querySelector('.viewer-loading');
    this.modelKey = root.dataset.model || 'pressure';
    this.autoRotate = root.dataset.rotate !== 'false';
    this.isVisible = false;
    this.animFrameId = null;
    this.loadToken = 0;
    this.destroyed = false;

    if (!this.canvas || !isWebGLAvailable()) {
      this.loading?.classList.add('is-hidden');
      this.fallback?.classList.remove('is-hidden');
      return;
    }

    try {
      this.scene = new THREE.Scene();
      this.camera = new THREE.PerspectiveCamera(32, 1, .1, 2000);
      this.renderer = new THREE.WebGLRenderer({
        canvas: this.canvas,
        alpha: true,
        antialias: true,
        powerPreference: 'high-performance'
      });
      this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
      this.renderer.outputColorSpace = THREE.SRGBColorSpace;
      this.renderer.shadowMap.enabled = true;
      this.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

      this.controls = new OrbitControls(this.camera, this.canvas);
      this.controls.enableDamping = true;
      this.controls.dampingFactor = .07;
      this.controls.enablePan = false;
      this.controls.minDistance = 180;
      this.controls.maxDistance = 700;
      this.controls.autoRotate = this.autoRotate;
      this.controls.autoRotateSpeed = .32;

      this.group = new THREE.Group();
      this.scene.add(this.group);
      this.setupScene();
      this.bindControls();

      this.resizeObserver = new ResizeObserver(() => this.resize());
      this.resizeObserver.observe(root);
      this.visibilityObserver = new IntersectionObserver((entries) => {
        for (const entry of entries) {
          this.isVisible = entry.isIntersecting;
        }
        if (this.isVisible) this.animate();
      }, { threshold: 0 });
      this.visibilityObserver.observe(root);

      this.resize();
      this.load(this.modelKey);
    } catch (error) {
      console.error('Viewer init failed', error);
      this.loading?.classList.add('is-hidden');
      this.fallback?.classList.remove('is-hidden');
    }
  }

  setupScene() {
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0xc9c7c3, 2.6));
    const key = new THREE.DirectionalLight(0xffffff, 3.2);
    key.position.set(140, 220, 180);
    key.castShadow = true;
    this.scene.add(key);
    const rim = new THREE.DirectionalLight(0xff6b00, 2.1);
    rim.position.set(-180, 70, -130);
    this.scene.add(rim);
    const grid = new THREE.GridHelper(360, 18, 0xd9d8d4, 0xe9e8e5);
    grid.position.y = -42;
    grid.material.opacity = .55;
    grid.material.transparent = true;
    this.scene.add(grid);
  }

  load(modelKey) {
    if (this.destroyed || !this.renderer) return;
    const key = MODEL_URLS[modelKey] ? modelKey : 'pressure';
    this.modelKey = key;
    this.loadToken += 1;
    const token = this.loadToken;
    this.loading?.classList.remove('is-hidden');

    const loader = new STLLoader();
    const applyGeometry = (geometry) => {
      if (token !== this.loadToken || this.destroyed) {
        geometry.dispose();
        return;
      }
      this.disposeMeshes();
      geometry.computeVertexNormals();
      geometry.center();
      geometry.rotateX(-Math.PI / 2);
      const material = new THREE.MeshPhysicalMaterial({
        color: 0x2a2b44,
        roughness: .58,
        metalness: .18,
        clearcoat: .22,
        clearcoatRoughness: .55,
        side: THREE.DoubleSide
      });
      const mesh = new THREE.Mesh(geometry, material);
      mesh.castShadow = true;
      mesh.receiveShadow = true;
      this.group.add(mesh);
      const edges = new THREE.LineSegments(
        new THREE.EdgesGeometry(geometry, 42),
        new THREE.LineBasicMaterial({ color: 0x151624, transparent: true, opacity: .22 })
      );
      this.group.add(edges);
      this.group.rotation.y = -.2;
      this.setView('top');
      this.fallback?.classList.add('is-hidden');
      this.loading?.classList.add('is-hidden');
    };

    const embedded = window.MITK_MODELS?.[key];
    if (embedded) {
      try {
        const binary = atob(embedded);
        const bytes = Uint8Array.from(binary, (c) => c.charCodeAt(0));
        applyGeometry(loader.parse(bytes.buffer));
      } catch (error) {
        console.error('Embedded model could not be parsed', error);
        if (token === this.loadToken) this.loading?.classList.add('is-hidden');
      }
      return;
    }
    loader.load(MODEL_URLS[key], applyGeometry, undefined, () => {
      if (token === this.loadToken) this.loading?.classList.add('is-hidden');
    });
  }

  disposeMeshes() {
    while (this.group.children.length) {
      const child = this.group.children.pop();
      child.geometry?.dispose();
      child.material?.dispose();
      if (child.material?.length) child.material.forEach((m) => m.dispose());
    }
  }

  setView(view) {
    const positions = {
      iso: [245, 185, 265],
      top: [0, 390, .1],
      front: [0, 45, 390]
    };
    const [x, y, z] = positions[view] || positions.iso;
    this.camera.up.set(0, view === 'top' ? 0 : 1, view === 'top' ? -1 : 0);
    this.camera.position.set(x, y, z);
    this.controls.target.set(0, 0, 0);
    this.controls.update();
  }

  bindControls() {
    this.root.querySelectorAll('[data-view]').forEach((button) => {
      button.addEventListener('click', () => {
        this.root.querySelectorAll('[data-view]').forEach((item) => item.classList.toggle('is-active', item === button));
        this.setView(button.dataset.view);
      });
    });
    this.root.querySelector('[data-rotate]')?.addEventListener('click', (event) => {
      this.controls.autoRotate = !this.controls.autoRotate;
      event.currentTarget.classList.toggle('is-active', this.controls.autoRotate);
    });
  }

  resize() {
    if (!this.renderer) return;
    const width = this.root.clientWidth;
    const height = this.root.clientHeight;
    const w = Math.max(width, 1);
    const h = Math.max(height, 1);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h, false);
  }

  animate() {
    if (!this.isVisible || this.destroyed || !this.renderer) return;
    if (this.animFrameId !== null) return;
    const loop = () => {
      if (!this.isVisible || this.destroyed) {
        this.animFrameId = null;
        return;
      }
      this.controls.update();
      this.renderer.render(this.scene, this.camera);
      this.animFrameId = requestAnimationFrame(loop);
    };
    this.animFrameId = requestAnimationFrame(loop);
  }

  destroy() {
    this.destroyed = true;
    if (this.animFrameId !== null) {
      cancelAnimationFrame(this.animFrameId);
      this.animFrameId = null;
    }
    this.resizeObserver?.disconnect();
    this.visibilityObserver?.disconnect();
    this.disposeMeshes();
    this.renderer?.dispose();
    this.controls?.dispose();
  }
}

const viewers = [...document.querySelectorAll('[data-viewer]')];
const registry = viewers.map((root) => new EngineeringViewer(root));
window.addEventListener('mitk:modelchange', (event) => registry.forEach((viewer) => viewer.load(event.detail?.model)));
window.addEventListener('beforeunload', () => registry.forEach((viewer) => viewer.destroy()));
