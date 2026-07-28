(() => {
  const header = document.querySelector('.site-header');
  const menuButton = document.querySelector('.menu-button');
  const navigation = document.querySelector('.main-nav');

  function updateHeader() {
    header?.classList.toggle('is-scrolled', window.scrollY > 8);
  }
  updateHeader();
  window.addEventListener('scroll', updateHeader, { passive: true });

  menuButton?.addEventListener('click', () => {
    const open = menuButton.getAttribute('aria-expanded') === 'true';
    menuButton.setAttribute('aria-expanded', String(!open));
    menuButton.setAttribute('aria-label', open ? 'Открыть меню' : 'Закрыть меню');
    navigation?.classList.toggle('is-open', !open);
  });

  document.addEventListener('click', (event) => {
    if (!navigation?.classList.contains('is-open')) return;
    if (event.target.closest('.main-nav') || event.target.closest('.menu-button')) return;
    navigation.classList.remove('is-open');
    menuButton?.setAttribute('aria-expanded', 'false');
    menuButton?.setAttribute('aria-label', 'Открыть меню');
  });

  navigation?.addEventListener('click', (event) => {
    if (!event.target.closest('a')) return;
    navigation.classList.remove('is-open');
    menuButton?.setAttribute('aria-expanded', 'false');
    menuButton?.setAttribute('aria-label', 'Открыть меню');
  });

  const activePage = document.body.dataset.page;
  document.querySelectorAll('.main-nav a').forEach((link) => {
    if (link.dataset.page === activePage) link.setAttribute('aria-current', 'page');
  });

  const revealItems = document.querySelectorAll('[data-reveal]');
  const observer = new IntersectionObserver((entries) => {
    entries.forEach((entry) => {
      if (!entry.isIntersecting) return;
      entry.target.classList.add('is-visible');
      entry.target.querySelectorAll('[data-count]').forEach(animateCounter);
      observer.unobserve(entry.target);
    });
  }, { threshold: .12, rootMargin: '0px 0px -35px' });
  revealItems.forEach((item) => observer.observe(item));

  function animateCounter(node) {
    if (node.dataset.counted === 'true') return;
    node.dataset.counted = 'true';
    const target = Number(node.dataset.count);
    const decimals = Number(node.dataset.decimals || 0);
    const suffix = node.dataset.suffix || '';
    const prefix = node.dataset.prefix || '';
    const start = performance.now();
    const duration = 900;
    const frame = (now) => {
      const progress = Math.min((now - start) / duration, 1);
      const eased = 1 - Math.pow(1 - progress, 3);
      node.textContent = `${prefix}${(target * eased).toFixed(decimals).replace('.', ',')}${suffix}`;
      if (progress < 1) requestAnimationFrame(frame);
    };
    requestAnimationFrame(frame);
  }

  document.querySelectorAll('[data-tabs]').forEach((group) => {
    const buttons = group.querySelectorAll('[data-tab]');
    const targetSelector = group.dataset.target;
    const panels = document.querySelectorAll(targetSelector);
    buttons.forEach((button) => {
      button.addEventListener('click', () => {
        const key = button.dataset.tab;
        buttons.forEach((item) => {
          const selected = item === button;
          item.classList.toggle('is-active', selected);
          item.setAttribute('aria-selected', String(selected));
        });
        panels.forEach((panel) => panel.classList.toggle('is-active', panel.dataset.panel === key));
      });
    });
  });

  document.querySelectorAll('[data-doc-filter]').forEach((button) => {
    button.addEventListener('click', () => {
      const category = button.dataset.docFilter;
      document.querySelectorAll('[data-doc-filter]').forEach((item) => item.classList.toggle('is-active', item === button));
      let count = 0;
      document.querySelectorAll('[data-document]').forEach((card) => {
        const visible = category === 'all' || card.dataset.document === category;
        card.hidden = !visible;
        if (visible) count += 1;
      });
      const counter = document.querySelector('[data-document-count]');
      if (counter) counter.textContent = String(count);
    });
  });

  document.querySelectorAll('[data-model-select]').forEach((button) => {
    button.addEventListener('click', () => {
      const model = button.dataset.modelSelect;
      document.querySelectorAll('[data-model-select]').forEach((item) => {
        const active = item === button;
        item.classList.toggle('is-active', active);
        item.setAttribute('aria-selected', String(active));
      });
      document.querySelectorAll('[data-model-data]').forEach((panel) => panel.classList.toggle('is-active', panel.dataset.modelData === model));
      window.dispatchEvent(new CustomEvent('mitk:modelchange', { detail: { model } }));
    });
  });

  /* ── Hero particle flow ── */
  const heroCanvas = document.querySelector('.hero-particles');
  if (heroCanvas && heroCanvas.getContext) {
    const ctx = heroCanvas.getContext('2d');
    let w, h, mouse = { x: -1000, y: -1000 };
    const particles = [];
    const TRAIL = 40;
    const COUNT = 90;

    function resize() {
      const rect = heroCanvas.parentElement.getBoundingClientRect();
      w = heroCanvas.width = rect.width;
      h = heroCanvas.height = rect.height;
    }
    resize();
    window.addEventListener('resize', resize);

    heroCanvas.parentElement.addEventListener('mousemove', (e) => {
      const rect = heroCanvas.parentElement.getBoundingClientRect();
      mouse.x = e.clientX - rect.left;
      mouse.y = e.clientY - rect.top;
    });
    heroCanvas.parentElement.addEventListener('mouseleave', () => {
      mouse.x = -1000; mouse.y = -1000;
    });

    class Particle {
      constructor() { this.reset(); }
      reset() {
        this.x = Math.random() * w;
        this.y = Math.random() * h;
        this.vx = .15 + Math.random() * .4;
        this.vy = (Math.random() - .5) * .15;
        this.life = 0;
        this.maxLife = 300 + Math.random() * 500;
        this.size = .5 + Math.random() * 1.2;
        this.hue = Math.random() > .75 ? 24 : 0;
        this.sat = this.hue === 24 ? 100 : 0;
        this.light = this.hue === 24 ? 55 : 80;
        this.trail = [];
      }
      update() {
        this.life++;
        if (this.life > this.maxLife || this.x > w + 20) { this.reset(); this.x = -10; return; }

        const dx = mouse.x - this.x;
        const dy = mouse.y - this.y;
        const dist = Math.sqrt(dx * dx + dy * dy);
        if (dist < 200 && dist > 0) {
          const force = (200 - dist) / 200 * .6;
          this.vx -= (dx / dist) * force * .02;
          this.vy -= (dy / dist) * force * .03;
        }

        this.vx += (Math.random() - .48) * .008;
        this.vy += (Math.random() - .5) * .005;
        this.vx = Math.max(.08, Math.min(this.vx, 1.2));
        this.vy = Math.max(-.4, Math.min(this.vy, .4));

        this.x += this.vx;
        this.y += this.vy;

        this.trail.push({ x: this.x, y: this.y });
        if (this.trail.length > TRAIL) this.trail.shift();
      }
      draw() {
        if (this.trail.length < 2) return;
        const fadeIn = Math.min(this.life / 40, 1);
        const fadeOut = Math.max((this.maxLife - this.life) / 60, 0);
        const alpha = Math.min(fadeIn, fadeOut) * .35;

        ctx.beginPath();
        ctx.moveTo(this.trail[0].x, this.trail[0].y);
        for (let i = 1; i < this.trail.length; i++) {
          const p = this.trail[i];
          const pp = this.trail[i - 1];
          ctx.quadraticCurveTo(pp.x, pp.y, (pp.x + p.x) / 2, (pp.y + p.y) / 2);
        }
        ctx.strokeStyle = `hsla(${this.hue}, ${this.sat}%, ${this.light}%, ${alpha})`;
        ctx.lineWidth = this.size;
        ctx.lineCap = 'round';
        ctx.stroke();
      }
    }

    for (let i = 0; i < COUNT; i++) {
      const p = new Particle();
      p.life = Math.random() * p.maxLife;
      particles.push(p);
    }

    function loop() {
      ctx.clearRect(0, 0, w, h);
      for (const p of particles) { p.update(); p.draw(); }
      requestAnimationFrame(loop);
    }
    loop();
  }

  /* ── Interactive impeller explorer ── */
  const explorer = document.querySelector('.param-explorer');
  if (explorer) {
    const svg = explorer.querySelector('.param-svg');
    const sliders = explorer.querySelectorAll('input[type="range"]');
    const a_sound = Math.sqrt(1.4 * 287.05 * 293.15);
    const p = { d2: 200, blades: 9, backsweep: 35, rpm: 12000 };

    function fmt(n, d) { return n.toFixed(d || 1).replace('.', ','); }
    function smooth(t) { return t * t * t * (t * (t * 6 - 15) + 10); }

    function draw() {
      const R2 = p.d2 / 2;
      const R1h = R2 * 0.20;
      const R1s = R2 * 0.55;
      const nb = p.blades;
      const wrap = 0.25 + p.backsweep * 0.014;
      const cx = 160, cy = 160;
      const scale = 120 / R2;

      function px(r, a) { return cx + r * scale * Math.cos(a); }
      function py(r, a) { return cy + r * scale * Math.sin(a); }

      function bladeCurve(r1, r2, ang, w, n) {
        let d = '';
        for (let i = 0; i <= n; i++) {
          const t = i / n;
          const r = r1 + (r2 - r1) * t;
          const th = ang + w * smooth(t);
          d += (i ? 'L' : 'M') + px(r, th).toFixed(1) + ',' + py(r, th).toFixed(1) + ' ';
        }
        return d;
      }

      let s = '';

      s += '<g stroke="#e8e8e5" stroke-width=".5" fill="none" opacity=".5">';
      for (let i = 1; i <= 3; i++) s += `<circle cx="${cx}" cy="${cy}" r="${(R2 * scale * i / 3).toFixed(1)}" stroke-dasharray="2 3"/>`;
      s += '</g>';

      s += `<circle cx="${cx}" cy="${cy}" r="${(R2 * scale).toFixed(1)}" fill="rgba(255,107,0,.03)" stroke="#292a42" stroke-width="1.8"/>`;
      s += `<circle cx="${cx}" cy="${cy}" r="${(R1s * scale).toFixed(1)}" fill="none" stroke="#bbb" stroke-width=".7" stroke-dasharray="4 2"/>`;

      for (let i = 0; i < nb; i++) {
        const ang = (i + 0.5) * 2 * Math.PI / nb;
        s += `<path d="${bladeCurve(R1s + 0.25 * (R2 - R1s), R2, ang, wrap * 0.55, 30)}" fill="none" stroke="#ff6b00" stroke-width="2" opacity=".55" stroke-linecap="round"/>`;
      }

      for (let i = 0; i < nb; i++) {
        const ang = i * 2 * Math.PI / nb;
        s += `<path d="${bladeCurve(R1h, R2, ang, wrap, 40)}" fill="none" stroke="#292a42" stroke-width="2.5" stroke-linecap="round"/>`;
      }

      s += `<circle cx="${cx}" cy="${cy}" r="${(R1h * scale).toFixed(1)}" fill="#292a42"/>`;
      s += `<circle cx="${cx}" cy="${cy}" r="${(R1h * scale * 0.35).toFixed(1)}" fill="#ff6b00" opacity=".25"/>`;
      s += `<line x1="${cx - 3}" y1="${cy}" x2="${cx + 3}" y2="${cy}" stroke="#fff" stroke-width="1"/><line x1="${cx}" y1="${cy - 3}" x2="${cx}" y2="${cy + 3}" stroke="#fff" stroke-width="1"/>`;

      s += `<line x1="${cx}" y1="${cy - (R2 * scale).toFixed(1) - 12}" x2="${cx}" y2="${cy - (R2 * scale).toFixed(1) - 2}" stroke="#888" stroke-width=".6"/>`;
      s += `<text x="${cx}" y="${cy - (R2 * scale).toFixed(1) - 16}" text-anchor="middle" font-family="ui-monospace,monospace" font-size="9" fill="#888">Ø${p.d2}</text>`;

      s += `<text x="${cx + R2 * scale + 8}" y="${cy + 4}" font-family="ui-monospace,monospace" font-size="9" fill="#aaa">z↓</text>`;

      svg.innerHTML = s;

      const omega = 2 * Math.PI * p.rpm / 60;
      const U2 = omega * (p.d2 / 2000);
      const mach = U2 / a_sound;
      const beta2 = 90 - p.backsweep;

      document.getElementById('res-u2').textContent = fmt(U2);
      document.getElementById('res-mach').textContent = fmt(mach, 3);
      document.getElementById('res-beta2').textContent = beta2;
    }

    sliders.forEach((sl) => {
      sl.addEventListener('input', () => {
        const key = sl.dataset.param;
        const v = Number(sl.value);
        const out = sl.parentElement.querySelector('output');
        if (key === 'rpm') { p[key] = v; out.textContent = v.toLocaleString('ru-RU'); }
        else { p[key] = v; out.textContent = v; }
        draw();
      });
    });

    draw();
  }
})();
