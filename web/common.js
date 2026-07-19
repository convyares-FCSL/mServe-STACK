// Shared header/footer, injected into each page's #site-header/#site-footer
// placeholder — one place to edit when a page is added/renamed instead of N
// copies of the same nav links to keep in sync by hand.

const PAGES = [
  { href: 'index.html', label: 'Debug Bridge' },
  { href: 'base.html', label: 'Base' },
  { href: 'drivechain.html', label: 'Drivechain' },
  { href: 'camera.html', label: 'Camera' },
  { href: 'lidar.html', label: 'LiDAR' },
  { href: 'joystick.html', label: 'Joystick' },
  { href: 'sensehat.html', label: 'Sense HAT' },
];

function renderSiteHeader() {
  const el = document.getElementById('site-header');
  if (!el) return;
  const current = window.location.pathname.split('/').pop() || 'index.html';
  const links = PAGES.map(({ href, label }) => (
    href === current
      ? `<span class="site-nav-current">${label}</span>`
      : `<a href="${href}">${label}</a>`
  )).join('<span class="site-nav-sep">|</span>');
  el.innerHTML = `<nav class="site-nav">${links}</nav>`;
}

function renderSiteFooter() {
  const el = document.getElementById('site-footer');
  if (!el) return;
  el.innerHTML =
    '<footer class="site-footer">' +
    '<a href="https://github.com/convyares-FCSL/mServe-STACK" target="_blank" rel="noopener">mServe-STACK on GitHub →</a>' +
    '</footer>';
}

renderSiteHeader();
renderSiteFooter();
