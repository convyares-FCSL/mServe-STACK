// Shared top-down LaserScan canvas renderer — used by lidar.html (full page,
// zoomable) and base.html (mini preview alongside the camera). Pure render
// function: caller owns the canvas, the zoom level, and any stats.

function drawLaserScan(ctx, canvas, msg, viewRangeM) {
  const w = canvas.width, h = canvas.height;
  const cx = w / 2, cy = h / 2;
  const pxPerMeter = (Math.min(w, h) / 2 - 12) / viewRangeM;

  ctx.clearRect(0, 0, w, h);

  // Range rings every 2m
  ctx.strokeStyle = '#1e293b';
  ctx.lineWidth = 1;
  for (let r = 2; r <= viewRangeM; r += 2) {
    ctx.beginPath();
    ctx.arc(cx, cy, r * pxPerMeter, 0, Math.PI * 2);
    ctx.stroke();
  }
  // Forward axis
  ctx.strokeStyle = '#334155';
  ctx.beginPath();
  ctx.moveTo(cx, cy); ctx.lineTo(cx, cy - (Math.min(w, h) / 2 - 12));
  ctx.stroke();

  // Points — angle measured from +X (forward), counter-clockwise (REP-103);
  // canvas has +Y down, forward drawn as "up", so: screen_x = cx - r*sin(a), screen_y = cy - r*cos(a)
  ctx.fillStyle = '#38bdf8';
  for (let i = 0; i < msg.ranges.length; i++) {
    const r = msg.ranges[i];
    if (!Number.isFinite(r) || r < msg.range_min || r > msg.range_max || r > viewRangeM) continue;
    const angle = msg.angle_min + i * msg.angle_increment;
    const x = cx - r * Math.sin(angle) * pxPerMeter;
    const y = cy - r * Math.cos(angle) * pxPerMeter;
    ctx.fillRect(x - 1, y - 1, 2, 2);
  }

  // Robot marker
  ctx.fillStyle = '#f87171';
  ctx.beginPath();
  ctx.arc(cx, cy, 4, 0, Math.PI * 2);
  ctx.fill();
}
