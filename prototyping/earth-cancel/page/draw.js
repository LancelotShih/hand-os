// Small helpers shared by the page's scripts.
const $ = id => document.getElementById(id);
const css = n => getComputedStyle(document.documentElement).getPropertyValue(n).trim();

function fit(c) {
  const r = c.getBoundingClientRect(), s = devicePixelRatio || 1;
  if (c.width !== Math.round(r.width * s) || c.height !== Math.round(r.height * s)) {
    c.width = Math.round(r.width * s); c.height = Math.round(r.height * s);
  }
  const g = c.getContext("2d");
  g.setTransform(s, 0, 0, s, 0, 0);
  g.clearRect(0, 0, r.width, r.height);
  return [g, r.width, r.height];
}
