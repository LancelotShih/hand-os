// Rendering-only permutation/signs. Calibration and saved coordinates never change.
let displayAxes = ["x", "y", "z"];
const DISPLAY_AXIS_OPTIONS = ["x", "-x", "y", "-y", "z", "-z"];
function validDisplayAxes(axes) {
  return Array.isArray(axes) && axes.length === 3 && axes.every(a => DISPLAY_AXIS_OPTIONS.includes(a))
    && new Set(axes.map(a => a.replace("-", ""))).size === 3;
}
function displayVector(v) {
  return displayAxes.map(a => (a.startsWith("-") ? -1 : 1) * v["xyz".indexOf(a.replace("-", ""))]);
}
function displayBasis() {
  return displayAxes.map(a => [0, 1, 2].map(k => "xyz"[k] === a.replace("-", "") ? (a.startsWith("-") ? -1 : 1) : 0));
}
(function displayAxisControls() {
  try {
    const saved = JSON.parse(localStorage.getItem("magtrack.displayAxes"));
    if (validDisplayAxes(saved)) displayAxes = saved;
  } catch (_) {}
  const picks = ["displayx", "displayy", "displayz"].map(id => $(id));
  const show = () => picks.forEach((pick, k) => { pick.value = displayAxes[k]; });
  picks.forEach(pick => {
    DISPLAY_AXIS_OPTIONS.forEach(value => {
      const option = document.createElement("option");
      option.value = value; option.textContent = (value.startsWith("-") ? "-" : "+") + value.replace("-", "").toUpperCase();
      pick.appendChild(option);
    });
    pick.onchange = () => {
      const proposed = picks.map(p => p.value), k = picks.indexOf(pick);
      const other = displayAxes.findIndex((a, i) => i !== k && a.replace("-", "") === pick.value.replace("-", ""));
      if (other >= 0) proposed[other] = displayAxes[k];
      if (validDisplayAxes(proposed)) displayAxes = proposed;
      try { localStorage.setItem("magtrack.displayAxes", JSON.stringify(displayAxes)); } catch (_) {}
      show();
      if (typeof posLatest !== "undefined" && posLatest) positionSample(posLatest);
    };
  });
  $("axesreset").onclick = () => {
    displayAxes = ["x", "y", "z"]; show();
    try { localStorage.removeItem("magtrack.displayAxes"); } catch (_) {}
    if (typeof posLatest !== "undefined" && posLatest) positionSample(posLatest);
  };
  show();
})();
