// One panel per magnetometer, stamped from the same template, so they show the
// same things the same way.  ``part`` finds a piece of a panel by its class.
// The MMC's panel is hidden while it's switched off (the default; --mmc turns it on).
const SENSORS = {icm: "ICM-20948", mmc: "MMC5983MA"};
const SIDES = Object.keys(SENSORS).map(key => {
  const panel = $("side").content.firstElementChild.cloneNode(true);
  panel.querySelector(".sidename").textContent = SENSORS[key];
  $("sides").appendChild(panel);
  return {key, panel, part: name => panel.querySelector("." + name), hist: [], trail: []};
});
