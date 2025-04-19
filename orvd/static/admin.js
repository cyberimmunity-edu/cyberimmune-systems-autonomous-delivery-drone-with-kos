const TILES_URL = "https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}";
const TILES_LOCAL_PATH = "static/resources/tiles";

function getSearchParameters() {
  var prmstr = window.location.search.substr(1);
  return prmstr != null && prmstr != "" ? transformToAssocArray(prmstr) : {};
}

function transformToAssocArray( prmstr ) {
  var params = {};
  var prmarr = prmstr.split("&");
  for ( var i = 0; i < prmarr.length; i++) {
    var tmparr = prmarr[i].split("=");
    params[tmparr[0]] = tmparr[1];
  }
  return params;
}

var params = getSearchParameters();

document.getElementById('arm').onclick = arm;
document.getElementById('disarm').onclick = disarm;
document.getElementById("id_select").addEventListener("change", onChangeSelector, false);
document.getElementById('mission_checkbox').onclick = mission_decision;
document.getElementById('kill_switch').onclick = kill_switch;
document.getElementById('fly_accept_checkbox').onclick = fly_accept;
document.getElementById('forbidden_zones_checkbox').onclick = toggleForbiddenZones;
document.getElementById('revised-mission-accept').onclick = () => revised_mission_decision(0);
document.getElementById('revised-mission-decline').onclick = () => revised_mission_decision(1);
document.getElementById('monitoring-checkbox').onclick = () => toggle_display_mode();
document.getElementById('flight-info-checkbox').onclick = () => toggle_flight_info_response_mode();
document.getElementById('copy-id').onclick = () => copy_current_id();
document.getElementById('toggle-trajectory').onclick = toggleTrajectory;


ol.proj.useGeographic()
const place = [27.85731575, 60.0026278];

let ids = [];
let active_id = null;
let current_mission = null;
let uav = null;
let access_token = params.token;
var current_state = null;
let forbidden_zones_display = false;
let trajectoryFeature = null;

async function copyToClipboard(textToCopy) {
  if (navigator.clipboard && window.isSecureContext) {
    await navigator.clipboard.writeText(textToCopy);
  } else {
    const textArea = document.createElement("textarea");
    textArea.value = textToCopy;
    
    textArea.style.position = "absolute";
    textArea.style.left = "-999999px";
    
    document.body.prepend(textArea);
    textArea.select();
    
    try {
      document.execCommand('copy');
    } catch (error) {
      console.error(error);
    } finally {
      textArea.remove();
    }
  }
}

async function copy_current_id() {
  if (active_id !== null) {
    try {
      await copyToClipboard(active_id);
      console.log('Current ID copied to clipboard:', active_id);
      alert("ID " + active_id + " copied to clipboard!");
    } catch (err) {
      console.error('Failed to copy ID:', err);
    }
  } else {
    console.log('No active ID to copy.');
    alert("No active ID to copy.");
  }
}

let uav_style = new ol.style.Style({
  image: new ol.style.Icon({
    anchor: [0.5, 0.5],
    src: 'static/resources/vehicle_marker.svg',
    scale: 0.25
  })
});

let inactive_uav_style = new ol.style.Style({
  image: new ol.style.Icon({
    anchor: [0.5, 0.5],
    src: 'static/resources/vehicle_marker.svg',
    scale: 0.25,
    opacity: 0.6
  })
});

let home_marker_style = new ol.style.Style({
  image: new ol.style.Icon({
    anchor: [0.5, 1],
    src: 'static/resources/home_marker.png'
  })
})

let regular_marker_style = new ol.style.Style({
  image: new ol.style.Icon({
    anchor: [0.5, 1],
    src: 'static/resources/waypoint_marker.png'
  })
})

let servo_marker_style = new ol.style.Style({
  image: new ol.style.Icon({
    anchor: [0.5, 1],
    src: 'static/resources/servo_marker.png'
  })
})

let delay_marker_style = new ol.style.Style({
  image: new ol.style.Icon({
    anchor: [0.5, 1],
    src: 'static/resources/delay_marker.png'
  })
})

let polyline_style = new ol.style.Style({
  stroke: new ol.style.Stroke({
    color: [0, 0, 0, 255],
    width: 2
  })
});

let trajectory_style = new ol.style.Style({
  stroke: new ol.style.Stroke({
    color: 'yellow',
    width: 3
  })
});

let availableTiles = [];

await fetch('/tiles/index')
.then(response => response.json())
.then(data => {
  availableTiles = data;
});

function customTileLoadFunction(imageTile, src) {
  const urlPattern = /x=([0-9]+)&y=([0-9]+)&z=([0-9]+)/;
  const matches = src.match(urlPattern);
  const x = matches[1];
  const y = matches[2];
  const z = matches[3];
  
  const tilePath = `${z}/${x}/${y}`;
  const localUrl = `${TILES_LOCAL_PATH}/${tilePath}.png`;
  
  if (availableTiles.includes(tilePath)) {
    imageTile.getImage().src = localUrl;
  } else {
    imageTile.getImage().src = src;
  }
}

const tileLayer = new ol.layer.Tile({
  source: new ol.source.XYZ({
    url: TILES_URL,
    tileLoadFunction: customTileLoadFunction
  })
});

const map = new ol.Map({
  target: 'map',
  layers: [
    tileLayer
  ],
  view: new ol.View({
    center: place,
    zoom: 15,
  }),
});

function createArcLine(center, radius, startAngle, endAngle, points = 64) {
  const angleStep = (endAngle - startAngle) / points;
  const coordinates = [];
  for (let i = 0; i <= points; i++) {
    const angle = startAngle + i * angleStep;
    const lon = center[0] + radius * Math.cos(angle);
    const lat = center[1] + radius * Math.sin(angle) / 2;
    coordinates.push([lon, lat]);
  }
  return new ol.geom.LineString(coordinates);
}

const fieldLayer = new ol.layer.Vector({
  source: new ol.source.Vector({})
});
const fieldLayerMainColor = 'rgba(51, 153, 204, 1)';

const mainFieldStyle = new ol.style.Style({
  fill: new ol.style.Fill({
    color: 'rgba(255, 255, 255, 0.2)',
  }),
  stroke: new ol.style.Stroke({
    color: fieldLayerMainColor,
    width: 2,
  }),
});

const fieldDashStrokeStyle = new ol.style.Style({
  stroke: new ol.style.Stroke({
    color: fieldLayerMainColor,
    lineDash: [4],
    width: 1,
  }),
});

const polygon = new ol.geom.Polygon([
  [
    [
      27.8573025,
      60.0027103
    ],
    [
      27.8574654,
      60.0026645
    ],
    [
      27.857329,
      60.0025433
    ],
    [
      27.8571673,
      60.002593
    ],
    [
      27.8573025,
      60.0027103
    ]
  ],
]);
const polygonFeature = new ol.Feature(polygon);
polygonFeature.setStyle(mainFieldStyle);
fieldLayer.getSource().addFeature(polygonFeature);

const mainLine = new ol.Feature(new ol.geom.LineString([[27.8572349, 60.00265165],[27.8573972, 60.0026039]]));
mainLine.setStyle(
  new ol.style.Style({
    stroke: new ol.style.Stroke({
      color: fieldLayerMainColor,
      width: 2,
    }),
  })
);
fieldLayer.getSource().addFeature(mainLine);

const firstLine = new ol.Feature(new ol.geom.LineString([[27.8572574, 60.0026712],[27.8574199, 60.0026241]]));
const secondLine = new ol.Feature(new ol.geom.LineString([[27.8572123, 60.0026321],[27.8573744, 60.0025837]]));
firstLine.setStyle(fieldDashStrokeStyle);
secondLine.setStyle(fieldDashStrokeStyle);
fieldLayer.getSource().addFeature(firstLine);
fieldLayer.getSource().addFeature(secondLine);

const firstArcBase = -0.514079;
const firstArc = new ol.Feature(
  createArcLine([27.85738395, 60.0026874], 8e-5, firstArcBase + Math.PI, 2*Math.PI + firstArcBase)
)
firstArc.setStyle(fieldDashStrokeStyle);
fieldLayer.getSource().addFeature(firstArc);

const secondArcBase = -0.554079;
const secondArc = new ol.Feature(
  createArcLine([27.85724815, 60.00256815], 8e-5, secondArcBase, secondArcBase + Math.PI)
)
secondArc.setStyle(fieldDashStrokeStyle);
fieldLayer.getSource().addFeature(secondArc);

const circleFeature = new ol.Feature(new ol.geom.Circle([27.85731575, 60.0026278], 2e-5));
circleFeature.setStyle(
  new ol.style.Style({
    fill: new ol.style.Fill({
      color: 'rgba(255, 255, 255, 0.2)',
    }),
    stroke: new ol.style.Stroke({
      color: fieldLayerMainColor,
      width: 2,
    }),
  })
);
fieldLayer.getSource().addFeature(circleFeature);

map.addLayer(fieldLayer);

const styles = {
  'Polygon': new ol.style.Style({
    stroke: new ol.style.Stroke({
      color: 'red',
      lineDash: [4],
      width: 3,
    }),
    fill: new ol.style.Fill({
      color: 'rgba(255, 0, 0, 0.1)',
    }),
  })
};

const styleFunction = function (feature) {
  return styles[feature.getGeometry().getType()];
};

let geoJSONLayer;

async function createGeoJSONLayer() {
  const response = await fetch(`/admin/get_forbidden_zones?token=${access_token}`);
  if (!response.ok) {
    console.error('Failed to fetch forbidden zones');
    return;
  }
  const geojsonObject = await response.json();
  
  if (geoJSONLayer) {
    map.removeLayer(geoJSONLayer);
  }
  
  geoJSONLayer = new ol.layer.Vector({
    source: new ol.source.Vector({
      features: new ol.format.GeoJSON().readFeatures(geojsonObject),
    }),
    style: styleFunction
  });
  
  geoJSONLayer.getSource().getFeatures().forEach(feature => {
    feature.set('description', `Запрещенная зона: ${feature.get('name')}`);
  });
  
  map.addLayer(geoJSONLayer);
}

var markers = new ol.layer.Vector({
  source: new ol.source.Vector(),
});

map.addLayer(markers);

const info = document.getElementById('info');

let currentFeature;
const displayFeatureInfo = function (pixel, target) {
  const feature = target.closest('.ol-control')
  ? undefined
  : map.forEachFeatureAtPixel(pixel, function (feature) {
    return feature;
  });
  if (feature) {
    info.style.left = pixel[0] + 'px';
    info.style.top = pixel[1] + 'px';
    if (feature !== currentFeature) {
      info.style.visibility = 'visible';
      info.innerText = feature.get('description') || 'Нет описания';
    }
  } else {
    info.style.visibility = 'hidden';
  }
  currentFeature = feature;
};

map.on('pointermove', function (evt) {
  if (evt.dragging) {
    info.style.visibility = 'hidden';
    currentFeature = undefined;
    return;
  }
  const pixel = map.getEventPixel(evt.originalEvent);
  displayFeatureInfo(pixel, evt.originalEvent.target);
});

map.on('click', function (evt) {
  displayFeatureInfo(evt.pixel, evt.originalEvent.target);
});

map.getTargetElement().addEventListener('pointerleave', function () {
  currentFeature = undefined;
  info.style.visibility = 'hidden';
});


function clear_markers() {
  var features = markers.getSource().getFeatures();
  features.forEach((feature) => {
    const feature_id = feature.getId();
    if(feature_id === undefined || !feature.getId().includes('uav')) {
      markers.getSource().removeFeature(feature);
    }
  });
}

function add_marker(lat, lon, alt, marker_type) {
  if (marker_type == 'uav') {
    uav = new ol.Feature(new ol.geom.Point([lon, lat]));
    uav.setId('uav')
    uav.setStyle(uav_style);
    uav.set('description', 'Высота: ' + alt);
    markers.getSource().addFeature(uav);
  } else {
    var marker = new ol.Feature(new ol.geom.Point([lon, lat]));
    if (marker_type == 'home') {
      marker.setStyle(home_marker_style);
      marker.set('description', 'Высота: ' + alt);
    } else if (marker_type == 'servo') {
      marker.setStyle(servo_marker_style);
      marker.set('description', 'Сброс груза\nВысота: ' + alt);
    } else if (marker_type === 'delay') {
      marker.setStyle(delay_marker_style);
      marker.set('description', 'Задержка\nВысота: ' + alt);
    } else {
      marker.setStyle(regular_marker_style);
      marker.set('description', 'Высота: ' + alt);
    }
    markers.getSource().addFeature(marker);
  }
}

function add_polyline(line_path) {
  var polyline = new ol.geom.MultiLineString([line_path]);
  var polyline_feature = new ol.Feature({
    name: "Thing",
    geometry: polyline,
    description: null
  });
  polyline_feature.setStyle(polyline_style);
  markers.getSource().addFeature(polyline_feature);
}

function onChangeSelector() {
  let id_select = document.getElementById("id_select");
  change_active_id(id_select.options[id_select.selectedIndex].text);
}

async function arm() {
  console.log('arm')
  if (active_id != null) {
    let resp = await fetch("admin/arm_decision?id=" + active_id + "&decision=0" + "&token=" + access_token);
    console.log(await resp.text());
    status_change();
  }
}

async function disarm() {
  if (active_id != null) {
    let resp = await fetch("admin/arm_decision?id=" + active_id + "&decision=1" + "&token=" + access_token);
    console.log(await resp.text());
    status_change();
  }
}

async function f_disarm() {
  if (active_id != null) {
    let resp = await fetch("admin/force_disarm?id=" + active_id + "&token=" + access_token);
    console.log(await resp.text());
    status_change();
  }
}

async function fa_disarm() {
  if (active_id != null) {
    let resp = await fetch("admin/force_disarm_all" + "?token=" + access_token);
    console.log(await resp.text());
    status_change();
  }
}

async function kill_switch() {
  if (active_id != null) {
    let resp = await fetch("admin/kill_switch?id=" + active_id + "&token=" + access_token);
    console.log(await resp.text());
    status_change();
  }
}

async function mission_decision() {
  let mission_checkbox = document.getElementById('mission_checkbox');
  if (current_mission == null || current_mission == '$-1') {
    mission_checkbox.checked = false;
  }
  else {
    let query_str = "admin/mission_decision?id=" + active_id + "&decision=";
    if (mission_checkbox.checked) {
      query_str += 0;
    }
    else {
      query_str += 1;
    }
    query_str += "&token=" + access_token
    let mission_resp = await fetch(query_str);
    let mission_text = await mission_resp.text();
    console.log(mission_text);
  }
}

async function revised_mission_decision(decision) {
  const revisedMissionBlock = document.getElementById('revised-mission-block');
  revisedMissionBlock.style.visibility = 'hidden';
  const query_str = `admin/revise_mission_decision?id=${active_id}&decision=${decision}&token=${access_token}`;
  let mission_resp = await fetch(query_str);
  let mission_text = await mission_resp.text();
  console.log(mission_text);
}

async function toggle_display_mode() {
  const query_str = `admin/toggle_display_mode?token=${access_token}`;
  await fetch(query_str);
  const $monitoringCheckbox = document.getElementById('monitoring-checkbox')
  const $mainButtons = document.getElementById('main-buttons');
  if($monitoringCheckbox.checked) {
    $mainButtons.style.visibility = 'hidden';
  } else {
    $mainButtons.style.visibility = 'visible';
  }
}

async function get_display_mode() {
  const delay_resp = await fetch(`admin/get_display_mode?token=${access_token}`);
  const delay_text = await delay_resp.text();
  const $monitoringCheckbox = document.getElementById('monitoring-checkbox')
  const $mainButtons = document.getElementById('main-buttons');
  if (delay_text === '0') {
    $monitoringCheckbox.checked = true;
    $mainButtons.style.visibility = 'hidden';
  } else {
    $monitoringCheckbox.checked = false;
    $mainButtons.style.visibility = 'visible';
  }
}

async function toggle_flight_info_response_mode() {
  const query_str = `admin/toggle_flight_info_response_mode?token=${access_token}`;
  await fetch(query_str);
  await get_flight_info_response_mode();
}

async function get_flight_info_response_mode() {
  const delay_resp = await fetch(`admin/get_flight_info_response_mode?token=${access_token}`);
  const delay_text = await delay_resp.text();
  const $monitoringCheckbox = document.getElementById('flight-info-checkbox')
  if (delay_text === '0') {
    $monitoringCheckbox.checked = true;
  } else {
    $monitoringCheckbox.checked = false;
  }
}

async function fly_accept() {
  let fly_accept_checkbox = document.getElementById('fly_accept_checkbox');
  if (active_id == null || current_state == "Kill switch ON") {
    fly_accept_checkbox.checked = false;
  } else {
    let query_str = "admin/change_fly_accept?id=" + active_id + "&decision=";
    if (fly_accept_checkbox.checked) {
      query_str += 0;
    }
    else {
      query_str += 1;
    }
    query_str += "&token=" + access_token
    let fly_accept_resp = await fetch(query_str);
    let fly_accept_text = await fly_accept_resp.text();
    console.log(fly_accept_text);
  }
}

async function status_change() {
  let state_resp = await fetch("admin/get_state?id=" + active_id + "&token=" + access_token);
  let state_text = await state_resp.text();
  document.getElementById("status").innerHTML="Статус: " + state_text;
  current_state = state_text;
  if (state_text == 'В полете') {
    document.getElementById('fly_accept_checkbox').checked = true;
  } else {
    document.getElementById('fly_accept_checkbox').checked = false;
  }
}

async function waiters_change() {
  let waiter_resp = await fetch("admin/get_waiter_number" + "?token=" + access_token);
  let waiter_text = await waiter_resp.text();
  document.getElementById("waiters").innerHTML="Ожидают: " + waiter_text;
  let waiters_num = parseInt(waiter_text);
  if (waiters_num > 0) {
    document.getElementById('arm').disabled = false;
    document.getElementById('disarm').disabled = false;
  } else {
    document.getElementById('arm').disabled = true;
    document.getElementById('disarm').disabled = true;
  }
}

async function get_mission(id) {
  let mission_resp = await fetch("admin/get_mission?id=" + id + "&token=" + access_token);
  let mission_text = await mission_resp.text();
  if (mission_text == '$-1') {
    current_mission = mission_text;
  }
  if (mission_text != '$-1' && current_mission != mission_text) {
    clear_markers()
    console.log(mission_text)
    current_mission = mission_text;
    let mission_path = [];
    let mission_list = mission_text.split('&');
    for (let idx = 0; idx < mission_list.length; ++idx) {
      mission_list[idx] = [Array.from(mission_list[idx])[0]].concat(mission_list[idx].slice(1).split('_'));
    }
    for (let idx = 0; idx < mission_list.length; ++idx) {
      if (mission_list[idx][0] == 'H') {
        var lat = parseFloat(mission_list[idx][1]);
        var lon = parseFloat(mission_list[idx][2]);
        var alt = mission_list[idx][3];
        add_marker(lat, lon, alt, 'home');
        mission_path.push([lon, lat]);
        // map.getView().setCenter([lon, lat]);
      }
      else if (mission_list[idx][0] == 'W') {
        var lat = parseFloat(mission_list[idx][1]);
        var lon = parseFloat(mission_list[idx][2]);
        var alt = mission_list[idx][3];
        if (idx < mission_list.length - 1 && mission_list[idx+1][0] == 'S') {
          add_marker(lat, lon, alt, 'servo');              
        } else if (idx < mission_list.length - 1 && mission_list[idx+1][0] == 'D') {
          add_marker(lat, lon, alt, 'delay');
        }
        else {
          add_marker(lat, lon, alt, 'regular');
        }
        mission_path.push([lon, lat]);
      }
      else if (mission_list[idx][0] == 'L'){
        var lat = parseFloat(mission_list[idx][1]);
        var lon = parseFloat(mission_list[idx][2]);
        var alt = mission_list[idx][3];
        add_marker(lat, lon, alt, 'regular');
      }         
    }
    add_polyline(mission_path);
  }
}

let vehicles = {};

function add_or_update_vehicle_marker(id, lat, lon, alt, azimuth, speed) {
  //let rotationInRadians = azimuth * Math.PI / 180;
  const rotationInRadians = 0;
  let vehicleStyle = new ol.style.Style({
    image: new ol.style.Icon({
      anchor: [0.5, 0.5],
      src: 'static/resources/vehicle_marker.svg',
      scale: 0.25,
      rotation: rotationInRadians
    })
  });
  
  let inactiveVehicleStyle = new ol.style.Style({
    image: new ol.style.Icon({
      anchor: [0.5, 0.5],
      src: 'static/resources/vehicle_marker.svg',
      scale: 0.25,
      opacity: 0.6,
      rotation: rotationInRadians
    })
  });
  
  if (!vehicles[id]) {
    let vehicle = new ol.Feature(new ol.geom.Point([lon, lat]));
    vehicle.setId(`uav${id}`);
    vehicle.setStyle(id === active_id ? vehicleStyle : inactiveVehicleStyle);
    vehicle.set('description', `ID: ${id}\nВысота: ${alt}\nСкорость: ${speed}\nНаправление: ${azimuth}`);
    vehicles[id] = vehicle;
    markers.getSource().addFeature(vehicles[id]);
  } else {
    vehicles[id].getGeometry().setCoordinates([lon, lat]);
    vehicles[id].set('description', `ID: ${id}\nВысота: ${alt}\nСкорость: ${speed}\nНаправление: ${azimuth}`);
    vehicles[id].setStyle(id === active_id ? vehicleStyle : inactiveVehicleStyle);
  }
}

async function get_telemetry(id) {
  let telemetry_resp = await fetch("admin/get_telemetry?id=" + id + "&token=" + access_token);
  if (telemetry_resp.ok) {
    let telemetry_data = await telemetry_resp.json();
    if ('error' in telemetry_data) {
      return;
    }
    let lat = parseFloat(telemetry_data.lat);
    let lon = parseFloat(telemetry_data.lon);
    let alt = parseFloat(telemetry_data.alt);
    let azimuth = parseFloat(telemetry_data.azimuth);
    let dop = parseFloat(telemetry_data.dop);
    let sats = parseInt(telemetry_data.sats);
    let speed = parseFloat(telemetry_data.speed);
    document.getElementById("dop").innerHTML = "DOP: " + dop;
    document.getElementById("sats").innerHTML = "SATS: " + sats;
    add_or_update_vehicle_marker(id, lat, lon, alt, azimuth, speed);
    if (id === active_id) {
      map.getView().setCenter([lon, lat]);
    }
  } else {
    console.error("Failed to fetch telemetry data");
  }
}

async function get_mission_state(id) {
  let mission_state_resp = await fetch("admin/get_mission_state?id=" + id + "&token=" + access_token);
  let mission_state_text = await mission_state_resp.text();
  if (mission_state_text == '0') {
    document.getElementById('mission_checkbox').checked = true;
  } else if (mission_state_text == '1')  {
    document.getElementById('mission_checkbox').checked = false;
  } else if (mission_state_text == '2') {
    document.getElementById('mission_checkbox').checked = false;
    const revisedMissionBlock = document.getElementById('revised-mission-block');
    revisedMissionBlock.style.visibility = 'visible';
  } else {
    document.getElementById('mission_checkbox').checked = false;
  }
}

async function change_active_id(new_id) {
  const previous_active_id = active_id;
  active_id = new_id;
  current_mission = null;
  if (previous_active_id !== new_id) {
    removeTrajectory();
  }
  clear_markers()
  status_change();
  await get_mission(new_id);
  for(let idx = 0; idx < ids.length; idx++) {
    get_telemetry(ids[idx]);
  }
  get_mission_state(new_id);
  get_delay();
}


async function get_ids() {
  let ids_resp = await fetch("admin/get_id_list" + "?token=" + access_token);
  let ids_text = await ids_resp.text();
  let new_ids = JSON.parse(ids_text.replace(/'/g, '"'));
  let old_ids_len = ids.length;
  ids = new_ids;
  if (active_id == null && ids.length > 0) {
    change_active_id(ids[0]);
  }
  
  let id_select = document.getElementById("id_select");
  
  for (var i = old_ids_len; i<ids.length; i++){
    var opt = document.createElement('option');
    opt.value = ids[i];
    opt.innerHTML = ids[i];
    id_select.appendChild(opt);
  }
  
}


async function updateForbiddenZones() {
  await createGeoJSONLayer();
}

function toggleForbiddenZones() {
  forbidden_zones_display = document.getElementById('forbidden_zones_checkbox').checked;
  if (forbidden_zones_display) {
    updateForbiddenZones();
  } else if (geoJSONLayer) {
    map.removeLayer(geoJSONLayer);
    geoJSONLayer = null;
  }
}


document.getElementById('set_delay').onclick = set_delay;

async function set_delay() {
  let delay_value = document.getElementById('delay_input').value;
  if (active_id != null && delay_value != null) {
    let resp = await fetch(`admin/set_delay?id=${active_id}&delay=${delay_value}&token=${access_token}`);
    console.log(await resp.text());
    status_change();
  }
}

async function get_delay() {
  if (active_id != null) {
    let delay_resp = await fetch(`admin/get_delay?id=${active_id}&token=${access_token}`);
    let delay_text = await delay_resp.text();
    document.getElementById("delay").innerHTML = "Delay: " + delay_text;
  }
}

async function getAllData() {
  try {
    const response = await fetch(`/admin/get_all_data?token=${access_token}`);
    if (!response.ok) {
      throw new Error(`HTTP error! status: ${response.status}`);
    }
    const data = await response.json();
    
    ids = data.ids;
    let id_select = document.getElementById("id_select");
    const selectedIndex = id_select.selectedIndex;
    id_select.innerHTML = '';
    for (let i = 0; i < ids.length; i++) {
      let opt = document.createElement('option');
      opt.value = ids[i];
      opt.innerHTML = ids[i];
      id_select.appendChild(opt);
    }
    if (selectedIndex >= 0 && selectedIndex < id_select.options.length) {
      id_select.selectedIndex = selectedIndex;
    } else if(ids.length > 0) {
      change_active_id(ids[0]);
    }
    
    document.getElementById("waiters").innerHTML = "Ожидают: " + data.waiters;
    document.getElementById('arm').disabled = !(parseInt(data.waiters) > 0);
    document.getElementById('disarm').disabled = !(parseInt(data.waiters) > 0);
    
    for (const id in data.uav_data) {
      const uavData = data.uav_data[id];
      
      if (id === active_id) {
        document.getElementById("status").innerHTML = "Статус: " + uavData.state;
        current_state = uavData.state;
        
        if (uavData.state == 'В полете') {
          document.getElementById('fly_accept_checkbox').checked = true;
        } else {
          document.getElementById('fly_accept_checkbox').checked = false;
        }
      }
      
      if (uavData.telemetry) {
        const { lat, lon, alt, azimuth, dop, sats, speed } = uavData.telemetry;
        document.getElementById("dop").innerHTML = "DOP: " + dop;
        document.getElementById("sats").innerHTML = "SATS: " + sats;
        add_or_update_vehicle_marker(id, lat, lon, alt, azimuth, speed);
        if (id === active_id) {
          map.getView().setCenter([lon, lat]);
        }
      }
      
      if (id === active_id) {
        if (uavData.mission_state == '0') {
          document.getElementById('mission_checkbox').checked = true;
        } else if (uavData.mission_state == '1')  {
          document.getElementById('mission_checkbox').checked = false;
        } else if (uavData.mission_state == '2') {
          document.getElementById('mission_checkbox').checked = false;
          const revisedMissionBlock = document.getElementById('revised-mission-block');
          revisedMissionBlock.style.visibility = 'visible';
        } else {
          document.getElementById('mission_checkbox').checked = false;
        }
      }
      
      if (id === active_id) {
        document.getElementById("delay").innerHTML = "Delay: " + uavData.delay;
      }
    }
    
    if (forbidden_zones_display) {
      await updateForbiddenZones();
    }
    
    get_display_mode();
    get_flight_info_response_mode();
    
  } catch (error) {
    console.error("Failed to fetch all data:", error);
  }
}

setInterval(async function() {
  if(active_id) {
    await get_mission(active_id);
    await getAllData();
  } else {
    get_ids();
    get_display_mode();
    get_flight_info_response_mode();
    if (forbidden_zones_display) {
      await updateForbiddenZones();
    }
  }
}, 1000);

function removeTrajectory() {
  if (trajectoryFeature) {
    markers.getSource().removeFeature(trajectoryFeature);
    trajectoryFeature = null;
    console.log('Trajectory removed.');
  }
}

async function toggleTrajectory() {
  if (trajectoryFeature) {
    removeTrajectory();
  } else {
    if (!active_id) {
      alert("Please select a drone ID first.");
      return;
    }
    console.log(`Fetching trajectory for ID: ${active_id}`);
    try {
      const response = await fetch(`/logs/get_telemetry_csv?id=${active_id}`);
      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }
      const csvData = await response.text();
      console.log('Received CSV data.');
      
      const lines = csvData.trim().split('\n');
      if (lines.length <= 1) {
        console.log('No trajectory data found.');
        alert('No trajectory data available for this drone.');
        return;
      }
      
      const coordinates = [];
      const headers = lines[0].split(',');
      const latIndex = headers.indexOf('lat');
      const lonIndex = headers.indexOf('lon');
      
      if (latIndex === -1 || lonIndex === -1) {
        throw new Error('CSV data does not contain lat or lon columns.');
      }
      
      for (let i = 1; i < lines.length; i++) {
        const values = lines[i].split(',');
        const lat = parseFloat(values[latIndex]);
        const lon = parseFloat(values[lonIndex]);
        if (!isNaN(lat) && !isNaN(lon)) {
          coordinates.push([lon, lat]);
        }
      }
      
      if (coordinates.length === 0) {
        console.log('No valid coordinates found in trajectory data.');
        alert('No valid trajectory data available for this drone.');
        return;
      }
      
      const trajectoryLine = new ol.geom.LineString(coordinates);
      trajectoryFeature = new ol.Feature({
        geometry: trajectoryLine,
        name: 'Drone Trajectory',
        description: `Past trajectory for ID: ${active_id}`
      });
      trajectoryFeature.setStyle(trajectory_style);
      markers.getSource().addFeature(trajectoryFeature);
      console.log('Trajectory drawn.');
      
    } catch (error) {
      console.error("Failed to fetch or draw trajectory:", error);
      alert(`Failed to load trajectory: ${error.message}`);
    }
  }
}
