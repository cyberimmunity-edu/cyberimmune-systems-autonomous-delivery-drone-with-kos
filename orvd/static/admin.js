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

ol.proj.useGeographic()
const place = [142.812588, 46.617637];

let ids = [];
let active_id = null;
let current_mission = null;
let uav = null;
let access_token = params.token;
var current_state = null;

let uav_style = new ol.style.Style({
  image: new ol.style.Icon({
    anchor: [0.5, 0.5],
    src: 'static/resources/quad_marker.png'
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

let polyline_style = new ol.style.Style({
  stroke: new ol.style.Stroke({
    color: [0, 0, 0, 255],
    width: 2
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

const vectorLayer = new ol.layer.Vector({
  source: new ol.source.Vector({
    url: '/static/resources/area.kml',
    format: new ol.format.KML()
  })
});

const map = new ol.Map({
target: 'map',
layers: [
  tileLayer, vectorLayer
],
view: new ol.View({
  center: place,
  zoom: 15,
}),
});

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
      info.innerText = feature.get('description');
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
      if(feature.getId() != 'uav') {
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
  change_active_id(parseInt(id_select.options[id_select.selectedIndex].text));
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

async function get_mission() {
  let mission_resp = await fetch("admin/get_mission?id=" + active_id + "&token=" + access_token);
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
          map.getView().setCenter([lon, lat]);
      }
      else if (mission_list[idx][0] == 'W') {
          var lat = parseFloat(mission_list[idx][2]);
          var lon = parseFloat(mission_list[idx][3]);
          var alt = mission_list[idx][4];
          if (idx < mission_list.length - 1 && mission_list[idx+1][0] == 'S') {
              add_marker(lat, lon, alt, 'servo');              
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

async function get_telemetry() {
  let telemetry_resp = await fetch("admin/get_telemetry?id=" + active_id + "&token=" + access_token);
  let telemetry_text = await telemetry_resp.text();
  if (telemetry_text != '$-1') {
    let telemetry_list = telemetry_text.split('&');
    let lat = parseFloat(telemetry_list[0]);
    let lon = parseFloat(telemetry_list[1]);
    let alt = parseFloat(telemetry_list[2]);
    let azimuth = parseFloat(telemetry_list[3]);
    let dop = parseFloat(telemetry_list[4]);
    let sats = parseInt(telemetry_list[5]);
    document.getElementById("dop").innerHTML="DOP: " + dop;
    document.getElementById("sats").innerHTML="SATS: " + sats;
    if (uav == null) {
      add_marker(lat, lon, alt, 'uav');
    } else {
      uav.getGeometry().setCoordinates([lon, lat]);
      uav.set('description', 'Высота: ' + alt);
    }
    map.getView().setCenter([lon, lat]);
  }
}

async function get_mission_state() {
  let mission_state_resp = await fetch("admin/get_mission_state?id=" + active_id + "&token=" + access_token);
  let mission_state_text = await mission_state_resp.text();
  if (mission_state_text == '0') {
    document.getElementById('mission_checkbox').checked = true;
  } else {
    document.getElementById('mission_checkbox').checked = false;
  }
}

async function change_active_id(new_id) {
  active_id = new_id;
  current_mission = null;
  clear_markers()
  status_change();
  get_mission();
  get_telemetry();
  get_mission_state();
}


async function get_ids() {
  let ids_resp = await fetch("admin/get_id_list" + "?token=" + access_token);
  let ids_text = await ids_resp.text();
  let new_ids = JSON.parse(ids_text);
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


setInterval(async function(){
  get_ids();
  if(active_id != null) {
    status_change();
    waiters_change();
    get_mission();
    get_telemetry();
    get_mission_state();
  }
 }, 1000);
