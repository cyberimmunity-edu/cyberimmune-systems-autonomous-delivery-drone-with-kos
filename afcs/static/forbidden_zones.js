const TILES_URL = "https://mt1.google.com/vt/lyrs=y&x={x}&y={y}&z={z}";
const TILES_LOCAL_PATH = "static/resources/tiles";

ol.proj.useGeographic();
const place = [142.812588, 46.617637];

let availableTiles = [];

fetch('/tiles/index')
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

let vectorLayer;
let vectorSource;
let modify, snap;

let draw;

function isSelfIntersecting(coordinates) {
    function ccw(A, B, C) {
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0]);
    }

    function intersect(A, B, C, D) {
        return ccw(A, C, D) !== ccw(B, C, D) && ccw(A, B, C) !== ccw(A, B, D);
    }

    for (let i = 0; i < coordinates.length - 3; i++) {
        for (let j = i + 2; j < coordinates.length - 1; j++) {
            if (i === 0 && j === coordinates.length - 2) {
                continue;
            }
            if (intersect(coordinates[i], coordinates[i + 1], coordinates[j], coordinates[j + 1])) {
                return true;
            }
        }
    }
    return false;
}

let currentDrawnFeature = null;

function createDrawInteraction() {
    draw = new ol.interaction.Draw({
        source: vectorSource,
        type: 'Polygon',
    });

    draw.on('drawstart', function(event) {
        // Удаляем предыдущий незавершенный полигон, если он есть
        vectorSource.getFeatures().forEach(function(feature) {
            if (!feature.get('name')) {
                vectorSource.removeFeature(feature);
            }
        });
        currentDrawnFeature = null;
    });

    draw.on('drawend', function(event) {
        const feature = event.feature;
        const geometry = feature.getGeometry();
        const coordinates = geometry.getCoordinates()[0];
    
        if (isSelfIntersecting(coordinates)) {
            alert('Самопересекающиеся полигоны запрещены. Пожалуйста, нарисуйте полигон заново.');
            
            // Используем setTimeout для асинхронного удаления фичи
            setTimeout(() => {
                vectorSource.removeFeature(feature);
                vectorLayer.changed(); // Принудительно обновляем слой
                currentDrawnFeature = null;
            }, 0);
    
            // Перезапускаем инструмент рисования
            map.removeInteraction(draw);
            createDrawInteraction();
        } else {
            currentDrawnFeature = feature;
            map.removeInteraction(draw);
        }
    });

    map.addInteraction(draw);
}

async function createVectorLayer() {
    // Удаляем текущий vectorLayer, если он существует
    if (vectorLayer) {
        map.removeLayer(vectorLayer);
    }

    const token = document.getElementById('token').value;
    const response = await fetch(`/admin/get_forbidden_zones?token=${token}`);
    if (!response.ok) {
        console.error('Failed to fetch forbidden zones');
        return;
    }
    const geojsonObject = await response.json();

    vectorSource = new ol.source.Vector({
        features: new ol.format.GeoJSON().readFeatures(geojsonObject),
    });

    vectorSource.getFeatures().forEach(feature => {
        feature.set('description', `Запрещенная зона: ${feature.get('name')}`);
    });

    vectorLayer = new ol.layer.Vector({
        source: vectorSource,
        style: new ol.style.Style({
            fill: new ol.style.Fill({
                color: 'rgba(255, 0, 0, 0.2)',
            }),
            stroke: new ol.style.Stroke({
                color: '#ff0000',
                width: 2,
            }),
        }),
    });

    map.addLayer(vectorLayer);

    // Создаем взаимодействия после инициализации vectorSource
    if (draw) map.removeInteraction(draw);
    if (modify) map.removeInteraction(modify);
    if (snap) map.removeInteraction(snap);

    modify = new ol.interaction.Modify({ source: vectorSource });
    map.addInteraction(modify);

    snap = new ol.interaction.Snap({ source: vectorSource });
    map.addInteraction(snap);
}

const info = document.getElementById('info');

let currentFeature;
const displayFeatureInfo = function (pixel, target) {
    const feature = target.closest('.ol-control')
        ? undefined
        : map.forEachFeatureAtPixel(pixel, function (feature) {
            // Проверяем, что это полигон (зона), а не инструмент рисования
            if (feature.getGeometry().getType() === 'Polygon') {
            return feature;
            }
        });
    if (feature && feature.get('description')) {
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


let selectedFeature = null;

map.on('click', function (evt) {
    const feature = map.forEachFeatureAtPixel(evt.pixel, function (feature) {
        if (feature.getGeometry().getType() === 'Polygon') {
            return feature;
        }
    });

    if (feature) {
        selectedFeature = feature;
        const zoneName = feature.get('name');
        if (zoneName) {
            document.getElementById('zone-name').value = zoneName;
        }
    } else {
        selectedFeature = null;
        document.getElementById('zone-name').value = '';
    }
    displayFeatureInfo(evt.pixel, evt.originalEvent.target);
});

map.getTargetElement().addEventListener('pointerleave', function () {
  currentFeature = undefined;
  info.style.visibility = 'hidden';
});

createVectorLayer();

document.getElementById('save-zone').onclick = async function () {
    const zoneName = document.getElementById('zone-name').value;
    const token = document.getElementById('token').value;
    if (!zoneName) {
        alert('Введите имя зоны');
        return;
    }

    if (!currentDrawnFeature) {
        alert('Нарисуйте непересекающуюся зону на карте');
        return;
    }

    const coordinates = currentDrawnFeature.getGeometry().getCoordinates()[0];

    const response = await fetch('/admin/set_forbidden_zone', {
        method: 'POST',
        headers: {
            'Content-Type': 'application/json',
        },
        body: JSON.stringify({
            name: zoneName,
            geometry: coordinates,
            token: token,
        }),
    });

    if (response.ok) {
        alert('Зона сохранена');
        currentDrawnFeature.set('name', zoneName);
        currentDrawnFeature.set('description', `Запрещенная зона: ${zoneName}`);
        currentDrawnFeature = null;
        await createVectorLayer();
    } else {
        alert('Ошибка сохранения зоны');
    }
};

document.getElementById('start-drawing').onclick = function() {
    createDrawInteraction();
};

document.getElementById('delete-zone').onclick = async function () {
    if (!selectedFeature) {
        alert('Выберите зону для удаления');
        return;
    }

    const zoneName = selectedFeature.get('name');
    const token = document.getElementById('token').value;

    if (!zoneName) {
        alert('Выбранная зона не имеет имени');
        return;
    }

    const response = await fetch(`/admin/delete_forbidden_zone?name=${zoneName}&token=${token}`, {
        method: 'DELETE',
    });

    if (response.ok) {
        alert('Зона удалена');
        vectorSource.removeFeature(selectedFeature);
        selectedFeature = null;
        document.getElementById('zone-name').value = '';
    } else {
        alert('Ошибка удаления зоны');
    }
};

document.getElementById('export-zones').onclick = async function () {
    const token = document.getElementById('token').value;
    const response = await fetch(`/admin/export_forbidden_zones?token=${token}`);
    if (response.ok) {
        const blob = await response.blob();
        const url = window.URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.style.display = 'none';
        a.href = url;
        a.download = 'forbidden_zones.json';
        document.body.appendChild(a);
        a.click();
        window.URL.revokeObjectURL(url);
    } else {
        alert('Ошибка экспорта зон');
    }
};

document.getElementById('import-zones').onclick = function () {
    document.getElementById('import-zones-file').click();
};

document.getElementById('import-zones-file').onchange = async function (event) {
    const file = event.target.files[0];
    if (!file) {
        return;
    }

    const token = document.getElementById('token').value;
    const formData = new FormData();
    formData.append('file', file);
    formData.append('token', token);

    const response = await fetch('/admin/import_forbidden_zones', {
        method: 'POST',
        body: formData
    });

    if (response.ok) {
        alert('Зоны успешно импортированы');
        await createVectorLayer();
    } else {
        alert('Ошибка импорта зон');
    }
};