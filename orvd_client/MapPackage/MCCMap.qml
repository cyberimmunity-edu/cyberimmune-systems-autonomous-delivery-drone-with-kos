import QtQuick 6.6
import QtLocation 6.6
import QtPositioning 6.6
import G.ConnectionsHandler 1.0


Map {
    id: map
    bearing: 0
    property variant markers: []
    plugin: Plugin {
        id: mapPlugin
        name: "osm"
        property string projectname: "MCC"

        PluginParameter {
            name: "osm.mapping.providersrepository.address"
            value: Qt.resolvedUrl('../MapPackage/providers')
        }

        PluginParameter {
            name: "osm.mapping.offline.directory"
            value: './MapPackage/cache'
        }
    }
    center: QtPositioning.coordinate(46.939162, 142.756177)
    zoomLevel: 16
    property geoCoordinate startCentroid

    MapPolyline {
        id: pathLine
        line.width: 2
        line.color: 'black'
        path: []
    }

    Connections {
        target: ConnectionsHandler
        function onShowMission(mission_list) {
            destroy_markers()

            for (let idx = 0; idx < mission_list.length; ++idx) {
                if (mission_list[idx][0] == 'H') {
                    var alt = mission_list[idx][3]
                    var component = Qt.createComponent("../MapPackage/Marker.qml")
                    var ttext = 'Высота: ' + alt
                    markers.push(component.createObject(map, {text: 'H', tooltip_text: ttext, z: map.z+1}))
                    markers[markers.length - 1].coordinate = QtPositioning.coordinate(parseFloat(mission_list[idx][1]), mission_list[idx][2])
                    map.center = markers[markers.length - 1].coordinate
                    map.addMapItem(markers[markers.length - 1])
                    pathLine.path.push(markers[markers.length - 1].coordinate)
                }
                else if (mission_list[idx][0] == 'W') {
                    var component = 0
                    var ttext = ""
                    var alt = mission_list[idx][4]
                    if (idx < mission_list.length - 1 && mission_list[idx+1][0] == 'S') {
                        component = Qt.createComponent("../MapPackage/RedMarker.qml")
                        ttext = 'Сброс груза\nВысота: ' + alt
                        
                    }
                    else {
                        component = Qt.createComponent("../MapPackage/Marker.qml")
                        ttext = 'Высота: ' + alt
                    }
                    markers.push(component.createObject(map, {text: 'W', tooltip_text: ttext, z: map.z+1}))
                    markers[markers.length - 1].coordinate = QtPositioning.coordinate(parseFloat(mission_list[idx][2]), mission_list[idx][3])
                    map.addMapItem(markers[markers.length - 1])
                    pathLine.path.push(markers[markers.length - 1].coordinate)
                }                
            }
        }
        function onNoMission() {
            destroy_markers()
        }
    }

    function destroy_markers() {
        for (let idx = 0; idx < markers.length; ++idx) {
            map.removeMapItem(markers[idx])
            markers[idx].destroy()
        }
        markers = []
        pathLine.path = []
    }

    PinchHandler {
        id: pinch
        target: null
        onActiveChanged: if (active) {
            map.startCentroid = map.toCoordinate(pinch.centroid.position, false)
        }
        onScaleChanged: (delta) => {
            map.zoomLevel += Math.log2(delta)
            map.alignCoordinateToPoint(map.startCentroid, pinch.centroid.position)
        }
        onRotationChanged: (delta) => {
            map.bearing -= delta
            map.alignCoordinateToPoint(map.startCentroid, pinch.centroid.position)
        }
        grabPermissions: PointerHandler.TakeOverForbidden
    }
    WheelHandler {
        id: wheel
        acceptedDevices: Qt.platform.pluginName === "cocoa" || Qt.platform.pluginName === "wayland"
                            ? PointerDevice.Mouse | PointerDevice.TouchPad
                            : PointerDevice.Mouse
        rotationScale: 1/120
        property: "zoomLevel"
    }
    DragHandler {
            id: drag
            target: null
            onTranslationChanged: (delta) => { 
                map.pan(-delta.x, -delta.y)
            }
        }
    Shortcut {
        enabled: map.zoomLevel < map.maximumZoomLevel
        sequence: StandardKey.ZoomIn
        onActivated: map.zoomLevel = Math.round(map.zoomLevel + 1)
    }
    Shortcut {
        enabled: map.zoomLevel > map.minimumZoomLevel
        sequence: StandardKey.ZoomOut
        onActivated: map.zoomLevel = Math.round(map.zoomLevel - 1)
    }

}