import QtQuick 6.6
import QtLocation 6.6
import QtQuick.Controls 6.6


MapQuickItem {
    id: marker
    property variant text
    property variant tooltip_text : ""
    anchorPoint.x: image.width/2
    anchorPoint.y: image.height
    visible: true

    HoverHandler {
        id: hoverHandler
    }

    sourceItem: Image {
        id: image
        source: "../MapPackage/resources/green_marker.png"
        opacity: hoverHandler.hovered ? 0.6 : 1.0

        Text{
            id: number
            y: image.height/10
            width: image.width
            color: "white"
            font.bold: true
            font.pixelSize: 14
            horizontalAlignment: Text.AlignHCenter
            Component.onCompleted: {
                text = marker.text
            }
        }

    }

    ToolTip {
        parent: image.hoverHandler
        visible: hoverHandler.hovered
        text: tooltip_text
    }

}