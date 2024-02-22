import QtQuick 6.6
import QtQuick.Layouts 6.6
import QtQuick.Controls.Material 6.6

Popup {
    id: authPopup
    anchors.centerIn: parent
    contentWidth: 480
    property variant id: ''
    Material.background: '#FF0000'
    Material.foreground: '#000'

    ColumnLayout {
        anchors.fill: parent
        
        Text {
                id: label1
                text: qsTr("")
            }
    }

    function changeId(id) {
        label1.text = qsTr("id: " + id + ' авторизован.')
    }
}