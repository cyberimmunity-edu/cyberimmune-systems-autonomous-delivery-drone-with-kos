import QtQuick 6.6
import QtQuick.Controls 6.6
import QtQuick.Layouts 6.6
import QtQuick.Controls.Material 6.6
import G.ConnectionsHandler 1.0
import './popups'
import '../MapPackage'

ApplicationWindow {
    id: window
    width: 1920
    height: 1080
    title: qsTr('ORVD')
    visible: true

    property variant active_ids: []
    property variant wait_ids: []
    
    SplitView {
        id: splitView
        anchors.fill: parent
        orientation: Qt.Horizontal
        Item {
            SplitView.minimumWidth: window.width / 6
            width: window.width / 6
            Layout.fillHeight: true
            Layout.fillWidth: true

            ColumnLayout {
                id: info_panel
                spacing: 0
                width: parent.width
                Layout.alignment: Qt.AlignHCenter
                GridLayout {
                    Layout.alignment: Qt.AlignHCenter
                    Layout.topMargin: 12
                    columns: 2
                    Text {
                        id: label1
                        text: qsTr("ID:")
                    }
                    ComboBox {
                        id: chooseBox
                        model: window.active_ids
                        delegate: ItemDelegate {
                            id: box_delegate
                            width: chooseBox.width
                            required property string modelData

                            contentItem: Rectangle {
                                id: delegate_item
                                anchors.fill: parent
                                opacity: hovered ? 0.7 : 1.0
                                RowLayout {
                                    id: combo_layout_1
                                    anchors.fill: parent
                                    Text {
                                        Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                                        text: modelData
                                        font.pointSize: 10
                                    }
                                    Image {
                                        id: circle1
                                        Layout.alignment: Qt.AlignRight | Qt.AlignVCenter
                                        Layout.rightMargin: 10
                                        scale: 0.7
                                        fillMode: Image.PreserveAspectFit; clip: true
                                        source: wait_ids.includes(modelData) ? "../resources/red_circle.png" : "../resources/blank.png"
                                    }
                                }
                            }
                        }

                        contentItem: Rectangle {
                            anchors.fill: parent
                            RowLayout {
                                id: combo_layout_2
                                anchors.fill: parent
                                Text {
                                    Layout.alignment: Qt.AlignHCenter | Qt.AlignVCenter
                                    text: active_ids[chooseBox.currentIndex]
                                    font.pointSize: 12
                                }
                                Image {
                                    id: circle2
                                    Layout.alignment: Qt.AlignRight
                                    scale: 0.7
                                    fillMode: Image.PreserveAspectFit; clip: true
                                    source: "../resources/blank.png"
                                }
                            }
                        }
                        onCurrentIndexChanged: {
                            ConnectionsHandler.get_state(active_ids[chooseBox.currentIndex])
                            ConnectionsHandler.get_mission(active_ids[chooseBox.currentIndex])
                        }
                    }

                    Text {
                        id: label2
                        text: qsTr("Статус:")
                    }

                    Text {
                        id: info2
                        text: qsTr("Нет")
                    }

                    Text {
                        id: label3
                        text: qsTr("Ожидают:")
                    }

                    Text {
                        id: info3
                        text: qsTr("0")
                    }
                
                }

                Button {
                    id: armButton
                    Layout.topMargin: 30
                    Layout.alignment: Qt.AlignHCenter
                    Layout.preferredWidth: parent.width * 0.8
                    text: qsTr("&Arm")
                    Material.background: '#009982'
                    enabled: false
                    onClicked: {
                        var active_id = chooseBox.currentText
                        print("arm " + active_id)
                        ConnectionsHandler.arm_decision(active_id, 0)
                        postArmHandler(active_id)
                    }
                }

                Button {
                    id: disarmButton
                    Layout.alignment: Qt.AlignHCenter
                    text: qsTr("&Disarm")
                    Material.background: '#009982'
                    Layout.preferredWidth: parent.width * 0.8
                    enabled: false
                    onClicked: {
                        var active_id = chooseBox.currentText
                        print("disarm " + active_id)
                        ConnectionsHandler.arm_decision(active_id, 1)
                        postArmHandler(active_id)
                    }
                }

                CheckBox {
                    id: missionBox
                    text: qsTr("Подтверждение миссии")
                    checkState: Qt.Unchecked
                    tristate: false
                    Layout.alignment: Qt.AlignHCenter
                    onCheckedChanged: {
                        var active_id = active_ids[chooseBox.currentIndex]
                        if(checkState == Qt.Unchecked) {
                            ConnectionsHandler.change_mission_acceptance(active_id,1)
                        }
                        else {
                            ConnectionsHandler.change_mission_acceptance(active_id,0)
                        }
                    }
                }
                
                Button {
                    Layout.alignment: Qt.AlignHCenter
                    text: qsTr("&Force disarm")
                    Material.background: '#009982'
                    Layout.preferredWidth: parent.width * 0.8
                    onClicked: {
                        var active_id = chooseBox.currentText
                        print("force disarm " + active_id)
                        ConnectionsHandler.force_disarm(active_id)
                    }
                }

                Button {
                    Layout.alignment: Qt.AlignHCenter
                    text: qsTr("&Force disarm all")
                    Material.background: '#009982'
                    Layout.preferredWidth: parent.width * 0.8
                    onClicked: {
                        print("force disarm all")
                        ConnectionsHandler.force_disarm_all()
                    }
                }
            }
        }

        ColumnLayout {
            id: map_panel
            spacing: 0
            // Image {
            //     id: k_logo
            //     Layout.alignment: Qt.AlignHCenter
            //     scale: 0.7
            //     Layout.preferredHeight: 80
            //     fillMode: Image.PreserveAspectFit; clip: true
            //     source: "../resources/Kaspersky_logo.svg"

            // }
            MCCMap {
                id: map
                Layout.fillHeight: true
                Layout.fillWidth: true
            }
        
        }

    }

    function postArmHandler(id) {
        info3.text = parseInt(info3.text) - 1
        wait_ids = wait_ids.filter(a_id => a_id != id)
        refresh_ids()
    }

    function refresh_ids() {
        var currentIndex = chooseBox.currentIndex
        window.active_ids.push('')
        chooseBox.model = window.active_ids
        window.active_ids.pop()
        chooseBox.model = window.active_ids
        chooseBox.currentIndex = currentIndex
    }

    Connections {
        target: ConnectionsHandler
        function onArmRequest(id, address) {
            info3.text = parseInt(info3.text) + 1
            wait_ids.push(id)
            refresh_ids()
        }
        
        function onShowId(id) {
            if (active_ids.includes(id) == false) {
                active_ids.push(id)
            }
            var currentText = chooseBox.currentText
            chooseBox.model = window.active_ids
            if (currentText != '') {
                var currentIndex = active_ids.indexOf(currentText)
                chooseBox.currentIndex = currentIndex
            }
            authPopup.changeId(id)
            authPopup.open()
        }

        function onShowState(state, id) {
            if (active_ids[chooseBox.currentIndex] == id) {
                info2.text = state
                if (state == 'Ожидает') {
                    armButton.enabled = true
                    disarmButton.enabled = true
                }
                else {
                    armButton.enabled = false
                    disarmButton.enabled = false
                }
            }
        }

        function onShowMissionAcceptance(id, mission_accept) {
            if (mission_accept == 0) {
                missionBox.checkState = Qt.Checked
            }
            else {
                missionBox.checkState = Qt.Unchecked
            }
        }
        
        function onMissionGained(id) {
            if (active_ids[chooseBox.currentIndex] == id) {
                ConnectionsHandler.get_mission(active_ids[chooseBox.currentIndex])
            }
        }
    }

    AuthPopup { id: authPopup }
    
}