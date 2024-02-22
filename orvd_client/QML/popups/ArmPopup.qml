import QtQuick 6.6
import QtQuick.Controls 6.6
import QtQuick.Layouts 6.6
import QtQuick.Controls.Material 6.6
import G.ConnectionsHandler 1.0

ApplicationWindow {
    id: armPopup
    title: qsTr('Arm')

    width: 240
    height: 120

    property variant id: ''
    property variant address: ''
    property variant is_decision_maked: false

    ColumnLayout {

        Text {
            id: label1
            text: qsTr("id = " + id)
        }

        Button {
            text: qsTr("&Arm")
            onClicked: {
                armDecision(0)
                armPopup.close()
                }
            }
        Button {
            text: qsTr("&Dont arm")
            onClicked: {
                armDecision(1)
                armPopup.close()
                }
            }
        }
    
    onClosing: {
        if(armPopup.is_decision_maked == false) {
            armDecision(1)
        }
    }


    function armDecision(flag) {
        armPopup.is_decision_maked = true
        if(flag == 0) {
            print("arm " + id)
        }
        else {
            print("not arm " + id)
        }
        ConnectionsHandler.arm_decision(id, flag)
    }
}