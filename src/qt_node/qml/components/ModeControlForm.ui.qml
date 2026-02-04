import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt.theme 1.0

Item {
    id: root

    implicitHeight: 50

    property string title: "title"
    property alias manualButton: manualButton
    property alias constCurrentButton: constCurrentButton
    property alias tempControlButton: tempControlButton
    property int buttonWidth: 120

    Item{
        id: lablePanel
        anchors{
            left: parent.left
            top: parent.top
            bottom: parent.bottom
        }
        width: 150
        Label {
            id: titleLabel
            anchors.centerIn: parent
            text: title
            font: Theme.largeLabelFont
            color: Theme.orange
            verticalAlignment: Text.AlignVCenter
        }
    }

    RowLayout {
        anchors{
            left: lablePanel.right
            right: parent.right
            top: parent.top
            bottom: parent.bottom
            leftMargin: 30
        }
        spacing: Theme.subSpacing


        Item {
            Layout.fillHeight: true
            Layout.preferredWidth: buttonWidth
            ToggleActionButton {
                id: manualButton
                labelText: "手 动"
                anchors.fill: parent
                anchors.margins: 5
            }
        }

        Item {
            Layout.fillHeight: true
            Layout.preferredWidth: buttonWidth
            ToggleActionButton {
                id: constCurrentButton
                labelText: "恒 流"
                anchors.fill: parent
                anchors.margins: 5
            }
        }

        Item {
            Layout.fillHeight: true
            Layout.preferredWidth: buttonWidth
            ToggleActionButton {
                id: tempControlButton
                labelText: "温 控"
                anchors.fill: parent
                anchors.margins: 5
            }
        }
    }
}
