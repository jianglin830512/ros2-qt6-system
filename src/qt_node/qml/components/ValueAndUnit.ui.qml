import QtQuick
import QtQuick.Layouts
import QtQuick.Controls

Rectangle {

    color: "transparent"

    property string title: "title"
    property string value: "0"
    property string unit: "unit"

    RowLayout{
        anchors{
            fill: parent
            margins: 5
        }
        Label {
            Layout.fillHeight: true
            Layout.fillWidth: true
            //horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            text: title
            color: Theme.titleColor
            font: Theme.labelFont
        }
        Label {
            Layout.fillHeight: true
            Layout.fillWidth: true
            //Layout.preferredWidth: 35
            //horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            text: value
            color: Theme.textColor
            font: Theme.labelFont
        }
        Label {
            Layout.fillHeight: true
            Layout.fillWidth: true
            Layout.preferredWidth: 25
            horizontalAlignment: Text.AlignRight
            verticalAlignment: Text.AlignVCenter
            text: unit
            color: Theme.titleColor
            font: Theme.labelFont
        }
    }
}
