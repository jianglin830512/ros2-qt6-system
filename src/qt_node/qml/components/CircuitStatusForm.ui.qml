import QtQuick
import QtQuick.Layouts
import qt.theme 1.0

Rectangle{
    color: "transparent"
    border.color: Theme.highlightColor
    border.width: 3
    radius: 8

    // --- Aliases for logic file ---
    property alias testLoop: testLoop
    property alias refLoop: refLoop
    property alias modeControl: modeControl

    RowLayout {
        anchors{
            top: parent.top
            left: parent.left
            right: parent.right
            bottom: modeControl.top
            margins: 10
        }
        spacing: Theme.subSpacing
        LoopStatus {
            id: testLoop
        }
        LoopStatus {
            id: refLoop
        }
    }

    ModeControl {
        id: modeControl
        anchors{
            left: parent.left
            right: parent.right
            bottom: parent.bottom
            bottomMargin: 10
        }
        // height: 60
    }
}
