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

    RowLayout {
        anchors{
            fill: parent
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
}
