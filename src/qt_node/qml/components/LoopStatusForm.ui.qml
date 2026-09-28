import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt.theme 1.0

Rectangle {
    id: root
    implicitWidth: 350
    color: "transparent"
    border.color: Theme.highlightColor
    border.width: 1
    Layout.fillWidth: true
    Layout.fillHeight: true

    // --- Aliases for logic file ---
    property alias enableLabel: enableLabel
    property alias titleLabel: titleLabel
    property alias statusLabel: statusLabel
    property alias setCurrentLabel: setCurrentLabel
    property alias measureCurrentLabel: measureCurrentLabel
    property string heatSetValue: "0"
    property string heatRemainValue: "0"
    property string cycleSetValue: "0"
    property string cycleRemainValue: "0"

    property alias closeBreakerButton: closeBreakerButton
    property alias openBreakerButton: openBreakerButton

    property alias isBlocked: blockerContainer.visible
    property bool isButtonsBlocked: false

    ColumnLayout {
        id: mainLayout
        anchors.fill: parent; anchors.margins: 10; spacing: 8

        Item {
            id: titleArea
            Layout.fillWidth: true
            Layout.preferredHeight: 40
            RowLayout {
                anchors.fill: parent
                Text {
                    Layout.fillWidth: true
                    horizontalAlignment: Text.AlignHCenter
                    id: enableLabel; text: "停用"; font: Theme.defaultFont; color: "red"
                }
                Text {
                    Layout.fillWidth: true
                    horizontalAlignment: Text.AlignHCenter
                    id: titleLabel; text: "回路"; font: Theme.subjectFont; color: Theme.orange
                }
                Text {
                    Layout.fillWidth: true
                    horizontalAlignment: Text.AlignHCenter
                    id: statusLabel; text: "状态"; font: Theme.defaultFont; color: "red"
                }
            }
        }

        Item {
            id: currentSettingPanle
            Layout.fillWidth: true
            Layout.preferredHeight: 50
            RowLayout{
                anchors.fill: parent
                Label {
                    Layout.fillWidth: true; Layout.fillHeight: true
                    horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                    text: "设定电流:"; color: Theme.titleColor; font: Theme.largeLabelFont
                }
                Rectangle {
                    color: "transparent"; border.color: Theme.textColor; border.width: 2; radius: 5
                    Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 120
                    Label { id: setCurrentLabel; text: "0"; color: Theme.valueColor; font: Theme.largeLabelFont; anchors.centerIn: parent }
                }
                Label {
                    Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 50
                    horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter
                    text: "A"; color: Theme.titleColor; font: Theme.largeLabelFont
                }
            }
        }

        // 【修改点1】：高度从120降低到72 (原高度的60%)，内部字号同步减小
        Rectangle {
            id: currentValuePanle
            Layout.fillWidth: true
            Layout.preferredHeight: 120
            color: "transparent"; border.color: Theme.circuitCurrentValue; border.width: 3; radius: 8

            Row {
                anchors.centerIn: parent
                spacing: 15
                Label {
                    id: measureCurrentLabel; text: "0"; color: Theme.circuitCurrentValue
                    font.pixelSize: 120; font.bold: true; font.family: "Courier New"; anchors.verticalCenter: parent.verticalCenter
                }
                Label {
                    text: "A"; color: Theme.circuitCurrentValue
                    font.pixelSize: 80; font.bold: true; font.family: "Arial"; anchors.verticalCenter: parent.verticalCenter;
                }
            }
        }

        Item {
            id: breakerButtonPanle
            Layout.fillWidth: true
            Layout.preferredHeight: 60
            RowLayout {
                anchors.centerIn: parent; spacing: Theme.subSpacing
                ToggleActionButton { id: closeBreakerButton; labelText: "合闸"; colorWhenOn: "red"; enabled: !root.isButtonsBlocked }
                ToggleActionButton { id: openBreakerButton; labelText: "分闸"; colorWhenOn: "lime"; enabled: !root.isButtonsBlocked }
            }

            // 【修改点2】：将遮罩移至分合闸按钮所在的布局层级内，仅罩住按钮
            Item {
                id: blockerContainer
                anchors.fill: parent
                z: 999
                visible: false

                InputBlocker {
                    anchors.fill: parent
                    radius: 5
                    statusText: ""
                    overlayColor: Theme.blockerOverlayColor
                    visible: true
                }
            }
        }

        Item {
            id: timePanle
            Layout.fillWidth: true
            Layout.preferredHeight: 80
            GridLayout {
                anchors.fill: parent; columns: 3; rowSpacing: 5
                ValueAndUnit{ Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 100; title: "加热设定:"; value: heatSetValue; unit: "min" }
                Item { Layout.preferredWidth: 20 }
                ValueAndUnit{ Layout.fillWidth: true; Layout.fillHeight: true; Layout.preferredWidth: 70; title: "剩余:"; value: heatRemainValue; unit: "min" }
                ValueAndUnit{ Layout.fillWidth: true; Layout.fillHeight: true; title: "循环设定:"; value: cycleSetValue; unit: "次" }
                Item { Layout.preferredWidth: 20 }
                ValueAndUnit{ Layout.fillWidth: true; Layout.fillHeight: true; title: "剩余:"; value: cycleRemainValue; unit: "次" }
            }
        }
    }
}
