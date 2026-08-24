import QtQuick
import QtQuick.Layouts
import QtQuick.Controls
import qt.theme 1.0

Dialog {
    id: root

    // --- 供外部绑定的属性 ---
    property int cableId: -1
    property alias nameText: nameInput.text
    property alias coreDiaText: coreDiaInput.text
    property alias coreMatText: coreMatInput.text
    property alias insThickText: insThickInput.text
    property alias insMatText: insMatInput.text
    property alias voltageGradeText: voltageGradeInput.text
    property alias systemFormatIndex: systemFormatCombo.currentIndex
    property alias notesText: notesInput.text

    // --- 信号 ---
    signal saveRequested(int id, string name, double coreDia, string coreMat, double insThick, string insMat, int voltageGrade, int systemFormat, string notes)

    title: cableId < 0 ? "新增电缆" : "修改电缆"
    modal: true
    anchors.centerIn: Overlay.overlay
    width: 480

    // --- 背景样式 ---
    background: Rectangle {
        color: Theme.controlBgColor
        border.color: Theme.highlightColor
        border.width: 2
        radius: 8
    }

    // --- 标题栏 ---
    header: Label {
        text: root.title
        color: Theme.titleColor
        font: Theme.subjectFont
        padding: 15
        horizontalAlignment: Text.AlignHCenter
    }

    // --- 内容区：使用 GridLayout 完美实现表单对齐 ---
    contentItem: GridLayout {
        columns: 2
        columnSpacing: 15
        rowSpacing: 10
        anchors.margins: 15

        // 1. 名称
        // 【修改点】更换了标签提示词
        Label { text: "名称（*唯一）"; color: Theme.textColor; font: Theme.defaultFont; Layout.alignment: Qt.AlignRight | Qt.AlignVCenter }
        Rectangle {
            Layout.fillWidth: true; Layout.preferredHeight: 36
            color: "transparent"; border.color: Theme.buttonBorderColor; border.width: 2; radius: 5
            TextInput { id: nameInput; anchors.fill: parent; anchors.margins: 5; verticalAlignment: Text.AlignVCenter; color: Theme.textColor; font: Theme.defaultFont; clip: true }
        }

        // 2. 电芯直径
        Label { text: "电芯直径(mm)"; color: Theme.textColor; font: Theme.defaultFont; Layout.alignment: Qt.AlignRight | Qt.AlignVCenter }
        Rectangle {
            Layout.fillWidth: true; Layout.preferredHeight: 36
            color: "transparent"; border.color: Theme.buttonBorderColor; border.width: 2; radius: 5
            TextInput {
                id: coreDiaInput; anchors.fill: parent; anchors.margins: 5; verticalAlignment: Text.AlignVCenter;
                color: Theme.textColor; font: Theme.defaultFont; clip: true
                validator: DoubleValidator { bottom: 0.0 }
            }
        }

        // 3. 电芯材料
        Label { text: "电芯材料"; color: Theme.textColor; font: Theme.defaultFont; Layout.alignment: Qt.AlignRight | Qt.AlignVCenter }
        Rectangle {
            Layout.fillWidth: true; Layout.preferredHeight: 36
            color: "transparent"; border.color: Theme.buttonBorderColor; border.width: 2; radius: 5
            TextInput { id: coreMatInput; anchors.fill: parent; anchors.margins: 5; verticalAlignment: Text.AlignVCenter; color: Theme.textColor; font: Theme.defaultFont; clip: true }
        }

        // 4. 绝缘厚度
        Label { text: "绝缘厚度(mm)"; color: Theme.textColor; font: Theme.defaultFont; Layout.alignment: Qt.AlignRight | Qt.AlignVCenter }
        Rectangle {
            Layout.fillWidth: true; Layout.preferredHeight: 36
            color: "transparent"; border.color: Theme.buttonBorderColor; border.width: 2; radius: 5
            TextInput {
                id: insThickInput; anchors.fill: parent; anchors.margins: 5; verticalAlignment: Text.AlignVCenter;
                color: Theme.textColor; font: Theme.defaultFont; clip: true
                validator: DoubleValidator { bottom: 0.0 }
            }
        }

        // 5. 绝缘材料
        Label { text: "绝缘材料"; color: Theme.textColor; font: Theme.defaultFont; Layout.alignment: Qt.AlignRight | Qt.AlignVCenter }
        Rectangle {
            Layout.fillWidth: true; Layout.preferredHeight: 36
            color: "transparent"; border.color: Theme.buttonBorderColor; border.width: 2; radius: 5
            TextInput { id: insMatInput; anchors.fill: parent; anchors.margins: 5; verticalAlignment: Text.AlignVCenter; color: Theme.textColor; font: Theme.defaultFont; clip: true }
        }

        // 6. 电压等级
        // 【修改点】更改单位为 kV，且由于有 IntValidator 天然只能输入整数，无法打出小数点
        Label { text: "电压等级(kV)"; color: Theme.textColor; font: Theme.defaultFont; Layout.alignment: Qt.AlignRight | Qt.AlignVCenter }
        Rectangle {
            Layout.fillWidth: true; Layout.preferredHeight: 36
            color: "transparent"; border.color: Theme.buttonBorderColor; border.width: 2; radius: 5
            TextInput {
                id: voltageGradeInput; anchors.fill: parent; anchors.margins: 5; verticalAlignment: Text.AlignVCenter;
                color: Theme.textColor; font: Theme.defaultFont; clip: true
                validator: IntValidator { bottom: 0 }
            }
        }

        // 7. 系统制式
        Label { text: "系统制式"; color: Theme.textColor; font: Theme.defaultFont; Layout.alignment: Qt.AlignRight | Qt.AlignVCenter }
        ComboBox {
            id: systemFormatCombo
            Layout.fillWidth: true; Layout.preferredHeight: 36
            font: Theme.defaultFont
            model: ["0: 直流", "1: 交流"]
        }

        // 8. 备注 (多行文本)
        Label { text: "备注"; color: Theme.textColor; font: Theme.defaultFont; Layout.alignment: Qt.AlignRight | Qt.AlignTop; Layout.topMargin: 5 }
        Rectangle {
            Layout.fillWidth: true; Layout.preferredHeight: 60
            color: "transparent"; border.color: Theme.buttonBorderColor; border.width: 2; radius: 5
            TextArea {
                id: notesInput; anchors.fill: parent; anchors.margins: 5;
                color: Theme.textColor; font: Theme.defaultFont; wrapMode: TextEdit.Wrap; background: null; clip: true
            }
        }
    }

    // --- 底部按钮区 ---
    footer: DialogButtonBox {
        alignment: Qt.AlignHCenter
        background: Rectangle { color: "transparent" }
        padding: 15

        Button {
            text: "保存确认"
            implicitWidth: 120; implicitHeight: 40
            background: Rectangle { color: Theme.statusOkColor; radius: 5 }
            contentItem: Text { text: parent.text; color: "white"; font: Theme.buttonFont; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
            onClicked: {
                if (nameInput.text.trim() === "") {
                    return;
                }
                let cDia = parseFloat(coreDiaInput.text); if(isNaN(cDia)) cDia = 0.0;
                let iThick = parseFloat(insThickInput.text); if(isNaN(iThick)) iThick = 0.0;
                // 用 parseInt 再次确保输出的是整型数值
                let vGrade = parseInt(voltageGradeInput.text); if(isNaN(vGrade)) vGrade = 0;

                root.saveRequested(root.cableId, nameInput.text.trim(), cDia, coreMatInput.text, iThick, insMatInput.text, vGrade, systemFormatCombo.currentIndex, notesInput.text);
                root.close();
            }
        }

        Button {
            text: "取消"
            implicitWidth: 120; implicitHeight: 40
            background: Rectangle { color: Theme.buttonHoverColor; radius: 5 }
            contentItem: Text { text: parent.text; color: Theme.textColor; font: Theme.buttonFont; horizontalAlignment: Text.AlignHCenter; verticalAlignment: Text.AlignVCenter }
            onClicked: root.close()
        }
    }
}
