import QtQuick

pragma Singleton

QtObject {
    // -- 主配色 --
    readonly property color backgroundColor: "#000000" // 背景色: 纯黑
    readonly property color controlBgColor: "transparent" // 控件背景色: 透明
    readonly property color textColor: "#B7DDE8"       // 主文本颜色: 亮灰蓝色
    readonly property color titleColor: "#31859C"      // 新的深青蓝色
    readonly property color highlightColor: "#215968"
    readonly property color orange: "#EA700D"

    readonly property color buttonSelectedGradientStart: highlightColor
    readonly property color buttonSelectedGradientEnd: titleColor
    readonly property color buttonSelectedTextColor: "#FFFFFF" // 纯白色
    readonly property color buttonDisabledTextColor: "#BFBFBF" // 浅灰色
    readonly property color indicatorTextColor: "white"

    // -- 控件颜色 --
    readonly property color buttonBackgroundColor: "transparent"
    readonly property color buttonBorderColor: highlightColor
    readonly property color buttonHoverColor: "#4000BFFF" // 鼠标悬停时带透明度的背景色
    readonly property color gridLineColor: buttonHoverColor
    readonly property color chartBgColor: "#2E3136"
    readonly property color localRed: "#FF3333"    // 本地模式红
    readonly property color remoteGreen: "#33FF33"   // 远方模式绿
    readonly property color estopGray: "#444444"    // 急停常态灰
    readonly property color estopActiveRed: "#FF0000" // 急停触发红

    // -- 字体定义 --
    readonly property font titleFont: Qt.font({ family: "Microsoft YaHei", pointSize: 28, bold: true })
    readonly property font subTitleFont: Qt.font({ family: "Microsoft YaHei", pointSize: 14 })
    readonly property font subjectFont: Qt.font({ family: "Microsoft YaHei", pointSize: 16 })
    readonly property font smallLabelFont: Qt.font({ family: "Microsoft YaHei", pointSize: 12 })
    readonly property font labelFont: Qt.font({ family: "Microsoft YaHei", pointSize: 14 })
    readonly property font largeLabelFont: Qt.font({ family: "Microsoft YaHei", pointSize: 18 })
    readonly property font defaultFont: Qt.font({ family: "Microsoft YaHei", pointSize: 16 })
    readonly property font buttonFont: Qt.font({ family: "Microsoft YaHei", pointSize: 16 })
    readonly property font indicatorFont: Qt.font({ family: "Microsoft YaHei", pointSize: 16, bold: true })

    // -- 空隙定义 --
    readonly property int mainSpacing: 20
    readonly property int subSpacing: 15
}
