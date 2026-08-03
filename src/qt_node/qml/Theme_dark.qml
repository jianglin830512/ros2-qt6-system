import QtQuick

pragma Singleton

QtObject {
    // -- 主配色 --
    readonly property color backgroundColor: "#000000" 
    readonly property color controlBgColor: "transparent" 
    readonly property color textColor: "#B7DDE8"       
    readonly property color titleColor: "#31859C"      
    readonly property color highlightColor: "#215968"
    readonly property color orange: "#EA700D"
    readonly property color valueColor: "#31859C" // 数值颜色 (同 titleColor)

    readonly property color buttonSelectedGradientStart: highlightColor
    readonly property color buttonSelectedGradientEnd: titleColor
    readonly property color buttonSelectedTextColor: "#FFFFFF" 
    readonly property color buttonDisabledTextColor: "#BFBFBF" 
    readonly property color indicatorTextColor: "white"

    // -- 控件颜色 --
    readonly property color buttonBackgroundColor: "transparent"
    readonly property color buttonBorderColor: highlightColor
    readonly property color buttonHoverColor: "#4000BFFF" 
    readonly property color gridLineColor: buttonHoverColor
    readonly property color gridLineTransparentColor: Qt.rgba(gridLineColor.r, gridLineColor.g, gridLineColor.b, 0.3)
    readonly property color chartBgColor: "#2E3136"
    
    // -- 状态与指示灯颜色 (剥离出来的硬编码) --
    readonly property color localRed: "#FF3333"    
    readonly property color remoteGreen: "#33FF33"   
    readonly property color estopGray: "#444444"    
    readonly property color estopActiveRed: "#FF0000" 
    readonly property color indicatorBorderColor: "#202020"
    readonly property color indicatorOffColor: "#333333"
    readonly property color statusOkColor: "lime"
    readonly property color statusDisabledColor: "#888888"
    readonly property color statusHeatColor: "red"
    readonly property color statusCoolColor: "green"
    readonly property color statusStandbyColor: "orange"
    readonly property color errorColor: "red"

    // -- 遮罩与弹窗 --
    readonly property color blockerOverlayColor: "#AA000000"
    readonly property color blockerTextColor: "#AAAAAA"
    readonly property color popupBackgroundColor: "#222222"
    readonly property color buttonDangerColor: "#AA2222"
    readonly property color buttonDangerPressedColor: "#661111"

    // -- 图表与表格 --
    readonly property color axisLabelColor: "#A0B0C0"
    readonly property color chartBaseColor: "#008080"
    readonly property color chartWarnColor: "orange"
    readonly property color chartDangerColor: "red"
    readonly property color chartVoltageColor: "#E0E000"
    readonly property color arrowUpColor: "red"
    readonly property color arrowDownColor: "green"
    readonly property color placeholderTextColor: Qt.rgba(1, 1, 1, 0.3)
    readonly property color inputTextColor: "white"
    readonly property color checkboxCheckedColor: "white"
    readonly property color checkboxUncheckedColor: "#AAAAAA"
    readonly property color tableHoverColor: Qt.rgba(highlightColor.r, highlightColor.g, highlightColor.b, 0.5)
    readonly property color tableAlternateColor: "#1AFFFFFF"

    // -- 字体定义 (保持不变) --
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