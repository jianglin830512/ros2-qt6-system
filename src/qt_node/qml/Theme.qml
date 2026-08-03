import QtQuick

pragma Singleton

QtObject {
    // ==========================================
    // 1. 主配色 (工业冷灰与精密钢蓝)
    // ==========================================
    // 整体背景改用偏冷的浅工业灰，摒弃偏暖的白，增加仪器感
    readonly property color backgroundColor: "#E8ECEF"
    // 控件背景保持纯白，以确保数据可读性
    readonly property color controlBgColor: "#FFFFFF"

    // 主文本颜色：深碳灰，比纯黑对比度低，长时间注视不刺眼
    readonly property color textColor: "#2C3E50"
    // 标题及数值颜色：深邃的藏青色/海军蓝，显得非常专业、权威
    readonly property color titleColor: "#153E5C"
    // 高亮色：沉稳的科技蓝，取代了之前轻浮的亮蓝色
    readonly property color highlightColor: "#004A80"

    // 暖色点缀：使用沉稳的琥珀色/焦糖色，取代原本刺眼的亮橙色 -- 客户要求改成深蓝
    readonly property color orange: highlightColor //"#E65100"
    // 数据值显示颜色保持与标题一致的专业蓝
    readonly property color valueColor: titleColor

    readonly property color circuitCurrentValue: "red" // 4个实时电流显示的颜色，包括字体和外框

    // ==========================================
    // 2. 按钮与交互状态 (克制的渐变与高质感)
    // ==========================================
    // 选中/按下状态的渐变：采用低对比度的深蓝色过渡，消除“塑料光泽”，增加“金属氧化”质感
    readonly property color buttonSelectedGradientStart: "#005C99"
    readonly property color buttonSelectedGradientEnd: "#004375"
    readonly property color buttonSelectedTextColor: "#FFFFFF"
    readonly property color buttonDisabledTextColor: "#A0AAB5"
    readonly property color indicatorTextColor: "#FFFFFF"

    // 默认按钮状态：冷灰色调边框，悬停时带有极其轻微的冷蓝灰倾向
    readonly property color buttonBackgroundColor: "#FAFAFC"
    readonly property color buttonBorderColor: "#C9D1D9"
    readonly property color buttonHoverColor: "#E3EBF3"

    // 网格线与分割线：克制的冷灰
    readonly property color gridLineColor: "#D0D7DE"
    readonly property color gridLineTransparentColor: Qt.rgba(44, 62, 80, 0.08)
    readonly property color chartBgColor: "#FFFFFF"

    // ==========================================
    // 3. 状态与指示灯颜色 (降低明度，模拟真实LED质感)
    // ==========================================
    // 警报红：采用深砖红，更具警示性且不刺眼
    readonly property color localRed: "#C62828"
    // 正常绿：采用工业祖母绿
    readonly property color remoteGreen: "#00897B"
    readonly property color estopGray: "#7F8C8D"
    // 急停红：极致危险的深红
    readonly property color estopActiveRed: "#B71C1C"

    readonly property color indicatorBorderColor: "#B0BEC5"
    readonly property color indicatorOffColor: "#E0E6ED"

    // 运行状态色：更加成熟的色彩
    readonly property color statusOkColor: "#00897B"     // 工业绿
    readonly property color statusDisabledColor: "#95A5A6"// 质感灰
    readonly property color statusHeatColor: "#D32F2F"   // 加热红
    readonly property color statusCoolColor: "#0288D1"   // 冷却蓝 (原为绿色，工控中冷却通常用蓝色更直观)
    readonly property color statusStandbyColor: "#F39C12"// 待机黄
    readonly property color errorColor: "#C0392B"

    // ==========================================
    // 4. 遮罩、弹窗与警告层
    // ==========================================
    // 遮罩层：带有一点点冷蓝色的半透明白，更显高级
    readonly property color blockerOverlayColor: "#D9F0F4F8"
    readonly property color blockerTextColor: "#34495E"
    readonly property color popupBackgroundColor: "#FFFFFF"
    // 危险操作按钮：深红色
    readonly property color buttonDangerColor: "#D32F2F"
    readonly property color buttonDangerPressedColor: "#9A0007"

    // ==========================================
    // 5. 图表与表格 (高对比度数据呈现)
    // ==========================================
    readonly property color axisLabelColor: "#546E7A"
    readonly property color chartBaseColor: "#00796B"    // 基础柱状图：深青色
    readonly property color chartWarnColor: "#F57C00"    // 警告柱状图：深橙色
    readonly property color chartDangerColor: "#C62828"  // 危险柱状图：深红色
    readonly property color chartVoltageColor: "#D35400" // 电压曲线：锈橙色，与电流区分明显且不刺眼

    readonly property color arrowUpColor: "#C62828"
    readonly property color arrowDownColor: "#00897B"

    readonly property color placeholderTextColor: Qt.rgba(44, 62, 80, 0.4)
    readonly property color inputTextColor: titleColor
    readonly property color checkboxCheckedColor: highlightColor
    readonly property color checkboxUncheckedColor: "#7F8C8D"

    // 表格：更加微妙的交替行颜色
    readonly property color tableHoverColor: "#EBF0F5"
    readonly property color tableAlternateColor: "#F8F9FA"

    // ==========================================
    // 6. 字体定义 (保持 YaHei，但排版严谨)
    // ==========================================
    readonly property font titleFont: Qt.font({ family: "Microsoft YaHei", pointSize: 28, bold: true })
    readonly property font subTitleFont: Qt.font({ family: "Microsoft YaHei", pointSize: 14 })
    readonly property font subjectFont: Qt.font({ family: "Microsoft YaHei", pointSize: 16 })
    readonly property font smallLabelFont: Qt.font({ family: "Microsoft YaHei", pointSize: 12 })
    readonly property font labelFont: Qt.font({ family: "Microsoft YaHei", pointSize: 14 })
    readonly property font largeLabelFont: Qt.font({ family: "Microsoft YaHei", pointSize: 18 })
    readonly property font defaultFont: Qt.font({ family: "Microsoft YaHei", pointSize: 16 })
    readonly property font buttonFont: Qt.font({ family: "Microsoft YaHei", pointSize: 16 })
    readonly property font indicatorFont: Qt.font({ family: "Microsoft YaHei", pointSize: 16, bold: true })

    // ==========================================
    // 7. 空隙与尺寸
    // ==========================================
    readonly property int mainSpacing: 20
    readonly property int subSpacing: 15
}
