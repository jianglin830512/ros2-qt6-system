import QtQuick

// 继承自纯UI定义
ActionButtonForm {
    id: control

    // --- 信号 ---
    // 当按钮被点击且满足条件时，发射此信号
    signal sendCommand()

    // Control 基类提供了 onClicked 处理器
    onClicked: {
        // 只有当指示灯未亮时，才发送指令
        if (!indicatorOn) {
            sendCommand()
        }
    }
}
