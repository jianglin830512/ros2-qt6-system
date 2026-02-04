import QtQuick

// 继承自纯UI定义
ActionButtonForm {
    id: control

    // --- 信号 ---
    signal sendCommand()       // 按住时连续发射的信号
    signal releasedCommand()   // 弹起时发射一次的信号

    // --- 内部逻辑 ---
    // 使用 Timer 来实现连续发送
    Timer {
        id: continuousTimer
        interval: 50 // 50毫秒发送一次
        repeat: true
        onTriggered: {
            control.sendCommand()
        }
    }

    // 2. 处理按下状态的变化
    // 不能使用 onClicked，因为它只在弹起时触发一次
    onPressedChanged: {
        // 如果按钮被按下了
        if (pressed) {
            // 只有当指示灯未亮时，才响应操作
            if (!indicatorOn) {
                // 立即发送一次指令
                sendCommand()
                // 启动定时器，开始连续发送
                continuousTimer.start()
            }
        }
        // 如果按钮被弹起了
        else {
            // 无论之前是否响应，都确保定时器停止
            if (continuousTimer.running) {
                continuousTimer.stop()
                // 并且在下降沿（弹起时）发送一次性指令
                releasedCommand()
            }
        }
    }
}
