#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QThread>



#include "include/qt_node/qt_ros_node.hpp" // 引入我们的节点头文件
#include "qt_node/ros_proxy.hpp"
//#include "rclcpp/rclcpp.hpp" // 在节点头文件里面包含了

void _qmlMessageHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    // 将 QString 转换为 UTF-8 编码的字节数组
    QByteArray localMsg = msg.toUtf8();

    // 为了避免与 qInfo/qDebug 等自带的格式混淆，我们可以根据需要自行格式化输出
    // 这里我们简单地直接打印消息内容
    switch (type) {
    case QtDebugMsg:
        fprintf(stderr, "Qml debug: %s\n", localMsg.constData());
        break;
    case QtInfoMsg:
        // QML 的 console.log 通常会以 QtInfoMsg 的形式出现
        // 我们直接将它打印到标准输出
        fprintf(stdout, "Qml info: %s\n", localMsg.constData());
        break;
    case QtWarningMsg:
        fprintf(stderr, "Qml warning: %s\n", localMsg.constData());
        break;
    case QtCriticalMsg:
        fprintf(stderr, "Qml critical: %s\n", localMsg.constData());
        break;
    case QtFatalMsg:
        fprintf(stderr, "Fatal: %s\n", localMsg.constData());
        abort();
    }
    // 刷新缓冲区，确保立即显示
    fflush(stdout);
    fflush(stderr);
}

int main(int argc, char *argv[])
{

    qInstallMessageHandler(_qmlMessageHandler);

    // 1. 初始化ROS 2
    rclcpp::init(argc, argv);

    // 2. 使用 std::make_shared 来管理节点的生命周期
    auto ros_node = std::make_shared<QtROSNode>("qt_ros_node");
    auto ros_proxy = std::make_shared<ROSProxy>(); // Proxy活在主线程

    // 3. 创建一个QThread来运行ROS事件循环
    QThread* ros_thread = new QThread();

    // 4. 将我们的节点 "移动" 到新线程。这不是物理移动，而是设置其事件亲和性。
    ros_node->moveToThread(ros_thread);

    QGuiApplication app(argc, argv);

    // 注册自定义的类型 - struct
    qRegisterMetaType<CircuitStatusData>("CircuitStatusData");
    qRegisterMetaType<VoltageRegulatorStatusData>("VoltageRegulatorStatusData");
    // 注册自定义的类型 - class (顶层)
    qRegisterMetaType<SystemSettingsData>("SystemSettingsData");
    qRegisterMetaType<VoltageRegulatorSettingsData>("RegulatorSettingsData");
    qRegisterMetaType<CircuitSettingsData>("CircuitSettingsData");
    // 注册自定义的类型 - class (嵌套)
    qRegisterMetaType<LoopSettingsData>("LoopSettingsData");
    qRegisterMetaType<SampleSettingsData>("SampleSettingsData");

    // 5. 连接信号和槽
    // 订阅方向的连接：ROS -> GUI
    QObject::connect(ros_node.get(), &QtROSNode::circuitStatusReceived,
                     ros_proxy.get(), &ROSProxy::updateCircuitStatus,
                     Qt::QueuedConnection);
    QObject::connect(ros_node.get(), &QtROSNode::voltageRegulatorStatusReceived,
                     ros_proxy.get(), &ROSProxy::updateVoltageRegulatorStatus,
                     Qt::QueuedConnection);
    // 连接 SettingsUpdateResult
    QObject::connect(ros_node.get(), &QtROSNode::settingsUpdateResult,
                     ros_proxy.get(), &ROSProxy::onSettingsUpdateResult);

    // --- 发布方向的连接 GUI -> ROS ---
    QObject::connect(ros_proxy.get(), &ROSProxy::regulatorCommandRequested,
                     ros_node.get(), &QtROSNode::onSendRegulatorCommand,
                     Qt::QueuedConnection);
    QObject::connect(ros_proxy.get(), &ROSProxy::circuitCommandRequested,
                     ros_node.get(), &QtROSNode::onSendCircuitCommand,
                     Qt::QueuedConnection);
    QObject::connect(ros_proxy.get(), &ROSProxy::clearAlarmRequested,
                     ros_node.get(), &QtROSNode::onSendClearAlarm,
                     Qt::QueuedConnection);
    // 连接 SystemSettings
    QObject::connect(ros_proxy.get(), &ROSProxy::systemSettingsUpdateRequest,
                     ros_node.get(), &QtROSNode::onSetSystemSettings);
    // 连接 RegulatorSettings
    QObject::connect(ros_proxy.get(), &ROSProxy::regulatorSettingsUpdateRequest,
                     ros_node.get(), &QtROSNode::onSetRegulatorSettings);
    // 连接 CircuitSettings
    QObject::connect(ros_proxy.get(), &ROSProxy::circuitSettingsUpdateRequest,
                     ros_node.get(), &QtROSNode::onSetCircuitSettings);

    // 6. 当线程启动后，再去调用 startTimer 槽
    QObject::connect(ros_thread, &QThread::started, ros_node.get(), &QtROSNode::startTimer);

    /*
    // 7. 当Qt应用即将退出时，安全地关闭ROS 2
    QObject::connect(&app, &QGuiApplication::aboutToQuit, &app, [ros_node, ros_thread]() {
        RCLCPP_INFO(ros_node->get_logger(), "Application is about to quit. Shutting down ROS.");
        rclcpp::shutdown();
        ros_thread->quit(); // 告诉线程事件循环停止
        if (!ros_thread->wait(2000)) { // 等待最多2秒
            RCLCPP_WARN(ros_node->get_logger(), "ROS thread did not terminate gracefully. Forcing termination.");
            ros_thread->terminate();
            ros_thread->wait();
        }
        delete ros_thread;
    });
    */

    // 7. 当Qt应用即将退出时，执行一个精心设计的、线程安全的关闭序列
    QObject::connect(&app, &QGuiApplication::aboutToQuit, &app, [ros_node, ros_thread]() {
        if (!ros_thread->isRunning()) {
            return;
        }

        RCLCPP_INFO(ros_node->get_logger(), "Application is about to quit. Starting shutdown sequence.");

        // 第1步：停止ROS的spin()循环。
        // 这会解除对ros_thread事件循环的阻塞，使其能够处理后续的Qt事件（比如deleteLater）。
        rclcpp::shutdown();

        // 第2步：请求在ros_thread线程中安全地删除ros_node。
        // deleteLater()会向ros_thread的事件队列发布一个删除事件。
        // 节点将在自己的线程中被销毁，从而避免了跨线程销毁定时器的警告。
        ros_node->deleteLater();

        // 第3步：请求ros_thread的事件循环在处理完所有待办事件后退出。
        ros_thread->quit();

        // 第4步：阻塞主线程，等待ros_thread完全结束。
        // 推荐给一个超时时间，以防万一线程卡死。
        if (!ros_thread->wait(2000)) { // 等待最多2秒
            RCLCPP_WARN(ros_node->get_logger(), "ROS thread did not terminate gracefully. Forcing termination.");
            ros_thread->terminate(); // 作为最后的手段
            ros_thread->wait();
        }

        // 第5步：现在ros_thread已经停止，可以安全地从主线程删除QThread对象本身。
        delete ros_thread;

        RCLCPP_INFO(ros_node->get_logger(), "Shutdown sequence complete.");
    });

    // 8. 启动ROS线程
    ros_thread->start();

    QQmlApplicationEngine engine;

    // --- 注册枚举，以便在QML中使用 ---
    qmlRegisterUncreatableMetaObject(
        qt_node_constants::staticMetaObject,
        "qt_node",               // QML 导入 URI (保持不变)
        1, 0,
        "QtNodeConstants",                 // 在 QML 中使用的名字 (可以保持不变，为了UI侧的稳定性)
        "Error: Cannot create constants object."
        );

    // 修正这里的URL，使其与真实路径完全匹配
    const QUrl url(QStringLiteral("qrc:/qt_node/qml/Main.qml"));

    // 9. 将我们的ros_proxy代理暴露给QML，这样QML就可以调用它的方法
    engine.rootContext()->setContextProperty("rosProxy", ros_proxy.get());

    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
                         if (!obj && url == objUrl)
                             QCoreApplication::exit(-1);
                     }, Qt::QueuedConnection);

    engine.load(url);

    return app.exec();
}
