#include <QApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#include <QThread>
#include <QQuickStyle>

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

    QApplication app(argc, argv);
    // QGuiApplication app(argc, argv);
    QQuickStyle::setStyle("Fusion");
    qmlRegisterSingletonType(QUrl("qrc:/qt_node/qml/Theme.qml"), "qt.theme", 1, 0, "Theme");

    // 注册自定义的类型 - struct
    qRegisterMetaType<LoopStatusData>("LoopStatusData");
    qRegisterMetaType<CircuitStatusData>("CircuitStatusData");
    qRegisterMetaType<RegulatorStatusData>("RegulatorStatusData");
    // 注册自定义的类型 - class (顶层)
    qRegisterMetaType<SystemSettingsData>("SystemSettingsData");
    qRegisterMetaType<RegulatorSettingsData>("RegulatorSettingsData");
    qRegisterMetaType<CircuitSettingsData>("CircuitSettingsData");
    // 注册自定义的类型 - class (嵌套)
    qRegisterMetaType<LoopSettingsData>("LoopSettingsData");
    qRegisterMetaType<SampleSettingsData>("SampleSettingsData");
    // 注册 ROS 消息指针类型，以便跨线程传递
    qRegisterMetaType<SystemSettingsMsgPtr>("SystemSettingsMsgPtr");
    qRegisterMetaType<RegulatorSettingsMsgPtr>("RegulatorSettingsMsgPtr");
    qRegisterMetaType<CircuitSettingsMsgPtr>("CircuitSettingsMsgPtr");

    // 5. 连接信号和槽
    // --- 订阅方向的连接：ROS -> GUI ---
    QObject::connect(ros_node.get(), &QtROSNode::circuitStatusReceived,
                     ros_proxy.get(), &ROSProxy::updateCircuitStatus,
                     Qt::QueuedConnection); // 显示指定异步，用于跨线程
    QObject::connect(ros_node.get(), &QtROSNode::regulatorStatusReceived, // 变更
                     ros_proxy.get(), &ROSProxy::updateRegulatorStatus,
                     Qt::QueuedConnection);

    // Settings 更新连接 (ROS -> GUI) - 之前漏掉了这些！
    QObject::connect(ros_node.get(), &QtROSNode::systemSettingsReceived,
                     ros_proxy.get(), &ROSProxy::updateSystemSettings,
                     Qt::QueuedConnection);
    QObject::connect(ros_node.get(), &QtROSNode::regulatorSettingsReceived,
                     ros_proxy.get(), &ROSProxy::updateRegulatorSettings,
                     Qt::QueuedConnection);
    QObject::connect(ros_node.get(), &QtROSNode::circuitSettingsReceived,
                     ros_proxy.get(), &ROSProxy::updateCircuitSettings,
                     Qt::QueuedConnection);

    // --- Settings, Command 结果返回的连接：ROS -> GUI ---
    QObject::connect(ros_node.get(), &QtROSNode::settingsUpdateResult,
                     ros_proxy.get(), &ROSProxy::onSettingsUpdateResult);
    QObject::connect(ros_node.get(), &QtROSNode::commandResult,
                     ros_proxy.get(), &ROSProxy::onCommandResult);

    // --- Command 连接 GUI -> ROS ---
    QObject::connect(ros_proxy.get(), &ROSProxy::regulatorOperationCommandRequested,
                     ros_node.get(), &QtROSNode::onSendRegulatorOperationCommand, Qt::QueuedConnection);
    QObject::connect(ros_proxy.get(), &ROSProxy::regulatorBreakerCommandRequested,
                     ros_node.get(), &QtROSNode::onSendRegulatorBreakerCommand, Qt::QueuedConnection);
    QObject::connect(ros_proxy.get(), &ROSProxy::circuitModeCommandRequested,
                     ros_node.get(), &QtROSNode::onSendCircuitModeCommand, Qt::QueuedConnection);
    QObject::connect(ros_proxy.get(), &ROSProxy::circuitBreakerCommandRequested,
                     ros_node.get(), &QtROSNode::onSendCircuitBreakerCommand, Qt::QueuedConnection);
    QObject::connect(ros_proxy.get(), &ROSProxy::clearAlarmRequested,
                     ros_node.get(), &QtROSNode::onSendClearAlarm, Qt::QueuedConnection);

    // --- Settings 连接 GUI -> ROS ---
    QObject::connect(ros_proxy.get(), &ROSProxy::systemSettingsUpdateRequest,
                     ros_node.get(), &QtROSNode::onSetSystemSettings,
                     Qt::QueuedConnection);
    QObject::connect(ros_proxy.get(), &ROSProxy::regulatorSettingsUpdateRequest,
                     ros_node.get(), &QtROSNode::onSetRegulatorSettings,
                     Qt::QueuedConnection);
    QObject::connect(ros_proxy.get(), &ROSProxy::circuitSettingsUpdateRequest,
                     ros_node.get(), &QtROSNode::onSetCircuitSettings,
                     Qt::QueuedConnection);

    // --- 关闭程序的连接：GUI -> ROS ---
    QObject::connect(ros_proxy.get(), &ROSProxy::shutdownRequested,
                     ros_node.get(), &QtROSNode::onShutdownRequested,
                     Qt::QueuedConnection);

    // --- 关闭程序的连接：ROS -> GUI ---
    QObject::connect(ros_node.get(), &QtROSNode::shutdownFinished,
                     &app, [&app, ros_thread]() {
                         ros_thread->quit();
                         if (!ros_thread->wait(1000)) {
                             ros_thread->terminate();
                             ros_thread->wait();
                         }
                         delete ros_thread;

                         app.quit(); // 这是最后一步，真正关闭UI
                     },
                     Qt::QueuedConnection);

    // 6. 当线程启动后，再去调用 startTimer 槽
    QObject::connect(ros_thread, &QThread::started, ros_node.get(), &QtROSNode::startTimer);

    // 7. 启动ROS线程
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

    // 8. 将我们的ros_proxy代理暴露给QML，这样QML就可以调用它的方法
    engine.rootContext()->setContextProperty("rosProxy", ros_proxy.get());

    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
                         if (!obj && url == objUrl)
                             QCoreApplication::exit(-1);
                     }, Qt::QueuedConnection);

    engine.load(url);

    return app.exec();
}
