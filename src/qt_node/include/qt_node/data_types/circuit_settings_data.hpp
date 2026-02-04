#ifndef CIRCUIT_SETTINGS_DATA_HPP
#define CIRCUIT_SETTINGS_DATA_HPP

#include <QObject>
#include "loop_settings_data.hpp"
#include "sample_settings_data.hpp"

class CircuitSettingsData : public QObject
{
    Q_OBJECT
    // 嵌套的QObject对象指针，作为只读属性暴露
    Q_PROPERTY(LoopSettingsData* test_loop READ test_loop CONSTANT)
    Q_PROPERTY(LoopSettingsData* ref_loop READ ref_loop CONSTANT)
    Q_PROPERTY(SampleSettingsData* sample_params READ sample_params CONSTANT)

    // 简单的bool类型属性，提供读写和通知功能
    Q_PROPERTY(bool curr_mode_use_ref READ curr_mode_use_ref WRITE setCurr_mode_use_ref NOTIFY curr_mode_use_refChanged)

public:
    explicit CircuitSettingsData(QObject *parent = nullptr);

    CircuitSettingsData(const CircuitSettingsData& other) = delete;
    CircuitSettingsData& operator=(const CircuitSettingsData& other) = delete;

    // Getters
    LoopSettingsData* test_loop() const;
    LoopSettingsData* ref_loop() const;
    SampleSettingsData* sample_params() const;
    bool curr_mode_use_ref() const;

    // Setter for curr_mode_use_ref
    void setCurr_mode_use_ref(bool curr_mode_use_ref);

    bool operator==(const CircuitSettingsData& other) const;

signals:
    // 仅为可写属性添加 NOTIFY 信号
    void curr_mode_use_refChanged();

private:
    // 成员变量必须是指针，并在构造函数中初始化
    LoopSettingsData* m_test_loop;
    LoopSettingsData* m_ref_loop;
    SampleSettingsData* m_sample_params;
    bool m_curr_mode_use_ref = false; // 初始化为默认值
};

#endif // CIRCUIT_SETTINGS_DATA_HPP
