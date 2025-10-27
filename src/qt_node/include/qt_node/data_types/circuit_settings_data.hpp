#ifndef CIRCUIT_SETTINGS_DATA_HPP
#define CIRCUIT_SETTINGS_DATA_HPP

#include <QObject>
#include "loop_settings_data.hpp"
#include "sample_settings_data.hpp"

class CircuitSettingsData : public QObject
{
    Q_OBJECT
    // 关键改动：嵌套的QObject对象作为属性暴露，通常设为CONSTANT或READ-only
    Q_PROPERTY(LoopSettingsData* test_loop READ test_loop CONSTANT)
    Q_PROPERTY(LoopSettingsData* ref_loop READ ref_loop CONSTANT)
    Q_PROPERTY(SampleSettingsData* sample_params READ sample_params CONSTANT)

public:
    explicit CircuitSettingsData(QObject *parent = nullptr);

    CircuitSettingsData(const CircuitSettingsData& other) = delete;
    CircuitSettingsData& operator=(const CircuitSettingsData& other) = delete;

    // Getters for the nested objects
    LoopSettingsData* test_loop() const;
    LoopSettingsData* ref_loop() const;
    SampleSettingsData* sample_params() const;

    bool operator==(const CircuitSettingsData& other) const;

private:
    // 成员变量必须是指针，并在构造函数中初始化
    LoopSettingsData* m_test_loop;
    LoopSettingsData* m_ref_loop;
    SampleSettingsData* m_sample_params;
};

#endif // CIRCUIT_SETTINGS_DATA_HPP
