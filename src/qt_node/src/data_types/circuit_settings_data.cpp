#include "qt_node/data_types/circuit_settings_data.hpp"

CircuitSettingsData::CircuitSettingsData(QObject *parent)
    : QObject(parent)
{
    // 初始化子对象，并设置this作为它们的父对象，以便进行内存管理
    m_test_loop = new LoopSettingsData(this);
    m_ref_loop = new LoopSettingsData(this);
    m_sample_params = new SampleSettingsData(this);
}

LoopSettingsData* CircuitSettingsData::test_loop() const
{
    return m_test_loop;
}

LoopSettingsData* CircuitSettingsData::ref_loop() const
{
    return m_ref_loop;
}

SampleSettingsData* CircuitSettingsData::sample_params() const
{
    return m_sample_params;
}

bool CircuitSettingsData::operator==(const CircuitSettingsData& other) const
{
    // 比较指针所指向对象的内容
    return (*m_test_loop == *other.m_test_loop) &&
           (*m_ref_loop == *other.m_ref_loop) &&
           (*m_sample_params == *other.m_sample_params);
}
