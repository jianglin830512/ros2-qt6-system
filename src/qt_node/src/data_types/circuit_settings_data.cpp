#include "qt_node/data_types/circuit_settings_data.hpp"

CircuitSettingsData::CircuitSettingsData(QObject *parent)
    : QObject(parent)
{
    m_test_loop = new LoopSettingsData(this);
    m_ref_loop = new LoopSettingsData(this);
    m_sample_params = new SampleSettingsData(this);
}

// Getters
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

bool CircuitSettingsData::curr_mode_use_ref() const
{
    return m_curr_mode_use_ref;
}

// Setter
void CircuitSettingsData::setCurr_mode_use_ref(bool curr_mode_use_ref)
{
    // 仅当值发生变化时才更新并发送信号
    if (m_curr_mode_use_ref == curr_mode_use_ref)
        return;

    m_curr_mode_use_ref = curr_mode_use_ref;
    emit curr_mode_use_refChanged();
}

bool CircuitSettingsData::operator==(const CircuitSettingsData& other) const
{
    // 比较指针所指向对象的内容
    return (*m_test_loop == *other.m_test_loop) &&
           (*m_ref_loop == *other.m_ref_loop) &&
           (*m_sample_params == *other.m_sample_params)&&
           (m_curr_mode_use_ref == other.m_curr_mode_use_ref);
}
