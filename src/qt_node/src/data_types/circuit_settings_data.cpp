#include "qt_node/data_types/circuit_settings_data.hpp"

CircuitSettingsData::CircuitSettingsData(QObject *parent)
    : QObject(parent)
{
    m_test_loop = new LoopSettingsData(this);
    m_ref_loop = new LoopSettingsData(this);
    m_sample_cable = new CableData(this); // 替换
}

LoopSettingsData* CircuitSettingsData::test_loop() const { return m_test_loop; }
LoopSettingsData* CircuitSettingsData::ref_loop() const { return m_ref_loop; }
CableData* CircuitSettingsData::sample_cable() const { return m_sample_cable; } // 替换

bool CircuitSettingsData::operator==(const CircuitSettingsData& other) const
{
    return (*m_test_loop == *other.m_test_loop) &&
           (*m_ref_loop == *other.m_ref_loop) &&
           (*m_sample_cable == *other.m_sample_cable); // 替换
}
