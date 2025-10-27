#include "qt_node/data_types/voltage_regulator_settings_data.hpp"

VoltageRegulatorSettingsData::VoltageRegulatorSettingsData(QObject *parent) : QObject(parent)
{
}

int VoltageRegulatorSettingsData::over_current_a() const { return m_over_current_a; }
int VoltageRegulatorSettingsData::over_voltage_v() const { return m_over_voltage_v; }
int VoltageRegulatorSettingsData::voltage_up_speed_percent() const { return m_voltage_up_speed_percent; }
int VoltageRegulatorSettingsData::voltage_down_speed_percent() const { return m_voltage_down_speed_percent; }
bool VoltageRegulatorSettingsData::over_voltage_protection_mode() const { return m_over_voltage_protection_mode; }

void VoltageRegulatorSettingsData::setOver_current_a(int over_current_a)
{
    if (m_over_current_a != over_current_a) {
        m_over_current_a = over_current_a;
        emit over_current_aChanged();
    }
}

void VoltageRegulatorSettingsData::setOver_voltage_v(int over_voltage_v)
{
    if (m_over_voltage_v != over_voltage_v) {
        m_over_voltage_v = over_voltage_v;
        emit over_voltage_vChanged();
    }
}

void VoltageRegulatorSettingsData::setVoltage_up_speed_percent(int voltage_up_speed_percent)
{
    if (m_voltage_up_speed_percent != voltage_up_speed_percent) {
        m_voltage_up_speed_percent = voltage_up_speed_percent;
        emit voltage_up_speed_percentChanged();
    }
}

void VoltageRegulatorSettingsData::setVoltage_down_speed_percent(int voltage_down_speed_percent)
{
    if (m_voltage_down_speed_percent != voltage_down_speed_percent) {
        m_voltage_down_speed_percent = voltage_down_speed_percent;
        emit voltage_down_speed_percentChanged();
    }
}

void VoltageRegulatorSettingsData::setOver_voltage_protection_mode(bool over_voltage_protection_mode)
{
    if (m_over_voltage_protection_mode != over_voltage_protection_mode) {
        m_over_voltage_protection_mode = over_voltage_protection_mode;
        emit over_voltage_protection_modeChanged();
    }
}
