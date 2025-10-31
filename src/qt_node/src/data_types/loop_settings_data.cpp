#include "qt_node/data_types/loop_settings_data.hpp"

LoopSettingsData::LoopSettingsData(QObject *parent) : QObject(parent)
{
    // 构造函数保持默认即可
}

// --- Getters ---
int LoopSettingsData::start_current_a() const { return m_start_current_a; }
int LoopSettingsData::max_current_a() const { return m_max_current_a; }
int LoopSettingsData::current_change_range_percent() const { return m_current_change_range_percent; }
int LoopSettingsData::ct_ratio() const { return m_ct_ratio; }
int LoopSettingsData::cycle_count() const { return m_cycle_count; }
QDateTime LoopSettingsData::start_datetime() const { return m_start_datetime; }
int LoopSettingsData::heating_duration_sec() const { return m_heating_duration_sec; }

// --- Setters ---
void LoopSettingsData::setStart_current_a(int start_current_a)
{
    if (m_start_current_a != start_current_a) {
        m_start_current_a = start_current_a;
        emit start_current_aChanged();
    }
}
void LoopSettingsData::setMax_current_a(int max_current_a)
{
    if (m_max_current_a != max_current_a) {
        m_max_current_a = max_current_a;
        emit max_current_aChanged();
    }
}
void LoopSettingsData::setCurrent_change_range_percent(int current_change_range_percent)
{
    if (m_current_change_range_percent != current_change_range_percent) {
        m_current_change_range_percent = current_change_range_percent;
        emit current_change_range_percentChanged();
    }
}
void LoopSettingsData::setCt_ratio(int ct_ratio)
{
    if (m_ct_ratio != ct_ratio) {
        m_ct_ratio = ct_ratio;
        emit ct_ratioChanged();
    }
}
void LoopSettingsData::setCycle_count(int cycle_count)
{
    if (m_cycle_count != cycle_count) {
        m_cycle_count = cycle_count;
        emit cycle_countChanged();
    }
}

void LoopSettingsData::setStart_datetime(const QDateTime &start_datetime)
{
    if (m_start_datetime != start_datetime) {
        m_start_datetime = start_datetime;
        emit start_datetimeChanged();
    }
}
void LoopSettingsData::setHeating_duration_sec(int heating_duration_sec)
{
    if (m_heating_duration_sec != heating_duration_sec) {
        m_heating_duration_sec = heating_duration_sec;
        emit heating_duration_secChanged();
    }
}

// --- operator== ---
bool LoopSettingsData::operator==(const LoopSettingsData& other) const
{
    return m_start_current_a == other.m_start_current_a &&
           m_max_current_a == other.m_max_current_a &&
           m_current_change_range_percent == other.m_current_change_range_percent &&
           m_ct_ratio == other.m_ct_ratio &&
           m_cycle_count == other.m_cycle_count &&
           m_start_datetime == other.m_start_datetime &&
           m_heating_duration_sec == other.m_heating_duration_sec;
}
