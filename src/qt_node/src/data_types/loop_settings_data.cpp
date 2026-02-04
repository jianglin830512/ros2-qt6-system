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
QDateTime LoopSettingsData::start_date() const { return m_start_date; }
int LoopSettingsData::heating_start_time_sec() const { return m_heating_start_time_sec; }
int LoopSettingsData::heating_duration_sec() const { return m_heating_duration_sec; }
bool LoopSettingsData::enabled() const{ return m_enabled; }

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

void LoopSettingsData::setStart_date(const QDateTime &start_date)
{
    // 我们可以强制把时间设为 00:00:00，或者完全信任 UI 传递的日期
    // 这里只比较日期部分是否变化，或者直接比较 QDateTime
    if (m_start_date != start_date) {
        m_start_date = start_date;
        emit start_dateChanged();
    }
}

void LoopSettingsData::setHeating_start_time_sec(int heating_start_time_sec)
{
    if (m_heating_start_time_sec != heating_start_time_sec) {
        m_heating_start_time_sec = heating_start_time_sec;
        emit heating_start_time_secChanged();
    }
}

void LoopSettingsData::setHeating_duration_sec(int heating_duration_sec)
{
    if (m_heating_duration_sec != heating_duration_sec) {
        m_heating_duration_sec = heating_duration_sec;
        emit heating_duration_secChanged();
    }
}

void LoopSettingsData::setEnabled(bool enabled)
{
    if (m_enabled != enabled) {
        m_enabled = enabled;
        emit enabledChanged(); // 发射信号！
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
           m_start_date == other.m_start_date &&
           m_heating_start_time_sec == other.m_heating_start_time_sec &&
           m_heating_duration_sec == other.m_heating_duration_sec &&
           m_enabled == other.m_enabled;
}
