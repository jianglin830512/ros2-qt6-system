#include "qt_node/data_types/loop_settings_data.hpp"

LoopSettingsData::LoopSettingsData(QObject *parent) : QObject(parent)
{
}

int LoopSettingsData::start_current_a() const { return m_start_current_a; }
int LoopSettingsData::max_current_a() const { return m_max_current_a; }
int LoopSettingsData::current_change_range_percent() const { return m_current_change_range_percent; }
int LoopSettingsData::ct_ratio() const { return m_ct_ratio; }
QDate LoopSettingsData::start_date() const { return m_start_date; }
int LoopSettingsData::cycle_count() const { return m_cycle_count; }
QTime LoopSettingsData::heating_time() const { return m_heating_time; }
int LoopSettingsData::heating_duration_min() const { return m_heating_duration_min; }

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

void LoopSettingsData::setStart_date(const QDate &start_date)
{
    if (m_start_date != start_date) {
        m_start_date = start_date;
        emit start_dateChanged();
    }
}

void LoopSettingsData::setCycle_count(int cycle_count)
{
    if (m_cycle_count != cycle_count) {
        m_cycle_count = cycle_count;
        emit cycle_countChanged();
    }
}

void LoopSettingsData::setHeating_time(const QTime &heating_time)
{
    if (m_heating_time != heating_time) {
        m_heating_time = heating_time;
        emit heating_timeChanged();
    }
}

void LoopSettingsData::setHeating_duration_min(int heating_duration_min)
{
    if (m_heating_duration_min != heating_duration_min) {
        m_heating_duration_min = heating_duration_min;
        emit heating_duration_minChanged();
    }
}

bool LoopSettingsData::operator==(const LoopSettingsData& other) const
{
    return m_start_current_a == other.m_start_current_a &&
           m_max_current_a == other.m_max_current_a &&
           m_current_change_range_percent == other.m_current_change_range_percent &&
           m_ct_ratio == other.m_ct_ratio &&
           m_start_date == other.m_start_date &&
           m_cycle_count == other.m_cycle_count &&
           m_heating_time == other.m_heating_time &&
           m_heating_duration_min == other.m_heating_duration_min;
}
