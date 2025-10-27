#include "qt_node/data_types/system_settings_data.hpp"

SystemSettingsData::SystemSettingsData(QObject *parent) : QObject(parent)
{
}

int SystemSettingsData::sample_interval_sec() const { return m_sample_interval_sec; }
int SystemSettingsData::record_interval_min() const { return m_record_interval_min; }
bool SystemSettingsData::keep_record_on_shutdown() const { return m_keep_record_on_shutdown; }

void SystemSettingsData::setSample_interval_sec(int sample_interval_sec)
{
    if (m_sample_interval_sec != sample_interval_sec) {
        m_sample_interval_sec = sample_interval_sec;
        emit sample_interval_secChanged();
    }
}

void SystemSettingsData::setRecord_interval_min(int record_interval_min)
{
    if (m_record_interval_min != record_interval_min) {
        m_record_interval_min = record_interval_min;
        emit record_interval_minChanged();
    }
}

void SystemSettingsData::setKeep_record_on_shutdown(bool keep_record_on_shutdown)
{
    if (m_keep_record_on_shutdown != keep_record_on_shutdown) {
        m_keep_record_on_shutdown = keep_record_on_shutdown;
        emit keep_record_on_shutdownChanged();
    }
}
