#ifndef SYSTEM_SETTINGS_DATA_HPP
#define SYSTEM_SETTINGS_DATA_HPP

#include <QObject>

class SystemSettingsData : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int sample_interval_sec READ sample_interval_sec WRITE setSample_interval_sec NOTIFY sample_interval_secChanged)
    Q_PROPERTY(int record_interval_min READ record_interval_min WRITE setRecord_interval_min NOTIFY record_interval_minChanged)
    Q_PROPERTY(bool keep_record_on_shutdown READ keep_record_on_shutdown WRITE setKeep_record_on_shutdown NOTIFY keep_record_on_shutdownChanged)

public:
    explicit SystemSettingsData(QObject *parent = nullptr);

    SystemSettingsData(const SystemSettingsData& other) = delete;
    SystemSettingsData& operator=(const SystemSettingsData& other) = delete;

    // Getters
    int sample_interval_sec() const;
    int record_interval_min() const;
    bool keep_record_on_shutdown() const;

    // Setters
    void setSample_interval_sec(int sample_interval_sec);
    void setRecord_interval_min(int record_interval_min);
    void setKeep_record_on_shutdown(bool keep_record_on_shutdown);


signals:
    void sample_interval_secChanged();
    void record_interval_minChanged();
    void keep_record_on_shutdownChanged();

private:
    int m_sample_interval_sec = 1;
    int m_record_interval_min = 1;
    bool m_keep_record_on_shutdown = false;
};

#endif // SYSTEM_SETTINGS_DATA_HPP
