#ifndef LOOPSETTINGSDATA_HPP
#define LOOPSETTINGSDATA_HPP

#include <QObject>
#include <QDate>
#include <QTime>

class LoopSettingsData : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int start_current_a READ start_current_a WRITE setStart_current_a NOTIFY start_current_aChanged)
    Q_PROPERTY(int max_current_a READ max_current_a WRITE setMax_current_a NOTIFY max_current_aChanged)
    Q_PROPERTY(int current_change_range_percent READ current_change_range_percent WRITE setCurrent_change_range_percent NOTIFY current_change_range_percentChanged)
    Q_PROPERTY(int ct_ratio READ ct_ratio WRITE setCt_ratio NOTIFY ct_ratioChanged)
    Q_PROPERTY(QDate start_date READ start_date WRITE setStart_date NOTIFY start_dateChanged)
    Q_PROPERTY(int cycle_count READ cycle_count WRITE setCycle_count NOTIFY cycle_countChanged)
    Q_PROPERTY(QTime heating_time READ heating_time WRITE setHeating_time NOTIFY heating_timeChanged)
    Q_PROPERTY(int heating_duration_min READ heating_duration_min WRITE setHeating_duration_min NOTIFY heating_duration_minChanged)

public:
    explicit LoopSettingsData(QObject *parent = nullptr);

    LoopSettingsData(const LoopSettingsData& other) = delete;
    LoopSettingsData& operator=(const LoopSettingsData& other) = delete;

    // Getters
    int start_current_a() const;
    int max_current_a() const;
    int current_change_range_percent() const;
    int ct_ratio() const;
    QDate start_date() const;
    int cycle_count() const;
    QTime heating_time() const;
    int heating_duration_min() const;

    // Setters
    void setStart_current_a(int start_current_a);
    void setMax_current_a(int max_current_a);
    void setCurrent_change_range_percent(int current_change_range_percent);
    void setCt_ratio(int ct_ratio);
    void setStart_date(const QDate &start_date);
    void setCycle_count(int cycle_count);
    void setHeating_time(const QTime &heating_time);
    void setHeating_duration_min(int heating_duration_min);

    bool operator==(const LoopSettingsData& other) const;

signals:
    void start_current_aChanged();
    void max_current_aChanged();
    void current_change_range_percentChanged();
    void ct_ratioChanged();
    void start_dateChanged();
    void cycle_countChanged();
    void heating_timeChanged();
    void heating_duration_minChanged();

private:
    int m_start_current_a = 0;
    int m_max_current_a = 0;
    int m_current_change_range_percent = 0;
    int m_ct_ratio = 0;
    QDate m_start_date = QDate::currentDate();
    int m_cycle_count = 0;
    QTime m_heating_time = QTime(0, 0);
    int m_heating_duration_min = 0;
};

#endif // LOOPSETTINGSDATA_HPP
