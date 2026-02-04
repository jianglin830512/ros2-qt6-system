#ifndef LOOPSETTINGSDATA_HPP
#define LOOPSETTINGSDATA_HPP

#include <QObject>
#include <QDateTime> // 包含 QDateTime

class LoopSettingsData : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int start_current_a READ start_current_a WRITE setStart_current_a NOTIFY start_current_aChanged)
    Q_PROPERTY(int max_current_a READ max_current_a WRITE setMax_current_a NOTIFY max_current_aChanged)
    Q_PROPERTY(int current_change_range_percent READ current_change_range_percent WRITE setCurrent_change_range_percent NOTIFY current_change_range_percentChanged)
    Q_PROPERTY(int ct_ratio READ ct_ratio WRITE setCt_ratio NOTIFY ct_ratioChanged)
    Q_PROPERTY(int cycle_count READ cycle_count WRITE setCycle_count NOTIFY cycle_countChanged)
    // 试验开启日期 (QDateTime方便QML处理，但逻辑上只用日期部分)
    Q_PROPERTY(QDateTime start_date READ start_date WRITE setStart_date NOTIFY start_dateChanged)
    // 每日加热开启时刻 (距离00:00:00的秒数)
    Q_PROPERTY(int heating_start_time_sec READ heating_start_time_sec WRITE setHeating_start_time_sec NOTIFY heating_start_time_secChanged)
    Q_PROPERTY(int heating_duration_sec READ heating_duration_sec WRITE setHeating_duration_sec NOTIFY heating_duration_secChanged)
    Q_PROPERTY(bool enabled READ enabled WRITE setEnabled NOTIFY enabledChanged)
public:
    explicit LoopSettingsData(QObject *parent = nullptr);

    LoopSettingsData(const LoopSettingsData& other) = delete;
    LoopSettingsData& operator=(const LoopSettingsData& other) = delete;

    // --- Getters ---
    int start_current_a() const;
    int max_current_a() const;
    int current_change_range_percent() const;
    int ct_ratio() const;
    int cycle_count() const;
    QDateTime start_date() const;
    int heating_start_time_sec() const;
    int heating_duration_sec() const;
     bool enabled() const;


    // --- Setters ---
    void setStart_current_a(int start_current_a);
    void setMax_current_a(int max_current_a);
    void setCurrent_change_range_percent(int current_change_range_percent);
    void setCt_ratio(int ct_ratio);
    void setCycle_count(int cycle_count);
    void setStart_date(const QDateTime &start_date);
    void setHeating_start_time_sec(int heating_start_time_sec);
    void setHeating_duration_sec(int heating_duration_sec);
    void setEnabled(bool enabled);

    // --- 运算符重载 ---
    bool operator==(const LoopSettingsData& other) const;

signals:
    // --- 信号 ---
    void start_current_aChanged();
    void max_current_aChanged();
    void current_change_range_percentChanged();
    void ct_ratioChanged();
    void cycle_countChanged();
    void start_dateChanged();
    void heating_start_time_secChanged();
    void heating_duration_secChanged();
    void enabledChanged();

private:
    // --- 成员变量 ---
    int m_start_current_a = 0;
    int m_max_current_a = 0;
    int m_current_change_range_percent = 0;
    int m_ct_ratio = 0;
    int m_cycle_count = 0;
    QDateTime m_start_date = QDateTime::currentDateTime();
    int m_heating_start_time_sec = 0;
    int m_heating_duration_sec = 0; // 使用秒
    bool m_enabled = false;
};

#endif // LOOPSETTINGSDATA_HPP
