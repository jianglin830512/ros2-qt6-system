#ifndef LOOPSETTINGSDATA_HPP
#define LOOPSETTINGSDATA_HPP

#include <QObject>
#include <QDateTime> // 包含 QDateTime

class LoopSettingsData : public QObject
{
    Q_OBJECT
    // --- 属性保持不变 ---
    Q_PROPERTY(int start_current_a READ start_current_a WRITE setStart_current_a NOTIFY start_current_aChanged)
    Q_PROPERTY(int max_current_a READ max_current_a WRITE setMax_current_a NOTIFY max_current_aChanged)
    Q_PROPERTY(int current_change_range_percent READ current_change_range_percent WRITE setCurrent_change_range_percent NOTIFY current_change_range_percentChanged)
    Q_PROPERTY(int ct_ratio READ ct_ratio WRITE setCt_ratio NOTIFY ct_ratioChanged)
    Q_PROPERTY(int cycle_count READ cycle_count WRITE setCycle_count NOTIFY cycle_countChanged)

    // --- 属性已修改 ---
    Q_PROPERTY(QDateTime start_datetime READ start_datetime WRITE setStart_datetime NOTIFY start_datetimeChanged)
    Q_PROPERTY(int heating_duration_sec READ heating_duration_sec WRITE setHeating_duration_sec NOTIFY heating_duration_secChanged)

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
    QDateTime start_datetime() const;
    int heating_duration_sec() const;


    // --- Setters ---
    void setStart_current_a(int start_current_a);
    void setMax_current_a(int max_current_a);
    void setCurrent_change_range_percent(int current_change_range_percent);
    void setCt_ratio(int ct_ratio);
    void setCycle_count(int cycle_count);
    void setStart_datetime(const QDateTime &start_datetime);
    void setHeating_duration_sec(int heating_duration_sec);

    // --- 运算符重载 ---
    bool operator==(const LoopSettingsData& other) const;

signals:
    // --- 信号 ---
    void start_current_aChanged();
    void max_current_aChanged();
    void current_change_range_percentChanged();
    void ct_ratioChanged();
    void cycle_countChanged();
    void start_datetimeChanged();
    void heating_duration_secChanged();


private:
    // --- 成员变量 ---
    int m_start_current_a = 0;
    int m_max_current_a = 0;
    int m_current_change_range_percent = 0;
    int m_ct_ratio = 0;
    int m_cycle_count = 0;
    QDateTime m_start_datetime = QDateTime::currentDateTime(); // 使用 QDateTime
    int m_heating_duration_sec = 0; // 使用秒
};

#endif // LOOPSETTINGSDATA_HPP
