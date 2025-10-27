#ifndef VOLTAGE_REGULATOR_SETTINGS_DATA_HPP
#define VOLTAGE_REGULATOR_SETTINGS_DATA_HPP

#include <QObject>

class VoltageRegulatorSettingsData : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int over_current_a READ over_current_a WRITE setOver_current_a NOTIFY over_current_aChanged)
    Q_PROPERTY(int over_voltage_v READ over_voltage_v WRITE setOver_voltage_v NOTIFY over_voltage_vChanged)
    Q_PROPERTY(int voltage_up_speed_percent READ voltage_up_speed_percent WRITE setVoltage_up_speed_percent NOTIFY voltage_up_speed_percentChanged)
    Q_PROPERTY(int voltage_down_speed_percent READ voltage_down_speed_percent WRITE setVoltage_down_speed_percent NOTIFY voltage_down_speed_percentChanged)
    Q_PROPERTY(bool over_voltage_protection_mode READ over_voltage_protection_mode WRITE setOver_voltage_protection_mode NOTIFY over_voltage_protection_modeChanged)

public:
    explicit VoltageRegulatorSettingsData(QObject *parent = nullptr);

    VoltageRegulatorSettingsData(const VoltageRegulatorSettingsData& other) = delete;
    VoltageRegulatorSettingsData& operator=(const VoltageRegulatorSettingsData& other) = delete;

    // Getters
    int over_current_a() const;
    int over_voltage_v() const;
    int voltage_up_speed_percent() const;
    int voltage_down_speed_percent() const;
    bool over_voltage_protection_mode() const;

    // Setters
    void setOver_current_a(int over_current_a);
    void setOver_voltage_v(int over_voltage_v);
    void setVoltage_up_speed_percent(int voltage_up_speed_percent);
    void setVoltage_down_speed_percent(int voltage_down_speed_percent);
    void setOver_voltage_protection_mode(bool over_voltage_protection_mode);

signals:
    void over_current_aChanged();
    void over_voltage_vChanged();
    void voltage_up_speed_percentChanged();
    void voltage_down_speed_percentChanged();
    void over_voltage_protection_modeChanged();

private:
    int m_over_current_a = 0;
    int m_over_voltage_v = 0;
    int m_voltage_up_speed_percent = 0;
    int m_voltage_down_speed_percent = 0;
    bool m_over_voltage_protection_mode = false;
};

#endif // VOLTAGE_REGULATOR_SETTINGS_DATA_HPP
