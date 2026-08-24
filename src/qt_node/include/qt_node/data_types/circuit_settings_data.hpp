#ifndef CIRCUIT_SETTINGS_DATA_HPP
#define CIRCUIT_SETTINGS_DATA_HPP

#include <QObject>
#include "loop_settings_data.hpp"
#include "cable_data.hpp"

class CircuitSettingsData : public QObject
{
    Q_OBJECT
    Q_PROPERTY(LoopSettingsData* test_loop READ test_loop CONSTANT)
    Q_PROPERTY(LoopSettingsData* ref_loop READ ref_loop CONSTANT)
    Q_PROPERTY(CableData* sample_cable READ sample_cable CONSTANT)

public:
    explicit CircuitSettingsData(QObject *parent = nullptr);

    CircuitSettingsData(const CircuitSettingsData& other) = delete;
    CircuitSettingsData& operator=(const CircuitSettingsData& other) = delete;

    LoopSettingsData* test_loop() const;
    LoopSettingsData* ref_loop() const;
    CableData* sample_cable() const; // 替换

    bool operator==(const CircuitSettingsData& other) const;

signals:
    void curr_mode_use_refChanged();

private:
    LoopSettingsData* m_test_loop;
    LoopSettingsData* m_ref_loop;
    CableData* m_sample_cable;       // 替换
};

#endif // CIRCUIT_SETTINGS_DATA_HPP
