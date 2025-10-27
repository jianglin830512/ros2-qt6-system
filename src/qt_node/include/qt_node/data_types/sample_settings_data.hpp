#ifndef SAMPLESETTINGSDATA_HPP
#define SAMPLESETTINGSDATA_HPP

#include <QObject>
#include <QString>

class SampleSettingsData : public QObject
{
    Q_OBJECT
    Q_PROPERTY(QString cable_type READ cable_type WRITE setCable_type NOTIFY cable_typeChanged)
    Q_PROPERTY(QString cable_spec READ cable_spec WRITE setCable_spec NOTIFY cable_specChanged)
    Q_PROPERTY(QString insulation_material READ insulation_material WRITE setInsulation_material NOTIFY insulation_materialChanged)
    Q_PROPERTY(double insulation_thickness READ insulation_thickness WRITE setInsulation_thickness NOTIFY insulation_thicknessChanged)

public:
    explicit SampleSettingsData(QObject *parent = nullptr);

    // 禁止拷贝构造和拷贝赋值
    SampleSettingsData(const SampleSettingsData& other) = delete;
    SampleSettingsData& operator=(const SampleSettingsData& other) = delete;

    // Getter methods
    QString cable_type() const;
    QString cable_spec() const;
    QString insulation_material() const;
    double insulation_thickness() const;

    // Setter methods
    void setCable_type(const QString &cable_type);
    void setCable_spec(const QString &cable_spec);
    void setInsulation_material(const QString &insulation_material);
    void setInsulation_thickness(double insulation_thickness);

    bool operator==(const SampleSettingsData& other) const;

signals:
    void cable_typeChanged();
    void cable_specChanged();
    void insulation_materialChanged();
    void insulation_thicknessChanged();

private:
    QString m_cable_type;
    QString m_cable_spec;
    QString m_insulation_material;
    double m_insulation_thickness = 0.0;
};

#endif // SAMPLESETTINGSDATA_HPP
