#ifndef CABLE_DATA_HPP
#define CABLE_DATA_HPP

#include <QObject>
#include <QString>

class CableData : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int id READ id WRITE setId NOTIFY idChanged)
    Q_PROPERTY(QString name READ name WRITE setName NOTIFY nameChanged)
    Q_PROPERTY(double core_diameter READ core_diameter WRITE setCore_diameter NOTIFY core_diameterChanged)
    Q_PROPERTY(QString core_material READ core_material WRITE setCore_material NOTIFY core_materialChanged)
    Q_PROPERTY(double insulation_thickness READ insulation_thickness WRITE setInsulation_thickness NOTIFY insulation_thicknessChanged)
    Q_PROPERTY(QString insulation_material READ insulation_material WRITE setInsulation_material NOTIFY insulation_materialChanged)
    Q_PROPERTY(int voltage_grade READ voltage_grade WRITE setVoltage_grade NOTIFY voltage_gradeChanged)
    Q_PROPERTY(int system_format READ system_format WRITE setSystem_format NOTIFY system_formatChanged)
    Q_PROPERTY(QString notes READ notes WRITE setNotes NOTIFY notesChanged)
    Q_PROPERTY(QString last_modified READ last_modified WRITE setLast_modified NOTIFY last_modifiedChanged)

public:
    explicit CableData(QObject *parent = nullptr);

    CableData(const CableData& other) = delete;
    CableData& operator=(const CableData& other) = delete;

    int id() const; void setId(int id);
    QString name() const; void setName(const QString &name);
    double core_diameter() const; void setCore_diameter(double core_diameter);
    QString core_material() const; void setCore_material(const QString &core_material);
    double insulation_thickness() const; void setInsulation_thickness(double insulation_thickness);
    QString insulation_material() const; void setInsulation_material(const QString &insulation_material);
    int voltage_grade() const; void setVoltage_grade(int voltage_grade);
    int system_format() const; void setSystem_format(int system_format);
    QString notes() const; void setNotes(const QString &notes);
    QString last_modified() const; void setLast_modified(const QString &last_modified);

    bool operator==(const CableData& other) const;

signals:
    void idChanged();
    void nameChanged();
    void core_diameterChanged();
    void core_materialChanged();
    void insulation_thicknessChanged();
    void insulation_materialChanged();
    void voltage_gradeChanged();
    void system_formatChanged();
    void notesChanged();
    void last_modifiedChanged();

private:
    int m_id = -1;
    QString m_name;
    double m_core_diameter = 0.0;
    QString m_core_material;
    double m_insulation_thickness = 0.0;
    QString m_insulation_material;
    int m_voltage_grade = 0;
    int m_system_format = 0;
    QString m_notes;
    QString m_last_modified;
};

#endif // CABLE_DATA_HPP
