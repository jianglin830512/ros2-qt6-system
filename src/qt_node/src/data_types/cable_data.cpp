#include "qt_node/data_types/cable_data.hpp"
#include <QtMath>

CableData::CableData(QObject *parent) : QObject(parent) {}

int CableData::id() const { return m_id; }
void CableData::setId(int id) { if (m_id != id) { m_id = id; emit idChanged(); } }

QString CableData::name() const { return m_name; }
void CableData::setName(const QString &name) { if (m_name != name) { m_name = name; emit nameChanged(); } }

double CableData::core_diameter() const { return m_core_diameter; }
void CableData::setCore_diameter(double core_diameter) { if (!qFuzzyCompare(m_core_diameter, core_diameter)) { m_core_diameter = core_diameter; emit core_diameterChanged(); } }

QString CableData::core_material() const { return m_core_material; }
void CableData::setCore_material(const QString &core_material) { if (m_core_material != core_material) { m_core_material = core_material; emit core_materialChanged(); } }

double CableData::insulation_thickness() const { return m_insulation_thickness; }
void CableData::setInsulation_thickness(double insulation_thickness) { if (!qFuzzyCompare(m_insulation_thickness, insulation_thickness)) { m_insulation_thickness = insulation_thickness; emit insulation_thicknessChanged(); } }

QString CableData::insulation_material() const { return m_insulation_material; }
void CableData::setInsulation_material(const QString &insulation_material) { if (m_insulation_material != insulation_material) { m_insulation_material = insulation_material; emit insulation_materialChanged(); } }

int CableData::voltage_grade() const { return m_voltage_grade; }
void CableData::setVoltage_grade(int voltage_grade) { if (m_voltage_grade != voltage_grade) { m_voltage_grade = voltage_grade; emit voltage_gradeChanged(); } }

int CableData::system_format() const { return m_system_format; }
void CableData::setSystem_format(int system_format) { if (m_system_format != system_format) { m_system_format = system_format; emit system_formatChanged(); } }

QString CableData::notes() const { return m_notes; }
void CableData::setNotes(const QString &notes) { if (m_notes != notes) { m_notes = notes; emit notesChanged(); } }

QString CableData::last_modified() const { return m_last_modified; }
void CableData::setLast_modified(const QString &last_modified) { if (m_last_modified != last_modified) { m_last_modified = last_modified; emit last_modifiedChanged(); } }

bool CableData::operator==(const CableData& other) const {
    return m_id == other.m_id && m_name == other.m_name && qFuzzyCompare(m_core_diameter, other.m_core_diameter) &&
           m_core_material == other.m_core_material && qFuzzyCompare(m_insulation_thickness, other.m_insulation_thickness) &&
           m_insulation_material == other.m_insulation_material && m_voltage_grade == other.m_voltage_grade &&
           m_system_format == other.m_system_format && m_notes == other.m_notes && m_last_modified == other.m_last_modified;
}
