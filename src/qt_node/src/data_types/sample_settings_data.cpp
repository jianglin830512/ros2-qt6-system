#include "qt_node/data_types/sample_settings_data.hpp"
#include <QtMath>

SampleSettingsData::SampleSettingsData(QObject *parent) : QObject(parent)
{
}

QString SampleSettingsData::cable_type() const
{
    return m_cable_type;
}

void SampleSettingsData::setCable_type(const QString &cable_type)
{
    if (m_cable_type != cable_type) {
        m_cable_type = cable_type;
        emit cable_typeChanged();
    }
}

QString SampleSettingsData::cable_spec() const
{
    return m_cable_spec;
}

void SampleSettingsData::setCable_spec(const QString &cable_spec)
{
    if (m_cable_spec != cable_spec) {
        m_cable_spec = cable_spec;
        emit cable_specChanged();
    }
}

QString SampleSettingsData::insulation_material() const
{
    return m_insulation_material;
}

void SampleSettingsData::setInsulation_material(const QString &insulation_material)
{
    if (m_insulation_material != insulation_material) {
        m_insulation_material = insulation_material;
        emit insulation_materialChanged();
    }
}

double SampleSettingsData::insulation_thickness() const
{
    return m_insulation_thickness;
}

void SampleSettingsData::setInsulation_thickness(double insulation_thickness)
{
    if (!qFuzzyCompare(m_insulation_thickness, insulation_thickness)) {
        m_insulation_thickness = insulation_thickness;
        emit insulation_thicknessChanged();
    }
}

bool SampleSettingsData::operator==(const SampleSettingsData& other) const
{
    return m_cable_type == other.m_cable_type &&
           m_cable_spec == other.m_cable_spec &&
           m_insulation_material == other.m_insulation_material &&
           qFuzzyCompare(m_insulation_thickness, other.m_insulation_thickness);
}
