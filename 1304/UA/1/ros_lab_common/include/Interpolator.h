#ifndef PROJECT_ANGLEVELOCITY_H
#define PROJECT_ANGLEVELOCITY_H

#include <cmath>

class Interpolator
{
public:
    Interpolator(const float toMinimum,
                   const float toMaximum,
                   const float fromMinimum,
                   const float fromMaximum)
            : m_toMaximum(toMaximum)
            , m_toMinimum(toMinimum)
            , m_fromMaximum(fromMaximum)
            , m_fromMinimum(fromMinimum)
    {
    }

    float interpolate(float fromValue) const
    {
        float coefficient = fromValue / (m_fromMaximum - m_fromMinimum);
        return (m_toMaximum - m_toMinimum) * std::sin((float) (coefficient * M_PI)) + m_toMinimum;
    }

private:
    const float m_toMaximum;
    const float m_toMinimum;
    const float m_fromMaximum;
    const float m_fromMinimum;
};

#endif //PROJECT_ANGLEVELOCITY_H
