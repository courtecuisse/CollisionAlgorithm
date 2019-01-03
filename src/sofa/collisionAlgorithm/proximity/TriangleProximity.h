#pragma once

#include <sofa/collisionAlgorithm/geometry/TriangleGeometry.h>
#include <sofa/collisionAlgorithm/element/TriangleElement.h>

namespace sofa
{

namespace collisionAlgorithm
{

class TriangleProximity : public ConstraintProximity
{
public :

    static defaulttype::Vector3 getPhongNormal(const TriangleElement* element, double fact[3])
    {
        const std::vector<defaulttype::Vector3> & normals = element->geometry()->m_pointNormal;
        return normals[element->m_pid[0]] * fact[0] +
               normals[element->m_pid[1]] * fact[1] +
               normals[element->m_pid[2]] * fact[2];
    }
    static ConstraintProximity::SPtr createProximity(const TriangleElement * elmt,double f1,double f2,double f3);


    TriangleProximity(const TriangleElement * elmt,double f1,double f2,double f3, bool phongNormals = true)
        : ConstraintProximity (elmt)
        , m_element(elmt)
    {
        m_fact[0] = f1;
        m_fact[1] = f2;
        m_fact[2] = f3;

        if(phongNormals)
        {
            m_normalFunction = std::bind(&TriangleProximity::getPhongNormal, elmt, m_fact);
        }
        else
        {
            m_normalFunction = [](const TriangleElement* elm, const double /*fact*/[3]) -> defaulttype::Vector3
            {
                return elm->geometry()->m_triangle_info[elm->m_eid].tn;
            };
        }

    }

    virtual ~TriangleProximity()
    {
    }

    virtual defaulttype::Vector3 getPosition(core::VecCoordId v) const
    {
        const helper::ReadAccessor<DataVecCoord> & pos = m_state->read(v);

        return pos[element()->m_pid[0]] * m_fact[0] +
               pos[element()->m_pid[1]] * m_fact[1] +
               pos[element()->m_pid[2]] * m_fact[2];
    }

    virtual defaulttype::Vector3 getNormal() const
    {
        return m_normalFunction(element(), m_fact);
    }

    virtual std::map<unsigned,double> getContributions() const
    {
        std::map<unsigned,double> res;

        res[element()->m_pid[0]] = m_fact[0];
        res[element()->m_pid[1]] = m_fact[1];
        res[element()->m_pid[2]] = m_fact[2];

        return res;
    }

    virtual inline const TriangleElement* element() const
    {
        return m_element;
    }

    double m_fact[3];
protected:
    const TriangleElement* m_element;

    std::function<defaulttype::Vector3(const TriangleElement*, const double [3]) > m_normalFunction;
};

}

}
