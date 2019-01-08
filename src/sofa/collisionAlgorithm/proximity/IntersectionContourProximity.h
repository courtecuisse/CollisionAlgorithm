#pragma once

#include <sofa/collisionAlgorithm/geometry/IntersectionContourGeometry.h>
#include <sofa/collisionAlgorithm/element/IntersectionContourElement.h>

namespace sofa
{

namespace collisionAlgorithm
{

class IntersectionContourProximity : public BaseProximity
{
public :
    IntersectionContourProximity(const IntersectionContourElement * elmt)
        : BaseProximity(elmt)
    {}

    defaulttype::Vector3 getPosition(core::VecCoordId v) const
    {
        const helper::ReadAccessor<Data<helper::vector<defaulttype::Vector3> > > & pos = m_state->read(v);

        defaulttype::Vector3 P = pos[element()->m_pid[0]] * element()->m_fact[0];
        defaulttype::Vector3 Q = pos[element()->m_pid[1]] * element()->m_fact[1];

        return (P+Q);
    }

    defaulttype::Vector3 getNormal() const
    {
        return element()->geometry()->m_pointNormal[element()->m_pid[0]] * element()->m_fact[0] +
               element()->geometry()->m_pointNormal[element()->m_pid[1]] * element()->m_fact[1];
    }

    std::map<unsigned,double> getContributions() const {
        std::map<unsigned,double> res;

        res[element()->m_pid[0]] = element()->m_fact[0];
        res[element()->m_pid[1]] = element()->m_fact[1];

        return res;
    }

    inline const IntersectionContourElement * element() const {
        return m_element;
    }

private:
    const IntersectionContourElement* m_element;
};

}

}
