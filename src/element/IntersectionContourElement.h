#pragma once

#include <BaseGeometry.h>

namespace sofa {

namespace collisionAlgorithm {

class IntersectionContourElement : public ConstraintElement {
    friend class IntersectionContourProximity;
    friend class IntersectionContourGeometry;
public:

    /**************************************************************************/
    /******************************PROXIMITY***********************************/
    /**************************************************************************/

    class IntersectionContourProximity : public ConstraintProximity {
    public :
        IntersectionContourProximity(IntersectionContourElement * elmt) : ConstraintProximity(elmt) {}

        defaulttype::Vector3 getPosition(core::VecCoordId v) const {
            const core::behavior::ReadAccessor<defaulttype::Vector3> & pos = m_state->read(v);

            defaulttype::Vector3 P = pos[element()->m_pid[0]] * element()->m_fact[0];
            defaulttype::Vector3 Q = pos[element()->m_pid[1]] * element()->m_fact[1];

            return (P+Q);
        }

        defaulttype::Vector3 getNormal() const {
            return element()->geometry()->m_pointNormal[element()->m_pid[0]] * element()->m_fact[0] +
                   element()->geometry()->m_pointNormal[element()->m_pid[1]] * element()->m_fact[1];
        }

        std::map<unsigned,double> getContributions() {
            std::map<unsigned,double> res;

            res[element()->m_pid[0]] = element()->m_fact[0];
            res[element()->m_pid[1]] = element()->m_fact[1];

            return res;
        }

        inline IntersectionContourElement * element() const {
            return (IntersectionContourElement*) m_element;
        }
    };

    /**************************************************************************/
    /******************************ELEMENT*************************************/
    /**************************************************************************/

    inline ConstraintProximityPtr createProximity() {
        return std::make_shared<IntersectionContourProximity>(this);
    }

    IntersectionContourElement(IntersectionContourGeometry *geo, unsigned pid1, unsigned pid2,double f1,double f2) : ConstraintElement(geo,1) {
        m_pid[0] = pid1;
        m_pid[1] = pid2;

        m_fact[0] = f1;
        m_fact[1] = f2;
    }

    ConstraintProximityPtr project(defaulttype::Vector3 /*P*/) {
        return createProximity();
    }

    ConstraintProximityPtr getControlPoint(const int /*i*/) {
        return createProximity();
    }

    IntersectionContourGeometry * geometry() {
        return (IntersectionContourGeometry *)m_geometry;
    }

    void draw(const core::visual::VisualParams * /*vparams*/) {
        glColor4dv(geometry()->d_color.getValue().data());

        glBegin(GL_POINTS);
        glPointSize(20);
        glVertex3dv(getControlPoint(0)->getPosition().data());
        glEnd();
    }

protected:
    unsigned m_pid[2];
    double m_fact[2];
};

}

}
