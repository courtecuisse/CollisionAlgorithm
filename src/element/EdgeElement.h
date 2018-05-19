#pragma once

#include <geometry/EdgeGeometry.h>
#include <qopengl.h>

namespace collisionAlgorithm {

/**************************************************************************/
/******************************ELEMENT*************************************/
/**************************************************************************/

class EdgeElement : public ConstraintElement {
    friend class EdgeGeometry;
    friend class EdgeProximity;

public:

    /**************************************************************************/
    /******************************PROXIMITY***********************************/
    /**************************************************************************/

    class EdgeProximity : public ConstraintProximity {
    public :

        inline EdgeElement * element() const {
            return (EdgeElement*) m_element;
        }

        EdgeProximity(EdgeElement * elmt,double f1,double f2) : ConstraintProximity(elmt) {
            m_fact[0] = f1;
            m_fact[1] = f2;
        }

        Vector3 getPosition(VecID v) const {
            const ReadAccessor<Vector3> & pos = m_state->read(v);
            return pos[element()->m_pid[0]] * m_fact[0] + pos[element()->m_pid[1]] * m_fact[1];
        }

        Vector3 getNormal() const {
            return Vector3(1,0,0);
        }

        std::map<unsigned,double> getContributions() {
            std::map<unsigned,double> res;

            res[element()->m_pid[0]] = m_fact[0];
            res[element()->m_pid[1]] = m_fact[1];

            return res;
        }

    protected:
        double m_fact[2];
    };

    inline ConstraintProximityPtr createProximity(EdgeElement * elmt,double f1,double f2) {
        return std::make_shared<EdgeProximity>(elmt,f1,f2);
    }

    EdgeElement(EdgeGeometry * geo,unsigned eid) : ConstraintElement(geo,2) {
        m_eid = eid;

        const std::vector<Topology::Edge> & edges = geometry()->p_topology->getEdges();

        m_pid[0] = edges[eid][0];
        m_pid[1] = edges[eid][1];
    }

    ConstraintProximityPtr getControlPoint(const int cid) {
        if (cid == 0) return createProximity(this,1,0);
        else if (cid == 1) return createProximity(this,0,1);
        return createProximity(this,1.0/2.0,1.0/2.0);
    }

    //this function project the point P on the element and return the corresponding proximity
    ConstraintProximityPtr project(Vector3 P) {
        double fact_u,fact_v;

        const ReadAccessor<Vector3> & pos = geometry()->getState()->read(VecCoordId::position());

        Vector3 P1 = pos[m_pid[0]];
        Vector3 P2 = pos[m_pid[1]];

        Vector3 v = P2-P1;
        fact_v = dot(P - P1,v) / dot(v,v);

        if (fact_v<0.0) fact_v = 0.0;
        else if (fact_v>1.0) fact_v = 1.0;

        fact_u = 1.0-fact_v;

        return createProximity(this,fact_u,fact_v);
    }

    EdgeGeometry * geometry() {
        return (EdgeGeometry * )m_geometry;
    }


    void draw(const VisualParams * /*vparams*/) {
        glColor4dv(geometry()->d_color.getValue().data());

        glBegin(GL_LINES);
            glVertex3dv(getControlPoint(0)->getPosition().data());
            glVertex3dv(getControlPoint(1)->getPosition().data());
        glEnd();
    }

protected:
    unsigned m_pid[2];
    unsigned m_eid;
};





}

