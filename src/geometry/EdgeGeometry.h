/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#pragma once

#include <geometry/PointGeometry.h>

namespace collisionAlgorithm {

class EdgeGeometry : public BaseGeometry {
    friend class EdgeElement;
public:
    void prepareDetection();
};

class EdgeElement : public ConstraintElement {
    friend class EdgeProximity;
    friend class EdgeGeometry;

public:

    EdgeElement(EdgeGeometry *geo, unsigned pid);

    ConstraintProximityPtr getControlPoint(const int i);

    ConstraintProximityPtr project(Vector3 /*P*/);

    void draw(const std::vector<Vector3> & X);

protected:
    unsigned m_pid[2];
    unsigned m_eid;
};

class EdgeProximity : public ConstraintProximity {
public :
    EdgeProximity(EdgeElement *geo,double f1,double f2);

    Vector3 getPosition(TVecId v) const;

    Vector3 getNormal() const;

    std::map<unsigned,Vector3> getContribution(const Vector3 & N);

    inline EdgeElement * element() const {
        return (EdgeElement*) m_element;
    }

protected:
    double m_fact[2];
};

}

