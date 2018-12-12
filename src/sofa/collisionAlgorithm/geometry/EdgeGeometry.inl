#pragma once

#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>
#include <sofa/collisionAlgorithm/element/EdgeElement.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

ConstraintProximity::SPtr EdgeGeometry::createProximity(const EdgeElement * elmt,double f1,double f2)
{
    return std::shared_ptr<EdgeProximity>(new EdgeProximity(elmt, f1, f2));
}


void EdgeGeometry::init()
{
    m_elements.clear();

    for (unsigned i=0;i<l_topology->getNbEdges();i++)
    {
        m_elements.push_back(EdgeElement::createElement(this,i));
    }
}

void EdgeGeometry::prepareDetection()
{
    if (m_elements.size() != l_topology->getNbEdges())
        init();
}

}

}
