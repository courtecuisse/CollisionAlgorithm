#pragma once

#include <sofa/collisionAlgorithm/geometry/EdgeGeometry.h>
#include <sofa/collisionAlgorithm/element/EdgeElement.h>
#include <sofa/collisionAlgorithm/proximity/EdgeProximity.h>

namespace sofa
{

namespace collisionAlgorithm
{

ConstraintProximity::SPtr EdgeGeometry::createProximity(const EdgeElement * elmt,double f1,double f2) const
{
    return std::shared_ptr<EdgeProximity>(new EdgeProximity(elmt, f1, f2));
}


void EdgeGeometry::init()
{
    if(d_edges.getValue().empty())
    {
        msg_error(this) << "Edges are not set (data is empty). Giving up.";
    }

    m_elements.clear();

    const VecEdges& edges = d_edges.getValue();
    for (size_t i=0;i<edges.size();i++)
    {
        m_elements.push_back(EdgeElement::createElement(this,i));
    }
}

void EdgeGeometry::prepareDetection()
{
    const VecEdges& edges = d_edges.getValue();
    if (m_elements.size() != edges.size())
        init();
}

}

}
