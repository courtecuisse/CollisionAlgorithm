#pragma once

#include <Collision.h>
#include <decorator/AABBDecorator.h>

namespace sofa {

namespace collisionAlgorithm {

class CollisionDetectionAlgorithm : public Collision {
public:

    Data<std::string> p_from;
    Data<std::string> p_dest;

    CollisionDetectionAlgorithm();

    void processAlgorithm();

    bool canCreate() {
        m_from = getContext()->get<BaseGeometry>(p_from.getValue());
        m_dest = getContext()->get<BaseGeometry>(p_dest.getValue());

        return m_from && m_dest;
    }

private:

    template<class ElementIterator>
    PairProximity getClosestPoint(ElementIterator geo);

    BaseGeometry * m_from;
    BaseGeometry * m_dest;
};

}

}
