#pragma once

#include <Collision.h>
#include <qopengl.h>

namespace collisionAlgorithm {

Collision::Collision()
: p_type("out",LEFT,this) {
    m_dirty = true;
}

}

CONNECTABLE(collisionAlgorithm::Collision)
