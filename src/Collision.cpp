#include <Collision.h>
#include <qopengl.h>

namespace collisionAlgorithm {

Collision::Collision()
: p_type("out",this) {
    m_dirty = true;
}

}
