#include <Collision.h>
#include <qopengl.h>

namespace collisionAlgorithm {

Collision::Collision()
: p_out("out",this, this) {
    m_dirty = true;
}

}
