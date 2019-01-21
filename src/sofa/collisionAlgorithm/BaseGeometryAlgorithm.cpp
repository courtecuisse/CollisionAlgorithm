#include <sofa/collisionAlgorithm/BaseGeometryAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseFilter.h>

namespace sofa
{

namespace collisionAlgorithm
{

bool BaseGeometryAlgorithm::acceptFilter(const BaseProximity::SPtr & pfrom,const BaseProximity::SPtr & pdest) const {
    for (auto itfilter=m_filters.cbegin();itfilter != m_filters.cend();itfilter++) {
        if (! (*itfilter)->accept(pdest,pfrom)) return false;
    }
    return true;
}

}

}
