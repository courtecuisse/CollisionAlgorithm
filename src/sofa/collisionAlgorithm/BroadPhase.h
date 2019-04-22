#pragma once

#include <sofa/core/collision/Pipeline.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseElementContainer.h>

namespace sofa {

namespace collisionAlgorithm {

/*!
 * \brief The BroadPhase class defines an interface for the collision detection broad phase
 */
class BroadPhase : public core::objectmodel::BaseObject
{
public:
    SOFA_ABSTRACT_CLASS(BroadPhase,core::objectmodel::BaseObject);

    Data<defaulttype::Vector4> d_color;
    core::objectmodel::SingleLink<BroadPhase,BaseElementContainer,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_DATALINK> l_elements;

    /*!
     * \brief BroadPhase Constructor
     */
    BroadPhase()
    : d_color(initData(&d_color, defaulttype::Vector4(1,0,1,1), "color", "Color of the collision model"))
    , l_elements(initLink("elements", "link to state")) {}

    /*!
     * \brief ~BroadPhase destructor
     */
    virtual ~BroadPhase() {
        if (l_elements != NULL) l_elements->setBroadPhase(NULL);
    }

    void init( ) override {
        if (l_elements != NULL) {
            sout << "Register to geometry " << l_elements->getOwner()->getName() << sendl;
            l_elements->setBroadPhase(this);
        }
        else serr << "No geometry found" << sendl;
    }

    /*!
     * \brief prepareDetection virtual method to implement
     * detection pre-processing
     */
    virtual void prepareDetection() = 0;

    /*!
     * \brief getBoxSize
     * \return bounding box size in a vec3i
     */
    virtual defaulttype::Vec3i getBoxSize() const = 0;

    /*!
     * \brief getBoxCoord
     * \param P : point in space
     * \return the box's coordinates (vec3i) containing point P
     */
    virtual defaulttype::Vec3i getBoxCoord(const defaulttype::Vector3 & P) const = 0;

    virtual void getElementSet(unsigned cx,unsigned cy, unsigned cz, std::set<unsigned> & selectElements) const = 0;

    bool findDataLinkDest(BaseElementContainer *& ptr, const std::string& path, const core::objectmodel::BaseLink* link)
    {
        core::objectmodel::BaseData* base = NULL;
        if (!this->getContext()->findDataLinkDest(base, path, link)) return false;
        ptr = dynamic_cast<BaseElementContainer*>(base);
        return (ptr != NULL);
    }

};


}

}
