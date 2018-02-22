/******************************************************************************
*             Private SOFA components, (c) 2012 INRIA                         *
* CONFIDENTIAL SOURCE CODE. This file is the property of INRIA and should not *
* be redistributed. Commercial use is prohibited without a specific license.  *
******************************************************************************/
#ifndef SOFA_COMPONENT_INTERSECTIONCONTOURGEOMETRY_H
#define SOFA_COMPONENT_INTERSECTIONCONTOURGEOMETRY_H

//#include <sofa/component/component.h>
#include "BaseGeometry.h"
#include <sofa/defaulttype/Mat.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/defaulttype/VecTypes.h>

namespace sofa {

namespace core {

namespace behavior {

class IntersectionContourGeometry : public BaseGeometry {
    friend class IntersectionProximity;

public:
        SOFA_CLASS(IntersectionContourGeometry, BaseGeometry);


        typedef defaulttype::Vector3 Vector3;
        typedef defaulttype::Rigid3dTypes::Coord Ridig;

        Data<helper::vector<Ridig> > d_planePos;

        IntersectionContourGeometry();

        virtual void prepareDetection();

        struct IntersectionEdgeInfo {
            unsigned p1,p2;
            double alpha;

            IntersectionEdgeInfo(unsigned _p1, unsigned _p2,double a) {
                p1 = _p1;
                p2 = _p2;
                alpha = a;
            }
        };

        unsigned getNbElements() const;

        ConstraintElementPtr getElement(unsigned i) const;

        void draw(const core::visual::VisualParams * vparams);

    protected:

        helper::vector<IntersectionEdgeInfo> m_intersection;
        helper::vector<unsigned> m_crs; // compressed row sparse

};

} // namespace helpMeSee

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_HELPMESEEPLUGIN_IntersectionContourGeometry_H
