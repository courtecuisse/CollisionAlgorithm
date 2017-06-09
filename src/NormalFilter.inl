#ifndef SOFA_COMPONENT_NORMALFILTER_INL
#define SOFA_COMPONENT_NORMALFILTER_INL

#include "NormalFilter.h"
#include <sofa/helper/Quater.h>
#include <sofa/core/visual/VisualParams.h>
#include <SofaOpenglVisual/OglModel.h>
#include <sofa/simulation/AnimateBeginEvent.h>
#include <SofaConstraint/BilateralInteractionConstraint.h>
#include <SofaBaseMechanics/MechanicalObject.h>
#include <sofa/core/visual/VisualParams.h>
#include <SofaOpenglVisual/OglModel.h>

namespace sofa {

namespace core {

namespace behavior {

NormalFilter::NormalFilter()
: d_angle(initData(&d_angle, -1.0,"angle","Draw Bbox")) {}

bool NormalFilter::filter(const ConstraintProximityPtr &from, const ConstraintProximityPtr &dst) {
    return dot(from->getNormal(),dst->getNormal())>=d_angle.getValue();
}

} //controller

} //component

}//Sofa

#endif
