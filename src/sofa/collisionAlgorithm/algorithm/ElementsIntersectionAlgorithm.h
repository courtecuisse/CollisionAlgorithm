#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/operations/Intersect.h>
#include <sofa/collisionAlgorithm/operations/Project.h>

namespace sofa::collisionAlgorithm {

 //Specific operation to find the intersection of an element with a geometry (the code is in the c++ class)
class IntersectWithGeometryOperation : public Operations::GenericOperation<IntersectWithGeometryOperation,
        std::function<void (BaseElement* , BaseGeometry *, std::vector<BaseElement::SPtr> &) > > {
public:

    using Inherit = GenericOperation;

    GenericOperation::FUNC getDefault() const override;
};



class ElementsIntersectionAlgorithm : public BaseAlgorithm {
public:
    SOFA_CLASS(ElementsIntersectionAlgorithm, BaseAlgorithm);

    core::objectmodel::SingleLink<ElementsIntersectionAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_from;
    core::objectmodel::SingleLink<ElementsIntersectionAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_dest;
    Data<bool> d_drawCollision ;
//    Data<sofa::type::vector<BaseElement::SPtr> > d_output;
    Data<sofa::type::vector<double> > d_outputDist;

    ElementsIntersectionAlgorithm()
    : l_from(initLink("from", "link to from geometry"))
    , l_dest(initLink("dest", "link to dest geometry"))
    , d_drawCollision (initData(&d_drawCollision, true, "drawcollision", "draw collision"))/*
    , d_ou tput(initData(&d_output,"output", "output of the collision detection"))*/
    , d_outputDist(initData(&d_outputDist,"outputDist", "Distance of the outpu pair of detections"))
    {}

    void draw(const core::visual::VisualParams* vparams) {
//        if (! vparams->displayFlags().getShowCollisionModels() && ! d_drawCollision.getValue()) return;
//        glDisable(GL_LIGHTING);
//        glColor4f(0,1,0,1);

//        glBegin(GL_LINES);
//        std::vector<BaseElement::SPtr> output = d_output.getValue() ;
//        for (unsigned i=0;i<output.size();i++) {
////            glVertex3dv(output[i].first->getPosition().data());
////            glVertex3dv(output[i].second->getPosition().data());
//            output[i]->draw(vparams);
//        }
//        glEnd();
    }



    void doDetection() {
//        if (l_from == NULL) return;
//        if (l_dest == NULL) return;

//        std::vector<BaseElement::SPtr> & output = *d_output.beginEdit();
//        output.clear();

//        auto IntersectOp = IntersectWithGeometryOperation::get(l_dest->begin()->getOperationsHash());
//        std::vector<BaseElement::SPtr> intersection;

//        for (auto itfrom=l_from->begin();itfrom!=l_from->end();itfrom++) {

//            intersection.clear();
//            IntersectOp(itfrom->element(), l_dest.get(), intersection);
//            if (intersection.size() == 0) continue;


//            //  /////// Fill outputDetection !!! ///////   Be careful with the type of output
//            //  /!\ fill outputDetection with all the elements from the "intersection" vector --> browse it
////            output.push_back(///////);


//        }

//        d_output.endEdit();
    }

};

}

