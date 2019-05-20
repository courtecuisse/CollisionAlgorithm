#pragma once

#include <sofa/collisionAlgorithm/algorithm/BaseClosestProximityAlgorithm.h>
#include <sofa/collisionAlgorithm/BroadPhase.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

#include <sofa/collisionAlgorithm/data/DataDistanceMeasure.h>

namespace sofa
{

namespace collisionAlgorithm
{

class CatheterBindingAlgorithm : public BaseClosestProximityAlgorithm
{
public:
    SOFA_CLASS(CatheterBindingAlgorithm, BaseClosestProximityAlgorithm);

    core::objectmodel::SingleLink<CatheterBindingAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_from;
    core::objectmodel::SingleLink<CatheterBindingAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_dest;
    Data<bool> d_drawCollision ;
    Data<DetectionOutput> d_output;
    Data<double> d_proj_dist;


    CatheterBindingAlgorithm()
        : l_from(initLink("from", "link to from geometry"))
        , l_dest(initLink("dest", "link to dest geometry"))
        , d_drawCollision (initData(&d_drawCollision, false, "drawcollision", "draw collision"))
        , d_proj_dist(initData(&d_proj_dist, (double) 0.0, "projDist", "Projection Distance"))
        , d_output(initData(&d_output,"output", "output of the collision detection")) {}

    void draw(const core::visual::VisualParams* /*vparams*/) {
        if (! d_drawCollision.getValue()) return;
        glDisable(GL_LIGHTING);
        glColor4f(0,1,0,1);

        glBegin(GL_LINES);
        DetectionOutput output = d_output.getValue() ;
        for (unsigned i=0;i<output.size();i++) {
            glVertex3dv(output[i].first->getPosition().data());
            glVertex3dv(output[i].second->getPosition().data());
        }
        glEnd();
    }

    void doDetection() {
        if (l_from == NULL) return;
        if (l_dest == NULL) return;

        DetectionOutput & output = *d_output.beginEdit();
        output.clear();

        for (auto itfrom=l_from->begin();itfrom!=l_from->end();itfrom++) {
            PairDetection min_pair = findClosestPointCatheter(*itfrom,l_dest.get());
            if (min_pair.first == nullptr || min_pair.second == nullptr) {
                continue;
            }

            output.add(min_pair.first,min_pair.second);
        }
        d_output.endEdit();
    }



    PairDetection findClosestPointCatheter(const BaseElementIterator *itfrom,  BaseGeometry *geo) {
        double min_dist = 5;
        BaseProximity::SPtr minprox_from = nullptr;
        BaseProximity::SPtr minprox_dest = nullptr;

        for(BaseElementIterator::UPtr itdest = getDestIterator(itfrom->center()->getPosition(),geo); // this function may create an iterator on using the bradphase if defined in the geometry
            itdest != geo->end(); itdest++)
        {
            BaseProximity::SPtr pfrom = itfrom->center();
            BaseProximity::SPtr pdest = itdest->center();

            double dist = d_distance_measure.getValue().compute(PairDetection(pfrom,pdest));

            if (dist<min_dist) {
                min_dist = dist;
                minprox_dest = pdest;
                minprox_from = pfrom;
            }
        }

        return PairDetection(minprox_from,minprox_dest);
    }




};


}

}
