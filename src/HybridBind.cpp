#ifndef SOFA_COMPONENT_CONSTRAINT_FINDCLOSESTALGO_H
#define SOFA_COMPONENT_CONSTRAINT_FINDCLOSESTALGO_H

#include "CollisionAlgorithm.h"
#include "ConstraintProximity.h"
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <math.h>
#include <sofa/defaulttype/Vec.h>
#include "AABBDecorator.h"

namespace sofa {

namespace core {

namespace behavior {


void CollisionAlgorithm::HybridBind(const helper::vector<defaulttype::Vector3> & p1, const helper::vector<defaulttype::Vector3> & p2 , helper::vector<int> & bindId, double coeff, double minDist, double minScore) {
    bindId.clear();
    bindId.resize(p1.size(),-1);

    if (p1.empty()) return;
    if (p2.empty()) return;

    bool change = true;

    helper::vector<int> invBind;
    invBind.resize(p2.size(),-1);

    helper::vector<double> totaldistp1;
    totaldistp1.resize(p1.size(),0);
    helper::vector<double> totaldistp2;
    totaldistp2.resize(p2.size(),0);

    helper::vector<double> hybrid_score;
    hybrid_score.resize(p1.size());

    for (unsigned p=0;p<p1.size();p++) {
        for (unsigned q=0;q<p1.size();q++) {
            defaulttype::Vector3 P=p1[p];
            defaulttype::Vector3 Q=p1[q];
            totaldistp1[p]+=(P-Q).norm();
        }
        totaldistp1[p]/=(double)p1.size();
    }

    for (unsigned p=0;p<p2.size();p++) {
        for (unsigned q=0;q<p2.size();q++) {
            defaulttype::Vector3 P=p2[p];
            defaulttype::Vector3 Q=p2[q];
            totaldistp2[p]+=(P-Q).norm();
        }
        totaldistp2[p]/=(double)p2.size();
    }

    while (change) {
        change = false;

        for (unsigned p=0;p<p1.size();p++) {
            if (bindId[p] != -1) continue;

            int closestId = -1;
            defaulttype::Vector3 P = p1[p];
            double min_hybrid = std::numeric_limits<double>::max();
            //Find minimal distance
            for (unsigned i=0;i<p2.size();i++) {
                double score = (totaldistp1[p]-totaldistp2[i])*(totaldistp1[p]-totaldistp2[i])*1000;
                double dist = (p2[i]-P).norm();

                if (score<minScore || dist<minDist) {
                    double internal_coeff = coeff;
//                    if (score>=minScore) internal_coeff = 1.0;
//                    else if (dist>=minDist) internal_coeff = 0.0;

                    double hybrid = internal_coeff * dist + (1.0-internal_coeff) * score;
                    if (hybrid < min_hybrid && invBind[i] == -1) {
                        closestId = i;
                        min_hybrid = hybrid;
                    }
                }
            }

//            if ((minDist!=0) && (minDist > min_hybrid)) closestId = -1;

            hybrid_score[p] = min_hybrid;
            bindId[p] = closestId;
        }

        //try to remove point that are binded multiple times to the same one
        for (unsigned i=0;i<p1.size();i++) {
            if (bindId[i] == -1) continue;

            int & B = bindId[i]; //current point in p1
            int & C = invBind[bindId[i]]; // previous binded point in p2

            if (C == -1) { // the point has not yet been associated
                C = i;
            } else if (C != (int) i) { // the point is already associated
                int & A = bindId[C]; // previous binded point in p1

                change = true; // we retry the binding because two points are associated with the same point

                if (hybrid_score[B]<hybrid_score[A]) {
                    A = -1; // invalidate A
                    C = i; // change the invbinding
                } else {
                    B = -1; // invalidate B
                }
            }
        }
    }

//    std::cout << "BIND=" << bindId << std::endl;
//    std::cout << "IBIND=" << invBind << std::endl;
}

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
