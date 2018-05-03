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


void CollisionAlgorithm::totalDistanceBind(const helper::vector<defaulttype::Vector3> & p1, const helper::vector<defaulttype::Vector3> & p2 , helper::vector<int> & bindId, double minDist) {

   bindId.resize(p1.size(),-1);
    helper::vector<int> bindId1(bindId);
    helper::vector<int> bindId2(bindId);

    if (p1.empty()) return;
    if (p2.empty()) return;

    bool change1 = true;
    bool change2 = true;
    helper::vector<int> invBind1;
    helper::vector<int> invBind2;
    invBind1.resize(p2.size(),-1);
    invBind2.resize(p2.size(),-1);

    helper::vector<double> totaldistp1;
    totaldistp1.resize(p1.size(),0);
    helper::vector<double> totaldistp2;
    totaldistp2.resize(p2.size(),0);

    for (unsigned p=0;p<p1.size();p++) {
        for (unsigned q=0;q<p1.size();q++) {
            defaulttype::Vector3 P=p1[p];
            defaulttype::Vector3 Q=p1[q];
            totaldistp1[p]+=(P-Q).norm();
        }
        totaldistp1[p]/=p1.size();
    }

    for (unsigned p=0;p<p2.size();p++) {
        for (unsigned q=0;q<p2.size();q++) {
            defaulttype::Vector3 P=p2[p];
            defaulttype::Vector3 Q=p2[q];
            totaldistp2[p]+=(P-Q).norm();
        }
        totaldistp2[p]/=p2.size();
    }
    while (change1) {
        change1 = false;

        for (unsigned p=0;p<p1.size();p++) {
            if (bindId1[p] != -1) continue;

            int closestId = -1;
            double closestDist = std::numeric_limits<double>::max();

            //Find minimal distance
            for (unsigned i=0;i<p2.size();i++) {
  
                double dist = (totaldistp1[p]-totaldistp2[i])*(totaldistp1[p]-totaldistp2[i]);

                if (dist < closestDist && invBind1[i] == -1) {
                    closestId = i;
                    closestDist = dist;
                }
            }


            bindId1[p] = closestId;
        }

        for (unsigned i=0;i<p1.size();i++) {
            if (bindId1[i] == -1) continue;

            int & B = bindId1[i]; //current point in p1
            int & C = invBind1[bindId1[i]]; // previous binded point in p2

            if (C == -1) { // the point has not yet been associated
                C = i;
            } else if (C != (int) i) { // the point is already associated
                int & A = bindId1[C]; // previous binded point in p1

                change1 = true; // we retry the binding because two points are associated with the same point

                double d1 = (totaldistp1[A] - totaldistp2[C])*(totaldistp1[A] - totaldistp2[C]);
                double d2 = (totaldistp1[B] - totaldistp2[C])*(totaldistp1[B] - totaldistp2[C]);

                if (d2<d1) {
                    A = -1; // invalidate A
                    C = i; // change the invbinding
                } else {
                    B = -1; // invalidate A
                }
            }
        }
    }

    while (change2) {
//        printf("NEW LOOP\n");
//        std::cout << "BIND=" << bindId << std::endl;
//        std::cout << "IBIND=" << invBind << std::endl;
        change2 = false;//ne new change

        for (unsigned p=0;p<p1.size();p++) {
            if (bindId2[p] != -1) continue;

            defaulttype::Vector3 R = p1[p];
            int closestId = -1;
            double closestDist = std::numeric_limits<double>::max();

            //Find minimal distance
            for (unsigned i=0;i<p2.size();i++) {
                defaulttype::Vector3 S = p2[i];
                double dist = (S-R).norm();

                if (dist < closestDist && invBind2[i] == -1) {
                    closestId = i;
                    closestDist = dist;
//                    printf("TMP ASSO p<->i %d %d\n",p,i);
                }
            }

//            printf("MIN DIST %f    p=%f    %d\n",closestDist,minDist,closestId);


            if ((minDist!=0) && (closestDist > minDist)) {
                closestId = -1;
            }
//            printf("CLOSEST ID %d\n",closestId);

            bindId2[p] = closestId;
        }

        //try to remove point that are binded multiple times to the same one
        for (unsigned i=0;i<p1.size();i++) {
            if (bindId2[i] == -1) continue;

            int & B = bindId2[i]; //current point in p1
            int & C = invBind2[bindId2[i]]; // previous binded point in p2

            if (C == -1) { // the point has not yet been associated
                C = i;
            } else if (C != (int) i) { // the point is already associated
                int & A = bindId2[C]; // previous binded point in p1

                change2 = true; // we retry the binding because two points are associated with the same point

                double d1 = (p1[A] - p2[C]).norm();
                double d2 = (p1[B] - p2[C]).norm();

                if (d2<d1) {
                    A = -1; // invalidate A
                    C = i; // change the invbinding
                } else {
                    B = -1; // invalidate A
                }
            }
        }
    }
    double error1=0;
    double Nb1=0;
    double error2=0;
    double Nb2=0;
    for (unsigned i=0;i<p1.size();i++) {
        if (bindId1[i]!=-1) {
            defaulttype::Vector3 T = p1[i];
            defaulttype::Vector3 U = p2[bindId1[i]];
            error1=error1+ (T-U).norm();
            Nb1=Nb1+1;
        }
        if (bindId2[i]!=-1) {
            defaulttype::Vector3 T = p1[i];
            defaulttype::Vector3 U = p2[bindId2[i]];
            error2=error2 + (T-U).norm();
            Nb2=Nb2+1;
        }
    }
    if (Nb1>0)
        error1=error1/Nb1;
    if (Nb2>0)
        error2=error2/Nb2;



  // if (error1/3<error2) {
        bindId=bindId1;
//        std::cout << "totaldistance " << error1+1 << " closestpoint "<< error2 << std::endl;
//    }
//   if (error1/3>=error2) {
//       bindId=bindId2;
//       std::cout << "totaldistance " << error1 << " closestpoint "<< error2+1 << std::endl;
//   }


}

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_CONTROLLER_NeedleConstraint_H
