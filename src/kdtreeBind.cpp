#include "CollisionAlgorithm.h"
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/core/behavior/BaseController.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <math.h>
#include <sofa/defaulttype/Vec.h>

namespace sofa {

namespace core {

namespace behavior {

void treeBranch(const helper::vector<defaulttype::Vector3> & pointCloud, helper::vector<int> & branch, helper::vector<int> & tree, int axis ) {

    helper::vector<int> branchinf(branch.size()/2-1,0);
    helper::vector<int> branchsup(branch.size()-branchinf.size()-1,0);
    helper::vector<double> coordAxis1(branch.size(),0);
    int medianIndex;

    if (axis==0) { //trie selon x
        for (unsigned i=0;i<branch.size();i++) {
            coordAxis1[i]=pointCloud[branch[i]].x();
        }
        for (unsigned i=0;i<branch.size()/2;i++) {
            int minIndex=i;
            double min=coordAxis1[i];
            for (unsigned j=i;j<branch.size();i++) {
                if (coordAxis1[j]<min) {
                    min=coordAxis1[j];
                    minIndex=j;
                }
            }
            coordAxis1[minIndex]=std::numeric_limits<double>::max();
            medianIndex=minIndex;
        }
        for (unsigned i=0;i<branch.size();i++) {
            coordAxis1[i]=pointCloud[branch[i]].x();
        }
    }

    if (axis==1) { //trie selon y
        for (unsigned i=0;i<branch.size();i++) {
            coordAxis1[i]=pointCloud[branch[i]].y();
        }
        for (unsigned i=0;i<branch.size()/2;i++) {
            int minIndex=i;
            double min=coordAxis1[i];
            for (unsigned j=i;j<branch.size();i++) {
                if (coordAxis1[j]<min) {
                    min=coordAxis1[j];
                    minIndex=j;
                }
            }
            coordAxis1[minIndex]=std::numeric_limits<double>::max();
            medianIndex=minIndex;
        }
        for (unsigned i=0;i<branch.size();i++) {
            coordAxis1[i]=pointCloud[branch[i]].y();
        }
    }

    if (axis==2) { //trie selon z
        for (unsigned i=0;i<branch.size();i++) {
            coordAxis1[i]=pointCloud[branch[i]].z();
        }
        for (unsigned i=0;i<branch.size()/2;i++) {
            int minIndex=i;
            double min=coordAxis1[i];
            for (unsigned int j=i;j<branch.size();i++) {
                if (coordAxis1[j]<min) {
                    min=coordAxis1[j];
                    minIndex=j;
                }
            }
            coordAxis1[minIndex]=std::numeric_limits<double>::max();
            medianIndex=minIndex;
        }
        for (unsigned i=0;i<branch.size();i++) {
            coordAxis1[i]=pointCloud[branch[i]].z();
        }
    }

    if (axis<2) {
        axis=axis+1;
    } else {
        axis=0;
    }
    tree.push_back(branch[medianIndex]);

    unsigned counter1=0;
    unsigned counter2=0;
    while (counter1<branchinf.size()) {
        if (coordAxis1[counter2]<coordAxis1[medianIndex]) {
            branchinf[counter1]=branch[counter2];
            counter1=counter1+1;
        }
        counter2=counter2+1;
    }

    counter1=0;
    counter2=0;
    while (counter1<branchsup.size()) {
        if (coordAxis1[counter2]>coordAxis1[medianIndex]) {
            branchsup[counter1]=branch[counter2];
            counter1=counter1+1;
        }
        counter2=counter2+1;
    }
    helper::vector<int>().helper::vector<int>::swap(branch);
    helper::vector<double>().helper::vector<double>::swap(coordAxis1);
    if (branchinf.size()>1)
        treeBranch(pointCloud, branchinf, tree, axis );
    if (branchinf.size()==1)
        tree.push_back(branchinf[0]);
    if (branchsup.size()>1)
        treeBranch(pointCloud, branchsup, tree, axis );
    if (branchsup.size()==1)
        tree.push_back(branchsup[0]);
}





void CollisionAlgorithm::kdtreeBind(const helper::vector<defaulttype::Vector3> & p1, const helper::vector<defaulttype::Vector3> & p2 , helper::vector<int> & bindId, double minDist) {

    bindId.resize(p1.size(),-1);

    if (p1.empty()) return;
    if (p2.empty()) return;

    bool change = true;


    helper::vector<int> invBind;
    invBind.resize(p2.size(),-1);

    helper::vector<int> branchp1(p1.size(),0);
    helper::vector<int> branchp2(p2.size(),0);
    helper::vector<int> treep1;
    helper::vector<int> treep2;
    for (unsigned i=0;i<p1.size();i++)
        branchp1[i]=i;
    for (unsigned i=0;i<p2.size();i++)
        branchp2[i]=i;

    treeBranch(p1,branchp1,treep1,0);
    treeBranch(p2,branchp2,treep2,0);
    if (p1.size()<p2.size())
        
            
    while(change) {
        change=false;

    
        for (unsigned p=0;p<p1.size();p++) {
            defaulttype::Vector3 P=p1[treep1[p]];
            defaulttype::Vector3 Q=p2[treep2[p]];
            if ((minDist!=0) && ((Q-P).norm()>minDist)) {
                bindId[treep1[p]]=-1;
            } else {
                bindId[treep1[p]]=treep2[p];
            }
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
}

//    std::cout << "BIND=" << bindId << std::endl;
//    std::cout << "IBIND=" << invBind << std::endl;


} // namespace controller

} // namespace component

} // namespace sofa
