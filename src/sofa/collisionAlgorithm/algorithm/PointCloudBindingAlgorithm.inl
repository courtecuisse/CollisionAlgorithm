#pragma once

#include <sofa/collisionAlgorithm/algorithm/PointCloudBindingAlgorithm.h>

namespace sofa
{

namespace collisionAlgorithm
{

PointCloudBindingAlgorithm::PointCloudBindingAlgorithm()
: d_maxDist(initData(&d_maxDist, std::numeric_limits<double>::min(), "maxDist", "Maximum distance")) {}

void PointCloudBindingAlgorithm::bind(const std::vector<defaulttype::Vector3> & p1, const std::vector<defaulttype::Vector3> & p2, helper::vector<int> & bindId, helper::vector<int> & invBind, double maxDist) {
    bindId.resize(p1.size(),-1);
    invBind.resize(p2.size(),-1);

    bool change = true;



    while (change)
    {
//        printf("NEW LOOP\n");
//        std::cout << "BIND=" << bindId << std::endl;
//        std::cout << "IBIND=" << invBind << std::endl;
        change = false;//ne new change

        for (unsigned p=0;p<p1.size();p++) {
            if (bindId[p] != -1) continue;

            Vector3 P = p1[p];
            int closestId = -1;
            double closestDist = std::numeric_limits<double>::max();

            //Find minimal distance
            for (unsigned i=0;i<p2.size();i++)
            {
                Vector3 Q = p2[i];
                double dist = (Q-P).norm();

                if (dist < closestDist && invBind[i] == -1)
                {
                    closestId = i;
                    closestDist = dist;
//                    printf("TMP ASSO p<->i %d %d\n",p,i);
                }
            }

//            printf("MIN DIST %f    p=%f    %d\n",closestDist,minDist,closestId);

            if ((maxDist > std::numeric_limits<double>::epsilon()) && (closestDist > maxDist))
                closestId = -1;

//            printf("CLOSEST ID %d\n",closestId);

            bindId[p] = closestId;
        }

        //try to remove point that are binded multiple times to the same one
        for (unsigned i=0;i<p1.size();i++)
        {
            if (bindId[i] == -1) continue;

            int & B = bindId[i]; //current point in p1
            int & C = invBind[bindId[i]]; // previous binded point in p2

            if (C == -1) // the point has not yet been associated
            {
                C = i;
            }
            else if (C != (int) i) // the point is already associated
            {
                int & A = bindId[C]; // previous binded point in p1

                change = true; // we retry the binding because two points are associated with the same point

                double d1 = (p1[A] - p2[C]).norm();
                double d2 = (p1[B] - p2[C]).norm();

                if (d2<d1)
                {
                    A = -1; // invalidate A
                    C = i; // change the invbinding
                }
                else
                {
                    B = -1; // invalidate A
                }
            }
        }
    }
}

void PointCloudBindingAlgorithm::processAlgorithm(BaseElementIterator::UPtr itfrom, const BaseGeometry * g2, helper::vector< PairDetection > & output) {
    helper::vector<defaulttype::Vector3> p1;
    helper::vector<defaulttype::Vector3> p2;

    helper::vector<BaseProximity::SPtr> prox1;
    helper::vector<BaseProximity::SPtr> prox2;

    while (! itfrom->end()) {
        BaseProximity::SPtr center = (*itfrom)->center();
        prox1.push_back(center);
        p1.push_back(center->getPosition());
        itfrom++;
    }

    for (auto itdest = g2->begin(); itdest != g2->end();itdest++) {
        BaseProximity::SPtr center = (*itdest)->center();
        prox2.push_back(center);
        p2.push_back(center->getPosition());
    }

    helper::vector<int> bindId;
    helper::vector<int> invBind;
    bind(p1,p2,bindId,invBind,d_maxDist.getValue());

    for (unsigned i=0;i<bindId.size();i++)
    {
        if (bindId[i] == -1 || invBind[i] == -1)
            continue;

        std::pair<BaseProximity::SPtr,BaseProximity::SPtr> pair;
        pair.first = prox1[bindId[i]];
        pair.second = prox2[invBind[i]];

        output.push_back(PairDetection(pair.first,pair.second));
    }
}

} // namespace collisionAlgorithm

} // namespace sofa
