#pragma once

#include <sofa/collisionAlgorithm/algorithm/PointCloudBindingAlgorithm.h>

namespace sofa
{

namespace collisionAlgorithm
{

PointCloudBindingAlgorithm::PointCloudBindingAlgorithm()
: d_maxDist(initData(&d_maxDist, std::numeric_limits<double>::min(), "maxDist", "Maximum distance"))
, d_output(initData(&d_output, "output" , "this"))
, d_from(initData(&d_from, "from" , "this"))
, d_dest(initData(&d_dest, "dest" , "this")) {}

void PointCloudBindingAlgorithm::computeCollisionDetection()
{
    std::pair<BaseProximity::SPtr,BaseProximity::SPtr> res;

//    if (d_from.getValue().end() == nullptr)
//        return ;
//    if (d_dest.getValue().end() == nullptr)
//        return ;

    if (d_from.getValue().begin() != d_dest.getValue().end()) return ;
    if (d_dest.getValue().begin() != d_dest.getValue().end()) return ;

    helper::vector<BaseProximity::SPtr> p1;
    helper::vector<BaseProximity::SPtr> p2;

    for (auto it = d_from.getValue().begin(); it != d_from.getValue().end();it++)
    {
        p1.push_back(it->center());
    }

    for (auto it = d_dest.getValue().begin(); it != d_dest.getValue().end();it++)
    {
        p2.push_back(it->center());
    }

    helper::vector<int> bindId;
    bindId.resize(p1.size(),-1);

    bool change = true;

    helper::vector<int> invBind;
    invBind.resize(p2.size(),-1);

    const double& maxDist = d_maxDist.getValue();

    while (change)
    {
//        printf("NEW LOOP\n");
//        std::cout << "BIND=" << bindId << std::endl;
//        std::cout << "IBIND=" << invBind << std::endl;
        change = false;//ne new change

        for (unsigned p=0;p<p1.size();p++) {
            if (bindId[p] != -1) continue;

            Vector3 P = p1[p]->getPosition();
            int closestId = -1;
            double closestDist = std::numeric_limits<double>::max();

            //Find minimal distance
            for (unsigned i=0;i<p2.size();i++)
            {
                Vector3 Q = p2[i]->getPosition();
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

                double d1 = (p1[A]->getPosition() - p2[C]->getPosition()).norm();
                double d2 = (p1[B]->getPosition() - p2[C]->getPosition()).norm();

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


    DetectionOutput * output = d_output.beginEdit();

    for (unsigned i=0;i<bindId.size();i++)
    {
        if (bindId[i] == -1 || invBind[i] == -1)
            continue;

        std::pair<BaseProximity::SPtr,BaseProximity::SPtr> pair;
        pair.first = p1[bindId[i]];
        pair.second = p2[invBind[i]];

        output->add(pair.first,pair.second);
    }

    d_output.endEdit();

}

void PointCloudBindingAlgorithm::computeCollisionReset() {
    d_output.beginEdit()->clear();
    d_output.endEdit();
}

} // namespace collisionAlgorithm

} // namespace sofa
