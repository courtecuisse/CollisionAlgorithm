#include "CollisionAlgorithm.h"
#include <sofa/core/behavior/MechanicalState.h>
#include <math.h>

namespace sofa {

namespace collisionAlgorithm {

#ifdef PCL_ENABLE

void CollisionAlgorithm::KdTreeClosestPoint(const helper::vector<defaulttype::Vector3> & p1, const helper::vector<defaulttype::Vector3> & p2 , helper::vector<int> & bindId, double minDist) {

    bindId.resize(p1.size(),-1);

    if (p1.empty()) return;
    if (p2.empty()) return;

    bool change = true;

    helper::vector<int> invBind;
    invBind.resize(p2.size(),-1);

    /*pcl cloud data initialisation*/

    //registration through kdtree algorithm

    pcl::PointCloud<pcl::PointXYZ>::Ptr p1cloudconst (new pcl::PointCloud<pcl::PointXYZ>);
    p1cloudconst->width = p1.size();
    p1cloudconst->height = 1;
    p1cloudconst->points.resize (p1cloudconst->width * p1cloudconst->height);
    for (size_t i = 0; i < p1.size(); ++i)
    {
        //p1cloud.push_back(pcl::PointXYZ(p1[i].x(),p1[i].y(), p1[i].z() ));

        p1cloudconst->points[i].x = p1[i].x();
        p1cloudconst->points[i].y = p1[i].y();
        p1cloudconst->points[i].z = p1[i].z();

    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr p2cloudconst (new pcl::PointCloud<pcl::PointXYZ>);
    p2cloudconst->width = p2.size();
    p2cloudconst->height = 1;
    p2cloudconst->points.resize (p2cloudconst->width * p2cloudconst->height);
    for (size_t i = 0; i < p2.size(); ++i)
    {
        //p1cloud.push_back(pcl::PointXYZ(p1[i].x(),p1[i].y(), p1[i].z() ));

        p2cloudconst->points[i].x = p2[i].x();
        p2cloudconst->points[i].y = p2[i].y();
        p2cloudconst->points[i].z = p2[i].z();

    }
    //pcl::PointCloud<pcl::PointXYZ>::Ptr p1cloudconst(&p1cloud);
    pcl::registration::CorrespondenceEstimation<pcl::PointXYZ, pcl::PointXYZ>::Ptr KdtreeEstimator;

    KdtreeEstimator->setInputSource(p1cloudconst);
    std::cout << "RAMISTAR" << std::endl;
    KdtreeEstimator->setInputTarget(p2cloudconst);
    int _index_query;
    int _index_match;
    float _distance;
    pcl::Correspondence reciprocalBinding(_index_query, _index_match, _distance);
    //KdtreeEstimator->determineReciprocalCorrespondences (all_correspondences);


    while (change) {
//        printf("NEW LOOP\n");
//          std::cout << "BIND=" << bindId << std::endl;
//        std::cout << "IBIND=" << invBind << std::endl;
        change = false;//ne new change

        for (unsigned p=0;p<p1.size();p++) {
            if (bindId[p] != -1) continue;

            defaulttype::Vector3 P = p1[p];
            int closestId = -1;
            double closestDist = std::numeric_limits<double>::max();

            //Find minimal distance
            for (unsigned i=0;i<p2.size();i++) {
                defaulttype::Vector3 Q = p2[i];
                double dist = (Q-P).norm();

                if (dist < closestDist && invBind[i] == -1) {
                    closestId = i;
                    closestDist = dist;
//                    printf("TMP ASSO p<->i %d %d\n",p,i);
                }
            }

//            printf("MIN DIST %f    p=%f    %d\n",closestDist,minDist,closestId);

            if ((minDist!=0) && (closestDist > minDist)) closestId = -1;

//            printf("CLOSEST ID %d\n",closestId);

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

//    std::cout << "BIND=" << bindId << std::endl;
//    std::cout << "IBIND=" << invBind << std::endl;
}
#endif

}

}
