#pragma once

#include <sofa/collisionAlgorithm/algorithm/BaseClosestProximityAlgorithm.h>

namespace sofa {

namespace collisionAlgorithm {

class AABBSearchMethod : public sofa::core::objectmodel::BaseObject
{
public:
    SOFA_CLASS(AABBSearchMethod, sofa::core::objectmodel::BaseObject);

    core::objectmodel::SingleLink<AABBSearchMethod,BaseClosestProximityAlgorithm, BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_algorithm;

    AABBSearchMethod()
    : l_algorithm(initLink("algo","Link to the algorithm to optimize")) {
        l_algorithm.setPath(("@."));
    }

    void init() {
        if (l_algorithm!=NULL) {
            l_algorithm->setSearchMethod(std::bind(&AABBSearchMethod::findClosestProximity,this,std::placeholders::_1,std::placeholders::_2));
            l_algorithm->addSlave(this);
        }
    }

    void fillElementSet(const BaseGeometry::BroadPhase::SPtr decorator, defaulttype::Vec3i cbox, std::set<unsigned> & selectElements, int d) const {
        defaulttype::Vec3i nbox = decorator->getBoxSize();

        {
            int i=-d;
            if (cbox[0]+i >= 0 && cbox[0]+i < nbox[0])
            {
                for (int j=-d; j <= d; j++)
                {
                    if (cbox[1]+j < 0 || cbox[1]+j >= nbox[1])
                        continue;
                    for (int k=-d;k<=d;k++)
                    {
                        if (cbox[2]+k < 0 || cbox[2]+k >= nbox[2])
                            continue;

                        decorator->getElementSet(defaulttype::Vec3i(cbox[0] + i,cbox[1] + j,cbox[2] + k), selectElements);
                    }
                }
            }
        }

        {
            int i=d;
            if (cbox[0]+i >= 0 && cbox[0]+i < nbox[0])
            {
                for (int j=-d;j<=d;j++)
                {
                    if (cbox[1]+j < 0 || cbox[1]+j >= nbox[1])
                        continue;

                    for (int k=-d;k<=d;k++)
                    {
                        if (cbox[2]+k < 0 || cbox[2]+k >= nbox[2])
                            continue;

                        decorator->getElementSet(defaulttype::Vec3i(cbox[0] + i,cbox[1] + j,cbox[2] + k), selectElements);
                    }
                }
            }
        }


        {
            int j=-d;
            if (cbox[1]+j >= 0 && cbox[1]+j < nbox[1])
            {
                for (int i=-d+1;i<d;i++)
                {
                    if (cbox[0]+i < 0 || cbox[0]+i >= nbox[0])
                        continue;

                    for (int k=-d;k<=d;k++)
                    {
                        if (cbox[2]+k < 0 || cbox[2]+k >= nbox[2])
                            continue;

                        decorator->getElementSet(defaulttype::Vec3i(cbox[0] + i,cbox[1] + j,cbox[2] + k), selectElements);
                    }
                }
            }
        }

        {
            int j=d;
            if (cbox[1]+j >= 0 && cbox[1]+j < nbox[1])
            {
                for (int i=-d+1;i<d;i++)
                {
                    if (cbox[0]+i < 0 || cbox[0]+i >= nbox[0])
                        continue;

                    for (int k=-d;k<=d;k++)
                    {
                        if (cbox[2]+k < 0 || cbox[2]+k >= nbox[2])
                            continue;

                        decorator->getElementSet(defaulttype::Vec3i(cbox[0] + i,cbox[1] + j,cbox[2] + k), selectElements);
                    }
                }
            }
        }

        {
            int k=-d;
            if (cbox[2]+k >= 0 && cbox[2]+k < nbox[2])
            {
                for (int i=-d+1;i<d;i++)
                {
                    if (cbox[0]+i < 0 || cbox[0]+i >= nbox[0])
                        continue;

                    for (int j=-d+1;j<d;j++)
                    {
                        if (cbox[1]+j < 0 || cbox[1]+j >= nbox[1])
                            continue;

                        decorator->getElementSet(defaulttype::Vec3i(cbox[0] + i,cbox[1] + j,cbox[2] + k), selectElements);
                    }
                }
            }
        }

        {
            int k=d;
            if (cbox[2]+k >= 0 && cbox[2]+k < nbox[2])
            {
                for (int i=-d+1;i<d;i++)
                {
                    if (cbox[0]+i < 0 || cbox[0]+i >= nbox[0])
                        continue;

                    for (int j=-d+1;j<d;j++)
                    {
                        if (cbox[1]+j < 0 || cbox[1]+j >= nbox[1])
                            continue;

                        decorator->getElementSet(defaulttype::Vec3i(cbox[0] + i,cbox[1] + j,cbox[2] + k), selectElements);
                    }
                }
            }
        }
    }

    BaseProximity::SPtr findClosestProximity(const BaseProximity::SPtr & pfrom, BaseGeometry *geo) {
        BaseGeometry::BroadPhase::SPtr decorator = geo->getBroadPhase();

        if (decorator == NULL) {
            BaseElementIterator::UPtr begin = geo->begin();
            return toolBox::doFindClosestProximityIt(pfrom,begin,
                                                     l_algorithm->getFilterMethod(),
                                                     l_algorithm->getDistanceMethod());
        } else {
            //take the first broad phase...
            defaulttype::Vector3 P = pfrom->getPosition();

            defaulttype::Vec3i bindex = decorator->getBoxCoord(P);
            defaulttype::Vec3i bsize = decorator->getBoxSize();

            int max = 0;

            for (int i = 0 ; i < 3 ; i++) {
                max = std::max (max, bindex[i]) ;
                max = std::max (max, bsize[i]-bindex[i]) ;
            }

            BaseProximity::SPtr minprox_dest = nullptr;
            int d = 0;
            std::set<unsigned> selectedElements;
            while (selectedElements.empty() && d<max) {
                fillElementSet(decorator,bindex,selectedElements,d);

                BaseElementIterator::UPtr begin(new SubsetElementIterator(geo, selectedElements));
                minprox_dest = toolBox::doFindClosestProximityIt(pfrom, begin,
                                                                 l_algorithm->getFilterMethod(),
                                                                 l_algorithm->getDistanceMethod());

                d++;// we look for boxed located at d+1

                //take the first on that satisfy filters
                if (minprox_dest != NULL) return minprox_dest;
            }
        }

        return NULL;
    }

};


}

}
