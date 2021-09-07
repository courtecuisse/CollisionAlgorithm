#pragma once

#include <sofa/collisionAlgorithm/algorithm/FindClosestProximityAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

class FindUniqueClosestProximityAlgorithm : public FindClosestProximityAlgorithm
{
public:
    SOFA_CLASS(FindUniqueClosestProximityAlgorithm, FindClosestProximityAlgorithm);

    FindUniqueClosestProximityAlgorithm(){}

    void doDetection() {
        if (l_from == NULL) return;
        if (l_dest == NULL) return;

        DetectionOutput & output = *d_output.beginEdit();
        output.clear();
        sofa::type::vector<double> & outputDist = *d_outputDist.beginEdit();
        outputDist.clear();

        sofa::type::vector<unsigned> & matchIdFrom = *d_matchIdFrom.beginEdit();
        sofa::type::vector<unsigned> & matchIdDest = *d_matchIdDest.beginEdit();
        matchIdFrom.clear();
        matchIdDest.clear();

        //Map used to identify if element from dest geometry has already been paired
        std::map<unsigned,collisionAlgorithm::PairDetection> uniqueCouple;

        for (auto itfrom=l_from->begin();itfrom!=l_from->end();itfrom++) {
            PairDetection min_pair = findClosestPoint(*itfrom,l_dest.get());
            if (min_pair.first == nullptr || min_pair.second == nullptr) {
                continue;
            }
            auto it = uniqueCouple.find(min_pair.second->getElementId());
            //If dest element has already been paired
            if( it != uniqueCouple.end())
            {
                //If new 'from' element is closed than the current one, change the pair
                if(m_distanceMethod(min_pair)<m_distanceMethod((*it).second))
                {
                    it->second=min_pair;
                }
            }
            else
            {
                uniqueCouple.emplace(std::pair<unsigned,PairDetection>(min_pair.second->getElementId(),min_pair));
            }
        }

        //Push only the unique pairs
        for(auto it=uniqueCouple.begin(); it!=uniqueCouple.end(); it++)
        {
            output.add(it->second.first,it->second.second);
            outputDist.push_back(m_distanceMethod(output.back()));

            matchIdFrom.push_back(it->second.first->getElementId());
            matchIdDest.push_back(it->second.second->getElementId());
        }



        d_output.endEdit();
        d_matchIdFrom.endEdit();
        d_matchIdDest.endEdit();
    }

};


}

}
