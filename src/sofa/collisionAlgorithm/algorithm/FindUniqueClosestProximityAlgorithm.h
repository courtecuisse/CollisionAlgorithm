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

        //The goal is to find every potential pair given the filter. Then for each i in the from geom,
        // sort its potential pairs depending on the distance. Then iterate on every list of pair of the from geom and
        // always take the pair presenting the minimum distance if the second element hasn't been taken yet.
        // when a fom object has been taken, forget its list.

        std::map<unsigned, std::list<std::pair<double,collisionAlgorithm::PairDetection> > > potentialPairForEachFromElem;
        std::map<unsigned, bool> destPointTaken;

        std::list<collisionAlgorithm::PairDetection> uniqueCouple;

        //Iterate on the from geom and find each potential pair
        for (auto itfrom=l_from->begin();itfrom!=l_from->end();itfrom++)
        {
            for (auto itDest=l_dest->begin();itDest!=l_dest->end();itDest++)
            {
                auto pfrom = itfrom->createProximity();
                auto pdest = itDest->createProximity();
                for (unsigned it=0;it<d_iterations.getValue();it++) {
                    pfrom = itfrom->project(pdest->getPosition()); // reproject on pfrom
                    pdest = itDest->project(pfrom->getPosition());

                    if (pfrom == NULL || pdest == NULL) break;
                }
                bool accept = true;
                for( auto filt : m_filters)
                {
                    accept &= filt->accept(pfrom,pdest);
                }
                if(accept)
                {
                    potentialPairForEachFromElem[itfrom->id()].push_back(std::make_pair(m_distanceMethod(collisionAlgorithm::PairDetection(pfrom,pdest)),collisionAlgorithm::PairDetection(pfrom,pdest)));
                    destPointTaken[itDest->id()] = false;
                }
            }
        }

        //Now sort all the potential pairs
        auto greater =  [](const std::pair<double,collisionAlgorithm::PairDetection>& first ,const std::pair<double,collisionAlgorithm::PairDetection>& second )
                 { return first.first<second.first;};
        for(auto it=potentialPairForEachFromElem.begin(); it!=potentialPairForEachFromElem.end(); it++)
        {
            it->second.sort(greater);
        }

        //Now that every list is sorted, iterate on each first element of each list to find the pair providing the smaller distance
        bool empty=false;
        while(!empty)
        {
            double min_dist = std::numeric_limits<double>::max();
            collisionAlgorithm::PairDetection min_pair(NULL,NULL);
            empty = true;
            //For each remaining element of 'from' that haven't been paired : find in the minimum distance pair among
            // all the 'from' points. Because of the sort, only comparing the smallest pair of each 'from' element is
            // sufficient -> you always take the smallest pair among all the possible pair among all the 'from' points.
            for(auto it=potentialPairForEachFromElem.begin(); it!=potentialPairForEachFromElem.end(); it++)
            {
                //Loop over the remaining potential pair in a increasing manner in terms of distance, delete those with
                // the dest element already paired and stop at the first 'dest' element that is not paired
                while(it->second.size() && destPointTaken[it->second.begin()->second.second->getElementId()])
                    it->second.pop_front();
                // If no pair remains, this point should not be paired
                if(!(it->second.size()))
                    continue;
                // If here then the map is not empty, potential pairs still remains.
                empty = false;

                //Now find if it is the smallest of those already encountered
                if(it->second.begin()->first<min_dist)
                {
                    min_dist=it->second.begin()->first;
                    min_pair = it->second.begin()->second;
                }
            }
            //If oboth of the proximity are NULL then the map is empty, otherwise, stock this pair.
            if((min_pair.first != NULL )&& (min_pair.second != NULL))
            {
                //Store the pair
                uniqueCouple.push_back(min_pair);
                //Mark the 'dest' element as taken
                destPointTaken[min_pair.second->getElementId()] = true;
                //Erase the line of the 'from' element --> no need to pair it anymore
                potentialPairForEachFromElem.erase(min_pair.first->getElementId());
            }
        }


        //Push only the unique pairs
        for(auto it=uniqueCouple.begin(); it!=uniqueCouple.end(); it++)
        {
            output.add(it->first,it->second);
            outputDist.push_back(m_distanceMethod(output.back()));

            matchIdFrom.push_back(it->first->getElementId());
            matchIdDest.push_back(it->second->getElementId());
        }



        d_output.endEdit();
        d_matchIdFrom.endEdit();
        d_matchIdDest.endEdit();
    }

};


}

}
