#pragma once

#include <sofa/collisionAlgorithm/algorithm/FindClosestProximityAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>

namespace sofa
{

namespace collisionAlgorithm
{

class FindUniqueClosestProximityAlgorithm : public BaseAlgorithm
{
public:
    SOFA_CLASS(FindUniqueClosestProximityAlgorithm, BaseAlgorithm);

	core::objectmodel::SingleLink<FindUniqueClosestProximityAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_from;
    core::objectmodel::SingleLink<FindUniqueClosestProximityAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_dest;
	Data<int> d_iterations ;
	Data<bool> d_drawCollision ;
    Data<DetectionOutput> d_output;
    Data<sofa::type::vector<double> > d_outputDist;

    FindUniqueClosestProximityAlgorithm()
	: l_from(initLink("from", "link to from geometry"))
    , l_dest(initLink("dest", "link to dest geometry"))
    , d_iterations (initData(&d_iterations, 1, "iterations", "number of iterations for the reprojections"))
    , d_drawCollision (initData(&d_drawCollision, true, "drawcollision", "draw collision"))
    , d_output(initData(&d_output,"output", "output of the collision detection"))
    , d_outputDist(initData(&d_outputDist,"outputDist", "Distance of the outpu pair of detections"))
	{}

	typedef struct
	{
		BaseProximity::SPtr prox;
		BaseElement * elem;
	} proxiWithElem;

	typedef std::pair<proxiWithElem,proxiWithElem> pairDetectionWithElem;

    void doDetection() {
        if (l_from == NULL) return;
        if (l_dest == NULL) return;

        DetectionOutput & output = *d_output.beginEdit();
        output.clear();
        sofa::type::vector<double> & outputDist = *d_outputDist.beginEdit();
        outputDist.clear();

        //The goal is to find every potential pair given the filter. Then for each i in the from geom,
        // sort its potential pairs depending on the distance. Then iterate on every list of pair of the from geom and
        // always take the pair presenting the minimum distance if the second element hasn't been taken yet.
        // when a from object has been taken, forget its list.

        std::map<BaseElement * , std::list<std::pair<double,pairDetectionWithElem> > > potentialPairForEachFromElem;
        std::map<BaseElement *, bool> destPointTaken;

        std::list<collisionAlgorithm::PairDetection> uniqueCouple;

		auto createProximityOp = Operations::CreateCenterProximityOperation::get(l_from->begin()->getOperationsHash());
		auto projectOPFrom = Operations::ProjectOperation::get(l_from->begin()->getOperationsHash());
		auto projectOPDest = Operations::ProjectOperation::get(l_dest->begin()->getOperationsHash());


        //Iterate on the from geom and find each potential pair
        for (auto itfrom=l_from->begin();itfrom!=l_from->end();itfrom++)
        {
            for (auto itDest=l_dest->begin();itDest!=l_dest->end();itDest++)
            {
                auto pfrom = createProximityOp(itfrom->element());
                auto pdest = createProximityOp(itDest->element());
                for (unsigned it=0;it<d_iterations.getValue();it++) {
                    pfrom = projectOPFrom(pdest->getPosition(),itfrom->element()); // reproject on pfrom
                    pdest = projectOPDest(pfrom->getPosition(),itDest->element());

                    if (pfrom == NULL || pdest == NULL) break;
                }
                bool accept = true;
                for( auto filt : m_filters)
                {
                    accept &= filt->accept(pfrom,pdest);
                }
                if(accept)
                {
					proxiWithElem from;
					from.elem = itfrom->element();
					from.prox = pfrom;

					proxiWithElem dest;
					dest.elem = itDest->element();
					dest.prox = pdest;

                    potentialPairForEachFromElem[from.elem].push_back(std::make_pair((pfrom->getPosition() - pdest->getPosition()).norm(),pairDetectionWithElem(from,dest)));
                    destPointTaken[from.elem] = false;
                }
            }
        }

        //Now sort all the potential pairs
        auto greater =  [](const std::pair<double,pairDetectionWithElem>& first ,const std::pair<double,pairDetectionWithElem>& second )
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
			pairDetectionWithElem min_pair;
			min_pair.first.elem = NULL;
			min_pair.second.elem = NULL;

            empty = true;
            //For each remaining element of 'from' that haven't been paired : find in the minimum distance pair among
            // all the 'from' points. Because of the sort, only comparing the smallest pair of each 'from' element is
            // sufficient -> you always take the smallest pair among all the possible pair among all the 'from' points.
            for(auto it=potentialPairForEachFromElem.begin(); it!=potentialPairForEachFromElem.end(); it++)
            {
                //Loop over the remaining potential pair in a increasing manner in terms of distance, delete those with
                // the dest element already paired and stop at the first 'dest' element that is not paired
                while(it->second.size() && destPointTaken[it->second.begin()->second.second.elem])
                    it->second.pop_front();
                // If no pair remains, this point should not be paired
                if(!(it->second.size()))
                    continue;
                // If here then the map is not empty, potential pairs still remains.
                empty = false;

                //Now find if it is the smallest of those already encountered
                if(it->second.begin()->first<min_dist)
                {
                    min_dist = it->second.begin()->first;
                    min_pair = it->second.begin()->second;
                }
            }
            //If oboth of the proximity are NULL then the map is empty, otherwise, stock this pair.
            if((min_pair.first.elem != NULL )&& (min_pair.second.elem != NULL))
            {
                //Store the pair
                uniqueCouple.push_back(PairDetection(min_pair.first.prox,min_pair.second.prox));
                //Mark the 'dest' element as taken
                destPointTaken[min_pair.second.elem] = true;
                //Erase the line of the 'from' element --> no need to pair it anymore
                potentialPairForEachFromElem.erase(min_pair.first.elem);
            }
        }


        //Push only the unique pairs
        for(auto it=uniqueCouple.begin(); it!=uniqueCouple.end(); it++)
        {
            output.add(it->first,it->second);
            outputDist.push_back((output.back().first->getPosition() - output.back().second->getPosition()).norm());
        }
        d_output.endEdit();
    }

};


}

}
