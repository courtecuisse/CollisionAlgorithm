#pragma once

#include <algorithm/CollisionDetectionAlgorithm.h>
#include <limits>
#include <decorator/AABBDecorator.h>

namespace collisionAlgorithm {

CollisionDetectionAlgorithm::CollisionDetectionAlgorithm()
: p_from("from",this)
, p_dest("dest",this)
{}

template<class ElementIterator>
PairProximity CollisionDetectionAlgorithm::getClosestPoint(ElementIterator it_element) {
    PairProximity min_pair;
    double min_dist = std::numeric_limits<double>::max();

    ConstraintProximityPtr pfrom = it_element.efrom->getControlPoint();
    if (pfrom == NULL) return min_pair;
    Vector3 P = pfrom->getPosition();

    for (unsigned i=0;i<it_element.size();i++) {
        ConstraintElementPtr edest = it_element.element(i);
        ConstraintProximityPtr pdest = edest->project(P);
        pfrom = it_element.efrom->project(pdest->getPosition());

        //iterate until to find the correct location on pfrom
        for (int itearation = 0;itearation<10 && pfrom->distance(P)>0.0001;itearation++) {
            P = pfrom->getPosition();
            pdest = edest->project(P);
            pfrom = it_element.efrom->project(pdest->getPosition());
        }

        //compute all the distances with to elements
        double dist = pdest->distance(P);

        if (dist<min_dist) {
            min_dist = dist;
            min_pair.first = pfrom;
            min_pair.second = pdest;
        }
    }

    return min_pair;
}


class AABBElementIterator {
public :

    AABBElementIterator(ConstraintElementPtr from,AABBDecorator * aabb) {
        m_aabb = aabb;
        m_geo = aabb->p_geometry();
        efrom = from;

        Vector3 P = from->getControlPoint()->getPosition();

        //compute the box where is P
        Vec3i cbox;
        cbox[0] = floor((P[0] - m_aabb->m_Bmin[0])/m_aabb->m_cellSize[0]);
        cbox[1] = floor((P[1] - m_aabb->m_Bmin[1])/m_aabb->m_cellSize[1]);
        cbox[2] = floor((P[2] - m_aabb->m_Bmin[2])/m_aabb->m_cellSize[2]);

        //project the box in the bounding box of the object
        //search with the closest box in bbox
        for (int i=0;i<3;i++) {
            if (cbox[i] < 0) cbox[i] = 0;
            else if (cbox[i] > m_aabb->m_nbox[i]) cbox[i] = m_aabb->m_nbox[i];
        }

        int d = 0;
        int max = std::max(std::max(m_aabb->m_nbox[0],m_aabb->m_nbox[1]),m_aabb->m_nbox[2]);
        std::set<int> selectElements;
        while (selectElements.empty() && d<max) {
            fillElementSet(cbox,d,selectElements);
            d++;// we look for boxed located at d+1
        }

        m_selectElements.clear();
        for (std::set<int>::iterator it = selectElements.begin();it != selectElements.end(); it++) m_selectElements.push_back(*it);
    }

    void fillElementSet(Vec3i cbox, int d, std::set<int> & selectElements) {
        {
            int i=-d;
            if (cbox[0]+i >= 0 && cbox[0]+i < m_aabb->m_nbox[0]) {
                for (int j=-d;j<=d;j++) {
                    if (cbox[1]+j < 0 || cbox[1]+j >= m_aabb->m_nbox[1]) continue;
                    for (int k=-d;k<=d;k++) {
                        if (cbox[2]+k < 0 || cbox[2]+k >= m_aabb->m_nbox[2]) continue;

                        const std::set<unsigned> & elemntsID = m_aabb->getIndexedElements(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                        selectElements.insert(elemntsID.begin(),elemntsID.end());
                    }
                }
            }
        }

        {
            int i=d;
            if (cbox[0]+i >= 0 && cbox[0]+i < m_aabb->m_nbox[0]) {
                for (int j=-d;j<=d;j++) {
                    if (cbox[1]+j < 0 || cbox[1]+j >= m_aabb->m_nbox[1]) continue;

                    for (int k=-d;k<=d;k++) {
                        if (cbox[2]+k < 0 || cbox[2]+k >= m_aabb->m_nbox[2]) continue;

                        const std::set<unsigned> & elemntsID = m_aabb->getIndexedElements(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                        selectElements.insert(elemntsID.begin(),elemntsID.end());
                    }
                }
            }
        }


        {
            int j=-d;
            if (cbox[1]+j >= 0 && cbox[1]+j < m_aabb->m_nbox[1]) {
                for (int i=-d+1;i<d;i++) {
                    if (cbox[0]+i < 0 || cbox[0]+i >= m_aabb->m_nbox[0]) continue;

                    for (int k=-d;k<=d;k++) {
                        if (cbox[2]+k < 0 || cbox[2]+k >= m_aabb->m_nbox[2]) continue;

                        const std::set<unsigned> & elemntsID = m_aabb->getIndexedElements(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                        selectElements.insert(elemntsID.begin(),elemntsID.end());
                    }
                }
            }
        }

        {
            int j=d;
            if (cbox[1]+j >= 0 && cbox[1]+j < m_aabb->m_nbox[1]) {
                for (int i=-d+1;i<d;i++) {
                    if (cbox[0]+i < 0 || cbox[0]+i >= m_aabb->m_nbox[0]) continue;

                    for (int k=-d;k<=d;k++) {
                        if (cbox[2]+k < 0 || cbox[2]+k >= m_aabb->m_nbox[2]) continue;

                        const std::set<unsigned> & elemntsID = m_aabb->getIndexedElements(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                        selectElements.insert(elemntsID.begin(),elemntsID.end());
                    }
                }
            }
        }

        {
            int k=-d;
            if (cbox[2]+k >= 0 && cbox[2]+k < m_aabb->m_nbox[2]) {
                for (int i=-d+1;i<d;i++) {
                    if (cbox[0]+i < 0 || cbox[0]+i >= m_aabb->m_nbox[0]) continue;

                    for (int j=-d+1;j<d;j++) {
                        if (cbox[1]+j < 0 || cbox[1]+j >= m_aabb->m_nbox[1]) continue;

                        const std::set<unsigned> & elemntsID = m_aabb->getIndexedElements(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                        selectElements.insert(elemntsID.begin(),elemntsID.end());
                    }
                }
            }
        }

        {
            int k=d;
            if (cbox[2]+k >= 0 && cbox[2]+k < m_aabb->m_nbox[2]) {
                for (int i=-d+1;i<d;i++) {
                    if (cbox[0]+i < 0 || cbox[0]+i >= m_aabb->m_nbox[0]) continue;

                    for (int j=-d+1;j<d;j++) {
                        if (cbox[1]+j < 0 || cbox[1]+j >= m_aabb->m_nbox[1]) continue;

                        const std::set<unsigned> & elemntsID = m_aabb->getIndexedElements(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                        selectElements.insert(elemntsID.begin(),elemntsID.end());
                    }
                }
            }
        }
    }

    unsigned size() {
        return (unsigned) m_selectElements.size();
    }

    inline ConstraintElementPtr element(unsigned i) {
        return m_geo->getElement(m_selectElements[i]);
    }

    BaseGeometry * m_geo;
    ConstraintElementPtr efrom;
    AABBDecorator * m_aabb;
    std::vector<int> m_selectElements;
};

class DefaultIterator {
public:
    DefaultIterator(ConstraintElementPtr from, BaseGeometry * geo) {
        efrom = from;
        m_geo = geo;
    }

    unsigned size() {
        return m_geo->getNbElements();
    }

    inline ConstraintElementPtr element(unsigned i) {
        return m_geo->getElement(i);
    }

    ConstraintElementPtr efrom;
    BaseGeometry * m_geo;
};

void CollisionDetectionAlgorithm::processAlgorithm() {
    m_pairDetection.clear();

    AABBDecorator * from = NULL;
    AABBDecorator * dest = NULL;

    //first we search if there is a AABB connected to the geometry
    for (unsigned i=0;i<p_from->p_type.size();i++) {
        if ((from = dynamic_cast<AABBDecorator *>(p_from->p_type[i]))) break;
    }

    for (unsigned i=0;i<p_dest->p_type.size();i++) {
        if ((dest = dynamic_cast<AABBDecorator *>(p_dest->p_type[i]))) break;
    }

    //we do the collision from first to second
    for (unsigned i=0;i<p_from->getNbElements();i++) {
        PairProximity pair = (dest == NULL) ? getClosestPoint(DefaultIterator(p_from->getElement(i), p_dest())) : getClosestPoint(AABBElementIterator(p_from->getElement(i), dest));

        if (pair.first == NULL) continue;
        if (pair.second == NULL) continue;

        m_pairDetection.push_back(PairProximity(pair.first,pair.second));
    }

    //then from second to first
    for (unsigned i=0;i<p_dest->getNbElements();i++) {
        PairProximity pair = (from == NULL) ? getClosestPoint(DefaultIterator(p_dest->getElement(i), p_from())) : getClosestPoint(AABBElementIterator(p_dest->getElement(i), from));

        if (pair.first == NULL) continue;
        if (pair.second == NULL) continue;

        m_pairDetection.push_back(PairProximity(pair.second,pair.first));
    }
}

}
