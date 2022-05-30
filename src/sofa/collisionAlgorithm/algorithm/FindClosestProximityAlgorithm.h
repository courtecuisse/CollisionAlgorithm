#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/operations/CreateCenterProximity.h>
#include <sofa/collisionAlgorithm/operations/Project.h>

namespace sofa::collisionAlgorithm {

class FindClosestProximityAlgorithm : public BaseAlgorithm {
public:
    SOFA_CLASS(FindClosestProximityAlgorithm, BaseAlgorithm);

    core::objectmodel::SingleLink<FindClosestProximityAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_from;
    core::objectmodel::SingleLink<FindClosestProximityAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_dest;
    Data<bool> d_drawCollision ;
    Data<DetectionOutput> d_output;
    Data<sofa::type::vector<double> > d_outputDist;

    FindClosestProximityAlgorithm()
    : l_from(initLink("from", "link to from geometry"))
    , l_dest(initLink("dest", "link to dest geometry"))
    , d_drawCollision (initData(&d_drawCollision, true, "drawcollision", "draw collision"))
    , d_output(initData(&d_output,"output", "output of the collision detection"))
    , d_outputDist(initData(&d_outputDist,"outputDist", "Distance of the outpu pair of detections"))    
    {}

    void draw(const core::visual::VisualParams* vparams) {
        if (! vparams->displayFlags().getShowCollisionModels() && ! d_drawCollision.getValue()) return;
        glDisable(GL_LIGHTING);
        glColor4f(0,1,0,1);

        glBegin(GL_LINES);
        DetectionOutput output = d_output.getValue() ;
        for (unsigned i=0;i<output.size();i++) {
            glVertex3dv(output[i].first->getPosition().data());
            glVertex3dv(output[i].second->getPosition().data());
        }
        glEnd();
    }

    void doDetection() {
        if (l_from == NULL) return;
        if (l_dest == NULL) return;

        DetectionOutput & output = *d_output.beginEdit();
        output.clear();

        Operations::CreateCenterProximityOperation::FUNC createProximityOp = Operations::CreateCenterProximityOperation::get(l_from.get());
        Operations::ProjectOperation::FUNC projectOp = (l_dest->getBroadPhase()) ? Operations::ProjectOperation::get(l_dest->getBroadPhase()) : Operations::ProjectOperation::get(l_dest.get());

        for (auto itfrom=l_from->begin();itfrom!=l_from->end();itfrom++) {
            auto pfrom = createProximityOp(itfrom->element());
            if (pfrom == nullptr) continue;

            ElementIterator::SPtr itdest = (l_dest->getBroadPhase()) ? broadPhaseIterator(pfrom, l_dest->getBroadPhase()) : l_dest->begin();

            auto pdest = doFindClosestProx(pfrom,itdest,projectOp);
            if (pdest == nullptr) continue;

            output.push_back(PairDetection(pfrom,pdest));
        }

        d_output.endEdit();
    }

    BaseProximity::SPtr doFindClosestProx(BaseProximity::SPtr prox, ElementIterator::SPtr itdest, Operations::ProjectOperation::FUNC projectOp) {
        double min_dist = std::numeric_limits<double>::max();
        BaseProximity::SPtr res = NULL;

        type::Vector3 P = prox->getPosition();

        for (; ! itdest->end();itdest++) {
            auto edest = itdest->element();
            if (edest == nullptr) continue;

            BaseProximity::SPtr pdest = projectOp(prox->getPosition(),edest);
            if (pdest == NULL) continue;

            double d = (P - pdest->getPosition()).norm();

            if (d < min_dist) {
                if (! acceptFilter(prox,pdest)) continue;

                res = pdest;
                min_dist = d;
            }
        }

        return res;
    }

    static ElementIterator::SPtr broadPhaseIterator(BaseProximity::SPtr prox, BroadPhase * broadphase) {
        //old params : type::Vec3i cbox, std::set<BaseProximity::Index> & selectElements, int d
        type::Vec3i nbox = broadphase->getNbox();

        //take the first broad phase...
        type::Vector3 P = prox->getPosition();

        type::Vec3i cbox = broadphase->getBoxCoord(P);

        //project the box in the bounding box of the object
        //search with the closest box in bbox
        cbox[0] = std::max(0,std::min(nbox[0]-1,cbox[0]));
        cbox[1] = std::max(0,std::min(nbox[1]-1,cbox[1]));
        cbox[2] = std::max(0,std::min(nbox[2]-1,cbox[2]));

        int max = 0;
        for (int i = 0 ; i < 3 ; i++) {
            max = std::max (max, cbox[i]) ;
            max = std::max (max, nbox[i]-cbox[i]) ;
        }

        std::set<BaseElement::SPtr> selectedElements;

        for (int d=0; selectedElements.empty() && d<max;d++) {
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

                            const std::set<BaseElement::SPtr> & elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                            selectedElements.insert(elmts.cbegin(),elmts.cend());
                        }
                    }
                }
            }

            //No need to process all the box for d==0 since they all point on the same box
            if (d==0) continue;

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

                            const std::set<BaseElement::SPtr> & elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                            selectedElements.insert(elmts.cbegin(),elmts.cend());
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

                            const std::set<BaseElement::SPtr> & elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                            selectedElements.insert(elmts.cbegin(),elmts.cend());
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

                            const std::set<BaseElement::SPtr> & elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                            selectedElements.insert(elmts.cbegin(),elmts.cend());
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

                            const std::set<BaseElement::SPtr> & elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                            selectedElements.insert(elmts.cbegin(),elmts.cend());
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

                            const std::set<BaseElement::SPtr> & elmts = broadphase->getElementSet(cbox[0] + i,cbox[1] + j,cbox[2] + k);
                            selectedElements.insert(elmts.cbegin(),elmts.cend());
                        }
                    }
                }
            }
        }

        std::cout << "SELECTED = " << selectedElements.size() << std::endl;
        return ElementIterator::SPtr(new TDefaultElementIteratorPtr(selectedElements));
    }
};

}

