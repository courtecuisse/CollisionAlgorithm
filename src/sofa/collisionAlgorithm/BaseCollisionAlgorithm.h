#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/core/collision/Pipeline.h>

namespace sofa
{

namespace collisionAlgorithm
{

typedef std::pair<ConstraintProximity::SPtr,ConstraintProximity::SPtr> PairProximity;
typedef helper::vector<PairProximity> PairProximityVector;
typedef helper::vector<ConstraintProximity::SPtr> ProximityVector;

class BaseCollisionAlgorithm : public core::collision::Pipeline
{
public :
    SOFA_ABSTRACT_CLASS(BaseCollisionAlgorithm, core::collision::Pipeline);

    void draw(const core::visual::VisualParams * vparams) override
    {
        if (! vparams->displayFlags().getShowCollisionModels()) return;
        glDisable(GL_LIGHTING);

        glColor4f(0,1,0,1);
        glBegin(GL_LINES);
        for (unsigned i=0;i<m_pairDetection.size();i++)
        {
            glVertex3dv(m_pairDetection[i].first->getPosition().data());
            glVertex3dv(m_pairDetection[i].second->getPosition().data());
        }
        glEnd();
    }

    virtual void reset() override {}

    virtual void computeCollisionReset() override
    {
        m_pairDetection.clear();
    }

    virtual void computeCollisionResponse() override {}

    void computeCollisionDetection() override
    {
        processAlgorithm();
    }

    PairProximityVector& getCollisionPairs()
    {
        return m_pairDetection;
    }

    virtual void getState(std::set<sofa::core::behavior::MechanicalState<defaulttype::Vec3dTypes>* > & list_state) = 0;

    virtual std::set< std::string > getResponseList() const override
    {
        std::set< std::string > res;
        return res;
    }

protected:
    virtual void doCollisionReset() override
    {
//         m_pairDetection.clear();
    }

    virtual void doCollisionDetection(const sofa::helper::vector<core::CollisionModel*>& /*collisionModels*/) override
    {

    }

    virtual void doCollisionResponse() override
    {

    }

    virtual void processAlgorithm() = 0;

    PairProximityVector m_pairDetection;
};

}

}
