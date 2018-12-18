#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/BaseElementFilter.h>
#include <sofa/core/collision/Pipeline.h>

namespace sofa
{

namespace collisionAlgorithm
{


class DetectionOutput {
public:
    typedef std::shared_ptr<DetectionOutput> SPtr;

    virtual unsigned size() const = 0;

    virtual ConstraintProximity::SPtr getProximity(unsigned sz) const = 0;

    virtual defaulttype::Vector3 getNormal() const {
        return m_dir;
    }

    DetectionOutput(const defaulttype::Vector3 & d) : m_dir(d.normalized()) {}

    virtual void draw() const {}
//        for (unsigned i=0;i<size();i++) {
//            vparams->drawTool()->drawArrow((*this)[i]m_pproxy.second->getPosition(),
//                                           m_pproxy.second->getPosition() + m_directions.m_normals[i] * scale,
//                                           scale*0.1,
//                                           c);

//        }
//    }

protected:
    const defaulttype::Vector3 m_dir;
};

class BaseCollisionAlgorithm : public core::collision::Pipeline
{
public :
    SOFA_ABSTRACT_CLASS(BaseCollisionAlgorithm, core::collision::Pipeline);

    BaseCollisionAlgorithm()
        : l_filter(initLink("filter", "Link to an optional filter"))
    {

    }
    virtual ~BaseCollisionAlgorithm() override {}

    void draw(const core::visual::VisualParams * vparams) override
    {
        if (! vparams->displayFlags().getShowCollisionModels()) return;
        for (unsigned i=0;i<m_output.size();i++) m_output[i]->draw();
    }

    virtual void reset() override {}

    virtual void computeCollisionReset() override
    {
        m_output.clear();
    }

    virtual void computeCollisionResponse() override {}

    void computeCollisionDetection() override
    {
        processAlgorithm();
    }

    virtual void getState(std::set<sofa::core::behavior::MechanicalState<defaulttype::Vec3dTypes>* > & list_state) = 0;

    virtual std::set< std::string > getResponseList() const override
    {
        std::set< std::string > res;
        return res;
    }

    inline void addDetectionOutput(defaulttype::Vector3 mainDir, ConstraintProximity::SPtr p1) {
        m_output.push_back(DetectionOutput::SPtr(new SingleDetectionOutput(mainDir,p1)));
    }

    inline void addDetectionOutput(defaulttype::Vector3 mainDir, ConstraintProximity::SPtr p1, ConstraintProximity::SPtr p2) {
        m_output.push_back(DetectionOutput::SPtr(new PairDetectionOutput(mainDir,p1,p2)));
    }

    inline unsigned getNbOutput() {
        return m_output.size();
    }

    inline DetectionOutput::SPtr getOutput(unsigned i) {
        return m_output[i];
    }

protected:
    core::objectmodel::SingleLink<BaseCollisionAlgorithm,BaseElementFilter,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_filter;

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

private:
    helper::vector<DetectionOutput::SPtr> m_output;


    class PairDetectionOutput : public DetectionOutput {
    public:

        PairDetectionOutput(const defaulttype::Vector3 & mainDir, ConstraintProximity::SPtr p1, ConstraintProximity::SPtr p2) : DetectionOutput(mainDir){
            m_prox[0] = p1;
            m_prox[1] = p2;
        }
        virtual ~PairDetectionOutput() {}

        unsigned size() const { return 2; }

        ConstraintProximity::SPtr getProximity(unsigned i) const {
            return m_prox[i];
        }

        ConstraintProximity::SPtr m_prox[2];
    };

    class SingleDetectionOutput : public DetectionOutput {
    public:

        SingleDetectionOutput(const defaulttype::Vector3 & mainDir, ConstraintProximity::SPtr p1) : DetectionOutput(mainDir){
            m_prox = p1;
        }
        virtual ~SingleDetectionOutput() {}

        unsigned size() const { return 1; }

        ConstraintProximity::SPtr getProximity(unsigned ) const {
            return m_prox;
        }

        ConstraintProximity::SPtr m_prox;
    };

};

}

}
