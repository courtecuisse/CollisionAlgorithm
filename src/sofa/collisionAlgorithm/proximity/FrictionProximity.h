#pragma once

#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <numeric>
#include <execution>

namespace sofa
{

namespace collisionAlgorithm
{


class FrictionProximity : public BaseProximity {
public:
    typedef BaseProximity::Index Index;

    FrictionProximity(){}

    constexpr static CONTROL_POINT nbControlPoints() {
        return CONTROL_1;
    }

    defaulttype::Vector3 getPosition(core::VecCoordId v) const {
        defaulttype::Vector3 position = defaulttype::Vector3(0,0,0);
        if (!m_list.size()) return (position);
        else{
            for(auto& prox : m_list){
                position += prox->getPosition();
            }
            return (position/m_list.size());
        }
    }

   defaulttype::Vector3 getNormal() const {
       defaulttype::Vector3 normal = defaulttype::Vector3(0,0,0);
       if (!m_list.size()) return (normal);
       else{
           for(auto& prox : m_list){
               normal += prox->getNormal();
           }
           return (normal/m_list.size());
       }
    }

    void buildJacobianConstraint(core::MultiMatrixDerivId cId, const helper::vector<defaulttype::Vector3> & normals, double fact, Index constraintId) const {
        // normals.size() == 1 because 1dof in FrictionProximity
        helper::vector<defaulttype::Vector3> subNormals = {defaulttype::Vector3(0,0,0)};
        subNormals[0] = normals[0]/m_list.size();
        for(auto& prox : m_list){
            std::cout<<"subNormals.size() = "<<subNormals.size()<<std::endl;
            std::cout<<"cId = "<<cId<<std::endl;
            prox->buildJacobianConstraint(cId,subNormals,fact,constraintId);
        }
    }

    void storeLambda(const core::ConstraintParams* cParams, core::MultiVecDerivId res, Index cid_global, Index cid_local,const sofa::defaulttype::BaseVector* lambda) const {
        for(auto& prox : m_list){
            prox->storeLambda(cParams, res, cid_global, cid_local, lambda);
        }
    }

    Index getElementId() const {
        return 0;
    }

    void addProximity(collisionAlgorithm::BaseProximity::SPtr prox){
        m_list.push_back(prox);
    }

    type::vector<collisionAlgorithm::BaseProximity::SPtr> m_list;

//    addProximity
};

}

}