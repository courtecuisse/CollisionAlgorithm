#pragma once

#include <sofa/helper/AdvancedTimer.h>
#include <sofa/collisionAlgorithm/BaseAABBBroadPhase.h>
#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseElement.h>

namespace sofa::collisionAlgorithm {

class AABBBroadPhase : public BaseAABBBroadPhase {
public:

    SOFA_CLASS(AABBBroadPhase,BaseAABBBroadPhase);


    AABBBroadPhase() {}

    void updateData() override {
        m_offset[0] = m_nbox[1]*m_nbox[2];
        m_offset[1] = m_nbox[2];
    }

    const std::set<BaseElement::SPtr> & getElementSet(unsigned i, unsigned j, unsigned k) const override {
        auto it = m_indexedElement.find(getKey(i,j,k));
        if (it == m_indexedElement.end()) {
            static std::set<BaseElement::SPtr> empty;
            return empty;
        } else {
            return it->second;
        }
    }

    void newContainer() override {
        m_indexedElement.clear();
    }

    void addElement(int i, int j, int k, BaseElement::SPtr elmt) override {
        m_offset[0] = m_nbox[1]*m_nbox[2];
        m_offset[1] = m_nbox[2];

        unsigned key_i = i*m_offset[0];
        unsigned key_j = j*m_offset[1];
        unsigned key = key_i + key_j + k;

        m_indexedElement[key].insert(elmt);
    }


    inline Index getKey(size_t i,size_t j,size_t k) const {
        return i*m_offset[0] + j * m_offset[1] + k;
    }

    inline unsigned getIKey(unsigned key) {
        return key/m_offset[0];
    }

    inline unsigned getJKey(unsigned key) {
        return (key - getIKey(key) * m_offset[0])/m_offset[1];
    }

    inline unsigned getKKey(unsigned key) {
        return key - getIKey(key) * m_offset[0] - getJKey(key) * m_offset[1];
    }



protected:
    std::map<unsigned, std::set<BaseElement::SPtr> > m_indexedElement;
    type::Vec<2, size_t> m_offset;
};


}
