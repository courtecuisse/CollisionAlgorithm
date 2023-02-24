#pragma once

#include <sofa/collisionAlgorithm/CollisionPipeline.h>
#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/ElementIterator.h>
#include <sofa/gl/gl.h>
#include <sofa/helper/AdvancedTimer.h>
#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/elements/EdgeElement.h>
#include <sofa/collisionAlgorithm/elements/TriangleElement.h>
#include <sofa/collisionAlgorithm/elements/TetrahedronElement.h>

namespace sofa ::collisionAlgorithm {


class InternalDataContainer {
public:


    class InternalData {
    public:
        InternalData() = default;
        virtual ~InternalData() = default;

    };

    InternalDataContainer() = default;


    template<class KEY,class CLASS = KEY>
    inline CLASS * get() {
        size_t hash = typeid(KEY).hash_code();
        auto it = m_internalData.find(hash);
        if (it==m_internalData.cend()) return NULL;
        return reinterpret_cast<CLASS*>(it->second.get());
    }

    template<class T>
    inline void set(InternalData * ptr) {
        size_t hash = typeid (T).hash_code();
        m_internalData[hash] = std::shared_ptr<InternalData>(ptr);
    }

    template<class T>
    inline void clear() {
        size_t hash = typeid (T).hash_code();
        m_internalData.erase(hash);
    }

    template<class KEY,class CLASS = KEY, class... ARGS>
    inline CLASS * get_or_create(ARGS... args) {
        size_t hash = typeid (KEY).hash_code();

        auto it = m_internalData.find(hash);
        if (it==m_internalData.cend()) {
            CLASS * ptr = new CLASS(args...);

            m_internalData[hash] = std::shared_ptr<InternalData>(ptr);

            std::cout<<"CREATE"<<std::endl;

            return ptr;


        }
        std::cout<<"GET"<<std::endl;

        return reinterpret_cast<CLASS*>(it->second.get());
    }



    unsigned size() const { return m_internalData.size(); }

    std::map<size_t,std::shared_ptr<InternalData> >::const_iterator cbegin() const { return m_internalData.cbegin(); }

    std::map<size_t,std::shared_ptr<InternalData> >::const_iterator cend() const { return m_internalData.cend(); }

private:
    std::map<size_t,std::shared_ptr<InternalData> > m_internalData;
};



}

