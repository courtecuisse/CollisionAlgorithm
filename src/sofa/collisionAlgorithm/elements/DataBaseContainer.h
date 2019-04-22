#pragma once

#include <sofa/collisionAlgorithm/BaseElementContainer.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/BroadPhase.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class GEOMETRY, class ELMT, class PROXIMITYDATA>
class DataBaseContainer : public core::objectmodel::Data<helper::vector<ELMT> >, public BaseElementContainer {
public:

    typedef GEOMETRY TGEOMETRY;
    typedef typename GEOMETRY::TDataTypes DataTypes;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef typename DataTypes::Coord Coord;
    typedef typename DataTypes::Real Real;
    typedef typename DataTypes::VecDeriv VecDeriv;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename DataTypes::Deriv Deriv1;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef core::objectmodel::Data< VecDeriv >        DataVecDeriv;
    typedef core::objectmodel::Data< MatrixDeriv >     DataMatrixDeriv;

    DataBaseContainer(const typename DataBaseContainer<GEOMETRY, ELMT, PROXIMITYDATA>::InitData& init)
    : core::objectmodel::Data<helper::vector<ELMT> >(init)
    , m_update_time(-1.0) {
        m_geometry = dynamic_cast<const GEOMETRY*>(init.owner);
    }

    inline unsigned end() const {
        return elements().size();
    }

    inline void drawNormals(const core::visual::VisualParams *vparams) {
        for (auto it=begin();it!=end();it++) {
            BaseProximity::SPtr center = (*it)->center();
            vparams->drawTool()->drawArrow(
                center->getPosition(),
                center->getPosition() + center->getNormal() * m_geometry->d_drawScaleNormal.getValue(),
                m_geometry->d_drawScaleNormal.getValue() * 0.1,
                m_geometry->d_color.getValue()
            );
        }
    }

    inline sofa::core::behavior::MechanicalState<DataTypes> * getState() const override {
        return m_geometry->getState();
    }

    inline const ELMT & element(unsigned id) const {
        return elements()[id];
    }

    inline const helper::vector<ELMT> & elements() const {
        return this->getValue();
    }

    sofa::core::objectmodel::BaseData* getData() const override {
        return core::objectmodel::Data<helper::vector<ELMT>>::getData();
    }

    sofa::core::objectmodel::Base* getOwner() const override {
        return core::objectmodel::Data<helper::vector<ELMT>>::getOwner();
    }

    virtual void init() {}

    virtual void prepareDetection() {}

    inline void updateContainer() {
        double time = m_geometry->getTime();

        if (m_update_time < 0) {
            init();
            if (this->m_broadPhase) this->m_broadPhase->init();
        }

        if (m_update_time < time) {
            m_update_time = time;
            prepareDetection();
            if (this->m_broadPhase) this->m_broadPhase->prepareDetection();
        }
    }

protected:
    const GEOMETRY * m_geometry;
    double m_update_time;

};

template<class CONTAINER>
class DataContainer : public CONTAINER {
public:

    DataContainer(const typename CONTAINER::InitData& init)
    : CONTAINER(init) {}


    inline BaseElementIterator::UPtr begin(unsigned eid = 0) override {
        return DefaultElementIterator<CONTAINER>::create(this, eid);
    }
};

}

}

