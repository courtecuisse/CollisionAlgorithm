#pragma once

#include <sofa/collisionAlgorithm/BaseElement.h>
#include <sofa/collisionAlgorithm/BaseProximity.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>

namespace sofa
{

namespace collisionAlgorithm
{

template<class GEOMETRY, class ELMT, class PROXIMITYDATA>
class DataElementContainer : public core::objectmodel::Data<helper::vector<ELMT> >, public BaseElementContainer {
public:

    typedef DataElementContainer<GEOMETRY, ELMT, PROXIMITYDATA> CONTAINER;

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

    typedef typename TBaseProximity<DataTypes,PROXIMITYDATA>::PositionFunctor PositionFunctor;
    typedef typename TBaseProximity<DataTypes,PROXIMITYDATA>::NormalFunctor NormalFunctor;

    explicit DataElementContainer(const typename DataElementContainer<GEOMETRY, ELMT, PROXIMITYDATA>::InitData& init)
    : core::objectmodel::Data<helper::vector<ELMT> >(init) {
        m_geometry = dynamic_cast<const GEOMETRY*>(init.owner);

        m_positionFunctor = std::bind(&CONTAINER::getPosition, this, std::placeholders::_1,std::placeholders::_2);
        m_normalFunctor = std::bind(&CONTAINER::getNormal, this, std::placeholders::_1);
    }

//    DataElementContainer(const GEOMETRY * geo)
//    : m_geometry(geo) {
//    }


//    explicit DataElementContainer(const typename core::objectmodel::Data<helper::vector<ELMT> >::InitData& init)
//    : Data<helper::vector<ELMT>>(init) {
//        m_broadPhase = NULL;
//    }

    virtual BaseElementIterator::UPtr begin(unsigned eid = 0) {
        updateInternalData();
        return DefaultElementIterator<CONTAINER>::create(this, eid);
    }

    virtual double getTime() const {
        return m_geometry->getContext()->getTime();
    }

    virtual unsigned end() const {
        return this->getValue().size();
    }

    virtual void draw(const core::visual::VisualParams *vparams, const defaulttype::Vector4 & color) {
        if (! vparams->displayFlags().getShowNormals()) return;
        if (m_geometry->d_drawScaleNormal.getValue() == 0) return;

        for (auto it=begin();it!=end();it++) {
            BaseProximity::SPtr center = (*it)->center();
            vparams->drawTool()->drawArrow(
                center->getPosition(),
                center->getPosition() + center->getNormal() * m_geometry->d_drawScaleNormal.getValue(),
                m_geometry->d_drawScaleNormal.getValue() * 0.1,
                color
            );
        }
    }


    BaseProximity::SPtr project(const BaseElementIterator * it, const defaulttype::Vector3 & P) const override {
        return createProximity(project(it->id(),P));
    }

    BaseProximity::SPtr center(const BaseElementIterator * it) const override {
        return createProximity(center(it->id()));
    }

    defaulttype::BoundingBox getBBox(const BaseElementIterator * it) const override {
        return getBBox(it->id());
    }

    inline BaseProximity::SPtr createProximity(const PROXIMITYDATA & data) const {
        return BaseProximity::SPtr(new TBaseProximity<DataTypes, PROXIMITYDATA>(getState(), data, m_positionFunctor,m_normalFunctor));
    }

    virtual PROXIMITYDATA center(unsigned eid) const = 0;

    virtual PROXIMITYDATA project(unsigned eid,const defaulttype::Vector3 & /*P*/) const = 0;

    virtual defaulttype::BoundingBox getBBox(unsigned eid) const = 0;

    virtual defaulttype::Vector3 getPosition(const PROXIMITYDATA & data, core::VecCoordId v) const = 0;

    virtual defaulttype::Vector3 getNormal(const PROXIMITYDATA & data) const = 0;

    sofa::core::behavior::MechanicalState<DataTypes> * getState() const override {
        return m_geometry->getState();
    }

    inline const ELMT & element(unsigned id) const {
        return elements()[id];
    }

    inline const helper::vector<ELMT> & elements() const {
        return this->getValue();
    }

    virtual sofa::core::objectmodel::Base* getOwner() const {
        return core::objectmodel::Data<helper::vector<ELMT> >::getOwner();
    }

    virtual sofa::core::objectmodel::BaseData* getData() const {
        return core::objectmodel::Data<helper::vector<ELMT> >::getData();
    }

protected:
    const GEOMETRY * m_geometry;
    PositionFunctor m_positionFunctor;
    NormalFunctor m_normalFunctor;
};

}

}

