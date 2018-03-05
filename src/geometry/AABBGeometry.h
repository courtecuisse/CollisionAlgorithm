#pragma once

#include <BaseGeometry.h>
#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/VecTypes.h>

namespace sofa {

namespace core {

namespace behavior {

class AABBGeometry : public BaseGeometry {
    friend class AABBElement;

public:
    SOFA_CLASS(AABBGeometry , BaseGeometry);

    typedef sofa::defaulttype::Vec3dTypes DataTypes;
    typedef DataTypes::VecCoord VecCoord;

    Data<defaulttype::Vec3i> d_nbox;
    Data<std::string> d_geometry;

//    class ElementAABBIterator : public ElementIterator {
//    public:
//        ElementAABBIterator(BaseGeometry * geo,std::set<int> set) {
//            m_selectedElements = set;
//            it = m_selectedElements.begin();
//            m_geo = geo;
//        }

//        bool next() {
//            it++;
//            return it != m_selectedElements.end();
//        }

//        ConstraintElementPtr getElement() {
//            return m_geo->getElement(*it);
//        }

//    private:
//        std::set<int> m_selectedElements;
//        std::set<int>::iterator it;
//        BaseGeometry * m_geo;
//    };

    AABBGeometry();

    void init();

    void prepareDetection();

    void draw(const core::visual::VisualParams * vparams);

    virtual unsigned getNbElements() const;

    virtual ConstraintElementPtr getElement(unsigned eid) const;

protected:
    defaulttype::Vector3 m_Bmin,m_Bmax,m_cellSize;
    helper::vector<helper::vector<helper::vector<helper::vector<unsigned> > > >  m_indexedElement;
    BaseGeometry * m_geo;

};

} // namespace forcefield

} // namespace component

} // namespace sofa

