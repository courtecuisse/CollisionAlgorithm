#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/SubsetElementIterator.h>
#include <sofa/helper/set.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class SubsetGeometry : public BaseGeometry {
public:
    typedef DataTypes TDataTypes;
    typedef BaseProximity TPROXIMITYDATA;
    typedef SubsetGeometry<DataTypes> GEOMETRY;
    typedef BaseGeometry Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef std::set<unsigned int> SetIndex;

    SOFA_CLASS(GEOMETRY,Inherit);

    Data<SetIndex> d_indices;
    //whole geometry
    core::objectmodel::SingleLink<SubsetGeometry,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_wholeGeometry;

    std::set<unsigned int> m_setIndices;

    SubsetGeometry()
    : d_indices(initData(&d_indices, "indices", "Indices of the primitives in the underlying geometry"))
    , l_wholeGeometry(initLink("wholeGeometry", "Whole geometry on which we want the subet"))
    {

    }

    inline BaseElementIterator::UPtr begin(unsigned /* eid */) override {
        return BaseElementIterator::UPtr(new SubsetElementIterator(l_wholeGeometry.get(), d_indices.getValue()));
    }

    unsigned end() const override {
        return m_setIndices.size();
    }

    sofa::core::behavior::BaseMechanicalState * getState() const override
    {
        return l_wholeGeometry->getState();
    }
};

}

}