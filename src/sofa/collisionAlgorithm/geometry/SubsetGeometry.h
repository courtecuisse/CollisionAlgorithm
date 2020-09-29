#pragma once

#include <sofa/helper/set.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/SubsetElementIterator.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class SubsetGeometry : public BaseGeometry {
public:
    typedef DataTypes TDataTypes;
    typedef BaseProximity TPROXIMITYDATA;
    typedef SubsetGeometry<DataTypes> GEOMETRY;
    typedef BaseGeometry Inherit;
    typedef BaseProximity::index_type index_type;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef typename DataTypes::MatrixDeriv MatrixDeriv;
    typedef typename MatrixDeriv::RowIterator MatrixDerivRowIterator;
    typedef std::set<index_type> SetIndex;

    SOFA_CLASS(GEOMETRY,Inherit);

    Data<SetIndex> d_indices;
    //whole geometry
    core::objectmodel::SingleLink<SubsetGeometry,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_wholeGeometry;

    std::set<index_type> m_setIndices;

    SubsetGeometry()
    : d_indices(initData(&d_indices, "indices", "Indices of the primitives in the underlying geometry"))
    , l_wholeGeometry(initLink("wholeGeometry", "Whole geometry on which we want the subet"))
    {}

    inline BaseElementIterator::UPtr begin(index_type /* eid */) const override {
        return BaseElementIterator::UPtr(new SubsetElementIterator(l_wholeGeometry.get(), d_indices.getValue()));
    }

    sofa::core::behavior::BaseMechanicalState * getState() const override
    {
        return l_wholeGeometry->getState();
    }

    void recomputeNormals() override {
        l_wholeGeometry->recomputeNormals();
    }
};

}

}
