#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/proximity/PointProximity.h>
#include <sofa/collisionAlgorithm/elements/PointElement.h>
#include <sofa/collisionAlgorithm/toolbox/PointToolBox.h>
#include <sofa/collisionAlgorithm/proximity/TopologyProximity.h>

namespace sofa {

namespace collisionAlgorithm {

template<class DataTypes>
class SubsetGeometry : public TBaseGeometry<DataTypes> {
public:
    typedef DataTypes TDataTypes;
    typedef PointElement ELEMENT;
    typedef SubsetGeometry<DataTypes> GEOMETRY;
    typedef TBaseGeometry<DataTypes> Inherit;
    typedef typename DataTypes::VecCoord VecCoord;
    typedef core::objectmodel::Data< VecCoord >        DataVecCoord;
    typedef std::function<BaseProximity::SPtr(const PointElement * elmt)> ProximityCreatorFunc;

    SOFA_CLASS(GEOMETRY,Inherit);

    Data<type::vector<int> > d_indices;

    core::objectmodel::SingleLink<GEOMETRY,BaseGeometry,BaseLink::FLAG_STRONGLINK|BaseLink::FLAG_STOREPATH> l_geometry;

    SubsetGeometry()
      : l_geometry(initLink("wholeGeometry", "link to topology"))
      , d_indices(initData(&d_indices,"indices", "Indices to keep"))
    {
      l_geometry.setPath("@.");
    }
//    type::Vector3 getPosition(unsigned pid) override {
//        const helper::ReadAccessor<DataVecCoord> & pos = this->getState()->read(core::VecCoordId::position());
//        return pos[pid];
//    }

	void init()
	{
		type::vector<int>  indices = d_indices.getValue();

		std::sort(indices.begin(), indices.end());

		m_elements.clear();
		auto iter = l_geometry->begin();

		int id=0;
		for(int keepId : indices)
		{
			while(id != keepId)
			{
				++id;
				iter++;
			}
			m_elements.push_back(iter->element());
		}
		std::cout<<"SubsetGeometry : "<<m_elements.size()<<std::endl;

	}


    void prepareDetection() override {}

    ElementIterator::SPtr begin(unsigned id = 0) const override {
        return pointBegin(id);
    }

    inline ElementIterator::SPtr pointBegin(unsigned id = 0) const {
        return ElementIterator::SPtr(new TDefaultElementIteratorPtr(m_elements,id));
    }


private:
    std::vector<BaseElement *> m_elements;
};

}

}
