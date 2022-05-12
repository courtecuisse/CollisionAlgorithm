#pragma once

#include <sofa/collisionAlgorithm/BaseAlgorithm.h>
#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/BaseOperation.h>
#include <sofa/collisionAlgorithm/operations/CreateCenterProximity.h>
#include <sofa/collisionAlgorithm/operations/Project.h>
#include <sofa/core/objectmodel/DataCallback.h>


namespace sofa::collisionAlgorithm {

//Specific operation to find the closest point on a geometry (the code is in the c++ class)
class FindClosestProximityOperation : public Operations::GenericOperation<FindClosestProximityOperation,
        std::function<BaseProximity::SPtr(BaseProximity::SPtr,BaseGeometry *) > > {
public:

    using Inherit = GenericOperation;

    GenericOperation::FUNC getDefault() const override;
};

class ManualBindAlgorithm : public BaseAlgorithm {
public:
    SOFA_CLASS(ManualBindAlgorithm, BaseAlgorithm);

    core::objectmodel::SingleLink<ManualBindAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_from;
    core::objectmodel::SingleLink<ManualBindAlgorithm,BaseGeometry,BaseLink::FLAG_STOREPATH|BaseLink::FLAG_STRONGLINK> l_dest;

	core::objectmodel::DataCallback c_callBack;

    Data<type::vector<int>> d_bindFrom ;
    Data<type::vector<int>> d_bindDest ;
    Data<bool> d_drawCollision ;
    Data<DetectionOutput> d_output;
    Data<sofa::type::vector<double> > d_outputDist;

    ManualBindAlgorithm()
    : l_from(initLink("from", "link to from geometry"))
    , l_dest(initLink("dest", "link to dest geometry"))
	, d_bindFrom(initData(&d_bindFrom, "bindFrom", "draw collision"))
	, d_bindDest(initData(&d_bindDest, "bindDest", "draw collision"))
    , d_drawCollision (initData(&d_drawCollision, true, "drawcollision", "draw collision"))
    , d_output(initData(&d_output,"output", "output of the collision detection"))
    , d_outputDist(initData(&d_outputDist,"outputDist", "Distance of the outpu pair of detections"))    
    {
		m_isDirty = true;
		c_callBack.addInputs({&d_bindDest,&d_bindFrom});
		c_callBack.addCallback(std::bind(&ManualBindAlgorithm::bindingChanged,this));

	}

    void draw(const core::visual::VisualParams* vparams) {
        if (! vparams->displayFlags().getShowCollisionModels() && ! d_drawCollision.getValue()) return;
        glDisable(GL_LIGHTING);
        glColor4f(0,1,0,1);

        glBegin(GL_LINES);
        DetectionOutput output = d_output.getValue() ;
        for (unsigned i=0;i<output.size();i++) {
            glVertex3dv(output[i].first->getPosition().data());
            glVertex3dv(output[i].second->getPosition().data());
        }
        glEnd();
    }

    void doDetection() {

		if (l_from == NULL) return;
        if (l_dest == NULL) return;

		if(d_bindDest.getValue().size() != d_bindFrom.getValue().size()) return;

		if(! m_isDirty) return;
		m_isDirty = false;

		DetectionOutput & output = *d_output.beginEdit();
        output.clear();

        auto createProximityFrom = Operations::CreateCenterProximityOperation::get(l_from->begin()->getOperationsHash());
        auto createProximityDest = Operations::CreateCenterProximityOperation::get(l_dest->begin()->getOperationsHash());

        for (int i = 0; i<d_bindDest.getValue().size(); i++)
		{
			auto fromIt = l_from->begin();
			auto destIt = l_dest->begin();

			for(int j=0; j<d_bindFrom.getValue()[i]; j++)
				fromIt ++;

			for(int j=0; j<d_bindDest.getValue()[i]; j++)
				destIt ++;

            output.push_back(PairDetection(createProximityFrom(fromIt->element()),createProximityDest(destIt->element())));
        }

        d_output.endEdit();
    }

	void bindingChanged()
	{
		m_isDirty = true;
	}

	bool m_isDirty;

};

}

