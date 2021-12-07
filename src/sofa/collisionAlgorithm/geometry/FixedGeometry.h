#pragma once

#include <sofa/collisionAlgorithm/BaseGeometry.h>
#include <sofa/collisionAlgorithm/iterators/DefaultElementIterator.h>
#include <sofa/collisionAlgorithm/proximity/FixedProximity.h>

namespace sofa {

namespace collisionAlgorithm {

class FixedGeometry : public BaseGeometry {
public:
    typedef sofa::defaulttype::Vec3dTypes TDataTypes;
    typedef BaseProximity::Index Index;
    typedef FixedGeometry GEOMETRY;
    typedef BaseGeometry Inherit;

    SOFA_CLASS(GEOMETRY,Inherit);

    Data<double> d_drawRadius;
    Data<sofa::type::vector<type::Vector3> > d_position;
    Data<sofa::type::vector<type::Vector3> > d_normals;

    FixedGeometry()
    : d_drawRadius(initData(&d_drawRadius, (double) 1.0, "drawRadius", "radius of drawing"))
    , d_position(initData(&d_position,sofa::type::vector<type::Vector3>(), "position","normals"))
    , d_normals(initData(&d_normals,sofa::type::vector<type::Vector3>(), "normals","normals"))
    {}

    inline BaseElementIterator::SPtr begin(Index eid = 0) const override {
        return DefaultElementIterator<FixedProximity>::create(this, d_position.getValue(), eid);
    }

    sofa::core::behavior::BaseMechanicalState * getState() const {
        return NULL;
    }

    void draw(const core::visual::VisualParams *vparams) override {
        if(!this->d_draw.getValue()) return;

//        if (! vparams->displayFlags().getShowCollisionModels()) return;
        if (! vparams->displayFlags().getShowCollisionModels()) return ;

        const sofa::type::RGBAColor & color = this->d_color.getValue();
        if (color[3] == 0.0) return;
        if (d_drawRadius.getValue() == 0.0) return;

        const sofa::type::vector<type::Vector3> & pos = d_position.getValue();

        glColor4f(color[0],color[1],color[2],color[3]);

        for(auto it=this->begin();it != this->end();it++) {
            vparams->drawTool()->drawSphere(pos[it->id()],d_drawRadius.getValue());
        }
    }

    inline FixedProximity createProximity(Index eid, int /*pid*/ = -1) const {
        const sofa::type::vector<type::Vector3> & pos = d_position.getValue();

        if(d_normals.getValue().size()>eid)
            return FixedProximity(pos[eid],d_normals.getValue()[eid]);
        else
            return FixedProximity(pos[eid]);
    }

    //do not change the dataProximity.
    inline FixedProximity project(const type::Vector3 & /*Q*/, Index pid) const {
        const sofa::type::vector<type::Vector3> & pos = d_position.getValue();

        if(d_normals.getValue().size()>pid)
            return FixedProximity(pos[pid],d_normals.getValue()[pid]);
        else
            return FixedProximity(pos[pid]);
    }

    inline type::Vector3 getPosition(const FixedProximity & data, core::VecCoordId v) const {
        return data.getPosition(v);
    }

    inline type::Vector3 getNormal(const FixedProximity & data) const {
        return data.m_normal;
    }

    inline void buildJacobianConstraint(const FixedProximity & data, core::MultiMatrixDerivId cId, const sofa::type::vector<type::Vector3> & normals, double fact, Index constraintId) const {
        data.buildJacobianConstraint(cId,normals,fact,constraintId);
    }

    inline void storeLambda(const core::ConstraintParams* /*cParams*/, core::MultiVecDerivId /*resId*/, Index /*cid_global*/, Index /*cid_local*/, const sofa::defaulttype::BaseVector* /*lambda*/) const {}

    virtual void recomputeNormals() {}
};

}

}
