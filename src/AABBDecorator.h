/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, version 1.0 RC 1        *
*                (c) 2006-2011 MGH, INRIA, USTL, UJF, CNRS                    *
*                                                                             *
* This library is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This library is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this library; if not, write to the Free Software Foundation,     *
* Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301 USA.          *
*******************************************************************************
*                               SOFA :: Modules                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_AABBDECORATOR_H
#define SOFA_COMPONENT_AABBDECORATOR_H

#include "CollisionAlgorithm.h"
#include "ConstraintGeometry.h"
#include <sofa/core/behavior/ForceField.h>
#include <sofa/core/behavior/MechanicalState.h>
#include <sofa/core/objectmodel/Data.h>
#include <sofa/defaulttype/VecTypes.h>
#include <SofaConstraint/BilateralInteractionConstraint.h>
#include "ConstraintProximity.h"

namespace sofa {

namespace core {

namespace behavior {

#define min3(a,b,c) std::min(std::min(a,b),c)
#define max3(a,b,c) std::max(std::max(a,b),c)

class AABBDecorator : public core::BehaviorModel, public BaseDecorator
{
public:
    SOFA_CLASS(AABBDecorator , core::BehaviorModel);

    typedef sofa::defaulttype::Vec3dTypes DataTypes;
    typedef DataTypes::VecCoord VecCoord;

    class AABBIterator : public BaseConstraintIterator {
    public :

        AABBIterator(AABBDecorator * deco,const ConstraintProximityPtr & src) {
            m_decorator = deco;
            d = 0;

            defaulttype::Vector3 T = src->getPosition();

            cbox[0] = floor((T[0] - m_decorator->m_Bmin[0])/m_decorator->m_cellSize[0]);
            cbox[1] = floor((T[1] - m_decorator->m_Bmin[1])/m_decorator->m_cellSize[1]);
            cbox[2] = floor((T[2] - m_decorator->m_Bmin[2])/m_decorator->m_cellSize[2]);

            //project P in the bounding box of the pbject
            //search with the closest box in bbox
            for (int i=0;i<3;i++) {
                if (cbox[i]<0) cbox[i] = 0;
                else if (cbox[i]>m_decorator->d_nbox.getValue()[i]) cbox[i] = m_decorator->d_nbox.getValue()[i];
            }

            m_max = max3(m_decorator->d_nbox.getValue()[0],
                         m_decorator->d_nbox.getValue()[1],
                         m_decorator->d_nbox.getValue()[2]);

            fillTriangleSet(d);
        }

        void fillTriangleSet(int d) {
            m_selectTriangle.clear();

            for (int i=-d;i<=d;i++) {
                if (cbox[0]+i < 0 || cbox[0]+i > m_decorator->d_nbox.getValue()[0]) continue;

                for (int j=-d;j<=d;j++) {
                    if (cbox[1]+j < 0 || cbox[1]+j > m_decorator->d_nbox.getValue()[1]) continue;

                    for (int k=-d;k<=d;k++) {
                        if (cbox[2]+k < 0 || cbox[2]+k > m_decorator->d_nbox.getValue()[2]) continue;

                        if (i!=d && j!=d && k!=d) continue; // already seen

                        const helper::vector<unsigned> & triangles = m_decorator->m_triangleboxes[cbox[0] + i][cbox[1] + j][cbox[2] + k];
                        for (unsigned t=0;t<triangles.size();t++) m_selectTriangle.insert(triangles[t]);
                    }
                }
            }

            m_setIterator = m_selectTriangle.begin();
        }

        bool end(const ConstraintProximityPtr & E = NULL) {
            if (d>=m_max) return true;
            if (m_selectTriangle.empty() || (m_setIterator == m_selectTriangle.end())) {
                if (E != NULL) return true; // the proximity is not empty i.e. we already fond a closer bindind
                d++;// we look for boxed located at d+1
                fillTriangleSet(d);
            }
            return false;
        }

        int getElement() {
            return *m_setIterator;
        }

        void next() {
            if (m_selectTriangle.size()) m_setIterator++;
        }

    private :
        unsigned d,m_max;
        AABBDecorator * m_decorator;
        std::set<int> m_selectTriangle;
        std::set<int>::iterator m_setIterator;
        defaulttype::Vec3i cbox; //box in which is located the src
    };

    AABBDecorator();

    void draw(const core::visual::VisualParams */*vparams*/);

    Data<defaulttype::Vec3i> d_nbox;
    Data<bool> d_drawBbox;

    virtual void init();

    virtual void reinit();

    virtual void prepareDetection();

    BaseConstraintIteratorPtr getIterator(const ConstraintProximityPtr & P = NULL);

    void updatePosition(SReal /*dt*/) {
        prepareDetection();
    }

protected:
    defaulttype::Vector3 m_Bmin,m_Bmax,m_cellSize;
    helper::vector<helper::vector<helper::vector<helper::vector<unsigned> > > >  m_triangleboxes;
};






} // namespace forcefield

} // namespace component

} // namespace sofa


#endif // NeedleLinearDescription_H
