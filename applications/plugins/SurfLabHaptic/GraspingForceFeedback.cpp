#include "GraspingForceFeedback.h"

namespace sofa
{

	namespace component
	{

		namespace controller
		{
			void GraspingForceFeedback::init()
			{
				this->ForceFeedback::init();
			}
			void GraspingForceFeedback::computeForce(SReal x, SReal y, SReal z, SReal u, SReal v, SReal w, SReal q, SReal& fx, SReal& fy, SReal& fz)
			{
        const sofa::core::objectmodel::Data<VecDeriv> *d = this->state->read(core::ConstVecDerivId::force());
        if (d != NULL && d->getValue().size() > 0)
        {
          const defaulttype::Vec3d f = d->getValue()[0];
          fx = f[0] * scale;
          fy = f[1] * scale;
          fz = f[2] * scale;
        }
        else
        {
          fx = 0; 
          fy = 0;
          fz = 0;
        }
        
			}
			void GraspingForceFeedback::computeWrench(const defaulttype::SolidTypes<SReal>::Transform &world_H_tool, const defaulttype::SolidTypes<SReal>::SpatialVector &V_tool_world, defaulttype::SolidTypes<SReal>::SpatialVector &W_tool_world)
			{

			}
		}
	}
}