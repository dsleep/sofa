/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2015 INRIA, USTL, UJF, CNRS, MGH                    *
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
*                               SOFA :: Plugins                               *
*                                                                             *
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_ODESOLVER_AAOMNISOLVER_H
#define SOFA_COMPONENT_ODESOLVER_AAOMNISOLVER_H
#define AA_OMNI_DRIVER_NAME_S "AAOmniDriver"

//Sensable include
#include "AAOmniDevice.h"
#include "hdDefines.h"
#include "hduVector.h"
#include <sofa/helper/LCPcalc.h>
#include <sofa/defaulttype/SolidTypes.h>
#include <sofa/defaulttype/RigidTypes.h>
#include <sofa/defaulttype/Vec.h>
#include <sofa/helper/Quater.h>


#include <sofa/core/behavior/BaseController.h>
#include <SofaOpenglVisual/OglModel.h>
#include <SofaRigid/RigidMapping.h>
#include <SofaUserInteraction/Controller.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/Simulation.h>
#include <SofaBaseMechanics/MechanicalObject.h>


//force feedback
#include <SofaHaptics/ForceFeedback.h>
#include <SofaHaptics/MechanicalStateForceFeedback.h>
#include <SofaHaptics/LCPForceFeedback.h>
#include <SofaHaptics/NullForceFeedbackT.h>

#include <sofa/simulation/Node.h>
#include <cstring>

#include <SofaOpenglVisual/OglModel.h>
#include <SofaSimulationTree/GNode.h>
#include <SofaBaseTopology/TopologyData.h>
#include <SofaBaseVisual/InteractiveCamera.h>

#include <math.h>

namespace sofa
{
namespace simulation { class Node; }

namespace component
{
namespace visualModel { class OglModel; }

namespace controller
{

class ForceFeedback;

using namespace sofa::defaulttype;
using core::objectmodel::Data;

/** Holds data retrieved from HDAPI. */
struct AADeviceData
{
    HHD id;
    int nupdates;
    uint16_t m_buttonState;					/* Has the device button has been pressed. */
    hduVector3Dd m_devicePosition;	/* Current device coordinates. */
    HDErrorInfo m_error;
    Vec3d pos;
    Quat quat;
    bool ready;
    bool stop;
};

struct AAOmniData
{
    ForceFeedback::SPtr forceFeedback;
    simulation::Node::SPtr *context;

    sofa::defaulttype::SolidTypes<double>::Transform endOmni_H_virtualTool;
    //Transform baseOmni_H_endOmni;
    sofa::defaulttype::SolidTypes<double>::Transform world_H_baseOmni;
    double forceScale;
    double scale;
    bool permanent_feedback;
	Vec3d desirePosition;
	bool move2Pos;
	double stiffness;

	// API OMNI //
    AADeviceData servoDeviceData;  // for the haptic loop
    AADeviceData deviceData;		 // for the simulation loop

    double currentForce[3];

};

struct AllAAOmniData
{
    std::vector<AAOmniData> omniData;
} ;

/**
* Omni driver
*/
class AAOmniDriver : public Controller
{

public:
    SOFA_CLASS(AAOmniDriver, Controller);
	static std::string getName()
		return AA_OMNI_DRIVER_NAME_S;
	typedef RigidTypes::Coord Coord;
    typedef RigidTypes::VecCoord VecCoord;
	typedef sofa::defaulttype::Vec3Types DataTypes;
    typedef component::container::MechanicalObject<sofa::defaulttype::Rigid3dTypes> MMechanicalObject;

    struct VisualComponent
    {
        simulation::Node::SPtr node;
        sofa::component::visualmodel::OglModel::SPtr visu;
        sofa::component::mapping::RigidMapping< Rigid3dTypes , ExtVec3fTypes  >::SPtr mapping;
    };

    Data<double> forceScale;
    Data<double> scale;
    Data<Vec3d> positionBase;
    Data<Quat> orientationBase;
    Data<Vec3d> positionTool;
    Data<Quat> orientationTool;
    Data<bool> permanent;
    Data<bool> omniVisu;
    Data< VecCoord > posDevice;
    Data< VecCoord > posStylus;
    Data< std::string > locDOF;
    Data< std::string > deviceName;
    Data< int > deviceIndex;
    Data<Vec1d> openTool;
    Data<double> maxTool;
    Data<double> minTool;
    Data<double> openSpeedTool;
    Data<double> closeSpeedTool;
    Data<bool> setRestShape;
    Data<bool> applyMappings;
    Data<bool> alignOmniWithCamera;
	Data<bool> stateButton1;
	Data<bool> stateButton2;
	Data<bool> setDesirePosition;
	Data<Vec3d> desirePosition;

	sofa::component::container::MechanicalObject<sofa::defaulttype::Rigid3dTypes>::SPtr DOF;
	sofa::component::container::MechanicalObject<sofa::defaulttype::Rigid3dTypes>::SPtr toolDOF;
	sofa::component::visualmodel::BaseCamera::SPtr camera;

    bool initVisu;

    AAOmniData data;
    AllAAOmniData allData;

    AAOmniDriver();
    virtual ~AAOmniDriver();

    virtual void init();
    virtual void bwdInit();
    virtual void reset();
    void reinit();

    int initDevice();

    void cleanup();
	virtual void draw(const core::visual::VisualParams*) override;
    virtual void draw();

    void setForceFeedback(ForceFeedback* ff);

    void onKeyPressedEvent(core::objectmodel::KeypressedEvent *);
    void onKeyReleasedEvent(core::objectmodel::KeyreleasedEvent *);
    void onAnimateBeginEvent();

    void setDataValue();

    //variable pour affichage graphique
    simulation::Node *parent;
    enum
    {
        VN_stylus = 0,
        VN_joint2 = 1,
        VN_joint1 = 2,
        VN_arm2   = 3,
        VN_arm1   = 4,
        VN_joint0 = 5,
        VN_base   = 6,
        VN_X      = 7,
        VN_Y      = 8,
        VN_Z      = 9,
        NVISUALNODE = 10
    };
    VisualComponent visualNode[NVISUALNODE];
    static const char* visualNodeNames[NVISUALNODE];
    static const char* visualNodeFiles[NVISUALNODE];
    simulation::Node::SPtr nodePrincipal;
    MMechanicalObject::SPtr rigidDOF;
    bool changeScale;
    bool firstInit;
    float oldScale;
    bool visuActif;
    bool isInitialized;
    Vec3d positionBase_buf;
    bool modX;
    bool modY;
    bool modZ;
    bool modS;
    bool axesActif;
    HDfloat angle1[3];
    HDfloat angle2[3];
    bool firstDevice;
    //vector<AAOmniDriver*> autreOmniDriver;
    AAOmniDevice* aaOmniDeviceHandle;
private:
    void handleEvent(core::objectmodel::Event *);
    bool noDevice;

};

} // namespace controller

} // namespace component

} // namespace sofa

#endif // SOFA_COMPONENT_ODESOLVER_AAOMNISOLVER_H
