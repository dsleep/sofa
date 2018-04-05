/******************************************************************************
*       SOFA, Simulation Open-Framework Architecture, development version     *
*                (c) 2006-2018 INRIA, USTL, UJF, CNRS, MGH                    *
*                                                                             *
* This program is free software; you can redistribute it and/or modify it     *
* under the terms of the GNU Lesser General Public License as published by    *
* the Free Software Foundation; either version 2.1 of the License, or (at     *
* your option) any later version.                                             *
*                                                                             *
* This program is distributed in the hope that it will be useful, but WITHOUT *
* ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or       *
* FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License *
* for more details.                                                           *
*                                                                             *
* You should have received a copy of the GNU Lesser General Public License    *
* along with this program. If not, see <http://www.gnu.org/licenses/>.        *
*******************************************************************************
* Authors: The SOFA Team and external contributors (see Authors.txt)          *
*                                                                             *
* Contact information: contact@sofa-framework.org                             *
******************************************************************************/
#ifndef SOFA_COMPONENT_MISC_PAUSEANIMATIONONEVENT_H
#define SOFA_COMPONENT_MISC_PAUSEANIMATIONONEVENT_H
#include "config.h"

#include <SofaGraphComponent/PauseAnimation.h>

#include <fstream>

namespace sofa
{

namespace component
{

namespace misc
{

/**
*/
class PauseAnimationOnEvent : public PauseAnimation
{
public:
    SOFA_CLASS(PauseAnimationOnEvent,PauseAnimation);
protected:
    PauseAnimationOnEvent();

    virtual ~PauseAnimationOnEvent();
public:
    virtual void init() override;

    bool paused;
    bool isPaused() override;

    virtual void handleEvent(sofa::core::objectmodel::Event* event) override;

};


} // namespace misc

} // namespace component

} // namespace sofa

#endif
