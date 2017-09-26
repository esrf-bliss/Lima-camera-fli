//###########################################################################
// This file is part of LImA, a Library for Image Acquisition
//
// Copyright (C) : 2009-2017
// European Synchrotron Radiation Facility
// BP 220, Grenoble 38043
// FRANCE
//
// This is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 3 of the License, or
// (at your option) any later version.
//
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, see <http://www.gnu.org/licenses/>.
//###########################################################################
#include "FliShutterCtrlObj.h"

using namespace lima;
using namespace lima::Fli;
using namespace std;

//-----------------------------------------------------
// @brief Ctor
//-----------------------------------------------------
ShutterCtrlObj::ShutterCtrlObj(Camera& cam)
    : m_cam(cam)
{
    DEB_CONSTRUCTOR();
}

//-----------------------------------------------------
// @brief Dtor
//-----------------------------------------------------
ShutterCtrlObj::~ShutterCtrlObj()
{
    DEB_DESTRUCTOR();
}

//-----------------------------------------------------
// @brief return true if the mode is valid
//-----------------------------------------------------
bool ShutterCtrlObj::checkMode(ShutterMode shut_mode) const
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(shut_mode);
    
    bool valid_mode;
    switch (shut_mode) 
    {	
    case ShutterAutoFrame:
    case ShutterManual:
	valid_mode = true;
	break;
    default:
	// No AutoSequence mode for Fli !
	valid_mode = false;
    }
    
    DEB_RETURN() << DEB_VAR1(valid_mode);
    return valid_mode;
}

//-----------------------------------------------------
// @brief return the shutter valid mode list
//-----------------------------------------------------
void ShutterCtrlObj::getModeList(ShutterModeList& mode_list) const
{
    DEB_MEMBER_FUNCT();
    mode_list.push_back(ShutterAutoFrame);
    mode_list.push_back(ShutterManual);
}

//-----------------------------------------------------
// @brief set the shutter mode
//-----------------------------------------------------
void ShutterCtrlObj::setMode(ShutterMode shut_mode)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(shut_mode);
    
    if (!checkMode(shut_mode))
	THROW_HW_ERROR(InvalidValue) << "Invalid " 
				     << DEB_VAR1(shut_mode);
    
    Camera::ShutterMode cam_mode;
    cam_mode = (shut_mode == ShutterAutoFrame) ? Camera::FRAME : Camera::MANUAL;
    m_cam.setShutterMode(cam_mode);
}

//-----------------------------------------------------
// @brief return the shutter mode
//-----------------------------------------------------
void ShutterCtrlObj::getMode(ShutterMode& shut_mode) const
{
    DEB_MEMBER_FUNCT();
    
    Camera::ShutterMode cam_mode;
    m_cam.getShutterMode(cam_mode);
    shut_mode = (cam_mode == Camera::FRAME) ? ShutterAutoFrame : ShutterManual;
    DEB_RETURN() << DEB_VAR1(shut_mode);
}

//-----------------------------------------------------
// @brief open or close manually the shutter
//-----------------------------------------------------
void ShutterCtrlObj::setState(bool open)
{
    DEB_MEMBER_FUNCT();
    m_cam.setShutter(open);
}

//-----------------------------------------------------
// @brief return the shutter state, valid if the shutter
// is in manual mode
//-----------------------------------------------------
void ShutterCtrlObj::getState(bool& open) const
{
    DEB_MEMBER_FUNCT();
    m_cam.getShutter(open);
}

//-----------------------------------------------------
// @brief set the shutter opening time
//-----------------------------------------------------
void ShutterCtrlObj::setOpenTime(double shut_open_time)
{
    DEB_MEMBER_FUNCT();			
}

//-----------------------------------------------------
// @brief return the shutter opening time
//-----------------------------------------------------
void ShutterCtrlObj::getOpenTime(double& shut_open_time) const
{
    DEB_MEMBER_FUNCT();
}

//-----------------------------------------------------
// @brief set the shutter closing time
//-----------------------------------------------------
void ShutterCtrlObj::setCloseTime(double shut_close_time)
{
    DEB_MEMBER_FUNCT();
}

//-----------------------------------------------------
// @brief return the shutter closing time
//-----------------------------------------------------
void ShutterCtrlObj::getCloseTime(double& shut_close_time) const
{
    DEB_MEMBER_FUNCT();
}
