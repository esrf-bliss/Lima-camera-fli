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
#ifndef FLISHUTTERCTRLOBJ_H
#define FLISHUTTERCTRLOBJ_H

#include "FliCompatibility.h"
#include "lima/HwShutterCtrlObj.h"
#include "FliCamera.h"

namespace lima
{
    namespace Fli
    {

/*******************************************************************
 * \class ShutterCtrlObj
 * \brief Control object providing Fli shutter interface
 *******************************************************************/

	class LIBFLI_API ShutterCtrlObj : public HwShutterCtrlObj
	{
	    DEB_CLASS_NAMESPC(DebModCamera, "ShutterCtrlObj", "Fli");

	public:
	    ShutterCtrlObj(Camera& cam);
	    virtual ~ShutterCtrlObj();
    
	    virtual bool checkMode(ShutterMode shut_mode) const;
	    virtual void getModeList(ShutterModeList&  mode_list) const;
	    virtual void setMode(ShutterMode  shut_mode);
	    virtual void getMode(ShutterMode& shut_mode) const;

	    virtual void setState(bool  shut_open);
	    virtual void getState(bool& shut_open) const;

	private:
	    Camera& m_cam;
	};

    } // namespace Fli
} // namespace lima

#endif // FLISHUTTERCTRLOBJ_H
