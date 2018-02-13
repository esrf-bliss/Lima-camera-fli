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
#ifndef FLIINTERFACE_H
#define FLIINTERFACE_H

#include "FliCompatibility.h"
#include "lima/HwInterface.h"
#include "FliCamera.h"
#include "FliDetInfoCtrlObj.h"
#include "FliSyncCtrlObj.h"
#include "FliShutterCtrlObj.h"
#include "FliBinCtrlObj.h"
#include "FliRoiCtrlObj.h"


namespace lima
{
    namespace Fli
    {
/*******************************************************************
 * \class Interface
 * \brief Fli hardware interface
 *******************************************************************/

	class LIBFLI_API Interface : public HwInterface
	{
	    DEB_CLASS_NAMESPC(DebModCamera, "FliInterface", "Fli");

	public:
	    Interface(Camera& cam);
	    virtual ~Interface();

	    //- From HwInterface
	    virtual void    getCapList(CapList&) const;
	    virtual void    reset(ResetLevel reset_level);
	    virtual void    prepareAcq();
	    virtual void    startAcq();
	    virtual void    stopAcq();
	    virtual void    getStatus(StatusType& status);
	    virtual int     getNbHwAcquiredFrames();

	    // - From FliCamera
	    void setTemperatureSP(double temperature){m_cam.setTemperatureSP(temperature);};
	    void getTemperatureSP(double& temperature){m_cam.getTemperatureSP(temperature);};
	    void getTemperatureCCD (double& temperature){m_cam.getTemperatureCCD(temperature);};
	    void getTemperatureBase (double& temperature){m_cam.getTemperatureBase(temperature);};
	    void getCoolerPower(double& power){m_cam.getCoolerPower(power);};
	    void setShutterLevel(int level) {m_cam.setShutterLevel(level);}
	    void getShutterLevel(int& level) {m_cam.getShutterLevel(level);}
	    
	    Camera& getCamera() { return m_cam;}

	private:
	    Camera&         m_cam;
	    CapList         m_cap_list;
	    DetInfoCtrlObj  m_det_info;
	    SyncCtrlObj     m_sync;
	    BinCtrlObj      m_bin;
	    RoiCtrlObj      m_roi;
	    ShutterCtrlObj  m_shutter;
	    mutable Cond    m_cond;
	};



    } // namespace Fli
} // namespace lima

#endif // FLIINTERFACE_H
