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
#ifndef FLICAMERA_H
#define FLICAMERA_H

#include <stdlib.h>
#include <limits>
#include "lima/HwMaxImageSizeCallback.h"
#include "lima/HwBufferMgr.h"

#include <ostream>
#include <string>

#include <libfli.h>

using namespace std;


namespace lima
{
    namespace Fli
    {

/*******************************************************************
 * \class Camera
 * \brief object controlling the FLI camera via FLI SDK (libfli)
 *******************************************************************/
	class LIBFLI_API Camera
	{
	    DEB_CLASS_NAMESPC(DebModCamera, "Camera", "Fli");
	    friend class Interface;
	public:

	    enum FrameType {
	      FrameTypeNormal = FLI_FRAME_TYPE_NORMAL,
	      FrameTypeDark = FLI_FRAME_TYPE_DARK
	    };

	    enum TemperatureChannel {
	      TemperatureChannelInternal = FLI_TEMPERATURE_INTERNAL,
	      TemperatureChannelExternal = FLI_TEMPERATURE_EXTERNAL,
	      TemperatureChannelCcd      = FLI_TEMPERATURE_CCD,
	      TemperatureChannelBase     = FLI_TEMPERATURE_BASE
	    };

	    enum Status {
	      StatusUnknown = FLI_CAMERA_STATUS_UNKNOWN,
	      StatusMask    = FLI_CAMERA_STATUS_MASK,
	      StatusIdle    = FLI_CAMERA_STATUS_IDLE,
	      StatusWaitingForTrigger = FLI_CAMERA_STATUS_WAITING_FOR_TRIGGER,
	      StatusExposing = FLI_CAMERA_STATUS_EXPOSING,
	      StatusDataReady = FLI_CAMERA_DATA_READY
	    };

	    enum DebugLevel {
	      DebugNone = FLIDEBUG_NONE,
	      DebugInfo = FLIDEBUG_INFO,
	      DebugWarn = FLIDEBUG_WARN,
	      DebugFail = FLIDEBUG_FAIL,
	      DebugIO   = FLIDEBUG_IO,
	      DebugAll  = FLIDEBUG_ALL 
	    };

	    enum FanSpeed {
	      FanSpeedOn  = FLI_FAN_SPEED_ON,
	      FanSpeedOff = FLI_FAN_SPEED_OFF
	    };

	    Camera(int camera_number=0);
	    ~Camera();

	    void prepareAcq();
	    void startAcq();
	    void stopAcq();
    
	    // -- detector info object
	    void getImageType(ImageType& type);
	    void setImageType(ImageType type);

	    void getDetectorType(std::string& type);
	    void getDetectorModel(std::string& model);
	    void getDetectorImageSize(Size& size);
    
	    // -- Buffer control object
	    HwBufferCtrlObj* getBufferCtrlObj();
    
	    //-- Synch control object
	    bool checkTrigMode(TrigMode trig_mode);
	    void setTrigMode(TrigMode  mode);
	    void getTrigMode(TrigMode& mode);
    
	    void setExpTime(double  exp_time);
	    void getExpTime(double& exp_time);

	    void setLatTime(double  lat_time);
	    void getLatTime(double& lat_time);

	    void getExposureTimeRange(double& min_expo, double& max_expo) const;
	    void getLatTimeRange(double& min_lat, double& max_lat) const;    

	    void setNbFrames(int  nb_frames);
	    void getNbFrames(int& nb_frames);
	    void getNbHwAcquiredFrames(int &nb_acq_frames);

	    void checkRoi(const Roi& set_roi, Roi& hw_roi);
	    void setRoi(const Roi& set_roi);
	    void getRoi(Roi& hw_roi);    

	    void checkBin(Bin&);
	    void setBin(const Bin&);
	    void getBin(Bin&);
	    bool isBinningAvailable();
    
	    void setShutter(bool flag);
	    void getShutter(bool& flag);
	    void setShutterMode(ShutterMode mode);
	    void getShutterMode(ShutterMode& mode);
    

	    void getPixelSize(double& sizex, double& sizey);
    
	    void getStatus(Camera::Status& status);
    
	    void reset();

	    // -- Fli specific
	    void getFrameType(FrameType& frametype);
	    void setFrameType(FrameType frametype);
	    void getVersions(std::string& version);
	    void setTemperatureSP(double temperature);
	    void getTemperatureSP(double& temperature);
	    void getTemperatures (double& internal, double& external, double& ccd, double& base);
	    void getCoolerPower(double& power);
	    void getIOPort(long& ioport);
	    void setIOPort(long ioport);
	    void getIOPortDirection(long& direction);
	    void setIOPortDirection(long direction);
	    

	private:
	    class _AcqThread;
	    friend class _AcqThread;
	    void _stopAcq(bool);
	    void _setStatus(Camera::Status status,bool force);

	    //- acquisition thread stuff    
	    _AcqThread*                 m_acq_thread;
	    Cond                        m_cond;

	    //- lima stuff
	    SoftBufferCtrlObj	        m_buffer_ctrl_obj;
	    int                         m_nb_frames;    
	    Camera::Status              m_status;
	    volatile bool               m_wait_flag;
	    volatile bool               m_quit;
	    volatile bool               m_thread_running;
	    int                         m_image_number;
	    int                         m_timeout;
	    double                      m_latency_time;
	    Roi                         m_roi;
	    Bin                         m_bin;
	    Bin                         m_bin_max;
	    TrigMode                    m_trig_mode;

	    ShutterMode                 m_shutter_mode;
	    bool                        m_shutter_state;
        
	    //- camera stuff 
	    string                      m_detector_model;
	    string                      m_detector_type;
	    int                         m_detector_serial;
	    float                       m_exp_time;
    
	    //- Fli SDK stuff
	    flidev_t                    m_device;
	    double                      m_temperature_sp;

	};
    } // namespace Fli
} // namespace lima


#endif // FLICAMERA_H
