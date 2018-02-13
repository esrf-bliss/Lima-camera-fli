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
#include "FliCompatibility.h"

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
	    enum Status
	    {
	      Ready, Exposure, Readout, Latency, Fault
	    };
	    /*	    enum Status {
	      StatusUnknown = FLI_CAMERA_STATUS_UNKNOWN,
	      StatusMask    = FLI_CAMERA_STATUS_MASK,
	      StatusIdle    = FLI_CAMERA_STATUS_IDLE,
	      StatusWaitingForTrigger = FLI_CAMERA_STATUS_WAITING_FOR_TRIGGER,
	      StatusExposing = FLI_CAMERA_STATUS_EXPOSING,
	      StatusDataReady = FLI_CAMERA_DATA_READY
	    };
	    */
	    enum DebugLevel {
	      DebugNone = FLIDEBUG_NONE,
	      DebugInfo = FLIDEBUG_INFO,
	      DebugWarn = FLIDEBUG_WARN,
	      DebugFail = FLIDEBUG_FAIL,
	      DebugIO   = FLIDEBUG_IO,
	      DebugAll  = FLIDEBUG_ALL 
	    };


	    Camera(const std::string& camera_path);
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
	    void setTemperatureSP(double temperature);
	    void getTemperatureSP(double& temperature);
	    void getTemperatureCCD (double& temperature);
	    void getTemperatureBase (double& temperature);
	    void getCoolerPower(double& power);
	    void setShutterLevel(int level);
	    void getShutterLevel(int& level);
	    //void getIOPort(long& ioport);
	    //void setIOPort(long ioport);
	    //void getIOPortDirection(long& direction);
	    //void setIOPortDirection(long direction);
	    

	private:
	    class CameraThread: public CmdThread
	    {
	      DEB_CLASS_NAMESPC(DebModCamera, "CameraThread", "Lambda");
	    public:
	      enum
	      { // Status
		Ready = MaxThreadStatus, Exposure, Readout, Latency,
	      };
	      
	      enum
	      { // Cmd
		StartAcq = MaxThreadCmd, StopAcq,
	      };
	      
	      CameraThread(Camera& cam);	      
	      virtual void start();
	      bool m_force_stop;
	      
	    protected:	      
	      virtual void init();
	      virtual void execCmd(int cmd);
	    private:
	      void execStartAcq();
	      void execStopAcq();
	      Camera* m_cam;
	      
	    };
	    friend class CameraThread;
	    
	    //- acquisition thread stuff    
	    CameraThread                m_thread;

	    //- lima stuff
	    SoftBufferCtrlObj	        m_buffer_ctrl_obj;
	    int                         m_nb_frames;    
	    Camera::Status              m_status;
	    int                         m_image_number;
	    int                         m_timeout;
	    double                      m_latency_time;
	    Roi                         m_roi;
	    Bin                         m_bin;
	    Bin                         m_bin_max;
	    TrigMode                    m_trig_mode;
	    int                         m_acq_frame_nb;
	    ShutterMode                 m_shutter_mode;
	    bool                        m_shutter_state;
        
	    //- camera stuff 
	    string                      m_detector_model;
	    string                      m_detector_type;
	    string                      m_detector_serial;
	    long                        m_detector_hw_rev;
	    long                        m_detector_fw_rev;
	    pair<double, double>        m_pixel_size;
	    pair<Point, Point>          m_array_area;
	    pair<Point, Point>          m_visible_area;
	    float                       m_exp_time;
    
	    //- Fli SDK stuff
	    flidev_t                    m_device;
	    flibitdepth_t               m_bit_depth;
	    flishutter_t                m_shutter_level;
	    double                      m_temperature_sp;

	};
    } // namespace Fli
} // namespace lima


#endif // FLICAMERA_H
