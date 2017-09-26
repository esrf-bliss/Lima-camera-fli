///###########################################################################
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
//############################################################################

#include <sstream>
#include <iostream>
#include <string>
#include <math.h>
#include <unistd.h>

#include "FliCamera.h"

using namespace lima;
using namespace lima::Fli;
using namespace std;

//---------------------------
//- utility function
//---------------------------
//static methods predefinition
#define THROW_IF_NOT_SUCCESS(command,error_prefix)			\
{									\
  long ret_code = command;						\
  if ( ret_code != 0 )						\
    THROW_HW_ERROR(Error) << error_prefix << DEB_VAR1( strerror((int)-error_code) ); \
}
//---------------------------
//- utility thread
//---------------------------

class Camera::_AcqThread : public Thread
{
    DEB_CLASS_NAMESPC(DebModCamera, "Camera", "_AcqThread");
public:
    _AcqThread(Camera &aCam);
    virtual ~_AcqThread();
    
protected:
    virtual void threadFunction();
    
private:
    Camera&    m_cam;
};


//---------------------------
// @brief  Ctor
//---------------------------
Camera::Camera(int camera_number)
    : m_status(Ready),
      m_wait_flag(true),
      m_quit(false),
      m_thread_running(true),
      m_image_number(0),
      m_latency_time(0.),
      m_bin(1,1),
      m_shutter_state(false),
      m_fan_mode(FAN_UNSUPPORTED),
      m_exp_time(1.)
{
    DEB_CONSTRUCTOR();
    m_camera_number = camera_number;
  
    // --- Get available cameras and select the choosen one.
    int numCameras;
    DEB_TRACE() << "Get all attached cameras";
    THROW_IF_NOT_SUCCESS(GetAvailableCameras(&numCameras), "No camera present!");
    
    DEB_TRACE() << "Found "<< numCameras << " camera" << ((numCameras>1)? "s": "");
    DEB_TRACE() << "Try to set current camera to number " << m_camera_number;
    
    if (m_camera_number < numCameras && m_camera_number >=0)
    {        
        THROW_IF_NOT_SUCCESS(GetCameraHandle(m_camera_number, &m_camera_handle),"Cannot get camera handle");
	THROW_IF_NOT_SUCCESS(SetCurrentCamera(m_camera_handle), "Cannot set camera handle");
    }
    else
    {
	DEB_ERROR() << "Invalid camera number " << m_camera_number << ", there is "<< numCameras << " available";
	THROW_HW_ERROR(InvalidValue) << "Invalid Camera number ";
    }


    // --- Initialize  the library    
    THROW_IF_NOT_SUCCESS(Initialize((char *)m_config_path.c_str()), "Library initialization failed, check the config. path");
    
    // --- Get Camera model
    char	model[AT_CONTROLLER_CARD_MODEL_LEN];
    int         serial;
    THROW_IF_NOT_SUCCESS(GetHeadModel(model), "Cannot get camera model");
    THROW_IF_NOT_SUCCESS(GetCameraSerialNumber(&serial), "Cannot get camera serial number");

    m_detector_model = model;
    m_detector_serial = serial;
    m_detector_type = m_andor_type_maps[m_camera_capabilities.ulCameraType];
    
    DEB_TRACE() << "Fli Camera device found:\n" 
		<< "    * Type     : " << m_detector_type << "("
		<< m_camera_capabilities.ulCameraType <<")\n"
		<< "    * Model    : " << m_detector_model <<"\n"
		<< "    * Serial # : " << m_detector_serial;

    
    // --- Initialise deeper parameters of the controller                
    initialiseController();            

    // Fan off, HighCapacity mode and Baseline clamping off as default if supported
    if (m_camera_capabilities.ulSetFunctions & AC_SETFUNCTION_HIGHCAPACITY)
      setHighCapacity(HIGH_CAPACITY);
    if (m_camera_capabilities.ulFeatures & AC_FEATURES_FANCONTROL)
      setFanMode(FAN_OFF);
    if (m_camera_capabilities.ulSetFunctions & AC_SETFUNCTION_BASELINECLAMP)
      setBaselineClamp(BLCLAMP_DISABLED);

    //--- Set detector for single image acquisition and get max binning
    m_read_mode = 4;
    THROW_IF_NOT_SUCCESS(SetReadMode(m_read_mode), "Cannot camera read mode");
 
    int xbin_max, ybin_max;   
    THROW_IF_NOT_SUCCESS(GetMaximumBinning(m_read_mode, 0, &xbin_max), "Cannot get the horizontal maximum binning");
    THROW_IF_NOT_SUCCESS(GetMaximumBinning(m_read_mode, 1, &ybin_max), "Cannot get the vertical maximum binning");

    m_bin_max = Bin(xbin_max, ybin_max);   
    DEB_TRACE() << "Maximum binning : " << xbin_max << " x " << ybin_max;                   

    // --- set default ROI because there is no way to read bck th image size
    // --- BIN already set to 1,1 above.
    // --- Fli sets the ROI by starting coordinates at 1 and not 0 !!!!
    Size sizeMax;
    getDetectorImageSize(sizeMax);
    Roi aRoi = Roi(0,0, sizeMax.getWidth(), sizeMax.getHeight());    
    
    // --- setRoi applies both bin and roi
    DEB_TRACE() << "Set the ROI to full frame: "<< aRoi;
    setRoi(aRoi);
    
    
    // --- Get the maximum exposure time allowed and set default
    THROW_IF_NOT_SUCCESS(GetMaximumExposure(&m_exp_time_max), "Cannot get the maximum exposure time");
    DEB_TRACE() << "Maximum exposure time : "<< m_exp_time_max << "sec.";
    
    setExpTime(m_exp_time);
    
    // --- Set detector for software single image mode    
    m_trig_mode_maps[IntTrig] = 0;
    m_trig_mode_maps[ExtTrigMult] = 1;
    m_trig_mode_maps[ExtTrigSingle] = 6;
    m_trig_mode_maps[ExtGate] = 7;
    m_trig_mode_maps[IntTrigMult] = 10;  
    setTrigMode(IntTrig);
    
    // --- Set the Fli specific acquistion mode.
    // --- We set acquisition mode to run-till-abort 
    m_acq_mode = 5; //Run Till Abort
    m_nb_frames = 1;
    THROW_IF_NOT_SUCCESS(SetAcquisitionMode(m_acq_mode), "Cannot set the acquisition mode");
     // --- set shutter mode to FRAME
    setShutterMode(FRAME);        
    
    // --- finally start the acq thread
    m_acq_thread = new _AcqThread(*this);
    m_acq_thread->start();
}

//---------------------------
// @brief  Dtor
//---------------------------
Camera::~Camera()
{
    DEB_DESTRUCTOR();
    // Stop Acq thread
    delete m_acq_thread;
    m_acq_thread = NULL;
                
    // Close camera
    if (m_cooler)
    {
	DEB_ERROR() <<"Please stop the cooling before shuting dowm the camera\n"                             
		    << "brutale heating could damage the sensor.\n"
		    << "And wait until temperature rises above 5 deg, before shuting down.";

	THROW_HW_ERROR(Error)<<"Please stop the cooling before shuting dowm the camera\n"                             
			     << "brutale heating could damage the sensor.\n"
			     << "And wait until temperature rises above 5 deg, before shuting down.";
    }
    
    DEB_TRACE() << "Shutdown camera";
    ShutDown();
    m_camera_handle = 0;
}

//---------------------------
// @brief  prepare the acquistion
//---------------------------
void Camera::prepareAcq()
{
    DEB_MEMBER_FUNCT();
    m_image_number=0;
    
    THROW_IF_NOT_SUCCESS(PrepareAcquisition(), "Cannot prepare acquisition");
}
//---------------------------
// @brief  start the acquistion
//---------------------------
void Camera::startAcq()
{
    DEB_MEMBER_FUNCT();
        
    // --- check first the acquisition is idle
    int status;
    THROW_IF_NOT_SUCCESS(GetStatus(&status), "Cannot get status");
    if (status != DRV_IDLE)
    {
        _setStatus(Camera::Fault,false);        
        THROW_HW_ERROR(Error) << "Cannot start acquisition, camera is not idle";            
    }   
    
    // --- Don't forget to request the maximum number of images the circular buffer can store
    // --- based on the current acquisition settings.
    THROW_IF_NOT_SUCCESS(GetSizeOfCircularBuffer(&m_ring_buffer_size), "Cannot get size of circular buffer");

    DEB_TRACE() << "Fli Circular buffer size = " << m_ring_buffer_size << " images";
            
    // Wait running stat of acquisition thread
    AutoMutex aLock(m_cond.mutex());
    m_wait_flag = false;
    m_cond.broadcast();
    while(!m_thread_running)
        m_cond.wait();
       
    if(m_image_number == 0)
    {
        StdBufferCbMgr& buffer_mgr = m_buffer_ctrl_obj.getBuffer();
	buffer_mgr.setStartTimestamp(Timestamp::now());
	THROW_IF_NOT_SUCCESS(StartAcquisition(), "Cannot start acquisition");
    }
    if (m_trig_mode == IntTrigMult)
      THROW_IF_NOT_SUCCESS(SendSoftwareTrigger(), "Cannot start acquisition");

}

//---------------------------
// @brief stop the acquisition
//---------------------------
void Camera::stopAcq()
{
    _stopAcq(false);
}

//---------------------------
// @brief private method
//---------------------------
void Camera::_stopAcq(bool internalFlag)
{
    DEB_MEMBER_FUNCT();

    AutoMutex aLock(m_cond.mutex());
    if(m_status != Camera::Ready)
    {
        while(!internalFlag && m_thread_running)
        {
	    // signal the acq. thread to stop acquiring and to return the wait state
            m_wait_flag = true;

	    // Thread is maybe waiting for the Fli acq. event
	    THROW_IF_NOT_SUCCESS(CancelWait(), "CancelWait() failed");
            m_cond.wait();
        }
	aLock.unlock();

        //Let the acq thread stop the acquisition
        if(!internalFlag) return;
            
        // Stop acquisition
        DEB_TRACE() << "Stop acquisition";
        THROW_IF_NOT_SUCCESS(AbortAcquisition(), "Cannot abort acquisition");
        _setStatus(Camera::Ready,false);    
    }
}
//---------------------------
// @brief the thread function for acquisition
//---------------------------
void Camera::_AcqThread::threadFunction()
{
    DEB_MEMBER_FUNCT();
    AutoMutex aLock(m_cam.m_cond.mutex());
    StdBufferCbMgr& buffer_mgr = m_cam.m_buffer_ctrl_obj.getBuffer();

    while(!m_cam.m_quit)
    {
        while(m_cam.m_wait_flag && !m_cam.m_quit)
        {
            DEB_TRACE() << "Wait";
            m_cam.m_thread_running = false;
            m_cam.m_cond.broadcast();
            m_cam.m_cond.wait();
        }
        DEB_TRACE() << "Run";
        m_cam.m_thread_running = true;
        if(m_cam.m_quit) return;
    
        m_cam.m_status = Camera::Exposure;
        m_cam.m_cond.broadcast();
        aLock.unlock();

        bool continueAcq = true;
	
#if defined(WIN32)
	long first = 0, last = 0, prev_last = 0;		
	long validfirst, validlast;
#else
	int first = 0, last = 0, prev_last = 0;
	int validfirst, validlast;	
#endif	
	FrameDim frame_dim = buffer_mgr.getFrameDim();
	Size  frame_size = frame_dim.getSize();
	int size = frame_size.getWidth() * frame_size.getHeight();
	int ret;

        while(continueAcq && (!m_cam.m_nb_frames || m_cam.m_image_number < m_cam.m_nb_frames))
        {
	    // Check first if acq. has been stopped
	    if (m_cam.m_wait_flag) 
	    {
		continueAcq = false;
		continue;
	    }
	    // Wait for an "acquisition" event, and use less cpu resources, in kinetic mode (multiframe)
            // an event is generated for each new image
	    if ((ret = WaitForAcquisition()) != DRV_SUCCESS)
	    {
		// If CancelWait() or acq. not started yet
		if(ret == DRV_NO_NEW_DATA) continue;
		else 
		{
		  DEB_ERROR() << "WaitForAcquisition() failed" << " : error code = " << error_code(ret);
		    THROW_HW_ERROR(Error) << "WaitForAcquisition() failed";
		}
	    }

            // --- Get the available images in cicular buffer            
            prev_last = last;
            if ((ret = GetNumberNewImages(&first, &last)) != DRV_SUCCESS)
            {
                if (ret == DRV_NO_NEW_DATA) continue;
                else
                {
		  DEB_ERROR() << "Cannot get number of new images" << " : error code = " << error_code(ret);
                    THROW_HW_ERROR(Error) << "Cannot get number of new images";
                }
            }        
            DEB_TRACE() << "Available images: first = " << first << " last = " << last;
            // Check if we lose an image
	    if(first != prev_last +1 )
	    {
		m_cam._setStatus(Camera::Fault,false);
		continueAcq = false;
		DEB_ERROR() << "Lost image(s) from " << prev_last << "to "<< first-1;
		THROW_HW_ERROR(Error) << "Lost image(s) from " << prev_last << "to "<< first-1;	
	    }
            // --- Images are available, process images
            m_cam._setStatus(Camera::Readout,false);
	    
            for (long im=first; im <= last; im++)
            {
                DEB_TRACE()  << "image #" << m_cam.m_image_number <<" acquired !";
                // ---  must get image one by one to copy to the buffer manager
                void *ptr = buffer_mgr.getFrameBufferPtr(m_cam.m_image_number);
                
                if ((ret=GetImages16(im, im,(unsigned short*) ptr, (unsigned long)size,&validfirst, &validlast))!= DRV_SUCCESS)
                {
                    m_cam._setStatus(Camera::Fault,false);
                    continueAcq = false;
                    DEB_TRACE() << "size = " << size;
                    DEB_ERROR() << "Cannot get image #" << im << " : error code = " << error_code(ret);
                    THROW_HW_ERROR(Error) << "Cannot get last image";                
                }
                HwFrameInfoType frame_info;
                frame_info.acq_frame_nb = m_cam.m_image_number;
                continueAcq = buffer_mgr.newFrameReady(frame_info);
                DEB_TRACE() << DEB_VAR1(continueAcq);
                ++m_cam.m_image_number;
            }
        }

        m_cam._stopAcq(true);

        aLock.lock();
        m_cam.m_wait_flag = true;
    }
}

//-----------------------------------------------------
// @brief the acquisition thread Ctor
//-----------------------------------------------------
Camera::_AcqThread::_AcqThread(Camera &aCam) :
    m_cam(aCam)
{
    pthread_attr_setscope(&m_thread_attr,PTHREAD_SCOPE_PROCESS);
}
//-----------------------------------------------------
// @brief the acquisition thread Dtor
//-----------------------------------------------------
Camera::_AcqThread::~_AcqThread()
{
    AutoMutex aLock(m_cam.m_cond.mutex());
    m_cam.m_quit = true;
    m_cam.m_cond.broadcast();
    aLock.unlock();
    
    join();
}

//-----------------------------------------------------
// brief return the detector image size 
//-----------------------------------------------------
void Camera::getDetectorImageSize(Size& size)
{
    DEB_MEMBER_FUNCT();
    int xmax, ymax;
    
    // --- Get the max image size of the detector
    THROW_IF_NOT_SUCCESS(GetDetector(&xmax, &ymax), "Cannot get detector size");
    size= Size(xmax, ymax);
}


//-----------------------------------------------------
// @brief return the image type 
//-----------------------------------------------------
void Camera::getImageType(ImageType& type)
{
    DEB_MEMBER_FUNCT();
    int bits;
    // --- Get the AD channel dynamic range in bits per pixel
    // --- suppose here channel 0 is set, in fact we do not provide any
    // --- command to select a different ADC if the detector has several.

    
    THROW_IF_NOT_SUCCESS(GetBitDepth(m_adc_speeds[m_adc_speed_index].adc, &bits), "Cannot get detector bit depth");
    // --- not clear from documentation with bit depth are possible
    // --- according to the FliCapabilites structure cameras can support more image type
    // --- with color ones as well.
 
 
    if (bits <=8) type = Bpp8;
    else if (bits <=16) type = Bpp16;
    else type = Bpp32;
       
    // --- previous code do not return the data image type.
    // --- can either be bpp16 or bpp32, just depends on the function used
    // --- to read the image (GetImages()- 32bpp, GetImage16() - 16bpp
    // ---  will later on add support for 32bpp if requested.

}

//-----------------------------------------------------
// @brief set the image type, if supported
//-----------------------------------------------------
void Camera::setImageType(ImageType type)
{
    DEB_MEMBER_FUNCT();
    // --- see above for future immprovement
    return;
}
//-----------------------------------------------------
// @brief return the detector type
//-----------------------------------------------------
void Camera::getDetectorType(string& type)
{
    DEB_MEMBER_FUNCT();
    
    type = m_detector_type;
}

//-----------------------------------------------------
// @brief return the detector model
//-----------------------------------------------------
void Camera::getDetectorModel(string& type)
{
    DEB_MEMBER_FUNCT();
    stringstream ss;
    ss << ", S/N. "<< m_detector_serial;
    type = m_detector_model + ss.str();
}

//-----------------------------------------------------
// @brief return the internal buffer manager
//-----------------------------------------------------
HwBufferCtrlObj* Camera::getBufferCtrlObj()
{
    DEB_MEMBER_FUNCT();
    return &m_buffer_ctrl_obj;
}


//-----------------------------------------------------
// @brief return true if passed trigger mode is supported
//-----------------------------------------------------
bool Camera::checkTrigMode(TrigMode trig_mode)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(trig_mode);
    bool valid_mode; 
    int ret;



    switch (trig_mode)
    {       
    case IntTrig:
    case IntTrigMult:
    case ExtTrigSingle:
    case ExtTrigMult:
    case ExtGate:
	ret = IsTriggerModeAvailable(m_trig_mode_maps[trig_mode]);
	switch (ret)
	{
	case DRV_SUCCESS:
	    valid_mode = true;
	    break;
	case DRV_INVALID_MODE:
	    valid_mode = false;
	    break;
	case DRV_NOT_INITIALIZED:                
	    valid_mode = false;
	    DEB_ERROR() << "System not initializsed, cannot get trigger mode status" << " : error code = " << error_code(ret);
	    THROW_HW_ERROR(Error) << "System not initializsed, cannot get trigger mode status";
	    break;                                                     
	}                
        break;

    default:
	valid_mode = false;
        break;
    }
    return valid_mode;
}
//-----------------------------------------------------
// @brief set the new trigger mode
//-----------------------------------------------------
void Camera::setTrigMode(TrigMode mode)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(mode);

    THROW_IF_NOT_SUCCESS(SetTriggerMode(m_trig_mode_maps[mode]), "Cannot set trigger mode");
    m_trig_mode = mode;    
}

//-----------------------------------------------------
// @brief return the current trigger mode
//-----------------------------------------------------
void Camera::getTrigMode(TrigMode& mode)
{
    DEB_MEMBER_FUNCT();
    mode = m_trig_mode;
    
    DEB_RETURN() << DEB_VAR1(mode);
}


//-----------------------------------------------------
// @brief set the new exposure time
//-----------------------------------------------------
void Camera::setExpTime(double exp_time)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(exp_time);
    
    THROW_IF_NOT_SUCCESS(SetExposureTime((float)exp_time), "Cannot set exposure time");
    m_exp_time = exp_time;
}

//-----------------------------------------------------
// @brief return the current exposure time
//-----------------------------------------------------
void Camera::getExpTime(double& exp_time)
{
    DEB_MEMBER_FUNCT();
    float exp, acc, kin;
    
    // --- because Fli can adjust the exposure time
    // --- need to hw read the acquisition timings here.
    // --- kin time is the kinetic (multi-frame) time between two frames
    THROW_IF_NOT_SUCCESS(GetAcquisitionTimings(&exp, &acc, &kin),  "Cannot get acquisition timings");
    m_exp_time = exp;
    m_kin_time = kin;
    
    exp_time = (double) exp;
    
    DEB_RETURN() << DEB_VAR1(exp_time);
}

//-----------------------------------------------------
// @brief set the new latency time between images
//-----------------------------------------------------
void Camera::setLatTime(double lat_time)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(lat_time);
    float exp, acc, kin;
        
    // --- Changing the latency time changes the kinetic cycle time
    // --- need to read back the timings which can differ from the set values.
    
    THROW_IF_NOT_SUCCESS(GetAcquisitionTimings(&exp, &acc, &kin), "Cannot get acquisition timings");
    m_exp_time = exp;
    m_kin_time = exp + lat_time;
    THROW_IF_NOT_SUCCESS(SetKineticCycleTime(m_kin_time), "Cannot set kinetic cycle time");
    m_latency_time = lat_time;
    
}

//-----------------------------------------------------
// @brief return the current latency time
//-----------------------------------------------------
void Camera::getLatTime(double& lat_time)
{
    DEB_MEMBER_FUNCT();
    // --- we do calculate the latency by using the kinetic cycle time (time between 2 frames)
    // --- minus the exposure time
    float exp, acc, kin;
    
    // --- because Fli can adjust the exposure time
    // --- need to hw read the acquisition timings here.
    // --- kin time is the kinetic (multi-frame) time between two frames
    // --- we do not know with andor how much is the readout time !!!!
    THROW_IF_NOT_SUCCESS(GetAcquisitionTimings(&exp, &acc, &kin), "Cannot get acquisition timings");
    m_exp_time = exp;
    m_kin_time = kin;
    
    lat_time = kin - exp;
    
    DEB_RETURN() << DEB_VAR1(lat_time);
}

//-----------------------------------------------------
// @brief returnt the exposure time range
//-----------------------------------------------------
void Camera::getExposureTimeRange(double& min_expo, double& max_expo) const
{
    DEB_MEMBER_FUNCT();
    
    min_expo = 0.;    
    max_expo = (double) m_exp_time_max;
 
    DEB_RETURN() << DEB_VAR2(min_expo, max_expo);
}

//-----------------------------------------------------
// @brief return the latency time range
//-----------------------------------------------------
void Camera::getLatTimeRange(double& min_lat, double& max_lat) const
{   
    DEB_MEMBER_FUNCT();

    // --- no info on min latency
    min_lat = 0.;       
    
    // --- do not know how to get the max_lat, fix it as the max exposure time
    max_lat = (double) m_exp_time_max;

    DEB_RETURN() << DEB_VAR2(min_lat, max_lat);
}

//-----------------------------------------------------
// @brief set the number of frames to be taken
//-----------------------------------------------------
void Camera::setNbFrames(int nb_frames)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(nb_frames);
    // --- Hoops continuous mode not yet supported    
    //    if (nb_frames == 0)
    //    {
    //        DEB_ERROR() << "Sorry continuous acquisition (setNbFrames(0)) not yet implemented";
    //        THROW_HW_ERROR(Error) << "Sorry continuous acquisition (setNbFrames(0)) not yet implemented";                
    //    }
    //    // --- We only work on kinetics mode which allow multi-frames to be taken
    //    // ---
    //    THROW_IF_NOT_SUCCESS(SetNumberKinetics(nb_frames), "Cannot set number of frames");
    m_nb_frames = nb_frames;
}

//-----------------------------------------------------
// @brief return the number of frames to be taken
//-----------------------------------------------------
void Camera::getNbFrames(int& nb_frames)
{
    DEB_MEMBER_FUNCT();
    nb_frames = m_nb_frames;
    DEB_RETURN() << DEB_VAR1(nb_frames);
}

//-----------------------------------------------------
// @brief return the current acquired frames
//-----------------------------------------------------
void Camera::getNbHwAcquiredFrames(int &nb_acq_frames)
{ 
    DEB_MEMBER_FUNCT();    
    nb_acq_frames = m_image_number;
}
  
//-----------------------------------------------------
// @brief return the camera status
//-----------------------------------------------------
void Camera::getStatus(Camera::Status& status)
{
    DEB_MEMBER_FUNCT();
    AutoMutex aLock(m_cond.mutex());
    status = m_status;
    //Check if the camera is not waiting for soft. trigger
    if (status == Camera::Readout && 
	m_trig_mode == IntTrigMult)
      {
	status = Camera::Ready;
      }
    DEB_RETURN() << DEB_VAR1(DEB_HEX(status));
}

//-----------------------------------------------------
// @brief set the new camera status
//-----------------------------------------------------
void Camera::_setStatus(Camera::Status status,bool force)
{
    DEB_MEMBER_FUNCT();
    AutoMutex aLock(m_cond.mutex());
    if(force || m_status != Camera::Fault)
        m_status = status;
    m_cond.broadcast();
}
//-----------------------------------------------------
// @brief do nothing, hw_roi = set_roi.
//-----------------------------------------------------
void Camera::checkRoi(const Roi& set_roi, Roi& hw_roi)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(set_roi);
    hw_roi = set_roi;

    DEB_RETURN() << DEB_VAR1(hw_roi);
}

//-----------------------------------------------------
// @brief set the new roi
//-----------------------------------------------------
void Camera::setRoi(const Roi& set_roi)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(set_roi);

    Point topleft, size;
    int binx, biny;
    int hstart, hend, vstart, vend;
    Roi hw_roi, roiMax;
    Size sizeMax;
    
    getDetectorImageSize(sizeMax);
    roiMax = Roi(0,0, sizeMax.getWidth(), sizeMax.getHeight());    

    // --- Warning, SetImage() needs coodinates in full image size not with binning
    // --- but Lima passes image size with binning applied on
    // --- so set a internal binning factor (binx/biny) for size correction.

    if(m_roi == set_roi) return;    
               
    if(set_roi.isActive() && set_roi != roiMax)
    {
	// --- a real roi available
	hw_roi = set_roi;
	binx = m_bin.getX(); biny = m_bin.getY();    	
    }
    else
    {
	// ---  either No roi or roi fit with max size!!!	
	// --- in that case binning for full size calculation is 1
	hw_roi = roiMax;
	binx=1; biny=1;
    }    
    // --- Fli sets the ROI by starting coordinates at 1 and not 0 !!!!
    // --- Warning, SetImage() needs coodinates in full image size not with binning
    // --- but Lima passes here image size with binning applied on

    topleft = hw_roi.getTopLeft(); size = hw_roi.getSize();
    hstart = topleft.x*binx +1;          vstart = topleft.y*biny +1;
    hend   = hstart + size.x*binx -1;    vend   = vstart + size.y*biny -1;
    
    DEB_TRACE() << "bin =  " << m_bin.getX() <<"x"<< m_bin.getY();
    DEB_TRACE() << "roi = " << hstart << "-" << hend << ", " << vstart << "-" << vend;
    //- then fix the new ROI
    THROW_IF_NOT_SUCCESS(SetImage(m_bin.getX(), m_bin.getY(), hstart, hend, vstart, vend), "Cannot set detector ROI");
    // cache the real ROI, used when setting BIN
    m_roi = hw_roi;
}

//-----------------------------------------------------
// @brief return the new roi 
//-----------------------------------------------------
void Camera::getRoi(Roi& hw_roi)
{
    DEB_MEMBER_FUNCT();
    // ---  no way to read the roi, Fli does not provide any function to do that!
    hw_roi = m_roi;
    
    DEB_RETURN() << DEB_VAR1(hw_roi);
}

//-----------------------------------------------------
// @brief range the binning to the maximum allowed
//-----------------------------------------------------
void Camera::checkBin(Bin &hw_bin)
{
    DEB_MEMBER_FUNCT();


    int x = hw_bin.getX();
    if(x > m_bin_max.getX())
        x = m_bin_max.getX();

    int y = hw_bin.getY();
    if(y > m_bin_max.getY())
        y = m_bin_max.getY();

    hw_bin = Bin(x,y);
    DEB_RETURN() << DEB_VAR1(hw_bin);
}
//-----------------------------------------------------
// @brief set the new binning mode
//-----------------------------------------------------
void Camera::setBin(const Bin &set_bin)
{
    DEB_MEMBER_FUNCT();
    Point topleft, size;
    
    int binx, biny;
    int hstart, hend, vstart, vend;
    Roi hw_roi, roiMax;
    Size sizeMax;

    getDetectorImageSize(sizeMax);
    roiMax = Roi(0,0, sizeMax.getWidth(), sizeMax.getHeight());

    if(m_bin == set_bin) return;

    // --- Warning, SetImage() needs coodinates in full image size not with binning
    // --- but Lima passes image size with binning applied on
    // --- so set a internal binning factor (binx/biny) for size correction.
               
    if(m_roi.isActive() && m_roi != roiMax) 
    {
	// --- a real available
	binx = set_bin.getX();  biny = set_bin.getY();
	hw_roi = m_roi;
    }
    else
    {
	// ---  either No roi or roi fit with max size!!!	
	// --- in that case binning for full size calculation is 1
	hw_roi = roiMax;
	binx = 1; biny = 1;
    }
    topleft = hw_roi.getTopLeft(); size = hw_roi.getSize();
    hstart = topleft.x*binx +1;          vstart = topleft.y*biny +1;
    hend   = hstart + size.x*binx -1;    vend   = vstart + size.y*biny -1;

    DEB_TRACE() << "bin =  " << set_bin.getX() <<"x"<< set_bin.getY();
    DEB_TRACE() << "roi = " << hstart << "-" << hend << ", " << vstart << "-" << vend;
    THROW_IF_NOT_SUCCESS(SetImage(set_bin.getX(), set_bin.getY(), hstart, hend, vstart, vend), "Cannot set detector BIN");
    m_bin = set_bin;
    
    DEB_RETURN() << DEB_VAR1(set_bin);
}

//-----------------------------------------------------
// @brief return the current binning mode
//-----------------------------------------------------
void Camera::getBin(Bin &hw_bin)
{
    DEB_MEMBER_FUNCT();
    // ---  no way to read the bin Fli does not provide any function to do that!
    hw_bin = m_bin;
    
    DEB_RETURN() << DEB_VAR1(hw_bin);
}

//-----------------------------------------------------
// @brief return always true, hw binning mode is supported
//-----------------------------------------------------
bool Camera::isBinningAvailable()
{
    DEB_MEMBER_FUNCT();
    bool isAvailable = true;

    // --- ok not realy need this function but could be completed
    // for further camera model which do not support binning
    return isAvailable;
}


//-----------------------------------------------------
// @brief return the detector pixel size in meter
//-----------------------------------------------------
void Camera::getPixelSize(double& sizex, double& sizey)
{
    DEB_MEMBER_FUNCT();
    float xsize, ysize;
    
    THROW_IF_NOT_SUCCESS(GetPixelSize(&xsize, &ysize), "Cannot get pixel size");
    sizex = xsize * 1e-6;
    sizey = ysize * 1e-6;
    DEB_RETURN() << DEB_VAR2(sizex, sizey); 
}


//-----------------------------------------------------
// @brief reset the camera, no hw reset available on Fli camera
//-----------------------------------------------------
void Camera::reset()
{
    DEB_MEMBER_FUNCT();
    return;
}

//-----------------------------------------------------
// @brief	set the shutter output level
// @param	level 0 or 1
//
//-----------------------------------------------------
void Camera::setShutterLevel(int level)
{
    DEB_MEMBER_FUNCT();

    DEB_TRACE() << "Camera::setShutterLevel - " << DEB_VAR1(level);		
    THROW_IF_NOT_SUCCESS(SetShutter(level, m_shutter_mode, m_shutter_close_time, m_shutter_open_time),  "Failed to set shutter level");
    m_shutter_level = level;
}

//-----------------------------------------------------
// @brief	get the shutter output level
// @param	level 0 or 1
//
//-----------------------------------------------------
void Camera::getShutterLevel(int& level)
{
    DEB_MEMBER_FUNCT();
    level = m_shutter_level;    
}


//-----------------------------------------------------
// @brief	set the shutter mode 
// @param	mode FRAME or MANUAL
//
//-----------------------------------------------------
void Camera::setShutterMode(ShutterMode mode)
{
    DEB_MEMBER_FUNCT();
    // --- SetShutter() param mode is both used to set  auto or manual mode and to open and close
    // --- 0 - Auto, 1 - Open, 2 - Close	
    int aMode = (mode == FRAME)? 0:2;
	DEB_TRACE() << "Camera::setShutterMode - " << DEB_VAR1(aMode)
                <<" - Close Time  = "<<m_shutter_close_time
                <<" - Open Time  = "<<m_shutter_open_time;		
    if (mode == FRAME)
    {    
        THROW_IF_NOT_SUCCESS(SetShutter(m_shutter_level, aMode, m_shutter_close_time, m_shutter_open_time), "Failed to set the shutter mode");
    }
    m_shutter_mode = mode;
}

//-----------------------------------------------------
// @brief	return the shutter mode
// @param	mode FRAME or MANUAL
//
//-----------------------------------------------------
void Camera::getShutterMode(ShutterMode& mode)
{
    DEB_MEMBER_FUNCT();
    mode = m_shutter_mode;
}


//-----------------------------------------------------
// @brief	set the shutter open or close
// @param	flag True-Open / False-Close
//
//-----------------------------------------------------
void Camera::setShutter(bool flag)
{
    DEB_MEMBER_FUNCT();
    // --- SetShutter() param mode is both used to set in auto or manual mode and to open and close
    // --- 0 - Auto, 1 - Open, 2 - Close
    int aMode = (flag)? 1:2; 
    DEB_TRACE() << "Camera::setShutter - " << DEB_VAR1(aMode)
                <<" - Close Time  = "<<m_shutter_close_time
                <<" - Open Time  = "<<m_shutter_open_time;			
    THROW_IF_NOT_SUCCESS(FLIControlShutter(m_device``````, "Failed close/open the shutter");
    m_shutter_state = flag;
}


//-----------------------------------------------------
// @brief	return the status of the shutter
// @param	flag True-Open / False-Close
//
//-----------------------------------------------------
void Camera::getShutter(bool& flag)
{
    DEB_MEMBER_FUNCT();
    flag = m_shutter_state;
}

//-----------------------------------------------------
// @brief	set the temperature
// @param	temperature in centigrade
//
//-----------------------------------------------------
void Camera::setTemperatureSP(int temp)
{
    DEB_MEMBER_FUNCT();
    THROW_IF_NOT_SUCCESS(SetTemperature(m_device, temp), "Failed to set temperature set-point");
    m_temperature_sp = temp;
}

//-----------------------------------------------------
// @brief	return the temperature set-point
// @param	temperature in centigrade
//
//-----------------------------------------------------
void Camera::getTemperatureSP(int& temp)
{
    DEB_MEMBER_FUNCT();
    temp = m_temperature_sp;
}

//-----------------------------------------------------
// @brief	Gets cooler power
// @param	float power
//
//-----------------------------------------------------
void Camera::getCoolerPower(float& power)   
{
    DEB_MEMBER_FUNCT();    
    THROW_IF_NOT_SUCCESS(GetCoolerPower(m_device, power));
}
