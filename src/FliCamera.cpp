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
      THROW_HW_ERROR(Error) << error_prefix << ": " << strerror((int)-ret_code); \
}


//---------------------------------------------------------------------------------------
//! Camera::CameraThread::CameraThread()
//---------------------------------------------------------------------------------------
Camera::CameraThread::CameraThread(Camera& cam)
  : m_cam(&cam)
{
    DEB_CONSTRUCTOR();
    m_acq_frame_nb = 0;
}

//---------------------------------------------------------------------------------------
//! Camera::CameraThread::~CameraThread()
//---------------------------------------------------------------------------------------
Camera::CameraThread::~CameraThread()
{
    DEB_DESTRUCTOR();
    abort();
}

//---------------------------------------------------------------------------------------
//! Camera::CameraThread::start()
//---------------------------------------------------------------------------------------
void Camera::CameraThread::start()
{
    DEB_MEMBER_FUNCT();
    CmdThread::start();
    waitStatus(Ready);
}

//---------------------------------------------------------------------------------------
//! Camera::CameraThread::init()
//---------------------------------------------------------------------------------------
void Camera::CameraThread::init()
{
    DEB_MEMBER_FUNCT();
    setStatus(Ready);
}

//---------------------------------------------------------------------------------------
//! Camera::CameraThread::execCmd()
//---------------------------------------------------------------------------------------
void Camera::CameraThread::execCmd(int cmd)
{
    DEB_MEMBER_FUNCT();
    switch (cmd)
    {
    case StartAcq:
	execStartAcq();
	break;
    case StopAcq:
        setStatus(Ready);
        break;
    }
}


#define READ_STATUS() \
{ \
    THROW_IF_NOT_SUCCESS(FLIGetDeviceStatus(m_cam->m_device, &camera_status), "Cannot get camera status "); \
    THROW_IF_NOT_SUCCESS(FLIGetExposureStatus(m_cam->m_device, &remaining_exposure), "Cannot get remaining exp time ");	\
}

#define IMAGE_READY ( ((camera_status == FLI_CAMERA_STATUS_UNKNOWN) && (remaining_exposure == 0)) || \
		     ((camera_status != FLI_CAMERA_STATUS_UNKNOWN) && ((camera_status & FLI_CAMERA_DATA_READY) != 0)) )

//---------------------------------------------------------------------------------------
//! Camera::CameraThread::execStartAcq()
//---------------------------------------------------------------------------------------
void Camera::CameraThread::execStartAcq()
{
	
    DEB_MEMBER_FUNCT();
    setStatus(Exposure);
    
    StdBufferCbMgr& buffer_mgr = m_cam->m_buffer_ctrl_obj.getBuffer();
    buffer_mgr.setStartTimestamp(Timestamp::now());
        
    long remaining_exposure = 0;
    long camera_status = 0;

    FrameDim frame_dim = buffer_mgr.getFrameDim();
    int  frame_size = frame_dim.getMemSize();
    // --- set number of flushes before exposing
    THROW_IF_NOT_SUCCESS(FLISetNFlushes(m_cam->m_device, 1), "Cannot flush background");    
    // --- start acquisition, try with video mode
    THROW_IF_NOT_SUCCESS(FLIStartVideoMode(m_cam->m_device), "Cannot start video acquisition");

    // simulate here the IntTrigMult
    int nb_frames = m_cam->m_trig_mode != IntTrigMult ? m_cam->m_nb_frames : m_acq_frame_nb + 1;
    int& frame_nb = m_acq_frame_nb;
    
    for(;nb_frames == 0 || frame_nb < nb_frames;frame_nb++)
    {

	// -- read a first camera status
	READ_STATUS();
	// -- wait for an image
	while (getNextCmd() != StopAcq && !IMAGE_READY)
	{
	    READ_STATUS();
	    //DEB_TRACE() << DEB_VAR2(camera_status, remaining_exposure);
	    usleep(100);
	}

	if(getNextCmd() == StopAcq)
	{
	    waitNextCmd();
	    break;
	}
		
	setStatus(Readout);	
	buffer_mgr.setStartTimestamp(Timestamp::now());
	void *ptr = buffer_mgr.getFrameBufferPtr(frame_nb);

	THROW_IF_NOT_SUCCESS(FLIGrabVideoFrame(m_cam->m_device, ptr, (size_t)frame_size), "Cannot grab video frame ");
	DEB_TRACE() << "Declare a new Frame Ready.";
	HwFrameInfoType frame_info;
	frame_info.acq_frame_nb = frame_nb;
	buffer_mgr.newFrameReady(frame_info);

	// --- simulate here a latency time
	if (m_cam->m_latency_time > 0) {
	    setStatus(Latency);
	    usleep(long(m_cam->m_latency_time * 1e6));
	}	
    }
  
    // stop acquisition
    THROW_IF_NOT_SUCCESS(FLICancelExposure(m_cam->m_device), "Canot cancel exposure ");
       
    setStatus(Ready);
}

//---------------------------
// @brief  Ctor
//---------------------------
Camera::Camera(const std::string& camera_path)
    : m_thread(*this),
      m_image_number(0),
      m_latency_time(0.),
      m_bin(1,1),
      m_shutter_state(false),
      m_exp_time(1.),
      m_ext_trigger_level(FLI_SHUTTER_EXTERNAL_TRIGGER_HIGH),
      m_detector_type("Fli")
{
    DEB_CONSTRUCTOR();
    
    THROW_IF_NOT_SUCCESS(FLIOpen(&m_device, (char *)camera_path.c_str(), FLIDOMAIN_USB|FLIDEVICE_CAMERA),
			 "Cannot open FLI camera as " << DEB_VAR1(camera_path));

    long tmp1, tmp2, tmp3, tmp4, img_rows, row_width;
    double d1, d2;
    const int buff_size = 1024;
    char buff[buff_size];
    // --- Get Camera model, and other ident info
    THROW_IF_NOT_SUCCESS(FLIGetModel(m_device, buff, buff_size), "Cannot get camera model ");    
    m_detector_model = buff;
    THROW_IF_NOT_SUCCESS(FLIGetSerialString(m_device, buff, buff_size), "Cannot get camera serial # ");        
    m_detector_serial = buff;
    THROW_IF_NOT_SUCCESS(FLIGetHWRevision(m_device, &m_detector_hw_rev), "Cannot get camera HW revision ");
    THROW_IF_NOT_SUCCESS(FLIGetFWRevision(m_device, &m_detector_fw_rev), "Cannot get camera FW revision ");
    THROW_IF_NOT_SUCCESS(FLIGetPixelSize(m_device, &(m_pixel_size.first), &(m_pixel_size.second)), "Cannot get camera pixel size ");
    THROW_IF_NOT_SUCCESS(FLIGetArrayArea(m_device, (long*)&(m_array_area.first.x), (long*)&(m_array_area.first.y),
					 (long*)&(m_array_area.second.x), (long*)&(m_array_area.second.y)), "Cannot get camera array area ");
    THROW_IF_NOT_SUCCESS(FLIGetVisibleArea(m_device,  (long*)&(m_visible_area.first.x),  (long*)&(m_visible_area.first.y),
					    (long*)&(m_visible_area.second.x),  (long*)&(m_visible_area.second.y)), "Cannot get camera visible area ");
    
    DEB_ALWAYS() << "Fli Camera device found on path " << camera_path << ":\n" 
		 << "    * Model    : " << m_detector_model << "\n"
		 << "    * Serial # : " << m_detector_serial << "\n"
		 << "    * HW & FW rev.: " <<  m_detector_hw_rev << ", " << m_detector_fw_rev << "\n"
		 << "    * Array area : (" << m_array_area.first << ", " << m_array_area.second << ")\n"
		 << "    * Array visible : (" << m_visible_area.first << ", " << m_visible_area.second << ")\n";
    
    m_bin_max = Bin(16, 16);   

    // --- set bin, roi and depth for default, there is no way to read back them with sdk function
    // --- so use caches
    THROW_IF_NOT_SUCCESS(FLISetHBin(m_device, 1),"Cannot set HBin to 1 ");
    THROW_IF_NOT_SUCCESS(FLISetVBin(m_device, 1),"Cannot set VBin to 1 ");
    m_bin = Bin(1,1);
    THROW_IF_NOT_SUCCESS(FLISetImageArea(m_device, 0, 0, (long)m_array_area.second.x, (long)m_array_area.second.y),"Cannot set image area ");
    m_roi = Roi(Point(0,0), m_array_area.second);
    // command not supported supposed here all the cameras are 16bit
    // THROW_IF_NOT_SUCCESS(FLISetBitDepth(m_device, FLI_MODE_16BIT),"Cannot set bit depth to 8bit ");
    m_bit_depth = FLI_MODE_16BIT;
    
    // --- set shutter mode to manual close
    THROW_IF_NOT_SUCCESS(FLIControlShutter(m_device, FLI_SHUTTER_CLOSE),"Cannot close shutter ");

    // --- set image mode to exposure vs. background
    THROW_IF_NOT_SUCCESS(FLISetFrameType(m_device, FLI_FRAME_TYPE_NORMAL),"Cannot close shutter ");
    
    // --- finally start the acq thread
    m_thread.start();
}

//---------------------------
// @brief  Dtor
//---------------------------
Camera::~Camera()
{
    DEB_DESTRUCTOR();
    // Stop Acq thread
    stopAcq();
    // Close camera
    if(m_device != FLI_INVALID_DEVICE)
	FLIClose(m_device);
}

//---------------------------
// @brief  prepare the acquistion
//---------------------------
void Camera::prepareAcq()
{
    DEB_MEMBER_FUNCT();
    
    m_thread.m_acq_frame_nb = 0;
    
    if (m_trig_mode == ExtTrigMult) {
	// seems we must set mask to 0x0e for setting bit0 to 0 to close shutter as well
	THROW_IF_NOT_SUCCESS(FLIControlShutter(m_device, m_ext_trigger_level & 0x0e), "Failed set external trigger mode");
    } else {
	THROW_IF_NOT_SUCCESS(FLIControlShutter(m_device, m_shutter_state), "Failed set external trigger mode");
    }
}
//---------------------------
// @brief  start the acquistion
//---------------------------
void Camera::startAcq()
{
    DEB_MEMBER_FUNCT();
    int status = m_thread.getStatus();
    if (status != CameraThread::Ready)
        THROW_HW_ERROR(Error) << "Camera not Ready";    
    m_thread.sendCmd(CameraThread::StartAcq);
    m_thread.waitNotStatus(CameraThread::Ready);
}

//---------------------------
// @brief stop the acquisition
//---------------------------
void Camera::stopAcq()
{
    DEB_MEMBER_FUNCT();
    
    if(m_thread.getStatus() != CameraThread::Ready) {
        m_thread.sendCmd(CameraThread::StopAcq);
        m_thread.waitStatus(CameraThread::Ready);
    }

}


//-----------------------------------------------------
// brief return the detector image size 
//-----------------------------------------------------
void Camera::getDetectorImageSize(Size& size)
{
    DEB_MEMBER_FUNCT();
    int xmax, ymax;
    
    // --- Suppose to return the max image size here, already taken by contructor
    size= Size(m_array_area.second.x, m_array_area.second.y);
}


//-----------------------------------------------------
// @brief return the image type 
//-----------------------------------------------------
void Camera::getImageType(ImageType& type)
{
    DEB_MEMBER_FUNCT();
    int bits;
    
    if (m_bit_depth == FLI_MODE_8BIT) type = Bpp8;
    else if (m_bit_depth == FLI_MODE_16BIT) type = Bpp16;
}

//-----------------------------------------------------
// @brief set the image type, if supported
//-----------------------------------------------------
void Camera::setImageType(ImageType type)
{
    DEB_MEMBER_FUNCT();
    switch (type)
    {
    case Bpp8: m_bit_depth = FLI_MODE_8BIT; break;
    case Bpp16: m_bit_depth = FLI_MODE_16BIT; break;
    default: THROW_HW_ERROR(InvalidValue) << DEB_VAR1(type);
    }
    
    THROW_IF_NOT_SUCCESS(FLISetBitDepth(m_device, m_bit_depth), "cannot set bit depth");   
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
    case ExtTrigMult:
	valid_mode = true;
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
    // --- in millisecond
    long exptime = (long) (exp_time*1000);
    THROW_IF_NOT_SUCCESS(FLISetExposureTime(m_device, exptime), "Cannot set exposure time");
    m_exp_time = (double) (exptime/1000);
}

//-----------------------------------------------------
// @brief return the current exposure time
//-----------------------------------------------------
void Camera::getExpTime(double& exp_time)
{
    DEB_MEMBER_FUNCT();

    
    exp_time = (double) m_exp_time;
    
    DEB_RETURN() << DEB_VAR1(exp_time);
}

//-----------------------------------------------------
// @brief set the new latency time between images
//-----------------------------------------------------
void Camera::setLatTime(double lat_time)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(lat_time);
  
    m_latency_time = lat_time;
    
}

//-----------------------------------------------------
// @brief return the current latency time
//-----------------------------------------------------
void Camera::getLatTime(double& lat_time)
{
    DEB_MEMBER_FUNCT();
    lat_time = m_latency_time;
    DEB_RETURN() << DEB_VAR1(lat_time);
}

//-----------------------------------------------------
// @brief returnt the exposure time range
//-----------------------------------------------------
void Camera::getExposureTimeRange(double& min_expo, double& max_expo) const
{
    DEB_MEMBER_FUNCT();
    
    min_expo = 0.;    
    max_expo = (double) 1e6;
 
    DEB_RETURN() << DEB_VAR2(min_expo, max_expo);
}

//-----------------------------------------------------
// @brief return the latency time range
//-----------------------------------------------------
void Camera::getLatTimeRange(double& min_lat, double& max_lat) const
{   
    DEB_MEMBER_FUNCT();

    // --- no latency neither frame rate, so just by simulation (see CameraThread::execStartAcq() )

    min_lat = 0.;       
    
    // --- 3 max. why not? 
    max_lat = (double) 3;

    DEB_RETURN() << DEB_VAR2(min_lat, max_lat);
}

//-----------------------------------------------------
// @brief set the number of frames to be taken
//-----------------------------------------------------
void Camera::setNbFrames(int nb_frames)
{
    DEB_MEMBER_FUNCT();
    DEB_PARAM() << DEB_VAR1(nb_frames);
 
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
    nb_acq_frames = m_thread.m_acq_frame_nb;
}
  
//-----------------------------------------------------
// @brief return the camera status
//-----------------------------------------------------
void Camera::getStatus(Camera::Status& status)
{
    DEB_MEMBER_FUNCT();
    int thread_status = m_thread.getStatus();
    
    DEB_RETURN() << DEB_VAR1(thread_status);
	
    switch (thread_status)
    {
    case CameraThread::Ready:
	status = Camera::Ready; break;
    case CameraThread::Exposure:
	status =  Camera::Exposure; break;
    case CameraThread::Readout:
	status = Camera::Readout; break;
    case CameraThread::Latency:
	status =  Camera::Latency; break;
    default:
	throw LIMA_HW_EXC(Error, "Invalid thread status");
    }
    
    DEB_RETURN() << DEB_VAR1(DEB_HEX(status));
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
    long ul_x, ul_y, lr_x, lr_y;
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
    // --- Warning, SetImageArea() needs absolute topleft point binned bottomright point
    // --- but Lima passes here image size with binning applied on

    topleft = hw_roi.getTopLeft(); size = hw_roi.getSize();
    ul_x = topleft.x*binx;          ul_y = topleft.y*biny;
    lr_x   = topleft.x + size.x;    lr_y   = topleft.y + size.y;
    
    DEB_TRACE() << "bin =  " << m_bin.getX() <<"x"<< m_bin.getY();
    DEB_TRACE() << "FLI roi = " << ul_x << "-" << ul_y << ", " << lr_x << "-" << lr_y;
    //- then fix the new ROI
    THROW_IF_NOT_SUCCESS(FLISetImageArea(m_device,ul_x, ul_y, lr_x, lr_y), "Cannot set detector ROI");
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

    THROW_IF_NOT_SUCCESS(FLISetHBin(m_device, set_bin.getX()),"Cannot set HBin ");
    THROW_IF_NOT_SUCCESS(FLISetVBin(m_device, set_bin.getY()),"Cannot set VBin ");
   
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
    
    sizex =  m_pixel_size.first;
    sizey = m_pixel_size.second;
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
// @brief	set the external trigger input level
// @param	level 0 or 1
//
//-----------------------------------------------------
void Camera::setExtTriggerLevel(int level)
{
    DEB_MEMBER_FUNCT();
    // --- same function is used to manually open/close internal shutter and to enable external trigger for exposure
    // --- manage both in prepareAcq() 
    DEB_TRACE() << "Camera::setExtTriggerLevel - " << DEB_VAR1(level);		
    m_ext_trigger_level = (level)? FLI_SHUTTER_EXTERNAL_TRIGGER_HIGH: FLI_SHUTTER_EXTERNAL_TRIGGER_LOW;
}

//-----------------------------------------------------
// @brief	get the external trigger input level
// @param	level 0 or 1
//
//-----------------------------------------------------
void Camera::getExtTriggerLevel(int& level)
{
    DEB_MEMBER_FUNCT();
    level = (m_ext_trigger_level==FLI_SHUTTER_EXTERNAL_TRIGGER_HIGH)?1:0;    
}


//-----------------------------------------------------
// @brief	set the shutter mode 
// @param	mode FRAME or MANUAL
//
//-----------------------------------------------------
void Camera::setShutterMode(ShutterMode mode)
{
    DEB_MEMBER_FUNCT();
    // --- same function is used to manually open/close internal shutter and to enable external trigger for exposure
    // --- manage both in prepareAcq() 
    DEB_TRACE() << "Camera::setShutterMode - " << DEB_VAR1(mode);
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
    // --- same function is used to manually open/close internal shutter and to enable external trigger for exposure
    // --- manage both in prepareAcq() 
    flishutter_t aMode = (flag)? FLI_SHUTTER_OPEN:FLI_SHUTTER_CLOSE; 
    DEB_TRACE() << "Camera::setShutter - " << DEB_VAR1(aMode);
    THROW_IF_NOT_SUCCESS(FLIControlShutter(m_device, aMode), "Failed close/open the shutter");
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
// @param	temperature in centigrade -55 to 45 C
//
//-----------------------------------------------------
void Camera::setTemperatureSP(double temp)
{
    DEB_MEMBER_FUNCT();
    if (temp < -55 || temp >45)
    {
	THROW_HW_ERROR(InvalidValue) << "Temperature SP range is -45/55 C";
    }
    THROW_IF_NOT_SUCCESS(FLISetTemperature(m_device, temp), "Failed to set temperature set-point");
    m_temperature_sp = temp;
}

//-----------------------------------------------------
// @brief	return the temperature set-point
// @param	temperature in centigrade
//
//-----------------------------------------------------
void Camera::getTemperatureSP(double& temp)
{
    DEB_MEMBER_FUNCT();
    temp = m_temperature_sp;
}

//-----------------------------------------------------
// @brief	return the temperature of The CCD (internal temperature)
// @param	temperature in centigrade
//
//-----------------------------------------------------
void Camera::getTemperatureCCD(double& temp)
{
    DEB_MEMBER_FUNCT();
    double aTemp;
    THROW_IF_NOT_SUCCESS(FLIReadTemperature(m_device,  FLI_TEMPERATURE_CCD, &aTemp), "Failed to get CCD temperature ");
    temp = aTemp;
}

//-----------------------------------------------------
// @brief	return the temperature of the base (external temperature)
// @param	temperature in centigrade
//
//-----------------------------------------------------
void Camera::getTemperatureBase(double& temp)
{
    DEB_MEMBER_FUNCT();
    double aTemp;
    THROW_IF_NOT_SUCCESS(FLIReadTemperature(m_device, FLI_TEMPERATURE_BASE, &aTemp), "Failed to get BASE temperature ");
    temp = aTemp;
}

//-----------------------------------------------------
// @brief	Gets cooler power
// @param	float power
//
//-----------------------------------------------------
void Camera::getCoolerPower(double& power)   
{
    DEB_MEMBER_FUNCT();
    double aPower;
    THROW_IF_NOT_SUCCESS(FLIGetCoolerPower(m_device, &aPower), "Cannot read cooler power ");
    power = aPower;
}
