
namespace Fli
{
%TypeHeaderCode
#include <FliCamera.h>
%End

  class Camera
  {
%TypeHeaderCode
#include <FliCamera.h>
%End

  public:

    enum Status {
      Ready, Exposure, Readout, Latency, Fault
    };

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

    void getImageType(ImageType& type /Out/);
    void setImageType(ImageType type);

    void getDetectorType(std::string& type /Out/);
    void getDetectorModel(std::string& model /Out/);
    void getDetectorImageSize(Size& size /Out/);
    HwBufferCtrlObj* getBufferCtrlObj();

    void setTrigMode(TrigMode  mode);
    void getTrigMode(TrigMode& mode /Out/);
	
    void setExpTime(double  exp_time);
    void getExpTime(double& exp_time /Out/);

    void setLatTime(double  lat_time);
    void getLatTime(double& lat_time /Out/);

    void getExposureTimeRange(double& min_expo /Out/, double& max_expo /Out/);
    void getLatTimeRange(double& min_lat /Out/, double& max_lat /Out/);

    void setNbFrames(int  nb_frames);
    void getNbFrames(int& nb_frames /Out/);

    void checkRoi(const Roi& set_roi, Roi& hw_roi /Out/);
    void setRoi(const Roi& set_roi);
    void getRoi(Roi& hw_roi /Out/);

    void checkBin(Bin& /In,Out/);
    void setBin(const Bin&);
    void getBin(Bin& /Out/);

    void setShutter(bool flag);
    void getShutter(bool& flag /Out/);
    void setShutterMode(ShutterMode mode);
    void getShutterMode(ShutterMode& mode /Out/);

    void getPixelSize(double& xsize /Out/, double& ysize /Out/);

    void getStatus(Fli::Camera::Status& status /Out/);

    void setTemperatureSP(double temperature);
    void getTemperatureSP(double& temperature /Out/);
    void getTemperatureCCD (double& temperature /Out/);
    void getTemperatureBase (double& temperature /Out/);
    void getCoolerPower(double& power /Out/);
    void setExtTriggerLevel(int level);
    void getExtTriggerLevel(int& level /Out/);
    
  };

};
