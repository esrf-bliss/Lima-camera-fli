namespace Fli
{
  class Interface : HwInterface
  {
%TypeHeaderCode
#include <FliInterface.h>
%End
  public:


    Interface(Fli::Camera& cam /KeepReference/);
    virtual ~Interface();

    //- From HwInterface
    //    virtual void 	getCapList(CapList& /Out/) const;
    virtual void	getCapList(std::vector<HwCap> &cap_list /Out/) const;
    virtual void	reset(ResetLevel reset_level);
    virtual void 	prepareAcq();
    virtual void 	startAcq();
    virtual void 	stopAcq();
    virtual void 	getStatus(StatusType& status /Out/);
    virtual int 	getNbHwAcquiredFrames();

    void setAdcSpeed(int index);
    void getAdcSpeed(int& index /Out/);
    void getAdcSpeedMaxIndex(int &max_index /Out/);
    void getAdcSpeedPaireString(int index, std::string& paire /Out/);
    void setVsSpeed(int index);
    void getVsSpeed(int& index /Out/);
    void getVsSpeedMaxIndex(int& max_index /Out/);
    void getVsSpeedString(int index, std::string& speed /Out/);
    void setPGain(int index);
    void getPGain(int&index);
    void getPGainMaxIndex(int& max_index /Out/);
    void getPGainString(int index, std::string& pgain /Out/);
    void setFastExtTrigger(bool flag);
    void getFastExtTrigger(bool& flag /Out/);
    void setShutterLevel(int level);
    void getShutterLevel(int& level /Out/);
    void setTemperatureSP(int temp);
    void getTemperatureSP(int& temp);
    void getTemperature(int& temp /Out/);
    void setCooler(bool flag);
    void getCooler(bool& flag /Out/);
    void getCoolingStatus(std::string& status /Out/);
    void setSpooling(bool flag, Fli::SpoolingMethod method, std::string path, int frameBufferSize);
    void setHighCapacity(Fli::HighCapacityMode mode);
    void getHighCapacity(Fli::HighCapacityMode& mode /Out/);
    void setGateMode(Fli::GateMode mode);
    void setFanMode(Fli::FanMode mode);
    void getFanMode(Fli::FanMode& mode /Out/);
    void setBaselineClamp(Fli::BaselineClamp mode);
    void getBaselineClamp(Fli::BaselineClamp& mode /Out/);
  };
};
