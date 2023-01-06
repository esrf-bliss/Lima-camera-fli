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

    void setTemperatureSP(double temperature);
    void getTemperatureSP(double& temperature /Out/);
    void getTemperatureCCD (double& temperature /Out/);
    void getTemperatureBase (double& temperature /Out/);
    void getCoolerPower(double& power /Out/);
    void setExtTriggerLevel(int level);
    void getExtTriggerLevel(int& level /Out/); 

  private:
    Interface(const Interface&);
  };
};
