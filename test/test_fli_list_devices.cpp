#include <stdlib.h>
#include <iostream>
#include <string>
#include <list>


#include <libfli.h>

#define DOFLIAPI(F) { if ((status = (F)) != 0) { std::cout  << #F << "failed in "<< __FILE__ <<" line "<< __LINE__ << ". status = " << status; break; } }

#define LIBVERSIZE 1024
#define MAX_DEVICES 32
#define BUFF_SIZE 1024
#define MAX_PATH 260

long status = 0;

int numCams = 0;

typedef std::pair<std::string, int> Device;
std::list< Device > listDevice;

void EnumerateCameras(flidomain_t enumDomain)
{
  numCams = 0;
  
  char file[MAX_PATH], name[MAX_PATH];
  long domain;
  
  FLICreateList(enumDomain);
  
  if(FLIListFirst(&domain, file, MAX_PATH, name, MAX_PATH) == 0)
    {
      do
	{
	  listDevice.push_back(std::make_pair(file, domain));
	  numCams++;
	}
      while((FLIListNext(&domain, file, MAX_PATH, name, MAX_PATH) == 0) && (numCams < MAX_DEVICES));
    }
  
  FLIDeleteList();
}



int main()
{

  char libver[LIBVERSIZE];

  if(FLIGetLibVersion(libver, LIBVERSIZE) != 0)
    {
      std::cout << "Unable to retrieve library version!\n";
      exit(0);
    }
  
  std::cout << "Library version " << libver;
  
  EnumerateCameras(FLIDOMAIN_USB | FLIDEVICE_CAMERA);

  
  if(listDevice.empty())
    {
      std::cout << "\nNo FLI cameras have been detected\n";
    }
  
  flidev_t dev = FLI_INVALID_DEVICE;
  
  for(std::list< Device >::iterator it = listDevice.begin();
      it!=listDevice.end(); ++it)
    {
      long tmp1, tmp2, tmp3, tmp4, img_rows, row_width;
      double d1, d2;
      char buff[BUFF_SIZE];
      unsigned short *img;
      int row;
      
      std::cout << "\nConnecting to camera '"<< (*it).first <<"' in domain " << (*it).second << std::endl;
      
      DOFLIAPI(FLIOpen(&dev, (char *)(*it).first.c_str(), (*it).second));
      if(status != 0)
	{
	  continue;
	}
      
      DOFLIAPI(FLIGetModel(dev, buff, BUFF_SIZE));
      std::cout << "Model:        " << buff << std::endl;
      
      DOFLIAPI(FLIGetSerialString(dev, buff, BUFF_SIZE));
      std::cout << "Serial Num:   " << buff << std::endl;
      
      DOFLIAPI(FLIGetHWRevision(dev, &tmp1));
      std::cout << "Hardware Rev: " << tmp1 << std::endl;
      
      DOFLIAPI(FLIGetFWRevision(dev, &tmp1));
      std::cout << "Firmware Rev: " << tmp1 << std::endl;
      
      DOFLIAPI(FLIGetPixelSize(dev, &d1, &d2));
      std::cout << "Pixel Size:   "<< d1 << " x " << d2 << std::endl;
      
      DOFLIAPI(FLIGetArrayArea(dev, &tmp1, &tmp2, &tmp3, &tmp4));
      std::cout << "Array Area:   (" << tmp1 <<", " << tmp2 << "), ("<< tmp3 << ", "<< tmp4 <<")" << std::endl;
      
      DOFLIAPI(FLIGetVisibleArea(dev, &tmp1, &tmp2, &tmp3, &tmp4));
      std::cout << "Visible Area: (" << tmp1 <<", " << tmp2 << "), ("<< tmp3 << ", "<< tmp4 <<")" << std::endl;
      
      
      DOFLIAPI(FLISetTemperature(dev, -10.0));
      
      d1 = 0.0;
      
      DOFLIAPI(FLIReadTemperature(dev, FLI_TEMPERATURE_CCD, &d1));
      std::cout << "CCD Temp:     " << d1 << std::endl;
      
      DOFLIAPI(FLIReadTemperature(dev, FLI_TEMPERATURE_BASE, &d1));
      std::cout << "Base Temp:    " << d1 << std::endl;
      
      d1 = 0;
      FLIGetCoolerPower(dev, &d1);
      std::cout << "Cooler Power: " << d1 << std::endl;
      
      if(dev != FLI_INVALID_DEVICE)
	{
	  FLIClose(dev);
	}
    }
}
