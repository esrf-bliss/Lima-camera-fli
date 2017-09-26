andor-objs = FliCamera.o FliInterface.o FliDetInfoCtrlObj.o FliSyncCtrlObj.o FliShutterCtrlObj.o FliBinCtrlObj.o FliRoiCtrlObj.o

SRCS = $(andor-objs:.o=.cpp) 

CXXFLAGS += -I../include -I../../../hardware/include -I../../../common/include \
			-I../sdk/libfli-1.104 \
			-Wall -pthread -fPIC -g

all:	Fli.o

Fli.o:	$(andor-objs)
	$(LD) -o $@ -r $+

clean:
	rm -f *.o *.P

%.o : %.cpp
	$(COMPILE.cpp) -MD $(CXXFLAGS) -o $@ $<
	@cp $*.d $*.P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	-e '/^$$/ d' -e 's/$$/ :/' < $*.d >> $*.P; \
	rm -f $*.d

-include $(SRCS:.cpp=.P)

.PHONY: check-syntax
check-syntax:
	$(CXX) -Wall -Wextra -fsyntax-only $(CXXFLAGS) $(CHK_SOURCES)
