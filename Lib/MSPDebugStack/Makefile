CXXFLAGS := -fPIC -std=c++0x -fvisibility=hidden -fvisibility-inlines-hidden

PCH_HEADER := ./DLL430_v3/src/TI/DLL430/pch.h
PCH_COMPILED := ./DLL430_v3/src/TI/DLL430/pch.h.gch

DEFINES := -DUNIX

ifdef DEBUG
CXXFLAGS += -g -O0
else
CXXFLAGS += -Os
DEFINES += -DNDEBUG
endif

MAKE_PCH += -x c++-header
USE_PCH += -include $(PCH_HEADER)

export BOOST_DIR
export STATIC
export DEBUG

INCLUDES := \
	-I./DLL430_v3/src/TI/DLL430 \
	-I./DLL430_v3/include \
	-I./DLL430_v3/src/TI/DLL430/EnergyTrace_TSPA \
	-I./Bios/include \
	-I./ThirdParty/include \
	-I./ThirdParty/BSL430_DLL \
	-I/usr/include/hidapi


LIBS :=
STATIC_LIBS :=

ifdef STATIC
STATIC_LIBS += -lboost_filesystem -lboost_system -lbsl430 -lboost_date_time -lboost_chrono -lboost_thread
else
LIBS += -lboost_filesystem -lboost_system -lbsl430 -lboost_date_time -lboost_chrono -lboost_thread
endif

LIBTHIRD := ./ThirdParty/lib64
LIBDIRS := -L$(LIBTHIRD)

PLATFORM := $(shell uname -s)
ifeq ($(PLATFORM),Linux)
	CXX:= g++
	
	STATICOUTPUT := linux64

	OUTPUT := libmsp430.so
	DEFINES += -DUNIX

	ifdef STATIC
	STATIC_LIBS += -lusb
	else
	LIBS += -lusb
	endif

	LIBS += -lusb -lrt -lpthread

	ifdef BOOST_DIR
	INCLUDES += -I$(BOOST_DIR)
	LIBDIRS += -L$(BOOST_DIR)/stage/lib
	endif

	OUTNAME := -Wl,-soname,
	BSTATIC := -Wl,-Bstatic
	BDYNAMIC := -Wl,-Bdynamic

	HIDOBJ := -lhidapi-libusb
else
	CXX:= clang++

	OUTPUT := libmsp430.dylib
	STATICOUTPUT := mac64

	ifdef STATIC
	STATIC_LIBS += -framework CoreFoundation -framework IOKit -lhidapi
	else
	LIBS += -framework CoreFoundation -framework IOKit -lhidapi
	endif

	ifdef BOOST_DIR
	INCLUDES += -I$(BOOST_DIR)/include
	LIBDIRS += -L$(BOOST_DIR)/lib
	endif

	OUTNAME := -install_name
	BSTATIC :=
	BDYNAMIC :=

	HIDOBJ :=
endif


BSLLIB := $(LIBTHIRD)/libbsl430.a


SRC := \
        ./DLL430_v3/src/TI/DLL430/EEM/CycleCounter.cpp \
        $(wildcard ./DLL430_v3/src/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/BreakpointManager/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/CycleCounter/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/EemRegisters/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/EmulationManager/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/Exceptions/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/Sequencer/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/SoftwareBreakpoints/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/StateStorage430/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/Trace/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/Trigger/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/TriggerCondition/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/TriggerManager/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EM/VariableWatch/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/EnergyTrace_TSPA/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/logging/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/TemplateDeviceDb/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/DeviceDb/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/MSP432/*.cpp) \
        $(wildcard ./DLL430_v3/src/TI/DLL430/warnings/*.cpp) \
        $(wildcard ./ThirdParty/src/*.cpp)

OBJS := $(patsubst %.cpp, %.o, $(SRC))

all: $(BSLLIB) $(OBJS)
	$(CXX) $(CXXFLAGS) -shared $(OUTNAME)$(OUTPUT) -o $(OUTPUT) $(OBJS) $(HIDOBJ) $(LIBDIRS) $(BSTATIC) $(STATIC_LIBS) $(BDYNAMIC) $(LIBS)
	rm -f $(STATICOUTPUT).a
	ar -rs $(STATICOUTPUT).a $(OBJS)

$(PCH_COMPILED): $(PCH_HEADER)
	$(CXX) $(MAKE_PCH) -o $@ $< $(CXXFLAGS) $(INCLUDES) $(DEFINES)

%.o: %.cpp $(PCH_COMPILED)
	$(CXX) -c -o $@ $< $(USE_PCH) $(CXXFLAGS) $(INCLUDES) $(DEFINES)

$(BSLLIB):
	$(MAKE) -C ./ThirdParty/BSL430_DLL

install:
	cp $(OUTPUT) /usr/local/lib/

clean:
	$(MAKE) -C ./ThirdParty/BSL430_DLL clean
	@for i in $(OBJS); do rm -f $$i; done
	@rm -f $(PCH_HEADER).?ch build.log
