NAME=MSPFETSim
OBJECTS=Src/main.o

CXX      = g++
CXXFLAGS = -std=c++17 -O0 -g3 -Wall -Weffc++ $(IDIRS)
LFLAGS   = -ludev -lpthread -lusb-1.0 -lftdi1
IDIRS    = -iquote Src							\
           -iquote Lib							\
           -iquote Lib/VirtualUSBDevice/Src		\
           -iquote Lib/MSPDebugStack

ifneq ($(wildcard ../MDC/*),)
IDIRS	+= -iquote ../MDC/Tools/Shared			\
           -iquote ../MDC/Code/Shared			\
           -iquote ../MDC/Code/STM32/Shared
endif

all: ${OBJECTS}
	$(CXX) $(CXXFLAGS) $? -o $(NAME) $(LFLAGS)

clean:
	rm -Rf Src/*.o $(NAME)
