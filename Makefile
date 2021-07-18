NAME=MSPProbeSim
OBJECTS=Src/main.o

CXX      = g++
CXXFLAGS = -O0 -g3 -Wall -std=c++17 $(IDIRS)
LFLAGS   = -ludev -lpthread -lusb-1.0
IDIRS    = -iquote Src                          \
           -iquote Lib/Toastbox                 \
           -iquote Lib/VirtualUSBDevice/src     \
           -iquote Lib/MSPDebugStack

all: ${OBJECTS}
	$(CXX) $(CXXFLAGS) $? -o $(NAME) $(LFLAGS)

clean:
	rm -Rf Src/*.o $(NAME)
