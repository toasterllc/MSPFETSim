NAME=MSPProbeSim
OBJECTS=Src/main.o

CXX      = g++
CXXFLAGS = -O0 -g3 -Wall -std=c++17 -include Src/TI/Prefix.h $(IDIRS)
LFLAGS   = -ludev -lpthread
IDIRS    = -iquote Lib/Toastbox					\
           -iquote Lib/VirtualUSBDevice/src		\
           -iquote Lib/MSPDebugStack

all: ${OBJECTS}
	$(CXX) $(CXXFLAGS) $? -o $(NAME) $(LFLAGS)

clean:
	rm -Rf Src/*.o $(NAME)
