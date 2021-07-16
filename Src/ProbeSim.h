#pragma once
#include "MSP430.h"

namespace TI::DLL430
{

// ProbeSim simulates a debug probe in order to translate TI library
// commands (received from a ProbeSimIoChannel) into GPIO toggling
// sequences that can be implemented by generic hardware.
// ProbeSim currently supports Spy-bi-wire (SBW) connections to the
// target MSP430. JTAG support is possible but unimplemented.
class ProbeSim
{
public:
    struct Msg {
        uint8_t data[260]; // 1b length + 255b payload + 2b CRC
        size_t len = 0;
    };
    using MsgPtr = std::unique_ptr<Msg>;
    
    ProbeSim(const std::string& name, const MSP430_ProbeSimDescriptor& desc);
    ~ProbeSim();
    
    ProbeSim(const ProbeSim&) = delete;
    ProbeSim& operator=(const ProbeSim&) = delete;
    
    const std::string& name() const { return _name; }
    
    void open();
    void close();
    
    MsgPtr read();
    void write(const uint8_t* data, size_t len);
    
private:
    const std::string _name;
    const MSP430_ProbeSimDescriptor _desc;
    
    struct {
        std::mutex lock; // Protects this struct
        
        struct {
            std::unique_lock<std::mutex> waitLock;
            std::condition_variable signal;
            std::deque<MsgPtr> queue;
        } msgs;
        
        struct {
            std::condition_variable signal;
            std::deque<MsgPtr> queue;
        } resps;
    } _state;
    
    void _waitForMsg();
    void _firmwareThread();
    void _handleMsg(const uint8_t* data, size_t len);
    
    // Make sure the C basic types aren't used.
    // They must be replaced by explicit-width types (eg uint8_t) to ensure
    // execution that's consistent with the embedded device.
    #define char    BAD_TYPE
    #define short   BAD_TYPE
    #define int     BAD_TYPE
    #define long    BAD_TYPE
    
    #include "ProbeSim_USB.h"
    
    #undef char
    #undef short
    #undef int
    #undef long
};

} // TI::DLL430
