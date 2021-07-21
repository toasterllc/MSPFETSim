#pragma once

class MSPInterface {
public:
    enum class Pin : uint8_t {
        Test,
        Rst,
    };
    
    enum class Op : uint8_t {
        Out0,
        Out1,
        In,
        Read,
    };
    
    virtual ~MSPInterface() {}
    
    // sbwPin(): Perform an operation on a pin.
    //   
    //   Op::Out0: write a 0 to the pin
    //   Op::Out1: write a 1 to the pin
    //   Op::In:   configure the pin as an input
    //   Op::Read: read the value of the pin (only used for Pin::Rst)
    virtual void sbwPin(Pin pin, Op op) {}
    
    // sbwFlush(): Marks a 'flush boundary': a point where queued operations can be
    //   flushed to the hardware.
    //   
    //   To guarantee timing requirements, queued operations must only be flushed to the hardware
    //   at points where this function is called. Note that queued operations aren't required to
    //   be flushed when this function is called, but when the queued operations are flushed,
    //   they must be flushed on a flush boundary.
    virtual void sbwFlush() {}
    
    // sbwRead(): Retrieves data previously queued via Op::Read operations.
    //   
    //   This function must flush outstanding operations to the hardware and return the data
    //   queued from Op::Read operations. Note that this function implies a flush
    //   boundary (see sbwFlush()).
    //   
    //   len==0 is valid and must flush outstanding IO operations without returning any data.
    virtual void sbwRead(void* buf, size_t len) {}
    
    
    
    
    // sbwFlush(): Marks a 'flush boundary': a point where queued operations can or must be
    //   flushed, depending on the `required` argument.
    // 
    // To guarantee timing requirements, queued operations must only be flushed to the hardware
    // at points where this function is called.
//    // sbwFlush(): Marks a 'flush boundary': a point where queued operations can be flushed.
//    // To guarantee timing requirements, queued operations must only be flushed to the hardware
//    // at points that are marked by this function.
//    // Note that queued operations aren't required to be flushed when this function is called,
//    // but when the queued operations are flushed, they must be flushed on a flush boundary.
//        // 
//    // Notes:
//    //   - queued operations are only required to be flushed if required==true
//    //   - if required==false and you choose not to flush the queued operations, the point in
//    //     a buffer
//    //     may need to be rememberedmust be rememebered so that a future 
//    //   - But when the queued operations are flushed, they must be flushed on a flush boundary.
//    virtual void sbwFlush() {}
};
