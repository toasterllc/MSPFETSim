#include <pch.h>
#include "ProbeSim.h"
#include <chrono>
using namespace TI::DLL430;
using namespace std::chrono_literals;

ProbeSim::ProbeSim(const std::string& name, const MSP430_ProbeSimDescriptor& desc) :
_name(name), _desc(desc) {
    std::thread thread([this] { _firmwareThread(); });
    thread.detach();
}









ProbeSim::~ProbeSim () {}

void ProbeSim::open() {
    printf("ProbeSim::open\n");
    bool br = _desc.callbacks.open(_desc.ctx);
    if (!br) throw std::runtime_error("open callback failed");
}

void ProbeSim::close() {
    printf("ProbeSim::close\n");
    bool br = _desc.callbacks.close(_desc.ctx);
    if (!br) throw std::runtime_error("close callback failed");
}

ProbeSim::MsgPtr ProbeSim::read() {
//    printf("ProbeSim::read(thread=%p) [START]\n", (void*)pthread_self());
    
    MsgPtr resp;
    auto lock = std::unique_lock(_state.lock);
        if (_state.resps.queue.empty()) {
            _state.resps.signal.wait_for(lock, 100ms);
        }
        
        if (!_state.resps.queue.empty()) {
            resp = std::move(_state.resps.queue.front());
            _state.resps.queue.pop_front();
        }
    lock.unlock();
    
    if (resp) {
        printf("ProbeSim::read(thread=%p, len=%zu) [FINISH]: ", (void*)pthread_self(), resp->len);
        for (size_t i=0; i<resp->len; i++) {
            printf("%02x ", resp->data[i]);
        }
        printf("\n");
    }
    return resp;
}

void ProbeSim::write(const uint8_t* data, size_t len) {
    printf("ProbeSim::write(thread=%p, len=%zu): ", (void*)pthread_self(), len);
    for (size_t i=0; i<len; i++) {
        printf("%02x ", data[i]);
    }
    printf("\n");
    
    _handleMsg(data, len);
}

//const uint8_t* ProbeSim::_outboxData() const {
//    return _outbox.buf.data()+_outbox.off;
//}
//
//size_t ProbeSim::_outboxLen() const {
//    assert(_outbox.buf.size() >= _outbox.off);
//    return _outbox.buf.size()-_outbox.off;
//}
//
//void ProbeSim::_outboxAdvance(size_t len) {
//    assert(_outboxLen() >= len);
//    _outbox.off += len;
//}
//
//ProbeSim::Msg ProbeSim::_outboxPopMsg() {
//    size_t len = 0;
//    bool br = _outboxRead(len);
//    if (!br) return Msg{};
//    
//    const uint8_t* data = _outboxData();
//    _outboxAdvance(len);
//    return Msg{data, len};
//}
//
//template <typename T>
//bool _readVal(const uint8_t*& buf, size_t& len, T& val) {
//    if (len < sizeof(val)) return false;
//    memcpy(&val, buf, sizeof(val));
//    buf += sizeof(val);
//    len -= sizeof(val);
//    return true;
//}
//
//size_t _msgLen(std::vector<uint8_t>& vbuf) {
//    const uint8_t* buf = vbuf.data();
//    size_t bufLen = vbuf.size();
//    
//    uint8_t msgLen = 0;
//    bool br = _readVal(buf, bufLen, msgLen);
//    if (!br) return 0;
//    if (bufLen < msgLen) return 0;
//    
//    const size_t consumedLen = sizeof(msgLen)+msgLen;
//    return consumedLen;
//}
//
//size_t ProbeSim::_respParse(std::vector<uint8_t>& vbuf, HalResponse& resp) {
//    const uint8_t* buf = vbuf.data();
//    size_t bufLen = vbuf.size();
//    
//    uint8_t msgLen = 0;
//    bool br = _readVal(buf, bufLen, msgLen);
//    if (!br) return 0;
//    
//    // Verify that the buffer holds the entire message
//    size_t remLen = msgLen;
//    if (bufLen < remLen) return 0;
//    
//    uint8_t type = 0;
//    br = _readVal(buf, remLen, type);
//    if (!br) return 0;
//    
//    uint8_t id = 0;
//    br = _readVal(buf, remLen, id);
//    if (!br) return 0;
//    
//    resp.setType(type);
//    resp.setId(id & 0x7f); //Don't mask async bit (0x40)
//    resp.setIsComplete(id);
//    resp.append(vbuf.data()+sizeof(msgLen), msgLen);
//    
//    // Shift `vbuf`
//    const size_t consumedLen = sizeof(msgLen)+msgLen;
//    vbuf.erase(vbuf.begin(), vbuf.begin()+consumedLen);
//    return consumedLen;
//}

void ProbeSim::_handleMsg(const uint8_t* data, size_t len) {
    assert(len <= std::numeric_limits<uint8_t>::max());
    
    MsgPtr msgPtr = std::make_unique<Msg>();
    Msg& msg = *msgPtr;
    memcpy(msg.data, data, len);
    msg.len = len;
    
    // Firmware requires that the total message length is even
    if (msg.len & 1) {
        msg.data[msg.len] = 0;
        msg.len++;
    }
    
    auto lock = std::unique_lock(_state.lock);
    _state.msgs.queue.push_back(std::move(msgPtr));
    const bool wakeUp = USBCDC_handleDataReceived(DEBUG_CHANNEL);
    if (wakeUp) {
        printf("_handleMsg: NOTIFY\n");
        _state.msgs.signal.notify_all();
    }
    lock.unlock();
    
//    V3OP_Scheduler();
//    
//    const uint8_t* str = msg.data;
//    
//    // Set the message that the STREAM_* APIs will use
//    _state.stream.msg = msg;
//    V3OP_Rx(str);
}

void ProbeSim::_waitForMsg() {
    _state.msgs.signal.wait(_state.msgs.waitLock);
}

void ProbeSim::_firmwareThread() {
    printf("[FW THREAD] Started\n");
    
    ResetFirmware();
    BIOS_InitCom();
    
    
    for (;;) {
        _state.msgs.waitLock = std::unique_lock(_state.lock);
//        printf("[FW THREAD] Calling V3OP_Scheduler\n");
        
        // Keep calling V3OP_Scheduler until there's no work to do
        bool serviced = true;
        for (int i=0; i<3 && serviced; i++) {
            serviced = V3OP_Scheduler();
        }
        
//        if (!_state.msgs.queue.empty()) {
//            printf("[FW THREAD] GOT MESSAGE\n");
//            V3OP_Scheduler();
//        }
        
//        printf("[FW THREAD] Waiting for message\n");
        
        _state.msgs.signal.wait_for(_state.msgs.waitLock, 100ms);
        
//        _waitForMsg();
//        printf("[FW THREAD] Got message notification\n");
        
        _state.msgs.waitLock = std::unique_lock<std::mutex>();
        
//        printf("[FW THREAD] AGAIN (empty: %d)\n", _state.msgs.queue.empty());
//        
//        sleep(1);
//        
//        usleep(10000);
    }
    
}

//void ProbeSim::_handleMessages() {
//    for (;;) {
//        const uint8_t* msg = _bufIn.data();
//        size_t msgLen = _msgLen(_bufIn);
//        if (!msgLen) break;
//        _handleMessage(msg);
//        
//        // Shift `_bufIn`
//        _bufIn.erase(_bufIn.begin(), _bufIn.begin()+msgLen);
//    }
//}
