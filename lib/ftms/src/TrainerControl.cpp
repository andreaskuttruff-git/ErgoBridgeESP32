#include "TrainerControl.h"

namespace ftms {

namespace {

size_t encodeStatus(StatusCode code, uint8_t* out) {
    out[0] = static_cast<uint8_t>(code);
    return 1;
}

size_t encodeTargetPowerChanged(int watt, uint8_t* out) {
    out[0] = static_cast<uint8_t>(StatusCode::TargetPowerChanged);
    const uint16_t raw = static_cast<uint16_t>(static_cast<int16_t>(watt));
    out[1] = static_cast<uint8_t>(raw & 0xFF);
    out[2] = static_cast<uint8_t>(raw >> 8);
    return 3;
}

} // namespace

TrainerControl::TrainerControl(ApplyTarget applyTarget, CurrentTarget currentTarget)
    : applyTarget_(std::move(applyTarget)), currentTarget_(std::move(currentTarget)) {}

uint16_t TrainerControl::controller() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return controller_;
}

TrainerControl::Reply TrainerControl::respond(uint8_t opCode, ResultCode result) {
    Reply reply;
    reply.responseLength = encodeControlPointResponse(opCode, result, reply.response);
    return reply;
}

TrainerControl::Reply TrainerControl::handleControlPoint(uint16_t conn, const uint8_t* data, size_t length,
                                                         uint32_t /*nowMs*/) {
    if (length == 0)
        return Reply{};
    const uint8_t op = data[0];
    std::lock_guard<std::mutex> lock(mutex_);

    switch (static_cast<OpCode>(op)) {
    case OpCode::RequestControl:
        if (controller_ != NoConnection && controller_ != conn)
            return respond(op, ResultCode::ControlNotPermitted);
        controller_ = conn;
        fallbackPending_ = false; // an app took over again
        return respond(op, ResultCode::Success);

    case OpCode::Reset:
    case OpCode::SetTargetPower:
    case OpCode::StartOrResume:
    case OpCode::StopOrPause:
        break; // supported, need control

    default:
        return respond(op, ResultCode::OpCodeNotSupported);
    }

    if (controller_ != conn)
        return respond(op, ResultCode::ControlNotPermitted);

    switch (static_cast<OpCode>(op)) {
    case OpCode::Reset: {
        // Reset returns the target to its default: a load the app set stays nowhere once it is gone. Control stays
        // with the app (lenient on purpose: some apps send Reset and then Set Target Power without asking again).
        if (currentTarget_() > FallbackWatt)
            applyTarget_(FallbackWatt);
        Reply reply = respond(op, ResultCode::Success);
        reply.statusLength = encodeStatus(StatusCode::Reset, reply.status);
        return reply;
    }
    case OpCode::SetTargetPower: {
        if (length != 3)
            return respond(op, ResultCode::InvalidParameter);
        const int requested = static_cast<int16_t>(data[1] | (data[2] << 8));
        if (requested < 0)
            return respond(op, ResultCode::InvalidParameter);
        const int accepted = applyTarget_(requested);
        Reply reply = respond(op, ResultCode::Success);
        reply.statusLength = encodeTargetPowerChanged(accepted, reply.status);
        return reply;
    }
    case OpCode::StartOrResume: {
        Reply reply = respond(op, ResultCode::Success);
        reply.statusLength = encodeStatus(StatusCode::StartedOrResumedByUser, reply.status);
        return reply;
    }
    case OpCode::StopOrPause: {
        if (length != 2 || (data[1] != 0x01 && data[1] != 0x02))
            return respond(op, ResultCode::InvalidParameter);
        Reply reply = respond(op, ResultCode::Success);
        reply.status[0] = static_cast<uint8_t>(StatusCode::StoppedOrPausedByUser);
        reply.status[1] = data[1]; // 1 = stop, 2 = pause
        reply.statusLength = 2;
        return reply;
    }
    default:
        return respond(op, ResultCode::OpCodeNotSupported); // unreachable
    }
}

size_t TrainerControl::onDisconnect(uint16_t conn, uint32_t nowMs, uint8_t* status) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (conn != controller_)
        return 0;
    controller_ = NoConnection;
    fallbackPending_ = true;
    controllerLostAtMs_ = nowMs;
    return encodeStatus(StatusCode::ControlPermissionLost, status);
}

size_t TrainerControl::tick(uint32_t nowMs, uint8_t* status) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!fallbackPending_ || nowMs - controllerLostAtMs_ < HoldAfterDisconnectMs)
        return 0;
    fallbackPending_ = false;
    if (currentTarget_() <= FallbackWatt)
        return 0;
    return encodeTargetPowerChanged(applyTarget_(FallbackWatt), status);
}

} // namespace ftms
