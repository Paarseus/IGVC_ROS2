// source: https://github.com/UTNuclearRobotics/FRCCan/blob/8e2d039/src/frcCan.cpp (commit 8e2d039, fetched 2026-09-28)
#include "frcCan.hpp"

template <typename T>
static T FRCCan::unpack(const uint8_t* b, int offset = 0)
{
  T v;
  std::memcpy(&v, b + offset, sizeof(T));
  return v;
}
template <typename T>
uint8_t* FRCCan::pack(T payload, int offset = 0) {
    std::memcpy(&msg.buf[offset], &payload, sizeof(T));
}

uint32_t FRCCan::encodeMsgID(FRCCan::MsgID frameID) {
    ASSERT(frameID.apiClass < 64);
    ASSERT(frameID.apiIndex < 16);
    ASSERT(frameID.deviceID < 64);
    uint32_t devType = (static_cast<uint32_t>(frameID.deviceType)   & 0x1F) << 24;
    uint32_t manufacturer = (static_cast<uint32_t>(frameID.manufacturer) & 0xFF) << 16;
    uint32_t apiClass= (static_cast<uint32_t>(frameID.apiClass)     & 0x3F) << 10;
    uint32_t apiIdx  = (static_cast<uint32_t>(frameID.apiIndex)     & 0x0F) << 6;
    uint32_t devID   = (static_cast<uint32_t>(frameID.deviceID)     & 0x3F);
    return (devType | manufacturer | apiClass | apiIdx | devID);
}

FRCCan::MsgID FRCCan::decodeMsgID(uint32_t id) {
    MsgID ret = {
        .deviceType   = static_cast<DeviceType>((id >> 24) & 0x1F),
        .manufacturer = static_cast<Manufacturer>((id >> 16) & 0xFF),
        .apiClass     = static_cast<uint8_t>((id >> 10) & 0x3F),
        .apiIndex     = static_cast<uint8_t>((id >> 6)  & 0x0F),
        .deviceID     = static_cast<uint8_t>(id        & 0x3F)
    };
    return ret;
}
void FRCCan::msgIDPrint(FRCCan::MsgID id) {
    Serial.print("msg.devID: ");
    Serial.println(static_cast<uint32_t>(id.deviceID));
    Serial.print(" | id.deviceType: ");
    Serial.println(static_cast<uint32_t>(id.deviceType));
    Serial.print(" | id.manufacturer: ");
    Serial.println(static_cast<uint32_t>(id.manufacturer));
    Serial.print(" | msg.apiClass: ");
    Serial.println(static_cast<uint32_t>(id.apiClass));
    Serial.print(" | msg.apiIndex: ");
    Serial.println(static_cast<uint32_t>(id.apiIndex));
    return;
}

void FRCCan::payloadPrint(const uint8_t* buf, uint8_t len) {
    for (int i  = 0; i < len; i++) {
        Serial.print("[");
        Serial.print(buf[i]);
        Serial.print("] ");
    }
    Serial.println();
}

const FRCCan::APIEntry* FRCCan::CanManager::apiLookup(MsgID id) {
    ASSERT(id.apiClass < kMaxApiClasses);
    ASSERT(id.apiIndex < kMaxApiIndexes);
    for (const auto& table : tables) {
        if (id.deviceType == table.deviceType && id.manufacturer == table.manufacturer) {
            return table.lookup(id.apiClass, id.apiIndex);
        }
    }
    return nullptr;
}

void FRCCan::CanManager::registerDeviceProtocol(const DeviceProtocol& prot) {
    DispatchTable table;
    table.initialize(prot);
    tables.push_back(table);
}

void FRCCan::CanManager::registerBundle(const DeviceProtocol* protocols, size_t count) {
    for(size_t i = 0; i < count; i++)
        registerDeviceProtocol(protocols[i]);
}


bool FRCCan::CanManager::receive(FRCCan::MsgID id, CAN_message_t &msg) {
    // Instant O(1) pointer jump. Zero searching, zero hashing.
    if (id.apiClass < kMaxApiClasses && id.apiIndex < kMaxApiIndexes) {
        const APIEntry* entry = apiLookup(id);
        if (entry) {
            ASSERT(entry.direction == MsgHandlerType::RX);
            entry->fn(id, msg, nullptr);
            return true;
        }
    }
    return false;
}

void FRCCan::CanManager::canSniff(const CAN_message_t &msg) {
    MsgID id = decodeMsgID(msg.id);
    #if DEBUG
        msgIDPrint(id);
    #endif
    bool success = receive(id, msg);
    ASSERT(success);
}

bool FRCCan::CanManager::broadcastMsg(FRCCan::BroadcastMsg signal) {
    FRCCan::MsgID broadcastID = {
        .deviceType = DeviceType::BROADCAST,
        .manufacturer = Manufacturer::BROADCAST,
        .apiClass = 0,
        .apiIndex = static_cast<uint8_t>(signal),
        .deviceID = 0
    };
    CAN_message_t newMsg{};
    newMsg.flags.extended = true;
    newMsg.len = 0;
    newMsg.id = encodeMsgID(broadcastID);
    bool success = _can.write(newMsg);
    ASSERT(success);
    return success;
}
bool FRCCan::CanManager::universalHeartbeat(bool enabled) {
    static uint32_t lastHeartbeat = 0;
    constexpr uint32_t PERIOD_MS = 20;
    if (millis() - lastHeartbeat < PERIOD_MS) {
      return false;
    }

    lastHeartbeat = millis();
    if (enabled) {
        CAN_message_t newMsg{};
        newMsg.flags.extended = true;
        newMsg.len = 8;

        newMsg.id = UNIVERSAL_HEARTBEAT_ID;
        newMsg.buf[4] = 0x18; // testMode and systemWatchdog on

        bool success = _can.write(newMsg);
        ASSERT(success);
        return success;
    }
    return false;
}

bool FRCCan::CanManager::send(MsgID id, const void* context) {
    const APIEntry* entry = apiLookup(id);
    if (entry) {
        if(entry.direction != MsgHandlerType::TX)
            return false;
        CAN_message_t newMsg{};
        newMsg.flags.extended = true;
        newMsg.id = encodeMsgID(id);
        entry->fn(id, newMsg, context);
        bool success = _can.write(newMsg);
        ASSERT(success);
        return success;
    }
    return false;
    
}