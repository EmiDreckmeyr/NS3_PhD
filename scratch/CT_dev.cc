//Utilities
#include "ns3/command-line.h"
#include "ns3/log.h"
#include "ns3/application.h"
#include "ns3/callback.h"
#include "ns3/simulator.h"
#include "ns3/names.h"
#include <fstream>
#include <vector>
#include <map>
#include <unordered_set>
#include <cmath>
#include <iomanip>
#include <sstream>
//Losses
#include "ns3/okumura-hata-propagation-loss-model.h"
#include "ns3/propagation-environment.h"
#include "ns3/propagation-module.h"
//Device mobility and position
#include "ns3/constant-position-mobility-model.h"
#include "ns3/mobility-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/animation-interface.h"
#include "ns3/position-allocator.h"
//LoRa End Devices and Gateways
#include "ns3/end-device-lora-phy.h"
#include "ns3/end-device-lorawan-mac.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/lora-helper.h"
#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/lora-net-device.h"
#include "ns3/lora-frame-header.h"
#include "ns3/lorawan-mac-header.h"
#include "ns3/lora-phy.h"
#include "ns3/lora-tag.h"
//Energy Models
#include "ns3/basic-energy-source.h"
#include "ns3/lora-radio-energy-model.h"
#include "ns3/lora-radio-energy-model-helper.h"
#include "ns3/basic-energy-source-helper.h"
//Periodic Sender
#include "ns3/node-container.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/packet.h"
#include "ns3/random-variable-stream.h"
// Network Server and Forwarder
#include "ns3/internet-stack-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/forwarder-helper.h"
#include "ns3/network-server-helper.h"
//Namespaces
using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE("CT_dev");

/**********************
 * Global simulation parameters
 **********************/
static const uint32_t SIM_END_HOURS = 24;          // Total simulation time in hours
static const uint32_t N_END_DEVICES = 20;          // Number of end devices
static const uint32_t N_GATEWAYS = 1;             // Number of gateways
static const Time PERIOD_SENDER = Minutes(15);    // Periodic sender interval
static const double GATEWAY_X_POS = -800.0;        // Gateway X coordinate in meters
static const double GATEWAY_Y_POS = 100.0;          // Gateway Y coordinate in meters
// Global variable to toggle confirmed/unconfirmed messages
static const bool USE_CONFIRMED_UPLINK = true;    // true = confirmed, false = unconfirmed
// Global variable to toggle increased polling at 12th hour
static const bool ENABLE_12TH_HOUR_POLLING = false; // true = enable increased polling, false = maintain default period

/**********************
 * Global variables
 **********************/
static std::vector<uint32_t> g_ackCount;
static std::unordered_set<uint32_t> receivedPacketIds; // track uniques
static double totalToA_RX1 = 0.0;  // Total time on air in seconds for RX1
static double totalToA_RX2 = 0.0;  // Total time on air in seconds for RX2
static std::vector<double> hourlyToA_RX1;  // Store ToA for each hour for RX1
static std::vector<double> hourlyToA_RX2;  // Store ToA for each hour for RX2
static double totalEndDeviceToA = 0.0;  // Total ToA for furthest end device (seconds)
static std::vector<double> hourlyEndDeviceToA;  // Store furthest end device ToA
static uint32_t furthestDeviceIndex = 0;  // Index of furthest end device

//Packet Tracking
std::vector<int> packetsSent(6, 0);     // DR5 -> DR0
std::vector<int> packetsReceived(6, 0);
std::map<uint32_t, uint32_t> packetSenderMap;  // unique packet id -> sender node id
std::vector<int> packetsReceivedPerNode;


/**********************
 * Utility Functions
 **********************/
double CalculateTimeOnAir(uint32_t payloadSize, uint8_t sf, double bandwidthHz = 125000.0, uint8_t codingRate = 1, bool crcEnabled = true, bool headerEnabled = true, uint8_t nPreamble = 8) {
    // Validate spreading factor
    if (sf < 7 || sf > 12) {
        NS_LOG_ERROR("Invalid SF " << unsigned(sf) << " in CalculateTimeOnAir, using default SF7");
        sf = 7;  // Default to SF7 if invalid
    }

    // Validate bandwidth to prevent division by zero
    if (bandwidthHz <= 0) {
        NS_LOG_ERROR("Invalid bandwidth " << bandwidthHz << "Hz, using default 125000Hz");
        bandwidthHz = 125000.0;
    }

    // LoRa time-on-air calculation based on EU868 parameters
    double Ts = (1 << sf) / bandwidthHz;  // Symbol duration (seconds)
    double Tpreamble = (nPreamble + 4.25) * Ts;  // Preamble duration

    // Payload symbols calculation
    int DE = (sf >= 11) ? 1 : 0;  // Low data rate optimization for SF11 and SF12
    int H = headerEnabled ? 0 : 1;  // Header enabled (0) or disabled (1)
    int CR = codingRate;  // Coding rate (1 for 4/5, 2 for 4/6, etc.)
    double payloadSymbNb = 8 + std::max(std::ceil((8.0 * payloadSize - 4.0 * sf + 28 + 16 * crcEnabled - 20 * H) / (4.0 * (sf - 2 * DE))) * (CR + 4), 0.0);
    double Tpayload = payloadSymbNb * Ts;

    double toa = Tpreamble + Tpayload;
    if (!std::isfinite(toa) || toa < 0) {
        NS_LOG_ERROR("Calculated ToA is invalid (" << toa << "s) for SF" << unsigned(sf) << ", payloadSize=" << payloadSize);
        return 0.0;  // Return 0 to avoid accumulating invalid ToA
    }

    NS_LOG_DEBUG("Calculated ToA: " << toa << "s for SF" << unsigned(sf) << ", payloadSize=" << payloadSize);
    return toa;
}

void FindFurthestDevice(NodeContainer endDevices, NodeContainer gateways) {
    Ptr<Node> gateway = gateways.Get(0);  // Assume single gateway
    Ptr<MobilityModel> gatewayMobility = gateway->GetObject<MobilityModel>();
    Vector gatewayPos = gatewayMobility->GetPosition();
    double maxDistance = 0.0;

    for (uint32_t i = 0; i < endDevices.GetN(); ++i) {
        Ptr<Node> device = endDevices.Get(i);
        Ptr<MobilityModel> deviceMobility = device->GetObject<MobilityModel>();
        Vector devicePos = deviceMobility->GetPosition();
        double distance = std::sqrt(std::pow(devicePos.x - gatewayPos.x, 2) +
                                    std::pow(devicePos.y - gatewayPos.y, 2) +
                                    std::pow(devicePos.z - gatewayPos.z, 2));
        if (distance > maxDistance) {
            maxDistance = distance;
            furthestDeviceIndex = i;
        }
    }
    NS_LOG_INFO("Furthest end device is index " << furthestDeviceIndex << " at distance " << maxDistance << " meters");
}

/**********************
 * Ack tracing callback
 **********************/
void OnGatewayAck(uint32_t gwIndex, Ptr<const Packet> p) {
    if (gwIndex >= g_ackCount.size()) {
        g_ackCount.resize(gwIndex + 1, 0);
    }
    g_ackCount[gwIndex]++;
    //NS_LOG_INFO("Gateway " << gwIndex
    //            << " sent ACK at " << Simulator::Now().GetSeconds() << "s");
}

/**********************
 * Gateway PHY StartSending tracer (detect downlink ACKs)
 **********************/
// In OnGatewayPhyStartSending, add ToA calculation (assuming single gateway)
void OnGatewayPhyStartSending(uint32_t gwIndex, Ptr<const Packet> packet, uint32_t phyIndex) {
    Ptr<Packet> copy = packet->Copy();
    LorawanMacHeader macHdr;
    copy->RemoveHeader(macHdr);

    if (macHdr.GetMType() == LorawanMacHeader::UNCONFIRMED_DATA_DOWN ||
        macHdr.GetMType() == LorawanMacHeader::CONFIRMED_DATA_DOWN) {
        LoraFrameHeader frameHdr;
        copy->RemoveHeader(frameHdr);

        if (frameHdr.GetAck()) {
            if (gwIndex >= g_ackCount.size()) {
                g_ackCount.resize(gwIndex + 1, 0);
            }
            g_ackCount[gwIndex]++;
           // NS_LOG_INFO("Gateway " << gwIndex << " transmitted ACK at " << Simulator::Now().GetSeconds() << "s");
        }
    }

    LoraTag tag;
    double frequency = 0.0;
    uint8_t sf;
    if (packet->PeekPacketTag(tag)) {
        sf = tag.GetSpreadingFactor();
        frequency = tag.GetFrequency();
        if (sf < 7 || sf > 12) {
            //NS_LOG_ERROR("Invalid SF " << unsigned(sf) << " for gateway " << gwIndex << ", forcing SF7");
            sf = 7;
            tag.SetSpreadingFactor(sf);
            Ptr<Packet> packetCopy = packet->Copy();
            packetCopy->ReplacePacketTag(tag);
        }
    } else {
        NS_LOG_ERROR("No LoraTag found for gateway " << gwIndex << ", forcing SF7");
        sf = 7;
        tag.SetSpreadingFactor(sf);
        Ptr<Packet> packetCopy = packet->Copy();
        packetCopy->AddPacketTag(tag);
    }

    uint32_t size = packet->GetSize();
    double toa = CalculateTimeOnAir(size, sf, 125000.0, 1, true, true, 8);
    if (frequency == 869525000.0) {
        totalToA_RX2 += toa;
        //NS_LOG_INFO("Gateway " << gwIndex << " transmitted packet (RX2) with SF" << unsigned(sf) << ", ToA " << toa << "s, cumulative RX2 ToA " << totalToA_RX2 << "s");
    } else {
        totalToA_RX1 += toa;
        //NS_LOG_INFO("Gateway " << gwIndex << " transmitted packet (RX1) with SF" << unsigned(sf) << ", ToA " << toa << "s, cumulative RX1 ToA " << totalToA_RX1 << "s");
    }
}

void OnEndDevicePhyStartSending(uint32_t deviceIndex, Ptr<const Packet> packet, uint8_t dr) {
    if (deviceIndex != furthestDeviceIndex) {
        return;  // Only process packets from furthest device
    }

    uint8_t sf = 7;  // Default SF
    LoraTag tag;
    if (packet->PeekPacketTag(tag)) {
        sf = tag.GetSpreadingFactor();
        if (sf < 7 || sf > 12) {
            NS_LOG_ERROR("Invalid SF " << unsigned(sf) << " for end device " << deviceIndex << ", using SF7");
            sf = 7;
        }
    } else {
        NS_LOG_ERROR("No LoraTag found for end device " << deviceIndex << " packet, using default SF7");
        sf = (dr <= 5) ? (12 - dr) : 7;  // Fallback to DR-based SF
    }
    uint32_t size = packet->GetSize();
    double toa = CalculateTimeOnAir(size, sf, 125000.0, 1, true, true, 8);
    totalEndDeviceToA += toa;
   // NS_LOG_INFO("End device " << deviceIndex << " transmitted packet with SF" << unsigned(sf) << ", ToA " << toa << "s, cumulative end device ToA " << totalEndDeviceToA << "s");
}

void OnEndDeviceSentNewPacket(uint32_t deviceIndex, Ptr<EndDeviceLorawanMac> mac, Ptr<const Packet> packet) {
    uint8_t dr = mac->GetDataRate();
    //uint8_t sf = (dr <= 5) ? (12 - dr) : 7;
    //NS_LOG_DEBUG("End device " << deviceIndex << " sent packet with DR" << unsigned(dr) << " (SF" << unsigned(sf) << ") at " << Simulator::Now().GetSeconds() << "s");
    LoraTag tag;
    if (!packet->PeekPacketTag(tag)) {
        NS_LOG_ERROR("No LoraTag found in SentNewPacket for end device " << deviceIndex);
    }
    if (deviceIndex == furthestDeviceIndex) {
        OnEndDevicePhyStartSending(deviceIndex, packet, dr);
    }
}

/**********************
 * Duty Cycle Check
 **********************/
void CheckGatewayDutyCycle() {
    double maxToA_RX1 = 36.0;  // 1% of 3600 seconds (ETSI limit for RX1 in EU868 sub-bands g1/g2)
    double maxToA_RX2 = 360.0; // 10% of 3600 seconds (ETSI limit for RX2 in EU868 sub-band g3)
    hourlyToA_RX1.push_back(totalToA_RX1);
    hourlyToA_RX2.push_back(totalToA_RX2);
    NS_LOG_INFO("DutyCycleChecker: Gateway RX1 time on air in hour " << hourlyToA_RX1.size() << ": " << totalToA_RX1 << " seconds");
    if (totalToA_RX1 <= maxToA_RX1) {
        NS_LOG_INFO("DutyCycleChecker: Gateway RX1 compliant with ETSI 1% duty cycle.");
    } else {
        NS_LOG_INFO("DutyCycleChecker: Gateway RX1 non-compliant with ETSI 1% duty cycle (exceeds 36s).");
    }
    NS_LOG_INFO("DutyCycleChecker: Gateway RX2 time on air in hour " << hourlyToA_RX2.size() << ": " << totalToA_RX2 << " seconds");
    if (totalToA_RX2 <= maxToA_RX2) {
        NS_LOG_INFO("DutyCycleChecker: Gateway RX2 compliant with ETSI 10% duty cycle.");
    } else {
        NS_LOG_INFO("DutyCycleChecker: Gateway RX2 non-compliant with ETSI 10% duty cycle (exceeds 360s).");
    }
    totalToA_RX1 = 0.0;  // Reset for next hour
    totalToA_RX2 = 0.0;  // Reset for next hour
    if (Simulator::Now().GetSeconds() < SIM_END_HOURS * 3600.0) {
        Simulator::Schedule(Seconds(3600.0), &CheckGatewayDutyCycle);
    }
}

void CheckEndDeviceDutyCycle() {
    double maxToA = 36.0;  // 1% of 3600 seconds (ETSI limit for EU868 sub-band)
    hourlyEndDeviceToA.push_back(totalEndDeviceToA);
    NS_LOG_INFO("DutyCycleChecker: Furthest end device total time on air in hour " << hourlyEndDeviceToA.size() << ": " << totalEndDeviceToA << " seconds");
    if (totalEndDeviceToA <= maxToA) {
        NS_LOG_INFO("DutyCycleChecker: Furthest end device compliant with ETSI 1% duty cycle.");
    } else {
        NS_LOG_INFO("DutyCycleChecker: Furthest end device non-compliant with ETSI 1% duty cycle (exceeds 36s).");
    }
    totalEndDeviceToA = 0.0;  // Reset for next hour
    if (Simulator::Now().GetSeconds() < SIM_END_HOURS * 3600.0) {
        Simulator::Schedule(Seconds(3600.0), &CheckEndDeviceDutyCycle);
    }
}


/***************
 * UniquePacketIdTag Definition
 ***************/
class UniquePacketIdTag : public Tag {
public:
    UniquePacketIdTag() : m_id(0) {}
    UniquePacketIdTag(uint32_t id) : m_id(id) {}

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("UniquePacketIdTag")
            .SetParent<Tag>()
            .AddConstructor<UniquePacketIdTag>();
        return tid;
    }
    virtual TypeId GetInstanceTypeId(void) const { return GetTypeId(); }
    virtual void Serialize(TagBuffer i) const { i.WriteU32(m_id); }
    virtual void Deserialize(TagBuffer i) { m_id = i.ReadU32(); }
    virtual uint32_t GetSerializedSize(void) const { return 4; }
    virtual void Print(std::ostream &os) const { os << "UniquePacketId=" << m_id; }
    void SetId(uint32_t id) { m_id = id; }
    uint32_t GetId() const { return m_id; }
private:
    uint32_t m_id;
};

/***************
 * Custom PeriodicSender application adding UniquePacketIdTag
 ***************/
static uint32_t globalPacketId = 0;

class TaggingPeriodicSender : public Application {
public:
    TaggingPeriodicSender() : m_period(Seconds(60)), m_packetSize(20), m_packetsSent(0) {}

    void Setup(Ptr<Node> node, Ptr<NetDevice> device, Time period, uint32_t packetSize) {
        m_node = node;
        m_device = device;
        m_period = period;
        m_packetSize = packetSize;
    }

    static TypeId GetTypeId(void) {
        static TypeId tid = TypeId("TaggingPeriodicSender")
            .SetParent<Application>()
            .AddConstructor<TaggingPeriodicSender>();
        return tid;
    }

    virtual void StartApplication() override {
        ScheduleNextTx(Seconds(0));
    }

    virtual void StopApplication() override {
        Simulator::Cancel(m_sendEvent);
    }
    void SetPeriod(Time newPeriod) {
        Simulator::Cancel(m_sendEvent);
        m_period = newPeriod;
        ScheduleNextTx(Seconds(0));  // Reschedule immediately with new period
    }

private:
    void ScheduleNextTx(Time delay) {
        m_sendEvent = Simulator::Schedule(delay, &TaggingPeriodicSender::SendPacket, this);
    }

    void SendPacket() {
        Ptr<Packet> packet = Create<Packet>(m_packetSize);
        UniquePacketIdTag idTag(++globalPacketId);
        packet->AddPacketTag(idTag);
    
        LorawanMacHeader macHdr;
        if (USE_CONFIRMED_UPLINK) {
            macHdr.SetMType(LorawanMacHeader::CONFIRMED_DATA_UP);
        } else {
            macHdr.SetMType(LorawanMacHeader::UNCONFIRMED_DATA_UP);
        }
        packet->AddHeader(macHdr);
    
        // Add LoraTag with SF based on device's data rate
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(m_device);
        if (!loraNetDevice) {
            NS_LOG_ERROR("Device is not a LoraNetDevice");
            return;
        }
        Ptr<EndDeviceLorawanMac> mac = DynamicCast<EndDeviceLorawanMac>(loraNetDevice->GetMac());
        if (!mac) {
            NS_LOG_ERROR("MAC is not an EndDeviceLorawanMac");
            return;
        }
        uint8_t dr = mac->GetDataRate();
        uint8_t sf = (dr <= 5) ? (12 - dr) : 7;
        LoraTag tag;
        tag.SetSpreadingFactor(sf);
        packet->AddPacketTag(tag);
        NS_LOG_DEBUG("Added LoraTag with SF" << unsigned(sf) << " for packet from device");
    
        loraNetDevice->GetMac()->Send(packet);
    
        m_packetsSent++;
        ScheduleNextTx(m_period);
    }

    Ptr<Node> m_node;
    Ptr<NetDevice> m_device;
    Time m_period;
    uint32_t m_packetSize;
    EventId m_sendEvent;
    uint32_t m_packetsSent;
};

/***************
 * Callbacks for tracing packets at PHY layer
 ***************/
void OnTransmissionCallback(uint32_t deviceIndex, Ptr<const Packet> packet, uint32_t phyIndex) {
    LoraTag tag;
    if (packet->PeekPacketTag(tag)) {
        int idx = tag.GetSpreadingFactor() - 7;
        if (idx >= 0 && idx < 6) {
            packetsSent.at(idx)++;
        }
    }
    UniquePacketIdTag idTag;
    if (packet->PeekPacketTag(idTag)) {
        packetSenderMap[idTag.GetId()] = deviceIndex;
    }
}

void OnPacketReceptionCallback(Ptr<const Packet> packet, uint32_t phyIndex) {
    LoraTag tag;
    if (packet->PeekPacketTag(tag)) {
        int idx = tag.GetSpreadingFactor() - 7;
        if (idx >= 0 && idx < 6) {
            packetsReceived.at(idx)++;
        }
    }
    UniquePacketIdTag idTag;
    if (packet->PeekPacketTag(idTag)) {
        uint32_t packetId = idTag.GetId();

        if (receivedPacketIds.find(packetId) != receivedPacketIds.end()) {
            return;
        }

        receivedPacketIds.insert(packetId);

        auto it = packetSenderMap.find(packetId);
        if (it != packetSenderMap.end()) {
            uint32_t senderId = it->second;
            if (senderId < packetsReceivedPerNode.size()) {
                packetsReceivedPerNode[senderId]++;
            }
        }
    }
}

void OnMacPacketOutcome(uint8_t transmissions, bool successful, Time firstAttempt, Ptr<Packet> packet) {
   // if (successful) {
   //     NS_LOG_INFO("Confirmed uplink succeeded after " << unsigned(transmissions) << " attempts.");
    //} else {
    //    NS_LOG_INFO("Confirmed uplink failed after " << unsigned(transmissions) << " attempts.");
   // }
}

/***************
 * Main simulation code
 ***************/
int main(int argc, char *argv[]) {
    LogComponentEnable("CT_dev", LOG_LEVEL_INFO);
    //LogComponentEnable("NetworkServer", LOG_LEVEL_ALL);
    //LogComponentEnable("GatewayLorawanMac", LOG_LEVEL_ALL);
    //LogComponentEnableAll(LOG_PREFIX_FUNC);
    //LogComponentEnableAll(LOG_PREFIX_NODE);
    //LogComponentEnableAll(LOG_PREFIX_TIME);
    NS_LOG_INFO("Starting CT_dev simulation...");

    /**********************
     * Channel Setup
     **********************/
    Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
    loss->SetPathLossExponent(3.9);
    loss->SetReference(1.0, 32.4);

    Ptr<NakagamiPropagationLossModel> fading = CreateObject<NakagamiPropagationLossModel>();
    fading->SetAttribute("m0", DoubleValue(1.0));
    fading->SetAttribute("m1", DoubleValue(1.5));
    fading->SetAttribute("m2", DoubleValue(3.0));
    loss->SetNext(fading);

    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();
    Ptr<LoraChannel> channel = CreateObject<LoraChannel>(loss, delay);
    NS_LOG_INFO("Channel setup complete.");

    /**********************
     * Mobility Setup
     **********************/
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();

    const double spacing = 5.0;
    const double endDeviceHeight = 1.5; // Height for end devices (meters)
    const double gatewayHeight = 10.0;  // Height for gateways (meters)
    const double networkServerHeight = 10.0; // Height for network server (meters)
    for (uint32_t i = 0; i < N_END_DEVICES; ++i) {
        double x = i * spacing + 5;
        double y = (i % 2 == 0) ? 0 : 1;
        allocator->Add(Vector(x, y, endDeviceHeight));
        NS_LOG_INFO("Placed end device " << i << " at x=" << x << ", y=" << y << ", z=" << endDeviceHeight);
    }
    allocator->Add(Vector(GATEWAY_X_POS, GATEWAY_Y_POS, gatewayHeight));
    NS_LOG_INFO("Placed gateway at x=" << GATEWAY_X_POS << ", y=" << GATEWAY_Y_POS << ", z=" << gatewayHeight);
    allocator->Add(Vector(GATEWAY_X_POS + 10, GATEWAY_Y_POS + 10, networkServerHeight));
    NS_LOG_INFO("Placed network server at x=" << GATEWAY_X_POS + 10 << ", y=" << GATEWAY_Y_POS + 10 << ", z=" << networkServerHeight);

    mobility.SetPositionAllocator(allocator);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    /**********************
     * Nodes Creation
     **********************/
    NodeContainer endDevices;
    endDevices.Create(N_END_DEVICES);
    NodeContainer gateways;
    gateways.Create(N_GATEWAYS);
    Ptr<Node> networkServer = CreateObject<Node>();


    mobility.Install(endDevices);
    mobility.Install(gateways);
    mobility.Install(networkServer);
    NS_LOG_INFO("Nodes creation complete..");

    // Compute distances to gateway
    std::vector<double> distances(N_END_DEVICES);
    Ptr<MobilityModel> gwMob = gateways.Get(0)->GetObject<MobilityModel>();
    Vector gwPos = gwMob->GetPosition();
    for (uint32_t i = 0; i < N_END_DEVICES; ++i) {
        Ptr<MobilityModel> devMob = endDevices.Get(i)->GetObject<MobilityModel>();
        Vector devPos = devMob->GetPosition();
        double dx = devPos.x - gwPos.x;
        double dy = devPos.y - gwPos.y;
        double dz = devPos.z - gwPos.z;
        distances[i] = std::sqrt(dx * dx + dy * dy + dz * dz);
    }
    NS_LOG_INFO("Distances to gateway computed.");

    FindFurthestDevice(endDevices, gateways);

    packetsReceivedPerNode.resize(endDevices.GetN(), 0);

    /**********************
     * Helpers Setup
     **********************/
    LoraPhyHelper phyHelper;
    phyHelper.SetChannel(channel);
    LorawanMacHelper macHelper;
    macHelper.SetRegion(LorawanMacHelper::EU);
    LoraHelper helper;
    helper.EnablePacketTracking();

    /**********************
     * Devices Setup
     **********************/
    phyHelper.SetDeviceType(LoraPhyHelper::ED);
    macHelper.SetDeviceType(LorawanMacHelper::ED_A);
    macHelper.SetRegion(LorawanMacHelper::EU);
    NetDeviceContainer endDevicesNet = helper.Install(phyHelper, macHelper, endDevices);

    phyHelper.SetDeviceType(LoraPhyHelper::GW);
    macHelper.SetDeviceType(LorawanMacHelper::GW);
    NetDeviceContainer gatewaysNet = helper.Install(phyHelper, macHelper, gateways);

    g_ackCount.resize(gatewaysNet.GetN(), 0);
    for (uint32_t g = 0; g < gatewaysNet.GetN(); ++g) {
        Ptr<LoraNetDevice> gwDevice = DynamicCast<LoraNetDevice>(gatewaysNet.Get(g));
        gwDevice->GetPhy()->TraceConnectWithoutContext("StartSending", MakeBoundCallback(&OnGatewayPhyStartSending, g));
    }

    for (uint32_t i = 0; i < endDevicesNet.GetN(); ++i) {
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(endDevicesNet.Get(i));
        Ptr<EndDeviceLorawanMac> mac = DynamicCast<EndDeviceLorawanMac>(loraNetDevice->GetMac());
        if (USE_CONFIRMED_UPLINK) {
            mac->SetMType(LorawanMacHeader::CONFIRMED_DATA_UP);
        } else {
            mac->SetMType(LorawanMacHeader::UNCONFIRMED_DATA_UP);
        }
        mac->TraceConnectWithoutContext("RequiredTransmissions", MakeCallback(&OnMacPacketOutcome));
        mac->TraceConnectWithoutContext("SentNewPacket", MakeBoundCallback(&OnEndDeviceSentNewPacket, i, mac));
    }
    NS_LOG_INFO("Devices setup...");

    /**********************
     * Backbone Network Setup (Point-to-Point for gateways to NS)
     **********************/
    PointToPointHelper pointToPoint;
    pointToPoint.SetDeviceAttribute("DataRate", StringValue("5Mbps"));
    pointToPoint.SetChannelAttribute("Delay", TimeValue(MilliSeconds(2)));

    P2PGwRegistration_t gwRegistration;
    for (uint32_t i = 0; i < gateways.GetN(); ++i) {
        NetDeviceContainer p2pDevices = pointToPoint.Install(networkServer, gateways.Get(i));
        Ptr<PointToPointNetDevice> serverP2PNetDev = DynamicCast<PointToPointNetDevice>(p2pDevices.Get(0));
        gwRegistration.emplace_back(serverP2PNetDev, gateways.Get(i));
    }

    NS_LOG_INFO("Server setup complete..");

    /**********************
     * Forwarder and Network Server Setup
     **********************/
    ForwarderHelper forwarderHelper;
    ApplicationContainer forwarderApps = forwarderHelper.Install(gateways);

    NetworkServerHelper networkServerHelper;
    networkServerHelper.SetGatewaysP2P(gwRegistration);
    networkServerHelper.SetEndDevices(endDevices);
    networkServerHelper.Install(networkServer);

    /**********************
     * Applications Setup
     **********************/
    Ptr<UniformRandomVariable> randStart = CreateObject<UniformRandomVariable>();
    randStart->SetAttribute("Min", DoubleValue(0.0));
    randStart->SetAttribute("Max", DoubleValue(PERIOD_SENDER.GetSeconds()));

    ApplicationContainer apps;
    for (uint32_t i = 0; i < endDevices.GetN(); ++i)
    {
        Ptr<TaggingPeriodicSender> app = CreateObject<TaggingPeriodicSender>();
        app->Setup(endDevices.Get(i), endDevicesNet.Get(i), PERIOD_SENDER, 24);
        endDevices.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(randStart->GetValue()));
        app->SetStopTime(Hours(SIM_END_HOURS));
        apps.Add(app);
    }

     // Schedule period change for hour 12 (39600s to 43200s) if enabled
     if (ENABLE_12TH_HOUR_POLLING) {
        Simulator::Schedule(Seconds(39600.0), [&apps]() {
            for (uint32_t i = 0; i < apps.GetN(); ++i)
            {
                Ptr<TaggingPeriodicSender> sender = DynamicCast<TaggingPeriodicSender>(apps.Get(i));
                if (sender)
                {
                    sender->SetPeriod(Seconds(90));
                }
            }
        });
        Simulator::Schedule(Seconds(43200.0), [&apps]() {
            for (uint32_t i = 0; i < apps.GetN(); ++i)
            {
                Ptr<TaggingPeriodicSender> sender = DynamicCast<TaggingPeriodicSender>(apps.Get(i));
                if (sender)
                {
                    sender->SetPeriod(PERIOD_SENDER);
                }
            }
        });
    }
    NS_LOG_INFO("Created application..");

    /**********************
     * Energy Setup
     **********************/
    NS_LOG_INFO("Setting up energy model...");
    NS_LOG_INFO("8 Ah at 3.3 V -> 95,040 J, use 10% of battery capacity for comms");
    BasicEnergySourceHelper basicSourceHelper;
    basicSourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(10000.0));
    basicSourceHelper.Set("BasicEnergySupplyVoltageV", DoubleValue(3.3));

    LoraRadioEnergyModelHelper radioEnergyHelper;
    radioEnergyHelper.Set("StandbyCurrentA", DoubleValue(0.0004));
    radioEnergyHelper.Set("TxCurrentA", DoubleValue(0.120));
    radioEnergyHelper.Set("RxCurrentA", DoubleValue(0.011));
    radioEnergyHelper.Set("SleepCurrentA", DoubleValue(0.0000015));
    radioEnergyHelper.SetTxCurrentModel("ns3::ConstantLoraTxCurrentModel",
                                        "TxCurrent",
                                        DoubleValue(0.090));

    EnergySourceContainer sources = basicSourceHelper.Install(endDevices);
    DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install(endDevicesNet, sources);
    NS_LOG_INFO("Energy model installed.");

    /**********************
     * Spreading Factors
     **********************/
    NS_LOG_INFO("Setting spreading factors...");
    LorawanMacHelper::SetSpreadingFactorsUp(endDevices, gateways, channel);
    NS_LOG_INFO("Spreading factors set.");

    std::vector<uint8_t> spreadingFactors;
    for (uint32_t i = 0; i < endDevices.GetN(); ++i) {
        Ptr<Node> node = endDevices.Get(i);
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(node->GetDevice(0));
        Ptr<EndDeviceLorawanMac> mac = DynamicCast<EndDeviceLorawanMac>(loraNetDevice->GetMac());
        uint8_t dr = mac->GetDataRate();
        uint8_t sf = 12 - dr;
        if (sf < 7 || sf > 12) {
            NS_LOG_ERROR("Invalid SF for node " << i << ": " << unsigned(sf));
            sf = 7;
        }
        spreadingFactors.push_back(sf);
        NS_LOG_INFO("End device " << i << " assigned SF" << unsigned(sf));
    }

    /**********************
     * Connect Traces for tracking on PHY
     **********************/
    for (uint32_t i = 0; i < endDevices.GetN(); ++i) {
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(endDevices.Get(i)->GetDevice(0));
        loraNetDevice->GetPhy()->TraceConnectWithoutContext("StartSending", MakeBoundCallback(&OnTransmissionCallback, i));
    }
    for (uint32_t i = 0; i < gateways.GetN(); ++i) {
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(gateways.Get(i)->GetDevice(0));
        loraNetDevice->GetPhy()->TraceConnectWithoutContext("ReceivedPacket", MakeCallback(&OnPacketReceptionCallback));
    }

    /**********************
     * NetAnim Setup
     **********************/
    AnimationInterface anim("CT_dev.xml");
    for (uint32_t i = 0; i < endDevices.GetN(); ++i) {
        anim.UpdateNodeDescription(endDevices.Get(i), "ED" + std::to_string(i));
        anim.UpdateNodeColor(endDevices.Get(i), 0, 255, 0);
    }
    anim.UpdateNodeDescription(gateways.Get(0), "GW");
    anim.UpdateNodeColor(gateways.Get(0), 255, 0, 0);
    anim.UpdateNodeDescription(networkServer, "NS");
    anim.UpdateNodeColor(networkServer, 0, 0, 255);

    /**********************
     * Schedule Duty Cycle Checks
     **********************/
    Simulator::Schedule(Seconds(3600.0), &CheckGatewayDutyCycle);
    Simulator::Schedule(Seconds(3600.0), &CheckEndDeviceDutyCycle);

    Simulator::Stop(Hours(SIM_END_HOURS));
    Simulator::Run();

    // Packet stats
    NS_LOG_INFO("Packets sent vs received per DR (SF7 -> SF12):");
    for (int i = 0; i < 6; i++) {
        std::cout << "DR" << (5 - i) << " (SF" << (7 + i) << "): Sent = "
                  << packetsSent.at(i) << ", Received = " << packetsReceived.at(i) << std::endl;
    }
    NS_LOG_INFO("Successful transmission to Gateway per end device:");
    for (uint32_t i = 0; i < packetsReceivedPerNode.size(); ++i) {
        std::cout << "Node " << i << " (SF" << unsigned(spreadingFactors[i]) << "): "
                  << packetsReceivedPerNode[i] << " packets received successfully by GW." << std::endl;
    }
    std::cout << "================= ACK SUMMARY =================\n";
    for (uint32_t g = 0; g < g_ackCount.size(); ++g) {
        std::cout << "Gateway " << g << " sent " << g_ackCount[g] << " ACKs\n";
    }
    std::cout << "==============================================\n";

    /**********************
     * Energy Logging
     **********************/
    NS_LOG_INFO("Logging energy consumption...");
    double simDuration = Simulator::Now().GetSeconds();
    NS_LOG_INFO("Total simulation duration: " << simDuration << " seconds");

    // Generate dynamic filename based on parameters
    std::ostringstream oss;
    oss << "CT_dev_";
    oss << (USE_CONFIRMED_UPLINK ? "confirmed" : "unconfirmed") << "_";
    oss << (ENABLE_12TH_HOUR_POLLING ? "increasedPolling" : "noPolling") << "_";
    oss << "gwX" << static_cast<int>(GATEWAY_X_POS)<< "m_";
    oss << "Ndev" << static_cast<int>(N_END_DEVICES) << ".tex";
    std::string filename = oss.str();

    std::ofstream texFile(filename);
    texFile << "\\documentclass{article}\n"
            << "\\usepackage{booktabs}\n"
            << "\\usepackage{geometry}\n"
            << "\\geometry{a4paper, margin=1in}\n"
            << "\\begin{document}\n"
            << "\\section{Simulation Parameters}\n"
            << "Simulation duration: " << simDuration << " seconds.\\\\\n"
            << "Number of end devices: " << N_END_DEVICES << "\\\\\n"
            << "Number of gateways: " << N_GATEWAYS << "\\\\\n"
            << "Sender period: " << PERIOD_SENDER.GetSeconds() << " seconds\\\\\n"
            << "Traffic type: " << (USE_CONFIRMED_UPLINK ? "Confirmed" : "Unconfirmed") << "\\\\\n"
            << "Gateway position: (" << GATEWAY_X_POS << ", " << GATEWAY_Y_POS << ")\\\\\n"
            << (ENABLE_12TH_HOUR_POLLING ? "Increased polling enabled at 12th hour." : "No increased polling.") << "\\\\\n\n"
            << "\\section{Gateway Distances to Nodes}\n"
            << "\\begin{tabular}{cc}\n"
            << "\\toprule\n"
            << "Node ID & Distance to GW (m) \\\\\n"
            << "\\midrule\n";
    for (uint32_t i = 0; i < distances.size(); ++i) {
        texFile << i << " & " << std::fixed << distances[i] << " \\\\\n";
    }
    texFile << "\\bottomrule\n"
            << "\\end{tabular}\n\n"
            << "\\section{Packet Transmission Statistics}\n"
            << "\\subsection{Per Spreading Factor}\n"
            << "\\begin{tabular}{ccc}\n"
            << "\\toprule\n"
            << "SF & Sent & Received \\\\\n"
            << "\\midrule\n";
    for (int i = 0; i < 6; ++i) {
        int sf = 7 + i;
        texFile << "SF" << sf << " & " << packetsSent[i] << " & " << packetsReceived[i] << " \\\\\n";
    }
    texFile << "\\bottomrule\n"
            << "\\end{tabular}\n\n"
            << "\\subsection{Per Node}\n"
            << "\\begin{tabular}{ccc}\n"
            << "\\toprule\n"
            << "Node ID & SF & Received \\\\\n"
            << "\\midrule\n";
    for (uint32_t i = 0; i < packetsReceivedPerNode.size(); ++i) {
        texFile << i << " & SF" << unsigned(spreadingFactors[i]) << " & " << packetsReceivedPerNode[i] << " \\\\\n";
    }
    texFile << "\\bottomrule\n"
            << "\\end{tabular}\n\n"
            << "Total unique packets received at GW: " << receivedPacketIds.size() << "\\\\\n"
            << "Total ACKs sent by GW: " << g_ackCount[0] << "\\\\\n\n"
            << "\\section{Energy Consumption Details}\n"
            << "\\begin{tabular}{ccc}\n"
            << "\\toprule\n"
            << "Node ID & Initial Energy (J) & Energy Consumed (J) \\\\\n"
            << "\\midrule\n";

    for (uint32_t i = 0; i < sources.GetN(); ++i) {
        Ptr<BasicEnergySource> src = sources.Get(i)->GetObject<BasicEnergySource>();
        double initialEnergy = src->GetInitialEnergy();
        double remainingEnergy = src->GetRemainingEnergy();
        double consumed = initialEnergy - remainingEnergy;

        NS_LOG_INFO("Node " << i << ": Initial=" << initialEnergy
                    << " J, Consumed=" << consumed << " J, Remaining=" << remainingEnergy << " J");

        texFile << i << " & " << initialEnergy << " & " << std::fixed <<  consumed << " \\\\\n";
    }

    texFile << "\\bottomrule\n"
            << "\\end{tabular}\n"
            << "\\end{document}\n";
    texFile.close();
    NS_LOG_INFO("Energy log saved to CT_Dev.tex");

    Simulator::Destroy();
    return 0;
}