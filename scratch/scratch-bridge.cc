
//==========================================================================================================
// LoRaWAN Network Simulation with coordinates and Pathloss from MatLab Brdige Scenario and ADR
// Simulates end devices with automatic spreading factor assignment (ADR)
// Features: Packet tracking, energy consumption, NetAnim visualization
//==========================================================================================================

// Utilities
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
#include <string>
#include <algorithm>

// Propagation and Channel Models
#include "ns3/okumura-hata-propagation-loss-model.h"
#include "ns3/propagation-module.h"

// Mobility and Positioning
#include "ns3/constant-position-mobility-model.h"
#include "ns3/mobility-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/animation-interface.h"
#include "ns3/position-allocator.h"

// LoRaWAN Devices and Components
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

// Energy Models
#include "ns3/basic-energy-source.h"
#include "ns3/lora-radio-energy-model.h"
#include "ns3/lora-radio-energy-model-helper.h"
#include "ns3/basic-energy-source-helper.h"

// Network Applications and Helpers
#include "ns3/node-container.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/packet.h"
#include "ns3/random-variable-stream.h"
#include "ns3/internet-stack-helper.h"
#include "ns3/point-to-point-helper.h"
#include "ns3/ipv4-address-helper.h"
#include "ns3/forwarder-helper.h"
#include "ns3/network-server-helper.h"

// Namespaces
using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE("enddeviceB");

/**
 * ============================================================================
 * SIMULATION PARAMETERS
 * ============================================================================
 */
static const uint32_t SIM_END_HOURS = 24;           // Simulation duration (hours)
static const Time PERIOD_SENDER = Minutes(15);      // Packet transmission period
static std::ofstream logFile;
static uint32_t N_END_DEVICES = 0;   // set from CSV at runtime
static uint32_t N_GATEWAYS = 0;      // set from CSV at runtime
std::vector<Vector> g_endDevicePositions;
std::vector<Vector> g_gatewayPositions;
Vector g_nsPosition;

// Configuration flags
static const bool USE_CONFIRMED_UPLINK = true;
static const bool ENABLE_12TH_HOUR_POLLING = false; 

/**
 * ============================================================================
 * GLOBAL STATE VARIABLES
 * ============================================================================
 */
std::vector<std::vector<double>> distancesToGateways; // resized in main() after CSV load
std::vector<uint32_t> g_ackCount;
std::unordered_set<uint32_t> receivedPacketIds;
std::vector<int> packetsSent(6, 0);           // SF7-SF12
std::vector<int> packetsReceived(6, 0);       // SF7-SF12
std::map<uint32_t, uint32_t> packetSenderMap;
std::vector<int> packetsReceivedPerNode;
std::vector<uint32_t> g_retransmissionsPerNode; 
std::vector<uint8_t> spreadingFactors;        // Actual assigned SFs
uint32_t furthestDeviceIndex = 0;
std::vector<uint32_t> g_energyEventCount; 

/**
 * ============================================================================
 * UTILITY FUNCTIONS
 * ============================================================================
 */
/**
 * Read locations from csv
 */
struct BridgeNodeRecord {
    uint32_t nodeId;
    std::string type;
    uint32_t typeId;
    double x, y, z;
    double txPowerDbm;
    double frequencyMHz;
    uint8_t sf;
    double bwKHz;
    double antennaGainDbi;
    double rxSensitivityDbm;
    double distanceM;
    double concreteMembersCrossed;
    double steelMembersCrossed;
    double rssiPredictedDbm;   
};

std::vector<BridgeNodeRecord> LoadNodeRecordsFromCsv(const std::string &path) {
    std::vector<BridgeNodeRecord> records;
    std::ifstream file(path);
    if (!file.is_open()) {
        NS_FATAL_ERROR("Cannot open node position CSV: " << path);
    }
    std::string line;
    std::getline(file, line); // skip header row
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        std::stringstream ss(line);
        std::string field;
        BridgeNodeRecord rec;
        std::getline(ss, field, ','); rec.nodeId          = std::stoul(field);
        std::getline(ss, field, ','); rec.type             = field;
        std::getline(ss, field, ','); rec.typeId           = std::stoul(field);
        std::getline(ss, field, ','); rec.x                = std::stod(field);
        std::getline(ss, field, ','); rec.y                = std::stod(field);
        std::getline(ss, field, ','); rec.z                = std::stod(field);
        std::getline(ss, field, ','); rec.txPowerDbm       = std::stod(field);
        std::getline(ss, field, ','); rec.frequencyMHz     = std::stod(field);
        std::getline(ss, field, ','); rec.sf               = static_cast<uint8_t>(std::stoul(field));
        std::getline(ss, field, ','); rec.bwKHz            = std::stod(field);
        std::getline(ss, field, ','); rec.antennaGainDbi   = std::stod(field);
        std::getline(ss, field, ','); rec.rxSensitivityDbm = std::stod(field);
        std::getline(ss, field, ','); rec.distanceM              = std::stod(field);
        std::getline(ss, field, ','); rec.concreteMembersCrossed = std::stod(field);
        std::getline(ss, field, ','); rec.steelMembersCrossed    = std::stod(field);
        std::getline(ss, field, ','); rec.rssiPredictedDbm       = std::stod(field); // "NaN" parses fine via strtod
        // Remaining columns (LinkMargin_dB, SF_ADR, TxPower_ADR_dBm,
        // LinkMargin_ADR_dB, MarginInsufficient) are intentionally NOT read —
        // ns-3's own ADR keeps assigning SF/TxPower,.
        records.push_back(rec);
    }
    NS_LOG_INFO("Loaded " << records.size() << " node records from " << path);
    return records;
}

uint8_t DataRateToSf(uint8_t dr) {
    return (dr <= 5) ? (12 - dr) : 7;
}
/**
 * Find end device furthest from any gateway (minimum distance metric)
 */
void FindFurthestDevice(uint32_t nEndDevices, uint32_t nGateways) {
    furthestDeviceIndex = 0;
    double maxMinDistance = 0.0;
    for (uint32_t i = 0; i < nEndDevices; ++i) {
        double minDistToAnyGw = std::numeric_limits<double>::max();
        for (uint32_t g = 0; g < nGateways; ++g) {
            if (distancesToGateways[g][i] < minDistToAnyGw) minDistToAnyGw = distancesToGateways[g][i];
        }
        if (minDistToAnyGw > maxMinDistance) {
            maxMinDistance = minDistToAnyGw;
            furthestDeviceIndex = i;
        }
    }
    NS_LOG_INFO("Furthest end device (max min-distance) is index " << furthestDeviceIndex << " at " << maxMinDistance << "m");
}

/**
 * Write SF assignments and distances to CSV file
 */
void WriteSfAssignmentsToCsv(const std::vector<Vector>& positions) {
    std::ofstream sfFile("node_sf_assignments.csv");
    sfFile << "node_id,x_m,y_m,z_m,assigned_sf,distance_to_gateway_m\n";
    for (uint32_t i = 0; i < std::min(spreadingFactors.size(), static_cast<size_t>(N_END_DEVICES)); ++i) {
        double dist = distancesToGateways[0][i];
        sfFile << i << "," << positions[i].x << "," << positions[i].y << ","
               << positions[i].z << "," << unsigned(spreadingFactors[i]) << ","
               << std::fixed << std::setprecision(2) << dist << "\n";
    }
    sfFile.close();
    NS_LOG_INFO("SF assignments saved to node_sf_assignments.csv");
}
/**
 * ============================================================================
 * PACKET TRACING CALLBACKS
 * ============================================================================
 */
void LogToFile(std::ofstream &f, const std::string &msg) {
    f << std::fixed << std::setprecision(3) << Simulator::Now().GetSeconds() << "s: " << msg << "\n";
}

void OnGatewayAck(uint32_t gwIndex, Ptr<const Packet> p) {
    g_ackCount[gwIndex]++;
    std::stringstream msg;
    msg << "Gateway " << gwIndex << " ACK sent";
    LogToFile(logFile, msg.str());
}

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
        }
    }
    LoraTag tag;
    uint8_t sf;
    if (packet->PeekPacketTag(tag)) {
        sf = tag.GetSpreadingFactor();
        if (sf < 7 || sf > 12) {
            sf = 7;
            tag.SetSpreadingFactor(sf);
        }
    } else {
        NS_LOG_ERROR("No LoraTag found for gateway " << gwIndex << ", forcing SF7");
        sf = 7;
        tag.SetSpreadingFactor(sf);
    }

}
void LogFurthestDevicePhyStartSending(uint32_t deviceIndex, Ptr<const Packet> packet, uint32_t phyIndex) {
    if (deviceIndex != furthestDeviceIndex) {
        return;
    }
    uint8_t sf = 7;
    LoraTag tag;
    if (packet->PeekPacketTag(tag)) {
        sf = tag.GetSpreadingFactor();
        if (sf < 7 || sf > 12) {
            NS_LOG_ERROR("Invalid SF " << unsigned(sf) << " for end device " << deviceIndex << ", using SF7");
            sf = 7;
        }
    } else {
        NS_LOG_ERROR("No LoraTag found for end device " << deviceIndex << " packet, using default SF7");
    }
}

void OnEndDeviceSentNewPacket(uint32_t deviceIndex, Ptr<EndDeviceLorawanMac> mac, Ptr<const Packet> packet) {
    LoraTag tag;
    if (!packet->PeekPacketTag(tag)) {
        NS_LOG_ERROR("No LoraTag found in SentNewPacket for end device " << deviceIndex);
    }
}
 

/**
 * ============================================================================
 * CUSTOM PACKET TAGGING
 * ============================================================================
 */
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


/**
 * ============================================================================
 * CUSTOM PERIODIC SENDER APPLICATION
 * ============================================================================
 */
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
        ScheduleNextTx(Seconds(0));
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
        uint8_t sf = DataRateToSf(dr);
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
    UniquePacketIdTag idTag;
    if (packet->PeekPacketTag(idTag)) {
        uint32_t id = idTag.GetId();
        std::stringstream msg;
        msg << "Packet " << id << " received at PHY index " << phyIndex;
        LogToFile(logFile, msg.str());
    }
    LoraTag tag;
    if (packet->PeekPacketTag(tag)) {
        int idx = tag.GetSpreadingFactor() - 7;
        if (idx >= 0 && idx < 6) {
            packetsReceived.at(idx)++;
        }
    }
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

void OnMacPacketOutcome(uint32_t deviceIndex, uint8_t transmissions, bool successful, Time firstAttempt, Ptr<Packet> packet) {
    uint8_t retransmissions = (transmissions > 0) ? (transmissions - 1) : 0;
    if (deviceIndex < g_retransmissionsPerNode.size()) {
        g_retransmissionsPerNode[deviceIndex] += retransmissions;
    }
    std::stringstream msg;
    msg << "Node " << deviceIndex << " packet outcome: " << unsigned(transmissions)
        << " transmission(s), " << unsigned(retransmissions) << " retransmission(s), "
        << (successful ? "SUCCESS" : "FAILED");
    LogToFile(logFile, msg.str());
}

/**
 * ============================================================================
 * ENERGY CSV LOGGING FUNCTIONS
 * ============================================================================
 */

/**
 * Save current energy state of all nodes to CSV (called periodically)
 */
void LogEnergyToCsv(double currentTime, const EnergySourceContainer& sources,
                     const std::vector<Vector>& positions) {
    static std::ofstream energyCsv("energy_consumption.csv");
    static bool headerWritten = false;
    if (!headerWritten) {
        energyCsv << "time_s,node_id,x_m,y_m,z_m,initial_energy_J,remaining_energy_J,consumed_energy_J,sf\n";
        headerWritten = true;
    }
    for (uint32_t i = 0; i < sources.GetN(); ++i) {
        Ptr<BasicEnergySource> src = DynamicCast<BasicEnergySource>(sources.Get(i));
        if (src) {
            double initialEnergy = src->GetInitialEnergy();
            double remainingEnergy = src->GetRemainingEnergy();
            energyCsv << std::fixed << std::setprecision(3)
                      << currentTime << "," << i << ","
                      << std::setprecision(4) << positions[i].x << "," << positions[i].y << "," << positions[i].z << ","
                      << std::setprecision(3) << initialEnergy << "," << remainingEnergy << ","
                      << (initialEnergy - remainingEnergy) << "," << unsigned(spreadingFactors[i]) << "\n";
        }
    }
    energyCsv.flush();
}

/**
 * Energy trace callback - logs changes for individual nodes
 */
void EnergyTraceCallback(uint32_t nodeId, double oldEnergy, double newEnergy) {
    if (nodeId < g_energyEventCount.size()) {
        g_energyEventCount[nodeId]++;
    }
    double currentTime = Simulator::Now().GetSeconds();
    std::stringstream msg;
    msg << "Node " << nodeId << " energy: " << oldEnergy << "J -> " << newEnergy << "J at " << currentTime << "s";
    LogToFile(logFile, msg.str());
}

/**
 * ============================================================================
 * MAIN SIMULATION FUNCTION
 * ============================================================================
 */
int main(int argc, char *argv[]) {
    // Enable logging
    LogComponentEnable("enddeviceB", LOG_LEVEL_INFO);
    NS_LOG_INFO("=== Starting LoRaWAN simulation with MATLAB bridge scenario coordinates and pathloss ===");

    // Open simulation log
    logFile.open("enddeviceB_log.txt");
    if (!logFile.is_open()) {
        NS_FATAL_ERROR("Cannot open enddevice_log.txt");
    }

    // 1. CHANNEL SETUP
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
    NS_LOG_INFO("✓ Channel setup complete");

    /**********************
     * 1. LOAD NODE POSITIONS FROM CSV & SETUP POSITION ALLOCATOR
     **********************/
    std::string csvPath = "/user/edreckme/home/Downloads/Matlab/Bridge-Simulation-main/bridge_sensor_gateway_positions.csv";
    CommandLine cmd;
    cmd.AddValue("csv", "Path to node position CSV", csvPath);
    cmd.Parse(argc, argv);

    std::vector<BridgeNodeRecord> records = LoadNodeRecordsFromCsv(csvPath);
    for (const auto &rec : records) {
        Vector pos(rec.x, rec.y, rec.z);
        if (rec.type == "Gateway") {
            g_gatewayPositions.push_back(pos);
        } else {
            g_endDevicePositions.push_back(pos);
        }
    }
    N_END_DEVICES = g_endDevicePositions.size();
    N_GATEWAYS = g_gatewayPositions.size();
    if (N_END_DEVICES == 0 || N_GATEWAYS == 0) {
        NS_FATAL_ERROR("CSV must contain at least one end device row and one Gateway row");
    }
    g_nsPosition = g_gatewayPositions.front(); // NS co-located with first gateway

    distancesToGateways.resize(N_GATEWAYS);

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();

    // Place end devices using CSV positions
    for (uint32_t i = 0; i < N_END_DEVICES; ++i) {
        const Vector &pos = g_endDevicePositions[i];
        allocator->Add(pos);
        NS_LOG_INFO("Placed end device " << i << " at x=" << std::fixed << std::setprecision(4)
                  << pos.x << ", y=" << pos.y << ", z=" << pos.z);
    }

    // Place gateways
    for (uint32_t i = 0; i < N_GATEWAYS; i++) {
        allocator->Add(g_gatewayPositions[i]);
        NS_LOG_INFO("Placed gateway " << i << " at " << g_gatewayPositions[i]);
    }

    allocator->Add(g_nsPosition);
    NS_LOG_INFO("Placed network server at " << g_nsPosition);

    mobility.SetPositionAllocator(allocator);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    /**********************
     * 2. CREATE NODES
     **********************/
    NodeContainer endDevices;
    endDevices.Create(N_END_DEVICES);
    NodeContainer gateways;
    gateways.Create(N_GATEWAYS);
    Ptr<Node> networkServer = CreateObject<Node>();

    /**********************
     * 3. INSTALL MOBILITY - POSITIONS NOW ASSIGNED!
     **********************/
    mobility.Install(endDevices);
    mobility.Install(gateways);
    mobility.Install(networkServer);
    NS_LOG_INFO("Mobility installed - positions assigned!");

    /**********************
     * 4. COMPUTE DISTANCES
     **********************/
    NS_LOG_INFO("Computing distances to gateways...");
    for (uint32_t g = 0; g < N_GATEWAYS; ++g) {
        Ptr<MobilityModel> gwMob = gateways.Get(g)->GetObject<MobilityModel>();
        Vector gwPos = gwMob->GetPosition();
        distancesToGateways[g].resize(N_END_DEVICES);
        for (uint32_t i = 0; i < N_END_DEVICES; ++i) {
            Ptr<MobilityModel> devMob = endDevices.Get(i)->GetObject<MobilityModel>();
            Vector devPos = devMob->GetPosition();
            distancesToGateways[g][i] = ns3::CalculateDistance(devPos, gwPos);
            NS_LOG_INFO("Node " << i << " (" << std::fixed << std::setprecision(2)
                      << devPos.x << "," << devPos.y << "," << devPos.z
                      << ") → GW" << g << " distance: "
                      << std::setprecision(1) << distancesToGateways[g][i] << "m");
        }
    }
    NS_LOG_INFO("Distances to gateways computed.");

    FindFurthestDevice(N_END_DEVICES, N_GATEWAYS);
    packetsReceivedPerNode.resize(endDevices.GetN(), 0);
    g_retransmissionsPerNode.resize(endDevices.GetN(), 0); 
    g_energyEventCount.resize(endDevices.GetN(), 0); 

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

    // Connect gateway traces
    for (uint32_t i = 0; i < gateways.GetN(); i++) {
        Ptr<GatewayLorawanMac> gwMac = gatewaysNet.Get(i)
                                             ->GetObject<LoraNetDevice>()
                                             ->GetMac()
                                             ->GetObject<GatewayLorawanMac>();
        gwMac->TraceConnectWithoutContext("SentAck", MakeBoundCallback(&OnGatewayAck, i));
        
        Ptr<GatewayLoraPhy> gwPhy = gatewaysNet.Get(i)
                                             ->GetObject<LoraNetDevice>()
                                             ->GetPhy()
                                             ->GetObject<GatewayLoraPhy>();
        gwPhy->TraceConnectWithoutContext("StartSending", MakeBoundCallback(&OnGatewayPhyStartSending, i));
    }

    // Connect end device traces
    for (uint32_t i = 0; i < endDevicesNet.GetN(); ++i) {
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(endDevicesNet.Get(i));
        Ptr<EndDeviceLorawanMac> mac = DynamicCast<EndDeviceLorawanMac>(loraNetDevice->GetMac());
        if (USE_CONFIRMED_UPLINK) {
            mac->SetMType(LorawanMacHeader::CONFIRMED_DATA_UP);
        } else {
            mac->SetMType(LorawanMacHeader::UNCONFIRMED_DATA_UP);
        }
        mac->TraceConnectWithoutContext("RequiredTransmissions", MakeBoundCallback(&OnMacPacketOutcome, i));
        mac->TraceConnectWithoutContext("SentNewPacket", MakeBoundCallback(&OnEndDeviceSentNewPacket, i, mac));
    }
    NS_LOG_INFO("Devices setup complete.");

    /**********************
     * Backbone Network Setup
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
    NS_LOG_INFO("Server setup complete.");

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
    for (uint32_t i = 0; i < endDevices.GetN(); ++i) {
        Ptr<TaggingPeriodicSender> app = CreateObject<TaggingPeriodicSender>();
        app->Setup(endDevices.Get(i), endDevicesNet.Get(i), PERIOD_SENDER, 24);
        endDevices.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(randStart->GetValue()));
        app->SetStopTime(Hours(SIM_END_HOURS));
        apps.Add(app);
    }

    if (ENABLE_12TH_HOUR_POLLING) {
        Simulator::Schedule(Seconds(39600.0), [&apps]() {
            for (uint32_t i = 0; i < apps.GetN(); ++i) {
                Ptr<TaggingPeriodicSender> sender = DynamicCast<TaggingPeriodicSender>(apps.Get(i));
                if (sender) sender->SetPeriod(Seconds(90));
            }
        });
        Simulator::Schedule(Seconds(43200.0), [&apps]() {
            for (uint32_t i = 0; i < apps.GetN(); ++i) {
                Ptr<TaggingPeriodicSender> sender = DynamicCast<TaggingPeriodicSender>(apps.Get(i));
                if (sender) sender->SetPeriod(PERIOD_SENDER);
            }
        });
    }
    NS_LOG_INFO("Applications created.");

    /**********************
     * Energy Setup
     **********************/
    NS_LOG_INFO("Setting up energy model...");
    BasicEnergySourceHelper basicSourceHelper;
    basicSourceHelper.Set("BasicEnergySourceInitialEnergyJ", DoubleValue(10000.0));
    basicSourceHelper.Set("BasicEnergySupplyVoltageV", DoubleValue(3.3));

    LoraRadioEnergyModelHelper radioEnergyHelper;
    radioEnergyHelper.Set("StandbyCurrentA", DoubleValue(0.0004));
    radioEnergyHelper.Set("TxCurrentA", DoubleValue(0.120));
    radioEnergyHelper.Set("RxCurrentA", DoubleValue(0.011));
    radioEnergyHelper.Set("SleepCurrentA", DoubleValue(0.0000015));
    radioEnergyHelper.SetTxCurrentModel("ns3::ConstantLoraTxCurrentModel", "TxCurrent", DoubleValue(0.090));

    EnergySourceContainer sources = basicSourceHelper.Install(endDevices);
    DeviceEnergyModelContainer deviceModels = radioEnergyHelper.Install(endDevicesNet, sources);
    NS_LOG_INFO("Energy model installed.");

    // Connect energy traces for detailed logging
    for (uint32_t i = 0; i < sources.GetN(); ++i) {
        Ptr<BasicEnergySource> src = DynamicCast<BasicEnergySource>(sources.Get(i));
        if (src) {
            src->TraceConnectWithoutContext("RemainingEnergy", 
                            MakeBoundCallback(&EnergyTraceCallback, i));
        }
    }
    NS_LOG_INFO("Energy traces connected.");


    /**********************
     * Spreading Factors - AUTOMATIC ns-3 ASSIGNMENT
     **********************/
    NS_LOG_INFO("Setting spreading factors automatically using ns-3 ADR...");
    LorawanMacHelper::SetSpreadingFactorsUp(endDevices, gateways, channel);
    
    // Read ACTUAL assigned SFs
    spreadingFactors.resize(N_END_DEVICES, 7);
    for (uint32_t i = 0; i < N_END_DEVICES; ++i) {
        Ptr<Node> node = endDevices.Get(i);
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(node->GetDevice(0));
        Ptr<EndDeviceLorawanMac> mac = DynamicCast<EndDeviceLorawanMac>(loraNetDevice->GetMac());
        
        uint8_t dr = mac->GetDataRate();
        uint8_t sf = DataRateToSf(dr);
        spreadingFactors[i] = sf;
        
        double dist = distancesToGateways[0][i];
        NS_LOG_INFO("Node " << i << " AUTO-ASSIGNED SF" << unsigned(sf) << " (DR" << unsigned(dr) 
          << ") pos(" << std::fixed << std::setprecision(2) << g_endDevicePositions[i].x 
          << "," << g_endDevicePositions[i].y << "," << g_endDevicePositions[i].z << ") Dist:" 
          << std::setprecision(1) << dist << "m");

    }
    
    WriteSfAssignmentsToCsv(g_endDevicePositions);
    NS_LOG_INFO("Automatic SF assignment complete.");

    /**********************
     * Connect PHY Traces
     **********************/
    for (uint32_t i = 0; i < endDevices.GetN(); ++i) {
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(endDevices.Get(i)->GetDevice(0));
        loraNetDevice->GetPhy()->TraceConnectWithoutContext("StartSending", MakeBoundCallback(&OnTransmissionCallback, i));
        loraNetDevice->GetPhy()->TraceConnectWithoutContext("StartSending", MakeBoundCallback(&LogFurthestDevicePhyStartSending, i));
    }
    for (uint32_t i = 0; i < gateways.GetN(); ++i) {
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(gateways.Get(i)->GetDevice(0));
        loraNetDevice->GetPhy()->TraceConnectWithoutContext("ReceivedPacket", MakeCallback(&OnPacketReceptionCallback));
    }

    /**********************
     * NetAnim Setup with SF coloring
     **********************/
    AnimationInterface anim("CT_dev.xml");
    for (uint32_t i = 0; i < endDevices.GetN(); ++i) {
        std::string label = "ED" + std::to_string(i) + "_SF" + std::to_string(unsigned(spreadingFactors[i]));
        anim.UpdateNodeDescription(endDevices.Get(i), label);
        
        // Color by SF
        uint8_t sf = spreadingFactors[i];
        if (sf == 7) anim.UpdateNodeColor(endDevices.Get(i), 0, 255, 0);       // Green
        else if (sf == 8) anim.UpdateNodeColor(endDevices.Get(i), 0, 255, 255); // Cyan
        else if (sf == 9) anim.UpdateNodeColor(endDevices.Get(i), 0, 128, 255);  // Blue
        else if (sf == 10) anim.UpdateNodeColor(endDevices.Get(i), 255, 0, 255); // Magenta
        else if (sf == 11) anim.UpdateNodeColor(endDevices.Get(i), 255, 128, 0); // Orange
        else anim.UpdateNodeColor(endDevices.Get(i), 255, 0, 0);                // Red
        anim.UpdateNodeSize(i, 12.0, 12.0);
    }
    for (uint32_t g = 0; g < gateways.GetN(); ++g) {
        anim.UpdateNodeDescription(gateways.Get(g), "GW" + std::to_string(g));
        anim.UpdateNodeColor(gateways.Get(g), 255, 0, 0);
        anim.UpdateNodeSize(N_END_DEVICES + g, 20.0, 20.0);
    }
    anim.UpdateNodeDescription(networkServer, "NS");
    anim.UpdateNodeColor(networkServer, 0, 0, 255);
    anim.EnablePacketMetadata(true);

    Time logInterval = Hours(1);
    for (uint32_t hour = 1; hour <= SIM_END_HOURS; ++hour) {
        Simulator::Schedule(Hours(hour), &LogEnergyToCsv, Hours(hour).GetSeconds(), sources, g_endDevicePositions);
    }

    // 15. RUN SIMULATION
    Simulator::Stop(Hours(SIM_END_HOURS));
    Simulator::Run();

    // 16. FINAL STATISTICS
    NS_LOG_INFO("===============================================");

    NS_LOG_INFO("=== SIMULATION COMPLETE ===");

    NS_LOG_INFO("===============================================");
    
   NS_LOG_INFO("Packets sent vs received per DR (SF7 -> SF12):");
    for (int i = 0; i < 6; i++) {
        std::cout << "DR" << (5 - i) << " (SF" << (7 + i) << "): Sent = "
                  << packetsSent.at(i) << ", Received = " << packetsReceived.at(i) << std::endl;
    }
    NS_LOG_INFO("===============================================");
    NS_LOG_INFO("Successful transmission to Gateway per end device:");
    for (uint32_t i = 0; i < packetsReceivedPerNode.size(); ++i) {
        std::cout << "Node " << i << " (SF" << unsigned(spreadingFactors[i]) << "): "
                  << packetsReceivedPerNode[i] << " packets received successfully by GW." << std::endl;
    }
    std::cout << "============== RETRANSMISSION SUMMARY ==============\n";
    for (uint32_t i = 0; i < g_retransmissionsPerNode.size(); ++i) {
        std::cout << "Node " << i << " (SF" << unsigned(spreadingFactors[i]) << "): "
                << g_retransmissionsPerNode[i] << " retransmission(s)\n";
    }
    std::cout << "============== ENERGY EVENT COUNT ==============\n";
    for (uint32_t i = 0; i < g_energyEventCount.size(); ++i) {
        std::cout << "Node " << i << ": " << g_energyEventCount[i] << " energy state change(s)\n";
    }
    std::cout << "================= ACK SUMMARY =================\n";
    for (uint32_t g = 0; g < g_ackCount.size(); ++g) {
        std::cout << "Gateway " << g << " sent " << g_ackCount[g] << " ACKs\n";
    }
    std::cout << "==============================================\n";

    // Energy logging
    NS_LOG_INFO("Logging energy consumption...");
    double simDuration = Simulator::Now().GetSeconds();
    NS_LOG_INFO("Total simulation duration: " << simDuration << " seconds");
    std::cout << "\n================= ENERGY SUMMARY =================\n";
    std::cout << "NodeID | Initial(J) | Remaining(J) | Consumed(J)\n";
    std::cout << "--------------------------------------------------\n";
    for (uint32_t i = 0; i < sources.GetN(); ++i) {
        Ptr<BasicEnergySource> src = sources.Get(i)->GetObject<BasicEnergySource>();
        double initialEnergy = src->GetInitialEnergy();
        double remainingEnergy = src->GetRemainingEnergy();
        double consumed = initialEnergy - remainingEnergy;
        std::cout << "Node " << i
                  << " | " << initialEnergy
                  << " | " << remainingEnergy
                  << " | " << consumed
                  << std::endl;
    }
    std::cout << "==================================================\n\n";

    logFile.close();
    Simulator::Destroy();
    return 0;
}