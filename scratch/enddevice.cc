
//==============================================================================
// LoRaWAN Network Simulation with ENU Coordinates and ADR
// Simulates 30 end devices with automatic spreading factor assignment (ADR)
// Features: Packet tracking, energy consumption, NetAnim visualization
//==============================================================================

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

NS_LOG_COMPONENT_DEFINE("enddevice");

/**
 * ============================================================================
 * SIMULATION PARAMETERS
 * ============================================================================
 */
static const uint32_t SIM_END_HOURS = 24;           // Simulation duration (hours)
static const uint32_t N_END_DEVICES = 100;           // Number of end devices
static const uint32_t N_GATEWAYS = 3;               // Number of gateways
static const Time PERIOD_SENDER = Minutes(15);      // Packet transmission period
static std::ofstream logFile;


// ENU coordinates for end devices (x, y, z in meters)
static const std::vector<std::vector<double>> ENU_DATA = {
    {0, 0, 20}, // node 0
    {-18.0194755079705, -18.9031375300491, 21}, // node 1
    {-35.8069406017416, -37.8062750593909, 22}, // node 2
    {-53.5944056954512, -56.7094125887328, 23}, // node 3
    {-71.3818707891607, -75.6125501187819, 24}, // node 4
    {-89.1693358828703, -94.5156876481237, 25}, // node 5
    {-106.956800976641, -113.418825178173, 27}, // node 6
    {-124.744266070351, -132.321962707515, 28}, // node 7
    {-142.531731164122, -151.225100236856, 29}, // node 8
    {-160.31919625777, -170.128237766906, 31}, // node 9
    {-178.106661351541, -189.031375296247, 33}, // node 10
    {-194.347390350116, -207.934512826297, 35}, // node 11
    {-208.268015206143, -226.837650354931, 36}, // node 12
    {-219.868535919376, -245.740787884273, 38}, // node 13
    {-227.602216394926, -264.643925414322, 40}, // node 14
    {-229.922320537597, -283.547062943664, 42}, // node 15
    {-227.602216394926, -302.450200473713, 44}, // node 16
    {-222.188640062047, -321.353338003055, 46}, // node 17
    {-216.001695681631, -340.256475532396, 48}, // node 18
    {-209.814751301216, -359.159613062446, 50}, // node 19
    {-204.401174968337, -378.062750591787, 52}, // node 20
    {-200.534334730593, -396.965888121837, 53}, // node 21
    {-198.214230587922, -415.869025651178, 55}, // node 22
    {-196.667494492787, -434.772163181228, 56}, // node 23
    {-195.120758397714, -453.675300710569, 58}, // node 24
    {-194.347390350116, -472.578438239204, 59}, // node 25
    {-193.57402230258, -491.481575769253, 60}, // node 26
    {-192.800654255043, -510.384713298595, 61}, // node 27
    {-195.894126445251, -529.287850828644, 62}, // node 28
    {-200.534334730593, -548.190988357986, 63}, // node 29
    {-208.268015206143, -567.094125887327, 64}, // node 30
    {-216.001695681631, -585.997263417377, 64}, // node 31
    {-223.73537615712, -604.900400946718, 65}, // node 32
    {-231.469056632731, -623.803538476768, 65}, // node 33
    {-243.069577346025, -642.706676006109, 65}, // node 34
    {-254.670098059258, -661.609813535451, 65}, // node 35
    {-266.270618772614, -680.5129510655, 66}, // node 36
    {-277.871139485908, -699.416088594842, 66}, // node 37
    {-289.471660199202, -718.319226124891, 66}, // node 38
    {-301.072180912496, -737.222363653526, 66}, // node 39
    {-310.35259748312, -756.125501182867, 66}, // node 40
    {-319.633014053743, -776.140587978808, 66}, // node 41
    {-328.913430624427, -796.155674775455, 66}, // node 42
    {-338.19384719505, -816.170761571395, 66}, // node 43
    {-347.474263765735, -836.185848367335, 66}, // node 44
    {-356.754680336358, -856.200935163275, 66}, // node 45
    {-366.035096906981, -876.216021959216, 66}, // node 46
    {-376.862249572738, -896.231108755156, 66}, // node 47
    {-387.689402238496, -916.246195551803, 66}, // node 48
    {-398.516554904254, -936.261282347743, 65}, // node 49
    {-409.343707570011, -956.276369143683, 65}, // node 50
    {-420.170860235769, -976.291455939623, 65}, // node 51
    {-430.998012901465, -996.306542735563, 64}, // node 52
    {-441.825165567223, -1016.3216295315, 64}, // node 53
    {-452.65231823298, -1036.33671632815, 63}, // node 54
    {-463.479470898677, -1056.35180312409, 63}, // node 55
    {-474.306623564434, -1076.36688992003, 62}, // node 56
    {-485.133776230192, -1096.38197671597, 62}, // node 57
    {-495.960928895888, -1116.39706351191, 61}, // node 58
    {-506.788081561646, -1136.41215030785, 60}, // node 59
    {-517.615234227403, -1156.42723710379, 59}, // node 60
    {-528.442386893161, -1176.44232390044, 58}, // node 61
    {-539.269539558919, -1196.45741069638, 57}, // node 62
    {-550.096692224676, -1216.47249749161, 56}, // node 63
    {-560.923844890434, -1236.48758428755, 55}, // node 64
    {-571.75099755613, -1256.50267108349, 54}, // node 65
    {-582.578150221888, -1276.51775787943, 52}, // node 66
    {-593.405302887646, -1296.53284467537, 51}, // node 67
    {-604.232455553403, -1316.54793147202, 50}, // node 68
    {-615.059608219161, -1336.56301826796, 48}, // node 69
    {-625.886760884918, -1356.5781050639, 47}, // node 70
    {-635.167177455541, -1373.25734406005, 45}, // node 71
    {-644.447594026164, -1389.93658305761, 44}, // node 72
    {-653.728010596788, -1406.61582205375, 42}, // node 73
    {-663.008427167411, -1423.2950610506, 40}, // node 74
    {-672.288843738157, -1439.97430004746, 38}, // node 75
    {-681.56926030878, -1454.42964051111, 36}, // node 76
    {-690.849676879403, -1467.77303170888, 34}, // node 77
    {-698.583357354891, -1478.89252437274, 32}, // node 78
    {-706.317037830441, -1490.01201703801, 30}, // node 79
    {-712.503982210918, -1498.90761116938, 29}, // node 80
    {-718.690926591272, -1506.69125603416, 28}, // node 81
    {-724.877870971749, -1514.47490089964, 27}, // node 82
    {-729.51807925703, -1520.03464723192, 26}, // node 83
    {-732.611551447299, -1524.48244429761, 25}, // node 84
    {-735.705023637506, -1527.81829209599, 24}, // node 85
    {-738.025127780116, -1530.04219062919, 23}, // node 86
    {-740.345231922787, -1532.26608916309, 22}, // node 87
    {-742.665336065397, -1533.37803842898, 22}, // node 88
    {-744.98544020813, -1534.48998769558, 21}, // node 89
    {-747.305544350801, -1535.60193696147, 21}, // node 90
    {-749.62564849341, -1536.71388622807, 21}, // node 91
    {-751.172384588545, -1537.26986086207, 20}, // node 92
    {-752.719120683679, -1537.82583549467, 20}, // node 93
    {-754.265856778753, -1538.38181012797, 20}, // node 94
    {-755.812592873887, -1538.93778476126, 20}, // node 95
    {-756.585960921424, -1534.48998769558, 20}, // node 96
    {-756.972644945192, -1525.5943935635, 20}, // node 97
    {-756.972644945192, -1518.92269796532, 20}, // node 98
    {-756.972644945192, -1512.25100236644, 20} // node 99
};


// Gateway and Network Server positions
static std::vector<Vector> GATEWAY_POSITIONS = { Vector(-365.179895736156,-865.128761708643,	67.5), Vector(-694.451816578074,-1453.93269096186,67.5), Vector(-161.493872527076,-259.432813793174,67.5)};
//static std::vector<Vector> GATEWAY_POSITIONS = { Vector(-636.601825208211,-1382.99029010993,67.5), Vector(-194.668915371024,-418.158900834699,67.5)};
static std::vector<Vector> NS_POSITION = {Vector(-151.145782048547,-210.085425844995,67.5)};

// Configuration flags
static const bool USE_CONFIRMED_UPLINK = true;
static const bool ENABLE_12TH_HOUR_POLLING = false; 

/**
 * ============================================================================
 * GLOBAL STATE VARIABLES
 * ============================================================================
 */
std::vector<std::vector<double>> distancesToGateways(N_GATEWAYS);
std::vector<uint32_t> g_ackCount;
std::unordered_set<uint32_t> receivedPacketIds;
std::vector<int> packetsSent(6, 0);           // SF7-SF12
std::vector<int> packetsReceived(6, 0);       // SF7-SF12
std::map<uint32_t, uint32_t> packetSenderMap;
std::vector<int> packetsReceivedPerNode;
std::vector<uint8_t> spreadingFactors;        // Actual assigned SFs
uint32_t furthestDeviceIndex = 0;

/**
 * ============================================================================
 * UTILITY FUNCTIONS
 * ============================================================================
 */

/** 
 * Calculate LoRa Time-on-Air for a packet
 * @param payloadSize Bytes in payload
 * @param sf Spreading Factor (7-12)
 * @return Time-on-air in seconds
 */
double CalculateTimeOnAir(uint32_t payloadSize, uint8_t sf, double bandwidthHz = 125000.0, 
                         uint8_t codingRate = 1, bool crcEnabled = true, bool headerEnabled = true, 
                         uint8_t nPreamble = 8) {
    if (sf < 7 || sf > 12) {
        NS_LOG_ERROR("Invalid SF " << unsigned(sf) << ", using SF7");
        sf = 7;
    }
    
    double Ts = (1 << sf) / bandwidthHz;
    double Tpreamble = (nPreamble + 4.25) * Ts;
    int DE = (sf >= 11) ? 1 : 0;
    int H = headerEnabled ? 0 : 1;
    int CR = codingRate;
    
    double payloadSymbNb = 8 + std::max(std::ceil((8.0 * payloadSize - 4.0 * sf + 28 + 
                                                  16 * crcEnabled - 20 * H) / 
                                                 (4.0 * (sf - 2 * DE))) * (CR + 4), 0.0);
    double Tpayload = payloadSymbNb * Ts;
    return Tpreamble + Tpayload;
}

/**
 * Find end device furthest from any gateway (minimum distance metric)
 */
void FindFurthestDevice(NodeContainer endDevices, NodeContainer gateways) {
    furthestDeviceIndex = 0;
    double maxMinDistance = 0.0;
    for (uint32_t i = 0; i < endDevices.GetN(); ++i) {
        Ptr<Node> device = endDevices.Get(i);
        Ptr<MobilityModel> deviceMobility = device->GetObject<MobilityModel>();
        Vector devPos = deviceMobility->GetPosition();
        double minDistToAnyGw = std::numeric_limits<double>::max();
        for (uint32_t g = 0; g < gateways.GetN(); ++g) {
            Ptr<MobilityModel> gwMob = gateways.Get(g)->GetObject<MobilityModel>();
            Vector gwPos = gwMob->GetPosition();
            double dist = ns3::CalculateDistance(devPos, gwPos);  // ✅ FIXED
            if (dist < minDistToAnyGw) minDistToAnyGw = dist;
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
void WriteSfAssignmentsToCsv(const std::vector<std::vector<double>>& enuData) {
    std::ofstream sfFile("node_sf_assignments.csv");
    sfFile << "node_id,x_enu_m,y_enu_m,z_enu_m,assigned_sf,distance_to_gateway_m\n";
    
    for (uint32_t i = 0; i < std::min(spreadingFactors.size(), static_cast<size_t>(N_END_DEVICES)); ++i) {
        double dist = distancesToGateways[0][i];
        sfFile << i << "," << enuData[i][0] << "," << enuData[i][1] << "," 
               << enuData[i][2] << "," << unsigned(spreadingFactors[i]) << "," 
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

}
void OnEndDevicePhyStartSending(uint32_t deviceIndex, Ptr<const Packet> packet, uint8_t dr) {
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
        sf = (dr <= 5) ? (12 - dr) : 7;
    }
}

void OnEndDeviceSentNewPacket(uint32_t deviceIndex, Ptr<EndDeviceLorawanMac> mac, Ptr<const Packet> packet) {
    uint8_t dr = mac->GetDataRate();
    LoraTag tag;
    if (!packet->PeekPacketTag(tag)) {
        NS_LOG_ERROR("No LoraTag found in SentNewPacket for end device " << deviceIndex);
    }
    if (deviceIndex == furthestDeviceIndex) {
        OnEndDevicePhyStartSending(deviceIndex, packet, dr);
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

void OnMacPacketOutcome(uint8_t transmissions, bool successful, Time firstAttempt, Ptr<Packet> packet) {
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
                   const std::vector<std::vector<double>>& enuData) {
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
            double consumedEnergy = initialEnergy - remainingEnergy;
            
            energyCsv << std::fixed << std::setprecision(3)
                      << currentTime << "," << i << "," 
                      << std::setprecision(4) << enuData[i][0] << "," << enuData[i][1] << "," << enuData[i][2] << ","
                      << std::setprecision(3) << initialEnergy << "," << remainingEnergy << "," 
                      << consumedEnergy << "," << unsigned(spreadingFactors[i]) << "\n";
        }
    }
    energyCsv.flush();  // Ensure data is written immediately
}

/**
 * Energy trace callback - logs changes for individual nodes
 */
void EnergyTraceCallback(double oldEnergy, double newEnergy, uint32_t nodeId) {
    double currentTime = Simulator::Now().GetSeconds();
    std::stringstream msg;
    msg << "Node " << nodeId << " energy: " << oldEnergy << "J → " << newEnergy << "J at " << currentTime << "s";
    LogToFile(logFile, msg.str());
}

/**
 * ============================================================================
 * MAIN SIMULATION FUNCTION
 * ============================================================================
 */
int main(int argc, char *argv[]) {
    // Enable logging
    LogComponentEnable("enddevice", LOG_LEVEL_INFO);
    NS_LOG_INFO("=== Starting LoRaWAN simulation with ENU coordinates ===");

    // Open simulation log
    logFile.open("enddevice_log.txt");
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
     * 1. SETUP POSITION ALLOCATOR with ENU coordinates
     **********************/
    if (GATEWAY_POSITIONS.size() < N_GATEWAYS) {
        NS_FATAL_ERROR("Not enough gateway positions defined for N_GATEWAYS");
    }
    if (ENU_DATA.size() < N_END_DEVICES) {
        NS_FATAL_ERROR("Not enough ENU coordinates for N_END_DEVICES. Have " << ENU_DATA.size() << ", need " << N_END_DEVICES);
    }

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();

    // Place end devices using ENU coordinates
    for (uint32_t i = 0; i < N_END_DEVICES; ++i) {
        Vector pos(ENU_DATA[i][0], ENU_DATA[i][1], ENU_DATA[i][2]);
        allocator->Add(pos);
        NS_LOG_INFO("Placed end device " << i << " at ENU: x=" << std::fixed << std::setprecision(4)
                  << pos.x << ", y=" << pos.y << ", z=" << pos.z);
    }
    
    // Place gateways
    for (uint32_t i = 0; i < N_GATEWAYS; i++) {
        allocator->Add(GATEWAY_POSITIONS[i]);
        NS_LOG_INFO("Placed gateway " << i << " at " << GATEWAY_POSITIONS[i]);
    }
    
    allocator->Add(NS_POSITION[0]);
    NS_LOG_INFO("Placed network server at " << NS_POSITION[0]);

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
     * 4. NOW COMPUTE CORRECT DISTANCES
     **********************/
    NS_LOG_INFO("Computing CORRECT distances to gateway...");
    for (uint32_t g = 0; g < N_GATEWAYS; ++g) {
        Ptr<MobilityModel> gwMob = gateways.Get(g)->GetObject<MobilityModel>();
        Vector gwPos = gwMob->GetPosition();
        distancesToGateways[g].resize(N_END_DEVICES);
        
        for (uint32_t i = 0; i < N_END_DEVICES; ++i) {
            Ptr<MobilityModel> devMob = endDevices.Get(i)->GetObject<MobilityModel>();
            Vector devPos = devMob->GetPosition();
            distancesToGateways[g][i] = ns3::CalculateDistance(devPos, gwPos);
            
            // Verify ENU coordinates match
            NS_LOG_INFO("Node " << i << " ENU(" << std::fixed << std::setprecision(2)
                      << devPos.x << "," << devPos.y << "," << devPos.z 
                      << ") → GW" << g << " distance: " 
                      << std::setprecision(1) << distancesToGateways[g][i] << "m");
        }
    }
    NS_LOG_INFO("Distances to gateway computed CORRECTLY.");

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
        mac->TraceConnectWithoutContext("RequiredTransmissions", MakeCallback(&OnMacPacketOutcome));
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
            src->TraceConnectWithoutContext("RemainingEnergyChange", 
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
        uint8_t sf = (dr <= 5) ? (12 - dr) : 7;
        spreadingFactors[i] = sf;
        
        double dist = distancesToGateways[0][i];
        NS_LOG_INFO("Node " << i << " AUTO-ASSIGNED SF" << unsigned(sf) << " (DR" << unsigned(dr) 
                  << ") ENU(" << std::fixed << std::setprecision(2) << ENU_DATA[i][0] 
                  << "," << ENU_DATA[i][1] << "," << ENU_DATA[i][2] << ") Dist:" 
                  << std::setprecision(1) << dist << "m");
    }
    
    WriteSfAssignmentsToCsv(ENU_DATA);
    NS_LOG_INFO("Automatic SF assignment complete.");

    /**********************
     * Connect PHY Traces
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
        Simulator::Schedule(Hours(hour), &LogEnergyToCsv, Hours(hour).GetSeconds(), sources, ENU_DATA);
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