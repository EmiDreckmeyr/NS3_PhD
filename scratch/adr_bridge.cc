#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/lorawan-module.h"
#include "ns3/lora-helper.h"
#include "ns3/log.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/application.h"
#include <iostream>

using namespace ns3;
using namespace ns3::lorawan;

NS_LOG_COMPONENT_DEFINE("BridgeExperimental");

/**********************
 * Global variables
 **********************/
static std::vector<uint32_t> g_ackCount;

// ---- RX window callbacks ----
void OnRxWindowStart(uint32_t nodeId) {
    std::cout << "Node " << nodeId << " RX window opened at "
              << Simulator::Now().GetSeconds() << "s" << std::endl;
}
void OnRxWindowEnd(uint32_t nodeId) {
    std::cout << "Node " << nodeId << " RX window closed at "
              << Simulator::Now().GetSeconds() << "s" << std::endl;
}

// ---- Trace callback for EndRx ----
void RxWindowEndCallback(unsigned int nodeId) {
    OnRxWindowEnd(nodeId);
}
void StartRxCallback(unsigned int nodeId) {
    OnRxWindowStart(nodeId);
}

// ---- ACK reception callback ----
void EdPacketReceived(ns3::Ptr<const ns3::Packet> packet, unsigned int nodeId)
{
    LorawanMacHeader macHdr;
    Ptr<Packet> copy = packet->Copy();
    if (copy->PeekHeader(macHdr)) {
        if (macHdr.GetMType() == LorawanMacHeader::CONFIRMED_DATA_DOWN) {
            std::cout << "Node " << nodeId << " received ACK at "
                      << Simulator::Now().GetSeconds() << "s" << std::endl;
        }
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
// Declare global unique packet ID counter at top-level (before any class)
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
        
        private:
            void ScheduleNextTx(Time delay) {
                m_sendEvent = Simulator::Schedule(delay, &TaggingPeriodicSender::SendPacket, this);
            }
        
            void SendPacket() {
                // Create packet
                Ptr<Packet> packet = Create<Packet>(m_packetSize);
                UniquePacketIdTag idTag(++globalPacketId);
                packet->AddPacketTag(idTag);
            
                // Construct MAC header
                LorawanMacHeader macHdr;
                macHdr.SetMType(LorawanMacHeader::CONFIRMED_DATA_UP);
            
                // Add MAC header to packet
                packet->AddHeader(macHdr);
            
                Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(m_device);
                if (!loraNetDevice) {
                    NS_LOG_ERROR("Device is not a LoraNetDevice");
                    return;
                }
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
 * PACKET TRACKING
 ***************/
std::vector<int> packetsSent(6, 0);     // DR5 -> DR0
std::vector<int> packetsReceived(6, 0);
std::map<uint32_t, uint32_t> packetSenderMap;  // unique packet id -> sender node id
 std::vector<int> packetsReceivedPerNode;
    
/***************
* Callbacks for tracing packets at PHY layer
***************/
void OnTransmissionCallback(Ptr<const Packet> packet, uint32_t senderNodeId) {
    LoraTag tag;
    if (packet->PeekPacketTag(tag)) {
        packetsSent.at(tag.GetSpreadingFactor() - 7)++;
    }
    UniquePacketIdTag idTag;
    if (packet->PeekPacketTag(idTag)) {
        packetSenderMap[idTag.GetId()] = senderNodeId;
    }
}

// Count all messages including ACK
void OnPacketReceptionCallback(Ptr<const Packet> packet, uint32_t receiverNodeId) {
    LoraTag tag;
    if (packet->PeekPacketTag(tag)) {
        packetsReceived.at(tag.GetSpreadingFactor() - 7)++;
    }
    UniquePacketIdTag idTag;
    if (packet->PeekPacketTag(idTag)) {
        uint32_t packetId = idTag.GetId();
        auto it = packetSenderMap.find(packetId);
        if (it != packetSenderMap.end()) {
            uint32_t senderId = it->second;
            if (senderId < packetsReceivedPerNode.size()) {
                packetsReceivedPerNode[senderId]++;
            }
        }
    }
}

int main(int argc, char *argv[]) {
    CommandLine cmd;
    cmd.Parse(argc, argv);

    LogComponentEnable("BridgeExperimental", LOG_LEVEL_INFO);
    NS_LOG_INFO("Starting BridgeExperimental simulation...");

    // Lora Helper
    LoraHelper helper;
    helper.EnablePacketTracking();

    // Nodes and mobility
    const int nDevices = 2;
    NodeContainer endDevices;
    endDevices.Create(nDevices);
    NodeContainer gateways;
    gateways.Create(1);

    MobilityHelper mobility;
    Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();

    const double spacing = 5.0;
    for (int i = 0; i < nDevices; ++i) {
        double x = i * spacing + 5;
        double y = (i % 2 == 0) ? 0 : 1;
        allocator->Add(Vector(x, y, 0));
        NS_LOG_INFO("Placed end device " << i << " at x=" << x << ", y=" << y);
    }
    allocator->Add(Vector(0, 0, 0)); // Gateway at origin
    NS_LOG_INFO("Placed gateway at x=0, y=0");

    mobility.SetPositionAllocator(allocator);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(endDevices);
    mobility.Install(gateways);

    // Channel Setup
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

    // PHY and MAC Setup
    LoraPhyHelper phyHelper;
    phyHelper.SetChannel(channel);
    LorawanMacHelper macHelper;

    // End devices
    phyHelper.SetDeviceType(LoraPhyHelper::ED);
    macHelper.SetDeviceType(LorawanMacHelper::ED_A);
    NetDeviceContainer endDevicesNet = helper.Install(phyHelper, macHelper, endDevices);

    // Gateway
    phyHelper.SetDeviceType(LoraPhyHelper::GW);
    macHelper.SetDeviceType(LorawanMacHelper::GW);
    NetDeviceContainer gatewaysNet = helper.Install(phyHelper, macHelper, gateways);

    

     /**********************
     *  Spreading Factors *
     **********************/
    NS_LOG_INFO("Setting spreading factors...");
    LorawanMacHelper::SetSpreadingFactorsUp(endDevices, gateways, channel);
    NS_LOG_INFO("Spreading factors set.");

    std::vector<uint8_t> spreadingFactors; // Index matches endDevices index

    // Retrieve spreading factors after assignment
    for (uint32_t i = 0; i < endDevices.GetN(); ++i) {
        Ptr<Node> node = endDevices.Get(i);
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(node->GetDevice(0));
        Ptr<EndDeviceLorawanMac> mac = DynamicCast<EndDeviceLorawanMac>(loraNetDevice->GetMac());
        uint8_t dr = mac->GetDataRate(); // Get data rate (DR0 to DR5)
        uint8_t sf = 12 - dr; // Map DR to SF (EU868: DR0=SF12, DR1=SF11, ..., DR5=SF7)
        if (sf < 7 || sf > 12) {
            NS_LOG_ERROR("Invalid SF for node " << i << ": " << unsigned(sf));
            sf = 7; // Fallback to SF7
        }
        spreadingFactors.push_back(sf);
        NS_LOG_INFO("End device " << i << " assigned SF" << unsigned(sf));
    }

     /**********************
     * Connect Traces for tracking on PHY
     **********************/
    for (uint32_t i = 0; i < endDevices.GetN(); ++i) {
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(endDevices.Get(i)->GetDevice(0));
        loraNetDevice->GetPhy()->TraceConnectWithoutContext("StartSending", MakeCallback(OnTransmissionCallback));
    }
    for (uint32_t i = 0; i < gateways.GetN(); ++i) {
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(gateways.Get(i)->GetDevice(0));
        loraNetDevice->GetPhy()->TraceConnectWithoutContext("ReceivedPacket", MakeCallback(OnPacketReceptionCallback));
    }

    // Attach tracing and add periodic sender application for each end device
    for (uint32_t i = 0; i < endDevicesNet.GetN(); ++i) {
        Ptr<LoraNetDevice> dev = DynamicCast<LoraNetDevice>(endDevicesNet.Get(i));
        Ptr<LoraPhy> phy = dev->GetPhy();

        phy->TraceConnectWithoutContext("StartRx", MakeCallback(&StartRxCallback));
        phy->TraceConnectWithoutContext("EndRx", MakeCallback(&RxWindowEndCallback));
        phy->TraceConnectWithoutContext("ReceivedPacket", MakeCallback(&EdPacketReceived));

        // Create and install a periodic sender app on the end device
        Ptr<TaggingPeriodicSender> app = CreateObject<TaggingPeriodicSender>();
        app->Setup(endDevices.Get(i), endDevicesNet.Get(i), Minutes(15), 24); // 24 byte packets
        endDevices.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(i * 20));
        app->SetStopTime(Hours(24));
    }

    

    Simulator::Stop(Hours(3)); // 20 minutes simulation
    Simulator::Run();
    Simulator::Destroy();

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
    

    return 0;
}

