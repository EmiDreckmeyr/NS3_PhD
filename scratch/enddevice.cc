#include "ns3/command-line.h"
#include "ns3/okumura-hata-propagation-loss-model.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/end-device-lora-phy.h"
#include "ns3/end-device-lorawan-mac.h"
#include "ns3/gateway-lora-phy.h"
#include "ns3/gateway-lorawan-mac.h"
#include "ns3/log.h"
#include "ns3/lora-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/node-container.h"
#include "ns3/periodic-sender-helper.h"
#include "ns3/position-allocator.h"
#include "ns3/simulator.h"
#include "ns3/basic-energy-source.h"
#include "ns3/lora-radio-energy-model.h"
#include "ns3/lora-radio-energy-model-helper.h"
#include "ns3/basic-energy-source-helper.h"
#include "ns3/class-a-end-device-lorawan-mac.h"
#include "ns3/netanim-module.h"
#include "ns3/animation-interface.h"
#include "ns3/lora-net-device.h"
#include "ns3/lora-frame-header.h"
#include "ns3/packet.h"
#include "ns3/names.h"
#include <fstream>
#include <vector>
#include <map>
#include "ns3/propagation-module.h"
#include "ns3/application.h"
#include "ns3/callback.h"
#include "ns3/propagation-environment.h"
#include "ns3/simulator.h"
#include "ns3/lorawan-mac-header.h"

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE("BridgeLorawanNetworkNLOST");

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
    TaggingPeriodicSender() : m_period(Seconds(60)), m_packetSize(20), m_packetsSent(0) {}  // Fix: init order same as declaration order

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
        Ptr<Packet> packet = Create<Packet>(m_packetSize);
        UniquePacketIdTag idTag(++globalPacketId);
        packet->AddPacketTag(idTag);

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

/***************
 * Main simulation code
 ***************/
int main(int argc, char *argv[]) {
    LogComponentEnable("BridgeLorawanNetworkNLOST", LOG_LEVEL_INFO);
    NS_LOG_INFO("Starting BridgeLorawanNetworkNLOST simulation...");

    /**********************
     * Channel Setup
     **********************/
    Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
    loss->SetPathLossExponent(3.9);
    loss->SetReference(1.0, 32.4);

    // Set the environment (Urban/SubUrban/Open)
    // For harsh bridge, Urban is usually realistic
    Ptr<OkumuraHataPropagationLossModel> okumuraLoss = CreateObject<OkumuraHataPropagationLossModel>();
    okumuraLoss->SetAttribute("Environment", EnumValue(ns3::UrbanEnvironment)); 
    okumuraLoss->SetAttribute("CitySize", EnumValue(CitySize::LargeCity));
    okumuraLoss->SetAttribute("Frequency", DoubleValue(868.0));

    Ptr<NakagamiPropagationLossModel> fading = CreateObject<NakagamiPropagationLossModel>();
    fading->SetAttribute("m0", DoubleValue(1.0));
    fading->SetAttribute("m1", DoubleValue(1.5));
    fading->SetAttribute("m2", DoubleValue(3.0));
    loss->SetNext(fading);

    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();
    Ptr<LoraChannel> channel = CreateObject<LoraChannel>(loss, delay);
    NS_LOG_INFO("Channel setup complete.");

    /**********************
     *  Mobility Setup     *
     **********************/
    NS_LOG_INFO("Setting up mobility...");
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> allocator = CreateObject<ListPositionAllocator>();

    const int nDevices = 20;
    const double spacing = 5.0; // meters between devices
    for (int i = 0; i < nDevices; ++i)
    {
        double x = i * spacing + 5;
        double y = (i % 2 == 0) ? 0 : 1; // Example: alternate placement

        allocator->Add(Vector(x, y, 0));
        NS_LOG_INFO("Placed end device " << i << " at x=" << x << ", y=" << y);
    }
    allocator->Add(Vector(-100, -5, 0)); // Gateway
    NS_LOG_INFO("Placed gateway at origin, x = 100m, y = 5m.");

    mobility.SetPositionAllocator(allocator);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    /**********************
     * Nodes Creation
     **********************/
    NodeContainer endDevices;
    endDevices.Create(nDevices);
    NodeContainer gateways;
    gateways.Create(1);

    mobility.Install(endDevices);
    mobility.Install(gateways);

    packetsReceivedPerNode.resize(endDevices.GetN(), 0);

    /**********************
     * Helpers Setup
     **********************/
    LoraPhyHelper phyHelper;
    phyHelper.SetChannel(channel);
    LorawanMacHelper macHelper;
    LoraHelper helper;
    helper.EnablePacketTracking();

    /**********************
     * Devices Setup
     **********************/
    phyHelper.SetDeviceType(LoraPhyHelper::ED);
    macHelper.SetDeviceType(LorawanMacHelper::ED_A);
    NetDeviceContainer endDevicesNet = helper.Install(phyHelper, macHelper, endDevices);

    phyHelper.SetDeviceType(LoraPhyHelper::GW);
    macHelper.SetDeviceType(LorawanMacHelper::GW);
    NetDeviceContainer gatewaysNet = helper.Install(phyHelper, macHelper, gateways);

    /**********************
     * Applications Setup - Use custom TaggingPeriodicSender
     **********************/
    for (uint32_t i = 0; i < endDevices.GetN(); ++i) {
        Ptr<TaggingPeriodicSender> app = CreateObject<TaggingPeriodicSender>();
        app->Setup(endDevices.Get(i), endDevicesNet.Get(i), Minutes(15), 24); // 24 byte packets
        endDevices.Get(i)->AddApplication(app);
        app->SetStartTime(Seconds(i * 20));
        app->SetStopTime(Hours(24));
    }

    /**********************
     *  Energy Setup      *
     **********************/
    NS_LOG_INFO("Setting up energy model...");
    NS_LOG_INFO("8 Ah at 3.3 V -> 95,040 J, use 10% of battery capacity for comms");
    // 8 Ah at 3.3 V -> 95,040 J, use 10% of battery capacity for comms
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

    // install source on end devices' nodes
    EnergySourceContainer sources = basicSourceHelper.Install(endDevices);
    //Names::Add("/Names/EnergySource", sources.Get(0));
    // install device model
    DeviceEnergyModelContainer deviceModels =radioEnergyHelper.Install(endDevicesNet, sources);
    NS_LOG_INFO("Energy model installed.");


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

    /**********************
     *  NetAnim Setup     *
     **********************/
    AnimationInterface anim("BridgeLorawanNetworkNLOST.xml");
    for (uint32_t i = 0; i < endDevices.GetN(); ++i)
    {
        anim.UpdateNodeDescription(endDevices.Get(i), "ED" + std::to_string(i));
        anim.UpdateNodeColor(endDevices.Get(i), 0, 255, 0);
    }
    anim.UpdateNodeDescription(gateways.Get(0), "GW");
    anim.UpdateNodeColor(gateways.Get(0), 255, 0, 0);


    Simulator::Stop(Hours(24));
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

    /**********************
     *  Energy Logging    *
     **********************/
    NS_LOG_INFO("Logging energy consumption...");
    double simDuration = Simulator::Now().GetSeconds();
    NS_LOG_INFO("Total simulation duration: " << simDuration << " seconds");

    std::ofstream texFile("EndNodeTimeDrivenNLOST.tex");
    texFile << "\\documentclass{article}\n"
            << "\\usepackage{booktabs}\n"
            << "\\begin{document}\n"
            << "Simulation duration: " << simDuration << " seconds.\\\\\n\n"
            << "\\begin{tabular}{ccc}\n"
            << "\\toprule\n"
            << "Node ID & Initial Energy (J) & Energy Consumed (J) \\\\\n"
            << "\\midrule\n";

    for (uint32_t i = 0; i < sources.GetN(); ++i)
    {
        Ptr<BasicEnergySource> src = sources.Get(i)->GetObject<BasicEnergySource>();
        double initialEnergy = src->GetInitialEnergy();
        double remainingEnergy = src->GetRemainingEnergy();
        double consumed = initialEnergy - remainingEnergy;

        NS_LOG_INFO("Node " << i << ": Initial=" << initialEnergy 
                    << " J, Consumed=" << consumed << " J, Remaining=" << remainingEnergy << " J");

        texFile << i << " & " << initialEnergy << " & " << consumed << " \\\\\n";
    }

    texFile << "\\bottomrule\n"
            << "\\end{tabular}\n"
            << "\\end{document}\n";
    texFile.close();
    NS_LOG_INFO("Energy log saved to EndNodeTimeDrivenNLOST.tex");

    Simulator::Destroy();
    return 0;
}
