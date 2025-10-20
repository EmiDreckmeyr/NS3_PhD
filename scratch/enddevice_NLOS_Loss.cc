#include "ns3/command-line.h"
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
#include "ns3/lora-frame-header.h"   // <-- For LoraTag
#include "ns3/packet.h"
#include "ns3/names.h"
#include <fstream>
#include <vector>
#include "ns3/propagation-module.h"   // <-- This one is important

using namespace ns3;
using namespace lorawan;

NS_LOG_COMPONENT_DEFINE("BridgeLorawanNetworkNLOS");

/***************
 * PACKET TRACKING
 ***************/
auto packetsSent = std::vector<int>(6, 0);     // DR5 -> DR0
auto packetsReceived = std::vector<int>(6, 0);
std::map<Ptr<const Packet>, uint32_t> packetSenderMap;
// Number of successfully received packets per end device
std::vector<int> packetsReceivedPerNode;


void OnTransmissionCallback(Ptr<const Packet> packet, uint32_t senderNodeId)
{
    LoraTag tag;
    packet->PeekPacketTag(tag);
    packetsSent.at(tag.GetSpreadingFactor() - 7)++;

    // Store sender node ID using Ptr as key (no .get())
    packetSenderMap[packet] = senderNodeId;
}

void OnPacketReceptionCallback(Ptr<const Packet> packet, uint32_t receiverNodeId)
{
    LoraTag tag;
    packet->PeekPacketTag(tag);
    packetsReceived.at(tag.GetSpreadingFactor() - 7)++;

    auto it = packetSenderMap.find(packet);
    if (it != packetSenderMap.end())
    {
        uint32_t senderId = it->second;
        if (senderId < packetsReceivedPerNode.size())
        {
            packetsReceivedPerNode[senderId]++;
        }
    }
}

int main(int argc, char *argv[])
{
    LogComponentEnable("BridgeLorawanNetworkNLOS", LOG_LEVEL_INFO);
    NS_LOG_INFO("Starting BridgeLorawanNetworkNLOS simulation...");

    /**********************
     *  Channel Setup      *
     **********************/
    NS_LOG_INFO("Setting up channel...");

    // Base log-distance model
    Ptr<LogDistancePropagationLossModel> loss = CreateObject<LogDistancePropagationLossModel>();
    loss->SetPathLossExponent(3.9);
    loss->SetReference(1.0, 32.4);  // FSPL at 1 m for 868 MHz
    
    // Add Nakagami fading (multipath)
    Ptr<NakagamiPropagationLossModel> fading = CreateObject<NakagamiPropagationLossModel>();
    fading->SetAttribute("m0", DoubleValue(1.0));
    fading->SetAttribute("m1", DoubleValue(1.5));
    fading->SetAttribute("m2", DoubleValue(3.0));
    loss->SetNext(fading);
    
    // Propagation delay
    Ptr<PropagationDelayModel> delay = CreateObject<ConstantSpeedPropagationDelayModel>();
    
    // Full LoRa channel
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
    allocator->Add(Vector(0, -5, 0)); // Gateway
    NS_LOG_INFO("Placed gateway at origin, y = 5.");

    mobility.SetPositionAllocator(allocator);
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

    /**********************
     *  Nodes Creation    *
     **********************/
    NS_LOG_INFO("Creating nodes...");
    NodeContainer endDevices;
    endDevices.Create(nDevices);
    NodeContainer gateways;
    gateways.Create(1);

    mobility.Install(endDevices);
    mobility.Install(gateways);
    NS_LOG_INFO("Nodes creation and mobility installation complete.");

    // Initialize per-node packet counters
    packetsReceivedPerNode.resize(endDevices.GetN(), 0);

    /**********************
     *  Helpers Setup     *
     **********************/
    NS_LOG_INFO("Setting up helpers...");
    LoraPhyHelper phyHelper;
    phyHelper.SetChannel(channel);
    LorawanMacHelper macHelper;
    LoraHelper helper;
    helper.EnablePacketTracking(); // Enable packet tracking
    NS_LOG_INFO("Helpers setup complete.");

    /**********************
     *  Devices Setup     *
     **********************/
    NS_LOG_INFO("Installing end device network devices...");
    phyHelper.SetDeviceType(LoraPhyHelper::ED);
    macHelper.SetDeviceType(LorawanMacHelper::ED_A);
    NetDeviceContainer endDevicesNet = helper.Install(phyHelper, macHelper, endDevices);
    NS_LOG_INFO("End devices installed.");

    NS_LOG_INFO("Installing gateway network device...");
    phyHelper.SetDeviceType(LoraPhyHelper::GW);
    macHelper.SetDeviceType(LorawanMacHelper::GW);
    NetDeviceContainer gatewaysNet = helper.Install(phyHelper, macHelper, gateways);
    NS_LOG_INFO("Gateway installed.");

    /**********************
     *  Applications      *
     **********************/
    NS_LOG_INFO("Setting up periodic applications...");
    PeriodicSenderHelper sender;
    //sender.SetAttribute("Interval", TimeValue(Minutes(15)));
    sender.SetPeriod(Minutes(15));
    NS_LOG_INFO("Sender Interval of 15 minutes.");

    for (uint32_t i = 0; i < endDevices.GetN(); ++i)
    {
        sender.SetAttribute("StartTime", TimeValue(Seconds(i * 20)));
        sender.Install(endDevices.Get(i));
        NS_LOG_INFO("Periodic sender installed on device " << i << " with start time " << i * 20 << " seconds.");
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

    /**********************
     *  Connect Traces    *
     **********************/
    // End devices
    for (uint32_t i = 0; i < endDevices.GetN(); ++i)
    {
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(endDevices.Get(i)->GetDevice(0));
        loraNetDevice->GetPhy()->TraceConnectWithoutContext("StartSending", MakeCallback(OnTransmissionCallback));
    }
    // Gateways
    for (uint32_t i = 0; i < gateways.GetN(); ++i)
    {
        Ptr<LoraNetDevice> loraNetDevice = DynamicCast<LoraNetDevice>(gateways.Get(i)->GetDevice(0));
        loraNetDevice->GetPhy()->TraceConnectWithoutContext("ReceivedPacket", MakeCallback(OnPacketReceptionCallback));
    }

    /**********************
     *  NetAnim Setup     *
     **********************/
    AnimationInterface anim("BridgeLorawanNetworkNLOS.xml");
    for (uint32_t i = 0; i < endDevices.GetN(); ++i)
    {
        anim.UpdateNodeDescription(endDevices.Get(i), "ED" + std::to_string(i));
        anim.UpdateNodeColor(endDevices.Get(i), 0, 255, 0);
    }
    anim.UpdateNodeDescription(gateways.Get(0), "GW");
    anim.UpdateNodeColor(gateways.Get(0), 255, 0, 0);

    /**********************
     *  Simulation        *
     **********************/
    NS_LOG_INFO("Starting simulation for 24 hours...");
    Simulator::Stop(Hours(24));
    Simulator::Run();

    /**********************
     *  Packet Stats      *
     **********************/
    NS_LOG_INFO("Packets sent vs received per DR (SF7 -> SF12):");
    for (int i = 0; i < 6; i++)
    {
        std::cout << "DR" << (5 - i) << " (SF" << (7 + i) << "): Sent = "
                  << packetsSent.at(i) << ", Received = " << packetsReceived.at(i) << std::endl;
    }
    NS_LOG_INFO("Successful transmission to Gateway per end device:");
    for (uint32_t i = 0; i < packetsReceivedPerNode.size(); ++i)
    {
        std::cout << "Node " << i << ": " << packetsReceivedPerNode[i] << " packets received successfully by GW." << std::endl;
    }

    /**********************
     *  Energy Logging    *
     **********************/
    NS_LOG_INFO("Logging energy consumption...");
    double simDuration = Simulator::Now().GetSeconds();
    NS_LOG_INFO("Total simulation duration: " << simDuration << " seconds");

    std::ofstream texFile("EndNodeTimeDrivenNLOS.tex");
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
    NS_LOG_INFO("Energy log saved to EndNodeTimeDrivenNLOS.tex");

    Simulator::Destroy();
    NS_LOG_INFO("Simulation finished.");
    return 0;
}
