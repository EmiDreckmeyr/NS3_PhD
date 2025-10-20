#include "ns3/core-module.h"
#include "ns3/csma-module.h"
#include "ns3/applications-module.h"
#include "ns3/netanim-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("ScratchBridge");

int main(int argc, char *argv[])
{
    LogComponentEnable("ScratchBridge", LOG_LEVEL_INFO);

    NodeContainer nodes;
    nodes.Create(8); // 1 gateway + 7 regular nodes
    NS_LOG_INFO("Created 8 nodes: 1 gateway (Node_0) and 7 regular nodes.");

    CsmaHelper csma;
    csma.SetChannelAttribute("DataRate", StringValue("100Mbps"));
    csma.SetChannelAttribute("Delay", TimeValue(NanoSeconds(6560)));

    NetDeviceContainer devices = csma.Install(nodes);
    NS_LOG_INFO("Configured CSMA channel and installed devices on all nodes.");

    InternetStackHelper stack;
    stack.Install(nodes);
    NS_LOG_INFO("Installed internet stack on all nodes.");

    Ipv4AddressHelper address;
    address.SetBase("192.168.1.0", "255.255.255.0");
    Ipv4InterfaceContainer interfaces = address.Assign(devices);
    NS_LOG_INFO("Assigned IP addresses in the range 192.168.1.0/24.");

    // UDP server on gateway (Node 0) to receive responses from regular nodes
    uint16_t serverPort = 9;
    UdpServerHelper gatewayServer(serverPort);
    ApplicationContainer gatewayServerApp = gatewayServer.Install(nodes.Get(0));
    gatewayServerApp.Start(Seconds(1.0));
    gatewayServerApp.Stop(Seconds(300.0));
    NS_LOG_INFO("Installed UDP server on Gateway (Node_0) to receive responses.");

    // UDP servers on regular nodes (1 to 7) to receive polls
    UdpServerHelper server(serverPort);
    ApplicationContainer serverApps;
    for (uint32_t i = 1; i < nodes.GetN(); ++i)
    {
        serverApps.Add(server.Install(nodes.Get(i)));
        NS_LOG_INFO("Installed UDP server on Regular Node_" << i << " to receive polls.");
    }
    serverApps.Start(Seconds(1.0));
    serverApps.Stop(Seconds(300.0));

    // UDP clients on gateway (Node 0) to poll each regular node sequentially, once per sequence
    ApplicationContainer clientApps;
    for (uint32_t i = 1; i < nodes.GetN(); ++i)
    {
        UdpClientHelper client(interfaces.GetAddress(i), serverPort);
        client.SetAttribute("MaxPackets", UintegerValue(43)); // ~42 sequences in 300s (300 ÷ 7)
        client.SetAttribute("Interval", TimeValue(Seconds(7.0))); // Repeat every sequence (7s)
        client.SetAttribute("PacketSize", UintegerValue(1024));
        ApplicationContainer clientApp = client.Install(nodes.Get(0));
        clientApp.Start(Seconds(2.0 + (i - 1))); // Sequential: 2s, 3s, 4s, ...
        clientApp.Stop(Seconds(300.0));
        clientApps.Add(clientApp);
        NS_LOG_INFO("Installed UDP client on Gateway (Node_0) to poll Regular Node_" << i << " starting at " << 2.0 + (i - 1) << "s, once per 7s sequence.");
    }

    // UDP clients on regular nodes (1 to 7) to respond to polls
    for (uint32_t i = 1; i < nodes.GetN(); ++i)
    {
        UdpClientHelper responseClient(interfaces.GetAddress(0), serverPort);
        responseClient.SetAttribute("MaxPackets", UintegerValue(43)); // Match polling cycles
        responseClient.SetAttribute("Interval", TimeValue(Seconds(7.0))); // Match sequence interval
        responseClient.SetAttribute("PacketSize", UintegerValue(1024));
        ApplicationContainer responseApp = responseClient.Install(nodes.Get(i));
        responseApp.Start(Seconds(2.1 + (i - 1))); // Respond 0.1s after poll: 2.1s, 3.1s, ...
        responseApp.Stop(Seconds(300.0));
        NS_LOG_INFO("Installed UDP client on Regular Node_" << i << " to respond to polls starting at " << 2.1 + (i - 1) << "s, once per 7s sequence.");
    }

    // Position nodes at specified locations
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobility.Install(nodes);

    // Set exact positions (x, y, z) for each node
    Ptr<MobilityModel> mobilityModel;
    
    // Node 0 (Gateway) at x=0m, y=0m
    mobilityModel = nodes.Get(0)->GetObject<MobilityModel>();
    mobilityModel->SetPosition(Vector(0.0, 0.0, 0.0));
    
    // Node 1 at x=10.3m, y=0.5m
    mobilityModel = nodes.Get(1)->GetObject<MobilityModel>();
    mobilityModel->SetPosition(Vector(10.3, 0.5, 0.0));
    
    // Node 2 at x=26.05m, y=0m
    mobilityModel = nodes.Get(2)->GetObject<MobilityModel>();
    mobilityModel->SetPosition(Vector(26.05, 0.0, 0.0));
    
    // Node 3 at x=29.2m, y=0.5m
    mobilityModel = nodes.Get(3)->GetObject<MobilityModel>();
    mobilityModel->SetPosition(Vector(29.2, 0.5, 0.0));
    
    // Node 4 at x=35.5m, y=0m
    mobilityModel = nodes.Get(4)->GetObject<MobilityModel>();
    mobilityModel->SetPosition(Vector(35.5, 0.0, 0.0));
    
    // Node 5 at x=37.5m, y=0.5m
    mobilityModel = nodes.Get(5)->GetObject<MobilityModel>();
    mobilityModel->SetPosition(Vector(37.5, 0.5, 0.0));
    
    // Node 6 at x=43.8m, y=0m
    mobilityModel = nodes.Get(6)->GetObject<MobilityModel>();
    mobilityModel->SetPosition(Vector(43.8, 0.0, 0.0));
    
    // Node 7 at x=46.95m, y=0.5m
    mobilityModel = nodes.Get(7)->GetObject<MobilityModel>();
    mobilityModel->SetPosition(Vector(46.95, 0.5, 0.0));
    
    NS_LOG_INFO("Positioned nodes along 73m bridge: Gateway (Node_0) at (0m, 0m), Regular Node_1 at (10.3m, 0.5m), Node_2 at (26.05m, 0m), Node_3 at (29.2m, 0.5m), Node_4 at (35.5m, 0m), Node_5 at (37.5m, 0.5m), Node_6 at (43.8m, 0m), Node_7 at (46.95m, 0.5m).");

    // Enable NetAnim
    AnimationInterface anim("bridge-network.xml");
    for (uint32_t i = 0; i < nodes.GetN(); ++i)
    {
        std::ostringstream label;
        if (i == 0)
        {
            label << "Gateway";
            anim.UpdateNodeDescription(nodes.Get(i), label.str());
            anim.UpdateNodeColor(nodes.Get(i), 255, 0, 0); // Red for Gateway
        }
        else
        {
            label << "Node_" << i;
            anim.UpdateNodeDescription(nodes.Get(i), label.str());
            anim.UpdateNodeColor(nodes.Get(i), 0, 0, 255); // Blue for regular nodes
        }
    }
    NS_LOG_INFO("Configured NetAnim visualization (bridge-network.xml).");

    // Enable PCAP tracing
    csma.EnablePcapAll("bridge-network");
    NS_LOG_INFO("Enabled PCAP tracing (bridge-network-*.pcap).");

    Simulator::Stop(Seconds(300.0));
    NS_LOG_INFO("Starting simulation for 300 seconds...");
    Simulator::Run();
    NS_LOG_INFO("Simulation completed.");

    Simulator::Destroy();
    NS_LOG_INFO("Simulation resources destroyed.");

    // Automatically launch NetAnim viewer
    int status = system("netanim bridge-network.xml");
    if (status != 0)
    {
        NS_LOG_WARN("Could not open NetAnim automatically. Please open 'bridge-network.xml' manually.");
    }

    return 0;
}
