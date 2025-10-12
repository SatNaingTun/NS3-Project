#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include <fstream>
#include <iostream>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiPerformanceSim");

int main(int argc, char *argv[])
{
    uint32_t nSta = 10;
    double txPower = 16.0;
    bool isIndoor = true;
    double simTime = 20.0;

    CommandLine cmd;
    cmd.AddValue("nSta", "Number of STA nodes", nSta);
    cmd.AddValue("txPower", "Transmission power (dBm)", txPower);
    cmd.AddValue("isIndoor", "Indoor (true) or Outdoor (false)", isIndoor);
    cmd.AddValue("simTime", "Simulation time (s)", simTime);
    cmd.Parse(argc, argv);

    NodeContainer wifiStaNodes;
    wifiStaNodes.Create(nSta);
    NodeContainer wifiApNode;
    wifiApNode.Create(1);

    // --- WiFi Configuration ---
    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ac);

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    if (isIndoor)
    {
        channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel");
    }
    else
    {
        channel.AddPropagationLoss("ns3::FriisPropagationLossModel");
    }

    YansWifiPhyHelper phy = YansWifiPhyHelper();
    phy.SetChannel(channel.Create());
    phy.Set("TxPowerStart", DoubleValue(txPower));
    phy.Set("TxPowerEnd", DoubleValue(txPower));

    WifiMacHelper mac;
    Ssid ssid = Ssid("wifi-perf");
    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid),
                "ActiveProbing", BooleanValue(false));
    NetDeviceContainer staDevices = wifi.Install(phy, mac, wifiStaNodes);

    mac.SetType("ns3::ApWifiMac",
                "Ssid", SsidValue(ssid));
    NetDeviceContainer apDevice = wifi.Install(phy, mac, wifiApNode);

    // --- Mobility ---
    MobilityHelper mobility;
    mobility.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                              "Bounds", RectangleValue(Rectangle(-50, 50, -50, 50)));
    mobility.Install(wifiStaNodes);

    MobilityHelper mobilityAp;
    mobilityAp.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobilityAp.Install(wifiApNode);

    // --- Internet Stack ---
    InternetStackHelper stack;
    stack.Install(wifiApNode);
    stack.Install(wifiStaNodes);

    Ipv4AddressHelper address;
    address.SetBase("10.1.3.0", "255.255.255.0");
    Ipv4InterfaceContainer staInterfaces = address.Assign(staDevices);
    Ipv4InterfaceContainer apInterface = address.Assign(apDevice);

    // --- Application (UDP) ---
    uint16_t port = 9;
    UdpEchoServerHelper echoServer(port);
    ApplicationContainer serverApp = echoServer.Install(wifiApNode.Get(0));
    serverApp.Start(Seconds(1.0));
    serverApp.Stop(Seconds(simTime - 1));

    UdpEchoClientHelper echoClient(apInterface.GetAddress(0), port);
    echoClient.SetAttribute("MaxPackets", UintegerValue(10000));
    echoClient.SetAttribute("Interval", TimeValue(MilliSeconds(10)));
    echoClient.SetAttribute("PacketSize", UintegerValue(1024));
    ApplicationContainer clientApp = echoClient.Install(wifiStaNodes);
    clientApp.Start(Seconds(2.0));
    clientApp.Stop(Seconds(simTime));

    // --- Enable PCAP (Wireshark) ---
    phy.EnablePcapAll("outputs/wifi-trace", true);

    // --- NetAnim Trace ---
    AnimationInterface anim("outputs/wifi-netanim.xml");
    anim.SetMobilityPollInterval(Seconds(1));
    anim.EnablePacketMetadata(true);
    // anim.EnableIpv4RouteTracking("outputs/wifi-routing.xml", Seconds(1), Seconds(simTime));

    // --- Flow Monitor ---
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    Simulator::Stop(Seconds(simTime + 1));
    Simulator::Run();

    // --- Export Flow Monitor Results to CSV ---
    std::ofstream csv("outputs/wifi-performance.csv");
    csv << "FlowID,Source,Destination,Throughput(Mbps),Delay(ms),Jitter(ms),PacketLoss(%)\n";

    monitor->CheckForLostPackets();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats();

    for (auto const &flow : stats)
    {
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(flow.first);
        double throughput = (flow.second.rxBytes * 8.0 /
                             (flow.second.timeLastRxPacket.GetSeconds() -
                              flow.second.timeFirstTxPacket.GetSeconds())) /
                            1e6;
        double delay = (flow.second.delaySum.GetSeconds() / flow.second.rxPackets) * 1000;
        double jitter = (flow.second.jitterSum.GetSeconds() / flow.second.rxPackets) * 1000;
        double loss = 100.0 * (flow.second.txPackets - flow.second.rxPackets) / flow.second.txPackets;

        csv << flow.first << ","
            << t.sourceAddress << ","
            << t.destinationAddress << ","
            << throughput << ","
            << delay << ","
            << jitter << ","
            << loss << "\n";
    }
    csv.close();

    monitor->SerializeToXmlFile("outputs/wifi-flow.xml", true, true);

    Simulator::Destroy();
    std::cout << "✅ Simulation completed.\n"
              << "Generated files:\n"
              << " - wifi-performance.csv\n"
              << " - wifi-flow.xml\n"
              << " - wifi-netanim.xml\n"
              << " - wifi-trace-*.pcap\n";
    return 0;
}
