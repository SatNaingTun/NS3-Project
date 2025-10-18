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
#include <iomanip>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiPerfRandom");

// ---- RSSI tracer (MonitorSnifferRx) ----
static void RssiTracer(std::ofstream *csv,
                       Ptr<const Packet> pkt,
                       uint16_t channelFreqMhz,
                       WifiTxVector txVector,
                       MpduInfo mpduInfo,
                       SignalNoiseDbm signalNoise,
                       uint16_t staId)
{
  if (!csv || !csv->good()) return;
  *csv << std::fixed << std::setprecision(6)
       << Simulator::Now().GetSeconds() << ","
       << channelFreqMhz << ","
       << signalNoise.signal << ","
       << signalNoise.noise << "\n";
}

int main(int argc, char *argv[])
{
  // ------- Parameters -------
  bool isIndoor = true;
  uint32_t nStaMin = 5;
  uint32_t nStaMax = 30;
  double areaHalf = 50.0;
  double simTime = 30.0;
  double txPower = 16.0;
  bool enableInterference = true;
  double bgLoadMbps = 10.0;
  uint32_t seed = 12345;
  uint16_t appPort = 9999;
  uint32_t packetSize = 1024;
  double clientIntervalMs = 10.0;
  std::string outPrefix = "outputs/wifi-random";

  CommandLine cmd;
  cmd.AddValue("isIndoor", "Indoor (true) or Outdoor (false)", isIndoor);
  cmd.AddValue("nStaMin", "Minimum number of stations", nStaMin);
  cmd.AddValue("nStaMax", "Maximum number of stations", nStaMax);
  cmd.AddValue("areaHalf", "+/- meters for RandomWalk bounds", areaHalf);
  cmd.AddValue("simTime", "Simulation time (s)", simTime);
  cmd.AddValue("txPower", "Wi-Fi Tx power (dBm)", txPower);
  cmd.AddValue("enableInterference", "Enable co-channel interference BSS", enableInterference);
  cmd.AddValue("bgLoadMbps", "Background load (Mbps) if interference enabled", bgLoadMbps);
  cmd.AddValue("packetSize", "App packet size (bytes)", packetSize);
  cmd.AddValue("clientIntervalMs", "Client send interval (ms)", clientIntervalMs);
  cmd.AddValue("seed", "RNG seed", seed);
  cmd.AddValue("outPrefix", "Output files prefix (without extension)", outPrefix);
  cmd.Parse(argc, argv);

  // RNG + random STA count
  RngSeedManager::SetSeed(seed);
  Ptr<UniformRandomVariable> u = CreateObject<UniformRandomVariable>();
  u->SetAttribute("Min", DoubleValue(nStaMin));
  u->SetAttribute("Max", DoubleValue(nStaMax + 1));
  uint32_t nSta = (uint32_t)std::floor(u->GetValue());
  if (nSta < 1) nSta = 1;

  NS_LOG_UNCOND("\n=== Wi-Fi Scenario ===");
  NS_LOG_UNCOND("isIndoor=" << isIndoor << ", nSta=" << nSta
                 << ", areaHalf=" << areaHalf
                 << ", simTime=" << simTime
                 << "s, txPower=" << txPower << " dBm");

  // -------- Nodes --------
  NodeContainer staNodes; staNodes.Create(nSta);
  NodeContainer apNode;   apNode.Create(1);
  NodeContainer intfApNode;
  NodeContainer intfStaNodes;

  if (enableInterference)
  {
    intfApNode.Create(1);
    intfStaNodes.Create(std::max(1u, nSta / 3));
  }

  // -------- Wi-Fi configuration --------
  WifiHelper wifi; wifi.SetStandard(WIFI_STANDARD_80211ac);

  YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
  if (isIndoor)
  {
    channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                               "Exponent", DoubleValue(3.0));
    channel.AddPropagationLoss("ns3::NakagamiPropagationLossModel");
  }
  else
  {
    channel.AddPropagationLoss("ns3::FriisPropagationLossModel");
    channel.AddPropagationLoss("ns3::NakagamiPropagationLossModel");
  }

  YansWifiPhyHelper phy; phy.SetChannel(channel.Create());
  phy.Set("TxPowerStart", DoubleValue(txPower));
  phy.Set("TxPowerEnd", DoubleValue(txPower));

  WifiMacHelper mac;
  Ssid ssid = Ssid("bss-main");

  mac.SetType("ns3::StaWifiMac",
              "Ssid", SsidValue(ssid),
              "ActiveProbing", BooleanValue(false));
  NetDeviceContainer staDevs = wifi.Install(phy, mac, staNodes);

  mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
  NetDeviceContainer apDev = wifi.Install(phy, mac, apNode);

  // Interfering BSS
  NetDeviceContainer intfStaDevs, intfApDev;
  Ssid ssid2 = Ssid("bss-intf");
  if (enableInterference)
  {
    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid2),
                "ActiveProbing", BooleanValue(false));
    intfStaDevs = wifi.Install(phy, mac, intfStaNodes);

    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid2));
    intfApDev = wifi.Install(phy, mac, intfApNode);
  }

  // -------- Mobility --------
  MobilityHelper mobSta;
  mobSta.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                          "Bounds", RectangleValue(Rectangle(-areaHalf, areaHalf, -areaHalf, areaHalf)));
  mobSta.Install(staNodes);

  MobilityHelper mobAp;
  mobAp.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobAp.Install(apNode);

  if (enableInterference)
  {
    MobilityHelper mobIntfSta;
    mobIntfSta.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                                "Bounds", RectangleValue(Rectangle(-areaHalf, areaHalf, -areaHalf, areaHalf)));
    mobIntfSta.Install(intfStaNodes);

    MobilityHelper mobIntfAp;
    mobIntfAp.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    mobIntfAp.Install(intfApNode);

    Ptr<MobilityModel> apmm = intfApNode.Get(0)->GetObject<MobilityModel>();
    if (apmm)
      apmm->SetPosition(Vector(areaHalf, 0.0, 0.0));
  }

  // -------- Internet stack + IPs --------
  InternetStackHelper stack;
  stack.Install(apNode);
  stack.Install(staNodes);
  if (enableInterference)
  {
    stack.Install(intfApNode);
    stack.Install(intfStaNodes);
  }

  Ipv4AddressHelper ip;
  ip.SetBase("10.1.3.0", "255.255.255.0");
  Ipv4InterfaceContainer staIf = ip.Assign(staDevs);
  Ipv4InterfaceContainer apIf  = ip.Assign(apDev);

  Ipv4InterfaceContainer intfStaIf, intfApIf;
  if (enableInterference)
  {
    ip.SetBase("10.1.4.0", "255.255.255.0");
    intfStaIf = ip.Assign(intfStaDevs);
    intfApIf  = ip.Assign(intfApDev);
  }

  // -------- Applications --------
  UdpClientHelper client(apIf.GetAddress(0), appPort);
  client.SetAttribute("MaxPackets", UintegerValue(0));
  client.SetAttribute("Interval", TimeValue(MilliSeconds(clientIntervalMs)));
  client.SetAttribute("PacketSize", UintegerValue(packetSize));
  ApplicationContainer clientApps;
  for (uint32_t i = 0; i < staNodes.GetN(); ++i)
    clientApps.Add(client.Install(staNodes.Get(i)));

  UdpServerHelper server(appPort);
  ApplicationContainer serverApp = server.Install(apNode.Get(0));

  // Interfering background traffic
  ApplicationContainer bgClients, bgServer;
  if (enableInterference)
  {
    UdpServerHelper bgSrv(7777);
    bgServer = bgSrv.Install(intfApNode.Get(0));

    double ratebps = bgLoadMbps * 1e6;
    OnOffHelper onoff("ns3::UdpSocketFactory",
                      InetSocketAddress(intfApIf.GetAddress(0), 7777));
    onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    onoff.SetAttribute("DataRate", DataRateValue(DataRate((uint64_t)ratebps)));
    onoff.SetAttribute("PacketSize", UintegerValue(1200));

    for (uint32_t i = 0; i < intfStaNodes.GetN(); ++i)
      bgClients.Add(onoff.Install(intfStaNodes.Get(i)));
  }

  // Timing
  serverApp.Start(Seconds(1.0));
  clientApps.Start(Seconds(2.0));
  if (enableInterference)
  {
    bgServer.Start(Seconds(1.0));
    bgClients.Start(Seconds(1.5));
  }
  clientApps.Stop(Seconds(simTime));
  serverApp.Stop(Seconds(simTime));
  if (enableInterference)
  {
    bgClients.Stop(Seconds(simTime));
    bgServer.Stop(Seconds(simTime));
  }

  // -------- Flow Monitor --------
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  // -------- PCAP and RSSI tracing --------
  std::string pcapPrefix = outPrefix + "-trace";
  phy.EnablePcapAll(pcapPrefix, true);

  std::ofstream rssiCsv((outPrefix + "-rssi.csv").c_str());
  rssiCsv << "time_s,channel_MHz,signal_dBm,noise_dBm\n";
  Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
                                MakeBoundCallback(&RssiTracer, &rssiCsv));

  // -------- NetAnim visualization --------
  AnimationInterface anim(outPrefix + "-netanim.xml");
  anim.SetMobilityPollInterval(Seconds(1));
  anim.EnablePacketMetadata(true);
  anim.UpdateNodeDescription(apNode.Get(0), "AP");
  anim.UpdateNodeColor(apNode.Get(0), 0, 255, 0);
  for (uint32_t i = 0; i < staNodes.GetN(); ++i)
  {
    anim.UpdateNodeDescription(staNodes.Get(i), ("STA-" + std::to_string(i)).c_str());
    anim.UpdateNodeColor(staNodes.Get(i), 0, 0, 255);
  }
  if (enableInterference)
  {
    anim.UpdateNodeDescription(intfApNode.Get(0), "AP-INTF");
    anim.UpdateNodeColor(intfApNode.Get(0), 255, 128, 0);
    for (uint32_t i = 0; i < intfStaNodes.GetN(); ++i)
    {
      anim.UpdateNodeDescription(intfStaNodes.Get(i), ("I-STA-" + std::to_string(i)).c_str());
      anim.UpdateNodeColor(intfStaNodes.Get(i), 180, 0, 180);
    }
  }

  // -------- Run simulation --------
  Simulator::Stop(Seconds(simTime + 1));
  Simulator::Run();

  // -------- Export FlowMonitor to CSV --------
  std::ofstream perfCsv((outPrefix + "-perf.csv").c_str());
  perfCsv << "FlowID,Source,Destination,Throughput(Mbps),Latency_avg(ms),Jitter_avg(ms),PacketLoss(%)\n";

  monitor->CheckForLostPackets();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
  auto stats = monitor->GetFlowStats();

  for (auto const &kv : stats)
  {
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(kv.first);
    const FlowMonitor::FlowStats &s = kv.second;
    double duration = (s.timeLastRxPacket - s.timeFirstTxPacket).GetSeconds();
    double throughput = (duration > 0 && s.rxBytes > 0)
                          ? (s.rxBytes * 8.0 / duration) / 1e6 : 0.0;
    double avgDelayMs = (s.rxPackets > 0) ? (s.delaySum.GetSeconds() / s.rxPackets) * 1000.0 : 0.0;
    double avgJitterMs = (s.rxPackets > 0) ? (s.jitterSum.GetSeconds() / s.rxPackets) * 1000.0 : 0.0;
    double lossPct = (s.txPackets > 0) ? 100.0 * (s.txPackets - s.rxPackets) / s.txPackets : 0.0;

    perfCsv << kv.first << ","
            << t.sourceAddress << ","
            << t.destinationAddress << ","
            << throughput << ","
            << avgDelayMs << ","
            << avgJitterMs << ","
            << lossPct << "\n";
  }
  perfCsv.close();
  rssiCsv.close();
  monitor->SerializeToXmlFile(outPrefix + "-flow.xml", true, true);

  Simulator::Destroy();

  std::cout << "\n✅ Simulation complete. Files generated:\n"
            << " - " << outPrefix << "-perf.csv (throughput, latency, jitter, loss)\n"
            << " - " << outPrefix << "-rssi.csv (RSSI, noise)\n"
            << " - " << outPrefix << "-netanim.xml (for NetAnim)\n"
            << " - " << outPrefix << "-trace-*.pcap (Wireshark)\n"
            << std::endl;
  return 0;
}
// --- IGNORE ---
