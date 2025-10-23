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
#include <ctime>
#include <sstream>
#include <cmath>
#include <vector>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiPerfRandom");

// ---- RSSI/SNR/BER tracer with runTag ----
static void
RssiSnrBerTracer(uint32_t randSeed,
                 std::string runTag,
                 std::ofstream *csv,
                 Ptr<const Packet> pkt,
                 uint16_t channelFreqMhz,
                 WifiTxVector txVector,
                 MpduInfo mpduInfo,
                 SignalNoiseDbm signalNoise,
                 uint16_t staId)
{
  if (!csv || !csv->good())
    return;

  double rssi = signalNoise.signal;
  double noise = signalNoise.noise;
  double snr = rssi - noise; // dB
  double snrLinear = std::pow(10.0, snr / 10.0);
  double ber = 0.5 * std::erfc(std::sqrt(snrLinear));

  // placeholder AvgSNR/AvgBER, to be overwritten later
  *csv << std::fixed << std::setprecision(6)
       << Simulator::Now().GetSeconds() << ","
       << channelFreqMhz << ","
       << rssi << ","
       << noise << ","
       << snr << ","
       << ber << ","
       << 0.0 << "," << 0.0 << ","
       << randSeed << "," << runTag << "\n";
}

int main(int argc, char *argv[])
{
  bool isIndoor = true;
  uint32_t nStaMin = 5, nStaMax = 30;
  double areaHalf = 50.0, simTime = 30.0, txPower = 16.0;
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
  cmd.AddValue("bgLoadMbps", "Background load (Mbps)", bgLoadMbps);
  cmd.AddValue("packetSize", "App packet size (bytes)", packetSize);
  cmd.AddValue("clientIntervalMs", "Client send interval (ms)", clientIntervalMs);
  cmd.AddValue("seed", "RNG seed", seed);
  cmd.AddValue("outPrefix", "Output files prefix (without extension)", outPrefix);
  cmd.Parse(argc, argv);

  // ------- Timestamp suffix -------
  time_t now = time(0);
  tm *ltm = localtime(&now);
  std::ostringstream dateSuffix;
  dateSuffix << std::setfill('0') << std::setw(2) << ltm->tm_mday << "-"
             << std::put_time(ltm, "%b") << "-"
             << (1900 + ltm->tm_year) << "_"
             << std::setw(2) << ltm->tm_hour << "-"
             << std::setw(2) << ltm->tm_min;
  std::string runTag = dateSuffix.str();

  // ------- Directory prefixes (with seed tag) -------
  std::ostringstream seedStr;
  seedStr << seed;
  std::string csvPrefix = "outputs/csv/wifi-random-" + runTag + "-seed" + seedStr.str();
  std::string pcapPrefix = "outputs/pcap/wifi-random-" + runTag + "-seed" + seedStr.str();
  std::string animPrefix = "outputs/netanim/wifi-random-" + runTag + "-seed" + seedStr.str();
  std::string flowPrefix = "outputs/netflows/wifi-random-" + runTag + "-seed" + seedStr.str();

  // RNG setup
  RngSeedManager::SetSeed(seed);
  Ptr<UniformRandomVariable> u = CreateObject<UniformRandomVariable>();
  u->SetAttribute("Min", DoubleValue(nStaMin));
  u->SetAttribute("Max", DoubleValue(nStaMax + 1));
  uint32_t nSta = (uint32_t)std::floor(u->GetValue());
  if (nSta < 1)
    nSta = 1;

  NS_LOG_UNCOND("\n=== Wi-Fi Scenario ===");
  NS_LOG_UNCOND("isIndoor=" << isIndoor << ", nSta=" << nSta
                             << ", areaHalf=" << areaHalf
                             << ", simTime=" << simTime
                             << "s, txPower=" << txPower
                             << " dBm, seed=" << seed);

  // -------- Nodes --------
  NodeContainer staNodes;
  staNodes.Create(nSta);
  NodeContainer apNode;
  apNode.Create(1);
  NodeContainer intfApNode, intfStaNodes;
  if (enableInterference)
  {
    intfApNode.Create(1);
    intfStaNodes.Create(std::max(1u, nSta / 3));
  }

  // -------- Wi-Fi --------
  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211ac);
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
  if (isIndoor)
  {
    channel.AddPropagationLoss("ns3::LogDistancePropagationLossModel", "Exponent", DoubleValue(3.0));
    channel.AddPropagationLoss("ns3::NakagamiPropagationLossModel");
  }
  else
  {
    channel.AddPropagationLoss("ns3::FriisPropagationLossModel");
    channel.AddPropagationLoss("ns3::NakagamiPropagationLossModel");
  }
  YansWifiPhyHelper phy;
  phy.SetChannel(channel.Create());
  phy.Set("TxPowerStart", DoubleValue(txPower));
  phy.Set("TxPowerEnd", DoubleValue(txPower));

  WifiMacHelper mac;
  Ssid ssid = Ssid("bss-main");
  mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing", BooleanValue(false));
  NetDeviceContainer staDevs = wifi.Install(phy, mac, staNodes);

  mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
  NetDeviceContainer apDev = wifi.Install(phy, mac, apNode);

  // Interference setup
  NetDeviceContainer intfStaDevs, intfApDev;
  if (enableInterference)
  {
    Ssid ssid2 = Ssid("bss-intf");
    mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid2), "ActiveProbing", BooleanValue(false));
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

  // -------- Internet --------
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
  Ipv4InterfaceContainer apIf = ip.Assign(apDev);

  if (enableInterference)
  {
    ip.SetBase("10.1.4.0", "255.255.255.0");
    ip.Assign(intfStaDevs);
    ip.Assign(intfApDev);
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

  // Background load
  if (enableInterference)
  {
    UdpServerHelper bgSrv(7777);
    bgSrv.Install(intfApNode.Get(0));
    double ratebps = bgLoadMbps * 1e6;
    OnOffHelper onoff("ns3::UdpSocketFactory", InetSocketAddress(Ipv4Address("10.1.4.1"), 7777));
    onoff.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=1]"));
    onoff.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0]"));
    onoff.SetAttribute("DataRate", DataRateValue(DataRate((uint64_t)ratebps)));
    onoff.SetAttribute("PacketSize", UintegerValue(1200));
  }

  serverApp.Start(Seconds(1.0));
  clientApps.Start(Seconds(2.0));
  clientApps.Stop(Seconds(simTime));
  serverApp.Stop(Seconds(simTime));

  // -------- FlowMonitor --------
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  // -------- Traces --------
  phy.EnablePcapAll(pcapPrefix, true);
  std::ofstream rssiCsv((csvPrefix + "-rssi.csv").c_str());
  rssiCsv << "time_s,channel_MHz,signal_dBm,noise_dBm,SNR_dB,BER,AvgSNR(dB),AvgBER,RandSeed,RunDateTime\n";
  Config::ConnectWithoutContext(
      "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
      MakeBoundCallback(&RssiSnrBerTracer, seed, runTag, &rssiCsv));

  AnimationInterface anim(animPrefix + "-netanim.xml");
  anim.SetMobilityPollInterval(Seconds(1));

  // -------- Run Simulation --------
  Simulator::Stop(Seconds(simTime + 1));
  Simulator::Run();

  // --- Compute average SNR/BER ---
  rssiCsv.close();
  std::ifstream inSnrIn((csvPrefix + "-rssi.csv").c_str());
  std::vector<std::string> lines;
  std::string header, l;
  std::getline(inSnrIn, header);
  double snrSum = 0, berSum = 0; int count = 0;
  while (std::getline(inSnrIn, l))
  {
    std::stringstream ss(l);
    double t,f,sig,noise,snr,ber,tmp1,tmp2; uint32_t s; std::string tag; char c;
    ss >> t >> c >> f >> c >> sig >> c >> noise >> c >> snr >> c >> ber;
    if(!ss.fail()){ snrSum += snr; berSum += ber; count++; }
    lines.push_back(l);
  }
  inSnrIn.close();
  double avgSnr = (count>0)?snrSum/count:0.0;
  double avgBer = (count>0)?berSum/count:0.0;

  // rewrite RSSI CSV with AvgSNR/AvgBER per row
  std::ofstream outSnr((csvPrefix + "-rssi.csv").c_str());
  outSnr << header << "\n";
  for(auto &l : lines)
  {
    std::stringstream ss(l);
    double t,f,sig,noise,snr,ber,a1,a2; uint32_t s; std::string tag; char c;
    ss >> t >> c >> f >> c >> sig >> c >> noise >> c >> snr >> c >> ber >> c >> a1 >> c >> a2 >> c >> s >> c >> tag;
    if(!ss.fail())
      outSnr << std::fixed << std::setprecision(6)
             << t << "," << f << "," << sig << "," << noise << "," << snr << "," << ber << ","
             << avgSnr << "," << avgBer << "," << s << "," << tag << "\n";
  }
  outSnr.close();

  // -------- Export FlowMonitor --------
  std::ofstream perfCsv((csvPrefix + "-perf.csv").c_str());
  perfCsv << "FlowID,Source,Destination,Throughput(Mbps),Latency_avg(ms),Jitter_avg(ms),PacketLoss(%),AvgSNR(dB),AvgBER,RandSeed,RunDateTime\n";
  monitor->CheckForLostPackets();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
  auto stats = monitor->GetFlowStats();

  for (auto const &kv : stats)
  {
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(kv.first);
    const FlowMonitor::FlowStats &s = kv.second;
    double duration = (s.timeLastRxPacket - s.timeFirstTxPacket).GetSeconds();
    double throughput = (duration > 0 && s.rxBytes > 0)?(s.rxBytes * 8.0 / duration)/1e6:0.0;
    double avgDelayMs = (s.rxPackets>0)?(s.delaySum.GetSeconds()/s.rxPackets)*1000.0:0.0;
    double avgJitterMs = (s.rxPackets>0)?(s.jitterSum.GetSeconds()/s.rxPackets)*1000.0:0.0;
    double lossPct = (s.txPackets>0)?100.0*(s.txPackets-s.rxPackets)/s.txPackets:0.0;

    perfCsv << kv.first << "," << t.sourceAddress << "," << t.destinationAddress << ","
            << throughput << "," << avgDelayMs << "," << avgJitterMs << "," << lossPct << ","
            << avgSnr << "," << avgBer << "," << seed << "," << runTag << "\n";
  }

  monitor->SerializeToXmlFile(flowPrefix + "-flow.xml", true, true);
  Simulator::Destroy();

  std::cout << "\n✅ Simulation complete. Files generated:\n"
            << "  CSV:      " << csvPrefix << "-*.csv\n"
            << "  PCAP:     " << pcapPrefix << "-*.pcap\n"
            << "  NetAnim:  " << animPrefix << "-netanim.xml\n"
            << "  NetFlows: " << flowPrefix << "-flow.xml\n"
            << std::endl;

  return 0;
}
