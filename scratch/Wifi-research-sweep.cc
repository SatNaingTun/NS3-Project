#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include <fstream>
#include <iomanip>
#include <ctime>
#include <filesystem>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiResearchSweep");

// --- RSSI tracer ---
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
  // -------- Default Parameters --------
  bool isIndoor = true;
  bool enableInterference = true;
  double areaHalf = 50.0;
  double simTime = 30.0;
  double bgLoadMbps = 10.0;
  uint32_t seed = 12345;
  uint16_t appPort = 9999;
  uint32_t packetSize = 1024;
  double clientIntervalMs = 10.0;
  std::string baseOut = "outputs";

  // Swept parameters
  std::vector<double> txPowerList = {10.0, 14.0, 18.0, 22.0};
  std::vector<uint32_t> nStaList = {10, 20, 30, 40};
  uint32_t nRuns = 3;

  CommandLine cmd;
  cmd.AddValue("isIndoor", "Indoor (true) or Outdoor (false)", isIndoor);
  cmd.AddValue("enableInterference", "Enable co-channel interference", enableInterference);
  cmd.AddValue("seed", "Base RNG seed", seed);
  cmd.AddValue("baseOut", "Base output directory", baseOut);
  cmd.Parse(argc, argv);

  // -------- Directory structure --------
  std::string csvDir = baseOut + "/csv/wifi-research";
  std::string flowDir = baseOut + "/netflows/wifi-research";
  std::filesystem::create_directories(csvDir);
  std::filesystem::create_directories(flowDir);

  // --- Timestamp for filenames ---
  time_t now = time(0);
  tm *ltm = localtime(&now);
  std::ostringstream dateStr;
  dateStr << std::setfill('0') << std::setw(2) << ltm->tm_mday << "-"
          << std::put_time(ltm, "%b") << "-"
          << (1900 + ltm->tm_year) << "_"
          << std::setw(2) << ltm->tm_hour << "-"
          << std::setw(2) << ltm->tm_min;
  std::string dateTag = dateStr.str();

  // Metadata CSV
  std::ofstream meta((csvDir + "/research-meta-" + dateTag + ".csv").c_str());
  meta << "RunID,TxPower_dBm,NodeCount,AreaHalf,Indoor,Interference,Seed,Run,"
          "Throughput_Mbps,Latency_ms,Jitter_ms,Loss_pct\n";

  uint32_t runID = 0;

  // -------- Parameter Sweeping --------
  for (double txPower : txPowerList)
  {
    for (uint32_t nSta : nStaList)
    {
      for (uint32_t run = 1; run <= nRuns; ++run)
      {
        runID++;
        std::cout << "\n=== Run " << runID << " | Tx=" << txPower
                  << " dBm | STA=" << nSta << " | Run=" << run << " ===\n";

        // RNG setup
        RngSeedManager::SetSeed(seed);
        RngSeedManager::SetRun(run);

        // Node setup
        NodeContainer staNodes; staNodes.Create(nSta);
        NodeContainer apNode; apNode.Create(1);

        // Wi-Fi config
        WifiHelper wifi; wifi.SetStandard(WIFI_STANDARD_80211ac);
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
        YansWifiPhyHelper phy; phy.SetChannel(channel.Create());
        phy.Set("TxPowerStart", DoubleValue(txPower));
        phy.Set("TxPowerEnd", DoubleValue(txPower));

        WifiMacHelper mac;
        Ssid ssid = Ssid("bss-main");
        mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing", BooleanValue(false));
        NetDeviceContainer staDevs = wifi.Install(phy, mac, staNodes);
        mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
        NetDeviceContainer apDev = wifi.Install(phy, mac, apNode);

        // Mobility with random initial positions
        Ptr<UniformRandomVariable> ux = CreateObject<UniformRandomVariable>();
        Ptr<UniformRandomVariable> uy = CreateObject<UniformRandomVariable>();
        ux->SetAttribute("Min", DoubleValue(-areaHalf));
        ux->SetAttribute("Max", DoubleValue(areaHalf));
        uy->SetAttribute("Min", DoubleValue(-areaHalf));
        uy->SetAttribute("Max", DoubleValue(areaHalf));

        Ptr<RandomRectanglePositionAllocator> posAlloc = CreateObject<RandomRectanglePositionAllocator>();
        posAlloc->SetX(ux);
        posAlloc->SetY(uy);

        MobilityHelper mobSta;
        mobSta.SetPositionAllocator(posAlloc);
        mobSta.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                                "Bounds", RectangleValue(Rectangle(-areaHalf, areaHalf, -areaHalf, areaHalf)));
        mobSta.Install(staNodes);

        MobilityHelper mobAp;
        mobAp.SetMobilityModel("ns3::ConstantPositionMobilityModel");
        mobAp.Install(apNode);

        // Internet
        InternetStackHelper stack;
        stack.Install(apNode);
        stack.Install(staNodes);

        Ipv4AddressHelper ip;
        ip.SetBase("10.1.3.0", "255.255.255.0");
        Ipv4InterfaceContainer staIf = ip.Assign(staDevs);
        Ipv4InterfaceContainer apIf = ip.Assign(apDev);

        // Application
        UdpClientHelper client(apIf.GetAddress(0), appPort);
        client.SetAttribute("MaxPackets", UintegerValue(0));
        client.SetAttribute("Interval", TimeValue(MilliSeconds(clientIntervalMs)));
        client.SetAttribute("PacketSize", UintegerValue(packetSize));
        ApplicationContainer clientApps;
        for (uint32_t i = 0; i < staNodes.GetN(); ++i)
          clientApps.Add(client.Install(staNodes.Get(i)));
        UdpServerHelper server(appPort);
        ApplicationContainer serverApp = server.Install(apNode.Get(0));

        serverApp.Start(Seconds(1.0));
        clientApps.Start(Seconds(2.0));
        clientApps.Stop(Seconds(simTime));
        serverApp.Stop(Seconds(simTime));

        FlowMonitorHelper flowmon;
        Ptr<FlowMonitor> monitor = flowmon.InstallAll();

        std::ostringstream tag;
        tag << "Tx" << txPower << "_N" << nSta << "_Run" << run << "_" << dateTag;
        std::string prefix = csvDir + "/wifi-research-" + tag.str();

        std::ofstream rssiCsv((prefix + "-rssi.csv").c_str());
        rssiCsv << "time_s,channel_MHz,signal_dBm,noise_dBm\n";
        Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
                                      MakeBoundCallback(&RssiTracer, &rssiCsv));

        Simulator::Stop(Seconds(simTime + 1));
        Simulator::Run();

        // --- Collect FlowMonitor results ---
        std::ofstream perfCsv((prefix + "-perf.csv").c_str());
        perfCsv << "FlowID,Source,Destination,Throughput(Mbps),Latency_ms,Jitter_ms,PacketLoss(%)\n";

        monitor->CheckForLostPackets();
        Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
        auto stats = monitor->GetFlowStats();

        double totalThr = 0, totalDelay = 0, totalJitter = 0, totalLoss = 0;
        uint32_t count = 0;

        for (auto const &kv : stats)
        {
          Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(kv.first);
          const FlowMonitor::FlowStats &s = kv.second;
          double dur = (s.timeLastRxPacket - s.timeFirstTxPacket).GetSeconds();
          double thr = (dur > 0 && s.rxBytes > 0) ? (s.rxBytes * 8.0 / dur) / 1e6 : 0.0;
          double dly = (s.rxPackets > 0) ? (s.delaySum.GetSeconds() / s.rxPackets) * 1000.0 : 0.0;
          double jit = (s.rxPackets > 0) ? (s.jitterSum.GetSeconds() / s.rxPackets) * 1000.0 : 0.0;
          double los = (s.txPackets > 0) ? 100.0 * (s.txPackets - s.rxPackets) / s.txPackets : 0.0;

          perfCsv << kv.first << "," << t.sourceAddress << "," << t.destinationAddress << ","
                  << thr << "," << dly << "," << jit << "," << los << "\n";

          totalThr += thr; totalDelay += dly; totalJitter += jit; totalLoss += los;
          count++;
        }

        perfCsv.close();
        rssiCsv.close();

        if (count > 0)
        {
          meta << runID << "," << txPower << "," << nSta << "," << areaHalf << ","
               << (isIndoor ? 1 : 0) << "," << (enableInterference ? 1 : 0) << ","
               << seed << "," << run << ","
               << totalThr / count << "," << totalDelay / count << ","
               << totalJitter / count << "," << totalLoss / count << "\n";
        }

        monitor->SerializeToXmlFile(flowDir + "/wifi-research-flow-" + tag.str() + ".xml", true, true);
        Simulator::Destroy();
      }
    }
  }

  meta.close();
  std::cout << "\n✅ Research sweep complete. Files saved under " << csvDir << "\n";
  return 0;
}
