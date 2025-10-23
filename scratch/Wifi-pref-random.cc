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
#include <map>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiPerfRandom");

// ==========================================================
//  Global Variables
// ==========================================================
static uint32_t activeNodes;
struct DensityRecord { double start; double end; uint32_t nodes; };
static std::vector<DensityRecord> densityLog;

// ==========================================================
//  RSSI/SNR/BER Tracer
// ==========================================================
static void
RssiSnrBerTracer(uint32_t randSeed, std::string runTag, std::ofstream *csv,
                 Ptr<const Packet> pkt, uint16_t channelFreqMhz, WifiTxVector txVector,
                 MpduInfo mpduInfo, SignalNoiseDbm signalNoise, uint16_t staId)
{
  if (!csv || !csv->good()) return;
  double rssi = signalNoise.signal;
  double noise = signalNoise.noise;
  double snr = rssi - noise;
  double snrLinear = std::pow(10.0, snr / 10.0);
  double ber = 0.5 * std::erfc(std::sqrt(snrLinear));
  *csv << std::fixed << std::setprecision(6)
       << Simulator::Now().GetSeconds() << "," << channelFreqMhz << ","
       << rssi << "," << noise << "," << snr << "," << ber << ","
       << randSeed << "," << runTag << "\n";
}

// ==========================================================
//  Dynamic Node Density Controller
// ==========================================================
void ChangeActiveStations(ApplicationContainer &apps, uint32_t &currentN,
                          uint32_t nMin, uint32_t nMax, uint32_t seed, double interval)
{
  Ptr<UniformRandomVariable> rnd = CreateObject<UniformRandomVariable>();
  rnd->SetStream(seed + (uint32_t)Simulator::Now().GetSeconds() * 17);

  int delta = (int)std::round(rnd->GetValue(-3.0, 3.0));
  int newN = std::max((int)nMin, std::min((int)nMax, (int)currentN + delta));

  double now = Simulator::Now().GetSeconds();

  if (newN != (int)currentN)
  {
    NS_LOG_UNCOND("[" << now << "s] Node density changed: " << currentN << " → " << newN);
    if (!densityLog.empty()) densityLog.back().end = now;
    densityLog.push_back({now, now + interval, (uint32_t)newN});
    currentN = newN;

    for (uint32_t i = 0; i < apps.GetN(); ++i)
    {
      if (i < currentN)
        apps.Get(i)->SetAttribute("StartTime", TimeValue(Seconds(now)));
      else
        apps.Get(i)->SetAttribute("StopTime", TimeValue(Seconds(now)));
    }
  }

  Simulator::Schedule(Seconds(interval), &ChangeActiveStations, std::ref(apps),
                      std::ref(currentN), nMin, nMax, seed + 1, interval);
}

// ==========================================================
//  Setup Wi-Fi BSS
// ==========================================================
struct WifiBss
{
  NodeContainer ap;
  NodeContainer stas;
  NetDeviceContainer apDev;
  NetDeviceContainer staDevs;
  Ipv4InterfaceContainer apIf;
  Ipv4InterfaceContainer staIf;
};

WifiBss SetupWifiBss(std::string subnet, double txPower, double areaHalf,
                     uint32_t nSta, bool isIndoor)
{
  WifiBss bss;
  bss.ap.Create(1);
  bss.stas.Create(nSta);

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
  Ssid ssid = Ssid(subnet);
  mac.SetType("ns3::StaWifiMac", "Ssid", SsidValue(ssid), "ActiveProbing", BooleanValue(false));
  bss.staDevs = wifi.Install(phy, mac, bss.stas);
  mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
  bss.apDev = wifi.Install(phy, mac, bss.ap);

  MobilityHelper mobSta;
  mobSta.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                          "Bounds", RectangleValue(Rectangle(-areaHalf, areaHalf, -areaHalf, areaHalf)));
  mobSta.Install(bss.stas);
  MobilityHelper mobAp;
  mobAp.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobAp.Install(bss.ap);

  InternetStackHelper stack;
  stack.Install(bss.ap);
  stack.Install(bss.stas);

  Ipv4AddressHelper ip;
  ip.SetBase(subnet.c_str(), "255.255.255.0");
  bss.staIf = ip.Assign(bss.staDevs);
  bss.apIf = ip.Assign(bss.apDev);

  return bss;
}

// ==========================================================
//  Install UDP Traffic
// ==========================================================
ApplicationContainer InstallTraffic(WifiBss &bss, uint16_t port, uint32_t packetSize,
                                    double intervalMs, double simTime)
{
  UdpClientHelper client(bss.apIf.GetAddress(0), port);
  client.SetAttribute("MaxPackets", UintegerValue(0));
  client.SetAttribute("Interval", TimeValue(MilliSeconds(intervalMs)));
  client.SetAttribute("PacketSize", UintegerValue(packetSize));

  ApplicationContainer clients;
  for (uint32_t i = 0; i < bss.stas.GetN(); ++i)
    clients.Add(client.Install(bss.stas.Get(i)));

  UdpServerHelper server(port);
  ApplicationContainer serverApp = server.Install(bss.ap.Get(0));

  serverApp.Start(Seconds(1.0));
  clients.Start(Seconds(2.0));
  clients.Stop(Seconds(simTime));
  serverApp.Stop(Seconds(simTime));

  return clients;
}

// ==========================================================
//  Main
// ==========================================================
int main(int argc, char *argv[])
{
  bool isIndoor = true;
  uint32_t nStaMin = 5, nStaMax = 30;
  double areaHalf = 50.0, simTime = 30.0, txPower = 16.0;
  bool enableInterference = true;
  uint32_t seed = 12345;
  uint16_t appPort = 9999;
  uint32_t packetSize = 1024;
  double clientIntervalMs = 10.0;
  double densityChangeInterval = 5.0;

  CommandLine cmd;
  cmd.AddValue("isIndoor", "", isIndoor);
  cmd.AddValue("nStaMin", "", nStaMin);
  cmd.AddValue("nStaMax", "", nStaMax);
  cmd.AddValue("areaHalf", "", areaHalf);
  cmd.AddValue("simTime", "", simTime);
  cmd.AddValue("txPower", "", txPower);
  cmd.AddValue("enableInterference", "", enableInterference);
  cmd.AddValue("packetSize", "", packetSize);
  cmd.AddValue("clientIntervalMs", "", clientIntervalMs);
  cmd.AddValue("seed", "", seed);
  cmd.AddValue("densityChangeInterval", "", densityChangeInterval);
  cmd.Parse(argc, argv);

  // Timestamp for filenames
  time_t now = time(0);
  tm *ltm = localtime(&now);
  std::ostringstream dateSuffix;
  dateSuffix << std::setfill('0') << std::setw(2) << ltm->tm_mday << "-"
             << std::put_time(ltm, "%b") << "-"
             << (1900 + ltm->tm_year) << "_"
             << std::setw(2) << ltm->tm_hour << "-"
             << std::setw(2) << ltm->tm_min;
  std::string runTag = dateSuffix.str();

  std::ostringstream seedStr;
  seedStr << seed;
  std::string csvPrefix = "outputs/csv/wifi-random-" + runTag + "-seed" + seedStr.str();

  // RNG and initial nodes
  RngSeedManager::SetSeed(seed);
  Ptr<UniformRandomVariable> u = CreateObject<UniformRandomVariable>();
  u->SetAttribute("Min", DoubleValue(nStaMin));
  u->SetAttribute("Max", DoubleValue(nStaMax + 1));
  activeNodes = (uint32_t)std::floor(u->GetValue());
  if (activeNodes < 1) activeNodes = 1;

  NS_LOG_UNCOND("\n=== Wi-Fi Scenario ===");
  NS_LOG_UNCOND("isIndoor=" << isIndoor << ", nStaInitial=" << activeNodes
                             << ", txPower=" << txPower << " dBm, seed=" << seed);

  WifiBss mainBss = SetupWifiBss("10.1.3.0", txPower, areaHalf, nStaMax, isIndoor);
  WifiBss intfBss;
  if (enableInterference)
    intfBss = SetupWifiBss("10.1.4.0", txPower, areaHalf, std::max(1u, nStaMax/3), isIndoor);

  ApplicationContainer clientApps = InstallTraffic(mainBss, appPort, packetSize, clientIntervalMs, simTime);
  if (enableInterference)
    InstallTraffic(intfBss, 8888, packetSize, clientIntervalMs, simTime);

  densityLog.push_back({0.0, densityChangeInterval, activeNodes});
  Simulator::Schedule(Seconds(densityChangeInterval),
                      &ChangeActiveStations, std::ref(clientApps),
                      std::ref(activeNodes), nStaMin, nStaMax, seed, densityChangeInterval);

  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  std::ofstream rssiCsv((csvPrefix + "-rssi.csv").c_str());
  rssiCsv << "time_s,channel_MHz,signal_dBm,noise_dBm,SNR_dB,BER,RandSeed,RunDateTime\n";
  Config::ConnectWithoutContext(
      "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
      MakeBoundCallback(&RssiSnrBerTracer, seed, runTag, &rssiCsv));

  Simulator::Stop(Seconds(simTime + 1));
  Simulator::Run();

  // =============================================================
  // PERF.CSV: FlowMonitor Output
  // =============================================================
  monitor->CheckForLostPackets();
  auto stats = monitor->GetFlowStats();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());

  std::ofstream perfCsv((csvPrefix + "-perf.csv").c_str());
  perfCsv << "FlowID,NetworkType,Source,Destination,Throughput(Mbps),Latency_avg(ms),"
             "Jitter_avg(ms),PacketLoss(%),RandSeed,RunDateTime\n";

  for (auto const &kv : stats)
  {
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow(kv.first);
    const FlowMonitor::FlowStats &s = kv.second;
    double dur = (s.timeLastRxPacket - s.timeFirstRxPacket).GetSeconds();
    double thr = (dur>0 && s.rxBytes>0)?(s.rxBytes*8.0/dur)/1e6:0.0;
    double delayMs = (s.rxPackets>0)?(s.delaySum.GetSeconds()/s.rxPackets)*1000.0:0.0;
    double jitterMs = (s.rxPackets>0)?(s.jitterSum.GetSeconds()/s.rxPackets)*1000.0:0.0;
    double loss = (s.txPackets>0)?100.0*(s.txPackets-s.rxPackets)/s.txPackets:0.0;
    std::string netType = (t.sourceAddress.Get() >= 0x0A010300 && t.sourceAddress.Get() < 0x0A010400)
                            ? "MainBSS" : "InterfBSS";

    perfCsv << kv.first << "," << netType << "," << t.sourceAddress << "," << t.destinationAddress
            << "," << thr << "," << delayMs << "," << jitterMs << "," << loss
            << "," << seed << "," << runTag << "\n";
  }
  perfCsv.close();

  // =============================================================
  // NODEDENSITY.CSV: Aggregated Throughput, Jitter, RSSI, SNR, BER
  // =============================================================
  std::vector<std::tuple<double,double,double,double,double>> rssiData;
  {
    std::ifstream rf((csvPrefix + "-rssi.csv").c_str());
    std::string line;
    std::getline(rf, line);
    double t, ch, sig, noise, snr, ber;
    char comma;
    while (rf >> t >> comma >> ch >> comma >> sig >> comma >> noise >> comma >> snr >> comma >> ber)
      rssiData.push_back({t, sig, noise, snr, ber});
  }

  std::ofstream ndCsv((csvPrefix + "-nodedensity.csv").c_str());
  ndCsv << "StartDateTime,EndDateTime,Duration_s,NodeDensity,"
           "TotalThroughput(Mbps),AvgJitter(ms),AvgRSSI(dBm),AvgSNR(dB),AvgBER\n";

  time_t simStart = time(nullptr);
  for (auto &rec : densityLog)
  {
    if (rec.end > simTime) rec.end = simTime;
    double start = rec.start;
    double end = rec.end;
    double dur = end - start;

    double totalThr = 0.0, sumJit = 0.0; int flowCount = 0;
    for (auto &kv : stats)
    {
      const FlowMonitor::FlowStats &s = kv.second;
      if (s.timeLastRxPacket.GetSeconds() >= start && s.timeFirstTxPacket.GetSeconds() <= end)
      {
        double durFlow = (s.timeLastRxPacket - s.timeFirstRxPacket).GetSeconds();
        double thr = (durFlow>0 && s.rxBytes>0)?(s.rxBytes*8.0/durFlow)/1e6:0.0;
        double jit = (s.rxPackets>0)?(s.jitterSum.GetSeconds()/s.rxPackets)*1000.0:0.0;
        totalThr += thr;
        sumJit += jit;
        flowCount++;
      }
    }
    double avgJit = (flowCount>0)?sumJit/flowCount:0.0;

    double sumRssi=0,sumSnr=0,sumBer=0; int rCount=0;
    for (auto &r : rssiData)
    {
      double t = std::get<0>(r);
      if (t >= start && t < end)
      {
        sumRssi += std::get<1>(r);
        sumSnr  += std::get<3>(r);
        sumBer  += std::get<4>(r);
        rCount++;
      }
    }

    double avgRssi = (rCount>0)?sumRssi/rCount:0.0;
    double avgSnr  = (rCount>0)?sumSnr/rCount:0.0;
    double avgBer  = (rCount>0)?sumBer/rCount:0.0;

    char startBuf[64], endBuf[64];
    time_t startTs = simStart + (time_t)start;
    time_t endTs   = simStart + (time_t)end;
    strftime(startBuf, sizeof(startBuf), "%Y-%m-%d %H:%M:%S", localtime(&startTs));
    strftime(endBuf,   sizeof(endBuf),   "%Y-%m-%d %H:%M:%S", localtime(&endTs));

    ndCsv << startBuf << "," << endBuf << ","
          << dur << "," << rec.nodes << ","
          << totalThr << "," << avgJit << ","
          << avgRssi << "," << avgSnr << "," << avgBer << "\n";
  }

  ndCsv.close();
  rssiCsv.close();
  Simulator::Destroy();

  std::cout << "\n✅ Outputs generated:\n"
            << "  → " << csvPrefix << "-perf.csv (FlowMonitor)\n"
            << "  → " << csvPrefix << "-rssi.csv (RSSI/SNR/BER)\n"
            << "  → " << csvPrefix << "-nodedensity.csv (Enriched summary)\n";
}
