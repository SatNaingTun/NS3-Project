#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/netanim-module.h"
#include "ns3/wifi-ppdu.h"

#include <fstream>
#include <iomanip>
#include <ctime>
#include <sstream>
#include <cmath>
#include <vector>

using namespace ns3;
NS_LOG_COMPONENT_DEFINE("WifiPerfRandom");

// ==========================================================
// Global variables
// ==========================================================
static uint32_t activeNodes;
struct DensityRecord { double start; double end; uint32_t nodes; };
static std::vector<DensityRecord> densityLog;

// ==========================================================
//  RSSI / SNR / BER Tracer (real BER + BER in dB, no fake floor)
// ==========================================================
static void
RssiSnrBerTracer(uint32_t randSeed, std::string runTag, std::ofstream *csv,
                 Ptr<const Packet>, uint16_t channelFreqMhz, WifiTxVector txVector,
                 MpduInfo, SignalNoiseDbm signalNoise, uint16_t staId)
{
  if (!csv || !csv->good()) return;

  // --- PHY parameters ---
  double rssi  = signalNoise.signal;
  double noise = signalNoise.noise;
  double snrDb = rssi - noise;
  double snrLinear = std::pow(10.0, snrDb / 10.0);

  // --- NS-3 error model ---
  Ptr<YansErrorRateModel> errModel = CreateObject<YansErrorRateModel>();
  WifiMode mode = txVector.GetMode();

  uint64_t nbits = txVector.GetNss() * 8 * 1500;
  uint8_t channelWidth = txVector.GetChannelWidth();
  WifiPpduField field = WIFI_PPDU_FIELD_DATA;

  // --- Success rate → BER ---
  double success = errModel->GetChunkSuccessRate(
      mode, txVector, snrLinear, nbits, channelWidth, field, staId);

  double berLinear = (1.0 - success) / nbits;
  if (berLinear <= 0.0)
      berLinear = std::numeric_limits<double>::min();  // prevent log(0)
  double berDb = 10.0 * std::log10(berLinear);

  // --- Write both linear and dB forms ---
  *csv << std::scientific << std::setprecision(8)
       << Simulator::Now().GetSeconds() << ","
       << channelFreqMhz << ","
       << rssi << "," << noise << "," << snrDb << ","
       << berLinear << "," << berDb << ","
       << randSeed << "," << runTag << "\n";
}


// ==========================================================
//  Modulation-Aware BER Tracer (real BER + BER in dB, no fake floor)
// ==========================================================
static void
ModulationBerTracer(uint32_t randSeed, std::string runTag, std::ofstream *csv,
                    Ptr<const Packet>, uint16_t channelFreqMhz, WifiTxVector txVector,
                    MpduInfo, SignalNoiseDbm signalNoise, uint16_t staId)
{
  if (!csv || !csv->good()) return;

  // --- PHY signal parameters ---
  double rssi  = signalNoise.signal;
  double noise = signalNoise.noise;
  double snrDb = rssi - noise;
  double snrLinear = std::pow(10.0, snrDb / 10.0);

  // --- NS-3 Error Rate Model ---
  Ptr<YansErrorRateModel> errModel = CreateObject<YansErrorRateModel>();
  WifiMode mode = txVector.GetMode();

  // --- Modulation class name ---
  std::string modulation;
  switch (mode.GetModulationClass()) {
    case WIFI_MOD_CLASS_DSSS:  modulation = "DSSS"; break;
    case WIFI_MOD_CLASS_OFDM:  modulation = "OFDM"; break;
    case WIFI_MOD_CLASS_HT:    modulation = "HT";   break;
    case WIFI_MOD_CLASS_VHT:   modulation = "VHT";  break;
    case WIFI_MOD_CLASS_HE:    modulation = "HE";   break;
    default:                   modulation = "UNKNOWN"; break;
  }

  // --- Compute success probability and BER ---
  uint64_t nbits = txVector.GetNss() * 8 * 1500;
  uint8_t channelWidth = txVector.GetChannelWidth();
  WifiPpduField field = WIFI_PPDU_FIELD_DATA;

  double success = errModel->GetChunkSuccessRate(
      mode, txVector, snrLinear, nbits, channelWidth, field, staId);

  double berLinear = (1.0 - success) / nbits;

  // Avoid -inf in log10() when success=1.0 (no errors)
  if (berLinear <= 0.0)
      berLinear = std::numeric_limits<double>::min();

  double berDb = 10.0 * std::log10(berLinear);  // convert to dB scale

  // --- PHY rate and constellation ---
  uint16_t constellation = mode.GetConstellationSize();
  double phyRateMbps = mode.GetDataRate(txVector) / 1e6;

  // --- Write both linear and dB BER to CSV ---
  *csv << std::scientific << std::setprecision(8)
       << Simulator::Now().GetSeconds() << ","
       << channelFreqMhz << ","
       << modulation << ","
       << constellation << ","
       << phyRateMbps << ","
       << rssi << "," << noise << "," << snrDb << ","
       << berLinear << "," << berDb << ","     // both forms
       << randSeed << "," << runTag << "\n";
}


// ==========================================================
// Dynamic node-density controller
// ==========================================================
void ChangeActiveStations(ApplicationContainer &apps, uint32_t &curN,
                          uint32_t nMin, uint32_t nMax, uint32_t seed, double interval)
{
  Ptr<UniformRandomVariable> rnd = CreateObject<UniformRandomVariable>();
  rnd->SetStream(seed + (uint32_t)Simulator::Now().GetSeconds() * 17);
  int delta = (int)std::round(rnd->GetValue(-3.0, 3.0));
  int newN = std::max((int)nMin, std::min((int)nMax, (int)curN + delta));
  double now = Simulator::Now().GetSeconds();

  if (newN != (int)curN) {
    NS_LOG_UNCOND("["<<now<<"s] Node density changed: "<<curN<<" → "<<newN);
    if (!densityLog.empty()) densityLog.back().end = now;
    densityLog.push_back({now, now + interval, (uint32_t)newN});
    curN = newN;
    for (uint32_t i=0;i<apps.GetN();++i)
      apps.Get(i)->SetAttribute(i<curN?"StartTime":"StopTime", TimeValue(Seconds(now)));
  }
  Simulator::Schedule(Seconds(interval), &ChangeActiveStations,
                      std::ref(apps), std::ref(curN),
                      nMin, nMax, seed + 1, interval);
}

// ==========================================================
// Wi-Fi BSS setup
// ==========================================================
struct WifiBss {
  NodeContainer ap, stas;
  NetDeviceContainer apDev, staDevs;
  Ipv4InterfaceContainer apIf, staIf;
};
WifiBss SetupWifiBss(std::string subnet,double txPower,double areaHalf,
                     uint32_t nSta,bool indoor)
{
  WifiBss b;
  b.ap.Create(1); b.stas.Create(nSta);
  WifiHelper wifi; wifi.SetStandard(WIFI_STANDARD_80211ac);
  YansWifiChannelHelper ch = YansWifiChannelHelper::Default();
  if (indoor) {
    ch.AddPropagationLoss("ns3::LogDistancePropagationLossModel","Exponent",DoubleValue(3.0));
    ch.AddPropagationLoss("ns3::NakagamiPropagationLossModel");
  } else {
    ch.AddPropagationLoss("ns3::FriisPropagationLossModel");
    ch.AddPropagationLoss("ns3::NakagamiPropagationLossModel");
  }
  YansWifiPhyHelper phy; phy.SetChannel(ch.Create());
  phy.Set("TxPowerStart",DoubleValue(txPower));
  phy.Set("TxPowerEnd",DoubleValue(txPower));
  WifiMacHelper mac; Ssid ssid=Ssid(subnet);
  mac.SetType("ns3::StaWifiMac","Ssid",SsidValue(ssid),"ActiveProbing",BooleanValue(false));
  b.staDevs=wifi.Install(phy,mac,b.stas);
  mac.SetType("ns3::ApWifiMac","Ssid",SsidValue(ssid));
  b.apDev=wifi.Install(phy,mac,b.ap);
  MobilityHelper ms; ms.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
        "Bounds",RectangleValue(Rectangle(-areaHalf,areaHalf,-areaHalf,areaHalf)));
  ms.Install(b.stas);
  MobilityHelper ma; ma.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  ma.Install(b.ap);
  InternetStackHelper stack; stack.Install(b.ap); stack.Install(b.stas);
  Ipv4AddressHelper ip; ip.SetBase(subnet.c_str(),"255.255.255.0");
  b.staIf=ip.Assign(b.staDevs); b.apIf=ip.Assign(b.apDev);
  return b;
}

// ==========================================================
// UDP traffic
// ==========================================================
ApplicationContainer InstallTraffic(WifiBss &b,uint16_t port,
                                    uint32_t pktSz,double intv,double simTime)
{
  UdpClientHelper client(b.apIf.GetAddress(0),port);
  client.SetAttribute("MaxPackets",UintegerValue(0));
  client.SetAttribute("Interval",TimeValue(MilliSeconds(intv)));
  client.SetAttribute("PacketSize",UintegerValue(pktSz));
  ApplicationContainer c;
  for(uint32_t i=0;i<b.stas.GetN();++i) c.Add(client.Install(b.stas.Get(i)));
  UdpServerHelper srv(port); auto s=srv.Install(b.ap.Get(0));
  s.Start(Seconds(1.0)); c.Start(Seconds(2.0));
  c.Stop(Seconds(simTime)); s.Stop(Seconds(simTime));
  return c;
}

// ==========================================================
// Main
// ==========================================================
int main(int argc, char* argv[])
{
  bool indoor=true; uint32_t nMin=5, nMax=30;
  double area=50.0, sim=30.0, txP=16.0;
  bool inter=true; uint32_t seed=12345;
  uint16_t port=9999; uint32_t pkt=1024; double intv=10.0;
  double change=5.0;

  CommandLine cmd;
  cmd.AddValue("isIndoor","",indoor);
  cmd.AddValue("nStaMin","",nMin);
  cmd.AddValue("nStaMax","",nMax);
  cmd.AddValue("areaHalf","",area);
  cmd.AddValue("simTime","",sim);
  cmd.AddValue("txPower","",txP);
  cmd.AddValue("enableInterference","",inter);
  cmd.AddValue("packetSize","",pkt);
  cmd.AddValue("clientIntervalMs","",intv);
  cmd.AddValue("seed","",seed);
  cmd.AddValue("densityChangeInterval","",change);
  cmd.Parse(argc, argv);

  time_t now=time(0); tm* lt=localtime(&now);
  std::ostringstream tag;
  tag<<std::setfill('0')<<std::setw(2)<<lt->tm_mday<<"-"<<std::put_time(lt,"%b")<<"-"<<(1900+lt->tm_year)
     <<"_"<<std::setw(2)<<lt->tm_hour<<"-"<<std::setw(2)<<lt->tm_min;
  std::string run = tag.str();
  std::string prefix = "outputs/csv/wifi-random-"+run+"-seed"+std::to_string(seed);

  RngSeedManager::SetSeed(seed);
  Ptr<UniformRandomVariable> u=CreateObject<UniformRandomVariable>();
  u->SetAttribute("Min", DoubleValue(nMin));
  u->SetAttribute("Max", DoubleValue(nMax+1));
  activeNodes = (uint32_t)std::floor(u->GetValue());
  if(activeNodes<1) activeNodes=1;

  WifiBss main = SetupWifiBss("10.1.3.0", txP, area, nMax, indoor);
  WifiBss intf;
  if (inter) intf = SetupWifiBss("10.1.4.0", txP, area, std::max(1u, nMax/3), indoor);

  ApplicationContainer apps = InstallTraffic(main, port, pkt, intv, sim);
  if (inter) InstallTraffic(intf, 8888, pkt, intv, sim);

  densityLog.push_back({0.0, change, activeNodes});
  Simulator::Schedule(Seconds(change), &ChangeActiveStations,
                      std::ref(apps), std::ref(activeNodes),
                      nMin, nMax, seed, change);

  FlowMonitorHelper flowmon; Ptr<FlowMonitor> monitor = flowmon.InstallAll();

  std::ofstream rssi((prefix+"-rssi.csv").c_str());
  rssi << "time_s,channel_MHz,signal_dBm,noise_dBm,SNR_dB,BER,RandSeed,RunDateTime\n";
  Config::ConnectWithoutContext(
      "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
      MakeBoundCallback(&RssiSnrBerTracer, seed, run, &rssi));

  std::ofstream mod((prefix+"-modulation.csv").c_str());
  mod << "time_s,channel_MHz,Modulation,ConstellationSize,PhyRate_Mbps,signal_dBm,noise_dBm,SNR_dB,BER,RandSeed,RunDateTime\n";
  Config::ConnectWithoutContext(
      "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
      MakeBoundCallback(&ModulationBerTracer, seed, run, &mod));

  Simulator::Stop(Seconds(sim+1));
  Simulator::Run();

  // ===================== POST-PROCESSING =====================
  monitor->CheckForLostPackets();
  auto stats = monitor->GetFlowStats();
  Ptr<Ipv4FlowClassifier> cls = DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());

  // ---- perf.csv ----
  std::ofstream perf((prefix+"-perf.csv").c_str());
  perf << "FlowID,Source,Destination,Throughput(Mbps),Latency_avg(ms),Jitter_avg(ms),PacketLoss(%)\n";
  for (auto &kv : stats) {
    Ipv4FlowClassifier::FiveTuple t = cls->FindFlow(kv.first);
    const FlowMonitor::FlowStats &s = kv.second;
    double dur = (s.timeLastRxPacket - s.timeFirstRxPacket).GetSeconds();
    double thr = (dur>0 && s.rxBytes>0)? (s.rxBytes*8.0/dur)/1e6 : 0.0;
    double delay = (s.rxPackets>0)? (s.delaySum.GetSeconds()/s.rxPackets)*1000.0 : 0.0;
    double jit = (s.rxPackets>0)? (s.jitterSum.GetSeconds()/s.rxPackets)*1000.0 : 0.0;
    double loss = (s.txPackets>0)? 100.0*(s.txPackets - s.rxPackets)/s.txPackets : 0.0;
    perf << kv.first << "," << t.sourceAddress << "," << t.destinationAddress << ","
         << thr << "," << delay << "," << jit << "," << loss << "\n";
  }
  perf.close();

  // ---- load modulation.csv for realistic BER aggregation ----
  std::vector<std::tuple<double,double,double,double,double>> modData; // t, rssi, noise, snr, ber
  {
    std::ifstream mf((prefix+"-modulation.csv").c_str());
    std::string line; std::getline(mf, line);
    double t,ch,constell,rate,sig,noi,snr,ber; char comma;
    while (mf >> t >> comma >> ch >> comma) {
      std::string modName; std::getline(mf, modName, ','); // skip string safely
      mf >> constell >> comma >> rate >> comma >> sig >> comma >> noi >> comma >> snr >> comma >> ber;
      modData.push_back({t, sig, noi, snr, ber});
    }
  }

  // ---- nodedensity.csv (loss-aware AvgBER from modulation.csv) ----
  std::ofstream nd((prefix+"-nodedensity.csv").c_str());
  nd << "StartDateTime,EndDateTime,Duration_s,NodeDensity,"
        "TotalTxPackets,TotalRxPackets,TotalThroughput(Mbps),"
        "TotalPacketLoss(%),AvgJitter(ms),AvgRSSI(dBm),AvgSNR(dB),AvgBER\n";

  time_t base = time(nullptr);
  for (auto &r : densityLog) {
    if (r.end > sim) r.end = sim;
    const double st=r.start, en=r.end, dur=en-st;

    double sumThr=0.0, sumJit=0.0; uint64_t totTx=0, totRx=0; int flows=0;
    for (auto &kv : stats) {
      const FlowMonitor::FlowStats &s = kv.second;
      if (s.timeLastRxPacket.GetSeconds()>=st && s.timeFirstTxPacket.GetSeconds()<=en) {
        double d = (s.timeLastRxPacket - s.timeFirstRxPacket).GetSeconds();
        double thr = (d>0 && s.rxBytes>0)? (s.rxBytes*8.0/d)/1e6 : 0.0;
        double jit = (s.rxPackets>0)? (s.jitterSum.GetSeconds()/s.rxPackets)*1000.0 : 0.0;
        sumThr += thr; sumJit += jit; totTx += s.txPackets; totRx += s.rxPackets; ++flows;
      }
    }
    const double avgJ = (flows>0)? sumJit/flows : 0.0;
    const double lossPct = (totTx>0)? 100.0*(double)(totTx - totRx)/totTx : 0.0;

    double sRssi=0, sSnr=0, sBer=0; int rc=0;
    for (auto &x : modData) {
      double t = std::get<0>(x);
      if (t>=st && t<en) { sRssi+=std::get<1>(x); sSnr+=std::get<3>(x); sBer+=std::get<4>(x); ++rc; }
    }
    const double aRssi = (rc>0)? sRssi/rc : 0.0;
    const double aSnr  = (rc>0)? sSnr/rc  : 0.0;
    // loss-aware BER: include PHY BER and inflate by MAC loss
    const double aBer  = (totRx>0 && rc>0)? (sBer/rc)*(1.0 + lossPct/100.0) : 0.0;

    char sb[64], eb[64];
    time_t sT = base + (time_t)st, eT = base + (time_t)en;
    strftime(sb,sizeof(sb),"%Y-%m-%d %H:%M:%S",localtime(&sT));
    strftime(eb,sizeof(eb),"%Y-%m-%d %H:%M:%S",localtime(&eT));

    nd << sb << "," << eb << "," << dur << "," << r.nodes << ","
       << totTx << "," << totRx << "," << sumThr << "," << lossPct << ","
       << avgJ << "," << aRssi << "," << aSnr << "," << aBer << "\n";
  }
  nd.close();

  rssi.close(); mod.close();
  Simulator::Destroy();

  std::cout << "\n✅ Outputs generated:\n"
            << "  → " << prefix << "-perf.csv\n"
            << "  → " << prefix << "-rssi.csv\n"
            << "  → " << prefix << "-modulation.csv\n"
            << "  → " << prefix << "-nodedensity.csv\n";
}
