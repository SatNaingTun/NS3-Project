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
// RSSI / SNR / BER tracer (YansErrorRateModel)
// ==========================================================
static void
RssiSnrBerTracer(uint32_t randSeed, std::string runTag, std::ofstream *csv,
                 Ptr<const Packet>, uint16_t chMHz, WifiTxVector txVec,
                 MpduInfo, SignalNoiseDbm sn, uint16_t staId)
{
  if (!csv || !csv->good()) return;
  double rssi = sn.signal, noise = sn.noise;
  double snrDb = rssi - noise, snrLin = std::pow(10.0, snrDb / 10.0);

  Ptr<YansErrorRateModel> err = CreateObject<YansErrorRateModel>();
  double succ = err->GetChunkSuccessRate(txVec.GetMode(), txVec, snrLin,
                                         txVec.GetNss() * 8 * 1500,
                                         txVec.GetChannelWidth(),
                                         WIFI_PPDU_FIELD_DATA, staId);
  double ber = 1.0 - succ;
  if (ber < 1e-9) ber = 1e-9;

  *csv << std::fixed << std::setprecision(6)
       << Simulator::Now().GetSeconds() << "," << chMHz << ","
       << rssi << "," << noise << "," << snrDb << "," << ber << ","
       << randSeed << "," << runTag << "\n";
}

// ==========================================================
// Modulation-aware BER tracer (modern NS-3 API)
// ==========================================================
static void
ModulationBerTracer(uint32_t randSeed, std::string runTag, std::ofstream *csv,
                    Ptr<const Packet>, uint16_t channelFreqMhz, WifiTxVector txVector,
                    MpduInfo, SignalNoiseDbm signalNoise, uint16_t staId)
{
  if (!csv || !csv->good()) return;

  double rssi  = signalNoise.signal;
  double noise = signalNoise.noise;
  double snrDb = rssi - noise;
  double snrLinear = std::pow(10.0, snrDb / 10.0);

  Ptr<YansErrorRateModel> errModel = CreateObject<YansErrorRateModel>();
  WifiMode mode = txVector.GetMode();

  // --- Determine modulation class (for labeling) ---
  std::string modulation;
  switch (mode.GetModulationClass()) {
    case WIFI_MOD_CLASS_DSSS:  modulation = "DSSS"; break;
    case WIFI_MOD_CLASS_OFDM:  modulation = "OFDM"; break;
    case WIFI_MOD_CLASS_HT:    modulation = "HT";   break;
    case WIFI_MOD_CLASS_VHT:   modulation = "VHT";  break;
    case WIFI_MOD_CLASS_HE:    modulation = "HE";   break;
    default:                   modulation = "UNKNOWN"; break;
  }

  // --- Use public GetChunkSuccessRate() to approximate BER ---
  uint64_t nbits = txVector.GetNss() * 8 * 1500;
  uint8_t channelWidth = txVector.GetChannelWidth();
  WifiPpduField field = WIFI_PPDU_FIELD_DATA;

  double success = errModel->GetChunkSuccessRate(
      mode, txVector, snrLinear, nbits, channelWidth, field, staId);

  // Approximate bit error rate as probability of failure divided by bits
  double ber = (1.0 - success) / nbits;
  if (ber < 1e-9) ber = 1e-9;

  // --- Add modulation constellation and PHY rate for reference ---
  uint16_t constellation = mode.GetConstellationSize();
  double phyRateMbps = mode.GetDataRate(txVector) / 1e6;

  *csv << std::fixed << std::setprecision(6)
       << Simulator::Now().GetSeconds() << ","
       << channelFreqMhz << ","
       << modulation << ","
       << constellation << ","
       << phyRateMbps << ","
       << rssi << "," << noise << "," << snrDb << "," << ber << ","
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
int main(int argc,char *argv[])
{
  bool indoor=true; uint32_t nMin=5,nMax=30;
  double area=50,sim=30,txP=16; bool inter=true;
  uint32_t seed=12345; uint16_t port=9999;
  uint32_t pkt=1024; double intv=10,change=5;
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
  cmd.Parse(argc,argv);

  time_t now=time(0); tm *lt=localtime(&now);
  std::ostringstream tag; tag<<std::setfill('0')<<std::setw(2)<<lt->tm_mday<<"-"
     <<std::put_time(lt,"%b")<<"-"<<(1900+lt->tm_year)<<"_"
     <<std::setw(2)<<lt->tm_hour<<"-"<<std::setw(2)<<lt->tm_min;
  std::string run=tag.str(), prefix="outputs/csv/wifi-random-"+run+"-seed"+std::to_string(seed);

  RngSeedManager::SetSeed(seed);
  Ptr<UniformRandomVariable> u=CreateObject<UniformRandomVariable>();
  u->SetAttribute("Min",DoubleValue(nMin)); u->SetAttribute("Max",DoubleValue(nMax+1));
  activeNodes=(uint32_t)std::floor(u->GetValue());
  if(activeNodes<1)activeNodes=1;
  WifiBss main=SetupWifiBss("10.1.3.0",txP,area,nMax,indoor);
  WifiBss intf;
  if(inter) intf=SetupWifiBss("10.1.4.0",txP,area,std::max(1u,nMax/3),indoor);

  ApplicationContainer apps=InstallTraffic(main,port,pkt,intv,sim);
  if(inter) InstallTraffic(intf,8888,pkt,intv,sim);
  densityLog.push_back({0.0,change,activeNodes});
  Simulator::Schedule(Seconds(change),&ChangeActiveStations,std::ref(apps),
                      std::ref(activeNodes),nMin,nMax,seed,change);

  FlowMonitorHelper flowmon; Ptr<FlowMonitor> mon=flowmon.InstallAll();

  std::ofstream rssi((prefix+"-rssi.csv").c_str());
  rssi<<"time_s,channel_MHz,signal_dBm,noise_dBm,SNR_dB,BER,RandSeed,RunDateTime\n";
  Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
      MakeBoundCallback(&RssiSnrBerTracer,seed,run,&rssi));
  std::ofstream mod((prefix+"-modulation.csv").c_str());
  mod<<"time_s,channel_MHz,Modulation,ConstellationSize,PhyRate_Mbps,signal_dBm,noise_dBm,SNR_dB,BER,RandSeed,RunDateTime\n";
  Config::ConnectWithoutContext("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
      MakeBoundCallback(&ModulationBerTracer,seed,run,&mod));

  Simulator::Stop(Seconds(sim+1)); Simulator::Run();

  // ------------------- PERF.CSV -------------------
  mon->CheckForLostPackets();
  auto stats=mon->GetFlowStats();
  Ptr<Ipv4FlowClassifier> cls=DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());
  std::ofstream perf((prefix+"-perf.csv").c_str());
  perf<<"FlowID,Source,Destination,Throughput(Mbps),Latency_avg(ms),Jitter_avg(ms),PacketLoss(%)\n";
  for(auto &kv:stats){
    Ipv4FlowClassifier::FiveTuple t=cls->FindFlow(kv.first);
    const FlowMonitor::FlowStats &s=kv.second;
    double dur=(s.timeLastRxPacket-s.timeFirstRxPacket).GetSeconds();
    double thr=(dur>0&&s.rxBytes>0)?(s.rxBytes*8.0/dur)/1e6:0;
    double delay=(s.rxPackets>0)?(s.delaySum.GetSeconds()/s.rxPackets)*1000:0;
    double jit=(s.rxPackets>0)?(s.jitterSum.GetSeconds()/s.rxPackets)*1000:0;
    double loss=(s.txPackets>0)?100.0*(s.txPackets-s.rxPackets)/s.txPackets:0;
    perf<<kv.first<<","<<t.sourceAddress<<","<<t.destinationAddress<<","
        <<thr<<","<<delay<<","<<jit<<","<<loss<<"\n";
  }

  // ------------------- NODEDENSITY.CSV -------------------
  std::vector<std::tuple<double,double,double,double,double>> rData;
  { std::ifstream rf((prefix+"-rssi.csv").c_str()); std::string line; std::getline(rf,line);
    double t,ch,sig,noi,snr,ber; char c;
    while(rf>>t>>c>>ch>>c>>sig>>c>>noi>>c>>snr>>c>>ber)
      rData.push_back({t,sig,noi,snr,ber});
  }
  std::ofstream nd((prefix+"-nodedensity.csv").c_str());
  nd<<"StartDateTime,EndDateTime,Duration_s,NodeDensity,"
       "TotalTxPackets,TotalRxPackets,TotalThroughput(Mbps),"
       "TotalPacketLoss(%),AvgJitter(ms),AvgRSSI(dBm),AvgSNR(dB),AvgBER\n";
  time_t base=time(nullptr);
  for(auto &r:densityLog){
    if(r.end>sim)r.end=sim;
    double st=r.start,en=r.end,dur=en-st;
    double thr=0,jitSum=0; uint64_t tx=0,rx=0; int n=0;
    for(auto &kv:stats){
      const FlowMonitor::FlowStats &s=kv.second;
      if(s.timeLastRxPacket.GetSeconds()>=st&&s.timeFirstTxPacket.GetSeconds()<=en){
        double d=(s.timeLastRxPacket-s.timeFirstRxPacket).GetSeconds();
        double t=(d>0&&s.rxBytes>0)?(s.rxBytes*8.0/d)/1e6:0;
        double j=(s.rxPackets>0)?(s.jitterSum.GetSeconds()/s.rxPackets)*1000:0;
        thr+=t;jitSum+=j;tx+=s.txPackets;rx+=s.rxPackets;n++;
      }}
    double avgJ=(n>0)?jitSum/n:0;
    double loss=(tx>0)?100.0*(double)(tx-rx)/tx:0;
    double sR=0,sS=0,sB=0;int rc=0;
    for(auto &x:rData){double t=std::get<0>(x);
      if(t>=st&&t<en){sR+=std::get<1>(x);sS+=std::get<3>(x);sB+=std::get<4>(x);rc++;}}
    double aR=(rc>0)?sR/rc:0,aS=(rc>0)?sS/rc:0,aB=(rc>0)?sB/rc:0;
    char sb[64],eb[64]; time_t sT=base+(time_t)st,eT=base+(time_t)en;
    strftime(sb,sizeof(sb),"%Y-%m-%d %H:%M:%S",localtime(&sT));
    strftime(eb,sizeof(eb),"%Y-%m-%d %H:%M:%S",localtime(&eT));
    nd<<sb<<","<<eb<<","<<dur<<","<<r.nodes<<","<<tx<<","<<rx<<","<<thr<<","
      <<loss<<","<<avgJ<<","<<aR<<","<<aS<<","<<aB<<"\n";
  }

  nd.close(); perf.close(); rssi.close(); mod.close();
  Simulator::Destroy();
  std::cout<<"\n✅ Outputs generated:\n"
           <<"  → "<<prefix<<"-perf.csv\n"
           <<"  → "<<prefix<<"-rssi.csv\n"
           <<"  → "<<prefix<<"-modulation.csv\n"
           <<"  → "<<prefix<<"-nodedensity.csv\n";
}
