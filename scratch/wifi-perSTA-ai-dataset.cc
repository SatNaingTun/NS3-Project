#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"

#include <fstream>
#include <map>
#include <vector>

using namespace ns3;

/* =============================
 * Scenario definition
 * ============================= */
enum ScenarioType { LOS = 0, MID = 1, NLOS = 2 };

static std::string
ScenarioLabel (ScenarioType s)
{
  return (s == LOS ? "LOS" : (s == MID ? "MID" : "NLOS"));
}

/* =============================
 * Time-series storage
 * ============================= */
struct TimeSample
{
  double time;
  double throughputMbps;
  double delayMs;
};

static std::map<uint32_t, std::vector<TimeSample>> g_timeSeries;

/* =============================
 * Periodic FlowMonitor sampler
 * ============================= */
static void
SampleStats (Ptr<FlowMonitor> monitor,
             Ptr<Ipv4FlowClassifier> classifier,
             double interval,
             uint32_t nSta)
{
  monitor->CheckForLostPackets ();
  auto stats = monitor->GetFlowStats ();

  for (const auto &kv : stats)
  {
    FlowId flowId = kv.first;
    const FlowMonitor::FlowStats &st = kv.second;

    if (st.rxPackets == 0)
      continue;

    Ipv4FlowClassifier::FiveTuple tuple =
      classifier->FindFlow (flowId);

    /* STA IPs: 10.1.1.(2 + staId) */
    uint32_t src = tuple.sourceAddress.Get ();
    uint32_t staId = src - 0x0A010102; // 10.1.1.2

    if (staId >= nSta)
      continue;

    double throughput =
      (st.rxBytes * 8.0) /
      (Simulator::Now ().GetSeconds () * 1e6);

    double delay =
      (st.delaySum.GetSeconds () / st.rxPackets) * 1000.0;

    g_timeSeries[staId].push_back ({
      Simulator::Now ().GetSeconds (),
      throughput,
      delay
    });
  }

  Simulator::Schedule (
    Seconds (interval),
    &SampleStats,
    monitor,
    classifier,
    interval,
    nSta);
}

/* =============================
 * One simulation run
 * ============================= */
static void
RunSimulation (ScenarioType scenario,
               uint32_t seed,
               uint32_t nSta,
               double simTime,
               double sampleInterval)
{
  RngSeedManager::SetSeed (seed);

  NodeContainer ap, sta, interferer;
  ap.Create (1);
  sta.Create (nSta);

  if (scenario == MID)
    interferer.Create (1);

  MobilityHelper mob;
  mob.SetMobilityModel ("ns3::ConstantPositionMobilityModel");

  mob.Install (ap);
  mob.Install (sta);
  if (scenario == MID)
    mob.Install (interferer);

  ap.Get (0)->GetObject<MobilityModel> ()
    ->SetPosition (Vector (0, 0, 1.5));

  for (uint32_t i = 0; i < nSta; ++i)
    sta.Get (i)->GetObject<MobilityModel> ()
      ->SetPosition (Vector (8 + i, 0, 1.5));

  if (scenario == MID)
    interferer.Get (0)->GetObject<MobilityModel> ()
      ->SetPosition (Vector (4, 0, 1.5));

  Ptr<PropagationLossModel> loss;

  if (scenario == LOS)
  {
    auto m = CreateObject<LogDistancePropagationLossModel> ();
    m->SetPathLossExponent (2.0);
    loss = m;
  }
  else if (scenario == NLOS)
  {
    auto m = CreateObject<LogDistancePropagationLossModel> ();
    m->SetPathLossExponent (3.3);

    auto sh = CreateObject<RandomPropagationLossModel> ();
    sh->SetAttribute (
      "Variable",
      StringValue ("ns3::NormalRandomVariable[Mean=-7|Variance=4]"));

    m->SetNext (sh);
    loss = m;
  }
  else
  {
    auto m = CreateObject<LogDistancePropagationLossModel> ();
    m->SetPathLossExponent (2.7);

    auto nak = CreateObject<NakagamiPropagationLossModel> ();
    nak->SetAttribute ("m0", DoubleValue (0.7));

    m->SetNext (nak);
    loss = m;
  }

  Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel> ();
  channel->SetPropagationLossModel (loss);
  channel->SetPropagationDelayModel (
    CreateObject<ConstantSpeedPropagationDelayModel> ());

  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211ac);

  YansWifiPhyHelper phy;
  phy.SetChannel (channel);

  WifiMacHelper mac;
  Ssid ssid ("ai-dataset");

  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid));
  NetDeviceContainer apDev = wifi.Install (phy, mac, ap);

  mac.SetType ("ns3::StaWifiMac",
               "Ssid", SsidValue (ssid),
               "ActiveProbing", BooleanValue (false));
  NetDeviceContainer staDev = wifi.Install (phy, mac, sta);

  InternetStackHelper internet;
  internet.Install (ap);
  internet.Install (sta);
  if (scenario == MID)
    internet.Install (interferer);

  Ipv4AddressHelper ip;
  ip.SetBase ("10.1.1.0", "255.255.255.0");
  ip.Assign (apDev);
  ip.Assign (staDev);

  UdpServerHelper server (5000);
  server.Install (ap.Get (0));

  UdpClientHelper client (Ipv4Address ("10.1.1.1"), 5000);
  client.SetAttribute ("Interval", TimeValue (MilliSeconds (10)));
  client.SetAttribute ("PacketSize", UintegerValue (1024));
  client.SetAttribute ("MaxPackets", UintegerValue (1000000));

  for (uint32_t i = 0; i < nSta; ++i)
    client.Install (sta.Get (i));

  FlowMonitorHelper fm;
  Ptr<FlowMonitor> monitor = fm.InstallAll ();
  Ptr<Ipv4FlowClassifier> classifier =
    DynamicCast<Ipv4FlowClassifier> (fm.GetClassifier ());

  Simulator::Schedule (
    Seconds (sampleInterval),
    &SampleStats,
    monitor,
    classifier,
    sampleInterval,
    nSta);

  Simulator::Stop (Seconds (simTime));
  Simulator::Run ();
  Simulator::Destroy ();
}

/* =============================
 * Dataset export
 * ============================= */
static void
ExportDataset (uint32_t seed, ScenarioType scenario)
{
  std::ofstream out ("ai_time_series.csv", std::ios::app);

  for (const auto &kv : g_timeSeries)
  {
    for (const auto &s : kv.second)
    {
      out << seed << ","
          << kv.first << ","
          << s.time << ","
          << s.throughputMbps << ","
          << s.delayMs << ","
          << ScenarioLabel (scenario)
          << std::endl;
    }
  }

  out.close ();
  g_timeSeries.clear ();
}

/* =============================
 * Main: automation
 * ============================= */
int
main ()
{
  uint32_t nSta = 5;
  double simTime = 20.0;
  double sampleInterval = 0.1;
  uint32_t seeds[] = {1, 2, 3, 4, 5};

  for (ScenarioType s : {LOS, MID, NLOS})
  {
    for (uint32_t seed : seeds)
    {
      RunSimulation (s, seed, nSta, simTime, sampleInterval);
      ExportDataset (seed, s);
    }
  }
  return 0;
}
