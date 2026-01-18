#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"

#include <fstream>
#include <map>
#include <string>

using namespace ns3;

/* ================= PARAMETERS ================= */
static const double SIM_TIME = 30.0;
static const double WINDOW   = 0.02;   // 20 ms
static const double LOS_D1   = 10.0;
static const double LOS_D2   = 25.0;

/* ================= LOS STATE ================= */
static std::string
GetLosState (Ptr<MobilityModel> ap, Ptr<MobilityModel> sta)
{
  double d = ap->GetDistanceFrom (sta);
  if (d < LOS_D1) return "LOS";
  if (d < LOS_D2) return "MID";
  return "NLOS";
}

/* ================= WINDOW STATS ================= */
struct WindowStats
{
  uint64_t rxBytes = 0;
  uint64_t txPackets = 0;
  uint64_t rxPackets = 0;
  uint64_t lostPackets = 0;
  Time delaySum = Seconds (0);
  Time jitterSum = Seconds (0);
};

static std::map<uint32_t, WindowStats> g_window;

/* ================= WINDOW SAMPLER ================= */
static void
SampleWindow (Ptr<FlowMonitor> fm,
              Ptr<Ipv4FlowClassifier> classifier,
              NodeContainer sta,
              Ptr<Node> ap,
              uint32_t nSta,
              std::ofstream *out)
{
  fm->CheckForLostPackets ();

  for (const auto &kv : fm->GetFlowStats ())
  {
    const FlowMonitor::FlowStats &st = kv.second;
    if (st.txPackets == 0) continue;

    auto tuple = classifier->FindFlow (kv.first);

    bool uplink =
      (tuple.destinationAddress == Ipv4Address ("10.1.1.1"));

    uint32_t staId =
      uplink
        ? (tuple.sourceAddress.Get ()
           - Ipv4Address ("10.1.1.2").Get ())
        : (tuple.destinationAddress.Get ()
           - Ipv4Address ("10.1.1.2").Get ());

    if (staId >= nSta) continue;

    auto &w = g_window[staId];
    w.rxBytes     += st.rxBytes;
    w.txPackets   += st.txPackets;
    w.rxPackets   += st.rxPackets;
    w.lostPackets += st.lostPackets;
    w.delaySum    += st.delaySum;
    w.jitterSum   += st.jitterSum;
  }

  for (uint32_t i = 0; i < nSta; ++i)
  {
    auto &w = g_window[i];

    double throughput =
      (w.rxBytes * 8.0) / (WINDOW * 1e6);

    double delay =
      (w.rxPackets > 0)
        ? (w.delaySum.GetSeconds () / w.rxPackets) * 1000.0
        : 0.0;

    double jitter =
      (w.rxPackets > 1)
        ? (w.jitterSum.GetSeconds () / (w.rxPackets - 1)) * 1000.0
        : 0.0;

    double loss =
      (w.txPackets > 0)
        ? static_cast<double>(w.lostPackets) / w.txPackets
        : 0.0;

    std::string los =
      GetLosState (
        ap->GetObject<MobilityModel> (),
        sta.Get (i)->GetObject<MobilityModel> ());

    (*out)
      << Simulator::Now ().GetSeconds () << ","
      << i << ","
      << nSta << ","
      << los << ","
      << throughput << ","
      << delay << ","
      << jitter << ","
      << loss << "\n";

    w = WindowStats ();
  }

  Simulator::Schedule (
    Seconds (WINDOW),
    &SampleWindow, fm, classifier,
    sta, ap, nSta, out);
}

/* ================= MAIN ================= */
int main ()
{
  /* Retry limit → packet loss under burst */
  Config::SetDefault (
    "ns3::WifiMac::FrameRetryLimit",
    UintegerValue (3));

  uint32_t nSta = 20;

  NodeContainer ap;
  ap.Create (1);

  NodeContainer sta;
  sta.Create (nSta);

  /* ---------- Mobility ---------- */
  MobilityHelper mobility;
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (ap);

  ap.Get (0)->GetObject<MobilityModel> ()
    ->SetPosition (Vector (0, 0, 1.5));

  mobility.SetMobilityModel (
    "ns3::RandomWalk2dMobilityModel",
    "Bounds", RectangleValue (Rectangle (-40, 40, -40, 40)));
  mobility.Install (sta);

  /* ---------- Wi-Fi + LOS/NLOS ---------- */
  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211ac);

  Ptr<LogDistancePropagationLossModel> logLoss =
    CreateObject<LogDistancePropagationLossModel> ();
  logLoss->SetReference (1.0, 46.6777);

  Ptr<NakagamiPropagationLossModel> nakagami =
    CreateObject<NakagamiPropagationLossModel> ();
  nakagami->SetAttribute ("m0", DoubleValue (1.0));
  nakagami->SetAttribute ("m1", DoubleValue (0.75));
  nakagami->SetAttribute ("m2", DoubleValue (0.5));
  nakagami->SetAttribute ("Distance1", DoubleValue (LOS_D1));
  nakagami->SetAttribute ("Distance2", DoubleValue (LOS_D2));

  logLoss->SetNext (nakagami);

  Ptr<YansWifiChannel> channel =
    CreateObject<YansWifiChannel> ();
  channel->SetPropagationDelayModel (
    CreateObject<ConstantSpeedPropagationDelayModel> ());
  channel->SetPropagationLossModel (logLoss);

  YansWifiPhyHelper phy;
  phy.SetChannel (channel);

  WifiMacHelper mac;
  Ssid ssid ("bursty-los-nlos");

  mac.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid));
  NetDeviceContainer apDev = wifi.Install (phy, mac, ap);

  mac.SetType ("ns3::StaWifiMac",
               "Ssid", SsidValue (ssid),
               "ActiveProbing", BooleanValue (false));
  NetDeviceContainer staDev = wifi.Install (phy, mac, sta);

  /* ---------- Internet ---------- */
  InternetStackHelper internet;
  internet.Install (ap);
  internet.Install (sta);

  Ipv4AddressHelper ip;
  ip.SetBase ("10.1.1.0", "255.255.255.0");
  ip.Assign (apDev);
  ip.Assign (staDev);

  /* ---------- Applications (BURSTY USERS) ---------- */
  UdpServerHelper server (5000);
  server.Install (ap.Get (0));

  for (uint32_t i = 0; i < nSta; ++i)
  {
    OnOffHelper onoff ("ns3::UdpSocketFactory",
      InetSocketAddress (Ipv4Address ("10.1.1.1"), 5000));

    onoff.SetAttribute ("DataRate", StringValue ("50Mbps"));
    onoff.SetAttribute ("PacketSize", UintegerValue (1500));

    onoff.SetAttribute ("OnTime",
      StringValue ("ns3::ExponentialRandomVariable[Mean=0.5]"));
    onoff.SetAttribute ("OffTime",
      StringValue ("ns3::ExponentialRandomVariable[Mean=1.5]"));

    onoff.Install (sta.Get (i));
  }

  /* ---------- FlowMonitor ---------- */
  FlowMonitorHelper fmHelper;
  Ptr<FlowMonitor> fm = fmHelper.InstallAll ();
  Ptr<Ipv4FlowClassifier> classifier =
    DynamicCast<Ipv4FlowClassifier> (fmHelper.GetClassifier ());
  namespace fs = std::filesystem;
  fs::create_directories ("output/csv/wifi-2026");

  std::ofstream out ("output/csv/wifi-2026/persta_bursty.csv");
  out << "time,staId,nSta,los_state,throughput,delay,jitter,loss\n";

  Simulator::Schedule (
    Seconds (WINDOW),
    &SampleWindow, fm, classifier,
    sta, ap.Get (0), nSta, &out);

  Simulator::Stop (Seconds (SIM_TIME));
  Simulator::Run ();
  Simulator::Destroy ();

  out.close ();
  return 0;
}
