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
#include <string>
#include <sys/stat.h>

using namespace ns3;

/* ================= Reproducibility ================= */
static constexpr uint32_t SEED = 1;
static constexpr uint32_t RUN  = 1;

/* ================= Geometry thresholds ================= */
static constexpr double LOS_DIST = 10.0;
static constexpr double MID_DIST = 25.0;

/* ================= Simulation ================= */
static constexpr double SIM_TIME = 20.0;
static constexpr double SAMPLE_INTERVAL = 0.1;

/* ================= Dataset ================= */
struct Sample
{
  double time;
  double throughput;
  double delay;
  double dhcpTime;
  std::string link;
  std::string dir;   // UL / DL
};

static std::map<uint32_t, std::vector<Sample>> g_series;
static std::map<uint32_t, double> g_dhcpTime;

/* ================= Utility ================= */
static bool
FileExists (const std::string &filename)
{
  struct stat buffer;
  return (stat (filename.c_str (), &buffer) == 0);
}

/* ================= Link classification ================= */
static std::string
LinkState (Ptr<MobilityModel> sta, Ptr<MobilityModel> ap)
{
  double d = sta->GetDistanceFrom (ap);
  if (d < LOS_DIST) return "LOS";
  if (d < MID_DIST) return "MID";
  return "NLOS";
}

/* ================= Flow sampling ================= */
static void
SampleStats (Ptr<FlowMonitor> fm,
             Ptr<Ipv4FlowClassifier> cl,
             NodeContainer sta,
             Ptr<MobilityModel> ap)
{
  fm->CheckForLostPackets ();

  for (const auto &kv : fm->GetFlowStats ())
  {
    const FlowMonitor::FlowStats &st = kv.second;
    if (st.rxPackets == 0)
      continue;

    Ipv4FlowClassifier::FiveTuple f =
      cl->FindFlow (kv.first);

    bool uplink =
      (f.destinationAddress == Ipv4Address ("10.1.1.1"));

    uint32_t staId =
      uplink
        ? f.sourceAddress.Get () - 0x0A010102   // 10.1.1.2
        : f.destinationAddress.Get () - 0x0A010102;

    if (staId >= sta.GetN ())
      continue;

    double throughputMbps =
      (st.rxBytes * 8.0) /
      (Simulator::Now ().GetSeconds () * 1e6);

    double delayMs =
      (st.delaySum.GetSeconds () / st.rxPackets) * 1000.0;

    g_series[staId].push_back ({
      Simulator::Now ().GetSeconds (),
      throughputMbps,
      delayMs,
      g_dhcpTime[staId],
      LinkState (
        sta.Get (staId)->GetObject<MobilityModel> (),
        ap),
      uplink ? "UL" : "DL"
    });
  }

  Simulator::Schedule (Seconds (SAMPLE_INTERVAL),
                       &SampleStats,
                       fm, cl, sta, ap);
}

/* ================= Main ================= */
int
main ()
{
  RngSeedManager::SetSeed (SEED);
  RngSeedManager::SetRun  (RUN);

  Ptr<UniformRandomVariable> rv =
    CreateObject<UniformRandomVariable> ();

  uint32_t nSta = rv->GetInteger (3, 10);

  NodeContainer ap, sta;
  ap.Create (1);
  sta.Create (nSta);

  /* ---------- Mobility ---------- */
  MobilityHelper mob;

  // AP fixed
  mob.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mob.Install (ap);
  ap.Get (0)->GetObject<MobilityModel> ()
    ->SetPosition (Vector (0.0, 0.0, 1.5));

  // STAs
  for (uint32_t i = 0; i < nSta; ++i)
  {
    bool mobile = (rv->GetValue () < 0.5);

    if (mobile)
    {
      mob.SetMobilityModel (
        "ns3::RandomWalk2dMobilityModel",
        "Bounds", RectangleValue (
          Rectangle (-30.0, 30.0, -30.0, 30.0)));
    }
    else
    {
      mob.SetMobilityModel (
        "ns3::ConstantPositionMobilityModel");
    }

    mob.Install (sta.Get (i));

    // Initial position MUST be inside bounds
    sta.Get (i)->GetObject<MobilityModel> ()
      ->SetPosition (Vector (
        rv->GetValue (-25.0, 25.0),
        rv->GetValue (-25.0, 25.0),
        1.5));

    // Emulated DHCP completion time (feature)
    g_dhcpTime[i] = rv->GetValue (0.5, 2.0);
  }

  /* ---------- Wi-Fi ---------- */
  WifiHelper wifi;
  wifi.SetStandard (WIFI_STANDARD_80211ac);

  Ptr<YansWifiChannel> channel =
    CreateObject<YansWifiChannel> ();

  channel->SetPropagationLossModel (
    CreateObject<LogDistancePropagationLossModel> ());

  // REQUIRED: fixes null pointer crash
  channel->SetPropagationDelayModel (
    CreateObject<ConstantSpeedPropagationDelayModel> ());

  YansWifiPhyHelper phy;
  phy.SetChannel (channel);

  WifiMacHelper mac;
  Ssid ssid ("ai-net");

  mac.SetType ("ns3::ApWifiMac",
               "Ssid", SsidValue (ssid));
  NetDeviceContainer apDev =
    wifi.Install (phy, mac, ap);

  mac.SetType ("ns3::StaWifiMac",
               "Ssid", SsidValue (ssid),
               "ActiveProbing", BooleanValue (false));
  NetDeviceContainer staDev =
    wifi.Install (phy, mac, sta);

  /* ---------- Internet ---------- */
  InternetStackHelper internet;
  internet.InstallAll ();

  Ipv4AddressHelper ip;
  ip.SetBase ("10.1.1.0", "255.255.255.0");
  ip.Assign (apDev);
  ip.Assign (staDev);

  /* ---------- Traffic ---------- */

  // Uplink server on AP
  UdpServerHelper ulServer (5000);
  ulServer.Install (ap.Get (0));

  // Downlink servers on STAs
  for (uint32_t i = 0; i < nSta; ++i)
  {
    UdpServerHelper dlServer (6000 + i);
    dlServer.Install (sta.Get (i));
  }

  // Uplink clients
  for (uint32_t i = 0; i < nSta; ++i)
  {
    UdpClientHelper ulClient (
      Ipv4Address ("10.1.1.1"), 5000);
    ulClient.SetAttribute (
      "Interval", TimeValue (MilliSeconds (10)));
    ulClient.SetAttribute (
      "PacketSize", UintegerValue (1024));
    ulClient.Install (sta.Get (i));
  }

  // Downlink clients
  for (uint32_t i = 0; i < nSta; ++i)
  {
    Ipv4Address staIp (
      Ipv4Address ("10.1.1.0").Get () + i + 2);

    UdpClientHelper dlClient (
      staIp, 6000 + i);
    dlClient.SetAttribute (
      "Interval", TimeValue (MilliSeconds (20)));
    dlClient.SetAttribute (
      "PacketSize", UintegerValue (1024));
    dlClient.Install (ap.Get (0));
  }

  /* ---------- FlowMonitor ---------- */
  FlowMonitorHelper fmh;
  Ptr<FlowMonitor> fm = fmh.InstallAll ();
  Ptr<Ipv4FlowClassifier> cl =
    DynamicCast<Ipv4FlowClassifier> (
      fmh.GetClassifier ());

  Simulator::Schedule (
    Seconds (SAMPLE_INTERVAL),
    &SampleStats,
    fm, cl, sta,
    ap.Get (0)->GetObject<MobilityModel> ());

  Simulator::Stop (Seconds (SIM_TIME));
  Simulator::Run ();
  Simulator::Destroy ();

  /* ---------- CSV Export ---------- */
  const std::string filename = "ai_dataset.csv";
  bool writeHeader = !FileExists (filename);

  std::ofstream out (filename, std::ios::app);

  if (writeHeader)
  {
    out << "seed,"
        << "nSta,"
        << "staId,"
        << "time,"
        << "throughput,"
        << "delay,"
        << "dhcpTime,"
        << "link,"
        << "direction\n";
  }

  for (const auto &kv : g_series)
  {
    for (const auto &s : kv.second)
    {
      out << SEED << ","
          << nSta << ","
          << kv.first << ","
          << s.time << ","
          << s.throughput << ","
          << s.delay << ","
          << s.dhcpTime << ","
          << s.link << ","
          << s.dir << "\n";
    }
  }

  out.close ();
  return 0;
}
