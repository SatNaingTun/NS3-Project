#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"

#include <fstream>
#include <map>

using namespace ns3;

/* =========================
 * Scenario definitions
 * ========================= */
enum ScenarioType
{
  SCENARIO_LOS  = 0,
  SCENARIO_MID  = 1,
  SCENARIO_NLOS = 2
};

static std::string
ScenarioLabel (ScenarioType s)
{
  switch (s)
  {
    case SCENARIO_LOS:  return "LOS";
    case SCENARIO_MID:  return "MID";
    case SCENARIO_NLOS: return "NLOS";
    default:            return "UNKNOWN";
  }
}

/* =========================
 * Per-STA RSSI storage
 * ========================= */
static std::map<uint16_t, double>   g_rssiSum;
static std::map<uint16_t, uint32_t> g_rssiCnt;

/* =========================
 * CORRECT PHY TRACE CALLBACK
 * (matches NS-3 exactly)
 * ========================= */
static void
MonitorSnifferRx (
  Ptr<const Packet> packet,
  uint16_t channelFreqMhz,
  WifiTxVector txVector,
  MpduInfo mpdu,
  SignalNoiseDbm sn,
  uint16_t staId          // <-- PROVIDED BY NS-3
)
{
  g_rssiSum[staId] += sn.signal;
  g_rssiCnt[staId]++;
}

/* =========================
 * Main
 * ========================= */
int
main (int argc, char *argv[])
{
  uint32_t nSta = 5;
  double simTime = 20.0;
  uint32_t seed = 1;

  /* Scenario selector (NO enum parsing) */
  uint32_t scenarioInt = 0; // 0=LOS,1=MID,2=NLOS

  CommandLine cmd;
  cmd.AddValue("scenario", "0=LOS, 1=MID, 2=NLOS", scenarioInt);
  cmd.AddValue("nSta", "Number of STAs", nSta);
  cmd.Parse(argc, argv);

  ScenarioType scenario;
  if      (scenarioInt == 0) scenario = SCENARIO_LOS;
  else if (scenarioInt == 1) scenario = SCENARIO_MID;
  else if (scenarioInt == 2) scenario = SCENARIO_NLOS;
  else NS_FATAL_ERROR("Invalid scenario");

  RngSeedManager::SetSeed(seed);

  /* =========================
   * Nodes
   * ========================= */
  NodeContainer apNode, staNodes, interferer;
  apNode.Create(1);
  staNodes.Create(nSta);

  if (scenario == SCENARIO_MID)
    interferer.Create(1);

  /* =========================
   * Mobility
   * ========================= */
  MobilityHelper mobility;
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");

  mobility.Install(apNode);
  mobility.Install(staNodes);
  if (scenario == SCENARIO_MID)
    mobility.Install(interferer);

  apNode.Get(0)->GetObject<MobilityModel>()
    ->SetPosition(Vector(0.0, 0.0, 1.5));

  for (uint32_t i = 0; i < nSta; ++i)
  {
    staNodes.Get(i)->GetObject<MobilityModel>()
      ->SetPosition(Vector(8.0 + i, 0.0, 1.5));
  }

  if (scenario == SCENARIO_MID)
  {
    interferer.Get(0)->GetObject<MobilityModel>()
      ->SetPosition(Vector(4.0, 0.0, 1.5)); // midpoint
  }

  /* =========================
   * Propagation models
   * ========================= */
  Ptr<PropagationLossModel> loss;

  if (scenario == SCENARIO_LOS)
  {
    auto m = CreateObject<LogDistancePropagationLossModel>();
    m->SetPathLossExponent(2.0);
    loss = m;
  }
  else if (scenario == SCENARIO_NLOS)
  {
    auto m = CreateObject<LogDistancePropagationLossModel>();
    m->SetPathLossExponent(3.3);

    auto shadow = CreateObject<RandomPropagationLossModel>();
    shadow->SetAttribute(
      "Variable",
      StringValue("ns3::NormalRandomVariable[Mean=-7|Variance=4]"));

    m->SetNext(shadow);
    loss = m;
  }
  else // MID
  {
    auto m = CreateObject<LogDistancePropagationLossModel>();
    m->SetPathLossExponent(2.7);

    auto nak = CreateObject<NakagamiPropagationLossModel>();
    nak->SetAttribute("m0", DoubleValue(0.7));

    m->SetNext(nak);
    loss = m;
  }

  Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel>();
  channel->SetPropagationLossModel(loss);
  channel->SetPropagationDelayModel(
    CreateObject<ConstantSpeedPropagationDelayModel>());

  /* =========================
   * Wi-Fi
   * ========================= */
  YansWifiPhyHelper phy;
  phy.SetChannel(channel);
  phy.Set("TxPowerStart", DoubleValue(18.0));
  phy.Set("TxPowerEnd",   DoubleValue(18.0));

  WifiHelper wifi;
  wifi.SetStandard(WIFI_STANDARD_80211ac);

  WifiMacHelper mac;
  Ssid ssid("perSTA-ai");

  mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
  NetDeviceContainer apDev = wifi.Install(phy, mac, apNode);

  mac.SetType("ns3::StaWifiMac",
              "Ssid", SsidValue(ssid),
              "ActiveProbing", BooleanValue(false));
  NetDeviceContainer staDev = wifi.Install(phy, mac, staNodes);

  if (scenario == SCENARIO_MID)
    wifi.Install(phy, mac, interferer);

  /* =========================
   * PHY tracing (CRITICAL FIX)
   * ========================= */
  for (uint32_t i = 0; i < staDev.GetN(); ++i)
  {
    Ptr<WifiNetDevice> dev =
      DynamicCast<WifiNetDevice>(staDev.Get(i));

    dev->GetPhy()->TraceConnectWithoutContext(
      "MonitorSnifferRx",
      MakeCallback(&MonitorSnifferRx));
  }

  /* =========================
   * Internet + traffic
   * ========================= */
  InternetStackHelper internet;
  internet.Install(apNode);
  internet.Install(staNodes);
  if (scenario == SCENARIO_MID)
    internet.Install(interferer);

  Ipv4AddressHelper ip;
  ip.SetBase("10.1.1.0", "255.255.255.0");
  ip.Assign(apDev);
  ip.Assign(staDev);

  UdpServerHelper server(5000);
  server.Install(apNode.Get(0));

  UdpClientHelper client(Ipv4Address("10.1.1.1"), 5000);
  client.SetAttribute("Interval", TimeValue(MilliSeconds(10)));
  client.SetAttribute("PacketSize", UintegerValue(1024));
  client.SetAttribute("MaxPackets", UintegerValue(1e6));

  for (uint32_t i = 0; i < nSta; ++i)
    client.Install(staNodes.Get(i));

  /* =========================
   * Run
   * ========================= */
  Simulator::Stop(Seconds(simTime));
  Simulator::Run();

  /* =========================
   * CSV output
   * ========================= */
  std::ofstream out("per_sta_dataset.csv", std::ios::app);

  for (auto &kv : g_rssiCnt)
  {
    uint16_t staId = kv.first;
    double avgRssi = g_rssiSum[staId] / kv.second;

    out << seed << ","
        << staId << ","
        << avgRssi << ","
        << ScenarioLabel(scenario)
        << std::endl;
  }

  out.close();
  Simulator::Destroy();
  return 0;
}
