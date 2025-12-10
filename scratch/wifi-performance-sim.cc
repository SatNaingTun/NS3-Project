#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/mobility-module.h"
#include "ns3/wifi-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include <fstream>
#include <iomanip>
#include <sstream>
#include <ctime>
#include <vector>
#include <tuple>
#include <iostream>

using namespace ns3;

// =======================================================================
// GLOBALS
// =======================================================================
struct DensityRecord {
    double start;
    double end;
    uint32_t nodes;
};

static std::vector<DensityRecord> g_densityLog;

// (time_s, rssi, noise, snr)
using ModSample = std::tuple<double,double,double,double>;
static std::vector<ModSample> g_modSamples;

static std::ofstream g_rssiCsv;
static std::ofstream g_modCsv;

static uint32_t g_seed = 12345;
static std::string g_runTag;
static std::string g_prefix;

static uint32_t g_currentActive = 0;

static std::string g_outputDir = "outputs/csv/wifi-random/";

// =======================================================================
// RUN TAGS
// =======================================================================
std::string MakeRunTag() {
    std::time_t now = std::time(nullptr);
    char buf[64];
    std::strftime(buf, sizeof(buf), "%d-%b-%Y_%H-%M", std::localtime(&now));
    return std::string(buf);
}

std::string MakePrefix(const std::string &tag, uint32_t seed) {
    std::ostringstream oss;
    oss << "wifi-random-" << tag << "-seed" << seed;
    return oss.str();
}

// =======================================================================
// ASCII PROGRESS BAR
// =======================================================================
void PrintProgressBar(size_t current, size_t total, const std::string &label) {
    double pct = (double)current / total;
    int barWidth = 30;
    int filled = pct * barWidth;

    std::cout << "\r[" << label << "] ["
              << std::string(filled, '#')
              << std::string(barWidth - filled, '-')
              << "] "
              << std::fixed << std::setprecision(1)
              << (pct * 100.0) << "%";

    if (current + 1 == total)
        std::cout << std::endl;
}

// =======================================================================
// SIM PROGRESS (prints every 1 sec)
// =======================================================================
void PrintSimulationProgress(double simTime) {
    double now = Simulator::Now().GetSeconds();
    double pct = (now / simTime) * 100.0;

    std::cout << "[SIM] " << std::fixed << std::setprecision(1)
              << now << " / " << simTime << " sec (" << pct << "%)" << std::endl;

    Simulator::Schedule(Seconds(1.0), &PrintSimulationProgress, simTime);
}

// =======================================================================
// PHY TRACE CALLBACK — Correct Signature
// =======================================================================
void PhyMonitorSnifferRx(std::string context,
                         Ptr<const Packet> packet,
                         uint16_t channelFreqMhz,
                         WifiTxVector txVector,
                         MpduInfo aMpdu,
                         SignalNoiseDbm signalNoise,
                         uint16_t stationId)
{
    if (!g_rssiCsv.good() || !g_modCsv.good()) return;

    const double t = Simulator::Now().GetSeconds();
    const double rssi  = signalNoise.signal;
    const double noise = signalNoise.noise;
    const double snrDb = rssi - noise;

    WifiMode mode = txVector.GetMode();
    std::string modClass;

    switch (mode.GetModulationClass()) {
        case WIFI_MOD_CLASS_DSSS: modClass = "DSSS"; break;
        case WIFI_MOD_CLASS_OFDM: modClass = "OFDM"; break;
        case WIFI_MOD_CLASS_HT:   modClass = "HT";   break;
        case WIFI_MOD_CLASS_VHT:  modClass = "VHT";  break;
        case WIFI_MOD_CLASS_HE:   modClass = "HE";   break;
        default: modClass = "OTHER";
    }

    uint32_t constellation = mode.GetConstellationSize();
    double phyRateMbps = mode.GetDataRate(txVector) / 1e6;

    // --- rssi.csv ---
    g_rssiCsv << std::fixed << std::setprecision(6)
              << t << "," << rssi << "," << noise << "," << snrDb << ","
              << g_seed << "," << g_runTag << "\n";

    // --- modulation.csv ---
    g_modCsv << std::fixed << std::setprecision(6)
             << t << "," << channelFreqMhz << ","
             << modClass << "," << constellation << ","
             << phyRateMbps << ","
             << rssi << "," << noise << "," << snrDb << ","
             << g_seed << "," << g_runTag << "\n";

    // Store sample for interval aggregation
    g_modSamples.emplace_back(t, rssi, noise, snrDb);

    (void)stationId;
}

// =======================================================================
// SOLUTION C: ALWAYS CHOOSE NEW DENSITY RANDOMLY PER INTERVAL
// =======================================================================
void ChangeActiveStations(uint32_t nMin, uint32_t nMax, double interval)
{
    Ptr<UniformRandomVariable> rnd = CreateObject<UniformRandomVariable>();
    rnd->SetStream(g_seed + (uint32_t)Simulator::Now().GetSeconds() * 919);

    // Always pick a new density
    uint32_t next = rnd->GetInteger(nMin, nMax);

    double now = Simulator::Now().GetSeconds();

    // Close previous interval
    if (!g_densityLog.empty())
        g_densityLog.back().end = now;

    // Create new interval
    g_densityLog.push_back({ now, now + interval, next });

    g_currentActive = next;

    Simulator::Schedule(Seconds(interval),
                        &ChangeActiveStations, nMin, nMax, interval);
}

// =======================================================================
// PERF WRITER
// =======================================================================
void WritePerfCsv(const std::string &prefix,
                  Ptr<FlowMonitor> monitor,
                  Ptr<Ipv4FlowClassifier> classifier)
{
    std::ofstream f(g_outputDir + prefix + "-perf.csv");

    f << "FlowID,Source,Destination,Throughput(Mbps),Latency_avg(ms),"
         "Jitter_avg(ms),PacketLoss(%),Duration_s,RandSeed,RunDateTime\n";

    auto stats = monitor->GetFlowStats();
    size_t idx=0, total=stats.size();

    for (auto &kv : stats) {
        PrintProgressBar(idx, total, "FlowMonitor");
        idx++;

        const auto &s = kv.second;
        auto ft = classifier->FindFlow(kv.first);

        double d = (s.timeLastRxPacket - s.timeFirstTxPacket).GetSeconds();
        if (d <= 0 || s.rxPackets == 0) continue;

        double thr = (s.rxBytes * 8.0) / d / 1e6;
        double lat = (s.delaySum.GetSeconds() / s.rxPackets) * 1000.0;
        double jit = (s.jitterSum.GetSeconds() / s.rxPackets) * 1000.0;
        double loss = (s.txPackets > 0) ?
            100.0 * (s.txPackets - s.rxPackets) / s.txPackets : 0.0;

        f << kv.first << "," << ft.sourceAddress << "," << ft.destinationAddress
          << "," << thr << "," << lat << "," << jit << "," << loss
          << "," << d << "," << g_seed << "," << g_runTag << "\n";
    }
}

// =======================================================================
// NODEDENSITY WRITER
// =======================================================================
void WriteNodeDensityCsv(const std::string &prefix,
                         Ptr<FlowMonitor> monitor,
                         double simTime)
{
    std::ofstream f(g_outputDir + prefix + "-nodedensity.csv");

    f << "StartDateTime,EndDateTime,Duration_s,NodeDensity,"
         "TotalTxPackets,TotalRxPackets,AvgThroughput(Mbps),"
         "PacketLoss(%),AvgLatency(ms),AvgJitter(ms),AvgRSSI(dBm),AvgSNR(dB)\n";

    auto stats = monitor->GetFlowStats();
    size_t idx=0, total=g_densityLog.size();
    time_t base = time(nullptr);

    for (auto &rec : g_densityLog) {
        PrintProgressBar(idx, total, "NodeDensity");
        idx++;

        double st = rec.start;
        double en = std::min(rec.end, simTime);
        double dur = en - st;

        double sumThr=0, sumLat=0, sumJit=0;
        uint64_t totTx=0, totRx=0;
        uint32_t valid=0;

        // FLOW interval aggregation
        for (auto &kv : stats) {
            const auto &s = kv.second;
            if (s.rxPackets == 0) continue;

            double t1 = s.timeFirstTxPacket.GetSeconds();
            double t2 = s.timeLastRxPacket.GetSeconds();

            if (t2 < st || t1 > en) continue;

            double flowD = t2 - t1;
            if (flowD <= 0) continue;

            double thr = (s.rxBytes * 8.0) / flowD / 1e6;
            double lat = (s.delaySum.GetSeconds() / s.rxPackets) * 1000.0;
            double jit = (s.jitterSum.GetSeconds() / s.rxPackets) * 1000.0;

            sumThr += thr;
            sumLat += lat;
            sumJit += jit;

            totTx += s.txPackets;
            totRx += s.rxPackets;
            valid++;
        }

        double avgThr = (valid > 0) ? sumThr / valid : 0;
        double avgLat = (valid > 0) ? sumLat / valid : 0;
        double avgJit = (valid > 0) ? sumJit / valid : 0;
        double loss   = (totTx > 0) ? 100.0 * (totTx - totRx) / totTx : 0;

        // PHY interval aggregation
        double sumR=0, sumSnr=0; int rc=0;
        for (auto &x : g_modSamples) {
            double t = std::get<0>(x);
            if (t >= st && t < en) {
                sumR   += std::get<1>(x);
                sumSnr += std::get<3>(x);
                rc++;
            }
        }

        double avgRssi = (rc>0)? sumR/rc : 0;
        double avgSnr  = (rc>0)? sumSnr/rc : 0;

        // timestamps
        char sb[64], eb[64];
        strftime(sb, sizeof(sb), "%Y-%m-%d %H:%M:%S", localtime(&base + (time_t)st));
        strftime(eb, sizeof(eb), "%Y-%m-%d %H:%M:%S", localtime(&base + (time_t)en));

        f << sb << "," << eb << "," << dur << ","
          << rec.nodes << "," << totTx << "," << totRx << ","
          << avgThr << "," << loss << ","
          << avgLat << "," << avgJit << ","
          << avgRssi << "," << avgSnr << "\n";
    }
}

// =======================================================================
// MAIN
// =======================================================================
int main(int argc, char *argv[])
{
    bool indoor = true;
    uint32_t nMin = 5, nMax = 30;
    double area = 50, simTime = 30, txPower = 16;
    bool enableInterference = true;
    uint16_t port = 9999;
    uint32_t pktSize = 1024;
    double appInterval = 0.01;
    double changeInterval = 5.0;

    CommandLine cmd;
    cmd.AddValue("simTime", "Simulation duration", simTime);
    cmd.AddValue("seed", "Random seed", g_seed);
    cmd.AddValue("changeInterval", "Density change interval", changeInterval);
    cmd.Parse(argc, argv);

    g_runTag = MakeRunTag();
    g_prefix = MakePrefix(g_runTag, g_seed);

    // ensure output directory
    system(("mkdir -p " + g_outputDir).c_str());

    // Open CSVs
    g_rssiCsv.open((g_outputDir + g_prefix + "-rssi.csv").c_str());
    g_modCsv.open((g_outputDir + g_prefix + "-modulation.csv").c_str());

    g_rssiCsv << "time_s,RSSI,Noise,SNR_dB,RandSeed,RunDateTime\n";
    g_modCsv  << "time_s,channel_MHz,Modulation,ConstellationSize,PhyRate_Mbps,"
                 "signal_dBm,noise_dBm,SNR_dB,RandSeed,RunDateTime\n";

    // -------------------------------------------------------------------
    // SCENARIO BUILD
    // -------------------------------------------------------------------
    NodeContainer staNodes, apNode;
    staNodes.Create(nMax);
    apNode.Create(1);

    YansWifiChannelHelper channel = YansWifiChannelHelper::Default();
    YansWifiPhyHelper phy;
    phy.SetChannel(channel.Create());
    phy.Set("TxPowerStart", DoubleValue(txPower));
    phy.Set("TxPowerEnd",   DoubleValue(txPower));

    WifiHelper wifi;
    WifiMacHelper mac;
    Ssid ssid("ns3-ssid");

    // STA
    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid),
                "ActiveProbing", BooleanValue(false));
    NetDeviceContainer staDevs = wifi.Install(phy, mac, staNodes);

    // AP
    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    NetDeviceContainer apDevs = wifi.Install(phy, mac, apNode);

    MobilityHelper mob;
    mob.SetPositionAllocator("ns3::RandomRectanglePositionAllocator",
        "X", StringValue("ns3::UniformRandomVariable[Min=-50|Max=50]"),
        "Y", StringValue("ns3::UniformRandomVariable[Min=-50|Max=50]"));
    mob.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
        "Bounds", RectangleValue(Rectangle(-area, area, -area, area)));
    mob.Install(staNodes);

    MobilityHelper apMob;
    apMob.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    apMob.Install(apNode);

    InternetStackHelper stack;
    stack.Install(staNodes);
    stack.Install(apNode);

    Ipv4AddressHelper addr;
    addr.SetBase("10.1.3.0", "255.255.255.0");
    addr.Assign(apDevs);
    addr.Assign(staDevs);

    Ptr<Ipv4> ipv4 = apNode.Get(0)->GetObject<Ipv4>();
    Ipv4Address apAddr = ipv4->GetAddress(1,0).GetLocal();

    // Traffic
    UdpServerHelper server(port);
    auto srvApps = server.Install(apNode.Get(0));
    srvApps.Start(Seconds(0.1));
    srvApps.Stop(Seconds(simTime));

    UdpClientHelper client(apAddr, port);
    client.SetAttribute("MaxPackets", UintegerValue(0xFFFFFFFF));
    client.SetAttribute("Interval", TimeValue(Seconds(appInterval)));
    client.SetAttribute("PacketSize", UintegerValue(pktSize));

    for (uint32_t i=0; i<nMax; i++) {
        auto apps = client.Install(staNodes.Get(i));
        apps.Start(Seconds(0.5));
        apps.Stop(Seconds(simTime));
    }

    g_currentActive = nMax;
    g_densityLog.push_back({0.0, changeInterval, nMax});

    // PHY tracing
    Config::Connect("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
                    MakeCallback(&PhyMonitorSnifferRx));

    // schedule density changes
    Simulator::Schedule(Seconds(changeInterval),
                        &ChangeActiveStations, nMin, nMax, changeInterval);

    // sim progress
    Simulator::Schedule(Seconds(1.0),
                        &PrintSimulationProgress, simTime);

    // FlowMonitor
    FlowMonitorHelper fm;
    Ptr<FlowMonitor> monitor = fm.InstallAll();

    Simulator::Stop(Seconds(simTime));
    Simulator::Run();
    Simulator::Destroy();

    Ptr<Ipv4FlowClassifier> classifier =
        DynamicCast<Ipv4FlowClassifier>(fm.GetClassifier());

    g_rssiCsv.close();
    g_modCsv.close();

    WritePerfCsv(g_prefix, monitor, classifier);
    WriteNodeDensityCsv(g_prefix, monitor, simTime);

    std::cout << "\n===== OUTPUTS GENERATED =====\n";
    std::cout << g_outputDir << g_prefix << "-perf.csv\n";
    std::cout << g_outputDir << g_prefix << "-rssi.csv\n";
    std::cout << g_outputDir << g_prefix << "-modulation.csv\n";
    std::cout << g_outputDir << g_prefix << "-nodedensity.csv\n";
    std::cout << "================================\n";

    return 0;
}
