// =============================================================
//   Wifi-pref-random.cc  (Original version, patched)
// =============================================================

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
#include <limits>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("WifiPerfRandom");

// ========================================================
// Globals
// ========================================================
static uint32_t activeNodes = 0;

struct DensityRecord {
    double start;
    double end;
    uint32_t nodes;
};

static std::vector<DensityRecord> densityLog;

// ========================================================
// RSSI tracer (original)
// ========================================================
static void
RssiTracer(uint32_t randSeed, std::string runTag, std::ofstream *csv,
           Ptr<const Packet>, uint16_t channelFreqMhz, WifiTxVector,
           MpduInfo, SignalNoiseDbm signalNoise, uint16_t)
{
    if (!csv || !csv->good()) return;

    (*csv) << std::fixed << std::setprecision(6)
           << Simulator::Now().GetSeconds() << ","
           << channelFreqMhz << ","
           << signalNoise.signal << ","
           << signalNoise.noise << ","
           << (signalNoise.signal - signalNoise.noise) << ","
           << randSeed << "," << runTag << "\n";
}

// ========================================================
// Modulation-aware BER tracer (original)
// ========================================================
static void
ModulationBerTracer(uint32_t randSeed, std::string runTag, std::ofstream *csv,
                    Ptr<const Packet>,
                    uint16_t channelFreqMhz,
                    WifiTxVector txVector,
                    MpduInfo,
                    SignalNoiseDbm signalNoise,
                    uint16_t staId)
{
    if (!csv || !csv->good()) return;

    double rssi  = signalNoise.signal;
    double noise = signalNoise.noise;
    double snrDb = rssi - noise;
    double snrLinear = std::pow(10.0, snrDb / 10.0);

    Ptr<YansErrorRateModel> err = CreateObject<YansErrorRateModel>();
    WifiMode mode = txVector.GetMode();

    uint64_t nbits = txVector.GetNss() * 1500 * 8;
    uint8_t chWidth = txVector.GetChannelWidth();

    double succ = err->GetChunkSuccessRate(mode, txVector, snrLinear,
                                           nbits, chWidth,
                                           WIFI_PPDU_FIELD_DATA, staId);

    double berLinear = (1.0 - succ) / std::max<uint64_t>(1, nbits);
    if (berLinear <= 0.0)
        berLinear = std::numeric_limits<double>::min();

    double berDb = 10.0 * std::log10(berLinear);

    std::string modName;
    switch (mode.GetModulationClass()) {
        case WIFI_MOD_CLASS_DSSS: modName = "DSSS"; break;
        case WIFI_MOD_CLASS_OFDM: modName = "OFDM"; break;
        case WIFI_MOD_CLASS_HT:   modName = "HT";   break;
        case WIFI_MOD_CLASS_VHT:  modName = "VHT";  break;
        case WIFI_MOD_CLASS_HE:   modName = "HE";   break;
        default: modName = "UNKNOWN"; break;
    }

    (*csv) << std::scientific << std::setprecision(8)
           << Simulator::Now().GetSeconds() << ","
           << channelFreqMhz << ","
           << modName << ","
           << mode.GetConstellationSize() << ","
           << mode.GetDataRate(txVector) / 1e6 << ","
           << rssi << "," << noise << "," << snrDb << ","
           << berLinear << "," << berDb << ","
           << randSeed << "," << runTag << "\n";
}

// ========================================================
// Node density change controller (original)
// ========================================================
void ChangeActiveStations(ApplicationContainer &apps,
                          uint32_t &curN,
                          uint32_t nMin,
                          uint32_t nMax,
                          uint32_t seed,
                          double interval)
{
    double now = Simulator::Now().GetSeconds();
    Ptr<UniformRandomVariable> rnd = CreateObject<UniformRandomVariable>();
    rnd->SetStream(seed + (uint32_t)now * 17);

    int delta = std::round(rnd->GetValue(-3.0, 3.0));
    int newN  = std::max((int)nMin, std::min((int)nMax, (int)curN + delta));

    if (newN != (int)curN) {
        if (!densityLog.empty())
            densityLog.back().end = now;

        densityLog.push_back({now, now + interval, (uint32_t)newN});
        curN = newN;

        for (uint32_t i = 0; i < apps.GetN(); i++) {
            apps.Get(i)->SetAttribute(
                (i < curN) ? "StartTime" : "StopTime",
                TimeValue(Seconds(now))
            );
        }
    }

    Simulator::Schedule(Seconds(interval),
                        &ChangeActiveStations,
                        std::ref(apps),
                        std::ref(curN),
                        nMin, nMax,
                        seed + 1,
                        interval);
}

// ========================================================
// Wi-Fi setup (original)
// ========================================================
struct WifiBss {
    NodeContainer ap, stas;
    NetDeviceContainer apDev, staDevs;
    Ipv4InterfaceContainer apIf, staIf;
};

WifiBss SetupWifiBss(std::string subnet, double txPower, double areaHalf,
                     uint32_t nSta, bool indoor)
{
    WifiBss b;
    b.ap.Create(1);
    b.stas.Create(nSta);

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ac);

    YansWifiChannelHelper chan = YansWifiChannelHelper::Default();
    chan.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                            "Exponent", DoubleValue(3.0));
    chan.AddPropagationLoss("ns3::NakagamiPropagationLossModel");

    YansWifiPhyHelper phy;
    phy.SetChannel(chan.Create());
    phy.Set("TxPowerStart", DoubleValue(txPower));
    phy.Set("TxPowerEnd",   DoubleValue(txPower));

    WifiMacHelper mac;
    Ssid ssid = Ssid(subnet);

    mac.SetType("ns3::StaWifiMac",
                "Ssid", SsidValue(ssid),
                "ActiveProbing", BooleanValue(false));
    b.staDevs = wifi.Install(phy, mac, b.stas);

    mac.SetType("ns3::ApWifiMac", "Ssid", SsidValue(ssid));
    b.apDev = wifi.Install(phy, mac, b.ap);

    MobilityHelper ms;
    ms.SetMobilityModel("ns3::RandomWalk2dMobilityModel",
                        "Bounds",
                        RectangleValue(Rectangle(-areaHalf, areaHalf,
                                                 -areaHalf, areaHalf)));
    ms.Install(b.stas);

    MobilityHelper ma;
    ma.SetMobilityModel("ns3::ConstantPositionMobilityModel");
    ma.Install(b.ap);

    InternetStackHelper stack;
    stack.Install(b.ap);
    stack.Install(b.stas);

    Ipv4AddressHelper ip;
    ip.SetBase(subnet.c_str(), "255.255.255.0");
    b.staIf = ip.Assign(b.staDevs);
    b.apIf  = ip.Assign(b.apDev);

    return b;
}

// ========================================================
// UDP traffic (original)
// ========================================================
ApplicationContainer InstallTraffic(WifiBss &b,
                                    uint16_t port,
                                    uint32_t pktSz,
                                    double intervalMs,
                                    double simTime)
{
    UdpClientHelper client(b.apIf.GetAddress(0), port);
    client.SetAttribute("MaxPackets", UintegerValue(0));
    client.SetAttribute("Interval",   TimeValue(MilliSeconds(intervalMs)));
    client.SetAttribute("PacketSize", UintegerValue(pktSz));

    ApplicationContainer apps;
    for (uint32_t i = 0; i < b.stas.GetN(); i++)
        apps.Add(client.Install(b.stas.Get(i)));

    UdpServerHelper srv(port);
    auto s = srv.Install(b.ap.Get(0));

    apps.Start(Seconds(2.0));
    apps.Stop(Seconds(simTime));
    s.Start(Seconds(1.0));
    s.Stop(Seconds(simTime));

    return apps;
}

// ========================================================
// MAIN (original + patch)
// ========================================================
int main(int argc, char *argv[])
{
    uint32_t nMin = 5, nMax = 30;
    double simTime = 30.0, txPower = 16.0;
    double area = 50.0;
    uint32_t seed = 8;
    double change = 5.0;
    uint32_t pktSize = 1024;
    double clientIntv = 10.0;
    bool indoor = true;

    CommandLine cmd;
    cmd.AddValue("nStaMin", "", nMin);
    cmd.AddValue("nStaMax", "", nMax);
    cmd.AddValue("simTime", "", simTime);
    cmd.AddValue("txPower", "", txPower);
    cmd.AddValue("seed", "", seed);
    cmd.AddValue("densityChangeInterval", "", change);
    cmd.Parse(argc, argv);

    // timestamp
    time_t now = time(0);
    tm *lt = localtime(&now);
    std::ostringstream ts;
    ts << std::setfill('0') << std::setw(2) << lt->tm_mday << "-"
       << std::put_time(lt, "%b") << "-"
       << (1900 + lt->tm_year)
       << "_" << std::setw(2) << lt->tm_hour
       << "-" << std::setw(2) << lt->tm_min;

    std::string runTag = ts.str();
    std::string prefix =
        "outputs/csv/wifi-random/wifi-random-" + runTag +
        "-seed" + std::to_string(seed);

    // RNG
    RngSeedManager::SetSeed(seed);
    Ptr<UniformRandomVariable> rnd = CreateObject<UniformRandomVariable>();
    rnd->SetAttribute("Min", DoubleValue(nMin));
    rnd->SetAttribute("Max", DoubleValue(nMax + 1));
    activeNodes = rnd->GetInteger();

    // Setup Wi-Fi
    WifiBss bss = SetupWifiBss("10.1.3.0", txPower, area, nMax, indoor);

    ApplicationContainer apps =
        InstallTraffic(bss, 9999, pktSize, clientIntv, simTime);

    // density logging
    densityLog.clear();
    densityLog.push_back({0.0, change, activeNodes});
    Simulator::Schedule(Seconds(change),
                        &ChangeActiveStations,
                        std::ref(apps),
                        std::ref(activeNodes),
                        nMin, nMax,
                        seed,
                        change);

    // Flow Monitor
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    // ============================
    // CSV tracers (original)
    // ============================
    std::ofstream rssi((prefix + "-rssi.csv").c_str());
    rssi << "time_s,channel_MHz,signal_dBm,noise_dBm,SNR_dB,"
            "RandSeed,RunDateTime\n";

    Config::ConnectWithoutContext(
        "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
        MakeBoundCallback(&RssiTracer, seed, runTag, &rssi));

    std::ofstream mod((prefix + "-modulation.csv").c_str());
    mod << "time_s,channel_MHz,Modulation,ConstellationSize,PhyRate_Mbps,"
           "signal_dBm,noise_dBm,SNR_dB,BERLinear,BER_dB,"
           "RandSeed,RunDateTime\n";

    Config::ConnectWithoutContext(
        "/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/MonitorSnifferRx",
        MakeBoundCallback(&ModulationBerTracer, seed, runTag, &mod));

    Simulator::Stop(Seconds(simTime + 1));
    Simulator::Run();

    // CLOSE TRACE FILES BEFORE POST-PROCESSING
    rssi.close();
    mod.close();

    // ====================================================
    // PERF CSV (original)
    // ====================================================
    monitor->CheckForLostPackets();
    auto stats = monitor->GetFlowStats();
    Ptr<Ipv4FlowClassifier> cls =
        DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());

    std::ofstream perf((prefix + "-perf.csv").c_str());
    perf << "FlowID,Source,Destination,Throughput(Mbps),"
            "Latency_avg(ms),Jitter_avg(ms),PacketLoss(%),"
            "Duration_s,RandSeed,RunDateTime\n";

    for (auto &kv : stats) {
        const auto &st = kv.second;
        auto t = cls->FindFlow(kv.first);

        double d = (st.timeLastRxPacket - st.timeFirstRxPacket).GetSeconds();
        double thr = (d > 0 && st.rxBytes > 0)
                       ? (st.rxBytes * 8.0 / d) / 1e6
                       : 0.0;
        double lat = (st.rxPackets > 0)
                       ? (st.delaySum.GetSeconds() / st.rxPackets) * 1000.0
                       : 0.0;
        double jit = (st.rxPackets > 0)
                       ? (st.jitterSum.GetSeconds() / st.rxPackets) * 1000.0
                       : 0.0;
        double loss = (st.txPackets > 0)
                        ? 100.0 * (st.txPackets - st.rxPackets) / st.txPackets
                        : 0.0;

        perf << kv.first << ","
             << t.sourceAddress << ","
             << t.destinationAddress << ","
             << thr << "," << lat << ","
             << jit << "," << loss << ","
             << d << "," << seed << "," << runTag << "\n";
    }

    perf.close();

    // =====================================================
    // LOAD modulation.csv (original)
    // =====================================================
    std::vector<std::tuple<double,double,double,double,double>> modData;
    {
        std::ifstream mf((prefix + "-modulation.csv").c_str());
        std::string hdr; std::getline(mf, hdr);

        double t, ch, constell, rate, sig, noi, snr, berLin, berDb;
        char comma;

        while (mf >> t >> comma >> ch >> comma) {
            std::string modName;
            std::getline(mf, modName, ',');
            mf >> constell >> comma >> rate >> comma
               >> sig >> comma >> noi >> comma
               >> snr >> comma >> berLin >> comma >> berDb;
            modData.push_back({t, sig, noi, snr, berLin});
        }
    }

    // =====================================================
    // NODE DENSITY CSV (original + PATCH)
    // =====================================================
    std::ofstream nd((prefix + "-nodedensity.csv").c_str());
    nd << "StartDateTime,EndDateTime,Duration_s,NodeDensity,"
          "TotalTxPackets,TotalRxPackets,AvgThroughput(Mbps),"
          "PacketLoss(%),AvgLatency(ms),AvgJitter(ms),"
          "AvgRSSI(dBm),AvgSNR(dB),AvgBER\n";

    time_t base = time(nullptr);

    // ============================
    // PATCH: Forward fill memory
    // ============================
    static double lastRssi = 0.0;
    static double lastSnr  = 0.0;
    static double lastBer  = 0.0;

    for (auto &rec : densityLog)
    {
        double st = rec.start;
        double en = rec.end;
        if (en > simTime) en = simTime;

        double dur = en - st;
        if (dur <= 0.0) continue;

        uint64_t txTot = 0, rxTot = 0;
        double sumThr = 0, sumLat = 0, sumJit = 0;
        int flows = 0;

        for (auto &kv : stats) {
            auto &s = kv.second;

            if (s.txPackets == 0 || s.rxPackets == 0) continue;
            if (s.timeLastRxPacket.GetSeconds() < st ||
                s.timeFirstTxPacket.GetSeconds() > en)
                continue;

            double d = (s.timeLastRxPacket - s.timeFirstTxPacket).GetSeconds();
            if (d <= 0) continue;

            double thr = (s.rxBytes * 8.0) / d / 1e6;
            double lat = (s.delaySum.GetSeconds() / s.rxPackets) * 1000.0;
            double jit = (s.jitterSum.GetSeconds() / s.rxPackets) * 1000.0;

            sumThr += thr;
            sumLat += lat;
            sumJit += jit;
            txTot  += s.txPackets;
            rxTot  += s.rxPackets;
            flows++;
        }

        double avgThr = flows ? sumThr / flows : 0.0;
        double avgLat = flows ? sumLat / flows : 0.0;
        double avgJit = flows ? sumJit / flows : 0.0;
        double lossPct = txTot > 0 ? 100.0 * (txTot - rxTot) / txTot : 0.0;

        // ======================================================
        // =============== PATCH ADDED HERE ======================
        // Forward-fill PHY values (NO MORE ZERO INTERVALS)
        // ======================================================
        double sR = 0, sS = 0, sB = 0;
        int rc = 0;
        for (auto &x : modData) {
            double t = std::get<0>(x);
            if (t >= st && t < en) {
                sR += std::get<1>(x); // RSSI
                sS += std::get<3>(x); // SNR
                sB += std::get<4>(x); // BER linear
                rc++;
            }
        }

        double aR, aS, aB;

        if (rc > 0) {
            aR = sR / rc;
            aS = sS / rc;
            aB = (sB / rc) * (1.0 + lossPct / 100.0);

            lastRssi = aR;
            lastSnr  = aS;
            lastBer  = aB;
        } else {
            aR = lastRssi;
            aS = lastSnr;
            aB = lastBer;
        }
        // ======================================================

        char sbuf[64], ebuf[64];
        time_t s1 = base + (time_t)st;
        time_t s2 = base + (time_t)en;
        strftime(sbuf, sizeof(sbuf), "%Y-%m-%d %H:%M:%S", localtime(&s1));
        strftime(ebuf, sizeof(ebuf), "%Y-%m-%d %H:%M:%S", localtime(&s2));

        nd << sbuf << "," << ebuf << ","
           << dur << ","
           << rec.nodes << ","
           << txTot << "," << rxTot << ","
           << avgThr << "," << lossPct << ","
           << avgLat << "," << avgJit << ","
           << aR << "," << aS << "," << aB << "\n";
    }

    nd.close();
    Simulator::Destroy();

    return 0;
}
