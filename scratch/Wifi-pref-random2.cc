// =============================================================
//   Wifi-pref-random.cc
//   - Original logic
//   - Per-STA PHY capture (StaId)
//   - Compatible with AI training dataset pipeline
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

struct DensityRecord
{
    double start;
    double end;
    uint32_t nodes;
};

static std::vector<DensityRecord> densityLog;

// ========================================================
// Per-STA RSSI tracer (MonitorSnifferRx on each STA WifiPhy)
// CSV: time_s,StaId,channel_MHz,signal_dBm,noise_dBm,SNR_dB,RandSeed,RunDateTime
// ========================================================
static void
RssiTracer(uint32_t staIndex,
           uint32_t randSeed,
           std::string runTag,
           std::ofstream *csv,
           Ptr<const Packet> packet,
           uint16_t channelFreqMhz,
           WifiTxVector txVector,
           MpduInfo mpdu,
           SignalNoiseDbm signalNoise,
           uint16_t staIdFromPhy)
{
    if (!csv || !csv->good())
    {
        return;
    }

    double timeNow = Simulator::Now().GetSeconds();
    double rssi    = signalNoise.signal;
    double noise   = signalNoise.noise;
    double snrDb   = rssi - noise;

    (*csv) << std::fixed << std::setprecision(6)
           << timeNow << ","
           << staIndex << ","
           << channelFreqMhz << ","
           << rssi << ","
           << noise << ","
           << snrDb << ","
           << randSeed << ","
           << runTag << "\n";
}

// ========================================================
// Per-STA modulation + BER tracer
//
// CSV:
// time_s,StaId,channel_MHz,Modulation,ConstellationSize,PhyRate_Mbps,
// signal_dBm,noise_dBm,SNR_dB,BERLinear,BER_dB,RandSeed,RunDateTime
//
// Later, for aggregation, we will keep in memory:
//   (time, rssi, noise, snr, berLinear, staId)
// ========================================================
static void
ModulationBerTracer(uint32_t staIndex,
                    uint32_t randSeed,
                    std::string runTag,
                    std::ofstream *csv,
                    Ptr<const Packet> packet,
                    uint16_t channelFreqMhz,
                    WifiTxVector txVector,
                    MpduInfo mpdu,
                    SignalNoiseDbm signalNoise,
                    uint16_t staIdFromPhy)
{
    if (!csv || !csv->good())
    {
        return;
    }

    double timeNow = Simulator::Now().GetSeconds();

    // PHY values
    double rssi    = signalNoise.signal;
    double noise   = signalNoise.noise;
    double snrDb   = rssi - noise;
    double snrLin  = std::pow(10.0, snrDb / 10.0);

    WifiMode mode = txVector.GetMode();

    // Error model
    Ptr<YansErrorRateModel> errModel = CreateObject<YansErrorRateModel>();

    // Assume 1500-byte payload for BER estimation
    uint64_t nbits   = txVector.GetNss() * 1500u * 8u;
    uint8_t  chWidth = txVector.GetChannelWidth();

    double succ = errModel->GetChunkSuccessRate(
        mode,
        txVector,
        snrLin,
        nbits,
        chWidth,
        WIFI_PPDU_FIELD_DATA,
        0 /* receiverId, not needed here */);

    double berLinear = (1.0 - succ) / std::max<uint64_t>(1, nbits);
    if (berLinear <= 0.0)
    {
        berLinear = std::numeric_limits<double>::min();
    }
    double berDb = 10.0 * std::log10(berLinear);

    // Modulation class name (string)
    std::string modName;
    switch (mode.GetModulationClass())
    {
        case WIFI_MOD_CLASS_DSSS: modName = "DSSS"; break;
        case WIFI_MOD_CLASS_OFDM: modName = "OFDM"; break;
        case WIFI_MOD_CLASS_HT:   modName = "HT";   break;
        case WIFI_MOD_CLASS_VHT:  modName = "VHT";  break;
        case WIFI_MOD_CLASS_HE:   modName = "HE";   break;
        default:                  modName = "UNKNOWN"; break;
    }

    double phyRateMbps = mode.GetDataRate(txVector) / 1e6;
    uint16_t constSize = mode.GetConstellationSize();

    (*csv) << std::scientific << std::setprecision(8)
           << timeNow << ","
           << staIndex << ","
           << channelFreqMhz << ","
           << modName << ","
           << constSize << ","
           << phyRateMbps << ","
           << rssi << ","
           << noise << ","
           << snrDb << ","
           << berLinear << ","
           << berDb << ","
           << randSeed << ","
           << runTag << "\n";
}

// ========================================================
// (The rest of the file follows in Part 2 and Part 3)
// ========================================================
// ========================================================
// Node density change controller
// ========================================================
void
ChangeActiveStations(ApplicationContainer &apps,
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

    if (newN != (int)curN)
    {
        if (!densityLog.empty())
        {
            densityLog.back().end = now;
        }

        densityLog.push_back({now, now + interval, (uint32_t)newN});
        curN = (uint32_t)newN;

        for (uint32_t i = 0; i < apps.GetN(); ++i)
        {
            apps.Get(i)->SetAttribute(
                (i < curN) ? "StartTime" : "StopTime",
                TimeValue(Seconds(now)));
        }
    }

    Simulator::Schedule(Seconds(interval),
                        &ChangeActiveStations,
                        std::ref(apps),
                        std::ref(curN),
                        nMin,
                        nMax,
                        seed + 1,
                        interval);
}

// ========================================================
// Wi-Fi BSS setup
// ========================================================
struct WifiBss
{
    NodeContainer ap, stas;
    NetDeviceContainer apDev, staDevs;
    Ipv4InterfaceContainer apIf, staIf;
};

WifiBss
SetupWifiBss(std::string subnet,
             double txPower,
             double areaHalf,
             uint32_t nSta,
             bool indoor)
{
    WifiBss b;
    b.ap.Create(1);
    b.stas.Create(nSta);

    WifiHelper wifi;
    wifi.SetStandard(WIFI_STANDARD_80211ac);

    YansWifiChannelHelper chan = YansWifiChannelHelper::Default();
    chan.AddPropagationLoss("ns3::LogDistancePropagationLossModel",
                            "Exponent",
                            DoubleValue(3.0));
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

    mac.SetType("ns3::ApWifiMac",
                "Ssid", SsidValue(ssid));
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
// UDP traffic
// ========================================================
ApplicationContainer
InstallTraffic(WifiBss &b,
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
    for (uint32_t i = 0; i < b.stas.GetN(); ++i)
    {
        apps.Add(client.Install(b.stas.Get(i)));
    }

    UdpServerHelper srv(port);
    auto s = srv.Install(b.ap.Get(0));

    apps.Start(Seconds(2.0));
    apps.Stop(Seconds(simTime));
    s.Start(Seconds(1.0));
    s.Stop(Seconds(simTime));

    return apps;
}

// ========================================================
// MAIN
// ========================================================
int
main(int argc, char *argv[])
{
    uint32_t nMin      = 5;
    uint32_t nMax      = 30;
    double   simTime   = 30.0;
    double   txPower   = 20.0;
    double   area      = 50.0;
    uint32_t seed      = 10;
    double   changeInt = 5.0;
    uint32_t pktSize   = 1024;
    double   clientIntvMs = 10.0;
    bool     indoor    = true;

    CommandLine cmd;
    cmd.AddValue("nStaMin", "", nMin);
    cmd.AddValue("nStaMax", "", nMax);
    cmd.AddValue("simTime", "", simTime);
    cmd.AddValue("txPower", "", txPower);
    cmd.AddValue("seed",    "", seed);
    cmd.AddValue("densityChangeInterval", "", changeInt);
    cmd.Parse(argc, argv);

    // Build run tag: DD-Mon-YYYY_HH-MM
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

    // RNG seed & initial active nodes
    RngSeedManager::SetSeed(seed);
    Ptr<UniformRandomVariable> rnd = CreateObject<UniformRandomVariable>();
    rnd->SetAttribute("Min", DoubleValue(nMin));
    rnd->SetAttribute("Max", DoubleValue(nMax + 1));
    activeNodes = rnd->GetInteger();
    if (activeNodes < 1)
    {
        activeNodes = 1;
    }

    // Setup Wi-Fi BSS
    WifiBss bss = SetupWifiBss("10.1.3.0", txPower, area, nMax, indoor);

    // Install traffic
    ApplicationContainer apps =
        InstallTraffic(bss, 9999, pktSize, clientIntvMs, simTime);

    // Initialize density intervals
    densityLog.clear();
    densityLog.push_back({0.0, changeInt, activeNodes});

    // Schedule dynamic node-density changes
    Simulator::Schedule(Seconds(changeInt),
                        &ChangeActiveStations,
                        std::ref(apps),
                        std::ref(activeNodes),
                        nMin,
                        nMax,
                        seed,
                        changeInt);

    // Flow monitor
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll();

    // ----------------------------------------------------
    // CSV tracers with per-STA capture
    // ----------------------------------------------------
    std::ofstream rssi((prefix + "-rssi.csv").c_str());
    rssi << "time_s,StaId,channel_MHz,signal_dBm,noise_dBm,SNR_dB,"
            "RandSeed,RunDateTime\n";

    std::ofstream mod((prefix + "-modulation.csv").c_str());
    mod << "time_s,StaId,channel_MHz,Modulation,ConstellationSize,"
           "PhyRate_Mbps,signal_dBm,noise_dBm,SNR_dB,BERLinear,BER_dB,"
           "RandSeed,RunDateTime\n";

    // Attach tracers to each STA WifiPhy
    for (uint32_t i = 0; i < bss.stas.GetN(); ++i)
    {
        Ptr<Node> node = bss.stas.Get(i);
        // assume device index 0 is the Wi-Fi device for this STA
        Ptr<NetDevice> dev = node->GetDevice(0);
        Ptr<WifiNetDevice> wifiDev = DynamicCast<WifiNetDevice>(dev);
        if (!wifiDev)
        {
            NS_LOG_WARN("Node " << node->GetId()
                         << " device 0 is not a WifiNetDevice");
            continue;
        }
        Ptr<WifiPhy> phy = wifiDev->GetPhy();

        phy->TraceConnectWithoutContext(
            "MonitorSnifferRx",
            MakeBoundCallback(&RssiTracer, i, seed, runTag, &rssi));

        phy->TraceConnectWithoutContext(
            "MonitorSnifferRx",
            MakeBoundCallback(&ModulationBerTracer, i, seed, runTag, &mod));
    }

    Simulator::Stop(Seconds(simTime + 1.0));
    Simulator::Run();

    // Close trace files before post-processing
    rssi.close();
    mod.close();

    // Post-processing (PERF + nodedensity + per-STA aggregates)
    // implemented in Part 3.

    // (Do not return here; Part 3 continues from this point.)
    // ====================================================
    // PERF CSV
    // ====================================================
    monitor->CheckForLostPackets();
    auto stats = monitor->GetFlowStats();
    Ptr<Ipv4FlowClassifier> cls =
        DynamicCast<Ipv4FlowClassifier>(flowmon.GetClassifier());

    std::ofstream perf((prefix + "-perf.csv").c_str());
    perf << "FlowID,Source,Destination,Throughput(Mbps),"
            "Latency_avg(ms),Jitter_avg(ms),PacketLoss(%),"
            "Duration_s,RandSeed,RunDateTime\n";

    for (auto &kv : stats)
    {
        const FlowMonitor::FlowStats &st = kv.second;
        auto t = cls->FindFlow(kv.first);

        double d = (st.timeLastRxPacket - st.timeFirstRxPacket).GetSeconds();
        double thr = (d > 0.0 && st.rxBytes > 0)
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
    // LOAD modulation.csv  → modData: (t, rssi, noise, snr, berLin, staId)
    // =====================================================
    std::vector<
        std::tuple<double,double,double,double,double,uint32_t>
    > modData;

    {
        std::ifstream mf((prefix + "-modulation.csv").c_str());
        std::string line;
        std::getline(mf, line); // skip header

        while (std::getline(mf, line))
        {
            if (line.empty())
                continue;

            std::stringstream ss(line);
            std::string field;

            // time_s
            if (!std::getline(ss, field, ',')) continue;
            double t = std::atof(field.c_str());

            // StaId
            if (!std::getline(ss, field, ',')) continue;
            uint32_t staId = static_cast<uint32_t>(std::atoi(field.c_str()));

            // channel_MHz
            if (!std::getline(ss, field, ',')) continue;
            // std::string chStr = field; // not used

            // Modulation
            if (!std::getline(ss, field, ',')) continue;
            // std::string modName = field; // not used

            // ConstellationSize
            if (!std::getline(ss, field, ',')) continue;
            // int constSize = std::atoi(field.c_str()); // not used

            // PhyRate_Mbps
            if (!std::getline(ss, field, ',')) continue;
            // double phyRate = std::atof(field.c_str()); // not used

            // signal_dBm
            if (!std::getline(ss, field, ',')) continue;
            double sig = std::atof(field.c_str());

            // noise_dBm
            if (!std::getline(ss, field, ',')) continue;
            double noi = std::atof(field.c_str());

            // SNR_dB
            if (!std::getline(ss, field, ',')) continue;
            double snr = std::atof(field.c_str());

            // BERLinear
            if (!std::getline(ss, field, ',')) continue;
            double berLin = std::atof(field.c_str());

            // BER_dB (ignore)
            if (!std::getline(ss, field, ',')) {
                // no more fields → still okay
            }

            // RandSeed, RunDateTime may follow but are ignored here

            modData.push_back(std::make_tuple(t, sig, noi, snr, berLin, staId));
        }
    }

    // =====================================================
    // NODE DENSITY CSV with per-STA aggregates
    // =====================================================
    const uint32_t totalSta = bss.stas.GetN();

    std::ofstream nd((prefix + "-nodedensity.csv").c_str());

    // Dynamic header:
    nd << "StartDateTime,EndDateTime,Duration_s,NodeDensity,"
          "TotalTxPackets,TotalRxPackets,AvgThroughput(Mbps),"
          "PacketLoss(%),AvgLatency(ms),AvgJitter(ms),"
          "GlobalAvgRSSI(dBm),GlobalAvgSNR(dB),GlobalAvgBER";

    // Per-STA RSSI columns
    for (uint32_t s = 0; s < totalSta; ++s)
    {
        nd << ",RSSI_STA_" << s;
    }
    // Per-STA SNR columns
    for (uint32_t s = 0; s < totalSta; ++s)
    {
        nd << ",SNR_STA_" << s;
    }
    // Per-STA BER columns
    for (uint32_t s = 0; s < totalSta; ++s)
    {
        nd << ",BER_STA_" << s;
    }
    nd << "\n";

    time_t base = time(nullptr);

    // Forward-fill memory for global PHY values
    double lastRssi = 0.0;
    double lastSnr  = 0.0;
    double lastBer  = 0.0;

    for (auto &rec : densityLog)
    {
        double st = rec.start;
        double en = rec.end;
        if (en > simTime)
        {
            en = simTime;
        }

        double dur = en - st;
        if (dur <= 0.0)
        {
            continue;
        }

        // --------------------------------------------
        // Aggregate FlowMonitor stats in [st, en)
        // --------------------------------------------
        uint64_t txTot = 0;
        uint64_t rxTot = 0;
        double   sumThr = 0.0;
        double   sumLat = 0.0;
        double   sumJit = 0.0;
        int      flows   = 0;

        for (auto &kv : stats)
        {
            const FlowMonitor::FlowStats &s = kv.second;

            if (s.txPackets == 0 || s.rxPackets == 0)
                continue;

            if (s.timeLastRxPacket.GetSeconds() < st ||
                s.timeFirstTxPacket.GetSeconds() > en)
            {
                continue;
            }

            double d = (s.timeLastRxPacket - s.timeFirstTxPacket).GetSeconds();
            if (d <= 0.0)
                continue;

            double thr = (s.rxBytes * 8.0) / d / 1e6;
            double lat = (s.delaySum.GetSeconds() / s.rxPackets) * 1000.0;
            double jit = (s.jitterSum.GetSeconds() / s.rxPackets) * 1000.0;

            sumThr += thr;
            sumLat += lat;
            sumJit += jit;

            txTot += s.txPackets;
            rxTot += s.rxPackets;
            flows++;
        }

        double avgThr   = (flows > 0) ? sumThr / flows : 0.0;
        double avgLat   = (flows > 0) ? sumLat / flows : 0.0;
        double avgJit   = (flows > 0) ? sumJit / flows : 0.0;
        double lossPct  = (txTot > 0) ? 100.0 * (txTot - rxTot) / txTot : 0.0;

        // --------------------------------------------
        // PER-STA PHY AGGREGATION in [st, en)
        // modData: (t, rssi, noise, snr, berLin, staId)
        // --------------------------------------------
        std::vector<double> staSumR(totalSta, 0.0);
        std::vector<double> staSumS(totalSta, 0.0);
        std::vector<double> staSumB(totalSta, 0.0);
        std::vector<int>    staCnt (totalSta, 0);

        double sR = 0.0, sS = 0.0, sB = 0.0;
        int    rc = 0;

        for (auto &x : modData)
        {
            double   t     = std::get<0>(x);
            if (t < st || t >= en)
                continue;

            double   rssi  = std::get<1>(x);
            double   snr   = std::get<3>(x);
            double   ber   = std::get<4>(x);
            uint32_t staId = std::get<5>(x);

            if (staId >= totalSta)
                continue;

            staSumR[staId] += rssi;
            staSumS[staId] += snr;
            staSumB[staId] += ber;
            staCnt [staId]++;

            sR += rssi;
            sS += snr;
            sB += ber;
            rc++;
        }

        // Global averages (with forward-fill)
        double gR, gS, gB;
        if (rc > 0)
        {
            gR = sR / rc;
            gS = sS / rc;
            gB = (sB / rc) * (1.0 + lossPct / 100.0);

            lastRssi = gR;
            lastSnr  = gS;
            lastBer  = gB;
        }
        else
        {
            gR = lastRssi;
            gS = lastSnr;
            gB = lastBer;
        }

        // Per-STA averages (fallback to global if STA has no samples)
        std::vector<double> staAvgR(totalSta, 0.0);
        std::vector<double> staAvgS(totalSta, 0.0);
        std::vector<double> staAvgB(totalSta, 0.0);

        for (uint32_t s = 0; s < totalSta; ++s)
        {
            if (staCnt[s] > 0)
            {
                staAvgR[s] = staSumR[s] / staCnt[s];
                staAvgS[s] = staSumS[s] / staCnt[s];
                staAvgB[s] = (staSumB[s] / staCnt[s]) * (1.0 + lossPct / 100.0);
            }
            else
            {
                // no per-STA samples in this interval -> use global
                staAvgR[s] = gR;
                staAvgS[s] = gS;
                staAvgB[s] = gB;
            }
        }

        // --------------------------------------------
        // Write row to nodedensity.csv
        // --------------------------------------------
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
           << gR << "," << gS << "," << gB;

        // Per-STA RSSI
        for (uint32_t s = 0; s < totalSta; ++s)
        {
            nd << "," << staAvgR[s];
        }
        // Per-STA SNR
        for (uint32_t s = 0; s < totalSta; ++s)
        {
            nd << "," << staAvgS[s];
        }
        // Per-STA BER
        for (uint32_t s = 0; s < totalSta; ++s)
        {
            nd << "," << staAvgB[s];
        }

        nd << "\n";
    }

    nd.close();

    Simulator::Destroy();
    return 0;
}
