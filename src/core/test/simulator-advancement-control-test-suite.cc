/*
 * Copyright (c) 2018. Lawrence Livermore National Security, LLC.
 *
 * SPDX-License-Identifier: GPL-2.0-only
 *
 *
 *
 * Author: Steven Smith <smith84@llnl.gov>
 */
#include "ns3/calendar-scheduler.h"
#include "ns3/heap-scheduler.h"
#include "ns3/list-scheduler.h"
#include "ns3/map-scheduler.h"
#include "ns3/simulator.h"
#include "ns3/test.h"

using namespace ns3;

class SimulatorAdvancementControlTestCase : public TestCase
{
  public:
    SimulatorAdvancementControlTestCase(void);
    virtual void DoRun(void);
    void EventA(void);
    void EventB(void);
    void EventC(void);
    void EventD(void);
    bool m_b;
    bool m_a;
    bool m_c;
    bool m_d;
};

SimulatorAdvancementControlTestCase::SimulatorAdvancementControlTestCase(void)
    : TestCase("Check simulator time advancement control via run and stop is working")
{
}

void
SimulatorAdvancementControlTestCase::EventA(void)
{
    m_a = true;
}

void
SimulatorAdvancementControlTestCase::EventB(void)
{
    m_b = true;
}

void
SimulatorAdvancementControlTestCase::EventC(void)
{
    m_c = true;
}

void
SimulatorAdvancementControlTestCase::EventD(void)
{
    m_d = true;
}

void
SimulatorAdvancementControlTestCase::DoRun(void)
{
    /* Test default run */
    m_a = false;
    m_b = false;
    m_c = false;
    m_d = false;

    Simulator::Schedule(MicroSeconds(1), &SimulatorAdvancementControlTestCase::EventA, this);
    Simulator::Schedule(MicroSeconds(2), &SimulatorAdvancementControlTestCase::EventB, this);
    Simulator::Schedule(MicroSeconds(3), &SimulatorAdvancementControlTestCase::EventC, this);
    Simulator::Schedule(MicroSeconds(4), &SimulatorAdvancementControlTestCase::EventD, this);

    Simulator::Run();

    NS_TEST_EXPECT_MSG_EQ(m_a, true, "Event A did not run ?");
    NS_TEST_EXPECT_MSG_EQ(m_b, true, "Event B did not run ?");
    NS_TEST_EXPECT_MSG_EQ(m_c, true, "Event C did not run ?");
    NS_TEST_EXPECT_MSG_EQ(m_d, true, "Event D did not run ?");

    NS_TEST_EXPECT_MSG_EQ(Simulator::IsFinished(), true, "Simulator is not finished ?");

    /* Test default (exclusive) run to specified time */
    m_a = false;
    m_b = false;
    m_c = false;
    m_d = false;

    Simulator::Schedule(MicroSeconds(1), &SimulatorAdvancementControlTestCase::EventA, this);
    Simulator::Schedule(MicroSeconds(2), &SimulatorAdvancementControlTestCase::EventB, this);
    Simulator::Schedule(MicroSeconds(3), &SimulatorAdvancementControlTestCase::EventC, this);
    Simulator::Schedule(MicroSeconds(4), &SimulatorAdvancementControlTestCase::EventD, this);

    int stage = 1;
    Simulator::Stop(MicroSeconds(2));
    Simulator::Run();
    NS_TEST_EXPECT_MSG_EQ(m_a, true, "Stage " << stage << " : Event A did not run ?");
    NS_TEST_EXPECT_MSG_EQ(m_b, false, "Stage " << stage << " : Event B ran ?");
    NS_TEST_EXPECT_MSG_EQ(m_c, false, "Stage " << stage << " : Event C ran ?");
    NS_TEST_EXPECT_MSG_EQ(m_d, false, "Stage " << stage << " : Event D ran ?");

    ++stage;
    Simulator::Stop(MicroSeconds(1));
    Simulator::Run();
    NS_TEST_EXPECT_MSG_EQ(m_b, true, "Stage " << stage << " : Event B did not run ?");
    NS_TEST_EXPECT_MSG_EQ(m_c, false, "Stage " << stage << " : Event C ran ?");
    NS_TEST_EXPECT_MSG_EQ(m_d, false, "Stage " << stage << " : Event D ran ?");

    ++stage;
    Simulator::Stop(MicroSeconds(1));
    Simulator::Run();
    NS_TEST_EXPECT_MSG_EQ(m_c, true, "Stage " << stage << " : Event C did not run ?");
    NS_TEST_EXPECT_MSG_EQ(m_d, false, "Stage " << stage << " : Event D ran ?");

    ++stage;
    Simulator::Stop(MicroSeconds(1));
    Simulator::Run();
    NS_TEST_EXPECT_MSG_EQ(m_d, true, "Stage " << stage << " : Event D did not run ?");

    /* Test inclusive run to specified time */
    m_a = false;
    m_b = false;
    m_c = false;
    m_d = false;

    Simulator::Schedule(MicroSeconds(1), &SimulatorAdvancementControlTestCase::EventA, this);
    Simulator::Schedule(MicroSeconds(2), &SimulatorAdvancementControlTestCase::EventB, this);
    Simulator::Schedule(MicroSeconds(3), &SimulatorAdvancementControlTestCase::EventC, this);
    Simulator::Schedule(MicroSeconds(4), &SimulatorAdvancementControlTestCase::EventD, this);

    ++stage;
    Simulator::Stop(MicroSeconds(2), Simulator::TimeWindowControl::INCLUSIVE);
    Simulator::Run();
    NS_TEST_EXPECT_MSG_EQ(m_a, true, "Stage " << stage << " : Event A did not run ?");
    NS_TEST_EXPECT_MSG_EQ(m_b, true, "Stage " << stage << " : Event B did not run ?");
    NS_TEST_EXPECT_MSG_EQ(m_c, false, "Stage " << stage << " : Event C ran ?");
    NS_TEST_EXPECT_MSG_EQ(m_d, false, "Stage " << stage << " : Event D ran ?");

    ++stage;
    Simulator::Stop(MicroSeconds(1), Simulator::TimeWindowControl::INCLUSIVE);
    Simulator::Run();
    NS_TEST_EXPECT_MSG_EQ(m_c, true, "Stage " << stage << " : Event C did not run ?");
    NS_TEST_EXPECT_MSG_EQ(m_d, false, "Stage " << stage << " : Event D ran ?");

    ++stage;
    Simulator::Stop(MicroSeconds(1), Simulator::TimeWindowControl::INCLUSIVE);
    Simulator::Run();
    NS_TEST_EXPECT_MSG_EQ(m_d, true, "Stage " << stage << " : Event D did not run ?");

    /* Step at interval smaller then scheduled events */
    m_a = false;
    m_b = false;
    m_c = false;
    m_d = false;

    Simulator::Schedule(MicroSeconds(1), &SimulatorAdvancementControlTestCase::EventA, this);
    Simulator::Schedule(MicroSeconds(4), &SimulatorAdvancementControlTestCase::EventB, this);

    ++stage;
    Time startTime = Simulator::Now();
    Simulator::Stop(MicroSeconds(1), Simulator::TimeWindowControl::EXCLUSIVE);
    Simulator::Run();
    NS_TEST_EXPECT_MSG_EQ(Simulator::Now(),
                          startTime + MicroSeconds(1),
                          "Stage " << stage << " : Current time is incorrect");
    NS_TEST_EXPECT_MSG_EQ(m_a, false, "Stage " << stage << " : Event A ran ?");
    NS_TEST_EXPECT_MSG_EQ(m_b, false, "Stage " << stage << " : Event B ran ?");

    ++stage;
    startTime = Simulator::Now();
    Simulator::Stop(MicroSeconds(1), Simulator::TimeWindowControl::EXCLUSIVE);
    Simulator::Run();
    NS_TEST_EXPECT_MSG_EQ(Simulator::Now(),
                          startTime + MicroSeconds(1),
                          "Stage " << stage << " : Current time is incorrect");
    NS_TEST_EXPECT_MSG_EQ(m_a, true, "Stage " << stage << " : Event A did not run ?");
    NS_TEST_EXPECT_MSG_EQ(m_b, false, "Stage " << stage << " : Event B ran ?");

    ++stage;
    startTime = Simulator::Now();
    Simulator::Stop(MicroSeconds(1), Simulator::TimeWindowControl::EXCLUSIVE);
    Simulator::Run();
    NS_TEST_EXPECT_MSG_EQ(Simulator::Now(),
                          startTime + MicroSeconds(1),
                          "Stage " << stage << " : Current time is incorrect");
    NS_TEST_EXPECT_MSG_EQ(m_b, false, "Stage " << stage << " : Event B ran ?");

    ++stage;
    startTime = Simulator::Now();
    Simulator::Stop(MicroSeconds(1), Simulator::TimeWindowControl::EXCLUSIVE);
    Simulator::Run();
    NS_TEST_EXPECT_MSG_EQ(Simulator::Now(),
                          startTime + MicroSeconds(1),
                          "Stage " << stage << " : Current time is incorrect");
    NS_TEST_EXPECT_MSG_EQ(m_b, false, "Stage " << stage << " : Event B ran ?");

    ++stage;
    startTime = Simulator::Now();
    Simulator::Stop(MicroSeconds(1), Simulator::TimeWindowControl::EXCLUSIVE);
    Simulator::Run();
    NS_TEST_EXPECT_MSG_EQ(Simulator::Now(),
                          startTime + MicroSeconds(1),
                          "Stage " << stage << " : Current time is incorrect");
    NS_TEST_EXPECT_MSG_EQ(m_b, true, "Stage " << stage << " : Event B did not run ?");

    /* Test multiple calls to Stop; earliest stop should control */
    m_a = false;
    m_b = false;
    m_c = false;
    m_d = false;

    Simulator::Schedule(MicroSeconds(1), &SimulatorAdvancementControlTestCase::EventA, this);
    Simulator::Schedule(MicroSeconds(2), &SimulatorAdvancementControlTestCase::EventB, this);
    Simulator::Schedule(MicroSeconds(3), &SimulatorAdvancementControlTestCase::EventC, this);
    Simulator::Schedule(MicroSeconds(4), &SimulatorAdvancementControlTestCase::EventD, this);

    ++stage;
    Simulator::Stop(MicroSeconds(2), Simulator::TimeWindowControl::INCLUSIVE);
    Simulator::Stop(MicroSeconds(1), Simulator::TimeWindowControl::INCLUSIVE);
    Simulator::Stop(MicroSeconds(1));
    Simulator::Run();
    NS_TEST_EXPECT_MSG_EQ(m_a, false, "Stage " << stage << " : Event A ran ?");
    NS_TEST_EXPECT_MSG_EQ(m_b, false, "Stage " << stage << " : Event B ran ?");
    NS_TEST_EXPECT_MSG_EQ(m_c, false, "Stage " << stage << " : Event C ran ?");
    NS_TEST_EXPECT_MSG_EQ(m_d, false, "Stage " << stage << " : Event D ran ?");

    /* Test running to completion after a previous stop */
    ++stage;
    Simulator::Run();
    NS_TEST_EXPECT_MSG_EQ(m_a, true, "Stage " << stage << " : Event A did not run ?");
    NS_TEST_EXPECT_MSG_EQ(m_b, true, "Stage " << stage << " : Event B did not run ?");
    NS_TEST_EXPECT_MSG_EQ(m_c, true, "Stage " << stage << " : Event C did not run ?");
    NS_TEST_EXPECT_MSG_EQ(m_d, true, "Stage " << stage << " : Event D did not run ?");

    Simulator::Destroy();
}

class SimulatorAdvancementControlTestSuite : public TestSuite
{
  public:
    SimulatorAdvancementControlTestSuite()
        : TestSuite("simulator-advancement-control")
    {
        AddTestCase(new SimulatorAdvancementControlTestCase(), TestCase::QUICK);
    }
} g_simulatorFederatedTestSuite;
