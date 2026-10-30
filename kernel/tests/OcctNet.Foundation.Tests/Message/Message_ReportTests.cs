// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Tests of Message_Report, the alerts and their attributes (no GTests in OCCT).

using System.Text;
using static Occt.Message_Gravity;
using static Occt.Message_MetricType;

namespace OcctNet.Foundation.Tests;

internal sealed class TestAlertA : Message_Alert;

internal sealed class TestAlertB : Message_Alert;

internal class TestAlertBase : Message_Alert;

internal sealed class TestAlertDerived : TestAlertBase;

/// <summary>An alert that never merges, as the alerts with shapes.</summary>
internal sealed class TestAlertNoMerge : Message_Alert
{
    public override bool SupportsMerge() => false;
}

/// <summary>A printer that records what it is sent.</summary>
internal sealed class ReportTestPrinter : Message_Printer
{
    public readonly List<(string Text, Message_Gravity Gravity)> Messages = [];

    public ReportTestPrinter() => SetTraceLevel(Message_Trace);

    protected override void send(string theString, Message_Gravity theGravity) => Messages.Add((theString, theGravity));
}

public class Message_ReportTests
{
    [Fact]
    public void AddAlert_ListsByGravity()
    {
        Message_Report aReport = new();
        Assert.True(aReport.GetAlerts(Message_Fail).IsEmpty());
        TestAlertNoMerge aFail = new();
        TestAlertNoMerge aWarning = new();
        aReport.AddAlert(Message_Fail, aFail);
        aReport.AddAlert(Message_Warning, aWarning);
        Assert.Equal(1, aReport.GetAlerts(Message_Fail).Extent());
        Assert.Same(aFail, aReport.GetAlerts(Message_Fail).First());
        Assert.Same(aWarning, aReport.GetAlerts(Message_Warning).First());
        Assert.True(aReport.GetAlerts(Message_Info).IsEmpty());
        Assert.Equal(-1, aReport.Limit());
    }

    [Fact]
    public void HasAlert_IsExactType()
    {
        Message_Report aReport = new();
        aReport.AddAlert(Message_Warning, new TestAlertDerived());
        Assert.True(aReport.HasAlert(typeof(TestAlertDerived)));
        Assert.True(aReport.HasAlert(typeof(TestAlertDerived), Message_Warning));
        Assert.False(aReport.HasAlert(typeof(TestAlertDerived), Message_Fail));
        // IsInstance, as OCCT: a base type doesn't match
        Assert.False(aReport.HasAlert(typeof(TestAlertBase)));
        Assert.False(aReport.HasAlert(typeof(Message_Alert)));
    }

    [Fact]
    public void AddAlert_MergingRemovesTheFirstAlert_OcctIssue()
    {
        // OCCT: a simple alert merges with one of the same type, and Message_Report::AddAlert then removes the first
        // alert of the gravity because the default limit is -1 (occt-issues.md, unit 6A)
        Message_Report aReport = new();
        TestAlertA a1 = new(), a2 = new(), a3 = new();
        aReport.AddAlert(Message_Fail, a1);
        Assert.Equal(1, aReport.GetAlerts(Message_Fail).Extent());
        aReport.AddAlert(Message_Fail, a2); // merged into a1, then a1 removed
        Assert.True(aReport.GetAlerts(Message_Fail).IsEmpty());
        Assert.False(aReport.HasAlert(typeof(TestAlertA)));
        aReport.AddAlert(Message_Fail, a3); // appended again
        Assert.Same(a3, aReport.GetAlerts(Message_Fail).First());

        // with another type first, the merge of a second A removes that other alert, not the A
        Message_Report aReport2 = new();
        TestAlertB b = new();
        aReport2.AddAlert(Message_Warning, b);
        aReport2.AddAlert(Message_Warning, new TestAlertA());
        aReport2.AddAlert(Message_Warning, new TestAlertA());
        Assert.Equal(1, aReport2.GetAlerts(Message_Warning).Extent());
        Assert.True(aReport2.HasAlert(typeof(TestAlertA)));
        Assert.False(aReport2.HasAlert(typeof(TestAlertB)));
    }

    [Fact]
    public void AddAlert_LimitAppliesOnlyOnMerge_OcctIssue()
    {
        Message_Report aReport = new();
        aReport.SetLimit(1);
        for (int i = 0; i < 5; ++i)
        {
            aReport.AddAlert(Message_Info, new TestAlertNoMerge());
        }
        // appended alerts are never limited
        Assert.Equal(5, aReport.GetAlerts(Message_Info).Extent());
    }

    [Fact]
    public void CompositeAlerts_MergeAndRemove()
    {
        Message_CompositeAlerts aComposite = new();
        TestAlertA a1 = new();
        Assert.True(aComposite.AddAlert(Message_Info, a1));
        Assert.False(aComposite.AddAlert(Message_Info, new TestAlertA())); // merged
        Assert.True(aComposite.AddAlert(Message_Info, new TestAlertB()));
        Assert.Equal(2, aComposite.Alerts(Message_Info).Extent());
        Assert.True(aComposite.HasAlert(a1));
        Assert.True(aComposite.RemoveAlert(Message_Info, a1));
        Assert.False(aComposite.RemoveAlert(Message_Info, a1));
        Assert.False(aComposite.HasAlert(a1));
        Assert.False(aComposite.AddAlert(Message_Info, null));
        Assert.True(aComposite.Alerts((Message_Gravity)42).IsEmpty());
    }

    [Fact]
    public void Clear_AllGravityAndType()
    {
        Message_Report aReport = new();
        aReport.Clear(); // nothing yet
        aReport.AddAlert(Message_Fail, new TestAlertA());
        aReport.AddAlert(Message_Fail, new TestAlertNoMerge());
        aReport.AddAlert(Message_Fail, new TestAlertB());
        aReport.AddAlert(Message_Warning, new TestAlertA());

        // OCCT's loop would never end on the alerts of other types (occt-issues.md, unit 6A); the port advances
        aReport.Clear(typeof(TestAlertA));
        Assert.Equal(2, aReport.GetAlerts(Message_Fail).Extent());
        Assert.True(aReport.GetAlerts(Message_Warning).IsEmpty());
        Assert.True(aReport.HasAlert(typeof(TestAlertB), Message_Fail));

        aReport.AddAlert(Message_Warning, new TestAlertA());
        aReport.Clear(Message_Fail);
        Assert.True(aReport.GetAlerts(Message_Fail).IsEmpty());
        Assert.False(aReport.GetAlerts(Message_Warning).IsEmpty());

        aReport.Clear();
        Assert.True(aReport.GetAlerts(Message_Warning).IsEmpty());
    }

    [Fact]
    public void Merge_AddsTheOtherReportsAlerts()
    {
        Message_Report aReport = new();
        Message_Report anOther = new();
        anOther.AddAlert(Message_Fail, new TestAlertNoMerge());
        anOther.AddAlert(Message_Warning, new TestAlertB());
        aReport.AddAlert(Message_Warning, new TestAlertA());
        aReport.Merge(anOther);
        Assert.Equal(1, aReport.GetAlerts(Message_Fail).Extent());
        Assert.Equal(2, aReport.GetAlerts(Message_Warning).Extent());

        Message_Report aFails = new();
        aFails.Merge(anOther, Message_Fail);
        Assert.True(aFails.GetAlerts(Message_Warning).IsEmpty());
        Assert.Equal(1, aFails.GetAlerts(Message_Fail).Extent());
    }

    [Fact]
    public void Dump_WritesMessageKeysByGravity()
    {
        Message_Report aReport = new();
        aReport.AddAlert(Message_Fail, new TestAlertA());
        aReport.AddAlert(Message_Warning, new TestAlertB());
        Message_AlertExtended.AddAlert(aReport, new Message_Attribute("named"), Message_Fail);
        Message_AlertExtended.AddAlert(aReport, null, Message_Fail);

        StringWriter aFails = new();
        aReport.Dump(aFails, Message_Fail);
        Assert.Equal("TestAlertA\nnamed\nMessage_AlertExtended\n", aFails.ToString());

        StringWriter anAll = new();
        aReport.Dump(anAll);
        Assert.Equal("TestAlertB\nTestAlertA\nnamed\nMessage_AlertExtended\n", anAll.ToString());
    }

    [Fact]
    public void SendMessages_SendsKeysWithGravity()
    {
        Message_Report aReport = new();
        aReport.AddAlert(Message_Fail, new TestAlertA());
        aReport.AddAlert(Message_Trace, new TestAlertB());
        ReportTestPrinter aPrinter = new();
        Message_Messenger aMessenger = new(aPrinter);
        aReport.SendMessages(aMessenger);
        Assert.Equal([("TestAlertB", Message_Trace), ("TestAlertA", Message_Fail)], aPrinter.Messages);
    }

    [Fact]
    public void AlertExtended_ChildrenAndMerge()
    {
        Message_AlertExtended anAlert = new();
        Assert.Null(anAlert.CompositeAlerts());
        Assert.True(anAlert.SupportsMerge());
        Assert.False(anAlert.Merge(new Message_AlertExtended()));
        anAlert.CompositeAlerts(true)!.AddAlert(Message_Info, new TestAlertA());
        Assert.False(anAlert.SupportsMerge()); // hierarchical alerts can't merge

        // children are dumped below their parent
        Message_Report aReport = new();
        aReport.AddAlert(Message_Info, anAlert);
        StringWriter aText = new();
        aReport.Dump(aText, Message_Info);
        Assert.Equal("Message_AlertExtended\nTestAlertA\n", aText.ToString());
    }

    [Fact]
    public void Attributes()
    {
        Assert.Equal("", new Message_Attribute().GetMessageKey());
        Message_AttributeObject anObject = new(new Message_Report(), "obj");
        Assert.IsType<Message_Report>(anObject.Object());
        Assert.Equal("obj", anObject.GetName());
        Message_AttributeStream aStream = new("text", "s");
        Assert.Equal("text", aStream.Stream());
        aStream.SetStream("other");
        Assert.Equal("other", aStream.Stream());

        Message_AttributeMeter aMeter = new("m");
        Assert.False(aMeter.HasMetric(Message_MetricType_WallClock));
        Assert.Equal(Message_AttributeMeter.UndefinedMetricValue(), aMeter.StartValue(Message_MetricType_WallClock));
        aMeter.SetStopValue(Message_MetricType_WallClock, 2.0); // no metric yet: ignored
        Assert.False(aMeter.HasMetric(Message_MetricType_WallClock));
        aMeter.SetStartValue(Message_MetricType_WallClock, 1.0);
        Assert.False(aMeter.IsMetricValid(Message_MetricType_WallClock));
        aMeter.SetStopValue(Message_MetricType_WallClock, 3.0);
        Assert.True(aMeter.IsMetricValid(Message_MetricType_WallClock));
        Assert.Equal(1.0, aMeter.StartValue(Message_MetricType_WallClock));
        Assert.Equal(3.0, aMeter.StopValue(Message_MetricType_WallClock));
    }

    [Fact]
    public void MetricNames()
    {
        Assert.Equal("WallClock", Message.MetricToString(Message_MetricType_WallClock));
        Assert.Equal(Message_MetricType_MemHeapUsage, Message.MetricFromString("MemHeapUsage"));
        Assert.Equal(Message_MetricType_None, Message.MetricFromString("nonsense"));
        Assert.False(Message.MetricFromString("nonsense", out _));
    }

    [Fact]
    public void ActivateInMessenger_PrinterToReport()
    {
        Message_Report aReport = new();
        Message_Messenger aMessenger = new(new ReportTestPrinter());
        Assert.False(aReport.IsActiveInMessenger());
        aReport.ActivateInMessenger(true, aMessenger);
        Assert.True(aReport.IsActiveInMessenger());
        Assert.Equal(2, aMessenger.Printers().Length());

        // a string sent to the messenger becomes an alert named by it
        aMessenger.Send("hello", Message_Warning);
        Assert.Equal("hello", aReport.GetAlerts(Message_Warning).First().GetMessageKey());

        // a stream is held back as the name of the next alert
        aMessenger.Send(new StringBuilder("first"), Message_Info);
        Assert.True(aReport.GetAlerts(Message_Info).IsEmpty());
        aMessenger.Send(new StringBuilder("second"), Message_Info);
        Assert.Equal("first", aReport.GetAlerts(Message_Info).First().GetMessageKey());

        // an object becomes an alert with an object attribute named by the held-back stream
        Message_Report aPayload = new();
        aMessenger.Send(aPayload, Message_Alarm);
        var anObjectAlert = Assert.IsType<Message_AlertExtended>(aReport.GetAlerts(Message_Alarm).First());
        var anAttribute = Assert.IsType<Message_AttributeObject>(anObjectAlert.Attribute());
        Assert.Same(aPayload, anAttribute.Object());
        Assert.Equal("second", anAttribute.GetName());

        aReport.UpdateActiveInMessenger(aMessenger);
        Assert.True(aReport.IsActiveInMessenger());
        aReport.ActivateInMessenger(false, aMessenger);
        Assert.False(aReport.IsActiveInMessenger());
        Assert.Equal(1, aMessenger.Printers().Length());
        aReport.UpdateActiveInMessenger(aMessenger);
        Assert.False(aReport.IsActiveInMessenger());
    }

    [Fact]
    public void ThreadSafeAddAlert()
    {
        Message_Report aReport = new();
        Parallel.For(0, 1000, _ => aReport.AddAlert(Message_Warning, new TestAlertNoMerge()));
        Assert.Equal(1000, aReport.GetAlerts(Message_Warning).Extent());
    }
}

/// <summary>The tests that use the process-wide default report, one at a time.</summary>
[Collection(nameof(DefaultReportCollection))]
public class Message_LevelTests
{
    [Fact]
    public void Levels_NestAlertsUnderTheirRoots()
    {
        Message_Report aReport = new();
        Message_Messenger aMessenger = new(new ReportTestPrinter());
        Message.SetDefaultReport(aReport);
        try
        {
            // the report must be active for levels to open
            using (new Message_Level("inactive"))
            {
            }
            Assert.True(aReport.GetAlerts(Message_Info).IsEmpty());

            aReport.ActivateInMessenger(true, aMessenger);
            using (Message_Level aLevel1 = new("level1"))
            {
                aReport.AddAlert(Message_Info, Message_AlertExtended.AddAlert(new Message_Report(), new Message_Attribute("a"), Message_Info));
                using (new Message_Level("level2"))
                {
                    Message_AlertExtended.AddAlert(aReport, new Message_Attribute("b"), Message_Warning);
                    // a simple alert can't go below a level: Message_Level::AddAlert drops it
                    aReport.AddAlert(Message_Fail, new TestAlertA());
                }
                Assert.Equal("level1", aLevel1.RootAlert()!.GetMessageKey());
            }
            Message_AlertExtended.AddAlert(aReport, new Message_Attribute("top"), Message_Info);

            StringWriter anInfo = new();
            aReport.Dump(anInfo, Message_Info);
            Assert.Equal("level1\na\nlevel2\ntop\n", anInfo.ToString());
            StringWriter aWarning = new();
            aReport.Dump(aWarning, Message_Warning);
            Assert.Equal("", aWarning.ToString()); // the warning is a child of level2, in its Warning list
            Message_AlertExtended aRoot1 = (Message_AlertExtended)aReport.GetAlerts(Message_Info).First();
            Message_AlertExtended aRoot2 = (Message_AlertExtended)aRoot1.CompositeAlerts()!.Alerts(Message_Info).Last();
            Assert.Equal("b", aRoot2.CompositeAlerts()!.Alerts(Message_Warning).First().GetMessageKey());
            Assert.True(aReport.GetAlerts(Message_Fail).IsEmpty());
        }
        finally
        {
            Message.SetDefaultReport(null);
        }
    }

    [Fact]
    public void Levels_RecordMetrics()
    {
        Message_Report aReport = new();
        aReport.SetActiveMetric(Message_MetricType_WallClock, true);
        aReport.SetActiveMetric(Message_MetricType_MemHeapUsage, true);
        aReport.SetActiveMetric(Message_MetricType_WallClock, true); // already active
        Assert.Equal(2, aReport.ActiveMetrics().Extent());
        Message_Messenger aMessenger = new(new ReportTestPrinter());
        Message.SetDefaultReport(aReport);
        try
        {
            aReport.ActivateInMessenger(true, aMessenger);
            using (new Message_Level("timed"))
            {
                Message_AlertExtended.AddAlert(aReport, new Message_AttributeMeter("step"), Message_Info);
            }
            Message_AlertExtended aRoot = (Message_AlertExtended)aReport.GetAlerts(Message_Info).First();
            Message_AttributeMeter aMeter = Assert.IsType<Message_AttributeMeter>(aRoot.Attribute());
            Assert.True(aMeter.IsMetricValid(Message_MetricType_WallClock));
            Assert.True(aMeter.StopValue(Message_MetricType_WallClock) >= aMeter.StartValue(Message_MetricType_WallClock));
            Assert.True(aMeter.HasMetric(Message_MetricType_MemHeapUsage));
            Assert.False(aMeter.HasMetric(Message_MetricType_ThreadCPUUserTime));
        }
        finally
        {
            Message.SetDefaultReport(null);
        }
        aReport.ClearMetrics();
        Assert.True(aReport.ActiveMetrics().IsEmpty());
    }
}

[CollectionDefinition(nameof(DefaultReportCollection), DisableParallelization = true)]
public sealed class DefaultReportCollection;
