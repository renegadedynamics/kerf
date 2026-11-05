// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Tests of Message_Report, the alerts and their attributes (no GTests in OCCT).

using System.Text;
using static Kerf.Engine.Foundation.Gravity;
using static Kerf.Engine.Foundation.MetricType;

namespace Kerf.Kernel.Foundation.Tests;

internal sealed class TestAlertA : global::Kerf.Engine.Foundation.Alert;

internal sealed class TestAlertB : global::Kerf.Engine.Foundation.Alert;

internal class TestAlertBase : global::Kerf.Engine.Foundation.Alert;

internal sealed class TestAlertDerived : TestAlertBase;

/// <summary>An alert that never merges, as the alerts with shapes.</summary>
internal sealed class TestAlertNoMerge : global::Kerf.Engine.Foundation.Alert
{
    public override bool SupportsMerge() => false;
}

/// <summary>A printer that records what it is sent.</summary>
internal sealed class ReportTestPrinter : global::Kerf.Engine.Foundation.Printer
{
    public readonly List<(string Text, global::Kerf.Engine.Foundation.Gravity Gravity)> Messages = [];

    public ReportTestPrinter() => SetTraceLevel(Trace);

    protected override void SendCore(string theString, global::Kerf.Engine.Foundation.Gravity theGravity) => Messages.Add((theString, theGravity));
}

public class Message_ReportTests
{
    [Fact]
    public void AddAlert_ListsByGravity()
    {
        global::Kerf.Engine.Foundation.Report aReport = new();
        Assert.True(aReport.GetAlerts(Fail).IsEmpty());
        TestAlertNoMerge aFail = new();
        TestAlertNoMerge aWarning = new();
        aReport.AddAlert(Fail, aFail);
        aReport.AddAlert(Warning, aWarning);
        Assert.Equal(1, aReport.GetAlerts(Fail).Extent());
        Assert.Same(aFail, aReport.GetAlerts(Fail).First());
        Assert.Same(aWarning, aReport.GetAlerts(Warning).First());
        Assert.True(aReport.GetAlerts(Info).IsEmpty());
        Assert.Equal(-1, aReport.Limit());
    }

    [Fact]
    public void HasAlert_IsExactType()
    {
        global::Kerf.Engine.Foundation.Report aReport = new();
        aReport.AddAlert(Warning, new TestAlertDerived());
        Assert.True(aReport.HasAlert(typeof(TestAlertDerived)));
        Assert.True(aReport.HasAlert(typeof(TestAlertDerived), Warning));
        Assert.False(aReport.HasAlert(typeof(TestAlertDerived), Fail));
        // IsInstance, as OCCT: a base type doesn't match
        Assert.False(aReport.HasAlert(typeof(TestAlertBase)));
        Assert.False(aReport.HasAlert(typeof(global::Kerf.Engine.Foundation.Alert)));
    }

    [Fact]
    public void AddAlert_MergingRemovesTheFirstAlert_OcctIssue()
    {
        // OCCT: a simple alert merges with one of the same type, and Message_Report::AddAlert then removes the first
        // alert of the gravity because the default limit is -1 (occt-issues.md, unit 6A)
        global::Kerf.Engine.Foundation.Report aReport = new();
        TestAlertA a1 = new(), a2 = new(), a3 = new();
        aReport.AddAlert(Fail, a1);
        Assert.Equal(1, aReport.GetAlerts(Fail).Extent());
        aReport.AddAlert(Fail, a2); // merged into a1, then a1 removed
        Assert.True(aReport.GetAlerts(Fail).IsEmpty());
        Assert.False(aReport.HasAlert(typeof(TestAlertA)));
        aReport.AddAlert(Fail, a3); // appended again
        Assert.Same(a3, aReport.GetAlerts(Fail).First());

        // with another type first, the merge of a second A removes that other alert, not the A
        global::Kerf.Engine.Foundation.Report aReport2 = new();
        TestAlertB b = new();
        aReport2.AddAlert(Warning, b);
        aReport2.AddAlert(Warning, new TestAlertA());
        aReport2.AddAlert(Warning, new TestAlertA());
        Assert.Equal(1, aReport2.GetAlerts(Warning).Extent());
        Assert.True(aReport2.HasAlert(typeof(TestAlertA)));
        Assert.False(aReport2.HasAlert(typeof(TestAlertB)));
    }

    [Fact]
    public void AddAlert_LimitAppliesOnlyOnMerge_OcctIssue()
    {
        global::Kerf.Engine.Foundation.Report aReport = new();
        aReport.SetLimit(1);
        for (int i = 0; i < 5; ++i)
        {
            aReport.AddAlert(Info, new TestAlertNoMerge());
        }
        // appended alerts are never limited
        Assert.Equal(5, aReport.GetAlerts(Info).Extent());
    }

    [Fact]
    public void CompositeAlerts_MergeAndRemove()
    {
        global::Kerf.Engine.Foundation.CompositeAlerts aComposite = new();
        TestAlertA a1 = new();
        Assert.True(aComposite.AddAlert(Info, a1));
        Assert.False(aComposite.AddAlert(Info, new TestAlertA())); // merged
        Assert.True(aComposite.AddAlert(Info, new TestAlertB()));
        Assert.Equal(2, aComposite.Alerts(Info).Extent());
        Assert.True(aComposite.HasAlert(a1));
        Assert.True(aComposite.RemoveAlert(Info, a1));
        Assert.False(aComposite.RemoveAlert(Info, a1));
        Assert.False(aComposite.HasAlert(a1));
        Assert.False(aComposite.AddAlert(Info, null));
        Assert.True(aComposite.Alerts((global::Kerf.Engine.Foundation.Gravity)42).IsEmpty());
    }

    [Fact]
    public void Clear_AllGravityAndType()
    {
        global::Kerf.Engine.Foundation.Report aReport = new();
        aReport.Clear(); // nothing yet
        aReport.AddAlert(Fail, new TestAlertA());
        aReport.AddAlert(Fail, new TestAlertNoMerge());
        aReport.AddAlert(Fail, new TestAlertB());
        aReport.AddAlert(Warning, new TestAlertA());

        // OCCT's loop would never end on the alerts of other types (occt-issues.md, unit 6A); the port advances
        aReport.Clear(typeof(TestAlertA));
        Assert.Equal(2, aReport.GetAlerts(Fail).Extent());
        Assert.True(aReport.GetAlerts(Warning).IsEmpty());
        Assert.True(aReport.HasAlert(typeof(TestAlertB), Fail));

        aReport.AddAlert(Warning, new TestAlertA());
        aReport.Clear(Fail);
        Assert.True(aReport.GetAlerts(Fail).IsEmpty());
        Assert.False(aReport.GetAlerts(Warning).IsEmpty());

        aReport.Clear();
        Assert.True(aReport.GetAlerts(Warning).IsEmpty());
    }

    [Fact]
    public void Merge_AddsTheOtherReportsAlerts()
    {
        global::Kerf.Engine.Foundation.Report aReport = new();
        global::Kerf.Engine.Foundation.Report anOther = new();
        anOther.AddAlert(Fail, new TestAlertNoMerge());
        anOther.AddAlert(Warning, new TestAlertB());
        aReport.AddAlert(Warning, new TestAlertA());
        aReport.Merge(anOther);
        Assert.Equal(1, aReport.GetAlerts(Fail).Extent());
        Assert.Equal(2, aReport.GetAlerts(Warning).Extent());

        global::Kerf.Engine.Foundation.Report aFails = new();
        aFails.Merge(anOther, Fail);
        Assert.True(aFails.GetAlerts(Warning).IsEmpty());
        Assert.Equal(1, aFails.GetAlerts(Fail).Extent());
    }

    [Fact]
    public void Dump_WritesMessageKeysByGravity()
    {
        global::Kerf.Engine.Foundation.Report aReport = new();
        aReport.AddAlert(Fail, new TestAlertA());
        aReport.AddAlert(Warning, new TestAlertB());
        global::Kerf.Engine.Foundation.AlertExtended.AddAlert(aReport, new global::Kerf.Engine.Foundation.MessageAttribute("named"), Fail);
        global::Kerf.Engine.Foundation.AlertExtended.AddAlert(aReport, null, Fail);

        StringWriter aFails = new();
        aReport.Dump(aFails, Fail);
        Assert.Equal("TestAlertA\nnamed\nMessage_AlertExtended\n", aFails.ToString());

        StringWriter anAll = new();
        aReport.Dump(anAll);
        Assert.Equal("TestAlertB\nTestAlertA\nnamed\nMessage_AlertExtended\n", anAll.ToString());
    }

    [Fact]
    public void SendMessages_SendsKeysWithGravity()
    {
        global::Kerf.Engine.Foundation.Report aReport = new();
        aReport.AddAlert(Fail, new TestAlertA());
        aReport.AddAlert(Trace, new TestAlertB());
        ReportTestPrinter aPrinter = new();
        global::Kerf.Engine.Foundation.Messenger aMessenger = new(aPrinter);
        aReport.SendMessages(aMessenger);
        Assert.Equal([("TestAlertB", Trace), ("TestAlertA", Fail)], aPrinter.Messages);
    }

    [Fact]
    public void AlertExtended_ChildrenAndMerge()
    {
        global::Kerf.Engine.Foundation.AlertExtended anAlert = new();
        Assert.Null(anAlert.CompositeAlerts());
        Assert.True(anAlert.SupportsMerge());
        Assert.False(anAlert.Merge(new global::Kerf.Engine.Foundation.AlertExtended()));
        anAlert.CompositeAlerts(true)!.AddAlert(Info, new TestAlertA());
        Assert.False(anAlert.SupportsMerge()); // hierarchical alerts can't merge

        // children are dumped below their parent
        global::Kerf.Engine.Foundation.Report aReport = new();
        aReport.AddAlert(Info, anAlert);
        StringWriter aText = new();
        aReport.Dump(aText, Info);
        Assert.Equal("Message_AlertExtended\nTestAlertA\n", aText.ToString());
    }

    [Fact]
    public void Attributes()
    {
        Assert.Equal("", new global::Kerf.Engine.Foundation.MessageAttribute().GetMessageKey());
        global::Kerf.Engine.Foundation.AttributeObject anObject = new(new global::Kerf.Engine.Foundation.Report(), "obj");
        Assert.IsType<global::Kerf.Engine.Foundation.Report>(anObject.Object());
        Assert.Equal("obj", anObject.GetName());
        global::Kerf.Engine.Foundation.AttributeStream aStream = new("text", "s");
        Assert.Equal("text", aStream.Stream());
        aStream.SetStream("other");
        Assert.Equal("other", aStream.Stream());

        global::Kerf.Engine.Foundation.AttributeMeter aMeter = new("m");
        Assert.False(aMeter.HasMetric(MetricType_WallClock));
        Assert.Equal(global::Kerf.Engine.Foundation.AttributeMeter.UndefinedMetricValue(), aMeter.StartValue(MetricType_WallClock));
        aMeter.SetStopValue(MetricType_WallClock, 2.0); // no metric yet: ignored
        Assert.False(aMeter.HasMetric(MetricType_WallClock));
        aMeter.SetStartValue(MetricType_WallClock, 1.0);
        Assert.False(aMeter.IsMetricValid(MetricType_WallClock));
        aMeter.SetStopValue(MetricType_WallClock, 3.0);
        Assert.True(aMeter.IsMetricValid(MetricType_WallClock));
        Assert.Equal(1.0, aMeter.StartValue(MetricType_WallClock));
        Assert.Equal(3.0, aMeter.StopValue(MetricType_WallClock));
    }

    [Fact]
    public void MetricNames()
    {
        Assert.Equal("WallClock", global::Kerf.Engine.Foundation.Message.MetricToString(MetricType_WallClock));
        Assert.Equal(MetricType_MemHeapUsage, global::Kerf.Engine.Foundation.Message.MetricFromString("MemHeapUsage"));
        Assert.Equal(MetricType_None, global::Kerf.Engine.Foundation.Message.MetricFromString("nonsense"));
        Assert.False(global::Kerf.Engine.Foundation.Message.MetricFromString("nonsense", out _));
    }

    [Fact]
    public void ActivateInMessenger_PrinterToReport()
    {
        global::Kerf.Engine.Foundation.Report aReport = new();
        global::Kerf.Engine.Foundation.Messenger aMessenger = new(new ReportTestPrinter());
        Assert.False(aReport.IsActiveInMessenger());
        aReport.ActivateInMessenger(true, aMessenger);
        Assert.True(aReport.IsActiveInMessenger());
        Assert.Equal(2, aMessenger.Printers().Length());

        // a string sent to the messenger becomes an alert named by it
        aMessenger.Send("hello", Warning);
        Assert.Equal("hello", aReport.GetAlerts(Warning).First().GetMessageKey());

        // a stream is held back as the name of the next alert
        aMessenger.Send(new StringBuilder("first"), Info);
        Assert.True(aReport.GetAlerts(Info).IsEmpty());
        aMessenger.Send(new StringBuilder("second"), Info);
        Assert.Equal("first", aReport.GetAlerts(Info).First().GetMessageKey());

        // an object becomes an alert with an object attribute named by the held-back stream
        global::Kerf.Engine.Foundation.Report aPayload = new();
        aMessenger.Send(aPayload, Alarm);
        var anObjectAlert = Assert.IsType<global::Kerf.Engine.Foundation.AlertExtended>(aReport.GetAlerts(Alarm).First());
        var anAttribute = Assert.IsType<global::Kerf.Engine.Foundation.AttributeObject>(anObjectAlert.Attribute());
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
        global::Kerf.Engine.Foundation.Report aReport = new();
        Parallel.For(0, 1000, _ => aReport.AddAlert(Warning, new TestAlertNoMerge()));
        Assert.Equal(1000, aReport.GetAlerts(Warning).Extent());
    }
}

/// <summary>The tests that use the process-wide default report, one at a time.</summary>
[Collection(nameof(DefaultReportCollection))]
public class Message_LevelTests
{
    [Fact]
    public void Levels_NestAlertsUnderTheirRoots()
    {
        global::Kerf.Engine.Foundation.Report aReport = new();
        global::Kerf.Engine.Foundation.Messenger aMessenger = new(new ReportTestPrinter());
        global::Kerf.Engine.Foundation.Message.SetDefaultReport(aReport);
        try
        {
            // the report must be active for levels to open
            using (new global::Kerf.Engine.Foundation.Level("inactive"))
            {
            }
            Assert.True(aReport.GetAlerts(Info).IsEmpty());

            aReport.ActivateInMessenger(true, aMessenger);
            using (global::Kerf.Engine.Foundation.Level aLevel1 = new("level1"))
            {
                aReport.AddAlert(Info, global::Kerf.Engine.Foundation.AlertExtended.AddAlert(new global::Kerf.Engine.Foundation.Report(), new global::Kerf.Engine.Foundation.MessageAttribute("a"), Info));
                using (new global::Kerf.Engine.Foundation.Level("level2"))
                {
                    global::Kerf.Engine.Foundation.AlertExtended.AddAlert(aReport, new global::Kerf.Engine.Foundation.MessageAttribute("b"), Warning);
                    // a simple alert can't go below a level: Message_Level::AddAlert drops it
                    aReport.AddAlert(Fail, new TestAlertA());
                }
                Assert.Equal("level1", aLevel1.RootAlert()!.GetMessageKey());
            }
            global::Kerf.Engine.Foundation.AlertExtended.AddAlert(aReport, new global::Kerf.Engine.Foundation.MessageAttribute("top"), Info);

            StringWriter anInfo = new();
            aReport.Dump(anInfo, Info);
            Assert.Equal("level1\na\nlevel2\ntop\n", anInfo.ToString());
            StringWriter aWarning = new();
            aReport.Dump(aWarning, Warning);
            Assert.Equal("", aWarning.ToString()); // the warning is a child of level2, in its Warning list
            global::Kerf.Engine.Foundation.AlertExtended aRoot1 = (global::Kerf.Engine.Foundation.AlertExtended)aReport.GetAlerts(Info).First();
            global::Kerf.Engine.Foundation.AlertExtended aRoot2 = (global::Kerf.Engine.Foundation.AlertExtended)aRoot1.CompositeAlerts()!.Alerts(Info).Last();
            Assert.Equal("b", aRoot2.CompositeAlerts()!.Alerts(Warning).First().GetMessageKey());
            Assert.True(aReport.GetAlerts(Fail).IsEmpty());
        }
        finally
        {
            global::Kerf.Engine.Foundation.Message.SetDefaultReport(null);
        }
    }

    [Fact]
    public void Levels_RecordMetrics()
    {
        global::Kerf.Engine.Foundation.Report aReport = new();
        aReport.SetActiveMetric(MetricType_WallClock, true);
        aReport.SetActiveMetric(MetricType_MemHeapUsage, true);
        aReport.SetActiveMetric(MetricType_WallClock, true); // already active
        Assert.Equal(2, aReport.ActiveMetrics().Count);
        global::Kerf.Engine.Foundation.Messenger aMessenger = new(new ReportTestPrinter());
        global::Kerf.Engine.Foundation.Message.SetDefaultReport(aReport);
        try
        {
            aReport.ActivateInMessenger(true, aMessenger);
            using (new global::Kerf.Engine.Foundation.Level("timed"))
            {
                global::Kerf.Engine.Foundation.AlertExtended.AddAlert(aReport, new global::Kerf.Engine.Foundation.AttributeMeter("step"), Info);
            }
            global::Kerf.Engine.Foundation.AlertExtended aRoot = (global::Kerf.Engine.Foundation.AlertExtended)aReport.GetAlerts(Info).First();
            global::Kerf.Engine.Foundation.AttributeMeter aMeter = Assert.IsType<global::Kerf.Engine.Foundation.AttributeMeter>(aRoot.Attribute());
            Assert.True(aMeter.IsMetricValid(MetricType_WallClock));
            Assert.True(aMeter.StopValue(MetricType_WallClock) >= aMeter.StartValue(MetricType_WallClock));
            Assert.True(aMeter.HasMetric(MetricType_MemHeapUsage));
            Assert.False(aMeter.HasMetric(MetricType_ThreadCPUUserTime));
        }
        finally
        {
            global::Kerf.Engine.Foundation.Message.SetDefaultReport(null);
        }
        aReport.ClearMetrics();
        Assert.True(aReport.ActiveMetrics().Count == 0);
    }
}

[CollectionDefinition(nameof(DefaultReportCollection), DisableParallelization = true)]
public sealed class DefaultReportCollection;
