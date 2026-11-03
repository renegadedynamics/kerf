// Port of: src/FoundationClasses/TKernel/Message/Message.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using static Kerf.Engine.Foundation.Gravity;

namespace Kerf.Engine.Foundation;

/// <summary>
/// The process-wide messenger and short-cuts to it: <c>Message.SendWarning("text")</c>,
/// <c>Message.SendFail().Append("text ").Append(n).Flush()</c>.
/// </summary>
/// <remarks>
/// <para>
/// Decision 493: OCCT's default messenger has one printer to <c>std::cout</c>; the port's has no printer, so a
/// library call (in a Bento cell, for example) never writes to the console. The messenger is replaced only through the
/// internal <see cref="SetDefaultMessenger"/> (process-wide state, decision 415's rule); Draw-lite installs one that
/// prints as DRAWEXE does.
/// </para>
/// <para>
/// <see cref="DefaultReport"/> (unit 6A) is internal process-wide state too (decisions 415 and 584): only
/// <see cref="Level"/> and <see cref="PrinterToReport"/> reach it, and an algorithm's alerts go to its
/// own report. Left out: <c>ToOSDMetric</c> and <c>ToMessageMetric</c> (OSD_MemInfo isn't ported;
/// <see cref="AttributeMeter"/> reads .NET's counters instead).
/// </para>
/// </remarks>
public static class Message
{
    private static volatile Messenger s_defaultMessenger = Messenger.Empty();

    // OCCT.NET (decision 1275): a messenger standing for the default one in the current flow of execution, as
    // DRAWEXE's data exchange commands (XSTEPDRAWRUN) give the default messenger a printer to the Tcl result while
    // they run; Draw-lite installs one per command, and parallel interpreters don't see each other's
    private static readonly AsyncLocal<Messenger?> FlowMessenger = new();

    /// <summary>
    /// The process-wide messenger. By default it has no printer: messages sent to it go nowhere. In a flow of execution
    /// that installed a messenger of its own (<see cref="SetFlowMessenger"/>), that one.
    /// </summary>
    public static Messenger DefaultMessenger() => FlowMessenger.Value ?? s_defaultMessenger;

    /// <summary>
    /// Installs a messenger standing for the default one in the current flow of execution (the thread and the tasks it
    /// starts), until replaced; null removes it. Returns the one it replaces (OCCT.NET, for Draw-lite's commands).
    /// </summary>
    internal static Messenger? SetFlowMessenger(Messenger? messenger)
    {
        Messenger? old = FlowMessenger.Value;
        FlowMessenger.Value = messenger;
        return old;
    }

    /// <summary>Replaces the process-wide messenger (Draw-lite); null restores an empty one.</summary>
    internal static void SetDefaultMessenger(Messenger? messenger) =>
        s_defaultMessenger = messenger ?? Messenger.Empty();

    private static volatile Report? s_defaultReport;
    private static readonly Lock DefaultReportLock = new();

    /// <summary>
    /// The process-wide report, made first if <paramref name="toCreate"/> (null otherwise until made). Internal:
    /// only the report levels and the printer to a report use it (decision 584).
    /// </summary>
    internal static Report? DefaultReport(bool toCreate = false)
    {
        if (s_defaultReport is null && toCreate)
        {
            lock (DefaultReportLock)
            {
                s_defaultReport ??= new Report();
            }
        }
        return s_defaultReport;
    }

    /// <summary>For tests and Draw-lite: replaces (or, with null, removes) the process-wide report.</summary>
    internal static void SetDefaultReport(Report? report) => s_defaultReport = report;

    private static readonly string[] Message_Table_PrintMetricTypeEnum =
    [
        "NONE", "ThreadCPUUserTime", "ThreadCPUSystemTime", "ProcessCPUUserTime", "ProcessCPUSystemTime", "WallClock",
        "MemPrivate", "MemVirtual", "MemWorkingSet", "MemWorkingSetPeak", "MemSwapUsage", "MemSwapUsagePeak",
        "MemHeapUsage",
    ];

    /// <summary>The name of the metric ("WallClock", ...).</summary>
    public static string MetricToString(MetricType type) => Message_Table_PrintMetricTypeEnum[(int)type];

    /// <summary>The metric named by the string; false if there is none.</summary>
    public static bool MetricFromString(string theString, out MetricType type)
    {
        for (int metricIter = 0; metricIter <= (int)MetricType.MetricType_MemHeapUsage; ++metricIter)
        {
            if (theString == Message_Table_PrintMetricTypeEnum[metricIter])
            {
                type = (MetricType)metricIter;
                return true;
            }
        }
        type = MetricType.MetricType_None;
        return false;
    }

    /// <summary>The metric named by the string, or <see cref="MetricType.MetricType_None"/>.</summary>
    public static MetricType MetricFromString(string theString)
    {
        MetricFromString(theString, out MetricType metric);
        return metric;
    }

    // ---- Short-cuts to DefaultMessenger

    public static Messenger.StreamBuffer Send(Gravity gravity) => DefaultMessenger().Send(gravity);

    public static void Send(string message, Gravity gravity) => DefaultMessenger().Send(message, gravity);

    public static Messenger.StreamBuffer SendFail() => DefaultMessenger().SendFail();

    public static Messenger.StreamBuffer SendAlarm() => DefaultMessenger().SendAlarm();

    public static Messenger.StreamBuffer SendWarning() => DefaultMessenger().SendWarning();

    public static Messenger.StreamBuffer SendInfo() => DefaultMessenger().SendInfo();

    public static Messenger.StreamBuffer SendTrace() => DefaultMessenger().SendTrace();

    public static void SendFail(string message) => DefaultMessenger().SendFail(message);

    public static void SendAlarm(string message) => DefaultMessenger().SendAlarm(message);

    public static void SendWarning(string message) => DefaultMessenger().SendWarning(message);

    public static void SendInfo(string message) => DefaultMessenger().SendInfo(message);

    public static void SendTrace(string message) => DefaultMessenger().SendTrace(message);

    /// <summary>
    /// Hours, minutes and seconds as text: (5, 12, 26.3345) gives "05h:12m:26.33s", (0, 6, 34.496) "06m:34.50s",
    /// (0, 0, 4.5) "4.50s".
    /// </summary>
    public static string FillTime(int hour, int minute, double second)
    {
        if (hour > 0)
        {
            return Msg.Sprintf("%02d", hour) + "h:" + Msg.Sprintf("%02d", minute) + "m:"
                   + Msg.Sprintf("%.2f", second) + "s";
        }
        else if (minute > 0)
        {
            return Msg.Sprintf("%02d", minute) + "m:" + Msg.Sprintf("%.2f", second) + "s";
        }
        else
        {
            return Msg.Sprintf("%.2f", second) + "s";
        }
    }
}
