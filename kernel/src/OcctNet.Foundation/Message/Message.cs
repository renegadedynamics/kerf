// Port of: src/FoundationClasses/TKernel/Message/Message.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Message_Gravity;

namespace Occt;

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
/// <see cref="Message_Level"/> and <see cref="Message_PrinterToReport"/> reach it, and an algorithm's alerts go to its
/// own report. Left out: <c>ToOSDMetric</c> and <c>ToMessageMetric</c> (OSD_MemInfo isn't ported;
/// <see cref="Message_AttributeMeter"/> reads .NET's counters instead).
/// </para>
/// </remarks>
public static class Message
{
    private static volatile Message_Messenger theDefaultMessenger = Message_Messenger.Empty();

    // OCCT.NET (decision 13212): a messenger standing for the default one in the current flow of execution, as
    // DRAWEXE's data exchange commands (XSTEPDRAWRUN) give the default messenger a printer to the Tcl result while
    // they run; Draw-lite installs one per command, and parallel interpreters don't see each other's
    private static readonly AsyncLocal<Message_Messenger?> theFlowMessenger = new();

    /// <summary>
    /// The process-wide messenger. By default it has no printer: messages sent to it go nowhere. In a flow of execution
    /// that installed a messenger of its own (<see cref="SetFlowMessenger"/>), that one.
    /// </summary>
    public static Message_Messenger DefaultMessenger() => theFlowMessenger.Value ?? theDefaultMessenger;

    /// <summary>
    /// Installs a messenger standing for the default one in the current flow of execution (the thread and the tasks it
    /// starts), until replaced; null removes it. Returns the one it replaces (OCCT.NET, for Draw-lite's commands).
    /// </summary>
    internal static Message_Messenger? SetFlowMessenger(Message_Messenger? theMessenger)
    {
        Message_Messenger? anOld = theFlowMessenger.Value;
        theFlowMessenger.Value = theMessenger;
        return anOld;
    }

    /// <summary>Replaces the process-wide messenger (Draw-lite); null restores an empty one.</summary>
    internal static void SetDefaultMessenger(Message_Messenger? theMessenger) =>
        theDefaultMessenger = theMessenger ?? Message_Messenger.Empty();

    private static volatile Message_Report? theDefaultReport;
    private static readonly Lock theDefaultReportLock = new();

    /// <summary>
    /// The process-wide report, made first if <paramref name="theToCreate"/> (null otherwise until made). Internal:
    /// only the report levels and the printer to a report use it (decision 584).
    /// </summary>
    internal static Message_Report? DefaultReport(bool theToCreate = false)
    {
        if (theDefaultReport is null && theToCreate)
        {
            lock (theDefaultReportLock)
            {
                theDefaultReport ??= new Message_Report();
            }
        }
        return theDefaultReport;
    }

    /// <summary>For tests and Draw-lite: replaces (or, with null, removes) the process-wide report.</summary>
    internal static void SetDefaultReport(Message_Report? theReport) => theDefaultReport = theReport;

    private static readonly string[] Message_Table_PrintMetricTypeEnum =
    [
        "NONE", "ThreadCPUUserTime", "ThreadCPUSystemTime", "ProcessCPUUserTime", "ProcessCPUSystemTime", "WallClock",
        "MemPrivate", "MemVirtual", "MemWorkingSet", "MemWorkingSetPeak", "MemSwapUsage", "MemSwapUsagePeak",
        "MemHeapUsage",
    ];

    /// <summary>The name of the metric ("WallClock", ...).</summary>
    public static string MetricToString(Message_MetricType theType) => Message_Table_PrintMetricTypeEnum[(int)theType];

    /// <summary>The metric named by the string; false if there is none.</summary>
    public static bool MetricFromString(string theString, out Message_MetricType theType)
    {
        for (int aMetricIter = 0; aMetricIter <= (int)Message_MetricType.Message_MetricType_MemHeapUsage; ++aMetricIter)
        {
            if (theString == Message_Table_PrintMetricTypeEnum[aMetricIter])
            {
                theType = (Message_MetricType)aMetricIter;
                return true;
            }
        }
        theType = Message_MetricType.Message_MetricType_None;
        return false;
    }

    /// <summary>The metric named by the string, or <see cref="Message_MetricType.Message_MetricType_None"/>.</summary>
    public static Message_MetricType MetricFromString(string theString)
    {
        MetricFromString(theString, out Message_MetricType aMetric);
        return aMetric;
    }

    // ---- Short-cuts to DefaultMessenger

    public static Message_Messenger.StreamBuffer Send(Message_Gravity theGravity) => DefaultMessenger().Send(theGravity);

    public static void Send(string theMessage, Message_Gravity theGravity) => DefaultMessenger().Send(theMessage, theGravity);

    public static Message_Messenger.StreamBuffer SendFail() => DefaultMessenger().SendFail();

    public static Message_Messenger.StreamBuffer SendAlarm() => DefaultMessenger().SendAlarm();

    public static Message_Messenger.StreamBuffer SendWarning() => DefaultMessenger().SendWarning();

    public static Message_Messenger.StreamBuffer SendInfo() => DefaultMessenger().SendInfo();

    public static Message_Messenger.StreamBuffer SendTrace() => DefaultMessenger().SendTrace();

    public static void SendFail(string theMessage) => DefaultMessenger().SendFail(theMessage);

    public static void SendAlarm(string theMessage) => DefaultMessenger().SendAlarm(theMessage);

    public static void SendWarning(string theMessage) => DefaultMessenger().SendWarning(theMessage);

    public static void SendInfo(string theMessage) => DefaultMessenger().SendInfo(theMessage);

    public static void SendTrace(string theMessage) => DefaultMessenger().SendTrace(theMessage);

    /// <summary>
    /// Hours, minutes and seconds as text: (5, 12, 26.3345) gives "05h:12m:26.33s", (0, 6, 34.496) "06m:34.50s",
    /// (0, 0, 4.5) "4.50s".
    /// </summary>
    public static string FillTime(int hour, int minute, double second)
    {
        if (hour > 0)
        {
            return Message_Msg.Sprintf("%02d", hour) + "h:" + Message_Msg.Sprintf("%02d", minute) + "m:"
                   + Message_Msg.Sprintf("%.2f", second) + "s";
        }
        else if (minute > 0)
        {
            return Message_Msg.Sprintf("%02d", minute) + "m:" + Message_Msg.Sprintf("%.2f", second) + "s";
        }
        else
        {
            return Message_Msg.Sprintf("%.2f", second) + "s";
        }
    }
}
