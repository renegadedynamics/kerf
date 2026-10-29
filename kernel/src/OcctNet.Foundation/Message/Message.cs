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
/// Decision 4008: OCCT's default messenger has one printer to <c>std::cout</c>; the port's has no printer, so a
/// library call (in a Bento cell, for example) never writes to the console. The messenger is replaced only through the
/// internal <see cref="SetDefaultMessenger"/> (process-wide state, decision 415's rule); Draw-lite installs one that
/// prints as DRAWEXE does.
/// </para>
/// <para>
/// Left out: <c>DefaultReport</c> (Message_Report and the alerts aren't ported), the metric helpers
/// (<c>MetricToString</c>, <c>MetricFromString</c>, <c>ToOSDMetric</c>, <c>ToMessageMetric</c>: OSD_MemInfo isn't
/// ported).
/// </para>
/// </remarks>
public static class Message
{
    private static volatile Message_Messenger theDefaultMessenger = Message_Messenger.Empty();

    /// <summary>The process-wide messenger. By default it has no printer: messages sent to it go nowhere.</summary>
    public static Message_Messenger DefaultMessenger() => theDefaultMessenger;

    /// <summary>Replaces the process-wide messenger (Draw-lite); null restores an empty one.</summary>
    internal static void SetDefaultMessenger(Message_Messenger? theMessenger) =>
        theDefaultMessenger = theMessenger ?? Message_Messenger.Empty();

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
