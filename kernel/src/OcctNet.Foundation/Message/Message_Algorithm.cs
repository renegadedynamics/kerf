// Port of: src/FoundationClasses/TKernel/Message/Message_Algorithm.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_Algorithm.lxx
// Port of: src/FoundationClasses/TKernel/Message/Message_Algorithm.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2003-2026 OPEN CASCADE SAS. Original implementation copyright (c) RINA S.p.A.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text;
using static Occt.Message_Gravity;
using static Occt.Message_StatusType;

namespace Occt;

/// <summary>
/// The base of algorithms that report how their execution went: an execution status (<see cref="Message_ExecStatus"/>),
/// set with <c>SetStatus</c>, where each status flag can carry numbers or strings naming the items that caused it, and
/// messages for the statuses sent to a messenger by <see cref="SendMessages"/>.
/// </summary>
/// <remarks>
/// <para>
/// A working object, as OCCT's: its status is set in place by the algorithm. The message for a status is taken from
/// <see cref="Message_MsgFile"/> under "{class name}.{status name}" ("Message_Algorithm.Fail5"), trying the base
/// classes up to <see cref="Standard_Transient"/> in turn. OCCT reads those texts from resource files, which the port
/// doesn't load (decision 4008), so without an <see cref="Message_MsgFile.AddMsg"/> the text is the "Unknown message
/// invoked with the keyword ..." surrogate, followed by nothing: the numbers and strings fill the message's
/// placeholders, of which the surrogate has none. The class names are .NET's, which are OCCT's.
/// </para>
/// <para>
/// TCollection's strings are <see cref="string"/>, so the five string overloads of <c>SetStatus</c> are one. The
/// numbers of a status are an <see cref="NCollection_PackedMapOfInteger"/> (OCCT's
/// <c>TColStd_HPackedMapOfInteger</c>), its strings an <see cref="NCollection_HSequence{T}"/> of strings; both are
/// the algorithm's own, as OCCT returns its handles.
/// </para>
/// </remarks>
public class Message_Algorithm : Standard_Transient
{
    protected Message_ExecStatus myStatus;
    protected Message_Messenger myMessenger;

    private NCollection_Array1<NCollection_PackedMapOfInteger?>? myReportIntegers;
    private NCollection_Array1<NCollection_HSequence<string>?>? myReportStrings;
    private NCollection_Array1<Message_Msg?>? myReportMessages;

    /// <summary>An algorithm with no status, sending to <see cref="Message.DefaultMessenger"/>.</summary>
    public Message_Algorithm()
    {
        myMessenger = Message.DefaultMessenger();
    }

    /// <summary>Sets the messenger; null sets <see cref="Message.DefaultMessenger"/>.</summary>
    public void SetMessenger(Message_Messenger? theMsgr)
    {
        if (theMsgr is null)
        {
            myMessenger = Message.DefaultMessenger();
        }
        else
        {
            myMessenger = theMsgr;
        }
    }

    /// <summary>Sets a status flag.</summary>
    public void SetStatus(Message_Status theStat)
    {
        myStatus.Set(theStat);
    }

    /// <summary>Sets a status flag with an integer parameter.</summary>
    public void SetStatus(Message_Status theStat, int theInt)
    {
        // Set status flag
        SetStatus(theStat);

        // Find index of bit corresponding to that flag
        int aFlagIndex = Message_ExecStatus.StatusIndex(theStat);
        if (aFlagIndex == 0)
        {
            return;
        }

        // Create map of integer parameters for a given flag, if not yet done
        myReportIntegers ??= new NCollection_Array1<NCollection_PackedMapOfInteger?>(Message_ExecStatus.FirstStatus,
                                                                                    Message_ExecStatus.LastStatus);
        ref NCollection_PackedMapOfInteger? aData = ref myReportIntegers[aFlagIndex];
        aData ??= new NCollection_PackedMapOfInteger();

        // add integer parameter for the status
        aData.Add(theInt);
    }

    /// <summary>
    /// Sets a status flag with a string parameter. With <paramref name="noRepetitions"/>, the string is added only if
    /// the status doesn't already have it. A null string sets the flag only.
    /// </summary>
    public void SetStatus(Message_Status theStat, string? theStr, bool noRepetitions = true)
    {
        // Set status flag
        SetStatus(theStat);
        if (theStr is null)
        {
            return;
        }

        // Find index of bit corresponding to that flag
        int aFlagIndex = Message_ExecStatus.StatusIndex(theStat);
        if (aFlagIndex == 0)
        {
            return;
        }

        // Create sequence of string parameters for a given flag, if not yet done
        myReportStrings ??= new NCollection_Array1<NCollection_HSequence<string>?>(Message_ExecStatus.FirstStatus,
                                                                                  Message_ExecStatus.LastStatus);
        ref NCollection_HSequence<string>? aData = ref myReportStrings[aFlagIndex];
        aData ??= new NCollection_HSequence<string>();

        // Add string parameter
        NCollection_HSequence<string> aReportSeq = aData;
        if (noRepetitions)
        {
            // if the provided string has been already registered, just do nothing
            for (int i = 1; i <= aReportSeq.Length(); i++)
            {
                if (string.Equals(aReportSeq.Value(i), theStr, StringComparison.Ordinal))
                {
                    return;
                }
            }
        }

        aReportSeq.Append(theStr);
    }

    /// <summary>
    /// Sets a status flag with a prepared message, which is then sent for the status in place of the message file's.
    /// </summary>
    public void SetStatus(Message_Status theStat, Message_Msg theMsg)
    {
        // Set status flag
        SetStatus(theStat);

        // Find index of bit corresponding to that flag
        int aFlagIndex = Message_ExecStatus.StatusIndex(theStat);
        if (aFlagIndex == 0)
        {
            return;
        }

        // Create sequence of messages for a given flag, if not yet done
        myReportMessages ??= new NCollection_Array1<Message_Msg?>(Message_ExecStatus.FirstStatus, Message_ExecStatus.LastStatus);

        myReportMessages[aFlagIndex] = new Message_Msg(theMsg);
    }

    /// <summary>The execution status.</summary>
    public Message_ExecStatus GetStatus() => myStatus;

    /// <summary>The execution status itself, to change it.</summary>
    public ref Message_ExecStatus ChangeStatus() => ref myStatus;

    /// <summary>Clears the status and the parameters and messages of every status.</summary>
    public void ClearStatus()
    {
        myStatus.Clear();
        myReportIntegers = null;
        myReportStrings = null;
        myReportMessages = null;
    }

    /// <summary>The messenger (never null).</summary>
    public Message_Messenger GetMessenger() => myMessenger;

    /// <summary>
    /// Sends the messages of the statuses that are set both here and in <paramref name="theFilter"/>, with the gravity
    /// <paramref name="theTraceLevel"/>, each with at most <paramref name="theMaxCount"/> of its numbers or strings.
    /// </summary>
    public virtual void SendStatusMessages(in Message_ExecStatus theFilter, Message_Gravity theTraceLevel = Message_Warning,
                                           int theMaxCount = 20)
    {
        Message_Messenger? aMsgr = GetMessenger();
        if (aMsgr is null)
        {
            return;
        }

        // Iterate on all set flags in the specified range
        for (int i = Message_ExecStatus.FirstStatus; i <= Message_ExecStatus.LastStatus; i++)
        {
            Message_Status stat = Message_ExecStatus.StatusByIndex(i);
            if (!theFilter.IsSet(stat) || !myStatus.IsSet(stat))
            {
                continue;
            }

            Message_Msg? aMsgCustom = null;
            if (myReportMessages is not null)
            {
                aMsgCustom = myReportMessages.Value(i);
            }
            if (aMsgCustom is not null)
            {
                // print custom message (OCCT converts the stored message through Get(), which fills it in place)
                aMsgr.Send(aMsgCustom.Get(), theTraceLevel);
                continue;
            }

            // construct message suffix
            string aSuffix;
            switch (Message_ExecStatus.TypeOfStatus(stat))
            {
                case Message_DONE:
                    aSuffix = ".Done";
                    break;
                case Message_WARN:
                    aSuffix = ".Warn";
                    break;
                case Message_ALARM:
                    aSuffix = ".Alarm";
                    break;
                case Message_FAIL:
                    aSuffix = ".Fail";
                    break;
                default:
                    continue;
            }
            aSuffix += Message_ExecStatus.LocalStatusIndex(stat).ToString(CultureInfo.InvariantCulture);

            // find message, prefixed by class type name, iterating by base classes if necessary
            string aMsgName = "";
            for (Type? aType = DynamicType(); aType is not null && aType != typeof(object); aType = aType.BaseType)
            {
                aMsgName = typeName(aType);
                aMsgName += aSuffix;
                if (Message_MsgFile.HasMsg(aMsgName))
                {
                    break;
                }
            }

            // create a message
            Message_Msg aMsg = new(aMsgName);

            // if additional parameters are defined for a given status flag,
            // try to feed them into the message
            if (myReportIntegers is not null)
            {
                NCollection_PackedMapOfInteger? aMapErrors = myReportIntegers.Value(i);
                if (aMapErrors is not null)
                {
                    aMsg.ArgExtended(PrepareReport(aMapErrors, theMaxCount));
                }
            }
            if (myReportStrings is not null && myReportStrings.Value(i) is not null)
            {
                NCollection_HSequence<string>? aReportSeq = myReportStrings.Value(i);
                if (aReportSeq is not null)
                {
                    aMsg.ArgExtended(PrepareReport(aReportSeq.Sequence(), theMaxCount));
                }
            }

            // output the message
            aMsgr.Send(aMsg.Get(), theTraceLevel);
        }
    }

    /// <summary>
    /// <see cref="SendStatusMessages"/> for every Warn, Alarm and Fail status (not the Done ones).
    /// </summary>
    public void SendMessages(Message_Gravity theTraceLevel = Message_Warning, int theMaxCount = 20)
    {
        Message_ExecStatus aStat = new();
        aStat.SetAllWarn();
        aStat.SetAllAlarm();
        aStat.SetAllFail();
        SendStatusMessages(aStat, theTraceLevel, theMaxCount);
    }

    /// <summary>Adds the statuses of the other algorithm, with their numbers and strings.</summary>
    public void AddStatus(Message_Algorithm theOtherAlgo)
    {
        AddStatus(theOtherAlgo.GetStatus(), theOtherAlgo);
    }

    /// <summary>
    /// Adds the statuses of the other algorithm that are set in <paramref name="theAllowedStatus"/>, with their numbers
    /// and strings.
    /// </summary>
    /// <remarks>OCCT: the last string of each status is not copied (the loop stops one short); reproduced.</remarks>
    public void AddStatus(in Message_ExecStatus theAllowedStatus, Message_Algorithm theOtherAlgo)
    {
        // Iterate on all set flags in the specified range
        Message_ExecStatus aStatusOfAlgo = theOtherAlgo.GetStatus();
        for (int i = Message_ExecStatus.FirstStatus; i <= Message_ExecStatus.LastStatus; i++)
        {
            Message_Status stat = Message_ExecStatus.StatusByIndex(i);
            if (!theAllowedStatus.IsSet(stat) || !aStatusOfAlgo.IsSet(stat))
            {
                continue;
            }

            SetStatus(stat);

            // if additional parameters are defined for a given status flag,
            // move them to <this> algorithm
            // a) numbers
            NCollection_PackedMapOfInteger? aNumsOther = theOtherAlgo.GetMessageNumbers(stat);
            if (aNumsOther is not null)
            {
                // Create sequence of integer parameters for a given flag, if not yet done
                myReportIntegers ??= new NCollection_Array1<NCollection_PackedMapOfInteger?>(Message_ExecStatus.FirstStatus,
                                                                                            Message_ExecStatus.LastStatus);
                ref NCollection_PackedMapOfInteger? aData = ref myReportIntegers[i];
                aData ??= new NCollection_PackedMapOfInteger();

                // add integer parameter for the status
                NCollection_PackedMapAlgo.Unite(aData, aNumsOther);
            }
            // b) strings
            NCollection_HSequence<string>? aStrsOther = theOtherAlgo.GetMessageStrings(stat);
            if (aStrsOther is not null)
            {
                for (int n = 1; n < aStrsOther.Length(); n++) // OCCT: n < Length skips the last string
                {
                    SetStatus(stat, aStrsOther.Value(n));
                }
            }
        }
    }

    /// <summary>The numbers of the status; null if the status has none.</summary>
    public NCollection_PackedMapOfInteger? GetMessageNumbers(Message_Status theStatus)
    {
        if (myReportIntegers is null)
        {
            return null;
        }

        // Find index of bit corresponding to that flag
        int aFlagIndex = Message_ExecStatus.StatusIndex(theStatus);
        if (aFlagIndex == 0)
        {
            return null;
        }

        return myReportIntegers.Value(aFlagIndex);
    }

    /// <summary>The strings of the status; null if the status has none.</summary>
    public NCollection_HSequence<string>? GetMessageStrings(Message_Status theStatus)
    {
        if (myReportStrings is null)
        {
            return null;
        }

        // Find index of bit corresponding to that flag
        int aFlagIndex = Message_ExecStatus.StatusIndex(theStatus);
        if (aFlagIndex == 0)
        {
            return null;
        }

        return myReportStrings.Value(aFlagIndex);
    }

    /// <summary>
    /// The numbers of the map separated by spaces, at most <paramref name="theMaxCount"/> of them, then
    /// " ... (total N)" if there are more.
    /// </summary>
    public static string PrepareReport(NCollection_PackedMapOfInteger theMapError, int theMaxCount)
    {
        StringBuilder aNewReport = new();
        NCollection_PackedMapOfInteger.Iterator anIt = new(theMapError);
        int nb = 1;
        for (; anIt.More() && nb <= theMaxCount; anIt.Next(), nb++)
        {
            if (nb > 1)
            {
                aNewReport.Append(' ');
            }
            aNewReport.Append(anIt.Key().ToString(CultureInfo.InvariantCulture));
        }

        if (anIt.More())
        {
            aNewReport.Append(" ... (total ");
            aNewReport.Append(theMapError.Extent().ToString(CultureInfo.InvariantCulture));
            aNewReport.Append(')');
        }
        return aNewReport.ToString();
    }

    /// <summary>
    /// The strings of the sequence, quoted and separated by ", ", at most <paramref name="theMaxCount"/> of them, then
    /// " ... (total N) " if there are more.
    /// </summary>
    public static string PrepareReport(NCollection_Sequence<string> theReportSeq, int theMaxCount)
    {
        StringBuilder aNewReport = new();
        int nb = 1;
        for (; nb <= theReportSeq.Length() && nb <= theMaxCount; nb++)
        {
            aNewReport.Append(nb > 1 ? ", '" : "'");
            aNewReport.Append(theReportSeq.Value(nb));
            aNewReport.Append('\'');
        }

        if (theReportSeq.Length() > theMaxCount)
        {
            aNewReport.Append(" ... (total ");
            aNewReport.Append(theReportSeq.Length().ToString(CultureInfo.InvariantCulture));
            aNewReport.Append(") ");
        }
        return aNewReport.ToString();
    }

    // OCCT's Standard_Type::Name(): the class name, without .NET's generic arity suffix.
    private static string typeName(Type theType)
    {
        string aName = theType.Name;
        int aTick = aName.IndexOf('`');
        return aTick < 0 ? aName : aName.Substring(0, aTick);
    }
}
