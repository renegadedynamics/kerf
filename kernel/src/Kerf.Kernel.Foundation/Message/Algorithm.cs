// Port of: src/FoundationClasses/TKernel/Message/Message_Algorithm.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_Algorithm.lxx
// Port of: src/FoundationClasses/TKernel/Message/Message_Algorithm.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2003-2026 OPEN CASCADE SAS. Original implementation copyright (c) RINA S.p.A.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;
using System.Globalization;
using System.Text;
using static Kerf.Engine.Foundation.Gravity;
using static Kerf.Engine.Foundation.StatusType;

namespace Kerf.Engine.Foundation;

/// <summary>
/// The base of algorithms that report how their execution went: an execution status (<see cref="ExecStatus"/>),
/// set with <c>SetStatus</c>, where each status flag can carry numbers or strings naming the items that caused it, and
/// messages for the statuses sent to a messenger by <see cref="SendMessages"/>.
/// </summary>
/// <remarks>
/// <para>
/// A working object, as OCCT's: its status is set in place by the algorithm. The message for a status is taken from
/// <see cref="MsgFile"/> under "{class name}.{status name}" ("Message_Algorithm.Fail5"), trying the base
/// classes up to <see cref="Transient"/> in turn. OCCT reads those texts from resource files, which the port
/// doesn't load (decision 493), so without an <see cref="MsgFile.AddMsg"/> the text is the "Unknown message
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
internal class Algorithm : Transient
{
    protected ExecStatus _status;
    protected Messenger _messenger;

    private NCollection_Array1<NCollection_PackedMapOfInteger?>? _reportIntegers;
    private NCollection_Array1<NCollection_HSequence<string>?>? _reportStrings;
    private NCollection_Array1<Msg?>? _reportMessages;

    /// <summary>An algorithm with no status, sending to <see cref="Message.DefaultMessenger"/>.</summary>
    public Algorithm()
    {
        _messenger = global::Kerf.Engine.Foundation.Message.DefaultMessenger();
    }

    /// <summary>Sets the messenger; null sets <see cref="Message.DefaultMessenger"/>.</summary>
    public void SetMessenger(Messenger? msgr)
    {
        if (msgr is null)
        {
            _messenger = global::Kerf.Engine.Foundation.Message.DefaultMessenger();
        }
        else
        {
            _messenger = msgr;
        }
    }

    /// <summary>Sets a status flag.</summary>
    public void SetStatus(Status stat)
    {
        _status.Set(stat);
    }

    /// <summary>Sets a status flag with an integer parameter.</summary>
    public void SetStatus(Status stat, int theInt)
    {
        // Set status flag
        SetStatus(stat);

        // Find index of bit corresponding to that flag
        int flagIndex = ExecStatus.StatusIndex(stat);
        if (flagIndex == 0)
        {
            return;
        }

        // Create map of integer parameters for a given flag, if not yet done
        _reportIntegers ??= new NCollection_Array1<NCollection_PackedMapOfInteger?>(ExecStatus.FirstStatus,
                                                                                    ExecStatus.LastStatus);
        ref NCollection_PackedMapOfInteger? data = ref _reportIntegers[flagIndex];
        data ??= new NCollection_PackedMapOfInteger();

        // add integer parameter for the status
        data.Add(theInt);
    }

    /// <summary>
    /// Sets a status flag with a string parameter. With <paramref name="noRepetitions"/>, the string is added only if
    /// the status doesn't already have it. A null string sets the flag only.
    /// </summary>
    public void SetStatus(Status stat, string? str, bool noRepetitions = true)
    {
        // Set status flag
        SetStatus(stat);
        if (str is null)
        {
            return;
        }

        // Find index of bit corresponding to that flag
        int flagIndex = ExecStatus.StatusIndex(stat);
        if (flagIndex == 0)
        {
            return;
        }

        // Create sequence of string parameters for a given flag, if not yet done
        _reportStrings ??= new NCollection_Array1<NCollection_HSequence<string>?>(ExecStatus.FirstStatus,
                                                                                  ExecStatus.LastStatus);
        ref NCollection_HSequence<string>? data = ref _reportStrings[flagIndex];
        data ??= new NCollection_HSequence<string>();

        // Add string parameter
        NCollection_HSequence<string> reportSeq = data;
        if (noRepetitions)
        {
            // if the provided string has been already registered, just do nothing
            for (int i = 1; i <= reportSeq.Length(); i++)
            {
                if (string.Equals(reportSeq.Value(i), str, StringComparison.Ordinal))
                {
                    return;
                }
            }
        }

        reportSeq.Append(str);
    }

    /// <summary>
    /// Sets a status flag with a prepared message, which is then sent for the status in place of the message file's.
    /// </summary>
    public void SetStatus(Status stat, Msg msg)
    {
        // Set status flag
        SetStatus(stat);

        // Find index of bit corresponding to that flag
        int flagIndex = ExecStatus.StatusIndex(stat);
        if (flagIndex == 0)
        {
            return;
        }

        // Create sequence of messages for a given flag, if not yet done
        _reportMessages ??= new NCollection_Array1<Msg?>(ExecStatus.FirstStatus, ExecStatus.LastStatus);

        _reportMessages[flagIndex] = new Msg(msg);
    }

    /// <summary>The execution status.</summary>
    public ExecStatus GetStatus() => _status;

    /// <summary>The execution status itself, to change it.</summary>
    public ref ExecStatus ChangeStatus() => ref _status;

    /// <summary>Clears the status and the parameters and messages of every status.</summary>
    public void ClearStatus()
    {
        _status.Clear();
        _reportIntegers = null;
        _reportStrings = null;
        _reportMessages = null;
    }

    /// <summary>The messenger (never null).</summary>
    public Messenger GetMessenger() => _messenger;

    /// <summary>
    /// Sends the messages of the statuses that are set both here and in <paramref name="filter"/>, with the gravity
    /// <paramref name="traceLevel"/>, each with at most <paramref name="maxCount"/> of its numbers or strings.
    /// </summary>
    public virtual void SendStatusMessages(in ExecStatus filter, Gravity traceLevel = Warning,
                                           int maxCount = 20)
    {
        Messenger? msgr = GetMessenger();
        if (msgr is null)
        {
            return;
        }

        // Iterate on all set flags in the specified range
        for (int i = ExecStatus.FirstStatus; i <= ExecStatus.LastStatus; i++)
        {
            Status stat = ExecStatus.StatusByIndex(i);
            if (!filter.IsSet(stat) || !_status.IsSet(stat))
            {
                continue;
            }

            Msg? msgCustom = null;
            if (_reportMessages is not null)
            {
                msgCustom = _reportMessages.Value(i);
            }
            if (msgCustom is not null)
            {
                // print custom message (OCCT converts the stored message through Get(), which fills it in place)
                msgr.Send(msgCustom.Get(), traceLevel);
                continue;
            }

            // construct message suffix
            string suffix;
            switch (ExecStatus.TypeOfStatus(stat))
            {
                case Done:
                    suffix = ".Done";
                    break;
                case Warn:
                    suffix = ".Warn";
                    break;
                case StatusType.Alarm:
                    suffix = ".Alarm";
                    break;
                case StatusType.Fail:
                    suffix = ".Fail";
                    break;
                default:
                    continue;
            }
            suffix += ExecStatus.LocalStatusIndex(stat).ToString(CultureInfo.InvariantCulture);

            // find message, prefixed by class type name, iterating by base classes if necessary
            string msgName = "";
            for (Type? type = DynamicType(); type is not null && type != typeof(object); type = type.BaseType)
            {
                msgName = TypeName(type);
                msgName += suffix;
                if (MsgFile.HasMsg(msgName))
                {
                    break;
                }
            }

            // create a message
            Msg msg = new(msgName);

            // if additional parameters are defined for a given status flag,
            // try to feed them into the message
            if (_reportIntegers is not null)
            {
                NCollection_PackedMapOfInteger? mapErrors = _reportIntegers.Value(i);
                if (mapErrors is not null)
                {
                    msg.ArgExtended(PrepareReport(mapErrors, maxCount));
                }
            }
            if (_reportStrings is not null && _reportStrings.Value(i) is not null)
            {
                NCollection_HSequence<string>? reportSeq = _reportStrings.Value(i);
                if (reportSeq is not null)
                {
                    msg.ArgExtended(PrepareReport(reportSeq.Sequence(), maxCount));
                }
            }

            // output the message
            msgr.Send(msg.Get(), traceLevel);
        }
    }

    /// <summary>
    /// <see cref="SendStatusMessages"/> for every Warn, Alarm and Fail status (not the Done ones).
    /// </summary>
    public void SendMessages(Gravity traceLevel = Warning, int maxCount = 20)
    {
        ExecStatus stat = new();
        stat.SetAllWarn();
        stat.SetAllAlarm();
        stat.SetAllFail();
        SendStatusMessages(stat, traceLevel, maxCount);
    }

    /// <summary>Adds the statuses of the other algorithm, with their numbers and strings.</summary>
    public void AddStatus(Algorithm otherAlgo)
    {
        AddStatus(otherAlgo.GetStatus(), otherAlgo);
    }

    /// <summary>
    /// Adds the statuses of the other algorithm that are set in <paramref name="allowedStatus"/>, with their numbers
    /// and strings.
    /// </summary>
    /// <remarks>OCCT: the last string of each status is not copied (the loop stops one short); reproduced.</remarks>
    public void AddStatus(in ExecStatus allowedStatus, Algorithm otherAlgo)
    {
        // Iterate on all set flags in the specified range
        ExecStatus statusOfAlgo = otherAlgo.GetStatus();
        for (int i = ExecStatus.FirstStatus; i <= ExecStatus.LastStatus; i++)
        {
            Status stat = ExecStatus.StatusByIndex(i);
            if (!allowedStatus.IsSet(stat) || !statusOfAlgo.IsSet(stat))
            {
                continue;
            }

            SetStatus(stat);

            // if additional parameters are defined for a given status flag,
            // move them to <this> algorithm
            // a) numbers
            NCollection_PackedMapOfInteger? numsOther = otherAlgo.GetMessageNumbers(stat);
            if (numsOther is not null)
            {
                // Create sequence of integer parameters for a given flag, if not yet done
                _reportIntegers ??= new NCollection_Array1<NCollection_PackedMapOfInteger?>(ExecStatus.FirstStatus,
                                                                                            ExecStatus.LastStatus);
                ref NCollection_PackedMapOfInteger? data = ref _reportIntegers[i];
                data ??= new NCollection_PackedMapOfInteger();

                // add integer parameter for the status
                NCollection_PackedMapAlgo.Unite(data, numsOther);
            }
            // b) strings
            NCollection_HSequence<string>? strsOther = otherAlgo.GetMessageStrings(stat);
            if (strsOther is not null)
            {
                for (int n = 1; n < strsOther.Length(); n++) // OCCT: n < Length skips the last string
                {
                    SetStatus(stat, strsOther.Value(n));
                }
            }
        }
    }

    /// <summary>The numbers of the status; null if the status has none.</summary>
    public NCollection_PackedMapOfInteger? GetMessageNumbers(Status status)
    {
        if (_reportIntegers is null)
        {
            return null;
        }

        // Find index of bit corresponding to that flag
        int flagIndex = ExecStatus.StatusIndex(status);
        if (flagIndex == 0)
        {
            return null;
        }

        return _reportIntegers.Value(flagIndex);
    }

    /// <summary>The strings of the status; null if the status has none.</summary>
    public NCollection_HSequence<string>? GetMessageStrings(Status status)
    {
        if (_reportStrings is null)
        {
            return null;
        }

        // Find index of bit corresponding to that flag
        int flagIndex = ExecStatus.StatusIndex(status);
        if (flagIndex == 0)
        {
            return null;
        }

        return _reportStrings.Value(flagIndex);
    }

    /// <summary>
    /// The numbers of the map separated by spaces, at most <paramref name="maxCount"/> of them, then
    /// " ... (total N)" if there are more.
    /// </summary>
    public static string PrepareReport(NCollection_PackedMapOfInteger mapError, int maxCount)
    {
        StringBuilder newReport = new();
        NCollection_PackedMapOfInteger.Iterator it = new(mapError);
        int nb = 1;
        for (; it.More() && nb <= maxCount; it.Next(), nb++)
        {
            if (nb > 1)
            {
                newReport.Append(' ');
            }
            newReport.Append(it.Key().ToString(CultureInfo.InvariantCulture));
        }

        if (it.More())
        {
            newReport.Append(" ... (total ");
            newReport.Append(mapError.Extent().ToString(CultureInfo.InvariantCulture));
            newReport.Append(')');
        }
        return newReport.ToString();
    }

    /// <summary>
    /// The strings of the sequence, quoted and separated by ", ", at most <paramref name="maxCount"/> of them, then
    /// " ... (total N) " if there are more.
    /// </summary>
    public static string PrepareReport(NCollection_Sequence<string> reportSeq, int maxCount)
    {
        StringBuilder newReport = new();
        int nb = 1;
        for (; nb <= reportSeq.Length() && nb <= maxCount; nb++)
        {
            newReport.Append(nb > 1 ? ", '" : "'");
            newReport.Append(reportSeq.Value(nb));
            newReport.Append('\'');
        }

        if (reportSeq.Length() > maxCount)
        {
            newReport.Append(" ... (total ");
            newReport.Append(reportSeq.Length().ToString(CultureInfo.InvariantCulture));
            newReport.Append(") ");
        }
        return newReport.ToString();
    }

    // OCCT's Standard_Type::Name(): the class name, without .NET's generic arity suffix.
    private static string TypeName(Type type)
    {
        string name = OcctTypeNames.Of(type);
        int tick = name.IndexOf('`');
        return tick < 0 ? name : name.Substring(0, tick);
    }
}
