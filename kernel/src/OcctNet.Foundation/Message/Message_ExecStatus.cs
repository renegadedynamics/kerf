// Port of: src/FoundationClasses/TKernel/Message/Message_ExecStatus.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2003-2026 OPEN CASCADE SAS. Original implementation copyright (c) RINA S.p.A.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Message_Status;
using static Occt.Message_StatusType;

namespace Occt;

/// <summary>
/// The execution status of an algorithm: a set of bit flags, 32 for each of the four types of status (Done, Warn,
/// Alarm, Fail), each set and tested on its own.
/// </summary>
/// <remarks>
/// A value type, as in OCCT. Its mutators (<c>Set</c>, <c>Clear</c>, <c>Add</c>, <c>And</c>, <c>SetAll*</c>,
/// <c>ClearAll*</c>) are internal; their public twins return a changed copy (<see cref="With"/>,
/// <see cref="Without"/>, <see cref="Cleared"/>, <see cref="Added"/>, <see cref="Anded"/>, <c>WithAll*</c>,
/// <c>WithoutAll*</c>) and <c>|</c> and <c>&amp;</c> stand for OCCT's <c>|=</c> and <c>&amp;=</c>. The default value is
/// the empty status, as OCCT's default constructor.
/// </remarks>
public partial struct Message_ExecStatus : IEquatable<Message_ExecStatus>
{
    // Mask to separate bits indicating status type and index within the type (OCCT's enum StatusMask)
    private const int MType = 0x0000ff00;
    private const int MIndex = 0x000000ff;

    private static int getBitFlag(int theStatus) => 0x1 << (theStatus & MIndex);

    private int myDone;
    private int myWarn;
    private int myAlarm;
    private int myFail;

    /// <summary>The status with the one flag <paramref name="theStatus"/> set.</summary>
    public Message_ExecStatus(Message_Status theStatus)
    {
        myDone = (int)Message_None;
        myWarn = (int)Message_None;
        myAlarm = (int)Message_None;
        myFail = (int)Message_None;
        Set(theStatus);
    }

    // ---- Creation and simple operations with statuses

    /// <summary>Sets a status flag.</summary>
    internal void Set(Message_Status theStatus)
    {
        switch (TypeOfStatus(theStatus))
        {
            case Message_DONE:
                myDone |= getBitFlag((int)theStatus);
                break;
            case Message_WARN:
                myWarn |= getBitFlag((int)theStatus);
                break;
            case Message_ALARM:
                myAlarm |= getBitFlag((int)theStatus);
                break;
            case Message_FAIL:
                myFail |= getBitFlag((int)theStatus);
                break;
        }
    }

    /// <summary>True if the status flag is set.</summary>
    public readonly bool IsSet(Message_Status theStatus)
    {
        switch (TypeOfStatus(theStatus))
        {
            case Message_DONE:
                return (myDone & getBitFlag((int)theStatus)) != 0;
            case Message_WARN:
                return (myWarn & getBitFlag((int)theStatus)) != 0;
            case Message_ALARM:
                return (myAlarm & getBitFlag((int)theStatus)) != 0;
            case Message_FAIL:
                return (myFail & getBitFlag((int)theStatus)) != 0;
        }
        return false;
    }

    /// <summary>Clears one status flag.</summary>
    internal void Clear(Message_Status theStatus)
    {
        switch (TypeOfStatus(theStatus))
        {
            case Message_DONE:
                myDone &= ~getBitFlag((int)theStatus);
                return;
            case Message_WARN:
                myWarn &= ~getBitFlag((int)theStatus);
                return;
            case Message_ALARM:
                myAlarm &= ~getBitFlag((int)theStatus);
                return;
            case Message_FAIL:
                myFail &= ~getBitFlag((int)theStatus);
                return;
        }
    }

    // ---- Group operations

    /// <summary>True if at least one Done flag is set.</summary>
    public readonly bool IsDone() => myDone != (int)Message_None;

    /// <summary>True if at least one Fail flag is set.</summary>
    public readonly bool IsFail() => myFail != (int)Message_None;

    /// <summary>True if at least one Warn flag is set.</summary>
    public readonly bool IsWarn() => myWarn != (int)Message_None;

    /// <summary>True if at least one Alarm flag is set.</summary>
    public readonly bool IsAlarm() => myAlarm != (int)Message_None;

    internal void SetAllDone() => myDone = ~0;

    internal void SetAllWarn() => myWarn = ~0;

    internal void SetAllAlarm() => myAlarm = ~0;

    internal void SetAllFail() => myFail = ~0;

    internal void ClearAllDone() => myDone = (int)Message_None;

    internal void ClearAllWarn() => myWarn = (int)Message_None;

    internal void ClearAllAlarm() => myAlarm = (int)Message_None;

    internal void ClearAllFail() => myFail = (int)Message_None;

    /// <summary>Clears all statuses.</summary>
    internal void Clear() => myDone = myWarn = myAlarm = myFail = (int)Message_None;

    /// <summary>Adds the statuses of <paramref name="theOther"/>.</summary>
    internal void Add(in Message_ExecStatus theOther)
    {
        myDone |= theOther.myDone;
        myWarn |= theOther.myWarn;
        myAlarm |= theOther.myAlarm;
        myFail |= theOther.myFail;
    }

    /// <summary>Keeps only the statuses common with <paramref name="theOther"/>.</summary>
    internal void And(in Message_ExecStatus theOther)
    {
        myDone &= theOther.myDone;
        myWarn &= theOther.myWarn;
        myAlarm &= theOther.myAlarm;
        myFail &= theOther.myFail;
    }

    // ---- Iteration and analysis of status flags (OCCT's enum StatusRange)

    public const int FirstStatus = 1;
    public const int StatusesPerType = 32;
    public const int NbStatuses = 128;
    public const int LastStatus = 129;

    /// <summary>The index of the status in the whole range [FirstStatus, LastStatus].</summary>
    public static int StatusIndex(Message_Status theStatus)
    {
        switch (TypeOfStatus(theStatus))
        {
            case Message_DONE:
                return 0 * StatusesPerType + LocalStatusIndex(theStatus);
            case Message_WARN:
                return 1 * StatusesPerType + LocalStatusIndex(theStatus);
            case Message_ALARM:
                return 2 * StatusesPerType + LocalStatusIndex(theStatus);
            case Message_FAIL:
                return 3 * StatusesPerType + LocalStatusIndex(theStatus);
        }
        return 0;
    }

    /// <summary>The index of the status inside its type, in [1, StatusesPerType].</summary>
    public static int LocalStatusIndex(Message_Status theStatus) => (int)(((uint)theStatus & (uint)MIndex) + 1);

    /// <summary>The type of the status (DONE, WARN, ALARM or FAIL).</summary>
    public static Message_StatusType TypeOfStatus(Message_Status theStatus) =>
        (Message_StatusType)((uint)theStatus & (uint)MType);

    /// <summary>The status with index <paramref name="theIndex"/> in the whole range [FirstStatus, LastStatus].</summary>
    public static Message_Status StatusByIndex(int theIndex)
    {
        int indx = theIndex - 1;
        if (indx < 32)
        {
            return (Message_Status)((int)Message_DONE + indx);
        }
        else if (indx < 64)
        {
            return (Message_Status)((int)Message_WARN + (indx - 32));
        }
        else if (indx < 96)
        {
            return (Message_Status)((int)Message_ALARM + (indx - 64));
        }
        else if (indx < 128)
        {
            return (Message_Status)((int)Message_FAIL + (indx - 96));
        }
        return Message_None;
    }

    // ---- Public twins of the mutators (additions)

    /// <summary>A copy with the status flag set.</summary>
    public readonly Message_ExecStatus With(Message_Status theStatus)
    {
        Message_ExecStatus aCopy = this;
        aCopy.Set(theStatus);
        return aCopy;
    }

    /// <summary>A copy with the status flag cleared.</summary>
    public readonly Message_ExecStatus Without(Message_Status theStatus)
    {
        Message_ExecStatus aCopy = this;
        aCopy.Clear(theStatus);
        return aCopy;
    }

    /// <summary>The empty status.</summary>
    public readonly Message_ExecStatus Cleared() => new();

    /// <summary>A copy with the statuses of <paramref name="theOther"/> added.</summary>
    public readonly Message_ExecStatus Added(in Message_ExecStatus theOther)
    {
        Message_ExecStatus aCopy = this;
        aCopy.Add(theOther);
        return aCopy;
    }

    /// <summary>A copy keeping only the statuses common with <paramref name="theOther"/>.</summary>
    public readonly Message_ExecStatus Anded(in Message_ExecStatus theOther)
    {
        Message_ExecStatus aCopy = this;
        aCopy.And(theOther);
        return aCopy;
    }

    public readonly Message_ExecStatus WithAllDone() => this with { myDone = ~0 };

    public readonly Message_ExecStatus WithAllWarn() => this with { myWarn = ~0 };

    public readonly Message_ExecStatus WithAllAlarm() => this with { myAlarm = ~0 };

    public readonly Message_ExecStatus WithAllFail() => this with { myFail = ~0 };

    public readonly Message_ExecStatus WithoutAllDone() => this with { myDone = 0 };

    public readonly Message_ExecStatus WithoutAllWarn() => this with { myWarn = 0 };

    public readonly Message_ExecStatus WithoutAllAlarm() => this with { myAlarm = 0 };

    public readonly Message_ExecStatus WithoutAllFail() => this with { myFail = 0 };

    /// <summary>The union of the statuses (OCCT's <c>|=</c>).</summary>
    public static Message_ExecStatus operator |(in Message_ExecStatus theLeft, in Message_ExecStatus theRight) =>
        theLeft.Added(theRight);

    /// <summary>The common statuses (OCCT's <c>&amp;=</c>).</summary>
    public static Message_ExecStatus operator &(in Message_ExecStatus theLeft, in Message_ExecStatus theRight) =>
        theLeft.Anded(theRight);

    public readonly bool Equals(Message_ExecStatus theOther) =>
        myDone == theOther.myDone && myWarn == theOther.myWarn && myAlarm == theOther.myAlarm && myFail == theOther.myFail;

    public override readonly bool Equals(object? theObj) => theObj is Message_ExecStatus anOther && Equals(anOther);

    public override readonly int GetHashCode() => HashCode.Combine(myDone, myWarn, myAlarm, myFail);

    public static bool operator ==(in Message_ExecStatus theLeft, in Message_ExecStatus theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Message_ExecStatus theLeft, in Message_ExecStatus theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() =>
        $"Message_ExecStatus(Done 0x{myDone:x8}, Warn 0x{myWarn:x8}, Alarm 0x{myAlarm:x8}, Fail 0x{myFail:x8})";
}
