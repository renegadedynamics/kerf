// Port of: src/FoundationClasses/TKernel/Message/Message_ExecStatus.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2003-2026 OPEN CASCADE SAS. Original implementation copyright (c) RINA S.p.A.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Kerf.Engine.Foundation.Status;
using static Kerf.Engine.Foundation.StatusType;

namespace Kerf.Engine.Foundation;

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
internal partial struct ExecStatus : IEquatable<ExecStatus>
{
    // Mask to separate bits indicating status type and index within the type (OCCT's enum StatusMask)
    private const int MType = 0x0000ff00;
    private const int MIndex = 0x000000ff;

    private static int GetBitFlag(int status) => 0x1 << (status & MIndex);

    private int _done;
    private int _warn;
    private int _alarm;
    private int _fail;

    /// <summary>The status with the one flag <paramref name="status"/> set.</summary>
    public ExecStatus(Status status)
    {
        _done = (int)None;
        _warn = (int)None;
        _alarm = (int)None;
        _fail = (int)None;
        Set(status);
    }

    // ---- Creation and simple operations with statuses

    /// <summary>Sets a status flag.</summary>
    internal void Set(Status status)
    {
        switch (TypeOfStatus(status))
        {
            case Done:
                _done |= GetBitFlag((int)status);
                break;
            case Warn:
                _warn |= GetBitFlag((int)status);
                break;
            case Alarm:
                _alarm |= GetBitFlag((int)status);
                break;
            case Fail:
                _fail |= GetBitFlag((int)status);
                break;
        }
    }

    /// <summary>True if the status flag is set.</summary>
    public readonly bool IsSet(Status status)
    {
        switch (TypeOfStatus(status))
        {
            case Done:
                return (_done & GetBitFlag((int)status)) != 0;
            case Warn:
                return (_warn & GetBitFlag((int)status)) != 0;
            case Alarm:
                return (_alarm & GetBitFlag((int)status)) != 0;
            case Fail:
                return (_fail & GetBitFlag((int)status)) != 0;
        }
        return false;
    }

    /// <summary>Clears one status flag.</summary>
    internal void Clear(Status status)
    {
        switch (TypeOfStatus(status))
        {
            case Done:
                _done &= ~GetBitFlag((int)status);
                return;
            case Warn:
                _warn &= ~GetBitFlag((int)status);
                return;
            case Alarm:
                _alarm &= ~GetBitFlag((int)status);
                return;
            case Fail:
                _fail &= ~GetBitFlag((int)status);
                return;
        }
    }

    // ---- Group operations

    /// <summary>True if at least one Done flag is set.</summary>
    public readonly bool IsDone => _done != (int)None;

    /// <summary>True if at least one Fail flag is set.</summary>
    public readonly bool IsFail => _fail != (int)None;

    /// <summary>True if at least one Warn flag is set.</summary>
    public readonly bool IsWarn => _warn != (int)None;

    /// <summary>True if at least one Alarm flag is set.</summary>
    public readonly bool IsAlarm => _alarm != (int)None;

    internal void SetAllDone() => _done = ~0;

    internal void SetAllWarn() => _warn = ~0;

    internal void SetAllAlarm() => _alarm = ~0;

    internal void SetAllFail() => _fail = ~0;

    internal void ClearAllDone() => _done = (int)None;

    internal void ClearAllWarn() => _warn = (int)None;

    internal void ClearAllAlarm() => _alarm = (int)None;

    internal void ClearAllFail() => _fail = (int)None;

    /// <summary>Clears all statuses.</summary>
    internal void Clear() => _done = _warn = _alarm = _fail = (int)None;

    /// <summary>Adds the statuses of <paramref name="other"/>.</summary>
    internal void Add(in ExecStatus other)
    {
        _done |= other._done;
        _warn |= other._warn;
        _alarm |= other._alarm;
        _fail |= other._fail;
    }

    /// <summary>Keeps only the statuses common with <paramref name="other"/>.</summary>
    internal void And(in ExecStatus other)
    {
        _done &= other._done;
        _warn &= other._warn;
        _alarm &= other._alarm;
        _fail &= other._fail;
    }

    // ---- Iteration and analysis of status flags (OCCT's enum StatusRange)

    public const int FirstStatus = 1;
    public const int StatusesPerType = 32;
    public const int NbStatuses = 128;
    public const int LastStatus = 129;

    /// <summary>The index of the status in the whole range [FirstStatus, LastStatus].</summary>
    public static int StatusIndex(Status status)
    {
        switch (TypeOfStatus(status))
        {
            case Done:
                return 0 * StatusesPerType + LocalStatusIndex(status);
            case Warn:
                return 1 * StatusesPerType + LocalStatusIndex(status);
            case Alarm:
                return 2 * StatusesPerType + LocalStatusIndex(status);
            case Fail:
                return 3 * StatusesPerType + LocalStatusIndex(status);
        }
        return 0;
    }

    /// <summary>The index of the status inside its type, in [1, StatusesPerType].</summary>
    public static int LocalStatusIndex(Status status) => (int)(((uint)status & (uint)MIndex) + 1);

    /// <summary>The type of the status (DONE, WARN, ALARM or FAIL).</summary>
    public static StatusType TypeOfStatus(Status status) =>
        (StatusType)((uint)status & (uint)MType);

    /// <summary>The status with index <paramref name="index"/> in the whole range [FirstStatus, LastStatus].</summary>
    public static Status StatusByIndex(int index)
    {
        int indx = index - 1;
        if (indx < 32)
        {
            return (Status)((int)Done + indx);
        }
        else if (indx < 64)
        {
            return (Status)((int)Warn + (indx - 32));
        }
        else if (indx < 96)
        {
            return (Status)((int)Alarm + (indx - 64));
        }
        else if (indx < 128)
        {
            return (Status)((int)Fail + (indx - 96));
        }
        return None;
    }

    // ---- Public twins of the mutators (additions)

    /// <summary>A copy with the status flag set.</summary>
    public readonly ExecStatus With(Status status)
    {
        ExecStatus copy = this;
        copy.Set(status);
        return copy;
    }

    /// <summary>A copy with the status flag cleared.</summary>
    public readonly ExecStatus Without(Status status)
    {
        ExecStatus copy = this;
        copy.Clear(status);
        return copy;
    }

    /// <summary>The empty status.</summary>
    public readonly ExecStatus Cleared() => new();

    /// <summary>A copy with the statuses of <paramref name="other"/> added.</summary>
    public readonly ExecStatus Added(in ExecStatus other)
    {
        ExecStatus copy = this;
        copy.Add(other);
        return copy;
    }

    /// <summary>A copy keeping only the statuses common with <paramref name="other"/>.</summary>
    public readonly ExecStatus Anded(in ExecStatus other)
    {
        ExecStatus copy = this;
        copy.And(other);
        return copy;
    }

    public readonly ExecStatus WithAllDone() => this with { _done = ~0 };

    public readonly ExecStatus WithAllWarn() => this with { _warn = ~0 };

    public readonly ExecStatus WithAllAlarm() => this with { _alarm = ~0 };

    public readonly ExecStatus WithAllFail() => this with { _fail = ~0 };

    public readonly ExecStatus WithoutAllDone() => this with { _done = 0 };

    public readonly ExecStatus WithoutAllWarn() => this with { _warn = 0 };

    public readonly ExecStatus WithoutAllAlarm() => this with { _alarm = 0 };

    public readonly ExecStatus WithoutAllFail() => this with { _fail = 0 };

    /// <summary>The union of the statuses (OCCT's <c>|=</c>).</summary>
    public static ExecStatus operator |(in ExecStatus left, in ExecStatus right) =>
        left.Added(right);

    /// <summary>The common statuses (OCCT's <c>&amp;=</c>).</summary>
    public static ExecStatus operator &(in ExecStatus left, in ExecStatus right) =>
        left.Anded(right);

    public readonly bool Equals(ExecStatus other) =>
        _done == other._done && _warn == other._warn && _alarm == other._alarm && _fail == other._fail;

    public override readonly bool Equals(object? obj) => obj is ExecStatus other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_done, _warn, _alarm, _fail);

    public static bool operator ==(in ExecStatus left, in ExecStatus right) => left.Equals(right);

    public static bool operator !=(in ExecStatus left, in ExecStatus right) => !left.Equals(right);

    public override readonly string ToString() =>
        $"Message_ExecStatus(Done 0x{_done:x8}, Warn 0x{_warn:x8}, Alarm 0x{_alarm:x8}, Fail 0x{_fail:x8})";
}
