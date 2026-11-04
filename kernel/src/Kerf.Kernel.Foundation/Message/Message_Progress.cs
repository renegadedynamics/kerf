// Port of: src/FoundationClasses/TKernel/Message/Message_ProgressIndicator.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_ProgressRange.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_ProgressScope.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Foundation;

/// <summary>
/// Progress and cancellation for long operations. A .NET caller makes one from a <see cref="CancellationToken"/>
/// and an optional <see cref="IProgress{T}"/> (0 to 1), and passes <see cref="Start"/> to the operation.
/// </summary>
/// <remarks>
/// OCCT's indicator is abstract, with a UserBreak and a Show to override; here both are the .NET types. Progress
/// is reported as OCCT computes it, and never moves backwards. Thread-safe, as OCCT's is.
/// </remarks>
internal sealed class ProgressIndicator
{
    private readonly CancellationToken _token;
    private readonly IProgress<double>? _progress;
    private readonly Lock _lock = new();
    private double _position;

    public ProgressIndicator(CancellationToken token = default, IProgress<double>? progress = null)
    {
        _token = token;
        _progress = progress;
    }

    /// <summary>The range covering the whole operation.</summary>
    public ProgressRange Start()
    {
        lock (_lock)
        {
            _position = 0.0;
        }

        return new ProgressRange(this, 0.0, 1.0);
    }

    /// <summary>The fraction done, from 0 to 1.</summary>
    public double GetPosition()
    {
        lock (_lock)
        {
            return _position;
        }
    }

    public bool UserBreak() => _token.IsCancellationRequested;

    internal void Increment(double step)
    {
        double position;
        lock (_lock)
        {
            _position = Math.Min(1.0, _position + step);
            position = _position;
        }

        _progress?.Report(position);
    }
}

/// <summary>
/// A share of an operation's progress, handed to a sub-operation. The default range is inactive: no progress, never
/// cancelled. A range that is never used is consumed when it is closed, as in OCCT; in C# that means calling
/// <see cref="Close"/> (C++ does it in the destructor), which ported code does where OCCT relies on it.
/// </summary>
public struct ProgressRange
{
    internal ProgressIndicator? _indicator;
    internal readonly double _start;
    internal double _delta;
    internal bool _wasUsed;

    internal ProgressRange(ProgressIndicator? indicator, double start, double delta)
    {
        _indicator = indicator;
        _start = start;
        _delta = delta;
        _wasUsed = false;
    }

    public readonly bool UserBreak() => _indicator is not null && _indicator.UserBreak();

    public readonly bool More() => !UserBreak();

    public readonly bool IsActive => !_wasUsed && _indicator is not null;

    /// <summary>Marks the range as done, advancing the indicator by its share if no scope used it.</summary>
    public void Close()
    {
        if (!IsActive)
        {
            return;
        }

        _indicator!.Increment(_delta);
        _indicator = null;
        _wasUsed = true;
    }
}

/// <summary>
/// Divides a range into steps: <c>for (Message_ProgressScope aPS(theRange, "name", n); aPS.More(); ) { op(aPS.Next()); }</c>.
/// Dispose it (or call <see cref="Close"/>) where the C++ scope would be destroyed, so the remaining share is counted.
/// </summary>
public sealed class ProgressScope : IDisposable
{
    private readonly ProgressIndicator? _indicator;
    private readonly double _portion;
    private readonly double _max;
    private double _value;
    private double _reported;
    private bool _isClosed;

    public ProgressScope(ProgressRange range, string? name, double max, bool isInfinite = false)
    {
        Name = name;
        _max = max > 0.0 ? max : 1.0;
        IsInfinite = isInfinite;
        if (range.IsActive)
        {
            _indicator = range._indicator;
            _portion = range._delta;
        }
        else
        {
            _isClosed = true;
        }
    }

    public string? Name { get; }

    public bool IsInfinite { get; }

    public double MaxValue() => _max;

    public double Value() => _value;

    public bool UserBreak() => _indicator is not null && _indicator.UserBreak();

    public bool More() => !UserBreak();

    public bool IsActive => !_isClosed && _indicator is not null;

    /// <summary>The range of the next <paramref name="step"/> steps, which advances this scope.</summary>
    public ProgressRange Next(double step = 1.0)
    {
        if (!IsActive || step <= 0.0)
        {
            return default;
        }

        double aFrom = _value;
        _value = Math.Min(_max, _value + step);
        double share = (_value - aFrom) / _max * _portion;
        _reported += share;
        return new ProgressRange(_indicator, 0.0, share);
    }

    /// <summary>Counts any share not handed out by <see cref="Next"/>, and closes the scope.</summary>
    public void Close()
    {
        if (!IsActive)
        {
            return;
        }

        double remaining = _portion - _reported;
        if (remaining > 0.0)
        {
            _indicator!.Increment(remaining);
        }

        _isClosed = true;
    }

    public void Dispose() => Close();
}
