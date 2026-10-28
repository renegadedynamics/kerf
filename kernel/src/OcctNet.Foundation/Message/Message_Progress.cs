// Port of: src/FoundationClasses/TKernel/Message/Message_ProgressIndicator.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_ProgressRange.hxx
// Port of: src/FoundationClasses/TKernel/Message/Message_ProgressScope.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2020-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Progress and cancellation for long operations. A .NET caller makes one from a <see cref="CancellationToken"/>
/// and an optional <see cref="IProgress{T}"/> (0 to 1), and passes <see cref="Start"/> to the operation.
/// </summary>
/// <remarks>
/// OCCT's indicator is abstract, with a UserBreak and a Show to override; here both are the .NET types. Progress
/// is reported as OCCT computes it, and never moves backwards. Thread-safe, as OCCT's is.
/// </remarks>
public sealed class Message_ProgressIndicator
{
    private readonly CancellationToken myToken;
    private readonly IProgress<double>? myProgress;
    private readonly Lock myLock = new();
    private double myPosition;

    public Message_ProgressIndicator(CancellationToken theToken = default, IProgress<double>? theProgress = null)
    {
        myToken = theToken;
        myProgress = theProgress;
    }

    /// <summary>The range covering the whole operation.</summary>
    public Message_ProgressRange Start()
    {
        lock (myLock)
        {
            myPosition = 0.0;
        }

        return new Message_ProgressRange(this, 0.0, 1.0);
    }

    /// <summary>The fraction done, from 0 to 1.</summary>
    public double GetPosition()
    {
        lock (myLock)
        {
            return myPosition;
        }
    }

    public bool UserBreak() => myToken.IsCancellationRequested;

    internal void Increment(double theStep)
    {
        double aPosition;
        lock (myLock)
        {
            myPosition = Math.Min(1.0, myPosition + theStep);
            aPosition = myPosition;
        }

        myProgress?.Report(aPosition);
    }
}

/// <summary>
/// A share of an operation's progress, handed to a sub-operation. The default range is inactive: no progress, never
/// cancelled. A range that is never used is consumed when it is closed, as in OCCT; in C# that means calling
/// <see cref="Close"/> (C++ does it in the destructor), which ported code does where OCCT relies on it.
/// </summary>
public struct Message_ProgressRange
{
    internal Message_ProgressIndicator? myIndicator;
    internal readonly double myStart;
    internal double myDelta;
    internal bool myWasUsed;

    internal Message_ProgressRange(Message_ProgressIndicator? theIndicator, double theStart, double theDelta)
    {
        myIndicator = theIndicator;
        myStart = theStart;
        myDelta = theDelta;
        myWasUsed = false;
    }

    public readonly bool UserBreak() => myIndicator is not null && myIndicator.UserBreak();

    public readonly bool More() => !UserBreak();

    public readonly bool IsActive() => !myWasUsed && myIndicator is not null;

    /// <summary>Marks the range as done, advancing the indicator by its share if no scope used it.</summary>
    public void Close()
    {
        if (!IsActive())
        {
            return;
        }

        myIndicator!.Increment(myDelta);
        myIndicator = null;
        myWasUsed = true;
    }
}

/// <summary>
/// Divides a range into steps: <c>for (Message_ProgressScope aPS(theRange, "name", n); aPS.More(); ) { op(aPS.Next()); }</c>.
/// Dispose it (or call <see cref="Close"/>) where the C++ scope would be destroyed, so the remaining share is counted.
/// </summary>
public sealed class Message_ProgressScope : IDisposable
{
    private readonly Message_ProgressIndicator? myIndicator;
    private readonly double myPortion;
    private readonly double myMax;
    private double myValue;
    private double myReported;
    private bool myIsClosed;

    public Message_ProgressScope(Message_ProgressRange theRange, string? theName, double theMax, bool theIsInfinite = false)
    {
        Name = theName;
        myMax = theMax > 0.0 ? theMax : 1.0;
        IsInfinite = theIsInfinite;
        if (theRange.IsActive())
        {
            myIndicator = theRange.myIndicator;
            myPortion = theRange.myDelta;
        }
        else
        {
            myIsClosed = true;
        }
    }

    public string? Name { get; }

    public bool IsInfinite { get; }

    public double MaxValue() => myMax;

    public double Value() => myValue;

    public bool UserBreak() => myIndicator is not null && myIndicator.UserBreak();

    public bool More() => !UserBreak();

    public bool IsActive() => !myIsClosed && myIndicator is not null;

    /// <summary>The range of the next <paramref name="theStep"/> steps, which advances this scope.</summary>
    public Message_ProgressRange Next(double theStep = 1.0)
    {
        if (!IsActive() || theStep <= 0.0)
        {
            return default;
        }

        double aFrom = myValue;
        myValue = Math.Min(myMax, myValue + theStep);
        double aShare = (myValue - aFrom) / myMax * myPortion;
        myReported += aShare;
        return new Message_ProgressRange(myIndicator, 0.0, aShare);
    }

    /// <summary>Counts any share not handed out by <see cref="Next"/>, and closes the scope.</summary>
    public void Close()
    {
        if (!IsActive())
        {
            return;
        }

        double aRemaining = myPortion - myReported;
        if (aRemaining > 0.0)
        {
            myIndicator!.Increment(aRemaining);
        }

        myIsClosed = true;
    }

    public void Dispose() => Close();
}
