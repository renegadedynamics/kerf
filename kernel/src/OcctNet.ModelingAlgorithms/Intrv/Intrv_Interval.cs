// Port of: src/ModelingAlgorithms/TKHLR/Intrv/Intrv_Interval.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Intrv/Intrv_Interval.lxx
// Port of: src/ModelingAlgorithms/TKHLR/Intrv/Intrv_Interval.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using static Occt.Intrv_Position;

namespace Occt;

/// <summary>
/// An interval of parameters with a (single precision) tolerance on each bound, and its position relative to
/// another one:
/// <code>
///                   **-----------****             Other
///     ***-----*                                   IsBefore
///     ***------------*                            IsJustBefore
///     ***-----------------*                       IsOverlappingAtStart
///     ***--------------------------*              IsJustEnclosingAtEnd
///     ***-------------------------------------*   IsEnclosing
///                  ***----*                       IsJustOverlappingAtStart
///                  ***-------------*              IsSimilar
///                  ***------------------------*   IsJustEnclosingAtStart
///                         ***-*                   IsInside
///                         ***------*              IsJustOverlappingAtEnd
///                         ***-----------------*   IsOverlappingAtEnd
///                                  ***--------*   IsJustAfter
///                                       ***---*   IsAfter
/// </code>
/// </summary>
/// <remarks>
/// A C++ value class, so a struct. <c>new()</c> is OCCT's empty constructor (the whole real line); <c>default</c> is
/// [0, 0] with null tolerances, so ported code writes <c>new()</c>. The mutators are internal. The tolerances are
/// <c>float</c> as OCCT's (<c>Standard_ShortReal</c>), and the arithmetic keeps OCCT's float/double promotions.
/// </remarks>
public partial struct Intrv_Interval : IEquatable<Intrv_Interval>
{
    private double myStart;
    private double myEnd;
    private float myTolStart;
    private float myTolEnd;

    /// <summary>The whole real line.</summary>
    public Intrv_Interval()
    {
        myStart = RealFirst();
        myEnd = RealLast();
        myTolStart = (float)Epsilon(RealFirst());
        myTolEnd = (float)Epsilon(RealLast());
    }

    /// <summary>[Start, End] with the bounds' epsilons as tolerances.</summary>
    public Intrv_Interval(double Start, double End)
    {
        myStart = Start;
        myEnd = End;
        myTolStart = (float)Epsilon(myStart);
        myTolEnd = (float)Epsilon(myEnd);
    }

    /// <summary>[Start, End] with the given tolerances (at least the bounds' epsilons).</summary>
    public Intrv_Interval(double Start, float TolStart, double End, float TolEnd)
    {
        myStart = Start;
        myEnd = End;
        myTolStart = TolStart;
        myTolEnd = TolEnd;
        float epsStart = (float)Epsilon(myStart);
        float epsEnd = (float)Epsilon(myEnd);
        if (myTolStart < epsStart)
        {
            myTolStart = epsStart;
        }
        if (myTolEnd < epsEnd)
        {
            myTolEnd = epsEnd;
        }
    }

    public readonly double Start() => myStart;

    public readonly double End() => myEnd;

    public readonly float TolStart() => myTolStart;

    public readonly float TolEnd() => myTolEnd;

    public readonly void Bounds(out double Start, out float TolStart, out double End, out float TolEnd)
    {
        Start = myStart;
        TolStart = myTolStart;
        End = myEnd;
        TolEnd = myTolEnd;
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetStart(double Start, float TolStart)
    {
        myStart = Start;
        myTolStart = TolStart;
    }

    /// <summary>Fuses the start with another one: the union of the two tolerance zones (none on an infinite start).</summary>
    internal void FuseAtStart(double Start, float TolStart)
    {
        if (myStart != RealFirst())
        {
            double a = Min(myStart - myTolStart, Start - TolStart);
            double b = Min(myStart + myTolStart, Start + TolStart);
            myStart = (a + b) / 2;
            myTolStart = (float)(b - a) / 2;
        }
    }

    /// <summary>Cuts the start by the end of a tool.</summary>
    internal void CutAtStart(double Start, float TolStart)
    {
        if (myStart != RealFirst())
        {
            double a = Max(myStart - myTolStart, Start - TolStart);
            double b = Max(myStart + myTolStart, Start + TolStart);
            myStart = (a + b) / 2;
            myTolStart = (float)(b - a) / 2;
        }
    }

    internal void SetEnd(double End, float TolEnd)
    {
        myEnd = End;
        myTolEnd = TolEnd;
    }

    /// <summary>Fuses the end with another one.</summary>
    internal void FuseAtEnd(double End, float TolEnd)
    {
        if (myEnd != RealLast())
        {
            double a = Max(myEnd - myTolEnd, End - TolEnd);
            double b = Max(myEnd + myTolEnd, End + TolEnd);
            myEnd = (a + b) / 2;
            myTolEnd = (float)(b - a) / 2;
        }
    }

    /// <summary>Cuts the end by the start of a tool.</summary>
    internal void CutAtEnd(double End, float TolEnd)
    {
        if (myEnd != RealLast())
        {
            double a = Min(myEnd - myTolEnd, End - TolEnd);
            double b = Min(myEnd + myTolEnd, End + TolEnd);
            myEnd = (a + b) / 2;
            myTolEnd = (float)(b - a) / 2;
        }
    }

    // OCCT's file-scope inline AreFused (Intrv_Interval.lxx)
    private static bool AreFused(double c1, float t1, double c2, float t2) => t1 + t2 >= Math.Abs(c1 - c2);

    /// <summary>True if the start and the end are fused within their tolerances.</summary>
    public readonly bool IsProbablyEmpty() => AreFused(myStart, myTolStart, myEnd, myTolEnd);

    /// <summary>The position of this interval relative to <paramref name="Other"/>.</summary>
    public readonly Intrv_Position Position(in Intrv_Interval Other)
    {
        double mySMin = myStart - myTolStart;
        double mySMax = myStart + myTolStart;
        double myEMin = myEnd - myTolEnd;
        double myEMax = myEnd + myTolEnd;
        double otSMin = Other.myStart - Other.myTolStart;
        double otSMax = Other.myStart + Other.myTolStart;
        double otEMin = Other.myEnd - Other.myTolEnd;
        double otEMax = Other.myEnd + Other.myTolEnd;
        Intrv_Position P;
        if (mySMax < otSMin)
        {
            if (myEMax < otSMin)
            {
                P = Intrv_Before;
            }
            else if (otSMax >= myEMin)
            {
                P = Intrv_JustBefore;
            }
            else if (myEMax < otEMin)
            {
                P = Intrv_OverlappingAtStart;
            }
            else if (otEMax >= myEMin)
            {
                P = Intrv_JustEnclosingAtEnd;
            }
            else
            {
                P = Intrv_Enclosing;
            }
        }
        else if (otSMax >= mySMin)
        {
            if (myEMax < otEMin)
            {
                P = Intrv_JustOverlappingAtStart;
            }
            else if (otEMax >= myEMin)
            {
                P = Intrv_Similar;
            }
            else
            {
                P = Intrv_JustEnclosingAtStart;
            }
        }
        else if (mySMax < otEMin)
        {
            if (myEMax < otEMin)
            {
                P = Intrv_Inside;
            }
            else if (otEMax >= myEMin)
            {
                P = Intrv_JustOverlappingAtEnd;
            }
            else
            {
                P = Intrv_OverlappingAtEnd;
            }
        }
        else if (otEMax >= mySMin)
        {
            P = Intrv_JustAfter;
        }
        else
        {
            P = Intrv_After;
        }
        return P;
    }

    public readonly bool IsBefore(in Intrv_Interval Other) => myTolEnd + Other.myTolStart < Other.myStart - myEnd;

    public readonly bool IsAfter(in Intrv_Interval Other) => myTolStart + Other.myTolEnd < myStart - Other.myEnd;

    public readonly bool IsInside(in Intrv_Interval Other)
        => myTolStart + Other.myTolStart < myStart - Other.myStart && myTolEnd + Other.myTolEnd < Other.myEnd - myEnd;

    public readonly bool IsEnclosing(in Intrv_Interval Other)
        => myTolStart + Other.myTolStart < Other.myStart - myStart && myTolEnd + Other.myTolEnd < myEnd - Other.myEnd;

    public readonly bool IsJustEnclosingAtStart(in Intrv_Interval Other)
        => AreFused(myStart, myTolStart, Other.myStart, Other.myTolStart)
           && myTolEnd + Other.myTolEnd < myEnd - Other.myEnd;

    public readonly bool IsJustEnclosingAtEnd(in Intrv_Interval Other)
        => myTolStart + Other.myTolStart < Other.myStart - myStart
           && AreFused(Other.myEnd, Other.myTolEnd, myEnd, myTolEnd);

    public readonly bool IsJustBefore(in Intrv_Interval Other)
        => AreFused(myEnd, myTolEnd, Other.myStart, Other.myTolStart);

    public readonly bool IsJustAfter(in Intrv_Interval Other)
        => AreFused(Other.myEnd, Other.myTolEnd, myStart, myTolStart);

    public readonly bool IsOverlappingAtStart(in Intrv_Interval Other)
        => myTolStart + Other.myTolStart < Other.myStart - myStart
           && myTolEnd + Other.myTolStart < myEnd - Other.myStart
           && myTolEnd + Other.myTolEnd < Other.myEnd - myEnd;

    public readonly bool IsOverlappingAtEnd(in Intrv_Interval Other)
        => myTolStart + Other.myTolStart < myStart - Other.myStart
           && myTolStart + Other.myTolEnd < Other.myEnd - myStart
           && myTolEnd + Other.myTolEnd < myEnd - Other.myEnd;

    public readonly bool IsJustOverlappingAtStart(in Intrv_Interval Other)
        => AreFused(myStart, myTolStart, Other.myStart, Other.myTolStart)
           && myTolEnd + Other.myTolEnd < Other.myEnd - myEnd;

    public readonly bool IsJustOverlappingAtEnd(in Intrv_Interval Other)
        => myTolStart + Other.myTolStart < myStart - Other.myStart
           && AreFused(Other.myEnd, Other.myTolEnd, myEnd, myTolEnd);

    public readonly bool IsSimilar(in Intrv_Interval Other)
    {
        bool b1, b2;
        b1 = AreFused(myStart, myTolStart, Other.myStart, Other.myTolStart);
        b2 = AreFused(myEnd, myTolEnd, Other.myEnd, Other.myTolEnd);
        return b1 && b2;
    }

    public readonly Intrv_Interval WithStart(double Start, float TolStart)
    {
        Intrv_Interval aCopy = this;
        aCopy.SetStart(Start, TolStart);
        return aCopy;
    }

    public readonly Intrv_Interval WithEnd(double End, float TolEnd)
    {
        Intrv_Interval aCopy = this;
        aCopy.SetEnd(End, TolEnd);
        return aCopy;
    }

    public readonly bool Equals(Intrv_Interval other)
        => myStart.Equals(other.myStart) && myEnd.Equals(other.myEnd) && myTolStart.Equals(other.myTolStart)
           && myTolEnd.Equals(other.myTolEnd);

    public override readonly bool Equals(object? obj) => obj is Intrv_Interval other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myStart, myEnd, myTolStart, myTolEnd);

    public static bool operator ==(in Intrv_Interval theLeft, in Intrv_Interval theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Intrv_Interval theLeft, in Intrv_Interval theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => Constants.Format("Intrv_Interval", myStart, myTolStart, myEnd, myTolEnd);
}
