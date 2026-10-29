// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_Range.hxx
// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_Range.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A range in 1D space bounded by two reals. A range can be void (contain no point).</summary>
/// <remarks>
/// OCCT's default (and <c>new Bnd_Range()</c>) is the void range [0, -1]. <c>default(Bnd_Range)</c> is not: it is
/// the range [0, 0]. Ported code must write <c>new Bnd_Range()</c>.
/// <para>
/// Inside this type OCCT's <c>std::min</c>/<c>std::max</c> are written <c>Standard.Min</c>/<c>Standard.Max</c>,
/// because the range's own <c>Min()</c> and <c>Max()</c> hide them.
/// </para>
/// </remarks>
public partial struct Bnd_Range : IEquatable<Bnd_Range>
{
    /// <summary>The range bounds (Min, Max).</summary>
    public readonly record struct Bounds(double Min, double Max);

    /// <summary>Status of the intersection with a periodic value (see <see cref="IsIntersected"/>).</summary>
    public enum IntersectStatus
    {
        /// <summary>No intersection with theVal+k*thePeriod.</summary>
        IntersectStatus_Out = 0,

        /// <summary>The range strictly contains theVal+k*thePeriod.</summary>
        IntersectStatus_In = 1,

        /// <summary>A range boundary coincides with theVal+k*thePeriod.</summary>
        IntersectStatus_Boundary = 2,
    }

    private double myFirst; // Start of range
    private double myLast;  // End   of range

    /// <summary>Creates a void range.</summary>
    public Bnd_Range()
    {
        myFirst = 0.0;
        myLast = -1.0;
    }

    /// <summary>Creates the range [theMin, theMax]; throws Standard_ConstructionError if theMax &lt; theMin.</summary>
    public Bnd_Range(double theMin, double theMax)
    {
        myFirst = theMin;
        myLast = theMax;
        if (myLast < myFirst)
        {
            throw new Standard_ConstructionError("Last < First");
        }
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Replaces this range with the common part of this range and <paramref name="theOther"/>.</summary>
    internal void Common(in Bnd_Range theOther)
    {
        if (theOther.IsVoid())
        {
            SetVoid();
            return;
        }

        if (IsVoid())
        {
            return;
        }

        myFirst = Standard.Max(myFirst, theOther.myFirst);
        myLast = Standard.Min(myLast, theOther.myLast);
    }

    /// <summary>Joins this range and <paramref name="theOther"/>; false (and no change) if either is void or they are separated.</summary>
    internal bool Union(in Bnd_Range theOther)
    {
        if (IsVoid() || theOther.IsVoid())
        {
            return false;
        }

        if (myLast < theOther.myFirst)
        {
            return false;
        }

        if (myFirst > theOther.myLast)
        {
            return false;
        }

        myFirst = Standard.Min(myFirst, theOther.myFirst);
        myLast = Standard.Max(myLast, theOther.myLast);

        return true;
    }

    internal void Add(double theParameter)
    {
        if (IsVoid())
        {
            myFirst = myLast = theParameter;
            return;
        }

        myFirst = Standard.Min(myFirst, theParameter);
        myLast = Standard.Max(myLast, theParameter);
    }

    internal void Add(in Bnd_Range theRange)
    {
        if (theRange.IsVoid())
        {
            return;
        }
        else if (IsVoid())
        {
            this = theRange;
            return;
        }

        myFirst = Standard.Min(myFirst, theRange.myFirst);
        myLast = Standard.Max(myLast, theRange.myLast);
    }

    internal void SetVoid()
    {
        myLast = -1.0;
        myFirst = 0.0;
    }

    internal void Enlarge(double theDelta)
    {
        if (IsVoid())
        {
            return;
        }

        myFirst -= theDelta;
        myLast += theDelta;
    }

    internal void Shift(double theVal)
    {
        if (!IsVoid())
        {
            myFirst += theVal;
            myLast += theVal;
        }
    }

    /// <summary>Trims the first value by the lower limit (the range becomes void if the limit is above its max).</summary>
    internal void TrimFrom(double theValLower)
    {
        if (!IsVoid())
        {
            myFirst = Standard.Max(myFirst, theValLower);
        }
    }

    /// <summary>Trims the last value by the upper limit (the range becomes void if the limit is below its min).</summary>
    internal void TrimTo(double theValUpper)
    {
        if (!IsVoid())
        {
            myLast = Standard.Min(myLast, theValUpper);
        }
    }

    // Copies with a mutator applied (the public way to "set")

    /// <summary>The common part of this range and <paramref name="theOther"/>.</summary>
    public readonly Bnd_Range Commoned(in Bnd_Range theOther)
    {
        Bnd_Range aCopy = this;
        aCopy.Common(theOther);
        return aCopy;
    }

    /// <summary>
    /// This range joined with <paramref name="theOther"/>; <paramref name="theIsDone"/> is false (and the range
    /// unchanged) if either is void or they are separated.
    /// </summary>
    public readonly Bnd_Range United(in Bnd_Range theOther, out bool theIsDone)
    {
        Bnd_Range aCopy = this;
        theIsDone = aCopy.Union(theOther);
        return aCopy;
    }

    public readonly Bnd_Range Added(double theParameter)
    {
        Bnd_Range aCopy = this;
        aCopy.Add(theParameter);
        return aCopy;
    }

    public readonly Bnd_Range Added(in Bnd_Range theRange)
    {
        Bnd_Range aCopy = this;
        aCopy.Add(theRange);
        return aCopy;
    }

    public readonly Bnd_Range WithVoid() => new();

    public readonly Bnd_Range Enlarged(double theDelta)
    {
        Bnd_Range aCopy = this;
        aCopy.Enlarge(theDelta);
        return aCopy;
    }

    public readonly Bnd_Range TrimmedFrom(double theValLower)
    {
        Bnd_Range aCopy = this;
        aCopy.TrimFrom(theValLower);
        return aCopy;
    }

    public readonly Bnd_Range TrimmedTo(double theValUpper)
    {
        Bnd_Range aCopy = this;
        aCopy.TrimTo(theValUpper);
        return aCopy;
    }

    // Queries

    /// <summary>
    /// Splits this range by theVal (and, if <paramref name="thePeriod"/> is not 0, by every theVal + k*thePeriod),
    /// appending the pieces to <paramref name="theList"/>.
    /// </summary>
    public readonly void Split(double theVal, NCollection_List<Bnd_Range> theList, double thePeriod = 0.0)
    {
        double aPeriod = Math.Abs(thePeriod);
        if (IsIntersected(theVal, aPeriod) != IntersectStatus.IntersectStatus_In)
        {
            theList.Append(this);
            return;
        }

        bool isPeriodic = (aPeriod > 0.0);

        if (!isPeriodic)
        {
            theList.Append(new Bnd_Range(myFirst, theVal));
            theList.Append(new Bnd_Range(theVal, myLast));
            return;
        }

        double aValPrev = theVal + aPeriod * Math.Ceiling((myFirst - theVal) / aPeriod);

        // Now, (myFirst <= aValPrev < myFirst+aPeriod).

        if (aValPrev > myFirst)
        {
            theList.Append(new Bnd_Range(myFirst, aValPrev));
        }

        for (double aVal = aValPrev + aPeriod; aVal <= myLast; aVal += aPeriod)
        {
            theList.Append(new Bnd_Range(aValPrev, aVal));
            aValPrev = aVal;
        }

        if (aValPrev < myLast)
        {
            theList.Append(new Bnd_Range(aValPrev, myLast));
        }
    }

    /// <summary>
    /// Whether this range intersects the values theVal + k*thePeriod (k integer). If the range is a single point,
    /// only Out or Boundary can be returned.
    /// </summary>
    public readonly IntersectStatus IsIntersected(double theVal, double thePeriod = 0.0)
    {
        if (IsVoid())
        {
            return IntersectStatus.IntersectStatus_Out;
        }

        double aPeriod = Math.Abs(thePeriod);
        double aDF = myFirst - theVal, aDL = myLast - theVal;

        if (aPeriod <= RealSmall())
        {
            double aDelta = aDF * aDL;
            if (IsEqual(aDelta, 0.0))
            {
                return IntersectStatus.IntersectStatus_Boundary;
            }

            if (aDelta > 0.0)
            {
                return IntersectStatus.IntersectStatus_Out;
            }

            return IntersectStatus.IntersectStatus_In;
        }

        // If <this> intersects theVal then there exists an integer
        // number N such as
        //     (myFirst <= theVal+aPeriod*N <= myLast) <=>
        //     ((myFirst-theVal)/aPeriod <= N <= (myLast-theVal)/aPeriod).
        // I.e. the interval [aDF/aPeriod, aDL/aPeriod] must contain at least one
        // integer number.
        // In this case, std::floor(aDF/aPeriod) and std::floor(aDL/aPeriod)
        // return different values or aDF/aPeriod (aDL/aPeriod)
        // is strictly integer number.

        double aVal1 = aDF / aPeriod, aVal2 = aDL / aPeriod;
        int aPar1 = (int)Math.Floor(aVal1);
        int aPar2 = (int)Math.Floor(aVal2);
        if (aPar1 != aPar2)
        { // Interval (myFirst, myLast] intersects seam-edge
            if (IsEqual(aVal2, (double)aPar2))
            { // aVal2 is an integer number => myLast lies ON the "seam-edge"
                return IntersectStatus.IntersectStatus_Boundary;
            }

            return IntersectStatus.IntersectStatus_In;
        }

        // Here, aPar1 == aPar2.

        if (IsEqual(aVal1, (double)aPar1))
        { // aVal1 is an integer number => myFirst lies ON the "seam-edge"
            return IntersectStatus.IntersectStatus_Boundary;
        }

        return IntersectStatus.IntersectStatus_Out;
    }

    /// <summary>The min boundary; false (and <paramref name="thePar"/> untouched) if void.</summary>
    public readonly bool GetMin(ref double thePar)
    {
        if (IsVoid())
        {
            return false;
        }

        thePar = myFirst;
        return true;
    }

    /// <summary>The max boundary; false (and <paramref name="thePar"/> untouched) if void.</summary>
    public readonly bool GetMax(ref double thePar)
    {
        if (IsVoid())
        {
            return false;
        }

        thePar = myLast;
        return true;
    }

    /// <summary>The boundaries; false (and the arguments untouched) if void.</summary>
    public readonly bool GetBounds(ref double theFirstPar, ref double theLastPar)
    {
        if (IsVoid())
        {
            return false;
        }

        theFirstPar = myFirst;
        theLastPar = myLast;
        return true;
    }

    /// <summary>The bounds, or null if void.</summary>
    public readonly Bounds? Get()
    {
        if (IsVoid())
        {
            return null;
        }

        return new Bounds(myFirst, myLast);
    }

    /// <summary>MIN + theLambda * (MAX - MIN); false (and <paramref name="theParameter"/> untouched) if void.</summary>
    public readonly bool GetIntermediatePoint(double theLambda, ref double theParameter)
    {
        if (IsVoid())
        {
            return false;
        }

        theParameter = myFirst + theLambda * (myLast - myFirst);
        return true;
    }

    /// <summary>(Min + Max) / 2, or null if void.</summary>
    public readonly double? Center()
    {
        if (IsVoid())
        {
            return null;
        }

        return 0.5 * (myFirst + myLast);
    }

    /// <summary>MAX - MIN (negative for a void range).</summary>
    public readonly double Delta() => (myLast - myFirst);

    public readonly bool IsVoid() => (myLast < myFirst);

    /// <summary>A copy shifted by theVal.</summary>
    public readonly Bnd_Range Shifted(double theVal)
        => !IsVoid() ? new Bnd_Range(myFirst + theVal, myLast + theVal) : new Bnd_Range();

    public readonly bool IsOut(double theValue) => IsVoid() || theValue < myFirst || theValue > myLast;

    public readonly bool IsOut(in Bnd_Range theRange)
        => IsVoid() || theRange.IsVoid() || theRange.myLast < myFirst || theRange.myFirst > myLast;

    public readonly bool Contains(double theValue) => !IsOut(theValue);

    public readonly bool Intersects(in Bnd_Range theRange) => !IsOut(theRange);

    /// <summary>The min boundary, or null if void.</summary>
    public readonly double? Min()
    {
        if (IsVoid())
        {
            return null;
        }

        return myFirst;
    }

    /// <summary>The max boundary, or null if void.</summary>
    public readonly double? Max()
    {
        if (IsVoid())
        {
            return null;
        }

        return myLast;
    }

    // DumpJson is left out: ToString replaces it.

    // Exact equality, for use as a value (OCCT's operator== compares the same two fields with ==)

    public readonly bool Equals(Bnd_Range other) => myFirst.Equals(other.myFirst) && myLast.Equals(other.myLast);

    public override readonly bool Equals(object? obj) => obj is Bnd_Range other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myFirst, myLast);

    public static bool operator ==(in Bnd_Range theLeft, in Bnd_Range theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Bnd_Range theLeft, in Bnd_Range theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => gp.Format("Bnd_Range", myFirst, myLast);
}
