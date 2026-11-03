// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_Range.hxx
// Port of: src/FoundationClasses/TKMath/Bnd/Bnd_Range.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2016-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Occt;

namespace Kerf.Engine.Geometry.Bounds;

/// <summary>A range in 1D space bounded by two reals. A range can be void (contain no point).</summary>
/// <remarks>
/// OCCT's default (and <c>new Bnd_Range()</c>) is the void range [0, -1]. <c>default(Bnd_Range)</c> is not: it is
/// the range [0, 0]. Ported code must write <c>new Bnd_Range()</c>.
/// <para>
/// Inside this type OCCT's <c>std::min</c>/<c>std::max</c> are written <c>Standard.Min</c>/<c>Standard.Max</c>,
/// because the range's own <c>Min()</c> and <c>Max()</c> hide them.
/// </para>
/// </remarks>
public partial struct BoundRange : IEquatable<BoundRange>
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

    private double _first; // Start of range
    private double _last;  // End   of range

    /// <summary>Creates a void range.</summary>
    public BoundRange()
    {
        _first = 0.0;
        _last = -1.0;
    }

    /// <summary>Creates the range [theMin, theMax]; throws Standard_ConstructionError if theMax &lt; theMin.</summary>
    public BoundRange(double min, double max)
    {
        _first = min;
        _last = max;
        if (_last < _first)
        {
            throw new ConstructionErrorException("Last < First");
        }
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Replaces this range with the common part of this range and <paramref name="other"/>.</summary>
    internal void Common(in BoundRange other)
    {
        if (other.IsVoid)
        {
            SetVoid();
            return;
        }

        if (IsVoid)
        {
            return;
        }

        _first = global::Kerf.Engine.Foundation.Standard.Max(_first, other._first);
        _last = global::Kerf.Engine.Foundation.Standard.Min(_last, other._last);
    }

    /// <summary>Joins this range and <paramref name="other"/>; false (and no change) if either is void or they are separated.</summary>
    internal bool Union(in BoundRange other)
    {
        if (IsVoid || other.IsVoid)
        {
            return false;
        }

        if (_last < other._first)
        {
            return false;
        }

        if (_first > other._last)
        {
            return false;
        }

        _first = global::Kerf.Engine.Foundation.Standard.Min(_first, other._first);
        _last = global::Kerf.Engine.Foundation.Standard.Max(_last, other._last);

        return true;
    }

    internal void Add(double parameter)
    {
        if (IsVoid)
        {
            _first = _last = parameter;
            return;
        }

        _first = global::Kerf.Engine.Foundation.Standard.Min(_first, parameter);
        _last = global::Kerf.Engine.Foundation.Standard.Max(_last, parameter);
    }

    internal void Add(in BoundRange range)
    {
        if (range.IsVoid)
        {
            return;
        }
        else if (IsVoid)
        {
            this = range;
            return;
        }

        _first = global::Kerf.Engine.Foundation.Standard.Min(_first, range._first);
        _last = global::Kerf.Engine.Foundation.Standard.Max(_last, range._last);
    }

    internal void SetVoid()
    {
        _last = -1.0;
        _first = 0.0;
    }

    internal void Enlarge(double delta)
    {
        if (IsVoid)
        {
            return;
        }

        _first -= delta;
        _last += delta;
    }

    internal void Shift(double val)
    {
        if (!IsVoid)
        {
            _first += val;
            _last += val;
        }
    }

    /// <summary>Trims the first value by the lower limit (the range becomes void if the limit is above its max).</summary>
    internal void TrimFrom(double valLower)
    {
        if (!IsVoid)
        {
            _first = global::Kerf.Engine.Foundation.Standard.Max(_first, valLower);
        }
    }

    /// <summary>Trims the last value by the upper limit (the range becomes void if the limit is below its min).</summary>
    internal void TrimTo(double valUpper)
    {
        if (!IsVoid)
        {
            _last = global::Kerf.Engine.Foundation.Standard.Min(_last, valUpper);
        }
    }

    // Copies with a mutator applied (the public way to "set")

    /// <summary>The common part of this range and <paramref name="other"/>.</summary>
    public readonly BoundRange Commoned(in BoundRange other)
    {
        BoundRange copy = this;
        copy.Common(other);
        return copy;
    }

    /// <summary>
    /// This range joined with <paramref name="other"/>; <paramref name="isDone"/> is false (and the range
    /// unchanged) if either is void or they are separated.
    /// </summary>
    public readonly BoundRange United(in BoundRange other, out bool isDone)
    {
        BoundRange copy = this;
        isDone = copy.Union(other);
        return copy;
    }

    public readonly BoundRange Added(double parameter)
    {
        BoundRange copy = this;
        copy.Add(parameter);
        return copy;
    }

    public readonly BoundRange Added(in BoundRange range)
    {
        BoundRange copy = this;
        copy.Add(range);
        return copy;
    }

    public readonly BoundRange WithVoid() => new();

    public readonly BoundRange Enlarged(double delta)
    {
        BoundRange copy = this;
        copy.Enlarge(delta);
        return copy;
    }

    public readonly BoundRange TrimmedFrom(double valLower)
    {
        BoundRange copy = this;
        copy.TrimFrom(valLower);
        return copy;
    }

    public readonly BoundRange TrimmedTo(double valUpper)
    {
        BoundRange copy = this;
        copy.TrimTo(valUpper);
        return copy;
    }

    // Queries

    /// <summary>
    /// Splits this range by theVal (and, if <paramref name="period"/> is not 0, by every theVal + k*thePeriod),
    /// appending the pieces to <paramref name="list"/>.
    /// </summary>
    public readonly void Split(double val, NCollection_List<BoundRange> list, double period = 0.0)
    {
        double aPeriod = Math.Abs(period);
        if (IsIntersected(val, aPeriod) != IntersectStatus.IntersectStatus_In)
        {
            list.Append(this);
            return;
        }

        bool isPeriodic = (aPeriod > 0.0);

        if (!isPeriodic)
        {
            list.Append(new BoundRange(_first, val));
            list.Append(new BoundRange(val, _last));
            return;
        }

        double valPrev = val + aPeriod * Math.Ceiling((_first - val) / aPeriod);

        // Now, (myFirst <= aValPrev < myFirst+aPeriod).

        if (valPrev > _first)
        {
            list.Append(new BoundRange(_first, valPrev));
        }

        for (double aVal = valPrev + aPeriod; aVal <= _last; aVal += aPeriod)
        {
            list.Append(new BoundRange(valPrev, aVal));
            valPrev = aVal;
        }

        if (valPrev < _last)
        {
            list.Append(new BoundRange(valPrev, _last));
        }
    }

    /// <summary>
    /// Whether this range intersects the values theVal + k*thePeriod (k integer). If the range is a single point,
    /// only Out or Boundary can be returned.
    /// </summary>
    public readonly IntersectStatus IsIntersected(double val, double period = 0.0)
    {
        if (IsVoid)
        {
            return IntersectStatus.IntersectStatus_Out;
        }

        double aPeriod = Math.Abs(period);
        double df = _first - val, dl = _last - val;

        if (aPeriod <= RealSmall())
        {
            double delta = df * dl;
            if (IsEqual(delta, 0.0))
            {
                return IntersectStatus.IntersectStatus_Boundary;
            }

            if (delta > 0.0)
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

        double val1 = df / aPeriod, val2 = dl / aPeriod;
        int par1 = (int)Math.Floor(val1);
        int par2 = (int)Math.Floor(val2);
        if (par1 != par2)
        { // Interval (myFirst, myLast] intersects seam-edge
            if (IsEqual(val2, (double)par2))
            { // aVal2 is an integer number => myLast lies ON the "seam-edge"
                return IntersectStatus.IntersectStatus_Boundary;
            }

            return IntersectStatus.IntersectStatus_In;
        }

        // Here, aPar1 == aPar2.

        if (IsEqual(val1, (double)par1))
        { // aVal1 is an integer number => myFirst lies ON the "seam-edge"
            return IntersectStatus.IntersectStatus_Boundary;
        }

        return IntersectStatus.IntersectStatus_Out;
    }

    /// <summary>The min boundary; false (and <paramref name="par"/> untouched) if void.</summary>
    public readonly bool GetMin(ref double par)
    {
        if (IsVoid)
        {
            return false;
        }

        par = _first;
        return true;
    }

    /// <summary>The max boundary; false (and <paramref name="par"/> untouched) if void.</summary>
    public readonly bool GetMax(ref double par)
    {
        if (IsVoid)
        {
            return false;
        }

        par = _last;
        return true;
    }

    /// <summary>The boundaries; false (and the arguments untouched) if void.</summary>
    public readonly bool GetBounds(ref double firstPar, ref double lastPar)
    {
        if (IsVoid)
        {
            return false;
        }

        firstPar = _first;
        lastPar = _last;
        return true;
    }

    /// <summary>The bounds, or null if void.</summary>
    public readonly Bounds? Get()
    {
        if (IsVoid)
        {
            return null;
        }

        return new Bounds(_first, _last);
    }

    /// <summary>MIN + theLambda * (MAX - MIN); false (and <paramref name="parameter"/> untouched) if void.</summary>
    public readonly bool GetIntermediatePoint(double lambda, ref double parameter)
    {
        if (IsVoid)
        {
            return false;
        }

        parameter = _first + lambda * (_last - _first);
        return true;
    }

    /// <summary>(Min + Max) / 2, or null if void.</summary>
    public readonly double? Center()
    {
        if (IsVoid)
        {
            return null;
        }

        return 0.5 * (_first + _last);
    }

    /// <summary>MAX - MIN (negative for a void range).</summary>
    public readonly double Delta() => (_last - _first);

    public readonly bool IsVoid => (_last < _first);

    /// <summary>A copy shifted by theVal.</summary>
    public readonly BoundRange Shifted(double val)
        => !IsVoid ? new BoundRange(_first + val, _last + val) : new BoundRange();

    public readonly bool IsOut(double value) => IsVoid || value < _first || value > _last;

    public readonly bool IsOut(in BoundRange range)
        => IsVoid || range.IsVoid || range._last < _first || range._first > _last;

    public readonly bool Contains(double value) => !IsOut(value);

    public readonly bool Intersects(in BoundRange range) => !IsOut(range);

    /// <summary>The min boundary, or null if void.</summary>
    public readonly double? Min()
    {
        if (IsVoid)
        {
            return null;
        }

        return _first;
    }

    /// <summary>The max boundary, or null if void.</summary>
    public readonly double? Max()
    {
        if (IsVoid)
        {
            return null;
        }

        return _last;
    }

    // DumpJson is left out: ToString replaces it.

    // Exact equality, for use as a value (OCCT's operator== compares the same two fields with ==)

    public readonly bool Equals(BoundRange other) => _first.Equals(other._first) && _last.Equals(other._last);

    public override readonly bool Equals(object? obj) => obj is BoundRange other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_first, _last);

    public static bool operator ==(in BoundRange left, in BoundRange right) => left.Equals(right);

    public static bool operator !=(in BoundRange left, in BoundRange right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("Bnd_Range", _first, _last);
}
