// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_Domain.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_Domain.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_Domain.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// The domain of parameters on a 2D curve, usually bounded by two extremities, each made of a point, a parameter
/// and a tolerance; it may also have one extremity or none (an infinite or semi-infinite line). For the
/// intersection algorithms, the domains of ellipses and circles must be closed
/// (<see cref="WithEquivalentParameters"/> after the bounds are set).
/// </summary>
/// <remarks>
/// OCCT's default (an infinite domain, every field zero) is also <c>default(IntRes2d_Domain)</c>. The
/// <c>SetValues</c> and <c>SetEquivalentParameters</c> mutators are internal; the constructors and
/// <see cref="WithEquivalentParameters"/> are their twins. Infinite points and parameters are limited to
/// ±<see cref="Precision.Infinite"/>, as in OCCT.
/// </remarks>
public partial struct IntRes2d_Domain : IEquatable<IntRes2d_Domain>
{
    //-- hasfirst <--> status & 1
    //-- haslast  <--> status & 2
    //-- closed   <--> status & 4
    private int status;
    private double first_param;
    private double last_param;
    private double first_tol;
    private double last_tol;
    private Point2D first_point;
    private Point2D last_point;
    private double periodfirst;
    private double periodlast;

    // limit infinite points and parameters in order to make arithmetic operation on them safe
    private static readonly double infVal = global::Kerf.Engine.Foundation.Precision.Infinite();

    private static double LimitInfinite(double Val)
    {
        return (Math.Abs(Val) > infVal ? (Val > 0 ? infVal : -infVal) : Val);
    }

    /// <summary>Creates an infinite domain (no first point, no last point).</summary>
    public IntRes2d_Domain()
    {
        status = 0;
        first_param = 0.0;
        last_param = 0.0;
        first_tol = 0.0;
        last_tol = 0.0;
        first_point = new Point2D(0.0, 0.0);
        last_point = new Point2D(0.0, 0.0);
        periodfirst = 0.0;
        periodlast = 0.0;
    }

    /// <summary>Creates a bounded domain.</summary>
    public IntRes2d_Domain(in Point2D Pnt1, double Par1, double Tol1, in Point2D Pnt2, double Par2, double Tol2)
        : this()
    {
        SetValues(Pnt1, Par1, Tol1, Pnt2, Par2, Tol2);
    }

    /// <summary>
    /// Creates a semi-infinite domain: the given point is its first point if <paramref name="First"/> is true,
    /// its last point otherwise.
    /// </summary>
    public IntRes2d_Domain(in Point2D Pnt, double Par, double Tol, bool First)
        : this()
    {
        SetValues(Pnt, Par, Tol, First);
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the values for a bounded domain.</summary>
    internal void SetValues(in Point2D Pnt1, double Par1, double Tol1, in Point2D Pnt2, double Par2, double Tol2)
    {
        status = 3;
        periodfirst = periodlast = 0.0;

        first_param = LimitInfinite(Par1);
        first_point.SetCoord(LimitInfinite(Pnt1.X()), LimitInfinite(Pnt1.Y()));
        first_tol = Tol1;

        last_param = LimitInfinite(Par2);
        last_point.SetCoord(LimitInfinite(Pnt2.X()), LimitInfinite(Pnt2.Y()));
        last_tol = Tol2;
    }

    /// <summary>Sets the values for an infinite domain (the bounds' values are kept but not used).</summary>
    internal void SetValues()
    {
        status = 0;
        periodfirst = periodlast = 0.0;
    }

    /// <summary>Sets the values for a semi-infinite domain.</summary>
    internal void SetValues(in Point2D Pnt, double Par, double Tol, bool First)
    {
        periodfirst = periodlast = 0.0;
        if (First)
        {
            status = 1;
            first_param = LimitInfinite(Par);
            first_point.SetCoord(LimitInfinite(Pnt.X()), LimitInfinite(Pnt.Y()));
            first_tol = Tol;
        }
        else
        {
            status = 2;
            last_param = LimitInfinite(Par);
            last_point.SetCoord(LimitInfinite(Pnt.X()), LimitInfinite(Pnt.Y()));
            last_tol = Tol;
        }
    }

    /// <summary>Defines a closed domain.</summary>
    /// <exception cref="DomainErrorException">The domain is not bounded on both sides.</exception>
    internal void SetEquivalentParameters(double p_first, double p_last)
    {
        if ((status & 3) != 3)
        {
            throw new DomainErrorException();
        }

        status |= 4;
        periodfirst = p_first;
        periodlast = p_last;
    }

    // Queries

    /// <summary>True if the domain has a first point, defining the lowest admitted parameter on the curve.</summary>
    public readonly bool HasFirstPoint() => (status & 1) != 0;

    /// <summary>The parameter of the first point of the domain.</summary>
    /// <exception cref="DomainErrorException"><see cref="HasFirstPoint"/> is false.</exception>
    public readonly double FirstParameter()
    {
        if ((status & 1) == 0)
        {
            throw new DomainErrorException();
        }

        return (first_param);
    }

    /// <summary>The first point of the domain.</summary>
    /// <exception cref="DomainErrorException"><see cref="HasFirstPoint"/> is false.</exception>
    public readonly Point2D FirstPoint()
    {
        if ((status & 1) == 0)
        {
            throw new DomainErrorException();
        }

        return (first_point);
    }

    /// <summary>The tolerance of the first (left) bound.</summary>
    /// <exception cref="DomainErrorException"><see cref="HasFirstPoint"/> is false.</exception>
    public readonly double FirstTolerance()
    {
        if ((status & 1) == 0)
        {
            throw new DomainErrorException();
        }

        return (first_tol);
    }

    /// <summary>True if the domain has a last point, defining the highest admitted parameter on the curve.</summary>
    public readonly bool HasLastPoint() => (status & 2) != 0;

    /// <summary>The parameter of the last point of the domain.</summary>
    /// <exception cref="DomainErrorException"><see cref="HasLastPoint"/> is false.</exception>
    public readonly double LastParameter()
    {
        if ((status & 2) == 0)
        {
            throw new DomainErrorException();
        }

        return (last_param);
    }

    /// <summary>The last point of the domain.</summary>
    /// <exception cref="DomainErrorException"><see cref="HasLastPoint"/> is false.</exception>
    public readonly Point2D LastPoint()
    {
        if ((status & 2) == 0)
        {
            throw new DomainErrorException();
        }

        return (last_point);
    }

    /// <summary>The tolerance of the last (right) bound.</summary>
    /// <exception cref="DomainErrorException"><see cref="HasLastPoint"/> is false.</exception>
    public readonly double LastTolerance()
    {
        if ((status & 2) == 0)
        {
            throw new DomainErrorException();
        }

        return (last_tol);
    }

    /// <summary>True if the domain is closed.</summary>
    public readonly bool IsClosed() => (status & 4) != 0;

    /// <summary>
    /// The equivalent parameters of a closed domain (OCCT documents a DomainError on an open domain but doesn't
    /// raise it: the values are then 0).
    /// </summary>
    public readonly void EquivalentParameters(out double p_first, out double p_last)
    {
        p_first = periodfirst;
        p_last = periodlast;
    }

    // Non-mutating twins

    /// <summary>A closed copy of this bounded domain (the twin of SetEquivalentParameters).</summary>
    /// <exception cref="DomainErrorException">The domain is not bounded on both sides.</exception>
    public readonly IntRes2d_Domain WithEquivalentParameters(double p_first, double p_last)
    {
        IntRes2d_Domain aCopy = this;
        aCopy.SetEquivalentParameters(p_first, p_last);
        return aCopy;
    }

    public readonly bool Equals(IntRes2d_Domain other)
        => status == other.status && first_param.Equals(other.first_param) && last_param.Equals(other.last_param)
           && first_tol.Equals(other.first_tol) && last_tol.Equals(other.last_tol)
           && first_point.Equals(other.first_point) && last_point.Equals(other.last_point)
           && periodfirst.Equals(other.periodfirst) && periodlast.Equals(other.periodlast);

    public override readonly bool Equals(object? obj) => obj is IntRes2d_Domain other && Equals(other);

    public override readonly int GetHashCode()
    {
        HashCode aHash = new();
        aHash.Add(status);
        aHash.Add(first_param);
        aHash.Add(last_param);
        aHash.Add(first_tol);
        aHash.Add(last_tol);
        aHash.Add(first_point);
        aHash.Add(last_point);
        aHash.Add(periodfirst);
        aHash.Add(periodlast);
        return aHash.ToHashCode();
    }

    public static bool operator ==(in IntRes2d_Domain theLeft, in IntRes2d_Domain theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in IntRes2d_Domain theLeft, in IntRes2d_Domain theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => $"IntRes2d_Domain(status: {status}, "
           + Constants.Format("first", first_param, first_tol, first_point.X(), first_point.Y()) + ", "
           + Constants.Format("last", last_param, last_tol, last_point.X(), last_point.Y()) + ", "
           + Constants.Format("period", periodfirst, periodlast) + ")";
}
