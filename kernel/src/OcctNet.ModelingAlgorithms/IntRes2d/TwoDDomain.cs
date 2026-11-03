// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_Domain.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_Domain.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_Domain.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.TwoD;

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
public partial struct TwoDDomain : IEquatable<TwoDDomain>
{
    //-- hasfirst <--> status & 1
    //-- haslast  <--> status & 2
    //-- closed   <--> status & 4
    private int _status;
    private double first_param;
    private double last_param;
    private double first_tol;
    private double last_tol;
    private Point2D first_point;
    private Point2D last_point;
    private double _periodfirst;
    private double _periodlast;

    // limit infinite points and parameters in order to make arithmetic operation on them safe
    private static readonly double InfVal = global::Kerf.Engine.Foundation.Precision.Infinite();

    private static double LimitInfinite(double val)
    {
        return (Math.Abs(val) > InfVal ? (val > 0 ? InfVal : -InfVal) : val);
    }

    /// <summary>Creates an infinite domain (no first point, no last point).</summary>
    public TwoDDomain()
    {
        _status = 0;
        first_param = 0.0;
        last_param = 0.0;
        first_tol = 0.0;
        last_tol = 0.0;
        first_point = new Point2D(0.0, 0.0);
        last_point = new Point2D(0.0, 0.0);
        _periodfirst = 0.0;
        _periodlast = 0.0;
    }

    /// <summary>Creates a bounded domain.</summary>
    public TwoDDomain(in Point2D pnt1, double par1, double tol1, in Point2D pnt2, double par2, double tol2)
        : this()
    {
        SetValues(pnt1, par1, tol1, pnt2, par2, tol2);
    }

    /// <summary>
    /// Creates a semi-infinite domain: the given point is its first point if <paramref name="first"/> is true,
    /// its last point otherwise.
    /// </summary>
    public TwoDDomain(in Point2D pnt, double par, double tol, bool first)
        : this()
    {
        SetValues(pnt, par, tol, first);
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the values for a bounded domain.</summary>
    internal void SetValues(in Point2D pnt1, double par1, double tol1, in Point2D pnt2, double par2, double tol2)
    {
        _status = 3;
        _periodfirst = _periodlast = 0.0;

        first_param = LimitInfinite(par1);
        first_point.SetCoord(LimitInfinite(pnt1.X()), LimitInfinite(pnt1.Y()));
        first_tol = tol1;

        last_param = LimitInfinite(par2);
        last_point.SetCoord(LimitInfinite(pnt2.X()), LimitInfinite(pnt2.Y()));
        last_tol = tol2;
    }

    /// <summary>Sets the values for an infinite domain (the bounds' values are kept but not used).</summary>
    internal void SetValues()
    {
        _status = 0;
        _periodfirst = _periodlast = 0.0;
    }

    /// <summary>Sets the values for a semi-infinite domain.</summary>
    internal void SetValues(in Point2D pnt, double par, double tol, bool first)
    {
        _periodfirst = _periodlast = 0.0;
        if (first)
        {
            _status = 1;
            first_param = LimitInfinite(par);
            first_point.SetCoord(LimitInfinite(pnt.X()), LimitInfinite(pnt.Y()));
            first_tol = tol;
        }
        else
        {
            _status = 2;
            last_param = LimitInfinite(par);
            last_point.SetCoord(LimitInfinite(pnt.X()), LimitInfinite(pnt.Y()));
            last_tol = tol;
        }
    }

    /// <summary>Defines a closed domain.</summary>
    /// <exception cref="DomainErrorException">The domain is not bounded on both sides.</exception>
    internal void SetEquivalentParameters(double p_first, double p_last)
    {
        if ((_status & 3) != 3)
        {
            throw new DomainErrorException();
        }

        _status |= 4;
        _periodfirst = p_first;
        _periodlast = p_last;
    }

    // Queries

    /// <summary>True if the domain has a first point, defining the lowest admitted parameter on the curve.</summary>
    public readonly bool HasFirstPoint => (_status & 1) != 0;

    /// <summary>The parameter of the first point of the domain.</summary>
    /// <exception cref="DomainErrorException"><see cref="HasFirstPoint"/> is false.</exception>
    public readonly double FirstParameter()
    {
        if ((_status & 1) == 0)
        {
            throw new DomainErrorException();
        }

        return (first_param);
    }

    /// <summary>The first point of the domain.</summary>
    /// <exception cref="DomainErrorException"><see cref="HasFirstPoint"/> is false.</exception>
    public readonly Point2D FirstPoint()
    {
        if ((_status & 1) == 0)
        {
            throw new DomainErrorException();
        }

        return (first_point);
    }

    /// <summary>The tolerance of the first (left) bound.</summary>
    /// <exception cref="DomainErrorException"><see cref="HasFirstPoint"/> is false.</exception>
    public readonly double FirstTolerance()
    {
        if ((_status & 1) == 0)
        {
            throw new DomainErrorException();
        }

        return (first_tol);
    }

    /// <summary>True if the domain has a last point, defining the highest admitted parameter on the curve.</summary>
    public readonly bool HasLastPoint => (_status & 2) != 0;

    /// <summary>The parameter of the last point of the domain.</summary>
    /// <exception cref="DomainErrorException"><see cref="HasLastPoint"/> is false.</exception>
    public readonly double LastParameter()
    {
        if ((_status & 2) == 0)
        {
            throw new DomainErrorException();
        }

        return (last_param);
    }

    /// <summary>The last point of the domain.</summary>
    /// <exception cref="DomainErrorException"><see cref="HasLastPoint"/> is false.</exception>
    public readonly Point2D LastPoint()
    {
        if ((_status & 2) == 0)
        {
            throw new DomainErrorException();
        }

        return (last_point);
    }

    /// <summary>The tolerance of the last (right) bound.</summary>
    /// <exception cref="DomainErrorException"><see cref="HasLastPoint"/> is false.</exception>
    public readonly double LastTolerance()
    {
        if ((_status & 2) == 0)
        {
            throw new DomainErrorException();
        }

        return (last_tol);
    }

    /// <summary>True if the domain is closed.</summary>
    public readonly bool IsClosed => (_status & 4) != 0;

    /// <summary>
    /// The equivalent parameters of a closed domain (OCCT documents a DomainError on an open domain but doesn't
    /// raise it: the values are then 0).
    /// </summary>
    public readonly void EquivalentParameters(out double p_first, out double p_last)
    {
        p_first = _periodfirst;
        p_last = _periodlast;
    }

    // Non-mutating twins

    /// <summary>A closed copy of this bounded domain (the twin of SetEquivalentParameters).</summary>
    /// <exception cref="DomainErrorException">The domain is not bounded on both sides.</exception>
    public readonly TwoDDomain WithEquivalentParameters(double p_first, double p_last)
    {
        TwoDDomain copy = this;
        copy.SetEquivalentParameters(p_first, p_last);
        return copy;
    }

    public readonly bool Equals(TwoDDomain other)
        => _status == other._status && first_param.Equals(other.first_param) && last_param.Equals(other.last_param)
           && first_tol.Equals(other.first_tol) && last_tol.Equals(other.last_tol)
           && first_point.Equals(other.first_point) && last_point.Equals(other.last_point)
           && _periodfirst.Equals(other._periodfirst) && _periodlast.Equals(other._periodlast);

    public override readonly bool Equals(object? obj) => obj is TwoDDomain other && Equals(other);

    public override readonly int GetHashCode()
    {
        HashCode hash = new();
        hash.Add(_status);
        hash.Add(first_param);
        hash.Add(last_param);
        hash.Add(first_tol);
        hash.Add(last_tol);
        hash.Add(first_point);
        hash.Add(last_point);
        hash.Add(_periodfirst);
        hash.Add(_periodlast);
        return hash.ToHashCode();
    }

    public static bool operator ==(in TwoDDomain left, in TwoDDomain right) => left.Equals(right);

    public static bool operator !=(in TwoDDomain left, in TwoDDomain right) => !left.Equals(right);

    public override readonly string ToString()
        => $"IntRes2d_Domain(status: {_status}, "
           + Constants.Format("first", first_param, first_tol, first_point.X(), first_point.Y()) + ", "
           + Constants.Format("last", last_param, last_tol, last_point.X(), last_point.Y()) + ", "
           + Constants.Format("period", _periodfirst, _periodlast) + ")";
}
