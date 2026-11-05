// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_IntPoint.hxx
// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_IntPoint.lxx
// Port of: src/ModelingData/TKGeomBase/IntAna2d/IntAna2d_IntPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.TwoD;

/// <summary>
/// An intersection point between two 2D curves: the point, its parameter on the first curve and, unless the
/// second curve is given by an implicit equation, its parameter on the second curve.
/// </summary>
/// <remarks>
/// <c>new IntAna2d_IntPoint()</c> has every coordinate and parameter at <see cref="Standard.RealLast"/>, as in OCCT;
/// <c>default(IntAna2d_IntPoint)</c> is all zeros. OCCT's virtual <c>SetValue</c> methods are internal mutators
/// (no class derives from IntAna2d_IntPoint); the constructors are their non-mutating twins.
/// </remarks>
internal partial struct IntPoint : IEquatable<IntPoint>
{
    private double _myu1;
    private double _myu2;
    private Point2D _myp;
    private bool _myimplicit;

    /// <summary>An intersection point between two parametric 2D curves.</summary>
    public IntPoint(double x, double y, double u1, double u2)
    {
        _myu1 = u1;
        _myu2 = u2;
        _myp = new Point2D(x, y);
        _myimplicit = false;
    }

    /// <summary>An intersection point between a parametric 2D curve and a curve given by an implicit equation.</summary>
    public IntPoint(double x, double y, double u1)
    {
        _myu1 = u1;
        _myu2 = RealLast();
        _myp = new Point2D(x, y);
        _myimplicit = true;
    }

    public IntPoint()
    {
        _myu1 = RealLast();
        _myu2 = RealLast();
        _myp = new Point2D(RealLast(), RealLast());
        _myimplicit = false;
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the values for a "non-implicit" point.</summary>
    internal void SetValue(double x, double y, double u1, double u2)
    {
        _myimplicit = false;
        _myp = _myp.WithCoord(x, y);
        _myu1 = u1;
        _myu2 = u2;
    }

    /// <summary>Sets the values for an "implicit" point.</summary>
    internal void SetValue(double x, double y, double u1)
    {
        _myimplicit = true;
        _myp = _myp.WithCoord(x, y);
        _myu1 = u1;
        _myu2 = RealLast();
    }

    // Queries

    /// <summary>The geometric point.</summary>
    public readonly Point2D Value() => _myp;

    /// <summary>True if the second curve is implicit.</summary>
    public readonly bool SecondIsImplicit() => _myimplicit;

    /// <summary>The parameter on the first element.</summary>
    public readonly double ParamOnFirst() => _myu1;

    /// <summary>The parameter on the second element.</summary>
    /// <exception cref="DomainErrorException">The second element is an implicit curve.</exception>
    public readonly double ParamOnSecond()
    {
        if (_myimplicit)
        {
            throw new DomainErrorException();
        }

        return _myu2;
    }

    public readonly bool Equals(IntPoint other)
        => _myu1.Equals(other._myu1) && _myu2.Equals(other._myu2) && _myp.Equals(other._myp) && _myimplicit == other._myimplicit;

    public override readonly bool Equals(object? obj) => obj is IntPoint other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_myu1, _myu2, _myp, _myimplicit);

    public static bool operator ==(in IntPoint left, in IntPoint right) => left.Equals(right);

    public static bool operator !=(in IntPoint left, in IntPoint right) => !left.Equals(right);

    public override readonly string ToString()
        => _myimplicit
               ? Constants.Format("IntAna2d_IntPoint", _myp.X, _myp.Y, _myu1)
               : Constants.Format("IntAna2d_IntPoint", _myp.X, _myp.Y, _myu1, _myu2);
}
