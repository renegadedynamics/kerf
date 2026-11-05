// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_IntersectionPoint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_IntersectionPoint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntRes2d/IntRes2d_IntersectionPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.TwoD;

/// <summary>An intersection point between two 2D curves: the point, its parameters and transitions on both curves.</summary>
/// <remarks>
/// <c>new IntRes2d_IntersectionPoint()</c> is OCCT's default: the origin, both parameters at
/// <see cref="Standard.RealLast"/> and two default (undecided) transitions; <c>default(IntRes2d_IntersectionPoint)</c>
/// is all zeros. <c>SetValues</c> is an internal mutator; the constructor is its twin.
/// </remarks>
internal partial struct IntersectionPoint2D : IEquatable<IntersectionPoint2D>
{
    private Point2D _pt;
    private double _p1;
    private double _p2;
    private Transition _trans1;
    private Transition _trans2;

    /// <summary>Empty constructor.</summary>
    public IntersectionPoint2D()
    {
        _pt = new Point2D();
        _p1 = RealLast();
        _p2 = RealLast();
        _trans1 = new Transition();
        _trans2 = new Transition();
    }

    /// <summary>
    /// Creates an intersection point. If <paramref name="reversedFlag"/> is false, <paramref name="uc1"/> and
    /// <paramref name="trans1"/> (resp. <paramref name="uc2"/> and <paramref name="trans2"/>) refer to the first
    /// curve (resp. the second); otherwise to the second (resp. the first).
    /// </summary>
    public IntersectionPoint2D(in Point2D p, double uc1, double uc2, in Transition trans1,
                                      in Transition trans2, bool reversedFlag)
    {
        _pt = p;
        _p1 = uc1;
        _p2 = uc2;
        _trans1 = trans1;
        _trans2 = trans2;
        if (reversedFlag)
        {
            _trans1 = trans2;
            _trans2 = trans1;
            _p1 = uc2;
            _p2 = uc1;
        }
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the values of the point; the parameters mean what they mean for the constructor.</summary>
    internal void SetValues(in Point2D p, double uc1, double uc2, in Transition trans1,
                            in Transition trans2, bool reversedFlag)
    {
        _pt = p;
        if (!reversedFlag)
        {
            _trans1 = trans1;
            _trans2 = trans2;
            _p1 = uc1;
            _p2 = uc2;
        }
        else
        {
            _trans1 = trans2;
            _trans2 = trans1;
            _p1 = uc2;
            _p2 = uc1;
        }
    }

    // Queries

    /// <summary>The intersection point in the 2D space.</summary>
    public readonly Point2D Value() => _pt;

    /// <summary>The parameter on the first curve.</summary>
    public readonly double ParamOnFirst() => _p1;

    /// <summary>The parameter on the second curve.</summary>
    public readonly double ParamOnSecond() => _p2;

    /// <summary>The transition of the first curve compared to the second one.</summary>
    public readonly Transition TransitionOfFirst() => _trans1;

    /// <summary>The transition of the second curve compared to the first one.</summary>
    public readonly Transition TransitionOfSecond() => _trans2;

    public readonly bool Equals(IntersectionPoint2D other)
        => _pt.Equals(other._pt) && _p1.Equals(other._p1) && _p2.Equals(other._p2) && _trans1.Equals(other._trans1)
           && _trans2.Equals(other._trans2);

    public override readonly bool Equals(object? obj) => obj is IntersectionPoint2D other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_pt, _p1, _p2, _trans1, _trans2);

    public static bool operator ==(in IntersectionPoint2D left, in IntersectionPoint2D right)
        => left.Equals(right);

    public static bool operator !=(in IntersectionPoint2D left, in IntersectionPoint2D right)
        => !left.Equals(right);

    public override readonly string ToString()
        => $"{Constants.Format("IntRes2d_IntersectionPoint", _pt.X, _pt.Y, _p1, _p2)} {_trans1} {_trans2}";
}
