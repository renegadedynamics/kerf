// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_PointOnBis.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_PointOnBis.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A point of a bisecting curve's polygon: the parameters of its projections on the two curves it separates, its
/// parameter on the bisector, its distance to the curves and the point.
/// </summary>
/// <remarks>
/// A value type (translation rules, section 3): the setters are internal; <c>default</c> is OCCT's default (zeros,
/// not infinite).
/// </remarks>
public partial struct Bisector_PointOnBis : IEquatable<Bisector_PointOnBis>
{
    private double param1;
    private double param2;
    private double paramBis;
    private double distance;
    private bool infinite;
    private gp_Pnt2d point;

    /// <summary>All zeros, not infinite.</summary>
    public Bisector_PointOnBis()
    {
        param1 = 0.0;
        param2 = 0.0;
        paramBis = 0.0;
        distance = 0.0;
        infinite = false;
        point = new gp_Pnt2d();
    }

    /// <summary>The point P of parameter ParamBis on the bisector, of parameters Param1 and Param2 on the curves.</summary>
    public Bisector_PointOnBis(double Param1, double Param2, double ParamBis, double Distance, in gp_Pnt2d P)
    {
        param1 = Param1;
        param2 = Param2;
        paramBis = ParamBis;
        distance = Distance;
        point = P;
        infinite = false;
    }

    internal void ParamOnC1(double Param) => param1 = Param;

    internal void ParamOnC2(double Param) => param2 = Param;

    internal void ParamOnBis(double Param) => paramBis = Param;

    internal void Distance(double Distance) => distance = Distance;

    internal void Point(in gp_Pnt2d P) => point = P;

    internal void IsInfinite(bool Infinite) => infinite = Infinite;

    /// <summary>The parameter on the first curve.</summary>
    public readonly double ParamOnC1() => param1;

    /// <summary>The parameter on the second curve.</summary>
    public readonly double ParamOnC2() => param2;

    /// <summary>The parameter on the bisector.</summary>
    public readonly double ParamOnBis() => paramBis;

    /// <summary>The distance to the curves.</summary>
    public readonly double Distance() => distance;

    /// <summary>The point.</summary>
    public readonly gp_Pnt2d Point() => point;

    /// <summary>True if the point is at infinity.</summary>
    public readonly bool IsInfinite() => infinite;

    /// <summary>A copy with the parameter on the first curve changed.</summary>
    public readonly Bisector_PointOnBis WithParamOnC1(double Param)
    {
        Bisector_PointOnBis aCopy = this;
        aCopy.ParamOnC1(Param);
        return aCopy;
    }

    /// <summary>A copy with the parameter on the second curve changed.</summary>
    public readonly Bisector_PointOnBis WithParamOnC2(double Param)
    {
        Bisector_PointOnBis aCopy = this;
        aCopy.ParamOnC2(Param);
        return aCopy;
    }

    /// <summary>A copy with the parameter on the bisector changed.</summary>
    public readonly Bisector_PointOnBis WithParamOnBis(double Param)
    {
        Bisector_PointOnBis aCopy = this;
        aCopy.ParamOnBis(Param);
        return aCopy;
    }

    /// <summary>A copy with the distance changed.</summary>
    public readonly Bisector_PointOnBis WithDistance(double Distance)
    {
        Bisector_PointOnBis aCopy = this;
        aCopy.Distance(Distance);
        return aCopy;
    }

    /// <summary>A copy with the point changed.</summary>
    public readonly Bisector_PointOnBis WithPoint(in gp_Pnt2d P)
    {
        Bisector_PointOnBis aCopy = this;
        aCopy.Point(P);
        return aCopy;
    }

    /// <summary>A copy with the infinite flag changed.</summary>
    public readonly Bisector_PointOnBis WithIsInfinite(bool Infinite)
    {
        Bisector_PointOnBis aCopy = this;
        aCopy.IsInfinite(Infinite);
        return aCopy;
    }

    // OCCT's Dump (std::cout) is left out; ToString replaces it.

    /// <summary>Exact equality of every field.</summary>
    public readonly bool Equals(Bisector_PointOnBis other) =>
        param1.Equals(other.param1) && param2.Equals(other.param2) && paramBis.Equals(other.paramBis)
        && distance.Equals(other.distance) && infinite == other.infinite && point.Equals(other.point);

    /// <inheritdoc/>
    public override readonly bool Equals(object? obj) => obj is Bisector_PointOnBis other && Equals(other);

    /// <inheritdoc/>
    public override readonly int GetHashCode() => HashCode.Combine(param1, param2, paramBis, distance, infinite, point);

    /// <summary>Exact equality.</summary>
    public static bool operator ==(in Bisector_PointOnBis theLeft, in Bisector_PointOnBis theRight) =>
        theLeft.Equals(theRight);

    /// <summary>Exact inequality.</summary>
    public static bool operator !=(in Bisector_PointOnBis theLeft, in Bisector_PointOnBis theRight) =>
        !theLeft.Equals(theRight);

    /// <inheritdoc/>
    public override readonly string ToString() =>
        gp.Format("Bisector_PointOnBis", param1, param2, paramBis, distance, infinite ? 1.0 : 0.0, point.X(), point.Y());
}
