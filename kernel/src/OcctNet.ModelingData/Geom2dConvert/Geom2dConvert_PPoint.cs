// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert_PPoint.hxx
// Port of: src/ModelingData/TKGeomBase/Geom2dConvert/Geom2dConvert_PPoint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Occt;

/// <summary>A point on a curve: its parameter, its 2D coordinates and its first derivative (the tangent).</summary>
/// <remarks>
/// The parameterless constructor gives OCCT's empty point (parameter RealLast(), point and derivative zero);
/// <c>default</c> has parameter 0. OCCT's <c>operator==</c> and <c>operator!=</c>, which compare the parameters
/// within Precision::PConfusion(), are <see cref="IsEqual"/>; <c>==</c> and <c>!=</c> compare exactly, as for every
/// value type of the port.
/// </remarks>
public partial struct Geom2dConvert_PPoint : IEquatable<Geom2dConvert_PPoint>
{
    private double myParameter; // Parameter value
    private Point2D myPoint; // Point location
    private Vector2D myD1; // derivatives by parameter (components of the tangent).

    /// <summary>The empty point: parameter RealLast(), point and derivative zero.</summary>
    public Geom2dConvert_PPoint()
    {
        myParameter = RealLast();
        myPoint = new Point2D(0.0, 0.0);
        myD1 = new Vector2D(0.0, 0.0);
    }

    /// <summary>The point of parameter theParameter at thePoint, with derivative theD1.</summary>
    public Geom2dConvert_PPoint(double theParameter, in Xy thePoint, in Xy theD1)
    {
        myParameter = theParameter;
        myPoint = new Point2D(thePoint);
        myD1 = new Vector2D(theD1);
    }

    /// <summary>The point of theAdaptor at theParameter, with its first derivative.</summary>
    public Geom2dConvert_PPoint(double theParameter, Adaptor2d_Curve2d theAdaptor)
    {
        myParameter = theParameter;
        theAdaptor.D1(theParameter, out myPoint, out myD1);
    }

    /// <summary>The distance between the two points.</summary>
    public readonly double Dist(in Geom2dConvert_PPoint theOth) => myPoint.Distance(theOth.myPoint);

    /// <summary>The parameter.</summary>
    public readonly double Parameter() => myParameter;

    /// <summary>The point.</summary>
    public readonly Xy Point() => myPoint.XY();

    /// <summary>The first derivative.</summary>
    public readonly Xy D1() => myD1.XY();

    internal void SetD1(in Xy theD1) => myD1.SetXY(theD1);

    /// <summary>A copy with the derivative theD1.</summary>
    public readonly Geom2dConvert_PPoint WithD1(in Xy theD1)
    {
        Geom2dConvert_PPoint aCopy = this;
        aCopy.SetD1(theD1);
        return aCopy;
    }

    /// <summary>
    /// OCCT's <c>operator==</c>: whether the parameters are within Precision::PConfusion(). (OCCT's
    /// <c>operator!=</c> is its negation.)
    /// </summary>
    public readonly bool IsEqual(in Geom2dConvert_PPoint theOther) =>
        Math.Abs(myParameter - theOther.Parameter()) <= global::Kerf.Engine.Foundation.Precision.PConfusion();

    // OCCT's operator!=, as it is written (the exact negation of IsEqual but for NaN).
    internal readonly bool IsNotEqual(in Geom2dConvert_PPoint theOther) =>
        Math.Abs(myParameter - theOther.Parameter()) > global::Kerf.Engine.Foundation.Precision.PConfusion();

    public readonly bool Equals(Geom2dConvert_PPoint other) =>
        myParameter.Equals(other.myParameter) && myPoint.Equals(other.myPoint) && myD1.Equals(other.myD1);

    public override readonly bool Equals(object? obj) => obj is Geom2dConvert_PPoint other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myParameter, myPoint, myD1);

    public static bool operator ==(in Geom2dConvert_PPoint theLeft, in Geom2dConvert_PPoint theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Geom2dConvert_PPoint theLeft, in Geom2dConvert_PPoint theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() =>
        $"Geom2dConvert_PPoint({myParameter.ToString("R", System.Globalization.CultureInfo.InvariantCulture)}, {myPoint}, {myD1})";
}
