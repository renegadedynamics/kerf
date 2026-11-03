// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_POnCurv2d.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>A point on a 2D curve: its parameter on the curve and the point.</summary>
/// <remarks>
/// <c>new Extrema_POnCurv2d()</c> and <c>default(Extrema_POnCurv2d)</c> are the same "indefinite" point (parameter 0,
/// the origin). The constructor is the twin of the internal <c>SetValues</c>.
/// </remarks>
public partial struct Extrema_POnCurv2d : IEquatable<Extrema_POnCurv2d>
{
    private double myU;
    private Point2D myP;

    /// <summary>An indefinite point on curve.</summary>
    public Extrema_POnCurv2d()
    {
        myU = 0.0;
        myP = new Point2D();
    }

    /// <summary>A point on curve with its parameter value on the curve.</summary>
    public Extrema_POnCurv2d(double theU, in Point2D theP)
    {
        myU = theU;
        myP = theP;
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the point and parameter values.</summary>
    internal void SetValues(double theU, in Point2D theP)
    {
        myU = theU;
        myP = theP;
    }

    // Queries

    /// <summary>The point.</summary>
    public readonly Point2D Value() => myP;

    /// <summary>The parameter on the curve.</summary>
    public readonly double Parameter() => myU;

    public readonly bool Equals(Extrema_POnCurv2d other) => myU.Equals(other.myU) && myP.Equals(other.myP);

    public override readonly bool Equals(object? obj) => obj is Extrema_POnCurv2d other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myU, myP);

    public static bool operator ==(in Extrema_POnCurv2d theLeft, in Extrema_POnCurv2d theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Extrema_POnCurv2d theLeft, in Extrema_POnCurv2d theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"Extrema_POnCurv2d({myU.ToString("R", System.Globalization.CultureInfo.InvariantCulture)}, {myP})";
}
