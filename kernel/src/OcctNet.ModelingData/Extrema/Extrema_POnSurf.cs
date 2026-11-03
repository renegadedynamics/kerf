// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_POnSurf.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using System.Globalization;

namespace Occt;

/// <summary>A point on a surface: its parameters on the surface and the point.</summary>
/// <remarks>
/// <c>new Extrema_POnSurf()</c> is OCCT's "indefinite" point, with parameters and coordinates at
/// <see cref="Precision.Infinite"/>; <c>default(Extrema_POnSurf)</c> is all zeros. The constructor is the twin of the
/// internal <c>SetParameters</c>.
/// </remarks>
public partial struct Extrema_POnSurf : IEquatable<Extrema_POnSurf>
{
    private double myU;
    private double myV;
    private Point3D myP;

    /// <summary>An indefinite point on surface.</summary>
    public Extrema_POnSurf()
    {
        myU = global::Kerf.Engine.Foundation.Precision.Infinite();
        myV = global::Kerf.Engine.Foundation.Precision.Infinite();
        myP = new Point3D(global::Kerf.Engine.Foundation.Precision.Infinite(), global::Kerf.Engine.Foundation.Precision.Infinite(), global::Kerf.Engine.Foundation.Precision.Infinite());
    }

    /// <summary>A point on surface with its parameter values on the surface.</summary>
    public Extrema_POnSurf(double theU, double theV, in Point3D theP)
    {
        myU = theU;
        myV = theV;
        myP = theP;
    }

    // Mutators (internal: see docs/translation-rules.md)

    /// <summary>Sets the parameters and the point.</summary>
    internal void SetParameters(double theU, double theV, in Point3D thePnt)
    {
        myU = theU;
        myV = theV;
        myP = thePnt;
    }

    // Queries

    /// <summary>The 3D point.</summary>
    public readonly Point3D Value() => myP;

    /// <summary>The parameter values on the surface.</summary>
    public readonly void Parameter(out double theU, out double theV)
    {
        theU = myU;
        theV = myV;
    }

    public readonly bool Equals(Extrema_POnSurf other) => myU.Equals(other.myU) && myV.Equals(other.myV) && myP.Equals(other.myP);

    public override readonly bool Equals(object? obj) => obj is Extrema_POnSurf other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myU, myV, myP);

    public static bool operator ==(in Extrema_POnSurf theLeft, in Extrema_POnSurf theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in Extrema_POnSurf theLeft, in Extrema_POnSurf theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString()
        => string.Create(CultureInfo.InvariantCulture, $"Extrema_POnSurf({myU:R}, {myV:R}, {myP})");
}
