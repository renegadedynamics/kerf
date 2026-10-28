// Port of: src/FoundationClasses/TKMath/gp/gp_Elips.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Elips.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// An ellipse in 3D space, defined by its major and minor radii and positioned by a right-handed coordinate
/// system: the origin is the centre, the X direction the major axis and the main direction the normal to its
/// plane. It is parameterised by <c>P(U) = O + MajorRad * cos(U) * XDir + MinorRad * sin(U) * YDir</c>.
/// </summary>
/// <remarks>
/// <c>new gp_Elips()</c> is OCCT's indefinite ellipse (major radius <see cref="Standard.RealLast"/>, minor radius
/// <see cref="Standard.RealSmall"/>); <c>default(gp_Elips)</c> is not valid.
/// </remarks>
public partial struct gp_Elips : IEquatable<gp_Elips>
{
    private gp_Ax2 pos;
    private double majorRadius;
    private double minorRadius;

    public gp_Elips()
    {
        pos = new gp_Ax2();
        majorRadius = RealLast();
        minorRadius = RealSmall();
    }

    /// <exception cref="Standard_ConstructionError">
    /// <paramref name="theMinorRadius"/> is negative or greater than <paramref name="theMajorRadius"/>.
    /// </exception>
    public gp_Elips(in gp_Ax2 theA2, double theMajorRadius, double theMinorRadius)
    {
        pos = theA2;
        majorRadius = theMajorRadius;
        minorRadius = theMinorRadius;
        Standard_ConstructionError_Raise_if(theMinorRadius < 0.0 || theMajorRadius < theMinorRadius,
                                            "gp_Elips() - invalid construction parameters");
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetAxis(in gp_Ax1 theA1) => pos.SetAxis(theA1);

    internal void SetLocation(in gp_Pnt theP) => pos.SetLocation(theP);

    internal void SetMajorRadius(double theMajorRadius)
    {
        Standard_ConstructionError_Raise_if(
            theMajorRadius < minorRadius,
            "gp_Elips::SetMajorRadius() - major radius should be greater or equal to minor radius");
        majorRadius = theMajorRadius;
    }

    internal void SetMinorRadius(double theMinorRadius)
    {
        Standard_ConstructionError_Raise_if(theMinorRadius < 0.0 || majorRadius < theMinorRadius,
                                            "gp_Elips::SetMinorRadius() - minor radius should be a "
                                            + "positive number lesser or equal to major radius");
        minorRadius = theMinorRadius;
    }

    internal void SetPosition(in gp_Ax2 theA2) => pos = theA2;

    internal void Mirror(in gp_Pnt P) => pos.Mirror(P);

    internal void Mirror(in gp_Ax1 A1) => pos.Mirror(A1);

    internal void Mirror(in gp_Ax2 A2) => pos.Mirror(A2);

    internal void Rotate(in gp_Ax1 theA1, double theAng) => pos.Rotate(theA1, theAng);

    internal void Scale(in gp_Pnt theP, double theS)
    {
        majorRadius *= theS;
        if (majorRadius < 0)
        {
            majorRadius = -majorRadius;
        }

        minorRadius *= theS;
        if (minorRadius < 0)
        {
            minorRadius = -minorRadius;
        }

        pos.Scale(theP, theS);
    }

    internal void Transform(in gp_Trsf theT)
    {
        majorRadius *= theT.ScaleFactor();
        if (majorRadius < 0)
        {
            majorRadius = -majorRadius;
        }

        minorRadius *= theT.ScaleFactor();
        if (minorRadius < 0)
        {
            minorRadius = -minorRadius;
        }

        pos.Transform(theT);
    }

    internal void Translate(in gp_Vec theV) => pos.Translate(theV);

    internal void Translate(in gp_Pnt theP1, in gp_Pnt theP2) => pos.Translate(theP1, theP2);

    // Copies with a mutator applied

    public readonly gp_Elips WithAxis(in gp_Ax1 theA1)
    {
        gp_Elips aCopy = this;
        aCopy.SetAxis(theA1);
        return aCopy;
    }

    public readonly gp_Elips WithLocation(in gp_Pnt theP)
    {
        gp_Elips aCopy = this;
        aCopy.SetLocation(theP);
        return aCopy;
    }

    public readonly gp_Elips WithMajorRadius(double theMajorRadius)
    {
        gp_Elips aCopy = this;
        aCopy.SetMajorRadius(theMajorRadius);
        return aCopy;
    }

    public readonly gp_Elips WithMinorRadius(double theMinorRadius)
    {
        gp_Elips aCopy = this;
        aCopy.SetMinorRadius(theMinorRadius);
        return aCopy;
    }

    public readonly gp_Elips WithPosition(in gp_Ax2 theA2)
    {
        gp_Elips aCopy = this;
        aCopy.SetPosition(theA2);
        return aCopy;
    }

    // Queries

    public readonly double Area() => M_PI * majorRadius * minorRadius;

    /// <summary>The main axis: the normal to the ellipse's plane through its centre.</summary>
    public readonly gp_Ax1 Axis() => pos.Axis();

    /// <summary>The first directrix, on the positive side of the major axis, parallel to the minor axis.</summary>
    /// <exception cref="Standard_ConstructionError">The eccentricity is zero (a circle).</exception>
    public readonly gp_Ax1 Directrix1()
    {
        double anE = Eccentricity();
        Standard_ConstructionError_Raise_if(anE <= gp.Resolution(), "gp_Elips::Directrix1() - zero eccentricity");
        gp_XYZ anOrig = pos.XDirection().XYZ();
        anOrig.Multiply(majorRadius / anE);
        anOrig.Add(pos.Location().XYZ());
        return new gp_Ax1(new gp_Pnt(anOrig), pos.YDirection());
    }

    /// <summary>The second directrix, symmetric to the first about the minor axis.</summary>
    /// <exception cref="Standard_ConstructionError">The eccentricity is zero (a circle).</exception>
    public readonly gp_Ax1 Directrix2()
    {
        double anE = Eccentricity();
        Standard_ConstructionError_Raise_if(anE <= gp.Resolution(), "gp_Elips::Directrix2() - zero eccentricity");
        gp_XYZ anOrig = pos.XDirection().XYZ();
        anOrig.Multiply(-majorRadius / anE);
        anOrig.Add(pos.Location().XYZ());
        return new gp_Ax1(new gp_Pnt(anOrig), pos.YDirection());
    }

    /// <summary>sqrt(Major^2 - Minor^2) / Major, in [0, 1); 0 when the major radius is 0.</summary>
    public readonly double Eccentricity()
    {
        if (majorRadius == 0.0)
        {
            return 0.0;
        }
        else
        {
            return Math.Sqrt(majorRadius * majorRadius - minorRadius * minorRadius) / majorRadius;
        }
    }

    /// <summary>The distance between the two foci.</summary>
    public readonly double Focal() => 2.0 * Math.Sqrt(majorRadius * majorRadius - minorRadius * minorRadius);

    /// <summary>The focus on the positive side of the major axis.</summary>
    public readonly gp_Pnt Focus1()
    {
        double aC = Math.Sqrt(majorRadius * majorRadius - minorRadius * minorRadius);
        gp_Pnt aPP = pos.Location();
        gp_Dir aDD = pos.XDirection();
        return new gp_Pnt(aPP.X() + aC * aDD.X(), aPP.Y() + aC * aDD.Y(), aPP.Z() + aC * aDD.Z());
    }

    /// <summary>The focus on the negative side of the major axis.</summary>
    public readonly gp_Pnt Focus2()
    {
        double aC = Math.Sqrt(majorRadius * majorRadius - minorRadius * minorRadius);
        gp_Pnt aPP = pos.Location();
        gp_Dir aDD = pos.XDirection();
        return new gp_Pnt(aPP.X() - aC * aDD.X(), aPP.Y() - aC * aDD.Y(), aPP.Z() - aC * aDD.Z());
    }

    /// <summary>The centre of the ellipse.</summary>
    public readonly gp_Pnt Location() => pos.Location();

    public readonly double MajorRadius() => majorRadius;

    public readonly double MinorRadius() => minorRadius;

    /// <summary>The focal parameter, Minor^2 / Major; 0 when the major radius is 0.</summary>
    public readonly double Parameter()
    {
        if (majorRadius == 0.0)
        {
            return 0.0;
        }
        else
        {
            return (minorRadius * minorRadius) / majorRadius;
        }
    }

    public readonly gp_Ax2 Position() => pos;

    /// <summary>The major axis.</summary>
    public readonly gp_Ax1 XAxis() => new(pos.Location(), pos.XDirection());

    /// <summary>The minor axis.</summary>
    public readonly gp_Ax1 YAxis() => new(pos.Location(), pos.YDirection());

    public readonly gp_Elips Mirrored(in gp_Pnt P)
    {
        gp_Elips E = this;
        E.pos.Mirror(P);
        return E;
    }

    public readonly gp_Elips Mirrored(in gp_Ax1 A1)
    {
        gp_Elips E = this;
        E.pos.Mirror(A1);
        return E;
    }

    public readonly gp_Elips Mirrored(in gp_Ax2 A2)
    {
        gp_Elips E = this;
        E.pos.Mirror(A2);
        return E;
    }

    public readonly gp_Elips Rotated(in gp_Ax1 theA1, double theAng)
    {
        gp_Elips anE = this;
        anE.pos.Rotate(theA1, theAng);
        return anE;
    }

    public readonly gp_Elips Scaled(in gp_Pnt theP, double theS)
    {
        gp_Elips anE = this;
        anE.majorRadius *= theS;
        if (anE.majorRadius < 0)
        {
            anE.majorRadius = -anE.majorRadius;
        }

        anE.minorRadius *= theS;
        if (anE.minorRadius < 0)
        {
            anE.minorRadius = -anE.minorRadius;
        }

        anE.pos.Scale(theP, theS);
        return anE;
    }

    public readonly gp_Elips Transformed(in gp_Trsf theT)
    {
        gp_Elips anE = this;
        anE.majorRadius *= theT.ScaleFactor();
        if (anE.majorRadius < 0)
        {
            anE.majorRadius = -anE.majorRadius;
        }

        anE.minorRadius *= theT.ScaleFactor();
        if (anE.minorRadius < 0)
        {
            anE.minorRadius = -anE.minorRadius;
        }

        anE.pos.Transform(theT);
        return anE;
    }

    public readonly gp_Elips Translated(in gp_Vec theV)
    {
        gp_Elips anE = this;
        anE.pos.Translate(theV);
        return anE;
    }

    public readonly gp_Elips Translated(in gp_Pnt theP1, in gp_Pnt theP2)
    {
        gp_Elips anE = this;
        anE.pos.Translate(theP1, theP2);
        return anE;
    }

    public readonly bool Equals(gp_Elips other)
        => pos.Equals(other.pos) && majorRadius.Equals(other.majorRadius) && minorRadius.Equals(other.minorRadius);

    public override readonly bool Equals(object? obj) => obj is gp_Elips other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(pos, majorRadius, minorRadius);

    public static bool operator ==(in gp_Elips theLeft, in gp_Elips theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Elips theLeft, in gp_Elips theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => FormattableString.Invariant($"gp_Elips({pos}, {majorRadius:R}, {minorRadius:R})");
}
