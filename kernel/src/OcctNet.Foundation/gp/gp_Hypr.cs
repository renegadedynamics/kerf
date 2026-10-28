// Port of: src/FoundationClasses/TKMath/gp/gp_Hypr.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Hypr.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A branch of a hyperbola in 3D space, defined by its major and minor radii and positioned by a right-handed
/// coordinate system: the origin is the centre, the X direction the major axis (the branch lies on its positive
/// side) and the main direction the normal to its plane. It is parameterised by
/// <c>P(U) = O + MajRad * cosh(U) * XDir + MinRad * sinh(U) * YDir</c>.
/// </summary>
/// <remarks>
/// <c>new gp_Hypr()</c> is OCCT's indefinite hyperbola (major radius <see cref="Standard.RealLast"/>, minor radius
/// <see cref="Standard.RealFirst"/>); <c>default(gp_Hypr)</c> is not valid.
/// </remarks>
public partial struct gp_Hypr : IEquatable<gp_Hypr>
{
    private gp_Ax2 pos;
    private double majorRadius;
    private double minorRadius;

    public gp_Hypr()
    {
        pos = new gp_Ax2();
        majorRadius = RealLast();
        minorRadius = RealFirst(); // OCCT: a negative minor radius, which no Set method or constructor accepts
    }

    /// <exception cref="Standard_ConstructionError">A radius is negative.</exception>
    public gp_Hypr(in gp_Ax2 theA2, double theMajorRadius, double theMinorRadius)
    {
        pos = theA2;
        majorRadius = theMajorRadius;
        minorRadius = theMinorRadius;
        Standard_ConstructionError_Raise_if(theMinorRadius < 0.0 || theMajorRadius < 0.0,
                                            "gp_Hypr() - invalid construction parameters");
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetAxis(in gp_Ax1 theA1) => pos.SetAxis(theA1);

    internal void SetLocation(in gp_Pnt theP) => pos = new gp_Ax2(theP, pos.Direction(), pos.XDirection());

    internal void SetMajorRadius(double theMajorRadius)
    {
        Standard_ConstructionError_Raise_if(
            theMajorRadius < 0.0,
            "gp_Hypr::SetMajorRadius() - major radius should be greater or equal zero");
        majorRadius = theMajorRadius;
    }

    internal void SetMinorRadius(double theMinorRadius)
    {
        Standard_ConstructionError_Raise_if(
            theMinorRadius < 0.0,
            "gp_Hypr::SetMinorRadius() - minor radius should be greater or equal zero");
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

    public readonly gp_Hypr WithAxis(in gp_Ax1 theA1)
    {
        gp_Hypr aCopy = this;
        aCopy.SetAxis(theA1);
        return aCopy;
    }

    public readonly gp_Hypr WithLocation(in gp_Pnt theP)
    {
        gp_Hypr aCopy = this;
        aCopy.SetLocation(theP);
        return aCopy;
    }

    public readonly gp_Hypr WithMajorRadius(double theMajorRadius)
    {
        gp_Hypr aCopy = this;
        aCopy.SetMajorRadius(theMajorRadius);
        return aCopy;
    }

    public readonly gp_Hypr WithMinorRadius(double theMinorRadius)
    {
        gp_Hypr aCopy = this;
        aCopy.SetMinorRadius(theMinorRadius);
        return aCopy;
    }

    public readonly gp_Hypr WithPosition(in gp_Ax2 theA2)
    {
        gp_Hypr aCopy = this;
        aCopy.SetPosition(theA2);
        return aCopy;
    }

    // Queries

    /// <summary>The first asymptote, of direction <c>XDir + (Minor / Major) * YDir</c>.</summary>
    /// <exception cref="Standard_ConstructionError">The major radius is zero.</exception>
    public readonly gp_Ax1 Asymptote1()
    {
        Standard_ConstructionError_Raise_if(majorRadius <= gp.Resolution(), "gp_Hypr::Asymptote1() - major radius is zero");
        gp_Vec aV1 = new(pos.YDirection());
        aV1.Multiply(minorRadius / majorRadius);
        gp_Vec aV = new(pos.XDirection());
        aV.Add(aV1);
        return new gp_Ax1(pos.Location(), new gp_Dir(aV));
    }

    /// <summary>The second asymptote, of direction <c>XDir - (Minor / Major) * YDir</c>.</summary>
    /// <exception cref="Standard_ConstructionError">The major radius is zero.</exception>
    public readonly gp_Ax1 Asymptote2()
    {
        // OCCT: the message names Asymptote1
        Standard_ConstructionError_Raise_if(majorRadius <= gp.Resolution(), "gp_Hypr::Asymptote1() - major radius is zero");
        gp_Vec aV1 = new(pos.YDirection());
        aV1.Multiply(-minorRadius / majorRadius);
        gp_Vec aV = new(pos.XDirection());
        aV.Add(aV1);
        return new gp_Ax1(pos.Location(), new gp_Dir(aV));
    }

    /// <summary>The main axis: the normal to the hyperbola's plane through its centre.</summary>
    public readonly gp_Ax1 Axis() => pos.Axis();

    /// <summary>The branch on the positive side of the Y axis: major and minor radii swapped.</summary>
    public readonly gp_Hypr ConjugateBranch1()
        => new(new gp_Ax2(pos.Location(), pos.Direction(), pos.YDirection()), minorRadius, majorRadius);

    /// <summary>The branch on the negative side of the Y axis: major and minor radii swapped.</summary>
    public readonly gp_Hypr ConjugateBranch2()
    {
        gp_Dir aD = pos.YDirection();
        aD.Reverse();
        return new gp_Hypr(new gp_Ax2(pos.Location(), pos.Direction(), aD), minorRadius, majorRadius);
    }

    /// <summary>The first directrix, on the positive side of the major axis, parallel to the minor axis.</summary>
    /// <exception cref="Standard_DomainError">The major radius is zero.</exception>
    public readonly gp_Ax1 Directrix1()
    {
        double anE = Eccentricity();
        gp_XYZ anOrig = pos.XDirection().XYZ();
        anOrig.Multiply(majorRadius / anE);
        anOrig.Add(pos.Location().XYZ());
        return new gp_Ax1(new gp_Pnt(anOrig), pos.YDirection());
    }

    /// <summary>The second directrix, symmetric to the first about the minor axis.</summary>
    /// <exception cref="Standard_DomainError">The major radius is zero.</exception>
    public readonly gp_Ax1 Directrix2()
    {
        double anE = Eccentricity();
        gp_XYZ anOrig = pos.XDirection().XYZ();
        anOrig.Multiply(-majorRadius / anE);
        anOrig.Add(pos.Location().XYZ());
        return new gp_Ax1(new gp_Pnt(anOrig), pos.YDirection());
    }

    /// <summary>sqrt(Major^2 + Minor^2) / Major, greater than 1.</summary>
    /// <exception cref="Standard_DomainError">The major radius is zero.</exception>
    public readonly double Eccentricity()
    {
        Standard_DomainError_Raise_if(majorRadius <= gp.Resolution(), "gp_Hypr::Eccentricity() - major radius is zero");
        return Math.Sqrt(majorRadius * majorRadius + minorRadius * minorRadius) / majorRadius;
    }

    /// <summary>The distance between the two foci.</summary>
    public readonly double Focal() => 2.0 * Math.Sqrt(majorRadius * majorRadius + minorRadius * minorRadius);

    /// <summary>The focus on the positive side of the major axis.</summary>
    public readonly gp_Pnt Focus1()
    {
        double aC = Math.Sqrt(majorRadius * majorRadius + minorRadius * minorRadius);
        gp_Pnt aPP = pos.Location();
        gp_Dir aDD = pos.XDirection();
        return new gp_Pnt(aPP.X() + aC * aDD.X(), aPP.Y() + aC * aDD.Y(), aPP.Z() + aC * aDD.Z());
    }

    /// <summary>The focus on the negative side of the major axis.</summary>
    public readonly gp_Pnt Focus2()
    {
        double aC = Math.Sqrt(majorRadius * majorRadius + minorRadius * minorRadius);
        gp_Pnt aPP = pos.Location();
        gp_Dir aDD = pos.XDirection();
        return new gp_Pnt(aPP.X() - aC * aDD.X(), aPP.Y() - aC * aDD.Y(), aPP.Z() - aC * aDD.Z());
    }

    /// <summary>The centre of the hyperbola.</summary>
    public readonly gp_Pnt Location() => pos.Location();

    public readonly double MajorRadius() => majorRadius;

    public readonly double MinorRadius() => minorRadius;

    /// <summary>The other branch, on the negative side of the major axis.</summary>
    public readonly gp_Hypr OtherBranch()
    {
        gp_Dir aD = pos.XDirection();
        aD.Reverse();
        return new gp_Hypr(new gp_Ax2(pos.Location(), pos.Direction(), aD), majorRadius, minorRadius);
    }

    /// <summary>The focal parameter, Minor^2 / Major.</summary>
    /// <exception cref="Standard_DomainError">The major radius is zero.</exception>
    public readonly double Parameter()
    {
        Standard_DomainError_Raise_if(majorRadius <= gp.Resolution(), "gp_Hypr::Parameter() - major radius is zero");
        return (minorRadius * minorRadius) / majorRadius;
    }

    public readonly gp_Ax2 Position() => pos;

    /// <summary>The major axis.</summary>
    public readonly gp_Ax1 XAxis() => new(pos.Location(), pos.XDirection());

    /// <summary>The minor axis.</summary>
    public readonly gp_Ax1 YAxis() => new(pos.Location(), pos.YDirection());

    public readonly gp_Hypr Mirrored(in gp_Pnt P)
    {
        gp_Hypr H = this;
        H.pos.Mirror(P);
        return H;
    }

    public readonly gp_Hypr Mirrored(in gp_Ax1 A1)
    {
        gp_Hypr H = this;
        H.pos.Mirror(A1);
        return H;
    }

    public readonly gp_Hypr Mirrored(in gp_Ax2 A2)
    {
        gp_Hypr H = this;
        H.pos.Mirror(A2);
        return H;
    }

    public readonly gp_Hypr Rotated(in gp_Ax1 theA1, double theAng)
    {
        gp_Hypr aH = this;
        aH.pos.Rotate(theA1, theAng);
        return aH;
    }

    public readonly gp_Hypr Scaled(in gp_Pnt theP, double theS)
    {
        gp_Hypr aH = this;
        aH.majorRadius *= theS;
        if (aH.majorRadius < 0)
        {
            aH.majorRadius = -aH.majorRadius;
        }

        aH.minorRadius *= theS;
        if (aH.minorRadius < 0)
        {
            aH.minorRadius = -aH.minorRadius;
        }

        aH.pos.Scale(theP, theS);
        return aH;
    }

    public readonly gp_Hypr Transformed(in gp_Trsf theT)
    {
        gp_Hypr aH = this;
        aH.majorRadius *= theT.ScaleFactor();
        if (aH.majorRadius < 0)
        {
            aH.majorRadius = -aH.majorRadius;
        }

        aH.minorRadius *= theT.ScaleFactor();
        if (aH.minorRadius < 0)
        {
            aH.minorRadius = -aH.minorRadius;
        }

        aH.pos.Transform(theT);
        return aH;
    }

    public readonly gp_Hypr Translated(in gp_Vec theV)
    {
        gp_Hypr aH = this;
        aH.pos.Translate(theV);
        return aH;
    }

    public readonly gp_Hypr Translated(in gp_Pnt theP1, in gp_Pnt theP2)
    {
        gp_Hypr aH = this;
        aH.pos.Translate(theP1, theP2);
        return aH;
    }

    public readonly bool Equals(gp_Hypr other)
        => pos.Equals(other.pos) && majorRadius.Equals(other.majorRadius) && minorRadius.Equals(other.minorRadius);

    public override readonly bool Equals(object? obj) => obj is gp_Hypr other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(pos, majorRadius, minorRadius);

    public static bool operator ==(in gp_Hypr theLeft, in gp_Hypr theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Hypr theLeft, in gp_Hypr theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => FormattableString.Invariant($"gp_Hypr({pos}, {majorRadius:R}, {minorRadius:R})");
}
