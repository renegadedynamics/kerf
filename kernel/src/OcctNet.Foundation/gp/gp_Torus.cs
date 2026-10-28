// Port of: src/FoundationClasses/TKMath/gp/gp_Torus.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Torus.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A torus, defined by its major and minor radii and positioned by a coordinate system whose origin is the
/// centre: the surface is swept by a circle of the minor radius, in the plane (origin, X direction, main
/// direction) and centred on the X axis at the major radius, rotating about the main direction.
/// </summary>
/// <remarks>
/// <c>new gp_Torus()</c> is OCCT's indefinite torus (major radius <see cref="Standard.RealLast"/>, minor radius
/// <see cref="Standard.RealSmall"/>); <c>default(gp_Torus)</c> is not valid.
/// </remarks>
public partial struct gp_Torus : IEquatable<gp_Torus>
{
    private gp_Ax3 pos;
    private double majorRadius;
    private double minorRadius;

    public gp_Torus()
    {
        pos = new gp_Ax3();
        majorRadius = RealLast();
        minorRadius = RealSmall();
    }

    /// <remarks>A torus with both radii 0 is allowed.</remarks>
    /// <exception cref="Standard_ConstructionError">A radius is negative.</exception>
    public gp_Torus(in gp_Ax3 theA3, double theMajorRadius, double theMinorRadius)
    {
        pos = theA3;
        majorRadius = theMajorRadius;
        minorRadius = theMinorRadius;
        Standard_ConstructionError_Raise_if(theMinorRadius < 0.0 || theMajorRadius < 0.0,
                                            "gp_Torus() - invalid construction parameters");
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetAxis(in gp_Ax1 theA1) => pos.SetAxis(theA1);

    internal void SetLocation(in gp_Pnt theLoc) => pos.SetLocation(theLoc);

    internal void SetMajorRadius(double theMajorRadius)
    {
        Standard_ConstructionError_Raise_if(theMajorRadius - minorRadius <= gp.Resolution(),
                                            "gp_Torus::SetMajorRadius() - invalid input parameters");
        majorRadius = theMajorRadius;
    }

    internal void SetMinorRadius(double theMinorRadius)
    {
        Standard_ConstructionError_Raise_if(theMinorRadius < 0.0
                                            || majorRadius - theMinorRadius <= gp.Resolution(),
                                            "gp_Torus::SetMinorRadius() - invalid input parameters");
        minorRadius = theMinorRadius;
    }

    internal void SetPosition(in gp_Ax3 theA3) => pos = theA3;

    /// <summary>Reverses the U parametrisation (the Y direction).</summary>
    internal void UReverse() => pos.YReverse();

    /// <summary>Reverses the V parametrisation (the main direction).</summary>
    internal void VReverse() => pos.ZReverse();

    internal void Mirror(in gp_Pnt P) => pos.Mirror(P);

    internal void Mirror(in gp_Ax1 A1) => pos.Mirror(A1);

    internal void Mirror(in gp_Ax2 A2) => pos.Mirror(A2);

    internal void Rotate(in gp_Ax1 theA1, double theAng) => pos.Rotate(theA1, theAng);

    internal void Scale(in gp_Pnt theP, double theS)
    {
        pos.Scale(theP, theS);
        double s = theS;
        if (s < 0)
        {
            s = -s;
        }

        majorRadius *= s;
        minorRadius *= s;
    }

    internal void Transform(in gp_Trsf theT)
    {
        pos.Transform(theT);
        double aT = theT.ScaleFactor();
        if (aT < 0)
        {
            aT = -aT;
        }

        minorRadius *= aT;
        majorRadius *= aT;
    }

    internal void Translate(in gp_Vec theV) => pos.Translate(theV);

    internal void Translate(in gp_Pnt theP1, in gp_Pnt theP2) => pos.Translate(theP1, theP2);

    // Copies with a mutator applied

    public readonly gp_Torus WithAxis(in gp_Ax1 theA1)
    {
        gp_Torus aCopy = this;
        aCopy.SetAxis(theA1);
        return aCopy;
    }

    public readonly gp_Torus WithLocation(in gp_Pnt theLoc)
    {
        gp_Torus aCopy = this;
        aCopy.SetLocation(theLoc);
        return aCopy;
    }

    public readonly gp_Torus WithMajorRadius(double theMajorRadius)
    {
        gp_Torus aCopy = this;
        aCopy.SetMajorRadius(theMajorRadius);
        return aCopy;
    }

    public readonly gp_Torus WithMinorRadius(double theMinorRadius)
    {
        gp_Torus aCopy = this;
        aCopy.SetMinorRadius(theMinorRadius);
        return aCopy;
    }

    public readonly gp_Torus WithPosition(in gp_Ax3 theA3)
    {
        gp_Torus aCopy = this;
        aCopy.SetPosition(theA3);
        return aCopy;
    }

    public readonly gp_Torus UReversed()
    {
        gp_Torus aCopy = this;
        aCopy.UReverse();
        return aCopy;
    }

    public readonly gp_Torus VReversed()
    {
        gp_Torus aCopy = this;
        aCopy.VReverse();
        return aCopy;
    }

    // Queries

    public readonly double Area() => 4.0 * M_PI * M_PI * minorRadius * majorRadius;

    /// <summary>True when the coordinate system is right-handed.</summary>
    public readonly bool Direct() => pos.Direct();

    /// <summary>The symmetry axis.</summary>
    public readonly gp_Ax1 Axis() => pos.Axis();

    /// <summary>
    /// The 35 coefficients of the implicit quartic equation in the global system, in OCCT's order:
    /// X^4, Y^4, Z^4, X^3.Y, X^3.Z, Y^3.X, Y^3.Z, Z^3.X, Z^3.Y, X^2.Y^2, X^2.Z^2, Y^2.Z^2, X^2.Y.Z, X.Y^2.Z,
    /// X.Y.Z^2, X^3, Y^3, Z^3, X^2.Y, X^2.Z, Y^2.X, Y^2.Z, Z^2.X, Z^2.Y, X.Y.Z, X^2, Y^2, Z^2, X.Y, X.Z, Y.Z,
    /// X, Y, Z, 1.
    /// </summary>
    /// <remarks>
    /// OCCT fills an <c>NCollection_Array1</c> from its lower bound; the port fills a span from index 0, so
    /// OCCT's <c>Coef(Lower() + i)</c> is <c>theCoef[i]</c>.
    /// </remarks>
    /// <exception cref="Standard_DimensionError"><paramref name="theCoef"/> has fewer than 35 elements.</exception>
    public readonly void Coefficients(Span<double> theCoef)
    {
        //  R = majorRadius;
        //  r = minorRadius.
        //  X = (R + r*cos(V))*cos(U)
        //  Y = (R + r*cos(V))*sin(U)
        //  Z = r*sin(V)
        // Therefore,
        //   4*R*R*(r*r - Z*Z) = (X*X + Y*Y + Z*Z - R*R - r*r)^2
        // Or
        //   X^4+Y^4+Z^4+
        //   2*((X*Y)^2+(X*Z)^2+(Y*Z)^2)-
        //   2*(R^2+r^2)*(X^2+Y^2)+
        //   2*(R^2-r^2)*Z^2+(R^2-r^2)^2 = 0.0
        const int aLowIndex = 0;
        Standard_DimensionError_Raise_if(theCoef.Length < 35, "gp_Torus::theCoefficients(): Dimension mismatch");

        gp_Trsf aTr = new();
        aTr.SetTransformation(pos);
        double aT11 = aTr.Value(1, 1);
        double aT12 = aTr.Value(1, 2);
        double aT13 = aTr.Value(1, 3);
        double aT14 = aTr.Value(1, 4);
        double aT21 = aTr.Value(2, 1);
        double aT22 = aTr.Value(2, 2);
        double aT23 = aTr.Value(2, 3);
        double aT24 = aTr.Value(2, 4);
        double aT31 = aTr.Value(3, 1);
        double aT32 = aTr.Value(3, 2);
        double aT33 = aTr.Value(3, 3);
        double aT34 = aTr.Value(3, 4);

        double aTcol1sq = aT11 * aT11 + aT21 * aT21 + aT31 * aT31;
        double aTcol2sq = aT12 * aT12 + aT22 * aT22 + aT32 * aT32;
        double aTcol3sq = aT13 * aT13 + aT23 * aT23 + aT33 * aT33;
        double aTcol4sq = aT14 * aT14 + aT24 * aT24 + aT34 * aT34;
        double aTcol1Tcol2 = aT11 * aT12 + aT21 * aT22 + aT31 * aT32;
        double aTcol1Tcol3 = aT11 * aT13 + aT21 * aT23 + aT31 * aT33;
        double aTcol2Tcol3 = aT12 * aT13 + aT22 * aT23 + aT32 * aT33;
        double aTcol1Tcol4 = aT11 * aT14 + aT21 * aT24 + aT31 * aT34;
        double aTcol2Tcol4 = aT12 * aT14 + aT22 * aT24 + aT32 * aT34;
        double aTcol3Tcol4 = aT13 * aT14 + aT23 * aT24 + aT33 * aT34;

        double aSumRadius = (majorRadius * majorRadius + minorRadius * minorRadius);
        double aSubRadius = (majorRadius * majorRadius - minorRadius * minorRadius);

        // After substitution
        // Transpose([X Y Z 1]) = aTr*Transpose([X Y Z 1])
        // we will obtain:
        theCoef[aLowIndex] = aTcol1sq * aTcol1sq; // X^4
        theCoef[aLowIndex + 1] = aTcol2sq * aTcol2sq; // Y^4
        theCoef[aLowIndex + 2] = aTcol3sq * aTcol3sq; // Z^4
        theCoef[aLowIndex + 3] = 4.0 * aTcol1sq * aTcol1Tcol2; // X^3*Y
        theCoef[aLowIndex + 4] = 4.0 * aTcol1sq * aTcol1Tcol3; // X^3*Z
        theCoef[aLowIndex + 5] = 4.0 * aTcol2sq * aTcol1Tcol2; // X*Y^3
        theCoef[aLowIndex + 6] = 4.0 * aTcol2sq * aTcol2Tcol3; // Y^3*Z
        theCoef[aLowIndex + 7] = 4.0 * aTcol3sq * aTcol1Tcol3; // X*Z^3
        theCoef[aLowIndex + 8] = 4.0 * aTcol3sq * aTcol2Tcol3; // Y*Z^3
        theCoef[aLowIndex + 9] = 2.0 * (aTcol1sq * aTcol2sq + 2.0 * aTcol1Tcol2 * aTcol1Tcol2); // X^2*Y^2
        theCoef[aLowIndex + 10] =
            2.0 * (aTcol1sq * aTcol3sq + 2.0 * aTcol1Tcol3 * aTcol1Tcol3); // X^2*Z^2
        theCoef[aLowIndex + 11] =
            2.0 * (aTcol2sq * aTcol3sq + 2.0 * aTcol2Tcol3 * aTcol2Tcol3); // Y^2*Z^2
        theCoef[aLowIndex + 12] =
            4.0 * (aTcol1sq * aTcol2Tcol3 + 2.0 * aTcol1Tcol2 * aTcol1Tcol3); // X^2*Y*Z
        theCoef[aLowIndex + 13] =
            4.0 * (aTcol2sq * aTcol1Tcol3 + 2.0 * aTcol1Tcol2 * aTcol2Tcol3); // X*Y^2*Z
        theCoef[aLowIndex + 14] =
            4.0 * (aTcol3sq * aTcol1Tcol2 + 2.0 * aTcol1Tcol3 * aTcol2Tcol3); // X*Y*Z^2

        theCoef[aLowIndex + 15] = 4.0 * aTcol1sq * aTcol1Tcol4; // X^3
        theCoef[aLowIndex + 16] = 4.0 * aTcol2sq * aTcol2Tcol4; // Y^3
        theCoef[aLowIndex + 17] = 4.0 * aTcol3sq * aTcol3Tcol4; // Z^3
        theCoef[aLowIndex + 18] =
            4.0 * (aTcol1sq * aTcol2Tcol4 + 2.0 * aTcol1Tcol4 * aTcol1Tcol2); // X^2*Y
        theCoef[aLowIndex + 19] =
            4.0 * (aTcol1sq * aTcol3Tcol4 + 2.0 * aTcol1Tcol4 * aTcol1Tcol3); // X^2*Z
        theCoef[aLowIndex + 20] =
            4.0 * (aTcol2sq * aTcol1Tcol4 + 2.0 * aTcol2Tcol4 * aTcol1Tcol2); // X*Y^2
        theCoef[aLowIndex + 21] =
            4.0 * (aTcol2sq * aTcol3Tcol4 + 2.0 * aTcol2Tcol4 * aTcol2Tcol3); // Y^2*Z
        theCoef[aLowIndex + 22] =
            4.0 * (aTcol3sq * aTcol1Tcol4 + 2.0 * aTcol3Tcol4 * aTcol1Tcol3); // X*Z^2
        theCoef[aLowIndex + 23] =
            4.0 * (aTcol3sq * aTcol2Tcol4 + 2.0 * aTcol3Tcol4 * aTcol2Tcol3); // Y*Z^2
        theCoef[aLowIndex + 24] =
            8.0
            * (aTcol1Tcol2 * aTcol3Tcol4 + aTcol2Tcol3 * aTcol1Tcol4 + aTcol2Tcol4 * aTcol1Tcol3); // X*Y*Z

        theCoef[aLowIndex + 25] = 2.0
                                  * (aSubRadius * aT31 * aT31 - aSumRadius * (aT11 * aT11 + aT21 * aT21)
                                     + aTcol4sq * aTcol1sq + 2.0 * aTcol1Tcol4 * aTcol1Tcol4); // X^2
        theCoef[aLowIndex + 26] = 2.0
                                  * (aSubRadius * aT32 * aT32 - aSumRadius * (aT12 * aT12 + aT22 * aT22)
                                     + aTcol4sq * aTcol2sq + 2.0 * aTcol2Tcol4 * aTcol2Tcol4); // Y^2
        theCoef[aLowIndex + 27] = 2.0
                                  * (aSubRadius * aT33 * aT33 - aSumRadius * (aT13 * aT13 + aT23 * aT23)
                                     + aTcol4sq * aTcol3sq + 2.0 * aTcol3Tcol4 * aTcol3Tcol4); // Z^2
        theCoef[aLowIndex + 28] = 4.0
                                  * (aSubRadius * aT31 * aT32 - aSumRadius * (aT11 * aT12 + aT21 * aT22)
                                     + aTcol4sq * aTcol1Tcol2 + 2.0 * aTcol1Tcol4 * aTcol2Tcol4); // X*Y
        theCoef[aLowIndex + 29] = 4.0
                                  * (aSubRadius * aT31 * aT33 - aSumRadius * (aT11 * aT13 + aT21 * aT23)
                                     + aTcol4sq * aTcol1Tcol3 + 2.0 * aTcol1Tcol4 * aTcol3Tcol4); // X*Z
        theCoef[aLowIndex + 30] = 4.0
                                  * (aSubRadius * aT32 * aT33 - aSumRadius * (aT12 * aT13 + aT22 * aT23)
                                     + aTcol4sq * aTcol2Tcol3 + 2.0 * aTcol2Tcol4 * aTcol3Tcol4); // Y*Z

        theCoef[aLowIndex + 31] = 4.0
                                  * (aTcol4sq * aTcol1Tcol4 + aSubRadius * aT31 * aT34
                                     - aSumRadius * (aT11 * aT14 + aT21 * aT24)); // X
        theCoef[aLowIndex + 32] = 4.0
                                  * (aTcol4sq * aTcol2Tcol4 + aSubRadius * aT32 * aT34
                                     - aSumRadius * (aT12 * aT14 + aT22 * aT24)); // Y
        theCoef[aLowIndex + 33] = 4.0
                                  * (aTcol4sq * aTcol3Tcol4 + aSubRadius * aT33 * aT34
                                     - aSumRadius * (aT13 * aT14 + aT23 * aT24)); // Z;

        theCoef[aLowIndex + 34] = 2.0 * aSubRadius * aT34 * aT34
                                  - 2.0 * aSumRadius * (aT14 * aT14 + aT24 * aT24) + aTcol4sq * aTcol4sq
                                  + aSubRadius * aSubRadius;
    }

    public readonly gp_Pnt Location() => pos.Location();

    public readonly gp_Ax3 Position() => pos;

    public readonly double MajorRadius() => majorRadius;

    public readonly double MinorRadius() => minorRadius;

    public readonly double Volume() => (M_PI * minorRadius * minorRadius) * (2.0 * M_PI * majorRadius);

    public readonly gp_Ax1 XAxis() => new(pos.Location(), pos.XDirection());

    public readonly gp_Ax1 YAxis() => new(pos.Location(), pos.YDirection());

    public readonly gp_Torus Mirrored(in gp_Pnt P)
    {
        gp_Torus C = this;
        C.pos.Mirror(P);
        return C;
    }

    public readonly gp_Torus Mirrored(in gp_Ax1 A1)
    {
        gp_Torus C = this;
        C.pos.Mirror(A1);
        return C;
    }

    public readonly gp_Torus Mirrored(in gp_Ax2 A2)
    {
        gp_Torus C = this;
        C.pos.Mirror(A2);
        return C;
    }

    public readonly gp_Torus Rotated(in gp_Ax1 theA1, double theAng)
    {
        gp_Torus aC = this;
        aC.pos.Rotate(theA1, theAng);
        return aC;
    }

    /// <summary>The torus scaled by |<paramref name="theS"/>| about <paramref name="theP"/>.</summary>
    public readonly gp_Torus Scaled(in gp_Pnt theP, double theS)
    {
        gp_Torus aC = this;
        aC.pos.Scale(theP, theS);
        aC.majorRadius *= theS;
        if (aC.majorRadius < 0)
        {
            aC.majorRadius = -aC.majorRadius;
        }

        aC.minorRadius *= theS;
        if (aC.minorRadius < 0)
        {
            aC.minorRadius = -aC.minorRadius;
        }

        return aC;
    }

    public readonly gp_Torus Transformed(in gp_Trsf theT)
    {
        gp_Torus aC = this;
        aC.pos.Transform(theT);
        aC.majorRadius *= theT.ScaleFactor();
        if (aC.majorRadius < 0)
        {
            aC.majorRadius = -aC.majorRadius;
        }

        aC.minorRadius *= theT.ScaleFactor();
        if (aC.minorRadius < 0)
        {
            aC.minorRadius = -aC.minorRadius;
        }

        return aC;
    }

    public readonly gp_Torus Translated(in gp_Vec theV)
    {
        gp_Torus aC = this;
        aC.pos.Translate(theV);
        return aC;
    }

    public readonly gp_Torus Translated(in gp_Pnt theP1, in gp_Pnt theP2)
    {
        gp_Torus aC = this;
        aC.pos.Translate(theP1, theP2);
        return aC;
    }

    public readonly bool Equals(gp_Torus other)
        => pos.Equals(other.pos) && majorRadius.Equals(other.majorRadius) && minorRadius.Equals(other.minorRadius);

    public override readonly bool Equals(object? obj) => obj is gp_Torus other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(pos, majorRadius, minorRadius);

    public static bool operator ==(in gp_Torus theLeft, in gp_Torus theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Torus theLeft, in gp_Torus theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => FormattableString.Invariant($"gp_Torus({pos}, {majorRadius:R}, {minorRadius:R})");
}
