// Port of: src/FoundationClasses/TKMath/gp/gp_Hypr2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Hypr2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A branch of a hyperbola in 2D space, positioned by a coordinate system whose origin is the centre and
/// whose X axis is the major axis. The branch lies on the positive side of the X axis.
/// </summary>
/// <remarks>
/// <c>new gp_Hypr2d()</c> is OCCT's indefinite hyperbola (both radii <see cref="Standard.RealLast"/>);
/// <c>default(gp_Hypr2d)</c> is not valid.
/// </remarks>
public partial struct gp_Hypr2d : IEquatable<gp_Hypr2d>
{
    private gp_Ax22d pos;
    private double majorRadius;
    private double minorRadius;

    public gp_Hypr2d()
    {
        pos = new gp_Ax22d();
        majorRadius = RealLast();
        minorRadius = RealLast();
    }

    /// <summary>A hyperbola whose major axis is <paramref name="theMajorAxis"/>, direct when <paramref name="theIsSense"/> is true.</summary>
    /// <exception cref="Standard_ConstructionError">A radius is negative.</exception>
    public gp_Hypr2d(in gp_Ax2d theMajorAxis, double theMajorRadius, double theMinorRadius, bool theIsSense = true)
    {
        pos = new gp_Ax22d(theMajorAxis, theIsSense);
        majorRadius = theMajorRadius;
        minorRadius = theMinorRadius;
        Standard_ConstructionError_Raise_if(theMinorRadius < 0.0 || theMajorRadius < 0.0, "gp_Hypr2d() - invalid construction parameters");
    }

    /// <exception cref="Standard_ConstructionError">A radius is negative.</exception>
    public gp_Hypr2d(in gp_Ax22d theA, double theMajorRadius, double theMinorRadius)
    {
        pos = theA;
        majorRadius = theMajorRadius;
        minorRadius = theMinorRadius;
        Standard_ConstructionError_Raise_if(theMinorRadius < 0.0 || theMajorRadius < 0.0, "gp_Hypr2d() - invalid construction parameters");
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetLocation(in gp_Pnt2d theP) => pos.SetLocation(theP);

    internal void SetMajorRadius(double theMajorRadius)
    {
        Standard_ConstructionError_Raise_if(theMajorRadius < 0.0, "gp_Hypr2d::SetMajorRadius() - major radius should be greater or equal zero");
        majorRadius = theMajorRadius;
    }

    internal void SetMinorRadius(double theMinorRadius)
    {
        Standard_ConstructionError_Raise_if(theMinorRadius < 0.0, "gp_Hypr2d::SetMinorRadius() - minor radius should be greater or equal zero");
        minorRadius = theMinorRadius;
    }

    internal void SetAxis(in gp_Ax22d theA) => pos.SetAxis(theA);

    internal void SetXAxis(in gp_Ax2d theA) => pos.SetXAxis(theA);

    internal void SetYAxis(in gp_Ax2d theA) => pos.SetYAxis(theA);

    internal void Reverse()
    {
        gp_Dir2d aTemp = pos.YDirection();
        aTemp.Reverse();
        pos.SetAxis(new gp_Ax22d(pos.Location(), pos.XDirection(), aTemp));
    }

    internal void Mirror(in gp_Pnt2d P) => pos.Mirror(P);

    internal void Mirror(in gp_Ax2d A) => pos.Mirror(A);

    internal void Rotate(in gp_Pnt2d theP, double theAng) => pos.Rotate(theP, theAng);

    internal void Scale(in gp_Pnt2d theP, double theS)
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

    internal void Transform(in gp_Trsf2d theT)
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

    internal void Translate(in gp_Vec2d theV) => pos.Translate(theV);

    internal void Translate(in gp_Pnt2d theP1, in gp_Pnt2d theP2) => pos.Translate(theP1, theP2);

    // Copies with a mutator applied

    public readonly gp_Hypr2d WithLocation(in gp_Pnt2d theP)
    {
        gp_Hypr2d aCopy = this;
        aCopy.SetLocation(theP);
        return aCopy;
    }

    public readonly gp_Hypr2d WithMajorRadius(double theMajorRadius)
    {
        gp_Hypr2d aCopy = this;
        aCopy.SetMajorRadius(theMajorRadius);
        return aCopy;
    }

    public readonly gp_Hypr2d WithMinorRadius(double theMinorRadius)
    {
        gp_Hypr2d aCopy = this;
        aCopy.SetMinorRadius(theMinorRadius);
        return aCopy;
    }

    public readonly gp_Hypr2d WithAxis(in gp_Ax22d theA)
    {
        gp_Hypr2d aCopy = this;
        aCopy.SetAxis(theA);
        return aCopy;
    }

    public readonly gp_Hypr2d WithXAxis(in gp_Ax2d theA)
    {
        gp_Hypr2d aCopy = this;
        aCopy.SetXAxis(theA);
        return aCopy;
    }

    public readonly gp_Hypr2d WithYAxis(in gp_Ax2d theA)
    {
        gp_Hypr2d aCopy = this;
        aCopy.SetYAxis(theA);
        return aCopy;
    }

    // Queries

    /// <summary>The first asymptote, meant to have direction X + (MinorRadius / MajorRadius) * Y.</summary>
    /// <exception cref="Standard_ConstructionError">The major radius is at most <see cref="gp.Resolution"/>.</exception>
    public readonly gp_Ax2d Asymptote1()
    {
        Standard_ConstructionError_Raise_if(majorRadius <= gp.Resolution(), "gp_Hypr2d::Asymptote1() - major radius is zero");
        gp_Dir2d aVdir = pos.XDirection();
        // OCCT builds the direction from the Y direction alone (Y + (b/a) Y), where gp_Hypr adds the X direction,
        // so the result is along Y (docs/occt-issues.md); kept so results match OCCT
        gp_XY aCoord1 = pos.YDirection().XY();
        gp_XY aCoord2 = aCoord1.Multiplied(minorRadius / majorRadius);
        aCoord1.Add(aCoord2);
        aVdir.SetXY(aCoord1);
        return new gp_Ax2d(pos.Location(), aVdir);
    }

    /// <summary>The second asymptote, meant to have direction X - (MinorRadius / MajorRadius) * Y.</summary>
    /// <exception cref="Standard_ConstructionError">The major radius is at most <see cref="gp.Resolution"/>.</exception>
    public readonly gp_Ax2d Asymptote2()
    {
        Standard_ConstructionError_Raise_if(majorRadius <= gp.Resolution(), "gp_Hypr2d::Asymptote2() - major radius is zero");
        gp_Vec2d aVdir = new(pos.XDirection());
        // OCCT builds the direction from the Y direction alone (Y - (b/a) Y), where gp_Hypr adds the X direction,
        // so the result is along Y, or throws when the radii are equal (docs/occt-issues.md); kept so results match OCCT
        gp_XY aCoord1 = pos.YDirection().XY();
        gp_XY aCoord2 = aCoord1.Multiplied(-minorRadius / majorRadius);
        aCoord1.Add(aCoord2);
        aVdir.SetXY(aCoord1);
        return new gp_Ax2d(pos.Location(), new gp_Dir2d(aVdir));
    }

    /// <summary>
    /// The coefficients of the hyperbola's implicit equation
    /// <c>A * X^2 + B * Y^2 + 2 * C * X * Y + 2 * D * X + 2 * E * Y + F = 0</c>.
    /// </summary>
    public readonly void Coefficients(out double A, out double B, out double C, out double D, out double E, out double F)
    {
        double DMin = minorRadius * minorRadius;
        double DMaj = majorRadius * majorRadius;
        if (DMin <= gp.Resolution() && DMaj <= gp.Resolution())
        {
            A = B = C = D = E = F = 0.0;
        }
        else
        {
            gp_Trsf2d T = new();
            T.SetTransformation(pos.XAxis());
            double T11 = T.Value(1, 1);
            double T12 = T.Value(1, 2);
            double T13 = T.Value(1, 3);
            if (DMin <= gp.Resolution())
            {
                A = T11 * T11;
                B = T12 * T12;
                C = T11 * T12;
                D = T11 * T13;
                E = T12 * T13;
                F = T13 * T13 - DMaj;
            }
            else
            {
                double T21 = T.Value(2, 1);
                double T22 = T.Value(2, 2);
                double T23 = T.Value(2, 3);
                A = (T11 * T11 / DMaj) - (T21 * T21 / DMin);
                B = (T12 * T12 / DMaj) - (T22 * T22 / DMin);
                C = (T11 * T12 / DMaj) - (T21 * T22 / DMin);
                D = (T11 * T13 / DMaj) - (T21 * T23 / DMin);
                E = (T12 * T13 / DMaj) - (T22 * T23 / DMin);
                F = (T13 * T13 / DMaj) - (T23 * T23 / DMin) - 1.0;
            }
        }
    }

    /// <summary>The conjugate branch on the positive side of the Y axis.</summary>
    public readonly gp_Hypr2d ConjugateBranch1()
    {
        gp_Dir2d aV = pos.YDirection();
        bool isSign = (pos.XDirection().Crossed(pos.YDirection())) >= 0.0;
        return new gp_Hypr2d(new gp_Ax2d(pos.Location(), aV), minorRadius, majorRadius, isSign);
    }

    /// <summary>The conjugate branch on the negative side of the Y axis.</summary>
    public readonly gp_Hypr2d ConjugateBranch2()
    {
        gp_Dir2d aV = pos.YDirection().Reversed();
        bool isSign = (pos.XDirection().Crossed(pos.YDirection())) >= 0.0;
        return new gp_Hypr2d(new gp_Ax2d(pos.Location(), aV), minorRadius, majorRadius, isSign);
    }

    /// <summary>The directrix on the positive side of the major axis, at distance MajorRadius / e from the centre.</summary>
    /// <exception cref="Standard_DomainError">The major radius is at most <see cref="gp.Resolution"/>.</exception>
    public readonly gp_Ax2d Directrix1()
    {
        double anE = Eccentricity();
        gp_XY anOrig = pos.XDirection().XY();
        anOrig.Multiply(majorRadius / anE);
        anOrig.Add(pos.Location().XY());
        return new gp_Ax2d(new gp_Pnt2d(anOrig), pos.YDirection());
    }

    /// <summary>The second directrix.</summary>
    /// <exception cref="Standard_DomainError">The major radius is at most <see cref="gp.Resolution"/>.</exception>
    public readonly gp_Ax2d Directrix2()
    {
        double anE = Eccentricity();
        gp_XY anOrig = pos.XDirection().XY();
        // OCCT places it at Focus1 + (Parameter / e) X, where gp_Hypr uses Location - (MajorRadius / e) X
        // (docs/occt-issues.md); kept so results match OCCT
        anOrig.Multiply(Parameter() / anE);
        anOrig.Add(Focus1().XY());
        return new gp_Ax2d(new gp_Pnt2d(anOrig), pos.YDirection());
    }

    /// <exception cref="Standard_DomainError">The major radius is at most <see cref="gp.Resolution"/>.</exception>
    public readonly double Eccentricity()
    {
        Standard_DomainError_Raise_if(majorRadius <= gp.Resolution(), "gp_Hypr2d::Eccentricity() - major radius is zero");
        return Math.Sqrt(majorRadius * majorRadius + minorRadius * minorRadius) / majorRadius;
    }

    /// <summary>The distance between the foci.</summary>
    public readonly double Focal() => 2.0 * Math.Sqrt(majorRadius * majorRadius + minorRadius * minorRadius);

    public readonly gp_Pnt2d Focus1()
    {
        double aC = Math.Sqrt(majorRadius * majorRadius + minorRadius * minorRadius);
        return new gp_Pnt2d(pos.Location().X() + aC * pos.XDirection().X(),
                            pos.Location().Y() + aC * pos.XDirection().Y());
    }

    public readonly gp_Pnt2d Focus2()
    {
        double aC = Math.Sqrt(majorRadius * majorRadius + minorRadius * minorRadius);
        return new gp_Pnt2d(pos.Location().X() - aC * pos.XDirection().X(),
                            pos.Location().Y() - aC * pos.XDirection().Y());
    }

    public readonly gp_Pnt2d Location() => pos.Location();

    public readonly double MajorRadius() => majorRadius;

    public readonly double MinorRadius() => minorRadius;

    /// <summary>The branch on the negative side of the X axis.</summary>
    public readonly gp_Hypr2d OtherBranch()
    {
        bool isSign = (pos.XDirection().Crossed(pos.YDirection())) >= 0.0;
        return new gp_Hypr2d(new gp_Ax2d(pos.Location(), pos.XDirection().Reversed()), majorRadius, minorRadius, isSign);
    }

    /// <summary>The parameter (semi-latus rectum) of the hyperbola: MinorRadius^2 / MajorRadius.</summary>
    /// <exception cref="Standard_DomainError">The major radius is at most <see cref="gp.Resolution"/>.</exception>
    public readonly double Parameter()
    {
        Standard_DomainError_Raise_if(majorRadius <= gp.Resolution(), "gp_Hypr2d::Parameter() - major radius is zero");
        return (minorRadius * minorRadius) / majorRadius;
    }

    public readonly gp_Ax22d Axis() => pos;

    public readonly gp_Ax2d XAxis() => pos.XAxis();

    public readonly gp_Ax2d YAxis() => pos.YAxis();

    /// <summary>True when the hyperbola's system is direct (counterclockwise).</summary>
    public readonly bool IsDirect() => (pos.XDirection().Crossed(pos.YDirection())) >= 0.0;

    public readonly gp_Hypr2d Reversed()
    {
        gp_Hypr2d aH = this;
        gp_Dir2d aTemp = pos.YDirection();
        aTemp.Reverse();
        aH.pos.SetAxis(new gp_Ax22d(pos.Location(), pos.XDirection(), aTemp));
        return aH;
    }

    public readonly gp_Hypr2d Mirrored(in gp_Pnt2d P)
    {
        gp_Hypr2d H = this;
        H.pos.Mirror(P);
        return H;
    }

    public readonly gp_Hypr2d Mirrored(in gp_Ax2d A)
    {
        gp_Hypr2d H = this;
        H.pos.Mirror(A);
        return H;
    }

    public readonly gp_Hypr2d Rotated(in gp_Pnt2d theP, double theAng)
    {
        gp_Hypr2d aH = this;
        aH.pos.Rotate(theP, theAng);
        return aH;
    }

    public readonly gp_Hypr2d Scaled(in gp_Pnt2d theP, double theS)
    {
        gp_Hypr2d aH = this;
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

    public readonly gp_Hypr2d Transformed(in gp_Trsf2d theT)
    {
        gp_Hypr2d aH = this;
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

    public readonly gp_Hypr2d Translated(in gp_Vec2d theV)
    {
        gp_Hypr2d aH = this;
        aH.pos.Translate(theV);
        return aH;
    }

    public readonly gp_Hypr2d Translated(in gp_Pnt2d theP1, in gp_Pnt2d theP2)
    {
        gp_Hypr2d aH = this;
        aH.pos.Translate(theP1, theP2);
        return aH;
    }

    public readonly bool Equals(gp_Hypr2d other)
        => pos.Equals(other.pos) && majorRadius.Equals(other.majorRadius) && minorRadius.Equals(other.minorRadius);

    public override readonly bool Equals(object? obj) => obj is gp_Hypr2d other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(pos, majorRadius, minorRadius);

    public static bool operator ==(in gp_Hypr2d theLeft, in gp_Hypr2d theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Hypr2d theLeft, in gp_Hypr2d theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"gp_Hypr2d({pos}, major {majorRadius}, minor {minorRadius})";
}
