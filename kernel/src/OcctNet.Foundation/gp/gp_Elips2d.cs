// Port of: src/FoundationClasses/TKMath/gp/gp_Elips2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Elips2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// An ellipse in 2D space, positioned by a coordinate system whose origin is the centre and whose X axis is
/// the major axis. It is parametrized from the X axis, counterclockwise when the system is direct.
/// </summary>
/// <remarks>
/// <c>new gp_Elips2d()</c> is OCCT's indefinite ellipse (major radius <see cref="Standard.RealLast"/>, minor
/// radius <see cref="Standard.RealSmall"/>); <c>default(gp_Elips2d)</c> is not valid.
/// </remarks>
public partial struct gp_Elips2d : IEquatable<gp_Elips2d>
{
    private gp_Ax22d pos;
    private double majorRadius;
    private double minorRadius;

    public gp_Elips2d()
    {
        pos = new gp_Ax22d();
        majorRadius = RealLast();
        minorRadius = RealSmall();
    }

    /// <summary>An ellipse whose major axis is <paramref name="theMajorAxis"/>, direct when <paramref name="theIsSense"/> is true.</summary>
    /// <exception cref="Standard_ConstructionError">The minor radius is negative or greater than the major radius.</exception>
    public gp_Elips2d(in gp_Ax2d theMajorAxis, double theMajorRadius, double theMinorRadius, bool theIsSense = true)
    {
        pos = new gp_Ax22d(theMajorAxis, theIsSense);
        majorRadius = theMajorRadius;
        minorRadius = theMinorRadius;
        Standard_ConstructionError_Raise_if(theMinorRadius < 0.0 || theMajorRadius < theMinorRadius, "gp_Elips2d() - invalid construction parameters");
    }

    /// <exception cref="Standard_ConstructionError">The minor radius is negative or greater than the major radius.</exception>
    public gp_Elips2d(in gp_Ax22d theA, double theMajorRadius, double theMinorRadius)
    {
        pos = theA;
        majorRadius = theMajorRadius;
        minorRadius = theMinorRadius;
        Standard_ConstructionError_Raise_if(theMinorRadius < 0.0 || theMajorRadius < theMinorRadius, "gp_Elips2d() - invalid construction parameters");
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetLocation(in gp_Pnt2d theP) => pos.SetLocation(theP);

    internal void SetMajorRadius(double theMajorRadius)
    {
        Standard_ConstructionError_Raise_if(theMajorRadius < minorRadius,
                                            "gp_Elips2d::SetMajorRadius() - major radius should be greater or equal to minor radius");
        majorRadius = theMajorRadius;
    }

    internal void SetMinorRadius(double theMinorRadius)
    {
        Standard_ConstructionError_Raise_if(theMinorRadius < 0.0 || majorRadius < theMinorRadius,
                                            "gp_Elips2d::SetMinorRadius() - minor radius should be a "
                                            + "positive number lesser or equal to major radius");
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
        double aTSca = theT.ScaleFactor();
        if (aTSca < 0.0)
        {
            aTSca = -aTSca;
        }

        majorRadius *= aTSca;
        minorRadius *= aTSca;
        pos.Transform(theT);
    }

    internal void Translate(in gp_Vec2d theV) => pos.Translate(theV);

    internal void Translate(in gp_Pnt2d theP1, in gp_Pnt2d theP2) => pos.Translate(theP1, theP2);

    // Copies with a mutator applied

    public readonly gp_Elips2d WithLocation(in gp_Pnt2d theP)
    {
        gp_Elips2d aCopy = this;
        aCopy.SetLocation(theP);
        return aCopy;
    }

    public readonly gp_Elips2d WithMajorRadius(double theMajorRadius)
    {
        gp_Elips2d aCopy = this;
        aCopy.SetMajorRadius(theMajorRadius);
        return aCopy;
    }

    public readonly gp_Elips2d WithMinorRadius(double theMinorRadius)
    {
        gp_Elips2d aCopy = this;
        aCopy.SetMinorRadius(theMinorRadius);
        return aCopy;
    }

    public readonly gp_Elips2d WithAxis(in gp_Ax22d theA)
    {
        gp_Elips2d aCopy = this;
        aCopy.SetAxis(theA);
        return aCopy;
    }

    public readonly gp_Elips2d WithXAxis(in gp_Ax2d theA)
    {
        gp_Elips2d aCopy = this;
        aCopy.SetXAxis(theA);
        return aCopy;
    }

    public readonly gp_Elips2d WithYAxis(in gp_Ax2d theA)
    {
        gp_Elips2d aCopy = this;
        aCopy.SetYAxis(theA);
        return aCopy;
    }

    // Queries

    public readonly double Area() => M_PI * majorRadius * minorRadius;

    /// <summary>
    /// The coefficients of the ellipse's implicit equation
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
                A = (T11 * T11 / DMaj) + (T21 * T21 / DMin);
                B = (T12 * T12 / DMaj) + (T22 * T22 / DMin);
                C = (T11 * T12 / DMaj) + (T21 * T22 / DMin);
                D = (T11 * T13 / DMaj) + (T21 * T23 / DMin);
                E = (T12 * T13 / DMaj) + (T22 * T23 / DMin);
                F = (T13 * T13 / DMaj) + (T23 * T23 / DMin) - 1.0;
            }
        }
    }

    /// <summary>The directrix on the positive side of the major axis, at distance MajorRadius / e from the centre.</summary>
    /// <exception cref="Standard_ConstructionError">The eccentricity is zero (a circle).</exception>
    public readonly gp_Ax2d Directrix1()
    {
        double anE = Eccentricity();
        Standard_ConstructionError_Raise_if(anE <= gp.Resolution(), "gp_Elips2d::Directrix1() - zero eccentricity");
        gp_XY anOrig = pos.XDirection().XY();
        anOrig.Multiply(majorRadius / anE);
        anOrig.Add(pos.Location().XY());
        return new gp_Ax2d(new gp_Pnt2d(anOrig), pos.YDirection());
    }

    /// <summary>The directrix on the negative side of the major axis.</summary>
    /// <exception cref="Standard_ConstructionError">The eccentricity is zero (a circle).</exception>
    public readonly gp_Ax2d Directrix2()
    {
        double anE = Eccentricity();
        Standard_ConstructionError_Raise_if(anE <= gp.Resolution(), "gp_Elips2d::Directrix2() - zero eccentricity");
        gp_XY anOrig = pos.XDirection().XY();
        anOrig.Multiply(-majorRadius / anE);
        anOrig.Add(pos.Location().XY());
        return new gp_Ax2d(new gp_Pnt2d(anOrig), pos.YDirection());
    }

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

    /// <summary>The distance between the foci.</summary>
    public readonly double Focal() => 2.0 * Math.Sqrt(majorRadius * majorRadius - minorRadius * minorRadius);

    public readonly gp_Pnt2d Focus1()
    {
        double aC = Math.Sqrt(majorRadius * majorRadius - minorRadius * minorRadius);
        gp_Pnt2d aPP = pos.Location();
        gp_Dir2d aDD = pos.XDirection();
        return new gp_Pnt2d(aPP.X() + aC * aDD.X(), aPP.Y() + aC * aDD.Y());
    }

    public readonly gp_Pnt2d Focus2()
    {
        double aC = Math.Sqrt(majorRadius * majorRadius - minorRadius * minorRadius);
        gp_Pnt2d aPP = pos.Location();
        gp_Dir2d aDD = pos.XDirection();
        return new gp_Pnt2d(aPP.X() - aC * aDD.X(), aPP.Y() - aC * aDD.Y());
    }

    public readonly gp_Pnt2d Location() => pos.Location();

    public readonly double MajorRadius() => majorRadius;

    public readonly double MinorRadius() => minorRadius;

    /// <summary>The parameter (semi-latus rectum) of the ellipse: MinorRadius^2 / MajorRadius.</summary>
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

    public readonly gp_Ax22d Axis() => pos;

    public readonly gp_Ax2d XAxis() => pos.XAxis();

    public readonly gp_Ax2d YAxis() => pos.YAxis();

    /// <summary>True when the ellipse's system is direct (counterclockwise).</summary>
    public readonly bool IsDirect() => (pos.XDirection().Crossed(pos.YDirection())) >= 0.0;

    public readonly gp_Elips2d Reversed()
    {
        gp_Elips2d anE = this;
        gp_Dir2d aTemp = pos.YDirection();
        aTemp.Reverse();
        anE.pos.SetAxis(new gp_Ax22d(pos.Location(), pos.XDirection(), aTemp));
        return anE;
    }

    public readonly gp_Elips2d Mirrored(in gp_Pnt2d P)
    {
        gp_Elips2d E = this;
        E.pos.Mirror(P);
        return E;
    }

    public readonly gp_Elips2d Mirrored(in gp_Ax2d A)
    {
        gp_Elips2d E = this;
        E.pos.Mirror(A);
        return E;
    }

    public readonly gp_Elips2d Rotated(in gp_Pnt2d theP, double theAng)
    {
        gp_Elips2d anE = this;
        anE.pos.Rotate(theP, theAng);
        return anE;
    }

    public readonly gp_Elips2d Scaled(in gp_Pnt2d theP, double theS)
    {
        gp_Elips2d anE = this;
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

    public readonly gp_Elips2d Transformed(in gp_Trsf2d theT)
    {
        gp_Elips2d anE = this;
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

    public readonly gp_Elips2d Translated(in gp_Vec2d theV)
    {
        gp_Elips2d anE = this;
        anE.pos.Translate(theV);
        return anE;
    }

    public readonly gp_Elips2d Translated(in gp_Pnt2d theP1, in gp_Pnt2d theP2)
    {
        gp_Elips2d anE = this;
        anE.pos.Translate(theP1, theP2);
        return anE;
    }

    public readonly bool Equals(gp_Elips2d other)
        => pos.Equals(other.pos) && majorRadius.Equals(other.majorRadius) && minorRadius.Equals(other.minorRadius);

    public override readonly bool Equals(object? obj) => obj is gp_Elips2d other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(pos, majorRadius, minorRadius);

    public static bool operator ==(in gp_Elips2d theLeft, in gp_Elips2d theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Elips2d theLeft, in gp_Elips2d theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"gp_Elips2d({pos}, major {majorRadius}, minor {minorRadius})";
}
