// Port of: src/FoundationClasses/TKMath/gp/gp_Circ2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Circ2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A circle in 2D space, positioned by a coordinate system whose origin is the centre. It is parametrized
/// from the X axis, counterclockwise when the system is direct.
/// </summary>
/// <remarks>
/// <c>new gp_Circ2d()</c> is OCCT's indefinite circle (radius <see cref="Standard.RealLast"/>);
/// <c>default(gp_Circ2d)</c> is not valid.
/// </remarks>
public partial struct gp_Circ2d : IEquatable<gp_Circ2d>
{
    private gp_Ax22d pos;
    private double radius;

    public gp_Circ2d()
    {
        pos = new gp_Ax22d();
        radius = RealLast();
    }

    /// <summary>A circle centred on the location of <paramref name="theXAxis"/>, direct when <paramref name="theIsSense"/> is true.</summary>
    /// <exception cref="Standard_ConstructionError">The radius is negative.</exception>
    public gp_Circ2d(in gp_Ax2d theXAxis, double theRadius, bool theIsSense = true)
    {
        pos = new gp_Ax22d(theXAxis, theIsSense);
        radius = theRadius;
        Standard_ConstructionError_Raise_if(theRadius < 0.0, "gp_Circ2d() - radius should be positive number");
    }

    /// <exception cref="Standard_ConstructionError">The radius is negative.</exception>
    public gp_Circ2d(in gp_Ax22d theAxis, double theRadius)
    {
        pos = theAxis;
        radius = theRadius;
        Standard_ConstructionError_Raise_if(theRadius < 0.0, "gp_Circ2d() - radius should be positive number");
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetLocation(in gp_Pnt2d theP) => pos.SetLocation(theP);

    internal void SetXAxis(in gp_Ax2d theA) => pos.SetXAxis(theA);

    internal void SetAxis(in gp_Ax22d theA) => pos.SetAxis(theA);

    internal void SetYAxis(in gp_Ax2d theA) => pos.SetYAxis(theA);

    internal void SetRadius(double theRadius)
    {
        Standard_ConstructionError_Raise_if(theRadius < 0.0, "gp_Circ2d::SetRadius() - radius should be positive number");
        radius = theRadius;
    }

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
        radius *= theS;
        if (radius < 0)
        {
            radius = -radius;
        }

        pos.Scale(theP, theS);
    }

    internal void Transform(in gp_Trsf2d theT)
    {
        radius *= theT.ScaleFactor();
        if (radius < 0)
        {
            radius = -radius;
        }

        pos.Transform(theT);
    }

    internal void Translate(in gp_Vec2d theV) => pos.Translate(theV);

    internal void Translate(in gp_Pnt2d theP1, in gp_Pnt2d theP2) => pos.Translate(theP1, theP2);

    // Copies with a mutator applied

    public readonly gp_Circ2d WithLocation(in gp_Pnt2d theP)
    {
        gp_Circ2d aCopy = this;
        aCopy.SetLocation(theP);
        return aCopy;
    }

    public readonly gp_Circ2d WithXAxis(in gp_Ax2d theA)
    {
        gp_Circ2d aCopy = this;
        aCopy.SetXAxis(theA);
        return aCopy;
    }

    public readonly gp_Circ2d WithAxis(in gp_Ax22d theA)
    {
        gp_Circ2d aCopy = this;
        aCopy.SetAxis(theA);
        return aCopy;
    }

    public readonly gp_Circ2d WithYAxis(in gp_Ax2d theA)
    {
        gp_Circ2d aCopy = this;
        aCopy.SetYAxis(theA);
        return aCopy;
    }

    public readonly gp_Circ2d WithRadius(double theRadius)
    {
        gp_Circ2d aCopy = this;
        aCopy.SetRadius(theRadius);
        return aCopy;
    }

    // Queries

    public readonly double Area() => M_PI * radius * radius;

    /// <summary>
    /// The coefficients of the circle's implicit equation
    /// <c>A * X^2 + B * Y^2 + 2 * C * X * Y + 2 * D * X + 2 * E * Y + F = 0</c>.
    /// </summary>
    public readonly void Coefficients(out double theA, out double theB, out double theC, out double theD, out double theE, out double theF)
    {
        double aXc = pos.Location().X();
        double anYc = pos.Location().Y();
        theA = 1.0;
        theB = 1.0;
        theC = 0.0;
        theD = -aXc;
        theE = -anYc;
        theF = aXc * aXc + anYc * anYc - radius * radius;
    }

    public readonly bool Contains(in gp_Pnt2d theP, double theLinearTolerance) => Distance(theP) <= theLinearTolerance;

    public readonly double Distance(in gp_Pnt2d theP)
    {
        gp_XY aCoord = theP.XY();
        aCoord.Subtract(pos.Location().XY());
        double aD = radius - aCoord.Modulus();
        if (aD < 0)
        {
            aD = -aD;
        }

        return aD;
    }

    public readonly double SquareDistance(in gp_Pnt2d theP)
    {
        gp_XY aCoord = theP.XY();
        aCoord.Subtract(pos.Location().XY());
        double aD = radius - aCoord.Modulus();
        return aD * aD;
    }

    public readonly double Length() => 2.0 * M_PI * radius;

    public readonly gp_Pnt2d Location() => pos.Location();

    public readonly double Radius() => radius;

    public readonly gp_Ax22d Axis() => pos;

    public readonly gp_Ax22d Position() => pos;

    public readonly gp_Ax2d XAxis() => pos.XAxis();

    public readonly gp_Ax2d YAxis() => pos.YAxis();

    /// <summary>True when the circle's system is direct (counterclockwise).</summary>
    public readonly bool IsDirect() => (pos.XDirection().Crossed(pos.YDirection())) >= 0.0;

    public readonly gp_Circ2d Reversed()
    {
        gp_Circ2d aCirc = this;
        gp_Dir2d aTemp = pos.YDirection();
        aTemp.Reverse();
        aCirc.pos.SetAxis(new gp_Ax22d(pos.Location(), pos.XDirection(), aTemp));
        return aCirc;
    }

    public readonly gp_Circ2d Mirrored(in gp_Pnt2d P)
    {
        gp_Circ2d C = this;
        C.pos.Mirror(P);
        return C;
    }

    public readonly gp_Circ2d Mirrored(in gp_Ax2d A)
    {
        gp_Circ2d C = this;
        C.pos.Mirror(A);
        return C;
    }

    public readonly gp_Circ2d Rotated(in gp_Pnt2d theP, double theAng)
    {
        gp_Circ2d aCirc = this;
        aCirc.pos.Rotate(theP, theAng);
        return aCirc;
    }

    public readonly gp_Circ2d Scaled(in gp_Pnt2d theP, double theS)
    {
        gp_Circ2d aCirc = this;
        aCirc.radius *= theS;
        if (aCirc.radius < 0)
        {
            aCirc.radius = -aCirc.radius;
        }

        aCirc.pos.Scale(theP, theS);
        return aCirc;
    }

    public readonly gp_Circ2d Transformed(in gp_Trsf2d theT)
    {
        gp_Circ2d aCirc = this;
        aCirc.radius *= theT.ScaleFactor();
        if (aCirc.radius < 0)
        {
            aCirc.radius = -aCirc.radius;
        }

        aCirc.pos.Transform(theT);
        return aCirc;
    }

    public readonly gp_Circ2d Translated(in gp_Vec2d theV)
    {
        gp_Circ2d aCirc = this;
        aCirc.pos.Translate(theV);
        return aCirc;
    }

    public readonly gp_Circ2d Translated(in gp_Pnt2d theP1, in gp_Pnt2d theP2)
    {
        gp_Circ2d aCirc = this;
        aCirc.pos.Translate(theP1, theP2);
        return aCirc;
    }

    public readonly bool Equals(gp_Circ2d other) => pos.Equals(other.pos) && radius.Equals(other.radius);

    public override readonly bool Equals(object? obj) => obj is gp_Circ2d other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(pos, radius);

    public static bool operator ==(in gp_Circ2d theLeft, in gp_Circ2d theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Circ2d theLeft, in gp_Circ2d theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"gp_Circ2d({pos}, radius {radius})";
}
