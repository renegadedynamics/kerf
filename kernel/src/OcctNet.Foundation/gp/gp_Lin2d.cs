// Port of: src/FoundationClasses/TKMath/gp/gp_Lin2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Lin2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>An infinite line in 2D space, positioned by an axis (a location and a direction).</summary>
/// <remarks><c>new gp_Lin2d()</c> is the X axis, as in OCCT; <c>default(gp_Lin2d)</c> is not valid.</remarks>
public partial struct gp_Lin2d : IEquatable<gp_Lin2d>
{
    private gp_Ax2d pos;

    public gp_Lin2d() => pos = new gp_Ax2d();

    public gp_Lin2d(in gp_Ax2d theA) => pos = theA;

    public gp_Lin2d(in gp_Pnt2d theP, in gp_Dir2d theV) => pos = new gp_Ax2d(theP, theV);

    /// <summary>The line of equation <c>A * X + B * Y + C = 0</c>.</summary>
    /// <exception cref="Standard_ConstructionError">A * A + B * B is at most <see cref="gp.Resolution"/>.</exception>
    public gp_Lin2d(double A, double B, double C)
    {
        double Norm2 = A * A + B * B;
        Standard_ConstructionError_Raise_if(Norm2 <= gp.Resolution(), " ");
        gp_Pnt2d P = new(-A * C / Norm2, -B * C / Norm2);
        gp_Dir2d V = new(-B, A);
        pos = new gp_Ax2d(P, V);
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void Reverse() => pos.Reverse();

    internal void SetDirection(in gp_Dir2d theV) => pos.SetDirection(theV);

    internal void SetLocation(in gp_Pnt2d theP) => pos.SetLocation(theP);

    internal void SetPosition(in gp_Ax2d theA) => pos = theA;

    internal void Mirror(in gp_Pnt2d P) => pos.Mirror(P);

    internal void Mirror(in gp_Ax2d A) => pos.Mirror(A);

    internal void Rotate(in gp_Pnt2d theP, double theAng) => pos.Rotate(theP, theAng);

    internal void Scale(in gp_Pnt2d theP, double theS) => pos.Scale(theP, theS);

    internal void Transform(in gp_Trsf2d theT) => pos.Transform(theT);

    internal void Translate(in gp_Vec2d theV) => pos.Translate(theV);

    internal void Translate(in gp_Pnt2d theP1, in gp_Pnt2d theP2) => pos.Translate(theP1, theP2);

    // Copies with a mutator applied

    public readonly gp_Lin2d WithDirection(in gp_Dir2d theV)
    {
        gp_Lin2d aCopy = this;
        aCopy.SetDirection(theV);
        return aCopy;
    }

    public readonly gp_Lin2d WithLocation(in gp_Pnt2d theP)
    {
        gp_Lin2d aCopy = this;
        aCopy.SetLocation(theP);
        return aCopy;
    }

    public readonly gp_Lin2d WithPosition(in gp_Ax2d theA) => new(theA);

    // Queries

    /// <summary>The coefficients of the line's equation <c>A * X + B * Y + C = 0</c>.</summary>
    public readonly void Coefficients(out double theA, out double theB, out double theC)
    {
        theA = pos.Direction().Y();
        theB = -pos.Direction().X();
        theC = -(theA * pos.Location().X() + theB * pos.Location().Y());
    }

    public readonly gp_Dir2d Direction() => pos.Direction();

    public readonly gp_Pnt2d Location() => pos.Location();

    public readonly gp_Ax2d Position() => pos;

    /// <summary>The signed angle between the directions of the two lines, in ]-PI, PI].</summary>
    public readonly double Angle(in gp_Lin2d theOther) => pos.Direction().Angle(theOther.pos.Direction());

    public readonly bool Contains(in gp_Pnt2d theP, double theLinearTolerance) => Distance(theP) <= theLinearTolerance;

    public readonly double Distance(in gp_Pnt2d theP)
    {
        gp_XY aCoord = theP.XY();
        aCoord.Subtract((pos.Location()).XY());
        return Math.Abs(aCoord.Crossed(pos.Direction().XY()));
    }

    /// <summary>The distance between the lines: zero unless they are parallel.</summary>
    public readonly double Distance(in gp_Lin2d theOther)
    {
        double aD = 0.0;
        if (pos.IsParallel(theOther.pos, gp.Resolution()))
        {
            aD = theOther.Distance(pos.Location());
        }

        return aD;
    }

    public readonly double SquareDistance(in gp_Pnt2d theP)
    {
        gp_XY aCoord = theP.XY();
        aCoord.Subtract((pos.Location()).XY());
        double aD = aCoord.Crossed(pos.Direction().XY());
        return aD * aD;
    }

    public readonly double SquareDistance(in gp_Lin2d theOther)
    {
        double aD = 0.0;
        if (pos.IsParallel(theOther.pos, gp.Resolution()))
        {
            aD = theOther.SquareDistance(pos.Location());
        }

        return aD;
    }

    /// <summary>The line through <paramref name="theP"/> normal to this one.</summary>
    public readonly gp_Lin2d Normal(in gp_Pnt2d theP)
        => new(new gp_Ax2d(theP, new gp_Dir2d(-(pos.Direction().Y()), pos.Direction().X())));

    public readonly gp_Lin2d Reversed()
    {
        gp_Lin2d aL = this;
        aL.pos.Reverse();
        return aL;
    }

    public readonly gp_Lin2d Mirrored(in gp_Pnt2d P)
    {
        gp_Lin2d L = this;
        L.pos.Mirror(P);
        return L;
    }

    public readonly gp_Lin2d Mirrored(in gp_Ax2d A)
    {
        gp_Lin2d L = this;
        L.pos.Mirror(A);
        return L;
    }

    public readonly gp_Lin2d Rotated(in gp_Pnt2d theP, double theAng)
    {
        gp_Lin2d aL = this;
        aL.pos.Rotate(theP, theAng);
        return aL;
    }

    public readonly gp_Lin2d Scaled(in gp_Pnt2d theP, double theS)
    {
        gp_Lin2d aL = this;
        aL.pos.Scale(theP, theS);
        return aL;
    }

    public readonly gp_Lin2d Transformed(in gp_Trsf2d theT)
    {
        gp_Lin2d aL = this;
        aL.pos.Transform(theT);
        return aL;
    }

    public readonly gp_Lin2d Translated(in gp_Vec2d theV)
    {
        gp_Lin2d aL = this;
        aL.pos.Translate(theV);
        return aL;
    }

    public readonly gp_Lin2d Translated(in gp_Pnt2d theP1, in gp_Pnt2d theP2)
    {
        gp_Lin2d aL = this;
        aL.pos.Translate(new gp_Vec2d(theP1, theP2));
        return aL;
    }

    public readonly bool Equals(gp_Lin2d other) => pos.Equals(other.pos);

    public override readonly bool Equals(object? obj) => obj is gp_Lin2d other && Equals(other);

    public override readonly int GetHashCode() => pos.GetHashCode();

    public static bool operator ==(in gp_Lin2d theLeft, in gp_Lin2d theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Lin2d theLeft, in gp_Lin2d theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"gp_Lin2d({pos.Location()}, {pos.Direction()})";
}
