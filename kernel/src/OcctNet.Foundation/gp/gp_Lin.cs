// Port of: src/FoundationClasses/TKMath/gp/gp_Lin.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Lin.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>An infinite line in 3D space, positioned by an axis (a location and a unit direction).</summary>
/// <remarks><c>new gp_Lin()</c> is the Z axis, as in OCCT; <c>default(gp_Lin)</c> is not valid.</remarks>
public partial struct gp_Lin : IEquatable<gp_Lin>
{
    private gp_Ax1 pos;

    public gp_Lin() => pos = new gp_Ax1();

    public gp_Lin(in gp_Ax1 theA1) => pos = theA1;

    public gp_Lin(in gp_Pnt theP, in gp_Dir theV) => pos = new gp_Ax1(theP, theV);

    // Mutators (internal: see docs/translation-rules.md)

    internal void Reverse() => pos.Reverse();

    internal void SetDirection(in gp_Dir theV) => pos.SetDirection(theV);

    internal void SetLocation(in gp_Pnt theP) => pos.SetLocation(theP);

    internal void SetPosition(in gp_Ax1 theA1) => pos = theA1;

    internal void Mirror(in gp_Pnt P) => pos.Mirror(P);

    internal void Mirror(in gp_Ax1 A1) => pos.Mirror(A1);

    internal void Mirror(in gp_Ax2 A2) => pos.Mirror(A2);

    internal void Rotate(in gp_Ax1 theA1, double theAng) => pos.Rotate(theA1, theAng);

    internal void Scale(in gp_Pnt theP, double theS) => pos.Scale(theP, theS);

    internal void Transform(in gp_Trsf theT) => pos.Transform(theT);

    internal void Translate(in gp_Vec theV) => pos.Translate(theV);

    internal void Translate(in gp_Pnt theP1, in gp_Pnt theP2) => pos.Translate(theP1, theP2);

    // Copies with a mutator applied

    public readonly gp_Lin WithDirection(in gp_Dir theV)
    {
        gp_Lin aCopy = this;
        aCopy.SetDirection(theV);
        return aCopy;
    }

    public readonly gp_Lin WithLocation(in gp_Pnt theP)
    {
        gp_Lin aCopy = this;
        aCopy.SetLocation(theP);
        return aCopy;
    }

    public readonly gp_Lin WithPosition(in gp_Ax1 theA1) => new(theA1);

    // Queries

    public readonly gp_Lin Reversed()
    {
        gp_Lin aL = this;
        aL.pos.Reverse();
        return aL;
    }

    public readonly gp_Dir Direction() => pos.Direction();

    public readonly gp_Pnt Location() => pos.Location();

    public readonly gp_Ax1 Position() => pos;

    /// <summary>The angle between the directions of the two lines, in [0, PI].</summary>
    public readonly double Angle(in gp_Lin theOther) => pos.Direction().Angle(theOther.pos.Direction());

    public readonly bool Contains(in gp_Pnt theP, double theLinearTolerance) => Distance(theP) <= theLinearTolerance;

    public readonly double Distance(in gp_Pnt theP)
    {
        gp_XYZ aCoord = theP.XYZ();
        aCoord.Subtract((pos.Location()).XYZ());
        aCoord.Cross((pos.Direction()).XYZ());
        return aCoord.Modulus();
    }

    /// <summary>The minimum distance between the two lines.</summary>
    public readonly double Distance(in gp_Lin Other)
    {
        if (pos.IsParallel(Other.pos, gp.Resolution()))
        {
            return Other.Distance(pos.Location());
        }
        else
        {
            gp_Dir dir = pos.Direction().Crossed(Other.pos.Direction());
            return Math.Abs(new gp_Vec(pos.Location(), Other.pos.Location()).Dot(new gp_Vec(dir)));
        }
    }

    public readonly double SquareDistance(in gp_Pnt theP)
    {
        gp_Pnt aLoc = pos.Location();
        gp_Vec aV = new(theP.X() - aLoc.X(), theP.Y() - aLoc.Y(), theP.Z() - aLoc.Z());
        aV.Cross(new gp_Vec(pos.Direction()));
        return aV.SquareMagnitude();
    }

    public readonly double SquareDistance(in gp_Lin theOther)
    {
        double aD = Distance(theOther);
        return aD * aD;
    }

    /// <summary>The line through <paramref name="theP"/> normal to this line and intersecting it.</summary>
    /// <exception cref="Standard_ConstructionError"><paramref name="theP"/> is on this line.</exception>
    public readonly gp_Lin Normal(in gp_Pnt theP)
    {
        gp_Pnt aLoc = pos.Location();
        gp_Dir aV = new(theP.X() - aLoc.X(), theP.Y() - aLoc.Y(), theP.Z() - aLoc.Z());
        aV = pos.Direction().CrossCrossed(aV, pos.Direction());
        return new gp_Lin(theP, aV);
    }

    public readonly gp_Lin Mirrored(in gp_Pnt P)
    {
        gp_Lin L = this;
        L.pos.Mirror(P);
        return L;
    }

    public readonly gp_Lin Mirrored(in gp_Ax1 A1)
    {
        gp_Lin L = this;
        L.pos.Mirror(A1);
        return L;
    }

    public readonly gp_Lin Mirrored(in gp_Ax2 A2)
    {
        gp_Lin L = this;
        L.pos.Mirror(A2);
        return L;
    }

    public readonly gp_Lin Rotated(in gp_Ax1 theA1, double theAng)
    {
        gp_Lin aL = this;
        aL.pos.Rotate(theA1, theAng);
        return aL;
    }

    public readonly gp_Lin Scaled(in gp_Pnt theP, double theS)
    {
        gp_Lin aL = this;
        aL.pos.Scale(theP, theS);
        return aL;
    }

    public readonly gp_Lin Transformed(in gp_Trsf theT)
    {
        gp_Lin aL = this;
        aL.pos.Transform(theT);
        return aL;
    }

    public readonly gp_Lin Translated(in gp_Vec theV)
    {
        gp_Lin aL = this;
        aL.pos.Translate(theV);
        return aL;
    }

    public readonly gp_Lin Translated(in gp_Pnt theP1, in gp_Pnt theP2)
    {
        gp_Lin aL = this;
        aL.pos.Translate(new gp_Vec(theP1, theP2));
        return aL;
    }

    public readonly bool Equals(gp_Lin other) => pos.Equals(other.pos);

    public override readonly bool Equals(object? obj) => obj is gp_Lin other && Equals(other);

    public override readonly int GetHashCode() => pos.GetHashCode();

    public static bool operator ==(in gp_Lin theLeft, in gp_Lin theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Lin theLeft, in gp_Lin theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"gp_Lin({pos.Location()}, {pos.Direction()})";
}
