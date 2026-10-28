// Port of: src/FoundationClasses/TKMath/gp/gp_Ax2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Ax2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>An axis in 2D space: a location and a direction.</summary>
/// <remarks><c>new gp_Ax2d()</c> is the X axis at the origin, as in OCCT; <c>default(gp_Ax2d)</c> is not valid.</remarks>
public partial struct gp_Ax2d : IEquatable<gp_Ax2d>
{
    private gp_Pnt2d loc;
    private gp_Dir2d vdir;

    public gp_Ax2d()
    {
        loc = new gp_Pnt2d(0.0, 0.0);
        vdir = new gp_Dir2d();
    }

    public gp_Ax2d(in gp_Pnt2d theP, in gp_Dir2d theV)
    {
        loc = theP;
        vdir = theV;
    }

    public gp_Ax2d(in gp_Pnt2d theP, gp_Dir2d.D theDir)
    {
        loc = theP;
        vdir = new gp_Dir2d(theDir);
    }

    public gp_Ax2d(gp_Dir2d.D theDir)
    {
        loc = new gp_Pnt2d(0.0, 0.0);
        vdir = new gp_Dir2d(theDir);
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetLocation(in gp_Pnt2d theP) => loc = theP;

    internal void SetDirection(in gp_Dir2d theV) => vdir = theV;

    internal void Reverse() => vdir.Reverse();

    internal void Mirror(in gp_Pnt2d P)
    {
        loc.Mirror(P);
        vdir.Reverse();
    }

    internal void Mirror(in gp_Ax2d A)
    {
        loc.Mirror(A);
        vdir.Mirror(A.vdir);
    }

    internal void Rotate(in gp_Pnt2d theP, double theAng)
    {
        loc.Rotate(theP, theAng);
        vdir.Rotate(theAng);
    }

    internal void Scale(in gp_Pnt2d P, double S)
    {
        loc.Scale(P, S);
        if (S < 0.0)
        {
            vdir.Reverse();
        }
    }

    internal void Transform(in gp_Trsf2d theT)
    {
        loc.Transform(theT);
        vdir.Transform(theT);
    }

    internal void Translate(in gp_Vec2d theV) => loc.Translate(theV);

    internal void Translate(in gp_Pnt2d theP1, in gp_Pnt2d theP2) => loc.Translate(theP1, theP2);

    // Copies with a mutator applied

    public readonly gp_Ax2d WithLocation(in gp_Pnt2d theP) => new(theP, vdir);

    public readonly gp_Ax2d WithDirection(in gp_Dir2d theV) => new(loc, theV);

    // Queries

    public readonly gp_Pnt2d Location() => loc;

    public readonly gp_Dir2d Direction() => vdir;

    /// <summary>True when the directions are parallel within the angular tolerance and each location is within the linear tolerance of the other axis.</summary>
    public readonly bool IsCoaxial(in gp_Ax2d Other, double AngularTolerance, double LinearTolerance)
    {
        gp_XY XY1 = loc.XY();
        XY1.Subtract(Other.loc.XY());
        double D1 = XY1.Crossed(Other.vdir.XY());
        if (D1 < 0)
        {
            D1 = -D1;
        }

        gp_XY XY2 = Other.loc.XY();
        XY2.Subtract(loc.XY());
        double D2 = XY2.Crossed(vdir.XY());
        if (D2 < 0)
        {
            D2 = -D2;
        }

        return (vdir.IsParallel(Other.vdir, AngularTolerance) && D1 <= LinearTolerance && D2 <= LinearTolerance);
    }

    public readonly bool IsNormal(in gp_Ax2d theOther, double theAngularTolerance) => vdir.IsNormal(theOther.vdir, theAngularTolerance);

    public readonly bool IsOpposite(in gp_Ax2d theOther, double theAngularTolerance) => vdir.IsOpposite(theOther.vdir, theAngularTolerance);

    public readonly bool IsParallel(in gp_Ax2d theOther, double theAngularTolerance) => vdir.IsParallel(theOther.vdir, theAngularTolerance);

    /// <summary>The signed angle between the directions, in ]-PI, PI].</summary>
    public readonly double Angle(in gp_Ax2d theOther) => vdir.Angle(theOther.vdir);

    public readonly gp_Ax2d Reversed()
    {
        gp_Ax2d aTemp = this;
        aTemp.Reverse();
        return aTemp;
    }

    public readonly gp_Ax2d Mirrored(in gp_Pnt2d P)
    {
        gp_Ax2d A = this;
        A.Mirror(P);
        return A;
    }

    public readonly gp_Ax2d Mirrored(in gp_Ax2d A)
    {
        gp_Ax2d AA = this;
        AA.Mirror(A);
        return AA;
    }

    public readonly gp_Ax2d Rotated(in gp_Pnt2d theP, double theAng)
    {
        gp_Ax2d anA = this;
        anA.Rotate(theP, theAng);
        return anA;
    }

    public readonly gp_Ax2d Scaled(in gp_Pnt2d theP, double theS)
    {
        gp_Ax2d anA = this;
        anA.Scale(theP, theS);
        return anA;
    }

    public readonly gp_Ax2d Transformed(in gp_Trsf2d theT)
    {
        gp_Ax2d anA = this;
        anA.Transform(theT);
        return anA;
    }

    public readonly gp_Ax2d Translated(in gp_Vec2d theV)
    {
        gp_Ax2d anA = this;
        (anA.loc).Translate(theV);
        return anA;
    }

    public readonly gp_Ax2d Translated(in gp_Pnt2d theP1, in gp_Pnt2d theP2)
    {
        gp_Ax2d anA = this;
        (anA.loc).Translate(theP1, theP2);
        return anA;
    }

    public readonly bool Equals(gp_Ax2d other) => loc.Equals(other.loc) && vdir.Equals(other.vdir);

    public override readonly bool Equals(object? obj) => obj is gp_Ax2d other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(loc, vdir);

    public static bool operator ==(in gp_Ax2d theLeft, in gp_Ax2d theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Ax2d theLeft, in gp_Ax2d theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"gp_Ax2d({loc}, {vdir})";
}
