// Port of: src/FoundationClasses/TKMath/gp/gp_Ax1.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Ax1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>An axis in 3D space: a location and a direction.</summary>
/// <remarks><c>new gp_Ax1()</c> is the Z axis at the origin, as in OCCT; <c>default(gp_Ax1)</c> is not valid.</remarks>
public partial struct gp_Ax1 : IEquatable<gp_Ax1>
{
    private gp_Pnt loc;
    private gp_Dir vdir;

    public gp_Ax1()
    {
        loc = new gp_Pnt(0.0, 0.0, 0.0);
        vdir = new gp_Dir(gp_Dir.D.Z);
    }

    public gp_Ax1(in gp_Pnt theP, in gp_Dir theV)
    {
        loc = theP;
        vdir = theV;
    }

    public gp_Ax1(in gp_Pnt theP, gp_Dir.D theDir)
    {
        loc = theP;
        vdir = new gp_Dir(theDir);
    }

    public gp_Ax1(gp_Dir.D theDir)
    {
        loc = new gp_Pnt(0.0, 0.0, 0.0);
        vdir = new gp_Dir(theDir);
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetDirection(in gp_Dir theV) => vdir = theV;

    internal void SetLocation(in gp_Pnt theP) => loc = theP;

    internal void Reverse() => vdir.Reverse();

    internal void Mirror(in gp_Pnt P)
    {
        loc.Mirror(P);
        vdir.Reverse();
    }

    internal void Mirror(in gp_Ax1 A1)
    {
        loc.Mirror(A1);
        vdir.Mirror(A1.vdir);
    }

    internal void Mirror(in gp_Ax2 A2)
    {
        loc.Mirror(A2);
        vdir.Mirror(A2);
    }

    internal void Rotate(in gp_Ax1 theA1, double theAngRad)
    {
        loc.Rotate(theA1, theAngRad);
        vdir.Rotate(theA1, theAngRad);
    }

    internal void Scale(in gp_Pnt theP, double theS)
    {
        loc.Scale(theP, theS);
        if (theS < 0.0)
        {
            vdir.Reverse();
        }
    }

    internal void Transform(in gp_Trsf theT)
    {
        loc.Transform(theT);
        vdir.Transform(theT);
    }

    internal void Translate(in gp_Vec theV) => loc.Translate(theV);

    internal void Translate(in gp_Pnt theP1, in gp_Pnt theP2) => loc.Translate(theP1, theP2);

    // Copies with a mutator applied

    public readonly gp_Ax1 WithDirection(in gp_Dir theV) => new(loc, theV);

    public readonly gp_Ax1 WithLocation(in gp_Pnt theP) => new(theP, vdir);

    // Queries

    public readonly gp_Dir Direction() => vdir;

    public readonly gp_Pnt Location() => loc;

    public readonly bool IsCoaxial(in gp_Ax1 Other, double AngularTolerance, double LinearTolerance)
    {
        gp_XYZ XYZ1 = loc.XYZ();
        XYZ1.Subtract(Other.loc.XYZ());
        XYZ1.Cross(Other.vdir.XYZ());
        double D1 = XYZ1.Modulus();
        gp_XYZ XYZ2 = Other.loc.XYZ();
        XYZ2.Subtract(loc.XYZ());
        XYZ2.Cross(vdir.XYZ());
        double D2 = XYZ2.Modulus();
        return vdir.IsEqual(Other.vdir, AngularTolerance) && D1 <= LinearTolerance && D2 <= LinearTolerance;
    }

    public readonly bool IsNormal(in gp_Ax1 theOther, double theAngularTolerance) => vdir.IsNormal(theOther.vdir, theAngularTolerance);

    public readonly bool IsOpposite(in gp_Ax1 theOther, double theAngularTolerance) => vdir.IsOpposite(theOther.vdir, theAngularTolerance);

    public readonly bool IsParallel(in gp_Ax1 theOther, double theAngularTolerance) => vdir.IsParallel(theOther.vdir, theAngularTolerance);

    public readonly double Angle(in gp_Ax1 theOther) => vdir.Angle(theOther.vdir);

    public readonly gp_Ax1 Reversed() => new(loc, vdir.Reversed());

    public readonly gp_Ax1 Mirrored(in gp_Pnt P)
    {
        gp_Ax1 A1 = this;
        A1.Mirror(P);
        return A1;
    }

    public readonly gp_Ax1 Mirrored(in gp_Ax1 A1)
    {
        gp_Ax1 A = this;
        A.Mirror(A1);
        return A;
    }

    public readonly gp_Ax1 Mirrored(in gp_Ax2 A2)
    {
        gp_Ax1 A1 = this;
        A1.Mirror(A2);
        return A1;
    }

    public readonly gp_Ax1 Rotated(in gp_Ax1 theA1, double theAngRad)
    {
        gp_Ax1 A = this;
        A.Rotate(theA1, theAngRad);
        return A;
    }

    public readonly gp_Ax1 Scaled(in gp_Pnt theP, double theS)
    {
        gp_Ax1 A1 = this;
        A1.Scale(theP, theS);
        return A1;
    }

    public readonly gp_Ax1 Transformed(in gp_Trsf theT)
    {
        gp_Ax1 A1 = this;
        A1.Transform(theT);
        return A1;
    }

    public readonly gp_Ax1 Translated(in gp_Vec theV)
    {
        gp_Ax1 A1 = this;
        A1.loc.Translate(theV);
        return A1;
    }

    public readonly gp_Ax1 Translated(in gp_Pnt theP1, in gp_Pnt theP2)
    {
        gp_Ax1 A1 = this;
        A1.loc.Translate(theP1, theP2);
        return A1;
    }

    public readonly bool Equals(gp_Ax1 other) => loc.Equals(other.loc) && vdir.Equals(other.vdir);

    public override readonly bool Equals(object? obj) => obj is gp_Ax1 other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(loc, vdir);

    public static bool operator ==(in gp_Ax1 theLeft, in gp_Ax1 theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Ax1 theLeft, in gp_Ax1 theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"gp_Ax1({loc}, {vdir})";
}
