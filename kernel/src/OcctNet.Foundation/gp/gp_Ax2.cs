// Port of: src/FoundationClasses/TKMath/gp/gp_Ax2.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Ax2.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A right-handed coordinate system in 3D space: a location, a main direction, and X and Y directions.</summary>
/// <remarks><c>new gp_Ax2()</c> is the global system, as in OCCT; <c>default(gp_Ax2)</c> is not valid.</remarks>
public partial struct gp_Ax2 : IEquatable<gp_Ax2>
{
    private gp_Ax1 axis;
    private gp_Dir vydir;
    private gp_Dir vxdir;

    public gp_Ax2()
    {
        axis = new gp_Ax1();
        vydir = new gp_Dir(gp_Dir.D.Y);
        vxdir = new gp_Dir(gp_Dir.D.X);
    }

    /// <summary>A system at <paramref name="P"/> with main direction <paramref name="N"/>, and X direction from <paramref name="Vx"/> projected onto the plane normal to N.</summary>
    public gp_Ax2(in gp_Pnt P, in gp_Dir N, in gp_Dir Vx)
    {
        axis = new gp_Ax1(P, N);
        vydir = N;
        vxdir = N;
        vxdir.CrossCross(Vx, N);
        vydir.Cross(vxdir);
    }

    public gp_Ax2(in gp_Pnt theP, gp_Dir.D theN, gp_Dir.D theVx)
    {
        axis = new gp_Ax1(theP, theN);
        vydir = new gp_Dir(crossStandardDir(theN, theVx));
        vxdir = new gp_Dir(theVx);
    }

    /// <summary>A system at <paramref name="P"/> with main direction <paramref name="V"/>, and X and Y directions chosen by OCCT's rule.</summary>
    public gp_Ax2(in gp_Pnt P, in gp_Dir V)
    {
        axis = new gp_Ax1(P, V);
        vydir = new gp_Dir();
        vxdir = new gp_Dir();
        double A = V.X();
        double B = V.Y();
        double C = V.Z();
        double Aabs = Math.Abs(A);
        double Babs = Math.Abs(B);
        double Cabs = Math.Abs(C);
        gp_Dir D = new();

        // Pick the X direction perpendicular to V from the component of smallest magnitude
        if (Babs <= Aabs && Babs <= Cabs)
        {
            if (Aabs > Cabs)
            {
                D.SetCoord(-C, 0.0, A);
            }
            else
            {
                D.SetCoord(C, 0.0, -A);
            }
        }
        else if (Aabs <= Babs && Aabs <= Cabs)
        {
            if (Babs > Cabs)
            {
                D.SetCoord(0.0, -C, B);
            }
            else
            {
                D.SetCoord(0.0, C, -B);
            }
        }
        else
        {
            if (Aabs > Babs)
            {
                D.SetCoord(-B, A, 0.0);
            }
            else
            {
                D.SetCoord(B, -A, 0.0);
            }
        }

        SetXDirection(D);
    }

    public gp_Ax2(in gp_Pnt theP, gp_Dir.D theV)
    {
        axis = new gp_Ax1(theP, theV);
        vydir = new gp_Dir(getPerpendicularYDir(theV));
        vxdir = new gp_Dir(getPerpendicularXDir(theV));
    }

    public gp_Ax2(gp_Dir.D theV) : this(new gp_Pnt(0.0, 0.0, 0.0), theV) { }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetAxis(in gp_Ax1 theA1)
    {
        double a = theA1.Direction() * vxdir;
        if (Math.Abs(Math.Abs(a) - 1.0) <= Precision.Angular())
        {
            if (a > 0.0)
            {
                vxdir = vydir;
                vydir = axis.Direction();
                axis = theA1;
            }
            else
            {
                vxdir = axis.Direction();
                axis = theA1;
            }
        }
        else
        {
            axis = theA1;
            vxdir = axis.Direction().CrossCrossed(vxdir, axis.Direction());
            vydir = axis.Direction().Crossed(vxdir);
        }
    }

    internal void SetDirection(in gp_Dir theV)
    {
        double a = theV * vxdir;
        if (Math.Abs(Math.Abs(a) - 1.0) <= Precision.Angular())
        {
            if (a > 0.0)
            {
                vxdir = vydir;
                vydir = axis.Direction();
                axis.SetDirection(theV);
            }
            else
            {
                vxdir = axis.Direction();
                axis.SetDirection(theV);
            }
        }
        else
        {
            axis.SetDirection(theV);
            vxdir = theV.CrossCrossed(vxdir, theV);
            vydir = theV.Crossed(vxdir);
        }
    }

    internal void SetLocation(in gp_Pnt theP) => axis.SetLocation(theP);

    internal void SetXDirection(in gp_Dir theVx)
    {
        vxdir = axis.Direction().CrossCrossed(theVx, axis.Direction());
        vydir = axis.Direction().Crossed(vxdir);
    }

    internal void SetYDirection(in gp_Dir theVy)
    {
        vxdir = theVy.Crossed(axis.Direction());
        vydir = axis.Direction().Crossed(vxdir);
    }

    internal void Mirror(in gp_Pnt P)
    {
        gp_Pnt Temp = axis.Location();
        Temp.Mirror(P);
        axis.SetLocation(Temp);
        vxdir.Reverse();
        vydir.Reverse();
    }

    internal void Mirror(in gp_Ax1 A1)
    {
        vydir.Mirror(A1);
        vxdir.Mirror(A1);
        gp_Pnt Temp = axis.Location();
        Temp.Mirror(A1);
        axis.SetLocation(Temp);
        axis.SetDirection(vxdir.Crossed(vydir));
    }

    internal void Mirror(in gp_Ax2 A2)
    {
        vydir.Mirror(A2);
        vxdir.Mirror(A2);
        gp_Pnt Temp = axis.Location();
        Temp.Mirror(A2);
        axis.SetLocation(Temp);
        axis.SetDirection(vxdir.Crossed(vydir));
    }

    internal void Rotate(in gp_Ax1 theA1, double theAng)
    {
        gp_Pnt aTemp = axis.Location();
        aTemp.Rotate(theA1, theAng);
        axis.SetLocation(aTemp);
        vxdir.Rotate(theA1, theAng);
        vydir.Rotate(theA1, theAng);
        axis.SetDirection(vxdir.Crossed(vydir));
    }

    internal void Scale(in gp_Pnt theP, double theS)
    {
        gp_Pnt aTemp = axis.Location();
        aTemp.Scale(theP, theS);
        axis.SetLocation(aTemp);
        if (theS < 0.0)
        {
            vxdir.Reverse();
            vydir.Reverse();
        }
    }

    internal void Transform(in gp_Trsf theT)
    {
        gp_Pnt aTemp = axis.Location();
        aTemp.Transform(theT);
        axis.SetLocation(aTemp);
        vxdir.Transform(theT);
        vydir.Transform(theT);
        axis.SetDirection(vxdir.Crossed(vydir));
    }

    internal void Translate(in gp_Vec theV) => axis.Translate(theV);

    internal void Translate(in gp_Pnt theP1, in gp_Pnt theP2) => axis.Translate(theP1, theP2);

    // Copies with a mutator applied

    public readonly gp_Ax2 WithAxis(in gp_Ax1 theA1)
    {
        gp_Ax2 aCopy = this;
        aCopy.SetAxis(theA1);
        return aCopy;
    }

    public readonly gp_Ax2 WithDirection(in gp_Dir theV)
    {
        gp_Ax2 aCopy = this;
        aCopy.SetDirection(theV);
        return aCopy;
    }

    public readonly gp_Ax2 WithLocation(in gp_Pnt theP)
    {
        gp_Ax2 aCopy = this;
        aCopy.SetLocation(theP);
        return aCopy;
    }

    public readonly gp_Ax2 WithXDirection(in gp_Dir theVx)
    {
        gp_Ax2 aCopy = this;
        aCopy.SetXDirection(theVx);
        return aCopy;
    }

    public readonly gp_Ax2 WithYDirection(in gp_Dir theVy)
    {
        gp_Ax2 aCopy = this;
        aCopy.SetYDirection(theVy);
        return aCopy;
    }

    // Queries

    public readonly double Angle(in gp_Ax2 theOther) => axis.Angle(theOther.axis);

    public readonly gp_Ax1 Axis() => axis;

    public readonly gp_Dir Direction() => axis.Direction();

    public readonly gp_Pnt Location() => axis.Location();

    public readonly gp_Dir XDirection() => vxdir;

    public readonly gp_Dir YDirection() => vydir;

    public readonly bool IsCoplanar(in gp_Ax2 theOther, double theLinearTolerance, double theAngularTolerance)
    {
        gp_Dir DD = axis.Direction();
        gp_Pnt PP = axis.Location();
        gp_Pnt OP = theOther.axis.Location();
        double D1 = DD.X() * (OP.X() - PP.X()) + DD.Y() * (OP.Y() - PP.Y()) + DD.Z() * (OP.Z() - PP.Z());
        if (D1 < 0)
        {
            D1 = -D1;
        }

        return D1 <= theLinearTolerance && axis.IsParallel(theOther.axis, theAngularTolerance);
    }

    public readonly bool IsCoplanar(in gp_Ax1 theA, double theLinearTolerance, double theAngularTolerance)
    {
        gp_Dir DD = axis.Direction();
        gp_Pnt PP = axis.Location();
        gp_Pnt AP = theA.Location();
        double D1 = DD.X() * (AP.X() - PP.X()) + DD.Y() * (AP.Y() - PP.Y()) + DD.Z() * (AP.Z() - PP.Z());
        if (D1 < 0)
        {
            D1 = -D1;
        }

        return D1 <= theLinearTolerance && axis.IsNormal(theA, theAngularTolerance);
    }

    public readonly gp_Ax2 Mirrored(in gp_Pnt P)
    {
        gp_Ax2 Temp = this;
        Temp.Mirror(P);
        return Temp;
    }

    public readonly gp_Ax2 Mirrored(in gp_Ax1 A1)
    {
        gp_Ax2 Temp = this;
        Temp.Mirror(A1);
        return Temp;
    }

    public readonly gp_Ax2 Mirrored(in gp_Ax2 A2)
    {
        gp_Ax2 Temp = this;
        Temp.Mirror(A2);
        return Temp;
    }

    public readonly gp_Ax2 Rotated(in gp_Ax1 theA1, double theAng)
    {
        gp_Ax2 aTemp = this;
        aTemp.Rotate(theA1, theAng);
        return aTemp;
    }

    public readonly gp_Ax2 Scaled(in gp_Pnt theP, double theS)
    {
        gp_Ax2 aTemp = this;
        aTemp.Scale(theP, theS);
        return aTemp;
    }

    public readonly gp_Ax2 Transformed(in gp_Trsf theT)
    {
        gp_Ax2 aTemp = this;
        aTemp.Transform(theT);
        return aTemp;
    }

    public readonly gp_Ax2 Translated(in gp_Vec theV)
    {
        gp_Ax2 aTemp = this;
        aTemp.Translate(theV);
        return aTemp;
    }

    public readonly gp_Ax2 Translated(in gp_Pnt theP1, in gp_Pnt theP2)
    {
        gp_Ax2 aTemp = this;
        aTemp.Translate(theP1, theP2);
        return aTemp;
    }

    internal static gp_Dir.D getPerpendicularXDir(gp_Dir.D theMainDir)
        => theMainDir is gp_Dir.D.X or gp_Dir.D.NX ? gp_Dir.D.Y
           : theMainDir is gp_Dir.D.Y or gp_Dir.D.NY ? gp_Dir.D.Z
           : gp_Dir.D.X;

    internal static gp_Dir.D getPerpendicularYDir(gp_Dir.D theMainDir)
        => theMainDir == gp_Dir.D.Z ? gp_Dir.D.Y
           : theMainDir == gp_Dir.D.NZ ? gp_Dir.D.NY
           : theMainDir == gp_Dir.D.X ? gp_Dir.D.Z
           : theMainDir == gp_Dir.D.NX ? gp_Dir.D.NZ
           : theMainDir == gp_Dir.D.Y ? gp_Dir.D.X
           : gp_Dir.D.NX;

    /// <summary>The cross product of two of the six axis directions (Y = Z x X).</summary>
    internal static gp_Dir.D crossStandardDir(gp_Dir.D theA, gp_Dir.D theB)
    {
        switch (theA)
        {
            case gp_Dir.D.X:
                return theB switch
                {
                    gp_Dir.D.Y => gp_Dir.D.Z,
                    gp_Dir.D.Z => gp_Dir.D.NY,
                    gp_Dir.D.NY => gp_Dir.D.NZ,
                    gp_Dir.D.NZ => gp_Dir.D.Y,
                    _ => gp_Dir.D.Z,
                };
            case gp_Dir.D.Y:
                return theB switch
                {
                    gp_Dir.D.Z => gp_Dir.D.X,
                    gp_Dir.D.X => gp_Dir.D.NZ,
                    gp_Dir.D.NX => gp_Dir.D.Z,
                    gp_Dir.D.NZ => gp_Dir.D.NX,
                    _ => gp_Dir.D.X,
                };
            case gp_Dir.D.Z:
                return theB switch
                {
                    gp_Dir.D.X => gp_Dir.D.Y,
                    gp_Dir.D.Y => gp_Dir.D.NX,
                    gp_Dir.D.NX => gp_Dir.D.NY,
                    gp_Dir.D.NY => gp_Dir.D.X,
                    _ => gp_Dir.D.Y,
                };
            case gp_Dir.D.NX:
                return theB switch
                {
                    gp_Dir.D.Y => gp_Dir.D.NZ,
                    gp_Dir.D.Z => gp_Dir.D.Y,
                    gp_Dir.D.NY => gp_Dir.D.Z,
                    gp_Dir.D.NZ => gp_Dir.D.NY,
                    _ => gp_Dir.D.NZ,
                };
            case gp_Dir.D.NY:
                return theB switch
                {
                    gp_Dir.D.Z => gp_Dir.D.NX,
                    gp_Dir.D.X => gp_Dir.D.Z,
                    gp_Dir.D.NX => gp_Dir.D.NZ,
                    gp_Dir.D.NZ => gp_Dir.D.X,
                    _ => gp_Dir.D.NX,
                };
            case gp_Dir.D.NZ:
                return theB switch
                {
                    gp_Dir.D.X => gp_Dir.D.NY,
                    gp_Dir.D.Y => gp_Dir.D.X,
                    gp_Dir.D.NX => gp_Dir.D.Y,
                    gp_Dir.D.NY => gp_Dir.D.NX,
                    _ => gp_Dir.D.NY,
                };
            default:
                return gp_Dir.D.Z;
        }
    }

    public readonly bool Equals(gp_Ax2 other) => axis.Equals(other.axis) && vxdir.Equals(other.vxdir) && vydir.Equals(other.vydir);

    public override readonly bool Equals(object? obj) => obj is gp_Ax2 other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(axis, vxdir, vydir);

    public static bool operator ==(in gp_Ax2 theLeft, in gp_Ax2 theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Ax2 theLeft, in gp_Ax2 theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"gp_Ax2({axis.Location()}, {axis.Direction()}, {vxdir})";
}
