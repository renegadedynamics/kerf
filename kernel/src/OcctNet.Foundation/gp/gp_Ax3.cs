// Port of: src/FoundationClasses/TKMath/gp/gp_Ax3.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Ax3.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A coordinate system in 3D space, right- or left-handed ("direct" or "indirect").</summary>
/// <remarks><c>new gp_Ax3()</c> is the global system, as in OCCT; <c>default(gp_Ax3)</c> is not valid.</remarks>
public partial struct gp_Ax3 : IEquatable<gp_Ax3>
{
    private gp_Ax1 axis;
    private gp_Dir vydir;
    private gp_Dir vxdir;

    public gp_Ax3()
    {
        axis = new gp_Ax1();
        vydir = new gp_Dir(gp_Dir.D.Y);
        vxdir = new gp_Dir(gp_Dir.D.X);
    }

    public gp_Ax3(in gp_Ax2 theA)
    {
        axis = theA.Axis();
        vydir = theA.YDirection();
        vxdir = theA.XDirection();
    }

    public gp_Ax3(in gp_Pnt theP, in gp_Dir theN, in gp_Dir theVx)
    {
        axis = new gp_Ax1(theP, theN);
        vydir = theN;
        vxdir = theN;
        vxdir.CrossCross(theVx, theN);
        vydir.Cross(vxdir);
    }

    public gp_Ax3(in gp_Pnt theP, gp_Dir.D theN, gp_Dir.D theVx)
    {
        axis = new gp_Ax1(theP, theN);
        vydir = new gp_Dir(gp_Ax2.crossStandardDir(theN, theVx));
        vxdir = new gp_Dir(theVx);
    }

    public gp_Ax3(in gp_Pnt P, in gp_Dir V)
    {
        axis = new gp_Ax1(P, V);
        double A = V.X();
        double B = V.Y();
        double C = V.Z();
        double Aabs = Math.Abs(A);
        double Babs = Math.Abs(B);
        double Cabs = Math.Abs(C);
        gp_Dir D = new();

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

        vxdir = D;
        vydir = V.Crossed(vxdir);
    }

    public gp_Ax3(in gp_Pnt theP, gp_Dir.D theV)
    {
        axis = new gp_Ax1(theP, theV);
        vydir = new gp_Dir(gp_Ax2.getPerpendicularYDir(theV));
        vxdir = new gp_Dir(gp_Ax2.getPerpendicularXDir(theV));
    }

    public gp_Ax3(gp_Dir.D theV) : this(new gp_Pnt(0.0, 0.0, 0.0), theV) { }

    // Mutators (internal: see docs/translation-rules.md)

    internal void XReverse() => vxdir.Reverse();

    internal void YReverse() => vydir.Reverse();

    internal void ZReverse() => axis.Reverse();

    internal void SetAxis(in gp_Ax1 theA1)
    {
        axis.SetLocation(theA1.Location());
        SetDirection(theA1.Direction());
    }

    internal void SetDirection(in gp_Dir theV)
    {
        double aDot = theV.Dot(vxdir);
        if (1.0 - Math.Abs(aDot) <= Precision.Angular())
        {
            if (aDot > 0)
            {
                vxdir = vydir;
                vydir = axis.Direction();
            }
            else
            {
                vxdir = axis.Direction();
            }

            axis.SetDirection(theV);
        }
        else
        {
            bool direct = Direct();
            axis.SetDirection(theV);
            vxdir = theV.CrossCrossed(vxdir, theV);
            if (direct)
            {
                vydir = theV.Crossed(vxdir);
            }
            else
            {
                vydir = vxdir.Crossed(theV);
            }
        }
    }

    internal void SetLocation(in gp_Pnt theP) => axis.SetLocation(theP);

    internal void SetXDirection(in gp_Dir theVx)
    {
        double aDot = theVx.Dot(axis.Direction());
        if (1.0 - Math.Abs(aDot) <= Precision.Angular())
        {
            if (aDot > 0)
            {
                axis.SetDirection(vxdir);
                vydir = -vydir;
            }
            else
            {
                axis.SetDirection(vxdir);
            }

            vxdir = theVx;
        }
        else
        {
            bool direct = Direct();
            vxdir = axis.Direction().CrossCrossed(theVx, axis.Direction());
            if (direct)
            {
                vydir = axis.Direction().Crossed(vxdir);
            }
            else
            {
                vydir = vxdir.Crossed(axis.Direction());
            }
        }
    }

    internal void SetYDirection(in gp_Dir theVy)
    {
        double aDot = theVy.Dot(axis.Direction());
        if (1.0 - Math.Abs(aDot) <= Precision.Angular())
        {
            if (aDot > 0)
            {
                axis.SetDirection(vydir);
                vxdir = -vxdir;
            }
            else
            {
                axis.SetDirection(vydir);
            }

            vydir = theVy;
        }
        else
        {
            bool isDirect = Direct();
            vxdir = theVy.Crossed(axis.Direction());
            vydir = axis.Direction().Crossed(vxdir);
            if (!isDirect)
            {
                vxdir.Reverse();
            }
        }
    }

    internal void Mirror(in gp_Pnt P)
    {
        axis.Mirror(P);
        vxdir.Reverse();
        vydir.Reverse();
    }

    internal void Mirror(in gp_Ax1 A1)
    {
        vydir.Mirror(A1);
        vxdir.Mirror(A1);
        axis.Mirror(A1);
    }

    internal void Mirror(in gp_Ax2 A2)
    {
        vydir.Mirror(A2);
        vxdir.Mirror(A2);
        axis.Mirror(A2);
    }

    internal void Rotate(in gp_Ax1 theA1, double theAng)
    {
        axis.Rotate(theA1, theAng);
        vxdir.Rotate(theA1, theAng);
        vydir.Rotate(theA1, theAng);
    }

    internal void Scale(in gp_Pnt theP, double theS)
    {
        axis.Scale(theP, theS);
        if (theS < 0.0)
        {
            vxdir.Reverse();
            vydir.Reverse();
        }
    }

    internal void Transform(in gp_Trsf theT)
    {
        axis.Transform(theT);
        vxdir.Transform(theT);
        vydir.Transform(theT);
    }

    internal void Translate(in gp_Vec theV) => axis.Translate(theV);

    internal void Translate(in gp_Pnt theP1, in gp_Pnt theP2) => Translate(new gp_Vec(theP1, theP2));

    // Copies with a mutator applied

    public readonly gp_Ax3 XReversed()
    {
        gp_Ax3 aCopy = this;
        aCopy.XReverse();
        return aCopy;
    }

    public readonly gp_Ax3 YReversed()
    {
        gp_Ax3 aCopy = this;
        aCopy.YReverse();
        return aCopy;
    }

    public readonly gp_Ax3 ZReversed()
    {
        gp_Ax3 aCopy = this;
        aCopy.ZReverse();
        return aCopy;
    }

    public readonly gp_Ax3 WithAxis(in gp_Ax1 theA1)
    {
        gp_Ax3 aCopy = this;
        aCopy.SetAxis(theA1);
        return aCopy;
    }

    public readonly gp_Ax3 WithDirection(in gp_Dir theV)
    {
        gp_Ax3 aCopy = this;
        aCopy.SetDirection(theV);
        return aCopy;
    }

    public readonly gp_Ax3 WithLocation(in gp_Pnt theP)
    {
        gp_Ax3 aCopy = this;
        aCopy.SetLocation(theP);
        return aCopy;
    }

    public readonly gp_Ax3 WithXDirection(in gp_Dir theVx)
    {
        gp_Ax3 aCopy = this;
        aCopy.SetXDirection(theVx);
        return aCopy;
    }

    public readonly gp_Ax3 WithYDirection(in gp_Dir theVy)
    {
        gp_Ax3 aCopy = this;
        aCopy.SetYDirection(theVy);
        return aCopy;
    }

    // Queries

    public readonly double Angle(in gp_Ax3 theOther) => axis.Angle(theOther.axis);

    public readonly gp_Ax1 Axis() => axis;

    /// <summary>The right-handed system with the same location, X direction and (for a direct system) main direction.</summary>
    public readonly gp_Ax2 Ax2()
    {
        gp_Dir aZz = axis.Direction();
        if (!Direct())
        {
            aZz.Reverse();
        }

        return new gp_Ax2(axis.Location(), aZz, vxdir);
    }

    public readonly gp_Dir Direction() => axis.Direction();

    public readonly gp_Pnt Location() => axis.Location();

    public readonly gp_Dir XDirection() => vxdir;

    public readonly gp_Dir YDirection() => vydir;

    /// <summary>True for a right-handed system.</summary>
    public readonly bool Direct() => vxdir.Crossed(vydir).Dot(axis.Direction()) > 0.0;

    public readonly bool IsCoplanar(in gp_Ax3 theOther, double theLinearTolerance, double theAngularTolerance)
    {
        gp_Vec aVec = new(axis.Location(), theOther.axis.Location());
        double aD1 = new gp_Vec(axis.Direction()).Dot(aVec);
        if (aD1 < 0)
        {
            aD1 = -aD1;
        }

        double aD2 = new gp_Vec(theOther.axis.Direction()).Dot(aVec);
        if (aD2 < 0)
        {
            aD2 = -aD2;
        }

        return aD1 <= theLinearTolerance && aD2 <= theLinearTolerance && axis.IsParallel(theOther.axis, theAngularTolerance);
    }

    public readonly bool IsCoplanar(in gp_Ax1 theA1, double theLinearTolerance, double theAngularTolerance)
    {
        gp_Vec aVec = new(axis.Location(), theA1.Location());
        double aD1 = new gp_Vec(axis.Direction()).Dot(aVec);
        if (aD1 < 0)
        {
            aD1 = -aD1;
        }

        double aD2 = new gp_Vec(theA1.Direction()).Crossed(aVec).Magnitude();
        if (aD2 < 0)
        {
            aD2 = -aD2;
        }

        return aD1 <= theLinearTolerance && aD2 <= theLinearTolerance && axis.IsNormal(theA1, theAngularTolerance);
    }

    public readonly gp_Ax3 Mirrored(in gp_Pnt P)
    {
        gp_Ax3 Temp = this;
        Temp.Mirror(P);
        return Temp;
    }

    public readonly gp_Ax3 Mirrored(in gp_Ax1 A1)
    {
        gp_Ax3 Temp = this;
        Temp.Mirror(A1);
        return Temp;
    }

    public readonly gp_Ax3 Mirrored(in gp_Ax2 A2)
    {
        gp_Ax3 Temp = this;
        Temp.Mirror(A2);
        return Temp;
    }

    public readonly gp_Ax3 Rotated(in gp_Ax1 theA1, double theAng)
    {
        gp_Ax3 aTemp = this;
        aTemp.Rotate(theA1, theAng);
        return aTemp;
    }

    public readonly gp_Ax3 Scaled(in gp_Pnt theP, double theS)
    {
        gp_Ax3 aTemp = this;
        aTemp.Scale(theP, theS);
        return aTemp;
    }

    public readonly gp_Ax3 Transformed(in gp_Trsf theT)
    {
        gp_Ax3 aTemp = this;
        aTemp.Transform(theT);
        return aTemp;
    }

    public readonly gp_Ax3 Translated(in gp_Vec theV)
    {
        gp_Ax3 aTemp = this;
        aTemp.Translate(theV);
        return aTemp;
    }

    public readonly gp_Ax3 Translated(in gp_Pnt theP1, in gp_Pnt theP2) => Translated(new gp_Vec(theP1, theP2));

    public readonly bool Equals(gp_Ax3 other) => axis.Equals(other.axis) && vxdir.Equals(other.vxdir) && vydir.Equals(other.vydir);

    public override readonly bool Equals(object? obj) => obj is gp_Ax3 other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(axis, vxdir, vydir);

    public static bool operator ==(in gp_Ax3 theLeft, in gp_Ax3 theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Ax3 theLeft, in gp_Ax3 theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"gp_Ax3({axis.Location()}, {axis.Direction()}, {vxdir}, {vydir})";
}
