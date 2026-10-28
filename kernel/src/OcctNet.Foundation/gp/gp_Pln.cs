// Port of: src/FoundationClasses/TKMath/gp/gp_Pln.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Pln.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A plane in 3D space, positioned by a coordinate system: the origin is a point of the plane, the X and Y
/// directions span it and the main direction is its normal. It is parameterised by
/// <c>P(U, V) = O + U * XDir + V * YDir</c>.
/// </summary>
/// <remarks><c>new gp_Pln()</c> is the XOY plane, as in OCCT; <c>default(gp_Pln)</c> is not valid.</remarks>
public partial struct gp_Pln : IEquatable<gp_Pln>
{
    private gp_Ax3 myPosition;

    public gp_Pln() => myPosition = new gp_Ax3();

    public gp_Pln(in gp_Ax3 theA3) => myPosition = theA3;

    /// <summary>The plane through <paramref name="theP"/> with normal <paramref name="theV"/>; the X direction is chosen.</summary>
    public gp_Pln(in gp_Pnt theP, in gp_Dir theV)
    {
        double A = theV.X();
        double B = theV.Y();
        double C = theV.Z();
        double anAbsA = Math.Abs(A);
        double anAbsB = Math.Abs(B);
        double anAbsC = Math.Abs(C);
        // To determine the X axis:
        // we require that the dot product Vx.V = 0.
        // and we search for the max(A,B,C) to perform the division.
        // one of the coordinates of the vector is zero.
        if (anAbsB <= anAbsA && anAbsB <= anAbsC)
        {
            if (anAbsA > anAbsC)
            {
                myPosition = new gp_Ax3(theP, theV, new gp_Dir(-C, 0.0, A));
            }
            else
            {
                myPosition = new gp_Ax3(theP, theV, new gp_Dir(C, 0.0, -A));
            }
        }
        else if (anAbsA <= anAbsB && anAbsA <= anAbsC)
        {
            if (anAbsB > anAbsC)
            {
                myPosition = new gp_Ax3(theP, theV, new gp_Dir(0.0, -C, B));
            }
            else
            {
                myPosition = new gp_Ax3(theP, theV, new gp_Dir(0.0, C, -B));
            }
        }
        else
        {
            if (anAbsA > anAbsB)
            {
                myPosition = new gp_Ax3(theP, theV, new gp_Dir(-B, A, 0.0));
            }
            else
            {
                myPosition = new gp_Ax3(theP, theV, new gp_Dir(B, -A, 0.0));
            }
        }
    }

    /// <summary>The plane <c>A * X + B * Y + C * Z + D = 0</c>.</summary>
    /// <exception cref="Standard_ConstructionError">A, B and C are all zero.</exception>
    public gp_Pln(double theA, double theB, double theC, double theD)
    {
        double anAbsA = Math.Abs(theA);
        double anAbsB = Math.Abs(theB);
        double anAbsC = Math.Abs(theC);
        if (anAbsB <= anAbsA && anAbsB <= anAbsC)
        {
            if (anAbsA > anAbsC)
            {
                myPosition = new gp_Ax3(new gp_Pnt(-theD / theA, 0.0, 0.0), new gp_Dir(theA, theB, theC), new gp_Dir(-theC, 0.0, theA));
            }
            else
            {
                myPosition = new gp_Ax3(new gp_Pnt(0.0, 0.0, -theD / theC), new gp_Dir(theA, theB, theC), new gp_Dir(theC, 0.0, -theA));
            }
        }
        else if (anAbsA <= anAbsB && anAbsA <= anAbsC)
        {
            if (anAbsB > anAbsC)
            {
                myPosition = new gp_Ax3(new gp_Pnt(0.0, -theD / theB, 0.0), new gp_Dir(theA, theB, theC), new gp_Dir(0.0, -theC, theB));
            }
            else
            {
                myPosition = new gp_Ax3(new gp_Pnt(0.0, 0.0, -theD / theC), new gp_Dir(theA, theB, theC), new gp_Dir(0.0, theC, -theB));
            }
        }
        else
        {
            if (anAbsA > anAbsB)
            {
                myPosition = new gp_Ax3(new gp_Pnt(-theD / theA, 0.0, 0.0), new gp_Dir(theA, theB, theC), new gp_Dir(-theB, theA, 0.0));
            }
            else
            {
                myPosition = new gp_Ax3(new gp_Pnt(0.0, -theD / theB, 0.0), new gp_Dir(theA, theB, theC), new gp_Dir(theB, -theA, 0.0));
            }
        }
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetAxis(in gp_Ax1 theA1) => myPosition.SetAxis(theA1);

    internal void SetLocation(in gp_Pnt theLoc) => myPosition.SetLocation(theLoc);

    internal void SetPosition(in gp_Ax3 theA3) => myPosition = theA3;

    /// <summary>Reverses the U parametrisation (the X direction), which reverses the normal of an indirect system.</summary>
    internal void UReverse() => myPosition.XReverse();

    /// <summary>Reverses the V parametrisation (the Y direction).</summary>
    internal void VReverse() => myPosition.YReverse();

    internal void Mirror(in gp_Pnt theP) => myPosition.Mirror(theP);

    internal void Mirror(in gp_Ax1 theA1) => myPosition.Mirror(theA1);

    internal void Mirror(in gp_Ax2 theA2) => myPosition.Mirror(theA2);

    internal void Rotate(in gp_Ax1 theA1, double theAng) => myPosition.Rotate(theA1, theAng);

    internal void Scale(in gp_Pnt theP, double theS) => myPosition.Scale(theP, theS);

    internal void Transform(in gp_Trsf theT) => myPosition.Transform(theT);

    internal void Translate(in gp_Vec theV) => myPosition.Translate(theV);

    internal void Translate(in gp_Pnt theP1, in gp_Pnt theP2) => myPosition.Translate(theP1, theP2);

    // Copies with a mutator applied

    public readonly gp_Pln WithAxis(in gp_Ax1 theA1)
    {
        gp_Pln aCopy = this;
        aCopy.SetAxis(theA1);
        return aCopy;
    }

    public readonly gp_Pln WithLocation(in gp_Pnt theLoc)
    {
        gp_Pln aCopy = this;
        aCopy.SetLocation(theLoc);
        return aCopy;
    }

    public readonly gp_Pln WithPosition(in gp_Ax3 theA3) => new(theA3);

    public readonly gp_Pln UReversed()
    {
        gp_Pln aCopy = this;
        aCopy.UReverse();
        return aCopy;
    }

    public readonly gp_Pln VReversed()
    {
        gp_Pln aCopy = this;
        aCopy.VReverse();
        return aCopy;
    }

    // Queries

    /// <summary>
    /// The coefficients of the plane's equation <c>A * X + B * Y + C * Z + D = 0</c>, with (A, B, C) the normal of
    /// a direct system (reversed for an indirect one).
    /// </summary>
    public readonly void Coefficients(out double theA, out double theB, out double theC, out double theD)
    {
        gp_Dir aDir = myPosition.Direction();
        if (myPosition.Direct())
        {
            theA = aDir.X();
            theB = aDir.Y();
            theC = aDir.Z();
        }
        else
        {
            theA = -aDir.X();
            theB = -aDir.Y();
            theC = -aDir.Z();
        }

        gp_Pnt aP = myPosition.Location();
        theD = -(theA * aP.X() + theB * aP.Y() + theC * aP.Z());
    }

    /// <summary>True when the coordinate system is right-handed.</summary>
    public readonly bool Direct() => myPosition.Direct();

    /// <summary>The normal axis through the plane's origin.</summary>
    public readonly gp_Ax1 Axis() => myPosition.Axis();

    public readonly gp_Pnt Location() => myPosition.Location();

    public readonly gp_Ax3 Position() => myPosition;

    public readonly double Distance(in gp_Pnt theP) => Math.Abs(SignedDistance(theP));

    /// <summary>The distance to a line parallel to the plane; 0 when the line is not parallel.</summary>
    public readonly double Distance(in gp_Lin theL) => Math.Abs(SignedDistance(theL));

    /// <summary>The distance to a parallel plane; 0 when the planes are not parallel.</summary>
    public readonly double Distance(in gp_Pln theOther) => Math.Abs(SignedDistance(theOther));

    /// <summary>The signed distance, positive on the side the normal points to.</summary>
    public readonly double SignedDistance(in gp_Pnt theP)
    {
        gp_Pnt aLoc = myPosition.Location();
        gp_Dir aDir = myPosition.Direction();
        return (aDir.X() * (theP.X() - aLoc.X()) + aDir.Y() * (theP.Y() - aLoc.Y())
                + aDir.Z() * (theP.Z() - aLoc.Z()));
    }

    public readonly double SignedDistance(in gp_Lin theL)
    {
        if (!myPosition.Direction().IsNormal(theL.Direction(), gp.Resolution()))
        {
            return 0.0;
        }

        return SignedDistance(theL.Location());
    }

    public readonly double SignedDistance(in gp_Pln theOther)
    {
        if (!myPosition.Direction().IsParallel(theOther.myPosition.Direction(), gp.Resolution()))
        {
            return 0.0;
        }

        return SignedDistance(theOther.myPosition.Location());
    }

    public readonly double SquareDistance(in gp_Pnt theP)
    {
        double aD = Distance(theP);
        return aD * aD;
    }

    public readonly double SquareDistance(in gp_Lin theL)
    {
        double aD = Distance(theL);
        return aD * aD;
    }

    public readonly double SquareDistance(in gp_Pln theOther)
    {
        double aD = Distance(theOther);
        return aD * aD;
    }

    /// <summary>The X axis: the U parameter direction through the origin.</summary>
    public readonly gp_Ax1 XAxis() => new(myPosition.Location(), myPosition.XDirection());

    /// <summary>The Y axis: the V parameter direction through the origin.</summary>
    public readonly gp_Ax1 YAxis() => new(myPosition.Location(), myPosition.YDirection());

    public readonly bool Contains(in gp_Pnt theP, double theLinearTolerance) => Distance(theP) <= theLinearTolerance;

    public readonly bool Contains(in gp_Lin theL, double theLinearTolerance, double theAngularTolerance)
        => Contains(theL.Location(), theLinearTolerance)
           && myPosition.Direction().IsNormal(theL.Direction(), theAngularTolerance);

    public readonly gp_Pln Mirrored(in gp_Pnt theP) => new(myPosition.Mirrored(theP));

    public readonly gp_Pln Mirrored(in gp_Ax1 theA1) => new(myPosition.Mirrored(theA1));

    public readonly gp_Pln Mirrored(in gp_Ax2 theA2) => new(myPosition.Mirrored(theA2));

    public readonly gp_Pln Rotated(in gp_Ax1 theA1, double theAng)
    {
        gp_Pln aPl = this;
        aPl.myPosition.Rotate(theA1, theAng);
        return aPl;
    }

    public readonly gp_Pln Scaled(in gp_Pnt theP, double theS)
    {
        gp_Pln aPl = this;
        aPl.myPosition.Scale(theP, theS);
        return aPl;
    }

    public readonly gp_Pln Transformed(in gp_Trsf theT)
    {
        gp_Pln aPl = this;
        aPl.myPosition.Transform(theT);
        return aPl;
    }

    public readonly gp_Pln Translated(in gp_Vec theV)
    {
        gp_Pln aPl = this;
        aPl.myPosition.Translate(theV);
        return aPl;
    }

    public readonly gp_Pln Translated(in gp_Pnt theP1, in gp_Pnt theP2)
    {
        gp_Pln aPl = this;
        aPl.myPosition.Translate(theP1, theP2);
        return aPl;
    }

    public readonly bool Equals(gp_Pln other) => myPosition.Equals(other.myPosition);

    public override readonly bool Equals(object? obj) => obj is gp_Pln other && Equals(other);

    public override readonly int GetHashCode() => myPosition.GetHashCode();

    public static bool operator ==(in gp_Pln theLeft, in gp_Pln theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Pln theLeft, in gp_Pln theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"gp_Pln({myPosition})";
}
