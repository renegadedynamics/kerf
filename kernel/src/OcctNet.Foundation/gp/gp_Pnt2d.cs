// Port of: src/FoundationClasses/TKMath/gp/gp_Pnt2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Pnt2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics.CodeAnalysis;
using static Occt.gp_TrsfForm;

namespace Occt;

/// <summary>A point in 2D space.</summary>
public partial struct gp_Pnt2d : IEquatable<gp_Pnt2d>
{
    private gp_XY coord;

    public gp_Pnt2d(in gp_XY theCoord) => coord = theCoord;

    public gp_Pnt2d(double theXp, double theYp) => coord = new gp_XY(theXp, theYp);

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetCoord(int theIndex, double theXi) => coord.SetCoord(theIndex, theXi);

    internal void SetCoord(double theXp, double theYp) => coord.SetCoord(theXp, theYp);

    internal void SetX(double theX) => coord.SetX(theX);

    internal void SetY(double theY) => coord.SetY(theY);

    internal void SetXY(in gp_XY theCoord) => coord = theCoord;

    [UnscopedRef]
    internal ref gp_XY ChangeCoord() => ref coord;

    internal void Mirror(in gp_Pnt2d P)
    {
        coord.Reverse();
        gp_XY XY = P.coord;
        XY.Multiply(2.0);
        coord.Add(XY);
    }

    internal void Mirror(in gp_Ax2d A)
    {
        gp_Trsf2d T = new();
        T.SetMirror(A);
        T.Transforms(ref coord);
    }

    internal void Rotate(in gp_Pnt2d theP, double theAng)
    {
        gp_Trsf2d aT = new();
        aT.SetRotation(theP, theAng);
        aT.Transforms(ref coord);
    }

    internal void Scale(in gp_Pnt2d theP, double theS)
    {
        gp_XY aXY = theP.coord;
        aXY.Multiply(1.0 - theS);
        coord.Multiply(theS);
        coord.Add(aXY);
    }

    internal void Transform(in gp_Trsf2d T)
    {
        if (T.Form() == gp_Identity)
        {
        }
        else if (T.Form() == gp_Translation)
        {
            coord.Add(T.TranslationPart());
        }
        else if (T.Form() == gp_Scale)
        {
            coord.Multiply(T.ScaleFactor());
            coord.Add(T.TranslationPart());
        }
        else if (T.Form() == gp_PntMirror)
        {
            coord.Reverse();
            coord.Add(T.TranslationPart());
        }
        else
        {
            T.Transforms(ref coord);
        }
    }

    internal void Translate(in gp_Vec2d theV) => coord.Add(theV.XY());

    internal void Translate(in gp_Pnt2d theP1, in gp_Pnt2d theP2)
    {
        coord.Add(theP2.coord);
        coord.Subtract(theP1.coord);
    }

    // Copies with a mutator applied

    public readonly gp_Pnt2d WithCoord(int theIndex, double theXi)
    {
        gp_Pnt2d aCopy = this;
        aCopy.SetCoord(theIndex, theXi);
        return aCopy;
    }

    public readonly gp_Pnt2d WithCoord(double theXp, double theYp) => new(theXp, theYp);

    public readonly gp_Pnt2d WithX(double theX) => new(theX, coord.Y());

    public readonly gp_Pnt2d WithY(double theY) => new(coord.X(), theY);

    public readonly gp_Pnt2d WithXY(in gp_XY theCoord) => new(theCoord);

    // Queries

    public readonly double Coord(int theIndex) => coord.Coord(theIndex);

    public readonly void Coord(out double theXp, out double theYp) => coord.Coord(out theXp, out theYp);

    public readonly double X() => coord.X();

    public readonly double Y() => coord.Y();

    public readonly gp_XY XY() => coord;

    public readonly gp_XY Coord() => coord;

    public readonly bool IsEqual(in gp_Pnt2d theOther, double theLinearTolerance) => Distance(theOther) <= theLinearTolerance;

    public readonly double Distance(in gp_Pnt2d theOther)
    {
        gp_XY aXY = theOther.coord;
        double aX = coord.X() - aXY.X();
        double aY = coord.Y() - aXY.Y();
        return Math.Sqrt(aX * aX + aY * aY);
    }

    public readonly double SquareDistance(in gp_Pnt2d theOther)
    {
        gp_XY aXY = theOther.coord;
        double aX = coord.X() - aXY.X();
        double aY = coord.Y() - aXY.Y();
        return (aX * aX + aY * aY);
    }

    public readonly gp_Pnt2d Mirrored(in gp_Pnt2d P)
    {
        gp_Pnt2d Pres = this;
        Pres.Mirror(P);
        return Pres;
    }

    public readonly gp_Pnt2d Mirrored(in gp_Ax2d A)
    {
        gp_Pnt2d P = this;
        P.Mirror(A);
        return P;
    }

    public readonly gp_Pnt2d Rotated(in gp_Pnt2d theP, double theAng)
    {
        gp_Pnt2d aPres = this;
        aPres.Rotate(theP, theAng);
        return aPres;
    }

    public readonly gp_Pnt2d Scaled(in gp_Pnt2d theP, double theS)
    {
        gp_Pnt2d aPres = this;
        aPres.Scale(theP, theS);
        return aPres;
    }

    public readonly gp_Pnt2d Transformed(in gp_Trsf2d theT)
    {
        gp_Pnt2d aPres = this;
        aPres.Transform(theT);
        return aPres;
    }

    public readonly gp_Pnt2d Translated(in gp_Vec2d theV)
    {
        gp_Pnt2d aP = this;
        aP.coord.Add(theV.XY());
        return aP;
    }

    public readonly gp_Pnt2d Translated(in gp_Pnt2d theP1, in gp_Pnt2d theP2)
    {
        gp_Pnt2d aP = this;
        aP.Translate(theP1, theP2);
        return aP;
    }

    public readonly bool Equals(gp_Pnt2d other) => coord.Equals(other.coord);

    public override readonly bool Equals(object? obj) => obj is gp_Pnt2d other && Equals(other);

    public override readonly int GetHashCode() => coord.GetHashCode();

    public static bool operator ==(in gp_Pnt2d theLeft, in gp_Pnt2d theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Pnt2d theLeft, in gp_Pnt2d theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => gp.Format("gp_Pnt2d", coord.X(), coord.Y());
}
