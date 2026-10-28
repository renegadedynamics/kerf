// Port of: src/FoundationClasses/TKMath/gp/gp_Pnt.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Pnt.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics.CodeAnalysis;
using static Occt.gp_TrsfForm;

namespace Occt;

/// <summary>A point in 3D space.</summary>
public partial struct gp_Pnt : IEquatable<gp_Pnt>
{
    private gp_XYZ coord;

    public gp_Pnt(in gp_XYZ theCoord) => coord = theCoord;

    public gp_Pnt(double theXp, double theYp, double theZp) => coord = new gp_XYZ(theXp, theYp, theZp);

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetCoord(int theIndex, double theXi) => coord.SetCoord(theIndex, theXi);

    internal void SetCoord(double theXp, double theYp, double theZp) => coord.SetCoord(theXp, theYp, theZp);

    internal void SetX(double theX) => coord.SetX(theX);

    internal void SetY(double theY) => coord.SetY(theY);

    internal void SetZ(double theZ) => coord.SetZ(theZ);

    internal void SetXYZ(in gp_XYZ theCoord) => coord = theCoord;

    [UnscopedRef]
    internal ref gp_XYZ ChangeCoord() => ref coord;

    internal void BaryCenter(double theAlpha, in gp_Pnt theP, double theBeta)
    {
        coord.SetLinearForm(theAlpha, coord, theBeta, theP.coord);
        coord.Divide(theAlpha + theBeta);
    }

    internal void Mirror(in gp_Pnt theP)
    {
        coord.Reverse();
        gp_XYZ XYZ = theP.coord;
        XYZ.Multiply(2.0);
        coord.Add(XYZ);
    }

    internal void Mirror(in gp_Ax1 theA1)
    {
        gp_Trsf T = new();
        T.SetMirror(theA1);
        T.Transforms(ref coord);
    }

    internal void Mirror(in gp_Ax2 theA2)
    {
        gp_Trsf T = new();
        T.SetMirror(theA2);
        T.Transforms(ref coord);
    }

    internal void Rotate(in gp_Ax1 theA1, double theAng)
    {
        gp_Trsf aT = new();
        aT.SetRotation(theA1, theAng);
        aT.Transforms(ref coord);
    }

    internal void Scale(in gp_Pnt theP, double theS)
    {
        gp_XYZ aXYZ = theP.coord;
        aXYZ.Multiply(1.0 - theS);
        coord.Multiply(theS);
        coord.Add(aXYZ);
    }

    internal void Transform(in gp_Trsf theT)
    {
        if (theT.Form() == gp_Identity)
        {
        }
        else if (theT.Form() == gp_Translation)
        {
            coord.Add(theT.TranslationPart());
        }
        else if (theT.Form() == gp_Scale)
        {
            coord.Multiply(theT.ScaleFactor());
            coord.Add(theT.TranslationPart());
        }
        else if (theT.Form() == gp_PntMirror)
        {
            coord.Reverse();
            coord.Add(theT.TranslationPart());
        }
        else
        {
            theT.Transforms(ref coord);
        }
    }

    internal void Translate(in gp_Vec theV) => coord.Add(theV.XYZ());

    internal void Translate(in gp_Pnt theP1, in gp_Pnt theP2)
    {
        coord.Add(theP2.coord);
        coord.Subtract(theP1.coord);
    }

    // Copies with a mutator applied

    public readonly gp_Pnt WithCoord(int theIndex, double theXi)
    {
        gp_Pnt aCopy = this;
        aCopy.SetCoord(theIndex, theXi);
        return aCopy;
    }

    public readonly gp_Pnt WithCoord(double theXp, double theYp, double theZp) => new(theXp, theYp, theZp);

    public readonly gp_Pnt WithX(double theX) => new(theX, coord.Y(), coord.Z());

    public readonly gp_Pnt WithY(double theY) => new(coord.X(), theY, coord.Z());

    public readonly gp_Pnt WithZ(double theZ) => new(coord.X(), coord.Y(), theZ);

    public readonly gp_Pnt WithXYZ(in gp_XYZ theCoord) => new(theCoord);

    /// <summary>The barycentre of this point with weight <paramref name="theAlpha"/> and <paramref name="theP"/> with weight <paramref name="theBeta"/>.</summary>
    public readonly gp_Pnt BaryCentered(double theAlpha, in gp_Pnt theP, double theBeta)
    {
        gp_Pnt aCopy = this;
        aCopy.BaryCenter(theAlpha, theP, theBeta);
        return aCopy;
    }

    // Queries

    public readonly double Coord(int theIndex) => coord.Coord(theIndex);

    public readonly void Coord(out double theXp, out double theYp, out double theZp) => coord.Coord(out theXp, out theYp, out theZp);

    public readonly double X() => coord.X();

    public readonly double Y() => coord.Y();

    public readonly double Z() => coord.Z();

    public readonly gp_XYZ XYZ() => coord;

    public readonly gp_XYZ Coord() => coord;

    public readonly bool IsEqual(in gp_Pnt theOther, double theLinearTolerance) => Distance(theOther) <= theLinearTolerance;

    public readonly double Distance(in gp_Pnt theOther) => Math.Sqrt(SquareDistance(theOther));

    public readonly double SquareDistance(in gp_Pnt theOther)
    {
        gp_XYZ aXYZ = theOther.coord;
        double aDx = coord.X() - aXYZ.X();
        double aDy = coord.Y() - aXYZ.Y();
        double aDz = coord.Z() - aXYZ.Z();
        return aDx * aDx + aDy * aDy + aDz * aDz;
    }

    public readonly gp_Pnt Mirrored(in gp_Pnt theP)
    {
        gp_Pnt Pres = this;
        Pres.Mirror(theP);
        return Pres;
    }

    public readonly gp_Pnt Mirrored(in gp_Ax1 theA1)
    {
        gp_Pnt P = this;
        P.Mirror(theA1);
        return P;
    }

    public readonly gp_Pnt Mirrored(in gp_Ax2 theA2)
    {
        gp_Pnt P = this;
        P.Mirror(theA2);
        return P;
    }

    public readonly gp_Pnt Rotated(in gp_Ax1 theA1, double theAng)
    {
        gp_Pnt aP = this;
        aP.Rotate(theA1, theAng);
        return aP;
    }

    public readonly gp_Pnt Scaled(in gp_Pnt theP, double theS)
    {
        gp_Pnt aPres = this;
        aPres.Scale(theP, theS);
        return aPres;
    }

    public readonly gp_Pnt Transformed(in gp_Trsf theT)
    {
        gp_Pnt aP = this;
        aP.Transform(theT);
        return aP;
    }

    public readonly gp_Pnt Translated(in gp_Vec theV)
    {
        gp_Pnt aP = this;
        aP.coord.Add(theV.XYZ());
        return aP;
    }

    public readonly gp_Pnt Translated(in gp_Pnt theP1, in gp_Pnt theP2)
    {
        gp_Pnt aP = this;
        aP.Translate(theP1, theP2);
        return aP;
    }

    public readonly bool Equals(gp_Pnt other) => coord.Equals(other.coord);

    public override readonly bool Equals(object? obj) => obj is gp_Pnt other && Equals(other);

    public override readonly int GetHashCode() => coord.GetHashCode();

    public static bool operator ==(in gp_Pnt theLeft, in gp_Pnt theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Pnt theLeft, in gp_Pnt theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => gp.Format("gp_Pnt", coord.X(), coord.Y(), coord.Z());
}
