// Port of: src/FoundationClasses/TKMath/gp/gp_Dir2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Dir2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gp_TrsfForm;

namespace Occt;

/// <summary>A unit vector in 2D space.</summary>
/// <remarks><c>new gp_Dir2d()</c> is (1, 0), as in OCCT; <c>default(gp_Dir2d)</c> is all zeros and not a valid direction.</remarks>
public partial struct gp_Dir2d : IEquatable<gp_Dir2d>
{
    /// <summary>The four directions along the axes of the global system.</summary>
    public enum D
    {
        /// <summary>(1, 0)</summary>
        X,
        /// <summary>(0, 1)</summary>
        Y,
        /// <summary>(-1, 0)</summary>
        NX,
        /// <summary>(0, -1)</summary>
        NY,
    }

    private gp_XY coord;

    public gp_Dir2d() => coord = new gp_XY(1.0, 0.0);

    public gp_Dir2d(D theDir)
        => coord = new gp_XY(theDir == D.X ? 1.0 : theDir == D.NX ? -1.0 : 0.0,
                             theDir == D.Y ? 1.0 : theDir == D.NY ? -1.0 : 0.0);

    public gp_Dir2d(in gp_Vec2d theV) : this(theV.XY()) { }

    public gp_Dir2d(in gp_XY theXY) : this(theXY.X(), theXY.Y()) { }

    public gp_Dir2d(double theXv, double theYv)
    {
        coord = default;
        double aSqMod = theXv * theXv + theYv * theYv;
        if (aSqMod >= (1.0 - gp.Resolution()) && aSqMod <= (1.0 + gp.Resolution()))
        {
            coord.SetCoord(theXv, theYv);
            return;
        }

        Standard_ConstructionError_Raise_if(aSqMod <= gp.Resolution() * gp.Resolution(), "gp_Dir2d() - input vector has zero norm");
        double aD = Math.Sqrt(aSqMod);
        coord.SetCoord(theXv / aD, theYv / aD);
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetCoord(int theIndex, double theXi)
    {
        Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > 2, "gp_Dir2d::SetCoord() - index is out of range [1, 2]");
        if (theIndex == 1)
        {
            SetX(theXi);
        }
        else
        {
            SetY(theXi);
        }
    }

    internal void SetCoord(double theXv, double theYv)
    {
        double aSqMod = theXv * theXv + theYv * theYv;
        if (aSqMod >= (1.0 - gp.Resolution()) && aSqMod <= (1.0 + gp.Resolution()))
        {
            coord.SetCoord(theXv, theYv);
            return;
        }

        Standard_ConstructionError_Raise_if(aSqMod <= gp.Resolution() * gp.Resolution(), "gp_Dir2d::SetCoord() - result vector has zero norm");
        double aD = Math.Sqrt(aSqMod);
        coord.SetCoord(theXv / aD, theYv / aD);
    }

    internal void SetX(double theX) => SetCoord(theX, coord.Y());

    internal void SetY(double theY) => SetCoord(coord.X(), theY);

    internal void SetXY(in gp_XY theXY) => SetCoord(theXY.X(), theXY.Y());

    internal void Reverse() => coord.Reverse();

    internal void Mirror(in gp_Dir2d V)
    {
        gp_XY XY = V.coord;
        double A = XY.X();
        double B = XY.Y();
        double X = coord.X();
        double Y = coord.Y();
        double M1 = 2.0 * A * B;
        double XX = ((2.0 * A * A) - 1.0) * X + M1 * Y;
        double YY = M1 * X + ((2.0 * B * B) - 1.0) * Y;
        coord.SetCoord(XX, YY);
    }

    internal void Mirror(in gp_Ax2d A2)
    {
        gp_XY XY = A2.Direction().XY();
        double A = XY.X();
        double B = XY.Y();
        double X = coord.X();
        double Y = coord.Y();
        double M1 = 2.0 * A * B;
        double XX = ((2.0 * A * A) - 1.0) * X + M1 * Y;
        double YY = M1 * X + ((2.0 * B * B) - 1.0) * Y;
        coord.SetCoord(XX, YY);
    }

    internal void Rotate(double theAng)
    {
        gp_Trsf2d aT = new();
        aT.SetRotation(new gp_Pnt2d(0.0, 0.0), theAng);
        coord.Multiply(aT.HVectorialPart());
    }

    internal void Transform(in gp_Trsf2d T)
    {
        if (T.Form() == gp_Identity || T.Form() == gp_Translation)
        {
        }
        else if (T.Form() == gp_PntMirror)
        {
            coord.Reverse();
        }
        else if (T.Form() == gp_Scale)
        {
            if (T.ScaleFactor() < 0.0)
            {
                coord.Reverse();
            }
        }
        else
        {
            coord.Multiply(T.HVectorialPart());
            double D = coord.Modulus();
            coord.Divide(D);
            if (T.ScaleFactor() < 0.0)
            {
                coord.Reverse();
            }
        }
    }

    // Copies with a mutator applied

    public readonly gp_Dir2d WithCoord(int theIndex, double theXi)
    {
        gp_Dir2d aCopy = this;
        aCopy.SetCoord(theIndex, theXi);
        return aCopy;
    }

    public readonly gp_Dir2d WithCoord(double theXv, double theYv)
    {
        gp_Dir2d aCopy = this;
        aCopy.SetCoord(theXv, theYv);
        return aCopy;
    }

    public readonly gp_Dir2d WithX(double theX)
    {
        gp_Dir2d aCopy = this;
        aCopy.SetX(theX);
        return aCopy;
    }

    public readonly gp_Dir2d WithY(double theY)
    {
        gp_Dir2d aCopy = this;
        aCopy.SetY(theY);
        return aCopy;
    }

    public readonly gp_Dir2d WithXY(in gp_XY theXY)
    {
        gp_Dir2d aCopy = this;
        aCopy.SetXY(theXY);
        return aCopy;
    }

    // Queries

    public readonly double Coord(int theIndex) => coord.Coord(theIndex);

    public readonly void Coord(out double theXv, out double theYv) => coord.Coord(out theXv, out theYv);

    public readonly double X() => coord.X();

    public readonly double Y() => coord.Y();

    public readonly gp_XY XY() => coord;

    public readonly bool IsEqual(in gp_Dir2d theOther, double theAngularTolerance)
    {
        double anAng = Angle(theOther);
        if (anAng < 0)
        {
            anAng = -anAng;
        }

        return anAng <= theAngularTolerance;
    }

    public readonly bool IsNormal(in gp_Dir2d theOther, double theAngularTolerance)
    {
        double anAng = Angle(theOther);
        if (anAng < 0)
        {
            anAng = -anAng;
        }

        anAng = M_PI / 2.0 - anAng;
        if (anAng < 0)
        {
            anAng = -anAng;
        }

        return anAng <= theAngularTolerance;
    }

    public readonly bool IsOpposite(in gp_Dir2d theOther, double theAngularTolerance)
    {
        double anAng = Angle(theOther);
        if (anAng < 0)
        {
            anAng = -anAng;
        }

        return M_PI - anAng <= theAngularTolerance;
    }

    public readonly bool IsParallel(in gp_Dir2d theOther, double theAngularTolerance)
    {
        double anAng = Angle(theOther);
        if (anAng < 0)
        {
            anAng = -anAng;
        }

        return anAng <= theAngularTolerance || M_PI - anAng <= theAngularTolerance;
    }

    /// <summary>The signed angle from this direction to <paramref name="Other"/>, in ]-PI, PI].</summary>
    public readonly double Angle(in gp_Dir2d Other)
    {
        double Cosinus = coord.Dot(Other.coord);
        double Sinus = coord.Crossed(Other.coord);
        if (Cosinus > -0.70710678118655 && Cosinus < 0.70710678118655)
        {
            if (Sinus > 0.0)
            {
                return Math.Acos(Cosinus);
            }
            else
            {
                return -Math.Acos(Cosinus);
            }
        }
        else
        {
            if (Cosinus > 0.0)
            {
                return Math.Asin(Sinus);
            }
            else
            {
                if (Sinus > 0.0)
                {
                    return M_PI - Math.Asin(Sinus);
                }
                else
                {
                    return -M_PI - Math.Asin(Sinus);
                }
            }
        }
    }

    /// <summary>The scalar cross product.</summary>
    public readonly double Crossed(in gp_Dir2d theRight) => coord.Crossed(theRight.coord);

    public readonly double Dot(in gp_Dir2d theOther) => coord.Dot(theOther.coord);

    public readonly gp_Dir2d Reversed()
    {
        gp_Dir2d aV = this;
        aV.coord.Reverse();
        return aV;
    }

    public readonly gp_Dir2d Mirrored(in gp_Dir2d V)
    {
        gp_Dir2d Vres = this;
        Vres.Mirror(V);
        return Vres;
    }

    public readonly gp_Dir2d Mirrored(in gp_Ax2d A)
    {
        gp_Dir2d V = this;
        V.Mirror(A);
        return V;
    }

    public readonly gp_Dir2d Rotated(double theAng)
    {
        gp_Dir2d aV = this;
        aV.Rotate(theAng);
        return aV;
    }

    public readonly gp_Dir2d Transformed(in gp_Trsf2d theT)
    {
        gp_Dir2d aV = this;
        aV.Transform(theT);
        return aV;
    }

    public static gp_Dir2d operator -(in gp_Dir2d theV) => theV.Reversed();

    /// <summary>The dot product, as OCCT's <c>operator*</c>.</summary>
    public static double operator *(in gp_Dir2d theLeft, in gp_Dir2d theRight) => theLeft.Dot(theRight);

    /// <summary>The scalar cross product, as OCCT's <c>operator^</c>.</summary>
    public static double operator ^(in gp_Dir2d theLeft, in gp_Dir2d theRight) => theLeft.Crossed(theRight);

    public readonly bool Equals(gp_Dir2d other) => coord.Equals(other.coord);

    public override readonly bool Equals(object? obj) => obj is gp_Dir2d other && Equals(other);

    public override readonly int GetHashCode() => coord.GetHashCode();

    public static bool operator ==(in gp_Dir2d theLeft, in gp_Dir2d theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Dir2d theLeft, in gp_Dir2d theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => gp.Format("gp_Dir2d", coord.X(), coord.Y());
}
