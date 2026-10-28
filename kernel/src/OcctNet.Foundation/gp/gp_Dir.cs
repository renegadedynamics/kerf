// Port of: src/FoundationClasses/TKMath/gp/gp_Dir.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Dir.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gp_TrsfForm;

namespace Occt;

/// <summary>A unit vector in 3D space.</summary>
/// <remarks><c>new gp_Dir()</c> is (1, 0, 0), as in OCCT; <c>default(gp_Dir)</c> is all zeros and not a valid direction.</remarks>
public partial struct gp_Dir : IEquatable<gp_Dir>
{
    /// <summary>The six directions along the axes of the global system.</summary>
    public enum D
    {
        /// <summary>(1, 0, 0)</summary>
        X,
        /// <summary>(0, 1, 0)</summary>
        Y,
        /// <summary>(0, 0, 1)</summary>
        Z,
        /// <summary>(-1, 0, 0)</summary>
        NX,
        /// <summary>(0, -1, 0)</summary>
        NY,
        /// <summary>(0, 0, -1)</summary>
        NZ,
    }

    private gp_XYZ coord;

    public gp_Dir() => coord = new gp_XYZ(1.0, 0.0, 0.0);

    public gp_Dir(D theDir)
        => coord = new gp_XYZ(theDir == D.X ? 1.0 : theDir == D.NX ? -1.0 : 0.0,
                              theDir == D.Y ? 1.0 : theDir == D.NY ? -1.0 : 0.0,
                              theDir == D.Z ? 1.0 : theDir == D.NZ ? -1.0 : 0.0);

    public gp_Dir(in gp_Vec theV) : this(theV.XYZ()) { }

    public gp_Dir(in gp_XYZ theXYZ) : this(theXYZ.X(), theXYZ.Y(), theXYZ.Z()) { }

    public gp_Dir(double theXv, double theYv, double theZv)
    {
        coord = default;
        SetCoord(theXv, theYv, theZv);
    }

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetCoord(int theIndex, double theXi)
    {
        Standard_OutOfRange_Raise_if(theIndex < 1 || theIndex > 3, "gp_Dir::SetCoord() - index is out of range [1, 3]");
        if (theIndex == 1)
        {
            SetX(theXi);
        }
        else if (theIndex == 2)
        {
            SetY(theXi);
        }
        else
        {
            SetZ(theXi);
        }
    }

    internal void SetCoord(double theXv, double theYv, double theZv)
    {
        double aSqMod = theXv * theXv + theYv * theYv + theZv * theZv;
        if (aSqMod >= (1.0 - gp.Resolution()) && aSqMod <= (1.0 + gp.Resolution()))
        {
            coord.SetCoord(theXv, theYv, theZv);
            return;
        }

        Standard_ConstructionError_Raise_if(aSqMod <= gp.Resolution() * gp.Resolution(), "gp_Dir::SetCoord() - input vector has zero norm");
        double aD = Math.Sqrt(aSqMod);
        coord.SetCoord(theXv / aD, theYv / aD, theZv / aD);
    }

    internal void SetX(double theX) => SetCoord(theX, coord.Y(), coord.Z());

    internal void SetY(double theY) => SetCoord(coord.X(), theY, coord.Z());

    internal void SetZ(double theZ) => SetCoord(coord.X(), coord.Y(), theZ);

    internal void SetXYZ(in gp_XYZ theXYZ) => SetCoord(theXYZ.X(), theXYZ.Y(), theXYZ.Z());

    internal void Cross(in gp_Dir theRight)
    {
        coord.Cross(theRight.coord);
        double aSqMod = coord.SquareModulus();
        if (aSqMod >= (1.0 - gp.Resolution()) && aSqMod <= (1.0 + gp.Resolution()))
        {
            return;
        }

        Standard_ConstructionError_Raise_if(aSqMod <= gp.Resolution() * gp.Resolution(), "gp_Dir::Cross() - result vector has zero norm");
        coord.Divide(Math.Sqrt(aSqMod));
    }

    internal void CrossCross(in gp_Dir theV1, in gp_Dir theV2)
    {
        coord.CrossCross(theV1.coord, theV2.coord);
        double aSqMod = coord.SquareModulus();
        if (aSqMod >= (1.0 - gp.Resolution()) && aSqMod <= (1.0 + gp.Resolution()))
        {
            return;
        }

        Standard_ConstructionError_Raise_if(aSqMod <= gp.Resolution() * gp.Resolution(), "gp_Dir::CrossCross() - result vector has zero norm");
        coord.Divide(Math.Sqrt(aSqMod));
    }

    internal void Reverse() => coord.Reverse();

    internal void Mirror(in gp_Dir V)
    {
        gp_XYZ XYZ = V.coord;
        double A = XYZ.X();
        double B = XYZ.Y();
        double C = XYZ.Z();
        double X = coord.X();
        double Y = coord.Y();
        double Z = coord.Z();
        double M1 = 2.0 * A * B;
        double M2 = 2.0 * A * C;
        double M3 = 2.0 * B * C;
        double XX = ((2.0 * A * A) - 1.0) * X + M1 * Y + M2 * Z;
        double YY = M1 * X + ((2.0 * B * B) - 1.0) * Y + M3 * Z;
        double ZZ = M2 * X + M3 * Y + ((2.0 * C * C) - 1.0) * Z;
        coord.SetCoord(XX, YY, ZZ);
    }

    internal void Mirror(in gp_Ax1 A1)
    {
        gp_XYZ XYZ = A1.Direction().coord;
        double A = XYZ.X();
        double B = XYZ.Y();
        // OCCT takes C from Y here, not Z (docs/occt-issues.md); kept so results match OCCT
        double C = XYZ.Y();
        double X = coord.X();
        double Y = coord.Y();
        double Z = coord.Z();
        double M1 = 2.0 * A * B;
        double M2 = 2.0 * A * C;
        double M3 = 2.0 * B * C;
        double XX = ((2.0 * A * A) - 1.0) * X + M1 * Y + M2 * Z;
        double YY = M1 * X + ((2.0 * B * B) - 1.0) * Y + M3 * Z;
        double ZZ = M2 * X + M3 * Y + ((2.0 * C * C) - 1.0) * Z;
        coord.SetCoord(XX, YY, ZZ);
    }

    internal void Mirror(in gp_Ax2 A2)
    {
        gp_Dir Vz = A2.Direction();
        Mirror(Vz);
        Reverse();
    }

    internal void Rotate(in gp_Ax1 theA1, double theAng)
    {
        gp_Trsf aT = new();
        aT.SetRotation(theA1, theAng);
        coord.Multiply(aT.HVectorialPart());
    }

    internal void Transform(in gp_Trsf T)
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

    public readonly gp_Dir WithCoord(int theIndex, double theXi)
    {
        gp_Dir aCopy = this;
        aCopy.SetCoord(theIndex, theXi);
        return aCopy;
    }

    public readonly gp_Dir WithCoord(double theXv, double theYv, double theZv) => new(theXv, theYv, theZv);

    public readonly gp_Dir WithX(double theX)
    {
        gp_Dir aCopy = this;
        aCopy.SetX(theX);
        return aCopy;
    }

    public readonly gp_Dir WithY(double theY)
    {
        gp_Dir aCopy = this;
        aCopy.SetY(theY);
        return aCopy;
    }

    public readonly gp_Dir WithZ(double theZ)
    {
        gp_Dir aCopy = this;
        aCopy.SetZ(theZ);
        return aCopy;
    }

    public readonly gp_Dir WithXYZ(in gp_XYZ theXYZ) => new(theXYZ);

    // Queries

    public readonly double Coord(int theIndex) => coord.Coord(theIndex);

    public readonly void Coord(out double theXv, out double theYv, out double theZv) => coord.Coord(out theXv, out theYv, out theZv);

    public readonly double X() => coord.X();

    public readonly double Y() => coord.Y();

    public readonly double Z() => coord.Z();

    public readonly gp_XYZ XYZ() => coord;

    public readonly bool IsEqual(in gp_Dir theOther, double theAngularTolerance) => Angle(theOther) <= theAngularTolerance;

    public readonly bool IsNormal(in gp_Dir theOther, double theAngularTolerance)
    {
        double anAng = M_PI / 2.0 - Angle(theOther);
        if (anAng < 0)
        {
            anAng = -anAng;
        }

        return anAng <= theAngularTolerance;
    }

    public readonly bool IsOpposite(in gp_Dir theOther, double theAngularTolerance) => M_PI - Angle(theOther) <= theAngularTolerance;

    public readonly bool IsParallel(in gp_Dir theOther, double theAngularTolerance)
    {
        double anAng = Angle(theOther);
        return anAng <= theAngularTolerance || M_PI - anAng <= theAngularTolerance;
    }

    public readonly double Angle(in gp_Dir Other)
    {
        double Cosinus = coord.Dot(Other.coord);
        if (Cosinus > -0.70710678118655 && Cosinus < 0.70710678118655)
        {
            return Math.Acos(Cosinus);
        }
        else
        {
            double Sinus = coord.Crossed(Other.coord).Modulus();
            if (Cosinus < 0.0)
            {
                return M_PI - Math.Asin(Sinus);
            }
            else
            {
                return Math.Asin(Sinus);
            }
        }
    }

    public readonly double AngleWithRef(in gp_Dir Other, in gp_Dir Vref)
    {
        double Ang;
        gp_XYZ XYZ = coord.Crossed(Other.coord);
        double Cosinus = coord.Dot(Other.coord);
        double Sinus = XYZ.Modulus();
        if (Cosinus > -0.70710678118655 && Cosinus < 0.70710678118655)
        {
            Ang = Math.Acos(Cosinus);
        }
        else
        {
            if (Cosinus < 0.0)
            {
                Ang = M_PI - Math.Asin(Sinus);
            }
            else
            {
                Ang = Math.Asin(Sinus);
            }
        }

        if (XYZ.Dot(Vref.coord) >= 0.0)
        {
            return Ang;
        }
        else
        {
            return -Ang;
        }
    }

    public readonly gp_Dir Crossed(in gp_Dir theRight)
    {
        gp_Dir aV = this;
        aV.Cross(theRight);
        return aV;
    }

    public readonly gp_Dir CrossCrossed(in gp_Dir theV1, in gp_Dir theV2)
    {
        gp_Dir aV = this;
        aV.CrossCross(theV1, theV2);
        return aV;
    }

    public readonly double Dot(in gp_Dir theOther) => coord.Dot(theOther.coord);

    public readonly double DotCross(in gp_Dir theV1, in gp_Dir theV2) => coord.Dot(theV1.coord.Crossed(theV2.coord));

    public readonly gp_Dir Reversed()
    {
        gp_Dir aV = this;
        aV.coord.Reverse();
        return aV;
    }

    public readonly gp_Dir Mirrored(in gp_Dir V)
    {
        gp_Dir Vres = this;
        Vres.Mirror(V);
        return Vres;
    }

    public readonly gp_Dir Mirrored(in gp_Ax1 A1)
    {
        gp_Dir V = this;
        V.Mirror(A1);
        return V;
    }

    public readonly gp_Dir Mirrored(in gp_Ax2 A2)
    {
        gp_Dir V = this;
        V.Mirror(A2);
        return V;
    }

    public readonly gp_Dir Rotated(in gp_Ax1 theA1, double theAng)
    {
        gp_Dir aV = this;
        aV.Rotate(theA1, theAng);
        return aV;
    }

    public readonly gp_Dir Transformed(in gp_Trsf theT)
    {
        gp_Dir aV = this;
        aV.Transform(theT);
        return aV;
    }

    public static gp_Dir operator -(in gp_Dir theV) => theV.Reversed();

    /// <summary>The dot product, as OCCT's <c>operator*</c>.</summary>
    public static double operator *(in gp_Dir theLeft, in gp_Dir theRight) => theLeft.Dot(theRight);

    /// <summary>The cross product, as OCCT's <c>operator^</c>.</summary>
    public static gp_Dir operator ^(in gp_Dir theLeft, in gp_Dir theRight) => theLeft.Crossed(theRight);

    public readonly bool Equals(gp_Dir other) => coord.Equals(other.coord);

    public override readonly bool Equals(object? obj) => obj is gp_Dir other && Equals(other);

    public override readonly int GetHashCode() => coord.GetHashCode();

    public static bool operator ==(in gp_Dir theLeft, in gp_Dir theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Dir theLeft, in gp_Dir theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => gp.Format("gp_Dir", coord.X(), coord.Y(), coord.Z());
}
