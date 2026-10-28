// Port of: src/FoundationClasses/TKMath/gp/gp_Vec2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Vec2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gp_TrsfForm;

namespace Occt;

/// <summary>A vector in 2D space.</summary>
public partial struct gp_Vec2d : IEquatable<gp_Vec2d>
{
    private gp_XY coord;

    public gp_Vec2d(in gp_Dir2d theV) => coord = theV.XY();

    public gp_Vec2d(in gp_XY theCoord) => coord = theCoord;

    public gp_Vec2d(double theXv, double theYv) => coord = new gp_XY(theXv, theYv);

    /// <summary>The vector from <paramref name="theP1"/> to <paramref name="theP2"/>.</summary>
    public gp_Vec2d(in gp_Pnt2d theP1, in gp_Pnt2d theP2) => coord = theP2.XY().Subtracted(theP1.XY());

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetCoord(int theIndex, double theXi) => coord.SetCoord(theIndex, theXi);

    internal void SetCoord(double theXv, double theYv) => coord.SetCoord(theXv, theYv);

    internal void SetX(double theX) => coord.SetX(theX);

    internal void SetY(double theY) => coord.SetY(theY);

    internal void SetXY(in gp_XY theCoord) => coord = theCoord;

    internal void Add(in gp_Vec2d theOther) => coord.Add(theOther.coord);

    internal void Divide(double theScalar) => coord.Divide(theScalar);

    internal void Multiply(double theScalar) => coord.Multiply(theScalar);

    internal void Normalize()
    {
        double aD = coord.Modulus();
        Standard_ConstructionError_Raise_if(aD <= gp.Resolution(), "gp_Vec2d::Normalize() - vector has zero norm");
        coord.Divide(aD);
    }

    internal void Reverse() => coord.Reverse();

    internal void Subtract(in gp_Vec2d theRight) => coord.Subtract(theRight.coord);

    internal void SetLinearForm(double theA1, in gp_Vec2d theV1, double theA2, in gp_Vec2d theV2, in gp_Vec2d theV3)
        => coord.SetLinearForm(theA1, theV1.coord, theA2, theV2.coord, theV3.coord);

    internal void SetLinearForm(double theA1, in gp_Vec2d theV1, double theA2, in gp_Vec2d theV2)
        => coord.SetLinearForm(theA1, theV1.coord, theA2, theV2.coord);

    internal void SetLinearForm(double theA1, in gp_Vec2d theV1, in gp_Vec2d theV2) => coord.SetLinearForm(theA1, theV1.coord, theV2.coord);

    internal void SetLinearForm(in gp_Vec2d theV1, in gp_Vec2d theV2) => coord.SetLinearForm(theV1.coord, theV2.coord);

    internal void Mirror(in gp_Vec2d theV)
    {
        double aMagnitude = theV.coord.Modulus();
        if (aMagnitude > gp.Resolution())
        {
            gp_XY aMirrorVecXY = theV.coord;
            double aOrigX = coord.X();
            double aOrigY = coord.Y();

            double aNormDirX = aMirrorVecXY.X() / aMagnitude;
            double aNormDirY = aMirrorVecXY.Y() / aMagnitude;

            double aCrossTerm = 2.0 * aNormDirX * aNormDirY;
            double aXXTerm = 2.0 * aNormDirX * aNormDirX - 1.0;
            double aYYTerm = 2.0 * aNormDirY * aNormDirY - 1.0;

            coord.SetX(aXXTerm * aOrigX + aCrossTerm * aOrigY);
            coord.SetY(aCrossTerm * aOrigX + aYYTerm * aOrigY);
        }
    }

    internal void Mirror(in gp_Ax2d theA1)
    {
        gp_XY aDirectionXY = theA1.Direction().XY();
        double aOrigX = coord.X();
        double aOrigY = coord.Y();
        double aDirX = aDirectionXY.X();
        double aDirY = aDirectionXY.Y();

        double aCrossTerm = 2.0 * aDirX * aDirY;
        double aXXTerm = 2.0 * aDirX * aDirX - 1.0;
        double aYYTerm = 2.0 * aDirY * aDirY - 1.0;

        coord.SetX(aXXTerm * aOrigX + aCrossTerm * aOrigY);
        coord.SetY(aCrossTerm * aOrigX + aYYTerm * aOrigY);
    }

    internal void Rotate(double theAng)
    {
        gp_Trsf2d aT = new();
        aT.SetRotation(new gp_Pnt2d(0.0, 0.0), theAng);
        coord.Multiply(aT.VectorialPart());
    }

    internal void Scale(double theS) => coord.Multiply(theS);

    internal void Transform(in gp_Trsf2d theT)
    {
        switch (theT.Form())
        {
            case gp_Identity:
            case gp_Translation:
                break;
            case gp_PntMirror:
                coord.Reverse();
                break;
            case gp_Scale:
                coord.Multiply(theT.ScaleFactor());
                break;
            default:
                coord.Multiply(theT.VectorialPart());
                break;
        }
    }

    // Copies with a mutator applied

    public readonly gp_Vec2d WithCoord(int theIndex, double theXi)
    {
        gp_Vec2d aCopy = this;
        aCopy.SetCoord(theIndex, theXi);
        return aCopy;
    }

    public readonly gp_Vec2d WithCoord(double theXv, double theYv) => new(theXv, theYv);

    public readonly gp_Vec2d WithX(double theX) => new(theX, coord.Y());

    public readonly gp_Vec2d WithY(double theY) => new(coord.X(), theY);

    public readonly gp_Vec2d WithXY(in gp_XY theCoord) => new(theCoord);

    // Queries

    public readonly double Coord(int theIndex) => coord.Coord(theIndex);

    public readonly void Coord(out double theXv, out double theYv) => coord.Coord(out theXv, out theYv);

    public readonly double X() => coord.X();

    public readonly double Y() => coord.Y();

    public readonly gp_XY XY() => coord;

    /// <summary>True when the lengths differ by at most the linear tolerance and, if both are longer than it, the angle is at most the angular tolerance.</summary>
    public readonly bool IsEqual(in gp_Vec2d theOther, double theLinearTolerance, double theAngularTolerance)
    {
        double aNorm = Magnitude();
        double anOtherNorm = theOther.Magnitude();
        double aVal = Math.Abs(aNorm - anOtherNorm);

        bool isEqualLength = (aVal <= theLinearTolerance);

        if (aNorm > theLinearTolerance && anOtherNorm > theLinearTolerance)
        {
            double anAng = Math.Abs(Angle(theOther));
            return isEqualLength && (anAng <= theAngularTolerance);
        }

        return isEqualLength;
    }

    public readonly bool IsNormal(in gp_Vec2d theOther, double theAngularTolerance)
    {
        double anAng = Math.Abs(M_PI_2 - Math.Abs(Angle(theOther)));
        return !(anAng > theAngularTolerance);
    }

    public readonly bool IsOpposite(in gp_Vec2d theOther, double theAngularTolerance)
    {
        double anAng = Math.Abs(Angle(theOther));
        return M_PI - anAng <= theAngularTolerance;
    }

    public readonly bool IsParallel(in gp_Vec2d theOther, double theAngularTolerance)
    {
        double anAng = Math.Abs(Angle(theOther));
        return anAng <= theAngularTolerance || M_PI - anAng <= theAngularTolerance;
    }

    /// <summary>The signed angle from this vector to <paramref name="theOther"/>, in ]-PI, PI].</summary>
    /// <exception cref="gp_VectorWithNullMagnitude">Either vector's magnitude is at most <see cref="gp.Resolution"/>.</exception>
    public readonly double Angle(in gp_Vec2d theOther)
    {
        double aNorm = Magnitude();
        double anOtherNorm = theOther.Magnitude();
        if (aNorm <= gp.Resolution() || anOtherNorm <= gp.Resolution())
        {
            throw new gp_VectorWithNullMagnitude();
        }

        double aD = aNorm * anOtherNorm;
        double aCosinus = coord.Dot(theOther.coord) / aD;
        double aSinus = coord.Crossed(theOther.coord) / aD;

        const double aCOS_45_DEG = 0.70710678118654752440; // M_SQRT1_2

        if (aCosinus > -aCOS_45_DEG && aCosinus < aCOS_45_DEG)
        {
            return (aSinus > 0.0) ? Math.Acos(aCosinus) : -Math.Acos(aCosinus);
        }
        else
        {
            if (aCosinus > 0.0)
            {
                return Math.Asin(aSinus);
            }
            else
            {
                return (aSinus > 0.0) ? M_PI - Math.Asin(aSinus) : -M_PI - Math.Asin(aSinus);
            }
        }
    }

    public readonly double Magnitude() => coord.Modulus();

    public readonly double SquareMagnitude() => coord.SquareModulus();

    public readonly gp_Vec2d Added(in gp_Vec2d theOther)
    {
        gp_Vec2d aV = this;
        aV.coord.Add(theOther.coord);
        return aV;
    }

    /// <summary>The scalar cross product.</summary>
    public readonly double Crossed(in gp_Vec2d theRight) => coord.Crossed(theRight.coord);

    public readonly double CrossMagnitude(in gp_Vec2d theRight) => coord.CrossMagnitude(theRight.coord);

    public readonly double CrossSquareMagnitude(in gp_Vec2d theRight) => coord.CrossSquareMagnitude(theRight.coord);

    public readonly gp_Vec2d Divided(double theScalar)
    {
        gp_Vec2d aV = this;
        aV.coord.Divide(theScalar);
        return aV;
    }

    public readonly double Dot(in gp_Vec2d theOther) => coord.Dot(theOther.coord);

    /// <summary>The vector (Y, -X): this vector turned by -PI/2.</summary>
    public readonly gp_Vec2d GetNormal() => new(this.Y(), (-1) * this.X());

    public readonly gp_Vec2d Multiplied(double theScalar)
    {
        gp_Vec2d aV = this;
        aV.coord.Multiply(theScalar);
        return aV;
    }

    public readonly gp_Vec2d Normalized()
    {
        double aD = coord.Modulus();
        Standard_ConstructionError_Raise_if(aD <= gp.Resolution(), "gp_Vec2d::Normalized() - vector has zero norm");
        gp_Vec2d aV = this;
        aV.coord.Divide(aD);
        return aV;
    }

    public readonly gp_Vec2d Reversed()
    {
        gp_Vec2d aV = this;
        aV.coord.Reverse();
        return aV;
    }

    public readonly gp_Vec2d Subtracted(in gp_Vec2d theRight)
    {
        gp_Vec2d aV = this;
        aV.coord.Subtract(theRight.coord);
        return aV;
    }

    public readonly gp_Vec2d Mirrored(in gp_Vec2d theV)
    {
        gp_Vec2d aVres = this;
        aVres.Mirror(theV);
        return aVres;
    }

    public readonly gp_Vec2d Mirrored(in gp_Ax2d theA1)
    {
        gp_Vec2d Vres = this;
        Vres.Mirror(theA1);
        return Vres;
    }

    public readonly gp_Vec2d Rotated(double theAng)
    {
        gp_Vec2d aV = this;
        aV.Rotate(theAng);
        return aV;
    }

    public readonly gp_Vec2d Scaled(double theS)
    {
        gp_Vec2d aV = this;
        aV.coord.Multiply(theS);
        return aV;
    }

    public readonly gp_Vec2d Transformed(in gp_Trsf2d theT)
    {
        gp_Vec2d aV = this;
        aV.Transform(theT);
        return aV;
    }

    public static gp_Vec2d operator +(in gp_Vec2d theLeft, in gp_Vec2d theRight) => theLeft.Added(theRight);

    public static gp_Vec2d operator -(in gp_Vec2d theLeft, in gp_Vec2d theRight) => theLeft.Subtracted(theRight);

    public static gp_Vec2d operator -(in gp_Vec2d theV) => theV.Reversed();

    /// <summary>The dot product, as OCCT's <c>operator*</c>.</summary>
    public static double operator *(in gp_Vec2d theLeft, in gp_Vec2d theRight) => theLeft.Dot(theRight);

    /// <summary>The scalar cross product, as OCCT's <c>operator^</c>.</summary>
    public static double operator ^(in gp_Vec2d theLeft, in gp_Vec2d theRight) => theLeft.Crossed(theRight);

    public static gp_Vec2d operator *(in gp_Vec2d theV, double theScalar) => theV.Multiplied(theScalar);

    public static gp_Vec2d operator *(double theScalar, in gp_Vec2d theV) => theV.Multiplied(theScalar);

    public static gp_Vec2d operator /(in gp_Vec2d theV, double theScalar) => theV.Divided(theScalar);

    public readonly bool Equals(gp_Vec2d other) => coord.Equals(other.coord);

    public override readonly bool Equals(object? obj) => obj is gp_Vec2d other && Equals(other);

    public override readonly int GetHashCode() => coord.GetHashCode();

    public static bool operator ==(in gp_Vec2d theLeft, in gp_Vec2d theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Vec2d theLeft, in gp_Vec2d theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => gp.Format("gp_Vec2d", coord.X(), coord.Y());
}
