// Port of: src/FoundationClasses/TKMath/gp/gp_Vec.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_Vec.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gp_TrsfForm;

namespace Occt;

/// <summary>A vector in 3D space.</summary>
public partial struct gp_Vec : IEquatable<gp_Vec>
{
    private gp_XYZ coord;

    public gp_Vec(in gp_Dir theV) => coord = theV.XYZ();

    public gp_Vec(in gp_XYZ theCoord) => coord = theCoord;

    public gp_Vec(double theXv, double theYv, double theZv) => coord = new gp_XYZ(theXv, theYv, theZv);

    /// <summary>The vector from <paramref name="theP1"/> to <paramref name="theP2"/>.</summary>
    public gp_Vec(in gp_Pnt theP1, in gp_Pnt theP2) => coord = theP2.XYZ().Subtracted(theP1.XYZ());

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetCoord(int theIndex, double theXi) => coord.SetCoord(theIndex, theXi);

    internal void SetCoord(double theXv, double theYv, double theZv)
    {
        coord.SetX(theXv);
        coord.SetY(theYv);
        coord.SetZ(theZv);
    }

    internal void SetX(double theX) => coord.SetX(theX);

    internal void SetY(double theY) => coord.SetY(theY);

    internal void SetZ(double theZ) => coord.SetZ(theZ);

    internal void SetXYZ(in gp_XYZ theCoord) => coord = theCoord;

    internal void Add(in gp_Vec theOther) => coord.Add(theOther.coord);

    internal void Subtract(in gp_Vec theRight) => coord.Subtract(theRight.coord);

    internal void Multiply(double theScalar) => coord.Multiply(theScalar);

    internal void Divide(double theScalar) => coord.Divide(theScalar);

    internal void Cross(in gp_Vec theRight) => coord.Cross(theRight.coord);

    internal void CrossCross(in gp_Vec theV1, in gp_Vec theV2) => coord.CrossCross(theV1.coord, theV2.coord);

    internal void Normalize()
    {
        double aD = coord.Modulus();
        Standard_ConstructionError_Raise_if(aD <= gp.Resolution(), "gp_Vec::Normalize() - vector has zero norm");
        coord.Divide(aD);
    }

    internal void Reverse() => coord.Reverse();

    internal void SetLinearForm(double theA1, in gp_Vec theV1, double theA2, in gp_Vec theV2, double theA3, in gp_Vec theV3, in gp_Vec theV4)
        => coord.SetLinearForm(theA1, theV1.coord, theA2, theV2.coord, theA3, theV3.coord, theV4.coord);

    internal void SetLinearForm(double theA1, in gp_Vec theV1, double theA2, in gp_Vec theV2, double theA3, in gp_Vec theV3)
        => coord.SetLinearForm(theA1, theV1.coord, theA2, theV2.coord, theA3, theV3.coord);

    internal void SetLinearForm(double theA1, in gp_Vec theV1, double theA2, in gp_Vec theV2, in gp_Vec theV3)
        => coord.SetLinearForm(theA1, theV1.coord, theA2, theV2.coord, theV3.coord);

    internal void SetLinearForm(double theA1, in gp_Vec theV1, double theA2, in gp_Vec theV2)
        => coord.SetLinearForm(theA1, theV1.coord, theA2, theV2.coord);

    internal void SetLinearForm(double theA1, in gp_Vec theV1, in gp_Vec theV2) => coord.SetLinearForm(theA1, theV1.coord, theV2.coord);

    internal void SetLinearForm(in gp_Vec theV1, in gp_Vec theV2) => coord.SetLinearForm(theV1.coord, theV2.coord);

    internal void Mirror(in gp_Vec theVec)
    {
        double aMagnitude = theVec.coord.Modulus();
        if (aMagnitude > gp.Resolution())
        {
            gp_XYZ aMirrorVecXYZ = theVec.coord;
            double aOrigX = coord.X();
            double aOrigY = coord.Y();
            double aOrigZ = coord.Z();

            double aNormDirX = aMirrorVecXYZ.X() / aMagnitude;
            double aNormDirY = aMirrorVecXYZ.Y() / aMagnitude;
            double aNormDirZ = aMirrorVecXYZ.Z() / aMagnitude;

            double aCrossTermXY = 2.0 * aNormDirX * aNormDirY;
            double aCrossTermXZ = 2.0 * aNormDirX * aNormDirZ;
            double aCrossTermYZ = 2.0 * aNormDirY * aNormDirZ;
            double aXXTerm = 2.0 * aNormDirX * aNormDirX - 1.0;
            double aYYTerm = 2.0 * aNormDirY * aNormDirY - 1.0;
            double aZZTerm = 2.0 * aNormDirZ * aNormDirZ - 1.0;

            coord.SetX(aXXTerm * aOrigX + aCrossTermXY * aOrigY + aCrossTermXZ * aOrigZ);
            coord.SetY(aCrossTermXY * aOrigX + aYYTerm * aOrigY + aCrossTermYZ * aOrigZ);
            coord.SetZ(aCrossTermXZ * aOrigX + aCrossTermYZ * aOrigY + aZZTerm * aOrigZ);
        }
    }

    internal void Mirror(in gp_Ax1 theAxis)
    {
        gp_XYZ aDirectionXYZ = theAxis.Direction().XYZ();
        double aOrigX = coord.X();
        double aOrigY = coord.Y();
        double aOrigZ = coord.Z();
        double aDirX = aDirectionXYZ.X();
        double aDirY = aDirectionXYZ.Y();
        double aDirZ = aDirectionXYZ.Z();

        double aCrossTermXY = 2.0 * aDirX * aDirY;
        double aCrossTermXZ = 2.0 * aDirX * aDirZ;
        double aCrossTermYZ = 2.0 * aDirY * aDirZ;
        double aXXTerm = 2.0 * aDirX * aDirX - 1.0;
        double aYYTerm = 2.0 * aDirY * aDirY - 1.0;
        double aZZTerm = 2.0 * aDirZ * aDirZ - 1.0;

        coord.SetX(aXXTerm * aOrigX + aCrossTermXY * aOrigY + aCrossTermXZ * aOrigZ);
        coord.SetY(aCrossTermXY * aOrigX + aYYTerm * aOrigY + aCrossTermYZ * aOrigZ);
        coord.SetZ(aCrossTermXZ * aOrigX + aCrossTermYZ * aOrigY + aZZTerm * aOrigZ);
    }

    internal void Mirror(in gp_Ax2 theAxis)
    {
        gp_XYZ aZDir = theAxis.Direction().XYZ();
        gp_XYZ aMirXYZ = aZDir.Crossed(coord);

        if (aMirXYZ.Modulus() <= gp.Resolution())
        {
            coord.Reverse();
        }
        else
        {
            gp_XYZ aNewZ = aZDir;
            aNewZ.Cross(aMirXYZ);
            Mirror(new gp_Vec(aNewZ));
        }
    }

    internal void Rotate(in gp_Ax1 theA1, double theAng)
    {
        gp_Trsf aT = new();
        aT.SetRotation(theA1, theAng);
        coord.Multiply(aT.VectorialPart());
    }

    internal void Scale(double theS) => coord.Multiply(theS);

    internal void Transform(in gp_Trsf theTransformation)
    {
        if (theTransformation.Form() == gp_Identity || theTransformation.Form() == gp_Translation)
        {
        }
        else if (theTransformation.Form() == gp_PntMirror)
        {
            coord.Reverse();
        }
        else if (theTransformation.Form() == gp_Scale)
        {
            coord.Multiply(theTransformation.ScaleFactor());
        }
        else
        {
            coord.Multiply(theTransformation.VectorialPart());
        }
    }

    // Copies with a mutator applied

    public readonly gp_Vec WithCoord(int theIndex, double theXi)
    {
        gp_Vec aCopy = this;
        aCopy.SetCoord(theIndex, theXi);
        return aCopy;
    }

    public readonly gp_Vec WithCoord(double theXv, double theYv, double theZv) => new(theXv, theYv, theZv);

    public readonly gp_Vec WithX(double theX) => new(theX, coord.Y(), coord.Z());

    public readonly gp_Vec WithY(double theY) => new(coord.X(), theY, coord.Z());

    public readonly gp_Vec WithZ(double theZ) => new(coord.X(), coord.Y(), theZ);

    public readonly gp_Vec WithXYZ(in gp_XYZ theCoord) => new(theCoord);

    // Queries

    public readonly double Coord(int theIndex) => coord.Coord(theIndex);

    public readonly void Coord(out double theXv, out double theYv, out double theZv)
    {
        theXv = coord.X();
        theYv = coord.Y();
        theZv = coord.Z();
    }

    public readonly double X() => coord.X();

    public readonly double Y() => coord.Y();

    public readonly double Z() => coord.Z();

    public readonly gp_XYZ XYZ() => coord;

    public readonly bool IsEqual(in gp_Vec theOther, double theLinearTolerance, double theAngularTolerance)
    {
        double aMagnitude = Magnitude();
        double anOtherMagnitude = theOther.Magnitude();
        if (aMagnitude <= theLinearTolerance || anOtherMagnitude <= theLinearTolerance)
        {
            double aVal = Math.Abs(aMagnitude - anOtherMagnitude);
            return aVal <= theLinearTolerance;
        }
        else
        {
            double aVal = Math.Abs(aMagnitude - anOtherMagnitude);
            return aVal <= theLinearTolerance && Angle(theOther) <= theAngularTolerance;
        }
    }

    public readonly bool IsNormal(in gp_Vec theOther, double theAngularTolerance)
    {
        double anAng = Math.Abs(M_PI_2 - Angle(theOther));
        return anAng <= theAngularTolerance;
    }

    public readonly bool IsOpposite(in gp_Vec theOther, double theAngularTolerance)
    {
        double anAng = M_PI - Angle(theOther);
        return anAng <= theAngularTolerance;
    }

    public readonly bool IsParallel(in gp_Vec theOther, double theAngularTolerance)
    {
        double anAng = Angle(theOther);
        return anAng <= theAngularTolerance || M_PI - anAng <= theAngularTolerance;
    }

    public readonly double Angle(in gp_Vec theOther)
    {
        gp_VectorWithNullMagnitude_Raise_if(coord.Modulus() <= gp.Resolution() || theOther.coord.Modulus() <= gp.Resolution(), " ");
        return new gp_Dir(coord).Angle(new gp_Dir(theOther.coord));
    }

    public readonly double AngleWithRef(in gp_Vec theOther, in gp_Vec theVRef)
    {
        gp_VectorWithNullMagnitude_Raise_if(coord.Modulus() <= gp.Resolution()
                                            || theVRef.coord.Modulus() <= gp.Resolution()
                                            || theOther.coord.Modulus() <= gp.Resolution(), " ");
        return new gp_Dir(coord).AngleWithRef(new gp_Dir(theOther.coord), new gp_Dir(theVRef.coord));
    }

    public readonly double Magnitude() => coord.Modulus();

    public readonly double SquareMagnitude() => coord.SquareModulus();

    public readonly gp_Vec Added(in gp_Vec theOther)
    {
        gp_Vec aV = this;
        aV.coord.Add(theOther.coord);
        return aV;
    }

    public readonly gp_Vec Subtracted(in gp_Vec theRight)
    {
        gp_Vec aV = this;
        aV.coord.Subtract(theRight.coord);
        return aV;
    }

    public readonly gp_Vec Multiplied(double theScalar)
    {
        gp_Vec aV = this;
        aV.coord.Multiply(theScalar);
        return aV;
    }

    public readonly gp_Vec Divided(double theScalar)
    {
        gp_Vec aV = this;
        aV.coord.Divide(theScalar);
        return aV;
    }

    public readonly gp_Vec Crossed(in gp_Vec theRight)
    {
        gp_Vec aV = this;
        aV.coord.Cross(theRight.coord);
        return aV;
    }

    public readonly double CrossMagnitude(in gp_Vec theRight) => coord.CrossMagnitude(theRight.coord);

    public readonly double CrossSquareMagnitude(in gp_Vec theRight) => coord.CrossSquareMagnitude(theRight.coord);

    public readonly gp_Vec CrossCrossed(in gp_Vec theV1, in gp_Vec theV2)
    {
        gp_Vec aV = this;
        aV.coord.CrossCross(theV1.coord, theV2.coord);
        return aV;
    }

    public readonly double Dot(in gp_Vec theOther) => coord.Dot(theOther.coord);

    public readonly double DotCross(in gp_Vec theV1, in gp_Vec theV2) => coord.DotCross(theV1.coord, theV2.coord);

    public readonly gp_Vec Normalized()
    {
        double aD = coord.Modulus();
        Standard_ConstructionError_Raise_if(aD <= gp.Resolution(), "gp_Vec::Normalized() - vector has zero norm");
        gp_Vec aV = this;
        aV.coord.Divide(aD);
        return aV;
    }

    public readonly gp_Vec Reversed()
    {
        gp_Vec aV = this;
        aV.coord.Reverse();
        return aV;
    }

    public readonly gp_Vec Mirrored(in gp_Vec theVec)
    {
        gp_Vec aResult = this;
        aResult.Mirror(theVec);
        return aResult;
    }

    public readonly gp_Vec Mirrored(in gp_Ax1 theAxis)
    {
        gp_Vec aResult = this;
        aResult.Mirror(theAxis);
        return aResult;
    }

    public readonly gp_Vec Mirrored(in gp_Ax2 theAxis)
    {
        gp_Vec aResult = this;
        aResult.Mirror(theAxis);
        return aResult;
    }

    public readonly gp_Vec Rotated(in gp_Ax1 theA1, double theAng)
    {
        gp_Vec aVres = this;
        aVres.Rotate(theA1, theAng);
        return aVres;
    }

    public readonly gp_Vec Scaled(double theS)
    {
        gp_Vec aV = this;
        aV.coord.Multiply(theS);
        return aV;
    }

    public readonly gp_Vec Transformed(in gp_Trsf theT)
    {
        gp_Vec aV = this;
        aV.Transform(theT);
        return aV;
    }

    public static gp_Vec operator +(in gp_Vec theLeft, in gp_Vec theRight) => theLeft.Added(theRight);

    public static gp_Vec operator -(in gp_Vec theLeft, in gp_Vec theRight) => theLeft.Subtracted(theRight);

    public static gp_Vec operator -(in gp_Vec theV) => theV.Reversed();

    /// <summary>The dot product, as OCCT's <c>operator*</c>.</summary>
    public static double operator *(in gp_Vec theLeft, in gp_Vec theRight) => theLeft.Dot(theRight);

    /// <summary>The cross product, as OCCT's <c>operator^</c>.</summary>
    public static gp_Vec operator ^(in gp_Vec theLeft, in gp_Vec theRight) => theLeft.Crossed(theRight);

    public static gp_Vec operator *(in gp_Vec theV, double theScalar) => theV.Multiplied(theScalar);

    public static gp_Vec operator *(double theScalar, in gp_Vec theV) => theV.Multiplied(theScalar);

    public static gp_Vec operator /(in gp_Vec theV, double theScalar) => theV.Divided(theScalar);

    public readonly bool Equals(gp_Vec other) => coord.Equals(other.coord);

    public override readonly bool Equals(object? obj) => obj is gp_Vec other && Equals(other);

    public override readonly int GetHashCode() => coord.GetHashCode();

    public static bool operator ==(in gp_Vec theLeft, in gp_Vec theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_Vec theLeft, in gp_Vec theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => gp.Format("gp_Vec", coord.X(), coord.Y(), coord.Z());
}
