// Port of: src/FoundationClasses/TKMath/gp/gp_GTrsf2d.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_GTrsf2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gp_TrsfForm;

namespace Occt;

/// <summary>
/// A general (affine) transformation in 2D space: <c>matrix * P + loc</c>, where the matrix need not be
/// orthogonal. Its form is gp_Other unless it was built from a <see cref="gp_Trsf2d"/>.
/// </summary>
/// <remarks><c>new gp_GTrsf2d()</c> is the identity, as in OCCT; <c>default(gp_GTrsf2d)</c> is not valid.</remarks>
public partial struct gp_GTrsf2d : IEquatable<gp_GTrsf2d>
{
    private gp_Mat2d matrix;
    private gp_XY loc;
    private gp_TrsfForm shape;
    private double scale;

    public gp_GTrsf2d()
    {
        matrix = new gp_Mat2d();
        loc = new gp_XY(0.0, 0.0);
        shape = gp_Identity;
        scale = 1.0;
        matrix.SetScale(1.0);
    }

    public gp_GTrsf2d(in gp_Trsf2d theT)
    {
        shape = theT.shape;
        matrix = theT.matrix;
        loc = theT.loc;
        scale = theT.scale;
    }

    /// <summary>The transformation <c>theM * P + theV</c>.</summary>
    public gp_GTrsf2d(in gp_Mat2d theM, in gp_XY theV)
    {
        matrix = theM;
        loc = theV;
        shape = gp_Other;
        scale = 0.0;
    }

    // Static factories: a new transformation with one of OCCT's Set methods applied

    /// <summary>The affinity of ratio <paramref name="theRatio"/> along the normal to the axis <paramref name="theA"/>.</summary>
    public static gp_GTrsf2d Affinity(in gp_Ax2d theA, double theRatio) => new gp_GTrsf2d().WithAffinity(theA, theRatio);

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetAffinity(in gp_Ax2d A, double Ratio)
    {
        shape = gp_Other;
        scale = 0.0;
        double a = A.Direction().X();
        double b = A.Direction().Y();
        matrix.SetValue(1, 1, (1.0 - Ratio) * a * a + Ratio);
        matrix.SetValue(2, 2, (1.0 - Ratio) * b * b + Ratio);
        matrix.SetValue(1, 2, (1.0 - Ratio) * a * b);
        matrix.SetValue(2, 1, matrix.Value(1, 2));
        loc = A.Location().XY();
        loc.Reverse();
        loc.Multiply(matrix);
        loc.Add(A.Location().XY());
    }

    internal void SetValue(int theRow, int theCol, double theValue)
    {
        Standard_OutOfRange_Raise_if(theRow < 1 || theRow > 2 || theCol < 1 || theCol > 3, " ");
        if (theCol == 3)
        {
            loc.SetCoord(theRow, theValue);
        }
        else
        {
            matrix.SetValue(theRow, theCol, theValue);
        }

        shape = gp_Other;
    }

    internal void SetTranslationPart(in gp_XY Coord)
    {
        loc = Coord;
        if (Form() == gp_CompoundTrsf || Form() == gp_Other || Form() == gp_Translation)
        {
        }
        else if (Form() == gp_Identity)
        {
            shape = gp_Translation;
        }
        else
        {
            shape = gp_CompoundTrsf;
        }
    }

    internal void SetTrsf2d(in gp_Trsf2d theT)
    {
        shape = theT.shape;
        matrix = theT.matrix;
        loc = theT.loc;
        scale = theT.scale;
    }

    internal void SetVectorialPart(in gp_Mat2d theMatrix)
    {
        matrix = theMatrix;
        shape = gp_Other;
        scale = 0.0;
    }

    internal void Invert()
    {
        if (shape == gp_Other)
        {
            matrix.Invert();
            loc.Multiply(matrix);
            loc.Reverse();
        }
        else
        {
            gp_Trsf2d T = Trsf2d();
            T.Invert();
            SetTrsf2d(T);
        }
    }

    internal void Multiply(in gp_GTrsf2d T)
    {
        if (Form() == gp_Other || T.Form() == gp_Other)
        {
            shape = gp_Other;
            loc.Add(T.loc.Multiplied(matrix));
            matrix.Multiply(T.matrix);
        }
        else
        {
            gp_Trsf2d T1 = Trsf2d();
            gp_Trsf2d T2 = T.Trsf2d();
            T1.Multiply(T2);
            matrix = T1.matrix;
            loc = T1.loc;
            scale = T1.scale;
            shape = T1.shape;
        }
    }

    internal void PreMultiply(in gp_GTrsf2d T)
    {
        if (Form() == gp_Other || T.Form() == gp_Other)
        {
            shape = gp_Other;
            loc.Multiply(T.matrix);
            loc.Add(T.loc);
            matrix.PreMultiply(T.matrix);
        }
        else
        {
            gp_Trsf2d T1 = Trsf2d();
            gp_Trsf2d T2 = T.Trsf2d();
            T1.PreMultiply(T2);
            matrix = T1.matrix;
            loc = T1.loc;
            scale = T1.scale;
            shape = T1.shape;
        }
    }

    internal void Power(int N)
    {
        if (N == 0)
        {
            scale = 1.0;
            shape = gp_Identity;
            matrix.SetIdentity();
            loc = new gp_XY(0.0, 0.0);
        }
        else if (N == 1)
        {
        }
        else if (N == -1)
        {
            Invert();
        }
        else
        {
            if (N < 0)
            {
                Invert();
            }

            if (shape == gp_Other)
            {
                int Npower = Math.Abs(N) - 1;
                gp_XY Temploc = loc;
                gp_Mat2d Tempmatrix = matrix;
                for (;;)
                {
                    if (IsOdd(Npower))
                    {
                        loc.Add(Temploc.Multiplied(matrix));
                        matrix.Multiply(Tempmatrix);
                    }

                    if (Npower == 1)
                    {
                        break;
                    }

                    Temploc.Add(Temploc.Multiplied(Tempmatrix));
                    // OCCT squares in place through an aliased reference (gp_Mat2d::Power, docs/occt-issues.md)
                    Tempmatrix.Multiply(Tempmatrix);
                    Npower = Npower / 2;
                }
            }
            else
            {
                gp_Trsf2d T = Trsf2d();
                T.Power(N);
                SetTrsf2d(T);
            }
        }
    }

    // Copies with a mutator applied

    public readonly gp_GTrsf2d WithAffinity(in gp_Ax2d theA, double theRatio)
    {
        gp_GTrsf2d aCopy = this;
        aCopy.SetAffinity(theA, theRatio);
        return aCopy;
    }

    public readonly gp_GTrsf2d WithValue(int theRow, int theCol, double theValue)
    {
        gp_GTrsf2d aCopy = this;
        aCopy.SetValue(theRow, theCol, theValue);
        return aCopy;
    }

    public readonly gp_GTrsf2d WithTranslationPart(in gp_XY theCoord)
    {
        gp_GTrsf2d aCopy = this;
        aCopy.SetTranslationPart(theCoord);
        return aCopy;
    }

    public readonly gp_GTrsf2d WithTrsf2d(in gp_Trsf2d theT) => new(theT);

    public readonly gp_GTrsf2d WithVectorialPart(in gp_Mat2d theMatrix)
    {
        gp_GTrsf2d aCopy = this;
        aCopy.SetVectorialPart(theMatrix);
        return aCopy;
    }

    // Queries

    public readonly bool IsNegative() => matrix.Determinant() < 0.0;

    public readonly bool IsSingular() => matrix.IsSingular();

    public readonly gp_TrsfForm Form() => shape;

    public readonly gp_XY TranslationPart() => loc;

    public readonly gp_Mat2d VectorialPart() => matrix;

    /// <summary>The coefficient at a row (1 to 2) and column (1 to 3) of the 2x3 matrix.</summary>
    public readonly double Value(int theRow, int theCol)
    {
        Standard_OutOfRange_Raise_if(theRow < 1 || theRow > 2 || theCol < 1 || theCol > 3, " ");
        if (theCol == 3)
        {
            return loc.Coord(theRow);
        }

        if (shape == gp_Other)
        {
            return matrix.Value(theRow, theCol);
        }

        return scale * matrix.Value(theRow, theCol);
    }

    /// <summary>The coefficient at a row and column numbered from 1, as OCCT's <c>operator()</c>.</summary>
    public readonly double this[int theRow, int theCol] => Value(theRow, theCol);

    public readonly gp_GTrsf2d Inverted()
    {
        gp_GTrsf2d aT = this;
        aT.Invert();
        return aT;
    }

    /// <summary><c>this * theT</c>, which applies <paramref name="theT"/> first.</summary>
    public readonly gp_GTrsf2d Multiplied(in gp_GTrsf2d theT)
    {
        gp_GTrsf2d aTres = this;
        aTres.Multiply(theT);
        return aTres;
    }

    /// <summary><c>theT * this</c>, which applies this transformation first.</summary>
    public readonly gp_GTrsf2d PreMultiplied(in gp_GTrsf2d theT)
    {
        gp_GTrsf2d aTres = this;
        aTres.PreMultiply(theT);
        return aTres;
    }

    public readonly gp_GTrsf2d Powered(int theN)
    {
        gp_GTrsf2d aT = this;
        aT.Power(theN);
        return aT;
    }

    public readonly void Transforms(ref gp_XY theCoord)
    {
        theCoord.Multiply(matrix);
        if (!(shape == gp_Other) && !(scale == 1.0))
        {
            theCoord.Multiply(scale);
        }

        theCoord.Add(loc);
    }

    public readonly gp_XY Transformed(in gp_XY theCoord)
    {
        gp_XY aNewCoord = theCoord;
        Transforms(ref aNewCoord);
        return aNewCoord;
    }

    public readonly void Transforms(ref double theX, ref double theY)
    {
        gp_XY aDoublet = new(theX, theY);
        aDoublet.Multiply(matrix);
        if (!(shape == gp_Other) && !(scale == 1.0))
        {
            aDoublet.Multiply(scale);
        }

        aDoublet.Add(loc);
        aDoublet.Coord(out theX, out theY);
    }

    /// <summary>This transformation as a <see cref="gp_Trsf2d"/>.</summary>
    /// <exception cref="Standard_ConstructionError">The form is gp_Other, or the matrix is not orthogonal.</exception>
    public readonly gp_Trsf2d Trsf2d()
    {
        // Test of orthogonality
        double aTolerance = Precision.Angular();
        double aTolerance2 = 2.0 * aTolerance;

        if (Form() == gp_Other)
        {
            throw new Standard_ConstructionError("gp_GTrsf2d::Trsf2d() - non-orthogonal GTrsf2d(0)");
        }

        double value = (matrix.Value(1, 1) * matrix.Value(1, 1) + matrix.Value(2, 1) * matrix.Value(2, 1));
        if (Math.Abs(value - 1.0) > aTolerance2)
        {
            throw new Standard_ConstructionError("gp_GTrsf2d::Trsf2d() - non-orthogonal GTrsf2d(1)");
        }

        value = (matrix.Value(1, 2) * matrix.Value(1, 2) + matrix.Value(2, 2) * matrix.Value(2, 2));
        if (Math.Abs(value - 1.0) > aTolerance2)
        {
            throw new Standard_ConstructionError("gp_GTrsf2d::Trsf2d() - non-orthogonal GTrsf2d(2)");
        }

        value = (matrix.Value(1, 1) * matrix.Value(1, 2) + matrix.Value(2, 1) * matrix.Value(2, 2));
        if (Math.Abs(value) > aTolerance)
        {
            throw new Standard_ConstructionError("gp_GTrsf2d::Trsf2d() - non-orthogonal GTrsf2d(3)");
        }

        gp_Trsf2d aTransformation = new();
        aTransformation.matrix = matrix;
        aTransformation.shape = shape;
        aTransformation.scale = scale;
        aTransformation.loc = loc;
        return aTransformation;
    }

    public static gp_GTrsf2d operator *(in gp_GTrsf2d theLeft, in gp_GTrsf2d theRight) => theLeft.Multiplied(theRight);

    public readonly bool Equals(gp_GTrsf2d other)
        => matrix.Equals(other.matrix) && loc.Equals(other.loc) && shape == other.shape && scale.Equals(other.scale);

    public override readonly bool Equals(object? obj) => obj is gp_GTrsf2d other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(matrix, loc, shape, scale);

    public static bool operator ==(in gp_GTrsf2d theLeft, in gp_GTrsf2d theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_GTrsf2d theLeft, in gp_GTrsf2d theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"gp_GTrsf2d({shape}, scale {scale}, {matrix}, {loc})";
}
