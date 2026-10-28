// Port of: src/FoundationClasses/TKMath/gp/gp_GTrsf.hxx
// Port of: src/FoundationClasses/TKMath/gp/gp_GTrsf.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.gp_TrsfForm;

namespace Occt;

/// <summary>
/// A general transformation in 3D space: an affine map <c>P' = matrix * P + loc</c> whose vectorial part may be
/// any 3x3 matrix (affinities, non-uniform scales, ...). When <see cref="Form"/> is not <c>gp_Other</c>, the
/// transformation is a <see cref="gp_Trsf"/> and its scale factor is kept apart from the matrix.
/// </summary>
/// <remarks><c>new gp_GTrsf()</c> is the identity, as in OCCT; <c>default(gp_GTrsf)</c> is not valid.</remarks>
public partial struct gp_GTrsf : IEquatable<gp_GTrsf>
{
    private gp_Mat matrix;
    private gp_XYZ loc;
    private gp_TrsfForm shape;
    private double scale;

    public gp_GTrsf()
    {
        loc = new gp_XYZ(0.0, 0.0, 0.0);
        shape = gp_Identity;
        scale = 1.0;
        matrix = new gp_Mat();
        matrix.SetScale(1.0);
    }

    public gp_GTrsf(in gp_Trsf theT)
    {
        shape = theT.Form();
        matrix = theT.matrix;
        loc = theT.TranslationPart();
        scale = theT.ScaleFactor();
    }

    /// <summary>The transformation with vectorial part <paramref name="theM"/> and translation part <paramref name="theV"/>.</summary>
    public gp_GTrsf(in gp_Mat theM, in gp_XYZ theV)
    {
        matrix = theM;
        loc = theV;
        shape = gp_Other;
        scale = 0.0;
    }

    // Static factories: a new transformation with one of OCCT's Set methods applied

    /// <summary>The affinity of ratio <paramref name="theRatio"/> about an axis.</summary>
    public static gp_GTrsf Affinity(in gp_Ax1 theA1, double theRatio) => new gp_GTrsf().WithAffinity(theA1, theRatio);

    /// <summary>The affinity of ratio <paramref name="theRatio"/> about a plane.</summary>
    public static gp_GTrsf Affinity(in gp_Ax2 theA2, double theRatio) => new gp_GTrsf().WithAffinity(theA2, theRatio);

    /// <summary>The transformation from a 4x4 matrix given row by row (16 values; the last row is ignored).</summary>
    public static gp_GTrsf Mat4(ReadOnlySpan<double> theMat) => new gp_GTrsf().WithMat4(theMat);

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetAffinity(in gp_Ax1 theA1, double theRatio)
    {
        shape = gp_Other;
        scale = 0.0;
        matrix.SetDot(theA1.Direction().XYZ());
        matrix.Multiply(1.0 - theRatio);
        matrix.SetDiagonal(matrix.Value(1, 1) + theRatio,
                           matrix.Value(2, 2) + theRatio,
                           matrix.Value(3, 3) + theRatio);
        loc = theA1.Location().XYZ();
        loc.Reverse();
        loc.Multiply(matrix);
        loc.Add(theA1.Location().XYZ());
    }

    internal void SetAffinity(in gp_Ax2 theA2, double theRatio)
    {
        shape = gp_Other;
        scale = 0.0;
        matrix.SetDot(theA2.Direction().XYZ());
        matrix.Multiply(theRatio - 1.0);
        loc = theA2.Location().XYZ();
        loc.Reverse();
        loc.Multiply(matrix);
        matrix.SetDiagonal(matrix.Value(1, 1) + 1.0, matrix.Value(2, 2) + 1.0, matrix.Value(3, 3) + 1.0);
    }

    internal void SetValue(int theRow, int theCol, double theValue)
    {
        Standard_OutOfRange_Raise_if(theRow < 1 || theRow > 3 || theCol < 1 || theCol > 4, " ");
        if (theCol == 4)
        {
            loc.SetCoord(theRow, theValue);
            if (shape == gp_Identity)
            {
                shape = gp_Translation;
            }

            return;
        }
        else
        {
            if (!(shape == gp_Other) && !(scale == 1.0))
            {
                matrix.Multiply(scale);
            }

            matrix.SetValue(theRow, theCol, theValue);
            shape = gp_Other;
            scale = 0.0;
            return;
        }
    }

    internal void SetVectorialPart(in gp_Mat theMatrix)
    {
        matrix = theMatrix;
        shape = gp_Other;
        scale = 0.0;
    }

    internal void SetTranslationPart(in gp_XYZ Coord)
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

    internal void SetTrsf(in gp_Trsf theT)
    {
        shape = theT.shape;
        matrix = theT.matrix;
        loc = theT.loc;
        scale = theT.scale;
    }

    /// <summary>Recomputes the form from the matrix: <c>gp_Other</c> unless the matrix is a uniform scale of an orthogonal matrix.</summary>
    internal void SetForm()
    {
        double tol = 1.0e-12; // Precision::Angular();
        //
        // don t trust the initial values !
        //
        gp_Mat M = matrix;
        double s = M.Determinant();

        if (Math.Abs(s) < gp.Resolution())
        {
            throw new Standard_ConstructionError("gp_GTrsf::SetForm, null determinant");
        }

        if (s > 0)
        {
            s = Math.Pow(s, 1.0 / 3.0);
        }
        else
        {
            s = -Math.Pow(-s, 1.0 / 3.0);
        }

        M.Divide(s);

        // check if the matrix is an uniform matrix
        // the transposition should be the invert.
        gp_Mat TM = M;
        TM.Transpose();
        TM.Multiply(M);
        gp_Mat anIdentity = new();
        anIdentity.SetIdentity();
        TM.Subtract(anIdentity);
        if (shape == gp_Other)
        {
            shape = gp_CompoundTrsf;
        }

        for (int i = 1; i <= 3; i++)
        {
            for (int j = 1; j <= 3; j++)
            {
                if (Math.Abs(TM.Value(i, j)) > tol)
                {
                    shape = gp_Other;
                    return;
                }
            }
        }
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
            gp_Trsf T = Trsf();
            T.Invert();
            SetTrsf(T);
        }
    }

    /// <summary>this = this * T, which applies T first.</summary>
    internal void Multiply(in gp_GTrsf T)
    {
        if (Form() == gp_Other || T.Form() == gp_Other)
        {
            shape = gp_Other;
            loc.Add(T.loc.Multiplied(matrix));
            matrix.Multiply(T.matrix);
        }
        else
        {
            gp_Trsf T1 = Trsf();
            gp_Trsf T2 = T.Trsf();
            T1.Multiply(T2);
            matrix = T1.matrix;
            loc = T1.loc;
            scale = T1.scale;
            shape = T1.shape;
        }
    }

    /// <summary>this = T * this, which applies this transformation first.</summary>
    internal void PreMultiply(in gp_GTrsf T)
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
            gp_Trsf T1 = Trsf();
            gp_Trsf T2 = T.Trsf();
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
            loc = new gp_XYZ(0.0, 0.0, 0.0);
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
            if (shape == gp_Other)
            {
                // OCCT: a negative N other than -1 is treated as |N|; the inverse is never taken.
                int Npower = Math.Abs(N) - 1;
                gp_XYZ Temploc = loc;
                gp_Mat Tempmatrix = matrix;
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
                    Tempmatrix.Multiply(Tempmatrix);
                    Npower = Npower / 2;
                }
            }
            else
            {
                gp_Trsf T = Trsf();
                T.Power(N);
                SetTrsf(T);
            }
        }
    }

    /// <summary>Sets the transformation from a 4x4 matrix given row by row (OCCT's <c>SetMat4</c> on an <c>NCollection_Mat4</c>).</summary>
    internal void SetMat4(ReadOnlySpan<double> theMat)
    {
        if (theMat.Length < 16)
        {
            throw new ArgumentException("A 4x4 matrix needs 16 elements.", nameof(theMat));
        }

        shape = gp_Other;
        scale = 0.0;
        matrix.SetValue(1, 1, theMat[0]);
        matrix.SetValue(1, 2, theMat[1]);
        matrix.SetValue(1, 3, theMat[2]);
        matrix.SetValue(2, 1, theMat[4]);
        matrix.SetValue(2, 2, theMat[5]);
        matrix.SetValue(2, 3, theMat[6]);
        matrix.SetValue(3, 1, theMat[8]);
        matrix.SetValue(3, 2, theMat[9]);
        matrix.SetValue(3, 3, theMat[10]);
        loc.SetCoord(theMat[3], theMat[7], theMat[11]);
    }

    // Copies with a mutator applied

    public readonly gp_GTrsf WithAffinity(in gp_Ax1 theA1, double theRatio)
    {
        gp_GTrsf aCopy = this;
        aCopy.SetAffinity(theA1, theRatio);
        return aCopy;
    }

    public readonly gp_GTrsf WithAffinity(in gp_Ax2 theA2, double theRatio)
    {
        gp_GTrsf aCopy = this;
        aCopy.SetAffinity(theA2, theRatio);
        return aCopy;
    }

    /// <summary>A copy with one coefficient of the 3x4 matrix replaced (row 1 to 3, column 1 to 4).</summary>
    public readonly gp_GTrsf WithValue(int theRow, int theCol, double theValue)
    {
        gp_GTrsf aCopy = this;
        aCopy.SetValue(theRow, theCol, theValue);
        return aCopy;
    }

    public readonly gp_GTrsf WithVectorialPart(in gp_Mat theMatrix)
    {
        gp_GTrsf aCopy = this;
        aCopy.SetVectorialPart(theMatrix);
        return aCopy;
    }

    public readonly gp_GTrsf WithTranslationPart(in gp_XYZ theCoord)
    {
        gp_GTrsf aCopy = this;
        aCopy.SetTranslationPart(theCoord);
        return aCopy;
    }

    public readonly gp_GTrsf WithTrsf(in gp_Trsf theT) => new(theT);

    /// <summary>A copy whose form is recomputed from its matrix (OCCT's <c>SetForm</c>).</summary>
    public readonly gp_GTrsf WithForm()
    {
        gp_GTrsf aCopy = this;
        aCopy.SetForm();
        return aCopy;
    }

    public readonly gp_GTrsf WithMat4(ReadOnlySpan<double> theMat)
    {
        gp_GTrsf aCopy = this;
        aCopy.SetMat4(theMat);
        return aCopy;
    }

    // Queries

    /// <summary>True when the determinant of the vectorial part is negative.</summary>
    public readonly bool IsNegative() => matrix.Determinant() < 0.0;

    /// <summary>True when the vectorial part is singular, so the transformation cannot be inverted.</summary>
    public readonly bool IsSingular() => matrix.IsSingular();

    public readonly gp_TrsfForm Form() => shape;

    public readonly gp_XYZ TranslationPart() => loc;

    /// <summary>The 3x3 vectorial part, without the scale factor of a non-<c>gp_Other</c> form.</summary>
    public readonly gp_Mat VectorialPart() => matrix;

    /// <summary>The coefficient at a row (1 to 3) and column (1 to 4) of the 3x4 matrix, scale included.</summary>
    public readonly double Value(int theRow, int theCol)
    {
        Standard_OutOfRange_Raise_if(theRow < 1 || theRow > 3 || theCol < 1 || theCol > 4, " ");
        if (theCol == 4)
        {
            return loc.Coord(theRow);
        }

        if (shape == gp_Other)
        {
            return matrix.Value(theRow, theCol);
        }

        return scale * matrix.Value(theRow, theCol);
    }

    public readonly double this[int theRow, int theCol] => Value(theRow, theCol);

    public readonly gp_GTrsf Inverted()
    {
        gp_GTrsf aT = this;
        aT.Invert();
        return aT;
    }

    /// <summary><c>this * theT</c>, which applies <paramref name="theT"/> first.</summary>
    public readonly gp_GTrsf Multiplied(in gp_GTrsf theT)
    {
        gp_GTrsf aTres = this;
        aTres.Multiply(theT);
        return aTres;
    }

    /// <summary><c>theT * this</c>, which applies this transformation first.</summary>
    public readonly gp_GTrsf PreMultiplied(in gp_GTrsf theT)
    {
        gp_GTrsf aTres = this;
        aTres.PreMultiply(theT);
        return aTres;
    }

    public readonly gp_GTrsf Powered(int theN)
    {
        gp_GTrsf aT = this;
        aT.Power(theN);
        return aT;
    }

    public readonly void Transforms(ref gp_XYZ theCoord)
    {
        theCoord.Multiply(matrix);
        if (!(shape == gp_Other) && !(scale == 1.0))
        {
            theCoord.Multiply(scale);
        }

        theCoord.Add(loc);
    }

    public readonly void Transforms(ref double theX, ref double theY, ref double theZ)
    {
        gp_XYZ aTriplet = new(theX, theY, theZ);
        aTriplet.Multiply(matrix);
        if (!(shape == gp_Other) && !(scale == 1.0))
        {
            aTriplet.Multiply(scale);
        }

        aTriplet.Add(loc);
        aTriplet.Coord(out theX, out theY, out theZ);
    }

    /// <summary>The coordinates transformed: the non-ref form of <see cref="Transforms(ref gp_XYZ)"/>.</summary>
    public readonly gp_XYZ Transformed(in gp_XYZ theCoord)
    {
        gp_XYZ aCoord = theCoord;
        Transforms(ref aCoord);
        return aCoord;
    }

    /// <summary>This transformation as a <see cref="gp_Trsf"/>.</summary>
    /// <exception cref="Standard_ConstructionError">The form is <c>gp_Other</c>.</exception>
    public readonly gp_Trsf Trsf()
    {
        if (Form() == gp_Other)
        {
            throw new Standard_ConstructionError("gp_GTrsf::Trsf() - non-orthogonal GTrsf");
        }

        gp_Trsf aT = new();
        aT.shape = shape;
        aT.scale = scale;
        aT.matrix = matrix;
        aT.loc = loc;
        return aT;
    }

    /// <summary>The 4x4 matrix, row by row, as OCCT's <c>GetMat4</c>.</summary>
    public readonly void GetMat4(Span<double> theMat)
    {
        if (theMat.Length < 16)
        {
            throw new ArgumentException("A 4x4 matrix needs 16 elements.", nameof(theMat));
        }

        if (shape == gp_Identity)
        {
            theMat[..16].Clear();
            theMat[0] = theMat[5] = theMat[10] = theMat[15] = 1.0;
            return;
        }

        for (int aRow = 1; aRow <= 3; aRow++)
        {
            for (int aCol = 1; aCol <= 4; aCol++)
            {
                theMat[(aRow - 1) * 4 + (aCol - 1)] = Value(aRow, aCol);
            }
        }

        theMat[12] = 0.0;
        theMat[13] = 0.0;
        theMat[14] = 0.0;
        theMat[15] = 1.0;
    }

    public static gp_GTrsf operator *(in gp_GTrsf theLeft, in gp_GTrsf theRight) => theLeft.Multiplied(theRight);

    public readonly bool Equals(gp_GTrsf other)
        => matrix.Equals(other.matrix) && loc.Equals(other.loc) && shape == other.shape && scale.Equals(other.scale);

    public override readonly bool Equals(object? obj) => obj is gp_GTrsf other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(matrix, loc, shape, scale);

    public static bool operator ==(in gp_GTrsf theLeft, in gp_GTrsf theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in gp_GTrsf theLeft, in gp_GTrsf theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => $"gp_GTrsf({shape}, scale {scale}, {matrix}, {loc})";
}
