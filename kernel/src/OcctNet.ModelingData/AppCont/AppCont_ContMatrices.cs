// Port of: src/ModelingData/TKGeomBase/AppCont/AppCont_ContMatrices.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;

namespace Occt;

/// <summary>
/// The precomputed matrices of the continuous least squares in the Bernstein basis (AppCont_LeastSquare): mass
/// matrices and their inverses, the inverses for pass-point and tangency constraints, and the Bernstein polynomials
/// at the Gauss points. The tables are generated from OCCT's <c>.pxx</c> (tools/gen-appcont-matrices.cs).
/// </summary>
/// <remarks>OCCT's namespace <c>AppCont_ContMatrices</c>, internal to OCCT's TKGeomBase (a <c>.pxx</c>).</remarks>
internal static partial class AppCont_ContMatrices
{
    internal const int THE_MAX_CLASSE = 26;
    internal const int THE_MAX_MMATRIX_CLASSE = 24;
    internal const int THE_MAX_NBPOINTS = 24;

    /// <summary>
    /// The offset of a classe in the flat MMatrix/InvMMatrix tables: the blocks for classe 2 to 24 are packed one
    /// after the other, each of classe × classe entries.
    /// </summary>
    internal static int MMatrixOffset(int theClasse)
    {
        int anOffset = 0;
        for (int i = 2; i < theClasse; ++i)
        {
            anOffset += i * i;
        }
        return anOffset;
    }

    /// <summary>The offset of a classe in the flat IBPMatrix table ((classe - 2)² entries each, from classe 3).</summary>
    internal static int IBPMatrixOffset(int theClasse)
    {
        int anOffset = 0;
        for (int i = 1; i < theClasse - 2; ++i)
        {
            anOffset += i * i;
        }
        return anOffset;
    }

    /// <summary>The offset of a classe in the flat IBTMatrix table ((classe - 4)² entries each, from classe 5).</summary>
    internal static int IBTMatrixOffset(int theClasse)
    {
        int anOffset = 0;
        for (int i = 1; i < theClasse - 4; ++i)
        {
            anOffset += i * i;
        }
        return anOffset;
    }

    /// <summary>The Bernstein basis mass matrix of size classe × classe, written from M's lower bounds.</summary>
    internal static void MMatrix(int classe, Matrix M)
    {
        if (classe > THE_MAX_MMATRIX_CLASSE)
        {
            throw new DimensionErrorException("MMatrix: classe > 24");
        }

        ReadOnlySpan<double> aData = THE_BBMATRIX_DATA.Slice(MMatrixOffset(classe));
        int k = 0;
        for (int i = 1; i <= classe; ++i)
        {
            for (int j = 1; j <= classe; ++j)
            {
                M[i + M.LowerRow() - 1, j + M.LowerCol() - 1] = aData[k++];
            }
        }
    }

    /// <summary>The inverse of the Bernstein basis mass matrix, written from InvM's lower bounds.</summary>
    internal static void InvMMatrix(int classe, Matrix InvM)
    {
        if (classe > THE_MAX_MMATRIX_CLASSE)
        {
            throw new DimensionErrorException("InvMMatrix: classe > 24");
        }

        ReadOnlySpan<double> aData = THE_INV_MMATRIX_DATA.Slice(MMatrixOffset(classe));
        int k = 0;
        for (int i = 1; i <= classe; ++i)
        {
            for (int j = 1; j <= classe; ++j)
            {
                InvM[i + InvM.LowerRow() - 1, j + InvM.LowerCol() - 1] = aData[k++];
            }
        }
    }

    /// <summary>
    /// The inverse of MPᵀ·MP for pass-point constraints, where MP is M without its first and last columns:
    /// (classe - 2) × (classe - 2) values, written from IBPMa's lower bounds.
    /// </summary>
    internal static void IBPMatrix(int classe, Matrix IBPMa)
    {
        if (classe > THE_MAX_CLASSE)
        {
            throw new DimensionErrorException("IBPMatrix: classe > 26");
        }

        int aDim = classe - 2;
        ReadOnlySpan<double> aData = THE_IBPMATRIX_DATA.Slice(IBPMatrixOffset(classe));
        int k = 0;
        for (int i = 1; i <= aDim; ++i)
        {
            for (int j = 1; j <= aDim; ++j)
            {
                IBPMa[i + IBPMa.LowerRow() - 1, j + IBPMa.LowerCol() - 1] = aData[k++];
            }
        }
    }

    /// <summary>
    /// The inverse of MTᵀ·MT for tangency constraints, where MT is M without its first two and last two columns:
    /// (classe - 4) × (classe - 4) values, written from IBTMa's lower bounds.
    /// </summary>
    internal static void IBTMatrix(int classe, Matrix IBTMa)
    {
        if (classe > THE_MAX_CLASSE)
        {
            throw new DimensionErrorException("IBTMatrix: classe > 26");
        }

        int aDim = classe - 4;
        ReadOnlySpan<double> aData = THE_IBTMATRIX_DATA.Slice(IBTMatrixOffset(classe));
        int k = 0;
        for (int i = 1; i <= aDim; ++i)
        {
            for (int j = 1; j <= aDim; ++j)
            {
                IBTMa[i + IBTMa.LowerRow() - 1, j + IBTMa.LowerCol() - 1] = aData[k++];
            }
        }
    }

    /// <summary>
    /// The Bernstein polynomials of degree classe - 1 at the nbpoints Gauss-Legendre points of [0, 1]:
    /// M(i, j), i = 1..classe, j = 1..nbpoints (absolute indices, as OCCT).
    /// </summary>
    internal static void VBernstein(int classe, int nbpoints, Matrix M)
    {
        if (classe > THE_MAX_CLASSE)
        {
            throw new DimensionErrorException("VBernstein: classe > 26");
        }
        if (nbpoints > THE_MAX_NBPOINTS)
        {
            throw new DimensionErrorException("VBernstein: nbpoints > 24");
        }

        int Som = (int)(300.0 * ((classe - 1) * classe / 2.0 - 1.0)
                        + classe * (nbpoints - 1) * nbpoints / 2.0);

        ReadOnlySpan<double> aData = THE_VBMATRIX_DATA.Slice(Som);
        int k = 0;
        for (int i = 1; i <= classe; ++i)
        {
            for (int j = 1; j <= nbpoints; ++j)
            {
                M[i, j] = aData[k];
                ++k;
            }
        }
    }
}
