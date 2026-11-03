// Port of: src/ModelingData/TKGeomBase/AppCont/AppCont_ContMatrices.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;
using Occt;

namespace Kerf.Engine.Approximation;

/// <summary>
/// The precomputed matrices of the continuous least squares in the Bernstein basis (AppCont_LeastSquare): mass
/// matrices and their inverses, the inverses for pass-point and tangency constraints, and the Bernstein polynomials
/// at the Gauss points. The tables are generated from OCCT's <c>.pxx</c> (tools/gen-appcont-matrices.cs).
/// </summary>
/// <remarks>OCCT's namespace <c>AppCont_ContMatrices</c>, internal to OCCT's TKGeomBase (a <c>.pxx</c>).</remarks>
internal static partial class ContMatrices
{
    internal const int THE_MAX_CLASSE = 26;
    internal const int THE_MAX_MMATRIX_CLASSE = 24;
    internal const int THE_MAX_NBPOINTS = 24;

    /// <summary>
    /// The offset of a classe in the flat MMatrix/InvMMatrix tables: the blocks for classe 2 to 24 are packed one
    /// after the other, each of classe × classe entries.
    /// </summary>
    internal static int MMatrixOffset(int classe)
    {
        int offset = 0;
        for (int i = 2; i < classe; ++i)
        {
            offset += i * i;
        }
        return offset;
    }

    /// <summary>The offset of a classe in the flat IBPMatrix table ((classe - 2)² entries each, from classe 3).</summary>
    internal static int IBPMatrixOffset(int classe)
    {
        int offset = 0;
        for (int i = 1; i < classe - 2; ++i)
        {
            offset += i * i;
        }
        return offset;
    }

    /// <summary>The offset of a classe in the flat IBTMatrix table ((classe - 4)² entries each, from classe 5).</summary>
    internal static int IBTMatrixOffset(int classe)
    {
        int offset = 0;
        for (int i = 1; i < classe - 4; ++i)
        {
            offset += i * i;
        }
        return offset;
    }

    /// <summary>The Bernstein basis mass matrix of size classe × classe, written from M's lower bounds.</summary>
    internal static void MMatrix(int classe, Matrix m)
    {
        if (classe > THE_MAX_MMATRIX_CLASSE)
        {
            throw new DimensionErrorException("MMatrix: classe > 24");
        }

        ReadOnlySpan<double> data = THE_BBMATRIX_DATA.Slice(MMatrixOffset(classe));
        int k = 0;
        for (int i = 1; i <= classe; ++i)
        {
            for (int j = 1; j <= classe; ++j)
            {
                m[i + m.LowerRow() - 1, j + m.LowerCol() - 1] = data[k++];
            }
        }
    }

    /// <summary>The inverse of the Bernstein basis mass matrix, written from InvM's lower bounds.</summary>
    internal static void InvMMatrix(int classe, Matrix invM)
    {
        if (classe > THE_MAX_MMATRIX_CLASSE)
        {
            throw new DimensionErrorException("InvMMatrix: classe > 24");
        }

        ReadOnlySpan<double> data = THE_INV_MMATRIX_DATA.Slice(MMatrixOffset(classe));
        int k = 0;
        for (int i = 1; i <= classe; ++i)
        {
            for (int j = 1; j <= classe; ++j)
            {
                invM[i + invM.LowerRow() - 1, j + invM.LowerCol() - 1] = data[k++];
            }
        }
    }

    /// <summary>
    /// The inverse of MPᵀ·MP for pass-point constraints, where MP is M without its first and last columns:
    /// (classe - 2) × (classe - 2) values, written from IBPMa's lower bounds.
    /// </summary>
    internal static void IBPMatrix(int classe, Matrix ibpMa)
    {
        if (classe > THE_MAX_CLASSE)
        {
            throw new DimensionErrorException("IBPMatrix: classe > 26");
        }

        int dim = classe - 2;
        ReadOnlySpan<double> data = THE_IBPMATRIX_DATA.Slice(IBPMatrixOffset(classe));
        int k = 0;
        for (int i = 1; i <= dim; ++i)
        {
            for (int j = 1; j <= dim; ++j)
            {
                ibpMa[i + ibpMa.LowerRow() - 1, j + ibpMa.LowerCol() - 1] = data[k++];
            }
        }
    }

    /// <summary>
    /// The inverse of MTᵀ·MT for tangency constraints, where MT is M without its first two and last two columns:
    /// (classe - 4) × (classe - 4) values, written from IBTMa's lower bounds.
    /// </summary>
    internal static void IBTMatrix(int classe, Matrix ibtMa)
    {
        if (classe > THE_MAX_CLASSE)
        {
            throw new DimensionErrorException("IBTMatrix: classe > 26");
        }

        int dim = classe - 4;
        ReadOnlySpan<double> data = THE_IBTMATRIX_DATA.Slice(IBTMatrixOffset(classe));
        int k = 0;
        for (int i = 1; i <= dim; ++i)
        {
            for (int j = 1; j <= dim; ++j)
            {
                ibtMa[i + ibtMa.LowerRow() - 1, j + ibtMa.LowerCol() - 1] = data[k++];
            }
        }
    }

    /// <summary>
    /// The Bernstein polynomials of degree classe - 1 at the nbpoints Gauss-Legendre points of [0, 1]:
    /// M(i, j), i = 1..classe, j = 1..nbpoints (absolute indices, as OCCT).
    /// </summary>
    internal static void VBernstein(int classe, int nbpoints, Matrix m)
    {
        if (classe > THE_MAX_CLASSE)
        {
            throw new DimensionErrorException("VBernstein: classe > 26");
        }
        if (nbpoints > THE_MAX_NBPOINTS)
        {
            throw new DimensionErrorException("VBernstein: nbpoints > 24");
        }

        int som = (int)(300.0 * ((classe - 1) * classe / 2.0 - 1.0)
                        + classe * (nbpoints - 1) * nbpoints / 2.0);

        ReadOnlySpan<double> data = THE_VBMATRIX_DATA.Slice(som);
        int k = 0;
        for (int i = 1; i <= classe; ++i)
        {
            for (int j = 1; j <= nbpoints; ++j)
            {
                m[i, j] = data[k];
                ++k;
            }
        }
    }
}
