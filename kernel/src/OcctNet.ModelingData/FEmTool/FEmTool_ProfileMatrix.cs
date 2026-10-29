// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_ProfileMatrix.hxx
// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_ProfileMatrix.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// A symmetric sparse matrix stored by profile (skyline), for 1D finite element methods. Row i stores the
/// coefficients from column <c>FirstIndexes(i)</c> to the diagonal. It is solved by Cholesky factorisation.
/// </summary>
public class FEmTool_ProfileMatrix : FEmTool_SparseMatrix
{
    // profile(1, i): the number of stored coefficients left of the diagonal in row i;
    // profile(2, i): the address of the diagonal coefficient of row i in ProfileMatrix
    private readonly NCollection_Array2<int> profile;
    private readonly NCollection_HArray1<double> ProfileMatrix;
    private readonly NCollection_HArray1<double> SMatrix;
    private readonly NCollection_HArray1<int> NextCoeff;
    private bool IsDecomp;

    /// <summary>
    /// A matrix of size <c>FirstIndexes.Length()</c>, where row i (from 1) stores columns
    /// <c>FirstIndexes(i)</c> to i.
    /// </summary>
    public FEmTool_ProfileMatrix(NCollection_Array1<int> FirstIndexes)
    {
        profile = new NCollection_Array2<int>(1, 2, 1, FirstIndexes.Length());
        int i, j, k, l;
        profile[1, 1] = 0;
        profile[2, 1] = 1;
        for (i = 2; i <= FirstIndexes.Length(); i++)
        {
            profile[1, i] = i - FirstIndexes[i];
            profile[2, i] = profile[2, i - 1] + profile[1, i] + 1;
        }
        NextCoeff = new NCollection_HArray1<int>(1, profile[2, FirstIndexes.Length()]);

        for (i = 1, k = 1; i <= FirstIndexes.Length(); i++)
        {
            for (j = FirstIndexes[i]; j <= i; j++)
            {
                for (l = i + 1; l <= FirstIndexes.Length() && j < FirstIndexes[l]; l++)
                {
                    ;
                }

                if (l > FirstIndexes.Length())
                {
                    NextCoeff.SetValue(k, 0);
                }
                else
                {
                    NextCoeff.SetValue(k, l);
                }
                k++;
            }
        }

        ProfileMatrix = new NCollection_HArray1<double>(1, profile[2, FirstIndexes.Length()]);
        SMatrix = new NCollection_HArray1<double>(1, profile[2, FirstIndexes.Length()]);
        IsDecomp = false;
    }

    public override void Init(double Value)
    {
        ProfileMatrix.Init(Value);
        IsDecomp = false;
    }

    /// <summary>
    /// The coefficient (<paramref name="I"/>, <paramref name="J"/>) or (J, I), which are the same; it must be in
    /// the profile.
    /// </summary>
    public override ref double ChangeValue(int I, int J)
    {
        int Ind;
        Ind = I - J;
        if (Ind < 0)
        {
            Ind = -Ind;
            Standard_OutOfRange_Raise_if(Ind > profile[1, J], "FEmTool_ProfileMatrix::ChangeValue");
            Ind = profile[2, J] - Ind;
        }
        else
        {
            Standard_OutOfRange_Raise_if(Ind > profile[1, I], "FEmTool_ProfileMatrix::ChangeValue");
            Ind = profile[2, I] - Ind;
        }
        return ref ProfileMatrix[Ind];
    }

    /// <summary>Cholesky factorisation S tS of the matrix; false if it is not positive definite.</summary>
    public override bool Decompose()
    {
        int i, j, k, ik, jk, DiagAddr, CurrAddr, Kmin, Kj;
        double Sum, a, Eps = 1.0e-32;

        SMatrix.Init(0.0);
        // SMA and PM are the C arrays shifted to be indexed from 1
        Span<double> SMA = SMatrix.AsSpan();
        ReadOnlySpan<double> PM = ProfileMatrix.AsSpan();

        IsDecomp = false;
        for (j = 1; j <= RowNumber(); j++)
        {
            DiagAddr = profile[2, j];
            Kj = j - profile[1, j];
            Sum = 0;
            for (k = DiagAddr - profile[1, j]; k < DiagAddr; k++)
            {
                Sum += SMA[k - 1] * SMA[k - 1];
            }

            a = PM[DiagAddr - 1] - Sum;
            if (a < Eps)
            {
                return false; // Matrix is not positive defined
            }
            a = Math.Sqrt(a);
            SMA[DiagAddr - 1] = a;

            CurrAddr = DiagAddr;
            while ((i = NextCoeff.Value(CurrAddr)) > 0)
            {
                CurrAddr = profile[2, i] - (i - j);

                // Computation of Sum of S  .S  for k = 1,..,j-1
                //                        ik  jk
                Sum = 0;
                Kmin = Max((i - profile[1, i]), Kj);
                ik = profile[2, i] - i + Kmin;
                jk = DiagAddr - j + Kmin;
                for (k = Kmin; k < j; k++, ik++, jk++)
                {
                    Sum += SMA[ik - 1] * SMA[jk - 1];
                }
                SMA[CurrAddr - 1] = (PM[CurrAddr - 1] - Sum) / a;
            }
        }
        IsDecomp = true;
        return IsDecomp;
    }

    /// <summary>Resolution of the system S tS X = B, after <see cref="Decompose"/>.</summary>
    public override void Solve(math_Vector B, math_Vector X)
    {
        if (!IsDecomp)
        {
            throw new StdFail_NotDone("Decomposition must be done");
        }

        int i, j, jj, DiagAddr, CurrAddr;
        double Sum;

        // x, b, SMA and NC are the C arrays shifted to be indexed from 1
        Span<double> x = X.AsSpan();
        ReadOnlySpan<double> b = B.AsSpan();
        ReadOnlySpan<double> SMA = SMatrix.AsSpan();
        ReadOnlySpan<int> NC = NextCoeff.AsSpan();

        // Resolution of Sw = B;
        for (i = 1; i <= RowNumber(); i++)
        {
            DiagAddr = profile[2, i];
            Sum = 0;
            for (j = i - profile[1, i], jj = DiagAddr - (i - j); j < i; j++, jj++)
            {
                Sum += SMA[jj - 1] * x[j - 1];
            }
            x[i - 1] = (b[i - 1] - Sum) / SMA[DiagAddr - 1];
        }

        // Resolution of t(S)X = w;
        for (i = ColNumber(); i >= 1; i--)
        {
            DiagAddr = profile[2, i];
            j = NC[DiagAddr - 1];
            Sum = 0;
            while (j > 0)
            {
                CurrAddr = profile[2, j] - (j - i);
                Sum += SMA[CurrAddr - 1] * x[j - 1];
                j = NC[CurrAddr - 1];
            }
            x[i - 1] = (x[i - 1] - Sum) / SMA[DiagAddr - 1];
        }
    }

    /// <summary>Not implemented: throws Standard_NotImplemented, as OCCT.</summary>
    public override bool Prepare()
    {
        throw new Standard_NotImplemented("FEmTool_ProfileMatrix::Prepare");
    }

    /// <summary>Not implemented: throws Standard_NotImplemented, as OCCT.</summary>
    public override void Solve(math_Vector B, math_Vector Init, math_Vector X, math_Vector Residual,
                               double Tolerance = 1.0e-8, int NbIterations = 50)
    {
        throw new Standard_NotImplemented("FEmTool_ProfileMatrix::Solve");
    }

    /// <summary>MX = H X.</summary>
    public override void Multiplied(math_Vector X, math_Vector MX)
    {
        int i, j, jj, DiagAddr, CurrAddr;
        // m, x, PM and NC are the C arrays shifted to be indexed from 1
        Span<double> m = MX.AsSpan();
        ReadOnlySpan<double> x = X.AsSpan();
        ReadOnlySpan<double> PM = ProfileMatrix.AsSpan();
        ReadOnlySpan<int> NC = NextCoeff.AsSpan();

        for (i = 1; i <= RowNumber(); i++)
        {
            DiagAddr = profile[2, i];
            m[i - 1] = 0;
            for (j = i - profile[1, i], jj = DiagAddr - (i - j); j <= i; j++, jj++)
            {
                m[i - 1] += PM[jj - 1] * x[j - 1];
            }

            CurrAddr = DiagAddr;
            for (j = NC[CurrAddr - 1]; j > 0; j = NC[CurrAddr - 1])
            {
                CurrAddr = profile[2, j] - (j - i);
                m[i - 1] += PM[CurrAddr - 1] * x[j - 1];
            }
        }
    }

    public override int RowNumber() => profile.RowLength();

    public override int ColNumber() => profile.RowLength();

    /// <summary>True if the coefficient (<paramref name="i"/>, <paramref name="j"/>) is stored.</summary>
    public bool IsInProfile(int i, int j)
    {
        if (j <= i)
        {
            return (i - j) <= profile[1, i];
        }
        else if ((j - i) <= profile[1, j])
        {
            return true;
        }

        return false;
    }

    /// <summary>Writes the matrix and its chaining, as OCCT's <c>OutM</c> (which writes to std::cout).</summary>
    public void OutM(TextWriter theO)
    {
        int i, j;
        theO.Write("Matrix A\n");
        for (i = 1; i <= RowNumber(); i++)
        {
            for (j = 1; j < i - profile[1, i]; j++)
            {
                theO.Write("0 ");
            }

            for (j = profile[2, i] - profile[1, i]; j <= profile[2, i]; j++)
            {
                theO.Write(ProfileMatrix.Value(j).ToString(CultureInfo.InvariantCulture) + " ");
            }
            theO.Write('\n');
        }

        theO.Write("NextCoeff\n");
        for (i = 1; i <= profile[2, RowNumber()]; i++)
        {
            theO.Write(NextCoeff.Value(i).ToString(CultureInfo.InvariantCulture) + " ");
        }
        theO.Write('\n');
    }

    /// <summary>Writes the factor S, as OCCT's <c>OutS</c> (which writes to std::cout).</summary>
    public void OutS(TextWriter theO)
    {
        int i, j;
        theO.Write("Matrix S\n");
        for (i = 1; i <= RowNumber(); i++)
        {
            for (j = 1; j < i - profile[1, i]; j++)
            {
                theO.Write("0 ");
            }

            for (j = profile[2, i] - profile[1, i]; j <= profile[2, i]; j++)
            {
                theO.Write(SMatrix.Value(j).ToString(CultureInfo.InvariantCulture) + " ");
            }
            theO.Write('\n');
        }
    }
}
