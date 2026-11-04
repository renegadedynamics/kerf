// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_ProfileMatrix.hxx
// Port of: src/ModelingData/TKGeomBase/FEmTool/FEmTool_ProfileMatrix.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Occt;
using System.Globalization;

namespace Kerf.Engine.Approximation.FiniteElements;

/// <summary>
/// A symmetric sparse matrix stored by profile (skyline), for 1D finite element methods. Row i stores the
/// coefficients from column <c>FirstIndexes(i)</c> to the diagonal. It is solved by Cholesky factorisation.
/// </summary>
internal class FemProfileMatrix : SparseMatrix
{
    // profile(1, i): the number of stored coefficients left of the diagonal in row i;
    // profile(2, i): the address of the diagonal coefficient of row i in ProfileMatrix
    private readonly NCollection_Array2<int> _profile;
    private readonly NCollection_HArray1<double> _profileMatrix;
    private readonly NCollection_HArray1<double> _sMatrix;
    private readonly NCollection_HArray1<int> _nextCoeff;
    private bool _isDecomp;

    /// <summary>
    /// A matrix of size <c>FirstIndexes.Length()</c>, where row i (from 1) stores columns
    /// <c>FirstIndexes(i)</c> to i.
    /// </summary>
    public FemProfileMatrix(NCollection_Array1<int> firstIndexes)
    {
        _profile = new NCollection_Array2<int>(1, 2, 1, firstIndexes.Length());
        int i, j, k, l;
        _profile[1, 1] = 0;
        _profile[2, 1] = 1;
        for (i = 2; i <= firstIndexes.Length(); i++)
        {
            _profile[1, i] = i - firstIndexes[i];
            _profile[2, i] = _profile[2, i - 1] + _profile[1, i] + 1;
        }
        _nextCoeff = new NCollection_HArray1<int>(1, _profile[2, firstIndexes.Length()]);

        for (i = 1, k = 1; i <= firstIndexes.Length(); i++)
        {
            for (j = firstIndexes[i]; j <= i; j++)
            {
                for (l = i + 1; l <= firstIndexes.Length() && j < firstIndexes[l]; l++)
                {
                    ;
                }

                if (l > firstIndexes.Length())
                {
                    _nextCoeff.SetValue(k, 0);
                }
                else
                {
                    _nextCoeff.SetValue(k, l);
                }
                k++;
            }
        }

        _profileMatrix = new NCollection_HArray1<double>(1, _profile[2, firstIndexes.Length()]);
        _sMatrix = new NCollection_HArray1<double>(1, _profile[2, firstIndexes.Length()]);
        _isDecomp = false;
    }

    public override void Init(double value)
    {
        _profileMatrix.Init(value);
        _isDecomp = false;
    }

    /// <summary>
    /// The coefficient (<paramref name="i"/>, <paramref name="j"/>) or (J, I), which are the same; it must be in
    /// the profile.
    /// </summary>
    public override ref double ChangeValue(int i, int j)
    {
        int ind;
        ind = i - j;
        if (ind < 0)
        {
            ind = -ind;
            Standard_OutOfRange_Raise_if(ind > _profile[1, j], "FEmTool_ProfileMatrix::ChangeValue");
            ind = _profile[2, j] - ind;
        }
        else
        {
            Standard_OutOfRange_Raise_if(ind > _profile[1, i], "FEmTool_ProfileMatrix::ChangeValue");
            ind = _profile[2, i] - ind;
        }
        return ref _profileMatrix[ind];
    }

    /// <summary>Cholesky factorisation S tS of the matrix; false if it is not positive definite.</summary>
    public override bool Decompose()
    {
        int i, j, k, ik, jk, diagAddr, currAddr, kmin, kj;
        double sum, a, eps = 1.0e-32;

        _sMatrix.Init(0.0);
        // SMA and PM are the C arrays shifted to be indexed from 1
        Span<double> sma = _sMatrix.AsSpan();
        ReadOnlySpan<double> pm = _profileMatrix.AsSpan();

        _isDecomp = false;
        for (j = 1; j <= RowNumber(); j++)
        {
            diagAddr = _profile[2, j];
            kj = j - _profile[1, j];
            sum = 0;
            for (k = diagAddr - _profile[1, j]; k < diagAddr; k++)
            {
                sum += sma[k - 1] * sma[k - 1];
            }

            a = pm[diagAddr - 1] - sum;
            if (a < eps)
            {
                return false; // Matrix is not positive defined
            }
            a = Math.Sqrt(a);
            sma[diagAddr - 1] = a;

            currAddr = diagAddr;
            while ((i = _nextCoeff.Value(currAddr)) > 0)
            {
                currAddr = _profile[2, i] - (i - j);

                // Computation of Sum of S  .S  for k = 1,..,j-1
                //                        ik  jk
                sum = 0;
                kmin = Max((i - _profile[1, i]), kj);
                ik = _profile[2, i] - i + kmin;
                jk = diagAddr - j + kmin;
                for (k = kmin; k < j; k++, ik++, jk++)
                {
                    sum += sma[ik - 1] * sma[jk - 1];
                }
                sma[currAddr - 1] = (pm[currAddr - 1] - sum) / a;
            }
        }
        _isDecomp = true;
        return _isDecomp;
    }

    /// <summary>Resolution of the system S tS X = B, after <see cref="Decompose"/>.</summary>
    public override void Solve(math_Vector B, math_Vector X)
    {
        if (!_isDecomp)
        {
            throw new NotDoneException("Decomposition must be done");
        }

        int i, j, jj, diagAddr, currAddr;
        double sum;

        // x, b, SMA and NC are the C arrays shifted to be indexed from 1
        Span<double> x = X.AsSpan();
        ReadOnlySpan<double> b = B.AsSpan();
        ReadOnlySpan<double> sma = _sMatrix.AsSpan();
        ReadOnlySpan<int> nc = _nextCoeff.AsSpan();

        // Resolution of Sw = B;
        for (i = 1; i <= RowNumber(); i++)
        {
            diagAddr = _profile[2, i];
            sum = 0;
            for (j = i - _profile[1, i], jj = diagAddr - (i - j); j < i; j++, jj++)
            {
                sum += sma[jj - 1] * x[j - 1];
            }
            x[i - 1] = (b[i - 1] - sum) / sma[diagAddr - 1];
        }

        // Resolution of t(S)X = w;
        for (i = ColNumber(); i >= 1; i--)
        {
            diagAddr = _profile[2, i];
            j = nc[diagAddr - 1];
            sum = 0;
            while (j > 0)
            {
                currAddr = _profile[2, j] - (j - i);
                sum += sma[currAddr - 1] * x[j - 1];
                j = nc[currAddr - 1];
            }
            x[i - 1] = (x[i - 1] - sum) / sma[diagAddr - 1];
        }
    }

    /// <summary>Not implemented: throws Standard_NotImplemented, as OCCT.</summary>
    public override bool Prepare()
    {
        throw new MethodNotImplementedException("FEmTool_ProfileMatrix::Prepare");
    }

    /// <summary>Not implemented: throws Standard_NotImplemented, as OCCT.</summary>
    public override void Solve(math_Vector b, math_Vector init, math_Vector x, math_Vector residual,
                               double tolerance = 1.0e-8, int nbIterations = 50)
    {
        throw new MethodNotImplementedException("FEmTool_ProfileMatrix::Solve");
    }

    /// <summary>MX = H X.</summary>
    public override void Multiplied(math_Vector X, math_Vector mx)
    {
        int i, j, jj, diagAddr, currAddr;
        // m, x, PM and NC are the C arrays shifted to be indexed from 1
        Span<double> m = mx.AsSpan();
        ReadOnlySpan<double> x = X.AsSpan();
        ReadOnlySpan<double> pm = _profileMatrix.AsSpan();
        ReadOnlySpan<int> nc = _nextCoeff.AsSpan();

        for (i = 1; i <= RowNumber(); i++)
        {
            diagAddr = _profile[2, i];
            m[i - 1] = 0;
            for (j = i - _profile[1, i], jj = diagAddr - (i - j); j <= i; j++, jj++)
            {
                m[i - 1] += pm[jj - 1] * x[j - 1];
            }

            currAddr = diagAddr;
            for (j = nc[currAddr - 1]; j > 0; j = nc[currAddr - 1])
            {
                currAddr = _profile[2, j] - (j - i);
                m[i - 1] += pm[currAddr - 1] * x[j - 1];
            }
        }
    }

    public override int RowNumber() => _profile.RowLength();

    public override int ColNumber() => _profile.RowLength();

    /// <summary>True if the coefficient (<paramref name="i"/>, <paramref name="j"/>) is stored.</summary>
    public bool IsInProfile(int i, int j)
    {
        if (j <= i)
        {
            return (i - j) <= _profile[1, i];
        }
        else if ((j - i) <= _profile[1, j])
        {
            return true;
        }

        return false;
    }

    /// <summary>Writes the matrix and its chaining, as OCCT's <c>OutM</c> (which writes to std::cout).</summary>
    public void OutM(TextWriter o)
    {
        int i, j;
        o.Write("Matrix A\n");
        for (i = 1; i <= RowNumber(); i++)
        {
            for (j = 1; j < i - _profile[1, i]; j++)
            {
                o.Write("0 ");
            }

            for (j = _profile[2, i] - _profile[1, i]; j <= _profile[2, i]; j++)
            {
                o.Write(_profileMatrix.Value(j).ToString(CultureInfo.InvariantCulture) + " ");
            }
            o.Write('\n');
        }

        o.Write("NextCoeff\n");
        for (i = 1; i <= _profile[2, RowNumber()]; i++)
        {
            o.Write(_nextCoeff.Value(i).ToString(CultureInfo.InvariantCulture) + " ");
        }
        o.Write('\n');
    }

    /// <summary>Writes the factor S, as OCCT's <c>OutS</c> (which writes to std::cout).</summary>
    public void OutS(TextWriter o)
    {
        int i, j;
        o.Write("Matrix S\n");
        for (i = 1; i <= RowNumber(); i++)
        {
            for (j = 1; j < i - _profile[1, i]; j++)
            {
                o.Write("0 ");
            }

            for (j = _profile[2, i] - _profile[1, i]; j <= _profile[2, i]; j++)
            {
                o.Write(_sMatrix.Value(j).ToString(CultureInfo.InvariantCulture) + " ");
            }
            o.Write('\n');
        }
    }
}
