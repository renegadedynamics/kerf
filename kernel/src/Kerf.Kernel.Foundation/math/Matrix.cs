// Port of: src/FoundationClasses/TKMath/math/math_Matrix.hxx
// Port of: src/FoundationClasses/TKMath/math/math_Matrix.lxx
// Port of: src/FoundationClasses/TKMath/math/math_Matrix.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine;
using System.Globalization;

namespace Kerf.Engine.Numerics;

/// <summary>
/// OCCT's real matrix with arbitrary row and column ranges, for example <c>new math_Matrix(-3, 5, 2, 4)</c>.
/// Values are accessed with <c>M[row, col]</c> (OCCT's <c>M(row, col)</c>) within the ranges; outside them
/// <see cref="OutOfRangeException"/> is thrown. Operations on matrices or vectors of incompatible dimensions throw
/// <see cref="DimensionErrorException"/>.
/// </summary>
/// <remarks>
/// A mutable workspace for ported algorithms (docs/decisions.md, decision 21), and a class:
/// <list type="bullet">
/// <item>C++ copy construction (<c>math_Matrix a(b);</c>, <c>math_Matrix a = b;</c>) is <c>new(b)</c>.</item>
/// <item>C++ assignment (<c>a = b;</c>, including a moved temporary) is <c>a.Initialized(b)</c>, which copies the
/// values and keeps <c>a</c>'s bounds; it throws if the sizes differ, as OCCT does.</item>
/// <item>Compound assignments (<c>+=</c>, <c>-=</c>, <c>*=</c>, <c>/=</c>) change the matrix in place, as in C++.</item>
/// </list>
/// </remarks>
internal class Matrix
{
    private DoubleTab _array;

    /// <summary>A matrix of range [LowerRow..UpperRow, LowerCol..UpperCol], its values zero (OCCT leaves them uninitialised).</summary>
    public Matrix(int lowerRow, int upperRow, int lowerCol, int upperCol)
    {
        _array = new DoubleTab(lowerRow, upperRow, lowerCol, upperCol);
    }

    /// <summary>A matrix of range [LowerRow..UpperRow, LowerCol..UpperCol], every value <paramref name="initialValue"/>.</summary>
    public Matrix(int lowerRow, int upperRow, int lowerCol, int upperCol, double initialValue)
    {
        _array = new DoubleTab(lowerRow, upperRow, lowerCol, upperCol);
        _array.Init(initialValue);
    }

    /// <summary>A matrix sharing the data of a "C array", row by row from its start (OCCT's constructor from <c>void*</c>).</summary>
    public Matrix(double[] tab, int lowerRow, int upperRow, int lowerCol, int upperCol)
    {
        _array = new DoubleTab(tab, lowerRow, upperRow, lowerCol, upperCol);
    }

    /// <summary>A matrix sharing an array's data, row by row from its element <paramref name="index"/>.</summary>
    public Matrix(NCollection_Array1<double> tab, int index, int lowerRow, int upperRow, int lowerCol, int upperCol)
    {
        _array = new DoubleTab(tab, index, lowerRow, upperRow, lowerCol, upperCol);
    }

    /// <summary>A copy (OCCT's copy constructor).</summary>
    public Matrix(Matrix other)
    {
        _array = new DoubleTab(other._array);
    }

    public void Init(double initialValue) => _array.Init(initialValue);

    public int RowNumber() => _array.RowCount;

    public int ColNumber() => _array.ColumnCount;

    public int LowerRow() => _array.LowerRow();

    public int UpperRow() => _array.UpperRow();

    public int LowerCol() => _array.LowerCol();

    public int UpperCol() => _array.UpperCol();

    /// <summary>The determinant, by Gauss decomposition; 0 if the matrix is singular. Throws <see cref="NotSquareException"/> if not square.</summary>
    public double Determinant()
    {
        Gauss sol = new(this);

        if (sol.IsDone)
        {
            return sol.Determinant();
        }
        else
        {
            return 0.0;
        }
    }

    /// <summary>Transposes a square matrix in place. Throws <see cref="NotSquareException"/> if not square.</summary>
    public void Transpose()
    {
        math_NotSquare_Raise_if(RowNumber() != ColNumber(), "math_Matrix::Transpose() - matrix is not square");

        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();
        int lowerCol = _array.LowerCol();
        int upperCol = _array.UpperCol();

        int row = lowerRow;
        int col = lowerCol;
        SetLowerCol(lowerRow);
        double temp;
        for (int i = lowerRow; i <= upperRow; i++)
        {
            for (int j = i; j <= upperCol; j++)
            {
                temp = _array[i, j];
                _array[i, j] = _array[j, i];
                _array[j, i] = temp;
            }
        }

        SetLowerRow(col);
        SetLowerCol(row);
    }

    /// <summary>Inverts the matrix by Gauss decomposition. Throws <see cref="NotSquareException"/> or <see cref="SingularMatrixException"/>.</summary>
    public void Invert()
    {
        math_NotSquare_Raise_if(RowNumber() != ColNumber(), "math_Matrix::Invert() - matrix is not square");

        Gauss sol = new(this);
        if (sol.IsDone)
        {
            sol.Invert(this);
        }
        else
        {
            throw new SingularMatrixException();
        }
    }

    /// <summary>Multiplies every value by <paramref name="right"/>.</summary>
    public void Multiply(double right)
    {
        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();
        int lowerCol = _array.LowerCol();
        int upperCol = _array.UpperCol();

        for (int i = lowerRow; i <= upperRow; i++)
        {
            for (int j = lowerCol; j <= upperCol; j++)
            {
                _array[i, j] = _array[i, j] * right;
            }
        }
    }

    public Matrix Multiplied(double right)
    {
        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();
        int lowerCol = _array.LowerCol();
        int upperCol = _array.UpperCol();

        Matrix result = new(lowerRow, upperRow, lowerCol, upperCol);
        for (int i = lowerRow; i <= upperRow; i++)
        {
            for (int j = lowerCol; j <= upperCol; j++)
            {
                result._array[i, j] = _array[i, j] * right;
            }
        }

        return result;
    }

    public Matrix TMultiplied(double right)
    {
        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();
        int lowerCol = _array.LowerCol();
        int upperCol = _array.UpperCol();

        Matrix result = new(lowerRow, upperRow, lowerCol, upperCol);
        for (int i = lowerRow; i <= upperRow; i++)
        {
            for (int j = lowerCol; j <= upperCol; j++)
            {
                result._array[i, j] = _array[i, j] * right;
            }
        }

        return result;
    }

    /// <summary>Divides every value by <paramref name="right"/>; throws <see cref="ZeroDivisorException"/> if it is zero.</summary>
    public void Divide(double right)
    {
        Standard_DivideByZero_Raise_if(Math.Abs(right) <= RealEpsilon(), "math_Matrix::Divide() - zero divisor");

        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();
        int lowerCol = _array.LowerCol();
        int upperCol = _array.UpperCol();

        for (int i = lowerRow; i <= upperRow; i++)
        {
            for (int j = lowerCol; j <= upperCol; j++)
            {
                _array[i, j] = _array[i, j] / right;
            }
        }
    }

    public Matrix Divided(double right)
    {
        Standard_DivideByZero_Raise_if(Math.Abs(right) <= RealEpsilon(), "math_Matrix::Divided() - zero divisor");
        Matrix temp = Multiplied(1.0 / right);
        return temp;
    }

    public void Add(Matrix right)
    {
        Standard_DimensionError_Raise_if((RowNumber() != right.RowNumber()) || (ColNumber() != right.ColNumber()),
                                         "math_Matrix::Add() - input matrix has different dimensions");

        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();
        int lowerCol = _array.LowerCol();
        int upperCol = _array.UpperCol();

        int i2 = right.LowerRow();
        for (int i = lowerRow; i <= upperRow; i++)
        {
            int j2 = right.LowerCol();
            for (int j = lowerCol; j <= upperCol; j++)
            {
                _array[i, j] = _array[i, j] + right._array[i2, j2];
                j2++;
            }

            i2++;
        }
    }

    public Matrix Added(Matrix right)
    {
        Standard_DimensionError_Raise_if((RowNumber() != right.RowNumber()) || (ColNumber() != right.ColNumber()),
                                         "math_Matrix::Added() - input matrix has different dimensions");

        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();
        int lowerCol = _array.LowerCol();
        int upperCol = _array.UpperCol();

        Matrix result = new(lowerRow, upperRow, lowerCol, upperCol);

        int i2 = right.LowerRow();
        for (int i = lowerRow; i <= upperRow; i++)
        {
            int j2 = right.LowerCol();
            for (int j = lowerCol; j <= upperCol; j++)
            {
                result._array[i, j] = _array[i, j] + right._array[i2, j2];
                j2++;
            }

            i2++;
        }

        return result;
    }

    /// <summary>Sets this matrix to <paramref name="left"/> + <paramref name="right"/>.</summary>
    public void Add(Matrix left, Matrix right)
    {
        Standard_DimensionError_Raise_if((RowNumber() != right.RowNumber()) || (ColNumber() != right.ColNumber())
                                           || (right.RowNumber() != left.RowNumber()) || (right.ColNumber() != left.ColNumber()),
                                         "math_Matrix::Add() - matrices have incompatible dimensions");

        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();
        int lowerCol = _array.LowerCol();
        int upperCol = _array.UpperCol();

        int i1 = left.LowerRow();
        int i2 = right.LowerRow();
        for (int i = lowerRow; i <= upperRow; i++)
        {
            int j1 = left.LowerCol();
            int j2 = right.LowerCol();
            for (int j = lowerCol; j <= upperCol; j++)
            {
                _array[i, j] = left._array[i1, j1] + right._array[i2, j2];
                j1++;
                j2++;
            }

            i1++;
            i2++;
        }
    }

    public void Subtract(Matrix right)
    {
        Standard_DimensionError_Raise_if((RowNumber() != right.RowNumber()) || (ColNumber() != right.ColNumber()),
                                         "math_Matrix::Subtract() - input matrix has different dimensions");

        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();
        int lowerCol = _array.LowerCol();
        int upperCol = _array.UpperCol();

        int i2 = right.LowerRow();
        for (int i = lowerRow; i <= upperRow; i++)
        {
            int j2 = right.LowerCol();
            for (int j = lowerCol; j <= upperCol; j++)
            {
                _array[i, j] = _array[i, j] - right._array[i2, j2];
                j2++;
            }

            i2++;
        }
    }

    public Matrix Subtracted(Matrix right)
    {
        Standard_DimensionError_Raise_if((RowNumber() != right.RowNumber()) || (ColNumber() != right.ColNumber()),
                                         "math_Matrix::Subtracted() - input matrix has different dimensions");

        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();
        int lowerCol = _array.LowerCol();
        int upperCol = _array.UpperCol();

        Matrix result = new(lowerRow, upperRow, lowerCol, upperCol);

        int i2 = right.LowerRow();
        for (int i = lowerRow; i <= upperRow; i++)
        {
            int j2 = right.LowerCol();
            for (int j = lowerCol; j <= upperCol; j++)
            {
                result._array[i, j] = _array[i, j] - right._array[i2, j2];
                j2++;
            }

            i2++;
        }

        return result;
    }

    /// <summary>Sets the values in rows I1..I2 and columns J1..J2 to those of <paramref name="m"/>.</summary>
    public void Set(int i1, int i2, int j1, int j2, Matrix m)
    {
        Standard_DimensionError_Raise_if((i1 > i2) || (j1 > j2) || (i2 - i1 + 1 != m.RowNumber()) || (j2 - j1 + 1 != m.ColNumber()),
                                         "math_Matrix::Set() - invalid indices");

        int ii = m.LowerRow();
        for (int i = i1; i <= i2; i++)
        {
            int jj = m.LowerCol();
            for (int j = j1; j <= j2; j++)
            {
                _array[i, j] = m._array[ii, jj];
                jj++;
            }

            ii++;
        }
    }

    /// <summary>Sets the row <paramref name="row"/> to the vector <paramref name="v"/>.</summary>
    public void SetRow(int row, math_Vector v)
    {
        Standard_DimensionError_Raise_if(ColNumber() != v.Length(), "math_Matrix::SetRow() - input vector has wrong dimensions");

        int lowerCol = _array.LowerCol();
        int upperCol = _array.UpperCol();

        int i = v.Lower();
        for (int index = lowerCol; index <= upperCol; index++)
        {
            _array[row, index] = v[i];
            i++;
        }
    }

    /// <summary>Sets the column <paramref name="col"/> to the vector <paramref name="v"/>.</summary>
    public void SetCol(int col, math_Vector v)
    {
        Standard_DimensionError_Raise_if(RowNumber() != v.Length(), "math_Matrix::SetCol() - input vector has wrong dimensions");

        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();

        int i = v.Lower();
        for (int index = lowerRow; index <= upperRow; index++)
        {
            _array[index, col] = v[i];
            i++;
        }
    }

    /// <summary>Sets the diagonal to <paramref name="value"/>. Throws <see cref="NotSquareException"/> if not square.</summary>
    public void SetDiag(double value)
    {
        math_NotSquare_Raise_if(RowNumber() != ColNumber(), "math_Matrix::SetDiag() - matrix is not square");

        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();

        for (int i = lowerRow; i <= upperRow; i++)
        {
            _array[i, i] = value;
        }
    }

    /// <summary>A copy of the row <paramref name="row"/>.</summary>
    public math_Vector Row(int row)
    {
        int lowerCol = _array.LowerCol();
        int upperCol = _array.UpperCol();

        math_Vector result = new(lowerCol, upperCol);

        for (int index = lowerCol; index <= upperCol; index++)
        {
            result[index] = _array[row, index];
        }

        return result;
    }

    /// <summary>A copy of the column <paramref name="col"/>.</summary>
    public math_Vector Col(int col)
    {
        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();

        math_Vector result = new(lowerRow, upperRow);

        for (int index = lowerRow; index <= upperRow; index++)
        {
            result[index] = _array[index, col];
        }

        return result;
    }

    public void SwapRow(int row1, int row2)
    {
        math_Vector v1 = Row(row1);
        math_Vector v2 = Row(row2);
        SetRow(row1, v2);
        SetRow(row2, v1);
    }

    public void SwapCol(int col1, int col2)
    {
        math_Vector v1 = Col(col1);
        math_Vector v2 = Col(col2);
        SetCol(col1, v2);
        SetCol(col2, v1);
    }

    /// <summary>The transposed matrix, of range [LowerCol..UpperCol, LowerRow..UpperRow].</summary>
    public Matrix Transposed()
    {
        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();
        int lowerCol = _array.LowerCol();
        int upperCol = _array.UpperCol();

        Matrix result = new(lowerCol, upperCol, lowerRow, upperRow);

        for (int i = lowerRow; i <= upperRow; i++)
        {
            for (int j = lowerCol; j <= upperCol; j++)
            {
                result._array[j, i] = _array[i, j];
            }
        }

        return result;
    }

    /// <summary>The inverse. Throws <see cref="NotSquareException"/> or <see cref="SingularMatrixException"/>.</summary>
    public Matrix Inverse()
    {
        Matrix result = new(this);
        result.Invert();
        return result;
    }

    /// <summary>The product of the transpose of this matrix and <paramref name="right"/>.</summary>
    public Matrix TMultiply(Matrix right)
    {
        Standard_DimensionError_Raise_if(RowNumber() != right.RowNumber(), "math_Matrix::TMultiply() - matrices have incompatible dimensions");

        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();
        int lowerCol = _array.LowerCol();
        int upperCol = _array.UpperCol();

        Matrix result = new(lowerCol, upperCol, right.LowerCol(), right.UpperCol());

        double som;
        for (int i = lowerCol; i <= upperCol; i++)
        {
            for (int j2 = right.LowerCol(); j2 <= right.UpperCol(); j2++)
            {
                som = 0.0;
                int i2 = right.LowerRow();
                for (int j = lowerRow; j <= upperRow; j++)
                {
                    som = som + _array[j, i] * right._array[i2, j2];
                    i2++;
                }

                result._array[i, j2] = som;
            }
        }

        return result;
    }

    /// <summary>Sets this matrix to the product of the column vector <paramref name="left"/> and the row vector <paramref name="right"/>.</summary>
    public void Multiply(math_Vector left, math_Vector right)
    {
        Standard_DimensionError_Raise_if((RowNumber() != left.Length()) || (ColNumber() != right.Length()),
                                         "math_Matrix::Multiply() - input vectors have incompatible dimensions");

        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();
        int lowerCol = _array.LowerCol();
        int upperCol = _array.UpperCol();

        for (int i = lowerRow; i <= upperRow; i++)
        {
            for (int j = lowerCol; j <= upperCol; j++)
            {
                _array[i, j] = left[i] * right[j];
            }
        }
    }

    /// <summary>Sets this matrix to the product <paramref name="left"/> * <paramref name="right"/>.</summary>
    public void Multiply(Matrix left, Matrix right)
    {
        Standard_DimensionError_Raise_if((left.ColNumber() != right.RowNumber()) || (RowNumber() != left.RowNumber())
                                           || (ColNumber() != right.ColNumber()),
                                         "math_Matrix::Multiply() - matrices have incompatible dimensions");

        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();
        int lowerCol = _array.LowerCol();
        int upperCol = _array.UpperCol();

        Init(0.0);

        // Cache-friendly i-k-j loop order for row-major matrix storage.
        int i1 = left.LowerRow();
        for (int i = lowerRow; i <= upperRow; i++)
        {
            int j1 = left.LowerCol();
            int i2 = right.LowerRow();
            for (int k = left.LowerCol(); k <= left.UpperCol(); k++)
            {
                double leftVal = left._array[i1, j1];
                int j2 = right.LowerCol();
                for (int j = lowerCol; j <= upperCol; j++)
                {
                    _array[i, j] += leftVal * right._array[i2, j2];
                    j2++;
                }

                j1++;
                i2++;
            }

            i1++;
        }
    }

    /// <summary>Sets this matrix to the product of the transpose of <paramref name="tLeft"/> and <paramref name="right"/>.</summary>
    public void TMultiply(Matrix tLeft, Matrix right)
    {
        Standard_DimensionError_Raise_if((tLeft.RowNumber() != right.RowNumber()) || (RowNumber() != tLeft.ColNumber())
                                           || (ColNumber() != right.ColNumber()),
                                         "math_Matrix::TMultiply() - matrices have incompatible dimensions");

        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();
        int lowerCol = _array.LowerCol();
        int upperCol = _array.UpperCol();

        double som;
        int i1 = tLeft.LowerCol();
        for (int i = lowerRow; i <= upperRow; i++)
        {
            int j2 = right.LowerCol();
            for (int j = lowerCol; j <= upperCol; j++)
            {
                som = 0.0;
                int j1 = tLeft.LowerRow();
                int i2 = right.LowerRow();
                for (int k = tLeft.LowerRow(); k <= tLeft.UpperRow(); k++)
                {
                    som = som + tLeft._array[j1, i1] * right._array[i2, j2];
                    j1++;
                    i2++;
                }

                _array[i, j] = som;
                j2++;
            }

            i1++;
        }
    }

    /// <summary>Sets this matrix to <paramref name="left"/> - <paramref name="right"/>.</summary>
    public void Subtract(Matrix left, Matrix right)
    {
        Standard_DimensionError_Raise_if((RowNumber() != right.RowNumber()) || (ColNumber() != right.ColNumber())
                                           || (right.RowNumber() != left.RowNumber()) || (right.ColNumber() != left.ColNumber()),
                                         "math_Matrix::Subtract() - matrices have incompatible dimensions");

        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();
        int lowerCol = _array.LowerCol();
        int upperCol = _array.UpperCol();

        int i1 = left.LowerRow();
        int i2 = right.LowerRow();
        for (int i = lowerRow; i <= upperRow; i++)
        {
            int j1 = left.LowerCol();
            int j2 = right.LowerCol();
            for (int j = lowerCol; j <= upperCol; j++)
            {
                _array[i, j] = left._array[i1, j1] - right._array[i2, j2];
                j1++;
                j2++;
            }

            i1++;
            i2++;
        }
    }

    /// <summary>The value at (Row, Col), for reading and writing.</summary>
    public ref double Value(int row, int col) => ref _array.Value(row, col);

    /// <summary>OCCT's <c>operator()(Row, Col)</c>, for reading and writing.</summary>
    public ref double this[int row, int col] => ref _array.Value(row, col);

    /// <summary>
    /// Copies the values of <paramref name="other"/>, keeping this matrix's bounds (OCCT's <c>operator=</c>).
    /// Throws <see cref="DimensionMismatchException"/> if the sizes differ.
    /// </summary>
    public Matrix Initialized(Matrix other)
    {
        other._array.Copy(_array);
        return this;
    }

    /// <summary>Sets this matrix to its product with <paramref name="right"/>.</summary>
    public void Multiply(Matrix right)
    {
        Standard_DimensionError_Raise_if(ColNumber() != right.RowNumber(), "math_Matrix::Multiply() - input matrix has incompatible dimensions");

        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();
        int lowerCol = _array.LowerCol();
        int upperCol = _array.UpperCol();

        // Create a temporary copy to avoid corrupting our own data during calculation
        Matrix temp = new(this);
        if (ReferenceEquals(this, right))
        {
            Multiply(temp, temp);
            return;
        }

        double som;
        for (int i = lowerRow; i <= upperRow; i++)
        {
            for (int j2 = right.LowerCol(); j2 <= right.UpperCol(); j2++)
            {
                som = 0.0;
                int i2 = right.LowerRow();
                for (int j = lowerCol; j <= upperCol; j++)
                {
                    som += temp._array[i, j] * right._array[i2, j2];
                    i2++;
                }

                // OCCT writes column J2 of Right's column range into this matrix, so a Right whose column range
                // differs from this matrix's (not square, or other bounds) writes the wrong columns or out of range.
                _array[i, j2] = som;
            }
        }
    }

    public Matrix Multiplied(Matrix right)
    {
        Standard_DimensionError_Raise_if(ColNumber() != right.RowNumber(), "math_Matrix::Multiplied() - matrices have incompatible dimensions");

        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();
        int lowerCol = _array.LowerCol();
        int upperCol = _array.UpperCol();

        // Initialize result to zero
        Matrix result = new(lowerRow, upperRow, right.LowerCol(), right.UpperCol(), 0.0);

        // Cache-friendly i-k-j loop order for row-major matrix storage.
        for (int i = lowerRow; i <= upperRow; i++)
        {
            int i2 = right.LowerRow();
            for (int j = lowerCol; j <= upperCol; j++)
            {
                double leftVal = _array[i, j];
                int j2 = right.LowerCol();
                for (int jr = right.LowerCol(); jr <= right.UpperCol(); jr++)
                {
                    result._array[i, jr] += leftVal * right._array[i2, j2];
                    j2++;
                }

                i2++;
            }
        }

        return result;
    }

    /// <summary>The product of this matrix and the vector <paramref name="right"/>.</summary>
    public math_Vector Multiplied(math_Vector right)
    {
        Standard_DimensionError_Raise_if(ColNumber() != right.Length(), "math_Matrix::Multiplied() - input vector has incompatible dimensions");

        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();
        int lowerCol = _array.LowerCol();
        int upperCol = _array.UpperCol();

        math_Vector result = new(lowerRow, upperRow);

        for (int i = lowerRow; i <= upperRow; i++)
        {
            result[i] = 0.0;
            int ii = right.Lower();
            for (int j = lowerCol; j <= upperCol; j++)
            {
                result[i] = result[i] + _array[i, j] * right[ii];
                ii++;
            }
        }

        return result;
    }

    public Matrix Opposite()
    {
        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();
        int lowerCol = _array.LowerCol();
        int upperCol = _array.UpperCol();

        Matrix result = new(lowerRow, upperRow, lowerCol, upperCol);

        for (int i = lowerRow; i <= upperRow; i++)
        {
            for (int j = lowerCol; j <= upperCol; j++)
            {
                result._array[i, j] = -_array[i, j];
            }
        }

        return result;
    }

    /// <summary>Prints the matrix, as OCCT's <c>Dump</c>.</summary>
    public void Dump(TextWriter o)
    {
        int lowerRow = _array.LowerRow();
        int upperRow = _array.UpperRow();
        int lowerCol = _array.LowerCol();
        int upperCol = _array.UpperCol();

        o.Write("math_Matrix of RowNumber = " + RowNumber());
        o.Write(" and ColNumber = " + ColNumber() + "\n");

        for (int i = lowerRow; i <= upperRow; i++)
        {
            for (int j = lowerCol; j <= upperCol; j++)
            {
                o.Write("math_Matrix ( " + i + ", " + j + " ) = ");
                o.Write(_array[i, j].ToString(CultureInfo.InvariantCulture) + "\n");
            }
        }
    }

    public override string ToString()
    {
        using var writer = new StringWriter(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }

    /// <summary>The values row by row, without copying (in place of OCCT's pointer to the first value).</summary>
    public Span<double> AsSpan() => _array.AsSpan();

    protected internal void SetLowerRow(int lowerRow) => _array.SetLowerRow(lowerRow);

    protected internal void SetLowerCol(int lowerCol) => _array.SetLowerCol(lowerCol);

    protected internal void SetLower(int lowerRow, int lowerCol)
    {
        SetLowerRow(lowerRow);
        SetLowerCol(lowerCol);
    }

    // Operators. The compound assignments change the matrix in place, as OCCT's do.

    public void operator *=(double right) => Multiply(right);

    public void operator /=(double right) => Divide(right);

    public void operator +=(Matrix right) => Add(right);

    public void operator -=(Matrix right) => Subtract(right);

    public void operator *=(Matrix right) => Multiply(right);

    public static Matrix operator *(Matrix left, double right) => left.Multiplied(right);

    public static Matrix operator *(double left, Matrix right) => right.Multiplied(left);

    public static Matrix operator /(Matrix left, double right) => left.Divided(right);

    public static Matrix operator +(Matrix left, Matrix right) => left.Added(right);

    public static Matrix operator -(Matrix left, Matrix right) => left.Subtracted(right);

    public static Matrix operator -(Matrix matrix) => matrix.Opposite();

    public static Matrix operator *(Matrix left, Matrix right) => left.Multiplied(right);

    public static math_Vector operator *(Matrix left, math_Vector right) => left.Multiplied(right);
}
