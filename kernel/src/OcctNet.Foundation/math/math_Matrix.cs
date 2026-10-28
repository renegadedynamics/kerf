// Port of: src/FoundationClasses/TKMath/math/math_Matrix.hxx
// Port of: src/FoundationClasses/TKMath/math/math_Matrix.lxx
// Port of: src/FoundationClasses/TKMath/math/math_Matrix.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;

namespace Occt;

/// <summary>
/// OCCT's real matrix with arbitrary row and column ranges, for example <c>new math_Matrix(-3, 5, 2, 4)</c>.
/// Values are accessed with <c>M[row, col]</c> (OCCT's <c>M(row, col)</c>) within the ranges; outside them
/// <see cref="Standard_OutOfRange"/> is thrown. Operations on matrices or vectors of incompatible dimensions throw
/// <see cref="Standard_DimensionError"/>.
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
public class math_Matrix
{
    private math_DoubleTab Array;

    /// <summary>A matrix of range [LowerRow..UpperRow, LowerCol..UpperCol], its values zero (OCCT leaves them uninitialised).</summary>
    public math_Matrix(int LowerRow, int UpperRow, int LowerCol, int UpperCol)
    {
        Array = new math_DoubleTab(LowerRow, UpperRow, LowerCol, UpperCol);
    }

    /// <summary>A matrix of range [LowerRow..UpperRow, LowerCol..UpperCol], every value <paramref name="InitialValue"/>.</summary>
    public math_Matrix(int LowerRow, int UpperRow, int LowerCol, int UpperCol, double InitialValue)
    {
        Array = new math_DoubleTab(LowerRow, UpperRow, LowerCol, UpperCol);
        Array.Init(InitialValue);
    }

    /// <summary>A matrix sharing the data of a "C array", row by row from its start (OCCT's constructor from <c>void*</c>).</summary>
    public math_Matrix(double[] Tab, int LowerRow, int UpperRow, int LowerCol, int UpperCol)
    {
        Array = new math_DoubleTab(Tab, LowerRow, UpperRow, LowerCol, UpperCol);
    }

    /// <summary>A matrix sharing an array's data, row by row from its element <paramref name="theIndex"/>.</summary>
    public math_Matrix(NCollection_Array1<double> Tab, int theIndex, int LowerRow, int UpperRow, int LowerCol, int UpperCol)
    {
        Array = new math_DoubleTab(Tab, theIndex, LowerRow, UpperRow, LowerCol, UpperCol);
    }

    /// <summary>A copy (OCCT's copy constructor).</summary>
    public math_Matrix(math_Matrix Other)
    {
        Array = new math_DoubleTab(Other.Array);
    }

    public void Init(double InitialValue) => Array.Init(InitialValue);

    public int RowNumber() => Array.NbRows();

    public int ColNumber() => Array.NbColumns();

    public int LowerRow() => Array.LowerRow();

    public int UpperRow() => Array.UpperRow();

    public int LowerCol() => Array.LowerCol();

    public int UpperCol() => Array.UpperCol();

    /// <summary>The determinant, by Gauss decomposition; 0 if the matrix is singular. Throws <see cref="math_NotSquare"/> if not square.</summary>
    public double Determinant()
    {
        math_Gauss Sol = new(this);

        if (Sol.IsDone())
        {
            return Sol.Determinant();
        }
        else
        {
            return 0.0;
        }
    }

    /// <summary>Transposes a square matrix in place. Throws <see cref="math_NotSquare"/> if not square.</summary>
    public void Transpose()
    {
        math_NotSquare_Raise_if(RowNumber() != ColNumber(), "math_Matrix::Transpose() - matrix is not square");

        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();
        int aLowerCol = Array.LowerCol();
        int anUpperCol = Array.UpperCol();

        int Row = aLowerRow;
        int Col = aLowerCol;
        SetLowerCol(aLowerRow);
        double Temp;
        for (int I = aLowerRow; I <= anUpperRow; I++)
        {
            for (int J = I; J <= anUpperCol; J++)
            {
                Temp = Array[I, J];
                Array[I, J] = Array[J, I];
                Array[J, I] = Temp;
            }
        }

        SetLowerRow(Col);
        SetLowerCol(Row);
    }

    /// <summary>Inverts the matrix by Gauss decomposition. Throws <see cref="math_NotSquare"/> or <see cref="math_SingularMatrix"/>.</summary>
    public void Invert()
    {
        math_NotSquare_Raise_if(RowNumber() != ColNumber(), "math_Matrix::Invert() - matrix is not square");

        math_Gauss Sol = new(this);
        if (Sol.IsDone())
        {
            Sol.Invert(this);
        }
        else
        {
            throw new math_SingularMatrix();
        }
    }

    /// <summary>Multiplies every value by <paramref name="Right"/>.</summary>
    public void Multiply(double Right)
    {
        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();
        int aLowerCol = Array.LowerCol();
        int anUpperCol = Array.UpperCol();

        for (int I = aLowerRow; I <= anUpperRow; I++)
        {
            for (int J = aLowerCol; J <= anUpperCol; J++)
            {
                Array[I, J] = Array[I, J] * Right;
            }
        }
    }

    public math_Matrix Multiplied(double Right)
    {
        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();
        int aLowerCol = Array.LowerCol();
        int anUpperCol = Array.UpperCol();

        math_Matrix Result = new(aLowerRow, anUpperRow, aLowerCol, anUpperCol);
        for (int I = aLowerRow; I <= anUpperRow; I++)
        {
            for (int J = aLowerCol; J <= anUpperCol; J++)
            {
                Result.Array[I, J] = Array[I, J] * Right;
            }
        }

        return Result;
    }

    public math_Matrix TMultiplied(double Right)
    {
        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();
        int aLowerCol = Array.LowerCol();
        int anUpperCol = Array.UpperCol();

        math_Matrix Result = new(aLowerRow, anUpperRow, aLowerCol, anUpperCol);
        for (int I = aLowerRow; I <= anUpperRow; I++)
        {
            for (int J = aLowerCol; J <= anUpperCol; J++)
            {
                Result.Array[I, J] = Array[I, J] * Right;
            }
        }

        return Result;
    }

    /// <summary>Divides every value by <paramref name="Right"/>; throws <see cref="Standard_DivideByZero"/> if it is zero.</summary>
    public void Divide(double Right)
    {
        Standard_DivideByZero_Raise_if(Math.Abs(Right) <= RealEpsilon(), "math_Matrix::Divide() - zero divisor");

        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();
        int aLowerCol = Array.LowerCol();
        int anUpperCol = Array.UpperCol();

        for (int I = aLowerRow; I <= anUpperRow; I++)
        {
            for (int J = aLowerCol; J <= anUpperCol; J++)
            {
                Array[I, J] = Array[I, J] / Right;
            }
        }
    }

    public math_Matrix Divided(double Right)
    {
        Standard_DivideByZero_Raise_if(Math.Abs(Right) <= RealEpsilon(), "math_Matrix::Divided() - zero divisor");
        math_Matrix temp = Multiplied(1.0 / Right);
        return temp;
    }

    public void Add(math_Matrix Right)
    {
        Standard_DimensionError_Raise_if((RowNumber() != Right.RowNumber()) || (ColNumber() != Right.ColNumber()),
                                         "math_Matrix::Add() - input matrix has different dimensions");

        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();
        int aLowerCol = Array.LowerCol();
        int anUpperCol = Array.UpperCol();

        int I2 = Right.LowerRow();
        for (int I = aLowerRow; I <= anUpperRow; I++)
        {
            int J2 = Right.LowerCol();
            for (int J = aLowerCol; J <= anUpperCol; J++)
            {
                Array[I, J] = Array[I, J] + Right.Array[I2, J2];
                J2++;
            }

            I2++;
        }
    }

    public math_Matrix Added(math_Matrix Right)
    {
        Standard_DimensionError_Raise_if((RowNumber() != Right.RowNumber()) || (ColNumber() != Right.ColNumber()),
                                         "math_Matrix::Added() - input matrix has different dimensions");

        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();
        int aLowerCol = Array.LowerCol();
        int anUpperCol = Array.UpperCol();

        math_Matrix Result = new(aLowerRow, anUpperRow, aLowerCol, anUpperCol);

        int I2 = Right.LowerRow();
        for (int I = aLowerRow; I <= anUpperRow; I++)
        {
            int J2 = Right.LowerCol();
            for (int J = aLowerCol; J <= anUpperCol; J++)
            {
                Result.Array[I, J] = Array[I, J] + Right.Array[I2, J2];
                J2++;
            }

            I2++;
        }

        return Result;
    }

    /// <summary>Sets this matrix to <paramref name="Left"/> + <paramref name="Right"/>.</summary>
    public void Add(math_Matrix Left, math_Matrix Right)
    {
        Standard_DimensionError_Raise_if((RowNumber() != Right.RowNumber()) || (ColNumber() != Right.ColNumber())
                                           || (Right.RowNumber() != Left.RowNumber()) || (Right.ColNumber() != Left.ColNumber()),
                                         "math_Matrix::Add() - matrices have incompatible dimensions");

        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();
        int aLowerCol = Array.LowerCol();
        int anUpperCol = Array.UpperCol();

        int I1 = Left.LowerRow();
        int I2 = Right.LowerRow();
        for (int I = aLowerRow; I <= anUpperRow; I++)
        {
            int J1 = Left.LowerCol();
            int J2 = Right.LowerCol();
            for (int J = aLowerCol; J <= anUpperCol; J++)
            {
                Array[I, J] = Left.Array[I1, J1] + Right.Array[I2, J2];
                J1++;
                J2++;
            }

            I1++;
            I2++;
        }
    }

    public void Subtract(math_Matrix Right)
    {
        Standard_DimensionError_Raise_if((RowNumber() != Right.RowNumber()) || (ColNumber() != Right.ColNumber()),
                                         "math_Matrix::Subtract() - input matrix has different dimensions");

        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();
        int aLowerCol = Array.LowerCol();
        int anUpperCol = Array.UpperCol();

        int I2 = Right.LowerRow();
        for (int I = aLowerRow; I <= anUpperRow; I++)
        {
            int J2 = Right.LowerCol();
            for (int J = aLowerCol; J <= anUpperCol; J++)
            {
                Array[I, J] = Array[I, J] - Right.Array[I2, J2];
                J2++;
            }

            I2++;
        }
    }

    public math_Matrix Subtracted(math_Matrix Right)
    {
        Standard_DimensionError_Raise_if((RowNumber() != Right.RowNumber()) || (ColNumber() != Right.ColNumber()),
                                         "math_Matrix::Subtracted() - input matrix has different dimensions");

        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();
        int aLowerCol = Array.LowerCol();
        int anUpperCol = Array.UpperCol();

        math_Matrix Result = new(aLowerRow, anUpperRow, aLowerCol, anUpperCol);

        int I2 = Right.LowerRow();
        for (int I = aLowerRow; I <= anUpperRow; I++)
        {
            int J2 = Right.LowerCol();
            for (int J = aLowerCol; J <= anUpperCol; J++)
            {
                Result.Array[I, J] = Array[I, J] - Right.Array[I2, J2];
                J2++;
            }

            I2++;
        }

        return Result;
    }

    /// <summary>Sets the values in rows I1..I2 and columns J1..J2 to those of <paramref name="M"/>.</summary>
    public void Set(int I1, int I2, int J1, int J2, math_Matrix M)
    {
        Standard_DimensionError_Raise_if((I1 > I2) || (J1 > J2) || (I2 - I1 + 1 != M.RowNumber()) || (J2 - J1 + 1 != M.ColNumber()),
                                         "math_Matrix::Set() - invalid indices");

        int II = M.LowerRow();
        for (int I = I1; I <= I2; I++)
        {
            int JJ = M.LowerCol();
            for (int J = J1; J <= J2; J++)
            {
                Array[I, J] = M.Array[II, JJ];
                JJ++;
            }

            II++;
        }
    }

    /// <summary>Sets the row <paramref name="Row"/> to the vector <paramref name="V"/>.</summary>
    public void SetRow(int Row, math_Vector V)
    {
        Standard_DimensionError_Raise_if(ColNumber() != V.Length(), "math_Matrix::SetRow() - input vector has wrong dimensions");

        int aLowerCol = Array.LowerCol();
        int anUpperCol = Array.UpperCol();

        int I = V.Lower();
        for (int Index = aLowerCol; Index <= anUpperCol; Index++)
        {
            Array[Row, Index] = V[I];
            I++;
        }
    }

    /// <summary>Sets the column <paramref name="Col"/> to the vector <paramref name="V"/>.</summary>
    public void SetCol(int Col, math_Vector V)
    {
        Standard_DimensionError_Raise_if(RowNumber() != V.Length(), "math_Matrix::SetCol() - input vector has wrong dimensions");

        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();

        int I = V.Lower();
        for (int Index = aLowerRow; Index <= anUpperRow; Index++)
        {
            Array[Index, Col] = V[I];
            I++;
        }
    }

    /// <summary>Sets the diagonal to <paramref name="Value"/>. Throws <see cref="math_NotSquare"/> if not square.</summary>
    public void SetDiag(double Value)
    {
        math_NotSquare_Raise_if(RowNumber() != ColNumber(), "math_Matrix::SetDiag() - matrix is not square");

        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();

        for (int I = aLowerRow; I <= anUpperRow; I++)
        {
            Array[I, I] = Value;
        }
    }

    /// <summary>A copy of the row <paramref name="Row"/>.</summary>
    public math_Vector Row(int Row)
    {
        int aLowerCol = Array.LowerCol();
        int anUpperCol = Array.UpperCol();

        math_Vector Result = new(aLowerCol, anUpperCol);

        for (int Index = aLowerCol; Index <= anUpperCol; Index++)
        {
            Result[Index] = Array[Row, Index];
        }

        return Result;
    }

    /// <summary>A copy of the column <paramref name="Col"/>.</summary>
    public math_Vector Col(int Col)
    {
        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();

        math_Vector Result = new(aLowerRow, anUpperRow);

        for (int Index = aLowerRow; Index <= anUpperRow; Index++)
        {
            Result[Index] = Array[Index, Col];
        }

        return Result;
    }

    public void SwapRow(int Row1, int Row2)
    {
        math_Vector V1 = Row(Row1);
        math_Vector V2 = Row(Row2);
        SetRow(Row1, V2);
        SetRow(Row2, V1);
    }

    public void SwapCol(int Col1, int Col2)
    {
        math_Vector V1 = Col(Col1);
        math_Vector V2 = Col(Col2);
        SetCol(Col1, V2);
        SetCol(Col2, V1);
    }

    /// <summary>The transposed matrix, of range [LowerCol..UpperCol, LowerRow..UpperRow].</summary>
    public math_Matrix Transposed()
    {
        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();
        int aLowerCol = Array.LowerCol();
        int anUpperCol = Array.UpperCol();

        math_Matrix Result = new(aLowerCol, anUpperCol, aLowerRow, anUpperRow);

        for (int I = aLowerRow; I <= anUpperRow; I++)
        {
            for (int J = aLowerCol; J <= anUpperCol; J++)
            {
                Result.Array[J, I] = Array[I, J];
            }
        }

        return Result;
    }

    /// <summary>The inverse. Throws <see cref="math_NotSquare"/> or <see cref="math_SingularMatrix"/>.</summary>
    public math_Matrix Inverse()
    {
        math_Matrix Result = new(this);
        Result.Invert();
        return Result;
    }

    /// <summary>The product of the transpose of this matrix and <paramref name="Right"/>.</summary>
    public math_Matrix TMultiply(math_Matrix Right)
    {
        Standard_DimensionError_Raise_if(RowNumber() != Right.RowNumber(), "math_Matrix::TMultiply() - matrices have incompatible dimensions");

        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();
        int aLowerCol = Array.LowerCol();
        int anUpperCol = Array.UpperCol();

        math_Matrix Result = new(aLowerCol, anUpperCol, Right.LowerCol(), Right.UpperCol());

        double Som;
        for (int I = aLowerCol; I <= anUpperCol; I++)
        {
            for (int J2 = Right.LowerCol(); J2 <= Right.UpperCol(); J2++)
            {
                Som = 0.0;
                int I2 = Right.LowerRow();
                for (int J = aLowerRow; J <= anUpperRow; J++)
                {
                    Som = Som + Array[J, I] * Right.Array[I2, J2];
                    I2++;
                }

                Result.Array[I, J2] = Som;
            }
        }

        return Result;
    }

    /// <summary>Sets this matrix to the product of the column vector <paramref name="Left"/> and the row vector <paramref name="Right"/>.</summary>
    public void Multiply(math_Vector Left, math_Vector Right)
    {
        Standard_DimensionError_Raise_if((RowNumber() != Left.Length()) || (ColNumber() != Right.Length()),
                                         "math_Matrix::Multiply() - input vectors have incompatible dimensions");

        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();
        int aLowerCol = Array.LowerCol();
        int anUpperCol = Array.UpperCol();

        for (int I = aLowerRow; I <= anUpperRow; I++)
        {
            for (int J = aLowerCol; J <= anUpperCol; J++)
            {
                Array[I, J] = Left[I] * Right[J];
            }
        }
    }

    /// <summary>Sets this matrix to the product <paramref name="Left"/> * <paramref name="Right"/>.</summary>
    public void Multiply(math_Matrix Left, math_Matrix Right)
    {
        Standard_DimensionError_Raise_if((Left.ColNumber() != Right.RowNumber()) || (RowNumber() != Left.RowNumber())
                                           || (ColNumber() != Right.ColNumber()),
                                         "math_Matrix::Multiply() - matrices have incompatible dimensions");

        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();
        int aLowerCol = Array.LowerCol();
        int anUpperCol = Array.UpperCol();

        Init(0.0);

        // Cache-friendly i-k-j loop order for row-major matrix storage.
        int I1 = Left.LowerRow();
        for (int I = aLowerRow; I <= anUpperRow; I++)
        {
            int J1 = Left.LowerCol();
            int I2 = Right.LowerRow();
            for (int K = Left.LowerCol(); K <= Left.UpperCol(); K++)
            {
                double aLeftVal = Left.Array[I1, J1];
                int J2 = Right.LowerCol();
                for (int J = aLowerCol; J <= anUpperCol; J++)
                {
                    Array[I, J] += aLeftVal * Right.Array[I2, J2];
                    J2++;
                }

                J1++;
                I2++;
            }

            I1++;
        }
    }

    /// <summary>Sets this matrix to the product of the transpose of <paramref name="TLeft"/> and <paramref name="Right"/>.</summary>
    public void TMultiply(math_Matrix TLeft, math_Matrix Right)
    {
        Standard_DimensionError_Raise_if((TLeft.RowNumber() != Right.RowNumber()) || (RowNumber() != TLeft.ColNumber())
                                           || (ColNumber() != Right.ColNumber()),
                                         "math_Matrix::TMultiply() - matrices have incompatible dimensions");

        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();
        int aLowerCol = Array.LowerCol();
        int anUpperCol = Array.UpperCol();

        double Som;
        int I1 = TLeft.LowerCol();
        for (int I = aLowerRow; I <= anUpperRow; I++)
        {
            int J2 = Right.LowerCol();
            for (int J = aLowerCol; J <= anUpperCol; J++)
            {
                Som = 0.0;
                int J1 = TLeft.LowerRow();
                int I2 = Right.LowerRow();
                for (int K = TLeft.LowerRow(); K <= TLeft.UpperRow(); K++)
                {
                    Som = Som + TLeft.Array[J1, I1] * Right.Array[I2, J2];
                    J1++;
                    I2++;
                }

                Array[I, J] = Som;
                J2++;
            }

            I1++;
        }
    }

    /// <summary>Sets this matrix to <paramref name="Left"/> - <paramref name="Right"/>.</summary>
    public void Subtract(math_Matrix Left, math_Matrix Right)
    {
        Standard_DimensionError_Raise_if((RowNumber() != Right.RowNumber()) || (ColNumber() != Right.ColNumber())
                                           || (Right.RowNumber() != Left.RowNumber()) || (Right.ColNumber() != Left.ColNumber()),
                                         "math_Matrix::Subtract() - matrices have incompatible dimensions");

        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();
        int aLowerCol = Array.LowerCol();
        int anUpperCol = Array.UpperCol();

        int I1 = Left.LowerRow();
        int I2 = Right.LowerRow();
        for (int I = aLowerRow; I <= anUpperRow; I++)
        {
            int J1 = Left.LowerCol();
            int J2 = Right.LowerCol();
            for (int J = aLowerCol; J <= anUpperCol; J++)
            {
                Array[I, J] = Left.Array[I1, J1] - Right.Array[I2, J2];
                J1++;
                J2++;
            }

            I1++;
            I2++;
        }
    }

    /// <summary>The value at (Row, Col), for reading and writing.</summary>
    public ref double Value(int Row, int Col) => ref Array.Value(Row, Col);

    /// <summary>OCCT's <c>operator()(Row, Col)</c>, for reading and writing.</summary>
    public ref double this[int Row, int Col] => ref Array.Value(Row, Col);

    /// <summary>
    /// Copies the values of <paramref name="Other"/>, keeping this matrix's bounds (OCCT's <c>operator=</c>).
    /// Throws <see cref="Standard_DimensionMismatch"/> if the sizes differ.
    /// </summary>
    public math_Matrix Initialized(math_Matrix Other)
    {
        Other.Array.Copy(Array);
        return this;
    }

    /// <summary>Sets this matrix to its product with <paramref name="Right"/>.</summary>
    public void Multiply(math_Matrix Right)
    {
        Standard_DimensionError_Raise_if(ColNumber() != Right.RowNumber(), "math_Matrix::Multiply() - input matrix has incompatible dimensions");

        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();
        int aLowerCol = Array.LowerCol();
        int anUpperCol = Array.UpperCol();

        // Create a temporary copy to avoid corrupting our own data during calculation
        math_Matrix aTemp = new(this);
        if (ReferenceEquals(this, Right))
        {
            Multiply(aTemp, aTemp);
            return;
        }

        double Som;
        for (int I = aLowerRow; I <= anUpperRow; I++)
        {
            for (int J2 = Right.LowerCol(); J2 <= Right.UpperCol(); J2++)
            {
                Som = 0.0;
                int I2 = Right.LowerRow();
                for (int J = aLowerCol; J <= anUpperCol; J++)
                {
                    Som += aTemp.Array[I, J] * Right.Array[I2, J2];
                    I2++;
                }

                Array[I, J2] = Som;
            }
        }
    }

    public math_Matrix Multiplied(math_Matrix Right)
    {
        Standard_DimensionError_Raise_if(ColNumber() != Right.RowNumber(), "math_Matrix::Multiplied() - matrices have incompatible dimensions");

        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();
        int aLowerCol = Array.LowerCol();
        int anUpperCol = Array.UpperCol();

        // Initialize result to zero
        math_Matrix Result = new(aLowerRow, anUpperRow, Right.LowerCol(), Right.UpperCol(), 0.0);

        // Cache-friendly i-k-j loop order for row-major matrix storage.
        for (int I = aLowerRow; I <= anUpperRow; I++)
        {
            int I2 = Right.LowerRow();
            for (int J = aLowerCol; J <= anUpperCol; J++)
            {
                double aLeftVal = Array[I, J];
                int J2 = Right.LowerCol();
                for (int JR = Right.LowerCol(); JR <= Right.UpperCol(); JR++)
                {
                    Result.Array[I, JR] += aLeftVal * Right.Array[I2, J2];
                    J2++;
                }

                I2++;
            }
        }

        return Result;
    }

    /// <summary>The product of this matrix and the vector <paramref name="Right"/>.</summary>
    public math_Vector Multiplied(math_Vector Right)
    {
        Standard_DimensionError_Raise_if(ColNumber() != Right.Length(), "math_Matrix::Multiplied() - input vector has incompatible dimensions");

        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();
        int aLowerCol = Array.LowerCol();
        int anUpperCol = Array.UpperCol();

        math_Vector Result = new(aLowerRow, anUpperRow);

        for (int I = aLowerRow; I <= anUpperRow; I++)
        {
            Result[I] = 0.0;
            int II = Right.Lower();
            for (int J = aLowerCol; J <= anUpperCol; J++)
            {
                Result[I] = Result[I] + Array[I, J] * Right[II];
                II++;
            }
        }

        return Result;
    }

    public math_Matrix Opposite()
    {
        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();
        int aLowerCol = Array.LowerCol();
        int anUpperCol = Array.UpperCol();

        math_Matrix Result = new(aLowerRow, anUpperRow, aLowerCol, anUpperCol);

        for (int I = aLowerRow; I <= anUpperRow; I++)
        {
            for (int J = aLowerCol; J <= anUpperCol; J++)
            {
                Result.Array[I, J] = -Array[I, J];
            }
        }

        return Result;
    }

    /// <summary>Prints the matrix, as OCCT's <c>Dump</c>.</summary>
    public void Dump(TextWriter o)
    {
        int aLowerRow = Array.LowerRow();
        int anUpperRow = Array.UpperRow();
        int aLowerCol = Array.LowerCol();
        int anUpperCol = Array.UpperCol();

        o.Write("math_Matrix of RowNumber = " + RowNumber());
        o.Write(" and ColNumber = " + ColNumber() + "\n");

        for (int I = aLowerRow; I <= anUpperRow; I++)
        {
            for (int J = aLowerCol; J <= anUpperCol; J++)
            {
                o.Write("math_Matrix ( " + I + ", " + J + " ) = ");
                o.Write(Array[I, J].ToString(CultureInfo.InvariantCulture) + "\n");
            }
        }
    }

    public override string ToString()
    {
        using var aWriter = new StringWriter(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }

    /// <summary>The values row by row, without copying (in place of OCCT's pointer to the first value).</summary>
    public Span<double> AsSpan() => Array.AsSpan();

    protected internal void SetLowerRow(int LowerRow) => Array.SetLowerRow(LowerRow);

    protected internal void SetLowerCol(int LowerCol) => Array.SetLowerCol(LowerCol);

    protected internal void SetLower(int LowerRow, int LowerCol)
    {
        SetLowerRow(LowerRow);
        SetLowerCol(LowerCol);
    }

    // Operators. The compound assignments change the matrix in place, as OCCT's do.

    public void operator *=(double Right) => Multiply(Right);

    public void operator /=(double Right) => Divide(Right);

    public void operator +=(math_Matrix Right) => Add(Right);

    public void operator -=(math_Matrix Right) => Subtract(Right);

    public void operator *=(math_Matrix Right) => Multiply(Right);

    public static math_Matrix operator *(math_Matrix Left, double Right) => Left.Multiplied(Right);

    public static math_Matrix operator *(double Left, math_Matrix Right) => Right.Multiplied(Left);

    public static math_Matrix operator /(math_Matrix Left, double Right) => Left.Divided(Right);

    public static math_Matrix operator +(math_Matrix Left, math_Matrix Right) => Left.Added(Right);

    public static math_Matrix operator -(math_Matrix Left, math_Matrix Right) => Left.Subtracted(Right);

    public static math_Matrix operator -(math_Matrix Matrix) => Matrix.Opposite();

    public static math_Matrix operator *(math_Matrix Left, math_Matrix Right) => Left.Multiplied(Right);

    public static math_Vector operator *(math_Matrix Left, math_Vector Right) => Left.Multiplied(Right);
}
