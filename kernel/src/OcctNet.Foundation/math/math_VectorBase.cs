// Port of: src/FoundationClasses/TKMath/math/math_VectorBase.hxx
// Port of: src/FoundationClasses/TKMath/math/math_VectorBase.lxx
// Port of: src/FoundationClasses/TKMath/math/math_Vector.hxx
// Port of: src/FoundationClasses/TKMath/math/math_IntegerVector.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Numerics;

namespace Occt;

/// <summary>
/// OCCT's vector with an arbitrary range of indexes. <c>math_Vector</c> is <c>math_VectorBase&lt;double&gt;</c> and
/// <c>math_IntegerVector</c> is <c>math_VectorBase&lt;int&gt;</c>; both names are global using aliases, as they are
/// <c>using</c> aliases in OCCT.
/// </summary>
/// <remarks>
/// A mutable workspace for ported algorithms (docs/decisions.md, decision 21), and a class:
/// <list type="bullet">
/// <item>C++ copy construction (<c>math_Vector a(b);</c>, <c>math_Vector a = b;</c>) is <c>new(b)</c>.</item>
/// <item>C++ assignment (<c>a = b;</c>, including a moved temporary <c>a = b + c;</c>) is <c>a.Initialized(b)</c>,
/// which copies the values and keeps <c>a</c>'s bounds; it throws if the lengths differ, as OCCT does.</item>
/// <item>Compound assignments (<c>+=</c>, <c>-=</c>, <c>*=</c>, <c>/=</c>) change the vector in place, as in C++.</item>
/// </list>
/// OCCT keeps vectors of up to 32 elements in an inline buffer; here the storage is always a .NET array.
/// </remarks>
public class math_VectorBase<TheItemType>
    where TheItemType : struct, INumber<TheItemType>
{
    private NCollection_Array1<TheItemType> Array;

    /// <summary>A vector of range [theLower..theUpper], its values zero (OCCT leaves them uninitialised).</summary>
    public math_VectorBase(int theLower, int theUpper)
    {
        Array = new NCollection_Array1<TheItemType>(theLower, theUpper);
    }

    /// <summary>A vector of range [theLower..theUpper], every value <paramref name="theInitialValue"/>.</summary>
    public math_VectorBase(int theLower, int theUpper, TheItemType theInitialValue)
    {
        Array = new NCollection_Array1<TheItemType>(theLower, theUpper);
        Array.Init(theInitialValue);
    }

    /// <summary>
    /// A vector of range [theLower..theUpper] over the storage of <paramref name="theTab"/> from its start, without
    /// copying: OCCT's constructor from a <c>const TheItemType*</c>. Writes go to <paramref name="theTab"/>.
    /// </summary>
    public math_VectorBase(TheItemType[] theTab, int theLower, int theUpper)
    {
        Array = new NCollection_Array1<TheItemType>(theTab, 0, theLower, theUpper);
    }

    /// <summary>
    /// A vector of range [theLower..theUpper] over the storage of <paramref name="theTab"/> from its element
    /// <paramref name="theIndex"/>, without copying (OCCT passes <c>&amp;theTab(theIndex)</c>).
    /// </summary>
    public math_VectorBase(NCollection_Array1<TheItemType> theTab, int theIndex, int theLower, int theUpper)
    {
        Array = new NCollection_Array1<TheItemType>(theTab, theIndex, theLower, theUpper);
    }

    /// <summary>
    /// A vector of range [theLower..theUpper] over the storage of another vector from its element
    /// <paramref name="theIndex"/>, without copying (OCCT passes <c>&amp;theVec(theIndex)</c>).
    /// </summary>
    public math_VectorBase(math_VectorBase<TheItemType> theTab, int theIndex, int theLower, int theUpper)
    {
        Array = new NCollection_Array1<TheItemType>(theTab.Array, theIndex, theLower, theUpper);
    }

    /// <summary>A vector of range [1..2] holding the coordinates of <paramref name="theOther"/>.</summary>
    public math_VectorBase(in gp_XY theOther)
    {
        Array = new NCollection_Array1<TheItemType>(1, 2);
        Array[1] = TheItemType.CreateTruncating(theOther.X());
        Array[2] = TheItemType.CreateTruncating(theOther.Y());
    }

    /// <summary>A vector of range [1..3] holding the coordinates of <paramref name="theOther"/>.</summary>
    public math_VectorBase(in gp_XYZ theOther)
    {
        Array = new NCollection_Array1<TheItemType>(1, 3);
        Array[1] = TheItemType.CreateTruncating(theOther.X());
        Array[2] = TheItemType.CreateTruncating(theOther.Y());
        Array[3] = TheItemType.CreateTruncating(theOther.Z());
    }

    /// <summary>A copy (OCCT's copy constructor).</summary>
    public math_VectorBase(math_VectorBase<TheItemType> theOther)
    {
        Array = new NCollection_Array1<TheItemType>(theOther.Array);
    }

    public void Init(TheItemType theInitialValue) => Array.Init(theInitialValue);

    public int Length() => Array.Length();

    public int Lower() => Array.Lower();

    public int Upper() => Array.Upper();

    /// <summary>The Euclidean norm.</summary>
    public double Norm()
    {
        int aLen = Length();
        ReadOnlySpan<TheItemType> aPtr = Array.AsSpan();
        double aSum1 = 0.0, aSum2 = 0.0, aSum3 = 0.0, aSum4 = 0.0;
        int i = 0;
        int aLen4 = aLen - 3;

        for (; i < aLen4; i += 4)
        {
            double aVal0 = double.CreateTruncating(aPtr[i]);
            double aVal1 = double.CreateTruncating(aPtr[i + 1]);
            double aVal2 = double.CreateTruncating(aPtr[i + 2]);
            double aVal3 = double.CreateTruncating(aPtr[i + 3]);
            aSum1 += aVal0 * aVal0;
            aSum2 += aVal1 * aVal1;
            aSum3 += aVal2 * aVal2;
            aSum4 += aVal3 * aVal3;
        }

        for (; i < aLen; ++i)
        {
            double aVal = double.CreateTruncating(aPtr[i]);
            aSum1 += aVal * aVal;
        }

        return Math.Sqrt((aSum1 + aSum2) + (aSum3 + aSum4));
    }

    /// <summary>The square of the Euclidean norm.</summary>
    public double Norm2()
    {
        int aLen = Length();
        ReadOnlySpan<TheItemType> aPtr = Array.AsSpan();
        double aSum1 = 0.0, aSum2 = 0.0, aSum3 = 0.0, aSum4 = 0.0;
        int i = 0;
        int aLen4 = aLen - 3;

        for (; i < aLen4; i += 4)
        {
            double aVal0 = double.CreateTruncating(aPtr[i]);
            double aVal1 = double.CreateTruncating(aPtr[i + 1]);
            double aVal2 = double.CreateTruncating(aPtr[i + 2]);
            double aVal3 = double.CreateTruncating(aPtr[i + 3]);
            aSum1 += aVal0 * aVal0;
            aSum2 += aVal1 * aVal1;
            aSum3 += aVal2 * aVal2;
            aSum4 += aVal3 * aVal3;
        }

        for (; i < aLen; ++i)
        {
            double aVal = double.CreateTruncating(aPtr[i]);
            aSum1 += aVal * aVal;
        }

        return (aSum1 + aSum2) + (aSum3 + aSum4);
    }

    /// <summary>The index of the largest value (the first one if several are equal).</summary>
    public int Max()
    {
        int I = 0;
        double X = RealFirst();

        for (int Index = Lower(); Index <= Upper(); Index++)
        {
            if (double.CreateTruncating(Array[Index]) > X)
            {
                X = double.CreateTruncating(Array[Index]);
                I = Index;
            }
        }

        return I;
    }

    /// <summary>The index of the smallest value (the first one if several are equal).</summary>
    public int Min()
    {
        int I = 0;
        double X = RealLast();

        for (int Index = Lower(); Index <= Upper(); Index++)
        {
            if (double.CreateTruncating(Array[Index]) < X)
            {
                X = double.CreateTruncating(Array[Index]);
                I = Index;
            }
        }

        return I;
    }

    /// <summary>Divides the vector by its norm. Throws <see cref="Standard_NullValue"/> if the norm is zero.</summary>
    public void Normalize()
    {
        double Result = Norm();
        Standard_NullValue_Raise_if(Result <= RealEpsilon(), "math_VectorBase::Normalize() - vector has zero norm");
        for (int Index = Lower(); Index <= Upper(); Index++)
        {
            Array[Index] = TheItemType.CreateTruncating(double.CreateTruncating(Array[Index]) / Result);
        }
    }

    public math_VectorBase<TheItemType> Normalized()
    {
        math_VectorBase<TheItemType> Result = new(this);
        Result.Normalize();
        return Result;
    }

    /// <summary>Reverses the order of the values.</summary>
    public void Invert()
    {
        for (int Index = Lower(); Index <= (Lower() + Length()) >> 1; Index++)
        {
            int J = Upper() + Lower() - Index;
            TheItemType aTemp = Array[Index];
            Array[Index] = Array[J];
            Array[J] = aTemp;
        }
    }

    public math_VectorBase<TheItemType> Inverse()
    {
        math_VectorBase<TheItemType> Result = new(this);
        Result.Invert();
        return Result;
    }

    /// <summary>Sets the values from index theI1 to theI2 to those of <paramref name="theV"/>.</summary>
    public void Set(int theI1, int theI2, math_VectorBase<TheItemType> theV)
    {
        Standard_RangeError_Raise_if((theI1 < Lower()) || (theI2 > Upper()) || (theI1 > theI2)
                                       || (theI2 - theI1 + 1 != theV.Length()),
                                     "math_VectorBase::Set() - invalid indices");
        int I = theV.Lower();
        for (int Index = theI1; Index <= theI2; Index++)
        {
            Array[Index] = theV.Array[I];
            I++;
        }
    }

    /// <summary>The values from index theI1 to theI2, with that range (theI2..theI1 if theI2 &lt; theI1).</summary>
    public math_VectorBase<TheItemType> Slice(int theI1, int theI2)
    {
        Standard_RangeError_Raise_if((theI1 < Lower()) || (theI1 > Upper()) || (theI2 < Lower()) || (theI2 > Upper()),
                                     "math_VectorBase::Slice() - invalid indices");

        if (theI2 >= theI1)
        {
            math_VectorBase<TheItemType> Result = new(theI1, theI2);
            for (int Index = theI1; Index <= theI2; Index++)
            {
                Result.Array[Index] = Array[Index];
            }

            return Result;
        }
        else
        {
            math_VectorBase<TheItemType> Result = new(theI2, theI1);
            for (int Index = theI1; Index >= theI2; Index--)
            {
                Result.Array[Index] = Array[Index];
            }

            return Result;
        }
    }

    public void Multiply(TheItemType theRight)
    {
        for (int Index = Lower(); Index <= Upper(); Index++)
        {
            Array[Index] = Array[Index] * theRight;
        }
    }

    public math_VectorBase<TheItemType> Multiplied(TheItemType theRight)
    {
        math_VectorBase<TheItemType> Result = new(Lower(), Upper());
        for (int Index = Lower(); Index <= Upper(); Index++)
        {
            Result.Array[Index] = Array[Index] * theRight;
        }

        return Result;
    }

    public math_VectorBase<TheItemType> TMultiplied(TheItemType theRight)
    {
        math_VectorBase<TheItemType> Result = new(Lower(), Upper());
        for (int Index = Lower(); Index <= Upper(); Index++)
        {
            Result.Array[Index] = Array[Index] * theRight;
        }

        return Result;
    }

    /// <summary>Divides every value by <paramref name="theRight"/>; throws <see cref="Standard_DivideByZero"/> if it is zero.</summary>
    public void Divide(TheItemType theRight)
    {
        Standard_DivideByZero_Raise_if(double.CreateTruncating(TheItemType.Abs(theRight)) <= RealEpsilon(),
                                       "math_VectorBase::Divide() - devisor is zero");

        for (int Index = Lower(); Index <= Upper(); Index++)
        {
            Array[Index] = Array[Index] / theRight;
        }
    }

    public math_VectorBase<TheItemType> Divided(TheItemType theRight)
    {
        Standard_DivideByZero_Raise_if(double.CreateTruncating(TheItemType.Abs(theRight)) <= RealEpsilon(),
                                       "math_VectorBase::Divided() - devisor is zero");
        // OCCT multiplies by 1. / theRight converted to the item type, which is 0 for most integers.
        math_VectorBase<TheItemType> temp = Multiplied(TheItemType.CreateTruncating(1.0 / double.CreateTruncating(theRight)));
        return temp;
    }

    public void Add(math_VectorBase<TheItemType> theRight)
    {
        Standard_DimensionError_Raise_if(Length() != theRight.Length(),
                                         "math_VectorBase::Add() - input vector has wrong dimensions");

        int I = theRight.Lower();
        for (int Index = Lower(); Index <= Upper(); Index++)
        {
            Array[Index] = Array[Index] + theRight.Array[I];
            I++;
        }
    }

    public math_VectorBase<TheItemType> Added(math_VectorBase<TheItemType> theRight)
    {
        Standard_DimensionError_Raise_if(Length() != theRight.Length(),
                                         "math_VectorBase::Added() - input vector has wrong dimensions");

        math_VectorBase<TheItemType> Result = new(Lower(), Upper());

        int I = theRight.Lower();
        for (int Index = Lower(); Index <= Upper(); Index++)
        {
            Result.Array[Index] = Array[Index] + theRight.Array[I];
            I++;
        }

        return Result;
    }

    /// <summary>Sets this vector to the product of the matrix <paramref name="theLeft"/> and the vector <paramref name="theRight"/>.</summary>
    public void Multiply(math_Matrix theLeft, math_VectorBase<TheItemType> theRight)
    {
        Standard_DimensionError_Raise_if((Length() != theLeft.RowNumber()) || (theLeft.ColNumber() != theRight.Length()),
                                         "math_VectorBase::Multiply() - input matrix and /or vector have wrong dimensions");

        // result[r] = sum_c theLeft(r, c) * theRight[c]
        int aNRows = theLeft.RowNumber();
        int aNCols = theLeft.ColNumber();
        ReadOnlySpan<double> aMatData = theLeft.AsSpan();
        ReadOnlySpan<TheItemType> aRightPtr = theRight.Array.AsSpan();
        Span<TheItemType> aResPtr = Array.AsSpan();

        for (int r = 0; r < aNRows; r++)
        {
            TheItemType aSum = TheItemType.Zero;
            ReadOnlySpan<double> aRowData = aMatData.Slice(r * aNCols);
            for (int c = 0; c < aNCols; c++)
            {
                aSum = TheItemType.CreateTruncating(double.CreateTruncating(aSum) + aRowData[c] * double.CreateTruncating(aRightPtr[c]));
            }

            aResPtr[r] = aSum;
        }
    }

    /// <summary>Sets this vector to the product of the vector <paramref name="theLeft"/> and the matrix <paramref name="theRight"/>.</summary>
    public void Multiply(math_VectorBase<TheItemType> theLeft, math_Matrix theRight)
    {
        Standard_DimensionError_Raise_if((Length() != theRight.ColNumber()) || (theLeft.Length() != theRight.RowNumber()),
                                         "math_VectorBase::Multiply() - input matrix and /or vector have wrong dimensions");

        // result[c] = sum_r theLeft[r] * theRight(r, c)
        int aNCols = theRight.ColNumber();
        int aNRows = theRight.RowNumber();
        ReadOnlySpan<double> aMatData = theRight.AsSpan();
        ReadOnlySpan<TheItemType> aLeftPtr = theLeft.Array.AsSpan();
        Span<TheItemType> aResPtr = Array.AsSpan();

        for (int c = 0; c < aNCols; c++)
        {
            TheItemType aSum = TheItemType.Zero;
            for (int r = 0; r < aNRows; r++)
            {
                aSum = TheItemType.CreateTruncating(double.CreateTruncating(aSum) + double.CreateTruncating(aLeftPtr[r]) * aMatData[r * aNCols + c]);
            }

            aResPtr[c] = aSum;
        }
    }

    /// <summary>Sets this vector to the product of the transpose of <paramref name="theTLeft"/> and <paramref name="theRight"/>.</summary>
    public void TMultiply(math_Matrix theTLeft, math_VectorBase<TheItemType> theRight)
    {
        Standard_DimensionError_Raise_if((Length() != theTLeft.ColNumber()) || (theTLeft.RowNumber() != theRight.Length()),
                                         "math_VectorBase::TMultiply() - input matrix and /or vector have wrong dimensions");

        // result[c] = sum_r theTLeft(r, c) * theRight[r]  (transpose-multiply)
        int aNCols = theTLeft.ColNumber();
        int aNRows = theTLeft.RowNumber();
        ReadOnlySpan<double> aMatData = theTLeft.AsSpan();
        ReadOnlySpan<TheItemType> aRightPtr = theRight.Array.AsSpan();
        Span<TheItemType> aResPtr = Array.AsSpan();

        for (int c = 0; c < aNCols; c++)
        {
            TheItemType aSum = TheItemType.Zero;
            for (int r = 0; r < aNRows; r++)
            {
                aSum = TheItemType.CreateTruncating(double.CreateTruncating(aSum) + aMatData[r * aNCols + c] * double.CreateTruncating(aRightPtr[r]));
            }

            aResPtr[c] = aSum;
        }
    }

    /// <summary>Sets this vector to the product of <paramref name="theLeft"/> and the transpose of <paramref name="theTRight"/>.</summary>
    public void TMultiply(math_VectorBase<TheItemType> theLeft, math_Matrix theTRight)
    {
        Standard_DimensionError_Raise_if((Length() != theTRight.RowNumber()) || (theLeft.Length() != theTRight.ColNumber()),
                                         "math_VectorBase::TMultiply() - input matrix and /or vector have wrong dimensions");

        // result[r] = sum_c theLeft[c] * theTRight(r, c)
        int aNCols = theTRight.ColNumber();
        int aNRows = theTRight.RowNumber();
        ReadOnlySpan<double> aMatData = theTRight.AsSpan();
        ReadOnlySpan<TheItemType> aLeftPtr = theLeft.Array.AsSpan();
        Span<TheItemType> aResPtr = Array.AsSpan();

        for (int r = 0; r < aNRows; r++)
        {
            TheItemType aSum = TheItemType.Zero;
            ReadOnlySpan<double> aRowData = aMatData.Slice(r * aNCols);
            for (int c = 0; c < aNCols; c++)
            {
                aSum = TheItemType.CreateTruncating(double.CreateTruncating(aSum) + double.CreateTruncating(aLeftPtr[c]) * aRowData[c]);
            }

            aResPtr[r] = aSum;
        }
    }

    /// <summary>Sets this vector to <paramref name="theLeft"/> + <paramref name="theRight"/>.</summary>
    public void Add(math_VectorBase<TheItemType> theLeft, math_VectorBase<TheItemType> theRight)
    {
        Standard_DimensionError_Raise_if((Length() != theRight.Length()) || (theRight.Length() != theLeft.Length()),
                                         "math_VectorBase::Add() - input vectors have wrong dimensions");

        int I = theLeft.Lower();
        int J = theRight.Lower();
        for (int Index = Lower(); Index <= Upper(); Index++)
        {
            Array[Index] = theLeft.Array[I] + theRight.Array[J];
            I++;
            J++;
        }
    }

    /// <summary>Sets this vector to <paramref name="theLeft"/> - <paramref name="theRight"/>.</summary>
    public void Subtract(math_VectorBase<TheItemType> theLeft, math_VectorBase<TheItemType> theRight)
    {
        Standard_DimensionError_Raise_if((Length() != theRight.Length()) || (theRight.Length() != theLeft.Length()),
                                         "math_VectorBase::Subtract() - input vectors have wrong dimensions");

        int I = theLeft.Lower();
        int J = theRight.Lower();
        for (int Index = Lower(); Index <= Upper(); Index++)
        {
            Array[Index] = theLeft.Array[I] - theRight.Array[J];
            I++;
            J++;
        }
    }

    /// <summary>The value at index <paramref name="theNum"/>, for reading and writing.</summary>
    public ref TheItemType Value(int theNum) => ref Array[theNum];

    /// <summary>OCCT's <c>operator()</c>: the value at index <paramref name="theNum"/>, for reading and writing.</summary>
    public ref TheItemType this[int theNum] => ref Array[theNum];

    /// <summary>
    /// Copies the values of <paramref name="theOther"/>, keeping this vector's bounds (OCCT's <c>operator=</c>).
    /// Throws <see cref="Standard_DimensionMismatch"/> if the lengths differ.
    /// </summary>
    public math_VectorBase<TheItemType> Initialized(math_VectorBase<TheItemType> theOther)
    {
        Array.CopyValues(theOther.Array);
        return this;
    }

    /// <summary>The scalar product.</summary>
    public TheItemType Multiplied(math_VectorBase<TheItemType> theRight)
    {
        Standard_DimensionError_Raise_if(Length() != theRight.Length(),
                                         "math_VectorBase::Multiplied() - input vector has wrong dimensions");

        int aLen = Length();
        ReadOnlySpan<TheItemType> aLeftPtr = Array.AsSpan();
        ReadOnlySpan<TheItemType> aRightPtr = theRight.Array.AsSpan();
        TheItemType aResult = TheItemType.Zero;
        for (int i = 0; i < aLen; i++)
        {
            aResult += aLeftPtr[i] * aRightPtr[i];
        }

        return aResult;
    }

    /// <summary>The product of this vector (as a row) and the matrix <paramref name="theRight"/>.</summary>
    public math_VectorBase<TheItemType> Multiplied(math_Matrix theRight)
    {
        Standard_DimensionError_Raise_if(Length() != theRight.RowNumber(),
                                         "math_VectorBase::Multiplied() - input matrix has wrong dimensions");

        math_VectorBase<TheItemType> Result = new(theRight.LowerCol(), theRight.UpperCol());
        for (int J2 = theRight.LowerCol(); J2 <= theRight.UpperCol(); J2++)
        {
            Result.Array[J2] = TheItemType.Zero;
            int theI2 = theRight.LowerRow();
            for (int I = Lower(); I <= Upper(); I++)
            {
                Result.Array[J2] = TheItemType.CreateTruncating(double.CreateTruncating(Result.Array[J2])
                                                               + double.CreateTruncating(Array[I]) * theRight[theI2, J2]);
                theI2++;
            }
        }

        return Result;
    }

    public math_VectorBase<TheItemType> Opposite()
    {
        math_VectorBase<TheItemType> Result = new(Lower(), Upper());
        for (int Index = Lower(); Index <= Upper(); Index++)
        {
            Result.Array[Index] = -Array[Index];
        }

        return Result;
    }

    public void Subtract(math_VectorBase<TheItemType> theRight)
    {
        Standard_DimensionError_Raise_if(Length() != theRight.Length(),
                                         "math_VectorBase::Subtract() - input vector has wrong dimensions");

        int I = theRight.Lower();
        for (int Index = Lower(); Index <= Upper(); Index++)
        {
            Array[Index] = Array[Index] - theRight.Array[I];
            I++;
        }
    }

    public math_VectorBase<TheItemType> Subtracted(math_VectorBase<TheItemType> theRight)
    {
        Standard_DimensionError_Raise_if(Length() != theRight.Length(),
                                         "math_VectorBase::Subtracted() - input vector has wrong dimensions");

        math_VectorBase<TheItemType> Result = new(Lower(), Upper());

        int I = theRight.Lower();
        for (int Index = Lower(); Index <= Upper(); Index++)
        {
            Result.Array[Index] = Array[Index] - theRight.Array[I];
            I++;
        }

        return Result;
    }

    /// <summary>Sets this vector to <paramref name="theLeft"/> * <paramref name="theRight"/>.</summary>
    public void Multiply(TheItemType theLeft, math_VectorBase<TheItemType> theRight)
    {
        Standard_DimensionError_Raise_if(Length() != theRight.Length(),
                                         "math_VectorBase::Multiply() - input vector has wrong dimensions");
        for (int I = Lower(); I <= Upper(); I++)
        {
            Array[I] = theLeft * theRight.Array[I];
        }
    }

    /// <summary>Prints the vector, as OCCT's <c>Dump</c>.</summary>
    public void Dump(TextWriter theO)
    {
        theO.Write("math_Vector of Length = " + Length() + "\n");
        for (int Index = Lower(); Index <= Upper(); Index++)
        {
            theO.Write("math_Vector(" + Index + ") = " + Convert.ToString(Array[Index], CultureInfo.InvariantCulture) + "\n");
        }
    }

    /// <summary>The underlying array, shared (OCCT returns it by const reference).</summary>
    public NCollection_Array1<TheItemType> Array1() => Array;

    /// <summary>Changes the length, keeping the lower bound and the values that fit.</summary>
    /// <remarks>OCCT shrinks and regrows within its inline buffer of 32 without clearing it, so values beyond the old
    /// length can reappear there; here new values are zero.</remarks>
    public void Resize(int theSize)
    {
        int theLower = Array.Lower();
        int theUpper = theLower + theSize - 1;
        Array.Resize(theLower, theUpper, true);
    }

    /// <summary>The values, without copying; index 0 is <see cref="Lower"/>.</summary>
    public Span<TheItemType> AsSpan() => Array.AsSpan();

    protected internal void SetLower(int theLower) => Array.UpdateLowerBound(theLower);

    public override string ToString()
    {
        using var aWriter = new StringWriter(CultureInfo.InvariantCulture);
        Dump(aWriter);
        return aWriter.ToString();
    }

    // Operators. The compound assignments change the vector in place, as OCCT's do.

    public void operator *=(TheItemType theRight) => Multiply(theRight);

    public void operator /=(TheItemType theRight) => Divide(theRight);

    public void operator +=(math_VectorBase<TheItemType> theRight) => Add(theRight);

    public void operator -=(math_VectorBase<TheItemType> theRight) => Subtract(theRight);

    public static math_VectorBase<TheItemType> operator *(math_VectorBase<TheItemType> theLeft, TheItemType theRight)
        => theLeft.Multiplied(theRight);

    public static math_VectorBase<TheItemType> operator *(TheItemType theLeft, math_VectorBase<TheItemType> theRight)
        => theRight.Multiplied(theLeft);

    public static math_VectorBase<TheItemType> operator /(math_VectorBase<TheItemType> theLeft, TheItemType theRight)
        => theLeft.Divided(theRight);

    public static math_VectorBase<TheItemType> operator +(math_VectorBase<TheItemType> theLeft, math_VectorBase<TheItemType> theRight)
        => theLeft.Added(theRight);

    public static math_VectorBase<TheItemType> operator -(math_VectorBase<TheItemType> theLeft, math_VectorBase<TheItemType> theRight)
        => theLeft.Subtracted(theRight);

    public static math_VectorBase<TheItemType> operator -(math_VectorBase<TheItemType> theVector) => theVector.Opposite();

    /// <summary>The scalar product.</summary>
    public static TheItemType operator *(math_VectorBase<TheItemType> theLeft, math_VectorBase<TheItemType> theRight)
        => theLeft.Multiplied(theRight);

    public static math_VectorBase<TheItemType> operator *(math_VectorBase<TheItemType> theLeft, math_Matrix theRight)
        => theLeft.Multiplied(theRight);
}
