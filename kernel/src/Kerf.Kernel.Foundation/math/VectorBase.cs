// Port of: src/FoundationClasses/TKMath/math/math_VectorBase.hxx
// Port of: src/FoundationClasses/TKMath/math/math_VectorBase.lxx
// Port of: src/FoundationClasses/TKMath/math/math_Vector.hxx
// Port of: src/FoundationClasses/TKMath/math/math_IntegerVector.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Occt;
using System.Globalization;
using System.Numerics;

namespace Kerf.Engine.Numerics;

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
internal class VectorBase<TheItemType>
    where TheItemType : struct, INumber<TheItemType>
{
    private NCollection_Array1<TheItemType> _array;

    /// <summary>A vector of range [theLower..theUpper], its values zero (OCCT leaves them uninitialised).</summary>
    public VectorBase(int lower, int upper)
    {
        _array = new NCollection_Array1<TheItemType>(lower, upper);
    }

    /// <summary>A vector of range [theLower..theUpper], every value <paramref name="initialValue"/>.</summary>
    public VectorBase(int lower, int upper, TheItemType initialValue)
    {
        _array = new NCollection_Array1<TheItemType>(lower, upper);
        _array.Init(initialValue);
    }

    /// <summary>
    /// A vector of range [theLower..theUpper] over the storage of <paramref name="tab"/> from its start, without
    /// copying: OCCT's constructor from a <c>const TheItemType*</c>. Writes go to <paramref name="tab"/>.
    /// </summary>
    public VectorBase(TheItemType[] tab, int lower, int upper)
    {
        _array = new NCollection_Array1<TheItemType>(tab, 0, lower, upper);
    }

    /// <summary>
    /// A vector of range [theLower..theUpper] over the storage of <paramref name="tab"/> from its element
    /// <paramref name="index"/>, without copying (OCCT passes <c>&amp;theTab(theIndex)</c>).
    /// </summary>
    public VectorBase(NCollection_Array1<TheItemType> tab, int index, int lower, int upper)
    {
        _array = new NCollection_Array1<TheItemType>(tab, index, lower, upper);
    }

    /// <summary>
    /// A vector of range [theLower..theUpper] over the storage of another vector from its element
    /// <paramref name="index"/>, without copying (OCCT passes <c>&amp;theVec(theIndex)</c>).
    /// </summary>
    public VectorBase(VectorBase<TheItemType> tab, int index, int lower, int upper)
    {
        _array = new NCollection_Array1<TheItemType>(tab._array, index, lower, upper);
    }

    /// <summary>A vector of range [1..2] holding the coordinates of <paramref name="other"/>.</summary>
    public VectorBase(in Xy other)
    {
        _array = new NCollection_Array1<TheItemType>(1, 2);
        _array[1] = TheItemType.CreateTruncating(other.X());
        _array[2] = TheItemType.CreateTruncating(other.Y());
    }

    /// <summary>A vector of range [1..3] holding the coordinates of <paramref name="other"/>.</summary>
    public VectorBase(in Xyz other)
    {
        _array = new NCollection_Array1<TheItemType>(1, 3);
        _array[1] = TheItemType.CreateTruncating(other.X());
        _array[2] = TheItemType.CreateTruncating(other.Y());
        _array[3] = TheItemType.CreateTruncating(other.Z());
    }

    /// <summary>A copy (OCCT's copy constructor).</summary>
    public VectorBase(VectorBase<TheItemType> other)
    {
        _array = new NCollection_Array1<TheItemType>(other._array);
    }

    public void Init(TheItemType initialValue) => _array.Init(initialValue);

    public int Length() => _array.Length();

    public int Lower() => _array.Lower();

    public int Upper() => _array.Upper();

    /// <summary>The Euclidean norm.</summary>
    public double Norm()
    {
        int len = Length();
        ReadOnlySpan<TheItemType> ptr = _array.AsSpan();
        double sum1 = 0.0, sum2 = 0.0, sum3 = 0.0, sum4 = 0.0;
        int i = 0;
        int len4 = len - 3;

        for (; i < len4; i += 4)
        {
            double val0 = double.CreateTruncating(ptr[i]);
            double val1 = double.CreateTruncating(ptr[i + 1]);
            double val2 = double.CreateTruncating(ptr[i + 2]);
            double val3 = double.CreateTruncating(ptr[i + 3]);
            sum1 += val0 * val0;
            sum2 += val1 * val1;
            sum3 += val2 * val2;
            sum4 += val3 * val3;
        }

        for (; i < len; ++i)
        {
            double val = double.CreateTruncating(ptr[i]);
            sum1 += val * val;
        }

        return Math.Sqrt((sum1 + sum2) + (sum3 + sum4));
    }

    /// <summary>The square of the Euclidean norm.</summary>
    public double Norm2()
    {
        int len = Length();
        ReadOnlySpan<TheItemType> ptr = _array.AsSpan();
        double sum1 = 0.0, sum2 = 0.0, sum3 = 0.0, sum4 = 0.0;
        int i = 0;
        int len4 = len - 3;

        for (; i < len4; i += 4)
        {
            double val0 = double.CreateTruncating(ptr[i]);
            double val1 = double.CreateTruncating(ptr[i + 1]);
            double val2 = double.CreateTruncating(ptr[i + 2]);
            double val3 = double.CreateTruncating(ptr[i + 3]);
            sum1 += val0 * val0;
            sum2 += val1 * val1;
            sum3 += val2 * val2;
            sum4 += val3 * val3;
        }

        for (; i < len; ++i)
        {
            double val = double.CreateTruncating(ptr[i]);
            sum1 += val * val;
        }

        return (sum1 + sum2) + (sum3 + sum4);
    }

    /// <summary>The index of the largest value (the first one if several are equal).</summary>
    public int Max()
    {
        int i = 0;
        double x = RealFirst();

        for (int index = Lower(); index <= Upper(); index++)
        {
            if (double.CreateTruncating(_array[index]) > x)
            {
                x = double.CreateTruncating(_array[index]);
                i = index;
            }
        }

        return i;
    }

    /// <summary>The index of the smallest value (the first one if several are equal).</summary>
    public int Min()
    {
        int i = 0;
        double x = RealLast();

        for (int index = Lower(); index <= Upper(); index++)
        {
            if (double.CreateTruncating(_array[index]) < x)
            {
                x = double.CreateTruncating(_array[index]);
                i = index;
            }
        }

        return i;
    }

    /// <summary>Divides the vector by its norm. Throws <see cref="NullValueException"/> if the norm is zero.</summary>
    public void Normalize()
    {
        double result = Norm();
        Standard_NullValue_Raise_if(result <= RealEpsilon(), "math_VectorBase::Normalize() - vector has zero norm");
        for (int index = Lower(); index <= Upper(); index++)
        {
            _array[index] = TheItemType.CreateTruncating(double.CreateTruncating(_array[index]) / result);
        }
    }

    public VectorBase<TheItemType> Normalized()
    {
        VectorBase<TheItemType> result = new(this);
        result.Normalize();
        return result;
    }

    /// <summary>Reverses the order of the values.</summary>
    public void Invert()
    {
        for (int index = Lower(); index <= (Lower() + Length()) >> 1; index++)
        {
            int j = Upper() + Lower() - index;
            TheItemType temp = _array[index];
            _array[index] = _array[j];
            _array[j] = temp;
        }
    }

    public VectorBase<TheItemType> Inverse()
    {
        VectorBase<TheItemType> result = new(this);
        result.Invert();
        return result;
    }

    /// <summary>Sets the values from index theI1 to theI2 to those of <paramref name="v"/>.</summary>
    public void Set(int i1, int i2, VectorBase<TheItemType> v)
    {
        Standard_RangeError_Raise_if((i1 < Lower()) || (i2 > Upper()) || (i1 > i2)
                                       || (i2 - i1 + 1 != v.Length()),
                                     "math_VectorBase::Set() - invalid indices");
        int i = v.Lower();
        for (int index = i1; index <= i2; index++)
        {
            _array[index] = v._array[i];
            i++;
        }
    }

    /// <summary>The values from index theI1 to theI2, with that range (theI2..theI1 if theI2 &lt; theI1).</summary>
    public VectorBase<TheItemType> Slice(int i1, int i2)
    {
        Standard_RangeError_Raise_if((i1 < Lower()) || (i1 > Upper()) || (i2 < Lower()) || (i2 > Upper()),
                                     "math_VectorBase::Slice() - invalid indices");

        if (i2 >= i1)
        {
            VectorBase<TheItemType> result = new(i1, i2);
            for (int index = i1; index <= i2; index++)
            {
                result._array[index] = _array[index];
            }

            return result;
        }
        else
        {
            VectorBase<TheItemType> result = new(i2, i1);
            for (int index = i1; index >= i2; index--)
            {
                result._array[index] = _array[index];
            }

            return result;
        }
    }

    public void Multiply(TheItemType right)
    {
        for (int index = Lower(); index <= Upper(); index++)
        {
            _array[index] = _array[index] * right;
        }
    }

    public VectorBase<TheItemType> Multiplied(TheItemType right)
    {
        VectorBase<TheItemType> result = new(Lower(), Upper());
        for (int index = Lower(); index <= Upper(); index++)
        {
            result._array[index] = _array[index] * right;
        }

        return result;
    }

    public VectorBase<TheItemType> TMultiplied(TheItemType right)
    {
        VectorBase<TheItemType> result = new(Lower(), Upper());
        for (int index = Lower(); index <= Upper(); index++)
        {
            result._array[index] = _array[index] * right;
        }

        return result;
    }

    /// <summary>Divides every value by <paramref name="right"/>; throws <see cref="ZeroDivisorException"/> if it is zero.</summary>
    public void Divide(TheItemType right)
    {
        Standard_DivideByZero_Raise_if(double.CreateTruncating(TheItemType.Abs(right)) <= RealEpsilon(),
                                       "math_VectorBase::Divide() - devisor is zero");

        for (int index = Lower(); index <= Upper(); index++)
        {
            _array[index] = _array[index] / right;
        }
    }

    public VectorBase<TheItemType> Divided(TheItemType right)
    {
        Standard_DivideByZero_Raise_if(double.CreateTruncating(TheItemType.Abs(right)) <= RealEpsilon(),
                                       "math_VectorBase::Divided() - devisor is zero");
        // OCCT multiplies by 1. / theRight converted to the item type, which is 0 for most integers.
        VectorBase<TheItemType> temp = Multiplied(TheItemType.CreateTruncating(1.0 / double.CreateTruncating(right)));
        return temp;
    }

    public void Add(VectorBase<TheItemType> right)
    {
        Standard_DimensionError_Raise_if(Length() != right.Length(),
                                         "math_VectorBase::Add() - input vector has wrong dimensions");

        int i = right.Lower();
        for (int index = Lower(); index <= Upper(); index++)
        {
            _array[index] = _array[index] + right._array[i];
            i++;
        }
    }

    public VectorBase<TheItemType> Added(VectorBase<TheItemType> right)
    {
        Standard_DimensionError_Raise_if(Length() != right.Length(),
                                         "math_VectorBase::Added() - input vector has wrong dimensions");

        VectorBase<TheItemType> result = new(Lower(), Upper());

        int i = right.Lower();
        for (int index = Lower(); index <= Upper(); index++)
        {
            result._array[index] = _array[index] + right._array[i];
            i++;
        }

        return result;
    }

    /// <summary>Sets this vector to the product of the matrix <paramref name="left"/> and the vector <paramref name="right"/>.</summary>
    public void Multiply(Matrix left, VectorBase<TheItemType> right)
    {
        Standard_DimensionError_Raise_if((Length() != left.RowNumber()) || (left.ColNumber() != right.Length()),
                                         "math_VectorBase::Multiply() - input matrix and /or vector have wrong dimensions");

        // result[r] = sum_c theLeft(r, c) * theRight[c]
        int nRows = left.RowNumber();
        int nCols = left.ColNumber();
        ReadOnlySpan<double> matData = left.AsSpan();
        ReadOnlySpan<TheItemType> rightPtr = right._array.AsSpan();
        Span<TheItemType> resPtr = _array.AsSpan();

        for (int r = 0; r < nRows; r++)
        {
            TheItemType sum = TheItemType.Zero;
            ReadOnlySpan<double> rowData = matData.Slice(r * nCols);
            for (int c = 0; c < nCols; c++)
            {
                sum = TheItemType.CreateTruncating(double.CreateTruncating(sum) + rowData[c] * double.CreateTruncating(rightPtr[c]));
            }

            resPtr[r] = sum;
        }
    }

    /// <summary>Sets this vector to the product of the vector <paramref name="left"/> and the matrix <paramref name="right"/>.</summary>
    public void Multiply(VectorBase<TheItemType> left, Matrix right)
    {
        Standard_DimensionError_Raise_if((Length() != right.ColNumber()) || (left.Length() != right.RowNumber()),
                                         "math_VectorBase::Multiply() - input matrix and /or vector have wrong dimensions");

        // result[c] = sum_r theLeft[r] * theRight(r, c)
        int nCols = right.ColNumber();
        int nRows = right.RowNumber();
        ReadOnlySpan<double> matData = right.AsSpan();
        ReadOnlySpan<TheItemType> leftPtr = left._array.AsSpan();
        Span<TheItemType> resPtr = _array.AsSpan();

        for (int c = 0; c < nCols; c++)
        {
            TheItemType sum = TheItemType.Zero;
            for (int r = 0; r < nRows; r++)
            {
                sum = TheItemType.CreateTruncating(double.CreateTruncating(sum) + double.CreateTruncating(leftPtr[r]) * matData[r * nCols + c]);
            }

            resPtr[c] = sum;
        }
    }

    /// <summary>Sets this vector to the product of the transpose of <paramref name="tLeft"/> and <paramref name="right"/>.</summary>
    public void TMultiply(Matrix tLeft, VectorBase<TheItemType> right)
    {
        Standard_DimensionError_Raise_if((Length() != tLeft.ColNumber()) || (tLeft.RowNumber() != right.Length()),
                                         "math_VectorBase::TMultiply() - input matrix and /or vector have wrong dimensions");

        // result[c] = sum_r theTLeft(r, c) * theRight[r]  (transpose-multiply)
        int nCols = tLeft.ColNumber();
        int nRows = tLeft.RowNumber();
        ReadOnlySpan<double> matData = tLeft.AsSpan();
        ReadOnlySpan<TheItemType> rightPtr = right._array.AsSpan();
        Span<TheItemType> resPtr = _array.AsSpan();

        for (int c = 0; c < nCols; c++)
        {
            TheItemType sum = TheItemType.Zero;
            for (int r = 0; r < nRows; r++)
            {
                sum = TheItemType.CreateTruncating(double.CreateTruncating(sum) + matData[r * nCols + c] * double.CreateTruncating(rightPtr[r]));
            }

            resPtr[c] = sum;
        }
    }

    /// <summary>Sets this vector to the product of <paramref name="left"/> and the transpose of <paramref name="tRight"/>.</summary>
    public void TMultiply(VectorBase<TheItemType> left, Matrix tRight)
    {
        Standard_DimensionError_Raise_if((Length() != tRight.RowNumber()) || (left.Length() != tRight.ColNumber()),
                                         "math_VectorBase::TMultiply() - input matrix and /or vector have wrong dimensions");

        // result[r] = sum_c theLeft[c] * theTRight(r, c)
        int nCols = tRight.ColNumber();
        int nRows = tRight.RowNumber();
        ReadOnlySpan<double> matData = tRight.AsSpan();
        ReadOnlySpan<TheItemType> leftPtr = left._array.AsSpan();
        Span<TheItemType> resPtr = _array.AsSpan();

        for (int r = 0; r < nRows; r++)
        {
            TheItemType sum = TheItemType.Zero;
            ReadOnlySpan<double> rowData = matData.Slice(r * nCols);
            for (int c = 0; c < nCols; c++)
            {
                sum = TheItemType.CreateTruncating(double.CreateTruncating(sum) + double.CreateTruncating(leftPtr[c]) * rowData[c]);
            }

            resPtr[r] = sum;
        }
    }

    /// <summary>Sets this vector to <paramref name="left"/> + <paramref name="right"/>.</summary>
    public void Add(VectorBase<TheItemType> left, VectorBase<TheItemType> right)
    {
        Standard_DimensionError_Raise_if((Length() != right.Length()) || (right.Length() != left.Length()),
                                         "math_VectorBase::Add() - input vectors have wrong dimensions");

        int i = left.Lower();
        int j = right.Lower();
        for (int index = Lower(); index <= Upper(); index++)
        {
            _array[index] = left._array[i] + right._array[j];
            i++;
            j++;
        }
    }

    /// <summary>Sets this vector to <paramref name="left"/> - <paramref name="right"/>.</summary>
    public void Subtract(VectorBase<TheItemType> left, VectorBase<TheItemType> right)
    {
        Standard_DimensionError_Raise_if((Length() != right.Length()) || (right.Length() != left.Length()),
                                         "math_VectorBase::Subtract() - input vectors have wrong dimensions");

        int i = left.Lower();
        int j = right.Lower();
        for (int index = Lower(); index <= Upper(); index++)
        {
            _array[index] = left._array[i] - right._array[j];
            i++;
            j++;
        }
    }

    /// <summary>The value at index <paramref name="num"/>, for reading and writing.</summary>
    public ref TheItemType Value(int num) => ref _array[num];

    /// <summary>OCCT's <c>operator()</c>: the value at index <paramref name="num"/>, for reading and writing.</summary>
    public ref TheItemType this[int num] => ref _array[num];

    /// <summary>
    /// Copies the values of <paramref name="other"/>, keeping this vector's bounds (OCCT's <c>operator=</c>).
    /// Throws <see cref="DimensionMismatchException"/> if the lengths differ.
    /// </summary>
    public VectorBase<TheItemType> Initialized(VectorBase<TheItemType> other)
    {
        _array.CopyValues(other._array);
        return this;
    }

    /// <summary>The scalar product.</summary>
    public TheItemType Multiplied(VectorBase<TheItemType> right)
    {
        Standard_DimensionError_Raise_if(Length() != right.Length(),
                                         "math_VectorBase::Multiplied() - input vector has wrong dimensions");

        int len = Length();
        ReadOnlySpan<TheItemType> leftPtr = _array.AsSpan();
        ReadOnlySpan<TheItemType> rightPtr = right._array.AsSpan();
        TheItemType result = TheItemType.Zero;
        for (int i = 0; i < len; i++)
        {
            result += leftPtr[i] * rightPtr[i];
        }

        return result;
    }

    /// <summary>The product of this vector (as a row) and the matrix <paramref name="right"/>.</summary>
    public VectorBase<TheItemType> Multiplied(Matrix right)
    {
        Standard_DimensionError_Raise_if(Length() != right.RowNumber(),
                                         "math_VectorBase::Multiplied() - input matrix has wrong dimensions");

        VectorBase<TheItemType> result = new(right.LowerCol(), right.UpperCol());
        for (int j2 = right.LowerCol(); j2 <= right.UpperCol(); j2++)
        {
            result._array[j2] = TheItemType.Zero;
            int theI2 = right.LowerRow();
            for (int i = Lower(); i <= Upper(); i++)
            {
                result._array[j2] = TheItemType.CreateTruncating(double.CreateTruncating(result._array[j2])
                                                               + double.CreateTruncating(_array[i]) * right[theI2, j2]);
                theI2++;
            }
        }

        return result;
    }

    public VectorBase<TheItemType> Opposite()
    {
        VectorBase<TheItemType> result = new(Lower(), Upper());
        for (int index = Lower(); index <= Upper(); index++)
        {
            result._array[index] = -_array[index];
        }

        return result;
    }

    public void Subtract(VectorBase<TheItemType> right)
    {
        Standard_DimensionError_Raise_if(Length() != right.Length(),
                                         "math_VectorBase::Subtract() - input vector has wrong dimensions");

        int i = right.Lower();
        for (int index = Lower(); index <= Upper(); index++)
        {
            _array[index] = _array[index] - right._array[i];
            i++;
        }
    }

    public VectorBase<TheItemType> Subtracted(VectorBase<TheItemType> right)
    {
        Standard_DimensionError_Raise_if(Length() != right.Length(),
                                         "math_VectorBase::Subtracted() - input vector has wrong dimensions");

        VectorBase<TheItemType> result = new(Lower(), Upper());

        int i = right.Lower();
        for (int index = Lower(); index <= Upper(); index++)
        {
            result._array[index] = _array[index] - right._array[i];
            i++;
        }

        return result;
    }

    /// <summary>Sets this vector to <paramref name="left"/> * <paramref name="right"/>.</summary>
    public void Multiply(TheItemType left, VectorBase<TheItemType> right)
    {
        Standard_DimensionError_Raise_if(Length() != right.Length(),
                                         "math_VectorBase::Multiply() - input vector has wrong dimensions");
        for (int i = Lower(); i <= Upper(); i++)
        {
            _array[i] = left * right._array[i];
        }
    }

    /// <summary>Prints the vector, as OCCT's <c>Dump</c>.</summary>
    public void Dump(TextWriter o)
    {
        o.Write("math_Vector of Length = " + Length() + "\n");
        for (int index = Lower(); index <= Upper(); index++)
        {
            o.Write("math_Vector(" + index + ") = " + Convert.ToString(_array[index], CultureInfo.InvariantCulture) + "\n");
        }
    }

    /// <summary>The underlying array, shared (OCCT returns it by const reference).</summary>
    public NCollection_Array1<TheItemType> Array1() => _array;

    /// <summary>Changes the length, keeping the lower bound and the values that fit.</summary>
    /// <remarks>OCCT shrinks and regrows within its inline buffer of 32 without clearing it, so values beyond the old
    /// length can reappear there; here new values are zero.</remarks>
    public void Resize(int size)
    {
        int theLower = _array.Lower();
        int theUpper = theLower + size - 1;
        _array.Resize(theLower, theUpper, true);
    }

    /// <summary>The values, without copying; index 0 is <see cref="Lower"/>.</summary>
    public Span<TheItemType> AsSpan() => _array.AsSpan();

    protected internal void SetLower(int lower) => _array.UpdateLowerBound(lower);

    public override string ToString()
    {
        using var writer = new StringWriter(CultureInfo.InvariantCulture);
        Dump(writer);
        return writer.ToString();
    }

    // Operators. The compound assignments change the vector in place, as OCCT's do.

    public void operator *=(TheItemType right) => Multiply(right);

    public void operator /=(TheItemType right) => Divide(right);

    public void operator +=(VectorBase<TheItemType> right) => Add(right);

    public void operator -=(VectorBase<TheItemType> right) => Subtract(right);

    public static VectorBase<TheItemType> operator *(VectorBase<TheItemType> left, TheItemType right)
        => left.Multiplied(right);

    public static VectorBase<TheItemType> operator *(TheItemType left, VectorBase<TheItemType> right)
        => right.Multiplied(left);

    public static VectorBase<TheItemType> operator /(VectorBase<TheItemType> left, TheItemType right)
        => left.Divided(right);

    public static VectorBase<TheItemType> operator +(VectorBase<TheItemType> left, VectorBase<TheItemType> right)
        => left.Added(right);

    public static VectorBase<TheItemType> operator -(VectorBase<TheItemType> left, VectorBase<TheItemType> right)
        => left.Subtracted(right);

    public static VectorBase<TheItemType> operator -(VectorBase<TheItemType> vector) => vector.Opposite();

    /// <summary>The scalar product.</summary>
    public static TheItemType operator *(VectorBase<TheItemType> left, VectorBase<TheItemType> right)
        => left.Multiplied(right);

    public static VectorBase<TheItemType> operator *(VectorBase<TheItemType> left, Matrix right)
        => left.Multiplied(right);
}
