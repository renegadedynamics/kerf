// Port of: src/FoundationClasses/TKernel/NCollection/NCollection_AliasedArray.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Occt;

/// <summary>
/// An array of fixed-size elements whose type is chosen at run time by the element size (the stride): the storage of
/// Poly_ArrayOfNodes, which holds either double-precision points (<c>gp_Pnt</c>) or single-precision ones
/// (<c>NCollection_Vec3&lt;float&gt;</c>). Indexed from 0.
/// </summary>
/// <remarks>
/// OCCT holds a raw, aligned byte buffer. The port holds a .NET array of any unmanaged element type and reads it
/// through typed references (<see cref="Value{T}"/>, <see cref="ChangeValue{T}"/>, <see cref="AsSpan{T}"/>), so
/// a subclass can allocate an array of its natural element type (<see cref="allocate"/>), which other code can then
/// share without copying (Poly_Triangulation.MapNodeArray). OCCT's alignment template parameter is dropped. The
/// "wrapping" constructors share the caller's array, as OCCT's share its memory; <see cref="Move"/> leaves the
/// moved-from array sharing the storage too, as in OCCT.
/// </remarks>
public class NCollection_AliasedArray
{
    /// <summary>The storage: an array of an unmanaged element type, or null before allocation.</summary>
    protected Array? myData;

    /// <summary>The offset of element 0 in <see cref="myData"/>, in bytes.</summary>
    protected int myOffset;

    /// <summary>The element size, in bytes.</summary>
    protected int myStride;

    /// <summary>The number of elements.</summary>
    protected int mySize;

    /// <summary>True if this array allocated its storage (OCCT frees it on destruction).</summary>
    protected bool myDeletable;

    /// <summary>An empty array of elements of <paramref name="theStride"/> bytes.</summary>
    public NCollection_AliasedArray(int theStride)
    {
        myStride = theStride;
        if (theStride <= 0)
        {
            throw new Standard_RangeError("NCollection_AliasedArray, stride should be positive");
        }
    }

    /// <summary>An array of <paramref name="theLength"/> elements of <paramref name="theStride"/> bytes, zeroed.</summary>
    public NCollection_AliasedArray(int theStride, int theLength)
    {
        myStride = theStride;
        mySize = theLength;
        myDeletable = true;
        if (theLength <= 0 || myStride <= 0)
        {
            throw new Standard_RangeError("NCollection_AliasedArray, stride and length should be positive");
        }

        myData = allocate(theLength);
    }

    /// <summary>A copy (OCCT's copy constructor).</summary>
    public NCollection_AliasedArray(NCollection_AliasedArray theOther)
    {
        myStride = theOther.myStride;
        mySize = theOther.mySize;
        if (mySize != 0)
        {
            myDeletable = true;
            myData = allocate(mySize);
            Assign(theOther);
        }
    }

    /// <summary>
    /// An array over <paramref name="theLength"/> elements of another array's storage, starting at byte
    /// <paramref name="theByteOffset"/>, without copying them (OCCT's constructor wrapping a C array). The elements of
    /// <paramref name="theData"/> must be of the wrapped type, <paramref name="theStride"/> bytes each.
    /// </summary>
    protected NCollection_AliasedArray(Array theData, int theByteOffset, int theStride, int theLength)
    {
        myData = theData;
        myOffset = theByteOffset;
        myStride = theStride;
        mySize = theLength;
        if (theLength <= 0)
        {
            throw new Standard_RangeError("NCollection_AliasedArray, length should be positive");
        }

        if ((long)theByteOffset + (long)theStride * theLength > theData.LongLength * theStride || theByteOffset < 0)
        {
            throw new Standard_OutOfRange("NCollection_AliasedArray, wrapped range is out of the array");
        }
    }

    /// <summary>
    /// Allocates zeroed storage for <paramref name="theLength"/> elements of <see cref="myStride"/> bytes. A subclass
    /// may return an array of its element type; the default is a byte array.
    /// </summary>
    protected virtual Array allocate(int theLength) => new byte[(long)myStride * theLength];

    /// <summary>The element size, in bytes.</summary>
    public int Stride() => myStride;

    public int Size() => mySize;

    public int Length() => mySize;

    public bool IsEmpty() => mySize == 0;

    public int Lower() => 0;

    public int Upper() => mySize - 1;

    public bool IsDeletable() => myDeletable;

    public bool IsAllocated() => myDeletable;

    /// <summary>The size of the data, in bytes.</summary>
    public long SizeBytes() => (long)myStride * mySize;

    /// <summary>
    /// Copies the data of an array with the same stride and length; otherwise throws
    /// <see cref="Standard_DimensionMismatch"/>.
    /// </summary>
    public NCollection_AliasedArray Assign(NCollection_AliasedArray theOther)
    {
        if (!ReferenceEquals(theOther, this))
        {
            if (myStride != theOther.myStride || mySize != theOther.mySize)
            {
                throw new Standard_DimensionMismatch("NCollection_AliasedArray::Assign(), arrays have different size");
            }

            if (myData is not null)
            {
                theOther.bytes().CopyTo(bytes());
            }
        }

        return this;
    }

    /// <summary>
    /// Takes the storage of <paramref name="theOther"/>. As in OCCT, the moved-from array keeps referring to the same
    /// storage (it only stops owning it).
    /// </summary>
    public NCollection_AliasedArray Move(NCollection_AliasedArray theOther)
    {
        if (!ReferenceEquals(theOther, this))
        {
            myStride = theOther.myStride;
            mySize = theOther.mySize;
            myDeletable = theOther.myDeletable;
            myData = theOther.myData;
            myOffset = theOther.myOffset;
            theOther.myDeletable = false;
        }

        return this;
    }

    /// <summary>
    /// Resizes the array. Nothing happens if the length doesn't change; otherwise new storage is allocated, and the
    /// old values are copied into it if <paramref name="theToCopyData"/> is set.
    /// </summary>
    public void Resize(int theLength, bool theToCopyData)
    {
        if (theLength <= 0)
        {
            throw new Standard_RangeError("NCollection_AliasedArray::Resize, length should be positive");
        }

        if (mySize == theLength)
        {
            return;
        }

        int anOldLen = mySize;
        Array? anOldData = myData;
        int anOldOffset = myOffset;
        mySize = theLength;
        myData = allocate(theLength);
        myOffset = 0;
        if (!theToCopyData)
        {
            myDeletable = true;
            return;
        }

        if (anOldData is not null)
        {
            int aLenCopy = Min(anOldLen, theLength) * myStride;
            MemoryMarshal.CreateReadOnlySpan(ref Unsafe.Add(ref MemoryMarshal.GetArrayDataReference(anOldData), anOldOffset), aLenCopy)
                .CopyTo(bytes());
        }

        myDeletable = true;
    }

    /// <summary>The raw bytes of an element (OCCT's <c>value()</c>).</summary>
    public ReadOnlySpan<byte> value(int theIndex) => MemoryMarshal.CreateReadOnlySpan(ref byteRef(theIndex), myStride);

    /// <summary>The raw bytes of an element, for writing (OCCT's <c>changeValue()</c>).</summary>
    public Span<byte> changeValue(int theIndex) => MemoryMarshal.CreateSpan(ref byteRef(theIndex), myStride);

    /// <summary>Sets every element to <paramref name="theValue"/>.</summary>
    public void Init<T>(T theValue) where T : unmanaged
    {
        for (int anIter = 0; anIter < mySize; ++anIter)
        {
            ChangeValue<T>(anIter) = theValue;
        }
    }

    /// <summary>The element at an index, as a type whose size is the stride.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public T Value<T>(int theIndex) where T : unmanaged
    {
        Standard_TypeMismatch_Raise_if(myStride != Unsafe.SizeOf<T>(), "NCollection_AliasedArray::Value(), wrong type");
        return Unsafe.ReadUnaligned<T>(ref byteRef(theIndex));
    }

    /// <summary>The element at an index, as a type whose size is the stride, for writing.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public ref T ChangeValue<T>(int theIndex) where T : unmanaged
    {
        Standard_TypeMismatch_Raise_if(myStride != Unsafe.SizeOf<T>(), "NCollection_AliasedArray::ChangeValue(), wrong type");
        return ref Unsafe.As<byte, T>(ref byteRef(theIndex));
    }

    /// <summary>The start of the element at an index, as a type no larger than the stride (e.g. 2 components of 3).</summary>
    public T Value2<T>(int theIndex) where T : unmanaged
    {
        Standard_TypeMismatch_Raise_if(myStride < Unsafe.SizeOf<T>(), "NCollection_AliasedArray::Value2(), wrong type");
        return Unsafe.ReadUnaligned<T>(ref byteRef(theIndex));
    }

    /// <summary>The start of the element at an index, as a type no larger than the stride, for writing.</summary>
    public ref T ChangeValue2<T>(int theIndex) where T : unmanaged
    {
        Standard_TypeMismatch_Raise_if(myStride < Unsafe.SizeOf<T>(), "NCollection_AliasedArray::ChangeValue2(), wrong type");
        return ref Unsafe.As<byte, T>(ref byteRef(theIndex));
    }

    public T First<T>() where T : unmanaged => Value<T>(0);

    public ref T ChangeFirst<T>() where T : unmanaged => ref ChangeValue<T>(0);

    public T Last<T>() where T : unmanaged => Value<T>(mySize - 1);

    // OCCT returns Value<>() here, which doesn't compile if used
    public ref T ChangeLast<T>() where T : unmanaged => ref ChangeValue<T>(mySize - 1);

    /// <summary>
    /// The elements as a span of a type whose size is the stride, without copying. Valid until the array is resized.
    /// </summary>
    public Span<T> AsSpan<T>() where T : unmanaged
    {
        Standard_TypeMismatch_Raise_if(myStride != Unsafe.SizeOf<T>(), "NCollection_AliasedArray::AsSpan(), wrong type");
        if (myData is null || mySize == 0)
        {
            return [];
        }

        return MemoryMarshal.CreateSpan(ref Unsafe.As<byte, T>(ref Unsafe.Add(ref MemoryMarshal.GetArrayDataReference(myData), myOffset)), mySize);
    }

    /// <summary>The storage and the byte offset of element 0 in it, for views sharing this array's data.</summary>
    internal (Array? Data, int ByteOffset) Storage() => (myData, myOffset);

    private Span<byte> bytes()
        => myData is null
               ? []
               : MemoryMarshal.CreateSpan(ref Unsafe.Add(ref MemoryMarshal.GetArrayDataReference(myData), myOffset), checked(mySize * myStride));

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private ref byte byteRef(int theIndex)
    {
        if ((uint)theIndex >= (uint)mySize || myData is null)
        {
            throw new Standard_OutOfRange("NCollection_AliasedArray::value(), out of range index");
        }

        return ref Unsafe.Add(ref MemoryMarshal.GetArrayDataReference(myData), myOffset + myStride * theIndex);
    }
}
