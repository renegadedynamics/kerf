// Port of: src/FoundationClasses/TKMath/Poly/Poly_ArrayOfNodes.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_ArrayOfNodes.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2021-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.CompilerServices;

namespace Occt;

/// <summary>
/// An array of 3D nodes, in double precision (<see cref="gp_Pnt"/>, the default) or single precision
/// (<see cref="NCollection_Vec3{T}"/> of float), chosen before allocation. Indexed from 0.
/// </summary>
/// <remarks>
/// The storage is a <c>gp_Pnt[]</c> or a <c>NCollection_Vec3&lt;float&gt;[]</c> (24 or 12 bytes a node, x, y, z in
/// order), so the nodes can be read in bulk without copying: <c>AsSpan&lt;gp_Pnt&gt;()</c> or
/// <c>AsSpan&lt;NCollection_Vec3&lt;float&gt;&gt;()</c> according to <see cref="IsDoublePrecision"/>. OCCT's
/// <c>size_t</c> overloads of Value and SetValue are left out.
/// </remarks>
public class Poly_ArrayOfNodes : NCollection_AliasedArray
{
    private static readonly int THE_DOUBLE_STRIDE = Unsafe.SizeOf<gp_Pnt>();
    private static readonly int THE_FLOAT_STRIDE = Unsafe.SizeOf<NCollection_Vec3<float>>();

    /// <summary>An empty double-precision array.</summary>
    public Poly_ArrayOfNodes() : base(THE_DOUBLE_STRIDE) { }

    /// <summary>A double-precision array of <paramref name="theLength"/> nodes at the origin.</summary>
    public Poly_ArrayOfNodes(int theLength) : base(THE_DOUBLE_STRIDE, theLength) { }

    /// <summary>A copy (OCCT's copy constructor).</summary>
    public Poly_ArrayOfNodes(Poly_ArrayOfNodes theOther) : base(theOther) { }

    /// <summary>An array over <paramref name="theLength"/> points of <paramref name="theBegin"/> from <paramref name="theStart"/>, without copying them.</summary>
    public Poly_ArrayOfNodes(gp_Pnt[] theBegin, int theStart, int theLength)
        : base(theBegin, theStart * THE_DOUBLE_STRIDE, THE_DOUBLE_STRIDE, theLength) { }

    /// <summary>An array over <paramref name="theLength"/> single-precision points of <paramref name="theBegin"/> from <paramref name="theStart"/>, without copying them.</summary>
    public Poly_ArrayOfNodes(NCollection_Vec3<float>[] theBegin, int theStart, int theLength)
        : base(theBegin, theStart * THE_FLOAT_STRIDE, THE_FLOAT_STRIDE, theLength) { }

    protected override Array allocate(int theLength)
        => myStride == THE_DOUBLE_STRIDE ? new gp_Pnt[theLength] : new NCollection_Vec3<float>[theLength];

    /// <summary>True if the nodes are in double precision.</summary>
    public bool IsDoublePrecision() => myStride == THE_DOUBLE_STRIDE;

    /// <summary>Sets whether the nodes are in double or single precision. Throws if the array is already allocated.</summary>
    public void SetDoublePrecision(bool theIsDouble)
    {
        if (myData is not null)
        {
            throw new Standard_ProgramError("Poly_ArrayOfNodes::SetDoublePrecision() should be called before allocation");
        }

        myStride = theIsDouble ? THE_DOUBLE_STRIDE : THE_FLOAT_STRIDE;
    }

    /// <summary>
    /// Copies the nodes of another array of the same length, converting between precisions if they differ.
    /// </summary>
    public Poly_ArrayOfNodes Assign(Poly_ArrayOfNodes theOther)
    {
        if (ReferenceEquals(theOther, this))
        {
            return this;
        }

        if (myStride == theOther.myStride)
        {
            // fast copy
            base.Assign(theOther);
            return this;
        }

        // slow copy
        if (mySize != theOther.mySize)
        {
            throw new Standard_DimensionMismatch("Poly_ArrayOfNodes::Assign(), arrays have different sizes");
        }

        for (int anIter = 0; anIter < mySize; ++anIter)
        {
            gp_Pnt aPnt = theOther.Value(anIter);
            SetValue(anIter, aPnt);
        }

        return this;
    }

    /// <summary>Takes the storage of another array (which keeps referring to it, as in OCCT).</summary>
    public Poly_ArrayOfNodes Move(Poly_ArrayOfNodes theOther)
    {
        base.Move(theOther);
        return this;
    }

    /// <summary>The node at an index, in double precision whatever the storage.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public gp_Pnt Value(int theIndex)
    {
        if (myStride == THE_DOUBLE_STRIDE)
        {
            return Value<gp_Pnt>(theIndex);
        }
        else
        {
            NCollection_Vec3<float> aVec3 = Value<NCollection_Vec3<float>>(theIndex);
            return new gp_Pnt(aVec3.x(), aVec3.y(), aVec3.z());
        }
    }

    /// <summary>Sets the node at an index (rounded to float in single precision).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void SetValue(int theIndex, in gp_Pnt theValue)
    {
        if (myStride == THE_DOUBLE_STRIDE)
        {
            ChangeValue<gp_Pnt>(theIndex) = theValue;
        }
        else
        {
            ref NCollection_Vec3<float> aVec3 = ref ChangeValue<NCollection_Vec3<float>>(theIndex);
            aVec3.SetValues((float)theValue.X(), (float)theValue.Y(), (float)theValue.Z());
        }
    }

    /// <summary>The node at an index (OCCT's <c>operator[]</c>).</summary>
    public gp_Pnt this[int theIndex] => Value(theIndex);
}
