// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Array.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_ArrayOfEdges.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_ArrayOfPointNormal.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_ArrayOfPoints.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_ArrayOfSectionLines.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_ArrayOfTangentZones.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_ArrayOfTriangles.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_PMaillageAffinage.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Intersection.Polyhedra;

/// <summary>
/// IntPolyh's dynamic array, indexed from 0: a number of "allocated" (constructed) items and a smaller number of stored
/// items, which <see cref="IncrementNbItems"/> grows, allocating <c>aIncrement</c> more items when it reaches the end.
/// </summary>
/// <remarks>
/// <para>
/// OCCT's typedefs are written out: <c>IntPolyh_ArrayOfPoints</c> is <c>IntPolyh_Array&lt;IntPolyh_Point&gt;</c>, and
/// likewise ArrayOfEdges, ArrayOfTriangles, ArrayOfTangentZones (of <see cref="PolyhedronStartPoint"/>),
/// ArrayOfSectionLines and ArrayOfPointNormal. <c>IntPolyh_PMaillageAffinage</c> (a raw pointer) is a reference to
/// <see cref="PolyhedronMaillageAffinage"/>.
/// </para>
/// <para>
/// As OCCT's <c>NCollection_DynamicArray</c>, the items live in blocks that never move, so a <c>ref</c> to an item
/// stays valid while the array grows: <c>IntPolyh_Triangle::MiddleRefinement</c> appends triangles to the array that
/// holds the triangle being refined. Items are constructed with <c>new T()</c> (OCCT's default constructors), and
/// <see cref="Init"/> on an array that already holds items keeps them, as OCCT's does (decision 591).
/// </para>
/// <para>
/// A working object of the intersection (decision 21): its readers are public, its mutators internal.
/// </para>
/// </remarks>
internal sealed class PolyhedronArray<T> where T : new()
{
    // OCCT's NCollection_DynamicArray default block size
    private const int THE_BLOCK_SIZE = 256;

    // Value types are constructed once and copied; IntPolyh_SectionLine (a class) is constructed for each item.
    private static readonly T? Prototype = typeof(T).IsValueType ? new T() : default;

    private readonly List<T[]> _blocks = [];
    private int _length; // the items constructed (NCollection_DynamicArray::Length)

    private int _nbAllocated;
    private int _nbItems;
    private readonly int _increment;

    /// <summary>An empty array, growing by <paramref name="aIncrement"/> items; call <see cref="Init"/> first.</summary>
    public PolyhedronArray(int aIncrement = 256)
    {
        _nbAllocated = 0;
        _nbItems = 0;
        _increment = aIncrement;
    }

    /// <summary>An array with <paramref name="aN"/> allocated items, growing by <paramref name="aIncrement"/>.</summary>
    public PolyhedronArray(int aN, int aIncrement)
    {
        _nbItems = 0;
        _increment = aIncrement;
        Init(aN);
    }

    /// <summary>Copies the other array (its items, the stored and allocated numbers); C++'s <c>operator=</c>.</summary>
    internal PolyhedronArray<T> Copy(PolyhedronArray<T> aOther)
    {
        if (ReferenceEquals(this, aOther))
        {
            return this;
        }

        Clear();
        Init(aOther._nbAllocated);
        // myVectorOfType = aOther.myVectorOfType: the other's items, as many as it has
        Clear();
        for (int i = 0; i < aOther._length; ++i)
        {
            T item = aOther.ItemAt(i);
            if (!typeof(T).IsValueType && item is PolyhedronSectionLine sl)
            {
                item = (T)(object)new PolyhedronSectionLine(sl);
            }

            AppendItem(item);
        }

        _nbItems = aOther._nbItems;
        return this;
    }

    /// <summary>Allocates (constructs) items up to index <paramref name="aN"/>; the item at <paramref name="aN"/> is
    /// reset to a default one, the others are kept.</summary>
    internal void Init(int aN)
    {
        SetValue(aN, NewItem());
        _nbAllocated = aN;
    }

    /// <summary>Increments the number of stored items, allocating more when needed.</summary>
    internal void IncrementNbItems()
    {
        _nbItems++;
        if (_nbItems >= _nbAllocated)
        {
            int n = _nbAllocated + _increment;
            Init(n);
        }
    }

    /// <summary>The number of allocated items.</summary>
    public int GetN() => _nbAllocated;

    /// <summary>The number of stored items.</summary>
    public int ItemCount => _nbItems;

    /// <summary>Sets the number of stored items.</summary>
    internal void SetNbItems(int aNb) => _nbItems = aNb;

    /// <summary>The item at <paramref name="aIndex"/> (any allocated item, stored or not).</summary>
    public ref readonly T Value(int aIndex) => ref ChangeValue(aIndex);

    /// <summary>The item at <paramref name="aIndex"/>, for writing; the reference stays valid while the array grows.</summary>
    internal ref T ChangeValue(int aIndex)
    {
        if (aIndex < 0 || aIndex >= _length)
        {
            throw new OutOfRangeException("NCollection_DynamicArray::ChangeValue");
        }

        return ref _blocks[aIndex / THE_BLOCK_SIZE][aIndex % THE_BLOCK_SIZE];
    }

    /// <summary>C++'s <c>operator[]</c>.</summary>
    internal ref T this[int aIndex] => ref ChangeValue(aIndex);

    /// <summary>Writes the stored items that have a <c>Dump()</c>, as OCCT's <c>Dump()</c> prints them.</summary>
    public void Dump(TextWriter stream)
    {
        stream.Write($"\n ArrayOfSectionLines 0-> {Format.D(_nbItems - 1)}");
        for (int i = 0; i < _nbItems; i++)
        {
            if (Value(i) is IPolyhedronDump item)
            {
                item.Dump(stream);
            }
        }

        stream.Write("\n");
    }

    private static T NewItem() => typeof(T).IsValueType ? Prototype! : new T();

    private T ItemAt(int aIndex) => _blocks[aIndex / THE_BLOCK_SIZE][aIndex % THE_BLOCK_SIZE];

    private void Clear()
    {
        _blocks.Clear();
        _length = 0;
    }

    private void AppendItem(T item)
    {
        if (_length == _blocks.Count * THE_BLOCK_SIZE)
        {
            _blocks.Add(new T[THE_BLOCK_SIZE]);
        }

        _blocks[_length / THE_BLOCK_SIZE][_length % THE_BLOCK_SIZE] = item;
        _length++;
    }

    // NCollection_DynamicArray::SetValue: grows with default items up to the index, then sets it
    private void SetValue(int aIndex, T value)
    {
        while (_length <= aIndex)
        {
            AppendItem(NewItem());
        }

        ChangeValue(aIndex) = value;
    }
}

/// <summary>A point of a surface with the surface's normal there (OCCT's struct of IntPolyh_ArrayOfPointNormal).</summary>
/// <remarks>Both are zero by default, as C++ default-constructs <c>gp_Pnt</c> and <c>gp_Vec</c>.</remarks>
internal struct PolyhedronPointNormal : IEquatable<PolyhedronPointNormal>
{
    public Point3D Point;
    public Vector3D Normal;

    public readonly bool Equals(PolyhedronPointNormal other) => Point.Equals(other.Point) && Normal.Equals(other.Normal);

    public override readonly bool Equals(object? obj) => obj is PolyhedronPointNormal other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(Point, Normal);

    public static bool operator ==(in PolyhedronPointNormal left, in PolyhedronPointNormal right) => left.Equals(right);

    public static bool operator !=(in PolyhedronPointNormal left, in PolyhedronPointNormal right) => !left.Equals(right);

    public override readonly string ToString() => $"IntPolyh_PointNormal({Point}, {Normal})";
}
