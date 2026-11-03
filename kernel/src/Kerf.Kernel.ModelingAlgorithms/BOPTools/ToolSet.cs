// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_Set.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPTools/BOPTools_Set.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Booleans.Tools;

/// <summary>
/// The set of the sub-shapes of a given type of a shape (degenerated edges left out, INTERNAL ones taken both
/// FORWARD and REVERSED), compared as a set of shapes: BOPAlgo keys maps by it to find shapes made of the same
/// sub-shapes.
/// </summary>
/// <remarks>
/// <para>
/// A working object filled by <see cref="Add"/> and then used as a map key (decision 6432), so a sealed class
/// with OCCT's copy constructor and <see cref="Assign"/> written out where OCCT copies by value.
/// </para>
/// <para>
/// Equality is OCCT's <see cref="IsEqual"/>: the same number of shapes, and every shape of the other set
/// <see cref="Shape.IsSame"/> as one of this set. The hash is OCCT's <see cref="GetSum"/>, a sum of the shapes'
/// normalised <see cref="ShapeHasher"/> hashes. The hash values differ from native's (native hashes
/// addresses), which changes no result: the port's maps iterate in insertion order (decision 18).
/// </para>
/// </remarks>
public sealed class ToolSet : IEquatable<ToolSet>
{
    private NCollection_DynamicArray<Shape> _shapes;
    private Shape _shape;
    private int _nbShapes;
    private ulong _sum;
    private int _upper;

    /// <summary>An empty set.</summary>
    public ToolSet()
    {
        _shapes = new NCollection_DynamicArray<Shape>(256);
        _nbShapes = 0;
        _sum = 0;
        _upper = 432123;
    }

    /// <summary>A copy of the set (OCCT's copy constructor).</summary>
    public ToolSet(ToolSet other)
    {
        _shape = other._shape;
        _nbShapes = other._nbShapes;
        _sum = other._sum;
        _upper = other._upper;
        _shapes = new NCollection_DynamicArray<Shape>(other._shapes);
    }

    /// <summary>Makes this set a copy of the other (OCCT's assignment).</summary>
    public ToolSet Assign(ToolSet other)
    {
        _shape = other._shape;
        _nbShapes = other._nbShapes;
        _sum = other._sum;
        _upper = other._upper;
        _shapes = new NCollection_DynamicArray<Shape>(other._shapes);
        return this;
    }

    /// <summary>The shape the set was made from.</summary>
    public Shape Shape() => _shape;

    /// <summary>
    /// Fills the set with the sub-shapes of type <paramref name="type"/> of <paramref name="s"/>, replacing its
    /// content. Don't call it on a set that is a key of a map.
    /// </summary>
    public void Add(in Shape s, ShapeType type)
    {
        ulong id, idN;
        Orientation or;
        Explorer exp = new();
        //
        _shape = s;
        _shapes.Clear();
        _nbShapes = 0;
        _sum = 0;
        //
        exp.Init(s, type);
        for (; exp.More(); exp.Next())
        {
            Shape sx = exp.Current();
            if (type == ShapeType.Edge)
            {
                Edge ex = ShapeCasts.Edge(sx);
                if (Tool.Degenerated(ex))
                {
                    continue;
                }
            }
            //
            or = sx.Orientation();
            if (or == Internal)
            {
                Shape sy;
                //
                sy = sx;
                //
                sy.Orientation(Forward);
                _shapes.Append(sy);
                //
                sy.Orientation(Reversed);
                _shapes.Append(sy);
            }
            else
            {
                _shapes.Append(sx);
            }
        }
        //
        _nbShapes = _shapes.Length();
        if (_nbShapes == 0)
        {
            return;
        }
        //
        for (int i = 0; i < _shapes.Length(); ++i)
        {
            Shape sx = _shapes.Value(i);
            // OCCT's size_t hash; the port's is an int, taken as unsigned
            id = unchecked((ulong)(uint)ShapeHasher.Instance.GetHashCode(sx)) % (ulong)_upper + 1;
            idN = NormalizedIds(id, _nbShapes);
            _sum = unchecked(_sum + idN);
        }
    }

    /// <summary>The number of shapes in the set (INTERNAL ones count twice).</summary>
    public int ShapeCount => _nbShapes;

    /// <summary>True if the sets have the same number of shapes and every shape of the other is in this one (by IsSame).</summary>
    public bool IsEqual(ToolSet aOther)
    {
        if (aOther._nbShapes != _nbShapes)
        {
            return false;
        }
        //
        HashSet<Shape> m1 = new(ShapeHasher.Instance);
        //
        for (int i = 0; i < _shapes.Length(); ++i)
        {
            m1.Add(_shapes.Value(i));
        }
        //
        for (int i = 0; i < aOther._shapes.Length(); ++i)
        {
            if (!m1.Contains(aOther._shapes.Value(i)))
            {
                return false;
            }
        }
        //
        return true;
    }

    /// <summary>The hash of the set: the sum of its shapes' normalised hashes.</summary>
    public ulong GetSum() => _sum;

    /// <summary>Empties the set.</summary>
    private void Clear()
    {
        _nbShapes = 0;
        _sum = 0;
        _shapes.Clear();
    }

    private static ulong NormalizedIds(ulong aId, int aDiv)
    {
        ulong max, tresh, idRet;
        //
        idRet = aId;
        max = ulong.MaxValue;
        tresh = max / (ulong)aDiv;
        if (aId > tresh)
        {
            idRet = aId % tresh;
        }
        return idRet;
    }

    /// <inheritdoc/>
    public bool Equals(ToolSet? other) => other is not null && IsEqual(other);

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is ToolSet set && IsEqual(set);

    /// <summary>OCCT's <c>std::hash</c>: <see cref="GetSum"/>, folded.</summary>
    public override int GetHashCode() => _sum.GetHashCode();
}
