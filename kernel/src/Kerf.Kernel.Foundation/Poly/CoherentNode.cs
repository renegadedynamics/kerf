// Port of: src/FoundationClasses/TKMath/Poly/Poly_CoherentNode.hxx
// Port of: src/FoundationClasses/TKMath/Poly/Poly_CoherentNode.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2007-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine;
using System.Globalization;

namespace Kerf.Engine.Meshing;

/// <summary>
/// A node of a <see cref="CoherentTriangulation"/>: a 3D point, UV coordinates, a normal, the list of triangles
/// using the node and an index (normally the node's index in the original triangulation).
/// </summary>
/// <remarks>
/// OCCT's node derives from gp_XYZ, a value type; here it is a class holding one (<see cref="XYZ"/>, with
/// <see cref="X"/>, <see cref="Y"/>, <see cref="Z"/> and <see cref="SetCoord"/> for convenience), since nodes are
/// elements of a linked structure edited in place. OCCT's allocator arguments are dropped.
/// </remarks>
internal class CoherentNode
{
    private Xyz _xyz;
    private double _uv0;
    private double _uv1;
    private CoherentTriPtr? _triangles;
    private int _index;
    private float _normal0;
    private float _normal1;
    private float _normal2;

    /// <summary>A node at the origin, without UV coordinates, normal or triangles.</summary>
    /// <remarks>OCCT leaves the normal uninitialised here; the port's is zero (no normal).</remarks>
    public CoherentNode()
    {
        _xyz = new Xyz(0.0, 0.0, 0.0);
        _triangles = null;
        _index = -1;
        _uv0 = global::Kerf.Engine.Foundation.Precision.Infinite();
        _uv1 = global::Kerf.Engine.Foundation.Precision.Infinite();
        // OCCT leaves myNormal uninitialised
    }

    /// <summary>A node at a point, without UV coordinates, normal or triangles.</summary>
    public CoherentNode(in Xyz pnt)
    {
        _xyz = pnt;
        _triangles = null;
        _index = -1;
        _uv0 = global::Kerf.Engine.Foundation.Precision.Infinite();
        _uv1 = global::Kerf.Engine.Foundation.Precision.Infinite();
        _normal0 = 0.0f;
        _normal1 = 0.0f;
        _normal2 = 0.0f;
    }

    /// <summary>The point.</summary>
    public Xyz XYZ() => _xyz;

    public double X() => _xyz.X();

    public double Y() => _xyz.Y();

    public double Z() => _xyz.Z();

    /// <summary>Sets the point.</summary>
    public void SetCoord(double x, double y, double z) => _xyz = new Xyz(x, y, z);

    /// <summary>Sets the UV coordinates.</summary>
    public void SetUV(double u, double v)
    {
        _uv0 = u;
        _uv1 = v;
    }

    /// <summary>The U coordinate (infinite if not set).</summary>
    public double GetU() => _uv0;

    /// <summary>The V coordinate (infinite if not set).</summary>
    public double GetV() => _uv1;

    /// <summary>Sets the normal, rounded to single precision.</summary>
    public void SetNormal(in Xyz vector)
    {
        _normal0 = (float)vector.X();
        _normal1 = (float)vector.Y();
        _normal2 = (float)vector.Z();
    }

    /// <summary>True if the node has a normal (of square length above Precision.Confusion()).</summary>
    public bool HasNormal
        => (_normal0 * _normal0 + _normal1 * _normal1 + _normal2 * _normal2) > global::Kerf.Engine.Foundation.Precision.Confusion();

    /// <summary>The normal.</summary>
    public Xyz GetNormal() => new(_normal0, _normal1, _normal2);

    /// <summary>Sets the index.</summary>
    public void SetIndex(int index) => _index = index;

    /// <summary>The index.</summary>
    public int GetIndex() => _index;

    /// <summary>True if no triangle uses this node.</summary>
    public bool IsFreeNode => _triangles is null;

    /// <summary>Resets the node to the origin, without UV coordinates or normal, and releases its list of triangles.</summary>
    /// <remarks>
    /// OCCT frees the list but keeps pointing to it, so the node is not free afterwards (and iterating its triangles
    /// reads freed memory). The port keeps the list.
    /// </remarks>
    public void Clear()
    {
        CoherentTriPtr.RemoveList(_triangles); // OCCT: myTriangles is not reset
        _uv0 = global::Kerf.Engine.Foundation.Precision.Infinite();
        _uv1 = global::Kerf.Engine.Foundation.Precision.Infinite();
        _normal0 = 0.0f;
        _normal1 = 0.0f;
        _normal2 = 0.0f;
        SetCoord(0.0, 0.0, 0.0);
    }

    /// <summary>Connects a triangle to this node.</summary>
    public void AddTriangle(CoherentTriangle tri)
    {
        if (_triangles is null)
        {
            _triangles = new CoherentTriPtr(tri);
        }
        else
        {
            _triangles.Prepend(tri);
        }
    }

    /// <summary>Disconnects a triangle from this node; false if it wasn't connected.</summary>
    public bool RemoveTriangle(CoherentTriangle tri)
    {
        bool result = false;
        if (_triangles!.GetTriangle() == tri)
        {
            CoherentTriPtr lostPtr = _triangles;
            if (_triangles == _triangles.Next())
            {
                _triangles = null;
            }
            else
            {
                _triangles = _triangles.Next();
            }

            CoherentTriPtr.Remove(lostPtr);
            result = true;
        }
        else
        {
            var iter = new CoherentTriPtr.Iterator(_triangles);
            for (iter.Next(); iter.More(); iter.Next())
            {
                if (iter.Value() == tri)
                {
                    CoherentTriPtr.Remove(iter.PtrValue());
                    result = true;
                    break;
                }
            }
        }

        return result;
    }

    /// <summary>An iterator of the triangles using this node.</summary>
    public CoherentTriPtr.Iterator TriangleIterator() => new(_triangles);

    /// <summary>Writes the point and the triangles of this node.</summary>
    public void Dump(TextWriter stream)
    {
        stream.Write("  X =" + F94(X()) + "; Y =" + F94(Y()) + "; Z =" + F94(Z()) + '\n');
        var iter = new CoherentTriPtr.Iterator(_triangles);
        for (; iter.More(); iter.Next())
        {
            CoherentTriangle tri = iter.Value();
            stream.Write("      " + D5(tri.Node(0)) + " " + D5(tri.Node(1)) + " " + D5(tri.Node(2)) + '\n');
        }
    }

    // printf's %9.4f and %5d
    private static string F94(double value) => value.ToString("F4", CultureInfo.InvariantCulture).PadLeft(9);

    private static string D5(int value) => value.ToString(CultureInfo.InvariantCulture).PadLeft(5);

    public override string ToString() => $"Poly_CoherentNode({_xyz}, index {_index})";
}
