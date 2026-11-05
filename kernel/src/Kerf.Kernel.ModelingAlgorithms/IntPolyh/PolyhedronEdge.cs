// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Edge.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPolyh/IntPolyh_Edge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine;

namespace Kerf.Engine.Intersection.Polyhedra;

/// <summary>An edge between two IntPolyh points (by index), linked to two IntPolyh triangles (by index).</summary>
/// <remarks>
/// OCCT's default edge has every index -1; <c>default(IntPolyh_Edge)</c> has zeros, so ported code writes
/// <c>new IntPolyh_Edge()</c>.
/// </remarks>
internal partial struct PolyhedronEdge : IEquatable<PolyhedronEdge>
{
    private int _point1;
    private int _point2;
    private int _triangle1;
    private int _triangle2;

    /// <summary>An edge with no points and no triangles (every index -1).</summary>
    public PolyhedronEdge()
    {
        _point1 = -1;
        _point2 = -1;
        _triangle1 = -1;
        _triangle2 = -1;
    }

    public PolyhedronEdge(int point1, int point2, int triangle1, int triangle2)
    {
        _point1 = point1;
        _point2 = point2;
        _triangle1 = triangle1;
        _triangle2 = triangle2;
    }

    public readonly int FirstPoint() => _point1;

    public readonly int SecondPoint() => _point2;

    public readonly int FirstTriangle() => _triangle1;

    public readonly int SecondTriangle() => _triangle2;

    // Mutators (internal: see docs/translation-rules.md)

    internal void SetFirstPoint(int point) => _point1 = point;

    internal void SetSecondPoint(int point) => _point2 = point;

    internal void SetFirstTriangle(int triangle) => _triangle1 = triangle;

    internal void SetSecondTriangle(int triangle) => _triangle2 = triangle;

    public readonly PolyhedronEdge WithFirstPoint(int point)
    {
        PolyhedronEdge copy = this;
        copy.SetFirstPoint(point);
        return copy;
    }

    public readonly PolyhedronEdge WithSecondPoint(int point)
    {
        PolyhedronEdge copy = this;
        copy.SetSecondPoint(point);
        return copy;
    }

    public readonly PolyhedronEdge WithFirstTriangle(int triangle)
    {
        PolyhedronEdge copy = this;
        copy.SetFirstTriangle(triangle);
        return copy;
    }

    public readonly PolyhedronEdge WithSecondTriangle(int triangle)
    {
        PolyhedronEdge copy = this;
        copy.SetSecondTriangle(triangle);
        return copy;
    }

    /// <summary>Writes the edge with its index, as OCCT's <c>Dump(i)</c> prints it.</summary>
    public readonly void Dump(int i, TextWriter stream)
    {
        stream.Write($"\nEdge({Format.D(i, 3)}) : P1:{Format.D(_point1, 5)}   "
                        + $"P2:{Format.D(_point2, 5)}   T1:{Format.D(_triangle1, 5)}   "
                        + $"T2:{Format.D(_triangle2, 5)}\n");
    }

    public readonly bool Equals(PolyhedronEdge other)
        => _point1 == other._point1 && _point2 == other._point2 && _triangle1 == other._triangle1
           && _triangle2 == other._triangle2;

    public override readonly bool Equals(object? obj) => obj is PolyhedronEdge other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_point1, _point2, _triangle1, _triangle2);

    public static bool operator ==(in PolyhedronEdge left, in PolyhedronEdge right) => left.Equals(right);

    public static bool operator !=(in PolyhedronEdge left, in PolyhedronEdge right) => !left.Equals(right);

    public override readonly string ToString()
        => $"IntPolyh_Edge(P {_point1}, {_point2}; T {_triangle1}, {_triangle2})";
}
