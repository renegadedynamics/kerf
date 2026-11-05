// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Vertex.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine;
using System.Diagnostics.CodeAnalysis;
using static Kerf.Engine.Meshing.DegreeOfFreedom;

namespace Kerf.Engine.Meshing;

/// <summary>
/// A vertex of the mesh in the parametric space, possibly associated with a 3D point stored elsewhere (by its index).
/// </summary>
/// <remarks>
/// The default vertex is the origin, with no 3D point and <c>BRepMesh_Free</c>, as OCCT's; <c>default(T)</c> is the
/// same. <see cref="Equals(MeshVertex)"/> and <c>==</c> compare the fields exactly (decision 539); OCCT's
/// <c>operator==</c>, which compares within <c>Precision::PConfusion()</c> and never matches a deleted vertex, is
/// <see cref="IsEqual"/>, and maps in ported code take <see cref="Hasher"/>, OCCT's hash and equality.
/// </remarks>
internal partial struct MeshVertex : IEquatable<MeshVertex>
{
    private Xy _uv;
    private int _location3d;
    private DegreeOfFreedom _movability;

    /// <summary>A vertex associated with the 3D point of index <paramref name="location3d"/>.</summary>
    public MeshVertex(in Xy uv, int location3d, DegreeOfFreedom movability)
    {
        Initialize(uv, location3d, movability);
    }

    /// <summary>A vertex without a 3D point.</summary>
    public MeshVertex(double u, double v, DegreeOfFreedom movability)
    {
        _uv = new Xy(u, v);
        _location3d = 0;
        _movability = movability;
    }

    /// <summary>Sets the position, the index of the 3D point and the movability.</summary>
    internal void Initialize(in Xy uv, int location3d, DegreeOfFreedom movability)
    {
        _uv = uv;
        _location3d = location3d;
        _movability = movability;
    }

    /// <summary>The position in the parametric space.</summary>
    public readonly Xy Coord() => _uv;

    /// <summary>The position in the parametric space, for writing.</summary>
    [UnscopedRef]
    internal ref Xy ChangeCoord() => ref _uv;

    /// <summary>The index of the associated 3D point.</summary>
    public readonly int Location3d() => _location3d;

    /// <summary>The movability.</summary>
    public readonly DegreeOfFreedom Movability() => _movability;

    /// <summary>Sets the movability.</summary>
    internal void SetMovability(DegreeOfFreedom movability) => _movability = movability;

    /// <summary>This vertex with another movability (the twin of <c>SetMovability</c>).</summary>
    public readonly MeshVertex WithMovability(DegreeOfFreedom movability)
    {
        MeshVertex copy = this;
        copy.SetMovability(movability);
        return copy;
    }

    /// <summary>This vertex at another position (the twin of writing through <c>ChangeCoord</c>).</summary>
    public readonly MeshVertex WithCoord(in Xy uv)
    {
        MeshVertex copy = this;
        copy._uv = uv;
        return copy;
    }

    /// <summary>
    /// OCCT's equality: false if either vertex is deleted, else whether the positions are equal within
    /// <c>Precision::PConfusion()</c>.
    /// </summary>
    public readonly bool IsEqual(in MeshVertex other)
    {
        if (_movability == DegreeOfFreedom.Deleted || other._movability == DegreeOfFreedom.Deleted)
        {
            return false;
        }

        return _uv.IsEqual(other._uv, global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    /// <summary>OCCT's hasher: <c>std::hash&lt;BRepMesh_Vertex&gt;</c> and <c>operator==</c> (<see cref="IsEqual"/>).</summary>
    public sealed class Hasher : IEqualityComparer<MeshVertex>
    {
        public static readonly Hasher Instance = new();

        public bool Equals(MeshVertex v1, MeshVertex v2) => v1.IsEqual(v2);

        public int GetHashCode(MeshVertex vertex)
            => (Math.Floor(1e5 * vertex.Coord().X()) * Math.Floor(1e5 * vertex.Coord().Y())).GetHashCode();
    }

    public readonly bool Equals(MeshVertex other)
        => _uv.Equals(other._uv) && _location3d == other._location3d && _movability == other._movability;

    public override readonly bool Equals(object? obj) => obj is MeshVertex other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_uv, _location3d, _movability);

    public static bool operator ==(in MeshVertex left, in MeshVertex right) => left.Equals(right);

    public static bool operator !=(in MeshVertex left, in MeshVertex right) => !left.Equals(right);

    public override readonly string ToString() => $"BRepMesh_Vertex({_uv}, {_location3d}, {_movability})";
}
