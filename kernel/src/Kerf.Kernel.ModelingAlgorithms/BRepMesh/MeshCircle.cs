// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Circle.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Meshing;

/// <summary>A 2D circle stored as its centre and radius only (three doubles).</summary>
/// <remarks>The default circle is the origin with radius 0, as OCCT's; <c>default(T)</c> is the same.</remarks>
internal partial struct MeshCircle : IEquatable<MeshCircle>
{
    private Xy _location;
    private double _radius;

    /// <summary>A circle with the given centre and radius.</summary>
    public MeshCircle(in Xy location, double radius)
    {
        _location = location;
        _radius = radius;
    }

    /// <summary>Sets the centre.</summary>
    internal void SetLocation(in Xy location) => _location = location;

    /// <summary>Sets the radius.</summary>
    internal void SetRadius(double radius) => _radius = radius;

    /// <summary>This circle with another centre.</summary>
    public readonly MeshCircle WithLocation(in Xy location) => new(location, _radius);

    /// <summary>This circle with another radius.</summary>
    public readonly MeshCircle WithRadius(double radius) => new(_location, radius);

    /// <summary>The centre.</summary>
    public readonly Xy Location() => _location;

    /// <summary>The radius.</summary>
    public readonly double Radius() => _radius;

    public readonly bool Equals(MeshCircle other) => _location.Equals(other._location) && _radius.Equals(other._radius);

    public override readonly bool Equals(object? obj) => obj is MeshCircle other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(_location, _radius);

    public static bool operator ==(in MeshCircle left, in MeshCircle right) => left.Equals(right);

    public static bool operator !=(in MeshCircle left, in MeshCircle right) => !left.Equals(right);

    public override readonly string ToString() => Constants.Format("MeshCircle", _location.X(), _location.Y(), _radius);
}
