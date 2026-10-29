// Port of: src/ModelingAlgorithms/TKMesh/BRepMesh/BRepMesh_Circle.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2013-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A 2D circle stored as its centre and radius only (three doubles).</summary>
/// <remarks>The default circle is the origin with radius 0, as OCCT's; <c>default(T)</c> is the same.</remarks>
public partial struct BRepMesh_Circle : IEquatable<BRepMesh_Circle>
{
    private gp_XY myLocation;
    private double myRadius;

    /// <summary>A circle with the given centre and radius.</summary>
    public BRepMesh_Circle(in gp_XY theLocation, double theRadius)
    {
        myLocation = theLocation;
        myRadius = theRadius;
    }

    /// <summary>Sets the centre.</summary>
    internal void SetLocation(in gp_XY theLocation) => myLocation = theLocation;

    /// <summary>Sets the radius.</summary>
    internal void SetRadius(double theRadius) => myRadius = theRadius;

    /// <summary>This circle with another centre.</summary>
    public readonly BRepMesh_Circle WithLocation(in gp_XY theLocation) => new(theLocation, myRadius);

    /// <summary>This circle with another radius.</summary>
    public readonly BRepMesh_Circle WithRadius(double theRadius) => new(myLocation, theRadius);

    /// <summary>The centre.</summary>
    public readonly gp_XY Location() => myLocation;

    /// <summary>The radius.</summary>
    public readonly double Radius() => myRadius;

    public readonly bool Equals(BRepMesh_Circle other) => myLocation.Equals(other.myLocation) && myRadius.Equals(other.myRadius);

    public override readonly bool Equals(object? obj) => obj is BRepMesh_Circle other && Equals(other);

    public override readonly int GetHashCode() => HashCode.Combine(myLocation, myRadius);

    public static bool operator ==(in BRepMesh_Circle theLeft, in BRepMesh_Circle theRight) => theLeft.Equals(theRight);

    public static bool operator !=(in BRepMesh_Circle theLeft, in BRepMesh_Circle theRight) => !theLeft.Equals(theRight);

    public override readonly string ToString() => gp.Format("BRepMesh_Circle", myLocation.X(), myLocation.Y(), myRadius);
}
