// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_CartesianPointHasher.pxx
// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_DirectionHasher.pxx
// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_VectorHasher.pxx
// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_LineHasher.pxx
// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_PlaneHasher.pxx
// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_CircleHasher.pxx
// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_Axis2Placement2dHasher.pxx
// Port of: src/DataExchange/TKDESTEP/StepTidy/StepTidy_Axis2Placement3dHasher.pxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2024-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Foundation;

namespace Kerf.Engine.Exchange.Step.Tidy;

/// <summary>What the hashers share: how names and values compare, and how they hash.</summary>
internal static class TidyHash
{
    // Values merge when they're exactly equal (0 and -0 are). OCCT's hashers compare within 1e-12, but its hash reads the
    // array of three doubles with a length of 3 taken as bytes: only low bytes of the first coordinate. So what it merges is
    // in effect what's exactly equal, and all round first coordinates share a bucket. Hashing properly and keeping 1e-12
    // gave 20.7% over the dataset, not 17.9%, and changed what one file read back as (decision 1665).
    public static int Of(double value) => value.GetHashCode();

    public static int Of(ReadOnlySpan<double> values)
    {
        var hash = default(HashCode);
        foreach (double value in values)
        {
            hash.Add(Of(value));
        }
        return hash.ToHashCode();
    }

    public static int WithName(int hash, TCollection_HAsciiString? name)
        => name is null ? hash : HashCode.Combine(hash, name.ToCString());

    public static bool SameName(TCollection_HAsciiString? name1, TCollection_HAsciiString? name2)
        => name1 is null || name2 is null ? name1 is null && name2 is null : name1.IsSameString(name2);

    public static bool Same(double value1, double value2) => value1 == value2;

    public static bool Same(ReadOnlySpan<double> values1, ReadOnlySpan<double> values2)
    {
        if (values1.Length != values2.Length)
        {
            return false;
        }
        for (int i = 0; i < values1.Length; ++i)
        {
            if (!Same(values1[i], values2[i]))
            {
                return false;
            }
        }
        return true;
    }
}

/// <summary>Cartesian points: their names and coordinates.</summary>
internal sealed class CartesianPointHasher : IEqualityComparer<StepCartesianPoint>
{
    public static CartesianPointHasher Instance { get; } = new();

    public int GetHashCode(StepCartesianPoint point) => TidyHash.WithName(TidyHash.Of(Values(point)), point.Name());

    public bool Equals(StepCartesianPoint? point1, StepCartesianPoint? point2)
        => TidyHash.SameName(point1!.Name(), point2!.Name()) && TidyHash.Same(Values(point1), Values(point2));

    // The coordinates it has: a 2D point has room for three, and isn't the 3D point with a zero Z
    private static ReadOnlySpan<double> Values(StepCartesianPoint point) => point.Coordinates()[..point.CoordinateCount];
}

/// <summary>Directions: their names and ratios.</summary>
internal sealed class DirectionHasher : IEqualityComparer<StepDirection>
{
    public static DirectionHasher Instance { get; } = new();

    public int GetHashCode(StepDirection direction) => TidyHash.WithName(TidyHash.Of(Values(direction)), direction.Name());

    public bool Equals(StepDirection? direction1, StepDirection? direction2)
        => TidyHash.SameName(direction1!.Name(), direction2!.Name())
           && TidyHash.Same(Values(direction1), Values(direction2));

    // The ratios it has, as for a point
    private static ReadOnlySpan<double> Values(StepDirection direction) => direction.DirectionRatios()[..direction.DirectionRatioCount];
}

/// <summary>Vectors: their names, magnitudes and orientations.</summary>
internal sealed class VectorHasher : IEqualityComparer<StepVector>
{
    public static VectorHasher Instance { get; } = new();

    public int GetHashCode(StepVector vector)
        => TidyHash.WithName(HashCode.Combine(DirectionHasher.Instance.GetHashCode(vector.Orientation()), TidyHash.Of(vector.Magnitude())), vector.Name());

    public bool Equals(StepVector? vector1, StepVector? vector2)
        => TidyHash.SameName(vector1!.Name(), vector2!.Name())
           && TidyHash.Same(vector1.Magnitude(), vector2.Magnitude())
           && DirectionHasher.Instance.Equals(vector1.Orientation(), vector2.Orientation());
}

/// <summary>Lines: their names, points and vectors.</summary>
internal sealed class LineHasher : IEqualityComparer<StepLine>
{
    public static LineHasher Instance { get; } = new();

    public int GetHashCode(StepLine line)
        => TidyHash.WithName(HashCode.Combine(CartesianPointHasher.Instance.GetHashCode(line.Pnt()), VectorHasher.Instance.GetHashCode(line.Dir())), line.Name());

    public bool Equals(StepLine? line1, StepLine? line2)
        => TidyHash.SameName(line1!.Name(), line2!.Name())
           && CartesianPointHasher.Instance.Equals(line1.Pnt(), line2.Pnt())
           && VectorHasher.Instance.Equals(line1.Dir(), line2.Dir());
}

/// <summary>Placements in 2D: their names, locations and reference directions.</summary>
internal sealed class Axis2Placement2dHasher : IEqualityComparer<StepAxis2Placement2d>
{
    public static Axis2Placement2dHasher Instance { get; } = new();

    public int GetHashCode(StepAxis2Placement2d placement)
        => TidyHash.WithName(
            HashCode.Combine(CartesianPointHasher.Instance.GetHashCode(placement.Location()),
                             placement.HasRefDirection ? DirectionHasher.Instance.GetHashCode(placement.RefDirection()) : 0),
            placement.Name());

    public bool Equals(StepAxis2Placement2d? placement1, StepAxis2Placement2d? placement2)
        => TidyHash.SameName(placement1!.Name(), placement2!.Name())
           && CartesianPointHasher.Instance.Equals(placement1.Location(), placement2.Location())
           && placement1.HasRefDirection == placement2.HasRefDirection
           && (!placement1.HasRefDirection || DirectionHasher.Instance.Equals(placement1.RefDirection(), placement2.RefDirection()));
}

/// <summary>Placements in 3D: their names, locations, axes and reference directions.</summary>
internal sealed class Axis2Placement3dHasher : IEqualityComparer<StepAxis2Placement3d>
{
    public static Axis2Placement3dHasher Instance { get; } = new();

    public int GetHashCode(StepAxis2Placement3d placement)
        => TidyHash.WithName(
            HashCode.Combine(CartesianPointHasher.Instance.GetHashCode(placement.Location()),
                             placement.HasAxis ? DirectionHasher.Instance.GetHashCode(placement.Axis()) : 0,
                             placement.HasRefDirection ? DirectionHasher.Instance.GetHashCode(placement.RefDirection()) : 0),
            placement.Name());

    public bool Equals(StepAxis2Placement3d? placement1, StepAxis2Placement3d? placement2)
        => TidyHash.SameName(placement1!.Name(), placement2!.Name())
           && CartesianPointHasher.Instance.Equals(placement1.Location(), placement2.Location())
           && placement1.HasAxis == placement2.HasAxis
           && (!placement1.HasAxis || DirectionHasher.Instance.Equals(placement1.Axis(), placement2.Axis()))
           && placement1.HasRefDirection == placement2.HasRefDirection
           && (!placement1.HasRefDirection || DirectionHasher.Instance.Equals(placement1.RefDirection(), placement2.RefDirection()));
}

/// <summary>Planes: their names and placements.</summary>
internal sealed class PlaneHasher : IEqualityComparer<StepPlane>
{
    public static PlaneHasher Instance { get; } = new();

    public int GetHashCode(StepPlane plane) => TidyHash.WithName(Axis2Placement3dHasher.Instance.GetHashCode(plane.Position()), plane.Name());

    public bool Equals(StepPlane? plane1, StepPlane? plane2)
        => TidyHash.SameName(plane1!.Name(), plane2!.Name()) && Axis2Placement3dHasher.Instance.Equals(plane1.Position(), plane2.Position());
}

/// <summary>Circles: their names, placements (in 2D or 3D) and radii.</summary>
internal sealed class CircleHasher : IEqualityComparer<StepCircle>
{
    public static CircleHasher Instance { get; } = new();

    public int GetHashCode(StepCircle circle)
    {
        StepAxis2Placement position = circle.Position();
        int positionHash = position.Axis2Placement2d() is { } placement2d ? Axis2Placement2dHasher.Instance.GetHashCode(placement2d)
            : position.Axis2Placement3d() is { } placement3d ? Axis2Placement3dHasher.Instance.GetHashCode(placement3d)
            : 0;
        return TidyHash.WithName(HashCode.Combine(positionHash, TidyHash.Of(circle.Radius())), circle.Name());
    }

    public bool Equals(StepCircle? circle1, StepCircle? circle2)
    {
        if (!TidyHash.SameName(circle1!.Name(), circle2!.Name()))
        {
            return false;
        }
        StepAxis2Placement position1 = circle1.Position();
        StepAxis2Placement position2 = circle2.Position();
        if (position1.CaseNumber() != position2.CaseNumber())
        {
            return false;
        }
        if (position1.CaseNumber() == 1 && !Axis2Placement2dHasher.Instance.Equals(position1.Axis2Placement2d(), position2.Axis2Placement2d()))
        {
            return false;
        }
        if (position1.CaseNumber() == 2 && !Axis2Placement3dHasher.Instance.Equals(position1.Axis2Placement3d(), position2.Axis2Placement3d()))
        {
            return false;
        }
        return TidyHash.Same(circle1.Radius(), circle2.Radius());
    }
}
