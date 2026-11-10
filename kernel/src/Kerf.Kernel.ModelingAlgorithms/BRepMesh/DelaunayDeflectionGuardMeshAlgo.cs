// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;

namespace Kerf.Engine.Meshing;

/// <summary>
/// For a torus, whose grid alone OCCT trusts: inserts nodes only where the mesh is further from the surface than
/// the deflection asked for, so a grid that meets it stays as it is (decision 1929). OCCT #1444's torus is 1.43 out for 0.45
/// asked. At a relative deflection the limit is the face's own, which relative mode works out from the face's and its
/// edges' sizes (decision 2056). Once the control has nothing left to add, each triangle is also measured exactly from
/// the torus at the midpoints of its three medians, where a skewed triangle is furthest from it, and gets a node at the
/// furthest of them if that is over the limit (decision 2102); a mesh within it there is left as it was.
/// </summary>
internal sealed class DelaunayDeflectionGuardMeshAlgo<RangeSplitter> : DelaunayDeflectionControlMeshAlgo<RangeSplitter>
    where RangeSplitter : DefaultRangeSplitter, new()
{
    // The torus, for the measure inside a triangle; unset on another surface, which gets none
    private bool _hasTorus;
    private Point3D _centre;
    private Direction3D _axis;
    private double _major;
    private double _minor;

    /// <inheritdoc/>
    protected override double ControlledDeflection() => Deflection.SurfaceLimit(GetDFace(), GetParameters());

    /// <inheritdoc/>
    protected override bool ClearsFrontier => true;

    /// <inheritdoc/>
    protected override bool ReportsExceeded => true;

    /// <inheritdoc/>
    protected override bool MeasuresInside => _hasTorus;

    /// <inheritdoc/>
    protected override void OptimizeMesh(Delaun mesher, ProgressRange range)
    {
        FaceSurfaceAdaptor surface = GetDFace().GetSurface();
        _hasTorus = surface.GetType() == SurfaceKind.Torus;
        if (_hasTorus)
        {
            Torus3D torus = surface.Torus();
            _centre = torus.Location;
            _axis = torus.Axis.Direction;
            _major = torus.MajorRadius;
            _minor = torus.MinorRadius;
        }

        base.OptimizeMesh(mesher, range);
    }

    /// <inheritdoc/>
    protected override void CheckInside(ReadOnlySpan<TriangleNodeInfo> nodesInfo)
    {
        if (!_hasTorus)
        {
            return;
        }

        // The midpoint of each median: half way from a node to the middle of the link opposite it
        double worstSq = -1;
        Xy worst2d = default;
        for (int i = 0; i < 3; i++)
        {
            int j = (i + 1) % 3, k = (i + 2) % 3;
            Xyz point = nodesInfo[i].Point * 0.5 + (nodesInfo[j].Point + nodesInfo[k].Point) * 0.25;
            double distance = DistanceFromTorus(point);
            double sq = distance * distance;
            if (sq > worstSq)
            {
                worstSq = sq;
                worst2d = nodesInfo[i].Point2d * 0.5 + (nodesInfo[j].Point2d + nodesInfo[k].Point2d) * 0.25;
            }
        }

        NoteSquareDeviation(worstSq);
        double limit = ControlledDeflection();
        if (worstSq > limit * limit)
        {
            QueueInside(worst2d, nodesInfo);
        }
    }

    // The point's distance from the torus, exactly: its height above the torus's plane and its distance from the axis
    // give its distance from the tube's centre circle
    private double DistanceFromTorus(in Xyz point)
    {
        double x = point.X() - _centre.X, y = point.Y() - _centre.Y, z = point.Z() - _centre.Z;
        double h = x * _axis.X + y * _axis.Y + z * _axis.Z;
        double rx = x - h * _axis.X, ry = y - h * _axis.Y, rz = z - h * _axis.Z;
        double rho = Math.Sqrt(rx * rx + ry * ry + rz * rz) - _major;
        return Math.Abs(Math.Sqrt(rho * rho + h * h) - _minor);
    }
}
