// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Cone.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Cone.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a conical surface (Geom_ConicalSurface) from the boundary circle arcs, opened along the
/// axis for infinite V.
/// </summary>
public sealed class ConeBounds : SurfaceBounds.ISurfaceEvaluator
{
    private readonly ConicalSurface _geom;

    public ConeBounds(ConicalSurface surf)
    {
        _geom = surf;
    }

    public ConicalSurface Geometry() => _geom;

    /// <summary>For analytical surfaces, BoxOptimal is same as Box.</summary>
    public Box BoxOptimal(double uMin, double uMax, double vMin, double vMax, double tol)
        => Box(uMin, uMax, vMin, vMax, tol);

    /// <summary>Compute optimal bounding box for full surface.</summary>
    public Box BoxOptimal(double tol) => Box(tol);

    /// <summary>Compute bounding box for a finite cone patch.</summary>
    private static void ComputeCone(in Cone cone,
                                    double uMin,
                                    double uMax,
                                    double vMin,
                                    double vMax,
                                    ref Box box)
    {
        CoordinateSystem pos = cone.Position;
        double r = cone.RefRadius;
        double sAng = cone.SemiAngle;

        // Add circle at VMin.
        Circle3D c = global::Kerf.Engine.Geometry.ElSLib.ConeVIso(pos, r, sAng, vMin);
        if (c.Radius > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            box.Add(CircleBounds.Box(c, uMin, uMax, 0.0));
        }
        else
        {
            box.Add(c.Location);
        }

        // Add circle at VMax.
        c = global::Kerf.Engine.Geometry.ElSLib.ConeVIso(pos, r, sAng, vMax);
        if (c.Radius > global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            box.Add(CircleBounds.Box(c, uMin, uMax, 0.0));
        }
        else
        {
            box.Add(c.Location);
        }
    }

    /// <summary>Compute bounding box for full cone.</summary>
    public Box Box(double tol)
    {
        _geom.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
        return Box(u1, u2, v1, v2, tol);
    }

    /// <summary>Compute bounding box for cone patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Box Box(double uMin, double uMax, double vMin, double vMax, double tol)
    {
        Box box = new();
        Cone cone = _geom.Cone;
        Direction3D dir = cone.Axis.Direction;

        // OCCT opens the box only along the axis for an infinite V: the cone widens away from its apex, so the
        // radial directions of an axis-aligned cone stay closed and the box does not contain the surface
        // (occt-issues.md).
        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMin))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMax))
            {
                throw new FailureException("GeomBndLib_Cone::Box - bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMax))
            {
                // Fully infinite V: add the circle at V=0 to give the box finite radial extent.
                Circle3D c = global::Kerf.Engine.Geometry.ElSLib.ConeVIso(cone.Position, cone.RefRadius, cone.SemiAngle, 0.0);
                if (c.Radius > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    box.Add(CircleBounds.Box(c, uMin, uMax, 0.0));
                }
                else
                {
                    box.Add(c.Location);
                }
                InfiniteHelpers.OpenMinMax(dir, ref box);
            }
            else
            {
                ComputeCone(cone, uMin, uMax, 0.0, vMax, ref box);
                InfiniteHelpers.OpenMin(dir, ref box);
            }
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMin))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMax))
            {
                // Reversed infinite V: add the circle at V=0 to give the box finite radial extent.
                Circle3D c = global::Kerf.Engine.Geometry.ElSLib.ConeVIso(cone.Position, cone.RefRadius, cone.SemiAngle, 0.0);
                if (c.Radius > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    box.Add(CircleBounds.Box(c, uMin, uMax, 0.0));
                }
                else
                {
                    box.Add(c.Location);
                }
                InfiniteHelpers.OpenMinMax(dir, ref box);
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMax))
            {
                throw new FailureException("GeomBndLib_Cone::Box - bad parameter");
            }
            else
            {
                ComputeCone(cone, uMin, uMax, 0.0, vMax, ref box);
                InfiniteHelpers.OpenMax(dir, ref box);
            }
        }
        else
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMax))
            {
                ComputeCone(cone, uMin, uMax, vMin, 0.0, ref box);
                InfiniteHelpers.OpenMin(dir, ref box);
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMax))
            {
                ComputeCone(cone, uMin, uMax, vMin, 0.0, ref box);
                InfiniteHelpers.OpenMax(dir, ref box);
            }
            else
            {
                ComputeCone(cone, uMin, uMax, vMin, vMax, ref box);
            }
        }

        box.Enlarge(tol);
        return box;
    }
}
