// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Cylinder.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Cylinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a cylindrical surface (Geom_CylindricalSurface) from the boundary circle arcs, opened
/// along the axis for infinite V.
/// </summary>
public sealed class CylinderBounds : SurfaceBounds.ISurfaceEvaluator
{
    private readonly CylindricalSurface _geom;

    public CylinderBounds(CylindricalSurface surf)
    {
        _geom = surf;
    }

    public CylindricalSurface Geometry() => _geom;

    /// <summary>Compute bounding box for full cylinder.</summary>
    public Box Box(double tol)
    {
        _geom.Bounds(out double u1, out double u2, out double v1, out double v2);
        return Box(u1, u2, v1, v2, tol);
    }

    /// <summary>For analytical surfaces, BoxOptimal is same as Box.</summary>
    public Box BoxOptimal(double uMin, double uMax, double vMin, double vMax, double tol)
        => Box(uMin, uMax, vMin, vMax, tol);

    /// <summary>Compute optimal bounding box for full surface.</summary>
    public Box BoxOptimal(double tol) => Box(tol);

    /// <summary>Compute bounding box for a finite cylinder patch.</summary>
    private static void ComputeCylinder(in Cylinder cyl,
                                        double uMin,
                                        double uMax,
                                        double vMin,
                                        double vMax,
                                        ref Box box)
    {
        // Add circle at VMin.
        Circle3D c = global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(cyl.Position(), cyl.Radius(), vMin);
        box.Add(CircleBounds.Box(c, uMin, uMax, 0.0));

        // Add circle at VMax by translating.
        Vector3D t = (vMax - vMin) * new Vector3D(cyl.Axis().Direction());
        c.Translate(t);
        box.Add(CircleBounds.Box(c, uMin, uMax, 0.0));
    }

    /// <summary>Compute bounding box for cylinder patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Box Box(double uMin, double uMax, double vMin, double vMax, double tol)
    {
        Box box = new();
        Cylinder cyl = _geom.Cylinder();
        Direction3D dir = cyl.Axis().Direction();

        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMin))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMax))
            {
                throw new FailureException("GeomBndLib_Cylinder::Box - bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMax))
            {
                // Fully infinite V: add the arc at V=0 to give the box finite radial extent.
                Circle3D c = global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(cyl.Position(), cyl.Radius(), 0.0);
                box.Add(CircleBounds.Box(c, uMin, uMax, 0.0));
                InfiniteHelpers.OpenMinMax(dir, ref box);
            }
            else
            {
                ComputeCylinder(cyl, uMin, uMax, 0.0, vMax, ref box);
                InfiniteHelpers.OpenMin(dir, ref box);
            }
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMin))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMax))
            {
                // Reversed infinite V: add the arc at V=0 to give the box finite radial extent.
                Circle3D c = global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(cyl.Position(), cyl.Radius(), 0.0);
                box.Add(CircleBounds.Box(c, uMin, uMax, 0.0));
                InfiniteHelpers.OpenMinMax(dir, ref box);
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMax))
            {
                throw new FailureException("GeomBndLib_Cylinder::Box - bad parameter");
            }
            else
            {
                ComputeCylinder(cyl, uMin, uMax, 0.0, vMax, ref box);
                InfiniteHelpers.OpenMax(dir, ref box);
            }
        }
        else
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(vMax))
            {
                ComputeCylinder(cyl, uMin, uMax, vMin, 0.0, ref box);
                InfiniteHelpers.OpenMin(dir, ref box);
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(vMax))
            {
                ComputeCylinder(cyl, uMin, uMax, vMin, 0.0, ref box);
                InfiniteHelpers.OpenMax(dir, ref box);
            }
            else
            {
                ComputeCylinder(cyl, uMin, uMax, vMin, vMax, ref box);
            }
        }

        box.Enlarge(tol);
        return box;
    }
}
