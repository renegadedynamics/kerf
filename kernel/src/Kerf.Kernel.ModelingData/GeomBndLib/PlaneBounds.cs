// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Plane.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a plane (Geom_Plane). Handles infinite parameters by opening box sides based on the
/// plane normal direction.
/// </summary>
internal sealed class PlaneBounds : SurfaceBounds.ISurfaceEvaluator
{
    private readonly PlaneSurface _geom;

    public PlaneBounds(PlaneSurface plane)
    {
        _geom = plane;
    }

    public PlaneSurface Geometry() => _geom;

    /// <summary>Compute bounding box for full plane (infinite).</summary>
    public Box Box(double tol)
    {
        _geom.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
        return Box(u1, u2, v1, v2, tol);
    }

    /// <summary>Compute bounding box for plane patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Box Box(double uMin, double uMax, double vMin, double vMax, double tol)
    {
        Plane3D pln = _geom.Plane;
        Box box = new();

        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(vMin) || global::Kerf.Engine.Foundation.Precision.IsInfinite(vMax)
            || global::Kerf.Engine.Foundation.Precision.IsInfinite(uMin) || global::Kerf.Engine.Foundation.Precision.IsInfinite(uMax))
        {
            TreatInfinitePlane(pln, uMin, uMax, vMin, vMax, tol, ref box);
            return box;
        }

        box.Add(global::Kerf.Engine.Geometry.ElSLib.Value(uMin, vMin, pln));
        box.Add(global::Kerf.Engine.Geometry.ElSLib.Value(uMin, vMax, pln));
        box.Add(global::Kerf.Engine.Geometry.ElSLib.Value(uMax, vMin, pln));
        box.Add(global::Kerf.Engine.Geometry.ElSLib.Value(uMax, vMax, pln));
        box.Enlarge(tol);
        return box;
    }

    /// <summary>For analytical surfaces, BoxOptimal is same as Box.</summary>
    public Box BoxOptimal(double uMin, double uMax, double vMin, double vMax, double tol)
        => Box(uMin, uMax, vMin, vMax, tol);

    /// <summary>Compute optimal bounding box for full surface.</summary>
    public Box BoxOptimal(double tol) => Box(tol);

    /// <summary>Compute barycenter point for infinite plane.</summary>
    private static Point3D BaryCenter(in Plane3D plane, double uMin, double uMax, double vMin, double vMax)
    {
        double u, v;
        bool isU1Inf = global::Kerf.Engine.Foundation.Precision.IsInfinite(uMin);
        bool isU2Inf = global::Kerf.Engine.Foundation.Precision.IsInfinite(uMax);
        bool isV1Inf = global::Kerf.Engine.Foundation.Precision.IsInfinite(vMin);
        bool isV2Inf = global::Kerf.Engine.Foundation.Precision.IsInfinite(vMax);

        if (isU1Inf && isU2Inf)
            u = 0;
        else if (isU1Inf)
            u = uMax - 10.0;
        else if (isU2Inf)
            u = uMin + 10.0;
        else
            u = (uMin + uMax) / 2.0;

        if (isV1Inf && isV2Inf)
            v = 0;
        else if (isV1Inf)
            v = vMax - 10.0;
        else if (isV2Inf)
            v = vMin + 10.0;
        else
            v = (vMin + vMax) / 2.0;

        return global::Kerf.Engine.Geometry.ElSLib.Value(u, v, plane);
    }

    /// <summary>Handle infinite plane by opening appropriate box sides.</summary>
    private static void TreatInfinitePlane(in Plane3D plane,
                                           double uMin,
                                           double uMax,
                                           double vMin,
                                           double vMax,
                                           double tol,
                                           ref Box box)
    {
        Direction3D norm = plane.Axis.Direction;
        double angularTol = RealEpsilon();

        Point3D location = BaryCenter(plane, uMin, uMax, vMin, vMax);

        if (norm.IsParallel(Constants.DX(), angularTol))
        {
            box.Add(location);
            box.OpenYmin();
            box.OpenYmax();
            box.OpenZmin();
            box.OpenZmax();
        }
        else if (norm.IsParallel(Constants.DY(), angularTol))
        {
            box.Add(location);
            box.OpenXmin();
            box.OpenXmax();
            box.OpenZmin();
            box.OpenZmax();
        }
        else if (norm.IsParallel(Constants.DZ(), angularTol))
        {
            box.Add(location);
            box.OpenXmin();
            box.OpenXmax();
            box.OpenYmin();
            box.OpenYmax();
        }
        else
        {
            box.SetWhole();
            return;
        }

        box.Enlarge(tol);
    }
}
