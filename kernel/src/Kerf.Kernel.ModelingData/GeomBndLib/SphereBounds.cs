// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Sphere.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Sphere.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a spherical surface (Geom_SphericalSurface): the six axis-aligned extremal points that
/// lie in the patch, and the boundary iso-circles.
/// </summary>
public sealed class SphereBounds : SurfaceBounds.ISurfaceEvaluator
{
    private readonly SphericalSurface _geom;

    public SphereBounds(SphericalSurface surf)
    {
        _geom = surf;
    }

    public SphericalSurface Geometry() => _geom;

    /// <summary>For analytical surfaces, BoxOptimal is same as Box.</summary>
    public Box BoxOptimal(double uMin, double uMax, double vMin, double vMax, double tol)
        => Box(uMin, uMax, vMin, vMax, tol);

    /// <summary>Compute optimal bounding box for full surface.</summary>
    public Box BoxOptimal(double tol) => Box(tol);

    /// <summary>Compute bounding box for full sphere.</summary>
    public Box Box(double tol)
    {
        Box box = new();
        Sphere sphere = _geom.Sphere;
        Point3D p = sphere.Location;
        double r = sphere.Radius;
        box.Update(p.X - r, p.Y - r, p.Z - r, p.X + r, p.Y + r, p.Z + r);
        box.Enlarge(tol);
        return box;
    }

    /// <summary>Compute bounding box for a sphere patch.</summary>
    private static void ComputeSphere(in Sphere sphere,
                                      double uMin,
                                      double uMax,
                                      double vMin,
                                      double vMax,
                                      ref Box box)
    {
        Point3D p = sphere.Location;
        double r = sphere.Radius;

        double xmin = p.X - r;
        double xmax = p.X + r;
        double ymin = p.Y - r;
        double ymax = p.Y + r;
        double zmin = p.Z - r;
        double zmax = p.Z + r;

        double uper = 2.0 * M_PI - global::Kerf.Engine.Foundation.Precision.PConfusion();
        double vper = M_PI - global::Kerf.Engine.Foundation.Precision.PConfusion();
        if (uMax - uMin >= uper && vMax - vMin >= vper)
        {
            // Whole sphere.
            box.Update(xmin, ymin, zmin, xmax, ymax, zmax);
            return;
        }

        // Check 6 extremal points (axis-aligned extrema of the full sphere).
        double u, v;
        double umax = uMin + 2.0 * M_PI;
        CoordinateSystem pos = sphere.Position;
        Point3D pExt = p;

        // -X extremum
        pExt = pExt.WithX(xmin);
        global::Kerf.Engine.Geometry.ElSLib.SphereParameters(pos, r, pExt, out u, out v);
        u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, uMin, umax);
        if (u >= uMin && u <= uMax && v >= vMin && v <= vMax)
        {
            box.Add(pExt);
        }
        // +X extremum
        pExt = pExt.WithX(xmax);
        global::Kerf.Engine.Geometry.ElSLib.SphereParameters(pos, r, pExt, out u, out v);
        u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, uMin, umax);
        if (u >= uMin && u <= uMax && v >= vMin && v <= vMax)
        {
            box.Add(pExt);
        }
        pExt = pExt.WithX(p.X);
        // -Y extremum
        pExt = pExt.WithY(ymin);
        global::Kerf.Engine.Geometry.ElSLib.SphereParameters(pos, r, pExt, out u, out v);
        u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, uMin, umax);
        if (u >= uMin && u <= uMax && v >= vMin && v <= vMax)
        {
            box.Add(pExt);
        }
        // +Y extremum
        pExt = pExt.WithY(ymax);
        global::Kerf.Engine.Geometry.ElSLib.SphereParameters(pos, r, pExt, out u, out v);
        u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, uMin, umax);
        if (u >= uMin && u <= uMax && v >= vMin && v <= vMax)
        {
            box.Add(pExt);
        }
        pExt = pExt.WithY(p.Y);
        // -Z extremum
        pExt = pExt.WithZ(zmin);
        global::Kerf.Engine.Geometry.ElSLib.SphereParameters(pos, r, pExt, out u, out v);
        u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, uMin, umax);
        if (u >= uMin && u <= uMax && v >= vMin && v <= vMax)
        {
            box.Add(pExt);
        }
        // +Z extremum
        pExt = pExt.WithZ(zmax);
        global::Kerf.Engine.Geometry.ElSLib.SphereParameters(pos, r, pExt, out u, out v);
        u = global::Kerf.Engine.Geometry.ElCLib.InPeriod(u, uMin, umax);
        if (u >= uMin && u <= uMax && v >= vMin && v <= vMax)
        {
            box.Add(pExt);
        }

        // Add boundary iso-curves of the patch.
        Circle3D c = global::Kerf.Engine.Geometry.ElSLib.SphereUIso(pos, r, uMin);
        box.Add(CircleBounds.Box(c, vMin, vMax, 0.0));
        c = global::Kerf.Engine.Geometry.ElSLib.SphereUIso(pos, r, uMax);
        box.Add(CircleBounds.Box(c, vMin, vMax, 0.0));
        c = global::Kerf.Engine.Geometry.ElSLib.SphereVIso(pos, r, vMin);
        box.Add(CircleBounds.Box(c, uMin, uMax, 0.0));
        c = global::Kerf.Engine.Geometry.ElSLib.SphereVIso(pos, r, vMax);
        box.Add(CircleBounds.Box(c, uMin, uMax, 0.0));
    }

    /// <summary>Compute bounding box for sphere patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Box Box(double uMin, double uMax, double vMin, double vMax, double tol)
    {
        Sphere sphere = _geom.Sphere;

        // Check if it is a whole sphere.
        if (Math.Abs(uMin) < global::Kerf.Engine.Foundation.Precision.Angular()
            && Math.Abs(uMax - 2.0 * M_PI) < global::Kerf.Engine.Foundation.Precision.Angular()
            && Math.Abs(vMin + M_PI / 2.0) < global::Kerf.Engine.Foundation.Precision.Angular()
            && Math.Abs(vMax - M_PI / 2.0) < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            return Box(tol);
        }

        Box box = new();
        ComputeSphere(sphere, uMin, uMax, vMin, vMax, ref box);
        box.Enlarge(tol);
        return box;
    }
}
