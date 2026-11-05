// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OffsetSurface.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OffsetSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for an offset surface (Geom_OffsetSurface): through its analytic equivalent surface when
/// it has one, otherwise the basis surface's box enlarged by |offset|.
/// </summary>
internal sealed class OffsetSurfaceBounds : SurfaceBounds.ISurfaceEvaluator
{
    private readonly global::Kerf.Geometry.OffsetSurface _geom;

    public OffsetSurfaceBounds(global::Kerf.Geometry.OffsetSurface surf)
    {
        _geom = surf;
    }

    public global::Kerf.Geometry.OffsetSurface Geometry() => _geom;

    /// <summary>Compute bounding box for full surface.</summary>
    public Box Box(double tol)
    {
        _geom.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
        return Box(u1, u2, v1, v2, tol);
    }

    /// <summary>Compute bounding box for surface patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Box Box(double uMin, double uMax, double vMin, double vMax, double tol)
    {
        Surface? equiv = _geom.Surface();
        if (equiv is not null)
        {
            SurfaceBounds surfEval0 = new(equiv);
            if (surfEval0.GetType() != OtherSurface)
            {
                return surfEval0.Box(uMin, uMax, vMin, vMax, tol);
            }
        }

        // Conservative fallback: basis surface box enlarged by |offset|.
        Surface basis = _geom.BasisSurface;
        double offset = Math.Abs(_geom.Offset);
        SurfaceBounds surfEval = new(basis);
        Box localBox = surfEval.Box(uMin, uMax, vMin, vMax, 0.0);
        // OCCT: Enlarge keeps the larger gap, so the tolerance is not added to the offset (occt-issues.md).
        localBox.Enlarge(offset);
        localBox.Enlarge(tol);
        return localBox;
    }

    /// <summary>Compute precise bounding box for full surface.</summary>
    public Box BoxOptimal(double tol)
    {
        _geom.ParameterBounds(out double u1, out double u2, out double v1, out double v2);
        return BoxOptimal(u1, u2, v1, v2, tol);
    }

    /// <summary>Compute precise bounding box for surface patch using PSO numerical optimization.</summary>
    public Box BoxOptimal(double uMin, double uMax, double vMin, double vMax, double tol)
    {
        // Try the analytic equivalent surface first (e.g. offset plane -> plane).
        Surface? equiv = _geom.Surface();
        if (equiv is not null)
        {
            SurfaceBounds surfEval = new(equiv);
            if (surfEval.GetType() != OtherSurface)
            {
                return surfEval.BoxOptimal(uMin, uMax, vMin, vMax, tol);
            }
        }

        // Sample the actual offset surface directly for a tight bounding box.
        SurfaceAdaptor adaptor = new(_geom);
        OtherSurfaceBounds other = new(adaptor);
        return other.BoxOptimal(uMin, uMax, vMin, vMax, tol);
    }
}
