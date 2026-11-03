// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Torus.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Torus.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a toroidal surface (Geom_ToroidalSurface) through BndLib; the precise box by
/// PSO + Powell optimization (GeomBndLib_OtherSurface).
/// </summary>
internal sealed class TorusBounds : SurfaceBounds.ISurfaceEvaluator
{
    private readonly ToroidalSurface _geom;

    public TorusBounds(ToroidalSurface surf)
    {
        _geom = surf;
    }

    public ToroidalSurface Geometry() => _geom;

    /// <summary>Compute bounding box for full torus.</summary>
    public Box Box(double tol)
    {
        Box box = new();
        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(_geom.Torus, tol, ref box);
        return box;
    }

    /// <summary>Compute bounding box for torus patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Box Box(double uMin, double uMax, double vMin, double vMax, double tol)
    {
        Box box = new();
        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(_geom.Torus, uMin, uMax, vMin, vMax, tol, ref box);
        return box;
    }

    /// <summary>Compute precise bounding box for torus patch using PSO + Powell optimization.</summary>
    public Box BoxOptimal(double uMin, double uMax, double vMin, double vMax, double tol)
    {
        SurfaceAdaptor adaptor = new(_geom);
        OtherSurfaceBounds other = new(adaptor);
        return other.BoxOptimal(uMin, uMax, vMin, vMax, tol);
    }

    /// <summary>Compute precise bounding box for full torus.</summary>
    public Box BoxOptimal(double tol)
    {
        SurfaceAdaptor adaptor = new(_geom);
        OtherSurfaceBounds other = new(adaptor);
        return other.BoxOptimal(tol);
    }
}
