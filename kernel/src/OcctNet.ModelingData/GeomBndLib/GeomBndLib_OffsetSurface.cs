// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OffsetSurface.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_OffsetSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;
using static Kerf.Geometry.SurfaceKind;

namespace Occt;

/// <summary>
/// Computes bounding box for an offset surface (Geom_OffsetSurface): through its analytic equivalent surface when
/// it has one, otherwise the basis surface's box enlarged by |offset|.
/// </summary>
public sealed class GeomBndLib_OffsetSurface : GeomBndLib_Surface.ISurfaceEvaluator
{
    private readonly Geom_OffsetSurface myGeom;

    public GeomBndLib_OffsetSurface(Geom_OffsetSurface theSurf)
    {
        myGeom = theSurf;
    }

    public Geom_OffsetSurface Geometry() => myGeom;

    /// <summary>Compute bounding box for full surface.</summary>
    public Box Box(double theTol)
    {
        myGeom.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        return Box(aU1, aU2, aV1, aV2, theTol);
    }

    /// <summary>Compute bounding box for surface patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Box Box(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
    {
        Geom_Surface? anEquiv = myGeom.Surface();
        if (anEquiv is not null)
        {
            GeomBndLib_Surface aSurfEval0 = new(anEquiv);
            if (aSurfEval0.GetType() != OtherSurface)
            {
                return aSurfEval0.Box(theUMin, theUMax, theVMin, theVMax, theTol);
            }
        }

        // Conservative fallback: basis surface box enlarged by |offset|.
        Geom_Surface aBasis = myGeom.BasisSurface();
        double anOffset = Math.Abs(myGeom.Offset());
        GeomBndLib_Surface aSurfEval = new(aBasis);
        Box aLocalBox = aSurfEval.Box(theUMin, theUMax, theVMin, theVMax, 0.0);
        // OCCT: Enlarge keeps the larger gap, so the tolerance is not added to the offset (occt-issues.md).
        aLocalBox.Enlarge(anOffset);
        aLocalBox.Enlarge(theTol);
        return aLocalBox;
    }

    /// <summary>Compute precise bounding box for full surface.</summary>
    public Box BoxOptimal(double theTol)
    {
        myGeom.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        return BoxOptimal(aU1, aU2, aV1, aV2, theTol);
    }

    /// <summary>Compute precise bounding box for surface patch using PSO numerical optimization.</summary>
    public Box BoxOptimal(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
    {
        // Try the analytic equivalent surface first (e.g. offset plane -> plane).
        Geom_Surface? anEquiv = myGeom.Surface();
        if (anEquiv is not null)
        {
            GeomBndLib_Surface aSurfEval = new(anEquiv);
            if (aSurfEval.GetType() != OtherSurface)
            {
                return aSurfEval.BoxOptimal(theUMin, theUMax, theVMin, theVMax, theTol);
            }
        }

        // Sample the actual offset surface directly for a tight bounding box.
        GeomAdaptor_Surface anAdaptor = new(myGeom);
        GeomBndLib_OtherSurface anOther = new(anAdaptor);
        return anOther.BoxOptimal(theUMin, theUMax, theVMin, theVMax, theTol);
    }
}
