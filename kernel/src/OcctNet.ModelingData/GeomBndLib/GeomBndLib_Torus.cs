// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Torus.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Torus.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Computes bounding box for a toroidal surface (Geom_ToroidalSurface) through BndLib; the precise box by
/// PSO + Powell optimization (GeomBndLib_OtherSurface).
/// </summary>
public sealed class GeomBndLib_Torus : GeomBndLib_Surface.ISurfaceEvaluator
{
    private readonly Geom_ToroidalSurface myGeom;

    public GeomBndLib_Torus(Geom_ToroidalSurface theSurf)
    {
        myGeom = theSurf;
    }

    public Geom_ToroidalSurface Geometry() => myGeom;

    /// <summary>Compute bounding box for full torus.</summary>
    public Bnd_Box Box(double theTol)
    {
        Bnd_Box aBox = new();
        BndLib.Add(myGeom.Torus(), theTol, ref aBox);
        return aBox;
    }

    /// <summary>Compute bounding box for torus patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Bnd_Box Box(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
    {
        Bnd_Box aBox = new();
        BndLib.Add(myGeom.Torus(), theUMin, theUMax, theVMin, theVMax, theTol, ref aBox);
        return aBox;
    }

    /// <summary>Compute precise bounding box for torus patch using PSO + Powell optimization.</summary>
    public Bnd_Box BoxOptimal(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
    {
        GeomAdaptor_Surface anAdaptor = new(myGeom);
        GeomBndLib_OtherSurface anOther = new(anAdaptor);
        return anOther.BoxOptimal(theUMin, theUMax, theVMin, theVMax, theTol);
    }

    /// <summary>Compute precise bounding box for full torus.</summary>
    public Bnd_Box BoxOptimal(double theTol)
    {
        GeomAdaptor_Surface anAdaptor = new(myGeom);
        GeomBndLib_OtherSurface anOther = new(anAdaptor);
        return anOther.BoxOptimal(theTol);
    }
}
