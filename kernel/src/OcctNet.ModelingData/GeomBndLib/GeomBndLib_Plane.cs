// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Plane.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Computes bounding box for a plane (Geom_Plane). Handles infinite parameters by opening box sides based on the
/// plane normal direction.
/// </summary>
public sealed class GeomBndLib_Plane : GeomBndLib_Surface.ISurfaceEvaluator
{
    private readonly Geom_Plane myGeom;

    public GeomBndLib_Plane(Geom_Plane thePlane)
    {
        myGeom = thePlane;
    }

    public Geom_Plane Geometry() => myGeom;

    /// <summary>Compute bounding box for full plane (infinite).</summary>
    public Bnd_Box Box(double theTol)
    {
        myGeom.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        return Box(aU1, aU2, aV1, aV2, theTol);
    }

    /// <summary>Compute bounding box for plane patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Bnd_Box Box(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
    {
        gp_Pln aPln = myGeom.Pln();
        Bnd_Box aBox = new();

        if (Precision.IsInfinite(theVMin) || Precision.IsInfinite(theVMax)
            || Precision.IsInfinite(theUMin) || Precision.IsInfinite(theUMax))
        {
            TreatInfinitePlane(aPln, theUMin, theUMax, theVMin, theVMax, theTol, ref aBox);
            return aBox;
        }

        aBox.Add(ElSLib.Value(theUMin, theVMin, aPln));
        aBox.Add(ElSLib.Value(theUMin, theVMax, aPln));
        aBox.Add(ElSLib.Value(theUMax, theVMin, aPln));
        aBox.Add(ElSLib.Value(theUMax, theVMax, aPln));
        aBox.Enlarge(theTol);
        return aBox;
    }

    /// <summary>For analytical surfaces, BoxOptimal is same as Box.</summary>
    public Bnd_Box BoxOptimal(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
        => Box(theUMin, theUMax, theVMin, theVMax, theTol);

    /// <summary>Compute optimal bounding box for full surface.</summary>
    public Bnd_Box BoxOptimal(double theTol) => Box(theTol);

    /// <summary>Compute barycenter point for infinite plane.</summary>
    private static gp_Pnt BaryCenter(in gp_Pln thePlane, double theUMin, double theUMax, double theVMin, double theVMax)
    {
        double aU, aV;
        bool isU1Inf = Precision.IsInfinite(theUMin);
        bool isU2Inf = Precision.IsInfinite(theUMax);
        bool isV1Inf = Precision.IsInfinite(theVMin);
        bool isV2Inf = Precision.IsInfinite(theVMax);

        if (isU1Inf && isU2Inf)
            aU = 0;
        else if (isU1Inf)
            aU = theUMax - 10.0;
        else if (isU2Inf)
            aU = theUMin + 10.0;
        else
            aU = (theUMin + theUMax) / 2.0;

        if (isV1Inf && isV2Inf)
            aV = 0;
        else if (isV1Inf)
            aV = theVMax - 10.0;
        else if (isV2Inf)
            aV = theVMin + 10.0;
        else
            aV = (theVMin + theVMax) / 2.0;

        return ElSLib.Value(aU, aV, thePlane);
    }

    /// <summary>Handle infinite plane by opening appropriate box sides.</summary>
    private static void TreatInfinitePlane(in gp_Pln thePlane,
                                           double theUMin,
                                           double theUMax,
                                           double theVMin,
                                           double theVMax,
                                           double theTol,
                                           ref Bnd_Box theBox)
    {
        gp_Dir aNorm = thePlane.Axis().Direction();
        double anAngularTol = RealEpsilon();

        gp_Pnt aLocation = BaryCenter(thePlane, theUMin, theUMax, theVMin, theVMax);

        if (aNorm.IsParallel(gp.DX(), anAngularTol))
        {
            theBox.Add(aLocation);
            theBox.OpenYmin();
            theBox.OpenYmax();
            theBox.OpenZmin();
            theBox.OpenZmax();
        }
        else if (aNorm.IsParallel(gp.DY(), anAngularTol))
        {
            theBox.Add(aLocation);
            theBox.OpenXmin();
            theBox.OpenXmax();
            theBox.OpenZmin();
            theBox.OpenZmax();
        }
        else if (aNorm.IsParallel(gp.DZ(), anAngularTol))
        {
            theBox.Add(aLocation);
            theBox.OpenXmin();
            theBox.OpenXmax();
            theBox.OpenYmin();
            theBox.OpenYmax();
        }
        else
        {
            theBox.SetWhole();
            return;
        }

        theBox.Enlarge(theTol);
    }
}
