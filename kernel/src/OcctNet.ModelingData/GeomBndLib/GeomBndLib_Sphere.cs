// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Sphere.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Sphere.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Computes bounding box for a spherical surface (Geom_SphericalSurface): the six axis-aligned extremal points that
/// lie in the patch, and the boundary iso-circles.
/// </summary>
public sealed class GeomBndLib_Sphere : GeomBndLib_Surface.ISurfaceEvaluator
{
    private readonly Geom_SphericalSurface myGeom;

    public GeomBndLib_Sphere(Geom_SphericalSurface theSurf)
    {
        myGeom = theSurf;
    }

    public Geom_SphericalSurface Geometry() => myGeom;

    /// <summary>For analytical surfaces, BoxOptimal is same as Box.</summary>
    public Box BoxOptimal(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
        => Box(theUMin, theUMax, theVMin, theVMax, theTol);

    /// <summary>Compute optimal bounding box for full surface.</summary>
    public Box BoxOptimal(double theTol) => Box(theTol);

    /// <summary>Compute bounding box for full sphere.</summary>
    public Box Box(double theTol)
    {
        Box aBox = new();
        Sphere aSphere = myGeom.Sphere();
        Point3D aP = aSphere.Location();
        double aR = aSphere.Radius();
        aBox.Update(aP.X() - aR, aP.Y() - aR, aP.Z() - aR, aP.X() + aR, aP.Y() + aR, aP.Z() + aR);
        aBox.Enlarge(theTol);
        return aBox;
    }

    /// <summary>Compute bounding box for a sphere patch.</summary>
    private static void computeSphere(in Sphere theSphere,
                                      double theUMin,
                                      double theUMax,
                                      double theVMin,
                                      double theVMax,
                                      ref Box theBox)
    {
        Point3D aP = theSphere.Location();
        double aR = theSphere.Radius();

        double aXmin = aP.X() - aR;
        double aXmax = aP.X() + aR;
        double aYmin = aP.Y() - aR;
        double aYmax = aP.Y() + aR;
        double aZmin = aP.Z() - aR;
        double aZmax = aP.Z() + aR;

        double anUper = 2.0 * M_PI - global::Kerf.Engine.Foundation.Precision.PConfusion();
        double aVper = M_PI - global::Kerf.Engine.Foundation.Precision.PConfusion();
        if (theUMax - theUMin >= anUper && theVMax - theVMin >= aVper)
        {
            // Whole sphere.
            theBox.Update(aXmin, aYmin, aZmin, aXmax, aYmax, aZmax);
            return;
        }

        // Check 6 extremal points (axis-aligned extrema of the full sphere).
        double anU, aV;
        double anUmax = theUMin + 2.0 * M_PI;
        CoordinateSystem aPos = theSphere.Position();
        Point3D aPExt = aP;

        // -X extremum
        aPExt.SetX(aXmin);
        global::Kerf.Engine.Geometry.ElSLib.SphereParameters(aPos, aR, aPExt, out anU, out aV);
        anU = global::Kerf.Engine.Geometry.ElCLib.InPeriod(anU, theUMin, anUmax);
        if (anU >= theUMin && anU <= theUMax && aV >= theVMin && aV <= theVMax)
        {
            theBox.Add(aPExt);
        }
        // +X extremum
        aPExt.SetX(aXmax);
        global::Kerf.Engine.Geometry.ElSLib.SphereParameters(aPos, aR, aPExt, out anU, out aV);
        anU = global::Kerf.Engine.Geometry.ElCLib.InPeriod(anU, theUMin, anUmax);
        if (anU >= theUMin && anU <= theUMax && aV >= theVMin && aV <= theVMax)
        {
            theBox.Add(aPExt);
        }
        aPExt.SetX(aP.X());
        // -Y extremum
        aPExt.SetY(aYmin);
        global::Kerf.Engine.Geometry.ElSLib.SphereParameters(aPos, aR, aPExt, out anU, out aV);
        anU = global::Kerf.Engine.Geometry.ElCLib.InPeriod(anU, theUMin, anUmax);
        if (anU >= theUMin && anU <= theUMax && aV >= theVMin && aV <= theVMax)
        {
            theBox.Add(aPExt);
        }
        // +Y extremum
        aPExt.SetY(aYmax);
        global::Kerf.Engine.Geometry.ElSLib.SphereParameters(aPos, aR, aPExt, out anU, out aV);
        anU = global::Kerf.Engine.Geometry.ElCLib.InPeriod(anU, theUMin, anUmax);
        if (anU >= theUMin && anU <= theUMax && aV >= theVMin && aV <= theVMax)
        {
            theBox.Add(aPExt);
        }
        aPExt.SetY(aP.Y());
        // -Z extremum
        aPExt.SetZ(aZmin);
        global::Kerf.Engine.Geometry.ElSLib.SphereParameters(aPos, aR, aPExt, out anU, out aV);
        anU = global::Kerf.Engine.Geometry.ElCLib.InPeriod(anU, theUMin, anUmax);
        if (anU >= theUMin && anU <= theUMax && aV >= theVMin && aV <= theVMax)
        {
            theBox.Add(aPExt);
        }
        // +Z extremum
        aPExt.SetZ(aZmax);
        global::Kerf.Engine.Geometry.ElSLib.SphereParameters(aPos, aR, aPExt, out anU, out aV);
        anU = global::Kerf.Engine.Geometry.ElCLib.InPeriod(anU, theUMin, anUmax);
        if (anU >= theUMin && anU <= theUMax && aV >= theVMin && aV <= theVMax)
        {
            theBox.Add(aPExt);
        }

        // Add boundary iso-curves of the patch.
        Circle3D aC = global::Kerf.Engine.Geometry.ElSLib.SphereUIso(aPos, aR, theUMin);
        theBox.Add(GeomBndLib_Circle.Box(aC, theVMin, theVMax, 0.0));
        aC = global::Kerf.Engine.Geometry.ElSLib.SphereUIso(aPos, aR, theUMax);
        theBox.Add(GeomBndLib_Circle.Box(aC, theVMin, theVMax, 0.0));
        aC = global::Kerf.Engine.Geometry.ElSLib.SphereVIso(aPos, aR, theVMin);
        theBox.Add(GeomBndLib_Circle.Box(aC, theUMin, theUMax, 0.0));
        aC = global::Kerf.Engine.Geometry.ElSLib.SphereVIso(aPos, aR, theVMax);
        theBox.Add(GeomBndLib_Circle.Box(aC, theUMin, theUMax, 0.0));
    }

    /// <summary>Compute bounding box for sphere patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Box Box(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
    {
        Sphere aSphere = myGeom.Sphere();

        // Check if it is a whole sphere.
        if (Math.Abs(theUMin) < global::Kerf.Engine.Foundation.Precision.Angular()
            && Math.Abs(theUMax - 2.0 * M_PI) < global::Kerf.Engine.Foundation.Precision.Angular()
            && Math.Abs(theVMin + M_PI / 2.0) < global::Kerf.Engine.Foundation.Precision.Angular()
            && Math.Abs(theVMax - M_PI / 2.0) < global::Kerf.Engine.Foundation.Precision.Angular())
        {
            return Box(theTol);
        }

        Box aBox = new();
        computeSphere(aSphere, theUMin, theUMax, theVMin, theVMax, ref aBox);
        aBox.Enlarge(theTol);
        return aBox;
    }
}
