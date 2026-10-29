// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Sphere.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Sphere.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

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
    public Bnd_Box BoxOptimal(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
        => Box(theUMin, theUMax, theVMin, theVMax, theTol);

    /// <summary>Compute optimal bounding box for full surface.</summary>
    public Bnd_Box BoxOptimal(double theTol) => Box(theTol);

    /// <summary>Compute bounding box for full sphere.</summary>
    public Bnd_Box Box(double theTol)
    {
        Bnd_Box aBox = new();
        gp_Sphere aSphere = myGeom.Sphere();
        gp_Pnt aP = aSphere.Location();
        double aR = aSphere.Radius();
        aBox.Update(aP.X() - aR, aP.Y() - aR, aP.Z() - aR, aP.X() + aR, aP.Y() + aR, aP.Z() + aR);
        aBox.Enlarge(theTol);
        return aBox;
    }

    /// <summary>Compute bounding box for a sphere patch.</summary>
    private static void computeSphere(in gp_Sphere theSphere,
                                      double theUMin,
                                      double theUMax,
                                      double theVMin,
                                      double theVMax,
                                      ref Bnd_Box theBox)
    {
        gp_Pnt aP = theSphere.Location();
        double aR = theSphere.Radius();

        double aXmin = aP.X() - aR;
        double aXmax = aP.X() + aR;
        double aYmin = aP.Y() - aR;
        double aYmax = aP.Y() + aR;
        double aZmin = aP.Z() - aR;
        double aZmax = aP.Z() + aR;

        double anUper = 2.0 * M_PI - Precision.PConfusion();
        double aVper = M_PI - Precision.PConfusion();
        if (theUMax - theUMin >= anUper && theVMax - theVMin >= aVper)
        {
            // Whole sphere.
            theBox.Update(aXmin, aYmin, aZmin, aXmax, aYmax, aZmax);
            return;
        }

        // Check 6 extremal points (axis-aligned extrema of the full sphere).
        double anU, aV;
        double anUmax = theUMin + 2.0 * M_PI;
        gp_Ax3 aPos = theSphere.Position();
        gp_Pnt aPExt = aP;

        // -X extremum
        aPExt.SetX(aXmin);
        ElSLib.SphereParameters(aPos, aR, aPExt, out anU, out aV);
        anU = ElCLib.InPeriod(anU, theUMin, anUmax);
        if (anU >= theUMin && anU <= theUMax && aV >= theVMin && aV <= theVMax)
        {
            theBox.Add(aPExt);
        }
        // +X extremum
        aPExt.SetX(aXmax);
        ElSLib.SphereParameters(aPos, aR, aPExt, out anU, out aV);
        anU = ElCLib.InPeriod(anU, theUMin, anUmax);
        if (anU >= theUMin && anU <= theUMax && aV >= theVMin && aV <= theVMax)
        {
            theBox.Add(aPExt);
        }
        aPExt.SetX(aP.X());
        // -Y extremum
        aPExt.SetY(aYmin);
        ElSLib.SphereParameters(aPos, aR, aPExt, out anU, out aV);
        anU = ElCLib.InPeriod(anU, theUMin, anUmax);
        if (anU >= theUMin && anU <= theUMax && aV >= theVMin && aV <= theVMax)
        {
            theBox.Add(aPExt);
        }
        // +Y extremum
        aPExt.SetY(aYmax);
        ElSLib.SphereParameters(aPos, aR, aPExt, out anU, out aV);
        anU = ElCLib.InPeriod(anU, theUMin, anUmax);
        if (anU >= theUMin && anU <= theUMax && aV >= theVMin && aV <= theVMax)
        {
            theBox.Add(aPExt);
        }
        aPExt.SetY(aP.Y());
        // -Z extremum
        aPExt.SetZ(aZmin);
        ElSLib.SphereParameters(aPos, aR, aPExt, out anU, out aV);
        anU = ElCLib.InPeriod(anU, theUMin, anUmax);
        if (anU >= theUMin && anU <= theUMax && aV >= theVMin && aV <= theVMax)
        {
            theBox.Add(aPExt);
        }
        // +Z extremum
        aPExt.SetZ(aZmax);
        ElSLib.SphereParameters(aPos, aR, aPExt, out anU, out aV);
        anU = ElCLib.InPeriod(anU, theUMin, anUmax);
        if (anU >= theUMin && anU <= theUMax && aV >= theVMin && aV <= theVMax)
        {
            theBox.Add(aPExt);
        }

        // Add boundary iso-curves of the patch.
        gp_Circ aC = ElSLib.SphereUIso(aPos, aR, theUMin);
        theBox.Add(GeomBndLib_Circle.Box(aC, theVMin, theVMax, 0.0));
        aC = ElSLib.SphereUIso(aPos, aR, theUMax);
        theBox.Add(GeomBndLib_Circle.Box(aC, theVMin, theVMax, 0.0));
        aC = ElSLib.SphereVIso(aPos, aR, theVMin);
        theBox.Add(GeomBndLib_Circle.Box(aC, theUMin, theUMax, 0.0));
        aC = ElSLib.SphereVIso(aPos, aR, theVMax);
        theBox.Add(GeomBndLib_Circle.Box(aC, theUMin, theUMax, 0.0));
    }

    /// <summary>Compute bounding box for sphere patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Bnd_Box Box(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
    {
        gp_Sphere aSphere = myGeom.Sphere();

        // Check if it is a whole sphere.
        if (Math.Abs(theUMin) < Precision.Angular()
            && Math.Abs(theUMax - 2.0 * M_PI) < Precision.Angular()
            && Math.Abs(theVMin + M_PI / 2.0) < Precision.Angular()
            && Math.Abs(theVMax - M_PI / 2.0) < Precision.Angular())
        {
            return Box(theTol);
        }

        Bnd_Box aBox = new();
        computeSphere(aSphere, theUMin, theUMax, theVMin, theVMax, ref aBox);
        aBox.Enlarge(theTol);
        return aBox;
    }
}
