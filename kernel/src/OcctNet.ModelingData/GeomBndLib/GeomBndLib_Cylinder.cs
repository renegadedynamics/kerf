// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Cylinder.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Cylinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Computes bounding box for a cylindrical surface (Geom_CylindricalSurface) from the boundary circle arcs, opened
/// along the axis for infinite V.
/// </summary>
public sealed class GeomBndLib_Cylinder : GeomBndLib_Surface.ISurfaceEvaluator
{
    private readonly Geom_CylindricalSurface myGeom;

    public GeomBndLib_Cylinder(Geom_CylindricalSurface theSurf)
    {
        myGeom = theSurf;
    }

    public Geom_CylindricalSurface Geometry() => myGeom;

    /// <summary>Compute bounding box for full cylinder.</summary>
    public Bnd_Box Box(double theTol)
    {
        myGeom.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        return Box(aU1, aU2, aV1, aV2, theTol);
    }

    /// <summary>For analytical surfaces, BoxOptimal is same as Box.</summary>
    public Bnd_Box BoxOptimal(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
        => Box(theUMin, theUMax, theVMin, theVMax, theTol);

    /// <summary>Compute optimal bounding box for full surface.</summary>
    public Bnd_Box BoxOptimal(double theTol) => Box(theTol);

    /// <summary>Compute bounding box for a finite cylinder patch.</summary>
    private static void computeCylinder(in gp_Cylinder theCyl,
                                        double theUMin,
                                        double theUMax,
                                        double theVMin,
                                        double theVMax,
                                        ref Bnd_Box theBox)
    {
        // Add circle at VMin.
        gp_Circ aC = ElSLib.CylinderVIso(theCyl.Position(), theCyl.Radius(), theVMin);
        theBox.Add(GeomBndLib_Circle.Box(aC, theUMin, theUMax, 0.0));

        // Add circle at VMax by translating.
        gp_Vec aT = (theVMax - theVMin) * new gp_Vec(theCyl.Axis().Direction());
        aC.Translate(aT);
        theBox.Add(GeomBndLib_Circle.Box(aC, theUMin, theUMax, 0.0));
    }

    /// <summary>Compute bounding box for cylinder patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Bnd_Box Box(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
    {
        Bnd_Box aBox = new();
        gp_Cylinder aCyl = myGeom.Cylinder();
        gp_Dir aDir = aCyl.Axis().Direction();

        if (Precision.IsNegativeInfinite(theVMin))
        {
            if (Precision.IsNegativeInfinite(theVMax))
            {
                throw new Standard_Failure("GeomBndLib_Cylinder::Box - bad parameter");
            }
            else if (Precision.IsPositiveInfinite(theVMax))
            {
                // Fully infinite V: add the arc at V=0 to give the box finite radial extent.
                gp_Circ aC = ElSLib.CylinderVIso(aCyl.Position(), aCyl.Radius(), 0.0);
                aBox.Add(GeomBndLib_Circle.Box(aC, theUMin, theUMax, 0.0));
                GeomBndLib_InfiniteHelpers.OpenMinMax(aDir, ref aBox);
            }
            else
            {
                computeCylinder(aCyl, theUMin, theUMax, 0.0, theVMax, ref aBox);
                GeomBndLib_InfiniteHelpers.OpenMin(aDir, ref aBox);
            }
        }
        else if (Precision.IsPositiveInfinite(theVMin))
        {
            if (Precision.IsNegativeInfinite(theVMax))
            {
                // Reversed infinite V: add the arc at V=0 to give the box finite radial extent.
                gp_Circ aC = ElSLib.CylinderVIso(aCyl.Position(), aCyl.Radius(), 0.0);
                aBox.Add(GeomBndLib_Circle.Box(aC, theUMin, theUMax, 0.0));
                GeomBndLib_InfiniteHelpers.OpenMinMax(aDir, ref aBox);
            }
            else if (Precision.IsPositiveInfinite(theVMax))
            {
                throw new Standard_Failure("GeomBndLib_Cylinder::Box - bad parameter");
            }
            else
            {
                computeCylinder(aCyl, theUMin, theUMax, 0.0, theVMax, ref aBox);
                GeomBndLib_InfiniteHelpers.OpenMax(aDir, ref aBox);
            }
        }
        else
        {
            if (Precision.IsNegativeInfinite(theVMax))
            {
                computeCylinder(aCyl, theUMin, theUMax, theVMin, 0.0, ref aBox);
                GeomBndLib_InfiniteHelpers.OpenMin(aDir, ref aBox);
            }
            else if (Precision.IsPositiveInfinite(theVMax))
            {
                computeCylinder(aCyl, theUMin, theUMax, theVMin, 0.0, ref aBox);
                GeomBndLib_InfiniteHelpers.OpenMax(aDir, ref aBox);
            }
            else
            {
                computeCylinder(aCyl, theUMin, theUMax, theVMin, theVMax, ref aBox);
            }
        }

        aBox.Enlarge(theTol);
        return aBox;
    }
}
