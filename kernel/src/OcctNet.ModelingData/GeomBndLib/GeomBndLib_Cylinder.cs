// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Cylinder.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Cylinder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

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
    public Box Box(double theTol)
    {
        myGeom.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        return Box(aU1, aU2, aV1, aV2, theTol);
    }

    /// <summary>For analytical surfaces, BoxOptimal is same as Box.</summary>
    public Box BoxOptimal(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
        => Box(theUMin, theUMax, theVMin, theVMax, theTol);

    /// <summary>Compute optimal bounding box for full surface.</summary>
    public Box BoxOptimal(double theTol) => Box(theTol);

    /// <summary>Compute bounding box for a finite cylinder patch.</summary>
    private static void computeCylinder(in Cylinder theCyl,
                                        double theUMin,
                                        double theUMax,
                                        double theVMin,
                                        double theVMax,
                                        ref Box theBox)
    {
        // Add circle at VMin.
        Circle3D aC = global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(theCyl.Position(), theCyl.Radius(), theVMin);
        theBox.Add(GeomBndLib_Circle.Box(aC, theUMin, theUMax, 0.0));

        // Add circle at VMax by translating.
        Vector3D aT = (theVMax - theVMin) * new Vector3D(theCyl.Axis().Direction());
        aC.Translate(aT);
        theBox.Add(GeomBndLib_Circle.Box(aC, theUMin, theUMax, 0.0));
    }

    /// <summary>Compute bounding box for cylinder patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Box Box(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
    {
        Box aBox = new();
        Cylinder aCyl = myGeom.Cylinder();
        Direction3D aDir = aCyl.Axis().Direction();

        if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theVMin))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theVMax))
            {
                throw new FailureException("GeomBndLib_Cylinder::Box - bad parameter");
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theVMax))
            {
                // Fully infinite V: add the arc at V=0 to give the box finite radial extent.
                Circle3D aC = global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(aCyl.Position(), aCyl.Radius(), 0.0);
                aBox.Add(GeomBndLib_Circle.Box(aC, theUMin, theUMax, 0.0));
                GeomBndLib_InfiniteHelpers.OpenMinMax(aDir, ref aBox);
            }
            else
            {
                computeCylinder(aCyl, theUMin, theUMax, 0.0, theVMax, ref aBox);
                GeomBndLib_InfiniteHelpers.OpenMin(aDir, ref aBox);
            }
        }
        else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theVMin))
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theVMax))
            {
                // Reversed infinite V: add the arc at V=0 to give the box finite radial extent.
                Circle3D aC = global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(aCyl.Position(), aCyl.Radius(), 0.0);
                aBox.Add(GeomBndLib_Circle.Box(aC, theUMin, theUMax, 0.0));
                GeomBndLib_InfiniteHelpers.OpenMinMax(aDir, ref aBox);
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theVMax))
            {
                throw new FailureException("GeomBndLib_Cylinder::Box - bad parameter");
            }
            else
            {
                computeCylinder(aCyl, theUMin, theUMax, 0.0, theVMax, ref aBox);
                GeomBndLib_InfiniteHelpers.OpenMax(aDir, ref aBox);
            }
        }
        else
        {
            if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theVMax))
            {
                computeCylinder(aCyl, theUMin, theUMax, theVMin, 0.0, ref aBox);
                GeomBndLib_InfiniteHelpers.OpenMin(aDir, ref aBox);
            }
            else if (global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theVMax))
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
