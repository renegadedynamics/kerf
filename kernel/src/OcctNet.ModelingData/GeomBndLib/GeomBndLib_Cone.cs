// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Cone.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Cone.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Computes bounding box for a conical surface (Geom_ConicalSurface) from the boundary circle arcs, opened along the
/// axis for infinite V.
/// </summary>
public sealed class GeomBndLib_Cone : GeomBndLib_Surface.ISurfaceEvaluator
{
    private readonly Geom_ConicalSurface myGeom;

    public GeomBndLib_Cone(Geom_ConicalSurface theSurf)
    {
        myGeom = theSurf;
    }

    public Geom_ConicalSurface Geometry() => myGeom;

    /// <summary>For analytical surfaces, BoxOptimal is same as Box.</summary>
    public Bnd_Box BoxOptimal(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
        => Box(theUMin, theUMax, theVMin, theVMax, theTol);

    /// <summary>Compute optimal bounding box for full surface.</summary>
    public Bnd_Box BoxOptimal(double theTol) => Box(theTol);

    /// <summary>Compute bounding box for a finite cone patch.</summary>
    private static void computeCone(in gp_Cone theCone,
                                    double theUMin,
                                    double theUMax,
                                    double theVMin,
                                    double theVMax,
                                    ref Bnd_Box theBox)
    {
        gp_Ax3 aPos = theCone.Position();
        double aR = theCone.RefRadius();
        double aSAng = theCone.SemiAngle();

        // Add circle at VMin.
        gp_Circ aC = ElSLib.ConeVIso(aPos, aR, aSAng, theVMin);
        if (aC.Radius() > Precision.Confusion())
        {
            theBox.Add(GeomBndLib_Circle.Box(aC, theUMin, theUMax, 0.0));
        }
        else
        {
            theBox.Add(aC.Location());
        }

        // Add circle at VMax.
        aC = ElSLib.ConeVIso(aPos, aR, aSAng, theVMax);
        if (aC.Radius() > Precision.Confusion())
        {
            theBox.Add(GeomBndLib_Circle.Box(aC, theUMin, theUMax, 0.0));
        }
        else
        {
            theBox.Add(aC.Location());
        }
    }

    /// <summary>Compute bounding box for full cone.</summary>
    public Bnd_Box Box(double theTol)
    {
        myGeom.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        return Box(aU1, aU2, aV1, aV2, theTol);
    }

    /// <summary>Compute bounding box for cone patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Bnd_Box Box(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
    {
        Bnd_Box aBox = new();
        gp_Cone aCone = myGeom.Cone();
        gp_Dir aDir = aCone.Axis().Direction();

        // OCCT opens the box only along the axis for an infinite V: the cone widens away from its apex, so the
        // radial directions of an axis-aligned cone stay closed and the box does not contain the surface
        // (occt-issues.md).
        if (Precision.IsNegativeInfinite(theVMin))
        {
            if (Precision.IsNegativeInfinite(theVMax))
            {
                throw new Standard_Failure("GeomBndLib_Cone::Box - bad parameter");
            }
            else if (Precision.IsPositiveInfinite(theVMax))
            {
                // Fully infinite V: add the circle at V=0 to give the box finite radial extent.
                gp_Circ aC = ElSLib.ConeVIso(aCone.Position(), aCone.RefRadius(), aCone.SemiAngle(), 0.0);
                if (aC.Radius() > Precision.Confusion())
                {
                    aBox.Add(GeomBndLib_Circle.Box(aC, theUMin, theUMax, 0.0));
                }
                else
                {
                    aBox.Add(aC.Location());
                }
                GeomBndLib_InfiniteHelpers.OpenMinMax(aDir, ref aBox);
            }
            else
            {
                computeCone(aCone, theUMin, theUMax, 0.0, theVMax, ref aBox);
                GeomBndLib_InfiniteHelpers.OpenMin(aDir, ref aBox);
            }
        }
        else if (Precision.IsPositiveInfinite(theVMin))
        {
            if (Precision.IsNegativeInfinite(theVMax))
            {
                // Reversed infinite V: add the circle at V=0 to give the box finite radial extent.
                gp_Circ aC = ElSLib.ConeVIso(aCone.Position(), aCone.RefRadius(), aCone.SemiAngle(), 0.0);
                if (aC.Radius() > Precision.Confusion())
                {
                    aBox.Add(GeomBndLib_Circle.Box(aC, theUMin, theUMax, 0.0));
                }
                else
                {
                    aBox.Add(aC.Location());
                }
                GeomBndLib_InfiniteHelpers.OpenMinMax(aDir, ref aBox);
            }
            else if (Precision.IsPositiveInfinite(theVMax))
            {
                throw new Standard_Failure("GeomBndLib_Cone::Box - bad parameter");
            }
            else
            {
                computeCone(aCone, theUMin, theUMax, 0.0, theVMax, ref aBox);
                GeomBndLib_InfiniteHelpers.OpenMax(aDir, ref aBox);
            }
        }
        else
        {
            if (Precision.IsNegativeInfinite(theVMax))
            {
                computeCone(aCone, theUMin, theUMax, theVMin, 0.0, ref aBox);
                GeomBndLib_InfiniteHelpers.OpenMin(aDir, ref aBox);
            }
            else if (Precision.IsPositiveInfinite(theVMax))
            {
                computeCone(aCone, theUMin, theUMax, theVMin, 0.0, ref aBox);
                GeomBndLib_InfiniteHelpers.OpenMax(aDir, ref aBox);
            }
            else
            {
                computeCone(aCone, theUMin, theUMax, theVMin, theVMax, ref aBox);
            }
        }

        aBox.Enlarge(theTol);
        return aBox;
    }
}
