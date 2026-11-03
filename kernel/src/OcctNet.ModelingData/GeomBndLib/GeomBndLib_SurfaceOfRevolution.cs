// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_SurfaceOfRevolution.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_SurfaceOfRevolution.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Computes bounding box for a surface of revolution (Geom_SurfaceOfRevolution): the basis curve's box is computed,
/// and the eight corners of that box are revolved about the axis, each circle arc bounded by GeomBndLib_Circle.
/// </summary>
public sealed class GeomBndLib_SurfaceOfRevolution : GeomBndLib_Surface.ISurfaceEvaluator
{
    private readonly Geom_SurfaceOfRevolution myGeom;

    public GeomBndLib_SurfaceOfRevolution(Geom_SurfaceOfRevolution theSurf)
    {
        myGeom = theSurf;
    }

    public Geom_SurfaceOfRevolution Geometry() => myGeom;

    /// <summary>Add the bounding box contribution of a single revolution circle for a point revolved around the axis.</summary>
    private static void addRevolutionCircle(in Point3D theOrigin,
                                            in Direction3D theAxisDir,
                                            in Point3D theBasisPt,
                                            double theUMin,
                                            double theUMax,
                                            ref Box theBox)
    {
        // Project the basis curve point onto the revolution axis.
        Xyz aDelta = theBasisPt.XYZ() - theOrigin.XYZ();
        double aH = aDelta.Dot(theAxisDir.XYZ());
        Xyz aCenter = theOrigin.XYZ() + aH * theAxisDir.XYZ();

        // Radial vector from axis to basis point.
        Xyz aRadial = theBasisPt.XYZ() - aCenter;
        double aRadius = aRadial.Modulus();
        if (aRadius < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            // Point is on the axis - revolution is a single point.
            theBox.Add(theBasisPt);
            return;
        }

        // Construct the revolution circle.
        Direction3D aXDir = new(aRadial);
        Frame3D anAx2 = new(new Point3D(aCenter), theAxisDir, aXDir);
        Circle3D aCirc = new(anAx2, aRadius);
        theBox.Add(GeomBndLib_Circle.Box(aCirc, theUMin, theUMax, 0.0));
    }

    /// <summary>Revolves the eight corners of a box (OCCT's two identical triple loops).</summary>
    private static void addRevolvedCorners(double aXmin, double aXmax, double aYmin, double aYmax, double aZmin, double aZmax,
                                           in Point3D theOrigin, in Direction3D theAxisDir, double theUMin, double theUMax,
                                           ref Box aBox)
    {
        ReadOnlySpan<double> aXVals = [aXmin, aXmax];
        ReadOnlySpan<double> aYVals = [aYmin, aYmax];
        ReadOnlySpan<double> aZVals = [aZmin, aZmax];
        for (int ix = 0; ix < 2; ++ix)
        {
            for (int iy = 0; iy < 2; ++iy)
            {
                for (int iz = 0; iz < 2; ++iz)
                {
                    addRevolutionCircle(theOrigin,
                                        theAxisDir,
                                        new Point3D(aXVals[ix], aYVals[iy], aZVals[iz]),
                                        theUMin,
                                        theUMax,
                                        ref aBox);
                }
            }
        }
    }

    /// <summary>Build the revolution box from a pre-computed basis curve box.</summary>
    private static Box buildRevolutionBox(in Box theCurveBox,
                                              in Point3D theOrigin,
                                              in Direction3D theAxisDir,
                                              double theUMin,
                                              double theUMax,
                                              double theTol)
    {
        Box aBox = new();
        if (theCurveBox.IsVoid)
        {
            aBox.Enlarge(theTol);
            return aBox;
        }

        if (theCurveBox.IsOpen)
        {
            // Revolve the finite part corners for the bounded portion.
            if (theCurveBox.HasFinitePart)
            {
                (double aXmin, double aXmax, double aYmin, double aYmax, double aZmin, double aZmax) = theCurveBox.FinitePart().Get();
                addRevolvedCorners(aXmin, aXmax, aYmin, aYmax, aZmin, aZmax, theOrigin, theAxisDir, theUMin, theUMax, ref aBox);
            }

            // Propagate curve openness: open directions remain open after revolution.
            if (theCurveBox.IsOpenXmin)
            {
                aBox.OpenXmin();
            }
            if (theCurveBox.IsOpenXmax)
            {
                aBox.OpenXmax();
            }
            if (theCurveBox.IsOpenYmin)
            {
                aBox.OpenYmin();
            }
            if (theCurveBox.IsOpenYmax)
            {
                aBox.OpenYmax();
            }
            if (theCurveBox.IsOpenZmin)
            {
                aBox.OpenZmin();
            }
            if (theCurveBox.IsOpenZmax)
            {
                aBox.OpenZmax();
            }
            aBox.Enlarge(theTol);
            return aBox;
        }

        // Get() returns {Xmin, Xmax, Ymin, Ymax, Zmin, Zmax}.
        (double bXmin, double bXmax, double bYmin, double bYmax, double bZmin, double bZmax) = theCurveBox.Get();

        // Revolve all 8 corners of the basis curve bounding box.
        addRevolvedCorners(bXmin, bXmax, bYmin, bYmax, bZmin, bZmax, theOrigin, theAxisDir, theUMin, theUMax, ref aBox);

        aBox.Enlarge(theTol);
        return aBox;
    }

    /// <summary>Clamps the V range to the basis curve's bounds, as both of OCCT's methods do inline.</summary>
    private static void clampVRange(Geom_Curve aBasisCurve, double theVMin, double theVMax, out double aVMin, out double aVMax)
    {
        double aVFirst = aBasisCurve.FirstParameter();
        double aVLast = aBasisCurve.LastParameter();
        aVMin = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theVMin) ? aVFirst : theVMin;
        aVMax = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theVMax) ? aVLast : theVMax;
        if (!aBasisCurve.IsPeriodic())
        {
            if (aVMin < aVFirst)
            {
                aVMin = aVFirst;
            }
            else if (aVMin > aVLast)
            {
                aVMin = aVLast;
            }
            if (aVMax < aVFirst)
            {
                aVMax = aVFirst;
            }
            else if (aVMax > aVLast)
            {
                aVMax = aVLast;
            }
            if (aVMin > aVMax)
            {
                double aTmp = aVMin;
                aVMin = aVMax;
                aVMax = aTmp;
            }
        }
    }

    /// <summary>Compute bounding box for full surface.</summary>
    public Box Box(double theTol)
    {
        myGeom.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        return Box(aU1, aU2, aV1, aV2, theTol);
    }

    /// <summary>Compute bounding box for surface patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Box Box(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
    {
        try
        {
            Geom_Curve aBasisCurve = myGeom.BasisCurve();
            Axis3D anAxis = myGeom.Axis();
            Point3D anOrigin = anAxis.Location();
            Direction3D anAxisDir = anAxis.Direction();

            // Clamp V range to basis curve bounds.
            clampVRange(aBasisCurve, theVMin, theVMax, out double aVMin, out double aVMax);

            // Compute the basis curve bounding box, then revolve all 8 corners around the axis.
            GeomBndLib_Curve aCurveEval = new(aBasisCurve);
            Box aCurveBox = aCurveEval.Box(aVMin, aVMax, 0.0);
            if (aCurveBox.IsVoid)
            {
                GeomAdaptor_Surface anAdaptor0 = new(myGeom);
                GeomBndLib_OtherSurface anOther0 = new(anAdaptor0);
                return anOther0.Box(theUMin, theUMax, theVMin, theVMax, theTol);
            }

            return buildRevolutionBox(aCurveBox, anOrigin, anAxisDir, theUMin, theUMax, theTol);
        }
        catch (FailureException)
        {
            // Fall back to robust generic sampling if basis-curve segmentation fails.
            GeomAdaptor_Surface anAdaptor = new(myGeom);
            GeomBndLib_OtherSurface anOther = new(anAdaptor);
            return anOther.Box(theUMin, theUMax, theVMin, theVMax, theTol);
        }
    }

    /// <summary>Compute precise bounding box for full surface.</summary>
    public Box BoxOptimal(double theTol)
    {
        myGeom.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        return BoxOptimal(aU1, aU2, aV1, aV2, theTol);
    }

    /// <summary>Compute precise bounding box using tight basis curve bounds.</summary>
    public Box BoxOptimal(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
    {
        try
        {
            Geom_Curve aBasisCurve = myGeom.BasisCurve();
            Axis3D anAxis = myGeom.Axis();
            Point3D anOrigin = anAxis.Location();
            Direction3D anAxisDir = anAxis.Direction();

            // Clamp V range to basis curve bounds.
            clampVRange(aBasisCurve, theVMin, theVMax, out double aVMin, out double aVMax);

            // Use the tight basis curve box for a more precise result.
            GeomBndLib_Curve aCurveEval = new(aBasisCurve);
            Box aCurveBox = aCurveEval.BoxOptimal(aVMin, aVMax, 0.0);
            if (aCurveBox.IsVoid)
            {
                GeomAdaptor_Surface anAdaptor0 = new(myGeom);
                GeomBndLib_OtherSurface anOther0 = new(anAdaptor0);
                return anOther0.BoxOptimal(theUMin, theUMax, theVMin, theVMax, theTol);
            }

            return buildRevolutionBox(aCurveBox, anOrigin, anAxisDir, theUMin, theUMax, theTol);
        }
        catch (FailureException)
        {
            // Fall back to robust generic sampling if basis-curve segmentation fails.
            GeomAdaptor_Surface anAdaptor = new(myGeom);
            GeomBndLib_OtherSurface anOther = new(anAdaptor);
            return anOther.BoxOptimal(theUMin, theUMax, theVMin, theVMax, theTol);
        }
    }
}
