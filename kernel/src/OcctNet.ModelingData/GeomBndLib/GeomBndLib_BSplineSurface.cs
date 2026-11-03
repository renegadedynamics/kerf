// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_BSplineSurface.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_BSplineSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Computes bounding box for a B-spline surface (Geom_BSplineSurface): the convex hull of the poles that influence
/// the patch (selected through the knots), grid sampling outside the surface's bounds, sampling + PSO/Powell
/// optimization for <c>BoxOptimal</c>.
/// </summary>
public sealed class GeomBndLib_BSplineSurface : GeomBndLib_Surface.ISurfaceEvaluator
{
    private readonly Geom_BSplineSurface myGeom;

    public GeomBndLib_BSplineSurface(Geom_BSplineSurface theSurf)
    {
        myGeom = theSurf;
    }

    public Geom_BSplineSurface Geometry() => myGeom;

    /// <summary>
    /// Compute start and finish pole indexes for a given parameter range using knot-based index selection via
    /// BSplCLib::Hunt and BSplCLib::PoleIndex.
    /// </summary>
    private static void ComputePolesIndexes(NCollection_Array1<double> theKnots,
                                            NCollection_Array1<int> theMults,
                                            int theDegree,
                                            double theMin,
                                            double theMax,
                                            int theMaxPoleIdx,
                                            bool theIsPeriodic,
                                            ref int theOutMinIdx,
                                            ref int theOutMaxIdx)
    {
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(theKnots, theMin, out theOutMinIdx);
        theOutMinIdx = Math.Clamp(theOutMinIdx, theKnots.Lower(), theKnots.Upper());

        global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(theKnots, theMax, out theOutMaxIdx);
        theOutMaxIdx++;
        theOutMaxIdx = Math.Clamp(theOutMaxIdx, theKnots.Lower(), theKnots.Upper());
        int aMultiplier = theMults[theOutMaxIdx];

        theOutMinIdx = global::Kerf.Engine.Geometry.Splines.BSplCLib.PoleIndex(theDegree, theOutMinIdx, theIsPeriodic, theMults) + 1;
        theOutMinIdx = Max(theOutMinIdx, 1);
        theOutMaxIdx = global::Kerf.Engine.Geometry.Splines.BSplCLib.PoleIndex(theDegree, theOutMaxIdx, theIsPeriodic, theMults) + 1;
        theOutMaxIdx += theDegree - aMultiplier;
        if (!theIsPeriodic)
        {
            theOutMaxIdx = Min(theOutMaxIdx, theMaxPoleIdx);
        }
    }

    /// <summary>Compute bounding box for full surface.</summary>
    public Box Box(double theTol)
    {
        myGeom.Bounds(out double anUMin, out double anUMax, out double aVMin, out double aVMax);
        return Box(anUMin, anUMax, aVMin, aVMax, theTol);
    }

    /// <summary>Compute bounding box for surface patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Box Box(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
    {
        Box aBox = new();
        double aPTol = global::Kerf.Engine.Foundation.Precision.Parametric(global::Kerf.Engine.Foundation.Precision.Confusion());

        // Get geometry bounds.
        myGeom.Bounds(out double anUMinParam, out double anUMaxParam, out double aVMinParam, out double aVMaxParam);

        // Check if parameters lie inside geometry bounds for convex hull applicability.
        bool isOutOfBounds = ((theUMin - anUMinParam) < -aPTol || (theVMin - aVMinParam) < -aPTol
                              || (theUMax - anUMaxParam) > aPTol || (theVMax - aVMaxParam) > aPTol);

        if (!isOutOfBounds)
        {
            // Use convex hull algorithm with poles.
            int aNbUPoles = myGeom.NbUPoles();
            int aNbVPoles = myGeom.NbVPoles();
            NCollection_Array2<Point3D> aPoles = myGeom.InternalPoles();

            int anUMinIdx = 1, anUMaxIdx = aNbUPoles;
            int aVMinIdx = 1, aVMaxIdx = aNbVPoles;
            bool isUPeriodic = myGeom.IsUPeriodic();
            bool isVPeriodic = myGeom.IsVPeriodic();

            if (theUMin > anUMinParam || theUMax < anUMaxParam)
            {
                NCollection_Array1<int> aMults = myGeom.InternalUMultiplicities();
                NCollection_Array1<double> aKnots = myGeom.InternalUKnots();
                ComputePolesIndexes(aKnots,
                                    aMults,
                                    myGeom.UDegree(),
                                    theUMin,
                                    theUMax,
                                    aNbUPoles,
                                    isUPeriodic,
                                    ref anUMinIdx,
                                    ref anUMaxIdx);
            }
            if (theVMin > aVMinParam || theVMax < aVMaxParam)
            {
                NCollection_Array1<int> aMults = myGeom.InternalVMultiplicities();
                NCollection_Array1<double> aKnots = myGeom.InternalVKnots();
                ComputePolesIndexes(aKnots,
                                    aMults,
                                    myGeom.VDegree(),
                                    theVMin,
                                    theVMax,
                                    aNbVPoles,
                                    isVPeriodic,
                                    ref aVMinIdx,
                                    ref aVMaxIdx);
            }

            // Iterate over selected pole range.
            for (int anI = anUMinIdx; anI <= anUMaxIdx; anI++)
            {
                int anIP = anI;
                if (isUPeriodic && anIP > aNbUPoles)
                {
                    anIP = anIP - aNbUPoles;
                }
                for (int aJ = aVMinIdx; aJ <= aVMaxIdx; aJ++)
                {
                    int aJP = aJ;
                    if (isVPeriodic && aJP > aNbVPoles)
                    {
                        aJP = aJP - aNbVPoles;
                    }
                    aBox.Add(aPoles[anIP, aJP]);
                }
            }
            aBox.Enlarge(theTol);
            return aBox;
        }

        // Out of geometry bounds: fall back to grid sampling.
        // OCCT samples the grid without enlarging the box by a deflection estimate (occt-issues.md).
        GeomAdaptor_Surface aGASurf = new(myGeom);
        int aNbUSamples = GeomBndLib_SamplingHelpers.ComputeNbUSamples(aGASurf);
        int aNbVSamples = GeomBndLib_SamplingHelpers.ComputeNbVSamples(aGASurf);

        NCollection_Array1<double> aUParams = new(1, aNbUSamples);
        NCollection_Array1<double> aVParams = new(1, aNbVSamples);
        for (int anI = 1; anI <= aNbUSamples; anI++)
        {
            aUParams.SetValue(anI, theUMin + ((theUMax - theUMin) * (anI - 1) / (aNbUSamples - 1)));
        }
        for (int aJ = 1; aJ <= aNbVSamples; aJ++)
        {
            aVParams.SetValue(aJ, theVMin + ((theVMax - theVMin) * (aJ - 1) / (aNbVSamples - 1)));
        }

        GeomGridEval_Surface anEvaluator = new(aGASurf);
        NCollection_Array2<Point3D> aGrid = anEvaluator.EvaluateGrid(aUParams, aVParams);
        for (int anI = aGrid.LowerRow(); anI <= aGrid.UpperRow(); anI++)
        {
            for (int aJ = aGrid.LowerCol(); aJ <= aGrid.UpperCol(); aJ++)
            {
                aBox.Add(aGrid.Value(anI, aJ));
            }
        }
        aBox.Enlarge(theTol);
        return aBox;
    }

    /// <summary>Compute precise bounding box for full surface.</summary>
    public Box BoxOptimal(double theTol)
    {
        myGeom.Bounds(out double anUMin, out double anUMax, out double aVMin, out double aVMax);
        return BoxOptimal(anUMin, anUMax, aVMin, aVMax, theTol);
    }

    /// <summary>Compute precise bounding box using numerical optimization.</summary>
    /// <remarks>OCCT's body is GeomBndLib_BezierSurface::BoxOptimal's text after the adaptor; it is shared.</remarks>
    public Box BoxOptimal(double theUMin, double theUMax, double theVMin, double theVMax, double theTol)
    {
        GeomAdaptor_Surface aGASurf = new(myGeom);
        return GeomBndLib_BezierSurface.SurfaceBoxOptimal(aGASurf, theUMin, theUMax, theVMin, theVMax, theTol);
    }
}
