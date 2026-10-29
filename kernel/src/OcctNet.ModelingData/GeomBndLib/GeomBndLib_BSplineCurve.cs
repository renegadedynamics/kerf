// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_BSplineCurve.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_BSplineCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Computes bounding box for a 3D B-spline curve (Geom_BSplineCurve): the curve is segmented to the range, sampled
/// span by span with a deflection estimate, and the result reduced by the poles' convex hull.
/// </summary>
public sealed class GeomBndLib_BSplineCurve : GeomBndLib_Curve.ICurveEvaluator
{
    private readonly Geom_BSplineCurve myGeom;

    public GeomBndLib_BSplineCurve(Geom_BSplineCurve theCurve)
    {
        myGeom = theCurve;
    }

    public Geom_BSplineCurve Geometry() => myGeom;

    Geom_Curve? GeomBndLib_Curve.ICurveEvaluator.CurveGeometry() => myGeom;

    /// <summary>Compute bounding box for full curve.</summary>
    public Bnd_Box Box(double theTol) => Box(myGeom.FirstParameter(), myGeom.LastParameter(), theTol);

    /// <summary>Compute bounding box for arc [theU1, theU2].</summary>
    public Bnd_Box Box(double theU1, double theU2, double theTol)
    {
        const double aWeakness = 1.5;
        Geom_BSplineCurve aCurve = myGeom;
        double aU1 = theU1;
        double aU2 = theU2;

        if (Math.Abs(aCurve.FirstParameter() - aU1) > Precision.Parametric(theTol)
            || Math.Abs(aCurve.LastParameter() - aU2) > Precision.Parametric(theTol))
        {
            Geom_Geometry aGeometry = aCurve.Copy();
            Geom_BSplineCurve aSegmentedCurve = (Geom_BSplineCurve)aGeometry;
            if (aSegmentedCurve.IsPeriodic())
            {
                ElCLib.AdjustPeriodic(aSegmentedCurve.FirstParameter(),
                                      aSegmentedCurve.LastParameter(),
                                      Precision.PConfusion(),
                                      ref aU1,
                                      ref aU2);
            }
            else
            {
                if (aSegmentedCurve.FirstParameter() > aU1)
                {
                    aU1 = aSegmentedCurve.FirstParameter();
                }
                if (aSegmentedCurve.LastParameter() < aU2)
                {
                    aU2 = aSegmentedCurve.LastParameter();
                }
            }

            double aSegmentTol = 2.0 * Precision.PConfusion();
            if (aSegmentedCurve.IsPeriodic())
            {
                double aPeriod = aSegmentedCurve.LastParameter() - aSegmentedCurve.FirstParameter();
                double aDirectDiff = Math.Abs(aU2 - aU1);
                double aCrossPeriodDiff1 = Math.Abs(aU2 - aPeriod - aU1);
                double aCrossPeriodDiff2 = Math.Abs(aU1 - aPeriod - aU2);
                double aMinDiff = Min(aDirectDiff, Min(aCrossPeriodDiff1, aCrossPeriodDiff2));
                if (aMinDiff < aSegmentTol)
                {
                    aSegmentTol = aMinDiff * 0.01;
                }
            }
            else if (Math.Abs(aU2 - aU1) < aSegmentTol)
            {
                aSegmentTol = Math.Abs(aU2 - aU1) * 0.01;
            }
            aSegmentedCurve.Segment(aU1, aU2, aSegmentTol);
            aCurve = aSegmentedCurve;
        }

        Bnd_Box aSampledBox = new();
        int aKnotFirst = aCurve.FirstUKnotIndex();
        int aKnotLast = aCurve.LastUKnotIndex();
        int aDegree = aCurve.Degree();
        NCollection_Array1<double> aKnots = aCurve.InternalKnots();
        GeomAdaptor_Curve aGACurve = new(aCurve);
        double aMaxDeflection = 0.0;
        double aFirst = aKnots[aKnotFirst];
        for (int aKnot = aKnotFirst + 1; aKnot <= aKnotLast; ++aKnot)
        {
            double aLast = aKnots[aKnot];
            aMaxDeflection = Max(GeomBndLib_SplineHelpers.FillBox(ref aSampledBox, aGACurve, aFirst, aLast, aDegree),
                                 aMaxDeflection);
            aFirst = aLast;
        }

        Bnd_Box aBox = new();
        if (!aSampledBox.IsVoid())
        {
            aSampledBox.Enlarge(aWeakness * aMaxDeflection);
            // The poles of the whole curve, as OCCT does, also when the curve was segmented.
            GeomBndLib_SplineHelpers.ReduceSplineBox(myGeom.InternalPoles(), aSampledBox, ref aBox);
            aBox.Enlarge(theTol);
        }
        return aBox;
    }

    /// <summary>Compute precise bounding box using numerical optimization.</summary>
    public Bnd_Box BoxOptimal(double theU1, double theU2, double theTol)
    {
        GeomAdaptor_Curve aGACurve = new(myGeom);
        return GeomBndLib_SplineHelpers.CurveBoxOptimal(aGACurve, theU1, theU2, theTol);
    }
}
