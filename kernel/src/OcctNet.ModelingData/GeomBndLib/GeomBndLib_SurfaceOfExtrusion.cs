// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_SurfaceOfExtrusion.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_SurfaceOfExtrusion.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Bounds;

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Computes bounding box for a surface of linear extrusion (Geom_SurfaceOfLinearExtrusion) analytically:
/// P(U, V) = BasisCurve(U) + V * Direction, so the box is the basis curve box swept along the direction.
/// </summary>
public sealed class GeomBndLib_SurfaceOfExtrusion : GeomBndLib_Surface.ISurfaceEvaluator
{
    private readonly Geom_SurfaceOfLinearExtrusion myGeom;

    public GeomBndLib_SurfaceOfExtrusion(Geom_SurfaceOfLinearExtrusion theSurf)
    {
        myGeom = theSurf;
    }

    public Geom_SurfaceOfLinearExtrusion Geometry() => myGeom;

    /// <summary>Build the extrusion box from a pre-computed basis curve box.</summary>
    private static Box buildExtrusionBox(in Box theCurveBox,
                                             in Direction3D theDir,
                                             in Xyz theDirXYZ,
                                             double theVMin,
                                             double theVMax,
                                             double theTol)
    {
        Box aBox = new();
        if (theCurveBox.IsVoid)
        {
            aBox.Enlarge(theTol);
            return aBox;
        }

        bool isVMinInf = global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(theVMin);
        bool isVMaxInf = global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(theVMax);

        if (theCurveBox.IsOpen)
        {
            // Propagate curve openness through the extrusion rather than returning whole space.
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

            if (isVMinInf && isVMaxInf)
            {
                GeomBndLib_InfiniteHelpers.OpenMinMax(theDir, ref aBox);
            }
            else if (isVMinInf)
            {
                GeomBndLib_InfiniteHelpers.OpenMin(theDir, ref aBox);
            }
            else if (isVMaxInf)
            {
                GeomBndLib_InfiniteHelpers.OpenMax(theDir, ref aBox);
            }

            if (theCurveBox.HasFinitePart)
            {
                (double aXmin, double aXmax, double aYmin, double aYmax, double aZmin, double aZmax) = theCurveBox.FinitePart().Get();
                if (!isVMinInf)
                {
                    Xyz aShift = theVMin * theDirXYZ;
                    aBox.Add(new Point3D(aXmin + aShift.X(), aYmin + aShift.Y(), aZmin + aShift.Z()));
                    aBox.Add(new Point3D(aXmax + aShift.X(), aYmax + aShift.Y(), aZmax + aShift.Z()));
                }
                if (!isVMaxInf)
                {
                    Xyz aShift = theVMax * theDirXYZ;
                    aBox.Add(new Point3D(aXmin + aShift.X(), aYmin + aShift.Y(), aZmin + aShift.Z()));
                    aBox.Add(new Point3D(aXmax + aShift.X(), aYmax + aShift.Y(), aZmax + aShift.Z()));
                }
            }

            aBox.Enlarge(theTol);
            return aBox;
        }

        if (isVMinInf && isVMaxInf)
        {
            aBox.Add(theCurveBox);
            GeomBndLib_InfiniteHelpers.OpenMinMax(theDir, ref aBox);
            aBox.Enlarge(theTol);
            return aBox;
        }

        (double bXmin, double bXmax, double bYmin, double bYmax, double bZmin, double bZmax) = theCurveBox.Get();
        if (isVMinInf)
        {
            GeomBndLib_InfiniteHelpers.OpenMin(theDir, ref aBox);
            Xyz aShift = theVMax * theDirXYZ;
            aBox.Add(new Point3D(bXmin + aShift.X(), bYmin + aShift.Y(), bZmin + aShift.Z()));
            aBox.Add(new Point3D(bXmax + aShift.X(), bYmax + aShift.Y(), bZmax + aShift.Z()));
        }
        else if (isVMaxInf)
        {
            GeomBndLib_InfiniteHelpers.OpenMax(theDir, ref aBox);
            Xyz aShift = theVMin * theDirXYZ;
            aBox.Add(new Point3D(bXmin + aShift.X(), bYmin + aShift.Y(), bZmin + aShift.Z()));
            aBox.Add(new Point3D(bXmax + aShift.X(), bYmax + aShift.Y(), bZmax + aShift.Z()));
        }
        else
        {
            Xyz aShiftMin = theVMin * theDirXYZ;
            aBox.Add(new Point3D(bXmin + aShiftMin.X(), bYmin + aShiftMin.Y(), bZmin + aShiftMin.Z()));
            aBox.Add(new Point3D(bXmax + aShiftMin.X(), bYmax + aShiftMin.Y(), bZmax + aShiftMin.Z()));
            Xyz aShiftMax = theVMax * theDirXYZ;
            aBox.Add(new Point3D(bXmin + aShiftMax.X(), bYmin + aShiftMax.Y(), bZmin + aShiftMax.Z()));
            aBox.Add(new Point3D(bXmax + aShiftMax.X(), bYmax + aShiftMax.Y(), bZmax + aShiftMax.Z()));
        }

        aBox.Enlarge(theTol);
        return aBox;
    }

    /// <summary>Clamps [theU1, theU2] to a non-periodic basis curve's range, as both of OCCT's methods do inline.</summary>
    private static void clampCurveRange(Geom_Curve aBasisCurve, ref double aCurveU1, ref double aCurveU2)
    {
        if (!aBasisCurve.IsPeriodic())
        {
            double aFirst = aBasisCurve.FirstParameter();
            double aLast = aBasisCurve.LastParameter();
            if (aCurveU1 < aFirst)
            {
                aCurveU1 = aFirst;
            }
            else if (aCurveU1 > aLast)
            {
                aCurveU1 = aLast;
            }
            if (aCurveU2 < aFirst)
            {
                aCurveU2 = aFirst;
            }
            else if (aCurveU2 > aLast)
            {
                aCurveU2 = aLast;
            }
            if (aCurveU1 > aCurveU2)
            {
                double aTmp = aCurveU1;
                aCurveU1 = aCurveU2;
                aCurveU2 = aTmp;
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
            // P(U, V) = BasisCurve(U) + V * Direction
            Geom_Curve aBasisCurve = myGeom.BasisCurve();
            Direction3D aDir = myGeom.Direction();
            Xyz aDirXYZ = aDir.XYZ();

            double aCurveU1 = theUMin;
            double aCurveU2 = theUMax;
            clampCurveRange(aBasisCurve, ref aCurveU1, ref aCurveU2);

            GeomBndLib_Curve aCurveEval = new(aBasisCurve);
            Box aCurveBox = aCurveEval.Box(aCurveU1, aCurveU2, 0.0);
            if (aCurveBox.IsVoid)
            {
                GeomAdaptor_Surface anAdaptor0 = new(myGeom);
                GeomBndLib_OtherSurface anOther0 = new(anAdaptor0);
                return anOther0.Box(theUMin, theUMax, theVMin, theVMax, theTol);
            }

            return buildExtrusionBox(aCurveBox, aDir, aDirXYZ, theVMin, theVMax, theTol);
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
            // Use the tight basis curve box for a more precise result.
            Geom_Curve aBasisCurve = myGeom.BasisCurve();
            Direction3D aDir = myGeom.Direction();
            Xyz aDirXYZ = aDir.XYZ();

            double aCurveU1 = theUMin;
            double aCurveU2 = theUMax;
            clampCurveRange(aBasisCurve, ref aCurveU1, ref aCurveU2);

            GeomBndLib_Curve aCurveEval = new(aBasisCurve);
            Box aCurveBox = aCurveEval.BoxOptimal(aCurveU1, aCurveU2, 0.0);
            if (aCurveBox.IsVoid)
            {
                GeomAdaptor_Surface anAdaptor0 = new(myGeom);
                GeomBndLib_OtherSurface anOther0 = new(anAdaptor0);
                return anOther0.BoxOptimal(theUMin, theUMax, theVMin, theVMax, theTol);
            }

            return buildExtrusionBox(aCurveBox, aDir, aDirXYZ, theVMin, theVMax, theTol);
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
