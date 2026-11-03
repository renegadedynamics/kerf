// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_VinertGK.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_VinertGK.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.IsoType;
using static Occt.GProp_ValueType;

namespace Occt;

/// <summary>
/// The global properties of the region of 3D space delimited by a face and the reference point, a point or a plane,
/// by adaptive Gauss-Kronrod integration (a double integral: <see cref="BRepGProp_TFunction"/> over the boundary
/// curves of the integral of <see cref="BRepGProp_UFunction"/> in U).
/// </summary>
/// <remarks>
/// The volume is always computed; the centre of mass if <c>theCGFlag</c> or <c>theIFlag</c>, the inertia if
/// <c>theIFlag</c>. Each Perform returns the absolute error reached, or -1 on failure.
/// </remarks>
public class BRepGProp_VinertGK : GProp_GProps
{
    private double myErrorReached;
    private double myAbsolutError;

    /// <summary>An empty system.</summary>
    public BRepGProp_VinertGK()
    {
        myErrorReached = 0.0;
        myAbsolutError = 0.0;
    }

    public BRepGProp_VinertGK(BRepGProp_Face theSurface,
                              in Point3D theLocation,
                              double theTolerance = 0.001,
                              bool theCGFlag = false,
                              bool theIFlag = false)
    {
        myErrorReached = 0.0;
        SetLocation(theLocation);
        Perform(theSurface, theTolerance, theCGFlag, theIFlag);
    }

    public BRepGProp_VinertGK(BRepGProp_Face theSurface,
                              in Point3D thePoint,
                              in Point3D theLocation,
                              double theTolerance = 0.001,
                              bool theCGFlag = false,
                              bool theIFlag = false)
    {
        myErrorReached = 0.0;
        SetLocation(theLocation);
        Perform(theSurface, thePoint, theTolerance, theCGFlag, theIFlag);
    }

    public BRepGProp_VinertGK(BRepGProp_Face theSurface,
                              BRepGProp_Domain theDomain,
                              in Point3D theLocation,
                              double theTolerance = 0.001,
                              bool theCGFlag = false,
                              bool theIFlag = false)
    {
        myErrorReached = 0.0;
        SetLocation(theLocation);
        Perform(theSurface, theDomain, theTolerance, theCGFlag, theIFlag);
    }

    public BRepGProp_VinertGK(BRepGProp_Face theSurface,
                              BRepGProp_Domain theDomain,
                              in Point3D thePoint,
                              in Point3D theLocation,
                              double theTolerance = 0.001,
                              bool theCGFlag = false,
                              bool theIFlag = false)
    {
        myErrorReached = 0.0;
        SetLocation(theLocation);
        Perform(theSurface, theDomain, thePoint, theTolerance, theCGFlag, theIFlag);
    }

    public BRepGProp_VinertGK(BRepGProp_Face theSurface,
                              in Plane3D thePlane,
                              in Point3D theLocation,
                              double theTolerance = 0.001,
                              bool theCGFlag = false,
                              bool theIFlag = false)
    {
        myErrorReached = 0.0;
        SetLocation(theLocation);
        Perform(theSurface, thePlane, theTolerance, theCGFlag, theIFlag);
    }

    public BRepGProp_VinertGK(BRepGProp_Face theSurface,
                              BRepGProp_Domain theDomain,
                              in Plane3D thePlane,
                              in Point3D theLocation,
                              double theTolerance = 0.001,
                              bool theCGFlag = false,
                              bool theIFlag = false)
    {
        myErrorReached = 0.0;
        SetLocation(theLocation);
        Perform(theSurface, theDomain, thePlane, theTolerance, theCGFlag, theIFlag);
    }

    /// <summary>Sets the reference point.</summary>
    public void SetLocation(in Point3D theLocation) => loc = theLocation;

    /// <summary>The region between the face, over its natural bounds, and the location.</summary>
    public double Perform(BRepGProp_Face theSurface, double theTolerance = 0.001, bool theCGFlag = false, bool theIFlag = false)
    {
        ReadOnlySpan<double> aShift = [0.0, 0.0, 0.0];

        return PrivatePerform(theSurface, null, true, aShift, theTolerance, theCGFlag, theIFlag);
    }

    /// <summary>The region between the face, over its natural bounds, and the location; inertia at <paramref name="thePoint"/>.</summary>
    public double Perform(BRepGProp_Face theSurface,
                          in Point3D thePoint,
                          double theTolerance = 0.001,
                          bool theCGFlag = false,
                          bool theIFlag = false)
    {
        Xyz aXYZ = thePoint.XYZ().Subtracted(loc.XYZ());
        Span<double> aShift = stackalloc double[3];

        aXYZ.Coord(out aShift[0], out aShift[1], out aShift[2]);

        return PrivatePerform(theSurface, null, true, aShift, theTolerance, theCGFlag, theIFlag);
    }

    /// <summary>The region between the face, bounded by its edges, and the location.</summary>
    public double Perform(BRepGProp_Face theSurface,
                          BRepGProp_Domain theDomain,
                          double theTolerance = 0.001,
                          bool theCGFlag = false,
                          bool theIFlag = false)
    {
        ReadOnlySpan<double> aShift = [0.0, 0.0, 0.0];

        return PrivatePerform(theSurface, theDomain, true, aShift, theTolerance, theCGFlag, theIFlag);
    }

    /// <summary>The region between the face, bounded by its edges, and the location; inertia at <paramref name="thePoint"/>.</summary>
    public double Perform(BRepGProp_Face theSurface,
                          BRepGProp_Domain theDomain,
                          in Point3D thePoint,
                          double theTolerance = 0.001,
                          bool theCGFlag = false,
                          bool theIFlag = false)
    {
        Xyz aXYZ = thePoint.XYZ().Subtracted(loc.XYZ());
        Span<double> aShift = stackalloc double[3];

        aXYZ.Coord(out aShift[0], out aShift[1], out aShift[2]);

        return PrivatePerform(theSurface, theDomain, true, aShift, theTolerance, theCGFlag, theIFlag);
    }

    /// <summary>The region between the face, over its natural bounds, and the plane.</summary>
    public double Perform(BRepGProp_Face theSurface,
                          in Plane3D thePlane,
                          double theTolerance = 0.001,
                          bool theCGFlag = false,
                          bool theIFlag = false)
    {
        Span<double> aCoeff = stackalloc double[4];
        double aXLoc;
        double aYLoc;
        double aZLoc;

        loc.Coord(out aXLoc, out aYLoc, out aZLoc);
        thePlane.Coefficients(out aCoeff[0], out aCoeff[1], out aCoeff[2], out aCoeff[3]);
        aCoeff[3] = aCoeff[3] - aCoeff[0] * aXLoc - aCoeff[1] * aYLoc - aCoeff[2] * aZLoc;

        return PrivatePerform(theSurface, null, false, aCoeff, theTolerance, theCGFlag, theIFlag);
    }

    /// <summary>The region between the face, bounded by its edges, and the plane.</summary>
    public double Perform(BRepGProp_Face theSurface,
                          BRepGProp_Domain theDomain,
                          in Plane3D thePlane,
                          double theTolerance = 0.001,
                          bool theCGFlag = false,
                          bool theIFlag = false)
    {
        Span<double> aCoeff = stackalloc double[4];
        double aXLoc;
        double aYLoc;
        double aZLoc;

        loc.Coord(out aXLoc, out aYLoc, out aZLoc);
        thePlane.Coefficients(out aCoeff[0], out aCoeff[1], out aCoeff[2], out aCoeff[3]);
        aCoeff[3] = aCoeff[3] - aCoeff[0] * aXLoc - aCoeff[1] * aYLoc - aCoeff[2] * aZLoc;

        return PrivatePerform(theSurface, theDomain, false, aCoeff, theTolerance, theCGFlag, theIFlag);
    }

    /// <summary>The relative error of the volume reached by the last Perform.</summary>
    public double GetErrorReached() => myErrorReached;

    /// <summary>The absolute error of the volume reached by the last Perform.</summary>
    // OCCT declares GetAbsolutError but never defines it (a link error if called: occt-issues.md); the port returns
    // the absolute error that Perform returns
    public double GetAbsolutError() => myAbsolutError;

    private double PrivatePerform(BRepGProp_Face theSurface,
                                  BRepGProp_Domain? thePtrDomain,
                                  bool IsByPoint,
                                  ReadOnlySpan<double> theCoeffs,
                                  double theTolerance,
                                  bool theCGFlag,
                                  bool theIFlag)
    {
        const double aTTol = 1.0e-9;
        ReadOnlySpan<double> aCoeffs = theCoeffs;

        // Compute the number of 2d bounding curves of the face.
        BRepGProp_Domain? aPDomain = null;
        int aNbCurves = 0;

        // If the pointer to the domain is NULL, there is only one curve to treat:
        // U isoline with the UMax parameter.
        if (thePtrDomain is null)
        {
            aNbCurves = 1;
        }
        else
        {
            aPDomain = thePtrDomain;

            for (aPDomain.Init(); aPDomain.More(); aPDomain.Next())
            {
                aNbCurves++;
            }
        }

        if (aNbCurves == 0)
        {
            myErrorReached = -1.0;

            return myErrorReached;
        }

        // double    aCrvTol = 0.5*theTolerance/aNbCurves;
        double aCrvTol = 0.1 * theTolerance;
        double aUMin;
        double aUMax;
        double aTMin;
        double aTMax;
        int aNbPnts;
        int aNbMaxIter = 1000;
        int aNbVal = 10;
        int k;
        var aLocalValue = new math_Vector(1, aNbVal);
        var aLocalTolReached = new math_Vector(1, aNbVal);
        var aValue = new math_Vector(1, aNbVal);
        var aTolReached = new math_Vector(1, aNbVal);
        var CFlags = new NCollection_Array1<bool>(1, aNbVal);
        CFlags.Init(false);
        bool isMore;

        // aNbVal = 1;
        aValue.Init(0.0);
        aTolReached.Init(0.0);

        CFlags.Init(false);
        CFlags[1] = true;

        if (theCGFlag || theIFlag)
        {
            int i;

            for (i = 2; i <= 4; ++i)
            {
                CFlags[i] = true;
            }
        }

        if (theIFlag)
        {
            int i;

            for (i = 5; i <= 10; ++i)
            {
                CFlags[i] = true;
            }
        }

        theSurface.Bounds(out aUMin, out aUMax, out aTMin, out aTMax);

        if (thePtrDomain is null)
        {
            isMore = true;
        }
        else
        {
            aPDomain!.Init();
            isMore = aPDomain.More();
        }

        while (isMore)
        {
            // If the pointer to the domain is NULL, there is only one curve to treat:
            // U isoline with the UMax parameter.

            if (thePtrDomain is null)
            {
                theSurface.Load(false, IsoU);
            }
            else
            {
                theSurface.Load(aPDomain!.Value());
            }

            aTMin = theSurface.FirstParameter();
            aTMax = theSurface.LastParameter();

            // Get the spans on the curve.
            var aTFunc = new BRepGProp_TFunction(theSurface, loc, IsByPoint, theCoeffs, aUMin, aCrvTol);

            NCollection_HArray1<double> aTKnots = theSurface.GetTKnots(aTMin, aTMax);

            int iU = aTKnots.Upper();
            int aNbTIntervals = aTKnots.Length() - 1;
            // double                 aTolSpan       = aCrvTol/aNbTIntervals;
            double aTolSpan = 0.9 * theTolerance; // Relative error
            var anIntegral = new KronrodSingleIntegration();
            GProp_ValueType aValueType;

            // Empirical criterion.
            aNbPnts = Min(15, theSurface.IntegrationOrder() / aNbTIntervals + 1);
            aNbPnts = Max(5, aNbPnts);
            //     aNbPnts = theSurface.IntegrationOrder();

            aLocalValue.Init(0.0);
            aLocalTolReached.Init(0.0);

            for (k = 1; k <= aNbVal; k++)
            {
                if (!CFlags[k])
                {
                    continue;
                }

                int i = aTKnots.Lower();

                switch (k)
                {
                    case 1:
                        aValueType = GProp_Mass;
                        break;
                    case 2:
                        aValueType = GProp_CenterMassX;
                        break;
                    case 3:
                        aValueType = GProp_CenterMassY;
                        break;
                    case 4:
                        aValueType = GProp_CenterMassZ;
                        break;
                    case 5:
                        aValueType = GProp_InertiaXX;
                        break;
                    case 6:
                        aValueType = GProp_InertiaYY;
                        break;
                    case 7:
                        aValueType = GProp_InertiaZZ;
                        break;
                    case 8:
                        aValueType = GProp_InertiaXY;
                        break;
                    case 9:
                        aValueType = GProp_InertiaXZ;
                        break;
                    case 10:
                        aValueType = GProp_InertiaYZ;
                        break;

                    default:
                        myErrorReached = -1.0;
                        return myErrorReached;
                }

                aTFunc.SetValueType(aValueType);

                double err1 = 0.0;
                while (i < iU)
                {
                    // std::cout << "-------------- Span " << i << " nbp: " << aNbPnts << std::endl;
                    double aT1 = aTKnots.Value(i++);
                    double aT2 = aTKnots.Value(i);

                    if (aT2 - aT1 < aTTol)
                    {
                        continue;
                    }

                    aTFunc.SetNbKronrodPoints(aNbPnts);
                    aTFunc.Init();
                    aTFunc.SetTolerance(aCrvTol / (aT2 - aT1));
                    anIntegral.Perform(aTFunc, aT1, aT2, aNbPnts, aTolSpan, aNbMaxIter);

                    if (!anIntegral.IsDone)
                    {
                        myErrorReached = -1.0;

                        return myErrorReached;
                    }

                    aLocalValue[k] += anIntegral.Value();
                    err1 = aTFunc.AbsolutError() * (aT2 - aT1);
                    // std::cout << "Errors: " << anIntegral.NbIterReached() << " " << anIntegral.AbsolutError()
                    // << " " << err1 << std::endl;
                    aLocalTolReached[k] += anIntegral.AbsolutError() + err1;
                    // std::cout << "--- Errors: " << anIntegral.NbIterReached() << " " <<
                    // anIntegral.AbsolutError() << " " << err1 << std::endl;
                }

                aValue[k] += aLocalValue[k];
                aTolReached[k] += aLocalTolReached[k];
            }

            // If the pointer to the domain is NULL, there is only one curve to treat:
            // U isoline with the UMax parameter.
            if (thePtrDomain is null)
            {
                isMore = false;
            }
            else
            {
                aPDomain!.Next();
                isMore = aPDomain.More();
            }
        }

        // Get volume value.
        dim = aValue[1];
        myErrorReached = aTolReached[1];
        myAbsolutError = myErrorReached;
        double anAbsDim = Math.Abs(dim);
        double aVolTol = Epsilon(myAbsolutError);
        if (anAbsDim >= aVolTol)
        {
            myErrorReached /= anAbsDim;
        }

        if (theCGFlag || theIFlag)
        {
            // Compute values of center of mass.
            if (anAbsDim >= aVolTol)
            {
                if (IsByPoint)
                {
                    aValue[2] = aCoeffs[0] + aValue[2] / dim;
                    aValue[3] = aCoeffs[1] + aValue[3] / dim;
                    aValue[4] = aCoeffs[2] + aValue[4] / dim;
                }
                else
                {
                    aValue[2] /= dim;
                    aValue[3] /= dim;
                    aValue[4] /= dim;
                }
            }
            else
            {
                aValue[2] = 0.0;
                aValue[3] = 0.0;
                aValue[4] = 0.0;
                dim = 0.0;
            }

            g.SetCoord(aValue[2], aValue[3], aValue[4]);
        }

        if (theIFlag)
        {
            // Fill the matrix of inertia.
            inertia.SetCols(new Xyz(aValue[5], aValue[8], aValue[9]),
                            new Xyz(aValue[8], aValue[6], aValue[10]),
                            new Xyz(aValue[9], aValue[10], aValue[7]));
        }

        // return myErrorReached;
        return myAbsolutError;
    }
}
