// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_VinertGK.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_VinertGK.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Measurement;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Geometry.IsoType;
using static Kerf.Engine.Measurement.PropertyValueType;

namespace Kerf.Engine.Measurement;

/// <summary>
/// The global properties of the region of 3D space delimited by a face and the reference point, a point or a plane,
/// by adaptive Gauss-Kronrod integration (a double integral: <see cref="TFunction"/> over the boundary
/// curves of the integral of <see cref="UFunction"/> in U).
/// </summary>
/// <remarks>
/// The volume is always computed; the centre of mass if <c>theCGFlag</c> or <c>theIFlag</c>, the inertia if
/// <c>theIFlag</c>. Each Perform returns the absolute error reached, or -1 on failure.
/// </remarks>
internal class VinertGK : Properties
{
    private double _errorReached;
    private double _absolutError;

    /// <summary>An empty system.</summary>
    public VinertGK()
    {
        _errorReached = 0.0;
        _absolutError = 0.0;
    }

    public VinertGK(FaceIntegrand surface,
                              in Point3D location,
                              double tolerance = 0.001,
                              bool cgFlag = false,
                              bool iFlag = false)
    {
        _errorReached = 0.0;
        SetLocation(location);
        Perform(surface, tolerance, cgFlag, iFlag);
    }

    public VinertGK(FaceIntegrand surface,
                              in Point3D point,
                              in Point3D location,
                              double tolerance = 0.001,
                              bool cgFlag = false,
                              bool iFlag = false)
    {
        _errorReached = 0.0;
        SetLocation(location);
        Perform(surface, point, tolerance, cgFlag, iFlag);
    }

    public VinertGK(FaceIntegrand surface,
                              Domain domain,
                              in Point3D location,
                              double tolerance = 0.001,
                              bool cgFlag = false,
                              bool iFlag = false)
    {
        _errorReached = 0.0;
        SetLocation(location);
        Perform(surface, domain, tolerance, cgFlag, iFlag);
    }

    public VinertGK(FaceIntegrand surface,
                              Domain domain,
                              in Point3D point,
                              in Point3D location,
                              double tolerance = 0.001,
                              bool cgFlag = false,
                              bool iFlag = false)
    {
        _errorReached = 0.0;
        SetLocation(location);
        Perform(surface, domain, point, tolerance, cgFlag, iFlag);
    }

    public VinertGK(FaceIntegrand surface,
                              in Plane3D plane,
                              in Point3D location,
                              double tolerance = 0.001,
                              bool cgFlag = false,
                              bool iFlag = false)
    {
        _errorReached = 0.0;
        SetLocation(location);
        Perform(surface, plane, tolerance, cgFlag, iFlag);
    }

    public VinertGK(FaceIntegrand surface,
                              Domain domain,
                              in Plane3D plane,
                              in Point3D location,
                              double tolerance = 0.001,
                              bool cgFlag = false,
                              bool iFlag = false)
    {
        _errorReached = 0.0;
        SetLocation(location);
        Perform(surface, domain, plane, tolerance, cgFlag, iFlag);
    }

    /// <summary>Sets the reference point.</summary>
    public void SetLocation(in Point3D location) => _loc = location;

    /// <summary>The region between the face, over its natural bounds, and the location.</summary>
    public double Perform(FaceIntegrand surface, double tolerance = 0.001, bool cgFlag = false, bool iFlag = false)
    {
        ReadOnlySpan<double> shift = [0.0, 0.0, 0.0];

        return PrivatePerform(surface, null, true, shift, tolerance, cgFlag, iFlag);
    }

    /// <summary>The region between the face, over its natural bounds, and the location; inertia at <paramref name="point"/>.</summary>
    public double Perform(FaceIntegrand surface,
                          in Point3D point,
                          double tolerance = 0.001,
                          bool cgFlag = false,
                          bool iFlag = false)
    {
        Xyz xyz = point.XYZ().Subtracted(_loc.XYZ());
        Span<double> shift = stackalloc double[3];

        xyz.Coord(out shift[0], out shift[1], out shift[2]);

        return PrivatePerform(surface, null, true, shift, tolerance, cgFlag, iFlag);
    }

    /// <summary>The region between the face, bounded by its edges, and the location.</summary>
    public double Perform(FaceIntegrand surface,
                          Domain domain,
                          double tolerance = 0.001,
                          bool cgFlag = false,
                          bool iFlag = false)
    {
        ReadOnlySpan<double> shift = [0.0, 0.0, 0.0];

        return PrivatePerform(surface, domain, true, shift, tolerance, cgFlag, iFlag);
    }

    /// <summary>The region between the face, bounded by its edges, and the location; inertia at <paramref name="point"/>.</summary>
    public double Perform(FaceIntegrand surface,
                          Domain domain,
                          in Point3D point,
                          double tolerance = 0.001,
                          bool cgFlag = false,
                          bool iFlag = false)
    {
        Xyz xyz = point.XYZ().Subtracted(_loc.XYZ());
        Span<double> shift = stackalloc double[3];

        xyz.Coord(out shift[0], out shift[1], out shift[2]);

        return PrivatePerform(surface, domain, true, shift, tolerance, cgFlag, iFlag);
    }

    /// <summary>The region between the face, over its natural bounds, and the plane.</summary>
    public double Perform(FaceIntegrand surface,
                          in Plane3D plane,
                          double tolerance = 0.001,
                          bool cgFlag = false,
                          bool iFlag = false)
    {
        Span<double> coeff = stackalloc double[4];
        double xLoc;
        double yLoc;
        double zLoc;

        _loc.Coord(out xLoc, out yLoc, out zLoc);
        plane.Coefficients(out coeff[0], out coeff[1], out coeff[2], out coeff[3]);
        coeff[3] = coeff[3] - coeff[0] * xLoc - coeff[1] * yLoc - coeff[2] * zLoc;

        return PrivatePerform(surface, null, false, coeff, tolerance, cgFlag, iFlag);
    }

    /// <summary>The region between the face, bounded by its edges, and the plane.</summary>
    public double Perform(FaceIntegrand surface,
                          Domain domain,
                          in Plane3D plane,
                          double tolerance = 0.001,
                          bool cgFlag = false,
                          bool iFlag = false)
    {
        Span<double> coeff = stackalloc double[4];
        double xLoc;
        double yLoc;
        double zLoc;

        _loc.Coord(out xLoc, out yLoc, out zLoc);
        plane.Coefficients(out coeff[0], out coeff[1], out coeff[2], out coeff[3]);
        coeff[3] = coeff[3] - coeff[0] * xLoc - coeff[1] * yLoc - coeff[2] * zLoc;

        return PrivatePerform(surface, domain, false, coeff, tolerance, cgFlag, iFlag);
    }

    /// <summary>The relative error of the volume reached by the last Perform.</summary>
    public double GetErrorReached() => _errorReached;

    /// <summary>The absolute error of the volume reached by the last Perform.</summary>
    // OCCT declares GetAbsolutError but never defines it (a link error if called: occt-issues.md); the port returns
    // the absolute error that Perform returns
    public double GetAbsolutError() => _absolutError;

    private double PrivatePerform(FaceIntegrand surface,
                                  Domain? ptrDomain,
                                  bool isByPoint,
                                  ReadOnlySpan<double> coeffs,
                                  double tolerance,
                                  bool cgFlag,
                                  bool iFlag)
    {
        const double tTol = 1.0e-9;
        ReadOnlySpan<double> aCoeffs = coeffs;

        // Compute the number of 2d bounding curves of the face.
        Domain? pDomain = null;
        int nbCurves = 0;

        // If the pointer to the domain is NULL, there is only one curve to treat:
        // U isoline with the UMax parameter.
        if (ptrDomain is null)
        {
            nbCurves = 1;
        }
        else
        {
            pDomain = ptrDomain;

            for (pDomain.Init(); pDomain.More(); pDomain.Next())
            {
                nbCurves++;
            }
        }

        if (nbCurves == 0)
        {
            _errorReached = -1.0;

            return _errorReached;
        }

        // double    aCrvTol = 0.5*theTolerance/aNbCurves;
        double crvTol = 0.1 * tolerance;
        double uMin;
        double uMax;
        double tMin;
        double tMax;
        int nbPnts;
        int nbMaxIter = 1000;
        int nbVal = 10;
        int k;
        var localValue = new math_Vector(1, nbVal);
        var localTolReached = new math_Vector(1, nbVal);
        var value = new math_Vector(1, nbVal);
        var tolReached = new math_Vector(1, nbVal);
        var cFlags = new NCollection_Array1<bool>(1, nbVal);
        cFlags.Init(false);
        bool isMore;

        // aNbVal = 1;
        value.Init(0.0);
        tolReached.Init(0.0);

        cFlags.Init(false);
        cFlags[1] = true;

        if (cgFlag || iFlag)
        {
            int i;

            for (i = 2; i <= 4; ++i)
            {
                cFlags[i] = true;
            }
        }

        if (iFlag)
        {
            int i;

            for (i = 5; i <= 10; ++i)
            {
                cFlags[i] = true;
            }
        }

        surface.Bounds(out uMin, out uMax, out tMin, out tMax);

        if (ptrDomain is null)
        {
            isMore = true;
        }
        else
        {
            pDomain!.Init();
            isMore = pDomain.More();
        }

        while (isMore)
        {
            // If the pointer to the domain is NULL, there is only one curve to treat:
            // U isoline with the UMax parameter.

            if (ptrDomain is null)
            {
                surface.Load(false, IsoU);
            }
            else
            {
                surface.Load(pDomain!.Value());
            }

            tMin = surface.FirstParameter();
            tMax = surface.LastParameter();

            // Get the spans on the curve.
            var tFunc = new TFunction(surface, _loc, isByPoint, coeffs, uMin, crvTol);

            NCollection_HArray1<double> tKnots = surface.GetTKnots(tMin, tMax);

            int iU = tKnots.Upper();
            int nbTIntervals = tKnots.Length() - 1;
            // double                 aTolSpan       = aCrvTol/aNbTIntervals;
            double tolSpan = 0.9 * tolerance; // Relative error
            var integral = new KronrodSingleIntegration();
            PropertyValueType valueType;

            // Empirical criterion.
            nbPnts = Min(15, surface.IntegrationOrder() / nbTIntervals + 1);
            nbPnts = Max(5, nbPnts);
            //     aNbPnts = theSurface.IntegrationOrder();

            localValue.Init(0.0);
            localTolReached.Init(0.0);

            for (k = 1; k <= nbVal; k++)
            {
                if (!cFlags[k])
                {
                    continue;
                }

                int i = tKnots.Lower();

                switch (k)
                {
                    case 1:
                        valueType = global::Kerf.Engine.Measurement.PropertyValueType.Mass;
                        break;
                    case 2:
                        valueType = CenterMassX;
                        break;
                    case 3:
                        valueType = CenterMassY;
                        break;
                    case 4:
                        valueType = CenterMassZ;
                        break;
                    case 5:
                        valueType = InertiaXX;
                        break;
                    case 6:
                        valueType = InertiaYY;
                        break;
                    case 7:
                        valueType = InertiaZZ;
                        break;
                    case 8:
                        valueType = InertiaXY;
                        break;
                    case 9:
                        valueType = InertiaXZ;
                        break;
                    case 10:
                        valueType = InertiaYZ;
                        break;

                    default:
                        _errorReached = -1.0;
                        return _errorReached;
                }

                tFunc.SetValueType(valueType);

                double err1 = 0.0;
                while (i < iU)
                {
                    // std::cout << "-------------- Span " << i << " nbp: " << aNbPnts << std::endl;
                    double t1 = tKnots.Value(i++);
                    double t2 = tKnots.Value(i);

                    if (t2 - t1 < tTol)
                    {
                        continue;
                    }

                    tFunc.SetNbKronrodPoints(nbPnts);
                    tFunc.Init();
                    tFunc.SetTolerance(crvTol / (t2 - t1));
                    integral.Perform(tFunc, t1, t2, nbPnts, tolSpan, nbMaxIter);

                    if (!integral.IsDone)
                    {
                        _errorReached = -1.0;

                        return _errorReached;
                    }

                    localValue[k] += integral.Value();
                    err1 = tFunc.AbsolutError() * (t2 - t1);
                    // std::cout << "Errors: " << anIntegral.NbIterReached() << " " << anIntegral.AbsolutError()
                    // << " " << err1 << std::endl;
                    localTolReached[k] += integral.AbsolutError() + err1;
                    // std::cout << "--- Errors: " << anIntegral.NbIterReached() << " " <<
                    // anIntegral.AbsolutError() << " " << err1 << std::endl;
                }

                value[k] += localValue[k];
                tolReached[k] += localTolReached[k];
            }

            // If the pointer to the domain is NULL, there is only one curve to treat:
            // U isoline with the UMax parameter.
            if (ptrDomain is null)
            {
                isMore = false;
            }
            else
            {
                pDomain!.Next();
                isMore = pDomain.More();
            }
        }

        // Get volume value.
        _dim = value[1];
        _errorReached = tolReached[1];
        _absolutError = _errorReached;
        double absDim = Math.Abs(_dim);
        double volTol = Epsilon(_absolutError);
        if (absDim >= volTol)
        {
            _errorReached /= absDim;
        }

        if (cgFlag || iFlag)
        {
            // Compute values of center of mass.
            if (absDim >= volTol)
            {
                if (isByPoint)
                {
                    value[2] = aCoeffs[0] + value[2] / _dim;
                    value[3] = aCoeffs[1] + value[3] / _dim;
                    value[4] = aCoeffs[2] + value[4] / _dim;
                }
                else
                {
                    value[2] /= _dim;
                    value[3] /= _dim;
                    value[4] /= _dim;
                }
            }
            else
            {
                value[2] = 0.0;
                value[3] = 0.0;
                value[4] = 0.0;
                _dim = 0.0;
            }

            _g = _g.WithCoord(value[2], value[3], value[4]);
        }

        if (iFlag)
        {
            // Fill the matrix of inertia.
            _inertia = _inertia.WithCols(new Xyz(value[5], value[8], value[9]),
                            new Xyz(value[8], value[6], value[10]),
                            new Xyz(value[9], value[10], value[7]));
        }

        // return myErrorReached;
        return _absolutError;
    }
}
