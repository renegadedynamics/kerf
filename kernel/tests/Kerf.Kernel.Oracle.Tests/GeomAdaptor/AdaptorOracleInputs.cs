// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Geometry;
using static Kerf.Kernel.Oracle.Tests.GeomOracleInputs;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// Random curves and surfaces of every kind GeomAdaptor knows, built natively and in the port from the same raw
/// values through the same constructors (decision 165), and the comparisons the adaptor and offset-surface oracle
/// tests share.
/// </summary>
internal static class AdaptorOracleInputs
{
    internal static Exception? Catch(Action theAction)
    {
        try
        {
            theAction();
            return null;
        }
        catch (Exception anE)
        {
            return anE;
        }
    }

    /// <summary>Both threw, or neither did; true if neither did.</summary>
    internal static bool SameOutcome(Exception? theNative, Exception? thePort, string theWhat)
    {
        Assert.True((theNative is null) == (thePort is null),
                    $"{theWhat}: native {(theNative is null ? "succeeded" : "threw " + theNative.Message)}, "
                    + $"port {(thePort is null ? "succeeded" : "threw " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(thePort.GetType()) + " " + thePort.Message)}");
        return theNative is null;
    }

    // ---------------------------------------------------------------------------------------------------------
    // Curves

    /// <summary>
    /// A random B-spline curve: degree in [theMinDegree, theMaxDegree], 2 to 6 knots, inner multiplicities up to
    /// Degree - theSmoothness, periodic about one time in three (unless forced), rational half the time.
    /// </summary>
    internal static (N.Geom_BSplineCurve Native, BSplineCurve3D Port) RandomBSplineCurve(
        Oracle theO, bool? thePeriodic = null, int theMinDegree = 1, int theMaxDegree = 6, int theSmoothness = 0)
    {
        while (true)
        {
            int aDeg = theO.Int(theMinDegree, theMaxDegree);
            bool aPer = thePeriodic ?? theO.Int(0, 2) == 0;
            int aNbKnots = theO.Int(2, 6);
            var aKnots = new NCollection_Array1<double>(1, aNbKnots);
            var aMults = new NCollection_Array1<int>(1, aNbKnots);
            aKnots[1] = theO.Real(-5, 5);
            for (int i = 2; i <= aNbKnots; i++)
            {
                aKnots[i] = aKnots[i - 1] + theO.Real(0.2, 2);
            }
            int aMaxInner = Math.Max(1, aDeg - theSmoothness);
            for (int i = 2; i < aNbKnots; i++)
            {
                aMults[i] = theO.Int(1, aMaxInner);
            }
            if (aPer)
            {
                aMults[1] = aMults[aNbKnots] = theO.Int(1, aMaxInner);
            }
            else
            {
                aMults[1] = aMults[aNbKnots] = aDeg + 1;
            }
            int aNbPoles = Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(aDeg, aPer, aMults);
            if (aNbPoles < 2 || (aPer && aNbPoles <= aDeg))
            {
                continue;
            }
            var aPoles = new NCollection_Array1<Point3D>(1, aNbPoles);
            for (int i = 1; i <= aNbPoles; i++)
            {
                aPoles[i] = new Point3D(theO.Real(-10, 10), theO.Real(-10, 10), theO.Real(-10, 10));
            }
            NCollection_Array1<double>? aWeights = theO.Int(0, 1) == 0 ? null : theO.Weights(aNbPoles, true);
            return (Oracle.ToNativeCurve(aPoles, aWeights, aKnots, aMults, aDeg, aPer),
                    aWeights is null ? new BSplineCurve3D(aPoles, aKnots, aMults, aDeg, aPer)
                                     : new BSplineCurve3D(aPoles, aWeights, aKnots, aMults, aDeg, aPer));
        }
    }

    internal static (N.Geom_BezierCurve Native, BezierCurve3D Port) RandomBezierCurve(Oracle theO, int theMaxDegree = 9)
    {
        int aNbPoles = theO.Int(2, theMaxDegree + 1);
        var aPoles = new NCollection_Array1<Point3D>(1, aNbPoles);
        for (int i = 1; i <= aNbPoles; i++)
        {
            aPoles[i] = new Point3D(theO.Real(-10, 10), theO.Real(-10, 10), theO.Real(-10, 10));
        }
        using var aNPoles = Oracle.ToNative(aPoles);
        if (theO.Int(0, 1) == 0)
        {
            return (new N.Geom_BezierCurve(aNPoles), new BezierCurve3D(aPoles));
        }
        NCollection_Array1<double> aWeights = theO.Weights(aNbPoles, true);
        using var aNWeights = Oracle.ToNative(aWeights);
        return (new N.Geom_BezierCurve(aNPoles, aNWeights), new BezierCurve3D(aPoles, aWeights));
    }

    /// <summary>The names of <see cref="RandomCurve"/>'s kinds.</summary>
    internal static readonly string[] CurveKinds =
        ["line", "circle", "ellipse", "hyperbola", "parabola", "Bezier", "B-spline", "periodic B-spline", "offset", "trimmed"];

    /// <summary>
    /// A random curve of the given kind (<see cref="CurveKinds"/>); Range is a sensible half-range of parameters
    /// for an unbounded curve.
    /// </summary>
    internal static (N.Geom_Curve Native, Curve3D Port, double Range) RandomCurve(Oracle theO, int theKind)
    {
        switch (theKind)
        {
            case 0:
            {
                (N.gp_Ax1 aNAx1, Axis3D anAx1) = Ax1s(theO);
                using var aNAx1Owner = aNAx1;
                return (new N.Geom_Line(aNAx1), new LineCurve3D(anAx1), 50);
            }
            case 1:
            case 2:
            case 3:
            case 4:
            {
                (N.gp_Ax2 aNAx2, Frame3D anAx2) = Ax2sFromDirection(theO);
                using var aNAx2Owner = aNAx2;
                if (theKind == 1)
                {
                    double aR = theO.Real(0.1, 20);
                    return (new N.Geom_Circle(aNAx2, aR), new CircleCurve3D(anAx2, aR), 10);
                }
                if (theKind == 2)
                {
                    double aMaj = theO.Real(0.1, 20), aMin = theO.Real(0.05, aMaj);
                    return (new N.Geom_Ellipse(aNAx2, aMaj, aMin), new EllipseCurve3D(anAx2, aMaj, aMin), 10);
                }
                if (theKind == 3)
                {
                    double aMaj = theO.Real(0.1, 20), aMin = theO.Real(0.1, 20);
                    return (new N.Geom_Hyperbola(aNAx2, aMaj, aMin), new HyperbolaCurve3D(anAx2, aMaj, aMin), 3);
                }
                double aF = theO.Real(0.01, 10);
                return (new N.Geom_Parabola(aNAx2, aF), new ParabolaCurve3D(anAx2, aF), 20);
            }
            case 5:
            {
                (N.Geom_BezierCurve aN, BezierCurve3D aP) = RandomBezierCurve(theO);
                return (aN, aP, 1);
            }
            case 6:
            case 7:
            {
                (N.Geom_BSplineCurve aN, BSplineCurve3D aP) = RandomBSplineCurve(theO, theKind == 7);
                return (aN, aP, 5);
            }
            case 8:
            {
                // an offset of a smooth B-spline curve, a Bezier curve or a circle
                N.Geom_Curve aNB;
                Curve3D aPB;
                switch (theO.Int(0, 2))
                {
                    case 0:
                        (aNB, aPB) = RandomBSplineCurve(theO, null, 3, 6, 2);
                        break;
                    case 1:
                        (aNB, aPB) = RandomBezierCurve(theO, 7);
                        break;
                    default:
                        (aNB, aPB, _) = RandomCurve(theO, 1);
                        break;
                }
                using (aNB)
                {
                    (N.gp_Dir aNDir, Direction3D aDir) = Dirs(theO);
                    using var aNDirOwner = aNDir;
                    double anOffset = theO.Real(0.1, 5) * (theO.Bool() ? 1 : -1);
                    return (new N.Geom_OffsetCurve(aNB, anOffset, aNDir, true), new OffsetCurve3D(aPB, anOffset, aDir, true),
                            5);
                }
            }
            default:
            {
                // a trimmed curve of one of the others (not a trimmed curve)
                int aKind;
                (N.Geom_Curve aNB, Curve3D aPB, double aRange) = RandomCurve(theO, aKind = theO.Int(0, 8));
                using (aNB)
                {
                    double aF = aPB.FirstParameter(), aL = aPB.LastParameter();
                    if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aF) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aL) || aPB.IsPeriodic)
                    {
                        aF = -aRange;
                        aL = aRange;
                        if (aPB.IsPeriodic && aKind != 7)
                        {
                            aF = theO.Real(-7, 7);
                            aL = aF + theO.Real(0.5, 6);
                        }
                    }
                    double aU1 = theO.Real(aF, aF + 0.45 * (aL - aF)), aU2 = theO.Real(aF + 0.55 * (aL - aF), aL);
                    return (new N.Geom_TrimmedCurve(aNB, aU1, aU2), new TrimmedCurve3D(aPB, aU1, aU2), aRange);
                }
            }
        }
    }

    // ---------------------------------------------------------------------------------------------------------
    // Surfaces

    internal static (N.Geom_BSplineSurface Native, BSplineSurface Port) RandomBSplineSurface(
        Oracle theO, int theSmoothness = 0, bool theAllowPeriodic = true, int theMinDegree = 1)
    {
        int aUDeg = theO.Int(theMinDegree, 5), aVDeg = theO.Int(theMinDegree, 5);
        bool aUPer = theAllowPeriodic && theO.Int(0, 3) == 1, aVPer = theAllowPeriodic && theO.Int(0, 3) == 1;
        bool aRational = theO.Bool();
        theO.BSplineKnots(aUDeg, aUPer, out NCollection_Array1<double> aUKnots, out NCollection_Array1<int> aUMults);
        theO.BSplineKnots(aVDeg, aVPer, out NCollection_Array1<double> aVKnots, out NCollection_Array1<int> aVMults);
        ClampMults(aUMults, aUDeg, aUPer, theSmoothness);
        ClampMults(aVMults, aVDeg, aVPer, theSmoothness);
        int aNbU = Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(aUDeg, aUPer, aUMults);
        int aNbV = Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(aVDeg, aVPer, aVMults);
        var aPoles = new NCollection_Array2<Point3D>(1, aNbU, 1, aNbV);
        var aWeights = new NCollection_Array2<double>(1, aNbU, 1, aNbV);
        for (int i = 1; i <= aNbU; i++)
        {
            for (int j = 1; j <= aNbV; j++)
            {
                // a gently curved sheet, so that its offsets are regular
                aPoles[i, j] = new Point3D(i * 2.0 + theO.Real(-0.5, 0.5), j * 2.0 + theO.Real(-0.5, 0.5), theO.Real(-1, 1));
                aWeights[i, j] = aRational ? theO.Real(0.7, 1.3) : 1.0;
            }
        }
        return MakeBSplineSurface(aPoles, aRational ? aWeights : null, aUKnots, aVKnots, aUMults, aVMults, aUDeg, aVDeg, aUPer, aVPer);
    }

    // Lowers the inner multiplicities (all of them if periodic) to Degree - theSmoothness, at least 1.
    private static void ClampMults(NCollection_Array1<int> theMults, int theDeg, bool thePeriodic, int theSmoothness)
    {
        int aMax = Math.Max(1, theDeg - theSmoothness);
        for (int i = theMults.Lower(); i <= theMults.Upper(); i++)
        {
            if (thePeriodic || (i != theMults.Lower() && i != theMults.Upper()))
            {
                theMults[i] = Math.Min(theMults[i], aMax);
            }
        }
        if (thePeriodic)
        {
            theMults[theMults.Upper()] = theMults[theMults.Lower()];
        }
    }

    internal static (N.Geom_BSplineSurface Native, BSplineSurface Port) MakeBSplineSurface(
        NCollection_Array2<Point3D> thePoles, NCollection_Array2<double>? theWeights, NCollection_Array1<double> theUKnots,
        NCollection_Array1<double> theVKnots, NCollection_Array1<int> theUMults, NCollection_Array1<int> theVMults, int theUDeg,
        int theVDeg, bool theUPer, bool theVPer)
    {
        using N.TColgp_Array2OfPnt aNPoles = Oracle.ToNative(thePoles);
        using N.TColStd_Array1OfReal aNUKnots = Oracle.ToNative(theUKnots);
        using N.TColStd_Array1OfReal aNVKnots = Oracle.ToNative(theVKnots);
        using N.TColStd_Array1OfInteger aNUMults = Oracle.ToNative(theUMults);
        using N.TColStd_Array1OfInteger aNVMults = Oracle.ToNative(theVMults);
        if (theWeights is null)
        {
            return (new N.Geom_BSplineSurface(aNPoles, aNUKnots, aNVKnots, aNUMults, aNVMults, theUDeg, theVDeg, theUPer, theVPer),
                    new BSplineSurface(thePoles, theUKnots, theVKnots, theUMults, theVMults, theUDeg, theVDeg, theUPer, theVPer));
        }
        using N.TColStd_Array2OfReal aNWeights = Oracle.ToNative(theWeights);
        return (new N.Geom_BSplineSurface(aNPoles, aNWeights, aNUKnots, aNVKnots, aNUMults, aNVMults, theUDeg, theVDeg, theUPer, theVPer),
                new BSplineSurface(thePoles, theWeights, theUKnots, theVKnots, theUMults, theVMults, theUDeg, theVDeg, theUPer, theVPer));
    }

    internal static (N.Geom_BezierSurface Native, BezierSurface Port) RandomBezierSurface(Oracle theO)
    {
        int aNbU = theO.Int(2, 7), aNbV = theO.Int(2, 7);
        bool aRational = theO.Bool();
        var aPoles = new NCollection_Array2<Point3D>(1, aNbU, 1, aNbV);
        var aWeights = new NCollection_Array2<double>(1, aNbU, 1, aNbV);
        for (int i = 1; i <= aNbU; i++)
        {
            for (int j = 1; j <= aNbV; j++)
            {
                aPoles[i, j] = new Point3D(i * 2.0 + theO.Real(-0.5, 0.5), j * 2.0 + theO.Real(-0.5, 0.5), theO.Real(-1, 1));
                aWeights[i, j] = aRational ? theO.Real(0.7, 1.3) : 1.0;
            }
        }
        return MakeBezierSurface(aPoles, aRational ? aWeights : null);
    }

    internal static (N.Geom_BezierSurface Native, BezierSurface Port) MakeBezierSurface(NCollection_Array2<Point3D> thePoles,
                                                                                          NCollection_Array2<double>? theWeights)
    {
        using N.TColgp_Array2OfPnt aNPoles = Oracle.ToNative(thePoles);
        if (theWeights is null)
        {
            return (new N.Geom_BezierSurface(aNPoles), new BezierSurface(thePoles));
        }
        using N.TColStd_Array2OfReal aNWeights = Oracle.ToNative(theWeights);
        return (new N.Geom_BezierSurface(aNPoles, aNWeights), new BezierSurface(thePoles, theWeights));
    }

    /// <summary>The names of <see cref="RandomSurface"/>'s kinds.</summary>
    internal static readonly string[] SurfaceKinds =
    [
        "plane", "cylinder", "cone", "sphere", "torus", "revolution", "extrusion", "Bezier", "B-spline", "offset",
        "trimmed"
    ];

    /// <summary>A random surface of the given kind (<see cref="SurfaceKinds"/>).</summary>
    internal static (N.Geom_Surface Native, Surface Port) RandomSurface(Oracle theO, int theKind)
    {
        switch (theKind)
        {
            case 0:
            case 1:
            case 2:
            case 3:
            case 4:
            {
                (N.gp_Ax3 aNAx3, CoordinateSystem anAx3) = Ax3s(theO);
                using var aNAx3Owner = aNAx3;
                switch (theKind)
                {
                    case 0:
                        return (new N.Geom_Plane(aNAx3), new PlaneSurface(anAx3));
                    case 1:
                    {
                        double R = theO.Real(0.5, 20);
                        return (new N.Geom_CylindricalSurface(aNAx3, R), new CylindricalSurface(anAx3, R));
                    }
                    case 2:
                    {
                        double anAng = theO.Real(0.05, 1.4) * (theO.Bool() ? 1 : -1);
                        double R = theO.Int(0, 4) == 0 ? 0.0 : theO.Real(0.5, 20);
                        return (new N.Geom_ConicalSurface(aNAx3, anAng, R), new ConicalSurface(anAx3, anAng, R));
                    }
                    case 3:
                    {
                        double R = theO.Real(0.5, 20);
                        return (new N.Geom_SphericalSurface(aNAx3, R), new SphericalSurface(anAx3, R));
                    }
                    default:
                    {
                        double r = theO.Real(0.5, 5);
                        double R = theO.Real(r * 1.1, 20);
                        return (new N.Geom_ToroidalSurface(aNAx3, R, r), new ToroidalSurface(anAx3, R, r));
                    }
                }
            }
            case 5:
            case 6:
            {
                // a circle, a line, or a B-spline curve (smooth enough for an offset half the time)
                N.Geom_Curve aNC;
                Curve3D aPC;
                switch (theO.Int(0, 3))
                {
                    case 0:
                        (aNC, aPC, _) = RandomCurve(theO, 1);
                        break;
                    case 1:
                        (aNC, aPC, _) = RandomCurve(theO, 0);
                        break;
                    default:
                        (aNC, aPC) = RandomBSplineCurve(theO, null, 2, 5, theO.Int(0, 2));
                        break;
                }
                using (aNC)
                {
                    if (theKind == 5)
                    {
                        (N.gp_Ax1 aNAxis, Axis3D anAxis) = Ax1s(theO);
                        using var aNAxisOwner = aNAxis;
                        return (new N.Geom_SurfaceOfRevolution(aNC, aNAxis), new RevolutionSurface(aPC, anAxis));
                    }
                    (N.gp_Dir aNDir, Direction3D aDir) = Dirs(theO);
                    using var aNDirOwner = aNDir;
                    return (new N.Geom_SurfaceOfLinearExtrusion(aNC, aNDir), new ExtrusionSurface(aPC, aDir));
                }
            }
            case 7:
                return RandomBezierSurface(theO);
            case 8:
                return RandomBSplineSurface(theO, theO.Int(0, 2));
            case 9:
                return RandomOffsetSurface(theO);
            default:
            {
                while (true)
                {
                    (N.Geom_Surface aNB, Surface aPB) = RandomSurface(theO, theO.Int(0, 9));
                    using (aNB)
                    {
                        (N.Geom_Surface? aNT, Surface? aPT) = RandomTrim(theO, aNB, aPB);
                        if (aNT is not null)
                        {
                            return (aNT, aPT!);
                        }
                    }
                }
            }
        }
    }

    /// <summary>A rectangular trimmed surface of the given one, trimmed in both directions or in one.</summary>
    /// <summary>
    /// A rectangular trimmed surface of the given one, trimmed in both directions or in one; null on both sides
    /// when both constructions throw (the outcomes must agree).
    /// </summary>
    internal static (N.Geom_Surface? Native, Surface? Port) RandomTrim(Oracle theO, N.Geom_Surface theN, Surface theP)
    {
        theP.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
        (double aUF, double aUL) = TrimRange(theO, aU1, aU2, theP.IsUPeriodic);
        (double aVF, double aVL) = TrimRange(theO, aV1, aV2, theP.IsVPeriodic);
        int aMode = theO.Int(0, 2);
        N.Geom_Surface? aN = null;
        Surface? aP = null;
        Exception? aNErr = Catch(() => aN = aMode switch
        {
            0 => new N.Geom_RectangularTrimmedSurface(theN, aUF, aUL, true),
            1 => new N.Geom_RectangularTrimmedSurface(theN, aVF, aVL, false),
            _ => new N.Geom_RectangularTrimmedSurface(theN, aUF, aUL, aVF, aVL),
        });
        Exception? aPErr = Catch(() => aP = aMode switch
        {
            0 => new TrimmedSurface(theP, aUF, aUL, true),
            1 => new TrimmedSurface(theP, aVF, aVL, false),
            _ => new TrimmedSurface(theP, aUF, aUL, aVF, aVL),
        });
        SameOutcome(aNErr, aPErr, "Geom_RectangularTrimmedSurface construction");
        return (aN, aP);
    }

    private static (double, double) TrimRange(Oracle theO, double theFirst, double theLast, bool thePeriodic)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(theFirst) || global::Kerf.Engine.Foundation.Precision.IsInfinite(theLast))
        {
            double aF = theO.Real(-15, 0);
            return (aF, aF + theO.Real(1, 15));
        }
        double aD = theLast - theFirst;
        if (thePeriodic)
        {
            double aF = theO.Real(theFirst - aD, theLast);
            return (aF, aF + theO.Real(0.2, 0.95) * aD);
        }
        return (theO.Real(theFirst, theFirst + 0.4 * aD), theO.Real(theFirst + 0.6 * aD, theLast));
    }

    /// <summary>
    /// A random offset surface: of an elementary surface (with or without an equivalent surface, positive and
    /// negative offsets, degenerate radii), of a swept, Bezier or B-spline surface, of a trimmed one, of an offset;
    /// built with isNotCheckC0 now and then. Construction is retried while it throws on both sides (a C0 basis).
    /// </summary>
    internal static (N.Geom_Surface Native, Surface Port) RandomOffsetSurface(Oracle theO)
    {
        while (true)
        {
            int aKind = theO.Int(0, 10);
            N.Geom_Surface aNB;
            Surface aPB;
            if (aKind == 9)
            {
                (N.Geom_Surface aN0, Surface aP0) = RandomSurface(theO, theO.Int(0, 8));
                using (aN0)
                {
                    (N.Geom_Surface? aNT, Surface? aPT) = RandomTrim(theO, aN0, aP0);
                    if (aNT is null)
                    {
                        continue;
                    }
                    (aNB, aPB) = (aNT, aPT!);
                }
            }
            else if (aKind == 10)
            {
                (aNB, aPB) = RandomBasisForNestedOffset(theO);
            }
            else
            {
                (aNB, aPB) = RandomSurface(theO, aKind);
            }
            using (aNB)
            {
                double anOffset = theO.Int(0, 9) == 0 ? 0.0 : theO.Real(0.05, 4) * (theO.Bool() ? 1 : -1);
                if (aPB is CylindricalSurface aCyl && theO.Int(0, 4) == 0)
                {
                    // the equivalent cylinder flips or degenerates
                    anOffset = -aCyl.Radius() * (theO.Bool() ? 1 : 2) * (aCyl.Position().Direct() ? 1 : -1);
                }
                bool aNotCheck = theO.Int(0, 4) == 0;
                N.Geom_Surface? aN = null;
                Surface? aP = null;
                Exception? aNErr = Catch(() => aN = new N.Geom_OffsetSurface(aNB, anOffset, aNotCheck));
                Exception? aPErr = Catch(() => aP = new OffsetSurface(aPB, anOffset, aNotCheck));
                if (SameOutcome(aNErr, aPErr, "Geom_OffsetSurface construction"))
                {
                    return (aN!, aP!);
                }
            }
        }
    }

    private static (N.Geom_Surface Native, Surface Port) RandomBasisForNestedOffset(Oracle theO)
    {
        while (true)
        {
            (N.Geom_Surface aNB, Surface aPB) = RandomSurface(theO, theO.Int(0, 8));
            using (aNB)
            {
                double anOffset = theO.Real(-2, 2);
                N.Geom_Surface? aN = null;
                Surface? aP = null;
                Exception? aNErr = Catch(() => aN = new N.Geom_OffsetSurface(aNB, anOffset, true));
                Exception? aPErr = Catch(() => aP = new OffsetSurface(aPB, anOffset, true));
                if (SameOutcome(aNErr, aPErr, "inner Geom_OffsetSurface construction"))
                {
                    return (aN!, aP!);
                }
            }
        }
    }

    // ---------------------------------------------------------------------------------------------------------
    // Evaluation through Adaptor3d

    /// <summary>D0 to D3 and DN (orders 1 to theMaxN) of two curve adaptors at theU; each throws on both sides or on neither.</summary>
    /// <remarks>
    /// Plan 3.2's third level for an offset curve (occt-issues.md, the offset curves' CalculateD3 and EvaluateDN; Kerf
    /// decision 1487): the port's D3 is compared with its own D2 (V2) and its DN(3) with D3's V3, native's values read
    /// as recorded; native's DN(3) could throw (its basis D1 overwritten by D2), the port's doesn't.
    /// </remarks>
    internal static void SameCurveEval(Oracle theO, N.Adaptor3d_Curve theN, CurveAdaptorBase theP, double theU, int theMaxN,
                                       string theWhat, double theTol = Oracle.Tolerance)
    {
        bool anOffset = theP.GetType() == CurveKind.OffsetCurve;
        Vector3D aD2V2 = new();
        using var aNP = new N.gp_Pnt(0, 0, 0);
        using var aNV1 = new N.gp_Vec(0, 0, 0);
        using var aNV2 = new N.gp_Vec(0, 0, 0);
        using var aNV3 = new N.gp_Vec(0, 0, 0);
        Point3D P = new();
        Vector3D V1 = new(), V2 = new(), V3 = new();
        if (SameOutcome(Catch(() => theN.D0(theU, aNP)), Catch(() => theP.D0(theU, out P)), theWhat + " D0"))
        {
            theO.Same(aNP, P, theWhat + " D0", theTol);
        }
        if (SameOutcome(Catch(() => theN.D1(theU, aNP, aNV1)), Catch(() => theP.D1(theU, out P, out V1)), theWhat + " D1"))
        {
            theO.Same(aNP, P, theWhat + " D1 P", theTol);
            theO.Same(aNV1, V1, theWhat + " D1", theTol);
        }
        if (SameOutcome(Catch(() => theN.D2(theU, aNP, aNV1, aNV2)), Catch(() => theP.D2(theU, out P, out V1, out V2)),
                        theWhat + " D2"))
        {
            theO.Same(aNP, P, theWhat + " D2 P", theTol);
            theO.Same(aNV1, V1, theWhat + " D2 V1", theTol);
            theO.Same(aNV2, V2, theWhat + " D2", theTol);
            aD2V2 = V2;
        }
        bool aD3Done = false;
        if (SameOutcome(Catch(() => theN.D3(theU, aNP, aNV1, aNV2, aNV3)),
                        Catch(() => theP.D3(theU, out P, out V1, out V2, out V3)), theWhat + " D3"))
        {
            aD3Done = true;
            theO.Same(aNP, P, theWhat + " D3 P", theTol);
            theO.Same(aNV1, V1, theWhat + " D3 V1", theTol);
            if (anOffset)
            {
                _ = (aNV2.X(), aNV2.Y(), aNV2.Z(), aNV3.X(), aNV3.Y(), aNV3.Z());
                theO.Same(aD2V2.XYZ(), V2.XYZ(), theWhat + " D3 V2 against D2's", theTol);
            }
            else
            {
                theO.Same(aNV2, V2, theWhat + " D3 V2", theTol);
                theO.Same(aNV3, V3, theWhat + " D3", theTol);
            }
        }
        for (int n = 1; n <= theMaxN; n++)
        {
            N.gp_Vec? aNDN = null;
            Vector3D aDN = new();
            int aN = n;
            if (anOffset && n == 3)
            {
                Exception? aNE = Catch(() => aNDN = theN.DN(theU, aN));
                Exception? aPE = Catch(() => aDN = theP.DN(theU, aN));
                Assert.True(aPE is null || aNE is not null, $"{theWhat} DN(3): the port threw where native didn't");
                using (aNDN)
                {
                    if (aNDN is not null)
                    {
                        _ = (aNDN.X(), aNDN.Y(), aNDN.Z());
                    }
                }
                if (aPE is null && aD3Done)
                {
                    theO.Same(V3.XYZ(), aDN.XYZ(), $"{theWhat} DN(3) against D3's", theTol);
                }
                continue;
            }
            if (SameOutcome(Catch(() => aNDN = theN.DN(theU, aN)), Catch(() => aDN = theP.DN(theU, aN)), $"{theWhat} DN({n})"))
            {
                using (aNDN)
                {
                    theO.Same(aNDN!, aDN, $"{theWhat} DN({n})", theTol);
                }
            }
        }
    }

    /// <summary>
    /// D0 to D3 and DN (Nu + Nv from 1 to theMaxN) of two surface adaptors (or surfaces, through
    /// <see cref="SameSurfaceEval(Oracle, N.Geom_Surface, Surface, double, double, int, string, double)"/>)
    /// at (U, V); each throws on both sides or on neither.
    /// </summary>
    internal static void SameSurfaceEval(Oracle theO, N.Adaptor3d_Surface theN, SurfaceAdaptorBase theP, double U, double V,
                                         int theMaxN, string theWhat, double theTol = Oracle.Tolerance)
    {
        SameSurfaceEval(theO,
                        (P, DU, DV) => theN.D1(U, V, P, DU, DV),
                        (P, DU, DV, DUU, DVV, DUV) => theN.D2(U, V, P, DU, DV, DUU, DVV, DUV),
                        (P, DU, DV, DUU, DVV, DUV, DUUU, DVVV, DUUV, DUVV) => theN.D3(U, V, P, DU, DV, DUU, DVV, DUV, DUUU, DVVV, DUUV, DUVV),
                        (nu, nv) => theN.DN(U, V, nu, nv),
                        P => theN.D0(U, V, P),
                        () => theP.EvalD0(U, V), () => theP.EvalD1(U, V), () => theP.EvalD2(U, V), () => theP.EvalD3(U, V),
                        (nu, nv) => theP.DN(U, V, nu, nv), theMaxN, theWhat, theTol);
    }

    /// <summary>As the adaptor form, for two surfaces.</summary>
    internal static void SameSurfaceEval(Oracle theO, N.Geom_Surface theN, Surface theP, double U, double V,
                                         int theMaxN, string theWhat, double theTol = Oracle.Tolerance)
    {
        SameSurfaceEval(theO,
                        (P, DU, DV) => theN.D1(U, V, P, DU, DV),
                        (P, DU, DV, DUU, DVV, DUV) => theN.D2(U, V, P, DU, DV, DUU, DVV, DUV),
                        (P, DU, DV, DUU, DVV, DUV, DUUU, DVVV, DUUV, DUVV) => theN.D3(U, V, P, DU, DV, DUU, DVV, DUV, DUUU, DVVV, DUUV, DUVV),
                        (nu, nv) => theN.DN(U, V, nu, nv),
                        P => theN.D0(U, V, P),
                        () => theP.EvalD0(U, V), () => theP.EvalD1(U, V), () => theP.EvalD2(U, V), () => theP.EvalD3(U, V),
                        (nu, nv) => theP.DN(U, V, nu, nv), theMaxN, theWhat, theTol);
    }

    private static void SameSurfaceEval(Oracle theO,
                                        Action<N.gp_Pnt, N.gp_Vec, N.gp_Vec> theND1,
                                        Action<N.gp_Pnt, N.gp_Vec, N.gp_Vec, N.gp_Vec, N.gp_Vec, N.gp_Vec> theND2,
                                        Action<N.gp_Pnt, N.gp_Vec, N.gp_Vec, N.gp_Vec, N.gp_Vec, N.gp_Vec, N.gp_Vec, N.gp_Vec, N.gp_Vec, N.gp_Vec> theND3,
                                        Func<int, int, N.gp_Vec> theNDN,
                                        Action<N.gp_Pnt> theND0,
                                        Func<Point3D> thePD0, Func<Surface.ResD1> thePD1, Func<Surface.ResD2> thePD2,
                                        Func<Surface.ResD3> thePD3, Func<int, int, Vector3D> thePDN, int theMaxN, string theWhat,
                                        double theTol)
    {
        using var P = new N.gp_Pnt(0, 0, 0);
        using var DU = new N.gp_Vec(0, 0, 0);
        using var DV = new N.gp_Vec(0, 0, 0);
        using var DUU = new N.gp_Vec(0, 0, 0);
        using var DVV = new N.gp_Vec(0, 0, 0);
        using var DUV = new N.gp_Vec(0, 0, 0);
        using var DUUU = new N.gp_Vec(0, 0, 0);
        using var DVVV = new N.gp_Vec(0, 0, 0);
        using var DUUV = new N.gp_Vec(0, 0, 0);
        using var DUVV = new N.gp_Vec(0, 0, 0);

        Point3D aP0 = new();
        if (SameOutcome(Catch(() => theND0(P)), Catch(() => aP0 = thePD0()), theWhat + " D0"))
        {
            theO.Same(P, aP0, theWhat + " D0", theTol);
        }

        Surface.ResD1 aR1 = default;
        if (SameOutcome(Catch(() => theND1(P, DU, DV)), Catch(() => aR1 = thePD1()), theWhat + " D1"))
        {
            theO.Same(P, aR1.Point, theWhat + " D1 P", theTol);
            theO.Same(DU, aR1.D1U, theWhat + " D1U", theTol);
            theO.Same(DV, aR1.D1V, theWhat + " D1V", theTol);
        }

        Surface.ResD2 aR2 = default;
        if (SameOutcome(Catch(() => theND2(P, DU, DV, DUU, DVV, DUV)), Catch(() => aR2 = thePD2()), theWhat + " D2"))
        {
            theO.Same(P, aR2.Point, theWhat + " D2 P", theTol);
            theO.Same(DU, aR2.D1U, theWhat + " D2 D1U", theTol);
            theO.Same(DV, aR2.D1V, theWhat + " D2 D1V", theTol);
            theO.Same(DUU, aR2.D2U, theWhat + " D2U", theTol);
            theO.Same(DVV, aR2.D2V, theWhat + " D2V", theTol);
            theO.Same(DUV, aR2.D2UV, theWhat + " D2UV", theTol);
        }

        Surface.ResD3 aR3 = default;
        if (SameOutcome(Catch(() => theND3(P, DU, DV, DUU, DVV, DUV, DUUU, DVVV, DUUV, DUVV)), Catch(() => aR3 = thePD3()),
                        theWhat + " D3"))
        {
            theO.Same(P, aR3.Point, theWhat + " D3 P", theTol);
            theO.Same(DU, aR3.D1U, theWhat + " D3 D1U", theTol);
            theO.Same(DV, aR3.D1V, theWhat + " D3 D1V", theTol);
            theO.Same(DUU, aR3.D2U, theWhat + " D3 D2U", theTol);
            theO.Same(DVV, aR3.D2V, theWhat + " D3 D2V", theTol);
            theO.Same(DUV, aR3.D2UV, theWhat + " D3 D2UV", theTol);
            theO.Same(DUUU, aR3.D3U, theWhat + " D3U", theTol);
            theO.Same(DVVV, aR3.D3V, theWhat + " D3V", theTol);
            theO.Same(DUUV, aR3.D3UUV, theWhat + " D3UUV", theTol);
            theO.Same(DUVV, aR3.D3UVV, theWhat + " D3UVV", theTol);
        }

        for (int n = 1; n <= theMaxN; n++)
        {
            for (int nu = 0; nu <= n; nu++)
            {
                int aNu = nu, aNv = n - nu;
                N.gp_Vec? aNDN = null;
                Vector3D aDN = new();
                if (SameOutcome(Catch(() => aNDN = theNDN(aNu, aNv)), Catch(() => aDN = thePDN(aNu, aNv)), $"{theWhat} DN({aNu}, {aNv})"))
                {
                    using (aNDN)
                    {
                        theO.Same(aNDN!, aDN, $"{theWhat} DN({aNu}, {aNv})", theTol);
                    }
                }
            }
        }
    }

    /// <summary>The class name of a native surface (the wrappers have no DynamicType): the first DownCast that succeeds.</summary>
    internal static string NativeSurfaceKind(N.Geom_Surface theN)
    {
        (Func<N.Geom_Surface, IDisposable?> Cast, string Name)[] aCasts =
        [
            (s => N.Geom_Plane.DownCast(s), global::Kerf.Engine.Foundation.OcctTypeNames.Of(typeof(PlaneSurface))),
            (s => N.Geom_CylindricalSurface.DownCast(s), global::Kerf.Engine.Foundation.OcctTypeNames.Of(typeof(CylindricalSurface))),
            (s => N.Geom_ConicalSurface.DownCast(s), global::Kerf.Engine.Foundation.OcctTypeNames.Of(typeof(ConicalSurface))),
            (s => N.Geom_SphericalSurface.DownCast(s), global::Kerf.Engine.Foundation.OcctTypeNames.Of(typeof(SphericalSurface))),
            (s => N.Geom_ToroidalSurface.DownCast(s), global::Kerf.Engine.Foundation.OcctTypeNames.Of(typeof(ToroidalSurface))),
            (s => N.Geom_RectangularTrimmedSurface.DownCast(s), global::Kerf.Engine.Foundation.OcctTypeNames.Of(typeof(TrimmedSurface))),
            (s => N.Geom_OffsetSurface.DownCast(s), global::Kerf.Engine.Foundation.OcctTypeNames.Of(typeof(OffsetSurface))),
            (s => N.Geom_SurfaceOfRevolution.DownCast(s), global::Kerf.Engine.Foundation.OcctTypeNames.Of(typeof(RevolutionSurface))),
            (s => N.Geom_SurfaceOfLinearExtrusion.DownCast(s), global::Kerf.Engine.Foundation.OcctTypeNames.Of(typeof(ExtrusionSurface))),
            (s => N.Geom_BezierSurface.DownCast(s), global::Kerf.Engine.Foundation.OcctTypeNames.Of(typeof(BezierSurface))),
            (s => N.Geom_BSplineSurface.DownCast(s), global::Kerf.Engine.Foundation.OcctTypeNames.Of(typeof(BSplineSurface))),
        ];
        foreach ((Func<N.Geom_Surface, IDisposable?> aCast, string aName) in aCasts)
        {
            using IDisposable? aC = aCast(theN);
            if (aC is not null)
            {
                return aName;
            }
        }
        return "unknown";
    }

    // ---------------------------------------------------------------------------------------------------------
    // gp values

    internal static void SameDir(Oracle theO, N.gp_Dir theN, in Direction3D theP, string theWhat)
    {
        theO.Same(theN.X(), theP.X(), theWhat + " X");
        theO.Same(theN.Y(), theP.Y(), theWhat + " Y");
        theO.Same(theN.Z(), theP.Z(), theWhat + " Z");
    }

    internal static void SameAx1(Oracle theO, N.gp_Ax1 theN, in Axis3D theP, string theWhat)
    {
        using N.gp_Pnt aLoc = theN.Location();
        using N.gp_Dir aDir = theN.Direction();
        theO.Same(aLoc, theP.Location(), theWhat + " location");
        SameDir(theO, aDir, theP.Direction(), theWhat + " direction");
    }

    internal static void SameAx2(Oracle theO, N.gp_Ax2 theN, in Frame3D theP, string theWhat)
    {
        using N.gp_Pnt aLoc = theN.Location();
        using N.gp_Dir aDir = theN.Direction();
        using N.gp_Dir aXDir = theN.XDirection();
        using N.gp_Dir aYDir = theN.YDirection();
        theO.Same(aLoc, theP.Location(), theWhat + " location");
        SameDir(theO, aDir, theP.Direction(), theWhat + " direction");
        SameDir(theO, aXDir, theP.XDirection(), theWhat + " X direction");
        SameDir(theO, aYDir, theP.YDirection(), theWhat + " Y direction");
    }

    internal static void SameAx3(Oracle theO, N.gp_Ax3 theN, in CoordinateSystem theP, string theWhat)
    {
        using N.gp_Pnt aLoc = theN.Location();
        using N.gp_Dir aDir = theN.Direction();
        using N.gp_Dir aXDir = theN.XDirection();
        using N.gp_Dir aYDir = theN.YDirection();
        theO.Same(aLoc, theP.Location(), theWhat + " location");
        SameDir(theO, aDir, theP.Direction(), theWhat + " direction");
        SameDir(theO, aXDir, theP.XDirection(), theWhat + " X direction");
        SameDir(theO, aYDir, theP.YDirection(), theWhat + " Y direction");
    }

    /// <summary>A parameter in [theFirst, theLast], or in [-theRange, theRange] where a bound is infinite.</summary>
    internal static double Param(Oracle theO, double theFirst, double theLast, double theRange = 20)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(theFirst) || global::Kerf.Engine.Foundation.Precision.IsInfinite(theLast))
        {
            double aF = global::Kerf.Engine.Foundation.Precision.IsInfinite(theFirst) ? -theRange : theFirst;
            double aL = global::Kerf.Engine.Foundation.Precision.IsInfinite(theLast) ? theRange : theLast;
            if (aF > aL)
            {
                aF = aL - theRange;
            }
            return theO.Real(aF, aL);
        }
        return theO.Real(theFirst, theLast);
    }
}
