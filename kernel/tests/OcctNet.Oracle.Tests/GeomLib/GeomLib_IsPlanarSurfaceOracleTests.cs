// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Library;
using Kerf.Geometry;
using Xunit.Abstractions;
using static OcctNet.Oracle.Tests.GeomOracleInputs;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// GeomLib_IsPlanarSurface against native OCCT on every surface kind, built on both sides from the same raw numbers:
/// elementary surfaces, planar and nearly planar B-spline and Bezier surfaces (noise from 0 to 1e-2 off a random
/// plane), random B-splines, revolutions of segments perpendicular (or nearly) to the axis and of circles, extrusions
/// of planar curves along a direction in their plane (or tilted), trimmed and offset surfaces; OCCT's default
/// tolerance or one from 1e-9 to 1e-1. IsPlanar and the plane's position are compared.
/// </summary>
public class GeomLib_IsPlanarSurfaceOracleTests(ITestOutputHelper output)
{
    private const int Cases = 2000;

    private static readonly double[] Noises = [0.0, 0.0, 1e-12, 1e-10, 1e-8, 1e-6, 1e-4, 1e-2];
    private static readonly double[] Tolerances = [1e-9, 1e-7, 1e-5, 1e-3, 1e-1];

    private static void SameDir(Oracle theOracle, N.gp_Dir theN, in Direction3D theP, string theWhat)
    {
        theOracle.Same(theN.X(), theP.X(), theWhat + " X");
        theOracle.Same(theN.Y(), theP.Y(), theWhat + " Y");
        theOracle.Same(theN.Z(), theP.Z(), theWhat + " Z");
    }

    /// <summary>A point of the plane of theAx3 at (a, b), moved off it by theNoise times a random amount.</summary>
    private static Point3D PlanePoint(Oracle theOracle, in CoordinateSystem theAx3, double a, double b, double theNoise)
    {
        Xyz aP = theAx3.Location().XYZ() + a * theAx3.XDirection().XYZ() + b * theAx3.YDirection().XYZ()
                    + theNoise * theOracle.Real(-1, 1) * theAx3.Direction().XYZ();
        return new Point3D(aP);
    }

    private static (N.Geom_Surface Native, Surface Port) PlanarBSpline(Oracle theOracle, double theNoise,
                                                                           bool theBezier)
    {
        (N.gp_Ax3 aNAx3, CoordinateSystem anAx3) = Ax3s(theOracle);
        aNAx3.Dispose();
        int aUDeg = theOracle.Int(1, 4), aVDeg = theOracle.Int(1, 4);
        NCollection_Array1<double> aUKnots, aVKnots;
        NCollection_Array1<int> aUMults, aVMults;
        int aNbU, aNbV;
        if (theBezier)
        {
            aUKnots = aVKnots = new NCollection_Array1<double>(1, 1);
            aUMults = aVMults = new NCollection_Array1<int>(1, 1);
            aNbU = aUDeg + 1;
            aNbV = aVDeg + 1;
        }
        else
        {
            theOracle.BSplineKnots(aUDeg, false, out aUKnots, out aUMults);
            theOracle.BSplineKnots(aVDeg, false, out aVKnots, out aVMults);
            aNbU = Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(aUDeg, false, aUMults);
            aNbV = Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(aVDeg, false, aVMults);
        }
        bool aRational = theOracle.Int(0, 2) == 0;
        double aScale = theOracle.Real(0.1, 20);
        var aPoles = new NCollection_Array2<Point3D>(1, aNbU, 1, aNbV);
        var aWeights = new NCollection_Array2<double>(1, aNbU, 1, aNbV);
        for (int i = 1; i <= aNbU; i++)
        {
            for (int j = 1; j <= aNbV; j++)
            {
                // A distorted grid, so that the poles aren't on a line
                double a = aScale * (i + theOracle.Real(-0.3, 0.3)), b = aScale * (j + theOracle.Real(-0.3, 0.3));
                aPoles[i, j] = PlanePoint(theOracle, anAx3, a, b, theNoise * aScale);
                aWeights[i, j] = aRational ? theOracle.Real(0.5, 1.5) : 1.0;
            }
        }
        using N.TColgp_Array2OfPnt aNPoles = Oracle.ToNative(aPoles);
        using N.TColStd_Array2OfReal aNWeights = Oracle.ToNative(aWeights);
        if (theBezier)
        {
            return aRational
                       ? (new N.Geom_BezierSurface(aNPoles, aNWeights), new BezierSurface(aPoles, aWeights))
                       : (new N.Geom_BezierSurface(aNPoles), new BezierSurface(aPoles));
        }
        using N.TColStd_Array1OfReal aNUKnots = Oracle.ToNative(aUKnots), aNVKnots = Oracle.ToNative(aVKnots);
        using N.TColStd_Array1OfInteger aNUMults = Oracle.ToNative(aUMults), aNVMults = Oracle.ToNative(aVMults);
        return (new N.Geom_BSplineSurface(aNPoles, aNWeights, aNUKnots, aNVKnots, aNUMults, aNVMults, aUDeg, aVDeg, false, false),
                new BSplineSurface(aPoles, aWeights, aUKnots, aVKnots, aUMults, aVMults, aUDeg, aVDeg, false, false));
    }

    private static (N.Geom_Surface Native, Surface Port) RandomBSpline(Oracle theOracle)
    {
        int aUDeg = theOracle.Int(1, 4), aVDeg = theOracle.Int(1, 4);
        bool aUPer = theOracle.Int(0, 3) == 0, aVPer = theOracle.Int(0, 3) == 0;
        theOracle.BSplineKnots(aUDeg, aUPer, out NCollection_Array1<double> aUKnots, out NCollection_Array1<int> aUMults);
        theOracle.BSplineKnots(aVDeg, aVPer, out NCollection_Array1<double> aVKnots, out NCollection_Array1<int> aVMults);
        int aNbU = Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(aUDeg, aUPer, aUMults), aNbV = Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(aVDeg, aVPer, aVMults);
        var aPoles = new NCollection_Array2<Point3D>(1, aNbU, 1, aNbV);
        var aWeights = new NCollection_Array2<double>(1, aNbU, 1, aNbV);
        for (int i = 1; i <= aNbU; i++)
        {
            for (int j = 1; j <= aNbV; j++)
            {
                aPoles[i, j] = new Point3D(theOracle.Real(-10, 10), theOracle.Real(-10, 10), theOracle.Real(-10, 10));
                aWeights[i, j] = theOracle.Real(0.5, 1.5);
            }
        }
        using N.TColgp_Array2OfPnt aNPoles = Oracle.ToNative(aPoles);
        using N.TColStd_Array2OfReal aNWeights = Oracle.ToNative(aWeights);
        using N.TColStd_Array1OfReal aNUKnots = Oracle.ToNative(aUKnots), aNVKnots = Oracle.ToNative(aVKnots);
        using N.TColStd_Array1OfInteger aNUMults = Oracle.ToNative(aUMults), aNVMults = Oracle.ToNative(aVMults);
        return (new N.Geom_BSplineSurface(aNPoles, aNWeights, aNUKnots, aNVKnots, aNUMults, aNVMults, aUDeg, aVDeg, aUPer, aVPer),
                new BSplineSurface(aPoles, aWeights, aUKnots, aVKnots, aUMults, aVMults, aUDeg, aVDeg, aUPer, aVPer));
    }

    private static (N.Geom_Surface Native, Surface Port) Elementary(Oracle theOracle)
    {
        (N.gp_Ax3 aNAx3, CoordinateSystem anAx3) = Ax3s(theOracle);
        using var aNAx3Owner = aNAx3;
        switch (theOracle.Int(0, 4))
        {
            case 0:
                return (new N.Geom_Plane(aNAx3), new PlaneSurface(anAx3));
            case 1:
            {
                double R = theOracle.Real(0.1, 20);
                return (new N.Geom_CylindricalSurface(aNAx3, R), new CylindricalSurface(anAx3, R));
            }
            case 2:
            {
                double anAng = theOracle.Real(0.05, 1.5);
                double R = theOracle.Real(0.1, 20);
                return (new N.Geom_ConicalSurface(aNAx3, anAng, R), new ConicalSurface(anAx3, anAng, R));
            }
            case 3:
            {
                double R = theOracle.Real(0.1, 20);
                return (new N.Geom_SphericalSurface(aNAx3, R), new SphericalSurface(anAx3, R));
            }
            default:
            {
                double r = theOracle.Real(0.1, 5);
                double R = theOracle.Real(r, 20);
                return (new N.Geom_ToroidalSurface(aNAx3, R, r), new ToroidalSurface(anAx3, R, r));
            }
        }
    }

    /// <summary>
    /// A revolution of a segment from the axis, perpendicular to it or tilted by a small angle (a cone), or of a
    /// circle.
    /// </summary>
    private static (N.Geom_Surface Native, Surface Port) Revolution(Oracle theOracle)
    {
        (N.gp_Ax1 aNAxis, Axis3D anAxis) = Ax1s(theOracle);
        using var aNAxisOwner = aNAxis;
        if (theOracle.Int(0, 4) == 0)
        {
            (N.gp_Ax2 aNAx2, Frame3D anAx2) = Ax2s(theOracle);
            using var aNAx2Owner = aNAx2;
            double R = theOracle.Real(0.1, 10);
            using var aNC = new N.Geom_Circle(aNAx2, R);
            return (new N.Geom_SurfaceOfRevolution(aNC, aNAxis), new RevolutionSurface(new CircleCurve3D(anAx2, R), anAxis));
        }
        // Raw direction perpendicular to the axis, tilted along it by Noise
        Xyz aPerp = anAxis.Direction().XYZ() ^ RawDir(theOracle);
        double aTilt = Noises[theOracle.Int(0, Noises.Length - 1)];
        Xyz aRaw = aPerp + aTilt * aPerp.Modulus() * anAxis.Direction().XYZ();
        Point3D aStart = anAxis.Location();
        if (theOracle.Int(0, 2) == 0)
        {
            aStart = new Point3D(aStart.XYZ() + theOracle.Real(-5, 5) * anAxis.Direction().XYZ());
        }
        using var aNDir = new N.gp_Dir(aRaw.X(), aRaw.Y(), aRaw.Z());
        using var aNStart = Oracle.ToNative(aStart);
        using var aNLine = new N.Geom_Line(aNStart, aNDir);
        var aLine = new LineCurve3D(aStart, new Direction3D(aRaw.X(), aRaw.Y(), aRaw.Z()));
        double U1 = theOracle.Real(0, 5), U2 = U1 + theOracle.Real(0.5, 10);
        using var aNTrim = new N.Geom_TrimmedCurve(aNLine, U1, U2);
        return (new N.Geom_SurfaceOfRevolution(aNTrim, aNAxis),
                new RevolutionSurface(new TrimmedCurve3D(aLine, U1, U2), anAxis));
    }

    /// <summary>
    /// An extrusion of a planar B-spline curve (or a segment) along a direction in its plane, tilted by a noise; or of
    /// a curve with noise off the plane.
    /// </summary>
    private static (N.Geom_Surface Native, Surface Port) Extrusion(Oracle theOracle)
    {
        (N.gp_Ax3 aNAx3, CoordinateSystem anAx3) = Ax3s(theOracle);
        aNAx3.Dispose();
        double aCurveNoise = Noises[theOracle.Int(0, Noises.Length - 1)];
        double aTilt = Noises[theOracle.Int(0, Noises.Length - 1)];
        N.Geom_Curve aNC;
        Curve3D aPC;
        if (theOracle.Int(0, 3) == 0)
        {
            Point3D aP = PlanePoint(theOracle, anAx3, 0, 0, 0);
            Xyz aRaw = theOracle.Real(-1, 1) * anAx3.XDirection().XYZ() + anAx3.YDirection().XYZ();
            using var aNP = Oracle.ToNative(aP);
            using var aNDir = new N.gp_Dir(aRaw.X(), aRaw.Y(), aRaw.Z());
            using var aNLine = new N.Geom_Line(aNP, aNDir);
            var aLine = new LineCurve3D(aP, new Direction3D(aRaw.X(), aRaw.Y(), aRaw.Z()));
            double U1 = theOracle.Real(-5, 0), U2 = U1 + theOracle.Real(0.5, 10);
            aNC = new N.Geom_TrimmedCurve(aNLine, U1, U2);
            aPC = new TrimmedCurve3D(aLine, U1, U2);
        }
        else
        {
            int aDeg = theOracle.Int(1, 5);
            theOracle.BSplineKnots(aDeg, false, out NCollection_Array1<double> aKnots, out NCollection_Array1<int> aMults);
            int aNbPoles = Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(aDeg, false, aMults);
            var aPoles = new NCollection_Array1<Point3D>(1, aNbPoles);
            for (int i = 1; i <= aNbPoles; i++)
            {
                aPoles[i] = PlanePoint(theOracle, anAx3, theOracle.Real(-10, 10), theOracle.Real(-10, 10), aCurveNoise);
            }
            NCollection_Array1<double>? aWeights = theOracle.Int(0, 1) == 0 ? null : theOracle.Weights(aNbPoles, true);
            aNC = Oracle.ToNativeCurve(aPoles, aWeights, aKnots, aMults, aDeg, false);
            aPC = aWeights is null ? new BSplineCurve3D(aPoles, aKnots, aMults, aDeg)
                                   : new BSplineCurve3D(aPoles, aWeights, aKnots, aMults, aDeg);
        }
        Xyz anExt = theOracle.Real(-1, 1) * anAx3.XDirection().XYZ() + theOracle.Real(-1, 1) * anAx3.YDirection().XYZ()
                       + aTilt * anAx3.Direction().XYZ();
        if (anExt.Modulus() < 0.1)
        {
            anExt = anAx3.XDirection().XYZ();
        }
        using (aNC)
        using (var aNExt = new N.gp_Dir(anExt.X(), anExt.Y(), anExt.Z()))
        {
            var aS = new ExtrusionSurface(aPC, new Direction3D(anExt.X(), anExt.Y(), anExt.Z()));
            var aNS = new N.Geom_SurfaceOfLinearExtrusion(aNC, aNExt);
            if (theOracle.Int(0, 1) == 0)
            {
                // Bounded in V
                double V1 = theOracle.Real(-5, 0), V2 = V1 + theOracle.Real(0.5, 10);
                using (aNS)
                {
                    return (new N.Geom_RectangularTrimmedSurface(aNS, V1, V2, false, true),
                            new TrimmedSurface(aS, V1, V2, false, true));
                }
            }
            return (aNS, aS);
        }
    }

    private static (N.Geom_Surface Native, Surface Port) RandomSurface(Oracle theOracle, out string theKind)
    {
        int aKind = theOracle.Int(0, 8);
        double aNoise = Noises[theOracle.Int(0, Noises.Length - 1)];
        switch (aKind)
        {
            case 0:
                theKind = "elementary";
                return Elementary(theOracle);
            case 1:
            case 2:
                theKind = "planar B-spline";
                return PlanarBSpline(theOracle, aNoise, false);
            case 3:
                theKind = "planar Bezier";
                return PlanarBSpline(theOracle, aNoise, true);
            case 4:
                theKind = "B-spline";
                return RandomBSpline(theOracle);
            case 5:
                theKind = "revolution";
                return Revolution(theOracle);
            case 6:
                theKind = "extrusion";
                return Extrusion(theOracle);
            case 7:
            {
                theKind = "trimmed";
                (N.Geom_Surface aN, Surface aP) = theOracle.Int(0, 2) == 0
                                                           ? Elementary(theOracle)
                                                           : PlanarBSpline(theOracle, aNoise, false);
                using (aN)
                {
                    aP.Bounds(out double aU1, out double aU2, out double aV1, out double aV2);
                    double U1 = Bound(theOracle, aU1, aU2, out double U2);
                    double V1 = Bound(theOracle, aV1, aV2, out double V2);
                    return (new N.Geom_RectangularTrimmedSurface(aN, U1, U2, V1, V2),
                            new TrimmedSurface(aP, U1, U2, V1, V2));
                }
            }
            default:
            {
                theKind = "offset";
                (N.Geom_Surface aN, Surface aP) = PlanarBSpline(theOracle, aNoise, false);
                double anOffset = theOracle.Real(-2, 2);
                using (aN)
                {
                    return (new N.Geom_OffsetSurface(aN, anOffset, true), new OffsetSurface(aP, anOffset, true));
                }
            }
        }
    }

    /// <summary>A sub-range of [theFirst, theLast] (of [-10, 10] where it is infinite or periodic and long).</summary>
    private static double Bound(Oracle theOracle, double theFirst, double theLast, out double theSecond)
    {
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(theFirst) || global::Kerf.Engine.Foundation.Precision.IsInfinite(theLast))
        {
            theFirst = -10;
            theLast = 10;
        }
        double aLen = theLast - theFirst;
        double aT1 = theFirst + theOracle.Real(0, 0.4) * aLen;
        theSecond = theLast - theOracle.Real(0, 0.4) * aLen;
        return aT1;
    }

    [Fact]
    public void IsPlanarSurface_on_random_surfaces()
    {
        var anOracle = new Oracle(output);
        var aCounts = new Dictionary<string, (int Planar, int Total)>();
        for (int k = 0; k < Cases; k++)
        {
            (N.Geom_Surface aNS, Surface aPS) = RandomSurface(anOracle, out string aKind);
            using (aNS)
            {
                bool aDefault = anOracle.Int(0, 3) == 0;
                double aTol = Tolerances[anOracle.Int(0, Tolerances.Length - 1)];
                using N.GeomLib_IsPlanarSurface aN = aDefault ? new(aNS) : new(aNS, aTol);
                IsPlanarSurface aP = aDefault ? new(aPS) : new(aPS, aTol);
                string aWhat = $"case {k} ({aKind})";
                anOracle.Same(aN.IsPlanar(), aP.IsPlanar, aWhat + " IsPlanar");
                (int aPlanar, int aTotal) = aCounts.GetValueOrDefault(aKind);
                aCounts[aKind] = (aPlanar + (aP.IsPlanar ? 1 : 0), aTotal + 1);
                if (aN.IsPlanar() && aP.IsPlanar)
                {
                    using N.gp_Pln aNPln = aN.Plan();
                    using N.gp_Ax3 aNPos = aNPln.Position();
                    using N.gp_Pnt aNLoc = aNPos.Location();
                    using N.gp_Dir aNDir = aNPos.Direction();
                    using N.gp_Dir aNXDir = aNPos.XDirection();
                    using N.gp_Dir aNYDir = aNPos.YDirection();
                    CoordinateSystem aPos = aP.Plan().Position();
                    anOracle.Same(aNLoc, aPos.Location(), aWhat + " location");
                    SameDir(anOracle, aNDir, aPos.Direction(), aWhat + " direction");
                    SameDir(anOracle, aNXDir, aPos.XDirection(), aWhat + " X direction");
                    SameDir(anOracle, aNYDir, aPos.YDirection(), aWhat + " Y direction");
                }
                else
                {
                    Assert.ThrowsAny<global::Kerf.Engine.Foundation.NotDoneException>(() => aP.Plan());
                }
            }
        }
        foreach ((string aKind, (int aPlanar, int aTotal)) in aCounts)
        {
            output.WriteLine($"{aKind}: {aPlanar}/{aTotal} planar");
        }
        anOracle.Report("GeomLib_IsPlanarSurface");
    }
}
