// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Xunit.Abstractions;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// BRepIntCurveSurface_Inter against native OCCT on the random scenes of <see cref="ClassificationScenes"/> (those
/// with surfaces): lines through points in, on and around the shape (through two such points or in a random
/// direction), with Init(shape, line, tol), and segments, circles and B-spline curves through them as bounded or
/// unbounded GeomAdaptor_Curves, with Load once and Init(curve) for each. Every point of the iteration is compared:
/// its 3D point, U, V, W, state, transition and face (by its place in the shape), and the number of points.
/// </summary>
public class BRepIntCurveSurfaceOracleTests(ITestOutputHelper output)
{
    private const int Scenes = 120;

    [Fact]
    public void Inter_matches_native_OCCT()
    {
        var anOracle = new Oracle(output, 20261101);
        int aNbCurves = 0, aNbPoints = 0;
        for (int aScene = 0; aScene < Scenes; aScene++)
        {
            if (!ClassificationScenes.HasSurfaces(aScene))
            {
                continue;
            }

            Twin<N.TopoDS_Shape, TopoDS_Shape> aShape = ClassificationScenes.Scene(anOracle, aScene);
            List<Twin<N.TopoDS_Shape, TopoDS_Shape>> aFaces = ClassificationScenes.Explore(aShape, TopAbs_FACE);
            List<gp_Pnt> aPoints = ClassificationScenes.SamplePoints(anOracle, aShape.P, 6, 2);
            string aWhat = $"scene {aScene} ({ClassificationScenes.Name(aScene)})";
            double aTol = anOracle.Bool() ? 1e-7 : 1e-3;

            // lines, each with Init(shape, line, tol)
            for (int i = 0; i < 8; i++)
            {
                gp_Pnt aP = aPoints[anOracle.Int(0, aPoints.Count - 1)];
                gp_Pnt aQ = aPoints[anOracle.Int(0, aPoints.Count - 1)];
                gp_XYZ aD = aP.Distance(aQ) > 1e-3 && anOracle.Bool() ? aQ.XYZ() - aP.XYZ() : GeomOracleInputs.RawDir(anOracle);
                using var aNP = Oracle.ToNative(aP);
                using var aND = new N.gp_Dir(aD.X(), aD.Y(), aD.Z());
                using var aNL = new N.gp_Lin(aNP, aND);
                var aL = new gp_Lin(aP, new gp_Dir(aD.X(), aD.Y(), aD.Z()));
                using var aNI = new N.BRepIntCurveSurface_Inter();
                aNI.Init(aShape.N, aNL, aTol);
                var aPI = new BRepIntCurveSurface_Inter();
                aPI.Init(aShape.P, aL, aTol);
                aNbPoints += Compare(anOracle, aNI, aPI, aFaces, $"{aWhat}, line {i}");
                aNbCurves++;
            }

            // curves, with Load once and Init(curve)
            using var aNI2 = new N.BRepIntCurveSurface_Inter();
            aNI2.Load(aShape.N, aTol);
            var aPI2 = new BRepIntCurveSurface_Inter();
            aPI2.Load(aShape.P, aTol);
            for (int i = 0; i < 6; i++)
            {
                (N.GeomAdaptor_Curve aNC, GeomAdaptor_Curve aPC) = RandomCurve(anOracle, aPoints);
                using (aNC)
                {
                    aNI2.Init(aNC);
                    aPI2.Init(aPC);
                    aNbPoints += Compare(anOracle, aNI2, aPI2, aFaces, $"{aWhat}, curve {i} ({aPC.GetType()})");
                }

                aNbCurves++;
            }
        }

        output.WriteLine($"{aNbCurves} curves, {aNbPoints} points");
        anOracle.Report("BRepIntCurveSurface_Inter");
    }

    /// <summary>A segment, a line, a circle or a B-spline curve through the sample points, bounded or not, on both sides.</summary>
    private static (N.GeomAdaptor_Curve, GeomAdaptor_Curve) RandomCurve(Oracle theOracle, List<gp_Pnt> thePoints)
    {
        gp_Pnt aP = thePoints[theOracle.Int(0, thePoints.Count - 1)];
        gp_Pnt aQ = thePoints[theOracle.Int(0, thePoints.Count - 1)];
        using var aNP = Oracle.ToNative(aP);
        switch (theOracle.Int(0, 3))
        {
            case 0:
            case 1:
            {
                // a segment from P beyond Q (or a whole line)
                gp_XYZ aD = aP.Distance(aQ) > 1e-3 ? aQ.XYZ() - aP.XYZ() : GeomOracleInputs.RawDir(theOracle);
                using var aND = new N.gp_Dir(aD.X(), aD.Y(), aD.Z());
                var aNLine = new N.Geom_Line(aNP, aND);
                var aLine = new Geom_Line(aP, new gp_Dir(aD.X(), aD.Y(), aD.Z()));
                if (theOracle.Bool())
                {
                    double t1 = theOracle.Real(-5, 0), t2 = theOracle.Real(0.5, 3) * Math.Max(1.0, aD.Modulus());
                    return (new N.GeomAdaptor_Curve(aNLine, t1, t2), new GeomAdaptor_Curve(aLine, t1, t2));
                }

                return (new N.GeomAdaptor_Curve(aNLine), new GeomAdaptor_Curve(aLine));
            }

            case 2:
            {
                // a circle through P
                gp_XYZ aN = GeomOracleInputs.RawDir(theOracle);
                gp_XYZ aRef = GeomOracleInputs.RawDir(theOracle);
                gp_XYZ anX = aN.Crossed(aRef);
                while (anX.Modulus() < 0.1)
                {
                    aRef = GeomOracleInputs.RawDir(theOracle);
                    anX = aN.Crossed(aRef);
                }

                double r = theOracle.Real(0.5, 6);
                var aDirX = new gp_Dir(anX);
                gp_Pnt aC = aP.Translated(new gp_Vec(aDirX).Multiplied(-r));
                using var aNC = Oracle.ToNative(aC);
                using var aNN = new N.gp_Dir(aN.X(), aN.Y(), aN.Z());
                using var aNX = new N.gp_Dir(anX.X(), anX.Y(), anX.Z());
                using var aNAx = new N.gp_Ax2(aNC, aNN, aNX);
                var aNCircle = new N.Geom_Circle(aNAx, r);
                var aCircle = new Geom_Circle(new gp_Ax2(aC, new gp_Dir(aN.X(), aN.Y(), aN.Z()), new gp_Dir(anX.X(), anX.Y(), anX.Z())), r);
                if (theOracle.Bool())
                {
                    double t1 = theOracle.Real(-1, 0), t2 = theOracle.Real(0.5, 4);
                    return (new N.GeomAdaptor_Curve(aNCircle, t1, t2), new GeomAdaptor_Curve(aCircle, t1, t2));
                }

                return (new N.GeomAdaptor_Curve(aNCircle), new GeomAdaptor_Curve(aCircle));
            }

            default:
            {
                // a cubic B-spline whose poles wander around P and Q
                int aDeg = theOracle.Int(2, 3);
                theOracle.BSplineKnots(aDeg, false, out NCollection_Array1<double> aKnots, out NCollection_Array1<int> aMults);
                int aNb = BSplCLib.NbPoles(aDeg, false, aMults);
                var aPoles = new NCollection_Array1<gp_Pnt>(1, aNb);
                for (int i = 1; i <= aNb; i++)
                {
                    double t = (i - 1.0) / (aNb - 1.0);
                    gp_XYZ aBase = aP.XYZ() * (1.0 - t) + aQ.XYZ() * t;
                    aPoles[i] = new gp_Pnt(aBase.X() + theOracle.Real(-2, 2), aBase.Y() + theOracle.Real(-2, 2), aBase.Z() + theOracle.Real(-2, 2));
                }

                var aCurve = new Geom_BSplineCurve(aPoles, aKnots, aMults, aDeg);
                N.Geom_BSplineCurve aNCurve = Oracle.ToNativeCurve(aPoles, null, aKnots, aMults, aDeg, false);
                return (new N.GeomAdaptor_Curve(aNCurve), new GeomAdaptor_Curve(aCurve));
            }
        }
    }

    /// <summary>Iterates both and compares every point; returns their number.</summary>
    private static int Compare(Oracle theOracle, N.BRepIntCurveSurface_Inter theN, BRepIntCurveSurface_Inter theP,
                               List<Twin<N.TopoDS_Shape, TopoDS_Shape>> theFaces, string theWhat)
    {
        int aCount = 0;
        while (true)
        {
            bool aNMore = theN.More();
            theOracle.Same(aNMore, theP.More(), $"{theWhat} More({aCount})");
            if (!aNMore)
            {
                break;
            }

            string aWhat = $"{theWhat} point {aCount}";
            using (N.gp_Pnt aNPnt = theN.Pnt())
            {
                theOracle.Same(aNPnt, theP.Pnt(), aWhat + " Pnt");
            }

            theOracle.Same(theN.U(), theP.U(), aWhat + " U");
            theOracle.Same(theN.V(), theP.V(), aWhat + " V");
            theOracle.Same(theN.W(), theP.W(), aWhat + " W");
            theOracle.Same((int)theN.State(), (int)theP.State(), aWhat + " State");
            theOracle.Same((int)theN.Transition(), (int)theP.Transition(), aWhat + " Transition");
            using (N.TopoDS_Face aNF = theN.Face())
            {
                theOracle.Same(ClassificationScenes.NativeIndex(theFaces, aNF), ClassificationScenes.PortIndex(theFaces, theP.Face()), aWhat + " Face");
            }

            theN.Next();
            theP.Next();
            aCount++;
            Assert.True(aCount < 1000, theWhat + ": too many points");
        }

        return aCount;
    }
}
