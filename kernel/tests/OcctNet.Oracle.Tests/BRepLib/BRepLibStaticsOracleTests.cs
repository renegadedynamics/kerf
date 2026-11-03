// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Text;
using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// BRepLib's in-place statics (SameParameter, SameRange, CheckSameRange, BuildCurve3d, BuildCurves3d,
/// UpdateTolerances, UpdateInnerTolerances, UpdateEdgeTolerance, the ReShape forms, OrientClosedSolid) against native
/// OCCT. Each operation runs on a fresh twin scene, built with native and ported BRep_Builder call for call from the
/// same numbers (unfrozen on the port's side, as ported code holds its own shapes): a face on a plane, an axis-aligned
/// cylinder or a B-spline surface, with one to three edges whose pcurves are bent reparametrizations of their 3D curves
/// (quadratic B-splines on collinear poles), over the 3D range or another one, or edges with only a pcurve; random
/// face, edge and vertex tolerances, vertices moved off the curves. Results are compared through the BRep text of the
/// scene (or of the reshaped result), byte for byte, with the returned values.
/// </summary>
public sealed class BRepLibStaticsOracleTests(ITestOutputHelper output) : IDisposable
{
    private const int Scenes = 300;

    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-breplibs-").FullName;
    private int myFileNumber;

    public void Dispose()
    {
        try
        {
            Directory.Delete(myDirectory, true);
        }
        catch (IOException)
        {
        }
    }

    private string NewFile() => Path.Combine(myDirectory, $"s{myFileNumber++}.brep");

    private string NativeText(N.TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(N.BRepTools.Write(theShape, aFile));
        return File.ReadAllText(aFile, Encoding.Latin1);
    }

    private string PortText(in TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(BRepTools.Write(theShape, aFile));
        return File.ReadAllText(aFile, Encoding.Latin1);
    }

    // ------------------------------------------------------------ scenes

    private enum SurfaceKind { Plane, Cylinder, BSpline }

    private sealed record EdgeSpec(bool HasCurve, gp_Pnt2d A, gp_Pnt2d B, double Bend, double F3, double L3, double PF, double PL,
                                   double Tol, double TolV1, double TolV2, gp_Vec Move1, gp_Vec Move2,
                                   gp_Pnt2d[]? Poles2d, bool SameRangeFlag, bool SameParameterFlag);

    private sealed record SceneSpec(SurfaceKind Kind, gp_Ax3 Ax, double Radius, NCollection_Array2<gp_Pnt>? Poles, double FaceTol,
                                    EdgeSpec[] Edges);

    private static gp_Ax3 Ax3(Oracle theO)
    {
        // exact axes (a permutation of the global ones), so that both sides hold the same numbers
        gp_Dir[] aDirs = [new(1, 0, 0), new(0, 1, 0), new(0, 0, 1), new(-1, 0, 0), new(0, -1, 0), new(0, 0, -1)];
        while (true)
        {
            gp_Dir aN = aDirs[theO.Int(0, 5)];
            gp_Dir aX = aDirs[theO.Int(0, 5)];
            if (Math.Abs(aN.Dot(aX)) < 0.5)
            {
                return new gp_Ax3(new gp_Pnt(theO.Real(-10, 10), theO.Real(-10, 10), theO.Real(-10, 10)), aN, aX);
            }
        }
    }

    private static Geom_Surface PortSurface(SceneSpec theS) => theS.Kind switch
    {
        SurfaceKind.Plane => new Geom_Plane(theS.Ax),
        SurfaceKind.Cylinder => new Geom_CylindricalSurface(theS.Ax, theS.Radius),
        _ => new Geom_BSplineSurface(theS.Poles!, Knots01(4), Knots01(4), Mults(4, 3), Mults(4, 3), 3, 3),
    };

    private static NCollection_Array1<double> Knots01(int theNbPoles) => NCollection_Array1<double>.From([0.0, 1.0]);

    private static NCollection_Array1<int> Mults(int theNbPoles, int theDegree) => NCollection_Array1<int>.From([theDegree + 1, theDegree + 1]);

    private static N.Geom_Surface NativeSurface(SceneSpec theS)
    {
        using var anAx = Oracle.ToNative(theS.Ax);
        switch (theS.Kind)
        {
            case SurfaceKind.Plane:
                return new N.Geom_Plane(anAx);
            case SurfaceKind.Cylinder:
                return new N.Geom_CylindricalSurface(anAx, theS.Radius);
            default:
                using (var aPoles = Oracle.ToNative(theS.Poles!))
                using (var aK = Oracle.ToNative(Knots01(4)))
                using (var aM = Oracle.ToNative(Mults(4, 3)))
                {
                    return new N.Geom_BSplineSurface(aPoles, aK, aK, aM, aM, 3, 3);
                }
        }
    }

    private static SceneSpec NewScene(Oracle theO)
    {
        var aKind = (SurfaceKind)theO.Int(0, 2);
        gp_Ax3 anAx = Ax3(theO);
        double aRadius = theO.Real(1, 5);
        NCollection_Array2<gp_Pnt>? aPoles = null;
        if (aKind == SurfaceKind.BSpline)
        {
            aPoles = new NCollection_Array2<gp_Pnt>(1, 4, 1, 4);
            for (int i = 1; i <= 4; i++)
            {
                for (int j = 1; j <= 4; j++)
                {
                    aPoles[i, j] = new gp_Pnt(3 * i + theO.Real(-0.5, 0.5), 3 * j + theO.Real(-0.5, 0.5), theO.Real(-2, 2));
                }
            }
        }
        var aScene = new SceneSpec(aKind, anAx, aRadius, aPoles, Math.Pow(10, theO.Real(-7, -3)), []);
        Geom_Surface aSurf = PortSurface(aScene);
        int aNbEdges = theO.Int(1, 3);
        bool isWithCurves = theO.Int(0, 3) != 0;
        var anEdges = new EdgeSpec[aNbEdges];
        for (int k = 0; k < aNbEdges; k++)
        {
            gp_Pnt2d anA, aB;
            double aF3, aL3;
            switch (aKind)
            {
                case SurfaceKind.Plane:
                    anA = new gp_Pnt2d(theO.Real(-5, 5), theO.Real(-5, 5));
                    aB = new gp_Pnt2d(theO.Real(-5, 5), theO.Real(-5, 5));
                    aF3 = theO.Real(-3, 3);
                    aL3 = aF3 + theO.Real(0.5, 10);
                    break;
                case SurfaceKind.Cylinder:
                    {
                        // an arc of the iso v: the circle's parameter is u
                        double u0 = theO.Real(0, 3), v = theO.Real(-3, 3);
                        anA = new gp_Pnt2d(u0, v);
                        aB = new gp_Pnt2d(u0 + theO.Real(0.3, 3), v);
                        aF3 = anA.X();
                        aL3 = aB.X();
                        break;
                    }
                default:
                    {
                        // a stretch of the iso u: the B-spline's parameter is v
                        double u0 = theO.Real(0.05, 0.95), v0 = theO.Real(0, 0.4);
                        anA = new gp_Pnt2d(u0, v0);
                        aB = new gp_Pnt2d(u0, v0 + theO.Real(0.2, 0.6));
                        aF3 = anA.Y();
                        aL3 = aB.Y();
                        break;
                    }
            }

            bool isSameRange = theO.Bool();
            double aPF = isSameRange ? aF3 : theO.Real(-2, 2);
            double aPL = isSameRange ? aL3 : aPF + theO.Real(0.5, 5);
            gp_Pnt2d[]? aPoles2d = null;
            if (!isWithCurves)
            {
                // a pcurve alone: a cubic in the domain (from A to B through two random points)
                aPoles2d = new gp_Pnt2d[4];
                aPoles2d[0] = anA;
                aPoles2d[3] = aB;
                for (int i = 1; i <= 2; i++)
                {
                    gp_Pnt2d aM = new(anA.X() + (aB.X() - anA.X()) * i / 3.0, anA.Y() + (aB.Y() - anA.Y()) * i / 3.0);
                    double aD = aKind == SurfaceKind.BSpline ? 0.05 : 0.5;
                    aPoles2d[i] = new gp_Pnt2d(aM.X() + theO.Real(-aD, aD), aM.Y() + theO.Real(-aD, aD));
                }
            }
            double aMove = theO.Int(0, 2) == 0 ? Math.Pow(10, theO.Real(-6, -3)) : 0;
            anEdges[k] = new EdgeSpec(isWithCurves, anA, aB, theO.Real(0.1, 0.9), aF3, aL3, aPF, aPL,
                                      Math.Pow(10, theO.Real(-7, -4)), Math.Pow(10, theO.Real(-7, -3)), Math.Pow(10, theO.Real(-7, -3)),
                                      new gp_Vec(aMove * theO.Real(-1, 1), aMove * theO.Real(-1, 1), aMove * theO.Real(-1, 1)),
                                      new gp_Vec(0, 0, 0), aPoles2d, isSameRange && theO.Bool(), theO.Int(0, 3) == 0);
        }
        _ = aSurf;
        return aScene with { Edges = anEdges };
    }

    private sealed class Twin
    {
        public required N.TopoDS_Face NFace;
        public required TopoDS_Face PFace;
        public readonly List<(N.TopoDS_Edge N, TopoDS_Edge P)> Edges = [];
    }

    private static Twin Build(SceneSpec theS)
    {
        var aNB = new N.BRep_Builder();
        var aB = new BRep_Builder();
        Geom_Surface aSurf = PortSurface(theS);
        N.Geom_Surface aNSurf = NativeSurface(theS);
        var aT = new Twin { NFace = new N.TopoDS_Face(), PFace = new TopoDS_Face() };
        aNB.MakeFace(aT.NFace, aNSurf, theS.FaceTol);
        aB.MakeFace(ref aT.PFace, aSurf, theS.FaceTol);
        var aNW = new N.TopoDS_Wire();
        aNB.MakeWire(aNW);
        TopoDS_Wire aW = new();
        aB.MakeWire(ref aW);

        foreach (EdgeSpec anE in theS.Edges)
        {
            var aNE = new N.TopoDS_Edge();
            TopoDS_Edge aPE = new();
            // the 3D curve
            if (anE.HasCurve)
            {
                Geom_Curve aC;
                N.Geom_Curve aNC;
                switch (theS.Kind)
                {
                    case SurfaceKind.Plane:
                        {
                            gp_Pnt aPA = aSurf.Value(anE.A.X(), anE.A.Y()), aPB = aSurf.Value(anE.B.X(), anE.B.Y());
                            var aPoles = NCollection_Array1<gp_Pnt>.From([aPA, aPB]);
                            var aKnots = NCollection_Array1<double>.From([anE.F3, anE.L3]);
                            var aMults = NCollection_Array1<int>.From([2, 2]);
                            aC = new Geom_BSplineCurve(aPoles, aKnots, aMults, 1);
                            aNC = Oracle.ToNativeCurve(aPoles, null, aKnots, aMults, 1, false);
                            break;
                        }
                    case SurfaceKind.Cylinder:
                        {
                            // the iso v: a circle on the cylinder's exact axes
                            gp_Ax2 anAx2 = theS.Ax.Ax2();
                            anAx2.SetLocation(new gp_Pnt(theS.Ax.Location().XYZ() + theS.Ax.Direction().XYZ() * anE.A.Y()));
                            aC = new Geom_Circle(anAx2, theS.Radius);
                            using var aNAx2 = Oracle.ToNative(anAx2);
                            aNC = new N.Geom_Circle(aNAx2, theS.Radius);
                            break;
                        }
                    default:
                        {
                            var anIso = (Geom_BSplineCurve)((Geom_BSplineSurface)aSurf).UIso(anE.A.X());
                            var aPoles = new NCollection_Array1<gp_Pnt>(1, anIso.NbPoles());
                            anIso.Poles(aPoles);
                            var aKnots = new NCollection_Array1<double>(1, anIso.NbKnots());
                            anIso.Knots(aKnots);
                            var aMults = new NCollection_Array1<int>(1, anIso.NbKnots());
                            anIso.Multiplicities(aMults);
                            aC = new Geom_BSplineCurve(aPoles, aKnots, aMults, anIso.Degree());
                            aNC = Oracle.ToNativeCurve(aPoles, null, aKnots, aMults, anIso.Degree(), false);
                            break;
                        }
                }
                aNB.MakeEdge(aNE, aNC, anE.Tol);
                aB.MakeEdge(ref aPE, aC, anE.Tol);
            }
            else
            {
                aNB.MakeEdge(aNE);
                aB.MakeEdge(ref aPE);
                aNB.UpdateEdge(aNE, anE.Tol);
                aB.UpdateEdge(aPE, anE.Tol);
            }

            // the pcurve: a bent quadratic along A B, or the cubic
            NCollection_Array1<gp_Pnt2d> aPoles2d;
            int aDegree;
            if (anE.Poles2d is null)
            {
                gp_Pnt2d aMid = new(anE.A.X() + anE.Bend * (anE.B.X() - anE.A.X()), anE.A.Y() + anE.Bend * (anE.B.Y() - anE.A.Y()));
                aPoles2d = NCollection_Array1<gp_Pnt2d>.From([anE.A, aMid, anE.B]);
                aDegree = 2;
            }
            else
            {
                aPoles2d = NCollection_Array1<gp_Pnt2d>.From(anE.Poles2d);
                aDegree = 3;
            }
            var aKnots2d = NCollection_Array1<double>.From([anE.PF, anE.PL]);
            var aMults2d = NCollection_Array1<int>.From([aDegree + 1, aDegree + 1]);
            var aPC = new Geom2d_BSplineCurve(aPoles2d, aKnots2d, aMults2d, aDegree);
            N.Geom2d_BSplineCurve aNPC = Oracle.ToNativeCurve(aPoles2d, null, aKnots2d, aMults2d, aDegree, false);
            aNB.UpdateEdge(aNE, aNPC, aT.NFace, anE.Tol);
            aB.UpdateEdge(aPE, aPC, aT.PFace, anE.Tol);

            // the vertices at the ends, moved
            gp_Pnt aP1 = aSurf.Value(anE.A.X(), anE.A.Y()).Translated(anE.Move1);
            gp_Pnt aP2 = aSurf.Value(anE.B.X(), anE.B.Y()).Translated(anE.Move2);
            var aNV1 = new N.TopoDS_Vertex();
            var aNV2 = new N.TopoDS_Vertex();
            TopoDS_Vertex aV1 = new(), aV2 = new();
            using (var aNP1 = Oracle.ToNative(aP1))
            using (var aNP2 = Oracle.ToNative(aP2))
            {
                aNB.MakeVertex(aNV1, aNP1, anE.TolV1);
                aNB.MakeVertex(aNV2, aNP2, anE.TolV2);
            }
            aB.MakeVertex(ref aV1, aP1, anE.TolV1);
            aB.MakeVertex(ref aV2, aP2, anE.TolV2);
            using (var aNF = aNV1.Oriented(N.TopAbs_Orientation.TopAbs_FORWARD))
            using (var aNR = aNV2.Oriented(N.TopAbs_Orientation.TopAbs_REVERSED))
            {
                aNB.Add(aNE, aNF);
                aNB.Add(aNE, aNR);
            }
            aB.Add(aPE, aV1.Oriented(TopAbs_Orientation.TopAbs_FORWARD));
            aB.Add(aPE, aV2.Oriented(TopAbs_Orientation.TopAbs_REVERSED));

            // the ranges and flags
            aNB.Range(aNE, aT.NFace, anE.PF, anE.PL);
            aB.Range(aPE, aT.PFace, anE.PF, anE.PL);
            if (anE.HasCurve)
            {
                aNB.Range(aNE, anE.F3, anE.L3, true);
                aB.Range(aPE, anE.F3, anE.L3, true);
            }
            else
            {
                aNB.Range(aNE, anE.PF, anE.PL, true);
                aB.Range(aPE, anE.PF, anE.PL, true);
            }
            aNB.SameRange(aNE, anE.SameRangeFlag);
            aB.SameRange(aPE, anE.SameRangeFlag);
            aNB.SameParameter(aNE, anE.SameParameterFlag);
            aB.SameParameter(aPE, anE.SameParameterFlag);

            aNB.Add(aNW, aNE);
            aB.Add(aW, aPE);
            aT.Edges.Add((aNE, aPE));
        }
        aNB.Add(aT.NFace, aNW);
        aB.Add(aT.PFace, aW);
        return aT;
    }

    // ------------------------------------------------------------ the comparison

    private int myCompared;
    private int myThrown;
    private GeomAbs_Shape myCont;
    private int myMaxDeg;
    private int myMaxSeg;

    // After native OCCT threw on an operation: whether the port throws on the same operation (BuildCurves3d with some
    // continuities; UpdateEdgeTolerance on an edge without 3D curve flagged SameParameter, which reads an empty adaptor).
    private bool PortThrows(int theOp, Twin theT, double theTol, bool theFlag)
    {
        try
        {
            switch (theOp)
            {
                case 4:
                    BRepLib.BuildCurves3d(theT.PFace, theTol, myCont, myMaxDeg, myMaxSeg);
                    break;
                case 10:
                    BRepLib.UpdateEdgeTolerance(theT.PFace, 1e-5, theFlag ? 1.0 : 1e-5);
                    break;
                default:
                    throw new InvalidOperationException($"native OCCT threw on operation {theOp}");
            }
        }
        catch (Standard_Failure)
        {
            return true;
        }
        catch (NullReferenceException)
        {
            // UpdateEdgeTol evaluates an empty adaptor where OCCT dereferences a null handle (occt-issues.md)
            return true;
        }
        return false;
    }
    private int myIdentical;
    private int myValues;
    private readonly List<string> myDifferences = [];

    private void CompareText(string theWhat, N.TopoDS_Shape theNative, in TopoDS_Shape thePort)
    {
        string aN = NativeText(theNative);
        string aP = PortText(thePort);
        myCompared++;
        if (aN == aP)
        {
            myIdentical++;
        }
        else if (myDifferences.Count < 4)
        {
            myDifferences.Add($"{theWhat}:\n--- native\n{aN}\n--- port\n{aP}");
        }
    }

    private void CompareValue<T>(string theWhat, T theNative, T thePort)
    {
        myValues++;
        Assert.True(EqualityComparer<T>.Default.Equals(theNative, thePort), $"{theWhat}: native {theNative}, port {thePort}");
    }

    [Fact]
    public void Statics_change_twin_scenes_as_native_OCCT()
    {
        var anOracle = new Oracle(output, 20261029);
        int[] aCounts = new int[11];
        foreach (int aScene in Tape.Budget(Scenes))
        {
            SceneSpec aSpec = NewScene(anOracle);
            for (int anOp = 0; anOp <= 10; anOp++)
            {
                Twin aT = Build(aSpec);
                Exception? aNEx = null, aPEx = null;
                (N.TopoDS_Edge aNE, TopoDS_Edge aPE) = aT.Edges[0];
                double aTol = Math.Pow(10, anOracle.Real(-7, -3));
                bool aFlag = anOracle.Bool();
                string aWhat = $"scene {aScene} ({aSpec.Kind}, {aSpec.Edges.Length} edges, curves {aSpec.Edges[0].HasCurve}) op {anOp}";
                try
                {
                switch (anOp)
                {
                    case 0:
                        N.BRepLib.SameParameter(aNE, aTol);
                        BRepLib.SameParameter(aPE, aTol);
                        break;
                    case 1:
                        N.BRepLib.SameParameter(aT.NFace, aTol, aFlag);
                        BRepLib.SameParameter(aT.PFace, aTol, aFlag);
                        break;
                    case 2:
                        N.BRepLib.UpdateTolerances(aT.NFace, aFlag);
                        BRepLib.UpdateTolerances(aT.PFace, aFlag);
                        break;
                    case 3:
                        N.BRepLib.UpdateInnerTolerances(aT.NFace);
                        BRepLib.UpdateInnerTolerances(aT.PFace);
                        break;
                    case 4:
                        {
                            var aCont = (GeomAbs_Shape)(2 * anOracle.Int(0, 2));
                            myCont = aCont;
                            myMaxDeg = anOracle.Int(6, 14);
                            myMaxSeg = anOracle.Bool() ? 0 : anOracle.Int(30, 50);
                            bool aNOk = N.BRepLib.BuildCurves3d(aT.NFace, aTol, (N.GeomAbs_Shape)(int)aCont, myMaxDeg, myMaxSeg);
                            CompareValue(aWhat, aNOk, BRepLib.BuildCurves3d(aT.PFace, aTol, aCont, myMaxDeg, myMaxSeg));
                            break;
                        }
                    case 5:
                        CompareValue(aWhat, N.BRepLib.BuildCurve3d(aNE), BRepLib.BuildCurve3d(aPE));
                        break;
                    case 6:
                        {
                            double aConf = aFlag ? 1e-12 : 1e-3;
                            CompareValue(aWhat, N.BRepLib.CheckSameRange(aNE, aConf), BRepLib.CheckSameRange(aPE, aConf));
                            N.BRepLib.SameRange(aNE, aTol);
                            BRepLib.SameRange(aPE, aTol);
                            CompareValue(aWhat, N.BRepLib.CheckSameRange(aNE), BRepLib.CheckSameRange(aPE));
                            break;
                        }
                    case 7:
                        {
                            var aNR = new N.BRepTools_ReShape();
                            var aR = new BRepTools_ReShape();
                            N.BRepLib.SameParameter(aT.NFace, aNR, aTol, aFlag);
                            BRepLib.SameParameter(aT.PFace, aR, aTol, aFlag);
                            CompareText(aWhat + " (input)", aT.NFace, aT.PFace);
                            CompareText(aWhat, aNR.Apply(aT.NFace), aR.Apply(aT.PFace));
                            aCounts[anOp]++;
                            continue;
                        }
                    case 8:
                        {
                            var aNR = new N.BRepTools_ReShape();
                            var aR = new BRepTools_ReShape();
                            N.BRepLib.UpdateTolerances(aT.NFace, aNR, aFlag);
                            BRepLib.UpdateTolerances(aT.PFace, aR, aFlag);
                            CompareText(aWhat + " (input)", aT.NFace, aT.PFace);
                            CompareText(aWhat, aNR.Apply(aT.NFace), aR.Apply(aT.PFace));
                            aCounts[anOp]++;
                            continue;
                        }
                    case 9:
                        {
                            double aNNewTol = -1, aNewTol = -1;
                            N.TopoDS_Edge aNRes = N.BRepLib.SameParameter(aNE, aTol, ref aNNewTol, false);
                            TopoDS_Edge aRes = BRepLib.SameParameter(aPE, aTol, ref aNewTol, false);
                            CompareValue(aWhat, aNNewTol, aNewTol);
                            CompareValue(aWhat, aNRes.IsNull(), aRes.IsNull());
                            if (!aRes.IsNull())
                            {
                                CompareText(aWhat + " (new edge)", aNRes, aRes);
                            }
                            break;
                        }
                    case 10:
                        {
                            double aMax = aFlag ? 1.0 : 1e-5;
                            CompareValue(aWhat, N.BRepLib.UpdateEdgeTolerance(aT.NFace, 1e-5, aMax),
                                         BRepLib.UpdateEdgeTolerance(aT.PFace, 1e-5, aMax));
                            break;
                        }
                }
                }
                catch (N.OcctException anEx)
                {
                    aNEx = anEx;
                }
                catch (Standard_Failure anEx)
                {
                    aPEx = anEx;
                }
                if (aNEx is not null || aPEx is not null)
                {
                    // the native call comes first: if it threw, the port must throw on its own call
                    CompareValue(aWhat + " throws", true, aNEx is not null ? PortThrows(anOp, aT, aTol, aFlag) : false);
                    myThrown++;
                    continue;
                }
                CompareText(aWhat, aT.NFace, aT.PFace);
                aCounts[anOp]++;
            }
        }

        output.WriteLine($"{Scenes} scenes, operations {string.Join(", ", aCounts)}, {myThrown} threw on both sides: {myCompared} files compared, "
                         + $"{myIdentical} byte-identical; {myValues} returned values, all equal");
        foreach (string aD in myDifferences)
        {
            output.WriteLine(aD);
        }
        Assert.Equal(myCompared, myIdentical);
    }

    private static string MaskFree(string theText) => System.Text.RegularExpressions.Regex.Replace(theText, "(?m)^[01]([01]{6})$", "x$1");

    /// <summary>
    /// The functional twins on frozen port scenes against the in-place statics on the native twins: the twin's result
    /// must be native OCCT's changed shape (compared with the Free column masked, since the twin's result shares the
    /// input's frozen sub-shapes where nothing changed, design 4.4), and the port's input unchanged.
    /// </summary>
    [Fact]
    public void Twins_answer_as_the_native_in_place_statics()
    {
        var anOracle = new Oracle(output, 20261030);
        int aCompared = 0, anIdentical = 0, aMasked = 0, aKeptEdges = 0, anEdges = 0, aThrown = 0;
        var aDifferences = new List<string>();
        foreach (int aScene in Tape.Budget(Scenes))
        {
            SceneSpec aSpec = NewScene(anOracle);
            for (int anOp = 0; anOp <= 4; anOp++)
            {
                Twin aT = Build(aSpec);
                aT.PFace.TShape()!.FreezeGraph();
                string aBefore = PortText(aT.PFace);
                double aTol = Math.Pow(10, anOracle.Real(-7, -3));
                bool aFlag = anOracle.Bool();
                string aWhat = $"scene {aScene} ({aSpec.Kind}) twin {anOp}";
                TopoDS_Shape aResult;
                try
                {
                    switch (anOp)
                    {
                        case 0:
                            N.BRepLib.SameParameter(aT.NFace, aTol, aFlag);
                            aResult = BRepLib.SameParametered(aT.PFace, aTol, aFlag);
                            break;
                        case 1:
                            N.BRepLib.UpdateTolerances(aT.NFace, aFlag);
                            aResult = BRepLib.TolerancesUpdated(aT.PFace, aFlag);
                            break;
                        case 2:
                            N.BRepLib.UpdateInnerTolerances(aT.NFace);
                            aResult = BRepLib.InnerTolerancesUpdated(aT.PFace);
                            break;
                        case 3:
                        {
                            var aCont = (GeomAbs_Shape)(2 * anOracle.Int(0, 2));
                            int aMaxDeg = anOracle.Int(6, 14);
                            int aMaxSeg = anOracle.Bool() ? 0 : anOracle.Int(30, 50);
                            bool aNOk = N.BRepLib.BuildCurves3d(aT.NFace, aTol, (N.GeomAbs_Shape)(int)aCont, aMaxDeg, aMaxSeg);
                            aResult = BRepLib.Curves3dBuilt(aT.PFace, aTol, out bool anOk, aCont, aMaxDeg, aMaxSeg);
                            CompareValue(aWhat, aNOk, anOk);
                            break;
                        }
                        default:
                            N.BRepLib.EncodeRegularity(aT.NFace, aTol);
                            aResult = BRepLib.RegularityEncoded(aT.PFace, aTol);
                            break;
                    }
                }
                catch (N.OcctException)
                {
                    aThrown++;
                    continue;
                }

                Assert.Equal(aBefore, PortText(aT.PFace));
                Assert.True(aResult.TShape()!.IsFrozen());
                for (var anExp = new TopExp_Explorer(aT.PFace, TopAbs_ShapeEnum.TopAbs_EDGE); anExp.More(); anExp.Next())
                {
                    anEdges++;
                    for (var anExp2 = new TopExp_Explorer(aResult, TopAbs_ShapeEnum.TopAbs_EDGE); anExp2.More(); anExp2.Next())
                    {
                        if (anExp2.Current().IsSame(anExp.Current()))
                        {
                            aKeptEdges++;
                            break;
                        }
                    }
                }
                string aN = NativeText(aT.NFace), aP = PortText(aResult);
                aCompared++;
                if (aN == aP)
                {
                    anIdentical++;
                }
                else if (MaskFree(aN) == MaskFree(aP))
                {
                    aMasked++;
                }
                else if (aDifferences.Count < 4)
                {
                    aDifferences.Add($"{aWhat}:\n--- native\n{aN}\n--- port\n{aP}");
                }
            }
        }

        output.WriteLine($"twins: {aCompared} results, {anIdentical} byte-identical, {aMasked} equal with the Free column masked; "
                         + $"{aKeptEdges} of {anEdges} input edges kept as they were; {aThrown} native calls threw (skipped)");
        foreach (string aD in aDifferences)
        {
            output.WriteLine(aD);
        }
        Assert.Equal(aCompared, anIdentical + aMasked);
    }

    [Fact]
    public void OrientClosedSolid_answers_as_native_OCCT()
    {
        var anOracle = new Oracle(output, 20261030);
        int aCases = 0, aReversed = 0, aFailed = 0;
        foreach (int i in Tape.Budget(60))
        {
            using var aP = new N.gp_Pnt(anOracle.Real(-10, 10), anOracle.Real(-10, 10), anOracle.Real(-10, 10));
            var aBox = new N.BRepPrimAPI_MakeBox(aP, anOracle.Real(0.5, 5), anOracle.Real(0.5, 5), anOracle.Real(0.5, 5));
            N.TopoDS_Solid aNS = aBox.Solid();
            int aVariant = i % 3;
            if (aVariant == 1)
            {
                aNS = N.TopoDS.Solid(aNS.Reversed());
            }
            else if (aVariant == 2)
            {
                // an open solid: the box's shell without one face
                var aNB = new N.BRep_Builder();
                var aShell = new N.TopoDS_Shell();
                aNB.MakeShell(aShell);
                var anExp = new N.TopExp_Explorer(aNS, N.TopAbs_ShapeEnum.TopAbs_FACE, N.TopAbs_ShapeEnum.TopAbs_SHAPE);
                int k = 0;
                for (; anExp.More(); anExp.Next(), k++)
                {
                    if (k != i % 6)
                    {
                        aNB.Add(aShell, anExp.Current());
                    }
                }
                var aSolid = new N.TopoDS_Solid();
                aNB.MakeSolid(aSolid);
                aNB.Add(aSolid, aShell);
                aNS = aSolid;
            }

            string aFile = NewFile();
            Assert.True(N.BRepTools.Write(aNS, aFile));
            Assert.True(BRepTools.Read(out TopoDS_Shape aRead, aFile));
            TopoDS_Solid aPS = TopoDS.Solid(aRead);
            bool aNOk = N.BRepLib.OrientClosedSolid(aNS);
            bool aOk = BRepLib.OrientClosedSolid(ref aPS);
            Assert.Equal(aNOk, aOk);
            Assert.Equal((int)aNS.Orientation(), (int)aPS.Orientation());
            aCases++;
            if (aPS.Orientation() == TopAbs_Orientation.TopAbs_REVERSED)
            {
                aReversed++;
            }
            if (!aOk)
            {
                aFailed++;
            }
        }
        output.WriteLine($"{aCases} solids: {aReversed} reversed, {aFailed} not orientable; all as native OCCT");
    }
}
