#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Reflection;
using Xunit.Abstractions;
using N = Build123d.Occt;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// BOPTools_AlgoTools and IntTools_Context against native OCCT, through the classes the binding has (BOPTools and the
/// context aren't bound): <c>BOPTools_AlgoTools.ComputeState(point, solid, tol, context)</c> against a native
/// BRepClass3d_SolidClassifier, and the context's cached projectors (<c>ProjPS</c>, <c>ProjPC</c>) against native
/// GeomAPI_ProjectPointOnSurf and GeomAPI_ProjectPointOnCurve initialised as IntTools_Context initialises them.
/// </summary>
/// <remarks>
/// Both sides read the same brep files of data/occ and explore them in the same order. ComputeState is the context's
/// solid classifier's <c>Perform(P, tol)</c>; the native classifier is also made once per solid and reused, in the same
/// order of points, so both carry the same call history (the solid classifier's state between calls, decision 6410).
/// Projections are compared bit for bit: the number of points, the lower distance and its parameters.
/// </remarks>
public class BOPToolsOracleTests(ITestOutputHelper output)
{
    private const int MaxFacesPerFile = 80;
    private const int PointsPerFace = 6;

    private static string Metadata(string theKey)
        => typeof(BOPToolsOracleTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>()
                                      .Single(a => a.Key == theKey).Value!;

    private readonly List<string> myMismatches = [];
    private int myComparisons;

    private void Same(double theNative, double thePort, string theWhat)
    {
        myComparisons++;
        if (BitConverter.DoubleToInt64Bits(theNative) != BitConverter.DoubleToInt64Bits(thePort))
        {
            myMismatches.Add($"{theWhat}: native {theNative:R}, port {thePort:R}");
        }
    }

    private static IEnumerable<string> Files() =>
        Directory.GetFiles(Path.Combine(Metadata("OcctSourceDir"), "data", "occ"), "*.brep").Order(StringComparer.Ordinal);

    private static (N.TopoDS_Shape N, TopoDS_Shape P)? Read(string theFile)
    {
        var aNShape = new N.TopoDS_Shape();
        using (var aBuilder = new N.BRep_Builder())
        {
            if (!N.BRepTools.Read(aNShape, theFile, aBuilder))
            {
                return null;
            }
        }
        if (!BRepTools.Read(out TopoDS_Shape aPShape, theFile))
        {
            return null;
        }
        return (aNShape, aPShape);
    }

    [Fact]
    public void ComputeState_of_points_matches_the_native_solid_classifier()
    {
        var aRnd = new Random(64401);
        int aSolids = 0, aPoints = 0;
        int[] aStates = new int[4];
        foreach (string aFile in Files())
        {
            if (Read(aFile) is not { } aShapes)
            {
                continue;
            }
            var aNExp = new N.TopExp_Explorer(aShapes.N, N.TopAbs_ShapeEnum.TopAbs_SOLID);
            var aPExp = new TopExp_Explorer(aShapes.P, TopAbs_SOLID);
            int iS = 0;
            for (; aNExp.More() && iS < 4; aNExp.Next(), aPExp.Next(), iS++)
            {
                Assert.True(aPExp.More());
                TopoDS_Solid aPS = TopoDS.Solid(aPExp.Current());
                using var aNC = new N.BRepClass3d_SolidClassifier(aNExp.Current());
                var aCtx = new IntTools_Context();
                Bnd_Box aBox = new();
                BRepBndLib.Add(aPS, ref aBox);
                if (aBox.IsVoid())
                {
                    continue;
                }
                aBox.Get(out double x0, out double y0, out double z0, out double x1, out double y1, out double z1);
                aSolids++;
                // random points in and around the box, and points on and near the solid's vertices
                var aPnts = new List<gp_Pnt>();
                for (int i = 0; i < 80; i++)
                {
                    aPnts.Add(new gp_Pnt(x0 + (x1 - x0) * (1.2 * aRnd.NextDouble() - 0.1),
                                         y0 + (y1 - y0) * (1.2 * aRnd.NextDouble() - 0.1),
                                         z0 + (z1 - z0) * (1.2 * aRnd.NextDouble() - 0.1)));
                }
                int iV = 0;
                for (var aVExp = new TopExp_Explorer(aPS, TopAbs_VERTEX); aVExp.More() && iV < 20; aVExp.Next(), iV++)
                {
                    gp_Pnt aV = BRep_Tool.Pnt(TopoDS.Vertex(aVExp.Current()));
                    double d = 1e-3 * (x1 - x0) * aRnd.NextDouble();
                    aPnts.Add(new gp_Pnt(aV.X() + d, aV.Y() - d, aV.Z() + d));
                }
                foreach (gp_Pnt aP in aPnts)
                {
                    double aTol = aRnd.Next(2) == 0 ? 1e-7 : 1e-4;
                    TopAbs_State aState = BOPTools_AlgoTools.ComputeState(aP, aPS, aTol, aCtx);
                    using N.gp_Pnt aNP = Oracle.ToNative(aP);
                    aNC.Perform(aNP, aTol);
                    aStates[(int)aState]++;
                    aPoints++;
                    Same((int)aNC.State(), (int)aState, $"{Path.GetFileName(aFile)} solid {iS} at ({aP.X():R}, {aP.Y():R}, {aP.Z():R}) tol {aTol}");
                }
            }
        }
        output.WriteLine($"ComputeState: {aSolids} solids, {aPoints} points, {myMismatches.Count} mismatches; "
                         + $"IN {aStates[0]}, OUT {aStates[1]}, ON {aStates[2]}, UNKNOWN {aStates[3]}");
        foreach (string aM in myMismatches.Take(30))
        {
            output.WriteLine(aM);
        }
        Assert.Empty(myMismatches);
    }

    [Fact]
    public void Context_projectors_match_native_projectors_initialised_alike()
    {
        var aRnd = new Random(64402);
        int aFaces = 0, aEdges = 0, aPoints = 0;
        foreach (string aFile in Files())
        {
            if (Read(aFile) is not { } aShapes)
            {
                continue;
            }
            string aName = Path.GetFileName(aFile);
            var aCtx = new IntTools_Context();
            // faces: ProjPS
            var aNExp = new N.TopExp_Explorer(aShapes.N, N.TopAbs_ShapeEnum.TopAbs_FACE);
            var aPExp = new TopExp_Explorer(aShapes.P, TopAbs_FACE);
            for (int iF = 0; aNExp.More() && iF < MaxFacesPerFile; aNExp.Next(), aPExp.Next(), iF++)
            {
                TopoDS_Face aPF = TopoDS.Face(aPExp.Current());
                if (BRep_Tool.Surface(aPF) is null)
                {
                    continue;
                }
                N.TopoDS_Face aNF = N.TopoDS.Face(aNExp.Current());
                using var aNBAS = new N.BRepAdaptor_Surface(aNF, true);
                using var aNProj = new N.GeomAPI_ProjectPointOnSurf();
                aNProj.Init(N.BRep_Tool.Surface(aNF), aNBAS.FirstUParameter(), aNBAS.LastUParameter(),
                            aNBAS.FirstVParameter(), aNBAS.LastVParameter(), 1.0e-12);
                aNProj.SetExtremaFlag(N.Extrema_ExtFlag.Extrema_ExtFlag_MIN);
                GeomAPI_ProjectPointOnSurf aPProj = aCtx.ProjPS(aPF);
                aFaces++;
                BRepTools.UVBounds(aPF, out double u1, out double u2, out double v1, out double v2);
                Geom_Surface aS = BRep_Tool.Surface(aPF)!;
                for (int i = 0; i < PointsPerFace; i++)
                {
                    gp_Pnt aOn = aS.Value(u1 + (u2 - u1) * aRnd.NextDouble(), v1 + (v2 - v1) * aRnd.NextDouble());
                    double d = aRnd.Next(3) switch { 0 => 0.0, 1 => 1e-4, _ => 0.1 };
                    gp_Pnt aP = new(aOn.X() + d * (aRnd.NextDouble() - 0.5), aOn.Y() + d * (aRnd.NextDouble() - 0.5),
                                    aOn.Z() + d * (aRnd.NextDouble() - 0.5));
                    using N.gp_Pnt aNP = Oracle.ToNative(aP);
                    aNProj.Perform(aNP);
                    aPProj.Perform(aP);
                    aPoints++;
                    string aWhat = $"{aName} face {iF} point {i}";
                    Same(aNProj.IsDone() ? 1 : 0, aPProj.IsDone() ? 1 : 0, $"{aWhat} IsDone");
                    Same(aNProj.NbPoints(), aPProj.NbPoints(), $"{aWhat} NbPoints");
                    if (aNProj.IsDone() && aPProj.IsDone() && aNProj.NbPoints() > 0 && aPProj.NbPoints() > 0)
                    {
                        Same(aNProj.LowerDistance(), aPProj.LowerDistance(), $"{aWhat} LowerDistance");
                        double nu = 0, nv = 0;
                        aNProj.LowerDistanceParameters(ref nu, ref nv);
                        aPProj.LowerDistanceParameters(out double pu, out double pv);
                        Same(nu, pu, $"{aWhat} U");
                        Same(nv, pv, $"{aWhat} V");
                    }
                }
            }
            // edges: ProjPC
            var aNEExp = new N.TopExp_Explorer(aShapes.N, N.TopAbs_ShapeEnum.TopAbs_EDGE);
            var aPEExp = new TopExp_Explorer(aShapes.P, TopAbs_EDGE);
            for (int iE = 0; aNEExp.More() && iE < MaxFacesPerFile; aNEExp.Next(), aPEExp.Next(), iE++)
            {
                TopoDS_Edge aPE = TopoDS.Edge(aPEExp.Current());
                Geom_Curve? aC = BRep_Tool.Curve(aPE, out double f, out double l);
                if (aC is null || BRep_Tool.Degenerated(aPE))
                {
                    continue;
                }
                N.TopoDS_Edge aNE = N.TopoDS.Edge(aNEExp.Current());
                double nf = 0, nl = 0;
                N.Geom_Curve? aNC = N.BRep_Tool.Curve(aNE, ref nf, ref nl);
                using var aNProj = new N.GeomAPI_ProjectPointOnCurve();
                aNProj.Init(aNC, nf, nl);
                GeomAPI_ProjectPointOnCurve aPProj = aCtx.ProjPC(aPE);
                aEdges++;
                for (int i = 0; i < PointsPerFace; i++)
                {
                    gp_Pnt aOn = aC.Value(f + (l - f) * (1.2 * aRnd.NextDouble() - 0.1));
                    double d = aRnd.Next(3) switch { 0 => 0.0, 1 => 1e-4, _ => 0.1 };
                    gp_Pnt aP = new(aOn.X() + d * (aRnd.NextDouble() - 0.5), aOn.Y() + d * (aRnd.NextDouble() - 0.5),
                                    aOn.Z() + d * (aRnd.NextDouble() - 0.5));
                    using N.gp_Pnt aNP = Oracle.ToNative(aP);
                    aNProj.Perform(aNP);
                    aPProj.Perform(aP);
                    aPoints++;
                    string aWhat = $"{aName} edge {iE} point {i}";
                    Same(aNProj.NbPoints(), aPProj.NbPoints(), $"{aWhat} NbPoints");
                    if (aNProj.NbPoints() > 0 && aPProj.NbPoints() > 0)
                    {
                        Same(aNProj.LowerDistance(), aPProj.LowerDistance(), $"{aWhat} LowerDistance");
                        Same(aNProj.LowerDistanceParameter(), aPProj.LowerDistanceParameter(), $"{aWhat} parameter");
                    }
                }
            }
        }
        output.WriteLine($"Projections: {aFaces} faces, {aEdges} edges, {aPoints} points, {myComparisons} comparisons, "
                         + $"{myMismatches.Count} mismatches");
        foreach (string aM in myMismatches.Take(30))
        {
            output.WriteLine(aM);
        }
        Assert.Empty(myMismatches);
    }
}
#endif
