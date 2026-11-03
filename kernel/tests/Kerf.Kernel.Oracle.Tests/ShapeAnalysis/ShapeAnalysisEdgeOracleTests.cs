// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Xunit.Abstractions;
using static Kerf.Topology.ShapeType;
using static Kerf.Engine.Exchange.Brep.FormatVersion;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// ShapeAnalysis_Edge against native OCCT on every edge of every face of random native shapes.
/// </summary>
/// <remarks>
/// Each scene is a random native shape (BRepToolsIOOracleTests' primitives, sweeps, Booleans, fillets and lofts,
/// placed at random) written by native OCCT to a file and read back by both sides, so both start from the same shape.
/// The faces are explored in the same order on both sides, and their edges too. Every method the binding has is
/// compared: its result, its outputs and each status flag after it.
/// </remarks>
public class ShapeAnalysisEdgeOracleTests(ITestOutputHelper output) : IDisposable
{
    private const int Scenes = 72;

    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-sae-").FullName;
    private int myFileNumber;

    private static readonly ExtendStatus[] theStatuses = Enum.GetValues<ExtendStatus>();

    public void Dispose()
    {
        Directory.Delete(myDirectory, true);
        GC.SuppressFinalize(this);
    }

    private (N.TopoDS_Shape N, Shape P, string What) Scene(Oracle theOracle, int theIndex)
    {
        N.TopoDS_Shape aShape = BRepToolsIOOracleTests.NativeShape(theOracle, theIndex, out string aWhat);
        string aFile = Path.Combine(myDirectory, $"s{myFileNumber++}.brep");
        Assert.True(N.BRepTools.Write(aShape, aFile, true, true, N.TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3));
        var aNShape = new N.TopoDS_Shape();
        using (var aBuilder = new N.BRep_Builder())
        {
            Assert.True(N.BRepTools.Read(aNShape, aFile, aBuilder));
        }

        Assert.True(BrepTools.Read(out Shape aPShape, aFile));
        return (aNShape, aPShape, $"scene {theIndex} ({aWhat})");
    }

    private static void SameStatus(Oracle theOracle, N.ShapeAnalysis_Edge theN, EdgeAnalysis theP, string theWhat)
    {
        foreach (ExtendStatus aS in theStatuses)
        {
            theOracle.Same(theN.Status((N.ShapeExtend_Status)(int)aS) ? 1 : 0, theP.Status(aS) ? 1 : 0, $"{theWhat}, status {aS}");
        }
    }

    private static void SameBool(Oracle theOracle, bool theN, bool theP, string theWhat) => theOracle.Same(theN ? 1 : 0, theP ? 1 : 0, theWhat);

    [Fact]
    public void Edge_queries_and_checks_match_native_OCCT()
    {
        var anOracle = new Oracle(output, 42101);
        int aPairs = 0, anOverlaps = 0;
        foreach (int i in Tape.Budget(Scenes))
        {
            (N.TopoDS_Shape aNS, Shape aPS, string aWhat) = Scene(anOracle, i);
            var aNSae = new N.ShapeAnalysis_Edge();
            var aPSae = new EdgeAnalysis();

            var aNFExp = new N.TopExp_Explorer(aNS, N.TopAbs_ShapeEnum.TopAbs_FACE);
            var aPFExp = new Explorer(aPS, ShapeType.Face);
            int aFaceIndex = 0;
            for (; aNFExp.More(); aNFExp.Next(), aPFExp.Next(), aFaceIndex++)
            {
                Assert.True(aPFExp.More());
                N.TopoDS_Face aNF = N.TopoDS.Face(aNFExp.Current());
                Face aPF = ShapeCasts.Face(aPFExp.Current());
                var aNLoc = new N.TopLoc_Location();
                N.Geom_Surface? aNSurf = N.BRep_Tool.Surface(aNF, aNLoc);
                Surface? aPSurf = Tool.Surface(aPF, out Placement aPLoc);

                var aNEExp = new N.TopExp_Explorer(aNF, N.TopAbs_ShapeEnum.TopAbs_EDGE);
                var aPEExp = new Explorer(aPF, ShapeType.Edge);
                N.TopoDS_Edge? aNPrev = null;
                Edge aPPrev = new();
                int anEdgeIndex = 0;
                for (; aNEExp.More(); aNEExp.Next(), aPEExp.Next(), anEdgeIndex++)
                {
                    Assert.True(aPEExp.More());
                    N.TopoDS_Edge aNE = N.TopoDS.Edge(aNEExp.Current());
                    Edge aPE = ShapeCasts.Edge(aPEExp.Current());
                    string w = $"{aWhat}, face {aFaceIndex}, edge {anEdgeIndex}";
                    aPairs++;
                    CompareEdgeOnFace(anOracle, aNSae, aPSae, aNE, aPE, aNF, aPF, aNSurf, aPSurf, aNLoc, aPLoc, w);

                    // overlapping with the previous edge of the face, on a few faces (it is costly). Not with a degenerated
                    // edge: its length is 0, so IsOverlapPartEdges' step is 0 and its loop never ends when the vertex is
                    // within the tolerance of the other edge (native OCCT hangs as the port does; occt-issues.md)
                    if (aNPrev is not null && anEdgeIndex <= 2 && aFaceIndex % 3 == 0 && !Tool.Degenerated(aPE)
                        && !Tool.Degenerated(aPPrev))
                    {
                        anOverlaps++;
                        double aDomain = anEdgeIndex == 1 ? 0.0 : 0.5;
                        double aNTol = 1e-3, aPTol = 1e-3;
                        bool aNRes = aNSae.CheckOverlapping(aNPrev, aNE, ref aNTol, aDomain);
                        bool aPRes = aPSae.CheckOverlapping(aPPrev, aPE, ref aPTol, aDomain);
                        SameBool(anOracle, aNRes, aPRes, w + ", CheckOverlapping");
                        anOracle.Same(aNTol, aPTol, w + ", CheckOverlapping tolerance");
                        SameStatus(anOracle, aNSae, aPSae, w + ", CheckOverlapping");
                    }
                    aNPrev = aNE;
                    aPPrev = aPE;
                }
                Assert.False(aPEExp.More());
            }
            Assert.False(aPFExp.More());
        }

        output.WriteLine($"{aPairs} edge-face pairs, {anOverlaps} overlap checks");
        anOracle.Report("ShapeAnalysis_Edge");
    }

    private static void CompareEdgeOnFace(Oracle o, N.ShapeAnalysis_Edge n, EdgeAnalysis p, N.TopoDS_Edge aNE, Edge aPE,
                                          N.TopoDS_Face aNF, Face aPF, N.Geom_Surface? aNSurf, Surface? aPSurf,
                                          N.TopLoc_Location aNLoc, Placement aPLoc, string w)
    {
        SameBool(o, n.HasCurve3d(aNE), p.HasCurve3d(aPE), w + ", HasCurve3d");
        SameBool(o, n.IsClosed3d(aNE), p.IsClosed3d(aPE), w + ", IsClosed3d");
        SameBool(o, n.HasPCurve(aNE, aNF), p.HasPCurve(aPE, aPF), w + ", HasPCurve");
        SameBool(o, n.HasPCurve(aNE, aNSurf, aNLoc), p.HasPCurve(aPE, aPSurf, aPLoc), w + ", HasPCurve(S, L)");
        SameBool(o, n.IsSeam(aNE, aNF), p.IsSeam(aPE, aPF), w + ", IsSeam");
        SameBool(o, n.IsSeam(aNE, aNSurf, aNLoc), p.IsSeam(aPE, aPSurf, aPLoc), w + ", IsSeam(S, L)");

        // vertices
        N.TopoDS_Vertex aNV1 = n.FirstVertex(aNE), aNV2 = n.LastVertex(aNE);
        Vertex aPV1 = p.FirstVertex(aPE), aPV2 = p.LastVertex(aPE);
        SameBool(o, aNV1.IsNull(), aPV1.IsNull, w + ", FirstVertex null");
        SameBool(o, aNV2.IsNull(), aPV2.IsNull, w + ", LastVertex null");
        if (!aPV1.IsNull)
        {
            o.Same(N.BRep_Tool.Pnt(aNV1), Tool.Pnt(aPV1), w + ", FirstVertex");
            o.Same((int)aNV1.Orientation(), (int)aPV1.Orientation, w + ", FirstVertex orientation");
        }
        if (!aPV2.IsNull)
        {
            o.Same(N.BRep_Tool.Pnt(aNV2), Tool.Pnt(aPV2), w + ", LastVertex");
            o.Same((int)aNV2.Orientation(), (int)aPV2.Orientation, w + ", LastVertex orientation");
        }

        // bounds and end tangents of the pcurve
        var aNFirst = new N.gp_Pnt2d();
        var aNLast = new N.gp_Pnt2d();
        bool aNRes = n.BoundUV(aNE, aNF, aNFirst, aNLast);
        bool aPRes = p.BoundUV(aPE, aPF, out Point2D aPFirst, out Point2D aPLast);
        SameBool(o, aNRes, aPRes, w + ", BoundUV");
        if (aPRes)
        {
            o.Same(aNFirst, aPFirst, w + ", BoundUV first");
            o.Same(aNLast, aPLast, w + ", BoundUV last");
        }
        aNRes = n.BoundUV(aNE, aNSurf, aNLoc, aNFirst, aNLast);
        aPRes = p.BoundUV(aPE, aPSurf, aPLoc, out aPFirst, out aPLast);
        SameBool(o, aNRes, aPRes, w + ", BoundUV(S, L)");
        if (aPRes)
        {
            o.Same(aNFirst, aPFirst, w + ", BoundUV(S, L) first");
        }

        foreach (bool anAtEnd in new[] { false, true })
        {
            foreach (double aDParam in new[] { 0.0, 0.1 })
            {
                var aNPos = new N.gp_Pnt2d();
                var aNTan = new N.gp_Vec2d();
                aNRes = n.GetEndTangent2d(aNE, aNF, anAtEnd, aNPos, aNTan, aDParam);
                aPRes = p.GetEndTangent2d(aPE, aPF, anAtEnd, out Point2D aPPos, out Vector2D aPTan, aDParam);
                string t = $"{w}, GetEndTangent2d({anAtEnd}, {aDParam})";
                SameBool(o, aNRes, aPRes, t);
                if (aPRes)
                {
                    o.Same(aNPos, aPPos, t + " pos");
                    o.Same(aNTan, aPTan, t + " tangent");
                }
            }
        }

        // vertices against the curves
        foreach ((double aPreci, int aVtx) in new[] { (-1.0, 0), (1e-3, 0), (-1.0, 1), (1e-9, 2) })
        {
            string c = $"{w}, ({aPreci}, {aVtx})";
            SameBool(o, n.CheckVerticesWithCurve3d(aNE, aPreci, aVtx), p.CheckVerticesWithCurve3d(aPE, aPreci, aVtx), c + " CheckVerticesWithCurve3d");
            SameStatus(o, n, p, c + " CheckVerticesWithCurve3d");
            SameBool(o, n.CheckVerticesWithPCurve(aNE, aNF, aPreci, aVtx), p.CheckVerticesWithPCurve(aPE, aPF, aPreci, aVtx),
                     c + " CheckVerticesWithPCurve");
            SameStatus(o, n, p, c + " CheckVerticesWithPCurve");
            SameBool(o, n.CheckVerticesWithPCurve(aNE, aNSurf, aNLoc, aPreci, aVtx), p.CheckVerticesWithPCurve(aPE, aPSurf, aPLoc, aPreci, aVtx),
                     c + " CheckVerticesWithPCurve(S, L)");
            SameStatus(o, n, p, c + " CheckVerticesWithPCurve(S, L)");
        }

        double aNT1 = 0, aNT2 = 0;
        aNRes = n.CheckVertexTolerance(aNE, aNF, ref aNT1, ref aNT2);
        aPRes = p.CheckVertexTolerance(aPE, aPF, out double aPT1, out double aPT2);
        SameBool(o, aNRes, aPRes, w + ", CheckVertexTolerance(F)");
        SameStatus(o, n, p, w + ", CheckVertexTolerance(F)");
        if (!p.Status(ExtendStatus.FAIL1))
        {
            o.Same(aNT1, aPT1, w + ", CheckVertexTolerance(F) toler1");
            o.Same(aNT2, aPT2, w + ", CheckVertexTolerance(F) toler2");
        }
        aNRes = n.CheckVertexTolerance(aNE, ref aNT1, ref aNT2);
        aPRes = p.CheckVertexTolerance(aPE, out aPT1, out aPT2);
        SameBool(o, aNRes, aPRes, w + ", CheckVertexTolerance");
        SameStatus(o, n, p, w + ", CheckVertexTolerance");
        if (!p.Status(ExtendStatus.FAIL1))
        {
            o.Same(aNT1, aPT1, w + ", CheckVertexTolerance toler1");
            o.Same(aNT2, aPT2, w + ", CheckVertexTolerance toler2");
        }

        SameBool(o, n.CheckCurve3dWithPCurve(aNE, aNF), p.CheckCurve3dWithPCurve(aPE, aPF), w + ", CheckCurve3dWithPCurve");
        SameStatus(o, n, p, w + ", CheckCurve3dWithPCurve");

        // SameParameter, on the face and on every pcurve
        foreach (int aNb in new[] { 23, 10 })
        {
            double aNDev = -1, aPDev = -1;
            aNRes = n.CheckSameParameter(aNE, aNF, ref aNDev, aNb);
            aPRes = p.CheckSameParameter(aPE, aPF, ref aPDev, aNb);
            SameBool(o, aNRes, aPRes, $"{w}, CheckSameParameter(F, {aNb})");
            SameStatus(o, n, p, $"{w}, CheckSameParameter(F, {aNb})");
            o.Same(aNDev, aPDev, $"{w}, CheckSameParameter(F, {aNb}) maxdev");

            aNDev = aPDev = -1;
            aNRes = n.CheckSameParameter(aNE, ref aNDev, aNb);
            aPRes = p.CheckSameParameter(aPE, ref aPDev, aNb);
            SameBool(o, aNRes, aPRes, $"{w}, CheckSameParameter({aNb})");
            SameStatus(o, n, p, $"{w}, CheckSameParameter({aNb})");
            o.Same(aNDev, aPDev, $"{w}, CheckSameParameter({aNb}) maxdev");
        }

        // the pcurve's range, as it is and widened
        double aNF1 = 0, aNL1 = 0;
        N.Geom2d_Curve? aNPC = N.BRep_Tool.CurveOnSurface(aNE, aNF, ref aNF1, ref aNL1);
        Curve2D? aPPC = Tool.CurveOnSurface(aPE, aPF, out double aPF1, out double aPL1);
        SameBool(o, aNPC is null, aPPC is null, w + ", pcurve");
        if (aNPC is not null && aPPC is not null)
        {
            foreach (double aWiden in new[] { 0.0, 1e-9, 1e-3, 7.0 })
            {
                SameBool(o, n.CheckPCurveRange(aNF1 - aWiden, aNL1 + aWiden, aNPC), p.CheckPCurveRange(aPF1 - aWiden, aPL1 + aWiden, aPPC),
                         $"{w}, CheckPCurveRange({aWiden})");
            }
        }
    }
}
