// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// IntTools_Context: its queries on known shapes, and the cache audit (plan 2.4, decision 6410): a context reused
/// across calls, in any order, answers as a fresh context does, bit for bit.
/// </summary>
public class IntTools_ContextTests
{
    private static string Bits(double theValue) => BitConverter.DoubleToInt64Bits(theValue).ToString("X16");

    private static List<T> Sub<T>(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType, Func<TopoDS_Shape, T> theCast)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        var aList = new List<T>();
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            aList.Add(theCast(aMap.FindKey(i)));
        }
        return aList;
    }

    /// <summary>
    /// Queries of every kind the context caches a tool for, on random points near the shape's faces, edges and
    /// vertices; each returns its outputs as bits.
    /// </summary>
    private static List<Func<IntTools_Context, string>> Queries(in TopoDS_Shape theShape, int theSeed, int theNbPerKind)
    {
        var aRandom = new Random(theSeed);
        List<TopoDS_Face> aFaces = Sub(theShape, TopAbs_FACE, s => TopoDS.Face(s));
        List<TopoDS_Edge> anEdges = Sub(theShape, TopAbs_EDGE, s => TopoDS.Edge(s)).Where(e => !BRep_Tool.Degenerated(e)).ToList();
        List<TopoDS_Vertex> aVertices = Sub(theShape, TopAbs_VERTEX, s => TopoDS.Vertex(s));
        List<TopoDS_Solid> aSolids = Sub(theShape, TopAbs_SOLID, s => TopoDS.Solid(s));
        var aQueries = new List<Func<IntTools_Context, string>>();
        double Noise(double theScale) => (aRandom.NextDouble() - 0.5) * theScale;

        for (int i = 0; i < theNbPerKind; i++)
        {
            TopoDS_Face aF = aFaces[aRandom.Next(aFaces.Count)];
            BRepTools.UVBounds(aF, out double u1, out double u2, out double v1, out double v2);
            double du = u2 - u1, dv = v2 - v1;
            var aUV = new gp_Pnt2d(u1 - 0.1 * du + 1.2 * du * aRandom.NextDouble(), v1 - 0.1 * dv + 1.2 * dv * aRandom.NextDouble());
            Geom_Surface aS = BRep_Tool.Surface(aF)!;
            gp_Pnt aP = aS.Value(aUV.X(), aUV.Y());
            aP = new gp_Pnt(aP.X() + Noise(1e-3), aP.Y() + Noise(1e-3), aP.Z() + Noise(1e-3));
            aQueries.Add(c => $"S {c.StatePointFace(aF, aUV)} {c.IsPointInFace(aF, aUV)} {c.IsPointInOnFace(aF, aUV)}");
            aQueries.Add(c => $"F {c.FClass2d(aF).TestOnRestriction(aUV, 1e-3)} {c.FClass2d(aF).PerformInfinitePoint()}");
            aQueries.Add(c => $"V {c.IsValidPointForFace(aP, aF, 1e-3)} {c.IsPointInFace(aP, aF, 1e-3)}");
            gp_Pnt aPCopy = aP;
            aQueries.Add(c =>
            {
                GeomAPI_ProjectPointOnSurf aProj = c.ProjPS(aF);
                aProj.Perform(aPCopy);
                if (!aProj.IsDone())
                {
                    return "P -";
                }
                aProj.LowerDistanceParameters(out double u, out double v);
                return $"P {aProj.NbPoints()} {Bits(aProj.LowerDistance())} {Bits(u)} {Bits(v)}";
            });

            TopoDS_Edge anE = anEdges[aRandom.Next(anEdges.Count)];
            BRep_Tool.Range(anE, out double t1, out double t2);
            Geom_Curve aC = BRep_Tool.Curve(anE, out _, out _)!;
            gp_Pnt aPE = aC.Value(t1 + (t2 - t1) * (aRandom.NextDouble() * 1.2 - 0.1));
            aPE = new gp_Pnt(aPE.X() + Noise(1e-4), aPE.Y() + Noise(1e-4), aPE.Z() + Noise(1e-4));
            aQueries.Add(c =>
            {
                double aT = -1, aD = -1;
                int r = c.ComputePE(aPE, 1e-5, anE, ref aT, ref aD);
                double aT2 = -1;
                bool b = c.ProjectPointOnEdge(aPE, anE, ref aT2);
                return $"E {r} {Bits(aT)} {Bits(aD)} {b} {Bits(aT2)}";
            });

            TopoDS_Vertex aV = aVertices[aRandom.Next(aVertices.Count)];
            TopoDS_Edge anE2 = anEdges[aRandom.Next(anEdges.Count)];
            TopoDS_Face aF2 = aFaces[aRandom.Next(aFaces.Count)];
            aQueries.Add(c =>
            {
                double aT = -1, aTol = -1;
                int r = c.ComputeVE(aV, anE2, ref aT, ref aTol);
                double u = -1, v = -1, aTolF = -1;
                int r2 = c.ComputeVF(aV, aF2, ref u, ref v, ref aTolF);
                return $"VE {r} {Bits(aT)} {Bits(aTol)} VF {r2} {Bits(u)} {Bits(v)} {Bits(aTolF)}";
            });
            var anIC = new IntTools_Curve(aC, null, null);
            aQueries.Add(c =>
            {
                double aT = -1;
                bool b = c.IsVertexOnLine(aV, 1e-3, anIC, 1e-3, ref aT);
                return $"VL {b} {Bits(aT)}";
            });

            if (aSolids.Count > 0)
            {
                TopoDS_Solid aSo = aSolids[aRandom.Next(aSolids.Count)];
                gp_Pnt aPS = aRandom.Next(2) == 0 ? aP : aPE;
                aQueries.Add(c =>
                {
                    BRepClass3d_SolidClassifier aSC = c.SolidClassifier(aSo);
                    aSC.Perform(aPS, 1e-7);
                    return $"SC {aSC.State()}";
                });
            }
        }
        return aQueries;
    }

    private static void ReusedEqualsFresh(string theName, int theSeed, int theNbPerKind)
    {
        TopoDS_Shape aShape = BRepMeshModelKit.ReadDataOcc(theName);
        List<Func<IntTools_Context, string>> aQueries = Queries(aShape, theSeed, theNbPerKind);
        var aFresh = aQueries.Select(q => q(new IntTools_Context())).ToArray();

        // The same context for everything, in the queries' order and in a shuffled order
        var aContext = new IntTools_Context();
        for (int i = 0; i < aQueries.Count; i++)
        {
            Assert.True(aFresh[i] == aQueries[i](aContext), $"{theName}, query {i}: {aFresh[i]}");
        }
        var aRandom = new Random(theSeed + 1);
        int[] anOrder = Enumerable.Range(0, aQueries.Count).OrderBy(_ => aRandom.Next()).ToArray();
        var aShuffled = new IntTools_Context();
        foreach (int i in anOrder)
        {
            Assert.True(aFresh[i] == aQueries[i](aShuffled), $"{theName}, query {i} shuffled: {aFresh[i]}");
        }
        // The first context again, after all of that
        foreach (int i in anOrder.Reverse())
        {
            Assert.True(aFresh[i] == aQueries[i](aContext), $"{theName}, query {i} again: {aFresh[i]}");
        }
    }

    [Theory]
    [InlineData("bottle.brep", 64101)]
    [InlineData("Pump_Nut.brep", 64102)]
    [InlineData("wedge_ok.brep", 64103)]
    [InlineData("solid.brep", 64104)]
    [InlineData("hammer.brep", 64105)]
    public void ReusedContext_AnswersAsFreshOnes(string theName, int theSeed) => ReusedEqualsFresh(theName, theSeed, 60);

    [Fact]
    public void HoledSquare_Queries()
    {
        TopoDS_Face aF = ClassificationTestShapes.HoledSquare();
        var aContext = new IntTools_Context();
        Assert.Equal(TopAbs_IN, aContext.StatePointFace(aF, new gp_Pnt2d(1, 1)));
        Assert.Equal(TopAbs_OUT, aContext.StatePointFace(aF, new gp_Pnt2d(5, 5)));
        Assert.True(aContext.IsPointInFace(aF, new gp_Pnt2d(1, 1)));
        Assert.False(aContext.IsPointInFace(aF, new gp_Pnt2d(0, 1)));
        Assert.True(aContext.IsPointInOnFace(aF, new gp_Pnt2d(0, 1)));
        Assert.True(aContext.IsValidPointForFace(new gp_Pnt(1, 1, 1e-4), aF, 1e-3));
        Assert.False(aContext.IsValidPointForFace(new gp_Pnt(1, 1, 1e-2), aF, 1e-3));
        Assert.False(aContext.IsValidPointForFace(new gp_Pnt(5, 5, 0), aF, 1e-3));
        Assert.True(aContext.IsPointInFace(new gp_Pnt(1, 1, 1e-4), aF, 1e-3));
        // The classifier is cached per face, whatever its orientation
        TopoDS_Face aR = TopoDS.Face(aF.Reversed());
        Assert.Same(aContext.FClass2d(aF), aContext.FClass2d(aR));
        Assert.Same(aContext.ProjPS(aF), aContext.ProjPS(aF));
        Assert.Same(aContext.Hatcher(aF), aContext.Hatcher(aR));
        Assert.Same(aContext.SurfaceData(aF), aContext.SurfaceData(aF));
        aContext.UVBounds(aF, out double u1, out double u2, out double v1, out double v2);
        Assert.Equal((0.0, 10.0, 0.0, 10.0), (u1, u2, v1, v2));
        Assert.False(aContext.IsInfiniteFace(aF));
        // A new projection tolerance drops the cached surface projectors
        GeomAPI_ProjectPointOnSurf aProj = aContext.ProjPS(aF);
        aContext.SetPOnSProjectionTolerance(1e-9);
        Assert.NotSame(aProj, aContext.ProjPS(aF));
    }

    [Fact]
    public void Box_VertexEdgeFaceComputations()
    {
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(2, 3, 4).Shape();
        var aContext = new IntTools_Context();
        List<TopoDS_Vertex> aVs = Sub(aBox, TopAbs_VERTEX, s => TopoDS.Vertex(s));
        List<TopoDS_Edge> anEs = Sub(aBox, TopAbs_EDGE, s => TopoDS.Edge(s));
        List<TopoDS_Face> aFs = Sub(aBox, TopAbs_FACE, s => TopoDS.Face(s));
        // A vertex of an edge is on it, at an end of its range
        TopExp.Vertices(anEs[0], out TopoDS_Vertex aV1, out _);
        double aT = -1, aTol = -1;
        Assert.Equal(0, aContext.ComputeVE(aV1, anEs[0], ref aT, ref aTol));
        BRep_Tool.Range(anEs[0], out double f, out double l);
        Assert.True(aT == f || aT == l);
        // A vertex of a face projects onto its boundary: -3 (ON is not IN)
        double u = 0, v = 0, aTolF = 0;
        TopoDS_Vertex aVF = TopoDS.Vertex(new TopExp_Explorer(aFs[0], TopAbs_VERTEX).Current());
        Assert.Equal(-3, aContext.ComputeVF(aVF, aFs[0], ref u, ref v, ref aTolF));
        // A vertex far from an edge: -4
        TopoDS_Vertex aFar = aVs.First(x => BRep_Tool.Pnt(x).Distance(BRep_Tool.Pnt(aV1)) > 1);
        Assert.True(aContext.ComputeVE(aFar, anEs[0], ref aT, ref aTol) is 0 or -4);
        // The solid classifier
        TopoDS_Solid aSo = TopoDS.Solid(aBox);
        BRepClass3d_SolidClassifier aSC = aContext.SolidClassifier(aSo);
        aSC.Perform(new gp_Pnt(1, 1, 1), 1e-7);
        Assert.Equal(TopAbs_IN, aSC.State());
        aSC.Perform(new gp_Pnt(5, 1, 1), 1e-7);
        Assert.Equal(TopAbs_OUT, aSC.State());
        // IsVertexOnLine on the edge's own curve
        Geom_Curve aC = BRep_Tool.Curve(anEs[0], out _, out _)!;
        double aTL = -1;
        Assert.True(aContext.IsVertexOnLine(aV1, new IntTools_Curve(new Geom_TrimmedCurve(aC, f, l), null, null), 1e-7, ref aTL));
        // IsValidBlockForFace(s): the middle of an edge lies on its two faces
        var anIC = new IntTools_Curve(aC, null, null);
        int nF = 0;
        foreach (TopoDS_Face aF in aFs)
        {
            if (aContext.IsValidBlockForFace(f, l, anIC, aF, 1e-7))
            {
                nF++;
            }
        }
        Assert.Equal(2, nF);
        Assert.True(aContext.BndBox(aBox).IsVoid() == false);
    }
}
