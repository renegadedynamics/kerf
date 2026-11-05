// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// IntTools_Context: its queries on known shapes, and the cache audit (plan 2.4, decision 723): a context reused
/// across calls, in any order, answers as a fresh context does, bit for bit.
/// </summary>
public class IntTools_ContextTests
{
    private static string Bits(double theValue) => BitConverter.DoubleToInt64Bits(theValue).ToString("X16");

    private static List<T> Sub<T>(in Shape theShape, ShapeType theType, Func<Shape, T> theCast)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        var aList = new List<T>();
        for (int i = 0; i < aMap.Count; i++)
        {
            aList.Add(theCast(aMap[i]));
        }
        return aList;
    }

    /// <summary>
    /// Queries of every kind the context caches a tool for, on random points near the shape's faces, edges and
    /// vertices; each returns its outputs as bits.
    /// </summary>
    private static List<Func<IntersectionContext, string>> Queries(in Shape theShape, int theSeed, int theNbPerKind)
    {
        var aRandom = new Random(theSeed);
        List<Face> aFaces = Sub(theShape, ShapeType.Face, s => ShapeCasts.Face(s));
        List<Edge> anEdges = Sub(theShape, ShapeType.Edge, s => ShapeCasts.Edge(s)).Where(e => !Tool.Degenerated(e)).ToList();
        List<Vertex> aVertices = Sub(theShape, ShapeType.Vertex, s => ShapeCasts.Vertex(s));
        List<Solid> aSolids = Sub(theShape, ShapeType.Solid, s => ShapeCasts.Solid(s));
        var aQueries = new List<Func<IntersectionContext, string>>();
        double Noise(double theScale) => (aRandom.NextDouble() - 0.5) * theScale;

        for (int i = 0; i < theNbPerKind; i++)
        {
            Face aF = aFaces[aRandom.Next(aFaces.Count)];
            BrepTools.UVBounds(aF, out double u1, out double u2, out double v1, out double v2);
            double du = u2 - u1, dv = v2 - v1;
            var aUV = new Point2D(u1 - 0.1 * du + 1.2 * du * aRandom.NextDouble(), v1 - 0.1 * dv + 1.2 * dv * aRandom.NextDouble());
            Surface aS = Tool.Surface(aF)!;
            Point3D aP = aS.Value(aUV.X, aUV.Y);
            aP = new Point3D(aP.X + Noise(1e-3), aP.Y + Noise(1e-3), aP.Z + Noise(1e-3));
            aQueries.Add(c => $"S {c.StatePointFace(aF, aUV)} {c.IsPointInFace(aF, aUV)} {c.IsPointInOnFace(aF, aUV)}");
            aQueries.Add(c => $"F {c.FClass2d(aF).TestOnRestriction(aUV, 1e-3)} {c.FClass2d(aF).PerformInfinitePoint()}");
            aQueries.Add(c => $"V {c.IsValidPointForFace(aP, aF, 1e-3)} {c.IsPointInFace(aP, aF, 1e-3)}");
            Point3D aPCopy = aP;
            aQueries.Add(c =>
            {
                ProjectPointOnSurface aProj = c.ProjPS(aF);
                aProj.Perform(aPCopy);
                if (!aProj.IsDone)
                {
                    return "P -";
                }
                aProj.LowerDistanceParameters(out double u, out double v);
                return $"P {aProj.NbPoints()} {Bits(aProj.LowerDistance())} {Bits(u)} {Bits(v)}";
            });

            Edge anE = anEdges[aRandom.Next(anEdges.Count)];
            Tool.Range(anE, out double t1, out double t2);
            Curve3D aC = Tool.Curve(anE, out _, out _)!;
            Point3D aPE = aC.Value(t1 + (t2 - t1) * (aRandom.NextDouble() * 1.2 - 0.1));
            aPE = new Point3D(aPE.X + Noise(1e-4), aPE.Y + Noise(1e-4), aPE.Z + Noise(1e-4));
            aQueries.Add(c =>
            {
                double aT = -1, aD = -1;
                int r = c.ComputePE(aPE, 1e-5, anE, ref aT, ref aD);
                double aT2 = -1;
                bool b = c.ProjectPointOnEdge(aPE, anE, ref aT2);
                return $"E {r} {Bits(aT)} {Bits(aD)} {b} {Bits(aT2)}";
            });

            Vertex aV = aVertices[aRandom.Next(aVertices.Count)];
            Edge anE2 = anEdges[aRandom.Next(anEdges.Count)];
            Face aF2 = aFaces[aRandom.Next(aFaces.Count)];
            aQueries.Add(c =>
            {
                double aT = -1, aTol = -1;
                int r = c.ComputeVE(aV, anE2, ref aT, ref aTol);
                double u = -1, v = -1, aTolF = -1;
                int r2 = c.ComputeVF(aV, aF2, ref u, ref v, ref aTolF);
                return $"VE {r} {Bits(aT)} {Bits(aTol)} VF {r2} {Bits(u)} {Bits(v)} {Bits(aTolF)}";
            });
            var anIC = new IntersectionCurve(aC, null, null);
            aQueries.Add(c =>
            {
                double aT = -1;
                bool b = c.IsVertexOnLine(aV, 1e-3, anIC, 1e-3, ref aT);
                return $"VL {b} {Bits(aT)}";
            });

            if (aSolids.Count > 0)
            {
                Solid aSo = aSolids[aRandom.Next(aSolids.Count)];
                Point3D aPS = aRandom.Next(2) == 0 ? aP : aPE;
                aQueries.Add(c =>
                {
                    SolidClassifier aSC = c.SolidClassifier(aSo);
                    aSC.Perform(aPS, 1e-7);
                    return $"SC {aSC.State()}";
                });
            }
        }
        return aQueries;
    }

    private static void ReusedEqualsFresh(string theName, int theSeed, int theNbPerKind)
    {
        Shape aShape = BRepMeshModelKit.ReadDataOcc(theName);
        List<Func<IntersectionContext, string>> aQueries = Queries(aShape, theSeed, theNbPerKind);
        var aFresh = aQueries.Select(q => q(new IntersectionContext())).ToArray();

        // The same context for everything, in the queries' order and in a shuffled order
        var aContext = new IntersectionContext();
        for (int i = 0; i < aQueries.Count; i++)
        {
            Assert.True(aFresh[i] == aQueries[i](aContext), $"{theName}, query {i}: {aFresh[i]}");
        }
        var aRandom = new Random(theSeed + 1);
        int[] anOrder = Enumerable.Range(0, aQueries.Count).OrderBy(_ => aRandom.Next()).ToArray();
        var aShuffled = new IntersectionContext();
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
        Face aF = ClassificationTestShapes.HoledSquare();
        var aContext = new IntersectionContext();
        Assert.Equal(In, aContext.StatePointFace(aF, new Point2D(1, 1)));
        Assert.Equal(Out, aContext.StatePointFace(aF, new Point2D(5, 5)));
        Assert.True(aContext.IsPointInFace(aF, new Point2D(1, 1)));
        Assert.False(aContext.IsPointInFace(aF, new Point2D(0, 1)));
        Assert.True(aContext.IsPointInOnFace(aF, new Point2D(0, 1)));
        Assert.True(aContext.IsValidPointForFace(new Point3D(1, 1, 1e-4), aF, 1e-3));
        Assert.False(aContext.IsValidPointForFace(new Point3D(1, 1, 1e-2), aF, 1e-3));
        Assert.False(aContext.IsValidPointForFace(new Point3D(5, 5, 0), aF, 1e-3));
        Assert.True(aContext.IsPointInFace(new Point3D(1, 1, 1e-4), aF, 1e-3));
        // The classifier is cached per face, whatever its orientation
        Face aR = ShapeCasts.Face(aF.Reversed());
        Assert.Same(aContext.FClass2d(aF), aContext.FClass2d(aR));
        Assert.Same(aContext.ProjPS(aF), aContext.ProjPS(aF));
        Assert.Same(aContext.Hatcher(aF), aContext.Hatcher(aR));
        Assert.Same(aContext.SurfaceData(aF), aContext.SurfaceData(aF));
        aContext.UVBounds(aF, out double u1, out double u2, out double v1, out double v2);
        Assert.Equal((0.0, 10.0, 0.0, 10.0), (u1, u2, v1, v2));
        Assert.False(aContext.IsInfiniteFace(aF));
        // A new projection tolerance drops the cached surface projectors
        ProjectPointOnSurface aProj = aContext.ProjPS(aF);
        aContext.SetPOnSProjectionTolerance(1e-9);
        Assert.NotSame(aProj, aContext.ProjPS(aF));
    }

    [Fact]
    public void Box_VertexEdgeFaceComputations()
    {
        Shape aBox = new MakeBox(2, 3, 4).Shape();
        var aContext = new IntersectionContext();
        List<Vertex> aVs = Sub(aBox, ShapeType.Vertex, s => ShapeCasts.Vertex(s));
        List<Edge> anEs = Sub(aBox, ShapeType.Edge, s => ShapeCasts.Edge(s));
        List<Face> aFs = Sub(aBox, ShapeType.Face, s => ShapeCasts.Face(s));
        // A vertex of an edge is on it, at an end of its range
        Exploration.Vertices(anEs[0], out Vertex aV1, out _);
        double aT = -1, aTol = -1;
        Assert.Equal(0, aContext.ComputeVE(aV1, anEs[0], ref aT, ref aTol));
        Tool.Range(anEs[0], out double f, out double l);
        Assert.True(aT == f || aT == l);
        // A vertex of a face projects onto its boundary: -3 (ON is not IN)
        double u = 0, v = 0, aTolF = 0;
        Vertex aVF = ShapeCasts.Vertex(new Explorer(aFs[0], ShapeType.Vertex).Current());
        Assert.Equal(-3, aContext.ComputeVF(aVF, aFs[0], ref u, ref v, ref aTolF));
        // A vertex far from an edge: -4
        Vertex aFar = aVs.First(x => Tool.Pnt(x).DistanceTo(Tool.Pnt(aV1)) > 1);
        Assert.True(aContext.ComputeVE(aFar, anEs[0], ref aT, ref aTol) is 0 or -4);
        // The solid classifier
        Solid aSo = ShapeCasts.Solid(aBox);
        SolidClassifier aSC = aContext.SolidClassifier(aSo);
        aSC.Perform(new Point3D(1, 1, 1), 1e-7);
        Assert.Equal(In, aSC.State());
        aSC.Perform(new Point3D(5, 1, 1), 1e-7);
        Assert.Equal(Out, aSC.State());
        // IsVertexOnLine on the edge's own curve
        Curve3D aC = Tool.Curve(anEs[0], out _, out _)!;
        double aTL = -1;
        Assert.True(aContext.IsVertexOnLine(aV1, new IntersectionCurve(new TrimmedCurve3D(aC, f, l), null, null), 1e-7, ref aTL));
        // IsValidBlockForFace(s): the middle of an edge lies on its two faces
        var anIC = new IntersectionCurve(aC, null, null);
        int nF = 0;
        foreach (Face aF in aFs)
        {
            if (aContext.IsValidBlockForFace(f, l, anIC, aF, 1e-7))
            {
                nF++;
            }
        }
        Assert.Equal(2, nF);
        Assert.True(aContext.BndBox(aBox).IsVoid == false);
    }
}
