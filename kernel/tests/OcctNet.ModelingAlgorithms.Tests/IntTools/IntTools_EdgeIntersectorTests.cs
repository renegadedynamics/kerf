// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;
using static OcctNet.ModelingAlgorithms.Tests.EdgeIntersectorRecordings;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// IntTools_EdgeEdge, IntTools_EdgeFace and IntTools_BeanFaceIntersector on known configurations, and the edge/face
/// intersector run as a pave filler runs it: through BOPTools_Parallel with a context per worker, serially and in
/// parallel at several degrees, with edges of different tolerances on the same faces (decision 753).
/// </summary>
public class IntTools_EdgeIntersectorTests
{
    private static global::Kerf.Topology.Edge Segment(double x1, double y1, double z1, double x2, double y2, double z2) =>
        new BRepBuilderAPI_MakeEdge(new Point3D(x1, y1, z1), new Point3D(x2, y2, z2)).Edge();

    private static global::Kerf.Topology.Face PlaneFace(double theSize) =>
        new BRepBuilderAPI_MakeFace(new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), -theSize, theSize, -theSize, theSize).Face();

    [Fact]
    public void EdgeEdge_CrossingLines_GiveOneVertex()
    {
        IntTools_EdgeEdge anEE = new(Segment(-1, 0, 0, 1, 0, 0), Segment(0, -1, 0, 0, 1, 0));
        anEE.Perform();
        Assert.True(anEE.IsDone());
        Assert.Equal(1, anEE.CommonParts().Length());
        IntTools_CommonPrt aCP = anEE.CommonParts().Value(1);
        Assert.Equal(ShapeType.Vertex, aCP.Type());
        Assert.Equal(1.0, aCP.VertexParameter1(), 1e-12);
        Assert.Equal(1.0, aCP.VertexParameter2(), 1e-12);
    }

    [Fact]
    public void EdgeEdge_OverlappingLines_GiveOneEdge()
    {
        IntTools_EdgeEdge anEE = new(Segment(0, 0, 0, 2, 0, 0), Segment(1, 0, 0, 3, 0, 0));
        anEE.Perform();
        Assert.Equal(1, anEE.CommonParts().Length());
        IntTools_CommonPrt aCP = anEE.CommonParts().Value(1);
        Assert.Equal(global::Kerf.Topology.ShapeType.Edge, aCP.Type());
        aCP.Range1(out double f, out double l);
        Assert.Equal(1.0, f, 1e-12);
        Assert.Equal(2.0, l, 1e-12);
        Assert.Equal(0.0, aCP.Ranges2().Value(1).First(), 1e-12);
        Assert.Equal(1.0, aCP.Ranges2().Value(1).Last(), 1e-12);
    }

    [Fact]
    public void EdgeEdge_DisjointEdges_GiveNothing_AndNullEdgeIsNotDone()
    {
        IntTools_EdgeEdge anEE = new(Segment(0, 0, 0, 1, 0, 0), Segment(0, 1, 1, 1, 1, 1));
        anEE.Perform();
        Assert.True(anEE.IsDone());
        Assert.Equal(0, anEE.CommonParts().Length());

        IntTools_EdgeEdge aNull = new();
        aNull.SetEdge1(Segment(0, 0, 0, 1, 0, 0));
        aNull.Perform();
        Assert.False(aNull.IsDone());
    }

    [Fact]
    public void EdgeEdge_CircleAndLine_GiveTwoVertices_InEdge1sParameters()
    {
        global::Kerf.Topology.Edge aCircle = new BRepBuilderAPI_MakeEdge(new Circle3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 1.0)).Edge();
        global::Kerf.Topology.Edge aLine = Segment(-2, 0.5, 0, 2, 0.5, 0);
        IntTools_EdgeEdge anEE = new(aLine, aCircle);
        anEE.Perform();
        Assert.Equal(2, anEE.CommonParts().Length());
        foreach (IntTools_CommonPrt aCP in anEE.CommonParts())
        {
            Assert.Equal(ShapeType.Vertex, aCP.Type());
            Assert.True(aCP.Edge1().IsSame(aLine)); // the edges are swapped inside, not in the common parts
            Point3D aP = Tool.Curve(aLine, out _, out _)!.Value(aCP.VertexParameter1());
            Assert.Equal(1.0, Math.Sqrt(aP.X() * aP.X() + aP.Y() * aP.Y()), 1e-6);
        }
    }

    [Fact]
    public void EdgeFace_LineThroughPlane_GivesOneVertex_LineInPlane_GivesTheEdge()
    {
        global::Kerf.Topology.Face aF = PlaneFace(5);
        foreach ((global::Kerf.Topology.Edge E, ShapeType Type, double Param) in new[]
                 {
                     (Segment(1, 1, -1, 1, 1, 3), ShapeType.Vertex, 1.0),
                     (Segment(-1, 0, 0, 3, 0, 0), global::Kerf.Topology.ShapeType.Edge, 0.0),
                 })
        {
            IntTools_EdgeFace anEF = new();
            anEF.SetEdge(E);
            anEF.SetFace(aF);
            Tool.Range(E, out double f, out double l);
            anEF.SetRange(f, l);
            anEF.Perform();
            Assert.True(anEF.IsDone());
            Assert.Equal(0, anEF.ErrorStatus());
            Assert.Equal(1, anEF.CommonParts().Length());
            Assert.Equal(Type, anEF.CommonParts().Value(1).Type());
            if (Type == ShapeType.Vertex)
            {
                Assert.Equal(Param, anEF.CommonParts().Value(1).VertexParameter1(), 1e-9);
            }
        }
    }

    [Fact]
    public void EdgeFace_NotStarted_AndDegenerated()
    {
        IntTools_EdgeFace anEF = new();
        Assert.False(anEF.IsDone());
        Assert.Equal(1, anEF.ErrorStatus());
        Assert.Equal(RealLast(), anEF.MinimalDistance());
    }

    [Fact]
    public void BeanFace_CircleOnCylinder_IsCoincident_LineMissingSphere_IsEmpty()
    {
        global::Kerf.Topology.Face aCyl = ShapeCasts.Face(new Explorer(new BRepPrimAPI_MakeCylinder(1, 2).Shape(), global::Kerf.Topology.ShapeType.Face).Current());
        global::Kerf.Topology.Edge aCircle = new BRepBuilderAPI_MakeEdge(new Circle3D(new Frame3D(new Point3D(0, 0, 1), new Direction3D(0, 0, 1)), 1.0)).Edge();
        IntTools_BeanFaceIntersector aBF = new(aCircle, aCyl);
        Tool.Range(aCircle, out double f, out double l);
        aBF.SetBeanParameters(f, l);
        aBF.Perform();
        Assert.True(aBF.IsDone());
        Assert.Equal(1, aBF.Result().Length());
        Assert.Equal(f, aBF.Result().Value(1).First());
        Assert.Equal(l, aBF.Result().Value(1).Last());
        NCollection_Sequence<IntTools_Range> aCopy = new();
        aBF.Result(aCopy);
        Assert.Equal(1, aCopy.Length());

        global::Kerf.Topology.Face aSph = ShapeCasts.Face(new Explorer(new BRepPrimAPI_MakeSphere(1).Shape(), global::Kerf.Topology.ShapeType.Face).Current());
        global::Kerf.Topology.Edge aLine = Segment(-3, 2, 0, 3, 2, 0);
        IntTools_BeanFaceIntersector aBF2 = new(aLine, aSph);
        aBF2.SetBeanParameters(0, 6);
        aBF2.Perform();
        Assert.True(aBF2.IsDone());
        Assert.Equal(0, aBF2.Result().Length());
    }

    /// <summary>An edge/face pair as BOPAlgo_EdgeFace runs it: through BOPTools_Parallel with the worker's context.</summary>
    private sealed class EFSolver(global::Kerf.Topology.Edge theE, global::Kerf.Topology.Face theF, double theFuzz)
        : BOPTools_Parallel.IContextSolver<IntTools_Context>
    {
        private IntTools_Context? myContext;

        public string Result { get; private set; } = "";

        public EFSolver Fresh() => new(theE, theF, theFuzz);

        public void SetContext(IntTools_Context theContext) => myContext = theContext;

        public void Perform() => Result = RunEF(theE, theF, myContext!, theFuzz);
    }

    private static List<string> Run(List<EFSolver> theTemplate, bool theParallel)
    {
        List<EFSolver> aSolvers = [.. theTemplate.Select(s => s.Fresh())];
        BOPTools_Parallel.Perform(theParallel, aSolvers, new IntTools_Context());
        return [.. aSolvers.Select(s => s.Result)];
    }

    /// <summary>
    /// The edge/face pairs of a recorded case on faces where the intersector caches boxes, the probe edges' tolerances
    /// 1e-7, 1e-5 and 1e-3 and every other pair with a fuzzy value of 1e-4, so that edges of different criteria meet
    /// the same faces in a different order on each worker: parallel runs give the serial result, which is each pair's
    /// fresh-context result.
    /// </summary>
    [Theory]
    [InlineData("mat-probes")]
    [InlineData("shell1-probes")]
    public void EdgeFace_ThroughBOPToolsParallel_GivesTheSerialResult(string theCase)
    {
        Case aCase = Cases[theCase];
        List<EFSolver> aTemplate = [];
        int k = 0;
        foreach (EFRun aRun in aCase.EF.Where(r => r.Localized).Take(120))
        {
            aTemplate.Add(new EFSolver(Edge(aCase, aCase.EA, aRun.I), Face(aCase, aRun.J), k++ % 2 == 1 ? 1.0e-4 : 0.0));
        }
        Assert.NotEmpty(aTemplate);
        List<string> aSerial;
        using (OsdParallel.ForceSerialScope())
        {
            aSerial = Run(aTemplate, true);
        }
        List<string> aFresh = [.. aTemplate.Select(s =>
        {
            EFSolver aS = s.Fresh();
            aS.SetContext(new IntTools_Context());
            aS.Perform();
            return aS.Result;
        })];
        Assert.Equal(aFresh, aSerial);
        foreach (int aDegree in new[] { 2, 4, Environment.ProcessorCount })
        {
            using (OsdParallel.MaxDegreeOfParallelismScope(aDegree))
            {
                Assert.Equal(aSerial, Run(aTemplate, true));
            }
        }
    }
}
