// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.HiddenLines;
using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Engine.HiddenLines.Contours;
using Kerf.Engine.HiddenLines.Topology;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Unit tests of HLR's data model: the projected curves and surfaces, the edge and face data and their copies, the
/// shape bounds, the interference lists, HLRTopoBRep's data, the outlining's copy at the entry of a frozen shape
/// (decision 1095), and HLRBRep_Data's merging. The native comparisons are <see cref="HLRBRepDataNativeTests"/>.
/// </summary>
public class HLRBRepTests
{
    private static readonly AlgorithmProjector Axo =
        new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(1, -1, 1), new Direction3D(-1, 1, 2)));

    private static readonly AlgorithmProjector Persp =
        new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(1, -1, 1), new Direction3D(-1, 1, 2)), 400.0);

    private static string BRepText(Shape theShape)
    {
        StringWriter aWriter = new();
        BrepTools.Write(theShape, aWriter);
        return aWriter.ToString();
    }

    private static HiddenLineData Load(Shape theShape, AlgorithmProjector P, int nbIso,
                                     out OutLiner theOutLiner)
    {
        theOutLiner = new OutLiner(theShape);
        Dictionary<Shape, AdaptorTool> MST = new(ShapeHasher.Instance);
        HiddenLineData DS = ShapeToHlr.Load(theOutLiner, P, MST, nbIso);
        DS.Update(P);
        return DS;
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Curve_ProjectsAndConvertsParameters(bool thePerspective)
    {
        AlgorithmProjector P = thePerspective ? Persp : Axo;
        Shape aBox = new MakeBox(10.0, 20.0, 30.0).Shape();
        HiddenLineData DS = Load(aBox, P, 0, out _);
        Assert.Equal(12, DS.EdgeCount);
        Assert.Equal(6, DS.FaceCount);
        Assert.Equal(8, DS.VertexCount);
        for (int e = 1; e <= DS.EdgeCount; e++)
        {
            HiddenLineCurve C = DS.EDataArray()[e].Geometry();
            Assert.Equal(Line, C.GetType());
            double f3 = C.Curve().FirstParameter(), l3 = C.Curve().LastParameter();
            foreach (double t in new[] { 0.0, 0.3, 1.0 })
            {
                double p3 = f3 + (l3 - f3) * t;
                // Parameter3d inverts Parameter2d
                Assert.Equal(p3, C.Parameter3d(C.Parameter2d(p3)), 9);
                // the 2D point is the projection of the 3D point
                Point2D aExpected = new();
                P.Project(C.Value3D(p3), ref aExpected);
                Point2D a2d = C.Value(p3);
                Assert.True(a2d.DistanceTo(aExpected) < 1e-9);
            }
            // D1 against a finite difference of D0
            double pm = 0.5 * (f3 + l3), h = 1e-6;
            C.D1(pm, out Point2D _, out Vector2D V);
            Point2D aA = C.Value(pm - h), aB = C.Value(pm + h);
            Assert.True(Math.Abs(V.X - (aB.X - aA.X) / (2 * h)) < 1e-5);
            Assert.True(Math.Abs(V.Y - (aB.Y - aA.Y) / (2 * h)) < 1e-5);
        }
    }

    [Fact]
    public void Curve_CircleSeenObliquelyIsAnEllipse()
    {
        Shape aCyl = new ApiMakeCylinder(10.0, 30.0).Shape();
        HiddenLineData DS = Load(aCyl, Axo, 0, out _);
        int nbEllipses = 0;
        for (int e = 1; e <= DS.EdgeCount; e++)
        {
            HiddenLineCurve C = DS.EDataArray()[e].Geometry();
            if (C.GetType() != Ellipse)
            {
                continue;
            }
            nbEllipses++;
            Ellipse2D El = C.Ellipse();
            double f = C.FirstParameter(), l = C.LastParameter();
            foreach (double t in new[] { 0.0, 0.25, 0.8 })
            {
                // the 2D parameter is the ellipse's own parameter
                double p2 = f + (l - f) * t;
                Point2D aOnEllipse = global::Kerf.Engine.Geometry.ElCLib.Value(p2, El);
                Assert.True(aOnEllipse.DistanceTo(C.Value(C.Parameter3d(p2))) < 1e-9);
            }
        }
        Assert.True(nbEllipses >= 2); // the circles' arcs, split at the silhouette's ends
    }

    [Fact]
    public void Surface_PlaneSeenEdgeOnIsSide()
    {
        AlgorithmProjector aTop = new(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        Shape aBox = new MakeBox(10.0, 20.0, 30.0).Shape();
        HiddenLineData DS = Load(aBox, aTop, 0, out _);
        int nbSide = 0, nbHiding = 0;
        for (int f = 1; f <= DS.FaceCount; f++)
        {
            HiddenLineFaceData fd = DS.FDataArray()[f];
            Assert.True(fd.Plane(), $"plane {f}");
            nbSide += fd.Side() ? 1 : 0;
            nbHiding += fd.Hiding() ? 1 : 0;
        }
        // seen from the top, the four lateral faces are edge-on; the top face hides, the bottom one is behind it
        Assert.Equal(4, nbSide);
        Assert.Equal(1, nbHiding);
    }

    [Fact]
    public void EdgeData_AssignCopiesTheStatusAndTheCurve()
    {
        Shape aBox = new MakeBox(10.0, 20.0, 30.0).Shape();
        HiddenLineData DS = Load(aBox, Axo, 0, out _);
        HiddenLineEdgeData aSource = DS.EDataArray()[1];
        HiddenLineEdgeData aCopy = new();
        aCopy.Assign(aSource);
        Assert.NotSame(aSource.Status(), aCopy.Status());
        Assert.NotSame(aSource.Geometry(), aCopy.Geometry());
        Assert.Equal(aSource.VSta(), aCopy.VSta());
        Assert.Equal(aSource.MinMax(), aCopy.MinMax());
        aCopy.Status().HideAll();
        Assert.True(aCopy.Status().AllHidden());
        Assert.False(aSource.Status().AllHidden());
        Assert.Equal(aSource.Geometry().Value(0.5).X, aCopy.Geometry().Value(0.5).X);
    }

    [Fact]
    public void Data_WriteShiftsTheIndicesAndSharesTheWires()
    {
        Shape aBox = new MakeBox(10.0, 20.0, 30.0).Shape();
        HiddenLineData A = Load(aBox, Axo, 0, out _);
        HiddenLineData B = Load(new MakeBox(new Point3D(50, 0, 0), 5.0, 5.0, 5.0).Shape(), Axo, 0, out _);
        HiddenLineData AB = new(A.VertexCount + B.VertexCount, A.EdgeCount + B.EdgeCount, A.FaceCount + B.FaceCount);
        AB.Write(A, 0, 0, 0);
        int aBefore = B.FDataArray()[1].Wires()!.Wire(1)!.Edge(1);
        AB.Write(B, A.VertexCount, A.EdgeCount, A.FaceCount);
        Assert.Equal(24, AB.EdgeMap().Count);
        Assert.Equal(12, AB.FaceMap().Count);
        Assert.Equal(B.EDataArray()[1].VSta() + A.VertexCount, AB.EDataArray()[13].VSta());
        // OCCT: the copied face shares B's wires, whose edge indices are shifted too
        Assert.Same(B.FDataArray()[1].Wires(), AB.FDataArray()[7].Wires());
        Assert.Equal(aBefore + A.EdgeCount, B.FDataArray()[1].Wires()!.Wire(1)!.Edge(1));
    }

    [Fact]
    public void ShapeBounds_TranslateAndSizes()
    {
        ShapeBounds SB = new(new OutLiner(), 2, 1, 8, 1, 12, 1, 6);
        SB.Translate(10, 20, 30);
        SB.Sizes(out int NV, out int NE, out int NF);
        Assert.Equal((8, 12, 6), (NV, NE, NF));
        SB.Bounds(out int V1, out int V2, out int E1, out int E2, out int F1, out int F2);
        Assert.Equal((11, 18, 21, 32, 31, 36), (V1, V2, E1, E2, F1, F2));
        Assert.Equal(2, new ShapeBounds(SB).OfIsoCount);
    }

    [Fact]
    public void EdgeIList_AddInterferenceSortsByParameter()
    {
        Shape aBox = new MakeBox(10.0, 20.0, 30.0).Shape();
        HiddenLineData DS = Load(aBox, Axo, 0, out _);
        EdgeInterferenceTool T = new(DS);
        NCollection_List<Interference> IL = new();
        foreach (double p in new[] { 3.0, 1.0, 2.0, 1.0, 5.0 })
        {
            AlgoIntersection X = new(Internal, 1, 0, 0, p, 0.0f, Containment.In);
            EdgeIList.AddInterference(IL, new Interference(X, new Coincidence(),
                                                                          Forward, Forward,
                                                                          Forward), T);
        }
        Assert.Equal([1.0, 1.0, 2.0, 3.0, 5.0], IL.Select(I => I.Intersection().Parameter()).ToArray());
    }

    [Fact]
    public void TopoData_InsertsVerticesByParameter()
    {
        TopologyData DS = new();
        Edge E = ShapeCasts.Edge(new Explorer(new MakeBox(1.0, 1.0, 1.0).Shape(), ShapeType.Edge).Current());
        BrepBuilder B = new();
        double[] aPars = [0.5, 0.2, 0.8, 0.3];
        foreach (double p in aPars)
        {
            Vertex V = new();
            B.MakeVertex(ref V, new Point3D(p, 0, 0), 1e-7);
            bool inserted = false;
            for (DS.InitVertex(E); DS.MoreVertex(); DS.NextVertex())
            {
                if (p < DS.Parameter())
                {
                    DS.InsertBefore(V, p);
                    inserted = true;
                    break;
                }
            }
            if (!inserted)
            {
                DS.Append(V, p);
            }
        }
        List<double> aRead = [];
        for (DS.InitVertex(E); DS.MoreVertex(); DS.NextVertex())
        {
            aRead.Add(DS.Parameter());
        }
        Assert.Equal([0.2, 0.3, 0.5, 0.8], aRead);
        DS.InitEdge();
        Assert.True(DS.MoreEdge());
        Assert.True(DS.Edge().IsSame(E));
        DS.NextEdge();
        Assert.False(DS.MoreEdge());
    }

    [Fact]
    public void OutLiner_FrozenInputIsOutlinedInAWorkingOverlayAndUnchanged()
    {
        Shape aCyl = new ApiMakeCylinder(10.0, 30.0).Shape(); // frozen
        Assert.True(aCyl.TShape()!.IsFrozen);
        string aBefore = BRepText(aCyl);
        HiddenLineData DS = Load(aCyl, Axo, 3, out OutLiner S);
        Assert.Equal(aBefore, BRepText(aCyl));
        Assert.True(S.OriginalShape().IsEqual(aCyl));
        // the outlining split the circles at the silhouette's ends and added isolines; its data structure holds the
        // caller's own faces and edges (unit 10H6: it worked on them in a working overlay)
        TopologyData TopDS = S.DataStructure();
        int nbSplit = 0, nbIso = 0, nbIntL = 0;
        for (Explorer ex = new(aCyl, ShapeType.Face); ex.More(); ex.Next())
        {
            Face F = ShapeCasts.Face(ex.Current());
            nbIso += TopDS.FaceHasIsoL(F) ? TopDS.FaceIsoL(F).Extent() : 0;
            nbIntL += TopDS.FaceHasIntL(F) ? TopDS.FaceIntL(F).Extent() : 0;
            for (Explorer exE = new(F, ShapeType.Edge); exE.More(); exE.Next())
            {
                nbSplit += TopDS.EdgeHasSplE(ShapeCasts.Edge(exE.Current())) ? 1 : 0;
            }
        }
        Assert.True(nbSplit > 0);
        Assert.True(nbIso > 0);
        Assert.Equal(2, nbIntL);
        Assert.True(DS.EdgeCount > 3);
        // the outlined shape is published frozen
        Assert.True(S.OutLinedShape().TShape()!.IsFrozen);
    }

    [Fact]
    public void OutLiner_UnfrozenInputIsWrittenInPlaceWithTheSameResult()
    {
        Shape aFrozen = new ApiMakeCylinder(10.0, 30.0).Shape();
        Shape aWork = TestCopies.ExactCopyForWrite(aFrozen, out _);
        Assert.False(aWork.TShape()!.IsFrozen);

        HiddenLineData DS1 = Load(aFrozen, Axo, 3, out OutLiner S1);
        HiddenLineData DS2 = Load(aWork, Axo, 3, out OutLiner S2);
        // an unfrozen shape is outlined in place, as OCCT does (the split edges' end vertices are updated)
        Assert.True(S2.OriginalShape().IsSame(aWork));
        // and the data is the same as the frozen shape's overlay gives
        Assert.Equal(DS1.EdgeCount, DS2.EdgeCount);
        for (int e = 1; e <= DS1.EdgeCount; e++)
        {
            HiddenLineEdgeData a = DS1.EDataArray()[e], b = DS2.EDataArray()[e];
            Assert.Equal(a.MinMax(), b.MinMax());
            Assert.Equal((a.VSta(), a.VEnd(), a.Vertical()), (b.VSta(), b.VEnd(), b.Vertical()));
        }
        Assert.Equal(BRepText(S1.InternalOutLinedShape()).Length, BRepText(S2.InternalOutLinedShape()).Length);
    }

    [Fact]
    public void DSFiller_FrozenShapeThrows()
    {
        Shape aCyl = new ApiMakeCylinder(10.0, 30.0).Shape();
        TopologyData DS = new();
        Dictionary<Shape, AdaptorTool> MST = new(ShapeHasher.Instance);
        Contour FO = new(new Vector3D(1, -1, 1));
        Assert.Throws<LockedShapeException>(() => DSFiller.Insert(aCyl, FO, DS, MST, 0));
    }

    [Fact]
    public void Intersector_LineThroughSphere()
    {
        Shape aSph = new MakeSphere(new Point3D(0, 0, 0), 5.0).Shape();
        HiddenLineData DS = Load(aSph, Axo, 0, out _);
        HlrSurface S = DS.FDataArray()[1].Geometry();
        HiddenLineIntersector I = new();
        I.Load(S);
        I.Perform(new Line3D(new Point3D(-10, 0, 0), new Direction3D(1, 0, 0)), 100.0);
        Assert.True(I.IsDone());
        Assert.Equal(2, I.NbPoints());
        Assert.Equal(5.0, Math.Abs(I.CSPoint(1).Pnt().X), 9);
    }
}
