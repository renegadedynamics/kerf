// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Classification;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BOPTools;

/// <summary>BOPTools_AlgoTools (BOPTools_AlgoTools.cxx and _2.cxx): known answers and context reuse.</summary>
public class BOPTools_AlgoToolsTests
{
    private static Solid BoxSolid(double x, double y, double z, double dx, double dy, double dz) =>
        ShapeCasts.Solid(TopExp_ExplorerFirst(new MakeBox(new Point3D(x, y, z), dx, dy, dz).InternalShape(), ShapeType.Solid));

    private static Shape TopExp_ExplorerFirst(in Shape theS, ShapeType theType)
    {
        Explorer anExp = new(theS, theType);
        Assert.True(anExp.More());
        return anExp.Current();
    }

    private static IEnumerable<(string Name, Shape Shape)> Shapes()
    {
        foreach (var aS in BOPToolsTestShapes.Primitives())
        {
            yield return aS;
        }
        foreach (var aS in BOPToolsTestShapes.DataOcc())
        {
            yield return aS;
        }
    }

    // A face on a copy of the face's surface over the middle half of its UV bounds (FORWARD).
    private static Face SplitOf(in Face theF)
    {
        BrepTools.UVBounds(theF, out double u1, out double u2, out double v1, out double v2);
        double du = 0.25 * (u2 - u1), dv = 0.25 * (v2 - v1);
        Surface aS = (Surface)Tool.Surface(theF)!.Copy();
        Placement aLoc = theF.Location();
        Face aSplit = ShapeCasts.Face(new MakeFace(aS, u1 + du, u2 - du, v1 + dv, v2 - dv, global::Kerf.Engine.Foundation.Precision.Confusion()).InternalShape());
        return ShapeCasts.Face(aSplit.Located(aLoc));
    }

    [Fact]
    public void IsSplitToReverse_faces_on_copied_surfaces_follow_the_original_orientation()
    {
        int aNb = 0;
        foreach ((string aName, Shape aShape) in BOPToolsTestShapes.Primitives())
        {
            IntersectionContext aCtx = new();
            foreach (Face aF in BOPToolsTestShapes.Faces(aShape))
            {
                Face aSp = SplitOf(aF);
                bool bExpected = aF.Orientation() == Reversed;
                Assert.Equal(bExpected, AlgoTools.IsSplitToReverse(aSp, aF, aCtx, out int anErr));
                Assert.Equal(0, anErr);
                Face aSpR = ShapeCasts.Face(aSp.Reversed());
                Assert.Equal(!bExpected, AlgoTools.IsSplitToReverse(aSpR, aF, aCtx, out anErr));
                Assert.Equal(0, anErr);
                // the shape form dispatches
                Assert.Equal(bExpected, AlgoTools.IsSplitToReverse((Shape)aSp, (Shape)aF, aCtx));
                ++aNb;
            }
            _ = aName;
        }
        Assert.Equal(14, aNb);
    }

    [Fact]
    public void IsSplitToReverse_same_surface_compares_orientations()
    {
        Face aF = BOPToolsTestShapes.Faces(BOPToolsTestShapes.Box())[0];
        IntersectionContext aCtx = new();
        Assert.False(AlgoTools.IsSplitToReverse(aF, aF, aCtx));
        Assert.True(AlgoTools.IsSplitToReverse(ShapeCasts.Face(aF.Reversed()), aF, aCtx));
    }

    [Fact]
    public void IsSplitToReverse_edges_on_copied_curves_follow_the_original_orientation()
    {
        int aNb = 0;
        foreach ((string _, Shape aShape) in Shapes())
        {
            IntersectionContext aCtx = new();
            foreach (Edge aE in BOPToolsTestShapes.Edges(aShape))
            {
                if (Tool.Degenerated(aE))
                {
                    Assert.False(AlgoTools.IsSplitToReverse(aE, aE, aCtx, out int anErrD));
                    Assert.Equal(1, anErrD);
                    continue;
                }
                Curve3D? aC = Tool.Curve(aE, out Placement aLoc, out double f, out double l);
                if (aC is null)
                {
                    continue;
                }
                double d = 0.25 * (l - f);
                Edge aSp = ShapeCasts.Edge(new MakeEdge((Curve3D)aC.Copy(), f + d, l - d).InternalShape());
                aSp = ShapeCasts.Edge(aSp.Located(aLoc));
                bool bExpected = aE.Orientation() == Reversed;
                Assert.Equal(bExpected, AlgoTools.IsSplitToReverse(aSp, aE, aCtx, out int anErr));
                Assert.Equal(0, anErr);
                Assert.Equal(!bExpected, AlgoTools.IsSplitToReverse(ShapeCasts.Edge(aSp.Reversed()), aE, aCtx));
                ++aNb;
            }
        }
        Assert.True(aNb > 100);
    }

    [Fact]
    public void IsSplitToReverse_of_other_types_reports_100_and_WithWarn_adds_an_alert()
    {
        Shape aBox = BOPToolsTestShapes.Box();
        Vertex aV = ShapeCasts.Vertex(TopExp_ExplorerFirst(aBox, ShapeType.Vertex));
        Assert.False(AlgoTools.IsSplitToReverse((Shape)aV, (Shape)aV, new IntersectionContext(), out int anErr));
        Assert.Equal(100, anErr);
        global::Kerf.Engine.Foundation.Report aReport = new();
        Assert.False(AlgoTools.IsSplitToReverseWithWarn(aV, aV, new IntersectionContext(), aReport));
        Assert.Single(aReport.GetAlerts(global::Kerf.Engine.Foundation.Gravity.Warning));
        Assert.IsType<AlertUnableToOrientTheShape>(aReport.GetAlerts(global::Kerf.Engine.Foundation.Gravity.Warning).First());
    }

    [Fact]
    public void ComputeState_agrees_with_the_solid_classifier()
    {
        Random aRnd = new(6440);
        foreach ((string aName, Shape aShape) in Shapes())
        {
            Explorer aSolids = new(aShape, ShapeType.Solid);
            if (!aSolids.More())
            {
                continue;
            }
            Solid aSolid = ShapeCasts.Solid(aSolids.Current());
            Box aBox = new();
            Bounds.Add(aSolid, ref aBox);
            aBox.Get(out double x0, out double y0, out double z0, out double x1, out double y1, out double z1);
            IntersectionContext aCtx = new();
            for (int i = 0; i < 60; ++i)
            {
                Point3D aP = new(x0 + (x1 - x0) * (1.2 * aRnd.NextDouble() - 0.1),
                                y0 + (y1 - y0) * (1.2 * aRnd.NextDouble() - 0.1),
                                z0 + (z1 - z0) * (1.2 * aRnd.NextDouble() - 0.1));
                SolidClassifier aSC = new(aSolid, aP, 1.0e-7);
                Assert.True(aSC.State() == AlgoTools.ComputeState(aP, aSolid, 1.0e-7, aCtx), $"{aName} point {i}");
            }
            // vertices, edges and faces of the solid itself are ON
            foreach (Edge aE in BOPToolsTestShapes.Edges(aSolid).Take(10))
            {
                Assert.Equal(On, AlgoTools.ComputeState(aE, aSolid, 1.0e-7, aCtx));
                Assert.Equal(On, AlgoTools.ComputeStateByOnePoint(Exploration.FirstVertex(aE), aSolid, 1.0e-7, aCtx));
            }
        }
    }

    [Fact]
    public void ComputeState_of_faces_and_containers()
    {
        Solid anOuter = BoxSolid(0, 0, 0, 10, 10, 10);
        Solid anInner = BoxSolid(2, 2, 2, 3, 3, 3);
        Solid aFar = BoxSolid(20, 0, 0, 3, 3, 3);
        IntersectionContext aCtx = new();
        IndexedSet<Shape> aBounds = new(ShapeHasher.Instance);
        Exploration.MapShapes(anOuter, ShapeType.Edge, aBounds);
        foreach (Face aF in BOPToolsTestShapes.Faces(anInner))
        {
            Assert.Equal(In, AlgoTools.ComputeState(aF, anOuter, 1.0e-7, aBounds, aCtx));
            Assert.Equal(In, AlgoTools.ComputeStateByOnePoint(aF, anOuter, 1.0e-7, aCtx));
        }
        foreach (Face aF in BOPToolsTestShapes.Faces(aFar))
        {
            Assert.Equal(Out, AlgoTools.ComputeStateByOnePoint(aF, anOuter, 1.0e-7, aCtx));
        }
        // the solid's own faces: all edges are bounds, so a point inside the face is classified
        IndexedSet<Shape> aOwn = new(ShapeHasher.Instance);
        Exploration.MapShapes(anOuter, ShapeType.Edge, aOwn);
        foreach (Face aF in BOPToolsTestShapes.Faces(anOuter))
        {
            Assert.Equal(On, AlgoTools.ComputeState(aF, anOuter, 1.0e-7, aOwn, aCtx));
        }
        // OCCT: a container's state is UNKNOWN (the sub-shape's state is discarded)
        Assert.Equal(Unknown, AlgoTools.ComputeStateByOnePoint(anInner, anOuter, 1.0e-7, aCtx));
    }

    // The unit box, its edge on the z axis, the two box faces on it, and a planar face through that edge going
    // along (dx, dy, 0), sharing the edge and its vertices.
    private static (Solid Box, Edge E, Face F1, Face F2, Face Fin) FanAtZAxis(double dx, double dy)
    {
        Solid aBox = BoxSolid(0, 0, 0, 1, 1, 1);
        Edge aZ = new();
        foreach (Edge aE in BOPToolsTestShapes.Edges(aBox))
        {
            Exploration.Vertices(aE, out Vertex a, out Vertex b);
            Point3D pa = Tool.Pnt(a), pb = Tool.Pnt(b);
            if (pa.X() == 0 && pa.Y() == 0 && pb.X() == 0 && pb.Y() == 0)
            {
                aZ = aE;
                break;
            }
        }
        Assert.False(aZ.IsNull);
        IndexedMap<Shape, NCollection_List<Shape>> aMEF = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aBox, ShapeType.Edge, ShapeType.Face, aMEF);
        NCollection_List<Shape> aLF = aMEF.GetValue(aZ);
        Assert.Equal(2, aLF.Extent());
        Exploration.Vertices(aZ, out Vertex aV0, out Vertex aV1);
        Point3D p0 = Tool.Pnt(aV0), p1 = Tool.Pnt(aV1);
        Vertex aW0 = ShapeCasts.Vertex(new MakeVertex(new Point3D(p0.X() + dx, p0.Y() + dy, p0.Z())).InternalShape());
        Vertex aW1 = ShapeCasts.Vertex(new MakeVertex(new Point3D(p1.X() + dx, p1.Y() + dy, p1.Z())).InternalShape());
        Edge e1 = ShapeCasts.Edge(new MakeEdge(aV1, aW1).InternalShape());
        Edge e2 = ShapeCasts.Edge(new MakeEdge(aW1, aW0).InternalShape());
        Edge e3 = ShapeCasts.Edge(new MakeEdge(aW0, aV0).InternalShape());
        Edge aZf = ShapeCasts.Edge(aZ.Oriented(Forward));
        Wire aW = ShapeCasts.Wire(new MakeWire(aZf, e1, e2, e3).InternalShape());
        Face aFin = ShapeCasts.Face(new MakeFace(aW, true).InternalShape());
        return (aBox, aZ, ShapeCasts.Face(aLF.First()), ShapeCasts.Face(aLF.Last()), aFin);
    }

    [Fact]
    public void IsInternalFace_by_angles_and_by_classification()
    {
        IntersectionContext aCtx = new();
        (Solid aBox, Edge aE, Face aF1, Face aF2, Face aIn) = FanAtZAxis(0.5, 0.5);
        Assert.Equal(1, AlgoTools.IsInternalFace(aIn, aE, aF1, aF2, aCtx));
        NCollection_List<Shape> aLF = new();
        aLF.Append(aF1);
        aLF.Append(aF2);
        Assert.Equal(1, AlgoTools.IsInternalFace(aIn, aE, aLF, aCtx));
        IndexedMap<Shape, NCollection_List<Shape>> aMEF = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aBox, ShapeType.Edge, ShapeType.Face, aMEF);
        Assert.True(AlgoTools.IsInternalFace(aIn, aBox, aMEF, 1.0e-7, aCtx));

        (_, Edge aE2, Face aG1, Face aG2, Face aOut) = FanAtZAxis(-0.5, -0.5);
        Assert.Equal(0, AlgoTools.IsInternalFace(aOut, aE2, aG1, aG2, aCtx));

        // a face sharing no edge with the solid: classified by a point
        Solid anInner = BoxSolid(0.25, 0.25, 0.25, 0.5, 0.5, 0.5);
        Face anInnerFace = BOPToolsTestShapes.Faces(anInner)[0];
        Assert.True(AlgoTools.IsInternalFace(anInnerFace, aBox, aMEF, 1.0e-7, aCtx));
        Face aFarFace = BOPToolsTestShapes.Faces(BoxSolid(5, 5, 5, 1, 1, 1))[0];
        Assert.False(AlgoTools.IsInternalFace(aFarFace, aBox, aMEF, 1.0e-7, aCtx));
    }

    [Fact]
    public void GetFaceOff_finds_the_face_making_the_smallest_angle()
    {
        IntersectionContext aCtx = new();
        (_, Edge aE, Face aF1, Face aF2, Face aIn) = FanAtZAxis(0.5, 0.5);
        Edge aE1 = new(), aE2 = new(), aEIn = new();
        Assert.True(AlgoTools.GetEdgeOnFace(aE, aF1, ref aE1));
        Assert.True(AlgoTools.GetEdgeOnFace(aE, aF2, ref aE2));
        Assert.True(AlgoTools.GetEdgeOnFace(aE, aIn, ref aEIn));
        Assert.True(aE1.IsSame(aE) && aE2.IsSame(aE));
        Assert.NotEqual(aE1.Orientation(), aE2.Orientation());
        Edge aOff = new();
        Assert.True(AlgoTools.GetEdgeOff(aE1, aF2, ref aOff));
        Assert.Equal(aE2, aOff);
        Assert.False(AlgoTools.GetEdgeOff(aE1, aF1, ref aOff));

        NCollection_List<CoupleOfShape> aLCS = new();
        aLCS.Append(new CoupleOfShape().WithShape1(aEIn).WithShape2(aIn));
        aLCS.Append(new CoupleOfShape().WithShape1(aE2).WithShape2(aF2));
        Face aFOff = new();
        Assert.True(AlgoTools.GetFaceOff(aE1, aF1, aLCS, ref aFOff, aCtx));
        Assert.True(aFOff.IsSame(aIn) || aFOff.IsSame(aF2));
    }

    [Fact]
    public void AreFacesSameDomain_and_Sense()
    {
        IntersectionContext aCtx = new();
        List<Face> aFaces = BOPToolsTestShapes.Faces(BOPToolsTestShapes.Box());
        Face aF = aFaces[0];
        Face aSp = SplitOf(aF);
        Assert.True(AlgoTools.AreFacesSameDomain(aSp, aF, aCtx));
        Assert.False(AlgoTools.AreFacesSameDomain(aF, aFaces[1], aCtx));
        Assert.Equal(1, AlgoTools.Sense(aF, aF, aCtx));
        Assert.Equal(-1, AlgoTools.Sense(aF, ShapeCasts.Face(aF.Reversed()), aCtx));
        // no common edge
        Assert.Equal(0, AlgoTools.Sense(aF, aSp, aCtx));
    }

    [Fact]
    public void IsHole_by_the_wire_orientation()
    {
        Face aF = ShapeCasts.Face(new MakeFace(new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)), 0, 2, 0, 1).InternalShape());
        Wire aW = ShapeCasts.Wire(TopExp_ExplorerFirst(aF, ShapeType.Wire));
        Assert.False(AlgoTools.IsHole(aW, aF));
        Assert.True(AlgoTools.IsHole(aW.Reversed(), aF));
    }

    [Fact]
    public void MakeContainer_makes_each_type()
    {
        foreach (ShapeType aType in new[] { ShapeType.Compound, ShapeType.CompSolid, ShapeType.Solid, ShapeType.Shell, ShapeType.Wire })
        {
            Shape aC = new();
            AlgoTools.MakeContainer(aType, ref aC);
            Assert.Equal(aType, aC.ShapeType());
            Assert.False(new Iterator(aC).More());
        }
        Shape aKeep = BOPToolsTestShapes.Box();
        AlgoTools.MakeContainer(ShapeType.Face, ref aKeep);
        Assert.Equal(ShapeType.Solid, aKeep.ShapeType());
    }

    [Fact]
    public void MakePCurve_adds_pcurves_on_both_faces()
    {
        Solid aBox = BoxSolid(0, 0, 0, 2, 2, 2);
        // the two faces at the edge x = 2, z = 2: the line from (2, 0.5, 2) to (2, 1.5, 2) lies on both
        Face aFx = new(), aFz = new();
        foreach (Face aF in BOPToolsTestShapes.Faces(aBox))
        {
            FaceIntegrand aProps = new(aF);
            FaceSurfaceAdaptor aBAS = new(aF);
            Plane3D aPln = aBAS.Plane();
            Point3D aLoc = aPln.Location();
            Direction3D aN = aPln.Axis().Direction();
            if (Math.Abs(aN.X()) > 0.5 && aLoc.X() == 2)
            {
                aFx = aF;
            }
            if (Math.Abs(aN.Z()) > 0.5 && aLoc.Z() == 2)
            {
                aFz = aF;
            }
            _ = aProps;
        }
        Assert.False(aFx.IsNull || aFz.IsNull);
        LineCurve3D aLine = new(new Point3D(2, 0.5, 2), new Direction3D(0, 1, 0));
        Edge aE = ShapeCasts.Edge(new MakeEdge(aLine, 0, 1).InternalShape());
        IntersectionCurve aIC = new(aLine, null, null);
        AlgoTools.MakePCurve(aE, aFx, aFz, aIC, true, true, new IntersectionContext());
        foreach (Face aF in new[] { aFx, aFz })
        {
            Curve2D? aC2 = Tool.CurveOnSurface(aE, aF, out double f, out double l);
            Assert.NotNull(aC2);
            Surface aS = Tool.Surface(aF, out Placement aL)!;
            for (int i = 0; i <= 10; ++i)
            {
                double t = f + (l - f) * i / 10.0;
                Point2D uv = aC2!.Value(t);
                Point3D p = aS.Value(uv.X(), uv.Y()).Transformed(aL.Transformation());
                Assert.True(p.Distance(aLine.Value(t)) < 1.0e-9);
            }
        }
        Assert.True(Tool.SameParameter(aE));
    }

    [Fact]
    public void MakeEdge_MakeSectEdge_MakeVertex_and_ComputeVV()
    {
        LineCurve3D aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        IntersectionCurve aIC = new(aLine, null, null);
        AlgoTools.MakeNewVertex(new Point3D(0, 0, 0), 1.0e-7, out Vertex aV1);
        AlgoTools.MakeNewVertex(new Point3D(3, 0, 0), 1.0e-7, out Vertex aV2);
        AlgoTools.MakeEdge(aIC, aV1, 0, aV2, 3, 1.0e-4, out Edge aE);
        Tool.Range(aE, out double f, out double l);
        Assert.Equal(0.0, f);
        Assert.Equal(3.0, l);
        Assert.Equal(1.0e-4, Tool.Tolerance(aE));
        Assert.Equal(1.0e-4 + 1.0e-12, Tool.Tolerance(aV1));

        AlgoTools.MakeNewVertex(new Point3D(0.5, 0, 0), 1.0e-7, out Vertex aVh);
        AlgoTools.MakeSectEdge(aIC, aVh, 0.5, aV2, 3, out Edge aE2);
        Tool.Range(aE2, out f, out l);
        Assert.Equal(0.5, f);

        Assert.Equal(0, AlgoTools.ComputeVV(aV1, new Point3D(1.0e-4, 0, 0), 0.0));
        Assert.Equal(1, AlgoTools.ComputeVV(aV1, new Point3D(1.0e-3, 0, 0), 0.0));
        AlgoTools.MakeNewVertex(new Point3D(1.0e-4, 0, 0), 1.0e-7, out Vertex aV3);
        Assert.Equal(0, AlgoTools.ComputeVV(aV1, aV3));
        Assert.Equal(1, AlgoTools.ComputeVV(aV1, aV2));
        Assert.Equal(0, AlgoTools.ComputeVV(aV1, aV2, 3.0));

        NCollection_List<Shape> aLV = new();
        Vertex aVnew = new();
        AlgoTools.MakeVertex(aLV, ref aVnew);
        Assert.True(aVnew.IsNull);
        aLV.Append(aV1);
        AlgoTools.MakeVertex(aLV, ref aVnew);
        Assert.Equal(aV1, aVnew);
        aLV.Append(aV3);
        AlgoTools.MakeVertex(aLV, ref aVnew);
        Assert.False(aVnew.IsSame(aV1) || aVnew.IsSame(aV3));
        Assert.True(Tool.Pnt(aVnew).Distance(Tool.Pnt(aV1)) <= Tool.Tolerance(aVnew));
        Assert.True(Tool.Pnt(aVnew).Distance(Tool.Pnt(aV3)) <= Tool.Tolerance(aVnew));
    }

    [Fact]
    public void MakeConnexityBlocks_three_forms()
    {
        Solid aB1 = BoxSolid(0, 0, 0, 1, 1, 1), aB2 = BoxSolid(5, 0, 0, 1, 1, 1);
        Compound aC = new();
        BrepBuilder aBB = new();
        aBB.MakeCompound(ref aC);
        aBB.Add(aC, aB1);
        aBB.Add(aC, aB2);
        NCollection_List<Shape> aLCB = new();
        AlgoTools.MakeConnexityBlocks(aC, ShapeType.Edge, ShapeType.Face, aLCB);
        Assert.Equal(2, aLCB.Extent());
        foreach (Shape aBlock in aLCB)
        {
            Assert.Equal(6, BOPToolsTestShapes.Faces(aBlock).Count);
        }

        // list form: regular for a box's faces, irregular when a face is given twice
        NCollection_List<Shape> aLS = new(BOPToolsTestShapes.Faces(aB1).Select(f => (Shape)f));
        NCollection_List<ConnexityBlock> aLCB2 = new();
        AlgoTools.MakeConnexityBlocks(aLS, ShapeType.Edge, ShapeType.Face, aLCB2);
        Assert.Equal(1, aLCB2.Extent());
        Assert.True(aLCB2.First().IsRegular);
        Assert.Equal(6, aLCB2.First().Shapes().Extent());
        aLS.Append(aLS.First());
        NCollection_List<ConnexityBlock> aLCB3 = new();
        AlgoTools.MakeConnexityBlocks(aLS, ShapeType.Edge, ShapeType.Face, aLCB3);
        Assert.False(aLCB3.First().IsRegular);
        Assert.Equal(7, aLCB3.First().Shapes().Extent());

        // MakeConnexityBlock: avoiding the four vertical edges of the box splits off nothing (top and bottom still
        // connect through the side faces), avoiding every edge leaves the first face alone
        IndexedSet<Shape> aAvoid = new(ShapeHasher.Instance);
        NCollection_List<Shape> aOne = new();
        AlgoTools.MakeConnexityBlock(new NCollection_List<Shape>(BOPToolsTestShapes.Faces(aB1).Select(f => (Shape)f)), aAvoid, aOne);
        Assert.Equal(6, aOne.Extent());
        Exploration.MapShapes(aB1, ShapeType.Edge, aAvoid);
        aOne.Clear();
        AlgoTools.MakeConnexityBlock(new NCollection_List<Shape>(BOPToolsTestShapes.Faces(aB1).Select(f => (Shape)f)), aAvoid, aOne);
        Assert.Equal(1, aOne.Extent());
    }

    [Fact]
    public void OrientEdgesOnWire_chains_reversed_edges()
    {
        Face aF = BOPToolsTestShapes.Faces(BoxSolid(0, 0, 0, 1, 2, 3))[0];
        List<Edge> aEdges = BOPToolsTestShapes.Edges(aF);
        Wire aW = new();
        BrepBuilder aBB = new();
        aBB.MakeWire(ref aW);
        for (int i = 0; i < aEdges.Count; ++i)
        {
            aBB.Add(aW, i % 2 == 0 ? aEdges[i] : aEdges[i].Reversed());
        }
        Shape aWS = aW;
        AlgoTools.OrientEdgesOnWire(ref aWS);
        List<Edge> aOut = [];
        for (Iterator it = new(aWS); it.More(); it.Next())
        {
            aOut.Add(ShapeCasts.Edge(it.Value()));
        }
        Assert.Equal(aEdges.Count, aOut.Count);
        // every vertex ends one edge and starts the other (the edges are oriented head to tail; OCCT adds them in
        // traversal order, not in chain order)
        IndexedMap<Shape, NCollection_List<Shape>> aVE = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aWS, ShapeType.Vertex, ShapeType.Edge, aVE);
        for (int i = 0; i < aVE.Count; ++i)
        {
            Shape aV = aVE.KeyAt(i);
            int aNbFirst = 0, aNbLast = 0;
            foreach (Edge e in aOut)
            {
                Exploration.Vertices(e, out Vertex a, out Vertex b, true);
                aNbFirst += a.IsSame(aV) ? 1 : 0;
                aNbLast += b.IsSame(aV) ? 1 : 0;
            }
            Assert.Equal((1, 1), (aNbFirst, aNbLast));
        }
    }

    [Fact]
    public void OrientFacesOnShell_makes_shared_edges_opposite()
    {
        Shell aShell = ShapeCasts.Shell(TopExp_ExplorerFirst(BoxSolid(0, 0, 0, 1, 2, 3), ShapeType.Shell));
        List<Face> aFaces = BOPToolsTestShapes.Faces(aShell);
        Shell aMixed = new();
        BrepBuilder aBB = new();
        aBB.MakeShell(ref aMixed);
        for (int i = 0; i < aFaces.Count; ++i)
        {
            aBB.Add(aMixed, i is 1 or 4 ? aFaces[i].Reversed() : aFaces[i]);
        }
        Shape aS = aMixed;
        AlgoTools.OrientFacesOnShell(ref aS);
        Assert.Equal(6, BOPToolsTestShapes.Faces(aS).Count);
        IndexedMap<Shape, NCollection_List<Shape>> aMEF = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aS, ShapeType.Edge, ShapeType.Face, aMEF);
        for (int i = 0; i < aMEF.Count; ++i)
        {
            Edge aE = ShapeCasts.Edge(aMEF.KeyAt(i));
            NCollection_List<Shape> aLF = aMEF.ValueAt(i);
            Assert.Equal(2, aLF.Extent());
            Orientation o1 = Internal, o2 = Internal;
            for (Explorer x = new(aLF.First(), ShapeType.Edge); x.More(); x.Next())
            {
                if (x.Current().IsSame(aE)) { o1 = x.Current().Orientation(); }
            }
            for (Explorer x = new(aLF.Last(), ShapeType.Edge); x.More(); x.Next())
            {
                if (x.Current().IsSame(aE)) { o2 = x.Current().Orientation(); }
            }
            Assert.NotEqual(o1, o2);
        }
    }

    [Fact]
    public void IsMicroEdge_and_IsBlockInOnFace()
    {
        IntersectionContext aCtx = new();
        Solid aBox = BoxSolid(0, 0, 0, 1, 2, 3);
        Edge aE = BOPToolsTestShapes.Edges(aBox)[0];
        Assert.False(AlgoTools.IsMicroEdge(aE, aCtx));
        Edge aTiny = ShapeCasts.Edge(new MakeEdge(new Point3D(0, 0, 0), new Point3D(1.0e-8, 0, 0)).InternalShape());
        Assert.True(AlgoTools.IsMicroEdge(aTiny, aCtx));
        Edge aDegen = new();
        foreach (Edge x in BOPToolsTestShapes.Edges(BOPToolsTestShapes.Sphere()))
        {
            if (Tool.Degenerated(x)) { aDegen = x; break; }
        }
        Assert.True(AlgoTools.IsMicroEdge(aDegen, aCtx));

        IndexedMap<Shape, NCollection_List<Shape>> aMEF = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aBox, ShapeType.Edge, ShapeType.Face, aMEF);
        Tool.Range(aE, out double f, out double l);
        ParameterRange aR = new(f, l);
        foreach (Shape aF in aMEF.GetValue(aE))
        {
            Assert.True(AlgoTools.IsBlockInOnFace(aR, ShapeCasts.Face(aF), aE, aCtx));
        }
        int aNbOff = 0;
        foreach (Face aF in BOPToolsTestShapes.Faces(aBox))
        {
            if (!aMEF.GetValue(aE).Any(x => x.IsSame(aF)))
            {
                aNbOff += AlgoTools.IsBlockInOnFace(aR, aF, aE, aCtx) ? 0 : 1;
            }
        }
        Assert.Equal(4, aNbOff);
    }

    [Fact]
    public void AlgoTools_2_vertices_edges_and_ranges()
    {
        AlgoTools.MakeNewVertex(new Point3D(0, 0, 0), 1.0e-3, out Vertex aV1);
        AlgoTools.MakeNewVertex(new Point3D(0.01, 0, 0), 1.0e-4, out Vertex aV2);
        AlgoTools.MakeNewVertex(aV1, aV2, out Vertex aVm);
        Assert.Equal(0.005, Tool.Pnt(aVm).X(), 15);
        Assert.Equal(1.0e-3 + 0.005, Tool.Tolerance(aVm), 15);

        Edge aE = ShapeCasts.Edge(new MakeEdge(new Point3D(0, 0, 0), new Point3D(4, 0, 0)).InternalShape());
        Edge aE2 = ShapeCasts.Edge(new MakeEdge(new Point3D(1, 1.0e-3, 0), new Point3D(1, 5, 0)).InternalShape());
        AlgoTools.MakeNewVertex(aE, 1.0, aE2, 0.0, out Vertex aVee);
        Assert.Equal(1.0, Tool.Pnt(aVee).X(), 15);
        Face aF = ShapeCasts.Face(new MakeFace(new Plane3D(), 0, 1, 0, 1).InternalShape());
        AlgoTools.MakeNewVertex(aE, 2.0, aF, out Vertex aVef);
        Assert.Equal(Tool.Tolerance(aE) + Tool.Tolerance(aF) + 1.0e-12, Tool.Tolerance(aVef));

        // UpdateVertex forms
        AlgoTools.MakeNewVertex(new Point3D(2, 0.01, 0), 1.0e-7, out Vertex aVu);
        AlgoTools.UpdateVertex(aE, 2.0, aVu);
        Assert.Equal(0.01 + 1.0e-12, Tool.Tolerance(aVu), 15);
        AlgoTools.MakeNewVertex(new Point3D(2, 0.02, 0), 1.0e-7, out Vertex aVc);
        AlgoTools.UpdateVertex(new IntersectionCurve(Tool.Curve(aE, out _, out _), null, null), 2.0, aVc);
        Assert.Equal(0.02 + 1.0e-12, Tool.Tolerance(aVc), 15);
        AlgoTools.MakeNewVertex(new Point3D(2, 0.05, 0), 1.0e-3, out Vertex aVn);
        AlgoTools.UpdateVertex(aVu, aVn);
        Assert.True(Tool.Tolerance(aVu) > 0.04);

        // split and copy
        AlgoTools.MakeNewVertex(new Point3D(1, 0, 0), 1.0e-7, out Vertex aS1);
        AlgoTools.MakeNewVertex(new Point3D(3, 0, 0), 1.0e-7, out Vertex aS2);
        AlgoTools.MakeSplitEdge(aE, aS2, 3.0, aS1, 1.0, out Edge aSp);
        Tool.Range(aSp, out double f, out double l);
        Assert.Equal((1.0, 3.0), (f, l));
        Assert.Same(Tool.Curve(aE, out _, out _), Tool.Curve(aSp, out _, out _));
        Exploration.Vertices(aSp, out Vertex aVF, out Vertex aVL);
        Assert.True(aVF.IsSame(aS1) && aVL.IsSame(aS2));
        Edge aCopy = AlgoTools.CopyEdge(ShapeCasts.Edge(aE.Reversed()));
        Assert.False(aCopy.IsSame(aE));
        Assert.Equal(Reversed, aCopy.Orientation());
        Exploration.Vertices(aCopy, out Vertex aCF, out Vertex aCL);
        Exploration.Vertices(aE, out Vertex aEF, out Vertex aEL);
        Assert.True(aCF.IsSame(aEF) && aCL.IsSame(aEL));

        // CorrectRange: lines unchanged, circles shrunk by the tolerances' resolution
        ParameterRange aR = new(0.5, 3.5);
        AlgoTools.CorrectRange(aE, aE2, aR, out ParameterRange aNR);
        Assert.Equal(aR, aNR);
        Edge aCircle = ShapeCasts.Edge(new MakeEdge(new Circle3D(new Frame3D(), 10)).InternalShape());
        AlgoTools.CorrectRange(aCircle, aE, new ParameterRange(1, 2), out aNR);
        double aRes = 2.0 * (Tool.Tolerance(aCircle) + Tool.Tolerance(aE)) / 10.0;
        Assert.Equal(1 + aRes, aNR.First(), 14);
        Assert.Equal(2 - aRes, aNR.Last(), 14);
        AlgoTools.CorrectRange(aCircle, aF, new ParameterRange(1, 2), out aNR);
        Assert.Equal(1 + Tool.Tolerance(aF) / 10.0, aNR.First(), 14);
        AlgoTools.CorrectRange(aCircle, aF, new ParameterRange(1, 1 + 1.0e-10), out aNR);
        Assert.Equal(new ParameterRange(1, 1 + 1.0e-10), aNR);
    }

    [Fact]
    public void Dimensions_and_TreatCompound()
    {
        Solid aB = BoxSolid(0, 0, 0, 1, 1, 1);
        Face aF = BOPToolsTestShapes.Faces(aB)[0];
        Edge aE = BOPToolsTestShapes.Edges(aB)[0];
        Assert.Equal(3, AlgoTools.Dimension(aB));
        Assert.Equal(2, AlgoTools.Dimension(aF));
        Assert.Equal(1, AlgoTools.Dimension(aE));
        Assert.Equal(0, AlgoTools.Dimension(Exploration.FirstVertex(aE)));
        Compound aC = new(), aInner = new(), aEmpty = new();
        BrepBuilder aBB = new();
        aBB.MakeCompound(ref aC);
        aBB.MakeCompound(ref aInner);
        aBB.MakeCompound(ref aEmpty);
        AlgoTools.Dimensions(aEmpty, out int aMin, out int aMax);
        Assert.Equal((-1, -1), (aMin, aMax));
        aBB.Add(aInner, aF);
        aBB.Add(aInner, aE);
        aBB.Add(aC, aInner);
        aBB.Add(aC, aF);
        aBB.Add(aC, aEmpty);
        AlgoTools.Dimensions(aC, out aMin, out aMax);
        Assert.Equal((1, 2), (aMin, aMax));
        Assert.Equal(-1, AlgoTools.Dimension(aC));
        NCollection_List<Shape> aL = new();
        AlgoTools.TreatCompound(aC, aL);
        Assert.Equal(3, aL.Extent());
        aL.Clear();
        AlgoTools.TreatCompound(aC, aL, new HashSet<Shape>(ShapeHasher.Instance));
        Assert.Equal(2, aL.Extent());
    }

    [Fact]
    public void MakePCurve_and_MakeEdge_throw_on_frozen_input()
    {
        Shape aBox = BOPToolsTestShapes.Box(); // published: frozen
        Vertex aV = ShapeCasts.Vertex(TopExp_ExplorerFirst(aBox, ShapeType.Vertex));
        LineCurve3D aLine = new(Tool.Pnt(aV), new Direction3D(1, 0, 0));
        AlgoTools.MakeNewVertex(new Point3D(100, 0, 0), 1.0e-7, out Vertex aV2);
        Assert.ThrowsAny<Exception>(() =>
        {
            AlgoTools.MakeEdge(new IntersectionCurve(aLine, null, null), aV, 0, aV2, 1, 1.0e-3, out _);
        });
    }

    // A reused context answers as a fresh context per call, over shuffled inputs.
    [Fact]
    public void Reused_context_answers_as_fresh_ones()
    {
        Random aRnd = new(6441);
        foreach ((string aName, Shape aShape) in Shapes())
        {
            List<Face> aFaces = BOPToolsTestShapes.Faces(aShape);
            List<Edge> aEdges = BOPToolsTestShapes.Edges(aShape).Where(e => !Tool.Degenerated(e) && Tool.Curve(e, out _, out _) is not null).ToList();
            Explorer aSolids = new(aShape, ShapeType.Solid);
            Solid? aSolid = aSolids.More() ? ShapeCasts.Solid(aSolids.Current()) : null;
            IndexedMap<Shape, NCollection_List<Shape>> aMEF = new(ShapeHasher.Instance);
            if (aSolid is not null)
            {
                Exploration.MapShapesAndAncestors(aSolid.Value, ShapeType.Edge, ShapeType.Face, aMEF);
            }
            List<Func<IntersectionContext, string>> aQueries = [];
            foreach (Face aF in aFaces.Take(40))
            {
                Face aSp = SplitOf(aF);
                aQueries.Add(c => $"{AlgoTools.IsSplitToReverse(aSp, aF, c, out int e)} {e}");
                if (aSolid is not null)
                {
                    Solid s = aSolid.Value;
                    aQueries.Add(c => AlgoTools.IsInternalFace(aSp, s, aMEF, 1.0e-7, c).ToString());
                    aQueries.Add(c => AlgoTools.ComputeStateByOnePoint(aSp, s, 1.0e-7, c).ToString());
                }
            }
            foreach (Edge aE in aEdges.Take(40))
            {
                Curve3D aC = Tool.Curve(aE, out Placement aLoc, out double f, out double l)!;
                double d = 0.3 * (l - f);
                Edge aSp = ShapeCasts.Edge(new MakeEdge((Curve3D)aC.Copy(), f + d, l - d).InternalShape().Located(aLoc));
                aQueries.Add(c => $"{AlgoTools.IsSplitToReverse(aSp, aE, c, out int e)} {e}");
                if (aSolid is not null)
                {
                    Solid s = aSolid.Value;
                    aQueries.Add(c => AlgoTools.ComputeState(aSp, s, 1.0e-7, c).ToString());
                }
            }
            string[] aFresh = aQueries.Select(q => q(new IntersectionContext())).ToArray();
            int[] anOrder = Enumerable.Range(0, aQueries.Count).OrderBy(_ => aRnd.Next()).ToArray();
            IntersectionContext aReused = new();
            foreach (int i in anOrder.Concat(anOrder.Reverse()))
            {
                Assert.True(aFresh[i] == aQueries[i](aReused), $"{aName} query {i}");
            }
        }
    }
}
