// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace OcctNet.ModelingAlgorithms.Tests.BOPTools;

/// <summary>BOPTools_AlgoTools (BOPTools_AlgoTools.cxx and _2.cxx): known answers and context reuse.</summary>
public class BOPTools_AlgoToolsTests
{
    private static TopoDS_Solid BoxSolid(double x, double y, double z, double dx, double dy, double dz) =>
        TopoDS.Solid(TopExp_ExplorerFirst(new BRepPrimAPI_MakeBox(new gp_Pnt(x, y, z), dx, dy, dz).InternalShape(), TopAbs_SOLID));

    private static TopoDS_Shape TopExp_ExplorerFirst(in TopoDS_Shape theS, TopAbs_ShapeEnum theType)
    {
        TopExp_Explorer anExp = new(theS, theType);
        Assert.True(anExp.More());
        return anExp.Current();
    }

    private static IEnumerable<(string Name, TopoDS_Shape Shape)> Shapes()
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
    private static TopoDS_Face SplitOf(in TopoDS_Face theF)
    {
        BRepTools.UVBounds(theF, out double u1, out double u2, out double v1, out double v2);
        double du = 0.25 * (u2 - u1), dv = 0.25 * (v2 - v1);
        Geom_Surface aS = (Geom_Surface)BRep_Tool.Surface(theF)!.Copy();
        TopLoc_Location aLoc = theF.Location();
        TopoDS_Face aSplit = TopoDS.Face(new BRepBuilderAPI_MakeFace(aS, u1 + du, u2 - du, v1 + dv, v2 - dv, Precision.Confusion()).InternalShape());
        return TopoDS.Face(aSplit.Located(aLoc));
    }

    [Fact]
    public void IsSplitToReverse_faces_on_copied_surfaces_follow_the_original_orientation()
    {
        int aNb = 0;
        foreach ((string aName, TopoDS_Shape aShape) in BOPToolsTestShapes.Primitives())
        {
            IntTools_Context aCtx = new();
            foreach (TopoDS_Face aF in BOPToolsTestShapes.Faces(aShape))
            {
                TopoDS_Face aSp = SplitOf(aF);
                bool bExpected = aF.Orientation() == TopAbs_REVERSED;
                Assert.Equal(bExpected, BOPTools_AlgoTools.IsSplitToReverse(aSp, aF, aCtx, out int anErr));
                Assert.Equal(0, anErr);
                TopoDS_Face aSpR = TopoDS.Face(aSp.Reversed());
                Assert.Equal(!bExpected, BOPTools_AlgoTools.IsSplitToReverse(aSpR, aF, aCtx, out anErr));
                Assert.Equal(0, anErr);
                // the shape form dispatches
                Assert.Equal(bExpected, BOPTools_AlgoTools.IsSplitToReverse((TopoDS_Shape)aSp, (TopoDS_Shape)aF, aCtx));
                ++aNb;
            }
            _ = aName;
        }
        Assert.Equal(14, aNb);
    }

    [Fact]
    public void IsSplitToReverse_same_surface_compares_orientations()
    {
        TopoDS_Face aF = BOPToolsTestShapes.Faces(BOPToolsTestShapes.Box())[0];
        IntTools_Context aCtx = new();
        Assert.False(BOPTools_AlgoTools.IsSplitToReverse(aF, aF, aCtx));
        Assert.True(BOPTools_AlgoTools.IsSplitToReverse(TopoDS.Face(aF.Reversed()), aF, aCtx));
    }

    [Fact]
    public void IsSplitToReverse_edges_on_copied_curves_follow_the_original_orientation()
    {
        int aNb = 0;
        foreach ((string _, TopoDS_Shape aShape) in Shapes())
        {
            IntTools_Context aCtx = new();
            foreach (TopoDS_Edge aE in BOPToolsTestShapes.Edges(aShape))
            {
                if (BRep_Tool.Degenerated(aE))
                {
                    Assert.False(BOPTools_AlgoTools.IsSplitToReverse(aE, aE, aCtx, out int anErrD));
                    Assert.Equal(1, anErrD);
                    continue;
                }
                Geom_Curve? aC = BRep_Tool.Curve(aE, out TopLoc_Location aLoc, out double f, out double l);
                if (aC is null)
                {
                    continue;
                }
                double d = 0.25 * (l - f);
                TopoDS_Edge aSp = TopoDS.Edge(new BRepBuilderAPI_MakeEdge((Geom_Curve)aC.Copy(), f + d, l - d).InternalShape());
                aSp = TopoDS.Edge(aSp.Located(aLoc));
                bool bExpected = aE.Orientation() == TopAbs_REVERSED;
                Assert.Equal(bExpected, BOPTools_AlgoTools.IsSplitToReverse(aSp, aE, aCtx, out int anErr));
                Assert.Equal(0, anErr);
                Assert.Equal(!bExpected, BOPTools_AlgoTools.IsSplitToReverse(TopoDS.Edge(aSp.Reversed()), aE, aCtx));
                ++aNb;
            }
        }
        Assert.True(aNb > 100);
    }

    [Fact]
    public void IsSplitToReverse_of_other_types_reports_100_and_WithWarn_adds_an_alert()
    {
        TopoDS_Shape aBox = BOPToolsTestShapes.Box();
        TopoDS_Vertex aV = TopoDS.Vertex(TopExp_ExplorerFirst(aBox, TopAbs_VERTEX));
        Assert.False(BOPTools_AlgoTools.IsSplitToReverse((TopoDS_Shape)aV, (TopoDS_Shape)aV, new IntTools_Context(), out int anErr));
        Assert.Equal(100, anErr);
        Message_Report aReport = new();
        Assert.False(BOPTools_AlgoTools.IsSplitToReverseWithWarn(aV, aV, new IntTools_Context(), aReport));
        Assert.Single(aReport.GetAlerts(Message_Gravity.Message_Warning));
        Assert.IsType<BOPAlgo_AlertUnableToOrientTheShape>(aReport.GetAlerts(Message_Gravity.Message_Warning).First());
    }

    [Fact]
    public void ComputeState_agrees_with_the_solid_classifier()
    {
        Random aRnd = new(6440);
        foreach ((string aName, TopoDS_Shape aShape) in Shapes())
        {
            TopExp_Explorer aSolids = new(aShape, TopAbs_SOLID);
            if (!aSolids.More())
            {
                continue;
            }
            TopoDS_Solid aSolid = TopoDS.Solid(aSolids.Current());
            Bnd_Box aBox = new();
            BRepBndLib.Add(aSolid, ref aBox);
            aBox.Get(out double x0, out double y0, out double z0, out double x1, out double y1, out double z1);
            IntTools_Context aCtx = new();
            for (int i = 0; i < 60; ++i)
            {
                gp_Pnt aP = new(x0 + (x1 - x0) * (1.2 * aRnd.NextDouble() - 0.1),
                                y0 + (y1 - y0) * (1.2 * aRnd.NextDouble() - 0.1),
                                z0 + (z1 - z0) * (1.2 * aRnd.NextDouble() - 0.1));
                BRepClass3d_SolidClassifier aSC = new(aSolid, aP, 1.0e-7);
                Assert.True(aSC.State() == BOPTools_AlgoTools.ComputeState(aP, aSolid, 1.0e-7, aCtx), $"{aName} point {i}");
            }
            // vertices, edges and faces of the solid itself are ON
            foreach (TopoDS_Edge aE in BOPToolsTestShapes.Edges(aSolid).Take(10))
            {
                Assert.Equal(TopAbs_ON, BOPTools_AlgoTools.ComputeState(aE, aSolid, 1.0e-7, aCtx));
                Assert.Equal(TopAbs_ON, BOPTools_AlgoTools.ComputeStateByOnePoint(TopExp.FirstVertex(aE), aSolid, 1.0e-7, aCtx));
            }
        }
    }

    [Fact]
    public void ComputeState_of_faces_and_containers()
    {
        TopoDS_Solid anOuter = BoxSolid(0, 0, 0, 10, 10, 10);
        TopoDS_Solid anInner = BoxSolid(2, 2, 2, 3, 3, 3);
        TopoDS_Solid aFar = BoxSolid(20, 0, 0, 3, 3, 3);
        IntTools_Context aCtx = new();
        NCollection_IndexedMap<TopoDS_Shape> aBounds = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(anOuter, TopAbs_EDGE, aBounds);
        foreach (TopoDS_Face aF in BOPToolsTestShapes.Faces(anInner))
        {
            Assert.Equal(TopAbs_IN, BOPTools_AlgoTools.ComputeState(aF, anOuter, 1.0e-7, aBounds, aCtx));
            Assert.Equal(TopAbs_IN, BOPTools_AlgoTools.ComputeStateByOnePoint(aF, anOuter, 1.0e-7, aCtx));
        }
        foreach (TopoDS_Face aF in BOPToolsTestShapes.Faces(aFar))
        {
            Assert.Equal(TopAbs_OUT, BOPTools_AlgoTools.ComputeStateByOnePoint(aF, anOuter, 1.0e-7, aCtx));
        }
        // the solid's own faces: all edges are bounds, so a point inside the face is classified
        NCollection_IndexedMap<TopoDS_Shape> aOwn = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(anOuter, TopAbs_EDGE, aOwn);
        foreach (TopoDS_Face aF in BOPToolsTestShapes.Faces(anOuter))
        {
            Assert.Equal(TopAbs_ON, BOPTools_AlgoTools.ComputeState(aF, anOuter, 1.0e-7, aOwn, aCtx));
        }
        // OCCT: a container's state is UNKNOWN (the sub-shape's state is discarded)
        Assert.Equal(TopAbs_UNKNOWN, BOPTools_AlgoTools.ComputeStateByOnePoint(anInner, anOuter, 1.0e-7, aCtx));
    }

    // The unit box, its edge on the z axis, the two box faces on it, and a planar face through that edge going
    // along (dx, dy, 0), sharing the edge and its vertices.
    private static (TopoDS_Solid Box, TopoDS_Edge E, TopoDS_Face F1, TopoDS_Face F2, TopoDS_Face Fin) FanAtZAxis(double dx, double dy)
    {
        TopoDS_Solid aBox = BoxSolid(0, 0, 0, 1, 1, 1);
        TopoDS_Edge aZ = new();
        foreach (TopoDS_Edge aE in BOPToolsTestShapes.Edges(aBox))
        {
            TopExp.Vertices(aE, out TopoDS_Vertex a, out TopoDS_Vertex b);
            gp_Pnt pa = BRep_Tool.Pnt(a), pb = BRep_Tool.Pnt(b);
            if (pa.X() == 0 && pa.Y() == 0 && pb.X() == 0 && pb.Y() == 0)
            {
                aZ = aE;
                break;
            }
        }
        Assert.False(aZ.IsNull());
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMEF = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aBox, TopAbs_EDGE, TopAbs_FACE, aMEF);
        NCollection_List<TopoDS_Shape> aLF = aMEF.FindFromKey(aZ);
        Assert.Equal(2, aLF.Extent());
        TopExp.Vertices(aZ, out TopoDS_Vertex aV0, out TopoDS_Vertex aV1);
        gp_Pnt p0 = BRep_Tool.Pnt(aV0), p1 = BRep_Tool.Pnt(aV1);
        TopoDS_Vertex aW0 = TopoDS.Vertex(new BRepBuilderAPI_MakeVertex(new gp_Pnt(p0.X() + dx, p0.Y() + dy, p0.Z())).InternalShape());
        TopoDS_Vertex aW1 = TopoDS.Vertex(new BRepBuilderAPI_MakeVertex(new gp_Pnt(p1.X() + dx, p1.Y() + dy, p1.Z())).InternalShape());
        TopoDS_Edge e1 = TopoDS.Edge(new BRepBuilderAPI_MakeEdge(aV1, aW1).InternalShape());
        TopoDS_Edge e2 = TopoDS.Edge(new BRepBuilderAPI_MakeEdge(aW1, aW0).InternalShape());
        TopoDS_Edge e3 = TopoDS.Edge(new BRepBuilderAPI_MakeEdge(aW0, aV0).InternalShape());
        TopoDS_Edge aZf = TopoDS.Edge(aZ.Oriented(TopAbs_FORWARD));
        TopoDS_Wire aW = TopoDS.Wire(new BRepBuilderAPI_MakeWire(aZf, e1, e2, e3).InternalShape());
        TopoDS_Face aFin = TopoDS.Face(new BRepBuilderAPI_MakeFace(aW, true).InternalShape());
        return (aBox, aZ, TopoDS.Face(aLF.First()), TopoDS.Face(aLF.Last()), aFin);
    }

    [Fact]
    public void IsInternalFace_by_angles_and_by_classification()
    {
        IntTools_Context aCtx = new();
        (TopoDS_Solid aBox, TopoDS_Edge aE, TopoDS_Face aF1, TopoDS_Face aF2, TopoDS_Face aIn) = FanAtZAxis(0.5, 0.5);
        Assert.Equal(1, BOPTools_AlgoTools.IsInternalFace(aIn, aE, aF1, aF2, aCtx));
        NCollection_List<TopoDS_Shape> aLF = new();
        aLF.Append(aF1);
        aLF.Append(aF2);
        Assert.Equal(1, BOPTools_AlgoTools.IsInternalFace(aIn, aE, aLF, aCtx));
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMEF = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aBox, TopAbs_EDGE, TopAbs_FACE, aMEF);
        Assert.True(BOPTools_AlgoTools.IsInternalFace(aIn, aBox, aMEF, 1.0e-7, aCtx));

        (_, TopoDS_Edge aE2, TopoDS_Face aG1, TopoDS_Face aG2, TopoDS_Face aOut) = FanAtZAxis(-0.5, -0.5);
        Assert.Equal(0, BOPTools_AlgoTools.IsInternalFace(aOut, aE2, aG1, aG2, aCtx));

        // a face sharing no edge with the solid: classified by a point
        TopoDS_Solid anInner = BoxSolid(0.25, 0.25, 0.25, 0.5, 0.5, 0.5);
        TopoDS_Face anInnerFace = BOPToolsTestShapes.Faces(anInner)[0];
        Assert.True(BOPTools_AlgoTools.IsInternalFace(anInnerFace, aBox, aMEF, 1.0e-7, aCtx));
        TopoDS_Face aFarFace = BOPToolsTestShapes.Faces(BoxSolid(5, 5, 5, 1, 1, 1))[0];
        Assert.False(BOPTools_AlgoTools.IsInternalFace(aFarFace, aBox, aMEF, 1.0e-7, aCtx));
    }

    [Fact]
    public void GetFaceOff_finds_the_face_making_the_smallest_angle()
    {
        IntTools_Context aCtx = new();
        (_, TopoDS_Edge aE, TopoDS_Face aF1, TopoDS_Face aF2, TopoDS_Face aIn) = FanAtZAxis(0.5, 0.5);
        TopoDS_Edge aE1 = new(), aE2 = new(), aEIn = new();
        Assert.True(BOPTools_AlgoTools.GetEdgeOnFace(aE, aF1, ref aE1));
        Assert.True(BOPTools_AlgoTools.GetEdgeOnFace(aE, aF2, ref aE2));
        Assert.True(BOPTools_AlgoTools.GetEdgeOnFace(aE, aIn, ref aEIn));
        Assert.True(aE1.IsSame(aE) && aE2.IsSame(aE));
        Assert.NotEqual(aE1.Orientation(), aE2.Orientation());
        TopoDS_Edge aOff = new();
        Assert.True(BOPTools_AlgoTools.GetEdgeOff(aE1, aF2, ref aOff));
        Assert.Equal(aE2, aOff);
        Assert.False(BOPTools_AlgoTools.GetEdgeOff(aE1, aF1, ref aOff));

        NCollection_List<BOPTools_CoupleOfShape> aLCS = new();
        aLCS.Append(new BOPTools_CoupleOfShape().WithShape1(aEIn).WithShape2(aIn));
        aLCS.Append(new BOPTools_CoupleOfShape().WithShape1(aE2).WithShape2(aF2));
        TopoDS_Face aFOff = new();
        Assert.True(BOPTools_AlgoTools.GetFaceOff(aE1, aF1, aLCS, ref aFOff, aCtx));
        Assert.True(aFOff.IsSame(aIn) || aFOff.IsSame(aF2));
    }

    [Fact]
    public void AreFacesSameDomain_and_Sense()
    {
        IntTools_Context aCtx = new();
        List<TopoDS_Face> aFaces = BOPToolsTestShapes.Faces(BOPToolsTestShapes.Box());
        TopoDS_Face aF = aFaces[0];
        TopoDS_Face aSp = SplitOf(aF);
        Assert.True(BOPTools_AlgoTools.AreFacesSameDomain(aSp, aF, aCtx));
        Assert.False(BOPTools_AlgoTools.AreFacesSameDomain(aF, aFaces[1], aCtx));
        Assert.Equal(1, BOPTools_AlgoTools.Sense(aF, aF, aCtx));
        Assert.Equal(-1, BOPTools_AlgoTools.Sense(aF, TopoDS.Face(aF.Reversed()), aCtx));
        // no common edge
        Assert.Equal(0, BOPTools_AlgoTools.Sense(aF, aSp, aCtx));
    }

    [Fact]
    public void IsHole_by_the_wire_orientation()
    {
        TopoDS_Face aF = TopoDS.Face(new BRepBuilderAPI_MakeFace(new gp_Pln(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)), 0, 2, 0, 1).InternalShape());
        TopoDS_Wire aW = TopoDS.Wire(TopExp_ExplorerFirst(aF, TopAbs_WIRE));
        Assert.False(BOPTools_AlgoTools.IsHole(aW, aF));
        Assert.True(BOPTools_AlgoTools.IsHole(aW.Reversed(), aF));
    }

    [Fact]
    public void MakeContainer_makes_each_type()
    {
        foreach (TopAbs_ShapeEnum aType in new[] { TopAbs_COMPOUND, TopAbs_COMPSOLID, TopAbs_SOLID, TopAbs_SHELL, TopAbs_WIRE })
        {
            TopoDS_Shape aC = new();
            BOPTools_AlgoTools.MakeContainer(aType, ref aC);
            Assert.Equal(aType, aC.ShapeType());
            Assert.False(new TopoDS_Iterator(aC).More());
        }
        TopoDS_Shape aKeep = BOPToolsTestShapes.Box();
        BOPTools_AlgoTools.MakeContainer(TopAbs_FACE, ref aKeep);
        Assert.Equal(TopAbs_SOLID, aKeep.ShapeType());
    }

    [Fact]
    public void MakePCurve_adds_pcurves_on_both_faces()
    {
        TopoDS_Solid aBox = BoxSolid(0, 0, 0, 2, 2, 2);
        // the two faces at the edge x = 2, z = 2: the line from (2, 0.5, 2) to (2, 1.5, 2) lies on both
        TopoDS_Face aFx = new(), aFz = new();
        foreach (TopoDS_Face aF in BOPToolsTestShapes.Faces(aBox))
        {
            BRepGProp_Face aProps = new(aF);
            BRepAdaptor_Surface aBAS = new(aF);
            gp_Pln aPln = aBAS.Plane();
            gp_Pnt aLoc = aPln.Location();
            gp_Dir aN = aPln.Axis().Direction();
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
        Assert.False(aFx.IsNull() || aFz.IsNull());
        Geom_Line aLine = new(new gp_Pnt(2, 0.5, 2), new gp_Dir(0, 1, 0));
        TopoDS_Edge aE = TopoDS.Edge(new BRepBuilderAPI_MakeEdge(aLine, 0, 1).InternalShape());
        IntTools_Curve aIC = new(aLine, null, null);
        BOPTools_AlgoTools.MakePCurve(aE, aFx, aFz, aIC, true, true, new IntTools_Context());
        foreach (TopoDS_Face aF in new[] { aFx, aFz })
        {
            Geom2d_Curve? aC2 = BRep_Tool.CurveOnSurface(aE, aF, out double f, out double l);
            Assert.NotNull(aC2);
            Geom_Surface aS = BRep_Tool.Surface(aF, out TopLoc_Location aL)!;
            for (int i = 0; i <= 10; ++i)
            {
                double t = f + (l - f) * i / 10.0;
                gp_Pnt2d uv = aC2!.Value(t);
                gp_Pnt p = aS.Value(uv.X(), uv.Y()).Transformed(aL.Transformation());
                Assert.True(p.Distance(aLine.Value(t)) < 1.0e-9);
            }
        }
        Assert.True(BRep_Tool.SameParameter(aE));
    }

    [Fact]
    public void MakeEdge_MakeSectEdge_MakeVertex_and_ComputeVV()
    {
        Geom_Line aLine = new(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 0));
        IntTools_Curve aIC = new(aLine, null, null);
        BOPTools_AlgoTools.MakeNewVertex(new gp_Pnt(0, 0, 0), 1.0e-7, out TopoDS_Vertex aV1);
        BOPTools_AlgoTools.MakeNewVertex(new gp_Pnt(3, 0, 0), 1.0e-7, out TopoDS_Vertex aV2);
        BOPTools_AlgoTools.MakeEdge(aIC, aV1, 0, aV2, 3, 1.0e-4, out TopoDS_Edge aE);
        BRep_Tool.Range(aE, out double f, out double l);
        Assert.Equal(0.0, f);
        Assert.Equal(3.0, l);
        Assert.Equal(1.0e-4, BRep_Tool.Tolerance(aE));
        Assert.Equal(1.0e-4 + 1.0e-12, BRep_Tool.Tolerance(aV1));

        BOPTools_AlgoTools.MakeNewVertex(new gp_Pnt(0.5, 0, 0), 1.0e-7, out TopoDS_Vertex aVh);
        BOPTools_AlgoTools.MakeSectEdge(aIC, aVh, 0.5, aV2, 3, out TopoDS_Edge aE2);
        BRep_Tool.Range(aE2, out f, out l);
        Assert.Equal(0.5, f);

        Assert.Equal(0, BOPTools_AlgoTools.ComputeVV(aV1, new gp_Pnt(1.0e-4, 0, 0), 0.0));
        Assert.Equal(1, BOPTools_AlgoTools.ComputeVV(aV1, new gp_Pnt(1.0e-3, 0, 0), 0.0));
        BOPTools_AlgoTools.MakeNewVertex(new gp_Pnt(1.0e-4, 0, 0), 1.0e-7, out TopoDS_Vertex aV3);
        Assert.Equal(0, BOPTools_AlgoTools.ComputeVV(aV1, aV3));
        Assert.Equal(1, BOPTools_AlgoTools.ComputeVV(aV1, aV2));
        Assert.Equal(0, BOPTools_AlgoTools.ComputeVV(aV1, aV2, 3.0));

        NCollection_List<TopoDS_Shape> aLV = new();
        TopoDS_Vertex aVnew = new();
        BOPTools_AlgoTools.MakeVertex(aLV, ref aVnew);
        Assert.True(aVnew.IsNull());
        aLV.Append(aV1);
        BOPTools_AlgoTools.MakeVertex(aLV, ref aVnew);
        Assert.Equal(aV1, aVnew);
        aLV.Append(aV3);
        BOPTools_AlgoTools.MakeVertex(aLV, ref aVnew);
        Assert.False(aVnew.IsSame(aV1) || aVnew.IsSame(aV3));
        Assert.True(BRep_Tool.Pnt(aVnew).Distance(BRep_Tool.Pnt(aV1)) <= BRep_Tool.Tolerance(aVnew));
        Assert.True(BRep_Tool.Pnt(aVnew).Distance(BRep_Tool.Pnt(aV3)) <= BRep_Tool.Tolerance(aVnew));
    }

    [Fact]
    public void MakeConnexityBlocks_three_forms()
    {
        TopoDS_Solid aB1 = BoxSolid(0, 0, 0, 1, 1, 1), aB2 = BoxSolid(5, 0, 0, 1, 1, 1);
        TopoDS_Compound aC = new();
        BRep_Builder aBB = new();
        aBB.MakeCompound(ref aC);
        aBB.Add(aC, aB1);
        aBB.Add(aC, aB2);
        NCollection_List<TopoDS_Shape> aLCB = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(aC, TopAbs_EDGE, TopAbs_FACE, aLCB);
        Assert.Equal(2, aLCB.Extent());
        foreach (TopoDS_Shape aBlock in aLCB)
        {
            Assert.Equal(6, BOPToolsTestShapes.Faces(aBlock).Count);
        }

        // list form: regular for a box's faces, irregular when a face is given twice
        NCollection_List<TopoDS_Shape> aLS = new(BOPToolsTestShapes.Faces(aB1).Select(f => (TopoDS_Shape)f));
        NCollection_List<BOPTools_ConnexityBlock> aLCB2 = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(aLS, TopAbs_EDGE, TopAbs_FACE, aLCB2);
        Assert.Equal(1, aLCB2.Extent());
        Assert.True(aLCB2.First().IsRegular());
        Assert.Equal(6, aLCB2.First().Shapes().Extent());
        aLS.Append(aLS.First());
        NCollection_List<BOPTools_ConnexityBlock> aLCB3 = new();
        BOPTools_AlgoTools.MakeConnexityBlocks(aLS, TopAbs_EDGE, TopAbs_FACE, aLCB3);
        Assert.False(aLCB3.First().IsRegular());
        Assert.Equal(7, aLCB3.First().Shapes().Extent());

        // MakeConnexityBlock: avoiding the four vertical edges of the box splits off nothing (top and bottom still
        // connect through the side faces), avoiding every edge leaves the first face alone
        NCollection_IndexedMap<TopoDS_Shape> aAvoid = new(TopTools_ShapeMapHasher.Instance);
        NCollection_List<TopoDS_Shape> aOne = new();
        BOPTools_AlgoTools.MakeConnexityBlock(new NCollection_List<TopoDS_Shape>(BOPToolsTestShapes.Faces(aB1).Select(f => (TopoDS_Shape)f)), aAvoid, aOne);
        Assert.Equal(6, aOne.Extent());
        TopExp.MapShapes(aB1, TopAbs_EDGE, aAvoid);
        aOne.Clear();
        BOPTools_AlgoTools.MakeConnexityBlock(new NCollection_List<TopoDS_Shape>(BOPToolsTestShapes.Faces(aB1).Select(f => (TopoDS_Shape)f)), aAvoid, aOne);
        Assert.Equal(1, aOne.Extent());
    }

    [Fact]
    public void OrientEdgesOnWire_chains_reversed_edges()
    {
        TopoDS_Face aF = BOPToolsTestShapes.Faces(BoxSolid(0, 0, 0, 1, 2, 3))[0];
        List<TopoDS_Edge> aEdges = BOPToolsTestShapes.Edges(aF);
        TopoDS_Wire aW = new();
        BRep_Builder aBB = new();
        aBB.MakeWire(ref aW);
        for (int i = 0; i < aEdges.Count; ++i)
        {
            aBB.Add(aW, i % 2 == 0 ? aEdges[i] : aEdges[i].Reversed());
        }
        TopoDS_Shape aWS = aW;
        BOPTools_AlgoTools.OrientEdgesOnWire(ref aWS);
        List<TopoDS_Edge> aOut = [];
        for (TopoDS_Iterator it = new(aWS); it.More(); it.Next())
        {
            aOut.Add(TopoDS.Edge(it.Value()));
        }
        Assert.Equal(aEdges.Count, aOut.Count);
        // every vertex ends one edge and starts the other (the edges are oriented head to tail; OCCT adds them in
        // traversal order, not in chain order)
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aVE = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aWS, TopAbs_VERTEX, TopAbs_EDGE, aVE);
        for (int i = 1; i <= aVE.Extent(); ++i)
        {
            TopoDS_Shape aV = aVE.FindKey(i);
            int aNbFirst = 0, aNbLast = 0;
            foreach (TopoDS_Edge e in aOut)
            {
                TopExp.Vertices(e, out TopoDS_Vertex a, out TopoDS_Vertex b, true);
                aNbFirst += a.IsSame(aV) ? 1 : 0;
                aNbLast += b.IsSame(aV) ? 1 : 0;
            }
            Assert.Equal((1, 1), (aNbFirst, aNbLast));
        }
    }

    [Fact]
    public void OrientFacesOnShell_makes_shared_edges_opposite()
    {
        TopoDS_Shell aShell = TopoDS.Shell(TopExp_ExplorerFirst(BoxSolid(0, 0, 0, 1, 2, 3), TopAbs_SHELL));
        List<TopoDS_Face> aFaces = BOPToolsTestShapes.Faces(aShell);
        TopoDS_Shell aMixed = new();
        BRep_Builder aBB = new();
        aBB.MakeShell(ref aMixed);
        for (int i = 0; i < aFaces.Count; ++i)
        {
            aBB.Add(aMixed, i is 1 or 4 ? aFaces[i].Reversed() : aFaces[i]);
        }
        TopoDS_Shape aS = aMixed;
        BOPTools_AlgoTools.OrientFacesOnShell(ref aS);
        Assert.Equal(6, BOPToolsTestShapes.Faces(aS).Count);
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMEF = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aS, TopAbs_EDGE, TopAbs_FACE, aMEF);
        for (int i = 1; i <= aMEF.Extent(); ++i)
        {
            TopoDS_Edge aE = TopoDS.Edge(aMEF.FindKey(i));
            NCollection_List<TopoDS_Shape> aLF = aMEF.FindFromIndex(i);
            Assert.Equal(2, aLF.Extent());
            TopAbs_Orientation o1 = TopAbs_INTERNAL, o2 = TopAbs_INTERNAL;
            for (TopExp_Explorer x = new(aLF.First(), TopAbs_EDGE); x.More(); x.Next())
            {
                if (x.Current().IsSame(aE)) { o1 = x.Current().Orientation(); }
            }
            for (TopExp_Explorer x = new(aLF.Last(), TopAbs_EDGE); x.More(); x.Next())
            {
                if (x.Current().IsSame(aE)) { o2 = x.Current().Orientation(); }
            }
            Assert.NotEqual(o1, o2);
        }
    }

    [Fact]
    public void IsMicroEdge_and_IsBlockInOnFace()
    {
        IntTools_Context aCtx = new();
        TopoDS_Solid aBox = BoxSolid(0, 0, 0, 1, 2, 3);
        TopoDS_Edge aE = BOPToolsTestShapes.Edges(aBox)[0];
        Assert.False(BOPTools_AlgoTools.IsMicroEdge(aE, aCtx));
        TopoDS_Edge aTiny = TopoDS.Edge(new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(1.0e-8, 0, 0)).InternalShape());
        Assert.True(BOPTools_AlgoTools.IsMicroEdge(aTiny, aCtx));
        TopoDS_Edge aDegen = new();
        foreach (TopoDS_Edge x in BOPToolsTestShapes.Edges(BOPToolsTestShapes.Sphere()))
        {
            if (BRep_Tool.Degenerated(x)) { aDegen = x; break; }
        }
        Assert.True(BOPTools_AlgoTools.IsMicroEdge(aDegen, aCtx));

        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMEF = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aBox, TopAbs_EDGE, TopAbs_FACE, aMEF);
        BRep_Tool.Range(aE, out double f, out double l);
        IntTools_Range aR = new(f, l);
        foreach (TopoDS_Shape aF in aMEF.FindFromKey(aE))
        {
            Assert.True(BOPTools_AlgoTools.IsBlockInOnFace(aR, TopoDS.Face(aF), aE, aCtx));
        }
        int aNbOff = 0;
        foreach (TopoDS_Face aF in BOPToolsTestShapes.Faces(aBox))
        {
            if (!aMEF.FindFromKey(aE).Any(x => x.IsSame(aF)))
            {
                aNbOff += BOPTools_AlgoTools.IsBlockInOnFace(aR, aF, aE, aCtx) ? 0 : 1;
            }
        }
        Assert.Equal(4, aNbOff);
    }

    [Fact]
    public void AlgoTools_2_vertices_edges_and_ranges()
    {
        BOPTools_AlgoTools.MakeNewVertex(new gp_Pnt(0, 0, 0), 1.0e-3, out TopoDS_Vertex aV1);
        BOPTools_AlgoTools.MakeNewVertex(new gp_Pnt(0.01, 0, 0), 1.0e-4, out TopoDS_Vertex aV2);
        BOPTools_AlgoTools.MakeNewVertex(aV1, aV2, out TopoDS_Vertex aVm);
        Assert.Equal(0.005, BRep_Tool.Pnt(aVm).X(), 15);
        Assert.Equal(1.0e-3 + 0.005, BRep_Tool.Tolerance(aVm), 15);

        TopoDS_Edge aE = TopoDS.Edge(new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(4, 0, 0)).InternalShape());
        TopoDS_Edge aE2 = TopoDS.Edge(new BRepBuilderAPI_MakeEdge(new gp_Pnt(1, 1.0e-3, 0), new gp_Pnt(1, 5, 0)).InternalShape());
        BOPTools_AlgoTools.MakeNewVertex(aE, 1.0, aE2, 0.0, out TopoDS_Vertex aVee);
        Assert.Equal(1.0, BRep_Tool.Pnt(aVee).X(), 15);
        TopoDS_Face aF = TopoDS.Face(new BRepBuilderAPI_MakeFace(new gp_Pln(), 0, 1, 0, 1).InternalShape());
        BOPTools_AlgoTools.MakeNewVertex(aE, 2.0, aF, out TopoDS_Vertex aVef);
        Assert.Equal(BRep_Tool.Tolerance(aE) + BRep_Tool.Tolerance(aF) + 1.0e-12, BRep_Tool.Tolerance(aVef));

        // UpdateVertex forms
        BOPTools_AlgoTools.MakeNewVertex(new gp_Pnt(2, 0.01, 0), 1.0e-7, out TopoDS_Vertex aVu);
        BOPTools_AlgoTools.UpdateVertex(aE, 2.0, aVu);
        Assert.Equal(0.01 + 1.0e-12, BRep_Tool.Tolerance(aVu), 15);
        BOPTools_AlgoTools.MakeNewVertex(new gp_Pnt(2, 0.02, 0), 1.0e-7, out TopoDS_Vertex aVc);
        BOPTools_AlgoTools.UpdateVertex(new IntTools_Curve(BRep_Tool.Curve(aE, out _, out _), null, null), 2.0, aVc);
        Assert.Equal(0.02 + 1.0e-12, BRep_Tool.Tolerance(aVc), 15);
        BOPTools_AlgoTools.MakeNewVertex(new gp_Pnt(2, 0.05, 0), 1.0e-3, out TopoDS_Vertex aVn);
        BOPTools_AlgoTools.UpdateVertex(aVu, aVn);
        Assert.True(BRep_Tool.Tolerance(aVu) > 0.04);

        // split and copy
        BOPTools_AlgoTools.MakeNewVertex(new gp_Pnt(1, 0, 0), 1.0e-7, out TopoDS_Vertex aS1);
        BOPTools_AlgoTools.MakeNewVertex(new gp_Pnt(3, 0, 0), 1.0e-7, out TopoDS_Vertex aS2);
        BOPTools_AlgoTools.MakeSplitEdge(aE, aS2, 3.0, aS1, 1.0, out TopoDS_Edge aSp);
        BRep_Tool.Range(aSp, out double f, out double l);
        Assert.Equal((1.0, 3.0), (f, l));
        Assert.Same(BRep_Tool.Curve(aE, out _, out _), BRep_Tool.Curve(aSp, out _, out _));
        TopExp.Vertices(aSp, out TopoDS_Vertex aVF, out TopoDS_Vertex aVL);
        Assert.True(aVF.IsSame(aS1) && aVL.IsSame(aS2));
        TopoDS_Edge aCopy = BOPTools_AlgoTools.CopyEdge(TopoDS.Edge(aE.Reversed()));
        Assert.False(aCopy.IsSame(aE));
        Assert.Equal(TopAbs_REVERSED, aCopy.Orientation());
        TopExp.Vertices(aCopy, out TopoDS_Vertex aCF, out TopoDS_Vertex aCL);
        TopExp.Vertices(aE, out TopoDS_Vertex aEF, out TopoDS_Vertex aEL);
        Assert.True(aCF.IsSame(aEF) && aCL.IsSame(aEL));

        // CorrectRange: lines unchanged, circles shrunk by the tolerances' resolution
        IntTools_Range aR = new(0.5, 3.5);
        BOPTools_AlgoTools.CorrectRange(aE, aE2, aR, out IntTools_Range aNR);
        Assert.Equal(aR, aNR);
        TopoDS_Edge aCircle = TopoDS.Edge(new BRepBuilderAPI_MakeEdge(new gp_Circ(new gp_Ax2(), 10)).InternalShape());
        BOPTools_AlgoTools.CorrectRange(aCircle, aE, new IntTools_Range(1, 2), out aNR);
        double aRes = 2.0 * (BRep_Tool.Tolerance(aCircle) + BRep_Tool.Tolerance(aE)) / 10.0;
        Assert.Equal(1 + aRes, aNR.First(), 14);
        Assert.Equal(2 - aRes, aNR.Last(), 14);
        BOPTools_AlgoTools.CorrectRange(aCircle, aF, new IntTools_Range(1, 2), out aNR);
        Assert.Equal(1 + BRep_Tool.Tolerance(aF) / 10.0, aNR.First(), 14);
        BOPTools_AlgoTools.CorrectRange(aCircle, aF, new IntTools_Range(1, 1 + 1.0e-10), out aNR);
        Assert.Equal(new IntTools_Range(1, 1 + 1.0e-10), aNR);
    }

    [Fact]
    public void Dimensions_and_TreatCompound()
    {
        TopoDS_Solid aB = BoxSolid(0, 0, 0, 1, 1, 1);
        TopoDS_Face aF = BOPToolsTestShapes.Faces(aB)[0];
        TopoDS_Edge aE = BOPToolsTestShapes.Edges(aB)[0];
        Assert.Equal(3, BOPTools_AlgoTools.Dimension(aB));
        Assert.Equal(2, BOPTools_AlgoTools.Dimension(aF));
        Assert.Equal(1, BOPTools_AlgoTools.Dimension(aE));
        Assert.Equal(0, BOPTools_AlgoTools.Dimension(TopExp.FirstVertex(aE)));
        TopoDS_Compound aC = new(), aInner = new(), aEmpty = new();
        BRep_Builder aBB = new();
        aBB.MakeCompound(ref aC);
        aBB.MakeCompound(ref aInner);
        aBB.MakeCompound(ref aEmpty);
        BOPTools_AlgoTools.Dimensions(aEmpty, out int aMin, out int aMax);
        Assert.Equal((-1, -1), (aMin, aMax));
        aBB.Add(aInner, aF);
        aBB.Add(aInner, aE);
        aBB.Add(aC, aInner);
        aBB.Add(aC, aF);
        aBB.Add(aC, aEmpty);
        BOPTools_AlgoTools.Dimensions(aC, out aMin, out aMax);
        Assert.Equal((1, 2), (aMin, aMax));
        Assert.Equal(-1, BOPTools_AlgoTools.Dimension(aC));
        NCollection_List<TopoDS_Shape> aL = new();
        BOPTools_AlgoTools.TreatCompound(aC, aL);
        Assert.Equal(3, aL.Extent());
        aL.Clear();
        BOPTools_AlgoTools.TreatCompound(aC, aL, new NCollection_Map<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance));
        Assert.Equal(2, aL.Extent());
    }

    [Fact]
    public void MakePCurve_and_MakeEdge_throw_on_frozen_input()
    {
        TopoDS_Shape aBox = BOPToolsTestShapes.Box(); // published: frozen
        TopoDS_Vertex aV = TopoDS.Vertex(TopExp_ExplorerFirst(aBox, TopAbs_VERTEX));
        Geom_Line aLine = new(BRep_Tool.Pnt(aV), new gp_Dir(1, 0, 0));
        BOPTools_AlgoTools.MakeNewVertex(new gp_Pnt(100, 0, 0), 1.0e-7, out TopoDS_Vertex aV2);
        Assert.ThrowsAny<Exception>(() =>
        {
            BOPTools_AlgoTools.MakeEdge(new IntTools_Curve(aLine, null, null), aV, 0, aV2, 1, 1.0e-3, out _);
        });
    }

    // A reused context answers as a fresh context per call, over shuffled inputs.
    [Fact]
    public void Reused_context_answers_as_fresh_ones()
    {
        Random aRnd = new(6441);
        foreach ((string aName, TopoDS_Shape aShape) in Shapes())
        {
            List<TopoDS_Face> aFaces = BOPToolsTestShapes.Faces(aShape);
            List<TopoDS_Edge> aEdges = BOPToolsTestShapes.Edges(aShape).Where(e => !BRep_Tool.Degenerated(e) && BRep_Tool.Curve(e, out _, out _) is not null).ToList();
            TopExp_Explorer aSolids = new(aShape, TopAbs_SOLID);
            TopoDS_Solid? aSolid = aSolids.More() ? TopoDS.Solid(aSolids.Current()) : null;
            NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aMEF = new(TopTools_ShapeMapHasher.Instance);
            if (aSolid is not null)
            {
                TopExp.MapShapesAndAncestors(aSolid.Value, TopAbs_EDGE, TopAbs_FACE, aMEF);
            }
            List<Func<IntTools_Context, string>> aQueries = [];
            foreach (TopoDS_Face aF in aFaces.Take(40))
            {
                TopoDS_Face aSp = SplitOf(aF);
                aQueries.Add(c => $"{BOPTools_AlgoTools.IsSplitToReverse(aSp, aF, c, out int e)} {e}");
                if (aSolid is not null)
                {
                    TopoDS_Solid s = aSolid.Value;
                    aQueries.Add(c => BOPTools_AlgoTools.IsInternalFace(aSp, s, aMEF, 1.0e-7, c).ToString());
                    aQueries.Add(c => BOPTools_AlgoTools.ComputeStateByOnePoint(aSp, s, 1.0e-7, c).ToString());
                }
            }
            foreach (TopoDS_Edge aE in aEdges.Take(40))
            {
                Geom_Curve aC = BRep_Tool.Curve(aE, out TopLoc_Location aLoc, out double f, out double l)!;
                double d = 0.3 * (l - f);
                TopoDS_Edge aSp = TopoDS.Edge(new BRepBuilderAPI_MakeEdge((Geom_Curve)aC.Copy(), f + d, l - d).InternalShape().Located(aLoc));
                aQueries.Add(c => $"{BOPTools_AlgoTools.IsSplitToReverse(aSp, aE, c, out int e)} {e}");
                if (aSolid is not null)
                {
                    TopoDS_Solid s = aSolid.Value;
                    aQueries.Add(c => BOPTools_AlgoTools.ComputeState(aSp, s, 1.0e-7, c).ToString());
                }
            }
            string[] aFresh = aQueries.Select(q => q(new IntTools_Context())).ToArray();
            int[] anOrder = Enumerable.Range(0, aQueries.Count).OrderBy(_ => aRnd.Next()).ToArray();
            IntTools_Context aReused = new();
            foreach (int i in anOrder.Concat(anOrder.Reverse()))
            {
                Assert.True(aFresh[i] == aQueries[i](aReused), $"{aName} query {i}");
            }
        }
    }
}
