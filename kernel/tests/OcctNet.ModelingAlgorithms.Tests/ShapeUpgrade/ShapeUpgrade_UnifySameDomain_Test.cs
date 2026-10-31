// Port of: src/ModelingAlgorithms/TKShHealing/GTests/ShapeUpgrade_UnifySameDomain_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// OCCT's GTests of ShapeUpgrade_UnifySameDomain. The shapes are built with the makers' internal (unfrozen) results
/// where OCCT's test adds wires to a made face or pcurves to a made edge.
/// </summary>
public class ShapeUpgrade_UnifySameDomainTest
{
    private struct MissingPCurveShape
    {
        public TopoDS_Shell Shell;
        public TopoDS_Face  ReferenceFace;
        public TopoDS_Edge  MissingPCurveEdge;
        public TopoDS_Edge  ValidPCurveEdge;
    }

    private static TopoDS_Vertex makeVertex(BRep_Builder theBuilder, in gp_Pnt thePoint)
    {
        var aVertex = new TopoDS_Vertex();
        theBuilder.MakeVertex(ref aVertex, thePoint, Precision.Confusion());
        return aVertex;
    }

    private static TopoDS_Edge makeEdge(BRep_Builder    theBuilder,
                                        Geom_Surface?   theSurface,
                                        in TopLoc_Location theLocation,
                                        in TopoDS_Vertex   theFirst,
                                        in TopoDS_Vertex   theLast,
                                        bool            theAddPCurve)
    {
        var anEdgeMaker = new BRepBuilderAPI_MakeEdge(theFirst, theLast);
        if (!anEdgeMaker.IsDone())
        {
            return new TopoDS_Edge();
        }

        TopoDS_Edge anEdge = anEdgeMaker.InternalEdge();
        if (theAddPCurve)
        {
            gp_Pnt        aFirstPoint   = BRep_Tool.Pnt(theFirst);
            gp_Pnt        aLastPoint    = BRep_Tool.Pnt(theLast);
            var           aFirstPoint2d = new gp_Pnt2d(Math.Atan2(aFirstPoint.Y(), aFirstPoint.X()), aFirstPoint.Z());
            var           aLastPoint2d  = new gp_Pnt2d(Math.Atan2(aLastPoint.Y(), aLastPoint.X()), aLastPoint.Z());
            var           aDirection    = new gp_Vec2d(aFirstPoint2d, aLastPoint2d);
            Geom2d_Curve  aPCurve       = new Geom2d_Line(aFirstPoint2d, new gp_Dir2d(aDirection));
            theBuilder.UpdateEdge(anEdge, aPCurve, theSurface, theLocation, Precision.Confusion());
            theBuilder.Range(anEdge, 0.0, aDirection.Magnitude());
        }
        return anEdge;
    }

    private static MissingPCurveShape makeMissingPCurveShape(BRep_Builder theBuilder)
    {
        var           aSurface  = new Geom_CylindricalSurface(new gp_Ax3(gp.XOY()), 1.0);
        var           aLocation = new TopLoc_Location();
        TopoDS_Vertex aVertexA  = makeVertex(theBuilder, aSurface.Value(0.0, 0.0));
        TopoDS_Vertex aVertexB  = makeVertex(theBuilder, aSurface.Value(1.0, 0.0));
        TopoDS_Vertex aVertexC  = makeVertex(theBuilder, aSurface.Value(1.0, 1.0));
        TopoDS_Vertex aVertexD  = makeVertex(theBuilder, aSurface.Value(0.5, -1.0));
        TopoDS_Vertex aVertexE  = makeVertex(theBuilder, aSurface.Value(0.5, 1.0));

        TopoDS_Edge aCommonEdge = makeEdge(theBuilder, aSurface, aLocation, aVertexA, aVertexB, true);
        TopoDS_Edge anEdgeBC    = makeEdge(theBuilder, aSurface, aLocation, aVertexB, aVertexC, false);
        TopoDS_Edge anEdgeCB    = makeEdge(theBuilder, aSurface, aLocation, aVertexC, aVertexB, true);
        TopoDS_Edge anEdgeBD    = makeEdge(theBuilder, aSurface, aLocation, aVertexB, aVertexD, true);
        TopoDS_Edge anEdgeDA    = makeEdge(theBuilder, aSurface, aLocation, aVertexD, aVertexA, true);
        TopoDS_Edge anEdgeAE    = makeEdge(theBuilder, aSurface, aLocation, aVertexA, aVertexE, true);
        TopoDS_Edge anEdgeEB    = makeEdge(theBuilder, aSurface, aLocation, aVertexE, aVertexB, true);

        var aSelfTouchingWire = new TopoDS_Wire();
        theBuilder.MakeWire(ref aSelfTouchingWire);
        theBuilder.Add(aSelfTouchingWire, aCommonEdge);
        theBuilder.Add(aSelfTouchingWire, anEdgeBC);
        theBuilder.Add(aSelfTouchingWire, anEdgeCB);
        theBuilder.Add(aSelfTouchingWire, anEdgeBD);
        theBuilder.Add(aSelfTouchingWire, anEdgeDA);
        aSelfTouchingWire.Closed(true);

        var aSelfTouchingFace = new TopoDS_Face();
        theBuilder.MakeFace(ref aSelfTouchingFace, aSurface, aLocation, Precision.Confusion());
        theBuilder.Add(aSelfTouchingFace, aSelfTouchingWire);

        TopoDS_Edge aReversedCommonEdge = aCommonEdge;
        aReversedCommonEdge.Reverse();
        var aSecondWire = new TopoDS_Wire();
        theBuilder.MakeWire(ref aSecondWire);
        theBuilder.Add(aSecondWire, aReversedCommonEdge);
        theBuilder.Add(aSecondWire, anEdgeAE);
        theBuilder.Add(aSecondWire, anEdgeEB);
        aSecondWire.Closed(true);

        var aSecondFace = new TopoDS_Face();
        theBuilder.MakeFace(ref aSecondFace, aSurface, aLocation, Precision.Confusion());
        theBuilder.Add(aSecondFace, aSecondWire);

        var aResult = new MissingPCurveShape { Shell = new TopoDS_Shell() };
        theBuilder.MakeShell(ref aResult.Shell);
        theBuilder.Add(aResult.Shell, aSelfTouchingFace);
        theBuilder.Add(aResult.Shell, aSecondFace);
        aResult.ReferenceFace     = aSelfTouchingFace;
        aResult.MissingPCurveEdge = anEdgeBC;
        aResult.ValidPCurveEdge   = anEdgeBD;
        return aResult;
    }

    private static MissingPCurveShape makeMissingPCurveTransformationShape(BRep_Builder theBuilder)
    {
        var           aSurface  = new Geom_CylindricalSurface(new gp_Ax3(gp.XOY()), 1.0);
        var           aLocation = new TopLoc_Location();
        TopoDS_Vertex aVertexA  = makeVertex(theBuilder, aSurface.Value(0.0, 0.0));
        TopoDS_Vertex aVertexB  = makeVertex(theBuilder, aSurface.Value(0.5, 0.0));
        TopoDS_Vertex aVertexC  = makeVertex(theBuilder, aSurface.Value(0.5, 1.0));
        TopoDS_Vertex aVertexD  = makeVertex(theBuilder, aSurface.Value(0.0, 1.0));
        TopoDS_Vertex aVertexE  = makeVertex(theBuilder, aSurface.Value(1.0, 0.0));
        TopoDS_Vertex aVertexF  = makeVertex(theBuilder, aSurface.Value(1.0, 1.0));

        TopoDS_Edge anEdgeAB = makeEdge(theBuilder, aSurface, aLocation, aVertexA, aVertexB, true);
        TopoDS_Edge anEdgeBC = makeEdge(theBuilder, aSurface, aLocation, aVertexB, aVertexC, true);
        TopoDS_Edge anEdgeCD = makeEdge(theBuilder, aSurface, aLocation, aVertexC, aVertexD, true);
        TopoDS_Edge anEdgeDA = makeEdge(theBuilder, aSurface, aLocation, aVertexD, aVertexA, true);
        TopoDS_Edge anEdgeBE = makeEdge(theBuilder, aSurface, aLocation, aVertexB, aVertexE, false);
        TopoDS_Edge anEdgeEF = makeEdge(theBuilder, aSurface, aLocation, aVertexE, aVertexF, true);
        TopoDS_Edge anEdgeFC = makeEdge(theBuilder, aSurface, aLocation, aVertexF, aVertexC, true);

        var aLeftWire = new TopoDS_Wire();
        theBuilder.MakeWire(ref aLeftWire);
        theBuilder.Add(aLeftWire, anEdgeAB);
        theBuilder.Add(aLeftWire, anEdgeBC);
        theBuilder.Add(aLeftWire, anEdgeCD);
        theBuilder.Add(aLeftWire, anEdgeDA);
        aLeftWire.Closed(true);

        TopoDS_Edge aReversedEdgeBC = anEdgeBC;
        aReversedEdgeBC.Reverse();
        var aRightWire = new TopoDS_Wire();
        theBuilder.MakeWire(ref aRightWire);
        theBuilder.Add(aRightWire, anEdgeBE);
        theBuilder.Add(aRightWire, anEdgeEF);
        theBuilder.Add(aRightWire, anEdgeFC);
        theBuilder.Add(aRightWire, aReversedEdgeBC);
        aRightWire.Closed(true);

        var aResult = new MissingPCurveShape { Shell = new TopoDS_Shell(), ReferenceFace = new TopoDS_Face() };
        theBuilder.MakeFace(ref aResult.ReferenceFace, aSurface, aLocation, Precision.Confusion());
        theBuilder.Add(aResult.ReferenceFace, aLeftWire);
        var aRightFace = new TopoDS_Face();
        theBuilder.MakeFace(ref aRightFace, aSurface, aLocation, Precision.Confusion());
        theBuilder.Add(aRightFace, aRightWire);
        theBuilder.MakeShell(ref aResult.Shell);
        theBuilder.Add(aResult.Shell, aResult.ReferenceFace);
        theBuilder.Add(aResult.Shell, aRightFace);
        aResult.MissingPCurveEdge = anEdgeBE;
        aResult.ValidPCurveEdge   = anEdgeBC;
        return aResult;
    }

    private static bool addPlanarFacePair(BRep_Builder theBuilder, in TopoDS_Shell theShell)
    {
        Geom_Surface? aNullSurface = null;
        var           aLocation    = new TopLoc_Location();
        TopoDS_Vertex aVertex1     = makeVertex(theBuilder, new gp_Pnt(3.0, 0.0, 0.0));
        TopoDS_Vertex aVertex2     = makeVertex(theBuilder, new gp_Pnt(4.0, 0.0, 0.0));
        TopoDS_Vertex aVertex3     = makeVertex(theBuilder, new gp_Pnt(4.0, 1.0, 0.0));
        TopoDS_Vertex aVertex4     = makeVertex(theBuilder, new gp_Pnt(3.0, 1.0, 0.0));
        TopoDS_Vertex aVertex5     = makeVertex(theBuilder, new gp_Pnt(5.0, 0.0, 0.0));
        TopoDS_Vertex aVertex6     = makeVertex(theBuilder, new gp_Pnt(5.0, 1.0, 0.0));

        TopoDS_Edge anEdge12 = makeEdge(theBuilder, aNullSurface, aLocation, aVertex1, aVertex2, false);
        TopoDS_Edge anEdge23 = makeEdge(theBuilder, aNullSurface, aLocation, aVertex2, aVertex3, false);
        TopoDS_Edge anEdge34 = makeEdge(theBuilder, aNullSurface, aLocation, aVertex3, aVertex4, false);
        TopoDS_Edge anEdge41 = makeEdge(theBuilder, aNullSurface, aLocation, aVertex4, aVertex1, false);
        TopoDS_Edge anEdge25 = makeEdge(theBuilder, aNullSurface, aLocation, aVertex2, aVertex5, false);
        TopoDS_Edge anEdge56 = makeEdge(theBuilder, aNullSurface, aLocation, aVertex5, aVertex6, false);
        TopoDS_Edge anEdge63 = makeEdge(theBuilder, aNullSurface, aLocation, aVertex6, aVertex3, false);
        if (anEdge12.IsNull() || anEdge23.IsNull() || anEdge34.IsNull() || anEdge41.IsNull() || anEdge25.IsNull()
            || anEdge56.IsNull() || anEdge63.IsNull())
        {
            return false;
        }

        var aLeftWireMaker = new BRepBuilderAPI_MakeWire();
        aLeftWireMaker.Add(anEdge12);
        aLeftWireMaker.Add(anEdge23);
        aLeftWireMaker.Add(anEdge34);
        aLeftWireMaker.Add(anEdge41);
        if (!aLeftWireMaker.IsDone())
        {
            return false;
        }

        TopoDS_Edge aReversedEdge23 = anEdge23;
        aReversedEdge23.Reverse();
        var aRightWireMaker = new BRepBuilderAPI_MakeWire();
        aRightWireMaker.Add(aReversedEdge23);
        aRightWireMaker.Add(anEdge25);
        aRightWireMaker.Add(anEdge56);
        aRightWireMaker.Add(anEdge63);
        if (!aRightWireMaker.IsDone())
        {
            return false;
        }

        var aPlanarSurface  = new gp_Pln(new gp_Ax3(gp.XOY()));
        var aLeftFaceMaker  = new BRepBuilderAPI_MakeFace(aPlanarSurface, aLeftWireMaker.InternalWire());
        var aRightFaceMaker = new BRepBuilderAPI_MakeFace(aPlanarSurface, aRightWireMaker.InternalWire());
        if (!aLeftFaceMaker.IsDone() || !aRightFaceMaker.IsDone())
        {
            return false;
        }
        theBuilder.Add(theShell, aLeftFaceMaker.InternalFace());
        theBuilder.Add(theShell, aRightFaceMaker.InternalFace());
        return true;
    }

    private static int countFaces(in TopoDS_Shape theShape)
    {
        int aFaceCount = 0;
        for (var anExp = new TopExp_Explorer(theShape, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            ++aFaceCount;
        }
        return aFaceCount;
    }

    // Test for issue #925: Infinite loop when processing internal edges.
    // This test verifies that UnifySameDomain properly handles internal edges
    // without entering an infinite loop.
    [Fact]
    public void InternalEdgesTermination_Issue925()
    {
        // Create a planar face with an internal wire (internal edges).
        // This configuration previously caused an infinite loop in IntUnifyFaces
        // because edges removed from InternalEdges were not removed from IntVEmap.

        var aBuilder = new BRep_Builder();

        // Create a rectangular outer wire
        var aP1 = new gp_Pnt(0, 0, 0);
        var aP2 = new gp_Pnt(10, 0, 0);
        var aP3 = new gp_Pnt(10, 10, 0);
        var aP4 = new gp_Pnt(0, 10, 0);

        var aME1 = new BRepBuilderAPI_MakeEdge(aP1, aP2);
        var aME2 = new BRepBuilderAPI_MakeEdge(aP2, aP3);
        var aME3 = new BRepBuilderAPI_MakeEdge(aP3, aP4);
        var aME4 = new BRepBuilderAPI_MakeEdge(aP4, aP1);

        var aMW = new BRepBuilderAPI_MakeWire();
        aMW.Add(aME1.Edge());
        aMW.Add(aME2.Edge());
        aMW.Add(aME3.Edge());
        aMW.Add(aME4.Edge());

        Assert.True(aMW.IsDone(), "Failed to create outer wire");
        TopoDS_Wire anOuterWire = aMW.Wire();

        // Create a face from the outer wire
        var aPlane = new gp_Pln(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        var aMF    = new BRepBuilderAPI_MakeFace(aPlane, anOuterWire);
        Assert.True(aMF.IsDone(), "Failed to create face");

        TopoDS_Face aFace = aMF.InternalFace();

        // Create internal edges forming an open path inside the face.
        // These edges share a common vertex (aIP2) which is key to triggering
        // the infinite loop bug in IntVEmap processing.
        var aIP1 = new gp_Pnt(2, 2, 0);
        var aIP2 = new gp_Pnt(5, 5, 0);
        var aIP3 = new gp_Pnt(8, 2, 0);

        var aMIE1 = new BRepBuilderAPI_MakeEdge(aIP1, aIP2);
        var aMIE2 = new BRepBuilderAPI_MakeEdge(aIP2, aIP3);

        Assert.True(aMIE1.IsDone(), "Failed to create internal edge 1");
        Assert.True(aMIE2.IsDone(), "Failed to create internal edge 2");

        // Create an internal wire from the edges
        var aMIW = new BRepBuilderAPI_MakeWire();
        aMIW.Add(aMIE1.Edge());
        aMIW.Add(aMIE2.Edge());
        Assert.True(aMIW.IsDone(), "Failed to create internal wire");

        TopoDS_Wire anInternalWire = aMIW.Wire();

        // Set wire to INTERNAL orientation and add to face
        anInternalWire.Orientation(TopAbs_INTERNAL);
        aBuilder.Add(aFace, anInternalWire);

        // Create a shell with this face
        var aShell = new TopoDS_Shell();
        aBuilder.MakeShell(ref aShell);
        aBuilder.Add(aShell, aFace);

        // Run UnifySameDomain - before the fix, this would enter an infinite loop
        // because internal edges removed from InternalEdges were not removed from IntVEmap.
        var aUnifier = new ShapeUpgrade_UnifySameDomain(aShell);
        aUnifier.SetAngularTolerance(1e-6);
        aUnifier.SetLinearTolerance(1e-5);
        aUnifier.AllowInternalEdges(true);

        // This should complete without hanging
        aUnifier.Build();

        // Verify the result
        TopoDS_Shape aResult = aUnifier.Shape();
        Assert.False(aResult.IsNull(), "UnifySameDomain result should not be null");

        // Count faces in result - should have at least one face
        int aFaceCount = 0;
        for (var anExp = new TopExp_Explorer(aResult, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            aFaceCount++;
        }
        Assert.True(aFaceCount >= 1, "Result should contain at least one face");
    }

    // Test UnifySameDomain with multiple coplanar faces having internal wires.
    // This creates a more complex scenario where multiple faces need to be merged.
    [Fact]
    public void MultipleCoplanarFacesWithInternalWires()
    {
        var aBuilder = new BRep_Builder();

        // Create two adjacent coplanar rectangular faces
        var aPlane = new gp_Pln(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));

        // First face: 0-5 x 0-10
        var aP1 = new gp_Pnt(0, 0, 0);
        var aP2 = new gp_Pnt(5, 0, 0);
        var aP3 = new gp_Pnt(5, 10, 0);
        var aP4 = new gp_Pnt(0, 10, 0);

        var aMW1 = new BRepBuilderAPI_MakeWire();
        aMW1.Add(new BRepBuilderAPI_MakeEdge(aP1, aP2).Edge());
        aMW1.Add(new BRepBuilderAPI_MakeEdge(aP2, aP3).Edge());
        aMW1.Add(new BRepBuilderAPI_MakeEdge(aP3, aP4).Edge());
        aMW1.Add(new BRepBuilderAPI_MakeEdge(aP4, aP1).Edge());

        Assert.True(aMW1.IsDone(), "Failed to create wire 1");

        var aMF1 = new BRepBuilderAPI_MakeFace(aPlane, aMW1.Wire());
        Assert.True(aMF1.IsDone(), "Failed to create face 1");
        TopoDS_Face aFace1 = aMF1.InternalFace();

        // Second face: 5-10 x 0-10
        var aP5 = new gp_Pnt(5, 0, 0);
        var aP6 = new gp_Pnt(10, 0, 0);
        var aP7 = new gp_Pnt(10, 10, 0);
        var aP8 = new gp_Pnt(5, 10, 0);

        var aMW2 = new BRepBuilderAPI_MakeWire();
        aMW2.Add(new BRepBuilderAPI_MakeEdge(aP5, aP6).Edge());
        aMW2.Add(new BRepBuilderAPI_MakeEdge(aP6, aP7).Edge());
        aMW2.Add(new BRepBuilderAPI_MakeEdge(aP7, aP8).Edge());
        aMW2.Add(new BRepBuilderAPI_MakeEdge(aP8, aP5).Edge());

        Assert.True(aMW2.IsDone(), "Failed to create wire 2");

        var aMF2 = new BRepBuilderAPI_MakeFace(aPlane, aMW2.Wire());
        Assert.True(aMF2.IsDone(), "Failed to create face 2");
        TopoDS_Face aFace2 = aMF2.InternalFace();

        // Add internal wire to first face
        var aIP1  = new gp_Pnt(1, 5, 0);
        var aIP2  = new gp_Pnt(4, 5, 0);
        var aMIW1 = new BRepBuilderAPI_MakeWire();
        aMIW1.Add(new BRepBuilderAPI_MakeEdge(aIP1, aIP2).Edge());
        Assert.True(aMIW1.IsDone(), "Failed to create internal wire 1");
        TopoDS_Wire anInternalWire1 = aMIW1.Wire();
        anInternalWire1.Orientation(TopAbs_INTERNAL);
        aBuilder.Add(aFace1, anInternalWire1);

        // Add internal wire to second face
        var aIP3  = new gp_Pnt(6, 5, 0);
        var aIP4  = new gp_Pnt(9, 5, 0);
        var aMIW2 = new BRepBuilderAPI_MakeWire();
        aMIW2.Add(new BRepBuilderAPI_MakeEdge(aIP3, aIP4).Edge());
        Assert.True(aMIW2.IsDone(), "Failed to create internal wire 2");
        TopoDS_Wire anInternalWire2 = aMIW2.Wire();
        anInternalWire2.Orientation(TopAbs_INTERNAL);
        aBuilder.Add(aFace2, anInternalWire2);

        // Sew the faces together to share common edge
        var aSewer = new BRepBuilderAPI_Sewing(1e-6);
        aSewer.Add(aFace1);
        aSewer.Add(aFace2);
        aSewer.Perform();

        TopoDS_Shape aSewedShape = aSewer.SewedShape();
        Assert.False(aSewedShape.IsNull(), "Sewing failed");

        // Run UnifySameDomain
        var aUnifier = new ShapeUpgrade_UnifySameDomain(aSewedShape);
        aUnifier.SetAngularTolerance(1e-6);
        aUnifier.SetLinearTolerance(1e-5);
        aUnifier.AllowInternalEdges(true);

        aUnifier.Build();

        TopoDS_Shape aResult = aUnifier.Shape();
        Assert.False(aResult.IsNull(), "UnifySameDomain result should not be null");

        // After unification, we should have fewer faces (ideally 1 merged face)
        int aFaceCount = 0;
        for (var anExp = new TopExp_Explorer(aResult, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            aFaceCount++;
        }

        // The two coplanar faces should be merged into one
        Assert.Equal(1, aFaceCount);
    }

    // Test that UnifySameDomain handles closed internal wires correctly.
    [Fact]
    public void ClosedInternalWire()
    {
        var aBuilder = new BRep_Builder();

        // Create a large rectangular face
        var aP1 = new gp_Pnt(0, 0, 0);
        var aP2 = new gp_Pnt(20, 0, 0);
        var aP3 = new gp_Pnt(20, 20, 0);
        var aP4 = new gp_Pnt(0, 20, 0);

        var aMW = new BRepBuilderAPI_MakeWire();
        aMW.Add(new BRepBuilderAPI_MakeEdge(aP1, aP2).Edge());
        aMW.Add(new BRepBuilderAPI_MakeEdge(aP2, aP3).Edge());
        aMW.Add(new BRepBuilderAPI_MakeEdge(aP3, aP4).Edge());
        aMW.Add(new BRepBuilderAPI_MakeEdge(aP4, aP1).Edge());

        Assert.True(aMW.IsDone(), "Failed to create outer wire");

        var aPlane = new gp_Pln(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        var aMF    = new BRepBuilderAPI_MakeFace(aPlane, aMW.Wire());
        Assert.True(aMF.IsDone(), "Failed to create face");
        TopoDS_Face aFace = aMF.InternalFace();

        // Create a closed internal wire (triangle) inside the face
        var aIP1 = new gp_Pnt(5, 5, 0);
        var aIP2 = new gp_Pnt(15, 5, 0);
        var aIP3 = new gp_Pnt(10, 15, 0);

        var aMIW = new BRepBuilderAPI_MakeWire();
        aMIW.Add(new BRepBuilderAPI_MakeEdge(aIP1, aIP2).Edge());
        aMIW.Add(new BRepBuilderAPI_MakeEdge(aIP2, aIP3).Edge());
        aMIW.Add(new BRepBuilderAPI_MakeEdge(aIP3, aIP1).Edge());

        Assert.True(aMIW.IsDone(), "Failed to create internal wire");

        TopoDS_Wire anInternalWire = aMIW.Wire();
        anInternalWire.Orientation(TopAbs_INTERNAL);
        aBuilder.Add(aFace, anInternalWire);

        // Create shell
        var aShell = new TopoDS_Shell();
        aBuilder.MakeShell(ref aShell);
        aBuilder.Add(aShell, aFace);

        // Run UnifySameDomain
        var aUnifier = new ShapeUpgrade_UnifySameDomain(aShell);
        aUnifier.SetAngularTolerance(1e-6);
        aUnifier.SetLinearTolerance(1e-5);
        aUnifier.AllowInternalEdges(true);

        aUnifier.Build();

        TopoDS_Shape aResult = aUnifier.Shape();
        Assert.False(aResult.IsNull(), "UnifySameDomain result should not be null");
    }

    // Test basic UnifySameDomain functionality on a box (no internal edges).
    // This serves as a baseline test to ensure the algorithm works correctly.
    [Fact]
    public void BasicBoxUnification()
    {
        // Create a simple box using two corner points
        var aBoxMaker = new BRepPrimAPI_MakeBox(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 10, 10));
        aBoxMaker.Build();
        Assert.True(aBoxMaker.IsDone(), "Failed to create box");

        TopoDS_Shape aBox = aBoxMaker.Shape();

        // Run UnifySameDomain
        var aUnifier = new ShapeUpgrade_UnifySameDomain(aBox);
        aUnifier.Build();

        TopoDS_Shape aResult = aUnifier.Shape();
        Assert.False(aResult.IsNull(), "UnifySameDomain result should not be null");

        // A box should still have 6 faces after unification (no coplanar faces)
        int aFaceCount = 0;
        for (var anExp = new TopExp_Explorer(aResult, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            aFaceCount++;
        }
        Assert.Equal(6, aFaceCount);
    }

    // Test that a missing pcurve neither causes a null dereference nor aborts unrelated face merging.
    [Fact]
    public void MissingPCurveDoesNotAbortShellProcessing()
    {
        var                aBuilder   = new BRep_Builder();
        MissingPCurveShape aTestShape = makeMissingPCurveShape(aBuilder);
        Assert.False(aTestShape.Shell.IsNull());
        Assert.True(addPlanarFacePair(aBuilder, aTestShape.Shell));

        Assert.Null(BRep_Tool.CurveOnSurface(aTestShape.MissingPCurveEdge, aTestShape.ReferenceFace, out _, out _));
        Assert.NotNull(BRep_Tool.CurveOnSurface(aTestShape.ValidPCurveEdge, aTestShape.ReferenceFace, out _, out _));

        var aUnifier = new ShapeUpgrade_UnifySameDomain(aTestShape.Shell, false, true, false);
        aUnifier.AllowInternalEdges(true);
        aUnifier.Build();

        TopoDS_Shape aResult = aUnifier.Shape();
        Assert.False(aResult.IsNull());

        Assert.Equal(3, countFaces(aResult));
    }

    // Test that pcurve transformation skips an edge which has no source pcurve.
    [Fact]
    public void MissingSourcePCurveDuringTransformation()
    {
        var                aBuilder   = new BRep_Builder();
        MissingPCurveShape aTestShape = makeMissingPCurveTransformationShape(aBuilder);
        Assert.False(aTestShape.Shell.IsNull());

        Assert.Null(BRep_Tool.CurveOnSurface(aTestShape.MissingPCurveEdge, aTestShape.ReferenceFace, out _, out _));

        var aUnifier = new ShapeUpgrade_UnifySameDomain(aTestShape.Shell, false, true, false);
        aUnifier.AllowInternalEdges(true);
        aUnifier.Build();

        TopoDS_Shape aResult = aUnifier.Shape();
        Assert.False(aResult.IsNull());
        Assert.Equal(2, countFaces(aResult));
    }
}
