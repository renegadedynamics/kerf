// Port of: src/ModelingAlgorithms/TKShHealing/GTests/ShapeUpgrade_UnifySameDomain_Test.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Geometry;
using Kerf.Engine.Healing;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// OCCT's GTests of ShapeUpgrade_UnifySameDomain. The shapes are built with the makers' internal (unfrozen) results
/// where OCCT's test adds wires to a made face or pcurves to a made edge.
/// </summary>
public class ShapeUpgrade_UnifySameDomainTest
{
    private struct MissingPCurveShape
    {
        public Shell Shell;
        public Face  ReferenceFace;
        public Edge  MissingPCurveEdge;
        public Edge  ValidPCurveEdge;
    }

    private static Vertex makeVertex(BrepBuilder theBuilder, in Point3D thePoint)
    {
        var aVertex = new Vertex();
        theBuilder.MakeVertex(ref aVertex, thePoint, global::Kerf.Engine.Foundation.Precision.Confusion());
        return aVertex;
    }

    private static Edge makeEdge(BrepBuilder    theBuilder,
                                        Surface?   theSurface,
                                        in Placement theLocation,
                                        in Vertex   theFirst,
                                        in Vertex   theLast,
                                        bool            theAddPCurve)
    {
        var anEdgeMaker = new MakeEdge(theFirst, theLast);
        if (!anEdgeMaker.IsDone)
        {
            return new Edge();
        }

        Edge anEdge = anEdgeMaker.InternalEdge();
        if (theAddPCurve)
        {
            Point3D        aFirstPoint   = Tool.Pnt(theFirst);
            Point3D        aLastPoint    = Tool.Pnt(theLast);
            var           aFirstPoint2d = new Point2D(Math.Atan2(aFirstPoint.Y, aFirstPoint.X), aFirstPoint.Z);
            var           aLastPoint2d  = new Point2D(Math.Atan2(aLastPoint.Y, aLastPoint.X), aLastPoint.Z);
            var           aDirection    = new Vector2D(aFirstPoint2d, aLastPoint2d);
            Curve2D  aPCurve       = new LineCurve2D(aFirstPoint2d, new Direction2D(aDirection));
            theBuilder.UpdateEdge(anEdge, aPCurve, theSurface, theLocation, global::Kerf.Engine.Foundation.Precision.Confusion());
            theBuilder.Range(anEdge, 0.0, aDirection.Magnitude);
        }
        return anEdge;
    }

    private static MissingPCurveShape makeMissingPCurveShape(BrepBuilder theBuilder)
    {
        var           aSurface  = new CylindricalSurface(new CoordinateSystem(Constants.XOY()), 1.0);
        var           aLocation = new Placement();
        Vertex aVertexA  = makeVertex(theBuilder, aSurface.Value(0.0, 0.0));
        Vertex aVertexB  = makeVertex(theBuilder, aSurface.Value(1.0, 0.0));
        Vertex aVertexC  = makeVertex(theBuilder, aSurface.Value(1.0, 1.0));
        Vertex aVertexD  = makeVertex(theBuilder, aSurface.Value(0.5, -1.0));
        Vertex aVertexE  = makeVertex(theBuilder, aSurface.Value(0.5, 1.0));

        Edge aCommonEdge = makeEdge(theBuilder, aSurface, aLocation, aVertexA, aVertexB, true);
        Edge anEdgeBC    = makeEdge(theBuilder, aSurface, aLocation, aVertexB, aVertexC, false);
        Edge anEdgeCB    = makeEdge(theBuilder, aSurface, aLocation, aVertexC, aVertexB, true);
        Edge anEdgeBD    = makeEdge(theBuilder, aSurface, aLocation, aVertexB, aVertexD, true);
        Edge anEdgeDA    = makeEdge(theBuilder, aSurface, aLocation, aVertexD, aVertexA, true);
        Edge anEdgeAE    = makeEdge(theBuilder, aSurface, aLocation, aVertexA, aVertexE, true);
        Edge anEdgeEB    = makeEdge(theBuilder, aSurface, aLocation, aVertexE, aVertexB, true);

        var aSelfTouchingWire = new Wire();
        theBuilder.MakeWire(ref aSelfTouchingWire);
        theBuilder.Add(aSelfTouchingWire, aCommonEdge);
        theBuilder.Add(aSelfTouchingWire, anEdgeBC);
        theBuilder.Add(aSelfTouchingWire, anEdgeCB);
        theBuilder.Add(aSelfTouchingWire, anEdgeBD);
        theBuilder.Add(aSelfTouchingWire, anEdgeDA);
        aSelfTouchingWire.Closed(true);

        var aSelfTouchingFace = new Face();
        theBuilder.MakeFace(ref aSelfTouchingFace, aSurface, aLocation, global::Kerf.Engine.Foundation.Precision.Confusion());
        theBuilder.Add(aSelfTouchingFace, aSelfTouchingWire);

        Edge aReversedCommonEdge = aCommonEdge;
        aReversedCommonEdge.Reverse();
        var aSecondWire = new Wire();
        theBuilder.MakeWire(ref aSecondWire);
        theBuilder.Add(aSecondWire, aReversedCommonEdge);
        theBuilder.Add(aSecondWire, anEdgeAE);
        theBuilder.Add(aSecondWire, anEdgeEB);
        aSecondWire.Closed(true);

        var aSecondFace = new Face();
        theBuilder.MakeFace(ref aSecondFace, aSurface, aLocation, global::Kerf.Engine.Foundation.Precision.Confusion());
        theBuilder.Add(aSecondFace, aSecondWire);

        var aResult = new MissingPCurveShape { Shell = new Shell() };
        theBuilder.MakeShell(ref aResult.Shell);
        theBuilder.Add(aResult.Shell, aSelfTouchingFace);
        theBuilder.Add(aResult.Shell, aSecondFace);
        aResult.ReferenceFace     = aSelfTouchingFace;
        aResult.MissingPCurveEdge = anEdgeBC;
        aResult.ValidPCurveEdge   = anEdgeBD;
        return aResult;
    }

    private static MissingPCurveShape makeMissingPCurveTransformationShape(BrepBuilder theBuilder)
    {
        var           aSurface  = new CylindricalSurface(new CoordinateSystem(Constants.XOY()), 1.0);
        var           aLocation = new Placement();
        Vertex aVertexA  = makeVertex(theBuilder, aSurface.Value(0.0, 0.0));
        Vertex aVertexB  = makeVertex(theBuilder, aSurface.Value(0.5, 0.0));
        Vertex aVertexC  = makeVertex(theBuilder, aSurface.Value(0.5, 1.0));
        Vertex aVertexD  = makeVertex(theBuilder, aSurface.Value(0.0, 1.0));
        Vertex aVertexE  = makeVertex(theBuilder, aSurface.Value(1.0, 0.0));
        Vertex aVertexF  = makeVertex(theBuilder, aSurface.Value(1.0, 1.0));

        Edge anEdgeAB = makeEdge(theBuilder, aSurface, aLocation, aVertexA, aVertexB, true);
        Edge anEdgeBC = makeEdge(theBuilder, aSurface, aLocation, aVertexB, aVertexC, true);
        Edge anEdgeCD = makeEdge(theBuilder, aSurface, aLocation, aVertexC, aVertexD, true);
        Edge anEdgeDA = makeEdge(theBuilder, aSurface, aLocation, aVertexD, aVertexA, true);
        Edge anEdgeBE = makeEdge(theBuilder, aSurface, aLocation, aVertexB, aVertexE, false);
        Edge anEdgeEF = makeEdge(theBuilder, aSurface, aLocation, aVertexE, aVertexF, true);
        Edge anEdgeFC = makeEdge(theBuilder, aSurface, aLocation, aVertexF, aVertexC, true);

        var aLeftWire = new Wire();
        theBuilder.MakeWire(ref aLeftWire);
        theBuilder.Add(aLeftWire, anEdgeAB);
        theBuilder.Add(aLeftWire, anEdgeBC);
        theBuilder.Add(aLeftWire, anEdgeCD);
        theBuilder.Add(aLeftWire, anEdgeDA);
        aLeftWire.Closed(true);

        Edge aReversedEdgeBC = anEdgeBC;
        aReversedEdgeBC.Reverse();
        var aRightWire = new Wire();
        theBuilder.MakeWire(ref aRightWire);
        theBuilder.Add(aRightWire, anEdgeBE);
        theBuilder.Add(aRightWire, anEdgeEF);
        theBuilder.Add(aRightWire, anEdgeFC);
        theBuilder.Add(aRightWire, aReversedEdgeBC);
        aRightWire.Closed(true);

        var aResult = new MissingPCurveShape { Shell = new Shell(), ReferenceFace = new Face() };
        theBuilder.MakeFace(ref aResult.ReferenceFace, aSurface, aLocation, global::Kerf.Engine.Foundation.Precision.Confusion());
        theBuilder.Add(aResult.ReferenceFace, aLeftWire);
        var aRightFace = new Face();
        theBuilder.MakeFace(ref aRightFace, aSurface, aLocation, global::Kerf.Engine.Foundation.Precision.Confusion());
        theBuilder.Add(aRightFace, aRightWire);
        theBuilder.MakeShell(ref aResult.Shell);
        theBuilder.Add(aResult.Shell, aResult.ReferenceFace);
        theBuilder.Add(aResult.Shell, aRightFace);
        aResult.MissingPCurveEdge = anEdgeBE;
        aResult.ValidPCurveEdge   = anEdgeBC;
        return aResult;
    }

    private static bool addPlanarFacePair(BrepBuilder theBuilder, in Shell theShell)
    {
        Surface? aNullSurface = null;
        var           aLocation    = new Placement();
        Vertex aVertex1     = makeVertex(theBuilder, new Point3D(3.0, 0.0, 0.0));
        Vertex aVertex2     = makeVertex(theBuilder, new Point3D(4.0, 0.0, 0.0));
        Vertex aVertex3     = makeVertex(theBuilder, new Point3D(4.0, 1.0, 0.0));
        Vertex aVertex4     = makeVertex(theBuilder, new Point3D(3.0, 1.0, 0.0));
        Vertex aVertex5     = makeVertex(theBuilder, new Point3D(5.0, 0.0, 0.0));
        Vertex aVertex6     = makeVertex(theBuilder, new Point3D(5.0, 1.0, 0.0));

        Edge anEdge12 = makeEdge(theBuilder, aNullSurface, aLocation, aVertex1, aVertex2, false);
        Edge anEdge23 = makeEdge(theBuilder, aNullSurface, aLocation, aVertex2, aVertex3, false);
        Edge anEdge34 = makeEdge(theBuilder, aNullSurface, aLocation, aVertex3, aVertex4, false);
        Edge anEdge41 = makeEdge(theBuilder, aNullSurface, aLocation, aVertex4, aVertex1, false);
        Edge anEdge25 = makeEdge(theBuilder, aNullSurface, aLocation, aVertex2, aVertex5, false);
        Edge anEdge56 = makeEdge(theBuilder, aNullSurface, aLocation, aVertex5, aVertex6, false);
        Edge anEdge63 = makeEdge(theBuilder, aNullSurface, aLocation, aVertex6, aVertex3, false);
        if (anEdge12.IsNull || anEdge23.IsNull || anEdge34.IsNull || anEdge41.IsNull || anEdge25.IsNull
            || anEdge56.IsNull || anEdge63.IsNull)
        {
            return false;
        }

        var aLeftWireMaker = new MakeWire();
        aLeftWireMaker.Add(anEdge12);
        aLeftWireMaker.Add(anEdge23);
        aLeftWireMaker.Add(anEdge34);
        aLeftWireMaker.Add(anEdge41);
        if (!aLeftWireMaker.IsDone)
        {
            return false;
        }

        Edge aReversedEdge23 = anEdge23;
        aReversedEdge23.Reverse();
        var aRightWireMaker = new MakeWire();
        aRightWireMaker.Add(aReversedEdge23);
        aRightWireMaker.Add(anEdge25);
        aRightWireMaker.Add(anEdge56);
        aRightWireMaker.Add(anEdge63);
        if (!aRightWireMaker.IsDone)
        {
            return false;
        }

        var aPlanarSurface  = new Plane3D(new CoordinateSystem(Constants.XOY()));
        var aLeftFaceMaker  = new MakeFace(aPlanarSurface, aLeftWireMaker.InternalWire());
        var aRightFaceMaker = new MakeFace(aPlanarSurface, aRightWireMaker.InternalWire());
        if (!aLeftFaceMaker.IsDone || !aRightFaceMaker.IsDone)
        {
            return false;
        }
        theBuilder.Add(theShell, aLeftFaceMaker.InternalFace());
        theBuilder.Add(theShell, aRightFaceMaker.InternalFace());
        return true;
    }

    private static int countFaces(in Shape theShape)
    {
        int aFaceCount = 0;
        for (var anExp = new Explorer(theShape, ShapeType.Face); anExp.More(); anExp.Next())
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

        var aBuilder = new BrepBuilder();

        // Create a rectangular outer wire
        var aP1 = new Point3D(0, 0, 0);
        var aP2 = new Point3D(10, 0, 0);
        var aP3 = new Point3D(10, 10, 0);
        var aP4 = new Point3D(0, 10, 0);

        var aME1 = new MakeEdge(aP1, aP2);
        var aME2 = new MakeEdge(aP2, aP3);
        var aME3 = new MakeEdge(aP3, aP4);
        var aME4 = new MakeEdge(aP4, aP1);

        var aMW = new MakeWire();
        aMW.Add(aME1.Edge());
        aMW.Add(aME2.Edge());
        aMW.Add(aME3.Edge());
        aMW.Add(aME4.Edge());

        Assert.True(aMW.IsDone, "Failed to create outer wire");
        Wire anOuterWire = aMW.Wire();

        // Create a face from the outer wire
        var aPlane = new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        var aMF    = new MakeFace(aPlane, anOuterWire);
        Assert.True(aMF.IsDone, "Failed to create face");

        Face aFace = aMF.InternalFace();

        // Create internal edges forming an open path inside the face.
        // These edges share a common vertex (aIP2) which is key to triggering
        // the infinite loop bug in IntVEmap processing.
        var aIP1 = new Point3D(2, 2, 0);
        var aIP2 = new Point3D(5, 5, 0);
        var aIP3 = new Point3D(8, 2, 0);

        var aMIE1 = new MakeEdge(aIP1, aIP2);
        var aMIE2 = new MakeEdge(aIP2, aIP3);

        Assert.True(aMIE1.IsDone, "Failed to create internal edge 1");
        Assert.True(aMIE2.IsDone, "Failed to create internal edge 2");

        // Create an internal wire from the edges
        var aMIW = new MakeWire();
        aMIW.Add(aMIE1.Edge());
        aMIW.Add(aMIE2.Edge());
        Assert.True(aMIW.IsDone, "Failed to create internal wire");

        Wire anInternalWire = aMIW.Wire();

        // Set wire to INTERNAL orientation and add to face
        anInternalWire.Orient(Internal);
        aBuilder.Add(aFace, anInternalWire);

        // Create a shell with this face
        var aShell = new Shell();
        aBuilder.MakeShell(ref aShell);
        aBuilder.Add(aShell, aFace);

        // Run UnifySameDomain - before the fix, this would enter an infinite loop
        // because internal edges removed from InternalEdges were not removed from IntVEmap.
        var aUnifier = new UnifySameDomain(aShell);
        aUnifier.SetAngularTolerance(1e-6);
        aUnifier.SetLinearTolerance(1e-5);
        aUnifier.AllowInternalEdges(true);

        // This should complete without hanging
        aUnifier.Build();

        // Verify the result
        Shape aResult = aUnifier.Shape();
        Assert.False(aResult.IsNull, "UnifySameDomain result should not be null");

        // Count faces in result - should have at least one face
        int aFaceCount = 0;
        for (var anExp = new Explorer(aResult, ShapeType.Face); anExp.More(); anExp.Next())
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
        var aBuilder = new BrepBuilder();

        // Create two adjacent coplanar rectangular faces
        var aPlane = new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));

        // First face: 0-5 x 0-10
        var aP1 = new Point3D(0, 0, 0);
        var aP2 = new Point3D(5, 0, 0);
        var aP3 = new Point3D(5, 10, 0);
        var aP4 = new Point3D(0, 10, 0);

        var aMW1 = new MakeWire();
        aMW1.Add(new MakeEdge(aP1, aP2).Edge());
        aMW1.Add(new MakeEdge(aP2, aP3).Edge());
        aMW1.Add(new MakeEdge(aP3, aP4).Edge());
        aMW1.Add(new MakeEdge(aP4, aP1).Edge());

        Assert.True(aMW1.IsDone, "Failed to create wire 1");

        var aMF1 = new MakeFace(aPlane, aMW1.Wire());
        Assert.True(aMF1.IsDone, "Failed to create face 1");
        Face aFace1 = aMF1.InternalFace();

        // Second face: 5-10 x 0-10
        var aP5 = new Point3D(5, 0, 0);
        var aP6 = new Point3D(10, 0, 0);
        var aP7 = new Point3D(10, 10, 0);
        var aP8 = new Point3D(5, 10, 0);

        var aMW2 = new MakeWire();
        aMW2.Add(new MakeEdge(aP5, aP6).Edge());
        aMW2.Add(new MakeEdge(aP6, aP7).Edge());
        aMW2.Add(new MakeEdge(aP7, aP8).Edge());
        aMW2.Add(new MakeEdge(aP8, aP5).Edge());

        Assert.True(aMW2.IsDone, "Failed to create wire 2");

        var aMF2 = new MakeFace(aPlane, aMW2.Wire());
        Assert.True(aMF2.IsDone, "Failed to create face 2");
        Face aFace2 = aMF2.InternalFace();

        // Add internal wire to first face
        var aIP1  = new Point3D(1, 5, 0);
        var aIP2  = new Point3D(4, 5, 0);
        var aMIW1 = new MakeWire();
        aMIW1.Add(new MakeEdge(aIP1, aIP2).Edge());
        Assert.True(aMIW1.IsDone, "Failed to create internal wire 1");
        Wire anInternalWire1 = aMIW1.Wire();
        anInternalWire1.Orient(Internal);
        aBuilder.Add(aFace1, anInternalWire1);

        // Add internal wire to second face
        var aIP3  = new Point3D(6, 5, 0);
        var aIP4  = new Point3D(9, 5, 0);
        var aMIW2 = new MakeWire();
        aMIW2.Add(new MakeEdge(aIP3, aIP4).Edge());
        Assert.True(aMIW2.IsDone, "Failed to create internal wire 2");
        Wire anInternalWire2 = aMIW2.Wire();
        anInternalWire2.Orient(Internal);
        aBuilder.Add(aFace2, anInternalWire2);

        // Sew the faces together to share common edge
        var aSewer = new Sewing(1e-6);
        aSewer.Add(aFace1);
        aSewer.Add(aFace2);
        aSewer.Perform();

        Shape aSewedShape = aSewer.SewedShape();
        Assert.False(aSewedShape.IsNull, "Sewing failed");

        // Run UnifySameDomain
        var aUnifier = new UnifySameDomain(aSewedShape);
        aUnifier.SetAngularTolerance(1e-6);
        aUnifier.SetLinearTolerance(1e-5);
        aUnifier.AllowInternalEdges(true);

        aUnifier.Build();

        Shape aResult = aUnifier.Shape();
        Assert.False(aResult.IsNull, "UnifySameDomain result should not be null");

        // After unification, we should have fewer faces (ideally 1 merged face)
        int aFaceCount = 0;
        for (var anExp = new Explorer(aResult, ShapeType.Face); anExp.More(); anExp.Next())
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
        var aBuilder = new BrepBuilder();

        // Create a large rectangular face
        var aP1 = new Point3D(0, 0, 0);
        var aP2 = new Point3D(20, 0, 0);
        var aP3 = new Point3D(20, 20, 0);
        var aP4 = new Point3D(0, 20, 0);

        var aMW = new MakeWire();
        aMW.Add(new MakeEdge(aP1, aP2).Edge());
        aMW.Add(new MakeEdge(aP2, aP3).Edge());
        aMW.Add(new MakeEdge(aP3, aP4).Edge());
        aMW.Add(new MakeEdge(aP4, aP1).Edge());

        Assert.True(aMW.IsDone, "Failed to create outer wire");

        var aPlane = new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        var aMF    = new MakeFace(aPlane, aMW.Wire());
        Assert.True(aMF.IsDone, "Failed to create face");
        Face aFace = aMF.InternalFace();

        // Create a closed internal wire (triangle) inside the face
        var aIP1 = new Point3D(5, 5, 0);
        var aIP2 = new Point3D(15, 5, 0);
        var aIP3 = new Point3D(10, 15, 0);

        var aMIW = new MakeWire();
        aMIW.Add(new MakeEdge(aIP1, aIP2).Edge());
        aMIW.Add(new MakeEdge(aIP2, aIP3).Edge());
        aMIW.Add(new MakeEdge(aIP3, aIP1).Edge());

        Assert.True(aMIW.IsDone, "Failed to create internal wire");

        Wire anInternalWire = aMIW.Wire();
        anInternalWire.Orient(Internal);
        aBuilder.Add(aFace, anInternalWire);

        // Create shell
        var aShell = new Shell();
        aBuilder.MakeShell(ref aShell);
        aBuilder.Add(aShell, aFace);

        // Run UnifySameDomain
        var aUnifier = new UnifySameDomain(aShell);
        aUnifier.SetAngularTolerance(1e-6);
        aUnifier.SetLinearTolerance(1e-5);
        aUnifier.AllowInternalEdges(true);

        aUnifier.Build();

        Shape aResult = aUnifier.Shape();
        Assert.False(aResult.IsNull, "UnifySameDomain result should not be null");
    }

    // Test basic UnifySameDomain functionality on a box (no internal edges).
    // This serves as a baseline test to ensure the algorithm works correctly.
    [Fact]
    public void BasicBoxUnification()
    {
        // Create a simple box using two corner points
        var aBoxMaker = new MakeBox(new Point3D(0, 0, 0), new Point3D(10, 10, 10));
        aBoxMaker.Build();
        Assert.True(aBoxMaker.IsDone, "Failed to create box");

        Shape aBox = aBoxMaker.Shape();

        // Run UnifySameDomain
        var aUnifier = new UnifySameDomain(aBox);
        aUnifier.Build();

        Shape aResult = aUnifier.Shape();
        Assert.False(aResult.IsNull, "UnifySameDomain result should not be null");

        // A box should still have 6 faces after unification (no coplanar faces)
        int aFaceCount = 0;
        for (var anExp = new Explorer(aResult, ShapeType.Face); anExp.More(); anExp.Next())
        {
            aFaceCount++;
        }
        Assert.Equal(6, aFaceCount);
    }

    // Test that a missing pcurve neither causes a null dereference nor aborts unrelated face merging.
    [Fact]
    public void MissingPCurveDoesNotAbortShellProcessing()
    {
        var                aBuilder   = new BrepBuilder();
        MissingPCurveShape aTestShape = makeMissingPCurveShape(aBuilder);
        Assert.False(aTestShape.Shell.IsNull);
        Assert.True(addPlanarFacePair(aBuilder, aTestShape.Shell));

        Assert.Null(Tool.CurveOnSurface(aTestShape.MissingPCurveEdge, aTestShape.ReferenceFace, out _, out _));
        Assert.NotNull(Tool.CurveOnSurface(aTestShape.ValidPCurveEdge, aTestShape.ReferenceFace, out _, out _));

        var aUnifier = new UnifySameDomain(aTestShape.Shell, false, true, false);
        aUnifier.AllowInternalEdges(true);
        aUnifier.Build();

        Shape aResult = aUnifier.Shape();
        Assert.False(aResult.IsNull);

        Assert.Equal(3, countFaces(aResult));
    }

    // Test that pcurve transformation skips an edge which has no source pcurve.
    [Fact]
    public void MissingSourcePCurveDuringTransformation()
    {
        var                aBuilder   = new BrepBuilder();
        MissingPCurveShape aTestShape = makeMissingPCurveTransformationShape(aBuilder);
        Assert.False(aTestShape.Shell.IsNull);

        Assert.Null(Tool.CurveOnSurface(aTestShape.MissingPCurveEdge, aTestShape.ReferenceFace, out _, out _));

        var aUnifier = new UnifySameDomain(aTestShape.Shell, false, true, false);
        aUnifier.AllowInternalEdges(true);
        aUnifier.Build();

        Shape aResult = aUnifier.Shape();
        Assert.False(aResult.IsNull);
        Assert.Equal(2, countFaces(aResult));
    }
}
