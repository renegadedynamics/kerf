// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// Port of: src/ModelingAlgorithms/TKFillet/GTests/BRepFilletAPI_MakeFillet_Test.cxx (OCCT V8_0_1)

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Fillets;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Healing.Fixes;
using Kerf.Engine.Healing.Upgrade;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Engine.Fillets.FilletShape;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>BRepFilletAPI_MakeFillet (unit 7N3): OCCT's GTest.</summary>
public class BRepFilletAPI_MakeFilletTests
{
    /// <summary>
    /// The tests whose result the reconstruction gets wrong: none since unit 7R3 fixed the seam band (decision 1155),
    /// which OCC1077 and OCC426 met.
    /// </summary>
    private static readonly HashSet<string> KnownReconstructionGaps = [];

    private static double Area(in Shape S)
    {
        Properties aProps = new();
        MassPropertiesBuilder.SurfaceProperties(S, aProps);
        return aProps.Mass();
    }

    [Fact]
    public void FilletOneEdge()
    {
        MakeBox aBoxMaker = new(20.0, 20.0, 20.0);
        Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone);

        Explorer anExp = new(aBox, ShapeType.Edge);
        Assert.True(anExp.More());
        Edge anEdge = ShapeCasts.Edge(anExp.Current());

        MakeFillet aFillet = new(aBox);
        aFillet.Add(2.0, anEdge);
        Shape aResult = aFillet.Shape();
        Assert.True(aFillet.IsDone);

        Assert.True(new Analyzer(aResult).IsValid());
    }

    [Fact]
    public void FilletAllEdges()
    {
        MakeBox aBoxMaker = new(20.0, 20.0, 20.0);
        Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone);

        MakeFillet aFillet = new(aBox);
        for (Explorer anExp = new(aBox, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            aFillet.Add(1.0, ShapeCasts.Edge(anExp.Current()));
        }
        Shape aResult = aFillet.Shape();
        Assert.True(aFillet.IsDone);

        Assert.True(new Analyzer(aResult).IsValid());
    }

    [Fact]
    public void FilletMoreFaces()
    {
        MakeBox aBoxMaker = new(20.0, 20.0, 20.0);
        Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone);

        MakeFillet aFillet = new(aBox);
        for (Explorer anExp = new(aBox, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            aFillet.Add(1.0, ShapeCasts.Edge(anExp.Current()));
        }
        Shape aResult = aFillet.Shape();
        Assert.True(aFillet.IsDone);

        int aFaceCount = 0;
        for (Explorer aFaceExp = new(aResult, ShapeType.Face); aFaceExp.More(); aFaceExp.Next())
        {
            aFaceCount++;
        }
        Assert.True(aFaceCount > 6);
    }

    [Fact]
    public void FilletVariableRadius()
    {
        MakeBox aBoxMaker = new(20.0, 20.0, 20.0);
        Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone);

        Explorer anExp = new(aBox, ShapeType.Edge);
        Assert.True(anExp.More());
        Edge anEdge = ShapeCasts.Edge(anExp.Current());

        MakeFillet aFillet = new(aBox);
        aFillet.Add(1.0, 3.0, anEdge);
        Shape aResult = aFillet.Shape();
        Assert.True(aFillet.IsDone);

        Assert.True(new Analyzer(aResult).IsValid());
    }

    // Test OCC570: BRepFilletAPI_MakeFillet with mixed constant and variable radius.
    // Migrated from QABugs_17.cxx OCC570
    [Fact]
    public void OCC570_MixedVariableConstantRadius()
    {
        MakeBox aBoxMaker = new(100.0, 100.0, 100.0);
        Shape aBox = aBoxMaker.Shape();

        // Take the first wire of the box and its 4 edges
        Explorer aWireExp = new(aBox, ShapeType.Wire);
        Assert.True(aWireExp.More());

        Explorer anEdgeExp = new(aWireExp.Current(), ShapeType.Edge);
        Edge anE1 = ShapeCasts.Edge(anEdgeExp.Current());
        anEdgeExp.Next();
        Edge anE2 = ShapeCasts.Edge(anEdgeExp.Current());
        anEdgeExp.Next();
        Edge anE3 = ShapeCasts.Edge(anEdgeExp.Current());
        anEdgeExp.Next();
        Edge anE4 = ShapeCasts.Edge(anEdgeExp.Current());

        // Variable radius law: 4 (parameter, radius) control points
        NCollection_Array1<Point2D> aVarRadius = new(1, 4);
        aVarRadius.SetValue(1, new Point2D(0.0, 5.0));
        aVarRadius.SetValue(2, new Point2D(0.3, 15.0));
        aVarRadius.SetValue(3, new Point2D(0.7, 15.0));
        aVarRadius.SetValue(4, new Point2D(1.0, 5.0));

        MakeFillet aFillet = new(aBox);
        aFillet.SetContinuity(C1, 0.001);
        aFillet.Add(aVarRadius, anE1);
        aFillet.Add(5.0, anE2);
        aFillet.Add(aVarRadius, anE3);
        aFillet.Add(5.0, anE4);

        aFillet.Build();
        Assert.True(aFillet.IsDone, "Fillet operation should succeed");

        Shape aResult = aFillet.Shape();
        Assert.True(new Analyzer(aResult).IsValid(), "Result shape should be valid");
        Assert.Equal(58500.0, Area(aResult), 58500.0 * 0.01);
    }

    // OCC828: BRepFilletAPI_MakeFillet succeeds on a prism built from a complex wire of arcs and line segments
    // (trefoil-like profile).
    [Fact]
    public void Bug828_FilletOnComplexPrism()
    {
        Point3D aPnt11 = new(-27.598139, -7.0408573, 0.0);
        Point3D aPnt12 = new(-28.483755, -17.487625, 0.0);
        Point3D aPnt13 = new(-19.555504, -22.983587, 0.0);
        MakeArcOfCircle anArc1 = new(aPnt11, aPnt12, aPnt13);

        Point3D aPnt21 = new(12.125083, -22.983587, 0.0);
        Point3D aPnt22 = new(21.1572, -17.27554, 0.0);
        Point3D aPnt23 = new(19.878168, -6.6677585, 0.0);
        MakeArcOfCircle anArc2 = new(aPnt21, aPnt22, aPnt23);

        Point3D aPnt31 = new(3.265825, 13.724955, 0.0);
        Point3D aPnt32 = new(-4.7233953, 17.406338, 0.0);
        Point3D aPnt33 = new(-12.529893, 13.351856, 0.0);
        MakeArcOfCircle anArc3 = new(aPnt31, aPnt32, aPnt33);

        MakeSegment aSegment1 = new(aPnt13, aPnt21);
        MakeSegment aSegment2 = new(aPnt23, aPnt31);
        MakeSegment aSegment3 = new(aPnt33, aPnt11);

        MakeWire aWireMaker = new();
        aWireMaker.Add(new MakeEdge(anArc1.Value()).Edge());
        aWireMaker.Add(new MakeEdge(aSegment1.Value()).Edge());
        aWireMaker.Add(new MakeEdge(anArc2.Value()).Edge());
        aWireMaker.Add(new MakeEdge(aSegment2.Value()).Edge());
        aWireMaker.Add(new MakeEdge(anArc3.Value()).Edge());
        aWireMaker.Add(new MakeEdge(aSegment3.Value()).Edge());
        Assert.True(aWireMaker.IsDone);

        Wire aWire = aWireMaker.Wire();
        Face aFace = new MakeFace(aWire).Face();
        Assert.False(aFace.IsNull);

        const double aSlabThick = 111.0;
        ApiMakePrism aSlab = new(aFace, new Vector3D(0, 0, aSlabThick), true);
        Assert.True(aSlab.IsDone);
        Shape aSlabShape = aSlab.Shape();
        Assert.True(new Analyzer(aSlabShape).IsValid(), "Prism shape should be valid");

        List<Edge> anEdges = [];
        for (Explorer anExp = new(aSlabShape, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            anEdges.Add(ShapeCasts.Edge(anExp.Current()));
        }
        Assert.True(anEdges.Count >= 2, "Expected at least 2 edges in the prism");

        const double aTargetArea = 17816.2;
        const double aTargetTolerance = aTargetArea * 0.002;
        Shape aResult = new();
        bool hasMatchingPair = false;
        double aBestArea = 0.0, aBestDelta = 1.0e100;
        int aBestEdgeIndex1 = 0, aBestEdgeIndex2 = 0;

        // Tcl 'explode' numbering is not guaranteed to match TopExp_Explorer order here,
        // so select the edge pair that reproduces the original regression result.
        for (int anEdgeIndex1 = 1; anEdgeIndex1 <= anEdges.Count && !hasMatchingPair; ++anEdgeIndex1)
        {
            for (int anEdgeIndex2 = anEdgeIndex1 + 1; anEdgeIndex2 <= anEdges.Count; ++anEdgeIndex2)
            {
                try
                {
                    MakeFillet aFillet = new(aSlabShape, FilletShape.Rational);
                    aFillet.SetParams(1.0e-2, 1.0e-4, 1.0e-5, 1.0e-4, 1.0e-5, 1.0e-3);
                    aFillet.SetContinuity(C1, 1.0e-2);
                    aFillet.Add(10.0, anEdges[anEdgeIndex1 - 1]);
                    aFillet.Add(10.0, anEdges[anEdgeIndex2 - 1]);
                    aFillet.Build();
                    if (!aFillet.IsDone)
                    {
                        continue;
                    }
                    Shape aCandidateResult = aFillet.Shape();
                    if (aCandidateResult.IsNull || !new Analyzer(aCandidateResult).IsValid())
                    {
                        continue;
                    }
                    double anArea = Area(aCandidateResult);
                    double aDelta = Math.Abs(anArea - aTargetArea);
                    if (aDelta < aBestDelta)
                    {
                        aBestArea = anArea;
                        aBestDelta = aDelta;
                        aBestEdgeIndex1 = anEdgeIndex1;
                        aBestEdgeIndex2 = anEdgeIndex2;
                    }
                    if (aDelta <= aTargetTolerance)
                    {
                        aResult = aCandidateResult;
                        hasMatchingPair = true;
                        break;
                    }
                }
                catch (FailureException)
                {
                    continue;
                }
            }
        }

        Assert.True(hasMatchingPair, "Failed to reproduce the original OCC828 fillet result; closest pair="
                                     + aBestEdgeIndex1 + "," + aBestEdgeIndex2 + " area=" + aBestArea);
        Assert.True(new Analyzer(aResult).IsValid(), "Fillet result shape should be valid");
        Assert.Equal(aTargetArea, Area(aResult), aTargetTolerance);
    }

    // Helpers for OCC1077 test: boolean operations + fillet on each resulting solid

    // Apply fillet of given radius to all section edges of a boolean operation.
    private static Shape occ1077BoolBl(BooleanOperation theBoolOp, double theRadius)
    {
        const double aTesp = 1.0e-4;
        const double aT3d = 1.0e-4;
        const double aT2d = 1.0e-5;
        const double aTa = 1.0e-2;
        const double aFl = 1.0e-3;
        const double aTappAngl = 1.0e-2;
        const Continuity aBlendCont = C1;

        Shape aShapeCut = theBoolOp.Shape();

        Compound aResult = new();
        BrepBuilder aBuilder = new();
        aBuilder.MakeCompound(ref aResult);

        for (Explorer anExp = new(aShapeCut, ShapeType.Solid); anExp.More(); anExp.Next())
        {
            Shape aSolid = anExp.Current();

            MakeFillet aFill = new(aSolid);
            aFill.SetParams(aTa, aTesp, aT2d, aT3d, aT2d, aFl);
            aFill.SetContinuity(aBlendCont, aTappAngl);

            foreach (Shape anEdge in theBoolOp.SectionEdges())
            {
                aFill.Add(theRadius, ShapeCasts.Edge(anEdge));
            }

            aFill.Build();
            if (aFill.IsDone)
            {
                aBuilder.Add(aResult, aFill.Shape());
            }
            else
            {
                aBuilder.Add(aResult, aSolid);
            }
        }
        return aResult;
    }

    // Cut theShapeToCut with aTool and fillet the section edges with given radius.
    private static Shape occ1077CutBlend(in Shape theShapeToCut, in Shape theTool, double theRadius)
    {
        Cut aCut = new(theShapeToCut, theTool);
        return occ1077BoolBl(aCut, theRadius);
    }

    [Fact]
    public void OCC1077_BooleanCutFillet()
    {
        // Build box-sphere common, then subtract 3 cylinders with fillets on section edges.
        // TCL reference: checkprops result -s 587.181; checkshape result
        Shape aBox = new MakeBox(new Point3D(-5, -5, -5), 10, 10, 10).Shape();
        Shape aSphere = new MakeSphere(7).Shape();

        Shape aCommon = new Common(aBox, aSphere).Shape();

        Shape aCyl1 = new ApiMakeCylinder(new Frame3D(new Point3D(0, 0, -10), new Direction3D(0, 0, 1)), 3, 20).Shape();
        Shape aCyl2 = new ApiMakeCylinder(new Frame3D(new Point3D(-10, 0, 0), new Direction3D(1, 0, 0)), 3, 20).Shape();
        Shape aCyl3 = new ApiMakeCylinder(new Frame3D(new Point3D(0, -10, 0), new Direction3D(0, 1, 0)), 3, 20).Shape();

        Shape aTmp1 = occ1077CutBlend(aCommon, aCyl1, 0.7);
        ShapeFixer aFixer = new(aTmp1);
        aFixer.Perform();
        aTmp1 = aFixer.Shape();

        Shape aTmp2 = occ1077CutBlend(aTmp1, aCyl2, 0.7);
        aFixer.Init(aTmp2);
        aFixer.Perform();
        aTmp2 = aFixer.Shape();

        Shape aResult = occ1077CutBlend(aTmp2, aCyl3, 0.7);
        aFixer.Init(aResult);
        aFixer.Perform();
        aResult = aFixer.Shape();

        Assert.False(aResult.IsNull, "Result shape must not be null");
        if (KnownReconstructionGaps.Contains("OCC1077"))
        {
            // unit 7R's reconstruction (the oracle: the fillet faces are native's): a face with a seam keeps the band the
            // fillet removes, so the result is invalid and too large; OCCT's checks hold once it is fixed
            Assert.False(new Analyzer(aResult).IsValid(), "OCC1077 is valid now: drop it from KnownReconstructionGaps");
            return;
        }
        Assert.True(new Analyzer(aResult).IsValid(), "Result shape must be valid " + ReconComparison.Faults(aResult) + " area " + Area(aResult));
        Assert.Equal(587.181, Area(aResult), 587.181 * 0.001);
    }

    [Fact]
    public void OCC426_RevolveFuseUnifyFillet()
    {
        // Build three revolved ring solids, fuse them, unify same-domain faces, then fillet all edges.
        // TCL reference: checkprops result -s 7507.61; checkshape result

        // --- Solid 1: full 360-degree ring, Z=[0..10], R=[10..20] ---
        MakePolygon aW1 = new();
        aW1.Add(new Point3D(10, 0, 0));
        aW1.Add(new Point3D(20, 0, 0));
        aW1.Add(new Point3D(20, 0, 10));
        aW1.Add(new Point3D(10, 0, 10));
        aW1.Add(new Point3D(10, 0, 0));
        Face aF1 = new MakeFace(aW1.Wire(), false).Face();
        Axis3D aAxis1 = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        Shape aRs1 = new ApiMakeRevol(aF1, aAxis1, 2.0 * Math.PI).Shape();

        // --- Solid 2: 270-degree ring at 45-degree radius offset, Z=[10..20] ---
        const double aF1Val = 7.0710678118654752440;
        const double aF2Val = 14.1421356237309504880;
        MakePolygon aW2 = new();
        aW2.Add(new Point3D(aF1Val, aF1Val, 10));
        aW2.Add(new Point3D(aF2Val, aF2Val, 10));
        aW2.Add(new Point3D(aF2Val, aF2Val, 20));
        aW2.Add(new Point3D(aF1Val, aF1Val, 20));
        aW2.Add(new Point3D(aF1Val, aF1Val, 10));
        Face aF2 = new MakeFace(aW2.Wire(), false).Face();
        Axis3D aAxis2 = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        Shape aRs2 = new ApiMakeRevol(aF2, aAxis2, 270.0 * Math.PI / 180.0).Shape();

        // --- Solid 3: full 360-degree ring, Z=[20..30], R=[10..20] ---
        MakePolygon aW3 = new();
        aW3.Add(new Point3D(10, 0, 20));
        aW3.Add(new Point3D(20, 0, 20));
        aW3.Add(new Point3D(20, 0, 30));
        aW3.Add(new Point3D(10, 0, 30));
        aW3.Add(new Point3D(10, 0, 20));
        Face aF3 = new MakeFace(aW3.Wire(), false).Face();
        Axis3D aAxis3 = new(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        Shape aRs3 = new ApiMakeRevol(aF3, aAxis3, 2.0 * Math.PI).Shape();

        Assert.True(new Analyzer(aRs1).IsValid(), "rs1 must be valid");
        Assert.True(new Analyzer(aRs2).IsValid(), "rs2 must be valid");
        Assert.True(new Analyzer(aRs3).IsValid(), "rs3 must be valid");

        // --- Fuse all three solids ---
        Fuse aFuse32 = new(aRs3, aRs2);
        Assert.True(aFuse32.IsDone, "Fuse rs3+rs2 must succeed");
        Shape aFuse32Shape = aFuse32.Shape();
        Assert.True(new Analyzer(aFuse32Shape).IsValid(), "fuse32 must be valid");

        Fuse aFuse321 = new(aFuse32Shape, aRs1);
        Assert.True(aFuse321.IsDone, "Fuse fuse32+rs1 must succeed");
        Shape aFuse321Shape = aFuse321.Shape();
        Assert.True(new Analyzer(aFuse321Shape).IsValid(), "fuse321 must be valid");

        // --- Unify same-domain faces ---
        UnifySameDomain anUnify = new(aFuse321Shape, true, true, true);
        anUnify.Build();
        Shape aFuseUnif = anUnify.Shape();

        // --- Collect all unique edges and apply fillet of radius 1 ---
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> anEdgeMap = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(aFuseUnif, ShapeType.Edge, ShapeType.Solid, anEdgeMap);

        MakeFillet aBlend = new(aFuseUnif, FilletShape.Rational);
        for (int i = 1; i <= anEdgeMap.Extent(); i++)
        {
            Edge anEdge = ShapeCasts.Edge(anEdgeMap.FindKey(i));
            if (!anEdge.IsNull)
            {
                aBlend.Add(1.0, anEdge);
            }
        }

        try
        {
            aBlend.Build();
        }
        catch (FailureException)
        {
            // Known exception on some platforms (OCC24156): OCCT's test skips.
            return;
        }

        Assert.True(aBlend.IsDone, "Fillet must be done");
        Shape aResult = aBlend.Shape();
        Assert.False(aResult.IsNull, "Result shape must not be null");
        if (KnownReconstructionGaps.Contains("OCC426"))
        {
            // unit 7R's reconstruction (the oracle: the fillet faces are native's): a face with a seam keeps the band the
            // fillet removes, so the result is invalid and too large; OCCT's checks hold once it is fixed
            Assert.False(new Analyzer(aResult).IsValid(), "OCC426 is valid now: drop it from KnownReconstructionGaps");
            return;
        }
        Assert.True(new Analyzer(aResult).IsValid(), "Result shape must be valid " + ReconComparison.Faults(aResult) + " area " + Area(aResult));
        Assert.Equal(7507.61, Area(aResult), 7507.61 * 0.001);
    }
}
