// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// Port of: src/ModelingAlgorithms/TKFillet/GTests/BRepFilletAPI_MakeFillet_Test.cxx (OCCT V8_0_1)

using static Occt.ChFi3d_FilletShape;
using static Occt.GeomAbs_Shape;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>BRepFilletAPI_MakeFillet (unit 7N3): OCCT's GTest.</summary>
public class BRepFilletAPI_MakeFilletTests
{
    /// <summary>
    /// The tests whose result the reconstruction gets wrong: none since unit 7R3 fixed the seam band (decision 1155),
    /// which OCC1077 and OCC426 met.
    /// </summary>
    private static readonly HashSet<string> KnownReconstructionGaps = [];

    private static double Area(in TopoDS_Shape S)
    {
        GProp_GProps aProps = new();
        BRepGProp.SurfaceProperties(S, aProps);
        return aProps.Mass();
    }

    [Fact]
    public void FilletOneEdge()
    {
        BRepPrimAPI_MakeBox aBoxMaker = new(20.0, 20.0, 20.0);
        TopoDS_Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone());

        TopExp_Explorer anExp = new(aBox, TopAbs_EDGE);
        Assert.True(anExp.More());
        TopoDS_Edge anEdge = TopoDS.Edge(anExp.Current());

        BRepFilletAPI_MakeFillet aFillet = new(aBox);
        aFillet.Add(2.0, anEdge);
        TopoDS_Shape aResult = aFillet.Shape();
        Assert.True(aFillet.IsDone());

        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
    }

    [Fact]
    public void FilletAllEdges()
    {
        BRepPrimAPI_MakeBox aBoxMaker = new(20.0, 20.0, 20.0);
        TopoDS_Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone());

        BRepFilletAPI_MakeFillet aFillet = new(aBox);
        for (TopExp_Explorer anExp = new(aBox, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            aFillet.Add(1.0, TopoDS.Edge(anExp.Current()));
        }
        TopoDS_Shape aResult = aFillet.Shape();
        Assert.True(aFillet.IsDone());

        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
    }

    [Fact]
    public void FilletMoreFaces()
    {
        BRepPrimAPI_MakeBox aBoxMaker = new(20.0, 20.0, 20.0);
        TopoDS_Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone());

        BRepFilletAPI_MakeFillet aFillet = new(aBox);
        for (TopExp_Explorer anExp = new(aBox, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            aFillet.Add(1.0, TopoDS.Edge(anExp.Current()));
        }
        TopoDS_Shape aResult = aFillet.Shape();
        Assert.True(aFillet.IsDone());

        int aFaceCount = 0;
        for (TopExp_Explorer aFaceExp = new(aResult, TopAbs_FACE); aFaceExp.More(); aFaceExp.Next())
        {
            aFaceCount++;
        }
        Assert.True(aFaceCount > 6);
    }

    [Fact]
    public void FilletVariableRadius()
    {
        BRepPrimAPI_MakeBox aBoxMaker = new(20.0, 20.0, 20.0);
        TopoDS_Shape aBox = aBoxMaker.Shape();
        Assert.True(aBoxMaker.IsDone());

        TopExp_Explorer anExp = new(aBox, TopAbs_EDGE);
        Assert.True(anExp.More());
        TopoDS_Edge anEdge = TopoDS.Edge(anExp.Current());

        BRepFilletAPI_MakeFillet aFillet = new(aBox);
        aFillet.Add(1.0, 3.0, anEdge);
        TopoDS_Shape aResult = aFillet.Shape();
        Assert.True(aFillet.IsDone());

        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
    }

    // Test OCC570: BRepFilletAPI_MakeFillet with mixed constant and variable radius.
    // Migrated from QABugs_17.cxx OCC570
    [Fact]
    public void OCC570_MixedVariableConstantRadius()
    {
        BRepPrimAPI_MakeBox aBoxMaker = new(100.0, 100.0, 100.0);
        TopoDS_Shape aBox = aBoxMaker.Shape();

        // Take the first wire of the box and its 4 edges
        TopExp_Explorer aWireExp = new(aBox, TopAbs_WIRE);
        Assert.True(aWireExp.More());

        TopExp_Explorer anEdgeExp = new(aWireExp.Current(), TopAbs_EDGE);
        TopoDS_Edge anE1 = TopoDS.Edge(anEdgeExp.Current());
        anEdgeExp.Next();
        TopoDS_Edge anE2 = TopoDS.Edge(anEdgeExp.Current());
        anEdgeExp.Next();
        TopoDS_Edge anE3 = TopoDS.Edge(anEdgeExp.Current());
        anEdgeExp.Next();
        TopoDS_Edge anE4 = TopoDS.Edge(anEdgeExp.Current());

        // Variable radius law: 4 (parameter, radius) control points
        NCollection_Array1<gp_Pnt2d> aVarRadius = new(1, 4);
        aVarRadius.SetValue(1, new gp_Pnt2d(0.0, 5.0));
        aVarRadius.SetValue(2, new gp_Pnt2d(0.3, 15.0));
        aVarRadius.SetValue(3, new gp_Pnt2d(0.7, 15.0));
        aVarRadius.SetValue(4, new gp_Pnt2d(1.0, 5.0));

        BRepFilletAPI_MakeFillet aFillet = new(aBox);
        aFillet.SetContinuity(GeomAbs_C1, 0.001);
        aFillet.Add(aVarRadius, anE1);
        aFillet.Add(5.0, anE2);
        aFillet.Add(aVarRadius, anE3);
        aFillet.Add(5.0, anE4);

        aFillet.Build();
        Assert.True(aFillet.IsDone(), "Fillet operation should succeed");

        TopoDS_Shape aResult = aFillet.Shape();
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid(), "Result shape should be valid");
        Assert.Equal(58500.0, Area(aResult), 58500.0 * 0.01);
    }

    // OCC828: BRepFilletAPI_MakeFillet succeeds on a prism built from a complex wire of arcs and line segments
    // (trefoil-like profile).
    [Fact]
    public void Bug828_FilletOnComplexPrism()
    {
        gp_Pnt aPnt11 = new(-27.598139, -7.0408573, 0.0);
        gp_Pnt aPnt12 = new(-28.483755, -17.487625, 0.0);
        gp_Pnt aPnt13 = new(-19.555504, -22.983587, 0.0);
        GC_MakeArcOfCircle anArc1 = new(aPnt11, aPnt12, aPnt13);

        gp_Pnt aPnt21 = new(12.125083, -22.983587, 0.0);
        gp_Pnt aPnt22 = new(21.1572, -17.27554, 0.0);
        gp_Pnt aPnt23 = new(19.878168, -6.6677585, 0.0);
        GC_MakeArcOfCircle anArc2 = new(aPnt21, aPnt22, aPnt23);

        gp_Pnt aPnt31 = new(3.265825, 13.724955, 0.0);
        gp_Pnt aPnt32 = new(-4.7233953, 17.406338, 0.0);
        gp_Pnt aPnt33 = new(-12.529893, 13.351856, 0.0);
        GC_MakeArcOfCircle anArc3 = new(aPnt31, aPnt32, aPnt33);

        GC_MakeSegment aSegment1 = new(aPnt13, aPnt21);
        GC_MakeSegment aSegment2 = new(aPnt23, aPnt31);
        GC_MakeSegment aSegment3 = new(aPnt33, aPnt11);

        BRepBuilderAPI_MakeWire aWireMaker = new();
        aWireMaker.Add(new BRepBuilderAPI_MakeEdge(anArc1.Value()).Edge());
        aWireMaker.Add(new BRepBuilderAPI_MakeEdge(aSegment1.Value()).Edge());
        aWireMaker.Add(new BRepBuilderAPI_MakeEdge(anArc2.Value()).Edge());
        aWireMaker.Add(new BRepBuilderAPI_MakeEdge(aSegment2.Value()).Edge());
        aWireMaker.Add(new BRepBuilderAPI_MakeEdge(anArc3.Value()).Edge());
        aWireMaker.Add(new BRepBuilderAPI_MakeEdge(aSegment3.Value()).Edge());
        Assert.True(aWireMaker.IsDone());

        TopoDS_Wire aWire = aWireMaker.Wire();
        TopoDS_Face aFace = new BRepBuilderAPI_MakeFace(aWire).Face();
        Assert.False(aFace.IsNull());

        const double aSlabThick = 111.0;
        BRepPrimAPI_MakePrism aSlab = new(aFace, new gp_Vec(0, 0, aSlabThick), true);
        Assert.True(aSlab.IsDone());
        TopoDS_Shape aSlabShape = aSlab.Shape();
        Assert.True(new BRepCheck_Analyzer(aSlabShape).IsValid(), "Prism shape should be valid");

        List<TopoDS_Edge> anEdges = [];
        for (TopExp_Explorer anExp = new(aSlabShape, TopAbs_EDGE); anExp.More(); anExp.Next())
        {
            anEdges.Add(TopoDS.Edge(anExp.Current()));
        }
        Assert.True(anEdges.Count >= 2, "Expected at least 2 edges in the prism");

        const double aTargetArea = 17816.2;
        const double aTargetTolerance = aTargetArea * 0.002;
        TopoDS_Shape aResult = new();
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
                    BRepFilletAPI_MakeFillet aFillet = new(aSlabShape, ChFi3d_Rational);
                    aFillet.SetParams(1.0e-2, 1.0e-4, 1.0e-5, 1.0e-4, 1.0e-5, 1.0e-3);
                    aFillet.SetContinuity(GeomAbs_C1, 1.0e-2);
                    aFillet.Add(10.0, anEdges[anEdgeIndex1 - 1]);
                    aFillet.Add(10.0, anEdges[anEdgeIndex2 - 1]);
                    aFillet.Build();
                    if (!aFillet.IsDone())
                    {
                        continue;
                    }
                    TopoDS_Shape aCandidateResult = aFillet.Shape();
                    if (aCandidateResult.IsNull() || !new BRepCheck_Analyzer(aCandidateResult).IsValid())
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
                catch (Standard_Failure)
                {
                    continue;
                }
            }
        }

        Assert.True(hasMatchingPair, "Failed to reproduce the original OCC828 fillet result; closest pair="
                                     + aBestEdgeIndex1 + "," + aBestEdgeIndex2 + " area=" + aBestArea);
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid(), "Fillet result shape should be valid");
        Assert.Equal(aTargetArea, Area(aResult), aTargetTolerance);
    }

    // Helpers for OCC1077 test: boolean operations + fillet on each resulting solid

    // Apply fillet of given radius to all section edges of a boolean operation.
    private static TopoDS_Shape occ1077BoolBl(BRepAlgoAPI_BooleanOperation theBoolOp, double theRadius)
    {
        const double aTesp = 1.0e-4;
        const double aT3d = 1.0e-4;
        const double aT2d = 1.0e-5;
        const double aTa = 1.0e-2;
        const double aFl = 1.0e-3;
        const double aTappAngl = 1.0e-2;
        const GeomAbs_Shape aBlendCont = GeomAbs_C1;

        TopoDS_Shape aShapeCut = theBoolOp.Shape();

        TopoDS_Compound aResult = new();
        BRep_Builder aBuilder = new();
        aBuilder.MakeCompound(ref aResult);

        for (TopExp_Explorer anExp = new(aShapeCut, TopAbs_SOLID); anExp.More(); anExp.Next())
        {
            TopoDS_Shape aSolid = anExp.Current();

            BRepFilletAPI_MakeFillet aFill = new(aSolid);
            aFill.SetParams(aTa, aTesp, aT2d, aT3d, aT2d, aFl);
            aFill.SetContinuity(aBlendCont, aTappAngl);

            foreach (TopoDS_Shape anEdge in theBoolOp.SectionEdges())
            {
                aFill.Add(theRadius, TopoDS.Edge(anEdge));
            }

            aFill.Build();
            if (aFill.IsDone())
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
    private static TopoDS_Shape occ1077CutBlend(in TopoDS_Shape theShapeToCut, in TopoDS_Shape theTool, double theRadius)
    {
        BRepAlgoAPI_Cut aCut = new(theShapeToCut, theTool);
        return occ1077BoolBl(aCut, theRadius);
    }

    [Fact]
    public void OCC1077_BooleanCutFillet()
    {
        // Build box-sphere common, then subtract 3 cylinders with fillets on section edges.
        // TCL reference: checkprops result -s 587.181; checkshape result
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(new gp_Pnt(-5, -5, -5), 10, 10, 10).Shape();
        TopoDS_Shape aSphere = new BRepPrimAPI_MakeSphere(7).Shape();

        TopoDS_Shape aCommon = new BRepAlgoAPI_Common(aBox, aSphere).Shape();

        TopoDS_Shape aCyl1 = new BRepPrimAPI_MakeCylinder(new gp_Ax2(new gp_Pnt(0, 0, -10), new gp_Dir(0, 0, 1)), 3, 20).Shape();
        TopoDS_Shape aCyl2 = new BRepPrimAPI_MakeCylinder(new gp_Ax2(new gp_Pnt(-10, 0, 0), new gp_Dir(1, 0, 0)), 3, 20).Shape();
        TopoDS_Shape aCyl3 = new BRepPrimAPI_MakeCylinder(new gp_Ax2(new gp_Pnt(0, -10, 0), new gp_Dir(0, 1, 0)), 3, 20).Shape();

        TopoDS_Shape aTmp1 = occ1077CutBlend(aCommon, aCyl1, 0.7);
        ShapeFix_Shape aFixer = new(aTmp1);
        aFixer.Perform();
        aTmp1 = aFixer.Shape();

        TopoDS_Shape aTmp2 = occ1077CutBlend(aTmp1, aCyl2, 0.7);
        aFixer.Init(aTmp2);
        aFixer.Perform();
        aTmp2 = aFixer.Shape();

        TopoDS_Shape aResult = occ1077CutBlend(aTmp2, aCyl3, 0.7);
        aFixer.Init(aResult);
        aFixer.Perform();
        aResult = aFixer.Shape();

        Assert.False(aResult.IsNull(), "Result shape must not be null");
        if (KnownReconstructionGaps.Contains("OCC1077"))
        {
            // unit 7R's reconstruction (the oracle: the fillet faces are native's): a face with a seam keeps the band the
            // fillet removes, so the result is invalid and too large; OCCT's checks hold once it is fixed
            Assert.False(new BRepCheck_Analyzer(aResult).IsValid(), "OCC1077 is valid now: drop it from KnownReconstructionGaps");
            return;
        }
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid(), "Result shape must be valid " + ReconComparison.Faults(aResult) + " area " + Area(aResult));
        Assert.Equal(587.181, Area(aResult), 587.181 * 0.001);
    }

    [Fact]
    public void OCC426_RevolveFuseUnifyFillet()
    {
        // Build three revolved ring solids, fuse them, unify same-domain faces, then fillet all edges.
        // TCL reference: checkprops result -s 7507.61; checkshape result

        // --- Solid 1: full 360-degree ring, Z=[0..10], R=[10..20] ---
        BRepBuilderAPI_MakePolygon aW1 = new();
        aW1.Add(new gp_Pnt(10, 0, 0));
        aW1.Add(new gp_Pnt(20, 0, 0));
        aW1.Add(new gp_Pnt(20, 0, 10));
        aW1.Add(new gp_Pnt(10, 0, 10));
        aW1.Add(new gp_Pnt(10, 0, 0));
        TopoDS_Face aF1 = new BRepBuilderAPI_MakeFace(aW1.Wire(), false).Face();
        gp_Ax1 aAxis1 = new(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        TopoDS_Shape aRs1 = new BRepPrimAPI_MakeRevol(aF1, aAxis1, 2.0 * Math.PI).Shape();

        // --- Solid 2: 270-degree ring at 45-degree radius offset, Z=[10..20] ---
        const double aF1Val = 7.0710678118654752440;
        const double aF2Val = 14.1421356237309504880;
        BRepBuilderAPI_MakePolygon aW2 = new();
        aW2.Add(new gp_Pnt(aF1Val, aF1Val, 10));
        aW2.Add(new gp_Pnt(aF2Val, aF2Val, 10));
        aW2.Add(new gp_Pnt(aF2Val, aF2Val, 20));
        aW2.Add(new gp_Pnt(aF1Val, aF1Val, 20));
        aW2.Add(new gp_Pnt(aF1Val, aF1Val, 10));
        TopoDS_Face aF2 = new BRepBuilderAPI_MakeFace(aW2.Wire(), false).Face();
        gp_Ax1 aAxis2 = new(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        TopoDS_Shape aRs2 = new BRepPrimAPI_MakeRevol(aF2, aAxis2, 270.0 * Math.PI / 180.0).Shape();

        // --- Solid 3: full 360-degree ring, Z=[20..30], R=[10..20] ---
        BRepBuilderAPI_MakePolygon aW3 = new();
        aW3.Add(new gp_Pnt(10, 0, 20));
        aW3.Add(new gp_Pnt(20, 0, 20));
        aW3.Add(new gp_Pnt(20, 0, 30));
        aW3.Add(new gp_Pnt(10, 0, 30));
        aW3.Add(new gp_Pnt(10, 0, 20));
        TopoDS_Face aF3 = new BRepBuilderAPI_MakeFace(aW3.Wire(), false).Face();
        gp_Ax1 aAxis3 = new(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        TopoDS_Shape aRs3 = new BRepPrimAPI_MakeRevol(aF3, aAxis3, 2.0 * Math.PI).Shape();

        Assert.True(new BRepCheck_Analyzer(aRs1).IsValid(), "rs1 must be valid");
        Assert.True(new BRepCheck_Analyzer(aRs2).IsValid(), "rs2 must be valid");
        Assert.True(new BRepCheck_Analyzer(aRs3).IsValid(), "rs3 must be valid");

        // --- Fuse all three solids ---
        BRepAlgoAPI_Fuse aFuse32 = new(aRs3, aRs2);
        Assert.True(aFuse32.IsDone(), "Fuse rs3+rs2 must succeed");
        TopoDS_Shape aFuse32Shape = aFuse32.Shape();
        Assert.True(new BRepCheck_Analyzer(aFuse32Shape).IsValid(), "fuse32 must be valid");

        BRepAlgoAPI_Fuse aFuse321 = new(aFuse32Shape, aRs1);
        Assert.True(aFuse321.IsDone(), "Fuse fuse32+rs1 must succeed");
        TopoDS_Shape aFuse321Shape = aFuse321.Shape();
        Assert.True(new BRepCheck_Analyzer(aFuse321Shape).IsValid(), "fuse321 must be valid");

        // --- Unify same-domain faces ---
        ShapeUpgrade_UnifySameDomain anUnify = new(aFuse321Shape, true, true, true);
        anUnify.Build();
        TopoDS_Shape aFuseUnif = anUnify.Shape();

        // --- Collect all unique edges and apply fillet of radius 1 ---
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anEdgeMap = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(aFuseUnif, TopAbs_EDGE, TopAbs_SOLID, anEdgeMap);

        BRepFilletAPI_MakeFillet aBlend = new(aFuseUnif, ChFi3d_Rational);
        for (int i = 1; i <= anEdgeMap.Extent(); i++)
        {
            TopoDS_Edge anEdge = TopoDS.Edge(anEdgeMap.FindKey(i));
            if (!anEdge.IsNull())
            {
                aBlend.Add(1.0, anEdge);
            }
        }

        try
        {
            aBlend.Build();
        }
        catch (Standard_Failure)
        {
            // Known exception on some platforms (OCC24156): OCCT's test skips.
            return;
        }

        Assert.True(aBlend.IsDone(), "Fillet must be done");
        TopoDS_Shape aResult = aBlend.Shape();
        Assert.False(aResult.IsNull(), "Result shape must not be null");
        if (KnownReconstructionGaps.Contains("OCC426"))
        {
            // unit 7R's reconstruction (the oracle: the fillet faces are native's): a face with a seam keeps the band the
            // fillet removes, so the result is invalid and too large; OCCT's checks hold once it is fixed
            Assert.False(new BRepCheck_Analyzer(aResult).IsValid(), "OCC426 is valid now: drop it from KnownReconstructionGaps");
            return;
        }
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid(), "Result shape must be valid " + ReconComparison.Faults(aResult) + " area " + Area(aResult));
        Assert.Equal(7507.61, Area(aResult), 7507.61 * 0.001);
    }
}
