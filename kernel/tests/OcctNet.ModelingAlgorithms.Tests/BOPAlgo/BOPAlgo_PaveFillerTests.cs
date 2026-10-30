// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.TestHarness;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// OCCT's GTests of BOPAlgo_PaveFiller (src/ModelingAlgorithms/TKBO/GTests/BOPAlgo_PaveFiller_Test.cxx), and unit
/// tests of the pave filler on primitives.
/// </summary>
/// <remarks>
/// OCCT's tests run BRepAlgoAPI_Fuse, whose crash they guard against was in the pave filler (degenerated edges
/// without p-curves). With unit 6F's builders, OCCT's IsDone and volume checks are back. The two loft tests need
/// BRepOffsetAPI_ThruSections, not ported yet.
/// </remarks>
public class BOPAlgo_PaveFillerTests
{
    internal static BOPAlgo_PaveFiller Fill(bool theParallel, params TopoDS_Shape[] theArgs)
    {
        BOPAlgo_PaveFiller aPF = new();
        NCollection_List<TopoDS_Shape> aLS = new();
        foreach (TopoDS_Shape aS in theArgs)
        {
            aLS.Append(aS);
        }
        aPF.SetArguments(aLS);
        aPF.SetRunParallel(theParallel);
        aPF.Perform();
        return aPF;
    }

    //! Check if shape has degenerated edges without pcurves
    private static bool HasDegeneratedEdgesWithoutPCurves(in TopoDS_Shape theShape)
    {
        for (TopExp_Explorer aFaceExp = new(theShape, TopAbs_FACE); aFaceExp.More(); aFaceExp.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(aFaceExp.Current());
            for (TopExp_Explorer anEdgeExp = new(aFace, TopAbs_EDGE); anEdgeExp.More(); anEdgeExp.Next())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(anEdgeExp.Current());
                if (BRep_Tool.Degenerated(anEdge))
                {
                    Geom2d_Curve? aPCurve = BRep_Tool.CurveOnSurface(anEdge, aFace, out _, out _);
                    if (aPCurve is null)
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    private static double GetVolume(in TopoDS_Shape theShape)
    {
        GProp_GProps aProps = new();
        BRepGProp.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    [Fact(Skip = "BRepOffsetAPI_ThruSections (the loft) is not ported yet")]
    public void FuseConeLoftWithBox_DegeneratedEdge()
    {
    }

    [Fact]
    public void FuseConeWithBox_DegeneratedEdge()
    {
        // Create a cone - has degenerated edge at apex
        TopoDS_Shape aCone = new BRepPrimAPI_MakeCone(10.0, 0.0, 20.0).Shape(); // R1=10, R2=0 (apex), H=20
        Assert.False(aCone.IsNull());

        // Create a box that intersects near the apex
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(new gp_Pnt(-5, -5, 15), 10.0, 10.0, 10.0).Shape();

        // Perform Boolean fuse
        BRepAlgoAPI_Fuse aFuser = new(aCone, aBox);
        Assert.True(aFuser.IsDone()); // Boolean fuse of cone and box should succeed
        Assert.False(aFuser.Shape().IsNull());

        double aResultVolume = GetVolume(aFuser.Shape());
        Assert.True(aResultVolume > 0.0); // Result should have positive volume
    }

    [Fact(Skip = "BRepOffsetAPI_ThruSections (the loft) is not ported yet")]
    public void FuseTwoLofts_RobustnessCheck()
    {
    }

    [Fact]
    public void FuseConeWithRemovedPCurve_NullPCurveHandling()
    {
        // Create a standard cone - it has a degenerated edge at the apex
        TopoDS_Shape aCone = new BRepPrimAPI_MakeCone(10.0, 0.0, 20.0).Shape();
        Assert.False(aCone.IsNull());

        // Find the conical face and its degenerated edge, then rebuild without pcurve
        TopoDS_Face aConicalFace = new();
        TopoDS_Edge aDegeneratedEdge = new();
        for (TopExp_Explorer aFaceExp = new(aCone, TopAbs_FACE); aFaceExp.More(); aFaceExp.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(aFaceExp.Current());
            for (TopExp_Explorer anEdgeExp = new(aFace, TopAbs_EDGE); anEdgeExp.More(); anEdgeExp.Next())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(anEdgeExp.Current());
                if (BRep_Tool.Degenerated(anEdge))
                {
                    aConicalFace = aFace;
                    aDegeneratedEdge = anEdge;
                    break;
                }
            }
            if (!aDegeneratedEdge.IsNull())
            {
                break;
            }
        }
        Assert.False(aDegeneratedEdge.IsNull());

        // Get the surface of the conical face
        Geom_Surface aSurf = BRep_Tool.Surface(aConicalFace)!;
        double aFirst = 0.0;
        double aLast = 2.0 * Math.PI;
        TopoDS_Vertex aVertex = TopoDS.Vertex(new TopExp_Explorer(aDegeneratedEdge, TopAbs_VERTEX).Current());

        // Create a new degenerated edge without pcurve
        BRep_Builder aBuilder = new();
        TopoDS_Edge aNewDegEdge = new();
        aBuilder.MakeEdge(ref aNewDegEdge);
        aBuilder.Add(aNewDegEdge, aVertex.Oriented(TopAbs_FORWARD));
        aBuilder.Add(aNewDegEdge, aVertex.Oriented(TopAbs_REVERSED));
        aBuilder.Degenerated(aNewDegEdge, true);
        aBuilder.Range(aNewDegEdge, aFirst, aLast);
        // NOTE: We intentionally do NOT add a pcurve here!

        // Rebuild the wire with the new degenerated edge
        TopoDS_Wire aNewWire = new();
        aBuilder.MakeWire(ref aNewWire);
        TopExp_Explorer aWireExp = new(aConicalFace, TopAbs_WIRE);
        if (aWireExp.More()) // Only process first wire
        {
            TopoDS_Wire aWire = TopoDS.Wire(aWireExp.Current());
            for (TopExp_Explorer anEdgeExp = new(aWire, TopAbs_EDGE); anEdgeExp.More(); anEdgeExp.Next())
            {
                TopoDS_Edge anEdge = TopoDS.Edge(anEdgeExp.Current());
                if (BRep_Tool.Degenerated(anEdge))
                {
                    aBuilder.Add(aNewWire, aNewDegEdge.Oriented(anEdge.Orientation()));
                }
                else
                {
                    aBuilder.Add(aNewWire, anEdge);
                }
            }
        }

        // Create new face with the modified wire
        TopoDS_Face aNewFace = new();
        aBuilder.MakeFace(ref aNewFace, aSurf, Precision.Confusion());
        aBuilder.Add(aNewFace, aNewWire);

        // Build a new shell/solid with the modified face
        TopoDS_Shell aNewShell = new();
        aBuilder.MakeShell(ref aNewShell);
        for (TopExp_Explorer aFaceExp2 = new(aCone, TopAbs_FACE); aFaceExp2.More(); aFaceExp2.Next())
        {
            TopoDS_Face aFace = TopoDS.Face(aFaceExp2.Current());
            aBuilder.Add(aNewShell, aFace.IsSame(aConicalFace) ? aNewFace : aFace);
        }

        TopoDS_Solid aNewSolid = new();
        aBuilder.MakeSolid(ref aNewSolid);
        aBuilder.Add(aNewSolid, aNewShell);

        // Verify we created a shape with degenerated edge without pcurve
        Assert.True(HasDegeneratedEdgesWithoutPCurves(aNewSolid));

        // Create a box that intersects near the apex
        TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(new gp_Pnt(-5, -5, 15), 10.0, 10.0, 10.0).Shape();

        // Perform Boolean fuse - this should NOT crash with the fix
        // We expect either success or graceful failure, but NOT a crash
        BRepAlgoAPI_Fuse aFuser = new(aNewSolid, aBox);
        _ = aFuser.IsDone();
    }

    // ------------------------------------------------------------------------------------------- unit tests

    [Fact]
    public void TwoBoxes_SectionEdgesAndSplits()
    {
        TopoDS_Shape a = new BRepPrimAPI_MakeBox(new gp_Pnt(0, 0, 0), 1, 1, 1).Shape();
        TopoDS_Shape b = new BRepPrimAPI_MakeBox(new gp_Pnt(0.5, 0.25, 0.25), 1, 0.5, 0.5).Shape();
        BOPAlgo_PaveFiller aPF = Fill(false, a, b);
        Assert.False(aPF.HasErrors());
        Assert.True(aPF.NonDestructive()); // the arguments are frozen, so locked
        BOPDS_DS aDS = aPF.DS();
        // b's four edges along X cross a's face x = 1; the section of that face with b's four side faces
        Assert.Equal(4, aDS.InterfEF().Length());
        int aNbSection = 0;
        foreach (BOPDS_InterfFF aFF in aDS.InterfFF())
        {
            aNbSection += aFF.Curves().Length();
        }
        Assert.Equal(4, aNbSection);
        Assert.True(aDS.NbShapes() > aDS.NbSourceShapes());
    }

    [Fact]
    public void DisjointArguments_NoInterference()
    {
        TopoDS_Shape a = new BRepPrimAPI_MakeBox(new gp_Pnt(0, 0, 0), 1, 1, 1).Shape();
        TopoDS_Shape b = new BRepPrimAPI_MakeBox(new gp_Pnt(5, 0, 0), 1, 1, 1).Shape();
        BOPAlgo_PaveFiller aPF = Fill(true, a, b);
        Assert.False(aPF.HasErrors());
        Assert.Equal(0, aPF.DS().Interferences().Extent());
        Assert.Equal(aPF.DS().NbSourceShapes(), aPF.DS().NbShapes());
    }

    [Fact]
    public void Errors_TooFewAndNullArguments()
    {
        BOPAlgo_PaveFiller aPF = new();
        aPF.Perform();
        Assert.True(aPF.HasError(typeof(BOPAlgo_AlertTooFewArguments)));

        aPF = new BOPAlgo_PaveFiller();
        aPF.AddArgument(new BRepPrimAPI_MakeBox(1, 1, 1).Shape());
        aPF.AddArgument(new TopoDS_Shape());
        aPF.Perform();
        Assert.True(aPF.HasError(typeof(BOPAlgo_AlertNullInputShapes)));
    }

    [Fact]
    public void NonDestructive_FromLockedArguments()
    {
        // unfrozen arguments: the mode is the caller's
        TopoDS_Shape a = new BRepPrimAPI_MakeBox(new gp_Pnt(0, 0, 0), 1, 1, 1).InternalShape();
        TopoDS_Shape b = new BRepPrimAPI_MakeBox(new gp_Pnt(0.5, 0.5, 0.5), 1, 1, 1).InternalShape();
        BOPAlgo_PaveFiller aPF = Fill(false, a, b);
        Assert.False(aPF.HasErrors());
        Assert.False(aPF.NonDestructive());
        // a frozen argument turns it on (decision 577)
        aPF = Fill(false, new BRepPrimAPI_MakeBox(1, 1, 1).Shape(), new BRepPrimAPI_MakeSphere(0.7).Shape());
        Assert.False(aPF.HasErrors());
        Assert.True(aPF.NonDestructive());
    }

    [Fact]
    public void ConeAndBox_SerialEqualsParallel()
    {
        DeterminismHarness.AssertSerialEqualsParallel(() =>
        {
            TopoDS_Shape aCone = new BRepPrimAPI_MakeCone(10.0, 0.0, 20.0).Shape();
            TopoDS_Shape aBox = new BRepPrimAPI_MakeBox(new gp_Pnt(-5, -5, 15), 10.0, 10.0, 10.0).Shape();
            BOPAlgo_PaveFiller aPF = Fill(true, aCone, aBox);
            Assert.False(aPF.HasErrors());
            return PaveFillerDump.Write(aPF.DS());
        }, 2);
    }
}
