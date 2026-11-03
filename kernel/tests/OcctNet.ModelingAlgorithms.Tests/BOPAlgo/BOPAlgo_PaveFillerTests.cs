// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans;
using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Booleans.Data;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.TestHarness;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// OCCT's GTests of BOPAlgo_PaveFiller (src/ModelingAlgorithms/TKBO/GTests/BOPAlgo_PaveFiller_Test.cxx), and unit
/// tests of the pave filler on primitives.
/// </summary>
/// <remarks>
/// OCCT's tests run BRepAlgoAPI_Fuse, whose crash they guard against was in the pave filler (degenerated edges
/// without p-curves). With unit 6F's builders, OCCT's IsDone and volume checks are back; the two loft tests run since
/// unit 5T ported BRepOffsetAPI_ThruSections.
/// </remarks>
public class BOPAlgo_PaveFillerTests
{
    internal static PaveFiller Fill(bool theParallel, params Shape[] theArgs)
    {
        PaveFiller aPF = new();
        NCollection_List<Shape> aLS = new();
        foreach (Shape aS in theArgs)
        {
            aLS.Append(aS);
        }
        aPF.SetArguments(aLS);
        aPF.SetRunParallel(theParallel);
        aPF.Perform();
        return aPF;
    }

    //! Check if shape has degenerated edges without pcurves
    private static bool HasDegeneratedEdgesWithoutPCurves(in Shape theShape)
    {
        for (Explorer aFaceExp = new(theShape, ShapeType.Face); aFaceExp.More(); aFaceExp.Next())
        {
            Face aFace = ShapeCasts.Face(aFaceExp.Current());
            for (Explorer anEdgeExp = new(aFace, ShapeType.Edge); anEdgeExp.More(); anEdgeExp.Next())
            {
                Edge anEdge = ShapeCasts.Edge(anEdgeExp.Current());
                if (Tool.Degenerated(anEdge))
                {
                    Curve2D? aPCurve = Tool.CurveOnSurface(anEdge, aFace, out _, out _);
                    if (aPCurve is null)
                    {
                        return true;
                    }
                }
            }
        }
        return false;
    }

    private static double GetVolume(in Shape theShape)
    {
        Properties aProps = new();
        MassPropertiesBuilder.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    private static Wire CreateCircularWire(double theRadius, double theZ)
    {
        Circle3D aCircle = new(new Frame3D(new Point3D(0, 0, theZ), new Direction3D(0, 0, 1)), theRadius);
        MakeEdge anEdgeMaker = new(aCircle);
        MakeWire aWireMaker = new(anEdgeMaker.Edge());
        return aWireMaker.Wire();
    }

    [Fact]
    public void FuseConeLoftWithBox_DegeneratedEdge()
    {
        // Create a loft from a circular wire to a point (apex)
        // This creates a cone-like shape with a degenerated edge at the top
        Wire aBaseWire = CreateCircularWire(10.0, 0.0);
        Vertex anApex = new MakeVertex(new Point3D(0, 0, 20.0)).Vertex();

        ThruSections aLoftMaker = new(true, true); // solid, ruled
        aLoftMaker.AddWire(aBaseWire);
        aLoftMaker.AddVertex(anApex);
        aLoftMaker.Build();

        Assert.True(aLoftMaker.IsDone); // Loft operation should succeed
        Shape aLoft = aLoftMaker.Shape();
        Assert.False(aLoft.IsNull);

        // Create a box that intersects the loft
        Shape aBox = new MakeBox(new Point3D(-5, -5, 5), 10.0, 10.0, 10.0).Shape();

        // Perform Boolean fuse - this should not crash even if there are
        // degenerated edges without pcurves
        Fuse aFuser = new(aLoft, aBox);

        Assert.True(aFuser.IsDone); // Boolean fuse should succeed for loft with degenerated edges
        Assert.False(aFuser.Shape().IsNull);

        // Verify the result has reasonable volume (both shapes contribute)
        double aLoftVolume = GetVolume(aLoft);
        double aBoxVolume = GetVolume(aBox);
        double aResultVolume = GetVolume(aFuser.Shape());

        Assert.True(aResultVolume > 0.0);
        Assert.True(aResultVolume < aLoftVolume + aBoxVolume);
    }

    [Fact]
    public void FuseConeWithBox_DegeneratedEdge()
    {
        // Create a cone - has degenerated edge at apex
        Shape aCone = new ApiMakeCone(10.0, 0.0, 20.0).Shape(); // R1=10, R2=0 (apex), H=20
        Assert.False(aCone.IsNull);

        // Create a box that intersects near the apex
        Shape aBox = new MakeBox(new Point3D(-5, -5, 15), 10.0, 10.0, 10.0).Shape();

        // Perform Boolean fuse
        Fuse aFuser = new(aCone, aBox);
        Assert.True(aFuser.IsDone); // Boolean fuse of cone and box should succeed
        Assert.False(aFuser.Shape().IsNull);

        double aResultVolume = GetVolume(aFuser.Shape());
        Assert.True(aResultVolume > 0.0); // Result should have positive volume
    }

    private static Wire Quadrilateral(params Point3D[] thePoints)
    {
        MakeWire aWireMaker = new();
        for (int i = 0; i < 4; i++)
        {
            aWireMaker.Add(new MakeEdge(thePoints[i], thePoints[(i + 1) % 4]).Edge());
        }
        return aWireMaker.Wire();
    }

    [Fact]
    public void FuseTwoLofts_RobustnessCheck()
    {
        // Create first loft: quadrilateral wire to quadrilateral wire
        Wire aBottom1 = Quadrilateral(new Point3D(10, -10, 0), new Point3D(100, -10, 0), new Point3D(100, -100, 0), new Point3D(10, -100, 0));
        Wire aTop1 = Quadrilateral(new Point3D(0, 0, 10), new Point3D(100, 0, 10), new Point3D(100, -100, 10), new Point3D(0, -100, 10));

        ThruSections aLoft1 = new(true, true);
        aLoft1.AddWire(aBottom1);
        aLoft1.AddWire(aTop1);
        aLoft1.Build();
        Assert.True(aLoft1.IsDone); // First loft should succeed

        // Create second loft
        Wire aBottom2 = Quadrilateral(new Point3D(0, 0, 10), new Point3D(100, 0, 10), new Point3D(100, -100, 10), new Point3D(0, -100, 10));
        Wire aTop2 = Quadrilateral(new Point3D(0, 0, 250), new Point3D(100, 0, 250), new Point3D(100, -100, 250), new Point3D(0, -100, 250));

        ThruSections aLoft2 = new(true, true);
        aLoft2.AddWire(aBottom2);
        aLoft2.AddWire(aTop2);
        aLoft2.Build();
        Assert.True(aLoft2.IsDone); // Second loft should succeed

        // Fuse the two lofts - this is the operation that could crash
        Fuse aFuser = new(aLoft1.Shape(), aLoft2.Shape());

        Assert.True(aFuser.IsDone); // Boolean fuse of two lofts should succeed
        Assert.False(aFuser.Shape().IsNull);

        double aResultVolume = GetVolume(aFuser.Shape());
        Assert.True(aResultVolume > 0.0);
    }

    [Fact]
    public void FuseConeWithRemovedPCurve_NullPCurveHandling()
    {
        // Create a standard cone - it has a degenerated edge at the apex
        Shape aCone = new ApiMakeCone(10.0, 0.0, 20.0).Shape();
        Assert.False(aCone.IsNull);

        // Find the conical face and its degenerated edge, then rebuild without pcurve
        Face aConicalFace = new();
        Edge aDegeneratedEdge = new();
        for (Explorer aFaceExp = new(aCone, ShapeType.Face); aFaceExp.More(); aFaceExp.Next())
        {
            Face aFace = ShapeCasts.Face(aFaceExp.Current());
            for (Explorer anEdgeExp = new(aFace, ShapeType.Edge); anEdgeExp.More(); anEdgeExp.Next())
            {
                Edge anEdge = ShapeCasts.Edge(anEdgeExp.Current());
                if (Tool.Degenerated(anEdge))
                {
                    aConicalFace = aFace;
                    aDegeneratedEdge = anEdge;
                    break;
                }
            }
            if (!aDegeneratedEdge.IsNull)
            {
                break;
            }
        }
        Assert.False(aDegeneratedEdge.IsNull);

        // Get the surface of the conical face
        Surface aSurf = Tool.Surface(aConicalFace)!;
        double aFirst = 0.0;
        double aLast = 2.0 * Math.PI;
        Vertex aVertex = ShapeCasts.Vertex(new Explorer(aDegeneratedEdge, ShapeType.Vertex).Current());

        // Create a new degenerated edge without pcurve
        BrepBuilder aBuilder = new();
        Edge aNewDegEdge = new();
        aBuilder.MakeEdge(ref aNewDegEdge);
        aBuilder.Add(aNewDegEdge, aVertex.Oriented(Forward));
        aBuilder.Add(aNewDegEdge, aVertex.Oriented(Reversed));
        aBuilder.Degenerated(aNewDegEdge, true);
        aBuilder.Range(aNewDegEdge, aFirst, aLast);
        // NOTE: We intentionally do NOT add a pcurve here!

        // Rebuild the wire with the new degenerated edge
        Wire aNewWire = new();
        aBuilder.MakeWire(ref aNewWire);
        Explorer aWireExp = new(aConicalFace, ShapeType.Wire);
        if (aWireExp.More()) // Only process first wire
        {
            Wire aWire = ShapeCasts.Wire(aWireExp.Current());
            for (Explorer anEdgeExp = new(aWire, ShapeType.Edge); anEdgeExp.More(); anEdgeExp.Next())
            {
                Edge anEdge = ShapeCasts.Edge(anEdgeExp.Current());
                if (Tool.Degenerated(anEdge))
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
        Face aNewFace = new();
        aBuilder.MakeFace(ref aNewFace, aSurf, global::Kerf.Engine.Foundation.Precision.Confusion());
        aBuilder.Add(aNewFace, aNewWire);

        // Build a new shell/solid with the modified face
        Shell aNewShell = new();
        aBuilder.MakeShell(ref aNewShell);
        for (Explorer aFaceExp2 = new(aCone, ShapeType.Face); aFaceExp2.More(); aFaceExp2.Next())
        {
            Face aFace = ShapeCasts.Face(aFaceExp2.Current());
            aBuilder.Add(aNewShell, aFace.IsSame(aConicalFace) ? aNewFace : aFace);
        }

        Solid aNewSolid = new();
        aBuilder.MakeSolid(ref aNewSolid);
        aBuilder.Add(aNewSolid, aNewShell);

        // Verify we created a shape with degenerated edge without pcurve
        Assert.True(HasDegeneratedEdgesWithoutPCurves(aNewSolid));

        // Create a box that intersects near the apex
        Shape aBox = new MakeBox(new Point3D(-5, -5, 15), 10.0, 10.0, 10.0).Shape();

        // Perform Boolean fuse - this should NOT crash with the fix
        // We expect either success or graceful failure, but NOT a crash
        Fuse aFuser = new(aNewSolid, aBox);
        _ = aFuser.IsDone;
    }

    // ------------------------------------------------------------------------------------------- unit tests

    [Fact]
    public void TwoBoxes_SectionEdgesAndSplits()
    {
        Shape a = new MakeBox(new Point3D(0, 0, 0), 1, 1, 1).Shape();
        Shape b = new MakeBox(new Point3D(0.5, 0.25, 0.25), 1, 0.5, 0.5).Shape();
        PaveFiller aPF = Fill(false, a, b);
        Assert.False(aPF.HasErrors);
        Assert.True(aPF.NonDestructive()); // the arguments are frozen, so locked
        DS aDS = aPF.DS();
        // b's four edges along X cross a's face x = 1; the section of that face with b's four side faces
        Assert.Equal(4, aDS.InterfEF().Length());
        int aNbSection = 0;
        foreach (InterfFF aFF in aDS.InterfFF())
        {
            aNbSection += aFF.Curves().Length();
        }
        Assert.Equal(4, aNbSection);
        Assert.True(aDS.ShapeCount > aDS.SourceShapeCount);
    }

    [Fact]
    public void DisjointArguments_NoInterference()
    {
        Shape a = new MakeBox(new Point3D(0, 0, 0), 1, 1, 1).Shape();
        Shape b = new MakeBox(new Point3D(5, 0, 0), 1, 1, 1).Shape();
        PaveFiller aPF = Fill(true, a, b);
        Assert.False(aPF.HasErrors);
        Assert.Equal(0, aPF.DS().Interferences().Extent());
        Assert.Equal(aPF.DS().SourceShapeCount, aPF.DS().ShapeCount);
    }

    [Fact]
    public void Errors_TooFewAndNullArguments()
    {
        PaveFiller aPF = new();
        aPF.Perform();
        Assert.True(aPF.HasError(typeof(AlertTooFewArguments)));

        aPF = new PaveFiller();
        aPF.AddArgument(new MakeBox(1, 1, 1).Shape());
        aPF.AddArgument(new Shape());
        aPF.Perform();
        Assert.True(aPF.HasError(typeof(AlertNullInputShapes)));
    }

    [Fact]
    public void NonDestructive_FromLockedArguments()
    {
        // unfrozen arguments: the mode is the caller's
        Shape a = new MakeBox(new Point3D(0, 0, 0), 1, 1, 1).InternalShape();
        Shape b = new MakeBox(new Point3D(0.5, 0.5, 0.5), 1, 1, 1).InternalShape();
        PaveFiller aPF = Fill(false, a, b);
        Assert.False(aPF.HasErrors);
        Assert.False(aPF.NonDestructive());
        // a frozen argument turns it on (decision 577)
        aPF = Fill(false, new MakeBox(1, 1, 1).Shape(), new MakeSphere(0.7).Shape());
        Assert.False(aPF.HasErrors);
        Assert.True(aPF.NonDestructive());
    }

    [Fact]
    public void ConeAndBox_SerialEqualsParallel()
    {
        DeterminismHarness.AssertSerialEqualsParallel(() =>
        {
            Shape aCone = new ApiMakeCone(10.0, 0.0, 20.0).Shape();
            Shape aBox = new MakeBox(new Point3D(-5, -5, 15), 10.0, 10.0, 10.0).Shape();
            PaveFiller aPF = Fill(true, aCone, aBox);
            Assert.False(aPF.HasErrors);
            return PaveFillerDump.Write(aPF.DS());
        }, 2);
    }
}
