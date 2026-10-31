// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// Port of: src/ModelingAlgorithms/TKOffset/GTests/BRepOffsetAPI_MakePipeShell_Test.cxx (OCCT V8_0_1)

using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>BRepOffsetAPI_MakePipeShell (unit 5S2): OCCT's GTest (bug OCC332).</summary>
public class BRepOffsetAPI_MakePipeShellTests
{
    // Computes the total linear length of a wire.
    private static double wireLength(in TopoDS_Wire theWire)
    {
        GProp_GProps aProps = new();
        BRepGProp.LinearProperties(theWire, aProps);
        return aProps.Mass();
    }

    // Builds a bent tube solid (wallSolid) and an inner gas solid (gasSolid), as the OCC332 Draw command does: a circular
    // cross-section swept along an arc spine, a linear scaling law varying the inner radius from radius_l to radius_r,
    // the outer radius offset by the wall thickness throughout. False if any construction step fails.
    private static bool buildBentTube(double theWallThickness,
                                      double theDia1,
                                      double theDia2,
                                      double theMajorRadius,
                                      double theLength,
                                      out TopoDS_Solid theWallSolid,
                                      out TopoDS_Shape theGasSolid)
    {
        theWallSolid = new TopoDS_Solid();
        theGasSolid = new TopoDS_Shape();
        double aBendAngle = theLength / theMajorRadius;
        if (aBendAngle >= Math.PI)
        {
            return false;
        }

        double aRadiusL = theDia1 / 2.0;
        double aRadiusR = theDia2 / 2.0;

        gp_Ax2 anOrigin = new(new gp_Pnt(5000.0, -300.0, 1000.0), new gp_Dir(0.0, -1.0, -1.0));

        gp_Pln aCirc1Plane = new(anOrigin.Location(), anOrigin.Direction());
        gp_Circ aFaceCircle = new(anOrigin, aRadiusL);
        gp_Circ anOutFaceCircle = new(anOrigin, aRadiusL + theWallThickness);

        gp_Pnt aCircCenter = anOrigin.Location().Translated(theMajorRadius * new gp_Vec(anOrigin.XDirection()));
        gp_Ax1 aCircAxis = new(aCircCenter, anOrigin.YDirection());
        gp_Pln aCirc2Plane = aCirc1Plane.Rotated(aCircAxis, aBendAngle);

        gp_Ax2 aSpineAxis = new(aCircCenter, anOrigin.YDirection(), anOrigin.XDirection().Reversed());
        gp_Circ aSpineCircle = new(aSpineAxis, theMajorRadius);

        TopoDS_Wire aWire1 = new BRepBuilderAPI_MakeWire(new BRepBuilderAPI_MakeEdge(aFaceCircle).Edge()).Wire();
        TopoDS_Wire anOutWire = new BRepBuilderAPI_MakeWire(new BRepBuilderAPI_MakeEdge(anOutFaceCircle).Edge()).Wire();

        Geom_Curve aSpineCurve = new GC_MakeArcOfCircle(aSpineCircle, 0.0, aBendAngle, true).Value();

        Law_Linear aLaw1 = new();
        Law_Linear aLaw2 = new();
        aLaw1.Set(aSpineCurve.FirstParameter(), 1.0, aSpineCurve.LastParameter(), aRadiusR / aRadiusL);
        aLaw2.Set(aSpineCurve.FirstParameter(),
                  1.0,
                  aSpineCurve.LastParameter(),
                  (aRadiusR + theWallThickness) / (aRadiusL + theWallThickness));

        BRepBuilderAPI_MakeEdge aMkEdge = new(aSpineCurve);
        if (!aMkEdge.IsDone())
        {
            return false;
        }
        TopoDS_Wire aSpineWire = new BRepBuilderAPI_MakeWire(aMkEdge.Edge()).Wire();

        BRepOffsetAPI_MakePipeShell aMkPipe1 = new(aSpineWire);
        aMkPipe1.SetTolerance(1.0e-8, 1.0e-8, 1.0e-6);
        aMkPipe1.SetLaw(aWire1, aLaw1, false, false);
        aMkPipe1.Build();
        if (!aMkPipe1.IsDone())
        {
            return false;
        }

        BRepOffsetAPI_MakePipeShell aMkPipe2 = new(aSpineWire);
        aMkPipe2.SetTolerance(1.0e-8, 1.0e-8, 1.0e-6);
        aMkPipe2.SetLaw(anOutWire, aLaw2, false, false);
        aMkPipe2.Build();
        if (!aMkPipe2.IsDone())
        {
            return false;
        }

        // Build opening face at start of bend (annular ring)
        BRepBuilderAPI_MakeFace aMkFace = new();
        Geom_Plane aPlane1 = new(aCirc1Plane);
        aMkFace.Init(aPlane1, false, Precision.Confusion());
        aMkFace.Add(TopoDS.Wire(aMkPipe2.FirstShape()));
        aMkFace.Add(TopoDS.Wire(aMkPipe1.FirstShape().Reversed()));
        if (!aMkFace.IsDone())
        {
            return false;
        }
        TopoDS_Face aFace1 = aMkFace.Face();

        // Build opening face at end of bend (annular ring)
        Geom_Plane aPlane2 = new(aCirc2Plane);
        aMkFace.Init(aPlane2, false, Precision.Confusion());
        aMkFace.Add(TopoDS.Wire(aMkPipe2.LastShape()));
        aMkFace.Add(TopoDS.Wire(aMkPipe1.LastShape().Reversed()));
        if (!aMkFace.IsDone())
        {
            return false;
        }
        TopoDS_Face aFace2 = aMkFace.Face();

        // Assemble tube shell from inner pipe face, outer pipe face, and two annular ring faces
        TopoDS_Shell aTubeShell = new();
        BRep_Builder aBuilder = new();
        aBuilder.MakeShell(ref aTubeShell);

        TopExp_Explorer aFaceExp = new(aMkPipe1.Shape(), TopAbs_FACE);
        if (aFaceExp.More())
        {
            aBuilder.Add(aTubeShell, aFaceExp.Current().Reversed());
        }

        // Extract the gas solid before modifying mkPipe1 to a solid
        aMkPipe1.MakeSolid();
        theGasSolid = aMkPipe1.Shape();

        aFaceExp.Init(aMkPipe2.Shape(), TopAbs_FACE);
        if (aFaceExp.More())
        {
            aBuilder.Add(aTubeShell, aFaceExp.Current());
        }

        aBuilder.Add(aTubeShell, aFace1.Reversed());
        aBuilder.Add(aTubeShell, aFace2);
        aTubeShell.Closed(BRep_Tool.IsClosed(aTubeShell));

        aBuilder.MakeSolid(ref theWallSolid);
        aBuilder.Add(theWallSolid, aTubeShell);

        return true;
    }

    // Bug OCC332: BRepOffsetAPI_MakePipeShell with linear scaling law along a bent tube. Both solids are valid; the wall
    // solid holds four circular wires of circumferences pi*dia1, pi*(dia1+2*wall), pi*dia2, pi*(dia2+2*wall) within 0.1%.
    [Fact]
    public void Bug332_BentTubeWithScalingLaw()
    {
        const double aWallThickness = 10.0;
        const double aDia1 = 30.0;
        const double aDia2 = 50.0;
        const double aMajorRadius = 100.0;
        const double aLength = 200.0;

        bool aBuilt = buildBentTube(aWallThickness, aDia1, aDia2, aMajorRadius, aLength, out TopoDS_Solid aWallSolid,
                                    out TopoDS_Shape aGasSolid);
        Assert.True(aBuilt, "buildBentTube construction failed");

        // Validate both solids
        Assert.True(new BRepCheck_Analyzer(aWallSolid).IsValid(), "wallSolid is not valid");
        Assert.True(new BRepCheck_Analyzer(aGasSolid).IsValid(), "gasSolid is not valid");

        // Collect all wire lengths in wallSolid
        List<double> aWireLengths = [];
        for (TopExp_Explorer anExp = new(aWallSolid, TopAbs_WIRE); anExp.More(); anExp.Next())
        {
            double aLen = wireLength(TopoDS.Wire(anExp.Current()));
            if (aLen > Precision.Confusion())
            {
                aWireLengths.Add(aLen);
            }
        }

        double aExpLen1 = Math.PI * aDia1; // pi*30  ~ 94.25
        double aExpLen2 = Math.PI * (aDia1 + 2.0 * aWallThickness); // pi*50  ~ 157.08
        // aExpLen3 = pi*dia2 = pi*50, which equals aExpLen2 when dia2/2 == dia1/2 + wall
        double aExpLen4 = Math.PI * (aDia2 + 2.0 * aWallThickness); // pi*70  ~ 219.91

        const double aTol = 0.001; // 0.1% relative tolerance

        int countMatching(double theExpected) => aWireLengths.Count(aLen => Math.Abs(aLen - theExpected) / theExpected <= aTol);

        Assert.True(countMatching(aExpLen1) >= 1, $"No wire found with circumference pi*dia1 = {aExpLen1}");
        // aExpLen2 and aExpLen3 are identical (dia2/2 == dia1/2 + wall), so expect at least 2 matches
        Assert.True(countMatching(aExpLen2) >= 2, $"Expected 2 wires with circumference pi*50 = {aExpLen2}");
        Assert.True(countMatching(aExpLen4) >= 1, $"No wire found with circumference pi*(dia2+2*wall) = {aExpLen4}");
    }
}
