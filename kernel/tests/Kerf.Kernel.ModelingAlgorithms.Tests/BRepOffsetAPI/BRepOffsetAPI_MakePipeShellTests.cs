// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// Port of: src/ModelingAlgorithms/TKOffset/GTests/BRepOffsetAPI_MakePipeShell_Test.cxx (OCCT V8_0_1)

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Construction.Laws;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>BRepOffsetAPI_MakePipeShell (unit 5S2): OCCT's GTest (bug OCC332).</summary>
public class BRepOffsetAPI_MakePipeShellTests
{
    // Computes the total linear length of a wire.
    private static double wireLength(in Wire theWire)
    {
        Properties aProps = new();
        MassPropertiesBuilder.LinearProperties(theWire, aProps);
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
                                      out Solid theWallSolid,
                                      out Shape theGasSolid)
    {
        theWallSolid = new Solid();
        theGasSolid = new Shape();
        double aBendAngle = theLength / theMajorRadius;
        if (aBendAngle >= Math.PI)
        {
            return false;
        }

        double aRadiusL = theDia1 / 2.0;
        double aRadiusR = theDia2 / 2.0;

        Frame3D anOrigin = new(new Point3D(5000.0, -300.0, 1000.0), new Direction3D(0.0, -1.0, -1.0));

        Plane3D aCirc1Plane = new(anOrigin.Location(), anOrigin.Direction());
        Circle3D aFaceCircle = new(anOrigin, aRadiusL);
        Circle3D anOutFaceCircle = new(anOrigin, aRadiusL + theWallThickness);

        Point3D aCircCenter = anOrigin.Location().Translated(theMajorRadius * new Vector3D(anOrigin.XDirection()));
        Axis3D aCircAxis = new(aCircCenter, anOrigin.YDirection());
        Plane3D aCirc2Plane = aCirc1Plane.Rotated(aCircAxis, aBendAngle);

        Frame3D aSpineAxis = new(aCircCenter, anOrigin.YDirection(), anOrigin.XDirection().Reversed());
        Circle3D aSpineCircle = new(aSpineAxis, theMajorRadius);

        Wire aWire1 = new MakeWire(new MakeEdge(aFaceCircle).Edge()).Wire();
        Wire anOutWire = new MakeWire(new MakeEdge(anOutFaceCircle).Edge()).Wire();

        Curve3D aSpineCurve = new MakeArcOfCircle(aSpineCircle, 0.0, aBendAngle, true).Value();

        Linear aLaw1 = new();
        Linear aLaw2 = new();
        aLaw1.Set(aSpineCurve.FirstParameter(), 1.0, aSpineCurve.LastParameter(), aRadiusR / aRadiusL);
        aLaw2.Set(aSpineCurve.FirstParameter(),
                  1.0,
                  aSpineCurve.LastParameter(),
                  (aRadiusR + theWallThickness) / (aRadiusL + theWallThickness));

        MakeEdge aMkEdge = new(aSpineCurve);
        if (!aMkEdge.IsDone)
        {
            return false;
        }
        Wire aSpineWire = new MakeWire(aMkEdge.Edge()).Wire();

        MakePipeShell aMkPipe1 = new(aSpineWire);
        aMkPipe1.SetTolerance(1.0e-8, 1.0e-8, 1.0e-6);
        aMkPipe1.SetLaw(aWire1, aLaw1, false, false);
        aMkPipe1.Build();
        if (!aMkPipe1.IsDone)
        {
            return false;
        }

        MakePipeShell aMkPipe2 = new(aSpineWire);
        aMkPipe2.SetTolerance(1.0e-8, 1.0e-8, 1.0e-6);
        aMkPipe2.SetLaw(anOutWire, aLaw2, false, false);
        aMkPipe2.Build();
        if (!aMkPipe2.IsDone)
        {
            return false;
        }

        // Build opening face at start of bend (annular ring)
        MakeFace aMkFace = new();
        PlaneSurface aPlane1 = new(aCirc1Plane);
        aMkFace.Init(aPlane1, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        aMkFace.Add(ShapeCasts.Wire(aMkPipe2.FirstShape()));
        aMkFace.Add(ShapeCasts.Wire(aMkPipe1.FirstShape().Reversed()));
        if (!aMkFace.IsDone)
        {
            return false;
        }
        Face aFace1 = aMkFace.Face();

        // Build opening face at end of bend (annular ring)
        PlaneSurface aPlane2 = new(aCirc2Plane);
        aMkFace.Init(aPlane2, false, global::Kerf.Engine.Foundation.Precision.Confusion());
        aMkFace.Add(ShapeCasts.Wire(aMkPipe2.LastShape()));
        aMkFace.Add(ShapeCasts.Wire(aMkPipe1.LastShape().Reversed()));
        if (!aMkFace.IsDone)
        {
            return false;
        }
        Face aFace2 = aMkFace.Face();

        // Assemble tube shell from inner pipe face, outer pipe face, and two annular ring faces
        Shell aTubeShell = new();
        BrepBuilder aBuilder = new();
        aBuilder.MakeShell(ref aTubeShell);

        Explorer aFaceExp = new(aMkPipe1.Shape(), ShapeType.Face);
        if (aFaceExp.More())
        {
            aBuilder.Add(aTubeShell, aFaceExp.Current().Reversed());
        }

        // Extract the gas solid before modifying mkPipe1 to a solid
        aMkPipe1.MakeSolid();
        theGasSolid = aMkPipe1.Shape();

        aFaceExp.Init(aMkPipe2.Shape(), ShapeType.Face);
        if (aFaceExp.More())
        {
            aBuilder.Add(aTubeShell, aFaceExp.Current());
        }

        aBuilder.Add(aTubeShell, aFace1.Reversed());
        aBuilder.Add(aTubeShell, aFace2);
        aTubeShell.Closed(Tool.IsClosed(aTubeShell));

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

        bool aBuilt = buildBentTube(aWallThickness, aDia1, aDia2, aMajorRadius, aLength, out Solid aWallSolid,
                                    out Shape aGasSolid);
        Assert.True(aBuilt, "buildBentTube construction failed");

        // Validate both solids
        Assert.True(new Analyzer(aWallSolid).IsValid(), "wallSolid is not valid");
        Assert.True(new Analyzer(aGasSolid).IsValid(), "gasSolid is not valid");

        // Collect all wire lengths in wallSolid
        List<double> aWireLengths = [];
        for (Explorer anExp = new(aWallSolid, ShapeType.Wire); anExp.More(); anExp.Next())
        {
            double aLen = wireLength(ShapeCasts.Wire(anExp.Current()));
            if (aLen > global::Kerf.Engine.Foundation.Precision.Confusion())
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
