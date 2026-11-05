// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Fillets.Api;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// ChFiKPart_ComputeData_ChAsymPlnCyl: a distance-angle chamfer between a plane and a cylinder along a generatrix is the
/// same solid wherever the cylinder's axis points (occt-issues.md, unit 7N1's MakeChAsym row; Kerf decision 1475).
/// </summary>
public class ChFiKPartChAsymTests
{
    // Half a cylinder of radius 3 and height 5 about the axes' Z, chamfered by 1 at 30 degrees on a generatrix edge
    // where the lateral face meets the planar one: the chamfered solid and its volume.
    private static (Shape Solid, double Volume) Chamfered(in Frame3D theAxes)
    {
        Shape aHalf = new ApiMakeCylinder(theAxes, 3.0, 5.0, Math.PI).Shape();
        Edge anEdge = default;
        Face aPlane = default;
        for (var anE = new Explorer(aHalf, ShapeType.Edge); anE.More(); anE.Next())
        {
            Edge aCandidate = ShapeCasts.Edge(anE.Current());
            if (new EdgeCurveAdaptor(aCandidate).GetType() != CurveKind.Line)
            {
                continue;
            }
            // a generatrix of the cylinder, not the axis
            for (var aF = new Explorer(aHalf, ShapeType.Face); aF.More(); aF.Next())
            {
                Face aFace = ShapeCasts.Face(aF.Current());
                bool aHasEdge = false;
                for (var aFE = new Explorer(aFace, ShapeType.Edge); aFE.More(); aFE.Next())
                {
                    aHasEdge |= aFE.Current().IsSame(aCandidate);
                }
                if (aHasEdge && new FaceSurfaceAdaptor(aFace).GetType() == SurfaceKind.Cylinder)
                {
                    anEdge = aCandidate;
                }
                else if (aHasEdge && new FaceSurfaceAdaptor(aFace).GetType() == SurfaceKind.Plane)
                {
                    aPlane = aFace;
                }
            }
            if (!anEdge.IsNull)
            {
                break;
            }
            aPlane = default;
        }
        Assert.False(anEdge.IsNull || aPlane.IsNull);

        var aChamfer = new MakeChamfer(aHalf);
        aChamfer.AddDA(1.0, Math.PI / 6.0, anEdge, aPlane);
        aChamfer.Build();
        Assert.True(aChamfer.IsDone);
        Shape aResult = aChamfer.Shape();
        Assert.True(new Analyzer(aResult).IsValid());
        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(aResult, aProps);
        return (aResult, aProps.Mass());
    }

    [Fact]
    public void A_distance_angle_chamfer_doesnt_depend_on_the_axis_direction()
    {
        (_, double aV0) = Chamfered(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)));
        Assert.True(aV0 < Math.PI * 9.0 * 5.0 / 2.0);
        Assert.True(aV0 > Math.PI * 9.0 * 5.0 / 2.0 - 5.0);
        (_, double aV1) = Chamfered(new Frame3D(new Point3D(1, -2, 3), new Direction3D(1, 2, 3), new Direction3D(3, 0, -1)));
        Assert.Equal(aV0, aV1, 1.0e-6);
    }
}
