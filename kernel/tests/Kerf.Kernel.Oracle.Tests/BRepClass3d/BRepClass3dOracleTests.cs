// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Classification;
using Kerf.Geometry;
using Kerf.Topology;
using Xunit.Abstractions;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// BRepClass3d_SolidClassifier against native OCCT on the random scenes of <see cref="ClassificationScenes"/> (those
/// with surfaces): points at random in and around the shape, on its faces with and without small and large offsets,
/// on its edges and at its vertices, with two tolerances; one classifier per scene, loaded once (both sides), then
/// PerformInfinitePoint. State, Rejected, IsOnAFace and the face (by its place in the shape) are compared. This
/// reaches BRepClass (the face classifier and its intersector), BRepTopAdaptor_FClass2d, IntCurvesFace (with the
/// polyhedron of the surfaces of extrusion and revolution and of B-spline faces) and BRepBndLib (the tree of the
/// edges' and vertices' boxes).
/// </summary>
public class BRepClass3dOracleTests(ITestOutputHelper output)
{
    private const int Scenes = 150;

    [Fact]
    public void SolidClassifier_matches_native_OCCT()
    {
        var anOracle = new Oracle(output, 20261031);
        int aNbPoints = 0, aNbIn = 0, aNbOn = 0, aNbOut = 0;
        for (int aScene = 0; aScene < Scenes; aScene++)
        {
            if (!ClassificationScenes.HasSurfaces(aScene))
            {
                continue;
            }

            Twin<N.TopoDS_Shape, Shape> aShape = ClassificationScenes.Scene(anOracle, aScene);
            List<Twin<N.TopoDS_Shape, Shape>> aFaces = ClassificationScenes.Explore(aShape, ShapeType.Face);
            List<Point3D> aPoints = ClassificationScenes.SamplePoints(anOracle, aShape.P, 25, 4);
            string aWhat = $"scene {aScene} ({ClassificationScenes.Name(aScene)})";

            using var aNC = new N.BRepClass3d_SolidClassifier();
            aNC.Load(aShape.N);
            var aPC = new SolidClassifier();
            aPC.Load(aShape.P);
            foreach (Point3D aP in aPoints)
            {
                double aTol = anOracle.Bool() ? global::Kerf.Engine.Foundation.Precision.Confusion() : 1e-3;
                using var aNP = Oracle.ToNative(aP);
                aNC.Perform(aNP, aTol);
                aPC.Perform(aP, aTol);
                string aPWhat = $"{aWhat}, point {aP} tol {aTol}";
                anOracle.Same((int)aNC.State(), (int)aPC.State(), aPWhat + " State");
                anOracle.Same(aNC.Rejected(), aPC.Rejected(), aPWhat + " Rejected");
                anOracle.Same(aNC.IsOnAFace(), aPC.IsOnAFace, aPWhat + " IsOnAFace");
                using (N.TopoDS_Face aNF = aNC.Face())
                {
                    anOracle.Same(ClassificationScenes.NativeIndex(aFaces, aNF), ClassificationScenes.PortIndex(aFaces, aPC.Face()), aPWhat + " Face");
                }

                aNbPoints++;
                switch (aPC.State())
                {
                    case Containment.In:
                        aNbIn++;
                        break;
                    case Containment.On:
                        aNbOn++;
                        break;
                    default:
                        aNbOut++;
                        break;
                }
            }

            // a classifier made with the point, and the infinite point
            Point3D aFirst = aPoints[0];
            using (var aNP = Oracle.ToNative(aFirst))
            using (var aNC2 = new N.BRepClass3d_SolidClassifier(aShape.N, aNP, 1e-7))
            {
                var aPC2 = new SolidClassifier(aShape.P, aFirst, 1e-7);
                anOracle.Same((int)aNC2.State(), (int)aPC2.State(), aWhat + " constructor State");
            }

            aNC.PerformInfinitePoint(1e-7);
            aPC.PerformInfinitePoint(1e-7);
            anOracle.Same((int)aNC.State(), (int)aPC.State(), aWhat + " infinite point State");
            anOracle.Same(aNC.Rejected(), aPC.Rejected(), aWhat + " infinite point Rejected");
            using (N.TopoDS_Face aNF = aNC.Face())
            {
                anOracle.Same(ClassificationScenes.NativeIndex(aFaces, aNF), ClassificationScenes.PortIndex(aFaces, aPC.Face()), aWhat + " infinite point Face");
            }
        }

        output.WriteLine($"{aNbPoints} points: {aNbIn} IN, {aNbOn} ON, {aNbOut} OUT");
        anOracle.Report("BRepClass3d_SolidClassifier");
    }
}
