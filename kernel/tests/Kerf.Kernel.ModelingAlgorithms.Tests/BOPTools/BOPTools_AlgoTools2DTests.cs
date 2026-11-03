// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Tools;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests.BOPTools;

/// <summary>BOPTools_AlgoTools2D: pcurves of edges on faces.</summary>
public class BOPTools_AlgoTools2DTests
{
    private static Face FaceOfType<T>(in Shape theShape) where T : Surface =>
        BOPToolsTestShapes.Faces(theShape).First(f => Tool.Surface(f) is T);

    // The largest distance between the 3D curve and the surface at its pcurve, on 101 samples.
    private static double Deviation(Curve3D theC3D, Curve2D theC2D, Surface theS, double theT1, double theT2)
    {
        double aMax = 0.0;
        for (int i = 0; i <= 100; ++i)
        {
            double aT = theT1 + (theT2 - theT1) * i / 100.0;
            Point2D aUV = theC2D.Value(aT);
            aMax = Math.Max(aMax, theC3D.Value(aT).DistanceTo(theS.Value(aUV.X, aUV.Y)));
        }
        return aMax;
    }

    [Fact]
    public void IntermediatePoint_known_answers()
    {
        Assert.Equal(0.43213918, AlgoTools2D.IntermediatePoint(0.0, 1.0));
        Assert.Equal(IntersectionTools.IntermediatePoint(-3.0, 7.5), AlgoTools2D.IntermediatePoint(-3.0, 7.5));
        Edge aE = BOPToolsTestShapes.Edges(BOPToolsTestShapes.Box())[0];
        Tool.Range(aE, out double aT1, out double aT2);
        Assert.Equal(AlgoTools2D.IntermediatePoint(aT1, aT2), AlgoTools2D.IntermediatePoint(aE));
    }

    [Fact]
    public void CurveOnSurface_and_PointOnSurface_return_the_stored_pcurve()
    {
        foreach ((string aName, Shape aShape) in BOPToolsTestShapes.Primitives())
        {
            foreach (Face aF in BOPToolsTestShapes.Faces(aShape))
            {
                foreach (Edge aE in BOPToolsTestShapes.Edges(aF))
                {
                    Curve2D? aStored = Tool.CurveOnSurface(aE, aF, out double aF1, out double aL1);
                    Assert.NotNull(aStored);
                    Tool.Range(aE, out double aR1, out double aR2);
                    bool aLong = aR2 - aR1 >= global::Kerf.Engine.Foundation.Precision.PConfusion();
                    Assert.Equal(aLong, AlgoTools2D.HasCurveOnSurface(aE, aF));

                    AlgoTools2D.CurveOnSurface(aE, aF, out Curve2D? aC, out double aFirst, out double aLast, out double aTol);
                    Assert.Same(aStored, aC);
                    Assert.Equal(aF1, aFirst);
                    Assert.Equal(aL1, aLast);
                    Assert.Equal(Tool.Tolerance(aE), aTol);

                    double aT = AlgoTools2D.IntermediatePoint(aFirst, aLast);
                    AlgoTools2D.PointOnSurface(aE, aF, aT, out double aU, out double aV, new IntersectionContext());
                    Assert.Equal(aStored!.Value(aT), new Point2D(aU, aV));
                    _ = aName;
                }
            }
        }
    }

    [Fact]
    public void EdgeTangent_is_the_oriented_unit_tangent()
    {
        Shape aBox = BOPToolsTestShapes.Box();
        foreach (Edge aE in BOPToolsTestShapes.Edges(aBox))
        {
            Vector3D aTau = new();
            Assert.True(AlgoTools2D.EdgeTangent(aE, 0.5, ref aTau));
            Assert.Equal(1.0, aTau.Magnitude, 1.0e-15);
            Point3D aP1 = Tool.Pnt(Exploration.FirstVertex(aE, true));
            Point3D aP2 = Tool.Pnt(Exploration.LastVertex(aE, true));
            Assert.True(aTau.IsParallel(new Vector3D(aP1, aP2), 1.0e-12) && aTau.Dot(new Vector3D(aP1, aP2)) > 0);
        }
        // the sphere's degenerated edges: false, output kept
        Edge aDeg = BOPToolsTestShapes.Edges(BOPToolsTestShapes.Sphere()).First(e => Tool.Degenerated(e));
        Vector3D aKeep = new(1, 2, 3);
        Assert.False(AlgoTools2D.EdgeTangent(aDeg, 0.5, ref aKeep));
        Assert.Equal(new Vector3D(1, 2, 3), aKeep);
    }

    [Fact]
    public void IsEdgeIsoline_on_box_and_cylinder()
    {
        Shape aCyl = BOPToolsTestShapes.Cylinder();
        Face aF = FaceOfType<CylindricalSurface>(aCyl);
        foreach (Edge aE in BOPToolsTestShapes.Edges(aF))
        {
            AlgoTools2D.IsEdgeIsoline(aE, aF, out bool isU, out bool isV);
            if (Tool.IsClosed(aE, aF))
            {
                // the seam: constant U
                Assert.True(isU);
                Assert.False(isV);
            }
            else
            {
                // the circles: constant V
                Assert.False(isU);
                Assert.True(isV);
            }
        }
        foreach (Face aBF in BOPToolsTestShapes.Faces(BOPToolsTestShapes.Box()))
        {
            foreach (Edge aE in BOPToolsTestShapes.Edges(aBF))
            {
                AlgoTools2D.IsEdgeIsoline(aE, aBF, out bool isU, out bool isV);
                Assert.True(isU ^ isV);
            }
        }
    }

    [Fact]
    public void MakePCurveOnFace_projects_curves_lying_on_the_faces()
    {
        // a circle around the cylinder at height 2 (a V isoline), a meridian of the sphere, a circle of the torus, a
        // line of the cone
        Face aCylF = FaceOfType<CylindricalSurface>(BOPToolsTestShapes.Cylinder());
        Face aSphF = FaceOfType<SphericalSurface>(BOPToolsTestShapes.Sphere());
        Face aTorF = FaceOfType<ToroidalSurface>(BOPToolsTestShapes.Torus());
        Face aConF = FaceOfType<ConicalSurface>(BOPToolsTestShapes.Cone());
        (Face Face, Curve3D Curve, double T1, double T2)[] aCases =
        [
            (aCylF, new CircleCurve3D(new Frame3D(new Point3D(0, 0, 2), new Direction3D(0, 0, 1)), 2.0), 0.5, 2.0),
            (aSphF, new CircleCurve3D(new Frame3D(new Point3D(0, 0, 0), new Direction3D(0, 1, 0)), 3.0), 0.3, 1.2),
            (aTorF, new CircleCurve3D(new Frame3D(new Point3D(0, 0, 1.5), new Direction3D(0, 0, 1)), 5.0), 1.0, 5.0),
            (aConF, Tool.Curve(BOPToolsTestShapes.Edges(aConF).First(e => Tool.IsClosed(e, aConF)), out _, out _)!, 0.5, 3.5),
        ];
        foreach ((Face aF, Curve3D aC3D, double aT1, double aT2) in aCases)
        {
            double aTol = 0.0;
            AlgoTools2D.MakePCurveOnFace(aF, aC3D, aT1, aT2, out Curve2D? aC2D, ref aTol);
            Assert.NotNull(aC2D);
            Surface aS = Tool.Surface(aF)!;
            Assert.True(Deviation(aC3D, aC2D!, aS, aT1, aT2) < 1.0e-6);
            Assert.True(aTol < 1.0e-6, $"{aTol}");

            // the pcurve's middle point is inside the face's UV bounds
            BrepTools.UVBounds(aF, out double aU1, out double aU2, out double aV1, out double aV2);
            Point2D aMid = aC2D!.Value(0.5 * (aT1 + aT2));
            Assert.InRange(aMid.X, aU1 - 1.0e-9, aU2 + 1.0e-9);
            Assert.InRange(aMid.Y, aV1 - 1.0e-9, aV2 + 1.0e-9);

            // with a context: the same answer
            double aTolC = 0.0;
            AlgoTools2D.MakePCurveOnFace(aF, aC3D, aT1, aT2, out Curve2D? aC2DC, ref aTolC, new IntersectionContext());
            Assert.Equal(aTol, aTolC);
            Assert.Equal(aC2D.Value(aT1), aC2DC!.Value(aT1));
            Assert.Equal(aC2D.Value(aT2), aC2DC.Value(aT2));
        }
        // the whole-range overload on a trimmed curve
        TrimmedCurve3D aTrim = new(new CircleCurve3D(new Frame3D(new Point3D(0, 0, 3), new Direction3D(0, 0, 1)), 2.0), 1.0, 2.5);
        AlgoTools2D.MakePCurveOnFace(aCylF, aTrim, out Curve2D? aPC, out double aTolW);
        Assert.True(Deviation(aTrim, aPC!, Tool.Surface(aCylF)!, 1.0, 2.5) < 1.0e-6);
        Assert.True(aTolW < 1.0e-6);
    }

    [Fact]
    public void BuildPCurveForEdgeOnFace_stores_a_pcurve_in_an_unfrozen_edge()
    {
        Shape aCyl = new ApiMakeCylinder(2, 5).InternalShape();
        Face aF = FaceOfType<CylindricalSurface>(aCyl);
        CircleCurve3D aCircle = new(new Frame3D(new Point3D(0, 0, 2.5), new Direction3D(0, 0, 1)), 2.0);
        Edge aE = ShapeCasts.Edge(new MakeEdge(aCircle, 0.5, 2.0).InternalShape());
        Assert.False(AlgoTools2D.HasCurveOnSurface(aE, aF));

        AlgoTools2D.BuildPCurveForEdgeOnFace(aE, aF, new IntersectionContext());
        Assert.True(AlgoTools2D.HasCurveOnSurface(aE, aF));
        Curve2D aPC = Tool.CurveOnSurface(aE, aF, out double aT1, out double aT2)!;
        Assert.Equal(0.5, aT1);
        Assert.Equal(2.0, aT2);
        Assert.True(Deviation(aCircle, aPC, Tool.Surface(aF)!, aT1, aT2) < 1.0e-6);

        // a second call keeps the stored pcurve
        AlgoTools2D.BuildPCurveForEdgeOnFace(aE, aF);
        Assert.Same(aPC, Tool.CurveOnSurface(aE, aF, out _, out _));

        // a frozen edge is written into: it throws (plan 2.3)
        Edge aFrozen = ShapeCasts.Edge(new MakeEdge(aCircle, 0.5, 2.0).Shape());
        Assert.ThrowsAny<global::Kerf.Engine.Foundation.DomainErrorException>(() => AlgoTools2D.BuildPCurveForEdgeOnFace(aFrozen, aF));
    }

    [Fact]
    public void Make2D_returns_the_stored_pcurve_and_keeps_the_tolerance()
    {
        Face aF = BOPToolsTestShapes.Faces(BOPToolsTestShapes.Torus())[0];
        Edge aE = BOPToolsTestShapes.Edges(aF)[0];
        double aTol = 123.0;
        AlgoTools2D.Make2D(aE, aF, out Curve2D? aC, out _, out _, ref aTol);
        Assert.Same(Tool.CurveOnSurface(aE, aF, out _, out _), aC);
        Assert.Equal(123.0, aTol);
    }

    [Fact]
    public void AdjustPCurveOnFace_translates_by_whole_periods()
    {
        Face aF = FaceOfType<CylindricalSurface>(BOPToolsTestShapes.Cylinder());
        foreach (int k in new[] { -2, -1, 1, 3 })
        {
            LineCurve2D aL = new(new Point2D(1.0 + k * 2 * Math.PI, 0.0), new Direction2D(0, 1));
            AlgoTools2D.AdjustPCurveOnFace(aF, 1.0, 4.0, aL, out Curve2D? aA);
            Assert.NotSame(aL, aA);
            Assert.Equal(1.0, aA!.Value(2.0).X, 1.0e-12);
            Assert.Equal(2.0, aA.Value(2.0).Y, 1.0e-12);
        }
        // in range: the curve itself
        LineCurve2D aIn = new(new Point2D(1.0, 0.0), new Direction2D(0, 1));
        AlgoTools2D.AdjustPCurveOnFace(aF, 1.0, 4.0, aIn, out Curve2D? aSame, new IntersectionContext());
        Assert.Same(aIn, aSame);
        // the 3D-curve overload takes its range
        TrimmedCurve3D aC3D = new(new LineCurve3D(new Point3D(2, 0, 0), new Direction3D(0, 0, 1)), 1.0, 4.0);
        LineCurve2D aOut = new(new Point2D(1.0 + 2 * Math.PI, 0.0), new Direction2D(0, 1));
        AlgoTools2D.AdjustPCurveOnFace(aF, aC3D, aOut, out Curve2D? aA2);
        Assert.Equal(1.0, aA2!.Value(2.0).X, 1.0e-12);
    }

    [Fact]
    public void AttachExistingPCurve_without_old_pcurve_returns_1()
    {
        Face aF = BOPToolsTestShapes.Faces(BOPToolsTestShapes.Torus())[0];
        Edge anOld = BOPToolsTestShapes.Edges(BOPToolsTestShapes.Cone())[0];
        Edge aNew = ShapeCasts.Edge(new MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).InternalShape());
        Assert.Equal(1, AlgoTools2D.AttachExistingPCurve(anOld, aNew, aF, new IntersectionContext()));
    }

    /// <summary>
    /// OCCT's file-static IsClosed of BOPTools_AlgoTools2D_1.cxx: its loop condition <c>(More() || iCnt == 2)</c>
    /// restarts the exhausted explorer once both occurrences of a seam are counted, so a true seam gives false
    /// (occt-issues.md). The port reproduces it.
    /// </summary>
    [Fact]
    public void IsClosed_of_AlgoTools2D_1_reproduces_OCCTs_restarting_loop()
    {
        Face aF = FaceOfType<CylindricalSurface>(BOPToolsTestShapes.Cylinder());
        Edge aSeam = BOPToolsTestShapes.Edges(aF).First(e => Tool.IsClosed(e, aF));
        Assert.True(Tool.IsClosed(aSeam, aF));
        Assert.Equal(2, BOPToolsTestShapes.Edges(aF).Count(e => e.IsSame(aSeam)));
        Assert.False(AlgoTools2D.IsClosed(aSeam, aF)); // OCCT
        Edge aCircle = BOPToolsTestShapes.Edges(aF).First(e => !Tool.IsClosed(e, aF));
        Assert.False(AlgoTools2D.IsClosed(aCircle, aF));
    }
}
