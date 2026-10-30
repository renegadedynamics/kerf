// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests.BOPTools;

/// <summary>BOPTools_AlgoTools2D: pcurves of edges on faces.</summary>
public class BOPTools_AlgoTools2DTests
{
    private static TopoDS_Face FaceOfType<T>(in TopoDS_Shape theShape) where T : Geom_Surface =>
        BOPToolsTestShapes.Faces(theShape).First(f => BRep_Tool.Surface(f) is T);

    // The largest distance between the 3D curve and the surface at its pcurve, on 101 samples.
    private static double Deviation(Geom_Curve theC3D, Geom2d_Curve theC2D, Geom_Surface theS, double theT1, double theT2)
    {
        double aMax = 0.0;
        for (int i = 0; i <= 100; ++i)
        {
            double aT = theT1 + (theT2 - theT1) * i / 100.0;
            gp_Pnt2d aUV = theC2D.Value(aT);
            aMax = Math.Max(aMax, theC3D.Value(aT).Distance(theS.Value(aUV.X(), aUV.Y())));
        }
        return aMax;
    }

    [Fact]
    public void IntermediatePoint_known_answers()
    {
        Assert.Equal(0.43213918, BOPTools_AlgoTools2D.IntermediatePoint(0.0, 1.0));
        Assert.Equal(IntTools_Tools.IntermediatePoint(-3.0, 7.5), BOPTools_AlgoTools2D.IntermediatePoint(-3.0, 7.5));
        TopoDS_Edge aE = BOPToolsTestShapes.Edges(BOPToolsTestShapes.Box())[0];
        BRep_Tool.Range(aE, out double aT1, out double aT2);
        Assert.Equal(BOPTools_AlgoTools2D.IntermediatePoint(aT1, aT2), BOPTools_AlgoTools2D.IntermediatePoint(aE));
    }

    [Fact]
    public void CurveOnSurface_and_PointOnSurface_return_the_stored_pcurve()
    {
        foreach ((string aName, TopoDS_Shape aShape) in BOPToolsTestShapes.Primitives())
        {
            foreach (TopoDS_Face aF in BOPToolsTestShapes.Faces(aShape))
            {
                foreach (TopoDS_Edge aE in BOPToolsTestShapes.Edges(aF))
                {
                    Geom2d_Curve? aStored = BRep_Tool.CurveOnSurface(aE, aF, out double aF1, out double aL1);
                    Assert.NotNull(aStored);
                    BRep_Tool.Range(aE, out double aR1, out double aR2);
                    bool aLong = aR2 - aR1 >= Precision.PConfusion();
                    Assert.Equal(aLong, BOPTools_AlgoTools2D.HasCurveOnSurface(aE, aF));

                    BOPTools_AlgoTools2D.CurveOnSurface(aE, aF, out Geom2d_Curve? aC, out double aFirst, out double aLast, out double aTol);
                    Assert.Same(aStored, aC);
                    Assert.Equal(aF1, aFirst);
                    Assert.Equal(aL1, aLast);
                    Assert.Equal(BRep_Tool.Tolerance(aE), aTol);

                    double aT = BOPTools_AlgoTools2D.IntermediatePoint(aFirst, aLast);
                    BOPTools_AlgoTools2D.PointOnSurface(aE, aF, aT, out double aU, out double aV, new IntTools_Context());
                    Assert.Equal(aStored!.Value(aT), new gp_Pnt2d(aU, aV));
                    _ = aName;
                }
            }
        }
    }

    [Fact]
    public void EdgeTangent_is_the_oriented_unit_tangent()
    {
        TopoDS_Shape aBox = BOPToolsTestShapes.Box();
        foreach (TopoDS_Edge aE in BOPToolsTestShapes.Edges(aBox))
        {
            gp_Vec aTau = new();
            Assert.True(BOPTools_AlgoTools2D.EdgeTangent(aE, 0.5, ref aTau));
            Assert.Equal(1.0, aTau.Magnitude(), 1.0e-15);
            gp_Pnt aP1 = BRep_Tool.Pnt(TopExp.FirstVertex(aE, true));
            gp_Pnt aP2 = BRep_Tool.Pnt(TopExp.LastVertex(aE, true));
            Assert.True(aTau.IsParallel(new gp_Vec(aP1, aP2), 1.0e-12) && aTau.Dot(new gp_Vec(aP1, aP2)) > 0);
        }
        // the sphere's degenerated edges: false, output kept
        TopoDS_Edge aDeg = BOPToolsTestShapes.Edges(BOPToolsTestShapes.Sphere()).First(e => BRep_Tool.Degenerated(e));
        gp_Vec aKeep = new(1, 2, 3);
        Assert.False(BOPTools_AlgoTools2D.EdgeTangent(aDeg, 0.5, ref aKeep));
        Assert.Equal(new gp_Vec(1, 2, 3), aKeep);
    }

    [Fact]
    public void IsEdgeIsoline_on_box_and_cylinder()
    {
        TopoDS_Shape aCyl = BOPToolsTestShapes.Cylinder();
        TopoDS_Face aF = FaceOfType<Geom_CylindricalSurface>(aCyl);
        foreach (TopoDS_Edge aE in BOPToolsTestShapes.Edges(aF))
        {
            BOPTools_AlgoTools2D.IsEdgeIsoline(aE, aF, out bool isU, out bool isV);
            if (BRep_Tool.IsClosed(aE, aF))
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
        foreach (TopoDS_Face aBF in BOPToolsTestShapes.Faces(BOPToolsTestShapes.Box()))
        {
            foreach (TopoDS_Edge aE in BOPToolsTestShapes.Edges(aBF))
            {
                BOPTools_AlgoTools2D.IsEdgeIsoline(aE, aBF, out bool isU, out bool isV);
                Assert.True(isU ^ isV);
            }
        }
    }

    [Fact]
    public void MakePCurveOnFace_projects_curves_lying_on_the_faces()
    {
        // a circle around the cylinder at height 2 (a V isoline), a meridian of the sphere, a circle of the torus, a
        // line of the cone
        TopoDS_Face aCylF = FaceOfType<Geom_CylindricalSurface>(BOPToolsTestShapes.Cylinder());
        TopoDS_Face aSphF = FaceOfType<Geom_SphericalSurface>(BOPToolsTestShapes.Sphere());
        TopoDS_Face aTorF = FaceOfType<Geom_ToroidalSurface>(BOPToolsTestShapes.Torus());
        TopoDS_Face aConF = FaceOfType<Geom_ConicalSurface>(BOPToolsTestShapes.Cone());
        (TopoDS_Face Face, Geom_Curve Curve, double T1, double T2)[] aCases =
        [
            (aCylF, new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 2), new gp_Dir(0, 0, 1)), 2.0), 0.5, 2.0),
            (aSphF, new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 0), new gp_Dir(0, 1, 0)), 3.0), 0.3, 1.2),
            (aTorF, new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 1.5), new gp_Dir(0, 0, 1)), 5.0), 1.0, 5.0),
            (aConF, BRep_Tool.Curve(BOPToolsTestShapes.Edges(aConF).First(e => BRep_Tool.IsClosed(e, aConF)), out _, out _)!, 0.5, 3.5),
        ];
        foreach ((TopoDS_Face aF, Geom_Curve aC3D, double aT1, double aT2) in aCases)
        {
            double aTol = 0.0;
            BOPTools_AlgoTools2D.MakePCurveOnFace(aF, aC3D, aT1, aT2, out Geom2d_Curve? aC2D, ref aTol);
            Assert.NotNull(aC2D);
            Geom_Surface aS = BRep_Tool.Surface(aF)!;
            Assert.True(Deviation(aC3D, aC2D!, aS, aT1, aT2) < 1.0e-6);
            Assert.True(aTol < 1.0e-6, $"{aTol}");

            // the pcurve's middle point is inside the face's UV bounds
            BRepTools.UVBounds(aF, out double aU1, out double aU2, out double aV1, out double aV2);
            gp_Pnt2d aMid = aC2D!.Value(0.5 * (aT1 + aT2));
            Assert.InRange(aMid.X(), aU1 - 1.0e-9, aU2 + 1.0e-9);
            Assert.InRange(aMid.Y(), aV1 - 1.0e-9, aV2 + 1.0e-9);

            // with a context: the same answer
            double aTolC = 0.0;
            BOPTools_AlgoTools2D.MakePCurveOnFace(aF, aC3D, aT1, aT2, out Geom2d_Curve? aC2DC, ref aTolC, new IntTools_Context());
            Assert.Equal(aTol, aTolC);
            Assert.Equal(aC2D.Value(aT1), aC2DC!.Value(aT1));
            Assert.Equal(aC2D.Value(aT2), aC2DC.Value(aT2));
        }
        // the whole-range overload on a trimmed curve
        Geom_TrimmedCurve aTrim = new(new Geom_Circle(new gp_Ax2(new gp_Pnt(0, 0, 3), new gp_Dir(0, 0, 1)), 2.0), 1.0, 2.5);
        BOPTools_AlgoTools2D.MakePCurveOnFace(aCylF, aTrim, out Geom2d_Curve? aPC, out double aTolW);
        Assert.True(Deviation(aTrim, aPC!, BRep_Tool.Surface(aCylF)!, 1.0, 2.5) < 1.0e-6);
        Assert.True(aTolW < 1.0e-6);
    }

    [Fact]
    public void BuildPCurveForEdgeOnFace_stores_a_pcurve_in_an_unfrozen_edge()
    {
        TopoDS_Shape aCyl = new BRepPrimAPI_MakeCylinder(2, 5).InternalShape();
        TopoDS_Face aF = FaceOfType<Geom_CylindricalSurface>(aCyl);
        Geom_Circle aCircle = new(new gp_Ax2(new gp_Pnt(0, 0, 2.5), new gp_Dir(0, 0, 1)), 2.0);
        TopoDS_Edge aE = TopoDS.Edge(new BRepBuilderAPI_MakeEdge(aCircle, 0.5, 2.0).InternalShape());
        Assert.False(BOPTools_AlgoTools2D.HasCurveOnSurface(aE, aF));

        BOPTools_AlgoTools2D.BuildPCurveForEdgeOnFace(aE, aF, new IntTools_Context());
        Assert.True(BOPTools_AlgoTools2D.HasCurveOnSurface(aE, aF));
        Geom2d_Curve aPC = BRep_Tool.CurveOnSurface(aE, aF, out double aT1, out double aT2)!;
        Assert.Equal(0.5, aT1);
        Assert.Equal(2.0, aT2);
        Assert.True(Deviation(aCircle, aPC, BRep_Tool.Surface(aF)!, aT1, aT2) < 1.0e-6);

        // a second call keeps the stored pcurve
        BOPTools_AlgoTools2D.BuildPCurveForEdgeOnFace(aE, aF);
        Assert.Same(aPC, BRep_Tool.CurveOnSurface(aE, aF, out _, out _));

        // a frozen edge is written into: it throws (plan 2.3)
        TopoDS_Edge aFrozen = TopoDS.Edge(new BRepBuilderAPI_MakeEdge(aCircle, 0.5, 2.0).Shape());
        Assert.ThrowsAny<Standard_DomainError>(() => BOPTools_AlgoTools2D.BuildPCurveForEdgeOnFace(aFrozen, aF));
    }

    [Fact]
    public void Make2D_returns_the_stored_pcurve_and_keeps_the_tolerance()
    {
        TopoDS_Face aF = BOPToolsTestShapes.Faces(BOPToolsTestShapes.Torus())[0];
        TopoDS_Edge aE = BOPToolsTestShapes.Edges(aF)[0];
        double aTol = 123.0;
        BOPTools_AlgoTools2D.Make2D(aE, aF, out Geom2d_Curve? aC, out _, out _, ref aTol);
        Assert.Same(BRep_Tool.CurveOnSurface(aE, aF, out _, out _), aC);
        Assert.Equal(123.0, aTol);
    }

    [Fact]
    public void AdjustPCurveOnFace_translates_by_whole_periods()
    {
        TopoDS_Face aF = FaceOfType<Geom_CylindricalSurface>(BOPToolsTestShapes.Cylinder());
        foreach (int k in new[] { -2, -1, 1, 3 })
        {
            Geom2d_Line aL = new(new gp_Pnt2d(1.0 + k * 2 * Math.PI, 0.0), new gp_Dir2d(0, 1));
            BOPTools_AlgoTools2D.AdjustPCurveOnFace(aF, 1.0, 4.0, aL, out Geom2d_Curve? aA);
            Assert.NotSame(aL, aA);
            Assert.Equal(1.0, aA!.Value(2.0).X(), 1.0e-12);
            Assert.Equal(2.0, aA.Value(2.0).Y(), 1.0e-12);
        }
        // in range: the curve itself
        Geom2d_Line aIn = new(new gp_Pnt2d(1.0, 0.0), new gp_Dir2d(0, 1));
        BOPTools_AlgoTools2D.AdjustPCurveOnFace(aF, 1.0, 4.0, aIn, out Geom2d_Curve? aSame, new IntTools_Context());
        Assert.Same(aIn, aSame);
        // the 3D-curve overload takes its range
        Geom_TrimmedCurve aC3D = new(new Geom_Line(new gp_Pnt(2, 0, 0), new gp_Dir(0, 0, 1)), 1.0, 4.0);
        Geom2d_Line aOut = new(new gp_Pnt2d(1.0 + 2 * Math.PI, 0.0), new gp_Dir2d(0, 1));
        BOPTools_AlgoTools2D.AdjustPCurveOnFace(aF, aC3D, aOut, out Geom2d_Curve? aA2);
        Assert.Equal(1.0, aA2!.Value(2.0).X(), 1.0e-12);
    }

    [Fact]
    public void AttachExistingPCurve_without_old_pcurve_returns_1()
    {
        TopoDS_Face aF = BOPToolsTestShapes.Faces(BOPToolsTestShapes.Torus())[0];
        TopoDS_Edge anOld = BOPToolsTestShapes.Edges(BOPToolsTestShapes.Cone())[0];
        TopoDS_Edge aNew = TopoDS.Edge(new BRepBuilderAPI_MakeEdge(new gp_Pnt(0, 0, 0), new gp_Pnt(1, 0, 0)).InternalShape());
        Assert.Equal(1, BOPTools_AlgoTools2D.AttachExistingPCurve(anOld, aNew, aF, new IntTools_Context()));
    }

    /// <summary>
    /// OCCT's file-static IsClosed of BOPTools_AlgoTools2D_1.cxx: its loop condition <c>(More() || iCnt == 2)</c>
    /// restarts the exhausted explorer once both occurrences of a seam are counted, so a true seam gives false
    /// (occt-issues.md). The port reproduces it.
    /// </summary>
    [Fact]
    public void IsClosed_of_AlgoTools2D_1_reproduces_OCCTs_restarting_loop()
    {
        TopoDS_Face aF = FaceOfType<Geom_CylindricalSurface>(BOPToolsTestShapes.Cylinder());
        TopoDS_Edge aSeam = BOPToolsTestShapes.Edges(aF).First(e => BRep_Tool.IsClosed(e, aF));
        Assert.True(BRep_Tool.IsClosed(aSeam, aF));
        Assert.Equal(2, BOPToolsTestShapes.Edges(aF).Count(e => e.IsSame(aSeam)));
        Assert.False(BOPTools_AlgoTools2D.IsClosed(aSeam, aF)); // OCCT
        TopoDS_Edge aCircle = BOPToolsTestShapes.Edges(aF).First(e => !BRep_Tool.IsClosed(e, aF));
        Assert.False(BOPTools_AlgoTools2D.IsClosed(aCircle, aF));
    }
}
