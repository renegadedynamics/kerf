// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntPatch_IType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// IntPatch_Intersection's dispatch: analytic pairs (GLines), a quadric and a parametric surface, parametric pairs,
/// quasi-degenerate cones treated as parametric, the self-intersection, the start-point mode, and its static tools
/// (CheckSingularPoints, DefineUVMaxStep, PrepareSurfaces). Every walking line's points lie on both surfaces at
/// their parameters.
/// </summary>
/// <remarks>
/// The lines are compared with native OCCT through GeomAPI_IntSS by unit 5G's oracle. Tests reaching
/// IntPatch_WLineTool (the purging of walking lines, ALines' extension, cylinders' joining) wait for it.
/// </remarks>
public class IntPatch_IntersectionTests
{
    private const double Tol = 1.0e-7;

    private static GeomAdaptor_Surface Plane(gp_Pnt theP, gp_Dir theN, double theB = 2) =>
        new(new Geom_Plane(theP, theN), -theB, theB, -theB, theB);

    // A bicubic patch over [0, 1] x [0, 1], z = 0.2 + 0.1 x with a bump in the middle.
    private static GeomAdaptor_Surface Patch(double theDz = 0.0)
    {
        var aPoles = new NCollection_Array2<gp_Pnt>(1, 4, 1, 4);
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                double x = (i - 1) / 3.0, y = (j - 1) / 3.0;
                double aBump = (i == 2 || i == 3) && (j == 2 || j == 3) ? 0.05 : 0.0;
                aPoles[i, j] = new gp_Pnt(x, y, 0.2 + 0.1 * x + aBump + theDz * (y - 0.5));
            }
        }
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0;
        aKnots[2] = 1;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        return new GeomAdaptor_Surface(new Geom_BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 3, 3));
    }

    private static IntPatch_Intersection Run(GeomAdaptor_Surface theS1, GeomAdaptor_Surface theS2,
                                             bool thePostProc = true)
    {
        var anInt = new IntPatch_Intersection();
        anInt.SetTolerances(Tol, Tol, 0.05, 0.01);
        anInt.Perform(theS1, new Adaptor3d_TopolTool(theS1), theS2, new Adaptor3d_TopolTool(theS2), Tol, Tol,
                      theIsReqToPostWLProc: thePostProc);
        return anInt;
    }

    // Every point of every walking line on both surfaces at its parameters, and its vertices on the line.
    private static int CheckWalkingLines(IntPatch_Intersection theInt, GeomAdaptor_Surface theS1,
                                         GeomAdaptor_Surface theS2, double theTol = 1.0e-5)
    {
        int aNbW = 0;
        for (int i = 1; i <= theInt.NbLines(); i++)
        {
            if (theInt.Line(i) is not IntPatch_WLine aWL)
            {
                continue;
            }
            aNbW++;
            Assert.True(aWL.NbPnts() >= 2);
            for (int k = 1; k <= aWL.NbPnts(); k++)
            {
                IntSurf_PntOn2S aP = aWL.Point(k);
                aP.ParametersOnS1(out double u1, out double v1);
                aP.ParametersOnS2(out double u2, out double v2);
                Assert.True(theS1.Value(u1, v1).Distance(aP.Value()) < theTol, $"line {i} point {k} off S1");
                Assert.True(theS2.Value(u2, v2).Distance(aP.Value()) < theTol, $"line {i} point {k} off S2");
            }
            for (int k = 1; k <= aWL.NbVertex(); k++)
            {
                IntPatch_Point aV = aWL.Vertex(k);
                double aPar = aV.ParameterOnLine();
                Assert.InRange(aPar, 1.0, aWL.NbPnts());
            }
        }
        return aNbW;
    }

    [Fact]
    public void Not_done_before_perform()
    {
        var anInt = new IntPatch_Intersection();
        Assert.False(anInt.IsDone());
        Assert.Throws<StdFail_NotDone>(() => anInt.IsEmpty());
        Assert.Throws<StdFail_NotDone>(() => anInt.NbLines());
        Assert.Throws<StdFail_NotDone>(() => anInt.NbPnts());
        Assert.Throws<StdFail_NotDone>(() => anInt.TangentFaces());
    }

    [Fact]
    public void Two_planes_give_a_line()
    {
        GeomAdaptor_Surface aP1 = Plane(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        GeomAdaptor_Surface aP2 = Plane(new gp_Pnt(0, 0, 0.5), new gp_Dir(1, 0, 1));
        IntPatch_Intersection anInt = Run(aP1, aP2);
        Assert.True(anInt.IsDone());
        Assert.False(anInt.IsEmpty());
        Assert.False(anInt.TangentFaces());
        Assert.Equal(1, anInt.NbLines());
        Assert.Equal(IntPatch_Lin, anInt.Line(1).ArcType());
        gp_Lin aL = ((IntPatch_GLine)anInt.Line(1)).Line();
        Assert.True(aL.Direction().IsParallel(new gp_Dir(0, 1, 0), 1.0e-12));
        Assert.Equal(0.0, aL.Distance(new gp_Pnt(0.5, 0, 0)), 1.0e-12);
        Assert.Same(anInt.SequenceOfLine().Value(1), anInt.Line(1));
    }

    [Fact]
    public void Coincident_planes_are_tangent_faces()
    {
        GeomAdaptor_Surface aP1 = Plane(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        GeomAdaptor_Surface aP2 = Plane(new gp_Pnt(1, 1, 0), new gp_Dir(0, 0, -1));
        IntPatch_Intersection anInt = Run(aP1, aP2);
        Assert.True(anInt.IsDone());
        Assert.True(anInt.TangentFaces());
        Assert.True(anInt.OppositeFaces());
    }

    [Fact]
    public void Parallel_planes_are_empty()
    {
        IntPatch_Intersection anInt = Run(Plane(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)),
                                          Plane(new gp_Pnt(0, 0, 1), new gp_Dir(0, 0, 1)));
        Assert.True(anInt.IsDone());
        Assert.True(anInt.IsEmpty());
        Assert.Equal(0, anInt.NbLines());
        Assert.Throws<Standard_DomainError>(() => anInt.OppositeFaces());
    }

    [Fact]
    public void Plane_and_cylinder_give_conics()
    {
        var aCyl = new GeomAdaptor_Surface(new Geom_CylindricalSurface(new gp_Ax3(new gp_Pnt(0, 0, 0),
                                                                                  new gp_Dir(0, 0, 1)), 1),
                                           0, 2 * M_PI, -3, 3);
        IntPatch_Intersection anInt = Run(Plane(new gp_Pnt(0, 0, 0), new gp_Dir(0, 1, 2), 5), aCyl);
        Assert.True(anInt.IsDone());
        Assert.Equal(1, anInt.NbLines());
        Assert.Equal(IntPatch_Ellipse, anInt.Line(1).ArcType());

        IntPatch_Intersection aLines = Run(Plane(new gp_Pnt(0.5, 0, 0), new gp_Dir(1, 0, 0), 5), aCyl);
        Assert.Equal(2, aLines.NbLines());
        Assert.Equal(IntPatch_Lin, aLines.Line(1).ArcType());
        Assert.Equal(IntPatch_Lin, aLines.Line(2).ArcType());
    }

    [Fact]
    public void Plane_and_patch_walk_without_post_processing()
    {
        GeomAdaptor_Surface aPln = Plane(new gp_Pnt(0, 0, 0.24), new gp_Dir(0, 0, 1)), aPatch = Patch();
        IntPatch_Intersection anInt = Run(aPln, aPatch, thePostProc: false);
        Assert.True(anInt.IsDone());
        Assert.False(anInt.IsEmpty());
        Assert.True(CheckWalkingLines(anInt, aPln, aPatch) >= 1);

        // Either order: the quadric is found on either side.
        IntPatch_Intersection aRev = Run(aPatch, aPln, thePostProc: false);
        Assert.True(aRev.IsDone());
        Assert.True(CheckWalkingLines(aRev, aPatch, aPln) >= 1);
    }

    [Fact]
    public void Two_patches_walk_without_post_processing()
    {
        GeomAdaptor_Surface aP1 = Patch(), aP2 = Patch(0.3);
        IntPatch_Intersection anInt = Run(aP1, aP2, thePostProc: false);
        Assert.True(anInt.IsDone());
        Assert.False(anInt.IsEmpty());
        Assert.True(CheckWalkingLines(anInt, aP1, aP2) >= 1);
        for (int i = 1; i <= anInt.NbLines(); i++)
        {
            Assert.Equal(IntPatch_Walking, anInt.Line(i).ArcType());
        }
    }

    [Fact]
    public void Analytic_pairs_go_to_the_parametric_intersector_when_not_geomint()
    {
        // isGeomInt false: two planes through IntPatch_PrmPrmIntersection, a walking line.
        GeomAdaptor_Surface aP1 = Plane(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), 1);
        GeomAdaptor_Surface aP2 = Plane(new gp_Pnt(0, 0, 0), new gp_Dir(1, 0, 1), 1);
        var anInt = new IntPatch_Intersection();
        anInt.Perform(aP1, new Adaptor3d_TopolTool(aP1), aP2, new Adaptor3d_TopolTool(aP2), Tol, Tol,
                      isGeomInt: false, theIsReqToPostWLProc: false);
        Assert.True(anInt.IsDone());
        Assert.True(CheckWalkingLines(anInt, aP1, aP2) >= 1);
    }

    [Fact]
    public void Quasi_degenerate_cone_is_parametric_on_finite_domains()
    {
        // A cone of semi-angle 0.01 cut by a plane across its axis: treated as Bezier (TreatAsBiParametric), so
        // plane/cone goes through ImpPrm and gives walking lines, not a conic.
        var aCone = new GeomAdaptor_Surface(new Geom_ConicalSurface(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)),
                                                                    0.01, 1), 0, 2 * M_PI, -2, 2);
        GeomAdaptor_Surface aPln = Plane(new gp_Pnt(0, 0, 0.5), new gp_Dir(1, 0, 0), 3);
        IntPatch_Intersection anInt = Run(aPln, aCone, thePostProc: false);
        Assert.True(anInt.IsDone());
        Assert.False(anInt.IsEmpty());
        Assert.True(CheckWalkingLines(anInt, aPln, aCone) >= 1);

        // A plane normal to the cone's axis keeps it analytic: a circle.
        IntPatch_Intersection aCircle = Run(Plane(new gp_Pnt(0, 0, 0.5), new gp_Dir(0, 0, 1), 3), aCone);
        Assert.Equal(1, aCircle.NbLines());
        Assert.Equal(IntPatch_Circle, aCircle.Line(1).ArcType());
    }

    [Fact]
    public void Self_intersection_of_quadrics_is_empty()
    {
        GeomAdaptor_Surface aPln = Plane(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        var anInt = new IntPatch_Intersection(aPln, new Adaptor3d_TopolTool(aPln), Tol, Tol);
        Assert.True(anInt.IsDone());
        Assert.True(anInt.IsEmpty());
        Assert.Equal(0, anInt.NbLines());
    }

    [Fact]
    public void Self_intersection_of_a_flat_patch_is_done()
    {
        GeomAdaptor_Surface aPatch = Patch();
        var anInt = new IntPatch_Intersection(aPatch, new Adaptor3d_TopolTool(aPatch), Tol, Tol);
        Assert.True(anInt.IsDone());
        Assert.Equal(0, anInt.NbLines());
    }

    [Fact]
    public void Start_point_on_two_patches()
    {
        GeomAdaptor_Surface aP1 = Patch(), aP2 = Patch(0.3);
        var anInt = new IntPatch_Intersection();
        anInt.SetTolerances(Tol, Tol, 0.05, 0.01);
        // A point of the intersection: both patches meet where y = 0.5.
        anInt.Perform(aP1, new Adaptor3d_TopolTool(aP1), aP2, new Adaptor3d_TopolTool(aP2), 0.5, 0.5, 0.5, 0.5, Tol,
                      Tol);
        Assert.True(anInt.IsDone());
        Assert.True(CheckWalkingLines(anInt, aP1, aP2) >= 1);
    }

    [Fact]
    public void Start_point_with_a_plane()
    {
        GeomAdaptor_Surface aPln = Plane(new gp_Pnt(0, 0, 0.24), new gp_Dir(0, 0, 1)), aPatch = Patch();
        var anInt = new IntPatch_Intersection();
        anInt.SetTolerances(Tol, Tol, 0.05, 0.01);
        anInt.Perform(aPln, new Adaptor3d_TopolTool(aPln), aPatch, new Adaptor3d_TopolTool(aPatch), 0.0, 0.0, 0.5,
                      0.5, Tol, Tol);
        Assert.True(anInt.IsDone());
        Assert.True(CheckWalkingLines(anInt, aPln, aPatch) >= 1);
    }

    [Fact]
    public void Plane_and_patch_purged()
    {
        GeomAdaptor_Surface aPln = Plane(new gp_Pnt(0, 0, 0.24), new gp_Dir(0, 0, 1)), aPatch = Patch();
        IntPatch_Intersection anInt = Run(aPln, aPatch);
        Assert.True(anInt.IsDone());
        Assert.True(CheckWalkingLines(anInt, aPln, aPatch) >= 1);
    }

    [Fact]
    public void Sphere_and_cylinder_analytic_lines_become_walking_lines()
    {
        var aCyl = new GeomAdaptor_Surface(new Geom_CylindricalSurface(new gp_Ax3(new gp_Pnt(0, 0, 0),
                                                                                  new gp_Dir(0, 0, 1)), 1),
                                           0, 2 * M_PI, -3, 3);
        var aSph = new GeomAdaptor_Surface(new Geom_SphericalSurface(new gp_Ax3(new gp_Pnt(1, 0, 0),
                                                                                new gp_Dir(0, 0, 1)), 2));
        IntPatch_Intersection anInt = Run(aCyl, aSph);
        Assert.True(anInt.IsDone());
        Assert.True(CheckWalkingLines(anInt, aCyl, aSph) >= 1);
        for (int i = 1; i <= anInt.NbLines(); i++)
        {
            Assert.Equal(IntPatch_Walking, anInt.Line(i).ArcType());
            Assert.False(((IntPatch_WLine)anInt.Line(i)).IsPurgingAllowed());
        }
    }

    [Fact]
    public void Two_cylinders_walking_lines_joined()
    {
        var aC1 = new GeomAdaptor_Surface(new Geom_CylindricalSurface(new gp_Ax3(new gp_Pnt(0, 0, 0),
                                                                                 new gp_Dir(0, 0, 1)), 1),
                                          0, 2 * M_PI, -3, 3);
        var aC2 = new GeomAdaptor_Surface(new Geom_CylindricalSurface(new gp_Ax3(new gp_Pnt(0, 0, 0),
                                                                                 new gp_Dir(1, 0, 0)), 0.8),
                                          0, 2 * M_PI, -3, 3);
        IntPatch_Intersection anInt = Run(aC1, aC2);
        Assert.True(anInt.IsDone());
        Assert.True(CheckWalkingLines(anInt, aC1, aC2) >= 1);
    }

    [Fact]
    public void CheckSingularPoints_finds_the_pole_near_a_plane()
    {
        var aSph = new GeomAdaptor_Surface(new Geom_SphericalSurface(new gp_Ax3(new gp_Pnt(0, 0, 0),
                                                                                new gp_Dir(0, 0, 1)), 1));
        var aD = new Adaptor3d_TopolTool(aSph);
        GeomAdaptor_Surface aNear = Plane(new gp_Pnt(0, 0, 1 + 5.0e-6), new gp_Dir(0, 0, 1));
        Assert.True(IntPatch_Intersection.CheckSingularPoints(aSph, aD, aNear, out double aDist));
        Assert.Equal(5.0e-6, aDist, 1.0e-9);
        Assert.Equal(0.0001, IntPatch_Intersection.DefineUVMaxStep(aSph, aD, aNear, new Adaptor3d_TopolTool(aNear)));

        GeomAdaptor_Surface aFar = Plane(new gp_Pnt(0, 0, 2), new gp_Dir(0, 0, 1));
        Assert.Equal(0.001, IntPatch_Intersection.DefineUVMaxStep(aSph, aD, aFar, new Adaptor3d_TopolTool(aFar)));

        // The same surface: not singular, infinite distance.
        Assert.False(IntPatch_Intersection.CheckSingularPoints(aSph, aD, aSph, out double aSame));
        Assert.True(Precision.IsInfinite(aSame));

        // A plane has no singular boundary.
        Assert.False(IntPatch_Intersection.CheckSingularPoints(aFar, new Adaptor3d_TopolTool(aFar), aSph, out _));
    }

    [Fact]
    public void PrepareSurfaces_splits_a_flat_cone_at_its_apex()
    {
        var aCone = new GeomAdaptor_Surface(new Geom_ConicalSurface(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)),
                                                                    M_PI / 2 - 1.0e-9, 0), 0, 2 * M_PI, -1, 1);
        GeomAdaptor_Surface aPln = Plane(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        var aV1 = new NCollection_DynamicArray<Adaptor3d_Surface>();
        var aV2 = new NCollection_DynamicArray<Adaptor3d_Surface>();
        IntPatch_Intersection.PrepareSurfaces(aCone, new Adaptor3d_TopolTool(aCone), aPln, new Adaptor3d_TopolTool(aPln),
                                              1.0e-7, aV1, aV2);
        Assert.Equal(2, aV1.Length());
        Assert.Equal(-1.0, aV1[0].FirstVParameter());
        Assert.Equal(0.0, aV1[0].LastVParameter(), 1.0e-12);
        Assert.Equal(0.0, aV1[1].FirstVParameter(), 1.0e-12);
        Assert.Equal(1.0, aV1[1].LastVParameter());
        Assert.Equal(1, aV2.Length());
        Assert.Same(aPln, aV2[0]);

        // A cone of ordinary angle is kept whole.
        var aCone2 = new GeomAdaptor_Surface(new Geom_ConicalSurface(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1)),
                                                                     M_PI / 6, 0), 0, 2 * M_PI, -1, 1);
        var aV3 = new NCollection_DynamicArray<Adaptor3d_Surface>();
        var aV4 = new NCollection_DynamicArray<Adaptor3d_Surface>();
        IntPatch_Intersection.PrepareSurfaces(aCone2, new Adaptor3d_TopolTool(aCone2), aPln,
                                              new Adaptor3d_TopolTool(aPln), 1.0e-7, aV3, aV4);
        Assert.Equal(1, aV3.Length());
        Assert.Same(aCone2, aV3[0]);
    }

    [Fact]
    public void Same_lines_every_run_and_in_parallel()
    {
        static string Bits()
        {
            GeomAdaptor_Surface aP1 = Patch(), aP2 = Patch(0.3);
            IntPatch_Intersection anInt = Run(aP1, aP2, thePostProc: false);
            var aSb = new System.Text.StringBuilder();
            for (int i = 1; i <= anInt.NbLines(); i++)
            {
                var aWL = (IntPatch_WLine)anInt.Line(i);
                for (int k = 1; k <= aWL.NbPnts(); k++)
                {
                    IntSurf_PntOn2S aP = aWL.Point(k);
                    aP.Parameters(out double u1, out double v1, out double u2, out double v2);
                    aSb.Append(BitConverter.DoubleToInt64Bits(u1)).Append(BitConverter.DoubleToInt64Bits(v1))
                       .Append(BitConverter.DoubleToInt64Bits(u2)).Append(BitConverter.DoubleToInt64Bits(v2));
                }
                aSb.Append('|');
            }
            return aSb.ToString();
        }

        string aRef = Bits();
        Assert.Equal(aRef, Bits());
        string[] aPar = new string[4];
        Parallel.For(0, aPar.Length, i => aPar[i] = Bits());
        Assert.All(aPar, s => Assert.Equal(aRef, s));
    }
}
