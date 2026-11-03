// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using static Kerf.Engine.Intersection.Patches.PatchLineType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

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

    private static SurfaceAdaptor Plane(Point3D theP, Direction3D theN, double theB = 2) =>
        new(new PlaneSurface(theP, theN), -theB, theB, -theB, theB);

    // A bicubic patch over [0, 1] x [0, 1], z = 0.2 + 0.1 x with a bump in the middle.
    private static SurfaceAdaptor Patch(double theDz = 0.0)
    {
        var aPoles = new NCollection_Array2<Point3D>(1, 4, 1, 4);
        for (int i = 1; i <= 4; i++)
        {
            for (int j = 1; j <= 4; j++)
            {
                double x = (i - 1) / 3.0, y = (j - 1) / 3.0;
                double aBump = (i == 2 || i == 3) && (j == 2 || j == 3) ? 0.05 : 0.0;
                aPoles[i, j] = new Point3D(x, y, 0.2 + 0.1 * x + aBump + theDz * (y - 0.5));
            }
        }
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0;
        aKnots[2] = 1;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        return new SurfaceAdaptor(new BSplineSurface(aPoles, aKnots, aKnots, aMults, aMults, 3, 3));
    }

    private static PatchIntersection Run(SurfaceAdaptor theS1, SurfaceAdaptor theS2,
                                             bool thePostProc = true)
    {
        var anInt = new PatchIntersection();
        anInt.SetTolerances(Tol, Tol, 0.05, 0.01);
        anInt.Perform(theS1, new TopolTool(theS1), theS2, new TopolTool(theS2), Tol, Tol,
                      isReqToPostWLProc: thePostProc);
        return anInt;
    }

    // Every point of every walking line on both surfaces at its parameters, and its vertices on the line.
    private static int CheckWalkingLines(PatchIntersection theInt, SurfaceAdaptor theS1,
                                         SurfaceAdaptor theS2, double theTol = 1.0e-5)
    {
        int aNbW = 0;
        for (int i = 1; i <= theInt.LineCount; i++)
        {
            if (theInt.Line(i) is not WLine aWL)
            {
                continue;
            }
            aNbW++;
            Assert.True(aWL.NbPnts() >= 2);
            for (int k = 1; k <= aWL.NbPnts(); k++)
            {
                PntOn2S aP = aWL.Point(k);
                aP.ParametersOnS1(out double u1, out double v1);
                aP.ParametersOnS2(out double u2, out double v2);
                Assert.True(theS1.Value(u1, v1).Distance(aP.Value()) < theTol, $"line {i} point {k} off S1");
                Assert.True(theS2.Value(u2, v2).Distance(aP.Value()) < theTol, $"line {i} point {k} off S2");
            }
            for (int k = 1; k <= aWL.VertexCount; k++)
            {
                PatchPoint aV = aWL.Vertex(k);
                double aPar = aV.ParameterOnLine();
                Assert.InRange(aPar, 1.0, aWL.NbPnts());
            }
        }
        return aNbW;
    }

    [Fact]
    public void Not_done_before_perform()
    {
        var anInt = new PatchIntersection();
        Assert.False(anInt.IsDone);
        Assert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => anInt.IsEmpty);
        Assert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => anInt.LineCount);
        Assert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => anInt.PntCount);
        Assert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => anInt.TangentFaces());
    }

    [Fact]
    public void Two_planes_give_a_line()
    {
        SurfaceAdaptor aP1 = Plane(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        SurfaceAdaptor aP2 = Plane(new Point3D(0, 0, 0.5), new Direction3D(1, 0, 1));
        PatchIntersection anInt = Run(aP1, aP2);
        Assert.True(anInt.IsDone);
        Assert.False(anInt.IsEmpty);
        Assert.False(anInt.TangentFaces());
        Assert.Equal(1, anInt.LineCount);
        Assert.Equal(PatchLineType.Lin, anInt.Line(1).ArcType());
        Line3D aL = ((GLine)anInt.Line(1)).Line();
        Assert.True(aL.Direction().IsParallel(new Direction3D(0, 1, 0), 1.0e-12));
        Assert.Equal(0.0, aL.Distance(new Point3D(0.5, 0, 0)), 1.0e-12);
        Assert.Same(anInt.SequenceOfLine().Value(1), anInt.Line(1));
    }

    [Fact]
    public void Coincident_planes_are_tangent_faces()
    {
        SurfaceAdaptor aP1 = Plane(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        SurfaceAdaptor aP2 = Plane(new Point3D(1, 1, 0), new Direction3D(0, 0, -1));
        PatchIntersection anInt = Run(aP1, aP2);
        Assert.True(anInt.IsDone);
        Assert.True(anInt.TangentFaces());
        Assert.True(anInt.OppositeFaces());
    }

    [Fact]
    public void Parallel_planes_are_empty()
    {
        PatchIntersection anInt = Run(Plane(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)),
                                          Plane(new Point3D(0, 0, 1), new Direction3D(0, 0, 1)));
        Assert.True(anInt.IsDone);
        Assert.True(anInt.IsEmpty);
        Assert.Equal(0, anInt.LineCount);
        Assert.Throws<global::Kerf.Engine.Foundation.DomainErrorException>(() => anInt.OppositeFaces());
    }

    [Fact]
    public void Plane_and_cylinder_give_conics()
    {
        var aCyl = new SurfaceAdaptor(new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0),
                                                                                  new Direction3D(0, 0, 1)), 1),
                                           0, 2 * M_PI, -3, 3);
        PatchIntersection anInt = Run(Plane(new Point3D(0, 0, 0), new Direction3D(0, 1, 2), 5), aCyl);
        Assert.True(anInt.IsDone);
        Assert.Equal(1, anInt.LineCount);
        Assert.Equal(PatchLineType.Ellipse, anInt.Line(1).ArcType());

        PatchIntersection aLines = Run(Plane(new Point3D(0.5, 0, 0), new Direction3D(1, 0, 0), 5), aCyl);
        Assert.Equal(2, aLines.LineCount);
        Assert.Equal(PatchLineType.Lin, aLines.Line(1).ArcType());
        Assert.Equal(PatchLineType.Lin, aLines.Line(2).ArcType());
    }

    [Fact]
    public void Plane_and_patch_walk_without_post_processing()
    {
        SurfaceAdaptor aPln = Plane(new Point3D(0, 0, 0.24), new Direction3D(0, 0, 1)), aPatch = Patch();
        PatchIntersection anInt = Run(aPln, aPatch, thePostProc: false);
        Assert.True(anInt.IsDone);
        Assert.False(anInt.IsEmpty);
        Assert.True(CheckWalkingLines(anInt, aPln, aPatch) >= 1);

        // Either order: the quadric is found on either side.
        PatchIntersection aRev = Run(aPatch, aPln, thePostProc: false);
        Assert.True(aRev.IsDone);
        Assert.True(CheckWalkingLines(aRev, aPatch, aPln) >= 1);
    }

    [Fact]
    public void Two_patches_walk_without_post_processing()
    {
        SurfaceAdaptor aP1 = Patch(), aP2 = Patch(0.3);
        PatchIntersection anInt = Run(aP1, aP2, thePostProc: false);
        Assert.True(anInt.IsDone);
        Assert.False(anInt.IsEmpty);
        Assert.True(CheckWalkingLines(anInt, aP1, aP2) >= 1);
        for (int i = 1; i <= anInt.LineCount; i++)
        {
            Assert.Equal(PatchLineType.Walking, anInt.Line(i).ArcType());
        }
    }

    [Fact]
    public void Analytic_pairs_go_to_the_parametric_intersector_when_not_geomint()
    {
        // isGeomInt false: two planes through IntPatch_PrmPrmIntersection, a walking line.
        SurfaceAdaptor aP1 = Plane(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), 1);
        SurfaceAdaptor aP2 = Plane(new Point3D(0, 0, 0), new Direction3D(1, 0, 1), 1);
        var anInt = new PatchIntersection();
        anInt.Perform(aP1, new TopolTool(aP1), aP2, new TopolTool(aP2), Tol, Tol,
                      isGeomInt: false, isReqToPostWLProc: false);
        Assert.True(anInt.IsDone);
        Assert.True(CheckWalkingLines(anInt, aP1, aP2) >= 1);
    }

    [Fact]
    public void Quasi_degenerate_cone_is_parametric_on_finite_domains()
    {
        // A cone of semi-angle 0.01 cut by a plane across its axis: treated as Bezier (TreatAsBiParametric), so
        // plane/cone goes through ImpPrm and gives walking lines, not a conic.
        var aCone = new SurfaceAdaptor(new ConicalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)),
                                                                    0.01, 1), 0, 2 * M_PI, -2, 2);
        SurfaceAdaptor aPln = Plane(new Point3D(0, 0, 0.5), new Direction3D(1, 0, 0), 3);
        PatchIntersection anInt = Run(aPln, aCone, thePostProc: false);
        Assert.True(anInt.IsDone);
        Assert.False(anInt.IsEmpty);
        Assert.True(CheckWalkingLines(anInt, aPln, aCone) >= 1);

        // A plane normal to the cone's axis keeps it analytic: a circle.
        PatchIntersection aCircle = Run(Plane(new Point3D(0, 0, 0.5), new Direction3D(0, 0, 1), 3), aCone);
        Assert.Equal(1, aCircle.LineCount);
        Assert.Equal(PatchLineType.Circle, aCircle.Line(1).ArcType());
    }

    [Fact]
    public void Self_intersection_of_quadrics_is_empty()
    {
        SurfaceAdaptor aPln = Plane(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        var anInt = new PatchIntersection(aPln, new TopolTool(aPln), Tol, Tol);
        Assert.True(anInt.IsDone);
        Assert.True(anInt.IsEmpty);
        Assert.Equal(0, anInt.LineCount);
    }

    [Fact]
    public void Self_intersection_of_a_flat_patch_is_done()
    {
        SurfaceAdaptor aPatch = Patch();
        var anInt = new PatchIntersection(aPatch, new TopolTool(aPatch), Tol, Tol);
        Assert.True(anInt.IsDone);
        Assert.Equal(0, anInt.LineCount);
    }

    [Fact]
    public void Start_point_on_two_patches()
    {
        SurfaceAdaptor aP1 = Patch(), aP2 = Patch(0.3);
        var anInt = new PatchIntersection();
        anInt.SetTolerances(Tol, Tol, 0.05, 0.01);
        // A point of the intersection: both patches meet where y = 0.5.
        anInt.Perform(aP1, new TopolTool(aP1), aP2, new TopolTool(aP2), 0.5, 0.5, 0.5, 0.5, Tol,
                      Tol);
        Assert.True(anInt.IsDone);
        Assert.True(CheckWalkingLines(anInt, aP1, aP2) >= 1);
    }

    [Fact]
    public void Start_point_with_a_plane()
    {
        SurfaceAdaptor aPln = Plane(new Point3D(0, 0, 0.24), new Direction3D(0, 0, 1)), aPatch = Patch();
        var anInt = new PatchIntersection();
        anInt.SetTolerances(Tol, Tol, 0.05, 0.01);
        anInt.Perform(aPln, new TopolTool(aPln), aPatch, new TopolTool(aPatch), 0.0, 0.0, 0.5,
                      0.5, Tol, Tol);
        Assert.True(anInt.IsDone);
        Assert.True(CheckWalkingLines(anInt, aPln, aPatch) >= 1);
    }

    [Fact]
    public void Plane_and_patch_purged()
    {
        SurfaceAdaptor aPln = Plane(new Point3D(0, 0, 0.24), new Direction3D(0, 0, 1)), aPatch = Patch();
        PatchIntersection anInt = Run(aPln, aPatch);
        Assert.True(anInt.IsDone);
        Assert.True(CheckWalkingLines(anInt, aPln, aPatch) >= 1);
    }

    [Fact]
    public void Sphere_and_cylinder_analytic_lines_become_walking_lines()
    {
        var aCyl = new SurfaceAdaptor(new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0),
                                                                                  new Direction3D(0, 0, 1)), 1),
                                           0, 2 * M_PI, -3, 3);
        var aSph = new SurfaceAdaptor(new SphericalSurface(new CoordinateSystem(new Point3D(1, 0, 0),
                                                                                new Direction3D(0, 0, 1)), 2));
        PatchIntersection anInt = Run(aCyl, aSph);
        Assert.True(anInt.IsDone);
        Assert.True(CheckWalkingLines(anInt, aCyl, aSph) >= 1);
        for (int i = 1; i <= anInt.LineCount; i++)
        {
            Assert.Equal(PatchLineType.Walking, anInt.Line(i).ArcType());
            Assert.False(((WLine)anInt.Line(i)).IsPurgingAllowed);
        }
    }

    [Fact]
    public void Two_cylinders_walking_lines_joined()
    {
        var aC1 = new SurfaceAdaptor(new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0),
                                                                                 new Direction3D(0, 0, 1)), 1),
                                          0, 2 * M_PI, -3, 3);
        var aC2 = new SurfaceAdaptor(new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0),
                                                                                 new Direction3D(1, 0, 0)), 0.8),
                                          0, 2 * M_PI, -3, 3);
        PatchIntersection anInt = Run(aC1, aC2);
        Assert.True(anInt.IsDone);
        Assert.True(CheckWalkingLines(anInt, aC1, aC2) >= 1);
    }

    [Fact]
    public void CheckSingularPoints_finds_the_pole_near_a_plane()
    {
        var aSph = new SurfaceAdaptor(new SphericalSurface(new CoordinateSystem(new Point3D(0, 0, 0),
                                                                                new Direction3D(0, 0, 1)), 1));
        var aD = new TopolTool(aSph);
        SurfaceAdaptor aNear = Plane(new Point3D(0, 0, 1 + 5.0e-6), new Direction3D(0, 0, 1));
        Assert.True(PatchIntersection.CheckSingularPoints(aSph, aD, aNear, out double aDist));
        Assert.Equal(5.0e-6, aDist, 1.0e-9);
        Assert.Equal(0.0001, PatchIntersection.DefineUVMaxStep(aSph, aD, aNear, new TopolTool(aNear)));

        SurfaceAdaptor aFar = Plane(new Point3D(0, 0, 2), new Direction3D(0, 0, 1));
        Assert.Equal(0.001, PatchIntersection.DefineUVMaxStep(aSph, aD, aFar, new TopolTool(aFar)));

        // The same surface: not singular, infinite distance.
        Assert.False(PatchIntersection.CheckSingularPoints(aSph, aD, aSph, out double aSame));
        Assert.True(global::Kerf.Engine.Foundation.Precision.IsInfinite(aSame));

        // A plane has no singular boundary.
        Assert.False(PatchIntersection.CheckSingularPoints(aFar, new TopolTool(aFar), aSph, out _));
    }

    [Fact]
    public void PrepareSurfaces_splits_a_flat_cone_at_its_apex()
    {
        var aCone = new SurfaceAdaptor(new ConicalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)),
                                                                    M_PI / 2 - 1.0e-9, 0), 0, 2 * M_PI, -1, 1);
        SurfaceAdaptor aPln = Plane(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        var aV1 = new NCollection_DynamicArray<SurfaceAdaptorBase>();
        var aV2 = new NCollection_DynamicArray<SurfaceAdaptorBase>();
        PatchIntersection.PrepareSurfaces(aCone, new TopolTool(aCone), aPln, new TopolTool(aPln),
                                              1.0e-7, aV1, aV2);
        Assert.Equal(2, aV1.Length());
        Assert.Equal(-1.0, aV1[0].FirstVParameter());
        Assert.Equal(0.0, aV1[0].LastVParameter(), 1.0e-12);
        Assert.Equal(0.0, aV1[1].FirstVParameter(), 1.0e-12);
        Assert.Equal(1.0, aV1[1].LastVParameter());
        Assert.Equal(1, aV2.Length());
        Assert.Same(aPln, aV2[0]);

        // A cone of ordinary angle is kept whole.
        var aCone2 = new SurfaceAdaptor(new ConicalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)),
                                                                     M_PI / 6, 0), 0, 2 * M_PI, -1, 1);
        var aV3 = new NCollection_DynamicArray<SurfaceAdaptorBase>();
        var aV4 = new NCollection_DynamicArray<SurfaceAdaptorBase>();
        PatchIntersection.PrepareSurfaces(aCone2, new TopolTool(aCone2), aPln,
                                              new TopolTool(aPln), 1.0e-7, aV3, aV4);
        Assert.Equal(1, aV3.Length());
        Assert.Same(aCone2, aV3[0]);
    }

    [Fact]
    public void Same_lines_every_run_and_in_parallel()
    {
        static string Bits()
        {
            SurfaceAdaptor aP1 = Patch(), aP2 = Patch(0.3);
            PatchIntersection anInt = Run(aP1, aP2, thePostProc: false);
            var aSb = new System.Text.StringBuilder();
            for (int i = 1; i <= anInt.LineCount; i++)
            {
                var aWL = (WLine)anInt.Line(i);
                for (int k = 1; k <= aWL.NbPnts(); k++)
                {
                    PntOn2S aP = aWL.Point(k);
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
