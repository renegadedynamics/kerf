// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using static Kerf.Engine.Intersection.Patches.PatchLineType;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;
using Status = Kerf.Engine.Intersection.Patches.ImpImpIntersection.IntStatus;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// IntPatch_ImpImpIntersection on every pair of quadrics OCCT treats apart (plane, cylinder, cone, sphere, torus;
/// coaxial, parallel, tangent, through the apex, the same surface) and on the inputs of OCCT's own bug cases from the
/// corpus (tests/lowalgos/intss): the status, the empty and tangent-faces flags, the number, type and transitions of
/// the lines, their conics and vertices, the isolated points. Every line is also checked to lie on both surfaces:
/// the conics between their vertices, the analytic lines over their domain, the walking lines' points with their
/// parameters.
/// </summary>
/// <remarks>
/// The surfaces are GeomAdaptor_Surfaces with Adaptor3d_TopolTools, TolArc = TolTang = 1e-7, as GeomInt_IntSS (the
/// Draw command <c>intersect</c>) runs the intersection. The values follow OCCT's algorithm; the conics are checked
/// bit for bit against native OCCT by the oracle (ImpImpIntersectionOracleTests) and the rest is checked by unit 5G
/// through GeomAPI_IntSS. The corpus's expected curve counts are GeomInt's, after IntPatch_Intersection joins the
/// cylinders' walking lines and the line constructor splits the conics: where they differ from ImpImp's, the test
/// says so.
/// </remarks>
public class IntPatch_ImpImpIntersectionTests
{
    private const double Tol = 1.0e-7;

    private static CoordinateSystem Ax(double x, double y, double z, double dx, double dy, double dz) =>
        new(new Point3D(x, y, z), new Direction3D(dx, dy, dz));

    private static CoordinateSystem Ax(double x, double y, double z, double dx, double dy, double dz, double xx, double xy,
                             double xz) =>
        new(new Point3D(x, y, z), new Direction3D(dx, dy, dz), new Direction3D(xx, xy, xz));

    private static PlaneSurface Pln(CoordinateSystem theAx) => new(theAx);

    private static CylindricalSurface Cyl(CoordinateSystem theAx, double theR) => new(theAx, theR);

    private static ConicalSurface Cone(CoordinateSystem theAx, double theAng, double theR) => new(theAx, theAng, theR);

    private static SphericalSurface Sph(CoordinateSystem theAx, double theR) => new(theAx, theR);

    private static ToroidalSurface Tor(CoordinateSystem theAx, double theR, double ther) => new(theAx, theR, ther);

    private sealed record Result(ImpImpIntersection Inter, SurfaceAdaptor S1, SurfaceAdaptor S2);

    // The intersection as GeomInt_IntSS runs it; bounds {u1, u2, v1, v2} trim a surface.
    private static Result Run(Surface theS1, Surface theS2, double[]? theB1 = null, double[]? theB2 = null,
                              double theTol = Tol, bool theCheck = true)
    {
        SurfaceAdaptor aS1 = theB1 is null ? new(theS1) : new(theS1, theB1[0], theB1[1], theB1[2], theB1[3]);
        SurfaceAdaptor aS2 = theB2 is null ? new(theS2) : new(theS2, theB2[0], theB2[1], theB2[2], theB2[3]);
        var anInter = new ImpImpIntersection(aS1, new TopolTool(aS1), aS2,
                                                      new TopolTool(aS2), theTol, theTol);
        var aResult = new Result(anInter, aS1, aS2);
        if (theCheck && anInter.IsDone)
        {
            CheckOnBoth(aResult);
        }
        return aResult;
    }

    private static SurfaceQuadric Quadric(SurfaceAdaptor theS) => theS.GetType() switch
    {
        SurfaceKind.Plane => new SurfaceQuadric(theS.Plane()),
        SurfaceKind.Cylinder => new SurfaceQuadric(theS.Cylinder()),
        SurfaceKind.Cone => new SurfaceQuadric(theS.Cone()),
        SurfaceKind.Sphere => new SurfaceQuadric(theS.Sphere()),
        _ => new SurfaceQuadric(theS.Torus()),
    };

    private static void OnBoth(Result theR, in Point3D theP, double theTol, string theWhat)
    {
        double aScale = Math.Max(1.0, theP.XYZ().Modulus());
        Assert.True(Math.Abs(Quadric(theR.S1).Distance(theP)) <= theTol * aScale, $"{theWhat} {theP} is off surface 1");
        Assert.True(Math.Abs(Quadric(theR.S2).Distance(theP)) <= theTol * aScale, $"{theWhat} {theP} is off surface 2");
    }

    private static Point3D ConicValue(GLine theL, double theU) => theL.ArcType() switch
    {
        PatchLineType.Lin => global::Kerf.Engine.Geometry.ElCLib.Value(theU, theL.Line()),
        PatchLineType.Circle => global::Kerf.Engine.Geometry.ElCLib.Value(theU, theL.Circle()),
        PatchLineType.Ellipse => global::Kerf.Engine.Geometry.ElCLib.Value(theU, theL.Ellipse()),
        PatchLineType.Parabola => global::Kerf.Engine.Geometry.ElCLib.Value(theU, theL.Parabola()),
        _ => global::Kerf.Engine.Geometry.ElCLib.Value(theU, theL.Hyperbola()),
    };

    // Every line lies on both surfaces; every vertex is on its line at its parameter.
    private static void CheckOnBoth(Result theR)
    {
        ImpImpIntersection anInter = theR.Inter;
        if (anInter.IsEmpty || anInter.TangentFaces())
        {
            return;
        }
        for (int i = 1; i <= anInter.PntCount; i++)
        {
            OnBoth(theR, anInter.Point(i).Value(), 1.0e-6, $"point {i}");
        }
        for (int i = 1; i <= anInter.LineCount; i++)
        {
            switch (anInter.Line(i))
            {
                case GLine aG:
                    {
                        double aMin = -10.0, aMax = 10.0;
                        if (aG.VertexCount > 1)
                        {
                            aMin = double.MaxValue;
                            aMax = double.MinValue;
                        }
                        for (int k = 1; k <= aG.VertexCount; k++)
                        {
                            PatchPoint aV = aG.Vertex(k);
                            OnBoth(theR, aV.Value(), 1.0e-6, $"line {i} vertex {k}");
                            Assert.True(ConicValue(aG, aV.ParameterOnLine()).DistanceTo(aV.Value()) <= 1.0e-6,
                                        $"line {i} vertex {k} is not at its parameter");
                            aMin = Math.Min(aMin, aV.ParameterOnLine());
                            aMax = Math.Max(aMax, aV.ParameterOnLine());
                        }
                        for (int s = 0; s <= 16; s++)
                        {
                            OnBoth(theR, ConicValue(aG, aMin + (aMax - aMin) * s / 16.0), 1.0e-9, $"line {i}");
                        }
                        break;
                    }
                case ALine anA:
                    {
                        double aF = anA.FirstParameter(out _), aL = anA.LastParameter(out _);
                        for (int s = 0; s <= 32; s++)
                        {
                            OnBoth(theR, anA.Value(aF + (aL - aF) * s / 32.0), 1.0e-7, $"analytic line {i}");
                        }
                        for (int k = 1; k <= anA.VertexCount; k++)
                        {
                            OnBoth(theR, anA.Vertex(k).Value(), 1.0e-6, $"analytic line {i} vertex {k}");
                        }
                        break;
                    }
                case WLine aW:
                    for (int k = 1; k <= aW.NbPnts(); k++)
                    {
                        PntOn2S aP = aW.Point(k);
                        OnBoth(theR, aP.Value(), 1.0e-7, $"walking line {i} point {k}");
                        aP.Parameters(out double aU1, out double aV1, out double aU2, out double aV2);
                        Assert.True(theR.S1.Value(aU1, aV1).DistanceTo(aP.Value()) <= 1.0e-7 * Math.Max(1.0, aP.Value().XYZ().Modulus()));
                        Assert.True(theR.S2.Value(aU2, aV2).DistanceTo(aP.Value()) <= 1.0e-7 * Math.Max(1.0, aP.Value().XYZ().Modulus()));
                    }
                    break;
                case RLine aR:
                    for (int k = 1; k <= aR.VertexCount; k++)
                    {
                        OnBoth(theR, aR.Vertex(k).Value(), 1.0e-6, $"restriction {i} vertex {k}");
                    }
                    break;
            }
        }
    }

    private static GLine G(Result theR, int theIndex)
    {
        PatchLine aL = theR.Inter.Line(theIndex);
        return Assert.IsType<GLine>(aL);
    }

    private static void Lines(Result theR, params PatchLineType[] theTypes)
    {
        Assert.Equal(Status.IntStatus_OK, theR.Inter.GetStatus());
        Assert.False(theR.Inter.IsEmpty);
        Assert.False(theR.Inter.TangentFaces());
        Assert.Equal(theTypes, Enumerable.Range(1, theR.Inter.LineCount).Select(i => theR.Inter.Line(i).ArcType()));
    }

    private static void Transitions(PatchLine theL, WalkingTypeTrans theT1, WalkingTypeTrans theT2)
    {
        Assert.Equal(theT1, theL.TransitionOnS1());
        Assert.Equal(theT2, theL.TransitionOnS2());
    }

    private static void Empty(Result theR)
    {
        Assert.Equal(Status.IntStatus_OK, theR.Inter.GetStatus());
        Assert.True(theR.Inter.IsEmpty);
        Assert.Equal(0, theR.Inter.LineCount);
        Assert.Equal(0, theR.Inter.PntCount);
    }

    private static void SameSurfaces(Result theR, bool theOpposite)
    {
        Assert.Equal(Status.IntStatus_OK, theR.Inter.GetStatus());
        Assert.False(theR.Inter.IsEmpty);
        Assert.True(theR.Inter.TangentFaces());
        Assert.Equal(theOpposite, theR.Inter.OppositeFaces());
        Assert.Equal(0, theR.Inter.LineCount);
    }

    private static void Near(double theExpected, double theActual, double theTol = 1.0e-12) =>
        Assert.True(Math.Abs(theExpected - theActual) <= theTol, $"expected {theExpected:R}, got {theActual:R}");

    // ---------------------------------------------------------------- plane/plane

    [Fact]
    public void Plane_plane_crossing_is_a_line_with_opposite_transitions()
    {
        Result aR = Run(Pln(Ax(0, 0, 0, 0, 0, 1)), Pln(Ax(0, 0, 0, 1, 0, 0)));
        Lines(aR, PatchLineType.Lin);
        Transitions(aR.Inter.Line(1), WalkingTypeTrans.In, WalkingTypeTrans.Out);
        Assert.Equal(new Direction3D(0, 1, 0), G(aR, 1).Line().Direction);
        Assert.Equal(0, G(aR, 1).VertexCount); // infinite planes: no boundary
    }

    [Fact]
    public void Plane_plane_same_or_parallel()
    {
        SameSurfaces(Run(Pln(Ax(0, 0, 0, 0, 0, 1)), Pln(Ax(0, 0, 0, 0, 0, -1))), theOpposite: true);
        SameSurfaces(Run(Pln(Ax(0, 0, 0, 0, 0, 1)), Pln(Ax(1, 2, 0, 0, 0, 1))), theOpposite: false);
        Empty(Run(Pln(Ax(0, 0, 0, 0, 0, 1)), Pln(Ax(0, 0, 1, 0, 0, 1))));
        EngineAssert.Throws<global::Kerf.Engine.Foundation.DomainErrorException>(() => Run(Pln(Ax(0, 0, 0, 0, 0, 1)), Pln(Ax(0, 0, 0, 1, 0, 0))).Inter.OppositeFaces());
    }

    // lowalgos/intss/buc60815: the planes z = 0 (origin (1, 0, 0)) and x = 0, trimmed to squares of sides 100 and 150: one line, with a vertex where it crosses each
    // boundary of each square.
    [Fact]
    public void Buc60815_trimmed_planes()
    {
        Result aR = Run(Pln(Ax(1, 0, 0, 0, 0, 1)), Pln(Ax(0, 0, 0, 1, 0, 0)), [-50, 50, -50, 50], [-75, 75, -75, 75]);
        Lines(aR, PatchLineType.Lin);
        GLine aL = G(aR, 1);
        Assert.Equal([-75.0, -50.0, 50.0, 75.0], Enumerable.Range(1, aL.VertexCount).Select(k => aL.Vertex(k).ParameterOnLine()));
        PatchPoint aV = aL.Vertex(2);
        Assert.True(aV.IsOnDomS1);
        Assert.False(aV.IsOnDomS2);
        Assert.True(aL.Vertex(1).IsOnDomS2);
    }

    // ---------------------------------------------------------------- plane/cylinder

    [Fact]
    public void Plane_cylinder_circle_on_the_seam()
    {
        Result aR = Run(Pln(Ax(0, 0, 2, 0, 0, 1)), Cyl(Ax(0, 0, 0, 0, 0, 1, 0, 1, 0), 3));
        Lines(aR, PatchLineType.Circle);
        Transitions(aR.Inter.Line(1), WalkingTypeTrans.In, WalkingTypeTrans.Out);
        Circle3D aC = G(aR, 1).Circle();
        Assert.Equal(new Point3D(0, 0, 2), aC.Location);
        Assert.Equal(3.0, aC.Radius);
        // AdjustToSeam: the circle starts on the cylinder's seam (its X direction)
        Assert.Equal(new Direction3D(0, 1, 0), aC.XAxis.Direction);
        // the two vertices OCCT puts on a closed conic without boundary points: 0 and 2π
        Assert.Equal(2, G(aR, 1).VertexCount);
        Assert.Equal(0.0, G(aR, 1).Vertex(1).ParameterOnLine());
        Assert.Equal(2 * Math.PI, G(aR, 1).Vertex(2).ParameterOnLine());

        // reversed order: same circle, transitions exchanged
        Result aRev = Run(Cyl(Ax(0, 0, 0, 0, 0, 1, 0, 1, 0), 3), Pln(Ax(0, 0, 2, 0, 0, 1)));
        Lines(aRev, PatchLineType.Circle);
        Transitions(aRev.Inter.Line(1), WalkingTypeTrans.Out, WalkingTypeTrans.In);
        Assert.Equal(aC, G(aRev, 1).Circle());
    }

    [Fact]
    public void Plane_cylinder_ellipse_two_lines_tangent_line_and_empty()
    {
        Result anEllipse = Run(Pln(Ax(0, 0, 2, 0, 1, 1)), Cyl(Ax(0, 0, 0, 0, 0, 1), 3));
        Lines(anEllipse, PatchLineType.Ellipse);
        Near(3.0 * Math.Sqrt(2.0), G(anEllipse, 1).Ellipse().MajorRadius);
        Assert.Equal(3.0, G(anEllipse, 1).Ellipse().MinorRadius);

        Result aTwo = Run(Pln(Ax(1, 0, 0, 1, 0, 0)), Cyl(Ax(0, 0, 0, 0, 0, 1), 3));
        Lines(aTwo, PatchLineType.Lin, PatchLineType.Lin);
        Transitions(aTwo.Inter.Line(1), WalkingTypeTrans.Out, WalkingTypeTrans.In);
        Transitions(aTwo.Inter.Line(2), WalkingTypeTrans.In, WalkingTypeTrans.Out);
        Near(-Math.Sqrt(8.0), G(aTwo, 1).Line().Location.Y, 1.0e-15);
        Near(Math.Sqrt(8.0), G(aTwo, 2).Line().Location.Y, 1.0e-15);

        // a tangent plane: one tangent line, the plane "outside" (its normal and the cylinder's agree) and the
        // cylinder "inside" (its axis is behind the plane's normal), as OCCT's IntPCy decides them
        Result aTangent = Run(Pln(Ax(3, 0, 0, 1, 0, 0)), Cyl(Ax(0, 0, 0, 0, 0, 1), 3));
        Lines(aTangent, PatchLineType.Lin);
        PatchLine aL = aTangent.Inter.Line(1);
        Assert.True(aL.IsTangent);
        Assert.Equal(WalkingTypeTrans.Touch, aL.TransitionOnS1());
        Assert.Equal(WalkingSituation.Outside, aL.SituationS1());
        Assert.Equal(WalkingSituation.Inside, aL.SituationS2());

        Empty(Run(Pln(Ax(4, 0, 0, 1, 0, 0)), Cyl(Ax(0, 0, 0, 0, 0, 1), 3)));
    }

    // ---------------------------------------------------------------- plane/sphere

    [Fact]
    public void Plane_sphere_circle_and_tangent_point()
    {
        Result aCircle = Run(Pln(Ax(0, 0, 1, 0, 0, 1)), Sph(Ax(0, 0, 0, 0, 0, 1), 2));
        Lines(aCircle, PatchLineType.Circle);
        Assert.Equal(Math.Sqrt(3.0), G(aCircle, 1).Circle().Radius);

        Result aTilted = Run(Pln(Ax(0, 0, 1, 1, 0, 1)), Sph(Ax(0, 0, 0, 0, 0, 1), 2));
        Lines(aTilted, PatchLineType.Circle);
        Assert.Equal(new Direction3D(1, 0, 1), G(aTilted, 1).Circle().Axis.Direction);

        // touching the north pole: one tangent point, and the pole's degenerated boundary on the plane, a restriction
        Result aPoint = Run(Pln(Ax(0, 0, 2, 0, 0, 1)), Sph(Ax(0, 0, 0, 0, 0, 1), 2));
        Lines(aPoint, PatchLineType.Restriction);
        Assert.Equal(1, aPoint.Inter.PntCount);
        Assert.Equal(new Point3D(0, 0, 2), aPoint.Inter.Point(1).Value());
        Assert.True(aPoint.Inter.Point(1).IsTangencyPoint);

        Empty(Run(Pln(Ax(0, 0, 3, 0, 0, 1)), Sph(Ax(0, 0, 0, 0, 0, 1), 2)));
    }

    // ---------------------------------------------------------------- plane/cone

    [Fact]
    public void Plane_cone_conics()
    {
        ConicalSurface aCone = Cone(Ax(0, 0, 0, 0, 0, 1), Math.PI / 6, 1);
        Result aCircle = Run(Pln(Ax(0, 0, 2, 0, 0, 1)), aCone);
        Lines(aCircle, PatchLineType.Circle);
        Near(1.0 + 2.0 * Math.Tan(Math.PI / 6), G(aCircle, 1).Circle().Radius, 1.0e-15);

        Lines(Run(Pln(Ax(0, 0, 2, 0, 0.2, 1)), aCone), PatchLineType.Ellipse);

        // a plane parallel to the axis: the two branches of a hyperbola
        Result aHyperbola = Run(Pln(Ax(0.5, 0, 0, 1, 0, 0)), aCone);
        Lines(aHyperbola, PatchLineType.Hyperbola, PatchLineType.Hyperbola);
        Transitions(aHyperbola.Inter.Line(1), WalkingTypeTrans.Out, WalkingTypeTrans.In);
        Transitions(aHyperbola.Inter.Line(2), WalkingTypeTrans.Out, WalkingTypeTrans.In);

        // a plane parallel to a generatrix, not through the apex: a parabola
        double aC = Math.Cos(Math.PI / 6), aS = Math.Sin(Math.PI / 6);
        Result aParabola = Run(Pln(Ax(2, 0, 0, aC, 0, -aS)), aCone);
        Lines(aParabola, PatchLineType.Parabola);
    }

    [Fact]
    public void Plane_cone_through_the_apex()
    {
        // two generatrices, each split at the apex into two lines with a multiple vertex there
        Result aLines = Run(Pln(Ax(0, 0, 0, 1, 0, 0)), Cone(Ax(0, 0, -2, 0, 0, 1), Math.PI / 4, 2));
        Lines(aLines, PatchLineType.Lin, PatchLineType.Lin, PatchLineType.Lin, PatchLineType.Lin);
        Point3D anApex = new(0, 0, -4);
        for (int i = 1; i <= 4; i++)
        {
            GLine aL = G(aLines, i);
            Assert.True(aL.Line().Location.DistanceTo(anApex) < 1.0e-15);
            Assert.Contains(Enumerable.Range(1, aL.VertexCount).Select(aL.Vertex),
                            v => v.IsMultiple && v.Value().DistanceTo(anApex) < 1.0e-12);
        }
        Transitions(aLines.Inter.Line(1), WalkingTypeTrans.Out, WalkingTypeTrans.In);
        Transitions(aLines.Inter.Line(3), WalkingTypeTrans.In, WalkingTypeTrans.Out);

        // the tangent plane along a generatrix: two tangent half-lines from the apex
        double aC = Math.Cos(Math.PI / 6), aS = Math.Sin(Math.PI / 6);
        Result aTangent = Run(Pln(Ax(1, 0, 0, aC, 0, -aS)), Cone(Ax(0, 0, 0, 0, 0, 1), Math.PI / 6, 1));
        Lines(aTangent, PatchLineType.Lin, PatchLineType.Lin);
        Assert.True(aTangent.Inter.Line(1).IsTangent);
        Assert.Equal(WalkingTypeTrans.Touch, aTangent.Inter.Line(2).TransitionOnS2());

        // only the apex: an isolated point, and the apex's boundary on the plane
        Result aPoint = Run(Pln(Ax(0, 0, -2, 0, 0, 1)), Cone(Ax(0, 0, 0, 0, 0, 1), Math.PI / 4, 2));
        Assert.Equal(1, aPoint.Inter.PntCount);
        Assert.True(aPoint.Inter.Point(1).Value().DistanceTo(new Point3D(0, 0, -2)) < 1.0e-15);
    }

    // lowalgos/intss/bug20766: plane x = 0 and a cone of radius 0 far from the origin: an ellipse.
    [Fact]
    public void Bug20766_plane_cone_ellipse()
    {
        double aSemiAngle = 0.46689346637921703;
        Result aR = Run(Pln(Ax(0, 0, 0, 1, 0, 0)),
                        Cone(Ax(-3522.4299999999998, 2540, -1461.4300000000001, 5449.7857099692965, 0, 5537.6938973109054),
                             aSemiAngle, 0));
        Lines(aR, PatchLineType.Ellipse);
        Ellipse3D anE = G(aR, 1).Ellipse();
        Assert.True(anE.Location.X < 1.0e-9);
        Assert.Equal(new Direction3D(1, 0, 0), anE.Axis.Direction);
    }

    // ---------------------------------------------------------------- plane/torus

    [Fact]
    public void Plane_torus_circles()
    {
        // the equatorial plane: two circles, outer and inner
        Result aR = Run(Pln(Ax(0, 0, 0, 0, 0, 1)), Tor(Ax(0, 0, 0, 0, 0, 1), 10, 2));
        Lines(aR, PatchLineType.Circle, PatchLineType.Circle);
        Assert.Equal(12.0, G(aR, 1).Circle().Radius);
        Assert.Equal(8.0, G(aR, 2).Circle().Radius);
        Transitions(aR.Inter.Line(1), WalkingTypeTrans.In, WalkingTypeTrans.Out);
        Transitions(aR.Inter.Line(2), WalkingTypeTrans.Out, WalkingTypeTrans.In);
    }

    // lowalgos/intss/bug23177_1: torus 100 10 and the plane y = 0: two circles of radius 10 (GeomInt: 2 curves).
    [Fact]
    public void Bug23177_torus_plane_through_the_axis()
    {
        Result aR = Run(Tor(Ax(0, 0, 0, 0, 0, 1), 100, 10), Pln(Ax(0, 0, 0, 0, 1, 0)));
        Lines(aR, PatchLineType.Circle, PatchLineType.Circle);
        Assert.Equal(new Point3D(-100, 0, 0), G(aR, 1).Circle().Location);
        Assert.Equal(new Point3D(100, 0, 0), G(aR, 2).Circle().Location);
        Assert.All(new[] { G(aR, 1), G(aR, 2) }, l => Assert.Equal(10.0, l.Circle().Radius));
        // meridians: their axis is the plane's normal (IntPTo adjusts to the seam only planes not through the axis)
        Assert.Equal(new Direction3D(0, 1, 0), G(aR, 1).Circle().Axis.Direction);
    }

    // ---------------------------------------------------------------- cylinder/cylinder

    [Fact]
    public void Cylinder_cylinder_geometric_cases()
    {
        SameSurfaces(Run(Cyl(Ax(0, 0, 0, 0, 0, 1), 3), Cyl(Ax(0, 0, 5, 0, 0, -1), 3)), theOpposite: false);

        Result aParallel = Run(Cyl(Ax(0, 0, 0, 0, 0, 1), 3), Cyl(Ax(4, 0, 0, 0, 0, 1), 3));
        Lines(aParallel, PatchLineType.Lin, PatchLineType.Lin);
        Near(Math.Sqrt(5.0), Math.Abs(G(aParallel, 1).Line().Location.Y), 1.0e-15);

        Result aTangent = Run(Cyl(Ax(0, 0, 0, 0, 0, 1), 3), Cyl(Ax(5, 0, 0, 0, 0, 1), 2));
        Lines(aTangent, PatchLineType.Lin);
        Assert.True(aTangent.Inter.Line(1).IsTangent);

        // equal radii, crossing axes: two ellipses
        Result anEllipses = Run(Cyl(Ax(0, 0, 0, 0, 0, 1), 3), Cyl(Ax(0, 0, 0, 1, 0, 0), 3),
                                [0, 2 * Math.PI, -10, 10], [0, 2 * Math.PI, -10, 10]);
        Lines(anEllipses, PatchLineType.Ellipse, PatchLineType.Ellipse);
        Near(3.0 * Math.Sqrt(2.0), G(anEllipses, 1).Ellipse().MajorRadius, 1.0e-14);

        Empty(Run(Cyl(Ax(0, 0, 0, 0, 0, 1), 1), Cyl(Ax(5, 0, 0, 0, 1, 0), 1)));
    }

    // Different radii, crossing axes: no geometric solution, so ImpImp walks (OCCT 7.x's cylinder/cylinder
    // algorithm): walking lines from boundary to boundary, split where U1 is critical.
    [Fact]
    public void Cylinder_cylinder_walking_lines()
    {
        Result aR = Run(Cyl(Ax(0, 0, 0, 0, 0, 1), 3), Cyl(Ax(0, 0, 0, 1, 0, 0), 2),
                        [0, 2 * Math.PI, -10, 10], [0, 2 * Math.PI, -10, 10]);
        Lines(aR, Enumerable.Repeat(PatchLineType.Walking, 8).ToArray());
        for (int i = 1; i <= 8; i++)
        {
            var aW = (WLine)aR.Inter.Line(i);
            Assert.Equal(2, aW.VertexCount);
            Assert.True(aW.NbPnts() > 100);
            Assert.Equal(WalkingTypeTrans.Undecided, aW.TransitionOnS1());
            // each line runs between boundary points of the trimmed cylinders or critical points (|x| = 3 or |y| = 2)
            Point3D aF = aW.Point(1).Value(), aL = aW.Point(aW.NbPnts()).Value();
            Assert.True(Math.Abs(Math.Abs(aF.Y) - 2) < 1.0e-9 || Math.Abs(Math.Abs(aF.X) - 3) < 1.0e-9);
            Assert.True(Math.Abs(Math.Abs(aL.Y) - 2) < 1.0e-9 || Math.Abs(Math.Abs(aL.X) - 3) < 1.0e-9);
        }
    }

    // lowalgos/intss/bug21750: two cylinders of the same radius with opposite parallel axes: two lines (GeomInt: 2).
    [Fact]
    public void Bug21750_opposite_parallel_cylinders()
    {
        Result aR = Run(Cyl(Ax(1031.3339148728076, -113.25868616662650, 56.152387686082520, -0.86602540378443815,
                               0.50000000000000089, 0.0, -0.50000000000000089, -0.86602540378443815, 0.0), 76.499999999999986),
                        Cyl(Ax(1017.0706583606553, -103.24955863669388, 56.152387686082548, 0.86602540378443915,
                               -0.49999999999999933, 0.0, 0.49999999999999933, 0.86602540378443915, 0.0), 76.499999999999986));
        Lines(aR, PatchLineType.Lin, PatchLineType.Lin);
    }

    // lowalgos/intss/bug25782_2: parallel cylinders (axis 12 35 47): two lines (GeomInt: 2).
    [Fact]
    public void Bug25782_parallel_cylinders()
    {
        Result aR = Run(Cyl(Ax(0, 0, 0, 12, 35, 47), 5), Cyl(Ax(3, 2, 8, 12, 35, 47), 4));
        Lines(aR, PatchLineType.Lin, PatchLineType.Lin);
        Assert.Equal(new Direction3D(12, 35, 47), G(aR, 1).Line().Direction);
    }

    // lowalgos/intss/bug23178 and bug28222_2: crossing cylinders of different radii, infinite: eight walking lines,
    // which IntPatch_Intersection joins into GeomInt's six and four curves.
    [Fact]
    public void Bug23178_and_bug28222_crossing_cylinders()
    {
        Result a23178 = Run(Cyl(Ax(4.999999999813, 1.35836143386791, 0.20975890778078, -1, 0, 0, 0, 0, 1), 10),
                            Cyl(Ax(9.999999999813, 9.1401960568287, 0.11837300583107, 0, -0.75870713028692, 0.651431877061437,
                                   0, -0.651431877061437, -0.75870713028692), 5));
        Lines(a23178, Enumerable.Repeat(PatchLineType.Walking, 8).ToArray());

        Result a28222 = Run(Cyl(Ax(-35, 13.3706576198285, 30.5814570420266, 0, -0.258819045102521, -0.965925826289068,
                                   0, 0.965925826289068, -0.258819045102521), 11),
                            Cyl(Ax(0, 0, 0, 1, 0, 0, 0, 0, -1), 16.5));
        Lines(a28222, Enumerable.Repeat(PatchLineType.Walking, 8).ToArray());
    }

    // lowalgos/intss/bug26894: two almost coincident cylinders (the faces of bug26884-f1/f2.brep): the walking line
    // would be too long to compute reliably, so ImpImp returns IntStatus_InfiniteSectionCurve (GeomInt: no curve).
    [Fact]
    public void Bug26894_infinite_section_curve()
    {
        Result aR = Run(Cyl(Ax(3.4116087703040397e-016, 11.187868844028243, -3.0729039973350378, 3.6740185570594554e-017,
                               0.94365664763489343, -0.33092617209050501, -2.2723161366922015e-016, 0.33092617209050501,
                               0.94365664763489343), 0.67500000000080174),
                        Cyl(Ax(3.4116087703044888e-016, 11.187868844026969, -3.0729039973354424, 3.6740185570428899e-017,
                               0.94365664763541668, -0.33092617208901293, -2.2723161366927823e-016, 0.33092617208901293,
                               0.94365664763541668), 0.67499999999999927), theCheck: false);
        Assert.Equal(Status.IntStatus_InfiniteSectionCurve, aR.Inter.GetStatus());
        Assert.True(aR.Inter.IsDone);
        Assert.Equal(0, aR.Inter.LineCount);
    }

    // ---------------------------------------------------------------- cylinder, cone, sphere

    [Fact]
    public void Coaxial_quadrics_meet_in_circles()
    {
        Result aCySp = Run(Cyl(Ax(0, 0, 0, 0, 0, 1), 1), Sph(Ax(0, 0, 0, 0, 0, 1), 2));
        Lines(aCySp, PatchLineType.Circle, PatchLineType.Circle);
        Assert.Equal(Math.Sqrt(3.0), G(aCySp, 1).Circle().Location.Z);
        Transitions(aCySp.Inter.Line(1), WalkingTypeTrans.Out, WalkingTypeTrans.In);

        Result aCyCo = Run(Cyl(Ax(0, 0, 0, 0, 0, 1), 1), Cone(Ax(0, 0, 0, 0, 0, 1), Math.PI / 4, 0.5));
        Lines(aCyCo, PatchLineType.Circle, PatchLineType.Circle);

        Result aCoCo = Run(Cone(Ax(0, 0, 0, 0, 0, 1), Math.PI / 6, 1), Cone(Ax(0, 0, 0, 0, 0, 1), Math.PI / 4, 0.5));
        Lines(aCoCo, PatchLineType.Circle, PatchLineType.Circle);

        Result aCoSp = Run(Cone(Ax(0, 0, 0, 0, 0, 1), Math.PI / 6, 1), Sph(Ax(0, 0, 1, 0, 0, 1), 2));
        Lines(aCoSp, PatchLineType.Circle, PatchLineType.Circle);

        Result aSpSp = Run(Sph(Ax(0, 0, 0, 0, 0, 1), 2), Sph(Ax(2, 0, 0, 0, 0, 1), 2));
        Lines(aSpSp, PatchLineType.Circle);
        Assert.Equal(new Point3D(1, 0, 0), G(aSpSp, 1).Circle().Location);
    }

    [Fact]
    public void Non_coaxial_quadrics_meet_in_analytic_lines()
    {
        Result aCySp = Run(Cyl(Ax(0, 0, 0, 0, 0, 1), 1), Sph(Ax(1, 0, 0, 0, 0, 1), 2));
        Lines(aCySp, PatchLineType.Analytic, PatchLineType.Analytic, PatchLineType.Restriction, PatchLineType.Restriction);
        Transitions(aCySp.Inter.Line(1), WalkingTypeTrans.Out, WalkingTypeTrans.In);

        Result aCyCo = Run(Cyl(Ax(0, 0, 0, 0, 0, 1), 1), Cone(Ax(0.3, 0, 0, 1, 0, 1), Math.PI / 8, 0.5));
        Lines(aCyCo, PatchLineType.Analytic, PatchLineType.Analytic, PatchLineType.Analytic);

        Result aCoSp = Run(Cone(Ax(0, 0, 0, 0, 0, 1), Math.PI / 6, 1), Sph(Ax(0.2, 0, 1, 0, 0, 1), 2));
        Lines(aCoSp, PatchLineType.Analytic, PatchLineType.Analytic);
    }

    // lowalgos/intss/bug29807_i5002: two cones with parallel axes (80° and 5°), tolerance 1e-4: four analytic lines.
    [Fact]
    public void Bug29807_cones_with_parallel_axes()
    {
        Result aR = Run(Cone(Ax(11.4307383137554, 3.49999999999979, -89.7537975119388, 0, 0, 1, 1, 0, 0),
                             80.000725670142287835190342147806 * Math.PI / 180, 9.45659107381736),
                        Cone(Ax(-3.6479413426839, -11.578679656441, -89.9782110643133, 0, 0, 1, 0, 1, 0),
                             5 * Math.PI / 180, 0.250951325477062),
                        theTol: 1.0e-4, theCheck: false);
        Lines(aR, PatchLineType.Analytic, PatchLineType.Analytic, PatchLineType.Analytic, PatchLineType.Analytic);
        for (int i = 1; i <= 4; i++)
        {
            var anA = (ALine)aR.Inter.Line(i);
            double aF = anA.FirstParameter(out _), aL = anA.LastParameter(out _);
            for (int s = 0; s <= 16; s++)
            {
                OnBoth(aR, anA.Value(aF + (aL - aF) * s / 16.0), 1.0e-9, $"line {i}");
            }
        }
    }

    [Fact]
    public void Sphere_sphere_same_tangent_and_empty()
    {
        SameSurfaces(Run(Sph(Ax(0, 0, 0, 0, 0, 1), 2), Sph(Ax(0, 0, 0, 1, 0, 0), 2)), theOpposite: false);

        Result aTangent = Run(Sph(Ax(0, 0, 0, 0, 0, 1), 2), Sph(Ax(4, 0, 0, 0, 0, 1), 2));
        Assert.False(aTangent.Inter.IsEmpty);
        Assert.Equal(0, aTangent.Inter.LineCount);
        Assert.Equal(1, aTangent.Inter.PntCount);
        Assert.Equal(new Point3D(2, 0, 0), aTangent.Inter.Point(1).Value());

        Empty(Run(Sph(Ax(0, 0, 0, 0, 0, 1), 2), Sph(Ax(5, 0, 0, 0, 0, 1), 2)));
    }

    [Fact]
    public void Cones_with_a_common_apex_and_no_common_generatrix_are_empty()
    {
        Empty(Run(Cone(Ax(0, 0, 0, 0, 0, 1), Math.PI / 6, 0), Cone(Ax(0, 0, 0, 1, 0, 0), Math.PI / 6, 0)));
    }

    // ---------------------------------------------------------------- tori

    [Fact]
    public void Coaxial_torus_pairs()
    {
        Result aCyTo = Run(Cyl(Ax(0, 0, 0, 0, 0, 1), 10), Tor(Ax(0, 0, 0, 0, 0, 1), 10, 2));
        Lines(aCyTo, PatchLineType.Circle, PatchLineType.Circle);
        Assert.Equal(2.0, G(aCyTo, 1).Circle().Location.Z);

        Result aSpTo = Run(Sph(Ax(0, 0, 0, 0, 0, 1), 10), Tor(Ax(0, 0, 0, 0, 0, 1), 10, 2));
        Lines(aSpTo, PatchLineType.Circle, PatchLineType.Circle);
        Near(9.8, G(aSpTo, 1).Circle().Radius);

        Result aToTo = Run(Tor(Ax(0, 0, 0, 0, 0, 1), 10, 2), Tor(Ax(0, 0, 1, 0, 0, 1), 10, 2));
        Lines(aToTo, PatchLineType.Circle, PatchLineType.Circle);
        Assert.Equal(0.5, G(aToTo, 1).Circle().Location.Z);

        SameSurfaces(Run(Tor(Ax(0, 0, 0, 0, 0, 1), 10, 2), Tor(Ax(0, 0, 0, 0, 0, 1), 10, 2)), theOpposite: false);
    }

    // lowalgos/intss/bug24648: cone 60° and torus 30 20, both orders: four circles, the same in both orders with the
    // transitions exchanged (GeomInt: circles).
    [Fact]
    public void Bug24648_cone_torus_both_orders()
    {
        Result aCoTo = Run(Cone(Ax(0, 0, 0, 0, 0, 1), 60 * Math.PI / 180, 0), Tor(Ax(0, 0, 0, 0, 0, 1), 30, 20));
        Result aToCo = Run(Tor(Ax(0, 0, 0, 0, 0, 1), 30, 20), Cone(Ax(0, 0, 0, 0, 0, 1), 60 * Math.PI / 180, 0));
        Lines(aCoTo, PatchLineType.Circle, PatchLineType.Circle, PatchLineType.Circle, PatchLineType.Circle);
        Lines(aToCo, PatchLineType.Circle, PatchLineType.Circle, PatchLineType.Circle, PatchLineType.Circle);
        for (int i = 1; i <= 4; i++)
        {
            Assert.Equal(G(aCoTo, i).Circle(), G(aToCo, i).Circle());
            Assert.Equal(aCoTo.Inter.Line(i).TransitionOnS1(), aToCo.Inter.Line(i).TransitionOnS2());
        }
    }

    // lowalgos/intss/bug23163_1: two tori side by side (axes parallel, not coaxial): no geometric solution, so ImpImp
    // fails, and IntPatch_Intersection falls back to the parametric intersection.
    [Fact]
    public void Bug23163_non_coaxial_tori_fail()
    {
        Result aR = Run(Tor(Ax(0, 0, 0, 0, 0, 1), 20, 5), Tor(Ax(40, 0, 0, 0, 0, 1), 20, 5));
        Assert.Equal(Status.IntStatus_Fail, aR.Inter.GetStatus());
        Assert.False(aR.Inter.IsDone);
        EngineAssert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => aR.Inter.LineCount);
    }

    [Fact]
    public void Default_and_unsupported_surfaces()
    {
        var anInter = new ImpImpIntersection();
        Assert.False(anInter.IsDone);
        EngineAssert.Throws<global::Kerf.Engine.Foundation.NotDoneException>(() => anInter.IsEmpty);

        SurfaceAdaptor aBezier = new(new BezierSurface(new NCollection_Array2<Point3D>(1, 2, 1, 2)));
        SurfaceAdaptor aPlane = new(Pln(Ax(0, 0, 0, 0, 0, 1)));
        EngineAssert.Throws<global::Kerf.Engine.Foundation.ConstructionErrorException>(() => new ImpImpIntersection(
            aBezier, new TopolTool(aBezier), aPlane, new TopolTool(aPlane), Tol, Tol));
    }
}
