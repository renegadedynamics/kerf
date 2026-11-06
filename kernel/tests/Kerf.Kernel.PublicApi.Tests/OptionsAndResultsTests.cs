// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Booleans;
using Kerf.Construction;
using Kerf.Drawing;
using Kerf.Exchange;
using Kerf.Fillets;
using Kerf.Geometry;
using Kerf.Healing;
using Kerf.Measurement;
using Kerf.Meshing;
using Kerf.Offsets;
using Kerf.Topology;

namespace Kerf.Kernel.PublicApi.Tests;

/// <summary>
/// The operations' options no other test sets, and the small results and codes no other test reads (question 49): each
/// option is passed to its operation, with a value that keeps the result checkable.
/// </summary>
public class OptionsAndResultsTests
{
    private const double Tol = 1e-6;

    private static Solid Box(double size = 10) => Solid.Box(size, size, size).Value.Shape;

    private static Wire Square(double size, double z = 0) =>
        Wire.Polygon([new(0, 0, z), new(size, 0, z), new(size, size, z), new(0, size, z)], closed: true).Value;

    [Fact]
    public void Every_operation_has_its_failure_code()
    {
        Assert.Equal("boolean.failed", BooleanOperations.Failed.Value);
        Assert.Equal("construction.curves-3d-incomplete", ShapeFactory.Curves3DIncomplete.Value);
        Assert.Equal("construction.not-solid", Sweeps.NotSolid.Value);
        Assert.Equal("construction.helix-failed", Sweeps.HelixFailed.Value);
        Assert.Equal("construction.helix-tolerance", Sweeps.HelixTolerance.Value);
        Assert.Equal("drawing.failed", DrawingOperations.ProjectionFailed.Value);
        Assert.Equal("drawing.faces-not-meshed", DrawingOperations.FacesNotMeshed.Value);
        Assert.Equal("render.failed", Renderer.RenderFailed.Value);
        Assert.Equal("measurement.distance-failed", MeasureOperations.DistanceFailed.Value);
        Assert.Equal("meshing.failed", Mesher.MeshFailed.Value);
        Assert.Equal("meshing.faces-not-meshed", Mesher.FacesNotMeshed.Value);
    }

    [Fact]
    public void A_Boolean_gives_the_same_answer_under_its_options()
    {
        Solid a = Box();
        Solid b = Box().Moved(new Placement(Transform3D.Translation(new Vector3D(5, 0, 0)))).As<Solid>();
        double plain = a.Fuse(b).Value.Shape.IntegrateVolume();
        var options = new BooleanOptions { CheckInverted = false, Parallel = true, UseObb = true };
        Assert.True(options.Parallel && options.UseObb && !options.CheckInverted);
        Assert.Equal(plain, a.Fuse(b, options).Value.Shape.IntegrateVolume(), Tol);
        Assert.Equal(1500, plain, Tol);
    }

    [Fact]
    public void Extruding_lofting_and_sweeping_take_their_options()
    {
        Face square = Face.FromWire(Square(2)).Value;
        Built<Shape> prism = square.Extrude(new Vector3D(0, 0, 3), new ExtrudeOptions { Canonize = false }).Value;
        Assert.Equal(12, prism.Shape.IntegrateVolume(), Tol);

        var loftOptions = new LoftOptions
        {
            CheckCompatibility = true,
            Continuity = Continuity.C2,
            MaxDegree = 8,
            Parametrization = ParametrizationType.ChordLength,
            Smoothing = false,
            Tolerance = 1e-5,
        };
        Built<Shape> loft = Sweeps.Loft([Square(2), Square(2, 3)], loftOptions).Value;
        Assert.Equal(12, loft.Shape.IntegrateVolume(), 1e-3);

        Wire spine = Wire.Polygon([new(1, 1, 0), new(1, 1, 3)]).Value;
        var sweepOptions = new SweepOptions { Tolerance = 1e-5, WithContact = false, Frame = SweepFrame.Binormal, Binormal = Direction3D.UnitX };
        Built<Shape> swept = Sweeps.Sweep(Square(2), spine, sweepOptions).Value;
        Assert.Equal(12, swept.Shape.IntegrateVolume(), 1e-3);

        var helixOptions = new HelixOptions { Continuity = Continuity.C1, MaxDegree = 9, Tolerance = 1e-3 };
        Wire helix = Sweeps.Helix(5, 2, 4, helixOptions).Value;
        Assert.Equal(2 * Math.Sqrt(Math.Pow(2 * Math.PI * 5, 2) + 4), helix.IntegrateLength(), 1e-1);
    }

    [Fact]
    public void A_fill_takes_its_options()
    {
        Face plane = Face.FromWire(Square(10)).Value;
        FillBoundary[] boundaries = [.. plane.AllEdges().Select(e => new FillBoundary(e, Continuity.C0) { IsBoundary = true })];
        var options = new FillOptions
        {
            AngularTolerance = 0.1,
            Anisotropy = false,
            CurvatureTolerance = 0.1,
            Degree = 3,
            InitialSurface = plane,
            Iterations = 2,
            MaxDegree = 8,
            MaxSegments = 9,
            PointsOnCurve = 15,
            Tolerance2D = 1e-5,
            Tolerance3D = 1e-4,
        };
        Assert.Equal(100, Surfaces.Fill(boundaries, options).Value.Shape.IntegrateArea(), 1e-2);
    }

    [Fact]
    public void Fillets_and_chamfers_take_their_options()
    {
        Solid box = Box();
        Edge edge = box.AllEdges()[0];
        var tolerances = new FilletTolerances
        {
            Angular = 0.05,
            Approximation2D = 1e-5,
            Approximation3D = 1e-4,
            Deflection = 1e-3,
            Parametric = 1e-5,
            Spine = 1e-4,
        };
        var options = new FilletOptions
        {
            AngularTolerance = 0.05,
            InternalContinuity = Continuity.C1,
            Shape = FilletShape.Rational,
            Tolerances = tolerances,
        };
        double plain = box.Fillet([edge], 1).Value.Shape.IntegrateVolume();
        Assert.Equal(plain, box.Fillet([edge], 1, options).Value.Shape.IntegrateVolume(), 1e-3);
        Assert.Equal(1000 - (1 - Math.PI / 4) * 10, plain, 1e-3);

        Built<Shape> chamfer = box.Chamfer([edge], 1, new ChamferOptions { Mode = ChamferMode.Classic }).Value;
        Assert.Equal(1000 - 0.5 * 10, chamfer.Shape.IntegrateVolume(), Tol);
    }

    [Fact]
    public void Healing_takes_its_options()
    {
        Solid box = Box();
        var fix = new FixOptions { FixFaces = true, FixSameParameter = true, FixWires = true, MinTolerance = 1e-7, Precision = 1e-6 };
        Assert.Equal(1000, box.Fix(fix).Value.Shape.IntegrateVolume(), Tol);

        var sew = new SewOptions { MaxTolerance = 1e-2, MinTolerance = 1e-5, NonManifold = false };
        Built<Shape> sewn = HealingOperations.Sew(box.AllFaces().Select(f => (Shape)f), sew).Value;
        Assert.Equal(6, sewn.Shape.AllFaces().Length);

        Assert.Equal(1000, box.Unify(new UnifyOptions { AngularTolerance = 1e-3, LinearTolerance = 1e-3 }).Value.Shape.IntegrateVolume(), Tol);
        Assert.True(box.Validate(new ValidationOptions { Geometry = true }).IsValid);

        var approximate = new BSplineApproximationOptions
        {
            Continuity2D = Continuity.C1,
            Continuity3D = Continuity.C1,
            MaxDegree = 9,
            MaxSegments = 100,
            Polynomial = true,
            PreferDegree = true,
            Tolerance2D = 1e-3,
            Tolerance3D = 1e-3,
        };
        Solid cylinder = Fixtures.Cylinder();
        Assert.Equal(cylinder.IntegrateVolume(), cylinder.ApproximateBSplines(approximate with { CheckResult = false }).Value.Shape.IntegrateVolume(), 1e-1);

        var convert = BSplineConversionOptions.Default with { Extrusions = true, Offsets = true, Revolutions = true };
        Assert.True(convert.Revolutions && convert.Offsets && convert.Extrusions);
        Assert.Equal(cylinder.IntegrateVolume(), cylinder.ConvertToBSpline(convert).Value.Shape.IntegrateVolume(), 1e-3);
    }

    [Fact]
    public void Measuring_and_meshing_take_their_options()
    {
        Solid box = Box();
        Assert.Equal(1000, box.IntegrateVolume(new MassOptions { OnlyClosed = true, UseTriangulation = false }), Tol);
        Assert.Equal(0, box.Distance(box, new DistanceOptions { Parallel = true }).Value.Value, Tol);

        var options = new MeshOptions { InternalVertices = false, MinSize = 0.01 };
        ShapeMesh mesh = box.Mesh(options).Value;
        Assert.Same(options, mesh.Options);
        Assert.Equal(12, mesh.Faces.Sum(f => f.TriangleCount));
        Assert.All(mesh.Faces, f => Assert.True(f.Deflection >= 0));
        Assert.All(mesh.Edges, e => Assert.False(e.Edge.IsNull));
    }

    [Fact]
    public void Writers_and_readers_take_their_options()
    {
        Solid box = Box();
        using (var stream = new MemoryStream())
        {
            Assert.True(Brep.Write(box, stream, new BrepWriteOptions { WithNormals = false }).IsSuccess);
            stream.Position = 0;
            Assert.Equal(1000, Brep.Read(stream).Value.IntegrateVolume(), Tol);
        }
        using (var stream = new MemoryStream())
        {
            Assert.True(Step.Write(box, stream).IsSuccess);
            stream.Position = 0;
            Assert.Equal(1000, Step.Read(stream, new StepReadOptions { ProductNames = false }).Value.Shape!.Value.IntegrateVolume(), 1e-3);
        }

        ModelNode model = ModelNode.Part(box);
        string Obj(ObjWriteOptions options)
        {
            using var stream = new MemoryStream();
            Assert.True(Kerf.Exchange.Obj.Write(model, stream, options: options).IsSuccess);
            return System.Text.Encoding.UTF8.GetString(stream.ToArray());
        }
        // the Y-up swap and the scale reach the vertices: the box's far corner (10, 10, 10) at 20
        string scaled = Obj(new ObjWriteOptions { Scale = 2, YUp = true, Normals = false, Mesh = MeshOptions.Default });
        Assert.Contains("v 20 20", scaled, StringComparison.Ordinal);
        Assert.DoesNotContain("vn ", scaled, StringComparison.Ordinal);
        Assert.Contains("vn ", Obj(new ObjWriteOptions { Normals = true }), StringComparison.Ordinal);

        Assert.NotNull(ThreeMfWriteOptions.Default);
        using (var stream = new MemoryStream())
        {
            Assert.True(ThreeMf.Write(model, stream, ThreeMfWriteOptions.Default).IsSuccess);
            Assert.True(stream.Length > 0);
        }
    }

    [Fact]
    public void Drawings_and_pictures_take_their_options()
    {
        Solid box = Box();
        LineDrawing drawing = box.ProjectLines(Camera.Top, new DrawingOptions { Mesh = MeshOptions.Default, Tolerance = 1e-4 }).Value;
        Assert.Equal(Camera.Top, drawing.Camera);
        Assert.NotEmpty(drawing.Edges);

        string svg = Svg.WriteToString(drawing, new SvgOptions { Decimals = 1, HiddenStyle = new LineStyle { Width = 0.5 } });
        Assert.StartsWith("<?xml", svg, StringComparison.Ordinal);

        Camera along = Camera.Along(Direction3D.UnitZ, Direction3D.UnitY);
        Assert.Equal(Direction3D.UnitY, along.Up);
        Assert.Equal(1, Math.Abs(Camera.Right.ViewDirection.X), 12);

        ShapeMesh mesh = box.Mesh().Value;
        var options = new RenderOptions
        {
            Width = 64,
            Height = 48,
            Ambient = 0.5,
            EdgeColor = new Color(1f, 0f, 0f),
            EdgeWidth = 2,
            FeatureAngle = 0.5,
            Lights = [new DirectionalLight(new Vector3D(0, 0, -1), 0.5)],
            Margin = 0.1,
            Shading = Shading.Flat,
        };
        RgbaImage image = Renderer.Render(mesh, Camera.Isometric, options).Value;
        Assert.Equal(64, image.Width);
        Assert.Equal(48, image.Height);
    }

    [Fact]
    public void A_2D_offset_can_stay_open()
    {
        Wire open = Wire.Polygon([new(0, 0, 0), new(10, 0, 0)]).Value;
        Face plane = Face.FromPlane(new Plane3D(CoordinateSystem.World)).Value;
        Built<Shape> offset = open.Offset2D(1, plane, new Offset2DOptions { Open = true }).Value;
        Assert.True(offset.Shape.IntegrateLength() > 0);
    }

    [Fact]
    public void Small_results_read_back()
    {
        Assert.Equal("[2, 6]", new Interval(2, 6).ToString());

        var d = new SurfaceDerivatives(3, new Point3D(1, 2, 3), new Vector3D(1, 0, 0), new Vector3D(2, 0, 0), new Vector3D(3, 0, 0),
                                       new Vector3D(4, 0, 0), new Vector3D(5, 0, 0), new Vector3D(6, 0, 0), new Vector3D(7, 0, 0),
                                       new Vector3D(8, 0, 0), new Vector3D(9, 0, 0));
        Assert.Equal(new[] { 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0 },
                     new[] { d.DUU.X, d.DVV.X, d.DUV.X, d.DUUU.X, d.DVVV.X, d.DUUV.X, d.DUVV.X });

        Solid box = Box();
        ShapeGraph graph = ShapeGraph.Build(box);
        Assert.Equal(box, graph.Root);
        Assert.True(graph.Count > 26);

        Built<Shape> filleted = box.Fillet([box.AllEdges()[0]], 1).Value;
        Assert.True(filleted.History.HasGenerated);

        Edge line = box.AllEdges()[0];
        EdgeCurve curve = line.ToCurve();
        Assert.False(curve.IsClosed);
        Assert.Equal(Continuity.CN, curve.Continuity);
        Assert.True(box.AllFaces()[0].ToSurface().Bounds.Item1.Length > 0);

        WireCurve wire = Square(10).ToCurve();
        Assert.Equal(4, wire.Wire.AllEdges().Length);
        Assert.Equal(Continuity.C0, wire.Continuity);
        CurveDerivatives atStart = wire.Derivatives(0.5, 1);
        Assert.Equal(1, atStart.Order);
        Assert.Equal(atStart.D1, wire.DerivativeN(0.5, 1));

        var hits = box.Intersect(new Line3D(new Point3D(5, 5, -1), Direction3D.UnitZ));
        Assert.Equal(2, hits.Length);
        Assert.All(hits, h => Assert.InRange(h.U + h.V, -1e6, 1e6));

        DistanceResult distance = box.AllFaces()[0].Distance(box.AllFaces()[1]).Value;
        Assert.All(distance.Solutions, s =>
        {
            Assert.True(Enum.IsDefined(s.SupportA));
            Assert.Equal(s.OnA, s.OnA);
            Assert.False(double.IsNaN(s.ParametersA.X));
        });

        SubShapes<Face>.Enumerator faces = box.Explore<Face>().GetEnumerator();
        Assert.True(faces.MoveNext());
        Assert.False(faces.Current.IsNull);
        bool refused = false;
        try
        {
            faces.Reset();
        }
        catch (NotSupportedException)
        {
            refused = true;
        }
        Assert.True(refused);
    }

    [Fact]
    public void Approximation_takes_a_parametrization()
    {
        Point3D[] points = [new(0, 0, 0), new(1, 1, 0), new(2, 0, 0), new(3, 1, 0), new(4, 0, 0)];
        BSplineCurve3D curve = BSplineCurve3D.Approximate(points, new ApproximationOptions { Parametrization = ParametrizationType.Centripetal }).Value;
        Assert.True(curve.StartPoint.IsEqual(points[0], 1e-3));
        Assert.True(curve.EndPoint.IsEqual(points[^1], 1e-3));
    }

    [Fact]
    public void Hits_and_points_read_back()
    {
        var circle = new CircleCurve3D(Frame3D.World, 2);
        var plane = new PlaneSurface(new Plane3D(new Point3D(2, 0, 0), Direction3D.UnitX));
        var hits = circle.Intersect(plane).Value;
        Assert.NotEmpty(hits);
        Assert.Equal(0, hits[0].CurveParameter, 6);
        Assert.Equal(0, hits[0].U, 6);
        Assert.Equal(0, hits[0].V, 6);

        CurvePoint2D nearest = Assert.Single(new LineCurve2D(Point2D.Origin, new Direction2D(1, 0)).Project(new Point2D(3, 1)));
        Assert.True(nearest.Point.IsEqual(new Point2D(3, 0), 1e-9));
    }

    [Fact]
    public void Shape_problems_read_back()
    {
        var problem = new ShapeProblem(Box(), Box().AllFaces()[0], DiagnosticCode.InvalidResult, "message");
        Assert.Equal(ShapeType.Face, problem.Context.Type);
    }

    [Fact]
    public void Colors_print_and_compare()
    {
        var red = new Color(1f, 0f, 0f);
        Assert.True(red == new Color(1f, 0f, 0f));
        Assert.True(red != new Color(0f, 1f, 0f));
        Assert.Contains("1", red.ToString(), StringComparison.Ordinal);
    }

    [Fact]
    public void Lights_are_immutable_arrays()
    {
        ImmutableArray<DirectionalLight> lights = RenderOptions.Default.Lights;
        Assert.False(lights.IsDefaultOrEmpty);
    }
}
