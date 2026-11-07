// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Engine.Classification;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Measurement;
using Kerf.Engine.Measurement.Distance;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Measurement;

/// <summary>
/// Properties and measurement (docs/api/measurement.md): mass properties, bounding boxes, distances, classification and
/// where a line meets a shape. Each is a C# 14 extension member too, so <c>MeasureOperations.IntegrateVolume(part)</c> and
/// <c>part.IntegrateVolume()</c> both work.
/// </summary>
/// <remarks>
/// Volume, area and length are numbers, not results: OCCT's integration can't fail on a valid shape, and on an invalid
/// one the number is what it is. A distance can fail, so it is a <see cref="Result{T}"/>. A null shape is misuse
/// (<see cref="ArgumentException"/>).
/// </remarks>
public static class MeasureOperations
{
    /// <summary>The code of a distance that couldn't be computed: <c>measurement.distance-failed</c>.</summary>
    public static DiagnosticCode DistanceFailed { get; } = new("measurement.distance-failed");

    extension<TShape>(TShape shape) where TShape : struct, IShape<TShape>
    {
        /// <summary>The volume the shape's faces bound, whatever the shape is (OCCT's <c>BRepGProp::VolumeProperties</c>): zero for a shape
        /// without faces; for an open shell, the signed volume of the cone from its location's origin.</summary>
        /// <exception cref="ArgumentException">The shape is null.</exception>
        public double IntegrateVolume(MassOptions? options = null) => Integrate(NotNull(shape.AsShape()), Dimension.Volume, options, massOnly: true).Mass();

        /// <summary>The area of the shape's faces (<c>BRepGProp::SurfaceProperties</c>).</summary>
        /// <exception cref="ArgumentException">The shape is null.</exception>
        public double IntegrateArea(MassOptions? options = null) => Integrate(NotNull(shape.AsShape()), Dimension.Area, options, massOnly: true).Mass();

        /// <summary>
        /// The centre of mass of the shape's faces' area, without its moments: what <c>MassProperties(Dimension.Area).CentreOfMass</c>
        /// integrates, and exact for a planar face alone that's bounded by lines and circles.
        /// </summary>
        /// <exception cref="ArgumentException">The shape is null.</exception>
        public Point3D CentreOfArea(MassOptions? options = null)
        {
            Shape s = NotNull(shape.AsShape());
            options ??= Moments;
            if (s.Type == ShapeType.Face && options.Tolerance > 0 && !options.UseTriangulation
                && PlanarFaceMass.TryCentre(ShapeCasts.Face(s), new Domain(), out Point3D centre))
            {
                return centre;
            }

            return Integrate(s, Dimension.Area, options).CentreOfMass();
        }

        /// <summary>The length of the shape's edges (<c>BRepGProp::LinearProperties</c>).</summary>
        /// <exception cref="ArgumentException">The shape is null.</exception>
        public double IntegrateLength(MassOptions? options = null) => Integrate(NotNull(shape.AsShape()), Dimension.Length, options).Mass();

        /// <summary>The shape's mass properties of the given dimension, with density 1.</summary>
        /// <exception cref="ArgumentException">The shape is null.</exception>
        /// <exception cref="ArgumentOutOfRangeException"><paramref name="dimension"/> isn't one of the enum's.</exception>
        public MassProperties MassProperties(Dimension dimension, MassOptions? options = null)
            => ToMassProperties(Integrate(NotNull(shape.AsShape()), dimension, options ?? Moments), dimension);

        /// <summary>
        /// A box aligned with the axes containing the shape (OCCT's <c>BRepBndLib::Add</c>): enlarged by the shape's
        /// tolerances and loose (fast) by default, tight when <paramref name="optimal"/> (<c>AddOptimal</c>, slower).
        /// A stored mesh is used where a face or edge has one only with <paramref name="useTriangulation"/>; a shape
        /// with nothing to bound gives the empty box.
        /// </summary>
        /// <exception cref="ArgumentException">The shape is null.</exception>
        public BoundingBox BoundingBox(bool optimal = false, bool useTriangulation = false)
        {
            Shape s = NotNull(shape.AsShape());
            var box = new Box();
            if (optimal)
            {
                Bounds.AddOptimal(s, ref box, useTriangulation);
            }
            else
            {
                Bounds.Add(s, ref box, useTriangulation);
            }

            return ToBoundingBox(box);
        }

        /// <summary>
        /// A box in the orientation that fits the shape (OCCT's <c>BRepBndLib::AddOBB</c>): from the shape's points
        /// when it is made of planes and lines (or carries a stored mesh), otherwise along its axes of inertia; tighter
        /// and slower when <paramref name="optimal"/>. Enlarged by the shape's tolerances.
        /// </summary>
        /// <exception cref="ArgumentException">The shape is null.</exception>
        public OrientedBox OrientedBox(bool optimal = false)
        {
            var box = new Obb();
            Bounds.AddOBB(NotNull(shape.AsShape()), ref box, true, optimal);
            return Kerf.Geometry.OrientedBox.FromEngine(box);
        }

        /// <summary>
        /// The minimum distance between this shape and <paramref name="other"/>, with the closest points (OCCT's
        /// <c>BRepExtrema_DistShapeShape</c>). A point of one inside the other (a solid) gives zero.
        /// </summary>
        /// <returns>The distance, or a failure (<see cref="DistanceFailed"/>, or the engine's error) when there is no
        /// solution.</returns>
        /// <exception cref="ArgumentException">A shape is null.</exception>
        /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
        public Result<DistanceResult> Distance<TOther>(TOther other, DistanceOptions? options = null, CancellationToken cancellationToken = default)
            where TOther : struct, IShape<TOther>
            => DistanceCore(NotNull(shape.AsShape()), NotNull(other.AsShape()), options ?? DistanceOptions.Default, cancellationToken);

        /// <summary>
        /// Where the line meets the shape's faces (OCCT's <c>BRepIntCurveSurface_Inter</c>), sorted along the line
        /// (by <see cref="LineHit.LineParameter"/>), each point inside a face or on its boundary.
        /// </summary>
        /// <exception cref="ArgumentException">The shape is null, or the line is a default one.</exception>
        public ImmutableArray<LineHit> Intersect(in Line3D line, double tolerance = Tolerances.Confusion)
        {
            Shape s = NotNull(shape.AsShape());
            Direction3D d = line.Direction;
            if (d.X == 0 && d.Y == 0 && d.Z == 0)
            {
                throw new ArgumentException("The line must be a valid line, not a default one.", nameof(line));
            }

            var hits = ImmutableArray.CreateBuilder<LineHit>();
            var intersector = new CurveShapeIntersector();
            for (intersector.Init(s, line, tolerance); intersector.More(); intersector.Next())
            {
                hits.Add(new LineHit(intersector.Pnt(), intersector.W(), intersector.Face(), intersector.U(), intersector.V(),
                                     intersector.State()));
            }

            // Stable, so hits at one parameter keep the order OCCT found them in
            return [.. hits.OrderBy(static h => h.LineParameter)];
        }

        /// <summary>
        /// Whether <paramref name="point"/> lies on one of the shape's faces, its boundary included, within
        /// <paramref name="tolerance"/> (OCCT's <c>BRepClass3d_SolidClassifier</c>, its <c>IsOnAFace</c>): for faces and shells
        /// as well as solids.
        /// </summary>
        /// <exception cref="ArgumentException">The shape is null.</exception>
        public bool IsOnFace(Point3D point, double tolerance = Tolerances.Confusion)
        {
            var classifier = new SolidClassifier(NotNull(shape.AsShape()));
            classifier.Perform(point, tolerance);
            return classifier.IsOnAFace;
        }
    }

    extension(Face face)
    {
        /// <summary>
        /// Whether the point of the face's surface at <paramref name="parameters"/> (u, v) is inside the face, outside it
        /// or on its boundary within <paramref name="tolerance"/> in the parameters (OCCT's <c>IntTools_FClass2d</c>).
        /// </summary>
        /// <exception cref="ArgumentException">The face is null.</exception>
        public Containment Classify(Point2D parameters, double tolerance = Tolerances.Confusion)
        {
            var classifier = new FaceClassifier2D(NotNull(face.AsShape()).As<Face>(), tolerance);
            return classifier.Perform(parameters);
        }

        /// <summary>
        /// The parameters (u, v) on the face's surface of the point of it nearest <paramref name="point"/>, and the distance
        /// to it (OCCT's <c>ShapeAnalysis_Surface::ValueOfUV</c> and <c>Gap</c>, in the surface's own placement), within
        /// <paramref name="tolerance"/>. On a periodic surface they may be off the face by a period.
        /// </summary>
        /// <exception cref="ArgumentException">The face is null, or has no surface.</exception>
        public (Point2D Parameters, double Gap) ParametersOf(Point3D point, double tolerance = Tolerances.Confusion)
        {
            Face f = NotNull(face.AsShape()).As<Face>();
            Surface surface = Engine.Topology.Brep.Tool.Surface(f, out Placement location)
                              ?? throw new ArgumentException("The face has no surface.", nameof(face));
            Point3D local = point.Transformed(location.Transformation.Inverted());
            var analysis = new Engine.Healing.Analysis.SurfaceAnalysis(surface);
            Point2D uv = analysis.ValueOfUV(local, tolerance);
            return (uv, analysis.Gap());
        }
    }

    extension(Solid solid)
    {
        /// <summary>
        /// Whether <paramref name="point"/> is inside the solid, outside it, or on its boundary within
        /// <paramref name="tolerance"/> (OCCT's <c>BRepClass3d_SolidClassifier</c>).
        /// </summary>
        /// <exception cref="ArgumentException">The solid is null.</exception>
        public Containment Classify(Point3D point, double tolerance = Tolerances.Confusion) => solid.ClassifyPoint(point, tolerance);
    }

    extension<TShape>(TShape shape) where TShape : struct, IShape<TShape>
    {
        /// <summary>
        /// Whether <paramref name="point"/> is inside the shape, outside it, or on its boundary within
        /// <paramref name="tolerance"/> (OCCT's <c>BRepClass3d_SolidClassifier</c>, which treats a compound's solids as one):
        /// <c>Classify</c> for any shape.
        /// </summary>
        /// <exception cref="ArgumentException">The shape is null.</exception>
        public Containment ClassifyPoint(Point3D point, double tolerance = Tolerances.Confusion)
        {
            var classifier = new SolidClassifier(NotNull(shape.AsShape()));
            classifier.Perform(point, tolerance);
            return classifier.State();
        }
    }

    // A centre and moments follow a face's mass less closely than the mass does: 4e-6 out on a cylinder at 1e-6, exact from 1e-8 (decision 1808)
    private static readonly MassOptions Moments = new() { Tolerance = 1e-9 };

    private static Properties Integrate(in Shape shape, Dimension dimension, MassOptions? options, bool massOnly = false)
    {
        options ??= MassOptions.Default;
        var props = new Properties();
        switch (dimension)
        {
            case Dimension.Length:
                MassPropertiesBuilder.LinearProperties(shape, props, options.SkipShared, options.UseTriangulation, settled: true);
                break;
            case Dimension.Area when options.Tolerance > 0 && !options.UseTriangulation:
                MassPropertiesBuilder.SurfaceProperties(shape, props, options.Tolerance, options.SkipShared, massOnly);
                break;
            case Dimension.Area:
                MassPropertiesBuilder.SurfaceProperties(shape, props, options.SkipShared, options.UseTriangulation);
                break;
            case Dimension.Volume when options.Tolerance > 0 && !options.UseTriangulation:
                MassPropertiesBuilder.VolumeProperties(shape, props, options.Tolerance, options.OnlyClosed, options.SkipShared, massOnly);
                break;
            case Dimension.Volume:
                MassPropertiesBuilder.VolumeProperties(shape, props, options.OnlyClosed, options.SkipShared, options.UseTriangulation);
                break;
            default:
                throw new ArgumentOutOfRangeException(nameof(dimension), dimension, "Not a dimension.");
        }

        return props;
    }

    private static MassProperties ToMassProperties(Properties props, Dimension dimension)
    {
        double mass = props.Mass();
        Point3D centre = props.CentreOfMass();
        props.StaticMoments(out double mx, out double my, out double mz);
        (double, double, double) moments = (0, 0, 0);
        (Direction3D, Direction3D, Direction3D) axes = (Direction3D.UnitX, Direction3D.UnitY, Direction3D.UnitZ);
        if (mass != 0)
        {
            PrincipalProperties principal = props.PrincipalProperties();
            principal.Moments(out double i1, out double i2, out double i3);
            moments = (i1, i2, i3);
            axes = (new Direction3D(principal.FirstAxisOfInertia()), new Direction3D(principal.SecondAxisOfInertia()),
                    new Direction3D(principal.ThirdAxisOfInertia()));
        }

        return new MassProperties
        {
            Dimension = dimension,
            Mass = mass,
            CentreOfMass = centre,
            Inertia = props.MatrixOfInertia(),
            StaticMoments = new Vector3D(mx, my, mz),
            PrincipalMoments = moments,
            PrincipalAxes = axes,
        };
    }

    private static BoundingBox ToBoundingBox(in Box box)
    {
        if (box.IsVoid)
        {
            return Kerf.Geometry.BoundingBox.Empty;
        }

        box.Get(out double xMin, out double yMin, out double zMin, out double xMax, out double yMax, out double zMax);
        return new BoundingBox(new Point3D(xMin, yMin, zMin), new Point3D(xMax, yMax, zMax));
    }

    private static Result<DistanceResult> DistanceCore(in Shape a, in Shape b, DistanceOptions options, CancellationToken cancellationToken)
    {
        cancellationToken = Cancellation.Effective(cancellationToken);
        using Cancellation.Scope cancellation = Cancellation.Enter(cancellationToken);
        cancellationToken.ThrowIfCancellationRequested();
        var indicator = new ProgressIndicator(cancellationToken);
        var calc = new DistanceShapeShape();
        calc.SetDeflection(options.Deflection);
        calc.SetMultiThread(options.Parallel);
        calc.LoadS1(a);
        calc.LoadS2(b);
        try
        {
            calc.Perform(indicator.Start());
        }
        catch (Exception e) when (Refusal.IsFailure(e))
        {
            cancellationToken.ThrowIfCancellationRequested();
            return Result<DistanceResult>.Failure(OperationDiagnostics.FromException(e));
        }

        cancellationToken.ThrowIfCancellationRequested();
        if (!calc.IsDone || calc.SolutionCount == 0)
        {
            return Result<DistanceResult>.Failure(Diagnostic.Error(DistanceFailed, "The distance between the shapes couldn't be computed."));
        }

        var solutions = ImmutableArray.CreateBuilder<DistanceSolution>(calc.SolutionCount);
        for (int i = 1; i <= calc.SolutionCount; i++)
        {
            SupportType typeA = calc.SupportTypeShape1(i), typeB = calc.SupportTypeShape2(i);
            solutions.Add(new DistanceSolution(calc.PointOnShape1(i), calc.PointOnShape2(i),
                                               Kind(typeA), calc.SupportOnShape1(i), Parameters(calc, i, typeA, first: true),
                                               Kind(typeB), calc.SupportOnShape2(i), Parameters(calc, i, typeB, first: false)));
        }

        return new DistanceResult { Value = calc.Value(), Solutions = solutions.MoveToImmutable(), IsInnerSolution = calc.InnerSolution() };
    }

    private static SupportKind Kind(SupportType type) => type switch
    {
        SupportType.IsVertex => SupportKind.Vertex,
        SupportType.IsOnEdge => SupportKind.Edge,
        _ => SupportKind.Face,
    };

    private static Point2D Parameters(DistanceShapeShape calc, int i, SupportType type, bool first)
    {
        double u = 0, v = 0;
        switch (type)
        {
            case SupportType.IsOnEdge when first:
                calc.ParOnEdgeS1(i, out u);
                break;
            case SupportType.IsOnEdge:
                calc.ParOnEdgeS2(i, out u);
                break;
            case SupportType.IsInFace when first:
                calc.ParOnFaceS1(i, out u, out v);
                break;
            case SupportType.IsInFace:
                calc.ParOnFaceS2(i, out u, out v);
                break;
        }

        return new Point2D(u, v);
    }

    private static Shape NotNull(Shape shape, [System.Runtime.CompilerServices.CallerArgumentExpression(nameof(shape))] string? name = null)
        => shape.IsNull ? throw new ArgumentException("The shape is null.", name) : shape;
}
