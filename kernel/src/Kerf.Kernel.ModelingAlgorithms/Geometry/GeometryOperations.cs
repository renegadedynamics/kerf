// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Engine.Construction.Curves;
using Kerf.Engine.Construction.Values;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Geometry.Library;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Meshing.Tools;
using Occt;

namespace Kerf.Geometry;

/// <summary>
/// Geometry's operations (docs/api/geometry.md): arcs and segments, fitting, conversion, projection and intersection.
/// Each is a C# 14 extension member, so <c>TrimmedCurve3D.ArcThrough(…)</c>, <c>curve.Project(point)</c> and
/// <c>GeometryOperations.Project(curve, point)</c> all work. A failure a caller should expect is a failed result;
/// misuse (a null, too few points) throws.
/// </summary>
public static partial class GeometryOperations
{
    /// <summary>A curve couldn't be constructed from its data (OCCT's <c>gce_ErrorType</c>): <c>geometry.construction-failed</c>.</summary>
    public static DiagnosticCode ConstructionFailed { get; } = new("geometry.construction-failed");

    /// <summary>No B-spline passes through the points: <c>geometry.interpolation-failed</c>.</summary>
    public static DiagnosticCode InterpolationFailed { get; } = new("geometry.interpolation-failed");

    /// <summary>No B-spline fits the points within the options: <c>geometry.approximation-failed</c>.</summary>
    public static DiagnosticCode ApproximationFailed { get; } = new("geometry.approximation-failed");

    /// <summary>The curves to join don't meet end to end: <c>geometry.not-connected</c>.</summary>
    public static DiagnosticCode NotConnected { get; } = new("geometry.not-connected");

    /// <summary>The curve couldn't be converted: <c>geometry.conversion-failed</c>.</summary>
    public static DiagnosticCode ConversionFailed { get; } = new("geometry.conversion-failed");

    /// <summary>The intersection couldn't be computed: <c>geometry.intersection-failed</c>.</summary>
    public static DiagnosticCode IntersectionFailed { get; } = new("geometry.intersection-failed");

    // --- Arcs and segments (GC_MakeArcOf…, GC_MakeSegment)

    extension(TrimmedCurve3D)
    {
        /// <summary>The arc of circle from <paramref name="start"/> through <paramref name="middle"/> to <paramref name="end"/>.</summary>
        public static Result<TrimmedCurve3D> ArcThrough(Point3D start, Point3D middle, Point3D end) =>
            Made(() => new MakeArcOfCircle(start, middle, end), static m => m.IsDone, static m => m.Status(), static m => m.Value());

        /// <summary>The arc of circle from <paramref name="start"/> to <paramref name="end"/>, leaving
        /// <paramref name="start"/> along <paramref name="tangent"/>.</summary>
        public static Result<TrimmedCurve3D> ArcTangent(Point3D start, Vector3D tangent, Point3D end) =>
            Made(() => new MakeArcOfCircle(start, tangent, end), static m => m.IsDone, static m => m.Status(), static m => m.Value());

        /// <summary>The arc of <paramref name="circle"/> between two angles (radians), counter-clockwise about its axis
        /// unless <paramref name="counterClockwise"/> is false.</summary>
        public static Result<TrimmedCurve3D> Arc(in Circle3D circle, double startAngle, double endAngle, bool counterClockwise = true)
        {
            Circle3D c = circle;
            return Made(() => new MakeArcOfCircle(c, startAngle, endAngle, counterClockwise), static m => m.IsDone, static m => m.Status(), static m => m.Value());
        }

        /// <summary>The arc of <paramref name="ellipse"/> between two parameters (radians).</summary>
        public static Result<TrimmedCurve3D> Arc(in Ellipse3D ellipse, double startAngle, double endAngle, bool counterClockwise = true)
        {
            Ellipse3D e = ellipse;
            return Made(() => new MakeArcOfEllipse(e, startAngle, endAngle, counterClockwise), static m => m.IsDone, static m => m.Status(), static m => m.Value());
        }

        /// <summary>The arc of <paramref name="hyperbola"/> between two parameters.</summary>
        public static Result<TrimmedCurve3D> Arc(in Hyperbola3D hyperbola, double start, double end, bool counterClockwise = true)
        {
            Hyperbola3D h = hyperbola;
            return Made(() => new MakeArcOfHyperbola(h, start, end, counterClockwise), static m => m.IsDone, static m => m.Status(), static m => m.Value());
        }

        /// <summary>The arc of <paramref name="parabola"/> between two parameters.</summary>
        public static Result<TrimmedCurve3D> Arc(in Parabola3D parabola, double start, double end, bool counterClockwise = true)
        {
            Parabola3D p = parabola;
            return Made(() => new MakeArcOfParabola(p, start, end, counterClockwise), static m => m.IsDone, static m => m.Status(), static m => m.Value());
        }

        /// <summary>The straight segment from <paramref name="start"/> to <paramref name="end"/>.</summary>
        public static Result<TrimmedCurve3D> Segment(Point3D start, Point3D end) =>
            Made(() => new MakeSegment(start, end), static m => m.IsDone, static m => m.Status(), static m => m.Value());
    }

    // --- Fitting (GeomAPI_Interpolate, GeomAPI_PointsToBSpline…, GeomConvert_CompCurveToBSplineCurve)

    extension(BSplineCurve3D)
    {
        /// <summary>The B-spline through <paramref name="points"/>, in order.</summary>
        /// <exception cref="ArgumentException">Fewer than two points, or tangents or parameters of the wrong count.</exception>
        public static Result<BSplineCurve3D> Interpolate(ReadOnlySpan<Point3D> points, InterpolationOptions? options = null)
        {
            options ??= InterpolationOptions.Default;
            if (points.Length < 2)
            {
                throw new ArgumentException("Interpolation needs at least two points.", nameof(points));
            }
            ImmutableArray<Vector3D> tangents = options.Tangents.IsDefault ? [] : options.Tangents;
            ImmutableArray<double> parameters = options.Parameters.IsDefault ? [] : options.Parameters;
            int count = points.Length;
            if (tangents.Length is not (0 or 2) && tangents.Length != count)
            {
                throw new ArgumentException("Give no tangents, one at each end, or one at every point.", nameof(options));
            }
            if (!parameters.IsEmpty && parameters.Length != points.Length + (options.Periodic ? 1 : 0))
            {
                throw new ArgumentException("Give one parameter for every point (and one more, for the closing point, if periodic), or none.",
                                            nameof(options));
            }

            var poles = new NCollection_HArray1<Point3D>(GeometryArrays.Of(points));
            return Guarded(InterpolationFailed, () =>
            {
                ApiInterpolate interpolate;
                try
                {
                    interpolate = parameters.IsEmpty
                        ? new ApiInterpolate(poles, options.Periodic, options.Tolerance)
                        : new ApiInterpolate(poles, new NCollection_HArray1<double>(GeometryArrays.Of(parameters.AsSpan())), options.Periodic, options.Tolerance);
                }
                catch (Exception caught) when (Refusal.Of(caught) is ConstructionErrorException)
                {
                    return Result<BSplineCurve3D>.Failure(Diagnostic.Error(InterpolationFailed,
                        "Two points coincide (within the tolerance), so no curve passes through them in turn."));
                }

                if (tangents.Length == count)
                {
                    // A zero tangent leaves its point's tangent free
                    var flags = new NCollection_HArray1<bool>(1, count, true);
                    for (int i = 0; i < count; i++)
                    {
                        flags.SetValue(i + 1, tangents[i].SquareMagnitude > 0);
                    }
                    interpolate.Load(GeometryArrays.Of(tangents.AsSpan()), flags, options.ScaleTangents);
                }
                else if (tangents.Length == 2)
                {
                    interpolate.Load(tangents[0], tangents[1], options.ScaleTangents);
                }

                interpolate.Perform();
                return interpolate.IsDone
                    ? interpolate.Curve()
                    : Result<BSplineCurve3D>.Failure(Diagnostic.Error(InterpolationFailed, "No B-spline passes through the points with these tangents and parameters."));
            });
        }

        /// <summary>A B-spline fitted to <paramref name="points"/> within the options' tolerance.</summary>
        /// <exception cref="ArgumentException">Fewer than two points, or degrees out of order.</exception>
        public static Result<BSplineCurve3D> Approximate(ReadOnlySpan<Point3D> points, ApproximationOptions? options = null)
        {
            options ??= ApproximationOptions.Default;
            if (points.Length < 2)
            {
                throw new ArgumentException("Approximation needs at least two points.", nameof(points));
            }
            CheckDegrees(options);
            NCollection_Array1<Point3D> array = GeometryArrays.Of(points);
            return Guarded(ApproximationFailed, () =>
            {
                var fit = options.Smoothing is var (length, curvature, torsion)
                    ? new PointsToBSpline(array, length, curvature, torsion, options.MaxDegree, options.Continuity, options.Tolerance)
                    : new PointsToBSpline(array, options.Parametrization, options.MinDegree, options.MaxDegree, options.Continuity, options.Tolerance);
                return fit.IsDone
                    ? fit.Curve()
                    : Result<BSplineCurve3D>.Failure(Diagnostic.Error(ApproximationFailed, $"No B-spline of degree {options.MinDegree} to {options.MaxDegree} fits the points within {options.Tolerance}."));
            });
        }

        /// <summary>One B-spline joining <paramref name="curves"/> end to end, in order (each may be reversed to fit).</summary>
        /// <exception cref="ArgumentException">No curves, or a null among them.</exception>
        public static Result<BSplineCurve3D> Join(IEnumerable<BoundedCurve3D> curves, double tolerance = Tolerances.Confusion)
        {
            ArgumentNullException.ThrowIfNull(curves);
            BoundedCurve3D[] list = [.. curves];
            if (list.Length == 0 || Array.IndexOf(list, null) >= 0)
            {
                throw new ArgumentException("Join needs at least one curve, and no nulls.", nameof(curves));
            }
            return Guarded(ConversionFailed, () =>
            {
                var join = new CompCurveToBSplineCurve(list[0]);
                for (int i = 1; i < list.Length; i++)
                {
                    if (!join.Add(list[i], tolerance))
                    {
                        return Result<BSplineCurve3D>.Failure(Diagnostic.Error(NotConnected,
                            $"Curve {i} doesn't meet the curves before it within {tolerance}."));
                    }
                }
                return join.BSplineCurve() is { } joined
                    ? joined
                    : Result<BSplineCurve3D>.Failure(Diagnostic.Error(ConversionFailed, "The curves couldn't be joined."));
            });
        }
    }

    extension(BSplineSurface)
    {
        /// <summary>A B-spline surface fitted to a grid of points (rows along U, columns along V).</summary>
        /// <exception cref="ArgumentException">Fewer than two rows or columns, or degrees out of order.</exception>
        public static Result<BSplineSurface> Approximate(Grid<Point3D> points, ApproximationOptions? options = null)
        {
            options ??= ApproximationOptions.Default;
            if (points.RowCount < 2 || points.ColumnCount < 2)
            {
                throw new ArgumentException("Approximation needs at least two rows and two columns of points.", nameof(points));
            }
            CheckDegrees(options);
            NCollection_Array2<Point3D> array = GeometryArrays.Of(points);
            return Guarded(ApproximationFailed, () =>
            {
                var fit = options.Smoothing is var (length, curvature, torsion)
                    ? new PointsToBSplineSurface(array, length, curvature, torsion, options.MaxDegree, options.Continuity, options.Tolerance)
                    : new PointsToBSplineSurface(array, options.Parametrization, options.MinDegree, options.MaxDegree, options.Continuity, options.Tolerance);
                return fit.IsDone
                    ? fit.Surface()
                    : Result<BSplineSurface>.Failure(Diagnostic.Error(ApproximationFailed, $"No B-spline surface of degree {options.MinDegree} to {options.MaxDegree} fits the points within {options.Tolerance}."));
            });
        }
    }

    // --- Conversion, projection and intersection of curves

    extension(Curve3D curve)
    {
        /// <summary>
        /// True if the curve lies in a plane within <paramref name="tolerance"/> (0 for the confusion tolerance), OCCT's
        /// <c>ShapeAnalysis_Curve::IsPlanar</c>: lines, conics, trimmed and offset curves of them, B-spline and Bezier
        /// curves by their poles; false for any other curve. With <paramref name="normal"/>, the plane must be normal to it.
        /// </summary>
        public bool IsPlanar(Direction3D? normal = null, double tolerance = 0)
        {
            ArgumentNullException.ThrowIfNull(curve);
            Engine.Geometry.Xyz n = normal is { } d ? d.XYZ() : default;
            return CurveAnalysis.IsPlanar(curve, ref n, tolerance);
        }

        /// <summary>The curve as a B-spline: exactly where OCCT can, else approximated within the options.</summary>
        public Result<BSplineCurve3D> ToBSpline(ConversionOptions? options = null)
        {
            ArgumentNullException.ThrowIfNull(curve);
            options ??= ConversionOptions.Default;
            Curve3D source = curve;
            return Guarded(ConversionFailed, () =>
            {
                if (!options.ForceApproximation)
                {
                    try
                    {
                        return GeomConvert.CurveToBSplineCurve(source);
                    }
                    catch (Exception caught) when (Refusal.IsFailure(caught))
                    {
                        // Not convertible exactly (an offset curve, an untrimmed line): approximate
                    }
                }
                if (source.Range.IsInfinite)
                {
                    return Result<BSplineCurve3D>.Failure(Diagnostic.Error(ConversionFailed, "An unbounded curve can't be converted to a B-spline."));
                }
                var approx = new ApproxCurve(source, options.Tolerance, options.Continuity, options.MaxSegments, options.MaxDegree);
                return approx.HasResult && approx.Curve() is { } result
                    ? result
                    : Result<BSplineCurve3D>.Failure(Diagnostic.Error(ConversionFailed, $"The curve couldn't be approximated within {options.Tolerance}."));
            });
        }

        /// <summary>The curve in the 2D coordinates of <paramref name="plane"/>; it must lie in the plane.</summary>
        public Result<Curve2D> ToCurve2D(in Plane3D plane)
        {
            ArgumentNullException.ThrowIfNull(curve);
            Plane3D p = plane;
            Curve3D source = curve;
            return Guarded(ConversionFailed, () => Projections.To2d(source, p) is { } flat
                ? flat
                : Result<Curve2D>.Failure(Diagnostic.Error(ConversionFailed, "The curve can't be expressed in the plane's coordinates.")));
        }

        /// <summary>The points of the curve nearest to <paramref name="point"/> (the extrema of the distance), nearest
        /// first; empty if there are none.</summary>
        public ImmutableArray<CurvePoint> Project(Point3D point)
        {
            ArgumentNullException.ThrowIfNull(curve);
            try
            {
                var projection = new ProjectPointOnCurve(point, curve);
                var points = ImmutableArray.CreateBuilder<CurvePoint>(projection.NbPoints());
                for (int i = 1; i <= projection.NbPoints(); i++)
                {
                    points.Add(new CurvePoint(projection.Parameter(i), projection.Point(i), projection.Distance(i)));
                }
                // Stable, so of equally near points the first OCCT found comes first, as its LowerDistance picks it
                return [.. points.OrderBy(static p => p.Distance)];
            }
            catch (Exception e) when (Refusal.IsFailure(e) || ModelAlgo.IsCaughtFailure(e))
            {
                return [];
            }
        }

        /// <summary>The point of the curve nearest to <paramref name="point"/>, if the projection finds one.</summary>
        public bool TryProjectNearest(Point3D point, out CurvePoint nearest)
        {
            ImmutableArray<CurvePoint> all = curve.Project(point);
            nearest = all.IsEmpty ? default : all[0];
            return !all.IsEmpty;
        }

        /// <summary>The points where the curve crosses <paramref name="surface"/>. Where the curve lies in the surface
        /// along a stretch, that stretch isn't a point, and the result carries a warning.</summary>
        public Result<ImmutableArray<CurveSurfaceHit>> Intersect(Surface surface)
        {
            ArgumentNullException.ThrowIfNull(curve);
            ArgumentNullException.ThrowIfNull(surface);
            Curve3D source = curve;
            return Guarded(IntersectionFailed, () =>
            {
                var intersection = new CurveSurfaceIntersection(source, surface);
                if (!intersection.IsDone)
                {
                    return Result<ImmutableArray<CurveSurfaceHit>>.Failure(Diagnostic.Error(IntersectionFailed, "The curve's intersection with the surface couldn't be computed."));
                }
                var hits = ImmutableArray.CreateBuilder<CurveSurfaceHit>(intersection.PointCount);
                for (int i = 1; i <= intersection.PointCount; i++)
                {
                    intersection.Parameters(i, out double u, out double v, out double w);
                    hits.Add(new CurveSurfaceHit(intersection.Point(i), w, u, v));
                }
                ImmutableArray<Diagnostic> warnings = intersection.SegmentCount > 0
                    ? [Diagnostic.Warning(new DiagnosticCode("geometry.curve-in-surface"), $"The curve lies in the surface along {intersection.SegmentCount} stretch(es), which aren't points.")]
                    : [];
                return Result<ImmutableArray<CurveSurfaceHit>>.Success(hits.MoveToImmutable(), warnings);
            });
        }
    }

    extension(Curve2D curve)
    {
        /// <summary>The 2D curve placed in <paramref name="plane"/>.</summary>
        public Curve3D ToCurve3D(in Plane3D plane)
        {
            ArgumentNullException.ThrowIfNull(curve);
            return Projections.To3d(curve, plane);
        }

        /// <summary>The points of the curve nearest to <paramref name="point"/> (the extrema of the distance, OCCT's
        /// <c>Geom2dAPI_ProjectPointOnCurve</c>), nearest first; empty if there are none.</summary>
        public ImmutableArray<CurvePoint2D> Project(Point2D point)
        {
            ArgumentNullException.ThrowIfNull(curve);
            try
            {
                var projection = new ProjectPointOnCurve2D(point, curve);
                var points = ImmutableArray.CreateBuilder<CurvePoint2D>(projection.NbPoints());
                for (int i = 1; i <= projection.NbPoints(); i++)
                {
                    points.Add(new CurvePoint2D(projection.Parameter(i), projection.Point(i), projection.Distance(i)));
                }

                // Stable, so of equally near points the first OCCT found comes first, as its LowerDistance picks it
                return [.. points.OrderBy(static p => p.Distance)];
            }
            catch (Exception e) when (Refusal.IsFailure(e) || ModelAlgo.IsCaughtFailure(e))
            {
                return [];
            }
        }

        /// <summary>The points where the two curves cross (and touch); where they overlap along a stretch, the
        /// result carries a warning.</summary>
        public Result<ImmutableArray<CurveCurveHit2D>> Intersect(Curve2D other, double tolerance = 1e-6)
        {
            ArgumentNullException.ThrowIfNull(curve);
            ArgumentNullException.ThrowIfNull(other);
            Curve2D first = curve;
            return Guarded(IntersectionFailed, () =>
            {
                var intersection = new CurveCurveIntersection2D(first, other, tolerance);
                var hits = ImmutableArray.CreateBuilder<CurveCurveHit2D>(intersection.NbPoints());
                for (int i = 1; i <= intersection.NbPoints(); i++)
                {
                    var hit = intersection.Intersector().Point(i);
                    hits.Add(new CurveCurveHit2D(hit.Value(), hit.ParamOnFirst(), hit.ParamOnSecond()));
                }
                ImmutableArray<Diagnostic> warnings = intersection.NbSegments() > 0
                    ? [Diagnostic.Warning(new DiagnosticCode("geometry.curves-overlap"), $"The curves overlap along {intersection.NbSegments()} stretch(es), which aren't points.")]
                    : [];
                return Result<ImmutableArray<CurveCurveHit2D>>.Success(hits.MoveToImmutable(), warnings);
            });
        }

        /// <summary>
        /// Where the curve crosses itself, within <paramref name="tolerance"/> (OCCT's <c>Geom2dAPI_InterCurveCurve</c> on one
        /// curve): each crossing's point and its two parameters. A stretch where it runs along itself is a warning.
        /// </summary>
        public Result<ImmutableArray<CurveCurveHit2D>> SelfIntersections(double tolerance = 1e-6)
        {
            ArgumentNullException.ThrowIfNull(curve);
            Curve2D source = curve;
            return Guarded(IntersectionFailed, () =>
            {
                var intersection = new CurveCurveIntersection2D(source, tolerance);
                var hits = ImmutableArray.CreateBuilder<CurveCurveHit2D>(intersection.NbPoints());
                for (int i = 1; i <= intersection.NbPoints(); i++)
                {
                    var hit = intersection.Intersector().Point(i);
                    hits.Add(new CurveCurveHit2D(hit.Value(), hit.ParamOnFirst(), hit.ParamOnSecond()));
                }
                ImmutableArray<Diagnostic> warnings = intersection.NbSegments() > 0
                    ? [Diagnostic.Warning(new DiagnosticCode("geometry.curves-overlap"), $"The curve runs along itself over {intersection.NbSegments()} stretch(es), which aren't points.")]
                    : [];
                return Result<ImmutableArray<CurveCurveHit2D>>.Success(hits.MoveToImmutable(), warnings);
            });
        }
    }

    // --- Surfaces

    extension(Surface surface)
    {
        /// <summary>The points of the surface nearest to <paramref name="point"/> (the extrema of the distance), nearest
        /// first; empty if there are none.</summary>
        public ImmutableArray<SurfacePoint> Project(Point3D point)
        {
            ArgumentNullException.ThrowIfNull(surface);
            try
            {
                var projection = new ProjectPointOnSurface(point, surface);
                var points = ImmutableArray.CreateBuilder<SurfacePoint>(projection.NbPoints());
                for (int i = 1; i <= projection.NbPoints(); i++)
                {
                    projection.Parameters(i, out double u, out double v);
                    points.Add(new SurfacePoint(u, v, projection.Point(i), projection.Distance(i)));
                }
                // Stable, so of equally near points the first OCCT found comes first, as its LowerDistance picks it
                return [.. points.OrderBy(static p => p.Distance)];
            }
            catch (Exception e) when (Refusal.IsFailure(e) || ModelAlgo.IsCaughtFailure(e))
            {
                return [];
            }
        }

        /// <summary>The point of the surface nearest to <paramref name="point"/>, if the projection finds one.</summary>
        public bool TryProjectNearest(Point3D point, out SurfacePoint nearest)
        {
            ImmutableArray<SurfacePoint> all = surface.Project(point);
            nearest = all.IsEmpty ? default : all[0];
            return !all.IsEmpty;
        }

        /// <summary>The curves where the two surfaces meet (empty if they don't).</summary>
        public Result<ImmutableArray<Curve3D>> Intersect(Surface other, double tolerance = Tolerances.Confusion)
        {
            ArgumentNullException.ThrowIfNull(surface);
            ArgumentNullException.ThrowIfNull(other);
            Surface first = surface;
            return Guarded(IntersectionFailed, () =>
            {
                var intersection = new SurfaceSurfaceIntersection(first, other, tolerance);
                if (!intersection.IsDone)
                {
                    return Result<ImmutableArray<Curve3D>>.Failure(Diagnostic.Error(IntersectionFailed, "The surfaces' intersection couldn't be computed."));
                }
                var curves = ImmutableArray.CreateBuilder<Curve3D>(intersection.LineCount);
                for (int i = 1; i <= intersection.LineCount; i++)
                {
                    curves.Add(intersection.Line(i));
                }
                return curves.MoveToImmutable();
            });
        }

        /// <summary>Whether the surface is planar within <paramref name="tolerance"/>, and its plane if so (OCCT's
        /// <c>GeomLib_IsPlanarSurface</c>).</summary>
        public bool TryGetPlane(double tolerance, out Plane3D plane)
        {
            ArgumentNullException.ThrowIfNull(surface);
            var check = new IsPlanarSurface(surface, tolerance);
            plane = check.IsPlanar ? check.Plan() : default;
            return check.IsPlanar;
        }
    }

    private static void CheckDegrees(ApproximationOptions options)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(options.MinDegree, 1, nameof(options));
        ArgumentOutOfRangeException.ThrowIfLessThan(options.MaxDegree, options.MinDegree, nameof(options));
        ArgumentOutOfRangeException.ThrowIfGreaterThan(options.MaxDegree, BSplineCurve3D.MaxDegree, nameof(options));
    }

    // A GC maker's result, or its status as a diagnostic
    private static Result<TrimmedCurve3D> Made<TMaker>(Func<TMaker> make, Func<TMaker, bool> isDone, Func<TMaker, ErrorType> status,
                                                       Func<TMaker, TrimmedCurve3D> value) =>
        Guarded(ConstructionFailed, () =>
        {
            TMaker maker = make();
            return isDone(maker)
                ? value(maker)
                : Result<TrimmedCurve3D>.Failure(Diagnostic.Error(ConstructionFailed, Describe(status(maker))));
        });

    private static string Describe(ErrorType status) => status switch
    {
        ErrorType.ConfusedPoints => "Two of the points coincide.",
        ErrorType.ColinearPoints => "The points are collinear, so no circle passes through them.",
        ErrorType.NegativeRadius or ErrorType.NullRadius or ErrorType.InvertRadius => "The radius isn't valid.",
        ErrorType.NullAxis or ErrorType.InvertAxis or ErrorType.NullVector => "A direction is zero or reversed.",
        ErrorType.NullAngle or ErrorType.BadAngle => "The angles don't make an arc.",
        _ => $"The curve can't be constructed ({FailureDiagnostics.Kebab(status.ToString())}).",
    };

    // The engine's failures inside a geometry operation as a failed result (decision 1369's step 2, for values)
    private static Result<T> Guarded<T>(DiagnosticCode code, Func<Result<T>> run)
    {
        try
        {
            return run();
        }
        catch (Exception e) when (Refusal.IsFailure(e) || ModelAlgo.IsCaughtFailure(e))
        {
            Diagnostic engine = FailureDiagnostics.ToDiagnostic(e);
            return Result<T>.Failure(Diagnostic.Error(code, engine.Message));
        }
    }
}
