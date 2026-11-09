namespace Kerf.Modeling;

// The objects as results: each constructor that can fail on values a caller would think valid, as an operation that
// returns the failure instead of throwing it (decision 2008). Each is the constructor, run by Failures.Try, so the two
// have one code path and the same error: the ModelingException the constructor throws is the failure returned.
public static partial class Operations
{
    /// <summary>
    /// <see cref="Box"/> as a result rather than an exception (decision 2008): the part, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Part> MakeBox(double length, double width, double height, Rotation? rotation = null, Alignment? align = null) =>
        Failures.Try<Part>(() => new Box(length, width, height, rotation, align));

    /// <summary>
    /// <see cref="Cone"/> as a result rather than an exception (decision 2008): the part, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Part> MakeCone(double bottomRadius, double topRadius, double height, double arcSize = 360,
                                        Rotation? rotation = null, Alignment? align = null) =>
        Failures.Try<Part>(() => new Cone(bottomRadius, topRadius, height, arcSize, rotation, align));

    /// <summary>
    /// <see cref="Cylinder"/> as a result rather than an exception (decision 2008): the part, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Part> MakeCylinder(double radius, double height, double arcSize = 360,
                                            Rotation? rotation = null, Alignment? align = null) =>
        Failures.Try<Part>(() => new Cylinder(radius, height, arcSize, rotation, align));

    /// <summary>
    /// <see cref="Sphere"/> as a result rather than an exception (decision 2008): the part, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Part> MakeSphere(double radius, double arcSize1 = -90, double arcSize2 = 90,
                                          double arcSize3 = 360, Rotation? rotation = null, Alignment? align = null) =>
        Failures.Try<Part>(() => new Sphere(radius, arcSize1, arcSize2, arcSize3, rotation, align));

    /// <summary>
    /// <see cref="Torus"/> as a result rather than an exception (decision 2008): the part, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Part> MakeTorus(double majorRadius, double minorRadius, double minorStartAngle = 0,
                                         double minorEndAngle = 360, double majorAngle = 360, Rotation? rotation = null,
                                         Alignment? align = null) =>
        Failures.Try<Part>(() => new Torus(majorRadius, minorRadius, minorStartAngle, minorEndAngle, majorAngle, rotation, align));

    /// <summary>
    /// <see cref="Wedge"/> as a result rather than an exception (decision 2008): the part, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Part> MakeWedge(double xSize, double ySize, double zSize, double xMin, double zMin,
                                         double xMax, double zMax, Rotation? rotation = null,
                                         Alignment? align = null) =>
        Failures.Try<Part>(() => new Wedge(xSize, ySize, zSize, xMin, zMin, xMax, zMax, rotation, align));

    /// <summary>
    /// <see cref="CounterBoreHole"/> as a result rather than an exception (decision 2008): the part, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Part> MakeCounterBoreHole(double radius, double counterBoreRadius, double counterBoreDepth,
                                                   double? depth = null) =>
        Failures.Try<Part>(() => new CounterBoreHole(radius, counterBoreRadius, counterBoreDepth, depth));

    /// <summary>
    /// <see cref="CounterSinkHole"/> as a result rather than an exception (decision 2008): the part, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Part> MakeCounterSinkHole(double radius, double counterSinkRadius, double? depth = null,
                                                   double counterSinkAngle = 82) =>
        Failures.Try<Part>(() => new CounterSinkHole(radius, counterSinkRadius, depth, counterSinkAngle));

    /// <summary>
    /// <see cref="Hole"/> as a result rather than an exception (decision 2008): the part, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Part> MakeHole(double radius, double? depth = null) =>
        Failures.Try<Part>(() => new Hole(radius, depth));

    /// <summary>
    /// <see cref="Circle"/> as a result rather than an exception (decision 2008): the sketch, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Sketch> MakeCircle(double radius, double arcSize = 360.0, Alignment? align = null) =>
        Failures.Try<Sketch>(() => new Circle(radius, arcSize, align));

    /// <summary>
    /// <see cref="Ellipse"/> as a result rather than an exception (decision 2008): the sketch, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Sketch> MakeEllipse(double xRadius, double yRadius, double rotation = 0, Alignment? align = null) =>
        Failures.Try<Sketch>(() => new Ellipse(xRadius, yRadius, rotation, align));

    /// <summary>
    /// <see cref="Polygon"/> as a result rather than an exception (decision 2008): the sketch, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Sketch> MakePolygon(IEnumerable<Point3D> points, double rotation = 0, Alignment? align = null) =>
        Failures.Try<Sketch>(() => new Polygon(points, rotation, align));

    /// <summary>
    /// <see cref="Polygon"/> as a result rather than an exception (decision 2008): the sketch, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Sketch> MakePolygon(params Point3D[] points) =>
        Failures.Try<Sketch>(() => new Polygon(points));

    /// <summary>
    /// <see cref="Rectangle"/> as a result rather than an exception (decision 2008): the sketch, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Sketch> MakeRectangle(double width, double height, double rotation = 0, Alignment? align = null) =>
        Failures.Try<Sketch>(() => new Rectangle(width, height, rotation, align));

    /// <summary>
    /// <see cref="RectangleRounded"/> as a result rather than an exception (decision 2008): the sketch, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Sketch> MakeRectangleRounded(double width, double height, double radius, double rotation = 0,
                                                      Alignment? align = null) =>
        Failures.Try<Sketch>(() => new RectangleRounded(width, height, radius, rotation, align));

    /// <summary>
    /// <see cref="RegularPolygon"/> as a result rather than an exception (decision 2008): the sketch, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Sketch> MakeRegularPolygon(double radius, int sideCount, bool majorRadius = true,
                                                    double rotation = 0, Alignment? align = null) =>
        Failures.Try<Sketch>(() => new RegularPolygon(radius, sideCount, majorRadius, rotation, align));

    /// <summary>
    /// <see cref="SlotCurve"/> as a result rather than an exception (decision 2008): the sketch, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Sketch> MakeSlotCurve(IShape1D curve, double height, double rotation = 0) =>
        Failures.Try<Sketch>(() => new SlotCurve(curve, height, rotation));

    /// <summary>
    /// <see cref="SlotCenterPoint"/> as a result rather than an exception (decision 2008): the sketch, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Sketch> MakeSlotCenterPoint(Point3D center, Point3D point, double height, double rotation = 0) =>
        Failures.Try<Sketch>(() => new SlotCenterPoint(center, point, height, rotation));

    /// <summary>
    /// <see cref="SlotCenterToCenter"/> as a result rather than an exception (decision 2008): the sketch, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Sketch> MakeSlotCenterToCenter(double centerSeparation, double height, double rotation = 0) =>
        Failures.Try<Sketch>(() => new SlotCenterToCenter(centerSeparation, height, rotation));

    /// <summary>
    /// <see cref="SlotOverall"/> as a result rather than an exception (decision 2008): the sketch, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Sketch> MakeSlotOverall(double width, double height, double rotation = 0, Alignment? align = null) =>
        Failures.Try<Sketch>(() => new SlotOverall(width, height, rotation, align));

    /// <summary>
    /// <see cref="Superellipse"/> as a result rather than an exception (decision 2008): the sketch, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Sketch> MakeSuperellipse(double width, double height, double order = 4, int pointCount = 16,
                                                  double rotation = 0, Alignment? align = null) =>
        Failures.Try<Sketch>(() => new Superellipse(width, height, order, pointCount, rotation, align));

    /// <summary>
    /// <see cref="Trapezoid"/> as a result rather than an exception (decision 2008): the sketch, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Sketch> MakeTrapezoid(double width, double height, double leftSideAngle,
                                               double? rightSideAngle = null, double rotation = 0,
                                               Alignment? align = null) =>
        Failures.Try<Sketch>(() => new Trapezoid(width, height, leftSideAngle, rightSideAngle, rotation, align));

    /// <summary>
    /// <see cref="Triangle"/> as a result rather than an exception (decision 2008): the sketch, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Sketch> MakeTriangle(double? sideA = null, double? sideB = null, double? sideC = null,
                                              double? angleA = null, double? angleB = null, double? angleC = null,
                                              Alignment? align = null, double rotation = 0) =>
        Failures.Try<Sketch>(() => new Triangle(sideA, sideB, sideC, angleA, angleB, angleC, align, rotation));

    /// <summary>
    /// <see cref="Airfoil"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeAirfoil(string airfoilCode, int pointCount = 50, bool finiteTE = false) =>
        Failures.Try<Curve>(() => new Airfoil(airfoilCode, pointCount, finiteTE));

    /// <summary>
    /// <see cref="Bezier"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeBezier(IEnumerable<Point3D> controlPoints, IReadOnlyList<double>? weights = null) =>
        Failures.Try<Curve>(() => new Bezier(controlPoints, weights));

    /// <summary>
    /// <see cref="Bezier"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeBezier(params Point3D[] controlPoints) =>
        Failures.Try<Curve>(() => new Bezier(controlPoints));

    /// <summary>
    /// <see cref="BlendCurve"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeBlendCurve(Edge curve0, Edge curve1,
                                               ContinuityLevel continuity = ContinuityLevel.C2,
                                               (Point3D, Point3D)? endPoints = null,
                                               (double Start, double End)? tangentScalars = null) =>
        Failures.Try<Curve>(() => new BlendCurve(curve0, curve1, continuity, endPoints, tangentScalars));

    /// <summary>
    /// <see cref="BSpline"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeBSpline(IEnumerable<Point3D> controlPoints, IEnumerable<double> knots, int degree,
                                            IEnumerable<double>? weights = null, bool periodic = false) =>
        Failures.Try<Curve>(() => new BSpline(controlPoints, knots, degree, weights, periodic));

    /// <summary>
    /// <see cref="CenterArc"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeCenterArc(Point3D center, double radius, double startAngle, double arcSize) =>
        Failures.Try<Curve>(() => new CenterArc(center, radius, startAngle, arcSize));

    /// <summary>
    /// <see cref="CenterArc"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeCenterArc(Point3D center, double radius, double startAngle, CurveLimit arcLimit) =>
        Failures.Try<Curve>(() => new CenterArc(center, radius, startAngle, arcLimit));

    /// <summary>
    /// <see cref="ConstrainedArcs"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeConstrainedArcs(Tangent tangencyOne, Tangent tangencyTwo, double radius,
                                                    Sagitta sagitta = Sagitta.Short,
                                                    Func<ShapeList<Edge>, IEnumerable<Edge>>? selector = null) =>
        Failures.Try<Curve>(() => new ConstrainedArcs(tangencyOne, tangencyTwo, radius, sagitta, selector));

    /// <summary>
    /// <see cref="ConstrainedArcs"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeConstrainedArcs(Tangent tangencyOne, Tangent tangencyTwo, Tangent tangencyThree,
                                                    Sagitta sagitta = Sagitta.Short,
                                                    Func<ShapeList<Edge>, IEnumerable<Edge>>? selector = null) =>
        Failures.Try<Curve>(() => new ConstrainedArcs(tangencyOne, tangencyTwo, tangencyThree, sagitta, selector));

    /// <summary>
    /// <see cref="ConstrainedArcs"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeConstrainedArcs(Tangent tangencyOne, Point3D center,
                                                    Func<ShapeList<Edge>, IEnumerable<Edge>>? selector = null) =>
        Failures.Try<Curve>(() => new ConstrainedArcs(tangencyOne, center, selector));

    /// <summary>
    /// <see cref="ConstrainedArcs"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeConstrainedArcs(Tangent tangencyOne, double radius, Tangent centerOn,
                                                    Func<ShapeList<Edge>, IEnumerable<Edge>>? selector = null) =>
        Failures.Try<Curve>(() => new ConstrainedArcs(tangencyOne, radius, centerOn, selector));

    /// <summary>
    /// <see cref="ConstrainedLines"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeConstrainedLines(Tangent tangencyOne, Tangent tangencyTwo,
                                                     Func<ShapeList<Edge>, IEnumerable<Edge>>? selector = null) =>
        Failures.Try<Curve>(() => new ConstrainedLines(tangencyOne, tangencyTwo, selector));

    /// <summary>
    /// <see cref="ConstrainedLines"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeConstrainedLines(Tangent tangencyOne, Axis reference, double angle,
                                                     Func<ShapeList<Edge>, IEnumerable<Edge>>? selector = null) =>
        Failures.Try<Curve>(() => new ConstrainedLines(tangencyOne, reference, angle, selector));

    /// <summary>
    /// <see cref="ConstrainedLines"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeConstrainedLines(Tangent tangencyOne, Axis reference, Vector3D direction,
                                                     Func<ShapeList<Edge>, IEnumerable<Edge>>? selector = null) =>
        Failures.Try<Curve>(() => new ConstrainedLines(tangencyOne, reference, direction, selector));

    /// <summary>
    /// <see cref="DoubleTangentArc"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeDoubleTangentArc(Point3D point, Vector3D tangent, IShape1D other, Keep keep = Keep.Top) =>
        Failures.Try<Curve>(() => new DoubleTangentArc(point, tangent, other, keep));

    /// <summary>
    /// <see cref="EllipticalCenterArc"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeEllipticalCenterArc(Point3D center, double xRadius, double yRadius,
                                                        double startAngle = 0.0, double arcSize = 90.0,
                                                        double rotation = 0.0) =>
        Failures.Try<Curve>(() => new EllipticalCenterArc(center, xRadius, yRadius, startAngle, arcSize, rotation));

    /// <summary>
    /// <see cref="EllipticalCenterArc"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeEllipticalCenterArc(Point3D center, double xRadius, double yRadius,
                                                        double startAngle, CurveLimit arcLimit,
                                                        double rotation = 0.0) =>
        Failures.Try<Curve>(() => new EllipticalCenterArc(center, xRadius, yRadius, startAngle, arcLimit, rotation));

    /// <summary>
    /// <see cref="EllipticalStartArc"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeEllipticalStartArc(Point3D startPoint, Vector3D startTangent, double xRadius,
                                                       double yRadius, double arcSize, double? startAngle = null,
                                                       Vector3D? majorAxisDir = null) =>
        Failures.Try<Curve>(() => new EllipticalStartArc(startPoint, startTangent, xRadius, yRadius, arcSize, startAngle, majorAxisDir));

    /// <summary>
    /// <see cref="ParabolicCenterArc"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeParabolicCenterArc(Point3D vertex, double focalLength, double startAngle = 0.0,
                                                       double arcSize = 90.0, double rotation = 0.0) =>
        Failures.Try<Curve>(() => new ParabolicCenterArc(vertex, focalLength, startAngle, arcSize, rotation));

    /// <summary>
    /// <see cref="ParabolicCenterArc"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeParabolicCenterArc(Point3D vertex, double focalLength, double startAngle,
                                                       CurveLimit arcLimit, double rotation = 0.0) =>
        Failures.Try<Curve>(() => new ParabolicCenterArc(vertex, focalLength, startAngle, arcLimit, rotation));

    /// <summary>
    /// <see cref="HyperbolicCenterArc"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeHyperbolicCenterArc(Point3D center, double xRadius, double yRadius,
                                                        double startAngle = 0.0, double arcSize = 90.0,
                                                        double rotation = 0.0) =>
        Failures.Try<Curve>(() => new HyperbolicCenterArc(center, xRadius, yRadius, startAngle, arcSize, rotation));

    /// <summary>
    /// <see cref="HyperbolicCenterArc"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeHyperbolicCenterArc(Point3D center, double xRadius, double yRadius,
                                                        double startAngle, CurveLimit arcLimit,
                                                        double rotation = 0.0) =>
        Failures.Try<Curve>(() => new HyperbolicCenterArc(center, xRadius, yRadius, startAngle, arcLimit, rotation));

    /// <summary>
    /// <see cref="Helix"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeHelix(double pitch, double height, double radius, Point3D? center = null,
                                          Vector3D? direction = null, double coneAngle = 0, bool leftHanded = false) =>
        Failures.Try<Curve>(() => new Helix(pitch, height, radius, center, direction, coneAngle, leftHanded));

    /// <summary>
    /// <see cref="FilletPolyline"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeFilletPolyline(IEnumerable<Point3D> points, double radius, bool close = false) =>
        Failures.Try<Curve>(() => new FilletPolyline(points, radius, close));

    /// <summary>
    /// <see cref="FilletPolyline"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeFilletPolyline(IEnumerable<Point3D> points, IEnumerable<double> radii, bool close = false) =>
        Failures.Try<Curve>(() => new FilletPolyline(points, radii, close));

    /// <summary>
    /// <see cref="JernArc"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeJernArc(Point3D start, Vector3D tangent, double radius, double arcSize) =>
        Failures.Try<Curve>(() => new JernArc(start, tangent, radius, arcSize));

    /// <summary>
    /// <see cref="JernArc"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeJernArc(Point3D start, Vector3D tangent, double radius, CurveLimit arcLimit) =>
        Failures.Try<Curve>(() => new JernArc(start, tangent, radius, arcLimit));

    /// <summary>
    /// <see cref="Line"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeLine(Point3D start, Point3D end) =>
        Failures.Try<Curve>(() => new Line(start, end));

    /// <summary>
    /// <see cref="Line"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeLine(IEnumerable<Point3D> points) =>
        Failures.Try<Curve>(() => new Line(points));

    /// <summary>
    /// <see cref="IntersectingLine"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeIntersectingLine(Point3D start, Vector3D direction, Shape other) =>
        Failures.Try<Curve>(() => new IntersectingLine(start, direction, other));

    /// <summary>
    /// <see cref="PolarLine"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakePolarLine(Point3D start, double length, double? angle = null,
                                              Vector3D? direction = null,
                                              LengthMode lengthMode = LengthMode.Diagonal) =>
        Failures.Try<Curve>(() => new PolarLine(start, length, angle, direction, lengthMode));

    /// <summary>
    /// <see cref="PolarLine"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakePolarLine(Point3D start, CurveLimit lengthLimit, double? angle = null, Vector3D? direction = null) =>
        Failures.Try<Curve>(() => new PolarLine(start, lengthLimit, angle, direction));

    /// <summary>
    /// <see cref="Polyline"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakePolyline(IEnumerable<Point3D> points, bool close = false) =>
        Failures.Try<Curve>(() => new Polyline(points, close));

    /// <summary>
    /// <see cref="Polyline"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakePolyline(params Point3D[] points) =>
        Failures.Try<Curve>(() => new Polyline(points));

    /// <summary>
    /// <see cref="RadiusArc"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeRadiusArc(Point3D startPoint, Point3D endPoint, double radius, bool shortSagitta = true) =>
        Failures.Try<Curve>(() => new RadiusArc(startPoint, endPoint, radius, shortSagitta));

    /// <summary>
    /// <see cref="SagittaArc"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeSagittaArc(Point3D startPoint, Point3D endPoint, double sagitta) =>
        Failures.Try<Curve>(() => new SagittaArc(startPoint, endPoint, sagitta));

    /// <summary>
    /// <see cref="Spline"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeSpline(IEnumerable<Point3D> points, IEnumerable<Vector3D>? tangents = null,
                                           IEnumerable<double>? tangentScalars = null, bool periodic = false) =>
        Failures.Try<Curve>(() => new Spline(points, tangents, tangentScalars, periodic));

    /// <summary>
    /// <see cref="Spline"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeSpline(params Point3D[] points) =>
        Failures.Try<Curve>(() => new Spline(points));

    /// <summary>
    /// <see cref="TangentArc"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeTangentArc(Point3D start, Point3D end, Vector3D tangent, bool tangentFromFirst = true) =>
        Failures.Try<Curve>(() => new TangentArc(start, end, tangent, tangentFromFirst));

    /// <summary>
    /// <see cref="TangentArc"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeTangentArc(IEnumerable<Point3D> points, Vector3D tangent, bool tangentFromFirst = true) =>
        Failures.Try<Curve>(() => new TangentArc(points, tangent, tangentFromFirst));

    /// <summary>
    /// <see cref="ThreePointArc"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeThreePointArc(Point3D point1, Point3D point2, Point3D point3) =>
        Failures.Try<Curve>(() => new ThreePointArc(point1, point2, point3));

    /// <summary>
    /// <see cref="ThreePointArc"/> as a result rather than an exception (decision 2008): the curve, or the failure the
    /// constructor would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the constructor.</exception>
    public static Result<Curve> MakeThreePointArc(IEnumerable<Point3D> points) =>
        Failures.Try<Curve>(() => new ThreePointArc(points));

    /// <summary>
    /// <see cref="ConstrainedArcs.CenteredOn"/> as a result rather than an exception (decision 2008): the arcs, or the
    /// failure it would throw as a <see cref="ModelingException"/>, with the same error.
    /// </summary>
    /// <exception cref="ArgumentException">An argument is wrong on its own, as for the method.</exception>
    public static Result<Curve> MakeConstrainedArcsCenteredOn(Tangent tangencyOne, Tangent tangencyTwo,
                                                              Tangent centerOn, Sagitta sagitta = Sagitta.Short,
                                                              Func<ShapeList<Edge>, IEnumerable<Edge>>? selector = null) =>
        Failures.Try<Curve>(() => ConstrainedArcs.CenteredOn(tangencyOne, tangencyTwo, centerOn, sagitta, selector));
}
