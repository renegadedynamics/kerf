using System.Collections.Immutable;
using System.Globalization;
using System.Text;

namespace Kerf.Modeling;

// The entities of a DXF export, in the XY plane: what ExportDxf keeps until it writes
internal abstract record DxfEntity(string Layer);

internal sealed record DxfLine(string Layer, Point2D Start, Point2D End) : DxfEntity(Layer);

internal sealed record DxfCircle(string Layer, Point2D Center, double Radius) : DxfEntity(Layer);

// Angles in degrees, counter-clockwise from the start to the end, each in [0, 360)
internal sealed record DxfArc(string Layer, Point2D Center, double Radius, double StartAngle, double EndAngle) : DxfEntity(Layer);

// The parameters counter-clockwise from the start to the end, in [0, 2π] (0 and 2π for a whole ellipse)
internal sealed record DxfEllipse(string Layer, Point2D Center, Vector2D MajorAxis, double Ratio, double StartParameter, double EndParameter)
    : DxfEntity(Layer);

// A B-spline's whole knot sequence, poles, and weights (empty for a polynomial one)
internal sealed record DxfSpline(string Layer, int Degree, ImmutableArray<double> Knots, ImmutableArray<Point2D> ControlPoints,
                                 ImmutableArray<double> Weights) : DxfEntity(Layer);

internal sealed record DxfPolyline(string Layer, ImmutableArray<Point2D> Points) : DxfEntity(Layer);

/// <summary>2D curves as DXF entities, as build123d's ExportDXF converts edges, exactly where the version can.</summary>
internal sealed class CurveConverter(DxfVersion version, double tolerance)
{
    private const double TwoPi = 2 * Math.PI;

    /// <summary>Adds the curve between the parameters; <paramref name="reversed"/> runs a line from its end (an edge's
    /// orientation, which build123d follows for lines only).</summary>
    public void Add(Curve2D curve, double u0, double u1, bool reversed, string layer, ImmutableList<DxfEntity>.Builder entities)
    {
        while (curve is TrimmedCurve2D trimmed)
        {
            curve = trimmed.BasisCurve;
        }

        switch (curve)
        {
            case LineCurve2D:
                Point2D a = curve.Evaluate(u0), b = curve.Evaluate(u1);
                entities.Add(reversed ? new DxfLine(layer, b, a) : new DxfLine(layer, a, b));
                return;
            case CircleCurve2D circle:
                entities.Add(Circle(circle, u0, u1, layer));
                return;
            case EllipseCurve2D ellipse when version != DxfVersion.R12:
                entities.Add(Ellipse(ellipse, u0, u1, layer));
                return;
            case BSplineCurve2D spline when version != DxfVersion.R12:
                entities.Add(Spline(Trimmed(spline, u0, u1), layer));
                return;
            case BezierCurve2D bezier when version != DxfVersion.R12:
                entities.Add(Spline(Trimmed(bezier, u0, u1), layer));
                return;
            case BSplineCurve2D spline:
                entities.Add(new DxfPolyline(layer, [.. FlattenSpans(spline, u0, u1)]));
                return;
            case BezierCurve2D bezier:
                entities.Add(new DxfPolyline(layer, [.. FlattenSpans(Trimmed(bezier, u0, u1), u0, u1)]));
                return;
            default:
                entities.Add(new DxfPolyline(layer, [.. Flatten(curve.Evaluate, u0, u1, tolerance)]));
                return;
        }
    }

    /// <summary>Adds a curve as a polyline within the tolerance.</summary>
    public void AddFlattened(Func<double, Point2D> curve, double u0, double u1, string layer, ImmutableList<DxfEntity>.Builder entities)
        => entities.Add(new DxfPolyline(layer, [.. Flatten(curve, u0, u1, tolerance)]));

    // A spline flattened knot span by knot span, so that a corner at a knot (a polyline's, a degree-1 spline's) is
    // always a vertex, and a degree-1 spline is its poles exactly
    private List<Point2D> FlattenSpans(BSplineCurve2D spline, double u0, double u1)
    {
        var points = new List<Point2D> { spline.Evaluate(u0) };
        var breaks = spline.Knots.Where(k => k > u0 + Export2D.ParametricTolerance && k < u1 - Export2D.ParametricTolerance).Append(u1).ToList();
        double from = u0;
        foreach (double to in breaks)
        {
            if (spline.Degree == 1 && !spline.IsRational)
                points.Add(spline.Evaluate(to));
            else
                points.AddRange(Flatten(spline.Evaluate, from, to, tolerance).Skip(1));
            from = to;
        }
        return points;
    }

    private static DxfEntity Circle(CircleCurve2D curve, double u0, double u1, string layer)
    {
        Circle2D circle = curve.Circle;
        if (u1 - u0 >= TwoPi - Export2D.ParametricTolerance)
            return new DxfCircle(layer, circle.Location, circle.Radius);
        // A point at u is at the angle phi + u counter-clockwise from X, or phi - u where the circle runs clockwise
        Direction2D x = circle.XAxis.Direction;
        double phi = Math.Atan2(x.Y, x.X);
        (double start, double end) = circle.IsDirect ? (phi + u0, phi + u1) : (phi - u1, phi - u0);
        double startDegrees = Degrees(start), span = (u1 - u0) * 180 / Math.PI;
        return new DxfArc(layer, circle.Location, circle.Radius, startDegrees, Normalised(startDegrees + span, 360));
    }

    private static DxfEllipse Ellipse(EllipseCurve2D curve, double u0, double u1, string layer)
    {
        Frame2D frame = curve.Position;
        Direction2D x = frame.XDirection, y = frame.YDirection;
        var major = new Vector2D(x.X * curve.MajorRadius, x.Y * curve.MajorRadius);
        double ratio = curve.MinorRadius / curve.MajorRadius;
        if (u1 - u0 >= TwoPi - Export2D.ParametricTolerance)
            return new DxfEllipse(layer, frame.Location, major, ratio, 0, TwoPi);
        bool direct = x.X * y.Y - x.Y * y.X >= 0;
        double start = Normalised(direct ? u0 : -u1, TwoPi);
        return new DxfEllipse(layer, frame.Location, major, ratio, start, Normalised(start + (u1 - u0), TwoPi));
    }

    // The B-spline between the parameters, not periodic (DXF's knots are a plain sequence), as build123d's
    // GeomConvert.SplitBSplineCurve
    private static BSplineCurve2D Trimmed(BSplineCurve2D spline, double u0, double u1)
    {
        if (u0 > spline.FirstParameter + Export2D.ParametricTolerance || u1 < spline.LastParameter - Export2D.ParametricTolerance)
            spline = spline.Segmented(u0, u1, Export2D.ParametricTolerance);
        return spline.IsPeriodic ? spline.WithNotPeriodic() : spline;
    }

    private static BSplineCurve2D Trimmed(BezierCurve2D bezier, double u0, double u1)
    {
        if (u0 > bezier.FirstParameter + Export2D.ParametricTolerance || u1 < bezier.LastParameter - Export2D.ParametricTolerance)
            bezier = bezier.Segmented(u0, u1);
        int n = bezier.PoleCount;
        var poles = bezier.Poles.ToArray();
        int[] mults = [bezier.Degree + 1, bezier.Degree + 1];
        return bezier.IsRational
            ? new BSplineCurve2D(poles, [0.0, 1.0], mults, bezier.Degree, weights: bezier.Weights.ToArray())
            : new BSplineCurve2D(poles, [0.0, 1.0], mults, bezier.Degree);
    }

    private static DxfSpline Spline(BSplineCurve2D spline, string layer)
        => new(layer, spline.Degree, [.. spline.KnotSequence], [.. spline.Poles], spline.IsRational ? [.. spline.Weights] : []);

    private static double Degrees(double radians) => Normalised(radians * 180 / Math.PI, 360);

    private static double Normalised(double value, double period)
    {
        double r = value % period;
        if (r < 0)
            r += period;
        return r >= period ? 0 : r;
    }

    /// <summary>The points of a polyline within the tolerance of the curve: eight equal steps, each halved while its
    /// midpoint or quarter points are farther than the tolerance from its chord (Kerf.Drawing's flattening for SVG,
    /// which looks at the midpoint alone).</summary>
    internal static List<Point2D> Flatten(Func<double, Point2D> curve, double u0, double u1, double tolerance)
    {
        var points = new List<Point2D> { curve(u0) };
        const int steps = 8;
        Point2D previous = points[0];
        for (int i = 1; i <= steps; i++)
        {
            double a = u0 + (u1 - u0) * (i - 1) / steps, b = i == steps ? u1 : u0 + (u1 - u0) * i / steps;
            Point2D next = curve(b);
            Subdivide(curve, a, previous, b, next, tolerance, 0, points);
            previous = next;
        }
        return points;
    }

    private static void Subdivide(Func<double, Point2D> curve, double a, Point2D pa, double b, Point2D pb, double tolerance, int depth, List<Point2D> points)
    {
        double m = 0.5 * (a + b);
        Point2D pm = curve(m);
        if (depth < 16 && (DistanceToChord(pm, pa, pb) > tolerance
                           || DistanceToChord(curve(0.5 * (a + m)), pa, pb) > tolerance
                           || DistanceToChord(curve(0.5 * (m + b)), pa, pb) > tolerance))
        {
            Subdivide(curve, a, pa, m, pm, tolerance, depth + 1, points);
            Subdivide(curve, m, pm, b, pb, tolerance, depth + 1, points);
            return;
        }
        points.Add(pb);
    }

    private static double DistanceToChord(Point2D p, Point2D a, Point2D b)
    {
        double dx = b.X - a.X, dy = b.Y - a.Y, l2 = dx * dx + dy * dy;
        if (l2 <= 0)
            return p.DistanceTo(a);
        double t = Math.Clamp(((p.X - a.X) * dx + (p.Y - a.Y) * dy) / l2, 0, 1);
        double ex = a.X + t * dx - p.X, ey = a.Y + t * dy - p.Y;
        return Math.Sqrt(ex * ex + ey * ey);
    }
}

/// <summary>
/// The text of a DXF file: a 2013 file in the structure AutoCAD and ezdxf write (handles, owners, the symbol tables,
/// the model and paper space blocks and layouts), or an R12 file of the header, tables and entities. Numbers are
/// written to round-trip exactly, in the invariant culture.
/// </summary>
internal sealed class DxfWriter(DxfVersion version, Unit unit, IReadOnlyList<DxfLayer> layers, IReadOnlyList<DxfEntity> entities,
                                CancellationToken cancellationToken)
{
    /// <summary>DXF's line weights, in hundredths of a millimetre.</summary>
    internal static readonly ImmutableArray<int> LineWeights =
        [0, 5, 9, 13, 15, 18, 20, 25, 30, 35, 40, 50, 53, 60, 70, 80, 90, 100, 106, 120, 140, 158, 200, 211];

    private readonly StringBuilder _text = new();
    private readonly bool _r12 = version == DxfVersion.R12;
    private int _nextHandle = 1;

    /// <summary>A weight in millimetres as DXF's code, if it's one of the standard weights to within rounding.</summary>
    internal static int? LineWeight(double millimetres)
    {
        double hundredths = millimetres * 100;
        int rounded = (int)Math.Round(hundredths);
        return Math.Abs(hundredths - rounded) < 1e-6 && LineWeights.Contains(rounded) ? rounded : null;
    }

    internal static byte[] Encode(string text, DxfVersion version)
        => version == DxfVersion.R12 ? Encoding.ASCII.GetBytes(text) : new UTF8Encoding(false).GetBytes(text);

    public string Write()
    {
        var lineTypes = layers.Where(l => l.LineType is { } t && t != LineType.Continuous)
                              .Select(l => l.LineType!.Value).Distinct().Order().ToList();
        (Point2D min, Point2D max) = Extents();
        if (_r12)
        {
            WriteR12(lineTypes, min, max);
        }
        else
        {
            Write2013(lineTypes, min, max);
        }

        Pair(0, "EOF");
        return _text.ToString();
    }

    private void WriteR12(List<LineType> lineTypes, Point2D min, Point2D max)
    {
        Section("HEADER");
        Variable("$ACADVER", 1, "AC1009");
        Variable("$INSBASE", 10, Point2D.Origin);
        Variable("$EXTMIN", 10, min);
        Variable("$EXTMAX", 10, max);
        EndSection();

        Section("TABLES");
        Table("VPORT", 1);
        Pair(0, "VPORT");
        Pair(2, "*ACTIVE");
        Pair(70, 0);
        Viewport(min, max);
        EndTable();

        Table("LTYPE", 1 + lineTypes.Count);
        LineTypeRecord(0, "CONTINUOUS", "Solid line", [0.0]);
        foreach (LineType type in lineTypes)
        {
            LineTypeDefinition definition = Export2D.LineTypeDefinition(type);
            LineTypeRecord(0, definition.Name, definition.Description, Scaled(definition.Pattern));
        }

        EndTable();

        Table("LAYER", layers.Count);
        foreach (DxfLayer layer in layers)
        {
            Pair(0, "LAYER");
            Pair(2, layer.Name);
            Pair(70, 0);
            Pair(62, (int)(layer.Color ?? ColorIndex.Black));
            Pair(6, LineTypeName(layer.LineType));
        }

        EndTable();

        Table("STYLE", 1);
        Pair(0, "STYLE");
        Pair(2, "STANDARD");
        Pair(70, 0);
        Pair(40, 0.0);
        Pair(41, 1.0);
        Pair(50, 0.0);
        Pair(71, 0);
        Pair(42, 2.5);
        Pair(3, "txt");
        Pair(4, "");
        EndTable();
        EndSection();

        Section("BLOCKS");
        EndSection();

        Section("ENTITIES");
        foreach (DxfEntity entity in entities)
        {
            cancellationToken.ThrowIfCancellationRequested();
            Entity(entity, 0);
        }

        EndSection();
    }

    private void Write2013(List<LineType> lineTypes, Point2D min, Point2D max)
    {
        // Handles: the objects first, then the tables and blocks, then the entities
        int rootDictionary = Handle(), groupDictionary = Handle(), layoutDictionary = Handle(), mlineStyleDictionary = Handle(),
            plotStyleDictionary = Handle(), plotStyleNormal = Handle(), modelLayout = Handle(), paperLayout = Handle(), mlineStyle = Handle();
        int vportTable = Handle(), ltypeTable = Handle(), layerTable = Handle(), styleTable = Handle(), viewTable = Handle(),
            ucsTable = Handle(), appidTable = Handle(), dimstyleTable = Handle(), blockRecordTable = Handle();
        int modelRecord = Handle(), paperRecord = Handle(), modelBlock = Handle(), modelEnd = Handle(), paperBlock = Handle(), paperEnd = Handle();
        int activeViewport = Handle(), byBlock = Handle(), byLayer = Handle(), continuous = Handle();
        var lineTypeHandles = lineTypes.Select(_ => Handle()).ToList();
        var layerHandles = layers.Select(_ => Handle()).ToList();
        int standardStyle = Handle(), acadApp = Handle(), standardDimstyle = Handle();
        var entityHandles = entities.Select(_ => Handle()).ToList();
        int seed = _nextHandle;

        Section("HEADER");
        Variable("$ACADVER", 1, "AC1027");
        Variable("$DWGCODEPAGE", 3, "ANSI_1252");
        Variable("$INSBASE", 10, Point2D.Origin);
        Variable("$EXTMIN", 10, min);
        Variable("$EXTMAX", 10, max);
        Variable("$LIMMIN", 10, Point2D.Origin, threeD: false);
        Variable("$LIMMAX", 10, new Point2D(420, 297), threeD: false);
        Variable("$LTSCALE", 40, 1.0);
        Variable("$LUNITS", 70, 2);
        Variable("$HANDSEED", 5, Hex(seed));
        Variable("$MEASUREMENT", 70, unit is Unit.Inch or Unit.Foot ? 0 : 1);
        Variable("$INSUNITS", 70, unit switch
        {
            Unit.Micrometer => 13,
            Unit.Millimeter => 4,
            Unit.Centimeter => 5,
            Unit.Meter => 6,
            Unit.Inch => 1,
            _ => 2,
        });
        EndSection();

        Section("CLASSES");
        Class("ACDBDICTIONARYWDFLT", "AcDbDictionaryWithDefault");
        Class("ACDBPLACEHOLDER", "AcDbPlaceHolder");
        Class("LAYOUT", "AcDbLayout");
        EndSection();

        Section("TABLES");
        Table2013("VPORT", vportTable, 1);
        Record("VPORT", activeViewport, vportTable, "AcDbViewportTableRecord");
        Pair(2, "*Active");
        Pair(70, 0);
        Viewport(min, max);
        EndTable();

        Table2013("LTYPE", ltypeTable, 3 + lineTypes.Count);
        LineTypeRecord(byBlock, "ByBlock", "", [0.0], ltypeTable);
        LineTypeRecord(byLayer, "ByLayer", "", [0.0], ltypeTable);
        LineTypeRecord(continuous, "Continuous", "Solid line", [0.0], ltypeTable);
        for (int i = 0; i < lineTypes.Count; i++)
        {
            LineTypeDefinition definition = Export2D.LineTypeDefinition(lineTypes[i]);
            LineTypeRecord(lineTypeHandles[i], definition.Name, definition.Description, Scaled(definition.Pattern), ltypeTable);
        }

        EndTable();

        Table2013("LAYER", layerTable, layers.Count);
        for (int i = 0; i < layers.Count; i++)
        {
            DxfLayer layer = layers[i];
            Record("LAYER", layerHandles[i], layerTable, "AcDbLayerTableRecord");
            Pair(2, layer.Name);
            Pair(70, 0);
            Pair(62, (int)(layer.Color ?? ColorIndex.Black));
            Pair(6, LineTypeName(layer.LineType));
            Pair(370, layer.LineWeight is { } w ? LineWeight(w)!.Value : -3);
            HandlePair(390, plotStyleNormal);
        }

        EndTable();

        Table2013("STYLE", styleTable, 1);
        Record("STYLE", standardStyle, styleTable, "AcDbTextStyleTableRecord");
        Pair(2, "Standard");
        Pair(70, 0);
        Pair(40, 0.0);
        Pair(41, 1.0);
        Pair(50, 0.0);
        Pair(71, 0);
        Pair(42, 2.5);
        Pair(3, "txt");
        Pair(4, "");
        EndTable();

        Table2013("VIEW", viewTable, 0);
        EndTable();
        Table2013("UCS", ucsTable, 0);
        EndTable();

        Table2013("APPID", appidTable, 1);
        Record("APPID", acadApp, appidTable, "AcDbRegAppTableRecord");
        Pair(2, "ACAD");
        Pair(70, 0);
        EndTable();

        Table2013("DIMSTYLE", dimstyleTable, 1, "AcDbDimStyleTable");
        Pair(0, "DIMSTYLE");
        HandlePair(105, standardDimstyle);
        HandlePair(330, dimstyleTable);
        Pair(100, "AcDbSymbolTableRecord");
        Pair(100, "AcDbDimStyleTableRecord");
        Pair(2, "Standard");
        Pair(70, 0);
        EndTable();

        Table2013("BLOCK_RECORD", blockRecordTable, 2);
        Record("BLOCK_RECORD", modelRecord, blockRecordTable, "AcDbBlockTableRecord");
        Pair(2, "*Model_Space");
        HandlePair(340, modelLayout);
        Pair(70, 0);
        Pair(280, 1);
        Pair(281, 0);
        Record("BLOCK_RECORD", paperRecord, blockRecordTable, "AcDbBlockTableRecord");
        Pair(2, "*Paper_Space");
        HandlePair(340, paperLayout);
        Pair(70, 0);
        Pair(280, 1);
        Pair(281, 0);
        EndTable();
        EndSection();

        Section("BLOCKS");
        Block("*Model_Space", modelBlock, modelEnd, modelRecord);
        Block("*Paper_Space", paperBlock, paperEnd, paperRecord);
        EndSection();

        Section("ENTITIES");
        for (int i = 0; i < entities.Count; i++)
        {
            cancellationToken.ThrowIfCancellationRequested();
            Entity(entities[i], entityHandles[i], modelRecord);
        }

        EndSection();

        Section("OBJECTS");
        Dictionary(rootDictionary, 0, ("ACAD_GROUP", groupDictionary), ("ACAD_LAYOUT", layoutDictionary),
                   ("ACAD_MLINESTYLE", mlineStyleDictionary), ("ACAD_PLOTSTYLENAME", plotStyleDictionary));
        Dictionary(groupDictionary, rootDictionary);
        Dictionary(layoutDictionary, rootDictionary, ("Model", modelLayout), ("Layout1", paperLayout));
        Dictionary(mlineStyleDictionary, rootDictionary, ("Standard", mlineStyle));
        Pair(0, "ACDBDICTIONARYWDFLT");
        HandlePair(5, plotStyleDictionary);
        HandlePair(330, rootDictionary);
        Pair(100, "AcDbDictionary");
        Pair(281, 1);
        Pair(3, "Normal");
        HandlePair(350, plotStyleNormal);
        Pair(100, "AcDbDictionaryWithDefault");
        HandlePair(340, plotStyleNormal);
        Pair(0, "ACDBPLACEHOLDER");
        HandlePair(5, plotStyleNormal);
        HandlePair(330, plotStyleDictionary);
        Layout(modelLayout, layoutDictionary, "Model", 1024, 0, modelRecord);
        Layout(paperLayout, layoutDictionary, "Layout1", 0, 1, paperRecord);
        Pair(0, "MLINESTYLE");
        HandlePair(5, mlineStyle);
        HandlePair(330, mlineStyleDictionary);
        Pair(100, "AcDbMlineStyle");
        Pair(2, "Standard");
        Pair(70, 0);
        Pair(3, "");
        Pair(62, 256);
        Pair(51, 90.0);
        Pair(52, 90.0);
        Pair(71, 2);
        Pair(49, 0.5);
        Pair(62, 256);
        Pair(6, "BYLAYER");
        Pair(49, -0.5);
        Pair(62, 256);
        Pair(6, "BYLAYER");
        EndSection();
    }

    private void Entity(DxfEntity entity, int handle, int owner = 0)
    {
        switch (entity)
        {
            case DxfLine line:
                Start("LINE", entity, handle, owner, "AcDbLine");
                Point(10, line.Start);
                Point(11, line.End);
                break;
            case DxfCircle circle:
                Start("CIRCLE", entity, handle, owner, "AcDbCircle");
                Point(10, circle.Center);
                Pair(40, circle.Radius);
                break;
            case DxfArc arc:
                Start("ARC", entity, handle, owner, "AcDbCircle");
                Point(10, arc.Center);
                Pair(40, arc.Radius);
                if (!_r12)
                {
                    Pair(100, "AcDbArc");
                }

                Pair(50, arc.StartAngle);
                Pair(51, arc.EndAngle);
                break;
            case DxfEllipse ellipse:
                Start("ELLIPSE", entity, handle, owner, "AcDbEllipse");
                Point(10, ellipse.Center);
                Point(11, new Point2D(ellipse.MajorAxis.X, ellipse.MajorAxis.Y));
                Pair(40, ellipse.Ratio);
                Pair(41, ellipse.StartParameter);
                Pair(42, ellipse.EndParameter);
                break;
            case DxfSpline spline:
                Start("SPLINE", entity, handle, owner, "AcDbSpline");
                Pair(70, spline.Weights.IsEmpty ? 8 : 12);
                Pair(71, spline.Degree);
                Pair(72, spline.Knots.Length);
                Pair(73, spline.ControlPoints.Length);
                Pair(74, 0);
                foreach (double knot in spline.Knots)
                {
                    Pair(40, knot);
                }

                foreach (double weight in spline.Weights)
                {
                    Pair(41, weight);
                }

                foreach (Point2D pole in spline.ControlPoints)
                {
                    Point(10, pole);
                }

                break;
            case DxfPolyline polyline when _r12:
                Pair(0, "POLYLINE");
                Pair(8, entity.Layer);
                Pair(66, 1);
                Point(10, Point2D.Origin);
                Pair(70, 0);
                foreach (Point2D p in polyline.Points)
                {
                    Pair(0, "VERTEX");
                    Pair(8, entity.Layer);
                    Point(10, p);
                }

                Pair(0, "SEQEND");
                Pair(8, entity.Layer);
                break;
            case DxfPolyline polyline:
                Start("LWPOLYLINE", entity, handle, owner, "AcDbPolyline");
                Pair(90, polyline.Points.Length);
                Pair(70, 0);
                foreach (Point2D p in polyline.Points)
                {
                    Pair(10, p.X);
                    Pair(20, p.Y);
                }

                break;
        }
    }

    private void Start(string type, DxfEntity entity, int handle, int owner, string subclass)
    {
        Pair(0, type);
        if (!_r12)
        {
            HandlePair(5, handle);
            HandlePair(330, owner);
            Pair(100, "AcDbEntity");
        }

        Pair(8, entity.Layer);
        if (!_r12)
        {
            Pair(100, subclass);
        }
    }

    private void Viewport(Point2D min, Point2D max)
    {
        double width = max.X - min.X, height = max.Y - min.Y;
        Pair(10, 0.0);
        Pair(20, 0.0);
        Pair(11, 1.0);
        Pair(21, 1.0);
        Pair(12, 0.5 * (min.X + max.X));
        Pair(22, 0.5 * (min.Y + max.Y));
        Pair(13, 0.0);
        Pair(23, 0.0);
        Pair(14, 0.5);
        Pair(24, 0.5);
        Pair(15, 0.5);
        Pair(25, 0.5);
        Pair(16, 0.0);
        Pair(26, 0.0);
        Pair(36, 1.0);
        Pair(17, 0.0);
        Pair(27, 0.0);
        Pair(37, 0.0);
        Pair(40, Math.Max(1.1 * Math.Max(height, width / 1.34), 1e-6));
        Pair(41, 1.34);
        Pair(42, 50.0);
        Pair(43, 0.0);
        Pair(44, 0.0);
        Pair(50, 0.0);
        Pair(51, 0.0);
        Pair(71, 0);
        Pair(72, 1000);
        Pair(73, 1);
        Pair(74, 3);
        Pair(75, 0);
        Pair(76, 0);
        Pair(77, 0);
        Pair(78, 0);
    }

    private void LineTypeRecord(int handle, string name, string description, ImmutableArray<double> pattern, int table = 0)
    {
        if (_r12)
        {
            Pair(0, "LTYPE");
        }
        else
        {
            Record("LTYPE", handle, table, "AcDbLinetypeTableRecord");
        }

        Pair(2, name);
        Pair(70, 0);
        Pair(3, description);
        Pair(72, 65);
        int count = pattern.Length - 1;
        Pair(73, count);
        Pair(40, pattern[0]);
        for (int i = 1; i <= count; i++)
        {
            Pair(49, pattern[i]);
            if (!_r12)
            {
                Pair(74, 0);
            }
        }
    }

    // A pattern from tenths of an inch to the drawing's units
    private ImmutableArray<double> Scaled(ImmutableArray<double> pattern)
    {
        double scale = Export2D.LineTypeScale(unit);
        return [.. pattern.Select(v => v * scale)];
    }

    private string LineTypeName(LineType? type)
        => type is null or LineType.Continuous ? _r12 ? "CONTINUOUS" : "Continuous" : Export2D.LineTypeDefinition(type.Value).Name;

    private void Block(string name, int handle, int end, int record)
    {
        Pair(0, "BLOCK");
        HandlePair(5, handle);
        HandlePair(330, record);
        Pair(100, "AcDbEntity");
        Pair(8, "0");
        Pair(100, "AcDbBlockBegin");
        Pair(2, name);
        Pair(70, 0);
        Point(10, Point2D.Origin);
        Pair(3, name);
        Pair(1, "");
        Pair(0, "ENDBLK");
        HandlePair(5, end);
        HandlePair(330, record);
        Pair(100, "AcDbEntity");
        Pair(8, "0");
        Pair(100, "AcDbBlockEnd");
    }

    private void Dictionary(int handle, int owner, params (string Name, int Handle)[] items)
    {
        Pair(0, "DICTIONARY");
        HandlePair(5, handle);
        HandlePair(330, owner);
        Pair(100, "AcDbDictionary");
        Pair(281, 1);
        foreach ((string name, int item) in items)
        {
            Pair(3, name);
            HandlePair(350, item);
        }
    }

    private void Layout(int handle, int owner, string name, int plotFlags, int tabOrder, int record)
    {
        Pair(0, "LAYOUT");
        HandlePair(5, handle);
        HandlePair(330, owner);
        Pair(100, "AcDbPlotSettings");
        Pair(1, "");
        Pair(4, "A3");
        Pair(6, "");
        Pair(40, 7.5);
        Pair(41, 20.0);
        Pair(42, 7.5);
        Pair(43, 20.0);
        Pair(44, 420.0);
        Pair(45, 297.0);
        Pair(46, 0.0);
        Pair(47, 0.0);
        Pair(48, 0.0);
        Pair(49, 0.0);
        Pair(140, 0.0);
        Pair(141, 0.0);
        Pair(142, 1.0);
        Pair(143, 1.0);
        Pair(70, plotFlags);
        Pair(72, 1);
        Pair(73, 0);
        Pair(74, 5);
        Pair(7, "");
        Pair(75, 16);
        Pair(76, 0);
        Pair(77, 2);
        Pair(78, 300);
        Pair(147, 1.0);
        Pair(148, 0.0);
        Pair(149, 0.0);
        Pair(100, "AcDbLayout");
        Pair(1, name);
        Pair(70, 1);
        Pair(71, tabOrder);
        Pair(10, 0.0);
        Pair(20, 0.0);
        Pair(11, 420.0);
        Pair(21, 297.0);
        Pair(12, 0.0);
        Pair(22, 0.0);
        Pair(32, 0.0);
        Pair(14, 1e20);
        Pair(24, 1e20);
        Pair(34, 1e20);
        Pair(15, -1e20);
        Pair(25, -1e20);
        Pair(35, -1e20);
        Pair(146, 0.0);
        Pair(13, 0.0);
        Pair(23, 0.0);
        Pair(33, 0.0);
        Pair(16, 1.0);
        Pair(26, 0.0);
        Pair(36, 0.0);
        Pair(17, 0.0);
        Pair(27, 1.0);
        Pair(37, 0.0);
        Pair(76, 1);
        HandlePair(330, record);
    }

    private void Class(string name, string cppName)
    {
        Pair(0, "CLASS");
        Pair(1, name);
        Pair(2, cppName);
        Pair(3, "ObjectDBX Classes");
        Pair(90, 0);
        Pair(91, 0);
        Pair(280, 0);
        Pair(281, 0);
    }

    private void Table(string name, int count)
    {
        Pair(0, "TABLE");
        Pair(2, name);
        Pair(70, count);
    }

    private void Table2013(string name, int handle, int count, string? subclass = null)
    {
        Pair(0, "TABLE");
        Pair(2, name);
        HandlePair(5, handle);
        HandlePair(330, 0);
        Pair(100, "AcDbSymbolTable");
        Pair(70, count);
        if (subclass is not null)
        {
            Pair(100, subclass);
        }
    }

    private void Record(string type, int handle, int table, string subclass)
    {
        Pair(0, type);
        HandlePair(5, handle);
        HandlePair(330, table);
        Pair(100, "AcDbSymbolTableRecord");
        Pair(100, subclass);
    }

    private void EndTable() => Pair(0, "ENDTAB");

    private void Section(string name)
    {
        Pair(0, "SECTION");
        Pair(2, name);
    }

    private void EndSection() => Pair(0, "ENDSEC");

    private void Variable(string name, int code, string value)
    {
        Pair(9, name);
        Pair(code, value);
    }

    private void Variable(string name, int code, int value)
    {
        Pair(9, name);
        Pair(code, value);
    }

    private void Variable(string name, int code, double value)
    {
        Pair(9, name);
        Pair(code, value);
    }

    private void Variable(string name, int code, Point2D value, bool threeD = true)
    {
        Pair(9, name);
        Pair(code, value.X);
        Pair(code + 10, value.Y);
        if (threeD)
        {
            Pair(code + 20, 0.0);
        }
    }

    private void Point(int code, Point2D p)
    {
        Pair(code, p.X);
        Pair(code + 10, p.Y);
        Pair(code + 20, 0.0);
    }

    private int Handle() => _nextHandle++;

    private static string Hex(int handle) => handle.ToString("X", CultureInfo.InvariantCulture);

    private void HandlePair(int code, int handle) => Pair(code, Hex(handle));

    private void Pair(int code, int value) => Pair(code, value.ToString(CultureInfo.InvariantCulture));

    private void Pair(int code, double value) => Pair(code, Number(value));

    private void Pair(int code, string value)
    {
        if (code < 10)
        {
            _text.Append("  ");
        }
        else if (code < 100)
        {
            _text.Append(' ');
        }

        _text.Append(code.ToString(CultureInfo.InvariantCulture)).Append("\r\n").Append(value).Append("\r\n");
    }

    /// <summary>A number that reads back as the same double: the shortest round-trip form, with a decimal point.</summary>
    internal static string Number(double value)
    {
        if (!double.IsFinite(value))
        {
            throw new InvalidOperationException($"A DXF number must be finite, not {value}.");
        }

        if (value == 0)
        {
            return "0.0";
        }

        string text = value.ToString("R", CultureInfo.InvariantCulture);
        return text.Contains('.') || text.Contains('E') ? text : text + ".0";
    }

    // The drawing's extent, from points on its entities; the origin for an empty drawing
    private (Point2D Min, Point2D Max) Extents()
    {
        double xMin = double.PositiveInfinity, yMin = double.PositiveInfinity, xMax = double.NegativeInfinity, yMax = double.NegativeInfinity;
        void Add(Point2D p)
        {
            xMin = Math.Min(xMin, p.X);
            yMin = Math.Min(yMin, p.Y);
            xMax = Math.Max(xMax, p.X);
            yMax = Math.Max(yMax, p.Y);
        }

        foreach (DxfEntity entity in entities)
        {
            switch (entity)
            {
                case DxfLine line:
                    Add(line.Start);
                    Add(line.End);
                    break;
                case DxfCircle circle:
                    Add(new Point2D(circle.Center.X - circle.Radius, circle.Center.Y - circle.Radius));
                    Add(new Point2D(circle.Center.X + circle.Radius, circle.Center.Y + circle.Radius));
                    break;
                case DxfArc arc:
                    double span = arc.EndAngle - arc.StartAngle;
                    if (span <= 0)
                    {
                        span += 360;
                    }

                    for (int i = 0; i <= 64; i++)
                    {
                        double a = (arc.StartAngle + span * i / 64) * Math.PI / 180;
                        Add(new Point2D(arc.Center.X + arc.Radius * Math.Cos(a), arc.Center.Y + arc.Radius * Math.Sin(a)));
                    }

                    break;
                case DxfEllipse ellipse:
                    double sweep = ellipse.EndParameter - ellipse.StartParameter;
                    if (sweep <= 0)
                    {
                        sweep += 2 * Math.PI;
                    }

                    Vector2D major = ellipse.MajorAxis, minor = new(-major.Y * ellipse.Ratio, major.X * ellipse.Ratio);
                    for (int i = 0; i <= 64; i++)
                    {
                        double t = ellipse.StartParameter + sweep * i / 64;
                        Add(new Point2D(ellipse.Center.X + major.X * Math.Cos(t) + minor.X * Math.Sin(t),
                                        ellipse.Center.Y + major.Y * Math.Cos(t) + minor.Y * Math.Sin(t)));
                    }

                    break;
                case DxfSpline spline:
                    // The control polygon holds the curve
                    foreach (Point2D p in spline.ControlPoints)
                    {
                        Add(p);
                    }

                    break;
                case DxfPolyline polyline:
                    foreach (Point2D p in polyline.Points)
                    {
                        Add(p);
                    }

                    break;
            }
        }

        return xMin <= xMax ? (new Point2D(xMin, yMin), new Point2D(xMax, yMax)) : (Point2D.Origin, Point2D.Origin);
    }
}
