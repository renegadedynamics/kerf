using System.Collections.Immutable;
using System.Text;
using Kerf.Drawing;
using Kerf.Fonts;

namespace Kerf.Modeling;

/// <summary>A layer of a DXF export: its name, and the colour, line weight and line type of what is drawn on it.</summary>
/// <param name="Name">The name; <c>"0"</c> is the default layer, which every DXF file has.</param>
/// <param name="Color">The colour; null for AutoCAD's default, white on black and black on white.</param>
/// <param name="LineWeight">The line weight in millimetres; null for the default.</param>
/// <param name="LineType">The line type; null for a solid line.</param>
public sealed record DxfLayer(string Name, ColorIndex? Color, double? LineWeight, LineType? LineType);

/// <summary>
/// build123d's <c>ExportDXF</c>: shapes' edges, and line drawings, written as a DXF file of 2D entities on layers, for
/// other CAD programs and laser cutters. Written by Kerf's own code (build123d uses ezdxf).
/// </summary>
/// <remarks>
/// <para>
/// An exporter is immutable: <see cref="AddLayer"/>, <see cref="AddShape(IShape, string)"/> and
/// <see cref="AddDrawing"/> each return a new one, so chain them, as build123d's return values are used:
/// <c>new ExportDxf().AddLayer("cut", ColorIndex.Red).AddShape(sketch, "cut").Write("part.dxf")</c>.
/// </para>
/// <para>
/// Each edge is drawn in the XY plane, as it is seen from above: a line as a LINE, a circle as a CIRCLE, an arc of
/// one as an ARC, an ellipse or an arc of one as an ELLIPSE, and a B-spline or Bézier curve as a SPLINE, exactly, with
/// its weights if it's rational. A curve none of those can hold (an offset curve, a parabola) is a polyline within
/// <see cref="Tolerance"/>. Edges off the XY plane are projected onto it, and <see cref="Write(Stream, CancellationToken)"/>
/// warns of them (<see cref="ModelingDiagnostics.DxfNotPlanar"/>), as build123d does. Coordinates are written in the
/// model's units as they are, and <see cref="Unit"/> names them in the header.
/// </para>
/// <para>
/// <c>Text</c>'s faces are shapes like any other, written as their edges. A drawing's notes (<see cref="DrawingNote"/>,
/// <see cref="AddNotes"/>) are written as their outlines by default, each contour a closed polyline within
/// <see cref="Tolerance"/>, filled in DXF 2013 by a solid HATCH under each glyph's contours (<see cref="FillOutlines"/>),
/// or as TEXT with a text style for each font (<see cref="TextOutput"/>).
/// </para>
/// </remarks>
public sealed class ExportDxf : Export2D
{
    private readonly ImmutableList<DxfLayer> _layers;
    private readonly ImmutableList<DxfEntity> _entities;
    private readonly int _notPlanar;
    private readonly int _asOutlines;

    /// <summary>An exporter with nothing in it but the default layer, <c>"0"</c>.</summary>
    /// <param name="version">The file's version: 2013 by default, as build123d; <see cref="DxfVersion.R12"/> for older
    /// programs.</param>
    /// <param name="unit">The unit the header gives for the coordinates: micrometres, millimetres (the default),
    /// centimetres, metres, inches or feet.</param>
    /// <param name="color">The default layer's colour; null for AutoCAD's default.</param>
    /// <param name="lineWeight">The default layer's line weight in millimetres, one of DXF's standard weights (0.05,
    /// 0.09, 0.13, 0.15, 0.18, 0.2, 0.25, 0.3, 0.35, 0.4, 0.5 … 2.11); null for the default.</param>
    /// <param name="lineType">The default layer's line type; null for a solid line.</param>
    /// <param name="tolerance">How far a polyline written for a curve the version can't hold may be from the curve, in
    /// model units.</param>
    /// <param name="textOutput">How notes are written: as their outlines (the default), or as text.</param>
    /// <param name="fillOutlines">Whether notes written as outlines are filled, in DXF 2013, as the SVG fills them (the
    /// default); false for their contours alone, for a laser cutter to engrave the outlines, say. R12 has no fill, so
    /// its notes are their contours either way.</param>
    /// <exception cref="ArgumentException">The unit isn't a length, the version or text output isn't one of the enum's,
    /// the line weight isn't a standard one, or the tolerance isn't more than zero.</exception>
    public ExportDxf(DxfVersion version = DxfVersion.R2013, Unit unit = Unit.Millimeter, ColorIndex? color = null,
                     double? lineWeight = null, LineType? lineType = null, double tolerance = 1e-3,
                     TextOutput textOutput = TextOutput.Outlines, bool fillOutlines = true)
    {
        if (version is not (DxfVersion.R12 or DxfVersion.R2013))
            throw new ArgumentException($"{version} isn't a DXF version Kerf writes.", nameof(version));
        if (!Enum.IsDefined(textOutput))
            throw new ArgumentException($"{textOutput} isn't a way of writing text.", nameof(textOutput));
        _ = LineTypeScale(unit);
        if (!(tolerance > 0) || double.IsInfinity(tolerance))
            throw new ArgumentException("The tolerance must be more than zero.", nameof(tolerance));
        Version = version;
        Unit = unit;
        Tolerance = tolerance;
        TextOutput = textOutput;
        FillOutlines = fillOutlines;
        _layers = [Layer("0", color, lineWeight, lineType, nameof(lineWeight))];
        _entities = [];
    }

    private ExportDxf(ExportDxf from, ImmutableList<DxfLayer> layers, ImmutableList<DxfEntity> entities, int notPlanar, int asOutlines = -1)
    {
        Version = from.Version;
        Unit = from.Unit;
        Tolerance = from.Tolerance;
        TextOutput = from.TextOutput;
        FillOutlines = from.FillOutlines;
        _layers = layers;
        _entities = entities;
        _notPlanar = notPlanar;
        _asOutlines = asOutlines < 0 ? from._asOutlines : asOutlines;
    }

    /// <summary>The file's version.</summary>
    public DxfVersion Version { get; }

    /// <summary>The unit of the coordinates, named in the header.</summary>
    public Unit Unit { get; }

    /// <summary>How far a polyline written for a curve may be from it, in model units.</summary>
    public double Tolerance { get; }

    /// <summary>How notes are written: as their outlines, or as text.</summary>
    public TextOutput TextOutput { get; }

    /// <summary>Whether notes written as outlines are filled with solid hatches (DXF 2013 only), or are their contours
    /// alone.</summary>
    public bool FillOutlines { get; }

    /// <summary>The layers, the default layer <c>"0"</c> first.</summary>
    public IReadOnlyList<DxfLayer> Layers => _layers;

    /// <summary>The number of entities (lines, arcs, splines…) added so far.</summary>
    public int EntityCount => _entities.Count;

    /// <summary>The number of edges added that weren't in the XY plane, and were projected onto it.</summary>
    public int NotPlanarEdgeCount => _notPlanar;

    /// <summary>This exporter with a new layer.</summary>
    /// <param name="name">The layer's name: not empty, unique (ignoring case, as DXF does), and without
    /// <c>&lt; &gt; / \ " : ; ? * | = `</c> or control characters; in ASCII for <see cref="DxfVersion.R12"/>.</param>
    /// <param name="color">The colour of what is drawn on it; null for AutoCAD's default.</param>
    /// <param name="lineWeight">Its line weight in millimetres, one of DXF's standard weights; null for the default.</param>
    /// <param name="lineType">Its line type; null for a solid line.</param>
    /// <exception cref="ArgumentException">The name is empty, invalid or already used, or the line weight isn't a
    /// standard one.</exception>
    public ExportDxf AddLayer(string name, ColorIndex? color = null, double? lineWeight = null, LineType? lineType = null)
    {
        ArgumentNullException.ThrowIfNull(name);
        CheckName(name, Version);
        if (FindLayer(name) is not null)
            throw new ArgumentException($"There is already a layer named {name}.", nameof(name));
        return new ExportDxf(this, _layers.Add(Layer(name, color, lineWeight, lineType, nameof(lineWeight))), _entities, _notPlanar);
    }

    /// <summary>This exporter with a shape's edges added on a layer (each distinct edge once).</summary>
    /// <param name="shape">The shape: an edge, a wire, a face, a sketch, or anything else with edges.</param>
    /// <param name="layer">The layer's name; empty for the default layer, <c>"0"</c>.</param>
    /// <exception cref="ArgumentException">There is no layer of that name: add it first with <see cref="AddLayer"/>.</exception>
    public ExportDxf AddShape(IShape shape, string layer = "")
    {
        ArgumentNullException.ThrowIfNull(shape);
        string name = LayerName(layer);
        var entities = _entities.ToBuilder();
        int notPlanar = _notPlanar;
        AddEdges(shape, name, entities, ref notPlanar);
        return new ExportDxf(this, _layers, entities.ToImmutable(), notPlanar);
    }

    /// <summary>This exporter with shapes' edges added on a layer.</summary>
    /// <inheritdoc cref="AddShape(IShape, string)"/>
    public ExportDxf AddShape<T>(IEnumerable<T> shapes, string layer = "") where T : IShape
    {
        ArgumentNullException.ThrowIfNull(shapes);
        string name = LayerName(layer);
        var entities = _entities.ToBuilder();
        int notPlanar = _notPlanar;
        foreach (T shape in shapes)
        {
            if (shape is null)
                throw new ArgumentNullException(nameof(shapes), "A shape is null.");
            AddEdges(shape, name, entities, ref notPlanar);
        }
        return new ExportDxf(this, _layers, entities.ToImmutable(), notPlanar);
    }

    /// <summary>
    /// This exporter with a line drawing's lines added (from <c>ProjectLines</c>): its visible lines on one layer and
    /// its hidden ones on another, as <c>Kerf.Drawing.Svg</c> separates them. In the view plane's coordinates, X right
    /// and Y up, full size.
    /// </summary>
    /// <param name="drawing">The drawing.</param>
    /// <param name="visibleLayer">The visible lines' layer; added, solid and 0.35 mm wide, if there's none of that name.</param>
    /// <param name="hiddenLayer">The hidden lines' layer; added, grey, 0.18 mm wide and <see cref="LineType.Hidden2"/>, if
    /// there's none of that name; null to leave the hidden lines out.</param>
    /// <param name="notesLayer">The notes' layer, added if there's none of that name and the drawing has notes; null to
    /// leave the notes out.</param>
    /// <exception cref="ArgumentException">A layer name is invalid.</exception>
    public ExportDxf AddDrawing(LineDrawing drawing, string visibleLayer = "visible", string? hiddenLayer = "hidden",
                                string? notesLayer = "notes")
    {
        ArgumentNullException.ThrowIfNull(drawing);
        ArgumentNullException.ThrowIfNull(visibleLayer);
        var layers = _layers;
        string visible = EnsureLayer(ref layers, visibleLayer, null, 0.35, null);
        string? hidden = hiddenLayer is null ? null : EnsureLayer(ref layers, hiddenLayer, ColorIndex.Gray, 0.18, LineType.Hidden2);
        var entities = _entities.ToBuilder();
        var converter = new CurveConverter(Version, Tolerance);
        foreach (DrawnEdge edge in drawing.Edges)
        {
            if (edge.IsVisible)
                converter.Add(edge.Curve, edge.Curve.FirstParameter, edge.Curve.LastParameter, false, visible, entities);
            else if (hidden is not null)
                converter.Add(edge.Curve, edge.Curve.FirstParameter, edge.Curve.LastParameter, false, hidden, entities);
        }
        int asOutlines = _asOutlines;
        if (notesLayer is not null && !drawing.Notes.IsEmpty)
        {
            string notes = EnsureLayer(ref layers, notesLayer, null, null, null);
            foreach (DrawingNote note in drawing.Notes)
                AddNote(note, notes, entities, ref asOutlines);
        }
        return new ExportDxf(this, layers, entities.ToImmutable(), _notPlanar, asOutlines);
    }

    /// <summary>This exporter with notes added on a layer, as outlines or text (<see cref="TextOutput"/>), in the view
    /// plane's coordinates as they are.</summary>
    /// <param name="notes">The notes, from <see cref="DrawingNote.Make"/>.</param>
    /// <param name="layer">The layer's name; empty for the default layer, <c>"0"</c>.</param>
    /// <exception cref="ArgumentException">There is no layer of that name: add it first with <see cref="AddLayer"/>.</exception>
    public ExportDxf AddNotes(IEnumerable<DrawingNote> notes, string layer = "")
    {
        ArgumentNullException.ThrowIfNull(notes);
        string name = LayerName(layer);
        var entities = _entities.ToBuilder();
        int asOutlines = _asOutlines;
        foreach (DrawingNote note in notes)
        {
            if (note is null)
                throw new ArgumentNullException(nameof(notes), "A note is null.");
            AddNote(note, name, entities, ref asOutlines);
        }
        return new ExportDxf(this, _layers, entities.ToImmutable(), _notPlanar, asOutlines);
    }

    // A note as TEXT, a line each, or as closed polylines, a contour each, over solid hatches in 2013; an R12 file's
    // text is ASCII, so a note that isn't is written as its outlines there
    private void AddNote(DrawingNote note, string layer, ImmutableList<DxfEntity>.Builder entities, ref int asOutlines)
    {
        if (TextOutput == TextOutput.Text)
        {
            if (Version != DxfVersion.R12 || note.Lines.All(l => l.Text.All(c => c is >= ' ' and <= '~')))
            {
                DxfTextStyle style = StyleOf(note.Style.Font);
                double rotation = note.Style.Rotation % 360;
                if (rotation < 0)
                    rotation += 360;
                int alignment = note.Style.Horizontal switch { TextAlign.Center => 1, TextAlign.Right => 2, _ => 0 };
                foreach (NoteLine line in note.Lines)
                {
                    entities.Add(new DxfText(layer, Escaped(line.Text), line.Anchor, note.CapHeight, rotation >= 360 ? 0 : rotation,
                                             alignment, style, note.Bounds.Min, note.Bounds.Max));
                }
                return;
            }
            asOutlines++;
        }

        var contours = new List<ImmutableArray<Point2D>>(note.Contours.Length);
        foreach (ImmutableArray<Curve2D> contour in note.Contours)
        {
            var points = new List<Point2D>();
            foreach (Curve2D curve in contour)
            {
                List<Point2D> piece = curve is TrimmedCurve2D { BasisCurve: LineCurve2D }
                    ? [curve.Evaluate(curve.FirstParameter), curve.Evaluate(curve.LastParameter)]
                    : CurveConverter.Flatten(curve.Evaluate, curve.FirstParameter, curve.LastParameter, Tolerance);
                points.AddRange(points.Count == 0 ? piece : piece.Skip(1));
            }
            if (points.Count > 1 && points[^1].DistanceTo(points[0]) <= 1e-12 * Math.Max(1, points[0].DistanceTo(Point2D.Origin)))
                points.RemoveAt(points.Count - 1);
            if (points.Count >= 2)
                contours.Add([.. points]);
        }

        // Filled in 2013, as the SVG fills them: a hatch an outer contour, with the holes in it, and the contours over
        // the fills, so a reader that draws no hatches (or a laser cutter) still has the outlines
        if (FillOutlines && Version != DxfVersion.R12)
        {
            foreach (ImmutableArray<ImmutableArray<Point2D>> region in Regions(contours))
                entities.Add(new DxfHatch(layer, region));
        }
        foreach (ImmutableArray<Point2D> contour in contours)
            entities.Add(new DxfPolyline(layer, contour, Closed: true));
    }

    /// <summary>
    /// A note's contours as the regions they fill: each outer contour (counter-clockwise, decision 2030) with the holes
    /// (clockwise) whose smallest enclosing outer contour it is, as ezdxf groups paths for its hatches. A glyph's region
    /// is filled on its own, so glyphs that overlap (kerned, or a font's overlapping contours) stay filled where the
    /// non-zero rule fills them, where one odd-parity hatch of the whole note would leave the overlap empty. A contour of
    /// no area, or a hole in no outer contour, fills nothing.
    /// </summary>
    internal static List<ImmutableArray<ImmutableArray<Point2D>>> Regions(IReadOnlyList<ImmutableArray<Point2D>> contours)
    {
        var areas = contours.Select(SignedArea).ToList();
        var outers = Enumerable.Range(0, contours.Count).Where(i => areas[i] > 0).ToList();
        var boxes = contours.Select(Box).ToList();
        var holes = outers.ToDictionary(i => i, _ => new List<ImmutableArray<Point2D>>());
        for (int h = 0; h < contours.Count; h++)
        {
            if (!(areas[h] < 0))
                continue;
            Point2D p = contours[h][0];
            int best = -1;
            foreach (int o in outers)
            {
                if (boxes[h].Min.X >= boxes[o].Min.X && boxes[h].Max.X <= boxes[o].Max.X && boxes[h].Min.Y >= boxes[o].Min.Y
                    && boxes[h].Max.Y <= boxes[o].Max.Y && (best < 0 || areas[o] < areas[best]) && Inside(p, contours[o]))
                    best = o;
            }
            if (best >= 0)
                holes[best].Add(contours[h]);
        }
        return [.. outers.Select(o => (ImmutableArray<ImmutableArray<Point2D>>)[contours[o], .. holes[o]])];
    }

    // A closed polygon's area by the shoelace formula, positive counter-clockwise
    internal static double SignedArea(ImmutableArray<Point2D> polygon)
    {
        double sum = 0;
        for (int i = 0, j = polygon.Length - 1; i < polygon.Length; j = i++)
            sum += (polygon[j].X - polygon[i].X) * (polygon[j].Y + polygon[i].Y);
        return 0.5 * sum;
    }

    private static (Point2D Min, Point2D Max) Box(ImmutableArray<Point2D> polygon)
        => (new Point2D(polygon.Min(p => p.X), polygon.Min(p => p.Y)), new Point2D(polygon.Max(p => p.X), polygon.Max(p => p.Y)));

    // Whether a point is inside a polygon, by the crossings of a ray to +X
    private static bool Inside(Point2D p, ImmutableArray<Point2D> polygon)
    {
        bool inside = false;
        for (int i = 0, j = polygon.Length - 1; i < polygon.Length; j = i++)
        {
            Point2D a = polygon[i], b = polygon[j];
            if ((a.Y > p.Y) != (b.Y > p.Y) && p.X < a.X + (p.Y - a.Y) * (b.X - a.X) / (b.Y - a.Y))
                inside = !inside;
        }
        return inside;
    }

    // A text style for a font: named by its PostScript name, with its file (Kerf's own Liberation Sans by the file it
    // ships as) and its family for AutoCAD
    private static DxfTextStyle StyleOf(Font font)
    {
        bool bold = font.Style is FontStyle.Bold or FontStyle.BoldItalic, italic = font.Style is FontStyle.Italic or FontStyle.BoldItalic;
        var name = new StringBuilder();
        foreach (char c in font.PostScriptName.Length > 0 ? font.PostScriptName : font.FamilyName)
            name.Append(c is (>= 'A' and <= 'Z') or (>= 'a' and <= 'z') or (>= '0' and <= '9') or '-' or '_' ? c : '_');
        if (name.Length == 0)
            name.Append("Font");
        string file = ReferenceEquals(FontLibrary.Default(font.Style), font)
            ? $"LiberationSans-{font.Style}.ttf"
            : name + (font.HasCubicOutlines ? ".otf" : ".ttf");
        return new DxfTextStyle(name.ToString(), file, font.FamilyName, bold, italic);
    }

    // AutoCAD reads %% as the start of a control code (%%d, %%c, %%p); %%% is a percent sign
    private static string Escaped(string text) => text.Contains("%%", StringComparison.Ordinal) ? text.Replace("%", "%%%", StringComparison.Ordinal) : text;

    /// <summary>Writes the DXF to a stream (UTF-8 for 2013, ASCII for R12; <c>\r\n</c> line ends). The same exporter
    /// gives the same bytes on every machine.</summary>
    /// <returns>Success, with a warning if edges were projected onto the XY plane; or a failure if the stream couldn't
    /// be written.</returns>
    /// <exception cref="ArgumentNullException">The stream is null.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public Result Write(Stream stream, CancellationToken cancellationToken = default)
    {
        ArgumentNullException.ThrowIfNull(stream);
        byte[] bytes = DxfWriter.Encode(WriteToString(cancellationToken), Version);
        try
        {
            stream.Write(bytes, 0, bytes.Length);
            stream.Flush();
        }
        catch (Exception e) when (e is IOException or NotSupportedException or ObjectDisposedException or UnauthorizedAccessException)
        {
            return Result.Failure(Diagnostic.Error(Exporters3D.WriteFailed, $"The DXF couldn't be written: {e.Message}"));
        }
        return Result.Success(Warnings());
    }

    /// <summary>Writes the DXF to a file, whole or not at all.</summary>
    /// <inheritdoc cref="Write(Stream, CancellationToken)"/>
    /// <exception cref="DirectoryNotFoundException">The file's folder doesn't exist.</exception>
    public Result Write(string path, CancellationToken cancellationToken = default)
        => Exporters3D.ToFile(path, s => Write(s, cancellationToken));

    /// <summary>The DXF as text.</summary>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public string WriteToString(CancellationToken cancellationToken = default)
        => new DxfWriter(Version, Unit, _layers, _entities, cancellationToken).Write();

    private ImmutableArray<Diagnostic> Warnings()
    {
        var warnings = ImmutableArray.CreateBuilder<Diagnostic>();
        if (_notPlanar > 0)
            warnings.Add(Diagnostic.Warning(ModelingDiagnostics.DxfNotPlanar,
                $"{_notPlanar} edge{(_notPlanar == 1 ? " wasn't" : "s weren't")} in the XY plane, and {(_notPlanar == 1 ? "was" : "were")} projected onto it"));
        if (_asOutlines > 0)
            warnings.Add(Diagnostic.Warning(ModelingDiagnostics.DxfTextAsOutlines,
                $"{_asOutlines} note{(_asOutlines == 1 ? " isn't" : "s aren't")} ASCII, which an R12 file's text must be, and {(_asOutlines == 1 ? "was" : "were")} written as outlines"));
        return warnings.ToImmutable();
    }

    private void AddEdges(IShape shape, string layer, ImmutableList<DxfEntity>.Builder entities, ref int notPlanar)
    {
        if (shape.IsEmpty)
            return;
        var converter = new CurveConverter(Version, Tolerance);
        var xy = new Plane3D();
        foreach (Edge edge in shape.Edges())
        {
            if (!IsInPlane(edge))
                notPlanar++;
            // An offset curve's projection is only an approximation (OCCT's), of its own accuracy: flattened from the
            // curve itself, within the tolerance
            if (edge.TryGetCurve(out Curve3D? curve3D, out Interval range3D) && Basis(curve3D) is OffsetCurve3D)
            {
                converter.AddFlattened(u => { Point3D p = curve3D.Evaluate(u); return new Point2D(p.X, p.Y); },
                                       range3D.Min, range3D.Max, layer, entities);
                continue;
            }
            if (!edge.TryGetCurveOnPlane(xy, out Curve2D? curve, out Interval range))
                continue;
            converter.Add(curve, range.Min, range.Max, edge.AsShape().Orientation == Orientation.Reversed, layer, entities);
        }
    }

    private static Curve3D Basis(Curve3D curve)
    {
        while (curve is TrimmedCurve3D trimmed)
            curve = trimmed.BasisCurve;
        return curve;
    }

    // Whether an edge lies in the XY plane: its ends and points along it within 1e-6 of z = 0, build123d's test of
    // each point it writes
    private static bool IsInPlane(Edge edge)
    {
        if (!edge.TryGetCurve(out Curve3D? curve, out Interval range))
            return true;
        const int samples = 16;
        for (int i = 0; i <= samples; i++)
        {
            double u = i == samples ? range.Max : range.Min + (range.Max - range.Min) * i / samples;
            if (Math.Abs(curve.Evaluate(u).Z) > 1e-6)
                return false;
        }
        return true;
    }

    private string LayerName(string layer)
    {
        ArgumentNullException.ThrowIfNull(layer);
        if (layer.Length == 0)
            return "0";
        return FindLayer(layer)?.Name
               ?? throw new ArgumentException($"There is no layer named {layer}: add it first with AddLayer.", nameof(layer));
    }

    private DxfLayer? FindLayer(string name) => FindLayer(_layers, name);

    private static DxfLayer? FindLayer(ImmutableList<DxfLayer> layers, string name)
        => layers.FirstOrDefault(l => string.Equals(l.Name, name, StringComparison.OrdinalIgnoreCase));

    private string EnsureLayer(ref ImmutableList<DxfLayer> layers, string name, ColorIndex? color, double? weight, LineType? type)
    {
        if (name.Length == 0)
            return "0";
        if (FindLayer(layers, name) is { } existing)
            return existing.Name;
        CheckName(name, Version);
        layers = layers.Add(Layer(name, color, weight, type, nameof(name)));
        return name;
    }

    private static DxfLayer Layer(string name, ColorIndex? color, double? lineWeight, LineType? lineType, string weightParameter)
    {
        if (color is { } c && !Enum.IsDefined(c))
            throw new ArgumentException($"{c} isn't a colour of the index.", nameof(color));
        if (lineType is { } t)
            _ = LineTypeDefinition(t);
        if (lineWeight is { } w && DxfWriter.LineWeight(w) is null)
            throw new ArgumentException(
                $"{w} mm isn't one of DXF's line weights: {string.Join(", ", DxfWriter.LineWeights.Where(v => v > 0).Select(v => (v / 100.0).ToString(System.Globalization.CultureInfo.InvariantCulture)))}.",
                weightParameter);
        return new DxfLayer(name, color, lineWeight, lineType);
    }

    // DXF's rule for table names (ezdxf's), with control characters, which would break the file's lines
    private static void CheckName(string name, DxfVersion version)
    {
        if (name.Length == 0)
            throw new ArgumentException("A layer's name can't be empty.", nameof(name));
        if (name.Length > 255)
            throw new ArgumentException("A layer's name can't be longer than 255 characters.", nameof(name));
        foreach (char c in name)
        {
            if (c < ' ' || c == '\u007f' || "<>/\\\":;?*|=`".Contains(c))
                throw new ArgumentException($"A layer's name can't contain '{(c < ' ' ? $"\\u{(int)c:x4}" : c)}': {name}", nameof(name));
            if (version == DxfVersion.R12 && c > '~')
                throw new ArgumentException($"An R12 file's layer names are ASCII: {name}", nameof(name));
        }
        if (name != name.Trim())
            throw new ArgumentException($"A layer's name can't start or end with a space: '{name}'", nameof(name));
    }
}
