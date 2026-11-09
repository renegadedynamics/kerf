// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using System.Globalization;
using Kerf.Fonts;
using Kerf.Geometry;

namespace Kerf.Drawing;

/// <summary>A drawing sheet's paper: ISO 216's A4 to A0 and ANSI Y14.1's A to E.</summary>
public enum PageSize
{
    /// <summary>ISO A4, 297 × 210 mm.</summary>
    A4,

    /// <summary>ISO A3, 420 × 297 mm.</summary>
    A3,

    /// <summary>ISO A2, 594 × 420 mm.</summary>
    A2,

    /// <summary>ISO A1, 841 × 594 mm.</summary>
    A1,

    /// <summary>ISO A0, 1189 × 841 mm.</summary>
    A0,

    /// <summary>ANSI A (US letter), 11 × 8.5 in.</summary>
    AnsiA,

    /// <summary>ANSI B (ledger), 17 × 11 in.</summary>
    AnsiB,

    /// <summary>ANSI C, 22 × 17 in.</summary>
    AnsiC,

    /// <summary>ANSI D, 34 × 22 in.</summary>
    AnsiD,

    /// <summary>ANSI E, 44 × 34 in.</summary>
    AnsiE,
}

/// <summary>Which way up a sheet's paper is.</summary>
public enum PageOrientation
{
    /// <summary>Wider than tall.</summary>
    Landscape,

    /// <summary>Taller than wide.</summary>
    Portrait,
}

/// <summary>What a sheet's title block says: plain text for each field, filled by the caller.</summary>
public sealed record TitleBlock
{
    /// <summary>The drawing's title.</summary>
    public string Title { get; init => field = value ?? throw new ArgumentNullException(nameof(value)); } = "";

    /// <summary>The part's number.</summary>
    public string PartNumber { get; init => field = value ?? throw new ArgumentNullException(nameof(value)); } = "";

    /// <summary>The scale as written; null for the views' scale (<see cref="Sheet.ScaleText"/>), or "As shown" where they
    /// differ.</summary>
    public string? Scale { get; init; }

    /// <summary>The units the part's dimensions are in.</summary>
    public string Units { get; init => field = value ?? throw new ArgumentNullException(nameof(value)); } = "mm";

    /// <summary>The date, as it is to be written (a sheet doesn't read the clock, so the same sheet is the same file).</summary>
    public string Date { get; init => field = value ?? throw new ArgumentNullException(nameof(value)); } = "";

    /// <summary>Who drew it.</summary>
    public string Author { get; init => field = value ?? throw new ArgumentNullException(nameof(value)); } = "";

    /// <summary>The sheet's number.</summary>
    public string Sheet { get; init => field = value ?? throw new ArgumentNullException(nameof(value)); } = "1 of 1";
}

/// <summary>A view on a sheet: a line drawing, its extent's centre placed at its <see cref="Center"/> on the sheet (in
/// millimetres from the paper's bottom left corner), drawn at <see cref="Scale"/> millimetres of paper to the
/// drawing's unit.</summary>
public sealed record SheetView
{
    /// <summary>A view of <paramref name="drawing"/> centred at <paramref name="center"/>, at <paramref name="scale"/>.</summary>
    /// <exception cref="ArgumentNullException">The drawing is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The scale isn't more than zero, or the centre isn't finite.</exception>
    public SheetView(LineDrawing drawing, Point2D center, double scale = 1)
    {
        ArgumentNullException.ThrowIfNull(drawing);
        if (!double.IsFinite(scale) || scale <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(scale), scale, "The scale must be more than zero.");
        }

        if (!double.IsFinite(center.X) || !double.IsFinite(center.Y))
        {
            throw new ArgumentOutOfRangeException(nameof(center), center, "The centre must be finite.");
        }

        Drawing = drawing;
        Center = center;
        Scale = scale;
    }

    /// <summary>The drawing.</summary>
    public LineDrawing Drawing { get; }

    /// <summary>Where its extent's centre goes on the sheet, in millimetres.</summary>
    public Point2D Center { get; }

    /// <summary>Millimetres of paper to the drawing's unit: 0.5 for 1:2.</summary>
    public double Scale { get; }
}

/// <summary>
/// A drawing sheet: paper of a standard size with a border, a title block in its bottom right corner, and views of a part
/// at chosen scales. <see cref="Draw"/> makes it one <see cref="LineDrawing"/> in millimetres, from the paper's bottom
/// left corner, which <see cref="Svg"/> writes on a page of the paper's size (<see cref="PageOptions"/>)
/// and Kerf.Modeling's DXF exporter writes as it is.
/// </summary>
/// <remarks>
/// Plain, not a drafting standard: a border 20 mm from the left edge (for filing, as ISO 5457) and 10 mm from the others,
/// and a title block 180 mm wide and 25 high, of two rows of cells, each with its label and the caller's text, in the
/// default font. Immutable.
/// </remarks>
public sealed record Sheet
{
    private const double LeftMargin = 20, Margin = 10, BlockWidth = 180, BottomRow = 10, TopRow = 15, Inset = 1.5;

    /// <summary>A sheet with no views and an empty title block.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The size or orientation isn't one of the enum's.</exception>
    public Sheet(PageSize size, PageOrientation orientation = PageOrientation.Landscape)
    {
        if (!Enum.IsDefined(size))
        {
            throw new ArgumentOutOfRangeException(nameof(size), size, "Not a page size.");
        }

        if (!Enum.IsDefined(orientation))
        {
            throw new ArgumentOutOfRangeException(nameof(orientation), orientation, "Not an orientation.");
        }

        Size = size;
        Orientation = orientation;
        (double length, double breadth) = size switch
        {
            PageSize.A4 => (297.0, 210.0),
            PageSize.A3 => (420.0, 297.0),
            PageSize.A2 => (594.0, 420.0),
            PageSize.A1 => (841.0, 594.0),
            PageSize.A0 => (1189.0, 841.0),
            PageSize.AnsiA => (279.4, 215.9),
            PageSize.AnsiB => (431.8, 279.4),
            PageSize.AnsiC => (558.8, 431.8),
            PageSize.AnsiD => (863.6, 558.8),
            _ => (1117.6, 863.6),
        };
        (Width, Height) = orientation == PageOrientation.Landscape ? (length, breadth) : (breadth, length);
    }

    /// <summary>A view or a note that doesn't fit: outside the border, over the title block, or past its cell
    /// (a warning): <c>drawing.sheet-overflow</c>.</summary>
    public static DiagnosticCode Overflow { get; } = new("drawing.sheet-overflow");

    /// <summary>The paper.</summary>
    public PageSize Size { get; }

    /// <summary>Which way up it is.</summary>
    public PageOrientation Orientation { get; }

    /// <summary>The paper's width in millimetres.</summary>
    public double Width { get; }

    /// <summary>The paper's height in millimetres.</summary>
    public double Height { get; }

    /// <summary>What the title block says.</summary>
    public TitleBlock TitleBlock { get; init => field = value ?? throw new ArgumentNullException(nameof(value)); } = new();

    /// <summary>The views, drawn in order.</summary>
    public ImmutableArray<SheetView> Views
    {
        get;
        init => field = value.IsDefault || value.Any(v => v is null) ? throw new ArgumentNullException(nameof(value), "A view is null.") : value;
    } = [];

    /// <summary>The inside of the border, where views go.</summary>
    public (Point2D Min, Point2D Max) Frame => (new Point2D(LeftMargin, Margin), new Point2D(Width - Margin, Height - Margin));

    /// <summary>The title block, in the frame's bottom right corner.</summary>
    public (Point2D Min, Point2D Max) TitleBlockArea
        => (new Point2D(Width - Margin - Math.Min(BlockWidth, Width - Margin - LeftMargin), Margin),
            new Point2D(Width - Margin, Margin + BottomRow + TopRow));

    /// <summary>A scale as a title block writes it: <c>1:2</c> for 0.5, <c>5:1</c> for 5, <c>1:1</c>.</summary>
    /// <exception cref="ArgumentOutOfRangeException">The scale isn't more than zero.</exception>
    public static string ScaleText(double scale)
    {
        if (!double.IsFinite(scale) || scale <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(scale), scale, "The scale must be more than zero.");
        }

        string N(double v) => Math.Round(v, 3).ToString("0.###", CultureInfo.InvariantCulture);
        return scale >= 1 ? $"{N(scale)}:1" : $"1:{N(1 / scale)}";
    }

    /// <summary>SVG options for a page of the paper's size, the sheet's corner at the page's, at full size in millimetres,
    /// with <paramref name="style"/>'s lines, notes and background.</summary>
    public SvgOptions PageOptions(SvgOptions? style = null) => (style ?? SvgOptions.Default) with
    {
        Unit = SvgUnit.Millimetre,
        PageWidth = Width,
        PageHeight = Height,
        Scale = 1,
        Margin = 0,
        Area = (Point2D.Origin, new Point2D(Width, Height)),
    };

    /// <summary>
    /// The sheet as a line drawing in millimetres from the paper's bottom left corner: the border and the title block's
    /// lines (visible, sharp), each view's lines placed and scaled, the title block's text and each view's notes.
    /// </summary>
    /// <returns>The drawing, with an <see cref="Overflow"/> warning for each view outside the frame or over the title block
    /// and each entry wider than its cell; or a failure where an entry can't be set (a character the default font hasn't).</returns>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public Result<LineDrawing> Draw(CancellationToken cancellationToken = default)
    {
        var edges = new List<DrawnEdge>();
        var notes = new List<DrawingNote>();
        var warnings = ImmutableArray.CreateBuilder<Diagnostic>();
        void Line(double x0, double y0, double x1, double y1)
        {
            var a = new Point2D(x0, y0);
            var b = new Point2D(x1, y1);
            double length = a.DistanceTo(b);
            edges.Add(new DrawnEdge(new TrimmedCurve2D(new LineCurve2D(a, new Direction2D(x1 - x0, y1 - y0)), 0, length), LineKind.Sharp, true, default));
        }

        // The border
        (Point2D f0, Point2D f1) = Frame;
        Line(f0.X, f0.Y, f1.X, f0.Y);
        Line(f1.X, f0.Y, f1.X, f1.Y);
        Line(f1.X, f1.Y, f0.X, f1.Y);
        Line(f0.X, f1.Y, f0.X, f0.Y);

        // The title block: a top row of the title and the part number, a bottom row of the rest
        (Point2D b0, Point2D b1) = TitleBlockArea;
        double width = b1.X - b0.X, middle = b0.Y + BottomRow, k = width / BlockWidth;
        Line(b0.X, b1.Y, b1.X, b1.Y);
        Line(b0.X, b0.Y, b0.X, b1.Y);
        Line(b0.X, middle, b1.X, middle);
        TitleBlock block = TitleBlock;
        string scale = block.Scale ?? (Views.Select(v => v.Scale).Distinct().ToList() switch
        {
            [] => "",
            [double one] => ScaleText(one),
            _ => "As shown",
        });
        (string Label, string Value, double Width, double Size)[] top = [("TITLE", block.Title, 120 * k, 5), ("PART NUMBER", block.PartNumber, 60 * k, 3.5)];
        (string Label, string Value, double Width, double Size)[] bottom =
        [
            ("SCALE", scale, 30 * k, 3.5), ("UNITS", block.Units, 25 * k, 3.5), ("DATE", block.Date, 40 * k, 3.5),
            ("AUTHOR", block.Author, 55 * k, 3.5), ("SHEET", block.Sheet, 30 * k, 3.5),
        ];
        foreach (var (row, y0, y1) in new[] { (top, middle, b1.Y), (bottom, b0.Y, middle) })
        {
            double x = b0.X;
            for (int i = 0; i < row.Length; i++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                (string label, string value, double cell, double size) = row[i];
                if (i > 0)
                {
                    Line(x, y0, x, y1);
                }

                Result<DrawingNote> labelNote = DrawingNote.Make(label, new Point2D(x + Inset, y1 - Inset),
                    new NoteStyle { Size = 1.8, Vertical = TextAlign.Top }, cancellationToken);
                notes.Add(labelNote.Value);
                if (value.Trim().Length > 0)
                {
                    Result<DrawingNote> valueNote = DrawingNote.Make(value, new Point2D(x + Inset, y0 + 2),
                        new NoteStyle { Size = size }, cancellationToken);
                    if (!valueNote.IsSuccess)
                    {
                        return Result<LineDrawing>.Failure(valueNote.Diagnostics);
                    }

                    notes.Add(valueNote.Value);
                    warnings.AddRange(valueNote.Diagnostics);
                    if (valueNote.Value.Bounds.Max.X > x + cell - 0.5 || valueNote.Value.Bounds.Max.Y > y1 - 2.5)
                    {
                        warnings.Add(Diagnostic.Warning(Overflow, $"The title block's {label.ToLowerInvariant()} doesn't fit its cell: '{value}'."));
                    }
                }

                x += cell;
            }
        }

        // The views
        for (int v = 0; v < Views.Length; v++)
        {
            SheetView view = Views[v];
            if (view.Drawing.GetBounds() is not { } bounds)
            {
                continue;
            }

            var centre = new Point2D(0.5 * (bounds.Min.X + bounds.Max.X), 0.5 * (bounds.Min.Y + bounds.Max.Y));
            Transform2D placed = Transform2D.Translation(new Vector2D(view.Center.X - centre.X, view.Center.Y - centre.Y))
                .Multiplied(Transform2D.Scale(centre, view.Scale));
            foreach (DrawnEdge edge in view.Drawing.Edges)
            {
                cancellationToken.ThrowIfCancellationRequested();
                edges.Add(edge with { Curve = (Curve2D)edge.Curve.Transformed(placed) });
            }

            foreach (DrawingNote note in view.Drawing.Notes)
            {
                Result<DrawingNote> moved = note.With(placed * note.Position, note.Style with { Size = note.Style.Size * view.Scale }, cancellationToken);
                if (!moved.IsSuccess)
                {
                    return Result<LineDrawing>.Failure(moved.Diagnostics);
                }

                notes.Add(moved.Value);
            }

            Point2D min = placed * bounds.Min, max = placed * bounds.Max;
            bool outside = min.X < f0.X || min.Y < f0.Y || max.X > f1.X || max.Y > f1.Y;
            bool overBlock = max.X > b0.X && min.X < b1.X && min.Y < b1.Y && max.Y > b0.Y;
            if (outside || overBlock)
            {
                warnings.Add(Diagnostic.Warning(Overflow, string.Create(CultureInfo.InvariantCulture,
                    $"View {v + 1} ({min.X:0.#}, {min.Y:0.#}) to ({max.X:0.#}, {max.Y:0.#}) is {(outside ? "outside the border" : "over the title block")}.")));
            }
        }

        return Result<LineDrawing>.Success(new LineDrawing(Camera.Top, edges, notes), warnings.ToImmutable());
    }
}
