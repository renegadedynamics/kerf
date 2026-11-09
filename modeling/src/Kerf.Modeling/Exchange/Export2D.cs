using System.Collections.Immutable;

namespace Kerf.Modeling;

/// <summary>A layer's line type in a 2D export: build123d's <c>LineType</c>, AutoCAD's ANSI and ISO patterns.</summary>
public enum LineType
{
    /// <summary>A solid line.</summary>
    Continuous,
    /// <summary>Border: __ __ . __ __ .</summary>
    Border,
    /// <summary>Border at half size.</summary>
    Border2,
    /// <summary>Border at twice the size.</summary>
    BorderX2,
    /// <summary>Center: ____ _ ____ _</summary>
    Center,
    /// <summary>Center at half size.</summary>
    Center2,
    /// <summary>Center at twice the size.</summary>
    CenterX2,
    /// <summary>Dash dot: __ . __ .</summary>
    DashDot,
    /// <summary>Dash dot at half size.</summary>
    DashDot2,
    /// <summary>Dash dot at twice the size.</summary>
    DashDotX2,
    /// <summary>Dashed: __ __ __</summary>
    Dashed,
    /// <summary>Dashed at half size.</summary>
    Dashed2,
    /// <summary>Dashed at twice the size.</summary>
    DashedX2,
    /// <summary>Divide: ____ . . ____ . .</summary>
    Divide,
    /// <summary>Divide at half size.</summary>
    Divide2,
    /// <summary>Divide at twice the size.</summary>
    DivideX2,
    /// <summary>Dot: . . . .</summary>
    Dot,
    /// <summary>Dot at half size.</summary>
    Dot2,
    /// <summary>Dot at twice the size.</summary>
    DotX2,
    /// <summary>Hidden: __ __ __, shorter than <see cref="Dashed"/>.</summary>
    Hidden,
    /// <summary>Hidden at half size.</summary>
    Hidden2,
    /// <summary>Hidden at twice the size.</summary>
    HiddenX2,
    /// <summary>Phantom: ______ __ __ ______</summary>
    Phantom,
    /// <summary>Phantom at half size.</summary>
    Phantom2,
    /// <summary>Phantom at twice the size.</summary>
    PhantomX2,
    /// <summary>ISO dash (ACAD_ISO02W100): __ __ __</summary>
    IsoDash,
    /// <summary>ISO dash space (ACAD_ISO03W100): __    __    __</summary>
    IsoDashSpace,
    /// <summary>ISO long-dash dot (ACAD_ISO04W100): ____ . ____ .</summary>
    IsoLongDashDot,
    /// <summary>ISO long-dash double-dot (ACAD_ISO05W100): ____ .. ____ ..</summary>
    IsoLongDashDoubleDot,
    /// <summary>ISO long-dash triple-dot (ACAD_ISO06W100): ____ ... ____ ...</summary>
    IsoLongDashTripleDot,
    /// <summary>ISO dot (ACAD_ISO07W100): . . . .</summary>
    IsoDot,
    /// <summary>ISO long-dash short-dash (ACAD_ISO08W100): ____ __ ____ __</summary>
    IsoLongDashShortDash,
    /// <summary>ISO long-dash double-short-dash (ACAD_ISO09W100): ____ __ __ ____</summary>
    IsoLongDashDoubleShortDash,
    /// <summary>ISO dash dot (ACAD_ISO10W100): __ . __ .</summary>
    IsoDashDot,
    /// <summary>ISO double-dash dot (ACAD_ISO11W100): __ __ . __ __ .</summary>
    IsoDoubleDashDot,
    /// <summary>ISO dash double-dot (ACAD_ISO12W100): __ . . __ . .</summary>
    IsoDashDoubleDot,
    /// <summary>ISO double-dash double-dot (ACAD_ISO13W100): __ __ . . __ __ . .</summary>
    IsoDoubleDashDoubleDot,
    /// <summary>ISO dash triple-dot (ACAD_ISO14W100): __ . . . __ . . .</summary>
    IsoDashTripleDot,
    /// <summary>ISO double-dash triple-dot (ACAD_ISO15W100): __ __ . . . __ __ . . .</summary>
    IsoDoubleDashTripleDot,
}

/// <summary>A colour of AutoCAD's index (ACI), build123d's <c>ColorIndex</c>.</summary>
public enum ColorIndex
{
#pragma warning disable CS1591 // Self-describing colours
    Red = 1, Yellow = 2, Green = 3, Cyan = 4, Blue = 5, Magenta = 6, Black = 7, Gray = 8, LightGray = 9,
#pragma warning restore CS1591
}

/// <summary>A DXF file's version.</summary>
public enum DxfVersion
{
    /// <summary>
    /// AutoCAD R12 (AC1009), for older programs such as some laser cutters' software: lines, circles and arcs as they are,
    /// ellipses and splines as polylines (R12 has neither), no units in the header, layer names in ASCII.
    /// </summary>
    R12 = 12,

    /// <summary>AutoCAD 2013 (AC1027), build123d's default: every entity exact, units in the header, UTF-8.</summary>
    R2013 = 2013,
}

/// <summary>A line type's pattern: its DXF name, description, and dash lengths in tenths of an inch.</summary>
/// <param name="Name">The name in the DXF file's LTYPE table (<c>HIDDEN</c>, <c>ACAD_ISO02W100</c>).</param>
/// <param name="Description">The pattern as text, as AutoCAD's acad.lin describes it.</param>
/// <param name="Pattern">The total length, then each element: a dash (positive), a gap (negative) or a dot (zero), in
/// tenths of an inch, as build123d's <c>LINETYPE_DEFS</c>.</param>
public sealed record LineTypeDefinition(string Name, string Description, ImmutableArray<double> Pattern);

/// <summary>
/// build123d's <c>Export2D</c>, the base of its 2D exporters: their defaults and line types. Only
/// <see cref="ExportDxf"/> derives from it here (build123d's <c>ExportSVG</c> isn't ported: the kernel's
/// <c>Kerf.Drawing.Svg</c> writes SVG).
/// </summary>
public abstract class Export2D
{
    private protected Export2D()
    {
    }

    /// <summary>How close two parameters on a spline may be and still count as the same (OCCT's own check).</summary>
    public const double ParametricTolerance = 1e-9;

    /// <summary>The default colour of the SVG exporter's layers.</summary>
    public const ColorIndex DefaultColorIndex = ColorIndex.Black;

    /// <summary>The default line weight of the SVG exporter's layers, in millimetres.</summary>
    public const double DefaultLineWeight = 0.09;

    /// <summary>The default line type.</summary>
    public const LineType DefaultLineType = LineType.Continuous;

    private static readonly ImmutableArray<LineTypeDefinition> Definitions =
    [
        new("Continuous", "Solid line", [0.0]),
        new("BORDER", "Border __ __ . __ __ . __ __ . __ __ . __ __ .", Ansi(0.5, -0.25, 0.5, -0.25, 0, -0.25)),
        new("BORDER2", "Border (.5x) __.__.__.__.__.__.__.__.__.__.__.", Ansi(0.25, -0.125, 0.25, -0.125, 0, -0.125)),
        new("BORDERX2", "Border (2x) ____  ____  .  ____  ____  .  ___", Ansi(1.0, -0.5, 1.0, -0.5, 0, -0.5)),
        new("CENTER", "Center ____ _ ____ _ ____ _ ____ _ ____ _ ____", Ansi(1.25, -0.25, 0.25, -0.25)),
        new("CENTER2", "Center (.5x) ___ _ ___ _ ___ _ ___ _ ___ _ ___", Ansi(0.75, -0.125, 0.125, -0.125)),
        new("CENTERX2", "Center (2x) ________  __  ________  __  _____", Ansi(2.5, -0.5, 0.5, -0.5)),
        new("DASHDOT", "Dash dot __ . __ . __ . __ . __ . __ . __ . __", Ansi(0.5, -0.25, 0, -0.25)),
        new("DASHDOT2", "Dash dot (.5x) _._._._._._._._._._._._._._._.", Ansi(0.25, -0.125, 0, -0.125)),
        new("DASHDOTX2", "Dash dot (2x) ____  .  ____  .  ____  .  ___", Ansi(1.0, -0.5, 0, -0.5)),
        new("DASHED", "Dashed __ __ __ __ __ __ __ __ __ __ __ __ __ _", Ansi(0.5, -0.25)),
        new("DASHED2", "Dashed (.5x) _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ ", Ansi(0.25, -0.125)),
        new("DASHEDX2", "Dashed (2x) ____  ____  ____  ____  ____  ___", Ansi(1.0, -0.5)),
        new("DIVIDE", "Divide ____ . . ____ . . ____ . . ____ . . ____", Ansi(0.5, -0.25, 0, -0.25, 0, -0.25)),
        new("DIVIDE2", "Divide (.5x) __..__..__..__..__..__..__..__.._", Ansi(0.25, -0.125, 0, -0.125, 0, -0.125)),
        new("DIVIDEX2", "Divide (2x) ________  .  .  ________  .  .  _", Ansi(1.0, -0.5, 0, -0.5, 0, -0.5)),
        new("DOT", "Dot . . . . . . . . . . . . . . . . . . . . . . . .", Ansi(0, -0.25)),
        new("DOT2", "Dot (.5x) ........................................", Ansi(0, -0.125)),
        new("DOTX2", "Dot (2x) .  .  .  .  .  .  .  .  .  .  .  .  .  .", Ansi(0, -0.5)),
        new("HIDDEN", "Hidden __ __ __ __ __ __ __ __ __ __ __ __ __ __", Ansi(0.25, -0.125)),
        new("HIDDEN2", "Hidden (.5x) _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ ", Ansi(0.125, -0.0625)),
        new("HIDDENX2", "Hidden (2x) ____ ____ ____ ____ ____ ____ ____ ", Ansi(0.5, -0.25)),
        new("PHANTOM", "Phantom ______  __  __  ______  __  __  ______ ", Ansi(1.25, -0.25, 0.25, -0.25, 0.25, -0.25)),
        new("PHANTOM2", "Phantom (.5x) ___ _ _ ___ _ _ ___ _ _ ___ _ _", Ansi(0.625, -0.125, 0.125, -0.125, 0.125, -0.125)),
        new("PHANTOMX2", "Phantom (2x) ____________    ____    ____   _", Ansi(2.5, -0.5, 0.5, -0.5, 0.5, -0.5)),
        new("ACAD_ISO02W100", "ISO dash __ __ __ __ __ __ __ __ __ __ __ __ __", Iso(12, -3)),
        new("ACAD_ISO03W100", "ISO dash space __    __    __    __    __    __", Iso(12, -18)),
        new("ACAD_ISO04W100", "ISO long-dash dot ____ . ____ . ____ . ____ . _", Iso(24, -3, 0, -3)),
        new("ACAD_ISO05W100", "ISO long-dash double-dot ____ .. ____ .. ____ . ", Iso(24, -3, 0, -3, 0, -3)),
        new("ACAD_ISO06W100", "ISO long-dash triple-dot ____ ... ____ ... ____", Iso(24, -3, 0, -3, 0, -3, 0, -3)),
        new("ACAD_ISO07W100", "ISO dot . . . . . . . . . . . . . . . . . . . . ", Iso(0, -3)),
        new("ACAD_ISO08W100", "ISO long-dash short-dash ____ __ ____ __ ____ _", Iso(24, -3, 6, -3)),
        new("ACAD_ISO09W100", "ISO long-dash double-short-dash ____ __ __ ____", Iso(24, -3, 6, -3, 6, -3)),
        new("ACAD_ISO10W100", "ISO dash dot __ . __ . __ . __ . __ . __ . __ . ", Iso(12, -3, 0, -3)),
        new("ACAD_ISO11W100", "ISO double-dash dot __ __ . __ __ . __ __ . __ _", Iso(12, -3, 12, -3, 0, -3)),
        new("ACAD_ISO12W100", "ISO dash double-dot __ . . __ . . __ . . __ . . ", Iso(12, -3, 0, -3, 0, -3)),
        new("ACAD_ISO13W100", "ISO double-dash double-dot __ __ . . __ __ . . _", Iso(12, -3, 12, -3, 0, -3, 0, -3)),
        new("ACAD_ISO14W100", "ISO dash triple-dot __ . . . __ . . . __ . . . _", Iso(12, -3, 0, -3, 0, -3, 0, -3)),
        new("ACAD_ISO15W100", "ISO double-dash triple-dot __ __ . . . __ __ . .", Iso(12, -3, 12, -3, 0, -3, 0, -3, 0, -3)),
    ];

    /// <summary>A line type's name, description and pattern (build123d's <c>LINETYPE_DEFS</c>).</summary>
    /// <exception cref="ArgumentOutOfRangeException">The line type isn't one of the enum's.</exception>
    public static LineTypeDefinition LineTypeDefinition(LineType lineType)
        => (int)lineType >= 0 && (int)lineType < Definitions.Length
            ? Definitions[(int)lineType]
            : throw new ArgumentOutOfRangeException(nameof(lineType), lineType, "Not a line type.");

    /// <summary>
    /// The drawing units in a tenth of an inch, which line type patterns are given in: 2.54 for millimetres, 0.1 for
    /// inches (build123d's <c>LTYPE_SCALE</c>, with micrometres added).
    /// </summary>
    /// <exception cref="ArgumentException">The unit isn't a length.</exception>
    public static double LineTypeScale(Unit unit) => unit switch
    {
        Unit.Micrometer => 2540,
        Unit.Millimeter => 2.54,
        Unit.Centimeter => 0.254,
        Unit.Meter => 0.00254,
        Unit.Inch => 0.1,
        Unit.Foot => 0.1 / 12,
        _ => throw new ArgumentException($"{unit} isn't a unit of length.", nameof(unit)),
    };

    // A pattern in inches as tenths of an inch, its total length first (build123d's ansi_pattern)
    private static ImmutableArray<double> Ansi(params double[] lengths)
        => [.. new[] { lengths.Sum(Math.Abs) }.Concat(lengths).Select(l => l * 10)];

    // A pattern in millimetres as tenths of an inch, its total length first (build123d's iso_pattern)
    private static ImmutableArray<double> Iso(params double[] lengths)
        => [.. new[] { lengths.Sum(Math.Abs) }.Concat(lengths).Select(l => l / 2.54)];
}
