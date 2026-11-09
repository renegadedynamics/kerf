// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Kerf;

/// <summary>
/// A colour with opacity. Its components are sRGB, as people write colours (decision 1349, question 60): 0 to 1 each,
/// <c>new Color(1, 0.5, 0)</c> is orange. Formats that store linear RGB (STEP, glTF) convert with
/// <see cref="ToLinear"/>, as OCCT's <c>Quantity_Color</c> converts, so a colour read from a file and written again
/// keeps its stored values exactly.
/// </summary>
/// <remarks><c>default(Color)</c> is transparent black. Equality is exact, on the stored values.</remarks>
public readonly struct Color : IEquatable<Color>
{
    // Linear RGB as OCCT stores it (single precision), and the sRGB values computed from it
    private readonly float _linearR, _linearG, _linearB;

    /// <summary>A colour from sRGB components and an opacity, each from 0 to 1.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A value is outside 0 to 1, or NaN.</exception>
    public Color(double r, double g, double b, double a = 1)
    {
        Check(r, nameof(r));
        Check(g, nameof(g));
        Check(b, nameof(b));
        Check(a, nameof(a));
        var linear = new LinearColor(r, g, b, ColorSpace.TOC_sRGB);
        (_linearR, _linearG, _linearB) = ((float)linear.Red(), (float)linear.Green(), (float)linear.Blue());
        (R, G, B, A) = Srgb(linear);
        A = a;
    }

    private Color(LinearColor linear, double a)
    {
        (_linearR, _linearG, _linearB) = ((float)linear.Red(), (float)linear.Green(), (float)linear.Blue());
        (R, G, B, _) = Srgb(linear);
        A = a;
    }

    /// <summary>A colour from linear RGB components and an opacity, each from 0 to 1.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A value is outside 0 to 1, or NaN.</exception>
    public static Color FromLinear(double r, double g, double b, double a = 1)
    {
        Check(r, nameof(r));
        Check(g, nameof(g));
        Check(b, nameof(b));
        Check(a, nameof(a));
        return new Color(new LinearColor(r, g, b, ColorSpace.TocRgb), a);
    }

    /// <summary>The red component, sRGB.</summary>
    public double R { get; }

    /// <summary>The green component, sRGB.</summary>
    public double G { get; }

    /// <summary>The blue component, sRGB.</summary>
    public double B { get; }

    /// <summary>The opacity: 1 is opaque.</summary>
    public double A { get; }

    /// <summary>
    /// The colour OCCT names <paramref name="name"/> (its table of X11 colours: <c>"BLUE1"</c>, <c>"STEELBLUE"</c>, case
    /// ignored, OCCT's <c>Quantity_Color::ColorFromName</c>), opaque; false if OCCT has no colour of that name.
    /// </summary>
    public static bool TryFromName(string name, out Color color)
    {
        KernelArgumentNullException.ThrowIfNull(name);
        var linear = new LinearColor();
        if (!LinearColor.ColorFromName(name, ref linear))
        {
            color = default;
            return false;
        }

        color = new Color(linear, 1);
        return true;
    }

    /// <summary>The name of OCCT's named colour nearest this one (<c>"SIENNA4"</c>, OCCT's <c>Quantity_Color::Name</c>).</summary>
    public string NearestName() => LinearColor.StringName(new LinearColor(_linearR, _linearG, _linearB, ColorSpace.TocRgb).Name());

    /// <summary>The colour in linear RGB, as STEP and glTF store it.</summary>
    public (double R, double G, double B, double A) ToLinear() => (_linearR, _linearG, _linearB, A);

    /// <inheritdoc/>
    public bool Equals(Color other)
        => _linearR == other._linearR && _linearG == other._linearG && _linearB == other._linearB && A == other.A;

    /// <inheritdoc/>
    public override bool Equals(object? obj) => obj is Color other && Equals(other);

    /// <inheritdoc/>
    public override int GetHashCode() => HashCode.Combine(_linearR, _linearG, _linearB, A);

    /// <summary>Exact equality.</summary>
    public static bool operator ==(Color left, Color right) => left.Equals(right);

    /// <summary>Exact inequality.</summary>
    public static bool operator !=(Color left, Color right) => !left.Equals(right);

    /// <inheritdoc/>
    public override string ToString() => FormattableString.Invariant($"Color(sRGB {R:0.###}, {G:0.###}, {B:0.###}, alpha {A:0.###})");

    /// <summary>The engine's colour: OCCT's <c>Quantity_ColorRGBA</c>.</summary>
    internal ColorWithAlpha ToEngine() => new(_linearR, _linearG, _linearB, (float)A);

    /// <summary>The engine's colour as a public one, its stored values kept.</summary>
    internal static Color FromEngine(in ColorWithAlpha color) => new(color.GetRGB(), color.Alpha());

    private static (double R, double G, double B, double A) Srgb(in LinearColor linear)
    {
        linear.Values(out double r, out double g, out double b, ColorSpace.TOC_sRGB);
        return (r, g, b, 1);
    }

    private static void Check(double value, string name)
    {
        if (!(value is >= 0 and <= 1))
        {
            throw new KernelArgumentOutOfRangeException(name, value, "A colour's components are from 0 to 1.");
        }
    }
}
