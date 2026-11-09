// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Immutable;
using Kerf.Engine.Fonts;

namespace Kerf.Fonts;

/// <summary>
/// A TrueType or OpenType font, read by Kerf's own managed reader (unit 13D): its names, metrics, Unicode map, pair kerning
/// and glyph outlines, TrueType's quadratic (<c>glyf</c>) or CFF's cubic (<c>CFF </c>), from a <c>.ttf</c>, <c>.otf</c> or
/// one font of a <c>.ttc</c> collection. OCCT reads fonts through FreeType (<c>Font_FTFont</c>), which isn't managed; this
/// reads what OCCT's <c>StdPrs_BRepFont</c> and <c>Font_TextFormatter</c> ask FreeType for, and gives the same outlines.
/// </summary>
/// <remarks>
/// Immutable and safe to share between threads: a font holds its file's bytes and reads them for each request. Its tables
/// are checked when it is read, and every offset and count in them is checked against its table, so a damaged or hostile
/// file fails with a diagnostic (<see cref="Malformed"/>, <see cref="LimitExceeded"/> or <see cref="Unsupported"/>), never
/// an exception, a hang or a read past its data.
/// </remarks>
public sealed class Font
{
    private readonly FontData _file;
    private readonly CharacterMap _map;
    private readonly PairKerning _kerning;
    private readonly FontData _hmtx;
    private readonly int _metricCount;
    private readonly TrueTypeGlyphs? _trueType;
    private readonly CffGlyphs? _cff;

    /// <summary>Data that isn't a font, or a font damaged past reading: <c>font.malformed</c>.</summary>
    public static DiagnosticCode Malformed { get; } = new("font.malformed");

    /// <summary>A font past a <see cref="FontReadLimits"/> limit: <c>font.limit-exceeded</c>.</summary>
    public static DiagnosticCode LimitExceeded { get; } = new("font.limit-exceeded");

    /// <summary>A font in a form the reader doesn't read (WOFF, CFF2, Type 1, bitmaps only): <c>font.unsupported</c>.</summary>
    public static DiagnosticCode Unsupported { get; } = new("font.unsupported");

    private Font(FontData file, int faceIndex, SfntFile sfnt, FontReadLimits limits)
    {
        _file = file;
        FaceIndex = faceIndex;

        FontData head = sfnt.Table("head");
        UnitsPerEm = head.U16(18);
        if (UnitsPerEm is < 16 or > 16384)
        {
            throw FontData.Malformed($"its units per em ({UnitsPerEm}) are outside 16 to 16,384");
        }

        int macStyle = head.U16(44);
        int locaFormat = head.I16(50);

        FontData maxp = sfnt.Table("maxp");
        GlyphCount = maxp.U16(4);

        FontData hhea = sfnt.Table("hhea");
        int ascender = hhea.I16(4), descender = hhea.I16(6), lineGap = hhea.I16(8);
        _metricCount = hhea.U16(34);
        _hmtx = sfnt.Table("hmtx");
        if (_metricCount == 0 && GlyphCount > 0)
        {
            throw FontData.Malformed("its hhea gives no horizontal metrics");
        }

        _hmtx.CheckArray(0, _metricCount, 4, "the hmtx table");

        int fsSelection = -1;
        int height = ascender - descender + lineGap;
        if (sfnt.TryGetTable("OS/2", out FontData os2) && os2.Length >= 78)
        {
            fsSelection = os2.U16(62);
            Weight = os2.U16(4);
            // FreeType's FT_Face metrics: OS/2's typographic ones where it says to use them (USE_TYPO_METRICS), else
            // hhea's, else OS/2's typographic ones, else its Windows ones
            if ((fsSelection & 0x80) != 0)
            {
                int typoAscender = os2.I16(68), typoDescender = os2.I16(70), typoGap = os2.I16(72);
                (ascender, descender, height) = (typoAscender, typoDescender, typoAscender - typoDescender + typoGap);
            }
            else if (ascender == 0 && descender == 0)
            {
                int typoAscender = os2.I16(68), typoDescender = os2.I16(70), typoGap = os2.I16(72);
                if (typoAscender != 0 || typoDescender != 0)
                {
                    (ascender, descender, height) = (typoAscender, typoDescender, typoAscender - typoDescender + typoGap);
                }
                else
                {
                    int winAscent = os2.U16(74), winDescent = os2.U16(76);
                    (ascender, descender, height) = (winAscent, -winDescent, winAscent + winDescent);
                }
            }
        }
        else
        {
            Weight = (macStyle & 1) != 0 ? 700 : 400;
        }

        (Ascender, Descender, LineSpacing) = (ascender, descender, height);

        bool bold, italic;
        if (fsSelection >= 0)
        {
            bold = (fsSelection & 0x20) != 0;
            italic = (fsSelection & 0x201) != 0; // italic, or oblique
        }
        else
        {
            bold = (macStyle & 1) != 0;
            italic = (macStyle & 2) != 0;
        }

        Style = (bold, italic) switch
        {
            (true, true) => FontStyle.BoldItalic,
            (true, false) => FontStyle.Bold,
            (false, true) => FontStyle.Italic,
            _ => FontStyle.Regular,
        };

        if (sfnt.TryGetTable("post", out FontData post) && post.Length >= 16)
        {
            ItalicAngle = post.Fixed(4);
            IsFixedPitch = post.U32(12) != 0;
        }

        FontNames names = FontNames.Read(sfnt.TryGetTable("name", out FontData name) ? name : null);
        FamilyName = names.Family;
        StyleName = names.Subfamily;
        FullName = names.FullName;
        PostScriptName = names.PostScriptName;
        TypographicFamilyName = names.TypographicFamily.Length > 0 ? names.TypographicFamily : names.Family;
        TypographicStyleName = names.TypographicSubfamily.Length > 0 ? names.TypographicSubfamily : names.Subfamily;

        _map = CharacterMap.Read(sfnt.Table("cmap"));
        _kerning = PairKerning.Read(sfnt.TryGetTable("GPOS", out FontData gpos) ? gpos : null,
                                sfnt.TryGetTable("kern", out FontData kern) ? kern : null);

        if (sfnt.TryGetTable("glyf", out FontData glyf))
        {
            if (locaFormat is not (0 or 1))
            {
                throw FontData.Malformed($"its head gives the loca format {locaFormat}");
            }

            _trueType = new TrueTypeGlyphs(glyf, sfnt.Table("loca"), locaFormat == 1, GlyphCount, limits, LeftSideBearing);
            HasCubicOutlines = false;
        }
        else if (sfnt.TryGetTable("CFF ", out FontData cff))
        {
            _cff = CffGlyphs.Read(cff, UnitsPerEm, limits);
            if (_cff.GlyphCount != GlyphCount)
            {
                throw FontData.Malformed($"its CFF table has {_cff.GlyphCount} glyphs where maxp has {GlyphCount}");
            }

            HasCubicOutlines = true;
        }
        else if (sfnt.TryGetTable("CFF2", out _))
        {
            throw new FontUnsupportedException("it has CFF2 outlines (a variable font), which aren't read");
        }
        else
        {
            throw new FontUnsupportedException("it has no outlines (glyf or CFF): a bitmap font isn't read");
        }
    }

    /// <summary>The font's family (its name table's ID 1, as Windows groups styles): <c>"Liberation Sans"</c>.</summary>
    public string FamilyName { get; }

    /// <summary>Its style within <see cref="FamilyName"/> (ID 2): <c>"Regular"</c>, <c>"Bold Italic"</c>.</summary>
    public string StyleName { get; }

    /// <summary>The typographic family (ID 16), which groups more weights than <see cref="FamilyName"/>; else the family.</summary>
    public string TypographicFamilyName { get; }

    /// <summary>The typographic style (ID 17), else <see cref="StyleName"/>.</summary>
    public string TypographicStyleName { get; }

    /// <summary>The full name (ID 4): <c>"Liberation Sans Bold"</c>.</summary>
    public string FullName { get; }

    /// <summary>The PostScript name (ID 6): <c>"LiberationSans-Bold"</c>.</summary>
    public string PostScriptName { get; }

    /// <summary>Bold and italic as the font declares them (OS/2's selection flags, else head's style bits).</summary>
    public FontStyle Style { get; }

    /// <summary>The weight, 100 (thin) to 900 (black); 400 is normal, 700 bold.</summary>
    public int Weight { get; }

    /// <summary>The italic angle in degrees, counter-clockwise from vertical (negative leans right).</summary>
    public double ItalicAngle { get; }

    /// <summary>Whether every glyph has the same advance.</summary>
    public bool IsFixedPitch { get; }

    /// <summary>The font's units in an em: coordinates and metrics are in these.</summary>
    public int UnitsPerEm { get; }

    /// <summary>The height above the baseline a line reserves, in font units (FreeType's <c>ascender</c>).</summary>
    public int Ascender { get; }

    /// <summary>The depth below the baseline a line reserves, negative, in font units (FreeType's <c>descender</c>).</summary>
    public int Descender { get; }

    /// <summary>The distance from one baseline to the next (FreeType's <c>height</c>): ascender less descender, plus the gap.</summary>
    public int LineSpacing { get; }

    /// <summary>The number of glyphs.</summary>
    public int GlyphCount { get; }

    /// <summary>The font's index in its collection; 0 for a single font.</summary>
    public int FaceIndex { get; }

    /// <summary>Whether its outlines are CFF's cubic curves (an <c>.otf</c>), not TrueType's quadratics.</summary>
    public bool HasCubicOutlines { get; }

    /// <summary>Whether the font has pair kerning (GPOS's, or its <c>kern</c> table's).</summary>
    public bool HasKerning => _kerning.HasPairs;

    /// <summary>The bytes of the whole file, for the library to tell a font from its file.</summary>
    internal ReadOnlyMemory<byte> File => _file.Memory;

    /// <summary>
    /// The font in <paramref name="stream"/>: a TrueType or OpenType file, or the font at <paramref name="faceIndex"/> of a
    /// collection, read within <paramref name="limits"/>.
    /// </summary>
    /// <returns>The font, or a failure: <see cref="Malformed"/>, <see cref="LimitExceeded"/>, <see cref="Unsupported"/>, or
    /// <c>input.invalid</c> for a face past the collection's.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="stream"/> is null.</exception>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="faceIndex"/> is negative.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public static Result<Font> Read(Stream stream, int faceIndex = 0, FontReadLimits? limits = null, CancellationToken cancellationToken = default)
    {
        KernelArgumentNullException.ThrowIfNull(stream);
        KernelArgumentOutOfRangeException.ThrowIfNegative(faceIndex);
        limits ??= FontReadLimits.Default;
        cancellationToken = Cancellation.Effective(cancellationToken);
        using Cancellation.Scope cancellation = Cancellation.Enter(cancellationToken);
        Result<ReadOnlyMemory<byte>> bytes = ReadAll(stream, limits.MaxBytes, cancellationToken);
        return bytes.IsSuccess ? FromBytes(bytes.Value, faceIndex, limits) : Result<Font>.Failure(bytes.Diagnostics);
    }

    /// <summary>Every font of a collection (<c>.ttc</c>) in <paramref name="stream"/>, or the one font of a single file.</summary>
    /// <returns>The fonts, or the first failure, as <see cref="Read"/> gives it.</returns>
    /// <exception cref="ArgumentNullException"><paramref name="stream"/> is null.</exception>
    /// <exception cref="OperationCanceledException">The operation was cancelled.</exception>
    public static Result<IReadOnlyList<Font>> ReadCollection(Stream stream, FontReadLimits? limits = null, CancellationToken cancellationToken = default)
    {
        KernelArgumentNullException.ThrowIfNull(stream);
        limits ??= FontReadLimits.Default;
        cancellationToken = Cancellation.Effective(cancellationToken);
        using Cancellation.Scope cancellation = Cancellation.Enter(cancellationToken);
        Result<ReadOnlyMemory<byte>> bytes = ReadAll(stream, limits.MaxBytes, cancellationToken);
        if (!bytes.IsSuccess)
        {
            return Result<IReadOnlyList<Font>>.Failure(bytes.Diagnostics);
        }

        ReadOnlyMemory<byte> file = bytes.Value;
        return Guard<IReadOnlyList<Font>>(() =>
        {
            var data = new FontData(file);
            int[] offsets = SfntFile.FaceOffsets(data, limits);
            var fonts = ImmutableArray.CreateBuilder<Font>(offsets.Length);
            for (int i = 0; i < offsets.Length; i++)
            {
                cancellationToken.ThrowIfCancellationRequested();
                fonts.Add(new Font(data, i, SfntFile.Read(data, offsets[i]), limits));
            }

            return Result<IReadOnlyList<Font>>.Success(fonts.MoveToImmutable());
        });
    }

    /// <summary>The font at <paramref name="faceIndex"/> of a file's bytes, already read.</summary>
    internal static Result<Font> FromBytes(ReadOnlyMemory<byte> file, int faceIndex, FontReadLimits limits) => Guard<Font>(() =>
    {
        var data = new FontData(file);
        int[] offsets = SfntFile.FaceOffsets(data, limits);
        if (faceIndex >= offsets.Length)
        {
            return Result<Font>.Failure(Diagnostic.Error(DiagnosticCode.InvalidInput,
                $"The file holds {offsets.Length} font{(offsets.Length == 1 ? "" : "s")}: there's no font {faceIndex}."));
        }

        return new Font(data, faceIndex, SfntFile.Read(data, offsets[faceIndex]), limits);
    });

    /// <summary>The glyph <paramref name="codePoint"/> maps to, or 0 (the missing glyph) where the font has none.</summary>
    public int GlyphOf(int codePoint)
    {
        int glyph = Guarded(() => _map.GlyphOf(codePoint), 0);
        return glyph < GlyphCount ? glyph : 0;
    }

    /// <summary>Whether the font has a glyph for <paramref name="codePoint"/>.</summary>
    public bool HasGlyph(int codePoint) => GlyphOf(codePoint) != 0;

    /// <summary>The glyph's advance width, in font units (from <c>hmtx</c>).</summary>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="glyph"/> isn't one of the font's.</exception>
    public int AdvanceWidth(int glyph)
    {
        CheckGlyph(glyph);
        int at = Math.Min(glyph, _metricCount - 1);
        return _hmtx.U16(4 * at);
    }

    // The glyph's left side bearing in hmtx, or null where hmtx is too short to give it
    private int? LeftSideBearing(int glyph)
    {
        int at = glyph < _metricCount ? 4 * glyph + 2 : 4 * _metricCount + 2 * (glyph - _metricCount);
        return at <= _hmtx.Length - 2 ? _hmtx.I16(at) : null;
    }

    /// <summary>The kerning between two glyphs, in font units: the adjustment to <paramref name="left"/>'s advance when
    /// <paramref name="right"/> follows it, usually negative; 0 where the font has none.</summary>
    /// <exception cref="ArgumentOutOfRangeException">A glyph isn't one of the font's.</exception>
    public int Kerning(int left, int right)
    {
        CheckGlyph(left);
        CheckGlyph(right);
        return Guarded(() => _kerning.Value(left, right), 0);
    }

    /// <summary>The glyph's outline, in font units: none for a blank glyph.</summary>
    /// <returns>The outline, or a failure for a glyph whose data is damaged: <see cref="Malformed"/>,
    /// <see cref="LimitExceeded"/> or <see cref="Unsupported"/>.</returns>
    /// <exception cref="ArgumentOutOfRangeException"><paramref name="glyph"/> isn't one of the font's.</exception>
    public Result<GlyphOutline> Outline(int glyph)
    {
        CheckGlyph(glyph);
        return Guard<GlyphOutline>(() => new GlyphOutline(glyph, _trueType is { } tt ? tt.Outline(glyph) : _cff!.Outline(glyph)));
    }

    /// <inheritdoc/>
    public override string ToString() => FullName.Length > 0 ? FullName : $"{FamilyName} {StyleName}".Trim();

    private void CheckGlyph(int glyph)
    {
        if ((uint)glyph >= (uint)GlyphCount)
        {
            throw new KernelArgumentOutOfRangeException(nameof(glyph), glyph, $"The font has {GlyphCount} glyphs.");
        }
    }

    // Reads checked when the font was read can still meet a damaged structure that only a lookup reaches (a cmap range
    // past its table); such a lookup finds nothing
    private static T Guarded<T>(Func<T> read, T fallback)
    {
        try
        {
            return read();
        }
        catch (FontMalformedException)
        {
            return fallback;
        }
    }

    private static Result<T> Guard<T>(Func<Result<T>> read)
    {
        try
        {
            return read();
        }
        catch (FontMalformedException e)
        {
            return Result<T>.Failure(Diagnostic.Error(Malformed, $"The font couldn't be read: {e.Message}."));
        }
        catch (FontLimitException e)
        {
            return Result<T>.Failure(Diagnostic.Error(LimitExceeded, $"The font is past the reader's limits: {e.Message}."));
        }
        catch (FontUnsupportedException e)
        {
            return Result<T>.Failure(Diagnostic.Error(Unsupported, $"The font can't be read: {e.Message}."));
        }
        catch (InsufficientExecutionStackException)
        {
            return Result<T>.Failure(Diagnostic.Error(LimitExceeded, "The font is past the reader's limits: its glyphs nest too deep."));
        }
    }

    private static Result<ReadOnlyMemory<byte>> ReadAll(Stream stream, long maxBytes, CancellationToken cancellationToken)
    {
        if (stream.CanSeek && stream.Length - stream.Position > maxBytes)
        {
            return TooLarge(maxBytes);
        }

        var memory = new MemoryStream();
        byte[] buffer = new byte[81920];
        int read;
        while ((read = stream.Read(buffer, 0, buffer.Length)) > 0)
        {
            cancellationToken.ThrowIfCancellationRequested();
            if (memory.Length + read > maxBytes)
            {
                return TooLarge(maxBytes);
            }

            memory.Write(buffer, 0, read);
        }

        return new ReadOnlyMemory<byte>(memory.GetBuffer(), 0, (int)memory.Length);
    }

    private static Result<ReadOnlyMemory<byte>> TooLarge(long maxBytes) =>
        Result<ReadOnlyMemory<byte>>.Failure(Diagnostic.Error(LimitExceeded,
            $"The font is past the reader's limits: it is larger than {nameof(FontReadLimits.MaxBytes)} ({maxBytes} bytes)."));
}
