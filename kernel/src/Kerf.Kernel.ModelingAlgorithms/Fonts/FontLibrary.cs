// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Collections.Concurrent;
using System.Collections.Immutable;
using System.IO.Compression;
using Kerf.Engine.Fonts;

namespace Kerf.Fonts;

/// <summary>A font a <see cref="FontLibrary"/> lists: its family and style, and where it is.</summary>
/// <param name="Family">The family it is found by (its name table's ID 1, or ID 16 for its typographic family).</param>
/// <param name="Style">Its style within the family.</param>
/// <param name="StyleName">The style as the font names it: <c>"Bold Italic"</c>, <c>"Semibold"</c>.</param>
/// <param name="Path">Its file, or null for a font held in memory.</param>
/// <param name="FaceIndex">Its index in a collection; 0 for a single font.</param>
public sealed record FontEntry(string Family, FontStyle Style, string StyleName, string? Path, int FaceIndex);

/// <summary>
/// Fonts found by family and style (OCCT's <c>Font_FontMgr</c>): the system's and the user's installed fonts on Windows,
/// macOS and Linux (<see cref="System"/>), the fonts of chosen folders, or fonts in memory; and the default font Kerf ships,
/// Liberation Sans (<see cref="Default"/>).
/// </summary>
/// <remarks>
/// <para>A font is listed under its family (the name table's ID 1, as Windows groups four styles; "Arial", "Arial Black",
/// "Segoe UI Semibold") with the style its flags give, and under its typographic family (ID 16; "Segoe UI") where that
/// differs and the style isn't taken there; the style's words other than Regular, Bold, Italic and Oblique join the family
/// name, as build123d names faces ("Arial Narrow"). Names match without regard to case.</para>
/// <para><see cref="Find"/> never fails for want of a font: a family it hasn't, or a font it can't read, is replaced by
/// Liberation Sans, with a <see cref="Substituted"/> warning, as OCCT warns (decision 2008: Liberation Sans has Arial's
/// advance widths, so build123d's default, Arial, comes out the same size).</para>
/// <para>Immutable once made, and safe to share between threads; it reads a font's file the first time it is found and
/// keeps it. <see cref="System"/> is listed once, the first time it is used: fonts installed later aren't seen until
/// <see cref="Scan"/> lists them again.</para>
/// </remarks>
public sealed class FontLibrary
{
    private static readonly Lazy<FontLibrary> SystemLibrary = new(() => Scan(SystemFolders()), LazyThreadSafetyMode.ExecutionAndPublication);
    private static readonly Lazy<Font>[] DefaultFonts =
    [
        new(() => LoadDefault("Regular")),
        new(() => LoadDefault("Bold")),
        new(() => LoadDefault("Italic")),
        new(() => LoadDefault("BoldItalic")),
    ];

    private readonly Dictionary<(string Family, FontStyle Style), FontEntry> _byName;
    private readonly Dictionary<FontEntry, Font> _inMemory;
    private readonly ConcurrentDictionary<(string Path, int Face), Lazy<Result<Font>>> _loaded = new();
    private readonly FontReadLimits _limits;

    private FontLibrary(List<(FontEntry Entry, string TypographicFamily, string TypographicStyle, Font? Font)> fonts, FontReadLimits limits)
    {
        _limits = limits;
        _byName = new Dictionary<(string, FontStyle), FontEntry>(new NameComparer());
        _inMemory = [];
        var entries = ImmutableArray.CreateBuilder<FontEntry>();
        foreach (var (entry, _, _, font) in fonts)
        {
            if (_byName.TryAdd((entry.Family, entry.Style), entry))
            {
                entries.Add(entry);
            }

            if (font is not null)
            {
                _inMemory[entry] = font;
            }
        }

        // The typographic families second, so that they never take a place a family of the first kind has
        foreach (var (entry, family, style, font) in fonts)
        {
            FontEntry typographic = entry with { Family = Named(family, style), StyleName = style };
            if (!string.Equals(typographic.Family, entry.Family, StringComparison.OrdinalIgnoreCase)
                && _byName.TryAdd((typographic.Family, typographic.Style), typographic))
            {
                entries.Add(typographic);
                if (font is not null)
                {
                    _inMemory[typographic] = font;
                }
            }
        }

        Fonts = entries.ToImmutable();
    }

    /// <summary>A font was replaced by another: a family the library hasn't, a style the family hasn't, or a font file it
    /// can't read: <c>font.substituted</c>. The message says which was asked for and which is used.</summary>
    public static DiagnosticCode Substituted { get; } = new("font.substituted");

    /// <summary>The fonts installed on this machine, for everyone and for this user, listed the first time it is used.</summary>
    public static FontLibrary System => SystemLibrary.Value;

    /// <summary>The fonts listed, each family and style once, in the order found.</summary>
    public IReadOnlyList<FontEntry> Fonts { get; }

    /// <summary>
    /// The folders a system keeps fonts in: on Windows its Fonts folder and the user's; on macOS the system's, the
    /// library's and the user's; elsewhere <c>/usr/share/fonts</c>, <c>/usr/local/share/fonts</c>, the user's
    /// <c>~/.fonts</c> and <c>~/.local/share/fonts</c> (or <c>$XDG_DATA_HOME/fonts</c>), and Android's <c>/system/fonts</c>.
    /// Only those that exist.
    /// </summary>
    public static IReadOnlyList<string> SystemFolders()
    {
        var folders = new List<string>();
        string home = Environment.GetFolderPath(Environment.SpecialFolder.UserProfile);
        if (OperatingSystem.IsWindows())
        {
            folders.Add(Environment.GetFolderPath(Environment.SpecialFolder.Fonts));
            folders.Add(Path.Combine(Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData), "Microsoft", "Windows", "Fonts"));
        }
        else if (OperatingSystem.IsMacOS())
        {
            folders.AddRange(["/System/Library/Fonts", "/System/Library/Fonts/Supplemental", "/Library/Fonts", Path.Combine(home, "Library", "Fonts")]);
        }
        else
        {
            string? data = Environment.GetEnvironmentVariable("XDG_DATA_HOME");
            folders.AddRange(["/usr/share/fonts", "/usr/local/share/fonts", Path.Combine(home, ".fonts"),
                string.IsNullOrEmpty(data) ? Path.Combine(home, ".local", "share", "fonts") : Path.Combine(data, "fonts"), "/system/fonts"]);
        }

        return [.. folders.Where(f => f.Length > 0 && Directory.Exists(f)).Distinct(StringComparer.Ordinal)];
    }

    /// <summary>
    /// A library of the TrueType and OpenType fonts (<c>.ttf</c>, <c>.otf</c>, <c>.ttc</c>, <c>.otc</c>) in
    /// <paramref name="folders"/> and their subfolders (eight deep, not following links), in the order given. A file that
    /// isn't a font is left out; nothing is read but each file's names until a font is found.
    /// </summary>
    /// <exception cref="ArgumentNullException"><paramref name="folders"/> is null.</exception>
    public static FontLibrary Scan(IEnumerable<string> folders, FontReadLimits? limits = null)
    {
        ArgumentNullException.ThrowIfNull(folders);
        limits ??= FontReadLimits.Default;
        var options = new EnumerationOptions
        {
            RecurseSubdirectories = true,
            MaxRecursionDepth = 8,
            IgnoreInaccessible = true,
            AttributesToSkip = FileAttributes.ReparsePoint,
            MatchCasing = MatchCasing.CaseInsensitive,
        };
        var fonts = new List<(FontEntry, string, string, Font?)>();
        foreach (string folder in folders)
        {
            if (!Directory.Exists(folder))
            {
                continue;
            }

            foreach (string path in Directory.EnumerateFiles(folder, "*", options).Order(StringComparer.OrdinalIgnoreCase))
            {
                string extension = Path.GetExtension(path).ToLowerInvariant();
                if (extension is not (".ttf" or ".otf" or ".ttc" or ".otc"))
                {
                    continue;
                }

                try
                {
                    using FileStream file = File.OpenRead(path);
                    if (file.Length > limits.MaxBytes)
                    {
                        continue;
                    }

                    foreach (FontHeaders.Face face in FontHeaders.Read(file, limits))
                    {
                        fonts.Add((new FontEntry(Named(face.Family, face.StyleName), face.Style, face.StyleName, path, face.Index),
                                   face.TypographicFamily, face.TypographicStyleName, null));
                    }
                }
                catch (Exception e) when (e is IOException or UnauthorizedAccessException or FontMalformedException or FontLimitException)
                {
                    // Not a font this reads: left out, as OCCT's font manager leaves out what FreeType can't open
                }
            }
        }

        return new FontLibrary(fonts, limits);
    }

    /// <summary>A library of fonts already read, such as the fonts of one file.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="fonts"/> or one of them is null.</exception>
    public static FontLibrary FromFonts(IEnumerable<Font> fonts)
    {
        ArgumentNullException.ThrowIfNull(fonts);
        var list = new List<(FontEntry, string, string, Font?)>();
        foreach (Font font in fonts)
        {
            ArgumentNullException.ThrowIfNull(font, nameof(fonts));
            if (font.FamilyName.Length == 0)
            {
                continue;
            }

            list.Add((new FontEntry(Named(font.FamilyName, font.StyleName), font.Style, font.StyleName, null, font.FaceIndex),
                      font.TypographicFamilyName, font.TypographicStyleName, font));
        }

        return new FontLibrary(list, FontReadLimits.Default);
    }

    /// <summary>
    /// The default font, which Kerf ships: Liberation Sans 2.1.5 (SIL Open Font Licence 1.1; the packages' notices), in
    /// <paramref name="style"/>. It has Arial's advance widths, so it sets text as Arial would.
    /// </summary>
    public static Font Default(FontStyle style = FontStyle.Regular) => DefaultFonts[(int)style].Value;

    /// <summary>Whether the library has <paramref name="family"/> in any style.</summary>
    /// <exception cref="ArgumentNullException"><paramref name="family"/> is null.</exception>
    public bool HasFamily(string family)
    {
        ArgumentNullException.ThrowIfNull(family);
        return Enum.GetValues<FontStyle>().Any(s => _byName.ContainsKey((family, s)));
    }

    /// <summary>
    /// The font of <paramref name="family"/> in <paramref name="style"/>, read the first time it is asked for. Where the
    /// family hasn't the style, its nearest (OCCT's <c>Font_SystemFont::FontPathAny</c>: bold italic from bold, else regular;
    /// italic and bold from regular), and where the library hasn't the family, or can't read its font, Liberation Sans:
    /// either with a <see cref="Substituted"/> warning. A caller that asked for an italic and got an upright font may slant
    /// it, as OCCT synthesises an italic (<see cref="TextOptions.Slant"/>).
    /// </summary>
    /// <exception cref="ArgumentNullException"><paramref name="family"/> is null.</exception>
    public Result<Font> Find(string family, FontStyle style = FontStyle.Regular)
    {
        ArgumentNullException.ThrowIfNull(family);
        FontStyle[] order = style switch
        {
            FontStyle.BoldItalic => [FontStyle.BoldItalic, FontStyle.Bold, FontStyle.Regular, FontStyle.Italic],
            FontStyle.Bold => [FontStyle.Bold, FontStyle.Regular],
            FontStyle.Italic => [FontStyle.Italic, FontStyle.Regular],
            _ => [FontStyle.Regular, FontStyle.Bold, FontStyle.Italic, FontStyle.BoldItalic],
        };

        string? failure = null;
        foreach (FontStyle candidate in order)
        {
            if (!_byName.TryGetValue((family, candidate), out FontEntry? entry))
            {
                continue;
            }

            Result<Font> font = Load(entry);
            if (!font.IsSuccess)
            {
                failure = font.Error!.Message;
                break;
            }

            return candidate == style
                ? font
                : Result<Font>.Success(font.Value, [Warning($"The font '{family}' has no {Describe(style)} style: its {Describe(candidate)} style is used.")]);
        }

        Font fallback = Default(style);
        if (failure is null && string.Equals(family, DefaultFamily, StringComparison.OrdinalIgnoreCase))
        {
            return fallback; // what was asked for, from the packages
        }

        string reason = failure is not null
            ? $"The font '{family}' ({Describe(style)}) couldn't be read ({failure}): Liberation Sans is used instead."
            : $"The font '{family}' ({Describe(style)}) isn't installed: Liberation Sans, which has Arial's widths, is used instead.";
        return Result<Font>.Success(fallback, [Warning(reason)]);
    }

    /// <summary>The default font's family, "Liberation Sans".</summary>
    public const string DefaultFamily = "Liberation Sans";

    private Result<Font> Load(FontEntry entry)
    {
        if (_inMemory.TryGetValue(entry, out Font? font))
        {
            return font;
        }

        return _loaded.GetOrAdd((entry.Path!, entry.FaceIndex), key => new Lazy<Result<Font>>(() =>
        {
            try
            {
                using FileStream file = File.OpenRead(key.Path);
                return Font.Read(file, key.Face, _limits);
            }
            catch (Exception e) when (e is IOException or UnauthorizedAccessException)
            {
                return Result<Font>.Failure(Diagnostic.Error(Font.Malformed, $"The font file '{key.Path}' couldn't be opened: {e.Message}"));
            }
        }, LazyThreadSafetyMode.ExecutionAndPublication)).Value;
    }

    private static Diagnostic Warning(string message) => Diagnostic.Warning(Substituted, message);

    private static string Describe(FontStyle style) => style switch
    {
        FontStyle.Bold => "bold",
        FontStyle.Italic => "italic",
        FontStyle.BoldItalic => "bold italic",
        _ => "regular",
    };

    // build123d's naming of a face: the family, with the style's words other than Regular, Bold, Italic and Oblique
    private static string Named(string family, string styleName)
    {
        string[] rest = [.. styleName.Split(' ', StringSplitOptions.RemoveEmptyEntries)
            .Where(w => !w.Equals("Regular", StringComparison.OrdinalIgnoreCase) && !w.Equals("Bold", StringComparison.OrdinalIgnoreCase)
                        && !w.Equals("Italic", StringComparison.OrdinalIgnoreCase) && !w.Equals("Oblique", StringComparison.OrdinalIgnoreCase))];
        string name = rest.Length == 0 ? family : family + " " + string.Join(' ', rest);
        return name.Trim();
    }

    private static Font LoadDefault(string style)
    {
        using Stream compressed = typeof(FontLibrary).Assembly.GetManifestResourceStream($"Kerf.Fonts.LiberationSans-{style}.ttf.gz")
            ?? throw new InvalidOperationException($"The default font's {style} style isn't in the assembly.");
        using var font = new GZipStream(compressed, CompressionMode.Decompress);
        return Font.Read(font).Value;
    }

    private sealed class NameComparer : IEqualityComparer<(string Family, FontStyle Style)>
    {
        public bool Equals((string Family, FontStyle Style) x, (string Family, FontStyle Style) y) =>
            x.Style == y.Style && string.Equals(x.Family, y.Family, StringComparison.OrdinalIgnoreCase);

        public int GetHashCode((string Family, FontStyle Style) obj) =>
            HashCode.Combine(StringComparer.OrdinalIgnoreCase.GetHashCode(obj.Family), obj.Style);
    }
}
