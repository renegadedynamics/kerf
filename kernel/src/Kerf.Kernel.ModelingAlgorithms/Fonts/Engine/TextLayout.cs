// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).
// Port of: src/Visualization/TKService/Font/Font_TextFormatter.cxx (Append, Format, newLine), redesigned (unit 13D)

using Kerf.Fonts;

namespace Kerf.Engine.Fonts;

/// <summary>
/// Where each character of a text goes (OCCT's <c>Font_TextFormatter</c>, without wrapping): along a baseline by each
/// glyph's advance and the kerning with the next, a new line down a line's spacing at each line feed, each line aligned
/// left, centred or right on the advances of its characters, and the whole block aligned vertically.
/// </summary>
/// <remarks>
/// In doubles, where OCCT works in single precision (its positions are pixels of a 4,800-pixel em, so it rounds at about
/// 1e-7 of an em a character). A tab advances to the next multiple of <see cref="TextOptions.TabSize"/> characters' worth
/// of spaces: OCCT counts the characters before it one short, so its first tab of a line is a space wider (kernel
/// decision 2021).
/// </remarks>
internal static class TextLayout
{
    /// <summary>A character to draw, and its pen position: the left end of its baseline.</summary>
    public readonly record struct Placed(int CodePoint, double X, double Y);

    // OCCT's Font_TextFormatter::IsCommandSymbol: carriage controls it skips
    private static bool IsCommand(int c) => c is '\r' or '\a' or '\f' or '\b' or '\v';

    /// <summary>
    /// The visible characters of <paramref name="codePoints"/> placed, in model units; <paramref name="advance"/> gives a
    /// character's advance with the kerning to the next (0 after the last).
    /// </summary>
    public static List<Placed> Lay(IReadOnlyList<int> codePoints, Func<int, int, double> advance, double ascender, double lineSpacing,
        TextAlign horizontal, TextAlign vertical, int tabSize)
    {
        // Append: every symbol on one baseline, and where the lines break
        var symbols = new List<int>();
        var corners = new List<double>();
        var lineEnds = new List<double>();
        double pen = 0;
        int column = 0;
        for (int i = 0; i < codePoints.Count; i++)
        {
            int c = codePoints[i];
            if (IsCommand(c))
            {
                continue;
            }

            int next = i + 1 < codePoints.Count ? codePoints[i + 1] : 0;
            double width;
            if (c == '\n')
            {
                lineEnds.Add(pen);
                column = 0;
                width = 0;
            }
            else if (c == '\t')
            {
                int spaces = tabSize - column % tabSize;
                width = advance(' ', next) * spaces;
                column += spaces;
            }
            else
            {
                width = advance(c, next);
                column++;
            }

            symbols.Add(c);
            corners.Add(pen);
            pen += width;
        }

        var placed = new List<Placed>();
        if (symbols.Count == 0)
        {
            return placed;
        }

        // Format: each line aligned, then the block
        double lineTop = -ascender;
        var ys = new double[symbols.Count];
        int lineStart = 0, lines = 0;
        void NewLine(int last)
        {
            if (lineStart < symbols.Count)
            {
                double xMin = corners[lineStart];
                double xMax = last + 1 < symbols.Count ? corners[last + 1] : pen;
                double move = horizontal switch
                {
                    TextAlign.Right => -xMax,
                    TextAlign.Center => -0.5 * (xMin + xMax),
                    _ => -xMin,
                };
                for (int k = lineStart; k <= last; k++)
                {
                    corners[k] += move;
                    ys[k] = lineTop;
                }
            }

            lines++;
            lineTop -= lineSpacing;
            lineStart = last + 1;
        }

        for (int k = 0; k < symbols.Count; k++)
        {
            if (symbols[k] == '\n')
            {
                NewLine(k);
            }
        }

        NewLine(symbols.Count - 1);

        double top = vertical switch
        {
            TextAlign.Bottom => -lineSpacing - lineTop,
            TextAlign.Center => 0.5 * lineSpacing * lines,
            TextAlign.TopFirstLine => ascender,
            _ => 0,
        };

        for (int k = 0; k < symbols.Count; k++)
        {
            int c = symbols[k];
            if (c is not ('\n' or ' ' or '\t'))
            {
                placed.Add(new Placed(c, corners[k], ys[k] + top));
            }
        }

        return placed;
    }
}
