// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Foundation;
using Kerf.Geometry;
using System.Reflection;
using System.Text;
using System.Text.RegularExpressions;
using Xunit.Abstractions;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// GeomTools against the <c>.brep</c> files of OCCT's test data (data/occ): their "Curve2ds", "Curves" and "Surfaces"
/// sections are GeomTools output. Each section is read, written back and compared with the file's text.
/// </summary>
/// <remarks>
/// The files were written by older OCCT releases: some at 15 significant digits (OCCT's sets now write 17), by C
/// runtimes that wrote three exponent digits ("1e-007"), in text mode (CRLF line ends). The comparison writes at the
/// file's precision and normalises those two things in the file's text; everything else must match byte for byte.
/// </remarks>
public partial class GeomTools_CorpusTests(ITestOutputHelper output)
{
    internal static string OcctSourceDirectory
        => typeof(GeomTools_CorpusTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == "OcctSourceDir").Value!;

    internal static string DataDirectory => Path.Combine(OcctSourceDirectory, "data", "occ");

    internal static IEnumerable<string> BrepFiles()
    {
        Assert.True(Directory.Exists(DataDirectory), $"No OCCT source at {OcctSourceDirectory}: run tools/get-occt.ps1");
        return Directory.EnumerateFiles(DataDirectory, "*.brep").Order(StringComparer.Ordinal);
    }

    /// <summary>The text of a section, from its header line to the next one (exclusive), with LF line ends.</summary>
    internal static string? Section(string theText, string theHeader, string theNext)
    {
        string aText = theText.Replace("\r\n", "\n", StringComparison.Ordinal);
        int aStart = aText.IndexOf("\n" + theHeader + " ", StringComparison.Ordinal);
        if (aStart < 0)
        {
            return null;
        }

        aStart++;
        int anEnd = aText.IndexOf("\n" + theNext, aStart, StringComparison.Ordinal);
        return aText.Substring(aStart, anEnd + 1 - aStart);
    }

    [GeneratedRegex(@"e([-+])0(\d\d)\b")]
    private static partial Regex ThreeDigitExponent();

    // the old C runtime's three exponent digits, as today's two
    private static string Normalised(string theSection) => ThreeDigitExponent().Replace(theSection, "e$1$2");

    // the precision a file was written at: 17 if any number has more than 15 significant digits
    private static int PrecisionOf(string theSection)
    {
        foreach (Match aMatch in NumberRegex().Matches(theSection))
        {
            string aDigits = aMatch.Groups[1].Value.Replace(".", "", StringComparison.Ordinal).TrimStart('0');
            if (aDigits.Length > 15)
            {
                return 17;
            }
        }

        return 15;
    }

    [GeneratedRegex(@"(?<![\w.])-?([0-9]*\.?[0-9]+)(?:e[-+][0-9]+)?(?![\w.])")]
    private static partial Regex NumberRegex();

    private sealed record SectionKind(string Header, string Next, Func<string, int, string> RoundTrip);

    private static readonly SectionKind[] theKinds =
    [
        new("Curve2ds", "Curves ", (theSection, thePrecision) =>
        {
            var aSet = new Curve2dSet();
            aSet.Read(new StringReader(theSection));
            var aWriter = new StringWriter();
            StreamUtilities.Precision(aWriter, thePrecision);
            int aCount = Count(theSection);
            aWriter.Write($"Curve2ds {aCount}\n");
            for (int i = 1; i <= aCount; i++)
            {
                Curve2D? aCurve = aSet.Curve2d(i);
                Assert.NotNull(aCurve);
                Curve2dSet.PrintCurve2d(aCurve, aWriter, true);
            }

            return aWriter.ToString();
        }),
        new("Curves", "Polygon3D ", (theSection, thePrecision) =>
        {
            var aSet = new CurveSet();
            aSet.Read(new StringReader(theSection));
            var aWriter = new StringWriter();
            StreamUtilities.Precision(aWriter, thePrecision);
            int aCount = Count(theSection);
            aWriter.Write($"Curves {aCount}\n");
            for (int i = 1; i <= aCount; i++)
            {
                Curve3D? aCurve = aSet.Curve(i);
                Assert.NotNull(aCurve);
                CurveSet.PrintCurve(aCurve, aWriter, true);
            }

            return aWriter.ToString();
        }),
        new("Surfaces", "Triangulations ", (theSection, thePrecision) =>
        {
            var aSet = new SurfaceSet();
            aSet.Read(new StringReader(theSection));
            var aWriter = new StringWriter();
            StreamUtilities.Precision(aWriter, thePrecision);
            int aCount = Count(theSection);
            aWriter.Write($"Surfaces {aCount}\n");
            for (int i = 1; i <= aCount; i++)
            {
                Surface? aSurface = aSet.Surface(i);
                Assert.NotNull(aSurface);
                SurfaceSet.PrintSurface(aSurface, aWriter, true);
            }

            return aWriter.ToString();
        }),
    ];

    private static int Count(string theSection) => int.Parse(theSection[(theSection.IndexOf(' ', StringComparison.Ordinal) + 1)..theSection.IndexOf('\n', StringComparison.Ordinal)],
                                                              System.Globalization.CultureInfo.InvariantCulture);

    /// <summary>
    /// Each section read and written back at the file's precision gives the file's text, word for word; numbers may
    /// differ by two units in the last digit written (or by 1e-15 for the components of directions), because reading normalises directions (and orthogonalises axes)
    /// that an older OCCT wrote, and "-0" is "0" (the older C runtimes wrote negative zero as "0"). The oracle tests
    /// compare with native OCCT 8's output byte for byte.
    /// </summary>
    [Fact]
    public void Corpus_sections_are_written_back_word_for_word()
    {
        int aFiles = 0, aSections = 0, anObjects = 0, aWords = 0, aSameWords = 0, aCloseNumbers = 0;
        var aFailures = new StringBuilder();
        foreach (string aFile in BrepFiles())
        {
            string aText = File.ReadAllText(aFile);
            aFiles++;
            foreach (SectionKind aKind in theKinds)
            {
                string? aSection = Section(aText, aKind.Header, aKind.Next);
                Assert.NotNull(aSection);
                string anExpected = Normalised(aSection);
                int aPrecision = PrecisionOf(anExpected);
                string anActual = aKind.RoundTrip(anExpected, aPrecision);
                aSections++;
                anObjects += Count(anExpected);

                // the same lines, and the same words but for close numbers
                string[] anExpectedLines = anExpected.Split('\n');
                string[] anActualLines = anActual.Split('\n');
                if (anExpectedLines.Length != anActualLines.Length)
                {
                    aFailures.AppendLine($"{Path.GetFileName(aFile)} {aKind.Header}: {anActualLines.Length} lines, file {anExpectedLines.Length}");
                    continue;
                }

                for (int aLine = 0; aLine < anExpectedLines.Length; aLine++)
                {
                    string[] anExpectedWords = anExpectedLines[aLine].Split(' ');
                    string[] anActualWords = anActualLines[aLine].Split(' ');
                    bool isSame = anExpectedWords.Length == anActualWords.Length;
                    for (int w = 0; isSame && w < anExpectedWords.Length; w++)
                    {
                        aWords++;
                        if (anExpectedWords[w] == anActualWords[w] || (IsZero(anExpectedWords[w]) && IsZero(anActualWords[w])))
                        {
                            aSameWords++;
                        }
                        else if (AreClose(anExpectedWords[w], anActualWords[w], aPrecision))
                        {
                            aCloseNumbers++;
                        }
                        else
                        {
                            isSame = false;
                        }
                    }

                    if (!isSame)
                    {
                        aFailures.AppendLine($"{Path.GetFileName(aFile)} {aKind.Header} (precision {aPrecision}), line {aLine + 1}:");
                        aFailures.AppendLine($"  file: {anExpectedLines[aLine]}");
                        aFailures.AppendLine($"  port: {anActualLines[aLine]}");
                    }
                }
            }
        }

        output.WriteLine($"{aFiles} files, {aSections} sections, {anObjects} curves and surfaces, {aWords} words: " +
                         $"{aSameWords} identical, {aCloseNumbers} numbers different in the last two digits");
        Assert.True(aFailures.Length == 0, aFailures.ToString());
    }

    private static bool IsZero(string theWord) => theWord is "0" or "-0";

    // two numbers written at the precision, at most 2 units apart in the last significant digit written
    private static bool AreClose(string theA, string theB, int thePrecision)
    {
        if (!double.TryParse(theA, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out double a)
            || !double.TryParse(theB, System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out double b))
        {
            return false;
        }

        double aMax = Math.Max(Math.Abs(a), Math.Abs(b));
        double anUnit = Math.Pow(10.0, Math.Floor(Math.Log10(aMax)) - thePrecision + 1);
        // or, for the components of unit vectors near zero, 1e-15 apart
        return Math.Abs(a - b) <= Math.Max(2.0 * anUnit * (1.0 + 1e-9), 1e-15 * Math.Max(1.0, aMax));
    }

    private static string Excerpt(string theText, int theStart)
    {
        if (theStart >= theText.Length)
        {
            return "<end>";
        }

        int anEnd = theText.IndexOf('\n', theStart);
        return theText[theStart..(anEnd < 0 ? theText.Length : anEnd)];
    }
}
