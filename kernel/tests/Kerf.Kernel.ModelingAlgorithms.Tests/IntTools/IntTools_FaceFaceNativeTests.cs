// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Intersection.Tools;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Collections.Concurrent;
using System.Globalization;
using System.IO.Compression;
using System.Reflection;
using System.Text;
using Xunit.Abstractions;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// IntTools_FaceFace against native OCCT's, recorded once by tools/ffrec (decision 761) on face pairs of primitive
/// solids and free-form sheets at pseudo-random placements and of OCCT's data/occ shapes, with the parameters the
/// Boolean operations and other callers use.
/// </summary>
/// <remarks>
/// For each pair the recording has the outcome, then per line its curves' presence, its two tolerances (bits), the
/// token count and FNV-1a hash of the BRep text records of its 3D curve and of its 2D curves (written as an edge with
/// the 2D curves on two fixed planes; BRepTools writes 17 significant digits, so equal tokens are equal bits), and
/// nine points of each curve (bits); then every intersection point (bits). The port computes the same record and
/// the test compares them line by line; where a curve record differs, the largest distance between native's points
/// and the port's curve at the same parameters is reported.
/// </remarks>
public class IntTools_FaceFaceNativeTests(ITestOutputHelper output)
{
    private static string Metadata(string theKey)
        => typeof(IntTools_FaceFaceNativeTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>()
                                               .Single(a => a.Key == theKey).Value!;

    // ---------------------------------------------------------------------------------------------------------
    // The recording

    private sealed record Case(int Id, string Kind, string? FacesText, string? File1, int Index1, string? File2,
                               int Index2, int Variant, string Native);

    private static readonly Lazy<List<Case>> Cases = new(ReadCases);

    private static List<Case> ReadCases()
    {
        using Stream aRes = typeof(IntTools_FaceFaceNativeTests).Assembly
                                .GetManifestResourceStream("IntTools.FaceFaceRecordings.txt.gz")!;
        using GZipStream aZip = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aZip, Encoding.Latin1);
        string aText = aReader.ReadToEnd();
        List<Case> aCases = [];
        int aPos = 0;
        string NextLine()
        {
            int anEnd = aText.IndexOf('\n', aPos);
            string aLine = aText[aPos..anEnd];
            aPos = anEnd + 1;
            return aLine;
        }
        while (aPos < aText.Length)
        {
            string[] aHead = NextLine().Split(' ');
            Assert.Equal("case", aHead[0]);
            int anId = int.Parse(aHead[1], CultureInfo.InvariantCulture);
            string aKind = aHead[2];
            string aLine = NextLine();
            string? aFaces = null, aFile1 = null, aFile2 = null;
            int anIndex1 = 0, anIndex2 = 0;
            if (aLine.StartsWith("faces ", StringComparison.Ordinal))
            {
                int aLength = int.Parse(aLine[6..], CultureInfo.InvariantCulture);
                aFaces = aText.Substring(aPos, aLength);
                aPos += aLength + 1;
            }
            else
            {
                string[] aWords = aLine.Split(' ');
                aFile1 = aWords[1];
                anIndex1 = int.Parse(aWords[2], CultureInfo.InvariantCulture);
                aFile2 = aWords[3];
                anIndex2 = int.Parse(aWords[4], CultureInfo.InvariantCulture);
            }
            StringBuilder aNative = new();
            string aVariantLine = NextLine();
            int aVariant = int.Parse(aVariantLine.Split(' ')[1], CultureInfo.InvariantCulture);
            aNative.Append(aVariantLine).Append('\n');
            for (aLine = NextLine(); aLine != "end"; aLine = NextLine())
            {
                aNative.Append(aLine).Append('\n');
            }
            aCases.Add(new Case(anId, aKind, aFaces, aFile1, anIndex1, aFile2, anIndex2, aVariant, aNative.ToString()));
        }
        return aCases;
    }

    // The data/occ shapes, read once.
    private static readonly ConcurrentDictionary<string, IndexedSet<Shape>> OccFaces = new();

    private static IndexedSet<Shape> Faces(string theFile)
        => OccFaces.GetOrAdd(theFile, f =>
        {
            Assert.True(BrepTools.Read(out Shape aShape,
                                       Path.Combine(Metadata("OcctSourceDir"), "data", "occ", f)));
            IndexedSet<Shape> aMap = new(ShapeHasher.Instance);
            Exploration.MapShapes(aShape, ShapeType.Face, aMap);
            return aMap;
        });

    private static (Face, Face) CaseFaces(Case theCase)
    {
        if (theCase.FacesText is not null)
        {
            BrepTools.Read(out Shape aShape, new StringReader(theCase.FacesText));
            IndexedSet<Shape> aMap = new(ShapeHasher.Instance);
            Exploration.MapShapes(aShape, ShapeType.Face, aMap);
            return (ShapeCasts.Face(aMap[0]), ShapeCasts.Face(aMap[aMap.Count - 1])); // the same face twice: held once
        }
        return (ShapeCasts.Face(Faces(theCase.File1!)[theCase.Index1 - 1]),
                ShapeCasts.Face(Faces(theCase.File2!)[theCase.Index2 - 1]));
    }

    // ---------------------------------------------------------------------------------------------------------
    // The port's record, in the recorder's format

    private readonly record struct Variant(bool Approx, bool Approx1, bool Approx2, double TolApprox, double Fuzz,
                                           bool Reverse, int Prepare);

    private static readonly Variant[] Variants =
    [
        new(true, true, true, 1.0e-7, 1.0e-7, false, 1),
        new(false, true, true, 1.0e-7, 1.0e-7, false, 2),
        new(true, false, false, 1.0e-7, 1.0e-5, false, 0),
        new(true, true, false, 1.0e-6, 1.0e-7, true, 1),
    ];

    private static string Hex(double theValue) => " " + BitConverter.DoubleToInt64Bits(theValue).ToString("x16", CultureInfo.InvariantCulture);

    private static string PortRecord(Case theCase, bool theToRunParallel = false)
    {
        (Face aF1, Face aF2) = CaseFaces(theCase);
        Variant aV = Variants[theCase.Variant];
        StringBuilder aRec = new();
        aRec.Append(CultureInfo.InvariantCulture,
                    $"variant {theCase.Variant} {(aV.Approx ? 1 : 0)} {(aV.Approx1 ? 1 : 0)} {(aV.Approx2 ? 1 : 0)}");
        aRec.Append(Hex(aV.TolApprox)).Append(Hex(aV.Fuzz))
            .Append(CultureInfo.InvariantCulture, $" {(aV.Reverse ? 1 : 0)} {aV.Prepare}\n");
        FaceFace aFF = new();
        try
        {
            aFF.SetParameters(aV.Approx, aV.Approx1, aV.Approx2, aV.TolApprox);
            aFF.SetFuzzyValue(aV.Fuzz);
            if (aV.Reverse)
            {
                aFF.Perform(aF2, aF1, theToRunParallel);
            }
            else
            {
                aFF.Perform(aF1, aF2, theToRunParallel);
            }
            if (aFF.IsDone && aV.Prepare != 0)
            {
                aFF.PrepareLines3D(aV.Prepare == 2);
            }
        }
        catch (global::Kerf.Engine.Foundation.FailureException)
        {
            return aRec.Append("out throws\n").ToString();
        }
        if (!aFF.IsDone)
        {
            return aRec.Append(CultureInfo.InvariantCulture, $"out notdone {(aFF.TangentFaces() ? 1 : 0)}\n").ToString();
        }
        NCollection_Sequence<IntersectionCurve> aLines = aFF.Lines();
        NCollection_Sequence<PntOn2Faces> aPoints = aFF.Points();
        aRec.Append(CultureInfo.InvariantCulture,
                    $"out done {(aFF.TangentFaces() ? 1 : 0)} {aLines.Length()} {aPoints.Length()}\n");
        PlaneSurface aP1 = new(new Plane3D());
        PlaneSurface aP2 = new(new Plane3D(new Point3D(0, 0, 1), new Direction3D(0, 0, 1)));
        BrepBuilder aBB = new();
        for (int i = 1; i <= aLines.Length(); i++)
        {
            IntersectionCurve aIC = aLines.Value(i);
            aRec.Append(CultureInfo.InvariantCulture,
                        $"line {i} {(aIC.Curve() is null ? 0 : 1)} {(aIC.FirstCurve2d() is null ? 0 : 1)} {(aIC.SecondCurve2d() is null ? 0 : 1)}");
            aRec.Append(Hex(aIC.Tolerance())).Append(Hex(aIC.TangentialTolerance())).Append('\n');
            Edge anE = new();
            aBB.MakeEdge(ref anE);
            if (aIC.Curve() is not null)
            {
                aBB.UpdateEdge(anE, aIC.Curve(), 1.0e-7);
            }
            if (aIC.FirstCurve2d() is not null)
            {
                aBB.UpdateEdge(anE, aIC.FirstCurve2d(), aP1, new Placement(), 1.0e-7);
            }
            if (aIC.SecondCurve2d() is not null)
            {
                aBB.UpdateEdge(anE, aIC.SecondCurve2d(), aP2, new Placement(), 1.0e-7);
            }
            StringWriter aWriter = new() { NewLine = "\n" };
            BrepTools.Write(anE, aWriter, true, true, FormatVersion.FormatVersion_VERSION_3);
            string aText = aWriter.ToString();
            aRec.Append(Record("c3", SectionTokens(aText, "Curves")));
            aRec.Append(Record("c2", SectionTokens(aText, "Curve2ds")));
            if (aIC.Curve() is Curve3D aC3)
            {
                aRec.Append(Samples("s3", aC3.FirstParameter, aC3.LastParameter,
                                    t => { Point3D p = aC3.Value(t); return [p.X, p.Y, p.Z]; }));
            }
            if (aIC.FirstCurve2d() is Curve2D aC1)
            {
                aRec.Append(Samples("s1", aC1.FirstParameter, aC1.LastParameter,
                                    t => { Point2D p = aC1.Value(t); return [p.X, p.Y]; }));
            }
            if (aIC.SecondCurve2d() is Curve2D aC2)
            {
                aRec.Append(Samples("s2", aC2.FirstParameter, aC2.LastParameter,
                                    t => { Point2D p = aC2.Value(t); return [p.X, p.Y]; }));
            }
        }
        for (int i = 1; i <= aPoints.Length(); i++)
        {
            PntOn2Faces aP = aPoints.Value(i);
            aP.P1().Parameters(out double u1, out double v1);
            aP.P2().Parameters(out double u2, out double v2);
            aRec.Append(CultureInfo.InvariantCulture, $"pnt {i}");
            aRec.Append(Hex(aP.P1().Pnt().X)).Append(Hex(aP.P1().Pnt().Y)).Append(Hex(aP.P1().Pnt().Z));
            aRec.Append(Hex(u1)).Append(Hex(v1));
            aRec.Append(Hex(aP.P2().Pnt().X)).Append(Hex(aP.P2().Pnt().Y)).Append(Hex(aP.P2().Pnt().Z));
            aRec.Append(Hex(u2)).Append(Hex(v2)).Append('\n');
        }
        return aRec.ToString();
    }

    // As the recorder: the tokens of a section of a BRep text, up to the next section.
    private static List<string> SectionTokens(string theText, string theName)
    {
        List<string> aTokens = [];
        bool anIn = false;
        foreach (string aRaw in theText.Split('\n'))
        {
            string aLine = aRaw.TrimEnd('\r');
            if (!anIn)
            {
                anIn = aLine.StartsWith(theName + " ", StringComparison.Ordinal);
                continue;
            }
            if (aLine.Length > 0 && char.IsAsciiLetter(aLine[0]))
            {
                break;
            }
            aTokens.AddRange(aLine.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries));
        }
        return aTokens;
    }

    private static string Record(string theTag, List<string> theTokens)
    {
        ulong aHash = 14695981039346656037ul;
        byte[] aBytes = Encoding.ASCII.GetBytes(string.Join(' ', theTokens));
        foreach (byte aB in aBytes)
        {
            aHash ^= aB;
            aHash *= 1099511628211ul;
        }
        return string.Create(CultureInfo.InvariantCulture, $"{theTag} {theTokens.Count} {aHash:x16}\n");
    }

    private static string Samples(string theTag, double theF, double theL, Func<double, double[]> theValue)
    {
        double aF = global::Kerf.Engine.Foundation.Precision.IsInfinite(theF) ? -10.0 : theF;
        double aL = global::Kerf.Engine.Foundation.Precision.IsInfinite(theL) ? 10.0 : theL;
        StringBuilder aS = new(theTag);
        for (int k = 0; k <= 8; k++)
        {
            double t = aF + (aL - aF) * k / 8.0;
            aS.Append(Hex(t));
            foreach (double v in theValue(t))
            {
                aS.Append(Hex(v));
            }
        }
        return aS.Append('\n').ToString();
    }

    // ---------------------------------------------------------------------------------------------------------
    // Comparison

    private sealed class Tally
    {
        public int Cases, SameOutcome, Lines, IdenticalLines, Records, IdenticalRecords, Values, Points, IdenticalPoints;
        public double MaxDeviation;
        public string MaxDeviationWhat = "";
        public readonly List<string> Differences = [];
    }

    private static double[] Bits(string[] theWords, int theFrom)
        => [.. theWords.Skip(theFrom).Select(w => BitConverter.Int64BitsToDouble(long.Parse(w, NumberStyles.HexNumber, CultureInfo.InvariantCulture)))];

    // The largest distance between native's samples and the port's (same parameters: equal ranges).
    private static double SampleDeviation(string theNative, string thePort, int theDim)
    {
        double[] aN = Bits(theNative.Split(' '), 1), aP = Bits(thePort.Split(' '), 1);
        double aMax = 0.0;
        for (int k = 0; k + theDim < aN.Length && k + theDim < aP.Length; k += theDim + 1)
        {
            double aD2 = 0.0;
            for (int c = 1; c <= theDim; c++)
            {
                aD2 += (aN[k + c] - aP[k + c]) * (aN[k + c] - aP[k + c]);
            }
            aMax = Math.Max(aMax, Math.Sqrt(aD2));
        }
        return aMax;
    }

    private static void Compare(Case theCase, string thePort, Tally theTally)
    {
        string aWhat = $"case {theCase.Id} {theCase.Kind}";
        string[] aN = theCase.Native.TrimEnd('\n').Split('\n');
        string[] aP = thePort.TrimEnd('\n').Split('\n');
        theTally.Cases++;
        if (aN[1] != aP[1])
        {
            theTally.Differences.Add($"{aWhat}: native '{aN[1]}', port '{aP[1]}'");
            return;
        }
        theTally.SameOutcome++;
        if (aN.Length != aP.Length)
        {
            theTally.Differences.Add($"{aWhat}: native {aN.Length} record lines, port {aP.Length}");
            return;
        }
        bool aLineIdentical = true;
        for (int i = 2; i < aN.Length; i++)
        {
            string aTag = aN[i].Split(' ')[0];
            if (aTag == "line")
            {
                if (i > 2)
                {
                    theTally.IdenticalLines += aLineIdentical ? 1 : 0;
                }
                theTally.Lines++;
                aLineIdentical = true;
            }
            if (aTag == "pnt")
            {
                theTally.Points++;
                theTally.IdenticalPoints += aN[i] == aP[i] ? 1 : 0;
            }
            if (aTag is "c3" or "c2")
            {
                theTally.Records++;
                theTally.Values += int.Parse(aN[i].Split(' ')[1], CultureInfo.InvariantCulture);
                theTally.IdenticalRecords += aN[i] == aP[i] ? 1 : 0;
            }
            if (aN[i] == aP[i])
            {
                continue;
            }
            aLineIdentical = aTag == "pnt" && aLineIdentical;
            if (aTag is "s3" or "s1" or "s2")
            {
                double aDev = SampleDeviation(aN[i], aP[i], aTag == "s3" ? 3 : 2);
                if (aDev > theTally.MaxDeviation)
                {
                    theTally.MaxDeviation = aDev;
                    theTally.MaxDeviationWhat = $"{aWhat}, {aTag}";
                }
                continue;
            }
            if (aTag is "c3" or "c2" or "pnt")
            {
                theTally.Differences.Add($"{aWhat}: {aTag} native '{aN[i]}', port '{aP[i]}'");
                continue;
            }
            // line: presence of the curves and the tolerances
            theTally.Differences.Add($"{aWhat}: native '{aN[i]}', port '{aP[i]}'");
        }
        if (aN.Skip(2).Any(l => l.StartsWith("line ", StringComparison.Ordinal)))
        {
            theTally.IdenticalLines += aLineIdentical ? 1 : 0;
        }
    }

    private Tally CompareAll(Func<Case, bool> theFilter)
    {
        Tally aTally = new();
        foreach (Case aCase in Cases.Value.Where(theFilter))
        {
            Compare(aCase, PortRecord(aCase), aTally);
        }
        output.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"{aTally.Cases} pairs, {aTally.SameOutcome} same outcome; {aTally.IdenticalLines}/{aTally.Lines} lines bit-identical; {aTally.IdenticalRecords}/{aTally.Records} curve records ({aTally.Values} values); {aTally.IdenticalPoints}/{aTally.Points} points; largest deviation {aTally.MaxDeviation:E2} {aTally.MaxDeviationWhat}"));
        foreach (string aDiff in aTally.Differences.Take(40))
        {
            output.WriteLine(aDiff);
        }
        return aTally;
    }

    [Fact]
    public void Primitive_pairs_match_native()
    {
        Tally aTally = CompareAll(c => c.FacesText is not null);
        Assert.True(aTally.Cases > 300);
        Assert.Empty(aTally.Differences);
    }

    [Fact]
    public void DataOcc_pairs_match_native()
    {
        Tally aTally = CompareAll(c => c.FacesText is null);
        Assert.True(aTally.Cases > 100);
        Assert.Empty(aTally.Differences);
    }

    /// <summary>
    /// Serial = parallel (decision 580): the port's records with the tolerance computation run in parallel, and from
    /// parallel threads (one IntTools_FaceFace and context each), equal the serial records; a repeat equals the
    /// first run.
    /// </summary>
    [Fact]
    public void Serial_equals_parallel_and_repeated()
    {
        List<Case> aCases = [.. Cases.Value.Where((c, i) => i % 3 == 0)];
        string[] aSerial = [.. aCases.Select(c => PortRecord(c))];
        string[] aParallel = new string[aCases.Count];
        Parallel.For(0, aCases.Count, new ParallelOptions { MaxDegreeOfParallelism = 8 },
                     i => aParallel[i] = PortRecord(aCases[i], true));
        string[] aRepeat = [.. aCases.Select(c => PortRecord(c))];
        for (int i = 0; i < aCases.Count; i++)
        {
            Assert.True(aSerial[i] == aParallel[i], $"case {aCases[i].Id}: parallel differs");
            Assert.True(aSerial[i] == aRepeat[i], $"case {aCases[i].Id}: repeat differs");
        }
    }
}
