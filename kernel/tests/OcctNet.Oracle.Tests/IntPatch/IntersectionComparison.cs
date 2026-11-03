// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using System.Text;
using static Kerf.Engine.Exchange.Brep.FormatVersion;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// How surface/surface intersection results are compared with native OCCT (decision 578): the number of lines, then
/// each line as the BRep text of an edge on it: its type (the curve record's code), every value of the curve record
/// (analytic parameters, or degree, poles, weights and knots), bit for bit; where the records differ, the largest
/// distance between the two curves at the same parameters. 2D lines (pcurves) are compared the same way, through an
/// edge on a fixed plane.
/// </summary>
/// <remarks>
/// The surface/surface units (5B to 5G) call it with the lines of <c>GeomAPI_IntSS</c> (or GeomInt_IntSS's pcurves) on
/// both sides; <see cref="Report"/> gives the tallies of section 2.2 of the plan (lines bit-identical, values
/// bit-identical, largest deviation). A difference in the number of lines, in a line's type or in the shape of its
/// record (degree, numbers of poles and knots) fails the comparison: the targets ask for the same lines, "same count
/// and type" at least. The texts are written by BRepTools (version 3) on both sides, whose 17 significant digits
/// round-trip every double, so equal tokens are equal bits.
/// </remarks>
internal sealed class IntersectionComparison : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-intss-").FullName;
    private int myFileNumber;

    /// <summary>Lines compared.</summary>
    public int Lines { get; private set; }

    /// <summary>Lines whose records are identical (every value bit for bit).</summary>
    public int IdenticalLines { get; private set; }

    /// <summary>Values compared in the lines' records.</summary>
    public int Values { get; private set; }

    /// <summary>Values bit-identical.</summary>
    public int IdenticalValues { get; private set; }

    /// <summary>The largest distance between two lines whose records differ, at the same parameters.</summary>
    public double MaxDeviation { get; private set; }

    /// <summary>The comparison's description of the line with the largest deviation.</summary>
    public string MaxDeviationWhat { get; private set; } = "";

    public void Dispose() => Directory.Delete(myDirectory, true);

    /// <summary>Compares two lists of 3D lines, in order.</summary>
    public void CompareLines(IReadOnlyList<N.Geom_Curve> theNative, IReadOnlyList<Curve3D> thePort, string theWhat)
    {
        Assert.True(theNative.Count == thePort.Count, $"{theWhat}: native {theNative.Count} lines, port {thePort.Count}");
        for (int i = 0; i < theNative.Count; i++)
        {
            CompareLine(theNative[i], thePort[i], $"{theWhat}, line {i + 1}");
        }
    }

    /// <summary>Compares two 3D lines.</summary>
    public void CompareLine(N.Geom_Curve theNative, Curve3D thePort, string theWhat)
    {
        string aNText = NativeText(new N.BRepBuilderAPI_MakeEdge(theNative).Edge());
        string aPText = PortText(new MakeEdge(thePort).Edge());
        if (CompareRecords(Section(aNText, "Curves"), Section(aPText, "Curves"), theWhat))
        {
            return;
        }

        // The native line as the port reads it from the native text, to measure the distance between the lines.
        Curve3D aNCurve = ReadCurve(aNText, out double aFirst, out double aLast);
        Deviation(aFirst, aLast, u => aNCurve.Value(u).Distance(thePort.Value(u)), theWhat);
    }

    /// <summary>
    /// True if the two 3D lines' records are bit-identical (every token equal), without tallying or asserting: for
    /// matching lines whose order differs between the two sides (IntTools_FaceFace against GeomAPI_IntSS, unit 6D).
    /// </summary>
    public bool SameRecord(N.Geom_Curve theNative, Curve3D thePort)
    {
        string aNText = Section(NativeText(new N.BRepBuilderAPI_MakeEdge(theNative).Edge()), "Curves");
        string aPText = Section(PortText(new MakeEdge(thePort).Edge()), "Curves");
        return Tokens(aNText).SequenceEqual(Tokens(aPText));
    }

    /// <summary>Compares two 2D lines.</summary>
    public void CompareLine2d(N.Geom2d_Curve theNative, Curve2D thePort, string theWhat)
    {
        using var aNPlane = new N.Geom_Plane(new N.gp_Pln());
        string aNText = NativeText(new N.BRepBuilderAPI_MakeEdge(theNative, aNPlane).Edge());
        string aPText = PortText(new MakeEdge(thePort, new PlaneSurface(new Plane3D())).Edge());
        if (CompareRecords(Section(aNText, "Curve2ds"), Section(aPText, "Curve2ds"), theWhat))
        {
            return;
        }

        Assert.True(BrepTools.Read(out Shape aShape, Save(aNText)));
        Tool.CurveOnSurface(ShapeCasts.Edge(aShape), out Curve2D? aNCurve, out _, out _, out double aFirst,
                                 out double aLast);
        Assert.NotNull(aNCurve);
        Deviation(aFirst, aLast, u => aNCurve.Value(u).Distance(thePort.Value(u)), theWhat);
    }

    /// <summary>A native curve as the port reads it from the native BRep text of an edge on it (the same bits).</summary>
    public Curve3D PortCurve(N.Geom_Curve theNative)
        => ReadCurve(NativeText(new N.BRepBuilderAPI_MakeEdge(theNative).Edge()), out _, out _);

    /// <summary>
    /// A native curve read back by native OCCT from its own BRep text: reading builds directions with gp_Dir's
    /// constructor, which normalises them again, so a round trip can change their last bits. Compare the port's
    /// reading (<see cref="PortCurve"/>) with this, not with the curve written.
    /// </summary>
    public N.Geom_Curve NativeRoundTrip(N.Geom_Curve theNative)
    {
        string aFile = Save(NativeText(new N.BRepBuilderAPI_MakeEdge(theNative).Edge()));
        using var aShape = new N.TopoDS_Shape();
        using var aBuilder = new N.BRep_Builder();
        Assert.True(N.BRepTools.Read(aShape, aFile, aBuilder));
        double aFirst = 0.0, aLast = 0.0;
        return N.BRep_Tool.Curve(N.TopoDS.Edge(aShape), ref aFirst, ref aLast)!;
    }

    /// <summary>"what: a/b lines bit-identical, c/d values bit-identical, largest deviation e".</summary>
    public string Report(string theWhat)
        => string.Create(CultureInfo.InvariantCulture,
                         $"{theWhat}: {IdenticalLines}/{Lines} lines bit-identical, {IdenticalValues}/{Values} values bit-identical, largest deviation {MaxDeviation:E2}{(MaxDeviation > 0 ? " (" + MaxDeviationWhat + ")" : "")}");

    // Compares the records; true if they are identical.
    private bool CompareRecords(string theNative, string thePort, string theWhat)
    {
        Lines++;
        string[] aNTokens = Tokens(theNative), aPTokens = Tokens(thePort);
        Assert.True(aNTokens.Length > 0 && aPTokens.Length > 0, $"{theWhat}: no curve record");
        Assert.True(aNTokens[0] == aPTokens[0], $"{theWhat}: native curve type {aNTokens[0]}, port {aPTokens[0]}");
        Assert.True(aNTokens.Length == aPTokens.Length,
                    $"{theWhat}: the records differ in shape (native {aNTokens.Length} values, port {aPTokens.Length})");
        int anIdentical = 0;
        for (int i = 0; i < aNTokens.Length; i++)
        {
            if (aNTokens[i] == aPTokens[i])
            {
                anIdentical++;
            }
            else
            {
                bool anIsNumber = double.TryParse(aNTokens[i], NumberStyles.Float, CultureInfo.InvariantCulture, out _)
                                  && double.TryParse(aPTokens[i], NumberStyles.Float, CultureInfo.InvariantCulture, out _);
                Assert.True(anIsNumber, $"{theWhat}: value {i} differs in kind (native {aNTokens[i]}, port {aPTokens[i]})");
            }
        }
        Values += aNTokens.Length;
        IdenticalValues += anIdentical;
        if (anIdentical == aNTokens.Length)
        {
            IdenticalLines++;
            return true;
        }
        if (Environment.GetEnvironmentVariable("OCCTNET_DUMP_INTSS") != null)
        {
            File.AppendAllText(Environment.GetEnvironmentVariable("OCCTNET_DUMP_INTSS")!,
                               $"{theWhat}\nN: {theNative}\nP: {thePort}\n");
        }
        return false;
    }

    private void Deviation(double theFirst, double theLast, Func<double, double> theDistance, string theWhat)
    {
        // an infinite line is compared on [-10, 10] about its origin (a hyperbola's points grow as cosh)
        double aFirst = global::Kerf.Engine.Foundation.Precision.IsInfinite(theFirst) ? -10.0 : theFirst;
        double aLast = global::Kerf.Engine.Foundation.Precision.IsInfinite(theLast) ? 10.0 : theLast;
        const int aNbSamples = 64;
        double aMax = 0.0;
        for (int i = 0; i <= aNbSamples; i++)
        {
            double u = aFirst + (aLast - aFirst) * i / aNbSamples;
            aMax = Math.Max(aMax, theDistance(u));
        }
        if (aMax > MaxDeviation)
        {
            MaxDeviation = aMax;
            MaxDeviationWhat = theWhat;
        }
    }

    private Curve3D ReadCurve(string theText, out double theFirst, out double theLast)
    {
        Assert.True(BrepTools.Read(out Shape aShape, Save(theText)));
        return Tool.Curve(ShapeCasts.Edge(aShape), out theFirst, out theLast)!;
    }

    private static string[] Tokens(string theSection)
        => theSection.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);

    // The records of a section of a BRep text ("Curves", "Curve2ds"): the lines after the header, up to the next
    // section, the header's count dropped.
    private static string Section(string theText, string theName)
    {
        string[] aLines = theText.Split('\n');
        int aStart = Array.FindIndex(aLines, l => l.StartsWith(theName + " ", StringComparison.Ordinal));
        Assert.True(aStart >= 0, $"no {theName} section");
        StringBuilder aRecords = new();
        for (int i = aStart + 1; i < aLines.Length; i++)
        {
            string aLine = aLines[i];
            if (aLine.Length > 0 && char.IsLetter(aLine[0]))
            {
                break; // the next section
            }
            aRecords.Append(aLine).Append('\n');
        }
        return aRecords.ToString();
    }

    private string NewFile() => Path.Combine(myDirectory, $"l{myFileNumber++}.brep");

    private string Save(string theText)
    {
        string aFile = NewFile();
        File.WriteAllText(aFile, theText, Encoding.Latin1);
        return aFile;
    }

    private string NativeText(N.TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(N.BRepTools.Write(theShape, aFile, true, true, N.TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3));
        return File.ReadAllText(aFile, Encoding.Latin1);
    }

    private string PortText(in Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(BrepTools.Write(theShape, aFile, true, true, FormatVersion_VERSION_3));
        return File.ReadAllText(aFile, Encoding.Latin1);
    }
}
