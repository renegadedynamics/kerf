// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Healing.Custom;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using System.IO.Compression;
using System.Reflection;
using System.Text;
using System.Text.RegularExpressions;
using Xunit.Abstractions;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeCustom against native OCCT's recordings (tools/shcustomrec, unit 7A): BSplineRestriction with restriction
/// parameters build123d.NET's binding can't set, and ShapeCustom_Surface, Curve and Curve2d on the B-spline and Bezier
/// geometry of data/occ's shapes and the heal group's public inputs. The port reads each input frozen, as a caller's,
/// writes the same records as the recorder, and compares them: BRep text with the Free and Checked columns masked
/// where native writes into its own input (decision 347), GeomTools text and the outputs' bits exactly.
/// </summary>
public class ShapeCustomNativeTests(ITestOutputHelper output)
{
    private static string Metadata(string theKey) =>
        typeof(ShapeCustomNativeTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    private static readonly Regex s_flags = new(@"^[01]{7}$", RegexOptions.Multiline);

    private static string MaskFreeAndChecked(string theText) => s_flags.Replace(theText, m => "x" + m.Value[1] + "x" + m.Value[3..]);

    private static string Hex(double theValue) => BitConverter.DoubleToInt64Bits(theValue).ToString("x16", CultureInfo.InvariantCulture);

    private static List<(string Name, List<string> Lines)> Sections()
    {
        using Stream aRes = typeof(ShapeCustomNativeTests).Assembly.GetManifestResourceStream("ShapeCustom.shcustomrec.txt.gz")
                            ?? throw new InvalidOperationException("ShapeCustom.shcustomrec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.Latin1);
        var aSections = new List<(string, List<string>)>();
        string? aLine;
        while ((aLine = aReader.ReadLine()) is not null)
        {
            if (aLine.StartsWith("FILE ", StringComparison.Ordinal))
            {
                aSections.Add((aLine[5..], []));
            }
            else
            {
                aSections[^1].Item2.Add(aLine);
            }
        }
        return aSections;
    }

    // The records of a section: a header line and the lines of its block (the header's last number, for those that
    // have one).
    private static List<(string Head, string Body)> Records(List<string> theLines)
    {
        var aRecords = new List<(string, string)>();
        int i = 0;
        while (i < theLines.Count)
        {
            string aHead = theLines[i++];
            string aKeyword = aHead.Split(' ')[0];
            int aNb = aKeyword is "OUT" or "ANA" or "PER" or "LINE" or "SIMP" ? int.Parse(aHead.Split(' ')[^1], CultureInfo.InvariantCulture) : 0;
            var aBody = new StringBuilder();
            for (int k = 0; k < aNb; k++)
            {
                aBody.Append(theLines[i++]).Append('\n');
            }
            aRecords.Add((aHead, aBody.ToString()));
        }
        return aRecords;
    }

    private sealed class Writer
    {
        public readonly List<(string Head, string Body)> Records = [];

        public void Line(string theHead) => Records.Add((theHead, ""));

        public void Block(string theHead, string theText)
        {
            int aNb = theText.Count(c => c == '\n') + (theText.Length > 0 && theText[^1] != '\n' ? 1 : 0);
            Records.Add(($"{theHead} {aNb}", theText.Length > 0 && theText[^1] != '\n' ? theText + "\n" : theText));
        }

        public void Throw(Exception theException) => Line("THROW " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(theException.GetType()));
    }

    private static string Text(Action<StringWriter> theWrite)
    {
        var aS = new StringWriter(CultureInfo.InvariantCulture) { NewLine = "\n" };
        theWrite(aS);
        return aS.ToString();
    }

    private static string Text(Surface? theS) => theS is null ? "" : Text(w => global::Kerf.Engine.Exchange.Brep.GeomTools.Write(theS, w));

    private static string Text(Curve3D? theC) => theC is null ? "" : Text(w => global::Kerf.Engine.Exchange.Brep.GeomTools.Write(theC, w));

    private static string Text(Curve2D? theC) => theC is null ? "" : Text(w => global::Kerf.Engine.Exchange.Brep.GeomTools.Write(theC, w));

    private static RestrictionParameters Parameters(int theSet)
    {
        var aP = new RestrictionParameters();
        if (theSet == 1)
        {
            aP.ConvertPlane() = true;
            aP.ConvertCylindricalSurf() = true;
            aP.ConvertConicalSurf() = true;
            aP.ConvertSphericalSurf() = true;
            aP.ConvertToroidalSurf() = true;
        }
        else
        {
            aP.ConvertCurve3d() = false;
            aP.ConvertCurve2d() = false;
            aP.ConvertOffsetSurf() = false;
            aP.ConvertOffsetCurv3d() = false;
            aP.ConvertOffsetCurv2d() = false;
            aP.SegmentSurfaceMode() = false;
            aP.ConvertBezierSurf() = true;
            aP.GMaxDegree() = 9;
            aP.GMaxSeg() = 50;
        }
        return aP;
    }

    private static void BSR(Writer theW, in Shape theShape, int theSet)
    {
        theW.Line($"BSR {theSet}");
        try
        {
            Shape aRes = theSet == 1
                                  ? Customize.BSplineRestriction(theShape, 1e-3, 1e-3, 5, 100, C1, C1, true, true, Parameters(1))
                                  : Customize.BSplineRestriction(theShape, 1e-2, 1e-4, 3, 4, C2, C1, false, false, Parameters(2));
            Assert.True(aRes.TShape()!.IsFrozen);
            theW.Block("OUT", Text(w => BrepTools.Write(aRes, w, true, false, global::Kerf.Engine.Exchange.Brep.FormatVersion.FormatVersion_VERSION_3)));
        }
        catch (global::Kerf.Engine.Foundation.FailureException e) when (e is not (global::Kerf.Engine.Foundation.ImmutableObjectException or LockedShapeException))
        {
            theW.Throw(e);
        }
    }

    private static void Geometry(Writer theW, in Shape theShape)
    {
        var aFaces = new IndexedSet<Shape>(ShapeHasher.Instance);
        var anEdges = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, ShapeType.Face, aFaces);
        Exploration.MapShapes(theShape, ShapeType.Edge, anEdges);

        int aNbSurf = 0;
        for (int i = 1; i <= aFaces.Count && aNbSurf < 30; i++)
        {
            Surface? aS = Tool.Surface(ShapeCasts.Face(aFaces[i - 1]), out _);
            if (aS is not (BSplineSurface or BezierSurface))
            {
                continue;
            }
            ++aNbSurf;
            theW.Line($"SURF {i}");
            foreach (double aTol in new[] { 1e-4, 1e-2 })
            {
                try
                {
                    var aTool = new SurfaceCustom(aS);
                    Surface? aRes = aTool.ConvertToAnalytical(aTol, false);
                    theW.Block($"ANA {Hex(aTol)} {Hex(aTool.Gap())}", Text(aRes));
                }
                catch (global::Kerf.Engine.Foundation.FailureException e) when (e is not (global::Kerf.Engine.Foundation.ImmutableObjectException or LockedShapeException))
                {
                    theW.Throw(e);
                }
            }
            try
            {
                // the caller's frozen surface itself: the port converts a copy where OCCT changes it (decision 974)
                bool wasPeriodic = aS.IsUPeriodic || aS.IsVPeriodic;
                Surface? aRes = new SurfaceCustom(aS).ConvertToPeriodic(false);
                Assert.Equal(wasPeriodic, aS.IsUPeriodic || aS.IsVPeriodic);
                Assert.True(aRes is null || !ReferenceEquals(aRes, aS));
                theW.Block("PER", Text(aRes));
            }
            catch (global::Kerf.Engine.Foundation.FailureException e) when (e is not (global::Kerf.Engine.Foundation.ImmutableObjectException or LockedShapeException))
            {
                theW.Throw(e);
            }
        }

        int aNbCurv = 0;
        for (int i = 1; i <= anEdges.Count && aNbCurv < 30; i++)
        {
            Curve3D? aC = Tool.Curve(ShapeCasts.Edge(anEdges[i - 1]), out _, out _, out _);
            if (aC is not BSplineCurve3D)
            {
                continue;
            }
            ++aNbCurv;
            theW.Line($"CURV {i}");
            try
            {
                bool wasPeriodic = aC.IsPeriodic;
                Curve3D? aRes = new CustomCurve(aC).ConvertToPeriodic(false);
                Assert.Equal(wasPeriodic, aC.IsPeriodic);
                theW.Block("PER", Text(aRes));
            }
            catch (global::Kerf.Engine.Foundation.FailureException e) when (e is not (global::Kerf.Engine.Foundation.ImmutableObjectException or LockedShapeException))
            {
                theW.Throw(e);
            }
        }

        int aNbPCurv = 0;
        for (int j = 1; j <= aFaces.Count && aNbPCurv < 40; j++)
        {
            Face aF = ShapeCasts.Face(aFaces[j - 1]);
            var aFaceEdges = new IndexedSet<Shape>(ShapeHasher.Instance);
            Exploration.MapShapes(aF, ShapeType.Edge, aFaceEdges);
            for (int k = 1; k <= aFaceEdges.Count && aNbPCurv < 40; k++)
            {
                Edge anE = ShapeCasts.Edge(aFaceEdges[k - 1]);
                Curve2D? aC = Tool.CurveOnSurface(anE, aF, out double f, out double l);
                if (aC is not (BSplineCurve2D or BezierCurve2D))
                {
                    continue;
                }
                ++aNbPCurv;
                theW.Line($"PCURVE {(anEdges.IndexOf(anE) + 1)} {j}");
                try
                {
                    double cf = 0, cl = 0, aDev = 0;
                    LineCurve2D? aLine = CustomCurve2d.ConvertToLine2d(aC, f, l, 1e-4, ref cf, ref cl, ref aDev);
                    theW.Block($"LINE {Hex(cf)} {Hex(cl)} {Hex(aDev)}", Text(aLine));
                }
                catch (global::Kerf.Engine.Foundation.FailureException e) when (e is not (global::Kerf.Engine.Foundation.ImmutableObjectException or LockedShapeException))
                {
                    theW.Throw(e);
                }
                if (aC is BSplineCurve2D aBS)
                {
                    try
                    {
                        // the caller's frozen pcurve itself: the port removes the knots from a copy (decision 974)
                        int aNbKnots = aBS.KnotCount;
                        BSplineCurve2D aCurve = aBS;
                        bool isDone = CustomCurve2d.SimplifyBSpline2d(ref aCurve, global::Kerf.Engine.Foundation.Precision.Approximation());
                        Assert.Equal(aNbKnots, aBS.KnotCount);
                        theW.Block("SIMP " + (isDone ? "1" : "0"), Text(aCurve));
                    }
                    catch (global::Kerf.Engine.Foundation.FailureException e) when (e is not (global::Kerf.Engine.Foundation.ImmutableObjectException or LockedShapeException))
                    {
                        theW.Throw(e);
                    }
                }
            }
        }
    }

    [Fact]
    public void BSplineRestrictionAndTheGeometryTools_AsNativeOCCT()
    {
        var aByName = new Dictionary<string, string>(StringComparer.Ordinal);
        foreach (string aFile in Directory.GetFiles(Path.Combine(Metadata("OcctSourceDir"), "data", "occ"), "*.brep"))
        {
            aByName[Path.GetFileName(aFile)] = aFile;
        }
        string aData = Metadata("OcctTestDataDir");
        if (!string.IsNullOrEmpty(aData) && Directory.Exists(aData))
        {
            foreach (string aFile in Directory.EnumerateFiles(aData, "*", SearchOption.AllDirectories))
            {
                aByName.TryAdd(Path.GetFileName(aFile), aFile);
            }
        }

        int aFiles = 0, aMissing = 0, aRecords = 0, aMasked = 0;
        var aCounts = new SortedDictionary<string, int>(StringComparer.Ordinal);
        var aDifferences = new List<string>();
        foreach ((string aName, List<string> aLines) in Sections())
        {
            if (!aByName.TryGetValue(aName, out string? aPath))
            {
                aMissing++;
                continue;
            }
            aFiles++;
            Assert.True(BrepTools.Read(out Shape aShape, aPath), aPath);
            aShape.TShape()!.FreezeGraph(); // a caller's shape
            string anInput = Text(w => BrepTools.Write(aShape, w, true, false, global::Kerf.Engine.Exchange.Brep.FormatVersion.FormatVersion_VERSION_3));

            var aW = new Writer();
            if (new FileInfo(aPath).Length <= 30000)
            {
                BSR(aW, aShape, 1);
                BSR(aW, aShape, 2);
            }
            Geometry(aW, aShape);
            Assert.Equal(anInput, Text(w => BrepTools.Write(aShape, w, true, false, global::Kerf.Engine.Exchange.Brep.FormatVersion.FormatVersion_VERSION_3)));

            List<(string Head, string Body)> aNative = Records(aLines);
            if (aNative.Count != aW.Records.Count)
            {
                aDifferences.Add($"{aName}: {aNative.Count} records natively, {aW.Records.Count} in the port");
                continue;
            }
            for (int i = 0; i < aNative.Count; i++)
            {
                (string aNH, string aNB) = aNative[i];
                (string aPH, string aPB) = aW.Records[i];
                string aKeyword = aNH.Split(' ')[0];
                aCounts[aKeyword] = aCounts.GetValueOrDefault(aKeyword) + 1;
                aRecords++;
                if (aNH == aPH && aNB == aPB)
                {
                    continue;
                }
                if (aKeyword == "OUT" && aNH == aPH && MaskFreeAndChecked(aNB) == MaskFreeAndChecked(aPB))
                {
                    aMasked++;
                    continue;
                }
                aDifferences.Add($"{aName} record {i}: '{aNH}' natively, '{aPH}' in the port");
            }
        }
        output.WriteLine($"{aFiles} inputs ({aMissing} not found), {aRecords} records ({string.Join(", ", aCounts.Select(c => $"{c.Key} {c.Value}"))}): "
                         + $"{aRecords - aDifferences.Count - aMasked} identical, {aMasked} identical with the Free and Checked columns masked, "
                         + $"{aDifferences.Count} different");
        foreach (string aD in aDifferences.Take(40))
        {
            output.WriteLine("  " + aD);
        }
        Assert.Empty(aDifferences);
        Assert.True(aFiles > 0);
    }
}
