// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.IO.Compression;
using System.Reflection;
using System.Text;
using System.Text.RegularExpressions;
using Xunit.Abstractions;
using static Kerf.Geometry.Continuity;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

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

    private static string Text(Geom_Surface? theS) => theS is null ? "" : Text(w => GeomTools.Write(theS, w));

    private static string Text(Geom_Curve? theC) => theC is null ? "" : Text(w => GeomTools.Write(theC, w));

    private static string Text(Geom2d_Curve? theC) => theC is null ? "" : Text(w => GeomTools.Write(theC, w));

    private static ShapeCustom_RestrictionParameters Parameters(int theSet)
    {
        var aP = new ShapeCustom_RestrictionParameters();
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

    private static void BSR(Writer theW, in TopoDS_Shape theShape, int theSet)
    {
        theW.Line($"BSR {theSet}");
        try
        {
            TopoDS_Shape aRes = theSet == 1
                                  ? ShapeCustom.BSplineRestriction(theShape, 1e-3, 1e-3, 5, 100, C1, C1, true, true, Parameters(1))
                                  : ShapeCustom.BSplineRestriction(theShape, 1e-2, 1e-4, 3, 4, C2, C1, false, false, Parameters(2));
            Assert.True(aRes.TShape()!.IsFrozen);
            theW.Block("OUT", Text(w => BRepTools.Write(aRes, w, true, false, TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3)));
        }
        catch (global::Kerf.Engine.Foundation.FailureException e) when (e is not (global::Kerf.Engine.Foundation.ImmutableObjectException or TopoDS_LockedShape))
        {
            theW.Throw(e);
        }
    }

    private static void Geometry(Writer theW, in TopoDS_Shape theShape)
    {
        var aFaces = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var anEdges = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, TopAbs_FACE, aFaces);
        TopExp.MapShapes(theShape, TopAbs_EDGE, anEdges);

        int aNbSurf = 0;
        for (int i = 1; i <= aFaces.Extent() && aNbSurf < 30; i++)
        {
            Geom_Surface? aS = BRep_Tool.Surface(TopoDS.Face(aFaces[i]), out _);
            if (aS is not (Geom_BSplineSurface or Geom_BezierSurface))
            {
                continue;
            }
            ++aNbSurf;
            theW.Line($"SURF {i}");
            foreach (double aTol in new[] { 1e-4, 1e-2 })
            {
                try
                {
                    var aTool = new ShapeCustom_Surface(aS);
                    Geom_Surface? aRes = aTool.ConvertToAnalytical(aTol, false);
                    theW.Block($"ANA {Hex(aTol)} {Hex(aTool.Gap())}", Text(aRes));
                }
                catch (global::Kerf.Engine.Foundation.FailureException e) when (e is not (global::Kerf.Engine.Foundation.ImmutableObjectException or TopoDS_LockedShape))
                {
                    theW.Throw(e);
                }
            }
            try
            {
                // the caller's frozen surface itself: the port converts a copy where OCCT changes it (decision 974)
                bool wasPeriodic = aS.IsUPeriodic() || aS.IsVPeriodic();
                Geom_Surface? aRes = new ShapeCustom_Surface(aS).ConvertToPeriodic(false);
                Assert.Equal(wasPeriodic, aS.IsUPeriodic() || aS.IsVPeriodic());
                Assert.True(aRes is null || !ReferenceEquals(aRes, aS));
                theW.Block("PER", Text(aRes));
            }
            catch (global::Kerf.Engine.Foundation.FailureException e) when (e is not (global::Kerf.Engine.Foundation.ImmutableObjectException or TopoDS_LockedShape))
            {
                theW.Throw(e);
            }
        }

        int aNbCurv = 0;
        for (int i = 1; i <= anEdges.Extent() && aNbCurv < 30; i++)
        {
            Geom_Curve? aC = BRep_Tool.Curve(TopoDS.Edge(anEdges[i]), out _, out _, out _);
            if (aC is not Geom_BSplineCurve)
            {
                continue;
            }
            ++aNbCurv;
            theW.Line($"CURV {i}");
            try
            {
                bool wasPeriodic = aC.IsPeriodic();
                Geom_Curve? aRes = new ShapeCustom_Curve(aC).ConvertToPeriodic(false);
                Assert.Equal(wasPeriodic, aC.IsPeriodic());
                theW.Block("PER", Text(aRes));
            }
            catch (global::Kerf.Engine.Foundation.FailureException e) when (e is not (global::Kerf.Engine.Foundation.ImmutableObjectException or TopoDS_LockedShape))
            {
                theW.Throw(e);
            }
        }

        int aNbPCurv = 0;
        for (int j = 1; j <= aFaces.Extent() && aNbPCurv < 40; j++)
        {
            TopoDS_Face aF = TopoDS.Face(aFaces[j]);
            var aFaceEdges = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapes(aF, TopAbs_EDGE, aFaceEdges);
            for (int k = 1; k <= aFaceEdges.Extent() && aNbPCurv < 40; k++)
            {
                TopoDS_Edge anE = TopoDS.Edge(aFaceEdges[k]);
                Geom2d_Curve? aC = BRep_Tool.CurveOnSurface(anE, aF, out double f, out double l);
                if (aC is not (Geom2d_BSplineCurve or Geom2d_BezierCurve))
                {
                    continue;
                }
                ++aNbPCurv;
                theW.Line($"PCURVE {anEdges.FindIndex(anE)} {j}");
                try
                {
                    double cf = 0, cl = 0, aDev = 0;
                    Geom2d_Line? aLine = ShapeCustom_Curve2d.ConvertToLine2d(aC, f, l, 1e-4, ref cf, ref cl, ref aDev);
                    theW.Block($"LINE {Hex(cf)} {Hex(cl)} {Hex(aDev)}", Text(aLine));
                }
                catch (global::Kerf.Engine.Foundation.FailureException e) when (e is not (global::Kerf.Engine.Foundation.ImmutableObjectException or TopoDS_LockedShape))
                {
                    theW.Throw(e);
                }
                if (aC is Geom2d_BSplineCurve aBS)
                {
                    try
                    {
                        // the caller's frozen pcurve itself: the port removes the knots from a copy (decision 974)
                        int aNbKnots = aBS.NbKnots();
                        Geom2d_BSplineCurve aCurve = aBS;
                        bool isDone = ShapeCustom_Curve2d.SimplifyBSpline2d(ref aCurve, global::Kerf.Engine.Foundation.Precision.Approximation());
                        Assert.Equal(aNbKnots, aBS.NbKnots());
                        theW.Block("SIMP " + (isDone ? "1" : "0"), Text(aCurve));
                    }
                    catch (global::Kerf.Engine.Foundation.FailureException e) when (e is not (global::Kerf.Engine.Foundation.ImmutableObjectException or TopoDS_LockedShape))
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
            Assert.True(BRepTools.Read(out TopoDS_Shape aShape, aPath), aPath);
            aShape.TShape()!.FreezeGraph(); // a caller's shape
            string anInput = Text(w => BRepTools.Write(aShape, w, true, false, TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3));

            var aW = new Writer();
            if (new FileInfo(aPath).Length <= 30000)
            {
                BSR(aW, aShape, 1);
                BSR(aW, aShape, 2);
            }
            Geometry(aW, aShape);
            Assert.Equal(anInput, Text(w => BRepTools.Write(aShape, w, true, false, TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3)));

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
