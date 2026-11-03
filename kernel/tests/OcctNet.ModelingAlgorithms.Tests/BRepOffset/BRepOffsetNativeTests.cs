// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Brep;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Globalization;
using System.IO.Compression;
using System.Reflection;
using System.Text;
using System.Text.RegularExpressions;
using Xunit.Abstractions;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// The offset foundations against native OCCT's recordings (tools/offsetrec, unit 7E): BRepOffset_Analyse's edge types
/// and tangent explosion, BRepOffset_Offset's faces (both signs of the offset, every face up to 30 per input) and
/// BRepOffset_MakeSimpleOffset's results (both signs, with and without the solid), on primitives, Boolean and fillet
/// results (built natively, their BRep text recorded as the input) and data/occ's shapes. The port reads each input
/// frozen, as a caller's, follows the recording's headers (the offset values are native's bits) and compares: the
/// intervals' bits and types exactly, BRep text byte for byte, with the Free and Checked columns masked where only
/// they differ (open question 1).
/// </summary>
public class BRepOffsetNativeTests(ITestOutputHelper output)
{
    private static string Metadata(string theKey) =>
        typeof(BRepOffsetNativeTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    private static readonly Regex s_flags = new(@"^[01]{7}$", RegexOptions.Multiline);

    private static string MaskFreeAndChecked(string theText) => s_flags.Replace(theText, m => "x" + m.Value[1] + "x" + m.Value[3..]);

    private static string Hex(double theValue) => BitConverter.DoubleToInt64Bits(theValue).ToString("x16", CultureInfo.InvariantCulture);

    private static double FromHex(string theHex) => BitConverter.Int64BitsToDouble(long.Parse(theHex, NumberStyles.HexNumber, CultureInfo.InvariantCulture));

    private static readonly HashSet<string> s_blocks = ["INPUT", "ANA", "STATUS", "DONE"];

    // The recording's records: a header and, for the keywords with a block, the block's lines (the header's last number)
    private static List<(string Head, string Body)> Records()
    {
        using Stream aRes = typeof(BRepOffsetNativeTests).Assembly.GetManifestResourceStream("BRepOffset.offsetrec.txt.gz")
                            ?? throw new InvalidOperationException("BRepOffset.offsetrec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.Latin1);
        var aRecords = new List<(string, string)>();
        string? aLine;
        while ((aLine = aReader.ReadLine()) is not null)
        {
            string aKeyword = aLine.Split(' ')[0];
            var aBody = new StringBuilder();
            if (s_blocks.Contains(aKeyword))
            {
                int aNb = int.Parse(aLine.Split(' ')[^1], CultureInfo.InvariantCulture);
                for (int k = 0; k < aNb; k++)
                {
                    aBody.Append(aReader.ReadLine()).Append('\n');
                }
            }
            aRecords.Add((aLine, aBody.ToString()));
        }
        return aRecords;
    }

    private static string Text(in Shape theS)
    {
        var aS = new StringWriter(CultureInfo.InvariantCulture) { NewLine = "\n" };
        BrepTools.Write(theS, aS, true, false, FormatVersion.FormatVersion_VERSION_3);
        return aS.ToString();
    }

    private static (string Head, string Body) Block(string theHead, string theText)
    {
        int aNb = theText.Count(c => c == '\n') + (theText.Length > 0 && theText[^1] != '\n' ? 1 : 0);
        return ($"{theHead} {aNb}", theText.Length > 0 && theText[^1] != '\n' ? theText + "\n" : theText);
    }

    private static (string, string) Throw(Exception theE) => ("THROW " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(theE.GetType()), "");

    // The port's records for the native header theHead on theShape (one, or two for ANA: the types and EXPLODE)
    private static List<(string Head, string Body)> Port(string theHead, in Shape theShape)
    {
        string[] aWords = theHead.Split(' ');
        var aRes = new List<(string, string)>();
        switch (aWords[0])
        {
            case "ANA":
            {
                double anAngle = FromHex(aWords[1]);
                try
                {
                    var anA = new BRepOffset_Analyse(theShape, anAngle);
                    var anEdges = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
                    Exploration.MapShapes(theShape, ShapeType.Edge, anEdges);
                    var aS = new StringBuilder();
                    for (int i = 1; i <= anEdges.Extent(); i++)
                    {
                        NCollection_List<BRepOffset_Interval> aL = anA.Type(ShapeCasts.Edge(anEdges[i]));
                        aS.Append(i.ToString(CultureInfo.InvariantCulture)).Append(' ').Append(aL.Extent().ToString(CultureInfo.InvariantCulture));
                        foreach (BRepOffset_Interval anI in aL)
                        {
                            aS.Append(' ').Append(Hex(anI.First())).Append(' ').Append(Hex(anI.Last())).Append(' ')
                              .Append(((int)anI.Type()).ToString(CultureInfo.InvariantCulture));
                        }
                        aS.Append('\n');
                    }
                    aRes.Add(Block("ANA " + aWords[1], aS.ToString()));
                    var aCompounds = new NCollection_List<Shape>();
                    anA.Explode(aCompounds, ChFiDS_TypeOfConcavity.ChFiDS_Tangential);
                    var anExplode = new StringBuilder("EXPLODE ").Append(aCompounds.Extent().ToString(CultureInfo.InvariantCulture));
                    foreach (Shape aC in aCompounds)
                    {
                        int aNb = 0;
                        for (var anExp = new Explorer(aC, ShapeType.Face); anExp.More(); anExp.Next())
                        {
                            ++aNb;
                        }
                        anExplode.Append(' ').Append(aNb.ToString(CultureInfo.InvariantCulture));
                    }
                    aRes.Add((anExplode.ToString(), ""));
                }
                catch (FailureException e) when (e is not (ImmutableObjectException or LockedShapeException))
                {
                    aRes.Add(Throw(e));
                }
                break;
            }
            case "OFF":
            {
                var aFaces = new NCollection_IndexedMap<Shape>(ShapeHasher.Instance);
                Exploration.MapShapes(theShape, ShapeType.Face, aFaces);
                aRes.Add((theHead, ""));
                try
                {
                    var anOff = new BRepOffset_Offset(ShapeCasts.Face(aFaces[int.Parse(aWords[1], CultureInfo.InvariantCulture)]), FromHex(aWords[2]));
                    Face aF = anOff.Face();
                    Assert.True(aF.TShape()!.IsFrozen);
                    aRes.Add(Block("STATUS " + ((int)anOff.Status()).ToString(CultureInfo.InvariantCulture), Text(aF)));
                }
                catch (FailureException e) when (e is not (ImmutableObjectException or LockedShapeException))
                {
                    aRes.Add(Throw(e));
                }
                break;
            }
            case "SIMPLE":
            {
                aRes.Add((theHead, ""));
                try
                {
                    var aMaker = new BRepOffset_MakeSimpleOffset(theShape, FromHex(aWords[1]));
                    aMaker.SetBuildSolidFlag(aWords[2] == "1");
                    aMaker.Perform();
                    string aText = "";
                    if (aMaker.IsDone())
                    {
                        Shape aR = aMaker.GetResultShape();
                        Assert.True(aR.TShape()!.IsFrozen);
                        aText = Text(aR);
                    }
                    aRes.Add(Block($"DONE {(aMaker.IsDone() ? "1" : "0")} {((int)aMaker.GetError()).ToString(CultureInfo.InvariantCulture)}", aText));
                }
                catch (FailureException e) when (e is not (ImmutableObjectException or LockedShapeException))
                {
                    aRes.Add(Throw(e));
                }
                break;
            }
            default:
                throw new InvalidOperationException(theHead);
        }
        return aRes;
    }

    [Fact]
    public void AnalyseOffsetAndSimpleOffset_AsNativeOCCT()
    {
        List<(string Head, string Body)> aNative = Records();
        string anOcc = Path.Combine(Metadata("OcctSourceDir"), "data", "occ");
        int anInputs = 0, aRecords = 0, aMasked = 0, aSameException = 0;
        var aCounts = new SortedDictionary<string, int>(StringComparer.Ordinal);
        var aDifferences = new List<string>();
        int i = 0;
        while (i < aNative.Count)
        {
            (string aHead, string aBody) = aNative[i++];
            Shape aShape;
            string aName;
            if (aHead.StartsWith("INPUT ", StringComparison.Ordinal))
            {
                aName = aHead.Split(' ')[1];
                BrepTools.Read(out aShape, new StringReader(aBody));
            }
            else
            {
                Assert.StartsWith("FILE ", aHead);
                aName = aHead[5..];
                Assert.True(BrepTools.Read(out aShape, Path.Combine(anOcc, aName)), aName);
            }
            anInputs++;
            aShape.TShape()!.FreezeGraph(); // a caller's shape
            string anInput = Text(aShape);
            while (i < aNative.Count && !aNative[i].Head.StartsWith("INPUT ", StringComparison.Ordinal)
                   && !aNative[i].Head.StartsWith("FILE ", StringComparison.Ordinal))
            {
                string aCase = aNative[i].Head;
                List<(string Head, string Body)> aPort = Port(aCase, aShape);
                for (int k = 0; k < aPort.Count; k++, i++)
                {
                    (string aNH, string aNB) = i < aNative.Count ? aNative[i] : ("(end)", "");
                    (string aPH, string aPB) = aPort[k];
                    string aKeyword = aNH.Split(' ')[0];
                    aCounts[aKeyword] = aCounts.GetValueOrDefault(aKeyword) + 1;
                    aRecords++;
                    if (aNH == aPH && aNB == aPB)
                    {
                        continue;
                    }
                    if (aNH == aPH && MaskFreeAndChecked(aNB) == MaskFreeAndChecked(aPB))
                    {
                        aMasked++;
                        continue;
                    }
                    // OCCT dereferences a null shape where the port's TopoDS throws (MakeSimpleOffset on a shape
                    // without faces, occt-issues.md): the same failure, as decision 977 counts it
                    if (aNH == "THROW OSD_Exception_ACCESS_VIOLATION" && aPH is "THROW Standard_NullObject" or "THROW NullReferenceException")
                    {
                        aSameException++;
                        continue;
                    }
                    aDifferences.Add($"{aName} '{aCase}': '{aNH}' natively, '{aPH}' in the port");
                    string? aDump = Environment.GetEnvironmentVariable("OCCTNET_OFFSETREC_DUMP");
                    if (!string.IsNullOrEmpty(aDump))
                    {
                        string aBase = Path.Combine(aDump, $"{aName}-{aDifferences.Count}");
                        File.WriteAllText(aBase + ".native.txt", aNB);
                        File.WriteAllText(aBase + ".port.txt", aPB);
                    }
                }
                // OFF and SIMPLE are two records: the header (compared above with itself) and the outcome; a THROW
                // outcome natively against a block in the port (or the reverse) shows as a difference above
            }
            Assert.Equal(anInput, Text(aShape));
        }
        output.WriteLine($"{anInputs} inputs, {aRecords} records ({string.Join(", ", aCounts.Select(c => $"{c.Key} {c.Value}"))}): "
                         + $"{aRecords - aDifferences.Count - aMasked - aSameException} identical, {aMasked} identical with the Free and Checked columns masked, "
                         + $"{aSameException} the same exception, "
                         + $"{aDifferences.Count} different");
        foreach (string aD in aDifferences.Take(60))
        {
            output.WriteLine("  " + aD);
        }
        Assert.Empty(aDifferences);
    }
}
