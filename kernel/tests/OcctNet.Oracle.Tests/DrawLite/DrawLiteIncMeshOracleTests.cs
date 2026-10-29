#if HAS_ORACLE
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Reflection;
using System.Text;
using System.Text.RegularExpressions;
using OcctNet.DrawLite;
using Xunit.Abstractions;
using static Occt.TopTools_FormatVersion;
using N = Build123d.Occt;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// Draw-lite's incmesh and vdisplay (unit 4L2) against native OCCT: the options the native five-argument
/// BRepMesh_IncrementalMesh constructor can express (the deflection, -relative, -a, -parallel), and AIS's meshing of a
/// displayed shape (the deflection relative to the bounding box, 20 degrees, serial: StdSelect_BRepSelectionTool::Load
/// and StdPrs_ToolTriangulatedShape::Tessellate). Native writes each shape without meshes (F1); Draw-lite restores F1,
/// explodes its faces and runs the command; native meshes F1 in place with the same arguments. Compared: the meshed
/// shape's BRep text (byte for byte, or but for decision 551's Checked flags), the text of the first face exploded before
/// meshing (rebound by decision 495; natively the same TShape, meshed in place), trinfo's faces, triangles, nodes and
/// maximal deflection, and the "Meshing statuses" line against native GetStatusFlags.
/// </summary>
public sealed partial class DrawLiteIncMeshOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-drawincmesh-").FullName;
    private readonly List<string> myDisagreements = [];
    private int myFileNumber;
    private int myRuns;
    private int myTextsSame;
    private int myTextsSameButChecked;
    private int myValues;
    private int mySubShapes;

    public void Dispose()
    {
        try
        {
            Directory.Delete(myDirectory, true);
        }
        catch (IOException)
        {
        }
    }

    private static string Metadata(string theKey)
        => typeof(DrawLiteIncMeshOracleTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    private static string ResourcesDirectory => Path.Combine(Metadata("OcctSourceDir"), "resources", "DrawResources");

    private static string DataOcc => Path.Combine(Metadata("OcctSourceDir"), "data", "occ");

    private static string TclPath(string thePath) => thePath.Replace('\\', '/');

    private string NewFile() => Path.Combine(myDirectory, $"m{myFileNumber++}.brep");

    private static string Text(string theFile) => File.ReadAllText(theFile, Encoding.Latin1);

    private void Disagree(string theWhat)
    {
        if (myDisagreements.Count < 50)
        {
            output.WriteLine("DISAGREE " + theWhat);
        }

        myDisagreements.Add(theWhat);
    }

    private void Same<T>(T theNative, T thePort, string theWhat)
    {
        myValues++;
        if (!EqualityComparer<T>.Default.Equals(theNative, thePort))
        {
            Disagree($"{theWhat}: native {theNative}, port {thePort}");
        }
    }

    /// <summary>The text with the Checked flag (the third of a TShape's seven flags) of every shape blanked.</summary>
    private static string WithoutChecked(string theText)
        => string.Join('\n', theText.Split('\n').Select(l => l.Length == 7 && l.All(c => c is '0' or '1') ? l[..2] + "x" + l[3..] : l));

    private static N.TopoDS_Shape NativeRead(string theFile)
    {
        var aShape = new N.TopoDS_Shape();
        using var aBuilder = new N.BRep_Builder();
        Assert.True(N.BRepTools.Read(aShape, theFile, aBuilder), theFile);
        return aShape;
    }

    private string NativeWrite(N.TopoDS_Shape theShape, bool theWithTriangles = true)
    {
        string aFile = NewFile();
        Assert.True(N.BRepTools.Write(theShape, aFile, theWithTriangles, false, (N.TopTools_FormatVersion)(int)TopTools_FormatVersion_CURRENT));
        return aFile;
    }

    private string PortWrite(in TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(BRepTools.Write(theShape, aFile, true, false, TopTools_FormatVersion_CURRENT));
        return aFile;
    }

    /// <summary>The largest extent of the shape's box (BRepBndLib without triangulations), natively.</summary>
    private static double NativeSize(N.TopoDS_Shape theShape, out bool theIsVoid)
    {
        using var aBox = new N.Bnd_Box();
        N.BRepBndLib.Add(theShape, aBox, false);
        theIsVoid = aBox.IsVoid();
        if (theIsVoid)
        {
            return 0.0;
        }

        double aXMin = 0, aYMin = 0, aZMin = 0, aXMax = 0, aYMax = 0, aZMax = 0;
        aBox.Get(ref aXMin, ref aYMin, ref aZMin, ref aXMax, ref aYMax, ref aZMax);
        return Math.Max(Math.Max(aXMax - aXMin, aYMax - aYMin), aZMax - aZMin);
    }

    private static readonly string[] s_statusNames =
        ["OpenWire", "SelfIntersectingWire", "Failure", "ReMesh", "UnorientedWire", "TooFewPoints", "Outdated", "Reused", "UserBreak"];

    /// <summary>MeshTest.cxx's statuses line for native flags.</summary>
    private static string Statuses(int theFlags)
    {
        if (theFlags == 0)
        {
            return "Meshing statuses: NoError";
        }

        var aText = new StringBuilder("Meshing statuses: ");
        for (int i = 0; i < 9; i++)
        {
            if ((theFlags & (1 << i)) != 0)
            {
                aText.Append(s_statusNames[i]).Append(' ');
            }
        }

        return aText.ToString();
    }

    [GeneratedRegex(@"This shape contains (\d+) faces\.\n(?:\s+(\d+) empty faces\.\n)?\s+(\d+) triangles\.\n\s+(\d+) nodes\.\n\s+(\d+) polygons on triangulation\.\nMaximal deflection (\S+)\n")]
    private static partial Regex TrInfo();

    /// <summary>One run: Draw-lite's script on F1 against native meshing of F1 with the arguments.</summary>
    private void Check(string theF1, string theWhat, string theScript, double theDeflection, bool theIsRelative, double theAngle,
                       bool theInParallel)
    {
        myRuns++;
        var aDI = new DrawInterpreter(new StringWriter { NewLine = "\n" });
        aDI.LoadDrawResources(ResourcesDirectory);
        aDI.Eval($"restore {TclPath(theF1)} s; explode s f");
        string aResult;
        try
        {
            aResult = aDI.Eval(theScript);
        }
        catch (Exception anError)
        {
            Disagree($"{theWhat}: the port fails: {anError.Message}");
            return;
        }

        using N.TopoDS_Shape aNative = NativeRead(theF1);
        int aNativeFlags;
        using (var aMesher = new N.BRepMesh_IncrementalMesh(aNative, theDeflection, theIsRelative, theAngle, theInParallel))
        {
            aNativeFlags = aMesher.GetStatusFlags();
        }

        if (theScript.StartsWith("incmesh", StringComparison.Ordinal))
        {
            Same($"Incremental Mesh, multi-threading {(theInParallel ? "ON" : "OFF")}\n{Statuses(aNativeFlags)}", aResult, $"{theWhat} result");
        }

        // the whole shape
        string aNativeText = Text(NativeWrite(aNative));
        string aPortText = Text(PortWrite(aDI.GetShape("s")));
        if (aNativeText == aPortText)
        {
            myTextsSame++;
        }
        else if (WithoutChecked(aNativeText) == WithoutChecked(aPortText))
        {
            myTextsSameButChecked++;
        }
        else
        {
            Disagree($"{theWhat}: BRep text differs");
        }

        // the first face, exploded before meshing: the native face is the same TShape, meshed in place
        using (var anExp = new N.TopExp_Explorer(aNative, N.TopAbs_ShapeEnum.TopAbs_FACE))
        {
            // (explode skips the shape itself: a face has no s_1)
            if (anExp.More() && aNative.ShapeType() != N.TopAbs_ShapeEnum.TopAbs_FACE)
            {
                mySubShapes++;
                using N.TopoDS_Shape aNativeFace = anExp.Current();
                string aNativeFaceText = WithoutChecked(Text(NativeWrite(aNativeFace)));
                string aPortFaceText = WithoutChecked(Text(PortWrite(aDI.GetShape("s_1"))));
                if (aNativeFaceText != aPortFaceText)
                {
                    string[] aN = aNativeFaceText.Split('\n'), aP = aPortFaceText.Split('\n');
                    int k = 0;
                    while (k < aN.Length && k < aP.Length && aN[k] == aP[k])
                    {
                        k++;
                    }

                    Disagree($"{theWhat} s_1 text, line {k}: native '{(k < aN.Length ? aN[k] : "")}', port '{(k < aP.Length ? aP[k] : "")}'");
                }
            }
        }

        // trinfo
        int aNbFaces = 0, aNbEmpty = 0, aNbTriangles = 0, aNbNodes = 0;
        double aMaxDefl = 0.0;
        using (var anExp = new N.TopExp_Explorer(aNative, N.TopAbs_ShapeEnum.TopAbs_FACE))
        {
            for (; anExp.More(); anExp.Next())
            {
                aNbFaces++;
                using N.TopoDS_Shape aCurrent = anExp.Current();
                using N.TopoDS_Face aFace = N.TopoDS.Face(aCurrent);
                using var aLoc = new N.TopLoc_Location();
                using N.Poly_Triangulation? aT = N.BRep_Tool.Triangulation(aFace, aLoc);
                if (aT is null)
                {
                    aNbEmpty++;
                    continue;
                }

                aNbTriangles += aT.NbTriangles();
                aNbNodes += aT.NbNodes();
                aMaxDefl = Math.Max(aMaxDefl, aT.Deflection());
            }
        }

        Match aMatch = TrInfo().Match(aDI.Eval("trinfo s"));
        Assert.True(aMatch.Success, theWhat);
        Same(aNbFaces, int.Parse(aMatch.Groups[1].Value, CultureInfo.InvariantCulture), $"{theWhat} trinfo faces");
        Same(aNbEmpty, aMatch.Groups[2].Success ? int.Parse(aMatch.Groups[2].Value, CultureInfo.InvariantCulture) : 0, $"{theWhat} trinfo empty");
        Same(aNbTriangles, int.Parse(aMatch.Groups[3].Value, CultureInfo.InvariantCulture), $"{theWhat} trinfo triangles");
        Same(aNbNodes, int.Parse(aMatch.Groups[4].Value, CultureInfo.InvariantCulture), $"{theWhat} trinfo nodes");
        Same(BitConverter.DoubleToInt64Bits(aMaxDefl),
             BitConverter.DoubleToInt64Bits(double.Parse(aMatch.Groups[6].Value, NumberStyles.Float, CultureInfo.InvariantCulture)),
             $"{theWhat} trinfo deflection");
    }

    /// <summary>A deflection with three significant digits, which Draw's text and native's double agree on.</summary>
    private static double Round3(double theValue)
        => double.Parse(theValue.ToString("G3", CultureInfo.InvariantCulture), NumberStyles.Float, CultureInfo.InvariantCulture);

    private static string R(double theValue) => theValue.ToString("R", CultureInfo.InvariantCulture);

    /// <summary>The data files of OCCT's mesh group (tests/mesh/data) that the public dataset has, .brep and .rle.</summary>
    private static IEnumerable<string> MeshGroupFiles()
    {
        string aDataset = Metadata("OcctTestDataDir");
        if (!Directory.Exists(aDataset))
        {
            yield break;
        }

        var anIndex = new Dictionary<string, string>(StringComparer.Ordinal);
        foreach (string aFile in Directory.EnumerateFiles(aDataset, "*", SearchOption.AllDirectories).Order(StringComparer.Ordinal))
        {
            anIndex.TryAdd(Path.GetFileName(aFile), aFile);
        }

        var aNames = new SortedSet<string>(StringComparer.Ordinal);
        foreach (string aScript in Directory.EnumerateFiles(Path.Combine(Metadata("OcctSourceDir"), "tests", "mesh", "data"), "*", SearchOption.AllDirectories))
        {
            foreach (string aLine in File.ReadLines(aScript))
            {
                string[] aWords = aLine.Split((char[]?)null, StringSplitOptions.RemoveEmptyEntries);
                if (aWords.Length >= 3 && aWords[0] == "set" && aWords[1] == "TheFileName"
                    && (aWords[2].EndsWith(".brep", StringComparison.Ordinal) || aWords[2].EndsWith(".rle", StringComparison.Ordinal)))
                {
                    aNames.Add(aWords[2]);
                }
            }
        }

        foreach (string aName in aNames)
        {
            if (anIndex.TryGetValue(aName, out string? aPath))
            {
                yield return aPath;
            }
        }
    }

    [Fact]
    public void Incmesh_and_vdisplay_mesh_as_native_OCCT()
    {
        Assert.True(Directory.Exists(DataOcc), $"No OCCT source data at {DataOcc}");
        List<string> aFiles = Directory.GetFiles(DataOcc, "*.brep").Order(StringComparer.Ordinal)
                                       .Where(f => new FileInfo(f).Length <= 1_000_000).ToList();
        int aDataOcc = aFiles.Count;
        aFiles.AddRange(MeshGroupFiles().Where((_, k) => k % 2 == 0));
        output.WriteLine($"{aDataOcc} data/occ files, {aFiles.Count - aDataOcc} mesh-group files");

        foreach (string aFile in aFiles)
        {
            string aName = Path.GetFileName(aFile);
            string aF1;
            double aSize;
            bool isVoid;
            N.TopoDS_Shape? aRead = null;
            try
            {
                aRead = NativeRead(aFile);
                N.BRepTools.Clean(aRead);
                aF1 = NativeWrite(aRead, false);
                aSize = NativeSize(aRead, out isVoid);
            }
            catch (Exception anError)
            {
                output.WriteLine($"{aName}: native can't read or write it ({anError.Message})");
                continue;
            }
            finally
            {
                aRead?.Dispose();
            }

            if (isVoid || aSize <= 0.0)
            {
                continue;
            }

            // incmesh with the options the native constructor expresses
            double aD1 = Round3(0.004 * aSize), aD3 = Round3(0.01 * aSize);
            Check(aF1, $"{aName} incmesh {R(aD1)}", $"incmesh s {R(aD1)}", aD1, false, 0.5, false);
            Check(aF1, $"{aName} incmesh 0.05 -relative -a 20", "incmesh s 0.05 -relative -a 20", 0.05, true, 20 * Math.PI / 180, false);
            Check(aF1, $"{aName} incmesh {R(aD3)} -a 30 -parallel", $"incmesh s {R(aD3)} -a 30 -parallel", aD3, false, 30 * Math.PI / 180, true);

            // vdisplay: the selection meshes the shape with AIS's defaults (Prs3d::GetDeflection on the native box);
            // vsetdispmode 1 then finds it tessellated finely enough
            double aPrsDeflection = Math.Max(aSize * 0.001 * 4.0, Precision.Confusion());
            Check(aF1, $"{aName} vdisplay", "vinit; vdisplay s; vsetdispmode 1", aPrsDeflection, false, 20 * Math.PI / 180, false);
        }

        output.WriteLine($"{myRuns} runs: {myTextsSame} BRep texts byte-identical, {myTextsSameButChecked} but for the Checked flags "
                         + $"(decision 551), {myRuns - myTextsSame - myTextsSameButChecked} different; {mySubShapes} exploded faces; {myValues} other values compared, "
                         + $"{myDisagreements.Count} disagreements");
        Assert.Empty(myDisagreements);
    }
}
#endif
