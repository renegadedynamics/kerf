// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Diagnostics;
using System.Globalization;
using System.Reflection;
using System.Text.RegularExpressions;
using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;
using static Kerf.Engine.Exchange.Brep.FormatVersion;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// Draw-lite's checking commands (unit 3L2a) against native OCCT on the same files: checkshape's verdicts (with and
/// without -top, -parallel, -short and the structural dump) against BRepCheck_Analyzer::IsValid, bounding's values
/// (every option) against BRepBndLib, and bclassify's states against BRepClass3d_SolidClassifier.
/// </summary>
/// <remarks>
/// Native OCCT reads each file and writes it in the current format, which both sides then read (Draw-lite with
/// restore), as the BRepCheck oracle does. bounding prints with <c>%.17g</c>, so its values are compared exactly.
/// </remarks>
public partial class DrawLiteChecksOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-drawchecks-").FullName;
    private readonly List<string> myDisagreements = [];
    private int myFileNumber;
    private int myExceptions;

    public void Dispose()
    {
        Directory.Delete(myDirectory, true);
        GC.SuppressFinalize(this);
    }

    private static string Metadata(string theKey)
        => typeof(DrawLiteChecksOracleTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    private static string ResourcesDirectory => Path.Combine(Metadata("OcctSourceDir"), "resources", "DrawResources");

    private static string TclPath(string thePath) => thePath.Replace('\\', '/');

    [GeneratedRegex(@"[-+]?(?:inf|nan(?:\(ind\))?|\d+(?:\.\d*)?(?:e[-+]?\d+)?)")]
    private static partial Regex Number();

    private static List<double> Numbers(string theText)
        => Number().Matches(theText).Select(m => m.Value switch
        {
            "inf" or "+inf" => double.PositiveInfinity,
            "-inf" => double.NegativeInfinity,
            _ => double.Parse(m.Value, NumberStyles.Float, CultureInfo.InvariantCulture),
        }).ToList();

    private static string R(double theValue) => theValue.ToString("R", CultureInfo.InvariantCulture);

    /// <summary>Native OCCT reads the file and writes it in the current format: the file both sides use, or null.</summary>
    private (string File, N.TopoDS_Shape Shape)? NativeRead(string theFile)
    {
        var aShape = new N.TopoDS_Shape();
        using var aBuilder = new N.BRep_Builder();
        if (!N.BRepTools.Read(aShape, theFile, aBuilder) || aShape.IsNull())
        {
            aShape.Dispose();
            return null;
        }

        string aF1 = Path.Combine(myDirectory, $"s{myFileNumber++}.brep");
        Assert.True(N.BRepTools.Write(aShape, aF1, true, false, (N.TopTools_FormatVersion)(int)FormatVersion_CURRENT));
        aShape.Dispose();

        // both sides read the same file: the original file and its F1 copy can differ in the last bits
        var aF1Shape = new N.TopoDS_Shape();
        Assert.True(N.BRepTools.Read(aF1Shape, aF1, aBuilder), aF1);
        return (aF1, aF1Shape);
    }

    private static DrawInterpreter Interpreter()
    {
        var aDI = new DrawInterpreter(new StringWriter { NewLine = "\n" });
        aDI.LoadDrawResources(ResourcesDirectory);
        return aDI;
    }

    private void Disagree(string theWhat)
    {
        if (myDisagreements.Count < 50)
        {
            output.WriteLine("DISAGREE " + theWhat);
        }

        myDisagreements.Add(theWhat);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // checkshape

    private static string? NativeVerdict(N.TopoDS_Shape theShape, bool theGeom)
    {
        try
        {
            using var anAnalyzer = new N.BRepCheck_Analyzer(theShape, theGeom);
            return anAnalyzer.IsValid() ? "valid" : "invalid";
        }
        catch (Exception anException)
        {
            return "exception " + global::Kerf.Engine.Foundation.OcctTypeNames.Of(Refusal.Unwrapped(anException).GetType());
        }
    }

    /// <summary>
    /// checkshape in its forms against native IsValid: the contextual dump (serial and parallel give the same text),
    /// -short, and the structural dump. Returns the number of checks.
    /// </summary>
    private int CheckShape(DrawInterpreter theDI, N.TopoDS_Shape theNative, string theWhat, bool theAllForms)
    {
        int aChecks = 0;
        foreach (bool aGeom in theAllForms ? new[] { true, false } : [true])
        {
            string aTop = aGeom ? "" : "-top ";
            string? aNative = NativeVerdict(theNative, aGeom);
            string aSerial;
            try
            {
                aSerial = theDI.Eval($"checkshape {aTop}s");
            }
            catch (DrawException anError)
            {
                Disagree($"{theWhat} checkshape {aTop}: native {aNative}, port error {anError.Reason}");
                continue;
            }

            string aPort = aSerial == "This shape seems to be valid" ? "valid"
                         : aSerial.Contains("Faulty shapes in variables faulty_1 to faulty_", StringComparison.Ordinal) ? "invalid"
                         : "unexpected: " + aSerial;
            if (aNative != aPort)
            {
                Disagree($"{theWhat} checkshape {aTop}: native {aNative}, port {aPort}");
            }

            aChecks++;
            if (!theAllForms)
            {
                continue;
            }

            string aParallel = theDI.Eval($"checkshape {aTop}s -parallel");
            if (aParallel != aSerial)
            {
                Disagree($"{theWhat} checkshape {aTop}-parallel differs from serial");
            }

            string aShort = theDI.Eval($"checkshape {aTop}s -short");
            if (aShort != (aPort == "valid" ? "This shape seems to be valid" : "This shape has faulty shapes"))
            {
                Disagree($"{theWhat} checkshape {aTop}-short: {aShort}");
            }

            string aStructural = theDI.Eval($"checkshape {aTop}s r");
            bool aLooksOk = aStructural == " -- The Shape s looks OK";
            if (aLooksOk != (aPort == "valid") || (!aLooksOk && !aStructural.StartsWith(" -- The Shape s has problems :\n", StringComparison.Ordinal)))
            {
                Disagree($"{theWhat} checkshape {aTop}s r: {aStructural}");
            }

            aChecks += 3;
        }

        return aChecks;
    }

    // -----------------------------------------------------------------------------------------------------------------
    // bounding

    private static List<double> NativeBox(N.TopoDS_Shape theShape, string theOptions)
    {
        bool anOBB = theOptions.Contains("-obb", StringComparison.Ordinal);
        bool aTri = !theOptions.Contains("-noTriangulation", StringComparison.Ordinal);
        bool anOptimal = theOptions.Contains("-optimal", StringComparison.Ordinal);
        bool aTol = theOptions.Contains("-extToler", StringComparison.Ordinal);
        if (anOBB)
        {
            using var anOBBox = new N.Bnd_OBB();
            N.BRepBndLib.AddOBB(theShape, anOBBox, aTri, anOptimal, aTol);
            if (anOBBox.IsVoid())
            {
                return [];
            }

            using N.gp_XYZ aC = anOBBox.Center(), aX = anOBBox.XDirection(), aY = anOBBox.YDirection(), aZ = anOBBox.ZDirection();
            return [aC.X(), aC.Y(), aC.Z(), aX.X(), aX.Y(), aX.Z(), aY.X(), aY.Y(), aY.Z(), aZ.X(), aZ.Y(), aZ.Z(),
                    anOBBox.XHSize(), anOBBox.YHSize(), anOBBox.ZHSize()];
        }

        using var aBox = new N.Bnd_Box();
        if (anOptimal)
        {
            N.BRepBndLib.AddOptimal(theShape, aBox, aTri, aTol);
        }
        else
        {
            N.BRepBndLib.Add(theShape, aBox, aTri);
        }

        if (aBox.IsVoid())
        {
            return [];
        }

        using N.gp_Pnt aMin = aBox.CornerMin(), aMax = aBox.CornerMax();
        return [aMin.X(), aMin.Y(), aMin.Z(), aMax.X(), aMax.Y(), aMax.Z()];
    }

    private static readonly string[] theBoundingOptions =
    [
        "", "-noTriangulation", "-optimal", "-optimal -extToler", "-optimal -noTriangulation", "-obb", "-obb -optimal",
        "-obb -noTriangulation", "-obb -extToler",
    ];

    private int Bounding(Oracle theOracle, DrawInterpreter theDI, N.TopoDS_Shape theNative, string theWhat, IEnumerable<string> theOptions)
    {
        int aChecks = 0;
        foreach (string anOptions in theOptions)
        {
            string aWhat = $"{theWhat} bounding {anOptions}";
            List<double> aNative;
            string? aNativeError = null;
            try
            {
                aNative = NativeBox(theNative, anOptions);
            }
            catch (N.OcctException anException)
            {
                aNative = [];
                aNativeError = anException.Message;
            }

            string aText;
            try
            {
                aText = theDI.Eval($"bounding s {anOptions}");
            }
            catch (DrawException anError)
            {
                // an OCCT exception out of BRepBndLib: Draw reports "An exception was caught Type: message"
                string aType = aNativeError?.Split(':')[0] ?? "(none)";
                if (!anError.Reason.StartsWith("An exception was caught " + aType, StringComparison.Ordinal))
                {
                    Disagree($"{aWhat}: native {aNativeError ?? "a box"}, port error {anError.Reason}");
                }

                aChecks++;
                myExceptions++;
                continue;
            }

            if (aNativeError is not null)
            {
                Disagree($"{aWhat}: native {aNativeError}, port {aText}");
                continue;
            }

            if (aNative.Count == 0)
            {
                if (aText != "Void box.\n")
                {
                    Disagree($"{aWhat}: native void, port {aText}");
                }

                aChecks++;
                continue;
            }

            List<double> aPort = Numbers(aText);
            if (aPort.Count != aNative.Count)
            {
                Disagree($"{aWhat}: {aNative.Count} values natively, port {aText}");
                continue;
            }

            for (int i = 0; i < aNative.Count; i++)
            {
                double aScale = Math.Max(1.0, Math.Abs(aNative[i]));
                if (BitConverter.DoubleToInt64Bits(aNative[i]) != BitConverter.DoubleToInt64Bits(aPort[i]))
                {
                    output.WriteLine($"not bit-identical: {aWhat} [{i}]: native {R(aNative[i])}, port {R(aPort[i])}");
                }

                if (!(Math.Abs(aNative[i] - aPort[i]) <= Oracle.Tolerance * aScale) && !aNative[i].Equals(aPort[i]))
                {
                    Disagree($"{aWhat} [{i}]: native {R(aNative[i])}, port {R(aPort[i])}");
                }
                else
                {
                    theOracle.Same(aNative[i], aPort[i], $"{aWhat} [{i}]");
                }
            }

            aChecks++;
        }

        return aChecks;
    }

    // -----------------------------------------------------------------------------------------------------------------
    // bclassify

    private int BClassify(Oracle theOracle, DrawInterpreter theDI, N.TopoDS_Shape theNative, string theWhat, int theMaxSolids, int thePoints)
    {
        int aChecks = 0;
        // explode leaves out the shape itself
        string[] aSolids = theNative.ShapeType() == N.TopAbs_ShapeEnum.TopAbs_SOLID
                               ? ["s"]
                               : theDI.Eval("explode s so").Split(' ', StringSplitOptions.RemoveEmptyEntries);
        using var anExplorer = new N.TopExp_Explorer(theNative, N.TopAbs_ShapeEnum.TopAbs_SOLID);
        var aNativeSolids = new List<N.TopoDS_Shape>();
        using var aSeen = new N.TopTools_MapOfShape();
        for (; anExplorer.More(); anExplorer.Next())
        {
            if (aSeen.Add(anExplorer.Current()))
            {
                aNativeSolids.Add(anExplorer.Current());
            }
        }

        Assert.Equal(aNativeSolids.Count, aSolids.Length);
        for (int k = 0; k < Math.Min(theMaxSolids, aSolids.Length); k++)
        {
            List<double> aBox = Numbers(theDI.Eval($"bounding {aSolids[k]}"));
            double aDiag = Math.Sqrt(Math.Pow(aBox[3] - aBox[0], 2) + Math.Pow(aBox[4] - aBox[1], 2) + Math.Pow(aBox[5] - aBox[2], 2));
            for (int p = 0; p < thePoints; p++)
            {
                // points in the box enlarged by a tenth, and some on the box's planes
                double Coord(int i) => p % 5 == 4 ? (theOracle.Bool() ? aBox[i] : aBox[i + 3])
                                                  : theOracle.Real(aBox[i] - 0.1 * aDiag, aBox[i + 3] + 0.1 * aDiag);
                double x = Coord(0), y = Coord(1), z = Coord(2);
                double aTol = p % 3 == 0 ? 1e-7 : p % 3 == 1 ? 1e-3 : 1e-1;
                using var aNP = new N.gp_Pnt(x, y, z);
                string anExpected;
                try
                {
                    using var aNC = new N.BRepClass3d_SolidClassifier(aNativeSolids[k]);
                    aNC.Perform(aNP, aTol);
                    anExpected = aNC.State() switch
                    {
                        N.TopAbs_State.TopAbs_IN => "The point is IN shape\n",
                        N.TopAbs_State.TopAbs_OUT => "The point is OUT of shape\n",
                        N.TopAbs_State.TopAbs_ON => "The point is ON shape\n",
                        _ => "The point is UNKNOWN shape\n",
                    };
                }
                catch (N.OcctException anException)
                {
                    anExpected = "An exception was caught " + anException.Message.Split(':')[0];
                }

                string aPort;
                try
                {
                    aPort = theDI.Eval($"point p {R(x)} {R(y)} {R(z)}; bclassify {aSolids[k]} p {R(aTol)}");
                }
                catch (DrawException anError)
                {
                    aPort = anError.Reason.StartsWith(anExpected, StringComparison.Ordinal) ? anExpected : anError.Reason;
                }

                if (aPort != anExpected)
                {
                    Disagree($"{theWhat} bclassify {aSolids[k]} ({R(x)} {R(y)} {R(z)}) {R(aTol)}: native {anExpected.Trim()}, port {aPort.Trim()}");
                }

                aChecks++;
            }
        }

        return aChecks;
    }

    // -----------------------------------------------------------------------------------------------------------------
    // The files

    [Fact]
    public void Checks_on_the_data_occ_files_answer_as_native_OCCT()
    {
        string[] aFiles = Directory.GetFiles(Path.Combine(Metadata("OcctSourceDir"), "data", "occ"))
                                   .Where(f => f.EndsWith(".brep", StringComparison.Ordinal) || f.EndsWith(".rle", StringComparison.Ordinal))
                                   .Order(StringComparer.Ordinal).ToArray();
        Assert.Equal(42, aFiles.Length);
        var anOracle = new Oracle(output);
        int aCheckshape = 0, aBounding = 0, aClassify = 0, aValid = 0;
        foreach (string aFile in Tape.Budget(aFiles, isSpread: true))
        {
            string aName = Path.GetFileName(aFile);
            var aFileWatch = Stopwatch.StartNew();
            (string aF1, N.TopoDS_Shape aNS) = NativeRead(aFile)!.Value;
            using (aNS)
            {
                DrawInterpreter aDI = Interpreter();
                aDI.Eval($"restore {{{TclPath(aF1)}}} s");
                aCheckshape += CheckShape(aDI, aNS, aName, theAllForms: true);
                aValid += aDI.Eval("checkshape s") == "This shape seems to be valid" ? 1 : 0;
                aBounding += Bounding(anOracle, aDI, aNS, aName, theBoundingOptions);
                aClassify += BClassify(anOracle, aDI, aNS, aName, theMaxSolids: 4, thePoints: 40);
            }

            output.WriteLine($"{aName}: {aFileWatch.Elapsed.TotalSeconds:F1} s");
        }

        output.WriteLine($"data/occ: {aFiles.Length} files ({aValid} valid): {aCheckshape} checkshape verdicts, {aBounding} bounding boxes, "
                         + $"{aClassify} bclassify points, {myExceptions} boxes failing on both sides, {myDisagreements.Count} disagreements");
        anOracle.Report("bounding on data/occ");
        Assert.Empty(myDisagreements);
    }

    private const int MaxOccurrences = 200_000;

    private static int Occurrences(DrawInterpreter theDI)
    {
        Kerf.Topology.Shape aShape = theDI.GetShape("s");
        int aCount = 0;
        var aStack = new Stack<Kerf.Topology.Shape>();
        aStack.Push(aShape);
        while (aStack.Count > 0 && aCount <= MaxOccurrences)
        {
            Kerf.Topology.Shape aCurrent = aStack.Pop();
            aCount++;
            for (var anIt = new Kerf.Engine.Topology.Iterator(aCurrent); anIt.More(); anIt.Next())
            {
                aStack.Push(anIt.Value());
            }
        }

        return aCount;
    }

    /// <summary>
    /// OCCT's test dataset: checkshape (geometric controls on), bounding (plain, -optimal and -obb) and bclassify on a
    /// solid of each file, on every .brep and .rle native OCCT reads as a shape of at most 2 MB and
    /// <see cref="MaxOccurrences"/> sub-shape occurrences.
    /// </summary>
    [Fact]
    public void Checks_on_the_test_dataset_answer_as_native_OCCT()
    {
        string aRoot = Metadata("OcctTestDataDir");
        if (string.IsNullOrEmpty(aRoot) || !Directory.Exists(aRoot))
        {
            output.WriteLine("OcctTestDataDir isn't set: nothing to check");
            return;
        }

        string[] aFiles = Directory.EnumerateFiles(aRoot, "*.*", SearchOption.AllDirectories)
                                   .Where(f => f.EndsWith(".brep", StringComparison.OrdinalIgnoreCase) || f.EndsWith(".rle", StringComparison.OrdinalIgnoreCase))
                                   .Order(StringComparer.Ordinal).ToArray();
        var anOracle = new Oracle(output);
        var aWatch = Stopwatch.StartNew();
        int aChecked = 0, aSkipped = 0, aCheckshape = 0, aBounding = 0, aClassify = 0;
        foreach (string aFile in Tape.Budget(aFiles, isSpread: true))
        {
            if (new FileInfo(aFile).Length > 2_000_000 || NativeRead(aFile) is not { } aRead)
            {
                aSkipped++;
                continue;
            }

            using (aRead.Shape)
            {
                string aName = Path.GetRelativePath(aRoot, aFile);
                DrawInterpreter aDI = Interpreter();
                aDI.Eval($"restore {{{TclPath(aRead.File)}}} s");
                if (Occurrences(aDI) > MaxOccurrences)
                {
                    aSkipped++;
                    continue;
                }

                aCheckshape += CheckShape(aDI, aRead.Shape, aName, theAllForms: false);
                aBounding += Bounding(anOracle, aDI, aRead.Shape, aName, ["", "-optimal", "-obb"]);
                aClassify += BClassify(anOracle, aDI, aRead.Shape, aName, theMaxSolids: 1, thePoints: 5);
                aChecked++;
            }

            File.Delete(aRead.File);
        }

        output.WriteLine($"dataset: {aFiles.Length} files, {aChecked} checked, {aSkipped} left out (not shapes, over 2 MB or "
                         + $"{MaxOccurrences} occurrences); {aCheckshape} checkshape verdicts, {aBounding} bounding boxes, {aClassify} "
                         + $"bclassify points, {myExceptions} boxes failing on both sides, {myDisagreements.Count} disagreements, "
                         + $"{aWatch.Elapsed.TotalSeconds:F0} s");
        anOracle.Report("bounding on the dataset");
        Assert.Empty(myDisagreements);
    }
}
