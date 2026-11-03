// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using System.Diagnostics;
using System.Reflection;
using System.Text.RegularExpressions;
using OcctNet.TestHarness;
using Xunit.Abstractions;
using static Occt.ShapeExtend_Status;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// The shared part of the ShapeFix oracles of unit 6H3: inputs written by native OCCT and read by both sides (the
/// port's frozen, as a caller's), results compared as BRep text with the Free column masked (open question 1), then
/// canonically (decision 581), the port's input checked unchanged, and the share of the result's faces and edges that
/// are the caller's own counted.
/// </summary>
internal sealed class ShapeFixOracleRun(ITestOutputHelper output, string thePrefix) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory(thePrefix).FullName;
    private int myFileNumber;
    public readonly List<string> Differences = [];
    public int Compared, Identical, FreeMasked, CheckedMasked, Canonical, Exceptions, Unchanged, NativeChanged, KeptFaces, ResultFaces,
               KeptEdges, ResultEdges;

    public void Dispose() => Directory.Delete(myDirectory, true);

    public static string Metadata(string theKey)
        => typeof(ShapeFixOracleRun).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    private string NewFile() => Path.Combine(myDirectory, $"s{myFileNumber++}.brep");

    public string NativeText(N.TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(N.BRepTools.Write(theShape, aFile, true, false, N.TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3));
        string aText = File.ReadAllText(aFile);
        File.Delete(aFile);
        return aText;
    }

    public string PortText(in TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(BRepTools.Write(theShape, aFile, true, false, TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3));
        string aText = File.ReadAllText(aFile);
        File.Delete(aFile);
        return aText;
    }

    /// <summary>Native writes the shape; both read it, the port's frozen. Null if native OCCT can't write or read it.</summary>
    public (N.TopoDS_Shape N, TopoDS_Shape P)? Read(N.TopoDS_Shape theShape)
    {
        string aFile = NewFile();
        if (!N.BRepTools.Write(theShape, aFile, true, false, N.TopTools_FormatVersion.TopTools_FormatVersion_VERSION_3))
        {
            return null;
        }
        var aNShape = new N.TopoDS_Shape();
        using (var aBuilder = new N.BRep_Builder())
        {
            if (!N.BRepTools.Read(aNShape, aFile, aBuilder))
            {
                return null;
            }
        }
        Assert.True(BRepTools.Read(out TopoDS_Shape aPShape, aFile), aFile);
        File.Delete(aFile);
        if (aPShape.IsNull())
        {
            return null;
        }
        aPShape.TShape()!.FreezeGraph(); // a caller's shape
        return (aNShape, aPShape);
    }

    public static string? ErrorOf(Exception theException) => theException switch
    {
        N.OcctException e => e.Message.IndexOf(':') is int c and > 0 ? e.Message[..c] : e.Message,
        FailureException f => global::Kerf.Engine.Foundation.OcctTypeNames.Of(f.GetType()),
        _ => null,
    };

    /// <summary>
    /// Compares one fix: the results (null if the side threw), the exceptions, the statuses (as text), and the port's
    /// input (its text before the fix given).
    /// </summary>
    public void Check(string theWhat, N.TopoDS_Shape? theNative, string? theNativeError, string theNativeStatus,
                      TopoDS_Shape? thePort, string? thePortError, string thePortStatus,
                      N.TopoDS_Shape theNativeInput, TopoDS_Shape thePortInput, string thePortInputText)
    {
        Compared++;
        if (PortText(thePortInput) == thePortInputText)
        {
            Unchanged++;
        }
        else
        {
            Differences.Add($"{theWhat}: the input changed");
        }
        if (theNativeError is not null || thePortError is not null)
        {
            if (theNativeError == thePortError)
            {
                Identical++;
                Exceptions++;
            }
            else
            {
                Differences.Add($"{theWhat}: {theNativeError ?? "no exception"} natively, {thePortError ?? "no exception"} in the port");
            }
            return;
        }
        if (theNativeStatus != thePortStatus)
        {
            Differences.Add($"{theWhat}: status {theNativeStatus} natively, {thePortStatus} in the port");
        }
        if (theNative is null || theNative.IsNull() || thePort is not TopoDS_Shape aPRes || aPRes.IsNull())
        {
            if ((theNative is null || theNative.IsNull()) != (thePort is null || thePort.Value.IsNull()))
            {
                Differences.Add($"{theWhat}: a null result on one side only");
            }
            else
            {
                Identical++;
            }
            return;
        }
        if (!aPRes.TShape()!.IsFrozen)
        {
            Differences.Add($"{theWhat}: the port's result isn't frozen");
        }
        string aNT = NativeText(theNative), aPT = PortText(aPRes);
        if (aNT != NativeText(theNativeInput))
        {
            NativeChanged++;
        }
        CountKept(thePortInput, aPRes);
        BRepTextDiff aDiff = BRepTextComparison.Compare(aNT, aPT);
        if (aDiff.IsIdentical)
        {
            Identical++;
        }
        else if (aDiff.IsIdenticalMaskingFree)
        {
            Identical++;
            FreeMasked++;
        }
        else if (ShapeFixFaceOracleTests.MaskFreeAndChecked(aNT) == ShapeFixFaceOracleTests.MaskFreeAndChecked(aPT))
        {
            // native BRepTools::Update set the Checked bit of the caller's face in place (decision 738)
            Identical++;
            CheckedMasked++;
        }
        else
        {
            string aFile = NewFile();
            File.WriteAllText(aFile, aNT);
            Assert.True(BRepTools.Read(out TopoDS_Shape aNP, aFile));
            File.Delete(aFile);
            CanonicalComparison aCanon = CanonicalShapeComparer.Compare(aNP, aPRes, new CanonicalOptions { CheckValidity = false });
            if (aCanon.IsIdentical)
            {
                Canonical++;
            }
            else
            {
                Differences.Add($"{theWhat}: {aDiff}; canonically {aCanon}");
            }
        }
    }

    private void CountKept(in TopoDS_Shape theInput, in TopoDS_Shape theResult)
    {
        foreach ((TopAbs_ShapeEnum aType, bool isFace) in new[] { (TopAbs_ShapeEnum.TopAbs_FACE, true), (TopAbs_ShapeEnum.TopAbs_EDGE, false) })
        {
            var anInput = new HashSet<TopoDS_TShape>(ReferenceEqualityComparer.Instance);
            foreach (TopoDS_Shape aS in CanonicalShapeComparer.SubShapes(theInput, aType))
            {
                anInput.Add(aS.TShape()!);
            }
            foreach (TopoDS_Shape aS in CanonicalShapeComparer.SubShapes(theResult, aType))
            {
                bool isKept = anInput.Contains(aS.TShape()!);
                if (isFace)
                {
                    ResultFaces++;
                    KeptFaces += isKept ? 1 : 0;
                }
                else
                {
                    ResultEdges++;
                    KeptEdges += isKept ? 1 : 0;
                }
            }
        }
    }

    public void Report(string theWhat, TimeSpan theTime)
    {
        output.WriteLine($"{theWhat}: {Identical}/{Compared} results identical as BRep text ({FreeMasked} with the Free column masked, {CheckedMasked} with the Free and Checked columns masked, "
                         + $"{Exceptions} the same exception on both sides), {Canonical} canonically identical, {Differences.Count} different; "
                         + $"inputs unchanged {Unchanged}; native results differing from their input {NativeChanged}; "
                         + $"the caller's faces kept {KeptFaces}/{ResultFaces}, edges {KeptEdges}/{ResultEdges}; {theTime.TotalSeconds:F0} s");
        foreach (string aD in Differences.Take(60))
        {
            output.WriteLine("  " + aD);
        }
    }

    public static N.TopoDS_Shape? NativeRead(string theFile)
    {
        var aShape = new N.TopoDS_Shape();
        using var aBuilder = new N.BRep_Builder();
        try
        {
            return N.BRepTools.Read(aShape, theFile, aBuilder) && !aShape.IsNull() ? aShape : null;
        }
        catch (Exception)
        {
            return null;
        }
    }

    /// <summary>
    /// The data files of the heal group's cases (.brep and .rle up to <paramref name="theMaxSize"/> bytes) and data/occ's,
    /// by name; with the grids that use each heal file.
    /// </summary>
    public List<(string File, string Grids)> Inputs(long theMaxSize)
    {
        var aFiles = new List<(string, string)>();
        foreach (string aFile in Directory.GetFiles(Path.Combine(Metadata("OcctSourceDir"), "data", "occ"), "*.brep").Order(StringComparer.Ordinal))
        {
            if (new FileInfo(aFile).Length <= theMaxSize)
            {
                aFiles.Add((aFile, "data/occ"));
            }
        }
        string aRoot = Metadata("OcctTestDataDir");
        if (string.IsNullOrEmpty(aRoot) || !Directory.Exists(aRoot))
        {
            output.WriteLine("OcctTestDataDir isn't set: data/occ only");
            return aFiles;
        }
        var aNames = new SortedDictionary<string, SortedSet<string>>(StringComparer.Ordinal);
        var aLocate = new Regex(@"locate_data_file\s+([^\]\s]+)");
        string aHeal = Path.Combine(Metadata("OcctSourceDir"), "tests", "heal");
        foreach (string aCase in Directory.EnumerateFiles(aHeal, "*", SearchOption.AllDirectories))
        {
            string aGrid = Path.GetFileName(Path.GetDirectoryName(aCase)!);
            foreach (Match aMatch in aLocate.Matches(File.ReadAllText(aCase)))
            {
                if (!aNames.TryGetValue(aMatch.Groups[1].Value, out SortedSet<string>? aGrids))
                {
                    aNames[aMatch.Groups[1].Value] = aGrids = new SortedSet<string>(StringComparer.Ordinal);
                }
                aGrids.Add(aGrid);
            }
        }
        var aByName = new Dictionary<string, string>(StringComparer.Ordinal);
        foreach (string aFile in Directory.EnumerateFiles(aRoot, "*", SearchOption.AllDirectories))
        {
            aByName.TryAdd(Path.GetFileName(aFile), aFile);
        }
        foreach ((string aName, SortedSet<string> aGrids) in aNames)
        {
            if (aByName.TryGetValue(aName, out string? aFile)
                && (aName.EndsWith(".brep", StringComparison.OrdinalIgnoreCase) || aName.EndsWith(".rle", StringComparison.OrdinalIgnoreCase))
                && new FileInfo(aFile).Length <= theMaxSize)
            {
                aFiles.Add((aFile, string.Join(",", aGrids)));
            }
        }
        return aFiles;
    }

    // Up to theMax sub-shapes of the type, evenly sampled, distinct by TShape (as an indexed map gives them).
    public static List<N.TopoDS_Shape> Sample(N.TopoDS_Shape theShape, N.TopAbs_ShapeEnum theType, int theMax)
    {
        using var aMap = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(theShape, theType, aMap);
        int aNb = aMap.Extent();
        var aList = new List<N.TopoDS_Shape>();
        int aStep = Math.Max(1, aNb / Math.Max(1, theMax));
        for (int i = 1; i <= aNb && aList.Count < theMax; i += aStep)
        {
            aList.Add(aMap.FindKey(i));
        }
        return aList;
    }
}

/// <summary>
/// ShapeFix_Shape on whole shapes and their faces, shells and solids, and ShapeFix_Solid's SolidFromShell, against
/// native OCCT: the heal group's public inputs and data/occ, at the default settings and as the heal group's
/// <c>fix_shape</c> grid runs <c>fixshape</c> (precision 10^-3, maximal tolerance 5 10^-3); and faces as build123d.NET's
/// face making builds them (planar faces from random sketch wires, then ShapeFix_Shape).
/// </summary>
/// <remarks>
/// The statuses compared are Perform's result and DONE1 to DONE4 and FAIL (ShapeFix_Shape), DONE2 (SolidFromShell).
/// ShapeFix_Shape on faces, shells and solids reaches ShapeFix_Face (unit 6H3's other part). Histories aren't bound
/// natively (no <c>Context()</c>): the port's are tested by the unit tests.
/// </remarks>
public class ShapeFixShapeOracleTests(ITestOutputHelper output)
{
    private static string Status(bool theDone, Func<ShapeExtend_Status, bool> theStatus)
        => $"{theDone}/{theStatus(ShapeExtend_DONE1)}/{theStatus(ShapeExtend_DONE2)}/{theStatus(ShapeExtend_DONE3)}/{theStatus(ShapeExtend_DONE4)}/{theStatus(ShapeExtend_FAIL)}";

    private static N.ShapeExtend_Status NS(ShapeExtend_Status theStatus) => (N.ShapeExtend_Status)(int)theStatus;

    private static void CompareFixShape(ShapeFixOracleRun theRun, N.TopoDS_Shape theShape, string theWhat, double thePrecision = -1.0,
                                        double theMaxTol = -1.0)
    {
        if (theRun.Read(theShape) is not (N.TopoDS_Shape aNS, TopoDS_Shape aPS))
        {
            return;
        }
        string anInputText = theRun.PortText(aPS);

        N.TopoDS_Shape? aNRes = null;
        string? aNErr = null;
        string aNStatus = "";
        try
        {
            using var aFix = new N.ShapeFix_Shape(aNS);
            if (thePrecision > 0.0)
            {
                aFix.SetPrecision(thePrecision);
                aFix.SetMaxTolerance(theMaxTol);
            }
            bool isDone = aFix.Perform();
            aNRes = aFix.Shape();
            aNStatus = Status(isDone, s => aFix.Status(NS(s)));
        }
        catch (N.OcctException e)
        {
            aNErr = ShapeFixOracleRun.ErrorOf(e);
        }

        TopoDS_Shape? aPRes = null;
        string? aPErr = null;
        string aPStatus = "";
        try
        {
            var aFix = new ShapeFix_Shape(aPS);
            if (thePrecision > 0.0)
            {
                aFix.SetPrecision(thePrecision);
                aFix.SetMaxTolerance(theMaxTol);
            }
            bool isDone = aFix.Perform();
            aPRes = aFix.Shape();
            aPStatus = Status(isDone, aFix.Status);
        }
        catch (FailureException e)
        {
            aPErr = ShapeFixOracleRun.ErrorOf(e);
        }
        theRun.Check(theWhat, aNRes, aNErr, aNStatus, aPRes, aPErr, aPStatus, aNS, aPS, anInputText);
    }

    private static void CompareSolidFromShell(ShapeFixOracleRun theRun, N.TopoDS_Shape theShell, string theWhat)
    {
        if (theRun.Read(theShell) is not (N.TopoDS_Shape aNS, TopoDS_Shape aPS))
        {
            return;
        }
        string anInputText = theRun.PortText(aPS);

        N.TopoDS_Shape? aNRes = null;
        string? aNErr = null, aPErr = null;
        string aNStatus = "", aPStatus = "";
        try
        {
            using var aFix = new N.ShapeFix_Solid();
            aNRes = aFix.SolidFromShell(N.TopoDS.Shell(aNS));
            aNStatus = $"{aFix.Status(N.ShapeExtend_Status.ShapeExtend_DONE2)}";
        }
        catch (N.OcctException e)
        {
            aNErr = ShapeFixOracleRun.ErrorOf(e);
        }
        TopoDS_Shape? aPRes = null;
        try
        {
            var aFix = new ShapeFix_Solid();
            aPRes = aFix.SolidFromShell(TopoDS.Shell(aPS));
            aPStatus = $"{aFix.Status(ShapeExtend_DONE2)}";
        }
        catch (FailureException e)
        {
            aPErr = ShapeFixOracleRun.ErrorOf(e);
        }
        theRun.Check(theWhat, aNRes, aNErr, aNStatus, aPRes, aPErr, aPStatus, aNS, aPS, anInputText);
    }

    [Fact]
    public void ShapeFix_Solid_SolidFromShell_on_the_shells_of_the_heal_group_and_data_occ_matches_native_OCCT()
    {
        var aWatch = Stopwatch.StartNew();
        using var aRun = new ShapeFixOracleRun(output, "occtnet-sfsol-");
        int aShapes = 0;
        foreach ((string aFile, _) in Tape.Budget(aRun.Inputs(1_000_000), isSpread: true))
        {
            if (ShapeFixOracleRun.NativeRead(aFile) is not N.TopoDS_Shape aShape)
            {
                continue;
            }
            aShapes++;
            int i = 0;
            foreach (N.TopoDS_Shape aShell in ShapeFixOracleRun.Sample(aShape, N.TopAbs_ShapeEnum.TopAbs_SHELL, 6))
            {
                CompareSolidFromShell(aRun, aShell, $"{Path.GetFileName(aFile)}, shell {i}");
                CompareSolidFromShell(aRun, aShell.Reversed(), $"{Path.GetFileName(aFile)}, shell {i}, reversed");
                i++;
            }
        }
        output.WriteLine($"{aShapes} shapes");
        aRun.Report("ShapeFix_Solid::SolidFromShell on the shells of the heal group's and data/occ's shapes", aWatch.Elapsed);
        Assert.Empty(aRun.Differences);
    }

    [Fact]
    public void ShapeFix_Shape_on_the_heal_group_and_data_occ_matches_native_OCCT()
    {
        var aWatch = Stopwatch.StartNew();
        using var aRun = new ShapeFixOracleRun(output, "occtnet-sfs-");
        int aShapes = 0;
        foreach ((string aFile, string aGrids) in Tape.Budget(aRun.Inputs(1_000_000), isSpread: true))
        {
            if (ShapeFixOracleRun.NativeRead(aFile) is not N.TopoDS_Shape aShape)
            {
                continue;
            }
            aShapes++;
            string aName = Path.GetFileName(aFile);
            CompareFixShape(aRun, aShape, aName);
            if (aGrids.Contains("fix_shape", StringComparison.Ordinal) || aShapes % 4 == 0)
            {
                CompareFixShape(aRun, aShape, $"{aName}, fixshape 1e-3 5e-3", 1.0e-3, 5.0e-3);
            }
            foreach ((N.TopAbs_ShapeEnum aType, int aMax) in new[] { (N.TopAbs_ShapeEnum.TopAbs_SOLID, 2), (N.TopAbs_ShapeEnum.TopAbs_SHELL, 2),
                                                                      (N.TopAbs_ShapeEnum.TopAbs_FACE, 8) })
            {
                int i = 0;
                foreach (N.TopoDS_Shape aSub in ShapeFixOracleRun.Sample(aShape, aType, aMax))
                {
                    if (!aSub.IsSame(aShape))
                    {
                        CompareFixShape(aRun, aSub, $"{aName}, {aType} {i++}");
                    }
                }
            }
        }
        output.WriteLine($"{aShapes} shapes");
        aRun.Report("ShapeFix_Shape on the heal group's and data/occ's shapes and their solids, shells and faces", aWatch.Elapsed);
        Assert.Empty(aRun.Differences);
    }

    [Fact]
    public void ShapeFix_Shape_on_faces_made_from_random_sketch_wires_matches_native_OCCT()
    {
        // build123d.NET's MakeFaceFromWires: the wire fixed by ShapeFix_Shape, a planar face made on it, the face fixed;
        // here the face (made natively on the native fixed wire) goes through ShapeFix_Shape on both sides
        var aWatch = Stopwatch.StartNew();
        using var aRun = new ShapeFixOracleRun(output, "occtnet-sfsf-");
        var anOracle = new Oracle(output, 63301);
        foreach (int i in Tape.Budget(300))
        {
            N.TopoDS_Shape aWire = ShapeFixWireOracleTests.RandomWire(anOracle, i, out string aWhat);
            try
            {
                using var aFixWire = new N.ShapeFix_Shape(aWire);
                aFixWire.Perform();
                N.TopoDS_Shape aFixed = aFixWire.Shape();
                if (aFixed.ShapeType() != N.TopAbs_ShapeEnum.TopAbs_WIRE)
                {
                    continue;
                }
                using var aMakeFace = new N.BRepBuilderAPI_MakeFace(N.TopoDS.Wire(aFixed), true);
                if (!aMakeFace.IsDone())
                {
                    continue;
                }
                CompareFixShape(aRun, aMakeFace.Face(), $"face {i} ({aWhat})");
            }
            catch (N.OcctException)
            {
                // a wire native OCCT can't make a face from
            }
        }
        aRun.Report("ShapeFix_Shape on planar faces made from random sketch wires", aWatch.Elapsed);
        Assert.Empty(aRun.Differences);
    }
}
