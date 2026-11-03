// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Globalization;
using System.Reflection;
using System.Text;
using Xunit.Abstractions;
using static Kerf.Topology.ShapeType;
using static Kerf.Engine.Exchange.Brep.FormatVersion;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// Oracle milestone 1 (phase4-plan sections 3 and 5): the port's BRepMesh_IncrementalMesh against native OCCT's on the
/// same input, whole meshes.
/// </summary>
/// <remarks>
/// <para>
/// Decision 494's scheme. Native OCCT writes the shape without meshes (F1). Both sides read F1 and mesh it with the same
/// five arguments (deflection, relative, angle, parallel); native writes F2 with its triangulations, the port writes its
/// result. The two texts are compared byte for byte (shape, triangulations, polygons on triangulation with their
/// parameters, deflections, 15 significant digits), then each face's Poly_Triangulation node by node, UV node by UV
/// node and triangle by triangle, every double bit for bit, through the native binding.
/// </para>
/// <para>
/// A face is reported bit-identical; of the same topology (the same triangles, nodes within 1e-12 relative); or
/// different (with the node and triangle counts and the area difference). Each difference is printed for attribution.
/// </para>
/// </remarks>
public sealed class BRepMeshIncrementalMeshOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-brepmesh4d-").FullName;
    private readonly bool myKeepFiles = Environment.GetEnvironmentVariable("OCCTNET_KEEP_MESH_FILES") == "1";
    private int myFileNumber;

    public void Dispose()
    {
        if (myKeepFiles)
        {
            return;
        }
        try
        {
            Directory.Delete(myDirectory, true);
        }
        catch (IOException)
        {
        }
    }

    private string NewFile() => Path.Combine(myDirectory, $"m{myFileNumber++}.brep");

    private static string Metadata(string theKey)
        => typeof(BRepMeshIncrementalMeshOracleTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>()
                                                         .Single(a => a.Key == theKey).Value!;

    private static string DataOcc => Path.Combine(Metadata("OcctSourceDir"), "data", "occ");

    private static string Text(string theFile) => File.ReadAllText(theFile, Encoding.Latin1);

    private static Shape PortRead(string theFile)
    {
        Assert.True(BrepTools.Read(out Shape aShape, theFile), theFile);
        return aShape;
    }

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
        Assert.True(N.BRepTools.Write(theShape, aFile, theWithTriangles, false, (N.TopTools_FormatVersion)(int)FormatVersion_CURRENT));
        return aFile;
    }

    private string PortWrite(in Shape theShape)
    {
        string aFile = NewFile();
        Assert.True(BrepTools.Write(theShape, aFile, true, false, FormatVersion_CURRENT));
        return aFile;
    }

    /// <summary>F1: native OCCT's read of the file, cleaned of its meshes, written without triangulations.</summary>
    private string F1(string theFile)
    {
        N.TopoDS_Shape aNative = NativeRead(theFile);
        N.BRepTools.Clean(aNative);
        string aF1 = NativeWrite(aNative, false);
        aNative.Dispose();
        return aF1;
    }

    private static IndexedSet<Shape> Map(in Shape theShape, ShapeType theType)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap;
    }

    /// <summary>A setting of the five arguments; a deflection below zero is that fraction of the shape's size.</summary>
    private readonly record struct Setting(double Deflection, bool Relative, double Angle, bool Parallel)
    {
        public override string ToString()
            => string.Create(CultureInfo.InvariantCulture,
                             $"{(Deflection < 0 ? $"{-Deflection}*size" : Deflection.ToString("G3", CultureInfo.InvariantCulture))}"
                             + $"{(Relative ? " rel" : "")} {Angle}{(Parallel ? " par" : "")}");
    }

    /// <summary>A native face triangulation pulled out of the binding.</summary>
    private sealed class NativeTriangulation
    {
        public Xyz[] Nodes = [];
        public Xy[]? UVNodes;
        public int[] Triangles = [];
        public double Deflection;
    }

    private static NativeTriangulation? Pull(N.Poly_Triangulation? theT)
    {
        if (theT is null)
        {
            return null;
        }

        var aT = new NativeTriangulation
        {
            Nodes = new Xyz[theT.NbNodes()],
            UVNodes = theT.HasUVNodes() ? new Xy[theT.NbNodes()] : null,
            Triangles = new int[3 * theT.NbTriangles()],
            Deflection = theT.Deflection(),
        };
        for (int i = 1; i <= theT.NbNodes(); i++)
        {
            using (N.gp_Pnt aP = theT.Node(i))
            {
                aT.Nodes[i - 1] = new Xyz(aP.X(), aP.Y(), aP.Z());
            }
            if (aT.UVNodes is not null)
            {
                using N.gp_Pnt2d aUV = theT.UVNode(i);
                aT.UVNodes[i - 1] = new Xy(aUV.X(), aUV.Y());
            }
        }
        for (int i = 1; i <= theT.NbTriangles(); i++)
        {
            using N.Poly_Triangle aTri = theT.Triangle(i);
            int a = 0, b = 0, c = 0;
            aTri.Get(ref a, ref b, ref c);
            aT.Triangles[3 * i - 3] = a;
            aT.Triangles[3 * i - 2] = b;
            aT.Triangles[3 * i - 1] = c;
        }
        return aT;
    }

    /// <summary>The native side: reads F1, meshes it, writes F2; the faces' triangulations in TopExp::MapShapes order.</summary>
    private (string F2, NativeTriangulation?[] Tris, int Status) NativeMesh(string theF1, double theDeflection, Setting theSetting)
    {
        N.TopoDS_Shape aShape = NativeRead(theF1);
        int aStatus;
        using (var aMesh = new N.BRepMesh_IncrementalMesh(aShape, theDeflection, theSetting.Relative, theSetting.Angle, theSetting.Parallel))
        {
            aStatus = aMesh.GetStatusFlags();
        }
        string aF2 = NativeWrite(aShape);
        NativeTriangulation?[] aTris;
        using (var aMap = new N.TopTools_IndexedMapOfShape())
        {
            N.TopExp.MapShapes(aShape, N.TopAbs_ShapeEnum.TopAbs_FACE, aMap);
            N.TopoDS_Shape[] aFaces = aMap.ToArray();
            aTris = new NativeTriangulation?[aFaces.Length];
            for (int i = 0; i < aFaces.Length; i++)
            {
                using N.TopoDS_Face aFace = N.TopoDS.Face(aFaces[i]);
                using var aLoc = new N.TopLoc_Location();
                using N.Poly_Triangulation? aT = N.BRep_Tool.Triangulation(aFace, aLoc);
                aTris[i] = Pull(aT);
                aFaces[i].Dispose();
            }
        }
        aShape.Dispose();
        return (aF2, aTris, aStatus);
    }

    private sealed class Tally
    {
        public int Meshings, TextSame, TextSameButChecked, StatusSame, Faces, BitSame, SameTopology, Different, BothUnmeshed, OneUnmeshed,
                   BothRejected;
        public long Nodes, Triangles;
        public readonly List<string> Differences = [];
        public readonly List<string> TextDifferences = [];

        public void Add(Tally theOther)
        {
            Meshings += theOther.Meshings;
            TextSame += theOther.TextSame;
            TextSameButChecked += theOther.TextSameButChecked;
            BothRejected += theOther.BothRejected;
            StatusSame += theOther.StatusSame;
            Faces += theOther.Faces;
            BitSame += theOther.BitSame;
            SameTopology += theOther.SameTopology;
            Different += theOther.Different;
            BothUnmeshed += theOther.BothUnmeshed;
            OneUnmeshed += theOther.OneUnmeshed;
            Nodes += theOther.Nodes;
            Triangles += theOther.Triangles;
            Differences.AddRange(theOther.Differences);
            TextDifferences.AddRange(theOther.TextDifferences);
        }

        public double Share => Faces == 0 ? 1.0 : (double)BitSame / (Faces - BothUnmeshed);
    }

    private static double Area(Xyz[] theNodes, int[] theTriangles)
    {
        double anArea = 0.0;
        for (int i = 0; i < theTriangles.Length; i += 3)
        {
            Xyz a = theNodes[theTriangles[i] - 1], b = theNodes[theTriangles[i + 1] - 1], c = theNodes[theTriangles[i + 2] - 1];
            anArea += 0.5 * (b - a).Crossed(c - a).Modulus();
        }
        return anArea;
    }

    private static bool BitEqual(double a, double b) => BitConverter.DoubleToInt64Bits(a) == BitConverter.DoubleToInt64Bits(b);

    /// <summary>Compares the port's face triangulation with native's; returns the verdict for the tally.</summary>
    private static void CompareFace(Tally theTally, string theWhat, Triangulation? thePort, NativeTriangulation? theNative)
    {
        theTally.Faces++;
        if (thePort is null && theNative is null)
        {
            theTally.BothUnmeshed++;
            return;
        }
        if (thePort is null || theNative is null)
        {
            theTally.OneUnmeshed++;
            theTally.Different++;
            theTally.Differences.Add($"{theWhat}: meshed only by {(thePort is null ? "native" : "the port")}");
            return;
        }

        theTally.Nodes += theNative.Nodes.Length;
        theTally.Triangles += theNative.Triangles.Length / 3;
        var aPortNodes = new Xyz[thePort.NodeCount];
        for (int i = 1; i <= thePort.NodeCount; i++)
        {
            aPortNodes[i - 1] = thePort.Node(i).XYZ();
        }
        var aPortTris = new int[3 * thePort.TriangleCount];
        for (int i = 1; i <= thePort.TriangleCount; i++)
        {
            thePort.Triangle(i).Get(out aPortTris[3 * i - 3], out aPortTris[3 * i - 2], out aPortTris[3 * i - 1]);
        }

        if (aPortNodes.Length != theNative.Nodes.Length || aPortTris.Length != theNative.Triangles.Length
            || !aPortTris.AsSpan().SequenceEqual(theNative.Triangles))
        {
            theTally.Different++;
            double aNArea = Area(theNative.Nodes, theNative.Triangles), aPArea = Area(aPortNodes, aPortTris);
            theTally.Differences.Add(string.Create(CultureInfo.InvariantCulture,
                $"{theWhat}: {theNative.Nodes.Length}/{theNative.Triangles.Length / 3} nodes/triangles native, "
                + $"{aPortNodes.Length}/{aPortTris.Length / 3} port; area {aNArea:G17} native, relative difference {(aPArea - aNArea) / aNArea:E2}"));
            return;
        }

        bool isBitSame = BitEqual(thePort.Deflection(), theNative.Deflection) && thePort.HasUVNodes == (theNative.UVNodes is not null);
        double aMaxDiff = 0.0;
        for (int i = 0; i < aPortNodes.Length; i++)
        {
            Xyz p = aPortNodes[i], n = theNative.Nodes[i];
            if (!BitEqual(p.X(), n.X()) || !BitEqual(p.Y(), n.Y()) || !BitEqual(p.Z(), n.Z()))
            {
                isBitSame = false;
                double aScale = Math.Max(1.0, Math.Max(Math.Abs(n.X()), Math.Max(Math.Abs(n.Y()), Math.Abs(n.Z()))));
                aMaxDiff = Math.Max(aMaxDiff, (p - n).Modulus() / aScale);
            }
            if (theNative.UVNodes is not null && thePort.HasUVNodes)
            {
                Xy pu = thePort.UVNode(i + 1).XY(), nu = theNative.UVNodes[i];
                if (!BitEqual(pu.X(), nu.X()) || !BitEqual(pu.Y(), nu.Y()))
                {
                    isBitSame = false;
                    double aScale = Math.Max(1.0, Math.Max(Math.Abs(nu.X()), Math.Abs(nu.Y())));
                    aMaxDiff = Math.Max(aMaxDiff, (pu - nu).Modulus() / aScale);
                }
            }
        }

        if (isBitSame)
        {
            theTally.BitSame++;
        }
        else if (aMaxDiff <= 1e-12)
        {
            theTally.SameTopology++;
            theTally.Differences.Add(string.Create(CultureInfo.InvariantCulture,
                $"{theWhat}: same topology, nodes within {aMaxDiff:E2}, deflection {theNative.Deflection:G17} native, {thePort.Deflection():G17} port"));
        }
        else
        {
            theTally.Different++;
            theTally.Differences.Add(string.Create(CultureInfo.InvariantCulture,
                $"{theWhat}: same triangles, nodes differ by {aMaxDiff:E2}"));
        }
    }

    /// <summary>Meshes F1 on both sides with the setting and compares the results.</summary>
    private void Compare(Tally theTally, string theName, string theF1, Setting theSetting, in Shape? thePortInput = null)
    {
        Shape aPortF1 = thePortInput ?? PortRead(theF1);
        double aDeflection = theSetting.Deflection;
        if (aDeflection < 0)
        {
            var aBox = new Box();
            Bounds.Add(aPortF1, ref aBox, false);
            if (aBox.IsVoid)
            {
                return;
            }
            double aSize = 0.0;
            ShapeTool.BoxMaxDimension(aBox, ref aSize);
            aDeflection = -aDeflection * aSize;
        }

        string aWhat = $"{theName} ({theSetting})";
        if (aDeflection < global::Kerf.Engine.Foundation.Precision.Confusion())
        {
            // Both sides reject a deflection below the confusion tolerance (a tiny shape at a relative fraction).
            Assert.ThrowsAny<N.OcctException>(() => NativeMesh(theF1, aDeflection, theSetting));
            Assert.Throws<global::Kerf.Engine.Foundation.NumericErrorException>(() => new IncrementalMesh(aPortF1, aDeflection, theSetting.Relative, theSetting.Angle, theSetting.Parallel));
            theTally.BothRejected++;
            return;
        }

        (string aF2, NativeTriangulation?[] aNativeTris, int aNativeStatus) = NativeMesh(theF1, aDeflection, theSetting);

        var aMesh = new IncrementalMesh(aPortF1, aDeflection, theSetting.Relative, theSetting.Angle, theSetting.Parallel);
        Shape aResult = aMesh.Shape();
        string aP = PortWrite(aResult);

        theTally.Meshings++;
        if (aNativeStatus == aMesh.GetStatusFlags())
        {
            theTally.StatusSame++;
        }
        else
        {
            theTally.TextDifferences.Add($"{aWhat}: status {aNativeStatus} native, {aMesh.GetStatusFlags()} port");
        }
        string aNativeText = Text(aF2), aPortText = Text(aP);
        if (aNativeText == aPortText)
        {
            theTally.TextSame++;
        }
        else if (WithoutChecked(aNativeText) == WithoutChecked(aPortText))
        {
            // Native's BRepTools::Update sets the Checked flag of the faces the mesher doesn't write (kept or failed);
            // the port gives back the input's own face there (decision 551).
            theTally.TextSameButChecked++;
        }
        else
        {
            theTally.TextDifferences.Add($"{aWhat}: BRep text differs ({aF2}, {aP})");
        }

        IndexedSet<Shape> aFaces = Map(aResult, ShapeType.Face);
        Assert.Equal(aNativeTris.Length, aFaces.Count);
        for (int i = 1; i <= aFaces.Count; i++)
        {
            Triangulation? aT = Tool.Triangulation(ShapeCasts.Face(aFaces[i - 1]), out _);
            CompareFace(theTally, $"{aWhat} face {i}", aT, aNativeTris[i - 1]);
        }
    }

    /// <summary>The text with the Checked flag (the third of a TShape's seven flags) of every shape blanked.</summary>
    private static string WithoutChecked(string theText)
    {
        string[] aLines = theText.Split('\n');
        for (int i = 0; i < aLines.Length; i++)
        {
            string aLine = aLines[i].TrimEnd('\r');
            if (aLine.Length == 7 && aLine.All(c => c is '0' or '1'))
            {
                aLines[i] = aLine[..2] + "x" + aLine[3..];
            }
        }
        return string.Join('\n', aLines);
    }

    private void Report(Tally theTally, string theWhat, int theMaxLines = 60)
    {
        output.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"{theWhat}: {theTally.Meshings} meshings, BRep text byte-identical in {theTally.TextSame} "
            + $"(and in {theTally.TextSameButChecked} more but for the Checked flags of faces the mesher didn't write), "
            + $"status flags equal in {theTally.StatusSame}; {theTally.BothRejected} settings rejected by both; "
            + $"{theTally.Faces} faces ({theTally.Nodes} nodes, {theTally.Triangles} triangles): {theTally.BitSame} bit-identical "
            + $"({100.0 * theTally.Share:F3}% of meshed faces), {theTally.SameTopology} same topology, {theTally.Different} different "
            + $"({theTally.OneUnmeshed} meshed by one side only), {theTally.BothUnmeshed} unmeshed by both"));
        foreach (string aLine in theTally.TextDifferences.Take(theMaxLines))
        {
            output.WriteLine("  TEXT " + aLine);
        }
        foreach (string aLine in theTally.Differences.Take(theMaxLines))
        {
            output.WriteLine("  FACE " + aLine);
        }
    }

    /// <summary>
    /// Milestone 1: every text byte-identical (but for the Checked flags of decision 551), the same status flags, and at
    /// least 99% of the meshed faces bit-identical (decision 494).
    /// </summary>
    private static void AssertMilestone(Tally theTally)
    {
        Assert.Equal(theTally.Meshings, theTally.TextSame + theTally.TextSameButChecked);
        Assert.Equal(theTally.Meshings, theTally.StatusSame);
        Assert.True(theTally.Share >= 0.99, $"bit-identical share {theTally.Share:P3}");
    }

    // ------------------------------------------------------------------------------------------------- the inputs

    private static readonly Setting[] TheDataOccSettings =
    [
        new(0.05, true, 0.5, false),
        new(-0.004, false, 0.3, true),
        new(0.02, true, 0.1, true),
        new(-0.01, false, 0.5, false),
    ];

    /// <summary>data/occ's .brep files under 1 MB (as unit 4C's oracle).</summary>
    private static IEnumerable<string> DataOccFiles()
        => Directory.GetFiles(DataOcc, "*.brep").Order(StringComparer.Ordinal).Where(f => new FileInfo(f).Length <= 60_000);

    /// <summary>The data files of OCCT's mesh group (tests/mesh/data) that the public dataset has.</summary>
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

    /// <summary>
    /// The public dataset's other .brep files under 120 KB, every <paramref name="theStride"/>-th in name order (the
    /// mesh group's are in <see cref="MeshGroupFiles"/>).
    /// </summary>
    private static IEnumerable<string> DatasetFiles(int theStride)
    {
        string aDataset = Metadata("OcctTestDataDir");
        if (!Directory.Exists(aDataset))
        {
            return [];
        }
        var aMesh = new HashSet<string>(MeshGroupFiles(), StringComparer.Ordinal);
        return Directory.EnumerateFiles(aDataset, "*.brep", SearchOption.AllDirectories)
                        .Where(f => !aMesh.Contains(f) && new FileInfo(f).Length <= 120_000)
                        .Order(StringComparer.Ordinal)
                        .Where((f, i) => i % theStride == 0);
    }

    /// <summary>F1 of the file, or null if native OCCT doesn't read a shape from it.</summary>
    private string? TryF1(string theFile)
    {
        var aShape = new N.TopoDS_Shape();
        using (var aBuilder = new N.BRep_Builder())
        {
            if (!N.BRepTools.Read(aShape, theFile, aBuilder) || aShape.IsNull())
            {
                aShape.Dispose();
                return null;
            }
        }
        N.BRepTools.Clean(aShape);
        string aF1 = NativeWrite(aShape, false);
        aShape.Dispose();
        return aF1;
    }

    private Tally RunFiles(IEnumerable<string> theFiles, Setting[] theSettings)
    {
        var aTally = new Tally();
        foreach (string aFile in Tape.Budget(theFiles, isSpread: true))
        {
            string? aF1 = TryF1(aFile);
            if (aF1 is null)
            {
                continue;
            }
            foreach (Setting aSetting in theSettings)
            {
                Compare(aTally, Path.GetFileName(aFile), aF1, aSetting);
            }
        }
        return aTally;
    }

    // -------------------------------------------------------------------------------------------------- the tests

    [Fact]
    public void Data_occ_meshes_match_native()
    {
        Tally aTally = RunFiles(DataOccFiles(), TheDataOccSettings);
        Report(aTally, "data/occ");
        AssertMilestone(aTally);
    }

    [Fact]
    public void Mesh_group_data_meshes_match_native()
    {
        string[] aFiles = MeshGroupFiles().ToArray();
        if (aFiles.Length == 0)
        {
            output.WriteLine("OcctTestDataDir isn't set: nothing to check");
            return;
        }
        // The mesh group's own setting (incmesh res 0.1, serial and -parallel), and a relative one.
        Tally aTally = RunFiles(aFiles, [new(0.1, false, 0.5, false), new(0.1, false, 0.5, true), new(0.05, true, 0.3, true)]);
        Report(aTally, $"mesh group data ({aFiles.Length} files)");
        AssertMilestone(aTally);
    }

    [Fact]
    public void Dataset_brep_meshes_match_native()
    {
        string[] aFiles = DatasetFiles(2).ToArray();
        if (aFiles.Length == 0)
        {
            output.WriteLine("OcctTestDataDir isn't set: nothing to check");
            return;
        }
        Tally aTally = RunFiles(aFiles, [new(0.05, true, 0.5, true), new(-0.004, false, 0.3, false)]);
        Report(aTally, $"dataset .brep files ({aFiles.Length})");
        AssertMilestone(aTally);
    }

    /// <summary>Random native shapes: primitives, sweeps, Booleans and fillets.</summary>
    private IEnumerable<(string Name, string F1)> RandomShapes(Oracle theO, int theCount)
    {
        for (int i = 0; i < theCount; i++)
        {
            Direction3D aZ = theO.Dir();
            Direction3D aRef = theO.Dir();
            Direction3D aX = aZ.Crossed(aRef.IsParallel(aZ, 0.1) ? (aZ.IsParallel(new Direction3D(1, 0, 0), 0.1) ? new Direction3D(0, 1, 0) : new Direction3D(1, 0, 0)) : aRef);
            using var aNAx2 = new N.gp_Ax2(Oracle.ToNative(theO.Pnt()), Oracle.ToNative(aZ), Oracle.ToNative(aX));
            double r = theO.Real(0.5, 20), h = theO.Real(0.5, 30);
            N.TopoDS_Shape? aShape = null;
            string aKind;
            try
            {
                switch (i % 10)
                {
                    case 0:
                        aKind = "cylinder";
                        aShape = new N.BRepPrimAPI_MakeCylinder(aNAx2, r, h).Shape();
                        break;
                    case 1:
                        aKind = "cone";
                        aShape = new N.BRepPrimAPI_MakeCone(aNAx2, r, theO.Int(0, 1) == 0 ? 0.0 : r * theO.Real(0.1, 0.9), h).Shape();
                        break;
                    case 2:
                        aKind = "sphere";
                        aShape = new N.BRepPrimAPI_MakeSphere(aNAx2, r).Shape();
                        break;
                    case 3:
                        aKind = "torus";
                        aShape = new N.BRepPrimAPI_MakeTorus(aNAx2, r + h, r * theO.Real(0.1, 0.9)).Shape();
                        break;
                    case 4:
                        aKind = "partial cylinder";
                        aShape = new N.BRepPrimAPI_MakeCylinder(aNAx2, r, h, theO.Real(0.5, 6.0)).Shape();
                        break;
                    case 5:
                    {
                        aKind = "box with fillets";
                        using var aBox = new N.BRepPrimAPI_MakeBox(aNAx2, r + 1, h + 1, theO.Real(1, 20));
                        N.TopoDS_Shape aSolid = aBox.Shape();
                        using var aFillet = new N.BRepFilletAPI_MakeFillet(aSolid);
                        using var aMap = new N.TopTools_IndexedMapOfShape();
                        N.TopExp.MapShapes(aSolid, N.TopAbs_ShapeEnum.TopAbs_EDGE, aMap);
                        double aRadius = 0.2 * Math.Min(r + 1, Math.Min(h + 1, 1.0));
                        N.TopoDS_Shape[] anEdges = aMap.ToArray();
                        for (int e = 0; e < anEdges.Length; e += 1 + theO.Int(0, 2))
                        {
                            using N.TopoDS_Edge anEdge = N.TopoDS.Edge(anEdges[e]);
                            aFillet.Add(aRadius, anEdge);
                        }
                        aShape = aFillet.Shape();
                        break;
                    }
                    case 6:
                    {
                        aKind = "fuse of a box and a cylinder";
                        using var aBox = new N.BRepPrimAPI_MakeBox(aNAx2, 2 * r, 2 * r, h);
                        using var aCyl = new N.BRepPrimAPI_MakeCylinder(aNAx2, r * theO.Real(0.3, 0.9), 1.5 * h);
                        using var aFuse = new N.BRepAlgoAPI_Fuse(aBox.Shape(), aCyl.Shape());
                        aShape = aFuse.Shape();
                        break;
                    }
                    case 7:
                    {
                        aKind = "cut of a sphere by a box";
                        using var aSphere = new N.BRepPrimAPI_MakeSphere(aNAx2, r);
                        using var aBox = new N.BRepPrimAPI_MakeBox(aNAx2, r, r, r);
                        using var aCut = new N.BRepAlgoAPI_Cut(aSphere.Shape(), aBox.Shape());
                        aShape = aCut.Shape();
                        break;
                    }
                    case 8:
                    {
                        aKind = "prism of a B-spline";
                        int aNb = 5 + theO.Int(0, 4);
                        using var aPoles = new N.TColgp_Array1OfPnt(1, aNb);
                        for (int k = 1; k <= aNb; k++)
                        {
                            using N.gp_Pnt aP = Oracle.ToNative(new Point3D(k * r / aNb, theO.Real(-r, r) * 0.3, theO.Real(-1, 1)));
                            aPoles.SetValue(k, aP);
                        }
                        using var anApprox = new N.GeomAPI_PointsToBSpline(aPoles);
                        using var anEdge = new N.BRepBuilderAPI_MakeEdge(anApprox.Curve());
                        using N.gp_Vec aDir = Oracle.ToNative(new Vector3D(aZ.XYZ()) * h);
                        using var aPrism = new N.BRepPrimAPI_MakePrism(anEdge.Shape(), aDir);
                        aShape = aPrism.Shape();
                        break;
                    }
                    default:
                    {
                        aKind = "revolution of an arc";
                        using N.gp_Pnt aP1 = Oracle.ToNative(new Point3D(r, 0, 0));
                        using N.gp_Pnt aP2 = Oracle.ToNative(new Point3D(r + h * 0.3, 0, h * 0.5));
                        using N.gp_Pnt aP3 = Oracle.ToNative(new Point3D(r, 0, h));
                        using var anArc = new N.GC_MakeArcOfCircle(aP1, aP2, aP3);
                        using var anEdge = new N.BRepBuilderAPI_MakeEdge(anArc.Value());
                        using var anAxis = new N.gp_Ax1(Oracle.ToNative(Constants.Origin()), Oracle.ToNative(Constants.DZ()));
                        using var aRevol = new N.BRepPrimAPI_MakeRevol(anEdge.Shape(), anAxis, theO.Real(1.0, 2 * Math.PI));
                        aShape = aRevol.Shape();
                        break;
                    }
                }
            }
            catch (Exception anException) when (anException is not Xunit.Sdk.XunitException)
            {
                output.WriteLine($"random shape {i}: native construction failed ({anException.Message})");
                aShape?.Dispose();
                continue;
            }

            if (aShape is null || aShape.IsNull())
            {
                continue;
            }
            string aF1 = NativeWrite(aShape, false);
            aShape.Dispose();
            yield return ($"random {i} {aKind}", aF1);
        }
    }

    [Fact]
    public void Random_native_shapes_mesh_as_native()
    {
        var anOracle = new Oracle(output, 40401);
        var aTally = new Tally();
        foreach ((string aName, string aF1) in Tape.Budget(RandomShapes(anOracle, 80)))
        {
            foreach (Setting aSetting in new Setting[] { new(0.05, true, 0.5, false), new(-0.004, false, 0.3, true), new(0.1, true, 0.2, true) })
            {
                Compare(aTally, aName, aF1, aSetting);
            }
        }
        Report(aTally, "random native shapes");
        AssertMilestone(aTally);
    }

    /// <summary>
    /// Re-meshing (phase4-plan section 3): shapes carrying a native mesh (F2) are meshed again on both sides, coarser,
    /// equal and finer, so that both keep or replace the same faces' meshes.
    /// </summary>
    [Fact]
    public void Remeshing_keeps_and_replaces_meshes_as_native()
    {
        var aTally = new Tally();
        foreach (string aName in Tape.Budget(new[] { "solid.brep", "Room.brep", "mal_tige.brep", "fuse.brep", "face.brep", "bottle.brep", "Pump_Nut.brep", "MODERN_Table_1.brep" }))
        {
            string aF1 = F1(Path.Combine(DataOcc, aName));
            (string aF2, NativeTriangulation?[] _, int _) = NativeMesh(aF1, 0.05, new Setting(0.05, true, 0.5, false));
            foreach (Setting aSetting in new Setting[] { new(0.1, true, 0.5, false), new(0.05, true, 0.5, true), new(0.02, true, 0.5, false), new(0.05, true, 0.2, true) })
            {
                Compare(aTally, $"{aName} meshed again", aF2, aSetting);
            }
        }
        Report(aTally, "re-meshed data/occ");
        AssertMilestone(aTally);
    }

    /// <summary>
    /// BRepGProp and BRepBndLib with <c>useTriangulation</c>: the port's on its meshes against native's on native meshes.
    /// </summary>
    [Fact]
    public void Mesh_based_properties_and_boxes_match_native()
    {
        var anOracle = new Oracle(output, 40402);
        int aShapes = 0;
        foreach (string aFile in Tape.Budget(DataOccFiles(), isSpread: true))
        {
            string aF1 = F1(aFile);
            foreach (Setting aSetting in new Setting[] { new(0.05, true, 0.5, false), new(0.02, true, 0.3, true) })
            {
                N.TopoDS_Shape aNative = NativeRead(aF1);
                using (new N.BRepMesh_IncrementalMesh(aNative, aSetting.Deflection, aSetting.Relative, aSetting.Angle, aSetting.Parallel))
                {
                }
                Shape aPort = new IncrementalMesh(PortRead(aF1), aSetting.Deflection, aSetting.Relative, aSetting.Angle, aSetting.Parallel).Shape();
                string aWhat = $"{Path.GetFileName(aFile)} ({aSetting})";

                using (var aN = new N.GProp_GProps())
                {
                    N.BRepGProp.SurfaceProperties(aNative, aN, false, true);
                    Properties aP = MassPropertiesBuilder.SurfaceProperties(aPort, false, true);
                    anOracle.Same(aN.Mass(), aP.Mass(), aWhat + " area");
                    using N.gp_Pnt aNC = aN.CentreOfMass();
                    anOracle.Same(aNC, aP.CentreOfMass(), aWhat + " area centre");
                }
                using (var aN = new N.GProp_GProps())
                {
                    N.BRepGProp.LinearProperties(aNative, aN, false, true);
                    Properties aP = MassPropertiesBuilder.LinearProperties(aPort, false, true);
                    anOracle.Same(aN.Mass(), aP.Mass(), aWhat + " length");
                }
                using (var aNB = new N.Bnd_Box())
                {
                    N.BRepBndLib.Add(aNative, aNB, true);
                    var aPB = new Box();
                    Bounds.Add(aPort, ref aPB, true);
                    anOracle.Same(aNB.IsVoid(), aPB.IsVoid, aWhat + " box void");
                    if (!aNB.IsVoid())
                    {
                        double a1 = 0, a2 = 0, a3 = 0, a4 = 0, a5 = 0, a6 = 0;
                        aNB.Get(ref a1, ref a2, ref a3, ref a4, ref a5, ref a6);
                        aPB.Get(out double b1, out double b2, out double b3, out double b4, out double b5, out double b6);
                        anOracle.Same(a1, b1, aWhat + " box"); anOracle.Same(a2, b2, aWhat + " box"); anOracle.Same(a3, b3, aWhat + " box");
                        anOracle.Same(a4, b4, aWhat + " box"); anOracle.Same(a5, b5, aWhat + " box"); anOracle.Same(a6, b6, aWhat + " box");
                    }
                }
                aNative.Dispose();
                aShapes++;
            }
        }
        output.WriteLine($"{aShapes} meshed shapes");
        anOracle.Report("BRepGProp and BRepBndLib on meshes");
    }

    /// <summary>The port's meshes as bytes: the BRep text and every triangulation's doubles.</summary>
    private string PortDump(in Shape theShape)
    {
        var aBuilder = new StringBuilder(Text(PortWrite(theShape)));
        IndexedSet<Shape> aFaces = Map(theShape, ShapeType.Face);
        for (int i = 0; i < aFaces.Count; i++)
        {
            Triangulation? aT = Tool.Triangulation(ShapeCasts.Face(aFaces[i]), out _);
            if (aT is null)
            {
                aBuilder.Append("null\n");
                continue;
            }
            aBuilder.Append(BitConverter.DoubleToInt64Bits(aT.Deflection())).Append('\n');
            for (int k = 1; k <= aT.NodeCount; k++)
            {
                Point3D p = aT.Node(k);
                aBuilder.Append(BitConverter.DoubleToInt64Bits(p.X)).Append(' ').Append(BitConverter.DoubleToInt64Bits(p.Y)).Append(' ')
                        .Append(BitConverter.DoubleToInt64Bits(p.Z));
                if (aT.HasUVNodes)
                {
                    Point2D uv = aT.UVNode(k);
                    aBuilder.Append(' ').Append(BitConverter.DoubleToInt64Bits(uv.X)).Append(' ').Append(BitConverter.DoubleToInt64Bits(uv.Y));
                }
                aBuilder.Append('\n');
            }
        }
        return aBuilder.ToString();
    }

    /// <summary>
    /// Determinism (decision 490, phase4-plan 2.2): the port's parallel meshes equal its serial ones bit for bit, every
    /// run, and native's parallel meshes equal native's serial ones (so a native nondeterminism isn't blamed on the port).
    /// </summary>
    [Fact]
    public void Parallel_meshing_equals_serial_bit_for_bit()
    {
        int aShapes = 0, aNativeDiffer = 0;
        IEnumerable<string> aFiles = DataOccFiles().Concat(MeshGroupFiles().Where((f, i) => i % 3 == 0));
        foreach (string aFile in Tape.Budget(aFiles, isSpread: true))
        {
            string? aF1 = TryF1(aFile);
            if (aF1 is null)
            {
                continue;
            }
            foreach ((double aDeflection, bool isRelative, double anAngle) in new[] { (0.05, true, 0.5), (0.02, true, 0.2) })
            {
                Shape aShape = PortRead(aF1);
                string aSerial = PortDump(new IncrementalMesh(aShape, aDeflection, isRelative, anAngle, false).Shape());
                for (int aRun = 0; aRun < 3; aRun++)
                {
                    string aParallel = PortDump(new IncrementalMesh(aShape, aDeflection, isRelative, anAngle, true).Shape());
                    Assert.True(aSerial == aParallel, $"{aFile} ({aDeflection}, {anAngle}) parallel run {aRun} differs from serial");
                }
                using (global::Kerf.Engine.Foundation.OsdParallel.ForceSerialScope())
                {
                    string aForced = PortDump(new IncrementalMesh(aShape, aDeflection, isRelative, anAngle, true).Shape());
                    Assert.True(aSerial == aForced, $"{aFile}: forced serial differs");
                }

                // Native: serial against parallel, through the BRep text.
                (string aNS, NativeTriangulation?[] _, int _) = NativeMesh(aF1, aDeflection, new Setting(aDeflection, isRelative, anAngle, false));
                (string aNP, NativeTriangulation?[] _, int _) = NativeMesh(aF1, aDeflection, new Setting(aDeflection, isRelative, anAngle, true));
                if (Text(aNS) != Text(aNP))
                {
                    aNativeDiffer++;
                    output.WriteLine($"native parallel differs from native serial: {Path.GetFileName(aFile)} ({aDeflection}, {anAngle})");
                }
                aShapes++;
            }
        }
        output.WriteLine($"{aShapes} meshings: the port's parallel runs (3 each, and under ForceSerialScope) equal its serial run bit for bit; "
                         + $"native parallel differs from native serial in {aNativeDiffer}");
    }
}
