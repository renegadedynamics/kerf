// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Reflection;
using Xunit.Abstractions;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// The viewer's mesh extraction (unit 4W, open question 12): <see cref="MeshExtraction"/> against build123d.NET's native
/// <c>Extras.ExtractMesh</c> (b3d_mesh_extract) on the same shapes, float for float.
/// </summary>
/// <remarks>
/// <para>
/// Two comparisons. <b>The same mesh</b>: native OCCT meshes F1 and writes F2 with its triangulations and polygons; both
/// sides read F2 and extract, so only the extraction is compared (normals, locations, winding, the choice of each
/// edge's polyline and the tangential deflection of edges without one). <b>End to end</b>: both sides read F1, mesh it
/// with the same arguments and extract, which is what Bento's viewer does; differences there come from the mesher
/// (decision 494's attributions).
/// </para>
/// <para>
/// Faces and edges are taken in build123d.NET's <c>Shape.Faces()</c> and <c>Edges()</c> order on both sides
/// (<see cref="MeshExtraction.Faces"/>; natively the same exploration keyed by <c>std::hash</c>).
/// </para>
/// </remarks>
public sealed class MeshExtractionOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-extract4w-").FullName;
    private int myFileNumber;

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

    private string NewFile() => Path.Combine(myDirectory, $"x{myFileNumber++}.brep");

    private static string Metadata(string theKey)
        => typeof(MeshExtractionOracleTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>()
                                                  .Single(a => a.Key == theKey).Value!;

    private static string DataOcc => Path.Combine(Metadata("OcctSourceDir"), "data", "occ");

    private static TopoDS_Shape PortRead(string theFile)
    {
        Assert.True(BRepTools.Read(out TopoDS_Shape aShape, theFile), theFile);
        return aShape;
    }

    private static N.TopoDS_Shape NativeRead(string theFile)
    {
        var aShape = new N.TopoDS_Shape();
        using var aBuilder = new N.BRep_Builder();
        Assert.True(N.BRepTools.Read(aShape, theFile, aBuilder), theFile);
        return aShape;
    }

    private string NativeWrite(N.TopoDS_Shape theShape, bool theWithTriangles)
    {
        string aFile = NewFile();
        Assert.True(N.BRepTools.Write(theShape, aFile, theWithTriangles, false, (N.TopTools_FormatVersion)(int)TopTools_FormatVersion.TopTools_FormatVersion_CURRENT));
        return aFile;
    }

    /// <summary>F1: native OCCT's read of the file without its meshes; null if native reads no shape from it.</summary>
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

    /// <summary>build123d.NET's <c>TopoUtil.Entities</c>: distinct sub-shapes in exploration order, the last orientation kept.</summary>
    private static List<N.TopoDS_Shape> NativeEntities(N.TopoDS_Shape theShape, N.TopAbs_ShapeEnum theType)
    {
        var anIndex = new Dictionary<ulong, int>();
        var aFound = new List<N.TopoDS_Shape>();
        using var anExp = new N.TopExp_Explorer(theShape, theType);
        for (; anExp.More(); anExp.Next())
        {
            N.TopoDS_Shape anItem = anExp.Current();
            ulong aHash = anItem.StdHash();
            if (anIndex.TryGetValue(aHash, out int i))
            {
                aFound[i].Dispose();
                aFound[i] = anItem;
            }
            else
            {
                anIndex[aHash] = aFound.Count;
                aFound.Add(anItem);
            }
        }
        return aFound;
    }

    private static N.MeshBuffers NativeExtract(N.TopoDS_Shape theShape, double theDeflection, double theAngle)
    {
        List<N.TopoDS_Shape> aFaces = NativeEntities(theShape, N.TopAbs_ShapeEnum.TopAbs_FACE);
        List<N.TopoDS_Shape> anEdges = NativeEntities(theShape, N.TopAbs_ShapeEnum.TopAbs_EDGE);
        try
        {
            return N.Extras.ExtractMesh(theShape, aFaces, anEdges, theDeflection, theAngle);
        }
        finally
        {
            foreach (N.TopoDS_Shape aShape in aFaces.Concat(anEdges))
            {
                aShape.Dispose();
            }
        }
    }

    /// <summary>The viewer's deflection for a shape: 0.1% of its bounding box's diagonal (build123d.NET's MeshTolerances.For).</summary>
    private static double ViewerDeflection(in TopoDS_Shape theShape)
    {
        var aBox = new Bnd_Box();
        BRepBndLib.Add(theShape, ref aBox, true);
        double aDiagonal = aBox.IsVoid() ? 0.0 : Math.Sqrt(aBox.SquareExtent());
        return aDiagonal > 1e-6 ? aDiagonal * 0.1 / 100 : 1e-3;
    }

    private sealed class Tally
    {
        public int Shapes, Faces, FacesSame, Edges, EdgesSame, ShapesSame;
        public long Floats, FloatsSame;
        public readonly List<string> Differences = [];

        public double FaceShare => Faces == 0 ? 1.0 : (double)FacesSame / Faces;

        public double EdgeShare => Edges == 0 ? 1.0 : (double)EdgesSame / Edges;

        public double FloatShare => Floats == 0 ? 1.0 : (double)FloatsSame / Floats;
    }

    /// <summary>Bit for bit (so -0 differs from 0, and a NaN equals the same NaN).</summary>
    private static bool Same<T>(ReadOnlySpan<T> a, ReadOnlySpan<T> b) where T : unmanaged
        => System.Runtime.InteropServices.MemoryMarshal.AsBytes(a).SequenceEqual(System.Runtime.InteropServices.MemoryMarshal.AsBytes(b));

    private static int SameFloats(ReadOnlySpan<float> a, ReadOnlySpan<float> b)
    {
        int aCount = 0;
        for (int i = 0; i < Math.Min(a.Length, b.Length); i++)
        {
            if (BitConverter.SingleToInt32Bits(a[i]) == BitConverter.SingleToInt32Bits(b[i]))
            {
                aCount++;
            }
        }
        return aCount;
    }

    private static float MaxDiff(ReadOnlySpan<float> a, ReadOnlySpan<float> b)
    {
        float aMax = 0;
        for (int i = 0; i < Math.Min(a.Length, b.Length); i++)
        {
            aMax = Math.Max(aMax, Math.Abs(a[i] - b[i]));
        }
        return aMax;
    }

    /// <summary>Compares the port's buffers with native's, face by face and edge by edge, every float bit for bit.</summary>
    private static void Compare(Tally theTally, string theWhat, MeshBuffers thePort, N.MeshBuffers theNative)
    {
        theTally.Shapes++;
        Assert.Equal(theNative.FaceTypes.Length, thePort.FaceTypes.Length);
        Assert.Equal(theNative.EdgeTypes.Length, thePort.EdgeTypes.Length);
        bool isShapeSame = true;
        for (int f = 0; f < thePort.FaceTypes.Length; f++)
        {
            theTally.Faces++;
            long n0 = theNative.FaceNodes[f], n1 = theNative.FaceNodes[f + 1], p0 = thePort.FaceNodes[f], p1 = thePort.FaceNodes[f + 1];
            long t0 = theNative.FaceTriangles[f], t1 = theNative.FaceTriangles[f + 1], q0 = thePort.FaceTriangles[f], q1 = thePort.FaceTriangles[f + 1];
            ReadOnlySpan<float> aNP = theNative.Positions.AsSpan((int)(3 * n0), (int)(3 * (n1 - n0)));
            ReadOnlySpan<float> aPP = thePort.Positions.AsSpan((int)(3 * p0), (int)(3 * (p1 - p0)));
            ReadOnlySpan<float> aNN = theNative.Normals.AsSpan((int)(3 * n0), (int)(3 * (n1 - n0)));
            ReadOnlySpan<float> aPN = thePort.Normals.AsSpan((int)(3 * p0), (int)(3 * (p1 - p0)));
            ReadOnlySpan<int> aNT = theNative.Triangles.AsSpan((int)t0, (int)(t1 - t0));
            ReadOnlySpan<int> aPT = thePort.Triangles.AsSpan((int)q0, (int)(q1 - q0));
            theTally.Floats += aNP.Length + aNN.Length;
            theTally.FloatsSame += SameFloats(aNP, aPP) + SameFloats(aNN, aPN);
            bool isSame = theNative.FaceTypes[f] == thePort.FaceTypes[f] && Same(aNP, aPP) && Same(aNN, aPN) && Same(aNT, aPT);
            if (isSame)
            {
                theTally.FacesSame++;
            }
            else
            {
                isShapeSame = false;
                theTally.Differences.Add(string.Create(CultureInfo.InvariantCulture,
                    $"{theWhat} face {f}: type {theNative.FaceTypes[f]}/{thePort.FaceTypes[f]}, nodes {n1 - n0}/{p1 - p0}, "
                    + $"triangles {(t1 - t0) / 3}/{(q1 - q0) / 3}{(Same(aNT, aPT) ? " (same)" : "")}, positions within {MaxDiff(aNP, aPP):E2}, "
                    + $"normals within {MaxDiff(aNN, aPN):E2} (native/port)"));
            }
        }
        for (int e = 0; e < thePort.EdgeTypes.Length; e++)
        {
            theTally.Edges++;
            long n0 = theNative.EdgePoints[e], n1 = theNative.EdgePoints[e + 1], p0 = thePort.EdgePoints[e], p1 = thePort.EdgePoints[e + 1];
            ReadOnlySpan<float> aN = theNative.Points.AsSpan((int)(3 * n0), (int)(3 * (n1 - n0)));
            ReadOnlySpan<float> aP = thePort.Points.AsSpan((int)(3 * p0), (int)(3 * (p1 - p0)));
            theTally.Floats += aN.Length;
            theTally.FloatsSame += SameFloats(aN, aP);
            if (theNative.EdgeTypes[e] == thePort.EdgeTypes[e] && Same(aN, aP))
            {
                theTally.EdgesSame++;
            }
            else
            {
                isShapeSame = false;
                theTally.Differences.Add(string.Create(CultureInfo.InvariantCulture,
                    $"{theWhat} edge {e}: type {theNative.EdgeTypes[e]}/{thePort.EdgeTypes[e]}, points {n1 - n0}/{p1 - p0}, within {MaxDiff(aN, aP):E2} (native/port)"));
            }
        }
        if (isShapeSame)
        {
            theTally.ShapesSame++;
        }
    }

    private void Report(Tally theTally, string theWhat)
    {
        output.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"{theWhat}: {theTally.Shapes} extractions, {theTally.ShapesSame} identical; faces {theTally.FacesSame}/{theTally.Faces} "
            + $"({theTally.FaceShare:P3}), edges {theTally.EdgesSame}/{theTally.Edges} ({theTally.EdgeShare:P3}), floats "
            + $"{theTally.FloatsSame}/{theTally.Floats} ({theTally.FloatShare:P4}) bit-identical"));
        foreach (string aLine in theTally.Differences.Take(60))
        {
            output.WriteLine("  " + aLine);
        }
        if (theTally.Differences.Count > 60)
        {
            output.WriteLine($"  ... and {theTally.Differences.Count - 60} more");
        }
    }

    /// <summary>
    /// The same mesh on both sides: native meshes F1 (at <paramref name="theDeflection"/>, or the viewer's deflection
    /// if it is negative) and writes F2; both read F2 and extract.
    /// </summary>
    private void CompareSameMesh(Tally theTally, string theName, string theF1, double theDeflection, bool theRelative, double theAngle)
    {
        TopoDS_Shape aPortF1 = PortRead(theF1);
        double aDeflection = theDeflection < 0 ? ViewerDeflection(aPortF1) : theDeflection;
        if (aDeflection < Precision.Confusion())
        {
            return;
        }
        N.TopoDS_Shape aNative = NativeRead(theF1);
        new N.BRepMesh_IncrementalMesh(aNative, aDeflection, theRelative, theAngle, true).Dispose();
        string aF2 = NativeWrite(aNative, true);
        aNative.Dispose();

        N.TopoDS_Shape aNativeF2 = NativeRead(aF2);
        N.MeshBuffers aNativeBuffers = NativeExtract(aNativeF2, aDeflection, theAngle);
        aNativeF2.Dispose();
        MeshBuffers aPortBuffers = MeshExtraction.Extract(PortRead(aF2), aDeflection, theAngle);
        Compare(theTally, theName, aPortBuffers, aNativeBuffers);
    }

    /// <summary>End to end: both read F1, mesh it with the same arguments and extract.</summary>
    private void CompareEndToEnd(Tally theTally, string theName, string theF1, double theDeflection, bool theRelative, double theAngle, bool theParallel)
    {
        TopoDS_Shape aPortF1 = PortRead(theF1);
        double aDeflection = theDeflection < 0 ? ViewerDeflection(aPortF1) : theDeflection;
        if (aDeflection < Precision.Confusion())
        {
            return;
        }
        N.TopoDS_Shape aNative = NativeRead(theF1);
        new N.BRepMesh_IncrementalMesh(aNative, aDeflection, theRelative, theAngle, theParallel).Dispose();
        N.MeshBuffers aNativeBuffers = NativeExtract(aNative, aDeflection, theAngle);
        aNative.Dispose();

        TopoDS_Shape aMeshed = new BRepMesh_IncrementalMesh(aPortF1, aDeflection, theRelative, theAngle, theParallel).Shape();
        MeshBuffers aPortBuffers = MeshExtraction.Extract(aMeshed, aDeflection, theAngle);
        Compare(theTally, theName, aPortBuffers, aNativeBuffers);
    }

    private static IEnumerable<string> DataOccFiles()
        => Directory.GetFiles(DataOcc, "*.brep").Order(StringComparer.Ordinal).Where(f => new FileInfo(f).Length <= 60_000);

    [Fact]
    public void Extraction_of_the_same_mesh_matches_native_float_for_float()
    {
        var aTally = new Tally();
        foreach (string aFile in Tape.Budget(DataOccFiles(), isSpread: true))
        {
            string? aF1 = TryF1(aFile);
            if (aF1 is null)
            {
                continue;
            }
            CompareSameMesh(aTally, Path.GetFileName(aFile) + " viewer", aF1, -1, false, 0.2);
            CompareSameMesh(aTally, Path.GetFileName(aFile) + " rel 0.05", aF1, 0.05, true, 0.5);
        }
        foreach ((string aName, string aF1) in Tape.Budget(SpecialShapes()))
        {
            CompareSameMesh(aTally, aName, aF1, -1, false, 0.2);
        }
        Report(aTally, "extraction of native's mesh");
        Assert.True(aTally.Shapes > 0); // as recorded, a budgeted sample (decision 1337)
        Assert.Empty(aTally.Differences);
    }

    [Fact]
    public void Mesh_and_extract_matches_native()
    {
        var aTally = new Tally();
        foreach (string aFile in Tape.Budget(DataOccFiles(), isSpread: true))
        {
            string? aF1 = TryF1(aFile);
            if (aF1 is null)
            {
                continue;
            }
            CompareEndToEnd(aTally, Path.GetFileName(aFile) + " viewer", aF1, -1, false, 0.2, true);
        }
        foreach ((string aName, string aF1) in Tape.Budget(SpecialShapes()))
        {
            CompareEndToEnd(aTally, aName + " viewer", aF1, -1, false, 0.2, false);
            CompareEndToEnd(aTally, aName + " rel 0.1", aF1, 0.1, true, 0.3, true);
        }
        Report(aTally, "mesh and extract");
        // The mesher's milestone (decision 494): at least 99% of faces bit-identical; edges follow their faces
        Assert.True(aTally.FaceShare >= 0.99, $"faces {aTally.FaceShare:P3}");
        Assert.True(aTally.EdgeShare >= 0.99, $"edges {aTally.EdgeShare:P3}");
    }

    /// <summary>The data files of OCCT's mesh group (tests/mesh/data) that the public dataset has (as unit 4D's oracle).</summary>
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
    public void Mesh_group_data_meshes_and_extracts_as_native()
    {
        var aTally = new Tally();
        foreach (string aFile in Tape.Budget(MeshGroupFiles(), isSpread: true))
        {
            string? aF1 = TryF1(aFile);
            if (aF1 is null)
            {
                continue;
            }
            CompareEndToEnd(aTally, Path.GetFileName(aFile) + " viewer", aF1, -1, false, 0.2, true);
        }
        Report(aTally, "mesh group data, mesh and extract");
        Assert.True(aTally.Shapes > 0); // as recorded, a budgeted sample (decision 1337)
        Assert.True(aTally.FaceShare >= 0.99, $"faces {aTally.FaceShare:P3}");
        Assert.True(aTally.EdgeShare >= 0.99, $"edges {aTally.EdgeShare:P3}");
    }

    [Fact]
    public void Unmeshed_shapes_extract_edges_by_tangential_deflection_as_native()
    {
        var aTally = new Tally();
        foreach (string aFile in Tape.Budget(DataOccFiles(), isSpread: true))
        {
            string? aF1 = TryF1(aFile);
            if (aF1 is null)
            {
                continue;
            }
            TopoDS_Shape aPort = PortRead(aF1);
            double aDeflection = ViewerDeflection(aPort);
            if (aDeflection < Precision.Confusion())
            {
                continue;
            }
            N.TopoDS_Shape aNative = NativeRead(aF1);
            N.MeshBuffers aNativeBuffers = NativeExtract(aNative, aDeflection, 0.2);
            aNative.Dispose();
            MeshBuffers aPortBuffers = MeshExtraction.Extract(aPort, aDeflection, 0.2);
            Assert.All(aPortBuffers.FaceNodes, n => Assert.Equal(0, n));
            Compare(aTally, Path.GetFileName(aFile) + " unmeshed", aPortBuffers, aNativeBuffers);
        }
        Report(aTally, "unmeshed shapes (every edge by GCPnts_TangentialDeflection)");
        Assert.True(aTally.Edges > 0); // as recorded, a budgeted sample (decision 1337)
        // GCPnts on circles uses Math.Sin and Math.Cos, which may round differently from MSVC's (decision 494)
        Assert.True(aTally.EdgeShare >= 0.99, $"edges {aTally.EdgeShare:P3}");
    }

    /// <summary>
    /// Native shapes exercising what data/occ lacks: located, mirrored and reversed faces and solids, free edges
    /// (meshed to 3D polygons), primitives, a filleted box and Booleans.
    /// </summary>
    private IEnumerable<(string Name, string F1)> SpecialShapes()
    {
        using var anAx2 = new N.gp_Ax2(Oracle.ToNative(new gp_Pnt(1, 2, 3)), Oracle.ToNative(new gp_Dir(0.2, 0.3, 1)));
        using var aBoxMaker = new N.BRepPrimAPI_MakeBox(anAx2, 10, 20, 30);
        using var aCylMaker = new N.BRepPrimAPI_MakeCylinder(anAx2, 6, 40);
        using var aSphereMaker = new N.BRepPrimAPI_MakeSphere(anAx2, 12);
        using var aTorusMaker = new N.BRepPrimAPI_MakeTorus(anAx2, 20, 4);
        using var aConeMaker = new N.BRepPrimAPI_MakeCone(anAx2, 8, 2, 15);
        N.TopoDS_Shape aBox = aBoxMaker.Shape(), aCyl = aCylMaker.Shape(), aSphere = aSphereMaker.Shape();

        using var aFillet = new N.BRepFilletAPI_MakeFillet(aBox);
        List<N.TopoDS_Shape> aBoxEdges = NativeEntities(aBox, N.TopAbs_ShapeEnum.TopAbs_EDGE);
        for (int e = 0; e < aBoxEdges.Count; e += 2)
        {
            using N.TopoDS_Edge anEdge = N.TopoDS.Edge(aBoxEdges[e]);
            aFillet.Add(2.0, anEdge);
        }
        using var aFuse = new N.BRepAlgoAPI_Fuse(aBox, aCyl);
        using var aCut = new N.BRepAlgoAPI_Cut(aSphere, aBox);

        // A rotation with a translation, and a mirror (a location whose transformation is negative)
        using var aRotation = new N.gp_Trsf();
        using (var anAxis = new N.gp_Ax1(Oracle.ToNative(new gp_Pnt(3, -1, 2)), Oracle.ToNative(new gp_Dir(1, 1, 0.5))))
        {
            aRotation.SetRotation(anAxis, 0.7);
        }
        using (var aMove = new N.gp_Trsf())
        {
            using N.gp_Vec aVec = Oracle.ToNative(new gp_Vec(5, -7, 11));
            aMove.SetTranslation(aVec);
            aRotation.Multiply(aMove);
        }
        using var aMirror = new N.gp_Trsf();
        using (var aPlane = new N.gp_Ax2(Oracle.ToNative(new gp_Pnt(0, 0, 4)), Oracle.ToNative(new gp_Dir(0.3, 0.1, 1))))
        {
            aMirror.SetMirror(aPlane);
        }
        using var aRotLoc = new N.TopLoc_Location(aRotation);
        using var aMirLoc = new N.TopLoc_Location(aMirror);

        var aShapes = new List<(string, N.TopoDS_Shape)>
        {
            ("box", aBox),
            ("cylinder", aCyl),
            ("sphere", aSphere),
            ("torus", aTorusMaker.Shape()),
            ("cone", aConeMaker.Shape()),
            ("filleted box", aFillet.Shape()),
            ("fuse of a box and a cylinder", aFuse.Shape()),
            ("cut of a sphere by a box", aCut.Shape()),
            ("moved cylinder", aCyl.Moved(aRotLoc)),
            ("mirrored filleted box", aFillet.Shape().Moved(aMirLoc)),
            ("reversed sphere", aSphere.Reversed()),
            ("mirrored reversed torus", aTorusMaker.Shape().Reversed().Moved(aMirLoc)),
        };

        // A compound of single faces, located, mirrored and reversed, and free edges (meshed to 3D polygons)
        using (var aBuilder = new N.BRep_Builder())
        {
            var aCompound = new N.TopoDS_Compound();
            aBuilder.MakeCompound(aCompound);
            List<N.TopoDS_Shape> aCylFaces = NativeEntities(aFuse.Shape(), N.TopAbs_ShapeEnum.TopAbs_FACE);
            for (int i = 0; i < aCylFaces.Count; i++)
            {
                N.TopoDS_Shape aFace = (i % 4) switch
                {
                    0 => aCylFaces[i].Moved(aRotLoc),
                    1 => aCylFaces[i].Reversed(),
                    2 => aCylFaces[i].Moved(aMirLoc),
                    _ => aCylFaces[i].Reversed().Moved(aRotLoc),
                };
                aBuilder.Add(aCompound, aFace);
            }
            List<N.TopoDS_Shape> aTorusEdges = NativeEntities(aTorusMaker.Shape(), N.TopAbs_ShapeEnum.TopAbs_EDGE);
            foreach (N.TopoDS_Shape anEdge in aTorusEdges)
            {
                aBuilder.Add(aCompound, anEdge.Moved(aRotLoc));
                aBuilder.Add(aCompound, anEdge.Reversed().Moved(aMirLoc));
            }
            aShapes.Add(("compound of located and reversed faces and free edges", aCompound));
        }

        foreach ((string aName, N.TopoDS_Shape aShape) in aShapes)
        {
            yield return (aName, NativeWrite(aShape, false));
        }
    }
}
