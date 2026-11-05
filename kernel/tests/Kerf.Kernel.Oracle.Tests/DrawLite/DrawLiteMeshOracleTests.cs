// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Diagnostics;
using System.Globalization;
using System.Reflection;
using System.Text;
using System.Text.RegularExpressions;
using Kerf.Kernel.DrawLite;
using Xunit.Abstractions;
using static Kerf.Engine.Exchange.Brep.FormatVersion;

namespace Kerf.Kernel.Oracle.Tests;

/// <summary>
/// Draw-lite's mesh readers (unit 4L1) against native OCCT on shapes that carry triangulations: trinfo's counts and
/// maximal deflection, triarea's two areas, tricheck's free nodes, small triangles and faces without triangulation,
/// correctnormals (BRepLib::EnsureNormalConsistency: its verdict and the shape written with normals) and tclean
/// (BRepTools::Clean: the shape written after it).
/// </summary>
/// <remarks>
/// The shapes: data/occ's .brep files and the test dataset's .brep files under 400 KB that carry triangulations, each
/// with its own triangulations, and data/occ's files meshed natively (BRepTools::Clean, then BRepMesh_IncrementalMesh
/// at a relative deflection of 0.05). Native OCCT writes each one in the current format with its triangulations, and
/// both sides read that file. The native values that have no binding of their own (tricheck's) are computed here from
/// the native triangulations with native arithmetic.
/// </remarks>
public sealed partial class DrawLiteMeshOracleTests(ITestOutputHelper output) : IDisposable
{
    private readonly string myDirectory = Directory.CreateTempSubdirectory("occtnet-drawmesh-").FullName;
    private readonly List<string> myDisagreements = [];
    private int myFileNumber;
    private int myValues;
    private int myShapes;
    private int myTexts;
    private int myTextsEqual;
    private int myMeshCheckIssues;
    private int myBothFailed;

    public void Dispose() => Directory.Delete(myDirectory, true);

    private static string Metadata(string theKey)
        => typeof(DrawLiteMeshOracleTests).Assembly.GetCustomAttributes<AssemblyMetadataAttribute>().Single(a => a.Key == theKey).Value!;

    private static string ResourcesDirectory => Path.Combine(Metadata("OcctSourceDir"), "resources", "DrawResources");

    private static string DataOcc => Path.Combine(Metadata("OcctSourceDir"), "data", "occ");

    private static string TclPath(string thePath) => thePath.Replace('\\', '/');

    private string NewFile() => Path.Combine(myDirectory, $"m{myFileNumber++}.brep");

    private void Disagree(string theWhat)
    {
        if (myDisagreements.Count < 50)
        {
            output.WriteLine("DISAGREE " + theWhat);
        }

        myDisagreements.Add(theWhat);
    }

    private void Same(double theNative, double thePort, string theWhat)
    {
        myValues++;
        if (BitConverter.DoubleToInt64Bits(theNative) != BitConverter.DoubleToInt64Bits(thePort))
        {
            Disagree($"{theWhat}: native {theNative:R}, port {thePort:R}");
        }
    }

    private static DrawInterpreter Interpreter()
    {
        var aDI = new DrawInterpreter(new StringWriter { NewLine = "\n" });
        aDI.LoadDrawResources(ResourcesDirectory);
        return aDI;
    }

    private static bool HasTriangulations(string theFile)
    {
        using var aReader = new StreamReader(theFile, Encoding.Latin1);
        while (aReader.ReadLine() is { } aLine)
        {
            if (aLine.StartsWith("Triangulations ", StringComparison.Ordinal))
            {
                return aLine != "Triangulations 0";
            }

            if (aLine.StartsWith("TShapes ", StringComparison.Ordinal))
            {
                return false;
            }
        }

        return false;
    }

    /// <summary>Native OCCT reads the file (and meshes it), and writes it with its triangulations: the file both sides read.</summary>
    private string? NativeCopy(string theFile, bool theToMesh)
    {
        var aShape = new N.TopoDS_Shape();
        using var aBuilder = new N.BRep_Builder();
        try
        {
            if (!N.BRepTools.Read(aShape, theFile, aBuilder) || aShape.IsNull())
            {
                return null;
            }

            if (theToMesh)
            {
                N.BRepTools.Clean(aShape);
                using var aMesh = new N.BRepMesh_IncrementalMesh(aShape, 0.05, true, 0.5, false);
            }

            string aCopy = NewFile();
            return N.BRepTools.Write(aShape, aCopy, true, false, (N.TopTools_FormatVersion)(int)FormatVersion_CURRENT) ? aCopy : null;
        }
        catch (Exception)
        {
            return null;
        }
        finally
        {
            aShape.Dispose();
        }
    }

    private static N.TopoDS_Shape NativeRead(string theFile)
    {
        var aShape = new N.TopoDS_Shape();
        using var aBuilder = new N.BRep_Builder();
        Assert.True(N.BRepTools.Read(aShape, theFile, aBuilder), theFile);
        return aShape;
    }

    [GeneratedRegex(@"This shape contains (\d+) faces\.\n(?:\s+(\d+) empty faces\.\n)?\s+(\d+) triangles\.\n\s+(\d+) nodes\.\n\s+(\d+) polygons on triangulation\.\nMaximal deflection (\S+)\n")]
    private static partial Regex TrInfo();

    [GeneratedRegex(@"Free_links (\d+) Cross_face_errors (\d+) Async_edges (\d+) Free_nodes (\d+) Small triangles (\d+)")]
    private static partial Regex TriCheckSummary();

    private static double Real(string theText) => double.Parse(theText, NumberStyles.Float, CultureInfo.InvariantCulture);

    /// <summary>trinfo, triarea and tricheck on one file, then correctnormals and tclean.</summary>
    private void CheckFile(string theFile, string theWhat)
    {
        myShapes++;
        DrawInterpreter aDI = Interpreter();
        aDI.Eval($"restore {TclPath(theFile)} s");
        using N.TopoDS_Shape aNative = NativeRead(theFile);

        // trinfo: faces, empty faces, triangles, nodes, maximal deflection
        int aNbFaces = 0, aNbEmpty = 0, aNbTriangles = 0, aNbNodes = 0;
        double aMaxDefl = 0.0;
        var aFaceTriangulations = new List<N.Poly_Triangulation?>();
        using (var anExp = new N.TopExp_Explorer(aNative, N.TopAbs_ShapeEnum.TopAbs_FACE))
        {
            for (; anExp.More(); anExp.Next())
            {
                aNbFaces++;
                using N.TopoDS_Shape aCurrent = anExp.Current();
                using N.TopoDS_Face aFace = N.TopoDS.Face(aCurrent);
                using var aLoc = new N.TopLoc_Location();
                N.Poly_Triangulation? aT = N.BRep_Tool.Triangulation(aFace, aLoc);
                if (aT is null)
                {
                    aNbEmpty++;
                    continue;
                }

                aNbTriangles += aT.NbTriangles();
                aNbNodes += aT.NbNodes();
                aMaxDefl = Math.Max(aMaxDefl, aT.Deflection());
                aT.Dispose();
            }
        }

        string anInfo = aDI.Eval("trinfo s");
        Match aMatch = TrInfo().Match(anInfo);
        if (!aMatch.Success)
        {
            Disagree($"{theWhat} trinfo: unexpected text {anInfo}");
            return;
        }

        Same(aNbFaces, int.Parse(aMatch.Groups[1].Value, CultureInfo.InvariantCulture), $"{theWhat} trinfo faces");
        Same(aNbEmpty, aMatch.Groups[2].Success ? int.Parse(aMatch.Groups[2].Value, CultureInfo.InvariantCulture) : 0, $"{theWhat} trinfo empty faces");
        Same(aNbTriangles, int.Parse(aMatch.Groups[3].Value, CultureInfo.InvariantCulture), $"{theWhat} trinfo triangles");
        Same(aNbNodes, int.Parse(aMatch.Groups[4].Value, CultureInfo.InvariantCulture), $"{theWhat} trinfo nodes");
        Same(aMaxDefl, Real(aMatch.Groups[6].Value), $"{theWhat} trinfo deflection");

        // the faces once each (TopExp::MapShapes), for triarea and tricheck
        using var aMapF = new N.TopTools_IndexedMapOfShape();
        N.TopExp.MapShapes(aNative, N.TopAbs_ShapeEnum.TopAbs_FACE, aMapF);

        // a triangulation whose triangles name nodes it hasn't (a few dataset files) fails on both sides
        if (!ValidNodes(aMapF))
        {
            if (Fails(aDI, "triarea s") && Fails(aDI, "tricheck s"))
            {
                myBothFailed++;
            }
            else
            {
                Disagree($"{theWhat}: native OCCT fails on node indices out of range, the port doesn't");
            }

            return;
        }

        // triarea, without and with an epsilon
        foreach (string anEps in new[] { "", " 0.001" })
        {
            double aTriArea = 0;
            bool aHasPoly = false;
            var aNoTriangulation = new List<int>();
            for (int i = 1; i <= aMapF.Extent(); i++)
            {
                using N.TopoDS_Shape aKey = aMapF.FindKey(i);
                using N.TopoDS_Face aFace = N.TopoDS.Face(aKey);
                using var aLoc = new N.TopLoc_Location();
                using N.Poly_Triangulation? aPoly = N.BRep_Tool.Triangulation(aFace, aLoc);
                if (aPoly is null)
                {
                    aNoTriangulation.Add(i);
                    continue;
                }

                aHasPoly = true;
                for (int j = 1; j <= aPoly.NbTriangles(); j++)
                {
                    using N.Poly_Triangle aTri = aPoly.Triangle(j);
                    using N.gp_Pnt p1 = aPoly.Node(aTri.Value(1)), p2 = aPoly.Node(aTri.Value(2)), p3 = aPoly.Node(aTri.Value(3));
                    using var v1 = new N.gp_Vec(p1, p2);
                    using var v2 = new N.gp_Vec(p1, p3);
                    aTriArea += v1.CrossMagnitude(v2);
                }
            }

            aTriArea = aHasPoly ? aTriArea / 2 : 0;
            using var aProps = new N.GProp_GProps();
            if (anEps.Length == 0)
            {
                N.BRepGProp.SurfaceProperties(aNative, aProps);
            }
            else
            {
                N.BRepGProp.SurfaceProperties(aNative, aProps, 0.001);
            }

            var anOut = (StringWriter)aDI.Output;
            anOut.GetStringBuilder().Clear();
            string anArea = aDI.Eval("triarea s" + anEps);
            string[] aWords = anArea.Split(' ', StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);
            Same(aTriArea, Real(aWords[0]), $"{theWhat} triarea{anEps} triangles");
            Same(aProps.Mass(), Real(aWords[1]), $"{theWhat} triarea{anEps} geometry");
            string anExpected = aHasPoly ? string.Concat(aNoTriangulation.Select(i => $"face {i} has no triangulation\n")) : "";
            if (anOut.ToString() != anExpected)
            {
                Disagree($"{theWhat} triarea{anEps} printed {anOut}");
            }
        }

        // tricheck: free nodes, small triangles (with -small) and faces without triangulation, from the native meshes
        int aFreeNodes = 0, aSmall = 0;
        var aNoTri = new StringBuilder();
        for (int iF = 1; iF <= aMapF.Extent(); iF++)
        {
            using N.TopoDS_Shape aKey = aMapF.FindKey(iF);
            using N.TopoDS_Face aFace = N.TopoDS.Face(aKey);
            using var aLoc = new N.TopLoc_Location();
            using N.Poly_Triangulation? aT = N.BRep_Tool.Triangulation(aFace, aLoc);
            if (aT is null)
            {
                aNoTri.Append($"face {iF} has no triangulation\n");
                continue;
            }

            using N.gp_Trsf aTrsf = aLoc.Transformation();
            var aUsed = new HashSet<int>();
            for (int i = 1; i <= aT.NbTriangles(); i++)
            {
                using N.Poly_Triangle aTri = aT.Triangle(i);
                int n1 = aTri.Value(1), n2 = aTri.Value(2), n3 = aTri.Value(3);
                aUsed.Add(n1);
                aUsed.Add(n2);
                aUsed.Add(n3);
                using N.gp_Pnt q1 = aT.Node(n1), q2 = aT.Node(n2), q3 = aT.Node(n3);
                using N.gp_Pnt p1 = q1.Transformed(aTrsf), p2 = q2.Transformed(aTrsf), p3 = q3.Transformed(aTrsf);
                using var a = new N.gp_XYZ(p3.X() - p1.X(), p3.Y() - p1.Y(), p3.Z() - p1.Z());
                using var b = new N.gp_XYZ(p2.X() - p1.X(), p2.Y() - p1.Y(), p2.Z() - p1.Z());
                using N.gp_XYZ c = a.Crossed(b);
                double anArea = 0.5 * c.Modulus();
                if (anArea < 1e-7 * 1e-7)
                {
                    aSmall++;
                }
                else if (aT.HasUVNodes())
                {
                    using N.gp_Pnt2d u1 = aT.UVNode(n1), u2 = aT.UVNode(n2), u3 = aT.UVNode(n3);
                    double aU1 = u3.X() - u1.X(), aV1 = u3.Y() - u1.Y(), aU2 = u2.X() - u1.X(), aV2 = u2.Y() - u1.Y();
                    if (0.5 * Math.Abs(aU1 * aV2 - aV1 * aU2) < 1e-9 * 1e-9)
                    {
                        aSmall++;
                    }
                }
            }

            for (int k = 1; k <= aT.NbNodes(); k++)
            {
                if (!aUsed.Contains(k))
                {
                    aFreeNodes++;
                }
            }
        }

        string aCheck = aDI.Eval("tricheck s -small");
        if (!aCheck.StartsWith(aNoTri.ToString(), StringComparison.Ordinal))
        {
            Disagree($"{theWhat} tricheck: the faces without triangulation differ: {aCheck}");
        }

        Match aSummary = TriCheckSummary().Match(aCheck);
        Same(aFreeNodes, aSummary.Success ? Real(aSummary.Groups[4].Value) : 0, $"{theWhat} tricheck free nodes");
        Same(aSmall, aSummary.Success ? Real(aSummary.Groups[5].Value) : 0, $"{theWhat} tricheck small triangles");
        if (aSummary.Success && (aSummary.Groups[1].Value != "0" || aSummary.Groups[2].Value != "0" || aSummary.Groups[3].Value != "0"))
        {
            myMeshCheckIssues++;
        }

        // correctnormals: the verdict and the shape with its normals. OCCT dereferences a null polygon when an edge
        // has none on one of its two faces' triangulations (occt-issues.md): both sides fail then, and the native
        // shape is left half-corrected, so the file ends there.
        bool? aCorrected;
        try
        {
            aCorrected = N.BRepLib.EnsureNormalConsistency(aNative);
        }
        catch (N.OcctException)
        {
            aCorrected = null;
        }

        string? aCorrect;
        try
        {
            aCorrect = aDI.Eval("correctnormals s");
        }
        catch (DrawException)
        {
            aCorrect = null;
        }

        if (aCorrected is null || aCorrect is null)
        {
            if (aCorrected is null != aCorrect is null)
            {
                Disagree($"{theWhat} correctnormals: native {aCorrected?.ToString() ?? "failed"}, port {aCorrect ?? "failed"}");
            }
            else
            {
                myBothFailed++;
            }

            return;
        }

        if (aCorrected != (aCorrect == "Some corrections in source shape have been made!\n"))
        {
            Disagree($"{theWhat} correctnormals: native {aCorrected}, port {aCorrect}");
        }

        CompareTexts(aNative, aDI.GetShape("s"), true, $"{theWhat} correctnormals");

        // tclean
        N.BRepTools.Clean(aNative);
        aDI.Eval("tclean s");
        CompareTexts(aNative, aDI.GetShape("s"), false, $"{theWhat} tclean");
    }

    private static bool Fails(DrawInterpreter theDI, string theScript)
    {
        try
        {
            theDI.Eval(theScript);
            return false;
        }
        catch (DrawException)
        {
            return true;
        }
    }

    // every triangle's nodes are within the triangulation's
    private static bool ValidNodes(N.TopTools_IndexedMapOfShape theFaces)
    {
        for (int i = 1; i <= theFaces.Extent(); i++)
        {
            using N.TopoDS_Shape aKey = theFaces.FindKey(i);
            using N.TopoDS_Face aFace = N.TopoDS.Face(aKey);
            using var aLoc = new N.TopLoc_Location();
            using N.Poly_Triangulation? aT = N.BRep_Tool.Triangulation(aFace, aLoc);
            if (aT is null)
            {
                continue;
            }

            int aNbNodes = aT.NbNodes();
            for (int j = 1; j <= aT.NbTriangles(); j++)
            {
                using N.Poly_Triangle aTri = aT.Triangle(j);
                for (int k = 1; k <= 3; k++)
                {
                    if (aTri.Value(k) < 1 || aTri.Value(k) > aNbNodes)
                    {
                        return false;
                    }
                }
            }
        }

        return true;
    }

    private void CompareTexts(N.TopoDS_Shape theNative, in Shape thePort, bool theWithNormals, string theWhat)
    {
        string aNFile = NewFile(), aPFile = NewFile();
        Assert.True(N.BRepTools.Write(theNative, aNFile, true, theWithNormals, (N.TopTools_FormatVersion)(int)FormatVersion_CURRENT));
        Assert.True(BrepTools.Write(thePort, aPFile, true, theWithNormals, FormatVersion_CURRENT));
        myTexts++;
        if (File.ReadAllText(aNFile, Encoding.Latin1) == File.ReadAllText(aPFile, Encoding.Latin1))
        {
            myTextsEqual++;
        }
        else
        {
            Disagree($"{theWhat}: the BRep texts differ");
            string? aKeep = Environment.GetEnvironmentVariable("KERF_KEEP_DIFFS");
            if (aKeep is not null && myDisagreements.Count <= 5)
            {
                File.Copy(aNFile, Path.Combine(aKeep, $"diff{myDisagreements.Count}_native.brep"), true);
                File.Copy(aPFile, Path.Combine(aKeep, $"diff{myDisagreements.Count}_port.brep"), true);
            }
        }
    }

    [Fact]
    public void Mesh_readers_agree_with_native_OCCT()
    {
        var aWatch = Stopwatch.StartNew();
        var aFiles = new List<(string File, string What)>();
        foreach (string aFile in Tape.Budget(Directory.EnumerateFiles(DataOcc, "*.brep").Order(StringComparer.Ordinal), isSpread: true,
                                             theWeight: Tape.FileSize, theMaxWeight: 100_000))
        {
            if (HasTriangulations(aFile) && NativeCopy(aFile, false) is { } aCopy)
            {
                aFiles.Add((aCopy, Path.GetFileName(aFile)));
            }

            if (NativeCopy(aFile, true) is { } aMeshed)
            {
                aFiles.Add((aMeshed, Path.GetFileName(aFile) + " (meshed)"));
            }
        }

        string aDataset = Path.Combine(Metadata("OcctTestDataDir"), "brep");
        if (Directory.Exists(aDataset))
        {
            foreach (string aFile in Tape.Budget(Directory.EnumerateFiles(aDataset, "*.brep").Order(StringComparer.Ordinal), isSpread: true,
                                                 theWeight: Tape.FileSize, theMaxWeight: 100_000))
            {
                if (new FileInfo(aFile).Length <= 400_000 && HasTriangulations(aFile) && NativeCopy(aFile, false) is { } aCopy)
                {
                    aFiles.Add((aCopy, Path.GetFileName(aFile)));
                }
            }
        }

        foreach ((string aFile, string aWhat) in Tape.Budget(aFiles, isSpread: true))
        {
            try
            {
                CheckFile(aFile, aWhat);
            }
            catch (Exception anError) when (anError is DrawException || Refusal.IsFailure(anError))
            {
                Disagree($"{aWhat}: {Refusal.Unwrapped(anError).Message}");
            }
        }

        output.WriteLine($"{myShapes} shapes, {myValues} values ({myValues - myDisagreements.Count(d => d.Contains(" native "))} bit-identical), "
                         + $"{myTextsEqual} of {myTexts} BRep texts byte-identical (correctnormals, tclean); "
                         + $"{myMeshCheckIssues} shapes with free links, cross-face errors or async edges; "
                         + $"{myBothFailed} failed on both sides (bad node indices, correctnormals' null polygons); {aWatch.Elapsed.TotalSeconds:F0} s");
        Assert.Empty(myDisagreements);
    }
}
