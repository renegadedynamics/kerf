// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Globalization;
using System.Text.Json;
using Occt;
using OcctNet.WasmDemo;

namespace OcctNet.Benchmarks;

/// <summary>
/// The WebAssembly spike's comparison (argument "wasm-compare report.json data-folder"): runs the demo's own code
/// (DemoCore, linked from the sample) on win-x64 over the files the browser meshed, and compares every face's and
/// edge's buffer hash with the browser's; prints the timings side by side.
/// </summary>
internal static class WasmCompare
{
    private static readonly SortedDictionary<string, int> theTypes = new(StringComparer.Ordinal), theMeshDiff = new(StringComparer.Ordinal), theNormalDiff = new(StringComparer.Ordinal);

    public static void Run(string theReport, string theData)
    {
        using JsonDocument aDoc = JsonDocument.Parse(File.ReadAllText(theReport));
        JsonElement aRoot = aDoc.RootElement;
        Console.WriteLine($"Browser: {aRoot.GetProperty("userAgent").GetString()}; .NET started in {aRoot.GetProperty("startupMs").GetDouble():F0} ms; WebGL 2: {aRoot.GetProperty("webgl").GetBoolean()}");
        Console.WriteLine();
        Console.WriteLine("| file | faces | nodes | wasm read | wasm mesh | wasm extract | wasm warm mesh+extract | x64 read | x64 mesh | x64 extract | wasm/x64 (mesh+extract) | faces bit-identical | edges bit-identical |");
        Console.WriteLine("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|");
        var aWarm = new Dictionary<string, double>(StringComparer.Ordinal);
        foreach (JsonElement aRun in aRoot.GetProperty("runs").EnumerateArray())
        {
            if (aRun.GetProperty("pass").GetString() == "warm" && aRun.GetProperty("ok").GetBoolean())
            {
                aWarm[aRun.GetProperty("file").GetString()!] = aRun.GetProperty("meshMs").GetDouble() + aRun.GetProperty("extractMs").GetDouble();
            }
        }
        int aFaces = 0, aFacesSame = 0, anEdges = 0, anEdgesSame = 0, aFiles = 0, aFilesSame = 0, aFailed = 0;
        foreach (JsonElement aRun in aRoot.GetProperty("runs").EnumerateArray())
        {
            if (aRun.GetProperty("pass").GetString() != "cold")
            {
                continue;
            }
            string aName = aRun.GetProperty("file").GetString()!;
            if (!aRun.GetProperty("ok").GetBoolean())
            {
                aFailed++;
                Console.WriteLine($"| {aName} | failed in the browser: {aRun.GetProperty("error").GetString()} |");
                continue;
            }
            byte[] aBytes = File.ReadAllBytes(Path.Combine(theData, aName));
            DemoCore.Run(aBytes); // warm the JIT
            DemoResult aX64 = DemoCore.Run(aBytes);
            string[] aWasmFaces = aRun.GetProperty("faceHashes").EnumerateArray().Select(e => e.GetString()!).ToArray();
            string[] aWasmEdges = aRun.GetProperty("edgeHashes").EnumerateArray().Select(e => e.GetString()!).ToArray();
            string[] aWasmNormals = aRun.TryGetProperty("normalHashes", out JsonElement aNormalsElement)
                ? aNormalsElement.EnumerateArray().Select(e => e.GetString()!).ToArray() : [];
            if (aWasmFaces.Length == aX64.FaceHashes.Length)
            {
                for (int f = 0; f < aWasmFaces.Length; f++)
                {
                    bool isMeshSame = aWasmFaces[f] == aX64.FaceHashes[f].ToString("x16", CultureInfo.InvariantCulture);
                    bool isNormalSame = aWasmNormals.Length == aWasmFaces.Length
                                        && aWasmNormals[f] == aX64.NormalHashes[f].ToString("x16", CultureInfo.InvariantCulture);
                    string aKey = ((GeomAbs_SurfaceType)aX64.Buffers.FaceTypes[f]).ToString().Replace("GeomAbs_", "", StringComparison.Ordinal);
                    theTypes[aKey] = theTypes.GetValueOrDefault(aKey) + 1;
                    if (!isMeshSame)
                    {
                        theMeshDiff[aKey] = theMeshDiff.GetValueOrDefault(aKey) + 1;
                    }
                    else if (!isNormalSame)
                    {
                        theNormalDiff[aKey] = theNormalDiff.GetValueOrDefault(aKey) + 1;
                    }
                }
            }
            int aSameFaces = aWasmFaces.Length == aX64.FaceHashes.Length
                ? aWasmFaces.Where((h, i) => h == aX64.FaceHashes[i].ToString("x16", CultureInfo.InvariantCulture)
                                             && (aWasmNormals.Length == 0 || aWasmNormals[i] == aX64.NormalHashes[i].ToString("x16", CultureInfo.InvariantCulture))).Count() : 0;
            int aSameEdges = aWasmEdges.Length == aX64.EdgeHashes.Length
                ? aWasmEdges.Where((h, i) => h == aX64.EdgeHashes[i].ToString("x16", CultureInfo.InvariantCulture)).Count() : 0;
            aFiles++;
            aFaces += aX64.Faces;
            anEdges += aX64.Edges;
            aFacesSame += aSameFaces;
            anEdgesSame += aSameEdges;
            if (aSameFaces == aX64.Faces && aSameEdges == aX64.Edges)
            {
                aFilesSame++;
            }
            double aWasm = aRun.GetProperty("meshMs").GetDouble() + aRun.GetProperty("extractMs").GetDouble();
            Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
                $"| {aName} | {aX64.Faces} | {aX64.Nodes} | {aRun.GetProperty("readMs").GetDouble():F0} | {aRun.GetProperty("meshMs").GetDouble():F0} "
                + $"| {aRun.GetProperty("extractMs").GetDouble():F0} | {(aWarm.TryGetValue(aName, out double w) ? w.ToString("F0", CultureInfo.InvariantCulture) : "")} "
                + $"| {aX64.ReadMs:F1} | {aX64.MeshMs:F1} | {aX64.ExtractMs:F1} | {(aWarm.TryGetValue(aName, out double w2) ? w2 : aWasm) / (aX64.MeshMs + aX64.ExtractMs):F0}× "
                + $"| {aSameFaces}/{aX64.Faces} | {aSameEdges}/{aX64.Edges} |"));
        }
        Console.WriteLine();
        Console.WriteLine(string.Create(CultureInfo.InvariantCulture,
            $"{aFilesSame}/{aFiles} files bit-identical to win-x64; faces {aFacesSame}/{aFaces} ({(double)aFacesSame / Math.Max(1, aFaces):P2}), "
            + $"edges {anEdgesSame}/{anEdges} ({(double)anEdgesSame / Math.Max(1, anEdges):P2}); {aFailed} failed in the browser"));
        Console.WriteLine("Faces by surface type: total, then those whose nodes or triangles differ, then those with only the normals differing:");
        foreach ((string aType, int aCount) in theTypes)
        {
            Console.WriteLine($"  {aType}: {aCount}, mesh differs {theMeshDiff.GetValueOrDefault(aType)}, normals only {theNormalDiff.GetValueOrDefault(aType)}");
        }
    }
}
