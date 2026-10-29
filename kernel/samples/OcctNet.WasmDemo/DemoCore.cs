// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Diagnostics;
using System.Globalization;
using System.Runtime.InteropServices;
using System.Text;
using Occt;

namespace OcctNet.WasmDemo;

/// <summary>What one run gives: the timings, the counts, the buffers and a hash of each face's and edge's buffers.</summary>
public sealed class DemoResult
{
    public double ReadMs, MeshMs, ExtractMs, Deflection;
    public int Faces, Edges, Nodes, Triangles;
    public MeshBuffers Buffers = null!;
    public ulong[] FaceHashes = [], NormalHashes = [], EdgeHashes = [];
}

/// <summary>
/// The spike's work, the same in the browser and on win-x64 (the benchmark project links this file): read BRep text,
/// mesh it at the viewer's tolerances (0.1% of the diagonal, 0.2 rad; parallel requested, which the browser runs
/// serially), extract, and hash every face's and edge's buffers bit for bit.
/// </summary>
public static class DemoCore
{
    public const double Angle = 0.2;

    public static DemoResult Run(byte[] theBrep, double thePercent = 0.1)
    {
        var aResult = new DemoResult();
        long t0 = Stopwatch.GetTimestamp();
        TopoDS_Shape aShape;
        using (var aReader = new StreamReader(new MemoryStream(theBrep), Encoding.Latin1, false))
        {
            BRepTools.Read(out aShape, aReader);
        }
        long t1 = Stopwatch.GetTimestamp();
        // Some data/occ files carry meshes: meshed afresh, as a shape built in Bento would be
        aShape = BRepTools.Cleaned(aShape);

        var aBox = new Bnd_Box();
        BRepBndLib.Add(aShape, ref aBox, true);
        double aDiagonal = aBox.IsVoid() ? 0.0 : Math.Sqrt(aBox.SquareExtent());
        aResult.Deflection = aDiagonal > 1e-6 ? aDiagonal * thePercent / 100 : 1e-3;

        long t2 = Stopwatch.GetTimestamp();
        TopoDS_Shape aMeshed = new BRepMesh_IncrementalMesh(aShape, aResult.Deflection, false, Angle, true).Shape();
        long t3 = Stopwatch.GetTimestamp();
        MeshBuffers aBuffers = MeshExtraction.Extract(aMeshed, aResult.Deflection, Angle);
        long t4 = Stopwatch.GetTimestamp();

        aResult.ReadMs = Stopwatch.GetElapsedTime(t0, t1).TotalMilliseconds;
        aResult.MeshMs = Stopwatch.GetElapsedTime(t2, t3).TotalMilliseconds;
        aResult.ExtractMs = Stopwatch.GetElapsedTime(t3, t4).TotalMilliseconds;
        aResult.Buffers = aBuffers;
        aResult.Faces = aBuffers.FaceTypes.Length;
        aResult.Edges = aBuffers.EdgeTypes.Length;
        aResult.Nodes = (int)aBuffers.FaceNodes[^1];
        aResult.Triangles = (int)(aBuffers.FaceTriangles[^1] / 3);
        aResult.FaceHashes = new ulong[aResult.Faces];
        aResult.NormalHashes = new ulong[aResult.Faces];
        for (int f = 0; f < aResult.Faces; f++)
        {
            int n0 = (int)aBuffers.FaceNodes[f], n1 = (int)aBuffers.FaceNodes[f + 1];
            int t = (int)aBuffers.FaceTriangles[f], tEnd = (int)aBuffers.FaceTriangles[f + 1];
            // The mesh (nodes and triangles) and the normals apart, to attribute a difference
            ulong h = Hash(Basis, MemoryMarshal.AsBytes(aBuffers.Positions.AsSpan(3 * n0, 3 * (n1 - n0))));
            h = Hash(h, MemoryMarshal.AsBytes(aBuffers.Triangles.AsSpan(t, tEnd - t)));
            aResult.NormalHashes[f] = Hash(Basis, MemoryMarshal.AsBytes(aBuffers.Normals.AsSpan(3 * n0, 3 * (n1 - n0))));
            aResult.FaceHashes[f] = Hash(h, MemoryMarshal.AsBytes(aBuffers.FaceTypes.AsSpan(f, 1)));
        }
        aResult.EdgeHashes = new ulong[aResult.Edges];
        for (int e = 0; e < aResult.Edges; e++)
        {
            int p0 = (int)aBuffers.EdgePoints[e], p1 = (int)aBuffers.EdgePoints[e + 1];
            ulong h = Hash(Basis, MemoryMarshal.AsBytes(aBuffers.Points.AsSpan(3 * p0, 3 * (p1 - p0))));
            aResult.EdgeHashes[e] = Hash(h, MemoryMarshal.AsBytes(aBuffers.EdgeTypes.AsSpan(e, 1)));
        }
        return aResult;
    }

    private const ulong Basis = 14695981039346656037UL;

    /// <summary>FNV-1a over 8-byte words (then the tail's bytes), with the length mixed in.</summary>
    private static ulong Hash(ulong theHash, ReadOnlySpan<byte> theBytes)
    {
        const ulong Prime = 1099511628211UL;
        ReadOnlySpan<ulong> aWords = MemoryMarshal.Cast<byte, ulong>(theBytes);
        ulong h = (theHash ^ (ulong)theBytes.Length) * Prime;
        foreach (ulong w in aWords)
        {
            h = (h ^ w) * Prime;
        }
        for (int i = 8 * aWords.Length; i < theBytes.Length; i++)
        {
            h = (h ^ theBytes[i]) * Prime;
        }
        return h;
    }

    /// <summary>The run as JSON (written by hand: no reflection, so nothing for the trimmer to keep).</summary>
    public static string Json(string theName, DemoResult? theResult, string? theError = null)
    {
        var aText = new StringBuilder();
        aText.Append(CultureInfo.InvariantCulture, $"{{\"file\":\"{theName}\",\"ok\":{(theResult is null ? "false" : "true")}");
        if (theError is not null)
        {
            aText.Append(",\"error\":\"").Append(theError.Replace("\\", "\\\\", StringComparison.Ordinal).Replace("\"", "'", StringComparison.Ordinal)
                                                         .Replace("\n", " ", StringComparison.Ordinal).Replace("\r", " ", StringComparison.Ordinal)).Append('"');
        }
        if (theResult is not null)
        {
            aText.Append(CultureInfo.InvariantCulture,
                $",\"readMs\":{theResult.ReadMs:F2},\"meshMs\":{theResult.MeshMs:F2},\"extractMs\":{theResult.ExtractMs:F2},\"deflection\":{theResult.Deflection:R}");
            aText.Append(CultureInfo.InvariantCulture,
                $",\"faces\":{theResult.Faces},\"edges\":{theResult.Edges},\"nodes\":{theResult.Nodes},\"triangles\":{theResult.Triangles}");
            aText.Append(",\"faceHashes\":[").AppendJoin(',', theResult.FaceHashes.Select(h => "\"" + h.ToString("x16", CultureInfo.InvariantCulture) + "\""))
                 .Append("],\"normalHashes\":[").AppendJoin(',', theResult.NormalHashes.Select(h => "\"" + h.ToString("x16", CultureInfo.InvariantCulture) + "\""))
                 .Append("],\"edgeHashes\":[").AppendJoin(',', theResult.EdgeHashes.Select(h => "\"" + h.ToString("x16", CultureInfo.InvariantCulture) + "\""))
                 .Append(']');
        }
        return aText.Append('}').ToString();
    }
}
