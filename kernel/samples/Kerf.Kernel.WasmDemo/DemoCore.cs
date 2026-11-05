// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Exchange;
using Kerf.Geometry;
using Kerf.Measurement;
using Kerf.Meshing;
using Kerf.Topology;
using System.Diagnostics;
using System.Globalization;
using System.Runtime.InteropServices;
using System.Text;

namespace Kerf.Kernel.WasmDemo;

/// <summary>What one run gives: the timings, the counts, the buffers and a hash of each face's and edge's buffers.</summary>
public sealed class DemoResult
{
    public double ReadMs, MeshMs, Deflection;
    /// <summary>Always 0: the public API reads the buffers out as it meshes, within <see cref="MeshMs"/>. Kept for the page and the comparison.</summary>
    public double ExtractMs;
    public int Faces, Edges, Nodes, Triangles;
    public ShapeMesh Mesh = null!;
    public ulong[] FaceHashes = [], NormalHashes = [], EdgeHashes = [];
}

/// <summary>
/// The spike's work, the same in the browser and on win-x64 (the benchmark project links this file), on the public API
/// alone: read BRep text, mesh it at the viewer's tolerances (0.1% of the diagonal, 0.2 rad; parallel requested, which
/// the browser runs serially), and hash every face's and edge's buffers bit for bit.
/// </summary>
public static class DemoCore
{
    public const double Angle = 0.2;

    public static DemoResult Run(byte[] theBrep, double thePercent = 0.1)
    {
        var aResult = new DemoResult();
        long t0 = Stopwatch.GetTimestamp();
        Shape aShape = Brep.Read(new MemoryStream(theBrep)).Value;
        long t1 = Stopwatch.GetTimestamp();

        BoundingBox aBox = aShape.BoundingBox();
        double aDiagonal = aBox.IsEmpty ? 0.0 : aBox.Size.Magnitude;
        aResult.Deflection = aDiagonal > 1e-6 ? aDiagonal * thePercent / 100 : 1e-3;

        long t2 = Stopwatch.GetTimestamp();
        // Meshing includes reading the buffers out. A face of the file that already holds a mesh fine enough keeps it.
        ShapeMesh aMesh = aShape.Mesh(new MeshOptions { LinearDeflection = aResult.Deflection, AngularDeflection = Angle }).Value;
        long t3 = Stopwatch.GetTimestamp();

        aResult.ReadMs = Stopwatch.GetElapsedTime(t0, t1).TotalMilliseconds;
        aResult.MeshMs = Stopwatch.GetElapsedTime(t2, t3).TotalMilliseconds;
        aResult.Mesh = aMesh;
        aResult.Faces = aMesh.Faces.Count;
        aResult.Edges = aMesh.Edges.Count;
        aResult.Nodes = aMesh.NodeCount;
        aResult.Triangles = aMesh.TriangleCount;
        aResult.FaceHashes = new ulong[aResult.Faces];
        aResult.NormalHashes = new ulong[aResult.Faces];
        for (int f = 0; f < aResult.Faces; f++)
        {
            FaceMesh aFace = aMesh.Faces[f];
            int aKind = (int)aFace.Kind;
            // The mesh (nodes and triangles) and the normals apart, to attribute a difference
            ulong h = Hash(Basis, MemoryMarshal.AsBytes(aFace.Positions.Span));
            h = Hash(h, MemoryMarshal.AsBytes(aFace.Triangles.Span));
            aResult.NormalHashes[f] = Hash(Basis, MemoryMarshal.AsBytes(aFace.Normals.Span));
            aResult.FaceHashes[f] = Hash(h, MemoryMarshal.AsBytes(new ReadOnlySpan<int>(in aKind)));
        }
        aResult.EdgeHashes = new ulong[aResult.Edges];
        for (int e = 0; e < aResult.Edges; e++)
        {
            EdgePolyline anEdge = aMesh.Edges[e];
            int aKind = (int)anEdge.Kind;
            ulong h = Hash(Basis, MemoryMarshal.AsBytes(anEdge.Points.Span));
            aResult.EdgeHashes[e] = Hash(h, MemoryMarshal.AsBytes(new ReadOnlySpan<int>(in aKind)));
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
