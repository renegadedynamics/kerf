// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Meshing;
using System.Runtime.InteropServices;
using System.Runtime.InteropServices.JavaScript;
using System.Runtime.Versioning;

[assembly: SupportedOSPlatform("browser")]

Console.WriteLine("OCCT.NET WebAssembly demo ready.");

namespace Kerf.Kernel.WasmDemo
{
    /// <summary>
    /// What the page's script calls: mesh a .brep's bytes (returns the run as JSON), then fetch the last run's buffers.
    /// JavaScript interop marshals byte, int and double arrays, so the float buffers go as their bytes.
    /// </summary>
    public static partial class Interop
    {
        private static DemoResult? theLast;

        /// <summary>Reads, meshes and extracts; the timings, counts and hashes as JSON.</summary>
        [JSExport]
        public static string Mesh(string theName, byte[] theBrep, double thePercent)
        {
            try
            {
                theLast = DemoCore.Run(theBrep, thePercent);
                return DemoCore.Json(theName, theLast);
            }
            catch (Exception anException)
            {
                theLast = null;
                return DemoCore.Json(theName, null, anException.GetType().Name + ": " + anException.Message);
            }
        }

        private static ShapeMesh Last => theLast?.Mesh ?? throw new InvalidOperationException("Nothing meshed yet.");

        [JSExport]
        public static byte[] Positions() => MemoryMarshal.AsBytes(Last.Positions.Span).ToArray();

        [JSExport]
        public static byte[] Normals() => MemoryMarshal.AsBytes(Last.Normals.Span).ToArray();

        /// <summary>The triangles with each face's node offset added: indices into all the nodes.</summary>
        [JSExport]
        public static int[] Triangles()
        {
            ShapeMesh aMesh = Last;
            ReadOnlySpan<int> aTriangles = aMesh.Triangles.Span;
            var anIndices = new int[aTriangles.Length];
            foreach (FaceMesh aFace in aMesh.Faces)
            {
                int anEnd = 3 * (aFace.FirstTriangle + aFace.TriangleCount);
                for (int t = 3 * aFace.FirstTriangle; t < anEnd; t++)
                {
                    anIndices[t] = aTriangles[t] + aFace.FirstNode;
                }
            }
            return anIndices;
        }

        [JSExport]
        public static byte[] Points() => MemoryMarshal.AsBytes(Last.EdgePoints.Span).ToArray();

        /// <summary>Each edge's first point, then the total.</summary>
        [JSExport]
        public static int[] EdgePoints()
        {
            IReadOnlyList<EdgePolyline> anEdges = Last.Edges;
            var anOffsets = new int[anEdges.Count + 1];
            for (int e = 0; e < anEdges.Count; e++)
            {
                anOffsets[e + 1] = anOffsets[e] + anEdges[e].PointCount;
            }
            return anOffsets;
        }
    }
}
