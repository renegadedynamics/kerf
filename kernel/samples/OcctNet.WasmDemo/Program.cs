// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using System.Runtime.InteropServices;
using System.Runtime.InteropServices.JavaScript;
using System.Runtime.Versioning;

[assembly: SupportedOSPlatform("browser")]

Console.WriteLine("OCCT.NET WebAssembly demo ready.");

namespace OcctNet.WasmDemo
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

        private static global::Kerf.Engine.Meshing.MeshBuffers Last => theLast?.Buffers ?? throw new InvalidOperationException("Nothing meshed yet.");

        [JSExport]
        public static byte[] Positions() => MemoryMarshal.AsBytes(Last.Positions.AsSpan()).ToArray();

        [JSExport]
        public static byte[] Normals() => MemoryMarshal.AsBytes(Last.Normals.AsSpan()).ToArray();

        /// <summary>The triangles with each face's node offset added: indices into all the nodes.</summary>
        [JSExport]
        public static int[] Triangles()
        {
            global::Kerf.Engine.Meshing.MeshBuffers aBuffers = Last;
            var anIndices = new int[aBuffers.Triangles.Length];
            for (int f = 0; f < aBuffers.FaceTypes.Length; f++)
            {
                int anOffset = (int)aBuffers.FaceNodes[f];
                for (long t = aBuffers.FaceTriangles[f]; t < aBuffers.FaceTriangles[f + 1]; t++)
                {
                    anIndices[t] = aBuffers.Triangles[t] + anOffset;
                }
            }
            return anIndices;
        }

        [JSExport]
        public static byte[] Points() => MemoryMarshal.AsBytes(Last.Points.AsSpan()).ToArray();

        [JSExport]
        public static int[] EdgePoints() => Last.EdgePoints.Select(p => (int)p).ToArray();
    }
}
