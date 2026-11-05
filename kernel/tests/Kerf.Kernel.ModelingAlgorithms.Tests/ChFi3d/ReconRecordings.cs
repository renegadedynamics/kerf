// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Legacy.Data;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using System.Globalization;
using System.IO.Compression;
using System.Text;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// The recordings of <c>tools/dsrec --recon</c> (unit 7R): for each case (a native fillet or chamfer), the input as
/// BRep text, native's data structure as <c>ChFi3d_Builder::Compute</c> hands it to the HBuilder's <c>Perform</c>
/// (<see cref="TextDump"/>'s format), and native's result and history after <c>Build</c>: the result as
/// BRep text, the faces of each surface (<c>Builder()->NewFaces</c>), the lengths of the edges of each curve
/// (<c>Builder()->NewEdges</c>), <c>IsDeleted</c> and <c>Modified</c> of each
/// input face, <c>Generated</c> of each input edge and vertex, as indices in <c>TopExp::MapShapes(result, FACE)</c>.
/// </summary>
/// <remarks>
/// The cases: tools/dsrec's 15 (dsrec.txt), tests/blend/simple's A2 to D4 (Draw's <c>explode s E</c> numbers edges as
/// <c>TopExp::MapShapes</c>) and more chamfers. The edges and radii are in tools/dsrec/main.cpp and recon.inc.
/// </remarks>
internal static class ReconRecordings
{
    private static readonly Lazy<Dictionary<string, ReconCase>> theCases = new(Load);

    /// <summary>The cases by name, in the recording's order.</summary>
    internal static Dictionary<string, ReconCase> Cases => theCases.Value;

    private static int Int(string s) => int.Parse(s, CultureInfo.InvariantCulture);

    private static Dictionary<string, ReconCase> Load()
    {
        using Stream aRes = typeof(ReconRecordings).Assembly.GetManifestResourceStream("ChFi3d.reconrec.txt.gz")
                            ?? throw new InvalidOperationException("ChFi3d.reconrec.txt.gz");
        using GZipStream aGz = new(aRes, CompressionMode.Decompress);
        using StreamReader aReader = new(aGz, Encoding.ASCII);
        string[] aLines = aReader.ReadToEnd().Replace("\r\n", "\n").Split('\n');

        Dictionary<string, ReconCase> aCases = new();
        int k = 0;
        string Block(string theTag)
        {
            string aHead = aLines[k++];
            if (!aHead.StartsWith(theTag + " ", StringComparison.Ordinal))
            {
                throw new InvalidDataException("reconrec: " + theTag + " expected, " + aHead);
            }
            int n = Int(aHead[(theTag.Length + 1)..]);
            string aText = n == 0 ? "" : string.Join("\n", aLines, k, n) + "\n";
            k += n;
            return aText;
        }
        static int[] Faces(string[] theWords, int theAt)
        {
            int n = Int(theWords[theAt]);
            int[] r = new int[n];
            for (int i = 0; i < n; i++)
            {
                r[i] = Int(theWords[theAt + 1 + i]);
            }
            return r;
        }
        while (k < aLines.Length && aLines[k].StartsWith("case ", StringComparison.Ordinal))
        {
            string aName = aLines[k++][5..];
            string[] aStatus = aLines[k++].Split(' ');
            bool aDone = aStatus[2] == "1";
            int aCalls = Int(aStatus[4]);
            if (aCalls == 0)
            {
                k++; // end-case
                continue;
            }
            string aBRep = Block("brep");
            string aDs = Block("ds");
            string aResult = Block("result");
            List<int[]> aNewFaces = [], aModified = [], aGenE = [], aGenV = [];
            List<bool> aDeleted = [];
            List<double[]> aNewEdges = [];
            List<(bool Deleted, int[] Modified)> aSolids = [];
            if (aResult.Length > 0)
            {
                string[] h = aLines[k++].Split(' ');
                int nS = Int(h[1]), nF = Int(h[2]), nE = Int(h[3]), nV = Int(h[4]), nC = Int(h[5]);
                for (int i = 0; i < nS; i++)
                {
                    aNewFaces.Add(Faces(aLines[k++].Split(' '), 2));
                }
                for (int i = 0; i < nC; i++)
                {
                    string[] w = aLines[k++].Split(' ');
                    aNewEdges.Add(w.Skip(3).Select(x => double.Parse(x, CultureInfo.InvariantCulture)).ToArray());
                }
                for (int i = 0; i < nF; i++)
                {
                    string[] w = aLines[k++].Split(' ');
                    aDeleted.Add(w[2] == "1");
                    aModified.Add(Faces(w, 3));
                }
                for (int i = 0; i < nE; i++)
                {
                    aGenE.Add(Faces(aLines[k++].Split(' '), 3));
                }
                for (int i = 0; i < nV; i++)
                {
                    aGenV.Add(Faces(aLines[k++].Split(' '), 3));
                }
                while (aLines[k].StartsWith("msolid ", StringComparison.Ordinal))
                {
                    string[] w = aLines[k++].Split(' ');
                    aSolids.Add((w[2] == "1", Faces(w, 3)));
                }
            }
            if (aLines[k++] != "end-case")
            {
                throw new InvalidDataException("reconrec: end-case expected after " + aName);
            }
            aCases[aName] = new ReconCase(aName, aDone, aBRep, aDs, aResult, aNewFaces, aNewEdges, aDeleted, aModified,
                                          aGenE, aGenV, aSolids);
        }
        return aCases;
    }
}

/// <summary>A case of <see cref="ReconRecordings"/>.</summary>
internal sealed record ReconCase(string Name, bool Done, string BRepText, string DsText, string ResultText,
                                 List<int[]> NewFaces, List<double[]> NewEdges, List<bool> Deleted,
                                 List<int[]> Modified,
                                 List<int[]> GeneratedFromEdges, List<int[]> GeneratedFromVertices,
                                 List<(bool Deleted, int[] Modified)> Solids)
{
    /// <summary>The input shape (a new one at each call).</summary>
    internal Shape Input
    {
        get
        {
            BrepTools.Read(out Shape aShape, new StringReader(BRepText));
            return aShape;
        }
    }

    /// <summary>Native's result (a new one at each call), null if native's maker was not done.</summary>
    internal Shape? Result
    {
        get
        {
            if (ResultText.Length == 0)
            {
                return null;
            }
            BrepTools.Read(out Shape aShape, new StringReader(ResultText));
            return aShape;
        }
    }
}
