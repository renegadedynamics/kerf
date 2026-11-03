// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Exchange.Stl.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using System.Buffers.Binary;
using System.Globalization;
using System.Text;

namespace Kerf.Kernel.DataExchange.Tests;

// STL read as OCCT's file reader reads it, from streams too, and every domain of a multi-domain ASCII file
// (occt-issues.md, RWStl::ReadStream and RWStl_Reader::Read; Kerf unit 10K3, decisions 16815 and 16816).
public class RWStlFixedBugsTests
{
    // a triangle per facet: (i, 0, z), (i + 1, 0, z), (i, 1, z)
    private static string AsciiSolid(string theName, int theFacets, double theZ)
    {
        var aText = new StringBuilder("solid " + theName + "\n");
        for (int i = 0; i < theFacets; i++)
        {
            aText.Append(" facet normal 0 0 1\n  outer loop\n");
            foreach ((double x, double y) in new[] { (i, 0.0), (i + 1, 0.0), (i, 1.0) })
            {
                aText.Append(string.Create(CultureInfo.InvariantCulture, $"   vertex {x} {y} {theZ}\n"));
            }
            aText.Append("  endloop\n endfacet\n");
        }
        return aText.Append("endsolid " + theName + "\n").ToString();
    }

    private static byte[] Binary(string theHeader, int theFacets)
    {
        byte[] aData = new byte[84 + 50 * theFacets];
        Encoding.ASCII.GetBytes(theHeader).CopyTo(aData, 0);
        BinaryPrimitives.WriteUInt32LittleEndian(aData.AsSpan(80), (uint)theFacets);
        for (int i = 0; i < theFacets; i++)
        {
            Span<byte> aFacet = aData.AsSpan(84 + 50 * i, 50);
            float[] aValues = [0, 0, 1, i, 0, 0, i + 1, 0, 0, i, 1, 0];
            for (int k = 0; k < aValues.Length; k++)
            {
                BinaryPrimitives.WriteSingleLittleEndian(aFacet[(4 * k)..], aValues[k]);
            }
        }
        return aData;
    }

    [Fact]
    public void A_binary_stream_whose_header_starts_with_solid_reads()
    {
        byte[] aData = Binary("solid exported by some tool", 3);
        Triangulation? aRead = global::Kerf.Engine.Exchange.Stl.RWStl.ReadStream(new MemoryStream(aData));
        Assert.NotNull(aRead);
        Assert.Equal(3, aRead.TriangleCount);
        Triangulation? aBinary = global::Kerf.Engine.Exchange.Stl.RWStl.ReadBinaryStream(new MemoryStream(aData));
        Assert.Equal(aBinary!.NodeCount, aRead.NodeCount);

        Shape aShape = new();
        Assert.True(new StlApiReader().Read(ref aShape, new MemoryStream(aData)));
        Assert.False(aShape.IsNull);
    }

    [Fact]
    public void An_ascii_stream_in_capitals_reads()
    {
        byte[] aData = Encoding.ASCII.GetBytes(AsciiSolid("A", 4, 0.0).Replace("solid", "SOLID", StringComparison.Ordinal)
                                                                   .Replace("facet", "FACET", StringComparison.Ordinal));
        Triangulation? aRead = global::Kerf.Engine.Exchange.Stl.RWStl.ReadStream(new MemoryStream(aData));
        Assert.NotNull(aRead);
        Assert.Equal(4, aRead.TriangleCount);
    }

    private static List<int> Domains(string theText)
    {
        var aList = new NCollection_Sequence<Triangulation>();
        global::Kerf.Engine.Exchange.Stl.RWStl.ReadFile(new MemoryStream(Encoding.ASCII.GetBytes(theText)), Math.PI / 2.0, aList);
        var aCounts = new List<int>();
        for (int i = 1; i <= aList.Length(); i++)
        {
            aCounts.Add(aList.Value(i).TriangleCount);
        }
        return aCounts;
    }

    [Fact]
    public void Every_domain_of_a_small_ascii_file_is_read()
    {
        // under the reader's 1 KB block: OCCT read the first domain only (twosolids.stl)
        string aText = AsciiSolid("a", 2, 0.0) + AsciiSolid("b", 1, 5.0);
        Assert.True(aText.Length < 1024);
        Assert.Equal([2, 1], Domains(aText));
    }

    [Fact]
    public void Domains_are_read_whatever_falls_at_a_block_boundary()
    {
        // the second and third domains' starts and the blanks between them move across the 1 KB blocks' ends
        for (int aPad = 0; aPad < 80; aPad++)
        {
            string aText = AsciiSolid("a", 7, 0.0) + new string(' ', aPad) + "\n" + AsciiSolid("b", 3, 12.5)
                           + new string('\n', aPad % 3) + AsciiSolid("c", 9, -7.25);
            Assert.Equal([7, 3, 9], Domains(aText));
        }
    }
}
