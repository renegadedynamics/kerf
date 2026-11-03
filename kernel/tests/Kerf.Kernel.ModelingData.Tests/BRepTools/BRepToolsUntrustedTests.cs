// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using System.Diagnostics;
using System.Globalization;
using System.Text;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// Untrusted input (phase3-design.md, section 7) in the text format: the readers throw a Standard_Failure (for what
/// they can't read safely, Storage_StreamFormatError) on malformed input, and never hang, recurse without bound or
/// allocate more than the input could hold. Targeted cases, then every truncation and thousands of random mutations of
/// written shapes and of OCCT's data files.
/// </summary>
public class BRepToolsUntrustedTests
{
    private const string V3 = "\nCASCADE Topology V3, (c) Open Cascade\n";

    private const string NoGeometry = "Curve2ds 0\nCurves 0\nPolygon3D 0\nPolygonOnTriangulations 0\nSurfaces 0\nTriangulations 0\n";

    private static Shape Read(string theText) => BRepToolsIOTests.Read(theText);

    private static void Rejects(string theText)
    {
        var aWatch = Stopwatch.StartNew();
        Assert.Throws<StreamFormatErrorException>(() => Read(theText));
        Assert.True(aWatch.Elapsed < TimeSpan.FromSeconds(5), "too slow: " + aWatch.Elapsed);
    }

    private static string Str(long theValue) => theValue.ToString(CultureInfo.InvariantCulture);

    private const string Identity = "1 0 0 0 0 1 0 0 0 0 1 0\n";

    [Fact]
    public void Counts_beyond_the_input_are_rejected_before_allocating()
    {
        Rejects(V3 + "Locations 2000000000\n");
        Rejects(V3 + "Locations 0\nCurve2ds 0\nCurves 99999999\n");
        Rejects(V3 + "Locations 0\nCurve2ds 1000000\n");
        Rejects(V3 + "Locations 0\n" + NoGeometry + "\nTShapes 2000000000\n");
        Rejects(V3 + "Locations 0\nCurve2ds 0\nCurves 0\nPolygon3D 1\n2000000000 0\n0.1\n");
        Rejects(V3 + "Locations 0\nCurve2ds 0\nCurves 0\nPolygon3D 1\n-5 0\n0.1\n");
        Rejects(V3 + "Locations 0\nCurve2ds 0\nCurves 0\nPolygon3D 0\nPolygonOnTriangulations 1\n2000000000 1 2\n");
        Rejects(V3 + "Locations 0\nCurve2ds 0\nCurves 0\nPolygon3D 0\nPolygonOnTriangulations 0\nSurfaces 0\n"
                + "Triangulations 1\n2000000000 1 0 0 0\n");
        Rejects(V3 + "Locations 0\nCurve2ds 0\nCurves 0\nPolygon3D 0\nPolygonOnTriangulations 0\nSurfaces 0\n"
                + "Triangulations 1\n3 2000000000 0 0 0\n");

        // B-spline and Bezier geometry claiming more poles or knots than the input holds
        Rejects(V3 + "Locations 0\nCurve2ds 0\nCurves 1\n7 0 0 3 1000000000 2\n");
        Rejects(V3 + "Locations 0\nCurve2ds 0\nCurves 1\n7 0 0 3 4 1000000000\n");
        Rejects(V3 + "Locations 0\nCurve2ds 0\nCurves 1\n6 0 1000000000\n");
        Rejects(V3 + "Locations 0\nCurve2ds 1\n7 0 0 3 1000000000 2\n");
        Rejects(V3 + "Locations 0\nCurve2ds 0\nCurves 0\nPolygon3D 0\nPolygonOnTriangulations 0\nSurfaces 1\n8 0 0 30000 30000\n");
        Rejects(V3 + "Locations 0\nCurve2ds 0\nCurves 0\nPolygon3D 0\nPolygonOnTriangulations 0\nSurfaces 1\n"
                + "9 0 0 0 0 3 3 100000 100000 2 2\n");
    }

    [Fact]
    public void Counts_follow_the_declared_or_known_length_of_the_input()
    {
        string aText = V3 + "Locations 0\nCurve2ds 0\nCurves 1\n7 0 0 1 3000 2\n";
        // a reader of unknown length is assumed to hold UnknownInputLength characters, so 3000 poles are allowed (and
        // the curve can't be read: the stream ends)
        var aReader = new StringReader(aText);
        Assert.Equal(StreamUtilities.UnknownInputLength, StreamUtilities.InputLength(aReader));
        BrepTools.Read(out Shape aShape, aReader);
        Assert.True(aShape.IsNull);
        // a stream reader over a seekable stream knows its length
        var aStreamReader = new StreamReader(new MemoryStream(Encoding.ASCII.GetBytes(aText)));
        Assert.Equal(aText.Length, StreamUtilities.InputLength(aStreamReader));
        Assert.Throws<StreamFormatErrorException>(() => BrepTools.Read(out _, aStreamReader));
        Assert.Throws<ArgumentOutOfRangeException>(() => StreamUtilities.SetInputLength(new StringReader(""), -1));
    }

    [Fact]
    public void Words_longer_than_OCCTs_buffers_are_rejected()
    {
        Rejects(V3 + new string('L', 300) + " 0\n");
        Rejects(V3 + "Locations 0\n" + new string('C', 255) + " 0\n");
        Rejects(V3 + "Locations 0\n" + NoGeometry + "\nTShapes 1\nVe\n1e-07\n0 0 0\n0 0\n\n" + new string('1', 255) + "\n*\n\n+1 0 ");
        // 254 characters fit (OCCT's char[255] with its null)
        Assert.True(Read(V3 + "Locations 0\n" + NoGeometry + "\nTShapes 1\nVe\n1e-07\n0 0 0\n0 0\n\n" + new string('1', 254)
                         + "\n*\n\n+1 0 ").Free());
    }

    [Fact]
    public void Nested_geometry_is_capped()
    {
        string Trimmed(int theDepth)
        {
            var aText = new StringBuilder();
            for (int i = 1; i < theDepth; i++)
            {
                aText.Append("8 0 1\n");
            }

            return aText.Append("1 0 0 0 1 0 0\n").ToString();
        }

        string Curves(int theDepth) => V3 + "Locations 0\nCurve2ds 0\nCurves 1\n" + Trimmed(theDepth)
                                       + "Polygon3D 0\nPolygonOnTriangulations 0\nSurfaces 0\nTriangulations 0\n\nTShapes 0\n";

        Assert.True(Read(Curves(global::Kerf.Engine.Exchange.Brep.GeomTools.MaxGeometryNesting)).IsNull);
        Rejects(Curves(global::Kerf.Engine.Exchange.Brep.GeomTools.MaxGeometryNesting + 1));
        Rejects(Curves(100000));

        // offset surfaces of offset surfaces
        var aSurfaces = new StringBuilder();
        for (int i = 0; i < 200; i++)
        {
            aSurfaces.Append("11 1\n");
        }

        Rejects(V3 + "Locations 0\nCurve2ds 0\nCurves 0\nPolygon3D 0\nPolygonOnTriangulations 0\nSurfaces 1\n" + aSurfaces
                + "1 0 0 0 0 0 1 1 0 0 0 1 0\nTriangulations 0\n\nTShapes 0\n");

        // the static readers too
        Assert.Throws<StreamFormatErrorException>(() => global::Kerf.Engine.Exchange.Brep.CurveSet.ReadCurve(new StringReader(Trimmed(65))));
        Assert.NotNull(global::Kerf.Engine.Exchange.Brep.CurveSet.ReadCurve(new StringReader(Trimmed(64))));
    }

    // a vertex, then compounds each holding the one before
    private static string NestedCompounds(int theNbCompounds)
    {
        int aNb = theNbCompounds + 1;
        var aText = new StringBuilder(V3 + "Locations 0\n" + NoGeometry + "\nTShapes " + Str(aNb) + "\nVe\n1e-07\n0 0 0\n0 0\n\n1101101\n*\n");
        for (int i = 2; i <= aNb; i++)
        {
            aText.Append("Co\n\n1100000\n+" + Str(aNb - (i - 1) + 1) + " 0 *\n");
        }

        return aText.Append("\n+1 0 ").ToString();
    }

    [Fact]
    public void Nested_shapes_are_capped()
    {
        Shape aDeep = new();
        RunWithStack(() => aDeep = Read(NestedCompounds(global::Kerf.Engine.Exchange.Brep.ShapeSet.MaxShapeNesting - 1)), theStackSize: 4 << 20);
        Assert.Equal(ShapeType.Compound, aDeep.Type);
        // and can be written back: on 4 MB always; on 1 MB it depends on the JIT's frame sizes (tier-0 frames are
        // larger), so there the writer may stop with its clean InsufficientExecutionStackException, never a crash
        RunWithStack(() => BrepTools.Write(aDeep, new StringWriter()), theStackSize: 4 << 20);
        RunWithStack(() =>
        {
            try
            {
                BrepTools.Write(aDeep, new StringWriter());
            }
            catch (InsufficientExecutionStackException)
            {
            }
        });
        Rejects(NestedCompounds(global::Kerf.Engine.Exchange.Brep.ShapeSet.MaxShapeNesting));
        Rejects(NestedCompounds(20000));
    }

    [Fact]
    public void Writing_a_shape_nested_too_deeply_for_the_stack_throws()
    {
        // 100,000 nested compounds, built in code: the writer recurses once per level, and stops before the stack
        // overflows
        Shape aShape = TestShapes.Vertex(new Point3D());
        var aB = new BrepBuilder();
        for (int i = 0; i < 100000; i++)
        {
            Compound aC = new();
            aB.MakeCompound(ref aC);
            aB.Add(aC, aShape);
            aShape = aC;
        }

        RunWithStack(() => Assert.Throws<InsufficientExecutionStackException>(() => BrepTools.Write(aShape, new StringWriter())));
    }

    [Fact]
    public void Locations_are_bounded()
    {
        // two elementary locations: two datums, though both are the identity
        string aTwo = "Locations 2\n1\n" + Identity + "1\n" + Identity;
        // a complex location of two items raised to a large power
        Rejects(V3 + aTwo.Replace("Locations 2", "Locations 4", StringComparison.Ordinal) + "2 1 1 2 1 0\n2 3 1000000 0\n");
        Rejects(V3 + aTwo.Replace("Locations 2", "Locations 4", StringComparison.Ordinal) + "2 1 1 2 1 0\n2 3 -2147483648 0\n");
        // a long chain of factors
        var aChain = new StringBuilder("2");
        for (int i = 0; i < 1100; i++)
        {
            aChain.Append(i % 2 == 0 ? " 1 1" : " 2 1");
        }

        Rejects(V3 + aTwo.Replace("Locations 2", "Locations 3", StringComparison.Ordinal) + aChain + " 0\n");
        // an elementary location to any power is one item
        Assert.True(Read(V3 + aTwo.Replace("Locations 2", "Locations 3", StringComparison.Ordinal) + "2 1 1000000 0\n" + NoGeometry
                         + "\nTShapes 0\n").IsNull);
        // a reference to a location not read yet, and a chain that ends with the input (OCCT loops for ever)
        Rejects(V3 + aTwo.Replace("Locations 2", "Locations 3", StringComparison.Ordinal) + "2 7 1 0\n");
        Rejects(V3 + aTwo.Replace("Locations 2", "Locations 3", StringComparison.Ordinal) + "2 1 1 2");
    }

    [Fact]
    public void Lists_that_end_with_the_input_are_rejected_where_OCCT_loops()
    {
        string aShapes = V3 + "Locations 0\nCurve2ds 0\nCurves 1\n1 0 0 0 1 0 0\nPolygon3D 0\nPolygonOnTriangulations 0\n"
                         + "Surfaces 0\nTriangulations 0\n\nTShapes 1\n";
        // the representations of a vertex and of an edge
        Rejects(aShapes + "Ve\n1e-07\n0 0 0\n0.5 1 1 0\n0.5 1");
        Rejects(aShapes + "Ed\n 1e-07 1 1 0\n1  1 0 0 1\n1  1 0");
        // the sub-shapes of a shape
        Rejects(aShapes.Replace("TShapes 1", "TShapes 2", StringComparison.Ordinal)
                + "Ve\n1e-07\n0 0 0\n0 0\n\n1101101\n*\nCo\n\n1100000\n+2 0 +2 0 ");
    }

    [Fact]
    public void References_to_missing_shapes_and_triangulations_are_rejected()
    {
        string aPrefix = V3 + "Locations 0\n" + NoGeometry + "\nTShapes 2\nVe\n1e-07\n0 0 0\n0 0\n\n1101101\n*\n";
        // the compound refers to itself (index 1 from the end) and to a shape after it
        Rejects(aPrefix + "Co\n\n1100000\n+1 0 *\n\n+1 0 ");
        Rejects(aPrefix + "Co\n\n1100000\n+0 0 *\n\n+1 0 ");
        Rejects(aPrefix + "Co\n\n1100000\n+2 0 *\n\n+3 0 ");
        // a face made of a triangulation that isn't there
        Rejects(V3 + "Locations 0\n" + NoGeometry + "\nTShapes 1\nFa\n2  5\n\n1100000\n*\n\n+1 0 ");
        // a sub-shape of the wrong type is OCCT's TopoDS_UnCompatibleShapes
        Assert.Throws<IncompatibleShapesException>(
            () => Read(V3 + "Locations 0\n" + NoGeometry + "\nTShapes 2\nCo\n\n1100000\n*\nVe\n1e-07\n0 0 0\n0 0\n\n1101101\n+2 0 *\n\n+1 0 "));
    }

    // ---------------------------------------------------------------------------------------------------------------
    // Fuzzing

    private static void RunWithStack(Action theAction, TimeSpan? theTimeout = null, int theStackSize = 1 << 20)
    {
        // a thread with a 1 MB stack by default, as the readers will meet in callers' threads
        Exception? anError = null;
        var aThread = new Thread(() =>
        {
            try
            {
                theAction();
            }
            catch (Exception anException)
            {
                anError = anException;
            }
        }, theStackSize);
        aThread.Start();
        Assert.True(aThread.Join(theTimeout ?? TimeSpan.FromMinutes(5)), "the readers took too long (a hang?)");
        if (anError is not null)
        {
            throw new Xunit.Sdk.XunitException("unexpected exception: " + anError);
        }
    }

    // null if the text reads (and the shape read is frozen and writes) or fails with a Standard_Failure
    private static Exception? TryRead(string theText)
    {
        try
        {
            Shape aShape = Read(theText);
            if (!aShape.IsNull)
            {
                if (!aShape.TShape()!.IsFrozen)
                {
                    return new InvalidOperationException("a shape read isn't frozen");
                }

                try
                {
                    BrepTools.Write(aShape, new StringWriter());
                }
                catch (FailureException)
                {
                    // a shape OCCT can't write either (a null triangulation or polygon)
                }
            }

            return null;
        }
        catch (FailureException)
        {
            return null;
        }
        catch (Exception anException)
        {
            return anException;
        }
    }

    private static IEnumerable<(string Name, string Text)> Samples()
    {
        foreach ((string aName, Shape aShape) in BinTestShapes.All())
        {
            yield return (aName, BRepToolsIOTests.Write(aShape, global::Kerf.Engine.Exchange.Brep.FormatVersion.FormatVersion_VERSION_3, true, true));
        }

        yield return ("box, version 1", BRepToolsIOTests.Write(BinTestShapes.BoxWithRegularityAndPolygon(),
                                                                global::Kerf.Engine.Exchange.Brep.FormatVersion.FormatVersion_VERSION_1));
        yield return ("vertices, version 2", BRepToolsIOTests.Write(BinTestShapes.VertexRepresentations(true),
                                                                     global::Kerf.Engine.Exchange.Brep.FormatVersion.FormatVersion_VERSION_2));
    }

    private static IEnumerable<(string Name, string Text)> CorpusSamples()
    {
        // the smaller files of data/occ (both formats' every kind of geometry occurs in them)
        return Directory.GetFiles(GeomTools_CorpusTests.DataDirectory, "*.*")
                        .Where(f => new FileInfo(f).Length < 300_000)
                        .OrderBy(f => f, StringComparer.Ordinal)
                        .Select(f => (Path.GetFileName(f), File.ReadAllText(f, Encoding.Latin1)));
    }

    [Fact]
    public void Every_truncation_fails_cleanly()
    {
        int aNbReads = 0;
        RunWithStack(() =>
        {
            foreach ((string aName, string aText) in Samples())
            {
                for (int aLength = 0; aLength < aText.Length; aLength++)
                {
                    Exception? anUnexpected = TryRead(aText[..aLength]);
                    if (anUnexpected is not null)
                    {
                        throw new InvalidOperationException($"{aName} truncated to {aLength}: {anUnexpected}");
                    }

                    aNbReads++;
                }
            }

            var aRand = new Random(33013);
            foreach ((string aName, string aText) in CorpusSamples())
            {
                for (int aTrial = 0; aTrial < 100; aTrial++)
                {
                    int aLength = aRand.Next(aText.Length);
                    Exception? anUnexpected = TryRead(aText[..aLength]);
                    if (anUnexpected is not null)
                    {
                        throw new InvalidOperationException($"{aName} truncated to {aLength}: {anUnexpected}");
                    }

                    aNbReads++;
                }
            }
        });
        Assert.True(aNbReads > 30000, Str(aNbReads));
    }

    private static string Mutate(string theText, Random theRand)
    {
        var aText = new StringBuilder(theText);
        int aNbChanges = 1 + theRand.Next(4);
        for (int i = 0; i < aNbChanges && aText.Length > 0; i++)
        {
            int aPos = theRand.Next(aText.Length);
            switch (theRand.Next(7))
            {
                case 0:
                    aText[aPos] = "0123456789-+.e \n*ViCoFaEd"[theRand.Next(25)];
                    break;
                case 1:
                    aText.Remove(aPos, 1);
                    break;
                case 2:
                    aText.Insert(aPos, theRand.Next(2) == 0 ? "9999999" : "2147483647");
                    break;
                case 3:
                    aText.Insert(aPos, "-" + Str(theRand.Next(1, 100)) + " ");
                    break;
                case 4:
                {
                    // a run of the text repeated
                    int aLength = Math.Min(theRand.Next(1, 200), aText.Length - aPos);
                    aText.Insert(aPos, aText.ToString(aPos, aLength));
                }
                    break;
                case 5:
                {
                    // a run of the text removed
                    int aLength = Math.Min(theRand.Next(1, 200), aText.Length - aPos);
                    aText.Remove(aPos, aLength);
                }
                    break;
                default:
                    aText[aPos] = (char)theRand.Next(1, 256);
                    break;
            }
        }

        return aText.ToString();
    }

    [Fact]
    public void Random_mutations_fail_cleanly()
    {
        int aNbReads = 0;
        RunWithStack(() =>
        {
            var aRand = new Random(43013);
            foreach ((string aName, string aText) in Samples())
            {
                for (int aTrial = 0; aTrial < 2000; aTrial++)
                {
                    Exception? anUnexpected = TryRead(Mutate(aText, aRand));
                    if (anUnexpected is not null)
                    {
                        throw new InvalidOperationException($"{aName}, mutation {aTrial}: {anUnexpected}");
                    }

                    aNbReads++;
                }
            }

            foreach ((string aName, string aText) in CorpusSamples())
            {
                for (int aTrial = 0; aTrial < 60; aTrial++)
                {
                    Exception? anUnexpected = TryRead(Mutate(aText, aRand));
                    if (anUnexpected is not null)
                    {
                        throw new InvalidOperationException($"{aName}, mutation {aTrial}: {anUnexpected}");
                    }

                    aNbReads++;
                }
            }
        });
        Assert.True(aNbReads > 26000, Str(aNbReads));
    }
}
