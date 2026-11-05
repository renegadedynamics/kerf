// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using System.Buffers.Binary;
using System.Diagnostics;
using System.Text;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// Untrusted input (phase3-design.md, section 7): the binary readers throw a Standard_Failure (for what they can't
/// read safely, Storage_StreamFormatError) on malformed input, and never hang, recurse without bound or allocate more
/// than the input could hold. Targeted cases, then every truncation and thousands of random mutations of real files.
/// </summary>
public class BinToolsUntrustedTests
{
    private const string V4 = "\nOpen CASCADE Topology V4, (c) Open Cascade\n";

    private static byte[] Ascii(string theText) => Encoding.ASCII.GetBytes(theText);

    private static byte[] Concat(params byte[][] theParts) => theParts.SelectMany(thePart => thePart).ToArray();

    private static byte[] Int(int theValue)
    {
        byte[] aBuf = new byte[4];
        BinaryPrimitives.WriteInt32LittleEndian(aBuf, theValue);
        return aBuf;
    }

    private static byte[] Real(double theValue)
    {
        byte[] aBuf = new byte[8];
        BinaryPrimitives.WriteDoubleLittleEndian(aBuf, theValue);
        return aBuf;
    }

    private static byte[] Identity() => Concat(Real(1), Real(0), Real(0), Real(0), Real(0), Real(1), Real(0), Real(0), Real(0), Real(0), Real(1), Real(0));

    private static void ReadSet(byte[] theBytes)
    {
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Read(out Shape _, new MemoryStream(theBytes));
    }

    private static void ReadPositional(byte[] theBytes)
    {
        Shape aShape = new();
        new global::Kerf.Engine.Exchange.Brep.Binary.ShapeReader().Read(new MemoryStream(theBytes), ref aShape);
    }

    [Fact]
    public void Counts_beyond_the_input_are_rejected_before_allocating()
    {
        var aWatch = Stopwatch.StartNew();
        EngineAssert.Throws<StreamFormatErrorException>(() => ReadSet(Ascii(V4 + "Locations 2000000000\n")));
        EngineAssert.Throws<StreamFormatErrorException>(() => ReadSet(Ascii(V4 + "Locations -5\n")));
        EngineAssert.Throws<StreamFormatErrorException>(() => ReadSet(Ascii(V4 + "Locations 99999999999\n")));
        EngineAssert.Throws<StreamFormatErrorException>(() => ReadSet(Ascii(V4 + "Locations 0\nCurve2ds 0\nCurves 1000000\n")));

        // a B-spline curve claiming 2^30 poles
        byte[] aBSpline = Concat(Ascii("Curves 1\n"), [7, 0, 0, 3, 0], Int(1 << 30), Int(2));
        EngineAssert.Throws<StreamFormatErrorException>(() => new global::Kerf.Engine.Exchange.Brep.Binary.BinaryCurveSet().Read(new MemoryStream(aBSpline)));

        // a Bezier surface of degrees 65535 x 65535
        byte[] aBezier = Concat(Ascii("Surfaces 1\n"), [8, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF]);
        EngineAssert.Throws<StreamFormatErrorException>(() => new global::Kerf.Engine.Exchange.Brep.Binary.BinarySurfaceSet().Read(new MemoryStream(aBezier)));

        // a triangulation claiming 2^30 nodes, in the table-based and the position-based formats
        byte[] aTriangulations = Concat(Ascii("Triangulations 1\n"), Int(1 << 30), Int(1), [0, 0], Real(0));
        EngineAssert.Throws<StreamFormatErrorException>(() => new global::Kerf.Engine.Exchange.Brep.Binary.BinaryShapeSet().ReadTriangulation(new MemoryStream(aTriangulations)));
        byte[] aPositional = Concat([200 + 4 * 4, 7, 0], Real(0), [14, 7, 2, 19], Int(1 << 30), Int(1), [0, 0], Real(0));
        EngineAssert.Throws<StreamFormatErrorException>(() => ReadPositional(aPositional));

        // a non-seekable stream is bounded too
        byte[] aPolygons = Concat(Ascii("Polygon3D 1\n"), Int(int.MaxValue), [0], Real(0));
        EngineAssert.Throws<StreamFormatErrorException>(() => new global::Kerf.Engine.Exchange.Brep.Binary.BinaryShapeSet().ReadPolygon3D(new UnseekableStream(aPolygons)));
        Assert.True(aWatch.Elapsed < TimeSpan.FromSeconds(10));
    }

    [Fact]
    public void Words_longer_than_OCCTs_buffer_are_rejected()
    {
        EngineAssert.Throws<StreamFormatErrorException>(() => ReadSet(Ascii(V4 + new string('L', 300) + " 0\n")));
        // a header line too long for OCCT's getline: not a known version, so nothing is read
        var aSet = new global::Kerf.Engine.Exchange.Brep.Binary.BinaryShapeSet();
        aSet.Read(new MemoryStream(Ascii(new string('x', 500) + "\n")));
        Assert.Equal(0, aSet.ShapeCount);
        // ... and BinTools::Read then finds no shape reference
        EngineAssert.ThrowsAny<FailureException>(() => ReadSet(Ascii("not a shape\n")));
    }

    [Fact]
    public void Nested_geometry_is_capped()
    {
        static byte[] NestedTrimmed(int theDepth)
        {
            var aStream = new MemoryStream();
            aStream.Write(Ascii("Curves 1\n"));
            for (int i = 0; i < theDepth; i++)
            {
                aStream.WriteByte(8);
                aStream.Write(Real(0));
                aStream.Write(Real(1));
            }

            aStream.WriteByte(1);
            aStream.Write(Concat(Real(0), Real(0), Real(0), Real(1), Real(0), Real(0)));
            return aStream.ToArray();
        }

        var aSet = new global::Kerf.Engine.Exchange.Brep.Binary.BinaryCurveSet();
        aSet.Read(new MemoryStream(NestedTrimmed(global::Kerf.Engine.Exchange.Brep.Binary.BinTools.MaxGeometryNesting)));
        Assert.IsType<TrimmedCurve3D>(aSet.Curve(1));
        EngineAssert.Throws<StreamFormatErrorException>(() => new global::Kerf.Engine.Exchange.Brep.Binary.BinaryCurveSet().Read(new MemoryStream(NestedTrimmed(global::Kerf.Engine.Exchange.Brep.Binary.BinTools.MaxGeometryNesting + 1))));
        EngineAssert.Throws<StreamFormatErrorException>(() => new global::Kerf.Engine.Exchange.Brep.Binary.BinaryCurveSet().Read(new MemoryStream(NestedTrimmed(100000))));

        // offset surfaces
        var aSurfaces = new MemoryStream();
        aSurfaces.Write(Ascii("Surfaces 1\n"));
        for (int i = 0; i < 100000; i++)
        {
            aSurfaces.WriteByte(11);
            aSurfaces.Write(Real(1));
        }

        EngineAssert.Throws<StreamFormatErrorException>(() => new global::Kerf.Engine.Exchange.Brep.Binary.BinarySurfaceSet().Read(new MemoryStream(aSurfaces.ToArray())));
    }

    private static Shape NestedCompounds(int theDepth)
    {
        var aB = new BrepBuilder();
        Shape aShape = TestShapes.Vertex(new Point3D(1, 2, 3));
        for (int i = 0; i < theDepth; i++)
        {
            Compound aC = new();
            aB.MakeCompound(ref aC);
            aB.Add(aC, aShape);
            aShape = aC;
        }

        return aShape;
    }

    [Fact]
    public void Nested_shapes_are_capped()
    {
        // the table-based format: shapes refer to earlier ones, so the reading is flat, but writers and explorers recurse
        byte[] anOk = BinToolsRoundTripTests.WriteSet(NestedCompounds(global::Kerf.Engine.Exchange.Brep.Binary.BinTools.MaxShapeNesting - 1), 4);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Read(out Shape aRead, new MemoryStream(anOk));
        Assert.Equal(anOk, BinToolsRoundTripTests.WriteSet(aRead, 4));
        byte[] aDeep = BinToolsRoundTripTests.WriteSet(NestedCompounds(global::Kerf.Engine.Exchange.Brep.Binary.BinTools.MaxShapeNesting), 4);
        EngineAssert.Throws<StreamFormatErrorException>(() => ReadSet(aDeep));

        // the position-based format reads recursively
        byte[] aPositional = BinToolsRoundTripTests.WriteShapes(true, false, NestedCompounds(global::Kerf.Engine.Exchange.Brep.Binary.BinTools.MaxShapeNesting / 2 - 2));
        Shape aShape = BinToolsRoundTripTests.ReadShapes(aPositional, 1)[0];
        Assert.Equal(ShapeType.Compound, aShape.Type);
        byte[] aTooDeep = BinToolsRoundTripTests.WriteShapes(true, false, NestedCompounds(global::Kerf.Engine.Exchange.Brep.Binary.BinTools.MaxShapeNesting));
        EngineAssert.Throws<StreamFormatErrorException>(() => ReadPositional(aTooDeep));

        // 100,000 nested compounds as bytes, which no writer would produce: a deep stack becomes an error
        var aStream = new MemoryStream();
        for (int i = 0; i < 100000; i++)
        {
            aStream.WriteByte(200);
            aStream.WriteByte(7);
            aStream.WriteByte(0);
        }

        EngineAssert.Throws<StreamFormatErrorException>(() => ReadPositional(aStream.ToArray()));

        // and as many nested locations
        var aLocations = new MemoryStream();
        aLocations.WriteByte(200);
        for (int i = 0; i < 100000; i++)
        {
            aLocations.WriteByte(5);
        }

        EngineAssert.Throws<StreamFormatErrorException>(() => ReadPositional(aLocations.ToArray()));
    }

    [Fact]
    public void References_must_point_back_and_not_cycle()
    {
        // a compound whose first sub-shape refers to the compound itself
        byte[] aCycle = [200, 7, 0, 1, 3, 7, 0, 199];
        EngineAssert.Throws<StreamFormatErrorException>(() => ReadPositional(aCycle));

        // a location that contains itself
        byte[] aLocationCycle = [200, 5, 1, 1, 0, 0, 0, 0];
        EngineAssert.Throws<StreamFormatErrorException>(() => ReadPositional(aLocationCycle));

        // a reference before the start of the stream, a zero reference, a position past the end
        EngineAssert.Throws<StreamFormatErrorException>(() => ReadPositional([1, 5]));
        EngineAssert.ThrowsAny<FailureException>(() => ReadPositional([1, 0]));
        EngineAssert.Throws<StreamFormatErrorException>(() => new global::Kerf.Engine.Exchange.Brep.Binary.IStream(new MemoryStream([1, 2, 3])).GoTo(4));

        // an edge whose 3D curves are a chain of 100,000 references, each to the one before: a deep stack becomes an
        // error. Each representation: 1, the curve (here a reference: 1, offset), no location, the range.
        var aChain = new MemoryStream();
        aChain.Write([200 + 4 * 6, 7]); // a FORWARD edge, no location
        aChain.Write(Real(0)); // tolerance
        aChain.WriteByte(0); // flags
        aChain.WriteByte(1); // a 3D curve
        aChain.Write([9, 1]); // at 12: a line
        aChain.Write(Concat(Real(0), Real(0), Real(0), Real(1), Real(0), Real(0)));
        aChain.WriteByte(7);
        aChain.Write(Real(0));
        aChain.Write(Real(1));
        for (int i = 0; i < 100000; i++)
        {
            aChain.WriteByte(1);
            aChain.Write([1, (byte)(i == 0 ? 68 : 20)]);
            aChain.WriteByte(7);
            aChain.Write(Real(0));
            aChain.Write(Real(1));
        }

        EngineAssert.Throws<StreamFormatErrorException>(() => ReadPositional(aChain.ToArray()));
    }

    [Fact]
    public void Locations_with_unbounded_powers_or_chains_are_rejected()
    {
        // two elementary locations, a product of both, then the product to the power 10^9 (OCCT recurses 10^9 times)
        byte[] aPowered = Concat(Ascii("Locations 4\n"), [1], Identity(), [1], Identity(),
                                 [2], Int(1), Int(1), Int(2), Int(1), Int(0),
                                 [2], Int(3), Int(1000000000), Int(0));
        EngineAssert.Throws<StreamFormatErrorException>(() => new global::Kerf.Engine.Exchange.Brep.Binary.BinaryLocationSet().Read(new MemoryStream(aPowered)));
        byte[] aMinPower = Concat(Ascii("Locations 4\n"), [1], Identity(), [1], Identity(),
                                  [2], Int(1), Int(1), Int(2), Int(1), Int(0),
                                  [2], Int(3), Int(int.MinValue), Int(0));
        EngineAssert.Throws<StreamFormatErrorException>(() => new global::Kerf.Engine.Exchange.Brep.Binary.BinaryLocationSet().Read(new MemoryStream(aMinPower)));

        // a single datum to a huge power is one item: fine
        byte[] anElementary = Concat(Ascii("Locations 2\n"), [1], Identity(), [2], Int(1), Int(1000000000), Int(0));
        var aSet = new global::Kerf.Engine.Exchange.Brep.Binary.BinaryLocationSet();
        aSet.Read(new MemoryStream(anElementary));
        Assert.Equal(1000000000, aSet.Location(2).FirstPower());

        // a chain of 2,000 alternating datums
        var aChain = new MemoryStream();
        aChain.Write(Concat(Ascii("Locations 3\n"), [1], Identity(), [1], Identity(), [2]));
        for (int i = 0; i < 2000; i++)
        {
            aChain.Write(Int(1 + i % 2));
            aChain.Write(Int(1));
        }

        aChain.Write(Int(0));
        EngineAssert.Throws<StreamFormatErrorException>(() => new global::Kerf.Engine.Exchange.Brep.Binary.BinaryLocationSet().Read(new MemoryStream(aChain.ToArray())));

        // an index outside the set
        byte[] aBadIndex = Concat(Ascii("Locations 1\n"), [2], Int(5), Int(1), Int(0));
        EngineAssert.Throws<StreamFormatErrorException>(() => new global::Kerf.Engine.Exchange.Brep.Binary.BinaryLocationSet().Read(new MemoryStream(aBadIndex)));
    }

    [Fact]
    public void Indices_and_orientations_are_checked()
    {
        byte[] aGood = BinToolsRoundTripTests.WriteSet(TestShapes.Box(1, 1, 1), 4);
        ReadSet(aGood);

        // the final reference: orientation, TShape index from the end, location
        byte[] aBadOrientation = (byte[])aGood.Clone();
        aBadOrientation[^9] = 9;
        EngineAssert.Throws<StreamFormatErrorException>(() => ReadSet(aBadOrientation));
        byte[] aBadShape = (byte[])aGood.Clone();
        BinaryPrimitives.WriteInt32LittleEndian(aBadShape.AsSpan(aBadShape.Length - 8), 1000);
        EngineAssert.Throws<StreamFormatErrorException>(() => ReadSet(aBadShape));
        BinaryPrimitives.WriteInt32LittleEndian(aBadShape.AsSpan(aBadShape.Length - 8), int.MinValue);
        EngineAssert.Throws<StreamFormatErrorException>(() => ReadSet(aBadShape));
        byte[] aBadLocation = (byte[])aGood.Clone();
        BinaryPrimitives.WriteInt32LittleEndian(aBadLocation.AsSpan(aBadLocation.Length - 4), 3);
        EngineAssert.Throws<StreamFormatErrorException>(() => ReadSet(aBadLocation));
    }

    // -------------------------------------------------------------------------------------------------------------
    // Fuzzing

    private static IEnumerable<(string Name, byte[] Bytes, bool IsPositional)> Samples()
    {
        yield return ("meshed box v4", BinToolsRoundTripTests.WriteSet(BinTestShapes.MeshedBox(true, true), 4, true, true), false);
        yield return ("vertex representations v2", BinToolsRoundTripTests.WriteSet(BinTestShapes.VertexRepresentations(true), 2), false);
        yield return ("regularity and polygons v3", BinToolsRoundTripTests.WriteSet(BinTestShapes.BoxWithRegularityAndPolygon(), 3), false);
        yield return ("locations v1", BinToolsRoundTripTests.WriteSet(BinTestShapes.CompoundWithLocations(), 1), false);
        yield return ("positional locations", BinToolsRoundTripTests.WriteShapes(true, false, BinTestShapes.CompoundWithLocations()), true);
        yield return ("positional meshed cylinder", BinToolsRoundTripTests.WriteShapes(true, true, BinTestShapes.MeshedCylinder()), true);
        yield return ("positional vertex representations", BinToolsRoundTripTests.WriteShapes(true, false, BinTestShapes.VertexRepresentations(true)), true);
    }

    // Reads the bytes; returns null if that worked or threw a Standard_Failure, else the unexpected exception
    private static Exception? TryRead(byte[] theBytes, bool theIsPositional)
    {
        try
        {
            Shape aShape;
            if (theIsPositional)
            {
                aShape = new Shape();
                new global::Kerf.Engine.Exchange.Brep.Binary.ShapeReader().Read(new MemoryStream(theBytes), ref aShape);
            }
            else
            {
                global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Read(out aShape, new MemoryStream(theBytes));
            }

            if (!aShape.IsNull)
            {
                Assert.True(aShape.TShape()!.IsFrozen);
                // what was read can be written again (or the writer reports a Standard_Failure)
                try
                {
                    global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Write(aShape, new MemoryStream());
                }
                catch (Exception caught) when (Refusal.IsFailure(caught))
                {
                }
            }

            return null;
        }
        catch (Exception caught) when (Refusal.IsFailure(caught))
        {
            return null;
        }
        catch (Exception anException)
        {
            return anException;
        }
    }

    private static void RunWithTimeout(Action theAction, TimeSpan theTimeout)
    {
        // a thread with a 1 MB stack, as the readers will meet in callers' threads
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
        }, 1 << 20);
        aThread.Start();
        Assert.True(aThread.Join(theTimeout), "the readers took too long (a hang?)");
        if (anError is not null)
        {
            throw new Xunit.Sdk.XunitException("unexpected exception: " + anError);
        }
    }

    [Fact]
    public void Every_truncation_fails_cleanly()
    {
        int aNbReads = 0;
        RunWithTimeout(() =>
        {
            foreach ((string aName, byte[] aBytes, bool anIsPositional) in Samples())
            {
                for (int aLength = 0; aLength < aBytes.Length; aLength++)
                {
                    Exception? anUnexpected = TryRead(aBytes.AsSpan(0, aLength).ToArray(), anIsPositional);
                    if (anUnexpected is not null)
                    {
                        throw new InvalidOperationException($"{aName} truncated to {aLength}: {anUnexpected}");
                    }

                    aNbReads++;
                }
            }
        }, TimeSpan.FromMinutes(5));
        Assert.True(aNbReads > 10000, aNbReads.ToString(System.Globalization.CultureInfo.InvariantCulture));
    }

    [Fact]
    public void Random_mutations_fail_cleanly()
    {
        int aNbReads = 0;
        RunWithTimeout(() =>
        {
            var aRand = new Random(43010);
            foreach ((string aName, byte[] aBytes, bool anIsPositional) in Samples())
            {
                for (int aTrial = 0; aTrial < 3000; aTrial++)
                {
                    byte[] aMutated = (byte[])aBytes.Clone();
                    int aNbChanges = 1 + aRand.Next(4);
                    for (int i = 0; i < aNbChanges; i++)
                    {
                        int aPos = aRand.Next(aMutated.Length);
                        switch (aRand.Next(4))
                        {
                            case 0:
                                aMutated[aPos] = (byte)aRand.Next(256);
                                break;
                            case 1:
                                aMutated[aPos] ^= (byte)(1 << aRand.Next(8));
                                break;
                            case 2:
                                aMutated[aPos] = aRand.Next(2) == 0 ? (byte)0 : (byte)0xFF;
                                break;
                            default:
                                // a large count where an integer was
                                if (aPos + 4 <= aMutated.Length)
                                {
                                    BinaryPrimitives.WriteInt32LittleEndian(aMutated.AsSpan(aPos), aRand.Next(2) == 0 ? int.MaxValue : aRand.Next());
                                }

                                break;
                        }
                    }

                    Exception? anUnexpected = TryRead(aMutated, anIsPositional);
                    if (anUnexpected is not null)
                    {
                        throw new InvalidOperationException($"{aName}, mutation {aTrial}: {anUnexpected}");
                    }

                    aNbReads++;
                }
            }
        }, TimeSpan.FromMinutes(5));
        Assert.Equal(7 * 3000, aNbReads);
    }
}
