// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Hashing;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingData.Tests;

/// <summary>
/// Round trips through the binary formats: the table-based BinTools_ShapeSet (versions 1 to 4, with and without
/// triangulations and normals) and the position-based BinTools_ShapeWriter/Reader, on shapes holding every
/// representation, and on random geometry of every kind. Native OCCT isn't bound for BinTools (open-questions.md, item
/// 4), so these check that reading and writing again gives the same bytes, and BinToolsFormatTests checks the bytes
/// against OCCT's format.
/// </summary>
public class BinToolsRoundTripTests
{
    internal static byte[] WriteSet(in Shape theShape, int theVersion, bool theWithTriangles = true, bool theWithNormals = false)
    {
        var aStream = new MemoryStream();
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Write(theShape, aStream, theWithTriangles, theWithNormals, (global::Kerf.Engine.Exchange.Brep.Binary.BinaryFormatVersion)theVersion);
        return aStream.ToArray();
    }

    internal static Shape ReadSet(byte[] theBytes)
    {
        var aStream = new MemoryStream(theBytes);
        global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Read(out Shape aShape, aStream);
        Assert.Equal(theBytes.Length, aStream.Position);
        return aShape;
    }

    internal static byte[] WriteShapes(bool theWithTriangles, bool theWithNormals, params Shape[] theShapes)
    {
        var aStream = new MemoryStream();
        var aWriter = new global::Kerf.Engine.Exchange.Brep.Binary.ShapeWriter();
        aWriter.SetWithTriangles(theWithTriangles);
        aWriter.SetWithNormals(theWithNormals);
        foreach (Shape aShape in theShapes)
        {
            aWriter.Write(aShape, aStream);
        }

        return aStream.ToArray();
    }

    internal static Shape[] ReadShapes(byte[] theBytes, int theCount)
    {
        var aStream = new MemoryStream(theBytes);
        var aReader = new global::Kerf.Engine.Exchange.Brep.Binary.ShapeReader();
        var aShapes = new Shape[theCount];
        for (int i = 0; i < theCount; i++)
        {
            Shape aShape = new();
            aReader.Read(aStream, ref aShape);
            aShapes[i] = aShape;
        }

        Assert.Equal(theBytes.Length, aStream.Position);
        return aShapes;
    }

    private static int CountSubShapes(in Shape theShape, ShapeType theType)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        return aMap.Count;
    }

    private static void AssertSameStructure(in Shape theExpected, in Shape theActual)
    {
        Assert.False(theActual.IsNull);
        Assert.Equal(theExpected.ShapeType(), theActual.ShapeType());
        Assert.Equal(theExpected.Orientation(), theActual.Orientation());
        foreach (ShapeType aType in new[] { ShapeType.Vertex, ShapeType.Edge, ShapeType.Wire, ShapeType.Face, ShapeType.Shell, ShapeType.Solid, ShapeType.CompSolid })
        {
            Assert.Equal(CountSubShapes(theExpected, aType), CountSubShapes(theActual, aType));
        }

        // vertex positions, in exploration order
        var anE = new Explorer(theExpected, ShapeType.Vertex);
        var anA = new Explorer(theActual, ShapeType.Vertex);
        for (; anE.More(); anE.Next(), anA.Next())
        {
            Assert.True(anA.More());
            Point3D aP1 = Tool.Pnt(ShapeCasts.Vertex(anE.Current()));
            Point3D aP2 = Tool.Pnt(ShapeCasts.Vertex(anA.Current()));
            Assert.True(aP1.Distance(aP2) < 1e-12, $"{aP1} {aP2}");
        }
    }

    private static void AssertFrozen(in Shape theShape)
    {
        Assert.True(theShape.TShape()!.IsFrozen);
        for (var anIt = new Explorer(theShape, ShapeType.Vertex); anIt.More(); anIt.Next())
        {
            Assert.True(anIt.Current().TShape()!.IsFrozen);
        }

        for (var anIt = new Explorer(theShape, ShapeType.Edge); anIt.More(); anIt.Next())
        {
            Assert.True(anIt.Current().TShape()!.IsFrozen);
        }

        for (var anIt = new Explorer(theShape, ShapeType.Face); anIt.More(); anIt.Next())
        {
            Assert.True(anIt.Current().TShape()!.IsFrozen);
            Triangulation? aTri = Tool.Triangulation(ShapeCasts.Face(anIt.Current()), out _);
            Assert.True(aTri is null || aTri.IsFrozen);
        }
    }

    public static TheoryData<int, bool, bool> Versions()
    {
        var aData = new TheoryData<int, bool, bool>();
        for (int aVersion = 1; aVersion <= 4; aVersion++)
        {
            aData.Add(aVersion, true, false);
            aData.Add(aVersion, false, false);
            aData.Add(aVersion, true, true);
        }

        return aData;
    }

    [Theory]
    [MemberData(nameof(Versions))]
    public void Shape_sets_round_trip_byte_for_byte(int theVersion, bool theWithTriangles, bool theWithNormals)
    {
        foreach ((string aName, Shape aShape) in BinTestShapes.All())
        {
            byte[] aB1 = WriteSet(aShape, theVersion, theWithTriangles, theWithNormals);
            Shape aS1 = ReadSet(aB1);
            AssertSameStructure(aShape, aS1);
            AssertFrozen(aS1);
            byte[] aB2 = WriteSet(aS1, theVersion, theWithTriangles, theWithNormals);
            Shape aS2 = ReadSet(aB2);
            byte[] aB3 = WriteSet(aS2, theVersion, theWithTriangles, theWithNormals);
            bool isSwapping = aName.StartsWith("vertex representations", StringComparison.Ordinal);
            if (isSwapping)
            {
                // OCCT swaps the parameters of a point on a surface on each round trip (occt-issues.md)
                byte[] aB4 = WriteSet(ReadSet(aB3), theVersion, theWithTriangles, theWithNormals);
                Assert.True(aB2.AsSpan().SequenceEqual(aB4), $"{aName}: third round trip, version {theVersion}");
            }
            else
            {
                Assert.True(aB2.AsSpan().SequenceEqual(aB3), $"{aName}: second round trip, version {theVersion}");
            }

            // version 1 checks the faces it reads (their Checked flag changes)
            if (!isSwapping && theVersion != 1)
            {
                Assert.True(aB1.AsSpan().SequenceEqual(aB2), $"{aName}: first round trip, version {theVersion}");
            }
        }
    }

    [Theory]
    [InlineData(true, false)]
    [InlineData(false, false)]
    [InlineData(true, true)]
    public void Shape_writer_round_trips_byte_for_byte(bool theWithTriangles, bool theWithNormals)
    {
        foreach ((string aName, Shape aShape) in BinTestShapes.All())
        {
            byte[] aB1 = WriteShapes(theWithTriangles, theWithNormals, aShape);
            Shape aS1 = ReadShapes(aB1, 1)[0];
            AssertSameStructure(aShape, aS1);
            AssertFrozen(aS1);
            byte[] aB2 = WriteShapes(theWithTriangles, theWithNormals, aS1);
            Shape aS2 = ReadShapes(aB2, 1)[0];
            byte[] aB3 = WriteShapes(theWithTriangles, theWithNormals, aS2);
            bool isSwapping = aName.StartsWith("vertex representations", StringComparison.Ordinal);
            if (isSwapping)
            {
                // the first reading drops the point on a curve on a surface, and every reading swaps the point on a
                // surface's parameters (occt-issues.md)
                byte[] aB4 = WriteShapes(theWithTriangles, theWithNormals, ReadShapes(aB3, 1)[0]);
                Assert.True(aB2.AsSpan().SequenceEqual(aB4), $"{aName}: third round trip");
            }
            else
            {
                Assert.True(aB2.AsSpan().SequenceEqual(aB3), $"{aName}: second round trip");
                Assert.True(aB1.AsSpan().SequenceEqual(aB2), $"{aName}: first round trip");
            }
        }
    }

    [Fact]
    public void Shape_writer_shares_objects_between_shapes()
    {
        Shape aBox = BinTestShapes.MeshedBox(true, true);
        Shape aFace = new Explorer(aBox, ShapeType.Face).Current();
        var aT = new Transform3D();
        aT = aT.WithTranslation(new Vector3D(0, 0, 7));
        Shape aMoved = aFace.Moved(new Placement(aT));
        byte[] aBytes = WriteShapes(true, true, aBox, aMoved, aBox);
        byte[] aBoxOnly = WriteShapes(true, true, aBox);
        // the second and third shapes are references: a reference, a location, an orientation
        Assert.True(aBytes.Length < aBoxOnly.Length + 120, $"{aBytes.Length} {aBoxOnly.Length}");

        Shape[] aRead = ReadShapes(aBytes, 3);
        Shape aReadFace = new Explorer(aRead[0], ShapeType.Face).Current();
        Assert.True(aRead[1].IsPartner(aReadFace));
        Assert.Equal(aMoved.Location().Transformation().TranslationPart().Z(), aRead[1].Location().Transformation().TranslationPart().Z());
        Assert.True(aRead[2].IsSame(aRead[0]));
        Assert.True(aRead[2].IsEqual(aRead[0]));

        // the same bytes again from the shapes read
        Assert.True(aBytes.AsSpan().SequenceEqual(WriteShapes(true, true, aRead[0], aRead[1], aRead[2])));
    }

    [Fact]
    public void Regularity_and_3d_polygons_round_trip()
    {
        Shape aShape = BinTestShapes.BoxWithRegularityAndPolygon();
        foreach (int aVersion in new[] { 2, 3, 4 })
        {
            byte[] aB1 = WriteSet(aShape, aVersion);
            Shape aS1 = ReadSet(aB1);
            Assert.True(aB1.AsSpan().SequenceEqual(WriteSet(aS1, aVersion)));
            Edge anEdge = BinTestShapes.FirstEdge(aS1);
            var aFaces = BinTestShapes.FacesOf(aS1, anEdge);
            Assert.Equal(Continuity.G1, Tool.Continuity(anEdge, aFaces[0], aFaces[1]));
            Polygon3D? aPolygon = Tool.Polygon3D(BinTestShapes.NthEdge(aS1, 2), out _);
            Assert.NotNull(aPolygon);
            Assert.Equal(3, aPolygon.NodeCount);
            Assert.True(aPolygon.HasParameters);
            Assert.Equal(0.01, aPolygon.Deflection());
            Assert.True(aPolygon.IsFrozen);
        }

        byte[] aW1 = WriteShapes(true, false, aShape);
        Assert.True(aW1.AsSpan().SequenceEqual(WriteShapes(true, false, ReadShapes(aW1, 1)[0])));

        // without triangles, 3D polygons aren't written
        Shape aBare = ReadSet(WriteSet(aShape, 4, theWithTriangles: false));
        Assert.Null(Tool.Polygon3D(BinTestShapes.NthEdge(aBare, 2), out _));
    }

    [Fact]
    public void Triangulations_keep_their_uv_nodes_and_normals()
    {
        Shape aShape = BinTestShapes.MeshedBox(true, true);
        foreach ((bool aWithNormals, bool anExpectNormals) in new[] { (true, true), (false, false) })
        {
            Shape aRead = ReadSet(WriteSet(aShape, 4, true, aWithNormals));
            Face aFace = ShapeCasts.Face(new Explorer(aRead, ShapeType.Face).Current());
            Triangulation? aTri = Tool.Triangulation(aFace, out _);
            Assert.NotNull(aTri);
            Assert.Equal(4, aTri.NodeCount);
            Assert.Equal(2, aTri.TriangleCount);
            Assert.True(aTri.HasUVNodes);
            Assert.Equal(anExpectNormals, aTri.HasNormals);
            Assert.Equal(new Point2D(1, 0.5).X(), aTri.UVNode(2).X());

            // version 3 writes no normals
            Shape aV3 = ReadSet(WriteSet(aShape, 3, true, aWithNormals));
            Assert.False(Tool.Triangulation(ShapeCasts.Face(new Explorer(aV3, ShapeType.Face).Current()), out _)!.HasNormals);
        }

        // a face with only a triangulation keeps it, with its normals, even when triangles aren't asked for
        Shape aTriOnly = ReadSet(WriteSet(BinTestShapes.TriangulationOnlyFace(), 4, false, false));
        Triangulation? aOnly = Tool.Triangulation(ShapeCasts.Face(aTriOnly), out _);
        Assert.NotNull(aOnly);
        Assert.True(aOnly.HasNormals);
        aTriOnly = ReadShapes(WriteShapes(false, false, BinTestShapes.TriangulationOnlyFace()), 1)[0];
        Assert.True(Tool.Triangulation(ShapeCasts.Face(aTriOnly), out _)!.HasNormals);

        // polygons on triangulations, closed (the seam) and not
        Shape aCylinder = ReadSet(WriteSet(BinTestShapes.MeshedCylinder(), 4));
        int aNbClosed = 0;
        for (var anE = new Explorer(aCylinder, ShapeType.Edge); anE.More(); anE.Next())
        {
            BrepTEdge aTE = (BrepTEdge)anE.Current().TShape()!;
            foreach (CurveRepresentation aCR in aTE.CurveRepresentations())
            {
                if (aCR.IsPolygonOnClosedTriangulation)
                {
                    aNbClosed++;
                }
            }
        }

        // the seam is met twice (forward and reversed), each with its one representation on the closed triangulation
        Assert.Equal(2, aNbClosed);
    }

    [Fact]
    public void Free_flags_of_sub_shapes_are_cleared_by_reading()
    {
        // OCCT's reader adds each sub-shape to its parent, which clears its Free flag (design 4.4)
        Solid aBox = TestShapes.Box(1, 1, 1);
        Shape aRead = ReadSet(WriteSet(aBox, 4));
        Assert.True(aRead.Free());
        for (var anIt = new Explorer(aRead, ShapeType.Face); anIt.More(); anIt.Next())
        {
            Assert.False(anIt.Current().Free());
        }

        aRead = ReadShapes(WriteShapes(true, false, aBox), 1)[0];
        for (var anIt = new Explorer(aRead, ShapeType.Face); anIt.More(); anIt.Next())
        {
            Assert.False(anIt.Current().Free());
        }
    }

    [Fact]
    public void Version_1_checks_faces_on_reading()
    {
        // version 1 has no UV points: reading recomputes them (BRepTools::Update) and the faces come back checked
        Shape aShape = TestShapes.Box(1, 2, 3);
        byte[] aB1 = WriteSet(aShape, 1);
        Shape aS1 = ReadSet(aB1);
        AssertSameStructure(aShape, aS1);
        for (var anIt = new Explorer(aS1, ShapeType.Face); anIt.More(); anIt.Next())
        {
            Assert.True(anIt.Current().Checked());
        }
    }

    [Fact]
    public void Point_representations_round_trip()
    {
        Shape aShape = BinTestShapes.VertexRepresentations(includePointOnSurface: true);
        var anOriginal = ((BrepTVertex)FindFreeVertex(aShape).TShape()!).PointRepresentations();
        Assert.Contains(anOriginal, theRep => theRep.IsPointOnCurve());
        Assert.Contains(anOriginal, theRep => theRep.IsPointOnCurveOnSurface());
        Assert.Contains(anOriginal, theRep => theRep.IsPointOnSurface());

        // the table-based format keeps them all, the parameters of a point on a surface in their order (OCCT's readers
        // swapped them: occt-issues.md, BinTools_ShapeSet's point on a surface; Kerf decision 1500)
        PointRepresentation aOnSurface = anOriginal.First(theRep => theRep.IsPointOnSurface());
        byte[] aSecondFirst = [.. BitConverter.GetBytes(aOnSurface.Parameter2()), .. BitConverter.GetBytes(aOnSurface.Parameter())];
        foreach (int aVersion in new[] { 2, 3, 4 })
        {
            byte[] aWritten = WriteSet(aShape, aVersion);
            // the file is OCCT's, the second parameter written first, so files written before read right
            Assert.True(aWritten.AsSpan().IndexOf(aSecondFirst) >= 0);
            Shape aRead = ReadSet(aWritten);
            var aReps = ((BrepTVertex)FindFreeVertex(aRead).TShape()!).PointRepresentations();
            Assert.Equal(anOriginal.Count, aReps.Count);
            for (int i = 0; i < aReps.Count; i++)
            {
                AssertSameRepresentation(anOriginal[i], aReps[i]);
            }
        }

        // the position-based reader drops a point on a curve on a surface (it tests the surface for null the wrong way
        // round) and keeps a point on a surface's parameters in their order too
        Shape aRead4 = ReadShapes(WriteShapes(true, false, aShape), 1)[0];
        var aReps4 = ((BrepTVertex)FindFreeVertex(aRead4).TShape()!).PointRepresentations();
        var anExpected4 = anOriginal.Where(theRep => !theRep.IsPointOnCurveOnSurface()).ToList();
        Assert.Equal(anExpected4.Count, aReps4.Count);
        for (int i = 0; i < aReps4.Count; i++)
        {
            AssertSameRepresentation(anExpected4[i], aReps4[i]);
        }
    }

    private static void AssertSameRepresentation(PointRepresentation theOriginal, PointRepresentation theRead)
    {
        Assert.Equal(theOriginal.GetType(), theRead.GetType());
        Assert.Equal(theOriginal.Parameter(), theRead.Parameter());
        if (theOriginal.IsPointOnSurface())
        {
            Assert.Equal(theOriginal.Parameter2(), theRead.Parameter2());
        }
    }

    private static Vertex FindFreeVertex(in Shape theCompound)
    {
        for (var anIt = new Iterator(theCompound); anIt.More(); anIt.Next())
        {
            if (anIt.Value().ShapeType() == ShapeType.Vertex
                && Tool.Pnt(ShapeCasts.Vertex(anIt.Value())).Distance(new Point3D(0.25, 0.5, 0.75)) < 1e-12)
            {
                return ShapeCasts.Vertex(anIt.Value());
            }
        }

        throw new InvalidOperationException("no free vertex");
    }

    [Fact]
    public void Locations_elementary_and_complex_round_trip()
    {
        Shape aShape = BinTestShapes.CompoundWithLocations();
        var aExpected = new List<Transform3D>();
        for (var anIt = new Iterator(aShape); anIt.More(); anIt.Next())
        {
            aExpected.Add(anIt.Value().Location().Transformation());
        }

        foreach (Shape aRead in new[] { ReadSet(WriteSet(aShape, 4)), ReadShapes(WriteShapes(true, false, aShape), 1)[0] })
        {
            int i = 0;
            for (var anIt = new Iterator(aRead); anIt.More(); anIt.Next(), i++)
            {
                Transform3D aT = anIt.Value().Location().Transformation();
                for (int r = 1; r <= 3; r++)
                {
                    for (int c = 1; c <= 4; c++)
                    {
                        Assert.Equal(aExpected[i].Value(r, c), aT.Value(r, c), 1e-14);
                    }
                }
            }

            Assert.Equal(aExpected.Count, i);
        }

        // the location set alone, with its complex entries
        var aSet = new global::Kerf.Engine.Exchange.Brep.Binary.BinaryLocationSet();
        var aT1 = new Transform3D();
        aT1 = aT1.WithTranslation(new Vector3D(1, 2, 3));
        var aT2 = new Transform3D();
        aT2 = aT2.WithRotation(new Axis3D(new Point3D(), new Direction3D(1, 0, 0)), 1.0);
        var aL1 = new Placement(aT1);
        var aL2 = new Placement(aT2);
        Placement aComplex = aL2.Powered(3) * aL1.Powered(-2);
        Assert.Equal(0, aSet.Add(new Placement()));
        int anIndex = aSet.Add(aComplex);
        Assert.Equal(3, anIndex);
        Assert.Equal(3, aSet.LocationCount);
        var aStream = new MemoryStream();
        aSet.Write(aStream);
        aStream.Position = 0;
        var aRead2 = new global::Kerf.Engine.Exchange.Brep.Binary.BinaryLocationSet();
        aRead2.Read(aStream);
        Assert.Equal(3, aRead2.LocationCount);
        Placement aReadComplex = aRead2.Location(3);
        Assert.Equal(2, NbItems(aReadComplex));
        // the first item of a product is its right-hand factor
        Assert.Equal(-2, aReadComplex.FirstPower());
        Assert.Equal(3, aReadComplex.NextLocation().FirstPower());
        Assert.True(aRead2.Location(0).IsIdentity);
        var aAgain = new MemoryStream();
        aRead2.Write(aAgain);
        Assert.Equal(aStream.ToArray(), aAgain.ToArray());
    }

    private static int NbItems(Placement theLocation)
    {
        int aNb = 0;
        for (; !theLocation.IsIdentity; theLocation = theLocation.NextLocation())
        {
            aNb++;
        }

        return aNb;
    }

    [Fact]
    public void Random_curves_of_every_kind_round_trip()
    {
        var aRand = new Random(4301);
        var aSet = new global::Kerf.Engine.Exchange.Brep.Binary.BinaryCurveSet();
        var aCurves = new List<Curve3D>();
        for (int i = 0; i < 330; i++)
        {
            Curve3D aCurve = GeomHash_PortTests.Curve(aRand, i % 11);
            aCurves.Add(aCurve);
            Assert.Equal(i + 1, aSet.Add(aCurve));
        }

        Assert.Equal(0, aSet.Add(null));
        var aStream = new MemoryStream();
        aSet.Write(aStream);
        byte[] aBytes = aStream.ToArray();
        Assert.True(aBytes.AsSpan(0, 11).SequenceEqual("Curves 330\n"u8));

        var aRead = new global::Kerf.Engine.Exchange.Brep.Binary.BinaryCurveSet();
        aRead.Read(new MemoryStream(aBytes));
        var aHasher = new CurveHasher();
        for (int i = 1; i <= aCurves.Count; i++)
        {
            Curve3D? aCurve = aRead.Curve(i);
            Assert.NotNull(aCurve);
            Assert.True(aCurve.IsFrozen);
            Assert.Equal(aCurves[i - 1].DynamicType(), aCurve.DynamicType());
            Assert.True(aHasher.Equals(aCurves[i - 1], aCurve), $"curve {i}, {aCurve.DynamicTypeName()}");
            Assert.Equal(i, aRead.Index(aCurve));
        }

        Assert.Null(aRead.Curve(0));
        Assert.Equal(0, aRead.Index(null));

        // kinds without directions (which reading normalises again) write the same bytes
        for (int i = 1; i <= aCurves.Count; i++)
        {
            if (aCurves[i - 1] is BezierCurve3D or BSplineCurve3D)
            {
                var anOne = new MemoryStream();
                global::Kerf.Engine.Exchange.Brep.Binary.BinaryCurveSet.WriteCurve(aCurves[i - 1], new global::Kerf.Engine.Exchange.Brep.Binary.OStream(anOne));
                var anOther = new MemoryStream();
                global::Kerf.Engine.Exchange.Brep.Binary.BinaryCurveSet.WriteCurve(aRead.Curve(i)!, new global::Kerf.Engine.Exchange.Brep.Binary.OStream(anOther));
                Assert.Equal(anOne.ToArray(), anOther.ToArray());
            }
        }

        // a single curve, read on its own, isn't frozen
        var aSingle = new MemoryStream();
        global::Kerf.Engine.Exchange.Brep.Binary.BinaryCurveSet.WriteCurve(aCurves[7], new global::Kerf.Engine.Exchange.Brep.Binary.OStream(aSingle));
        aSingle.Position = 0;
        global::Kerf.Engine.Exchange.Brep.Binary.BinaryCurveSet.ReadCurve(aSingle, out Curve3D? aSingleRead);
        Assert.NotNull(aSingleRead);
        Assert.False(aSingleRead.IsFrozen);
        Assert.True(aHasher.Equals(aCurves[7], aSingleRead));
    }

    [Fact]
    public void Random_2d_curves_of_every_kind_round_trip()
    {
        var aRand = new Random(4302);
        var aSet = new global::Kerf.Engine.Exchange.Brep.Binary.BinaryCurve2dSet();
        var aCurves = new List<Curve2D>();
        for (int i = 0; i < 300; i++)
        {
            Curve2D aCurve = GeomHash_PortTests.Curve2d(aRand, i % 10);
            aCurves.Add(aCurve);
            aSet.Add(aCurve);
        }

        var aStream = new MemoryStream();
        aSet.Write(aStream);
        var aRead = new global::Kerf.Engine.Exchange.Brep.Binary.BinaryCurve2dSet();
        aRead.Read(new MemoryStream(aStream.ToArray()));
        var aHasher = new CurveHasher2D();
        for (int i = 1; i <= aCurves.Count; i++)
        {
            Curve2D? aCurve = aRead.Curve2d(i);
            Assert.NotNull(aCurve);
            Assert.True(aCurve.IsFrozen);
            Assert.Equal(aCurves[i - 1].DynamicType(), aCurve.DynamicType());
            Assert.True(aHasher.Equals(aCurves[i - 1], aCurve), $"curve {i}, {aCurve.DynamicTypeName()}");
        }

        // kinds without directions (which reading normalises again) write the same bytes
        for (int i = 1; i <= aCurves.Count; i++)
        {
            if (aCurves[i - 1] is BezierCurve2D or BSplineCurve2D)
            {
                var anOne = new MemoryStream();
                global::Kerf.Engine.Exchange.Brep.Binary.BinaryCurve2dSet.WriteCurve2d(aCurves[i - 1], new global::Kerf.Engine.Exchange.Brep.Binary.OStream(anOne));
                var anOther = new MemoryStream();
                global::Kerf.Engine.Exchange.Brep.Binary.BinaryCurve2dSet.WriteCurve2d(aRead.Curve2d(i)!, new global::Kerf.Engine.Exchange.Brep.Binary.OStream(anOther));
                Assert.Equal(anOne.ToArray(), anOther.ToArray());
            }
        }

        // the 2D set holds the null curve once, as OCCT's does
        var aWithNull = new global::Kerf.Engine.Exchange.Brep.Binary.BinaryCurve2dSet();
        Assert.Equal(1, aWithNull.Add(null));
        Assert.Equal(1, aWithNull.Add(null));
        Assert.Null(aWithNull.Curve2d(1));
        Assert.Equal(1, aWithNull.Index(null));
    }

    [Fact]
    public void Random_surfaces_of_every_kind_round_trip()
    {
        var aRand = new Random(4303);
        var aSet = new global::Kerf.Engine.Exchange.Brep.Binary.BinarySurfaceSet();
        var aSurfaces = new List<Surface>();
        for (int i = 0; i < 360; i++)
        {
            Surface aSurface = GeomHash_PortTests.Surface(aRand, i % 12);
            aSurfaces.Add(aSurface);
            aSet.Add(aSurface);
        }

        var aStream = new MemoryStream();
        aSet.Write(aStream);
        var aRead = new global::Kerf.Engine.Exchange.Brep.Binary.BinarySurfaceSet();
        aRead.Read(new MemoryStream(aStream.ToArray()));
        var aHasher = new SurfaceHasher();
        var aKinds = new HashSet<Type>();
        for (int i = 1; i <= aSurfaces.Count; i++)
        {
            Surface? aSurface = aRead.Surface(i);
            Assert.NotNull(aSurface);
            Assert.True(aSurface.IsFrozen);
            Assert.Equal(aSurfaces[i - 1].DynamicType(), aSurface.DynamicType());
            Assert.True(aHasher.Equals(aSurfaces[i - 1], aSurface), $"surface {i}, {aSurface.DynamicTypeName()}");
            aKinds.Add(aSurface.GetType());
        }

        Assert.Equal(11, aKinds.Count);
        // kinds without directions (which reading normalises again) write the same bytes
        for (int i = 1; i <= aSurfaces.Count; i++)
        {
            if (aSurfaces[i - 1] is BezierSurface or BSplineSurface)
            {
                var anOne = new MemoryStream();
                global::Kerf.Engine.Exchange.Brep.Binary.BinarySurfaceSet.WriteSurface(aSurfaces[i - 1], new global::Kerf.Engine.Exchange.Brep.Binary.OStream(anOne));
                var anOther = new MemoryStream();
                global::Kerf.Engine.Exchange.Brep.Binary.BinarySurfaceSet.WriteSurface(aRead.Surface(i)!, new global::Kerf.Engine.Exchange.Brep.Binary.OStream(anOther));
                Assert.Equal(anOne.ToArray(), anOther.ToArray());
            }
        }
    }

    [Fact]
    public void Shapes_with_random_geometry_round_trip()
    {
        // edges on random curves and faces on random surfaces, through both formats
        var aRand = new Random(4304);
        var aB = new BrepBuilder();
        Compound aCompound = new();
        aB.MakeCompound(ref aCompound);
        for (int i = 0; i < 44; i++)
        {
            Curve3D aCurve = GeomHash_PortTests.Curve(aRand, i % 11);
            Edge anEdge = new();
            aB.MakeEdge(ref anEdge, aCurve, 1e-7);
            aB.Range(anEdge, 0.1, 0.9);
            aB.Add(aCompound, anEdge);

            Surface aSurface = GeomHash_PortTests.Surface(aRand, i % 12);
            Face aFace = new();
            aB.MakeFace(ref aFace, aSurface, 1e-7);
            Edge aPEdge = new();
            aB.MakeEdge(ref aPEdge);
            aB.UpdateEdge(aPEdge, GeomHash_PortTests.Curve2d(aRand, i % 10), aFace, 1e-7);
            aB.Range(aPEdge, aFace, 0.2, 0.8);
            Wire aWire = new();
            aB.MakeWire(ref aWire);
            aB.Add(aWire, aPEdge);
            aB.Add(aFace, aWire);
            aB.Add(aCompound, aFace);
        }

        // reading normalises directions again, so only the layout is compared: the same lengths, the same structure
        foreach (int aVersion in new[] { 2, 3, 4 })
        {
            byte[] aB1 = WriteSet(aCompound, aVersion);
            Shape aS1 = ReadSet(aB1);
            AssertSameStructure(aCompound, aS1);
            Assert.Equal(aB1.Length, WriteSet(aS1, aVersion).Length);
        }

        byte[] aW1 = WriteShapes(true, false, aCompound);
        Shape aR1 = ReadShapes(aW1, 1)[0];
        AssertSameStructure(aCompound, aR1);
        Assert.Equal(aW1.Length, WriteShapes(true, false, aR1).Length);
    }

    [Fact]
    public void Files_and_unseekable_streams()
    {
        Shape aShape = BinTestShapes.MeshedBox(true, false);
        string aDir = Path.Combine(Path.GetTempPath(), "occtnet-bintools-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(aDir);
        try
        {
            string aFile = Path.Combine(aDir, "box.bin");
            Assert.True(global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Write(aShape, aFile));
            byte[] aBytes = File.ReadAllBytes(aFile);
            Assert.Equal(WriteSet(aShape, 4), aBytes);
            Assert.True(global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Read(out Shape aRead, aFile));
            AssertSameStructure(aShape, aRead);
            Assert.False(global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Read(out Shape aMissing, Path.Combine(aDir, "missing.bin")));
            Assert.True(aMissing.IsNull);
            Assert.False(global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Write(aShape, Path.Combine(aDir, "no", "such", "dir.bin")));

            // a stream that can't seek is buffered
            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Read(out Shape anUnseekable, new UnseekableStream(aBytes));
            Assert.Equal(aBytes, WriteSet(anUnseekable, 4));

            // the sets read a stream that can't seek: the character after a count is pushed back
            var aSet = new global::Kerf.Engine.Exchange.Brep.Binary.BinaryCurveSet();
            aSet.Add(new LineCurve3D(new Point3D(1, 2, 3), new Direction3D(0, 0, 1)));
            var aStream = new MemoryStream();
            aSet.Write(aStream);
            var aRead2 = new global::Kerf.Engine.Exchange.Brep.Binary.BinaryCurveSet();
            aRead2.Read(new UnseekableStream(aStream.ToArray()));
            Assert.NotNull(aRead2.Curve(1));

            // writing to a stream that can't seek gives the same bytes
            var aWriteOnly = new UnseekableStream([]);
            global::Kerf.Engine.Exchange.Brep.Binary.BinTools.Write(aShape, aWriteOnly);
            Assert.Equal(aBytes, aWriteOnly.Written());
            var aWriterOnly = new UnseekableStream([]);
            new global::Kerf.Engine.Exchange.Brep.Binary.ShapeWriter().Write(aShape, aWriterOnly);
            Assert.Equal(WriteShapes(false, false, aShape), aWriterOnly.Written());

            // the position-based reader needs to seek
            Assert.Throws<ProgramErrorException>(() =>
            {
                Shape aShapeRead = new();
                new global::Kerf.Engine.Exchange.Brep.Binary.ShapeReader().Read(new UnseekableStream(aWriterOnly.Written()), ref aShapeRead);
            });
        }
        finally
        {
            Directory.Delete(aDir, true);
        }
    }

    [Fact]
    public void Shape_set_api()
    {
        Solid aBox = TestShapes.Box(1, 1, 1);
        var aSet = new global::Kerf.Engine.Exchange.Brep.Binary.BinaryShapeSet();
        Assert.Equal(0, aSet.Add(new Shape()));
        int anIndex = aSet.Add(aBox);
        Assert.Equal(aSet.ShapeCount, anIndex);
        Assert.Equal(8 + 12 + 6 + 6 + 1 + 1, aSet.ShapeCount);
        Assert.Equal(anIndex, aSet.Index(aBox));
        Assert.True(aSet.Shape(anIndex).IsSame(aBox));
        Assert.Equal(anIndex, aSet.Add(aBox.Reversed()));
        Assert.Same(aSet.Locations(), aSet.ChangeLocations());
        Assert.Equal((int)global::Kerf.Engine.Exchange.Brep.Binary.BinaryFormatVersion.FormatVersion_CURRENT, aSet.FormatNb());
        aSet.SetFormatNb(9);
        Assert.Equal(4, aSet.FormatNb());
        aSet.SetFormatNb(2);
        Assert.Equal(2, aSet.FormatNb());
        Assert.False(aSet.IsWithTriangles);
        aSet.Clear();
        Assert.Equal(0, aSet.ShapeCount);
    }
}

/// <summary>A stream that reads from a buffer or writes to one, and can't seek.</summary>
internal sealed class UnseekableStream(byte[] theBytes) : Stream
{
    private readonly MemoryStream myInner = new(theBytes.Length == 0 ? [] : theBytes, theBytes.Length != 0);
    private readonly MemoryStream myWritten = new();

    public byte[] Written() => myWritten.ToArray();

    public override bool CanRead => true;

    public override bool CanSeek => false;

    public override bool CanWrite => true;

    public override long Length => throw new NotSupportedException();

    public override long Position
    {
        get => throw new NotSupportedException();
        set => throw new NotSupportedException();
    }

    public override void Flush()
    {
    }

    public override int Read(byte[] buffer, int offset, int count) => myInner.Read(buffer, offset, count);

    public override long Seek(long offset, SeekOrigin origin) => throw new NotSupportedException();

    public override void SetLength(long value) => throw new NotSupportedException();

    public override void Write(byte[] buffer, int offset, int count) => myWritten.Write(buffer, offset, count);
}
