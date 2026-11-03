// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Meshing;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingData.Tests;

/// <summary>
/// Round trips through the binary formats: the table-based BinTools_ShapeSet (versions 1 to 4, with and without
/// triangulations and normals) and the position-based BinTools_ShapeWriter/Reader, on shapes holding every
/// representation, and on random geometry of every kind. Native OCCT isn't bound for BinTools (open-questions.md, item
/// 4), so these check that reading and writing again gives the same bytes, and BinToolsFormatTests checks the bytes
/// against OCCT's format.
/// </summary>
public class BinToolsRoundTripTests
{
    internal static byte[] WriteSet(in TopoDS_Shape theShape, int theVersion, bool theWithTriangles = true, bool theWithNormals = false)
    {
        var aStream = new MemoryStream();
        BinTools.Write(theShape, aStream, theWithTriangles, theWithNormals, (BinTools_FormatVersion)theVersion);
        return aStream.ToArray();
    }

    internal static TopoDS_Shape ReadSet(byte[] theBytes)
    {
        var aStream = new MemoryStream(theBytes);
        BinTools.Read(out TopoDS_Shape aShape, aStream);
        Assert.Equal(theBytes.Length, aStream.Position);
        return aShape;
    }

    internal static byte[] WriteShapes(bool theWithTriangles, bool theWithNormals, params TopoDS_Shape[] theShapes)
    {
        var aStream = new MemoryStream();
        var aWriter = new BinTools_ShapeWriter();
        aWriter.SetWithTriangles(theWithTriangles);
        aWriter.SetWithNormals(theWithNormals);
        foreach (TopoDS_Shape aShape in theShapes)
        {
            aWriter.Write(aShape, aStream);
        }

        return aStream.ToArray();
    }

    internal static TopoDS_Shape[] ReadShapes(byte[] theBytes, int theCount)
    {
        var aStream = new MemoryStream(theBytes);
        var aReader = new BinTools_ShapeReader();
        var aShapes = new TopoDS_Shape[theCount];
        for (int i = 0; i < theCount; i++)
        {
            TopoDS_Shape aShape = new();
            aReader.Read(aStream, ref aShape);
            aShapes[i] = aShape;
        }

        Assert.Equal(theBytes.Length, aStream.Position);
        return aShapes;
    }

    private static int CountSubShapes(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        return aMap.Extent();
    }

    private static void AssertSameStructure(in TopoDS_Shape theExpected, in TopoDS_Shape theActual)
    {
        Assert.False(theActual.IsNull());
        Assert.Equal(theExpected.ShapeType(), theActual.ShapeType());
        Assert.Equal(theExpected.Orientation(), theActual.Orientation());
        foreach (TopAbs_ShapeEnum aType in new[] { TopAbs_VERTEX, TopAbs_EDGE, TopAbs_WIRE, TopAbs_FACE, TopAbs_SHELL, TopAbs_SOLID, TopAbs_COMPSOLID })
        {
            Assert.Equal(CountSubShapes(theExpected, aType), CountSubShapes(theActual, aType));
        }

        // vertex positions, in exploration order
        var anE = new TopExp_Explorer(theExpected, TopAbs_VERTEX);
        var anA = new TopExp_Explorer(theActual, TopAbs_VERTEX);
        for (; anE.More(); anE.Next(), anA.Next())
        {
            Assert.True(anA.More());
            Point3D aP1 = BRep_Tool.Pnt(TopoDS.Vertex(anE.Current()));
            Point3D aP2 = BRep_Tool.Pnt(TopoDS.Vertex(anA.Current()));
            Assert.True(aP1.Distance(aP2) < 1e-12, $"{aP1} {aP2}");
        }
    }

    private static void AssertFrozen(in TopoDS_Shape theShape)
    {
        Assert.True(theShape.TShape()!.IsFrozen);
        for (var anIt = new TopExp_Explorer(theShape, TopAbs_VERTEX); anIt.More(); anIt.Next())
        {
            Assert.True(anIt.Current().TShape()!.IsFrozen);
        }

        for (var anIt = new TopExp_Explorer(theShape, TopAbs_EDGE); anIt.More(); anIt.Next())
        {
            Assert.True(anIt.Current().TShape()!.IsFrozen);
        }

        for (var anIt = new TopExp_Explorer(theShape, TopAbs_FACE); anIt.More(); anIt.Next())
        {
            Assert.True(anIt.Current().TShape()!.IsFrozen);
            Triangulation? aTri = BRep_Tool.Triangulation(TopoDS.Face(anIt.Current()), out _);
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
        foreach ((string aName, TopoDS_Shape aShape) in BinTestShapes.All())
        {
            byte[] aB1 = WriteSet(aShape, theVersion, theWithTriangles, theWithNormals);
            TopoDS_Shape aS1 = ReadSet(aB1);
            AssertSameStructure(aShape, aS1);
            AssertFrozen(aS1);
            byte[] aB2 = WriteSet(aS1, theVersion, theWithTriangles, theWithNormals);
            TopoDS_Shape aS2 = ReadSet(aB2);
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
        foreach ((string aName, TopoDS_Shape aShape) in BinTestShapes.All())
        {
            byte[] aB1 = WriteShapes(theWithTriangles, theWithNormals, aShape);
            TopoDS_Shape aS1 = ReadShapes(aB1, 1)[0];
            AssertSameStructure(aShape, aS1);
            AssertFrozen(aS1);
            byte[] aB2 = WriteShapes(theWithTriangles, theWithNormals, aS1);
            TopoDS_Shape aS2 = ReadShapes(aB2, 1)[0];
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
        TopoDS_Shape aBox = BinTestShapes.MeshedBox(true, true);
        TopoDS_Shape aFace = new TopExp_Explorer(aBox, TopAbs_FACE).Current();
        var aT = new Transform3D();
        aT.SetTranslation(new Vector3D(0, 0, 7));
        TopoDS_Shape aMoved = aFace.Moved(new Placement(aT));
        byte[] aBytes = WriteShapes(true, true, aBox, aMoved, aBox);
        byte[] aBoxOnly = WriteShapes(true, true, aBox);
        // the second and third shapes are references: a reference, a location, an orientation
        Assert.True(aBytes.Length < aBoxOnly.Length + 120, $"{aBytes.Length} {aBoxOnly.Length}");

        TopoDS_Shape[] aRead = ReadShapes(aBytes, 3);
        TopoDS_Shape aReadFace = new TopExp_Explorer(aRead[0], TopAbs_FACE).Current();
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
        TopoDS_Shape aShape = BinTestShapes.BoxWithRegularityAndPolygon();
        foreach (int aVersion in new[] { 2, 3, 4 })
        {
            byte[] aB1 = WriteSet(aShape, aVersion);
            TopoDS_Shape aS1 = ReadSet(aB1);
            Assert.True(aB1.AsSpan().SequenceEqual(WriteSet(aS1, aVersion)));
            TopoDS_Edge anEdge = BinTestShapes.FirstEdge(aS1);
            var aFaces = BinTestShapes.FacesOf(aS1, anEdge);
            Assert.Equal(Continuity.G1, BRep_Tool.Continuity(anEdge, aFaces[0], aFaces[1]));
            Polygon3D? aPolygon = BRep_Tool.Polygon3D(BinTestShapes.NthEdge(aS1, 2), out _);
            Assert.NotNull(aPolygon);
            Assert.Equal(3, aPolygon.NodeCount);
            Assert.True(aPolygon.HasParameters);
            Assert.Equal(0.01, aPolygon.Deflection());
            Assert.True(aPolygon.IsFrozen);
        }

        byte[] aW1 = WriteShapes(true, false, aShape);
        Assert.True(aW1.AsSpan().SequenceEqual(WriteShapes(true, false, ReadShapes(aW1, 1)[0])));

        // without triangles, 3D polygons aren't written
        TopoDS_Shape aBare = ReadSet(WriteSet(aShape, 4, theWithTriangles: false));
        Assert.Null(BRep_Tool.Polygon3D(BinTestShapes.NthEdge(aBare, 2), out _));
    }

    [Fact]
    public void Triangulations_keep_their_uv_nodes_and_normals()
    {
        TopoDS_Shape aShape = BinTestShapes.MeshedBox(true, true);
        foreach ((bool aWithNormals, bool anExpectNormals) in new[] { (true, true), (false, false) })
        {
            TopoDS_Shape aRead = ReadSet(WriteSet(aShape, 4, true, aWithNormals));
            TopoDS_Face aFace = TopoDS.Face(new TopExp_Explorer(aRead, TopAbs_FACE).Current());
            Triangulation? aTri = BRep_Tool.Triangulation(aFace, out _);
            Assert.NotNull(aTri);
            Assert.Equal(4, aTri.NodeCount);
            Assert.Equal(2, aTri.TriangleCount);
            Assert.True(aTri.HasUVNodes);
            Assert.Equal(anExpectNormals, aTri.HasNormals);
            Assert.Equal(new Point2D(1, 0.5).X(), aTri.UVNode(2).X());

            // version 3 writes no normals
            TopoDS_Shape aV3 = ReadSet(WriteSet(aShape, 3, true, aWithNormals));
            Assert.False(BRep_Tool.Triangulation(TopoDS.Face(new TopExp_Explorer(aV3, TopAbs_FACE).Current()), out _)!.HasNormals);
        }

        // a face with only a triangulation keeps it, with its normals, even when triangles aren't asked for
        TopoDS_Shape aTriOnly = ReadSet(WriteSet(BinTestShapes.TriangulationOnlyFace(), 4, false, false));
        Triangulation? aOnly = BRep_Tool.Triangulation(TopoDS.Face(aTriOnly), out _);
        Assert.NotNull(aOnly);
        Assert.True(aOnly.HasNormals);
        aTriOnly = ReadShapes(WriteShapes(false, false, BinTestShapes.TriangulationOnlyFace()), 1)[0];
        Assert.True(BRep_Tool.Triangulation(TopoDS.Face(aTriOnly), out _)!.HasNormals);

        // polygons on triangulations, closed (the seam) and not
        TopoDS_Shape aCylinder = ReadSet(WriteSet(BinTestShapes.MeshedCylinder(), 4));
        int aNbClosed = 0;
        for (var anE = new TopExp_Explorer(aCylinder, TopAbs_EDGE); anE.More(); anE.Next())
        {
            BRep_TEdge aTE = (BRep_TEdge)anE.Current().TShape()!;
            foreach (BRep_CurveRepresentation aCR in aTE.CurveRepresentations())
            {
                if (aCR.IsPolygonOnClosedTriangulation())
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
        TopoDS_Solid aBox = TestShapes.Box(1, 1, 1);
        TopoDS_Shape aRead = ReadSet(WriteSet(aBox, 4));
        Assert.True(aRead.Free());
        for (var anIt = new TopExp_Explorer(aRead, TopAbs_FACE); anIt.More(); anIt.Next())
        {
            Assert.False(anIt.Current().Free());
        }

        aRead = ReadShapes(WriteShapes(true, false, aBox), 1)[0];
        for (var anIt = new TopExp_Explorer(aRead, TopAbs_FACE); anIt.More(); anIt.Next())
        {
            Assert.False(anIt.Current().Free());
        }
    }

    [Fact]
    public void Version_1_checks_faces_on_reading()
    {
        // version 1 has no UV points: reading recomputes them (BRepTools::Update) and the faces come back checked
        TopoDS_Shape aShape = TestShapes.Box(1, 2, 3);
        byte[] aB1 = WriteSet(aShape, 1);
        TopoDS_Shape aS1 = ReadSet(aB1);
        AssertSameStructure(aShape, aS1);
        for (var anIt = new TopExp_Explorer(aS1, TopAbs_FACE); anIt.More(); anIt.Next())
        {
            Assert.True(anIt.Current().Checked());
        }
    }

    [Fact]
    public void Point_representations_follow_OCCT()
    {
        TopoDS_Shape aShape = BinTestShapes.VertexRepresentations(includePointOnSurface: true);
        var anOriginal = ((BRep_TVertex)FindFreeVertex(aShape).TShape()!).PointRepresentations();
        Assert.Contains(anOriginal, theRep => theRep.IsPointOnCurve());
        Assert.Contains(anOriginal, theRep => theRep.IsPointOnCurveOnSurface());
        Assert.Contains(anOriginal, theRep => theRep.IsPointOnSurface());

        // the table-based format keeps them all, but swaps the parameters of a point on a surface (occt-issues.md)
        foreach (int aVersion in new[] { 2, 3, 4 })
        {
            TopoDS_Shape aRead = ReadSet(WriteSet(aShape, aVersion));
            var aReps = ((BRep_TVertex)FindFreeVertex(aRead).TShape()!).PointRepresentations();
            Assert.Equal(anOriginal.Count, aReps.Count);
            for (int i = 0; i < aReps.Count; i++)
            {
                AssertSameRepresentation(anOriginal[i], aReps[i]);
            }
        }

        // the position-based reader drops a point on a curve on a surface (it tests the surface for null the wrong way
        // round) and swaps the parameters of a point on a surface too
        TopoDS_Shape aRead4 = ReadShapes(WriteShapes(true, false, aShape), 1)[0];
        var aReps4 = ((BRep_TVertex)FindFreeVertex(aRead4).TShape()!).PointRepresentations();
        var anExpected4 = anOriginal.Where(theRep => !theRep.IsPointOnCurveOnSurface()).ToList();
        Assert.Equal(anExpected4.Count, aReps4.Count);
        for (int i = 0; i < aReps4.Count; i++)
        {
            AssertSameRepresentation(anExpected4[i], aReps4[i]);
        }
    }

    private static void AssertSameRepresentation(BRep_PointRepresentation theOriginal, BRep_PointRepresentation theRead)
    {
        Assert.Equal(theOriginal.GetType(), theRead.GetType());
        if (theOriginal.IsPointOnSurface())
        {
            Assert.Equal(theOriginal.Parameter2(), theRead.Parameter());
            Assert.Equal(theOriginal.Parameter(), theRead.Parameter2());
        }
        else
        {
            Assert.Equal(theOriginal.Parameter(), theRead.Parameter());
        }
    }

    private static TopoDS_Vertex FindFreeVertex(in TopoDS_Shape theCompound)
    {
        for (var anIt = new TopoDS_Iterator(theCompound); anIt.More(); anIt.Next())
        {
            if (anIt.Value().ShapeType() == TopAbs_VERTEX
                && BRep_Tool.Pnt(TopoDS.Vertex(anIt.Value())).Distance(new Point3D(0.25, 0.5, 0.75)) < 1e-12)
            {
                return TopoDS.Vertex(anIt.Value());
            }
        }

        throw new InvalidOperationException("no free vertex");
    }

    [Fact]
    public void Locations_elementary_and_complex_round_trip()
    {
        TopoDS_Shape aShape = BinTestShapes.CompoundWithLocations();
        var aExpected = new List<Transform3D>();
        for (var anIt = new TopoDS_Iterator(aShape); anIt.More(); anIt.Next())
        {
            aExpected.Add(anIt.Value().Location().Transformation());
        }

        foreach (TopoDS_Shape aRead in new[] { ReadSet(WriteSet(aShape, 4)), ReadShapes(WriteShapes(true, false, aShape), 1)[0] })
        {
            int i = 0;
            for (var anIt = new TopoDS_Iterator(aRead); anIt.More(); anIt.Next(), i++)
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
        var aSet = new BinTools_LocationSet();
        var aT1 = new Transform3D();
        aT1.SetTranslation(new Vector3D(1, 2, 3));
        var aT2 = new Transform3D();
        aT2.SetRotation(new Axis3D(new Point3D(), new Direction3D(1, 0, 0)), 1.0);
        var aL1 = new Placement(aT1);
        var aL2 = new Placement(aT2);
        Placement aComplex = aL2.Powered(3) * aL1.Powered(-2);
        Assert.Equal(0, aSet.Add(new Placement()));
        int anIndex = aSet.Add(aComplex);
        Assert.Equal(3, anIndex);
        Assert.Equal(3, aSet.NbLocations());
        var aStream = new MemoryStream();
        aSet.Write(aStream);
        aStream.Position = 0;
        var aRead2 = new BinTools_LocationSet();
        aRead2.Read(aStream);
        Assert.Equal(3, aRead2.NbLocations());
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
        var aSet = new BinTools_CurveSet();
        var aCurves = new List<Geom_Curve>();
        for (int i = 0; i < 330; i++)
        {
            Geom_Curve aCurve = GeomHash_PortTests.Curve(aRand, i % 11);
            aCurves.Add(aCurve);
            Assert.Equal(i + 1, aSet.Add(aCurve));
        }

        Assert.Equal(0, aSet.Add(null));
        var aStream = new MemoryStream();
        aSet.Write(aStream);
        byte[] aBytes = aStream.ToArray();
        Assert.True(aBytes.AsSpan(0, 11).SequenceEqual("Curves 330\n"u8));

        var aRead = new BinTools_CurveSet();
        aRead.Read(new MemoryStream(aBytes));
        var aHasher = new GeomHash_CurveHasher();
        for (int i = 1; i <= aCurves.Count; i++)
        {
            Geom_Curve? aCurve = aRead.Curve(i);
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
            if (aCurves[i - 1] is Geom_BezierCurve or Geom_BSplineCurve)
            {
                var anOne = new MemoryStream();
                BinTools_CurveSet.WriteCurve(aCurves[i - 1], new BinTools_OStream(anOne));
                var anOther = new MemoryStream();
                BinTools_CurveSet.WriteCurve(aRead.Curve(i)!, new BinTools_OStream(anOther));
                Assert.Equal(anOne.ToArray(), anOther.ToArray());
            }
        }

        // a single curve, read on its own, isn't frozen
        var aSingle = new MemoryStream();
        BinTools_CurveSet.WriteCurve(aCurves[7], new BinTools_OStream(aSingle));
        aSingle.Position = 0;
        BinTools_CurveSet.ReadCurve(aSingle, out Geom_Curve? aSingleRead);
        Assert.NotNull(aSingleRead);
        Assert.False(aSingleRead.IsFrozen);
        Assert.True(aHasher.Equals(aCurves[7], aSingleRead));
    }

    [Fact]
    public void Random_2d_curves_of_every_kind_round_trip()
    {
        var aRand = new Random(4302);
        var aSet = new BinTools_Curve2dSet();
        var aCurves = new List<Geom2d_Curve>();
        for (int i = 0; i < 300; i++)
        {
            Geom2d_Curve aCurve = GeomHash_PortTests.Curve2d(aRand, i % 10);
            aCurves.Add(aCurve);
            aSet.Add(aCurve);
        }

        var aStream = new MemoryStream();
        aSet.Write(aStream);
        var aRead = new BinTools_Curve2dSet();
        aRead.Read(new MemoryStream(aStream.ToArray()));
        var aHasher = new Geom2dHash_CurveHasher();
        for (int i = 1; i <= aCurves.Count; i++)
        {
            Geom2d_Curve? aCurve = aRead.Curve2d(i);
            Assert.NotNull(aCurve);
            Assert.True(aCurve.IsFrozen);
            Assert.Equal(aCurves[i - 1].DynamicType(), aCurve.DynamicType());
            Assert.True(aHasher.Equals(aCurves[i - 1], aCurve), $"curve {i}, {aCurve.DynamicTypeName()}");
        }

        // kinds without directions (which reading normalises again) write the same bytes
        for (int i = 1; i <= aCurves.Count; i++)
        {
            if (aCurves[i - 1] is Geom2d_BezierCurve or Geom2d_BSplineCurve)
            {
                var anOne = new MemoryStream();
                BinTools_Curve2dSet.WriteCurve2d(aCurves[i - 1], new BinTools_OStream(anOne));
                var anOther = new MemoryStream();
                BinTools_Curve2dSet.WriteCurve2d(aRead.Curve2d(i)!, new BinTools_OStream(anOther));
                Assert.Equal(anOne.ToArray(), anOther.ToArray());
            }
        }

        // the 2D set holds the null curve once, as OCCT's does
        var aWithNull = new BinTools_Curve2dSet();
        Assert.Equal(1, aWithNull.Add(null));
        Assert.Equal(1, aWithNull.Add(null));
        Assert.Null(aWithNull.Curve2d(1));
        Assert.Equal(1, aWithNull.Index(null));
    }

    [Fact]
    public void Random_surfaces_of_every_kind_round_trip()
    {
        var aRand = new Random(4303);
        var aSet = new BinTools_SurfaceSet();
        var aSurfaces = new List<Geom_Surface>();
        for (int i = 0; i < 360; i++)
        {
            Geom_Surface aSurface = GeomHash_PortTests.Surface(aRand, i % 12);
            aSurfaces.Add(aSurface);
            aSet.Add(aSurface);
        }

        var aStream = new MemoryStream();
        aSet.Write(aStream);
        var aRead = new BinTools_SurfaceSet();
        aRead.Read(new MemoryStream(aStream.ToArray()));
        var aHasher = new GeomHash_SurfaceHasher();
        var aKinds = new HashSet<Type>();
        for (int i = 1; i <= aSurfaces.Count; i++)
        {
            Geom_Surface? aSurface = aRead.Surface(i);
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
            if (aSurfaces[i - 1] is Geom_BezierSurface or Geom_BSplineSurface)
            {
                var anOne = new MemoryStream();
                BinTools_SurfaceSet.WriteSurface(aSurfaces[i - 1], new BinTools_OStream(anOne));
                var anOther = new MemoryStream();
                BinTools_SurfaceSet.WriteSurface(aRead.Surface(i)!, new BinTools_OStream(anOther));
                Assert.Equal(anOne.ToArray(), anOther.ToArray());
            }
        }
    }

    [Fact]
    public void Shapes_with_random_geometry_round_trip()
    {
        // edges on random curves and faces on random surfaces, through both formats
        var aRand = new Random(4304);
        var aB = new BRep_Builder();
        TopoDS_Compound aCompound = new();
        aB.MakeCompound(ref aCompound);
        for (int i = 0; i < 44; i++)
        {
            Geom_Curve aCurve = GeomHash_PortTests.Curve(aRand, i % 11);
            TopoDS_Edge anEdge = new();
            aB.MakeEdge(ref anEdge, aCurve, 1e-7);
            aB.Range(anEdge, 0.1, 0.9);
            aB.Add(aCompound, anEdge);

            Geom_Surface aSurface = GeomHash_PortTests.Surface(aRand, i % 12);
            TopoDS_Face aFace = new();
            aB.MakeFace(ref aFace, aSurface, 1e-7);
            TopoDS_Edge aPEdge = new();
            aB.MakeEdge(ref aPEdge);
            aB.UpdateEdge(aPEdge, GeomHash_PortTests.Curve2d(aRand, i % 10), aFace, 1e-7);
            aB.Range(aPEdge, aFace, 0.2, 0.8);
            TopoDS_Wire aWire = new();
            aB.MakeWire(ref aWire);
            aB.Add(aWire, aPEdge);
            aB.Add(aFace, aWire);
            aB.Add(aCompound, aFace);
        }

        // reading normalises directions again, so only the layout is compared: the same lengths, the same structure
        foreach (int aVersion in new[] { 2, 3, 4 })
        {
            byte[] aB1 = WriteSet(aCompound, aVersion);
            TopoDS_Shape aS1 = ReadSet(aB1);
            AssertSameStructure(aCompound, aS1);
            Assert.Equal(aB1.Length, WriteSet(aS1, aVersion).Length);
        }

        byte[] aW1 = WriteShapes(true, false, aCompound);
        TopoDS_Shape aR1 = ReadShapes(aW1, 1)[0];
        AssertSameStructure(aCompound, aR1);
        Assert.Equal(aW1.Length, WriteShapes(true, false, aR1).Length);
    }

    [Fact]
    public void Files_and_unseekable_streams()
    {
        TopoDS_Shape aShape = BinTestShapes.MeshedBox(true, false);
        string aDir = Path.Combine(Path.GetTempPath(), "occtnet-bintools-" + Guid.NewGuid().ToString("N"));
        Directory.CreateDirectory(aDir);
        try
        {
            string aFile = Path.Combine(aDir, "box.bin");
            Assert.True(BinTools.Write(aShape, aFile));
            byte[] aBytes = File.ReadAllBytes(aFile);
            Assert.Equal(WriteSet(aShape, 4), aBytes);
            Assert.True(BinTools.Read(out TopoDS_Shape aRead, aFile));
            AssertSameStructure(aShape, aRead);
            Assert.False(BinTools.Read(out TopoDS_Shape aMissing, Path.Combine(aDir, "missing.bin")));
            Assert.True(aMissing.IsNull());
            Assert.False(BinTools.Write(aShape, Path.Combine(aDir, "no", "such", "dir.bin")));

            // a stream that can't seek is buffered
            BinTools.Read(out TopoDS_Shape anUnseekable, new UnseekableStream(aBytes));
            Assert.Equal(aBytes, WriteSet(anUnseekable, 4));

            // the sets read a stream that can't seek: the character after a count is pushed back
            var aSet = new BinTools_CurveSet();
            aSet.Add(new Geom_Line(new Point3D(1, 2, 3), new Direction3D(0, 0, 1)));
            var aStream = new MemoryStream();
            aSet.Write(aStream);
            var aRead2 = new BinTools_CurveSet();
            aRead2.Read(new UnseekableStream(aStream.ToArray()));
            Assert.NotNull(aRead2.Curve(1));

            // writing to a stream that can't seek gives the same bytes
            var aWriteOnly = new UnseekableStream([]);
            BinTools.Write(aShape, aWriteOnly);
            Assert.Equal(aBytes, aWriteOnly.Written());
            var aWriterOnly = new UnseekableStream([]);
            new BinTools_ShapeWriter().Write(aShape, aWriterOnly);
            Assert.Equal(WriteShapes(false, false, aShape), aWriterOnly.Written());

            // the position-based reader needs to seek
            Assert.Throws<ProgramErrorException>(() =>
            {
                TopoDS_Shape aShapeRead = new();
                new BinTools_ShapeReader().Read(new UnseekableStream(aWriterOnly.Written()), ref aShapeRead);
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
        TopoDS_Solid aBox = TestShapes.Box(1, 1, 1);
        var aSet = new BinTools_ShapeSet();
        Assert.Equal(0, aSet.Add(new TopoDS_Shape()));
        int anIndex = aSet.Add(aBox);
        Assert.Equal(aSet.NbShapes(), anIndex);
        Assert.Equal(8 + 12 + 6 + 6 + 1 + 1, aSet.NbShapes());
        Assert.Equal(anIndex, aSet.Index(aBox));
        Assert.True(aSet.Shape(anIndex).IsSame(aBox));
        Assert.Equal(anIndex, aSet.Add(aBox.Reversed()));
        Assert.Same(aSet.Locations(), aSet.ChangeLocations());
        Assert.Equal((int)BinTools_FormatVersion.BinTools_FormatVersion_CURRENT, aSet.FormatNb());
        aSet.SetFormatNb(9);
        Assert.Equal(4, aSet.FormatNb());
        aSet.SetFormatNb(2);
        Assert.Equal(2, aSet.FormatNb());
        Assert.False(aSet.IsWithTriangles());
        aSet.Clear();
        Assert.Equal(0, aSet.NbShapes());
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
