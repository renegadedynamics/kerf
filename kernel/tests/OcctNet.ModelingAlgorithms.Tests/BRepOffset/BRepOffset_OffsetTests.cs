// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// BRepOffset_Offset (unit 7E) on frozen inputs: the offset of a face only reads it; the pipe and the sphere, which
/// write pcurves into the edges they are given, work on copies of frozen edges and leave the caller's unchanged, but
/// write in place into unfrozen ones (the 3D offset's working edges), as OCCT.
/// </summary>
public class BRepOffset_OffsetTests
{
    private static List<TopoDS_Shape> SubShapes(in TopoDS_Shape theShape, TopAbs_ShapeEnum theType)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, theType, aMap);
        var aList = new List<TopoDS_Shape>();
        for (int i = 1; i <= aMap.Extent(); i++)
        {
            aList.Add(aMap[i]);
        }
        return aList;
    }

    private static TopoDS_Shape Box() => new BRepPrimAPI_MakeBox(new gp_Pnt(0, 0, 0), new gp_Pnt(10, 20, 30)).Shape();

    private static TopoDS_Face FaceOnPlane(in TopoDS_Shape theShape, Func<gp_Pln, bool> thePredicate)
    {
        foreach (TopoDS_Shape aF in SubShapes(theShape, TopAbs_FACE))
        {
            if (BRep_Tool.Surface(TopoDS.Face(aF)) is Geom_Plane aP && thePredicate(aP.Pln()))
            {
                return TopoDS.Face(aF);
            }
        }
        throw new InvalidOperationException("no such face");
    }

    // The top (z = 30) and front (y = 0) faces of the box, and their common edge.
    private static (TopoDS_Face, TopoDS_Face, TopoDS_Edge) TopFrontAndEdge(in TopoDS_Shape theBox)
    {
        TopoDS_Face aTop = FaceOnPlane(theBox, p => Math.Abs(p.Location().Z() - 30) < 1e-9 && p.Axis().Direction().IsParallel(new gp_Dir(0, 0, 1), 1e-9));
        TopoDS_Face aFront = FaceOnPlane(theBox, p => Math.Abs(p.Location().Y()) < 1e-9 && p.Axis().Direction().IsParallel(new gp_Dir(0, 1, 0), 1e-9));
        foreach (TopoDS_Shape anE in SubShapes(aTop, TopAbs_EDGE))
        {
            foreach (TopoDS_Shape anE2 in SubShapes(aFront, TopAbs_EDGE))
            {
                if (anE.IsSame(anE2))
                {
                    return (aTop, aFront, TopoDS.Edge(anE));
                }
            }
        }
        throw new InvalidOperationException("no common edge");
    }

    [Fact]
    public void Face_OfAFrozenBoxFace_IsTheTranslatedFace_TheInputIsUnchanged()
    {
        TopoDS_Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        (TopoDS_Face aTop, _, TopoDS_Edge anEdge) = TopFrontAndEdge(aBox);

        var anOffset = new BRepOffset_Offset(aTop, 2.0);
        TopoDS_Face aFace = anOffset.Face();

        aSnap.AssertUnchanged(aBox);
        Assert.Equal(BRepOffset_Status.BRepOffset_Good, anOffset.Status());
        Assert.True(aFace.TShape()!.IsFrozen());
        Assert.True(new BRepCheck_Analyzer(aFace).IsValid());
        Assert.Equal(4, SubShapes(aFace, TopAbs_EDGE).Count);
        var aPln = ((Geom_Plane)BRep_Tool.Surface(aFace)!).Pln();
        Assert.Equal(32.0, aPln.Location().Z(), 9);

        TopoDS_Shape aGen = anOffset.Generated(anEdge);
        Assert.False(aGen.IsNull());
        Assert.True(aGen.TShape()!.IsFrozen());
        Assert.Contains(SubShapes(aFace, TopAbs_EDGE), e => e.IsSame(aGen));
        BRep_Tool.Range(TopoDS.Edge(aGen), out double aF, out double aL);
        gp_Pnt aP = BRep_Tool.Curve(TopoDS.Edge(aGen), out _, out _)!.Value(0.5 * (aF + aL));
        Assert.Equal(32.0, aP.Z(), 9);
        Assert.Equal(0.0, aP.Y(), 9);
    }

    [Fact]
    public void Face_OfAFrozenCylinderSide_IsTheLargerCylinder_TheInputIsUnchanged()
    {
        TopoDS_Shape aCyl = new BRepPrimAPI_MakeCylinder(5.0, 10.0).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCyl);
        TopoDS_Face aSide = TopoDS.Face(SubShapes(aCyl, TopAbs_FACE).First(f => BRep_Tool.Surface(TopoDS.Face(f)) is Geom_CylindricalSurface));

        var anOffset = new BRepOffset_Offset(aSide, 1.5);
        TopoDS_Face aFace = anOffset.Face();

        aSnap.AssertUnchanged(aCyl);
        Assert.True(new BRepCheck_Analyzer(aFace).IsValid());
        var aSurf = (Geom_CylindricalSurface)BRep_Tool.Surface(aFace)!;
        Assert.Equal(6.5, aSurf.Radius(), 9);
        foreach (TopoDS_Shape anE in SubShapes(aFace, TopAbs_EDGE))
        {
            Assert.NotNull(BRep_Tool.Curve(TopoDS.Edge(anE), out _, out _));
        }
    }

    [Fact]
    public void Pipe_OnFrozenEdges_WorksOnCopies_TheEdgesAreUnchanged()
    {
        TopoDS_Shape aBox = Box();
        (TopoDS_Face aTop, TopoDS_Face aFront, TopoDS_Edge anEdge) = TopFrontAndEdge(aBox);
        var anOF1 = new BRepOffset_Offset(aTop, 2.0);
        var anOF2 = new BRepOffset_Offset(aFront, 2.0);
        TopoDS_Edge anE1 = TopoDS.Edge(anOF1.Generated(anEdge)); // frozen
        TopoDS_Edge anE2 = TopoDS.Edge(anOF2.Generated(anEdge)); // frozen
        TopoDS_Shape aFace1 = anOF1.Face();
        TopoDS_Shape aFace2 = anOF2.Face();
        ShapeSnapshot aSnapBox = ShapeSnapshot.Of(aBox);
        ShapeSnapshot aSnap1 = ShapeSnapshot.Of(aFace1);
        ShapeSnapshot aSnap2 = ShapeSnapshot.Of(aFace2);

        var aPipe = new BRepOffset_Offset(anEdge, anE1, anE2, 2.0);
        TopoDS_Face aFace = aPipe.Face();

        aSnapBox.AssertUnchanged(aBox);
        aSnap1.AssertUnchanged(aFace1);
        aSnap2.AssertUnchanged(aFace2);
        Assert.True(aFace.TShape()!.IsFrozen());
        Assert.Equal(4, SubShapes(aFace, TopAbs_EDGE).Count);
        Assert.DoesNotContain(SubShapes(aFace, TopAbs_EDGE), e => e.IsSame(anE1) || e.IsSame(anE2));
        Assert.True(new BRepCheck_Analyzer(aFace).IsValid());
        Assert.IsType<Geom_CylindricalSurface>(((Geom_RectangularTrimmedSurface)BRep_Tool.Surface(aFace)!).BasisSurface());
    }

    [Fact]
    public void Pipe_OnUnfrozenEdges_WritesInPlace_AsOcct()
    {
        TopoDS_Shape aBox = Box();
        (TopoDS_Face aTop, TopoDS_Face aFront, TopoDS_Edge anEdge) = TopFrontAndEdge(aBox);
        var anOF1 = new BRepOffset_Offset(aTop, 2.0);
        var anOF2 = new BRepOffset_Offset(aFront, 2.0);
        TopoDS_Edge anE1 = TopoDS.Edge(anOF1.InternalGenerated(anEdge));
        TopoDS_Edge anE2 = TopoDS.Edge(anOF2.InternalGenerated(anEdge));
        Assert.False(anE1.TShape()!.IsFrozen());

        var aPipe = new BRepOffset_Offset(anEdge, anE1, anE2, 2.0);
        TopoDS_Face aFace = aPipe.InternalFace();

        // the pipe shares the offset faces' edges, which now have pcurves on it
        List<TopoDS_Shape> anEdges = SubShapes(aFace, TopAbs_EDGE);
        Assert.Contains(anEdges, e => e.IsSame(anE1));
        Assert.Contains(anEdges, e => e.IsSame(anE2));
        Assert.NotNull(BRep_Tool.CurveOnSurface(anE1, aFace, out _, out _));
        Assert.NotNull(BRep_Tool.CurveOnSurface(anE1, anOF1.InternalFace(), out _, out _));
        Assert.False(aFace.TShape()!.IsFrozen());
    }

    [Fact]
    public void Sphere_OnFrozenArcs_WorksOnCopies_TheArcsAreUnchanged()
    {
        const double R = 3.0;
        var aCenter = new gp_Pnt(0, 0, 0);
        TopoDS_Vertex aV = new BRepBuilderAPI_MakeVertex(aCenter).Vertex();
        TopoDS_Vertex aVX = new BRepBuilderAPI_MakeVertex(new gp_Pnt(R, 0, 0)).Vertex();
        TopoDS_Vertex aVY = new BRepBuilderAPI_MakeVertex(new gp_Pnt(0, R, 0)).Vertex();
        TopoDS_Vertex aVZ = new BRepBuilderAPI_MakeVertex(new gp_Pnt(0, 0, R)).Vertex();
        var aCXY = new gp_Circ(new gp_Ax2(aCenter, new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), R);
        var aCYZ = new gp_Circ(new gp_Ax2(aCenter, new gp_Dir(1, 0, 0), new gp_Dir(0, 1, 0)), R);
        var aCZX = new gp_Circ(new gp_Ax2(aCenter, new gp_Dir(0, 1, 0), new gp_Dir(0, 0, 1)), R);
        var aList = new NCollection_List<TopoDS_Shape>();
        aList.Append(new BRepBuilderAPI_MakeEdge(aCXY, aVX, aVY).Edge());
        aList.Append(new BRepBuilderAPI_MakeEdge(aCYZ, aVY, aVZ).Edge());
        aList.Append(new BRepBuilderAPI_MakeEdge(aCZX, aVZ, aVX).Edge());
        var aSnaps = aList.Select(e => (e, ShapeSnapshot.Of(e))).ToList();

        var aSphere = new BRepOffset_Offset(aV, aList, R);
        TopoDS_Face aFace = aSphere.Face();

        foreach ((TopoDS_Shape anE, ShapeSnapshot aSnap) in aSnaps)
        {
            aSnap.AssertUnchanged(anE);
        }
        Assert.IsType<Geom_SphericalSurface>(BRep_Tool.Surface(aFace));
        List<TopoDS_Shape> anEdges = SubShapes(aFace, TopAbs_EDGE);
        Assert.Equal(3, anEdges.Count);
        foreach (TopoDS_Shape anE in anEdges)
        {
            Assert.NotNull(BRep_Tool.CurveOnSurface(TopoDS.Edge(anE), aFace, out _, out _));
            Assert.DoesNotContain(aList, e => e.IsSame(anE));
        }
        // the vertices the construction left unchanged are the caller's own
        Assert.Contains(SubShapes(aFace, TopAbs_VERTEX), v => v.IsSame(aVX));
    }
}
