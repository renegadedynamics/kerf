// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Offsets;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// BRepOffset_Offset (unit 7E) on frozen inputs: the offset of a face only reads it; the pipe and the sphere, which
/// write pcurves into the edges they are given, work on copies of frozen edges and leave the caller's unchanged, but
/// write in place into unfrozen ones (the 3D offset's working edges), as OCCT.
/// </summary>
public class BRepOffset_OffsetTests
{
    private static List<Shape> SubShapes(in Shape theShape, ShapeType theType)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, theType, aMap);
        var aList = new List<Shape>();
        for (int i = 0; i < aMap.Count; i++)
        {
            aList.Add(aMap[i]);
        }
        return aList;
    }

    private static Shape Box() => new MakeBox(new Point3D(0, 0, 0), new Point3D(10, 20, 30)).Shape();

    private static Face FaceOnPlane(in Shape theShape, Func<Plane3D, bool> thePredicate)
    {
        foreach (Shape aF in SubShapes(theShape, ShapeType.Face))
        {
            if (Tool.Surface(ShapeCasts.Face(aF)) is PlaneSurface aP && thePredicate(aP.Pln()))
            {
                return ShapeCasts.Face(aF);
            }
        }
        throw new InvalidOperationException("no such face");
    }

    // The top (z = 30) and front (y = 0) faces of the box, and their common edge.
    private static (Face, Face, Edge) TopFrontAndEdge(in Shape theBox)
    {
        Face aTop = FaceOnPlane(theBox, p => Math.Abs(p.Location().Z() - 30) < 1e-9 && p.Axis().Direction().IsParallel(new Direction3D(0, 0, 1), 1e-9));
        Face aFront = FaceOnPlane(theBox, p => Math.Abs(p.Location().Y()) < 1e-9 && p.Axis().Direction().IsParallel(new Direction3D(0, 1, 0), 1e-9));
        foreach (Shape anE in SubShapes(aTop, ShapeType.Edge))
        {
            foreach (Shape anE2 in SubShapes(aFront, ShapeType.Edge))
            {
                if (anE.IsSame(anE2))
                {
                    return (aTop, aFront, ShapeCasts.Edge(anE));
                }
            }
        }
        throw new InvalidOperationException("no common edge");
    }

    [Fact]
    public void Face_OfAFrozenBoxFace_IsTheTranslatedFace_TheInputIsUnchanged()
    {
        Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        (Face aTop, _, Edge anEdge) = TopFrontAndEdge(aBox);

        var anOffset = new Offset(aTop, 2.0);
        Face aFace = anOffset.Face();

        aSnap.AssertUnchanged(aBox);
        Assert.Equal(OffsetStatus.Good, anOffset.Status());
        Assert.True(aFace.TShape()!.IsFrozen);
        Assert.True(new Analyzer(aFace).IsValid());
        Assert.Equal(4, SubShapes(aFace, ShapeType.Edge).Count);
        var aPln = ((PlaneSurface)Tool.Surface(aFace)!).Pln();
        Assert.Equal(32.0, aPln.Location().Z(), 9);

        Shape aGen = anOffset.Generated(anEdge);
        Assert.False(aGen.IsNull);
        Assert.True(aGen.TShape()!.IsFrozen);
        Assert.Contains(SubShapes(aFace, ShapeType.Edge), e => e.IsSame(aGen));
        Tool.Range(ShapeCasts.Edge(aGen), out double aF, out double aL);
        Point3D aP = Tool.Curve(ShapeCasts.Edge(aGen), out _, out _)!.Value(0.5 * (aF + aL));
        Assert.Equal(32.0, aP.Z(), 9);
        Assert.Equal(0.0, aP.Y(), 9);
    }

    [Fact]
    public void Face_OfAFrozenCylinderSide_IsTheLargerCylinder_TheInputIsUnchanged()
    {
        Shape aCyl = new ApiMakeCylinder(5.0, 10.0).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCyl);
        Face aSide = ShapeCasts.Face(SubShapes(aCyl, ShapeType.Face).First(f => Tool.Surface(ShapeCasts.Face(f)) is CylindricalSurface));

        var anOffset = new Offset(aSide, 1.5);
        Face aFace = anOffset.Face();

        aSnap.AssertUnchanged(aCyl);
        Assert.True(new Analyzer(aFace).IsValid());
        var aSurf = (CylindricalSurface)Tool.Surface(aFace)!;
        Assert.Equal(6.5, aSurf.Radius(), 9);
        foreach (Shape anE in SubShapes(aFace, ShapeType.Edge))
        {
            Assert.NotNull(Tool.Curve(ShapeCasts.Edge(anE), out _, out _));
        }
    }

    [Fact]
    public void Pipe_OnFrozenEdges_WorksOnCopies_TheEdgesAreUnchanged()
    {
        Shape aBox = Box();
        (Face aTop, Face aFront, Edge anEdge) = TopFrontAndEdge(aBox);
        var anOF1 = new Offset(aTop, 2.0);
        var anOF2 = new Offset(aFront, 2.0);
        Edge anE1 = ShapeCasts.Edge(anOF1.Generated(anEdge)); // frozen
        Edge anE2 = ShapeCasts.Edge(anOF2.Generated(anEdge)); // frozen
        Shape aFace1 = anOF1.Face();
        Shape aFace2 = anOF2.Face();
        ShapeSnapshot aSnapBox = ShapeSnapshot.Of(aBox);
        ShapeSnapshot aSnap1 = ShapeSnapshot.Of(aFace1);
        ShapeSnapshot aSnap2 = ShapeSnapshot.Of(aFace2);

        var aPipe = new Offset(anEdge, anE1, anE2, 2.0);
        Face aFace = aPipe.Face();

        aSnapBox.AssertUnchanged(aBox);
        aSnap1.AssertUnchanged(aFace1);
        aSnap2.AssertUnchanged(aFace2);
        Assert.True(aFace.TShape()!.IsFrozen);
        Assert.Equal(4, SubShapes(aFace, ShapeType.Edge).Count);
        Assert.DoesNotContain(SubShapes(aFace, ShapeType.Edge), e => e.IsSame(anE1) || e.IsSame(anE2));
        Assert.True(new Analyzer(aFace).IsValid());
        Assert.IsType<CylindricalSurface>(((TrimmedSurface)Tool.Surface(aFace)!).BasisSurface());
    }

    [Fact]
    public void Pipe_OnUnfrozenEdges_WritesInPlace_AsOcct()
    {
        Shape aBox = Box();
        (Face aTop, Face aFront, Edge anEdge) = TopFrontAndEdge(aBox);
        var anOF1 = new Offset(aTop, 2.0);
        var anOF2 = new Offset(aFront, 2.0);
        Edge anE1 = ShapeCasts.Edge(anOF1.InternalGenerated(anEdge));
        Edge anE2 = ShapeCasts.Edge(anOF2.InternalGenerated(anEdge));
        Assert.False(anE1.TShape()!.IsFrozen);

        var aPipe = new Offset(anEdge, anE1, anE2, 2.0);
        Face aFace = aPipe.InternalFace();

        // the pipe shares the offset faces' edges, which now have pcurves on it
        List<Shape> anEdges = SubShapes(aFace, ShapeType.Edge);
        Assert.Contains(anEdges, e => e.IsSame(anE1));
        Assert.Contains(anEdges, e => e.IsSame(anE2));
        Assert.NotNull(Tool.CurveOnSurface(anE1, aFace, out _, out _));
        Assert.NotNull(Tool.CurveOnSurface(anE1, anOF1.InternalFace(), out _, out _));
        Assert.False(aFace.TShape()!.IsFrozen);
    }

    [Fact]
    public void Sphere_OnFrozenArcs_WorksOnCopies_TheArcsAreUnchanged()
    {
        const double R = 3.0;
        var aCenter = new Point3D(0, 0, 0);
        Vertex aV = new MakeVertex(aCenter).Vertex();
        Vertex aVX = new MakeVertex(new Point3D(R, 0, 0)).Vertex();
        Vertex aVY = new MakeVertex(new Point3D(0, R, 0)).Vertex();
        Vertex aVZ = new MakeVertex(new Point3D(0, 0, R)).Vertex();
        var aCXY = new Circle3D(new Frame3D(aCenter, new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), R);
        var aCYZ = new Circle3D(new Frame3D(aCenter, new Direction3D(1, 0, 0), new Direction3D(0, 1, 0)), R);
        var aCZX = new Circle3D(new Frame3D(aCenter, new Direction3D(0, 1, 0), new Direction3D(0, 0, 1)), R);
        var aList = new NCollection_List<Shape>();
        aList.Append(new MakeEdge(aCXY, aVX, aVY).Edge());
        aList.Append(new MakeEdge(aCYZ, aVY, aVZ).Edge());
        aList.Append(new MakeEdge(aCZX, aVZ, aVX).Edge());
        var aSnaps = aList.Select(e => (e, ShapeSnapshot.Of(e))).ToList();

        var aSphere = new Offset(aV, aList, R);
        Face aFace = aSphere.Face();

        foreach ((Shape anE, ShapeSnapshot aSnap) in aSnaps)
        {
            aSnap.AssertUnchanged(anE);
        }
        Assert.IsType<SphericalSurface>(Tool.Surface(aFace));
        List<Shape> anEdges = SubShapes(aFace, ShapeType.Edge);
        Assert.Equal(3, anEdges.Count);
        foreach (Shape anE in anEdges)
        {
            Assert.NotNull(Tool.CurveOnSurface(ShapeCasts.Edge(anE), aFace, out _, out _));
            Assert.DoesNotContain(aList, e => e.IsSame(anE));
        }
        // the vertices the construction left unchanged are the caller's own
        Assert.Contains(SubShapes(aFace, ShapeType.Vertex), v => v.IsSame(aVX));
    }
}
