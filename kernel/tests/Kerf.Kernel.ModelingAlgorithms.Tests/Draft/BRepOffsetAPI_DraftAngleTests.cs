// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Booleans.Api;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Features.Api;
using Kerf.Engine.Features.Draft;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Engine.Features.Draft.DraftErrorStatus;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// Draft (unit 7G): BRepOffsetAPI_DraftAngle and Draft_Modification on frozen inputs. The modifier never writes into
/// the caller's shape, and CorrectWires works on a copy of the result where it holds the caller's sub-shapes
/// (decision 1044).
/// </summary>
public class BRepOffsetAPI_DraftAngleTests
{
    private static List<Shape> Faces(in Shape theShape) =>
        BRepOffset_MakeOffsetTests.SubShapes(theShape, ShapeType.Face);

    private static Face FaceWithNormal(in Shape theShape, Direction3D theNormal)
    {
        foreach (Shape aF in Faces(theShape))
        {
            if (Tool.Surface(ShapeCasts.Face(aF)) is PlaneSurface aPlane)
            {
                Direction3D aN = aPlane.Pln().Axis().Direction();
                if (aF.Orientation() == Orientation.Reversed)
                {
                    aN = aN.Reversed();
                }
                if (aN.IsEqual(theNormal, 1.0e-9))
                {
                    return ShapeCasts.Face(aF);
                }
            }
        }
        throw new InvalidOperationException("no such face");
    }

    [Fact]
    public void Angle_OfPlanesAndCones()
    {
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        Face aSide = FaceWithNormal(aBox, new Direction3D(1, 0, 0));
        Assert.Equal(0.0, DraftTools.Angle(aSide, new Direction3D(0, 0, 1)), 12);
        Face aTop = FaceWithNormal(aBox, new Direction3D(0, 0, 1));
        Assert.Equal(Math.PI / 2, DraftTools.Angle(aTop, new Direction3D(0, 0, 1)), 12);

        Shape aCone = new ApiMakeCone(5, 3, 10).Shape();
        Face aLateral = ShapeCasts.Face(Faces(aCone).First(f => Tool.Surface(ShapeCasts.Face(f)) is ConicalSurface));
        double anAngle = DraftTools.Angle(aLateral, new Direction3D(0, 0, 1));
        Assert.Equal(Math.Atan2(2, 10), anAngle, 9);

        Assert.Throws<DomainErrorException>(() => DraftTools.Angle(aLateral, new Direction3D(1, 0, 0)));
    }

    [Fact]
    public void Draft_OfABoxSide_TurnsTheFace_TheInputIsUnchanged()
    {
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        Face aSide = FaceWithNormal(aBox, new Direction3D(1, 0, 0));

        var aDraft = new DraftAngle(aBox);
        aDraft.Add(aSide, new Direction3D(0, 0, 1), 10.0 * Math.PI / 180.0, new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));
        Assert.True(aDraft.AddDone());
        aDraft.Build();
        Assert.True(aDraft.IsDone);

        aSnap.AssertUnchanged(aBox);
        Shape aResult = aDraft.Shape();
        Assert.True(BRepOffset_MakeOffsetTests.AllFrozen(aResult));
        Assert.True(new Analyzer(aResult).IsValid());
        // The side turned by 10 degrees about its bottom edge: the top is 10 tan(10) shorter (or longer).
        double aDelta = 0.5 * 10.0 * 10.0 * Math.Tan(10.0 * Math.PI / 180.0) * 10.0;
        double aVolume = BRepOffset_MakeOffsetTests.Volume(aResult);
        Assert.True(Math.Abs(Math.Abs(aVolume - 1000.0) - aDelta) < 1.0e-6, aVolume.ToString());

        // History: the side is generated into its drafted face, the bottom and the far side are the caller's faces.
        Assert.Single(aDraft.Generated(aSide));
        Assert.True(aDraft.Modified(aSide).Count == 0);
        Face aFar = FaceWithNormal(aBox, new Direction3D(-1, 0, 0));
        Assert.Contains(Faces(aResult), f => f.IsSame(aFar));
        Assert.Single(aDraft.ModifiedFaces());
        Assert.Single(aDraft.ConnectedFaces(aSide));
    }

    [Fact]
    public void Draft_OfACylinder_MakesACone()
    {
        Shape aCyl = new ApiMakeCylinder(5, 10).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCyl);
        Face aLateral = ShapeCasts.Face(Faces(aCyl).First(f => Tool.Surface(ShapeCasts.Face(f)) is CylindricalSurface));

        var aDraft = new DraftAngle(aCyl);
        aDraft.Add(aLateral, new Direction3D(0, 0, 1), 5.0 * Math.PI / 180.0, new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));
        Assert.True(aDraft.AddDone());
        aDraft.Build();
        Assert.True(aDraft.IsDone);
        aSnap.AssertUnchanged(aCyl);

        Shape aResult = aDraft.Shape();
        Assert.True(new Analyzer(aResult).IsValid());
        Assert.Contains(Faces(aResult), f => Tool.Surface(ShapeCasts.Face(f)) is ConicalSurface
                                             || Tool.Surface(ShapeCasts.Face(f)) is TrimmedSurface
                                                 { } t && t.BasisSurface() is ConicalSurface);
        double r1 = 5.0, r2 = 5.0 - 10.0 * Math.Tan(5.0 * Math.PI / 180.0);
        double aCone = Math.PI * 10.0 / 3.0 * (r1 * r1 + r1 * r2 + r2 * r2);
        double r3 = 5.0 + 10.0 * Math.Tan(5.0 * Math.PI / 180.0);
        double aCone2 = Math.PI * 10.0 / 3.0 * (r1 * r1 + r1 * r3 + r3 * r3);
        double aVolume = BRepOffset_MakeOffsetTests.Volume(aResult);
        Assert.True(Math.Abs(aVolume - aCone) < 1.0e-4 || Math.Abs(aVolume - aCone2) < 1.0e-4, aVolume.ToString());
    }

    [Fact]
    public void Add_OfAnUnsupportedFace_ReportsIt()
    {
        Shape aSphere = new MakeSphere(5).Shape();
        Face aFace = ShapeCasts.Face(Faces(aSphere)[0]);

        var aDraft = new DraftAngle(aSphere);
        aDraft.Add(aFace, new Direction3D(0, 0, 1), 0.1, new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));
        Assert.False(aDraft.AddDone());
        Assert.Equal(DraftErrorStatus.FaceRecomputation, aDraft.Status());
        Assert.True(aDraft.ProblematicShape().IsSame(aFace));
        Assert.Throws<ConstructionErrorException>(() => aDraft.Build());
        // OCCT: the face that failed was never added, so Remove can't take it back (occt-issues.md, unit 7G).
        Assert.Throws<NoSuchObjectException>(() => aDraft.Remove(aFace));
        aDraft.Clear();
        Assert.True(aDraft.AddDone());
    }

    [Fact]
    public void TinyAngles_AreIgnored()
    {
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        var aDraft = new DraftAngle(aBox);
        aDraft.Add(FaceWithNormal(aBox, new Direction3D(1, 0, 0)), new Direction3D(0, 0, 1), 1.0e-5,
                   new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));
        Assert.True(aDraft.AddDone());
        aDraft.Build();
        Assert.True(aDraft.IsDone);
        Assert.True(aDraft.Shape().IsSame(aBox));
    }

    [Fact]
    public void Add_WithoutAShape_Throws()
    {
        var aDraft = new DraftAngle();
        Shape aBox = new MakeBox(10, 10, 10).Shape();
        Assert.Throws<NullObjectException>(() => aDraft.Add(ShapeCasts.Face(Faces(aBox)[0]), new Direction3D(0, 0, 1), 0.1,
                                                             new Plane3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1))));
    }

    [Fact]
    public void Draft_NextToACylinderCutAcrossItsSeam_KeepsTheCallersShapes()
    {
        // A cylinder cut by a small box across its seam (u = 0, the +X side), with its top face drafted.
        Shape aCyl = new ApiMakeCylinder(5, 10).Shape();
        Shape aBox = new MakeBox(new Point3D(3, -1, 4), new Point3D(7, 1, 6)).Shape();
        Shape aCut = new Cut(aCyl, aBox).Shape();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCut);
        // The top face is drafted: the lateral face keeps its surface, and its hole wire is the caller's.
        Face aTop = FaceWithNormal(aCut, new Direction3D(0, 0, 1));

        var aDraft = new DraftAngle(aCut);
        aDraft.Add(aTop, new Direction3D(1, 0, 0), 5.0 * Math.PI / 180.0, new Plane3D(new Point3D(0, 0, 10), new Direction3D(1, 0, 0)));
        Assert.True(aDraft.AddDone());
        aDraft.Build();
        aSnap.AssertUnchanged(aCut);
        Assert.True(aDraft.IsDone);
        Shape aResult = aDraft.Shape();
        Assert.True(BRepOffset_MakeOffsetTests.AllFrozen(aResult));
        Assert.True(new Analyzer(aResult).IsValid());
        // the caller's bottom face (untouched by the draft) is in the result
        Face aBottom = FaceWithNormal(aCut, new Direction3D(0, 0, -1));
        Assert.Contains(Faces(aResult), f => f.IsSame(aBottom));
    }
}
