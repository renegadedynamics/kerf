// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Features;
using Kerf.Engine.Features.Local;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Engine.Features.StatusError;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// BRepFeat's makers (unit 7I) on frozen inputs, as callers hold them: the inputs are never written, the results and
/// histories are frozen and answer for the caller's shapes. Volumes and counts are OCCT's on the same cases (checked
/// against native through tools/locoperec's feature records).
/// </summary>
public class BRepFeatTests
{
    private static int Count(in Shape theShape, ShapeType theType) => LocOpeTests.Count(theShape, theType);

    private static double Volume(in Shape theShape) => LocOpeTests.Volume(theShape);

    // A box 10 x 20 x 30 and a rectangle 4 x 6 on its top face (centred), as a face on the top face's plane
    private static (Shape Box, Face Top, Face Rect) BoxAndRect()
    {
        Shape aBox = LocOpeTests.Box();
        Face aTop = LocOpeTests.TopFace(aBox);
        Face aRect = Rect(3, 7, 7, 13, 30);
        return (aBox, aTop, aRect);
    }

    internal static Face Rect(double x1, double x2, double y1, double y2, double z)
    {
        var aPoly = new MakePolygon(new Point3D(x1, y1, z), new Point3D(x2, y1, z), new Point3D(x2, y2, z),
                                                   new Point3D(x1, y2, z), true);
        return new MakeFace(aPoly.Wire(), true).Face();
    }

    [Fact]
    public void MakePrism_Protrusion_OfALength()
    {
        (Shape aBox, Face aTop, Face aRect) = BoxAndRect();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox), aRectSnap = ShapeSnapshot.Of(aRect);
        var aPrism = new MakePrism(aBox, aRect, aTop, new Direction3D(0, 0, 1), 1, true);
        aPrism.Perform(5.0);
        Assert.True(aPrism.IsDone);
        Shape aRes = aPrism.Shape();
        Assert.Equal(6000 + 4 * 6 * 5, Volume(aRes), 6);
        Assert.True(new Analyzer(aRes).IsValid());
        Assert.True(LocOpeTests.AllFrozen(aRes));
        aSnap.AssertUnchanged(aBox);
        aRectSnap.AssertUnchanged(aRect);
        // OCCT: a prism of a length without glued faces never reaches GlobalPerform, which sets the status
        Assert.Equal(StatusError.NotInitialized, aPrism.CurrentStatusError());
        Assert.Equal(11, Count(aRes, ShapeType.Face));
    }

    [Fact]
    public void MakePrism_Depression_ThruAll()
    {
        (Shape aBox, Face aTop, Face aRect) = BoxAndRect();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aPrism = new MakePrism(aBox, aRect, aTop, new Direction3D(0, 0, -1), 0, true);
        aPrism.PerformThruAll();
        Assert.True(aPrism.IsDone);
        Shape aRes = aPrism.Shape();
        Assert.Equal(6000 - 4 * 6 * 30, Volume(aRes), 6);
        Assert.True(new Analyzer(aRes).IsValid());
        aSnap.AssertUnchanged(aBox);
    }

    [Fact]
    public void MakePrism_Depression_UntilAFace()
    {
        (Shape aBox, Face aTop, Face aRect) = BoxAndRect();
        Shape aBottom = new Explorer(aBox, ShapeType.Face).Current();
        for (var anExp = new Explorer(aBox, ShapeType.Face); anExp.More(); anExp.Next())
        {
            var aProps = new Properties();
            MassPropertiesBuilder.SurfaceProperties(anExp.Current(), aProps);
            if (Math.Abs(aProps.CentreOfMass().Z) < 1e-9)
            {
                aBottom = anExp.Current();
            }
        }
        var aPrism = new MakePrism(aBox, aRect, aTop, new Direction3D(0, 0, -1), 0, true);
        aPrism.Perform(aBottom);
        Assert.True(aPrism.IsDone);
        Assert.Equal(6000 - 4 * 6 * 30, Volume(aPrism.Shape()), 6);
    }

    // The bottom face of the box (z = 0)
    private static Shape Bottom(in Shape theBox)
    {
        for (var anExp = new Explorer(theBox, ShapeType.Face); anExp.More(); anExp.Next())
        {
            var aProps = new Properties();
            MassPropertiesBuilder.SurfaceProperties(anExp.Current(), aProps);
            if (Math.Abs(aProps.CentreOfMass().Z) < 1e-9)
            {
                return anExp.Current();
            }
        }
        throw new InvalidOperationException("no bottom face");
    }

    [Fact]
    public void MakeDPrism_Taper_TheInputsAreUnchanged_AndTheUntouchedFacesAreTheCallers()
    {
        (Shape aBox, Face aTop, Face aRect) = BoxAndRect();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox), aRectSnap = ShapeSnapshot.Of(aRect);
        var aPrism = new MakeDraftPrism(aBox, aRect, aTop, 5 * Math.PI / 180, 1, false);
        aPrism.Perform(4.0);
        Assert.True(aPrism.IsDone);
        Shape aRes = aPrism.Shape();
        Assert.True(LocOpeTests.AllFrozen(aRes));
        Assert.True(new Analyzer(aRes).IsValid());
        Assert.True(Volume(aRes) > 6000 && Volume(aRes) < 6000 + 4 * 6 * 4);
        aSnap.AssertUnchanged(aBox);
        aRectSnap.AssertUnchanged(aRect);
        // the bottom face is untouched: the caller's own, not modified, not deleted
        Shape aBottom = Bottom(aBox);
        Assert.True(LocOpeTests.Shares(aRes, aBottom));
        Assert.Empty(aPrism.Modified(aBottom));
        Assert.False(aPrism.IsDeleted(aBottom));
        // the top face is modified into the face with the boss's hole
        Assert.Single(aPrism.Modified(aTop));
    }

    [Fact]
    public void MakeRevol_And_MakePipe_OnFrozenInputs()
    {
        (Shape aBox, Face aTop, Face aRect) = BoxAndRect();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aRevol = new MakeRevol(aBox, aRect, aTop, new Axis3D(new Point3D(0, 15, 30), new Direction3D(1, 0, 0)), 1, true);
        aRevol.Perform(30 * Math.PI / 180);
        Assert.True(aRevol.IsDone);
        Assert.True(new Analyzer(aRevol.Shape()).IsValid());
        var aSpine = new MakeWire(new MakeEdge(new Point3D(5, 10, 30), new Point3D(5, 10, 35)).Edge()).Wire();
        var aPipe = new MakePipe(aBox, aRect, aTop, aSpine, 1, true);
        aPipe.Perform();
        Assert.True(aPipe.IsDone);
        Assert.Equal(6000 + 4 * 6 * 5, Volume(aPipe.Shape()), 6);
        aSnap.AssertUnchanged(aBox);
    }

    [Fact]
    public void MakeRevol_UntilAndAngle_StopsAtTheUntilFace()
    {
        // a groove: the rectangle x in [3, 7], y in [14, 18] on the top face turned about the top back edge (y = 20,
        // z = 30) into the box, up to the back face y = 20: a quarter annulus of radii 2 and 6, 4 wide. Up to that face,
        // or to the infinite plane through it, at an angle beyond it: the same groove (OCCT's PerformUntilAngle had
        // Perform(Until)'s branches the other way round: occt-issues.md, unit 7I's BRepFeat_MakeRevol row; decision
        // 16721)
        Shape aBox = LocOpeTests.Box();
        Face aTop = LocOpeTests.TopFace(aBox);
        Face aRect = Rect(3, 7, 14, 18, 30);
        Shape aBack = new Explorer(aBox, ShapeType.Face).Current();
        for (var anExp = new Explorer(aBox, ShapeType.Face); anExp.More(); anExp.Next())
        {
            var aProps = new Properties();
            MassPropertiesBuilder.SurfaceProperties(anExp.Current(), aProps);
            if (Math.Abs(aProps.CentreOfMass().Y - 20) < 1e-9)
            {
                aBack = anExp.Current();
            }
        }
        var anAxis = new Axis3D(new Point3D(0, 20, 30), new Direction3D(1, 0, 0));
        double anExpected = 6000 - Math.PI / 4 * (36 - 4) * 4;

        var anUntil = new MakeRevol(aBox, aRect, aTop, anAxis, 0, true);
        anUntil.Perform(aBack);
        Assert.True(anUntil.IsDone);
        Assert.Equal(anExpected, Volume(anUntil.Shape()), 6);

        var anAngle = new MakeRevol(aBox, aRect, aTop, anAxis, 0, true);
        anAngle.PerformUntilAngle(aBack, 0.75 * Math.PI);
        Assert.True(anAngle.IsDone);
        Assert.True(new Analyzer(anAngle.Shape()).IsValid());
        Assert.Equal(anExpected, Volume(anAngle.Shape()), 6);

        // up to the infinite plane y = 20 (a face without wires): the same groove, where OCCT glued and left the box
        var aB = new global::Kerf.Engine.Topology.Brep.BrepBuilder();
        Face anInfinite = new();
        aB.MakeFace(ref anInfinite, new PlaneSurface(new Plane3D(new Point3D(0, 20, 0), new Direction3D(0, 1, 0))), 1e-7);
        var aPlaneUntil = new MakeRevol(aBox, aRect, aTop, anAxis, 0, true);
        aPlaneUntil.PerformUntilAngle(anInfinite, 0.75 * Math.PI);
        Assert.True(aPlaneUntil.IsDone);
        Assert.Equal(anExpected, Volume(aPlaneUntil.Shape()), 6);
    }

    [Fact]
    public void MakeCylindricalHole_Blind_And_Through()
    {
        Shape aBox = LocOpeTests.Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aHole = new MakeCylindricalHole();
        aHole.Init(aBox, new Axis3D(new Point3D(5, 10, 31), new Direction3D(0, 0, -1)));
        aHole.PerformBlind(2, 10);
        aHole.Build();
        Assert.Equal(FeatureStatus.NoError, aHole.Status());
        Assert.Equal(6000 - Math.PI * 4 * 9, Volume(aHole.Shape()), 6);
        aHole.Init(aBox, new Axis3D(new Point3D(5, 10, 31), new Direction3D(0, 0, -1)));
        aHole.Perform(2);
        aHole.Build();
        Assert.Equal(6000 - Math.PI * 4 * 30, Volume(aHole.Shape()), 6);
        aSnap.AssertUnchanged(aBox);
        // a hole too long for a blind hole
        aHole.Init(aBox, new Axis3D(new Point3D(5, 10, 31), new Direction3D(0, 0, -1)));
        aHole.PerformBlind(2, 40);
        Assert.Equal(FeatureStatus.HoleTooLong, aHole.Status());
    }

    [Fact]
    public void SplitShape_And_Gluer_OnFrozenInputs()
    {
        (Shape aBox, Face aTop, Face aRect) = BoxAndRect();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aSplit = new SplitShape(aBox);
        aSplit.Add(BrepTools.OuterWire(aRect), aTop);
        aSplit.Build();
        Assert.True(aSplit.IsDone);
        Assert.Equal(7, Count(aSplit.Shape(), ShapeType.Face));
        Assert.Equal(2, aSplit.Modified(aTop).Count);
        aSnap.AssertUnchanged(aBox);

        // a box glued on the top face
        Shape aNew = new ApiMakePrism(aRect, new Vector3D(0, 0, 5)).Shape();
        var aGluer = new Gluer(aNew, aBox);
        aGluer.Bind(aRect, aTop);
        var aFE = new FindEdges(aRect, aTop);
        for (aFE.InitIterator(); aFE.More(); aFE.Next())
        {
            aGluer.Bind(aFE.EdgeFrom(), aFE.EdgeTo());
        }
        Assert.Equal(6000 + 4 * 6 * 5, Volume(aGluer.Shape()), 6);
        Assert.True(LocOpeTests.Shares(aGluer.Shape(), Bottom(aBox)));
        aSnap.AssertUnchanged(aBox);
    }
}
