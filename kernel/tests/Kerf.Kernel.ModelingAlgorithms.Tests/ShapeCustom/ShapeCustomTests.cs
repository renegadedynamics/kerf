// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Healing.Build;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Healing.Custom;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Healing.Messages;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Kernel.ModelingData.Tests;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.ShapeType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeCustom's package functions and modifications (unit 7A) on frozen inputs (decision 973): the caller's shape is
/// never written, the result is frozen, a shape needing nothing comes back as itself, the sub-shapes a modification
/// leaves alone are the caller's own, compounds whose children share sub-shapes are modified, and a modifier reused
/// across public calls throws rather than change an earlier result.
/// </summary>
public class ShapeCustomTests
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

    private static bool AllFrozen(in Shape theShape)
    {
        var aMap = new IndexedSet<Shape>(ShapeHasher.Instance);
        Exploration.MapShapes(theShape, aMap);
        return aMap.All(s => s.TShape()!.IsFrozen);
    }

    private static Surface BasisOf(Surface theSurface)
        => theSurface is TrimmedSurface aRTS ? aRTS.BasisSurface : theSurface;

    private static Shape Mirrored(in Shape theShape)
    {
        var aT = new Transform3D();
        aT = aT.WithMirror(new Frame3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)));
        return new Transform(theShape, aT, true).Shape();
    }

    private static Shape Box() => new MakeBox(new Point3D(1, 2, 3), new Point3D(11, 12, 13)).Shape();

    private static Shape Compound(params Shape[] theShapes)
    {
        var B = new BrepBuilder();
        var aC = new Compound();
        B.MakeCompound(ref aC);
        foreach (Shape aS in theShapes)
        {
            B.Add(aC, aS);
        }
        aC.TShape()!.FreezeGraph();
        return aC;
    }

    [Fact]
    public void DirectFaces_OnAMirroredBox_MakesEveryFaceDirect_TheInputIsUnchanged()
    {
        Shape aBox = Mirrored(Box());
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        foreach (Shape aF in SubShapes(aBox, ShapeType.Face))
        {
            Assert.False(((ElementarySurface)Tool.Surface(ShapeCasts.Face(aF))!).Position.IsDirect);
        }

        Shape aResult = Customize.DirectFaces(aBox);

        aSnap.AssertUnchanged(aBox);
        Assert.True(AllFrozen(aResult));
        Assert.Equal(6, SubShapes(aResult, ShapeType.Face).Count);
        Assert.Equal(12, SubShapes(aResult, ShapeType.Edge).Count);
        foreach (Shape aF in SubShapes(aResult, ShapeType.Face))
        {
            Assert.True(((ElementarySurface)Tool.Surface(ShapeCasts.Face(aF))!).Position.IsDirect);
        }
        Assert.True(new Analyzer(aResult).IsValid());
        Assert.Equal(1000.0, VolumeOf(aResult), 6);
    }

    private static double VolumeOf(in Shape theShape)
    {
        var aProps = new Properties();
        MassPropertiesBuilder.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    [Fact]
    public void DirectFaces_OnADirectShape_GivesBackTheInputItself()
    {
        Shape aBox = Box();
        Shape aResult = Customize.DirectFaces(aBox);
        Assert.Same(aBox.TShape(), aResult.TShape());
    }

    [Fact]
    public void DirectFaces_OnACompoundOfFacesSharingEdges_BuildsOnTheModifiersOwnEdges()
    {
        // each face is a child of the compound, modified in turn by the same modifier: a later face adds its pcurve to
        // an edge an earlier face's result holds (the reason for the internal accessors, decision 973)
        Shape aBox = Mirrored(Box());
        Shape aFaces = Compound([.. SubShapes(aBox, ShapeType.Face)]);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFaces);

        Shape aResult = Customize.DirectFaces(aFaces);

        aSnap.AssertUnchanged(aFaces);
        Assert.True(AllFrozen(aResult));
        Assert.Equal(6, SubShapes(aResult, ShapeType.Face).Count);
        Assert.Equal(12, SubShapes(aResult, ShapeType.Edge).Count); // still shared
        foreach (Shape anE in SubShapes(aResult, ShapeType.Edge))
        {
            // two pcurves, one per face
            Assert.Equal(2, ((BrepTEdge)anE.TShape()!).CurveRepresentations().Count(c => c is GCurve g && g.IsCurveOnSurface()));
        }
    }

    [Fact]
    public void ApplyModifier_ReusedAcrossPublicCalls_OnSharedSubShapes_Throws_TheFirstResultIsUnchanged()
    {
        Shape aBox = Mirrored(Box());
        List<Shape> aFaces = SubShapes(aBox, ShapeType.Face);
        var aDM = new DirectModification();
        var aContext = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        var aMD = new Modifier();

        Shape aFirst = Customize.ApplyModifier(aFaces[0], aDM, aContext, aMD);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFirst);
        // the second face shares an edge with the first: the modifier would add a pcurve to the first result's edge
        Shape aNeighbour = aFaces.Skip(1).First(f => SubShapes(f, ShapeType.Edge).Any(e => SubShapes(aFaces[0], ShapeType.Edge).Any(e0 => e0.IsSame(e))));
        EngineAssert.Throws<ImmutableObjectException>(() => Customize.ApplyModifier(aNeighbour, aDM, aContext, aMD));
        aSnap.AssertUnchanged(aFirst);
    }

    [Fact]
    public void ApplyModifier_RecordsTheReplacementsOfTheCallersSubShapes()
    {
        Shape aBox = Mirrored(Box());
        var aReShape = new ReShapeBuilder();
        var aContext = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        Shape aResult = Customize.ApplyModifier(aBox, new DirectModification(), aContext, new Modifier(),
                                                         default, aReShape);
        Assert.True(AllFrozen(aResult));
        foreach (Shape aF in SubShapes(aBox, ShapeType.Face))
        {
            Shape aNew = aReShape.Value(aF);
            Assert.False(aNew.IsSame(aF));
            Assert.Contains(SubShapes(aResult, ShapeType.Face), f => f.IsSame(aNew));
        }
        Assert.True(aReShape.Value(aBox).IsSame(aResult));
    }

    [Fact]
    public void ApplyModifier_ACompoundSharingAChild_IsModifiedOnce()
    {
        Shape aBox = Mirrored(Box());
        Shape aCompound = Compound(aBox, aBox.Located(new Placement(new Transform3D())), aBox);
        var aContext = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        Shape aResult = Customize.ApplyModifier(aCompound, new DirectModification(), aContext, new Modifier());
        var aChildren = new List<Shape>();
        for (var it = new Iterator(aResult); it.More(); it.Next())
        {
            aChildren.Add(it.Value());
        }
        Assert.Equal(3, aChildren.Count);
        Assert.All(aChildren, c => Assert.Same(aChildren[0].TShape(), c.TShape()));
        Assert.False(aChildren[0].IsSame(aBox));
    }

    [Fact]
    public void ScaleShape_ScalesTheShapeAndItsOwnTolerances()
    {
        Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        Shape aResult = Customize.ScaleShape(aBox, 0.5);
        aSnap.AssertUnchanged(aBox);
        Assert.True(AllFrozen(aResult));
        Assert.Equal(125.0, VolumeOf(aResult), 6);
        foreach (Shape aV in SubShapes(aResult, ShapeType.Vertex))
        {
            // the vertex's own tolerance (1e-7) halved, below Precision::Confusion()
            Assert.Equal(0.5e-7, ((BrepTVertex)aV.TShape()!).Tolerance(), 15);
        }
    }

    [Fact]
    public void ConvertToRevolution_ConvertsElementarySurfaces_KeepsTheBoxOfACompound()
    {
        Shape aBox = Box();
        Shape aSphere = new MakeSphere(new Point3D(30, 0, 0), 5).Shape();
        Shape aCylinder = new ApiMakeCylinder(new Frame3D(new Point3D(-30, 0, 0), new Direction3D(0, 0, 1)), 3, 7).Shape();
        Shape aCompound = Compound(aBox, aSphere, aCylinder);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCompound);
        double aVolume = VolumeOf(aCompound);

        Shape aResult = Customize.ConvertToRevolution(aCompound);

        aSnap.AssertUnchanged(aCompound);
        Assert.True(AllFrozen(aResult));
        var aChildren = new List<Shape>();
        for (var it = new Iterator(aResult); it.More(); it.Next())
        {
            aChildren.Add(it.Value());
        }
        Assert.Same(aBox.TShape(), aChildren[0].TShape()); // planes only: the caller's own
        int aRevolutions = 0;
        foreach (Shape aF in SubShapes(aResult, ShapeType.Face))
        {
            aRevolutions += BasisOf(Tool.Surface(ShapeCasts.Face(aF))!) is RevolutionSurface ? 1 : 0;
        }
        Assert.Equal(2, aRevolutions); // the sphere and the cylinder's side
        Assert.Equal(aVolume, VolumeOf(aResult), 3);
    }

    [Fact]
    public void SweptToElementary_ConvertsARevolutionOfASegment_ToACylinder()
    {
        // (BRepPrimAPI_MakeRevol would make a cylinder itself)
        var aSurface = new RevolutionSurface(new LineCurve3D(new Point3D(4, 0, 0), new Direction3D(0, 0, 1)),
                                                    new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));
        Shape aRevol = new MakeFace(aSurface, 0, 2 * Math.PI, 0, 10, global::Kerf.Engine.Foundation.Precision.Confusion()).Shape();
        Face aFace = ShapeCasts.Face(SubShapes(aRevol, ShapeType.Face)[0]);
        Assert.IsType<RevolutionSurface>(Tool.Surface(aFace));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aRevol);

        Shape aResult = Customize.SweptToElementary(aRevol);

        aSnap.AssertUnchanged(aRevol);
        Assert.True(AllFrozen(aResult));
        Surface aNew = Tool.Surface(ShapeCasts.Face(SubShapes(aResult, ShapeType.Face)[0]))!;
        Assert.IsType<CylindricalSurface>(aNew);
        Assert.Equal(4.0, ((CylindricalSurface)aNew).Radius, 12);
    }

    // OCCT's data/brep/bug26359_parabola.brep: a revolution of a parabola (a B-spline) with a degenerated edge at its
    // apex
    private const string RevolvedParabola = """
        CASCADE Topology V1, (c) Matra-Datavision
        Locations 0
        Curve2ds 4
        1 0 0 1 0 
        1 0 4 1 0 
        1 6.2831853071795862 0 0 1 
        1 0 0 0 1 
        Curves 2
        2 64 0 0 1 0 -0 0 1 0 0 -0 1 16
        7 0 0  2 3 2  0 0 0  0 8 0  64 16 0 
         0 3 4 3
        Polygon3D 0
        PolygonOnTriangulations 0
        Surfaces 1
        7 0 0 0 1 0 -0 
        8 0 4
        7 0 0  2 3 2  0 0 0  0 8 0  64 16 0 
         0 3 4 3
        Triangulations 0

        TShapes 8
        Ve
        1e-007
        0 0 0
        0 0

        0101101
        *
        Ed
         1e-007 1 1 1
        2  1 1 0 0 6.28318530717959
        0

        0101000
        +8 0 -8 0 *
        Ve
        1e-007
        64 16 0
        0 0

        0101101
        *
        Ed
         1e-007 1 1 0
        1  1 0 0 6.28318530717959
        2  2 1 0 0 6.28318530717959
        0

        0101000
        +6 0 -6 0 *
        Ed
         1e-007 1 1 0
        1  2 0 0 4
        3  3 4CN 1 0 0 4
        0

        0101000
        +8 0 -6 0 *
        Wi

        0101100
        +7 0 -5 0 -4 0 +4 0 *
        Fa
        0  1e-007 1 0

        0111000
        +3 0 *
        Co

        1100000
        +2 0 *

        +1 0
        """;

    /// <summary>
    /// SweptToElementary on a face of revolution with a degenerated edge (OCCT's bug26359_parabola.brep, one of many
    /// data shapes). BRepTools_Modifier asks NewCurve2d whether to copy an edge's vertices with a null new face, which
    /// OCCT dereferenced (an access violation natively, a NullReferenceException in the port; occt-issues.md, unit 7A's
    /// ShapeCustom_SweptToElementary rows; Kerf unit 10K1, decision 1437). The surface isn't elementary: the result
    /// is the same face, valid, with the same area.
    /// </summary>
    [Fact]
    public void SweptToElementary_OnAFaceWithADegeneratedEdge_Succeeds()
    {
        BrepTools.Read(out Shape aShape, new StringReader(RevolvedParabola));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aShape);

        Shape aResult = Customize.SweptToElementary(aShape);

        aSnap.AssertUnchanged(aShape);
        Assert.True(AllFrozen(aResult));
        Assert.True(new Analyzer(aResult).IsValid());
        Properties aBefore = new(), anAfter = new();
        MassPropertiesBuilder.SurfaceProperties(aShape, aBefore);
        MassPropertiesBuilder.SurfaceProperties(aResult, anAfter);
        Assert.Equal(aBefore.Mass(), anAfter.Mass(), 6);
    }

    [Fact]
    public void ConvertToBSpline_WithThePlaneMode_ConvertsPlanes()
    {
        Shape aBox = Box();
        Assert.Same(aBox.TShape(), Customize.ConvertToBSpline(aBox, true, true, true).TShape());
        Shape aResult = Customize.ConvertToBSpline(aBox, true, true, true, true);
        Assert.True(AllFrozen(aResult));
        foreach (Shape aF in SubShapes(aResult, ShapeType.Face))
        {
            Assert.IsType<BSplineSurface>(BasisOf(Tool.Surface(ShapeCasts.Face(aF))!));
        }
        Assert.Equal(1000.0, VolumeOf(aResult), 6);
    }

    [Fact]
    public void BSplineRestriction_AsToSplines_ConvertsEverySurface_TheInputIsUnchanged()
    {
        Shape aCylinder = new ApiMakeCylinder(new Frame3D(new Point3D(1, 2, 3), new Direction3D(0, 1, 1)), 3, 7).Shape();
        var aParameters = new RestrictionParameters();
        aParameters.ConvertPlane() = true;
        aParameters.ConvertCylindricalSurf() = true;
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCylinder);

        Shape aResult = Customize.BSplineRestriction(aCylinder, 1e-3, 1e-3, 3, 1, C0, C0, true, true, aParameters);

        aSnap.AssertUnchanged(aCylinder);
        Assert.True(AllFrozen(aResult));
        foreach (Shape aF in SubShapes(aResult, ShapeType.Face))
        {
            Surface aS = Tool.Surface(ShapeCasts.Face(aF))!;
            Assert.IsType<BSplineSurface>(aS);
            Assert.True(((BSplineSurface)aS).UDegree <= 3 && ((BSplineSurface)aS).VDegree <= 3);
        }
        Assert.Equal(Math.PI * 9 * 7, VolumeOf(aResult), 1);
    }

    [Fact]
    public void BSplineRestriction_TheModificationsErrorsAndSpans()
    {
        Shape aCylinder = new ApiMakeCylinder(3, 7).Shape();
        var aParameters = new RestrictionParameters();
        aParameters.ConvertCylindricalSurf() = true;
        var aBSR = new BSplineRestriction(true, true, true, 1e-3, 1e-5, C1, C2, 6, 10, true, false, aParameters);
        var aContext = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        Customize.ApplyModifier(aCylinder, aBSR, aContext, new Modifier());
        Assert.True(aBSR.OfSpanCount > 0);
        double aSurfaceError = aBSR.MaxErrors(out double a3d, out double a2d);
        Assert.True(aSurfaceError <= 1e-3 && aSurfaceError >= global::Kerf.Engine.Foundation.Precision.Confusion());
        Assert.Equal(aSurfaceError, aBSR.SurfaceError());
        Assert.Equal(a3d, aBSR.Curve3dError());
        Assert.Equal(a2d, aBSR.Curve2dError());
    }

    [Fact]
    public void Modifications_SendTheirMessages()
    {
        // what ShapeExtend.Init loads, loaded unconditionally: Init skips it once another test's Msg has stored an
        // unknown-keyword text under its probe keyword (see SHMessageTests)
        global::Kerf.Engine.Foundation.MsgFile.LoadFromString(ShapeMessages.Text);
        Shape aBox = Mirrored(Box());
        var aDM = new DirectModification();
        var aMsg = new MsgRegistrator();
        aDM.SetMsgRegistrator(aMsg);
        Assert.Same(aMsg, aDM.MsgRegistrator());
        var aContext = new Dictionary<Shape, Shape>(ShapeHasher.Instance);
        Customize.ApplyModifier(aBox, aDM, aContext, new Modifier());
        Assert.Equal(6, aMsg.MapShape().Count);
        foreach (Shape aF in SubShapes(aBox, ShapeType.Face))
        {
            Assert.Equal("Direction of Face of revolution corrected", aMsg.MapShape().GetValue(aF).First().Value());
        }
    }

    [Fact]
    public void RestrictionParameters_Defaults_EachAccessorItsOwnFlag()
    {
        var aP = new RestrictionParameters();
        Assert.Equal(15, aP.GMaxDegree());
        Assert.Equal(10000, aP.GMaxSeg());
        Assert.False(aP.ConvertPlane() || aP.ConvertBezierSurf() || aP.ConvertConicalSurf() || aP.ConvertSphericalSurf()
                     || aP.ConvertCylindricalSurf() || aP.ConvertToroidalSurf());
        Assert.True(aP.ConvertRevolutionSurf() && aP.ConvertExtrusionSurf() && aP.ConvertOffsetSurf() && aP.SegmentSurfaceMode()
                    && aP.ConvertCurve3d() && aP.ConvertOffsetCurv3d() && aP.ConvertCurve2d() && aP.ConvertOffsetCurv2d());
        // OCCT's accessors of the cylinders' and tori's flags read each other's fields, but each always the same one
        aP.ConvertToroidalSurf() = true;
        Assert.False(aP.ConvertCylindricalSurf());
        Assert.True(aP.ConvertToroidalSurf());
        aP.GMaxDegree() = 9;
        Assert.Equal(9, aP.GMaxDegree());
    }
}
