// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.ModelingData.Tests;
using static Kerf.Geometry.Continuity;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// ShapeCustom's package functions and modifications (unit 7A) on frozen inputs (decision 973): the caller's shape is
/// never written, the result is frozen, a shape needing nothing comes back as itself, the sub-shapes a modification
/// leaves alone are the caller's own, compounds whose children share sub-shapes are modified, and a modifier reused
/// across public calls throws rather than change an earlier result.
/// </summary>
public class ShapeCustomTests
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

    private static bool AllFrozen(in TopoDS_Shape theShape)
    {
        var aMap = new NCollection_IndexedMap<TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapes(theShape, aMap);
        return aMap.All(s => s.TShape()!.IsFrozen);
    }

    private static Geom_Surface BasisOf(Geom_Surface theSurface)
        => theSurface is Geom_RectangularTrimmedSurface aRTS ? aRTS.BasisSurface() : theSurface;

    private static TopoDS_Shape Mirrored(in TopoDS_Shape theShape)
    {
        var aT = new Transform3D();
        aT.SetMirror(new Frame3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)));
        return new BRepBuilderAPI_Transform(theShape, aT, true).Shape();
    }

    private static TopoDS_Shape Box() => new BRepPrimAPI_MakeBox(new Point3D(1, 2, 3), new Point3D(11, 12, 13)).Shape();

    private static TopoDS_Shape Compound(params TopoDS_Shape[] theShapes)
    {
        var B = new BRep_Builder();
        var aC = new TopoDS_Compound();
        B.MakeCompound(ref aC);
        foreach (TopoDS_Shape aS in theShapes)
        {
            B.Add(aC, aS);
        }
        aC.TShape()!.FreezeGraph();
        return aC;
    }

    [Fact]
    public void DirectFaces_OnAMirroredBox_MakesEveryFaceDirect_TheInputIsUnchanged()
    {
        TopoDS_Shape aBox = Mirrored(Box());
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        foreach (TopoDS_Shape aF in SubShapes(aBox, TopAbs_FACE))
        {
            Assert.False(((Geom_ElementarySurface)BRep_Tool.Surface(TopoDS.Face(aF))!).Position().Direct());
        }

        TopoDS_Shape aResult = ShapeCustom.DirectFaces(aBox);

        aSnap.AssertUnchanged(aBox);
        Assert.True(AllFrozen(aResult));
        Assert.Equal(6, SubShapes(aResult, TopAbs_FACE).Count);
        Assert.Equal(12, SubShapes(aResult, TopAbs_EDGE).Count);
        foreach (TopoDS_Shape aF in SubShapes(aResult, TopAbs_FACE))
        {
            Assert.True(((Geom_ElementarySurface)BRep_Tool.Surface(TopoDS.Face(aF))!).Position().Direct());
        }
        Assert.True(new BRepCheck_Analyzer(aResult).IsValid());
        Assert.Equal(1000.0, VolumeOf(aResult), 6);
    }

    private static double VolumeOf(in TopoDS_Shape theShape)
    {
        var aProps = new GProp_GProps();
        BRepGProp.VolumeProperties(theShape, aProps);
        return aProps.Mass();
    }

    [Fact]
    public void DirectFaces_OnADirectShape_GivesBackTheInputItself()
    {
        TopoDS_Shape aBox = Box();
        TopoDS_Shape aResult = ShapeCustom.DirectFaces(aBox);
        Assert.Same(aBox.TShape(), aResult.TShape());
    }

    [Fact]
    public void DirectFaces_OnACompoundOfFacesSharingEdges_BuildsOnTheModifiersOwnEdges()
    {
        // each face is a child of the compound, modified in turn by the same modifier: a later face adds its pcurve to
        // an edge an earlier face's result holds (the reason for the internal accessors, decision 973)
        TopoDS_Shape aBox = Mirrored(Box());
        TopoDS_Shape aFaces = Compound([.. SubShapes(aBox, TopAbs_FACE)]);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFaces);

        TopoDS_Shape aResult = ShapeCustom.DirectFaces(aFaces);

        aSnap.AssertUnchanged(aFaces);
        Assert.True(AllFrozen(aResult));
        Assert.Equal(6, SubShapes(aResult, TopAbs_FACE).Count);
        Assert.Equal(12, SubShapes(aResult, TopAbs_EDGE).Count); // still shared
        foreach (TopoDS_Shape anE in SubShapes(aResult, TopAbs_EDGE))
        {
            // two pcurves, one per face
            Assert.Equal(2, ((BRep_TEdge)anE.TShape()!).CurveRepresentations().Count(c => c is BRep_GCurve g && g.IsCurveOnSurface()));
        }
    }

    [Fact]
    public void ApplyModifier_ReusedAcrossPublicCalls_OnSharedSubShapes_Throws_TheFirstResultIsUnchanged()
    {
        TopoDS_Shape aBox = Mirrored(Box());
        List<TopoDS_Shape> aFaces = SubShapes(aBox, TopAbs_FACE);
        var aDM = new ShapeCustom_DirectModification();
        var aContext = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        var aMD = new BRepTools_Modifier();

        TopoDS_Shape aFirst = ShapeCustom.ApplyModifier(aFaces[0], aDM, aContext, aMD);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aFirst);
        // the second face shares an edge with the first: the modifier would add a pcurve to the first result's edge
        TopoDS_Shape aNeighbour = aFaces.Skip(1).First(f => SubShapes(f, TopAbs_EDGE).Any(e => SubShapes(aFaces[0], TopAbs_EDGE).Any(e0 => e0.IsSame(e))));
        Assert.Throws<TopoDS_LockedShape>(() => ShapeCustom.ApplyModifier(aNeighbour, aDM, aContext, aMD));
        aSnap.AssertUnchanged(aFirst);
    }

    [Fact]
    public void ApplyModifier_RecordsTheReplacementsOfTheCallersSubShapes()
    {
        TopoDS_Shape aBox = Mirrored(Box());
        var aReShape = new ShapeBuild_ReShape();
        var aContext = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopoDS_Shape aResult = ShapeCustom.ApplyModifier(aBox, new ShapeCustom_DirectModification(), aContext, new BRepTools_Modifier(),
                                                         default, aReShape);
        Assert.True(AllFrozen(aResult));
        foreach (TopoDS_Shape aF in SubShapes(aBox, TopAbs_FACE))
        {
            TopoDS_Shape aNew = aReShape.Value(aF);
            Assert.False(aNew.IsSame(aF));
            Assert.Contains(SubShapes(aResult, TopAbs_FACE), f => f.IsSame(aNew));
        }
        Assert.True(aReShape.Value(aBox).IsSame(aResult));
    }

    [Fact]
    public void ApplyModifier_ACompoundSharingAChild_IsModifiedOnce()
    {
        TopoDS_Shape aBox = Mirrored(Box());
        TopoDS_Shape aCompound = Compound(aBox, aBox.Located(new Placement(new Transform3D())), aBox);
        var aContext = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        TopoDS_Shape aResult = ShapeCustom.ApplyModifier(aCompound, new ShapeCustom_DirectModification(), aContext, new BRepTools_Modifier());
        var aChildren = new List<TopoDS_Shape>();
        for (var it = new TopoDS_Iterator(aResult); it.More(); it.Next())
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
        TopoDS_Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        TopoDS_Shape aResult = ShapeCustom.ScaleShape(aBox, 0.5);
        aSnap.AssertUnchanged(aBox);
        Assert.True(AllFrozen(aResult));
        Assert.Equal(125.0, VolumeOf(aResult), 6);
        foreach (TopoDS_Shape aV in SubShapes(aResult, TopAbs_VERTEX))
        {
            // the vertex's own tolerance (1e-7) halved, below Precision::Confusion()
            Assert.Equal(0.5e-7, ((BRep_TVertex)aV.TShape()!).Tolerance(), 15);
        }
    }

    [Fact]
    public void ConvertToRevolution_ConvertsElementarySurfaces_KeepsTheBoxOfACompound()
    {
        TopoDS_Shape aBox = Box();
        TopoDS_Shape aSphere = new BRepPrimAPI_MakeSphere(new Point3D(30, 0, 0), 5).Shape();
        TopoDS_Shape aCylinder = new BRepPrimAPI_MakeCylinder(new Frame3D(new Point3D(-30, 0, 0), new Direction3D(0, 0, 1)), 3, 7).Shape();
        TopoDS_Shape aCompound = Compound(aBox, aSphere, aCylinder);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCompound);
        double aVolume = VolumeOf(aCompound);

        TopoDS_Shape aResult = ShapeCustom.ConvertToRevolution(aCompound);

        aSnap.AssertUnchanged(aCompound);
        Assert.True(AllFrozen(aResult));
        var aChildren = new List<TopoDS_Shape>();
        for (var it = new TopoDS_Iterator(aResult); it.More(); it.Next())
        {
            aChildren.Add(it.Value());
        }
        Assert.Same(aBox.TShape(), aChildren[0].TShape()); // planes only: the caller's own
        int aRevolutions = 0;
        foreach (TopoDS_Shape aF in SubShapes(aResult, TopAbs_FACE))
        {
            aRevolutions += BasisOf(BRep_Tool.Surface(TopoDS.Face(aF))!) is Geom_SurfaceOfRevolution ? 1 : 0;
        }
        Assert.Equal(2, aRevolutions); // the sphere and the cylinder's side
        Assert.Equal(aVolume, VolumeOf(aResult), 3);
    }

    [Fact]
    public void SweptToElementary_ConvertsARevolutionOfASegment_ToACylinder()
    {
        // (BRepPrimAPI_MakeRevol would make a cylinder itself)
        var aSurface = new Geom_SurfaceOfRevolution(new Geom_Line(new Point3D(4, 0, 0), new Direction3D(0, 0, 1)),
                                                    new Axis3D(new Point3D(0, 0, 0), new Direction3D(0, 0, 1)));
        TopoDS_Shape aRevol = new BRepBuilderAPI_MakeFace(aSurface, 0, 2 * Math.PI, 0, 10, global::Kerf.Engine.Foundation.Precision.Confusion()).Shape();
        TopoDS_Face aFace = TopoDS.Face(SubShapes(aRevol, TopAbs_FACE)[0]);
        Assert.IsType<Geom_SurfaceOfRevolution>(BRep_Tool.Surface(aFace));
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aRevol);

        TopoDS_Shape aResult = ShapeCustom.SweptToElementary(aRevol);

        aSnap.AssertUnchanged(aRevol);
        Assert.True(AllFrozen(aResult));
        Geom_Surface aNew = BRep_Tool.Surface(TopoDS.Face(SubShapes(aResult, TopAbs_FACE)[0]))!;
        Assert.IsType<Geom_CylindricalSurface>(aNew);
        Assert.Equal(4.0, ((Geom_CylindricalSurface)aNew).Radius(), 12);
    }

    [Fact]
    public void ConvertToBSpline_WithThePlaneMode_ConvertsPlanes()
    {
        TopoDS_Shape aBox = Box();
        Assert.Same(aBox.TShape(), ShapeCustom.ConvertToBSpline(aBox, true, true, true).TShape());
        TopoDS_Shape aResult = ShapeCustom.ConvertToBSpline(aBox, true, true, true, true);
        Assert.True(AllFrozen(aResult));
        foreach (TopoDS_Shape aF in SubShapes(aResult, TopAbs_FACE))
        {
            Assert.IsType<Geom_BSplineSurface>(BasisOf(BRep_Tool.Surface(TopoDS.Face(aF))!));
        }
        Assert.Equal(1000.0, VolumeOf(aResult), 6);
    }

    [Fact]
    public void BSplineRestriction_AsToSplines_ConvertsEverySurface_TheInputIsUnchanged()
    {
        TopoDS_Shape aCylinder = new BRepPrimAPI_MakeCylinder(new Frame3D(new Point3D(1, 2, 3), new Direction3D(0, 1, 1)), 3, 7).Shape();
        var aParameters = new ShapeCustom_RestrictionParameters();
        aParameters.ConvertPlane() = true;
        aParameters.ConvertCylindricalSurf() = true;
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aCylinder);

        TopoDS_Shape aResult = ShapeCustom.BSplineRestriction(aCylinder, 1e-3, 1e-3, 3, 1, C0, C0, true, true, aParameters);

        aSnap.AssertUnchanged(aCylinder);
        Assert.True(AllFrozen(aResult));
        foreach (TopoDS_Shape aF in SubShapes(aResult, TopAbs_FACE))
        {
            Geom_Surface aS = BRep_Tool.Surface(TopoDS.Face(aF))!;
            Assert.IsType<Geom_BSplineSurface>(aS);
            Assert.True(((Geom_BSplineSurface)aS).UDegree() <= 3 && ((Geom_BSplineSurface)aS).VDegree() <= 3);
        }
        Assert.Equal(Math.PI * 9 * 7, VolumeOf(aResult), 1);
    }

    [Fact]
    public void BSplineRestriction_TheModificationsErrorsAndSpans()
    {
        TopoDS_Shape aCylinder = new BRepPrimAPI_MakeCylinder(3, 7).Shape();
        var aParameters = new ShapeCustom_RestrictionParameters();
        aParameters.ConvertCylindricalSurf() = true;
        var aBSR = new ShapeCustom_BSplineRestriction(true, true, true, 1e-3, 1e-5, C1, C2, 6, 10, true, false, aParameters);
        var aContext = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        ShapeCustom.ApplyModifier(aCylinder, aBSR, aContext, new BRepTools_Modifier());
        Assert.True(aBSR.NbOfSpan() > 0);
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
        global::Kerf.Engine.Foundation.MsgFile.LoadFromString(SHMessage_SHAPE_us.Text);
        TopoDS_Shape aBox = Mirrored(Box());
        var aDM = new ShapeCustom_DirectModification();
        var aMsg = new ShapeExtend_MsgRegistrator();
        aDM.SetMsgRegistrator(aMsg);
        Assert.Same(aMsg, aDM.MsgRegistrator());
        var aContext = new NCollection_DataMap<TopoDS_Shape, TopoDS_Shape>(TopTools_ShapeMapHasher.Instance);
        ShapeCustom.ApplyModifier(aBox, aDM, aContext, new BRepTools_Modifier());
        Assert.Equal(6, aMsg.MapShape().Extent());
        foreach (TopoDS_Shape aF in SubShapes(aBox, TopAbs_FACE))
        {
            Assert.Equal("Direction of Face of revolution corrected", aMsg.MapShape()[aF].First().Value());
        }
    }

    [Fact]
    public void RestrictionParameters_Defaults_EachAccessorItsOwnFlag()
    {
        var aP = new ShapeCustom_RestrictionParameters();
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
