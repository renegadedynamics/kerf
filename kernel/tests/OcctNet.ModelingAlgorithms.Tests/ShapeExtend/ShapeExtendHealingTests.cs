// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using OcctNet.ModelingData.Tests;
using static Occt.ShapeExtend_Parametrisation;
using static Occt.TopAbs_ShapeEnum;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of ShapeExtend_Explorer, ShapeExtend_CompositeSurface and the message registrators.
/// </summary>
public class ShapeExtendHealingTests
{
    private static TopoDS_Shape Box() => new BRepPrimAPI_MakeBox(10, 20, 30).Shape();

    // a 2 x 1 grid of planar patches: [0,10]x[0,5] and [10,30]x[0,5] on z = 0
    private static NCollection_HArray2<Geom_Surface> Grid()
    {
        var aPlane = new Geom_Plane(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        var aGrid = new NCollection_HArray2<Geom_Surface>(1, 2, 1, 1);
        aGrid.SetValue(1, 1, new Geom_RectangularTrimmedSurface(aPlane, 0, 10, 0, 5));
        aGrid.SetValue(2, 1, new Geom_RectangularTrimmedSurface(aPlane, 10, 30, 0, 5));
        return aGrid;
    }

    [Fact]
    public void Explorer_SortedCompound_ShapeType_Sequences()
    {
        TopoDS_Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var see = new ShapeExtend_Explorer();

        TopoDS_Shape aFaces = see.SortedCompound(aBox, TopAbs_FACE, true, false);
        Assert.Equal(TopAbs_COMPOUND, aFaces.ShapeType());
        Assert.True(aFaces.TShape()!.IsFrozen());
        Assert.Equal(TopAbs_FACE, see.ShapeType(aFaces, true));
        Assert.Equal(6, see.SeqFromCompound(aFaces, false).Length());

        // a single shell is returned itself; an edge asked as a wire becomes a wire
        TopoDS_Shape aShell = see.SortedCompound(aBox, TopAbs_SHELL, true, false);
        Assert.Equal(TopAbs_SHELL, aShell.ShapeType());
        TopoDS_Shape anEdge = new TopExp_Explorer(aBox, TopAbs_EDGE).Current();
        Assert.Equal(TopAbs_WIRE, see.SortedCompound(anEdge, TopAbs_WIRE, false, false).ShapeType());
        Assert.True(see.SortedCompound(aBox, TopAbs_FACE, false, false).IsNull()); // no exploration of a solid

        // nested compounds, mixed types
        NCollection_HSequence<TopoDS_Shape> aSeq = see.SeqFromCompound(aFaces, false);
        aSeq.Append(anEdge);
        TopoDS_Shape aMixed = see.CompoundFromSeq(aSeq);
        Assert.Equal(TopAbs_COMPOUND, see.ShapeType(aMixed, true));
        Assert.Equal(TopAbs_COMPOUND, see.ShapeType(aMixed, false));
        Assert.Equal(TopAbs_SHAPE, see.ShapeType(new TopoDS_Shape(), true));

        var aList = new NCollection_List<TopoDS_Shape>();
        see.ListFromSeq(aSeq, aList);
        Assert.Equal(7, aList.Extent());
        Assert.Equal(7, see.SeqFromList(aList).Length());

        NCollection_HSequence<TopoDS_Shape>? v = null, e = null, w = null, f = null, sh = null, so = null, cs = null, c = null;
        see.DispatchList(aSeq, ref v, ref e, ref w, ref f, ref sh, ref so, ref cs, ref c);
        Assert.Equal(6, f!.Length());
        Assert.Equal(1, e!.Length());
        Assert.Equal(0, v!.Length());
        aSnap.AssertUnchanged(aBox);
    }

    [Fact]
    public void Explorer_InternalForms_DontFreeze()
    {
        var see = new ShapeExtend_Explorer();
        TopoDS_Shape aFaces = see.InternalSortedCompound(Box(), TopAbs_FACE, true, false);
        Assert.False(aFaces.TShape()!.IsFrozen());
        Assert.False(see.InternalCompoundFromSeq(see.SeqFromCompound(aFaces, false)).TShape()!.IsFrozen());
    }

    [Fact]
    public void CompositeSurface_NaturalParametrisation()
    {
        var aSurf = new ShapeExtend_CompositeSurface(Grid());
        Assert.Equal(2, aSurf.NbUPatches());
        Assert.Equal(1, aSurf.NbVPatches());
        aSurf.Bounds(out double U1, out double U2, out double V1, out double V2);
        Assert.Equal((0.0, 30.0, 0.0, 5.0), (U1, U2, V1, V2));
        Assert.Equal(1, aSurf.LocateUParameter(5));
        Assert.Equal(2, aSurf.LocateUParameter(15));
        Assert.Equal(2, aSurf.LocateUParameter(100)); // the last outside the range
        Assert.False(aSurf.IsUClosed());
        Assert.False(aSurf.IsUPeriodic());
        Assert.Equal(GeomAbs_Shape.GeomAbs_C0, aSurf.Continuity());

        gp_Pnt aP = aSurf.Value(new gp_Pnt2d(15, 2));
        Assert.Equal(new gp_Pnt(15, 2, 0), aP);
        Assert.Equal(aP, aSurf.EvalD0(15, 2));
        gp_Pnt2d aLocal = aSurf.GlobalToLocal(2, 1, new gp_Pnt2d(15, 2));
        Assert.Equal(new gp_Pnt2d(15, 2), aLocal);
        Assert.Equal(new gp_Pnt2d(15, 2), aSurf.LocalToGlobal(2, 1, aLocal));
        Assert.False(aSurf.GlobalToLocalTransformation(2, 1, out double aFact, out _));
        Assert.Equal(1.0, aFact);
        Assert.Null(aSurf.UIso(1)); // not implemented, as OCCT
    }

    [Fact]
    public void CompositeSurface_UniformParametrisation_AndBadJoints()
    {
        var aSurf = new ShapeExtend_CompositeSurface(Grid(), ShapeExtend_Uniform);
        Assert.Equal(1.0, aSurf.UJointValue(2));
        Assert.Equal(2.0, aSurf.UJointValue(3));
        Assert.Equal(1, aSurf.LocateUParameter(0.5));
        gp_Pnt aP = aSurf.Value(new gp_Pnt2d(1.5, 0.5));
        Assert.Equal(20.0, aP.X(), 1e-12);
        Assert.Equal(2.5, aP.Y(), 1e-12);
        Assert.True(aSurf.GlobalToLocalTransformation(2, 1, out _, out _));

        // bad joints (not increasing): natural ones instead
        var aBad = new NCollection_Array1<double>(1, 3);
        aBad[1] = 0;
        aBad[2] = 0;
        aBad[3] = 1;
        var aSurf2 = new ShapeExtend_CompositeSurface(Grid(), aBad, new NCollection_Array1<double>(1, 2));
        Assert.Equal(10.0, aSurf2.UJointValue(2));

        // the twins
        ShapeExtend_CompositeSurface aShifted = aSurf.WithUFirstValue(5);
        Assert.Equal(6.0, aShifted.UJointValue(2));
        Assert.Equal(1.0, aSurf.UJointValue(2));
        var aJoints = new NCollection_Array1<double>(1, 3);
        aJoints[1] = 0;
        aJoints[2] = 3;
        aJoints[3] = 4;
        ShapeExtend_CompositeSurface aSet = aSurf.WithUJointValues(aJoints, out bool isDone);
        Assert.True(isDone);
        Assert.Equal(3.0, aSet.UJointValue(2));
        aSurf.WithUJointValues(aBad, out isDone);
        Assert.False(isDone);
        Assert.Equal(10.0, aSurf.WithJointValues(ShapeExtend_Natural).UJointValue(2));
    }

    [Fact]
    public void CompositeSurface_Copy_LosesJointValues()
    {
        // OCCT: Copy() re-initializes the copy with the natural parametrisation (occt-issues.md, 6H1)
        var aSurf = new ShapeExtend_CompositeSurface(Grid(), ShapeExtend_Unitary);
        Assert.Equal(0.5, aSurf.UJointValue(2));
        var aCopy = (ShapeExtend_CompositeSurface)aSurf.Copy();
        Assert.Equal(10.0, aCopy.UJointValue(2));
    }

    [Fact]
    public void CompositeSurface_FrozenPatches_AndTransform()
    {
        NCollection_HArray2<Geom_Surface> aGrid = Grid();
        var aSurf = new ShapeExtend_CompositeSurface(aGrid);
        Geom_Surface aPatch = aSurf.Patch(1, 1);
        Assert.True(aPatch.IsFrozen());
        Assert.Same(aPatch, aGrid.Value(1, 1)); // shared, not copied

        // the caller's array is not the surface's
        aGrid.SetValue(1, 1, new Geom_Plane(new gp_Pnt(0, 0, 5), new gp_Dir(0, 0, 1)));
        Assert.Same(aPatch, aSurf.Patch(1, 1));

        var aT = new gp_Trsf();
        aT.SetTranslation(new gp_Vec(0, 0, 1));
        var aMoved = (ShapeExtend_CompositeSurface)aSurf.Transformed(aT);
        Assert.Equal(1.0, aMoved.Value(new gp_Pnt2d(5, 1)).Z());
        Assert.Equal(0.0, aSurf.Value(new gp_Pnt2d(5, 1)).Z());

        aSurf.Freeze();
        Assert.ThrowsAny<Standard_ImmutableObject>(() => aSurf.SetUFirstValue(1));
        Assert.ThrowsAny<Standard_ImmutableObject>(() => aSurf.Init(Grid()));
    }

    [Fact]
    public void CompositeSurface_ClosedInU()
    {
        // two half-cylinders: the grid closes in U
        var aCyl = new Geom_CylindricalSurface(new gp_Ax3(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1), new gp_Dir(1, 0, 0)), 2);
        var aGrid = new NCollection_HArray2<Geom_Surface>(1, 2, 1, 1);
        aGrid.SetValue(1, 1, new Geom_RectangularTrimmedSurface(aCyl, 0, M_PI, 0, 1));
        aGrid.SetValue(2, 1, new Geom_RectangularTrimmedSurface(aCyl, M_PI, 2 * M_PI, 0, 1));
        var aSurf = new ShapeExtend_CompositeSurface();
        Assert.True(aSurf.Init(aGrid));
        Assert.True(aSurf.IsUClosed());
        Assert.False(aSurf.IsVClosed());
    }

    [Fact]
    public void MsgRegistrator_CollectsMessagesPerObject()
    {
        TopoDS_Shape aBox = Box();
        TopoDS_Shape aFace = new TopExp_Explorer(aBox, TopAbs_FACE).Current();
        var aReg = new ShapeExtend_MsgRegistrator();
        aReg.Send(aFace, new Message_Msg("first"), Message_Gravity.Message_Warning);
        aReg.Send(aFace, new Message_Msg("second"), Message_Gravity.Message_Info);
        aReg.Send(new TopoDS_Shape(), new Message_Msg("ignored"), Message_Gravity.Message_Info);
        Assert.Equal(1, aReg.MapShape().Extent());
        Assert.Equal(2, aReg.MapShape().Find(aFace).Extent());

        var aSurf = new Geom_Plane(new gp_Pnt(0, 0, 0), new gp_Dir(0, 0, 1));
        aReg.Send(aSurf, new Message_Msg("geom"), Message_Gravity.Message_Fail);
        aReg.Send(new Message_Msg("no object"), Message_Gravity.Message_Fail); // a null transient: ignored
        Assert.Equal(1, aReg.MapTransient().Extent());

        // the base class does nothing
        var aBasic = new ShapeExtend_BasicMsgRegistrator();
        aBasic.Send(aFace, new Message_Msg("x"), Message_Gravity.Message_Info);
        aBasic.Send(new Message_Msg("x"), Message_Gravity.Message_Info);
    }
}
