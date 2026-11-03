// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Api;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Extend;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using OcctNet.ModelingData.Tests;
using static Kerf.Engine.Healing.Extend.Parametrisation;
using static Kerf.Topology.ShapeType;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>
/// Known answers of ShapeExtend_Explorer, ShapeExtend_CompositeSurface and the message registrators.
/// </summary>
public class ShapeExtendHealingTests
{
    private static Shape Box() => new MakeBox(10, 20, 30).Shape();

    // a 2 x 1 grid of planar patches: [0,10]x[0,5] and [10,30]x[0,5] on z = 0
    private static NCollection_HArray2<Surface> Grid()
    {
        var aPlane = new PlaneSurface(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        var aGrid = new NCollection_HArray2<Surface>(1, 2, 1, 1);
        aGrid.SetValue(1, 1, new TrimmedSurface(aPlane, 0, 10, 0, 5));
        aGrid.SetValue(2, 1, new TrimmedSurface(aPlane, 10, 30, 0, 5));
        return aGrid;
    }

    [Fact]
    public void Explorer_SortedCompound_ShapeType_Sequences()
    {
        Shape aBox = Box();
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var see = new ExtendExplorer();

        Shape aFaces = see.SortedCompound(aBox, ShapeType.Face, true, false);
        Assert.Equal(ShapeType.Compound, aFaces.ShapeType());
        Assert.True(aFaces.TShape()!.IsFrozen);
        Assert.Equal(ShapeType.Face, see.ShapeType(aFaces, true));
        Assert.Equal(6, see.SeqFromCompound(aFaces, false).Length());

        // a single shell is returned itself; an edge asked as a wire becomes a wire
        Shape aShell = see.SortedCompound(aBox, ShapeType.Shell, true, false);
        Assert.Equal(ShapeType.Shell, aShell.ShapeType());
        Shape anEdge = new Explorer(aBox, ShapeType.Edge).Current();
        Assert.Equal(ShapeType.Wire, see.SortedCompound(anEdge, ShapeType.Wire, false, false).ShapeType());
        Assert.True(see.SortedCompound(aBox, ShapeType.Face, false, false).IsNull); // no exploration of a solid

        // nested compounds, mixed types
        NCollection_HSequence<Shape> aSeq = see.SeqFromCompound(aFaces, false);
        aSeq.Append(anEdge);
        Shape aMixed = see.CompoundFromSeq(aSeq);
        Assert.Equal(ShapeType.Compound, see.ShapeType(aMixed, true));
        Assert.Equal(ShapeType.Compound, see.ShapeType(aMixed, false));
        Assert.Equal(ShapeType.Shape, see.ShapeType(new Shape(), true));

        var aList = new NCollection_List<Shape>();
        see.ListFromSeq(aSeq, aList);
        Assert.Equal(7, aList.Extent());
        Assert.Equal(7, see.SeqFromList(aList).Length());

        NCollection_HSequence<Shape>? v = null, e = null, w = null, f = null, sh = null, so = null, cs = null, c = null;
        see.DispatchList(aSeq, ref v, ref e, ref w, ref f, ref sh, ref so, ref cs, ref c);
        Assert.Equal(6, f!.Length());
        Assert.Equal(1, e!.Length());
        Assert.Equal(0, v!.Length());
        aSnap.AssertUnchanged(aBox);
    }

    [Fact]
    public void Explorer_InternalForms_DontFreeze()
    {
        var see = new ExtendExplorer();
        Shape aFaces = see.InternalSortedCompound(Box(), ShapeType.Face, true, false);
        Assert.False(aFaces.TShape()!.IsFrozen);
        Assert.False(see.InternalCompoundFromSeq(see.SeqFromCompound(aFaces, false)).TShape()!.IsFrozen);
    }

    [Fact]
    public void CompositeSurface_NaturalParametrisation()
    {
        var aSurf = new CompositeSurface(Grid());
        Assert.Equal(2, aSurf.UPatchCount);
        Assert.Equal(1, aSurf.VPatchCount);
        aSurf.Bounds(out double U1, out double U2, out double V1, out double V2);
        Assert.Equal((0.0, 30.0, 0.0, 5.0), (U1, U2, V1, V2));
        Assert.Equal(1, aSurf.LocateUParameter(5));
        Assert.Equal(2, aSurf.LocateUParameter(15));
        Assert.Equal(2, aSurf.LocateUParameter(100)); // the last outside the range
        Assert.False(aSurf.IsUClosed);
        Assert.False(aSurf.IsUPeriodic);
        Assert.Equal(Continuity.C0, aSurf.Continuity());

        Point3D aP = aSurf.Value(new Point2D(15, 2));
        Assert.Equal(new Point3D(15, 2, 0), aP);
        Assert.Equal(aP, aSurf.EvalD0(15, 2));
        Point2D aLocal = aSurf.GlobalToLocal(2, 1, new Point2D(15, 2));
        Assert.Equal(new Point2D(15, 2), aLocal);
        Assert.Equal(new Point2D(15, 2), aSurf.LocalToGlobal(2, 1, aLocal));
        Assert.False(aSurf.GlobalToLocalTransformation(2, 1, out double aFact, out _));
        Assert.Equal(1.0, aFact);
        Assert.Null(aSurf.UIso(1)); // not implemented, as OCCT
    }

    [Fact]
    public void CompositeSurface_UniformParametrisation_AndBadJoints()
    {
        var aSurf = new CompositeSurface(Grid(), Parametrisation.Uniform);
        Assert.Equal(1.0, aSurf.UJointValue(2));
        Assert.Equal(2.0, aSurf.UJointValue(3));
        Assert.Equal(1, aSurf.LocateUParameter(0.5));
        Point3D aP = aSurf.Value(new Point2D(1.5, 0.5));
        Assert.Equal(20.0, aP.X(), 1e-12);
        Assert.Equal(2.5, aP.Y(), 1e-12);
        Assert.True(aSurf.GlobalToLocalTransformation(2, 1, out _, out _));

        // bad joints (not increasing): natural ones instead
        var aBad = new NCollection_Array1<double>(1, 3);
        aBad[1] = 0;
        aBad[2] = 0;
        aBad[3] = 1;
        var aSurf2 = new CompositeSurface(Grid(), aBad, new NCollection_Array1<double>(1, 2));
        Assert.Equal(10.0, aSurf2.UJointValue(2));

        // the twins
        CompositeSurface aShifted = aSurf.WithUFirstValue(5);
        Assert.Equal(6.0, aShifted.UJointValue(2));
        Assert.Equal(1.0, aSurf.UJointValue(2));
        var aJoints = new NCollection_Array1<double>(1, 3);
        aJoints[1] = 0;
        aJoints[2] = 3;
        aJoints[3] = 4;
        CompositeSurface aSet = aSurf.WithUJointValues(aJoints, out bool isDone);
        Assert.True(isDone);
        Assert.Equal(3.0, aSet.UJointValue(2));
        aSurf.WithUJointValues(aBad, out isDone);
        Assert.False(isDone);
        Assert.Equal(10.0, aSurf.WithJointValues(Parametrisation.Natural).UJointValue(2));
    }

    [Fact]
    public void CompositeSurface_Copy_LosesJointValues()
    {
        // OCCT: Copy() re-initializes the copy with the natural parametrisation (occt-issues.md, 6H1)
        var aSurf = new CompositeSurface(Grid(), Parametrisation.Unitary);
        Assert.Equal(0.5, aSurf.UJointValue(2));
        var aCopy = (CompositeSurface)aSurf.Copy();
        Assert.Equal(10.0, aCopy.UJointValue(2));
    }

    [Fact]
    public void CompositeSurface_FrozenPatches_AndTransform()
    {
        NCollection_HArray2<Surface> aGrid = Grid();
        var aSurf = new CompositeSurface(aGrid);
        Surface aPatch = aSurf.Patch(1, 1);
        Assert.True(aPatch.IsFrozen);
        Assert.Same(aPatch, aGrid.Value(1, 1)); // shared, not copied

        // the caller's array is not the surface's
        aGrid.SetValue(1, 1, new PlaneSurface(new Point3D(0, 0, 5), new Direction3D(0, 0, 1)));
        Assert.Same(aPatch, aSurf.Patch(1, 1));

        var aT = new Transform3D();
        aT.SetTranslation(new Vector3D(0, 0, 1));
        var aMoved = (CompositeSurface)aSurf.Transformed(aT);
        Assert.Equal(1.0, aMoved.Value(new Point2D(5, 1)).Z());
        Assert.Equal(0.0, aSurf.Value(new Point2D(5, 1)).Z());

        aSurf.Freeze();
        Assert.ThrowsAny<ImmutableObjectException>(() => aSurf.SetUFirstValue(1));
        Assert.ThrowsAny<ImmutableObjectException>(() => aSurf.Init(Grid()));
    }

    [Fact]
    public void CompositeSurface_ClosedInU()
    {
        // two half-cylinders: the grid closes in U
        var aCyl = new CylindricalSurface(new CoordinateSystem(new Point3D(0, 0, 0), new Direction3D(0, 0, 1), new Direction3D(1, 0, 0)), 2);
        var aGrid = new NCollection_HArray2<Surface>(1, 2, 1, 1);
        aGrid.SetValue(1, 1, new TrimmedSurface(aCyl, 0, M_PI, 0, 1));
        aGrid.SetValue(2, 1, new TrimmedSurface(aCyl, M_PI, 2 * M_PI, 0, 1));
        var aSurf = new CompositeSurface();
        Assert.True(aSurf.Init(aGrid));
        Assert.True(aSurf.IsUClosed);
        Assert.False(aSurf.IsVClosed);
    }

    [Fact]
    public void MsgRegistrator_CollectsMessagesPerObject()
    {
        Shape aBox = Box();
        Shape aFace = new Explorer(aBox, ShapeType.Face).Current();
        var aReg = new MsgRegistrator();
        aReg.Send(aFace, new Msg("first"), Gravity.Warning);
        aReg.Send(aFace, new Msg("second"), Gravity.Info);
        aReg.Send(new Shape(), new Msg("ignored"), Gravity.Info);
        Assert.Equal(1, aReg.MapShape().Extent());
        Assert.Equal(2, aReg.MapShape().Find(aFace).Extent());

        var aSurf = new PlaneSurface(new Point3D(0, 0, 0), new Direction3D(0, 0, 1));
        aReg.Send(aSurf, new Msg("geom"), Gravity.Fail);
        aReg.Send(new Msg("no object"), Gravity.Fail); // a null transient: ignored
        Assert.Equal(1, aReg.MapTransient().Extent());

        // the base class does nothing
        var aBasic = new BasicMsgRegistrator();
        aBasic.Send(aFace, new Msg("x"), Gravity.Info);
        aBasic.Send(new Msg("x"), Gravity.Info);
    }
}
