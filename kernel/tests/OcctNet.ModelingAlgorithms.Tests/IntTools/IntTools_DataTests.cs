using Kerf.Geometry;
using OcctNet.ModelingData.Tests;
using static Kerf.Geometry.CurveKind;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>IntTools' by-value data classes: Range, Root, Curve, CommonPrt, PntOnFace, PntOn2Faces.</summary>
public class IntTools_DataTests
{
    [Fact]
    public void Range_AccessorsAndTwins()
    {
        IntTools_Range aDefault = new();
        Assert.Equal(0.0, aDefault.First());
        Assert.Equal(0.0, aDefault.Last());

        IntTools_Range aRange = new(1.5, 4.0);
        aRange.Range(out double aF, out double aL);
        Assert.Equal((1.5, 4.0), (aF, aL));

        IntTools_Range aMoved = aRange.WithFirst(2.0).WithLast(3.0);
        Assert.Equal(new IntTools_Range(2.0, 3.0), aMoved);
        Assert.Equal(new IntTools_Range(1.5, 4.0), aRange); // the twin copies
        Assert.NotEqual(aRange, aMoved);
        Assert.Equal(new IntTools_Range(1.5, 4.0).GetHashCode(), aRange.GetHashCode());

        aRange.SetFirst(-1.0);
        aRange.SetLast(7.0);
        Assert.Equal((-1.0, 7.0), (aRange.First(), aRange.Last()));
    }

    [Fact]
    public void Root_DefaultsAndConstructors()
    {
        IntTools_Root aDefault = new();
        Assert.Equal(0.0, aDefault.Root());
        Assert.Equal(-1, aDefault.Type());
        Assert.Equal(TopAbs_UNKNOWN, aDefault.StateBefore());
        Assert.Equal(TopAbs_UNKNOWN, aDefault.StateAfter());
        Assert.Equal(0.0, aDefault.LayerHeight());
        Assert.False(aDefault.IsValid());

        IntTools_Root aRoot = new(0.25, 2);
        Assert.Equal(0.25, aRoot.Root());
        Assert.Equal(2, aRoot.Type());
        aRoot.Interval(out double t1, out double t2, out double f1, out double f2);
        Assert.Equal((0.0, 0.0, 0.0, 0.0), (t1, t2, f1, f2));
    }

    [Fact]
    public void Root_SettersCopiesAndValidity()
    {
        IntTools_Root aRoot = new(1.0, 0);
        aRoot.SetInterval(0.5, 1.5, 2.0, -3.0);
        aRoot.SetLayerHeight(0.1);
        aRoot.Interval(out double t1, out double t2, out double f1, out double f2);
        Assert.Equal((0.5, 1.5, 2.0, -3.0), (t1, t2, f1, f2));
        Assert.Equal(0.1, aRoot.LayerHeight());

        IntTools_Root aCopy = aRoot; // a C++ copy
        aCopy.SetRoot(9.0);
        Assert.Equal(1.0, aRoot.Root());
        Assert.Equal(9.0, aCopy.Root());

        (TopAbs_State, TopAbs_State, bool)[] aCases =
        [
            (TopAbs_OUT, TopAbs_IN, true), (TopAbs_OUT, TopAbs_ON, true), (TopAbs_ON, TopAbs_OUT, true),
            (TopAbs_IN, TopAbs_OUT, true), (TopAbs_IN, TopAbs_IN, false), (TopAbs_ON, TopAbs_ON, false),
            (TopAbs_IN, TopAbs_ON, false), (TopAbs_ON, TopAbs_IN, false), (TopAbs_OUT, TopAbs_OUT, false),
        ];
        foreach ((TopAbs_State aBefore, TopAbs_State anAfter, bool isValid) in aCases)
        {
            Assert.Equal(isValid, aRoot.WithStateBefore(aBefore).WithStateAfter(anAfter).IsValid());
        }

        Assert.Equal(aRoot, aRoot.WithType(0));
        Assert.NotEqual(aRoot, aRoot.WithType(1));
        Assert.Equal(aRoot.WithInterval(0, 1, 2, 3).GetHashCode(), aRoot.WithInterval(0, 1, 2, 3).GetHashCode());
    }

    [Fact]
    public void Curve_CurvesTolerancesAndCopies()
    {
        Geom_Line aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        Geom2d_Line aL2d = new(new Point2D(0, 0), new Direction2D(1, 0));
        IntTools_Curve aCurve = new(aLine, aL2d, null, 1.0e-5, 2.0e-5);
        Assert.Same(aLine, aCurve.Curve());
        Assert.Same(aL2d, aCurve.FirstCurve2d());
        Assert.Null(aCurve.SecondCurve2d());
        Assert.Equal(1.0e-5, aCurve.Tolerance());
        Assert.Equal(2.0e-5, aCurve.TangentialTolerance());
        Assert.Equal(Line, aCurve.Type());

        IntTools_Curve aCopy = aCurve; // a C++ copy shares the curves
        aCopy.SetTolerance(1.0);
        aCopy.SetSecondCurve2d(aL2d);
        Assert.Equal(1.0e-5, aCurve.Tolerance());
        Assert.Null(aCurve.SecondCurve2d());
        Assert.Same(aCurve.Curve(), aCopy.Curve());

        IntTools_Curve anEmpty = new();
        Assert.Null(anEmpty.Curve());
        Assert.Equal(0.0, anEmpty.Tolerance());
        Assert.Equal(3.0, anEmpty.WithTangentialTolerance(3.0).TangentialTolerance());
    }

    [Fact]
    public void Curve_Bounds()
    {
        Geom_Line aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        IntTools_Curve anUnbounded = new(aLine, null, null);
        double aFirst = -5, aLast = -5;
        Point3D aP1 = new(7, 7, 7), aP2 = new(7, 7, 7);
        Assert.False(anUnbounded.HasBounds());
        Assert.False(anUnbounded.Bounds(ref aFirst, ref aLast, ref aP1, ref aP2));
        Assert.Equal(-5, aFirst); // left untouched
        Assert.Equal(new Point3D(7, 7, 7), aP1);

        IntTools_Curve aBounded = new(new Geom_TrimmedCurve(aLine, 1.0, 3.0), null, null);
        Assert.True(aBounded.HasBounds());
        Assert.True(aBounded.Bounds(ref aFirst, ref aLast, ref aP1, ref aP2));
        Assert.Equal((1.0, 3.0), (aFirst, aLast));
        Assert.True(aP1.IsEqual(new Point3D(1, 0, 0), 0.0));
        Assert.True(aP2.IsEqual(new Point3D(3, 0, 0), 0.0));
    }

    [Fact]
    public void Curve_D0_OcctBug_EvaluatesOutsideTheBounds()
    {
        // OCCT tests "t >= First || t <= Last", so every parameter is inside and evaluated
        Geom_Line aLine = new(new Point3D(0, 0, 0), new Direction3D(1, 0, 0));
        IntTools_Curve aCurve = new(new Geom_TrimmedCurve(aLine, 1.0, 3.0), null, null);
        Point3D aP = new();
        Assert.True(aCurve.D0(2.0, ref aP));
        Assert.True(aP.IsEqual(new Point3D(2, 0, 0), 0.0));
        Assert.True(aCurve.D0(10.0, ref aP)); // outside [1, 3]: should be false
        Assert.True(aP.IsEqual(new Point3D(10, 0, 0), 0.0));
        Assert.True(aCurve.D0(-4.0, ref aP));
        Assert.True(aP.IsEqual(new Point3D(-4, 0, 0), 0.0));
    }

    [Fact]
    public void CommonPrt_AccessorsAndCopy()
    {
        IntTools_CommonPrt aCP = new();
        Assert.Equal(TopAbs_SHAPE, aCP.Type());
        Assert.False(aCP.AllNullFlag());
        Assert.Equal(0.0, aCP.VertexParameter1());
        Assert.Equal(0, aCP.Ranges2().Length());

        TopoDS_Edge anE1 = new BRepBuilderAPI_MakeEdge(new Point3D(0, 0, 0), new Point3D(1, 0, 0)).Edge();
        TopoDS_Edge anE2 = new BRepBuilderAPI_MakeEdge(new Point3D(0, 1, 0), new Point3D(1, 1, 0)).Edge();
        aCP.SetEdge1(anE1);
        aCP.SetEdge2(anE2);
        aCP.SetType(TopAbs_EDGE);
        aCP.SetRange1(0.1, 0.9);
        aCP.AppendRange2(0.2, 0.8);
        aCP.AppendRange2(new IntTools_Range(0.3, 0.4));
        aCP.SetVertexParameter1(0.5);
        aCP.SetVertexParameter2(0.6);
        aCP.SetBoundingPoints(new Point3D(1, 2, 3), new Point3D(4, 5, 6));
        aCP.SetAllNullFlag(true);

        Assert.True(aCP.Edge1().IsSame(anE1));
        Assert.True(aCP.Edge2().IsSame(anE2));
        aCP.Range1(out double tf, out double tl);
        Assert.Equal((0.1, 0.9), (tf, tl));
        Assert.Equal(2, aCP.Ranges2().Length());
        aCP.BoundingPoints(out Point3D aP1, out Point3D aP2);
        Assert.Equal(new Point3D(4, 5, 6), aP2);
        Assert.Equal(new Point3D(1, 2, 3), aP1);

        // Copy: no flag, no bounding points (as OCCT)
        IntTools_CommonPrt aCopied = new();
        aCP.Copy(aCopied);
        Assert.Equal(TopAbs_EDGE, aCopied.Type());
        Assert.Equal(2, aCopied.Ranges2().Length());
        Assert.False(aCopied.AllNullFlag());
        aCopied.BoundingPoints(out aP1, out _);
        Assert.Equal(new Point3D(0, 0, 0), aP1);

        // the copy constructor copies the ranges into a sequence of its own
        IntTools_CommonPrt aConstructed = new(aCP);
        aConstructed.AppendRange2(0.0, 0.1);
        Assert.Equal(2, aCP.Ranges2().Length());
        Assert.Equal(3, aConstructed.Ranges2().Length());
        aConstructed.BoundingPoints(out aP1, out _);
        Assert.Equal(new Point3D(1, 2, 3), aP1);
    }

    [Fact]
    public void CommonPrt_OcctBug_CopyConstructorDropsTheFlag()
    {
        IntTools_CommonPrt aCP = new();
        aCP.SetAllNullFlag(true);
        IntTools_CommonPrt aCopy = new(aCP);
        Assert.False(aCopy.AllNullFlag()); // OCCT: the copy constructor initialises it to false

        IntTools_CommonPrt anAssigned = new();
        anAssigned.Assign(aCP);
        Assert.True(anAssigned.AllNullFlag()); // Assign copies it
    }

    [Fact]
    public void CommonPrt_OcctBug_AssignAppendsTheRanges()
    {
        IntTools_CommonPrt aSource = new();
        aSource.AppendRange2(1.0, 2.0);
        IntTools_CommonPrt aTarget = new();
        aTarget.AppendRange2(5.0, 6.0);
        aTarget.Assign(aSource);
        // OCCT appends the other's ranges to its own: two ranges, where an assignment should give one
        Assert.Equal(2, aTarget.Ranges2().Length());
        Assert.Equal(new IntTools_Range(5.0, 6.0), aTarget.Ranges2().Value(1));
        Assert.Equal(new IntTools_Range(1.0, 2.0), aTarget.Ranges2().Value(2));
        aTarget.Assign(aSource);
        Assert.Equal(3, aTarget.Ranges2().Length());
    }

    [Fact]
    public void PntOnFace_DefaultsInitAndTwins()
    {
        IntTools_PntOnFace aDefault = new();
        Assert.False(aDefault.Valid());
        aDefault.Parameters(out double u, out double v);
        Assert.Equal((99.0, 99.0), (u, v));
        Assert.True(aDefault.Face().IsNull());

        TopoDS_Face aFace = ((TopoDS_Shape)TestShapes.Box(1, 1, 1)).Faces().First();
        IntTools_PntOnFace aPnt = new();
        aPnt.Init(aFace, new Point3D(1, 2, 3), 0.25, 0.75);
        Assert.False(aPnt.Valid()); // Init doesn't set the flag
        aPnt.SetValid(true);
        Assert.True(aPnt.Valid());
        Assert.True(aPnt.Face().IsSame(aFace));
        Assert.Equal(new Point3D(1, 2, 3), aPnt.Pnt());
        aPnt.Parameters(out u, out v);
        Assert.Equal((0.25, 0.75), (u, v));

        IntTools_PntOnFace aTwin = aDefault.Inited(aFace, new Point3D(1, 2, 3), 0.25, 0.75).WithValid(true);
        Assert.Equal(aPnt, aTwin);
        Assert.Equal(aPnt.GetHashCode(), aTwin.GetHashCode());
        Assert.NotEqual(aPnt, aTwin.WithParameters(0.0, 0.0));
        Assert.False(aDefault.Valid());
    }

    [Fact]
    public void PntOn2Faces_Accessors()
    {
        IntTools_PntOnFace aP1 = new IntTools_PntOnFace().WithPnt(new Point3D(1, 0, 0));
        IntTools_PntOnFace aP2 = new IntTools_PntOnFace().WithPnt(new Point3D(2, 0, 0));
        IntTools_PntOn2Faces aPair = new(aP1, aP2);
        Assert.False(aPair.IsValid());
        Assert.Equal(aP1, aPair.P1());
        Assert.Equal(aP2, aPair.P2());
        aPair.SetValid(true);
        aPair.SetP1(aP2);
        Assert.True(aPair.IsValid());
        Assert.Equal(aP2, aPair.P1());
        Assert.Equal(aPair, new IntTools_PntOn2Faces(aP2, aP2).WithValid(true));
        Assert.False(new IntTools_PntOn2Faces().IsValid());
    }
}

internal static class IntToolsTestExtensions
{
    // The faces of a shape in exploration order.
    internal static IEnumerable<TopoDS_Face> Faces(this TopoDS_Shape theShape)
    {
        for (TopExp_Explorer anExp = new(theShape, TopAbs_FACE); anExp.More(); anExp.Next())
        {
            yield return TopoDS.Face(anExp.Current());
        }
    }
}
