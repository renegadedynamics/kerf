// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Api;
using Kerf.Engine.Features.Local;
using Kerf.Engine.Foundation;
using Kerf.Engine.Healing.Checks;
using Kerf.Engine.Measurement;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Kernel.ModelingData.Tests;
using Kerf.Topology;
using static Kerf.Topology.Orientation;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>
/// LocOpe's parts by what they must satisfy (decision 19805): the intersectors' points along a line, in order, in and out
/// of the material; the sweeps by area times path; the splits by the area they keep.
/// </summary>
public class LocOpeLawsTests
{
    private const double BoxVolume = 6000;

    private static Shape Box() => LocOpeTests.Box();

    private static double Volume(in Shape theShape) => LocOpeTests.Volume(theShape);

    private static double Area(in Shape theShape) => LocOpeTests.Area(theShape);

    private static int Count(in Shape theShape, ShapeType theType) => LocOpeTests.Count(theShape, theType);

    private static Edge Segment(double x1, double y1, double z1, double x2, double y2, double z2) =>
        LocOpeTests.Segment(new Point3D(x1, y1, z1), new Point3D(x2, y2, z2));

    private static Wire Polyline(params Point3D[] thePoints)
    {
        var aMake = new MakePolygon();
        foreach (Point3D aP in thePoints)
        {
            aMake.Add(aP);
        }
        return aMake.Wire();
    }

    private static Face FaceAt(in Shape theShape, Func<Point3D, bool> theWhere)
    {
        for (var anExp = new Explorer(theShape, ShapeType.Face); anExp.More(); anExp.Next())
        {
            var aProps = new Properties();
            MassPropertiesBuilder.SurfaceProperties(anExp.Current(), aProps);
            if (theWhere(aProps.CentreOfMass()))
            {
                return ShapeCasts.Face(anExp.Current());
            }
        }
        throw new InvalidOperationException("no such face");
    }

    // A C: the box with a slot from x = 0 to 8 and z = 10 to 20, so a line down at x = 4 crosses 30, 20, 10 and 0
    private static Shape CShape()
    {
        Shape aSlot = new MakeBox(new Point3D(-1, -1, 10), new Point3D(8, 21, 20)).Shape();
        return new Kerf.Engine.Booleans.Api.Cut(Box(), aSlot).Shape();
    }

    private static readonly Line3D Plumb = new(new Point3D(4, 10, 31), new Direction3D(0, 0, -1));

    private static CSIntersector PlumbIntersector()
    {
        var aLines = new NCollection_Sequence<Line3D>();
        aLines.Append(Plumb);
        var anInt = new CSIntersector(CShape());
        anInt.Perform(aLines);
        return anInt;
    }

    [Fact]
    public void CSIntersector_lists_the_crossings_in_order_along_the_line()
    {
        CSIntersector anInt = PlumbIntersector();
        Assert.True(anInt.IsDone);
        Assert.Equal(4, anInt.NbPoints(1));
        double[] anExpected = [1, 11, 21, 31];
        Orientation[] anOrients = [Forward, Reversed, Forward, Reversed];
        for (int i = 1; i <= 4; i++)
        {
            PntFace aP = anInt.Point(1, i);
            Assert.Equal(anExpected[i - 1], aP.Parameter(), 9);
            Assert.Equal(anOrients[i - 1], aP.Orientation());
            Assert.Equal(31 - anExpected[i - 1], aP.Pnt().Z, 9);
        }
    }

    public static TheoryData<double, bool, int, int, Orientation> FromParameter() => new()
    {
        // from, whether a point lies after it, then its interval and how the line crosses it
        { 0, true, 1, 1, Forward },
        { 1, true, 1, 1, Forward }, // a crossing at the parameter counts
        { 1.5, true, 2, 2, Reversed },
        { 5, true, 2, 2, Reversed },
        { 15, true, 3, 3, Forward },
        { 25, true, 4, 4, Reversed },
        { 31, true, 4, 4, Reversed },
        { 31.5, false, 0, 0, Forward },
        { 50, false, 0, 0, Forward },
    };

    [Theory]
    [MemberData(nameof(FromParameter))]
    public void CSIntersector_localises_after_a_parameter(double theFrom, bool theFound, int theIndFrom, int theIndTo, Orientation theOr)
    {
        CSIntersector anInt = PlumbIntersector();
        Orientation anOr = Internal;
        int aFrom = 0, aTo = 0;
        Assert.Equal(theFound, anInt.LocalizeAfter(1, theFrom, 1e-7, ref anOr, ref aFrom, ref aTo));
        if (theFound)
        {
            Assert.Equal(theIndFrom, aFrom);
            Assert.Equal(theIndTo, aTo);
            Assert.Equal(theOr, anOr);
        }
    }

    [Theory]
    [InlineData(50, true, 4, Reversed)]
    [InlineData(31, true, 4, Reversed)] // a crossing at the parameter counts
    [InlineData(30.5, true, 3, Forward)]
    [InlineData(25, true, 3, Forward)]
    [InlineData(15, true, 2, Reversed)]
    [InlineData(5, true, 1, Forward)]
    [InlineData(1, true, 1, Forward)]
    [InlineData(0.5, false, 0, Forward)]
    public void CSIntersector_localises_before_a_parameter(double theFrom, bool theFound, int theInd, Orientation theOr)
    {
        CSIntersector anInt = PlumbIntersector();
        Orientation anOr = Internal;
        int aFrom = 0, aTo = 0;
        Assert.Equal(theFound, anInt.LocalizeBefore(1, theFrom, 1e-7, ref anOr, ref aFrom, ref aTo));
        if (theFound)
        {
            Assert.Equal(theInd, aFrom);
            Assert.Equal(theInd, aTo);
            Assert.Equal(theOr, anOr);
        }
    }

    [Theory]
    [InlineData(1, true, 2, Reversed)]
    [InlineData(2, true, 3, Forward)]
    [InlineData(3, true, 4, Reversed)]
    [InlineData(4, false, 0, Forward)]
    public void CSIntersector_localises_after_an_index(int theFromInd, bool theFound, int theInd, Orientation theOr)
    {
        CSIntersector anInt = PlumbIntersector();
        Orientation anOr = Internal;
        int aFrom = 0, aTo = 0;
        Assert.Equal(theFound, anInt.LocalizeAfter(1, theFromInd, 1e-7, ref anOr, ref aFrom, ref aTo));
        if (theFound)
        {
            Assert.Equal(theInd, aFrom);
            Assert.Equal(theInd, aTo);
            Assert.Equal(theOr, anOr);
        }
    }

    [Theory]
    [InlineData(1, true, 1)]
    [InlineData(2, true, 1)]
    [InlineData(11, true, 2)]
    [InlineData(0, false, 0)]
    public void CSIntersector_localising_before_an_index_reads_it_as_a_parameter_as_OCCT_does(int theFromInd, bool theFound, int theInd)
    {
        // OCCT has no LocBefore by index: the int converts to the double (occt-issues.md)
        CSIntersector anInt = PlumbIntersector();
        Orientation anOr = Internal;
        int aFrom = 0, aTo = 0;
        Assert.Equal(theFound, anInt.LocalizeBefore(1, theFromInd, 1e-7, ref anOr, ref aFrom, ref aTo));
        if (theFound)
        {
            Assert.Equal(theInd, aFrom);
            Assert.Equal(theInd, aTo);
        }
    }

    [Fact]
    public void CSIntersector_after_an_index_before_the_first_starts_from_the_first()
    {
        CSIntersector anInt = PlumbIntersector();
        Orientation anOr = Internal;
        int aFrom = 0, aTo = 0;
        Assert.True(anInt.LocalizeAfter(1, 0, 1e-7, ref anOr, ref aFrom, ref aTo));
        Assert.Equal(1, aFrom);
        Assert.Equal(Forward, anOr);
    }

    [Fact]
    public void CSIntersector_gathers_crossings_within_the_tolerance_and_passes_over_those_that_disagree()
    {
        // within 11, in and out are one contact, neither in nor out, so no clean crossing is left
        CSIntersector anInt = PlumbIntersector();
        Orientation anOr = Internal;
        int aFrom = 0, aTo = 0;
        Assert.False(anInt.LocalizeAfter(1, 0.0, 11.0, ref anOr, ref aFrom, ref aTo));
        Assert.False(anInt.LocalizeBefore(1, 50.0, 11.0, ref anOr, ref aFrom, ref aTo));
        Assert.False(anInt.LocalizeAfter(1, 0, 11.0, ref anOr, ref aFrom, ref aTo));
        // a tolerance short of the gaps between crossings leaves them clean
        Assert.True(anInt.LocalizeAfter(1, 5.0, 3.0, ref anOr, ref aFrom, ref aTo));
        Assert.Equal(2, aFrom);
        Assert.Equal(2, aTo);
    }

    [Fact]
    public void CSIntersector_needs_a_shape_and_a_valid_index_and_can_be_emptied()
    {
        var anEmpty = new CSIntersector();
        Assert.False(anEmpty.IsDone);
        Assert.Throws<NotDoneException>(() => anEmpty.NbPoints(1));
        Assert.Throws<NotDoneException>(() => anEmpty.Point(1, 1));
        var aLines = new NCollection_Sequence<Line3D>();
        Assert.Throws<ConstructionErrorException>(() => anEmpty.Perform(aLines));
        var aCircles = new NCollection_Sequence<Circle3D>();
        Assert.Throws<ConstructionErrorException>(() => anEmpty.Perform(aCircles));
        var aCurves = new NCollection_Sequence<Curve3D?>();
        Assert.Throws<ConstructionErrorException>(() => anEmpty.Perform(aCurves));

        CSIntersector anInt = PlumbIntersector();
        Orientation anOr = Internal;
        int aFrom = 0, aTo = 0;
        Assert.Throws<OutOfRangeException>(() => anInt.NbPoints(2));
        Assert.Throws<OutOfRangeException>(() => anInt.Point(2, 1));
        Assert.Throws<OutOfRangeException>(() => anInt.LocalizeAfter(0, 0.0, 1e-7, ref anOr, ref aFrom, ref aTo));
        Assert.Throws<OutOfRangeException>(() => anInt.LocalizeBefore(2, 0.0, 1e-7, ref anOr, ref aFrom, ref aTo));
        Assert.Throws<OutOfRangeException>(() => anInt.LocalizeAfter(2, 1, 1e-7, ref anOr, ref aFrom, ref aTo));
        Assert.Throws<OutOfRangeException>(() => anInt.LocalizeBefore(2, 1, 1e-7, ref anOr, ref aFrom, ref aTo));
        anInt.Destroy();
    }

    [Fact]
    public void CSIntersector_on_curves_skips_those_it_has_no_curve_for()
    {
        var aCurves = new NCollection_Sequence<Curve3D?>();
        aCurves.Append(new TrimmedCurve3D(new LineCurve3D(new Line3D(new Point3D(4, 10, 31), new Direction3D(0, 0, -1))), 0, 40));
        aCurves.Append((Curve3D?)null);
        var anInt = new CSIntersector(CShape());
        anInt.Perform(aCurves);
        Assert.Equal(4, anInt.NbPoints(1));
        Assert.Equal(0, anInt.NbPoints(2));
    }

    [Fact]
    public void CurveShapeIntersector_along_a_circle_reads_angles()
    {
        // a circle at z = 15 round the centre of the box's plan, radius 8: it leaves through x = 0 and x = 10, 2 away from
        // its centre on y = 10
        Shape aBox = Box();
        var aCircle = new Circle3D(new Frame3D(new Point3D(5, 10, 15), new Direction3D(0, 0, 1)), 8);
        var anInt = new LocalCurveShapeIntersector(aCircle, aBox);
        Assert.True(anInt.IsDone);
        Assert.Equal(4, anInt.PointCount);
        double anAngle = Math.Acos(5.0 / 8);
        double[] anExpected = [anAngle, Math.PI - anAngle, Math.PI + anAngle, 2 * Math.PI - anAngle];
        for (int i = 1; i <= 4; i++)
        {
            Assert.Equal(anExpected[i - 1], anInt.Point(i).Parameter(), 7);
        }
        Orientation anOr = Internal;
        int aFrom = 0, aTo = 0;
        Assert.True(anInt.LocalizeAfter(0.0, ref anOr, ref aFrom, ref aTo));
        Assert.Equal(1, aFrom);
        Assert.True(anInt.LocalizeBefore(6.2, ref anOr, ref aFrom, ref aTo));
        Assert.Equal(4, aTo);
        Assert.True(anInt.LocalizeAfter(1, ref anOr, ref aFrom, ref aTo));
        Assert.Equal(2, aFrom);
        Assert.True(anInt.LocalizeBefore(4, ref anOr, ref aFrom, ref aTo));
        Assert.Equal(3, aTo);
        // the first two crossings are entering then leaving: opposite
        Assert.NotEqual(anInt.Point(1).Orientation(), anInt.Point(2).Orientation());
    }

    [Fact]
    public void CurveShapeIntersector_can_be_started_empty_and_refuses_a_null_shape()
    {
        var anInt = new LocalCurveShapeIntersector();
        Assert.False(anInt.IsDone);
        var anAxis = new Axis3D(new Point3D(4, 10, 31), new Direction3D(0, 0, -1));
        anInt.Init(anAxis, new Shape());
        Assert.False(anInt.IsDone);
        anInt.Init(new Circle3D(new Frame3D(new Point3D(), new Direction3D(0, 0, 1)), 1), new Shape());
        Assert.False(anInt.IsDone);
        Assert.Throws<NotDoneException>(() => anInt.PointCount);
        anInt.Init(anAxis, CShape());
        Assert.Equal(4, anInt.PointCount);
        Assert.Throws<OutOfRangeException>(() => anInt.Point(5));
        Orientation anOr = Internal;
        int aFrom = 0, aTo = 0;
        Assert.False(anInt.LocalizeAfter(40.0, ref anOr, ref aFrom, ref aTo));
        Assert.False(anInt.LocalizeBefore(0.5, ref anOr, ref aFrom, ref aTo));
        Assert.False(anInt.LocalizeAfter(4, ref anOr, ref aFrom, ref aTo));
        Assert.False(anInt.LocalizeBefore(1, ref anOr, ref aFrom, ref aTo));
    }

    // A planar face whose outline is one closed curve, and a second made from a rebuilt copy of the curve

    private static Face FaceOn(Curve3D theCurve, double theF, double theL)
    {
        Edge anEdge = new MakeEdge(theCurve, theF, theL).Edge();
        return new MakeFace(new MakeWire(anEdge).Wire(), true).Face();
    }

    private static Face FaceWithClosing(Curve3D theCurve, double theF, double theL)
    {
        Edge anEdge = new MakeEdge(theCurve, theF, theL).Edge();
        Edge aBack = new MakeEdge(theCurve.Value(theL), theCurve.Value(theF)).Edge();
        return new MakeFace(new MakeWire(anEdge, aBack).Wire(), true).Face();
    }

    private static int Pairs(in Shape theFrom, in Shape theTo)
    {
        var aFind = new FindEdges(theFrom, theTo);
        int aCount = 0;
        for (aFind.InitIterator(); aFind.More(); aFind.Next())
        {
            Assert.False(aFind.EdgeFrom().IsNull);
            Assert.False(aFind.EdgeTo().IsNull);
            aCount++;
        }
        return aCount;
    }

    [Fact]
    public void FindEdges_finds_a_shared_circle_and_not_another()
    {
        var aFrame = new Frame3D(new Point3D(1, 2, 3), new Direction3D(0, 0, 1));
        Face aFace = FaceOn(new CircleCurve3D(new Circle3D(aFrame, 4)), 0, 2 * Math.PI);
        Face aSame = FaceOn(new CircleCurve3D(new Circle3D(aFrame, 4)), 0, 2 * Math.PI);
        Assert.Equal(1, Pairs(aFace, aSame));
        Assert.Equal(0, Pairs(aFace, FaceOn(new CircleCurve3D(new Circle3D(aFrame, 5)), 0, 2 * Math.PI)));
        var anOther = new Frame3D(new Point3D(1, 2, 4), new Direction3D(0, 0, 1));
        Assert.Equal(0, Pairs(aFace, FaceOn(new CircleCurve3D(new Circle3D(anOther, 4)), 0, 2 * Math.PI)));
        // arcs: this one lies within the whole circle; the whole circle doesn't lie within it
        Face anArc = FaceWithClosing(new CircleCurve3D(new Circle3D(aFrame, 4)), 1, 3);
        Assert.Equal(1, Pairs(anArc, aFace));
        Assert.Equal(0, Pairs(aFace, anArc));
    }

    [Fact]
    public void FindEdges_finds_a_shared_ellipse_and_not_another()
    {
        var aFrame = new Frame3D(new Point3D(1, 2, 3), new Direction3D(0, 0, 1));
        Face aFace = FaceOn(new EllipseCurve3D(new Ellipse3D(aFrame, 5, 3)), 0, 2 * Math.PI);
        Face aSame = FaceOn(new EllipseCurve3D(new Ellipse3D(aFrame, 5, 3)), 0, 2 * Math.PI);
        Assert.Equal(1, Pairs(aFace, aSame));
        Assert.Equal(0, Pairs(aFace, FaceOn(new EllipseCurve3D(new Ellipse3D(aFrame, 5, 2)), 0, 2 * Math.PI)));
    }

    private static NCollection_Array1<Point3D> Poles(double theLift = 0)
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 4);
        aPoles[1] = new Point3D(0, 0, 0);
        aPoles[2] = new Point3D(1, 3, theLift);
        aPoles[3] = new Point3D(3, 3, theLift);
        aPoles[4] = new Point3D(4, 0, 0);
        return aPoles;
    }

    [Fact]
    public void FindEdges_finds_a_shared_Bezier_and_not_another()
    {
        Face aFace = FaceWithClosing(new BezierCurve3D(Poles()), 0, 1);
        Face aSame = FaceWithClosing(new BezierCurve3D(Poles()), 0, 1);
        Assert.Equal(2, Pairs(aFace, aSame)); // the curve and the closing line
        var aDifferent = Poles();
        aDifferent[2] = new Point3D(1, 5, 0);
        Assert.Equal(1, Pairs(aFace, FaceWithClosing(new BezierCurve3D(aDifferent), 0, 1))); // the closing line alone
    }

    [Fact]
    public void FindEdges_finds_a_shared_BSpline_and_not_another()
    {
        var aKnots = new NCollection_Array1<double>(1, 2);
        aKnots[1] = 0;
        aKnots[2] = 1;
        var aMults = new NCollection_Array1<int>(1, 2);
        aMults[1] = 4;
        aMults[2] = 4;
        BSplineCurve3D Spline(NCollection_Array1<Point3D> thePoles) => new(thePoles, aKnots, aMults, 3);
        Face aFace = FaceWithClosing(Spline(Poles()), 0, 1);
        Face aSame = FaceWithClosing(Spline(Poles()), 0, 1);
        Assert.Equal(2, Pairs(aFace, aSame));
        var aDifferent = Poles();
        aDifferent[2] = new Point3D(1, 5, 0);
        Assert.Equal(1, Pairs(aFace, FaceWithClosing(Spline(aDifferent), 0, 1)));
    }

    [Fact]
    public void FindEdges_on_a_located_face_compares_where_the_edges_are()
    {
        Shape aBox = Box();
        Face aTop = LocOpeTests.TopFace(aBox);
        Transform3D aMove = new Transform3D().WithTranslation(new Vector3D(0, 0, 5));
        Shape aMoved = aTop.Moved(new Placement(aMove));
        Assert.Equal(0, Pairs(aTop, aMoved));
        Assert.Equal(4, Pairs(aMoved, aMoved));
    }

    // Sweeps of a shell of two faces of the box, and of a face, in every form: the sweep of a face is a solid whose
    // volume is its area along the sweep

    [Fact]
    public void A_prism_with_a_transfer_starts_from_the_moved_face()
    {
        Face aFace = BRepFeatTests.Rect(0, 4, 0, 6, 0);
        var aPrism = new LocalPrism(aFace, new Vector3D(0, 0, 5), new Vector3D(0, 0, -2));
        Assert.Equal(120, Math.Abs(Volume(aPrism.Shape())), 7);
        var aProps = new Properties();
        MassPropertiesBuilder.SurfaceProperties(aPrism.FirstShape(), aProps);
        Assert.Equal(-2, aProps.CentreOfMass().Z, 9);
        MassPropertiesBuilder.SurfaceProperties(aPrism.LastShape(), aProps = new Properties());
        Assert.Equal(3, aProps.CentreOfMass().Z, 9);
        Curve3D aBaryc = aPrism.BarycCurve();
        Assert.Equal(2, aBaryc.Value(0).X, 9);
        Assert.Equal(3, aBaryc.Value(0).Y, 9);
        var aCurves = new NCollection_Sequence<Curve3D>();
        aPrism.Curves(aCurves);
        Assert.True(aCurves.Length() > 4);
    }

    [Fact]
    public void A_prism_of_a_shell_sweeps_its_free_edges_and_closes_on_its_two_ends()
    {
        (Shape aShell, _, _) = TwoFaces();
        var aPrism = new LocalPrism(aShell, new Vector3D(0, 0, 5));
        Shape aRes = aPrism.Shape();
        Assert.True(LocOpeTests.AllFrozen(aRes));
        // the two faces at each end, and one side for each of the 6 free edges: not the edge the faces share
        Assert.Equal(2 + 2 + 6, Count(aRes, ShapeType.Face));
        var aForm = new LinearForm(aShell, new Vector3D(0, 0, 5), new Point3D(), new Point3D(1, 0, 0));
        Assert.Equal(10, Count(aForm.Shape(), ShapeType.Face));
    }

    // The top face and the side x = 10 of the box, sharing an edge, as a shell
    private static (Shape Shell, Face Top, Face Side) TwoFaces()
    {
        Shape aBox = Box();
        Face aTop = LocOpeTests.TopFace(aBox);
        Face aSide = FaceAt(aBox, p => Math.Abs(p.X - 10) < 1e-9);
        var aBuilder = new BrepBuilder();
        Shell aShell = new();
        aBuilder.MakeShell(ref aShell);
        aBuilder.Add(aShell, aTop);
        aBuilder.Add(aShell, aSide);
        return (aShell, aTop, aSide);
    }

    [Fact]
    public void A_linear_form_with_a_transfer_keeps_the_volume_and_moves_the_start()
    {
        Face aFace = BRepFeatTests.Rect(0, 4, 0, 6, 0);
        var aForm = new LinearForm(aFace, new Vector3D(0, 0, 5), new Vector3D(0, 0, -2), new Point3D(), new Point3D(1, 0, 0));
        Assert.Equal(120, Math.Abs(Volume(aForm.Shape())), 7);
        var aProps = new Properties();
        MassPropertiesBuilder.SurfaceProperties(aForm.FirstShape(), aProps);
        Assert.Equal(-2, aProps.CentreOfMass().Z, 9);
        MassPropertiesBuilder.SurfaceProperties(aForm.LastShape(), aProps = new Properties());
        Assert.Equal(3, aProps.CentreOfMass().Z, 9);
    }

    [Fact]
    public void A_revolution_of_a_face_follows_Pappus_and_starts_from_the_decayed_angle()
    {
        Face aFace = BRepFeatTests.Rect(10, 14, 0, 6, 0); // its centroid 12 from the y axis
        var anAxis = new Axis3D(new Point3D(), new Direction3D(0, 1, 0));
        var aRevol = new LocalRevol();
        aRevol.Perform(aFace, anAxis, 0.5);
        Assert.Equal(24 * 12 * 0.5, Math.Abs(Volume(aRevol.Shape())), 7);
        var aFrom = new LocalRevol();
        aFrom.Perform(aFace, anAxis, 0.5, 0.25);
        Assert.Equal(24 * 12 * 0.5, Math.Abs(Volume(aFrom.Shape())), 7);
        // every curve of a revolution is a circle about the axis through its point
        var aCurves = new NCollection_Sequence<Curve3D>();
        aRevol.Curves(aCurves);
        foreach (Curve3D aCurve in aCurves)
        {
            var aCircle = Assert.IsType<CircleCurve3D>(aCurve);
            Assert.True(aCircle.Circle.Axis.IsCoaxial(anAxis, 1e-9, 1e-9) || aCircle.Circle.Axis.Reversed().IsCoaxial(anAxis, 1e-9, 1e-9) || aCircle.Circle.Axis.Direction.IsParallel(anAxis.Direction, 1e-9));
        }
        Curve3D? aBaryc = aRevol.BarycCurve();
        var aMiddle = Assert.IsType<CircleCurve3D>(aBaryc);
        Assert.Equal(12, aMiddle.Radius, 9);
        Assert.Equal(3, aMiddle.Circle.Location.Y, 9);
        // a point on the axis has no circle
        Circle3D aCi = new();
        Assert.False(LocalRevol.FindCircle(anAxis, new Point3D(0, 5, 0), ref aCi));
        Assert.True(LocalRevol.FindCircle(anAxis, new Point3D(3, 5, 0), ref aCi));
        Assert.Equal(3, aCi.Radius, 12);
    }

    [Fact]
    public void A_revolution_form_of_a_face_follows_Pappus()
    {
        Face aFace = BRepFeatTests.Rect(10, 14, 0, 6, 0);
        var aForm = new RevolutionForm();
        aForm.Perform(aFace, new Axis3D(new Point3D(), new Direction3D(0, 1, 0)), 0.5);
        Assert.Equal(24 * 12 * 0.5, Math.Abs(Volume(aForm.Shape())), 7);
        Assert.False(aForm.FirstShape().IsNull);
        Assert.False(aForm.LastShape().IsNull);
    }

    [Fact]
    public void Revolutions_of_a_shell_keep_their_edges_descendants()
    {
        (Shape aShell, _, _) = TwoFaces();
        var anAxis = new Axis3D(new Point3D(-5, 0, 0), new Direction3D(0, 1, 0));
        var aRevol = new LocalRevol();
        aRevol.Perform(aShell, anAxis, 0.5);
        Shape aRes = aRevol.Shape();
        Assert.True(LocOpeTests.AllFrozen(aRes));
        Assert.True(Count(aRes, ShapeType.Face) > 2);
        var aForm = new RevolutionForm();
        aForm.Perform(aShell, anAxis, 0.5);
        Assert.Equal(Count(aRes, ShapeType.Face), Count(aForm.Shape(), ShapeType.Face));
    }

    [Fact]
    public void Sweeps_refuse_to_answer_before_they_have_run()
    {
        Assert.Throws<NotDoneException>(() => new LocalRevol().Shape());
        Assert.Throws<NotDoneException>(() => new RevolutionForm().Shape());
        Assert.Throws<NotDoneException>(() => new LinearForm().Shape());
        Assert.Throws<NotDoneException>(() => new LocalPrism().Shape());
    }

    // Pipes, directly

    [Fact]
    public void A_pipe_along_two_edges_and_its_curves()
    {
        Face aFace = BRepFeatTests.Rect(3, 7, 7, 13, 0);
        var aPoles = new NCollection_Array1<Point3D>(1, 3);
        aPoles[1] = new Point3D(4, 9, 8);
        aPoles[2] = new Point3D(4, 9, 12);
        aPoles[3] = new Point3D(7, 9, 16);
        Edge aFirst = new MakeEdge(new TrimmedCurve3D(new BezierCurve3D(Bent()), 0, 1)).Edge();
        Edge aSecond = new MakeEdge(new BezierCurve3D(aPoles)).Edge();
        Wire aSpine = new MakeWire(aFirst, aSecond).Wire();
        var aPipe = new LocalPipe(aSpine, aFace);
        Assert.True(aPipe.Spine().IsSame(aSpine));
        Assert.True(aPipe.Profile().IsSame(aFace));
        Assert.True(LocOpeTests.AllFrozen(aPipe.Shape()));
        Assert.False(aPipe.FirstShape().IsNull);
        Assert.False(aPipe.LastShape().IsNull);
        var aPoints = new NCollection_Sequence<Point3D>();
        aPoints.Append(new Point3D(3, 7, 0));
        aPoints.Append(new Point3D(7, 13, 0));
        NCollection_Sequence<Curve3D?> aCurves = aPipe.Curves(aPoints);
        Assert.Equal(2, aCurves.Length());
        foreach (Curve3D? aCurve in aCurves)
        {
            // a curve runs from the profile's corner along both edges, joined
            var aSpline = Assert.IsType<BSplineCurve3D>(aCurve);
            Assert.Equal(0, aSpline.StartPoint.Z, 7);
            Assert.InRange(aSpline.EndPoint.Z, 14, 18);
        }
        var aBaryc = Assert.IsType<BSplineCurve3D>(aPipe.BarycCurve());
        Assert.Equal(5, aBaryc.StartPoint.X, 7);
        Assert.Equal(10, aBaryc.StartPoint.Y, 7);
    }

    private static NCollection_Array1<Point3D> Bent()
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 3);
        aPoles[1] = new Point3D(4, 9, 0);
        aPoles[2] = new Point3D(4, 9, 4);
        aPoles[3] = new Point3D(4, 9, 8);
        return aPoles;
    }

    [Fact]
    public void The_pipe_of_a_shell_tracks_its_edges()
    {
        (Shape aShell, _, _) = TwoFaces();
        Wire aSpine = new MakeWire(new MakeEdge(new BezierCurve3D(Bent())).Edge()).Wire();
        var aPipe = new LocalPipe(aSpine, aShell);
        Assert.Equal(10, Count(aPipe.Shape(), ShapeType.Face));
    }

    // The split drafts: OCCT only gets past its pipes when the two new planes don't meet and the wire isn't on a line
    // (a pipe's section can't be an infinite line: occt-issues.md)

    private static Wire StraightBezier(double x1, double y1, double z1, double x2, double y2, double z2)
    {
        var aPoles = new NCollection_Array1<Point3D>(1, 2);
        aPoles[1] = new Point3D(x1, y1, z1);
        aPoles[2] = new Point3D(x2, y2, z2);
        return new MakeWire(new MakeEdge(new BezierCurve3D(aPoles)).Edge()).Wire();
    }

    [Theory]
    [InlineData(5.0)]
    [InlineData(-5.0)]
    public void A_split_draft_with_one_plane_for_both_sides_tilts_the_whole_face(double theDegrees)
    {
        Shape aBox = Box();
        Face aSide = FaceAt(aBox, p => Math.Abs(p.X - 10) < 1e-9);
        ShapeSnapshot aSnap = ShapeSnapshot.Of(aBox);
        var aDrafts = new SplitDrafts(aBox);
        var aZ = new Direction3D(0, 0, 1);
        var aNeutral = new Plane3D(new Point3D(0, 0, 0), aZ);
        double anAngle = theDegrees * Math.PI / 180;
        aDrafts.Perform(aSide, StraightBezier(10, 10, 0, 10, 10, 30), aZ, aNeutral, anAngle, aZ, aNeutral, anAngle);
        aSnap.AssertUnchanged(aBox);
        Assert.True(aDrafts.IsDone);
        Shape aRes = aDrafts.Shape();
        Assert.True(new Analyzer(aRes).IsValid());
        // a wedge of 30 high, 30 tan(angle) deep at the top and 20 wide
        Assert.Equal(BoxVolume - 0.5 * 30 * 30 * Math.Tan(anAngle) * 20, Volume(aRes), 5);
        Assert.Equal(7, Count(aRes, ShapeType.Face));
        Assert.Equal(2, aDrafts.ShapesFromShape(aSide).Extent());
    }

    [Fact]
    public void A_split_draft_with_a_wire_on_a_line_throws_where_OCCT_does()
    {
        Shape aBox = Box();
        Face aSide = FaceAt(aBox, p => Math.Abs(p.X - 10) < 1e-9);
        Wire aWire = new MakeWire(Segment(10, 10, 0, 10, 10, 30)).Wire();
        var aDrafts = new SplitDrafts(aBox);
        var aZ = new Direction3D(0, 0, 1);
        var aNeutral = new Plane3D(new Point3D(0, 0, 0), aZ);
        Assert.Throws<DomainErrorException>(() => aDrafts.Perform(aSide, aWire, aZ, aNeutral, 0.1, aZ, aNeutral, 0.1));
        Assert.False(aDrafts.IsDone);
    }

    [Fact]
    public void A_split_draft_checks_what_it_is_given()
    {
        Shape aBox = Box();
        Face aSide = FaceAt(aBox, p => Math.Abs(p.X - 10) < 1e-9);
        Wire aWire = new MakeWire(Segment(10, 10, 0, 10, 10, 30)).Wire();
        var aZ = new Direction3D(0, 0, 1);
        var aNeutral = new Plane3D(new Point3D(0, 0, 0), aZ);
        var aDrafts = new SplitDrafts();
        Assert.Throws<NullObjectException>(() => aDrafts.Perform(aSide, aWire, aZ, aNeutral, 0.1));
        aDrafts.Init(aBox);
        Assert.Throws<NullObjectException>(() => aDrafts.Perform(new Face(), aWire, aZ, aNeutral, 0.1));
        Assert.Throws<NullObjectException>(() => aDrafts.Perform(aSide, new Wire(), aZ, aNeutral, 0.1));
        Assert.Throws<ConstructionErrorException>(() => aDrafts.Perform(aSide, aWire, aZ, aNeutral, 0.1, aZ, aNeutral, 0.1, false, false));
        // a face that isn't the shape's
        Assert.Throws<ConstructionErrorException>(() => aDrafts.Perform(BRepFeatTests.Rect(0, 1, 0, 1, 50), aWire, aZ, aNeutral, 0.1));
        // a face on a curved surface has no plane to tilt: nothing is done
        Shape aCylinder = new ApiMakeCylinder(2, 10).Shape();
        Face aRound = FaceAt(aCylinder, p => Math.Abs(p.Z - 5) < 1e-9);
        var aOnRound = new SplitDrafts(aCylinder);
        aOnRound.Perform(aRound, aWire, aZ, aNeutral, 0.1);
        Assert.False(aOnRound.IsDone);
        // a neutral plane parallel to the face has no line to turn it about: nothing is done
        var aParallel = new SplitDrafts(aBox);
        aParallel.Perform(aSide, aWire, aZ, new Plane3D(new Point3D(0, 0, 0), new Direction3D(1, 0, 0)), 0.1);
        Assert.False(aParallel.IsDone);
        // an extraction along the turning line can't give a draft: nothing is done
        var anAlong = new SplitDrafts(aBox);
        anAlong.Perform(aSide, aWire, new Direction3D(0, 1, 0), aNeutral, 0.1);
        Assert.False(anAlong.IsDone);
        // nor an angle wider than the extraction allows
        var aTooWide = new SplitDrafts(aBox);
        aTooWide.Perform(aSide, aWire, new Direction3D(0, 0.8, 0.6), aNeutral, 1.2);
        Assert.False(aTooWide.IsDone);
    }

    // Splits

    [Fact]
    public void An_open_wire_at_a_time_across_a_face_splits_it_in_as_many_more()
    {
        Shape aBox = Box();
        Face aTop = LocOpeTests.TopFace(aBox);
        double anArea = Area(aBox);
        var aSplit = new LocalSplitShape(aBox);
        // the vertices first: an edge can't be split once a wire has split its face
        var aWires = new List<Wire>();
        foreach (double aY in new[] { 7.0, 13.0 })
        {
            Vertex aV1 = VertexOnEdge(aSplit, aTop, new Point3D(0, aY, 30));
            Vertex aV2 = VertexOnEdge(aSplit, aTop, new Point3D(10, aY, 30));
            aWires.Add(new MakeWire(new MakeEdge(aV1, aV2).Edge()).Wire());
        }
        foreach (Wire aWire in aWires)
        {
            Assert.True(aSplit.Add(aWire, aTop));
        }
        Shape aRes = aSplit.DescendantShapes(aBox).First();
        Assert.Equal(anArea, Area(aRes), 7);
        Assert.Equal(8, Count(aRes, ShapeType.Face));
        Assert.True(new Analyzer(aRes).IsValid());
        Assert.Equal(3, aSplit.DescendantShapes(aTop).Extent());
    }

    [Fact]
    public void Two_sections_at_once_across_a_face_split_it_like_one_after_the_other()
    {
        Shape aBox = Box();
        Face aTop = LocOpeTests.TopFace(aBox);
        double anArea = Area(aBox);
        var aSplit = new LocalSplitShape(aBox);
        var aWires = new NCollection_List<Shape>();
        foreach (double aY in new[] { 7.0, 13.0 })
        {
            Vertex aV1 = VertexOnEdge(aSplit, aTop, new Point3D(0, aY, 30));
            Vertex aV2 = VertexOnEdge(aSplit, aTop, new Point3D(10, aY, 30));
            aWires.Append(new MakeWire(new MakeEdge(aV1, aV2).Edge()).Wire());
        }
        Assert.True(aSplit.Add(aWires, aTop));
        Assert.Equal(3, aSplit.DescendantShapes(aTop).Extent());
        Shape aRes = aSplit.DescendantShapes(aBox).First();
        Assert.Equal(anArea, Area(aRes), 7);
        Assert.Equal(8, Count(aRes, ShapeType.Face));
        Assert.True(new Analyzer(aRes).IsValid());
    }

    [Fact]
    public void Two_sections_from_one_vertex_of_the_boundary_choose_their_way_at_it()
    {
        // a V from the middle of the near edge to both sides: three faces, the wedge between them with 3 edges' worth
        Shape aBox = Box();
        Face aTop = LocOpeTests.TopFace(aBox);
        double anArea = Area(aBox);
        var aSplit = new LocalSplitShape(aBox);
        Vertex aFoot = VertexOnEdge(aSplit, aTop, new Point3D(5, 0, 30));
        Vertex aLeft = VertexOnEdge(aSplit, aTop, new Point3D(0, 10, 30));
        Vertex aRight = VertexOnEdge(aSplit, aTop, new Point3D(10, 10, 30));
        var aWires = new NCollection_List<Shape>();
        aWires.Append(new MakeWire(new MakeEdge(aFoot, aLeft).Edge()).Wire());
        aWires.Append(new MakeWire(new MakeEdge(aFoot, aRight).Edge()).Wire());
        Assert.True(aSplit.Add(aWires, aTop));
        Assert.Equal(3, aSplit.DescendantShapes(aTop).Extent());
        Shape aRes = aSplit.DescendantShapes(aBox).First();
        Assert.Equal(anArea, Area(aRes), 7);
        Assert.True(new Analyzer(aRes).IsValid());
        double[] anAreas = new double[3];
        int k = 0;
        foreach (Shape aFace in aSplit.DescendantShapes(aTop))
        {
            anAreas[k++] = Area(aFace);
        }
        Array.Sort(anAreas);
        Assert.Equal(25, anAreas[0], 7); // the triangles at the corners, 5 by 10
        Assert.Equal(25, anAreas[1], 7);
        Assert.Equal(150, anAreas[2], 7);
    }

    [Fact]
    public void A_wire_off_the_face_is_not_added()
    {
        Shape aBox = Box();
        Face aTop = LocOpeTests.TopFace(aBox);
        var aSplit = new LocalSplitShape(aBox);
        var aWires = new NCollection_List<Shape>();
        aWires.Append(new MakeWire(new MakeEdge(new Circle3D(new Frame3D(new Point3D(55, 5, 30), new Direction3D(0, 0, 1)), 2)).Edge()).Wire());
        Assert.False(aSplit.Add(aWires, aTop));
    }

    private static Vertex VertexOnEdge(LocalSplitShape theSplit, Face theFace, Point3D theP)
    {
        for (var anExp = new Explorer(theFace, ShapeType.Edge); anExp.More(); anExp.Next())
        {
            Edge anEdge = ShapeCasts.Edge(anExp.Current());
            Tool.Range(anEdge, out double aF, out double aL);
            Curve3D aCurve = Tool.Curve(anEdge, out _, out _)!;
            var aProject = new Kerf.Engine.Geometry.Api.ProjectPointOnCurve(theP, aCurve, aF, aL);
            if (aProject.NbPoints() > 0 && aProject.LowerDistance() < 1e-7)
            {
                Vertex aV = new MakeVertex(theP).Vertex();
                theSplit.Add(aV, aProject.LowerDistanceParameter(), anEdge);
                return aV;
            }
        }
        throw new InvalidOperationException("no edge there");
    }
}
