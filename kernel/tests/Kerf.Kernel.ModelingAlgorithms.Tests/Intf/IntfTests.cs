// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Engine.Intersection.Polygons.PIType;

namespace Kerf.Kernel.ModelingAlgorithms.Tests;

/// <summary>Intf's statics, section points, lines and zones, and the 2D polygon interference on known polygons.</summary>
public class IntfTests
{
    /// <summary>A polyline through the given points.</summary>
    private sealed class TestPolygon : global::Kerf.Engine.Intersection.Polygons.Polygon2d
    {
        private readonly Point2D[] myPoints;
        private readonly double myDeflection;
        private readonly bool myClosed;

        public TestPolygon(double theDeflection, bool theClosed, params (double X, double Y)[] thePoints)
        {
            myPoints = [.. thePoints.Select(p => new Point2D(p.X, p.Y))];
            myDeflection = theDeflection;
            myClosed = theClosed;
            foreach (Point2D aP in myPoints)
            {
                _box.Add(aP);
            }

            _box.Enlarge(theDeflection);
        }

        public override bool Closed() => myClosed;

        public override double DeflectionOverEstimation() => myDeflection;

        public override int SegmentCount => myPoints.Length - 1;

        public override void Segment(int theIndex, out Point2D theBegin, out Point2D theEnd)
        {
            theBegin = myPoints[theIndex - 1];
            theEnd = myPoints[theIndex];
        }
    }

    private static global::Kerf.Engine.Intersection.Polygons.SectionPoint SP(int i)
        => new(new Point2D(i, 0), global::Kerf.Engine.Intersection.Polygons.PIType.Vertex, i, 0.0, global::Kerf.Engine.Intersection.Polygons.PIType.Vertex, i, 0.0, 0.0);

    [Fact]
    public void PlaneEquationAndContain()
    {
        Point3D aP1 = new(0, 0, 2), aP2 = new(1, 0, 2), aP3 = new(0, 1, 2);
        global::Kerf.Engine.Intersection.Polygons.Intf.PlaneEquation(aP1, aP2, aP3, out Xyz aNormal, out double aDist);
        Assert.Equal(0, aNormal.X(), 15);
        Assert.Equal(0, aNormal.Y(), 15);
        Assert.Equal(1, aNormal.Z(), 15);
        Assert.Equal(2, aDist, 15);

        global::Kerf.Engine.Intersection.Polygons.Intf.PlaneEquation(aP1, aP1, aP2, out _, out aDist);
        Assert.Equal(0, aDist);

        Assert.True(global::Kerf.Engine.Intersection.Polygons.Intf.Contain(aP1, aP2, aP3, new Point3D(0.2, 0.2, 2)));
        Assert.True(global::Kerf.Engine.Intersection.Polygons.Intf.Contain(aP1, aP2, aP3, new Point3D(0.5, 0, 2)));
        Assert.False(global::Kerf.Engine.Intersection.Polygons.Intf.Contain(aP1, aP2, aP3, new Point3D(1, 1, 2)));
    }

    [Fact]
    public void SectionPoint_IsEqualComparesLogicalInformation()
    {
        var aA = new global::Kerf.Engine.Intersection.Polygons.SectionPoint(new Point2D(1, 2), global::Kerf.Engine.Intersection.Polygons.PIType.Edge, 3, 0.25, global::Kerf.Engine.Intersection.Polygons.PIType.Vertex, 4, 0.0, 0.5);
        var aB = new global::Kerf.Engine.Intersection.Polygons.SectionPoint(new Point2D(5, 6), global::Kerf.Engine.Intersection.Polygons.PIType.Edge, 3, 0.75, global::Kerf.Engine.Intersection.Polygons.PIType.Vertex, 4, 0.0, 0.1);
        Assert.True(aA.IsEqual(aB)); // OCCT's operator==
        Assert.False(aA == aB);      // exact comparison
        Assert.Equal(2.25, aA.ParamOnFirst());
        Assert.Equal(3.0, aA.ParamOnSecond());
        Assert.Equal(new Point3D(1, 2, 0), aA.Pnt());

        aA.InfoFirst(out global::Kerf.Engine.Intersection.Polygons.PIType aDim, out int anAdd1, out int anAdd2, out double aParam);
        Assert.Equal((global::Kerf.Engine.Intersection.Polygons.PIType.Edge, 0, 3, 0.25), (aDim, anAdd1, anAdd2, aParam));
        aA.InfoSecond(out aDim, out int anAdd, out aParam);
        Assert.Equal((global::Kerf.Engine.Intersection.Polygons.PIType.Vertex, 4, 0.0), (aDim, anAdd, aParam));
        Assert.Equal(new global::Kerf.Engine.Intersection.Polygons.SectionPoint(), default);
    }

    [Fact]
    public void SectionPoint_IsOnSameEdge()
    {
        Point3D aP = new(0, 0, 0);
        var anEdge12 = new global::Kerf.Engine.Intersection.Polygons.SectionPoint(aP, global::Kerf.Engine.Intersection.Polygons.PIType.Edge, 1, 2, 0.5, global::Kerf.Engine.Intersection.Polygons.PIType.Face, 7, 8, 0.0, 0.0);
        var anEdge12b = new global::Kerf.Engine.Intersection.Polygons.SectionPoint(aP, global::Kerf.Engine.Intersection.Polygons.PIType.Edge, 1, 2, 0.1, global::Kerf.Engine.Intersection.Polygons.PIType.Face, 9, 10, 0.0, 0.0);
        var aVertex2 = new global::Kerf.Engine.Intersection.Polygons.SectionPoint(aP, global::Kerf.Engine.Intersection.Polygons.PIType.Vertex, 2, 0, 0.0, global::Kerf.Engine.Intersection.Polygons.PIType.Face, 9, 10, 0.0, 0.0);
        Assert.True(anEdge12.IsOnSameEdge(anEdge12b));
        Assert.True(aVertex2.IsOnSameEdge(anEdge12));

        // OCCT: two points on the same vertex of the first argument compare the second argument's IndexT1
        var aVertex5a = new global::Kerf.Engine.Intersection.Polygons.SectionPoint(aP, global::Kerf.Engine.Intersection.Polygons.PIType.Vertex, 5, 0, 0.0, global::Kerf.Engine.Intersection.Polygons.PIType.Face, 1, 2, 0.0, 0.0);
        var aVertex5b = new global::Kerf.Engine.Intersection.Polygons.SectionPoint(aP, global::Kerf.Engine.Intersection.Polygons.PIType.Vertex, 5, 0, 0.0, global::Kerf.Engine.Intersection.Polygons.PIType.Face, 3, 4, 0.0, 0.0);
        Assert.False(aVertex5a.IsOnSameEdge(aVertex5b));
    }

    [Fact]
    public void SectionPoint_MergeKeepsHigherDimension()
    {
        var aA = new global::Kerf.Engine.Intersection.Polygons.SectionPoint(new Point3D(1, 1, 1), global::Kerf.Engine.Intersection.Polygons.PIType.Edge, 1, 2, 0.5, global::Kerf.Engine.Intersection.Polygons.PIType.Face, 3, 4, 0.1, 0.0);
        var aB = new global::Kerf.Engine.Intersection.Polygons.SectionPoint(new Point3D(2, 2, 2), global::Kerf.Engine.Intersection.Polygons.PIType.Vertex, 7, 0, 0.0, global::Kerf.Engine.Intersection.Polygons.PIType.Edge, 5, 6, 0.2, 0.0);
        aA.Merge(ref aB);
        Assert.True(aA.IsEqual(aB));
        Assert.Equal(global::Kerf.Engine.Intersection.Polygons.PIType.Vertex, aA.TypeOnFirst());
        Assert.Equal(global::Kerf.Engine.Intersection.Polygons.PIType.Edge, aA.TypeOnSecond());
        Assert.Equal(new Point3D(1, 1, 1), aB.Pnt());
    }

    [Fact]
    public void SectionLine_AppendPrependAndEnds()
    {
        var aLine = new global::Kerf.Engine.Intersection.Polygons.SectionLine();
        aLine.Append(SP(2));
        aLine.Append(SP(3));
        aLine.Prepend(SP(1));
        Assert.Equal(3, aLine.NumberOfPoints());
        Assert.Equal(1, aLine.IsEnd(SP(1)));
        Assert.Equal(3, aLine.IsEnd(SP(3)));
        Assert.Equal(0, aLine.IsEnd(SP(2)));
        Assert.True(aLine.Contains(SP(2)));
        Assert.False(aLine.IsClosed);

        var aCopy = new global::Kerf.Engine.Intersection.Polygons.SectionLine(aLine);
        aLine.Reverse();
        Assert.Equal(SP(3), aLine.GetPoint(1));
        Assert.Equal(SP(1), aCopy.GetPoint(1));
        Assert.False(aLine.IsEqual(aCopy));
        aLine.Reverse();
        Assert.True(aLine.IsEqual(aCopy));

        aLine.Append(SP(1));
        Assert.True(aLine.IsClosed); // the ends are logically equal; the closed flag isn't read
    }

    [Fact]
    public void TangentZone_RangesAndPolygonInsert()
    {
        var aZone = new global::Kerf.Engine.Intersection.Polygons.TangentZone();
        var aP1 = new global::Kerf.Engine.Intersection.Polygons.SectionPoint(new Point2D(0, 0), global::Kerf.Engine.Intersection.Polygons.PIType.Edge, 2, 0.5, global::Kerf.Engine.Intersection.Polygons.PIType.Edge, 3, 0.25, 0.0);
        var aP2 = new global::Kerf.Engine.Intersection.Polygons.SectionPoint(new Point2D(1, 0), global::Kerf.Engine.Intersection.Polygons.PIType.Vertex, 3, 0.0, global::Kerf.Engine.Intersection.Polygons.PIType.Edge, 4, 0.75, 0.0);
        var aP3 = new global::Kerf.Engine.Intersection.Polygons.SectionPoint(new Point2D(2, 0), global::Kerf.Engine.Intersection.Polygons.PIType.Edge, 2, 0.75, global::Kerf.Engine.Intersection.Polygons.PIType.Edge, 3, 0.5, 0.0);
        aZone.PolygonInsert(aP1);
        aZone.PolygonInsert(aP2);
        aZone.PolygonInsert(aP3); // inside the range on the first argument: inserted first
        Assert.Equal(3, aZone.NumberOfPoints());
        Assert.Equal(aP3, aZone.GetPoint(1));
        aZone.ParamOnFirst(out double aMin, out double aMax);
        Assert.Equal((1.5, 2.0), (aMin, aMax));
        aZone.InfoSecond(out int aSegMin, out double aParMin, out int aSegMax, out double aParMax);
        Assert.Equal((2, 0.25, 3, 0.75), (aSegMin, aParMin, aSegMax, aParMax));
        Assert.True(aZone.Contains(aP2));
        Assert.True(aZone.RangeContains(new global::Kerf.Engine.Intersection.Polygons.SectionPoint(new Point2D(), global::Kerf.Engine.Intersection.Polygons.PIType.Edge, 2, 0.6, global::Kerf.Engine.Intersection.Polygons.PIType.Edge, 3, 0.9, 0.0)));
        Assert.False(aZone.Insert(aP1)); // OCCT's Insert does nothing

        var anOther = new global::Kerf.Engine.Intersection.Polygons.TangentZone();
        anOther.Append(new global::Kerf.Engine.Intersection.Polygons.SectionPoint(new Point2D(), global::Kerf.Engine.Intersection.Polygons.PIType.Edge, 2, 0.9, global::Kerf.Engine.Intersection.Polygons.PIType.Edge, 4, 0.5, 0.0));
        Assert.True(aZone.HasCommonRange(anOther));
        var aFar = new global::Kerf.Engine.Intersection.Polygons.TangentZone();
        aFar.Append(new global::Kerf.Engine.Intersection.Polygons.SectionPoint(new Point2D(), global::Kerf.Engine.Intersection.Polygons.PIType.Edge, 9, 0.5, global::Kerf.Engine.Intersection.Polygons.PIType.Edge, 4, 0.9, 0.0));
        Assert.False(aZone.HasCommonRange(aFar));

        var aCopy = new global::Kerf.Engine.Intersection.Polygons.TangentZone(aZone);
        aZone.Append(anOther);
        Assert.Equal(4, aZone.NumberOfPoints());
        Assert.Equal(3, aCopy.NumberOfPoints());
        Assert.False(aZone.IsEqual(aCopy));
    }

    [Fact]
    public void Interference_InsertBuildsAndJoinsSectionLines()
    {
        var anInter = new global::Kerf.Engine.Intersection.Polygons.InterferencePolygon2d();
        anInter.Insert(SP(1), SP(2));
        anInter.Insert(SP(2), SP(3));
        anInter.Insert(SP(4), SP(1));
        Assert.Equal(1, anInter.SectionLineCount);
        Assert.Equal(4, anInter.LineValue(1).NumberOfPoints());
        Assert.Equal(SP(4), anInter.LineValue(1).GetPoint(1));
        Assert.False(anInter.LineValue(1).IsClosed);
        anInter.Insert(SP(3), SP(4));
        Assert.Equal(5, anInter.LineValue(1).NumberOfPoints());
        Assert.True(anInter.LineValue(1).IsClosed);
        Assert.True(anInter.Contains(SP(2)));
        Assert.False(anInter.Contains(SP(9)));

        // two lines joined by a segment between their ends
        var aJoin = new global::Kerf.Engine.Intersection.Polygons.InterferencePolygon2d();
        aJoin.Insert(SP(1), SP(2));
        aJoin.Insert(SP(3), SP(4));
        Assert.Equal(2, aJoin.SectionLineCount);
        aJoin.Insert(SP(2), SP(3));
        Assert.Equal(1, aJoin.SectionLineCount);
        global::Kerf.Engine.Intersection.Polygons.SectionLine aLine = aJoin.LineValue(1);
        Assert.Equal(4, aLine.NumberOfPoints());
        for (int i = 1; i <= 4; i++)
        {
            Assert.Equal(SP(i), aLine.GetPoint(i));
        }
    }

    [Fact]
    public void InterferencePolygon2d_CrossingSegments()
    {
        var aP1 = new TestPolygon(0, false, (0, 0), (2, 2));
        var aP2 = new TestPolygon(0, false, (0, 2), (2, 0));
        var anInter = new global::Kerf.Engine.Intersection.Polygons.InterferencePolygon2d(aP1, aP2);

        Assert.Equal(1, anInter.SectionPointCount);
        Assert.Equal(0, anInter.TangentZoneCount);
        Assert.Equal(Epsilon(1000.0), anInter.GetTolerance());
        global::Kerf.Engine.Intersection.Polygons.SectionPoint aPt = anInter.PntValue(1);
        Assert.Equal(global::Kerf.Engine.Intersection.Polygons.PIType.Edge, aPt.TypeOnFirst());
        Assert.Equal(global::Kerf.Engine.Intersection.Polygons.PIType.Edge, aPt.TypeOnSecond());
        Assert.Equal(0.5, aPt.ParamOnFirst(), 15);
        Assert.Equal(0.5, aPt.ParamOnSecond(), 15);
        Assert.Equal(1.0, aPt.Incidence(), 15);
        Assert.Equal(new Point2D(1, 1), anInter.Pnt2dValue(1));
    }

    [Fact]
    public void InterferencePolygon2d_CrossingAtVertex()
    {
        // The first polyline turns at (1, 1), on the second: each of its segments gives an EDGE/EDGE point there.
        var aP1 = new TestPolygon(0, false, (0, 0), (1, 1), (2, 0));
        var aP2 = new TestPolygon(0, false, (1, 0), (1, 2));
        var anInter = new global::Kerf.Engine.Intersection.Polygons.InterferencePolygon2d(aP1, aP2);
        Assert.Equal(2, anInter.SectionPointCount);
        for (int i = 1; i <= 2; i++)
        {
            Assert.Equal(1.0, anInter.PntValue(i).ParamOnFirst(), 15);
            Assert.Equal(0.5, anInter.PntValue(i).ParamOnSecond(), 15);
            Assert.Equal(1.0, anInter.Pnt2dValue(i).X, 15);
            Assert.Equal(1.0, anInter.Pnt2dValue(i).Y, 15);
        }
    }

    [Fact]
    public void InterferencePolygon2d_OverlapGivesTangentZone()
    {
        var aP1 = new TestPolygon(0, false, (0, 0), (2, 0));
        var aP2 = new TestPolygon(0, false, (1, 0), (3, 0));
        var anInter = new global::Kerf.Engine.Intersection.Polygons.InterferencePolygon2d(aP1, aP2);
        Assert.Equal(0, anInter.SectionPointCount);
        Assert.Equal(1, anInter.TangentZoneCount);
        global::Kerf.Engine.Intersection.Polygons.TangentZone aZone = anInter.ZoneValue(1);
        Assert.Equal(2, aZone.NumberOfPoints());
        aZone.ParamOnFirst(out double aMin, out double aMax);
        Assert.Equal((0.5, 1.0), (aMin, aMax));
        aZone.ParamOnSecond(out aMin, out aMax);
        Assert.Equal((0.0, 0.5), (aMin, aMax));
    }

    [Fact]
    public void InterferencePolygon2d_DisjointAndPerform()
    {
        var aP1 = new TestPolygon(0, false, (0, 0), (1, 0));
        var aP2 = new TestPolygon(0, false, (0, 5), (1, 5));
        var anInter = new global::Kerf.Engine.Intersection.Polygons.InterferencePolygon2d(aP1, aP2);
        Assert.Equal(0, anInter.SectionPointCount);
        Assert.Equal(0.0, anInter.GetTolerance()); // the boxes don't meet: nothing computed

        anInter.Perform(aP1, new TestPolygon(0, false, (0.5, -1), (0.5, 1)));
        Assert.Equal(1, anInter.SectionPointCount);
        anInter.Perform(aP1, aP2);
        Assert.Equal(0, anInter.SectionPointCount);
    }

    [Fact]
    public void InterferencePolygon2d_SelfIntersection()
    {
        // (0,0)-(2,2)-(2,0)-(0,2): the first and third segments cross at (1, 1); neighbours are skipped.
        var aP = new TestPolygon(0, false, (0, 0), (2, 2), (2, 0), (0, 2));
        var anInter = new global::Kerf.Engine.Intersection.Polygons.InterferencePolygon2d(aP);
        Assert.Equal(1, anInter.SectionPointCount);
        Assert.Equal(0.5, anInter.PntValue(1).ParamOnFirst(), 15);
        Assert.Equal(2.5, anInter.PntValue(1).ParamOnSecond(), 15);
        Assert.Equal(new Point2D(1, 1), anInter.Pnt2dValue(1));

        Assert.Equal(0, new global::Kerf.Engine.Intersection.Polygons.InterferencePolygon2d(new TestPolygon(0, false, (0, 0), (1, 0), (2, 1))).SectionPointCount);
    }
}
