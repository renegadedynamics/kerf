// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.Hatch_LineForm;

namespace OcctNet.ModelingAlgorithms.Tests;

/// <summary>Hatch_Hatcher on known configurations: lines trimmed by the oriented unit square.</summary>
public class Hatch_HatcherTests
{
    // The unit square, counterclockwise (inside on the left), as four trimming segments with indices 1 to 4.
    private static void TrimBySquare(Hatch_Hatcher theHatcher)
    {
        theHatcher.Trim(new Point2D(0, 0), new Point2D(1, 0), 1);
        theHatcher.Trim(new Point2D(1, 0), new Point2D(1, 1), 2);
        theHatcher.Trim(new Point2D(1, 1), new Point2D(0, 1), 3);
        theHatcher.Trim(new Point2D(0, 1), new Point2D(0, 0), 4);
    }

    [Fact]
    public void XAndYLines_TrimmedBySquare()
    {
        Hatch_Hatcher aHatcher = new(1.0e-7);
        aHatcher.AddXLine(0.5);
        aHatcher.AddYLine(0.25);
        TrimBySquare(aHatcher);

        Assert.Equal(2, aHatcher.NbLines());
        Assert.True(aHatcher.IsXLine(1));
        Assert.True(aHatcher.IsYLine(2));
        Assert.Equal(0.5, aHatcher.Coordinate(1));
        Assert.Equal(0.25, aHatcher.Coordinate(2));
        Assert.Equal(2, aHatcher.NbIntervals());

        // The X line x = 0.5 goes up (+Y): it enters at y = 0 through the bottom edge (index 1, at its middle) and
        // leaves at y = 1 through the top edge (index 3)
        Assert.Equal(1, aHatcher.NbIntervals(1));
        Assert.Equal(0.0, aHatcher.Start(1, 1), 1e-15);
        Assert.Equal(1.0, aHatcher.End(1, 1), 1e-15);
        aHatcher.StartIndex(1, 1, out int anIndex, out double aPar2);
        Assert.Equal(1, anIndex);
        Assert.Equal(0.5, aPar2, 1e-15);
        aHatcher.EndIndex(1, 1, out anIndex, out aPar2);
        Assert.Equal(3, anIndex);
        Assert.Equal(0.5, aPar2, 1e-15);

        // The Y line y = 0.25 goes right (+X): from the left edge to the right edge
        Assert.Equal(0.0, aHatcher.Start(2, 1), 1e-15);
        Assert.Equal(1.0, aHatcher.End(2, 1), 1e-15);
        aHatcher.StartIndex(2, 1, out anIndex, out aPar2);
        Assert.Equal(4, anIndex);
        Assert.Equal(0.75, aPar2, 1e-15);
    }

    [Fact]
    public void AnyLine_ByDirectionAndDistance()
    {
        Hatch_Hatcher aHatcher = new(1.0e-7);
        // Direction (0, 1) at distance -0.5: the points P with OP . (-1, 0) = -0.5, the line x = 0.5
        aHatcher.AddLine(new Direction2D(0, 1), -0.5);
        Assert.Equal(Hatch_ANYLINE, aHatcher.LineForm(1));
        Assert.Equal(0.5, aHatcher.Line(1).Location().X(), 1e-15);
        Assert.Throws<OutOfRangeException>(() => aHatcher.Coordinate(1));

        TrimBySquare(aHatcher);
        Assert.Equal(1, aHatcher.NbIntervals(1));
        Assert.Equal(0.0, aHatcher.Start(1, 1), 1e-15);
        Assert.Equal(1.0, aHatcher.End(1, 1), 1e-15);
    }

    [Fact]
    public void Untrimmed_OrientedIsInfinite_UnorientedIsEmpty()
    {
        Hatch_Hatcher anOriented = new(1.0e-7);
        anOriented.AddXLine(5.0);
        TrimBySquare(anOriented);
        Assert.Equal(1, anOriented.NbIntervals(1));
        Assert.Equal(RealFirst(), anOriented.Start(1, 1));
        Assert.Equal(RealLast(), anOriented.End(1, 1));
        anOriented.StartIndex(1, 1, out int anIndex, out double aPar2);
        Assert.Equal(0, anIndex);
        Assert.Equal(0.0, aPar2);

        Hatch_Hatcher anUnoriented = new(1.0e-7, false);
        anUnoriented.AddXLine(5.0);
        TrimBySquare(anUnoriented);
        Assert.Equal(0, anUnoriented.NbIntervals(1));
        Assert.Throws<OutOfRangeException>(() => anUnoriented.Start(1, 1));
    }

    [Fact]
    public void Oriented_HalfOpenIntervals()
    {
        // Trimmed by the single line y = 0 going left (-X): its left side is y < 0, so the X line x = 0.5 (going up)
        // is kept below y = 0: one interval from -infinity, ending at y = 0
        Hatch_Hatcher aHatcher = new(1.0e-7);
        aHatcher.AddXLine(0.5);
        aHatcher.Trim(new Line2D(new Point2D(0, 0), new Direction2D(-1, 0)), 7);
        Assert.Equal(1, aHatcher.NbIntervals(1));
        Assert.Equal(RealFirst(), aHatcher.Start(1, 1));
        Assert.Equal(0.0, aHatcher.End(1, 1), 1e-15);
        aHatcher.EndIndex(1, 1, out int anIndex, out double aPar2);
        Assert.Equal(7, anIndex);
        Assert.Equal(-0.5, aPar2, 1e-15);
    }

    [Fact]
    public void Trim_EndsOfTheTrimmingSegment()
    {
        // A segment ending exactly on the line is kept only if its direction crosses the line from right to left
        // there (OCCT: "accept if the other extremity is on the left")
        Hatch_Hatcher aHatcher = new(1.0e-7);
        aHatcher.AddYLine(1.0); // through the corner (1, 1)
        aHatcher.Trim(new Point2D(1, 0), new Point2D(1, 1), 2);
        aHatcher.Trim(new Point2D(1, 1), new Point2D(0, 1), 3);
        // Segment 2 ends on the line going up: norm = (0, 1) ^ (1, 0) = -1, kept, and ends an interval (its left
        // is x < 1); segment 3 lies on the line: ignored
        Assert.Equal(1, aHatcher.NbIntervals(1));
        Assert.Equal(RealFirst(), aHatcher.Start(1, 1));
        Assert.Equal(1.0, aHatcher.End(1, 1), 1e-15);

        // Starting on the line (the segment from the corner going down): norm = (0, -1) ^ (1, 0) = 1, kept, and
        // starts an interval (its left is x > 1)
        Hatch_Hatcher aDown = new(1.0e-7);
        aDown.AddYLine(1.0);
        aDown.Trim(new Point2D(1, 1), new Point2D(1, 0), 2);
        Assert.Equal(1.0, aDown.Start(1, 1), 1e-15);
        Assert.Equal(RealLast(), aDown.End(1, 1));

        // Starting on the line going up (from (1, 1) to (1, 2)): norm = -1, rejected
        Hatch_Hatcher anUp = new(1.0e-7);
        anUp.AddYLine(1.0);
        anUp.Trim(new Point2D(1, 1), new Point2D(1, 2), 2);
        Assert.Equal(RealFirst(), anUp.Start(1, 1));
        Assert.Equal(RealLast(), anUp.End(1, 1));
    }

    [Fact]
    public void Line_AddIntersection_SortsAndCancelsClosePairs()
    {
        Hatch_Line aLine = new(new Line2D(new Point2D(0, 0), new Direction2D(1, 0)), Hatch_YLINE);
        aLine.AddIntersection(2.0, false, 2, 0.0, 1.0e-7);
        aLine.AddIntersection(1.0, true, 1, 0.0, 1.0e-7);
        aLine.AddIntersection(3.0, true, 3, 0.0, 1.0e-7);
        Assert.Equal(3, aLine.myInters.Length());
        Assert.Equal(1.0, aLine.myInters[1].myPar1);
        Assert.Equal(2.0, aLine.myInters[2].myPar1);
        Assert.Equal(3.0, aLine.myInters[3].myPar1);
        // OCC109: an intersection within the tolerance of another removes it
        aLine.AddIntersection(2.0 + 1.0e-8, true, 4, 0.0, 1.0e-7);
        Assert.Equal(2, aLine.myInters.Length());
        Assert.Equal(3.0, aLine.myInters[2].myPar1);

        Assert.Equal(new Hatch_Parameter(1.0, true, 1, 0.0), aLine.myInters[1]);
        Assert.Equal(default, new Hatch_Parameter());
    }

    [Fact]
    public void Tolerance_SetAndGet()
    {
        Hatch_Hatcher aHatcher = new(1.0e-7);
        aHatcher.Tolerance(0.25);
        Assert.Equal(0.25, aHatcher.Tolerance());
    }
}
