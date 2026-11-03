// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Pnt2dBisec.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Pnt2dBisec.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Constraints.Analytic;

/// <summary>
/// The bisecting line of two 2D points: the perpendicular bisector of the segment joining them. There is no solution
/// if the points coincide.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117).</remarks>
public sealed class Pnt2dBisec
{
    private bool _wellDone;
    private bool _hasSol;
    private Line2D _linsol = new();

    /// <summary>Constructs the bisecting line of Point1 and Point2.</summary>
    public Pnt2dBisec(in Point2D point1, in Point2D point2)
    {
        _wellDone = false;
        //   if (Point1.Distance(Point2) > gp::Resolution()) {
        if (point1.Distance(point2) > 1.0e-10)
        {
            Direction2D dir1 = new(point2.XY() - point1.XY());
            _linsol = new Line2D(new Point2D((point2.X() + point1.X()) / 2.0, (point2.Y() + point1.Y()) / 2.0),
                                  new Direction2D(-dir1.Y(), dir1.X()));
            _hasSol = true;
            _wellDone = true;
        }
        else
        {
            _hasSol = false;
            _wellDone = true;
        }
    }

    /// <summary>True: this construction never fails.</summary>
    public bool IsDone
    {
        get
        {
            return _wellDone;
        }
    }

    /// <summary>True if the points are not coincident.</summary>
    public bool HasSolution
    {
        get
        {
            return _hasSol;
        }
    }

    /// <summary>The bisecting line.</summary>
    /// <exception cref="NoSolutionException">The points coincide.</exception>
    public Line2D ThisSolution()
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (!_hasSol)
        {
            throw new NoSolutionException();
        }
        return _linsol;
    }
}
