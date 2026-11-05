// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_LinPnt2dBisec.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_LinPnt2dBisec.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Constraints.Analytic;

/// <summary>
/// The bisecting curve of a 2D line and a point: the locus of the points equidistant from both, a parabola, or a line
/// when the point is on the line. There is always one solution.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117).</remarks>
internal sealed class LinPnt2dBisec
{
    private bool _wellDone;
    private ConstraintBisec? _bissol;

    /// <summary>Constructs the bisecting curve of Line1 and Point2.</summary>
    public LinPnt2dBisec(in Line2D line1, in Point2D point2)
    {
        _wellDone = false;

        double xdir = line1.Direction.X;
        double ydir = line1.Direction.Y;
        double xloc = line1.Location.X;
        double yloc = line1.Location.Y;
        double dist = line1.DistanceTo(point2);

        //   if ( dist > gp::Resolution()) {
        if (dist > 1.0e-10)
        {
            double xpoint2 = point2.X;
            double ypoint2 = point2.Y;
            if ((-ydir * (xpoint2 - xloc) + xdir * (ypoint2 - yloc)) > 0.0)
            {
                Axis2D axeparab = new(new Point2D(point2.XY() - dist / 2.0 * new Xy(-ydir, xdir)),
                                       new Direction2D(-ydir, xdir));
                Parabola2D bislinpnt = new(axeparab, dist / 2.0);
                _bissol = new BParab(bislinpnt);
            }
            else
            {
                Axis2D axeparab = new(new Point2D(point2.XY() + dist / 2.0 * new Xy(-ydir, xdir)),
                                       new Direction2D(ydir, -xdir));
                Parabola2D bislinpnt = new(axeparab, dist / 2.0);
                _bissol = new BParab(bislinpnt);
            }
            _wellDone = true;
        }
        else
        {
            Line2D bislinpnt = new(point2, new Direction2D(-ydir, xdir));
            _bissol = new BLine(bislinpnt);
            _wellDone = true;
        }
    }

    /// <summary>True if the construction succeeded.</summary>
    public bool IsDone
    {
        get
        {
            return _wellDone;
        }
    }

    /// <summary>The bisecting curve.</summary>
    public ConstraintBisec ThisSolution()
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        return _bissol!;
    }
}
