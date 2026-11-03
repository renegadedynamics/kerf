// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_CircPnt2dBisec.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_CircPnt2dBisec.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Constraints.Analytic;

/// <summary>
/// The bisecting curves of a 2D circle and a point: the loci of the points equidistant from both. An ellipse when the
/// point is inside the circle, a line when it is on it, the two branches of a hyperbola when it is outside, and a
/// circle when it is the centre.
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117); as in OCCT, each solution is built when it is
/// asked for.
/// </remarks>
public sealed class CircPnt2dBisec
{
    private bool _wellDone;
    private int _nbrSol;
    private Circle2D _circle;
    private Point2D _point;
    private int _theposition;
    private double _tolerance;

    /// <summary>Constructs the bisecting curves of Circle and Point.</summary>
    public CircPnt2dBisec(in Circle2D circle, in Point2D point)
    {
        _circle = circle;
        _point = point;
        _tolerance = 1.0e-10;
        DefineSolutions();
    }

    /// <summary>Constructs the bisecting curves of Circle and Point, with a tolerance (at least 1e-10).</summary>
    public CircPnt2dBisec(in Circle2D circle, in Point2D point, double tolerance)
    {
        _circle = circle;
        _point = point;
        _tolerance = 1.0e-10;
        if (_tolerance < tolerance)
        {
            _tolerance = tolerance;
        }

        DefineSolutions();
    }

    // Defines the number and the type of solutions depending on input data
    private void DefineSolutions()
    {
        double dist = _circle.Radius() - _point.Distance(_circle.Location());

        if (Math.Abs(dist) < _tolerance)
        {
            _theposition = 0;
            _nbrSol = 1;
        }
        else if (dist > 0.0)
        {
            _theposition = -1;
            _nbrSol = 1;
        }
        else
        {
            _theposition = 1;
            _nbrSol = 2;
        }

        _wellDone = true;
    }

    //=========================================================================
    //  Processing.                                                           +
    //  Return the coordinates of origins of the straight line (xloc,yloc) and+
    //  of the circle (xcencirc, ycencirc).                                       +
    //  Also return the coordinates of the direction of the straight line (xdir,   +
    //  ydir) and the radius of circle R1.                                       +
    //  Check at which side of the straight line is found the center of circle    +
    //  to orientate the parabola (sign).                                    +
    //  Create axis of each parabola (axeparab1, axeparb2), then    +
    //  two parabolas (biscirPnt1, biscirPnt1).                          +
    //=========================================================================

    /// <summary>The solution number Index.</summary>
    public ConstraintBisec ThisSolution(int index)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }

        if ((index <= 0) || (index > _nbrSol))
        {
            throw new OutOfRangeException();
        }

        ConstraintBisec bissol;
        double xpoint = _point.X();
        double ypoint = _point.Y();
        double xcencir = _circle.Location().X();
        double ycencir = _circle.Location().Y();
        double r1 = _circle.Radius();
        double dist = _point.Distance(_circle.Location());

        if (dist < _tolerance)
        {
            Circle2D biscirpnt1 = new(new Axis2D(_point, new Direction2D(Direction2D.D.X)), r1 / 2.0);
            bissol = new BCirc(biscirpnt1);
        }
        else
        {
            Point2D center = new((xpoint + xcencir) / 2.0, (ypoint + ycencir) / 2.0);
            Axis2D majax = new(center, new Direction2D(xpoint - xcencir, ypoint - ycencir));

            //=========================================================================
            //   The point is inside the circle.                                +
            //=========================================================================

            if (_theposition == -1)
            {
                Ellipse2D biscirpnt = new(majax, r1 / 2.0, Math.Sqrt(r1 * r1 - dist * dist) / 2.0);
                bissol = new BElips(biscirpnt);
            }

            //=========================================================================
            //   The point is on the circle.                                          +
            //   There is only one solution : straight line passing through point and the center +
            //   of the circle.                                                           +
            //=========================================================================

            else if (_theposition == 0)
            {
                Direction2D dirsol;
                if (_circle.IsDirect)
                {
                    dirsol = new Direction2D(xcencir - xpoint, ycencir - ypoint);
                }
                else
                {
                    dirsol = new Direction2D(xpoint - xcencir, ypoint - ycencir);
                }
                Line2D biscirpnt = new(_point, dirsol);
                bissol = new BLine(biscirpnt);
            }

            //=========================================================================
            //   The point is outside of the circle.                                +
            //   There are two solutions : two main branches of the hyperbola.+
            //=========================================================================

            else
            {
                //	   double d1 = sqrt(dist*R1-R1*R1);
                double d1 = Math.Sqrt(dist * dist - r1 * r1) / 2.0;
                double d2 = r1 / 2.0;
                if (index == 1)
                {
                    Hyperbola2D biscirpnt1 = new(majax, d2, d1);
                    bissol = new BHyper(biscirpnt1);
                }
                else
                {
                    Hyperbola2D biscirpnt1 = new(majax, d2, d1);
                    Hyperbola2D biscirpnt2 = biscirpnt1.OtherBranch();
                    bissol = new BHyper(biscirpnt2);
                }
            }
        }
        return bissol;
    }

    /// <summary>True: this construction never fails.</summary>
    public bool IsDone
    {
        get
        {
            return _wellDone;
        }
    }

    /// <summary>The number of bisecting curves.</summary>
    public int SolutionCount
    {
        get
        {
            return _nbrSol;
        }
    }
}
