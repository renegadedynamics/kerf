// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_CircLin2dBisec.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_CircLin2dBisec.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry;

using Kerf.Geometry;

namespace Kerf.Engine.Construction.Constraints.Analytic;

/// <summary>
/// The bisecting curves of a 2D circle and a line: the loci of the points equidistant from both, two parabolas (one
/// of them a line when the line is tangent to the circle).
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117); as in OCCT, each solution is built when it is
/// asked for.
/// </remarks>
internal sealed class CircLin2dBisec
{
    private bool _wellDone;
    private int _nbrSol;
    private Circle2D _circle;
    private Line2D _line;

    /// <summary>Constructs the bisecting curves of Circle and Line.</summary>
    public CircLin2dBisec(in Circle2D circle, in Line2D line)
    {
        _circle = circle;
        _line = line;

        //=========================================================================
        //  Initialization of fields :                                            +
        //            - circle                                                    +
        //            - line     (straight line.)                                 +
        //            - NbrSol   (number of solution.)                            +
        //            - WellDone (Boolean showing success or failure of algorithm)+
        //=========================================================================

        _nbrSol = 2;
        _wellDone = true;
    }

    //=========================================================================
    //  Processing.                                                           +
    //  Return coordinates of origins of the straight line (xloc,yloc) and    +
    //  the circle (xcencir, ycencir).                                        +
    //  Also return the coordinates of the direction of the straight line     +
    //  (xdir, ydir) and the radius of circle R1.                             +
    //  Check at which side of the straight line is found the center of the   +
    //  circle to orientate the parabola (sign).                              +
    //  Create axis of each parabola (axeparab1, axeparb2), then              +
    //  two parabolas (biscirlin1, biscirlin1).                               +
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
        double xdir = _line.Direction.X;
        double ydir = _line.Direction.Y;
        double xloc = _line.Location.X;
        double yloc = _line.Location.Y;
        double xcencir = _circle.Location.X;
        double ycencir = _circle.Location.Y;
        double r1 = _circle.Radius;
        double dist = _line.DistanceTo(_circle.Location);
        if ((Math.Abs(_line.DistanceTo(_circle.Location) - _circle.Radius) <= Constants.Resolution()) && (index == 1))
        {
            Line2D biscirlin1 = new(_circle.Location, new Direction2D(-ydir, xdir));
            bissol = new BLine(biscirlin1);
        }
        else
        {
            int signe;
            if ((-ydir * (xcencir - xloc) + xdir * (ycencir - yloc)) > 0.0)
            {
                signe = 1;
            }
            else
            {
                signe = -1;
            }
            Axis2D axeparab1;
            //    gp_Ax2d axeparab2;
            Parabola2D biscirlin;
            if (dist != r1)
            {
                if (index == 1)
                {
                    axeparab1 = new Axis2D(new Point2D(new Xy(xcencir + signe * ydir * (dist + r1) / 2,
                                                                   ycencir - signe * xdir * (dist + r1) / 2.0)),
                                            new Direction2D(-signe * ydir, signe * xdir));
                    biscirlin = new Parabola2D(axeparab1, (dist + r1) / 2.0);
                }
                else
                {
                    if (dist < r1)
                    {
                        axeparab1 = new Axis2D(new Point2D(new Xy(xcencir + signe * ydir * (dist - r1) / 2,
                                                                       ycencir - signe * xdir * (dist - r1) / 2.0)),
                                                new Direction2D(signe * ydir, -signe * xdir));
                    }
                    else
                    {
                        axeparab1 = new Axis2D(new Point2D(new Xy(xcencir + signe * ydir * (dist - r1) / 2,
                                                                       ycencir - signe * xdir * (dist - r1) / 2.0)),
                                                new Direction2D(-signe * ydir, signe * xdir));
                    }
                    biscirlin = new Parabola2D(axeparab1, Math.Abs(dist - r1) / 2.0);
                }
                bissol = new BParab(biscirlin);
            }
            else
            {
                axeparab1 = new Axis2D(new Point2D(new Xy(xcencir + signe * ydir * (dist + r1) / 2.0,
                                                               ycencir - signe * xdir * (dist + r1) / 2.0)),
                                        new Direction2D(signe * (-ydir), signe * xdir));
                biscirlin = new Parabola2D(axeparab1, r1);
                bissol = new BParab(biscirlin);
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

    /// <summary>The number of bisecting curves (2).</summary>
    public int SolutionCount
    {
        get
        {
            return _nbrSol;
        }
    }
}
