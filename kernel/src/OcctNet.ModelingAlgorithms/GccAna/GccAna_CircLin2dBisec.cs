// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_CircLin2dBisec.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_CircLin2dBisec.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The bisecting curves of a 2D circle and a line: the loci of the points equidistant from both, two parabolas (one
/// of them a line when the line is tangent to the circle).
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117); as in OCCT, each solution is built when it is
/// asked for.
/// </remarks>
public sealed class GccAna_CircLin2dBisec
{
    private bool WellDone;
    private int NbrSol;
    private gp_Circ2d circle;
    private gp_Lin2d line;

    /// <summary>Constructs the bisecting curves of Circle and Line.</summary>
    public GccAna_CircLin2dBisec(in gp_Circ2d Circle, in gp_Lin2d Line)
    {
        circle = Circle;
        line = Line;

        //=========================================================================
        //  Initialization of fields :                                            +
        //            - circle                                                    +
        //            - line     (straight line.)                                 +
        //            - NbrSol   (number of solution.)                            +
        //            - WellDone (Boolean showing success or failure of algorithm)+
        //=========================================================================

        NbrSol = 2;
        WellDone = true;
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
    public GccInt_Bisec ThisSolution(int Index)
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }

        if ((Index <= 0) || (Index > NbrSol))
        {
            throw new Standard_OutOfRange();
        }

        GccInt_Bisec bissol;
        double xdir = line.Direction().X();
        double ydir = line.Direction().Y();
        double xloc = line.Location().X();
        double yloc = line.Location().Y();
        double xcencir = circle.Location().X();
        double ycencir = circle.Location().Y();
        double R1 = circle.Radius();
        double dist = line.Distance(circle.Location());
        if ((Math.Abs(line.Distance(circle.Location()) - circle.Radius()) <= gp.Resolution()) && (Index == 1))
        {
            gp_Lin2d biscirlin1 = new(circle.Location(), new gp_Dir2d(-ydir, xdir));
            bissol = new GccInt_BLine(biscirlin1);
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
            gp_Ax2d axeparab1;
            //    gp_Ax2d axeparab2;
            gp_Parab2d biscirlin;
            if (dist != R1)
            {
                if (Index == 1)
                {
                    axeparab1 = new gp_Ax2d(new gp_Pnt2d(new gp_XY(xcencir + signe * ydir * (dist + R1) / 2,
                                                                   ycencir - signe * xdir * (dist + R1) / 2.0)),
                                            new gp_Dir2d(-signe * ydir, signe * xdir));
                    biscirlin = new gp_Parab2d(axeparab1, (dist + R1) / 2.0);
                }
                else
                {
                    if (dist < R1)
                    {
                        axeparab1 = new gp_Ax2d(new gp_Pnt2d(new gp_XY(xcencir + signe * ydir * (dist - R1) / 2,
                                                                       ycencir - signe * xdir * (dist - R1) / 2.0)),
                                                new gp_Dir2d(signe * ydir, -signe * xdir));
                    }
                    else
                    {
                        axeparab1 = new gp_Ax2d(new gp_Pnt2d(new gp_XY(xcencir + signe * ydir * (dist - R1) / 2,
                                                                       ycencir - signe * xdir * (dist - R1) / 2.0)),
                                                new gp_Dir2d(-signe * ydir, signe * xdir));
                    }
                    biscirlin = new gp_Parab2d(axeparab1, Math.Abs(dist - R1) / 2.0);
                }
                bissol = new GccInt_BParab(biscirlin);
            }
            else
            {
                axeparab1 = new gp_Ax2d(new gp_Pnt2d(new gp_XY(xcencir + signe * ydir * (dist + R1) / 2.0,
                                                               ycencir - signe * xdir * (dist + R1) / 2.0)),
                                        new gp_Dir2d(signe * (-ydir), signe * xdir));
                biscirlin = new gp_Parab2d(axeparab1, R1);
                bissol = new GccInt_BParab(biscirlin);
            }
        }

        return bissol;
    }

    /// <summary>True: this construction never fails.</summary>
    public bool IsDone()
    {
        return WellDone;
    }

    /// <summary>The number of bisecting curves (2).</summary>
    public int NbSolutions()
    {
        return NbrSol;
    }
}
