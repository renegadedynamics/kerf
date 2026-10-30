// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_CircPnt2dBisec.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_CircPnt2dBisec.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The bisecting curves of a 2D circle and a point: the loci of the points equidistant from both. An ellipse when the
/// point is inside the circle, a line when it is on it, the two branches of a hyperbola when it is outside, and a
/// circle when it is the centre.
/// </summary>
/// <remarks>
/// An algorithm object computed by its constructor (decision 117); as in OCCT, each solution is built when it is
/// asked for.
/// </remarks>
public sealed class GccAna_CircPnt2dBisec
{
    private bool WellDone;
    private int NbrSol;
    private gp_Circ2d circle;
    private gp_Pnt2d point;
    private int theposition;
    private double myTolerance;

    /// <summary>Constructs the bisecting curves of Circle and Point.</summary>
    public GccAna_CircPnt2dBisec(in gp_Circ2d Circle, in gp_Pnt2d Point)
    {
        circle = Circle;
        point = Point;
        myTolerance = 1.0e-10;
        DefineSolutions();
    }

    /// <summary>Constructs the bisecting curves of Circle and Point, with a tolerance (at least 1e-10).</summary>
    public GccAna_CircPnt2dBisec(in gp_Circ2d Circle, in gp_Pnt2d Point, double Tolerance)
    {
        circle = Circle;
        point = Point;
        myTolerance = 1.0e-10;
        if (myTolerance < Tolerance)
        {
            myTolerance = Tolerance;
        }

        DefineSolutions();
    }

    // Defines the number and the type of solutions depending on input data
    private void DefineSolutions()
    {
        double dist = circle.Radius() - point.Distance(circle.Location());

        if (Math.Abs(dist) < myTolerance)
        {
            theposition = 0;
            NbrSol = 1;
        }
        else if (dist > 0.0)
        {
            theposition = -1;
            NbrSol = 1;
        }
        else
        {
            theposition = 1;
            NbrSol = 2;
        }

        WellDone = true;
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
        double xpoint = point.X();
        double ypoint = point.Y();
        double xcencir = circle.Location().X();
        double ycencir = circle.Location().Y();
        double R1 = circle.Radius();
        double dist = point.Distance(circle.Location());

        if (dist < myTolerance)
        {
            gp_Circ2d biscirpnt1 = new(new gp_Ax2d(point, new gp_Dir2d(gp_Dir2d.D.X)), R1 / 2.0);
            bissol = new GccInt_BCirc(biscirpnt1);
        }
        else
        {
            gp_Pnt2d center = new((xpoint + xcencir) / 2.0, (ypoint + ycencir) / 2.0);
            gp_Ax2d majax = new(center, new gp_Dir2d(xpoint - xcencir, ypoint - ycencir));

            //=========================================================================
            //   The point is inside the circle.                                +
            //=========================================================================

            if (theposition == -1)
            {
                gp_Elips2d biscirpnt = new(majax, R1 / 2.0, Math.Sqrt(R1 * R1 - dist * dist) / 2.0);
                bissol = new GccInt_BElips(biscirpnt);
            }

            //=========================================================================
            //   The point is on the circle.                                          +
            //   There is only one solution : straight line passing through point and the center +
            //   of the circle.                                                           +
            //=========================================================================

            else if (theposition == 0)
            {
                gp_Dir2d dirsol;
                if (circle.IsDirect())
                {
                    dirsol = new gp_Dir2d(xcencir - xpoint, ycencir - ypoint);
                }
                else
                {
                    dirsol = new gp_Dir2d(xpoint - xcencir, ypoint - ycencir);
                }
                gp_Lin2d biscirpnt = new(point, dirsol);
                bissol = new GccInt_BLine(biscirpnt);
            }

            //=========================================================================
            //   The point is outside of the circle.                                +
            //   There are two solutions : two main branches of the hyperbola.+
            //=========================================================================

            else
            {
                //	   double d1 = sqrt(dist*R1-R1*R1);
                double d1 = Math.Sqrt(dist * dist - R1 * R1) / 2.0;
                double d2 = R1 / 2.0;
                if (Index == 1)
                {
                    gp_Hypr2d biscirpnt1 = new(majax, d2, d1);
                    bissol = new GccInt_BHyper(biscirpnt1);
                }
                else
                {
                    gp_Hypr2d biscirpnt1 = new(majax, d2, d1);
                    gp_Hypr2d biscirpnt2 = biscirpnt1.OtherBranch();
                    bissol = new GccInt_BHyper(biscirpnt2);
                }
            }
        }
        return bissol;
    }

    /// <summary>True: this construction never fails.</summary>
    public bool IsDone()
    {
        return WellDone;
    }

    /// <summary>The number of bisecting curves.</summary>
    public int NbSolutions()
    {
        return NbrSol;
    }
}
