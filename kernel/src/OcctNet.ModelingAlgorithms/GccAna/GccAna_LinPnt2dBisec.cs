// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_LinPnt2dBisec.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_LinPnt2dBisec.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The bisecting curve of a 2D line and a point: the locus of the points equidistant from both, a parabola, or a line
/// when the point is on the line. There is always one solution.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117).</remarks>
public sealed class GccAna_LinPnt2dBisec
{
    private bool WellDone;
    private GccInt_Bisec? bissol;

    /// <summary>Constructs the bisecting curve of Line1 and Point2.</summary>
    public GccAna_LinPnt2dBisec(in gp_Lin2d Line1, in gp_Pnt2d Point2)
    {
        WellDone = false;

        double xdir = Line1.Direction().X();
        double ydir = Line1.Direction().Y();
        double xloc = Line1.Location().X();
        double yloc = Line1.Location().Y();
        double dist = Line1.Distance(Point2);

        //   if ( dist > gp::Resolution()) {
        if (dist > 1.0e-10)
        {
            double xpoint2 = Point2.X();
            double ypoint2 = Point2.Y();
            if ((-ydir * (xpoint2 - xloc) + xdir * (ypoint2 - yloc)) > 0.0)
            {
                gp_Ax2d axeparab = new(new gp_Pnt2d(Point2.XY() - dist / 2.0 * new gp_XY(-ydir, xdir)),
                                       new gp_Dir2d(-ydir, xdir));
                gp_Parab2d bislinpnt = new(axeparab, dist / 2.0);
                bissol = new GccInt_BParab(bislinpnt);
            }
            else
            {
                gp_Ax2d axeparab = new(new gp_Pnt2d(Point2.XY() + dist / 2.0 * new gp_XY(-ydir, xdir)),
                                       new gp_Dir2d(ydir, -xdir));
                gp_Parab2d bislinpnt = new(axeparab, dist / 2.0);
                bissol = new GccInt_BParab(bislinpnt);
            }
            WellDone = true;
        }
        else
        {
            gp_Lin2d bislinpnt = new(Point2, new gp_Dir2d(-ydir, xdir));
            bissol = new GccInt_BLine(bislinpnt);
            WellDone = true;
        }
    }

    /// <summary>True if the construction succeeded.</summary>
    public bool IsDone()
    {
        return WellDone;
    }

    /// <summary>The bisecting curve.</summary>
    public GccInt_Bisec ThisSolution()
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        return bissol!;
    }
}
