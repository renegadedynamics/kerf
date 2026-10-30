// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Pnt2dBisec.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Pnt2dBisec.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The bisecting line of two 2D points: the perpendicular bisector of the segment joining them. There is no solution
/// if the points coincide.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117).</remarks>
public sealed class GccAna_Pnt2dBisec
{
    private bool WellDone;
    private bool HasSol;
    private gp_Lin2d linsol = new();

    /// <summary>Constructs the bisecting line of Point1 and Point2.</summary>
    public GccAna_Pnt2dBisec(in gp_Pnt2d Point1, in gp_Pnt2d Point2)
    {
        WellDone = false;
        //   if (Point1.Distance(Point2) > gp::Resolution()) {
        if (Point1.Distance(Point2) > 1.0e-10)
        {
            gp_Dir2d dir1 = new(Point2.XY() - Point1.XY());
            linsol = new gp_Lin2d(new gp_Pnt2d((Point2.X() + Point1.X()) / 2.0, (Point2.Y() + Point1.Y()) / 2.0),
                                  new gp_Dir2d(-dir1.Y(), dir1.X()));
            HasSol = true;
            WellDone = true;
        }
        else
        {
            HasSol = false;
            WellDone = true;
        }
    }

    /// <summary>True: this construction never fails.</summary>
    public bool IsDone()
    {
        return WellDone;
    }

    /// <summary>True if the points are not coincident.</summary>
    public bool HasSolution()
    {
        return HasSol;
    }

    /// <summary>The bisecting line.</summary>
    /// <exception cref="GccAna_NoSolution">The points coincide.</exception>
    public gp_Lin2d ThisSolution()
    {
        if (!WellDone)
        {
            throw new StdFail_NotDone();
        }
        else if (!HasSol)
        {
            throw new GccAna_NoSolution();
        }
        return linsol;
    }
}
