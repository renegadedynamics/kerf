// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccAna/GccAna_Circ2d2TanRad_4.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints.Analytic;

internal sealed partial class AnalyticCirc2d2TanRad
{
    // circular tangent to two lines of given radius
    //===============================================
    //========================================================================
    // Initialize WellDone to false.                                       +
    // Return two lines L1 and L2.                                 +
    // Leave with error if the construction is impossible.     +
    // Create parallel lines to L1 and L2 in the proper direction.                   +
    // Intersect parallels ==> The center point of the solution.   +
    // Create the solution to be added to the already found solutions.         +
    // Fill the fields.                                                +
    //========================================================================
    /// <summary>Circles of radius Radius tangent to two qualified lines. The tolerance is not used.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public AnalyticCirc2d2TanRad(in QualifiedLin qualified1, in QualifiedLin qualified2, double radius, double tolerance)
    {
        _qualifier1 = new(1, 4);
        _qualifier2 = new(1, 4);
        _theSame1 = new(1, 4);
        _theSame2 = new(1, 4);
        _cirsol = new(1, 4);
        _pnttg1sol = new(1, 4);
        _pnttg2sol = new(1, 4);
        _par1sol = new(1, 4);
        _par2sol = new(1, 4);
        _pararg1 = new(1, 4);
        _pararg2 = new(1, 4);


        Direction2D dirx = new(Direction2D.D.X);
        NCollection_Array1<double> cote1 = new(1, 2);
        NCollection_Array1<double> cote2 = new(1, 2);
        int                        nbrcote1 = 0;
        int                        nbrcote2 = 0;
        _nbrSol                              = 0;
        _wellDone                            = false;
        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsOutside || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Line2D l1     = qualified1.Qualified();
        Line2D l2     = qualified2.Qualified();
        double   x1dir  = (l1.Direction).X;
        double   y1dir  = (l1.Direction).Y;
        double   lx1loc = (l1.Location).X;
        double   ly1loc = (l1.Location).Y;
        double   x2dir  = (l2.Direction).X;
        double   y2dir  = (l2.Direction).Y;
        double   lx2loc = (l2.Location).X;
        double   ly2loc = (l2.Location).Y;
        Point2D origin1 = new(lx1loc, ly1loc);
        Point2D origin2 = new(lx2loc, ly2loc);
        // The lines' normals, to which the qualifiers computed for unqualified lines compare the side of the centre (OCCT
        // took the directions: Kerf decision 1542)
        Direction2D normL1 = new(-y1dir, x1dir);
        Direction2D normL2 = new(-y2dir, x2dir);
        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            if (l1.Direction.IsParallel(l2.Direction, global::Kerf.Engine.Foundation.Precision.Angular()))
            {
                _wellDone = true;
            }
            else
            {
                if (qualified1.IsEnclosed && qualified2.IsEnclosed)
                {
                    //    =======================================================
                    nbrcote1 = 1;
                    nbrcote2 = 1;
                    cote1[1] = 1.0;
                    cote2[1] = 1.0;
                }
                else if (qualified1.IsEnclosed && qualified2.IsOutside)
                {
                    //    ==========================================================
                    nbrcote1 = 1;
                    nbrcote2 = 1;
                    cote1[1] = 1.0;
                    cote2[1] = -1.0;
                }
                else if (qualified1.IsOutside && qualified2.IsEnclosed)
                {
                    //    ===========================================================
                    nbrcote1 = 1;
                    nbrcote2 = 1;
                    cote1[1] = -1.0;
                    cote2[1] = 1.0;
                }
                else if (qualified1.IsOutside && qualified2.IsOutside)
                {
                    //    =========================================================
                    nbrcote1 = 1;
                    nbrcote2 = 1;
                    cote1[1] = -1.0;
                    cote2[1] = -1.0;
                }
                if (qualified1.IsEnclosed && qualified2.IsUnqualified)
                {
                    //    =========================================================
                    nbrcote1 = 1;
                    nbrcote2 = 2;
                    cote1[1] = 1.0;
                    cote2[1] = 1.0;
                    cote2[2] = -1.0;
                }
                if (qualified1.IsUnqualified && qualified2.IsEnclosed)
                {
                    //    =========================================================
                    nbrcote1 = 2;
                    nbrcote2 = 1;
                    cote1[1] = 1.0;
                    cote1[2] = -1.0;
                    cote2[1] = 1.0;
                }
                else if (qualified1.IsOutside && qualified2.IsUnqualified)
                {
                    //    =============================================================
                    nbrcote1 = 1;
                    nbrcote2 = 2;
                    cote1[1] = -1.0;
                    cote2[1] = 1.0;
                    cote2[2] = -1.0;
                }
                if (qualified1.IsUnqualified && qualified2.IsOutside)
                {
                    //    ========================================================
                    nbrcote1 = 2;
                    nbrcote2 = 1;
                    cote1[1] = 1.0;
                    cote1[2] = -1.0;
                    cote2[1] = -1.0;
                }
                else if (qualified1.IsUnqualified && qualified2.IsUnqualified)
                {
                    //    =================================================================
                    nbrcote1 = 2;
                    nbrcote2 = 2;
                    cote1[1] = 1.0;
                    cote1[2] = -1.0;
                    cote2[1] = 1.0;
                    cote2[2] = -1.0;
                }
                for (int jcote1 = 1; jcote1 <= nbrcote1; jcote1++)
                {
                    for (int jcote2 = 1; jcote2 <= nbrcote2; jcote2++)
                    {
                        Line2D linint1 = new(new Point2D(lx1loc - cote1[jcote1] * y1dir * radius,
                                                                            ly1loc + cote1[jcote1] * x1dir * radius),
                                                          l1.Direction);
                        Line2D linint2 = new(new Point2D(lx2loc - cote2[jcote2] * y2dir * radius,
                                                                            ly2loc + cote2[jcote2] * x2dir * radius),
                                                          l2.Direction);
                        AnalyticIntersection intp = new(linint1, linint2);
                        if (intp.IsDone)
                        {
                            if (!intp.IsEmpty)
                            {
                                for (int i = 1; i <= intp.PointCount; i++)
                                {
                                    _nbrSol++;
                                    Point2D center = intp.Point(i).Value();
                                    _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius);
                                    //              =======================================================
                                    Direction2D dc1 = new(origin1.XY() - center.XY());
                                    Direction2D dc2 = new(origin2.XY() - center.XY());
                                    if (!qualified1.IsUnqualified)
                                    {
                                        _qualifier1[_nbrSol] = qualified1.Qualifier();
                                    }
                                    else if (dc1.Dot(normL1) > 0.0)
                                    {
                                        _qualifier1[_nbrSol] = Qualifier.Outside;
                                    }
                                    else
                                    {
                                        _qualifier1[_nbrSol] = Qualifier.Enclosed;
                                    }
                                    if (!qualified2.IsUnqualified)
                                    {
                                        _qualifier2[_nbrSol] = qualified2.Qualifier();
                                    }
                                    else if (dc2.Dot(normL2) > 0.0)
                                    {
                                        _qualifier2[_nbrSol] = Qualifier.Outside;
                                    }
                                    else
                                    {
                                        _qualifier2[_nbrSol] = Qualifier.Enclosed;
                                    }
                                    _theSame1[_nbrSol] = 0;
                                    _theSame2[_nbrSol] = 0;
                                    _pnttg1sol[_nbrSol] =
                                        new Point2D(center.XY() + cote1[jcote1] * radius * new Xy(y1dir, -x1dir));
                                    _pnttg2sol[_nbrSol] =
                                        new Point2D(center.XY() + cote2[jcote2] * radius * new Xy(y2dir, -x2dir));
                                }
                            }
                            _wellDone = true;
                        }
                    }
                }
            }
        }
        for (int i = 1; i <= _nbrSol; i++)
        {
            _par1sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[i], _pnttg1sol[i]);
            _pararg1[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, _pnttg1sol[i]);
            _par2sol[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[i], _pnttg2sol[i]);
            _pararg2[i] = global::Kerf.Engine.Geometry.ElCLib.Parameter(l2, _pnttg2sol[i]);
        }
    }
}
