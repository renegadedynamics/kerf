// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d2TanOnGeo.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d2TanOnGeo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;

namespace Kerf.Engine.Construction.Constraints;

//=========================================================================
//   Creation d un cercle tangent a deux elements : Droite.               +
//                                                  Cercle.               +
//                                                  Point.                +
//                                                  Courbes.              +
//                        centre sur un troisieme : Droite.               +
//                                                  Cercle.               +
//                                                  Courbes.              +
//=========================================================================

/// <summary>
/// Circles tangent to two arguments (qualified circles or lines, or points) whose centres lie on a curve: the
/// bisectors of the two arguments (GccAna) intersected with the curve (Geom2dInt).
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117), up to 8 solutions.</remarks>
internal sealed class Circ2d2TanOnGeo
{
    private const int ANbSolMAX = 8;

    private bool _wellDone;
    private int _nbrSol;
    private readonly NCollection_Array1<Circle2D> _cirsol = new(1, ANbSolMAX);
    private readonly NCollection_Array1<Qualifier> _qualifier1 = new(1, ANbSolMAX);
    private readonly NCollection_Array1<Qualifier> _qualifier2 = new(1, ANbSolMAX);
    private readonly NCollection_Array1<int> _theSame1 = new(1, ANbSolMAX);
    private readonly NCollection_Array1<int> _theSame2 = new(1, ANbSolMAX);
    private readonly NCollection_Array1<Point2D> _pnttg1sol = new(1, ANbSolMAX);
    private readonly NCollection_Array1<Point2D> _pnttg2sol = new(1, ANbSolMAX);
    private readonly NCollection_Array1<Point2D> _pntcen = new(1, ANbSolMAX);
    private readonly NCollection_Array1<double> _par1sol = new(1, ANbSolMAX);
    private readonly NCollection_Array1<double> _par2sol = new(1, ANbSolMAX);
    private readonly NCollection_Array1<double> _pararg1 = new(1, ANbSolMAX);
    private readonly NCollection_Array1<double> _pararg2 = new(1, ANbSolMAX);
    private readonly NCollection_Array1<double> _parcen3 = new(1, ANbSolMAX);

    /// <summary>
    /// Adds a candidate circle if it touches both arguments as their qualifiers ask and isn't a solution already, with
    /// the qualifiers it has, its tangency points and its centre's parameter on the curve (Kerf decision 1545).
    /// </summary>
    private void AddSolution(in Circle2D sol, in CircleContact a1, in CircleContact a2, double parcen, double tolerance)
    {
        if (!a1.Touches(sol, tolerance, out Qualifier q1) || !a2.Touches(sol, tolerance, out Qualifier q2))
        {
            return;
        }
        for (int i = 1; i <= _nbrSol; i++)
        {
            if (_cirsol[i].Location.DistanceTo(sol.Location) <= tolerance && Math.Abs(_cirsol[i].Radius - sol.Radius) <= tolerance)
            {
                return;
            }
        }
        if (_nbrSol == ANbSolMAX)
        {
            return;
        }
        int n = ++_nbrSol;
        _cirsol[n] = sol;
        _qualifier1[n] = q1;
        _qualifier2[n] = q2;
        _theSame1[n] = a1.Touch(n, sol, q1, tolerance, _pnttg1sol, _par1sol, _pararg1);
        _theSame2[n] = a2.Touch(n, sol, q2, tolerance, _pnttg2sol, _par2sol, _pararg2);
        _pntcen[n] = sol.Location;
        _parcen3[n] = parcen;
        _wellDone = true;
    }

    /// <summary>Circles tangent to two qualified circles, centred on a curve.</summary>
    public Circ2d2TanOnGeo(in QualifiedCirc qualified1, in QualifiedCirc qualified2, CurveAdaptor2D onCurv, double tolerance)
    {
        _wellDone        = false;
        double thefirst = -100000.0;
        double thelast  = 100000.0;
        double firstparam;
        double lastparam;
        double tol = Math.Abs(tolerance);
        _nbrSol     = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                          || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Circle2D          c1 = qualified1.Qualified();
        Circle2D          c2 = qualified2.Qualified();
        double             r1 = c1.Radius;
        double             r2 = c2.Radius;
        Direction2D           dirx = new(Direction2D.D.X);
        Point2D           center1 = c1.Location;
        Point2D           center2 = c2.Location;
        CircleContact a1 = new(qualified1);
        CircleContact a2 = new(qualified2);
        Span<double> radii = stackalloc double[4];
        Circ2dBisec bis = new(c1, c2);
        if (bis.IsDone)
        {
            TheIntConicCurveOfGInter intp = new();
            int                                nbsolution = bis.SolutionCount;
            CurveAdaptor2D   hCu2       = (CurveAdaptor2D)onCurv.ShallowCopy(); // new Geom2dAdaptor_Curve(OnCurv)
            OffsetCurveAdaptor2D              cu2 = new(hCu2, 0.0);
            firstparam = Max(cu2.FirstParameter(), thefirst);
            lastparam  = Min(cu2.LastParameter(), thelast);
            TwoDDomain d2 = new(cu2.Value(firstparam),
                                                  firstparam,
                                                  tol,
                                                  cu2.Value(lastparam),
                                                  lastparam,
                                                  tol);
            double          tol1 = Math.Abs(tolerance);
            double          tol2 = tol1;
            for (int i = 1; i <= nbsolution; i++)
            {
                ConstraintBisec              sol  = bis.ThisSolution(i);
                ConstraintCurveType              type = sol.ArcType();
                switch (type)
                {
                    case ConstraintCurveType.Cir:
                    {
                        Circle2D       circ = sol.Circle();
                        TwoDDomain d1 = new(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, circ),
                                                              0.0,
                                                              tol1,
                                                              global::Kerf.Engine.Geometry.ElCLib.Value(2.0 * M_PI, circ),
                                                              2.0 * M_PI,
                                                              tol2);
                        d1.SetEquivalentParameters(0.0, 2.0 * M_PI);
                        intp.Perform(circ, d1, cu2, d2, tol1, tol2);
                    }
                    break;
                    case ConstraintCurveType.Ell:
                    {
                        Ellipse2D      elips = sol.Ellipse();
                        TwoDDomain d1 = new(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, elips),
                                                              0.0,
                                                              tol1,
                                                              global::Kerf.Engine.Geometry.ElCLib.Value(2.0 * M_PI, elips),
                                                              2.0 * M_PI,
                                                              tol2);
                        d1.SetEquivalentParameters(0.0, 2.0 * M_PI);
                        intp.Perform(elips, d1, cu2, d2, tol1, tol2);
                    }
                    break;
                    case ConstraintCurveType.Hpr:
                    {
                        Hyperbola2D       hypr = sol.Hyperbola();
                        TwoDDomain d1 = new(global::Kerf.Engine.Geometry.ElCLib.Value(-4.0, hypr),
                                                              -4.0,
                                                              tol1,
                                                              global::Kerf.Engine.Geometry.ElCLib.Value(4.0, hypr),
                                                              4.0,
                                                              tol2);
                        intp.Perform(hypr, d1, cu2, d2, tol1, tol2);
                    }
                    break;
                    case ConstraintCurveType.Lin:
                    {
                        Line2D        line = sol.Line();
                        TwoDDomain d1 = new();
                        intp.Perform(line, d1, cu2, d2, tol1, tol2);
                    }
                    break;
                    default:
                    {
                        throw new ConstructionErrorException();
                    }
                }
                if (intp.IsDone)
                {
                    if ((!intp.IsEmpty))
                    {
                        for (int j = 1; j <= intp.PointCount; j++)
                        {
                            Point2D center = intp.Point(j).Value();
                            // kept with the radii that touch both circles as asked, through GccAna's CircleContact (OCCT
                            // wrote up to four radii into two slots, read slots it never set, and placed the tangency
                            // points on the wrong side of an enclosed solution: Kerf decision 1545)
                            int nbr = a1.Radii(center, radii);
                            nbr += a2.Radii(center, radii[nbr..]);
                            for (int k = 0; k < nbr; k++)
                            {
                                if (radii[k] > tol)
                                {
                                    AddSolution(new Circle2D(new Axis2D(center, dirx), radii[k]), a1, a2, intp.Point(j).ParamOnSecond(), tol);
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    //=========================================================================
    //   Creation d un cercle tangent a un Cercle C1 et a une Droite L2.      +
    //                        centre sur une courbe OnCurv.                   +
    //  Nous calculons les bissectrices a C1 et L2 qui nous donnent           +
    //  l ensemble des lieux possibles des centres de tous les cercles        +
    //  tangents a C1 et L2.                                                  +
    //  Nous intersectons ces bissectrices avec la courbe OnCurv ce qui nous  +
    //  donne les points parmis lesquels nous allons choisir les solutions.   +
    //  Les choix s effectuent a partir des Qualifieurs qualifiant C1 et L2.  +
    //=========================================================================
    /// <summary>Circles tangent to a qualified circle and a qualified line, centred on a curve.</summary>
    public Circ2d2TanOnGeo(in QualifiedCirc qualified1, in QualifiedLin qualified2, CurveAdaptor2D onCurv, double tolerance)
    {

        _wellDone        = false;
        double thefirst = -100000.0;
        double thelast  = 100000.0;
        double firstparam;
        double lastparam;
        _nbrSol     = 0;
        double tol = Math.Abs(tolerance);
        double radius;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsOutside || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Direction2D  dirx = new(Direction2D.D.X);
        Circle2D c1 = qualified1.Qualified();
        Line2D  l2 = qualified2.Qualified();
        double    r1 = c1.Radius;
        Point2D  center1 = c1.Location;
        Point2D  origin2 = l2.Location;
        Direction2D  dir2 = l2.Direction;
        Direction2D  normL2 = new(-dir2.Y, dir2.X);

        CircLin2dBisec bis = new(c1, l2);
        if (bis.IsDone)
        {
            double                             tol1 = Math.Abs(tolerance);
            double                             tol2 = tol1;
            TheIntConicCurveOfGInter intp = new();
            int                                nbsolution = bis.SolutionCount;
            CurveAdaptor2D   hCu2       = (CurveAdaptor2D)onCurv.ShallowCopy(); // new Geom2dAdaptor_Curve(OnCurv)
            OffsetCurveAdaptor2D              c2 = new(hCu2, 0.0);
            firstparam = Max(c2.FirstParameter(), thefirst);
            lastparam  = Min(c2.LastParameter(), thelast);
            TwoDDomain d2 = new(c2.Value(firstparam), firstparam, tol, c2.Value(lastparam), lastparam, tol);
            for (int i = 1; i <= nbsolution; i++)
            {
                ConstraintBisec              sol  = bis.ThisSolution(i);
                ConstraintCurveType              type = sol.ArcType();
                switch (type)
                {
                    case ConstraintCurveType.Lin:
                    {
                        Line2D        line = sol.Line();
                        TwoDDomain d1 = new();
                        intp.Perform(line, d1, c2, d2, tol1, tol2);
                    }
                    break;
                    case ConstraintCurveType.Par:
                    {
                        Parabola2D      parab = sol.Parabola();
                        TwoDDomain d1 = new(global::Kerf.Engine.Geometry.ElCLib.Value(-40, parab),
                                                              -40,
                                                              tol1,
                                                              global::Kerf.Engine.Geometry.ElCLib.Value(40, parab),
                                                              40,
                                                              tol1);
                        intp.Perform(parab, d1, c2, d2, tol1, tol2);
                    }
                    break;
                    default:
                    {
                        throw new ConstructionErrorException();
                    }
                }
                if (intp.IsDone)
                {
                    if (!intp.IsEmpty)
                    {
                        for (int j = 1; j <= intp.PointCount && _nbrSol < ANbSolMAX; j++)
                        {
                            Point2D center = intp.Point(j).Value();
                            double   dist1 = center.DistanceTo(center1);
                            //	    int nbsol = 1;
                            bool ok = false;
                            if (qualified1.IsEnclosed)
                            {
                                if (dist1 - r1 < tol)
                                {
                                    ok = true;
                                }
                            }
                            else if (qualified1.IsOutside)
                            {
                                if (r1 - dist1 < tol)
                                {
                                    ok = true;
                                }
                            }
                            else if (qualified1.IsEnclosing || qualified1.IsUnqualified)
                            {
                                ok = true;
                            }
                            radius = l2.DistanceTo(center);
                            if (qualified2.IsEnclosed && ok)
                            {
                                ok = false;
                                if ((((origin2.X - center.X) * (-dir2.Y))
                                          + ((origin2.Y - center.Y) * (dir2.X)))
                                        <= 0)
                                {
                                    ok = true;
                                }
                            }
                            else if (qualified2.IsOutside && ok)
                            {
                                ok = false;
                                if ((((origin2.X - center.X) * (-dir2.Y))
                                          + ((origin2.Y - center.Y) * (dir2.X)))
                                        >= 0)
                                {
                                    ok = true;
                                }
                            }
                            if (qualified1.IsEnclosing && dist1 > radius)
                            {
                                ok = false;
                            }
                            if (ok)
                            {
                                _nbrSol++;
                                _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius);
                                //            =======================================================
                                Direction2D dc2 = new(origin2.XY() - center.XY());
                                double   distcc1 = center.DistanceTo(center1);
                                if (!qualified1.IsUnqualified)
                                {
                                    _qualifier1[_nbrSol] = qualified1.Qualifier();
                                }
                                else if (Math.Abs(distcc1 + radius - r1) < tol)
                                {
                                    _qualifier1[_nbrSol] = Qualifier.Enclosed;
                                }
                                else if (Math.Abs(distcc1 - r1 - radius) < tol)
                                {
                                    _qualifier1[_nbrSol] = Qualifier.Outside;
                                }
                                else
                                {
                                    _qualifier1[_nbrSol] = Qualifier.Enclosing;
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
                                if (dist1 <= tol && Math.Abs(radius - c1.Radius) <= tol)
                                {
                                    _theSame1[_nbrSol] = 1;
                                }
                                else
                                {
                                    _theSame1[_nbrSol] = 0;
                                    Direction2D dc1 = new(center1.XY() - center.XY());
                                    _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius * dc1.XY());
                                    _par1sol[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                                    _pararg1[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
                                }
                                _theSame2[_nbrSol]  = 0;
                                double sign       = dc2.Dot(new Direction2D(-dir2.Y, dir2.X));
                                dc2               = new Direction2D(sign * new Xy(-dir2.Y, dir2.X));
                                _pnttg2sol[_nbrSol] = new Point2D(center.XY() + radius * dc2.XY());
                                _par2sol[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                                _pararg2[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(l2, _pnttg2sol[_nbrSol]);
                                _pntcen[_nbrSol]    = center;
                                _parcen3[_nbrSol]   = intp.Point(j).ParamOnSecond();
                            }
                        }
                    }
                    _wellDone = true;
                }
            }
        }
    }

    //=========================================================================
    //   Creation d un cercle tant a deux Droites L1 et L2.                   +
    //                        centre sur une courbe OnCurv.                   +
    //  Nous calculons les bissectrices a L1 et L2 qui nous donnent           +
    //  l ensemble des lieux possibles des centres de tous les cercles        +
    //  tants a L1 et L2.                                                     +
    //  Nous intersectons ces bissectrices avec la courbe OnCurv ce qui nous  +
    //  donne les points parmis lesquels nous allons choisir les solutions.   +
    //  Les choix s effectuent a partir des Qualifieurs qualifiant L1 et L2.  +
    //=========================================================================
    /// <summary>Circles tangent to two qualified lines, centred on a curve.</summary>
    public Circ2d2TanOnGeo(in QualifiedLin qualified1, in QualifiedLin qualified2, CurveAdaptor2D onCurv, double tolerance)
    {

        _wellDone        = false;
        double thefirst = -100000.0;
        double thelast  = 100000.0;
        double firstparam;
        double lastparam;
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified)
                || !(qualified2.IsEnclosed || qualified2.IsOutside || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        double            tol    = Math.Abs(tolerance);
        double            radius = 0;
        Direction2D          dirx = new(Direction2D.D.X);
        Line2D          l1 = qualified1.Qualified();
        Line2D          l2 = qualified2.Qualified();
        Direction2D          dir1 = l1.Direction;
        Direction2D          dir2 = l2.Direction;
        Direction2D          dnor1 = new(-dir1.Y, dir1.X);
        Direction2D          dnor2 = new(-dir2.Y, dir2.X);
        Point2D          origin1 = l1.Location;
        Point2D          origin2 = l2.Location;
        Lin2dBisec bis = new(l1, l2);
        if (bis.IsDone)
        {
            double                             tol1 = Math.Abs(tolerance);
            double                             tol2 = tol1;
            TheIntConicCurveOfGInter intp = new();
            int                                nbsolution = bis.SolutionCount;
            CurveAdaptor2D   hCu2       = (CurveAdaptor2D)onCurv.ShallowCopy(); // new Geom2dAdaptor_Curve(OnCurv)
            OffsetCurveAdaptor2D              c2 = new(hCu2, 0.0);
            firstparam = Max(c2.FirstParameter(), thefirst);
            lastparam  = Min(c2.LastParameter(), thelast);
            TwoDDomain d2 = new(c2.Value(firstparam), firstparam, tol, c2.Value(lastparam), lastparam, tol);
            TwoDDomain d1 = new();
            for (int i = 1; i <= nbsolution; i++)
            {
                intp.Perform(bis.ThisSolution(i), d1, c2, d2, tol1, tol2);
                if (intp.IsDone)
                {
                    if ((!intp.IsEmpty))
                    {
                        for (int j = 1; j <= intp.PointCount && _nbrSol < ANbSolMAX; j++)
                        {
                            Point2D center = intp.Point(j).Value();
                            double   dist1 = l1.DistanceTo(center);
                            double   dist2 = l2.DistanceTo(center);
                            //	    int nbsol = 1;
                            bool ok = false;
                            if (qualified1.IsEnclosed)
                            {
                                if ((((origin1.X - center.X) * (-dir1.Y))
                                          + ((origin1.Y - center.Y) * (dir1.X)))
                                        <= 0)
                                {
                                    ok = true;
                                }
                            }
                            else if (qualified1.IsOutside)
                            {
                                if ((((origin1.X - center.X) * (-dir1.Y))
                                          + ((origin1.Y - center.Y) * (dir1.X)))
                                        >= 0)
                                {
                                    ok = true;
                                }
                            }
                            else if (qualified1.IsUnqualified)
                            {
                                ok = true;
                            }
                            if (qualified2.IsEnclosed && ok)
                            {
                                ok = false;
                                if ((((origin2.X - center.X) * (-dir2.Y))
                                          + ((origin2.Y - center.Y) * (dir2.X)))
                                        <= 0)
                                {
                                    ok     = true;
                                    radius = (dist1 + dist2) / 2.0;
                                }
                            }
                            else if (qualified2.IsOutside && ok)
                            {
                                ok = false;
                                if ((((origin2.X - center.X) * (-dir2.Y))
                                          + ((origin2.Y - center.Y) * (dir2.X)))
                                        >= 0)
                                {
                                    ok     = true;
                                    radius = (dist1 + dist2) / 2.0;
                                }
                            }
                            else if (qualified2.IsUnqualified && ok)
                            {
                                radius = (dist1 + dist2) / 2.0;
                            }
                            if (ok)
                            {
                                _nbrSol++;
                                _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius);
                                //            =======================================================
                                Direction2D dc1 = new(origin1.XY() - center.XY());
                                Direction2D dc2 = new(origin2.XY() - center.XY());
                                if (!qualified1.IsUnqualified)
                                {
                                    _qualifier1[_nbrSol] = qualified1.Qualifier();
                                }
                                else if (dc1.Dot(dnor1) > 0.0)
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
                                else if (dc2.Dot(dnor2) > 0.0)
                                {
                                    _qualifier2[_nbrSol] = Qualifier.Outside;
                                }
                                else
                                {
                                    _qualifier2[_nbrSol] = Qualifier.Enclosed;
                                }
                                _theSame1[_nbrSol]  = 0;
                                _theSame2[_nbrSol]  = 0;
                                double sign       = dc1.Dot(dnor1);
                                dc1               = new Direction2D(sign * new Xy(-dir1.Y, dir1.X));
                                _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius * dc1.XY());
                                _par1sol[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                                _pararg1[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, _pnttg1sol[_nbrSol]);
                                sign              = dc2.Dot(new Direction2D(-dir2.Y, dir2.X));
                                dc2               = new Direction2D(sign * new Xy(-dir2.Y, dir2.X));
                                _pnttg2sol[_nbrSol] = new Point2D(center.XY() + radius * dc2.XY());
                                _par2sol[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                                _pararg2[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(l2, _pnttg2sol[_nbrSol]);
                                _pntcen[_nbrSol]    = center;
                                _parcen3[_nbrSol]   = intp.Point(j).ParamOnSecond();
                            }
                        }
                    }
                    _wellDone = true;
                }
            }
        }
    }

    //=========================================================================
    //   Creation d un cercle tant a un Cercle C1, passant par un point P2    +
    //                        centre sur une courbe OnCurv.                   +
    //  Nous calculons les bissectrices a C1 et Point2 qui nous donnent       +
    //  l ensemble des lieux possibles des centres de tous les cercles        +
    //  tants a C1 et Point2.                                                 +
    //  Nous intersectons ces bissectrices avec la courbe OnCurv ce qui nous  +
    //  donne les points parmis lesquels nous allons choisir les solutions.   +
    //  Les choix s effectuent a partir des Qualifieurs qualifiant C1.        +
    //=========================================================================
    /// <summary>Circles tangent to a qualified circle and through a point, centred on a curve.</summary>
    public Circ2d2TanOnGeo(in QualifiedCirc qualified1, in Point2D point2, CurveAdaptor2D onCurv, double tolerance)
    {

        _wellDone        = false;
        double thefirst = -100000.0;
        double thelast  = 100000.0;
        double firstparam;
        double lastparam;
        _nbrSol = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
                    || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        double                tol = Math.Abs(tolerance);
        double                radius;
        Direction2D              dirx = new(Direction2D.D.X);
        Circle2D             c1 = qualified1.Qualified();
        double                r1 = c1.Radius;
        Point2D              center1 = c1.Location;
        CircPnt2dBisec bis = new(c1, point2);
        if (bis.IsDone)
        {
            double                             tol1 = Math.Abs(tolerance);
            double                             tol2 = tol1;
            TheIntConicCurveOfGInter intp = new();
            int                                nbsolution = bis.SolutionCount;
            CurveAdaptor2D   hCu2       = (CurveAdaptor2D)onCurv.ShallowCopy(); // new Geom2dAdaptor_Curve(OnCurv)
            OffsetCurveAdaptor2D              c2 = new(hCu2, 0.0);
            firstparam = Max(c2.FirstParameter(), thefirst);
            lastparam  = Min(c2.LastParameter(), thelast);
            TwoDDomain d2 = new(c2.Value(firstparam), firstparam, tol, c2.Value(lastparam), lastparam, tol);
            for (int i = 1; i <= nbsolution; i++)
            {
                ConstraintBisec              sol  = bis.ThisSolution(i);
                ConstraintCurveType              type = sol.ArcType();
                switch (type)
                {
                    case ConstraintCurveType.Cir:
                    {
                        Circle2D       circ = sol.Circle();
                        TwoDDomain d1 = new(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, circ),
                                                              0.0,
                                                              tol1,
                                                              global::Kerf.Engine.Geometry.ElCLib.Value(2.0 * M_PI, circ),
                                                              2.0 * M_PI,
                                                              tol2);
                        d1.SetEquivalentParameters(0.0, 2.0 * M_PI);
                        intp.Perform(circ, d1, c2, d2, tol1, tol2);
                    }
                    break;
                    case ConstraintCurveType.Lin:
                    {
                        Line2D        line = sol.Line();
                        TwoDDomain d1 = new();
                        intp.Perform(line, d1, c2, d2, tol1, tol2);
                    }
                    break;
                    case ConstraintCurveType.Ell:
                    {
                        Ellipse2D      elips = sol.Ellipse();
                        TwoDDomain d1 = new(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, elips),
                                                              0.0,
                                                              tol1,
                                                              global::Kerf.Engine.Geometry.ElCLib.Value(2.0 * M_PI, elips),
                                                              2.0 * M_PI,
                                                              tol2);
                        d1.SetEquivalentParameters(0.0, 2.0 * M_PI);
                        intp.Perform(elips, d1, c2, d2, tol1, tol2);
                    }
                    break;
                    case ConstraintCurveType.Hpr:
                    {
                        Hyperbola2D       hypr = sol.Hyperbola();
                        TwoDDomain d1 = new(global::Kerf.Engine.Geometry.ElCLib.Value(-4.0, hypr),
                                                              -4.0,
                                                              tol1,
                                                              global::Kerf.Engine.Geometry.ElCLib.Value(4.0, hypr),
                                                              4.0,
                                                              tol2);
                        intp.Perform(hypr, d1, c2, d2, tol1, tol2);
                    }
                    break;
                    default:
                    {
                        throw new ConstructionErrorException();
                    }
                }
                if (intp.IsDone)
                {
                    if ((!intp.IsEmpty))
                    {
                        for (int j = 1; j <= intp.PointCount && _nbrSol < ANbSolMAX; j++)
                        {
                            Point2D center = intp.Point(j).Value();
                            radius       = center.DistanceTo(point2);
                            double dist1 = center1.DistanceTo(center);
                            //	    int nbsol = 1;
                            bool ok = false;
                            if (qualified1.IsEnclosed)
                            {
                                if (dist1 - r1 <= tol)
                                {
                                    ok = true;
                                }
                            }
                            else if (qualified1.IsOutside)
                            {
                                if (r1 - dist1 <= tol)
                                {
                                    ok = true;
                                }
                            }
                            else if (qualified1.IsEnclosing)
                            {
                                ok = true;
                            }
                            else if (qualified1.IsUnqualified)
                            {
                                ok = true;
                            }
                            if (ok)
                            {
                                _nbrSol++;
                                _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius);
                                //            =======================================================
                                double distcc1 = center.DistanceTo(center1);
                                if (!qualified1.IsUnqualified)
                                {
                                    _qualifier1[_nbrSol] = qualified1.Qualifier();
                                }
                                else if (Math.Abs(distcc1 + radius - r1) < tol)
                                {
                                    _qualifier1[_nbrSol] = Qualifier.Enclosed;
                                }
                                else if (Math.Abs(distcc1 - r1 - radius) < tol)
                                {
                                    _qualifier1[_nbrSol] = Qualifier.Outside;
                                }
                                else
                                {
                                    _qualifier1[_nbrSol] = Qualifier.Enclosing;
                                }
                                _qualifier2[_nbrSol] = Qualifier.Noqualifier;
                                if (dist1 <= tol && Math.Abs(radius - r1) <= tol)
                                {
                                    _theSame1[_nbrSol] = 1;
                                }
                                else
                                {
                                    _theSame1[_nbrSol] = 0;
                                    Direction2D dc1 = new(center1.XY() - center.XY());
                                    _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius * dc1.XY());
                                    _par1sol[_nbrSol]   = 0.0;
                                    _par1sol[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                                    _pararg1[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(c1, _pnttg1sol[_nbrSol]);
                                }
                                _theSame2[_nbrSol]  = 0;
                                _pnttg2sol[_nbrSol] = point2;
                                _pntcen[_nbrSol]    = center;
                                _parcen3[_nbrSol]   = intp.Point(j).ParamOnSecond();
                                _pararg2[_nbrSol]   = 0.0;
                                _par2sol[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                            }
                        }
                    }
                    _wellDone = true;
                }
            }
        }
    }

    //=========================================================================
    //   Creation d un cercle tant a une ligne L1, passant par un point P2    +
    //                        centre sur une courbe OnCurv.                   +
    //  Nous calculons les bissectrices a L1 et Point2 qui nous donnent       +
    //  l ensemble des lieux possibles des centres de tous les cercles        +
    //  tants a L1 et passant par Point2.                                     +
    //  Nous intersectons ces bissectrices avec la courbe OnCurv ce qui nous  +
    //  donne les points parmis lesquels nous allons choisir les solutions.   +
    //  Les choix s effectuent a partir des Qualifieurs qualifiant L1.        +
    //=========================================================================
    /// <summary>Circles tangent to a qualified line and through a point, centred on a curve.</summary>
    public Circ2d2TanOnGeo(in QualifiedLin qualified1, in Point2D point2, CurveAdaptor2D onCurv, double tolerance)
    {

        _wellDone        = false;
        double thefirst = -100000.0;
        double thelast  = 100000.0;
        double firstparam;
        double lastparam;
        double tol = Math.Abs(tolerance);
        _nbrSol     = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Direction2D             dirx = new(Direction2D.D.X);
        Line2D             l1 = qualified1.Qualified();
        Point2D             origin1 = l1.Location;
        Direction2D             dir1 = l1.Direction;
        Direction2D             normal = new(-dir1.Y, dir1.X);
        LinPnt2dBisec bis = new(l1, point2);
        if (bis.IsDone)
        {
            double                             tol1 = Math.Abs(tolerance);
            double                             tol2 = tol1;
            TheIntConicCurveOfGInter intp = new();
            CurveAdaptor2D   hCu2 = (CurveAdaptor2D)onCurv.ShallowCopy(); // new Geom2dAdaptor_Curve(OnCurv)
            OffsetCurveAdaptor2D              c2 = new(hCu2, 0.0);
            firstparam = Max(c2.FirstParameter(), thefirst);
            lastparam  = Min(c2.LastParameter(), thelast);
            TwoDDomain d2 = new(c2.Value(firstparam), firstparam, tol, c2.Value(lastparam), lastparam, tol);
            ConstraintBisec              sol  = bis.ThisSolution();
            ConstraintCurveType              type = sol.ArcType();
            switch (type)
            {
                case ConstraintCurveType.Lin:
                {
                    Line2D        line = sol.Line();
                    TwoDDomain d1 = new();
                    intp.Perform(line, d1, c2, d2, tol1, tol2);
                }
                break;
                case ConstraintCurveType.Par:
                {
                    Parabola2D      parab = sol.Parabola();
                    TwoDDomain d1 = new(global::Kerf.Engine.Geometry.ElCLib.Value(-40, parab),
                                                          -40,
                                                          tol1,
                                                          global::Kerf.Engine.Geometry.ElCLib.Value(40, parab),
                                                          40,
                                                          tol1);
                    intp.Perform(parab, d1, c2, d2, tol1, tol2);
                }
                break;
                default:
                {
                    throw new ConstructionErrorException();
                }
            }
            if (intp.IsDone)
            {
                if ((!intp.IsEmpty))
                {
                    for (int j = 1; j <= intp.PointCount && _nbrSol < ANbSolMAX; j++)
                    {
                        Point2D center = intp.Point(j).Value();
                        double   radius = l1.DistanceTo(center);
                        //	  int nbsol = 1;
                        bool ok = false;
                        if (qualified1.IsEnclosed)
                        {
                            if ((((origin1.X - center.X) * (-dir1.Y))
                                      + ((origin1.Y - center.Y) * (dir1.X)))
                                    <= 0)
                            {
                                ok = true;
                            }
                        }
                        else if (qualified1.IsOutside)
                        {
                            if ((((origin1.X - center.X) * (-dir1.Y))
                                      + ((origin1.Y - center.Y) * (dir1.X)))
                                    >= 0)
                            {
                                ok = true;
                            }
                        }
                        else if (qualified1.IsUnqualified)
                        {
                            ok = true;
                        }
                        if (ok)
                        {
                            _nbrSol++;
                            _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius);
                            //          =======================================================
                            _qualifier2[_nbrSol] = Qualifier.Noqualifier;
                            Direction2D dc2 = new(origin1.XY() - center.XY());
                            if (!qualified1.IsUnqualified)
                            {
                                _qualifier1[_nbrSol] = qualified1.Qualifier();
                            }
                            else if (dc2.Dot(normal) > 0.0)
                            {
                                _qualifier1[_nbrSol] = Qualifier.Outside;
                            }
                            else
                            {
                                _qualifier1[_nbrSol] = Qualifier.Enclosed;
                            }
                            _theSame1[_nbrSol] = 0;
                            _theSame2[_nbrSol] = 0;
                            Direction2D dc1 = new(origin1.XY() - center.XY());
                            double   sign     = dc1.Dot(new Direction2D(-dir1.Y, dir1.X));
                            dc1               = new Direction2D(sign * new Xy(-dir1.Y, dir1.X));
                            _pnttg1sol[_nbrSol] = new Point2D(center.XY() + radius * dc1.XY());
                            _par1sol[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                            _pararg1[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(l1, _pnttg1sol[_nbrSol]);
                            _pnttg2sol[_nbrSol] = point2;
                            _par2sol[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                            _pararg2[_nbrSol]   = 0.0;
                            _pntcen[_nbrSol]    = center;
                            _parcen3[_nbrSol]   = intp.Point(j).ParamOnSecond();
                        }
                    }
                }
                _wellDone = true;
            }
        }
    }

    //=========================================================================
    //   Creation d un cercle passant par deux point Point1 et Point2         +
    //                        centre sur une courbe OnCurv.                   +
    //  Nous calculons les bissectrices a Point1 et Point2 qui nous donnent   +
    //  l ensemble des lieux possibles des centres de tous les cercles        +
    //  passant par Point1 et Point2.                                         +
    //  Nous intersectons ces bissectrices avec la courbe OnCurv ce qui nous  +
    //  donne les points parmis lesquels nous allons choisir les solutions.   +
    //=========================================================================
    /// <summary>Circles through two points, centred on a curve.</summary>
    public Circ2d2TanOnGeo(in Point2D point1, in Point2D point2, CurveAdaptor2D onCurv, double tolerance)
    {

        _wellDone        = false;
        double thefirst = -100000.0;
        double thelast  = 100000.0;
        double firstparam;
        double lastparam;
        double tol = Math.Abs(tolerance);
        _nbrSol     = 0;
        Direction2D          dirx = new(Direction2D.D.X);
        Pnt2dBisec bis = new(point1, point2);
        if (bis.IsDone)
        {
            double                             tol1 = Math.Abs(tolerance);
            double                             tol2 = tol1;
            TheIntConicCurveOfGInter intp = new();
            CurveAdaptor2D   hCu2 = (CurveAdaptor2D)onCurv.ShallowCopy(); // new Geom2dAdaptor_Curve(OnCurv)
            OffsetCurveAdaptor2D              cu2 = new(hCu2, 0.0);
            firstparam = Max(cu2.FirstParameter(), thefirst);
            lastparam  = Min(cu2.LastParameter(), thelast);
            TwoDDomain d2 = new(cu2.Value(firstparam),
                                                  firstparam,
                                                  tol,
                                                  cu2.Value(lastparam),
                                                  lastparam,
                                                  tol);
            TwoDDomain d1 = new();
            if (bis.HasSolution)
            {
                intp.Perform(bis.ThisSolution(), d1, cu2, d2, tol1, tol2);
                if (intp.IsDone)
                {
                    if ((!intp.IsEmpty))
                    {
                        for (int j = 1; j <= intp.PointCount && _nbrSol < ANbSolMAX; j++)
                        {
                            Point2D center = intp.Point(j).Value();
                            double   radius = point2.DistanceTo(center);
                            _nbrSol++;
                            _cirsol[_nbrSol] = new Circle2D(new Axis2D(center, dirx), radius);
                            //           =======================================================
                            _qualifier1[_nbrSol] = Qualifier.Noqualifier;
                            _qualifier2[_nbrSol] = Qualifier.Noqualifier;
                            _theSame1[_nbrSol]   = 0;
                            _theSame2[_nbrSol]   = 0;
                            _pntcen[_nbrSol]     = center;
                            _pnttg1sol[_nbrSol]  = point1;
                            _pnttg2sol[_nbrSol]  = point2;
                            _pararg1[_nbrSol]    = 0.0;
                            _pararg2[_nbrSol]    = 0.0;
                            _par1sol[_nbrSol]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                            _par2sol[_nbrSol]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                            _parcen3[_nbrSol]    = intp.Point(j).ParamOnSecond();
                        }
                    }
                    _wellDone = true;
                }
            }
        }
    }

    /// <summary>True if the construction succeeded (possibly with no solution).</summary>
    public bool IsDone
    {
        get
        {
            return _wellDone;
        }
    }

    /// <summary>The number of solutions.</summary>
    public int SolutionCount
    {
        get
        {
            return _nbrSol;
        }
    }

    /// <summary>Solution Index.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbSolutions()].</exception>
    public Circle2D ThisSolution(int index)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }

        return _cirsol[index];
    }

    /// <summary>The qualifiers of solution Index relative to the two arguments.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbSolutions()].</exception>
    public void WhichQualifier(int index, out Qualifier qualif1, out Qualifier qualif2)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            qualif1 = _qualifier1[index];
            qualif2 = _qualifier2[index];
        }
    }

    /// <summary>The tangency point of solution Index on the first argument, with its parameters on both.</summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is the first argument.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbSolutions()].</exception>
    public void Tangency1(int index, out double parSol, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            if (_theSame1[index] == 0)
            {
                parSol = _par1sol[index];
                parArg = _pararg1[index];
                pntSol = _pnttg1sol[index];
            }
            else
            {
                throw new NotDoneException();
            }
        }
    }

    /// <summary>The tangency point of solution Index on the second argument, with its parameters on both.</summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is the second argument.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbSolutions()].</exception>
    public void Tangency2(int index, out double parSol, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            if (_theSame2[index] == 0)
            {
                parSol = _par2sol[index];
                parArg = _pararg2[index];
                pntSol = _pnttg2sol[index];
            }
            else
            {
                throw new NotDoneException();
            }
        }
    }

    /// <summary>The centre of solution Index and its parameter on the centre's curve.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbSolutions()].</exception>
    public void CenterOn3(int index, out double parArg, out Point2D pntSol)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        else if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }
        else
        {
            parArg = _parcen3[index];
            pntSol = _pntcen[index];
        }
    }

    /// <summary>True if solution Index is the first argument (a circle).</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbSolutions()].</exception>
    public bool IsTheSame1(int index)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }

        if (_theSame1[index] == 0)
        {
            return false;
        }

        return true;
    }

    /// <summary>True if solution Index is the second argument (a circle).</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbSolutions()].</exception>
    public bool IsTheSame2(int index)
    {
        if (!_wellDone)
        {
            throw new NotDoneException();
        }
        if (index <= 0 || index > _nbrSol)
        {
            throw new OutOfRangeException();
        }

        if (_theSame2[index] == 0)
        {
            return false;
        }

        return true;
    }
}
