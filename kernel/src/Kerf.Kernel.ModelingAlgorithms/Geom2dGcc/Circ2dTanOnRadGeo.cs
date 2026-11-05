// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2dTanOnRadGeo.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2dTanOnRadGeo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Constraints.Analytic;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Constraints.Qualifier;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>
/// 2D circles of a given radius tangent to an argument (a curve, a circle, a line or a point) whose centre lies on a
/// line, a circle or a curve: the centres are the intersections of the curve the centre lies on with the argument's
/// offsets at the radius.
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117). Up to 8 solutions.</remarks>
internal sealed class Circ2dTanOnRadGeo
{
    private const int ANbSolMAX = 8;

    private readonly bool _wellDone;
    private int _nbrSol;
    private readonly NCollection_Array1<Circle2D> _cirsol = new(1, ANbSolMAX);
    private readonly NCollection_Array1<Qualifier> _qualifier1 = new(1, ANbSolMAX);
    private readonly NCollection_Array1<int> _theSame1 = new(1, ANbSolMAX);
    private readonly NCollection_Array1<Point2D> _pnttg1sol = new(1, ANbSolMAX);
    private readonly NCollection_Array1<Point2D> _pntcen3 = new(1, ANbSolMAX);
    private readonly NCollection_Array1<double> _par1sol = new(1, ANbSolMAX);
    private readonly NCollection_Array1<double> _pararg1 = new(1, ANbSolMAX);
    private readonly NCollection_Array1<double> _parcen3 = new(1, ANbSolMAX);

    /// <summary>
    /// Keeps the solutions that obey the laws within ten times the tolerance (at least 1e-6): the tangency point on
    /// the solution and on the argument (for a curve, its tangent perpendicular to the radius; for a line or circle, its
    /// qualifier honoured), the centre on the curve at its parameter (Kerf decision 1547: OCCT's intersections of offset
    /// curves also gave circles that miss the argument or the curve, and some on the wrong side of a qualified circle).
    /// </summary>
    private void KeepLawful(CurveAdaptor2D? curve, Line2D? line, Circle2D? circle, Point2D? point, Qualifier qualifier,
                            Func<double, Point2D> onValue, double radius, double tolerance)
    {
        double tol = Math.Max(1.0e-6, 10.0 * Math.Abs(tolerance));
        int kept = 0;
        for (int i = 1; i <= _nbrSol; i++)
        {
            Circle2D sol = _cirsol[i];
            Point2D c = sol.Location;
            Point2D p = _pnttg1sol[i];
            bool ok = Math.Abs(p.DistanceTo(c) - radius) <= tol
                      && global::Kerf.Engine.Geometry.ElCLib.Value(_par1sol[i], sol).DistanceTo(p) <= tol
                      && onValue(_parcen3[i]).DistanceTo(c) <= tol;
            if (ok && curve is not null)
            {
                ConstraintCurveTool.D1(curve, _pararg1[i], out Point2D on, out Vector2D tan);
                ok = on.DistanceTo(p) <= tol && (tan.Magnitude <= 0.0 || Math.Abs(tan.Normalized().Dot(new Vector2D(c, p))) <= tol);
            }
            else if (ok && line is Line2D l)
            {
                ok = l.DistanceTo(p) <= tol && new CircleContact(new QualifiedLin(l, qualifier)).Touches(sol, tol, out _);
            }
            else if (ok && circle is Circle2D ci)
            {
                ok = Math.Abs(ci.Location.DistanceTo(p) - ci.Radius) <= tol
                     && new CircleContact(new QualifiedCirc(ci, qualifier)).Touches(sol, tol, out _);
            }
            else if (ok && point is Point2D pt)
            {
                ok = pt.DistanceTo(p) <= tol;
            }
            if (!ok)
            {
                continue;
            }
            kept++;
            _cirsol[kept] = _cirsol[i];
            _qualifier1[kept] = _qualifier1[i];
            _theSame1[kept] = _theSame1[i];
            _pnttg1sol[kept] = _pnttg1sol[i];
            _pntcen3[kept] = _pntcen3[i];
            _par1sol[kept] = _par1sol[i];
            _pararg1[kept] = _pararg1[i];
            _parcen3[kept] = _parcen3[i];
        }
        _nbrSol = kept;
    }

    //=========================================================================
    //  Cercle tangent  :  a un cercle Qualified1 (C1).                       +
    //         centre   :  sur une droite OnLine.                             +
    //         de rayon :  Radius.                                            +
    //                                                                        +
    //  On initialise le tableau de solutions cirsol ainsi que tous les       +
    //  champs.                                                               +
    //  On elimine en fonction du qualifieur les cas ne presentant pas de     +
    //  solutions.                                                            +
    //  On resoud l equation du second degre indiquant que le point de centre +
    //  recherche (xc,yc) est a une distance Radius du cercle C1 et           +
    //                        sur la droite OnLine.                           +
    //  Les solutions sont representees par les cercles :                     +
    //                   - de centre Pntcen(xc,yc)                            +
    //                   - de rayon Radius.                                   +
    //=========================================================================
    /// <summary>Circles of radius Radius tangent to the qualified curve, centred on the line OnLine.</summary>
    /// <exception cref="BadQualifierException">The qualifier is GccEnt_noqualifier.</exception>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public Circ2dTanOnRadGeo(in QCurve qualified1, in Line2D onLine, double radius, double tolerance)
    {
        //=========================================================================
        // Traitement.                                                            +
        //=========================================================================

        Direction2D dirx     = new(Direction2D.D.X);
        double   tol      = Math.Abs(tolerance);
        double   thefirst = -100000.0;
        double   thelast  = 100000.0;
        double   firstparam;
        double   lastparam;
        _wellDone = false;
        _nbrSol   = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        int                        nbrcote1 = 0;
        NCollection_Array1<double> coef     = new(1, 2);
        CurveAdaptor2D        cu1      = qualified1.Qualified();

        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            if (qualified1.IsEnclosed)
            {
                //    ===========================
                nbrcote1 = 1;
                coef[1]  = radius;
            }
            else if (qualified1.IsOutside)
            {
                //   ===============================
                nbrcote1 = 1;
                coef[1]  = -radius;
            }
            else if (qualified1.IsUnqualified)
            {
                //   ===================================
                nbrcote1 = 2;
                coef[1]  = radius;
                coef[2]  = -radius;
            }
            TwoDDomain                    d1   = new();
            TheIntConicCurveOfGInter intp = new();
            for (int jcote1 = 1; jcote1 <= nbrcote1; jcote1++)
            {
                CurveAdaptor2D hCu1 = (CurveAdaptor2D)cu1.ShallowCopy();
                // Adaptor2d_OffsetCurve C2(HCu1,Coef(jcote1));
                OffsetCurveAdaptor2D c2 = new(hCu1, -coef[jcote1]);
                firstparam               = Max(c2.FirstParameter(), thefirst);
                lastparam                = Min(c2.LastParameter(), thelast);
                TwoDDomain d2 = new(c2.Value(firstparam),
                                         firstparam,
                                         tol,
                                         c2.Value(lastparam),
                                         lastparam,
                                         tol);
                intp.Perform(onLine, d1, c2, d2, tol, tol);
                if (intp.IsDone)
                {
                    if (!intp.IsEmpty)
                    {
                        for (int i = 1; i <= intp.PointCount && _nbrSol < ANbSolMAX; i++)
                        {
                            _nbrSol++;
                            Point2D center = intp.Point(i).Value();
                            _cirsol[_nbrSol]  = new Circle2D(new Axis2D(center, dirx), radius);
                            //           =======================================================
                            _qualifier1[_nbrSol] = qualified1.Qualifier();
                            _theSame1[_nbrSol]   = 0;
                            _pararg1[_nbrSol]    = intp.Point(i).ParamOnSecond();
                            _parcen3[_nbrSol]    = intp.Point(i).ParamOnFirst();
                            _pnttg1sol[_nbrSol] = ConstraintCurveTool.Value(cu1, _pararg1[_nbrSol]);
                            // after the tangency point is set (OCCT read it before: Kerf decision 1547)
                            _par1sol[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                            _pntcen3[_nbrSol]   = center;
                        }
                    }
                    _wellDone = true;
                }
            }
        }
        Line2D onL = onLine;
        KeepLawful(qualified1.Qualified(), null, null, null, Qualifier.Unqualified, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, onL), radius, tolerance);
    }

    /// <summary>Circles of radius Radius tangent to the qualified curve, centred on the circle OnCirc.</summary>
    /// <exception cref="BadQualifierException">The qualifier is GccEnt_noqualifier.</exception>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public Circ2dTanOnRadGeo(in QCurve qualified1, in Circle2D onCirc, double radius, double tolerance)
    {
        //=========================================================================
        // Traitement.                                                            +
        //=========================================================================

        Direction2D dirx     = new(Direction2D.D.X);
        double   thefirst = -100000.0;
        double   thelast  = 100000.0;
        double   firstparam;
        double   lastparam;
        double   tol      = Math.Abs(tolerance);
        int      nbrcote1 = 0;
        _wellDone          = false;
        _nbrSol            = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        NCollection_Array1<double> cote1 = new(1, 2);
        CurveAdaptor2D        cu1   = qualified1.Qualified();

        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            if (qualified1.IsEnclosed)
            {
                //    ===========================
                nbrcote1 = 1;
                cote1[1] = radius;
            }
            else if (qualified1.IsOutside)
            {
                //   ===============================
                nbrcote1 = 1;
                cote1[1] = -radius;
            }
            else if (qualified1.IsUnqualified)
            {
                //   ===================================
                nbrcote1 = 2;
                cote1[1] = radius;
                cote1[2] = -radius;
            }
            TwoDDomain d1 = new(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, onCirc),
                                     0.0,
                                     tol,
                                     global::Kerf.Engine.Geometry.ElCLib.Value(2.0 * M_PI, onCirc),
                                     2.0 * M_PI,
                                     tol);
            d1.SetEquivalentParameters(0.0, 2.0 * M_PI);
            TheIntConicCurveOfGInter intp = new();
            for (int jcote1 = 1; jcote1 <= nbrcote1; jcote1++)
            {
                CurveAdaptor2D hCu1 = (CurveAdaptor2D)cu1.ShallowCopy();
                // Adaptor2d_OffsetCurve C2(HCu1,cote1(jcote1));
                OffsetCurveAdaptor2D c2 = new(hCu1, -cote1[jcote1]);
                firstparam               = Max(c2.FirstParameter(), thefirst);
                lastparam                = Min(c2.LastParameter(), thelast);
                TwoDDomain d2 = new(c2.Value(firstparam),
                                         firstparam,
                                         tol,
                                         c2.Value(lastparam),
                                         lastparam,
                                         tol);
                intp.Perform(onCirc, d1, c2, d2, tol, tol);
                if (intp.IsDone)
                {
                    if (!intp.IsEmpty)
                    {
                        for (int i = 1; i <= intp.PointCount && _nbrSol < ANbSolMAX; i++)
                        {
                            _nbrSol++;
                            Point2D center = intp.Point(i).Value();
                            _cirsol[_nbrSol]  = new Circle2D(new Axis2D(center, dirx), radius);
                            //           =======================================================
                            _qualifier1[_nbrSol] = qualified1.Qualifier();
                            _theSame1[_nbrSol]   = 0;
                            _pararg1[_nbrSol]    = intp.Point(i).ParamOnSecond();
                            _parcen3[_nbrSol]    = intp.Point(i).ParamOnFirst();
                            _pnttg1sol[_nbrSol] = ConstraintCurveTool.Value(cu1, _pararg1[_nbrSol]);
                            // after the tangency point is set (OCCT read it before: Kerf decision 1547)
                            _par1sol[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                            _pntcen3[_nbrSol]   = center;
                        }
                    }
                    _wellDone = true;
                }
            }
        }
        Circle2D onC = onCirc;
        KeepLawful(qualified1.Qualified(), null, null, null, Qualifier.Unqualified, u => global::Kerf.Engine.Geometry.ElCLib.Value(u, onC), radius, tolerance);
    }

    /// <summary>Circles of radius Radius tangent to the qualified circle, centred on the curve OnCurv.</summary>
    /// <exception cref="BadQualifierException">The qualifier is GccEnt_noqualifier.</exception>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public Circ2dTanOnRadGeo(in QualifiedCirc qualified1, CurveAdaptor2D onCurv, double radius,
                                       double tolerance)
    {
        //=========================================================================
        // Traitement.                                                            +
        //=========================================================================

        Direction2D dirx     = new(Direction2D.D.X);
        double   thefirst = -100000.0;
        double   thelast  = 100000.0;
        double   firstparam;
        double   lastparam;
        double   tol      = Math.Abs(tolerance);
        int      nbrcote1 = 0;
        _wellDone          = false;
        _nbrSol            = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        NCollection_Array1<double> cote1   = new(1, 2);
        Circle2D                  c1      = qualified1.Qualified();
        Point2D                   center1 = c1.Location;
        double                     r1      = c1.Radius;

        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            if (qualified1.IsEnclosed)
            {
                //    ===========================
                nbrcote1 = 1;
                cote1[1] = radius;
            }
            else if (qualified1.IsOutside)
            {
                //   ===============================
                nbrcote1 = 1;
                cote1[1] = -radius;
            }
            else if (qualified1.IsUnqualified)
            {
                //   ===================================
                nbrcote1 = 2;
                cote1[1] = radius;
                cote1[2] = -radius;
            }
            TheIntConicCurveOfGInter intp = new();
            for (int jcote1 = 1; jcote1 <= nbrcote1; jcote1++)
            {
                Circle2D       circ = new(c1.XAxis, r1 + cote1[jcote1]);
                TwoDDomain d1   = new(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, circ),
                                           0.0,
                                           tol,
                                           global::Kerf.Engine.Geometry.ElCLib.Value(2.0 * M_PI, circ),
                                           2.0 * M_PI,
                                           tol);
                d1.SetEquivalentParameters(0.0, 2.0 * M_PI);
                firstparam         = Max(ConstraintCurveTool.FirstParameter(onCurv), thefirst);
                lastparam          = Min(ConstraintCurveTool.LastParameter(onCurv), thelast);
                TwoDDomain d2 = new(ConstraintCurveTool.Value(onCurv, firstparam),
                                         firstparam,
                                         tol,
                                         ConstraintCurveTool.Value(onCurv, lastparam),
                                         lastparam,
                                         tol);
                intp.Perform(circ, d1, onCurv, d2, tol, tol);
                if (intp.IsDone)
                {
                    if (!intp.IsEmpty)
                    {
                        for (int i = 1; i <= intp.PointCount && _nbrSol < ANbSolMAX; i++)
                        {
                            _nbrSol++;
                            Point2D center = intp.Point(i).Value();
                            _cirsol[_nbrSol]  = new Circle2D(new Axis2D(center, dirx), radius);
                            //           =======================================================
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
                            _theSame1[_nbrSol] = 0;
                            _pararg1[_nbrSol]  = intp.Point(i).ParamOnFirst();
                            _parcen3[_nbrSol]  = intp.Point(i).ParamOnSecond();
                            _pnttg1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Value(_pararg1[_nbrSol], c1);
                            // after the tangency point is set (OCCT read it before: Kerf decision 1547)
                            _par1sol[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                            _pntcen3[_nbrSol]   = center;
                        }
                    }
                    _wellDone = true;
                }
            }
        }
        KeepLawful(null, null, qualified1.Qualified(), null, qualified1.Qualifier(), u => ConstraintCurveTool.Value(onCurv, u), radius, tolerance);
    }

    /// <summary>Circles of radius Radius tangent to the qualified line, centred on the curve OnCurv.</summary>
    /// <exception cref="BadQualifierException">The line is qualified enclosing or has no qualifier.</exception>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public Circ2dTanOnRadGeo(in QualifiedLin qualified1, CurveAdaptor2D onCurv, double radius,
                                       double tolerance)
    {
        //=========================================================================
        // Traitement.                                                            +
        //=========================================================================

        Direction2D dirx     = new(Direction2D.D.X);
        double   thefirst = -100000.0;
        double   thelast  = 100000.0;
        double   firstparam;
        double   lastparam;
        double   tol      = Math.Abs(tolerance);
        _wellDone          = false;
        _nbrSol            = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        int                        nbrcote1 = 0;
        NCollection_Array1<double> cote1    = new(1, 2);
        Line2D                   l1       = qualified1.Qualified();
        Point2D                   origin1  = l1.Location;
        Direction2D                   dir1     = l1.Direction;
        Direction2D                   norm1    = new(-dir1.Y, dir1.X);

        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            if (qualified1.IsEnclosed)
            {
                //    ===========================
                nbrcote1 = 1;
                cote1[1] = radius;
            }
            else if (qualified1.IsOutside)
            {
                //   ===============================
                nbrcote1 = 1;
                cote1[1] = -radius;
            }
            else if (qualified1.IsUnqualified)
            {
                //   ===================================
                nbrcote1 = 2;
                cote1[1] = radius;
                cote1[2] = -radius;
            }
            TheIntConicCurveOfGInter intp = new();
            for (int jcote1 = 1; jcote1 <= nbrcote1; jcote1++)
            {
                // the line offset from its origin (OCCT offset its direction: Kerf decision 1547); a parameter on it
                // is the foot's on L1
                Point2D        point = new(origin1.XY() + cote1[jcote1] * norm1.XY());
                Line2D        line  = new(point, dir1); // ligne avec deport.
                TwoDDomain d1    = new();
                firstparam            = Max(ConstraintCurveTool.FirstParameter(onCurv), thefirst);
                lastparam             = Min(ConstraintCurveTool.LastParameter(onCurv), thelast);
                TwoDDomain d2    = new(ConstraintCurveTool.Value(onCurv, firstparam),
                                            firstparam,
                                            tol,
                                            ConstraintCurveTool.Value(onCurv, lastparam),
                                            lastparam,
                                            tol);
                intp.Perform(line, d1, onCurv, d2, tol, tol);
                if (intp.IsDone)
                {
                    if (!intp.IsEmpty)
                    {
                        for (int i = 1; i <= intp.PointCount && _nbrSol < ANbSolMAX; i++)
                        {
                            _nbrSol++;
                            Point2D center = intp.Point(i).Value();
                            _cirsol[_nbrSol]  = new Circle2D(new Axis2D(center, dirx), radius);
                            //           =======================================================
                            Direction2D dc1 = new(origin1.XY() - center.XY());
                            if (!qualified1.IsUnqualified)
                            {
                                _qualifier1[_nbrSol] = qualified1.Qualifier();
                            }
                            else if (dc1.Dot(norm1) > 0.0)
                            {
                                _qualifier1[_nbrSol] = Qualifier.Outside;
                            }
                            else
                            {
                                _qualifier1[_nbrSol] = Qualifier.Enclosed;
                            }
                            _theSame1[_nbrSol] = 0;
                            _pararg1[_nbrSol]  = intp.Point(i).ParamOnFirst();
                            _parcen3[_nbrSol]  = intp.Point(i).ParamOnSecond();
                            _pnttg1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Value(_pararg1[_nbrSol], l1);
                            // after the tangency point is set (OCCT read it before: Kerf decision 1547)
                            _par1sol[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                            _pntcen3[_nbrSol]   = center;
                        }
                    }
                    _wellDone = true;
                }
            }
        }
        KeepLawful(null, qualified1.Qualified(), null, null, qualified1.Qualifier(), u => ConstraintCurveTool.Value(onCurv, u), radius, tolerance);
    }

    /// <summary>Circles of radius Radius tangent to the qualified curve, centred on the curve OnCurv.</summary>
    /// <exception cref="BadQualifierException">The qualifier is GccEnt_noqualifier.</exception>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public Circ2dTanOnRadGeo(in QCurve qualified1, CurveAdaptor2D onCurv, double radius,
                                       double tolerance)
    {
        //=========================================================================
        // Traitement.                                                            +
        //=========================================================================

        Direction2D dirx     = new(Direction2D.D.X);
        double   thefirst = -100000.0;
        double   thelast  = 100000.0;
        double   firstparam;
        double   lastparam;
        double   tol      = Math.Abs(tolerance);
        int      nbrcote1 = 0;
        _wellDone          = false;
        _nbrSol            = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        NCollection_Array1<double> cote1 = new(1, 2);
        CurveAdaptor2D        cu1   = qualified1.Qualified();

        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            if (qualified1.IsEnclosed)
            {
                //    ===========================
                nbrcote1 = 1;
                cote1[1] = radius;
            }
            else if (qualified1.IsOutside)
            {
                //   ===============================
                nbrcote1 = 1;
                cote1[1] = -radius;
            }
            else if (qualified1.IsUnqualified)
            {
                //   ===================================
                nbrcote1 = 2;
                cote1[1] = radius;
                cote1[2] = -radius;
            }
            GInter intp = new();
            for (int jcote1 = 1; jcote1 <= nbrcote1; jcote1++)
            {
                CurveAdaptor2D hCu1 = (CurveAdaptor2D)cu1.ShallowCopy();
                // Adaptor2d_OffsetCurve C1(HCu1,cote1(jcote1));
                OffsetCurveAdaptor2D c1 = new(hCu1, -cote1[jcote1]);
                firstparam               = Max(c1.FirstParameter(), thefirst);
                lastparam                = Min(c1.LastParameter(), thelast);
                TwoDDomain d1 = new(c1.Value(firstparam),
                                         firstparam,
                                         tol,
                                         c1.Value(lastparam),
                                         lastparam,
                                         tol);
                CurveAdaptor2D   hOnCurv = (CurveAdaptor2D)onCurv.ShallowCopy();
                // the curve itself, an offset of 0 (OCCT's one-argument constructor left the range [0, 0], a single
                // point: Kerf decision 1547)
                OffsetCurveAdaptor2D c2      = new(hOnCurv, 0.0);
                firstparam                    = Max(c2.FirstParameter(), thefirst);
                lastparam                     = Min(c2.LastParameter(), thelast);
                TwoDDomain d2 = new(c2.Value(firstparam),
                                         firstparam,
                                         tol,
                                         c2.Value(lastparam),
                                         lastparam,
                                         tol);
                intp.Perform(c1, d1, c2, d2, tol, tol);
                if (intp.IsDone)
                {
                    if (!intp.IsEmpty)
                    {
                        for (int i = 1; i <= intp.PointCount && _nbrSol < ANbSolMAX; i++)
                        {
                            _nbrSol++;
                            Point2D center = intp.Point(i).Value();
                            _cirsol[_nbrSol]  = new Circle2D(new Axis2D(center, dirx), radius);
                            //           =======================================================
                            _qualifier1[_nbrSol] = qualified1.Qualifier();
                            _theSame1[_nbrSol]   = 0;
                            _pararg1[_nbrSol]    = intp.Point(i).ParamOnFirst();
                            _parcen3[_nbrSol]    = intp.Point(i).ParamOnSecond();
                            _pnttg1sol[_nbrSol] = ConstraintCurveTool.Value(cu1, _pararg1[_nbrSol]);
                            // after the tangency point is set (OCCT read it before: Kerf decision 1547)
                            _par1sol[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                            _pntcen3[_nbrSol]   = center;
                        }
                    }
                    _wellDone = true;
                }
            }
        }
        KeepLawful(qualified1.Qualified(), null, null, null, Qualifier.Unqualified, u => ConstraintCurveTool.Value(onCurv, u), radius, tolerance);
    }

    /// <summary>Circles of radius Radius through Point1, centred on the curve OnCurv.</summary>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public Circ2dTanOnRadGeo(in Point2D point1, CurveAdaptor2D onCurv, double radius, double tolerance)
    {
        //=========================================================================
        // Traitement.                                                            +
        //=========================================================================

        Direction2D dirx     = new(Direction2D.D.X);
        double   thefirst = -100000.0;
        double   thelast  = 100000.0;
        double   firstparam;
        double   lastparam;
        double   tol      = Math.Abs(tolerance);
        _wellDone          = false;
        _nbrSol            = 0;

        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            //     gp_Dir2d Dir(-y1dir,x1dir);
            Circle2D       circ = new(new Axis2D(point1, new Direction2D(Direction2D.D.X)), radius);
            TwoDDomain d1   = new(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, circ),
                                       0.0,
                                       tol,
                                       global::Kerf.Engine.Geometry.ElCLib.Value(2.0 * M_PI, circ),
                                       2 * M_PI,
                                       tol);
            d1.SetEquivalentParameters(0.0, 2.0 * M_PI);
            firstparam         = Max(ConstraintCurveTool.FirstParameter(onCurv), thefirst);
            lastparam          = Min(ConstraintCurveTool.LastParameter(onCurv), thelast);
            TwoDDomain d2 = new(ConstraintCurveTool.Value(onCurv, firstparam),
                                     firstparam,
                                     tol,
                                     ConstraintCurveTool.Value(onCurv, lastparam),
                                     lastparam,
                                     tol);
            TheIntConicCurveOfGInter intp = new(circ, d1, onCurv, d2, tol, tol);
            if (intp.IsDone)
            {
                if (!intp.IsEmpty)
                {
                    for (int i = 1; i <= intp.PointCount && _nbrSol < ANbSolMAX; i++)
                    {
                        _nbrSol++;
                        Point2D center = intp.Point(i).Value();
                        _cirsol[_nbrSol]  = new Circle2D(new Axis2D(center, dirx), radius);
                        //         =======================================================
                        _qualifier1[_nbrSol] = Qualifier.Noqualifier;
                        _theSame1[_nbrSol]   = 0;
                        _pararg1[_nbrSol]    = intp.Point(i).ParamOnFirst();
                        _parcen3[_nbrSol]    = intp.Point(i).ParamOnSecond();
                        _pnttg1sol[_nbrSol] = point1;
                        // after the tangency point is set (OCCT read it before: Kerf decision 1547)
                        _par1sol[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                        _pntcen3[_nbrSol]   = center;
                    }
                    _wellDone = true;
                }
            }
        }
        KeepLawful(null, null, null, point1, Qualifier.Unqualified, u => ConstraintCurveTool.Value(onCurv, u), radius, tolerance);
    }

    //=========================================================================

    /// <summary>True if the construction succeeded.</summary>
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

    /// <summary>The solution of index Index.</summary>
    /// <exception cref="OutOfRangeException">Index is not in 1..NbSolutions().</exception>
    public Circle2D ThisSolution(int index)
    {
        if (index > _nbrSol || index <= 0)
        {
            throw new OutOfRangeException();
        }
        return _cirsol[index];
    }

    /// <summary>The qualifier of the argument for the solution of index Index.</summary>
    public void WhichQualifier(int index, out Qualifier qualif1)
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
        }
    }

    /// <summary>The tangency point between the solution of index Index and the argument, and its parameters.</summary>
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
            parSol = _par1sol[index];
            parArg = _pararg1[index];
            pntSol = _pnttg1sol[index];
        }
    }

    /// <summary>The parameter of the centre of the solution of index Index on the curve it lies on, and (OCCT) the tangency point.</summary>
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
            pntSol = _pntcen3[index]; // OCCT returned the tangency point (Kerf decision 1547)
        }
    }

    /// <summary>True if the solution of index Index is the argument itself.</summary>
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
}
