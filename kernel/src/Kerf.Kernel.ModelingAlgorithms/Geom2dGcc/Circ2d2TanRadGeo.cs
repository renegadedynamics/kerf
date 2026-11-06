// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d2TanRadGeo.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Geom2dGcc/Geom2dGcc_Circ2d2TanRadGeo.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>
/// Circles of a given radius tangent to two arguments, at least one of them a free-form curve (the other a qualified
/// line, circle or curve, or a point): the centres are the intersections of the arguments' offsets by the radius
/// (Adaptor2d_OffsetCurve and Geom2dInt).
/// </summary>
/// <remarks>An algorithm object computed by its constructor (decision 117), up to 16 solutions.</remarks>
internal sealed class Circ2d2TanRadGeo
{
    private const int ANbSolMAX = 16;

    private bool _wellDone;
    private int _nbrSol;
    private readonly NCollection_Array1<Circle2D> _cirsol = new(1, ANbSolMAX);
    private readonly NCollection_Array1<Qualifier> _qualifier1 = new(1, ANbSolMAX);
    private readonly NCollection_Array1<Qualifier> _qualifier2 = new(1, ANbSolMAX);
    private readonly NCollection_Array1<int> _theSame1 = new(1, ANbSolMAX);
    private readonly NCollection_Array1<int> _theSame2 = new(1, ANbSolMAX);
    private readonly NCollection_Array1<Point2D> _pnttg1sol = new(1, ANbSolMAX);
    private readonly NCollection_Array1<Point2D> _pnttg2sol = new(1, ANbSolMAX);
    private readonly NCollection_Array1<double> _par1sol = new(1, ANbSolMAX);
    private readonly NCollection_Array1<double> _par2sol = new(1, ANbSolMAX);
    private readonly NCollection_Array1<double> _pararg1 = new(1, ANbSolMAX);
    private readonly NCollection_Array1<double> _pararg2 = new(1, ANbSolMAX);

    // circulaire tant a une courbe et une droite ,de rayon donne
    //==============================================================

    //========================================================================
    // On initialise WellDone a false.                                       +
    // On recupere la courbe Cu2 et la droite L1.                            +
    // On sort en erreur dans les cas ou la construction est impossible.     +
    // On fait la parallele a Cu2 dans le bon sens.                          +
    // On fait la parallele a L1 dans le bon sens.                           +
    // On intersecte les paralleles ==> point de centre de la solution.      +
    // On cree la solution qu on ajoute aux solutions deja trouvees.         +
    // On remplit les champs.                                                +
    //========================================================================

    /// <summary>Circles of radius Radius tangent to a qualified line and a qualified curve.</summary>
    /// <exception cref="BadQualifierException">The line is qualified enclosing.</exception>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public Circ2d2TanRadGeo(in QualifiedLin qualified1, in QCurve qualified2, double radius, double tolerance)
    {
        //========================================================================
        // Traitement.                                                           +
        //========================================================================

        double                     tol      = Math.Abs(tolerance);
        double                     thefirst = -100000.0;
        double                     thelast  = 100000.0;
        double                     firstparam;
        double                     lastparam;
        Direction2D                   dirx     = new(Direction2D.D.X);
        NCollection_Array1<double> cote1    = new(1, 2);
        NCollection_Array1<double> cote2    = new(1, 2);
        int                        nbrcote1 = 0;
        int                        nbrcote2 = 0;
        _wellDone                            = false;
        _nbrSol                              = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsOutside || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                 || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Line2D            l1      = qualified1.Qualified();
        double              x1dir   = (l1.Direction).X;
        double              y1dir   = (l1.Direction).Y;
        double              lxloc   = (l1.Location).X;
        double              lyloc   = (l1.Location).Y;
        Point2D            origin1 = new(lxloc, lyloc);
        Direction2D            normL1  = new(-y1dir, x1dir);
        CurveAdaptor2D cu2     = qualified2.Qualified();
        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            if (qualified1.IsEnclosed && qualified2.IsEnclosed)
            {
                //   =======================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = radius;
                cote2[1] = radius;
            }
            else if (qualified1.IsEnclosed && qualified2.IsOutside)
            {
                //   ==========================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = radius;
                cote2[1] = -radius;
            }
            else if (qualified1.IsOutside && qualified2.IsEnclosed)
            {
                //   ===========================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = -radius;
                cote2[1] = radius;
            }
            else if (qualified1.IsOutside && qualified2.IsOutside)
            {
                //   =========================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = -radius;
                cote2[1] = -radius;
            }
            if (qualified1.IsEnclosed && qualified2.IsUnqualified)
            {
                //   =========================================================
                nbrcote1 = 1;
                nbrcote2 = 2;
                cote1[1] = radius;
                cote2[1] = radius;
                cote2[2] = -radius;
            }
            if (qualified1.IsUnqualified && qualified2.IsEnclosed)
            {
                //   =========================================================
                nbrcote1 = 2;
                nbrcote2 = 1;
                cote1[1] = radius;
                cote1[2] = -radius;
                cote2[1] = radius;
            }
            else if (qualified1.IsOutside && qualified2.IsUnqualified)
            {
                //   =============================================================
                nbrcote1 = 1;
                nbrcote2 = 2;
                cote1[1] = -radius;
                cote2[1] = radius;
                cote2[2] = -radius;
            }
            if (qualified1.IsUnqualified && qualified2.IsOutside)
            {
                //   ========================================================
                nbrcote1 = 2;
                nbrcote2 = 1;
                cote1[1] = radius;
                cote1[2] = -radius;
                cote2[1] = -radius;
            }
            else if (qualified1.IsUnqualified && qualified2.IsUnqualified)
            {
                //   =================================================================
                nbrcote1 = 2;
                nbrcote2 = 2;
                cote1[1] = radius;
                cote1[2] = -radius;
                cote2[1] = radius;
                cote2[2] = -radius;
            }
            Direction2D dir = new(-y1dir, x1dir);
            for (int jcote1 = 1; jcote1 <= nbrcote1; jcote1++)
            {
                Point2D        point = new(l1.Location.XY() + cote1[jcote1] * dir.XY());
                Line2D        line  = new(point, l1.Direction); // ligne avec deport.
                TwoDDomain d1    = new();
                for (int jcote2 = 1; jcote2 <= nbrcote2 && _nbrSol < ANbSolMAX; jcote2++)
                {
                    CurveAdaptor2D hCu2 = (CurveAdaptor2D)cu2.ShallowCopy(); // new Geom2dAdaptor_Curve(Cu2)
                    // Adaptor2d_OffsetCurve C2(HCu2,cote2(jcote2));
                    OffsetCurveAdaptor2D c2 = new(hCu2, -cote2[jcote2]);
                    firstparam = Max(c2.FirstParameter(), thefirst);
                    lastparam  = Min(c2.LastParameter(), thelast);
                    TwoDDomain d2 = new(c2.Value(firstparam), firstparam, tol, c2.Value(lastparam), lastparam, tol);
                    TheIntConicCurveOfGInter intp = new(line, d1, c2, d2, tol, tol);
                    if (intp.IsDone)
                    {
                        if (!intp.IsEmpty)
                        {
                            for (int i = 1; i <= intp.PointCount && _nbrSol < ANbSolMAX; i++)
                            {
                                _nbrSol++;
                                Point2D center = intp.Point(i).Value();
                                _cirsol[_nbrSol]  = new Circle2D(new Axis2D(center, dirx), radius);
                                //             =======================================================
                                Direction2D dc1       = new(origin1.XY() - center.XY());
                                _qualifier2[_nbrSol] = qualified2.Qualifier();
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
                                _theSame1[_nbrSol]  = 0;
                                _theSame2[_nbrSol]  = 0;
                                _pararg1[_nbrSol]   = intp.Point(i).ParamOnFirst();
                                _pararg2[_nbrSol]   = intp.Point(i).ParamOnSecond();
                                _pnttg1sol[_nbrSol] = global::Kerf.Engine.Geometry.ElCLib.Value(_pararg1[_nbrSol], l1);
                                _pnttg2sol[_nbrSol] = ConstraintCurveTool.Value(cu2, _pararg2[_nbrSol]);
                                _par1sol[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                                _par2sol[_nbrSol]   = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                            }
                        }
                        _wellDone = true;
                    }
                }
            }
        }
    }

    // circulaire tant a une courbe et un cercle ,de rayon donne
    //=============================================================

    //========================================================================
    // On initialise WellDone a false.                                       +
    // On recupere la courbe Cu2 et le cercle C1.                            +
    // On sort en erreur dans les cas ou la construction est impossible.     +
    // On fait la parallele a Cu2 dans le bon sens.                          +
    // On fait la parallele a C1 dans le bon sens.                           +
    // On intersecte les paralleles ==> point de centre de la solution.      +
    // On cree la solution qu on ajoute aux solutions deja trouvees.         +
    // On remplit les champs.                                                +
    //========================================================================

    /// <summary>Circles of radius Radius tangent to a qualified circle and a qualified curve.</summary>
    /// <exception cref="BadQualifierException">A qualifier isn't one of the four positions.</exception>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public Circ2d2TanRadGeo(in QualifiedCirc qualified1, in QCurve qualified2, double radius, double tolerance)
    {
        //========================================================================
        // Traitement.                                                           +
        //========================================================================

        double                     tol      = Math.Abs(tolerance);
        double                     thefirst = -100000.0;
        double                     thelast  = 100000.0;
        double                     firstparam;
        double                     lastparam;
        Direction2D                   dirx     = new(Direction2D.D.X);
        NCollection_Array1<double> cote1    = new(1, 2);
        NCollection_Array1<double> cote2    = new(1, 2);
        int                        nbrcote1 = 0;
        int                        nbrcote2 = 0;
        _wellDone                            = false;
        _nbrSol                              = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                 || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        Circle2D           c1      = qualified1.Qualified();
        Point2D            center1 = c1.Location;
        CurveAdaptor2D cu2     = qualified2.Qualified();
        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            if (qualified1.IsEnclosed && qualified2.IsEnclosed)
            {
                //   =======================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = radius;
                cote2[1] = radius;
            }
            else if (qualified1.IsEnclosed && qualified2.IsOutside)
            {
                //   ==========================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = radius;
                cote2[1] = -radius;
            }
            else if (qualified1.IsOutside && qualified2.IsEnclosed)
            {
                //   ===========================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = -radius;
                cote2[1] = radius;
            }
            else if (qualified1.IsOutside && qualified2.IsOutside)
            {
                //   =========================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = -radius;
                cote2[1] = -radius;
            }
            if (qualified1.IsEnclosed && qualified2.IsUnqualified)
            {
                //   =========================================================
                nbrcote1 = 1;
                nbrcote2 = 2;
                cote1[1] = radius;
                cote2[1] = radius;
                cote2[2] = -radius;
            }
            if (qualified1.IsUnqualified && qualified2.IsEnclosed)
            {
                //   =========================================================
                nbrcote1 = 2;
                nbrcote2 = 1;
                cote1[1] = radius;
                cote1[2] = -radius;
                cote2[1] = radius;
            }
            else if (qualified1.IsOutside && qualified2.IsUnqualified)
            {
                //   =============================================================
                nbrcote1 = 1;
                nbrcote2 = 2;
                cote1[1] = -radius;
                cote2[1] = radius;
                cote2[2] = -radius;
            }
            if (qualified1.IsUnqualified && qualified2.IsOutside)
            {
                //   ========================================================
                nbrcote1 = 2;
                nbrcote2 = 1;
                cote1[1] = radius;
                cote1[2] = -radius;
                cote2[1] = -radius;
            }
            else if (qualified1.IsUnqualified && qualified2.IsUnqualified)
            {
                //   =================================================================
                nbrcote1 = 2;
                nbrcote2 = 2;
                cote1[1] = radius;
                cote1[2] = -radius;
                cote2[1] = radius;
                cote2[2] = -radius;
            }
            double                             r1   = c1.Radius;
            TheIntConicCurveOfGInter intp = new();
            for (int jcote1 = 1; jcote1 <= nbrcote1 && _nbrSol < ANbSolMAX; jcote1++)
            {
                Circle2D       circ = new(c1.XAxis, r1 + cote1[jcote1]);
                TwoDDomain d1   = new(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, circ), 0.0, tol, global::Kerf.Engine.Geometry.ElCLib.Value(2.0 * M_PI, circ), 2.0 * M_PI, tol);
                d1.SetEquivalentParameters(0.0, 2.0 * M_PI);
                for (int jcote2 = 1; jcote2 <= nbrcote2; jcote2++)
                {
                    CurveAdaptor2D hCu2 = (CurveAdaptor2D)cu2.ShallowCopy(); // new Geom2dAdaptor_Curve(Cu2)
                    // Adaptor2d_OffsetCurve C2(HCu2,cote2(jcote2));
                    OffsetCurveAdaptor2D c2 = new(hCu2, -cote2[jcote2]);
                    firstparam = Max(c2.FirstParameter(), thefirst);
                    lastparam  = Min(c2.LastParameter(), thelast);
                    TwoDDomain d2 = new(c2.Value(firstparam), firstparam, tol, c2.Value(lastparam), lastparam, tol);
                    intp.Perform(circ, d1, c2, d2, tol, tol);
                    if (intp.IsDone)
                    {
                        if (!intp.IsEmpty)
                        {
                            for (int i = 1; i <= intp.PointCount && _nbrSol < ANbSolMAX; i++)
                            {
                                _nbrSol++;
                                Point2D center = intp.Point(i).Value();
                                _cirsol[_nbrSol]  = new Circle2D(new Axis2D(center, dirx), radius);
                                //             =======================================================
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
                                _qualifier2[_nbrSol] = qualified2.Qualifier();
                                _theSame1[_nbrSol]   = 0;
                                _theSame2[_nbrSol]   = 0;
                                _pararg1[_nbrSol]    = intp.Point(i).ParamOnFirst();
                                _pararg2[_nbrSol]    = intp.Point(i).ParamOnSecond();
                                _pnttg1sol[_nbrSol]  = global::Kerf.Engine.Geometry.ElCLib.Value(_pararg1[_nbrSol], c1);
                                _pnttg2sol[_nbrSol]  = ConstraintCurveTool.Value(cu2, _pararg2[_nbrSol]);
                                _par1sol[_nbrSol]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                                _par2sol[_nbrSol]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                            }
                        }
                        _wellDone = true;
                    }
                }
            }
        }
    }

    // circulaire tant a une courbe et un point ,de rayon donne
    //============================================================

    //========================================================================
    // On initialise WellDone a false.                                       +
    // On recupere la courbe Cu1 et le point P2.                             +
    // On sort en erreur dans les cas ou la construction est impossible.     +
    // On fait la parallele a Cu1 dans le bon sens.                          +
    // On fait la parallele a P2 dans le bon sens.                           +
    // On intersecte les paralleles ==> point de centre de la solution.      +
    // On cree la solution qu on ajoute aux solutions deja trouvees.         +
    // On remplit les champs.                                                +
    //========================================================================

    /// <summary>Circles of radius Radius tangent to a qualified curve and passing through a point.</summary>
    /// <exception cref="BadQualifierException">The qualifier isn't one of the four positions.</exception>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public Circ2d2TanRadGeo(in QCurve qualified1, in Point2D point2, double radius, double tolerance)
    {
        //========================================================================
        // Traitement.                                                           +
        //========================================================================

        double                     tol      = Math.Abs(tolerance);
        double                     thefirst = -100000.0;
        double                     thelast  = 100000.0;
        double                     firstparam;
        double                     lastparam;
        Direction2D                   dirx     = new(Direction2D.D.X);
        NCollection_Array1<double> cote1    = new(1, 2);
        int                        nbrcote1 = 0;
        _wellDone                            = false;
        _nbrSol                              = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        CurveAdaptor2D cu1 = qualified1.Qualified();
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
            Circle2D       circ = new(new Axis2D(point2, new Direction2D(Direction2D.D.X)), radius);
            TwoDDomain d1   = new(global::Kerf.Engine.Geometry.ElCLib.Value(0.0, circ), 0.0, tol, global::Kerf.Engine.Geometry.ElCLib.Value(M_PI + M_PI, circ), M_PI + M_PI, tol);
            d1.SetEquivalentParameters(0.0, M_PI + M_PI);
            TheIntConicCurveOfGInter intp = new();
            for (int jcote1 = 1; jcote1 <= nbrcote1 && _nbrSol < ANbSolMAX; jcote1++)
            {
                CurveAdaptor2D hCu1 = (CurveAdaptor2D)cu1.ShallowCopy(); // new Geom2dAdaptor_Curve(Cu1)
                // Adaptor2d_OffsetCurve Cu2(HCu1,cote1(jcote1));
                OffsetCurveAdaptor2D cu2 = new(hCu1, -cote1[jcote1]);
                firstparam = Max(cu2.FirstParameter(), thefirst);
                lastparam  = Min(cu2.LastParameter(), thelast);
                TwoDDomain d2 = new(cu2.Value(firstparam), firstparam, tol, cu2.Value(lastparam), lastparam, tol);
                intp.Perform(circ, d1, cu2, d2, tol, tol);
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
                            _qualifier2[_nbrSol] = Qualifier.Noqualifier;
                            _theSame1[_nbrSol]   = 0;
                            _theSame2[_nbrSol]   = 0;
                            _pararg1[_nbrSol]    = intp.Point(i).ParamOnSecond();
                            _pararg2[_nbrSol]    = 0.0;
                            _pnttg1sol[_nbrSol]  = ConstraintCurveTool.Value(cu1, _pararg1[_nbrSol]);
                            _pnttg2sol[_nbrSol]  = point2;
                            _par1sol[_nbrSol]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                            _par2sol[_nbrSol]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                        }
                    }
                    _wellDone = true;
                }
            }
        }
    }

    //=======================================================================
    // function : PrecRoot
    // purpose  : In case, when curves has tangent zones, intersection point
    //            found may be precised. This function uses precision algorithm
    //            of Extrema Curve-Curve method (dot product between every
    //            tangent vector and vector between points in two curves must
    //            be equal to zero).
    //=======================================================================
    private static void PrecRoot(OffsetCurveAdaptor2D c1,
                                 OffsetCurveAdaptor2D c2,
                                 double                u0,
                                 double                v0,
                                 out double            ufinal,
                                 out double            vfinal)
    {
        // It is necessary for precision to solve the system
        //     (x1(u) - x2(v)) * x1'(u) + (y1(u) - y2(v)) * y1'(u) = 0
        //     (x1(u) - x2(v)) * x2'(v) + (y1(u) - y2(v)) * y2'(v) = 0
        // by Newton's method (OCCT's comment gives the iteration of any 2 x 2 system).

        ufinal = u0;
        vfinal = v0;

        const int nbIterMax = 100;

        double u = u0, v = v0;

        int nbIter = 0;

        double stepU = 0.0, stepV = 0.0;

        double sqDistPrev = RealFirst();

        c1.D2(u, out Point2D pu, out Vector2D d1u, out Vector2D d2u);
        c2.D2(v, out Point2D pv, out Vector2D d1v, out Vector2D d2v);

        double crProd = Math.Abs(d1u.Crossed(d1v));
        if (crProd * crProd > 1.0e-6 * d1u.SquareMagnitude * d1v.SquareMagnitude)
        {
            // Curves are not tangent. Therefore, we consider that
            // 2D-intersection algorithm have found good point which
            // did not need in more precision.
            return;
        }

        do
        {
            nbIter++;

            Vector2D vuv = new(pv, pu);

            double sqDist = vuv.SquareMagnitude;
            if (IsEqual(sqDist, 0.0))
            {
                break;
            }

            if ((nbIter == 1) || (sqDist < sqDistPrev))
            {
                sqDistPrev = sqDist;
                ufinal   = u;
                vfinal   = v;
            }

            double g1 = d1u.Magnitude;
            double g2 = d1v.Magnitude;

            if (IsEqual(g1, 0.0) || IsEqual(g2, 0.0))
            { // Here we do not processing singular cases.
                break;
            }

            double f1 = vuv.Dot(d1u);
            double f2 = vuv.Dot(d1v);

            double fIu  = vuv.Dot(d2u);
            double fIv  = vuv.Dot(d2v);
            double psIu = d1u.Dot(d2u);
            double psIv = d1v.Dot(d2v);

            double theta = d1u * d1v;

            double s1 = f1 / g1;
            double s2 = f2 / g2;

            double ds1u = (g1 * g1 + fIu) / g1 - (s1 * psIu / (g1 * g1));
            double ds1v = -theta / g1;
            double ds2u = theta / g2;
            double ds2v = (fIv - g2 * g2) / g2 - (s2 * psIv / (g2 * g2));

            double det = ds1u * ds2v - ds1v * ds2u;

            if (IsEqual(det, 0.0))
            {
                if (!IsEqual(stepV, 0.0) && !IsEqual(ds1u, 0.0))
                {
                    v += stepV;
                    u = u - (ds1v * stepV - s1) / ds1u;
                }
                else if (!IsEqual(stepU, 0.0) && !IsEqual(ds1v, 0.0))
                {
                    u += stepU;
                    v = v - (ds1u * stepU - s1) / ds1v;
                }
                else
                {
                    break;
                }
            }
            else
            {
                stepU = -(s1 * ds2v - s2 * ds1v) / det;
                stepV = -(s2 * ds1u - s1 * ds2u) / det;

                if (Math.Abs(stepU) < Epsilon(Math.Abs(u)))
                {
                    if (Math.Abs(stepV) < Epsilon(Math.Abs(v)))
                    {
                        break;
                    }
                }

                u += stepU;
                v += stepV;
            }

            c1.D2(u, out pu, out d1u, out d2u);
            c2.D2(v, out pv, out d1v, out d2v);
        } while (nbIter <= nbIterMax);
    }

    // circulaire tant a deux courbes ,de rayon donne
    //==================================================

    //========================================================================
    // On initialise WellDone a false.                                       +
    // On recupere les courbes Cu1 et Cu2.                                   +
    // On sort en erreur dans les cas ou la construction est impossible.     +
    // On fait la parallele a Cu1 dans le bon sens.                          +
    // On fait la parallele a Cu2 dans le bon sens.                          +
    // On intersecte les paralleles ==> point de centre de la solution.      +
    // On cree la solution qu on ajoute aux solutions deja trouvees.         +
    // On remplit les champs.                                                +
    //========================================================================

    /// <summary>Circles of radius Radius tangent to two qualified curves.</summary>
    /// <exception cref="BadQualifierException">A qualifier isn't one of the four positions.</exception>
    /// <exception cref="NegativeValueException">Radius is negative.</exception>
    public Circ2d2TanRadGeo(in QCurve qualified1, in QCurve qualified2, double radius, double tolerance)
    {
        //========================================================================
        // Traitement.                                                           +
        //========================================================================

        double                     tol      = Math.Abs(tolerance);
        Direction2D                   dirx     = new(Direction2D.D.X);
        NCollection_Array1<double> cote1    = new(1, 2);
        NCollection_Array1<double> cote2    = new(1, 2);
        int                        nbrcote1 = 0;
        int                        nbrcote2 = 0;
        _wellDone                            = false;
        _nbrSol                              = 0;
        if (!(qualified1.IsEnclosed || qualified1.IsEnclosing || qualified1.IsOutside
              || qualified1.IsUnqualified)
            || !(qualified2.IsEnclosed || qualified2.IsEnclosing || qualified2.IsOutside
                 || qualified2.IsUnqualified))
        {
            throw new BadQualifierException();
        }
        CurveAdaptor2D cu1 = qualified1.Qualified();
        CurveAdaptor2D cu2 = qualified2.Qualified();
        if (radius < 0.0)
        {
            throw new NegativeValueException();
        }
        else
        {
            if (qualified1.IsEnclosed && qualified2.IsEnclosed)
            {
                //   =======================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = radius;
                cote2[1] = radius;
            }
            else if (qualified1.IsEnclosed && qualified2.IsOutside)
            {
                //   ==========================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = radius;
                cote2[1] = -radius;
            }
            else if (qualified1.IsOutside && qualified2.IsEnclosed)
            {
                //   ===========================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = -radius;
                cote2[1] = radius;
            }
            else if (qualified1.IsOutside && qualified2.IsOutside)
            {
                //   =========================================================
                nbrcote1 = 1;
                nbrcote2 = 1;
                cote1[1] = -radius;
                cote2[1] = -radius;
            }
            if (qualified1.IsEnclosed && qualified2.IsUnqualified)
            {
                //   =========================================================
                nbrcote1 = 1;
                nbrcote2 = 2;
                cote1[1] = radius;
                cote2[1] = radius;
                cote2[2] = -radius;
            }
            if (qualified1.IsUnqualified && qualified2.IsEnclosed)
            {
                //   =========================================================
                nbrcote1 = 2;
                nbrcote2 = 1;
                cote1[1] = radius;
                cote1[2] = -radius;
                cote2[1] = radius;
            }
            else if (qualified1.IsOutside && qualified2.IsUnqualified)
            {
                //   =============================================================
                nbrcote1 = 1;
                nbrcote2 = 2;
                cote1[1] = -radius;
                cote2[1] = radius;
                cote2[2] = -radius;
            }
            if (qualified1.IsUnqualified && qualified2.IsOutside)
            {
                //   ========================================================
                nbrcote1 = 2;
                nbrcote2 = 1;
                cote1[1] = radius;
                cote1[2] = -radius;
                cote2[1] = -radius;
            }
            else if (qualified1.IsUnqualified && qualified2.IsUnqualified)
            {
                //   =================================================================
                nbrcote1 = 2;
                nbrcote2 = 2;
                cote1[1] = radius;
                cote1[2] = -radius;
                cote2[1] = radius;
                cote2[2] = -radius;
            }
            GInter intp = new();
            for (int jcote1 = 1; jcote1 <= nbrcote1; jcote1++)
            {
                CurveAdaptor2D hCu1 = (CurveAdaptor2D)cu1.ShallowCopy(); // new Geom2dAdaptor_Curve(Cu1)
                // Adaptor2d_OffsetCurve C1(HCu1,cote1(jcote1));
                OffsetCurveAdaptor2D c1 = new(hCu1, -cote1[jcote1]);
                for (int jcote2 = 1; jcote2 <= nbrcote2 && _nbrSol < ANbSolMAX; jcote2++)
                {
                    CurveAdaptor2D hCu2 = (CurveAdaptor2D)cu2.ShallowCopy(); // new Geom2dAdaptor_Curve(Cu2)
                    // Adaptor2d_OffsetCurve C2(HCu2,cote2(jcote2));
                    OffsetCurveAdaptor2D c2 = new(hCu2, -cote2[jcote2]);
                    intp.Perform(c1, c2, tol, tol);
                    if (intp.IsDone)
                    {
                        if (!intp.IsEmpty)
                        {
                            double sqApproxTol = global::Kerf.Engine.Foundation.Precision.Approximation() * global::Kerf.Engine.Foundation.Precision.Approximation();
                            for (int i = 1; i <= intp.PointCount && _nbrSol < ANbSolMAX; i++)
                            {
                                double u0 = intp.Point(i).ParamOnFirst();
                                double v0 = intp.Point(i).ParamOnSecond();

                                double u1 = u0 - global::Kerf.Engine.Foundation.Precision.PApproximation();
                                double v1 = v0 - global::Kerf.Engine.Foundation.Precision.PApproximation();

                                double u2 = u0 + global::Kerf.Engine.Foundation.Precision.PApproximation();
                                double v2 = v0 + global::Kerf.Engine.Foundation.Precision.PApproximation();

                                Point2D p11 = c1.Value(u1);
                                Point2D p12 = c2.Value(v1);
                                Point2D p21 = c1.Value(u2);
                                Point2D p22 = c2.Value(v2);

                                double dist1112 = p11.SquareDistanceTo(p12);
                                double dist1122 = p11.SquareDistanceTo(p22);

                                double dist1221 = p12.SquareDistanceTo(p21);
                                double dist2122 = p21.SquareDistanceTo(p22);

                                if ((Min(dist1112, dist1122) <= sqApproxTol)
                                    && (Min(dist1221, dist2122) <= sqApproxTol))
                                {
                                    PrecRoot(c1, c2, u0, v0, out u0, out v0);
                                }

                                _nbrSol++;
                                Point2D center = c1.Value(u0);
                                _cirsol[_nbrSol]  = new Circle2D(new Axis2D(center, dirx), radius);
                                //             =======================================================
                                _qualifier1[_nbrSol] = qualified1.Qualifier();
                                // OCCT: sets qualifier1 twice and never qualifier2, which stays uninitialised
                                // natively (0, GccEnt_unqualified, in the port).
                                _qualifier1[_nbrSol] = qualified1.Qualifier();
                                _theSame1[_nbrSol]   = 0;
                                _theSame2[_nbrSol]   = 0;
                                // The tangency data are taken at the intersection's parameters, not PrecRoot's.
                                _pararg1[_nbrSol]    = intp.Point(i).ParamOnFirst();
                                _pararg2[_nbrSol]    = intp.Point(i).ParamOnSecond();
                                _pnttg1sol[_nbrSol]  = ConstraintCurveTool.Value(cu1, _pararg1[_nbrSol]);
                                _pnttg2sol[_nbrSol]  = ConstraintCurveTool.Value(cu2, _pararg2[_nbrSol]);
                                _par1sol[_nbrSol]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg1sol[_nbrSol]);
                                _par2sol[_nbrSol]    = global::Kerf.Engine.Geometry.ElCLib.Parameter(_cirsol[_nbrSol], _pnttg2sol[_nbrSol]);
                            }
                        }

                        _wellDone = true;
                    }
                }
            }
        }
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

    /// <summary>The solution of index Index (1 to NbSolutions).</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is out of range.</exception>
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

    /// <summary>The qualifiers of the arguments for the solution of index Index.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is out of range.</exception>
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

    /// <summary>The tangency point on the first argument: its parameters on the solution and on the argument.</summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is the argument.</exception>
    /// <exception cref="OutOfRangeException">Index is out of range.</exception>
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

    /// <summary>The tangency point on the second argument: its parameters on the solution and on the argument.</summary>
    /// <exception cref="NotDoneException">The construction failed, or the solution is the argument.</exception>
    /// <exception cref="OutOfRangeException">Index is out of range.</exception>
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

    /// <summary>True if the solution of index Index is the first argument.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is out of range.</exception>
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

    /// <summary>True if the solution of index Index is the second argument.</summary>
    /// <exception cref="NotDoneException">The construction failed.</exception>
    /// <exception cref="OutOfRangeException">Index is out of range.</exception>
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
