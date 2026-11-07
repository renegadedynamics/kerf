// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImpParGen/IntImpParGen_Intersector.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Engine.Numerics;
using Kerf.Geometry;

namespace Kerf.Engine.Intersection.Implicit;

/// <summary>
/// The static tool over a parametric curve that the generic intersector uses (OCCT's ParTool macro:
/// IntCurve_PConicTool, Geom2dInt_Geom2dCurveTool).
/// </summary>
internal interface IParTool<ParCurve>
{
    static abstract double EpsX(in ParCurve c);

    static abstract int NbSamples(in ParCurve c, double u0, double u1);

    static abstract Point2D Value(in ParCurve c, double x);

    static abstract void D2(in ParCurve c, double u, out Point2D p, out Vector2D t, out Vector2D n);
}

/// <summary>
/// The static projection of a point on a parametric curve that the generic intersector uses (OCCT's
/// ProjectOnPCurveTool macro: IntCurve_ProjectOnPConicTool, Geom2dInt_TheProjPCurOfGInter).
/// </summary>
internal interface IProjectOnPCurveTool<ParCurve>
{
    /// <summary>The parameter of the point of <paramref name="c"/> nearest to <paramref name="pnt"/>.</summary>
    static abstract double FindParameter(in ParCurve c, in Point2D pnt, double tol);

    /// <summary>
    /// The parameter of the point of <paramref name="c"/> nearest to <paramref name="pnt"/>, searched between the
    /// two parameters.
    /// </summary>
    static abstract double FindParameter(in ParCurve c, in Point2D pnt, double lowParameter, double highParameter,
                                         double tol);
}

/// <summary>
/// The function whose roots the generic intersector looks for: the signed distance to the implicit curve of the
/// point at a parameter on the parametric curve (OCCT's IntImpParGen_MyImpParTool macro, a
/// math_FunctionWithDerivative constructed from the two curves: IntCurve_MyImpParToolOfIntImpConicParConic,
/// Geom2dInt_MyImpParToolOfTheIntersectorOfTheIntConicCurveOfGInter).
/// </summary>
internal interface IMyImpParTool<TSelf, ImpTool, ParCurve>
    where TSelf : FunctionWithDerivative, IMyImpParTool<TSelf, ImpTool, ParCurve>
{
    /// <summary>OCCT's constructor <c>MyImpParTool(const ImpTool&amp; IT, const ParCurve&amp; PC)</c>.</summary>
    static abstract TSelf Create(in ImpTool it, in ParCurve pc);
}

/// <summary>
/// The intersection between an implicit curve (<typeparamref name="ImpTool"/>) and a bounded parametric curve
/// (<typeparamref name="ParCurve"/>): the roots of the signed distance along the parametric curve, then the
/// transitions and the domain bounds that are solutions.
/// </summary>
/// <remarks>
/// OCCT's generic file IntImpParGen_Intersector.gxx, ported once as a generic class (decision 202's approach);
/// its instantiations (IntCurve_IntImpConicParConic, Geom2dInt_TheIntersectorOfTheIntConicCurveOfGInter) are
/// sealed subclasses in IntCurve and Geom2dInt, with OCCT's constructors forwarded.
/// </remarks>
internal class Intersector<ImpTool, ParCurve, ParTool, ProjectOnPCurveTool, MyImpParTool>
    : Intersection2D
    where ImpTool : IImpTool
    where ParTool : IParTool<ParCurve>
    where ProjectOnPCurveTool : IProjectOnPCurveTool<ParCurve>
    where MyImpParTool : FunctionWithDerivative, IMyImpParTool<MyImpParTool, ImpTool, ParCurve>
{
    //======================================================================
    //--     C o n s t r u c t e u r s     e t     P e r f o r m

    /// <summary>Empty constructor.</summary>
    public Intersector()
    {
        _done = false;
    }

    /// <summary>Intersection between an implicit curve and a parametric curve.</summary>
    /// <exception cref="ConstructionErrorException">The domain of the parametric curve is not bounded on both sides.</exception>
    public Intersector(in ImpTool theImpTool,
                                    in TwoDDomain theImpCurveDomain,
                                    in ParCurve theParCurve,
                                    in TwoDDomain theParCurveDomain,
                                    double tolConf,
                                    double tol)
    {
        Perform(theImpTool, theImpCurveDomain, theParCurve, theParCurveDomain, tolConf, tol);
    }

    /// <summary>Intersection between an implicit curve and a parametric curve.</summary>
    /// <exception cref="ConstructionErrorException">The domain of the parametric curve is not bounded on both sides.</exception>
    public void Perform(in ImpTool theImpTool,
                        in TwoDDomain theImpCurveDomain,
                        in ParCurve theParCurve,
                        in TwoDDomain theParCurveDomain,
                        double tolConf,
                        double tol)
    {
        int i, nb_segments_solution, nb_points_solution;
        double param1, param2, epsX, epsNul, epsDist;

        Transition trans1 = new(), trans2 = new();
        Point2D pt1, pt2;
        Vector2D tan1, tan2, norm1, norm2;
        Position2D pos1, pos2;

        //----------------------------------------------
        //-- On teste apres appel aux maths si les bornes
        //-- des domaines sont des solutions
        //--
        bool headOnImp = false;
        bool headOnPar = false;
        bool endOnImp = false;
        bool endOnPar = false;

        this.ResetFields();

        MyImpParTool theImpParTool = MyImpParTool.Create(theImpTool, theParCurve);

        if (!(theParCurveDomain.HasFirstPoint && theParCurveDomain.HasLastPoint))
        {
            throw new ConstructionErrorException("Domaine sur courbe incorrect");
        }

        int nb_echantillons = ParTool.NbSamples(theParCurve,
                                                theParCurveDomain.FirstParameter(),
                                                theParCurveDomain.LastParameter());

        epsX = ParTool.EpsX(theParCurve);
        if (epsX > 1.0e-10)
        {
            epsX = 1.0e-10;
        }

        epsNul = (tolConf <= 1.0e-10) ? 1.0e-10 : tolConf;
        epsDist = (tol <= 1.0e-10) ? 1.0e-10 : tol;

        double Tolerance_Angulaire = epsDist;

        if ((theParCurveDomain.LastParameter() - theParCurveDomain.FirstParameter()) < 100.0 * epsX)
        {
            epsX = (theParCurveDomain.LastParameter() - theParCurveDomain.FirstParameter()) * 0.01;
        }

        FunctionSample sample2 = new(theParCurveDomain.FirstParameter(),
                                          theParCurveDomain.LastParameter(),
                                          nb_echantillons);

        FunctionAllRoots sol = new(theImpParTool, sample2, epsX, epsDist, epsNul);

        if (!sol.IsDone)
        {
            _done = false;
            return;
        }

        nb_segments_solution = sol.IntervalCount;
        nb_points_solution = sol.PointCount;

        //--------------------------------------------------------------------
        //--   T r a i t e m e n t    d e s   P o i n t s   S o l u t i o n s
        for (i = 1; i <= nb_points_solution; i++)
        {
            Point2D pt;
            param2 = sol.GetPoint(i);
            param1 = FindU(param2, out pt, theParCurve, theImpTool);

            if (theImpCurveDomain.IsClosed)
            {
                param1 = global::Kerf.Engine.Intersection.Implicit.IntImpParGen.NormalizeOnDomain(param1, theImpCurveDomain);
            }

            bool isOnTheImpCurveDomain = true;
            if (theImpCurveDomain.HasFirstPoint)
            {
                if (param1 < theImpCurveDomain.FirstParameter())
                {
                    if (pt.DistanceTo(theImpCurveDomain.FirstPoint()) > theImpCurveDomain.FirstTolerance())
                    {
                        isOnTheImpCurveDomain = false;
                    }
                }
            }

            if (isOnTheImpCurveDomain && theImpCurveDomain.HasLastPoint)
            {
                if (param1 > theImpCurveDomain.LastParameter())
                {
                    if (pt.DistanceTo(theImpCurveDomain.LastPoint()) > theImpCurveDomain.LastTolerance())
                    {
                        isOnTheImpCurveDomain = false;
                    }
                }
            }

            if (isOnTheImpCurveDomain)
            {
                theImpTool.D2(param1, out pt1, out tan1, out norm1);
                ParTool.D2(theParCurve, param2, out pt2, out tan2, out norm2);

                global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos1, theImpCurveDomain, pt1, param1);
                global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos2, theParCurveDomain, pt2, param2);

                if (pos1 == Position2D.End)
                {
                    endOnImp = true;
                }
                else if (pos1 == Position2D.Head)
                {
                    headOnImp = true;
                }

                if (pos2 == Position2D.End)
                {
                    endOnPar = true;
                }
                else if (pos2 == Position2D.Head)
                {
                    headOnPar = true;
                }

                global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(pos1,
                                                 ref tan1,
                                                 norm1,
                                                 ref trans1,
                                                 pos2,
                                                 ref tan2,
                                                 norm2,
                                                 ref trans2,
                                                 Tolerance_Angulaire);

                IntersectionPoint2D ip = new(pt1, param1, param2, trans1, trans2, ReversedParameters());
                Insert(ip);
            }
        }

        //-- F i n   d u   T r a i t e m e n t   d e s   P t s   S o l.

        //--------------------------------------------------------------------
        //--        T r a i t e m e n t   D e s   S e g m e n t s          ---
        //--------------------------------------------------------------------
        //--  On a N segments solution sur le domaine de V    soit au pire :
        //--    --> N segments solution sur le domaine de U
        //--    -->2N segments si la courbe en U est fermee
        //--
        NCollection_Array1<double> Inter2_and_Domaine2 = new(1, 2 + 8 * nb_segments_solution);
        NCollection_Array1<double> inter1 = new(1, 2 + 8 * nb_segments_solution);

        int nb_segments_crees = 0;

        for (int j2 = 1, j = 1; j <= nb_segments_solution; j++, j2 += 2)
        {
            double param2_inf, param2_sup;
            double param1_inf, param1_sup;
            Point2D ptemp;

            sol.GetInterval(j, out param2_inf, out param2_sup);
            param1_inf = FindU(param2_inf, out ptemp, theParCurve, theImpTool);
            param1_sup = FindU(param2_sup, out ptemp, theParCurve, theImpTool);

            //----------------------------------------------------------------------
            //--         C o u r b e    I m p l i c i t e    F e r m e e

            if (theImpCurveDomain.IsClosed)
            {
                Vector2D t1, t2, n1, n2;
                double param1_origine, param1_fin;

                theImpCurveDomain.EquivalentParameters(out param1_origine, out param1_fin);
                double periode = param1_fin - param1_origine;

                for (int aStep = 0; param1_inf < param1_origine; StepLimit.Next(ref aStep))
                {
                    param1_inf += periode;
                }

                for (int aStep = 0; param1_sup < param1_origine; StepLimit.Next(ref aStep))
                {
                    param1_sup += periode;
                }

                ParTool.D2(theParCurve, param2_inf, out ptemp, out t2, out n2);
                theImpTool.D2(param1_inf, out ptemp, out t1, out n1);
                if (t1.Magnitude <= Constants.Resolution())
                {
                    t1 = n1;
                }

                if (t2.Magnitude <= Constants.Resolution())
                {
                    t2 = n2;
                }

                if (t1.Dot(t2) >= 0.0)
                {
                    //---  param1_inf designe un point entrant (et T1 est vers la matiere)
                    if (param1_inf >= param1_sup)
                    {
                        param1_sup += periode;
                    }
                }
                else
                { //---  param1_inf : point sortant (et T1 est Hors matiere)
                    if (param1_inf <= param1_sup)
                    {
                        param1_inf += periode;
                    }
                }

                //--- On cree un nouveau segment decale de Periode
                //--  Exemple de Pb : Domaine PI/4  PI/2   et intervalle 0,PI
                //--                  Domaine 1.5PI 2.5PI  et intervalle 0,PI
                //--        -2pi                  0                2pi
                //--   ------|--------------------|-----------------|-----------
                //--            [----------------------------]               Domaine
                //--                                   [~~~~~~~~~~~~~~~~~]   Inters.
                //--
                //--  On cree un nouvel intervalle
                //--   interv decale
                //--  [a~~~~~~~~~~~~b]                 [a~~~~~~~~~~~~~~~b]  et  [a~~~]
                //--

                if (theImpCurveDomain.LastParameter()
                    > ((param1_inf > param1_sup) ? (param1_sup + periode) : (param1_inf + periode)))
                {
                    Inter2_and_Domaine2.SetValue(j2, param2_inf);
                    inter1.SetValue(j2, param1_inf + periode);

                    Inter2_and_Domaine2.SetValue(j2 + 1, param2_sup);
                    inter1.SetValue(j2 + 1, param1_sup + periode);
                    j2 += 2;
                    nb_segments_crees++;
                }

                if (theImpCurveDomain.FirstParameter()
                    < ((param1_inf < param1_sup) ? (param1_sup - periode) : (param1_inf - periode)))
                {
                    Inter2_and_Domaine2.SetValue(j2, param2_inf);
                    inter1.SetValue(j2, param1_inf - periode);
                    Inter2_and_Domaine2.SetValue(j2 + 1, param2_sup);
                    inter1.SetValue(j2 + 1, param1_sup - periode);
                    j2 += 2;
                    nb_segments_crees++;
                }
            }

            //--   F i n     C o u r b e    I m p l i c i t e    F e r m e e
            //----------------------------------------------------------------------

            Inter2_and_Domaine2.SetValue(j2, param2_inf);
            inter1.SetValue(j2, param1_inf);

            Inter2_and_Domaine2.SetValue(j2 + 1, param2_sup);
            inter1.SetValue(j2 + 1, param1_sup);
        }

        //------------------------------------------------------------------
        //--  INTER2_DOMAINE2 : Intersection AND CurveDomain : Function of PARAM2
        //--    INTER1        : Intersection AND CurveDomain : Function of PARAM1
        //------------------------------------------------------------------
        //--
        // OCCT: the result arrays are sized before nb_segments_crees is added, for at most 2 + nb_segments_solution
        // intervals, where And_Domaine_Objet1_Intersections may write two values for each of the
        // nb_segments_solution + nb_segments_crees intervals (out of bounds in C++, an exception here).
        NCollection_Array1<double> resultat1 = new(1, 2 + (1 + nb_segments_solution) * 2);
        NCollection_Array1<double> resultat2 = new(1, 2 + (1 + nb_segments_solution) * 2);
        nb_segments_solution += nb_segments_crees;
        int nbResultats = nb_segments_solution * 2;

        And_Domaine_Objet1_Intersections(theImpTool,
                                         theParCurve,
                                         theImpCurveDomain,
                                         theParCurveDomain,
                                         ref nbResultats,
                                         Inter2_and_Domaine2,
                                         inter1,
                                         resultat1,
                                         resultat2,
                                         epsNul);

        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        //~~~ Fonction Calcule_Toutes_Transitions Repportee ici pour cause ~~~~~
        //~~~~~       D acces aux methodes Protected APPEND
        //~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        {
            Point2D Pt1_on1, Pt2_on1, Pt1_on2, Pt2_on2;
            double Param1_on1, Param2_on1, Param1_on2, Param2_on2;

            double Dist_Mini_ImpCurve = tolConf; // EPSNUL
            double toleranceAngulaireDistMini = Dist_Mini_ImpCurve;

            for (int k = 1; k <= nbResultats; k += 2)
            {
                int ip1 = k + 1;
                bool onlyOnePoint = false;

                Param1_on1 = resultat1.Value(k);
                Param1_on2 = resultat2.Value(k);
                Param2_on1 = resultat1.Value(ip1);
                Param2_on2 = resultat2.Value(ip1);

                Pt1_on1 = theImpTool.Value(Param1_on1);
                Pt2_on1 = theImpTool.Value(Param2_on1);
                Pt1_on2 = ParTool.Value(theParCurve, Param1_on2);
                Pt2_on2 = ParTool.Value(theParCurve, Param2_on2);

                if (!theImpCurveDomain.IsClosed)
                {
                    if (Pt1_on1.DistanceTo(Pt2_on1) <= Dist_Mini_ImpCurve)
                    {
                        if (Pt1_on2.DistanceTo(Pt2_on2) <= Dist_Mini_ImpCurve)
                        {
                            onlyOnePoint = true;
                        }
                    }
                }

                Param1_on1 = global::Kerf.Engine.Intersection.Implicit.IntImpParGen.NormalizeOnDomain(Param1_on1, theImpCurveDomain);
                Param1_on2 = global::Kerf.Engine.Intersection.Implicit.IntImpParGen.NormalizeOnDomain(Param1_on2, theParCurveDomain);

                theImpTool.D2(Param1_on1, out Pt1_on1, out tan1, out norm1);
                ParTool.D2(theParCurve, Param1_on2, out Pt1_on2, out tan2, out norm2);

                global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos1, theImpCurveDomain, Pt1_on1, Param1_on1);
                global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos2, theParCurveDomain, Pt1_on2, Param1_on2);

                if (pos1 == Position2D.End)
                {
                    endOnImp = true;
                }
                else if (pos1 == Position2D.Head)
                {
                    headOnImp = true;
                }

                if (pos2 == Position2D.End)
                {
                    endOnPar = true;
                }
                else if (pos2 == Position2D.Head)
                {
                    headOnPar = true;
                }

                global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(pos1,
                                                 ref tan1,
                                                 norm1,
                                                 ref trans1,
                                                 pos2,
                                                 ref tan2,
                                                 norm2,
                                                 ref trans2,
                                                 toleranceAngulaireDistMini);

                //============== Detection du cas : L intersection est en bout
                //==============  sur les 2 domaines

                if (pos1 != Position2D.Middle && pos2 != Position2D.Middle)
                {
                    double m = 0.5 * (Pt1_on1.X + Pt1_on2.X);
                    Pt1_on1 = Pt1_on1.WithX(m);
                    m = 0.5 * (Pt1_on1.Y + Pt1_on2.Y);
                    Pt1_on1 = Pt1_on1.WithY(m);
                }

                IntersectionPoint2D new_p1 = new(Pt1_on1,
                                                        Param1_on1,
                                                        Param1_on2,
                                                        trans1,
                                                        trans2,
                                                        ReversedParameters());
                if (!onlyOnePoint)
                {
                    IntersectionPoint2D new_p2 = new();

                    Param2_on1 = global::Kerf.Engine.Intersection.Implicit.IntImpParGen.NormalizeOnDomain(Param2_on1, theImpCurveDomain);

                    Param2_on2 = global::Kerf.Engine.Intersection.Implicit.IntImpParGen.NormalizeOnDomain(Param2_on2, theParCurveDomain);

                    theImpTool.D2(Param2_on1, out Pt2_on1, out tan1, out norm1);
                    ParTool.D2(theParCurve, Param2_on2, out Pt2_on2, out tan2, out norm2);

                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos1, theImpCurveDomain, Pt2_on1, Param2_on1);
                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DeterminePosition(out pos2, theParCurveDomain, Pt2_on2, Param2_on2);

                    if (pos1 == Position2D.End)
                    {
                        endOnImp = true;
                    }
                    else if (pos1 == Position2D.Head)
                    {
                        headOnImp = true;
                    }

                    if (pos2 == Position2D.End)
                    {
                        endOnPar = true;
                    }
                    else if (pos2 == Position2D.Head)
                    {
                        headOnPar = true;
                    }

                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(pos1,
                                                     ref tan1,
                                                     norm1,
                                                     ref trans1,
                                                     pos2,
                                                     ref tan2,
                                                     norm2,
                                                     ref trans2,
                                                     toleranceAngulaireDistMini);

                    //============== Detection du cas : L intersection est en bout
                    //==============  sur les 2 domaines

                    if (pos1 != Position2D.Middle && pos2 != Position2D.Middle)
                    {
                        double m = 0.5 * (Pt2_on1.X + Pt2_on2.X);
                        Pt2_on1 = Pt2_on1.WithX(m);
                        m = 0.5 * (Pt2_on1.Y + Pt2_on2.Y);
                        Pt2_on1 = Pt2_on1.WithY(m);
                    }

                    new_p2.SetValues(Pt2_on1, Param2_on1, Param2_on2, trans1, trans2, ReversedParameters());

                    bool segopposite = ((tan1.Dot(tan2) < 0.0) ? true : false);

                    IntersectionSegment2D new_seg = new(new_p1, new_p2, segopposite, ReversedParameters());
                    Append(new_seg);
                }
                else
                {
                    Insert(new_p1);
                }
            }
        } //~~~~~~~~~~~~  Fin du corps de la fonction Calc...Transitions~~~~~~~~~~

        //-------------------------------------------
        //-- On teste les points en bouts solutions
        //--
        if (!headOnImp && theImpCurveDomain.HasFirstPoint)
        {
            if (!headOnPar)
            {
                if (theImpCurveDomain.FirstPoint().DistanceTo(theParCurveDomain.FirstPoint())
                    <= Max(theImpCurveDomain.FirstTolerance(), theParCurveDomain.FirstTolerance()))
                {
                    param1 = theImpCurveDomain.FirstParameter();
                    param2 = theParCurveDomain.FirstParameter();
                    theImpTool.D2(param1, out pt1, out tan1, out norm1);
                    ParTool.D2(theParCurve, param2, out pt2, out tan2, out norm2);
                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(Position2D.Head,
                                                     ref tan1,
                                                     norm1,
                                                     ref trans1,
                                                     Position2D.Head,
                                                     ref tan2,
                                                     norm2,
                                                     ref trans2,
                                                     Tolerance_Angulaire);
                    IntersectionPoint2D ip = new(theImpCurveDomain.FirstPoint(),
                                                        param1,
                                                        param2,
                                                        trans1,
                                                        trans2,
                                                        ReversedParameters());
                    Insert(ip);
                }
            }

            if (!endOnPar)
            {
                if (theImpCurveDomain.FirstPoint().DistanceTo(theParCurveDomain.LastPoint())
                    <= Max(theImpCurveDomain.FirstTolerance(), theParCurveDomain.LastTolerance()))
                {
                    param1 = theImpCurveDomain.FirstParameter();
                    param2 = theParCurveDomain.LastParameter();
                    theImpTool.D2(param1, out pt1, out tan1, out norm1);
                    ParTool.D2(theParCurve, param2, out pt2, out tan2, out norm2);
                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(Position2D.Head,
                                                     ref tan1,
                                                     norm1,
                                                     ref trans1,
                                                     Position2D.End,
                                                     ref tan2,
                                                     norm2,
                                                     ref trans2,
                                                     Tolerance_Angulaire);
                    IntersectionPoint2D ip = new(theImpCurveDomain.FirstPoint(),
                                                        param1,
                                                        param2,
                                                        trans1,
                                                        trans2,
                                                        ReversedParameters());
                    Insert(ip);
                }
            }
        }

        if (!endOnImp && theImpCurveDomain.HasLastPoint)
        {
            if (!headOnPar)
            {
                if (theImpCurveDomain.LastPoint().DistanceTo(theParCurveDomain.FirstPoint())
                    <= Max(theImpCurveDomain.LastTolerance(), theParCurveDomain.FirstTolerance()))
                {
                    param1 = theImpCurveDomain.LastParameter();
                    param2 = theParCurveDomain.FirstParameter();
                    theImpTool.D2(param1, out pt1, out tan1, out norm1);
                    ParTool.D2(theParCurve, param2, out pt2, out tan2, out norm2);
                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(Position2D.End,
                                                     ref tan1,
                                                     norm1,
                                                     ref trans1,
                                                     Position2D.Head,
                                                     ref tan2,
                                                     norm2,
                                                     ref trans2,
                                                     Tolerance_Angulaire);
                    IntersectionPoint2D ip = new(theImpCurveDomain.LastPoint(),
                                                        param1,
                                                        param2,
                                                        trans1,
                                                        trans2,
                                                        ReversedParameters());
                    Insert(ip);
                }
            }

            if (!endOnPar)
            {
                if (theImpCurveDomain.LastPoint().DistanceTo(theParCurveDomain.LastPoint())
                    <= Max(theImpCurveDomain.LastTolerance(), theParCurveDomain.LastTolerance()))
                {
                    param1 = theImpCurveDomain.LastParameter();
                    param2 = theParCurveDomain.LastParameter();
                    theImpTool.D2(param1, out pt1, out tan1, out norm1);
                    ParTool.D2(theParCurve, param2, out pt2, out tan2, out norm2);
                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(Position2D.End,
                                                     ref tan1,
                                                     norm1,
                                                     ref trans1,
                                                     Position2D.End,
                                                     ref tan2,
                                                     norm2,
                                                     ref trans2,
                                                     Tolerance_Angulaire);
                    IntersectionPoint2D ip = new(theImpCurveDomain.LastPoint(),
                                                        param1,
                                                        param2,
                                                        trans1,
                                                        trans2,
                                                        ReversedParameters());
                    Insert(ip);
                }
            }
        }

        _done = true;
    }

    /// <summary>The parameter on the implicit curve of the point at <paramref name="parameter"/> on the parametric curve.</summary>
    public double FindU(double parameter, out Point2D point, in ParCurve theParCurve, in ImpTool theImpTool)
    {
        point = ParTool.Value(theParCurve, parameter);
        return (theImpTool.FindParameter(point));
    }

    /// <summary>
    /// The parameter on the parametric curve of the point at <paramref name="parameter"/> on the implicit curve:
    /// projected on the whole curve if its domain is closed, otherwise between <paramref name="v0"/> and
    /// <paramref name="v1"/> and clamped to them.
    /// </summary>
    public double FindV(double parameter,
                        out Point2D point,
                        in ImpTool theImpTool,
                        in ParCurve theParCurve,
                        in TwoDDomain theParCurveDomain,
                        double v0,
                        double v1,
                        double tolerance)
    {
        point = theImpTool.Value(parameter);
        if (theParCurveDomain.IsClosed)
        {
            double v = ProjectOnPCurveTool.FindParameter(theParCurve, point, tolerance);
            return (global::Kerf.Engine.Intersection.Implicit.IntImpParGen.NormalizeOnDomain(v, theParCurveDomain));
        }
        else
        {
            double vv0 = v0;
            double vv1 = v1;
            if (v1 < v0)
            {
                vv0 = v1;
                vv1 = v0;
            }

            //-- ??????????????????????????????????????????????????????????????????????
            //-- Modif le 15 Septembre 1992 : On Teste le parametre retourne
            //--??????????????????????????????????????????????????????????????????????
            double x = ProjectOnPCurveTool.FindParameter(theParCurve, point, vv0, vv1, tolerance);
            if (x > vv1)
            {
                x = vv1;
            }
            else if (x < vv0)
            {
                x = vv0;
            }

            return (x);
        }
    }

    /// <summary>
    /// Restricts the solution intervals (pairs of parameters on both curves) to the domain of the implicit curve,
    /// replacing a bound outside it by the domain's bound and its parameter on the parametric curve.
    /// </summary>
    public void And_Domaine_Objet1_Intersections(in ImpTool theImpTool,
                                                 in ParCurve theParCurve,
                                                 in TwoDDomain theImpCurveDomain,
                                                 in TwoDDomain theParCurveDomain,
                                                 ref int nbResultats,
                                                 NCollection_Array1<double> Inter2_And_Domain2,
                                                 NCollection_Array1<double> inter1,
                                                 NCollection_Array1<double> resultat1,
                                                 NCollection_Array1<double> resultat2,
                                                 double epsNul)
    {
        int Nb_Bornes_Intersection = nbResultats;
        nbResultats = 0;

        for (int i = 1; i <= Nb_Bornes_Intersection; i += 2)
        {
            double param1 = inter1.Value(i);
            double param2 = inter1.Value(i + 1);

            int indice_1 = i;
            int indice_2 = i + 1;

            if (param1 > param2)
            {
                double t = param1;
                param1 = param2;
                param2 = t;
                indice_1 = i + 1;
                indice_2 = i;
            }

            Point2D pt1 = theImpTool.Value(param1);
            Point2D pt2 = theImpTool.Value(param2);
            Point2D pt;

            bool isOnTheImpCurveDomain1 = true;
            bool isOnTheImpCurveDomain2 = true;
            //--------------------------------------------------------------------
            if (theImpCurveDomain.HasFirstPoint)
            {
                if (param1 < theImpCurveDomain.FirstParameter())
                {
                    if (pt1.DistanceTo(theImpCurveDomain.FirstPoint()) > theImpCurveDomain.FirstTolerance())
                    {
                        isOnTheImpCurveDomain1 = false;
                    }
                }
            }

            if (isOnTheImpCurveDomain1 && theImpCurveDomain.HasLastPoint)
            {
                if (param1 > theImpCurveDomain.LastParameter())
                {
                    if (pt1.DistanceTo(theImpCurveDomain.LastPoint()) > theImpCurveDomain.LastTolerance())
                    {
                        isOnTheImpCurveDomain1 = false;
                    }
                }
            }

            //--------------------------------------------------------------------
            if (theImpCurveDomain.HasFirstPoint)
            {
                if (param2 < theImpCurveDomain.FirstParameter())
                {
                    if (pt2.DistanceTo(theImpCurveDomain.FirstPoint()) > theImpCurveDomain.FirstTolerance())
                    {
                        isOnTheImpCurveDomain2 = false;
                    }
                }
            }

            if (isOnTheImpCurveDomain2 && theImpCurveDomain.HasLastPoint)
            {
                if (param2 > theImpCurveDomain.LastParameter())
                {
                    if (pt2.DistanceTo(theImpCurveDomain.LastPoint()) > theImpCurveDomain.LastTolerance())
                    {
                        isOnTheImpCurveDomain2 = false;
                    }
                }
            }

            if (isOnTheImpCurveDomain1)
            {
                //------------------------------------------------------------------
                //---                 la borne 1 est sur le domaine               --
                nbResultats++;
                resultat1.SetValue(nbResultats, inter1.Value(indice_1));
                resultat2.SetValue(nbResultats, Inter2_And_Domain2.Value(indice_1));
                //---               la borne2 est aussi sur le domaine           ---
                if (isOnTheImpCurveDomain2)
                {
                    nbResultats++;
                    resultat1.SetValue(nbResultats, inter1.Value(indice_2));
                    resultat2.SetValue(nbResultats, Inter2_And_Domain2.Value(indice_2));
                }
                else
                {
                    //---    Borne1 sur domaine et Borne 2 Hors Domaine           ---
                    double t;
                    nbResultats++;
                    t = theImpCurveDomain.LastParameter();
                    resultat1.SetValue(nbResultats, t);
                    resultat2.SetValue(nbResultats,
                                       FindV(t,
                                             out pt,
                                             theImpTool,
                                             theParCurve,
                                             theParCurveDomain,
                                             Inter2_And_Domain2.Value(indice_1),
                                             Inter2_And_Domain2.Value(indice_2),
                                             epsNul));
                }
            }
            else
            { //======= la borne1 n est pas sur le domaine ========
                if (isOnTheImpCurveDomain2)
                {
                    double t;
                    nbResultats++;
                    t = theImpCurveDomain.FirstParameter();

                    resultat1.SetValue(nbResultats, t);
                    resultat2.SetValue(nbResultats,
                                       FindV(t,
                                             out pt,
                                             theImpTool,
                                             theParCurve,
                                             theParCurveDomain,
                                             Inter2_And_Domain2.Value(indice_1),
                                             Inter2_And_Domain2.Value(indice_2),
                                             epsNul));

                    nbResultats++;
                    resultat1.SetValue(nbResultats, inter1.Value(indice_2));
                    resultat2.SetValue(nbResultats, Inter2_And_Domain2.Value(indice_2));
                }
                else
                { //====== la borne2 et la borne1 sont hors domaine =====
                    if (param1 < theImpCurveDomain.FirstParameter()
                        && param2 > theImpCurveDomain.LastParameter())
                    {
                        double t;
                        nbResultats++;
                        t = theImpCurveDomain.FirstParameter();
                        resultat1.SetValue(nbResultats, t);
                        resultat2.SetValue(nbResultats,
                                           FindV(t,
                                                 out pt,
                                                 theImpTool,
                                                 theParCurve,
                                                 theParCurveDomain,
                                                 Inter2_And_Domain2.Value(indice_1),
                                                 Inter2_And_Domain2.Value(indice_2),
                                                 epsNul));

                        nbResultats++;
                        t = theImpCurveDomain.LastParameter();
                        resultat1.SetValue(nbResultats, t);
                        resultat2.SetValue(nbResultats,
                                           FindV(t,
                                                 out pt,
                                                 theImpTool,
                                                 theParCurve,
                                                 theParCurveDomain,
                                                 Inter2_And_Domain2.Value(indice_1),
                                                 Inter2_And_Domain2.Value(indice_2),
                                                 epsNul));
                    }
                }
            }
        }
    }
}
