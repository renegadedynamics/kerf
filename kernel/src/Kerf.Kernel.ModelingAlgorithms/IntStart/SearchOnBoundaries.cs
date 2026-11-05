// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntStart/IntStart_SearchOnBoundaries.gxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntStart/IntStart_SearchOnBoundaries.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Numerics;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;

namespace Kerf.Engine.Intersection.Walking;

/// <summary>
/// What the boundary search reads from its function (OCCT's TheFunction macro: IntPatch_ArcFunction,
/// Contap_ArcFunction): the function along the current arc, its samples, the points saved by
/// <c>GetStateNumber</c>, and the quadric and the surface it is built on.
/// </summary>
internal interface IArcFunction
{
    /// <summary>Loads the arc to search.</summary>
    void Set(Curve2DAdaptorBase a);

    /// <summary>The number of samples on the arc.</summary>
    int NbSamples();

    /// <summary>The point saved at state <paramref name="index"/>.</summary>
    Point3D Valpoint(int index);

    /// <summary>The quadric.</summary>
    SurfaceQuadric Quadric();

    /// <summary>The surface the arcs are on.</summary>
    SurfaceAdaptorBase Surface();

    /// <summary>The point of the last evaluation.</summary>
    Point3D LastComputedPoint();
}

/// <summary>
/// The static tool of the boundary search (OCCT's TheSOBTool macro: IntPatch_HInterTool, Contap_HContTool): the
/// vertices' parameters and tolerances on the arcs, the arcs' bounds, and the points and segments already known on an
/// arc.
/// </summary>
internal interface ISobTool
{
    static abstract bool HasBeenSeen(Curve2DAdaptorBase c);

    static abstract double Tolerance(HVertex v, Curve2DAdaptorBase c);

    static abstract double Parameter(HVertex v, Curve2DAdaptorBase c);

    static abstract void Bounds(Curve2DAdaptorBase c, out double ufirst, out double ulast);

    static abstract int NbPoints(Curve2DAdaptorBase c);

    static abstract void Value(Curve2DAdaptorBase c, int index, out Point3D pt, out double tol, out double u);

    static abstract bool IsVertex(Curve2DAdaptorBase c, int index);

    static abstract void Vertex(Curve2DAdaptorBase c, int index, out HVertex v);

    static abstract int NbSegments(Curve2DAdaptorBase c);

    static abstract bool HasFirstPoint(Curve2DAdaptorBase c, int index, out int indFirst);

    static abstract bool HasLastPoint(Curve2DAdaptorBase c, int index, out int indLast);

    static abstract bool IsAllSolution(Curve2DAdaptorBase c);
}

/// <summary>
/// A solution point of the boundary search (OCCT's IntStart_ThePathPoint macro: IntPatch_ThePathPointOfTheSOnBounds,
/// Contap_ThePathPointOfTheSearch): a struct, built and set as OCCT does.
/// </summary>
/// <typeparam name="TSelf">The implementing struct.</typeparam>
internal interface IPathPoint<TSelf>
    where TSelf : struct, IPathPoint<TSelf>
{
    /// <summary>A point on a vertex of an arc.</summary>
    static abstract TSelf Create(in Point3D p, double tol, HVertex v, Curve2DAdaptorBase a, double parameter);

    /// <summary>A new point on an arc.</summary>
    static abstract TSelf Create(in Point3D p, double tol, Curve2DAdaptorBase a, double parameter);

    /// <summary>Sets a point on a vertex of an arc.</summary>
    void SetValue(in Point3D p, double tol, HVertex v, Curve2DAdaptorBase a, double parameter);

    /// <summary>Sets a new point on an arc (the vertex is kept, as in OCCT).</summary>
    void SetValue(in Point3D p, double tol, Curve2DAdaptorBase a, double parameter);

    bool IsNew();

    HVertex Vertex();

    Curve2DAdaptorBase Arc();

    double Parameter();
}

/// <summary>
/// A solution segment of the boundary search (OCCT's IntStart_TheSegment macro: IntPatch_TheSegmentOfTheSOnBounds,
/// Contap_TheSegmentOfTheSearch): a struct.
/// </summary>
internal interface ISegment<ThePathPoint>
{
    /// <summary>Sets the arc (and forgets the limit points).</summary>
    void SetValue(Curve2DAdaptorBase a);

    /// <summary>Sets the first or the last point.</summary>
    void SetLimitPoint(in ThePathPoint v, bool first);
}

/// <summary>
/// The points and parts of the arcs of a domain (the restrictions of a surface) where a function F vanishes: for the
/// surface/surface intersection, where the arcs of a parametric surface meet a quadric. Points that fall on a vertex
/// of the domain are reported as that vertex.
/// </summary>
/// <remarks>
/// OCCT's generic file IntStart_SearchOnBoundaries.gxx, ported once as a generic class (decisions 337 and 607). The
/// macros that are the same in every instantiation (IntPatch_TheSOnBounds, Contap_TheSearch) are named directly:
/// TheVertex is Adaptor3d_HVertex, TheArc Adaptor2d_Curve2d, TheTopolTool Adaptor3d_TopolTool. TheArcTool is not used
/// by the body and is not a parameter. The others are type parameters: the static tool <typeparamref name="TheSOBTool"/>,
/// the function, and the result structs (whose constructors are the interfaces' static <c>Create</c>).
/// OCCT's instantiations are sealed subclasses.
/// <para>
/// OCCT's file-static helpers are private static methods. The roots found on an arc are sorted by parameter as OCCT's
/// <c>std::sort</c> does with MSVC on up to 32 roots (an insertion sort), on any number (decision 610).
/// </para>
/// </remarks>
internal class SearchOnBoundaries<TheSOBTool, TheFunction, ThePathPoint, TheSegment>
    where TheSOBTool : ISobTool
    where TheFunction : FunctionWithDerivative, IArcFunction
    where ThePathPoint : struct, IPathPoint<ThePathPoint>
    where TheSegment : struct, ISegment<ThePathPoint>
{
    private bool _done;
    private bool _all;
    private readonly NCollection_Sequence<TheSegment> _sseg = new();
    private readonly NCollection_Sequence<ThePathPoint> _spnt = new();

    /// <summary>The function squared, to minimise near a root (OCCT's file-local MinFunction).</summary>
    private sealed class MinFunction : Function
    {
        private readonly TheFunction _func;

        public MinFunction(TheFunction func) => _func = func;

        // returns value of the one-dimension-function when parameter
        // is equal to theX
        public override bool Value(double x, out double fVal)
        {
            if (!_func.Value(x, out fVal))
            {
                return false;
            }

            fVal *= fVal;
            return true;
        }

        // see analogical method for abstract owner class math_Function
        public override int GetStateNumber() => 0;
    }

    /// <summary>A root on an arc: its parameter and its rank in the solver (OCCT's file-local SolInfo).</summary>
    private struct SolInfo
    {
        private int _mathIndex;
        private double _value;

        public SolInfo()
        {
            _mathIndex = -1;
            _value = RealLast();
        }

        public void Init(FunctionAllRoots solution, int index)
        {
            _mathIndex = index;
            _value = solution.GetPoint(index);
        }

        public void Init(HInter solution, int index)
        {
            _mathIndex = index;
            _value = solution.Point(index).W();
        }

        public readonly double Value() => _value;

        public readonly int Index() => _mathIndex;

        public static bool operator <(SolInfo left, SolInfo right) => left._value < right._value;

        public static bool operator >(SolInfo left, SolInfo right) => left._value > right._value;

        [System.Diagnostics.CodeAnalysis.UnscopedRef]
        public ref double ChangeValue() => ref _value;
    }

    /// <summary>Empty constructor.</summary>
    public SearchOnBoundaries()
    {
        _done = false;
        _all = false;
    }

    /// <summary>
    /// Searches the points and parts of the arcs of <paramref name="domain"/> where F = 0.
    /// <paramref name="tolBoundary"/> defines whether a curve is on the quadric, <paramref name="tolTangency"/> whether a
    /// point is; with <paramref name="recheckOnRegularity"/>, a whole arc on a regular edge is searched as a segment.
    /// </summary>
    public void Perform(TheFunction func, TopolTool domain, double tolBoundary, double tolTangency,
                        bool recheckOnRegularity = false)
    {
        _done = false;
        _spnt.Clear();
        _sseg.Clear();

        bool arcsol;
        double pDeb, pFin, prm, tol;
        int i, nbknown, nbfound, index;
        Point3D pt;

        domain.Init();

        if (domain.More())
        {
            _all = true;
        }
        else
        {
            _all = false;
        }

        while (domain.More())
        {
            Curve2DAdaptorBase a = domain.Value();
            if (!TheSOBTool.HasBeenSeen(a))
            {
                func.Set(a);
                FindVertex(a, domain, func, _spnt, tolBoundary);
                TheSOBTool.Bounds(a, out pDeb, out pFin);
                if (global::Kerf.Engine.Foundation.Precision.IsNegativeInfinite(pDeb) || global::Kerf.Engine.Foundation.Precision.IsPositiveInfinite(pFin))
                {
                    ComputeBoundsfromInfinite(func, ref pDeb, ref pFin, out _);
                }
                BoundedArc(a, domain, pDeb, pFin, func, _spnt, _sseg, tolBoundary, tolTangency, out arcsol,
                           recheckOnRegularity);
                _all = (_all && arcsol);
            }

            else
            {
                // as it seems we'll never be here, because
                // TheSOBTool::HasBeenSeen(A) always returns FALSE
                nbfound = _spnt.Length();

                // On recupere les points connus
                nbknown = TheSOBTool.NbPoints(a);
                for (i = 1; i <= nbknown; i++)
                {
                    TheSOBTool.Value(a, i, out pt, out tol, out prm);
                    if (TheSOBTool.IsVertex(a, i))
                    {
                        TheSOBTool.Vertex(a, i, out HVertex vtx);
                        _spnt.Append(ThePathPoint.Create(pt, tol, vtx, a, prm));
                    }
                    else
                    {
                        _spnt.Append(ThePathPoint.Create(pt, tol, a, prm));
                    }
                }
                // On recupere les arcs solutions
                nbknown = TheSOBTool.NbSegments(a);
                for (i = 1; i <= nbknown; i++)
                {
                    TheSegment newseg = new();
                    newseg.SetValue(a);
                    if (TheSOBTool.HasFirstPoint(a, i, out index))
                    {
                        newseg.SetLimitPoint(_spnt.Value(nbfound + index), true);
                    }
                    if (TheSOBTool.HasLastPoint(a, i, out index))
                    {
                        newseg.SetLimitPoint(_spnt.Value(nbfound + index), false);
                    }
                    _sseg.Append(newseg);
                }
                _all = (_all & TheSOBTool.IsAllSolution(a));
            }
            domain.Next();
        }
        _done = true;
    }

    /// <summary>True if the computation succeeded.</summary>
    public bool IsDone => _done;

    /// <summary>True if every arc of the domain is a solution (inside the quadric). Throws StdFail_NotDone if not done.</summary>
    public bool AllArcSolution()
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _all;
    }

    /// <summary>The number of solution points. Throws StdFail_NotDone if not done.</summary>
    public int PointCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _spnt.Length();
        }
    }

    /// <summary>The solution point of range <paramref name="index"/>.</summary>
    public ThePathPoint Point(int index)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _spnt.Value(index);
    }

    /// <summary>The number of solution segments. Throws StdFail_NotDone if not done.</summary>
    public int SegmentCount
    {
        get
        {
            if (!_done)
            {
                throw new NotDoneException();
            }
            return _sseg.Length();
        }
    }

    /// <summary>The solution segment of range <paramref name="index"/>.</summary>
    public TheSegment Segment(int index)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        return _sseg.Value(index);
    }

    // Find the vertex of the arc A restriction solutions. It stores
    // Vertex in the list solutions pnt.
    private static void FindVertex(Curve2DAdaptorBase a, TopolTool domain, TheFunction func,
                                   NCollection_Sequence<ThePathPoint> pnt, double toler)
    {
        HVertex vtx;
        double param, valf;
        int itemp;

        domain.Initialize(a);
        domain.InitVertexIterator();
        while (domain.MoreVertex())
        {
            vtx = domain.Vertex();
            param = TheSOBTool.Parameter(vtx, a);

            // Evaluate the function and look compared to tolerance of the
            // Vertex. If distance <= tolerance then add a vertex to the list of solutions.
            // The arc is already assumed in the load function.

            func.Value(param, out valf);
            if (Math.Abs(valf) <= toler)
            {
                itemp = func.GetStateNumber();
                pnt.Append(ThePathPoint.Create(func.Valpoint(itemp), toler, vtx, a, param));
                // Solution is added
            }
            domain.NextVertex();
        }
    }

    private static bool IsDegenerated(CurveOnSurface curve)
    {
        if (curve.GetType() == Circle)
        {
            Circle3D circ = curve.Circle();
            if (circ.Radius <= global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                return true;
            }
        }
        return false;
    }

    private static bool IsDegenerated(in SurfaceQuadric quadric)
    {
        SurfaceKind typeQuad = quadric.TypeQuadric();
        if (typeQuad == SurfaceKind.Cone)
        {
            Cone3D cone = quadric.Cone();
            double semiAngle = Math.Abs(cone.SemiAngle);
            if (semiAngle < 0.02 || semiAngle > 1.55)
            {
                return true;
            }
        }
        return false;
    }

    // std::sort(aSI.begin(), aSI.end()) as MSVC's std::sort does it on up to 32 elements: an insertion sort
    // (_Insertion_sort_unchecked), which keeps tied elements in their order (decision 610).
    private static void SortSolInfo(NCollection_Array1<SolInfo> array)
    {
        int lower = array.Lower(), upper = array.Upper();
        for (int mid = lower + 1; mid <= upper; ++mid)
        {
            SolInfo val = array[mid];
            int hole = mid;
            if (val < array[lower])
            { // found new earliest element, move to front
                for (; hole > lower; --hole)
                {
                    array[hole] = array[hole - 1];
                }
                array[lower] = val;
            }
            else
            { // look for insertion point after first
                for (int prev = hole - 1; val < array[prev]; hole = prev, --prev)
                {
                    array[hole] = array[prev]; // move hole down
                }
                array[hole] = val; // insert element in hole
            }
        }
    }

    // Recherche des points solutions et des bouts d arc solution sur un arc donne.
    // On utilise la fonction math_FunctionAllRoots. Ne convient donc que pour
    // des arcs ayant un point debut et un point de fin (intervalle ferme de
    // parametrage).
    private static void BoundedArc(Curve2DAdaptorBase a, TopolTool domain, double pdeb, double pfin,
                                   TheFunction func, NCollection_Sequence<ThePathPoint> pnt,
                                   NCollection_Sequence<TheSegment> seg, double tolBoundary, double tolTangency,
                                   out bool arcsol, bool recheckOnRegularity)
    {
        int i, nbi = 0, nbp = 0;

        Point3D ptdeb, ptfin;
        double pardeb = 0.0, parfin = 0.0;
        int ideb, ifin, range, ranged, rangef;

        // Creer l echantillonage (math_FunctionSample ou classe heritant)
        // Appel a math_FunctionAllRoots

        //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        //@@@ La Tolerance est asociee a l arc  ( Incoherence avec le cheminement )
        //@@@   ( EpsX ~ 1e-5   et ResolutionU et V ~ 1e-9 )
        //@@@   le vertex trouve ici n'est pas retrouve comme point d arret d une
        //@@@   ligne de cheminement
        //@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
        double epsX = 1.0e-10;

        //  int NbEchant = TheSOBTool::NbSamplesOnArc(A);
        int nbEchant = func.NbSamples();
        if (nbEchant < 100)
        {
            nbEchant = 100; //-- lbr le 22 Avril 96
        }
        //-- Toujours des pbs

        //-- Modif 24  Aout 93 -----------------------------
        double nTolTangency = tolTangency;
        if ((pfin - pdeb) < (tolTangency * 10.0))
        {
            nTolTangency = (pfin - pdeb) * 0.1;
        }
        if (epsX > (nTolTangency + nTolTangency))
        {
            epsX = nTolTangency * 0.1;
        }

        //--------------------------------------------------
        //-- Plante avec un edge avec 2 Samples
        //-- dont les extremites son solutions (f=0)
        //-- et ou la derivee est nulle
        //-- Exemple : un segment diametre d une sphere
        //-- if(NbEchant<3) NbEchant = 3; //-- lbr le 19 Avril 95
        //--------------------------------------------------
        double para = 0, dist, maxdist;

        //-------------------------------------------------------------- REJECTIONS le 15 oct 98
        bool rejection = true;
        double maxdr, maxr, minr, ur, dur;
        minr = RealLast();
        maxr = -minr;
        maxdr = -minr;
        dur = (pfin - pdeb) * 0.2;
        for (i = 1, ur = pdeb; i <= 6; i++)
        {
            if (func.Values(ur, out double f, out double d))
            {
                double lminr, lmaxr;
                if (d < 0.0)
                {
                    d = -d;
                }
                d *= dur + dur;
                if (d > maxdr)
                {
                    maxdr = d;
                }
                lminr = f - d;
                lmaxr = f + d;
                if (lminr < minr)
                {
                    minr = lminr;
                }
                if (lmaxr > maxr)
                {
                    maxr = lmaxr;
                }
                if (minr < 0.0 && maxr > 0.0)
                {
                    rejection = false;
                    break;
                }
            }
            ur += dur;
        }
        if (rejection)
        {
            dur = 0.001 + maxdr + (maxr - minr) * 0.1;
            minr -= dur;
            maxr += dur;
            if (minr < 0.0 && maxr > 0.0)
            {
                rejection = false;
            }
        }

        arcsol = false;

        if (rejection == false)
        {
            SurfaceQuadric quadric = func.Quadric();
            SurfaceKind typeQuad = quadric.TypeQuadric();
            CurveKind typeConS = OtherCurve;

            HInter intCS = new();
            bool isIntCSdone = false;

            FunctionAllRoots? pSol = null;

            FunctionSample echant = new(pdeb, pfin, nbEchant);

            bool aelargir = true;
            // modified by NIZNHY-PKV Thu Apr 12 09:25:19 2001 f
            //
            // maxdist = 100.0*TolBoundary;
            maxdist = tolBoundary + tolTangency;
            //
            // modified by NIZNHY-PKV Thu Apr 12 09:25:23 2001 t
            for (i = 1; i <= nbEchant && aelargir; i++)
            {
                double u = echant.GetParameter(i);
                if (func.Value(u, out dist))
                {
                    if (dist > maxdist || -dist > maxdist)
                    {
                        aelargir = false;
                    }
                }
            }
            if (!(aelargir && maxdist < 0.01))
            {
                maxdist = tolBoundary;
            }

            if (typeQuad != OtherSurface) // intersection of boundary curve and quadric surface
            {
                // Exact solution
                SurfaceAdaptorBase surf = func.Surface();
                CurveOnSurface conS = new(a, surf);
                typeConS = conS.GetType();
                // OCCT_DEBUG: a Geom_Curve of the arc, for debugging only: left out.
                // C++: occ::handle<Adaptor3d_CurveOnSurface> HConS = new Adaptor3d_CurveOnSurface(ConS), a copy of a
                // local that is not used afterwards: the port uses the local.
                CurveOnSurface hConS = conS;
                Surface? quadSurf = null;
                switch (typeQuad)
                {
                    case Plane:
                    {
                        quadSurf = new PlaneSurface(quadric.Plane());
                        break;
                    }
                    case SurfaceKind.Cylinder:
                    {
                        quadSurf = new CylindricalSurface(quadric.Cylinder());
                        break;
                    }
                    case SurfaceKind.Cone:
                    {
                        quadSurf = new ConicalSurface(quadric.Cone());
                        break;
                    }
                    case SurfaceKind.Sphere:
                    {
                        quadSurf = new SphericalSurface(quadric.Sphere());
                        break;
                    }
                    case SurfaceKind.Torus:
                    {
                        quadSurf = new ToroidalSurface(quadric.Torus());
                        break;
                    }
                    default:
                        break;
                }
                SurfaceAdaptor gaHsurf = new(quadSurf!);

                if ((typeConS == Line || typeConS == Circle || typeConS == Ellipse
                     || typeConS == Parabola || typeConS == Hyperbola)
                    && typeQuad != SurfaceKind.Torus && !IsDegenerated(hConS) && !IsDegenerated(quadric))
                {
                    // exact intersection for only canonic curves and real quadric surfaces
                    intCS.Perform(hConS, gaHsurf);
                }

                isIntCSdone = intCS.IsDone;
                if (isIntCSdone)
                {
                    nbp = intCS.PointCount;
                    nbi = intCS.SegmentCount;
                }
                // If we have not got intersection, it may be touch with some tolerance,
                // need to be checked
                if (nbp == 0 && nbi == 0)
                {
                    isIntCSdone = false;
                }

            } // if (TypeQuad != GeomAbs_OtherSurface) - intersection of boundary curve and quadric surface

            if (!isIntCSdone)
            {
                pSol = new FunctionAllRoots(func, echant, epsX, maxdist, maxdist); //-- TolBoundary,nTolTangency);

                if (!pSol.IsDone)
                {
                    throw new FailureException();
                }

                nbp = pSol.PointCount;
            }
            //
            // jgv: build solution on the whole boundary
            if (recheckOnRegularity && nbp > 0 && IsRegularity(a, domain))
            {
                // double theTol = Domain->MaxTolerance(A);
                // theTol += theTol;
                double theTol = 5.0e-4;
                FunctionAllRoots solAgain = new(func, echant, epsX, theTol, theTol); //-- TolBoundary,nTolTangency);

                if (!solAgain.IsDone)
                {
                    throw new FailureException();
                }

                int Nbi_again = solAgain.IntervalCount;

                if (Nbi_again > 0)
                {
                    int nbSamples = 10;
                    double delta = (pfin - pdeb) / nbSamples;
                    double globalTol = theTol * 10;
                    bool solOnBoundary = true;
                    for (i = 0; i <= nbSamples; i++)
                    {
                        double aParam = pdeb + i * delta;
                        func.Value(aParam, out double value);
                        if (Math.Abs(value) > globalTol)
                        {
                            solOnBoundary = false;
                            break;
                        }
                    }

                    if (solOnBoundary)
                    {
                        for (i = 1; i <= Nbi_again; i++)
                        {
                            TheSegment newseg = new();
                            newseg.SetValue(a);
                            // Recuperer point debut et fin, et leur parametre.
                            solAgain.GetInterval(i, out pardeb, out parfin);

                            if (Math.Abs(pardeb - pdeb) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                pardeb = pdeb;
                            }
                            if (Math.Abs(parfin - pfin) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                parfin = pfin;
                            }

                            solAgain.GetIntervalState(i, out ideb, out ifin);

                            ptdeb = func.Valpoint(ideb);
                            ptfin = func.Valpoint(ifin);

                            PointProcess(ptdeb, pardeb, a, domain, pnt, theTol, out ranged);
                            newseg.SetLimitPoint(pnt.Value(ranged), true);
                            PointProcess(ptfin, parfin, a, domain, pnt, theTol, out rangef);
                            newseg.SetLimitPoint(pnt.Value(rangef), false);
                            seg.Append(newseg);
                        }
                        arcsol = true;
                        return;
                    }
                }
            } // if (RecheckOnRegularity && Nbp > 0 && IsRegularity(A, Domain))
            ////////////////////////////////////////////

            //-- detection du cas ou la fonction est quasi tangente et que les
            //-- zeros sont quasi confondus.
            //-- Dans ce cas on prend le point "milieu"
            //-- On suppose que les solutions sont triees.

            if (nbp != 0)
            {
                NCollection_Array1<SolInfo> si = new(1, nbp);

                for (i = 1; i <= nbp; i++)
                {
                    if (isIntCSdone)
                    {
                        si[i].Init(intCS, i);
                    }
                    else
                    {
                        si[i].Init(pSol!, i);
                    }
                }

                SortSolInfo(si);

                // modified by NIZNHY-PKV Wed Mar 21 18:34:18 2001 f
                //////////////////////////////////////////////////////////
                // The treatment of the situation when line(arc) that is
                // tangent to cylinder(domain).
                // We should have only one solution i.e Nbp=1. Ok?
                // But we have 2,3,.. solutions.     That is wrong ersult.
                // The TreatLC(...) function is dedicated to solve the pb.
                //                           PKV Fri Mar 23 12:17:29 2001

                int ip = TreatLC(a, domain, quadric, tolBoundary, pnt);
                if (ip != 0)
                {
                    //////////////////////////////////////////////////////////
                    // modified by NIZNHY-PKV Wed Mar 21 18:34:23 2001 t
                    //
                    //  Using of old usual way proposed by Laurent
                    //
                    for (i = 1; i < nbp; i++)
                    {
                        double parap1 = si[i + 1].Value();
                        para = si[i].Value();

                        double param = (para + parap1) * 0.5;
                        double yf = 0.0;
                        double ym = 0.0;
                        double yl = 0.0;
                        if (func.Value(param, out ym) && Math.Abs(ym) < maxdist)
                        {
                            double sm = Math.CopySign(1.0, ym);
                            bool tang = func.Value(para, out yf) && func.Value(parap1, out yl);
                            if (tang)
                            {
                                // Line can be tangent surface if all distances less then maxdist
                                tang = tang && Math.Abs(yf) < maxdist && Math.Abs(yl) < maxdist;
                            }
                            if (tang && isIntCSdone && typeConS == Line)
                            {
                                // Interval is got by exact intersection
                                // Line can be tangent if all points are on the same side of surface
                                // it means that signs of all distances are the same
                                double sf = Math.CopySign(1.0, yf), sl = Math.CopySign(1.0, yl);
                                tang = tang && (sm == sf) && (sm == sl);
                            }
                            if (tang)
                            {
                                //  Modified by skv - Tue Aug 31 12:13:51 2004 OCC569 Begin
                                // Consider this interval as tangent one. Treat it to find
                                // parameter with the lowest function value.
                                // Compute the number of nodes.
                                double tol = tolBoundary * 1000.0;
                                if (tol > 0.001)
                                {
                                    tol = 0.001;
                                }

                                // fix floating point exception 569, chl-922-e9
                                parap1 = (Math.Abs(parap1) < 1.0e9) ? parap1 : ((parap1 >= 0.0) ? 1.0e9 : -1.0e9);
                                para = (Math.Abs(para) < 1.0e9) ? para : ((para >= 0.0) ? 1.0e9 : -1.0e9);

                                int nbNodes = RealToInt(Math.Ceiling((parap1 - para) / tol));

                                double val = RealLast();
                                double valMax = 0.0;
                                // int aNbNodes = 23;
                                double aDelta = (parap1 - para) / (nbNodes + 1.0);
                                int ii;
                                double curPar;
                                double curVal;

                                for (ii = 0; ii <= nbNodes + 1; ii++)
                                {
                                    curPar = (ii < nbNodes + 1) ? para + ii * aDelta : parap1;

                                    if (func.Value(curPar, out curVal))
                                    {
                                        double absVal = Math.Abs(curVal);
                                        if (absVal < val)
                                        {
                                            val = absVal;
                                            param = curPar;
                                        }
                                        if (absVal > valMax)
                                        {
                                            valMax = absVal;
                                        }
                                    }
                                }
                                // At last, interval got by exact intersection can be considered as tangent if
                                // minimal distance is inside interval and
                                // minimal and maximal values are almost the same
                                if (isIntCSdone && nbNodes > 1)
                                {
                                    tang = Math.Abs(param - para) > epsX && Math.Abs(parap1 - param) > epsX
                                            && 0.01 * valMax <= val;
                                }
                                if (tang)
                                {
                                    si[i].ChangeValue() = pdeb - 1;
                                    si[i + 1].ChangeValue() = param;
                                }
                            }
                        }
                    }

                    for (i = 1; i <= nbp; i++)
                    {
                        para = si[i].Value();
                        if ((para - pdeb) < epsX || (pfin - para) < epsX)
                        {
                            continue;
                        }

                        if (!func.Value(para, out dist))
                        {
                            continue;
                        }

                        dist = Math.Abs(dist);

                        int indx = -1;
                        // const double aParam = Sol->GetPoint(aSI(i).Index());
                        double aParam = si[i].Value();
                        if (dist < maxdist)
                        {
                            if (!isIntCSdone
                                && (Math.Abs(aParam - pdeb) <= global::Kerf.Engine.Foundation.Precision.PConfusion()
                                    || Math.Abs(aParam - pfin) <= global::Kerf.Engine.Foundation.Precision.PConfusion()))
                            {
                                indx = pSol!.GetPointState(si[i].Index());
                            }
                        }

                        Point3D aPnt = indx < 0 ? func.LastComputedPoint() : func.Valpoint(indx);

                        if (dist > 0.1 * global::Kerf.Engine.Foundation.Precision.Confusion())
                        {
                            // Precise found points. It results in following:
                            //   1. Make the vertex nearer to the intersection line
                            //     (see description to issue #27252 in order to
                            //     understand necessity).
                            //   2. Merge two near vertices to single point.

                            // All members in TabSol array has already been sorted in increase order.
                            // Now, we limit precise boundaries in order to avoid changing this order.
                            double fPar = (i == 1) ? pdeb : (para + si[i - 1].Value()) / 2.0;
                            double lPar = (i == nbp) ? pfin : (para + si[i + 1].Value()) / 2.0;

                            MinFunction newFunc = new(func);
                            BrentMinimum min = new(global::Kerf.Engine.Foundation.Precision.Confusion());

                            min.Perform(newFunc, fPar, para, lPar);
                            if (min.IsDone)
                            {
                                para = min.Location();
                                Point2D p2d = a.Value(para);
                                aPnt = func.Surface().Value(p2d.X, p2d.Y);
                            }
                        }

                        PointProcess(aPnt, para, a, domain, pnt, tolBoundary, out range);
                    }
                } // end of if(ip)
            } // end of if(Nbp)

            // Pour chaque intervalle trouve faire
            //   Traiter les extremites comme des points
            //   Ajouter intervalle dans la liste des segments

            if (!isIntCSdone)
            {
                nbi = pSol!.IntervalCount;
            }

            if (!recheckOnRegularity && nbp != 0)
            {
                nbi = 0;
            }

            for (i = 1; i <= nbi; i++)
            {
                TheSegment newseg = new();
                newseg.SetValue(a);
                // Recuperer point debut et fin, et leur parametre.
                if (isIntCSdone)
                {
                    IntersectionSegment intSeg = intCS.Segment(i);
                    CurveSurfaceIntersectionPoint end1 = intSeg.FirstPoint();
                    CurveSurfaceIntersectionPoint end2 = intSeg.SecondPoint();
                    pardeb = end1.W();
                    parfin = end2.W();
                    ptdeb = end1.Pnt();
                    ptfin = end2.Pnt();
                }
                else
                {
                    pSol!.GetInterval(i, out pardeb, out parfin);
                    pSol.GetIntervalState(i, out ideb, out ifin);

                    ptdeb = func.Valpoint(ideb);
                    ptfin = func.Valpoint(ifin);
                }

                PointProcess(ptdeb, pardeb, a, domain, pnt, tolBoundary, out ranged);
                newseg.SetLimitPoint(pnt.Value(ranged), true);
                PointProcess(ptfin, parfin, a, domain, pnt, tolBoundary, out rangef);
                newseg.SetLimitPoint(pnt.Value(rangef), false);
                seg.Append(newseg);
            }

            if (nbi == 1)
            {
                if ((Math.Abs(pardeb - pdeb) < global::Kerf.Engine.Foundation.Precision.PConfusion()) && (Math.Abs(parfin - pfin) < global::Kerf.Engine.Foundation.Precision.PConfusion()))
                {
                    arcsol = true;
                }
            }
        }
    }

    // - PROVISIONAL - TEMPORARY - NOT GOOD - NYI - TO DO
    // - Temporary - temporary - not good - nyi - to do
    //
    // - We are looking for parameters for start and end of the arc (2d curve)
    // - Infinity, a way to intersect the quadric with a portion of arc
    // - Finished.
    //
    // - The quadric is a plane, a cylinder, a cone and a sphere.
    // - Idea: We take any point on the arc and the fact grow
    // - Terminals to the signed distance function values or is likely
    // - S cancel.
    //
    // - WARNING: The following calculations provide a very estimated coarse parameters.
    // - This avoids the raises and allows a case of Boxes
    // - Inifinies walk. It will take this code
    // - With curve surface intersections.
    private static void ComputeBoundsfromInfinite(TheFunction func, ref double pDeb, ref double pFin, out int nbEchant)
    {
        nbEchant = 100;

        double u0 = 0.0;
        double dU = 0.001;
        double dist0, dist1;

        func.Value(u0, out dist0);
        func.Value(u0 + dU, out dist1);
        double dDist = dist1 - dist0;
        if (dDist != 0)
        {
            u0 -= dU * dist0 / dDist;
            pDeb = pFin = u0;
            double umin = u0 - 1e5;
            func.Value(umin, out dist0);
            func.Value(umin + dU, out dist1);
            dDist = dist1 - dist0;
            if (dDist != 0)
            {
                umin -= dU * dist0 / dDist;
            }
            else
            {
                umin -= 10.0;
            }
            double umax = u0 + 1e8;
            func.Value(umax, out dist0);
            func.Value(umax + dU, out dist1);
            dDist = dist1 - dist0;
            if (dDist != 0)
            {
                umax -= dU * dist0 / dDist;
            }
            else
            {
                umax += 10.0;
            }
            if (umin > u0)
            {
                umin = u0 - 10.0;
            }
            if (umax < u0)
            {
                umax = u0 + 10.0;
            }

            pFin = umax + 10.0 * (umax - umin);
            pDeb = umin - 10.0 * (umax - umin);
        }
        else
        {
            //-- Possibilite de Arc totalement inclu ds Quad
            pDeb = 1e10;
            pFin = -1e10;
        }
    }

    // Check to see if a solution point is coincident with a vertex.
    // If confused, you should find this vertex in the list of
    // Start. It then returns the position of this point in the list pnt.
    // Otherwise, add the point in the list.
    private static void PointProcess(in Point3D pt, double para, Curve2DAdaptorBase a, TopolTool domain,
                                     NCollection_Sequence<ThePathPoint> pnt, double tol, out int range)
    {
        int k;
        bool found, goon;
        double dist, toler;

        range = 0; // C++ leaves Range unwritten when no case below applies (never: a point is always found or added)
        int nbsol = pnt.Length();
        HVertex vtx;
        ThePathPoint ptsol = new();

        domain.Initialize(a);
        domain.InitVertexIterator();
        found = false;
        goon = domain.MoreVertex();
        while (goon)
        {
            vtx = domain.Vertex();
            dist = Math.Abs(para - TheSOBTool.Parameter(vtx, a));
            toler = TheSOBTool.Tolerance(vtx, a);
            // OCCT_DEBUG: a warning on vertex tolerances above 0.1, which also clamps those above 10000 to 1e-7 in
            // debug builds only: left out, as in OCCT's release builds.

            if (dist <= toler)
            {
                // Locate the vertex in the list of solutions
                k = 1;
                found = (k > nbsol);
                while (!found)
                {
                    ptsol = pnt.Value(k);
                    if (!ptsol.IsNew())
                    {
                        // jag 940608  if (ptsol.Vertex() == vtx && ptsol.Arc()    == A) {
                        if (domain.Identical(ptsol.Vertex(), vtx) && ReferenceEquals(ptsol.Arc(), a)
                            && Math.Abs(ptsol.Parameter() - para) <= toler)
                        {
                            found = true;
                        }
                        else
                        {
                            k = k + 1;
                            found = (k > nbsol);
                        }
                    }
                    else
                    {
                        k = k + 1;
                        found = (k > nbsol);
                    }
                }
                if (k <= nbsol)
                { // We find the vertex
                    range = k;
                }
                else
                { // Otherwise
                    ptsol.SetValue(pt, tol, vtx, a, para);
                    pnt.Append(ptsol);
                    range = pnt.Length();
                }
                found = true;
                goon = false;
            }
            else
            {
                domain.NextVertex();
                goon = domain.MoreVertex();
            }
        }

        if (!found)
        { // No one is falling on a vertex
            // jgv: do not add segment's extremities if they already exist
            bool found_internal = false;
            for (k = 1; k <= pnt.Length(); k++)
            {
                ptsol = pnt.Value(k);
                if (!ReferenceEquals(ptsol.Arc(), a) || !ptsol.IsNew()) // vertex
                {
                    continue;
                }
                if (Math.Abs(ptsol.Parameter() - para) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    found_internal = true;
                    range = k;
                }
            }
            /////////////////////////////////////////////////////////////

            if (!found_internal)
            {
                double TOL = tol;
                TOL *= 1000.0;
                // if(TOL>0.001) TOL=0.001;
                if (TOL > 0.005)
                {
                    TOL = 0.005; // #24643
                }

                ptsol.SetValue(pt, TOL, a, para);
                pnt.Append(ptsol);
                range = pnt.Length();
            }
        }
    }

    private static bool IsRegularity(Curve2DAdaptorBase a, TopolTool aDomain)
    {
        object? eAddress = aDomain.Edge();
        if (eAddress == null)
        {
            return false;
        }

        Edge e = (Edge)eAddress;

        return (Tool.HasContinuity(e));
    }

    private static int TreatLC(Curve2DAdaptorBase a, TopolTool aDomain, in SurfaceQuadric aQuadric,
                               double tolBoundary, NCollection_Sequence<ThePathPoint> pnt)
    {
        int exitCode = 1, nbExt;

        object? eAddress = aDomain.Edge();
        if (eAddress == null)
        {
            return exitCode;
        }

        Edge e = (Edge)eAddress;

        if (Tool.Degenerated(e))
        {
            return exitCode;
        }

        CurveKind typeE;
        EdgeCurveAdaptor bac = new(e);
        typeE = bac.GetType();

        if (typeE != Line)
        {
            return exitCode;
        }

        SurfaceKind typeS;
        typeS = aQuadric.TypeQuadric();

        if (typeS != SurfaceKind.Cylinder)
        {
            return exitCode;
        }

        double f, l, u1f, u1l, u2f, u2l, uEgde, TOL, dist, r, rRel, tol;
        Curve3D cEdge = Tool.Curve(e, out f, out l)!;

        Cylinder3D cyl = aQuadric.Cylinder();
        Axis3D ax1 = cyl.Axis;
        Line3D lin = new(ax1);
        LineCurve3D cAxis = new(lin);
        r = cyl.Radius;

        u1f = cAxis.FirstParameter;
        u1l = cAxis.LastParameter;

        u2f = cEdge.FirstParameter;
        u2l = cEdge.LastParameter;

        CurveAdaptor c1 = new(), c2 = new();

        c1.Load(cAxis);
        c2.Load(cEdge);

        tol = global::Kerf.Engine.Foundation.Precision.PConfusion();

        ExtCC extCC = new(c1, c2, u1f, u1l, u2f, u2l, tol, tol);

        nbExt = extCC.ExtCount;
        if (nbExt != 1)
        {
            return exitCode;
        }

        Point3D p1, pEdge;

        extCC.Points(1, out POnCurv pc1, out POnCurv pc2);

        p1 = pc1.Value();
        pEdge = pc2.Value();

        uEgde = pc2.Parameter();

        dist = pEdge.DistanceTo(p1);
        rRel = Math.Abs(dist - r) / r;
        if (rRel > tolBoundary)
        {
            return exitCode;
        }

        if (uEgde < (f + tolBoundary) || uEgde > (l - tolBoundary))
        {
            return exitCode;
        }
        //
        // Do not wonder !
        // It was done as into PointProcess(...) function
        // printf("TreatLC()=> tangent line is found\n");
        TOL = 1000.0 * tolBoundary;
        if (TOL > 0.001)
        {
            TOL = 0.001;
        }

        ThePathPoint ptsol = new();
        ptsol.SetValue(pEdge, TOL, a, uEgde);
        pnt.Append(ptsol);

        exitCode = 0;
        return exitCode;
    }
}
