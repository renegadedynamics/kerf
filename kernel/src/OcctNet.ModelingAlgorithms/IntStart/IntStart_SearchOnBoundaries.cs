// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntStart/IntStart_SearchOnBoundaries.gxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntStart/IntStart_SearchOnBoundaries.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// What the boundary search reads from its function (OCCT's TheFunction macro: IntPatch_ArcFunction,
/// Contap_ArcFunction): the function along the current arc, its samples, the points saved by
/// <c>GetStateNumber</c>, and the quadric and the surface it is built on.
/// </summary>
public interface IIntStart_ArcFunction
{
    /// <summary>Loads the arc to search.</summary>
    void Set(Adaptor2d_Curve2d A);

    /// <summary>The number of samples on the arc.</summary>
    int NbSamples();

    /// <summary>The point saved at state <paramref name="Index"/>.</summary>
    gp_Pnt Valpoint(int Index);

    /// <summary>The quadric.</summary>
    IntSurf_Quadric Quadric();

    /// <summary>The surface the arcs are on.</summary>
    Adaptor3d_Surface Surface();

    /// <summary>The point of the last evaluation.</summary>
    gp_Pnt LastComputedPoint();
}

/// <summary>
/// The static tool of the boundary search (OCCT's TheSOBTool macro: IntPatch_HInterTool, Contap_HContTool): the
/// vertices' parameters and tolerances on the arcs, the arcs' bounds, and the points and segments already known on an
/// arc.
/// </summary>
public interface IIntStart_SOBTool
{
    static abstract bool HasBeenSeen(Adaptor2d_Curve2d C);

    static abstract double Tolerance(Adaptor3d_HVertex V, Adaptor2d_Curve2d C);

    static abstract double Parameter(Adaptor3d_HVertex V, Adaptor2d_Curve2d C);

    static abstract void Bounds(Adaptor2d_Curve2d C, out double Ufirst, out double Ulast);

    static abstract int NbPoints(Adaptor2d_Curve2d C);

    static abstract void Value(Adaptor2d_Curve2d C, int Index, out gp_Pnt Pt, out double Tol, out double U);

    static abstract bool IsVertex(Adaptor2d_Curve2d C, int Index);

    static abstract void Vertex(Adaptor2d_Curve2d C, int Index, out Adaptor3d_HVertex V);

    static abstract int NbSegments(Adaptor2d_Curve2d C);

    static abstract bool HasFirstPoint(Adaptor2d_Curve2d C, int Index, out int IndFirst);

    static abstract bool HasLastPoint(Adaptor2d_Curve2d C, int Index, out int IndLast);

    static abstract bool IsAllSolution(Adaptor2d_Curve2d C);
}

/// <summary>
/// A solution point of the boundary search (OCCT's IntStart_ThePathPoint macro: IntPatch_ThePathPointOfTheSOnBounds,
/// Contap_ThePathPointOfTheSearch): a struct, built and set as OCCT does.
/// </summary>
/// <typeparam name="TSelf">The implementing struct.</typeparam>
public interface IIntStart_PathPoint<TSelf>
    where TSelf : struct, IIntStart_PathPoint<TSelf>
{
    /// <summary>A point on a vertex of an arc.</summary>
    static abstract TSelf Create(in gp_Pnt P, double Tol, Adaptor3d_HVertex V, Adaptor2d_Curve2d A, double Parameter);

    /// <summary>A new point on an arc.</summary>
    static abstract TSelf Create(in gp_Pnt P, double Tol, Adaptor2d_Curve2d A, double Parameter);

    /// <summary>Sets a point on a vertex of an arc.</summary>
    void SetValue(in gp_Pnt P, double Tol, Adaptor3d_HVertex V, Adaptor2d_Curve2d A, double Parameter);

    /// <summary>Sets a new point on an arc (the vertex is kept, as in OCCT).</summary>
    void SetValue(in gp_Pnt P, double Tol, Adaptor2d_Curve2d A, double Parameter);

    bool IsNew();

    Adaptor3d_HVertex Vertex();

    Adaptor2d_Curve2d Arc();

    double Parameter();
}

/// <summary>
/// A solution segment of the boundary search (OCCT's IntStart_TheSegment macro: IntPatch_TheSegmentOfTheSOnBounds,
/// Contap_TheSegmentOfTheSearch): a struct.
/// </summary>
public interface IIntStart_Segment<ThePathPoint>
{
    /// <summary>Sets the arc (and forgets the limit points).</summary>
    void SetValue(Adaptor2d_Curve2d A);

    /// <summary>Sets the first or the last point.</summary>
    void SetLimitPoint(in ThePathPoint V, bool First);
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
public class IntStart_SearchOnBoundaries<TheSOBTool, TheFunction, ThePathPoint, TheSegment>
    where TheSOBTool : IIntStart_SOBTool
    where TheFunction : math_FunctionWithDerivative, IIntStart_ArcFunction
    where ThePathPoint : struct, IIntStart_PathPoint<ThePathPoint>
    where TheSegment : struct, IIntStart_Segment<ThePathPoint>
{
    private bool done;
    private bool all;
    private readonly NCollection_Sequence<TheSegment> sseg = new();
    private readonly NCollection_Sequence<ThePathPoint> spnt = new();

    /// <summary>The function squared, to minimise near a root (OCCT's file-local MinFunction).</summary>
    private sealed class MinFunction : math_Function
    {
        private readonly TheFunction myFunc;

        public MinFunction(TheFunction theFunc) => myFunc = theFunc;

        // returns value of the one-dimension-function when parameter
        // is equal to theX
        public override bool Value(double theX, out double theFVal)
        {
            if (!myFunc.Value(theX, out theFVal))
            {
                return false;
            }

            theFVal *= theFVal;
            return true;
        }

        // see analogical method for abstract owner class math_Function
        public override int GetStateNumber() => 0;
    }

    /// <summary>A root on an arc: its parameter and its rank in the solver (OCCT's file-local SolInfo).</summary>
    private struct SolInfo
    {
        private int myMathIndex;
        private double myValue;

        public SolInfo()
        {
            myMathIndex = -1;
            myValue = RealLast();
        }

        public void Init(math_FunctionAllRoots theSolution, int theIndex)
        {
            myMathIndex = theIndex;
            myValue = theSolution.GetPoint(theIndex);
        }

        public void Init(IntCurveSurface_HInter theSolution, int theIndex)
        {
            myMathIndex = theIndex;
            myValue = theSolution.Point(theIndex).W();
        }

        public readonly double Value() => myValue;

        public readonly int Index() => myMathIndex;

        public static bool operator <(SolInfo theLeft, SolInfo theRight) => theLeft.myValue < theRight.myValue;

        public static bool operator >(SolInfo theLeft, SolInfo theRight) => theLeft.myValue > theRight.myValue;

        [System.Diagnostics.CodeAnalysis.UnscopedRef]
        public ref double ChangeValue() => ref myValue;
    }

    /// <summary>Empty constructor.</summary>
    public IntStart_SearchOnBoundaries()
    {
        done = false;
        all = false;
    }

    /// <summary>
    /// Searches the points and parts of the arcs of <paramref name="Domain"/> where F = 0.
    /// <paramref name="TolBoundary"/> defines whether a curve is on the quadric, <paramref name="TolTangency"/> whether a
    /// point is; with <paramref name="RecheckOnRegularity"/>, a whole arc on a regular edge is searched as a segment.
    /// </summary>
    public void Perform(TheFunction Func, Adaptor3d_TopolTool Domain, double TolBoundary, double TolTangency,
                        bool RecheckOnRegularity = false)
    {
        done = false;
        spnt.Clear();
        sseg.Clear();

        bool Arcsol;
        double PDeb, PFin, prm, tol;
        int i, nbknown, nbfound, index;
        gp_Pnt pt;

        Domain.Init();

        if (Domain.More())
        {
            all = true;
        }
        else
        {
            all = false;
        }

        while (Domain.More())
        {
            Adaptor2d_Curve2d A = Domain.Value();
            if (!TheSOBTool.HasBeenSeen(A))
            {
                Func.Set(A);
                FindVertex(A, Domain, Func, spnt, TolBoundary);
                TheSOBTool.Bounds(A, out PDeb, out PFin);
                if (Precision.IsNegativeInfinite(PDeb) || Precision.IsPositiveInfinite(PFin))
                {
                    ComputeBoundsfromInfinite(Func, ref PDeb, ref PFin, out _);
                }
                BoundedArc(A, Domain, PDeb, PFin, Func, spnt, sseg, TolBoundary, TolTangency, out Arcsol,
                           RecheckOnRegularity);
                all = (all && Arcsol);
            }

            else
            {
                // as it seems we'll never be here, because
                // TheSOBTool::HasBeenSeen(A) always returns FALSE
                nbfound = spnt.Length();

                // On recupere les points connus
                nbknown = TheSOBTool.NbPoints(A);
                for (i = 1; i <= nbknown; i++)
                {
                    TheSOBTool.Value(A, i, out pt, out tol, out prm);
                    if (TheSOBTool.IsVertex(A, i))
                    {
                        TheSOBTool.Vertex(A, i, out Adaptor3d_HVertex vtx);
                        spnt.Append(ThePathPoint.Create(pt, tol, vtx, A, prm));
                    }
                    else
                    {
                        spnt.Append(ThePathPoint.Create(pt, tol, A, prm));
                    }
                }
                // On recupere les arcs solutions
                nbknown = TheSOBTool.NbSegments(A);
                for (i = 1; i <= nbknown; i++)
                {
                    TheSegment newseg = new();
                    newseg.SetValue(A);
                    if (TheSOBTool.HasFirstPoint(A, i, out index))
                    {
                        newseg.SetLimitPoint(spnt.Value(nbfound + index), true);
                    }
                    if (TheSOBTool.HasLastPoint(A, i, out index))
                    {
                        newseg.SetLimitPoint(spnt.Value(nbfound + index), false);
                    }
                    sseg.Append(newseg);
                }
                all = (all & TheSOBTool.IsAllSolution(A));
            }
            Domain.Next();
        }
        done = true;
    }

    /// <summary>True if the computation succeeded.</summary>
    public bool IsDone() => done;

    /// <summary>True if every arc of the domain is a solution (inside the quadric). Throws StdFail_NotDone if not done.</summary>
    public bool AllArcSolution()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return all;
    }

    /// <summary>The number of solution points. Throws StdFail_NotDone if not done.</summary>
    public int NbPoints()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return spnt.Length();
    }

    /// <summary>The solution point of range <paramref name="Index"/>.</summary>
    public ThePathPoint Point(int Index)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return spnt.Value(Index);
    }

    /// <summary>The number of solution segments. Throws StdFail_NotDone if not done.</summary>
    public int NbSegments()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return sseg.Length();
    }

    /// <summary>The solution segment of range <paramref name="Index"/>.</summary>
    public TheSegment Segment(int Index)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return sseg.Value(Index);
    }

    // Find the vertex of the arc A restriction solutions. It stores
    // Vertex in the list solutions pnt.
    private static void FindVertex(Adaptor2d_Curve2d A, Adaptor3d_TopolTool Domain, TheFunction Func,
                                   NCollection_Sequence<ThePathPoint> pnt, double Toler)
    {
        Adaptor3d_HVertex vtx;
        double param, valf;
        int itemp;

        Domain.Initialize(A);
        Domain.InitVertexIterator();
        while (Domain.MoreVertex())
        {
            vtx = Domain.Vertex();
            param = TheSOBTool.Parameter(vtx, A);

            // Evaluate the function and look compared to tolerance of the
            // Vertex. If distance <= tolerance then add a vertex to the list of solutions.
            // The arc is already assumed in the load function.

            Func.Value(param, out valf);
            if (Math.Abs(valf) <= Toler)
            {
                itemp = Func.GetStateNumber();
                pnt.Append(ThePathPoint.Create(Func.Valpoint(itemp), Toler, vtx, A, param));
                // Solution is added
            }
            Domain.NextVertex();
        }
    }

    private static bool IsDegenerated(Adaptor3d_CurveOnSurface theCurve)
    {
        if (theCurve.GetType() == GeomAbs_Circle)
        {
            gp_Circ aCirc = theCurve.Circle();
            if (aCirc.Radius() <= Precision.Confusion())
            {
                return true;
            }
        }
        return false;
    }

    private static bool IsDegenerated(in IntSurf_Quadric theQuadric)
    {
        GeomAbs_SurfaceType TypeQuad = theQuadric.TypeQuadric();
        if (TypeQuad == GeomAbs_Cone)
        {
            gp_Cone aCone = theQuadric.Cone();
            double aSemiAngle = Math.Abs(aCone.SemiAngle());
            if (aSemiAngle < 0.02 || aSemiAngle > 1.55)
            {
                return true;
            }
        }
        return false;
    }

    // std::sort(aSI.begin(), aSI.end()) as MSVC's std::sort does it on up to 32 elements: an insertion sort
    // (_Insertion_sort_unchecked), which keeps tied elements in their order (decision 610).
    private static void SortSolInfo(NCollection_Array1<SolInfo> theArray)
    {
        int aLower = theArray.Lower(), aUpper = theArray.Upper();
        for (int aMid = aLower + 1; aMid <= aUpper; ++aMid)
        {
            SolInfo aVal = theArray[aMid];
            int aHole = aMid;
            if (aVal < theArray[aLower])
            { // found new earliest element, move to front
                for (; aHole > aLower; --aHole)
                {
                    theArray[aHole] = theArray[aHole - 1];
                }
                theArray[aLower] = aVal;
            }
            else
            { // look for insertion point after first
                for (int aPrev = aHole - 1; aVal < theArray[aPrev]; aHole = aPrev, --aPrev)
                {
                    theArray[aHole] = theArray[aPrev]; // move hole down
                }
                theArray[aHole] = aVal; // insert element in hole
            }
        }
    }

    // Recherche des points solutions et des bouts d arc solution sur un arc donne.
    // On utilise la fonction math_FunctionAllRoots. Ne convient donc que pour
    // des arcs ayant un point debut et un point de fin (intervalle ferme de
    // parametrage).
    private static void BoundedArc(Adaptor2d_Curve2d A, Adaptor3d_TopolTool Domain, double Pdeb, double Pfin,
                                   TheFunction Func, NCollection_Sequence<ThePathPoint> pnt,
                                   NCollection_Sequence<TheSegment> seg, double TolBoundary, double TolTangency,
                                   out bool Arcsol, bool RecheckOnRegularity)
    {
        int i, Nbi = 0, Nbp = 0;

        gp_Pnt ptdeb, ptfin;
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
        double EpsX = 1.0e-10;

        //  int NbEchant = TheSOBTool::NbSamplesOnArc(A);
        int NbEchant = Func.NbSamples();
        if (NbEchant < 100)
        {
            NbEchant = 100; //-- lbr le 22 Avril 96
        }
        //-- Toujours des pbs

        //-- Modif 24  Aout 93 -----------------------------
        double nTolTangency = TolTangency;
        if ((Pfin - Pdeb) < (TolTangency * 10.0))
        {
            nTolTangency = (Pfin - Pdeb) * 0.1;
        }
        if (EpsX > (nTolTangency + nTolTangency))
        {
            EpsX = nTolTangency * 0.1;
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
        bool Rejection = true;
        double maxdr, maxr, minr, ur, dur;
        minr = RealLast();
        maxr = -minr;
        maxdr = -minr;
        dur = (Pfin - Pdeb) * 0.2;
        for (i = 1, ur = Pdeb; i <= 6; i++)
        {
            if (Func.Values(ur, out double F, out double D))
            {
                double lminr, lmaxr;
                if (D < 0.0)
                {
                    D = -D;
                }
                D *= dur + dur;
                if (D > maxdr)
                {
                    maxdr = D;
                }
                lminr = F - D;
                lmaxr = F + D;
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
                    Rejection = false;
                    break;
                }
            }
            ur += dur;
        }
        if (Rejection)
        {
            dur = 0.001 + maxdr + (maxr - minr) * 0.1;
            minr -= dur;
            maxr += dur;
            if (minr < 0.0 && maxr > 0.0)
            {
                Rejection = false;
            }
        }

        Arcsol = false;

        if (Rejection == false)
        {
            IntSurf_Quadric aQuadric = Func.Quadric();
            GeomAbs_SurfaceType TypeQuad = aQuadric.TypeQuadric();
            GeomAbs_CurveType TypeConS = GeomAbs_OtherCurve;

            IntCurveSurface_HInter IntCS = new();
            bool IsIntCSdone = false;

            math_FunctionAllRoots? pSol = null;

            math_FunctionSample Echant = new(Pdeb, Pfin, NbEchant);

            bool aelargir = true;
            // modified by NIZNHY-PKV Thu Apr 12 09:25:19 2001 f
            //
            // maxdist = 100.0*TolBoundary;
            maxdist = TolBoundary + TolTangency;
            //
            // modified by NIZNHY-PKV Thu Apr 12 09:25:23 2001 t
            for (i = 1; i <= NbEchant && aelargir; i++)
            {
                double u = Echant.GetParameter(i);
                if (Func.Value(u, out dist))
                {
                    if (dist > maxdist || -dist > maxdist)
                    {
                        aelargir = false;
                    }
                }
            }
            if (!(aelargir && maxdist < 0.01))
            {
                maxdist = TolBoundary;
            }

            if (TypeQuad != GeomAbs_OtherSurface) // intersection of boundary curve and quadric surface
            {
                // Exact solution
                Adaptor3d_Surface aSurf = Func.Surface();
                Adaptor3d_CurveOnSurface ConS = new(A, aSurf);
                TypeConS = ConS.GetType();
                // OCCT_DEBUG: a Geom_Curve of the arc, for debugging only: left out.
                // C++: occ::handle<Adaptor3d_CurveOnSurface> HConS = new Adaptor3d_CurveOnSurface(ConS), a copy of a
                // local that is not used afterwards: the port uses the local.
                Adaptor3d_CurveOnSurface HConS = ConS;
                Geom_Surface? QuadSurf = null;
                switch (TypeQuad)
                {
                    case GeomAbs_Plane:
                    {
                        QuadSurf = new Geom_Plane(aQuadric.Plane());
                        break;
                    }
                    case GeomAbs_Cylinder:
                    {
                        QuadSurf = new Geom_CylindricalSurface(aQuadric.Cylinder());
                        break;
                    }
                    case GeomAbs_Cone:
                    {
                        QuadSurf = new Geom_ConicalSurface(aQuadric.Cone());
                        break;
                    }
                    case GeomAbs_Sphere:
                    {
                        QuadSurf = new Geom_SphericalSurface(aQuadric.Sphere());
                        break;
                    }
                    case GeomAbs_Torus:
                    {
                        QuadSurf = new Geom_ToroidalSurface(aQuadric.Torus());
                        break;
                    }
                    default:
                        break;
                }
                GeomAdaptor_Surface GAHsurf = new(QuadSurf!);

                if ((TypeConS == GeomAbs_Line || TypeConS == GeomAbs_Circle || TypeConS == GeomAbs_Ellipse
                     || TypeConS == GeomAbs_Parabola || TypeConS == GeomAbs_Hyperbola)
                    && TypeQuad != GeomAbs_Torus && !IsDegenerated(HConS) && !IsDegenerated(aQuadric))
                {
                    // exact intersection for only canonic curves and real quadric surfaces
                    IntCS.Perform(HConS, GAHsurf);
                }

                IsIntCSdone = IntCS.IsDone();
                if (IsIntCSdone)
                {
                    Nbp = IntCS.NbPoints();
                    Nbi = IntCS.NbSegments();
                }
                // If we have not got intersection, it may be touch with some tolerance,
                // need to be checked
                if (Nbp == 0 && Nbi == 0)
                {
                    IsIntCSdone = false;
                }

            } // if (TypeQuad != GeomAbs_OtherSurface) - intersection of boundary curve and quadric surface

            if (!IsIntCSdone)
            {
                pSol = new math_FunctionAllRoots(Func, Echant, EpsX, maxdist, maxdist); //-- TolBoundary,nTolTangency);

                if (!pSol.IsDone())
                {
                    throw new Standard_Failure();
                }

                Nbp = pSol.NbPoints();
            }
            //
            // jgv: build solution on the whole boundary
            if (RecheckOnRegularity && Nbp > 0 && IsRegularity(A, Domain))
            {
                // double theTol = Domain->MaxTolerance(A);
                // theTol += theTol;
                double theTol = 5.0e-4;
                math_FunctionAllRoots SolAgain = new(Func, Echant, EpsX, theTol, theTol); //-- TolBoundary,nTolTangency);

                if (!SolAgain.IsDone())
                {
                    throw new Standard_Failure();
                }

                int Nbi_again = SolAgain.NbIntervals();

                if (Nbi_again > 0)
                {
                    int NbSamples = 10;
                    double delta = (Pfin - Pdeb) / NbSamples;
                    double GlobalTol = theTol * 10;
                    bool SolOnBoundary = true;
                    for (i = 0; i <= NbSamples; i++)
                    {
                        double aParam = Pdeb + i * delta;
                        Func.Value(aParam, out double aValue);
                        if (Math.Abs(aValue) > GlobalTol)
                        {
                            SolOnBoundary = false;
                            break;
                        }
                    }

                    if (SolOnBoundary)
                    {
                        for (i = 1; i <= Nbi_again; i++)
                        {
                            TheSegment newseg = new();
                            newseg.SetValue(A);
                            // Recuperer point debut et fin, et leur parametre.
                            SolAgain.GetInterval(i, out pardeb, out parfin);

                            if (Math.Abs(pardeb - Pdeb) <= Precision.PConfusion())
                            {
                                pardeb = Pdeb;
                            }
                            if (Math.Abs(parfin - Pfin) <= Precision.PConfusion())
                            {
                                parfin = Pfin;
                            }

                            SolAgain.GetIntervalState(i, out ideb, out ifin);

                            ptdeb = Func.Valpoint(ideb);
                            ptfin = Func.Valpoint(ifin);

                            PointProcess(ptdeb, pardeb, A, Domain, pnt, theTol, out ranged);
                            newseg.SetLimitPoint(pnt.Value(ranged), true);
                            PointProcess(ptfin, parfin, A, Domain, pnt, theTol, out rangef);
                            newseg.SetLimitPoint(pnt.Value(rangef), false);
                            seg.Append(newseg);
                        }
                        Arcsol = true;
                        return;
                    }
                }
            } // if (RecheckOnRegularity && Nbp > 0 && IsRegularity(A, Domain))
            ////////////////////////////////////////////

            //-- detection du cas ou la fonction est quasi tangente et que les
            //-- zeros sont quasi confondus.
            //-- Dans ce cas on prend le point "milieu"
            //-- On suppose que les solutions sont triees.

            if (Nbp != 0)
            {
                NCollection_Array1<SolInfo> aSI = new(1, Nbp);

                for (i = 1; i <= Nbp; i++)
                {
                    if (IsIntCSdone)
                    {
                        aSI[i].Init(IntCS, i);
                    }
                    else
                    {
                        aSI[i].Init(pSol!, i);
                    }
                }

                SortSolInfo(aSI);

                // modified by NIZNHY-PKV Wed Mar 21 18:34:18 2001 f
                //////////////////////////////////////////////////////////
                // The treatment of the situation when line(arc) that is
                // tangent to cylinder(domain).
                // We should have only one solution i.e Nbp=1. Ok?
                // But we have 2,3,.. solutions.     That is wrong ersult.
                // The TreatLC(...) function is dedicated to solve the pb.
                //                           PKV Fri Mar 23 12:17:29 2001

                int ip = TreatLC(A, Domain, aQuadric, TolBoundary, pnt);
                if (ip != 0)
                {
                    //////////////////////////////////////////////////////////
                    // modified by NIZNHY-PKV Wed Mar 21 18:34:23 2001 t
                    //
                    //  Using of old usual way proposed by Laurent
                    //
                    for (i = 1; i < Nbp; i++)
                    {
                        double parap1 = aSI[i + 1].Value();
                        para = aSI[i].Value();

                        double param = (para + parap1) * 0.5;
                        double yf = 0.0;
                        double ym = 0.0;
                        double yl = 0.0;
                        if (Func.Value(param, out ym) && Math.Abs(ym) < maxdist)
                        {
                            double sm = Math.CopySign(1.0, ym);
                            bool aTang = Func.Value(para, out yf) && Func.Value(parap1, out yl);
                            if (aTang)
                            {
                                // Line can be tangent surface if all distances less then maxdist
                                aTang = aTang && Math.Abs(yf) < maxdist && Math.Abs(yl) < maxdist;
                            }
                            if (aTang && IsIntCSdone && TypeConS == GeomAbs_Line)
                            {
                                // Interval is got by exact intersection
                                // Line can be tangent if all points are on the same side of surface
                                // it means that signs of all distances are the same
                                double sf = Math.CopySign(1.0, yf), sl = Math.CopySign(1.0, yl);
                                aTang = aTang && (sm == sf) && (sm == sl);
                            }
                            if (aTang)
                            {
                                //  Modified by skv - Tue Aug 31 12:13:51 2004 OCC569 Begin
                                // Consider this interval as tangent one. Treat it to find
                                // parameter with the lowest function value.
                                // Compute the number of nodes.
                                double aTol = TolBoundary * 1000.0;
                                if (aTol > 0.001)
                                {
                                    aTol = 0.001;
                                }

                                // fix floating point exception 569, chl-922-e9
                                parap1 = (Math.Abs(parap1) < 1.0e9) ? parap1 : ((parap1 >= 0.0) ? 1.0e9 : -1.0e9);
                                para = (Math.Abs(para) < 1.0e9) ? para : ((para >= 0.0) ? 1.0e9 : -1.0e9);

                                int aNbNodes = RealToInt(Math.Ceiling((parap1 - para) / aTol));

                                double aVal = RealLast();
                                double aValMax = 0.0;
                                // int aNbNodes = 23;
                                double aDelta = (parap1 - para) / (aNbNodes + 1.0);
                                int ii;
                                double aCurPar;
                                double aCurVal;

                                for (ii = 0; ii <= aNbNodes + 1; ii++)
                                {
                                    aCurPar = (ii < aNbNodes + 1) ? para + ii * aDelta : parap1;

                                    if (Func.Value(aCurPar, out aCurVal))
                                    {
                                        double anAbsVal = Math.Abs(aCurVal);
                                        if (anAbsVal < aVal)
                                        {
                                            aVal = anAbsVal;
                                            param = aCurPar;
                                        }
                                        if (anAbsVal > aValMax)
                                        {
                                            aValMax = anAbsVal;
                                        }
                                    }
                                }
                                // At last, interval got by exact intersection can be considered as tangent if
                                // minimal distance is inside interval and
                                // minimal and maximal values are almost the same
                                if (IsIntCSdone && aNbNodes > 1)
                                {
                                    aTang = Math.Abs(param - para) > EpsX && Math.Abs(parap1 - param) > EpsX
                                            && 0.01 * aValMax <= aVal;
                                }
                                if (aTang)
                                {
                                    aSI[i].ChangeValue() = Pdeb - 1;
                                    aSI[i + 1].ChangeValue() = param;
                                }
                            }
                        }
                    }

                    for (i = 1; i <= Nbp; i++)
                    {
                        para = aSI[i].Value();
                        if ((para - Pdeb) < EpsX || (Pfin - para) < EpsX)
                        {
                            continue;
                        }

                        if (!Func.Value(para, out dist))
                        {
                            continue;
                        }

                        dist = Math.Abs(dist);

                        int anIndx = -1;
                        // const double aParam = Sol->GetPoint(aSI(i).Index());
                        double aParam = aSI[i].Value();
                        if (dist < maxdist)
                        {
                            if (!IsIntCSdone
                                && (Math.Abs(aParam - Pdeb) <= Precision.PConfusion()
                                    || Math.Abs(aParam - Pfin) <= Precision.PConfusion()))
                            {
                                anIndx = pSol!.GetPointState(aSI[i].Index());
                            }
                        }

                        gp_Pnt aPnt = anIndx < 0 ? Func.LastComputedPoint() : Func.Valpoint(anIndx);

                        if (dist > 0.1 * Precision.Confusion())
                        {
                            // Precise found points. It results in following:
                            //   1. Make the vertex nearer to the intersection line
                            //     (see description to issue #27252 in order to
                            //     understand necessity).
                            //   2. Merge two near vertices to single point.

                            // All members in TabSol array has already been sorted in increase order.
                            // Now, we limit precise boundaries in order to avoid changing this order.
                            double aFPar = (i == 1) ? Pdeb : (para + aSI[i - 1].Value()) / 2.0;
                            double aLPar = (i == Nbp) ? Pfin : (para + aSI[i + 1].Value()) / 2.0;

                            MinFunction aNewFunc = new(Func);
                            math_BrentMinimum aMin = new(Precision.Confusion());

                            aMin.Perform(aNewFunc, aFPar, para, aLPar);
                            if (aMin.IsDone())
                            {
                                para = aMin.Location();
                                gp_Pnt2d aP2d = A.Value(para);
                                aPnt = Func.Surface().Value(aP2d.X(), aP2d.Y());
                            }
                        }

                        PointProcess(aPnt, para, A, Domain, pnt, TolBoundary, out range);
                    }
                } // end of if(ip)
            } // end of if(Nbp)

            // Pour chaque intervalle trouve faire
            //   Traiter les extremites comme des points
            //   Ajouter intervalle dans la liste des segments

            if (!IsIntCSdone)
            {
                Nbi = pSol!.NbIntervals();
            }

            if (!RecheckOnRegularity && Nbp != 0)
            {
                Nbi = 0;
            }

            for (i = 1; i <= Nbi; i++)
            {
                TheSegment newseg = new();
                newseg.SetValue(A);
                // Recuperer point debut et fin, et leur parametre.
                if (IsIntCSdone)
                {
                    IntCurveSurface_IntersectionSegment IntSeg = IntCS.Segment(i);
                    IntCurveSurface_IntersectionPoint End1 = IntSeg.FirstPoint();
                    IntCurveSurface_IntersectionPoint End2 = IntSeg.SecondPoint();
                    pardeb = End1.W();
                    parfin = End2.W();
                    ptdeb = End1.Pnt();
                    ptfin = End2.Pnt();
                }
                else
                {
                    pSol!.GetInterval(i, out pardeb, out parfin);
                    pSol.GetIntervalState(i, out ideb, out ifin);

                    ptdeb = Func.Valpoint(ideb);
                    ptfin = Func.Valpoint(ifin);
                }

                PointProcess(ptdeb, pardeb, A, Domain, pnt, TolBoundary, out ranged);
                newseg.SetLimitPoint(pnt.Value(ranged), true);
                PointProcess(ptfin, parfin, A, Domain, pnt, TolBoundary, out rangef);
                newseg.SetLimitPoint(pnt.Value(rangef), false);
                seg.Append(newseg);
            }

            if (Nbi == 1)
            {
                if ((Math.Abs(pardeb - Pdeb) < Precision.PConfusion()) && (Math.Abs(parfin - Pfin) < Precision.PConfusion()))
                {
                    Arcsol = true;
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
    private static void ComputeBoundsfromInfinite(TheFunction Func, ref double PDeb, ref double PFin, out int NbEchant)
    {
        NbEchant = 100;

        double U0 = 0.0;
        double dU = 0.001;
        double Dist0, Dist1;

        Func.Value(U0, out Dist0);
        Func.Value(U0 + dU, out Dist1);
        double dDist = Dist1 - Dist0;
        if (dDist != 0)
        {
            U0 -= dU * Dist0 / dDist;
            PDeb = PFin = U0;
            double Umin = U0 - 1e5;
            Func.Value(Umin, out Dist0);
            Func.Value(Umin + dU, out Dist1);
            dDist = Dist1 - Dist0;
            if (dDist != 0)
            {
                Umin -= dU * Dist0 / dDist;
            }
            else
            {
                Umin -= 10.0;
            }
            double Umax = U0 + 1e8;
            Func.Value(Umax, out Dist0);
            Func.Value(Umax + dU, out Dist1);
            dDist = Dist1 - Dist0;
            if (dDist != 0)
            {
                Umax -= dU * Dist0 / dDist;
            }
            else
            {
                Umax += 10.0;
            }
            if (Umin > U0)
            {
                Umin = U0 - 10.0;
            }
            if (Umax < U0)
            {
                Umax = U0 + 10.0;
            }

            PFin = Umax + 10.0 * (Umax - Umin);
            PDeb = Umin - 10.0 * (Umax - Umin);
        }
        else
        {
            //-- Possibilite de Arc totalement inclu ds Quad
            PDeb = 1e10;
            PFin = -1e10;
        }
    }

    // Check to see if a solution point is coincident with a vertex.
    // If confused, you should find this vertex in the list of
    // Start. It then returns the position of this point in the list pnt.
    // Otherwise, add the point in the list.
    private static void PointProcess(in gp_Pnt Pt, double Para, Adaptor2d_Curve2d A, Adaptor3d_TopolTool Domain,
                                     NCollection_Sequence<ThePathPoint> pnt, double Tol, out int Range)
    {
        int k;
        bool found, goon;
        double dist, toler;

        Range = 0; // C++ leaves Range unwritten when no case below applies (never: a point is always found or added)
        int Nbsol = pnt.Length();
        Adaptor3d_HVertex vtx;
        ThePathPoint ptsol = new();

        Domain.Initialize(A);
        Domain.InitVertexIterator();
        found = false;
        goon = Domain.MoreVertex();
        while (goon)
        {
            vtx = Domain.Vertex();
            dist = Math.Abs(Para - TheSOBTool.Parameter(vtx, A));
            toler = TheSOBTool.Tolerance(vtx, A);
            // OCCT_DEBUG: a warning on vertex tolerances above 0.1, which also clamps those above 10000 to 1e-7 in
            // debug builds only: left out, as in OCCT's release builds.

            if (dist <= toler)
            {
                // Locate the vertex in the list of solutions
                k = 1;
                found = (k > Nbsol);
                while (!found)
                {
                    ptsol = pnt.Value(k);
                    if (!ptsol.IsNew())
                    {
                        // jag 940608  if (ptsol.Vertex() == vtx && ptsol.Arc()    == A) {
                        if (Domain.Identical(ptsol.Vertex(), vtx) && ReferenceEquals(ptsol.Arc(), A)
                            && Math.Abs(ptsol.Parameter() - Para) <= toler)
                        {
                            found = true;
                        }
                        else
                        {
                            k = k + 1;
                            found = (k > Nbsol);
                        }
                    }
                    else
                    {
                        k = k + 1;
                        found = (k > Nbsol);
                    }
                }
                if (k <= Nbsol)
                { // We find the vertex
                    Range = k;
                }
                else
                { // Otherwise
                    ptsol.SetValue(Pt, Tol, vtx, A, Para);
                    pnt.Append(ptsol);
                    Range = pnt.Length();
                }
                found = true;
                goon = false;
            }
            else
            {
                Domain.NextVertex();
                goon = Domain.MoreVertex();
            }
        }

        if (!found)
        { // No one is falling on a vertex
            // jgv: do not add segment's extremities if they already exist
            bool found_internal = false;
            for (k = 1; k <= pnt.Length(); k++)
            {
                ptsol = pnt.Value(k);
                if (!ReferenceEquals(ptsol.Arc(), A) || !ptsol.IsNew()) // vertex
                {
                    continue;
                }
                if (Math.Abs(ptsol.Parameter() - Para) <= Precision.PConfusion())
                {
                    found_internal = true;
                    Range = k;
                }
            }
            /////////////////////////////////////////////////////////////

            if (!found_internal)
            {
                double TOL = Tol;
                TOL *= 1000.0;
                // if(TOL>0.001) TOL=0.001;
                if (TOL > 0.005)
                {
                    TOL = 0.005; // #24643
                }

                ptsol.SetValue(Pt, TOL, A, Para);
                pnt.Append(ptsol);
                Range = pnt.Length();
            }
        }
    }

    private static bool IsRegularity(Adaptor2d_Curve2d A, Adaptor3d_TopolTool aDomain)
    {
        object? anEAddress = aDomain.Edge();
        if (anEAddress == null)
        {
            return false;
        }

        TopoDS_Edge anE = (TopoDS_Edge)anEAddress;

        return (BRep_Tool.HasContinuity(anE));
    }

    private static int TreatLC(Adaptor2d_Curve2d A, Adaptor3d_TopolTool aDomain, in IntSurf_Quadric aQuadric,
                               double TolBoundary, NCollection_Sequence<ThePathPoint> pnt)
    {
        int anExitCode = 1, aNbExt;

        object? anEAddress = aDomain.Edge();
        if (anEAddress == null)
        {
            return anExitCode;
        }

        TopoDS_Edge anE = (TopoDS_Edge)anEAddress;

        if (BRep_Tool.Degenerated(anE))
        {
            return anExitCode;
        }

        GeomAbs_CurveType aTypeE;
        BRepAdaptor_Curve aBAC = new(anE);
        aTypeE = aBAC.GetType();

        if (aTypeE != GeomAbs_Line)
        {
            return anExitCode;
        }

        GeomAbs_SurfaceType aTypeS;
        aTypeS = aQuadric.TypeQuadric();

        if (aTypeS != GeomAbs_Cylinder)
        {
            return anExitCode;
        }

        double f, l, U1f, U1l, U2f, U2l, UEgde, TOL, aDist, aR, aRRel, Tol;
        Geom_Curve aCEdge = BRep_Tool.Curve(anE, out f, out l)!;

        gp_Cylinder aCyl = aQuadric.Cylinder();
        gp_Ax1 anAx1 = aCyl.Axis();
        gp_Lin aLin = new(anAx1);
        Geom_Line aCAxis = new(aLin);
        aR = aCyl.Radius();

        U1f = aCAxis.FirstParameter();
        U1l = aCAxis.LastParameter();

        U2f = aCEdge.FirstParameter();
        U2l = aCEdge.LastParameter();

        GeomAdaptor_Curve C1 = new(), C2 = new();

        C1.Load(aCAxis);
        C2.Load(aCEdge);

        Tol = Precision.PConfusion();

        Extrema_ExtCC anExtCC = new(C1, C2, U1f, U1l, U2f, U2l, Tol, Tol);

        aNbExt = anExtCC.NbExt();
        if (aNbExt != 1)
        {
            return anExitCode;
        }

        gp_Pnt P1, PEdge;

        anExtCC.Points(1, out Extrema_POnCurv PC1, out Extrema_POnCurv PC2);

        P1 = PC1.Value();
        PEdge = PC2.Value();

        UEgde = PC2.Parameter();

        aDist = PEdge.Distance(P1);
        aRRel = Math.Abs(aDist - aR) / aR;
        if (aRRel > TolBoundary)
        {
            return anExitCode;
        }

        if (UEgde < (f + TolBoundary) || UEgde > (l - TolBoundary))
        {
            return anExitCode;
        }
        //
        // Do not wonder !
        // It was done as into PointProcess(...) function
        // printf("TreatLC()=> tangent line is found\n");
        TOL = 1000.0 * TolBoundary;
        if (TOL > 0.001)
        {
            TOL = 0.001;
        }

        ThePathPoint ptsol = new();
        ptsol.SetValue(PEdge, TOL, A, UEgde);
        pnt.Append(ptsol);

        anExitCode = 0;
        return anExitCode;
    }
}
