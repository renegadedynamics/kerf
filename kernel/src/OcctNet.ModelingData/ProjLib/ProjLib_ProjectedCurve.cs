// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_ProjectedCurve.hxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_ProjectedCurve.cxx
// Port of: src/ModelingData/TKGeomBase/ProjLib/ProjLib_HProjectedCurve.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Created on: 1993-08-25
// Created by: Bruno DUMORTIER

//  Modified by skv - Wed Aug 11 15:45:58 2004 OCC6272

using static Occt.AppParCurves_Constraint;
using static Occt.Convert_ParameterisationType;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_IsoType;
using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// The 2D curve of a 3D curve projected on a surface. Particular cases (elementary curves on elementary surfaces)
/// are solved exactly; otherwise the projection is approximated, the tolerance being the maximal 3D deviation of
/// the projected curve from the exact projection (the 2D tolerance is computed from it with the surface's U and V
/// resolutions).
/// </summary>
/// <remarks>
/// An adaptor and a working object (decisions 21 and 101): <c>Load</c>, <see cref="Perform"/> and the setters are
/// public. It keeps the caller's adaptors, as OCCT keeps its handles: they must outlive the projection. Most adaptor
/// queries (continuity, intervals, evaluation, trimming) throw Standard_NotImplemented, as in OCCT: the result is
/// read through <see cref="GetType"/> and the curve accessors. <see cref="BSpline"/> and <see cref="Bezier"/> freeze
/// the result curve and return the same object at each call. OCCT's <c>ProjLib_HProjectedCurve</c> is a typedef of
/// this class.
/// </remarks>
public class ProjLib_ProjectedCurve : Adaptor2d_Curve2d
{
    private double myTolerance;
    private Adaptor3d_Surface? mySurface;
    private Adaptor3d_Curve? myCurve;
    private readonly ProjLib_Projector myResult = new();
    private int myDegMin;
    private int myDegMax;
    private int myMaxSegments;
    private double myMaxDist;
    private AppParCurves_Constraint myBndPnt;

    //=================================================================================================

    private static double ComputeTolU(Adaptor3d_Surface theSurf, double theTolerance)
    {
        double aTolU = theSurf.UResolution(theTolerance);
        if (theSurf.IsUPeriodic())
        {
            aTolU = Min(aTolU, 0.01 * theSurf.UPeriod());
        }

        return aTolU;
    }

    //=================================================================================================

    private static double ComputeTolV(Adaptor3d_Surface theSurf, double theTolerance)
    {
        double aTolV = theSurf.VResolution(theTolerance);
        if (theSurf.IsVPeriodic())
        {
            aTolV = Min(aTolV, 0.01 * theSurf.VPeriod());
        }

        return aTolV;
    }

    //=================================================================================================

    private static bool IsoIsDeg(Adaptor3d_Surface S, double Param, GeomAbs_IsoType IT, double TolMin, double TolMax)
    {
        double U1 = 0.0, U2 = 0.0, V1 = 0.0, V2 = 0.0, T;
        bool Along = true;
        U1 = S.FirstUParameter();
        U2 = S.LastUParameter();
        V1 = S.FirstVParameter();
        V2 = S.LastVParameter();
        gp_Vec D1U, D1V;
        gp_Pnt P;
        double Step, D1NormMax;
        if (IT == GeomAbs_IsoV)
        {
            Step = (U2 - U1) / 10;
            D1NormMax = 0.0;
            for (T = U1; T <= U2; T = T + Step)
            {
                S.D1(T, Param, out P, out D1U, out D1V);
                D1NormMax = Max(D1NormMax, D1U.Magnitude());
            }

            if (D1NormMax > TolMax || D1NormMax < TolMin)
            {
                Along = false;
            }
        }
        else
        {
            Step = (V2 - V1) / 10;
            D1NormMax = 0.0;
            for (T = V1; T <= V2; T = T + Step)
            {
                S.D1(Param, T, out P, out D1U, out D1V);
                D1NormMax = Max(D1NormMax, D1V.Magnitude());
            }

            if (D1NormMax > TolMax || D1NormMax < TolMin)
            {
                Along = false;
            }
        }
        return Along;
    }

    //=================================================================================================

    private static void TrimC3d(ref Adaptor3d_Curve myCurve,
                                Span<bool> IsTrimmed,
                                double dt,
                                in gp_Pnt Pole,
                                Span<int> SingularCase,
                                int NumberOfSingularCase,
                                double TolConf)
    {
        double f = myCurve.FirstParameter();
        double l = myCurve.LastParameter();

        gp_Pnt P = myCurve.Value(f);

        if (P.Distance(Pole) <= TolConf)
        {
            IsTrimmed[0] = true;
            f = f + dt;
            myCurve = myCurve.Trim(f, l, Precision.Confusion());
            SingularCase[0] = NumberOfSingularCase;
        }

        P = myCurve.Value(l);
        if (P.Distance(Pole) <= TolConf)
        {
            IsTrimmed[1] = true;
            l = l - dt;
            myCurve = myCurve.Trim(f, l, Precision.Confusion());
            SingularCase[1] = NumberOfSingularCase;
        }
    }

    //=================================================================================================

    // aRes is a curve the projection has just built (unfrozen); the extension is a new curve.
    private static void ExtendC2d(ref Geom2d_BSplineCurve? aRes,
                                  double t,
                                  double dt,
                                  double u1,
                                  double u2,
                                  double v1,
                                  double v2,
                                  int FirstOrLast,
                                  int NumberOfSingularCase)
    {
        _ = t;
        _ = dt;
        double theParam = (FirstOrLast == 0) ? aRes!.FirstParameter() : aRes!.LastParameter();

        gp_Pnt2d aPBnd;
        gp_Vec2d aVBnd;
        gp_Dir2d aDBnd = new();
        Geom2d_TrimmedCurve aSegment;
        Geom2dConvert_CompCurveToBSplineCurve aCompCurve = new(aRes, Convert_RationalC1);
        double aTol = Precision.Confusion();

        aRes.D1(theParam, out aPBnd, out aVBnd);
        aDBnd.SetXY(aVBnd.XY());
        gp_Lin2d aLin = new(aPBnd, aDBnd); // line in direction of derivative

        gp_Pnt2d thePole = new();
        gp_Dir2d theBoundDir = new();
        switch (NumberOfSingularCase)
        {
            case 1:
            {
                thePole.SetCoord(u1, v1);
                theBoundDir.SetCoord(0.0, 1.0);
                break;
            }
            case 2:
            {
                thePole.SetCoord(u2, v1);
                theBoundDir.SetCoord(0.0, 1.0);
                break;
            }
            case 3:
            {
                thePole.SetCoord(u1, v1);
                theBoundDir.SetCoord(1.0, 0.0);
                break;
            }
            case 4:
            {
                thePole.SetCoord(u1, v2);
                theBoundDir.SetCoord(1.0, 0.0);
                break;
            }
        }
        gp_Lin2d BoundLin = new(thePole, theBoundDir); // one of the bounds of rectangle
        double ParOnLin = 0.0;
        if (theBoundDir.IsParallel(aDBnd, 100.0 * Precision.Angular()))
        {
            ParOnLin = ElCLib.Parameter(aLin, thePole);
        }
        else
        {
            double U1x = BoundLin.Direction().X();
            double U1y = BoundLin.Direction().Y();
            double U2x = aLin.Direction().X();
            double U2y = aLin.Direction().Y();
            double Uo21x = aLin.Location().X() - BoundLin.Location().X();
            double Uo21y = aLin.Location().Y() - BoundLin.Location().Y();

            double D = U1y * U2x - U1x * U2y;

            ParOnLin = (Uo21y * U1x - Uo21x * U1y) / D; // parameter of intersection point
        }

        Geom2d_Line aSegLine = new(aLin);
        aSegment = (FirstOrLast == 0) ? new Geom2d_TrimmedCurve(aSegLine, ParOnLin, 0.0)
                                      : new Geom2d_TrimmedCurve(aSegLine, 0.0, ParOnLin);

        bool anAfter = FirstOrLast != 0;
        aCompCurve.Add(aSegment, aTol, anAfter);
        // The concatenation is discarded: its curve is read unfrozen (decision 244).
        aRes = aCompCurve.InternalBSplineCurve()!;
    }

    //=================================================================================================

    private static void Project(ProjLib_Projector P, Adaptor3d_Curve C)
    {
        GeomAbs_CurveType CType = C.GetType();
        switch (CType)
        {
            case GeomAbs_Line:
                P.Project(C.Line());
                break;
            case GeomAbs_Circle:
                P.Project(C.Circle());
                break;
            case GeomAbs_Ellipse:
                P.Project(C.Ellipse());
                break;
            case GeomAbs_Hyperbola:
                P.Project(C.Hyperbola());
                break;
            case GeomAbs_Parabola:
                P.Project(C.Parabola());
                break;
            case GeomAbs_BSplineCurve:
            case GeomAbs_BezierCurve:
            case GeomAbs_OffsetCurve:
            case GeomAbs_OtherCurve: // try the approximation
                break;
            default:
                throw new Standard_NoSuchObject(" ");
        }
    }

    //=================================================================================================

    /// <summary>
    /// OCCT's <c>NCollection_DataMap&lt;int, int&gt;</c> of Perform's period count, with OCCT's buckets, so that
    /// the iteration (which picks the first of equally populated periods) visits the keys in OCCT's order (decision
    /// 18): the default integer hash is the key itself (sign-extended to size_t), bucket <c>hash % NbBuckets + 1</c>,
    /// new keys at the head of their chain, resized to the next prime for the map before an insertion when the map
    /// is empty or holds more keys than buckets.
    /// </summary>
    internal sealed class IntDataMap
    {
        private sealed class Node(int theKey, int theValue, Node? theNext)
        {
            public readonly int Key = theKey;
            public int Value = theValue;
            public Node? Next = theNext;
        }

        private Node?[]? myData1;
        private long myNbBuckets = 1;
        private int mySize;

        private static long HashCode(int theKey, long theUpperBound) =>
            (long)((ulong)(long)theKey % (ulong)theUpperBound + 1);

        private void ReSize(long N)
        {
            long newBuck = NCollection_Primes.NextPrimeForMap(N);
            if (newBuck <= myNbBuckets)
            {
                if (myData1 is null)
                {
                    newBuck = myNbBuckets;
                }
                else
                {
                    return;
                }
            }
            Node?[] newdata = new Node?[newBuck + 1];
            if (myData1 is not null)
            {
                for (long i = 0; i <= myNbBuckets; ++i)
                {
                    Node? p = myData1[i];
                    while (p is not null)
                    {
                        long k = HashCode(p.Key, newBuck);
                        Node? q = p.Next;
                        p.Next = newdata[k];
                        newdata[k] = p;
                        p = q;
                    }
                }
            }
            myNbBuckets = newBuck;
            myData1 = newdata;
        }

        private Node? Lookup(int theKey)
        {
            if (mySize == 0)
            {
                return null;
            }
            for (Node? aNode = myData1![HashCode(theKey, myNbBuckets)]; aNode is not null; aNode = aNode.Next)
            {
                if (aNode.Key == theKey)
                {
                    return aNode;
                }
            }
            return null;
        }

        public bool IsBound(int theKey) => Lookup(theKey) is not null;

        public ref int ChangeFind(int theKey) => ref Lookup(theKey)!.Value;

        public void Bind(int theKey, int theItem)
        {
            if (mySize == 0 || mySize > myNbBuckets)
            {
                ReSize(mySize);
            }
            Node? aNode = Lookup(theKey);
            if (aNode is not null)
            {
                aNode.Value = theItem;
                return;
            }
            long aHash = HashCode(theKey, myNbBuckets);
            myData1![aHash] = new Node(theKey, theItem, myData1[aHash]);
            mySize++;
        }

        /// <summary>The (key, value) pairs in OCCT's iteration order: buckets 0 to NbBuckets, each chain from its
        /// head.</summary>
        public IEnumerable<(int Key, int Value)> Pairs()
        {
            if (myData1 is null)
            {
                yield break;
            }
            for (long i = 0; i <= myNbBuckets; ++i)
            {
                for (Node? aNode = myData1[i]; aNode is not null; aNode = aNode.Next)
                {
                    yield return (aNode.Key, aNode.Value);
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Empty constructor, it only sets some initial values for class fields.</summary>
    public ProjLib_ProjectedCurve()
    {
        myTolerance = Precision.Confusion();
        myDegMin = -1;
        myDegMax = -1;
        myMaxSegments = -1;
        myMaxDist = -1.0;
        myBndPnt = AppParCurves_TangencyPoint;
    }

    //=================================================================================================

    /// <summary>Constructor with initialisation field mySurface.</summary>
    public ProjLib_ProjectedCurve(Adaptor3d_Surface S)
        : this()
    {
        Load(S);
    }

    //=================================================================================================

    /// <summary>Constructor, which performs projecting. If projecting uses approximation, default parameters are
    /// used, in particular, 3d tolerance of approximation is Precision::Confusion().</summary>
    public ProjLib_ProjectedCurve(Adaptor3d_Surface S, Adaptor3d_Curve C)
        : this()
    {
        Load(S);
        Perform(C);
    }

    //=================================================================================================

    /// <summary>Constructor, which performs projecting. If projecting uses approximation, 3d tolerance is Tol,
    /// default parameters are used.</summary>
    public ProjLib_ProjectedCurve(Adaptor3d_Surface S, Adaptor3d_Curve C, double Tol)
        : this()
    {
        myTolerance = Max(Tol, Precision.Confusion());
        Load(S);
        Perform(C);
    }

    //=================================================================================================

    /// <summary>Shallow copy of adaptor: shallow copies of the surface and curve adaptors, the result shared.</summary>
    public override Adaptor2d_Curve2d ShallowCopy()
    {
        ProjLib_ProjectedCurve aCopy = new();

        aCopy.myTolerance = myTolerance;
        if (mySurface is not null)
        {
            aCopy.mySurface = mySurface.ShallowCopy();
        }
        if (myCurve is not null)
        {
            aCopy.myCurve = myCurve.ShallowCopy();
        }
        aCopy.myResult.Assign(myResult);
        aCopy.myDegMin = myDegMin;
        aCopy.myDegMax = myDegMax;
        aCopy.myMaxSegments = myMaxSegments;
        aCopy.myMaxDist = myMaxDist;
        aCopy.myBndPnt = myBndPnt;

        return aCopy;
    }

    //=================================================================================================

    /// <summary>Changes the Surface.</summary>
    public void Load(Adaptor3d_Surface S)
    {
        mySurface = S;
    }

    //=================================================================================================

    /// <summary>Changes the tolerance used to project the curve on the surface.</summary>
    public void Load(double theTol)
    {
        myTolerance = theTol;
    }

    //=================================================================================================

    /// <summary>
    /// Performs projecting for given curve. If projecting uses approximation, approximation parameters can be set
    /// before by corresponding methods SetDegree(...), SetMaxSegmets(...), SetBndPnt(...), SetMaxDist(...).
    /// </summary>
    /// <remarks>As in OCCT, a result of an earlier Perform is not cleared when the new projection fails on a
    /// non-analytical surface.</remarks>
    public void Perform(Adaptor3d_Curve C)
    {
        myTolerance = Max(myTolerance, Precision.Confusion());
        myCurve = C;
        double FirstPar = C.FirstParameter();
        double LastPar = C.LastParameter();
        GeomAbs_SurfaceType SType = mySurface!.GetType();
        GeomAbs_CurveType CType = myCurve.GetType();
        bool isAnalyticalSurf = true;
        Span<bool> IsTrimmed = stackalloc bool[2] { false, false };
        // OCCT leaves SingularCase uninitialised; it is read only where TrimC3d set it.
        Span<int> SingularCase = stackalloc int[2];
        const double eps = 0.01;
        double TolConf = Precision.Confusion();
        double dt = (LastPar - FirstPar) * eps;
        double U1 = 0.0, U2 = 0.0, V1 = 0.0, V2 = 0.0;
        U1 = mySurface.FirstUParameter();
        U2 = mySurface.LastUParameter();
        V1 = mySurface.FirstVParameter();
        V2 = mySurface.LastVParameter();

        switch (SType)
        {
            case GeomAbs_Plane:
            {
                ProjLib_Plane P = new(mySurface.Plane());
                Project(P, myCurve);
                myResult.Assign(P);
            }
            break;

            case GeomAbs_Cylinder:
            {
                ProjLib_Cylinder P = new(mySurface.Cylinder());
                Project(P, myCurve);
                myResult.Assign(P);
            }
            break;

            case GeomAbs_Cone:
            {
                ProjLib_Cone P = new(mySurface.Cone());
                Project(P, myCurve);
                myResult.Assign(P);
            }
            break;

            case GeomAbs_Sphere:
            {
                ProjLib_Sphere P = new(mySurface.Sphere());
                Project(P, myCurve);
                if (P.IsDone())
                {
                    // Place into the pseudo-period (since Sphere is not
                    // periodic in V!)
                    P.SetInBounds(myCurve.FirstParameter());
                }
                else
                {
                    const double Vmax = M_PI / 2.0;
                    const double Vmin = -Vmax;
                    const double minang = 1.0e-5 * M_PI;
                    gp_Sphere aSph = mySurface.Sphere();
                    double anR = aSph.Radius();
                    double f = myCurve.FirstParameter();
                    double l = myCurve.LastParameter();

                    gp_Pnt Pf = myCurve.Value(f);
                    gp_Pnt Pl = myCurve.Value(l);
                    gp_Pnt aLoc = aSph.Position().Location();
                    double maxdist = Max(Pf.Distance(aLoc), Pl.Distance(aLoc));
                    TolConf = Max(anR * minang, Math.Abs(anR - maxdist));

                    // Surface has pole at V = Vmin and Vmax
                    gp_Pnt Pole = mySurface.Value(U1, Vmin);
                    TrimC3d(ref myCurve, IsTrimmed, dt, Pole, SingularCase, 3, TolConf);
                    Pole = mySurface.Value(U1, Vmax);
                    TrimC3d(ref myCurve, IsTrimmed, dt, Pole, SingularCase, 4, TolConf);
                }
                myResult.Assign(P);
            }
            break;

            case GeomAbs_Torus:
            {
                ProjLib_Torus P = new(mySurface.Torus());
                Project(P, myCurve);
                myResult.Assign(P);
            }
            break;

            case GeomAbs_BezierSurface:
            case GeomAbs_BSplineSurface:
            {
                isAnalyticalSurf = false;
                double f, l;
                f = myCurve.FirstParameter();
                l = myCurve.LastParameter();
                dt = (l - f) * eps;

                Adaptor3d_Surface S = mySurface;
                U1 = S.FirstUParameter();
                U2 = S.LastUParameter();
                V1 = S.FirstVParameter();
                V2 = S.LastVParameter();

                if (IsoIsDeg(S, U1, GeomAbs_IsoU, 0.0, myTolerance))
                {
                    // Surface has pole at U = Umin
                    gp_Pnt Pole = mySurface.Value(U1, V1);
                    TrimC3d(ref myCurve, IsTrimmed, dt, Pole, SingularCase, 1, TolConf);
                }

                if (IsoIsDeg(S, U2, GeomAbs_IsoU, 0.0, myTolerance))
                {
                    // Surface has pole at U = Umax
                    gp_Pnt Pole = mySurface.Value(U2, V1);
                    TrimC3d(ref myCurve, IsTrimmed, dt, Pole, SingularCase, 2, TolConf);
                }

                if (IsoIsDeg(S, V1, GeomAbs_IsoV, 0.0, myTolerance))
                {
                    // Surface has pole at V = Vmin
                    gp_Pnt Pole = mySurface.Value(U1, V1);
                    TrimC3d(ref myCurve, IsTrimmed, dt, Pole, SingularCase, 3, TolConf);
                }

                if (IsoIsDeg(S, V2, GeomAbs_IsoV, 0.0, myTolerance))
                {
                    // Surface has pole at V = Vmax
                    gp_Pnt Pole = mySurface.Value(U1, V2);
                    TrimC3d(ref myCurve, IsTrimmed, dt, Pole, SingularCase, 4, TolConf);
                }

                ProjLib_ComputeApproxOnPolarSurface polar = new();
                polar.SetTolerance(myTolerance);
                polar.SetDegree(myDegMin, myDegMax);
                polar.SetMaxSegments(myMaxSegments);
                polar.SetBndPnt(myBndPnt);
                polar.SetMaxDist(myMaxDist);
                polar.Perform(myCurve, mySurface);

                // The local algorithm is discarded: its curve is extended in place (decision 287).
                Geom2d_BSplineCurve? aRes = polar.InternalBSpline();

                if (aRes is not null)
                {
                    myTolerance = polar.Tolerance();
                    if ((IsTrimmed[0] || IsTrimmed[1]))
                    {
                        if (IsTrimmed[0])
                        {
                            // Add segment before start of curve
                            f = myCurve.FirstParameter();
                            ExtendC2d(ref aRes, f, -dt, U1, U2, V1, V2, 0, SingularCase[0]);
                        }
                        if (IsTrimmed[1])
                        {
                            // Add segment after end of curve
                            l = myCurve.LastParameter();
                            ExtendC2d(ref aRes, l, dt, U1, U2, V1, V2, 1, SingularCase[1]);
                        }
                        Geom2d_Curve? NewCurve2d = null;
                        GeomLib.SameRange(Precision.PConfusion(),
                                          aRes,
                                          aRes!.FirstParameter(),
                                          aRes.LastParameter(),
                                          FirstPar,
                                          LastPar,
                                          ref NewCurve2d);
                        aRes = NewCurve2d as Geom2d_BSplineCurve;
                    }
                    myResult.SetBSpline(aRes);
                    myResult.Done();
                    myResult.SetType(GeomAbs_BSplineCurve);
                }
            }
            break;

            default:
            {
                isAnalyticalSurf = false;
                Span<double> Vsingular = stackalloc double[2] { 0.0, 0.0 }; // for surfaces of revolution
                double f = 0.0, l = 0.0;
                dt = 0.0;

                if (mySurface.GetType() == GeomAbs_SurfaceOfRevolution)
                {
                    // Check possible singularity

                    gp_Pnt P = mySurface.AxeOfRevolution().Location();
                    gp_Dir N = mySurface.AxeOfRevolution().Direction();

                    gp_Lin L = new(P, N);

                    f = myCurve.FirstParameter();
                    l = myCurve.LastParameter();
                    dt = (l - f) * eps;

                    P = myCurve.Value(f);
                    if (L.Distance(P) < Precision.Confusion())
                    {
                        IsTrimmed[0] = true;
                        f = f + dt;
                        myCurve = myCurve.Trim(f, l, Precision.Confusion());
                        // Searching the parameter on the basis curve for surface of revolution
                        Extrema_ExtPC anExtr = new(P, mySurface.BasisCurve(), myTolerance);
                        if (anExtr.IsDone())
                        {
                            double aMinDist = RealLast();
                            for (int anIdx = 1; anIdx <= anExtr.NbExt(); anIdx++)
                            {
                                if (anExtr.IsMin(anIdx) && anExtr.SquareDistance(anIdx) < aMinDist)
                                {
                                    aMinDist = anExtr.SquareDistance(anIdx);
                                    Vsingular[0] = anExtr.Point(anIdx).Parameter();
                                }
                            }
                        }
                        else
                        {
                            Vsingular[0] = ElCLib.Parameter(L, P);
                        }
                        // SingularCase[0] = 3;
                    }

                    P = myCurve.Value(l);
                    if (L.Distance(P) < Precision.Confusion())
                    {
                        IsTrimmed[1] = true;
                        l = l - dt;
                        myCurve = myCurve.Trim(f, l, Precision.Confusion());
                        // Searching the parameter on the basis curve for surface of revolution
                        Extrema_ExtPC anExtr = new(P, mySurface.BasisCurve(), myTolerance);
                        if (anExtr.IsDone())
                        {
                            double aMinDist = RealLast();
                            for (int anIdx = 1; anIdx <= anExtr.NbExt(); anIdx++)
                            {
                                if (anExtr.IsMin(anIdx) && anExtr.SquareDistance(anIdx) < aMinDist)
                                {
                                    aMinDist = anExtr.SquareDistance(anIdx);
                                    Vsingular[1] = anExtr.Point(anIdx).Parameter();
                                }
                            }
                        }
                        else
                        {
                            Vsingular[1] = ElCLib.Parameter(L, P);
                        }
                        // SingularCase[1] = 4;
                    }
                }

                double aTolU = Max(ComputeTolU(mySurface, myTolerance), Precision.Confusion());
                double aTolV = Max(ComputeTolV(mySurface, myTolerance), Precision.Confusion());
                double aTol2d = Math.Sqrt(aTolU * aTolU + aTolV * aTolV);

                double aMaxDist = 100.0 * myTolerance;
                if (myMaxDist > 0.0)
                {
                    aMaxDist = myMaxDist;
                }
                ProjLib_CompProjectedCurve HProjector = new(mySurface, myCurve, aTolU, aTolV, aMaxDist);

                // Normally, within the framework of ProjLib, the result
                // must be one and only one curve!
                // Moreover, this curve must not be a Single point
                int NbCurves = HProjector.NbCurves();
                double Udeb = 0.0, Ufin = 0.0;
                if (NbCurves > 0)
                {
                    HProjector.Bounds(1, out Udeb, out Ufin);
                }
                else
                {
                    return;
                }
                // Approximate this algorithmic curve.
                bool Only3d = false;
                bool Only2d = true;
                GeomAbs_Shape Continuity = GeomAbs_C1;
                if (myBndPnt == AppParCurves_PassPoint)
                {
                    Continuity = GeomAbs_C0;
                }
                int MaxDegree = 14;
                if (myDegMax > 0)
                {
                    MaxDegree = myDegMax;
                }
                int MaxSeg = 16;
                if (myMaxSegments > 0)
                {
                    MaxSeg = myMaxSegments;
                }

                Approx_CurveOnSurface appr = new(HProjector, mySurface, Udeb, Ufin, myTolerance);
                appr.Perform(MaxSeg, MaxDegree, Continuity, Only3d, Only2d);

                // The local approximation is discarded: its curve is changed in place (decision 287).
                Geom2d_BSplineCurve? aRes = appr.InternalCurve2d();

                if (aRes is not null)
                {
                    aTolU = appr.MaxError2dU();
                    aTolV = appr.MaxError2dV();
                    double aNewTol2d = Math.Sqrt(aTolU * aTolU + aTolV * aTolV);
                    myTolerance *= (aNewTol2d / aTol2d);
                    if (IsTrimmed[0] || IsTrimmed[1])
                    {
                        // Treatment only for surface of revolution
                        double u1, u2, v1, v2;
                        u1 = mySurface.FirstUParameter();
                        u2 = mySurface.LastUParameter();
                        v1 = mySurface.FirstVParameter();
                        v2 = mySurface.LastVParameter();

                        if (IsTrimmed[0])
                        {
                            // Add segment before start of curve
                            ExtendC2d(ref aRes, f, -dt, u1, u2, Vsingular[0], v2, 0, 3);
                        }
                        if (IsTrimmed[1])
                        {
                            // Add segment after end of curve
                            ExtendC2d(ref aRes, l, dt, u1, u2, v1, Vsingular[1], 1, 4);
                        }
                        Geom2d_Curve? NewCurve2d = null;
                        GeomLib.SameRange(Precision.PConfusion(),
                                          aRes,
                                          aRes!.FirstParameter(),
                                          aRes.LastParameter(),
                                          FirstPar,
                                          LastPar,
                                          ref NewCurve2d);
                        aRes = NewCurve2d as Geom2d_BSplineCurve;
                        if (Continuity == GeomAbs_C0)
                        {
                            // try to smoother the Curve GeomAbs_C1.
                            int aDeg = aRes!.Degree();
                            bool OK = true;
                            double aSmoothTol = Max(Precision.Confusion(), aNewTol2d);
                            for (int ij = 2; ij < aRes.NbKnots(); ij++)
                            {
                                OK = OK && aRes.RemoveKnot(ij, aDeg - 1, aSmoothTol);
                            }
                        }
                    }

                    myResult.SetBSpline(aRes);
                    myResult.Done();
                    myResult.SetType(GeomAbs_BSplineCurve);
                }
            }
            break;
        }

        if (!myResult.IsDone() && isAnalyticalSurf)
        {
            // Use advanced analytical projector if base analytical projection failed.
            ProjLib_ComputeApprox Comp = new();
            Comp.SetTolerance(myTolerance);
            Comp.SetDegree(myDegMin, myDegMax);
            Comp.SetMaxSegments(myMaxSegments);
            Comp.SetBndPnt(myBndPnt);
            Comp.Perform(myCurve, mySurface);
            // The local algorithm is discarded: its curves are read unfrozen (decision 287).
            if (Comp.InternalBezier() is null && Comp.InternalBSpline() is null)
            {
                return; // advanced projector has been failed too
            }
            myResult.Done();
            Geom2d_BSplineCurve? aRes;
            if (Comp.InternalBSpline() is null)
            {
                aRes = Geom2dConvert.CurveToBSplineCurve(Comp.InternalBezier()!);
            }
            else
            {
                aRes = Comp.InternalBSpline();
            }
            if ((IsTrimmed[0] || IsTrimmed[1]))
            {
                if (IsTrimmed[0])
                {
                    // Add segment before start of curve
                    double f = myCurve.FirstParameter();
                    ExtendC2d(ref aRes, f, -dt, U1, U2, V1, V2, 0, SingularCase[0]);
                }
                if (IsTrimmed[1])
                {
                    // Add segment after end of curve
                    double l = myCurve.LastParameter();
                    ExtendC2d(ref aRes, l, dt, U1, U2, V1, V2, 1, SingularCase[1]);
                }
                Geom2d_Curve? NewCurve2d = null;
                GeomLib.SameRange(Precision.PConfusion(),
                                  aRes,
                                  aRes!.FirstParameter(),
                                  aRes.LastParameter(),
                                  FirstPar,
                                  LastPar,
                                  ref NewCurve2d);
                aRes = NewCurve2d as Geom2d_BSplineCurve;
                myResult.SetBSpline(aRes);
                myResult.SetType(GeomAbs_BSplineCurve);
            }
            else
            {
                // set the type
                if (SType == GeomAbs_Plane && CType == GeomAbs_BezierCurve)
                {
                    myResult.SetType(GeomAbs_BezierCurve);
                    myResult.SetBezier(Comp.InternalBezier());
                }
                else
                {
                    myResult.SetType(GeomAbs_BSplineCurve);
                    myResult.SetBSpline(Comp.InternalBSpline());
                }
            }
            // set the periodicity flag
            if (SType == GeomAbs_Plane && CType == GeomAbs_BSplineCurve && myCurve.IsPeriodic())
            {
                myResult.SetPeriodic();
            }
            myTolerance = Comp.Tolerance();
        }

        Span<bool> isPeriodic = stackalloc bool[2] { mySurface.IsUPeriodic(), mySurface.IsVPeriodic() };
        if (myResult.IsDone() && (isPeriodic[0] || isPeriodic[1]))
        {
            // Check result curve to be in params space.

            // U and V parameters space correspondingly.
            Span<double> aSurfFirstPar = stackalloc double[2] { mySurface.FirstUParameter(), mySurface.FirstVParameter() };
            Span<double> aSurfPeriod = stackalloc double[2] { 0.0, 0.0 };
            if (isPeriodic[0])
            {
                aSurfPeriod[0] = mySurface.UPeriod();
            }
            if (isPeriodic[1])
            {
                aSurfPeriod[1] = mySurface.VPeriod();
            }

            for (int anIdx = 1; anIdx <= 2; anIdx++)
            {
                if (!isPeriodic[anIdx - 1])
                {
                    continue;
                }

                if (myResult.GetType() == GeomAbs_BSplineCurve)
                {
                    IntDataMap aMap = new();
                    // A curve this projection built (or the result of an earlier Perform, which OCCT translates
                    // again): unfrozen until handed out.
                    Geom2d_BSplineCurve aRes = myResult.InternalBSpline()!;
                    int aDeg = aRes.Degree();

                    for (int aKnotIdx = aRes.FirstUKnotIndex(); aKnotIdx < aRes.LastUKnotIndex(); aKnotIdx++)
                    {
                        double aFirstParam = aRes.Knot(aKnotIdx);
                        double aLastParam = aRes.Knot(aKnotIdx + 1);

                        for (int anIntIdx = 0; anIntIdx <= aDeg; anIntIdx++)
                        {
                            double aCurrParam = aFirstParam + (aLastParam - aFirstParam) * anIntIdx / (aDeg + 1.0);
                            gp_Pnt2d aPnt2d;
                            aRes.D0(aCurrParam, out aPnt2d);

                            int aMapKey = (int)((aPnt2d.Coord(anIdx) - aSurfFirstPar[anIdx - 1]) / aSurfPeriod[anIdx - 1]);

                            if (aPnt2d.Coord(anIdx) - aSurfFirstPar[anIdx - 1] < 0.0)
                            {
                                aMapKey--;
                            }

                            if (aMap.IsBound(aMapKey))
                            {
                                aMap.ChangeFind(aMapKey)++;
                            }
                            else
                            {
                                aMap.Bind(aMapKey, 1);
                            }
                        }
                    }

                    int aMaxPoints = 0, aMaxIdx = 0;
                    foreach ((int aKey, int aValue) in aMap.Pairs())
                    {
                        if (aValue > aMaxPoints)
                        {
                            aMaxPoints = aValue;
                            aMaxIdx = aKey;
                        }
                    }
                    if (aMaxIdx != 0)
                    {
                        gp_Pnt2d aFirstPnt = aRes.Value(aRes.FirstParameter());
                        gp_Pnt2d aSecondPnt = aFirstPnt;
                        aSecondPnt.SetCoord(anIdx, aFirstPnt.Coord(anIdx) - aSurfPeriod[anIdx - 1] * aMaxIdx);
                        aRes.Translate(new gp_Vec2d(aFirstPnt, aSecondPnt));
                    }
                }

                if (myResult.GetType() == GeomAbs_Line)
                {
                    double aT1 = myCurve.FirstParameter();
                    double aT2 = myCurve.LastParameter();

                    if (anIdx == 1)
                    {
                        // U param space.
                        myResult.UFrame(aT1, aT2, aSurfFirstPar[anIdx - 1], aSurfPeriod[anIdx - 1]);
                    }
                    else
                    {
                        // V param space.
                        myResult.VFrame(aT1, aT2, aSurfFirstPar[anIdx - 1], aSurfPeriod[anIdx - 1]);
                    }
                }
            }
        }
    }

    //=================================================================================================

    /// <summary>Set min and max possible degree of result BSpline curve2d, which is got by approximation. If
    /// theDegMin/Max &lt; 0, algorithm uses values that are chosen depending of types curve 3d and surface.</summary>
    public void SetDegree(int theDegMin, int theDegMax)
    {
        myDegMin = theDegMin;
        myDegMax = theDegMax;
    }

    //=================================================================================================

    /// <summary>Set the parameter, which defines maximal value of parametric intervals the projected curve can be
    /// cut for approximation. If theMaxSegments &lt; 0, algorithm uses default value = 1000.</summary>
    public void SetMaxSegments(int theMaxSegments)
    {
        myMaxSegments = theMaxSegments;
    }

    //=================================================================================================

    /// <summary>Set the parameter, which defines type of boundary condition between segments during
    /// approximation. It can be AppParCurves_PassPoint or AppParCurves_TangencyPoint. Default value is
    /// AppParCurves_TangencyPoint.</summary>
    public void SetBndPnt(AppParCurves_Constraint theBndPnt)
    {
        myBndPnt = theBndPnt;
    }

    //=================================================================================================

    /// <summary>Set the parameter, which defines maximal possible distance between projected curve and surface. It
    /// uses only for projecting on not analytical surfaces. If theMaxDist &lt; 0, algorithm uses default value
    /// 100.*Tolerance. If real distance between curve and surface more then theMaxDist, algorithm stops
    /// working.</summary>
    public void SetMaxDist(double theMaxDist)
    {
        myMaxDist = theMaxDist;
    }

    //=================================================================================================

    /// <summary>The caller's surface adaptor, or null before a Load.</summary>
    public Adaptor3d_Surface? GetSurface() => mySurface;

    //=================================================================================================

    /// <summary>The projected curve: the caller's adaptor, or a trimmed copy of it when the curve ends at a pole of
    /// the surface; null before a Perform.</summary>
    public Adaptor3d_Curve? GetCurve() => myCurve;

    //=================================================================================================

    /// <summary>Returns the tolerance reached if an approximation is Done.</summary>
    public double GetTolerance() => myTolerance;

    //=================================================================================================

    public override double FirstParameter() => myCurve!.FirstParameter();

    //=================================================================================================

    public override double LastParameter() => myCurve!.LastParameter();

    //=================================================================================================

    /// <exception cref="Standard_NotImplemented">Always, as in OCCT.</exception>
    public override GeomAbs_Shape Continuity() =>
        throw new Standard_NotImplemented("ProjLib_ProjectedCurve::Continuity() - method is not implemented");

    //=================================================================================================

    /// <exception cref="Standard_NotImplemented">Always, as in OCCT.</exception>
    public override int NbIntervals(GeomAbs_Shape S) =>
        throw new Standard_NotImplemented("ProjLib_ProjectedCurve::NbIntervals() - method is not implemented");

    //=================================================================================================

    /// <exception cref="Standard_NotImplemented">Always, as in OCCT.</exception>
    public override void Intervals(NCollection_Array1<double> T, GeomAbs_Shape S) =>
        throw new Standard_NotImplemented("ProjLib_ProjectedCurve::Intervals() - method is not implemented");

    //=================================================================================================

    /// <exception cref="Standard_NotImplemented">Always, as in OCCT.</exception>
    public override bool IsClosed() =>
        throw new Standard_NotImplemented("ProjLib_ProjectedCurve::IsClosed() - method is not implemented");

    //=================================================================================================

    public override bool IsPeriodic() => myResult.IsPeriodic();

    //=================================================================================================

    /// <exception cref="Standard_NotImplemented">Always, as in OCCT.</exception>
    public override double Period() =>
        throw new Standard_NotImplemented("ProjLib_ProjectedCurve::Period() - method is not implemented");

    //=================================================================================================

    /// <exception cref="Standard_NotImplemented">Always, as in OCCT.</exception>
    public override gp_Pnt2d Value(double U) =>
        throw new Standard_NotImplemented("ProjLib_ProjectedCurve::Value() - method is not implemented");

    //=================================================================================================

    /// <exception cref="Standard_NotImplemented">Always, as in OCCT.</exception>
    public override void D0(double U, out gp_Pnt2d P) =>
        throw new Standard_NotImplemented("ProjLib_ProjectedCurve::D0() - method is not implemented");

    //=================================================================================================

    /// <exception cref="Standard_NotImplemented">Always, as in OCCT.</exception>
    public override void D1(double U, out gp_Pnt2d P, out gp_Vec2d V) =>
        throw new Standard_NotImplemented("ProjLib_ProjectedCurve::D1() - method is not implemented");

    //=================================================================================================

    /// <exception cref="Standard_NotImplemented">Always, as in OCCT.</exception>
    public override void D2(double U, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2) =>
        throw new Standard_NotImplemented("ProjLib_ProjectedCurve::D2() - method is not implemented");

    //=================================================================================================

    /// <exception cref="Standard_NotImplemented">Always, as in OCCT.</exception>
    public override void D3(double U, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2, out gp_Vec2d V3) =>
        throw new Standard_NotImplemented("ProjLib_ProjectedCurve::D3() - method is not implemented");

    //=================================================================================================

    /// <exception cref="Standard_NotImplemented">Always, as in OCCT.</exception>
    public override gp_Vec2d DN(double U, int N) =>
        throw new Standard_NotImplemented("ProjLib_ProjectedCurve::DN() - method is not implemented");

    //=================================================================================================

    /// <exception cref="Standard_NotImplemented">Always, as in OCCT.</exception>
    public override double Resolution(double R3d) =>
        throw new Standard_NotImplemented("ProjLib_ProjectedCurve::Resolution() - method is not implemented");

    //=================================================================================================

    /// <summary>The type of the result: Line, Circle, Ellipse, Hyperbola, Parabola, BezierCurve, BSplineCurve or
    /// OtherCurve.</summary>
    public override GeomAbs_CurveType GetType() => myResult.GetType();

    //=================================================================================================

    public override gp_Lin2d Line() => myResult.Line();

    //=================================================================================================

    public override gp_Circ2d Circle() => myResult.Circle();

    //=================================================================================================

    public override gp_Elips2d Ellipse() => myResult.Ellipse();

    //=================================================================================================

    public override gp_Hypr2d Hyperbola() => myResult.Hyperbola();

    //=================================================================================================

    public override gp_Parab2d Parabola() => myResult.Parabola();

    //=================================================================================================

    /// <exception cref="Standard_NoSuchObject">The result is not a B-spline or Bezier curve.</exception>
    public override int Degree()
    {
        Standard_NoSuchObject_Raise_if((GetType() != GeomAbs_BSplineCurve) && (GetType() != GeomAbs_BezierCurve),
                                       "ProjLib_ProjectedCurve:Degree");
        if (GetType() == GeomAbs_BSplineCurve)
        {
            return myResult.InternalBSpline()!.Degree();
        }
        else if (GetType() == GeomAbs_BezierCurve)
        {
            return myResult.InternalBezier()!.Degree();
        }

        // porting WNT
        return 0;
    }

    //=================================================================================================

    /// <exception cref="Standard_NoSuchObject">The result is not a B-spline or Bezier curve.</exception>
    public override bool IsRational()
    {
        Standard_NoSuchObject_Raise_if((GetType() != GeomAbs_BSplineCurve) && (GetType() != GeomAbs_BezierCurve),
                                       "ProjLib_ProjectedCurve:IsRational");
        if (GetType() == GeomAbs_BSplineCurve)
        {
            return myResult.InternalBSpline()!.IsRational();
        }
        else if (GetType() == GeomAbs_BezierCurve)
        {
            return myResult.InternalBezier()!.IsRational();
        }
        // porting WNT
        return false;
    }

    //=================================================================================================

    /// <exception cref="Standard_NoSuchObject">The result is not a B-spline or Bezier curve.</exception>
    public override int NbPoles()
    {
        Standard_NoSuchObject_Raise_if((GetType() != GeomAbs_BSplineCurve) && (GetType() != GeomAbs_BezierCurve),
                                       "ProjLib_ProjectedCurve:NbPoles");
        if (GetType() == GeomAbs_BSplineCurve)
        {
            return myResult.InternalBSpline()!.NbPoles();
        }
        else if (GetType() == GeomAbs_BezierCurve)
        {
            return myResult.InternalBezier()!.NbPoles();
        }

        // porting WNT
        return 0;
    }

    //=================================================================================================

    /// <exception cref="Standard_NoSuchObject">The result is not a B-spline curve.</exception>
    public override int NbKnots()
    {
        Standard_NoSuchObject_Raise_if(GetType() != GeomAbs_BSplineCurve, "ProjLib_ProjectedCurve:NbKnots");
        return myResult.InternalBSpline()!.NbKnots();
    }

    //=================================================================================================

    /// <summary>The Bezier result, frozen (the same object at each call; null when the result is no Bezier curve).
    /// It is not trimmed to the curve's range.</summary>
    public override Geom2d_BezierCurve Bezier() => myResult.Bezier()!;

    //=================================================================================================

    /// <summary>The B-spline result, frozen (the same object at each call; null when the result is no B-spline
    /// curve). It is not trimmed to the curve's range.</summary>
    public override Geom2d_BSplineCurve BSpline() => myResult.BSpline()!;

    /// <summary>The Bezier result, unfrozen, for ported code that discards the projection (GeomProjLib).</summary>
    internal Geom2d_BezierCurve? InternalBezier() => myResult.InternalBezier();

    /// <summary>The B-spline result, unfrozen, for ported code that discards the projection (GeomProjLib).</summary>
    internal Geom2d_BSplineCurve? InternalBSpline() => myResult.InternalBSpline();

    //=================================================================================================

    /// <exception cref="Standard_NotImplemented">Always, as in OCCT.</exception>
    public override Adaptor2d_Curve2d Trim(double First, double Last, double Tol) =>
        throw new Standard_NotImplemented("ProjLib_ProjectedCurve::Trim() - method is not implemented");
}
