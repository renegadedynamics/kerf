// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtCS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtCS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//  Modified by skv - Thu Jul  7 12:29:34 2005 OCC9134

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// The extremum distances between a 3D curve and a surface on parameter ranges. A line against a plane, cylinder or
/// sphere, a circle against a plane, cylinder or sphere and a hyperbola against a plane are solved analytically
/// (<see cref="Extrema_ExtElCS"/>); other pairs by sampling and local search (Extrema_GenExtCS), with the curve's
/// sharp points projected on the surface. A curve parallel to the surface reports <see cref="IsParallel"/> with one
/// distance.
/// </summary>
/// <remarks>
/// An algorithm object (decision 117): construct it with the curve and the surface, or <c>Initialize</c> with the
/// surface and <c>Perform</c> with the curve. Keeps a reference to the surface, as OCCT keeps the pointer: the
/// caller's adaptor must outlive the algorithm. OCCT disallows copies.
/// </remarks>
public class Extrema_ExtCS
{
    private Adaptor3d_Surface? myS;
    private bool myDone;
    private bool myIsPar;
    private readonly Extrema_ExtElCS myExtElCS = new();
    private readonly NCollection_Sequence<Extrema_POnSurf> myPOnS = new();
    private readonly NCollection_Sequence<Extrema_POnCurv> myPOnC = new();
    private double myuinf;
    private double myusup;
    private double myvinf;
    private double myvsup;
    private double mytolC;
    private double mytolS;
    private double myucinf;
    private double myucsup;
    private readonly NCollection_Sequence<double> mySqDist = new();
    private GeomAbs_SurfaceType myStype;

    /// <summary>An empty algorithm.</summary>
    public Extrema_ExtCS()
    {
        myS = null;
        myDone = false;
        myIsPar = false;
        myuinf = 0.0;
        myusup = 0.0;
        myvinf = 0.0;
        myvsup = 0.0;
        mytolC = 0.0;
        mytolS = 0.0;
        myucinf = 0.0;
        myucsup = 0.0;
        myStype = GeomAbs_OtherSurface;
    }

    /// <summary>It calculates all the distances between C and S.</summary>
    public Extrema_ExtCS(Adaptor3d_Curve C, Adaptor3d_Surface S, double TolC, double TolS)
    {
        Initialize(S, TolC, TolS);
        Perform(C, C.FirstParameter(), C.LastParameter());
    }

    /// <summary>
    /// It calculates all the distances between C and S. UCinf and UCmax are the start and end parameters of the
    /// curve.
    /// </summary>
    public Extrema_ExtCS(Adaptor3d_Curve C,
                         Adaptor3d_Surface S,
                         double UCinf,
                         double UCsup,
                         double Uinf,
                         double Usup,
                         double Vinf,
                         double Vsup,
                         double TolC,
                         double TolS)
    {
        Initialize(S, Uinf, Usup, Vinf, Vsup, TolC, TolS);
        Perform(C, UCinf, UCsup);
    }

    /// <summary>Initializes the fields of the algorithm.</summary>
    public void Initialize(Adaptor3d_Surface S, double TolC, double TolS)
    {
        Initialize(S, S.FirstUParameter(), S.LastUParameter(), S.FirstVParameter(), S.LastVParameter(), TolC, TolS);
    }

    /// <summary>Initializes the fields of the algorithm.</summary>
    public void Initialize(Adaptor3d_Surface S, double Uinf, double Usup, double Vinf, double Vsup, double TolC, double TolS)
    {
        myS = S;
        myIsPar = false;
        myuinf = Uinf;
        myusup = Usup;
        myvinf = Vinf;
        myvsup = Vsup;
        mytolC = TolC;
        mytolS = TolS;
        myStype = myS.GetType();
    }

    /// <summary>Computes the distances. An exception is raised if the fields have not been initialized.</summary>
    public void Perform(Adaptor3d_Curve C, double Uinf, double Usup)
    {
        // OCCT dereferences the null surface pointer; the port throws.
        Adaptor3d_Surface aS = myS ?? throw new StdFail_NotDone("Extrema_ExtCS::Perform: not initialized");

        myucinf = Uinf;
        myucsup = Usup;
        myPOnS.Clear();
        myPOnC.Clear();
        mySqDist.Clear();
        int i, j;
        int NbT, NbU, NbV;
        NbT = 12;
        NbU = NbV = 10;
        GeomAbs_CurveType myCtype = C.GetType();

        myDone = false;
        // Try analytic computation of extrema
        bool isComputeAnalytic = true;

        // OCCT's switch with [[fallthrough]]: the cases that reach the generic line/surface block, and the circle
        // cases falling through to the hyperbola test and to `default`, are written out.
        switch (myCtype)
        {
            case GeomAbs_Line:
            {
                if (myStype == GeomAbs_Sphere)
                {
                    myExtElCS.Perform(C.Line(), aS.Sphere());
                    break;
                }
                if (myStype == GeomAbs_Cylinder)
                {
                    myExtElCS.Perform(C.Line(), aS.Cylinder());
                    break;
                }
                if (myStype == GeomAbs_Plane)
                {
                    myExtElCS.Perform(C.Line(), aS.Plane());
                    if (myExtElCS.IsParallel())
                    {
                        break;
                    }
                    // [[fallthrough]]
                }

                // GeomAbs_Plane (not parallel), Torus, Cone, BezierSurface, BSplineSurface, SurfaceOfRevolution,
                // SurfaceOfExtrusion, OffsetSurface, OtherSurface
                {
                    double cfirst = myucinf, clast = myucsup;
                    double ufirst = aS.FirstUParameter(), ulast = aS.LastUParameter(),
                           vfirst = aS.FirstVParameter(), vlast = aS.LastVParameter();

                    if (!(Precision.IsInfinite(ufirst) || Precision.IsInfinite(ulast)
                          || Precision.IsInfinite(vfirst) || Precision.IsInfinite(vlast)))
                    {
                        double tmin = Precision.Infinite(), tmax = -tmin;
                        Bnd_Box aSurfBox = new();
                        BndLib_AddSurface.Add(aS, ufirst, ulast, vfirst, vlast, Precision.Confusion(), ref aSurfBox);
                        aSurfBox.Get(out double xmin, out double ymin, out double zmin, out double xmax, out double ymax, out double zmax);
                        gp_Lin aLin = C.Line();
                        double aParOnLin;
                        Span<gp_Pnt> aLimPntArray =
                        [
                            new gp_Pnt(xmin, ymin, zmin),
                            new gp_Pnt(xmax, ymin, zmin),
                            new gp_Pnt(xmin, ymax, zmin),
                            new gp_Pnt(xmax, ymax, zmin),
                            new gp_Pnt(xmin, ymin, zmax),
                            new gp_Pnt(xmax, ymin, zmax),
                            new gp_Pnt(xmin, ymax, zmax),
                            new gp_Pnt(xmax, ymax, zmax),
                        ];

                        for (i = 0; i <= 7; i++)
                        {
                            aParOnLin = ElCLib.Parameter(aLin, aLimPntArray[i]);
                            tmin = Min(aParOnLin, tmin);
                            tmax = Max(aParOnLin, tmax);
                        }
                        cfirst = Max(cfirst, tmin);
                        clast = Min(clast, tmax);
                    }

                    if (aS.IsUPeriodic())
                    {
                        NbU = 13;
                    }
                    if (aS.IsVPeriodic())
                    {
                        NbV = 13;
                    }

                    if (clast - cfirst <= Precision.Confusion())
                    {
                        double aCPar = (cfirst + clast) / 2.0;
                        gp_Pnt aPm = C.Value(aCPar);
                        // TODO(4A): Extrema_ExtPS
                        Extrema_ExtPS anExtPS = new(aPm, aS, ufirst, ulast, vfirst, vlast, mytolS, mytolS,
                                                    Extrema_ExtFlag.Extrema_ExtFlag_MIN);
                        myDone = anExtPS.IsDone();
                        if (myDone)
                        {
                            int NbExt = anExtPS.NbExt();
                            double T = aCPar;
                            Extrema_POnCurv PC = new();
                            Extrema_POnSurf PS;
                            for (i = 1; i <= NbExt; i++)
                            {
                                PS = anExtPS.Point(i);
                                PS.Parameter(out double U, out double V);
                                // OCCT passes the point of a default Extrema_POnCurv (the origin), not aPm, as the
                                // point on the curve (occt-issues.md).
                                AddSolution(C, T, U, V, PC.Value(), PS.Value(), anExtPS.SquareDistance(i));
                            }
                        }
                        return;
                    }

                    // TODO(4A): Extrema_GenExtCS
                    Extrema_GenExtCS Ext = new(C, aS, NbT, NbU, NbV, cfirst, clast, ufirst, ulast, vfirst, vlast,
                                               mytolC, mytolS);

                    myDone = Ext.IsDone();
                    if (myDone)
                    {
                        int NbExt = Ext.NbExt();
                        double T;
                        Extrema_POnCurv PC;
                        Extrema_POnSurf PS;
                        for (i = 1; i <= NbExt; i++)
                        {
                            PC = Ext.PointOnCurve(i);
                            PS = Ext.PointOnSurface(i);
                            T = PC.Parameter();
                            PS.Parameter(out double U, out double V);
                            AddSolution(C, T, U, V, PC.Value(), PS.Value(), Ext.SquareDistance(i));
                        }
                    }
                    return;
                }
            }
            //  Modified by skv - Thu Jul  7 12:29:34 2005 OCC9134 Begin
            case GeomAbs_Circle:
            {
                if (myStype == GeomAbs_Cylinder)
                {
                    myExtElCS.Perform(C.Circle(), aS.Cylinder());
                    break;
                }
                else if (myStype == GeomAbs_Plane)
                {
                    myExtElCS.Perform(C.Circle(), aS.Plane());
                    break;
                }
                else if (myStype == GeomAbs_Sphere)
                {
                    myExtElCS.Perform(C.Circle(), aS.Sphere());
                    break;
                }
                // [[fallthrough]] to the hyperbola case, whose test fails for a circle, then to default
                isComputeAnalytic = false;
                break;
            }
            case GeomAbs_Hyperbola:
            {
                if (myCtype == GeomAbs_Hyperbola && myStype == GeomAbs_Plane)
                {
                    //  Modified by skv - Thu Jul  7 12:29:34 2005 OCC9134 End
                    myExtElCS.Perform(C.Hyperbola(), aS.Plane());
                    break;
                }
                // [[fallthrough]]
                isComputeAnalytic = false;
                break;
            }
            default:
            {
                isComputeAnalytic = false;
                break;
            }
        }

        if (isComputeAnalytic)
        {
            if (myExtElCS.IsDone())
            {
                myDone = true;
                myIsPar = myExtElCS.IsParallel();
                if (myIsPar)
                {
                    mySqDist.Append(myExtElCS.SquareDistance(1));
                }
                else
                {
                    int NbExt = myExtElCS.NbExt();
                    for (i = 1; i <= NbExt; i++)
                    {
                        myExtElCS.Points(i, out Extrema_POnCurv PC, out Extrema_POnSurf PS);
                        double Ucurve = PC.Parameter();
                        PS.Parameter(out double U, out double V);
                        AddSolution(C, Ucurve, U, V, PC.Value(), PS.Value(), myExtElCS.SquareDistance(i));
                    }

                    if (mySqDist.Length() == 0 && NbExt > 0)
                    {
                        // Analytical extrema seem to be out of curve/surface boundaries.
                        // Try extremity points of curve.
                        Span<gp_Pnt> aPOnC = [new gp_Pnt(), new gp_Pnt()];
                        Span<gp_Pnt> aPOnS = [new gp_Pnt(), new gp_Pnt()];
                        Span<double> aT = [myucinf, myucsup];
                        Span<double> U = [0.0, 0.0];
                        Span<double> V = [0.0, 0.0];
                        Span<double> aDist = [-1, -1];
                        for (i = 0; i < 2; ++i)
                        {
                            if (Precision.IsInfinite(aT[i]))
                            {
                                continue;
                            }

                            aPOnC[i] = C.Value(aT[i]);
                            switch (myStype)
                            {
                                case GeomAbs_Plane:
                                {
                                    ElSLib.Parameters(aS.Plane(), aPOnC[i], out U[i], out V[i]);
                                    aPOnS[i] = ElSLib.Value(U[i], V[i], aS.Plane());
                                    break;
                                }
                                case GeomAbs_Sphere:
                                {
                                    ElSLib.Parameters(aS.Sphere(), aPOnC[i], out U[i], out V[i]);
                                    aPOnS[i] = ElSLib.Value(U[i], V[i], aS.Sphere());
                                    break;
                                }
                                case GeomAbs_Cylinder:
                                {
                                    ElSLib.Parameters(aS.Cylinder(), aPOnC[i], out U[i], out V[i]);
                                    aPOnS[i] = ElSLib.Value(U[i], V[i], aS.Cylinder());
                                    break;
                                }
                                case GeomAbs_Torus:
                                {
                                    ElSLib.Parameters(aS.Torus(), aPOnC[i], out U[i], out V[i]);
                                    aPOnS[i] = ElSLib.Value(U[i], V[i], aS.Torus());
                                    break;
                                }
                                case GeomAbs_Cone:
                                {
                                    ElSLib.Parameters(aS.Cone(), aPOnC[i], out U[i], out V[i]);
                                    aPOnS[i] = ElSLib.Value(U[i], V[i], aS.Cone());
                                    break;
                                }
                                default:
                                    continue;
                            }

                            aDist[i] = aPOnC[i].SquareDistance(aPOnS[i]);
                        }

                        Span<bool> bAdd = [false, false];

                        // Choose solution to add
                        if (aDist[0] >= 0.0 && aDist[1] >= 0.0)
                        {
                            double aDiff = aDist[0] - aDist[1];
                            // Both computed -> take only minimal
                            if (Math.Abs(aDiff) < Precision.Confusion())
                            {
                                // Add both
                                bAdd[0] = bAdd[1] = true;
                            }
                            else if (aDiff < 0)
                            {
                                // Add first
                                bAdd[0] = true;
                            }
                            else
                            {
                                // Add second
                                bAdd[1] = true;
                            }
                        }
                        else if (aDist[0] >= 0.0)
                        {
                            // Add first
                            bAdd[0] = true;
                        }
                        else if (aDist[1] >= 0.0)
                        {
                            // Add second
                            bAdd[1] = true;
                        }

                        for (i = 0; i < 2; ++i)
                        {
                            if (bAdd[i])
                            {
                                AddSolution(C, aT[i], U[i], V[i], aPOnC[i], aPOnS[i], aDist[i]);
                            }
                        }
                    }
                }
                return;
            }
        }

        // Elementary extrema is not done, try generic solution
        // TODO(4A): Extrema_GenExtCS
        Extrema_GenExtCS Ext2 = new();
        Ext2.Initialize(aS, NbU, NbV, mytolS);
        if (myCtype == GeomAbs_Hyperbola)
        {
            double tmin = Max(-20.0, C.FirstParameter());
            double tmax = Min(20.0, C.LastParameter());
            Ext2.Perform(C, NbT, tmin, tmax, mytolC); // to avoid overflow
        }
        else
        {
            if ((myCtype == GeomAbs_Circle && NbT < 13) || (myCtype == GeomAbs_BSplineCurve && NbT < 13))
            {
                NbT = 13;
            }
            Ext2.Perform(C, NbT, mytolC);
        }

        myDone = Ext2.IsDone();
        if (myDone)
        {
            int NbExt = Ext2.NbExt();
            double T, U, V;
            Extrema_POnCurv PC;
            Extrema_POnSurf PS;
            for (i = 1; i <= NbExt; i++)
            {
                PC = Ext2.PointOnCurve(i);
                PS = Ext2.PointOnSurface(i);
                T = PC.Parameter();
                PS.Parameter(out U, out V);
                AddSolution(C, T, U, V, PC.Value(), PS.Value(), Ext2.SquareDistance(i));
            }

            // Add sharp points
            int SolNumber = mySqDist.Length();
            // OCCT casts the const curve to non-const (void* CopyC) to call NbIntervals and Intervals.
            Adaptor3d_Curve aC = C;
            int NbIntervals = aC.NbIntervals(GeomAbs_C1);
            NCollection_Array1<double> SharpPoints = new(1, NbIntervals + 1);
            aC.Intervals(SharpPoints, GeomAbs_C1);

            // TODO(4A): Extrema_ExtPS
            Extrema_ExtPS aProjPS = new();
            aProjPS.Initialize(aS,
                               aS.FirstUParameter(),
                               aS.LastUParameter(),
                               aS.FirstVParameter(),
                               aS.LastVParameter(),
                               mytolS,
                               mytolS);

            for (i = 2; i < SharpPoints.Upper(); ++i)
            {
                T = SharpPoints[i];
                gp_Pnt aPnt = C.Value(T);
                aProjPS.Perform(aPnt);
                if (!aProjPS.IsDone())
                {
                    continue;
                }
                int NbProj = aProjPS.NbExt(), jmin = 0;
                double MinSqDist = RealLast();
                for (j = 1; j <= NbProj; j++)
                {
                    double aSqDist = aProjPS.SquareDistance(j);
                    if (aSqDist < MinSqDist)
                    {
                        MinSqDist = aSqDist;
                        jmin = j;
                    }
                }
                if (jmin != 0)
                {
                    aProjPS.Point(jmin).Parameter(out U, out V);
                    AddSolution(C, T, U, V, aPnt, aProjPS.Point(jmin).Value(), MinSqDist);
                }
            }
            // Cut sharp solutions to keep only minimum and maximum
            int imin = SolNumber + 1, imax = mySqDist.Length();
            for (i = SolNumber + 1; i <= mySqDist.Length(); i++)
            {
                if (mySqDist[i] < mySqDist[imin])
                {
                    imin = i;
                }
                if (mySqDist[i] > mySqDist[imax])
                {
                    imax = i;
                }
            }
            if (mySqDist.Length() > SolNumber + 2)
            {
                double MinSqDist = mySqDist[imin];
                double MaxSqDist = mySqDist[imax];
                Extrema_POnCurv MinPC = myPOnC[imin];
                Extrema_POnCurv MaxPC = myPOnC[imax];
                Extrema_POnSurf MinPS = myPOnS[imin];
                Extrema_POnSurf MaxPS = myPOnS[imax];

                mySqDist.Remove(SolNumber + 1, mySqDist.Length());
                myPOnC.Remove(SolNumber + 1, myPOnC.Length());
                myPOnS.Remove(SolNumber + 1, myPOnS.Length());

                mySqDist.Append(MinSqDist);
                myPOnC.Append(MinPC);
                myPOnS.Append(MinPS);
                mySqDist.Append(MaxSqDist);
                myPOnC.Append(MaxPC);
                myPOnS.Append(MaxPS);
            }
        }
    }

    /// <summary>Returns True if the distances are found.</summary>
    public bool IsDone() => myDone;

    /// <summary>Returns True if the curve is on a parallel surface.</summary>
    public bool IsParallel()
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone();
        }

        return myIsPar;
    }

    /// <summary>Returns the value of the Nth resulting square distance.</summary>
    public double SquareDistance(int N)
    {
        if (N < 1 || N > NbExt())
        {
            throw new Standard_OutOfRange();
        }

        return mySqDist.Value(N);
    }

    /// <summary>Returns the number of extremum distances.</summary>
    public int NbExt()
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone();
        }

        return mySqDist.Length();
    }

    /// <summary>Returns the points of the Nth resulting distance.</summary>
    /// <remarks>
    /// When the curve is parallel to the surface, OCCT stores one distance and no points, and reading them fails;
    /// the port throws Standard_OutOfRange from the sequence there.
    /// </remarks>
    public void Points(int N, out Extrema_POnCurv P1, out Extrema_POnSurf P2)
    {
        if (N < 1 || N > NbExt())
        {
            throw new Standard_OutOfRange();
        }

        P1 = myPOnC.Value(N);
        P2 = myPOnS.Value(N);
    }

    private bool AddSolution(Adaptor3d_Curve theCurve,
                             double aT,
                             double aU,
                             double aV,
                             in gp_Pnt PointOnCurve,
                             in gp_Pnt PointOnSurf,
                             double SquareDist)
    {
        bool Added = false;

        double T = aT, U = aU, V = aV;

        if (theCurve.IsPeriodic())
        {
            T = ElCLib.InPeriod(T, myucinf, myucinf + theCurve.Period());
        }
        if (myS!.IsUPeriodic())
        {
            U = ElCLib.InPeriod(U, myuinf, myuinf + myS.UPeriod());
        }
        if (myS.IsVPeriodic())
        {
            V = ElCLib.InPeriod(V, myvinf, myvinf + myS.VPeriod());
        }

        Extrema_POnCurv aPC = new();
        Extrema_POnSurf aPS;
        if ((myucinf - T) <= mytolC && (T - myucsup) <= mytolC && (myuinf - U) <= mytolS && (U - myusup) <= mytolS
            && (myvinf - V) <= mytolS && (V - myvsup) <= mytolS)
        {
            bool IsNewSolution = true;
            for (int j = 1; j <= mySqDist.Length(); j++)
            {
                aPC = myPOnC[j];
                aPS = myPOnS[j];
                double Tj = aPC.Parameter();
                aPS.Parameter(out double Uj, out double Vj);
                if (Math.Abs(T - Tj) <= mytolC && Math.Abs(U - Uj) <= mytolS && Math.Abs(V - Vj) <= mytolS)
                {
                    IsNewSolution = false;
                    break;
                }
            }
            if (IsNewSolution)
            {
                mySqDist.Append(SquareDist);
                aPC.SetValues(T, PointOnCurve);
                myPOnC.Append(aPC);
                myPOnS.Append(new Extrema_POnSurf(U, V, PointOnSurf));
                Added = true;
            }
        }
        return Added;
    }
}
