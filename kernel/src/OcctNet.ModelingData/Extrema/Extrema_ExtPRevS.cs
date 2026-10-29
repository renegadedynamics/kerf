// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPRevS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPRevS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>
/// The extrema of the distance between a point and a surface of revolution: analytically when the basis curve is a
/// line or a conic whose plane contains the axis, with Extrema_GenExtPS otherwise.
/// </summary>
/// <remarks>The surface adaptor is kept, as OCCT keeps its handle.</remarks>
public class Extrema_ExtPRevS : Standard_Transient
{
    private GeomAdaptor_SurfaceOfRevolution? myS;
    private double myvinf;
    private double myvsup;
    private double mytolv;
    private gp_Ax2 myPosition = new();
    private readonly Extrema_GenExtPS myExtPS = new();
    private bool myIsAnalyticallyComputable;
    private bool myDone;
    private int myNbExt;
    private readonly double[] mySqDist = new double[8];
    private readonly Extrema_POnSurf[] myPoint = [new(), new(), new(), new(), new(), new(), new(), new()];

    private static gp_Ax2 GetPosition(GeomAdaptor_SurfaceOfRevolution S)
    {
        Adaptor3d_Curve C = S.BasisCurve();

        switch (C.GetType())
        {
            case GeomAbs_Line:
                {
                    gp_Lin L = C.Line();
                    gp_Dir N = S.AxeOfRevolution().Direction();
                    if (N.IsParallel(L.Direction(), Precision.Angular()))
                    {
                        gp_Vec OO = new(L.Location(), S.AxeOfRevolution().Location());
                        if (OO.Magnitude() <= gp.Resolution())
                        {
                            OO = new gp_Vec(L.Location(), ElCLib.Value(100, L));
                            if (N.IsParallel(new gp_Dir(OO), Precision.Angular()))
                            {
                                return new gp_Ax2(); // Line and axe of revolution coincide
                            }
                        }
                        N.Cross(new gp_Dir(OO));
                    }
                    else
                    {
                        N.Cross(L.Direction());
                    }
                    return new gp_Ax2(L.Location(), N, L.Direction());
                }
            case GeomAbs_Circle:
                return C.Circle().Position();
            case GeomAbs_Ellipse:
                return C.Ellipse().Position();
            case GeomAbs_Hyperbola:
                return C.Hyperbola().Position();
            case GeomAbs_Parabola:
                return C.Parabola().Position();
            default:
                return new gp_Ax2();
        }
    }

    private static bool HasSingularity(GeomAdaptor_SurfaceOfRevolution S)
    {
        Adaptor3d_Curve C = S.BasisCurve();
        gp_Dir N = S.AxeOfRevolution().Direction();
        gp_Pnt P = S.AxeOfRevolution().Location();

        gp_Lin L = new(P, N);

        P = C.Value(C.FirstParameter());

        if (L.SquareDistance(P) < Precision.SquareConfusion())
        {
            return true;
        }

        P = C.Value(C.LastParameter());

        return L.SquareDistance(P) < Precision.SquareConfusion();
    }

    private static void PerformExtPElC(Extrema_ExtPElC E, in gp_Pnt P, Adaptor3d_Curve C, double Tol)
    {
        switch (C.GetType())
        {
            case GeomAbs_Hyperbola:
                E.Perform(P, C.Hyperbola(), Tol, -Precision.Infinite(), Precision.Infinite());
                return;
            case GeomAbs_Line:
                E.Perform(P, C.Line(), Tol, -Precision.Infinite(), Precision.Infinite());
                return;
            case GeomAbs_Circle:
                E.Perform(P, C.Circle(), Tol, 0.0, 2.0 * M_PI);
                return;
            case GeomAbs_Ellipse:
                E.Perform(P, C.Ellipse(), Tol, 0.0, 2.0 * M_PI);
                return;
            case GeomAbs_Parabola:
                E.Perform(P, C.Parabola(), Tol, -Precision.Infinite(), Precision.Infinite());
                return;
            default:
                return;
        }
    }

    private static bool IsCaseAnalyticallyComputable(GeomAbs_CurveType theType, in gp_Ax2 theCurvePos,
        in gp_Ax1 AxeOfRevolution)
    {
        // check type
        switch (theType)
        {
            case GeomAbs_Line:
            case GeomAbs_Circle:
            case GeomAbs_Ellipse:
            case GeomAbs_Hyperbola:
            case GeomAbs_Parabola:
                break;
            default:
                return false;
        }
        //  the axe of revolution must be in the plane of the curve.
        gp_Pln pl = new(theCurvePos.Location(), theCurvePos.Direction());
        gp_Pnt p1 = AxeOfRevolution.Location();
        double dist = 100.0, dist2 = dist * dist;
        double aThreshold = Precision.Angular() * Precision.Angular() * dist2;
        gp_Pnt p2 = new(AxeOfRevolution.Location().XYZ() + dist * AxeOfRevolution.Direction().XYZ());

        return (pl.SquareDistance(p1) < aThreshold) && (pl.SquareDistance(p2) < aThreshold);
    }

    private static bool IsOriginalPnt(in gp_Pnt P, Extrema_POnSurf[] Points, int NbPoints)
    {
        for (int i = 1; i <= NbPoints; i++)
        {
            if (Points[i - 1].Value().IsEqual(P, Precision.Confusion()))
            {
                return false;
            }
        }
        return true;
    }

    private static bool IsExtremum(double U, double V, in gp_Pnt P, Adaptor3d_Surface S, out gp_Pnt E,
        out double Dist2, bool IsVSup, bool IsMin)
    {
        E = S.Value(U, V);
        Dist2 = P.SquareDistance(E);
        if (IsMin)
        {
            return (Dist2 < P.SquareDistance(S.Value(U + 1, V)) && Dist2 < P.SquareDistance(S.Value(U - 1, V))
                    && Dist2 < P.SquareDistance(S.Value(U, IsVSup ? V - 1 : V + 1)));
        }
        else
        {
            return (Dist2 > P.SquareDistance(S.Value(U + 1, V)) && Dist2 > P.SquareDistance(S.Value(U - 1, V))
                    && Dist2 > P.SquareDistance(S.Value(U, IsVSup ? V - 1 : V + 1)));
        }
    }

    public Extrema_ExtPRevS()
    {
        myvinf = myvsup = 0.0;
        mytolv = Precision.Confusion();
        myDone = false;
        myNbExt = 0;
        myIsAnalyticallyComputable = false;

        for (int i = 0; i < 8; i++)
        {
            mySqDist[i] = RealLast();
        }
    }

    /// <summary>The extrema between P and S on [Umin, Usup] x [Vmin, Vsup].</summary>
    public Extrema_ExtPRevS(in gp_Pnt theP, GeomAdaptor_SurfaceOfRevolution theS, double theUmin, double theUsup,
        double theVmin, double theVsup, double theTolU, double theTolV)
    {
        Initialize(theS, theUmin, theUsup, theVmin, theVsup, theTolU, theTolV);

        Perform(theP);
    }

    /// <summary>The extrema between P and S on S's whole range.</summary>
    public Extrema_ExtPRevS(in gp_Pnt theP, GeomAdaptor_SurfaceOfRevolution theS, double theTolU, double theTolV)
    {
        Initialize(theS, theS.FirstUParameter(), theS.LastUParameter(), theS.FirstVParameter(), theS.LastVParameter(),
            theTolU, theTolV);

        Perform(theP);
    }

    public void Initialize(GeomAdaptor_SurfaceOfRevolution theS, double theUmin, double theUsup, double theVmin,
        double theVsup, double theTolU, double theTolV)
    {
        myvinf = theVmin;
        myvsup = theVsup;
        mytolv = theTolV;

        myDone = false;
        myNbExt = 0;
        // OCCT: reset before the test below, so that initialising again with the same surface makes the analytic
        // case use Extrema_GenExtPS (which is then initialised)
        myIsAnalyticallyComputable = false;

        Adaptor3d_Curve anACurve = theS.BasisCurve();

        if (!ReferenceEquals(myS, theS))
        {
            myS = theS;
            myPosition = GetPosition(theS);
            myIsAnalyticallyComputable =
                IsCaseAnalyticallyComputable(anACurve.GetType(), myPosition, theS.AxeOfRevolution());
        }

        if (!myIsAnalyticallyComputable)
        {
            int aNbu = 32, aNbv = 32;

            if (HasSingularity(theS))
            {
                aNbv = 100;
            }

            myExtPS.Initialize(theS, aNbu, aNbv, theUmin, theUsup, theVmin, theVsup, theTolU, theTolV);
        }
    }

    public void Perform(in gp_Pnt P)
    {
        myDone = false;
        myNbExt = 0;

        if (!myIsAnalyticallyComputable)
        {
            myExtPS.Perform(P);
            myDone = myExtPS.IsDone();
            myNbExt = myExtPS.NbExt();
            return;
        }

        Adaptor3d_Curve anACurve = myS!.BasisCurve();

        gp_Ax1 Ax = myS.AxeOfRevolution();
        gp_Vec Dir = new(Ax.Direction()), Z = new(myPosition.Direction());
        gp_Pnt O = Ax.Location();

        double OPdir = new gp_Vec(O, P).Dot(Dir);
        gp_Pnt Pp = P.Translated(Dir.Multiplied(-OPdir));
        if (O.IsEqual(Pp, Precision.Confusion()))
        { // P is on the AxeOfRevolution
            return;
        }

        double U, V;
        gp_Pnt P1, Ppp;
        double OPpz = new gp_Vec(O, Pp).Dot(Z);
        if (Math.Abs(OPpz) <= gp.Resolution())
        {
            Ppp = Pp;
            U = 0;
        }
        else
        {
            Ppp = Pp.Translated(Z.Multiplied(-OPpz));
            if (O.IsEqual(Ppp, Precision.Confusion()))
            {
                U = M_PI / 2;
            }
            else
            {
                U = new gp_Vec(O, Ppp).AngleWithRef(new gp_Vec(O, Pp), Dir);
            }
        }

        gp_Vec OPpp = new(O, Ppp), OPq = new(O, myS.Value(M_PI / 2, 0));
        if (U != M_PI / 2)
        {
            if (Math.Abs(OPq.Magnitude()) <= gp.Resolution())
            {
                OPq = new gp_Vec(O, myS.Value(M_PI / 2, anACurve.LastParameter() / 10));
            }
            if (OPpp.AngleWithRef(OPq, Dir) < 0)
            {
                U += M_PI;
            }
        }

        gp_Trsf T = new();
        T.SetRotation(Ax, -U);
        P1 = P.Transformed(T);

        gp_Pnt E;
        double Dist2;
        int i;

        Extrema_ExtPElC anExt = new();
        PerformExtPElC(anExt, P1, anACurve, mytolv);

        if (anExt.IsDone())
        {
            myDone = true;
            for (i = 1; i <= anExt.NbExt(); i++)
            {
                Extrema_POnCurv POC = anExt.Point(i);
                V = POC.Parameter();
                if (V > myvsup)
                {
                    double newV = myvsup;

                    if ((anACurve.GetType() == GeomAbs_Circle) || (anACurve.GetType() == GeomAbs_Ellipse))
                    {
                        newV = ElCLib.InPeriod(V, myvinf, myvinf + 2.0 * M_PI);

                        if (newV > myvsup)
                        {
                            newV -= 2.0 * M_PI;

                            if (newV + mytolv < myvinf)
                            {
                                newV = myvsup;
                            }
                            else if (newV < myvinf)
                            {
                                newV = myvinf;
                            }
                        }
                    }
                    V = newV;

                    if (!IsExtremum(U, V, P, myS, out E, out Dist2, true, anExt.IsMin(i)))
                    {
                        continue;
                    }
                }
                else if (V < myvinf)
                {
                    double newV = myvinf;

                    if ((anACurve.GetType() == GeomAbs_Circle) || (anACurve.GetType() == GeomAbs_Ellipse))
                    {
                        newV = ElCLib.InPeriod(V, myvsup - 2.0 * M_PI, myvsup);

                        if (newV < myvinf)
                        {
                            newV += 2.0 * M_PI;

                            if (newV - mytolv > myvsup)
                            {
                                newV = myvinf;
                            }
                            else if (newV > myvsup)
                            {
                                newV = myvsup;
                            }
                        }
                    }
                    V = newV;

                    if (!IsExtremum(U, V, P, myS, out E, out Dist2, false, anExt.IsMin(i)))
                    {
                        continue;
                    }
                }
                else
                {
                    E = myS.Value(U, V);
                    Dist2 = P.SquareDistance(E);
                }
                if (IsOriginalPnt(E, myPoint, myNbExt))
                {
                    myPoint[myNbExt] = new Extrema_POnSurf(U, V, E);
                    mySqDist[myNbExt] = Dist2;
                    myNbExt++;
                }
            }
        }
        T.SetRotation(Ax, M_PI);
        P1.Transform(T);

        PerformExtPElC(anExt, P1, anACurve, mytolv);
        if (anExt.IsDone())
        {
            myDone = true;

            U += M_PI;

            for (i = 1; i <= anExt.NbExt(); i++)
            {
                Extrema_POnCurv POC = anExt.Point(i);
                V = POC.Parameter();
                if (V > myvsup)
                {
                    double newV = myvsup;

                    if ((anACurve.GetType() == GeomAbs_Circle) || (anACurve.GetType() == GeomAbs_Ellipse))
                    {
                        newV = ElCLib.InPeriod(V, myvinf, myvinf + 2.0 * M_PI);

                        if (newV > myvsup)
                        {
                            newV -= 2.0 * M_PI;

                            if (newV + mytolv < myvinf)
                            {
                                newV = myvsup;
                            }
                            else if (newV < myvinf)
                            {
                                newV = myvinf;
                            }
                        }
                    }
                    V = newV;

                    if (!IsExtremum(U, V, P, myS, out E, out Dist2, true, anExt.IsMin(i)))
                    {
                        continue;
                    }
                }
                else if (V < myvinf)
                {
                    double newV = myvinf;

                    if ((anACurve.GetType() == GeomAbs_Circle) || (anACurve.GetType() == GeomAbs_Ellipse))
                    {
                        newV = ElCLib.InPeriod(V, myvsup - 2.0 * M_PI, myvsup);

                        if (newV < myvinf)
                        {
                            newV += 2.0 * M_PI;

                            if (newV - mytolv > myvsup)
                            {
                                newV = myvinf;
                            }
                            else if (newV > myvsup)
                            {
                                newV = myvsup;
                            }
                        }
                    }
                    V = newV;

                    if (!IsExtremum(U, V, P, myS, out E, out Dist2, false, anExt.IsMin(i)))
                    {
                        continue;
                    }
                }
                else
                {
                    E = myS.Value(U, V);
                    Dist2 = P.SquareDistance(E);
                }
                if (IsOriginalPnt(E, myPoint, myNbExt))
                {
                    myPoint[myNbExt] = new Extrema_POnSurf(U, V, E);
                    mySqDist[myNbExt] = Dist2;
                    myNbExt++;
                }
            }
        }
    }

    public bool IsDone() => myDone;

    public int NbExt()
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone();
        }
        return myNbExt;
    }

    public double SquareDistance(int N)
    {
        if ((N < 1) || (N > NbExt()))
        {
            throw new Standard_OutOfRange();
        }
        if (myIsAnalyticallyComputable)
        {
            return mySqDist[N - 1];
        }
        else
        {
            return myExtPS.SquareDistance(N);
        }
    }

    public Extrema_POnSurf Point(int N)
    {
        if ((N < 1) || (N > NbExt()))
        {
            throw new Standard_OutOfRange();
        }
        if (myIsAnalyticallyComputable)
        {
            return myPoint[N - 1];
        }
        else
        {
            return myExtPS.Point(N);
        }
    }
}
