// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPRevS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPRevS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// The extrema of the distance between a point and a surface of revolution: analytically when the basis curve is a
/// line or a conic whose plane contains the axis, with Extrema_GenExtPS otherwise.
/// </summary>
/// <remarks>The surface adaptor is kept, as OCCT keeps its handle.</remarks>
public class Extrema_ExtPRevS : Transient
{
    private GeomAdaptor_SurfaceOfRevolution? myS;
    private double myvinf;
    private double myvsup;
    private double mytolv;
    private Frame3D myPosition = new();
    private readonly Extrema_GenExtPS myExtPS = new();
    private bool myIsAnalyticallyComputable;
    private bool myDone;
    private int myNbExt;
    private readonly double[] mySqDist = new double[8];
    private readonly Extrema_POnSurf[] myPoint = [new(), new(), new(), new(), new(), new(), new(), new()];

    private static Frame3D GetPosition(GeomAdaptor_SurfaceOfRevolution S)
    {
        Adaptor3d_Curve C = S.BasisCurve();

        switch (C.GetType())
        {
            case Line:
                {
                    Line3D L = C.Line();
                    Direction3D N = S.AxeOfRevolution().Direction();
                    if (N.IsParallel(L.Direction(), global::Kerf.Engine.Foundation.Precision.Angular()))
                    {
                        Vector3D OO = new(L.Location(), S.AxeOfRevolution().Location());
                        if (OO.Magnitude() <= Constants.Resolution())
                        {
                            OO = new Vector3D(L.Location(), global::Kerf.Engine.Geometry.ElCLib.Value(100, L));
                            if (N.IsParallel(new Direction3D(OO), global::Kerf.Engine.Foundation.Precision.Angular()))
                            {
                                return new Frame3D(); // Line and axe of revolution coincide
                            }
                        }
                        N.Cross(new Direction3D(OO));
                    }
                    else
                    {
                        N.Cross(L.Direction());
                    }
                    return new Frame3D(L.Location(), N, L.Direction());
                }
            case Circle:
                return C.Circle().Position();
            case Ellipse:
                return C.Ellipse().Position();
            case Hyperbola:
                return C.Hyperbola().Position();
            case Parabola:
                return C.Parabola().Position();
            default:
                return new Frame3D();
        }
    }

    private static bool HasSingularity(GeomAdaptor_SurfaceOfRevolution S)
    {
        Adaptor3d_Curve C = S.BasisCurve();
        Direction3D N = S.AxeOfRevolution().Direction();
        Point3D P = S.AxeOfRevolution().Location();

        Line3D L = new(P, N);

        P = C.Value(C.FirstParameter());

        if (L.SquareDistance(P) < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
        {
            return true;
        }

        P = C.Value(C.LastParameter());

        return L.SquareDistance(P) < global::Kerf.Engine.Foundation.Precision.SquareConfusion();
    }

    private static void PerformExtPElC(Extrema_ExtPElC E, in Point3D P, Adaptor3d_Curve C, double Tol)
    {
        switch (C.GetType())
        {
            case Hyperbola:
                E.Perform(P, C.Hyperbola(), Tol, -global::Kerf.Engine.Foundation.Precision.Infinite(), global::Kerf.Engine.Foundation.Precision.Infinite());
                return;
            case Line:
                E.Perform(P, C.Line(), Tol, -global::Kerf.Engine.Foundation.Precision.Infinite(), global::Kerf.Engine.Foundation.Precision.Infinite());
                return;
            case Circle:
                E.Perform(P, C.Circle(), Tol, 0.0, 2.0 * M_PI);
                return;
            case Ellipse:
                E.Perform(P, C.Ellipse(), Tol, 0.0, 2.0 * M_PI);
                return;
            case Parabola:
                E.Perform(P, C.Parabola(), Tol, -global::Kerf.Engine.Foundation.Precision.Infinite(), global::Kerf.Engine.Foundation.Precision.Infinite());
                return;
            default:
                return;
        }
    }

    private static bool IsCaseAnalyticallyComputable(CurveKind theType, in Frame3D theCurvePos,
        in Axis3D AxeOfRevolution)
    {
        // check type
        switch (theType)
        {
            case Line:
            case Circle:
            case Ellipse:
            case Hyperbola:
            case Parabola:
                break;
            default:
                return false;
        }
        //  the axe of revolution must be in the plane of the curve.
        Plane3D pl = new(theCurvePos.Location(), theCurvePos.Direction());
        Point3D p1 = AxeOfRevolution.Location();
        double dist = 100.0, dist2 = dist * dist;
        double aThreshold = global::Kerf.Engine.Foundation.Precision.Angular() * global::Kerf.Engine.Foundation.Precision.Angular() * dist2;
        Point3D p2 = new(AxeOfRevolution.Location().XYZ() + dist * AxeOfRevolution.Direction().XYZ());

        return (pl.SquareDistance(p1) < aThreshold) && (pl.SquareDistance(p2) < aThreshold);
    }

    private static bool IsOriginalPnt(in Point3D P, Extrema_POnSurf[] Points, int NbPoints)
    {
        for (int i = 1; i <= NbPoints; i++)
        {
            if (Points[i - 1].Value().IsEqual(P, global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                return false;
            }
        }
        return true;
    }

    private static bool IsExtremum(double U, double V, in Point3D P, Adaptor3d_Surface S, out Point3D E,
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
        mytolv = global::Kerf.Engine.Foundation.Precision.Confusion();
        myDone = false;
        myNbExt = 0;
        myIsAnalyticallyComputable = false;

        for (int i = 0; i < 8; i++)
        {
            mySqDist[i] = RealLast();
        }
    }

    /// <summary>The extrema between P and S on [Umin, Usup] x [Vmin, Vsup].</summary>
    public Extrema_ExtPRevS(in Point3D theP, GeomAdaptor_SurfaceOfRevolution theS, double theUmin, double theUsup,
        double theVmin, double theVsup, double theTolU, double theTolV)
    {
        Initialize(theS, theUmin, theUsup, theVmin, theVsup, theTolU, theTolV);

        Perform(theP);
    }

    /// <summary>The extrema between P and S on S's whole range.</summary>
    public Extrema_ExtPRevS(in Point3D theP, GeomAdaptor_SurfaceOfRevolution theS, double theTolU, double theTolV)
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

    public void Perform(in Point3D P)
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

        Axis3D Ax = myS.AxeOfRevolution();
        Vector3D Dir = new(Ax.Direction()), Z = new(myPosition.Direction());
        Point3D O = Ax.Location();

        double OPdir = new Vector3D(O, P).Dot(Dir);
        Point3D Pp = P.Translated(Dir.Multiplied(-OPdir));
        if (O.IsEqual(Pp, global::Kerf.Engine.Foundation.Precision.Confusion()))
        { // P is on the AxeOfRevolution
            return;
        }

        double U, V;
        Point3D P1, Ppp;
        double OPpz = new Vector3D(O, Pp).Dot(Z);
        if (Math.Abs(OPpz) <= Constants.Resolution())
        {
            Ppp = Pp;
            U = 0;
        }
        else
        {
            Ppp = Pp.Translated(Z.Multiplied(-OPpz));
            if (O.IsEqual(Ppp, global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                U = M_PI / 2;
            }
            else
            {
                U = new Vector3D(O, Ppp).AngleWithRef(new Vector3D(O, Pp), Dir);
            }
        }

        Vector3D OPpp = new(O, Ppp), OPq = new(O, myS.Value(M_PI / 2, 0));
        if (U != M_PI / 2)
        {
            if (Math.Abs(OPq.Magnitude()) <= Constants.Resolution())
            {
                OPq = new Vector3D(O, myS.Value(M_PI / 2, anACurve.LastParameter() / 10));
            }
            if (OPpp.AngleWithRef(OPq, Dir) < 0)
            {
                U += M_PI;
            }
        }

        Transform3D T = new();
        T.SetRotation(Ax, -U);
        P1 = P.Transformed(T);

        Point3D E;
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

                    if ((anACurve.GetType() == Circle) || (anACurve.GetType() == Ellipse))
                    {
                        newV = global::Kerf.Engine.Geometry.ElCLib.InPeriod(V, myvinf, myvinf + 2.0 * M_PI);

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

                    if ((anACurve.GetType() == Circle) || (anACurve.GetType() == Ellipse))
                    {
                        newV = global::Kerf.Engine.Geometry.ElCLib.InPeriod(V, myvsup - 2.0 * M_PI, myvsup);

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

                    if ((anACurve.GetType() == Circle) || (anACurve.GetType() == Ellipse))
                    {
                        newV = global::Kerf.Engine.Geometry.ElCLib.InPeriod(V, myvinf, myvinf + 2.0 * M_PI);

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

                    if ((anACurve.GetType() == Circle) || (anACurve.GetType() == Ellipse))
                    {
                        newV = global::Kerf.Engine.Geometry.ElCLib.InPeriod(V, myvsup - 2.0 * M_PI, myvsup);

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
            throw new NotDoneException();
        }
        return myNbExt;
    }

    public double SquareDistance(int N)
    {
        if ((N < 1) || (N > NbExt()))
        {
            throw new OutOfRangeException();
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
            throw new OutOfRangeException();
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
