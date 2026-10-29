// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPExtS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPExtS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;

namespace Occt;

/// <summary>
/// The extrema of the distance between a point and a surface of linear extrusion: analytically when the basis curve
/// is a conic or a line not in a plane parallel to the extrusion, with Extrema_GenExtPS otherwise.
/// </summary>
/// <remarks>The surface adaptor is kept, as OCCT keeps its handle.</remarks>
public class Extrema_ExtPExtS : Standard_Transient
{
    private double myuinf;
    private double myusup;
    private double mytolu;
    private double myvinf;
    private double myvsup;
    private double mytolv;
    private readonly Extrema_FuncPSNorm myF = new();
    private Adaptor3d_Curve? myC;
    private GeomAdaptor_SurfaceOfLinearExtrusion? myS;
    private gp_Vec myDirection = new();
    private gp_Ax2 myPosition = new();
    private readonly Extrema_GenExtPS myExtPS = new();
    private bool myIsAnalyticallyComputable;
    private bool myDone;
    private int myNbExt;
    private readonly double[] mySqDist = new double[4];
    private readonly Extrema_POnSurf[] myPoint = [new(), new(), new(), new()];

    // Returns the projection of a point Point on a plane ThePlane along a direction TheDir.
    private static gp_Pnt ProjectPnt(in gp_Ax2 ThePlane, in gp_Dir TheDir, in gp_Pnt Point)
    {
        gp_Vec PO = new(Point, ThePlane.Location());
        double Alpha = PO * new gp_Vec(ThePlane.Direction());
        Alpha /= TheDir * ThePlane.Direction();
        gp_Pnt P = new();
        P.SetXYZ(Point.XYZ() + Alpha * TheDir.XYZ());
        return P;
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

    private void MakePreciser(ref double U, in gp_Pnt P, bool isMin, in gp_Ax2 OrtogSection)
    {
        if (U > myusup)
        {
            U = myusup;
        }
        else if (U < myuinf)
        {
            U = myuinf;
        }
        else
        {
            gp_Dir aDirection = new(myDirection);
            double step = (myusup - myuinf) / 30, D2e, D2next, D2prev;
            gp_Pnt Pe = ProjectPnt(OrtogSection, aDirection, GetValue(U, myC!)),
                   Pprev = ProjectPnt(OrtogSection, aDirection, GetValue(U - step, myC!)),
                   Pnext = ProjectPnt(OrtogSection, aDirection, GetValue(U + step, myC!));
            D2e = P.SquareDistance(Pe);
            D2next = P.SquareDistance(Pnext);
            D2prev = P.SquareDistance(Pprev);
            bool notFound;
            if (isMin)
            {
                notFound = (D2e > D2prev || D2e > D2next);
            }
            else
            {
                notFound = (D2e < D2prev || D2e < D2next);
            }

            if (notFound && (D2e < D2next && isMin))
            {
                step = -step;
                D2next = D2prev;
                Pnext = Pprev;
            }
            while (notFound)
            {
                U = U + step;
                if (U > myusup)
                {
                    U = myusup;
                    break;
                }
                if (U < myuinf)
                {
                    U = myuinf;
                    break;
                }
                D2e = D2next;
                Pe = Pnext;
                Pnext = ProjectPnt(OrtogSection, aDirection, GetValue(U + step, myC!));
                D2next = P.SquareDistance(Pnext);
                if (isMin)
                {
                    notFound = D2e > D2next;
                }
                else
                {
                    notFound = D2e < D2next;
                }
            }
        }
    }

    public Extrema_ExtPExtS()
    {
        myuinf = 0.0;
        myusup = 0.0;
        mytolu = 0.0;
        myvinf = 0.0;
        myvsup = 0.0;
        mytolv = 0.0;
        myIsAnalyticallyComputable = false;
        myDone = false;
        myNbExt = 0;
        for (int anIdx = 0; anIdx < mySqDist.Length; anIdx++)
        {
            mySqDist[anIdx] = RealLast();
        }
    }

    /// <summary>The extrema between P and S on [Umin, Usup] x [Vmin, Vsup].</summary>
    public Extrema_ExtPExtS(in gp_Pnt theP, GeomAdaptor_SurfaceOfLinearExtrusion theS, double theUmin, double theUsup,
        double theVmin, double theVsup, double theTolU, double theTolV)
    {
        myuinf = theUmin;
        myusup = theUsup;
        mytolu = theTolU;
        myvinf = theVmin;
        myvsup = theVsup;
        mytolv = theTolV;
        myS = theS;
        myIsAnalyticallyComputable = false;
        myDone = false;
        myNbExt = 0;
        for (int anIdx = 0; anIdx < mySqDist.Length; anIdx++)
        {
            mySqDist[anIdx] = RealLast();
        }
        Initialize(theS, theUmin, theUsup, theVmin, theVsup, theTolU, theTolV);

        Perform(theP);
    }

    /// <summary>The extrema between P and S on S's whole range.</summary>
    public Extrema_ExtPExtS(in gp_Pnt theP, GeomAdaptor_SurfaceOfLinearExtrusion theS, double theTolU, double theTolV)
    {
        myuinf = theS.FirstUParameter();
        myusup = theS.LastUParameter();
        mytolu = theTolU;
        myvinf = theS.FirstVParameter();
        myvsup = theS.LastVParameter();
        mytolv = theTolV;
        myS = theS;
        myIsAnalyticallyComputable = false;
        myDone = false;
        myNbExt = 0;
        for (int anIdx = 0; anIdx < mySqDist.Length; anIdx++)
        {
            mySqDist[anIdx] = RealLast();
        }
        Initialize(theS, theS.FirstUParameter(), theS.LastUParameter(), theS.FirstVParameter(), theS.LastVParameter(),
            theTolU, theTolV);

        Perform(theP);
    }

    public void Initialize(GeomAdaptor_SurfaceOfLinearExtrusion theS, double theUinf, double theUsup, double theVinf,
        double theVsup, double theTolU, double theTolV)
    {
        myuinf = theUinf;
        myusup = theUsup;
        mytolu = theTolU;

        myvinf = theVinf;
        myvsup = theVsup;
        mytolv = theTolV;

        myIsAnalyticallyComputable = false;
        myDone = false;
        myNbExt = 0;

        Adaptor3d_Curve anACurve = theS.BasisCurve();

        myF.Initialize(theS);
        myC = anACurve;
        myS = theS;
        myPosition = GetPosition(myC);
        myDirection = new gp_Vec(theS.Direction());
        myIsAnalyticallyComputable = // false;
            IsCaseAnalyticallyComputable(myC.GetType(), myPosition, new gp_Dir(myDirection));

        if (!myIsAnalyticallyComputable)
        {
            myExtPS.Initialize(theS, 32, 32, theUinf, theUsup, theVinf, theVsup, theTolU, theTolV);
        }
    }

    public void Perform(in gp_Pnt P)
    {
        const int NbExtMax = 4; // dimension of arrays
                                // myPoint[] and mySqDist[]
                                // For "analytical" case
        myDone = false;
        myNbExt = 0;

        if (!myIsAnalyticallyComputable)
        {
            myExtPS.Perform(P);
            myDone = myExtPS.IsDone();
            //  modified by NIZHNY-EAP Wed Nov 17 12:59:08 1999 ___BEGIN___
            myNbExt = myExtPS.NbExt();
            //  modified by NIZHNY-EAP Wed Nov 17 12:59:09 1999 ___END___
            return;
        }

        // myDirection is a gp_Vec, which C++ converts implicitly (normalising it again) where a gp_Dir is expected
        gp_Dir aDirection = new(myDirection);
        gp_Pnt Pe, Pp = ProjectPnt(myPosition, aDirection, P);
        Extrema_ExtPElC anExt = new();
        PerformExtPElC(anExt, Pp, myC!, mytolu);
        if (!anExt.IsDone())
        {
            return;
        }

        gp_Ax2 anOrtogSection = new(P, aDirection);
        double U, V;
        bool isMin, isSimpleCase = myDirection.IsParallel(new gp_Vec(myPosition.Direction()), Precision.Angular());
        int i, aNbExt = anExt.NbExt();
        math_Vector UV = new(1, 2), Tol = new(1, 2), UVinf = new(1, 2), UVsup = new(1, 2);
        Tol[1] = mytolu;
        Tol[2] = mytolv;
        UVinf[1] = myuinf;
        UVinf[2] = myvinf;
        UVsup[1] = myusup;
        UVsup[2] = myvsup;

        for (i = 1; i <= aNbExt; i++)
        {
            Extrema_POnCurv POC = anExt.Point(i);
            U = POC.Parameter();
            //// modified by jgv, 23.12.2008 for OCC17194 ////
            if (myC!.IsPeriodic())
            {
                double U2 = U;
                ElCLib.AdjustPeriodic(myuinf, myuinf + 2.0 * M_PI, Precision.PConfusion(), ref U, ref U2);
            }
            //////////////////////////////////////////////////
            gp_Pnt E = POC.Value();
            Pe = ProjectPnt(anOrtogSection, aDirection, E);

            if (isSimpleCase)
            {
                V = new gp_Vec(E, Pe) * myDirection;
                // modified by NIZHNY-MKK  Thu Sep 18 14:46:14 2003.BEGIN
                myPoint[myNbExt] = new Extrema_POnSurf(U, V, Pe);
                mySqDist[myNbExt] = anExt.SquareDistance(i);
                myNbExt++;
                if (myNbExt == NbExtMax)
                {
                    break;
                }
                // modified by NIZHNY-MKK  Thu Sep 18 14:46:18 2003.END
            }
            else
            {
                myF.SetPoint(P);
                isMin = anExt.IsMin(i); //( Pp.Distance(GetValue(U+10,myC)) > anExt.Value(i) );

                MakePreciser(ref U, P, isMin, anOrtogSection);
                E = GetValue(U, myC);
                Pe = ProjectPnt(anOrtogSection, aDirection, E);
                V = new gp_Vec(E, Pe) * myDirection;
                UV[1] = U;
                UV[2] = V;
                math_FunctionSetRoot aFSR = new(myF, Tol);
                aFSR.Perform(myF, UV, UVinf, UVsup);
                int k;
                for (k = 1; k <= myF.NbExt(); k++)
                {
                    if (IsOriginalPnt(myF.Point(k).Value(), myPoint, myNbExt))
                    {
                        // modified by NIZHNY-MKK  Thu Sep 18 14:46:41 2003.BEGIN
                        myPoint[myNbExt] = myF.Point(k);
                        mySqDist[myNbExt] = myF.SquareDistance(k);
                        myNbExt++;
                        if (myNbExt == NbExtMax)
                        {
                            break;
                        }
                        // modified by NIZHNY-MKK  Thu Sep 18 14:46:43 2003.END
                    }
                }
                if (myNbExt == NbExtMax)
                {
                    break;
                }
                // try symmetric point
                myF.SetPoint(P); // To clear previous solutions
                U *= -1;
                MakePreciser(ref U, P, isMin, anOrtogSection);
                E = GetValue(U, myC);
                Pe = ProjectPnt(anOrtogSection, aDirection, E);
                V = new gp_Vec(E, Pe) * myDirection;
                UV[1] = U;
                UV[2] = V;

                aFSR.Perform(myF, UV, UVinf, UVsup);

                for (k = 1; k <= myF.NbExt(); k++)
                {
                    if (myF.SquareDistance(k) > Precision.Confusion() * Precision.Confusion())
                    {
                        // Additional checking solution: FSR sometimes is wrong
                        // when starting point is far from solution.
                        double dist = Math.Sqrt(myF.SquareDistance(k));
                        math_Vector Vals = new(1, 2);
                        Extrema_POnSurf PonS = myF.Point(k);
                        PonS.Parameter(out double u, out double v);
                        UV[1] = u;
                        UV[2] = v;
                        myF.Value(UV, Vals);
                        myS!.D1(u, v, out Pe, out gp_Vec du, out gp_Vec dv);
                        double mdu = du.Magnitude();
                        double mdv = dv.Magnitude();
                        u = Math.Abs(Vals[1]);
                        v = Math.Abs(Vals[2]);
                        if (mdu > Precision.PConfusion())
                        {
                            if (u / dist / mdu > Precision.PConfusion())
                            {
                                continue;
                            }
                        }
                        if (mdv > Precision.PConfusion())
                        {
                            if (v / dist / mdv > Precision.PConfusion())
                            {
                                continue;
                            }
                        }
                    }
                    if (IsOriginalPnt(myF.Point(k).Value(), myPoint, myNbExt))
                    {
                        // modified by NIZHNY-MKK  Thu Sep 18 14:46:59 2003.BEGIN
                        myPoint[myNbExt] = myF.Point(k);
                        mySqDist[myNbExt] = myF.SquareDistance(k);
                        myNbExt++;
                        if (myNbExt == NbExtMax)
                        {
                            break;
                        }
                        // modified by NIZHNY-MKK  Thu Sep 18 14:47:04 2003.END
                    }
                }
                if (myNbExt == NbExtMax)
                {
                    break;
                }
            }
        }
        myDone = true;
    }

    public bool IsDone() => myDone;

    public int NbExt()
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone();
        }
        if (myIsAnalyticallyComputable)
        {
            return myNbExt;
        }
        else
        {
            return myExtPS.NbExt();
        }
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

    private static gp_Ax2 GetPosition(Adaptor3d_Curve C)
    {
        switch (C.GetType())
        {
            case GeomAbs_Line:
                {
                    gp_Lin L = C.Line();
                    gp_Pln Pln = new(L.Location(), L.Direction());
                    //: abv 30.05.02: OCC  - use constructor instead of Set...s() to avoid exception
                    gp_Ax2 Pos = new(Pln.Location(), Pln.Position().Direction(), Pln.Position().XDirection());
                    return Pos;
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
        in gp_Dir theSurfaceDirection)
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
        // check if it is a plane
        return Math.Abs(theCurvePos.Direction() * theSurfaceDirection) > gp.Resolution();
    }

    private static gp_Pnt GetValue(double U, Adaptor3d_Curve C)
    {
        switch (C.GetType())
        {
            case GeomAbs_Line:
                return ElCLib.Value(U, C.Line());
            case GeomAbs_Circle:
                return ElCLib.Value(U, C.Circle());
            case GeomAbs_Ellipse:
                return ElCLib.Value(U, C.Ellipse());
            case GeomAbs_Hyperbola:
                return ElCLib.Value(U, C.Hyperbola());
            case GeomAbs_Parabola:
                return ElCLib.Value(U, C.Parabola());
            default:
                return new gp_Pnt();
        }
    }
}
