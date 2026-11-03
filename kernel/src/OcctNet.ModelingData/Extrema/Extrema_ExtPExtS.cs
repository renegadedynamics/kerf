// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPExtS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPExtS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// The extrema of the distance between a point and a surface of linear extrusion: analytically when the basis curve
/// is a conic or a line not in a plane parallel to the extrusion, with Extrema_GenExtPS otherwise.
/// </summary>
/// <remarks>The surface adaptor is kept, as OCCT keeps its handle.</remarks>
public class Extrema_ExtPExtS : Transient
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
    private Vector3D myDirection = new();
    private Frame3D myPosition = new();
    private readonly Extrema_GenExtPS myExtPS = new();
    private bool myIsAnalyticallyComputable;
    private bool myDone;
    private int myNbExt;
    private readonly double[] mySqDist = new double[4];
    private readonly Extrema_POnSurf[] myPoint = [new(), new(), new(), new()];

    // Returns the projection of a point Point on a plane ThePlane along a direction TheDir.
    private static Point3D ProjectPnt(in Frame3D ThePlane, in Direction3D TheDir, in Point3D Point)
    {
        Vector3D PO = new(Point, ThePlane.Location());
        double Alpha = PO * new Vector3D(ThePlane.Direction());
        Alpha /= TheDir * ThePlane.Direction();
        Point3D P = new();
        P.SetXYZ(Point.XYZ() + Alpha * TheDir.XYZ());
        return P;
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

    private void MakePreciser(ref double U, in Point3D P, bool isMin, in Frame3D OrtogSection)
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
            Direction3D aDirection = new(myDirection);
            double step = (myusup - myuinf) / 30, D2e, D2next, D2prev;
            Point3D Pe = ProjectPnt(OrtogSection, aDirection, GetValue(U, myC!)),
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
    public Extrema_ExtPExtS(in Point3D theP, GeomAdaptor_SurfaceOfLinearExtrusion theS, double theUmin, double theUsup,
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
    public Extrema_ExtPExtS(in Point3D theP, GeomAdaptor_SurfaceOfLinearExtrusion theS, double theTolU, double theTolV)
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
        myDirection = new Vector3D(theS.Direction());
        myIsAnalyticallyComputable = // false;
            IsCaseAnalyticallyComputable(myC.GetType(), myPosition, new Direction3D(myDirection));

        if (!myIsAnalyticallyComputable)
        {
            myExtPS.Initialize(theS, 32, 32, theUinf, theUsup, theVinf, theVsup, theTolU, theTolV);
        }
    }

    public void Perform(in Point3D P)
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
        Direction3D aDirection = new(myDirection);
        Point3D Pe, Pp = ProjectPnt(myPosition, aDirection, P);
        Extrema_ExtPElC anExt = new();
        PerformExtPElC(anExt, Pp, myC!, mytolu);
        if (!anExt.IsDone())
        {
            return;
        }

        Frame3D anOrtogSection = new(P, aDirection);
        double U, V;
        bool isMin, isSimpleCase = myDirection.IsParallel(new Vector3D(myPosition.Direction()), global::Kerf.Engine.Foundation.Precision.Angular());
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
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(myuinf, myuinf + 2.0 * M_PI, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref U, ref U2);
            }
            //////////////////////////////////////////////////
            Point3D E = POC.Value();
            Pe = ProjectPnt(anOrtogSection, aDirection, E);

            if (isSimpleCase)
            {
                V = new Vector3D(E, Pe) * myDirection;
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
                V = new Vector3D(E, Pe) * myDirection;
                UV[1] = U;
                UV[2] = V;
                FunctionSetRoot aFSR = new(myF, Tol);
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
                V = new Vector3D(E, Pe) * myDirection;
                UV[1] = U;
                UV[2] = V;

                aFSR.Perform(myF, UV, UVinf, UVsup);

                for (k = 1; k <= myF.NbExt(); k++)
                {
                    if (myF.SquareDistance(k) > global::Kerf.Engine.Foundation.Precision.Confusion() * global::Kerf.Engine.Foundation.Precision.Confusion())
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
                        myS!.D1(u, v, out Pe, out Vector3D du, out Vector3D dv);
                        double mdu = du.Magnitude();
                        double mdv = dv.Magnitude();
                        u = Math.Abs(Vals[1]);
                        v = Math.Abs(Vals[2]);
                        if (mdu > global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            if (u / dist / mdu > global::Kerf.Engine.Foundation.Precision.PConfusion())
                            {
                                continue;
                            }
                        }
                        if (mdv > global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            if (v / dist / mdv > global::Kerf.Engine.Foundation.Precision.PConfusion())
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
            throw new NotDoneException();
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

    private static Frame3D GetPosition(Adaptor3d_Curve C)
    {
        switch (C.GetType())
        {
            case Line:
                {
                    Line3D L = C.Line();
                    Plane3D Pln = new(L.Location(), L.Direction());
                    //: abv 30.05.02: OCC  - use constructor instead of Set...s() to avoid exception
                    Frame3D Pos = new(Pln.Location(), Pln.Position().Direction(), Pln.Position().XDirection());
                    return Pos;
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
        in Direction3D theSurfaceDirection)
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
        // check if it is a plane
        return Math.Abs(theCurvePos.Direction() * theSurfaceDirection) > Constants.Resolution();
    }

    private static Point3D GetValue(double U, Adaptor3d_Curve C)
    {
        switch (C.GetType())
        {
            case Line:
                return global::Kerf.Engine.Geometry.ElCLib.Value(U, C.Line());
            case Circle:
                return global::Kerf.Engine.Geometry.ElCLib.Value(U, C.Circle());
            case Ellipse:
                return global::Kerf.Engine.Geometry.ElCLib.Value(U, C.Ellipse());
            case Hyperbola:
                return global::Kerf.Engine.Geometry.ElCLib.Value(U, C.Hyperbola());
            case Parabola:
                return global::Kerf.Engine.Geometry.ElCLib.Value(U, C.Parabola());
            default:
                return new Point3D();
        }
    }
}
