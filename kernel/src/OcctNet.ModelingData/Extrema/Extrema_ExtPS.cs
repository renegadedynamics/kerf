// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_ExtPS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Extrema_ExtAlgo;
using static Occt.Extrema_ExtFlag;
using static Occt.GeomAbs_IsoType;
using static Occt.GeomAbs_SurfaceType;

namespace Occt;

/// <summary>
/// The extrema of the distance between a point and a surface: analytically on planes, cylinders, cones, spheres and
/// tori (Extrema_ExtPElS), with Extrema_ExtPExtS and Extrema_ExtPRevS on swept surfaces, and by sampling
/// (Extrema_GenExtPS) otherwise. Solutions outside the parameter range (put in the period first) are dropped.
/// </summary>
/// <remarks>The surface adaptor is kept, as OCCT keeps a pointer: it must outlive the algorithm.</remarks>
public class Extrema_ExtPS
{
    private Adaptor3d_Surface? myS;
    private bool myDone;
    private readonly Extrema_ExtPElS myExtPElS = new();
    private readonly Extrema_GenExtPS myExtPS = new();
    private readonly NCollection_Sequence<Extrema_POnSurf> myPoints = new();
    private double myuinf;
    private double myusup;
    private double myvinf;
    private double myvsup;
    private double mytolu;
    private double mytolv;
    private double d11;
    private double d12;
    private double d21;
    private double d22;
    private gp_Pnt P11 = new();
    private gp_Pnt P12 = new();
    private gp_Pnt P21 = new();
    private gp_Pnt P22 = new();
    private readonly NCollection_Sequence<double> mySqDist = new();
    private GeomAbs_SurfaceType mytype;
    private Extrema_ExtPExtS? myExtPExtS;
    private Extrema_ExtPRevS? myExtPRevS;

    private static bool IsoIsDeg(Adaptor3d_Surface S, double Param, GeomAbs_IsoType IT, double TolMin, double TolMax)
    {
        double U1 = 0.0, U2 = 0.0, V1 = 0.0, V2 = 0.0, T;
        bool Along = true;
        U1 = S.FirstUParameter();
        U2 = S.LastUParameter();
        V1 = S.FirstVParameter();
        V2 = S.LastVParameter();
        double Step, D1NormMax;
        if (IT == GeomAbs_IsoV)
        {
            if (!Precision.IsInfinite(U1) && !Precision.IsInfinite(U2))
            {
                Step = (U2 - U1) / 10;
                if (Step < Precision.PConfusion())
                {
                    return false;
                }
                D1NormMax = 0.0;

                for (T = U1; T <= U2; T = T + Step)
                {
                    S.D1(T, Param, out _, out gp_Vec D1U, out _);
                    D1NormMax = Max(D1NormMax, D1U.Magnitude());
                }

                if (D1NormMax > TolMax || D1NormMax < TolMin)
                {
                    Along = false;
                }
            }
        }
        else
        {
            if (!Precision.IsInfinite(V1) && !Precision.IsInfinite(V2))
            {
                Step = (V2 - V1) / 10;
                if (Step < Precision.PConfusion())
                {
                    return false;
                }
                D1NormMax = 0.0;
                for (T = V1; T <= V2; T = T + Step)
                {
                    S.D1(Param, T, out _, out _, out gp_Vec D1V);
                    D1NormMax = Max(D1NormMax, D1V.Magnitude());
                }

                if (D1NormMax > TolMax || D1NormMax < TolMin)
                {
                    Along = false;
                }
            }
        }
        return Along;
    }

    private void TreatSolution(in Extrema_POnSurf PS, double Val)
    {
        PS.Parameter(out double U, out double V);
        if (myS!.IsUPeriodic())
        {
            U = ElCLib.InPeriod(U, myuinf, myuinf + myS.UPeriod());

            // Handle trimmed surfaces.
            if (U > myusup + mytolu)
            {
                U -= myS.UPeriod();
            }
            if (U < myuinf - mytolu)
            {
                U += myS.UPeriod();
            }
        }
        if (myS.IsVPeriodic())
        {
            V = ElCLib.InPeriod(V, myvinf, myvinf + myS.VPeriod());

            // Handle trimmed surfaces.
            if (V > myvsup + mytolv)
            {
                V -= myS.VPeriod();
            }
            if (V < myvinf - mytolv)
            {
                V += myS.VPeriod();
            }
        }
        if ((myuinf - U) <= mytolu && (U - myusup) <= mytolu && (myvinf - V) <= mytolv && (V - myvsup) <= mytolv)
        {
            myPoints.Append(new Extrema_POnSurf(U, V, PS.Value()));
            mySqDist.Append(Val);
        }
    }

    public Extrema_ExtPS()
    {
        myS = null;
        myDone = false;
        myuinf = 0.0;
        myusup = 0.0;
        myvinf = 0.0;
        myvsup = 0.0;
        mytolu = 0.0;
        mytolv = 0.0;
        d11 = 0.0;
        d12 = 0.0;
        d21 = 0.0;
        d22 = 0.0;
        mytype = GeomAbs_OtherSurface;
    }

    /// <summary>
    /// The extrema between theP and theS on theS's whole range. theTolU and theTolV are the parametric tolerances of
    /// the solutions.
    /// </summary>
    public Extrema_ExtPS(in gp_Pnt theP, Adaptor3d_Surface theS, double theTolU, double theTolV,
        Extrema_ExtFlag theF = Extrema_ExtFlag_MINMAX, Extrema_ExtAlgo theA = Extrema_ExtAlgo_Grad)
    {
        myExtPS.SetFlag(theF);
        myExtPS.SetAlgo(theA);

        Initialize(theS, theS.FirstUParameter(), theS.LastUParameter(), theS.FirstVParameter(), theS.LastVParameter(),
            theTolU, theTolV);

        Perform(theP);
    }

    /// <summary>The extrema between theP and theS on [theUinf, theUsup] x [theVinf, theVsup].</summary>
    public Extrema_ExtPS(in gp_Pnt theP, Adaptor3d_Surface theS, double theUinf, double theUsup, double theVinf,
        double theVsup, double theTolU, double theTolV, Extrema_ExtFlag theF = Extrema_ExtFlag_MINMAX,
        Extrema_ExtAlgo theA = Extrema_ExtAlgo_Grad)
    {
        myExtPS.SetFlag(theF);
        myExtPS.SetAlgo(theA);

        Initialize(theS, theUinf, theUsup, theVinf, theVsup, theTolU, theTolV);

        Perform(theP);
    }

    /// <summary>Sets the surface and its range (infinite bounds become ±1e10) and the tolerances.</summary>
    public void Initialize(Adaptor3d_Surface theS, double theUinf, double theUsup, double theVinf, double theVsup,
        double theTolU, double theTolV)
    {
        myS = theS;
        myuinf = theUinf;
        myusup = theUsup;
        myvinf = theVinf;
        myvsup = theVsup;

        if (Precision.IsNegativeInfinite(myuinf))
        {
            myuinf = -1e10;
        }
        if (Precision.IsPositiveInfinite(myusup))
        {
            myusup = 1e10;
        }
        if (Precision.IsNegativeInfinite(myvinf))
        {
            myvinf = -1e10;
        }
        if (Precision.IsPositiveInfinite(myvsup))
        {
            myvsup = 1e10;
        }

        mytolu = theTolU;
        mytolv = theTolV;
        mytype = myS.GetType();

        bool isB = (myS.GetType() == GeomAbs_BSplineSurface || myS.GetType() == GeomAbs_BezierSurface);

        int nbU = (isB) ? 44 : 32;
        int nbV = (isB) ? 44 : 32;

        bool bUIsoIsDeg = false, bVIsoIsDeg = false;

        if (myS.GetType() != GeomAbs_Plane)
        {
            bUIsoIsDeg = IsoIsDeg(theS, myuinf, GeomAbs_IsoU, 0.0, 1.0e-9)
                         || IsoIsDeg(theS, myusup, GeomAbs_IsoU, 0.0, 1.0e-9);
            bVIsoIsDeg = IsoIsDeg(theS, myvinf, GeomAbs_IsoV, 0.0, 1.0e-9)
                         || IsoIsDeg(theS, myvsup, GeomAbs_IsoV, 0.0, 1.0e-9);
        }

        if (bUIsoIsDeg)
        {
            nbU = 300;
        }
        if (bVIsoIsDeg)
        {
            nbV = 300;
        }

        myExtPS.Initialize(myS, nbU, nbV, myuinf, myusup, myvinf, myvsup, mytolu, mytolv);

        myExtPExtS = null;
        myExtPRevS = null;
    }

    /// <summary>Computes the extrema for thePoint. Throws if the surface has not been initialized.</summary>
    public void Perform(in gp_Pnt thePoint)
    {
        myPoints.Clear();
        mySqDist.Clear();

        switch (mytype)
        {
            case GeomAbs_Cylinder:
                myExtPElS.Perform(thePoint, myS!.Cylinder(), Precision.Confusion());
                break;
            case GeomAbs_Plane:
                myExtPElS.Perform(thePoint, myS!.Plane(), Precision.Confusion());
                break;
            case GeomAbs_Cone:
                myExtPElS.Perform(thePoint, myS!.Cone(), Precision.Confusion());
                break;
            case GeomAbs_Sphere:
                myExtPElS.Perform(thePoint, myS!.Sphere(), Precision.Confusion());
                break;
            case GeomAbs_Torus:
                myExtPElS.Perform(thePoint, myS!.Torus(), Precision.Confusion());
                break;

            case GeomAbs_SurfaceOfExtrusion:
                {
                    if (myExtPExtS is null)
                    {
                        GeomAdaptor_SurfaceOfLinearExtrusion aS =
                            new(myS!.BasisCurve(), myS.Direction());

                        myExtPExtS = new Extrema_ExtPExtS(thePoint, aS, myuinf, myusup, myvinf, myvsup, mytolu, mytolv);
                    }
                    else
                    {
                        myExtPExtS.Perform(thePoint);
                    }

                    myDone = myExtPExtS.IsDone();
                    if (myDone)
                    {
                        for (int anIdx = 1; anIdx <= myExtPExtS.NbExt(); ++anIdx)
                        {
                            TreatSolution(myExtPExtS.Point(anIdx), myExtPExtS.SquareDistance(anIdx));
                        }
                    }

                    return;
                }

            case GeomAbs_SurfaceOfRevolution:
                {
                    if (myExtPRevS is null)
                    {
                        GeomAdaptor_SurfaceOfRevolution aS = new(myS!.BasisCurve(), myS.AxeOfRevolution());

                        myExtPRevS = new Extrema_ExtPRevS(thePoint, aS, myuinf, myusup, myvinf, myvsup, mytolu, mytolv);
                    }
                    else
                    {
                        myExtPRevS.Perform(thePoint);
                    }

                    myDone = myExtPRevS.IsDone();
                    if (myDone)
                    {
                        for (int anIdx = 1; anIdx <= myExtPRevS.NbExt(); ++anIdx)
                        {
                            TreatSolution(myExtPRevS.Point(anIdx), myExtPRevS.SquareDistance(anIdx));
                        }
                    }

                    return;
                }

            default:
                {
                    myExtPS.Perform(thePoint);
                    myDone = myExtPS.IsDone();
                    if (myDone)
                    {
                        for (int anIdx = 1; anIdx <= myExtPS.NbExt(); ++anIdx)
                        {
                            TreatSolution(myExtPS.Point(anIdx), myExtPS.SquareDistance(anIdx));
                        }
                    }
                    return;
                }
        }

        myDone = myExtPElS.IsDone();
        if (myDone)
        {
            for (int anIdx = 1; anIdx <= myExtPElS.NbExt(); ++anIdx)
            {
                TreatSolution(myExtPElS.Point(anIdx), myExtPElS.SquareDistance(anIdx));
            }
        }
    }

    /// <summary>True if the distances are found.</summary>
    public bool IsDone() => myDone;

    /// <summary>The Nth square distance.</summary>
    public double SquareDistance(int N)
    {
        if ((N < 1) || (N > NbExt()))
        {
            throw new Standard_OutOfRange();
        }
        return mySqDist.Value(N);
    }

    /// <summary>The number of extremum distances.</summary>
    public int NbExt()
    {
        if (!IsDone())
        {
            throw new StdFail_NotDone();
        }
        return mySqDist.Length();
    }

    /// <summary>The point of the Nth extremum distance.</summary>
    public Extrema_POnSurf Point(int N)
    {
        if ((N < 1) || (N > NbExt()))
        {
            throw new Standard_OutOfRange();
        }
        return myPoints.Value(N);
    }

    /// <summary>
    /// The square distances to the corners of the range and the corner points. OCCT never computes them: they are 0
    /// and the origin, or (after the parameterised constructors) whatever was in memory.
    /// </summary>
    public void TrimmedSquareDistances(out double dUfVf, out double dUfVl, out double dUlVf, out double dUlVl,
        out gp_Pnt PUfVf, out gp_Pnt PUfVl, out gp_Pnt PUlVf, out gp_Pnt PUlVl)
    {
        dUfVf = d11; // OCCT: d11, ..., P22 are never computed
        dUfVl = d12;
        dUlVf = d21;
        dUlVl = d22;
        PUfVf = P11;
        PUfVl = P12;
        PUlVf = P21;
        PUlVl = P22;
    }

    public void SetFlag(Extrema_ExtFlag F) => myExtPS.SetFlag(F);

    public void SetAlgo(Extrema_ExtAlgo A) => myExtPS.SetAlgo(A);
}
