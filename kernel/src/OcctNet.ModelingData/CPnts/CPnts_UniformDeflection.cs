// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_UniformDeflection.hxx
// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_UniformDeflection.lxx
// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_UniformDeflection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

//-------------------------------------------------------------------------
// cases processed : parameterized curve
//               the curve should be C2
// provide a max arrow
// algorithm of parameterized curve:
//   calculation of the step of advancement is
//             du = sqrt(8*fleche*||P'(u)||/||P'(u)^P''(u)||
//   calculate each point such as u+Du
//   check if the arrow is actually taken into account, if yes, continue
//   otherwise correct the step
//   si du cannot be calculated (null curvature, singularity on the curve)
//   take a constant step to reach the last point or to go past it
//   The last point is readjusted by the following criteria:
//     if the last calculated parameter is <2*resolution, reframe the last point found
//     between itself and the previous point and add the end point
//     (avoid a concentration at the end)
//     otherwise if the distance (last calculated point, end point)<arrow,
//     replace the last calculated point by the end point
//     otherwise calculate max arrow between the last but one calculated point
//     and the end point; if this arrow is greater than the arrow
//     replace the last point by this one and the end point
//    CONTROLS OF ARROW AND THE LAST POINT ARE DONE ONLY IF withControl=true
//   each iteration calculates at maximum 3 points
//-------------------------------------------------------------------------

namespace Occt;

/// <summary>
/// Computes a distribution of points on a curve such that the deflection (sag) between consecutive points stays
/// below a given value. The points are computed a few at a time, read with <c>More</c>, <c>Next</c>,
/// <c>Value</c> and <c>Point</c>. 2D curves give points with Z = 0.
/// </summary>
/// <remarks>A working object (decision 21): it keeps a reference to the curve while points are read.</remarks>
public sealed class CPnts_UniformDeflection
{
    private bool myDone;
    private bool my3d;
    private object? myCurve;
    private bool myFinish;
    private double myTolCur;
    private bool myControl;
    private int myIPoint;
    private int myNbPoints;
    private readonly double[] myParams = new double[3];
    private readonly gp_Pnt[] myPoints = new gp_Pnt[3];
    private double myDwmax;
    private double myDeflection;
    private double myFirstParam;
    private double myLastParam;
    private double myDu;

    private static void D03d(object C, double U, out gp_Pnt P)
    {
        ((Adaptor3d_Curve)C).D0(U, out P);
    }

    private static void D02d(object C, double U, out gp_Pnt PP)
    {
        ((Adaptor2d_Curve2d)C).D0(U, out gp_Pnt2d P);
        PP = new gp_Pnt();
        PP.SetCoord(P.X(), P.Y(), 0.0);
    }

    private static void D23d(object C, double U, out gp_Pnt P, out gp_Vec V1, out gp_Vec V2)
    {
        ((Adaptor3d_Curve)C).D2(U, out P, out V1, out V2);
    }

    private static void D22d(object C, double U, out gp_Pnt PP, out gp_Vec VV1, out gp_Vec VV2)
    {
        ((Adaptor2d_Curve2d)C).D2(U, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2);
        PP = new gp_Pnt();
        VV1 = new gp_Vec();
        VV2 = new gp_Vec();
        PP.SetCoord(P.X(), P.Y(), 0.0);
        VV1.SetCoord(V1.X(), V1.Y(), 0.0);
        VV2.SetCoord(V2.X(), V2.Y(), 0.0);
    }

    private void Perform()
    {
        gp_Pnt P, P1, P2;
        gp_Vec V1, V2, VV;
        double Un1;
        double NormD1, NormD2;

        myIPoint = -1;
        myNbPoints = -1;

        double anEspilon = Epsilon(myFirstParam);

        while ((myNbPoints < 2) && (!myFinish))
        {
            ++myNbPoints;
            myParams[myNbPoints] = myFirstParam;

            if (my3d)
            {
                D23d(myCurve!, myFirstParam, out myPoints[myNbPoints], out V1, out V2);
            }
            else
            {
                D22d(myCurve!, myFirstParam, out myPoints[myNbPoints], out V1, out V2);
            }
            P = myPoints[myNbPoints];
            NormD1 = V1.Magnitude();
            if (NormD1 < myTolCur || V2.Magnitude() < myTolCur)
            {
                // singularity on the tangent or null curvature
                myDu = Min(myDwmax, 1.5 * myDu);
            }
            else
            {
                NormD2 = V2.CrossMagnitude(V1);
                if (NormD2 / NormD1 < myDeflection)
                {
                    // collinearity of derivatives
                    myDu = Min(myDwmax, 1.5 * myDu);
                }
                else
                {
                    myDu = Math.Sqrt(8.0 * myDeflection * NormD1 / NormD2);
                    myDu = Min(Max(myDu, myTolCur), myDwmax);
                }
            }

            // check if the arrow is observed if WithControl
            if (myControl)
            {
                myDu = Min(myDu, myLastParam - myFirstParam);
                if (my3d)
                {
                    D03d(myCurve!, myFirstParam + myDu, out P);
                    D03d(myCurve!, myFirstParam + (myDu / 2.0), out P1);
                }
                else
                {
                    D02d(myCurve!, myFirstParam + myDu, out P);
                    D02d(myCurve!, myFirstParam + (myDu / 2.0), out P1);
                }
                V1 = new gp_Vec(myPoints[myNbPoints], P);
                NormD1 = V1.Magnitude();
                if (NormD1 >= myDeflection)
                {
                    V2 = new gp_Vec(myPoints[myNbPoints], P1);
                    NormD2 = V2.CrossMagnitude(V1) / NormD1;

                    // passing of arrow starting from which the redivision is done is arbitrary
                    // probably it will be necessary to readjust it (differentiate the first point
                    // from the others) this test does not work on the points of inflexion
                    if (NormD2 > myDeflection / 5.0)
                    {
                        NormD2 = Max(NormD2, 1.1 * myDeflection);
                        myDu = myDu * Math.Sqrt(myDeflection / NormD2);
                        myDu = Min(Max(myDu, myTolCur), myDwmax);
                    }
                }
            }
            myFirstParam += myDu;
            myFinish = myLastParam - myFirstParam < myTolCur || Math.Abs(myDu) < myTolCur ||
                       // to avoid less than double precision endless increment
                       myDu < anEspilon;
        }
        if (myFinish)
        {
            // the last point is corrected if control
            if (myControl && (myNbPoints == 1))
            {
                Un1 = myParams[0];
                if (myLastParam - Un1 < 0.33 * (myLastParam - myFirstParam))
                {
                    myFirstParam = (myLastParam + Un1) / 2.0;
                    myParams[0] = myFirstParam;
                    myParams[1] = myLastParam;
                    if (my3d)
                    {
                        D03d(myCurve!, myParams[0], out myPoints[0]);
                        D03d(myCurve!, myParams[1], out myPoints[1]);
                    }
                    else
                    {
                        D02d(myCurve!, myParams[0], out myPoints[0]);
                        D02d(myCurve!, myParams[1], out myPoints[1]);
                    }
                }
                else
                {
                    if (my3d)
                    {
                        D23d(myCurve!, myLastParam, out P1, out V1, out V2);
                    }
                    else
                    {
                        D22d(myCurve!, myLastParam, out P1, out V1, out V2);
                    }
                    P = myPoints[0];
                    VV = new gp_Vec(P1, P);
                    NormD1 = VV.Magnitude();
                    if (NormD1 < myDeflection)
                    {
                        myParams[1] = myLastParam;
                        myPoints[1] = P1;
                    }
                    else
                    {
                        myFirstParam = (myLastParam * (myParams[1] - Un1) + Un1 * myDu) / (myFirstParam - Un1);
                        if (my3d)
                        {
                            D03d(myCurve!, myFirstParam, out P2);
                        }
                        else
                        {
                            D02d(myCurve!, myFirstParam, out P2);
                        }
                        if ((VV.CrossMagnitude(new gp_Vec(P2, P)) / NormD1 < myDeflection)
                            && (Un1 >= myLastParam - myDwmax))
                        {
                            // point n is removed
                            myParams[1] = myLastParam;
                            myPoints[1] = P1;
                        }
                        else
                        {
                            myParams[1] = myFirstParam;
                            myPoints[1] = P2;
                            myParams[2] = myLastParam;
                            myPoints[2] = P1;
                            ++myNbPoints;
                        }
                    }
                }
            }
            else
            {
                ++myNbPoints;
                if (myNbPoints >= 3)
                {
                    myNbPoints = 2;
                }
                myParams[myNbPoints] = myLastParam;
                if (my3d)
                {
                    D03d(myCurve!, myLastParam, out myPoints[myNbPoints]);
                }
                else
                {
                    D02d(myCurve!, myLastParam, out myPoints[myNbPoints]);
                }
            }
        }
    }

    public CPnts_UniformDeflection()
    {
        myDone = false;
        my3d = false;
        myFinish = false;
        myTolCur = 0.0;
        myControl = false;
        myIPoint = 0;
        myNbPoints = 0;
        myDwmax = 0.0;
        myDeflection = 0.0;
        myFirstParam = 0.0;
        myLastParam = 0.0;
        myDu = 0.0;
        FillPoints();
    }

    /// <summary>Computes the points on the whole curve; Resolution is the tolerance on parameters.</summary>
    public CPnts_UniformDeflection(Adaptor3d_Curve C, double Deflection, double Resolution, bool WithControl)
    {
        FillPoints();
        Initialize(C, Deflection, Resolution, WithControl);
    }

    /// <summary>Computes the points on the whole curve; Resolution is the tolerance on parameters.</summary>
    public CPnts_UniformDeflection(Adaptor2d_Curve2d C, double Deflection, double Resolution, bool WithControl)
    {
        FillPoints();
        Initialize(C, Deflection, Resolution, WithControl);
    }

    /// <summary>Computes the points between the parameters U1 and U2.</summary>
    public CPnts_UniformDeflection(Adaptor3d_Curve C, double Deflection, double U1, double U2, double Resolution,
        bool WithControl)
    {
        FillPoints();
        Initialize(C, Deflection, U1, U2, Resolution, WithControl);
    }

    /// <summary>Computes the points between the parameters U1 and U2.</summary>
    public CPnts_UniformDeflection(Adaptor2d_Curve2d C, double Deflection, double U1, double U2, double Resolution,
        bool WithControl)
    {
        FillPoints();
        Initialize(C, Deflection, U1, U2, Resolution, WithControl);
    }

    // OCCT's gp_Pnt array elements are default-constructed (the origin, as .NET's zeros); kept explicit (rules, 3).
    private void FillPoints()
    {
        for (int i = 0; i < myPoints.Length; i++)
        {
            myPoints[i] = new gp_Pnt();
        }
    }

    /// <summary>Restarts the computation on the whole curve.</summary>
    public void Initialize(Adaptor3d_Curve C, double Deflection, double Resolution, bool WithControl)
    {
        Initialize(C, Deflection, C.FirstParameter(), C.LastParameter(), Resolution, WithControl);
    }

    /// <summary>Restarts the computation on the whole curve.</summary>
    public void Initialize(Adaptor2d_Curve2d C, double Deflection, double Resolution, bool WithControl)
    {
        Initialize(C, Deflection, C.FirstParameter(), C.LastParameter(), Resolution, WithControl);
    }

    /// <summary>Restarts the computation between the parameters U1 and U2.</summary>
    public void Initialize(Adaptor3d_Curve C, double Deflection, double U1, double U2, double Resolution,
        bool WithControl)
    {
        if (U1 > U2)
        {
            myFirstParam = U2;
            myLastParam = U1;
        }
        else
        {
            myFirstParam = U1;
            myLastParam = U2;
        }
        my3d = true;
        myDwmax = myLastParam - myFirstParam;
        myDu = myDwmax / 2.0;
        myDone = true;
        myCurve = C;
        myFinish = false;
        myTolCur = Resolution;
        myDeflection = Deflection;
        myControl = WithControl;
        Perform();
    }

    /// <summary>Restarts the computation between the parameters U1 and U2.</summary>
    public void Initialize(Adaptor2d_Curve2d C, double Deflection, double U1, double U2, double Resolution,
        bool WithControl)
    {
        if (U1 > U2)
        {
            myFirstParam = U2;
            myLastParam = U1;
        }
        else
        {
            myFirstParam = U1;
            myLastParam = U2;
        }
        my3d = false;
        myDwmax = myLastParam - myFirstParam;
        myDu = myDwmax / 2.0;
        myDone = true;
        myCurve = C;
        myFinish = false;
        myTolCur = Resolution;
        myDeflection = Deflection;
        myControl = WithControl;
        Perform();
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsAllDone() => myDone;

    /// <summary>Moves to the next point.</summary>
    /// <exception cref="Standard_OutOfRange">There are no more points.</exception>
    public void Next()
    {
        Standard_OutOfRange_Raise_if(myIPoint >= myNbPoints,
            "CPnts_UniformDeflection::Next() - no more points available");
        myIPoint++;
    }

    /// <summary>True if there is another point (computing the next ones when needed).</summary>
    public bool More()
    {
        if (!myDone)
        {
            return false;
        }
        else if (myIPoint == myNbPoints)
        {
            if (myFinish)
            {
                return false;
            }
            else
            {
                Perform();
                return myDone;
            }
        }
        else
        {
            return myIPoint < myNbPoints;
        }
    }

    /// <summary>The parameter of the current point.</summary>
    public double Value()
    {
        StdFail_NotDone_Raise_if(!myDone, "CPnts_UniformDeflection::Value() - no result");
        return myParams[myIPoint + 1];
    }

    /// <summary>The current point.</summary>
    public gp_Pnt Point()
    {
        StdFail_NotDone_Raise_if(!myDone, "CPnts_UniformDeflection::Point() - no result");
        return myPoints[myIPoint + 1];
    }
}
