// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GFuncExtPC.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_PCFOfEPCOfExtPC2d.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_PCFOfEPCOfELPCOfLocateExtPC2d.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_PCLocFOfLocEPCOfLocateExtPC2d.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// The function whose roots are the extrema of the distance between a point and a 2D curve:
/// F(u) = (C(u) - P).C'(u) / ||C'(u)||, with its derivative. Used by math_FunctionRoot and math_FunctionRoots;
/// each root they find (GetStateNumber) is stored with its square distance and whether it is a minimum.
/// </summary>
/// <remarks>
/// The 2D instantiation of OCCT's template Extrema_GFuncExtPC, which OCCT names Extrema_PCFOfEPCOfExtPC2d,
/// Extrema_PCFOfEPCOfELPCOfLocateExtPC2d and Extrema_PCLocFOfLocEPCOfLocateExtPC2d (one type). Where the first derivative
/// of a B-spline, Bezier, offset or other curve vanishes, it is replaced by the first non-zero higher derivative or
/// a finite difference. Keeps a reference to the curve, as OCCT does. Where OCCT's <c>Value</c>, <c>Values</c> and
/// <c>Derivative</c> return false without writing their results, the port sets them to 0 (decision 32).
/// </remarks>
public class Extrema_PCFOfEPCOfExtPC2d : FunctionWithDerivative
{
    private const double TolFactor = 1.0e-12;
    private const double MinTol = 1.0e-20;
    private const double MinStep = 1.0e-7;
    private const int MaxOrder = 3;

    private Point2D myP = new();
    private Adaptor2d_Curve2d? myC;
    private double myU;
    private Point2D myPc = new();
    private double myD1f;
    private readonly NCollection_Sequence<double> mySqDist = new();
    private readonly NCollection_Sequence<int> myIsMin = new();
    private readonly NCollection_Sequence<Extrema_POnCurv2d> myPoint = new();
    private bool myPinit;
    private bool myCinit;
#pragma warning disable CS0414 // OCCT sets myD1Init and never reads it
    private bool myD1Init;
#pragma warning restore CS0414
    private double myTol;
    private int myMaxDerivOrder;
    private double myUinfium;
    private double myUsupremum;

    public Extrema_PCFOfEPCOfExtPC2d()
    {
        myC = null;
        myU = 0.0;
        myD1f = 0.0;
        myPinit = false;
        myCinit = false;
        myD1Init = false;
        myTol = MinTol;
        myMaxDerivOrder = 0;
        myUinfium = 0.0;
        myUsupremum = 0.0;
    }

    /// <summary>The function for the point theP and the curve theC.</summary>
    public Extrema_PCFOfEPCOfExtPC2d(in Point2D theP, Adaptor2d_Curve2d theC)
    {
        myP = theP;
        myC = theC;
        myU = 0.0;
        myD1f = 0.0;
        myPinit = true;
        myCinit = true;
        myD1Init = false;

        SubIntervalInitialize(Extrema_Curve2dTool.FirstParameter(theC), Extrema_Curve2dTool.LastParameter(theC));

        switch (Extrema_Curve2dTool.GetType(theC))
        {
            case BezierCurve:
            case BSplineCurve:
            case OffsetCurve:
            case OtherCurve:
                myMaxDerivOrder = MaxOrder;
                myTol = SearchOfTolerance();
                break;
            default:
                myMaxDerivOrder = 0;
                myTol = MinTol;
                break;
        }
    }

    /// <summary>Sets the curve, and clears the extrema found.</summary>
    public void Initialize(Adaptor2d_Curve2d theC)
    {
        myC = theC;
        myCinit = true;
        myPoint.Clear();
        mySqDist.Clear();
        myIsMin.Clear();

        SubIntervalInitialize(Extrema_Curve2dTool.FirstParameter(theC), Extrema_Curve2dTool.LastParameter(theC));

        switch (Extrema_Curve2dTool.GetType(theC))
        {
            case BezierCurve:
            case BSplineCurve:
            case OffsetCurve:
            case OtherCurve:
                myMaxDerivOrder = MaxOrder;
                myTol = SearchOfTolerance();
                break;
            default:
                myMaxDerivOrder = 0;
                myTol = MinTol;
                break;
        }
    }

    /// <summary>Sets the point, and clears the extrema found.</summary>
    public void SetPoint(in Point2D theP)
    {
        myP = theP;
        myPinit = true;
        myPoint.Clear();
        mySqDist.Clear();
        myIsMin.Clear();
    }

    /// <summary>F(u).</summary>
    /// <exception cref="TypeMismatchException">The point or the curve is not set.</exception>
    public override bool Value(double theU, out double theF)
    {
        if (!myPinit || !myCinit)
        {
            throw new TypeMismatchException("No init");
        }

        myU = theU;
        Vector2D D1c;
        Extrema_Curve2dTool.D1(myC!, myU, out myPc, out D1c);

        // OCCT tests X and Y only, never Z
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(D1c.X()) || global::Kerf.Engine.Foundation.Precision.IsInfinite(D1c.Y()))
        {
            theF = global::Kerf.Engine.Foundation.Precision.Infinite();
            return false;
        }

        double Ndu = D1c.Magnitude();

        if (myMaxDerivOrder != 0)
        {
            if (Ndu <= myTol) // Singular case
            {
                const double DivisionFactor = 1.0e-3;
                double du;
                if ((myUsupremum >= RealLast()) || (myUinfium <= RealFirst()))
                {
                    du = 0.0;
                }
                else
                {
                    du = myUsupremum - myUinfium;
                }

                double aDelta = Max(du * DivisionFactor, MinStep);
                // Derivative is approximated by Taylor-series

                int n = 1; // Derivative order
                Vector2D V;
                bool IsDeriveFound;

                do
                {
                    V = Extrema_Curve2dTool.DN(myC!, myU, ++n);
                    Ndu = V.Magnitude();
                    IsDeriveFound = (Ndu > myTol);
                } while (!IsDeriveFound && n < myMaxDerivOrder);

                if (IsDeriveFound)
                {
                    double u;

                    if (myU - myUinfium < aDelta)
                    {
                        u = myU + aDelta;
                    }
                    else
                    {
                        u = myU - aDelta;
                    }

                    Extrema_Curve2dTool.D0(myC!, Min(myU, u), out Point2D P1);
                    Extrema_Curve2dTool.D0(myC!, Max(myU, u), out Point2D P2);

                    Vector2D V1 = new(P1, P2);
                    double aDirFactor = V.Dot(V1);

                    if (aDirFactor < 0.0)
                    {
                        D1c = -V;
                    }
                    else
                    {
                        D1c = V;
                    }
                }
                else
                {
                    // Derivative is approximated by three points
                    Point2D Ptemp = new();
                    Point2D P1, P2, P3;
                    bool IsParameterGrown;

                    if (myU - myUinfium < 2 * aDelta)
                    {
                        Extrema_Curve2dTool.D0(myC!, myU, out P1);
                        Extrema_Curve2dTool.D0(myC!, myU + aDelta, out P2);
                        Extrema_Curve2dTool.D0(myC!, myU + 2 * aDelta, out P3);
                        IsParameterGrown = true;
                    }
                    else
                    {
                        Extrema_Curve2dTool.D0(myC!, myU - 2 * aDelta, out P1);
                        Extrema_Curve2dTool.D0(myC!, myU - aDelta, out P2);
                        Extrema_Curve2dTool.D0(myC!, myU, out P3);
                        IsParameterGrown = false;
                    }

                    Vector2D V1 = new(Ptemp, P1), V2 = new(Ptemp, P2), V3 = new(Ptemp, P3);

                    if (IsParameterGrown)
                    {
                        D1c = -3 * V1 + 4 * V2 - V3;
                    }
                    else
                    {
                        D1c = V1 - 4 * V2 + 3 * V3;
                    }
                }
                Ndu = D1c.Magnitude();
            }
        }

        if (Ndu <= MinTol)
        {
            // Warning: 1st derivative is equal to zero!
            theF = 0.0;
            return false;
        }

        Vector2D PPc = new(myP, myPc);
        theF = PPc.Dot(D1c) / Ndu;
        return true;
    }

    /// <summary>F'(u).</summary>
    /// <exception cref="TypeMismatchException">The point or the curve is not set.</exception>
    public override bool Derivative(double theU, out double theDF)
    {
        if (!myPinit || !myCinit)
        {
            throw new TypeMismatchException();
        }
        return Values(theU, out _, out theDF);
    }

    /// <summary>F(u) and F'(u).</summary>
    /// <exception cref="TypeMismatchException">The point or the curve is not set.</exception>
    public override bool Values(double theU, out double theF, out double theDF)
    {
        if (!myPinit || !myCinit)
        {
            throw new TypeMismatchException("No init");
        }

        Point2D myPc_old = myPc, myP_old = myP;

        if (Value(theU, out theF) == false)
        {
            myD1Init = false;
            theDF = 0.0;
            return false;
        }

        myU = theU;
        myPc = myPc_old;
        myP = myP_old;

        Extrema_Curve2dTool.D2(myC!, myU, out myPc, out Vector2D D1c, out Vector2D D2c);

        double Ndu = D1c.Magnitude();
        if (Ndu <= myTol) // Singular case
        {
            // Derivative is approximated by three points
            const double DivisionFactor = 0.01;
            double du;
            if ((myUsupremum >= RealLast()) || (myUinfium <= RealFirst()))
            {
                du = 0.0;
            }
            else
            {
                du = myUsupremum - myUinfium;
            }

            double aDelta = Max(du * DivisionFactor, MinStep);

            double F1, F2, F3;

            if (myU - myUinfium < 2 * aDelta)
            {
                F1 = theF;
                double U2 = myU + aDelta;
                double U3 = myU + aDelta * 2.0;

                if (!((Value(U2, out F2)) && (Value(U3, out F3))))
                {
                    myD1Init = false;
                    theDF = 0.0;
                    return false;
                }

                theDF = (-3 * F1 + 4 * F2 - F3) / (2.0 * aDelta);
            }
            else
            {
                F3 = theF;
                double U1 = myU - aDelta * 2.0;
                double U2 = myU - aDelta;

                if (!((Value(U2, out F2)) && (Value(U1, out F1))))
                {
                    myD1Init = false;
                    theDF = 0.0;
                    return false;
                }

                theDF = (F1 - 4 * F2 + 3 * F3) / (2.0 * aDelta);
            }
            myU = theU;
            myPc = myPc_old;
            myP = myP_old;
        }
        else
        {
            Vector2D PPc = new(myP, myPc);
            theDF = Ndu + (PPc.Dot(D2c) / Ndu) - theF * (D1c.Dot(D2c)) / (Ndu * Ndu);
        }

        myD1f = theDF;

        myD1Init = true;
        return true;
    }

    /// <summary>Saves the extremum just found: its square distance, point, and whether it is a minimum.</summary>
    /// <exception cref="TypeMismatchException">The point or the curve is not set.</exception>
    public override int GetStateNumber()
    {
        if (!myPinit || !myCinit)
        {
            throw new TypeMismatchException();
        }
        mySqDist.Append(myPc.SquareDistance(myP));

        // It is necessary to always compute myD1f.
        myD1Init = true;
        Values(myU, out _, out _);

        int IntVal = 0;
        if (myD1f > 0.0)
        {
            IntVal = 1;
        }

        myIsMin.Append(IntVal);
        myPoint.Append(new Extrema_POnCurv2d(myU, myPc));
        return 0;
    }

    /// <summary>The number of extrema found.</summary>
    public int NbExt() => mySqDist.Length();

    /// <summary>The Nth square distance.</summary>
    /// <exception cref="TypeMismatchException">The point or the curve is not set.</exception>
    public double SquareDistance(int theN)
    {
        if (!myPinit || !myCinit)
        {
            throw new TypeMismatchException();
        }
        return mySqDist.Value(theN);
    }

    /// <summary>True if the Nth extremum is a minimum.</summary>
    /// <exception cref="TypeMismatchException">The point or the curve is not set.</exception>
    public bool IsMin(int theN)
    {
        if (!myPinit || !myCinit)
        {
            throw new TypeMismatchException();
        }
        return (myIsMin.Value(theN) == 1);
    }

    /// <summary>The Nth extremum point.</summary>
    /// <exception cref="TypeMismatchException">The point or the curve is not set.</exception>
    public Extrema_POnCurv2d Point(int theN)
    {
        if (!myPinit || !myCinit)
        {
            throw new TypeMismatchException();
        }
        return myPoint.Value(theN);
    }

    /// <summary>The bounds of the sub-interval of the root search.</summary>
    public void SubIntervalInitialize(double theUfirst, double theUlast)
    {
        myUinfium = theUfirst;
        myUsupremum = theUlast;
    }

    /// <summary>The tolerance below which the first derivative is taken as zero: 1e-12 of its maximum on 11 samples, at
    /// least 1e-20.</summary>
    public double SearchOfTolerance()
    {
        const int NPoint = 10;
        double aStep = (myUsupremum - myUinfium) / (double)NPoint;

        int aNum = 0;
        double aMax = -global::Kerf.Engine.Foundation.Precision.Infinite();

        do
        {
            double u = myUinfium + aNum * aStep;
            if (u > myUsupremum)
            {
                u = myUsupremum;
            }

            Extrema_Curve2dTool.D1(myC!, u, out _, out Vector2D VDer);

            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(VDer.X()) || global::Kerf.Engine.Foundation.Precision.IsInfinite(VDer.Y()))
            {
                continue;
            }

            double vm = VDer.Magnitude();
            if (vm > aMax)
            {
                aMax = vm;
            }
        } while (++aNum < NPoint + 1);

        return Max(aMax * TolFactor, MinTol);
    }
}
