// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_Variational.hxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_Variational.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// Jeannine PANCIATICI le 06/06/96
// Igor FEOKTISTOV 14/12/98 - correction of Approximate() and Init().
// Approximation of a MultiLine of points described by the tool MLineTool.
// with variational criteria (minimisation of a quadratic criterion and of a "fairness" criterion).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Numerics;
using Kerf.Engine.Numerics.Polynomials;
using Kerf.Geometry;
using static Occt.AppParCurves_Constraint;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// Smooths the points of a MultiLine into B-spline curves under passing, tangency and curvature constraints, by
/// minimising a least-squares criterion together with a variational "fair curve" criterion (tension, flexion,
/// jerk).
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21): the setters are public and recompute the criterion as OCCT does;
/// <see cref="Approximate"/> runs the computation. If (MaxDegree - Continuity) * MaxSegment - Nc0 - 2 Nc1 - 3 Nc2
/// is negative (Nc0, Nc1, Nc2 the numbers of passing, tangency and curvature constraints), the problem is
/// over-constrained (<see cref="IsOverConstrained"/>). The MultiLine must have one line (of dimension 2 or 3) or
/// several of the same kind.
/// </para>
/// <para>
/// <see cref="Value"/> returns a copy of the result that shares its poles, as OCCT's by-value return does;
/// <see cref="Parameters"/> and <see cref="Knots"/> return the object's own arrays (shared: don't modify), as OCCT
/// returns its handles (decision 207). <c>Dump</c> is left out (rules, section 6).
/// </para>
/// </remarks>
public class AppDef_Variational
{
    private readonly AppDef_MultiLine mySSP;
    private readonly int myNbP3d;
    private readonly int myNbP2d;
    private readonly int myDimension;
    private readonly int myFirstPoint;
    private readonly int myLastPoint;
    private readonly int myNbPoints;
    private readonly NCollection_HArray1<double> myTabPoints;
    private NCollection_HArray1<AppParCurves_ConstraintCouple> myConstraints;
    private int myNbConstraints;
    private NCollection_HArray1<double> myTabConstraints = null!;
    private int myNbPassPoints;
    private int myNbTangPoints;
    private int myNbCurvPoints;
    private NCollection_HArray1<int> myTypConstraints = null!;
    private NCollection_HArray1<double> myTtheta = null!;
    private NCollection_HArray1<double> myTfthet = null!;
    private int myMaxDegree;
    private int myMaxSegment;
    private int myNbIterations;
    private double myTolerance;
    private Continuity myContinuity;
    private int myNivCont;
    private bool myWithMinMax;
    private bool myWithCutting;
    private readonly double[] myPercent = new double[3];
    private readonly double[] myCriterium = new double[4];
    private AppDef_SmoothCriterion mySmoothCriterion;
    private readonly NCollection_HArray1<double> myParameters;
    private NCollection_HArray1<double> myKnots;
    private AppParCurves_MultiBSpCurve myMBSpCurve = new();
    private double myMaxError;
    private int myMaxErrorIndex;
    private double myAverageError;
    private bool myIsCreated;
    private bool myIsDone;
    private bool myIsOverConstr;

    /// <summary>
    /// Sets up the smoothing of the points <paramref name="FirstPoint"/> to <paramref name="LastPoint"/> of
    /// <paramref name="SSP"/> under the constraints <paramref name="TheConstraints"/> (kept, shared).
    /// </summary>
    public AppDef_Variational(AppDef_MultiLine SSP, int FirstPoint, int LastPoint,
                              NCollection_HArray1<AppParCurves_ConstraintCouple> TheConstraints, int MaxDegree = 14,
                              int MaxSegment = 100, Continuity Continuity = C2, bool WithMinMax = false,
                              bool WithCutting = true, double Tolerance = 1.0, int NbIterations = 2)
    {
        mySSP = SSP;
        myFirstPoint = FirstPoint;
        myLastPoint = LastPoint;
        myConstraints = TheConstraints;
        myMaxDegree = MaxDegree;
        myMaxSegment = MaxSegment;
        myNbIterations = NbIterations;
        myTolerance = Tolerance;
        myContinuity = Continuity;
        myWithMinMax = WithMinMax;
        myWithCutting = WithCutting;

        // Verifications:
        if (myMaxDegree < 1)
        {
            throw new DomainErrorException();
        }
        myMaxDegree = Min(30, myMaxDegree);
        //
        if (myMaxSegment < 1)
        {
            throw new DomainErrorException();
        }
        // OCCT's checks that myWithMinMax and myWithCutting are 0 or 1 always pass for a bool: left out.
        //
        myIsOverConstr = false;
        myIsCreated = false;
        myIsDone = false;
        switch (myContinuity)
        {
            case C0:
                myNivCont = 0;
                break;
            case C1:
                myNivCont = 1;
                break;
            case C2:
                myNivCont = 2;
                break;
            default:
                throw new ConstructionErrorException();
        }
        //
        myNbP2d = AppDef_MyLineTool.NbP2d(SSP);
        myNbP3d = AppDef_MyLineTool.NbP3d(SSP);
        myDimension = 2 * myNbP2d + 3 * myNbP3d;
        //
        myPercent[0] = 0.4;
        myPercent[1] = 0.2;
        myPercent[2] = 0.4;
        myKnots = new NCollection_HArray1<double>(1, 2);
        myKnots.SetValue(1, 0.0);
        myKnots.SetValue(2, 1.0);

        //  Declaration
        //
        mySmoothCriterion = new AppDef_LinearCriteria(mySSP, myFirstPoint, myLastPoint);
        myParameters = new NCollection_HArray1<double>(myFirstPoint, myLastPoint);
        myNbPoints = myLastPoint - myFirstPoint + 1;
        if (myNbPoints <= 0)
        {
            throw new ConstructionErrorException();
        }
        //
        myTabPoints = new NCollection_HArray1<double>(1, myDimension * myNbPoints);
        //
        //  Table of Points initialization
        //
        int ipoint, jp2d, jp3d, index;
        NCollection_Array1<Point3D> TabP3d = new(1, Max(1, myNbP3d));
        NCollection_Array1<Point2D> TabP2d = new(1, Max(1, myNbP2d));
        Point2D P2d;
        Point3D P3d;
        index = 1;

        for (ipoint = myFirstPoint; ipoint <= myLastPoint; ipoint++)
        {
            if (myNbP2d != 0 && myNbP3d == 0)
            {
                AppDef_MyLineTool.Value(mySSP, ipoint, TabP2d);

                for (jp2d = 1; jp2d <= myNbP2d; jp2d++)
                {
                    P2d = TabP2d.Value(jp2d);

                    myTabPoints.SetValue(index++, P2d.X());
                    myTabPoints.SetValue(index++, P2d.Y());
                }
            }
            if (myNbP3d != 0 && myNbP2d == 0)
            {
                AppDef_MyLineTool.Value(mySSP, ipoint, TabP3d);

                for (jp3d = 1; jp3d <= myNbP3d; jp3d++)
                {
                    P3d = TabP3d.Value(jp3d);

                    myTabPoints.SetValue(index++, P3d.X());
                    myTabPoints.SetValue(index++, P3d.Y());
                    myTabPoints.SetValue(index++, P3d.Z());
                }
            }
            if (myNbP3d != 0 && myNbP2d != 0)
            {
                AppDef_MyLineTool.Value(mySSP, ipoint, TabP3d, TabP2d);

                for (jp3d = 1; jp3d <= myNbP3d; jp3d++)
                {
                    P3d = TabP3d.Value(jp3d);

                    myTabPoints.SetValue(index++, P3d.X());
                    myTabPoints.SetValue(index++, P3d.Y());
                    myTabPoints.SetValue(index++, P3d.Z());
                }
                for (jp2d = 1; jp2d <= myNbP2d; jp2d++)
                {
                    P2d = TabP2d.Value(jp2d);

                    myTabPoints.SetValue(index++, P2d.X());
                    myTabPoints.SetValue(index++, P2d.Y());
                }
            }
        }
        Init();
    }

    private void Init()
    {
        int ipoint, jp2d, jp3d, index, jndex;
        int CurMultyPoint;
        NCollection_Array1<Vector3D> TabV3d = new(1, Max(1, myNbP3d));
        NCollection_Array1<Vector2D> TabV2d = new(1, Max(1, myNbP2d));
        NCollection_Array1<Vector3D> TabV3dcurv = new(1, Max(1, myNbP3d));
        NCollection_Array1<Vector2D> TabV2dcurv = new(1, Max(1, myNbP2d));

        Vector3D Vt3d, Vc3d;
        Vector2D Vt2d, Vc2d;

        myNbConstraints = myConstraints.Length();
        if (myNbConstraints < 0)
        {
            throw new ConstructionErrorException();
        }

        myTypConstraints = new NCollection_HArray1<int>(1, Max(1, 2 * myNbConstraints));
        myTabConstraints = new NCollection_HArray1<double>(1, Max(1, 2 * myDimension * myNbConstraints));
        myTtheta = new NCollection_HArray1<double>(1, Max(1, (2 * myNbP2d + 6 * myNbP3d) * myNbConstraints));
        myTfthet = new NCollection_HArray1<double>(1, Max(1, (2 * myNbP2d + 6 * myNbP3d) * myNbConstraints));

        //
        // Table of types initialization
        int iconstr;
        index = 1;
        jndex = 1;
        CurMultyPoint = 1;
        myNbPassPoints = 0;
        myNbTangPoints = 0;
        myNbCurvPoints = 0;
        AppParCurves_Constraint valcontr;

        for (iconstr = myConstraints.Lower(); iconstr <= myConstraints.Upper(); iconstr++)
        {
            ipoint = (myConstraints.Value(iconstr)).Index();

            valcontr = (myConstraints.Value(iconstr)).Constraint();
            switch (valcontr)
            {
                case AppParCurves_NoConstraint:
                    CurMultyPoint -= myNbP3d * 6 + myNbP2d * 2;
                    break;
                case AppParCurves_PassPoint:
                    myTypConstraints.SetValue(index++, ipoint);
                    myTypConstraints.SetValue(index++, 0);
                    myNbPassPoints++;
                    if (myNbP2d != 0)
                    {
                        jndex = jndex + 4 * myNbP2d;
                    }
                    if (myNbP3d != 0)
                    {
                        jndex = jndex + 6 * myNbP3d;
                    }
                    break;
                case AppParCurves_TangencyPoint:
                    myTypConstraints.SetValue(index++, ipoint);
                    myTypConstraints.SetValue(index++, 1);
                    myNbTangPoints++;
                    if (myNbP2d != 0 && myNbP3d == 0)
                    {
                        if (!AppDef_MyLineTool.Tangency(mySSP, ipoint, TabV2d))
                        {
                            throw new ConstructionErrorException();
                        }
                        for (jp2d = 1; jp2d <= myNbP2d; jp2d++)
                        {
                            Vt2d = TabV2d.Value(jp2d);
                            Vt2d.Normalize();
                            myTabConstraints.SetValue(jndex++, Vt2d.X());
                            myTabConstraints.SetValue(jndex++, Vt2d.Y());
                            jndex = jndex + 2;
                            InitTthetaF(2, valcontr, CurMultyPoint + (jp2d - 1) * 2, jndex - 4);
                        }
                    }
                    if (myNbP3d != 0 && myNbP2d == 0)
                    {
                        if (!AppDef_MyLineTool.Tangency(mySSP, ipoint, TabV3d))
                        {
                            throw new ConstructionErrorException();
                        }
                        for (jp3d = 1; jp3d <= myNbP3d; jp3d++)
                        {
                            Vt3d = TabV3d.Value(jp3d);
                            Vt3d.Normalize();
                            myTabConstraints.SetValue(jndex++, Vt3d.X());

                            myTabConstraints.SetValue(jndex++, Vt3d.Y());

                            myTabConstraints.SetValue(jndex++, Vt3d.Z());
                            jndex = jndex + 3;
                            InitTthetaF(3, valcontr, CurMultyPoint + (jp3d - 1) * 6, jndex - 6);
                        }
                    }
                    if (myNbP3d != 0 && myNbP2d != 0)
                    {
                        if (!AppDef_MyLineTool.Tangency(mySSP, ipoint, TabV3d, TabV2d))
                        {
                            throw new ConstructionErrorException();
                        }
                        for (jp3d = 1; jp3d <= myNbP3d; jp3d++)
                        {
                            Vt3d = TabV3d.Value(jp3d);
                            Vt3d.Normalize();
                            myTabConstraints.SetValue(jndex++, Vt3d.X());
                            myTabConstraints.SetValue(jndex++, Vt3d.Y());
                            myTabConstraints.SetValue(jndex++, Vt3d.Z());
                            jndex = jndex + 3;
                            InitTthetaF(3, valcontr, CurMultyPoint + (jp3d - 1) * 6, jndex - 6);
                        }

                        for (jp2d = 1; jp2d <= myNbP2d; jp2d++)
                        {
                            Vt2d = TabV2d.Value(jp2d);
                            Vt2d.Normalize();
                            myTabConstraints.SetValue(jndex++, Vt2d.X());
                            myTabConstraints.SetValue(jndex++, Vt2d.Y());
                            jndex = jndex + 2;
                            InitTthetaF(2, valcontr, CurMultyPoint + (jp2d - 1) * 2 + myNbP3d * 6, jndex - 4);
                        }
                    }

                    break;

                case AppParCurves_CurvaturePoint:
                    myTypConstraints.SetValue(index++, ipoint);
                    myTypConstraints.SetValue(index++, 2);
                    myNbCurvPoints++;
                    if (myNbP2d != 0 && myNbP3d == 0)
                    {
                        if (!AppDef_MyLineTool.Tangency(mySSP, ipoint, TabV2d))
                        {
                            throw new ConstructionErrorException();
                        }
                        if (!AppDef_MyLineTool.Curvature(mySSP, ipoint, TabV2dcurv))
                        {
                            throw new ConstructionErrorException();
                        }
                        for (jp2d = 1; jp2d <= myNbP2d; jp2d++)
                        {
                            Vt2d = TabV2d.Value(jp2d);
                            Vt2d.Normalize();
                            Vc2d = TabV2dcurv.Value(jp2d);
                            if (Math.Abs(Math.Abs(Vc2d.Angle(Vt2d)) - M_PI / 2.0) > global::Kerf.Engine.Foundation.Precision.Angular())
                            {
                                throw new ConstructionErrorException();
                            }
                            myTabConstraints.SetValue(jndex++, Vt2d.X());
                            myTabConstraints.SetValue(jndex++, Vt2d.Y());
                            myTabConstraints.SetValue(jndex++, Vc2d.X());
                            myTabConstraints.SetValue(jndex++, Vc2d.Y());
                            InitTthetaF(2, valcontr, CurMultyPoint + (jp2d - 1) * 2, jndex - 4);
                        }
                    }

                    if (myNbP3d != 0 && myNbP2d == 0)
                    {
                        if (!AppDef_MyLineTool.Tangency(mySSP, ipoint, TabV3d))
                        {
                            throw new ConstructionErrorException();
                        }
                        if (!AppDef_MyLineTool.Curvature(mySSP, ipoint, TabV3dcurv))
                        {
                            throw new ConstructionErrorException();
                        }
                        for (jp3d = 1; jp3d <= myNbP3d; jp3d++)
                        {
                            Vt3d = TabV3d.Value(jp3d);
                            Vt3d.Normalize();
                            Vc3d = TabV3dcurv.Value(jp3d);
                            if (!(Vc3d.Normalized()).IsNormal(Vt3d, global::Kerf.Engine.Foundation.Precision.Angular()))
                            {
                                throw new ConstructionErrorException();
                            }
                            myTabConstraints.SetValue(jndex++, Vt3d.X());
                            myTabConstraints.SetValue(jndex++, Vt3d.Y());
                            myTabConstraints.SetValue(jndex++, Vt3d.Z());
                            myTabConstraints.SetValue(jndex++, Vc3d.X());
                            myTabConstraints.SetValue(jndex++, Vc3d.Y());
                            myTabConstraints.SetValue(jndex++, Vc3d.Z());
                            InitTthetaF(3, valcontr, CurMultyPoint + (jp3d - 1) * 6, jndex - 6);
                        }
                    }
                    if (myNbP3d != 0 && myNbP2d != 0)
                    {
                        if (!AppDef_MyLineTool.Tangency(mySSP, ipoint, TabV3d, TabV2d))
                        {
                            throw new ConstructionErrorException();
                        }
                        if (!AppDef_MyLineTool.Curvature(mySSP, ipoint, TabV3dcurv, TabV2dcurv))
                        {
                            throw new ConstructionErrorException();
                        }
                        for (jp3d = 1; jp3d <= myNbP3d; jp3d++)
                        {
                            Vt3d = TabV3d.Value(jp3d);
                            Vt3d.Normalize();
                            Vc3d = TabV3dcurv.Value(jp3d);
                            if (!(Vc3d.Normalized()).IsNormal(Vt3d, global::Kerf.Engine.Foundation.Precision.Angular()))
                            {
                                throw new ConstructionErrorException();
                            }
                            myTabConstraints.SetValue(jndex++, Vt3d.X());
                            myTabConstraints.SetValue(jndex++, Vt3d.Y());
                            myTabConstraints.SetValue(jndex++, Vt3d.Z());
                            myTabConstraints.SetValue(jndex++, Vc3d.X());
                            myTabConstraints.SetValue(jndex++, Vc3d.Y());
                            myTabConstraints.SetValue(jndex++, Vc3d.Z());
                            InitTthetaF(3, valcontr, CurMultyPoint + (jp3d - 1) * 6, jndex - 6);
                        }
                        for (jp2d = 1; jp2d <= myNbP2d; jp2d++)
                        {
                            Vt2d = TabV2d.Value(jp2d);
                            Vt2d.Normalize();
                            Vc2d = TabV2dcurv.Value(jp2d);
                            if (Math.Abs(Math.Abs(Vc2d.Angle(Vt2d)) - M_PI / 2.0) > global::Kerf.Engine.Foundation.Precision.Angular())
                            {
                                throw new ConstructionErrorException();
                            }
                            myTabConstraints.SetValue(jndex++, Vt2d.X());
                            myTabConstraints.SetValue(jndex++, Vt2d.Y());
                            myTabConstraints.SetValue(jndex++, Vc2d.X());
                            myTabConstraints.SetValue(jndex++, Vc2d.Y());
                            InitTthetaF(2, valcontr, CurMultyPoint + (jp2d - 1) * 2 + myNbP3d * 6, jndex - 4);
                        }
                    }
                    break;
                default:
                    throw new ConstructionErrorException();
            }
            CurMultyPoint += myNbP3d * 6 + myNbP2d * 2;
        }
        // OverConstraint Detection
        int MaxSeg;
        if (myWithCutting)
        {
            MaxSeg = myMaxSegment;
        }
        else
        {
            MaxSeg = 1;
        }
        if (((myMaxDegree - myNivCont) * MaxSeg - myNbPassPoints - 2 * myNbTangPoints - 3 * myNbCurvPoints) < 0)
        {
            myIsOverConstr = true;
            myIsCreated = false;
        }
        else
        {
            InitSmoothCriterion();
            myIsCreated = true;
        }
    }

    /// <summary>Makes the approximation with the current fields. Throws StdFail_NotDone unless <see cref="IsCreated"/>.</summary>
    public void Approximate()
    {
        if (!myIsCreated)
        {
            throw new NotDoneException();
        }

        double WQuadratic, WQuality;

        NCollection_Array1<double> Ecarts = new(myFirstPoint, myLastPoint);

        mySmoothCriterion.GetWeight(out WQuadratic, out WQuality);

        FEmTool_Curve? TheCurve;

        mySmoothCriterion.GetCurve(out TheCurve);

        //---------------------------------------------------------------------

        TheMotor(mySmoothCriterion, WQuadratic, WQuality, ref TheCurve!, Ecarts);

        if (myWithMinMax && myTolerance < myMaxError)
        {
            Adjusting(ref mySmoothCriterion, ref WQuadratic, ref WQuality, ref TheCurve, Ecarts);
        }

        //---------------------------------------------------------------------

        int jp2d, jp3d, ipole, NbElem = TheCurve.NbElements();

        NCollection_Array1<Point3D> TabP3d = new(1, Max(1, myNbP3d));
        NCollection_Array1<Point2D> TabP2d = new(1, Max(1, myNbP2d));
        ReadOnlySpan<double> debfin = [-1.0, 1];

        Point2D P2d = new();
        Point3D P3d = new();
        {
            NCollection_HArray2<double> PolynomialIntervalsPtr = new(1, NbElem, 1, 2);

            NCollection_HArray1<int> NbCoeffPtr = new(1, myMaxSegment);

            int size = myMaxSegment * (myMaxDegree + 1) * myDimension;
            NCollection_HArray1<double> CoeffPtr = new(1, size);

            CoeffPtr.Init(0.0);

            NCollection_HArray1<double> IntervallesPtr = new(1, NbElem + 1);

            IntervallesPtr.ChangeArray1().Assign(TheCurve.Knots());

            TheCurve.GetPolynom(CoeffPtr.ChangeArray1());

            int ii;

            for (ii = 1; ii <= NbElem; ii++)
            {
                NbCoeffPtr.SetValue(ii, TheCurve.Degree(ii) + 1);
            }

            for (ii = PolynomialIntervalsPtr.LowerRow(); ii <= PolynomialIntervalsPtr.UpperRow(); ii++)
            {
                PolynomialIntervalsPtr.SetValue(ii, 1, debfin[0]);
                PolynomialIntervalsPtr.SetValue(ii, 2, debfin[1]);
            }
            CompPolynomialToPoles AConverter = new(NbElem,
                                                           myNivCont,
                                                           myDimension,
                                                           myMaxDegree,
                                                           NbCoeffPtr,
                                                           CoeffPtr,
                                                           PolynomialIntervalsPtr,
                                                           IntervallesPtr);
            if (AConverter.IsDone)
            {
                NCollection_HArray1<int> Mults;
                int NbPoles = AConverter.NbPoles();
                NCollection_Array1<AppParCurves_MultiPoint> TabMU = new(1, NbPoles);
                NCollection_Array2<double> aPoles = AConverter.Poles();
                myKnots = new NCollection_HArray1<double>(AConverter.Knots());
                Mults = new NCollection_HArray1<int>(AConverter.Multiplicities());

                for (ipole = aPoles.LowerRow(); ipole <= aPoles.UpperRow(); ipole++)
                {
                    int index = aPoles.LowerCol();
                    if (myNbP3d != 0)
                    {
                        for (jp3d = 1; jp3d <= myNbP3d; jp3d++)
                        {
                            P3d.SetX(aPoles.Value(ipole, index++));
                            P3d.SetY(aPoles.Value(ipole, index++));
                            P3d.SetZ(aPoles.Value(ipole, index++));
                            TabP3d.SetValue(jp3d, P3d);
                        }
                    }
                    if (myNbP2d != 0)
                    {
                        for (jp2d = 1; jp2d <= myNbP2d; jp2d++)
                        {
                            P2d.SetX(aPoles.Value(ipole, index++));
                            P2d.SetY(aPoles.Value(ipole, index++));
                            TabP2d.SetValue(jp2d, P2d);
                        }
                    }
                    if (myNbP2d != 0 && myNbP3d != 0)
                    {
                        AppParCurves_MultiPoint aMultiPoint = new(TabP3d, TabP2d);
                        TabMU.SetValue(ipole, aMultiPoint);
                    }
                    else if (myNbP2d != 0)
                    {
                        AppParCurves_MultiPoint aMultiPoint = new(TabP2d);
                        TabMU.SetValue(ipole, aMultiPoint);
                    }
                    else
                    {
                        AppParCurves_MultiPoint aMultiPoint = new(TabP3d);
                        TabMU.SetValue(ipole, aMultiPoint);
                    }
                }
                AppParCurves_MultiBSpCurve aCurve = new(TabMU, myKnots.Array1(), Mults.Array1());
                myMBSpCurve = aCurve;
                myIsDone = true;
            }
        }
    }

    /// <summary>True if the creation is done and corresponds to the current fields.</summary>
    public bool IsCreated() => myIsCreated;

    /// <summary>True if the approximation is done and corresponds to the current fields.</summary>
    public bool IsDone() => myIsDone;

    /// <summary>True if the problem is over-constrained, in which case the approximation can't be done.</summary>
    public bool IsOverConstrained() => myIsOverConstr;

    /// <summary>
    /// The B-spline curves approximating the MultiLine (a copy sharing the poles). Throws StdFail_NotDone unless
    /// <see cref="IsDone"/>.
    /// </summary>
    public AppParCurves_MultiBSpCurve Value()
    {
        if (!myIsDone)
        {
            throw new NotDoneException();
        }
        return new AppParCurves_MultiBSpCurve(myMBSpCurve);
    }

    /// <summary>The largest distance between the points of the MultiLine and the curves.</summary>
    public double MaxError()
    {
        if (!myIsDone)
        {
            throw new NotDoneException();
        }
        return myMaxError;
    }

    /// <summary>The index of the MultiPoint of <see cref="MaxError"/>.</summary>
    public int MaxErrorIndex()
    {
        if (!myIsDone)
        {
            throw new NotDoneException();
        }
        return myMaxErrorIndex;
    }

    /// <summary>The quadratic error: the sum of the squared distances between the points and the curves.</summary>
    public double QuadraticError()
    {
        if (!myIsDone)
        {
            throw new NotDoneException();
        }
        return myCriterium[0];
    }

    /// <summary>
    /// Writes into <paramref name="mat"/> the distances between the points of the MultiLine and the curves:
    /// mat(curve, point), rows from 1, columns from the matrix's lower column for the first point.
    /// </summary>
    public void Distance(Matrix mat)
    {
        if (!myIsDone)
        {
            throw new NotDoneException();
        }
        int ipoint, jp2d, jp3d, index;
        NCollection_Array1<Point3D> TabP3d = new(1, Max(1, myNbP3d));
        NCollection_Array1<Point2D> TabP2d = new(1, Max(1, myNbP2d));
        int j0 = mat.LowerCol() - myFirstPoint;

        Point2D P2d;
        Point3D P3d;

        Point3D Pt3d;
        Point2D Pt2d;

        for (ipoint = myFirstPoint; ipoint <= myLastPoint; ipoint++)
        {
            index = 1;
            if (myNbP3d != 0)
            {
                AppDef_MyLineTool.Value(mySSP, ipoint, TabP3d);

                for (jp3d = 1; jp3d <= myNbP3d; jp3d++)
                {
                    P3d = TabP3d.Value(jp3d);
                    myMBSpCurve.Value(index, myParameters.Value(ipoint), out Pt3d);
                    mat[index++, j0 + ipoint] = P3d.Distance(Pt3d);
                }
            }
            if (myNbP2d != 0)
            {
                if (myNbP3d == 0)
                {
                    AppDef_MyLineTool.Value(mySSP, ipoint, TabP2d);
                }
                else
                {
                    AppDef_MyLineTool.Value(mySSP, ipoint, TabP3d, TabP2d);
                }
                for (jp2d = 1; jp2d <= myNbP2d; jp2d++)
                {
                    P2d = TabP2d.Value(jp2d);
                    myMBSpCurve.Value(index, myParameters.Value(ipoint), out Pt2d);
                    mat[index++, j0 + ipoint] = P2d.Distance(Pt2d);
                }
            }
        }
    }

    /// <summary>The average distance between the points of the MultiLine and the curves.</summary>
    public double AverageError()
    {
        if (!myIsDone)
        {
            throw new NotDoneException();
        }
        return myAverageError;
    }

    /// <summary>The parameters of the points used by the approximation (the object's own array: don't modify).</summary>
    public NCollection_HArray1<double> Parameters()
    {
        if (!myIsDone)
        {
            throw new NotDoneException();
        }
        return myParameters;
    }

    /// <summary>The knots used by the approximation (the object's own array: don't modify).</summary>
    public NCollection_HArray1<double> Knots()
    {
        if (!myIsDone)
        {
            throw new NotDoneException();
        }
        return myKnots;
    }

    /// <summary>The square roots of the three quality criteria (tension, flexion, jerk).</summary>
    public void Criterium(out double VFirstOrder, out double VSecondOrder, out double VThirdOrder)
    {
        if (!myIsDone)
        {
            throw new NotDoneException();
        }
        VFirstOrder = myCriterium[1];
        VSecondOrder = myCriterium[2];
        VThirdOrder = myCriterium[3];
    }

    /// <summary>The weights (shares summing to 1) of the three quality criteria.</summary>
    public void CriteriumWeight(out double Percent1, out double Percent2, out double Percent3)
    {
        Percent1 = myPercent[0];
        Percent2 = myPercent[1];
        Percent3 = myPercent[2];
    }

    /// <summary>The maximum degree used in the approximation.</summary>
    public int MaxDegree() => myMaxDegree;

    /// <summary>The maximum number of segments used in the approximation.</summary>
    public int MaxSegment() => myMaxSegment;

    /// <summary>The continuity used in the approximation.</summary>
    public Continuity Continuity() => myContinuity;

    /// <summary>Whether the approximation minimises the maximum error.</summary>
    public bool WithMinMax() => myWithMinMax;

    /// <summary>Whether the approximation can insert knots.</summary>
    public bool WithCutting() => myWithCutting;

    /// <summary>The tolerance used in the approximation.</summary>
    public double Tolerance() => myTolerance;

    /// <summary>The number of iterations used in the approximation.</summary>
    public int NbIterations() => myNbIterations;

    // OCCT's Dump (iostream formatting of the state) is left out (rules, section 6).

    /// <summary>
    /// Sets the constraints (kept, shared) and recomputes the criterion; false if the problem is then
    /// over-constrained.
    /// </summary>
    public bool SetConstraints(NCollection_HArray1<AppParCurves_ConstraintCouple> aConstraint)
    {
        myConstraints = aConstraint;
        Init();
        return !myIsOverConstr;
    }

    /// <summary>Sets (copies) the parameters used by the approximation.</summary>
    public void SetParameters(NCollection_HArray1<double> param)
    {
        myParameters.ChangeArray1().Assign(param.Array1());
    }

    /// <summary>Sets (copies) the knots used by the approximation; always true.</summary>
    public bool SetKnots(NCollection_HArray1<double> knots)
    {
        myKnots.ChangeArray1().Assign(knots.Array1());
        return true;
    }

    /// <summary>Sets the maximum degree; false, changing nothing, if the problem would be over-constrained.</summary>
    public bool SetMaxDegree(int Degree)
    {
        if (((Degree - myNivCont) * myMaxSegment - myNbPassPoints - 2 * myNbTangPoints - 3 * myNbCurvPoints) < 0)
        {
            return false;
        }
        else
        {
            myMaxDegree = Degree;

            InitSmoothCriterion();

            return true;
        }
    }

    /// <summary>Sets the maximum number of segments; false, changing nothing, if the problem would be over-constrained.</summary>
    public bool SetMaxSegment(int NbSegment)
    {
        if (myWithCutting
            && ((myMaxDegree - myNivCont) * NbSegment - myNbPassPoints - 2 * myNbTangPoints - 3 * myNbCurvPoints) < 0)
        {
            return false;
        }
        else
        {
            myMaxSegment = NbSegment;
            return true;
        }
    }

    /// <summary>
    /// Sets the continuity (C0, C1 or C2; Standard_ConstructionError otherwise); false, changing nothing, if the
    /// problem would be over-constrained.
    /// </summary>
    public bool SetContinuity(Continuity C)
    {
        int NivCont = 0;
        switch (C)
        {
            case C0:
                NivCont = 0;
                break;
            case C1:
                NivCont = 1;
                break;
            case C2:
                NivCont = 2;
                break;
            default:
                throw new ConstructionErrorException();
        }
        if (((myMaxDegree - NivCont) * myMaxSegment - myNbPassPoints - 2 * myNbTangPoints - 3 * myNbCurvPoints) < 0)
        {
            return false;
        }
        else
        {
            myContinuity = C;
            myNivCont = NivCont;

            InitSmoothCriterion();
            return true;
        }
    }

    /// <summary>Sets whether the approximation minimises the maximum error.</summary>
    public void SetWithMinMax(bool MinMax)
    {
        myWithMinMax = MinMax;

        InitSmoothCriterion();
    }

    /// <summary>Sets whether the approximation can insert knots; false, changing nothing, if the problem would be over-constrained.</summary>
    public bool SetWithCutting(bool Cutting)
    {
        if (!Cutting)
        {
            if (((myMaxDegree - myNivCont) * myKnots.Length() - myNbPassPoints - 2 * myNbTangPoints
                 - 3 * myNbCurvPoints)
                < 0)
            {
                return false;
            }
            else
            {
                myWithCutting = Cutting;
                InitSmoothCriterion();
                return true;
            }
        }
        else
        {
            if (((myMaxDegree - myNivCont) * myMaxSegment - myNbPassPoints - 2 * myNbTangPoints
                 - 3 * myNbCurvPoints)
                < 0)
            {
                return false;
            }
            else
            {
                myWithCutting = Cutting;
                InitSmoothCriterion();
                return true;
            }
        }
    }

    /// <summary>
    /// Sets the weights of the three quality criteria (normalised to sum to 1). Throws Standard_DomainError if one
    /// is negative.
    /// </summary>
    public void SetCriteriumWeight(double Percent1, double Percent2, double Percent3)
    {
        if (Percent1 < 0 || Percent2 < 0 || Percent3 < 0)
        {
            throw new DomainErrorException();
        }
        double Total = Percent1 + Percent2 + Percent3;
        myPercent[0] = Percent1 / Total;
        myPercent[1] = Percent2 / Total;
        myPercent[2] = Percent3 / Total;

        InitSmoothCriterion();
    }

    /// <summary>
    /// Sets the weight of the criterion <paramref name="Order"/> (1 to 3; Standard_ConstructionError otherwise) and
    /// renormalises the three. Throws Standard_DomainError if <paramref name="Percent"/> is negative.
    /// </summary>
    public void SetCriteriumWeight(int Order, double Percent)
    {
        if (Percent < 0)
        {
            throw new DomainErrorException();
        }
        if (Order < 1 || Order > 3)
        {
            throw new ConstructionErrorException();
        }
        myPercent[Order - 1] = Percent;
        double Total = myPercent[0] + myPercent[1] + myPercent[2];
        myPercent[0] = myPercent[0] / Total;
        myPercent[1] = myPercent[1] / Total;
        myPercent[2] = myPercent[2] / Total;

        InitSmoothCriterion();
    }

    /// <summary>Sets the tolerance used in the approximation.</summary>
    public void SetTolerance(double Tol)
    {
        myTolerance = Tol;
        InitSmoothCriterion();
    }

    /// <summary>Sets the number of iterations used in the approximation.</summary>
    public void SetNbIterations(int Iter)
    {
        myNbIterations = Iter;
    }

    //====================== Private Methodes =============================//

    private void TheMotor(AppDef_SmoothCriterion J, double WQuadratic, double WQuality, ref FEmTool_Curve TheCurve,
                          NCollection_Array1<double> Ecarts)
    {
        const double BigValue = 1.0e37, SmallValue = 1.0e-6, SmallestValue = 1.0e-9;

        NCollection_HArray1<double> CurrentTi, NewTi, OldTi;
        NCollection_HArray2<int> Dependence;
        bool lestim, ToOptim, iscut;
        bool isnear = false, again = true;
        int NbEst, ICDANA, Iter;
        // OCCT leaves NumPnt uninitialised: Project sets it only where a distance is positive (occt-issues.md).
        int NumPnt = 0;
        int MaxNbEst = 5;
        double[] VOCRI = [BigValue, BigValue, BigValue];
        double EROLD = BigValue, ERRMAX = BigValue, ERRMOY, ERRQUA;
        double[] VALCRI = new double[3];
        double CBLONG, LNOLD;
        int NbrPnt = myLastPoint - myFirstPoint + 1;
        int NbrConstraint = myNbPassPoints + myNbTangPoints + myNbCurvPoints;
        FEmTool_Curve CCurrent, COld;
        FEmTool_Curve? CNew = null;
        double EpsLength = SmallValue;
        double EpsDeg;

        double e1, e2, e3;
        double J1min, J2min, J3min;
        int iprog;

        // (0) Init

        J.GetEstimation(out e1, out e2, out e3);
        J1min = 1.0e-8;
        J2min = J3min = (e1 + 1.0e-8) * 1.0e-6;

        if (e1 < J1min)
        {
            e1 = J1min; // Like in
        }
        if (e2 < J2min)
        {
            e2 = J2min; // MOTLIS
        }
        if (e3 < J3min)
        {
            e3 = J3min;
        }

        J.SetEstimation(e1, e2, e3);

        CCurrent = TheCurve;
        CurrentTi = new NCollection_HArray1<double>(1, myParameters.Length());
        CurrentTi.ChangeArray1().Assign(myParameters.Array1());
        OldTi = new NCollection_HArray1<double>(1, CurrentTi.Length());
        OldTi.ChangeArray1().Assign(CurrentTi.Array1());
        COld = CCurrent;
        LNOLD = CBLONG = J.EstLength();
        Dependence = J.DependenceTable();

        J.SetCurve(CCurrent);
        FEmTool_Assembly TheAssembly = new(Dependence.Array2(), J.AssemblyTable());

        //============        Optimization      ============================
        while (again)
        {
            // (1) Loop  Optimization / Estimation
            lestim = true;
            bool lconst = true;
            NbEst = 0;

            J.SetCurve(CCurrent);

            while (lestim)
            {
                //     (1.1) Curve's Optimization.
                EpsLength = SmallValue * CBLONG / NbrPnt;
                CNew = new FEmTool_Curve(CCurrent.Dimension(), CCurrent.NbElements(), CCurrent.Base(), EpsLength);
                CNew.Knots().Assign(CCurrent.Knots());

                J.SetParameters(CurrentTi);
                EpsDeg = Min(WQuality * .1, CBLONG * .001);

                Optimization(J, TheAssembly, lconst, EpsDeg, CNew, CurrentTi.Array1());

                lconst = false;

                //        (1.2) calculation of quality criteria and improvement
                //              of estimation.
                ICDANA = J.QualityValues(J1min, J2min, J3min, out VALCRI[0], out VALCRI[1], out VALCRI[2]);

                if (ICDANA > 0)
                {
                    lconst = true;
                }

                J.ErrorValues(out ERRMAX, out ERRQUA, out ERRMOY);

                isnear = ((Math.Sqrt(ERRQUA / NbrPnt) < 2 * WQuality) && (myNbIterations > 1));

                //       (1.3) Optimization of ti by orthogonal projection
                //             and calculation of the error at points.

                if (isnear)
                {
                    NewTi = new NCollection_HArray1<double>(1, CurrentTi.Length());
                    Project(CNew, CurrentTi.Array1(), NewTi.ChangeArray1(), Ecarts, ref NumPnt, out ERRMAX, out ERRQUA,
                            out ERRMOY, 2);
                }
                else
                {
                    NewTi = CurrentTi;
                }

                //        (1.4) Progression's test
                iprog = 0;
                if ((EROLD > WQuality) && (ERRMAX < 0.95 * EROLD))
                {
                    iprog++;
                }
                if ((EROLD > WQuality) && (ERRMAX < 0.8 * EROLD))
                {
                    iprog++;
                }
                if ((EROLD > WQuality) && (ERRMAX < WQuality))
                {
                    iprog++;
                }
                if ((EROLD > WQuality) && (ERRMAX < 0.99 * EROLD) && (ERRMAX < 1.1 * WQuality))
                {
                    iprog++;
                }
                if (VALCRI[0] < 0.975 * VOCRI[0])
                {
                    iprog++;
                }
                if (VALCRI[0] < 0.9 * VOCRI[0])
                {
                    iprog++;
                }
                if (VALCRI[1] < 0.95 * VOCRI[1])
                {
                    iprog++;
                }
                if (VALCRI[1] < 0.8 * VOCRI[1])
                {
                    iprog++;
                }
                if (VALCRI[2] < 0.95 * VOCRI[2])
                {
                    iprog++;
                }
                if (VALCRI[2] < 0.8 * VOCRI[2])
                {
                    iprog++;
                }
                if ((VOCRI[1] > SmallestValue) && (VOCRI[2] > SmallestValue))
                {
                    if ((VALCRI[1] / VOCRI[1] + 2 * VALCRI[2] / VOCRI[2]) < 2.8)
                    {
                        iprog++;
                    }
                }

                if (iprog < 2 && NbEst == 0)
                {
                    //             (1.5) Invalidation of new knots.
                    VALCRI[0] = VOCRI[0];
                    VALCRI[1] = VOCRI[1];
                    VALCRI[2] = VOCRI[2];
                    ERRMAX = EROLD;
                    CBLONG = LNOLD;
                    CCurrent = COld;
                    CurrentTi = OldTi;

                    goto L8000; // exit
                }

                VOCRI[0] = VALCRI[0];
                VOCRI[1] = VALCRI[1];
                VOCRI[2] = VALCRI[2];
                LNOLD = CBLONG;
                EROLD = ERRMAX;

                CCurrent = CNew;
                CurrentTi = NewTi;

                //       (1.6) Test if the Estimations seems  OK, else repeat
                NbEst++;
                lestim = ((NbEst < MaxNbEst) && (ICDANA == 2) && (iprog > 0));

                if (lestim && isnear)
                {
                    //           (1.7) Optimization of ti by ACR.

                    StableSort(CurrentTi);

                    int Decima = 4;

                    CCurrent.Length(0.0, 1.0, out CBLONG);
                    J.EstLength() = CBLONG;

                    ACR(CCurrent, CurrentTi.ChangeArray1(), Decima);
                    lconst = true;
                }
            }

            //     (2) loop of parametric / geometric optimization

            Iter = 1;
            ToOptim = ((Iter < myNbIterations) && (isnear));

            while (ToOptim)
            {
                Iter++;
                //     (2.1) Save current results
                VOCRI[0] = VALCRI[0];
                VOCRI[1] = VALCRI[1];
                VOCRI[2] = VALCRI[2];
                EROLD = ERRMAX;
                LNOLD = CBLONG;
                COld = CCurrent;
                OldTi.ChangeArray1().Assign(CurrentTi.Array1());

                //     (2.2) Optimization of ti by ACR.

                StableSort(CurrentTi);

                int Decima = 4;

                CCurrent.Length(0.0, 1.0, out CBLONG);
                J.EstLength() = CBLONG;

                ACR(CCurrent, CurrentTi.ChangeArray1(), Decima);
                lconst = true;

                //      (2.3) Optimization of curves
                EpsLength = SmallValue * CBLONG / NbrPnt;

                CNew = new FEmTool_Curve(CCurrent.Dimension(), CCurrent.NbElements(), CCurrent.Base(), EpsLength);
                CNew.Knots().Assign(CCurrent.Knots());

                J.SetParameters(CurrentTi);

                EpsDeg = Min(WQuality * .1, CBLONG * .001);
                Optimization(J, TheAssembly, lconst, EpsDeg, CNew, CurrentTi.Array1());

                CCurrent = CNew;

                //      (2.4) calculation of quality criteria and improvement
                //             of estimation.

                ICDANA = J.QualityValues(J1min, J2min, J3min, out VALCRI[0], out VALCRI[1], out VALCRI[2]);
                if (ICDANA > 0)
                {
                    lconst = true;
                }

                J.GetEstimation(out e1, out e2, out e3);
                //       (2.5) Optimization of ti by orthogonal projection

                NewTi = new NCollection_HArray1<double>(1, CurrentTi.Length());
                Project(CCurrent, CurrentTi.Array1(), NewTi.ChangeArray1(), Ecarts, ref NumPnt, out ERRMAX, out ERRQUA,
                        out ERRMOY, 2);

                //       (2.6)  Test de non regression

                int iregre = 0;
                if (NbrConstraint < NbrPnt)
                {
                    if ((ERRMAX > WQuality) && (ERRMAX > 1.05 * EROLD))
                    {
                        iregre++;
                    }
                    if ((ERRMAX > WQuality) && (ERRMAX > 2 * EROLD))
                    {
                        iregre++;
                    }
                    if ((EROLD > WQuality) && (ERRMAX <= 0.5 * EROLD))
                    {
                        iregre--;
                    }
                }
                double E1, E2, E3;
                J.GetEstimation(out E1, out E2, out E3);
                if ((VALCRI[0] > E1) && (VALCRI[0] > 1.1 * VOCRI[0]))
                {
                    iregre++;
                }
                if ((VALCRI[1] > E2) && (VALCRI[1] > 1.1 * VOCRI[1]))
                {
                    iregre++;
                }
                if ((VALCRI[2] > E3) && (VALCRI[2] > 1.1 * VOCRI[2]))
                {
                    iregre++;
                }

                if (iregre >= 2)
                {
                    // (2.7) restore the previous iteration
                    VALCRI[0] = VOCRI[0];
                    VALCRI[1] = VOCRI[1];
                    VALCRI[2] = VOCRI[2];
                    ERRMAX = EROLD;
                    CBLONG = LNOLD;
                    CCurrent = COld;
                    CurrentTi.ChangeArray1().Assign(OldTi.Array1());
                    ToOptim = false;
                }
                else
                { // Iteration is Ok.
                    CCurrent = CNew;
                    CurrentTi = NewTi;
                }
                if (Iter >= myNbIterations)
                {
                    ToOptim = false;
                }
            }

            // (3) Optional splitting

            if ((CCurrent.NbElements() < myMaxSegment) && myWithCutting)
            {
                //    (3.1) Sauvgarde de l'etat precedent
                VOCRI[0] = VALCRI[0];
                VOCRI[1] = VALCRI[1];
                VOCRI[2] = VALCRI[2];
                EROLD = ERRMAX;
                COld = CCurrent;
                OldTi.ChangeArray1().Assign(CurrentTi.Array1());

                //       (3.2) Arrange the ti: Sort + rescale to (0,1)
                //         ---> Sort to ensure proper ordering afterwards.

                StableSort(CurrentTi);

                if ((CurrentTi.Value(1) != 0.0) || (CurrentTi.Value(NbrPnt) != 1.0))
                {
                    double t, DelatT = 1.0 / (CurrentTi.Value(NbrPnt) - CurrentTi.Value(1));
                    for (int ii = 2; ii < NbrPnt; ii++)
                    {
                        t = (CurrentTi.Value(ii) - CurrentTi.Value(1)) * DelatT;
                        CurrentTi.SetValue(ii, t);
                    }
                    CurrentTi.SetValue(1, 0.0);
                    CurrentTi.SetValue(NbrPnt, 1.0);
                }

                //       (3.3) Insert new Knots

                SplitCurve(CCurrent, CurrentTi.Array1(), EpsLength, ref CNew, out iscut);
                if (!iscut)
                {
                    again = false;
                }
                else
                {
                    CCurrent = CNew!;
                    // New Knots => New Assembly.
                    J.SetCurve(CNew!);
                    TheAssembly = new FEmTool_Assembly(Dependence.Array2(), J.AssemblyTable());
                }
            }
            else
            {
                again = false;
            }
        }

    //    ================   Great loop end   ===================

    L8000:

        // (4) Compute the best Error.
        NewTi = new NCollection_HArray1<double>(1, CurrentTi.Length());
        Project(CCurrent, CurrentTi.Array1(), NewTi.ChangeArray1(), Ecarts, ref NumPnt, out ERRMAX, out ERRQUA,
                out ERRMOY, 10);

        // (5) field's update

        TheCurve = CCurrent;
        J.EstLength() = CBLONG;
        myParameters.ChangeArray1().Assign(NewTi.Array1());
        myCriterium[0] = ERRQUA;
        myCriterium[1] = Math.Sqrt(VALCRI[0]);
        myCriterium[2] = Math.Sqrt(VALCRI[1]);
        myCriterium[3] = Math.Sqrt(VALCRI[2]);
        myMaxError = ERRMAX;
        myMaxErrorIndex = NumPnt;
        if (NbrPnt > NbrConstraint)
        {
            myAverageError = ERRMOY / (NbrPnt - NbrConstraint);
        }
        else
        {
            myAverageError = ERRMOY / NbrConstraint;
        }
    }

    private void Optimization(AppDef_SmoothCriterion J, FEmTool_Assembly A, bool ToAssemble, double EpsDeg,
                              FEmTool_Curve Curve, NCollection_Array1<double> Parameters)
    {
        int MxDeg = Curve.Base().WorkDegree(), NbElm = Curve.NbElements(), NbDim = Curve.Dimension();

        NCollection_HArray2<NCollection_HArray1<int>> AssTable;

        Matrix H = new(0, MxDeg, 0, MxDeg);
        math_Vector G = new(0, MxDeg), Sol = new(1, A.NbGlobVar());

        int el, dim;

        AssTable = A.AssemblyTable();
        int NbConstr = myNbPassPoints + myNbTangPoints + myNbCurvPoints;

        double CBLONG = J.EstLength();

        // Updating Assembly
        if (ToAssemble)
        {
            A.NullifyMatrix();
        }
        A.NullifyVector();

        for (el = 1; el <= NbElm; el++)
        {
            if (ToAssemble)
            {
                J.Hessian(el, 1, 1, H);
                for (dim = 1; dim <= NbDim; dim++)
                {
                    A.AddMatrix(el, dim, dim, H);
                }
            }

            for (dim = 1; dim <= NbDim; dim++)
            {
                J.Gradient(el, dim, G);
                A.AddVector(el, dim, G);
            }
        }

        // Solution of system
        if (ToAssemble)
        {
            if (NbConstr != 0)
            { // Treatment of constraints
                AssemblingConstraints(Curve, Parameters, CBLONG, A);
            }
            A.Solve();
        }
        A.Solution(Sol);

        // Updating J
        J.SetCurve(Curve);
        J.InputVector(Sol, AssTable);

        // Updating Curve and reduction of degree

        int Newdeg;
        double MaxError;

        if (NbConstr == 0)
        {
            for (el = 1; el <= NbElm; el++)
            {
                Curve.ReduceDegree(el, EpsDeg, out Newdeg, out MaxError);
            }
        }
        else
        {
            NCollection_Array1<double> TabInt = Curve.Knots();
            int Icnt = 1, p0 = Parameters.Lower() - myFirstPoint, point;
            for (el = 1; el <= NbElm; el++)
            {
                while ((Icnt < NbConstr) && (Parameters[p0 + myTypConstraints.Value(2 * Icnt - 1)] <= TabInt[el]))
                {
                    Icnt++;
                }
                point = p0 + myTypConstraints.Value(2 * Icnt - 1);
                if (Parameters[point] <= TabInt[el] || Parameters[point] >= TabInt[el + 1])
                {
                    Curve.ReduceDegree(el, EpsDeg, out Newdeg, out MaxError);
                }
                else if (Curve.Degree(el) < MxDeg)
                {
                    Curve.SetDegree(el, MxDeg);
                }
            }
        }
    }

    private void Project(FEmTool_Curve C, NCollection_Array1<double> Ti, NCollection_Array1<double> ProjTi,
                         NCollection_Array1<double> Distance, ref int NumPoints, out double MaxErr, out double QuaErr,
                         out double AveErr, int NbIterations = 2)
    {
        // Initialisation

        const double Seuil = 1.0e-9, Eps = 1.0e-12;

        MaxErr = QuaErr = AveErr = 0.0;

        int Ipnt, NItCv, Iter, i, i0 = -myDimension, d0 = Distance.Lower() - 1;

        double TNew, Dist, T0, Dist0, F1, F2, Aux, DF, Ecart;

        bool EnCour;

        NCollection_Array1<double> ValOfC = new(1, myDimension), FirstDerOfC = new(1, myDimension),
                                   SecndDerOfC = new(1, myDimension);

        for (Ipnt = 1; Ipnt <= ProjTi.Length(); Ipnt++)
        {
            i0 += myDimension;

            TNew = Ti[Ipnt];

            EnCour = true;
            NItCv = 0;
            Iter = 0;
            C.D0(TNew, ValOfC);

            Dist = 0;
            for (i = 1; i <= myDimension; i++)
            {
                Aux = ValOfC[i] - myTabPoints.Value(i0 + i);
                Dist += Aux * Aux;
            }
            Dist = Math.Sqrt(Dist);

            // ------- Newton's method for solving (C'(t),C(t) - P) = 0

            while (EnCour)
            {
                Iter++;
                T0 = TNew;
                Dist0 = Dist;

                C.D2(TNew, SecndDerOfC);
                C.D1(TNew, FirstDerOfC);

                F1 = F2 = 0.0;
                for (i = 1; i <= myDimension; i++)
                {
                    Aux = ValOfC[i] - myTabPoints.Value(i0 + i);
                    DF = FirstDerOfC[i];
                    F1 += Aux * DF;                       // (C'(t),C(t) - P)
                    F2 += DF * DF + Aux * SecndDerOfC[i]; // ((C'(t),C(t) - P))'
                }

                if (Math.Abs(F2) < Eps)
                {
                    EnCour = false;
                }
                else
                {
                    // Formula of Newton x(k+1) = x(k) - F(x(k))/F'(x(k))
                    TNew -= F1 / F2;
                    if (TNew < 0.0)
                    {
                        TNew = 0.0;
                    }
                    if (TNew > 1.0)
                    {
                        TNew = 1.0;
                    }

                    // Analysis of result

                    C.D0(TNew, ValOfC);

                    Dist = 0;
                    for (i = 1; i <= myDimension; i++)
                    {
                        Aux = ValOfC[i] - myTabPoints.Value(i0 + i);
                        Dist += Aux * Aux;
                    }
                    Dist = Math.Sqrt(Dist);

                    Ecart = Dist0 - Dist;

                    if (Ecart <= -Seuil)
                    {
                        // No improvement, stop here
                        EnCour = false;
                        TNew = T0;
                        Dist = Dist0;
                    }
                    else if (Ecart <= Seuil)
                    {
                        // Convergence
                        NItCv++;
                    }
                    else
                    {
                        NItCv = 0;
                    }

                    if ((NItCv >= 2) || (Iter >= NbIterations))
                    {
                        EnCour = false;
                    }
                }
            }

            ProjTi[Ipnt] = TNew;
            Distance[d0 + Ipnt] = Dist;
            if (Dist > MaxErr)
            {
                MaxErr = Dist;
                NumPoints = Ipnt;
            }
            QuaErr += Dist * Dist;
            AveErr += Dist;
        }

        // OCCT: when no distance is positive, NumPoints keeps the caller's value and is shifted again (occt-issues.md).
        NumPoints = NumPoints + myFirstPoint - 1; // Setting NumPoints to interval [myFirstPoint, myLastPoint]
    }

    private void ACR(FEmTool_Curve Curve, NCollection_Array1<double> Ti, int Decima)
    {
        const double Eps = 1.0e-8;

        NCollection_Array1<double> Knots = Curve.Knots();
        int NbrPnt = Ti.Length(), TiFirst = Ti.Lower(), TiLast = Ti.Upper(), KFirst = Knots.Lower(),
            KLast = Knots.Upper();

        double CbLong, DeltaT, VTest, UNew, UOld, DU, TPara, TOld, DTInv, Ratio;
        int ipnt, ii, IElm, IOld, POld, PCnt, ICnt = 0;
        int NbCntr = myNbPassPoints + myNbTangPoints + myNbCurvPoints;

        //     (1) Compute the curve length

        Curve.Length(Ti[TiFirst], Ti[TiLast], out CbLong);

        //     (2)  Place the arc-length parameterization in Ti

        if (NbrPnt >= 2)
        {
            //     (2.0) Initialisation
            DeltaT = (Ti[TiLast] - Ti[TiFirst]) / Decima;
            VTest = Ti[TiFirst] + DeltaT;

            if (NbCntr > 0)
            {
                PCnt = myTypConstraints.Value(1) - myFirstPoint + TiFirst;
                ICnt = 1;
            }
            else
            {
                PCnt = TiLast + 1;
            }

            UOld = 0.0;

            TOld = Ti[TiFirst];
            POld = TiFirst;

            IElm = KFirst;
            IOld = IElm;

            Ti[TiFirst] = 0.0;

            for (ipnt = TiFirst + 1; ipnt <= TiLast; ipnt++)
            {
                while ((ICnt <= NbCntr) && (PCnt < ipnt))
                {
                    ICnt++;
                    // OCCT reads the point of constraint NbCntr + 1, past the constraints (occt-issues.md); the port
                    // takes a point no ipnt can equal, as OCCT's value is in practice.
                    PCnt = ICnt <= NbCntr ? myTypConstraints.Value(2 * ICnt - 1) - myFirstPoint + TiFirst : TiLast + 1;
                }

                TPara = Ti[ipnt];

                if (TPara >= VTest || PCnt == ipnt)
                {
                    if (Ti[TiLast] - TPara <= 1.0e-2 * DeltaT)
                    {
                        ipnt = TiLast;
                        TPara = Ti[ipnt];
                    }
                    //        (2.2), (2.3) Compute the curve length
                    Curve.Length(Ti[TiFirst], TPara, out UNew);

                    UNew /= CbLong;

                    while (Knots[IElm + 1] < TPara && IElm < KLast - 1)
                    {
                        IElm++;
                    }

                    //         (2.4) Update the splitting parameters
                    DTInv = 1.0 / (TPara - TOld);
                    DU = UNew - UOld;

                    for (ii = IOld + 1; ii <= IElm; ii++)
                    {
                        Ratio = (Knots[ii] - TOld) * DTInv;
                        Knots[ii] = UOld + Ratio * DU;
                    }

                    //           (2.5) Update the point parameters.

                    // Very strange loop, because it never works (POld+1 > ipnt-1)
                    for (ii = POld + 1; ii <= ipnt - 1; ii++)
                    {
                        Ratio = (Ti[ii] - TOld) * DTInv;
                        Ti[ii] = UOld + Ratio * DU;
                    }

                    Ti[ipnt] = UNew;

                    UOld = UNew;
                    IOld = IElm;
                    TOld = TPara;
                    POld = ipnt;
                }
                //        --> New parametric threshold for decimation

                if (TPara >= VTest)
                {
                    VTest += Math.Ceiling((TPara - VTest + Eps) / DeltaT) * DeltaT;
                    if (VTest > 1.0 - Eps)
                    {
                        VTest = 1.0;
                    }
                }
            }
        }

        //     --- Adjust the extreme values

        Ti[TiFirst] = 0.0;
        Ti[TiLast] = 1.0;
        ii = TiLast - 1;
        while (Ti[ii] > Knots[KLast])
        {
            Ti[ii] = 1.0;
            --ii;
        }
        Knots[KFirst] = 0.0;
        Knots[KLast] = 1.0;
    }

    private static int NearIndex(double T, NCollection_Array1<double> TabPar, double Eps, out int Flag)
    {
        int Loi = TabPar.Lower(), Upi = TabPar.Upper();

        Flag = 0;

        if (T < TabPar[Loi])
        {
            Flag = -1;
            return Loi;
        }
        if (T > TabPar[Upi])
        {
            Flag = 1;
            return Upi;
        }

        int Ibeg = Loi, Ifin = Upi, Imidl;

        while (Ibeg + 1 != Ifin)
        {
            Imidl = (Ibeg + Ifin) / 2;
            if ((T >= TabPar[Ibeg]) && (T <= TabPar[Imidl]))
            {
                Ifin = Imidl;
            }
            else
            {
                Ibeg = Imidl;
            }
        }

        if (Math.Abs(T - TabPar[Ifin]) < Eps)
        {
            return Ifin;
        }

        return Ibeg;
    }

    // Calculates the new knots of the elements whose degree is Deg.
    private static void GettingKnots(NCollection_Array1<double> TabPar, FEmTool_Curve InCurve, int Deg, ref int NbElm,
                                     NCollection_Array1<double> NewKnots)
    {
        const double Eps = 1.0e-12;

        NCollection_Array1<double> OldKnots = InCurve.Knots();
        int NbMaxOld = InCurve.NbElements();
        int NbMax = NewKnots.Upper(), Ipt, Ipt1, Ipt2;
        int el = 0, i1 = OldKnots.Lower(), i0 = i1 - 1, Flag;
        double TPar;

        while ((NbElm < NbMax) && (el < NbMaxOld))
        {
            el++;
            i0++;
            i1++; // i0, i1 are indexes of left and right knots of element el

            if (InCurve.Degree(el) == Deg)
            {
                NbElm++;

                Ipt1 = NearIndex(OldKnots[i0], TabPar, Eps, out Flag);
                if (Flag != 0)
                {
                    Ipt1 = TabPar.Lower();
                }
                Ipt2 = NearIndex(OldKnots[i1], TabPar, Eps, out Flag);
                if (Flag != 0)
                {
                    Ipt2 = TabPar.Upper();
                }

                if (Ipt2 - Ipt1 >= 1)
                {
                    Ipt = (Ipt1 + Ipt2) / 2;
                    if (2 * Ipt == Ipt1 + Ipt2)
                    {
                        TPar = 2.0 * TabPar[Ipt];
                    }
                    else
                    {
                        TPar = TabPar[Ipt] + TabPar[Ipt + 1];
                    }

                    NewKnots[NbElm] = (OldKnots[i0] + OldKnots[i1] + TPar) / 4.0;
                }
                else
                {
                    NewKnots[NbElm] = (OldKnots[i0] + OldKnots[i1]) / 2.0;
                }
            }
        }
    }

    private void SplitCurve(FEmTool_Curve InCurve, NCollection_Array1<double> Ti, double CurveTol,
                            ref FEmTool_Curve? OutCurve, out bool iscut)
    {
        int NbElmOld = InCurve.NbElements();

        if (NbElmOld >= myMaxSegment)
        {
            iscut = false;
            return;
        }
        int NbElm = NbElmOld;
        NCollection_Array1<double> NewKnots = new(NbElm + 1, myMaxSegment);
        GettingKnots(Ti, InCurve, InCurve.Base().WorkDegree(), ref NbElm, NewKnots);
        GettingKnots(Ti, InCurve, InCurve.Base().WorkDegree() - 1, ref NbElm, NewKnots);
        if (NbElm > NbElmOld)
        {
            iscut = true;

            OutCurve = new FEmTool_Curve(InCurve.Dimension(), NbElm, InCurve.Base(), CurveTol);
            NCollection_Array1<double> OutKnots = OutCurve.Knots();
            NCollection_Array1<double> InKnots = InCurve.Knots();

            int i, i0 = OutKnots.Lower();
            for (i = InKnots.Lower(); i <= InKnots.Upper(); i++)
            {
                OutKnots[i] = InKnots[i];
            }
            for (i = NbElmOld + 1; i <= NbElm; i++)
            {
                OutKnots[i + i0] = NewKnots[i];
            }

            // C++: std::sort. The values are knots, distinct or equal, so a stable sort gives the same array.
            StableSort(OutKnots);
        }
        else
        {
            iscut = false;
        }
    }

    private void InitSmoothCriterion()
    {
        const double Eps2 = 1.0e-6, Eps3 = 1.0e-9;

        double Length;

        InitParameters(out Length);

        mySmoothCriterion.SetParameters(myParameters);

        double E1, E2, E3;

        InitCriterionEstimations(Length, out E1, out E2, out E3);
        mySmoothCriterion.EstLength() = Length;
        mySmoothCriterion.SetEstimation(E1, E2, E3);

        double WQuadratic, WQuality;

        if (!myWithMinMax && myTolerance != 0.0)
        {
            WQuality = myTolerance;
        }
        else if (myTolerance == 0.0)
        {
            WQuality = 1.0;
        }
        else
        {
            WQuality = Max(myTolerance, Eps2 * Length);
        }

        int NbConstr = myNbPassPoints + myNbTangPoints + myNbCurvPoints;
        WQuadratic = Math.Sqrt((double)(myNbPoints - NbConstr)) * WQuality;
        if (WQuadratic > Eps3)
        {
            WQuadratic = 1.0 / WQuadratic;
        }

        if (WQuadratic == 0.0)
        {
            WQuadratic = Max(Math.Sqrt(E1), 1.0);
        }

        mySmoothCriterion.SetWeight(WQuadratic, WQuality, myPercent[0], myPercent[1], myPercent[2]);

        HermitJacobi TheBase = new(myMaxDegree, myContinuity);
        FEmTool_Curve TheCurve;
        int NbElem;
        double CurvTol = Eps2 * Length / myNbPoints;

        // Subdivision of the interval according to constraints
        if (myWithCutting && NbConstr != 0)
        {
            InitCutting(TheBase, CurvTol, out TheCurve);
        }
        else
        {
            NbElem = 1;
            TheCurve = new FEmTool_Curve(myDimension, NbElem, TheBase, CurvTol);
            TheCurve.Knots().SetValue(TheCurve.Knots().Lower(), myParameters.Value(myFirstPoint));
            TheCurve.Knots().SetValue(TheCurve.Knots().Upper(), myParameters.Value(myLastPoint));
        }

        mySmoothCriterion.SetCurve(TheCurve);
    }

    private void InitParameters(out double Length)
    {
        const double Eps1 = 1.0e-7 * .01; // Precision::Confusion() * .01

        double aux, dist;
        int i, i0, i1 = 0, ipoint;

        Length = 0.0;
        myParameters.SetValue(myFirstPoint, Length);

        for (ipoint = myFirstPoint + 1; ipoint <= myLastPoint; ipoint++)
        {
            i0 = i1;
            i1 += myDimension;
            dist = 0;
            for (i = 1; i <= myDimension; i++)
            {
                aux = myTabPoints.Value(i1 + i) - myTabPoints.Value(i0 + i);
                dist += aux * aux;
            }
            Length += Math.Sqrt(dist);
            myParameters.SetValue(ipoint, Length);
        }

        if (Length <= Eps1)
        {
            throw new ConstructionErrorException("AppDef_Variational::InitParameters");
        }

        for (ipoint = myFirstPoint + 1; ipoint <= myLastPoint - 1; ipoint++)
        {
            myParameters.SetValue(ipoint, myParameters.Value(ipoint) / Length);
        }

        myParameters.SetValue(myLastPoint, 1.0);

        // With few points there is likely underestimation ...
        if (myNbPoints < 10)
        {
            Length *= (1.0 + 0.1 / (myNbPoints - 1));
        }
    }

    private void InitCriterionEstimations(double Length, out double E1, out double E2, out double E3)
    {
        E1 = Length * Length;

        const double Eps1 = 1.0e-7 * .01; // Precision::Confusion() * .01

        math_Vector VTang1 = new(1, myDimension), VTang2 = new(1, myDimension), VTang3 = new(1, myDimension),
                    VScnd1 = new(1, myDimension), VScnd2 = new(1, myDimension), VScnd3 = new(1, myDimension);

        // ========== Treatment of first point =================

        int ipnt = myFirstPoint;

        EstTangent(ipnt, VTang1);
        ipnt++;
        EstTangent(ipnt, VTang2);
        ipnt++;
        EstTangent(ipnt, VTang3);

        ipnt = myFirstPoint;
        EstSecnd(ipnt, VTang1, VTang2, Length, VScnd1);
        ipnt++;
        EstSecnd(ipnt, VTang1, VTang3, Length, VScnd2);

        //  Modified by skv - Fri Apr  8 14:58:12 2005 OCC8559 Begin
        int anInd = ipnt;
        double Delta = .5 * (myParameters.Value(anInd) - myParameters.Value(--ipnt));
        //  Modified by skv - Fri Apr  8 14:58:12 2005 OCC8559 End

        if (Delta <= Eps1)
        {
            Delta = 1.0;
        }

        E2 = VScnd1.Norm2() * Delta;

        E3 = (Delta > Eps1) ? VScnd2.Subtracted(VScnd1).Norm2() / (4.0 * Delta) : 0.0;
        // ========== Treatment of internal points =================

        int CurrPoint = 2;

        for (ipnt = myFirstPoint + 1; ipnt < myLastPoint; ipnt++)
        {
            Delta = .5 * (myParameters.Value(ipnt + 1) - myParameters.Value(ipnt - 1));

            if (CurrPoint == 1)
            {
                if (ipnt + 1 != myLastPoint)
                {
                    EstTangent(ipnt + 2, VTang3);
                    EstSecnd(ipnt + 1, VTang1, VTang3, Length, VScnd2);
                }
                else
                {
                    EstSecnd(ipnt + 1, VTang1, VTang2, Length, VScnd2);
                }

                E2 += VScnd1.Norm2() * Delta;
                E3 += (Delta > Eps1) ? VScnd2.Subtracted(VScnd3).Norm2() / (4.0 * Delta) : 0.0;
            }
            else if (CurrPoint == 2)
            {
                if (ipnt + 1 != myLastPoint)
                {
                    EstTangent(ipnt + 2, VTang1);
                    EstSecnd(ipnt + 1, VTang2, VTang1, Length, VScnd3);
                }
                else
                {
                    EstSecnd(ipnt + 1, VTang2, VTang3, Length, VScnd3);
                }

                E2 += VScnd2.Norm2() * Delta;
                E3 += (Delta > Eps1) ? VScnd3.Subtracted(VScnd1).Norm2() / (4.0 * Delta) : 0.0;
            }
            else
            {
                if (ipnt + 1 != myLastPoint)
                {
                    EstTangent(ipnt + 2, VTang2);
                    EstSecnd(ipnt + 1, VTang3, VTang2, Length, VScnd1);
                }
                else
                {
                    EstSecnd(ipnt + 1, VTang3, VTang1, Length, VScnd1);
                }

                E2 += VScnd3.Norm2() * Delta;
                E3 += (Delta > Eps1) ? VScnd1.Subtracted(VScnd2).Norm2() / (4.0 * Delta) : 0.0;
            }

            CurrPoint++;
            if (CurrPoint == 4)
            {
                CurrPoint = 1;
            }
        }

        // ========== Treatment of last point =================

        Delta = .5 * (myParameters.Value(myLastPoint) - myParameters.Value(myLastPoint - 1));
        if (Delta <= Eps1)
        {
            Delta = 1.0;
        }

        double aux;

        if (CurrPoint == 1)
        {
            E2 += VScnd1.Norm2() * Delta;
            aux = VScnd1.Subtracted(VScnd3).Norm2();
            E3 += (Delta > Eps1) ? aux / (4.0 * Delta) : aux;
        }
        else if (CurrPoint == 2)
        {
            E2 += VScnd2.Norm2() * Delta;
            aux = VScnd2.Subtracted(VScnd1).Norm2();
            E3 += (Delta > Eps1) ? aux / (4.0 * Delta) : aux;
        }
        else
        {
            E2 += VScnd3.Norm2() * Delta;
            aux = VScnd3.Subtracted(VScnd2).Norm2();
            E3 += (Delta > Eps1) ? aux / (4.0 * Delta) : aux;
        }

        aux = Length * Length;

        E2 *= aux;
        E3 *= aux;
    }

    private void EstTangent(int ipnt, math_Vector VTang)
    {
        int i;
        const double Eps1 = 1.0e-7 * .01; // Precision::Confusion() * .01
        const double EpsNorm = 1.0e-9;

        double Wpnt = 1.0;

        if (ipnt == myFirstPoint)
        {
            // Estimation at first point
            if (myNbPoints < 3)
            {
                Wpnt = 0.0;
            }
            else
            {
                int adr1 = 1, adr2 = adr1 + myDimension, adr3 = adr2 + myDimension;

                math_Vector Pnt1 = new(myTabPoints, adr1, 1, myDimension);
                math_Vector Pnt2 = new(myTabPoints, adr2, 1, myDimension);
                math_Vector Pnt3 = new(myTabPoints, adr3, 1, myDimension);

                // Parabolic interpolation
                // if we have parabolic interpolation: F(t) = A0 + A1*t + A2*t*t,
                // first derivative for t=0 is A1 = ((d2-1)*P1 + P2 - d2*P3)/(d*(1-d))
                //       d= |P2-P1|/(|P2-P1|+|P3-P2|), d2 = d*d
                double V1 = Pnt2.Subtracted(Pnt1).Norm();
                double V2 = 0.0;
                if (V1 > Eps1)
                {
                    V2 = Pnt3.Subtracted(Pnt2).Norm();
                }
                if (V2 > Eps1)
                {
                    double d = V1 / (V1 + V2), d1;
                    d1 = 1.0 / (d * (1 - d));
                    d *= d;
                    VTang.Initialized(((d - 1.0) * Pnt1 + Pnt2 - d * Pnt3) * d1);
                }
                else
                {
                    // Simple 2-point estimation

                    VTang.Initialized(Pnt2 - Pnt1);
                }
            }
        }
        else if (ipnt == myLastPoint)
        {
            // Estimation at last point
            if (myNbPoints < 3)
            {
                Wpnt = 0.0;
            }
            else
            {
                // OCCT: the addresses assume myFirstPoint == 1 (occt-issues.md).
                int adr1 = (myLastPoint - 3) * myDimension + 1, adr2 = adr1 + myDimension, adr3 = adr2 + myDimension;

                math_Vector Pnt1 = new(myTabPoints, adr1, 1, myDimension);
                math_Vector Pnt2 = new(myTabPoints, adr2, 1, myDimension);
                math_Vector Pnt3 = new(myTabPoints, adr3, 1, myDimension);

                // Parabolic interpolation
                // if we have parabolic interpolation: F(t) = A0 + A1*t + A2*t*t,
                // first derivative for t=1 is 2*A2 + A1 = ((d2+1)*P1 - P2 - d2*P3)/(d*(1-d))
                //       d= |P2-P1|/(|P2-P1|+|P3-P2|), d2 = d*(d-2)
                double V1 = Pnt2.Subtracted(Pnt1).Norm();
                double V2 = 0.0;
                if (V1 > Eps1)
                {
                    V2 = Pnt3.Subtracted(Pnt2).Norm();
                }
                if (V2 > Eps1)
                {
                    double d = V1 / (V1 + V2), d1;
                    d1 = 1.0 / (d * (1 - d));
                    d *= d - 2;
                    VTang.Initialized(((d + 1.0) * Pnt1 - Pnt2 - d * Pnt3) * d1);
                }
                else
                {
                    // Simple 2-point estimation

                    VTang.Initialized(Pnt3 - Pnt2);
                }
            }
        }
        else
        {
            int adr1 = (ipnt - myFirstPoint - 1) * myDimension + 1, adr2 = adr1 + 2 * myDimension;

            math_Vector Pnt1 = new(myTabPoints, adr1, 1, myDimension);
            math_Vector Pnt2 = new(myTabPoints, adr2, 1, myDimension);

            VTang.Initialized(Pnt2 - Pnt1);
        }

        double Vnorm = VTang.Norm();

        if (Vnorm <= EpsNorm)
        {
            VTang.Init(0.0);
        }
        else
        {
            VTang /= Vnorm;
        }

        // Estimation with constraints

        double Wcnt = 0.0;
        int IdCnt = 1;

        // Warning!! Here it is suppoused that all points are in range [myFirstPoint, myLastPoint]

        int NbConstr = myNbPassPoints + myNbTangPoints + myNbCurvPoints;
        math_Vector VCnt = new(1, myDimension, 0.0);

        if (NbConstr > 0)
        {
            // OCCT tests the point before the count, so it reads the point of constraint NbConstr + 1 past the
            // constraints (occt-issues.md); the port tests the count first.
            while (IdCnt <= NbConstr && myTypConstraints.Value(2 * IdCnt - 1) < ipnt)
            {
                IdCnt++;
            }
            if (IdCnt <= NbConstr && (myTypConstraints.Value(2 * IdCnt - 1) == ipnt)
                && (myTypConstraints.Value(2 * IdCnt) >= 1))
            {
                Wcnt = 1.0;
                int i0 = 2 * myDimension * (IdCnt - 1), k = 0;
                for (i = 1; i <= myNbP3d; i++)
                {
                    for (int j = 1; j <= 3; j++)
                    {
                        VCnt[++k] = myTabConstraints.Value(++i0);
                    }
                    i0 += 3;
                }
                for (i = 1; i <= myNbP2d; i++)
                {
                    for (int j = 1; j <= 2; j++)
                    {
                        VCnt[++k] = myTabConstraints.Value(++i0);
                    }
                    i0 += 2;
                }
            }
        }

        // Averaging of estimation

        double Denom = Wpnt + Wcnt;
        if (Denom == 0.0)
        {
            Denom = 1.0;
        }
        else
        {
            Denom = 1.0 / Denom;
        }

        VTang.Initialized((Wpnt * VTang + Wcnt * VCnt) * Denom);

        Vnorm = VTang.Norm();

        if (Vnorm <= EpsNorm)
        {
            VTang.Init(0.0);
        }
        else
        {
            VTang /= Vnorm;
        }
    }

    private void EstSecnd(int ipnt, math_Vector VTang1, math_Vector VTang2, double Length, math_Vector VScnd)
    {
        int i;

        const double Eps = 1.0e-9;

        double Wpnt = 1.0;

        double aux;

        if (ipnt == myFirstPoint)
        {
            aux = myParameters.Value(ipnt + 1) - myParameters.Value(ipnt);
        }
        else if (ipnt == myLastPoint)
        {
            aux = myParameters.Value(ipnt) - myParameters.Value(ipnt - 1);
        }
        else
        {
            aux = myParameters.Value(ipnt + 1) - myParameters.Value(ipnt - 1);
        }

        if (aux <= Eps)
        {
            aux = 1.0;
        }
        else
        {
            aux = 1.0 / aux;
        }

        VScnd.Initialized((VTang2 - VTang1) * aux);

        // Estimation with constraints

        double Wcnt = 0.0;
        int IdCnt = 1;

        // Warning!! Here it is suppoused that all points are in range [myFirstPoint, myLastPoint]

        int NbConstr = myNbPassPoints + myNbTangPoints + myNbCurvPoints;
        math_Vector VCnt = new(1, myDimension, 0.0);

        if (NbConstr > 0)
        {
            // OCCT tests the point before the count (occt-issues.md); the port tests the count first.
            while (IdCnt <= NbConstr && myTypConstraints.Value(2 * IdCnt - 1) < ipnt)
            {
                IdCnt++;
            }

            if (IdCnt <= NbConstr && (myTypConstraints.Value(2 * IdCnt - 1) == ipnt)
                && (myTypConstraints.Value(2 * IdCnt) >= 2))
            {
                Wcnt = 1.0;
                int i0 = 2 * myDimension * (IdCnt - 1) + 3, k = 0;
                for (i = 1; i <= myNbP3d; i++)
                {
                    for (int j = 1; j <= 3; j++)
                    {
                        VCnt[++k] = myTabConstraints.Value(++i0);
                    }
                    i0 += 3;
                }
                i0--;
                for (i = 1; i <= myNbP2d; i++)
                {
                    for (int j = 1; j <= 2; j++)
                    {
                        VCnt[++k] = myTabConstraints.Value(++i0);
                    }
                    i0 += 2;
                }
            }
        }

        // Averaging of estimation

        double Denom = Wpnt + Wcnt;
        if (Denom == 0.0)
        {
            Denom = 1.0;
        }
        else
        {
            Denom = 1.0 / Denom;
        }

        VScnd.Initialized((Wpnt * VScnd + (Wcnt * Length) * VCnt) * Denom);
    }

    private void InitCutting(HermitJacobi aBase, double CurvTol, out FEmTool_Curve aCurve)
    {
        // Definition of number of elements
        int ORCMx = -1, NCont = 0, i, kk, NbElem;
        int NbConstr = myNbPassPoints + myNbTangPoints + myNbCurvPoints;

        for (i = 1; i <= NbConstr; i++)
        {
            kk = Math.Abs(myTypConstraints.Value(2 * i)) + 1;
            ORCMx = Max(ORCMx, kk);
            NCont += kk;
        }

        if (ORCMx > myMaxDegree - myNivCont)
        {
            throw new ConstructionErrorException("AppDef_Variational::InitCutting");
        }

        int NLibre = Max(myMaxDegree - myNivCont - (myMaxDegree + 1) / 4, myNivCont + 1);

        NbElem = (NCont % NLibre == 0) ? NCont / NLibre : NCont / NLibre + 1;

        while ((NbElem > myMaxSegment) && (NLibre < myMaxDegree - myNivCont))
        {
            NLibre++;
            NbElem = (NCont % NLibre == 0) ? NCont / NLibre : NCont / NLibre + 1;
        }

        if (NbElem > myMaxSegment)
        {
            throw new ConstructionErrorException("AppDef_Variational::InitCutting");
        }

        aCurve = new FEmTool_Curve(myDimension, NbElem, aBase, CurvTol);

        int NCnt = (NCont - 1) / NbElem + 1;
        int NPlus = NbElem - (NCnt * NbElem - NCont);

        NCollection_Array1<double> Knot = aCurve.Knots();

        int IDeb = 0, IFin = NbConstr + 1, NDeb = 0, NFin = 0, IndEl = Knot.Lower(), IUpper = Knot.Upper(),
            NbEl = 0;

        Knot[IndEl] = myParameters.Value(myFirstPoint);
        Knot[IUpper] = myParameters.Value(myLastPoint);

        while (NbElem - NbEl > 1)
        {
            IndEl++;
            NbEl++;
            if (NPlus == 0)
            {
                NCnt--;
            }

            while (NDeb < NCnt && IDeb < IFin)
            {
                IDeb++;
                NDeb += Math.Abs(myTypConstraints.Value(2 * IDeb)) + 1;
            }

            if (NDeb == NCnt)
            {
                NDeb = 0;
                if (NPlus == 1 && myParameters.Value(myTypConstraints.Value(2 * IDeb - 1)) > Knot[IndEl - 1])
                {
                    Knot[IndEl] = myParameters.Value(myTypConstraints.Value(2 * IDeb - 1));
                }
                else
                {
                    Knot[IndEl] = (myParameters.Value(myTypConstraints.Value(2 * IDeb - 1))
                                   + myParameters.Value(myTypConstraints.Value(2 * IDeb + 1)))
                                  / 2;
                }
            }
            else
            {
                NDeb -= NCnt;
                Knot[IndEl] = myParameters.Value(myTypConstraints.Value(2 * IDeb - 1));
            }

            NPlus--;
            if (NPlus == 0)
            {
                NCnt--;
            }

            if (NbElem - NbEl == 1)
            {
                break;
            }

            NbEl++;

            while (NFin < NCnt && IDeb < IFin)
            {
                IFin--;
                NFin += Math.Abs(myTypConstraints.Value(2 * IFin)) + 1;
            }

            if (NFin == NCnt)
            {
                NFin = 0;
                Knot[IUpper + 1 - IndEl] = (myParameters.Value(myTypConstraints.Value(2 * IFin - 1))
                                            + myParameters.Value(myTypConstraints.Value(2 * IFin - 3)))
                                           / 2;
            }
            else
            {
                NFin -= NCnt;
                if (myParameters.Value(myTypConstraints.Value(2 * IFin - 1)) < Knot[IUpper - IndEl + 1])
                {
                    Knot[IUpper + 1 - IndEl] = myParameters.Value(myTypConstraints.Value(2 * IFin - 1));
                }
                else
                {
                    Knot[IUpper + 1 - IndEl] = (myParameters.Value(myTypConstraints.Value(2 * IFin - 1))
                                                + myParameters.Value(myTypConstraints.Value(2 * IFin - 3)))
                                               / 2;
                }
            }
        }
    }

    private void Adjusting(ref AppDef_SmoothCriterion J, ref double WQuadratic, ref double WQuality,
                           ref FEmTool_Curve TheCurve, NCollection_Array1<double> Ecarts)
    {
        /* Initialized data */

        const int mxiter = 2;
        const double eps1 = 1e-6;
        int NbrPnt = myLastPoint - myFirstPoint + 1;
        int NbrConstraint = myNbPassPoints + myNbTangPoints + myNbCurvPoints;
        double CurvTol = eps1 * J.EstLength() / NbrPnt;

        /* Local variables */
        int iter, ipnt;
        double ecart, emold, erold, tpara;
        double j1cibl, vtest, vseuil;
        double[] vocri = new double[4];
        int i, numint, flag;
        NCollection_Array1<double> tbpoid = new(myFirstPoint, myLastPoint);
        bool loptim, lrejet;
        AppDef_SmoothCriterion JNew;
        FEmTool_Curve CNew;
        double E1, E2, E3;

        /* (0.b) Initialisations */

        loptim = true;
        iter = 0;
        tbpoid.Init(1.0);

        /* ============   loop on the smoothing engine  ============== */

        vtest = WQuality * .9;
        j1cibl = Math.Sqrt(myCriterium[0] / (NbrPnt - NbrConstraint));

        while (loptim)
        {
            ++iter;

            /*     (1) Save the previous state */

            vocri[0] = myCriterium[0];
            vocri[1] = myCriterium[1];
            vocri[2] = myCriterium[2];
            vocri[3] = myCriterium[3];
            erold = myMaxError;
            emold = myAverageError;

            /*     (2) Increase the least squares weight */

            if (j1cibl > vtest)
            {
                WQuadratic = j1cibl / vtest * WQuadratic;
            }

            /*     (3) Increase the weight associated with problematic points */

            vseuil = WQuality * .88;

            for (ipnt = myFirstPoint; ipnt <= myLastPoint; ++ipnt)
            {
                if (Ecarts[ipnt] > vtest)
                {
                    ecart = (Ecarts[ipnt] - vseuil) / WQuality;
                    tbpoid[ipnt] = (ecart * 3 + 1.0) * tbpoid[ipnt];
                }
            }

            /*     (4) Forced splitting */

            if (TheCurve.NbElements() < myMaxSegment && myWithCutting)
            {
                numint = NearIndex(myParameters.Value(myMaxErrorIndex), TheCurve.Knots(), 0, out flag);

                tpara = (TheCurve.Knots()[numint] + TheCurve.Knots()[numint + 1]
                         + myParameters.Value(myMaxErrorIndex) * 2)
                        / 4;

                CNew = new FEmTool_Curve(myDimension, TheCurve.NbElements() + 1, TheCurve.Base(), CurvTol);

                for (i = 1; i <= numint; i++)
                {
                    CNew.Knots()[i] = TheCurve.Knots()[i];
                }
                for (i = numint + 1; i <= TheCurve.Knots().Length(); i++)
                {
                    CNew.Knots()[i + 1] = TheCurve.Knots()[i];
                }

                CNew.Knots()[numint + 1] = tpara;
            }
            else
            {
                CNew = new FEmTool_Curve(myDimension, TheCurve.NbElements(), TheCurve.Base(), CurvTol);

                CNew.Knots().Assign(TheCurve.Knots());
            }

            JNew = new AppDef_LinearCriteria(mySSP, myFirstPoint, myLastPoint);

            JNew.EstLength() = J.EstLength();

            J.GetEstimation(out E1, out E2, out E3);

            JNew.SetEstimation(E1, E2, E3);

            JNew.SetParameters(myParameters);

            JNew.SetWeight(WQuadratic, WQuality, myPercent[0], myPercent[1], myPercent[2]);

            JNew.SetWeight(tbpoid);

            JNew.SetCurve(CNew);

            /*     (5) Relissage */

            TheMotor(JNew, WQuadratic, WQuality, ref CNew, Ecarts);

            /*     (6) Tests de rejet */

            j1cibl = Math.Sqrt(myCriterium[0] / (NbrPnt - NbrConstraint));
            vseuil = Math.Sqrt(vocri[1]) + (erold - myMaxError) * 4;

            lrejet = ((myMaxError > WQuality) && (myMaxError > erold * 1.01))
                     || (Math.Sqrt(myCriterium[1]) > vseuil * 1.05);

            if (lrejet)
            {
                myCriterium[0] = vocri[0];
                myCriterium[1] = vocri[1];
                myCriterium[2] = vocri[2];
                myCriterium[3] = vocri[3];
                myMaxError = erold;
                myAverageError = emold;

                loptim = false;
            }
            else
            {
                J = JNew;
                TheCurve = CNew;
                J.SetCurve(TheCurve);
            }

            /*     (7) Test de convergence */

            if (((iter >= mxiter) && (myMaxSegment == CNew.NbElements())) || myMaxError < WQuality)
            {
                loptim = false;
            }
        }
    }

    private static bool NotParallel(in Vector3D T, out Vector3D V)
    {
        V = T;
        V.SetX(V.X() + 1.0);
        if (V.CrossMagnitude(T) > 1.0e-12)
        {
            return true;
        }
        V.SetY(V.Y() + 1.0);
        if (V.CrossMagnitude(T) > 1.0e-12)
        {
            return true;
        }
        V.SetZ(V.Z() + 1.0);
        return V.CrossMagnitude(T) > 1.0e-12;
    }

    private void AssemblingConstraints(FEmTool_Curve Curve, NCollection_Array1<double> Parameters, double CBLONG,
                                       FEmTool_Assembly A)
    {
        int MxDeg = Curve.Base().WorkDegree(), NbElm = Curve.NbElements(), NbDim = Curve.Dimension();

        NCollection_Array1<double> G0 = new(0, MxDeg), G1 = new(0, MxDeg), G2 = new(0, MxDeg);
        math_Vector V0 = new(G0, 0, 0, MxDeg), V1 = new(G1, 0, 0, MxDeg), V2 = new(G2, 0, 0, MxDeg);

        int IndexOfConstraint, Ng3d, Ng2d, NBeg2d, NPass, NgPC1, NTang3d, NTang2d, Point, TypOfConstr,
            p0 = Parameters.Lower() - myFirstPoint, curel = 1, el, i, ipnt, ityp, j, k, pnt, curdim, jt,
            Ntheta = 6 * myNbP3d + 2 * myNbP2d;
        int NbConstr = myNbPassPoints + myNbTangPoints + myNbCurvPoints;

        //  Ng3d = 3 * NbConstr + 2 * myNbTangPoints + 5 * myNbCurvPoints;
        //  Ng2d = 2 * NbConstr + 1 * myNbTangPoints + 3 * myNbCurvPoints;
        Ng3d = 3 * NbConstr + 3 * myNbTangPoints + 5 * myNbCurvPoints;
        Ng2d = 2 * NbConstr + 2 * myNbTangPoints + 3 * myNbCurvPoints;
        NBeg2d = Ng3d * myNbP3d;
        //  NgPC1 = NbConstr + myNbCurvPoints;
        NgPC1 = NbConstr + myNbTangPoints + myNbCurvPoints;
        NPass = 0;
        NTang3d = 3 * NgPC1;
        NTang2d = 2 * NgPC1;

        NCollection_Array1<double> Intervals = Curve.Knots();

        double t, R1, R2;

        HermitJacobi myHermitJacobi = Curve.Base();
        int Order = myHermitJacobi.NivConstr() + 1;

        double UFirst, ULast, coeff, c0, mfact, mfact1;

        A.NullifyConstraint();

        ipnt = -1;
        ityp = 0;
        for (i = 1; i <= NbConstr; i++)
        {
            ipnt += 2;
            ityp += 2;

            Point = myTypConstraints.Value(ipnt);
            TypOfConstr = myTypConstraints.Value(ityp);

            t = Parameters[p0 + Point];

            for (el = curel; el <= NbElm;)
            {
                if (t <= Intervals[++el])
                {
                    curel = el - 1;
                    break;
                }
            }

            UFirst = Intervals[curel];
            ULast = Intervals[curel + 1];
            coeff = (ULast - UFirst) / 2.0;
            c0 = (ULast + UFirst) / 2.0;

            t = (t - c0) / coeff;

            if (TypOfConstr == 0)
            {
                myHermitJacobi.D0(t, G0);
                for (k = 1; k < Order; k++)
                {
                    mfact = Math.Pow(coeff, k);
                    G0[k] *= mfact;
                    G0[k + Order] *= mfact;
                }
            }
            else if (TypOfConstr == 1)
            {
                myHermitJacobi.D1(t, G0, G1);
                for (k = 1; k < Order; k++)
                {
                    mfact = Math.Pow(coeff, k);
                    G0[k] *= mfact;
                    G0[k + Order] *= mfact;
                    G1[k] *= mfact;
                    G1[k + Order] *= mfact;
                }
                mfact = 1.0 / coeff;
                for (k = 0; k <= MxDeg; k++)
                {
                    G1[k] *= mfact;
                }
            }
            else
            {
                myHermitJacobi.D2(t, G0, G1, G2);
                for (k = 1; k < Order; k++)
                {
                    mfact = Math.Pow(coeff, k);
                    G0[k] *= mfact;
                    G0[k + Order] *= mfact;
                    G1[k] *= mfact;
                    G1[k + Order] *= mfact;
                    G2[k] *= mfact;
                    G2[k + Order] *= mfact;
                }
                mfact = 1.0 / coeff;
                mfact1 = mfact / coeff;
                for (k = 0; k <= MxDeg; k++)
                {
                    G1[k] *= mfact;
                    G2[k] *= mfact1;
                }
            }

            NPass++;

            j = NbDim * (Point - myFirstPoint);
            int n0 = NPass;
            curdim = 0;
            for (pnt = 1; pnt <= myNbP3d; pnt++)
            {
                IndexOfConstraint = n0;
                for (k = 1; k <= 3; k++)
                {
                    curdim++;
                    A.AddConstraint(IndexOfConstraint, curel, curdim, V0, myTabPoints.Value(j + k));
                    IndexOfConstraint += NgPC1;
                }
                j += 3;
                n0 += Ng3d;
            }

            n0 = NPass + NBeg2d;
            for (pnt = 1; pnt <= myNbP2d; pnt++)
            {
                IndexOfConstraint = n0;
                for (k = 1; k <= 2; k++)
                {
                    curdim++;
                    A.AddConstraint(IndexOfConstraint, curel, curdim, V0, myTabPoints.Value(j + k));
                    IndexOfConstraint += NgPC1;
                }
                j += 2;
                n0 += Ng2d;
            }

            if (TypOfConstr == 1)
            {
                NPass++;
                n0 = NPass;
                j = 2 * NbDim * (i - 1);
                curdim = 0;
                for (pnt = 1; pnt <= myNbP3d; pnt++)
                {
                    IndexOfConstraint = n0;
                    for (k = 1; k <= 3; k++)
                    {
                        curdim++;
                        A.AddConstraint(IndexOfConstraint, curel, curdim, V1, CBLONG * myTabConstraints.Value(j + k));
                        IndexOfConstraint += NgPC1;
                    }
                    n0 += Ng3d;
                    j += 6;
                }

                n0 = NPass + NBeg2d;
                for (pnt = 1; pnt <= myNbP2d; pnt++)
                {
                    IndexOfConstraint = n0;
                    for (k = 1; k <= 2; k++)
                    {
                        curdim++;
                        A.AddConstraint(IndexOfConstraint, curel, curdim, V1, CBLONG * myTabConstraints.Value(j + k));
                        IndexOfConstraint += NgPC1;
                    }
                    n0 += Ng2d;
                    j += 4;
                }
            }
            if (TypOfConstr == 2)
            {
                NPass++;
                n0 = NPass;
                j = 2 * NbDim * (i - 1);
                curdim = 0;
                for (pnt = 1; pnt <= myNbP3d; pnt++)
                {
                    IndexOfConstraint = n0;
                    for (k = 1; k <= 3; k++)
                    {
                        curdim++;
                        A.AddConstraint(IndexOfConstraint, curel, curdim, V1, CBLONG * myTabConstraints.Value(j + k));
                        IndexOfConstraint += NgPC1;
                    }
                    n0 += Ng3d;
                    j += 6;
                }

                n0 = NPass + NBeg2d;
                for (pnt = 1; pnt <= myNbP2d; pnt++)
                {
                    IndexOfConstraint = n0;
                    for (k = 1; k <= 2; k++)
                    {
                        curdim++;
                        A.AddConstraint(IndexOfConstraint, curel, curdim, V1, CBLONG * myTabConstraints.Value(j + k));
                        IndexOfConstraint += NgPC1;
                    }
                    n0 += Ng2d;
                    j += 4;
                }

                j = 2 * NbDim * (i - 1) + 3;
                jt = Ntheta * (i - 1);
                IndexOfConstraint = NTang3d + 1;
                curdim = 0;
                for (pnt = 1; pnt <= myNbP3d; pnt++)
                {
                    R1 = 0.0;
                    R2 = 0.0;
                    for (k = 1; k <= 3; k++)
                    {
                        R1 += myTabConstraints.Value(j + k) * myTtheta.Value(jt + k);
                        R2 += myTabConstraints.Value(j + k) * myTtheta.Value(jt + 3 + k);
                    }
                    R1 *= CBLONG * CBLONG;
                    R2 *= CBLONG * CBLONG;
                    for (k = 1; k <= 3; k++)
                    {
                        curdim++;
                        if (k > 1)
                        {
                            R1 = R2 = 0.0;
                        }
                        A.AddConstraint(IndexOfConstraint, curel, curdim, myTfthet.Value(jt + k) * V2, R1);
                        A.AddConstraint(IndexOfConstraint + 1, curel, curdim, myTfthet.Value(jt + 3 + k) * V2, R2);
                    }
                    IndexOfConstraint += Ng3d;
                    j += 6;
                    jt += 6;
                }

                j--;
                IndexOfConstraint = NBeg2d + NTang2d + 1;
                for (pnt = 1; pnt <= myNbP2d; pnt++)
                {
                    R1 = 0.0;
                    for (k = 1; k <= 2; k++)
                    {
                        R1 += myTabConstraints.Value(j + k) * myTtheta.Value(jt + k);
                    }
                    R1 *= CBLONG * CBLONG;
                    for (k = 1; k <= 2; k++)
                    {
                        curdim++;
                        if (k > 1)
                        {
                            R1 = 0.0;
                        }
                        A.AddConstraint(IndexOfConstraint, curel, curdim, myTfthet.Value(jt + k) * V2, R1);
                    }
                    IndexOfConstraint += Ng2d;
                    j += 4;
                    jt += 2;
                }

                NTang3d += 2;
                NTang2d += 1;
            }
        }
    }

    private bool InitTthetaF(int ndimen, AppParCurves_Constraint typcon, int begin, int jndex)
    {
        if ((ndimen < 2) || (ndimen > 3))
        {
            return false;
        }
        Vector3D T = new(), V = new();
        Vector3D theta1, theta2 = new();
        Vector3D F = new();
        double XX, XY, YY, XZ, YZ, ZZ;

        if ((typcon == AppParCurves_TangencyPoint) || (typcon == AppParCurves_CurvaturePoint))
        {
            T.SetX(myTabConstraints.Value(jndex));
            T.SetY(myTabConstraints.Value(jndex + 1));
            if (ndimen == 3)
            {
                T.SetZ(myTabConstraints.Value(jndex + 2));
            }
            else
            {
                T.SetZ(0.0);
            }
            if (ndimen == 2)
            {
                V.SetX(0.0);
                V.SetY(0.0);
                V.SetZ(1.0);
            }
            if (ndimen == 3)
            {
                if (!NotParallel(T, out V))
                {
                    return false;
                }
            }
            theta1 = V ^ T;
            theta1.Normalize();
            myTtheta.SetValue(begin, theta1.X());
            myTtheta.SetValue(begin + 1, theta1.Y());
            if (ndimen == 3)
            {
                theta2 = T ^ theta1;
                theta2.Normalize();
                myTtheta.SetValue(begin + 2, theta1.Z());
                myTtheta.SetValue(begin + 3, theta2.X());
                myTtheta.SetValue(begin + 4, theta2.Y());
                myTtheta.SetValue(begin + 5, theta2.Z());
            }

            // Calculation of myTfthet
            if (typcon == AppParCurves_CurvaturePoint)
            {
                XX = Math.Pow(T.X(), 2);
                XY = T.X() * T.Y();
                YY = Math.Pow(T.Y(), 2);
                if (ndimen == 2)
                {
                    F.SetX(YY * theta1.X() - XY * theta1.Y());
                    F.SetY(XX * theta1.Y() - XY * theta1.X());
                    myTfthet.SetValue(begin, F.X());
                    myTfthet.SetValue(begin + 1, F.Y());
                }
                if (ndimen == 3)
                {
                    XZ = T.X() * T.Z();
                    YZ = T.Y() * T.Z();
                    ZZ = Math.Pow(T.Z(), 2);

                    F.SetX((ZZ + YY) * theta1.X() - XY * theta1.Y() - XZ * theta1.Z());
                    F.SetY((XX + ZZ) * theta1.Y() - XY * theta1.X() - YZ * theta1.Z());
                    F.SetZ((XX + YY) * theta1.Z() - XZ * theta1.X() - YZ * theta1.Y());
                    myTfthet.SetValue(begin, F.X());
                    myTfthet.SetValue(begin + 1, F.Y());
                    myTfthet.SetValue(begin + 2, F.Z());
                    F.SetX((ZZ + YY) * theta2.X() - XY * theta2.Y() - XZ * theta2.Z());
                    F.SetY((XX + ZZ) * theta2.Y() - XY * theta2.X() - YZ * theta2.Z());
                    F.SetZ((XX + YY) * theta2.Z() - XZ * theta2.X() - YZ * theta2.Y());
                    myTfthet.SetValue(begin + 3, F.X());
                    myTfthet.SetValue(begin + 4, F.Y());
                    myTfthet.SetValue(begin + 5, F.Z());
                }
            }
        }
        return true;
    }

    // std::stable_sort with operator<, for the parameter and knot arrays (insertion sort: they are nearly sorted).
    private static void StableSort(NCollection_Array1<double> theArray)
    {
        Span<double> a = theArray.AsSpan();
        for (int i = 1; i < a.Length; i++)
        {
            double v = a[i];
            int j = i - 1;
            while (j >= 0 && v < a[j])
            {
                a[j + 1] = a[j];
                j--;
            }
            a[j + 1] = v;
        }
    }
}
