// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_QuasiUniformDeflection.hxx
// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_QuasiUniformDeflection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GCPnts_DeflectionType;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// Computes points on a curve such that the deflection between the curve and the polygon through the points stays
/// below a given value, by recursive subdivision. Quicker than GCPnts_UniformDeflection and works on curves that
/// are not C2, but gives more points. 2D curves give points with Z = 0.
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>Initialize</c> recomputes. The templates over the curve type are ported per
/// instantiation (decision 187).
/// </remarks>
public sealed class GCPnts_QuasiUniformDeflection
{
    private const int MyMaxQuasiFleshe = 2000;

    private bool myDone;
    private double myDeflection;
    private readonly NCollection_Sequence<double> myParams = new();
    private readonly NCollection_Sequence<gp_Pnt> myPoints = new();
    private GeomAbs_Shape myCont;

    /// <summary>An empty algorithm; use <c>Initialize</c>.</summary>
    public GCPnts_QuasiUniformDeflection()
    {
        myDone = false;
        myDeflection = 0.0;
        myCont = GeomAbs_C1;
    }

    /// <summary>
    /// Points on the part theU1..theU2 of the curve with the deflection theDeflection. With theContinuity above G1
    /// the deflection is estimated from the derivatives too.
    /// </summary>
    public GCPnts_QuasiUniformDeflection(Adaptor3d_Curve theC, double theDeflection, double theU1, double theU2,
        GeomAbs_Shape theContinuity = GeomAbs_C1)
    {
        myDone = false;
        myDeflection = theDeflection;
        myCont = GeomAbs_C1;
        Initialize(theC, theDeflection, theU1, theU2, theContinuity);
    }

    /// <summary>Points on the part theU1..theU2 of the curve with the deflection theDeflection.</summary>
    public GCPnts_QuasiUniformDeflection(Adaptor2d_Curve2d theC, double theDeflection, double theU1, double theU2,
        GeomAbs_Shape theContinuity = GeomAbs_C1)
    {
        myDone = false;
        myDeflection = theDeflection;
        myCont = GeomAbs_C1;
        Initialize(theC, theDeflection, theU1, theU2, theContinuity);
    }

    /// <summary>Points on the curve with the deflection theDeflection.</summary>
    public GCPnts_QuasiUniformDeflection(Adaptor3d_Curve theC, double theDeflection,
        GeomAbs_Shape theContinuity = GeomAbs_C1)
    {
        myDone = false;
        myDeflection = theDeflection;
        myCont = GeomAbs_C1;
        Initialize(theC, theDeflection, theContinuity);
    }

    /// <summary>Points on the curve with the deflection theDeflection.</summary>
    public GCPnts_QuasiUniformDeflection(Adaptor2d_Curve2d theC, double theDeflection,
        GeomAbs_Shape theContinuity = GeomAbs_C1)
    {
        myDone = false;
        myDeflection = theDeflection;
        myCont = GeomAbs_C1;
        Initialize(theC, theDeflection, theContinuity);
    }

    /// <summary>Recomputes the points on the curve.</summary>
    public void Initialize(Adaptor3d_Curve theC, double theDeflection, GeomAbs_Shape theContinuity = GeomAbs_C1)
    {
        Initialize(theC, theDeflection, theC.FirstParameter(), theC.LastParameter(), theContinuity);
    }

    /// <summary>Recomputes the points on the curve.</summary>
    public void Initialize(Adaptor2d_Curve2d theC, double theDeflection, GeomAbs_Shape theContinuity = GeomAbs_C1)
    {
        Initialize(theC, theDeflection, theC.FirstParameter(), theC.LastParameter(), theContinuity);
    }

    /// <summary>
    /// Recomputes the points on the part theU1..theU2 of the curve (the bounds are swapped if theU1 &gt; theU2).
    /// </summary>
    public void Initialize(Adaptor3d_Curve theC, double theDeflection, double theU1, double theU2,
        GeomAbs_Shape theContinuity = GeomAbs_C1)
    {
        initialize(theC, theDeflection, theU1, theU2, theContinuity);
    }

    /// <summary>
    /// Recomputes the points on the part theU1..theU2 of the curve (the bounds are swapped if theU1 &gt; theU2).
    /// </summary>
    public void Initialize(Adaptor2d_Curve2d theC, double theDeflection, double theU1, double theU2,
        GeomAbs_Shape theContinuity = GeomAbs_C1)
    {
        initialize(theC, theDeflection, theU1, theU2, theContinuity);
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone() => myDone;

    /// <summary>The number of points computed.</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    public int NbPoints()
    {
        StdFail_NotDone_Raise_if(!myDone, "GCPnts_QuasiUniformDeflection::NbPoints()");
        return myParams.Length();
    }

    /// <summary>The parameter of the point of index Index (1 to NbPoints).</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    public double Parameter(int Index)
    {
        StdFail_NotDone_Raise_if(!myDone, "GCPnts_QuasiUniformDeflection::Parameter()");
        return myParams[Index];
    }

    /// <summary>The point of index theIndex (1 to NbPoints).</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    public gp_Pnt Value(int theIndex)
    {
        // OCCT names the wrong class and method in the message (occt-issues.md).
        StdFail_NotDone_Raise_if(!myDone, "GCPnts_QuasiUniformAbscissa::Parameter()");
        return myPoints.Value(theIndex);
    }

    /// <summary>The deflection given to the algorithm.</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    public double Deflection()
    {
        StdFail_NotDone_Raise_if(!myDone, "GCPnts_QuasiUniformDeflection::Deflection()");
        return myDeflection;
    }

    // mask the return of a Adaptor2d_Curve2d as a gp_Pnt
    private static gp_Pnt Value(Adaptor3d_Curve theC, double theParameter)
    {
        return theC.Value(theParameter);
    }

    private static gp_Pnt Value(Adaptor2d_Curve2d theC, double theParameter)
    {
        gp_Pnt aPoint = new();
        gp_Pnt2d a2dPoint = theC.Value(theParameter);
        aPoint.SetCoord(a2dPoint.X(), a2dPoint.Y(), 0.0);
        return aPoint;
    }

    private static void D1(Adaptor3d_Curve theC, double theParameter, out gp_Pnt theP, out gp_Vec theV)
    {
        theC.D1(theParameter, out theP, out theV);
    }

    private static void D1(Adaptor2d_Curve2d theC, double theParameter, out gp_Pnt theP, out gp_Vec theV)
    {
        theC.D1(theParameter, out gp_Pnt2d a2dPoint, out gp_Vec2d a2dVec);
        theP = new gp_Pnt();
        theV = new gp_Vec();
        theP.SetCoord(a2dPoint.X(), a2dPoint.Y(), 0.0);
        theV.SetCoord(a2dVec.X(), a2dVec.Y(), 0.0);
    }

    // ---- Templates over the curve type (3D instantiation) ----

    private static void QuasiFleche(Adaptor3d_Curve theC,
                                    double theDeflection2,
                                    double theUdeb,
                                    in gp_Pnt thePdeb,
                                    in gp_Vec theVdeb,
                                    double theUfin,
                                    in gp_Pnt thePfin,
                                    in gp_Vec theVfin,
                                    int theNbmin,
                                    double theEps,
                                    NCollection_Sequence<double> theParameters,
                                    NCollection_Sequence<gp_Pnt> thePoints,
                                    ref int theNbCalls)
    {
        ++theNbCalls;
        if (theNbCalls >= MyMaxQuasiFleshe)
        {
            return;
        }
        int aPtslength = thePoints.Length();
        if (theNbCalls > 100 && aPtslength < 2)
        {
            return;
        }

        double aUdelta = theUfin - theUdeb;
        gp_Pnt aPdelta;
        gp_Vec aVdelta;
        if (theNbmin > 2)
        {
            aUdelta /= (theNbmin - 1);
            D1(theC, theUdeb + aUdelta, out aPdelta, out aVdelta);
        }
        else
        {
            aPdelta = thePfin;
            aVdelta = theVfin;
        }

        // square length of chord
        double aNorme = new gp_Vec(thePdeb, aPdelta).SquareMagnitude();
        double aFleche = 0.0;
        bool isFlecheOk = false;
        if (aNorme > theEps && aNorme > 16.0 * theDeflection2)
        {
            // Evaluation of the deflection by interpolation. See IntWalk_IWalking::TestDeflection
            double N1 = theVdeb.SquareMagnitude();
            double N2 = aVdelta.SquareMagnitude();
            if (N1 > theEps && N2 > theEps)
            {
                // square distance between ends of two normalized vectors [0; 4]
                double aNormediff = (theVdeb.Normalized().XYZ() - aVdelta.Normalized().XYZ()).SquareModulus();
                if (aNormediff > theEps)
                {
                    aFleche = aNormediff * aNorme / 64.0;
                    // So, fleche <= (aNorme / 16), independently of Vdeb and Vdelta.
                    // And if (aNorme / 16) < theDeflection2, this approach gives
                    // fleche < theDeflection2 independently of real curve.
                    // That is why we exclude case aNorme < (16. * theDeflection2)
                    isFlecheOk = true;
                }
            }
        }

        gp_Pnt aPmid = new((thePdeb.XYZ() + aPdelta.XYZ()) * 0.5);
        gp_Pnt aPverif = Value(theC, theUdeb + aUdelta * 0.5);
        double aFlecheMidMid = aPmid.SquareDistance(aPverif);

        if (isFlecheOk)
        {
            // Algorithm, evaluating "fleche" by interpolation,
            // can give false-positive result.
            // So we check also distance between Pmid and Pverif (aFlecheMidMid).
            // But aFlecheMidMid gives worse result in case of non-uniform parameterisation.
            // Maximum aFlecheMidMid, that seems reasonable, is (chord/2)^2 + theDeflection2
            //   .---------------.Pverif  .
            //   |               |        | Deflection
            //   ._______. ______.        .
            // Pdeb    Pmid    Pdelta
            if (aFlecheMidMid > aNorme / 4.0 + theDeflection2)
            // if (aFlecheMidMid > aNorme/4.)
            {
                aFleche = aFlecheMidMid;
            }
        }
        else
        {
            aFleche = aFlecheMidMid;
        }

        if (aFleche < theDeflection2)
        {
            theParameters.Append(theUdeb + aUdelta);
            thePoints.Append(aPdelta);
        }
        else
        {
            QuasiFleche(theC,
                        theDeflection2,
                        theUdeb,
                        thePdeb,
                        theVdeb,
                        theUdeb + aUdelta,
                        aPdelta,
                        aVdelta,
                        3,
                        theEps,
                        theParameters,
                        thePoints,
                        ref theNbCalls);
        }

        if (theNbmin > 2)
        {
            QuasiFleche(theC,
                        theDeflection2,
                        theUdeb + aUdelta,
                        aPdelta,
                        aVdelta,
                        theUfin,
                        thePfin,
                        theVfin,
                        theNbmin - (thePoints.Length() - aPtslength),
                        theEps,
                        theParameters,
                        thePoints,
                        ref theNbCalls);
        }
        --theNbCalls;
    }

    private static void QuasiFleche(Adaptor3d_Curve theC,
                                    double theDeflection2,
                                    double theUdeb,
                                    in gp_Pnt thePdeb,
                                    double theUfin,
                                    in gp_Pnt thePfin,
                                    int theNbmin,
                                    NCollection_Sequence<double> theParameters,
                                    NCollection_Sequence<gp_Pnt> thePoints,
                                    ref int theNbCalls)
    {
        ++theNbCalls;
        if (theNbCalls >= MyMaxQuasiFleshe)
        {
            return;
        }
        int aPtslength = thePoints.Length();
        if (theNbCalls > 100 && aPtslength < 2)
        {
            return;
        }

        double aUdelta = theUfin - theUdeb;
        gp_Pnt aPdelta;
        if (theNbmin > 2)
        {
            aUdelta /= (theNbmin - 1);
            aPdelta = Value(theC, theUdeb + aUdelta);
        }
        else
        {
            aPdelta = thePfin;
        }

        gp_Pnt aPmid = new((thePdeb.XYZ() + aPdelta.XYZ()) * 0.5);
        gp_Pnt aPverif = Value(theC, theUdeb + aUdelta * 0.5);
        double aFleche = aPmid.SquareDistance(aPverif);
        if (aFleche < theDeflection2)
        {
            theParameters.Append(theUdeb + aUdelta);
            thePoints.Append(aPdelta);
        }
        else
        {
            QuasiFleche(theC,
                        theDeflection2,
                        theUdeb,
                        thePdeb,
                        theUdeb + aUdelta * 0.5,
                        aPverif,
                        2,
                        theParameters,
                        thePoints,
                        ref theNbCalls);

            QuasiFleche(theC,
                        theDeflection2,
                        theUdeb + aUdelta * 0.5,
                        aPverif,
                        theUdeb + aUdelta,
                        aPdelta,
                        2,
                        theParameters,
                        thePoints,
                        ref theNbCalls);
        }

        if (theNbmin > 2)
        {
            QuasiFleche(theC,
                        theDeflection2,
                        theUdeb + aUdelta,
                        aPdelta,
                        theUfin,
                        thePfin,
                        theNbmin - (thePoints.Length() - aPtslength),
                        theParameters,
                        thePoints,
                        ref theNbCalls);
        }
        --theNbCalls;
    }

    private static bool PerformLinear(Adaptor3d_Curve theC,
                                      NCollection_Sequence<double> theParameters,
                                      NCollection_Sequence<gp_Pnt> thePoints,
                                      double theU1,
                                      double theU2)
    {
        theParameters.Append(theU1);
        gp_Pnt aPoint = Value(theC, theU1);
        thePoints.Append(aPoint);

        theParameters.Append(theU2);
        aPoint = Value(theC, theU2);
        thePoints.Append(aPoint);
        return true;
    }

    private static bool PerformCircular(Adaptor3d_Curve theC,
                                        NCollection_Sequence<double> theParameters,
                                        NCollection_Sequence<gp_Pnt> thePoints,
                                        double theDeflection,
                                        double theU1,
                                        double theU2)
    {
        double anAngle = Max(1.0 - (theDeflection / theC.Circle().Radius()), 0.0);
        anAngle = 2.0 * Math.Acos(anAngle);
        int aNbPoints = (int)((theU2 - theU1) / anAngle);
        aNbPoints += 2;
        anAngle = (theU2 - theU1) / (double)(aNbPoints - 1);
        double U = theU1;
        for (int i = 1; i <= aNbPoints; ++i)
        {
            theParameters.Append(U);
            gp_Pnt aPoint = Value(theC, U);
            thePoints.Append(aPoint);
            U += anAngle;
        }
        return true;
    }

    private static GCPnts_DeflectionType GetDefType(Adaptor3d_Curve theC)
    {
        if (theC.NbIntervals(GeomAbs_C1) > 1)
        {
            return GCPnts_DefComposite;
        }
        // to force subdivisions at discontinuities.
        // G1 should work, but gives exceptions...

        switch (theC.GetType())
        {
            case GeomAbs_Line:
                return GCPnts_Linear;
            case GeomAbs_Circle:
                return GCPnts_Circular;
            case GeomAbs_BSplineCurve:
            {
                Geom_BSplineCurve aBS = theC.BSpline();
                return (aBS.NbPoles() == 2) ? GCPnts_Linear : GCPnts_Curved;
            }
            case GeomAbs_BezierCurve:
            {
                Geom_BezierCurve aBZ = theC.Bezier();
                return (aBZ.NbPoles() == 2) ? GCPnts_Linear : GCPnts_Curved;
            }
            default:
                return GCPnts_Curved;
        }
    }

    private static bool PerformCurve(NCollection_Sequence<double> theParameters,
                                     NCollection_Sequence<gp_Pnt> thePoints,
                                     Adaptor3d_Curve theC,
                                     double theDeflection,
                                     double theU1,
                                     double theU2,
                                     double theEPSILON,
                                     GeomAbs_Shape theContinuity)
    {
        int aNbmin = 2;
        int aNbCallQF = 0;

        gp_Pnt aPdeb;
        if (theContinuity <= GeomAbs_G1)
        {
            aPdeb = Value(theC, theU1);
            theParameters.Append(theU1);
            thePoints.Append(aPdeb);

            gp_Pnt aPfin = Value(theC, theU2);
            QuasiFleche(theC,
                        theDeflection * theDeflection,
                        theU1,
                        aPdeb,
                        theU2,
                        aPfin,
                        aNbmin,
                        theParameters,
                        thePoints,
                        ref aNbCallQF);
        }
        else
        {
            gp_Pnt aPfin;
            gp_Vec aDdeb, aDfin;
            D1(theC, theU1, out aPdeb, out aDdeb);
            theParameters.Append(theU1);
            thePoints.Append(aPdeb);

            double aDecreasedU2 = theU2 - Epsilon(theU2) * 10.0;
            D1(theC, aDecreasedU2, out aPfin, out aDfin);
            QuasiFleche(theC,
                        theDeflection * theDeflection,
                        theU1,
                        aPdeb,
                        aDdeb,
                        theU2,
                        aPfin,
                        aDfin,
                        aNbmin,
                        theEPSILON * theEPSILON,
                        theParameters,
                        thePoints,
                        ref aNbCallQF);
        }
        //  cout << "Nb of pts: " << Points.Length()<< endl;
        return true;
    }

    private static bool PerformComposite(NCollection_Sequence<double> theParameters,
                                         NCollection_Sequence<gp_Pnt> thePoints,
                                         Adaptor3d_Curve theC,
                                         double theDeflection,
                                         double theU1,
                                         double theU2,
                                         double theEPSILON,
                                         GeomAbs_Shape theContinuity)
    {
        // Consistency with Intervals
        int aNbIntervals = theC.NbIntervals(GeomAbs_C2);
        int aPIndex;
        NCollection_Array1<double> aTI = new(1, aNbIntervals + 1);
        theC.Intervals(aTI, GeomAbs_C2);
        BSplCLib.Hunt(aTI, theU1, out aPIndex);

        // iterate by continuous segments
        double aUa = theU1;
        for (int anIndex = aPIndex; ;)
        {
            double aUb = anIndex + 1 <= aTI.Upper() ? Min(theU2, aTI[anIndex + 1]) : theU2;
            if (!PerformCurve(theParameters,
                              thePoints,
                              theC,
                              theDeflection,
                              aUa,
                              aUb,
                              theEPSILON,
                              theContinuity))
            {
                return false;
            }

            ++anIndex;
            if (anIndex > aNbIntervals || theU2 < aTI[anIndex])
            {
                return true;
            }

            // remove last point to avoid duplication
            theParameters.Remove(theParameters.Length());
            thePoints.Remove(thePoints.Length());

            aUa = aUb;
        }
    }

    private void initialize(Adaptor3d_Curve theC, double theDeflection, double theU1, double theU2,
        GeomAbs_Shape theContinuity)
    {
        myCont = (theContinuity > GeomAbs_G1) ? GeomAbs_C1 : GeomAbs_C0;
        myDeflection = theDeflection;
        myDone = false;
        myParams.Clear();
        myPoints.Clear();

        double anEPSILON = Min(theC.Resolution(Precision.Confusion()), 1.0e50);
        GCPnts_DeflectionType aType = GetDefType(theC);
        double aU1 = Min(theU1, theU2);
        double aU2 = Max(theU1, theU2);

        if (aType == GCPnts_Curved || aType == GCPnts_DefComposite)
        {
            if (theC.GetType() == GeomAbs_BSplineCurve || theC.GetType() == GeomAbs_BezierCurve)
            {
                double aMaxPar = Max(Math.Abs(theC.FirstParameter()), Math.Abs(theC.LastParameter()));
                if (anEPSILON < Epsilon(aMaxPar))
                {
                    return;
                }
            }
        }

        switch (aType)
        {
            case GCPnts_Linear:
            {
                myDone = PerformLinear(theC, myParams, myPoints, aU1, aU2);
                break;
            }
            case GCPnts_Circular:
            {
                myDone = PerformCircular(theC, myParams, myPoints, theDeflection, aU1, aU2);
                break;
            }
            case GCPnts_Curved:
            {
                myDone = PerformCurve(myParams, myPoints, theC, theDeflection, aU1, aU2, anEPSILON, myCont);
                break;
            }
            case GCPnts_DefComposite:
            {
                myDone = PerformComposite(myParams, myPoints, theC, theDeflection, aU1, aU2, anEPSILON, myCont);
                break;
            }
        }
    }

    // ---- Templates over the curve type (2D instantiation) ----

    private static void QuasiFleche(Adaptor2d_Curve2d theC,
                                    double theDeflection2,
                                    double theUdeb,
                                    in gp_Pnt thePdeb,
                                    in gp_Vec theVdeb,
                                    double theUfin,
                                    in gp_Pnt thePfin,
                                    in gp_Vec theVfin,
                                    int theNbmin,
                                    double theEps,
                                    NCollection_Sequence<double> theParameters,
                                    NCollection_Sequence<gp_Pnt> thePoints,
                                    ref int theNbCalls)
    {
        ++theNbCalls;
        if (theNbCalls >= MyMaxQuasiFleshe)
        {
            return;
        }
        int aPtslength = thePoints.Length();
        if (theNbCalls > 100 && aPtslength < 2)
        {
            return;
        }

        double aUdelta = theUfin - theUdeb;
        gp_Pnt aPdelta;
        gp_Vec aVdelta;
        if (theNbmin > 2)
        {
            aUdelta /= (theNbmin - 1);
            D1(theC, theUdeb + aUdelta, out aPdelta, out aVdelta);
        }
        else
        {
            aPdelta = thePfin;
            aVdelta = theVfin;
        }

        // square length of chord
        double aNorme = new gp_Vec(thePdeb, aPdelta).SquareMagnitude();
        double aFleche = 0.0;
        bool isFlecheOk = false;
        if (aNorme > theEps && aNorme > 16.0 * theDeflection2)
        {
            // Evaluation of the deflection by interpolation. See IntWalk_IWalking::TestDeflection
            double N1 = theVdeb.SquareMagnitude();
            double N2 = aVdelta.SquareMagnitude();
            if (N1 > theEps && N2 > theEps)
            {
                // square distance between ends of two normalized vectors [0; 4]
                double aNormediff = (theVdeb.Normalized().XYZ() - aVdelta.Normalized().XYZ()).SquareModulus();
                if (aNormediff > theEps)
                {
                    aFleche = aNormediff * aNorme / 64.0;
                    // So, fleche <= (aNorme / 16), independently of Vdeb and Vdelta.
                    // And if (aNorme / 16) < theDeflection2, this approach gives
                    // fleche < theDeflection2 independently of real curve.
                    // That is why we exclude case aNorme < (16. * theDeflection2)
                    isFlecheOk = true;
                }
            }
        }

        gp_Pnt aPmid = new((thePdeb.XYZ() + aPdelta.XYZ()) * 0.5);
        gp_Pnt aPverif = Value(theC, theUdeb + aUdelta * 0.5);
        double aFlecheMidMid = aPmid.SquareDistance(aPverif);

        if (isFlecheOk)
        {
            // Algorithm, evaluating "fleche" by interpolation,
            // can give false-positive result.
            // So we check also distance between Pmid and Pverif (aFlecheMidMid).
            // But aFlecheMidMid gives worse result in case of non-uniform parameterisation.
            // Maximum aFlecheMidMid, that seems reasonable, is (chord/2)^2 + theDeflection2
            //   .---------------.Pverif  .
            //   |               |        | Deflection
            //   ._______. ______.        .
            // Pdeb    Pmid    Pdelta
            if (aFlecheMidMid > aNorme / 4.0 + theDeflection2)
            // if (aFlecheMidMid > aNorme/4.)
            {
                aFleche = aFlecheMidMid;
            }
        }
        else
        {
            aFleche = aFlecheMidMid;
        }

        if (aFleche < theDeflection2)
        {
            theParameters.Append(theUdeb + aUdelta);
            thePoints.Append(aPdelta);
        }
        else
        {
            QuasiFleche(theC,
                        theDeflection2,
                        theUdeb,
                        thePdeb,
                        theVdeb,
                        theUdeb + aUdelta,
                        aPdelta,
                        aVdelta,
                        3,
                        theEps,
                        theParameters,
                        thePoints,
                        ref theNbCalls);
        }

        if (theNbmin > 2)
        {
            QuasiFleche(theC,
                        theDeflection2,
                        theUdeb + aUdelta,
                        aPdelta,
                        aVdelta,
                        theUfin,
                        thePfin,
                        theVfin,
                        theNbmin - (thePoints.Length() - aPtslength),
                        theEps,
                        theParameters,
                        thePoints,
                        ref theNbCalls);
        }
        --theNbCalls;
    }

    private static void QuasiFleche(Adaptor2d_Curve2d theC,
                                    double theDeflection2,
                                    double theUdeb,
                                    in gp_Pnt thePdeb,
                                    double theUfin,
                                    in gp_Pnt thePfin,
                                    int theNbmin,
                                    NCollection_Sequence<double> theParameters,
                                    NCollection_Sequence<gp_Pnt> thePoints,
                                    ref int theNbCalls)
    {
        ++theNbCalls;
        if (theNbCalls >= MyMaxQuasiFleshe)
        {
            return;
        }
        int aPtslength = thePoints.Length();
        if (theNbCalls > 100 && aPtslength < 2)
        {
            return;
        }

        double aUdelta = theUfin - theUdeb;
        gp_Pnt aPdelta;
        if (theNbmin > 2)
        {
            aUdelta /= (theNbmin - 1);
            aPdelta = Value(theC, theUdeb + aUdelta);
        }
        else
        {
            aPdelta = thePfin;
        }

        gp_Pnt aPmid = new((thePdeb.XYZ() + aPdelta.XYZ()) * 0.5);
        gp_Pnt aPverif = Value(theC, theUdeb + aUdelta * 0.5);
        double aFleche = aPmid.SquareDistance(aPverif);
        if (aFleche < theDeflection2)
        {
            theParameters.Append(theUdeb + aUdelta);
            thePoints.Append(aPdelta);
        }
        else
        {
            QuasiFleche(theC,
                        theDeflection2,
                        theUdeb,
                        thePdeb,
                        theUdeb + aUdelta * 0.5,
                        aPverif,
                        2,
                        theParameters,
                        thePoints,
                        ref theNbCalls);

            QuasiFleche(theC,
                        theDeflection2,
                        theUdeb + aUdelta * 0.5,
                        aPverif,
                        theUdeb + aUdelta,
                        aPdelta,
                        2,
                        theParameters,
                        thePoints,
                        ref theNbCalls);
        }

        if (theNbmin > 2)
        {
            QuasiFleche(theC,
                        theDeflection2,
                        theUdeb + aUdelta,
                        aPdelta,
                        theUfin,
                        thePfin,
                        theNbmin - (thePoints.Length() - aPtslength),
                        theParameters,
                        thePoints,
                        ref theNbCalls);
        }
        --theNbCalls;
    }

    private static bool PerformLinear(Adaptor2d_Curve2d theC,
                                      NCollection_Sequence<double> theParameters,
                                      NCollection_Sequence<gp_Pnt> thePoints,
                                      double theU1,
                                      double theU2)
    {
        theParameters.Append(theU1);
        gp_Pnt aPoint = Value(theC, theU1);
        thePoints.Append(aPoint);

        theParameters.Append(theU2);
        aPoint = Value(theC, theU2);
        thePoints.Append(aPoint);
        return true;
    }

    private static bool PerformCircular(Adaptor2d_Curve2d theC,
                                        NCollection_Sequence<double> theParameters,
                                        NCollection_Sequence<gp_Pnt> thePoints,
                                        double theDeflection,
                                        double theU1,
                                        double theU2)
    {
        double anAngle = Max(1.0 - (theDeflection / theC.Circle().Radius()), 0.0);
        anAngle = 2.0 * Math.Acos(anAngle);
        int aNbPoints = (int)((theU2 - theU1) / anAngle);
        aNbPoints += 2;
        anAngle = (theU2 - theU1) / (double)(aNbPoints - 1);
        double U = theU1;
        for (int i = 1; i <= aNbPoints; ++i)
        {
            theParameters.Append(U);
            gp_Pnt aPoint = Value(theC, U);
            thePoints.Append(aPoint);
            U += anAngle;
        }
        return true;
    }

    private static GCPnts_DeflectionType GetDefType(Adaptor2d_Curve2d theC)
    {
        if (theC.NbIntervals(GeomAbs_C1) > 1)
        {
            return GCPnts_DefComposite;
        }
        // to force subdivisions at discontinuities.
        // G1 should work, but gives exceptions...

        switch (theC.GetType())
        {
            case GeomAbs_Line:
                return GCPnts_Linear;
            case GeomAbs_Circle:
                return GCPnts_Circular;
            case GeomAbs_BSplineCurve:
            {
                Geom2d_BSplineCurve aBS = theC.BSpline();
                return (aBS.NbPoles() == 2) ? GCPnts_Linear : GCPnts_Curved;
            }
            case GeomAbs_BezierCurve:
            {
                Geom2d_BezierCurve aBZ = theC.Bezier();
                return (aBZ.NbPoles() == 2) ? GCPnts_Linear : GCPnts_Curved;
            }
            default:
                return GCPnts_Curved;
        }
    }

    private static bool PerformCurve(NCollection_Sequence<double> theParameters,
                                     NCollection_Sequence<gp_Pnt> thePoints,
                                     Adaptor2d_Curve2d theC,
                                     double theDeflection,
                                     double theU1,
                                     double theU2,
                                     double theEPSILON,
                                     GeomAbs_Shape theContinuity)
    {
        int aNbmin = 2;
        int aNbCallQF = 0;

        gp_Pnt aPdeb;
        if (theContinuity <= GeomAbs_G1)
        {
            aPdeb = Value(theC, theU1);
            theParameters.Append(theU1);
            thePoints.Append(aPdeb);

            gp_Pnt aPfin = Value(theC, theU2);
            QuasiFleche(theC,
                        theDeflection * theDeflection,
                        theU1,
                        aPdeb,
                        theU2,
                        aPfin,
                        aNbmin,
                        theParameters,
                        thePoints,
                        ref aNbCallQF);
        }
        else
        {
            gp_Pnt aPfin;
            gp_Vec aDdeb, aDfin;
            D1(theC, theU1, out aPdeb, out aDdeb);
            theParameters.Append(theU1);
            thePoints.Append(aPdeb);

            double aDecreasedU2 = theU2 - Epsilon(theU2) * 10.0;
            D1(theC, aDecreasedU2, out aPfin, out aDfin);
            QuasiFleche(theC,
                        theDeflection * theDeflection,
                        theU1,
                        aPdeb,
                        aDdeb,
                        theU2,
                        aPfin,
                        aDfin,
                        aNbmin,
                        theEPSILON * theEPSILON,
                        theParameters,
                        thePoints,
                        ref aNbCallQF);
        }
        //  cout << "Nb of pts: " << Points.Length()<< endl;
        return true;
    }

    private static bool PerformComposite(NCollection_Sequence<double> theParameters,
                                         NCollection_Sequence<gp_Pnt> thePoints,
                                         Adaptor2d_Curve2d theC,
                                         double theDeflection,
                                         double theU1,
                                         double theU2,
                                         double theEPSILON,
                                         GeomAbs_Shape theContinuity)
    {
        // Consistency with Intervals
        int aNbIntervals = theC.NbIntervals(GeomAbs_C2);
        int aPIndex;
        NCollection_Array1<double> aTI = new(1, aNbIntervals + 1);
        theC.Intervals(aTI, GeomAbs_C2);
        BSplCLib.Hunt(aTI, theU1, out aPIndex);

        // iterate by continuous segments
        double aUa = theU1;
        for (int anIndex = aPIndex; ;)
        {
            double aUb = anIndex + 1 <= aTI.Upper() ? Min(theU2, aTI[anIndex + 1]) : theU2;
            if (!PerformCurve(theParameters,
                              thePoints,
                              theC,
                              theDeflection,
                              aUa,
                              aUb,
                              theEPSILON,
                              theContinuity))
            {
                return false;
            }

            ++anIndex;
            if (anIndex > aNbIntervals || theU2 < aTI[anIndex])
            {
                return true;
            }

            // remove last point to avoid duplication
            theParameters.Remove(theParameters.Length());
            thePoints.Remove(thePoints.Length());

            aUa = aUb;
        }
    }

    private void initialize(Adaptor2d_Curve2d theC, double theDeflection, double theU1, double theU2,
        GeomAbs_Shape theContinuity)
    {
        myCont = (theContinuity > GeomAbs_G1) ? GeomAbs_C1 : GeomAbs_C0;
        myDeflection = theDeflection;
        myDone = false;
        myParams.Clear();
        myPoints.Clear();

        double anEPSILON = Min(theC.Resolution(Precision.Confusion()), 1.0e50);
        GCPnts_DeflectionType aType = GetDefType(theC);
        double aU1 = Min(theU1, theU2);
        double aU2 = Max(theU1, theU2);

        if (aType == GCPnts_Curved || aType == GCPnts_DefComposite)
        {
            if (theC.GetType() == GeomAbs_BSplineCurve || theC.GetType() == GeomAbs_BezierCurve)
            {
                double aMaxPar = Max(Math.Abs(theC.FirstParameter()), Math.Abs(theC.LastParameter()));
                if (anEPSILON < Epsilon(aMaxPar))
                {
                    return;
                }
            }
        }

        switch (aType)
        {
            case GCPnts_Linear:
            {
                myDone = PerformLinear(theC, myParams, myPoints, aU1, aU2);
                break;
            }
            case GCPnts_Circular:
            {
                myDone = PerformCircular(theC, myParams, myPoints, theDeflection, aU1, aU2);
                break;
            }
            case GCPnts_Curved:
            {
                myDone = PerformCurve(myParams, myPoints, theC, theDeflection, aU1, aU2, anEPSILON, myCont);
                break;
            }
            case GCPnts_DefComposite:
            {
                myDone = PerformComposite(myParams, myPoints, theC, theDeflection, aU1, aU2, anEPSILON, myCont);
                break;
            }
        }
    }
}
