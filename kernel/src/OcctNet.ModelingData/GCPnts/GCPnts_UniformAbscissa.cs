// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_UniformAbscissa.hxx
// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_UniformAbscissa.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GCPnts_AbscissaType;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// Computes points on a curve spaced at a uniform curvilinear distance (abscissa), given either the distance or the
/// number of points.
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>Initialize</c> recomputes. A tolerance of -1 (the default) means
/// Precision::Confusion(). The templates over the curve type are ported per instantiation (decision 2004).
/// </remarks>
public sealed class GCPnts_UniformAbscissa
{
    private bool myDone;
    private int myNbPoints;
    private double myAbscissa;
    private NCollection_HArray1<double>? myParams;

    /// <summary>An empty algorithm; use <c>Initialize</c>.</summary>
    public GCPnts_UniformAbscissa()
    {
        myDone = false;
        myNbPoints = 0;
        myAbscissa = 0.0;
    }

    /// <summary>Points on the curve at the uniform distance theAbscissa.</summary>
    public GCPnts_UniformAbscissa(Adaptor3d_Curve theC, double theAbscissa, double theToler = -1)
        : this()
    {
        Initialize(theC, theAbscissa, theToler);
    }

    /// <summary>Points on the curve at the uniform distance theAbscissa.</summary>
    public GCPnts_UniformAbscissa(Adaptor2d_Curve2d theC, double theAbscissa, double theToler = -1)
        : this()
    {
        Initialize(theC, theAbscissa, theToler);
    }

    /// <summary>Points on the part theU1..theU2 of the curve at the uniform distance theAbscissa.</summary>
    public GCPnts_UniformAbscissa(Adaptor3d_Curve theC, double theAbscissa, double theU1, double theU2,
        double theToler = -1)
        : this()
    {
        Initialize(theC, theAbscissa, theU1, theU2, theToler);
    }

    /// <summary>Points on the part theU1..theU2 of the curve at the uniform distance theAbscissa.</summary>
    public GCPnts_UniformAbscissa(Adaptor2d_Curve2d theC, double theAbscissa, double theU1, double theU2,
        double theToler = -1)
        : this()
    {
        Initialize(theC, theAbscissa, theU1, theU2, theToler);
    }

    /// <summary>theNbPoints points on the curve at a uniform distance.</summary>
    /// <exception cref="Standard_ConstructionError">theNbPoints is below 2.</exception>
    public GCPnts_UniformAbscissa(Adaptor3d_Curve theC, int theNbPoints, double theToler = -1)
        : this()
    {
        Initialize(theC, theNbPoints, theToler);
    }

    /// <summary>theNbPoints points on the curve at a uniform distance.</summary>
    /// <exception cref="Standard_ConstructionError">theNbPoints is below 2.</exception>
    public GCPnts_UniformAbscissa(Adaptor2d_Curve2d theC, int theNbPoints, double theToler = -1)
        : this()
    {
        Initialize(theC, theNbPoints, theToler);
    }

    /// <summary>theNbPoints points on the part theU1..theU2 of the curve at a uniform distance.</summary>
    /// <exception cref="Standard_ConstructionError">theNbPoints is below 2.</exception>
    public GCPnts_UniformAbscissa(Adaptor3d_Curve theC, int theNbPoints, double theU1, double theU2,
        double theToler = -1)
        : this()
    {
        Initialize(theC, theNbPoints, theU1, theU2, theToler);
    }

    /// <summary>theNbPoints points on the part theU1..theU2 of the curve at a uniform distance.</summary>
    /// <exception cref="Standard_ConstructionError">theNbPoints is below 2.</exception>
    public GCPnts_UniformAbscissa(Adaptor2d_Curve2d theC, int theNbPoints, double theU1, double theU2,
        double theToler = -1)
        : this()
    {
        Initialize(theC, theNbPoints, theU1, theU2, theToler);
    }

    /// <summary>Recomputes points on the curve at the uniform distance theAbscissa.</summary>
    public void Initialize(Adaptor3d_Curve theC, double theAbscissa, double theToler = -1)
    {
        Initialize(theC, theAbscissa, theC.FirstParameter(), theC.LastParameter(), theToler);
    }

    /// <summary>Recomputes points on the curve at the uniform distance theAbscissa.</summary>
    public void Initialize(Adaptor2d_Curve2d theC, double theAbscissa, double theToler = -1)
    {
        Initialize(theC, theAbscissa, theC.FirstParameter(), theC.LastParameter(), theToler);
    }

    /// <summary>Recomputes points on the part theU1..theU2 of the curve at the uniform distance theAbscissa.</summary>
    public void Initialize(Adaptor3d_Curve theC, double theAbscissa, double theU1, double theU2, double theToler = -1)
    {
        initialize(theC, theAbscissa, theU1, theU2, theToler);
    }

    /// <summary>Recomputes points on the part theU1..theU2 of the curve at the uniform distance theAbscissa.</summary>
    public void Initialize(Adaptor2d_Curve2d theC, double theAbscissa, double theU1, double theU2,
        double theToler = -1)
    {
        initialize(theC, theAbscissa, theU1, theU2, theToler);
    }

    /// <summary>Recomputes theNbPoints points on the curve at a uniform distance.</summary>
    public void Initialize(Adaptor3d_Curve theC, int theNbPoints, double theToler = -1)
    {
        Initialize(theC, theNbPoints, theC.FirstParameter(), theC.LastParameter(), theToler);
    }

    /// <summary>Recomputes theNbPoints points on the curve at a uniform distance.</summary>
    public void Initialize(Adaptor2d_Curve2d theC, int theNbPoints, double theToler = -1)
    {
        Initialize(theC, theNbPoints, theC.FirstParameter(), theC.LastParameter(), theToler);
    }

    /// <summary>Recomputes theNbPoints points on the part theU1..theU2 of the curve at a uniform distance.</summary>
    public void Initialize(Adaptor3d_Curve theC, int theNbPoints, double theU1, double theU2, double theToler = -1)
    {
        initialize(theC, theNbPoints, theU1, theU2, theToler);
    }

    /// <summary>Recomputes theNbPoints points on the part theU1..theU2 of the curve at a uniform distance.</summary>
    public void Initialize(Adaptor2d_Curve2d theC, int theNbPoints, double theU1, double theU2,
        double theToler = -1)
    {
        initialize(theC, theNbPoints, theU1, theU2, theToler);
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone() => myDone;

    /// <summary>The number of points computed.</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    public int NbPoints()
    {
        StdFail_NotDone_Raise_if(!myDone, "GCPnts_UniformAbscissa::NbPoints()");
        return myNbPoints;
    }

    /// <summary>The parameter of the point of index Index (1 to NbPoints).</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    public double Parameter(int Index)
    {
        StdFail_NotDone_Raise_if(!myDone, "GCPnts_UniformAbscissa::Parameter()");
        return myParams!.Value(Index);
    }

    // NbPoints and Parameter without the StdFail_NotDone check, which OCCT's release build compiles out
    // (No_Exception): for ported callers that read a failed result as OCCT does (decision 2006).
    internal int NbPointsUnchecked() => myNbPoints;

    internal double ParameterUnchecked(int Index) => myParams!.Value(Index);

    /// <summary>The distance between consecutive points (computed, when a number of points was given).</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    public double Abscissa()
    {
        StdFail_NotDone_Raise_if(!myDone, "GCPnts_UniformAbscissa::Abscissa()");
        return myAbscissa;
    }

    // ---- Templates over the curve type (3D instantiation) ----

    private static double GetParameterLengthRatio(Adaptor3d_Curve theC)
    {
        switch (theC.GetType())
        {
            case GeomAbs_Circle:
            {
                return theC.Circle().Radius();
            }
            case GeomAbs_Line:
            {
                return 1.0;
            }
            case GeomAbs_BezierCurve:
            case GeomAbs_BSplineCurve:
            {
                if (!theC.IsRational())
                {
                    return theC.DN(0.0, 1).Magnitude();
                }
                return RealLast();
            }
            default:
            {
                return RealLast();
            }
        }
    }

    private static GCPnts_AbscissaType GetAbsType(Adaptor3d_Curve theC)
    {
        if (theC.NbIntervals(GeomAbs_C1) > 1)
        {
            return GCPnts_AbsComposite;
        }

        switch (theC.GetType())
        {
            case GeomAbs_Line:
            case GeomAbs_Circle:
            {
                return GCPnts_LengthParametrized;
            }
            case GeomAbs_BezierCurve:
            {
                Geom_BezierCurve aBZ = theC.Bezier();
                if (aBZ.NbPoles() == 2 && !aBZ.IsRational())
                {
                    return GCPnts_LengthParametrized;
                }
                return GCPnts_Parametrized;
            }
            case GeomAbs_BSplineCurve:
            {
                Geom_BSplineCurve aBS = theC.BSpline();
                if (aBS.NbPoles() == 2 && !aBS.IsRational())
                {
                    return GCPnts_LengthParametrized;
                }
                return GCPnts_Parametrized;
            }
            default:
            {
                return GCPnts_Parametrized;
            }
        }
    }

    private static bool Perform(NCollection_Array1<double> theParameters,
                                Adaptor3d_Curve theC,
                                double theAbscissa,
                                double theU1,
                                double theU2,
                                double theTotalLength,
                                out int theNbPoints,
                                double theEPSILON)
    {
        bool isLocalDone = true;
        double aUU1 = Min(theU1, theU2), aUU2 = Max(theU1, theU2);
        theNbPoints = 0;

        // this initialization avoids the computation of the Length of the curve
        double aDelta = (theAbscissa / theTotalLength) * (aUU2 - aUU1);
        int anIndex = 1;
        theParameters.SetValue(anIndex, aUU1);
        for (bool isNotDone = true; isNotDone;)
        {
            double aUi = theParameters.Value(anIndex) + aDelta;
            if (aUi > aUU2)
            {
                // MSV 21.04.2004: OCC5739 (GCPnts_UniformAbscissa gives incorrect distribution of points)
                // if (aUU2 - theParameters.Value (anIndex) > 0.01 * aDelta) { Index += 1; }
                // theParameters.SetValue (anIndex, aUU2);
                // isNotDone = false;
                // break;
                aUi = aUU2;
            }
            GCPnts_AbscissaPoint anAbscissaFinder = new(theC,
                                                        theAbscissa,
                                                        theParameters.Value(anIndex),
                                                        aUi,
                                                        theEPSILON);
            if (anAbscissaFinder.IsDone())
            {
                anIndex += 1;
                aUi = anAbscissaFinder.Parameter();
                if (Math.Abs(aUi - aUU2) <= theEPSILON)
                {
                    theParameters.SetValue(anIndex, aUU2);
                    isNotDone = false;
                }
                else if (aUi < aUU2)
                {
                    theParameters.SetValue(anIndex, aUi);
                }
                else
                {
                    theParameters.SetValue(anIndex, aUU2);
                    isNotDone = false;
                }
                isNotDone = isNotDone && (anIndex + 1 <= theParameters.Length());
            }
            else
            {
                isLocalDone = false;
                isNotDone = true;
                aDelta -= aDelta / 10;
                if (aDelta <= Precision.PConfusion())
                {
                    break;
                }
            }
        }
        theNbPoints = anIndex;
        return isLocalDone;
    }

    private static bool PerformLengthParametrized(NCollection_Array1<double> theParameters,
                                                  Adaptor3d_Curve theC,
                                                  double theAbscissa,
                                                  double theU1,
                                                  double theU2,
                                                  double theTotalLength,
                                                  out int theNbPoints,
                                                  double theEPSILON)
    {
        double aUU1 = Min(theU1, theU2);
        double aUU2 = Max(theU1, theU2);

        // Ratio is defined as dl = Ratio * du
        // for a circle of gp Ratio is equal to the radius of the circle.
        // for a line of gp ratio is equal to 1.0
        double aRatio = GetParameterLengthRatio(theC);

        if (theAbscissa < 0.0e0)
        {
            aUU2 = Min(theU1, theU2);
            aUU1 = Max(theU1, theU2);
        }

        double aDelta = (theAbscissa / theTotalLength) * (aUU2 - aUU1);
        int anIndex = 1;
        theNbPoints = 0;
        theParameters.SetValue(anIndex, aUU1);
        for (bool isNotDone = true; isNotDone;)
        {
            anIndex += 1;
            double aUi = theParameters.Value(anIndex - 1) + aDelta;
            if (Math.Abs(aUi - aUU2) <= theEPSILON)
            {
                theParameters.SetValue(anIndex, aUU2);
                isNotDone = false;
            }
            else if (aUi < aUU2)
            {
                theParameters.SetValue(anIndex, aUi);
            }
            else
            {
                isNotDone = false;
                if (Math.Abs(theParameters.Value(anIndex - 1) - aUU2) * aRatio / theAbscissa < 0.1)
                {
                    theParameters.SetValue(anIndex - 1, aUU2);
                    anIndex -= 1;
                }
                else
                {
                    theParameters.SetValue(anIndex, aUU2);
                }
            }
            isNotDone = (anIndex + 1 <= theParameters.Length()) && isNotDone;
        }
        theNbPoints = anIndex;
        return true;
    }

    private void initialize(Adaptor3d_Curve theC, double theAbscissa, double theU1, double theU2, double theTol)
    {
        myAbscissa = theAbscissa;
        myNbPoints = 0;
        myDone = false;

        double anEPSILON = theC.Resolution(Max(theTol, Precision.Confusion()));
        double aL = GCPnts_AbscissaPoint.Length(theC, theU1, theU2, anEPSILON);
        if (aL <= Precision.Confusion())
        {
            return;
        }

        // compute the total Length here so that we can guess
        // the number of points instead of letting the constructor
        // of CPnts_AbscissaPoint do that and lose the information
        double aSizeR = aL / Math.Abs(theAbscissa) + 5;
        if (aSizeR >= IntegerLast()) // modified by Igor Motchalov 23/04/2001
        {
            return;
        }

        int aSize = (int)aSizeR;
        if (myParams is not null)
        {
            if (myParams.Length() < aSize)
            {
                myParams = null;
                myParams = new NCollection_HArray1<double>(1, aSize);
            }
        }
        else
        {
            myParams = new NCollection_HArray1<double>(1, aSize);
        }

        GCPnts_AbscissaType aType = GetAbsType(theC);
        switch (aType)
        {
            case GCPnts_LengthParametrized:
            {
                myDone = PerformLengthParametrized(myParams.ChangeArray1(),
                                                   theC,
                                                   theAbscissa,
                                                   theU1,
                                                   theU2,
                                                   aL,
                                                   out myNbPoints,
                                                   anEPSILON);
                break;
            }
            case GCPnts_Parametrized:
            case GCPnts_AbsComposite:
            {
                myDone = Perform(myParams.ChangeArray1(),
                                 theC,
                                 theAbscissa,
                                 theU1,
                                 theU2,
                                 aL,
                                 out myNbPoints,
                                 anEPSILON);
                break;
            }
        }
    }

    private void initialize(Adaptor3d_Curve theC, int theNbPoints, double theU1, double theU2, double theTol)
    {
        Standard_ConstructionError_Raise_if(
            theNbPoints <= 1,
            "GCPnts_UniformAbscissa::Initialize() - number of points should be >= 2");
        myNbPoints = 0;
        myDone = false;

        double anEPSILON = theC.Resolution(Max(theTol, Precision.Confusion()));
        // although very similar to Initialize with Abscissa this avoid
        // the computation of the total length of the curve twice
        double aL = GCPnts_AbscissaPoint.Length(theC, theU1, theU2, anEPSILON);
        if (aL <= Precision.Confusion())
        {
            return;
        }

        double anAbscissa = myAbscissa = aL / (theNbPoints - 1);

        // compute the total Length here so that we can guess
        // the number of points instead of letting the constructor
        // of CPnts_AbscissaPoint do that and lose the information
        int aSize = theNbPoints + 5;
        if (myParams is not null)
        {
            if (myParams.Length() < aSize)
            {
                myParams = null;
                myParams = new NCollection_HArray1<double>(1, aSize);
            }
        }
        else
        {
            myParams = new NCollection_HArray1<double>(1, aSize);
        }

        myNbPoints = 0;
        GCPnts_AbscissaType aType = GetAbsType(theC);
        switch (aType)
        {
            case GCPnts_LengthParametrized:
            {
                myDone = PerformLengthParametrized(myParams.ChangeArray1(),
                                                   theC,
                                                   anAbscissa,
                                                   theU1,
                                                   theU2,
                                                   aL,
                                                   out myNbPoints,
                                                   anEPSILON);
                break;
            }
            case GCPnts_Parametrized:
            case GCPnts_AbsComposite:
            {
                myDone = Perform(myParams.ChangeArray1(),
                                 theC,
                                 anAbscissa,
                                 theU1,
                                 theU2,
                                 aL,
                                 out myNbPoints,
                                 anEPSILON);
                break;
            }
        }
    }

    // ---- Templates over the curve type (2D instantiation) ----

    private static double GetParameterLengthRatio(Adaptor2d_Curve2d theC)
    {
        switch (theC.GetType())
        {
            case GeomAbs_Circle:
            {
                return theC.Circle().Radius();
            }
            case GeomAbs_Line:
            {
                return 1.0;
            }
            case GeomAbs_BezierCurve:
            case GeomAbs_BSplineCurve:
            {
                if (!theC.IsRational())
                {
                    return theC.DN(0.0, 1).Magnitude();
                }
                return RealLast();
            }
            default:
            {
                return RealLast();
            }
        }
    }

    private static GCPnts_AbscissaType GetAbsType(Adaptor2d_Curve2d theC)
    {
        if (theC.NbIntervals(GeomAbs_C1) > 1)
        {
            return GCPnts_AbsComposite;
        }

        switch (theC.GetType())
        {
            case GeomAbs_Line:
            case GeomAbs_Circle:
            {
                return GCPnts_LengthParametrized;
            }
            case GeomAbs_BezierCurve:
            {
                Geom2d_BezierCurve aBZ = theC.Bezier();
                if (aBZ.NbPoles() == 2 && !aBZ.IsRational())
                {
                    return GCPnts_LengthParametrized;
                }
                return GCPnts_Parametrized;
            }
            case GeomAbs_BSplineCurve:
            {
                Geom2d_BSplineCurve aBS = theC.BSpline();
                if (aBS.NbPoles() == 2 && !aBS.IsRational())
                {
                    return GCPnts_LengthParametrized;
                }
                return GCPnts_Parametrized;
            }
            default:
            {
                return GCPnts_Parametrized;
            }
        }
    }

    private static bool Perform(NCollection_Array1<double> theParameters,
                                Adaptor2d_Curve2d theC,
                                double theAbscissa,
                                double theU1,
                                double theU2,
                                double theTotalLength,
                                out int theNbPoints,
                                double theEPSILON)
    {
        bool isLocalDone = true;
        double aUU1 = Min(theU1, theU2), aUU2 = Max(theU1, theU2);
        theNbPoints = 0;

        // this initialization avoids the computation of the Length of the curve
        double aDelta = (theAbscissa / theTotalLength) * (aUU2 - aUU1);
        int anIndex = 1;
        theParameters.SetValue(anIndex, aUU1);
        for (bool isNotDone = true; isNotDone;)
        {
            double aUi = theParameters.Value(anIndex) + aDelta;
            if (aUi > aUU2)
            {
                // MSV 21.04.2004: OCC5739 (GCPnts_UniformAbscissa gives incorrect distribution of points)
                // if (aUU2 - theParameters.Value (anIndex) > 0.01 * aDelta) { Index += 1; }
                // theParameters.SetValue (anIndex, aUU2);
                // isNotDone = false;
                // break;
                aUi = aUU2;
            }
            GCPnts_AbscissaPoint anAbscissaFinder = new(theC,
                                                        theAbscissa,
                                                        theParameters.Value(anIndex),
                                                        aUi,
                                                        theEPSILON);
            if (anAbscissaFinder.IsDone())
            {
                anIndex += 1;
                aUi = anAbscissaFinder.Parameter();
                if (Math.Abs(aUi - aUU2) <= theEPSILON)
                {
                    theParameters.SetValue(anIndex, aUU2);
                    isNotDone = false;
                }
                else if (aUi < aUU2)
                {
                    theParameters.SetValue(anIndex, aUi);
                }
                else
                {
                    theParameters.SetValue(anIndex, aUU2);
                    isNotDone = false;
                }
                isNotDone = isNotDone && (anIndex + 1 <= theParameters.Length());
            }
            else
            {
                isLocalDone = false;
                isNotDone = true;
                aDelta -= aDelta / 10;
                if (aDelta <= Precision.PConfusion())
                {
                    break;
                }
            }
        }
        theNbPoints = anIndex;
        return isLocalDone;
    }

    private static bool PerformLengthParametrized(NCollection_Array1<double> theParameters,
                                                  Adaptor2d_Curve2d theC,
                                                  double theAbscissa,
                                                  double theU1,
                                                  double theU2,
                                                  double theTotalLength,
                                                  out int theNbPoints,
                                                  double theEPSILON)
    {
        double aUU1 = Min(theU1, theU2);
        double aUU2 = Max(theU1, theU2);

        // Ratio is defined as dl = Ratio * du
        // for a circle of gp Ratio is equal to the radius of the circle.
        // for a line of gp ratio is equal to 1.0
        double aRatio = GetParameterLengthRatio(theC);

        if (theAbscissa < 0.0e0)
        {
            aUU2 = Min(theU1, theU2);
            aUU1 = Max(theU1, theU2);
        }

        double aDelta = (theAbscissa / theTotalLength) * (aUU2 - aUU1);
        int anIndex = 1;
        theNbPoints = 0;
        theParameters.SetValue(anIndex, aUU1);
        for (bool isNotDone = true; isNotDone;)
        {
            anIndex += 1;
            double aUi = theParameters.Value(anIndex - 1) + aDelta;
            if (Math.Abs(aUi - aUU2) <= theEPSILON)
            {
                theParameters.SetValue(anIndex, aUU2);
                isNotDone = false;
            }
            else if (aUi < aUU2)
            {
                theParameters.SetValue(anIndex, aUi);
            }
            else
            {
                isNotDone = false;
                if (Math.Abs(theParameters.Value(anIndex - 1) - aUU2) * aRatio / theAbscissa < 0.1)
                {
                    theParameters.SetValue(anIndex - 1, aUU2);
                    anIndex -= 1;
                }
                else
                {
                    theParameters.SetValue(anIndex, aUU2);
                }
            }
            isNotDone = (anIndex + 1 <= theParameters.Length()) && isNotDone;
        }
        theNbPoints = anIndex;
        return true;
    }

    private void initialize(Adaptor2d_Curve2d theC, double theAbscissa, double theU1, double theU2, double theTol)
    {
        myAbscissa = theAbscissa;
        myNbPoints = 0;
        myDone = false;

        double anEPSILON = theC.Resolution(Max(theTol, Precision.Confusion()));
        double aL = GCPnts_AbscissaPoint.Length(theC, theU1, theU2, anEPSILON);
        if (aL <= Precision.Confusion())
        {
            return;
        }

        // compute the total Length here so that we can guess
        // the number of points instead of letting the constructor
        // of CPnts_AbscissaPoint do that and lose the information
        double aSizeR = aL / Math.Abs(theAbscissa) + 5;
        if (aSizeR >= IntegerLast()) // modified by Igor Motchalov 23/04/2001
        {
            return;
        }

        int aSize = (int)aSizeR;
        if (myParams is not null)
        {
            if (myParams.Length() < aSize)
            {
                myParams = null;
                myParams = new NCollection_HArray1<double>(1, aSize);
            }
        }
        else
        {
            myParams = new NCollection_HArray1<double>(1, aSize);
        }

        GCPnts_AbscissaType aType = GetAbsType(theC);
        switch (aType)
        {
            case GCPnts_LengthParametrized:
            {
                myDone = PerformLengthParametrized(myParams.ChangeArray1(),
                                                   theC,
                                                   theAbscissa,
                                                   theU1,
                                                   theU2,
                                                   aL,
                                                   out myNbPoints,
                                                   anEPSILON);
                break;
            }
            case GCPnts_Parametrized:
            case GCPnts_AbsComposite:
            {
                myDone = Perform(myParams.ChangeArray1(),
                                 theC,
                                 theAbscissa,
                                 theU1,
                                 theU2,
                                 aL,
                                 out myNbPoints,
                                 anEPSILON);
                break;
            }
        }
    }

    private void initialize(Adaptor2d_Curve2d theC, int theNbPoints, double theU1, double theU2, double theTol)
    {
        Standard_ConstructionError_Raise_if(
            theNbPoints <= 1,
            "GCPnts_UniformAbscissa::Initialize() - number of points should be >= 2");
        myNbPoints = 0;
        myDone = false;

        double anEPSILON = theC.Resolution(Max(theTol, Precision.Confusion()));
        // although very similar to Initialize with Abscissa this avoid
        // the computation of the total length of the curve twice
        double aL = GCPnts_AbscissaPoint.Length(theC, theU1, theU2, anEPSILON);
        if (aL <= Precision.Confusion())
        {
            return;
        }

        double anAbscissa = myAbscissa = aL / (theNbPoints - 1);

        // compute the total Length here so that we can guess
        // the number of points instead of letting the constructor
        // of CPnts_AbscissaPoint do that and lose the information
        int aSize = theNbPoints + 5;
        if (myParams is not null)
        {
            if (myParams.Length() < aSize)
            {
                myParams = null;
                myParams = new NCollection_HArray1<double>(1, aSize);
            }
        }
        else
        {
            myParams = new NCollection_HArray1<double>(1, aSize);
        }

        myNbPoints = 0;
        GCPnts_AbscissaType aType = GetAbsType(theC);
        switch (aType)
        {
            case GCPnts_LengthParametrized:
            {
                myDone = PerformLengthParametrized(myParams.ChangeArray1(),
                                                   theC,
                                                   anAbscissa,
                                                   theU1,
                                                   theU2,
                                                   aL,
                                                   out myNbPoints,
                                                   anEPSILON);
                break;
            }
            case GCPnts_Parametrized:
            case GCPnts_AbsComposite:
            {
                myDone = Perform(myParams.ChangeArray1(),
                                 theC,
                                 anAbscissa,
                                 theU1,
                                 theU2,
                                 aL,
                                 out myNbPoints,
                                 anEPSILON);
                break;
            }
        }
    }
}
