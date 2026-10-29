// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_AbscissaPoint.hxx
// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_AbscissaPoint.cxx
// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_TCurveTypes.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GCPnts_AbscissaType;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// Computes the length of a curve, or a point on a curve at a given distance (abscissa) from another point.
/// </summary>
/// <remarks>
/// OCCT's function templates over the curve type are ported once per instantiation (Adaptor3d_Curve and
/// Adaptor2d_Curve2d), with identical bodies (decision 187). GCPnts_TCurveTypes, the traits that resolve the 2D
/// and 3D classes, is replaced by the types written out in each instantiation.
/// </remarks>
public sealed class GCPnts_AbscissaPoint
{
    private readonly CPnts_AbscissaPoint myComputer = new();

    /// <summary>The length of the curve.</summary>
    public static double Length(Adaptor3d_Curve theC)
    {
        return GCPnts_AbscissaPoint.Length(theC, theC.FirstParameter(), theC.LastParameter());
    }

    /// <summary>The length of the curve.</summary>
    public static double Length(Adaptor2d_Curve2d theC)
    {
        return GCPnts_AbscissaPoint.Length(theC, theC.FirstParameter(), theC.LastParameter());
    }

    /// <summary>The length of the curve with the given tolerance.</summary>
    public static double Length(Adaptor3d_Curve theC, double theTol)
    {
        return GCPnts_AbscissaPoint.Length(theC, theC.FirstParameter(), theC.LastParameter(), theTol);
    }

    /// <summary>The length of the curve with the given tolerance.</summary>
    public static double Length(Adaptor2d_Curve2d theC, double theTol)
    {
        return GCPnts_AbscissaPoint.Length(theC, theC.FirstParameter(), theC.LastParameter(), theTol);
    }

    /// <summary>The length of the part of the curve between the parameters theU1 and theU2.</summary>
    public static double Length(Adaptor3d_Curve theC, double theU1, double theU2)
    {
        return length(theC, theU1, theU2, null);
    }

    /// <summary>The length of the part of the curve between the parameters theU1 and theU2.</summary>
    public static double Length(Adaptor2d_Curve2d theC, double theU1, double theU2)
    {
        return length(theC, theU1, theU2, null);
    }

    /// <summary>The length of the part of the curve between theU1 and theU2 with the given tolerance.</summary>
    public static double Length(Adaptor3d_Curve theC, double theU1, double theU2, double theTol)
    {
        return length(theC, theU1, theU2, theTol);
    }

    /// <summary>The length of the part of the curve between theU1 and theU2 with the given tolerance.</summary>
    public static double Length(Adaptor2d_Curve2d theC, double theU1, double theU2, double theTol)
    {
        return length(theC, theU1, theU2, theTol);
    }

    /// <summary>An empty algorithm.</summary>
    public GCPnts_AbscissaPoint()
    {
    }

    /// <summary>The point at the distance theAbscissa from the point of parameter theU0.</summary>
    /// <exception cref="Standard_ConstructionError">The curve's length is below Precision::Confusion().</exception>
    public GCPnts_AbscissaPoint(Adaptor3d_Curve theC, double theAbscissa, double theU0)
    {
        compute(theC, theAbscissa, theU0);
    }

    /// <summary>The point at the distance theAbscissa from the point of parameter theU0.</summary>
    /// <exception cref="Standard_ConstructionError">The curve's length is below Precision::Confusion().</exception>
    public GCPnts_AbscissaPoint(Adaptor2d_Curve2d theC, double theAbscissa, double theU0)
    {
        compute(theC, theAbscissa, theU0);
    }

    /// <summary>The point at the distance theAbscissa from the point of parameter theU0, with the tolerance theTol.</summary>
    public GCPnts_AbscissaPoint(double theTol, Adaptor3d_Curve theC, double theAbscissa, double theU0)
    {
        advCompute(theTol, theC, theAbscissa, theU0);
    }

    /// <summary>The point at the distance theAbscissa from the point of parameter theU0, with the tolerance theTol.</summary>
    public GCPnts_AbscissaPoint(double theTol, Adaptor2d_Curve2d theC, double theAbscissa, double theU0)
    {
        advCompute(theTol, theC, theAbscissa, theU0);
    }

    /// <summary>The point at the distance theAbscissa from the point of parameter theU0, starting the search at theUi.</summary>
    public GCPnts_AbscissaPoint(Adaptor3d_Curve theC, double theAbscissa, double theU0, double theUi)
    {
        double anAbscis = theAbscissa, aUU0 = theU0, aUUi = theUi;
        Compute(myComputer, theC, ref anAbscis, ref aUU0, ref aUUi, theC.Resolution(Precision.Confusion()));
    }

    /// <summary>The point at the distance theAbscissa from the point of parameter theU0, starting the search at theUi.</summary>
    public GCPnts_AbscissaPoint(Adaptor2d_Curve2d theC, double theAbscissa, double theU0, double theUi)
    {
        double anAbscis = theAbscissa, aUU0 = theU0, aUUi = theUi;
        Compute(myComputer, theC, ref anAbscis, ref aUU0, ref aUUi, theC.Resolution(Precision.Confusion()));
    }

    /// <summary>
    /// The point at the distance theAbscissa from the point of parameter theU0, starting the search at theUi, with the
    /// parametric tolerance theTol.
    /// </summary>
    public GCPnts_AbscissaPoint(Adaptor3d_Curve theC, double theAbscissa, double theU0, double theUi, double theTol)
    {
        double anAbscis = theAbscissa, aUU0 = theU0, aUUi = theUi;
        AdvCompute(myComputer, theC, ref anAbscis, ref aUU0, ref aUUi, theTol);
    }

    /// <summary>
    /// The point at the distance theAbscissa from the point of parameter theU0, starting the search at theUi, with the
    /// parametric tolerance theTol.
    /// </summary>
    public GCPnts_AbscissaPoint(Adaptor2d_Curve2d theC, double theAbscissa, double theU0, double theUi, double theTol)
    {
        double anAbscis = theAbscissa, aUU0 = theU0, aUUi = theUi;
        AdvCompute(myComputer, theC, ref anAbscis, ref aUU0, ref aUUi, theTol);
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone() => myComputer.IsDone();

    /// <summary>The parameter of the point found.</summary>
    /// <exception cref="StdFail_NotDone">The computation failed.</exception>
    public double Parameter() => myComputer.Parameter();

    // ---- Templates over the curve type (3D instantiation) ----

    // Dimension independent used to implement GCPnts_AbscissaPoint
    // compute the type and the length ratio if GCPnts_LengthParametrized.
    private static GCPnts_AbscissaType computeType(Adaptor3d_Curve theC, ref double theRatio)
    {
        if (theC.NbIntervals(GeomAbs_CN) > 1)
        {
            return GCPnts_AbsComposite;
        }

        switch (theC.GetType())
        {
            case GeomAbs_Line:
            {
                theRatio = 1.0;
                return GCPnts_LengthParametrized;
            }
            case GeomAbs_Circle:
            {
                theRatio = theC.Circle().Radius();
                return GCPnts_LengthParametrized;
            }
            case GeomAbs_BezierCurve:
            {
                Geom_BezierCurve aBz = theC.Bezier();
                if (aBz.NbPoles() == 2 && !aBz.IsRational())
                {
                    theRatio = aBz.DN(0, 1).Magnitude();
                    return GCPnts_LengthParametrized;
                }
                return GCPnts_Parametrized;
            }
            case GeomAbs_BSplineCurve:
            {
                Geom_BSplineCurve aBs = theC.BSpline();
                if (aBs.NbPoles() == 2 && !aBs.IsRational())
                {
                    theRatio = aBs.DN(aBs.FirstParameter(), 1).Magnitude();
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

    // Compute a point at distance theAbscis from parameter theU0 using theUi as initial guess
    private static void Compute(CPnts_AbscissaPoint theComputer,
                                Adaptor3d_Curve theC,
                                ref double theAbscis,
                                ref double theU0,
                                ref double theUi,
                                double theEPSILON)
    {
        // test for easy solution
        if (Math.Abs(theAbscis) <= Precision.Confusion())
        {
            theComputer.SetParameter(theU0);
            return;
        }

        double aRatio = 1.0;
        GCPnts_AbscissaType aType = computeType(theC, ref aRatio);
        switch (aType)
        {
            case GCPnts_LengthParametrized:
            {
                theComputer.SetParameter(theU0 + theAbscis / aRatio);
                return;
            }
            case GCPnts_Parametrized:
            {
                theComputer.Init(theC);
                theComputer.Perform(theAbscis, theU0, theUi, theEPSILON);
                return;
            }
            case GCPnts_AbsComposite:
            {
                int aNbIntervals = theC.NbIntervals(GeomAbs_CN);
                NCollection_Array1<double> aTI = new(1, aNbIntervals + 1);
                theC.Intervals(aTI, GeomAbs_CN);
                double aL = 0.0, aSign = 1.0;
                int anIndex;
                BSplCLib.Hunt(aTI, theU0, out anIndex);
                int aDirection = 1;
                if (theAbscis < 0)
                {
                    aDirection = 0;
                    theAbscis = -theAbscis;
                    aSign = -1.0;
                }

                while (anIndex >= 1 && anIndex <= aNbIntervals)
                {
                    aL = CPnts_AbscissaPoint.Length(theC, theU0, aTI[anIndex + aDirection]);
                    if (Math.Abs(aL - theAbscis) <= Precision.Confusion())
                    {
                        theComputer.SetParameter(aTI[anIndex + aDirection]);
                        return;
                    }

                    if (aL > theAbscis)
                    {
                        if (theUi < aTI[anIndex] || theUi > aTI[anIndex + 1])
                        {
                            theUi = (theAbscis / aL) * (aTI[anIndex + 1] - theU0);
                            if (aDirection != 0)
                            {
                                theUi = theU0 + theUi;
                            }
                            else
                            {
                                theUi = theU0 - theUi;
                            }
                        }
                        theComputer.Init(theC, aTI[anIndex], aTI[anIndex + 1]);
                        theComputer.Perform(aSign * theAbscis, theU0, theUi, theEPSILON);
                        return;
                    }
                    else
                    {
                        theU0 = aTI[anIndex + aDirection];
                        theAbscis -= aL;
                    }
                    if (aDirection != 0)
                    {
                        ++anIndex;
                    }
                    else
                    {
                        --anIndex;
                    }
                }

                // Push a little bit outside the limits (hairy !!!)
                theUi = theU0 + 0.1;
                theComputer.Init(theC, theU0, theU0 + 0.2);
                theComputer.Perform(aSign * theAbscis, theU0, theUi, theEPSILON);
                return;
            }
        }
    }

    // Introduced by rbv for curvilinear parametrization
    // performs more appropriate tolerance management.
    private static void AdvCompute(CPnts_AbscissaPoint theComputer,
                                   Adaptor3d_Curve theC,
                                   ref double theAbscis,
                                   ref double theU0,
                                   ref double theUi,
                                   double theEPSILON)
    {
        double aRatio = 1.0;
        GCPnts_AbscissaType aType = computeType(theC, ref aRatio);
        switch (aType)
        {
            case GCPnts_LengthParametrized:
            {
                theComputer.SetParameter(theU0 + theAbscis / aRatio);
                return;
            }
            case GCPnts_Parametrized:
            {
                // theComputer.Init (theC);
                theComputer.Init(theC, theEPSILON); // rbv's modification
                theComputer.AdvPerform(theAbscis, theU0, theUi, theEPSILON);
                return;
            }
            case GCPnts_AbsComposite:
            {
                int aNbIntervals = theC.NbIntervals(GeomAbs_CN);
                NCollection_Array1<double> aTI = new(1, aNbIntervals + 1);
                theC.Intervals(aTI, GeomAbs_CN);
                double aL = 0.0, aSign = 1.0;
                int anIndex;
                BSplCLib.Hunt(aTI, theU0, out anIndex);

                int aDirection = 1;
                if (theAbscis < 0)
                {
                    aDirection = 0;
                    theAbscis = -theAbscis;
                    aSign = -1.0;
                }

                if (anIndex == 0 && aDirection > 0)
                {
                    aL = CPnts_AbscissaPoint.Length(theC, theU0, aTI[anIndex + aDirection], theEPSILON);
                    if (Math.Abs(aL - theAbscis) <= /*Precision::Confusion()*/ theEPSILON)
                    {
                        theComputer.SetParameter(aTI[anIndex + aDirection]);
                        return;
                    }

                    if (aL > theAbscis)
                    {
                        if (theUi > aTI[anIndex + 1])
                        {
                            theUi = (theAbscis / aL) * (aTI[anIndex + 1] - theU0);
                            theUi = theU0 + theUi;
                        }
                        theComputer.Init(theC, theU0, aTI[anIndex + 1], theEPSILON);
                        theComputer.AdvPerform(aSign * theAbscis, theU0, theUi, theEPSILON);
                        return;
                    }
                    else
                    {
                        theU0 = aTI[anIndex + aDirection];
                        theAbscis -= aL;
                    }
                    ++anIndex;
                }

                while (anIndex >= 1 && anIndex <= aNbIntervals)
                {
                    aL = CPnts_AbscissaPoint.Length(theC, theU0, aTI[anIndex + aDirection], theEPSILON);
                    if (Math.Abs(aL - theAbscis) <= Precision.PConfusion())
                    {
                        theComputer.SetParameter(aTI[anIndex + aDirection]);
                        return;
                    }

                    if (aL > theAbscis)
                    {
                        if (theUi < aTI[anIndex] || theUi > aTI[anIndex + 1])
                        {
                            theUi = (theAbscis / aL) * (aTI[anIndex + 1] - theU0);
                            if (aDirection != 0)
                            {
                                theUi = theU0 + theUi;
                            }
                            else
                            {
                                theUi = theU0 - theUi;
                            }
                        }
                        theComputer.Init(theC, aTI[anIndex], aTI[anIndex + 1], theEPSILON);
                        theComputer.AdvPerform(aSign * theAbscis, theU0, theUi, theEPSILON);
                        return;
                    }
                    else
                    {
                        theU0 = aTI[anIndex + aDirection];
                        theAbscis -= aL;
                    }
                    if (aDirection != 0)
                    {
                        ++anIndex;
                    }
                    else
                    {
                        --anIndex;
                    }
                }

                // Push a little bit outside the limits (hairy !!!)
                bool isNonPeriodic = !theC.IsPeriodic();
                theUi = theU0 + aSign * 0.1;
                double aU1 = theU0 + aSign * 0.2;
                if (isNonPeriodic)
                {
                    if (aSign > 0)
                    {
                        theUi = Min(theUi, theC.LastParameter());
                        aU1 = Min(aU1, theC.LastParameter());
                    }
                    else
                    {
                        theUi = Max(theUi, theC.FirstParameter());
                        aU1 = Max(aU1, theC.FirstParameter());
                    }
                }

                theComputer.Init(theC, theU0, aU1, theEPSILON);
                theComputer.AdvPerform(aSign * theAbscis, theU0, theUi, theEPSILON);
                return;
            }
        }
    }

    // theTol is null for no tolerance (OCCT's const double*).
    private static double length(Adaptor3d_Curve theC, double theU1, double theU2, double? theTol)
    {
        double aRatio = 1.0;
        GCPnts_AbscissaType aType = computeType(theC, ref aRatio);
        switch (aType)
        {
            case GCPnts_LengthParametrized:
            {
                return Math.Abs(theU2 - theU1) * aRatio;
            }
            case GCPnts_Parametrized:
            {
                return theTol != null ? CPnts_AbscissaPoint.Length(theC, theU1, theU2, theTol.Value)
                                      : CPnts_AbscissaPoint.Length(theC, theU1, theU2);
            }
            case GCPnts_AbsComposite:
            {
                int aNbIntervals = theC.NbIntervals(GeomAbs_CN);
                NCollection_Array1<double> aTI = new(1, aNbIntervals + 1);
                theC.Intervals(aTI, GeomAbs_CN);
                double aUU1 = Min(theU1, theU2);
                double aUU2 = Max(theU1, theU2);
                double aL = 0.0;
                for (int anIndex = 1; anIndex <= aNbIntervals; ++anIndex)
                {
                    if (aTI[anIndex] > aUU2)
                    {
                        break;
                    }
                    if (aTI[anIndex + 1] < aUU1)
                    {
                        continue;
                    }
                    if (theTol != null)
                    {
                        aL += CPnts_AbscissaPoint.Length(theC,
                                                         Max(aTI[anIndex], aUU1),
                                                         Min(aTI[anIndex + 1], aUU2),
                                                         theTol.Value);
                    }
                    else
                    {
                        aL += CPnts_AbscissaPoint.Length(theC,
                                                         Max(aTI[anIndex], aUU1),
                                                         Min(aTI[anIndex + 1], aUU2));
                    }
                }
                return aL;
            }
        }
        return RealLast();
    }

    private void compute(Adaptor3d_Curve theC, double theAbscissa, double theU0)
    {
        double aL = GCPnts_AbscissaPoint.Length(theC);
        if (aL < Precision.Confusion())
        {
            throw new Standard_ConstructionError();
        }

        double anAbscis = theAbscissa;
        double aUU0 = theU0;
        double aUUi = theU0 + (anAbscis / aL) * (theC.LastParameter() - theC.FirstParameter());
        Compute(myComputer, theC, ref anAbscis, ref aUU0, ref aUUi, theC.Resolution(Precision.Confusion()));
    }

    private void advCompute(double theTol, Adaptor3d_Curve theC, double theAbscissa, double theU0)
    {
        double aL = GCPnts_AbscissaPoint.Length(theC, theTol);
        /*if (aL < Precision::Confusion())
        {
          throw Standard_ConstructionError ("GCPnts_AbscissaPoint::GCPnts_AbscissaPoint");
        }*/
        double anAbscis = theAbscissa;
        double aUU0 = theU0;
        double aUUi = 0.0;
        if (aL >= Precision.Confusion())
        {
            aUUi = theU0 + (anAbscis / aL) * (theC.LastParameter() - theC.FirstParameter());
        }
        else
        {
            aUUi = theU0;
        }
        AdvCompute(myComputer, theC, ref anAbscis, ref aUU0, ref aUUi, theTol);
    }

    // ---- Templates over the curve type (2D instantiation) ----

    // Dimension independent used to implement GCPnts_AbscissaPoint
    // compute the type and the length ratio if GCPnts_LengthParametrized.
    private static GCPnts_AbscissaType computeType(Adaptor2d_Curve2d theC, ref double theRatio)
    {
        if (theC.NbIntervals(GeomAbs_CN) > 1)
        {
            return GCPnts_AbsComposite;
        }

        switch (theC.GetType())
        {
            case GeomAbs_Line:
            {
                theRatio = 1.0;
                return GCPnts_LengthParametrized;
            }
            case GeomAbs_Circle:
            {
                theRatio = theC.Circle().Radius();
                return GCPnts_LengthParametrized;
            }
            case GeomAbs_BezierCurve:
            {
                Geom2d_BezierCurve aBz = theC.Bezier();
                if (aBz.NbPoles() == 2 && !aBz.IsRational())
                {
                    theRatio = aBz.DN(0, 1).Magnitude();
                    return GCPnts_LengthParametrized;
                }
                return GCPnts_Parametrized;
            }
            case GeomAbs_BSplineCurve:
            {
                Geom2d_BSplineCurve aBs = theC.BSpline();
                if (aBs.NbPoles() == 2 && !aBs.IsRational())
                {
                    theRatio = aBs.DN(aBs.FirstParameter(), 1).Magnitude();
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

    // Compute a point at distance theAbscis from parameter theU0 using theUi as initial guess
    private static void Compute(CPnts_AbscissaPoint theComputer,
                                Adaptor2d_Curve2d theC,
                                ref double theAbscis,
                                ref double theU0,
                                ref double theUi,
                                double theEPSILON)
    {
        // test for easy solution
        if (Math.Abs(theAbscis) <= Precision.Confusion())
        {
            theComputer.SetParameter(theU0);
            return;
        }

        double aRatio = 1.0;
        GCPnts_AbscissaType aType = computeType(theC, ref aRatio);
        switch (aType)
        {
            case GCPnts_LengthParametrized:
            {
                theComputer.SetParameter(theU0 + theAbscis / aRatio);
                return;
            }
            case GCPnts_Parametrized:
            {
                theComputer.Init(theC);
                theComputer.Perform(theAbscis, theU0, theUi, theEPSILON);
                return;
            }
            case GCPnts_AbsComposite:
            {
                int aNbIntervals = theC.NbIntervals(GeomAbs_CN);
                NCollection_Array1<double> aTI = new(1, aNbIntervals + 1);
                theC.Intervals(aTI, GeomAbs_CN);
                double aL = 0.0, aSign = 1.0;
                int anIndex;
                BSplCLib.Hunt(aTI, theU0, out anIndex);
                int aDirection = 1;
                if (theAbscis < 0)
                {
                    aDirection = 0;
                    theAbscis = -theAbscis;
                    aSign = -1.0;
                }

                while (anIndex >= 1 && anIndex <= aNbIntervals)
                {
                    aL = CPnts_AbscissaPoint.Length(theC, theU0, aTI[anIndex + aDirection]);
                    if (Math.Abs(aL - theAbscis) <= Precision.Confusion())
                    {
                        theComputer.SetParameter(aTI[anIndex + aDirection]);
                        return;
                    }

                    if (aL > theAbscis)
                    {
                        if (theUi < aTI[anIndex] || theUi > aTI[anIndex + 1])
                        {
                            theUi = (theAbscis / aL) * (aTI[anIndex + 1] - theU0);
                            if (aDirection != 0)
                            {
                                theUi = theU0 + theUi;
                            }
                            else
                            {
                                theUi = theU0 - theUi;
                            }
                        }
                        theComputer.Init(theC, aTI[anIndex], aTI[anIndex + 1]);
                        theComputer.Perform(aSign * theAbscis, theU0, theUi, theEPSILON);
                        return;
                    }
                    else
                    {
                        theU0 = aTI[anIndex + aDirection];
                        theAbscis -= aL;
                    }
                    if (aDirection != 0)
                    {
                        ++anIndex;
                    }
                    else
                    {
                        --anIndex;
                    }
                }

                // Push a little bit outside the limits (hairy !!!)
                theUi = theU0 + 0.1;
                theComputer.Init(theC, theU0, theU0 + 0.2);
                theComputer.Perform(aSign * theAbscis, theU0, theUi, theEPSILON);
                return;
            }
        }
    }

    // Introduced by rbv for curvilinear parametrization
    // performs more appropriate tolerance management.
    private static void AdvCompute(CPnts_AbscissaPoint theComputer,
                                   Adaptor2d_Curve2d theC,
                                   ref double theAbscis,
                                   ref double theU0,
                                   ref double theUi,
                                   double theEPSILON)
    {
        double aRatio = 1.0;
        GCPnts_AbscissaType aType = computeType(theC, ref aRatio);
        switch (aType)
        {
            case GCPnts_LengthParametrized:
            {
                theComputer.SetParameter(theU0 + theAbscis / aRatio);
                return;
            }
            case GCPnts_Parametrized:
            {
                // theComputer.Init (theC);
                theComputer.Init(theC, theEPSILON); // rbv's modification
                theComputer.AdvPerform(theAbscis, theU0, theUi, theEPSILON);
                return;
            }
            case GCPnts_AbsComposite:
            {
                int aNbIntervals = theC.NbIntervals(GeomAbs_CN);
                NCollection_Array1<double> aTI = new(1, aNbIntervals + 1);
                theC.Intervals(aTI, GeomAbs_CN);
                double aL = 0.0, aSign = 1.0;
                int anIndex;
                BSplCLib.Hunt(aTI, theU0, out anIndex);

                int aDirection = 1;
                if (theAbscis < 0)
                {
                    aDirection = 0;
                    theAbscis = -theAbscis;
                    aSign = -1.0;
                }

                if (anIndex == 0 && aDirection > 0)
                {
                    aL = CPnts_AbscissaPoint.Length(theC, theU0, aTI[anIndex + aDirection], theEPSILON);
                    if (Math.Abs(aL - theAbscis) <= /*Precision::Confusion()*/ theEPSILON)
                    {
                        theComputer.SetParameter(aTI[anIndex + aDirection]);
                        return;
                    }

                    if (aL > theAbscis)
                    {
                        if (theUi > aTI[anIndex + 1])
                        {
                            theUi = (theAbscis / aL) * (aTI[anIndex + 1] - theU0);
                            theUi = theU0 + theUi;
                        }
                        theComputer.Init(theC, theU0, aTI[anIndex + 1], theEPSILON);
                        theComputer.AdvPerform(aSign * theAbscis, theU0, theUi, theEPSILON);
                        return;
                    }
                    else
                    {
                        theU0 = aTI[anIndex + aDirection];
                        theAbscis -= aL;
                    }
                    ++anIndex;
                }

                while (anIndex >= 1 && anIndex <= aNbIntervals)
                {
                    aL = CPnts_AbscissaPoint.Length(theC, theU0, aTI[anIndex + aDirection], theEPSILON);
                    if (Math.Abs(aL - theAbscis) <= Precision.PConfusion())
                    {
                        theComputer.SetParameter(aTI[anIndex + aDirection]);
                        return;
                    }

                    if (aL > theAbscis)
                    {
                        if (theUi < aTI[anIndex] || theUi > aTI[anIndex + 1])
                        {
                            theUi = (theAbscis / aL) * (aTI[anIndex + 1] - theU0);
                            if (aDirection != 0)
                            {
                                theUi = theU0 + theUi;
                            }
                            else
                            {
                                theUi = theU0 - theUi;
                            }
                        }
                        theComputer.Init(theC, aTI[anIndex], aTI[anIndex + 1], theEPSILON);
                        theComputer.AdvPerform(aSign * theAbscis, theU0, theUi, theEPSILON);
                        return;
                    }
                    else
                    {
                        theU0 = aTI[anIndex + aDirection];
                        theAbscis -= aL;
                    }
                    if (aDirection != 0)
                    {
                        ++anIndex;
                    }
                    else
                    {
                        --anIndex;
                    }
                }

                // Push a little bit outside the limits (hairy !!!)
                bool isNonPeriodic = !theC.IsPeriodic();
                theUi = theU0 + aSign * 0.1;
                double aU1 = theU0 + aSign * 0.2;
                if (isNonPeriodic)
                {
                    if (aSign > 0)
                    {
                        theUi = Min(theUi, theC.LastParameter());
                        aU1 = Min(aU1, theC.LastParameter());
                    }
                    else
                    {
                        theUi = Max(theUi, theC.FirstParameter());
                        aU1 = Max(aU1, theC.FirstParameter());
                    }
                }

                theComputer.Init(theC, theU0, aU1, theEPSILON);
                theComputer.AdvPerform(aSign * theAbscis, theU0, theUi, theEPSILON);
                return;
            }
        }
    }

    // theTol is null for no tolerance (OCCT's const double*).
    private static double length(Adaptor2d_Curve2d theC, double theU1, double theU2, double? theTol)
    {
        double aRatio = 1.0;
        GCPnts_AbscissaType aType = computeType(theC, ref aRatio);
        switch (aType)
        {
            case GCPnts_LengthParametrized:
            {
                return Math.Abs(theU2 - theU1) * aRatio;
            }
            case GCPnts_Parametrized:
            {
                return theTol != null ? CPnts_AbscissaPoint.Length(theC, theU1, theU2, theTol.Value)
                                      : CPnts_AbscissaPoint.Length(theC, theU1, theU2);
            }
            case GCPnts_AbsComposite:
            {
                int aNbIntervals = theC.NbIntervals(GeomAbs_CN);
                NCollection_Array1<double> aTI = new(1, aNbIntervals + 1);
                theC.Intervals(aTI, GeomAbs_CN);
                double aUU1 = Min(theU1, theU2);
                double aUU2 = Max(theU1, theU2);
                double aL = 0.0;
                for (int anIndex = 1; anIndex <= aNbIntervals; ++anIndex)
                {
                    if (aTI[anIndex] > aUU2)
                    {
                        break;
                    }
                    if (aTI[anIndex + 1] < aUU1)
                    {
                        continue;
                    }
                    if (theTol != null)
                    {
                        aL += CPnts_AbscissaPoint.Length(theC,
                                                         Max(aTI[anIndex], aUU1),
                                                         Min(aTI[anIndex + 1], aUU2),
                                                         theTol.Value);
                    }
                    else
                    {
                        aL += CPnts_AbscissaPoint.Length(theC,
                                                         Max(aTI[anIndex], aUU1),
                                                         Min(aTI[anIndex + 1], aUU2));
                    }
                }
                return aL;
            }
        }
        return RealLast();
    }

    private void compute(Adaptor2d_Curve2d theC, double theAbscissa, double theU0)
    {
        double aL = GCPnts_AbscissaPoint.Length(theC);
        if (aL < Precision.Confusion())
        {
            throw new Standard_ConstructionError();
        }

        double anAbscis = theAbscissa;
        double aUU0 = theU0;
        double aUUi = theU0 + (anAbscis / aL) * (theC.LastParameter() - theC.FirstParameter());
        Compute(myComputer, theC, ref anAbscis, ref aUU0, ref aUUi, theC.Resolution(Precision.Confusion()));
    }

    private void advCompute(double theTol, Adaptor2d_Curve2d theC, double theAbscissa, double theU0)
    {
        double aL = GCPnts_AbscissaPoint.Length(theC, theTol);
        /*if (aL < Precision::Confusion())
        {
          throw Standard_ConstructionError ("GCPnts_AbscissaPoint::GCPnts_AbscissaPoint");
        }*/
        double anAbscis = theAbscissa;
        double aUU0 = theU0;
        double aUUi = 0.0;
        if (aL >= Precision.Confusion())
        {
            aUUi = theU0 + (anAbscis / aL) * (theC.LastParameter() - theC.FirstParameter());
        }
        else
        {
            aUUi = theU0;
        }
        AdvCompute(myComputer, theC, ref anAbscis, ref aUU0, ref aUUi, theTol);
    }
}
