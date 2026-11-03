// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_UniformDeflection.hxx
// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_UniformDeflection.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Occt.GCPnts_DeflectionType;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// Computes points on a C2 curve such that the deflection between the curve and the polygon through the points
/// stays below a given value. Slower than GCPnts_QuasiUniformDeflection, but gives fewer points. 2D curves give
/// points with Z = 0.
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>Initialize</c> recomputes. The templates over the curve type are ported per
/// instantiation (decision 187).
/// </remarks>
public sealed class GCPnts_UniformDeflection
{
    private bool myDone;
    private double myDeflection;
    private readonly NCollection_Sequence<double> myParams = new();
    private readonly NCollection_Sequence<Point3D> myPoints = new();

    /// <summary>An empty algorithm; use <c>Initialize</c>.</summary>
    public GCPnts_UniformDeflection()
    {
        myDone = false;
        myDeflection = 0.0;
    }

    /// <summary>
    /// Points on the part theU1..theU2 of the curve with the deflection theDeflection. theWithControl checks the
    /// deflection of the result (slower), adding points where needed.
    /// </summary>
    public GCPnts_UniformDeflection(Adaptor3d_Curve theC, double theDeflection, double theU1, double theU2,
        bool theWithControl = true)
    {
        myDone = false;
        myDeflection = theDeflection;
        Initialize(theC, theDeflection, theU1, theU2, theWithControl);
    }

    /// <summary>Points on the curve with the deflection theDeflection.</summary>
    public GCPnts_UniformDeflection(Adaptor3d_Curve theC, double theDeflection, bool theWithControl = true)
    {
        myDone = false;
        myDeflection = theDeflection;
        Initialize(theC, theDeflection, theWithControl);
    }

    /// <summary>Points on the part theU1..theU2 of the curve with the deflection theDeflection.</summary>
    public GCPnts_UniformDeflection(Adaptor2d_Curve2d theC, double theDeflection, double theU1, double theU2,
        bool theWithControl = true)
    {
        myDone = false;
        myDeflection = theDeflection;
        Initialize(theC, theDeflection, theU1, theU2, theWithControl);
    }

    /// <summary>Points on the curve with the deflection theDeflection.</summary>
    public GCPnts_UniformDeflection(Adaptor2d_Curve2d theC, double theDeflection, bool theWithControl = true)
    {
        myDone = false;
        myDeflection = theDeflection;
        Initialize(theC, theDeflection, theWithControl);
    }

    /// <summary>Recomputes the points on the curve.</summary>
    public void Initialize(Adaptor3d_Curve theC, double theDeflection, bool theWithControl = true)
    {
        Initialize(theC, theDeflection, theC.FirstParameter(), theC.LastParameter(), theWithControl);
    }

    /// <summary>Recomputes the points on the curve.</summary>
    public void Initialize(Adaptor2d_Curve2d theC, double theDeflection, bool theWithControl = true)
    {
        Initialize(theC, theDeflection, theC.FirstParameter(), theC.LastParameter(), theWithControl);
    }

    /// <summary>
    /// Recomputes the points on the part theU1..theU2 of the curve (the bounds are swapped if theU1 &gt; theU2).
    /// </summary>
    public void Initialize(Adaptor3d_Curve theC, double theDeflection, double theU1, double theU2,
        bool theWithControl = true)
    {
        initialize(theC, theDeflection, theU1, theU2, theWithControl);
    }

    /// <summary>
    /// Recomputes the points on the part theU1..theU2 of the curve (the bounds are swapped if theU1 &gt; theU2).
    /// </summary>
    public void Initialize(Adaptor2d_Curve2d theC, double theDeflection, double theU1, double theU2,
        bool theWithControl = true)
    {
        initialize(theC, theDeflection, theU1, theU2, theWithControl);
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone() => myDone;

    /// <summary>The number of points computed.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public int NbPoints()
    {
        StdFail_NotDone_Raise_if(!myDone, "GCPnts_UniformDeflection::NbPoints()");
        return myParams.Length();
    }

    /// <summary>The parameter of the point of index Index (1 to NbPoints).</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double Parameter(int Index)
    {
        StdFail_NotDone_Raise_if(!myDone, "GCPnts_UniformDeflection::Parameter()");
        return myParams[Index];
    }

    /// <summary>The point of index theIndex (1 to NbPoints).</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public Point3D Value(int theIndex)
    {
        // OCCT names the wrong class and method in the message (occt-issues.md).
        StdFail_NotDone_Raise_if(!myDone, "GCPnts_UniformAbscissa::Parameter()");
        return myPoints.Value(theIndex);
    }

    /// <summary>The deflection given to the algorithm.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double Deflection()
    {
        StdFail_NotDone_Raise_if(!myDone, "GCPnts_UniformDeflection::Deflection()");
        return myDeflection;
    }

    // mask the return of a Adaptor2d_Curve2d as a gp_Pnt
    private static Point3D Value(Adaptor3d_Curve theC, double theParameter)
    {
        return theC.Value(theParameter);
    }

    private static Point3D Value(Adaptor2d_Curve2d theC, double theParameter)
    {
        Point2D a2dPoint = theC.Value(theParameter);
        return new Point3D(a2dPoint.X(), a2dPoint.Y(), 0.0);
    }

    // ---- Templates over the curve type (3D instantiation) ----

    // Control of the last points.
    private static void Controle(Adaptor3d_Curve theC,
                                 NCollection_Sequence<double> theParameters,
                                 NCollection_Sequence<Point3D> thePoints,
                                 double theU2)
    {
        int aNbPnts = thePoints.Length();
        if (aNbPnts > 2)
        {
            double aUa = theParameters[aNbPnts - 2];
            double aUb = theParameters[aNbPnts - 1];
            if (theU2 - aUb < 0.33 * (theU2 - aUa))
            {
                double aUc = (theU2 + aUa) * 0.5;
                theParameters[aNbPnts - 1] = aUc;
                thePoints[aNbPnts - 1] = Value(theC, aUc);
            }
        }
    }

    private static bool PerformLinear(Adaptor3d_Curve theC,
                                      NCollection_Sequence<double> theParameters,
                                      NCollection_Sequence<Point3D> thePoints,
                                      double theU1,
                                      double theU2)
    {
        theParameters.Append(theU1);
        Point3D aPoint = Value(theC, theU1);
        thePoints.Append(aPoint);

        theParameters.Append(theU2);
        aPoint = Value(theC, theU2);
        thePoints.Append(aPoint);
        return true;
    }

    private static bool PerformCircular(Adaptor3d_Curve theC,
                                        NCollection_Sequence<double> theParameters,
                                        NCollection_Sequence<Point3D> thePoints,
                                        double theDeflection,
                                        double theU1,
                                        double theU2)
    {
        Point3D aPoint;
        double anAngle = Max(1.0 - (theDeflection / theC.Circle().Radius()), 0.0);
        anAngle = 2.0e0 * Math.Acos(anAngle);
        int aNbPoints = (int)((theU2 - theU1) / anAngle);
        aNbPoints += 2;
        anAngle = (theU2 - theU1) / (double)(aNbPoints - 1);
        double aU = theU1;
        for (int i = 1; i <= aNbPoints; ++i)
        {
            theParameters.Append(aU);
            aPoint = Value(theC, aU);
            thePoints.Append(aPoint);
            aU += anAngle;
        }
        return true;
    }

    private static GCPnts_DeflectionType GetDefType(Adaptor3d_Curve theC)
    {
        if (theC.NbIntervals(C2) > 1)
        {
            return GCPnts_DefComposite;
        }

        switch (theC.GetType())
        {
            case Line:
                return GCPnts_Linear;
            case Circle:
                return GCPnts_Circular;
            case BSplineCurve:
            {
                Geom_BSplineCurve aBSpline = theC.BSpline();
                return (aBSpline.NbPoles() == 2) ? GCPnts_Linear : GCPnts_Curved;
            }
            case BezierCurve:
            {
                Geom_BezierCurve aBezier = theC.Bezier();
                return (aBezier.NbPoles() == 2) ? GCPnts_Linear : GCPnts_Curved;
            }
            default:
            {
                return GCPnts_Curved;
            }
        }
    }

    private static bool PerformCurve(NCollection_Sequence<double> theParameters,
                                     NCollection_Sequence<Point3D> thePoints,
                                     Adaptor3d_Curve theC,
                                     double theDeflection,
                                     double theU1,
                                     double theU2,
                                     double theEPSILON,
                                     bool theWithControl)
    {
        CPnts_UniformDeflection anIterator = new(theC, theDeflection, theU1, theU2, theEPSILON, theWithControl);
        for (; anIterator.More(); anIterator.Next())
        {
            theParameters.Append(anIterator.Value());
            thePoints.Append(anIterator.Point());
        }
        return anIterator.IsAllDone();
    }

    private static bool PerformComposite(NCollection_Sequence<double> theParameters,
                                         NCollection_Sequence<Point3D> thePoints,
                                         Adaptor3d_Curve theC,
                                         double theDeflection,
                                         double theU1,
                                         double theU2,
                                         double theEPSILON,
                                         bool theWithControl)
    {
        int aNbIntervals = theC.NbIntervals(C2);
        int aPIndex;
        NCollection_Array1<double> aTI = new(1, aNbIntervals + 1);
        theC.Intervals(aTI, C2);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(aTI, theU1, out aPIndex);

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
                              theWithControl))
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
        bool theWithControl)
    {
        double anEPSILON = theC.Resolution(global::Kerf.Engine.Foundation.Precision.Confusion());
        myDeflection = theDeflection;
        myDone = false;
        myParams.Clear();
        myPoints.Clear();

        double aU1 = Min(theU1, theU2);
        double aU2 = Max(theU1, theU2);
        GCPnts_DeflectionType aType = GetDefType(theC);
        switch (aType)
        {
            case GCPnts_Linear:
                myDone = PerformLinear(theC, myParams, myPoints, aU1, aU2);
                break;
            case GCPnts_Circular:
                myDone = PerformCircular(theC, myParams, myPoints, theDeflection, aU1, aU2);
                break;
            case GCPnts_Curved:
                myDone = PerformCurve(myParams, myPoints, theC, theDeflection, aU1, aU2, anEPSILON, theWithControl);
                break;
            case GCPnts_DefComposite:
                myDone = PerformComposite(myParams,
                                          myPoints,
                                          theC,
                                          theDeflection,
                                          aU1,
                                          aU2,
                                          anEPSILON,
                                          theWithControl);
                break;
        }

        // control of the last points
        Controle(theC, myParams, myPoints, aU2);
    }

    // ---- Templates over the curve type (2D instantiation) ----

    // Control of the last points.
    private static void Controle(Adaptor2d_Curve2d theC,
                                 NCollection_Sequence<double> theParameters,
                                 NCollection_Sequence<Point3D> thePoints,
                                 double theU2)
    {
        int aNbPnts = thePoints.Length();
        if (aNbPnts > 2)
        {
            double aUa = theParameters[aNbPnts - 2];
            double aUb = theParameters[aNbPnts - 1];
            if (theU2 - aUb < 0.33 * (theU2 - aUa))
            {
                double aUc = (theU2 + aUa) * 0.5;
                theParameters[aNbPnts - 1] = aUc;
                thePoints[aNbPnts - 1] = Value(theC, aUc);
            }
        }
    }

    private static bool PerformLinear(Adaptor2d_Curve2d theC,
                                      NCollection_Sequence<double> theParameters,
                                      NCollection_Sequence<Point3D> thePoints,
                                      double theU1,
                                      double theU2)
    {
        theParameters.Append(theU1);
        Point3D aPoint = Value(theC, theU1);
        thePoints.Append(aPoint);

        theParameters.Append(theU2);
        aPoint = Value(theC, theU2);
        thePoints.Append(aPoint);
        return true;
    }

    private static bool PerformCircular(Adaptor2d_Curve2d theC,
                                        NCollection_Sequence<double> theParameters,
                                        NCollection_Sequence<Point3D> thePoints,
                                        double theDeflection,
                                        double theU1,
                                        double theU2)
    {
        Point3D aPoint;
        double anAngle = Max(1.0 - (theDeflection / theC.Circle().Radius()), 0.0);
        anAngle = 2.0e0 * Math.Acos(anAngle);
        int aNbPoints = (int)((theU2 - theU1) / anAngle);
        aNbPoints += 2;
        anAngle = (theU2 - theU1) / (double)(aNbPoints - 1);
        double aU = theU1;
        for (int i = 1; i <= aNbPoints; ++i)
        {
            theParameters.Append(aU);
            aPoint = Value(theC, aU);
            thePoints.Append(aPoint);
            aU += anAngle;
        }
        return true;
    }

    private static GCPnts_DeflectionType GetDefType(Adaptor2d_Curve2d theC)
    {
        if (theC.NbIntervals(C2) > 1)
        {
            return GCPnts_DefComposite;
        }

        switch (theC.GetType())
        {
            case Line:
                return GCPnts_Linear;
            case Circle:
                return GCPnts_Circular;
            case BSplineCurve:
            {
                Geom2d_BSplineCurve aBSpline = theC.BSpline();
                return (aBSpline.NbPoles() == 2) ? GCPnts_Linear : GCPnts_Curved;
            }
            case BezierCurve:
            {
                Geom2d_BezierCurve aBezier = theC.Bezier();
                return (aBezier.NbPoles() == 2) ? GCPnts_Linear : GCPnts_Curved;
            }
            default:
            {
                return GCPnts_Curved;
            }
        }
    }

    private static bool PerformCurve(NCollection_Sequence<double> theParameters,
                                     NCollection_Sequence<Point3D> thePoints,
                                     Adaptor2d_Curve2d theC,
                                     double theDeflection,
                                     double theU1,
                                     double theU2,
                                     double theEPSILON,
                                     bool theWithControl)
    {
        CPnts_UniformDeflection anIterator = new(theC, theDeflection, theU1, theU2, theEPSILON, theWithControl);
        for (; anIterator.More(); anIterator.Next())
        {
            theParameters.Append(anIterator.Value());
            thePoints.Append(anIterator.Point());
        }
        return anIterator.IsAllDone();
    }

    private static bool PerformComposite(NCollection_Sequence<double> theParameters,
                                         NCollection_Sequence<Point3D> thePoints,
                                         Adaptor2d_Curve2d theC,
                                         double theDeflection,
                                         double theU1,
                                         double theU2,
                                         double theEPSILON,
                                         bool theWithControl)
    {
        int aNbIntervals = theC.NbIntervals(C2);
        int aPIndex;
        NCollection_Array1<double> aTI = new(1, aNbIntervals + 1);
        theC.Intervals(aTI, C2);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(aTI, theU1, out aPIndex);

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
                              theWithControl))
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
        bool theWithControl)
    {
        double anEPSILON = theC.Resolution(global::Kerf.Engine.Foundation.Precision.Confusion());
        myDeflection = theDeflection;
        myDone = false;
        myParams.Clear();
        myPoints.Clear();

        double aU1 = Min(theU1, theU2);
        double aU2 = Max(theU1, theU2);
        GCPnts_DeflectionType aType = GetDefType(theC);
        switch (aType)
        {
            case GCPnts_Linear:
                myDone = PerformLinear(theC, myParams, myPoints, aU1, aU2);
                break;
            case GCPnts_Circular:
                myDone = PerformCircular(theC, myParams, myPoints, theDeflection, aU1, aU2);
                break;
            case GCPnts_Curved:
                myDone = PerformCurve(myParams, myPoints, theC, theDeflection, aU1, aU2, anEPSILON, theWithControl);
                break;
            case GCPnts_DefComposite:
                myDone = PerformComposite(myParams,
                                          myPoints,
                                          theC,
                                          theDeflection,
                                          aU1,
                                          aU2,
                                          anEPSILON,
                                          theWithControl);
                break;
        }

        // control of the last points
        Controle(theC, myParams, myPoints, aU2);
    }
}
