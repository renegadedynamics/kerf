// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_QuasiUniformAbscissa.hxx
// Port of: src/ModelingData/TKGeomBase/GCPnts/GCPnts_QuasiUniformAbscissa.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using static Kerf.Geometry.CurveKind;

namespace Occt;

/// <summary>
/// Computes a given number of points on a curve spaced at an approximately uniform curvilinear distance. Bezier and
/// B-spline curves are divided along a polygonal approximation of their length; other curves go through
/// GCPnts_UniformAbscissa.
/// </summary>
/// <remarks>
/// A working object (decision 21): <c>Initialize</c> recomputes. The templates over the curve type are ported per
/// instantiation (decision 187).
/// </remarks>
public sealed class GCPnts_QuasiUniformAbscissa
{
    private bool myDone;
    private int myNbPoints;
    private NCollection_HArray1<double>? myParams;

    /// <summary>An empty algorithm; use <c>Initialize</c>.</summary>
    public GCPnts_QuasiUniformAbscissa()
    {
        myDone = false;
        myNbPoints = 0;
    }

    /// <summary>theNbPoints points on the curve at an approximately uniform distance.</summary>
    public GCPnts_QuasiUniformAbscissa(Adaptor3d_Curve theC, int theNbPoints)
        : this()
    {
        Initialize(theC, theNbPoints);
    }

    /// <summary>
    /// theNbPoints points on the part theU1..theU2 of the curve at an approximately uniform distance; the first and
    /// last are the points of theU1 and theU2.
    /// </summary>
    public GCPnts_QuasiUniformAbscissa(Adaptor3d_Curve theC, int theNbPoints, double theU1, double theU2)
        : this()
    {
        Initialize(theC, theNbPoints, theU1, theU2);
    }

    /// <summary>theNbPoints points on the curve at an approximately uniform distance.</summary>
    public GCPnts_QuasiUniformAbscissa(Adaptor2d_Curve2d theC, int theNbPoints)
        : this()
    {
        Initialize(theC, theNbPoints);
    }

    /// <summary>theNbPoints points on the part theU1..theU2 of the curve at an approximately uniform distance.</summary>
    public GCPnts_QuasiUniformAbscissa(Adaptor2d_Curve2d theC, int theNbPoints, double theU1, double theU2)
        : this()
    {
        Initialize(theC, theNbPoints, theU1, theU2);
    }

    /// <summary>Recomputes theNbPoints points on the curve.</summary>
    public void Initialize(Adaptor3d_Curve theC, int theNbPoints)
    {
        Initialize(theC, theNbPoints, theC.FirstParameter(), theC.LastParameter());
    }

    /// <summary>Recomputes theNbPoints points on the curve.</summary>
    public void Initialize(Adaptor2d_Curve2d theC, int theNbPoints)
    {
        Initialize(theC, theNbPoints, theC.FirstParameter(), theC.LastParameter());
    }

    /// <summary>Recomputes theNbPoints points on the part theU1..theU2 of the curve.</summary>
    public void Initialize(Adaptor3d_Curve theC, int theNbPoints, double theU1, double theU2)
    {
        initialize(theC, theNbPoints, theU1, theU2);
    }

    /// <summary>Recomputes theNbPoints points on the part theU1..theU2 of the curve.</summary>
    public void Initialize(Adaptor2d_Curve2d theC, int theNbPoints, double theU1, double theU2)
    {
        initialize(theC, theNbPoints, theU1, theU2);
    }

    /// <summary>True if the computation was successful.</summary>
    public bool IsDone() => myDone;

    /// <summary>The number of points computed.</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public int NbPoints()
    {
        StdFail_NotDone_Raise_if(!myDone, "GCPnts_QuasiUniformAbscissa::NbPoints()");
        return myNbPoints;
    }

    /// <summary>The parameter of the point of index Index (1 to NbPoints).</summary>
    /// <exception cref="NotDoneException">The computation failed.</exception>
    public double Parameter(int Index)
    {
        StdFail_NotDone_Raise_if(!myDone, "GCPnts_QuasiUniformAbscissa::Parameter()");
        return myParams!.Value(Index);
    }

    // ---- Templates over the curve type (3D instantiation) ----

    // This function divides given curve on the several parts with equal length.
    // It returns array of parameters in the control points.
    private void initialize(Adaptor3d_Curve theC, int theNbPoints, double theU1, double theU2)
    {
        if (theC.GetType() != BezierCurve && theC.GetType() != BSplineCurve)
        {
            GCPnts_UniformAbscissa aUA = new(theC, theNbPoints, theU1, theU2);
            myDone = aUA.IsDone();
            // OCCT reads NbPoints() and Parameter() without checking IsDone(), whose checks its release build
            // compiles out: a failed result is copied as it is (decision 189).
            myNbPoints = aUA.NbPointsUnchecked();
            myParams = new NCollection_HArray1<double>(1, myNbPoints);
            for (int aPntIter = 1; aPntIter <= myNbPoints; ++aPntIter)
            {
                myParams.SetValue(aPntIter, aUA.ParameterUnchecked(aPntIter));
            }
            return;
        }

        Standard_ConstructionError_Raise_if(
            theNbPoints <= 1,
            "GCPnts_QuasiUniformAbscissa::Initialize(), number of points should be >= 2");

        // evaluate the approximative length of the 3dCurve
        myNbPoints = theNbPoints;
        double aLength = 0.0;
        double dU = (theU2 - theU1) / (2 * theNbPoints - 1);

        NCollection_Array1<Point2D> aLP = new(1, 2 * theNbPoints); // table Length <-> Param
        Point3D aP1, aP2;
        aP1 = theC.Value(theU1);

        // Sum all distances
        for (int i = 0; i < 2 * theNbPoints; ++i)
        {
            aP2 = theC.Value(theU1 + i * dU);
            double aDist = aP1.Distance(aP2);
            aLength += aDist;
            aLP[i + 1] = new Point2D(aLength, theU1 + (i * dU));
            aP1 = aP2;
        }

        // Try to distribute NbPoints on the curve.
        // Place points approximately at intervals of Length/NbPoints.
        if (IsEqual(aLength, 0.0))
        { // use usual analytical grid
            double aStep = (theU2 - theU1) / (theNbPoints - 1);
            myParams = new NCollection_HArray1<double>(1, theNbPoints);
            myParams.SetValue(1, theU1);
            for (int i = 2; i < theNbPoints; ++i)
            {
                myParams.SetValue(i, theU1 + aStep * (i - 1));
            }
        }
        else
        {
            double aDCorde = aLength / (theNbPoints - 1);
            double aCorde = aDCorde;
            int anIndex = 1;
            myParams = new NCollection_HArray1<double>(1, theNbPoints);
            myParams.SetValue(1, theU1);
            for (int i = 2; i < theNbPoints; ++i)
            {
                while (aLP[anIndex].X() < aCorde)
                {
                    ++anIndex;
                }
                double anAlpha = (aCorde - aLP[anIndex - 1].X()) / (aLP[anIndex].X() - aLP[anIndex - 1].X());
                double aU = aLP[anIndex - 1].Y() + anAlpha * (aLP[anIndex].Y() - aLP[anIndex - 1].Y());
                myParams.SetValue(i, aU);
                aCorde = i * aDCorde;
            }
        }
        myParams.SetValue(theNbPoints, theU2);
        myDone = true;
    }

    // ---- Templates over the curve type (2D instantiation) ----

    // This function divides given curve on the several parts with equal length.
    // It returns array of parameters in the control points.
    private void initialize(Adaptor2d_Curve2d theC, int theNbPoints, double theU1, double theU2)
    {
        if (theC.GetType() != BezierCurve && theC.GetType() != BSplineCurve)
        {
            GCPnts_UniformAbscissa aUA = new(theC, theNbPoints, theU1, theU2);
            myDone = aUA.IsDone();
            // OCCT reads NbPoints() and Parameter() without checking IsDone(), whose checks its release build
            // compiles out: a failed result is copied as it is (decision 189).
            myNbPoints = aUA.NbPointsUnchecked();
            myParams = new NCollection_HArray1<double>(1, myNbPoints);
            for (int aPntIter = 1; aPntIter <= myNbPoints; ++aPntIter)
            {
                myParams.SetValue(aPntIter, aUA.ParameterUnchecked(aPntIter));
            }
            return;
        }

        Standard_ConstructionError_Raise_if(
            theNbPoints <= 1,
            "GCPnts_QuasiUniformAbscissa::Initialize(), number of points should be >= 2");

        // evaluate the approximative length of the 3dCurve
        myNbPoints = theNbPoints;
        double aLength = 0.0;
        double dU = (theU2 - theU1) / (2 * theNbPoints - 1);

        NCollection_Array1<Point2D> aLP = new(1, 2 * theNbPoints); // table Length <-> Param
        Point2D aP1, aP2;
        aP1 = theC.Value(theU1);

        // Sum all distances
        for (int i = 0; i < 2 * theNbPoints; ++i)
        {
            aP2 = theC.Value(theU1 + i * dU);
            double aDist = aP1.Distance(aP2);
            aLength += aDist;
            aLP[i + 1] = new Point2D(aLength, theU1 + (i * dU));
            aP1 = aP2;
        }

        // Try to distribute NbPoints on the curve.
        // Place points approximately at intervals of Length/NbPoints.
        if (IsEqual(aLength, 0.0))
        { // use usual analytical grid
            double aStep = (theU2 - theU1) / (theNbPoints - 1);
            myParams = new NCollection_HArray1<double>(1, theNbPoints);
            myParams.SetValue(1, theU1);
            for (int i = 2; i < theNbPoints; ++i)
            {
                myParams.SetValue(i, theU1 + aStep * (i - 1));
            }
        }
        else
        {
            double aDCorde = aLength / (theNbPoints - 1);
            double aCorde = aDCorde;
            int anIndex = 1;
            myParams = new NCollection_HArray1<double>(1, theNbPoints);
            myParams.SetValue(1, theU1);
            for (int i = 2; i < theNbPoints; ++i)
            {
                while (aLP[anIndex].X() < aCorde)
                {
                    ++anIndex;
                }
                double anAlpha = (aCorde - aLP[anIndex - 1].X()) / (aLP[anIndex].X() - aLP[anIndex - 1].X());
                double aU = aLP[anIndex - 1].Y() + anAlpha * (aLP[anIndex].Y() - aLP[anIndex - 1].Y());
                myParams.SetValue(i, aU);
                aCorde = i * aDCorde;
            }
        }
        myParams.SetValue(theNbPoints, theU2);
        myDone = true;
    }
}
