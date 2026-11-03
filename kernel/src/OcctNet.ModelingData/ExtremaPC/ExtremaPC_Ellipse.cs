// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_Ellipse.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Point-ellipse extrema: the point is projected onto the ellipse plane and the trigonometric equation
/// (B² - A²) cos u sin u - B Y cos u + A X sin u = 0 is solved by MathRoot.Trigonometric (up to 4 extrema).
/// </summary>
/// <remarks>When the point projects onto the centre of a circular ellipse (A = B): Status.InfiniteSolutions. The
/// domain is fixed at construction; a full period means the whole ellipse.</remarks>
public sealed class ExtremaPC_Ellipse : ExtremaPC.ICurveEvaluator
{
    private readonly Ellipse3D myEllipse;                    // Ellipse geometry
    private readonly global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D? myDomain;          // Parameter domain (null for full ellipse)
    private readonly ExtremaPC.Result myResult = new();     // Reusable result storage

    /// <summary>Constructor with ellipse geometry (full ellipse).</summary>
    public ExtremaPC_Ellipse(in Ellipse3D theEllipse)
    {
        myEllipse = theEllipse;
        myDomain = null;
    }

    /// <summary>Constructor with ellipse geometry and parameter domain in radians (fixed for all queries).</summary>
    public ExtremaPC_Ellipse(in Ellipse3D theEllipse, in global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D theDomain)
    {
        myEllipse = theEllipse;
        myDomain = theDomain.IsFullPeriod(2.0 * M_PI) ? null : theDomain;
    }

    /// <summary>Evaluates point on ellipse at parameter (radians).</summary>
    public Point3D Value(double theU) => global::Kerf.Engine.Geometry.ElCLib.Value(theU, myEllipse);

    /// <summary>Returns true if domain is bounded (partial arc).</summary>
    public bool IsBounded() => myDomain.HasValue;

    /// <summary>Returns the domain (only valid if IsBounded() is true).</summary>
    public global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D Domain() => myDomain!.Value;

    /// <summary>Computes the extrema between point P and the ellipse.</summary>
    public ExtremaPC.Result Perform(in Point3D theP, double theTol, ExtremaPC.SearchMode theMode = ExtremaPC.SearchMode.MinMax)
    {
        // Use stored domain or full parameter range [0, 2*PI]
        global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D aDomain = myDomain ?? new global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D(0.0, 2.0 * M_PI);
        performCore(theP, aDomain, theTol, theMode);
        return myResult;
    }

    /// <summary>Computes the extrema between point P and the ellipse arc including endpoints.</summary>
    public ExtremaPC.Result PerformWithEndpoints(in Point3D theP, double theTol, ExtremaPC.SearchMode theMode = ExtremaPC.SearchMode.MinMax)
    {
        _ = Perform(theP, theTol, theMode);

        // Add endpoints if interior computation succeeded and domain is bounded
        if (myResult.Status == ExtremaPC.Status.OK && myDomain.HasValue)
        {
            ExtremaPC.AddEndpointExtrema(myResult, theP, myDomain.Value, this, theTol, theMode);
        }

        return myResult;
    }

    /// <summary>Returns the ellipse geometry.</summary>
    public Ellipse3D Ellipse() => myEllipse;

    // Core algorithm - finds extrema with bounds checking. Stores results in myResult.
    private void performCore(in Point3D theP, in global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D theDomain, double theTol, ExtremaPC.SearchMode theMode)
    {
        myResult.Clear();

        double theUMin = theDomain.Min;
        double theUMax = theDomain.Max;

        // Step 1: Project point P onto the ellipse plane
        Point3D aCenter = myEllipse.Location();
        Direction3D aAxis = myEllipse.Axis().Direction();
        Vector3D aToP = new(aCenter, theP);
        double aHeight = aToP.Dot(new Vector3D(aAxis));
        Vector3D aTrsl = new Vector3D(aAxis) * (-aHeight);
        Point3D aPp = theP.Translated(aTrsl);

        // Step 2: Get ellipse radii and compute local coordinates
        double aA = myEllipse.MajorRadius();
        double aB = myEllipse.MinorRadius();

        Vector3D aOPp = new(aCenter, aPp);
        double aOPpMag = aOPp.Magnitude();

        // Check for degenerate case: point at center with circular ellipse
        if (aOPpMag < theTol)
        {
            if (Math.Abs(aA - aB) < theTol)
            {
                // Point at center of a circle - infinite solutions
                myResult.Status = ExtremaPC.Status.InfiniteSolutions;
                myResult.InfiniteSquareDistance = aA * aA + aHeight * aHeight;
                return;
            }
            // For non-circular ellipse at center, we still get valid extrema at semi-axes
        }

        // Local coordinates
        double aX = aOPp.Dot(new Vector3D(myEllipse.XAxis().Direction()));
        double aY = aOPp.Dot(new Vector3D(myEllipse.YAxis().Direction()));

        // Step 3: Solve trigonometric equation
        // (B^2 - A^2)*cos*sin - B*Y*cos + A*X*sin = 0
        // In MathRoot::Trigonometric form: a*cos^2 + 2*b*cos*sin + c*cos + d*sin + e = 0
        // a = 0, 2*b = (B^2 - A^2), c = -B*Y, d = A*X, e = 0
        double aKo2 = (aB * aB - aA * aA) / 2.0;
        double aKo3 = -aB * aY;
        double aKo4 = aA * aX;

        // MathRoot::Trigonometric handles all special cases including Y ~= 0
        global::Kerf.Engine.Numerics.Roots.MathRoot.TrigResult aTrigRes =
          global::Kerf.Engine.Numerics.Roots.MathRoot.Trigonometric(0.0, aKo2, aKo3, aKo4, 0.0, theUMin, theUMax);

        if (!aTrigRes.IsDone)
        {
            if (aTrigRes.InfiniteRoots)
            {
                myResult.Status = ExtremaPC.Status.InfiniteSolutions;
                // For infinite case, compute distance to any point (use U=0)
                Point3D aPtOnCurve = global::Kerf.Engine.Geometry.ElCLib.Value(0.0, myEllipse);
                myResult.InfiniteSquareDistance = theP.SquareDistance(aPtOnCurve);
            }
            else
            {
                myResult.Status = ExtremaPC.Status.NumericalError;
            }
            return;
        }

        // Step 4: Collect extrema
        double aTol2 = theTol * theTol;

        // Add solutions from solver
        for (int i = 0; i < aTrigRes.NbRoots; ++i)
        {
            addExtremum(aTrigRes.Roots[i], theP, theUMin, theUMax, theTol, aTol2, theMode);
        }

        myResult.Status = ExtremaPC.Status.OK;
    }

    // performCore's lambda addExtremum
    private void addExtremum(double aU, in Point3D theP, double theUMin, double theUMax, double theTol, double aTol2,
                             ExtremaPC.SearchMode theMode)
    {
        Point3D aCurvePt = global::Kerf.Engine.Geometry.ElCLib.Value(aU, myEllipse);

        // Check for duplicates using parameter proximity (more robust)
        for (int j = 0; j < myResult.Extrema.Length(); ++j)
        {
            if (Math.Abs(myResult.Extrema.Value(j).Parameter - aU) < theTol)
            {
                return;
            }
            if (aCurvePt.SquareDistance(myResult.Extrema.Value(j).Point) < aTol2)
            {
                return;
            }
        }

        double aSqDist = theP.SquareDistance(aCurvePt);

        // Determine if this is a minimum or maximum by checking neighboring points
        // Use step relative to parameter range
        double aStep = Max(ExtremaPC.THE_NEIGHBOR_STEP_RATIO,
                           (theUMax - theUMin) * ExtremaPC.THE_NEIGHBOR_STEP_RATIO);
        Point3D aPtPlus = global::Kerf.Engine.Geometry.ElCLib.Value(aU + aStep, myEllipse);
        Point3D aPtMinus = global::Kerf.Engine.Geometry.ElCLib.Value(aU - aStep, myEllipse);
        double aDistPlus = theP.SquareDistance(aPtPlus);
        double aDistMinus = theP.SquareDistance(aPtMinus);
        bool aIsMin = (aSqDist <= aDistPlus) && (aSqDist <= aDistMinus);

        // Filter by search mode
        if (theMode == ExtremaPC.SearchMode.Min && !aIsMin)
        {
            return;
        }
        if (theMode == ExtremaPC.SearchMode.Max && aIsMin)
        {
            return;
        }

        ExtremaPC.ExtremumResult anExt = new();
        anExt.Parameter = aU;
        anExt.Point = aCurvePt;
        anExt.SquareDistance = aSqDist;
        anExt.IsMinimum = aIsMin;

        myResult.Extrema.Append(anExt);
    }
}
