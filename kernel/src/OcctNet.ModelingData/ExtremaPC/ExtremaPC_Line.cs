// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_Line.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Point-line extrema: the closest point by direct projection, u = (P - O) . D. The domain is fixed at
/// construction; without one (or with an infinite one) the line is unbounded.
/// </summary>
/// <remarks>Lines always have exactly one extremum (minimum) if within bounds. <c>Perform</c> returns the
/// evaluator's own reusable result, which the next call overwrites.</remarks>
public sealed class ExtremaPC_Line : ExtremaPC.ICurveEvaluator
{
    private readonly Line3D myLine;                         // Line geometry
    private readonly global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D? myDomain;          // Parameter domain (null for unbounded)
    private readonly ExtremaPC.Result myResult = new();     // Reusable result storage

    /// <summary>Constructor with line geometry (unbounded).</summary>
    public ExtremaPC_Line(in Line3D theLine)
    {
        myLine = theLine;
        myDomain = null;
    }

    /// <summary>Constructor with line geometry and parameter domain (fixed for all queries).</summary>
    public ExtremaPC_Line(in Line3D theLine, in global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D theDomain)
    {
        myLine = theLine;
        myDomain = theDomain.IsFinite() ? theDomain : null;
    }

    /// <summary>Evaluates point on line at parameter.</summary>
    public Point3D Value(double theU)
    {
        return myLine.Location().Translated(theU * new Vector3D(myLine.Direction()));
    }

    /// <summary>Returns true if domain is bounded.</summary>
    public bool IsBounded() => myDomain.HasValue;

    /// <summary>Returns the domain (only valid if IsBounded() is true).</summary>
    public global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D Domain() => myDomain!.Value;

    /// <summary>Computes the extremum between point P and the line (theMode is unused: always the minimum).</summary>
    public ExtremaPC.Result Perform(in Point3D theP, double theTol, ExtremaPC.SearchMode theMode = ExtremaPC.SearchMode.MinMax)
    {
        _ = theMode; // Lines always have exactly one extremum (minimum)

        myResult.Clear();

        // Compute projection parameter: u = (P - O) . Direction
        Direction3D aDir = myLine.Direction();
        Point3D aOrigin = myLine.Location();
        Vector3D aVec = new(aOrigin, theP);
        double aU = aVec.Dot(new Vector3D(aDir));

        // Check bounds if domain is specified
        if (myDomain.HasValue)
        {
            global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D aDomain = myDomain.Value;
            if (aU < aDomain.Min - theTol || aU > aDomain.Max + theTol)
            {
                // Projection is outside bounds - no interior extremum
                myResult.Status = ExtremaPC.Status.OK;
                return myResult;
            }
            // Clamp to bounds
            aU = aU < aDomain.Min ? aDomain.Min : (aDomain.Max < aU ? aDomain.Max : aU); // std::clamp
        }

        // Compute point on line at projected parameter
        Point3D aPtOnLine = aOrigin.Translated(aU * new Vector3D(aDir));

        // Store result
        ExtremaPC.ExtremumResult anExt = new();
        anExt.Parameter = aU;
        anExt.Point = aPtOnLine;
        anExt.SquareDistance = theP.SquareDistance(aPtOnLine);
        anExt.IsMinimum = true;

        myResult.Extrema.Append(anExt);
        myResult.Status = ExtremaPC.Status.OK;
        return myResult;
    }

    /// <summary>Computes the extrema between point P and the line segment including endpoints.</summary>
    public ExtremaPC.Result PerformWithEndpoints(in Point3D theP, double theTol, ExtremaPC.SearchMode theMode = ExtremaPC.SearchMode.MinMax)
    {
        _ = Perform(theP, theTol, theMode);

        if (myResult.Status == ExtremaPC.Status.OK && myDomain.HasValue)
        {
            ExtremaPC.AddEndpointExtrema(myResult, theP, myDomain.Value, this, theTol, theMode);
        }

        return myResult;
    }

    /// <summary>Returns the line geometry.</summary>
    public Line3D Line() => myLine;
}
