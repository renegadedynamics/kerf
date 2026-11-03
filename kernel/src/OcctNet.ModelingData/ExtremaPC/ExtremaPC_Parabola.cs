// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_Parabola.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Point-parabola extrema: the point is projected onto the parabola plane and the cubic
/// (1/4F) u³ + (2F - X) u - 2F Y = 0 is solved by MathPoly.Cubic.
/// </summary>
/// <remarks>The domain is fixed at construction; without one (or with an infinite one) the parabola is
/// unbounded.</remarks>
public sealed class ExtremaPC_Parabola : ExtremaPC.ICurveEvaluator
{
    private readonly Parabola3D myParabola;                   // Parabola geometry
    private readonly global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D? myDomain;          // Parameter domain (null for infinite)
    private readonly ExtremaPC.Result myResult = new();     // Reusable result storage

    // Cached geometry components for fast computation
    private double myVertexX, myVertexY, myVertexZ; // Vertex location
    private double myXDirX, myXDirY, myXDirZ;       // X-axis direction
    private double myYDirX, myYDirY, myYDirZ;       // Y-axis direction
    private double myAxisX, myAxisY, myAxisZ;       // Main axis direction
    private double myFocal;                         // Focal length
    private double my1Over4F;                       // Precomputed 1/(4*F)
    private double my2F;                            // Precomputed 2*F

    /// <summary>Constructor with parabola geometry (unbounded).</summary>
    public ExtremaPC_Parabola(in Parabola3D theParabola)
    {
        myParabola = theParabola;
        myDomain = null;
        cacheGeometry();
    }

    /// <summary>Constructor with parabola geometry and parameter domain (fixed for all queries).</summary>
    public ExtremaPC_Parabola(in Parabola3D theParabola, in global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D theDomain)
    {
        myParabola = theParabola;
        myDomain = theDomain.IsFinite() ? theDomain : null;
        cacheGeometry();
    }

    /// <summary>Evaluates point on parabola at parameter.</summary>
    public Point3D Value(double theU)
    {
        // Parabola: P(u) = Vertex + (u^2/4F)*XDir + u*YDir
        double aX = theU * theU * my1Over4F;
        return new Point3D(myVertexX + aX * myXDirX + theU * myYDirX,
                          myVertexY + aX * myXDirY + theU * myYDirY,
                          myVertexZ + aX * myXDirZ + theU * myYDirZ);
    }

    /// <summary>Returns true if domain is bounded.</summary>
    public bool IsBounded() => myDomain.HasValue;

    /// <summary>Returns the domain (only valid if IsBounded() is true).</summary>
    public global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D Domain() => myDomain!.Value;

    /// <summary>Computes the extrema between point P and the parabola.</summary>
    public ExtremaPC.Result Perform(in Point3D theP, double theTol, ExtremaPC.SearchMode theMode = ExtremaPC.SearchMode.MinMax)
    {
        performCore(theP, myDomain, theTol, theMode);
        return myResult;
    }

    /// <summary>Computes the extrema between point P and the parabola including endpoints.</summary>
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

    /// <summary>Returns the parabola geometry.</summary>
    public Parabola3D Parabola() => myParabola;

    private void cacheGeometry()
    {
        Point3D aVertex = myParabola.Location();
        myVertexX = aVertex.X();
        myVertexY = aVertex.Y();
        myVertexZ = aVertex.Z();

        Direction3D aXDir = myParabola.XAxis().Direction();
        myXDirX = aXDir.X();
        myXDirY = aXDir.Y();
        myXDirZ = aXDir.Z();

        Direction3D aYDir = myParabola.YAxis().Direction();
        myYDirX = aYDir.X();
        myYDirY = aYDir.Y();
        myYDirZ = aYDir.Z();

        Direction3D aAxis = myParabola.Axis().Direction();
        myAxisX = aAxis.X();
        myAxisY = aAxis.Y();
        myAxisZ = aAxis.Z();

        myFocal = myParabola.Focal();
        my1Over4F = 1.0 / (4.0 * myFocal);
        my2F = 2.0 * myFocal;
    }

    private global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult solveCubic(in Point3D theP)
    {
        // Vector from vertex to point
        double aDx = theP.X() - myVertexX;
        double aDy = theP.Y() - myVertexY;
        double aDz = theP.Z() - myVertexZ;

        // Project point P onto the parabola plane
        // Height = (P - Vertex) . Axis
        double aHeight = aDx * myAxisX + aDy * myAxisY + aDz * myAxisZ;

        // Projected point Pp = P - Height * Axis
        double aPpX = theP.X() - aHeight * myAxisX;
        double aPpY = theP.Y() - aHeight * myAxisY;
        double aPpZ = theP.Z() - aHeight * myAxisZ;

        // Vector from vertex to projected point
        double aOPpX = aPpX - myVertexX;
        double aOPpY = aPpY - myVertexY;
        double aOPpZ = aPpZ - myVertexZ;

        // Local coordinates in parabola frame
        double aX = aOPpX * myXDirX + aOPpY * myXDirY + aOPpZ * myXDirZ;
        double aY = aOPpX * myYDirX + aOPpY * myYDirY + aOPpZ * myYDirZ;

        // Solve cubic equation: (1/(4F)) * u^3 + (2F - X) * u - 2F*Y = 0
        return global::Kerf.Engine.Numerics.Polynomials.MathPoly.Cubic(my1Over4F, 0.0, my2F - aX, -my2F * aY);
    }

    private void performCore(in Point3D theP, global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D? theDomain, double theTol, ExtremaPC.SearchMode theMode)
    {
        _ = theTol; // Tolerance used for endpoint detection

        myResult.Clear();

        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aPolyRes = solveCubic(theP);

        if (!aPolyRes.IsDone)
        {
            if (aPolyRes.Status == global::Kerf.Engine.Numerics.Utilities.MathUtils.Status.InfiniteSolutions)
            {
                myResult.Status = ExtremaPC.Status.InfiniteSolutions;
                Point3D aPtOnCurve = Value(0.0);
                myResult.InfiniteSquareDistance = theP.SquareDistance(aPtOnCurve);
            }
            else
            {
                myResult.Status = ExtremaPC.Status.NumericalError;
            }
            return;
        }

        double aTol2 = global::Kerf.Engine.Foundation.Precision.SquareConfusion();

        // Process all roots
        for (int i = 0; i < aPolyRes.NbRoots; ++i)
        {
            double aU = aPolyRes.Roots[i];

            // Check bounds if domain is specified
            if (theDomain.HasValue)
            {
                if (aU < theDomain.Value.Min || aU > theDomain.Value.Max)
                    continue;
            }

            Point3D aCurvePt = Value(aU);

            // Check for duplicates
            bool aDuplicate = false;
            for (int j = 0; j < myResult.Extrema.Length(); ++j)
            {
                if (aCurvePt.SquareDistance(myResult.Extrema.Value(j).Point) < aTol2)
                {
                    aDuplicate = true;
                    break;
                }
            }
            if (aDuplicate)
                continue;

            // Determine if minimum or maximum using neighbor comparison
            double aSqDist = theP.SquareDistance(aCurvePt);
            double aNeighborU;
            if (theDomain.HasValue)
            {
                // For bounded case, choose neighbor direction based on position in domain
                aNeighborU = aU + (aU < (theDomain.Value.Min + theDomain.Value.Max) * 0.5 ? 1.0 : -1.0);
                aNeighborU = Max(theDomain.Value.Min, Min(theDomain.Value.Max, aNeighborU));
            }
            else
            {
                // For unbounded case, just use +1.0
                aNeighborU = aU + 1.0;
            }
            Point3D aNeighborPt = Value(aNeighborU);
            double aNeighborDist = theP.SquareDistance(aNeighborPt);
            bool aIsMin = aSqDist <= aNeighborDist;

            // Filter by search mode
            if (theMode == ExtremaPC.SearchMode.Min && !aIsMin)
                continue;
            if (theMode == ExtremaPC.SearchMode.Max && aIsMin)
                continue;

            ExtremaPC.ExtremumResult anExt = new();
            anExt.Parameter = aU;
            anExt.Point = aCurvePt;
            anExt.SquareDistance = aSqDist;
            anExt.IsMinimum = aIsMin;

            myResult.Extrema.Append(anExt);
        }

        myResult.Status = ExtremaPC.Status.OK;
    }
}
