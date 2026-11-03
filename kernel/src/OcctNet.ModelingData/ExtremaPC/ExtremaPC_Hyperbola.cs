// Port of: src/ModelingData/TKGeomBase/ExtremaPC/ExtremaPC_Hyperbola.hxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2025 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

namespace Occt;

/// <summary>
/// Point-hyperbola extrema: the point is projected onto the hyperbola plane and a quartic in v = e^u is solved
/// by MathPoly.Quartic; positive roots give u = ln(v).
/// </summary>
/// <remarks>The domain is fixed at construction; without one (or with an infinite one) the hyperbola branch is
/// unbounded.</remarks>
public sealed class ExtremaPC_Hyperbola : ExtremaPC.ICurveEvaluator
{
    private readonly Hyperbola3D myHyperbola;                   // Hyperbola geometry
    private readonly global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D? myDomain;          // Parameter domain (null for infinite)
    private readonly ExtremaPC.Result myResult = new();     // Reusable result storage

    // Cached geometry components for fast computation
    private double myCenterX, myCenterY, myCenterZ; // Center location
    private double myXDirX, myXDirY, myXDirZ;       // X-axis direction
    private double myYDirX, myYDirY, myYDirZ;       // Y-axis direction
    private double myAxisX, myAxisY, myAxisZ;       // Main axis direction
    private double myMajorR;                        // Major radius
    private double myMinorR;                        // Minor radius
    private double myR2PlusR2Over4;                 // Precomputed (R^2 + r^2)/4

    /// <summary>Constructor with hyperbola geometry (unbounded).</summary>
    public ExtremaPC_Hyperbola(in Hyperbola3D theHyperbola)
    {
        myHyperbola = theHyperbola;
        myDomain = null;
        cacheGeometry();
    }

    /// <summary>Constructor with hyperbola geometry and parameter domain (fixed for all queries).</summary>
    public ExtremaPC_Hyperbola(in Hyperbola3D theHyperbola, in global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D theDomain)
    {
        myHyperbola = theHyperbola;
        myDomain = theDomain.IsFinite() ? theDomain : null;
        cacheGeometry();
    }

    /// <summary>Evaluates point on hyperbola at parameter.</summary>
    public Point3D Value(double theU)
    {
        // Hyperbola: P(u) = Center + R*cosh(u)*XDir + r*sinh(u)*YDir
        double aCosh = Math.Cosh(theU);
        double aSinh = Math.Sinh(theU);
        double aRCosh = myMajorR * aCosh;
        double arSinh = myMinorR * aSinh;
        return new Point3D(myCenterX + aRCosh * myXDirX + arSinh * myYDirX,
                          myCenterY + aRCosh * myXDirY + arSinh * myYDirY,
                          myCenterZ + aRCosh * myXDirZ + arSinh * myYDirZ);
    }

    /// <summary>Returns true if domain is bounded.</summary>
    public bool IsBounded() => myDomain.HasValue;

    /// <summary>Returns the domain (only valid if IsBounded() is true).</summary>
    public global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D Domain() => myDomain!.Value;

    /// <summary>Computes the extrema between point P and the hyperbola.</summary>
    public ExtremaPC.Result Perform(in Point3D theP, double theTol, ExtremaPC.SearchMode theMode = ExtremaPC.SearchMode.MinMax)
    {
        performCore(theP, myDomain, theTol, theMode);
        return myResult;
    }

    /// <summary>Computes the extrema between point P and the hyperbola including endpoints.</summary>
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

    /// <summary>Returns the hyperbola geometry.</summary>
    public Hyperbola3D Hyperbola() => myHyperbola;

    private void cacheGeometry()
    {
        Point3D aCenter = myHyperbola.Location();
        myCenterX = aCenter.X();
        myCenterY = aCenter.Y();
        myCenterZ = aCenter.Z();

        Direction3D aXDir = myHyperbola.XAxis().Direction();
        myXDirX = aXDir.X();
        myXDirY = aXDir.Y();
        myXDirZ = aXDir.Z();

        Direction3D aYDir = myHyperbola.YAxis().Direction();
        myYDirX = aYDir.X();
        myYDirY = aYDir.Y();
        myYDirZ = aYDir.Z();

        Direction3D aAxis = myHyperbola.Axis().Direction();
        myAxisX = aAxis.X();
        myAxisY = aAxis.Y();
        myAxisZ = aAxis.Z();

        myMajorR = myHyperbola.MajorRadius();
        myMinorR = myHyperbola.MinorRadius();
        myR2PlusR2Over4 = (myMajorR * myMajorR + myMinorR * myMinorR) / 4.0;
    }

    private global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult solveQuartic(in Point3D theP)
    {
        // Vector from center to point
        double aDx = theP.X() - myCenterX;
        double aDy = theP.Y() - myCenterY;
        double aDz = theP.Z() - myCenterZ;

        // Project point P onto the hyperbola plane
        // Height = (P - Center) . Axis
        double aHeight = aDx * myAxisX + aDy * myAxisY + aDz * myAxisZ;

        // Projected point Pp = P - Height * Axis
        double aPpX = theP.X() - aHeight * myAxisX;
        double aPpY = theP.Y() - aHeight * myAxisY;
        double aPpZ = theP.Z() - aHeight * myAxisZ;

        // Vector from center to projected point
        double aOPpX = aPpX - myCenterX;
        double aOPpY = aPpY - myCenterY;
        double aOPpZ = aPpZ - myCenterZ;

        // Local coordinates in hyperbola frame
        double aX = aOPpX * myXDirX + aOPpY * myXDirY + aOPpZ * myXDirZ;
        double aY = aOPpX * myYDirX + aOPpY * myYDirY + aOPpZ * myYDirZ;

        // Solve quartic equation in v = e^u
        double aC1 = myR2PlusR2Over4;
        double aC2 = -(aX * myMajorR + aY * myMinorR) / 2.0;
        double aC3 = 0.0;
        double aC4 = (aX * myMajorR - aY * myMinorR) / 2.0;
        double aC5 = -myR2PlusR2Over4;

        return global::Kerf.Engine.Numerics.Polynomials.MathPoly.Quartic(aC1, aC2, aC3, aC4, aC5);
    }

    private void performCore(in Point3D theP, global::Kerf.Engine.Numerics.Utilities.MathUtils.Domain1D? theDomain, double theTol, ExtremaPC.SearchMode theMode)
    {
        myResult.Clear();

        global::Kerf.Engine.Numerics.Utilities.MathUtils.PolyResult aPolyRes = solveQuartic(theP);

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

        double aTol2 = theTol * theTol;

        // Process all positive roots (v > 0 required for u = ln(v))
        for (int i = 0; i < aPolyRes.NbRoots; ++i)
        {
            double aV = aPolyRes.Roots[i];
            if (aV <= 0.0)
                continue;

            double aU = Math.Log(aV);

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
            bool aIsMin = aSqDist < aNeighborDist;

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
