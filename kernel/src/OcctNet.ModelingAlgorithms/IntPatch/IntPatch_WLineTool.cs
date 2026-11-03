// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_WLineTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_WLineTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2014 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Occt.TopAbs_State;

namespace Occt;

/// <summary>
/// Tools for walking lines: purging (removing equal, out-of-domain and collinear points), joining lines that share an
/// end point (two cylinders), and extending lines towards each other across a common singular point.
/// </summary>
/// <remarks>
/// A static class. The lines given are edited in place, as in OCCT (decision 609). The file-static functions of OCCT's
/// .cxx are private statics (IntPatch_WLineTool.Extend.cs holds ExtendTwoWLines and its helpers).
/// </remarks>
public static partial class IntPatch_WLineTool
{
    /// <summary>The largest angle (an empirical value, π/6) between two lines' end tangents for them to be joined
    /// or extended.</summary>
    public static readonly double myMaxConcatAngle = M_PI / 6;

    private const int aMinNbBadDistr = 15;
    private const int aNbSingleBezier = 30;

    // Replaces theParMIN = MIN(theParMIN, theParMAX), theParMAX = MAX(theParMIN, theParMAX).
    private static void MinMax(ref double theParMIN, ref double theParMAX)
    {
        if (theParMIN > theParMAX)
        {
            double aTmp = theParMAX;
            theParMAX = theParMIN;
            theParMIN = aTmp;
        }
    }

    // Fill points hash by input data. 1 - delete point; 0 - store point; -1 - vertex point (not deleted).
    private static void FillPointsHash(IntPatch_WLine theWLine, NCollection_Array1<int> thePointsHash)
    {
        int i, v;

        for (i = 1; i <= theWLine.NbPnts(); i++)
        {
            thePointsHash.SetValue(i, 0);
        }

        for (v = 1; v <= theWLine.NbVertex(); v++)
        {
            IntPatch_Point aVertex = theWLine.Vertex(v);
            int avertexindex = (int)aVertex.ParameterOnLine();
            thePointsHash.SetValue(avertexindex, -1);
        }
    }

    // Makes new walking line according to the points hash.
    private static IntPatch_WLine MakeNewWLine(IntPatch_WLine theWLine, NCollection_Array1<int> thePointsHash,
                                               bool theIsOuter)
    {
        int i;

        IntSurf_LineOn2S aPurgedLineOn2S = new IntSurf_LineOn2S();
        IntPatch_WLine aLocalWLine = new IntPatch_WLine(aPurgedLineOn2S, false);
        aLocalWLine.SetCreatingWayInfo(theWLine.GetCreatingWay());
        int anOldLineIdx = 1, aVertexIdx = 1, anIndexPrev = -1, anIdxOld = -1;
        Point3D aPPrev = new Point3D(), aPOld = new Point3D();
        for (i = 1; i <= thePointsHash.Upper(); i++)
        {
            if (thePointsHash[i] == 0)
            {
                // Point has to be added

                Point3D aP = theWLine.Point(i).Value();
                double aSqDistPrev = aPPrev.SquareDistance(aPOld);
                double aSqDist = aPPrev.SquareDistance(aP);

                double aRatio = (aSqDistPrev < Constants.Resolution()) ? 0.0 : 9.0 * aSqDist / aSqDistPrev;

                if (theIsOuter || (aRatio < Constants.Resolution()) || ((1.0 < aRatio) && (aRatio < 81.0))
                    || (i - anIndexPrev <= 1) || (i - anIdxOld <= 1))
                {
                    // difference in distances is satisfactory
                    // (1/9 < aSqDist/aSqDistPrev < 9)

                    // Store this point.
                    aPurgedLineOn2S.Add(theWLine.Point(i));
                    anOldLineIdx++;
                    aPOld = aPPrev;
                    aPPrev = aP;
                    anIdxOld = anIndexPrev;
                    anIndexPrev = i;
                }
                else if (aSqDist >= aSqDistPrev * 9.0)
                {
                    // current segment is much more longer
                    // (aSqDist/aSqDistPrev >= 9)

                    i = (i + anIndexPrev) / 2;
                    thePointsHash[i] = 0;
                    i--;
                }
                else
                {
                    // previous segment is much more longer
                    //(aSqDist/aSqDistPrev <= 1/9)

                    if (anIndexPrev - anIdxOld > 1)
                    {
                        // Delete aPPrev from WL
                        aPurgedLineOn2S.RemovePoint(aPurgedLineOn2S.NbPoints());
                        anOldLineIdx--;

                        // Insert point between aPOld and aPPrev
                        i = (anIdxOld + anIndexPrev) / 2;
                        thePointsHash[i] = 0;

                        aPPrev = aPOld;
                        anIndexPrev = anIdxOld;
                    }
                    else
                    {
                        aPOld = aPPrev;
                        anIdxOld = anIndexPrev;
                    }

                    // Next iterations will start from this inserted point.
                    i--;
                }
            }
            else if (thePointsHash[i] == -1)
            {
                // Add vertex.
                IntPatch_Point aVertex = theWLine.Vertex(aVertexIdx++);
                aVertex.SetParameter(anOldLineIdx++);
                aLocalWLine.AddVertex(aVertex);
                aPurgedLineOn2S.Add(theWLine.Point(i));
                aPPrev = aPOld = theWLine.Point(i).Value();
                anIndexPrev = anIdxOld = i;
            }

            // Other points will be rejected by purger.
        }

        return aLocalWLine;
    }

    // Move point into surface param space. No interpolation used because walking algorithm should care for closeness
    // to the param space.
    private static void MovePoint(Adaptor3d_Surface theS1, ref double U1, ref double V1)
    {
        if (U1 < theS1.FirstUParameter())
        {
            U1 = theS1.FirstUParameter();
        }

        if (U1 > theS1.LastUParameter())
        {
            U1 = theS1.LastUParameter();
        }

        if (V1 < theS1.FirstVParameter())
        {
            V1 = theS1.FirstVParameter();
        }

        if (V1 > theS1.LastVParameter())
        {
            V1 = theS1.LastVParameter();
        }
    }

    // Check and delete out of bounds points on walking line.
    private static IntPatch_WLine DeleteOuterPoints(IntPatch_WLine theWLine, Adaptor3d_Surface theS1,
                                                    Adaptor3d_Surface theS2, Adaptor3d_TopolTool theDom1,
                                                    Adaptor3d_TopolTool theDom2)
    {
        int i;

        NCollection_Array1<int> aDelOuterPointsHash = new(1, theWLine.NbPnts());
        FillPointsHash(theWLine, aDelOuterPointsHash);

        if (theS1.IsUPeriodic() || theS1.IsVPeriodic() || theS2.IsUPeriodic() || theS2.IsVPeriodic())
        {
            return theWLine;
        }

        Point2D aPntOnF1 = new Point2D(), aPntOnF2 = new Point2D();
        double aX1, aY1, aX2, aY2;

        // Iterate over points in walking line and delete which are out of bounds.
        // Forward.
        bool isAllDeleted = true;
        bool aChangedFirst = false;
        int aFirstGeomIdx = 1;
        for (i = 1; i <= theWLine.NbPnts(); i++)
        {
            theWLine.Point(i).Parameters(out aX1, out aY1, out aX2, out aY2);
            aPntOnF1.SetCoord(aX1, aY1);
            aPntOnF2.SetCoord(aX2, aY2);

            TopAbs_State aState1 = theDom1.Classify(aPntOnF1, global::Kerf.Engine.Foundation.Precision.Confusion());
            TopAbs_State aState2 = theDom2.Classify(aPntOnF2, global::Kerf.Engine.Foundation.Precision.Confusion());

            if (aState1 == TopAbs_OUT || aState2 == TopAbs_OUT)
            {
                aDelOuterPointsHash[i] = 1;
                aChangedFirst = true;
            }
            else
            {
                isAllDeleted = false;

                aFirstGeomIdx = Max(i - 1, 1);
                if (aDelOuterPointsHash[i] == -1)
                {
                    aFirstGeomIdx = i; // Use data what lies in (i) point / vertex.
                }

                aDelOuterPointsHash[i] = -1;
                break;
            }
        }

        if (isAllDeleted)
        {
            // ALL points are out of bounds:
            // case boolean bcut_complex F5 and similar.
            return theWLine;
        }

        // Backward.
        bool aChangedLast = false;
        int aLastGeomIdx = theWLine.NbPnts();
        for (i = theWLine.NbPnts(); i >= 1; i--)
        {
            theWLine.Point(i).Parameters(out aX1, out aY1, out aX2, out aY2);
            aPntOnF1.SetCoord(aX1, aY1);
            aPntOnF2.SetCoord(aX2, aY2);

            TopAbs_State aState1 = theDom1.Classify(aPntOnF1, global::Kerf.Engine.Foundation.Precision.Confusion());
            TopAbs_State aState2 = theDom2.Classify(aPntOnF2, global::Kerf.Engine.Foundation.Precision.Confusion());

            if (aState1 == TopAbs_OUT || aState2 == TopAbs_OUT)
            {
                aDelOuterPointsHash[i] = 1;
                aChangedLast = true; // Move vertex to first good point
            }
            else
            {
                aLastGeomIdx = Min(i + 1, theWLine.NbPnts());
                if (aDelOuterPointsHash[i] == -1)
                {
                    aLastGeomIdx = i; // Use data what lies in (i) point / vertex.
                }

                aDelOuterPointsHash[i] = -1;
                break;
            }
        }

        if (!aChangedFirst && !aChangedLast)
        {
            // Nothing is done, return input.
            return theWLine;
        }

        // Build new line and modify geometry of necessary vertices.
        IntPatch_WLine aLocalWLine = MakeNewWLine(theWLine, aDelOuterPointsHash, true);

        if (aChangedFirst)
        {
            // Vertex geometry.
            IntPatch_Point aVertex = aLocalWLine.Vertex(1);
            aVertex.SetValue(theWLine.Point(aFirstGeomIdx).Value());
            theWLine.Point(aFirstGeomIdx).Parameters(out double aU1, out double aV1, out double aU2, out double aV2);
            MovePoint(theS1, ref aU1, ref aV1);
            MovePoint(theS2, ref aU2, ref aV2);
            aVertex.SetParameters(aU1, aV1, aU2, aV2);
            aLocalWLine.Replace(1, aVertex);
            // Change point in walking line.
            aLocalWLine.SetPoint(1, aVertex);
        }

        if (aChangedLast)
        {
            // Vertex geometry.
            IntPatch_Point aVertex = aLocalWLine.Vertex(aLocalWLine.NbVertex());
            aVertex.SetValue(theWLine.Point(aLastGeomIdx).Value());
            theWLine.Point(aLastGeomIdx).Parameters(out double aU1, out double aV1, out double aU2, out double aV2);
            MovePoint(theS1, ref aU1, ref aV1);
            MovePoint(theS2, ref aU2, ref aV2);
            aVertex.SetParameters(aU1, aV1, aU2, aV2);
            aLocalWLine.Replace(aLocalWLine.NbVertex(), aVertex);
            // Change point in walking line.
            aLocalWLine.SetPoint(aLocalWLine.NbPnts(), aVertex);
        }

        return aLocalWLine;
    }

    // Check if aNextPnt lies inside of tube build on aBasePnt and aBaseVec. In 2d space.
    private static bool IsInsideIn2d(in Point2D aBasePnt, in Vector2D aBaseVec, in Point2D aNextPnt,
                                     double aSquareMaxDist)
    {
        Vector2D aVec2d = new Vector2D(aBasePnt, aNextPnt);

        // d*d = (basevec^(nextpnt-basepnt))**2 / basevec**2
        double aSqMag = aBaseVec.SquareMagnitude();
        if (aSqMag < global::Kerf.Engine.Foundation.Precision.SquareComputational())
        {
            return false;
        }
        double aCross = aVec2d.Crossed(aBaseVec);
        double aSquareDist = aCross * aCross / aSqMag;

        return (aSquareDist <= aSquareMaxDist);
    }

    // Check if aNextPnt lies inside of tube build on aBasePnt and aBaseVec. In 3d space.
    private static bool IsInsideIn3d(in Point3D aBasePnt, in Vector3D aBaseVec, in Point3D aNextPnt, double aSquareMaxDist)
    {
        Vector3D aVec = new Vector3D(aBasePnt, aNextPnt);

        // d*d = (basevec^(nextpnt-basepnt))**2 / basevec**2
        double aSqMag = aBaseVec.SquareMagnitude();
        if (aSqMag < global::Kerf.Engine.Foundation.Precision.SquareComputational())
        {
            return false;
        }
        double aSquareDist = aVec.CrossSquareMagnitude(aBaseVec) / aSqMag;

        return (aSquareDist <= aSquareMaxDist);
    }

    // Define is surface plane like or not.
    private static bool IsSurfPlaneLike(Adaptor3d_Surface theS)
    {
        if (theS.GetType() == Plane)
        {
            return true;
        }

        if (theS.GetType() == BSplineSurface)
        {
            if (theS.UDegree() == 1 && theS.VDegree() == 1)
            {
                return true;
            }
        }

        return false;
    }

    // Check and delete points using tube criteria.
    private static IntPatch_WLine DeleteByTube(IntPatch_WLine theWLine, Adaptor3d_Surface theS1,
                                               Adaptor3d_Surface theS2)
    {
        // III: Check points for tube criteria:
        // Workaround to handle case of small amount points after purge.
        // Test "boolean boptuc_complex B5" and similar.
        int aNbPnt = 0, i;

        if (theWLine.NbPnts() <= 2)
        {
            return theWLine;
        }

        NCollection_Array1<int> aNewPointsHash = new(1, theWLine.NbPnts());
        FillPointsHash(theWLine, aNewPointsHash);

        // Initial computations.
        Span<double> UonS1 = stackalloc double[3], VonS1 = stackalloc double[3], UonS2 = stackalloc double[3],
                     VonS2 = stackalloc double[3];
        theWLine.Point(1).ParametersOnS1(out UonS1[0], out VonS1[0]);
        theWLine.Point(2).ParametersOnS1(out UonS1[1], out VonS1[1]);
        theWLine.Point(1).ParametersOnS2(out UonS2[0], out VonS2[0]);
        theWLine.Point(2).ParametersOnS2(out UonS2[1], out VonS2[1]);

        Point2D aBase2dPnt1 = new Point2D(UonS1[0], VonS1[0]);
        Point2D aBase2dPnt2 = new Point2D(UonS2[0], VonS2[0]);
        Vector2D aBase2dVec1 = new Vector2D(UonS1[1] - UonS1[0], VonS1[1] - VonS1[0]);
        Vector2D aBase2dVec2 = new Vector2D(UonS2[1] - UonS2[0], VonS2[1] - VonS2[0]);
        Point3D aBase3dPnt = theWLine.Point(1).Value();
        Vector3D aBase3dVec = new Vector3D(theWLine.Point(1).Value(), theWLine.Point(2).Value());
        double aPrevStep = aBase3dVec.SquareMagnitude();

        // Choose base tolerance and scale it to pipe algorithm.
        double aBaseTolerance = global::Kerf.Engine.Foundation.Precision.Approximation();
        double aResS1Tol = Min(theS1.UResolution(aBaseTolerance), theS1.VResolution(aBaseTolerance));
        double aResS2Tol = Min(theS2.UResolution(aBaseTolerance), theS2.VResolution(aBaseTolerance));
        double aTol1 = aResS1Tol * aResS1Tol;
        double aTol2 = aResS2Tol * aResS2Tol;
        double aTol3d = aBaseTolerance * aBaseTolerance;

        const double aLimitCoeff = 0.99 * 0.99;
        const double aMaxSqrRatio = 15.0 * 15.0;
        bool isPlanePlane = IsSurfPlaneLike(theS1) && IsSurfPlaneLike(theS2);
        for (i = 3; i <= theWLine.NbPnts(); i++)
        {
            bool isDeleteState = false;

            theWLine.Point(i).ParametersOnS1(out UonS1[2], out VonS1[2]);
            theWLine.Point(i).ParametersOnS2(out UonS2[2], out VonS2[2]);
            Point2D aPnt2dOnS1 = new Point2D(UonS1[2], VonS1[2]);
            Point2D aPnt2dOnS2 = new Point2D(UonS2[2], VonS2[2]);
            Point3D aPnt3d = theWLine.Point(i).Value();

            if (aNewPointsHash[i - 1] != -1 && IsInsideIn2d(aBase2dPnt1, aBase2dVec1, aPnt2dOnS1, aTol1)
                && IsInsideIn2d(aBase2dPnt2, aBase2dVec2, aPnt2dOnS2, aTol2)
                && IsInsideIn3d(aBase3dPnt, aBase3dVec, aPnt3d, aTol3d))
            {
                // Handle possible uneven parametrization on one of 2d subspaces.
                // Delete point only when expected lengths are close to each other (aLimitCoeff).
                // Example:
                // c2d1 - line
                // c3d - line
                // c2d2 - geometrically line, but have uneven parametrization -> c2d2 is bspline.
                Xy aPntOnS1_0 = new Xy(UonS1[1] - UonS1[0], VonS1[1] - VonS1[0]);
                Xy aPntOnS1_1 = new Xy(UonS1[2] - UonS1[1], VonS1[2] - VonS1[1]);
                Xy aPntOnS2_0 = new Xy(UonS2[1] - UonS2[0], VonS2[1] - VonS2[0]);
                Xy aPntOnS2_1 = new Xy(UonS2[2] - UonS2[1], VonS2[2] - VonS2[1]);

                double aSqModS1 = aPntOnS1_1.SquareModulus();
                double aSqModS2 = aPntOnS2_1.SquareModulus();
                if (aSqModS1 < global::Kerf.Engine.Foundation.Precision.SquareComputational() || aSqModS2 < global::Kerf.Engine.Foundation.Precision.SquareComputational())
                {
                    continue;
                }
                double aStepOnS1 = aPntOnS1_0.SquareModulus() / aSqModS1;
                double aStepOnS2 = aPntOnS2_0.SquareModulus() / aSqModS2;

                // Check very rare case when wline fluctuates nearly one point and some of them may be equal.
                // Middle point will be deleted when such situation occurs.
                // bugs moddata_2 bug469.
                if (Min(aStepOnS1, aStepOnS2) >= aLimitCoeff * Max(aStepOnS1, aStepOnS2))
                {
                    // Set hash flag to "Delete" state.
                    double aCurrStep = aBase3dPnt.SquareDistance(aPnt3d);
                    double aSqrRatio = 0.0;
                    if (!isPlanePlane)
                    {
                        aSqrRatio = (aCurrStep > global::Kerf.Engine.Foundation.Precision.SquareComputational()) ? aPrevStep / aCurrStep
                                                                                   : aMaxSqrRatio;
                        if (aSqrRatio < 1.0)
                        {
                            aSqrRatio = 1.0 / aSqrRatio;
                        }
                    }
                    if (aSqrRatio < aMaxSqrRatio)
                    {
                        isDeleteState = true;
                        aNewPointsHash.SetValue(i - 1, 1);

                        // Change middle point.
                        UonS1[1] = UonS1[2];
                        UonS2[1] = UonS2[2];
                        VonS1[1] = VonS1[2];
                        VonS2[1] = VonS2[2];
                    }
                }
            }

            if (!isDeleteState)
            {
                // Compute new pipe parameters.
                UonS1[0] = UonS1[1];
                VonS1[0] = VonS1[1];
                UonS2[0] = UonS2[1];
                VonS2[0] = VonS2[1];

                UonS1[1] = UonS1[2];
                VonS1[1] = VonS1[2];
                UonS2[1] = UonS2[2];
                VonS2[1] = VonS2[2];

                aBase2dPnt1.SetCoord(UonS1[0], VonS1[0]);
                aBase2dPnt2.SetCoord(UonS2[0], VonS2[0]);
                aBase2dVec1.SetCoord(UonS1[1] - UonS1[0], VonS1[1] - VonS1[0]);
                aBase2dVec2.SetCoord(UonS2[1] - UonS2[0], VonS2[1] - VonS2[0]);
                aBase3dPnt = theWLine.Point(i - 1).Value();
                aBase3dVec = new Vector3D(theWLine.Point(i - 1).Value(), theWLine.Point(i).Value());

                aPrevStep = aBase3dVec.SquareMagnitude();

                aNbPnt++;
            }
        }

        // Workaround to handle case of small amount of points after purge.
        // Test "boolean boptuc_complex B5" and similar.
        // This is possible since there are at least two points.
        if (aNewPointsHash[1] == -1 && aNewPointsHash[2] == -1 && aNbPnt <= 3)
        {
            // Delete first.
            aNewPointsHash[1] = 1;
        }
        if (aNewPointsHash[theWLine.NbPnts() - 1] == -1 && aNewPointsHash[theWLine.NbPnts()] == -1 && aNbPnt <= 3)
        {
            // Delete last.
            aNewPointsHash[theWLine.NbPnts()] = 1;
        }

        // Purgre when too small amount of points left.
        if (aNbPnt <= 2)
        {
            for (i = aNewPointsHash.Lower(); i <= aNewPointsHash.Upper(); i++)
            {
                if (aNewPointsHash[i] != -1)
                {
                    aNewPointsHash[i] = 1;
                }
            }
        }

        // Handle possible bad distribution of points,
        // which are will converted into one single bezier curve (less than 30 points).
        // Make distribution more even:
        // max step will be nearly to 0.1 of param distance.
        if (aNbPnt + 2 > aMinNbBadDistr && aNbPnt + 2 < aNbSingleBezier)
        {
            for (int anIdx = 1; anIdx <= 8; anIdx++)
            {
                int aHashIdx = (anIdx * theWLine.NbPnts() / 9);

                // Vertex must be stored as VERTEX (HASH = -1)
                if (aNewPointsHash[aHashIdx] != -1)
                {
                    aNewPointsHash[aHashIdx] = 0;
                }
            }
        }

        return MakeNewWLine(theWLine, aNewPointsHash, false);
    }

    /// <summary>
    /// Removes from a walking line its equal points, its points out of the domains (neither surface periodic), and
    /// points inside the tube around their neighbours. Returns a new line (sharing nothing with the given one, except
    /// that the given line may be returned as it is), or null if the line degenerates to a point.
    /// </summary>
    public static IntPatch_WLine? ComputePurgedWLine(IntPatch_WLine theWLine, Adaptor3d_Surface theS1,
                                                     Adaptor3d_Surface theS2, Adaptor3d_TopolTool theDom1,
                                                     Adaptor3d_TopolTool theDom2)
    {
        int i, k, v, nb, nbvtx;
        IntPatch_WLine? aResult = null;
        nbvtx = theWLine.NbVertex();
        nb = theWLine.NbPnts();
        if (nb == 2)
        {
            IntSurf_PntOn2S p1 = theWLine.Point(1);
            IntSurf_PntOn2S p2 = theWLine.Point(2);
            if (p1.Value().IsEqual(p2.Value(), Constants.Resolution()))
            {
                return aResult;
            }
        }

        IntPatch_WLine aLocalWLine;
        IntPatch_WLine aTmpWLine;
        IntSurf_LineOn2S aLineOn2S = new IntSurf_LineOn2S();
        aLocalWLine = new IntPatch_WLine(aLineOn2S, false);
        aLocalWLine.SetCreatingWayInfo(theWLine.GetCreatingWay());
        for (i = 1; i <= nb; i++)
        {
            aLineOn2S.Add(theWLine.Point(i));
        }

        for (v = 1; v <= nbvtx; v++)
        {
            aLocalWLine.AddVertex(theWLine.Vertex(v));
        }

        // I: Delete equal points
        Span<double> UV = stackalloc double[8];
        for (i = 1; i <= aLineOn2S.NbPoints(); i++)
        {
            int aStartIndex = i + 1;
            int anEndIndex = i + 5;
            nb = aLineOn2S.NbPoints();
            anEndIndex = (anEndIndex > nb) ? nb : anEndIndex;

            if ((aStartIndex > nb) || (anEndIndex <= 1))
            {
                continue;
            }

            k = aStartIndex;

            while (k <= anEndIndex)
            {
                if (i != k)
                {
                    IntSurf_PntOn2S p1 = aLineOn2S.Value(i);
                    IntSurf_PntOn2S p2 = aLineOn2S.Value(k);

                    p1.Parameters(out UV[0], out UV[1], out UV[2], out UV[3]);
                    p2.Parameters(out UV[4], out UV[5], out UV[6], out UV[7]);

                    double aMax = Math.Abs(UV[0]);
                    for (int anIdx = 1; anIdx < 8; anIdx++)
                    {
                        if (aMax < Math.Abs(UV[anIdx]))
                        {
                            aMax = Math.Abs(UV[anIdx]);
                        }
                    }

                    if (p1.Value().IsEqual(p2.Value(), Constants.Resolution())
                        || Math.Abs(UV[0] - UV[4]) + Math.Abs(UV[1] - UV[5]) < 1.0e-16 * aMax
                        || Math.Abs(UV[2] - UV[6]) + Math.Abs(UV[3] - UV[7]) < 1.0e-16 * aMax)
                    {
                        aTmpWLine = aLocalWLine;
                        aLocalWLine = new IntPatch_WLine(aLineOn2S, false);
                        aLocalWLine.SetCreatingWayInfo(theWLine.GetCreatingWay());

                        for (v = 1; v <= aTmpWLine.NbVertex(); v++)
                        {
                            IntPatch_Point aVertex = aTmpWLine.Vertex(v);
                            int avertexindex = (int)aVertex.ParameterOnLine();

                            if (avertexindex >= k)
                            {
                                aVertex.SetParameter(aVertex.ParameterOnLine() - 1.0);
                            }
                            aLocalWLine.AddVertex(aVertex);
                        }
                        aLineOn2S.RemovePoint(k);
                        anEndIndex--;
                        continue;
                    }
                }
                k++;
            }
        }

        if (aLineOn2S.NbPoints() <= 2)
        {
            if (aLineOn2S.NbPoints() == 2)
            {
                return aLocalWLine;
            }
            else
            {
                return aResult;
            }
        }

        // Avoid purge in case of C0 continuity:
        // Intersection approximator may produce invalid curve after purge, example:
        // bugs modalg_5 bug24731.
        // Do not run purger when base number of points is too small.
        // (nb is the number of points before the last removal above, as in OCCT.)
        if (theS1.UContinuity() == C0 || theS1.VContinuity() == C0
            || theS2.UContinuity() == C0 || theS2.VContinuity() == C0 || nb < aNbSingleBezier)
        {
            return aLocalWLine;
        }

        // II: Delete out of borders points.
        aLocalWLine = DeleteOuterPoints(aLocalWLine, theS1, theS2, theDom1, theDom2);

        // III: Delete points by tube criteria.
        IntPatch_WLine aLocalWLineTube = DeleteByTube(aLocalWLine, theS1, theS2);

        if (aLocalWLineTube.NbPnts() > 1)
        {
            aResult = aLocalWLineTube;
        }
        return aResult;
    }

    // Returns TRUE if segment [thePtf, thePtl] intersects "seam-edge" (if it exist) or surface boundaries and both
    // thePtf and thePtl do not match "seam-edge" or boundaries. Point thePtmid lies in this segment (in both 3D and
    // 2D-space). If thePtmid match "seam-edge" or boundaries strictly (without any tolerance) then the function will
    // return TRUE. The arrays are [U1, V1, U2, V2].
    private static bool IsSeamOrBound(in IntSurf_PntOn2S thePtf, in IntSurf_PntOn2S thePtl,
                                      in IntSurf_PntOn2S thePtmid, ReadOnlySpan<double> theArrPeriods,
                                      ReadOnlySpan<double> theFBound, ReadOnlySpan<double> theLBound)
    {
        Span<double> aParF = stackalloc double[4];
        Span<double> aParL = stackalloc double[4];
        thePtf.Parameters(out aParF[0], out aParF[1], out aParF[2], out aParF[3]);
        thePtl.Parameters(out aParL[0], out aParL[1], out aParL[2], out aParL[3]);

        Span<BoundRange> aBndR = stackalloc BoundRange[4];

        for (int i = 0; i < 4; i++)
        {
            aBndR[i] = new BoundRange();
            aBndR[i].Add(aParF[i]);
            aBndR[i].Add(aParL[i]);

            if (aBndR[i].IsIntersected(theFBound[i], theArrPeriods[i])
                == BoundRange.IntersectStatus.IntersectStatus_In)
            {
                return true;
            }

            if (aBndR[i].IsIntersected(theLBound[i], theArrPeriods[i])
                == BoundRange.IntersectStatus.IntersectStatus_In)
            {
                return true;
            }
        }

        for (int i = 0; i < 4; i++)
        {
            if (theArrPeriods[i] == 0.0)
            {
                // Strictly equal
                continue;
            }

            double aDelta = Math.Abs(aParL[i] - aParF[i]);
            if (2.0 * aDelta > theArrPeriods[i])
            {
                // Most likely, seam is intersected.
                return true;
            }

            if (aBndR[i].IsIntersected(0.0, theArrPeriods[i]) == BoundRange.IntersectStatus.IntersectStatus_In)
            {
                return true;
            }
        }

        // The segment [thePtf, thePtl] does not intersect the boundaries and
        // the seam-edge of the surfaces.
        // Nevertheless, following situation is possible:

        //              seam or
        //               bound
        //                 |
        //    thePtf  *    |
        //                 |
        //                 * thePtmid
        //      thePtl  *  |
        //                 |

        // This case must be processed, too.

        Span<double> aMPar = stackalloc double[4];
        thePtmid.Parameters(out aMPar[0], out aMPar[1], out aMPar[2], out aMPar[3]);

        for (int i = 0; i < 4; i++)
        {
            BoundRange aBR = new BoundRange(aMPar[i], aMPar[i]);
            if (aBR.IsIntersected(theFBound[i], theArrPeriods[i]) != BoundRange.IntersectStatus.IntersectStatus_Out)
            {
                return true;
            }

            if (aBR.IsIntersected(theLBound[i], theArrPeriods[i]) != BoundRange.IntersectStatus.IntersectStatus_Out)
            {
                return true;
            }

            if (aBR.IsIntersected(0.0, theArrPeriods[i]) != BoundRange.IntersectStatus.IntersectStatus_Out)
            {
                return true;
            }
        }

        return false;
    }

    // Check if joining is possible (see JoinWLines). A global (non-static) function in OCCT's .cxx.
    private static bool CheckArgumentsToJoin(Adaptor3d_Surface theS1, Adaptor3d_Surface theS2,
                                             in IntSurf_PntOn2S thePnt, in Point3D theP1, in Point3D theP2,
                                             in Point3D theP3, double theMinRad)
    {
        double aRad = IntPatch_PointLine.CurvatureRadiusOfIntersLine(theS1, theS2, thePnt);

        if (aRad > theMinRad)
        {
            return true;
        }
        else if (aRad > 0.0)
        {
            return false;
        }

        // Curvature radius cannot be computed.
        // Check smoothness of polygon.

        //                  theP2
        //                    *
        //                    |
        //                    |
        //       *            o         *
        //      theP1         O       theP3

        // Joining is enabled if two conditions are satisfied together:
        //   1. Angle (theP1, theP2, theP3) is quite big;
        //   2. Modulus of perpendicular (O->theP2) to the segment (theP1->theP3)
        //   is less than 0.01*<modulus of this segment>.

        Vector3D aV12f = new Vector3D(theP1, theP2), aV12l = new Vector3D(theP2, theP3);

        if (aV12f.Angle(aV12l) > myMaxConcatAngle)
        {
            return false;
        }

        Vector3D aV13 = new Vector3D(theP1, theP3);
        double aSq13 = aV13.SquareMagnitude();

        return (aV12f.CrossSquareMagnitude(aV13) < 1.0e-4 * aSq13 * aSq13);
    }

    /// <summary>
    /// Joins the walking lines of <paramref name="theSlin"/> that share an end point (within Precision::Confusion,
    /// away from seams and bounds) where the intersection line is smooth there, and removes the joined points from
    /// <paramref name="theSPnt"/>. For two cylindrical surfaces only (their radii give the minimal curvature radius).
    /// </summary>
    public static void JoinWLines(NCollection_Sequence<IntPatch_Line> theSlin, NCollection_Sequence<IntPatch_Point> theSPnt,
                                  Adaptor3d_Surface theS1, Adaptor3d_Surface theS2, double theTol3D)
    {
        if (theSlin.Length() == 0)
        {
            return;
        }

        // For two cylindrical surfaces only
        double aMinRad = 1.0e-3 * Min(theS1.Cylinder().Radius(), theS2.Cylinder().Radius());

        ReadOnlySpan<double> anArrPeriods = stackalloc double[4]
        {
            theS1.IsUPeriodic() ? theS1.UPeriod() : 0.0, theS1.IsVPeriodic() ? theS1.VPeriod() : 0.0,
            theS2.IsUPeriodic() ? theS2.UPeriod() : 0.0, theS2.IsVPeriodic() ? theS2.VPeriod() : 0.0
        };

        ReadOnlySpan<double> anArrFBonds = stackalloc double[4]
        {
            theS1.FirstUParameter(), theS1.FirstVParameter(), theS2.FirstUParameter(), theS2.FirstVParameter()
        };
        ReadOnlySpan<double> anArrLBonds = stackalloc double[4]
        {
            theS1.LastUParameter(), theS1.LastVParameter(), theS2.LastUParameter(), theS2.LastVParameter()
        };

        for (int aN1 = 1; aN1 <= theSlin.Length(); aN1++)
        {
            IntPatch_WLine? aWLine1 = theSlin.Value(aN1) as IntPatch_WLine;

            if (aWLine1 is null)
            { // We must have failed to join not-point-lines
                continue;
            }

            int aNbPntsWL1 = aWLine1.NbPnts();
            IntSurf_PntOn2S aPntFWL1 = aWLine1.Point(1);
            IntSurf_PntOn2S aPntLWL1 = aWLine1.Point(aNbPntsWL1);

            for (int aNPt = 1; aNPt <= theSPnt.Length(); aNPt++)
            {
                IntSurf_PntOn2S aPntCur = theSPnt.Value(aNPt).PntOn2S();

                if (aPntCur.IsSame(aPntFWL1, global::Kerf.Engine.Foundation.Precision.Confusion()) || aPntCur.IsSame(aPntLWL1, global::Kerf.Engine.Foundation.Precision.Confusion()))
                {
                    theSPnt.Remove(aNPt);
                    aNPt--;
                }
            }

            NCollection_List<int> aListFC = new(), aListLC = new();

            bool isFirstConnected = false, isLastConnected = false;

            for (int aN2 = 1; aN2 <= theSlin.Length(); aN2++)
            {
                if (aN2 == aN1)
                {
                    continue;
                }

                IntPatch_WLine? aWLine2 = theSlin.Value(aN2) as IntPatch_WLine;

                if (aWLine2 is null)
                {
                    continue;
                }

                isFirstConnected = isLastConnected = false;

                int aNbPntsWL2 = aWLine2.NbPnts();

                IntSurf_PntOn2S aPntFWL2 = aWLine2.Point(1);
                IntSurf_PntOn2S aPntLWL2 = aWLine2.Point(aNbPntsWL2);

                double aSqDistF = aPntFWL1.Value().SquareDistance(aPntFWL2.Value());
                double aSqDistL = aPntFWL1.Value().SquareDistance(aPntLWL2.Value());

                double aSqMinFDist = Min(aSqDistF, aSqDistL);
                if (aSqMinFDist < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    bool isFM = (aSqDistF < aSqDistL);
                    IntSurf_PntOn2S aPt1 = aWLine1.Point(2);
                    IntSurf_PntOn2S aPt2 = isFM ? aWLine2.Point(2) : aWLine2.Point(aNbPntsWL2 - 1);
                    if (!IsSeamOrBound(aPt1, aPt2, aPntFWL1, anArrPeriods, anArrFBonds, anArrLBonds))
                    {
                        isFirstConnected = true;
                    }
                }

                aSqDistF = aPntLWL1.Value().SquareDistance(aPntFWL2.Value());
                aSqDistL = aPntLWL1.Value().SquareDistance(aPntLWL2.Value());

                double aSqMinLDist = Min(aSqDistF, aSqDistL);
                if (aSqMinLDist < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    bool isFM = (aSqDistF < aSqDistL);
                    IntSurf_PntOn2S aPt1 = aWLine1.Point(aNbPntsWL1 - 1);
                    IntSurf_PntOn2S aPt2 = isFM ? aWLine2.Point(2) : aWLine2.Point(aNbPntsWL2 - 1);
                    if (!IsSeamOrBound(aPt1, aPt2, aPntLWL1, anArrPeriods, anArrFBonds, anArrLBonds))
                    {
                        isLastConnected = true;
                    }
                }

                if (isFirstConnected && isLastConnected)
                {
                    if (aSqMinFDist < aSqMinLDist)
                    {
                        aListFC.Append(aN2);
                    }
                    else
                    {
                        aListLC.Append(aN2);
                    }
                }
                else if (isFirstConnected)
                {
                    aListFC.Append(aN2);
                }
                else if (isLastConnected)
                {
                    aListLC.Append(aN2);
                }
            }

            isFirstConnected = (aListFC.Extent() == 1);
            isLastConnected = (aListLC.Extent() == 1);

            if (!(isFirstConnected || isLastConnected))
            {
                continue;
            }

            {
                int anIndexWL2 = isFirstConnected ? aListFC.First() : aListLC.First();
                IntPatch_WLine aWLine2 = (IntPatch_WLine)theSlin.Value(anIndexWL2);
                int aNbPntsWL2 = aWLine2.NbPnts();
                IntSurf_PntOn2S aPntFWL2 = aWLine2.Point(1);
                IntSurf_PntOn2S aPntLWL2 = aWLine2.Point(aNbPntsWL2);

                if (isFirstConnected)
                {
                    double aSqDistF = aPntFWL1.Value().SquareDistance(aPntFWL2.Value());
                    double aSqDistL = aPntFWL1.Value().SquareDistance(aPntLWL2.Value());
                    bool isFM = (aSqDistF < aSqDistL);

                    IntSurf_PntOn2S aPt1 = aWLine1.Point(2);
                    IntSurf_PntOn2S aPt2 = isFM ? aWLine2.Point(2) : aWLine2.Point(aNbPntsWL2 - 1);

                    if (!CheckArgumentsToJoin(theS1, theS2, aPntFWL1, aPt1.Value(), aPntFWL1.Value(), aPt2.Value(),
                                              aMinRad))
                    {
                        continue;
                    }

                    aWLine1.ClearVertexes();

                    if (isFM)
                    {
                        // First-First-connection
                        for (int aNPt = 1; aNPt <= aNbPntsWL2; aNPt++)
                        {
                            IntSurf_PntOn2S aPt = aWLine2.Point(aNPt);
                            aWLine1.Curve().InsertBefore(1, aPt);
                        }
                    }
                    else
                    {
                        // First-Last-connection
                        for (int aNPt = aNbPntsWL2; aNPt >= 1; aNPt--)
                        {
                            IntSurf_PntOn2S aPt = aWLine2.Point(aNPt);
                            aWLine1.Curve().InsertBefore(1, aPt);
                        }
                    }
                }
                else // if (isLastConnected)
                {
                    double aSqDistF = aPntLWL1.Value().SquareDistance(aPntFWL2.Value());
                    double aSqDistL = aPntLWL1.Value().SquareDistance(aPntLWL2.Value());

                    bool isFM = (aSqDistF < aSqDistL);
                    IntSurf_PntOn2S aPt1 = aWLine1.Point(aNbPntsWL1 - 1);
                    IntSurf_PntOn2S aPt2 = isFM ? aWLine2.Point(2) : aWLine2.Point(aNbPntsWL2 - 1);

                    if (!CheckArgumentsToJoin(theS1, theS2, aPntLWL1, aPt1.Value(), aPntLWL1.Value(), aPt2.Value(),
                                              aMinRad))
                    {
                        continue;
                    }

                    aWLine1.ClearVertexes();

                    if (isFM)
                    {
                        // Last-First connection
                        for (int aNPt = 1; aNPt <= aNbPntsWL2; aNPt++)
                        {
                            IntSurf_PntOn2S aPt = aWLine2.Point(aNPt);
                            aWLine1.Curve().Add(aPt);
                        }
                    }
                    else
                    {
                        // Last-Last connection
                        for (int aNPt = aNbPntsWL2; aNPt >= 1; aNPt--)
                        {
                            IntSurf_PntOn2S aPt = aWLine2.Point(aNPt);
                            aWLine1.Curve().Add(aPt);
                        }
                    }
                }

                aWLine1.ComputeVertexParameters(theTol3D);
                theSlin.Remove(anIndexWL2);
                aN1--;
            }
        }
    }
}
