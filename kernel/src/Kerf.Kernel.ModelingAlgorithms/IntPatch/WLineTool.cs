// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_WLineTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntPatch/IntPatch_WLineTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2014 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Intersection.Patches;

/// <summary>
/// Tools for walking lines: purging (removing equal, out-of-domain and collinear points), joining lines that share an
/// end point (two cylinders), and extending lines towards each other across a common singular point.
/// </summary>
/// <remarks>
/// A static class. The lines given are edited in place, as in OCCT (decision 609). The file-static functions of OCCT's
/// .cxx are private statics (IntPatch_WLineTool.Extend.cs holds ExtendTwoWLines and its helpers).
/// </remarks>
internal static partial class WLineTool
{
    /// <summary>The largest angle (an empirical value, π/6) between two lines' end tangents for them to be joined
    /// or extended.</summary>
    public static readonly double myMaxConcatAngle = M_PI / 6;

    private const int AMinNbBadDistr = 15;
    private const int ANbSingleBezier = 30;

    // Replaces theParMIN = MIN(theParMIN, theParMAX), theParMAX = MAX(theParMIN, theParMAX).
    private static void MinMax(ref double parMIN, ref double parMAX)
    {
        if (parMIN > parMAX)
        {
            double tmp = parMAX;
            parMAX = parMIN;
            parMIN = tmp;
        }
    }

    // Fill points hash by input data. 1 - delete point; 0 - store point; -1 - vertex point (not deleted).
    private static void FillPointsHash(WLine wLine, NCollection_Array1<int> pointsHash)
    {
        int i, v;

        for (i = 1; i <= wLine.NbPnts(); i++)
        {
            pointsHash.SetValue(i, 0);
        }

        for (v = 1; v <= wLine.VertexCount; v++)
        {
            PatchPoint vertex = wLine.Vertex(v);
            int avertexindex = (int)vertex.ParameterOnLine();
            pointsHash.SetValue(avertexindex, -1);
        }
    }

    // Makes new walking line according to the points hash.
    private static WLine MakeNewWLine(WLine wLine, NCollection_Array1<int> pointsHash,
                                               bool isOuter)
    {
        int i;

        LineOn2S purgedLineOn2S = new LineOn2S();
        WLine localWLine = new WLine(purgedLineOn2S, false);
        localWLine.SetCreatingWayInfo(wLine.GetCreatingWay());
        int oldLineIdx = 1, vertexIdx = 1, indexPrev = -1, idxOld = -1;
        Point3D pPrev = new Point3D(), pOld = new Point3D();
        for (i = 1; i <= pointsHash.Upper(); i++)
        {
            if (pointsHash[i] == 0)
            {
                // Point has to be added

                Point3D p = wLine.Point(i).Value();
                double sqDistPrev = pPrev.SquareDistanceTo(pOld);
                double sqDist = pPrev.SquareDistanceTo(p);

                double ratio = (sqDistPrev < Constants.Resolution()) ? 0.0 : 9.0 * sqDist / sqDistPrev;

                if (isOuter || (ratio < Constants.Resolution()) || ((1.0 < ratio) && (ratio < 81.0))
                    || (i - indexPrev <= 1) || (i - idxOld <= 1))
                {
                    // difference in distances is satisfactory
                    // (1/9 < aSqDist/aSqDistPrev < 9)

                    // Store this point.
                    purgedLineOn2S.Add(wLine.Point(i));
                    oldLineIdx++;
                    pOld = pPrev;
                    pPrev = p;
                    idxOld = indexPrev;
                    indexPrev = i;
                }
                else if (sqDist >= sqDistPrev * 9.0)
                {
                    // current segment is much more longer
                    // (aSqDist/aSqDistPrev >= 9)

                    i = (i + indexPrev) / 2;
                    pointsHash[i] = 0;
                    i--;
                }
                else
                {
                    // previous segment is much more longer
                    //(aSqDist/aSqDistPrev <= 1/9)

                    if (indexPrev - idxOld > 1)
                    {
                        // Delete aPPrev from WL
                        purgedLineOn2S.RemovePoint(purgedLineOn2S.PointCount);
                        oldLineIdx--;

                        // Insert point between aPOld and aPPrev
                        i = (idxOld + indexPrev) / 2;
                        pointsHash[i] = 0;

                        pPrev = pOld;
                        indexPrev = idxOld;
                    }
                    else
                    {
                        pOld = pPrev;
                        idxOld = indexPrev;
                    }

                    // Next iterations will start from this inserted point.
                    i--;
                }
            }
            else if (pointsHash[i] == -1)
            {
                // Add vertex.
                PatchPoint vertex = wLine.Vertex(vertexIdx++);
                vertex.SetParameter(oldLineIdx++);
                localWLine.AddVertex(vertex);
                purgedLineOn2S.Add(wLine.Point(i));
                pPrev = pOld = wLine.Point(i).Value();
                indexPrev = idxOld = i;
            }

            // Other points will be rejected by purger.
        }

        return localWLine;
    }

    // Move point into surface param space. No interpolation used because walking algorithm should care for closeness
    // to the param space.
    private static void MovePoint(SurfaceAdaptorBase s1, ref double u1, ref double v1)
    {
        if (u1 < s1.FirstUParameter())
        {
            u1 = s1.FirstUParameter();
        }

        if (u1 > s1.LastUParameter())
        {
            u1 = s1.LastUParameter();
        }

        if (v1 < s1.FirstVParameter())
        {
            v1 = s1.FirstVParameter();
        }

        if (v1 > s1.LastVParameter())
        {
            v1 = s1.LastVParameter();
        }
    }

    // Check and delete out of bounds points on walking line.
    private static WLine DeleteOuterPoints(WLine wLine, SurfaceAdaptorBase s1,
                                                    SurfaceAdaptorBase s2, TopolTool dom1,
                                                    TopolTool dom2)
    {
        int i;

        NCollection_Array1<int> delOuterPointsHash = new(1, wLine.NbPnts());
        FillPointsHash(wLine, delOuterPointsHash);

        if (s1.IsUPeriodic || s1.IsVPeriodic || s2.IsUPeriodic || s2.IsVPeriodic)
        {
            return wLine;
        }

        Point2D pntOnF1 = new Point2D(), pntOnF2 = new Point2D();
        double x1, y1, x2, y2;

        // Iterate over points in walking line and delete which are out of bounds.
        // Forward.
        bool isAllDeleted = true;
        bool changedFirst = false;
        int firstGeomIdx = 1;
        for (i = 1; i <= wLine.NbPnts(); i++)
        {
            wLine.Point(i).Parameters(out x1, out y1, out x2, out y2);
            pntOnF1 = pntOnF1.WithCoord(x1, y1);
            pntOnF2 = pntOnF2.WithCoord(x2, y2);

            Containment state1 = dom1.Classify(pntOnF1, global::Kerf.Engine.Foundation.Precision.Confusion());
            Containment state2 = dom2.Classify(pntOnF2, global::Kerf.Engine.Foundation.Precision.Confusion());

            if (state1 == Out || state2 == Out)
            {
                delOuterPointsHash[i] = 1;
                changedFirst = true;
            }
            else
            {
                isAllDeleted = false;

                firstGeomIdx = Max(i - 1, 1);
                if (delOuterPointsHash[i] == -1)
                {
                    firstGeomIdx = i; // Use data what lies in (i) point / vertex.
                }

                delOuterPointsHash[i] = -1;
                break;
            }
        }

        if (isAllDeleted)
        {
            // ALL points are out of bounds:
            // case boolean bcut_complex F5 and similar.
            return wLine;
        }

        // Backward.
        bool changedLast = false;
        int lastGeomIdx = wLine.NbPnts();
        for (i = wLine.NbPnts(); i >= 1; i--)
        {
            wLine.Point(i).Parameters(out x1, out y1, out x2, out y2);
            pntOnF1 = pntOnF1.WithCoord(x1, y1);
            pntOnF2 = pntOnF2.WithCoord(x2, y2);

            Containment state1 = dom1.Classify(pntOnF1, global::Kerf.Engine.Foundation.Precision.Confusion());
            Containment state2 = dom2.Classify(pntOnF2, global::Kerf.Engine.Foundation.Precision.Confusion());

            if (state1 == Out || state2 == Out)
            {
                delOuterPointsHash[i] = 1;
                changedLast = true; // Move vertex to first good point
            }
            else
            {
                lastGeomIdx = Min(i + 1, wLine.NbPnts());
                if (delOuterPointsHash[i] == -1)
                {
                    lastGeomIdx = i; // Use data what lies in (i) point / vertex.
                }

                delOuterPointsHash[i] = -1;
                break;
            }
        }

        if (!changedFirst && !changedLast)
        {
            // Nothing is done, return input.
            return wLine;
        }

        // Build new line and modify geometry of necessary vertices.
        WLine localWLine = MakeNewWLine(wLine, delOuterPointsHash, true);

        if (changedFirst)
        {
            // Vertex geometry.
            PatchPoint vertex = localWLine.Vertex(1);
            vertex.SetValue(wLine.Point(firstGeomIdx).Value());
            wLine.Point(firstGeomIdx).Parameters(out double u1, out double v1, out double u2, out double v2);
            MovePoint(s1, ref u1, ref v1);
            MovePoint(s2, ref u2, ref v2);
            vertex.SetParameters(u1, v1, u2, v2);
            localWLine.Replace(1, vertex);
            // Change point in walking line.
            localWLine.SetPoint(1, vertex);
        }

        if (changedLast)
        {
            // Vertex geometry.
            PatchPoint vertex = localWLine.Vertex(localWLine.VertexCount);
            vertex.SetValue(wLine.Point(lastGeomIdx).Value());
            wLine.Point(lastGeomIdx).Parameters(out double u1, out double v1, out double u2, out double v2);
            MovePoint(s1, ref u1, ref v1);
            MovePoint(s2, ref u2, ref v2);
            vertex.SetParameters(u1, v1, u2, v2);
            localWLine.Replace(localWLine.VertexCount, vertex);
            // Change point in walking line.
            localWLine.SetPoint(localWLine.NbPnts(), vertex);
        }

        return localWLine;
    }

    // Check if aNextPnt lies inside of tube build on aBasePnt and aBaseVec. In 2d space.
    private static bool IsInsideIn2d(in Point2D aBasePnt, in Vector2D aBaseVec, in Point2D aNextPnt,
                                     double aSquareMaxDist)
    {
        Vector2D vec2d = new Vector2D(aBasePnt, aNextPnt);

        // d*d = (basevec^(nextpnt-basepnt))**2 / basevec**2
        double sqMag = aBaseVec.SquareMagnitude;
        if (sqMag < global::Kerf.Engine.Foundation.Precision.SquareComputational())
        {
            return false;
        }
        double cross = vec2d.Crossed(aBaseVec);
        double squareDist = cross * cross / sqMag;

        return (squareDist <= aSquareMaxDist);
    }

    // Check if aNextPnt lies inside of tube build on aBasePnt and aBaseVec. In 3d space.
    private static bool IsInsideIn3d(in Point3D aBasePnt, in Vector3D aBaseVec, in Point3D aNextPnt, double aSquareMaxDist)
    {
        Vector3D vec = new Vector3D(aBasePnt, aNextPnt);

        // d*d = (basevec^(nextpnt-basepnt))**2 / basevec**2
        double sqMag = aBaseVec.SquareMagnitude;
        if (sqMag < global::Kerf.Engine.Foundation.Precision.SquareComputational())
        {
            return false;
        }
        double squareDist = vec.CrossSquareMagnitude(aBaseVec) / sqMag;

        return (squareDist <= aSquareMaxDist);
    }

    // Define is surface plane like or not.
    private static bool IsSurfPlaneLike(SurfaceAdaptorBase s)
    {
        if (s.GetType() == Plane)
        {
            return true;
        }

        if (s.GetType() == SurfaceKind.BSplineSurface)
        {
            if (s.UDegree() == 1 && s.VDegree() == 1)
            {
                return true;
            }
        }

        return false;
    }

    // Check and delete points using tube criteria.
    private static WLine DeleteByTube(WLine wLine, SurfaceAdaptorBase s1,
                                               SurfaceAdaptorBase s2)
    {
        // III: Check points for tube criteria:
        // Workaround to handle case of small amount points after purge.
        // Test "boolean boptuc_complex B5" and similar.
        int nbPnt = 0, i;

        if (wLine.NbPnts() <= 2)
        {
            return wLine;
        }

        NCollection_Array1<int> newPointsHash = new(1, wLine.NbPnts());
        FillPointsHash(wLine, newPointsHash);

        // Initial computations.
        Span<double> uonS1 = stackalloc double[3], vonS1 = stackalloc double[3], uonS2 = stackalloc double[3],
                     vonS2 = stackalloc double[3];
        wLine.Point(1).ParametersOnS1(out uonS1[0], out vonS1[0]);
        wLine.Point(2).ParametersOnS1(out uonS1[1], out vonS1[1]);
        wLine.Point(1).ParametersOnS2(out uonS2[0], out vonS2[0]);
        wLine.Point(2).ParametersOnS2(out uonS2[1], out vonS2[1]);

        Point2D base2dPnt1 = new Point2D(uonS1[0], vonS1[0]);
        Point2D base2dPnt2 = new Point2D(uonS2[0], vonS2[0]);
        Vector2D base2dVec1 = new Vector2D(uonS1[1] - uonS1[0], vonS1[1] - vonS1[0]);
        Vector2D base2dVec2 = new Vector2D(uonS2[1] - uonS2[0], vonS2[1] - vonS2[0]);
        Point3D base3dPnt = wLine.Point(1).Value();
        Vector3D base3dVec = new Vector3D(wLine.Point(1).Value(), wLine.Point(2).Value());
        double prevStep = base3dVec.SquareMagnitude;

        // Choose base tolerance and scale it to pipe algorithm.
        double baseTolerance = global::Kerf.Engine.Foundation.Precision.Approximation();
        double resS1Tol = Min(s1.UResolution(baseTolerance), s1.VResolution(baseTolerance));
        double resS2Tol = Min(s2.UResolution(baseTolerance), s2.VResolution(baseTolerance));
        double tol1 = resS1Tol * resS1Tol;
        double tol2 = resS2Tol * resS2Tol;
        double tol3d = baseTolerance * baseTolerance;

        const double limitCoeff = 0.99 * 0.99;
        const double maxSqrRatio = 15.0 * 15.0;
        bool isPlanePlane = IsSurfPlaneLike(s1) && IsSurfPlaneLike(s2);
        for (i = 3; i <= wLine.NbPnts(); i++)
        {
            bool isDeleteState = false;

            wLine.Point(i).ParametersOnS1(out uonS1[2], out vonS1[2]);
            wLine.Point(i).ParametersOnS2(out uonS2[2], out vonS2[2]);
            Point2D pnt2dOnS1 = new Point2D(uonS1[2], vonS1[2]);
            Point2D pnt2dOnS2 = new Point2D(uonS2[2], vonS2[2]);
            Point3D pnt3d = wLine.Point(i).Value();

            if (newPointsHash[i - 1] != -1 && IsInsideIn2d(base2dPnt1, base2dVec1, pnt2dOnS1, tol1)
                && IsInsideIn2d(base2dPnt2, base2dVec2, pnt2dOnS2, tol2)
                && IsInsideIn3d(base3dPnt, base3dVec, pnt3d, tol3d))
            {
                // Handle possible uneven parametrization on one of 2d subspaces.
                // Delete point only when expected lengths are close to each other (aLimitCoeff).
                // Example:
                // c2d1 - line
                // c3d - line
                // c2d2 - geometrically line, but have uneven parametrization -> c2d2 is bspline.
                Xy pntOnS1_0 = new Xy(uonS1[1] - uonS1[0], vonS1[1] - vonS1[0]);
                Xy pntOnS1_1 = new Xy(uonS1[2] - uonS1[1], vonS1[2] - vonS1[1]);
                Xy pntOnS2_0 = new Xy(uonS2[1] - uonS2[0], vonS2[1] - vonS2[0]);
                Xy pntOnS2_1 = new Xy(uonS2[2] - uonS2[1], vonS2[2] - vonS2[1]);

                double sqModS1 = pntOnS1_1.SquareModulus();
                double sqModS2 = pntOnS2_1.SquareModulus();
                if (sqModS1 < global::Kerf.Engine.Foundation.Precision.SquareComputational() || sqModS2 < global::Kerf.Engine.Foundation.Precision.SquareComputational())
                {
                    continue;
                }
                double stepOnS1 = pntOnS1_0.SquareModulus() / sqModS1;
                double stepOnS2 = pntOnS2_0.SquareModulus() / sqModS2;

                // Check very rare case when wline fluctuates nearly one point and some of them may be equal.
                // Middle point will be deleted when such situation occurs.
                // bugs moddata_2 bug469.
                if (Min(stepOnS1, stepOnS2) >= limitCoeff * Max(stepOnS1, stepOnS2))
                {
                    // Set hash flag to "Delete" state.
                    double currStep = base3dPnt.SquareDistanceTo(pnt3d);
                    double sqrRatio = 0.0;
                    if (!isPlanePlane)
                    {
                        sqrRatio = (currStep > global::Kerf.Engine.Foundation.Precision.SquareComputational()) ? prevStep / currStep
                                                                                   : maxSqrRatio;
                        if (sqrRatio < 1.0)
                        {
                            sqrRatio = 1.0 / sqrRatio;
                        }
                    }
                    if (sqrRatio < maxSqrRatio)
                    {
                        isDeleteState = true;
                        newPointsHash.SetValue(i - 1, 1);

                        // Change middle point.
                        uonS1[1] = uonS1[2];
                        uonS2[1] = uonS2[2];
                        vonS1[1] = vonS1[2];
                        vonS2[1] = vonS2[2];
                    }
                }
            }

            if (!isDeleteState)
            {
                // Compute new pipe parameters.
                uonS1[0] = uonS1[1];
                vonS1[0] = vonS1[1];
                uonS2[0] = uonS2[1];
                vonS2[0] = vonS2[1];

                uonS1[1] = uonS1[2];
                vonS1[1] = vonS1[2];
                uonS2[1] = uonS2[2];
                vonS2[1] = vonS2[2];

                base2dPnt1 = base2dPnt1.WithCoord(uonS1[0], vonS1[0]);
                base2dPnt2 = base2dPnt2.WithCoord(uonS2[0], vonS2[0]);
                base2dVec1 = base2dVec1.WithCoord(uonS1[1] - uonS1[0], vonS1[1] - vonS1[0]);
                base2dVec2 = base2dVec2.WithCoord(uonS2[1] - uonS2[0], vonS2[1] - vonS2[0]);
                base3dPnt = wLine.Point(i - 1).Value();
                base3dVec = new Vector3D(wLine.Point(i - 1).Value(), wLine.Point(i).Value());

                prevStep = base3dVec.SquareMagnitude;

                nbPnt++;
            }
        }

        // Workaround to handle case of small amount of points after purge.
        // Test "boolean boptuc_complex B5" and similar.
        // This is possible since there are at least two points.
        if (newPointsHash[1] == -1 && newPointsHash[2] == -1 && nbPnt <= 3)
        {
            // Delete first.
            newPointsHash[1] = 1;
        }
        if (newPointsHash[wLine.NbPnts() - 1] == -1 && newPointsHash[wLine.NbPnts()] == -1 && nbPnt <= 3)
        {
            // Delete last.
            newPointsHash[wLine.NbPnts()] = 1;
        }

        // Purgre when too small amount of points left.
        if (nbPnt <= 2)
        {
            for (i = newPointsHash.Lower(); i <= newPointsHash.Upper(); i++)
            {
                if (newPointsHash[i] != -1)
                {
                    newPointsHash[i] = 1;
                }
            }
        }

        // Handle possible bad distribution of points,
        // which are will converted into one single bezier curve (less than 30 points).
        // Make distribution more even:
        // max step will be nearly to 0.1 of param distance.
        if (nbPnt + 2 > AMinNbBadDistr && nbPnt + 2 < ANbSingleBezier)
        {
            for (int idx = 1; idx <= 8; idx++)
            {
                int hashIdx = (idx * wLine.NbPnts() / 9);

                // Vertex must be stored as VERTEX (HASH = -1)
                if (newPointsHash[hashIdx] != -1)
                {
                    newPointsHash[hashIdx] = 0;
                }
            }
        }

        return MakeNewWLine(wLine, newPointsHash, false);
    }

    /// <summary>
    /// Removes from a walking line its equal points, its points out of the domains (neither surface periodic), and
    /// points inside the tube around their neighbours. Returns a new line (sharing nothing with the given one, except
    /// that the given line may be returned as it is), or null if the line degenerates to a point.
    /// </summary>
    public static WLine? ComputePurgedWLine(WLine wLine, SurfaceAdaptorBase s1,
                                                     SurfaceAdaptorBase s2, TopolTool dom1,
                                                     TopolTool dom2)
    {
        int i, k, v, nb, nbvtx;
        WLine? result = null;
        nbvtx = wLine.VertexCount;
        nb = wLine.NbPnts();
        if (nb == 2)
        {
            PntOn2S p1 = wLine.Point(1);
            PntOn2S p2 = wLine.Point(2);
            if (p1.Value().IsEqual(p2.Value(), Constants.Resolution()))
            {
                return result;
            }
        }

        WLine localWLine;
        WLine tmpWLine;
        LineOn2S lineOn2S = new LineOn2S();
        localWLine = new WLine(lineOn2S, false);
        localWLine.SetCreatingWayInfo(wLine.GetCreatingWay());
        for (i = 1; i <= nb; i++)
        {
            lineOn2S.Add(wLine.Point(i));
        }

        for (v = 1; v <= nbvtx; v++)
        {
            localWLine.AddVertex(wLine.Vertex(v));
        }

        // I: Delete equal points
        Span<double> uv = stackalloc double[8];
        for (i = 1; i <= lineOn2S.PointCount; i++)
        {
            int startIndex = i + 1;
            int endIndex = i + 5;
            nb = lineOn2S.PointCount;
            endIndex = (endIndex > nb) ? nb : endIndex;

            if ((startIndex > nb) || (endIndex <= 1))
            {
                continue;
            }

            k = startIndex;

            while (k <= endIndex)
            {
                if (i != k)
                {
                    PntOn2S p1 = lineOn2S.Value(i);
                    PntOn2S p2 = lineOn2S.Value(k);

                    p1.Parameters(out uv[0], out uv[1], out uv[2], out uv[3]);
                    p2.Parameters(out uv[4], out uv[5], out uv[6], out uv[7]);

                    double max = Math.Abs(uv[0]);
                    for (int idx = 1; idx < 8; idx++)
                    {
                        if (max < Math.Abs(uv[idx]))
                        {
                            max = Math.Abs(uv[idx]);
                        }
                    }

                    if (p1.Value().IsEqual(p2.Value(), Constants.Resolution())
                        || Math.Abs(uv[0] - uv[4]) + Math.Abs(uv[1] - uv[5]) < 1.0e-16 * max
                        || Math.Abs(uv[2] - uv[6]) + Math.Abs(uv[3] - uv[7]) < 1.0e-16 * max)
                    {
                        tmpWLine = localWLine;
                        localWLine = new WLine(lineOn2S, false);
                        localWLine.SetCreatingWayInfo(wLine.GetCreatingWay());

                        for (v = 1; v <= tmpWLine.VertexCount; v++)
                        {
                            PatchPoint vertex = tmpWLine.Vertex(v);
                            int avertexindex = (int)vertex.ParameterOnLine();

                            if (avertexindex >= k)
                            {
                                vertex.SetParameter(vertex.ParameterOnLine() - 1.0);
                            }
                            localWLine.AddVertex(vertex);
                        }
                        lineOn2S.RemovePoint(k);
                        endIndex--;
                        continue;
                    }
                }
                k++;
            }
        }

        if (lineOn2S.PointCount <= 2)
        {
            if (lineOn2S.PointCount == 2)
            {
                return localWLine;
            }
            else
            {
                return result;
            }
        }

        // Avoid purge in case of C0 continuity:
        // Intersection approximator may produce invalid curve after purge, example:
        // bugs modalg_5 bug24731.
        // Do not run purger when base number of points is too small.
        // (nb is the number of points before the last removal above, as in OCCT.)
        if (s1.UContinuity() == C0 || s1.VContinuity() == C0
            || s2.UContinuity() == C0 || s2.VContinuity() == C0 || nb < ANbSingleBezier)
        {
            return localWLine;
        }

        // II: Delete out of borders points.
        localWLine = DeleteOuterPoints(localWLine, s1, s2, dom1, dom2);

        // III: Delete points by tube criteria.
        WLine localWLineTube = DeleteByTube(localWLine, s1, s2);

        if (localWLineTube.NbPnts() > 1)
        {
            result = localWLineTube;
        }
        return result;
    }

    // Returns TRUE if segment [thePtf, thePtl] intersects "seam-edge" (if it exist) or surface boundaries and both
    // thePtf and thePtl do not match "seam-edge" or boundaries. Point thePtmid lies in this segment (in both 3D and
    // 2D-space). If thePtmid match "seam-edge" or boundaries strictly (without any tolerance) then the function will
    // return TRUE. The arrays are [U1, V1, U2, V2].
    private static bool IsSeamOrBound(in PntOn2S ptf, in PntOn2S ptl,
                                      in PntOn2S ptmid, ReadOnlySpan<double> arrPeriods,
                                      ReadOnlySpan<double> fBound, ReadOnlySpan<double> lBound)
    {
        Span<double> parF = stackalloc double[4];
        Span<double> parL = stackalloc double[4];
        ptf.Parameters(out parF[0], out parF[1], out parF[2], out parF[3]);
        ptl.Parameters(out parL[0], out parL[1], out parL[2], out parL[3]);

        Span<BoundRange> bndR = stackalloc BoundRange[4];

        for (int i = 0; i < 4; i++)
        {
            bndR[i] = new BoundRange();
            bndR[i].Add(parF[i]);
            bndR[i].Add(parL[i]);

            if (bndR[i].IsIntersected(fBound[i], arrPeriods[i])
                == BoundRange.IntersectStatus.IntersectStatus_In)
            {
                return true;
            }

            if (bndR[i].IsIntersected(lBound[i], arrPeriods[i])
                == BoundRange.IntersectStatus.IntersectStatus_In)
            {
                return true;
            }
        }

        for (int i = 0; i < 4; i++)
        {
            if (arrPeriods[i] == 0.0)
            {
                // Strictly equal
                continue;
            }

            double delta = Math.Abs(parL[i] - parF[i]);
            if (2.0 * delta > arrPeriods[i])
            {
                // Most likely, seam is intersected.
                return true;
            }

            if (bndR[i].IsIntersected(0.0, arrPeriods[i]) == BoundRange.IntersectStatus.IntersectStatus_In)
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

        Span<double> mPar = stackalloc double[4];
        ptmid.Parameters(out mPar[0], out mPar[1], out mPar[2], out mPar[3]);

        for (int i = 0; i < 4; i++)
        {
            BoundRange br = new BoundRange(mPar[i], mPar[i]);
            if (br.IsIntersected(fBound[i], arrPeriods[i]) != BoundRange.IntersectStatus.IntersectStatus_Out)
            {
                return true;
            }

            if (br.IsIntersected(lBound[i], arrPeriods[i]) != BoundRange.IntersectStatus.IntersectStatus_Out)
            {
                return true;
            }

            if (br.IsIntersected(0.0, arrPeriods[i]) != BoundRange.IntersectStatus.IntersectStatus_Out)
            {
                return true;
            }
        }

        return false;
    }

    // Check if joining is possible (see JoinWLines). A global (non-static) function in OCCT's .cxx.
    private static bool CheckArgumentsToJoin(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2,
                                             in PntOn2S pnt, in Point3D p1, in Point3D p2,
                                             in Point3D p3, double minRad)
    {
        double rad = PointLine.CurvatureRadiusOfIntersLine(s1, s2, pnt);

        if (rad > minRad)
        {
            return true;
        }
        else if (rad > 0.0)
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

        Vector3D v12f = new Vector3D(p1, p2), v12l = new Vector3D(p2, p3);

        if (v12f.AngleTo(v12l) > myMaxConcatAngle)
        {
            return false;
        }

        Vector3D v13 = new Vector3D(p1, p3);
        double sq13 = v13.SquareMagnitude;

        return (v12f.CrossSquareMagnitude(v13) < 1.0e-4 * sq13 * sq13);
    }

    /// <summary>
    /// Joins the walking lines of <paramref name="slin"/> that share an end point (within Precision::Confusion,
    /// away from seams and bounds) where the intersection line is smooth there, and removes the joined points from
    /// <paramref name="sPnt"/>. For two cylindrical surfaces only (their radii give the minimal curvature radius).
    /// </summary>
    public static void JoinWLines(NCollection_Sequence<PatchLine> slin, NCollection_Sequence<PatchPoint> sPnt,
                                  SurfaceAdaptorBase s1, SurfaceAdaptorBase s2, double tol3D)
    {
        if (slin.Length() == 0)
        {
            return;
        }

        // For two cylindrical surfaces only
        double minRad = 1.0e-3 * Min(s1.Cylinder().Radius, s2.Cylinder().Radius);

        ReadOnlySpan<double> arrPeriods = stackalloc double[4]
        {
            s1.IsUPeriodic ? s1.UPeriod() : 0.0, s1.IsVPeriodic ? s1.VPeriod() : 0.0,
            s2.IsUPeriodic ? s2.UPeriod() : 0.0, s2.IsVPeriodic ? s2.VPeriod() : 0.0
        };

        ReadOnlySpan<double> arrFBonds = stackalloc double[4]
        {
            s1.FirstUParameter(), s1.FirstVParameter(), s2.FirstUParameter(), s2.FirstVParameter()
        };
        ReadOnlySpan<double> arrLBonds = stackalloc double[4]
        {
            s1.LastUParameter(), s1.LastVParameter(), s2.LastUParameter(), s2.LastVParameter()
        };

        for (int n1 = 1; n1 <= slin.Length(); n1++)
        {
            WLine? wLine1 = slin.Value(n1) as WLine;

            if (wLine1 is null)
            { // We must have failed to join not-point-lines
                continue;
            }

            int nbPntsWL1 = wLine1.NbPnts();
            PntOn2S pntFWL1 = wLine1.Point(1);
            PntOn2S pntLWL1 = wLine1.Point(nbPntsWL1);

            for (int nPt = 1; nPt <= sPnt.Length(); nPt++)
            {
                PntOn2S pntCur = sPnt.Value(nPt).PntOn2S();

                if (pntCur.IsSame(pntFWL1, global::Kerf.Engine.Foundation.Precision.Confusion()) || pntCur.IsSame(pntLWL1, global::Kerf.Engine.Foundation.Precision.Confusion()))
                {
                    sPnt.Remove(nPt);
                    nPt--;
                }
            }

            NCollection_List<int> listFC = new(), listLC = new();

            bool isFirstConnected = false, isLastConnected = false;

            for (int n2 = 1; n2 <= slin.Length(); n2++)
            {
                if (n2 == n1)
                {
                    continue;
                }

                WLine? wLine2 = slin.Value(n2) as WLine;

                if (wLine2 is null)
                {
                    continue;
                }

                isFirstConnected = isLastConnected = false;

                int nbPntsWL2 = wLine2.NbPnts();

                PntOn2S pntFWL2 = wLine2.Point(1);
                PntOn2S pntLWL2 = wLine2.Point(nbPntsWL2);

                double sqDistF = pntFWL1.Value().SquareDistanceTo(pntFWL2.Value());
                double sqDistL = pntFWL1.Value().SquareDistanceTo(pntLWL2.Value());

                double sqMinFDist = Min(sqDistF, sqDistL);
                if (sqMinFDist < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    bool isFM = (sqDistF < sqDistL);
                    PntOn2S pt1 = wLine1.Point(2);
                    PntOn2S pt2 = isFM ? wLine2.Point(2) : wLine2.Point(nbPntsWL2 - 1);
                    if (!IsSeamOrBound(pt1, pt2, pntFWL1, arrPeriods, arrFBonds, arrLBonds))
                    {
                        isFirstConnected = true;
                    }
                }

                sqDistF = pntLWL1.Value().SquareDistanceTo(pntFWL2.Value());
                sqDistL = pntLWL1.Value().SquareDistanceTo(pntLWL2.Value());

                double sqMinLDist = Min(sqDistF, sqDistL);
                if (sqMinLDist < global::Kerf.Engine.Foundation.Precision.SquareConfusion())
                {
                    bool isFM = (sqDistF < sqDistL);
                    PntOn2S pt1 = wLine1.Point(nbPntsWL1 - 1);
                    PntOn2S pt2 = isFM ? wLine2.Point(2) : wLine2.Point(nbPntsWL2 - 1);
                    if (!IsSeamOrBound(pt1, pt2, pntLWL1, arrPeriods, arrFBonds, arrLBonds))
                    {
                        isLastConnected = true;
                    }
                }

                if (isFirstConnected && isLastConnected)
                {
                    if (sqMinFDist < sqMinLDist)
                    {
                        listFC.Append(n2);
                    }
                    else
                    {
                        listLC.Append(n2);
                    }
                }
                else if (isFirstConnected)
                {
                    listFC.Append(n2);
                }
                else if (isLastConnected)
                {
                    listLC.Append(n2);
                }
            }

            isFirstConnected = (listFC.Extent() == 1);
            isLastConnected = (listLC.Extent() == 1);

            if (!(isFirstConnected || isLastConnected))
            {
                continue;
            }

            {
                int indexWL2 = isFirstConnected ? listFC.First() : listLC.First();
                WLine wLine2 = (WLine)slin.Value(indexWL2);
                int nbPntsWL2 = wLine2.NbPnts();
                PntOn2S pntFWL2 = wLine2.Point(1);
                PntOn2S pntLWL2 = wLine2.Point(nbPntsWL2);

                if (isFirstConnected)
                {
                    double sqDistF = pntFWL1.Value().SquareDistanceTo(pntFWL2.Value());
                    double sqDistL = pntFWL1.Value().SquareDistanceTo(pntLWL2.Value());
                    bool isFM = (sqDistF < sqDistL);

                    PntOn2S pt1 = wLine1.Point(2);
                    PntOn2S pt2 = isFM ? wLine2.Point(2) : wLine2.Point(nbPntsWL2 - 1);

                    if (!CheckArgumentsToJoin(s1, s2, pntFWL1, pt1.Value(), pntFWL1.Value(), pt2.Value(),
                                              minRad))
                    {
                        continue;
                    }

                    wLine1.ClearVertexes();

                    if (isFM)
                    {
                        // First-First-connection
                        for (int nPt = 1; nPt <= nbPntsWL2; nPt++)
                        {
                            PntOn2S pt = wLine2.Point(nPt);
                            wLine1.Curve().InsertBefore(1, pt);
                        }
                    }
                    else
                    {
                        // First-Last-connection
                        for (int nPt = nbPntsWL2; nPt >= 1; nPt--)
                        {
                            PntOn2S pt = wLine2.Point(nPt);
                            wLine1.Curve().InsertBefore(1, pt);
                        }
                    }
                }
                else // if (isLastConnected)
                {
                    double sqDistF = pntLWL1.Value().SquareDistanceTo(pntFWL2.Value());
                    double sqDistL = pntLWL1.Value().SquareDistanceTo(pntLWL2.Value());

                    bool isFM = (sqDistF < sqDistL);
                    PntOn2S pt1 = wLine1.Point(nbPntsWL1 - 1);
                    PntOn2S pt2 = isFM ? wLine2.Point(2) : wLine2.Point(nbPntsWL2 - 1);

                    if (!CheckArgumentsToJoin(s1, s2, pntLWL1, pt1.Value(), pntLWL1.Value(), pt2.Value(),
                                              minRad))
                    {
                        continue;
                    }

                    wLine1.ClearVertexes();

                    if (isFM)
                    {
                        // Last-First connection
                        for (int nPt = 1; nPt <= nbPntsWL2; nPt++)
                        {
                            PntOn2S pt = wLine2.Point(nPt);
                            wLine1.Curve().Add(pt);
                        }
                    }
                    else
                    {
                        // Last-Last connection
                        for (int nPt = nbPntsWL2; nPt >= 1; nPt--)
                        {
                            PntOn2S pt = wLine2.Point(nPt);
                            wLine1.Curve().Add(pt);
                        }
                    }
                }

                wLine1.ComputeVertexParameters(tol3D);
                slin.Remove(indexWL2);
                n1--;
            }
        }
    }
}
