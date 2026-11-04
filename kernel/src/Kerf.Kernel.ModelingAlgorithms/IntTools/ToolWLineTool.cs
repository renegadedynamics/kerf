// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_WLineTool.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_WLineTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Intersection.Surfaces;
using Kerf.Engine.Intersection.Walking;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Intersection.Tools;

/// <summary>
/// Tools for the walking lines of IntTools_FaceFace: whether a line crosses a degenerated zone of a surface (then the
/// approximation doesn't use the surfaces), and the splitting of a line where it runs along the surfaces' periodic
/// boundaries.
/// </summary>
internal static class ToolWLineTool
{
    /////////////////////// NotUseSurfacesForApprox /////////////////////////

    // The block is dedicated to determine whether WLine [ifprm, ilprm]
    // crosses the degenerated zone on each given surface or not.
    // If Yes -> We will not use info about surfaces during approximation
    // because inside degenerated zone of the surface the approx. algo.
    // uses wrong values of normal, etc., and resulting curve will have
    // oscillations that we would not like to have.

    // static subfunction in IsDegeneratedZone
    private static bool IsDegeneratedZone(in Point2D aP2d, Surface aS, int iDir)
    {
        bool bFlag = true;
        double us1, us2, vs1, vs2, dY, dX, d1, d2, dD;
        double xm, ym, xb, yb, xe, ye;
        aS.ParameterBounds(out us1, out us2, out vs1, out vs2);

        Point3D pm, pb, pe;

        xm = aP2d.X;
        ym = aP2d.Y;

        aS.D0(xm, ym, out pm);

        dX = 1.0e-5;
        dY = 1.0e-5;
        dD = 1.0e-12;

        if (iDir == 1)
        {
            xb = xm;
            xe = xm;
            yb = ym - dY;
            if (yb < vs1)
            {
                yb = vs1;
            }
            ye = ym + dY;
            if (ye > vs2)
            {
                ye = vs2;
            }
            aS.D0(xb, yb, out pb);
            aS.D0(xe, ye, out pe);

            d1 = pm.DistanceTo(pb);
            d2 = pm.DistanceTo(pe);
            if (d1 < dD && d2 < dD)
            {
                return bFlag;
            }
            return !bFlag;
        }
        //
        else if (iDir == 2)
        {
            yb = ym;
            ye = ym;
            xb = xm - dX;
            if (xb < us1)
            {
                xb = us1;
            }
            xe = xm + dX;
            if (xe > us2)
            {
                xe = us2;
            }
            aS.D0(xb, yb, out pb);
            aS.D0(xe, ye, out pe);

            d1 = pm.DistanceTo(pb);
            d2 = pm.DistanceTo(pe);
            if (d1 < dD && d2 < dD)
            {
                return bFlag;
            }
            return !bFlag;
        }
        return !bFlag;
    }

    // static subfunction in NotUseSurfacesForApprox
    private static bool IsPointInDegeneratedZone(in PntOn2S aP2S, in Face aF1, in Face aF2)
    {
        bool bFlag = true;
        double us11, us12, vs11, vs12, us21, us22, vs21, vs22;
        double u1, v1, u2, v2, delta, d;
        Point2D p2d = new();

        Surface s1 = Tool.Surface(aF1)!;
        s1.ParameterBounds(out us11, out us12, out vs11, out vs12);
        SurfaceAdaptor gas1 = new(s1);

        Surface s2 = Tool.Surface(aF2)!;
        s1.ParameterBounds(out us21, out us22, out vs21, out vs22); // OCCT reads the first surface's bounds for the second
        SurfaceAdaptor gas2 = new(s2);
        //
        aP2S.Parameters(out u1, out v1, out u2, out v2);
        //
        delta = 1.0e-7;
        // Check on Surf 1
        d = gas1.UResolution(delta);
        p2d = p2d.WithCoord(u1, v1);
        if (Math.Abs(u1 - us11) < d)
        {
            bFlag = IsDegeneratedZone(p2d, s1, 1);
            if (bFlag)
            {
                return bFlag;
            }
        }
        if (Math.Abs(u1 - us12) < d)
        {
            bFlag = IsDegeneratedZone(p2d, s1, 1);
            if (bFlag)
            {
                return bFlag;
            }
        }
        d = gas1.VResolution(delta);
        // OCCT compares with aDelta, not the resolution aD, here and on the second surface
        if (Math.Abs(v1 - vs11) < delta)
        {
            bFlag = IsDegeneratedZone(p2d, s1, 2);
            if (bFlag)
            {
                return bFlag;
            }
        }
        if (Math.Abs(v1 - vs12) < delta)
        {
            bFlag = IsDegeneratedZone(p2d, s1, 2);
            if (bFlag)
            {
                return bFlag;
            }
        }
        // Check on Surf 2
        d = gas2.UResolution(delta);
        p2d = p2d.WithCoord(u2, v2);
        if (Math.Abs(u2 - us21) < delta)
        {
            bFlag = IsDegeneratedZone(p2d, s2, 1);
            if (bFlag)
            {
                return bFlag;
            }
        }
        if (Math.Abs(u2 - us22) < delta)
        {
            bFlag = IsDegeneratedZone(p2d, s2, 1);
            if (bFlag)
            {
                return bFlag;
            }
        }
        d = gas2.VResolution(delta);
        if (Math.Abs(v2 - vs21) < delta)
        {
            bFlag = IsDegeneratedZone(p2d, s2, 2);
            if (bFlag)
            {
                return bFlag;
            }
        }
        if (Math.Abs(v2 - vs22) < delta)
        {
            bFlag = IsDegeneratedZone(p2d, s2, 2);
            if (bFlag)
            {
                return bFlag;
            }
        }
        return !bFlag;
    }

    /// <summary>
    /// True if the first or the last point of the walking line's range [ifprm, ilprm] is in a degenerated zone of
    /// one of the faces' surfaces (a boundary that is a point): the approximation then shouldn't use the surfaces.
    /// </summary>
    public static bool NotUseSurfacesForApprox(in Face aF1, in Face aF2, WLine wl, int ifprm,
                                               int ilprm)
    {
        bool bPInDZ;

        LineOn2S lineOn2S = wl.Curve();

        PntOn2S p2Sfprm = lineOn2S.Value(ifprm);
        bPInDZ = IsPointInDegeneratedZone(p2Sfprm, aF1, aF2);
        if (bPInDZ)
        {
            return bPInDZ;
        }

        PntOn2S p2Slprm = lineOn2S.Value(ilprm);
        bPInDZ = IsPointInDegeneratedZone(p2Slprm, aF1, aF2);

        return bPInDZ;
    }

    /////////////////////// end of NotUseSurfacesForApprox //////////////////

    /////////////////////// DecompositionOfWLine ////////////////////////////

    // static subfunction in DecompositionOfWLine
    private static bool IsPointOnBoundary(double parameter, double firstBoundary, double secondBoundary,
                                          double resolution, out bool isOnFirstBoundary)
    {
        bool bRet;
        int i;
        double adist;
        //
        bRet = false;
        isOnFirstBoundary = false;
        for (i = 0; i < 2; ++i)
        {
            isOnFirstBoundary = (i == 0);
            if (isOnFirstBoundary)
            {
                adist = Math.Abs(parameter - firstBoundary);
            }
            else
            {
                adist = Math.Abs(parameter - secondBoundary);
            }
            if (adist < resolution)
            {
                return !bRet;
            }
        }
        return bRet;
    }

    // static subfunction in DecompositionOfWLine
    private static Point2D AdjustByNeighbour(in Point2D theaNeighbourPoint, in Point2D originalPoint,
                                              SurfaceAdaptor gaSurface)
    {
        Point2D ap1 = theaNeighbourPoint;
        Point2D ap2 = originalPoint;

        if (gaSurface.IsUPeriodic)
        {
            double period = gaSurface.UPeriod();
            Point2D pTest = ap2;
            double sqDistMin = 1.0e+100;

            for (int pIt = -1; pIt <= 1; pIt++)
            {
                pTest = pTest.WithX(originalPoint.X + period * pIt);
                double dd = ap1.SquareDistanceTo(pTest);

                if (dd < sqDistMin)
                {
                    ap2 = pTest;
                    sqDistMin = dd;
                }
            }
        }
        if (gaSurface.IsVPeriodic)
        {
            double period = gaSurface.VPeriod();
            Point2D pTest = ap2;
            double sqDistMin = 1.0e+100;

            for (int pIt = -1; pIt <= 1; pIt++)
            {
                pTest = pTest.WithY(originalPoint.Y + period * pIt);
                double dd = ap1.SquareDistanceTo(pTest);

                if (dd < sqDistMin)
                {
                    ap2 = pTest;
                    sqDistMin = dd;
                }
            }
        }
        return ap2;
    }

    // static subfunction in FindPoint
    private static void RefineVector(ref Vector2D aV2D)
    {
        int k, m;
        double eps, r1, r2, num;
        Span<double> c = stackalloc double[2];
        //
        eps = RealEpsilon();
        r1 = 1.0 - eps;
        r2 = 1.0 + eps;
        //
        aV2D.Coord(out c[0], out c[1]);
        //
        for (k = 0; k < 2; ++k)
        {
            m = (k + 1) % 2;
            num = Math.Abs(c[k]);
            if (num > r1 && num < r2)
            {
                if (c[k] < 0.0)
                {
                    c[k] = -1.0;
                }
                else
                {
                    c[k] = 1.0;
                }
                c[m] = 0.0;
                break;
            }
        }
        aV2D = aV2D.WithCoord(c[0], c[1]);
    }

    // static subfunction in DecompositionOfWLine
    private static bool FindPoint(in Point2D firstPoint, in Point2D lastPoint, double umin, double umax,
                                  double vmin, double vmax, ref Point2D newPoint)
    {
        Vector2D vec = new(firstPoint, lastPoint);
        int i = 0, j = 0;

        for (i = 0; i < 4; i++)
        {
            Vector2D otherVec = new();
            Vector2D otherVecNormal = new();
            Point2D aprojpoint = lastPoint;

            if ((i % 2) == 0)
            {
                otherVec = otherVec.WithX(0.0);
                otherVec = otherVec.WithY(1.0);
                otherVecNormal = otherVecNormal.WithX(1.0);
                otherVecNormal = otherVecNormal.WithY(0.0);

                if (i < 2)
                {
                    aprojpoint = aprojpoint.WithX(umin);
                }
                else
                {
                    aprojpoint = aprojpoint.WithX(umax);
                }
            }
            else
            {
                otherVec = otherVec.WithX(1.0);
                otherVec = otherVec.WithY(0.0);
                otherVecNormal = otherVecNormal.WithX(0.0);
                otherVecNormal = otherVecNormal.WithY(1.0);

                if (i < 2)
                {
                    aprojpoint = aprojpoint.WithY(vmin);
                }
                else
                {
                    aprojpoint = aprojpoint.WithY(vmax);
                }
            }
            Vector2D anormvec = vec;
            anormvec = anormvec.Normalized();
            RefineVector(ref anormvec);
            double adot1 = anormvec.Dot(otherVecNormal);

            if (Math.Abs(adot1) < global::Kerf.Engine.Foundation.Precision.Angular())
            {
                continue;
            }
            double adist = 0.0;
            bool bIsOut = false;

            if ((i % 2) == 0)
            {
                adist = (i < 2) ? Math.Abs(lastPoint.X - umin) : Math.Abs(lastPoint.X - umax);
                bIsOut = (i < 2) ? (lastPoint.X < umin) : (lastPoint.X > umax);
            }
            else
            {
                adist = (i < 2) ? Math.Abs(lastPoint.Y - vmin) : Math.Abs(lastPoint.Y - vmax);
                bIsOut = (i < 2) ? (lastPoint.Y < vmin) : (lastPoint.Y > vmax);
            }
            double anoffset = adist * otherVec.Dot(anormvec) / adot1;

            for (j = 0; j < 2; j++)
            {
                anoffset = (j == 0) ? anoffset : -anoffset;
                Point2D acurpoint = new(aprojpoint.XY() + (otherVec.XY() * anoffset));
                Vector2D acurvec = new(lastPoint, acurpoint);
                if (bIsOut)
                {
                    acurvec = acurvec.Reversed();
                }

                double dotX, angleX;
                //
                dotX = vec.Dot(acurvec);
                angleX = vec.AngleTo(acurvec);
                //
                if (dotX > 0.0 && Math.Abs(angleX) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    if ((i % 2) == 0)
                    {
                        if ((acurpoint.Y >= vmin) && (acurpoint.Y <= vmax))
                        {
                            newPoint = acurpoint;
                            return true;
                        }
                    }
                    else
                    {
                        if ((acurpoint.X >= umin) && (acurpoint.X <= umax))
                        {
                            newPoint = acurpoint;
                            return true;
                        }
                    }
                }
            }
        }
        return false;
    }

    /// <summary>
    /// Splits the walking line where it runs along a periodic boundary of a surface (its points on a boundary are
    /// dropped and the pieces' ends moved onto the boundary), restricted to the line constructor's parts unless
    /// theAvoidLConstructor. The pieces are appended to theNewLines; false if the line isn't split.
    /// </summary>
    public static bool DecompositionOfWLine(WLine wLine, SurfaceAdaptor surface1,
                                            SurfaceAdaptor surface2, in Face face1,
                                            in Face face2, LineConstructor lConstructor,
                                            bool avoidLConstructor, double tol,
                                            NCollection_Sequence<PatchLine> newLines, IntersectionContext aContext)
    {
        bool bRet, bAvoidLineConstructor;
        int nbPnts, nbParts;
        //
        bRet = false;
        nbPnts = wLine.NbPnts();
        bAvoidLineConstructor = avoidLConstructor;
        //
        if (nbPnts == 0)
        {
            return bRet;
        }
        if (!bAvoidLineConstructor)
        {
            nbParts = lConstructor.PartCount;
            if (nbParts == 0)
            {
                return bRet;
            }
        }
        //
        bool bIsPrevPointOnBoundary, bIsPointOnBoundary, bIsCurrentPointOnBoundary;
        int nblines, pit, i, j;
        double aTol;
        NCollection_Array1<NCollection_List<int>> arrayOfLines = new(1, nbPnts);
        NCollection_Array1<int> arrayOfLineType = new(1, nbPnts);
        NCollection_List<int> listOfPointIndex = new();

        //
        nblines = 0;
        aTol = global::Kerf.Engine.Foundation.Precision.Confusion();
        aTol = 0.5 * aTol;
        bIsPrevPointOnBoundary = false;
        bIsPointOnBoundary = false;
        //
        // 1. ...
        //
        // Points
        for (pit = 1; pit <= nbPnts; ++pit)
        {
            bool bIsOnFirstBoundary, isperiodic;
            double resolution, period, alowerboundary, aupperboundary, u, v;
            double parameter, anoffset, adjustPar;
            double umin, umax, vmin, vmax;
            //
            bIsCurrentPointOnBoundary = false;
            PntOn2S point = wLine.Point(pit);
            //
            // Surface
            for (i = 0; i < 2; ++i)
            {
                SurfaceAdaptor gaSurface = (i == 0) ? surface1 : surface2;
                gaSurface.Surface()!.ParameterBounds(out umin, out umax, out vmin, out vmax);
                if (i == 0)
                {
                    point.ParametersOnS1(out u, out v);
                }
                else
                {
                    point.ParametersOnS2(out u, out v);
                }
                // U, V
                for (j = 0; j < 2; j++)
                {
                    isperiodic = (j == 0) ? gaSurface.IsUPeriodic : gaSurface.IsVPeriodic;
                    if (!isperiodic)
                    {
                        continue;
                    }
                    //
                    if (j == 0)
                    {
                        resolution = gaSurface.UResolution(aTol);
                        period = gaSurface.UPeriod();
                        alowerboundary = umin;
                        aupperboundary = umax;
                        parameter = u;
                    }
                    else
                    {
                        resolution = gaSurface.VResolution(aTol);
                        period = gaSurface.VPeriod();
                        alowerboundary = vmin;
                        aupperboundary = vmax;
                        parameter = v;
                    }

                    global::Kerf.Engine.Intersection.Surfaces.GeomInt.AdjustPeriodic(parameter, alowerboundary, aupperboundary, period, out adjustPar,
                                           out anoffset);
                    //
                    bIsPointOnBoundary = IsPointOnBoundary(adjustPar, alowerboundary, aupperboundary, resolution,
                                                           out bIsOnFirstBoundary);
                    //
                    if (bIsPointOnBoundary)
                    {
                        bIsCurrentPointOnBoundary = true;
                        break;
                    }
                } // for(j = 0; j < 2; j++) {

                if (bIsCurrentPointOnBoundary)
                {
                    break;
                }
            } // for(i = 0; i < 2; ++i) {
            //
            if ((bIsCurrentPointOnBoundary != bIsPrevPointOnBoundary))
            {
                if (!listOfPointIndex.IsEmpty())
                {
                    nblines++;
                    arrayOfLines.SetValue(nblines, new NCollection_List<int>(listOfPointIndex));
                    arrayOfLineType.SetValue(nblines, bIsPrevPointOnBoundary ? 1 : 0);
                    listOfPointIndex.Clear();
                }
                bIsPrevPointOnBoundary = bIsCurrentPointOnBoundary;
            }
            listOfPointIndex.Append(pit);
        } // for(pit = 1; pit <= aNbPnts; ++pit) {
        //
        if (!listOfPointIndex.IsEmpty())
        {
            nblines++;
            arrayOfLines.SetValue(nblines, new NCollection_List<int>(listOfPointIndex));
            arrayOfLineType.SetValue(nblines, bIsPrevPointOnBoundary ? 1 : 0);
            listOfPointIndex.Clear();
        }
        //
        if (nblines <= 1)
        {
            return bRet; // false;
        }
        //
        //
        // 2. Correct wlines.begin
        NCollection_Array1<NCollection_List<int>> arrayOfLineEnds = new(1, nblines);
        LineOn2S seqOfPntOn2S = new();
        //
        for (i = 1; i <= nblines; i++)
        {
            if (arrayOfLineType.Value(i) != 0)
            {
                continue;
            }
            NCollection_List<int> listOfIndex = arrayOfLines.Value(i);
            NCollection_List<int> listOfFLIndex = new();

            for (j = 0; j < 2; j++)
            {
                int aneighbourindex = (j == 0) ? (i - 1) : (i + 1);

                if ((aneighbourindex < 1) || (aneighbourindex > nblines))
                {
                    continue;
                }

                if (arrayOfLineType.Value(aneighbourindex) == 0)
                {
                    continue;
                }
                NCollection_List<int> neighbour = arrayOfLines.Value(aneighbourindex);
                int index = (j == 0) ? neighbour.Last() : neighbour.First();
                PntOn2S point = wLine.Point(index);

                PntOn2S newP = point;
                if (listOfIndex.Extent() < 2)
                {
                    seqOfPntOn2S.Add(newP);
                    listOfFLIndex.Append(seqOfPntOn2S.PointCount);
                    continue;
                }
                //
                int iFirst = listOfIndex.First();
                int iLast = listOfIndex.Last();
                //
                for (int surfit = 0; surfit < 2; surfit++)
                {

                    SurfaceAdaptor gaSurface = (surfit == 0) ? surface1 : surface2;
                    double umin = 0.0, umax = 0.0, vmin = 0.0, vmax = 0.0;
                    gaSurface.Surface()!.ParameterBounds(out umin, out umax, out vmin, out vmax);
                    double u = 0.0, v = 0.0;

                    if (surfit == 0)
                    {
                        newP.ParametersOnS1(out u, out v);
                    }
                    else
                    {
                        newP.ParametersOnS2(out u, out v);
                    }
                    int nbboundaries = 0;

                    bool bIsNearBoundary = false;
                    // int aZoneIndex = 0;
                    bool bIsUBoundary = false; // use if nbboundaries == 1
                    bool bIsFirstBoundary = false; // use if nbboundaries == 1

                    for (int parit = 0; parit < 2; parit++)
                    {
                        bool isperiodic = (parit == 0) ? gaSurface.IsUPeriodic : gaSurface.IsVPeriodic;

                        double resolution = (parit == 0) ? gaSurface.UResolution(aTol) : gaSurface.VResolution(aTol);
                        double alowerboundary = (parit == 0) ? umin : vmin;
                        double aupperboundary = (parit == 0) ? umax : vmax;

                        double parameter = (parit == 0) ? u : v;
                        bool bIsOnFirstBoundary = true;

                        if (!isperiodic)
                        {
                            bIsPointOnBoundary = IsPointOnBoundary(parameter, alowerboundary, aupperboundary,
                                                                   resolution, out bIsOnFirstBoundary);
                            if (bIsPointOnBoundary)
                            {
                                bIsUBoundary = (parit == 0);
                                bIsFirstBoundary = bIsOnFirstBoundary;
                                nbboundaries++;
                            }
                        }
                        else
                        {
                            double period = (parit == 0) ? gaSurface.UPeriod() : gaSurface.VPeriod();
                            global::Kerf.Engine.Intersection.Surfaces.GeomInt.AdjustPeriodic(parameter, alowerboundary, aupperboundary, period,
                                                   out double adjustPar, out _);

                            bIsPointOnBoundary = IsPointOnBoundary(adjustPar, alowerboundary, aupperboundary,
                                                                   resolution, out bIsOnFirstBoundary);
                            if (bIsPointOnBoundary)
                            {
                                bIsUBoundary = (parit == 0);
                                bIsFirstBoundary = bIsOnFirstBoundary;
                                nbboundaries++;
                            }
                            else
                            {
                                // check neighbourhood of boundary
                                double epsilon = resolution * 100.0;
                                double part = (aupperboundary - alowerboundary) * 0.1;
                                epsilon = (epsilon > part) ? part : epsilon;

                                bIsNearBoundary = IsPointOnBoundary(adjustPar, alowerboundary, aupperboundary,
                                                                    epsilon, out bIsOnFirstBoundary);
                            }
                        }
                    }

                    bool bComputeLineEnd = false;

                    if (nbboundaries == 2)
                    {
                        // xf
                        bComputeLineEnd = true;
                        // xt
                    }
                    else if (nbboundaries == 1)
                    {
                        bool isperiodic = (bIsUBoundary) ? gaSurface.IsUPeriodic : gaSurface.IsVPeriodic;

                        if (isperiodic)
                        {
                            double alowerboundary = (bIsUBoundary) ? umin : vmin;
                            double aupperboundary = (bIsUBoundary) ? umax : vmax;
                            double period = (bIsUBoundary) ? gaSurface.UPeriod() : gaSurface.VPeriod();
                            double parameter = (bIsUBoundary) ? u : v;
                            global::Kerf.Engine.Intersection.Surfaces.GeomInt.AdjustPeriodic(parameter, alowerboundary, aupperboundary, period,
                                                   out double adjustPar, out double anoffset);

                            double adist = (bIsFirstBoundary) ? Math.Abs(adjustPar - alowerboundary)
                                                              : Math.Abs(adjustPar - aupperboundary);
                            double anotherPar = (bIsFirstBoundary) ? (aupperboundary - adist) : (alowerboundary + adist);
                            anotherPar += anoffset;
                            int aneighbourpointindex = (j == 0) ? listOfIndex.First() : listOfIndex.Last();
                            PntOn2S neighbourPoint = wLine.Point(aneighbourpointindex);
                            double nU1, nV1;

                            if (surfit == 0)
                            {
                                neighbourPoint.ParametersOnS1(out nU1, out nV1);
                            }
                            else
                            {
                                neighbourPoint.ParametersOnS2(out nU1, out nV1);
                            }

                            double adist1 = (bIsUBoundary) ? Math.Abs(nU1 - u) : Math.Abs(nV1 - v);
                            double adist2 = (bIsUBoundary) ? Math.Abs(nU1 - anotherPar) : Math.Abs(nV1 - anotherPar);
                            bComputeLineEnd = true;
                            bool bCheckAngle1 = false;
                            bool bCheckAngle2 = false;
                            Vector2D newVec = new();
                            double anewU = (bIsUBoundary) ? anotherPar : u;
                            double anewV = (bIsUBoundary) ? v : anotherPar;

                            if (((adist1 - adist2) > global::Kerf.Engine.Foundation.Precision.PConfusion()) && (adist2 < (period / 4.0)))
                            {
                                bCheckAngle1 = true;
                                newVec = new Vector2D(new Point2D(nU1, nV1), new Point2D(anewU, anewV));

                                if (newVec.SquareMagnitude < Constants.Resolution())
                                {
                                    newP.SetValue((surfit == 0), anewU, anewV);
                                    bCheckAngle1 = false;
                                }
                            }
                            else if (adist1 < (period / 4.0))
                            {
                                bCheckAngle2 = true;
                                newVec = new Vector2D(new Point2D(nU1, nV1), new Point2D(u, v));

                                if (newVec.SquareMagnitude < Constants.Resolution())
                                {
                                    bCheckAngle2 = false;
                                }
                            }

                            if (bCheckAngle1 || bCheckAngle2)
                            {
                                // assume there are at least two points in line (see "if" above)
                                int anindexother = aneighbourpointindex;

                                while ((anindexother <= iLast) && (anindexother >= iFirst))
                                {
                                    anindexother = (j == 0) ? (anindexother + 1) : (anindexother - 1);
                                    PntOn2S prevNeighbourPoint = wLine.Point(anindexother);
                                    double nU2, nV2;

                                    if (surfit == 0)
                                    {
                                        prevNeighbourPoint.ParametersOnS1(out nU2, out nV2);
                                    }
                                    else
                                    {
                                        prevNeighbourPoint.ParametersOnS2(out nU2, out nV2);
                                    }
                                    Vector2D vecOld = new(new Point2D(nU2, nV2), new Point2D(nU1, nV1));

                                    if (vecOld.SquareMagnitude <= Constants.Resolution())
                                    {
                                        continue;
                                    }
                                    else
                                    {
                                        double angle = newVec.AngleTo(vecOld);

                                        if ((Math.Abs(angle) < (M_PI * 0.25)) && (newVec.Dot(vecOld) > 0.0))
                                        {

                                            if (bCheckAngle1)
                                            {
                                                double U1, U2, V1, V2;
                                                PntOn2S atmppoint = newP;
                                                atmppoint.SetValue((surfit == 0), anewU, anewV);
                                                atmppoint.Parameters(out U1, out V1, out U2, out V2);
                                                Point3D p1 = surface1.Value(U1, V1);
                                                Point3D p2 = surface2.Value(U2, V2);
                                                Point3D p0 = point.Value();

                                                if (p0.IsEqual(p1, aTol) && p0.IsEqual(p2, aTol) && p1.IsEqual(p2, aTol))
                                                {
                                                    bComputeLineEnd = false;
                                                    newP.SetValue((surfit == 0), anewU, anewV);
                                                }
                                            }

                                            if (bCheckAngle2)
                                            {
                                                bComputeLineEnd = false;
                                            }
                                        }
                                        break;
                                    }
                                } // end while(anindexother...)
                            }
                        }
                    }
                    else if (bIsNearBoundary)
                    {
                        bComputeLineEnd = true;
                    }

                    if (bComputeLineEnd)
                    {

                        Point2D anewpoint = new();
                        bool found = false;

                        if (bIsNearBoundary)
                        {
                            // re-compute point near natural boundary or near tangent zone
                            double u1, v1, u2, v2;
                            newP.Parameters(out u1, out v1, out u2, out v2);
                            if (surfit == 0)
                            {
                                anewpoint = new Point2D(u1, v1);
                            }
                            else
                            {
                                anewpoint = new Point2D(u2, v2);
                            }

                            int aneighbourpointindex1 = (j == 0) ? iFirst : iLast;
                            PntOn2S neighbourPoint = wLine.Point(aneighbourpointindex1);
                            double nU1, nV1;

                            if (surfit == 0)
                            {
                                neighbourPoint.ParametersOnS1(out nU1, out nV1);
                            }
                            else
                            {
                                neighbourPoint.ParametersOnS2(out nU1, out nV1);
                            }
                            Point2D ap1 = new(nU1, nV1);
                            Point2D ap2;

                            if (gaSurface.IsUPeriodic || gaSurface.IsVPeriodic)
                            {
                                // re-compute point near boundary if shifted on a period
                                ap2 = AdjustByNeighbour(ap1, anewpoint, gaSurface);

                                if ((ap2.X < umin) || (ap2.X > umax) || (ap2.Y < vmin) || (ap2.Y > vmax))
                                {
                                    found = FindPoint(ap1, ap2, umin, umax, vmin, vmax, ref anewpoint);
                                }
                                else
                                {
                                    anewpoint = ap2;
                                    newP.SetValue((surfit == 0), anewpoint.X, anewpoint.Y);
                                }
                            }
                        }
                        else
                        {

                            int aneighbourpointindex1 = (j == 0) ? iFirst : iLast;
                            PntOn2S neighbourPoint = wLine.Point(aneighbourpointindex1);
                            double nU1, nV1;

                            if (surfit == 0)
                            {
                                neighbourPoint.ParametersOnS1(out nU1, out nV1);
                            }
                            else
                            {
                                neighbourPoint.ParametersOnS2(out nU1, out nV1);
                            }
                            Point2D ap1 = new(nU1, nV1);
                            Point2D ap2 = new(nU1, nV1);
                            int aneighbourpointindex2 = aneighbourpointindex1;

                            while ((aneighbourpointindex2 <= iLast) && (aneighbourpointindex2 >= iFirst))
                            {
                                aneighbourpointindex2 = (j == 0) ? (aneighbourpointindex2 + 1) : (aneighbourpointindex2 - 1);
                                PntOn2S prevNeighbourPoint = wLine.Point(aneighbourpointindex2);
                                double nU2, nV2;

                                if (surfit == 0)
                                {
                                    prevNeighbourPoint.ParametersOnS1(out nU2, out nV2);
                                }
                                else
                                {
                                    prevNeighbourPoint.ParametersOnS2(out nU2, out nV2);
                                }
                                ap2 = ap2.WithX(nU2);
                                ap2 = ap2.WithY(nV2);

                                if (ap1.SquareDistanceTo(ap2) > Constants.Resolution())
                                {
                                    break;
                                }
                            }
                            found = FindPoint(ap2, ap1, umin, umax, vmin, vmax, ref anewpoint);
                        }

                        if (found)
                        {
                            // check point
                            double criteria = tol;
                            ProjectPointOnSurface projector =
                                (surfit == 0) ? aContext.ProjPS(face2) : aContext.ProjPS(face1);
                            SurfaceAdaptor surface = (surfit == 0) ? surface1 : surface2;

                            SurfaceAdaptor surfaceOther = (surfit == 0) ? surface2 : surface1;

                            Point3D p3d = surface.Value(anewpoint.X, anewpoint.Y);
                            projector.Perform(p3d);

                            if (projector.IsDone)
                            {
                                if (projector.LowerDistance() < criteria)
                                {
                                    double foundU = u, foundV = v;
                                    projector.LowerDistanceParameters(out foundU, out foundV);

                                    // Correction of projected coordinates. Begin
                                    // Note, it may be shifted on a period
                                    int aneindex1 = (j == 0) ? iFirst : iLast;
                                    PntOn2S neighbourPoint = wLine.Point(aneindex1);
                                    double nUn, nVn;

                                    if (surfit == 0)
                                    {
                                        neighbourPoint.ParametersOnS2(out nUn, out nVn);
                                    }
                                    else
                                    {
                                        neighbourPoint.ParametersOnS1(out nUn, out nVn);
                                    }
                                    Point2D neighbour2d = new(nUn, nVn);
                                    Point2D adjustedPoint =
                                        AdjustByNeighbour(neighbour2d, new Point2D(foundU, foundV), surfaceOther);
                                    foundU = adjustedPoint.X;
                                    foundV = adjustedPoint.Y;

                                    // OCCT: the condition can't hold (X < umin and X > umax), so the branch never runs
                                    if ((adjustedPoint.X < umin) && (adjustedPoint.X > umax)
                                        && (adjustedPoint.Y < vmin) && (adjustedPoint.Y > vmax))
                                    {
                                        // attempt to roughly re-compute point
                                        foundU = (foundU < umin) ? umin : foundU;
                                        foundU = (foundU > umax) ? umax : foundU;
                                        foundV = (foundV < vmin) ? vmin : foundV;
                                        foundV = (foundV > vmax) ? vmax : foundV;

                                        ProjectPointOnSurface projector2 =
                                            (surfit == 0) ? aContext.ProjPS(face1) : aContext.ProjPS(face2);

                                        p3d = surfaceOther.Value(foundU, foundV);
                                        projector2.Perform(p3d);

                                        if (projector2.IsDone)
                                        {
                                            if (projector2.LowerDistance() < criteria)
                                            {
                                                projector2.LowerDistanceParameters(out double foundU2,
                                                                                    out double foundV2);
                                                anewpoint = anewpoint.WithX(foundU2);
                                                anewpoint = anewpoint.WithY(foundV2);
                                            }
                                        }
                                    }
                                    // Correction of projected coordinates. End

                                    if (surfit == 0)
                                    {
                                        newP.SetValue(p3d, anewpoint.X, anewpoint.Y, foundU, foundV);
                                    }
                                    else
                                    {
                                        newP.SetValue(p3d, foundU, foundV, anewpoint.X, anewpoint.Y);
                                    }
                                }
                            }
                        }
                    }
                }
                seqOfPntOn2S.Add(newP);
                listOfFLIndex.Append(seqOfPntOn2S.PointCount);
            }
            arrayOfLineEnds.SetValue(i, listOfFLIndex);
        }
        // Correct wlines.end

        // Split wlines.begin
        int nbiter;
        //
        nbiter = 1;
        if (!bAvoidLineConstructor)
        {
            nbiter = lConstructor.PartCount;
        }
        //
        for (j = 1; j <= nbiter; ++j)
        {
            double fprm, lprm;
            int ifprm, ilprm;
            //
            if (bAvoidLineConstructor)
            {
                ifprm = 1;
                ilprm = wLine.NbPnts();
            }
            else
            {
                lConstructor.Part(j, out fprm, out lprm);
                ifprm = (int)fprm;
                ilprm = (int)lprm;
            }

            LineOn2S lineOn2S = new();
            //
            for (i = 1; i <= nblines; i++)
            {
                if (arrayOfLineType.Value(i) != 0)
                {
                    continue;
                }
                NCollection_List<int> listOfIndex = arrayOfLines.Value(i);
                NCollection_List<int> listOfFLIndex = arrayOfLineEnds.Value(i);
                bool bhasfirstpoint = (listOfFLIndex.Extent() == 2);
                bool bhaslastpoint = (listOfFLIndex.Extent() == 2);

                if (!bhasfirstpoint && !listOfFLIndex.IsEmpty())
                {
                    bhasfirstpoint = (i != 1);
                }

                if (!bhaslastpoint && !listOfFLIndex.IsEmpty())
                {
                    bhaslastpoint = (i != nblines);
                }

                int iFirst = listOfIndex.First();
                int iLast = listOfIndex.Last();
                bool bIsFirstInside = ((ifprm >= iFirst) && (ifprm <= iLast));
                bool bIsLastInside = ((ilprm >= iFirst) && (ilprm <= iLast));

                if (!bIsFirstInside && !bIsLastInside)
                {
                    if ((ifprm < iFirst) && (ilprm > iLast))
                    {
                        // append whole line, and boundaries if necessary
                        if (bhasfirstpoint)
                        {
                            pit = listOfFLIndex.First();
                            lineOn2S.Add(seqOfPntOn2S.Value(pit));
                        }
                        NCollection_List<int>.Iterator it = new(listOfIndex);

                        for (; it.More(); it.Next())
                        {
                            pit = it.Value();
                            lineOn2S.Add(wLine.Point(pit));
                        }

                        if (bhaslastpoint)
                        {
                            pit = listOfFLIndex.Last();
                            lineOn2S.Add(seqOfPntOn2S.Value(pit));
                        }

                        // check end of split line (end is almost always)
                        int aneighbour = i + 1;
                        bool bIsEndOfLine = true;

                        if (aneighbour <= nblines)
                        {
                            NCollection_List<int> listOfNeighbourIndex = arrayOfLines.Value(aneighbour);

                            if ((arrayOfLineType.Value(aneighbour) != 0) && (listOfNeighbourIndex.IsEmpty()))
                            {
                                bIsEndOfLine = false;
                            }
                        }

                        if (bIsEndOfLine)
                        {
                            if (lineOn2S.PointCount > 1)
                            {
                                WLine newWLine = new(lineOn2S, false);
                                newWLine.SetCreatingWayInfo(wLine.GetCreatingWay());
                                newLines.Append(newWLine);
                            }
                            lineOn2S = new LineOn2S();
                        }
                    }
                    continue;
                }
                // end if(!bIsFirstInside && !bIsLastInside)

                if (bIsFirstInside && bIsLastInside)
                {
                    // append inside points between ifprm and ilprm
                    NCollection_List<int>.Iterator it = new(listOfIndex);

                    for (; it.More(); it.Next())
                    {
                        pit = it.Value();
                        if ((pit < ifprm) || (pit > ilprm))
                        {
                            continue;
                        }
                        lineOn2S.Add(wLine.Point(pit));
                    }
                }
                else
                {

                    if (bIsFirstInside)
                    {
                        // append points from ifprm to last point + boundary point
                        NCollection_List<int>.Iterator it = new(listOfIndex);

                        for (; it.More(); it.Next())
                        {
                            pit = it.Value();
                            if (pit < ifprm)
                            {
                                continue;
                            }
                            lineOn2S.Add(wLine.Point(pit));
                        }

                        if (bhaslastpoint)
                        {
                            pit = listOfFLIndex.Last();
                            lineOn2S.Add(seqOfPntOn2S.Value(pit));
                        }
                        // check end of split line (end is almost always)
                        int aneighbour = i + 1;
                        bool bIsEndOfLine = true;

                        if (aneighbour <= nblines)
                        {
                            NCollection_List<int> listOfNeighbourIndex = arrayOfLines.Value(aneighbour);

                            if ((arrayOfLineType.Value(aneighbour) != 0) && (listOfNeighbourIndex.IsEmpty()))
                            {
                                bIsEndOfLine = false;
                            }
                        }

                        if (bIsEndOfLine)
                        {
                            if (lineOn2S.PointCount > 1)
                            {
                                WLine newWLine = new(lineOn2S, false);
                                newWLine.SetCreatingWayInfo(wLine.GetCreatingWay());
                                newLines.Append(newWLine);
                            }
                            lineOn2S = new LineOn2S();
                        }
                    }
                    // end if(bIsFirstInside)

                    if (bIsLastInside)
                    {
                        // append points from first boundary point to ilprm
                        if (bhasfirstpoint)
                        {
                            pit = listOfFLIndex.First();
                            lineOn2S.Add(seqOfPntOn2S.Value(pit));
                        }
                        NCollection_List<int>.Iterator it = new(listOfIndex);

                        for (; it.More(); it.Next())
                        {
                            pit = it.Value();
                            if (pit > ilprm)
                            {
                                continue;
                            }
                            lineOn2S.Add(wLine.Point(pit));
                        }
                    }
                    // end if(bIsLastInside)
                }
            }

            if (lineOn2S.PointCount > 1)
            {
                WLine newWLine = new(lineOn2S, false);
                newWLine.SetCreatingWayInfo(wLine.GetCreatingWay());
                newLines.Append(newWLine);
            }
        }
        // Split wlines.end

        return true;
    }

    ///////////////////// end of DecompositionOfWLine ///////////////////////
}
