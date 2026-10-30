// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_WLineTool.hxx
// Port of: src/ModelingAlgorithms/TKBO/IntTools/IntTools_WLineTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Tools for the walking lines of IntTools_FaceFace: whether a line crosses a degenerated zone of a surface (then the
/// approximation doesn't use the surfaces), and the splitting of a line where it runs along the surfaces' periodic
/// boundaries.
/// </summary>
public static class IntTools_WLineTool
{
    /////////////////////// NotUseSurfacesForApprox /////////////////////////

    // The block is dedicated to determine whether WLine [ifprm, ilprm]
    // crosses the degenerated zone on each given surface or not.
    // If Yes -> We will not use info about surfaces during approximation
    // because inside degenerated zone of the surface the approx. algo.
    // uses wrong values of normal, etc., and resulting curve will have
    // oscillations that we would not like to have.

    // static subfunction in IsDegeneratedZone
    private static bool IsDegeneratedZone(in gp_Pnt2d aP2d, Geom_Surface aS, int iDir)
    {
        bool bFlag = true;
        double US1, US2, VS1, VS2, dY, dX, d1, d2, dD;
        double aXm, aYm, aXb, aYb, aXe, aYe;
        aS.Bounds(out US1, out US2, out VS1, out VS2);

        gp_Pnt aPm, aPb, aPe;

        aXm = aP2d.X();
        aYm = aP2d.Y();

        aS.D0(aXm, aYm, out aPm);

        dX = 1.0e-5;
        dY = 1.0e-5;
        dD = 1.0e-12;

        if (iDir == 1)
        {
            aXb = aXm;
            aXe = aXm;
            aYb = aYm - dY;
            if (aYb < VS1)
            {
                aYb = VS1;
            }
            aYe = aYm + dY;
            if (aYe > VS2)
            {
                aYe = VS2;
            }
            aS.D0(aXb, aYb, out aPb);
            aS.D0(aXe, aYe, out aPe);

            d1 = aPm.Distance(aPb);
            d2 = aPm.Distance(aPe);
            if (d1 < dD && d2 < dD)
            {
                return bFlag;
            }
            return !bFlag;
        }
        //
        else if (iDir == 2)
        {
            aYb = aYm;
            aYe = aYm;
            aXb = aXm - dX;
            if (aXb < US1)
            {
                aXb = US1;
            }
            aXe = aXm + dX;
            if (aXe > US2)
            {
                aXe = US2;
            }
            aS.D0(aXb, aYb, out aPb);
            aS.D0(aXe, aYe, out aPe);

            d1 = aPm.Distance(aPb);
            d2 = aPm.Distance(aPe);
            if (d1 < dD && d2 < dD)
            {
                return bFlag;
            }
            return !bFlag;
        }
        return !bFlag;
    }

    // static subfunction in NotUseSurfacesForApprox
    private static bool IsPointInDegeneratedZone(in IntSurf_PntOn2S aP2S, in TopoDS_Face aF1, in TopoDS_Face aF2)
    {
        bool bFlag = true;
        double US11, US12, VS11, VS12, US21, US22, VS21, VS22;
        double U1, V1, U2, V2, aDelta, aD;
        gp_Pnt2d aP2d = new();

        Geom_Surface aS1 = BRep_Tool.Surface(aF1)!;
        aS1.Bounds(out US11, out US12, out VS11, out VS12);
        GeomAdaptor_Surface aGAS1 = new(aS1);

        Geom_Surface aS2 = BRep_Tool.Surface(aF2)!;
        aS1.Bounds(out US21, out US22, out VS21, out VS22); // OCCT reads the first surface's bounds for the second
        GeomAdaptor_Surface aGAS2 = new(aS2);
        //
        aP2S.Parameters(out U1, out V1, out U2, out V2);
        //
        aDelta = 1.0e-7;
        // Check on Surf 1
        aD = aGAS1.UResolution(aDelta);
        aP2d.SetCoord(U1, V1);
        if (Math.Abs(U1 - US11) < aD)
        {
            bFlag = IsDegeneratedZone(aP2d, aS1, 1);
            if (bFlag)
            {
                return bFlag;
            }
        }
        if (Math.Abs(U1 - US12) < aD)
        {
            bFlag = IsDegeneratedZone(aP2d, aS1, 1);
            if (bFlag)
            {
                return bFlag;
            }
        }
        aD = aGAS1.VResolution(aDelta);
        // OCCT compares with aDelta, not the resolution aD, here and on the second surface
        if (Math.Abs(V1 - VS11) < aDelta)
        {
            bFlag = IsDegeneratedZone(aP2d, aS1, 2);
            if (bFlag)
            {
                return bFlag;
            }
        }
        if (Math.Abs(V1 - VS12) < aDelta)
        {
            bFlag = IsDegeneratedZone(aP2d, aS1, 2);
            if (bFlag)
            {
                return bFlag;
            }
        }
        // Check on Surf 2
        aD = aGAS2.UResolution(aDelta);
        aP2d.SetCoord(U2, V2);
        if (Math.Abs(U2 - US21) < aDelta)
        {
            bFlag = IsDegeneratedZone(aP2d, aS2, 1);
            if (bFlag)
            {
                return bFlag;
            }
        }
        if (Math.Abs(U2 - US22) < aDelta)
        {
            bFlag = IsDegeneratedZone(aP2d, aS2, 1);
            if (bFlag)
            {
                return bFlag;
            }
        }
        aD = aGAS2.VResolution(aDelta);
        if (Math.Abs(V2 - VS21) < aDelta)
        {
            bFlag = IsDegeneratedZone(aP2d, aS2, 2);
            if (bFlag)
            {
                return bFlag;
            }
        }
        if (Math.Abs(V2 - VS22) < aDelta)
        {
            bFlag = IsDegeneratedZone(aP2d, aS2, 2);
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
    public static bool NotUseSurfacesForApprox(in TopoDS_Face aF1, in TopoDS_Face aF2, IntPatch_WLine WL, int ifprm,
                                               int ilprm)
    {
        bool bPInDZ;

        IntSurf_LineOn2S aLineOn2S = WL.Curve();

        IntSurf_PntOn2S aP2Sfprm = aLineOn2S.Value(ifprm);
        bPInDZ = IsPointInDegeneratedZone(aP2Sfprm, aF1, aF2);
        if (bPInDZ)
        {
            return bPInDZ;
        }

        IntSurf_PntOn2S aP2Slprm = aLineOn2S.Value(ilprm);
        bPInDZ = IsPointInDegeneratedZone(aP2Slprm, aF1, aF2);

        return bPInDZ;
    }

    /////////////////////// end of NotUseSurfacesForApprox //////////////////

    /////////////////////// DecompositionOfWLine ////////////////////////////

    // static subfunction in DecompositionOfWLine
    private static bool IsPointOnBoundary(double theParameter, double theFirstBoundary, double theSecondBoundary,
                                          double theResolution, out bool IsOnFirstBoundary)
    {
        bool bRet;
        int i;
        double adist;
        //
        bRet = false;
        IsOnFirstBoundary = false;
        for (i = 0; i < 2; ++i)
        {
            IsOnFirstBoundary = (i == 0);
            if (IsOnFirstBoundary)
            {
                adist = Math.Abs(theParameter - theFirstBoundary);
            }
            else
            {
                adist = Math.Abs(theParameter - theSecondBoundary);
            }
            if (adist < theResolution)
            {
                return !bRet;
            }
        }
        return bRet;
    }

    // static subfunction in DecompositionOfWLine
    private static gp_Pnt2d AdjustByNeighbour(in gp_Pnt2d theaNeighbourPoint, in gp_Pnt2d theOriginalPoint,
                                              GeomAdaptor_Surface theGASurface)
    {
        gp_Pnt2d ap1 = theaNeighbourPoint;
        gp_Pnt2d ap2 = theOriginalPoint;

        if (theGASurface.IsUPeriodic())
        {
            double aPeriod = theGASurface.UPeriod();
            gp_Pnt2d aPTest = ap2;
            double aSqDistMin = 1.0e+100;

            for (int pIt = -1; pIt <= 1; pIt++)
            {
                aPTest.SetX(theOriginalPoint.X() + aPeriod * pIt);
                double dd = ap1.SquareDistance(aPTest);

                if (dd < aSqDistMin)
                {
                    ap2 = aPTest;
                    aSqDistMin = dd;
                }
            }
        }
        if (theGASurface.IsVPeriodic())
        {
            double aPeriod = theGASurface.VPeriod();
            gp_Pnt2d aPTest = ap2;
            double aSqDistMin = 1.0e+100;

            for (int pIt = -1; pIt <= 1; pIt++)
            {
                aPTest.SetY(theOriginalPoint.Y() + aPeriod * pIt);
                double dd = ap1.SquareDistance(aPTest);

                if (dd < aSqDistMin)
                {
                    ap2 = aPTest;
                    aSqDistMin = dd;
                }
            }
        }
        return ap2;
    }

    // static subfunction in FindPoint
    private static void RefineVector(ref gp_Vec2d aV2D)
    {
        int k, m;
        double aEps, aR1, aR2, aNum;
        Span<double> aC = stackalloc double[2];
        //
        aEps = RealEpsilon();
        aR1 = 1.0 - aEps;
        aR2 = 1.0 + aEps;
        //
        aV2D.Coord(out aC[0], out aC[1]);
        //
        for (k = 0; k < 2; ++k)
        {
            m = (k + 1) % 2;
            aNum = Math.Abs(aC[k]);
            if (aNum > aR1 && aNum < aR2)
            {
                if (aC[k] < 0.0)
                {
                    aC[k] = -1.0;
                }
                else
                {
                    aC[k] = 1.0;
                }
                aC[m] = 0.0;
                break;
            }
        }
        aV2D.SetCoord(aC[0], aC[1]);
    }

    // static subfunction in DecompositionOfWLine
    private static bool FindPoint(in gp_Pnt2d theFirstPoint, in gp_Pnt2d theLastPoint, double theUmin, double theUmax,
                                  double theVmin, double theVmax, ref gp_Pnt2d theNewPoint)
    {
        gp_Vec2d aVec = new(theFirstPoint, theLastPoint);
        int i = 0, j = 0;

        for (i = 0; i < 4; i++)
        {
            gp_Vec2d anOtherVec = new();
            gp_Vec2d anOtherVecNormal = new();
            gp_Pnt2d aprojpoint = theLastPoint;

            if ((i % 2) == 0)
            {
                anOtherVec.SetX(0.0);
                anOtherVec.SetY(1.0);
                anOtherVecNormal.SetX(1.0);
                anOtherVecNormal.SetY(0.0);

                if (i < 2)
                {
                    aprojpoint.SetX(theUmin);
                }
                else
                {
                    aprojpoint.SetX(theUmax);
                }
            }
            else
            {
                anOtherVec.SetX(1.0);
                anOtherVec.SetY(0.0);
                anOtherVecNormal.SetX(0.0);
                anOtherVecNormal.SetY(1.0);

                if (i < 2)
                {
                    aprojpoint.SetY(theVmin);
                }
                else
                {
                    aprojpoint.SetY(theVmax);
                }
            }
            gp_Vec2d anormvec = aVec;
            anormvec.Normalize();
            RefineVector(ref anormvec);
            double adot1 = anormvec.Dot(anOtherVecNormal);

            if (Math.Abs(adot1) < Precision.Angular())
            {
                continue;
            }
            double adist = 0.0;
            bool bIsOut = false;

            if ((i % 2) == 0)
            {
                adist = (i < 2) ? Math.Abs(theLastPoint.X() - theUmin) : Math.Abs(theLastPoint.X() - theUmax);
                bIsOut = (i < 2) ? (theLastPoint.X() < theUmin) : (theLastPoint.X() > theUmax);
            }
            else
            {
                adist = (i < 2) ? Math.Abs(theLastPoint.Y() - theVmin) : Math.Abs(theLastPoint.Y() - theVmax);
                bIsOut = (i < 2) ? (theLastPoint.Y() < theVmin) : (theLastPoint.Y() > theVmax);
            }
            double anoffset = adist * anOtherVec.Dot(anormvec) / adot1;

            for (j = 0; j < 2; j++)
            {
                anoffset = (j == 0) ? anoffset : -anoffset;
                gp_Pnt2d acurpoint = new(aprojpoint.XY() + (anOtherVec.XY() * anoffset));
                gp_Vec2d acurvec = new(theLastPoint, acurpoint);
                if (bIsOut)
                {
                    acurvec.Reverse();
                }

                double aDotX, anAngleX;
                //
                aDotX = aVec.Dot(acurvec);
                anAngleX = aVec.Angle(acurvec);
                //
                if (aDotX > 0.0 && Math.Abs(anAngleX) < Precision.PConfusion())
                {
                    if ((i % 2) == 0)
                    {
                        if ((acurpoint.Y() >= theVmin) && (acurpoint.Y() <= theVmax))
                        {
                            theNewPoint = acurpoint;
                            return true;
                        }
                    }
                    else
                    {
                        if ((acurpoint.X() >= theUmin) && (acurpoint.X() <= theUmax))
                        {
                            theNewPoint = acurpoint;
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
    public static bool DecompositionOfWLine(IntPatch_WLine theWLine, GeomAdaptor_Surface theSurface1,
                                            GeomAdaptor_Surface theSurface2, in TopoDS_Face theFace1,
                                            in TopoDS_Face theFace2, GeomInt_LineConstructor theLConstructor,
                                            bool theAvoidLConstructor, double theTol,
                                            NCollection_Sequence<IntPatch_Line> theNewLines, IntTools_Context aContext)
    {
        bool bRet, bAvoidLineConstructor;
        int aNbPnts, aNbParts;
        //
        bRet = false;
        aNbPnts = theWLine.NbPnts();
        bAvoidLineConstructor = theAvoidLConstructor;
        //
        if (aNbPnts == 0)
        {
            return bRet;
        }
        if (!bAvoidLineConstructor)
        {
            aNbParts = theLConstructor.NbParts();
            if (aNbParts == 0)
            {
                return bRet;
            }
        }
        //
        bool bIsPrevPointOnBoundary, bIsPointOnBoundary, bIsCurrentPointOnBoundary;
        int nblines, pit, i, j;
        double aTol;
        NCollection_Array1<NCollection_List<int>> anArrayOfLines = new(1, aNbPnts);
        NCollection_Array1<int> anArrayOfLineType = new(1, aNbPnts);
        NCollection_List<int> aListOfPointIndex = new();

        //
        nblines = 0;
        aTol = Precision.Confusion();
        aTol = 0.5 * aTol;
        bIsPrevPointOnBoundary = false;
        bIsPointOnBoundary = false;
        //
        // 1. ...
        //
        // Points
        for (pit = 1; pit <= aNbPnts; ++pit)
        {
            bool bIsOnFirstBoundary, isperiodic;
            double aResolution, aPeriod, alowerboundary, aupperboundary, U, V;
            double aParameter, anoffset, anAdjustPar;
            double umin, umax, vmin, vmax;
            //
            bIsCurrentPointOnBoundary = false;
            IntSurf_PntOn2S aPoint = theWLine.Point(pit);
            //
            // Surface
            for (i = 0; i < 2; ++i)
            {
                GeomAdaptor_Surface aGASurface = (i == 0) ? theSurface1 : theSurface2;
                aGASurface.Surface()!.Bounds(out umin, out umax, out vmin, out vmax);
                if (i == 0)
                {
                    aPoint.ParametersOnS1(out U, out V);
                }
                else
                {
                    aPoint.ParametersOnS2(out U, out V);
                }
                // U, V
                for (j = 0; j < 2; j++)
                {
                    isperiodic = (j == 0) ? aGASurface.IsUPeriodic() : aGASurface.IsVPeriodic();
                    if (!isperiodic)
                    {
                        continue;
                    }
                    //
                    if (j == 0)
                    {
                        aResolution = aGASurface.UResolution(aTol);
                        aPeriod = aGASurface.UPeriod();
                        alowerboundary = umin;
                        aupperboundary = umax;
                        aParameter = U;
                    }
                    else
                    {
                        aResolution = aGASurface.VResolution(aTol);
                        aPeriod = aGASurface.VPeriod();
                        alowerboundary = vmin;
                        aupperboundary = vmax;
                        aParameter = V;
                    }

                    GeomInt.AdjustPeriodic(aParameter, alowerboundary, aupperboundary, aPeriod, out anAdjustPar,
                                           out anoffset);
                    //
                    bIsPointOnBoundary = IsPointOnBoundary(anAdjustPar, alowerboundary, aupperboundary, aResolution,
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
                if (!aListOfPointIndex.IsEmpty())
                {
                    nblines++;
                    anArrayOfLines.SetValue(nblines, new NCollection_List<int>(aListOfPointIndex));
                    anArrayOfLineType.SetValue(nblines, bIsPrevPointOnBoundary ? 1 : 0);
                    aListOfPointIndex.Clear();
                }
                bIsPrevPointOnBoundary = bIsCurrentPointOnBoundary;
            }
            aListOfPointIndex.Append(pit);
        } // for(pit = 1; pit <= aNbPnts; ++pit) {
        //
        if (!aListOfPointIndex.IsEmpty())
        {
            nblines++;
            anArrayOfLines.SetValue(nblines, new NCollection_List<int>(aListOfPointIndex));
            anArrayOfLineType.SetValue(nblines, bIsPrevPointOnBoundary ? 1 : 0);
            aListOfPointIndex.Clear();
        }
        //
        if (nblines <= 1)
        {
            return bRet; // false;
        }
        //
        //
        // 2. Correct wlines.begin
        NCollection_Array1<NCollection_List<int>> anArrayOfLineEnds = new(1, nblines);
        IntSurf_LineOn2S aSeqOfPntOn2S = new();
        //
        for (i = 1; i <= nblines; i++)
        {
            if (anArrayOfLineType.Value(i) != 0)
            {
                continue;
            }
            NCollection_List<int> aListOfIndex = anArrayOfLines.Value(i);
            NCollection_List<int> aListOfFLIndex = new();

            for (j = 0; j < 2; j++)
            {
                int aneighbourindex = (j == 0) ? (i - 1) : (i + 1);

                if ((aneighbourindex < 1) || (aneighbourindex > nblines))
                {
                    continue;
                }

                if (anArrayOfLineType.Value(aneighbourindex) == 0)
                {
                    continue;
                }
                NCollection_List<int> aNeighbour = anArrayOfLines.Value(aneighbourindex);
                int anIndex = (j == 0) ? aNeighbour.Last() : aNeighbour.First();
                IntSurf_PntOn2S aPoint = theWLine.Point(anIndex);

                IntSurf_PntOn2S aNewP = aPoint;
                if (aListOfIndex.Extent() < 2)
                {
                    aSeqOfPntOn2S.Add(aNewP);
                    aListOfFLIndex.Append(aSeqOfPntOn2S.NbPoints());
                    continue;
                }
                //
                int iFirst = aListOfIndex.First();
                int iLast = aListOfIndex.Last();
                //
                for (int surfit = 0; surfit < 2; surfit++)
                {

                    GeomAdaptor_Surface aGASurface = (surfit == 0) ? theSurface1 : theSurface2;
                    double umin = 0.0, umax = 0.0, vmin = 0.0, vmax = 0.0;
                    aGASurface.Surface()!.Bounds(out umin, out umax, out vmin, out vmax);
                    double U = 0.0, V = 0.0;

                    if (surfit == 0)
                    {
                        aNewP.ParametersOnS1(out U, out V);
                    }
                    else
                    {
                        aNewP.ParametersOnS2(out U, out V);
                    }
                    int nbboundaries = 0;

                    bool bIsNearBoundary = false;
                    // int aZoneIndex = 0;
                    bool bIsUBoundary = false; // use if nbboundaries == 1
                    bool bIsFirstBoundary = false; // use if nbboundaries == 1

                    for (int parit = 0; parit < 2; parit++)
                    {
                        bool isperiodic = (parit == 0) ? aGASurface.IsUPeriodic() : aGASurface.IsVPeriodic();

                        double aResolution = (parit == 0) ? aGASurface.UResolution(aTol) : aGASurface.VResolution(aTol);
                        double alowerboundary = (parit == 0) ? umin : vmin;
                        double aupperboundary = (parit == 0) ? umax : vmax;

                        double aParameter = (parit == 0) ? U : V;
                        bool bIsOnFirstBoundary = true;

                        if (!isperiodic)
                        {
                            bIsPointOnBoundary = IsPointOnBoundary(aParameter, alowerboundary, aupperboundary,
                                                                   aResolution, out bIsOnFirstBoundary);
                            if (bIsPointOnBoundary)
                            {
                                bIsUBoundary = (parit == 0);
                                bIsFirstBoundary = bIsOnFirstBoundary;
                                nbboundaries++;
                            }
                        }
                        else
                        {
                            double aPeriod = (parit == 0) ? aGASurface.UPeriod() : aGASurface.VPeriod();
                            GeomInt.AdjustPeriodic(aParameter, alowerboundary, aupperboundary, aPeriod,
                                                   out double anAdjustPar, out _);

                            bIsPointOnBoundary = IsPointOnBoundary(anAdjustPar, alowerboundary, aupperboundary,
                                                                   aResolution, out bIsOnFirstBoundary);
                            if (bIsPointOnBoundary)
                            {
                                bIsUBoundary = (parit == 0);
                                bIsFirstBoundary = bIsOnFirstBoundary;
                                nbboundaries++;
                            }
                            else
                            {
                                // check neighbourhood of boundary
                                double anEpsilon = aResolution * 100.0;
                                double aPart = (aupperboundary - alowerboundary) * 0.1;
                                anEpsilon = (anEpsilon > aPart) ? aPart : anEpsilon;

                                bIsNearBoundary = IsPointOnBoundary(anAdjustPar, alowerboundary, aupperboundary,
                                                                    anEpsilon, out bIsOnFirstBoundary);
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
                        bool isperiodic = (bIsUBoundary) ? aGASurface.IsUPeriodic() : aGASurface.IsVPeriodic();

                        if (isperiodic)
                        {
                            double alowerboundary = (bIsUBoundary) ? umin : vmin;
                            double aupperboundary = (bIsUBoundary) ? umax : vmax;
                            double aPeriod = (bIsUBoundary) ? aGASurface.UPeriod() : aGASurface.VPeriod();
                            double aParameter = (bIsUBoundary) ? U : V;
                            GeomInt.AdjustPeriodic(aParameter, alowerboundary, aupperboundary, aPeriod,
                                                   out double anAdjustPar, out double anoffset);

                            double adist = (bIsFirstBoundary) ? Math.Abs(anAdjustPar - alowerboundary)
                                                              : Math.Abs(anAdjustPar - aupperboundary);
                            double anotherPar = (bIsFirstBoundary) ? (aupperboundary - adist) : (alowerboundary + adist);
                            anotherPar += anoffset;
                            int aneighbourpointindex = (j == 0) ? aListOfIndex.First() : aListOfIndex.Last();
                            IntSurf_PntOn2S aNeighbourPoint = theWLine.Point(aneighbourpointindex);
                            double nU1, nV1;

                            if (surfit == 0)
                            {
                                aNeighbourPoint.ParametersOnS1(out nU1, out nV1);
                            }
                            else
                            {
                                aNeighbourPoint.ParametersOnS2(out nU1, out nV1);
                            }

                            double adist1 = (bIsUBoundary) ? Math.Abs(nU1 - U) : Math.Abs(nV1 - V);
                            double adist2 = (bIsUBoundary) ? Math.Abs(nU1 - anotherPar) : Math.Abs(nV1 - anotherPar);
                            bComputeLineEnd = true;
                            bool bCheckAngle1 = false;
                            bool bCheckAngle2 = false;
                            gp_Vec2d aNewVec = new();
                            double anewU = (bIsUBoundary) ? anotherPar : U;
                            double anewV = (bIsUBoundary) ? V : anotherPar;

                            if (((adist1 - adist2) > Precision.PConfusion()) && (adist2 < (aPeriod / 4.0)))
                            {
                                bCheckAngle1 = true;
                                aNewVec = new gp_Vec2d(new gp_Pnt2d(nU1, nV1), new gp_Pnt2d(anewU, anewV));

                                if (aNewVec.SquareMagnitude() < gp.Resolution())
                                {
                                    aNewP.SetValue((surfit == 0), anewU, anewV);
                                    bCheckAngle1 = false;
                                }
                            }
                            else if (adist1 < (aPeriod / 4.0))
                            {
                                bCheckAngle2 = true;
                                aNewVec = new gp_Vec2d(new gp_Pnt2d(nU1, nV1), new gp_Pnt2d(U, V));

                                if (aNewVec.SquareMagnitude() < gp.Resolution())
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
                                    IntSurf_PntOn2S aPrevNeighbourPoint = theWLine.Point(anindexother);
                                    double nU2, nV2;

                                    if (surfit == 0)
                                    {
                                        aPrevNeighbourPoint.ParametersOnS1(out nU2, out nV2);
                                    }
                                    else
                                    {
                                        aPrevNeighbourPoint.ParametersOnS2(out nU2, out nV2);
                                    }
                                    gp_Vec2d aVecOld = new(new gp_Pnt2d(nU2, nV2), new gp_Pnt2d(nU1, nV1));

                                    if (aVecOld.SquareMagnitude() <= gp.Resolution())
                                    {
                                        continue;
                                    }
                                    else
                                    {
                                        double anAngle = aNewVec.Angle(aVecOld);

                                        if ((Math.Abs(anAngle) < (M_PI * 0.25)) && (aNewVec.Dot(aVecOld) > 0.0))
                                        {

                                            if (bCheckAngle1)
                                            {
                                                double U1, U2, V1, V2;
                                                IntSurf_PntOn2S atmppoint = aNewP;
                                                atmppoint.SetValue((surfit == 0), anewU, anewV);
                                                atmppoint.Parameters(out U1, out V1, out U2, out V2);
                                                gp_Pnt P1 = theSurface1.Value(U1, V1);
                                                gp_Pnt P2 = theSurface2.Value(U2, V2);
                                                gp_Pnt P0 = aPoint.Value();

                                                if (P0.IsEqual(P1, aTol) && P0.IsEqual(P2, aTol) && P1.IsEqual(P2, aTol))
                                                {
                                                    bComputeLineEnd = false;
                                                    aNewP.SetValue((surfit == 0), anewU, anewV);
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

                        gp_Pnt2d anewpoint = new();
                        bool found = false;

                        if (bIsNearBoundary)
                        {
                            // re-compute point near natural boundary or near tangent zone
                            double u1, v1, u2, v2;
                            aNewP.Parameters(out u1, out v1, out u2, out v2);
                            if (surfit == 0)
                            {
                                anewpoint = new gp_Pnt2d(u1, v1);
                            }
                            else
                            {
                                anewpoint = new gp_Pnt2d(u2, v2);
                            }

                            int aneighbourpointindex1 = (j == 0) ? iFirst : iLast;
                            IntSurf_PntOn2S aNeighbourPoint = theWLine.Point(aneighbourpointindex1);
                            double nU1, nV1;

                            if (surfit == 0)
                            {
                                aNeighbourPoint.ParametersOnS1(out nU1, out nV1);
                            }
                            else
                            {
                                aNeighbourPoint.ParametersOnS2(out nU1, out nV1);
                            }
                            gp_Pnt2d ap1 = new(nU1, nV1);
                            gp_Pnt2d ap2;

                            if (aGASurface.IsUPeriodic() || aGASurface.IsVPeriodic())
                            {
                                // re-compute point near boundary if shifted on a period
                                ap2 = AdjustByNeighbour(ap1, anewpoint, aGASurface);

                                if ((ap2.X() < umin) || (ap2.X() > umax) || (ap2.Y() < vmin) || (ap2.Y() > vmax))
                                {
                                    found = FindPoint(ap1, ap2, umin, umax, vmin, vmax, ref anewpoint);
                                }
                                else
                                {
                                    anewpoint = ap2;
                                    aNewP.SetValue((surfit == 0), anewpoint.X(), anewpoint.Y());
                                }
                            }
                        }
                        else
                        {

                            int aneighbourpointindex1 = (j == 0) ? iFirst : iLast;
                            IntSurf_PntOn2S aNeighbourPoint = theWLine.Point(aneighbourpointindex1);
                            double nU1, nV1;

                            if (surfit == 0)
                            {
                                aNeighbourPoint.ParametersOnS1(out nU1, out nV1);
                            }
                            else
                            {
                                aNeighbourPoint.ParametersOnS2(out nU1, out nV1);
                            }
                            gp_Pnt2d ap1 = new(nU1, nV1);
                            gp_Pnt2d ap2 = new(nU1, nV1);
                            int aneighbourpointindex2 = aneighbourpointindex1;

                            while ((aneighbourpointindex2 <= iLast) && (aneighbourpointindex2 >= iFirst))
                            {
                                aneighbourpointindex2 = (j == 0) ? (aneighbourpointindex2 + 1) : (aneighbourpointindex2 - 1);
                                IntSurf_PntOn2S aPrevNeighbourPoint = theWLine.Point(aneighbourpointindex2);
                                double nU2, nV2;

                                if (surfit == 0)
                                {
                                    aPrevNeighbourPoint.ParametersOnS1(out nU2, out nV2);
                                }
                                else
                                {
                                    aPrevNeighbourPoint.ParametersOnS2(out nU2, out nV2);
                                }
                                ap2.SetX(nU2);
                                ap2.SetY(nV2);

                                if (ap1.SquareDistance(ap2) > gp.Resolution())
                                {
                                    break;
                                }
                            }
                            found = FindPoint(ap2, ap1, umin, umax, vmin, vmax, ref anewpoint);
                        }

                        if (found)
                        {
                            // check point
                            double aCriteria = theTol;
                            GeomAPI_ProjectPointOnSurf aProjector =
                                (surfit == 0) ? aContext.ProjPS(theFace2) : aContext.ProjPS(theFace1);
                            GeomAdaptor_Surface aSurface = (surfit == 0) ? theSurface1 : theSurface2;

                            GeomAdaptor_Surface aSurfaceOther = (surfit == 0) ? theSurface2 : theSurface1;

                            gp_Pnt aP3d = aSurface.Value(anewpoint.X(), anewpoint.Y());
                            aProjector.Perform(aP3d);

                            if (aProjector.IsDone())
                            {
                                if (aProjector.LowerDistance() < aCriteria)
                                {
                                    double foundU = U, foundV = V;
                                    aProjector.LowerDistanceParameters(out foundU, out foundV);

                                    // Correction of projected coordinates. Begin
                                    // Note, it may be shifted on a period
                                    int aneindex1 = (j == 0) ? iFirst : iLast;
                                    IntSurf_PntOn2S aNeighbourPoint = theWLine.Point(aneindex1);
                                    double nUn, nVn;

                                    if (surfit == 0)
                                    {
                                        aNeighbourPoint.ParametersOnS2(out nUn, out nVn);
                                    }
                                    else
                                    {
                                        aNeighbourPoint.ParametersOnS1(out nUn, out nVn);
                                    }
                                    gp_Pnt2d aNeighbour2d = new(nUn, nVn);
                                    gp_Pnt2d anAdjustedPoint =
                                        AdjustByNeighbour(aNeighbour2d, new gp_Pnt2d(foundU, foundV), aSurfaceOther);
                                    foundU = anAdjustedPoint.X();
                                    foundV = anAdjustedPoint.Y();

                                    // OCCT: the condition can't hold (X < umin and X > umax), so the branch never runs
                                    if ((anAdjustedPoint.X() < umin) && (anAdjustedPoint.X() > umax)
                                        && (anAdjustedPoint.Y() < vmin) && (anAdjustedPoint.Y() > vmax))
                                    {
                                        // attempt to roughly re-compute point
                                        foundU = (foundU < umin) ? umin : foundU;
                                        foundU = (foundU > umax) ? umax : foundU;
                                        foundV = (foundV < vmin) ? vmin : foundV;
                                        foundV = (foundV > vmax) ? vmax : foundV;

                                        GeomAPI_ProjectPointOnSurf aProjector2 =
                                            (surfit == 0) ? aContext.ProjPS(theFace1) : aContext.ProjPS(theFace2);

                                        aP3d = aSurfaceOther.Value(foundU, foundV);
                                        aProjector2.Perform(aP3d);

                                        if (aProjector2.IsDone())
                                        {
                                            if (aProjector2.LowerDistance() < aCriteria)
                                            {
                                                aProjector2.LowerDistanceParameters(out double foundU2,
                                                                                    out double foundV2);
                                                anewpoint.SetX(foundU2);
                                                anewpoint.SetY(foundV2);
                                            }
                                        }
                                    }
                                    // Correction of projected coordinates. End

                                    if (surfit == 0)
                                    {
                                        aNewP.SetValue(aP3d, anewpoint.X(), anewpoint.Y(), foundU, foundV);
                                    }
                                    else
                                    {
                                        aNewP.SetValue(aP3d, foundU, foundV, anewpoint.X(), anewpoint.Y());
                                    }
                                }
                            }
                        }
                    }
                }
                aSeqOfPntOn2S.Add(aNewP);
                aListOfFLIndex.Append(aSeqOfPntOn2S.NbPoints());
            }
            anArrayOfLineEnds.SetValue(i, aListOfFLIndex);
        }
        // Correct wlines.end

        // Split wlines.begin
        int nbiter;
        //
        nbiter = 1;
        if (!bAvoidLineConstructor)
        {
            nbiter = theLConstructor.NbParts();
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
                ilprm = theWLine.NbPnts();
            }
            else
            {
                theLConstructor.Part(j, out fprm, out lprm);
                ifprm = (int)fprm;
                ilprm = (int)lprm;
            }

            IntSurf_LineOn2S aLineOn2S = new();
            //
            for (i = 1; i <= nblines; i++)
            {
                if (anArrayOfLineType.Value(i) != 0)
                {
                    continue;
                }
                NCollection_List<int> aListOfIndex = anArrayOfLines.Value(i);
                NCollection_List<int> aListOfFLIndex = anArrayOfLineEnds.Value(i);
                bool bhasfirstpoint = (aListOfFLIndex.Extent() == 2);
                bool bhaslastpoint = (aListOfFLIndex.Extent() == 2);

                if (!bhasfirstpoint && !aListOfFLIndex.IsEmpty())
                {
                    bhasfirstpoint = (i != 1);
                }

                if (!bhaslastpoint && !aListOfFLIndex.IsEmpty())
                {
                    bhaslastpoint = (i != nblines);
                }

                int iFirst = aListOfIndex.First();
                int iLast = aListOfIndex.Last();
                bool bIsFirstInside = ((ifprm >= iFirst) && (ifprm <= iLast));
                bool bIsLastInside = ((ilprm >= iFirst) && (ilprm <= iLast));

                if (!bIsFirstInside && !bIsLastInside)
                {
                    if ((ifprm < iFirst) && (ilprm > iLast))
                    {
                        // append whole line, and boundaries if necessary
                        if (bhasfirstpoint)
                        {
                            pit = aListOfFLIndex.First();
                            aLineOn2S.Add(aSeqOfPntOn2S.Value(pit));
                        }
                        NCollection_List<int>.Iterator anIt = new(aListOfIndex);

                        for (; anIt.More(); anIt.Next())
                        {
                            pit = anIt.Value();
                            aLineOn2S.Add(theWLine.Point(pit));
                        }

                        if (bhaslastpoint)
                        {
                            pit = aListOfFLIndex.Last();
                            aLineOn2S.Add(aSeqOfPntOn2S.Value(pit));
                        }

                        // check end of split line (end is almost always)
                        int aneighbour = i + 1;
                        bool bIsEndOfLine = true;

                        if (aneighbour <= nblines)
                        {
                            NCollection_List<int> aListOfNeighbourIndex = anArrayOfLines.Value(aneighbour);

                            if ((anArrayOfLineType.Value(aneighbour) != 0) && (aListOfNeighbourIndex.IsEmpty()))
                            {
                                bIsEndOfLine = false;
                            }
                        }

                        if (bIsEndOfLine)
                        {
                            if (aLineOn2S.NbPoints() > 1)
                            {
                                IntPatch_WLine aNewWLine = new(aLineOn2S, false);
                                aNewWLine.SetCreatingWayInfo(theWLine.GetCreatingWay());
                                theNewLines.Append(aNewWLine);
                            }
                            aLineOn2S = new IntSurf_LineOn2S();
                        }
                    }
                    continue;
                }
                // end if(!bIsFirstInside && !bIsLastInside)

                if (bIsFirstInside && bIsLastInside)
                {
                    // append inside points between ifprm and ilprm
                    NCollection_List<int>.Iterator anIt = new(aListOfIndex);

                    for (; anIt.More(); anIt.Next())
                    {
                        pit = anIt.Value();
                        if ((pit < ifprm) || (pit > ilprm))
                        {
                            continue;
                        }
                        aLineOn2S.Add(theWLine.Point(pit));
                    }
                }
                else
                {

                    if (bIsFirstInside)
                    {
                        // append points from ifprm to last point + boundary point
                        NCollection_List<int>.Iterator anIt = new(aListOfIndex);

                        for (; anIt.More(); anIt.Next())
                        {
                            pit = anIt.Value();
                            if (pit < ifprm)
                            {
                                continue;
                            }
                            aLineOn2S.Add(theWLine.Point(pit));
                        }

                        if (bhaslastpoint)
                        {
                            pit = aListOfFLIndex.Last();
                            aLineOn2S.Add(aSeqOfPntOn2S.Value(pit));
                        }
                        // check end of split line (end is almost always)
                        int aneighbour = i + 1;
                        bool bIsEndOfLine = true;

                        if (aneighbour <= nblines)
                        {
                            NCollection_List<int> aListOfNeighbourIndex = anArrayOfLines.Value(aneighbour);

                            if ((anArrayOfLineType.Value(aneighbour) != 0) && (aListOfNeighbourIndex.IsEmpty()))
                            {
                                bIsEndOfLine = false;
                            }
                        }

                        if (bIsEndOfLine)
                        {
                            if (aLineOn2S.NbPoints() > 1)
                            {
                                IntPatch_WLine aNewWLine = new(aLineOn2S, false);
                                aNewWLine.SetCreatingWayInfo(theWLine.GetCreatingWay());
                                theNewLines.Append(aNewWLine);
                            }
                            aLineOn2S = new IntSurf_LineOn2S();
                        }
                    }
                    // end if(bIsFirstInside)

                    if (bIsLastInside)
                    {
                        // append points from first boundary point to ilprm
                        if (bhasfirstpoint)
                        {
                            pit = aListOfFLIndex.First();
                            aLineOn2S.Add(aSeqOfPntOn2S.Value(pit));
                        }
                        NCollection_List<int>.Iterator anIt = new(aListOfIndex);

                        for (; anIt.More(); anIt.Next())
                        {
                            pit = anIt.Value();
                            if (pit > ilprm)
                            {
                                continue;
                            }
                            aLineOn2S.Add(theWLine.Point(pit));
                        }
                    }
                    // end if(bIsLastInside)
                }
            }

            if (aLineOn2S.NbPoints() > 1)
            {
                IntPatch_WLine aNewWLine = new(aLineOn2S, false);
                aNewWLine.SetCreatingWayInfo(theWLine.GetCreatingWay());
                theNewLines.Append(aNewWLine);
            }
        }
        // Split wlines.end

        return true;
    }

    ///////////////////// end of DecompositionOfWLine ///////////////////////
}
