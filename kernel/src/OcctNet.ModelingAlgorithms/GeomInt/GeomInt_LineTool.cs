// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_LineTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_LineTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntPatch_IType;

namespace Occt;

/// <summary>
/// Reads the vertices and bounds of any intersection line, and splits a walking line at the seams of periodic
/// surfaces (<see cref="DecompositionOfWLine"/>) for GeomInt_IntSS's approximation.
/// </summary>
/// <remarks>
/// OCCT's file-local ProjectPointOnSurf is a private nested class; its static functions are private static methods.
/// The NCollection_IncAllocator arguments are dropped, and the vector of index lists is a .NET array of lists.
/// </remarks>
public static class GeomInt_LineTool
{
    private sealed class ProjectPointOnSurf
    {
        private bool myIsDone;
        private int myIndex;
        private readonly Extrema_ExtPS myExtPS = new();
        private readonly GeomAdaptor_Surface myGeomAdaptor = new();

        public ProjectPointOnSurf()
        {
            myIsDone = false;
            myIndex = 0;
        }

        public void Init(Geom_Surface Surface, double Umin, double Usup, double Vmin, double Vsup)
        {
            double Tolerance = Precision.PConfusion();
            //
            myGeomAdaptor.Load(Surface, Umin, Usup, Vmin, Vsup);
            myExtPS.Initialize(myGeomAdaptor, Umin, Usup, Vmin, Vsup, Tolerance, Tolerance);
            myIsDone = false;
        }

        public void Init()
        {
            myIsDone = myExtPS.IsDone() && (myExtPS.NbExt() > 0);
            if (myIsDone)
            {
                // evaluate the lower distance and its index;
                double Dist2Min = myExtPS.SquareDistance(1);
                myIndex = 1;
                for (int i = 2; i <= myExtPS.NbExt(); i++)
                {
                    double Dist2 = myExtPS.SquareDistance(i);
                    if (Dist2 < Dist2Min)
                    {
                        Dist2Min = Dist2;
                        myIndex = i;
                    }
                }
            }
        }

        public void Perform(in gp_Pnt P)
        {
            myExtPS.Perform(P);
            Init();
        }

        public bool IsDone() => myIsDone;

        public void LowerDistanceParameters(out double U, out double V)
        {
            StdFail_NotDone_Raise_if(!myIsDone, "GeomInt_IntSS::ProjectPointOnSurf::LowerDistanceParameters");
            (myExtPS.Point(myIndex)).Parameter(out U, out V);
        }

        public double LowerDistance()
        {
            StdFail_NotDone_Raise_if(!myIsDone, "GeomInt_IntSS::ProjectPointOnSurf::LowerDistance");
            return Math.Sqrt(myExtPS.SquareDistance(myIndex));
        }
    }

    private static double AdjustPeriodic(double theParameter, double parmin, double parmax, double thePeriod,
                                         out double theOffset)
    {
        double aresult = theParameter;
        theOffset = 0.0;
        while (aresult < parmin)
        {
            aresult += thePeriod;
            theOffset += thePeriod;
        }
        while (aresult > parmax)
        {
            aresult -= thePeriod;
            theOffset -= thePeriod;
        }
        return aresult;
    }

    private static bool IsPointOnBoundary(double theParameter, double theFirstBoundary, double theSecondBoundary,
                                          double theResolution, out bool IsOnFirstBoundary)
    {
        IsOnFirstBoundary = true;
        if (Math.Abs(theParameter - theFirstBoundary) < theResolution)
        {
            return true;
        }
        if (Math.Abs(theParameter - theSecondBoundary) < theResolution)
        {
            IsOnFirstBoundary = false;
            return true;
        }
        return false;
    }

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
            double adot1 = anormvec.Dot(anOtherVecNormal);

            if (Math.Abs(adot1) < Precision.Angular())
            {
                continue;
            }
            double adist = 0.0;

            if ((i % 2) == 0)
            {
                adist = (i < 2) ? Math.Abs(theLastPoint.X() - theUmin) : Math.Abs(theLastPoint.X() - theUmax);
            }
            else
            {
                adist = (i < 2) ? Math.Abs(theLastPoint.Y() - theVmin) : Math.Abs(theLastPoint.Y() - theVmax);
            }
            double anoffset = adist * anOtherVec.Dot(anormvec) / adot1;

            for (j = 0; j < 2; j++)
            {
                anoffset = (j == 0) ? anoffset : -anoffset;
                gp_Pnt2d acurpoint = new(aprojpoint.XY() + (anOtherVec.XY() * anoffset));
                gp_Vec2d acurvec = new(theLastPoint, acurpoint);

                //
                double aDotX, anAngleX, aPC;
                //
                aDotX = aVec.Dot(acurvec);
                anAngleX = aVec.Angle(acurvec);
                aPC = Precision.PConfusion();
                //
                if (aDotX > 0.0 && Math.Abs(anAngleX) < aPC)
                {
                    //
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

    /// <summary>The number of vertices of the line, whatever its type.</summary>
    public static int NbVertex(IntPatch_Line L)
    {
        switch (L.ArcType())
        {
            case IntPatch_Analytic:
                return (L as IntPatch_ALine)!.NbVertex();
            case IntPatch_Restriction:
                return (L as IntPatch_RLine)!.NbVertex();
            case IntPatch_Walking:
                return (L as IntPatch_WLine)!.NbVertex();
            default:
                break;
        }
        return (L as IntPatch_GLine)!.NbVertex();
    }

    /// <summary>The vertex of index I of the line (a copy), whatever its type.</summary>
    public static IntPatch_Point Vertex(IntPatch_Line L, int I)
    {
        switch (L.ArcType())
        {
            case IntPatch_Analytic:
                return (L as IntPatch_ALine)!.Vertex(I);
            case IntPatch_Restriction:
                return (L as IntPatch_RLine)!.Vertex(I);
            case IntPatch_Walking:
                return (L as IntPatch_WLine)!.Vertex(I);
            default:
                break;
        }
        return (L as IntPatch_GLine)!.Vertex(I);
    }

    /// <summary>The first parameter of the line: its first vertex's, or the start of its natural domain.</summary>
    public static double FirstParameter(IntPatch_Line L)
    {
        IntPatch_IType typl = L.ArcType();
        switch (typl)
        {
            case IntPatch_Analytic:
            {
                IntPatch_ALine alin = (L as IntPatch_ALine)!;
                if (alin.HasFirstPoint())
                {
                    return alin.FirstPoint().ParameterOnLine();
                }
                double firstp = alin.FirstParameter(out bool included);
                if (!included)
                {
                    firstp += Epsilon(firstp);
                }
                return firstp;
            }

            case IntPatch_Restriction:
            {
                IntPatch_RLine rlin = (L as IntPatch_RLine)!;
                // a voir selon le type de la ligne 2d
                return (rlin.HasFirstPoint() ? rlin.FirstPoint().ParameterOnLine() : -Precision.Infinite());
            }

            case IntPatch_Walking:
            {
                IntPatch_WLine wlin = (L as IntPatch_WLine)!;
                return (wlin.HasFirstPoint() ? wlin.FirstPoint().ParameterOnLine() : 1.0);
            }

            default:
            {
                IntPatch_GLine glin = (L as IntPatch_GLine)!;
                if (glin.HasFirstPoint())
                {
                    return glin.FirstPoint().ParameterOnLine();
                }
                switch (typl)
                {
                    case IntPatch_Lin:
                    case IntPatch_Parabola:
                    case IntPatch_Hyperbola:
                        return -Precision.Infinite();
                    default:
                        break;
                }
                break;
            }
        }
        return 0.0;
    }

    /// <summary>The last parameter of the line: its last vertex's, or the end of its natural domain.</summary>
    public static double LastParameter(IntPatch_Line L)
    {
        IntPatch_IType typl = L.ArcType();
        switch (typl)
        {
            case IntPatch_Analytic:
            {
                IntPatch_ALine alin = (L as IntPatch_ALine)!;
                if (alin.HasLastPoint())
                {
                    return alin.LastPoint().ParameterOnLine();
                }
                double lastp = alin.LastParameter(out bool included);
                if (!included)
                {
                    lastp -= Epsilon(lastp);
                }
                return lastp;
            }

            case IntPatch_Restriction:
            {
                IntPatch_RLine rlin = (L as IntPatch_RLine)!;
                // a voir selon le type de la ligne 2d
                return (rlin.HasLastPoint() ? rlin.LastPoint().ParameterOnLine() : Precision.Infinite());
            }

            case IntPatch_Walking:
            {
                IntPatch_WLine wlin = (L as IntPatch_WLine)!;
                return (wlin.HasLastPoint() ? wlin.LastPoint().ParameterOnLine() : wlin.NbPnts());
            }

            default:
            {
                IntPatch_GLine glin = (L as IntPatch_GLine)!;
                if (glin.HasLastPoint())
                {
                    return glin.LastPoint().ParameterOnLine();
                }
                switch (typl)
                {
                    case IntPatch_Lin:
                    case IntPatch_Parabola:
                    case IntPatch_Hyperbola:
                        return Precision.Infinite();
                    case IntPatch_Circle:
                    case IntPatch_Ellipse:
                        return 2.0 * M_PI;
                    default:
                        break;
                }
                break;
            }
        }
        return 0.0;
    }

    /// <summary>
    /// Splits theWLine where it crosses the seams of periodic surfaces (points on a boundary of a periodic parameter
    /// start a new part), moves the ends of the inner parts onto the boundaries, and keeps the parts inside the line
    /// constructor's parts as new walking lines in theNewLines. Returns false if the line isn't split.
    /// </summary>
    public static bool DecompositionOfWLine(IntPatch_WLine theWLine, GeomAdaptor_Surface theSurface1,
                                            GeomAdaptor_Surface theSurface2, double aTolSum,
                                            GeomInt_LineConstructor theLConstructor,
                                            NCollection_Sequence<IntPatch_Line> theNewLines)
    {
        bool bIsPrevPointOnBoundary, bIsCurrentPointOnBoundary;
        int nblines, aNbPnts, aNbParts, pit, i, j, aNbListOfPointIndex;
        double aTol, umin, umax, vmin, vmax;

        NCollection_List<int> aListOfPointIndex = new();

        // GeomAPI_ProjectPointOnSurf aPrj1, aPrj2;
        ProjectPointOnSurf aPrj1 = new(), aPrj2 = new();
        Geom_Surface aSurf1, aSurf2;
        //
        aNbParts = theLConstructor.NbParts();
        aNbPnts = theWLine.NbPnts();
        //
        if ((aNbPnts == 0) || (aNbParts == 0))
        {
            return false;
        }
        //
        // OCCT's vector of lists, each copy-constructed from an empty list.
        NCollection_List<int>[] anArrayOfLines = new NCollection_List<int>[aNbPnts + 1];
        for (int k = 0; k <= aNbPnts; k++)
        {
            anArrayOfLines[k] = new NCollection_List<int>();
        }

        int[] anArrayOfLineType = new int[aNbPnts + 1];
        //
        nblines = 0;
        aTol = Precision.Confusion();
        //
        aSurf1 = theSurface1.Surface()!;
        aSurf1.Bounds(out umin, out umax, out vmin, out vmax);
        aPrj1.Init(aSurf1, umin, umax, vmin, vmax);
        //
        aSurf2 = theSurface2.Surface()!;
        aSurf2.Bounds(out umin, out umax, out vmin, out vmax);
        aPrj2.Init(aSurf2, umin, umax, vmin, vmax);
        //
        //
        bIsPrevPointOnBoundary = false;
        for (pit = 1; pit <= aNbPnts; pit++)
        {
            IntSurf_PntOn2S aPoint = theWLine.Point(pit);
            bIsCurrentPointOnBoundary = false;
            //
            // whether aPoint is on boundary or not
            //
            for (i = 0; i < 2; i++)
            { // exploration Surface 1,2
                GeomAdaptor_Surface aGASurface = (i == 0) ? theSurface1 : theSurface2;
                aGASurface.Surface()!.Bounds(out umin, out umax, out vmin, out vmax);
                //
                for (j = 0; j < 2; j++)
                { // exploration of coordinate U,V
                    bool isperiodic;
                    //
                    isperiodic = (j == 0) ? aGASurface.IsUPeriodic() : aGASurface.IsVPeriodic();
                    if (!isperiodic)
                    {
                        continue;
                    }
                    //
                    double aResolution, aPeriod, alowerboundary, aupperboundary, U, V;
                    double aParameter, anoffset, anAdjustPar;
                    bool bIsOnFirstBoundary, bIsPointOnBoundary;
                    //
                    aResolution = (j == 0) ? aGASurface.UResolution(aTol) : aGASurface.VResolution(aTol);
                    aPeriod = (j == 0) ? aGASurface.UPeriod() : aGASurface.VPeriod();
                    alowerboundary = (j == 0) ? umin : vmin;
                    aupperboundary = (j == 0) ? umax : vmax;
                    //
                    if (i == 0)
                    {
                        aPoint.ParametersOnS1(out U, out V);
                    }
                    else
                    {
                        aPoint.ParametersOnS2(out U, out V);
                    }
                    //
                    aParameter = (j == 0) ? U : V;
                    anAdjustPar = AdjustPeriodic(aParameter, alowerboundary, aupperboundary, aPeriod, out anoffset);
                    //
                    bIsPointOnBoundary = IsPointOnBoundary(anAdjustPar, alowerboundary, aupperboundary, aResolution,
                                                           out bIsOnFirstBoundary);

                    if (bIsPointOnBoundary)
                    {
                        bIsCurrentPointOnBoundary = true;
                        break;
                    }
                } // for(j=0; j<2; j++)

                if (bIsCurrentPointOnBoundary)
                {
                    break;
                }
            } // for(i=0; i<2; i++)
            //
            if ((bIsCurrentPointOnBoundary != bIsPrevPointOnBoundary))
            {

                if (!aListOfPointIndex.IsEmpty())
                {
                    nblines++;
                    anArrayOfLines[nblines] = new NCollection_List<int>(aListOfPointIndex); // C++'s copy assignment
                    anArrayOfLineType[nblines] = bIsPrevPointOnBoundary ? 1 : 0;
                    aListOfPointIndex.Clear();
                }
                bIsPrevPointOnBoundary = bIsCurrentPointOnBoundary;
            }
            aListOfPointIndex.Append(pit);
        } // for(pit=1; pit<=aNbPnts; pit++)
        //
        aNbListOfPointIndex = aListOfPointIndex.Extent();
        if (aNbListOfPointIndex != 0)
        {
            nblines++;
            anArrayOfLines[nblines] = new NCollection_List<int>(aListOfPointIndex);
            anArrayOfLineType[nblines] = bIsPrevPointOnBoundary ? 1 : 0;
            aListOfPointIndex.Clear();
        }
        //
        if (nblines <= 1)
        {
            return false;
        }
        //
        // Correct wlines.begin
        int aLineType;
        NCollection_Array1<NCollection_List<int>> anArrayOfLineEnds = new(1, nblines);
        IntSurf_LineOn2S aSeqOfPntOn2S = new();
        //
        for (i = 1; i <= nblines; i++)
        {
            aLineType = anArrayOfLineType[i];
            if (aLineType != 0)
            {
                continue;
            }
            //
            NCollection_List<int> aListOfIndex = anArrayOfLines[i];
            if (aListOfIndex.Extent() < 2)
            {
                continue;
            }
            //
            NCollection_List<int> aListOfFLIndex = new();
            int aneighbourindex, aLineTypeNeib;
            //
            for (j = 0; j < 2; j++)
            { // neighbour line choice
                aneighbourindex = (j == 0) ? (i - 1) : (i + 1);
                if ((aneighbourindex < 1) || (aneighbourindex > nblines))
                {
                    continue;
                }
                //
                aLineTypeNeib = anArrayOfLineType[aneighbourindex];
                if (aLineTypeNeib == 0)
                {
                    continue;
                }
                //
                NCollection_List<int> aNeighbour = anArrayOfLines[aneighbourindex];
                int anIndex = (j == 0) ? aNeighbour.Last() : aNeighbour.First();
                IntSurf_PntOn2S aPoint = theWLine.Point(anIndex);
                // check if need use derivative.begin .end [absence]
                //
                IntSurf_PntOn2S aNewP = aPoint;
                int surfit, parit;
                //
                for (surfit = 0; surfit < 2; ++surfit)
                {

                    GeomAdaptor_Surface aGASurface = (surfit == 0) ? theSurface1 : theSurface2;

                    umin = aGASurface.FirstUParameter();
                    umax = aGASurface.LastUParameter();
                    vmin = aGASurface.FirstVParameter();
                    vmax = aGASurface.LastVParameter();
                    double U, V;

                    if (surfit == 0)
                    {
                        aNewP.ParametersOnS1(out U, out V);
                    }
                    else
                    {
                        aNewP.ParametersOnS2(out U, out V);
                    }
                    //
                    int nbboundaries = 0;
                    bool bIsUBoundary = false; // use if nbboundaries == 1
                    bool bIsFirstBoundary = false; // use if nbboundaries == 1
                    //
                    for (parit = 0; parit < 2; parit++)
                    {
                        bool isperiodic = (parit == 0) ? aGASurface.IsUPeriodic() : aGASurface.IsVPeriodic();

                        double aResolution = (parit == 0) ? aGASurface.UResolution(aTol) : aGASurface.VResolution(aTol);
                        double alowerboundary = (parit == 0) ? umin : vmin;
                        double aupperboundary = (parit == 0) ? umax : vmax;

                        double aParameter = (parit == 0) ? U : V;
                        bool bIsOnFirstBoundary;

                        if (!isperiodic)
                        {
                            if (IsPointOnBoundary(aParameter, alowerboundary, aupperboundary, aResolution,
                                                  out bIsOnFirstBoundary))
                            {
                                bIsUBoundary = (parit == 0);
                                bIsFirstBoundary = bIsOnFirstBoundary;
                                nbboundaries++;
                            }
                        }
                        else
                        {
                            double aPeriod = (parit == 0) ? aGASurface.UPeriod() : aGASurface.VPeriod();
                            double anAdjustPar = AdjustPeriodic(aParameter, alowerboundary, aupperboundary, aPeriod,
                                                                out _);

                            if (IsPointOnBoundary(anAdjustPar, alowerboundary, aupperboundary, aResolution,
                                                  out bIsOnFirstBoundary))
                            {
                                bIsUBoundary = (parit == 0);
                                bIsFirstBoundary = bIsOnFirstBoundary;
                                nbboundaries++;
                            }
                        }
                    }
                    //
                    bool bComputeLineEnd = false;

                    if (nbboundaries == 2)
                    {
                        bComputeLineEnd = true;
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
                            double anAdjustPar = AdjustPeriodic(aParameter, alowerboundary, aupperboundary, aPeriod,
                                                                out double anoffset);

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
                            //
                            if (((adist1 - adist2) > Precision.PConfusion()) && (adist2 < (aPeriod / 4.0)))
                            {
                                bCheckAngle1 = true;
                                aNewVec = new gp_Vec2d(new gp_Pnt2d(nU1, nV1), new gp_Pnt2d(anewU, anewV));

                                if (aNewVec.SquareMagnitude() < (gp.Resolution() * gp.Resolution()))
                                {
                                    aNewP.SetValue((surfit == 0), anewU, anewV);
                                    bCheckAngle1 = false;
                                }
                            }
                            else if (adist1 < (aPeriod / 4.0))
                            {
                                bCheckAngle2 = true;
                                aNewVec = new gp_Vec2d(new gp_Pnt2d(nU1, nV1), new gp_Pnt2d(U, V));

                                if (aNewVec.SquareMagnitude() < (gp.Resolution() * gp.Resolution()))
                                {
                                    bCheckAngle2 = false;
                                }
                            }
                            //
                            if (bCheckAngle1 || bCheckAngle2)
                            {
                                // assume there are at least two points in line (see "if" above)
                                int anindexother = aneighbourpointindex;

                                while ((anindexother <= aListOfIndex.Last()) && (anindexother >= aListOfIndex.First()))
                                {
                                    anindexother = (j == 0) ? (anindexother + 1) : (anindexother - 1);
                                    // OCCT reads one point past the part when all its points coincide; the part is
                                    // inside the line except at the line's ends (then the port's Point throws).
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

                                    if (aVecOld.SquareMagnitude() <= (gp.Resolution() * gp.Resolution()))
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
                    //
                    if (bComputeLineEnd)
                    {
                        int aneighbourpointindex1 = (j == 0) ? aListOfIndex.First() : aListOfIndex.Last();
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

                        while ((aneighbourpointindex2 <= aListOfIndex.Last())
                               && (aneighbourpointindex2 >= aListOfIndex.First()))
                        {
                            aneighbourpointindex2 = (j == 0) ? (aneighbourpointindex2 + 1) : (aneighbourpointindex2 - 1);
                            // OCCT: may read past the part as above, when all its points coincide in this space.
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

                            if (ap1.SquareDistance(ap2) > (gp.Resolution() * gp.Resolution()))
                            {
                                break;
                            }
                        }
                        gp_Pnt2d anewpoint = new();
                        bool found = FindPoint(ap2, ap1, umin, umax, vmin, vmax, ref anewpoint);

                        if (found)
                        {
                            // check point
                            double aCriteria = aTolSum; // BRep_Tool::Tolerance(theFace1) + BRep_Tool::Tolerance(theFace2);
                            //GeomAPI_ProjectPointOnSurf& aProjector = (surfit == 0) ? aPrj2 : aPrj1;
                            ProjectPointOnSurf aProjector = (surfit == 0) ? aPrj2 : aPrj1;
                            GeomAdaptor_Surface aSurface = (surfit == 0) ? theSurface1 : theSurface2;

                            gp_Pnt aP3d = aSurface.Value(anewpoint.X(), anewpoint.Y());
                            aProjector.Perform(aP3d);

                            if (aProjector.IsDone())
                            {
                                if (aProjector.LowerDistance() < aCriteria)
                                {
                                    aProjector.LowerDistanceParameters(out double foundU, out double foundV);

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
        for (j = 1; j <= theLConstructor.NbParts(); j++)
        {
            theLConstructor.Part(j, out double fprm, out double lprm);
            int ifprm = (int)fprm;
            int ilprm = (int)lprm;
            //
            IntSurf_LineOn2S aLineOn2S = new();
            //
            for (i = 1; i <= nblines; i++)
            {
                if (anArrayOfLineType[i] != 0)
                {
                    continue;
                }
                NCollection_List<int> aListOfIndex = anArrayOfLines[i];

                if (aListOfIndex.Extent() < 2)
                {
                    continue;
                }
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
                bool bIsFirstInside = ((ifprm >= aListOfIndex.First()) && (ifprm <= aListOfIndex.Last()));
                bool bIsLastInside = ((ilprm >= aListOfIndex.First()) && (ilprm <= aListOfIndex.Last()));

                if (!bIsFirstInside && !bIsLastInside)
                {
                    if ((ifprm < aListOfIndex.First()) && (ilprm > aListOfIndex.Last()))
                    {
                        // append whole line, and boundaries if necessary
                        if (bhasfirstpoint)
                        {
                            IntSurf_PntOn2S aP = aSeqOfPntOn2S.Value(aListOfFLIndex.First());
                            aLineOn2S.Add(aP);
                        }
                        foreach (int anIndex in aListOfIndex)
                        {
                            IntSurf_PntOn2S aP = theWLine.Point(anIndex);
                            aLineOn2S.Add(aP);
                        }

                        if (bhaslastpoint)
                        {
                            IntSurf_PntOn2S aP = aSeqOfPntOn2S.Value(aListOfFLIndex.Last());
                            aLineOn2S.Add(aP);
                        }

                        // check end of split line (end is almost always)
                        int aneighbour = i + 1;
                        bool bIsEndOfLine = true;

                        if (aneighbour <= nblines)
                        {
                            NCollection_List<int> aListOfNeighbourIndex = anArrayOfLines[aneighbour];

                            if ((anArrayOfLineType[aneighbour] != 0) && (aListOfNeighbourIndex.IsEmpty()))
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
                    foreach (int anIndex in aListOfIndex)
                    {
                        if ((anIndex < ifprm) || (anIndex > ilprm))
                        {
                            continue;
                        }
                        IntSurf_PntOn2S aP = theWLine.Point(anIndex);
                        aLineOn2S.Add(aP);
                    }
                }
                else
                {

                    if (bIsFirstInside)
                    {
                        // append points from ifprm to last point + boundary point
                        foreach (int anIndex in aListOfIndex)
                        {
                            if (anIndex < ifprm)
                            {
                                continue;
                            }
                            IntSurf_PntOn2S aP = theWLine.Point(anIndex);
                            aLineOn2S.Add(aP);
                        }

                        if (bhaslastpoint)
                        {
                            IntSurf_PntOn2S aP = aSeqOfPntOn2S.Value(aListOfFLIndex.Last());
                            aLineOn2S.Add(aP);
                        }
                        // check end of split line (end is almost always)
                        int aneighbour = i + 1;
                        bool bIsEndOfLine = true;

                        if (aneighbour <= nblines)
                        {
                            NCollection_List<int> aListOfNeighbourIndex = anArrayOfLines[aneighbour];

                            if ((anArrayOfLineType[aneighbour] != 0) && (aListOfNeighbourIndex.IsEmpty()))
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
                            IntSurf_PntOn2S aP = aSeqOfPntOn2S.Value(aListOfFLIndex.First());
                            aLineOn2S.Add(aP);
                        }
                        foreach (int anIndex in aListOfIndex)
                        {
                            if (anIndex > ilprm)
                            {
                                continue;
                            }
                            IntSurf_PntOn2S aP = theWLine.Point(anIndex);
                            aLineOn2S.Add(aP);
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
        //
        // cda002/I3
        aNbParts = theLConstructor.NbParts();
        //
        for (j = 1; j <= aNbParts; j++)
        {
            theLConstructor.Part(j, out double fprm, out double lprm);
            int ifprm = (int)fprm;
            int ilprm = (int)lprm;
            //
            if ((ilprm - ifprm) == 1)
            {
                for (i = 1; i <= nblines; i++)
                {
                    aLineType = anArrayOfLineType[i];
                    if (aLineType != 0)
                    {
                        continue;
                    }
                    //
                    NCollection_List<int> aListOfIndex = anArrayOfLines[i];
                    int aNbPoints = aListOfIndex.Extent();
                    if (aNbPoints == 1)
                    {
                        int aIndex = aListOfIndex.First();
                        if (aIndex == ifprm || aIndex == ilprm)
                        {
                            IntSurf_LineOn2S aLineOn2S = new();
                            IntSurf_PntOn2S aP1 = theWLine.Point(ifprm);
                            IntSurf_PntOn2S aP2 = theWLine.Point(ilprm);
                            aLineOn2S.Add(aP1);
                            aLineOn2S.Add(aP2);
                            IntPatch_WLine aNewWLine = new(aLineOn2S, false);
                            aNewWLine.SetCreatingWayInfo(theWLine.GetCreatingWay());
                            theNewLines.Append(aNewWLine);
                        }
                    }
                }
            }
        }
        //
        return true;
    }
}
