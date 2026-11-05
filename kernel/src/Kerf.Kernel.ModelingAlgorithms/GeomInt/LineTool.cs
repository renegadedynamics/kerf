// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_LineTool.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomInt/GeomInt_LineTool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Extrema;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Patches;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Kerf.Engine;
using static Kerf.Engine.Intersection.Patches.PatchLineType;

namespace Kerf.Engine.Intersection.Surfaces;

/// <summary>
/// Reads the vertices and bounds of any intersection line, and splits a walking line at the seams of periodic
/// surfaces (<see cref="DecompositionOfWLine"/>) for GeomInt_IntSS's approximation.
/// </summary>
/// <remarks>
/// OCCT's file-local ProjectPointOnSurf is a private nested class; its static functions are private static methods.
/// The NCollection_IncAllocator arguments are dropped, and the vector of index lists is a .NET array of lists.
/// </remarks>
internal static class LineTool
{
    private sealed class ProjectPointOnSurf
    {
        private bool _isDone;
        private int _index;
        private readonly ExtPS _extPS = new();
        private readonly SurfaceAdaptor _geomAdaptor = new();

        public ProjectPointOnSurf()
        {
            _isDone = false;
            _index = 0;
        }

        public void Init(Surface surface, double umin, double usup, double vmin, double vsup)
        {
            double tolerance = global::Kerf.Engine.Foundation.Precision.PConfusion();
            //
            _geomAdaptor.Load(surface, umin, usup, vmin, vsup);
            _extPS.Initialize(_geomAdaptor, umin, usup, vmin, vsup, tolerance, tolerance);
            _isDone = false;
        }

        public void Init()
        {
            _isDone = _extPS.IsDone && (_extPS.ExtCount > 0);
            if (_isDone)
            {
                // evaluate the lower distance and its index;
                double dist2Min = _extPS.SquareDistance(1);
                _index = 1;
                for (int i = 2; i <= _extPS.ExtCount; i++)
                {
                    double dist2 = _extPS.SquareDistance(i);
                    if (dist2 < dist2Min)
                    {
                        dist2Min = dist2;
                        _index = i;
                    }
                }
            }
        }

        public void Perform(in Point3D p)
        {
            _extPS.Perform(p);
            Init();
        }

        public bool IsDone => _isDone;

        public void LowerDistanceParameters(out double u, out double v)
        {
            StdFail_NotDone_Raise_if(!_isDone, "GeomInt_IntSS::ProjectPointOnSurf::LowerDistanceParameters");
            (_extPS.Point(_index)).Parameter(out u, out v);
        }

        public double LowerDistance()
        {
            StdFail_NotDone_Raise_if(!_isDone, "GeomInt_IntSS::ProjectPointOnSurf::LowerDistance");
            return Math.Sqrt(_extPS.SquareDistance(_index));
        }
    }

    private static double AdjustPeriodic(double parameter, double parmin, double parmax, double period,
                                         out double offset)
    {
        double aresult = parameter;
        offset = 0.0;
        while (aresult < parmin)
        {
            aresult += period;
            offset += period;
        }
        while (aresult > parmax)
        {
            aresult -= period;
            offset -= period;
        }
        return aresult;
    }

    private static bool IsPointOnBoundary(double parameter, double firstBoundary, double secondBoundary,
                                          double resolution, out bool isOnFirstBoundary)
    {
        isOnFirstBoundary = true;
        if (Math.Abs(parameter - firstBoundary) < resolution)
        {
            return true;
        }
        if (Math.Abs(parameter - secondBoundary) < resolution)
        {
            isOnFirstBoundary = false;
            return true;
        }
        return false;
    }

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
            double adot1 = anormvec.Dot(otherVecNormal);

            if (Math.Abs(adot1) < global::Kerf.Engine.Foundation.Precision.Angular())
            {
                continue;
            }
            double adist = 0.0;

            if ((i % 2) == 0)
            {
                adist = (i < 2) ? Math.Abs(lastPoint.X - umin) : Math.Abs(lastPoint.X - umax);
            }
            else
            {
                adist = (i < 2) ? Math.Abs(lastPoint.Y - vmin) : Math.Abs(lastPoint.Y - vmax);
            }
            double anoffset = adist * otherVec.Dot(anormvec) / adot1;

            for (j = 0; j < 2; j++)
            {
                anoffset = (j == 0) ? anoffset : -anoffset;
                Point2D acurpoint = new(aprojpoint.XY() + (otherVec.XY() * anoffset));
                Vector2D acurvec = new(lastPoint, acurpoint);

                //
                double dotX, angleX, pc;
                //
                dotX = vec.Dot(acurvec);
                angleX = vec.AngleTo(acurvec);
                pc = global::Kerf.Engine.Foundation.Precision.PConfusion();
                //
                if (dotX > 0.0 && Math.Abs(angleX) < pc)
                {
                    //
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

    /// <summary>The number of vertices of the line, whatever its type.</summary>
    public static int NbVertex(PatchLine l)
    {
        switch (l.ArcType())
        {
            case PatchLineType.Analytic:
                return (l as ALine)!.VertexCount;
            case PatchLineType.Restriction:
                return (l as RLine)!.VertexCount;
            case PatchLineType.Walking:
                return (l as WLine)!.VertexCount;
            default:
                break;
        }
        return (l as GLine)!.VertexCount;
    }

    /// <summary>The vertex of index I of the line (a copy), whatever its type.</summary>
    public static PatchPoint Vertex(PatchLine l, int i)
    {
        switch (l.ArcType())
        {
            case PatchLineType.Analytic:
                return (l as ALine)!.Vertex(i);
            case PatchLineType.Restriction:
                return (l as RLine)!.Vertex(i);
            case PatchLineType.Walking:
                return (l as WLine)!.Vertex(i);
            default:
                break;
        }
        return (l as GLine)!.Vertex(i);
    }

    /// <summary>The first parameter of the line: its first vertex's, or the start of its natural domain.</summary>
    public static double FirstParameter(PatchLine l)
    {
        PatchLineType typl = l.ArcType();
        switch (typl)
        {
            case PatchLineType.Analytic:
            {
                ALine alin = (l as ALine)!;
                if (alin.HasFirstPoint)
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

            case PatchLineType.Restriction:
            {
                RLine rlin = (l as RLine)!;
                // a voir selon le type de la ligne 2d
                return (rlin.HasFirstPoint ? rlin.FirstPoint().ParameterOnLine() : -global::Kerf.Engine.Foundation.Precision.Infinite());
            }

            case PatchLineType.Walking:
            {
                WLine wlin = (l as WLine)!;
                return (wlin.HasFirstPoint ? wlin.FirstPoint().ParameterOnLine() : 1.0);
            }

            default:
            {
                GLine glin = (l as GLine)!;
                if (glin.HasFirstPoint)
                {
                    return glin.FirstPoint().ParameterOnLine();
                }
                switch (typl)
                {
                    case PatchLineType.Lin:
                    case PatchLineType.Parabola:
                    case PatchLineType.Hyperbola:
                        return -global::Kerf.Engine.Foundation.Precision.Infinite();
                    default:
                        break;
                }
                break;
            }
        }
        return 0.0;
    }

    /// <summary>The last parameter of the line: its last vertex's, or the end of its natural domain.</summary>
    public static double LastParameter(PatchLine l)
    {
        PatchLineType typl = l.ArcType();
        switch (typl)
        {
            case PatchLineType.Analytic:
            {
                ALine alin = (l as ALine)!;
                if (alin.HasLastPoint)
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

            case PatchLineType.Restriction:
            {
                RLine rlin = (l as RLine)!;
                // a voir selon le type de la ligne 2d
                return (rlin.HasLastPoint ? rlin.LastPoint().ParameterOnLine() : global::Kerf.Engine.Foundation.Precision.Infinite());
            }

            case PatchLineType.Walking:
            {
                WLine wlin = (l as WLine)!;
                return (wlin.HasLastPoint ? wlin.LastPoint().ParameterOnLine() : wlin.NbPnts());
            }

            default:
            {
                GLine glin = (l as GLine)!;
                if (glin.HasLastPoint)
                {
                    return glin.LastPoint().ParameterOnLine();
                }
                switch (typl)
                {
                    case PatchLineType.Lin:
                    case PatchLineType.Parabola:
                    case PatchLineType.Hyperbola:
                        return global::Kerf.Engine.Foundation.Precision.Infinite();
                    case PatchLineType.Circle:
                    case PatchLineType.Ellipse:
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
    public static bool DecompositionOfWLine(WLine wLine, SurfaceAdaptor surface1,
                                            SurfaceAdaptor surface2, double aTolSum,
                                            LineConstructor lConstructor,
                                            NCollection_Sequence<PatchLine> newLines)
    {
        bool bIsPrevPointOnBoundary, bIsCurrentPointOnBoundary;
        int nblines, nbPnts, nbParts, pit, i, j, nbListOfPointIndex;
        double tol, umin, umax, vmin, vmax;

        NCollection_List<int> listOfPointIndex = new();

        // GeomAPI_ProjectPointOnSurf aPrj1, aPrj2;
        ProjectPointOnSurf prj1 = new(), prj2 = new();
        Surface surf1, surf2;
        //
        nbParts = lConstructor.PartCount;
        nbPnts = wLine.NbPnts();
        //
        if ((nbPnts == 0) || (nbParts == 0))
        {
            return false;
        }
        //
        // OCCT's vector of lists, each copy-constructed from an empty list.
        NCollection_List<int>[] arrayOfLines = new NCollection_List<int>[nbPnts + 1];
        for (int k = 0; k <= nbPnts; k++)
        {
            arrayOfLines[k] = new NCollection_List<int>();
        }

        int[] arrayOfLineType = new int[nbPnts + 1];
        //
        nblines = 0;
        tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        //
        surf1 = surface1.Surface()!;
        surf1.ParameterBounds(out umin, out umax, out vmin, out vmax);
        prj1.Init(surf1, umin, umax, vmin, vmax);
        //
        surf2 = surface2.Surface()!;
        surf2.ParameterBounds(out umin, out umax, out vmin, out vmax);
        prj2.Init(surf2, umin, umax, vmin, vmax);
        //
        //
        bIsPrevPointOnBoundary = false;
        for (pit = 1; pit <= nbPnts; pit++)
        {
            PntOn2S point = wLine.Point(pit);
            bIsCurrentPointOnBoundary = false;
            //
            // whether aPoint is on boundary or not
            //
            for (i = 0; i < 2; i++)
            { // exploration Surface 1,2
                SurfaceAdaptor gaSurface = (i == 0) ? surface1 : surface2;
                gaSurface.Surface()!.ParameterBounds(out umin, out umax, out vmin, out vmax);
                //
                for (j = 0; j < 2; j++)
                { // exploration of coordinate U,V
                    bool isperiodic;
                    //
                    isperiodic = (j == 0) ? gaSurface.IsUPeriodic : gaSurface.IsVPeriodic;
                    if (!isperiodic)
                    {
                        continue;
                    }
                    //
                    double resolution, period, alowerboundary, aupperboundary, u, v;
                    double parameter, anoffset, adjustPar;
                    bool bIsOnFirstBoundary, bIsPointOnBoundary;
                    //
                    resolution = (j == 0) ? gaSurface.UResolution(tol) : gaSurface.VResolution(tol);
                    period = (j == 0) ? gaSurface.UPeriod() : gaSurface.VPeriod();
                    alowerboundary = (j == 0) ? umin : vmin;
                    aupperboundary = (j == 0) ? umax : vmax;
                    //
                    if (i == 0)
                    {
                        point.ParametersOnS1(out u, out v);
                    }
                    else
                    {
                        point.ParametersOnS2(out u, out v);
                    }
                    //
                    parameter = (j == 0) ? u : v;
                    adjustPar = AdjustPeriodic(parameter, alowerboundary, aupperboundary, period, out anoffset);
                    //
                    bIsPointOnBoundary = IsPointOnBoundary(adjustPar, alowerboundary, aupperboundary, resolution,
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

                if (!listOfPointIndex.IsEmpty())
                {
                    nblines++;
                    arrayOfLines[nblines] = new NCollection_List<int>(listOfPointIndex); // C++'s copy assignment
                    arrayOfLineType[nblines] = bIsPrevPointOnBoundary ? 1 : 0;
                    listOfPointIndex.Clear();
                }
                bIsPrevPointOnBoundary = bIsCurrentPointOnBoundary;
            }
            listOfPointIndex.Append(pit);
        } // for(pit=1; pit<=aNbPnts; pit++)
        //
        nbListOfPointIndex = listOfPointIndex.Extent();
        if (nbListOfPointIndex != 0)
        {
            nblines++;
            arrayOfLines[nblines] = new NCollection_List<int>(listOfPointIndex);
            arrayOfLineType[nblines] = bIsPrevPointOnBoundary ? 1 : 0;
            listOfPointIndex.Clear();
        }
        //
        if (nblines <= 1)
        {
            return false;
        }
        //
        // Correct wlines.begin
        int lineType;
        NCollection_Array1<NCollection_List<int>> arrayOfLineEnds = new(1, nblines);
        LineOn2S seqOfPntOn2S = new();
        //
        for (i = 1; i <= nblines; i++)
        {
            lineType = arrayOfLineType[i];
            if (lineType != 0)
            {
                continue;
            }
            //
            NCollection_List<int> listOfIndex = arrayOfLines[i];
            if (listOfIndex.Extent() < 2)
            {
                continue;
            }
            //
            NCollection_List<int> listOfFLIndex = new();
            int aneighbourindex, lineTypeNeib;
            //
            for (j = 0; j < 2; j++)
            { // neighbour line choice
                aneighbourindex = (j == 0) ? (i - 1) : (i + 1);
                if ((aneighbourindex < 1) || (aneighbourindex > nblines))
                {
                    continue;
                }
                //
                lineTypeNeib = arrayOfLineType[aneighbourindex];
                if (lineTypeNeib == 0)
                {
                    continue;
                }
                //
                NCollection_List<int> neighbour = arrayOfLines[aneighbourindex];
                int anIndex = (j == 0) ? neighbour.Last() : neighbour.First();
                PntOn2S point = wLine.Point(anIndex);
                // check if need use derivative.begin .end [absence]
                //
                PntOn2S newP = point;
                int surfit, parit;
                //
                for (surfit = 0; surfit < 2; ++surfit)
                {

                    SurfaceAdaptor gaSurface = (surfit == 0) ? surface1 : surface2;

                    umin = gaSurface.FirstUParameter();
                    umax = gaSurface.LastUParameter();
                    vmin = gaSurface.FirstVParameter();
                    vmax = gaSurface.LastVParameter();
                    double u, v;

                    if (surfit == 0)
                    {
                        newP.ParametersOnS1(out u, out v);
                    }
                    else
                    {
                        newP.ParametersOnS2(out u, out v);
                    }
                    //
                    int nbboundaries = 0;
                    bool bIsUBoundary = false; // use if nbboundaries == 1
                    bool bIsFirstBoundary = false; // use if nbboundaries == 1
                    //
                    for (parit = 0; parit < 2; parit++)
                    {
                        bool isperiodic = (parit == 0) ? gaSurface.IsUPeriodic : gaSurface.IsVPeriodic;

                        double resolution = (parit == 0) ? gaSurface.UResolution(tol) : gaSurface.VResolution(tol);
                        double alowerboundary = (parit == 0) ? umin : vmin;
                        double aupperboundary = (parit == 0) ? umax : vmax;

                        double parameter = (parit == 0) ? u : v;
                        bool bIsOnFirstBoundary;

                        if (!isperiodic)
                        {
                            if (IsPointOnBoundary(parameter, alowerboundary, aupperboundary, resolution,
                                                  out bIsOnFirstBoundary))
                            {
                                bIsUBoundary = (parit == 0);
                                bIsFirstBoundary = bIsOnFirstBoundary;
                                nbboundaries++;
                            }
                        }
                        else
                        {
                            double period = (parit == 0) ? gaSurface.UPeriod() : gaSurface.VPeriod();
                            double adjustPar = AdjustPeriodic(parameter, alowerboundary, aupperboundary, period,
                                                                out _);

                            if (IsPointOnBoundary(adjustPar, alowerboundary, aupperboundary, resolution,
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
                        bool isperiodic = (bIsUBoundary) ? gaSurface.IsUPeriodic : gaSurface.IsVPeriodic;

                        if (isperiodic)
                        {
                            double alowerboundary = (bIsUBoundary) ? umin : vmin;
                            double aupperboundary = (bIsUBoundary) ? umax : vmax;
                            double period = (bIsUBoundary) ? gaSurface.UPeriod() : gaSurface.VPeriod();
                            double parameter = (bIsUBoundary) ? u : v;
                            double adjustPar = AdjustPeriodic(parameter, alowerboundary, aupperboundary, period,
                                                                out double anoffset);

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
                            //
                            if (((adist1 - adist2) > global::Kerf.Engine.Foundation.Precision.PConfusion()) && (adist2 < (period / 4.0)))
                            {
                                bCheckAngle1 = true;
                                newVec = new Vector2D(new Point2D(nU1, nV1), new Point2D(anewU, anewV));

                                if (newVec.SquareMagnitude < (Constants.Resolution() * Constants.Resolution()))
                                {
                                    newP.SetValue((surfit == 0), anewU, anewV);
                                    bCheckAngle1 = false;
                                }
                            }
                            else if (adist1 < (period / 4.0))
                            {
                                bCheckAngle2 = true;
                                newVec = new Vector2D(new Point2D(nU1, nV1), new Point2D(u, v));

                                if (newVec.SquareMagnitude < (Constants.Resolution() * Constants.Resolution()))
                                {
                                    bCheckAngle2 = false;
                                }
                            }
                            //
                            if (bCheckAngle1 || bCheckAngle2)
                            {
                                // assume there are at least two points in line (see "if" above)
                                int anindexother = aneighbourpointindex;

                                while ((anindexother <= listOfIndex.Last()) && (anindexother >= listOfIndex.First()))
                                {
                                    anindexother = (j == 0) ? (anindexother + 1) : (anindexother - 1);
                                    // OCCT reads one point past the part when all its points coincide; the part is
                                    // inside the line except at the line's ends (then the port's Point throws).
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

                                    if (vecOld.SquareMagnitude <= (Constants.Resolution() * Constants.Resolution()))
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
                                                double u1, u2, v1, v2;
                                                PntOn2S atmppoint = newP;
                                                atmppoint.SetValue((surfit == 0), anewU, anewV);
                                                atmppoint.Parameters(out u1, out v1, out u2, out v2);
                                                Point3D P1 = surface1.Value(u1, v1);
                                                Point3D P2 = surface2.Value(u2, v2);
                                                Point3D p0 = point.Value();

                                                if (p0.IsEqual(P1, tol) && p0.IsEqual(P2, tol) && P1.IsEqual(P2, tol))
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
                    //
                    if (bComputeLineEnd)
                    {
                        int aneighbourpointindex1 = (j == 0) ? listOfIndex.First() : listOfIndex.Last();
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

                        while ((aneighbourpointindex2 <= listOfIndex.Last())
                               && (aneighbourpointindex2 >= listOfIndex.First()))
                        {
                            aneighbourpointindex2 = (j == 0) ? (aneighbourpointindex2 + 1) : (aneighbourpointindex2 - 1);
                            // OCCT: may read past the part as above, when all its points coincide in this space.
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

                            if (ap1.SquareDistanceTo(ap2) > (Constants.Resolution() * Constants.Resolution()))
                            {
                                break;
                            }
                        }
                        Point2D anewpoint = new();
                        bool found = FindPoint(ap2, ap1, umin, umax, vmin, vmax, ref anewpoint);

                        if (found)
                        {
                            // check point
                            double criteria = aTolSum; // BRep_Tool::Tolerance(theFace1) + BRep_Tool::Tolerance(theFace2);
                            //GeomAPI_ProjectPointOnSurf& aProjector = (surfit == 0) ? aPrj2 : aPrj1;
                            ProjectPointOnSurf projector = (surfit == 0) ? prj2 : prj1;
                            SurfaceAdaptor surface = (surfit == 0) ? surface1 : surface2;

                            Point3D p3d = surface.Value(anewpoint.X, anewpoint.Y);
                            projector.Perform(p3d);

                            if (projector.IsDone)
                            {
                                if (projector.LowerDistance() < criteria)
                                {
                                    projector.LowerDistanceParameters(out double foundU, out double foundV);

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
        for (j = 1; j <= lConstructor.PartCount; j++)
        {
            lConstructor.Part(j, out double fprm, out double lprm);
            int ifprm = (int)fprm;
            int ilprm = (int)lprm;
            //
            LineOn2S lineOn2S = new();
            //
            for (i = 1; i <= nblines; i++)
            {
                if (arrayOfLineType[i] != 0)
                {
                    continue;
                }
                NCollection_List<int> listOfIndex = arrayOfLines[i];

                if (listOfIndex.Extent() < 2)
                {
                    continue;
                }
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
                bool bIsFirstInside = ((ifprm >= listOfIndex.First()) && (ifprm <= listOfIndex.Last()));
                bool bIsLastInside = ((ilprm >= listOfIndex.First()) && (ilprm <= listOfIndex.Last()));

                if (!bIsFirstInside && !bIsLastInside)
                {
                    if ((ifprm < listOfIndex.First()) && (ilprm > listOfIndex.Last()))
                    {
                        // append whole line, and boundaries if necessary
                        if (bhasfirstpoint)
                        {
                            PntOn2S p = seqOfPntOn2S.Value(listOfFLIndex.First());
                            lineOn2S.Add(p);
                        }
                        foreach (int anIndex in listOfIndex)
                        {
                            PntOn2S p = wLine.Point(anIndex);
                            lineOn2S.Add(p);
                        }

                        if (bhaslastpoint)
                        {
                            PntOn2S p = seqOfPntOn2S.Value(listOfFLIndex.Last());
                            lineOn2S.Add(p);
                        }

                        // check end of split line (end is almost always)
                        int aneighbour = i + 1;
                        bool bIsEndOfLine = true;

                        if (aneighbour <= nblines)
                        {
                            NCollection_List<int> listOfNeighbourIndex = arrayOfLines[aneighbour];

                            if ((arrayOfLineType[aneighbour] != 0) && (listOfNeighbourIndex.IsEmpty()))
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
                    foreach (int anIndex in listOfIndex)
                    {
                        if ((anIndex < ifprm) || (anIndex > ilprm))
                        {
                            continue;
                        }
                        PntOn2S p = wLine.Point(anIndex);
                        lineOn2S.Add(p);
                    }
                }
                else
                {

                    if (bIsFirstInside)
                    {
                        // append points from ifprm to last point + boundary point
                        foreach (int anIndex in listOfIndex)
                        {
                            if (anIndex < ifprm)
                            {
                                continue;
                            }
                            PntOn2S p = wLine.Point(anIndex);
                            lineOn2S.Add(p);
                        }

                        if (bhaslastpoint)
                        {
                            PntOn2S p = seqOfPntOn2S.Value(listOfFLIndex.Last());
                            lineOn2S.Add(p);
                        }
                        // check end of split line (end is almost always)
                        int aneighbour = i + 1;
                        bool bIsEndOfLine = true;

                        if (aneighbour <= nblines)
                        {
                            NCollection_List<int> listOfNeighbourIndex = arrayOfLines[aneighbour];

                            if ((arrayOfLineType[aneighbour] != 0) && (listOfNeighbourIndex.IsEmpty()))
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
                            PntOn2S p = seqOfPntOn2S.Value(listOfFLIndex.First());
                            lineOn2S.Add(p);
                        }
                        foreach (int anIndex in listOfIndex)
                        {
                            if (anIndex > ilprm)
                            {
                                continue;
                            }
                            PntOn2S p = wLine.Point(anIndex);
                            lineOn2S.Add(p);
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
        //
        // cda002/I3
        nbParts = lConstructor.PartCount;
        //
        for (j = 1; j <= nbParts; j++)
        {
            lConstructor.Part(j, out double fprm, out double lprm);
            int ifprm = (int)fprm;
            int ilprm = (int)lprm;
            //
            if ((ilprm - ifprm) == 1)
            {
                for (i = 1; i <= nblines; i++)
                {
                    lineType = arrayOfLineType[i];
                    if (lineType != 0)
                    {
                        continue;
                    }
                    //
                    NCollection_List<int> listOfIndex = arrayOfLines[i];
                    int nbPoints = listOfIndex.Extent();
                    if (nbPoints == 1)
                    {
                        int index = listOfIndex.First();
                        if (index == ifprm || index == ilprm)
                        {
                            LineOn2S lineOn2S = new();
                            PntOn2S p1 = wLine.Point(ifprm);
                            PntOn2S p2 = wLine.Point(ilprm);
                            lineOn2S.Add(p1);
                            lineOn2S.Add(p2);
                            WLine newWLine = new(lineOn2S, false);
                            newWLine.SetCreatingWayInfo(wLine.GetCreatingWay());
                            newLines.Append(newWLine);
                        }
                    }
                }
            }
        }
        //
        return true;
    }
}
