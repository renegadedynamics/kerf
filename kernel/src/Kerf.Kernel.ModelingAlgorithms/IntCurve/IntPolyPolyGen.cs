// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntPolyPolyGen.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Implicit;
using Kerf.Engine.Intersection.Polygons;
using Kerf.Engine.Intersection.TwoD;
using Kerf.Geometry;

namespace Kerf.Engine.Intersection.Curves;

/// <summary>
/// The intersection of two bounded 2D curves, or the self-intersection of one, through the interference of their
/// polygons refined into exact points and tangent segments.
/// </summary>
/// <remarks>
/// OCCT's generic file IntCurve_IntPolyPolyGen.gxx, ported once as a generic class (decision 324); its
/// instantiation Geom2dInt_TheIntPCurvePCurveOfGInter is a sealed subclass. The file-static functions
/// <c>GetIntersection</c> and <c>HeadOrEndPoint</c> are private static methods.
/// </remarks>
internal class IntPolyPolyGen<TheCurve, TheCurveTool, TheProjPCur> : Intersection2D
    where TheCurveTool : ICurveCurveTool<TheCurve>
    where TheProjPCur : IProjectOnPCurveTool<TheCurve>
{
    private const int NBITER_MAX_POLYGON = 10;
    private const double TOL_CONF_MINI = 0.0000000001;
    private const double TOL_MINI = 0.0000000001;

    private TwoDDomain _domainOnCurve1 = new();
    private TwoDDomain _domainOnCurve2 = new();
    private int _minPntNb;

    public IntPolyPolyGen()
    {
        const int minPntNb = 20; // Minimum number of samples.
        _minPntNb = minPntNb;
        _done = false;
    }

    /// <summary>The intersection of two curves on their (bounded) domains.</summary>
    public void Perform(in TheCurve c1, in TwoDDomain d1, in TheCurve c2, in TwoDDomain d2,
                        double theTolConf, double theTol)
    {
        this.ResetFields();
        _domainOnCurve1 = d1;
        _domainOnCurve2 = d2;
        double du = d1.LastParameter() - d1.FirstParameter();
        double dv = d2.LastParameter() - d2.FirstParameter();
        double tl = (theTol < TOL_MINI) ? TOL_MINI : theTol;
        double tlConf = (theTolConf < TOL_CONF_MINI) ? TOL_CONF_MINI : theTolConf;
        Perform(c1, d1, c2, d2, tlConf, tl, 0, du, dv);
        //----------------------------------------------------------------------
        //-- Processing of end points
        //----------------------------------------------------------------------
        bool headOn1 = false;
        bool headOn2 = false;
        bool endOn1 = false;
        bool endOn2 = false;
        int i;
        int n = this.PointCount;

        //--------------------------------------------------------------------
        //-- The points Head Head ... End End are not rejected if
        //-- they are already present at the end of segment
        //-- ( It is not possible to test the equities on the parameters)
        //-- ( these points are not found at EpsX precision    )
        //-- PosSegment =            1    if Head Head
        //--                       2      if Head End
        //--                     4        if End  Head
        //--                   8          if End  End
        //--------------------------------------------------------------------
        int posSegment = 0;

        for (i = 1; i <= n; i++)
        {
            Position2D pos1 = this.Point(i).TransitionOfFirst().PositionOnCurve();
            if (pos1 == Position2D.Head)
            {
                headOn1 = true;
            }
            else if (pos1 == Position2D.End)
            {
                endOn1 = true;
            }

            Position2D pos2 = this.Point(i).TransitionOfSecond().PositionOnCurve();
            if (pos2 == Position2D.Head)
            {
                headOn2 = true;
            }
            else if (pos2 == Position2D.End)
            {
                endOn2 = true;
            }

            posSegment = AddPosSegment(posSegment, pos1, pos2);
        }

        n = this.SegmentCount;
        for (i = 1; i <= n; i++)
        {
            Position2D pos1 = this.Segment(i).FirstPoint().TransitionOfFirst().PositionOnCurve();
            if (pos1 == Position2D.Head)
            {
                headOn1 = true;
            }
            else if (pos1 == Position2D.End)
            {
                endOn1 = true;
            }

            Position2D pos2 = this.Segment(i).FirstPoint().TransitionOfSecond().PositionOnCurve();
            if (pos2 == Position2D.Head)
            {
                headOn2 = true;
            }
            else if (pos2 == Position2D.End)
            {
                endOn2 = true;
            }

            posSegment = AddPosSegment(posSegment, pos1, pos2);

            pos1 = this.Segment(i).LastPoint().TransitionOfFirst().PositionOnCurve();
            if (pos1 == Position2D.Head)
            {
                headOn1 = true;
            }
            else if (pos1 == Position2D.End)
            {
                endOn1 = true;
            }

            pos2 = this.Segment(i).LastPoint().TransitionOfSecond().PositionOnCurve();
            if (pos2 == Position2D.Head)
            {
                headOn2 = true;
            }
            else if (pos2 == Position2D.End)
            {
                endOn2 = true;
            }

            posSegment = AddPosSegment(posSegment, pos1, pos2);
        }

        double u0 = d1.FirstParameter();
        double u1 = d1.LastParameter();
        double v0 = d2.FirstParameter();
        double v1 = d2.LastParameter();
        IntersectionPoint2D intPt = new();

        if (d1.FirstTolerance() != 0 || d2.FirstTolerance() != 0)
        {
            if (HeadOrEndPoint(d1, c1, u0, d2, c2, v0, theTolConf, ref intPt, ref headOn1, ref headOn2, ref endOn1,
                               ref endOn2, posSegment))
            {
                this.Insert(intPt);
            }
        }
        if (d1.FirstTolerance() != 0 || d2.LastTolerance() != 0)
        {
            if (HeadOrEndPoint(d1, c1, u0, d2, c2, v1, theTolConf, ref intPt, ref headOn1, ref headOn2, ref endOn1,
                               ref endOn2, posSegment))
            {
                this.Insert(intPt);
            }
        }
        if (d1.LastTolerance() != 0 || d2.FirstTolerance() != 0)
        {
            if (HeadOrEndPoint(d1, c1, u1, d2, c2, v0, theTolConf, ref intPt, ref headOn1, ref headOn2, ref endOn1,
                               ref endOn2, posSegment))
            {
                this.Insert(intPt);
            }
        }
        if (d1.LastTolerance() != 0 || d2.LastTolerance() != 0)
        {
            if (HeadOrEndPoint(d1, c1, u1, d2, c2, v1, theTolConf, ref intPt, ref headOn1, ref headOn2, ref endOn1,
                               ref endOn2, posSegment))
            {
                this.Insert(intPt);
            }
        }
    }

    // The PosSegment flags of a point with the positions Pos1 and Pos2 (written out four times in OCCT).
    private static int AddPosSegment(int posSegment, Position2D pos1, Position2D pos2)
    {
        if (pos1 == Position2D.Head)
        {
            if (pos2 == Position2D.Head)
            {
                posSegment |= 1;
            }
            else if (pos2 == Position2D.End)
            {
                posSegment |= 2;
            }
        }
        else if (pos1 == Position2D.End)
        {
            if (pos2 == Position2D.Head)
            {
                posSegment |= 4;
            }
            else if (pos2 == Position2D.End)
            {
                posSegment |= 8;
            }
        }
        return posSegment;
    }

    //======================================================================
    //==      A u t o   I n t e r s e c t i o  n
    //======================================================================
    /// <summary>The self-intersection of a curve on its (bounded) domain.</summary>
    public void Perform(in TheCurve c1, in TwoDDomain d1, double theTolConf, double theTol)
    {
        this.ResetFields();
        _domainOnCurve1 = d1;
        _domainOnCurve2 = d1;
        double du = d1.LastParameter() - d1.FirstParameter();
        double tl = (theTol < TOL_MINI) ? TOL_MINI : theTol;
        double tlConf = (theTolConf < TOL_CONF_MINI) ? TOL_CONF_MINI : theTolConf;
        Perform(c1, d1, tlConf, tl, 0, du, du);
        // OCCT then computes the PosSegment flags of the points and segments found, and discards them.
    }

    /// <summary>The self-intersection of a curve: an iteration of the polygon interference.</summary>
    protected void Perform(in TheCurve c1, in TwoDDomain d1, double tolConf, double tol, int nbIter,
                           double deltaU, double deltaV)
    {
        Vector2D tan1, tan2, norm1, norm2;
        Point2D p1, p2;
        int nbsamples;
        _done = false;

        nbsamples = TheCurveTool.NbSamples(c1, d1.FirstParameter(), d1.LastParameter());

        if (nbIter > 3 || (nbIter > 2 && nbsamples > 100))
        {
            return;
        }

        nbsamples *= 2; //---  We take systematically two times more points
                        //--   than on a normal curve.
                        //--   Auto-intersecting curves often produce
                        //--   polygons rather far from the curve with parameter ct.

        if (nbIter > 0)
        {
            nbsamples = (3 * (nbsamples * nbIter)) / 2;
        }
        Polygon2dGen<TheCurve, TheCurveTool> poly1 = new(c1, nbsamples, d1, tol);
        if (!poly1.AutoIntersectionIsPossible())
        {
            _done = true;
            return;
        }
        //----------------------------------------------------------------------
        //-- If the deflection is less than the Tolerance of Confusion
        //-- then the deflection of the polygon is set in TolConf
        //-- (Detection of Tangency Zones)
        //----------------------------------------------------------------------
        if (poly1.DeflectionOverEstimation() < tolConf)
        {
            poly1.SetDeflectionOverEstimation(tolConf);
        }

        InterferencePolygon2d interPP = new(poly1);
        ExactIntersectionPoint<TheCurve, TheCurveTool> eip = new(c1, c1, tolConf);
        double u, v;

        //----------------------------------------------------------------------
        //-- Processing of SectionPoint
        //----------------------------------------------------------------------
        int nbsp = interPP.SectionPointCount;
        if (nbsp >= 1)
        {
            //-- ---------------------------------------------------------------------
            //-- filtering, filtering, filtering ...
            //--
            int[] triIndex = new int[nbsp + 1];
            int[] ptrSegIndex1 = new int[nbsp + 1];
            int[] ptrSegIndex2 = new int[nbsp + 1];
            bool triok;
            int segIndex1, segIndex2, SegIndex_1, SegIndex_2;
            double paramOn1, paramOn2;
            PIType type;
            int i;
            for (i = 1; i <= nbsp; i++)
            {
                triIndex[i] = i;
                SectionPoint sPnt1 = interPP.PntValue(i);
                sPnt1.InfoFirst(out type, out ptrSegIndex1[i], out paramOn1);
                sPnt1.InfoSecond(out type, out ptrSegIndex2[i], out paramOn2);
            }

            do
            {
                triok = true;

                for (int tr = 1; tr < nbsp; tr++)
                {
                    segIndex1 = ptrSegIndex1[triIndex[tr]];
                    SegIndex_1 = ptrSegIndex1[triIndex[tr + 1]];

                    segIndex2 = ptrSegIndex2[triIndex[tr]];
                    SegIndex_2 = ptrSegIndex2[triIndex[tr + 1]];

                    if (segIndex1 > SegIndex_1)
                    {
                        int q = triIndex[tr];
                        triIndex[tr] = triIndex[tr + 1];
                        triIndex[tr + 1] = q;
                        triok = false;
                    }
                    else if (segIndex1 == SegIndex_1)
                    {
                        if (segIndex2 > SegIndex_2)
                        {
                            int q = triIndex[tr];
                            triIndex[tr] = triIndex[tr + 1];
                            triIndex[tr + 1] = q;
                            triok = false;
                        }
                    }
                }
            } while (triok == false);

            //-- supression des doublons Si Si !
            for (i = 1; i < nbsp; i++)
            {
                if ((ptrSegIndex1[triIndex[i]] == ptrSegIndex1[triIndex[i + 1]])
                    && (ptrSegIndex2[triIndex[i]] == ptrSegIndex2[triIndex[i + 1]]))
                {
                    triIndex[i] = -i;
                }
            }

            for (int sp = 1; sp <= nbsp; sp++)
            {
                if (triIndex[sp] > 0)
                {
                    SectionPoint sPnt = interPP.PntValue(triIndex[sp]);

                    sPnt.InfoFirst(out type, out segIndex1, out paramOn1);
                    sPnt.InfoSecond(out type, out segIndex2, out paramOn2);

                    if (Math.Abs(segIndex1 - segIndex2) > 1)
                    {
                        eip.Perform(poly1, poly1, ref segIndex1, ref segIndex2, ref paramOn1, ref paramOn2);
                        if (eip.RootCount >= 1)
                        {
                            //--------------------------------------------------------------------
                            //-- It is checked if the found point is a root
                            //--------------------------------------------------------------------
                            eip.Roots(out u, out v);

                            TheCurveTool.D1(c1, u, out p1, out tan1);
                            TheCurveTool.D1(c1, v, out p2, out tan2);
                            double dist = p1.DistanceTo(p2);
                            double epsX1 = 10.0 * TheCurveTool.EpsX(c1);

                            if (Math.Abs(u - v) <= epsX1)
                            {
                                //-----------------------------------------
                                //-- Solution not valid
                                //-- The maths should have converged in a
                                //-- trivial solution  ( point U = V )
                                //-----------------------------------------
                                dist = tolConf + 1.0;
                            }

                            //-----------------------------------------------------------------
                            //-- It is checked if the point (u,v) already exists
                            //--
                            _done = true;
                            int nbp = PointCount;

                            for (int p = 1; p <= nbp; p++)
                            {
                                IntersectionPoint2D P = Point(p);
                                if (Math.Abs(u - P.ParamOnFirst()) <= epsX1)
                                {
                                    if (Math.Abs(v - P.ParamOnSecond()) <= epsX1)
                                    {
                                        dist = tolConf + 1.0;
                                        p += nbp;
                                    }
                                }
                            }

                            if (dist <= tolConf)
                            { //-- Or the point is already present
                                Position2D pos1 = Position2D.Middle;
                                Position2D pos2 = Position2D.Middle;
                                Transition trans1 = new(), trans2 = new();
                                //-----------------------------------------------------------------
                                //-- Calculate Positions of Points on the curve
                                //--
                                if (p1.DistanceTo(_domainOnCurve1.FirstPoint()) <= _domainOnCurve1.FirstTolerance())
                                {
                                    pos1 = Position2D.Head;
                                }
                                else if (p1.DistanceTo(_domainOnCurve1.LastPoint()) <= _domainOnCurve1.LastTolerance())
                                {
                                    pos1 = Position2D.End;
                                }

                                if (p2.DistanceTo(_domainOnCurve2.FirstPoint()) <= _domainOnCurve2.FirstTolerance())
                                {
                                    pos2 = Position2D.Head;
                                }
                                else if (p2.DistanceTo(_domainOnCurve2.LastPoint()) <= _domainOnCurve2.LastTolerance())
                                {
                                    pos2 = Position2D.End;
                                }
                                //-----------------------------------------------------------------
                                if (global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(pos1, ref tan1, ref trans1, pos2, ref tan2,
                                                                     ref trans2, tolConf)
                                    == false)
                                {
                                    TheCurveTool.D2(c1, u, out p1, out tan1, out norm1);
                                    TheCurveTool.D2(c1, v, out p2, out tan2, out norm2);
                                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(pos1, ref tan1, norm1, ref trans1, pos2, ref tan2,
                                                                     norm2, ref trans2, tolConf);
                                }
                                IntersectionPoint2D ip = new(p1, u, v, trans1, trans2, false);
                                Insert(ip);
                            }
                        }
                    }
                }
            }
        }

        //----------------------------------------------------------------------
        //-- Processing of TangentZone
        //----------------------------------------------------------------------
        int nbtz = interPP.TangentZoneCount;
        for (int tz = 1; tz <= nbtz; tz++)
        {
            int nbPnts = interPP.ZoneValue(tz).NumberOfPoints();
            //====================================================================
            //== Find the first and the last point in the tangency zone.
            //====================================================================
            double paramSupOnCurve2, paramInfOnCurve2;
            double paramSupOnCurve1, paramInfOnCurve1;
            int segIndex1onP1, segIndex1onP2;
            PIType type;
            double paramOnLine;
            double polyUInf, polyUSup, polyVInf, polyVSup;
            paramSupOnCurve2 = paramSupOnCurve1 = polyUSup = polyVSup = -RealLast();
            paramInfOnCurve2 = paramInfOnCurve1 = polyUInf = polyVInf = RealLast();
            for (int qq = 1; qq <= nbPnts; qq++)
            {
                SectionPoint sPnt1 = interPP.ZoneValue(tz).GetPoint(qq);
                //====================================================================
                //== The zones of tangency are discretized
                //== Test of stop : Check if
                //==     (Deflection  < Tolerance)
                //==  Or (Sample < EpsX)   (normally the first condition is
                //==                           more strict)
                //====================================================================
                double _PolyUInf, _PolyVInf;

                sPnt1.InfoFirst(out type, out segIndex1onP1, out paramOnLine);
                if (segIndex1onP1 > poly1.SegmentCount)
                {
                    segIndex1onP1--;
                    paramOnLine = 1.0;
                }
                if (segIndex1onP1 <= 0)
                {
                    segIndex1onP1 = 1;
                    paramOnLine = 0.0;
                }
                _PolyUInf = poly1.ApproxParamOnCurve(segIndex1onP1, paramOnLine);

                sPnt1.InfoSecond(out type, out segIndex1onP2, out paramOnLine);
                if (segIndex1onP2 > poly1.SegmentCount)
                {
                    segIndex1onP2--;
                    paramOnLine = 1.0;
                }
                if (segIndex1onP2 <= 0)
                {
                    segIndex1onP2 = 1;
                    paramOnLine = 0.0;
                }
                _PolyVInf = poly1.ApproxParamOnCurve(segIndex1onP2, paramOnLine);

                //----------------------------------------------------------------------

                if (paramInfOnCurve1 > _PolyUInf)
                {
                    paramInfOnCurve1 = _PolyUInf;
                }
                if (paramInfOnCurve2 > _PolyVInf)
                {
                    paramInfOnCurve2 = _PolyVInf;
                }

                if (paramSupOnCurve1 < _PolyUInf)
                {
                    paramSupOnCurve1 = _PolyUInf;
                }
                if (paramSupOnCurve2 < _PolyVInf)
                {
                    paramSupOnCurve2 = _PolyVInf;
                }
            }

            polyUInf = paramInfOnCurve1;
            polyUSup = paramSupOnCurve1;
            polyVInf = paramInfOnCurve2;
            polyVSup = paramSupOnCurve2;

            TheCurveTool.D0(c1, polyUInf, out p1);
            TheCurveTool.D0(c1, polyVInf, out p2);
            double distmemesens = p1.SquareDistanceTo(p2);
            TheCurveTool.D0(c1, polyVSup, out p2);
            double distdiffsens = p1.SquareDistanceTo(p2);
            if (distmemesens > distdiffsens)
            {
                double qwerty = polyVInf;
                polyVInf = polyVSup;
                polyVSup = qwerty;
            }

            //-----------------------------------------------------------------
            //-- Calculate Positions of Points on the curve and
            //-- Transitions on each limit of the segment

            Position2D pos1 = Position2D.Middle;
            Position2D pos2 = Position2D.Middle;
            Transition trans1 = new(), trans2 = new();

            TheCurveTool.D1(c1, polyUInf, out p1, out tan1);
            TheCurveTool.D1(c1, polyVInf, out p2, out tan2);

            if (p1.DistanceTo(_domainOnCurve1.FirstPoint()) <= _domainOnCurve1.FirstTolerance())
            {
                pos1 = Position2D.Head;
            }
            else if (p1.DistanceTo(_domainOnCurve1.LastPoint()) <= _domainOnCurve1.LastTolerance())
            {
                pos1 = Position2D.End;
            }
            if (p2.DistanceTo(_domainOnCurve2.FirstPoint()) <= _domainOnCurve2.FirstTolerance())
            {
                pos2 = Position2D.Head;
            }
            else if (p2.DistanceTo(_domainOnCurve2.LastPoint()) <= _domainOnCurve2.LastTolerance())
            {
                pos2 = Position2D.End;
            }

            if (pos1 == Position2D.Middle && pos2 != Position2D.Middle)
            {
                polyUInf = TheProjPCur.FindParameter(c1, p2, d1.FirstParameter(), d1.LastParameter(),
                                                     TheCurveTool.EpsX(c1));
            }
            else if (pos1 != Position2D.Middle && pos2 == Position2D.Middle)
            {
                polyVInf = TheProjPCur.FindParameter(c1, p1, d1.FirstParameter(), d1.LastParameter(),
                                                     TheCurveTool.EpsX(c1));
            }
            else if (Math.Abs(paramInfOnCurve1 - paramSupOnCurve1) > Math.Abs(paramInfOnCurve2 - paramSupOnCurve2))
            {
                polyVInf = TheProjPCur.FindParameter(c1, p1, d1.FirstParameter(), d1.LastParameter(),
                                                     TheCurveTool.EpsX(c1));
            }
            else
            {
                polyUInf = TheProjPCur.FindParameter(c1, p2, d1.FirstParameter(), d1.LastParameter(),
                                                     TheCurveTool.EpsX(c1));
            }

            if (global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(pos1, ref tan1, ref trans1, pos2, ref tan2, ref trans2, tolConf)
                == false)
            {
                TheCurveTool.D2(c1, polyUInf, out p1, out tan1, out norm1);
                TheCurveTool.D2(c1, polyVInf, out p2, out tan2, out norm2);
                global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(pos1, ref tan1, norm1, ref trans1, pos2, ref tan2, norm2, ref trans2,
                                                 tolConf);
            }
            IntersectionPoint2D ptSeg1 = new(p1, polyUInf, polyVInf, trans1, trans2, false);
            //----------------------------------------------------------------------

            if ((Math.Abs(polyUInf - polyUSup) <= TheCurveTool.EpsX(c1))
                || (Math.Abs(polyVInf - polyVSup) <= TheCurveTool.EpsX(c1)))
            {
                // bad segment
            }
            else
            {
                TheCurveTool.D1(c1, polyUSup, out p1, out tan1);
                TheCurveTool.D1(c1, polyVSup, out p2, out tan2);
                pos1 = Position2D.Middle;
                pos2 = Position2D.Middle;

                if (p1.DistanceTo(_domainOnCurve1.FirstPoint()) <= _domainOnCurve1.FirstTolerance())
                {
                    pos1 = Position2D.Head;
                }
                else if (p1.DistanceTo(_domainOnCurve1.LastPoint()) <= _domainOnCurve1.LastTolerance())
                {
                    pos1 = Position2D.End;
                }
                if (p2.DistanceTo(_domainOnCurve2.FirstPoint()) <= _domainOnCurve2.FirstTolerance())
                {
                    pos2 = Position2D.Head;
                }
                else if (p2.DistanceTo(_domainOnCurve2.LastPoint()) <= _domainOnCurve2.LastTolerance())
                {
                    pos2 = Position2D.End;
                }

                if (pos1 == Position2D.Middle && pos2 != Position2D.Middle)
                {
                    polyUSup = TheProjPCur.FindParameter(c1, p2, d1.FirstParameter(), d1.LastParameter(),
                                                         TheCurveTool.EpsX(c1));
                }
                else if (pos1 != Position2D.Middle && pos2 == Position2D.Middle)
                {
                    polyVSup = TheProjPCur.FindParameter(c1, p1, d1.FirstParameter(), d1.LastParameter(),
                                                         TheCurveTool.EpsX(c1));
                }
                else if (Math.Abs(paramInfOnCurve1 - paramSupOnCurve1)
                         > Math.Abs(paramInfOnCurve2 - paramSupOnCurve2))
                {
                    polyVSup = TheProjPCur.FindParameter(c1, p1, d1.FirstParameter(), d1.LastParameter(),
                                                         TheCurveTool.EpsX(c1));
                }
                else
                {
                    polyUSup = TheProjPCur.FindParameter(c1, p2, d1.FirstParameter(), d1.LastParameter(),
                                                         TheCurveTool.EpsX(c1));
                }

                if (global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(pos1, ref tan1, ref trans1, pos2, ref tan2, ref trans2, tolConf)
                    == false)
                {
                    TheCurveTool.D2(c1, polyUSup, out p1, out tan1, out norm1);
                    TheCurveTool.D2(c1, polyVSup, out p2, out tan2, out norm2);
                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(pos1, ref tan1, norm1, ref trans1, pos2, ref tan2, norm2,
                                                     ref trans2, tolConf);
                }
                IntersectionPoint2D ptSeg2 = new(p1, polyUSup, polyVSup, trans1, trans2, false);

                bool oppos = (tan1.Dot(tan2) > 0.0) ? false : true;
                if (paramInfOnCurve1 > paramSupOnCurve1)
                {
                    IntersectionSegment2D seg = new(ptSeg2, ptSeg1, oppos, false);
                    Append(seg);
                }
                else
                {
                    IntersectionSegment2D seg = new(ptSeg1, ptSeg2, oppos, false);
                    Append(seg);
                }
            }
        } // end of processing of TangentZone

        _done = true;
    }

    private static bool HeadOrEndPoint(in TwoDDomain d1,
                                       in TheCurve c1,
                                       double tu,
                                       in TwoDDomain d2,
                                       in TheCurve c2,
                                       double tv,
                                       double tolConf,
                                       ref IntersectionPoint2D intPt,
                                       ref bool headOn1,
                                       ref bool headOn2,
                                       ref bool endOn1,
                                       ref bool endOn2,
                                       int posSegment)
    {
        Point2D p1, p2, sp1 = new(), sp2 = new();
        Vector2D t1, t2, n1, n2;
        double u = tu;
        double v = tv;
        double svu = u;
        double svv = v;

        TheCurveTool.D1(c1, u, out p1, out t1);
        TheCurveTool.D1(c2, v, out p2, out t2);

        Position2D pos1 = Position2D.Middle;
        Position2D pos2 = Position2D.Middle;
        Transition trans1 = new(), trans2 = new();

        //----------------------------------------------------------------------
        //-- Head On 1   :        Head1 <-> P2
        if (p2.DistanceTo(d1.FirstPoint()) <= d1.FirstTolerance())
        {
            pos1 = Position2D.Head;
            headOn1 = true;
            sp1 = d1.FirstPoint();
            u = d1.FirstParameter();
        }
        //----------------------------------------------------------------------
        //-- End On 1   :         End1 <-> P2
        else if (p2.DistanceTo(d1.LastPoint()) <= d1.LastTolerance())
        {
            pos1 = Position2D.End;
            endOn1 = true;
            sp1 = d1.LastPoint();
            u = d1.LastParameter();
        }

        //----------------------------------------------------------------------
        //-- Head On 2   :        Head2 <-> P1
        else if (p1.DistanceTo(d2.FirstPoint()) <= d2.FirstTolerance())
        {
            pos2 = Position2D.Head;
            headOn2 = true;
            sp2 = d2.FirstPoint();
            v = d2.FirstParameter();
        }
        //----------------------------------------------------------------------
        //-- End On 2   :        End2 <-> P1
        else if (p1.DistanceTo(d2.LastPoint()) <= d2.LastTolerance())
        {
            pos2 = Position2D.End;
            endOn2 = true;
            sp2 = d2.LastPoint();
            v = d2.LastParameter();
        }

        double epsX1 = TheCurveTool.EpsX(c1);
        double epsX2 = TheCurveTool.EpsX(c2);

        if ((pos1 != Position2D.Middle) || (pos2 != Position2D.Middle))
        {
            if (pos1 == Position2D.Middle)
            {
                if (Math.Abs(u - d1.FirstParameter()) <= epsX1)
                {
                    pos1 = Position2D.Head;
                    p1 = d1.FirstPoint();
                    headOn1 = true;
                }
                else if (Math.Abs(u - d1.LastParameter()) <= epsX1)
                {
                    pos1 = Position2D.End;
                    p1 = d1.LastPoint();
                    endOn1 = true;
                }
            }
            else if (u != tu)
            {
                p1 = sp1;
            }

            if (pos2 == Position2D.Middle)
            {
                if (Math.Abs(v - d2.FirstParameter()) <= epsX2)
                {
                    pos2 = Position2D.Head;
                    headOn2 = true;
                    p2 = d2.FirstPoint();
                    if (pos1 != Position2D.Middle)
                    {
                        p1 = p1.WithCoord(0.5 * (p1.X + p2.X), 0.5 * (p1.Y + p2.Y));
                    }
                    else
                    {
                        p2 = p1;
                    }
                }
                else if (Math.Abs(v - d2.LastParameter()) <= epsX2)
                {
                    pos2 = Position2D.End;
                    endOn2 = true;
                    p2 = d2.LastPoint();
                    if (pos1 != Position2D.Middle)
                    {
                        p1 = p1.WithCoord(0.5 * (p1.X + p2.X), 0.5 * (p1.Y + p2.Y));
                    }
                    else
                    {
                        p2 = p1;
                    }
                }
            }
            // OCCT: SP2 is assigned and never read (P2 is not moved to the end of the second domain, as P1 is to SP1).
            _ = sp2;

            //--------------------------------------------------------------------
            //-- It is tested if a point at the end of segment already has its transitions
            //-- If Yes, the new point is not created
            //--
            //-- PosSegment =            1    if Head Head
            //--                       2      if Head End
            //--                     4        if End  Head
            //--                   8          if End  End
            //--------------------------------------------------------------------
            if (pos1 == Position2D.Head)
            {
                if ((pos2 == Position2D.Head) && (posSegment & 1) != 0)
                {
                    return (false);
                }
                if ((pos2 == Position2D.End) && (posSegment & 2) != 0)
                {
                    return (false);
                }
            }
            else if (pos1 == Position2D.End)
            {
                if ((pos2 == Position2D.Head) && (posSegment & 4) != 0)
                {
                    return (false);
                }
                if ((pos2 == Position2D.End) && (posSegment & 8) != 0)
                {
                    return (false);
                }
            }

            if (global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(pos1, ref t1, ref trans1, pos2, ref t2, ref trans2, tolConf) == false)
            {
                TheCurveTool.D2(c1, svu, out p1, out t1, out n1);
                TheCurveTool.D2(c2, svv, out p2, out t2, out n2);
                global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(pos1, ref t1, n1, ref trans1, pos2, ref t2, n2, ref trans2, tolConf);
            }
            intPt.SetValues(p1, u, v, trans1, trans2, false);
            return (true);
        }
        else
        {
            return (false);
        }
    }

    //=======================================================================
    // function : Perform
    // purpose  : Base method to perform polyline / polyline intersection for
    //           pair of curves.
    //=======================================================================
    /// <summary>The intersection of two curves: an iteration of the polygon interference.</summary>
    protected void Perform(in TheCurve c1, in TwoDDomain d1, in TheCurve c2, in TwoDDomain d2,
                           double tolConf, double tol, int nbIter, double deltaU, double deltaV)
    {
        int nbsamplesOnC1, nbsamplesOnC2;
        _done = false;

        if (nbIter > NBITER_MAX_POLYGON)
        {
            return;
        }

        // Number of samples running.
        nbsamplesOnC1 = TheCurveTool.NbSamples(c1, d1.FirstParameter(), d1.LastParameter());
        nbsamplesOnC2 = TheCurveTool.NbSamples(c2, d2.FirstParameter(), d2.LastParameter());

        if (nbIter == 0)
        {
            // Minimal number of points.
            nbsamplesOnC1 = Max(nbsamplesOnC1, _minPntNb);
            nbsamplesOnC2 = Max(nbsamplesOnC2, _minPntNb);
        }
        else
        {
            // Increase number of samples in second and next iterations.
            nbsamplesOnC1 = (5 * (nbsamplesOnC1 * nbIter)) / 4;
            nbsamplesOnC2 = (5 * (nbsamplesOnC2 * nbIter)) / 4;
        }

        Polygon2dGen<TheCurve, TheCurveTool> poly1 = new(c1, nbsamplesOnC1, d1, tol),
                                                       poly2 = new(c2, nbsamplesOnC2, d2, tol);

        if ((poly1.DeflectionOverEstimation() > tolConf) && (poly2.DeflectionOverEstimation() > tolConf))
        {
            double deflectionSum = Max(poly1.DeflectionOverEstimation(), tolConf)
                                    + Max(poly2.DeflectionOverEstimation(), tolConf);

            if (nbsamplesOnC2 > nbsamplesOnC1)
            {
                poly2.ComputeWithBox(c2, poly1.Bounding());
                poly1.SetDeflectionOverEstimation(deflectionSum);
                poly1.ComputeWithBox(c1, poly2.Bounding());
            }
            else
            {
                poly1.ComputeWithBox(c1, poly2.Bounding());
                poly2.SetDeflectionOverEstimation(deflectionSum);
                poly2.ComputeWithBox(c2, poly1.Bounding());
            }
        }

        //----------------------------------------------------------------------
        //-- if the deflection less then the Tolerance of Confusion
        //-- Then the deflection of the polygon is set in TolConf
        //-- (Detection of Tangency Zones)
        //----------------------------------------------------------------------

        if (poly1.DeflectionOverEstimation() < tolConf)
        {
            poly1.SetDeflectionOverEstimation(tolConf);
        }
        if (poly2.DeflectionOverEstimation() < tolConf)
        {
            poly2.SetDeflectionOverEstimation(tolConf);
        }
        // for case when a few polygon points were replaced by line
        // if exact solution was not found
        // then search of precise solution will be repeated
        // for polygon contains all initial points
        // secondary search will be performed only for case when initial points
        // were dropped
        bool isFullRepresentation =
            (poly1.SegmentCount == nbsamplesOnC1 && poly2.SegmentCount == nbsamplesOnC2);

        if (!FindIntersect(c1, d1, c2, d2, tolConf, tol, nbIter, deltaU, deltaV, poly1, poly2,
                           isFullRepresentation)
            && !isFullRepresentation)
        {
            if (poly1.SegmentCount < nbsamplesOnC1)
            {
                poly1 = new Polygon2dGen<TheCurve, TheCurveTool>(c1, nbsamplesOnC1, d1, tol);
            }
            if (poly2.SegmentCount < nbsamplesOnC2)
            {
                poly2 = new Polygon2dGen<TheCurve, TheCurveTool>(c2, nbsamplesOnC2, d2, tol);
            }

            FindIntersect(c1, d1, c2, d2, tolConf, tol, nbIter, deltaU, deltaV, poly1, poly2, true);
        }

        _done = true;
    }

    //======================================================================
    // Purpose : findIntersect
    //======================================================================
    private bool FindIntersect(in TheCurve c1,
                               in TwoDDomain d1,
                               in TheCurve c2,
                               in TwoDDomain d2,
                               double tolConf,
                               double tol,
                               int nbIter,
                               double deltaU,
                               double deltaV,
                               Polygon2dGen<TheCurve, TheCurveTool> poly1,
                               Polygon2dGen<TheCurve, TheCurveTool> poly2,
                               bool isFullPolygon)
    {
        Vector2D tan1, tan2, norm1, norm2;
        Point2D p1, p2;
        InterferencePolygon2d interPP = new(poly1, poly2);
        ExactIntersectionPoint<TheCurve, TheCurveTool> eip = new(c1, c2, tolConf);
        double u = 0.0, v = 0.0;
        bool anErrorOccurred = false;
        _done = true; // To prevent exception in nbp=NbPoints();
        //----------------------------------------------------------------------
        //-- Processing of SectionPoint
        //----------------------------------------------------------------------
        int nbsp = interPP.SectionPointCount;
        for (int sp = 1; sp <= nbsp; sp++)
        {
            SectionPoint sPnt = interPP.PntValue(sp);
            int segIndex1, segIndex2;
            double paramOn1, paramOn2;
            PIType type;

            sPnt.InfoFirst(out type, out segIndex1, out paramOn1);
            sPnt.InfoSecond(out type, out segIndex2, out paramOn2);
            eip.Perform(poly1, poly2, ref segIndex1, ref segIndex2, ref paramOn1, ref paramOn2);
            anErrorOccurred = eip.AnErrorOccurred();

            if (eip.RootCount == 0 && !isFullPolygon)
            {
                return false;
            }

            if (anErrorOccurred)
            {
                continue;
            }

            //--------------------------------------------------------------------
            //-- It is checked if the found point is really a root
            //--------------------------------------------------------------------

            eip.Roots(out u, out v);
            TheCurveTool.D1(c1, u, out p1, out tan1);
            TheCurveTool.D1(c2, v, out p2, out tan2);
            double dist = p1.DistanceTo(p2);
            if (eip.RootCount == 0 && dist > tolConf)
            {
                Transition trans = new();
                IntersectionPoint2D pInt = new(p1, u, v, trans, trans, false);
                double t1f, t1l, t2f, t2l;
                t1f = poly1.ApproxParamOnCurve(segIndex1, 0.0);
                t1l = poly1.ApproxParamOnCurve(segIndex1, 1.0);
                t2f = poly2.ApproxParamOnCurve(segIndex2, 0.0);
                t2l = poly2.ApproxParamOnCurve(segIndex2, 1.0);
                //
                int maxCount = 16, count = 0;
                GetIntersection(c1, t1f, t1l, c2, t2f, t2l, tolConf, maxCount, ref pInt, ref dist, ref count);
                u = pInt.ParamOnFirst();
                v = pInt.ParamOnSecond();
                TheCurveTool.D1(c1, u, out p1, out tan1);
                TheCurveTool.D1(c2, v, out p2, out tan2);
                dist = p1.DistanceTo(p2);
            }
            //-----------------------------------------------------------------
            //-- It is checked if the point (u,v) does not exist already
            //--
            int nbp = PointCount;
            double epsX1 = 10.0 * TheCurveTool.EpsX(c1);
            double epsX2 = 10.0 * TheCurveTool.EpsX(c2);
            for (int p = 1; p <= nbp; p++)
            {
                IntersectionPoint2D P = Point(p);
                if (Math.Abs(u - P.ParamOnFirst()) <= epsX1)
                {
                    if (Math.Abs(v - P.ParamOnSecond()) <= epsX2)
                    {
                        dist = tolConf + 1.0;
                        p += nbp;
                    }
                }
            }

            if (dist <= tolConf)
            { //-- Or the point is already present
                Position2D pos1 = Position2D.Middle;
                Position2D pos2 = Position2D.Middle;
                Transition trans1 = new(), trans2 = new();
                //-----------------------------------------------------------------
                //-- Calculate the Positions of Points on the curve
                //--
                if (p1.DistanceTo(_domainOnCurve1.FirstPoint()) <= _domainOnCurve1.FirstTolerance())
                {
                    pos1 = Position2D.Head;
                }
                else if (p1.DistanceTo(_domainOnCurve1.LastPoint()) <= _domainOnCurve1.LastTolerance())
                {
                    pos1 = Position2D.End;
                }

                if (p2.DistanceTo(_domainOnCurve2.FirstPoint()) <= _domainOnCurve2.FirstTolerance())
                {
                    pos2 = Position2D.Head;
                }
                else if (p2.DistanceTo(_domainOnCurve2.LastPoint()) <= _domainOnCurve2.LastTolerance())
                {
                    pos2 = Position2D.End;
                }
                //-----------------------------------------------------------------
                //-- Calculate the Transitions (see IntImpParGen.cxx)
                //--
                if (global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(pos1, ref tan1, ref trans1, pos2, ref tan2, ref trans2, tolConf)
                    == false)
                {
                    TheCurveTool.D2(c1, u, out p1, out tan1, out norm1);
                    TheCurveTool.D2(c2, v, out p2, out tan2, out norm2);
                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(pos1, ref tan1, norm1, ref trans1, pos2, ref tan2, norm2,
                                                     ref trans2, tolConf);
                }
                IntersectionPoint2D ip = new(p1, u, v, trans1, trans2, false);
                Insert(ip);
            }
        }

        //----------------------------------------------------------------------
        //-- Processing of TangentZone
        //----------------------------------------------------------------------
        int nbtz = interPP.TangentZoneCount;
        for (int tz = 1; tz <= nbtz; tz++)
        {
            int nbPnts = interPP.ZoneValue(tz).NumberOfPoints();
            //====================================================================
            //== Find the first and the last point in the tangency zone.
            //====================================================================
            double paramSupOnCurve2, paramInfOnCurve2;
            double paramSupOnCurve1, paramInfOnCurve1;
            int segIndex1onP1, segIndex1onP2;
            PIType type;
            double paramOnLine;
            double polyUInf, polyUSup, polyVInf, polyVSup;
            paramSupOnCurve2 = paramSupOnCurve1 = polyUSup = polyVSup = -RealLast();
            paramInfOnCurve2 = paramInfOnCurve1 = polyUInf = polyVInf = RealLast();
            for (int qq = 1; qq <= nbPnts; qq++)
            {
                SectionPoint sPnt1 = interPP.ZoneValue(tz).GetPoint(qq);
                //====================================================================
                //== The zones of tangency are discretized
                //== Test of stop : Check if
                //==     (Deflection  < Tolerance)
                //==  Or (Sample < EpsX)   (normally the first condition is
                //==                           more strict)
                //====================================================================
                double _PolyUInf, _PolyVInf;

                sPnt1.InfoFirst(out type, out segIndex1onP1, out paramOnLine);
                if (segIndex1onP1 > poly1.SegmentCount)
                {
                    segIndex1onP1--;
                    paramOnLine = 1.0;
                }
                if (segIndex1onP1 <= 0)
                {
                    segIndex1onP1 = 1;
                    paramOnLine = 0.0;
                }
                _PolyUInf = poly1.ApproxParamOnCurve(segIndex1onP1, paramOnLine);

                sPnt1.InfoSecond(out type, out segIndex1onP2, out paramOnLine);
                if (segIndex1onP2 > poly2.SegmentCount)
                {
                    segIndex1onP2--;
                    paramOnLine = 1.0;
                }
                if (segIndex1onP2 <= 0)
                {
                    segIndex1onP2 = 1;
                    paramOnLine = 0.0;
                }
                _PolyVInf = poly2.ApproxParamOnCurve(segIndex1onP2, paramOnLine);

                //----------------------------------------------------------------------

                if (paramInfOnCurve1 > _PolyUInf)
                {
                    paramInfOnCurve1 = _PolyUInf;
                }
                if (paramInfOnCurve2 > _PolyVInf)
                {
                    paramInfOnCurve2 = _PolyVInf;
                }

                if (paramSupOnCurve1 < _PolyUInf)
                {
                    paramSupOnCurve1 = _PolyUInf;
                }
                if (paramSupOnCurve2 < _PolyVInf)
                {
                    paramSupOnCurve2 = _PolyVInf;
                }
            }

            polyUInf = paramInfOnCurve1;
            polyUSup = paramSupOnCurve1;
            polyVInf = paramInfOnCurve2;
            polyVSup = paramSupOnCurve2;

            TheCurveTool.D0(c1, polyUInf, out p1);
            TheCurveTool.D0(c2, polyVInf, out p2);
            double distmemesens = p1.SquareDistanceTo(p2);
            TheCurveTool.D0(c2, polyVSup, out p2);
            double distdiffsens = p1.SquareDistanceTo(p2);
            if (distmemesens > distdiffsens)
            {
                double qwerty = polyVInf;
                polyVInf = polyVSup;
                polyVSup = qwerty;
            }

            if (((poly1.DeflectionOverEstimation() > tolConf) || (poly2.DeflectionOverEstimation() > tolConf))
                && (nbIter < NBITER_MAX_POLYGON))
            {
                TwoDDomain recursD1 = new(TheCurveTool.Value(c1, paramInfOnCurve1),
                                               paramInfOnCurve1,
                                               tolConf,
                                               TheCurveTool.Value(c1, paramSupOnCurve1),
                                               paramSupOnCurve1,
                                               tolConf);
                TwoDDomain recursD2 = new(TheCurveTool.Value(c2, paramInfOnCurve2),
                                               paramInfOnCurve2,
                                               tolConf,
                                               TheCurveTool.Value(c2, paramSupOnCurve2),
                                               paramSupOnCurve2,
                                               tolConf);
                //-- thePoly1(2) are not deleted,
                //-- finally they are destroyed.
                //-- !! No untimely return !!
                // OCCT passes Tol as TolConf and TolConf as Tol to the recursion.
                Perform(c1, recursD1, c2, recursD2, tol, tolConf, nbIter + 1, deltaU, deltaV);
            }
            else
            {
                //-----------------------------------------------------------------
                //-- Calculate Positions of Points on the curve and
                //-- Transitions on each limit of the segment

                Position2D pos1 = Position2D.Middle;
                Position2D pos2 = Position2D.Middle;
                Transition trans1 = new(), trans2 = new();

                TheCurveTool.D1(c1, polyUInf, out p1, out tan1);
                TheCurveTool.D1(c2, polyVInf, out p2, out tan2);

                if (p1.DistanceTo(_domainOnCurve1.FirstPoint()) <= _domainOnCurve1.FirstTolerance())
                {
                    pos1 = Position2D.Head;
                }
                else if (p1.DistanceTo(_domainOnCurve1.LastPoint()) <= _domainOnCurve1.LastTolerance())
                {
                    pos1 = Position2D.End;
                }
                if (p2.DistanceTo(_domainOnCurve2.FirstPoint()) <= _domainOnCurve2.FirstTolerance())
                {
                    pos2 = Position2D.Head;
                }
                else if (p2.DistanceTo(_domainOnCurve2.LastPoint()) <= _domainOnCurve2.LastTolerance())
                {
                    pos2 = Position2D.End;
                }

                if (pos1 == Position2D.Middle && pos2 != Position2D.Middle)
                {
                    polyUInf = TheProjPCur.FindParameter(c1, p2, d1.FirstParameter(), d1.LastParameter(),
                                                         TheCurveTool.EpsX(c1));
                }
                else if (pos1 != Position2D.Middle && pos2 == Position2D.Middle)
                {
                    polyVInf = TheProjPCur.FindParameter(c2, p1, d2.FirstParameter(), d2.LastParameter(),
                                                         TheCurveTool.EpsX(c2));
                }
                else if (Math.Abs(paramInfOnCurve1 - paramSupOnCurve1)
                         > Math.Abs(paramInfOnCurve2 - paramSupOnCurve2))
                {
                    polyVInf = TheProjPCur.FindParameter(c2, p1, d2.FirstParameter(), d2.LastParameter(),
                                                         TheCurveTool.EpsX(c2));
                }
                else
                {
                    polyUInf = TheProjPCur.FindParameter(c1, p2, d1.FirstParameter(), d1.LastParameter(),
                                                         TheCurveTool.EpsX(c1));
                }

                if (global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(pos1, ref tan1, ref trans1, pos2, ref tan2, ref trans2, tolConf)
                    == false)
                {
                    TheCurveTool.D2(c1, polyUInf, out p1, out tan1, out norm1);
                    TheCurveTool.D2(c2, polyVInf, out p2, out tan2, out norm2);
                    global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(pos1, ref tan1, norm1, ref trans1, pos2, ref tan2, norm2,
                                                     ref trans2, tolConf);
                }
                IntersectionPoint2D ptSeg1 = new(p1, polyUInf, polyVInf, trans1, trans2, false);
                //----------------------------------------------------------------------

                if ((Math.Abs(polyUInf - polyUSup) <= TheCurveTool.EpsX(c1))
                    || (Math.Abs(polyVInf - polyVSup) <= TheCurveTool.EpsX(c2)))
                {
                    Insert(ptSeg1);
                }
                else
                {
                    TheCurveTool.D1(c1, polyUSup, out p1, out tan1);
                    TheCurveTool.D1(c2, polyVSup, out p2, out tan2);
                    pos1 = Position2D.Middle;
                    pos2 = Position2D.Middle;

                    if (p1.DistanceTo(_domainOnCurve1.FirstPoint()) <= _domainOnCurve1.FirstTolerance())
                    {
                        pos1 = Position2D.Head;
                    }
                    else if (p1.DistanceTo(_domainOnCurve1.LastPoint()) <= _domainOnCurve1.LastTolerance())
                    {
                        pos1 = Position2D.End;
                    }
                    if (p2.DistanceTo(_domainOnCurve2.FirstPoint()) <= _domainOnCurve2.FirstTolerance())
                    {
                        pos2 = Position2D.Head;
                    }
                    else if (p2.DistanceTo(_domainOnCurve2.LastPoint()) <= _domainOnCurve2.LastTolerance())
                    {
                        pos2 = Position2D.End;
                    }

                    if (pos1 == Position2D.Middle && pos2 != Position2D.Middle)
                    {
                        polyUSup = TheProjPCur.FindParameter(c1, p2, d1.FirstParameter(), d1.LastParameter(),
                                                             TheCurveTool.EpsX(c1));
                    }
                    else if (pos1 != Position2D.Middle && pos2 == Position2D.Middle)
                    {
                        polyVSup = TheProjPCur.FindParameter(c2, p1, d2.FirstParameter(), d2.LastParameter(),
                                                             TheCurveTool.EpsX(c2));
                    }
                    else if (Math.Abs(paramInfOnCurve1 - paramSupOnCurve1)
                             > Math.Abs(paramInfOnCurve2 - paramSupOnCurve2))
                    {
                        polyVSup = TheProjPCur.FindParameter(c2, p1, d2.FirstParameter(), d2.LastParameter(),
                                                             TheCurveTool.EpsX(c2));
                    }
                    else
                    {
                        polyUSup = TheProjPCur.FindParameter(c1, p2, d1.FirstParameter(), d1.LastParameter(),
                                                             TheCurveTool.EpsX(c1));
                    }

                    if (global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(pos1, ref tan1, ref trans1, pos2, ref tan2, ref trans2,
                                                         tolConf)
                        == false)
                    {
                        TheCurveTool.D2(c1, polyUSup, out p1, out tan1, out norm1);
                        TheCurveTool.D2(c2, polyVSup, out p2, out tan2, out norm2);
                        global::Kerf.Engine.Intersection.Implicit.IntImpParGen.DetermineTransition(pos1, ref tan1, norm1, ref trans1, pos2, ref tan2, norm2,
                                                         ref trans2, tolConf);
                    }
                    IntersectionPoint2D ptSeg2 = new(p1, polyUSup, polyVSup, trans1, trans2, false);

                    bool oppos = (tan1.Dot(tan2) > 0.0) ? false : true;
                    if (paramInfOnCurve1 > paramSupOnCurve1)
                    {
                        IntersectionSegment2D seg = new(ptSeg2, ptSeg1, oppos, false);
                        Append(seg);
                    }
                    else
                    {
                        IntersectionSegment2D seg = new(ptSeg1, ptSeg2, oppos, false);
                        Append(seg);
                    }
                }
            }
        }
        return true;
    }

    //======================================================================
    //  GetIntersection
    //======================================================================
    private static void GetIntersection(in TheCurve c1,
                                        double t1f,
                                        double t1l,
                                        in TheCurve c2,
                                        double t2f,
                                        double t2l,
                                        double tolConf,
                                        int maxCount,
                                        ref IntersectionPoint2D pInt,
                                        ref double dist,
                                        ref int count)
    {
        count++;
        //
        double tol2 = tolConf * tolConf;
        double pTol1 = Max(100.0 * Epsilon(Max(Math.Abs(t1f), Math.Abs(t1l))), global::Kerf.Engine.Foundation.Precision.PConfusion());
        double pTol2 = Max(100.0 * Epsilon(Max(Math.Abs(t2f), Math.Abs(t2l))), global::Kerf.Engine.Foundation.Precision.PConfusion());
        Point2D p1f, p1l, p2f, p2l;
        Box2d b1 = new(), b2 = new();
        //
        TheCurveTool.D0(c1, t1f, out p1f);
        TheCurveTool.D0(c1, t1l, out p1l);
        b1.Add(p1f);
        b1.Add(p1l);
        b1.Enlarge(tolConf);
        //
        TheCurveTool.D0(c2, t2f, out p2f);
        TheCurveTool.D0(c2, t2l, out p2l);
        b2.Add(p2f);
        b2.Add(p2l);
        b2.Enlarge(tolConf);
        //
        if (b1.IsOut(b2))
        {
            count--;
            return;
        }
        //
        bool isSmall1 = (t1l - t1f) <= pTol1 || p1f.SquareDistanceTo(p1l) / 4.0 <= tol2;
        bool isSmall2 = (t2l - t2f) <= pTol2 || p2f.SquareDistanceTo(p2l) / 4.0 <= tol2;

        if ((isSmall1 && isSmall2) || (count > maxCount))
        {
            // Seems to be intersection
            // Simple treatment of segment intersection
            Span<Xy> pnts1 = [p1f.XY(), (p1f.XY() + p1l.XY()) / 2.0, p1l.XY()];
            Span<Xy> pnts2 = [p2f.XY(), (p2f.XY() + p2l.XY()) / 2.0, p2l.XY()];
            int i, j, imin = -1, jmin = -1;
            double dmin = RealLast(), d;
            for (i = 0; i < 3; i++)
            {
                for (j = 0; j < 3; j++)
                {
                    d = (pnts1[i] - pnts2[j]).SquareModulus();
                    if (d < dmin)
                    {
                        dmin = d;
                        imin = i;
                        jmin = j;
                    }
                }
            }
            //
            dmin = Math.Sqrt(dmin);
            if (dist > dmin)
            {
                dist = dmin;
                //
                double t1;
                if (imin == 0)
                {
                    t1 = t1f;
                }
                else if (imin == 1)
                {
                    t1 = (t1f + t1l) / 2.0;
                }
                else
                {
                    t1 = t1l;
                }
                //
                double t2;
                if (jmin == 0)
                {
                    t2 = t2f;
                }
                else if (jmin == 1)
                {
                    t2 = (t2f + t2l) / 2.0;
                }
                else
                {
                    t2 = t2l;
                }
                //
                Point2D pint = new((pnts1[imin] + pnts2[jmin]) / 2.0);
                //
                Transition trans1 = new(), trans2 = new();
                pInt.SetValues(pint, t1, t2, trans1, trans2, false);
            }
            count--;
            return;
        }

        if (isSmall1)
        {
            double t2m = (t2l + t2f) / 2.0;
            GetIntersection(c1, t1f, t1l, c2, t2f, t2m, tolConf, maxCount, ref pInt,
                            ref dist, ref count);
            GetIntersection(c1, t1f, t1l, c2, t2m, t2l, tolConf, maxCount, ref pInt,
                            ref dist, ref count);
        }
        else if (isSmall2)
        {
            double t1m = (t1l + t1f) / 2.0;
            GetIntersection(c1, t1f, t1m, c2, t2f, t2l, tolConf, maxCount, ref pInt,
                            ref dist, ref count);
            GetIntersection(c1, t1m, t1l, c2, t2f, t2l, tolConf, maxCount, ref pInt,
                            ref dist, ref count);
        }
        else
        {
            double t1m = (t1l + t1f) / 2.0;
            double t2m = (t2l + t2f) / 2.0;
            GetIntersection(c1, t1f, t1m, c2, t2f, t2m, tolConf, maxCount, ref pInt,
                            ref dist, ref count);
            GetIntersection(c1, t1f, t1m, c2, t2m, t2l, tolConf, maxCount, ref pInt,
                            ref dist, ref count);
            GetIntersection(c1, t1m, t1l, c2, t2f, t2m, tolConf, maxCount, ref pInt,
                            ref dist, ref count);
            GetIntersection(c1, t1m, t1l, c2, t2m, t2l, tolConf, maxCount, ref pInt,
                            ref dist, ref count);
        }
    }

    /// <summary>The minimal number of samples of each curve's polygon.</summary>
    public int GetMinNbSamples() => _minPntNb;

    /// <summary>Sets the minimal number of samples of each curve's polygon (20 by default).</summary>
    public void SetMinNbSamples(int minNbSamples)
    {
        _minPntNb = minNbSamples;
    }
}
