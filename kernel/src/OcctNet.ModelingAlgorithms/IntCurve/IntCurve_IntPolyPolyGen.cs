// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntCurve/IntCurve_IntPolyPolyGen.gxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntRes2d_Position;

namespace Occt;

/// <summary>
/// The intersection of two bounded 2D curves, or the self-intersection of one, through the interference of their
/// polygons refined into exact points and tangent segments.
/// </summary>
/// <remarks>
/// OCCT's generic file IntCurve_IntPolyPolyGen.gxx, ported once as a generic class (decision 324); its
/// instantiation Geom2dInt_TheIntPCurvePCurveOfGInter is a sealed subclass. The file-static functions
/// <c>GetIntersection</c> and <c>HeadOrEndPoint</c> are private static methods.
/// </remarks>
public class IntCurve_IntPolyPolyGen<TheCurve, TheCurveTool, TheProjPCur> : IntRes2d_Intersection
    where TheCurveTool : IIntCurve_CurveTool<TheCurve>
    where TheProjPCur : IIntImpParGen_ProjectOnPCurveTool<TheCurve>
{
    private const int NBITER_MAX_POLYGON = 10;
    private const double TOL_CONF_MINI = 0.0000000001;
    private const double TOL_MINI = 0.0000000001;

    private IntRes2d_Domain DomainOnCurve1 = new();
    private IntRes2d_Domain DomainOnCurve2 = new();
    private int myMinPntNb;

    public IntCurve_IntPolyPolyGen()
    {
        const int aMinPntNb = 20; // Minimum number of samples.
        myMinPntNb = aMinPntNb;
        done = false;
    }

    /// <summary>The intersection of two curves on their (bounded) domains.</summary>
    public void Perform(in TheCurve C1, in IntRes2d_Domain D1, in TheCurve C2, in IntRes2d_Domain D2,
                        double TheTolConf, double TheTol)
    {
        this.ResetFields();
        DomainOnCurve1 = D1;
        DomainOnCurve2 = D2;
        double DU = D1.LastParameter() - D1.FirstParameter();
        double DV = D2.LastParameter() - D2.FirstParameter();
        double Tl = (TheTol < TOL_MINI) ? TOL_MINI : TheTol;
        double TlConf = (TheTolConf < TOL_CONF_MINI) ? TOL_CONF_MINI : TheTolConf;
        Perform(C1, D1, C2, D2, TlConf, Tl, 0, DU, DV);
        //----------------------------------------------------------------------
        //-- Processing of end points
        //----------------------------------------------------------------------
        bool HeadOn1 = false;
        bool HeadOn2 = false;
        bool EndOn1 = false;
        bool EndOn2 = false;
        int i;
        int n = this.NbPoints();

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
        int PosSegment = 0;

        for (i = 1; i <= n; i++)
        {
            IntRes2d_Position Pos1 = this.Point(i).TransitionOfFirst().PositionOnCurve();
            if (Pos1 == IntRes2d_Head)
            {
                HeadOn1 = true;
            }
            else if (Pos1 == IntRes2d_End)
            {
                EndOn1 = true;
            }

            IntRes2d_Position Pos2 = this.Point(i).TransitionOfSecond().PositionOnCurve();
            if (Pos2 == IntRes2d_Head)
            {
                HeadOn2 = true;
            }
            else if (Pos2 == IntRes2d_End)
            {
                EndOn2 = true;
            }

            PosSegment = AddPosSegment(PosSegment, Pos1, Pos2);
        }

        n = this.NbSegments();
        for (i = 1; i <= n; i++)
        {
            IntRes2d_Position Pos1 = this.Segment(i).FirstPoint().TransitionOfFirst().PositionOnCurve();
            if (Pos1 == IntRes2d_Head)
            {
                HeadOn1 = true;
            }
            else if (Pos1 == IntRes2d_End)
            {
                EndOn1 = true;
            }

            IntRes2d_Position Pos2 = this.Segment(i).FirstPoint().TransitionOfSecond().PositionOnCurve();
            if (Pos2 == IntRes2d_Head)
            {
                HeadOn2 = true;
            }
            else if (Pos2 == IntRes2d_End)
            {
                EndOn2 = true;
            }

            PosSegment = AddPosSegment(PosSegment, Pos1, Pos2);

            Pos1 = this.Segment(i).LastPoint().TransitionOfFirst().PositionOnCurve();
            if (Pos1 == IntRes2d_Head)
            {
                HeadOn1 = true;
            }
            else if (Pos1 == IntRes2d_End)
            {
                EndOn1 = true;
            }

            Pos2 = this.Segment(i).LastPoint().TransitionOfSecond().PositionOnCurve();
            if (Pos2 == IntRes2d_Head)
            {
                HeadOn2 = true;
            }
            else if (Pos2 == IntRes2d_End)
            {
                EndOn2 = true;
            }

            PosSegment = AddPosSegment(PosSegment, Pos1, Pos2);
        }

        double U0 = D1.FirstParameter();
        double U1 = D1.LastParameter();
        double V0 = D2.FirstParameter();
        double V1 = D2.LastParameter();
        IntRes2d_IntersectionPoint IntPt = new();

        if (D1.FirstTolerance() != 0 || D2.FirstTolerance() != 0)
        {
            if (HeadOrEndPoint(D1, C1, U0, D2, C2, V0, TheTolConf, ref IntPt, ref HeadOn1, ref HeadOn2, ref EndOn1,
                               ref EndOn2, PosSegment))
            {
                this.Insert(IntPt);
            }
        }
        if (D1.FirstTolerance() != 0 || D2.LastTolerance() != 0)
        {
            if (HeadOrEndPoint(D1, C1, U0, D2, C2, V1, TheTolConf, ref IntPt, ref HeadOn1, ref HeadOn2, ref EndOn1,
                               ref EndOn2, PosSegment))
            {
                this.Insert(IntPt);
            }
        }
        if (D1.LastTolerance() != 0 || D2.FirstTolerance() != 0)
        {
            if (HeadOrEndPoint(D1, C1, U1, D2, C2, V0, TheTolConf, ref IntPt, ref HeadOn1, ref HeadOn2, ref EndOn1,
                               ref EndOn2, PosSegment))
            {
                this.Insert(IntPt);
            }
        }
        if (D1.LastTolerance() != 0 || D2.LastTolerance() != 0)
        {
            if (HeadOrEndPoint(D1, C1, U1, D2, C2, V1, TheTolConf, ref IntPt, ref HeadOn1, ref HeadOn2, ref EndOn1,
                               ref EndOn2, PosSegment))
            {
                this.Insert(IntPt);
            }
        }
    }

    // The PosSegment flags of a point with the positions Pos1 and Pos2 (written out four times in OCCT).
    private static int AddPosSegment(int PosSegment, IntRes2d_Position Pos1, IntRes2d_Position Pos2)
    {
        if (Pos1 == IntRes2d_Head)
        {
            if (Pos2 == IntRes2d_Head)
            {
                PosSegment |= 1;
            }
            else if (Pos2 == IntRes2d_End)
            {
                PosSegment |= 2;
            }
        }
        else if (Pos1 == IntRes2d_End)
        {
            if (Pos2 == IntRes2d_Head)
            {
                PosSegment |= 4;
            }
            else if (Pos2 == IntRes2d_End)
            {
                PosSegment |= 8;
            }
        }
        return PosSegment;
    }

    //======================================================================
    //==      A u t o   I n t e r s e c t i o  n
    //======================================================================
    /// <summary>The self-intersection of a curve on its (bounded) domain.</summary>
    public void Perform(in TheCurve C1, in IntRes2d_Domain D1, double TheTolConf, double TheTol)
    {
        this.ResetFields();
        DomainOnCurve1 = D1;
        DomainOnCurve2 = D1;
        double DU = D1.LastParameter() - D1.FirstParameter();
        double Tl = (TheTol < TOL_MINI) ? TOL_MINI : TheTol;
        double TlConf = (TheTolConf < TOL_CONF_MINI) ? TOL_CONF_MINI : TheTolConf;
        Perform(C1, D1, TlConf, Tl, 0, DU, DU);
        // OCCT then computes the PosSegment flags of the points and segments found, and discards them.
    }

    /// <summary>The self-intersection of a curve: an iteration of the polygon interference.</summary>
    protected void Perform(in TheCurve C1, in IntRes2d_Domain D1, double TolConf, double Tol, int NbIter,
                           double DeltaU, double DeltaV)
    {
        gp_Vec2d Tan1, Tan2, Norm1, Norm2;
        gp_Pnt2d P1, P2;
        int nbsamples;
        done = false;

        nbsamples = TheCurveTool.NbSamples(C1, D1.FirstParameter(), D1.LastParameter());

        if (NbIter > 3 || (NbIter > 2 && nbsamples > 100))
        {
            return;
        }

        nbsamples *= 2; //---  We take systematically two times more points
                        //--   than on a normal curve.
                        //--   Auto-intersecting curves often produce
                        //--   polygons rather far from the curve with parameter ct.

        if (NbIter > 0)
        {
            nbsamples = (3 * (nbsamples * NbIter)) / 2;
        }
        IntCurve_Polygon2dGen<TheCurve, TheCurveTool> Poly1 = new(C1, nbsamples, D1, Tol);
        if (!Poly1.AutoIntersectionIsPossible())
        {
            done = true;
            return;
        }
        //----------------------------------------------------------------------
        //-- If the deflection is less than the Tolerance of Confusion
        //-- then the deflection of the polygon is set in TolConf
        //-- (Detection of Tangency Zones)
        //----------------------------------------------------------------------
        if (Poly1.DeflectionOverEstimation() < TolConf)
        {
            Poly1.SetDeflectionOverEstimation(TolConf);
        }

        Intf_InterferencePolygon2d InterPP = new(Poly1);
        IntCurve_ExactIntersectionPoint<TheCurve, TheCurveTool> EIP = new(C1, C1, TolConf);
        double U, V;

        //----------------------------------------------------------------------
        //-- Processing of SectionPoint
        //----------------------------------------------------------------------
        int Nbsp = InterPP.NbSectionPoints();
        if (Nbsp >= 1)
        {
            //-- ---------------------------------------------------------------------
            //-- filtering, filtering, filtering ...
            //--
            int[] TriIndex = new int[Nbsp + 1];
            int[] PtrSegIndex1 = new int[Nbsp + 1];
            int[] PtrSegIndex2 = new int[Nbsp + 1];
            bool Triok;
            int SegIndex1, SegIndex2, SegIndex_1, SegIndex_2;
            double ParamOn1, ParamOn2;
            Intf_PIType Type;
            int i;
            for (i = 1; i <= Nbsp; i++)
            {
                TriIndex[i] = i;
                Intf_SectionPoint SPnt1 = InterPP.PntValue(i);
                SPnt1.InfoFirst(out Type, out PtrSegIndex1[i], out ParamOn1);
                SPnt1.InfoSecond(out Type, out PtrSegIndex2[i], out ParamOn2);
            }

            do
            {
                Triok = true;

                for (int tr = 1; tr < Nbsp; tr++)
                {
                    SegIndex1 = PtrSegIndex1[TriIndex[tr]];
                    SegIndex_1 = PtrSegIndex1[TriIndex[tr + 1]];

                    SegIndex2 = PtrSegIndex2[TriIndex[tr]];
                    SegIndex_2 = PtrSegIndex2[TriIndex[tr + 1]];

                    if (SegIndex1 > SegIndex_1)
                    {
                        int q = TriIndex[tr];
                        TriIndex[tr] = TriIndex[tr + 1];
                        TriIndex[tr + 1] = q;
                        Triok = false;
                    }
                    else if (SegIndex1 == SegIndex_1)
                    {
                        if (SegIndex2 > SegIndex_2)
                        {
                            int q = TriIndex[tr];
                            TriIndex[tr] = TriIndex[tr + 1];
                            TriIndex[tr + 1] = q;
                            Triok = false;
                        }
                    }
                }
            } while (Triok == false);

            //-- supression des doublons Si Si !
            for (i = 1; i < Nbsp; i++)
            {
                if ((PtrSegIndex1[TriIndex[i]] == PtrSegIndex1[TriIndex[i + 1]])
                    && (PtrSegIndex2[TriIndex[i]] == PtrSegIndex2[TriIndex[i + 1]]))
                {
                    TriIndex[i] = -i;
                }
            }

            for (int sp = 1; sp <= Nbsp; sp++)
            {
                if (TriIndex[sp] > 0)
                {
                    Intf_SectionPoint SPnt = InterPP.PntValue(TriIndex[sp]);

                    SPnt.InfoFirst(out Type, out SegIndex1, out ParamOn1);
                    SPnt.InfoSecond(out Type, out SegIndex2, out ParamOn2);

                    if (Math.Abs(SegIndex1 - SegIndex2) > 1)
                    {
                        EIP.Perform(Poly1, Poly1, ref SegIndex1, ref SegIndex2, ref ParamOn1, ref ParamOn2);
                        if (EIP.NbRoots() >= 1)
                        {
                            //--------------------------------------------------------------------
                            //-- It is checked if the found point is a root
                            //--------------------------------------------------------------------
                            EIP.Roots(out U, out V);

                            TheCurveTool.D1(C1, U, out P1, out Tan1);
                            TheCurveTool.D1(C1, V, out P2, out Tan2);
                            double Dist = P1.Distance(P2);
                            double EpsX1 = 10.0 * TheCurveTool.EpsX(C1);

                            if (Math.Abs(U - V) <= EpsX1)
                            {
                                //-----------------------------------------
                                //-- Solution not valid
                                //-- The maths should have converged in a
                                //-- trivial solution  ( point U = V )
                                //-----------------------------------------
                                Dist = TolConf + 1.0;
                            }

                            //-----------------------------------------------------------------
                            //-- It is checked if the point (u,v) already exists
                            //--
                            done = true;
                            int nbp = NbPoints();

                            for (int p = 1; p <= nbp; p++)
                            {
                                IntRes2d_IntersectionPoint P = Point(p);
                                if (Math.Abs(U - P.ParamOnFirst()) <= EpsX1)
                                {
                                    if (Math.Abs(V - P.ParamOnSecond()) <= EpsX1)
                                    {
                                        Dist = TolConf + 1.0;
                                        p += nbp;
                                    }
                                }
                            }

                            if (Dist <= TolConf)
                            { //-- Or the point is already present
                                IntRes2d_Position Pos1 = IntRes2d_Middle;
                                IntRes2d_Position Pos2 = IntRes2d_Middle;
                                IntRes2d_Transition Trans1 = new(), Trans2 = new();
                                //-----------------------------------------------------------------
                                //-- Calculate Positions of Points on the curve
                                //--
                                if (P1.Distance(DomainOnCurve1.FirstPoint()) <= DomainOnCurve1.FirstTolerance())
                                {
                                    Pos1 = IntRes2d_Head;
                                }
                                else if (P1.Distance(DomainOnCurve1.LastPoint()) <= DomainOnCurve1.LastTolerance())
                                {
                                    Pos1 = IntRes2d_End;
                                }

                                if (P2.Distance(DomainOnCurve2.FirstPoint()) <= DomainOnCurve2.FirstTolerance())
                                {
                                    Pos2 = IntRes2d_Head;
                                }
                                else if (P2.Distance(DomainOnCurve2.LastPoint()) <= DomainOnCurve2.LastTolerance())
                                {
                                    Pos2 = IntRes2d_End;
                                }
                                //-----------------------------------------------------------------
                                if (IntImpParGen.DetermineTransition(Pos1, ref Tan1, ref Trans1, Pos2, ref Tan2,
                                                                     ref Trans2, TolConf)
                                    == false)
                                {
                                    TheCurveTool.D2(C1, U, out P1, out Tan1, out Norm1);
                                    TheCurveTool.D2(C1, V, out P2, out Tan2, out Norm2);
                                    IntImpParGen.DetermineTransition(Pos1, ref Tan1, Norm1, ref Trans1, Pos2, ref Tan2,
                                                                     Norm2, ref Trans2, TolConf);
                                }
                                IntRes2d_IntersectionPoint IP = new(P1, U, V, Trans1, Trans2, false);
                                Insert(IP);
                            }
                        }
                    }
                }
            }
        }

        //----------------------------------------------------------------------
        //-- Processing of TangentZone
        //----------------------------------------------------------------------
        int Nbtz = InterPP.NbTangentZones();
        for (int tz = 1; tz <= Nbtz; tz++)
        {
            int NbPnts = InterPP.ZoneValue(tz).NumberOfPoints();
            //====================================================================
            //== Find the first and the last point in the tangency zone.
            //====================================================================
            double ParamSupOnCurve2, ParamInfOnCurve2;
            double ParamSupOnCurve1, ParamInfOnCurve1;
            int SegIndex1onP1, SegIndex1onP2;
            Intf_PIType Type;
            double ParamOnLine;
            double PolyUInf, PolyUSup, PolyVInf, PolyVSup;
            ParamSupOnCurve2 = ParamSupOnCurve1 = PolyUSup = PolyVSup = -RealLast();
            ParamInfOnCurve2 = ParamInfOnCurve1 = PolyUInf = PolyVInf = RealLast();
            for (int qq = 1; qq <= NbPnts; qq++)
            {
                Intf_SectionPoint SPnt1 = InterPP.ZoneValue(tz).GetPoint(qq);
                //====================================================================
                //== The zones of tangency are discretized
                //== Test of stop : Check if
                //==     (Deflection  < Tolerance)
                //==  Or (Sample < EpsX)   (normally the first condition is
                //==                           more strict)
                //====================================================================
                double _PolyUInf, _PolyVInf;

                SPnt1.InfoFirst(out Type, out SegIndex1onP1, out ParamOnLine);
                if (SegIndex1onP1 > Poly1.NbSegments())
                {
                    SegIndex1onP1--;
                    ParamOnLine = 1.0;
                }
                if (SegIndex1onP1 <= 0)
                {
                    SegIndex1onP1 = 1;
                    ParamOnLine = 0.0;
                }
                _PolyUInf = Poly1.ApproxParamOnCurve(SegIndex1onP1, ParamOnLine);

                SPnt1.InfoSecond(out Type, out SegIndex1onP2, out ParamOnLine);
                if (SegIndex1onP2 > Poly1.NbSegments())
                {
                    SegIndex1onP2--;
                    ParamOnLine = 1.0;
                }
                if (SegIndex1onP2 <= 0)
                {
                    SegIndex1onP2 = 1;
                    ParamOnLine = 0.0;
                }
                _PolyVInf = Poly1.ApproxParamOnCurve(SegIndex1onP2, ParamOnLine);

                //----------------------------------------------------------------------

                if (ParamInfOnCurve1 > _PolyUInf)
                {
                    ParamInfOnCurve1 = _PolyUInf;
                }
                if (ParamInfOnCurve2 > _PolyVInf)
                {
                    ParamInfOnCurve2 = _PolyVInf;
                }

                if (ParamSupOnCurve1 < _PolyUInf)
                {
                    ParamSupOnCurve1 = _PolyUInf;
                }
                if (ParamSupOnCurve2 < _PolyVInf)
                {
                    ParamSupOnCurve2 = _PolyVInf;
                }
            }

            PolyUInf = ParamInfOnCurve1;
            PolyUSup = ParamSupOnCurve1;
            PolyVInf = ParamInfOnCurve2;
            PolyVSup = ParamSupOnCurve2;

            TheCurveTool.D0(C1, PolyUInf, out P1);
            TheCurveTool.D0(C1, PolyVInf, out P2);
            double distmemesens = P1.SquareDistance(P2);
            TheCurveTool.D0(C1, PolyVSup, out P2);
            double distdiffsens = P1.SquareDistance(P2);
            if (distmemesens > distdiffsens)
            {
                double qwerty = PolyVInf;
                PolyVInf = PolyVSup;
                PolyVSup = qwerty;
            }

            //-----------------------------------------------------------------
            //-- Calculate Positions of Points on the curve and
            //-- Transitions on each limit of the segment

            IntRes2d_Position Pos1 = IntRes2d_Middle;
            IntRes2d_Position Pos2 = IntRes2d_Middle;
            IntRes2d_Transition Trans1 = new(), Trans2 = new();

            TheCurveTool.D1(C1, PolyUInf, out P1, out Tan1);
            TheCurveTool.D1(C1, PolyVInf, out P2, out Tan2);

            if (P1.Distance(DomainOnCurve1.FirstPoint()) <= DomainOnCurve1.FirstTolerance())
            {
                Pos1 = IntRes2d_Head;
            }
            else if (P1.Distance(DomainOnCurve1.LastPoint()) <= DomainOnCurve1.LastTolerance())
            {
                Pos1 = IntRes2d_End;
            }
            if (P2.Distance(DomainOnCurve2.FirstPoint()) <= DomainOnCurve2.FirstTolerance())
            {
                Pos2 = IntRes2d_Head;
            }
            else if (P2.Distance(DomainOnCurve2.LastPoint()) <= DomainOnCurve2.LastTolerance())
            {
                Pos2 = IntRes2d_End;
            }

            if (Pos1 == IntRes2d_Middle && Pos2 != IntRes2d_Middle)
            {
                PolyUInf = TheProjPCur.FindParameter(C1, P2, D1.FirstParameter(), D1.LastParameter(),
                                                     TheCurveTool.EpsX(C1));
            }
            else if (Pos1 != IntRes2d_Middle && Pos2 == IntRes2d_Middle)
            {
                PolyVInf = TheProjPCur.FindParameter(C1, P1, D1.FirstParameter(), D1.LastParameter(),
                                                     TheCurveTool.EpsX(C1));
            }
            else if (Math.Abs(ParamInfOnCurve1 - ParamSupOnCurve1) > Math.Abs(ParamInfOnCurve2 - ParamSupOnCurve2))
            {
                PolyVInf = TheProjPCur.FindParameter(C1, P1, D1.FirstParameter(), D1.LastParameter(),
                                                     TheCurveTool.EpsX(C1));
            }
            else
            {
                PolyUInf = TheProjPCur.FindParameter(C1, P2, D1.FirstParameter(), D1.LastParameter(),
                                                     TheCurveTool.EpsX(C1));
            }

            if (IntImpParGen.DetermineTransition(Pos1, ref Tan1, ref Trans1, Pos2, ref Tan2, ref Trans2, TolConf)
                == false)
            {
                TheCurveTool.D2(C1, PolyUInf, out P1, out Tan1, out Norm1);
                TheCurveTool.D2(C1, PolyVInf, out P2, out Tan2, out Norm2);
                IntImpParGen.DetermineTransition(Pos1, ref Tan1, Norm1, ref Trans1, Pos2, ref Tan2, Norm2, ref Trans2,
                                                 TolConf);
            }
            IntRes2d_IntersectionPoint PtSeg1 = new(P1, PolyUInf, PolyVInf, Trans1, Trans2, false);
            //----------------------------------------------------------------------

            if ((Math.Abs(PolyUInf - PolyUSup) <= TheCurveTool.EpsX(C1))
                || (Math.Abs(PolyVInf - PolyVSup) <= TheCurveTool.EpsX(C1)))
            {
                // bad segment
            }
            else
            {
                TheCurveTool.D1(C1, PolyUSup, out P1, out Tan1);
                TheCurveTool.D1(C1, PolyVSup, out P2, out Tan2);
                Pos1 = IntRes2d_Middle;
                Pos2 = IntRes2d_Middle;

                if (P1.Distance(DomainOnCurve1.FirstPoint()) <= DomainOnCurve1.FirstTolerance())
                {
                    Pos1 = IntRes2d_Head;
                }
                else if (P1.Distance(DomainOnCurve1.LastPoint()) <= DomainOnCurve1.LastTolerance())
                {
                    Pos1 = IntRes2d_End;
                }
                if (P2.Distance(DomainOnCurve2.FirstPoint()) <= DomainOnCurve2.FirstTolerance())
                {
                    Pos2 = IntRes2d_Head;
                }
                else if (P2.Distance(DomainOnCurve2.LastPoint()) <= DomainOnCurve2.LastTolerance())
                {
                    Pos2 = IntRes2d_End;
                }

                if (Pos1 == IntRes2d_Middle && Pos2 != IntRes2d_Middle)
                {
                    PolyUSup = TheProjPCur.FindParameter(C1, P2, D1.FirstParameter(), D1.LastParameter(),
                                                         TheCurveTool.EpsX(C1));
                }
                else if (Pos1 != IntRes2d_Middle && Pos2 == IntRes2d_Middle)
                {
                    PolyVSup = TheProjPCur.FindParameter(C1, P1, D1.FirstParameter(), D1.LastParameter(),
                                                         TheCurveTool.EpsX(C1));
                }
                else if (Math.Abs(ParamInfOnCurve1 - ParamSupOnCurve1)
                         > Math.Abs(ParamInfOnCurve2 - ParamSupOnCurve2))
                {
                    PolyVSup = TheProjPCur.FindParameter(C1, P1, D1.FirstParameter(), D1.LastParameter(),
                                                         TheCurveTool.EpsX(C1));
                }
                else
                {
                    PolyUSup = TheProjPCur.FindParameter(C1, P2, D1.FirstParameter(), D1.LastParameter(),
                                                         TheCurveTool.EpsX(C1));
                }

                if (IntImpParGen.DetermineTransition(Pos1, ref Tan1, ref Trans1, Pos2, ref Tan2, ref Trans2, TolConf)
                    == false)
                {
                    TheCurveTool.D2(C1, PolyUSup, out P1, out Tan1, out Norm1);
                    TheCurveTool.D2(C1, PolyVSup, out P2, out Tan2, out Norm2);
                    IntImpParGen.DetermineTransition(Pos1, ref Tan1, Norm1, ref Trans1, Pos2, ref Tan2, Norm2,
                                                     ref Trans2, TolConf);
                }
                IntRes2d_IntersectionPoint PtSeg2 = new(P1, PolyUSup, PolyVSup, Trans1, Trans2, false);

                bool Oppos = (Tan1.Dot(Tan2) > 0.0) ? false : true;
                if (ParamInfOnCurve1 > ParamSupOnCurve1)
                {
                    IntRes2d_IntersectionSegment Seg = new(PtSeg2, PtSeg1, Oppos, false);
                    Append(Seg);
                }
                else
                {
                    IntRes2d_IntersectionSegment Seg = new(PtSeg1, PtSeg2, Oppos, false);
                    Append(Seg);
                }
            }
        } // end of processing of TangentZone

        done = true;
    }

    private static bool HeadOrEndPoint(in IntRes2d_Domain D1,
                                       in TheCurve C1,
                                       double tu,
                                       in IntRes2d_Domain D2,
                                       in TheCurve C2,
                                       double tv,
                                       double TolConf,
                                       ref IntRes2d_IntersectionPoint IntPt,
                                       ref bool HeadOn1,
                                       ref bool HeadOn2,
                                       ref bool EndOn1,
                                       ref bool EndOn2,
                                       int PosSegment)
    {
        gp_Pnt2d P1, P2, SP1 = new(), SP2 = new();
        gp_Vec2d T1, T2, N1, N2;
        double u = tu;
        double v = tv;
        double svu = u;
        double svv = v;

        TheCurveTool.D1(C1, u, out P1, out T1);
        TheCurveTool.D1(C2, v, out P2, out T2);

        IntRes2d_Position Pos1 = IntRes2d_Middle;
        IntRes2d_Position Pos2 = IntRes2d_Middle;
        IntRes2d_Transition Trans1 = new(), Trans2 = new();

        //----------------------------------------------------------------------
        //-- Head On 1   :        Head1 <-> P2
        if (P2.Distance(D1.FirstPoint()) <= D1.FirstTolerance())
        {
            Pos1 = IntRes2d_Head;
            HeadOn1 = true;
            SP1 = D1.FirstPoint();
            u = D1.FirstParameter();
        }
        //----------------------------------------------------------------------
        //-- End On 1   :         End1 <-> P2
        else if (P2.Distance(D1.LastPoint()) <= D1.LastTolerance())
        {
            Pos1 = IntRes2d_End;
            EndOn1 = true;
            SP1 = D1.LastPoint();
            u = D1.LastParameter();
        }

        //----------------------------------------------------------------------
        //-- Head On 2   :        Head2 <-> P1
        else if (P1.Distance(D2.FirstPoint()) <= D2.FirstTolerance())
        {
            Pos2 = IntRes2d_Head;
            HeadOn2 = true;
            SP2 = D2.FirstPoint();
            v = D2.FirstParameter();
        }
        //----------------------------------------------------------------------
        //-- End On 2   :        End2 <-> P1
        else if (P1.Distance(D2.LastPoint()) <= D2.LastTolerance())
        {
            Pos2 = IntRes2d_End;
            EndOn2 = true;
            SP2 = D2.LastPoint();
            v = D2.LastParameter();
        }

        double EpsX1 = TheCurveTool.EpsX(C1);
        double EpsX2 = TheCurveTool.EpsX(C2);

        if ((Pos1 != IntRes2d_Middle) || (Pos2 != IntRes2d_Middle))
        {
            if (Pos1 == IntRes2d_Middle)
            {
                if (Math.Abs(u - D1.FirstParameter()) <= EpsX1)
                {
                    Pos1 = IntRes2d_Head;
                    P1 = D1.FirstPoint();
                    HeadOn1 = true;
                }
                else if (Math.Abs(u - D1.LastParameter()) <= EpsX1)
                {
                    Pos1 = IntRes2d_End;
                    P1 = D1.LastPoint();
                    EndOn1 = true;
                }
            }
            else if (u != tu)
            {
                P1 = SP1;
            }

            if (Pos2 == IntRes2d_Middle)
            {
                if (Math.Abs(v - D2.FirstParameter()) <= EpsX2)
                {
                    Pos2 = IntRes2d_Head;
                    HeadOn2 = true;
                    P2 = D2.FirstPoint();
                    if (Pos1 != IntRes2d_Middle)
                    {
                        P1.SetCoord(0.5 * (P1.X() + P2.X()), 0.5 * (P1.Y() + P2.Y()));
                    }
                    else
                    {
                        P2 = P1;
                    }
                }
                else if (Math.Abs(v - D2.LastParameter()) <= EpsX2)
                {
                    Pos2 = IntRes2d_End;
                    EndOn2 = true;
                    P2 = D2.LastPoint();
                    if (Pos1 != IntRes2d_Middle)
                    {
                        P1.SetCoord(0.5 * (P1.X() + P2.X()), 0.5 * (P1.Y() + P2.Y()));
                    }
                    else
                    {
                        P2 = P1;
                    }
                }
            }
            // OCCT: SP2 is assigned and never read (P2 is not moved to the end of the second domain, as P1 is to SP1).
            _ = SP2;

            //--------------------------------------------------------------------
            //-- It is tested if a point at the end of segment already has its transitions
            //-- If Yes, the new point is not created
            //--
            //-- PosSegment =            1    if Head Head
            //--                       2      if Head End
            //--                     4        if End  Head
            //--                   8          if End  End
            //--------------------------------------------------------------------
            if (Pos1 == IntRes2d_Head)
            {
                if ((Pos2 == IntRes2d_Head) && (PosSegment & 1) != 0)
                {
                    return (false);
                }
                if ((Pos2 == IntRes2d_End) && (PosSegment & 2) != 0)
                {
                    return (false);
                }
            }
            else if (Pos1 == IntRes2d_End)
            {
                if ((Pos2 == IntRes2d_Head) && (PosSegment & 4) != 0)
                {
                    return (false);
                }
                if ((Pos2 == IntRes2d_End) && (PosSegment & 8) != 0)
                {
                    return (false);
                }
            }

            if (IntImpParGen.DetermineTransition(Pos1, ref T1, ref Trans1, Pos2, ref T2, ref Trans2, TolConf) == false)
            {
                TheCurveTool.D2(C1, svu, out P1, out T1, out N1);
                TheCurveTool.D2(C2, svv, out P2, out T2, out N2);
                IntImpParGen.DetermineTransition(Pos1, ref T1, N1, ref Trans1, Pos2, ref T2, N2, ref Trans2, TolConf);
            }
            IntPt.SetValues(P1, u, v, Trans1, Trans2, false);
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
    protected void Perform(in TheCurve C1, in IntRes2d_Domain D1, in TheCurve C2, in IntRes2d_Domain D2,
                           double TolConf, double Tol, int NbIter, double DeltaU, double DeltaV)
    {
        int nbsamplesOnC1, nbsamplesOnC2;
        done = false;

        if (NbIter > NBITER_MAX_POLYGON)
        {
            return;
        }

        // Number of samples running.
        nbsamplesOnC1 = TheCurveTool.NbSamples(C1, D1.FirstParameter(), D1.LastParameter());
        nbsamplesOnC2 = TheCurveTool.NbSamples(C2, D2.FirstParameter(), D2.LastParameter());

        if (NbIter == 0)
        {
            // Minimal number of points.
            nbsamplesOnC1 = Max(nbsamplesOnC1, myMinPntNb);
            nbsamplesOnC2 = Max(nbsamplesOnC2, myMinPntNb);
        }
        else
        {
            // Increase number of samples in second and next iterations.
            nbsamplesOnC1 = (5 * (nbsamplesOnC1 * NbIter)) / 4;
            nbsamplesOnC2 = (5 * (nbsamplesOnC2 * NbIter)) / 4;
        }

        IntCurve_Polygon2dGen<TheCurve, TheCurveTool> aPoly1 = new(C1, nbsamplesOnC1, D1, Tol),
                                                       aPoly2 = new(C2, nbsamplesOnC2, D2, Tol);

        if ((aPoly1.DeflectionOverEstimation() > TolConf) && (aPoly2.DeflectionOverEstimation() > TolConf))
        {
            double aDeflectionSum = Max(aPoly1.DeflectionOverEstimation(), TolConf)
                                    + Max(aPoly2.DeflectionOverEstimation(), TolConf);

            if (nbsamplesOnC2 > nbsamplesOnC1)
            {
                aPoly2.ComputeWithBox(C2, aPoly1.Bounding());
                aPoly1.SetDeflectionOverEstimation(aDeflectionSum);
                aPoly1.ComputeWithBox(C1, aPoly2.Bounding());
            }
            else
            {
                aPoly1.ComputeWithBox(C1, aPoly2.Bounding());
                aPoly2.SetDeflectionOverEstimation(aDeflectionSum);
                aPoly2.ComputeWithBox(C2, aPoly1.Bounding());
            }
        }

        //----------------------------------------------------------------------
        //-- if the deflection less then the Tolerance of Confusion
        //-- Then the deflection of the polygon is set in TolConf
        //-- (Detection of Tangency Zones)
        //----------------------------------------------------------------------

        if (aPoly1.DeflectionOverEstimation() < TolConf)
        {
            aPoly1.SetDeflectionOverEstimation(TolConf);
        }
        if (aPoly2.DeflectionOverEstimation() < TolConf)
        {
            aPoly2.SetDeflectionOverEstimation(TolConf);
        }
        // for case when a few polygon points were replaced by line
        // if exact solution was not found
        // then search of precise solution will be repeated
        // for polygon contains all initial points
        // secondary search will be performed only for case when initial points
        // were dropped
        bool isFullRepresentation =
            (aPoly1.NbSegments() == nbsamplesOnC1 && aPoly2.NbSegments() == nbsamplesOnC2);

        if (!findIntersect(C1, D1, C2, D2, TolConf, Tol, NbIter, DeltaU, DeltaV, aPoly1, aPoly2,
                           isFullRepresentation)
            && !isFullRepresentation)
        {
            if (aPoly1.NbSegments() < nbsamplesOnC1)
            {
                aPoly1 = new IntCurve_Polygon2dGen<TheCurve, TheCurveTool>(C1, nbsamplesOnC1, D1, Tol);
            }
            if (aPoly2.NbSegments() < nbsamplesOnC2)
            {
                aPoly2 = new IntCurve_Polygon2dGen<TheCurve, TheCurveTool>(C2, nbsamplesOnC2, D2, Tol);
            }

            findIntersect(C1, D1, C2, D2, TolConf, Tol, NbIter, DeltaU, DeltaV, aPoly1, aPoly2, true);
        }

        done = true;
    }

    //======================================================================
    // Purpose : findIntersect
    //======================================================================
    private bool findIntersect(in TheCurve C1,
                               in IntRes2d_Domain D1,
                               in TheCurve C2,
                               in IntRes2d_Domain D2,
                               double TolConf,
                               double Tol,
                               int NbIter,
                               double DeltaU,
                               double DeltaV,
                               IntCurve_Polygon2dGen<TheCurve, TheCurveTool> thePoly1,
                               IntCurve_Polygon2dGen<TheCurve, TheCurveTool> thePoly2,
                               bool isFullPolygon)
    {
        gp_Vec2d Tan1, Tan2, Norm1, Norm2;
        gp_Pnt2d P1, P2;
        Intf_InterferencePolygon2d InterPP = new(thePoly1, thePoly2);
        IntCurve_ExactIntersectionPoint<TheCurve, TheCurveTool> EIP = new(C1, C2, TolConf);
        double U = 0.0, V = 0.0;
        bool AnErrorOccurred = false;
        done = true; // To prevent exception in nbp=NbPoints();
        //----------------------------------------------------------------------
        //-- Processing of SectionPoint
        //----------------------------------------------------------------------
        int Nbsp = InterPP.NbSectionPoints();
        for (int sp = 1; sp <= Nbsp; sp++)
        {
            Intf_SectionPoint SPnt = InterPP.PntValue(sp);
            int SegIndex1, SegIndex2;
            double ParamOn1, ParamOn2;
            Intf_PIType Type;

            SPnt.InfoFirst(out Type, out SegIndex1, out ParamOn1);
            SPnt.InfoSecond(out Type, out SegIndex2, out ParamOn2);
            EIP.Perform(thePoly1, thePoly2, ref SegIndex1, ref SegIndex2, ref ParamOn1, ref ParamOn2);
            AnErrorOccurred = EIP.AnErrorOccurred();

            if (EIP.NbRoots() == 0 && !isFullPolygon)
            {
                return false;
            }

            if (AnErrorOccurred)
            {
                continue;
            }

            //--------------------------------------------------------------------
            //-- It is checked if the found point is really a root
            //--------------------------------------------------------------------

            EIP.Roots(out U, out V);
            TheCurveTool.D1(C1, U, out P1, out Tan1);
            TheCurveTool.D1(C2, V, out P2, out Tan2);
            double Dist = P1.Distance(P2);
            if (EIP.NbRoots() == 0 && Dist > TolConf)
            {
                IntRes2d_Transition aTrans = new();
                IntRes2d_IntersectionPoint aPInt = new(P1, U, V, aTrans, aTrans, false);
                double aT1f, aT1l, aT2f, aT2l;
                aT1f = thePoly1.ApproxParamOnCurve(SegIndex1, 0.0);
                aT1l = thePoly1.ApproxParamOnCurve(SegIndex1, 1.0);
                aT2f = thePoly2.ApproxParamOnCurve(SegIndex2, 0.0);
                aT2l = thePoly2.ApproxParamOnCurve(SegIndex2, 1.0);
                //
                int aMaxCount = 16, aCount = 0;
                GetIntersection(C1, aT1f, aT1l, C2, aT2f, aT2l, TolConf, aMaxCount, ref aPInt, ref Dist, ref aCount);
                U = aPInt.ParamOnFirst();
                V = aPInt.ParamOnSecond();
                TheCurveTool.D1(C1, U, out P1, out Tan1);
                TheCurveTool.D1(C2, V, out P2, out Tan2);
                Dist = P1.Distance(P2);
            }
            //-----------------------------------------------------------------
            //-- It is checked if the point (u,v) does not exist already
            //--
            int nbp = NbPoints();
            double EpsX1 = 10.0 * TheCurveTool.EpsX(C1);
            double EpsX2 = 10.0 * TheCurveTool.EpsX(C2);
            for (int p = 1; p <= nbp; p++)
            {
                IntRes2d_IntersectionPoint P = Point(p);
                if (Math.Abs(U - P.ParamOnFirst()) <= EpsX1)
                {
                    if (Math.Abs(V - P.ParamOnSecond()) <= EpsX2)
                    {
                        Dist = TolConf + 1.0;
                        p += nbp;
                    }
                }
            }

            if (Dist <= TolConf)
            { //-- Or the point is already present
                IntRes2d_Position Pos1 = IntRes2d_Middle;
                IntRes2d_Position Pos2 = IntRes2d_Middle;
                IntRes2d_Transition Trans1 = new(), Trans2 = new();
                //-----------------------------------------------------------------
                //-- Calculate the Positions of Points on the curve
                //--
                if (P1.Distance(DomainOnCurve1.FirstPoint()) <= DomainOnCurve1.FirstTolerance())
                {
                    Pos1 = IntRes2d_Head;
                }
                else if (P1.Distance(DomainOnCurve1.LastPoint()) <= DomainOnCurve1.LastTolerance())
                {
                    Pos1 = IntRes2d_End;
                }

                if (P2.Distance(DomainOnCurve2.FirstPoint()) <= DomainOnCurve2.FirstTolerance())
                {
                    Pos2 = IntRes2d_Head;
                }
                else if (P2.Distance(DomainOnCurve2.LastPoint()) <= DomainOnCurve2.LastTolerance())
                {
                    Pos2 = IntRes2d_End;
                }
                //-----------------------------------------------------------------
                //-- Calculate the Transitions (see IntImpParGen.cxx)
                //--
                if (IntImpParGen.DetermineTransition(Pos1, ref Tan1, ref Trans1, Pos2, ref Tan2, ref Trans2, TolConf)
                    == false)
                {
                    TheCurveTool.D2(C1, U, out P1, out Tan1, out Norm1);
                    TheCurveTool.D2(C2, V, out P2, out Tan2, out Norm2);
                    IntImpParGen.DetermineTransition(Pos1, ref Tan1, Norm1, ref Trans1, Pos2, ref Tan2, Norm2,
                                                     ref Trans2, TolConf);
                }
                IntRes2d_IntersectionPoint IP = new(P1, U, V, Trans1, Trans2, false);
                Insert(IP);
            }
        }

        //----------------------------------------------------------------------
        //-- Processing of TangentZone
        //----------------------------------------------------------------------
        int Nbtz = InterPP.NbTangentZones();
        for (int tz = 1; tz <= Nbtz; tz++)
        {
            int NbPnts = InterPP.ZoneValue(tz).NumberOfPoints();
            //====================================================================
            //== Find the first and the last point in the tangency zone.
            //====================================================================
            double ParamSupOnCurve2, ParamInfOnCurve2;
            double ParamSupOnCurve1, ParamInfOnCurve1;
            int SegIndex1onP1, SegIndex1onP2;
            Intf_PIType Type;
            double ParamOnLine;
            double PolyUInf, PolyUSup, PolyVInf, PolyVSup;
            ParamSupOnCurve2 = ParamSupOnCurve1 = PolyUSup = PolyVSup = -RealLast();
            ParamInfOnCurve2 = ParamInfOnCurve1 = PolyUInf = PolyVInf = RealLast();
            for (int qq = 1; qq <= NbPnts; qq++)
            {
                Intf_SectionPoint SPnt1 = InterPP.ZoneValue(tz).GetPoint(qq);
                //====================================================================
                //== The zones of tangency are discretized
                //== Test of stop : Check if
                //==     (Deflection  < Tolerance)
                //==  Or (Sample < EpsX)   (normally the first condition is
                //==                           more strict)
                //====================================================================
                double _PolyUInf, _PolyVInf;

                SPnt1.InfoFirst(out Type, out SegIndex1onP1, out ParamOnLine);
                if (SegIndex1onP1 > thePoly1.NbSegments())
                {
                    SegIndex1onP1--;
                    ParamOnLine = 1.0;
                }
                if (SegIndex1onP1 <= 0)
                {
                    SegIndex1onP1 = 1;
                    ParamOnLine = 0.0;
                }
                _PolyUInf = thePoly1.ApproxParamOnCurve(SegIndex1onP1, ParamOnLine);

                SPnt1.InfoSecond(out Type, out SegIndex1onP2, out ParamOnLine);
                if (SegIndex1onP2 > thePoly2.NbSegments())
                {
                    SegIndex1onP2--;
                    ParamOnLine = 1.0;
                }
                if (SegIndex1onP2 <= 0)
                {
                    SegIndex1onP2 = 1;
                    ParamOnLine = 0.0;
                }
                _PolyVInf = thePoly2.ApproxParamOnCurve(SegIndex1onP2, ParamOnLine);

                //----------------------------------------------------------------------

                if (ParamInfOnCurve1 > _PolyUInf)
                {
                    ParamInfOnCurve1 = _PolyUInf;
                }
                if (ParamInfOnCurve2 > _PolyVInf)
                {
                    ParamInfOnCurve2 = _PolyVInf;
                }

                if (ParamSupOnCurve1 < _PolyUInf)
                {
                    ParamSupOnCurve1 = _PolyUInf;
                }
                if (ParamSupOnCurve2 < _PolyVInf)
                {
                    ParamSupOnCurve2 = _PolyVInf;
                }
            }

            PolyUInf = ParamInfOnCurve1;
            PolyUSup = ParamSupOnCurve1;
            PolyVInf = ParamInfOnCurve2;
            PolyVSup = ParamSupOnCurve2;

            TheCurveTool.D0(C1, PolyUInf, out P1);
            TheCurveTool.D0(C2, PolyVInf, out P2);
            double distmemesens = P1.SquareDistance(P2);
            TheCurveTool.D0(C2, PolyVSup, out P2);
            double distdiffsens = P1.SquareDistance(P2);
            if (distmemesens > distdiffsens)
            {
                double qwerty = PolyVInf;
                PolyVInf = PolyVSup;
                PolyVSup = qwerty;
            }

            if (((thePoly1.DeflectionOverEstimation() > TolConf) || (thePoly2.DeflectionOverEstimation() > TolConf))
                && (NbIter < NBITER_MAX_POLYGON))
            {
                IntRes2d_Domain RecursD1 = new(TheCurveTool.Value(C1, ParamInfOnCurve1),
                                               ParamInfOnCurve1,
                                               TolConf,
                                               TheCurveTool.Value(C1, ParamSupOnCurve1),
                                               ParamSupOnCurve1,
                                               TolConf);
                IntRes2d_Domain RecursD2 = new(TheCurveTool.Value(C2, ParamInfOnCurve2),
                                               ParamInfOnCurve2,
                                               TolConf,
                                               TheCurveTool.Value(C2, ParamSupOnCurve2),
                                               ParamSupOnCurve2,
                                               TolConf);
                //-- thePoly1(2) are not deleted,
                //-- finally they are destroyed.
                //-- !! No untimely return !!
                // OCCT passes Tol as TolConf and TolConf as Tol to the recursion.
                Perform(C1, RecursD1, C2, RecursD2, Tol, TolConf, NbIter + 1, DeltaU, DeltaV);
            }
            else
            {
                //-----------------------------------------------------------------
                //-- Calculate Positions of Points on the curve and
                //-- Transitions on each limit of the segment

                IntRes2d_Position Pos1 = IntRes2d_Middle;
                IntRes2d_Position Pos2 = IntRes2d_Middle;
                IntRes2d_Transition Trans1 = new(), Trans2 = new();

                TheCurveTool.D1(C1, PolyUInf, out P1, out Tan1);
                TheCurveTool.D1(C2, PolyVInf, out P2, out Tan2);

                if (P1.Distance(DomainOnCurve1.FirstPoint()) <= DomainOnCurve1.FirstTolerance())
                {
                    Pos1 = IntRes2d_Head;
                }
                else if (P1.Distance(DomainOnCurve1.LastPoint()) <= DomainOnCurve1.LastTolerance())
                {
                    Pos1 = IntRes2d_End;
                }
                if (P2.Distance(DomainOnCurve2.FirstPoint()) <= DomainOnCurve2.FirstTolerance())
                {
                    Pos2 = IntRes2d_Head;
                }
                else if (P2.Distance(DomainOnCurve2.LastPoint()) <= DomainOnCurve2.LastTolerance())
                {
                    Pos2 = IntRes2d_End;
                }

                if (Pos1 == IntRes2d_Middle && Pos2 != IntRes2d_Middle)
                {
                    PolyUInf = TheProjPCur.FindParameter(C1, P2, D1.FirstParameter(), D1.LastParameter(),
                                                         TheCurveTool.EpsX(C1));
                }
                else if (Pos1 != IntRes2d_Middle && Pos2 == IntRes2d_Middle)
                {
                    PolyVInf = TheProjPCur.FindParameter(C2, P1, D2.FirstParameter(), D2.LastParameter(),
                                                         TheCurveTool.EpsX(C2));
                }
                else if (Math.Abs(ParamInfOnCurve1 - ParamSupOnCurve1)
                         > Math.Abs(ParamInfOnCurve2 - ParamSupOnCurve2))
                {
                    PolyVInf = TheProjPCur.FindParameter(C2, P1, D2.FirstParameter(), D2.LastParameter(),
                                                         TheCurveTool.EpsX(C2));
                }
                else
                {
                    PolyUInf = TheProjPCur.FindParameter(C1, P2, D1.FirstParameter(), D1.LastParameter(),
                                                         TheCurveTool.EpsX(C1));
                }

                if (IntImpParGen.DetermineTransition(Pos1, ref Tan1, ref Trans1, Pos2, ref Tan2, ref Trans2, TolConf)
                    == false)
                {
                    TheCurveTool.D2(C1, PolyUInf, out P1, out Tan1, out Norm1);
                    TheCurveTool.D2(C2, PolyVInf, out P2, out Tan2, out Norm2);
                    IntImpParGen.DetermineTransition(Pos1, ref Tan1, Norm1, ref Trans1, Pos2, ref Tan2, Norm2,
                                                     ref Trans2, TolConf);
                }
                IntRes2d_IntersectionPoint PtSeg1 = new(P1, PolyUInf, PolyVInf, Trans1, Trans2, false);
                //----------------------------------------------------------------------

                if ((Math.Abs(PolyUInf - PolyUSup) <= TheCurveTool.EpsX(C1))
                    || (Math.Abs(PolyVInf - PolyVSup) <= TheCurveTool.EpsX(C2)))
                {
                    Insert(PtSeg1);
                }
                else
                {
                    TheCurveTool.D1(C1, PolyUSup, out P1, out Tan1);
                    TheCurveTool.D1(C2, PolyVSup, out P2, out Tan2);
                    Pos1 = IntRes2d_Middle;
                    Pos2 = IntRes2d_Middle;

                    if (P1.Distance(DomainOnCurve1.FirstPoint()) <= DomainOnCurve1.FirstTolerance())
                    {
                        Pos1 = IntRes2d_Head;
                    }
                    else if (P1.Distance(DomainOnCurve1.LastPoint()) <= DomainOnCurve1.LastTolerance())
                    {
                        Pos1 = IntRes2d_End;
                    }
                    if (P2.Distance(DomainOnCurve2.FirstPoint()) <= DomainOnCurve2.FirstTolerance())
                    {
                        Pos2 = IntRes2d_Head;
                    }
                    else if (P2.Distance(DomainOnCurve2.LastPoint()) <= DomainOnCurve2.LastTolerance())
                    {
                        Pos2 = IntRes2d_End;
                    }

                    if (Pos1 == IntRes2d_Middle && Pos2 != IntRes2d_Middle)
                    {
                        PolyUSup = TheProjPCur.FindParameter(C1, P2, D1.FirstParameter(), D1.LastParameter(),
                                                             TheCurveTool.EpsX(C1));
                    }
                    else if (Pos1 != IntRes2d_Middle && Pos2 == IntRes2d_Middle)
                    {
                        PolyVSup = TheProjPCur.FindParameter(C2, P1, D2.FirstParameter(), D2.LastParameter(),
                                                             TheCurveTool.EpsX(C2));
                    }
                    else if (Math.Abs(ParamInfOnCurve1 - ParamSupOnCurve1)
                             > Math.Abs(ParamInfOnCurve2 - ParamSupOnCurve2))
                    {
                        PolyVSup = TheProjPCur.FindParameter(C2, P1, D2.FirstParameter(), D2.LastParameter(),
                                                             TheCurveTool.EpsX(C2));
                    }
                    else
                    {
                        PolyUSup = TheProjPCur.FindParameter(C1, P2, D1.FirstParameter(), D1.LastParameter(),
                                                             TheCurveTool.EpsX(C1));
                    }

                    if (IntImpParGen.DetermineTransition(Pos1, ref Tan1, ref Trans1, Pos2, ref Tan2, ref Trans2,
                                                         TolConf)
                        == false)
                    {
                        TheCurveTool.D2(C1, PolyUSup, out P1, out Tan1, out Norm1);
                        TheCurveTool.D2(C2, PolyVSup, out P2, out Tan2, out Norm2);
                        IntImpParGen.DetermineTransition(Pos1, ref Tan1, Norm1, ref Trans1, Pos2, ref Tan2, Norm2,
                                                         ref Trans2, TolConf);
                    }
                    IntRes2d_IntersectionPoint PtSeg2 = new(P1, PolyUSup, PolyVSup, Trans1, Trans2, false);

                    bool Oppos = (Tan1.Dot(Tan2) > 0.0) ? false : true;
                    if (ParamInfOnCurve1 > ParamSupOnCurve1)
                    {
                        IntRes2d_IntersectionSegment Seg = new(PtSeg2, PtSeg1, Oppos, false);
                        Append(Seg);
                    }
                    else
                    {
                        IntRes2d_IntersectionSegment Seg = new(PtSeg1, PtSeg2, Oppos, false);
                        Append(Seg);
                    }
                }
            }
        }
        return true;
    }

    //======================================================================
    //  GetIntersection
    //======================================================================
    private static void GetIntersection(in TheCurve theC1,
                                        double theT1f,
                                        double theT1l,
                                        in TheCurve theC2,
                                        double theT2f,
                                        double theT2l,
                                        double theTolConf,
                                        int theMaxCount,
                                        ref IntRes2d_IntersectionPoint thePInt,
                                        ref double theDist,
                                        ref int theCount)
    {
        theCount++;
        //
        double aTol2 = theTolConf * theTolConf;
        double aPTol1 = Max(100.0 * Epsilon(Max(Math.Abs(theT1f), Math.Abs(theT1l))), Precision.PConfusion());
        double aPTol2 = Max(100.0 * Epsilon(Max(Math.Abs(theT2f), Math.Abs(theT2l))), Precision.PConfusion());
        gp_Pnt2d aP1f, aP1l, aP2f, aP2l;
        Bnd_Box2d aB1 = new(), aB2 = new();
        //
        TheCurveTool.D0(theC1, theT1f, out aP1f);
        TheCurveTool.D0(theC1, theT1l, out aP1l);
        aB1.Add(aP1f);
        aB1.Add(aP1l);
        aB1.Enlarge(theTolConf);
        //
        TheCurveTool.D0(theC2, theT2f, out aP2f);
        TheCurveTool.D0(theC2, theT2l, out aP2l);
        aB2.Add(aP2f);
        aB2.Add(aP2l);
        aB2.Enlarge(theTolConf);
        //
        if (aB1.IsOut(aB2))
        {
            theCount--;
            return;
        }
        //
        bool isSmall1 = (theT1l - theT1f) <= aPTol1 || aP1f.SquareDistance(aP1l) / 4.0 <= aTol2;
        bool isSmall2 = (theT2l - theT2f) <= aPTol2 || aP2f.SquareDistance(aP2l) / 4.0 <= aTol2;

        if ((isSmall1 && isSmall2) || (theCount > theMaxCount))
        {
            // Seems to be intersection
            // Simple treatment of segment intersection
            Span<gp_XY> aPnts1 = [aP1f.XY(), (aP1f.XY() + aP1l.XY()) / 2.0, aP1l.XY()];
            Span<gp_XY> aPnts2 = [aP2f.XY(), (aP2f.XY() + aP2l.XY()) / 2.0, aP2l.XY()];
            int i, j, imin = -1, jmin = -1;
            double dmin = RealLast(), d;
            for (i = 0; i < 3; i++)
            {
                for (j = 0; j < 3; j++)
                {
                    d = (aPnts1[i] - aPnts2[j]).SquareModulus();
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
            if (theDist > dmin)
            {
                theDist = dmin;
                //
                double t1;
                if (imin == 0)
                {
                    t1 = theT1f;
                }
                else if (imin == 1)
                {
                    t1 = (theT1f + theT1l) / 2.0;
                }
                else
                {
                    t1 = theT1l;
                }
                //
                double t2;
                if (jmin == 0)
                {
                    t2 = theT2f;
                }
                else if (jmin == 1)
                {
                    t2 = (theT2f + theT2l) / 2.0;
                }
                else
                {
                    t2 = theT2l;
                }
                //
                gp_Pnt2d aPint = new((aPnts1[imin] + aPnts2[jmin]) / 2.0);
                //
                IntRes2d_Transition aTrans1 = new(), aTrans2 = new();
                thePInt.SetValues(aPint, t1, t2, aTrans1, aTrans2, false);
            }
            theCount--;
            return;
        }

        if (isSmall1)
        {
            double aT2m = (theT2l + theT2f) / 2.0;
            GetIntersection(theC1, theT1f, theT1l, theC2, theT2f, aT2m, theTolConf, theMaxCount, ref thePInt,
                            ref theDist, ref theCount);
            GetIntersection(theC1, theT1f, theT1l, theC2, aT2m, theT2l, theTolConf, theMaxCount, ref thePInt,
                            ref theDist, ref theCount);
        }
        else if (isSmall2)
        {
            double aT1m = (theT1l + theT1f) / 2.0;
            GetIntersection(theC1, theT1f, aT1m, theC2, theT2f, theT2l, theTolConf, theMaxCount, ref thePInt,
                            ref theDist, ref theCount);
            GetIntersection(theC1, aT1m, theT1l, theC2, theT2f, theT2l, theTolConf, theMaxCount, ref thePInt,
                            ref theDist, ref theCount);
        }
        else
        {
            double aT1m = (theT1l + theT1f) / 2.0;
            double aT2m = (theT2l + theT2f) / 2.0;
            GetIntersection(theC1, theT1f, aT1m, theC2, theT2f, aT2m, theTolConf, theMaxCount, ref thePInt,
                            ref theDist, ref theCount);
            GetIntersection(theC1, theT1f, aT1m, theC2, aT2m, theT2l, theTolConf, theMaxCount, ref thePInt,
                            ref theDist, ref theCount);
            GetIntersection(theC1, aT1m, theT1l, theC2, theT2f, aT2m, theTolConf, theMaxCount, ref thePInt,
                            ref theDist, ref theCount);
            GetIntersection(theC1, aT1m, theT1l, theC2, aT2m, theT2l, theTolConf, theMaxCount, ref thePInt,
                            ref theDist, ref theCount);
        }
    }

    /// <summary>The minimal number of samples of each curve's polygon.</summary>
    public int GetMinNbSamples() => myMinPntNb;

    /// <summary>Sets the minimal number of samples of each curve's polygon (20 by default).</summary>
    public void SetMinNbSamples(int theMinNbSamples)
    {
        myMinPntNb = theMinNbSamples;
    }
}
