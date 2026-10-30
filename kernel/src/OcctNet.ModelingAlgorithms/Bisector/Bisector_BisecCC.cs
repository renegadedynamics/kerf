// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_BisecCC.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_BisecCC.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// The bisector of two curves, which may intersect only at their ends. Its parameter is the first curve's; it is
/// computed through a polygon of up to 21 points (Bisector_PolyBis) and may be extended at its ends by segments.
/// </summary>
/// <remarks>
/// A geometry class: its mutators (Perform, Transform and OCCT's private setters) are internal or private and check
/// that the curve isn't frozen. It owns copies of the curves it was given, except that ChangeGuide's bisector shares
/// them with this one, as OCCT's handles do; Transform copies a frozen curve before changing it. Reverse throws
/// Standard_NotImplemented, as in OCCT. ValueAndDist and ValueByInt take their outputs by <c>ref</c>: OCCT leaves
/// them unwritten on some paths.
/// </remarks>
public sealed class Bisector_BisecCC : Bisector_Curve
{
    private Geom2d_Curve curve1 = null!;
    private Geom2d_Curve curve2 = null!;
    private double sign1;
    private double sign2;
    private NCollection_Sequence<double> startIntervals = new();
    private NCollection_Sequence<double> endIntervals = new();
    // OCCT's currentInterval, read only by Dump, is left out.
    private Bisector_PolyBis myPolygon = new();
    private double shiftParameter;
    private double distMax;
    private bool isEmpty;
    private bool isConvex1;
    private bool isConvex2;
    private bool extensionStart;
    private bool extensionEnd;
    private gp_Pnt2d pointStart;
    private gp_Pnt2d pointEnd;

    /// <summary>An empty bisector.</summary>
    public Bisector_BisecCC()
    {
        sign1 = 0.0;
        sign2 = 0.0;
        shiftParameter = 0.0;
        distMax = 0.0;
        isEmpty = true;
        isConvex1 = false;
        isConvex2 = false;
        extensionStart = false;
        extensionEnd = false;
    }

    /// <summary>
    /// The bisector of Cu1 and Cu2, on the left of Cu1 (resp. Cu2) if Side1 (resp. Side2) is 1, on its right if -1,
    /// starting from Origin, limited to the points closer to the curves than DistMax.
    /// </summary>
    public Bisector_BisecCC(Geom2d_Curve Cu1, Geom2d_Curve Cu2, double Side1, double Side2, in gp_Pnt2d Origin,
                            double DistMax = 500)
    {
        Perform(Cu1, Cu2, Side1, Side2, Origin, DistMax);
    }

    /// <summary>
    /// Computes the bisector of Cu1 and Cu2, on the left of Cu1 (resp. Cu2) if Side1 (resp. Side2) is 1, on its
    /// right if -1, starting from Origin, limited to the points closer to the curves than DistMax.
    /// </summary>
    internal void Perform(Geom2d_Curve Cu1, Geom2d_Curve Cu2, double Side1, double Side2, in gp_Pnt2d Origin,
                          double DistMax = 500)
    {
        CheckMutable();
        isEmpty = false;
        distMax = DistMax;

        curve1 = (Geom2d_Curve)Cu1.Copy();
        curve2 = (Geom2d_Curve)Cu2.Copy();

        sign1 = Side1;
        sign2 = Side2;

        isConvex1 = Bisector.IsConvex(curve1, sign1);
        isConvex2 = Bisector.IsConvex(curve2, sign2);

        double U = 0.0, UC1 = 0.0, UC2 = 0.0, Dist = 0.0, dU, USol;
        gp_Pnt2d P;
        int NbPnts = 21;
        double EpsMin = 10 * Precision.Confusion();
        bool YaPoly = true;
        bool OriInPoly = false;
        //---------------------------------------------
        // Calculate first point of the polygon.
        //---------------------------------------------
        bool isProjDone = ProjOnCurve(Origin, curve1, out U);

        if (!isProjDone)
        {
            isEmpty = true;
            return;
        }

        P = ValueByInt(U, ref UC1, ref UC2, ref Dist);
        if (Dist < Precision.Confusion())
        {
            gp_Pnt2d aP1 = curve1.Value(UC1);
            gp_Pnt2d aP2 = curve2.Value(UC2);
            double dp = (aP1.Distance(P) + aP2.Distance(P));
            double dorig = (aP1.Distance(Origin) + aP2.Distance(Origin));
            if (dp < dorig)
            {
                isEmpty = true;
                return;
            }
        }

        if (Dist < Precision.Infinite())
        {
            //----------------------------------------------------
            // the parameter of the origin point gives a point
            // on the polygon.
            //----------------------------------------------------
            myPolygon.Append(new Bisector_PointOnBis(UC1, UC2, U, Dist, P));
            startIntervals.Append(U);
            if (P.IsEqual(Origin, Precision.Confusion()))
            {
                //----------------------------------------
                // test if the first point is the origin.
                //----------------------------------------
                OriInPoly = true;
            }
        }
        else
        {
            //-------------------------------------------------------
            // The origin point is on the extension.
            // Find the first point of the polygon by dichotomy.
            //-------------------------------------------------------
            dU = (curve1.LastParameter() - U) / (NbPnts - 1);
            U += dU;
            for (int i = 1; i <= NbPnts - 1; i++)
            {
                P = ValueByInt(U, ref UC1, ref UC2, ref Dist);
                if (Dist < Precision.Infinite())
                {
                    USol = SearchBound(U - dU, U);
                    P = ValueByInt(USol, ref UC1, ref UC2, ref Dist);
                    startIntervals.Append(USol);
                    myPolygon.Append(new Bisector_PointOnBis(UC1, UC2, USol, Dist, P));
                    break;
                }
                U += dU;
            }
        }

        if (myPolygon.Length() != 0)
        {
            SupLastParameter();
            //----------------------------------------------
            // Construction of the polygon of the bissectrice.
            //---------------------------------------------
            U = FirstParameter();
            double DU = LastParameter() - U;

            if (DU < EpsMin)
            {
                NbPnts = 3;
            }
            dU = DU / (NbPnts - 1);

            U += dU;
            //  modified by NIZHNY-EAP Fri Jan 21 09:33:20 2000 ___BEGIN___
            //  prevent addition of the same point
            gp_Pnt2d prevPnt = P;
            for (int i = 1; i <= NbPnts - 1; i++)
            {
                P = ValueByInt(U, ref UC1, ref UC2, ref Dist);
                if (Dist < Precision.Infinite())
                {
                    if (P.Distance(prevPnt) > Precision.Confusion())
                    {
                        myPolygon.Append(new Bisector_PointOnBis(UC1, UC2, U, Dist, P));
                    }
                }
                else
                {
                    USol = SearchBound(U - dU, U);
                    P = ValueByInt(USol, ref UC1, ref UC2, ref Dist);
                    endIntervals.SetValue(1, USol);
                    if (P.Distance(prevPnt) > Precision.Confusion())
                    {
                        myPolygon.Append(new Bisector_PointOnBis(UC1, UC2, USol, Dist, P));
                    }
                    break;
                }
                U += dU;
                prevPnt = P;
                //  modified by NIZHNY-EAP Fri Jan 21 09:33:24 2000 ___END___
            }
        }
        else
        {
            //----------------
            // Empty Polygon.
            //----------------
            YaPoly = false;
        }

        extensionStart = false;
        extensionEnd = false;
        pointStart = Origin;

        if (isConvex1 && isConvex2)
        {
            if (YaPoly)
            {
                pointEnd = myPolygon.Last().Point();
            }
        }
        else
        {
            //-----------------------------------------------------------------------------
            // Extension : The curve is extended at the beginning and/or the end if
            //                - one of two curves is concave.
            //                - the curves have a common point at the beginning and/or the end
            //                - the angle of opening at the common point between two curves
            //                  values M_PI.
            // the extension at the beginning  is taken into account if the origin is found above.
            // ie : the origin is not the in the polygon.
            //-----------------------------------------------------------------------------

            //---------------------------------
            // Do the extensions exist ?
            //---------------------------------
            if (OriInPoly)
            {
                extensionStart = false;
            }
            else
            {
                extensionStart = TestExtension(curve1, curve2, 1);
            }
            extensionEnd = TestExtension(curve1, curve2, 2);

            //-----------------
            // Calculate pointEnd.
            //-----------------
            if (extensionEnd)
            {
                pointEnd = curve1.Value(curve1.LastParameter());
            }
            else if (YaPoly)
            {
                pointEnd = myPolygon.Last().Point();
            }
            else
            {
                ComputePointEnd();
            }
            //------------------------------------------------------
            // Update the Limits of intervals of  definition.
            //------------------------------------------------------
            if (YaPoly)
            {
                if (extensionStart)
                {
                    gp_Pnt2d P1 = myPolygon.First().Point();
                    double UFirst = startIntervals.First() - pointStart.Distance(P1);
                    startIntervals.InsertBefore(1, UFirst);
                    endIntervals.InsertBefore(1, startIntervals.Value(2));
                }
                if (extensionEnd)
                {
                    gp_Pnt2d P1;
                    double UFirst, ULast;
                    P1 = myPolygon.Last().Point();
                    UFirst = endIntervals.Last();
                    ULast = UFirst + pointEnd.Distance(P1);
                    startIntervals.Append(UFirst);
                    endIntervals.Append(ULast);
                }
            }
            else
            {
                //--------------------------------------------------
                // No polygon => the bissectrice is a segment.
                //--------------------------------------------------
                startIntervals.Append(0.0);
                endIntervals.Append(pointEnd.Distance(pointStart));
            }
        }
        if (!YaPoly && !extensionStart && !extensionEnd)
        {
            isEmpty = true;
        }
        //  modified by NIZHNY-EAP Mon Jan 17 17:32:40 2000 ___BEGIN___
        if (myPolygon.Length() <= 2)
        {
            isEmpty = true;
        }
        //  modified by NIZHNY-EAP Mon Jan 17 17:32:42 2000 ___END___
    }

    /// <summary>True if the bisector is extended at its start.</summary>
    public override bool IsExtendAtStart() => extensionStart;

    /// <summary>True if the bisector is extended at its end.</summary>
    public override bool IsExtendAtEnd() => extensionEnd;

    /// <summary>True if the bisector is empty.</summary>
    public bool IsEmpty() => isEmpty;

    /// <summary>Not implemented in OCCT.</summary>
    /// <exception cref="Standard_NotImplemented">Always.</exception>
    protected internal override void Reverse()
    {
        CheckMutable();
        throw new Standard_NotImplemented();
    }

    /// <summary>LastParameter() + FirstParameter() - U.</summary>
    public override double ReversedParameter(double U) => LastParameter() + FirstParameter() - U;

    /// <summary>A new, unfrozen bisector with copies of the curves and of the polygon.</summary>
    public override Geom2d_Geometry Copy()
    {
        Geom2d_Curve CopyCurve1 = (Geom2d_Curve)curve1.Copy();
        Geom2d_Curve CopyCurve2 = (Geom2d_Curve)curve2.Copy();

        Bisector_BisecCC C = new();

        C.Curve(1, CopyCurve1);
        C.Curve(2, CopyCurve2);
        C.Sign(1, sign1);
        C.Sign(2, sign2);
        C.IsConvex(1, isConvex1);
        C.IsConvex(2, isConvex2);
        C.Polygon(myPolygon);
        C.IsEmpty(isEmpty);
        C.DistMax(distMax);
        C.StartIntervals(startIntervals);
        C.EndIntervals(endIntervals);
        C.ExtensionStart(extensionStart);
        C.ExtensionEnd(extensionEnd);
        C.PointStart(pointStart);
        C.PointEnd(pointEnd);

        return C;
    }

    //=============================================================================
    //           ATTENTION : - This can invert the direction  of parameterization.
    //                       - This concerns only the part of the curve
    //                         corresponding to the polygon.
    //=============================================================================
    /// <summary>
    /// The same bisector parameterised by the second curve (the curves exchanged; the new bisector shares them with
    /// this one). Only the part of the bisector covered by the polygon is kept; the parameterisation may be reversed.
    /// </summary>
    public Bisector_BisecCC ChangeGuide()
    {
        Bisector_BisecCC C = new();

        C.Curve(1, curve2);
        C.Curve(2, curve1);
        C.Sign(1, sign2);
        C.Sign(2, sign1);
        C.IsConvex(1, isConvex2);
        C.IsConvex(2, isConvex1);

        //-------------------------------------------------------------------------
        // Construction of the new polygon from the initial one.
        // inversion of PointOnBis and Calculation of new parameters on the bissectrice.
        //-------------------------------------------------------------------------
        Bisector_PolyBis Poly = new();
        if (sign1 == sign2)
        {
            //---------------------------------------------------------------
            // elements of the new polygon are ranked in the other direction.
            //---------------------------------------------------------------
            for (int i = myPolygon.Length(); i >= 1; i--)
            {
                Bisector_PointOnBis P = myPolygon.Value(i);
                Bisector_PointOnBis NewP = new(P.ParamOnC2(), P.ParamOnC1(), P.ParamOnC2(), P.Distance(), P.Point());
                Poly.Append(NewP);
            }
        }
        else
        {
            for (int i = 1; i <= myPolygon.Length(); i++)
            {
                Bisector_PointOnBis P = myPolygon.Value(i);
                Bisector_PointOnBis NewP = new(P.ParamOnC2(), P.ParamOnC1(), P.ParamOnC2(), P.Distance(), P.Point());
                Poly.Append(NewP);
            }
        }
        C.Polygon(Poly);
        C.FirstParameter(Poly.First().ParamOnBis());
        C.LastParameter(Poly.Last().ParamOnBis());

        return C;
    }

    /// <summary>Transforms the curves, the polygon and the end points.</summary>
    protected internal override void Transform(in gp_Trsf2d T)
    {
        CheckMutable();
        // OCCT.NET: a curve frozen elsewhere is copied before it changes.
        if (curve1.IsFrozen())
        {
            curve1 = (Geom2d_Curve)curve1.Copy();
        }
        if (curve2.IsFrozen())
        {
            curve2 = (Geom2d_Curve)curve2.Copy();
        }
        curve1.Transform(T);
        curve2.Transform(T);
        myPolygon.Transform(T);
        pointStart.Transform(T);
        pointEnd.Transform(T);
    }

    /// <summary>True if both curves are C(N+1).</summary>
    public override bool IsCN(int N) => (curve1.IsCN(N + 1) && curve2.IsCN(N + 1));

    /// <summary>The start of the first interval.</summary>
    public override double FirstParameter() => startIntervals.First();

    /// <summary>The end of the last interval.</summary>
    public override double LastParameter() => endIntervals.Last();

    /// <summary>The first curve's continuity less one.</summary>
    public override GeomAbs_Shape Continuity()
    {
        GeomAbs_Shape Cont = curve1.Continuity();
        switch (Cont)
        {
            case GeomAbs_C1:
                return GeomAbs_C0;
            case GeomAbs_C2:
                return GeomAbs_C1;
            case GeomAbs_C3:
                return GeomAbs_C2;
            case GeomAbs_CN:
                return GeomAbs_CN;
            default:
                break;
        }
        return GeomAbs_C0;
    }

    /// <summary>The number of intervals.</summary>
    public override int NbIntervals() => startIntervals.Length();

    /// <summary>The start of the interval Index.</summary>
    public override double IntervalFirst(int Index) => startIntervals.Value(Index);

    /// <summary>The end of the interval Index.</summary>
    public override double IntervalLast(int Index) => endIntervals.Value(Index);

    /// <summary>The continuity in the intervals: the first curve's less one.</summary>
    public GeomAbs_Shape IntervalContinuity()
    {
        GeomAbs_Shape Cont = curve1.Continuity();
        switch (Cont)
        {
            case GeomAbs_C1:
                return GeomAbs_C0;
            case GeomAbs_C2:
                return GeomAbs_C1;
            case GeomAbs_C3:
                return GeomAbs_C2;
            case GeomAbs_CN:
                return GeomAbs_CN;
            default:
                break;
        }
        return GeomAbs_C0;
    }

    /// <summary>True if the first curve is closed and the bisector covers it.</summary>
    public override bool IsClosed()
    {
        if (curve1.IsClosed())
        {
            if (startIntervals.First() == curve1.FirstParameter() && endIntervals.Last() == curve1.LastParameter())
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>False.</summary>
    public override bool IsPeriodic() => false;

    private static double Curvature(Geom2d_Curve C, double U, double Tol)
    {
        double K1;
        C.D2(U, out gp_Pnt2d _, out gp_Vec2d D1, out gp_Vec2d D2);
        double Norm2 = D1.SquareMagnitude();
        if (Norm2 < Tol)
        {
            K1 = 0.0;
        }
        else
        {
            K1 = (D1 ^ D2) / (Norm2 * Math.Sqrt(Norm2));
        }
        return K1;
    }

    //=============================================================================
    //           Calculate the current point, the distance from the current point to
    //           both curves, the parameters on each curve of the projection
    //           of the current point.
    //
    // method  : - Find start parameter by using <myPolygon>.
    //          - Calculate parameter U2 on curve C2 solution of H(U,V)= 0
    //          - P(U) = F(U,U2)
    //
    //          or :
    //                                 ||P2(v0)P1(u)||**2
    //           F(u,v) = P1(u) - 1/2 *----------------* N(u)
    //                                (N(u).P2(v0)P1(u))
    //
    //           H(u,v) = (Tu.P1(u)P2(v))**2||Tv||**2 - (Tv.P1(u)P2(v))**2||Tu||**2
    //=============================================================================
    /// <summary>
    /// The point of parameter U, with the parameters U1 and U2 of its projections on the curves and its square
    /// distance to them (left unchanged on the paths where OCCT doesn't write them).
    /// </summary>
    public gp_Pnt2d ValueAndDist(double U, ref double U1, ref double U2, ref double Distance)
    {
        //-----------------------------------------------
        // is the polygon reduced to a point or empty?
        //-----------------------------------------------
        if (myPolygon.Length() <= 1)
        {
            return Extension(U, ref U1, ref U2, out Distance, out gp_Vec2d _);
        }

        //-----------------------------------------------
        // test U out of the limits of the polygon.
        //-----------------------------------------------
        if (U < myPolygon.First().ParamOnBis())
        {
            return Extension(U, ref U1, ref U2, out Distance, out gp_Vec2d _);
        }
        if (U > myPolygon.Last().ParamOnBis())
        {
            return Extension(U, ref U1, ref U2, out Distance, out gp_Vec2d _);
        }

        //-------------------------------------------------------
        // Find start parameter by using <myPolygon>.
        //-------------------------------------------------------
        int IntervalIndex = myPolygon.Interval(U);
        double UMin = myPolygon.Value(IntervalIndex).ParamOnBis();
        double UMax = myPolygon.Value(IntervalIndex + 1).ParamOnBis();
        double VMin = myPolygon.Value(IntervalIndex).ParamOnC2();
        double VMax = myPolygon.Value(IntervalIndex + 1).ParamOnC2();
        double Alpha, VInit;

        if (Math.Abs(UMax - UMin) < gp.Resolution())
        {
            VInit = VMin;
        }
        else
        {
            Alpha = (U - UMin) / (UMax - UMin);
            VInit = VMin + Alpha * (VMax - VMin);
        }

        U1 = LinkBisCurve(U);
        double VTemp = Min(VMin, VMax);
        VMax = Max(VMin, VMax);
        VMin = VTemp;
        bool Valid = true;
        //---------------------------------------------------------------
        // Calculate parameter U2 on curve C2 solution of H(u,v)=0
        //---------------------------------------------------------------
        double EpsH = 1.0E-9;
        double EpsH100 = 1.0E-7;
        curve1.D1(U1, out gp_Pnt2d P1, out gp_Vec2d T1);
        gp_Vec2d N1 = new(T1.Y(), -T1.X());

        if ((VMax - VMin) < Precision.PConfusion())
        {
            U2 = VInit;
        }
        else
        {
            Bisector_FunctionH H = new(curve2, P1, sign1 * sign2 * T1);
            H.Value(VInit, out double FInit);
            if (Math.Abs(FInit) < EpsH)
            {
                U2 = VInit;
            }
            else
            {
                math_BissecNewton aNewSolution = new(EpsH);
                aNewSolution.Perform(H, VMin - EpsH100, VMax + EpsH100, 10);

                if (aNewSolution.IsDone())
                {
                    U2 = aNewSolution.Root();
                }
                else
                {
                    math_FunctionRoot SolRoot = new(H, VInit, EpsH, VMin - EpsH100, VMax + EpsH100);

                    if (SolRoot.IsDone())
                    {
                        U2 = SolRoot.Root();
                    }
                    else
                    {
                        Valid = false;
                    }
                }
            }
        }

        gp_Pnt2d PBis = pointStart;
        //----------------
        // P(U) = F(U1,U2)
        //----------------
        if (Valid)
        {
            gp_Pnt2d P2 = curve2.Value(U2);
            gp_Vec2d P2P1 = new(P1.X() - P2.X(), P1.Y() - P2.Y());
            double SquareP2P1 = P2P1.SquareMagnitude();
            double N1P2P1 = N1.Dot(P2P1);
            double anEps = Epsilon(1.0);

            if (P1.IsEqual(P2, Precision.Confusion()))
            {
                PBis = P1;
                Distance = 0.0;
            }
            else if (N1P2P1 * sign1 < anEps)
            {
                Valid = false;
            }
            else
            {
                PBis = P1.Translated(-(0.5 * SquareP2P1 / N1P2P1) * N1);
                Distance = P1.SquareDistance(PBis);
            }
        }

        //----------------------------------------------------------------
        // If the point is not valid
        // calculate by intersection.
        //----------------------------------------------------------------
        if (!Valid)
        {
            //--------------------------------------------------------------------
            // Construction of the bisectrice point curve and of the straight line passing
            // by P1 and carried by the normal. curve2 is limited by VMin and VMax.
            //--------------------------------------------------------------------
            double DMin = Precision.Infinite();
            gp_Pnt2d P;
            Bisector_BisecPC BisPC = new(curve2, P1, sign2, VMin, VMax);
            Geom2d_Line NorLi = new(P1, new gp_Dir2d(N1));

            Geom2dAdaptor_Curve ABisPC = new(BisPC);
            Geom2dAdaptor_Curve ANorLi = new(NorLi);
            //-------------------------------------------------------------------------
            Geom2dInt_GInter Intersect = new(ABisPC, ANorLi, Precision.Confusion(), Precision.Confusion());
            //-------------------------------------------------------------------------

            if (Intersect.IsDone() && !Intersect.IsEmpty())
            {
                for (int i = 1; i <= Intersect.NbPoints(); i++)
                {
                    if (Intersect.Point(i).ParamOnSecond() * sign1 < Precision.PConfusion())
                    {
                        P = Intersect.Point(i).Value();
                        if (P.SquareDistance(P1) < DMin)
                        {
                            DMin = P.SquareDistance(P1);
                            PBis = P;
                            U2 = BisPC.LinkBisCurve(Intersect.Point(i).ParamOnFirst());
                            Distance = DMin;
                        }
                    }
                }
            }
        }
        return PBis;
    }

    //=============================================================================
    //           Calculate the current point, the distance from the current point
    //           to two curves, the parameters on each curve of the projection of the
    //           current point.
    //           the current point with parameter U is the intersection of the
    //           bissectrice point curve (P1,curve2) and of the straight line
    //           passing through P1 of director vector N1.
    //           P1 is the current point of parameter U on curve1 and N1 the
    //           normal at this point.
    //=============================================================================
    /// <summary>
    /// The point of parameter U computed by the roots of H on the second curve, with the parameters U1 and U2 of its
    /// projections and its square distance (infinite where there is no point; U2 left unchanged where OCCT
    /// doesn't write it).
    /// </summary>
    public gp_Pnt2d ValueByInt(double U, ref double U1, ref double U2, ref double Distance)
    {
        //------------------------------------------------------------------
        // Return point, tangent, normal on C1 at parameter U.
        //-------------------------------------------------------------------
        U1 = LinkBisCurve(U);

        gp_Pnt2d P2, P, PSol = new();

        curve1.D1(U1, out gp_Pnt2d P1, out gp_Vec2d Tan1);
        gp_Vec2d N1 = new(Tan1.Y(), -Tan1.X());

        //--------------------------------------------------------------------------
        // test confusion of P1 with extremity of curve2.
        //--------------------------------------------------------------------------
        if (P1.Distance(curve2.Value(curve2.FirstParameter())) < Precision.Confusion())
        {
            U2 = curve2.FirstParameter();
            curve2.D1(U2, out P2, out gp_Vec2d Tan2);
            if (isConvex1 && isConvex2)
            {
                Distance = 0.0;
                return P1;
            }
            if (!Tan1.IsParallel(Tan2, Precision.Angular()))
            {
                Distance = 0.0;
                return P1;
            }
        }
        if (P1.Distance(curve2.Value(curve2.LastParameter())) < Precision.Confusion())
        {
            U2 = curve2.LastParameter();
            curve2.D1(U2, out P2, out gp_Vec2d Tan2);
            if (isConvex1 && isConvex2)
            {
                Distance = 0.0;
                return P1;
            }
            if (!Tan1.IsParallel(Tan2, Precision.Angular()))
            {
                Distance = 0.0;
                return P1;
            }
        }

        bool YaSol = false;
        double DMin = Precision.Infinite();
        double USol;
        double EpsMax = 1.0E-6;
        double EpsX;
        double EpsH = 1.0E-8;
        double DistPP1;
        int NbSamples = 20;
        double UFirstOnC2 = curve2.FirstParameter();
        double ULastOnC2 = curve2.LastParameter();

        if (!myPolygon.IsEmpty())
        {
            if (sign1 == sign2)
            {
                ULastOnC2 = myPolygon.Last().ParamOnC2();
            }
            else
            {
                UFirstOnC2 = myPolygon.Last().ParamOnC2();
            }
        }

        if (Math.Abs(ULastOnC2 - UFirstOnC2) < Precision.PConfusion() / 100.0)
        {
            Distance = Precision.Infinite();
            return P1;
        }

        DiscretPar(Math.Abs(ULastOnC2 - UFirstOnC2), EpsH, EpsMax, 2, 20, out EpsX, out NbSamples);

        Bisector_FunctionH H = new(curve2, P1, sign1 * sign2 * Tan1);
        math_FunctionRoots SolRoot = new(H, UFirstOnC2, ULastOnC2, NbSamples, EpsX, EpsH, EpsH);
        if (SolRoot.IsDone())
        {
            for (int j = 1; j <= SolRoot.NbSolutions(); j++)
            {
                USol = SolRoot.Value(j);
                gp_Pnt2d P2Curve2 = curve2.Value(USol);
                gp_Vec2d P2P1 = new(P1.X() - P2Curve2.X(), P1.Y() - P2Curve2.Y());
                double SquareP2P1 = P2P1.SquareMagnitude();
                double N1P2P1 = N1.Dot(P2P1);

                // Test if the solution is at the proper side of the curves.
                if (N1P2P1 * sign1 > 0)
                {
                    P = P1.Translated(-(0.5 * SquareP2P1 / N1P2P1) * N1);
                    DistPP1 = P1.SquareDistance(P);
                    if (DistPP1 < DMin)
                    {
                        DMin = DistPP1;
                        PSol = P;
                        U2 = USol;
                        YaSol = true;
                    }
                }
            }
        }

        // OCCT's commented-out search by intersection with a Bisector_BisecPC is left out.

        if (YaSol)
        {
            Distance = DMin;
            //--------------------------------------------------------------
            // Point found => Test curve distance + Angular Test
            //---------------------------------------------------------------
            P2 = curve2.Value(U2);
            gp_Vec2d PP1 = new(P1.X() - PSol.X(), P1.Y() - PSol.Y());
            gp_Vec2d PP2 = new(P2.X() - PSol.X(), P2.Y() - PSol.Y());

            //-----------------------------------------------
            // Dist = product of norms = distance at the square.
            //-----------------------------------------------
            if (PP1.Dot(PP2) > (1.0 - Precision.Angular()) * Distance)
            {
                YaSol = false;
            }
            else
            {
                if (!isConvex1)
                {
                    double K1 = Curvature(curve1, U1, Precision.Confusion());
                    if (K1 != 0.0)
                    {
                        if (Distance > 1 / (K1 * K1))
                        {
                            YaSol = false;
                        }
                    }
                }
                if (YaSol)
                {
                    if (!isConvex2)
                    {
                        double K2 = Curvature(curve2, U2, Precision.Confusion());
                        if (K2 != 0.0)
                        {
                            if (Distance > 1 / (K2 * K2))
                            {
                                YaSol = false;
                            }
                        }
                    }
                }
            }
        }

        if (!YaSol)
        {
            Distance = Precision.Infinite();
            PSol = P1;
        }
        return PSol;
    }

    /// <summary>The point of parameter U.</summary>
    public override gp_Pnt2d EvalD0(double U)
    {
        double U1 = 0.0, U2 = 0.0, Dist = 0.0;
        return ValueAndDist(U, ref U1, ref U2, ref Dist);
    }

    /// <summary>The point and first derivative at U.</summary>
    public override ResD1 EvalD1(double U)
    {
        ResD1 aResult;
        Values(U, 1, out aResult.Point, out aResult.D1, out gp_Vec2d _, out gp_Vec2d _);
        return aResult;
    }

    /// <summary>The point and first derivative at U (OCCT computes no second derivative: it is 0).</summary>
    public override ResD2 EvalD2(double U)
    {
        ResD2 aResult;
        Values(U, 2, out aResult.Point, out aResult.D1, out aResult.D2, out gp_Vec2d _);
        return aResult;
    }

    /// <summary>The point and first derivative at U (OCCT computes no higher derivatives: they are 0).</summary>
    public override ResD3 EvalD3(double U)
    {
        ResD3 aResult;
        Values(U, 3, out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
        return aResult;
    }

    /// <summary>The derivative of order N (1 to 3) at U (0 for N = 2 or 3, as OCCT computes them).</summary>
    /// <exception cref="Geom2d_UndefinedDerivative">N is not 1, 2 or 3.</exception>
    public override gp_Vec2d EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Geom2d_UndefinedDerivative("Bisector_BisecCC::EvalDN");
        }
        Values(U, N, out gp_Pnt2d _, out gp_Vec2d V1, out gp_Vec2d V2, out gp_Vec2d V3);
        switch (N)
        {
            case 1:
                return V1;
            case 2:
                return V2;
            case 3:
                return V3;
            default:
            {
                throw new Geom2d_UndefinedDerivative("Bisector_BisecCC::EvalDN");
            }
        }
    }

    //=============================================================================
    //           the bissectrice is defined by:
    //
    //           B(u) = F(u,v0)
    //           where v0 = Phi(u) is given by H (u,v) = 0.
    //
    //           with :
    //                                 ||P2(v0)P1(u)||**2
    //           F(u,v) = P1(u) - 1/2 *----------------* N(u)
    //                                (N(u).P2(v0)P1(u))
    //
    //           H(u,v) = (Tu.P1(u)P2(v))**2||Tv||**2 - (Tv.P1(u)P2(v))**2||Tu||**2
    //
    //           => dB(u)/du = dF/du + dF/dv(- dH/du:dH/dv)
    //
    //           Note : tangent to the bisectrice is bissectrice at
    //                      tangents  T1(u) and T2(v0)
    //=============================================================================
    private void Values(double U, int N, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2, out gp_Vec2d V3)
    {
        V1 = new gp_Vec2d(0.0, 0.0);
        V2 = new gp_Vec2d(0.0, 0.0);
        V3 = new gp_Vec2d(0.0, 0.0);
        //-------------------------------------------------------------------------
        // Calculate the current point on the bisectrice and the parameters on each
        // curve.
        //-------------------------------------------------------------------------
        double U0 = 0.0, V0 = 0.0, Dist = 0.0;

        //-----------------------------------------------
        // is the polygon reduced to a point or empty?
        //-----------------------------------------------
        if (myPolygon.Length() <= 1)
        {
            P = Extension(U, ref U0, ref V0, out Dist, out V1); // OCCT: no return (occt-issues.md).
        }
        if (U < myPolygon.First().ParamOnBis())
        {
            P = Extension(U, ref U0, ref V0, out Dist, out V1);
            return;
        }
        if (U > myPolygon.Last().ParamOnBis())
        {
            P = Extension(U, ref U0, ref V0, out Dist, out V1);
            return;
        }
        P = ValueAndDist(U, ref U0, ref V0, ref Dist);

        if (N == 0)
        {
            return;
        }
        //------------------------------------------------------------------
        // Return point, tangent, normal to C1 by parameter U0.
        //-------------------------------------------------------------------
        curve1.D2(U0, out gp_Pnt2d P1, out gp_Vec2d Tu, out gp_Vec2d Tuu); // point on C1, tangent, 2nd derivative.

        gp_Vec2d Nor = new(-Tu.Y(), Tu.X()); // Normal by U0.
        gp_Vec2d Nu = new(-Tuu.Y(), Tuu.X()); // derivative of the normal by U0.

        //-------------------------------------------------------------------
        // Return point, tangent, normale to C2 by parameter V0.
        //-------------------------------------------------------------------
        curve2.D2(V0, out gp_Pnt2d P2, out gp_Vec2d Tv, out gp_Vec2d Tvv); // point on C2, tangent, 2nd derivative.

        gp_Vec2d PuPv = new(P2.X() - P1.X(), P2.Y() - P1.Y());

        //-----------------------------
        // Calculate dH/du and dH/dv.
        //-----------------------------
        double TuTu, TvTv, TuTv;
        double TuPuPv, TvPuPv;
        double TuuPuPv, TuTuu;
        double TvvPuPv, TvTvv;

        TuTu = Tu.Dot(Tu);
        TvTv = Tv.Dot(Tv);
        TuTv = Tu.Dot(Tv);

        TuPuPv = Tu.Dot(PuPv);
        TvPuPv = Tv.Dot(PuPv);

        TuuPuPv = Tuu.Dot(PuPv);
        TuTuu = Tu.Dot(Tuu);

        TvvPuPv = Tvv.Dot(PuPv);
        TvTvv = Tv.Dot(Tvv);

        double dHdu = 2 * (TuPuPv * (TuuPuPv - TuTu) * TvTv + TvPuPv * TuTv * TuTu - TuTuu * TvPuPv * TvPuPv);
        double dHdv = 2 * (TuPuPv * TuTv * TvTv + TvTvv * TuPuPv * TuPuPv - TvPuPv * (TvvPuPv + TvTv) * TuTu);

        //-----------------------------
        // Calculate dF/du and dF/dv.
        //-----------------------------
        double NorPuPv, NuPuPv, NorTv;
        double A, B, dAdu, dAdv, dBdu, dBdv, BB;

        NorPuPv = Nor.Dot(PuPv);
        NuPuPv = Nu.Dot(PuPv);
        NorTv = Nor.Dot(Tv);

        A = 0.5 * PuPv.SquareMagnitude();
        B = -NorPuPv;
        BB = B * B;
        dAdu = -TuPuPv;
        dBdu = -NuPuPv;
        dAdv = TvPuPv;
        dBdv = -NorTv;

        //---------------------------------------
        // F(u,v) = Pu - (A(u,v)/B(u,v))*Nor(u)
        //----------------------------------------
        if (BB < gp.Resolution())
        {
            V1 = Tu.Normalized() + Tv.Normalized();
            V1 = 0.5 * Tu.SquareMagnitude() * V1;
        }
        else
        {
            gp_Vec2d dFdu = Tu - (dAdu / B - dBdu * A / BB) * Nor - (A / B) * Nu;
            gp_Vec2d dFdv = (-dAdv / B + dBdv * A / BB) * Nor;

            if (Math.Abs(dHdv) > gp.Resolution())
            {
                V1 = dFdu + dFdv * (-dHdu / dHdv);
            }
            else
            {
                V1 = Tu;
            }
        }
        if (N == 1)
        {
            return;
        }
    }

    //=============================================================================
    //           Extension of the bisector by tangence of the curve.
    //=============================================================================
    private gp_Pnt2d Extension(double U, ref double U1, ref double U2, out double Dist, out gp_Vec2d T)
    {
        Bisector_PointOnBis PRef = new();
        gp_Pnt2d P = new(), P1 = new(), P2, PBis = new();
        gp_Vec2d Tang = new();
        double dU = 0.0;
        bool ExtensionTangent = false;

        if (myPolygon.Length() == 0)
        {
            //---------------------------------------------
            // Empty Polygon => segment (pointStart,pointEnd)
            //---------------------------------------------
            dU = U - startIntervals.First();
            P = pointStart;
            P1 = pointEnd;
            U1 = curve1.LastParameter();
            if (sign1 == sign2)
            {
                U2 = curve2.FirstParameter();
            }
            else
            {
                U2 = curve2.LastParameter();
            }
            Tang.SetCoord(P1.X() - P.X(), P1.Y() - P.Y());
        }
        else if (U < myPolygon.First().ParamOnBis())
        {
            PRef = myPolygon.First();
            P = PRef.Point();
            dU = U - PRef.ParamOnBis();
            if (extensionStart)
            {
                //------------------------------------------------------------
                // extension = segment (pointstart, first point of the polygon.)
                //------------------------------------------------------------
                P1 = pointStart;
                U1 = curve1.FirstParameter();
                if (sign1 == sign2)
                {
                    U2 = curve2.LastParameter();
                }
                else
                {
                    U2 = curve2.FirstParameter();
                }
                Tang.SetCoord(P.X() - P1.X(), P.Y() - P1.Y());
            }
            else
            {
                ExtensionTangent = true;
            }
        }
        else if (U > myPolygon.Last().ParamOnBis())
        {
            PRef = myPolygon.Last();
            P = PRef.Point();
            dU = U - PRef.ParamOnBis();
            if (extensionEnd)
            {
                //------------------------------------------------------------
                // extension = segment (last point of the polygon.pointEnd)
                //------------------------------------------------------------
                P1 = pointEnd;
                U1 = curve1.LastParameter();
                if (sign1 == sign2)
                {
                    U2 = curve2.LastParameter();
                }
                else
                {
                    U2 = curve2.FirstParameter();
                }
                Tang.SetCoord(P1.X() - P.X(), P1.Y() - P.Y());
            }
            else
            {
                ExtensionTangent = true;
            }
        }

        if (ExtensionTangent)
        {
            //-----------------------------------------------------------
            // If the la curve has no a extension, it is extended by tangency
            //------------------------------------------------------------
            U1 = PRef.ParamOnC1();
            U2 = PRef.ParamOnC2();
            P2 = curve2.Value(U2);
            curve1.D1(U1, out P1, out gp_Vec2d T1);
            Tang.SetCoord(2 * P.X() - P1.X() - P2.X(), 2 * P.Y() - P1.Y() - P2.Y());
            if (Tang.Magnitude() < Precision.Confusion())
            {
                Tang = T1;
            }
            if (T1.Dot(Tang) < 0.0)
            {
                Tang = -Tang;
            }
        }

        T = Tang.Normalized();
        PBis.SetCoord(P.X() + dU * T.X(), P.Y() + dU * T.Y());
        Dist = P1.Distance(PBis);
        return PBis;
    }

    //=============================================================================
    //           The point of the bisector of CA and CB whose projection on CA is the point
    //           of parameter UOnA.
    //=============================================================================
    private static bool PointByInt(Geom2d_Curve CA, Geom2d_Curve CB, double SignA, double SignB, double UOnA,
                                   ref double UOnB, ref double Dist)
    {
        //------------------------------------------------------------------
        // Return point,tangent, normal on CA with parameter UOnA.
        //-------------------------------------------------------------------
        gp_Pnt2d P2, P, PSol = new();
        bool IsConvexA = Bisector.IsConvex(CA, SignA);
        bool IsConvexB = Bisector.IsConvex(CB, SignB);

        CA.D1(UOnA, out gp_Pnt2d P1, out gp_Vec2d Tan1);
        gp_Vec2d N1 = new(Tan1.Y(), -Tan1.X());

        //--------------------------------------------------------------------------
        // test of confusion of P1 with extremity of curve2.
        //--------------------------------------------------------------------------
        if (P1.Distance(CB.Value(CB.FirstParameter())) < Precision.Confusion())
        {
            UOnB = CB.FirstParameter();
            CB.D1(UOnB, out P2, out gp_Vec2d Tan2);
            if (IsConvexA && IsConvexB)
            {
                Dist = 0.0;
                return true;
            }
            if (!Tan1.IsParallel(Tan2, Precision.Angular()))
            {
                Dist = 0.0;
                return false;
            }
        }
        if (P1.Distance(CB.Value(CB.LastParameter())) < Precision.Confusion())
        {
            UOnB = CB.LastParameter();
            CB.D1(UOnB, out P2, out gp_Vec2d Tan2);
            if (IsConvexA && IsConvexB)
            {
                Dist = 0.0;
                return true;
            }
            if (!Tan1.IsParallel(Tan2, Precision.Angular()))
            {
                Dist = 0.0;
                return false;
            }
        }

        double DMin = Precision.Infinite();
        double UPC;
        bool YaSol = false;
        //--------------------------------------------------------------------
        // Construction of the bisectrice point curve and of the straight line passing
        // through P1 and carried by the normal.
        //--------------------------------------------------------------------
        Bisector_BisecPC BisPC = new(CB, P1, SignB);
        //-------------------------------
        // Test if the bissectrice exists.
        //-------------------------------
        if (BisPC.IsEmpty())
        {
            Dist = Precision.Infinite();
            return false;
        }

        Geom2d_Line NorLi = new(P1, new gp_Dir2d(N1));

        Geom2dAdaptor_Curve ABisPC = new(BisPC);
        Geom2dAdaptor_Curve ANorLi = new(NorLi);
        //-------------------------------------------------------------------------
        Geom2dInt_GInter Intersect = new(ABisPC, ANorLi, Precision.Confusion(), Precision.Confusion());
        //-------------------------------------------------------------------------

        if (Intersect.IsDone() && !Intersect.IsEmpty())
        {
            for (int i = 1; i <= Intersect.NbPoints(); i++)
            {
                if (Intersect.Point(i).ParamOnSecond() * SignA < Precision.PConfusion())
                {
                    P = Intersect.Point(i).Value();
                    if (P.SquareDistance(P1) < DMin)
                    {
                        DMin = P.SquareDistance(P1);
                        PSol = P;
                        UPC = Intersect.Point(i).ParamOnFirst();
                        UOnB = BisPC.LinkBisCurve(UPC);
                        Dist = DMin;
                        YaSol = true;
                    }
                }
            }
        }

        if (YaSol)
        {
            //--------------------------------------------------------------
            // Point found => Test distance curvature + Angular test
            //---------------------------------------------------------------
            P2 = CB.Value(UOnB);
            if (P1.SquareDistance(PSol) < 1.0e-32)
            {
                YaSol = false;
                return YaSol;
            }
            if (P2.SquareDistance(PSol) < 1.0e-32)
            {
                YaSol = false;
                return YaSol;
            }

            gp_Dir2d PP1Unit = new(P1.X() - PSol.X(), P1.Y() - PSol.Y());
            gp_Dir2d PP2Unit = new(P2.X() - PSol.X(), P2.Y() - PSol.Y());

            if (PP1Unit * PP2Unit > 1.0 - Precision.Angular())
            {
                YaSol = false;
            }
            else
            {
                Dist = Math.Sqrt(Dist);
                if (!IsConvexA)
                {
                    double K1 = Curvature(CA, UOnA, Precision.Confusion());
                    if (K1 != 0.0)
                    {
                        if (Dist > Math.Abs(1 / K1))
                        {
                            YaSol = false;
                        }
                    }
                }
                if (YaSol)
                {
                    if (!IsConvexB)
                    {
                        double K2 = Curvature(CB, UOnB, Precision.Confusion());
                        if (K2 != 0.0)
                        {
                            if (Dist > Math.Abs(1 / K2))
                            {
                                YaSol = false;
                            }
                        }
                    }
                }
            }
        }
        return YaSol;
    }

    private void SupLastParameter()
    {
        endIntervals.Append(curve1.LastParameter());
        // ----------------------------------------------------------------------
        // Calculate parameter on curve1 associated to one or the other of the extremities
        // of curve2 following the values of sign1 and sign2.
        // the bissectrice is limited by the obtained parameters.
        //------------------------------------------------------------------------
        double UOnC1 = 0.0, UOnC2, Dist = 0.0; // OCCT: UOnC1 and Dist uninitialised (see PointByInt).
        if (sign1 == sign2)
        {
            UOnC2 = curve2.FirstParameter();
        }
        else
        {
            UOnC2 = curve2.LastParameter();
        }
        bool YaSol = PointByInt(curve2, curve1, sign2, sign1, UOnC2, ref UOnC1, ref Dist);
        if (YaSol)
        {
            if (UOnC1 > startIntervals.First() && UOnC1 < endIntervals.Last())
            {
                endIntervals.SetValue(1, UOnC1);
            }
        }
    }

    /// <summary>The curve I (1 or 2), shared with this bisector as OCCT's handle.</summary>
    /// <exception cref="Standard_OutOfRange">I is not 1 or 2.</exception>
    public Geom2d_Curve Curve(int I)
    {
        if (I == 1)
        {
            return curve1;
        }
        else if (I == 2)
        {
            return curve2;
        }
        else
        {
            throw new Standard_OutOfRange();
        }
    }

    /// <summary>The parameter on the first curve of the projection of the point of parameter U.</summary>
    public double LinkBisCurve(double U) => (U - shiftParameter);

    /// <summary>The inverse of LinkBisCurve.</summary>
    public double LinkCurveBis(double U) => (U + shiftParameter);

    /// <summary>The polygon of the bisector (a working object of this bisector: don't keep it).</summary>
    public Bisector_PolyBis Polygon() => myPolygon;

    /// <summary>The parameter on the bisector of the point P.</summary>
    public override double Parameter(in gp_Pnt2d P)
    {
        double UOnCurve;

        if (P.IsEqual(Value(FirstParameter()), Precision.Confusion()))
        {
            UOnCurve = FirstParameter();
        }
        else if (P.IsEqual(Value(LastParameter()), Precision.Confusion()))
        {
            UOnCurve = LastParameter();
        }
        else
        {
            ProjOnCurve(P, curve1, out UOnCurve);
        }
        return UOnCurve;
    }

    // OCCT's Dump (std::cout) is left out.

    private void Curve(int I, Geom2d_Curve C)
    {
        if (I == 1)
        {
            curve1 = C;
        }
        else if (I == 2)
        {
            curve2 = C;
        }
        else
        {
            throw new Standard_OutOfRange();
        }
    }

    private void Sign(int I, double S)
    {
        if (I == 1)
        {
            sign1 = S;
        }
        else if (I == 2)
        {
            sign2 = S;
        }
        else
        {
            throw new Standard_OutOfRange();
        }
    }

    private void Polygon(Bisector_PolyBis P) => myPolygon = new Bisector_PolyBis(P);

    private void DistMax(double D) => distMax = D;

    private void IsConvex(int I, bool IsConvex)
    {
        if (I == 1)
        {
            isConvex1 = IsConvex;
        }
        else if (I == 2)
        {
            isConvex2 = IsConvex;
        }
        else
        {
            throw new Standard_OutOfRange();
        }
    }

    private void IsEmpty(bool IsEmpty) => isEmpty = IsEmpty;

    private void ExtensionStart(bool ExtensionStart) => extensionStart = ExtensionStart;

    private void ExtensionEnd(bool ExtensionEnd) => extensionEnd = ExtensionEnd;

    private void PointStart(in gp_Pnt2d Point) => pointStart = Point;

    private void PointEnd(in gp_Pnt2d Point) => pointEnd = Point;

    private void StartIntervals(NCollection_Sequence<double> StartIntervals) =>
        startIntervals = new NCollection_Sequence<double>(StartIntervals);

    private void EndIntervals(NCollection_Sequence<double> EndIntervals) =>
        endIntervals = new NCollection_Sequence<double>(EndIntervals);

    private void FirstParameter(double U) => startIntervals.Append(U);

    private void LastParameter(double U) => endIntervals.Append(U);

    private double SearchBound(double U1, double U2)
    {
        double UMid, Dist1 = 0.0, Dist2 = 0.0, DistMid = 0.0, U11, U22;
        double UC1 = 0.0, UC2 = 0.0;
        gp_Pnt2d PBis, PBisPrec;
        double TolPnt = Precision.Confusion();
        double TolPar = Precision.PConfusion();
        U11 = U1;
        U22 = U2;
        PBisPrec = ValueByInt(U11, ref UC1, ref UC2, ref Dist1);
        PBis = ValueByInt(U22, ref UC1, ref UC2, ref Dist2);

        while ((U22 - U11) > TolPar
               || ((Dist1 < Precision.Infinite() && Dist2 < Precision.Infinite() && !PBis.IsEqual(PBisPrec, TolPnt))))
        {
            PBisPrec = PBis;
            UMid = 0.5 * (U22 + U11);
            PBis = ValueByInt(UMid, ref UC1, ref UC2, ref DistMid);
            if ((Dist1 < Precision.Infinite()) == (DistMid < Precision.Infinite()))
            {
                U11 = UMid;
                Dist1 = DistMid;
            }
            else
            {
                U22 = UMid;
                Dist2 = DistMid;
            }
        }
        PBis = ValueByInt(U11, ref UC1, ref UC2, ref Dist1);
        if (Dist1 < Precision.Infinite())
        {
            UMid = U11;
        }
        else
        {
            UMid = U22;
        }
        return UMid;
    }

    private static bool ProjOnCurve(in gp_Pnt2d P, Geom2d_Curve C, out double theParam)
    {
        theParam = 0.0;
        C.D1(C.FirstParameter(), out gp_Pnt2d PF, out gp_Vec2d TF);
        C.D1(C.LastParameter(), out gp_Pnt2d PL, out gp_Vec2d TL);

        if (P.IsEqual(PF, Precision.Confusion()))
        {
            theParam = C.FirstParameter();
            return true;
        }

        if (P.IsEqual(PL, Precision.Confusion()))
        {
            theParam = C.LastParameter();
            return true;
        }

        gp_Vec2d PPF = new(PF.X() - P.X(), PF.Y() - P.Y());
        TF.Normalize();

        if (Math.Abs(PPF.Dot(TF)) < Precision.Confusion())
        {
            theParam = C.FirstParameter();
            return true;
        }
        gp_Vec2d PPL = new(PL.X() - P.X(), PL.Y() - P.Y());
        TL.Normalize();
        if (Math.Abs(PPL.Dot(TL)) < Precision.Confusion())
        {
            theParam = C.LastParameter();
            return true;
        }
        Geom2dAPI_ProjectPointOnCurve Proj = new(P, C, C.FirstParameter(), C.LastParameter());
        if (Proj.NbPoints() > 0)
        {
            theParam = Proj.LowerDistanceParameter();
        }
        else
        {
            return false;
        }
        return true;
    }

    private static bool TestExtension(Geom2d_Curve C1, Geom2d_Curve C2, int Start_End)
    {
        gp_Pnt2d P1, P2;
        gp_Vec2d T1, T2;
        bool Test = false;
        if (Start_End == 1)
        {
            C1.D1(C1.FirstParameter(), out P1, out T1);
        }
        else
        {
            C1.D1(C1.LastParameter(), out P1, out T1);
        }
        C2.D1(C2.FirstParameter(), out P2, out T2);
        if (P1.IsEqual(P2, Precision.Confusion()))
        {
            T1.Normalize();
            T2.Normalize();
            if (T1.Dot(T2) > 1.0 - Precision.Confusion())
            {
                Test = true;
            }
        }
        else
        {
            C2.D1(C2.LastParameter(), out P2, out T2);
            if (P1.IsEqual(P2, Precision.Confusion()))
            {
                T2.Normalize(); // OCCT: T1 is not normalised on this path (occt-issues.md).
                if (T1.Dot(T2) > 1.0 - Precision.Confusion())
                {
                    Test = true;
                }
            }
        }
        return Test;
    }

    private void ComputePointEnd()
    {
        double U1, U2;
        double KC, RC;
        U1 = curve1.FirstParameter();
        if (sign1 == sign2)
        {
            U2 = curve2.LastParameter();
        }
        else
        {
            U2 = curve2.FirstParameter();
        }
        double K1 = Curvature(curve1, U1, Precision.Confusion());
        double K2 = Curvature(curve2, U2, Precision.Confusion());
        if (!isConvex1 && !isConvex2)
        {
            if (K1 < K2)
            {
                KC = K1;
            }
            else
            {
                KC = K2;
            }
        }
        else if (!isConvex1)
        {
            KC = K1;
        }
        else
        {
            KC = K2;
        }

        curve1.D1(U1, out gp_Pnt2d PF, out gp_Vec2d TF);
        TF.Normalize();
        if (KC != 0.0)
        {
            RC = Math.Abs(1 / KC);
        }
        else
        {
            RC = Precision.Infinite();
        }
        pointEnd.SetCoord(PF.X() - sign1 * RC * TF.Y(), PF.Y() + sign1 * RC * TF.X());
    }

    private static bool DiscretPar(double DU, double EpsMin, double EpsMax, int NbMin, int NbMax, out double Eps,
                                   out int Nb)
    {
        if (DU <= NbMin * EpsMin)
        {
            Eps = DU / (NbMin + 1);
            Nb = NbMin;
            return false;
        }

        Eps = Min(EpsMax, DU / NbMax);

        if (Eps < EpsMin)
        {
            Eps = EpsMin;
            Nb = (int)(DU / EpsMin);
        }
        else
        {
            Nb = NbMax;
        }

        return true;
    }
}
