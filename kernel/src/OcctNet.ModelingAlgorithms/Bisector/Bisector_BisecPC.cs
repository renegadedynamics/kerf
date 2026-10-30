// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_BisecPC.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/Bisector/Bisector_BisecPC.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// The bisector of a point and a curve. The curvature of the curve must be monotonic, and the point can't be on
/// the curve except at its ends. The bisector's parameter is the curve's (shifted by a constant on periodic
/// curves); it may be extended at its ends by segments.
/// </summary>
/// <remarks>
/// A geometry class: its mutators (Perform, Transform) are internal and check that the curve isn't frozen. It owns
/// a copy of the curve it was given. Reverse throws Standard_NotImplemented, as in OCCT.
/// </remarks>
public sealed class Bisector_BisecPC : Bisector_Curve
{
    private Geom2d_Curve curve = null!;
    private gp_Pnt2d point;
    private double sign;
    private NCollection_Sequence<double> startIntervals = new();
    private NCollection_Sequence<double> endIntervals = new();
    private int bisInterval;
    private int currentInterval;
    private double shiftParameter;
    private double distMax;
    private bool isEmpty;
    private bool isConvex;
    private bool extensionStart;
    private bool extensionEnd;
    private gp_Pnt2d pointStartBis;
    private gp_Pnt2d pointEndBis;

    /// <summary>An empty bisector.</summary>
    public Bisector_BisecPC()
    {
        sign = 0.0;
        bisInterval = 0;
        currentInterval = 0;
        shiftParameter = 0.0;
        distMax = 0.0;
        isEmpty = true;
        isConvex = false;
        extensionStart = false;
        extensionEnd = false;
    }

    /// <summary>
    /// The bisector of the point P and the curve Cu, on the left of Cu if Side is 1, on its right if -1, limited to
    /// the points closer to Cu than DistMax.
    /// </summary>
    public Bisector_BisecPC(Geom2d_Curve Cu, in gp_Pnt2d P, double Side, double DistMax = 500)
    {
        Perform(Cu, P, Side, DistMax);
    }

    /// <summary>
    /// The bisector of the point P and the curve Cu between the parameters UMin and UMax, where it is supposed
    /// defined throughout. Side is 1 on the left of Cu, -1 on its right.
    /// </summary>
    /// <remarks>
    /// OCCT leaves distMax, isEmpty, currentInterval and shiftParameter uninitialised here (and evaluates the end
    /// points with that shiftParameter); the port's are 0 and false (occt-issues.md).
    /// </remarks>
    public Bisector_BisecPC(Geom2d_Curve Cu, in gp_Pnt2d P, double Side, double UMin, double UMax)
    {
        curve = (Geom2d_Curve)Cu.Copy();
        point = P;
        sign = Side;
        startIntervals.Append(UMin);
        endIntervals.Append(UMax);
        bisInterval = 1;
        extensionStart = false;
        extensionEnd = false;
        pointStartBis = Value(UMin); // OCCT: shiftParameter is uninitialised here.
        pointEndBis = Value(UMax);
        isConvex = Bisector.IsConvex(curve, sign);
    }

    /// <summary>
    /// Computes the bisector of the point P and the curve Cu, on the left of Cu if Side is 1, on its right if -1,
    /// limited to the points closer to Cu than DistMax.
    /// </summary>
    internal void Perform(Geom2d_Curve Cu, in gp_Pnt2d P, double Side, double DistMax = 500)
    {
        CheckMutable();
        curve = (Geom2d_Curve)Cu.Copy();
        point = P;
        distMax = DistMax;
        sign = Side;
        isConvex = Bisector.IsConvex(curve, sign);
        //--------------------------------------------
        // Calculate interval of definition.
        //--------------------------------------------
        ComputeIntervals();
        if (isEmpty)
        {
            return;
        }

        //-------------------------
        // Construction extensions.
        //-------------------------
        bisInterval = 1;
        extensionStart = false;
        extensionEnd = false;
        pointStartBis = Value(startIntervals.First());
        pointEndBis = Value(endIntervals.Last());

        if (!isConvex)
        {
            if (point.IsEqual(curve.Value(curve.FirstParameter()), Precision.Confusion()))
            {
                extensionStart = true;
                double UFirst = startIntervals.First() - P.Distance(pointStartBis);
                startIntervals.InsertBefore(1, UFirst);
                endIntervals.InsertBefore(1, startIntervals.Value(2));
                bisInterval = 2;
            }
            else if (point.IsEqual(curve.Value(curve.LastParameter()), Precision.Confusion()))
            {
                extensionEnd = true;
                double ULast = endIntervals.Last() + P.Distance(pointEndBis);
                startIntervals.Append(endIntervals.Last());
                endIntervals.Append(ULast);
                bisInterval = 1;
            }
        }
    }

    /// <summary>True if the bisector is extended at its start.</summary>
    public override bool IsExtendAtStart() => extensionStart;

    /// <summary>True if the bisector is extended at its end.</summary>
    public override bool IsExtendAtEnd() => extensionEnd;

    /// <summary>Not implemented in OCCT.</summary>
    /// <exception cref="Standard_NotImplemented">Always.</exception>
    protected internal override void Reverse()
    {
        CheckMutable();
        throw new Standard_NotImplemented();
    }

    /// <summary>LastParameter() + FirstParameter() - U.</summary>
    public override double ReversedParameter(double U) => LastParameter() + FirstParameter() - U;

    /// <summary>A new, unfrozen bisector with a copy of the curve.</summary>
    public override Geom2d_Geometry Copy()
    {
        Geom2d_Curve CopyC = (Geom2d_Curve)curve.Copy();
        Bisector_BisecPC C = new();

        C.Init(CopyC, point, sign, startIntervals, endIntervals, bisInterval, currentInterval, shiftParameter, distMax,
               isEmpty, isConvex, extensionStart, extensionEnd, pointStartBis, pointEndBis);
        return C;
    }

    /// <summary>Transforms the curve, the point and the end points.</summary>
    protected internal override void Transform(in gp_Trsf2d T)
    {
        CheckMutable();
        curve.Transform(T);
        point.Transform(T);
        pointStartBis.Transform(T);
        pointEndBis.Transform(T);
    }

    /// <summary>True if the curve is C(N+1).</summary>
    public override bool IsCN(int N) => curve.IsCN(N + 1);

    /// <summary>The start of the first interval.</summary>
    public override double FirstParameter() => startIntervals.First();

    /// <summary>The end of the last interval.</summary>
    public override double LastParameter() => endIntervals.Last();

    /// <summary>The curve's continuity less one.</summary>
    public override GeomAbs_Shape Continuity()
    {
        GeomAbs_Shape Cont = curve.Continuity();
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

    /// <summary>The start of the interval I.</summary>
    public override double IntervalFirst(int I) => startIntervals.Value(I);

    /// <summary>The end of the interval I.</summary>
    public override double IntervalLast(int I) => endIntervals.Value(I);

    /// <summary>The continuity in the intervals: the curve's less one.</summary>
    public GeomAbs_Shape IntervalContinuity()
    {
        GeomAbs_Shape Cont = curve.Continuity();
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

    /// <summary>True if the curve is closed and the bisector has one interval, the curve's.</summary>
    public override bool IsClosed()
    {
        if (curve.IsClosed())
        {
            //-----------------------------------------------------------------------
            // The bisectrice is closed if the curve is closed and the bissectrice
            // has only one domain of continuity equal to the one of the curve.
            // -----------------------------------------------------------------------
            if (startIntervals.First() == curve.FirstParameter() && endIntervals.First() == curve.LastParameter())
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>False.</summary>
    public override bool IsPeriodic() => false;

    //=============================================================================
    //           Extension of the bisector by segments.
    //=============================================================================
    private void Extension(double U, ref gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2, out gp_Vec2d V3)
    {
        double dU;

        V1 = new gp_Vec2d(0.0, 0.0);
        V2 = new gp_Vec2d(0.0, 0.0);
        V3 = new gp_Vec2d(0.0, 0.0);

        if (U < startIntervals.Value(bisInterval))
        {
            if (pointStartBis.IsEqual(point, Precision.PConfusion()))
            {
                P = pointStartBis;
            }
            else
            {
                dU = U - startIntervals.Value(bisInterval);
                gp_Dir2d DirExt = new(pointStartBis.X() - point.X(), pointStartBis.Y() - point.Y());
                P.SetCoord(pointStartBis.X() + dU * DirExt.X(), pointStartBis.Y() + dU * DirExt.Y());
                V1.SetCoord(DirExt.X(), DirExt.Y());
            }
        }
        else if (U > endIntervals.Value(bisInterval))
        {
            if (pointEndBis.IsEqual(point, Precision.PConfusion()))
            {
                P = pointEndBis;
            }
            else
            {
                dU = U - endIntervals.Value(bisInterval);
                gp_Dir2d DirExt = new(point.X() - pointEndBis.X(), point.Y() - pointEndBis.Y());
                P.SetCoord(pointEndBis.X() + dU * DirExt.X(), pointEndBis.Y() + dU * DirExt.Y());
                V1.SetCoord(DirExt.X(), DirExt.Y());
            }
        }
    }

    //=============================================================================
    //           Computation of the current point of parameter U on the
    //           bissectrice. The equation of the bissectrice is:
    //                              || PP(u)||**2
    //           F(u) = P(u) - 1/2* -------------- * N(u)
    //                              (N(u)|PP(u))
    //
    //           N(u) normal to the curve by u.
    //           ( | ) designation of the scalar product.
    //=============================================================================
    private void Values(double U, int N, ref gp_Pnt2d P, ref gp_Vec2d V1, ref gp_Vec2d V2, ref gp_Vec2d V3)
    {
        if (U < startIntervals.Value(bisInterval))
        {
            Extension(U, ref P, out V1, out V2, out V3);
            return;
        }
        else if (U > endIntervals.Value(bisInterval))
        {
            Extension(U, ref P, out V1, out V2, out V3);
            return;
        }
        double UOnCurve = LinkBisCurve(U);

        gp_Vec2d Tu = new(), Tuu = new(), T3u = new();
        gp_Pnt2d PC = new();

        // OCCT: no case for N = 3 (EvalD3): the tangent stays null and the point is the given point (occt-issues.md).
        switch (N)
        {
            case 0:
            {
                curve.D1(UOnCurve, out PC, out Tu);
                break;
            }
            case 1:
            {
                curve.D2(UOnCurve, out PC, out Tu, out Tuu);
                break;
            }
            case 2:
            {
                curve.D3(UOnCurve, out PC, out Tu, out Tuu, out T3u);
                break;
            }
        }

        gp_Vec2d aPPC = new(PC.X() - point.X(), PC.Y() - point.Y());
        gp_Vec2d Nor = new(-Tu.Y(), Tu.X());

        double SquarePPC = aPPC.SquareMagnitude();
        double NorPPC = Nor.Dot(aPPC);
        double A1;

        if (Math.Abs(NorPPC) > gp.Resolution() && (NorPPC * sign) < 0.0)
        {
            A1 = 0.5 * SquarePPC / NorPPC;
            P.SetCoord(PC.X() - Nor.X() * A1, PC.Y() - Nor.Y() * A1);
        }
        else
        {
            return;
        }

        if (N == 0)
        {
            return; // End Calculation Point;
        }

        gp_Vec2d Nu = new(-Tuu.Y(), Tuu.X()); // derivative of the normal by U.
        double NuPPC = Nu.Dot(aPPC);
        double TuPPC = Tu.Dot(aPPC);
        double NorPPCE2 = NorPPC * NorPPC;
        double A2 = TuPPC / NorPPC - 0.5 * NuPPC * SquarePPC / NorPPCE2;

        //--------------------------
        V1 = Tu - A1 * Nu - A2 * Nor;
        //--------------------------
        if (N == 1)
        {
            return; // End calculation D1.
        }

        gp_Vec2d Nuu = new(-T3u.Y(), T3u.X());

        double NorPPCE4 = NorPPCE2 * NorPPCE2;
        double NuuPPC = Nuu.Dot(aPPC);
        double TuuPPC = Tuu.Dot(aPPC);

        double A21 = TuuPPC / NorPPC - TuPPC * NuPPC / NorPPCE2;
        double A22 = (0.5 * NuuPPC * SquarePPC + NuPPC * TuPPC) / NorPPCE2
                     - NuPPC * SquarePPC * NorPPC * NuPPC / NorPPCE4;
        double A2u = A21 - A22;
        //----------------------------------------
        V2 = Tuu - 2 * A2 * Nu - A1 * Nuu - A2u * Nor;
        //----------------------------------------
    }

    //=============================================================================
    //           Square distance from the point of parameter U to the curve and the point:
    //
    //            2             ||PP(u)||**4           2
    //           d =   1/4* ------------------- ||Nor||
    //                         (Nor(u)/PP(u))**2
    //
    //           where Nor is the normal to the curve by U.
    //=============================================================================
    /// <summary>The square of the distance between the point of parameter U and the point (or the curve).</summary>
    public double Distance(double U)
    {
        double UOnCurve = LinkBisCurve(U);

        curve.D1(UOnCurve, out gp_Pnt2d PC, out gp_Vec2d Tan);

        gp_Vec2d aPPC = new(PC.X() - point.X(), PC.Y() - point.Y());
        gp_Vec2d Nor = new(-Tan.Y(), Tan.X());

        double NorNor = Nor.SquareMagnitude();
        double SquareMagPPC = aPPC.SquareMagnitude();
        double Prosca = Nor.Dot(aPPC);

        if (point.IsEqual(PC, Precision.Confusion()))
        {
            if (isConvex)
            {
                return 0.0;
            }
            //----------------------------------------------------
            // the point is on a concave curve.
            // The required point is not the common point.
            // This can avoid the discontinuity of the bisectrice.
            //----------------------------------------------------
            else
            {
                return Precision.Infinite();
            }
        }

        if (Math.Abs(Prosca) < Precision.Confusion() || (Prosca * sign) > 0.0)
        {
            return Precision.Infinite();
        }
        else
        {
            double A = 0.5 * SquareMagPPC / Prosca;
            double Dist = A * A * NorNor;
            // OCCT's curvature test (OCCT_DEBUG_CUR) is left out.
            return Dist;
        }
    }

    /// <summary>The point of parameter U.</summary>
    public override gp_Pnt2d EvalD0(double U)
    {
        gp_Pnt2d P = point;
        gp_Vec2d V1 = new(), V2 = new(), V3 = new();
        Values(U, 0, ref P, ref V1, ref V2, ref V3);
        return P;
    }

    /// <summary>The point and first derivative at U.</summary>
    public override ResD1 EvalD1(double U)
    {
        ResD1 aResult;
        aResult.Point = point;
        aResult.D1 = new gp_Vec2d(0.0, 0.0);
        gp_Vec2d V2 = new(), V3 = new();
        Values(U, 1, ref aResult.Point, ref aResult.D1, ref V2, ref V3);
        return aResult;
    }

    /// <summary>The point and first two derivatives at U.</summary>
    public override ResD2 EvalD2(double U)
    {
        ResD2 aResult;
        aResult.Point = point;
        aResult.D1 = new gp_Vec2d(0.0, 0.0);
        aResult.D2 = new gp_Vec2d(0.0, 0.0);
        gp_Vec2d V3 = new();
        Values(U, 2, ref aResult.Point, ref aResult.D1, ref aResult.D2, ref V3);
        return aResult;
    }

    /// <summary>The point and first three derivatives at U (OCCT's Values doesn't compute the third: it is 0).</summary>
    public override ResD3 EvalD3(double U)
    {
        ResD3 aResult;
        aResult.Point = point;
        aResult.D1 = new gp_Vec2d(0.0, 0.0);
        aResult.D2 = new gp_Vec2d(0.0, 0.0);
        aResult.D3 = new gp_Vec2d(0.0, 0.0);
        Values(U, 3, ref aResult.Point, ref aResult.D1, ref aResult.D2, ref aResult.D3);
        return aResult;
    }

    /// <summary>The derivative of order N (1 to 3) at U.</summary>
    /// <exception cref="Geom2d_UndefinedDerivative">N is not 1, 2 or 3.</exception>
    public override gp_Vec2d EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Geom2d_UndefinedDerivative("Bisector_BisecPC::EvalDN");
        }
        gp_Pnt2d P = point;
        gp_Vec2d V1 = new(0.0, 0.0);
        gp_Vec2d V2 = new(0.0, 0.0);
        gp_Vec2d V3 = new(0.0, 0.0);
        Values(U, N, ref P, ref V1, ref V2, ref V3);
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
                throw new Geom2d_UndefinedDerivative("Bisector_BisecPC::EvalDN");
            }
        }
    }

    //=============================================================================
    //           Search of the limit between the parts where the distance to the curve is
    //           greater and smaller than distMax, by dichotomy.
    //=============================================================================
    private double SearchBound(double U1, double U2)
    {
        double Dist1, DistMid, U11, U22;
        double UMid = 0.0;
        double Tol = Precision.PConfusion();
        double DistMax2 = distMax * distMax;
        U11 = U1;
        U22 = U2;
        Dist1 = Distance(U11);

        while ((U22 - U11) > Tol)
        {
            UMid = 0.5 * (U22 + U11);
            DistMid = Distance(UMid);
            if ((Dist1 > DistMax2) == (DistMid > DistMax2))
            {
                U11 = UMid;
                Dist1 = DistMid;
            }
            else
            {
                U22 = UMid;
            }
        }
        return UMid;
    }

    // OCCT's CuspFilter throws Standard_NotImplemented and is never called; left out.

    //=============================================================================
    //           Computation of the intervals of definition of the bisector.
    //=============================================================================
    private void ComputeIntervals()
    {
        double U1 = 0.0, U2 = 0.0, UProj = 0.0;
        double UStart = 0.0, UEnd = 0.0;
        double Dist1, Dist2, DistProj;
        isEmpty = false;
        shiftParameter = 0.0;
        bool YaProj = false;
        double DistMax2 = distMax * distMax;

        U1 = curve.FirstParameter();
        U2 = curve.LastParameter();
        Dist1 = Distance(U1);
        Dist2 = Distance(U2);
        DistProj = Precision.Infinite();

        Geom2dAPI_ProjectPointOnCurve Proj = new(point, curve, U1, U2);
        if (Proj.NbPoints() > 0)
        {
            UProj = Proj.LowerDistanceParameter();
            DistProj = Distance(UProj);
            YaProj = true;
        }

        if (Dist1 < DistMax2 && Dist2 < DistMax2)
        {
            if (DistProj > DistMax2 && YaProj)
            {
                isEmpty = true;
            }
            else
            {
                startIntervals.Append(U1);
                endIntervals.Append(U2);
            }
            return;
        }
        else if (Dist1 > DistMax2 && Dist2 > DistMax2)
        {
            if (DistProj < DistMax2)
            {
                UStart = SearchBound(U1, UProj);
                UEnd = SearchBound(UProj, U2);
            }
            else
            {
                isEmpty = true;
                return;
            }
        }
        else if (Dist1 < DistMax2)
        {
            UStart = U1;
            UEnd = SearchBound(U1, U2);
        }
        else if (Dist2 < DistMax2)
        {
            UEnd = U2;
            UStart = SearchBound(U1, U2);
        }
        startIntervals.Append(UStart);
        endIntervals.Append(UEnd);

        //------------------------------------------------------------------------
        // Eventual offset of the parameter on the curve correspondingly to the one
        // on the curve. The offset can be done if the curve is periodical and the
        // point of initial parameter is less then the interval of continuity.
        //------------------------------------------------------------------------
        // OCCT: dead code, one interval was appended (and Period() would throw: the bisector isn't periodic).
        if (curve.IsPeriodic())
        {
            if (startIntervals.Length() > 1)
            { // Plusieurs intervals.
                if (endIntervals.Last() == curve.LastParameter() && startIntervals.First() == curve.FirstParameter())
                {
                    //---------------------------------------------------------------
                    // the bissectrice is defined at the origin.
                    // => Fusion of the first and the last interval.
                    // => 0 on the bisectrice becomes the start of the first interval
                    // => offset of parameter on all limits of intervals.
                    //---------------------------------------------------------------
                    startIntervals.Remove(1);
                    endIntervals.Remove(endIntervals.Length());

                    shiftParameter = Period() - startIntervals.First();
                    for (int k = 1; k <= startIntervals.Length(); k++)
                    {
                        endIntervals.ChangeValue(k) += shiftParameter;
                        startIntervals.ChangeValue(k) += shiftParameter;
                    }
                    startIntervals.ChangeValue(1) = 0.0;
                }
            }
        }
    }

    /// <summary>The parameter on the curve of the projection of the point of parameter U on the bisector.</summary>
    public double LinkBisCurve(double U) => (U - shiftParameter);

    /// <summary>The inverse of LinkBisCurve.</summary>
    public double LinkCurveBis(double U) => (U + shiftParameter);

    /// <summary>True if the bisector is empty.</summary>
    public bool IsEmpty() => isEmpty;

    /// <summary>The parameter on the bisector of the point P.</summary>
    public override double Parameter(in gp_Pnt2d P)
    {
        double Tol = Precision.Confusion();

        if (P.IsEqual(pointStartBis, Tol))
        {
            return startIntervals.Value(bisInterval);
        }
        if (P.IsEqual(pointEndBis, Tol))
        {
            return endIntervals.Value(bisInterval);
        }

        if (extensionStart)
        {
            gp_Ax2d Axe = new(pointStartBis, new gp_Dir2d(pointStartBis.X() - P.X(), pointStartBis.Y() - P.Y()));
            double U = ElCLib.LineParameter(Axe, P);
            gp_Pnt2d Proj = ElCLib.LineValue(U, Axe);
            if (Proj.IsEqual(P, Tol) && U < 0.0)
            {
                return U + startIntervals.Value(bisInterval);
            }
        }
        if (extensionEnd)
        {
            gp_Ax2d Axe = new(pointEndBis, new gp_Dir2d(P.X() - pointEndBis.X(), P.Y() - pointEndBis.Y()));
            double U = ElCLib.LineParameter(Axe, P);
            gp_Pnt2d Proj = ElCLib.LineValue(U, Axe);
            if (Proj.IsEqual(P, Tol) && U > 0.0)
            {
                return U + endIntervals.Value(bisInterval);
            }
        }
        double UOnCurve = 0.0;
        Geom2dAPI_ProjectPointOnCurve Proj2 = new(P, curve, curve.FirstParameter(), curve.LastParameter());
        if (Proj2.NbPoints() > 0)
        {
            UOnCurve = Proj2.LowerDistanceParameter();
        }
        return LinkCurveBis(UOnCurve);
    }

    private void Init(Geom2d_Curve Curve, in gp_Pnt2d Point, double Sign, NCollection_Sequence<double> StartIntervals,
                      NCollection_Sequence<double> EndIntervals, int BisInterval, int CurrentInterval,
                      double ShiftParameter, double DistMax, bool IsEmpty, bool IsConvex, bool ExtensionStart,
                      bool ExtensionEnd, in gp_Pnt2d PointStartBis, in gp_Pnt2d PointEndBis)
    {
        curve = Curve;
        point = Point;
        sign = Sign;
        startIntervals = new NCollection_Sequence<double>(StartIntervals);
        endIntervals = new NCollection_Sequence<double>(EndIntervals);
        bisInterval = BisInterval;
        currentInterval = CurrentInterval;
        shiftParameter = ShiftParameter;
        distMax = DistMax;
        isEmpty = IsEmpty;
        isConvex = IsConvex;
        extensionStart = ExtensionStart;
        extensionEnd = ExtensionEnd;
        pointStartBis = PointStartBis;
        pointEndBis = PointEndBis;
    }

    // OCCT's Dump (std::cout) is left out.
}
