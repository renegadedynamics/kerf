// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Xunit.Abstractions;
using static Kerf.Geometry.CurveKind;
using static OcctNet.Oracle.Tests.AdaptorOracleInputs;

namespace OcctNet.Oracle.Tests;

/// <summary>
/// ProjLib_ProjectedCurve on planes against native OCCT, on random curves of every kind (lines, conics, Bezier and
/// B-spline curves, periodic or not, rational or not, offset and trimmed curves) and random planes, through:
/// <list type="bullet">
/// <item>GeomAPI::To2d, whose body the port mirrors: ProjLib_ProjectedCurve of the curve on the plane, then
/// Geom2dAdaptor::MakeCurve unless the result is an offset or other curve. The elementary curves go through
/// ProjLib_Plane, Bezier and B-spline curves through ProjLib_ComputeApprox's pole projection, offset curves through
/// its approximation (Approx_FitAndDivide2d).</item>
/// <item>BRep_Tool::CurveOnSurface on an edge having only a 3D curve and a plane (BRep_Tool::CurveOnPlane): the
/// curve, trimmed to the edge's range, is projected along the plane's normal by GeomProjLib::ProjectOnPlane
/// (ProjLib_ProjectOnPlane, keeping the parametrization), then by ProjLib_ProjectedCurve and
/// Geom2dAdaptor::MakeCurve, and a trimmed result is replaced by its basis. The port mirrors CurveOnPlane's
/// body.</item>
/// </list>
/// The 2D curves are compared: type, range, values along the range, and for B-splines degree, poles, weights,
/// knots and multiplicities.
/// </summary>
public class ProjLibProjectedCurveOracleTests(ITestOutputHelper output)
{
    private const int CasesPerKind = 60;

    // Compares two 2D results; theFirst and theLast is the range to evaluate (a bounded range).
    internal static void SamePCurve(Oracle theO, N.Geom2d_Curve? theN, Geom2d_Curve? theP, double theFirst, double theLast,
                                    string theWhat)
    {
        theO.Same(theN is null, theP is null, theWhat + " null");
        if (theN is null || theP is null)
        {
            return;
        }
        theO.Same(true, theN.IsInstance(global::Kerf.Engine.Foundation.OcctTypeNames.Of(theP.GetType())), $"{theWhat} type {Kerf.Engine.Foundation.OcctTypeNames.Of(theP.GetType())}");
        theO.Same(theN.FirstParameter(), theP.FirstParameter(), theWhat + " first");
        theO.Same(theN.LastParameter(), theP.LastParameter(), theWhat + " last");
        theO.Same(theN.IsPeriodic(), theP.IsPeriodic(), theWhat + " periodic");
        for (int i = 0; i <= 10; i++)
        {
            double t = theFirst + (theLast - theFirst) * i / 10;
            using N.gp_Pnt2d aNP = theN.Value(t);
            theO.Same(aNP, theP.Value(t), $"{theWhat} value at {t}");
        }
        if (theP is Geom2d_BSplineCurve aPB)
        {
            using N.Geom2d_BSplineCurve aNB = N.Geom2d_BSplineCurve.DownCast(theN)!;
            theO.Same(aNB.Degree(), aPB.Degree(), theWhat + " degree");
            theO.Same(aNB.NbPoles(), aPB.NbPoles(), theWhat + " poles");
            theO.Same(aNB.NbKnots(), aPB.NbKnots(), theWhat + " knots");
            theO.Same(aNB.IsRational(), aPB.IsRational(), theWhat + " rational");
            for (int i = 1; i <= aPB.NbPoles(); i++)
            {
                using N.gp_Pnt2d aNP = aNB.Pole(i);
                theO.Same(aNP, aPB.Pole(i), $"{theWhat} pole {i}");
                theO.Same(aNB.Weight(i), aPB.Weight(i), $"{theWhat} weight {i}");
            }
            for (int i = 1; i <= aPB.NbKnots(); i++)
            {
                theO.Same(aNB.Knot(i), aPB.Knot(i), $"{theWhat} knot {i}");
                theO.Same(aNB.Multiplicity(i), aPB.Multiplicity(i), $"{theWhat} mult {i}");
            }
        }
    }

    // A bounded range of the curve to evaluate on.
    internal static (double First, double Last) Range(Geom_Curve theC, double theHalfRange)
    {
        double aF = theC.FirstParameter(), aL = theC.LastParameter();
        if (global::Kerf.Engine.Foundation.Precision.IsInfinite(aF) || global::Kerf.Engine.Foundation.Precision.IsInfinite(aL))
        {
            return (-theHalfRange, theHalfRange);
        }
        return (aF, aL);
    }

    // GeomAPI::To2d's body.
    private static Geom2d_Curve? To2d(Geom_Curve C, in Plane3D P)
    {
        Geom2d_Curve? result = null;
        GeomAdaptor_Curve HC = new(C);
        Geom_Plane Plane = new(P);
        GeomAdaptor_Surface HS = new(Plane);

        ProjLib_ProjectedCurve Proj = new(HS, HC);

        if (Proj.GetType() != OffsetCurve && Proj.GetType() != OtherCurve)
        {
            result = Geom2dAdaptor.MakeCurve(Proj);
        }

        return result;
    }

    [Fact]
    public void To2d_AllCurveKinds()
    {
        var anO = new Oracle(output);
        int aThrown = 0;
        for (int aKind = 0; aKind < CurveKinds.Length; aKind++)
        {
            for (int i = 0; i < CasesPerKind; i++)
            {
                string aWhat = $"{CurveKinds[aKind]} {i}";
                (N.Geom_Curve aNC, Geom_Curve aPC, double aRange) = RandomCurve(anO, aKind);
                (N.gp_Ax3 aNAx, CoordinateSystem anAx) = GeomOracleInputs.Ax3s(anO);
                using (aNC)
                using (aNAx)
                using (var aNPln = new N.gp_Pln(aNAx))
                {
                    N.Geom2d_Curve? aNRes = null;
                    Geom2d_Curve? aPRes = null;
                    Exception? aNErr = Catch(() => aNRes = N.GeomAPI.To2d(aNC, aNPln));
                    Exception? aPErr = Catch(() => aPRes = To2d(aPC, new Plane3D(anAx)));
                    using (aNRes)
                    {
                        if (!SameOutcome(aNErr, aPErr, aWhat))
                        {
                            continue;
                        }
                        if (aNErr is not null)
                        {
                            aThrown++;
                            continue;
                        }
                        (double aF, double aL) = Range(aPC, aRange);
                        SamePCurve(anO, aNRes, aPRes, aF, aL, aWhat);
                    }
                }
            }
        }
        output.WriteLine($"{aThrown} cases threw on both sides");
        anO.Report("ProjLib_ProjectedCurve on planes through GeomAPI::To2d");
    }

    // BRep_Tool::CurveOnPlane's body (the edge's location is the identity).
    private static Geom2d_Curve? CurveOnPlane(Geom_Curve C3D, double f, double l, Geom_Plane GP)
    {
        Geom_TrimmedCurve aTrimmedCurve = new(C3D, f, l, true, false);

        // Perform projection
        Geom_Curve? ProjOnPlane = GeomProjLib.ProjectOnPlane(aTrimmedCurve, GP, GP.Position().Direction(), true);

        GeomAdaptor_Surface HS = new(GP);
        GeomAdaptor_Curve HC = new(ProjOnPlane!);

        ProjLib_ProjectedCurve Proj = new(HS, HC);
        Geom2d_Curve pc = Geom2dAdaptor.MakeCurve(Proj);

        if (pc is Geom2d_TrimmedCurve TC)
        {
            pc = TC.BasisCurve();
        }

        return pc;
    }

    [Fact]
    public void CurveOnPlane_AllCurveKinds()
    {
        var anO = new Oracle(output, 20261029);
        int aThrown = 0;
        for (int aKind = 0; aKind < CurveKinds.Length; aKind++)
        {
            for (int i = 0; i < CasesPerKind; i++)
            {
                string aWhat = $"{CurveKinds[aKind]} {i}";
                (N.Geom_Curve aNC, Geom_Curve aPC, double aRange) = RandomCurve(anO, aKind);
                (N.gp_Ax3 aNAx, CoordinateSystem anAx) = GeomOracleInputs.Ax3s(anO);
                // An edge range inside the curve's range (periodic curves: any range up to a period).
                (double aF0, double aL0) = Range(aPC, aRange);
                if (aPC.IsPeriodic())
                {
                    aF0 = anO.Real(-7, 7);
                    aL0 = aF0 + aPC.Period();
                }
                double aF = anO.Real(aF0, aF0 + 0.4 * (aL0 - aF0)), aL = anO.Real(aF0 + 0.6 * (aL0 - aF0), aL0);
                using (aNC)
                using (aNAx)
                using (var aNPlane = new N.Geom_Plane(aNAx))
                using (var aB = new N.BRep_Builder())
                using (var aNE = new N.TopoDS_Edge())
                using (var aLoc = new N.TopLoc_Location())
                {
                    aB.MakeEdge(aNE, aNC, 1.0e-7);
                    aB.Range(aNE, aF, aL);
                    var aPlane = new Geom_Plane(anAx);
                    double aNF = 0.0, aNL = 0.0;
                    N.Geom2d_Curve? aNRes = null;
                    Geom2d_Curve? aPRes = null;
                    Exception? aNErr = Catch(() => aNRes = N.BRep_Tool.CurveOnSurface(aNE, aNPlane, aLoc, ref aNF, ref aNL));
                    Exception? aPErr = Catch(() => aPRes = CurveOnPlane(aPC, aF, aL, aPlane));
                    using (aNRes)
                    {
                        if (!SameOutcome(aNErr, aPErr, aWhat))
                        {
                            continue;
                        }
                        if (aNErr is not null)
                        {
                            aThrown++;
                            continue;
                        }
                        anO.Same(aNF, aF, aWhat + " first");
                        anO.Same(aNL, aL, aWhat + " last");
                        SamePCurve(anO, aNRes, aPRes, aF, aL, aWhat);
                    }
                }
            }
        }
        output.WriteLine($"{aThrown} cases threw on both sides");
        anO.Report("GeomProjLib::ProjectOnPlane and ProjLib_ProjectedCurve through BRep_Tool::CurveOnPlane");
    }
}
