// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ExtremaCurveCurve.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ExtremaCurveCurve.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GeomAPI/GeomAPI_ExtremaCurveCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Computes the extrema of the distance between two 3D curves (through <see cref="Extrema_ExtCC"/>), the nearest
/// pair among them, and with the <c>Total</c> methods the minimal distance including the curves' end points.
/// </summary>
/// <remarks>
/// A working object (decision 21). OCCT's implicit conversions to <c>int</c> and <c>double</c> are explicit
/// conversions (decision 28); OCCT's <c>operator int</c> reads <c>myExtCC.NbExt()</c> directly, so it throws when
/// the computation failed where <see cref="NbExtrema"/> returns 0.
/// </remarks>
public class GeomAPI_ExtremaCurveCurve
{
    private bool myIsDone;
    private int myIndex;
    private readonly Extrema_ExtCC myExtCC = new();
    private readonly GeomAdaptor_Curve myC1 = new();
    private readonly GeomAdaptor_Curve myC2 = new();
    private bool myTotalExt;
    private bool myIsInfinite;
    private double myTotalDist;
    private readonly gp_Pnt[] myTotalPoints = [new gp_Pnt(), new gp_Pnt()];
    private readonly double[] myTotalPars = new double[2];

    /// <summary>An empty algorithm, to be initialised with <c>Init</c>.</summary>
    public GeomAPI_ExtremaCurveCurve()
    {
        myIsDone = false;
        myIndex = 0;
        myTotalExt = false;
        myIsInfinite = false;
        myTotalDist = 0.0;
    }

    /// <summary>The extrema between the whole curves <paramref name="C1"/> and <paramref name="C2"/>.</summary>
    public GeomAPI_ExtremaCurveCurve(Geom_Curve C1, Geom_Curve C2)
    {
        Init(C1, C2);
    }

    /// <summary>The extrema between <paramref name="C1"/> on [U1min, U1max] and <paramref name="C2"/> on [U2min, U2max].</summary>
    public GeomAPI_ExtremaCurveCurve(Geom_Curve C1, Geom_Curve C2, double U1min, double U1max, double U2min, double U2max)
    {
        Init(C1, C2, U1min, U1max, U2min, U2max);
    }

    /// <summary>Computes the extrema between the whole curves <paramref name="C1"/> and <paramref name="C2"/>.</summary>
    public void Init(Geom_Curve C1, Geom_Curve C2)
    {
        myTotalExt = false;

        double Tol = Precision.PConfusion();
        myC1.Load(C1);
        myC2.Load(C2);
        myExtCC.Initialize(myC1, myC2, Tol, Tol);
        myExtCC.Perform();

        myIsDone = myExtCC.IsDone() && (myExtCC.NbExt() > 0);

        if (myIsDone)
        {
            // evaluate the lower distance and its index;
            double Dist2, Dist2Min = myExtCC.SquareDistance(1);
            myIndex = 1;

            for (int i = 2; i <= myExtCC.NbExt(); i++)
            {
                Dist2 = myExtCC.SquareDistance(i);
                if (Dist2 < Dist2Min)
                {
                    Dist2Min = Dist2;
                    myIndex = i;
                }
            }
        }
    }

    /// <summary>Computes the extrema between <paramref name="C1"/> on [U1min, U1max] and <paramref name="C2"/> on [U2min, U2max].</summary>
    public void Init(Geom_Curve C1, Geom_Curve C2, double U1min, double U1max, double U2min, double U2max)
    {
        myTotalExt = false;

        double Tol = Precision.PConfusion();
        // OCCT loads the whole curves, not the ranges: the Total methods then report the curves' natural end
        // parameters (u11, u12, ...) with the ranges' end points, and project onto the whole curves
        myC1.Load(C1);
        myC2.Load(C2);
        myExtCC.Initialize(myC1, myC2, U1min, U1max, U2min, U2max, Tol, Tol);
        myExtCC.Perform();

        myIsDone = myExtCC.IsDone() && (myExtCC.NbExt() > 0);

        if (myIsDone)
        {
            // evaluate the lower distance and its index;
            double Dist2, Dist2Min = myExtCC.SquareDistance(1);
            myIndex = 1;

            for (int i = 2; i <= myExtCC.NbExt(); i++)
            {
                Dist2 = myExtCC.SquareDistance(i);
                if (Dist2 < Dist2Min)
                {
                    Dist2Min = Dist2;
                    myIndex = i;
                }
            }
        }
    }

    /// <summary>The number of extrema (0 when the computation failed).</summary>
    public int NbExtrema()
    {
        if (myIsDone)
        {
            return myExtCC.NbExt();
        }
        else
        {
            return 0;
        }
    }

    /// <summary>OCCT's <c>operator int</c>: the extrema algorithm's NbExt.</summary>
    public static explicit operator int(GeomAPI_ExtremaCurveCurve theExt) => theExt.myExtCC.NbExt();

    /// <summary>The points of the extremum of index <paramref name="Index"/>, on the first and second curve.</summary>
    public void Points(int Index, out gp_Pnt P1, out gp_Pnt P2)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbExtrema(), "GeomAPI_ExtremaCurveCurve::Points");

        myExtCC.Points(Index, out Extrema_POnCurv PC1, out Extrema_POnCurv PC2);

        P1 = PC1.Value();
        P2 = PC2.Value();
    }

    /// <summary>The parameters of the extremum of index <paramref name="Index"/> on the two curves.</summary>
    public void Parameters(int Index, out double U1, out double U2)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbExtrema(), "GeomAPI_ExtremaCurveCurve::Parameters");

        myExtCC.Points(Index, out Extrema_POnCurv PC1, out Extrema_POnCurv PC2);

        U1 = PC1.Parameter();
        U2 = PC2.Parameter();
    }

    /// <summary>The distance of the extremum of index <paramref name="Index"/>.</summary>
    public double Distance(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbExtrema(), "GeomAPI_ExtremaCurveCurve::Distance");

        return Math.Sqrt(myExtCC.SquareDistance(Index));
    }

    /// <summary>True if the curves are parallel (lines or circles).</summary>
    public bool IsParallel() => myExtCC.IsParallel();

    /// <summary>The points of the nearest extremum. Throws StdFail_NotDone if there is none.</summary>
    public void NearestPoints(out gp_Pnt P1, out gp_Pnt P2)
    {
        StdFail_NotDone_Raise_if(!myIsDone, "GeomAPI_ExtremaCurveCurve::NearestPoints");

        Points(myIndex, out P1, out P2);
    }

    /// <summary>The parameters of the nearest extremum. Throws StdFail_NotDone if there is none.</summary>
    public void LowerDistanceParameters(out double U1, out double U2)
    {
        StdFail_NotDone_Raise_if(!myIsDone, "GeomAPI_ExtremaCurveCurve::LowerDistanceParameters");

        Parameters(myIndex, out U1, out U2);
    }

    /// <summary>The distance of the nearest extremum. Throws StdFail_NotDone if there is none.</summary>
    public double LowerDistance()
    {
        StdFail_NotDone_Raise_if(!myIsDone, "GeomAPI_ExtremaCurveCurve::LowerDistance");

        return Math.Sqrt(myExtCC.SquareDistance(myIndex));
    }

    /// <summary>OCCT's <c>operator double</c>: <see cref="LowerDistance"/>.</summary>
    public static explicit operator double(GeomAPI_ExtremaCurveCurve theExt) => theExt.LowerDistance();

    /// <summary>The extrema algorithm, for advanced use.</summary>
    public Extrema_ExtCC Extrema() => myExtCC;

    /// <summary>
    /// The points of the minimal distance between the curves, end points included. False (and the outputs unchanged)
    /// if both curves are infinite and parallel.
    /// </summary>
    public bool TotalNearestPoints(ref gp_Pnt P1, ref gp_Pnt P2)
    {
        if (!myTotalExt)
        {
            TotalPerform();
            myTotalExt = true;
        }

        if (myIsInfinite)
        {
            return false;
        }

        P1 = myTotalPoints[0];
        P2 = myTotalPoints[1];

        return true;
    }

    /// <summary>
    /// The parameters of the minimal distance between the curves, end points included. False (and the outputs
    /// unchanged) if both curves are infinite and parallel.
    /// </summary>
    public bool TotalLowerDistanceParameters(ref double U1, ref double U2)
    {
        if (!myTotalExt)
        {
            TotalPerform();
            myTotalExt = true;
        }

        if (myIsInfinite)
        {
            return false;
        }

        U1 = myTotalPars[0];
        U2 = myTotalPars[1];

        return true;
    }

    /// <summary>The minimal distance between the curves, end points included.</summary>
    public double TotalLowerDistance()
    {
        if (!myTotalExt)
        {
            TotalPerform();
            myTotalExt = true;
        }

        return myTotalDist;
    }

    private void TotalPerform()
    {
        double u11 = myC1.FirstParameter();
        double u12 = myC1.LastParameter();
        double u21 = myC2.FirstParameter();
        double u22 = myC2.LastParameter();

        bool infinite = Precision.IsInfinite(u11) && Precision.IsInfinite(u12) && Precision.IsInfinite(u21)
                        && Precision.IsInfinite(u22);

        myIsInfinite = false;

        if (infinite && myExtCC.IsParallel())
        {
            myIsInfinite = true;

            // calculate distance between any suitable point on C1 and C2
            gp_Pnt PonC1 = myC1.Value(0.0);
            GeomAPI_ProjectPointOnCurve proj = new(PonC1, myC2.Curve()!);
            myTotalDist = proj.LowerDistance();

            return;
        }

        myTotalDist = RealLast();

        if (myIsDone && !myExtCC.IsParallel())
        {
            Points(myIndex, out myTotalPoints[0], out myTotalPoints[1]);
            Parameters(myIndex, out myTotalPars[0], out myTotalPars[1]);
            myTotalDist = Math.Sqrt(myExtCC.SquareDistance(myIndex));

            if (myTotalDist <= Precision.Confusion())
            {
                return;
            }
        }

        myExtCC.TrimmedSquareDistances(out double d11, out double d12, out double d21, out double d22,
                                       out gp_Pnt P11, out gp_Pnt P12, out gp_Pnt P21, out gp_Pnt P22);

        // OCCT compares every end distance with this first total distance, never updated, so a later end pair
        // nearer than it but farther than an earlier one replaces the better pair
        double aTotalDist2 = myTotalDist * myTotalDist;

        if (aTotalDist2 > d11)
        {
            myTotalDist = Math.Sqrt(d11);
            myTotalPoints[0] = P11;
            myTotalPoints[1] = P21;
            myTotalPars[0] = u11;
            myTotalPars[1] = u21;

            if (myTotalDist <= Precision.Confusion())
            {
                return;
            }
        }

        if (aTotalDist2 > d12)
        {
            myTotalDist = Math.Sqrt(d12);
            myTotalPoints[0] = P11;
            myTotalPoints[1] = P22;
            myTotalPars[0] = u11;
            myTotalPars[1] = u22;

            if (myTotalDist <= Precision.Confusion())
            {
                return;
            }
        }

        if (aTotalDist2 > d21)
        {
            myTotalDist = Math.Sqrt(d21);
            myTotalPoints[0] = P12;
            myTotalPoints[1] = P21;
            myTotalPars[0] = u12;
            myTotalPars[1] = u21;

            if (myTotalDist <= Precision.Confusion())
            {
                return;
            }
        }

        if (aTotalDist2 > d22)
        {
            myTotalDist = Math.Sqrt(d22);
            myTotalPoints[0] = P12;
            myTotalPoints[1] = P22;
            myTotalPars[0] = u12;
            myTotalPars[1] = u22;

            if (myTotalDist <= Precision.Confusion())
            {
                return;
            }
        }

        // calculate distances between extremities one curve and other curve
        if (!Precision.IsInfinite(u11))
        {
            GeomAPI_ProjectPointOnCurve proj = new(P11, myC2.Curve()!, u21, u22);

            if (proj.NbPoints() > 0)
            {
                double dmin = proj.LowerDistance();
                if (myTotalDist > dmin)
                {
                    myTotalDist = dmin;
                    myTotalPoints[0] = P11;
                    myTotalPars[0] = u11;
                    myTotalPoints[1] = proj.NearestPoint();
                    myTotalPars[1] = proj.LowerDistanceParameter();

                    if (myTotalDist <= Precision.Confusion())
                    {
                        return;
                    }
                }
            }
        }

        if (!Precision.IsInfinite(u12))
        {
            GeomAPI_ProjectPointOnCurve proj = new(P12, myC2.Curve()!, u21, u22);

            if (proj.NbPoints() > 0)
            {
                double dmin = proj.LowerDistance();
                if (myTotalDist > dmin)
                {
                    myTotalDist = dmin;
                    myTotalPoints[0] = P12;
                    myTotalPars[0] = u12;
                    myTotalPoints[1] = proj.NearestPoint();
                    myTotalPars[1] = proj.LowerDistanceParameter();

                    if (myTotalDist <= Precision.Confusion())
                    {
                        return;
                    }
                }
            }
        }

        if (!Precision.IsInfinite(u21))
        {
            GeomAPI_ProjectPointOnCurve proj = new(P21, myC1.Curve()!, u11, u12);

            if (proj.NbPoints() > 0)
            {
                double dmin = proj.LowerDistance();
                if (myTotalDist > dmin)
                {
                    myTotalDist = dmin;
                    myTotalPoints[0] = proj.NearestPoint();
                    myTotalPars[0] = proj.LowerDistanceParameter();
                    myTotalPoints[1] = P21;
                    myTotalPars[1] = u21;

                    if (myTotalDist <= Precision.Confusion())
                    {
                        return;
                    }
                }
            }
        }

        if (!Precision.IsInfinite(u22))
        {
            GeomAPI_ProjectPointOnCurve proj = new(P22, myC1.Curve()!, u11, u12);

            if (proj.NbPoints() > 0)
            {
                double dmin = proj.LowerDistance();
                if (myTotalDist > dmin)
                {
                    myTotalDist = dmin;
                    myTotalPoints[0] = proj.NearestPoint();
                    myTotalPars[0] = proj.LowerDistanceParameter();
                    myTotalPoints[1] = P22;
                    myTotalPars[1] = u22;
                }
            }
        }
    }
}
