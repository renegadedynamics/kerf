// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_PWalking.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_PWalking.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_PWalking.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.IntImp_ConstIsoparametric;
using static Occt.IntWalk_StatusDeflection;

namespace Occt;

/// <summary>
/// The walking of a parametric/parametric intersection: from a start point (u1, v1, u2, v2), the polyline of
/// points common to two surfaces, marched in both directions with an adaptive step (IntWalk_TheInt2S at each step)
/// until the line leaves the surfaces' domains, closes, or stops on a tangent point.
/// </summary>
/// <remarks>
/// <para>
/// OCCT's class, a working object: <see cref="Perform(NCollection_Array1{double})"/> computes the line, which
/// <see cref="Line"/> returns (the line itself, not a copy; IntPatch_PrmPrmIntersection edits it). OCCT's protected
/// members are internal (IntPatch and GeomInt call none of them from outside). The step arrays (<c>pasuv</c>,
/// <c>pasInit</c>, ...) are arrays allocated with the object. OCCT_DEBUG blocks and the <c>REGLAGE</c> dump of
/// <see cref="AddAPoint"/> are left out.
/// </para>
/// <para>
/// <see cref="StepTrace"/> is a test hook (decision 687): when set, it is called each time
/// <see cref="TestDeflection"/> returns, with the status returned and the point of the step being tested, so that
/// a walk can be compared step by step with a native recording. It changes nothing in the computation.
/// </para>
/// </remarks>
public partial class IntWalk_PWalking
{
    private bool done;
    private IntSurf_LineOn2S line = new();
    private bool close;
    private bool tgfirst;
    private bool tglast;
    private int myTangentIdx;
    private gp_Dir tgdir = new();
    private readonly double fleche;
    private double pasMax;
    private readonly double tolconf;
    private readonly double myTolTang;
    private readonly double[] pasuv = new double[4];
    private readonly double[] myStepMin = new double[4];
    private readonly double[] pasSav = new double[4];
    private readonly double[] pasInit = new double[4];
    private double Um1;
    private double UM1;
    private double Vm1;
    private double VM1;
    private double Um2;
    private double UM2;
    private double Vm2;
    private double VM2;
    private double ResoU1;
    private double ResoU2;
    private double ResoV1;
    private double ResoV2;
    private int sensCheminement;
    private IntImp_ConstIsoparametric choixIsoSav;
    private IntSurf_PntOn2S previousPoint = new();
    private bool previoustg;
    private gp_Dir previousd = new();
    private gp_Dir2d previousd1 = new();
    private gp_Dir2d previousd2 = new();
    private gp_Dir2d firstd1 = new();
    private gp_Dir2d firstd2 = new();
    private readonly IntWalk_TheInt2S myIntersectionOn2S;
    private int STATIC_BLOCAGE_SUR_PAS_TROP_GRAND;
    private int STATIC_PRECEDENT_INFLEXION;

    /// <summary>
    /// Test hook (decision 687): called each time <see cref="TestDeflection"/> returns, with its status and the
    /// point being tested (<c>myIntersectionOn2S.Point()</c>).
    /// </summary>
    internal Action<IntWalk_StatusDeflection, IntSurf_PntOn2S>? StepTrace;

    /// <summary>
    /// The same hook for the walkings that other algorithms create (IntPatch_PrmPrmIntersection's), on the current
    /// thread only; <see cref="StepTrace"/> takes precedence.
    /// </summary>
    [ThreadStatic]
    internal static Action<IntWalk_StatusDeflection, IntSurf_PntOn2S>? ThreadStepTrace;

    /// <summary>
    /// Prepares the walking of <paramref name="Caro1"/> and <paramref name="Caro2"/>. TolTangency is the tolerance of
    /// the tangency test, Epsilon the 3D tolerance of the deflection test, Deflection the largest deflection between
    /// consecutive points, Increment the largest step, relative to the parametric ranges.
    /// </summary>
    public IntWalk_PWalking(Adaptor3d_Surface Caro1, Adaptor3d_Surface Caro2, double TolTangency, double Epsilon,
                            double Deflection, double Increment)
    {
        done = true;
        close = false;
        tgfirst = false;
        tglast = false;
        myTangentIdx = 0;
        fleche = Deflection;
        pasMax = 0.0;
        tolconf = Epsilon;
        myTolTang = TolTangency;
        sensCheminement = 1;
        previoustg = false;
        myIntersectionOn2S = new IntWalk_TheInt2S(Caro1, Caro2, TolTangency);
        STATIC_BLOCAGE_SUR_PAS_TROP_GRAND = 0;
        STATIC_PRECEDENT_INFLEXION = 0;

        double KELARG = 20.0;
        //
        pasMax = Increment * 0.2; //-- June 25 99 after problems with precision
        Um1 = Adaptor3d_HSurfaceTool.FirstUParameter(Caro1);
        Vm1 = Adaptor3d_HSurfaceTool.FirstVParameter(Caro1);
        UM1 = Adaptor3d_HSurfaceTool.LastUParameter(Caro1);
        VM1 = Adaptor3d_HSurfaceTool.LastVParameter(Caro1);

        Um2 = Adaptor3d_HSurfaceTool.FirstUParameter(Caro2);
        Vm2 = Adaptor3d_HSurfaceTool.FirstVParameter(Caro2);
        UM2 = Adaptor3d_HSurfaceTool.LastUParameter(Caro2);
        VM2 = Adaptor3d_HSurfaceTool.LastVParameter(Caro2);

        ResoU1 = Adaptor3d_HSurfaceTool.UResolution(Caro1, Precision.Confusion());
        ResoV1 = Adaptor3d_HSurfaceTool.VResolution(Caro1, Precision.Confusion());

        ResoU2 = Adaptor3d_HSurfaceTool.UResolution(Caro2, Precision.Confusion());
        ResoV2 = Adaptor3d_HSurfaceTool.VResolution(Caro2, Precision.Confusion());

        double NEWRESO;
        double MAXVAL;
        double MAXVAL2;
        //
        MAXVAL = Math.Abs(Um1);
        MAXVAL2 = Math.Abs(UM1);
        if (MAXVAL2 > MAXVAL)
        {
            MAXVAL = MAXVAL2;
        }
        NEWRESO = ResoU1 * MAXVAL;
        if (NEWRESO > ResoU1 && NEWRESO < 10)
        {
            ResoU1 = NEWRESO;
        }

        MAXVAL = Math.Abs(Um2);
        MAXVAL2 = Math.Abs(UM2);
        if (MAXVAL2 > MAXVAL)
        {
            MAXVAL = MAXVAL2;
        }
        NEWRESO = ResoU2 * MAXVAL;
        if (NEWRESO > ResoU2 && NEWRESO < 10)
        {
            ResoU2 = NEWRESO;
        }

        MAXVAL = Math.Abs(Vm1);
        MAXVAL2 = Math.Abs(VM1);
        if (MAXVAL2 > MAXVAL)
        {
            MAXVAL = MAXVAL2;
        }
        NEWRESO = ResoV1 * MAXVAL;
        if (NEWRESO > ResoV1 && NEWRESO < 10)
        {
            ResoV1 = NEWRESO;
        }

        MAXVAL = Math.Abs(Vm2);
        MAXVAL2 = Math.Abs(VM2);
        if (MAXVAL2 > MAXVAL)
        {
            MAXVAL = MAXVAL2;
        }
        NEWRESO = ResoV2 * MAXVAL;
        if (NEWRESO > ResoV2 && NEWRESO < 10)
        {
            ResoV2 = NEWRESO;
        }

        pasuv[0] = pasMax * Math.Abs(UM1 - Um1);
        pasuv[1] = pasMax * Math.Abs(VM1 - Vm1);
        pasuv[2] = pasMax * Math.Abs(UM2 - Um2);
        pasuv[3] = pasMax * Math.Abs(VM2 - Vm2);

        if (ResoU1 > 0.0001 * pasuv[0])
        {
            ResoU1 = 0.00001 * pasuv[0];
        }
        if (ResoV1 > 0.0001 * pasuv[1])
        {
            ResoV1 = 0.00001 * pasuv[1];
        }
        if (ResoU2 > 0.0001 * pasuv[2])
        {
            ResoU2 = 0.00001 * pasuv[2];
        }
        if (ResoV2 > 0.0001 * pasuv[3])
        {
            ResoV2 = 0.00001 * pasuv[3];
        }

        if (!Adaptor3d_HSurfaceTool.IsUPeriodic(Caro1))
        {
            // UM1+=KELARG*pasuv[0];  Um1-=KELARG*pasuv[0];
        }
        else
        {
            double t = UM1 - Um1;
            if (t < Adaptor3d_HSurfaceTool.UPeriod(Caro1))
            {
                t = 0.5 * (Adaptor3d_HSurfaceTool.UPeriod(Caro1) - t);
                t = (t > KELARG * pasuv[0]) ? KELARG * pasuv[0] : t;
                UM1 += t;
                Um1 -= t;
            }
        }

        if (!Adaptor3d_HSurfaceTool.IsVPeriodic(Caro1))
        {
            // VM1+=KELARG*pasuv[1];  Vm1-=KELARG*pasuv[1];
        }
        else
        {
            double t = VM1 - Vm1;
            if (t < Adaptor3d_HSurfaceTool.VPeriod(Caro1))
            {
                t = 0.5 * (Adaptor3d_HSurfaceTool.VPeriod(Caro1) - t);
                t = (t > KELARG * pasuv[1]) ? KELARG * pasuv[1] : t;
                VM1 += t;
                Vm1 -= t;
            }
        }

        if (!Adaptor3d_HSurfaceTool.IsUPeriodic(Caro2))
        {
            // UM2+=KELARG*pasuv[2];  Um2-=KELARG*pasuv[2];
        }
        else
        {
            double t = UM2 - Um2;
            if (t < Adaptor3d_HSurfaceTool.UPeriod(Caro2))
            {
                t = 0.5 * (Adaptor3d_HSurfaceTool.UPeriod(Caro2) - t);
                t = (t > KELARG * pasuv[2]) ? KELARG * pasuv[2] : t;
                UM2 += t;
                Um2 -= t;
            }
        }

        if (!Adaptor3d_HSurfaceTool.IsVPeriodic(Caro2))
        {
            // VM2+=KELARG*pasuv[3];  Vm2-=KELARG*pasuv[3];
        }
        else
        {
            double t = VM2 - Vm2;
            if (t < Adaptor3d_HSurfaceTool.VPeriod(Caro2))
            {
                t = 0.5 * (Adaptor3d_HSurfaceTool.VPeriod(Caro2) - t);
                t = (t > KELARG * pasuv[3]) ? KELARG * pasuv[3] : t;
                VM2 += t;
                Vm2 -= t;
            }
        }

        myStepMin[0] = 100.0 * ResoU1;
        myStepMin[1] = 100.0 * ResoV1;
        myStepMin[2] = 100.0 * ResoU2;
        myStepMin[3] = 100.0 * ResoV2;

        //-- ComputePasInit(pasuv,Um1,UM1,Vm1,VM1,Um2,UM2,Vm2,VM2,Caro1,Caro2);

        for (int i = 0; i <= 3; i++)
        {
            if (pasuv[i] > 10)
            {
                pasuv[i] = 10;
            }
            pasInit[i] = pasSav[i] = pasuv[i];
        }
    }

    /// <summary>
    /// Prepares the walking as the other constructor, with the domains enlarged, and walks from (U1, V1, U2, V2).
    /// </summary>
    /// <remarks>
    /// OCCT leaves <c>tgfirst</c>, <c>tglast</c>, <c>myTangentIdx</c> and <c>previoustg</c> uninitialised here; Perform
    /// sets them before reading them, and the port's fields start false and 0.
    /// </remarks>
    public IntWalk_PWalking(Adaptor3d_Surface Caro1, Adaptor3d_Surface Caro2, double TolTangency, double Epsilon,
                            double Deflection, double Increment, double U1, double V1, double U2, double V2)
    {
        done = true;
        close = false;
        fleche = Deflection;
        tolconf = Epsilon;
        myTolTang = TolTangency;
        sensCheminement = 1;
        myIntersectionOn2S = new IntWalk_TheInt2S(Caro1, Caro2, TolTangency);
        STATIC_BLOCAGE_SUR_PAS_TROP_GRAND = 0;
        STATIC_PRECEDENT_INFLEXION = 0;

        double KELARG = 20.0;
        //
        pasMax = Increment * 0.2; //-- June 25 99 after problems with precision
        //
        Um1 = Adaptor3d_HSurfaceTool.FirstUParameter(Caro1);
        Vm1 = Adaptor3d_HSurfaceTool.FirstVParameter(Caro1);
        UM1 = Adaptor3d_HSurfaceTool.LastUParameter(Caro1);
        VM1 = Adaptor3d_HSurfaceTool.LastVParameter(Caro1);

        Um2 = Adaptor3d_HSurfaceTool.FirstUParameter(Caro2);
        Vm2 = Adaptor3d_HSurfaceTool.FirstVParameter(Caro2);
        UM2 = Adaptor3d_HSurfaceTool.LastUParameter(Caro2);
        VM2 = Adaptor3d_HSurfaceTool.LastVParameter(Caro2);

        ResoU1 = Adaptor3d_HSurfaceTool.UResolution(Caro1, Precision.Confusion());
        ResoV1 = Adaptor3d_HSurfaceTool.VResolution(Caro1, Precision.Confusion());

        ResoU2 = Adaptor3d_HSurfaceTool.UResolution(Caro2, Precision.Confusion());
        ResoV2 = Adaptor3d_HSurfaceTool.VResolution(Caro2, Precision.Confusion());
        //
        double NEWRESO, MAXVAL, MAXVAL2;
        //
        MAXVAL = Math.Abs(Um1);
        MAXVAL2 = Math.Abs(UM1);
        if (MAXVAL2 > MAXVAL)
        {
            MAXVAL = MAXVAL2;
        }
        NEWRESO = ResoU1 * MAXVAL;
        if (NEWRESO > ResoU1)
        {
            ResoU1 = NEWRESO;
        }
        //
        MAXVAL = Math.Abs(Um2);
        MAXVAL2 = Math.Abs(UM2);
        if (MAXVAL2 > MAXVAL)
        {
            MAXVAL = MAXVAL2;
        }
        NEWRESO = ResoU2 * MAXVAL;
        if (NEWRESO > ResoU2)
        {
            ResoU2 = NEWRESO;
        }
        //
        MAXVAL = Math.Abs(Vm1);
        MAXVAL2 = Math.Abs(VM1);
        if (MAXVAL2 > MAXVAL)
        {
            MAXVAL = MAXVAL2;
        }
        NEWRESO = ResoV1 * MAXVAL;
        if (NEWRESO > ResoV1)
        {
            ResoV1 = NEWRESO;
        }
        //
        MAXVAL = Math.Abs(Vm2);
        MAXVAL2 = Math.Abs(VM2);
        if (MAXVAL2 > MAXVAL)
        {
            MAXVAL = MAXVAL2;
        }
        NEWRESO = ResoV2 * MAXVAL;
        if (NEWRESO > ResoV2)
        {
            ResoV2 = NEWRESO;
        }
        //
        pasuv[0] = pasMax * Math.Abs(UM1 - Um1);
        pasuv[1] = pasMax * Math.Abs(VM1 - Vm1);
        pasuv[2] = pasMax * Math.Abs(UM2 - Um2);
        pasuv[3] = pasMax * Math.Abs(VM2 - Vm2);
        //
        if (!Adaptor3d_HSurfaceTool.IsUPeriodic(Caro1))
        {
            UM1 += KELARG * pasuv[0];
            Um1 -= KELARG * pasuv[0];
        }
        else
        {
            double t = UM1 - Um1;
            if (t < Adaptor3d_HSurfaceTool.UPeriod(Caro1))
            {
                t = 0.5 * (Adaptor3d_HSurfaceTool.UPeriod(Caro1) - t);
                t = (t > KELARG * pasuv[0]) ? KELARG * pasuv[0] : t;
                UM1 += t;
                Um1 -= t;
            }
        }
        //
        if (!Adaptor3d_HSurfaceTool.IsVPeriodic(Caro1))
        {
            VM1 += KELARG * pasuv[1];
            Vm1 -= KELARG * pasuv[1];
        }
        else
        {
            double t = VM1 - Vm1;
            if (t < Adaptor3d_HSurfaceTool.VPeriod(Caro1))
            {
                t = 0.5 * (Adaptor3d_HSurfaceTool.VPeriod(Caro1) - t);
                t = (t > KELARG * pasuv[1]) ? KELARG * pasuv[1] : t;
                VM1 += t;
                Vm1 -= t;
            }
        }
        //
        if (!Adaptor3d_HSurfaceTool.IsUPeriodic(Caro2))
        {
            UM2 += KELARG * pasuv[2];
            Um2 -= KELARG * pasuv[2];
        }
        else
        {
            double t = UM2 - Um2;
            if (t < Adaptor3d_HSurfaceTool.UPeriod(Caro2))
            {
                t = 0.5 * (Adaptor3d_HSurfaceTool.UPeriod(Caro2) - t);
                t = (t > KELARG * pasuv[2]) ? KELARG * pasuv[2] : t;
                UM2 += t;
                Um2 -= t;
            }
        }

        if (!Adaptor3d_HSurfaceTool.IsVPeriodic(Caro2))
        {
            VM2 += KELARG * pasuv[3];
            Vm2 -= KELARG * pasuv[3];
        }
        else
        {
            double t = VM2 - Vm2;
            if (t < Adaptor3d_HSurfaceTool.VPeriod(Caro2))
            {
                t = 0.5 * (Adaptor3d_HSurfaceTool.VPeriod(Caro2) - t);
                t = (t > KELARG * pasuv[3]) ? KELARG * pasuv[3] : t;
                VM2 += t;
                Vm2 -= t;
            }
        }
        //-- ComputePasInit(pasuv,Um1,UM1,Vm1,VM1,Um2,UM2,Vm2,VM2,Caro1,Caro2);

        for (int i = 0; i <= 3; i++)
        {
            pasInit[i] = pasSav[i] = pasuv[i];
        }

        if (ResoU1 > 0.0001 * pasuv[0])
        {
            ResoU1 = 0.00001 * pasuv[0];
        }
        if (ResoV1 > 0.0001 * pasuv[1])
        {
            ResoV1 = 0.00001 * pasuv[1];
        }
        if (ResoU2 > 0.0001 * pasuv[2])
        {
            ResoU2 = 0.00001 * pasuv[2];
        }
        if (ResoV2 > 0.0001 * pasuv[3])
        {
            ResoV2 = 0.00001 * pasuv[3];
        }

        myStepMin[0] = 100.0 * ResoU1;
        myStepMin[1] = 100.0 * ResoV1;
        myStepMin[2] = 100.0 * ResoU2;
        myStepMin[3] = 100.0 * ResoV2;

        //
        NCollection_Array1<double> Par = new(1, 4);
        Par[1] = U1;
        Par[2] = V1;
        Par[3] = U2;
        Par[4] = V2;
        Perform(Par);
    }

    // =====================================================================================================
    // The accessors (IntWalk_PWalking.lxx)
    // =====================================================================================================

    /// <summary>True if the walking succeeded.</summary>
    public bool IsDone() => done;

    /// <summary>The number of points of the line. Throws StdFail_NotDone if not done.</summary>
    public int NbPoints()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return line.NbPoints();
    }

    /// <summary>The point <paramref name="Index"/> of the line. Throws StdFail_NotDone if not done.</summary>
    public IntSurf_PntOn2S Value(int Index)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return line.Value(Index);
    }

    /// <summary>The line (the walking's own, not a copy). Throws StdFail_NotDone if not done.</summary>
    public IntSurf_LineOn2S Line()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return line;
    }

    public bool TangentAtFirst()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return tgfirst;
    }

    public bool TangentAtLast()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return tglast;
    }

    public bool IsClosed()
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        return close;
    }

    /// <summary>The tangent of the line at the point <paramref name="theIndex"/>.</summary>
    public gp_Dir TangentAtLine(out int theIndex)
    {
        if (!done)
        {
            throw new StdFail_NotDone();
        }
        theIndex = myTangentIdx;
        return tgdir;
    }

    /// <summary>Appends a point to the line.</summary>
    public void AddAPoint(in IntSurf_PntOn2S POn2S)
    {
        line.Add(POn2S);
        myTangentIdx = Math.Max(myTangentIdx, 1);
    }

    /// <summary>Removes the point <paramref name="theIndex"/> (or the last one if beyond) from the line.</summary>
    public void RemoveAPoint(int theIndex)
    {
        int anIdx = Math.Min(theIndex, line.NbPoints());
        if (anIdx < 1)
        {
            return;
        }
        if (anIdx <= myTangentIdx)
        {
            myTangentIdx--;
            if (myTangentIdx < 1)
            {
                myTangentIdx = 1;
            }
        }
        line.RemovePoint(anIdx);
    }

    /// <summary>The largest step along the parameter <paramref name="theIndex"/> (0 to 3: u1, v1, u2, v2).</summary>
    public double MaxStep(int theIndex)
    {
        Standard_OutOfRange_Raise_if((theIndex < 0) || (theIndex > 3),
                                     "IntWalk_PWalking::MaxStep() - index is out of range");
        return pasInit[theIndex];
    }

    // =====================================================================================================
    // IntWalk_PWalking.cxx
    // =====================================================================================================

    /// <summary>Estimates the largest steps, to avoid abrupt changes when the iso changes.</summary>
    internal void ComputePasInit(double theDeltaU1, double theDeltaV1, double theDeltaU2, double theDeltaV2)
    {
        const double aRangePart = 0.01;
        double Increment = 2.0 * pasMax;
        Adaptor3d_Surface Caro1 = myIntersectionOn2S.Function().AuxillarSurface1();
        Adaptor3d_Surface Caro2 = myIntersectionOn2S.Function().AuxillarSurface2();

        double aDeltaU1 = Math.Abs(UM1 - Um1);
        double aDeltaV1 = Math.Abs(VM1 - Vm1);
        double aDeltaU2 = Math.Abs(UM2 - Um2);
        double aDeltaV2 = Math.Abs(VM2 - Vm2);

        //-- limit the reduction of uv box estimate to 0.01 natural box
        //--  theDeltaU1 : On box of Inter
        //-- aDeltaU1 : On parametric space
        if (!Precision.IsInfinite(aDeltaU1))
        {
            pasuv[0] = Math.Max(Increment * Math.Max(theDeltaU1, aRangePart * aDeltaU1), pasuv[0]);
        }
        else
        {
            pasuv[0] = Math.Max(Increment * theDeltaU1, pasuv[0]);
        }

        if (!Precision.IsInfinite(aDeltaV1))
        {
            pasuv[1] = Math.Max(Increment * Math.Max(theDeltaV1, aRangePart * aDeltaV1), pasuv[1]);
        }
        else
        {
            pasuv[1] = Math.Max(Increment * theDeltaV1, pasuv[1]);
        }

        if (!Precision.IsInfinite(aDeltaU2))
        {
            pasuv[2] = Math.Max(Increment * Math.Max(theDeltaU2, aRangePart * aDeltaU2), pasuv[2]);
        }
        else
        {
            pasuv[2] = Math.Max(Increment * theDeltaU2, pasuv[2]);
        }

        if (!Precision.IsInfinite(aDeltaV2))
        {
            pasuv[3] = Math.Max(Increment * Math.Max(theDeltaV2, aRangePart * aDeltaV2), pasuv[3]);
        }
        else
        {
            pasuv[3] = Math.Max(Increment * theDeltaV2, pasuv[3]);
        }

        double ResoU1tol = Adaptor3d_HSurfaceTool.UResolution(Caro1, tolconf);
        double ResoV1tol = Adaptor3d_HSurfaceTool.VResolution(Caro1, tolconf);
        double ResoU2tol = Adaptor3d_HSurfaceTool.UResolution(Caro2, tolconf);
        double ResoV2tol = Adaptor3d_HSurfaceTool.VResolution(Caro2, tolconf);

        myStepMin[0] = Math.Max(myStepMin[0], 2.0 * ResoU1tol);
        myStepMin[1] = Math.Max(myStepMin[1], 2.0 * ResoV1tol);
        myStepMin[2] = Math.Max(myStepMin[2], 2.0 * ResoU2tol);
        myStepMin[3] = Math.Max(myStepMin[3], 2.0 * ResoV2tol);

        for (int i = 0; i < 4; i++)
        {
            pasuv[i] = Math.Max(myStepMin[i], pasuv[i]);
        }
    }

    /// <summary>
    /// Checks if theLine is parallel to a boundary of the surface given by theCheckSurf1 (oscillations of at most
    /// theToler). Lines too short to tell are classified as parallel.
    /// </summary>
    private static void IsParallel(IntSurf_LineOn2S theLine, bool theCheckSurf1, double theToler,
                                   out bool theIsUparallel, out bool theIsVparallel)
    {
        const int aNbPointsMAX = 23;

        theIsUparallel = theIsVparallel = true;

        int aNbLinePnts = theLine.NbPoints();
        int aNbPoints = aNbLinePnts;
        if (aNbPoints > aNbPointsMAX)
        {
            aNbPoints = aNbPointsMAX;
        }
        else if (aNbPoints < 3)
        {
            // Here we cannot estimate parallelism.
            // Do all same as for small lines
            return;
        }

        double aStep = IntToReal(theLine.NbPoints()) / aNbPoints;
        double aNPoint = 1.0;

        double aUmin = RealLast(), aUmax = RealFirst(), aVmin = RealLast(), aVmax = RealFirst();
        for (int aNum = 1; aNum <= aNbPoints; aNum++, aNPoint += aStep)
        {
            // Fix possible "out of parameter" case.
            if (aNPoint > aNbLinePnts)
            {
                aNPoint = aNbLinePnts;
            }

            double u, v;
            if (theCheckSurf1)
            {
                theLine.Value(RealToInt(aNPoint)).ParametersOnS1(out u, out v);
            }
            else
            {
                theLine.Value(RealToInt(aNPoint)).ParametersOnS2(out u, out v);
            }

            if (u < aUmin)
            {
                aUmin = u;
            }

            if (u > aUmax)
            {
                aUmax = u;
            }

            if (v < aVmin)
            {
                aVmin = v;
            }

            if (v > aVmax)
            {
                aVmax = v;
            }
        }

        theIsVparallel = ((aUmax - aUmin) < theToler);
        theIsUparallel = ((aVmax - aVmin) < theToler);
    }

    /// <summary>
    /// Clamps theNbElem parameters (from theParam[theFirst]) to the borders (from index theFirstBorder); true if one
    /// changed. OCCT passes pointers into the arrays.
    /// </summary>
    private static bool AdjustToDomain(int theNbElem, NCollection_Array1<double> theParam, int theFirst,
                                       double[] theLowBorder, double[] theUppBorder, int theFirstBorder)
    {
        bool aRetVal = false;
        for (int i = 0; i < theNbElem; i++)
        {
            ref double aParam = ref theParam[theFirst + i];
            if ((aParam - theLowBorder[theFirstBorder + i]) < -Precision.PConfusion())
            {
                aParam = theLowBorder[theFirstBorder + i];
                aRetVal = true;
            }

            if ((aParam - theUppBorder[theFirstBorder + i]) > Precision.PConfusion())
            {
                aParam = theUppBorder[theFirstBorder + i];
                aRetVal = true;
            }
        }

        return aRetVal;
    }

    /// <summary>Computes the first point from ParDep, without walking.</summary>
    public bool PerformFirstPoint(NCollection_Array1<double> ParDep, ref IntSurf_PntOn2S FirstPoint)
    {
        sensCheminement = 1;
        close = false;
        //
        int i;
        NCollection_Array1<double> Param = new(1, 4);
        //
        for (i = 1; i <= 4; ++i)
        {
            Param[i] = ParDep[i];
        }
        //-- calculate the first solution point
        math_FunctionSetRoot Rsnld = new(myIntersectionOn2S.Function());
        //
        myIntersectionOn2S.Perform(Param, Rsnld);
        if (!myIntersectionOn2S.IsDone())
        {
            return false;
        }

        if (myIntersectionOn2S.IsEmpty())
        {
            return false;
        }

        FirstPoint = myIntersectionOn2S.Point();
        return true;
    }

    /// <summary>Walks from ParDep (u1, v1, u2, v2) in the surfaces' domains.</summary>
    public void Perform(NCollection_Array1<double> ParDep)
    {
        Perform(ParDep, Um1, Vm1, Um2, Vm2, UM1, VM1, UM2, VM2);
    }

    /// <summary>The square distance between thePnt and theSurf, from (theU0, theV0).</summary>
    private static double SQDistPointSurface(in gp_Pnt thePnt, Adaptor3d_Surface theSurf, double theU0, double theV0)
    {
        Extrema_GenLocateExtPS aExtPS = new(theSurf);
        aExtPS.Perform(thePnt, theU0, theV0);

        if (!aExtPS.IsDone())
        {
            return RealLast();
        }

        return aExtPS.SquareDistance();
    }

    /// <summary>
    /// Additional check that the surfaces are tangent: whether points of each, around the start point, lie on the
    /// other (with the tolerance given).
    /// </summary>
    private static bool IsTangentExtCheck(Adaptor3d_Surface theSurf1, Adaptor3d_Surface theSurf2, double theU10,
                                          double theV10, double theU20, double theV20, double theToler,
                                          double[] theArrStep)
    {
        {
            theSurf1.D1(theU10, theV10, out gp_Pnt aPt, out gp_Vec aDu1, out gp_Vec aDv1);
            theSurf2.D1(theU20, theV20, out aPt, out gp_Vec aDu2, out gp_Vec aDv2);

            gp_Vec aN1 = aDu1.Crossed(aDv1), aN2 = aDu2.Crossed(aDv2);
            double aDP = aN1.Dot(aN2), aSQ1 = aN1.SquareMagnitude(), aSQ2 = aN2.SquareMagnitude();

            if ((aSQ1 < RealSmall()) || (aSQ2 < RealSmall()))
            {
                return true; // Tangent
            }

            if (aDP * aDP < 0.9998 * aSQ1 * aSQ2)
            { // cos(ang N1<->N2) < 0.9999
                return false; // Not tangent
            }
        }

        // For two faces (2^2 = 4)
        double aSQToler = 4.0 * theToler * theToler;
        const int aNbItems = 4;
        ReadOnlySpan<double> aParUS1 = [theU10 + theArrStep[0], theU10 - theArrStep[0], theU10, theU10];
        ReadOnlySpan<double> aParVS1 = [theV10, theV10, theV10 + theArrStep[1], theV10 - theArrStep[1]];
        ReadOnlySpan<double> aParUS2 = [theU20 + theArrStep[2], theU20 - theArrStep[2], theU20, theU20];
        ReadOnlySpan<double> aParVS2 = [theV20, theV20, theV20 + theArrStep[3], theV20 - theArrStep[3]];

        for (int i = 0; i < aNbItems; i++)
        {
            gp_Pnt aP = theSurf1.Value(aParUS1[i], aParVS1[i]);
            double aSqDist = SQDistPointSurface(aP, theSurf2, theU20, theV20);
            if (aSqDist > aSQToler)
            {
                return false;
            }
        }

        for (int i = 0; i < aNbItems; i++)
        {
            gp_Pnt aP = theSurf2.Value(aParUS2[i], aParVS2[i]);
            double aSqDist = SQDistPointSurface(aP, theSurf1, theU10, theV10);
            if (aSqDist > aSQToler)
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>Writes the parameters of a point into Param(1..4).</summary>
    private static void SetParameters(NCollection_Array1<double> Param, in IntSurf_PntOn2S thePoint)
    {
        thePoint.Parameters(out double u1, out double v1, out double u2, out double v2);
        Param[1] = u1;
        Param[2] = v1;
        Param[3] = u2;
        Param[4] = v2;
    }
}
