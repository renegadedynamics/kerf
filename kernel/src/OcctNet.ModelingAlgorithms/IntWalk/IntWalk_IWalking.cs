// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_IWalking.gxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntWalk/IntWalk_IWalking.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Numerics;
using Kerf.Geometry;
using static Occt.IntWalk_StatusDeflection;

namespace Occt;

/// <summary>What IntWalk_IWalking reads from its function, beyond math_FunctionSetWithDerivatives (decision 607).</summary>
/// <typeparam name="TSelf">The function's class.</typeparam>
public interface IIntWalk_IWFunction<TSelf> : IIntStart_SurfFunction<Adaptor3d_Surface>
    where TSelf : IIntWalk_IWFunction<TSelf>
{
    /// <summary>The parametric surface.</summary>
    Adaptor3d_Surface PSurface();

    /// <summary>A copy of <paramref name="theOther"/>: OCCT's copy constructor (decision 686).</summary>
    static abstract TSelf Copy(TSelf theOther);
}

/// <summary>
/// The walking of an implicit/parametric intersection: the polylines on a parametric surface where a function
/// F(u, v) vanishes, from start points on the surface's restrictions (open lines) and interior points (closed lines).
/// </summary>
/// <remarks>
/// OCCT's generic file IntWalk_IWalking.gxx, ported once as a generic class (decisions 607 and 686) over the
/// function (TheIWFunction: IntPatch_TheSurfFunction, later Contap_SurfFunction) and the line it builds (TheIWLine:
/// IntPatch_TheIWLineOfTheIWalking, Contap_TheIWLineOfTheIWalking). The other macros are the same types in both of
/// OCCT's instantiations and are named in the body: the path and interior points (IntSurf_PathPoint,
/// IntSurf_InteriorPoint and their tools), the surface (Adaptor3d_Surface and Adaptor3d_HSurfaceTool).
/// </remarks>
public class IntWalk_IWalking<TheIWFunction, TheIWLine>
    where TheIWFunction : FunctionSetWithDerivatives, IIntWalk_IWFunction<TheIWFunction>
    where TheIWLine : IntWalk_TheIWLine, new()
{
    private bool done;
    private readonly NCollection_Sequence<IntSurf_PathPoint> seqSingle = new();
    private double fleche;
    private double pas;
    private readonly math_Vector tolerance = new(1, 2);
    private double epsilon;
    private bool reversed;
    private readonly NCollection_LinearVector<IntWalk_WalkingData> wd1 = new();
    private NCollection_LinearVector<IntWalk_WalkingData> wd2 = new();
    private readonly NCollection_LinearVector<int> nbMultiplicities = new();
    private BoundRange mySRangeU = new(); // Estimated U-range for section curve
    private BoundRange mySRangeV = new(); // Estimated V-range for section curve
    private double Um;
    private double UM;
    private double Vm;
    private double VM;
    private IntSurf_PntOn2S previousPoint = new();
    private Vector3D previousd3d = new();
    private Direction2D previousd2d = new();
    private readonly NCollection_Sequence<int> seqAjout = new();
    private readonly NCollection_Sequence<int> seqAlone = new();
    private readonly NCollection_DataMap<int, NCollection_List<int>> PointLineLine = new();
    private readonly NCollection_Sequence<TheIWLine> lines = new();
    private readonly bool ToFillHoles;

    /// <summary>
    /// Deflection is the largest deflection admitted between two consecutive points of a line, Step the largest
    /// increment between two consecutive points (in the parametric space, relative to the surface's range), Epsilon
    /// the distance below which two points are confused. theToFillHoles fills possible holes between the lines
    /// (false for IntPatch's walking).
    /// </summary>
    public IntWalk_IWalking(double Epsilon, double Deflection, double Step, bool theToFillHoles = false)
    {
        done = false;
        fleche = Deflection;
        pas = Step;
        epsilon = Epsilon * Epsilon;
        reversed = false;
        Um = 0.0;
        UM = 0.0;
        Vm = 0.0;
        VM = 0.0;
        ToFillHoles = theToFillHoles;
    }

    public void SetTolerance(double Epsilon, double Deflection, double Step)
    {
        fleche = Deflection;
        pas = Step;
        epsilon = Epsilon * Epsilon;
    }

    /// <summary>True if the computation succeeded.</summary>
    public bool IsDone() => done;

    /// <summary>The number of lines. Throws StdFail_NotDone if not done.</summary>
    public int NbLines()
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        return lines.Length();
    }

    /// <summary>The line <paramref name="Index"/>. Throws StdFail_NotDone if not done.</summary>
    public TheIWLine Value(int Index)
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        return lines.Value(Index);
    }

    /// <summary>The number of the start points on which no line starts or ends. Throws StdFail_NotDone if not done.</summary>
    public int NbSinglePnts()
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        return seqSingle.Length();
    }

    /// <summary>The single point <paramref name="Index"/>. Throws StdFail_NotDone if not done.</summary>
    public IntSurf_PathPoint SinglePnt(int Index)
    {
        if (!done)
        {
            throw new NotDoneException();
        }
        return seqSingle.Value(Index);
    }


    // IntWalk_IWalking_1.gxx

    private const double CosRef3D = 0.98; // rule by tests in U4, corresponds to 11.478 d
    private const double CosRef2D = 0.88; // corresponds to 25 d
    private const int MaxDivision = 60; // max number of step divisions because the angle is too great in 2d (U4)

    /// <summary>
    /// Test hook (decision 687): called each time <see cref="TestDeflection"/> returns, with its status and the point
    /// it tested (the function's point and the parameters on the parametric surface), so that a walk can be compared
    /// step by step with a native recording. It changes nothing in the computation.
    /// </summary>
    internal Action<IntWalk_StatusDeflection, IntSurf_PntOn2S>? StepTrace;

    /// <summary>
    /// The same hook for the walkings that other algorithms create (IntPatch_ImpPrmIntersection's), on the current
    /// thread only; <see cref="StepTrace"/> takes precedence.
    /// </summary>
    [ThreadStatic]
    internal static Action<IntWalk_StatusDeflection, IntSurf_PntOn2S>? ThreadStepTrace;

    // OCCT's stack buffers of the stop tests (written by the function, never read).
    private readonly math_Vector myBidF = new(1, 1);
    private readonly Matrix myBidD = new(1, 1, 1, 2);

    /// <summary>
    /// Additional check if the point (theU, theV) is a tangent point. If it is, any direction leads to the next
    /// intersection point, which is difficult to process; the walking then returns an empty line.
    /// </summary>
    private static bool IsTangentExtCheck(TheIWFunction theFunc, double theU, double theV, double theStepU,
                                          double theStepV, double theUinf, double theUsup, double theVinf,
                                          double theVsup)
    {
        double aTol = theFunc.Tolerance();
        const int aNbItems = 4;
        ReadOnlySpan<double> aParU = [Math.Min(theU + theStepU, theUsup), Math.Max(theU - theStepU, theUinf), theU, theU];
        ReadOnlySpan<double> aParV = [theV, theV, Math.Min(theV + theStepV, theVsup), Math.Max(theV - theStepV, theVinf)];

        math_Vector aX = new(1, 2), aVal = new(1, 1);

        for (int i = 0; i < aNbItems; i++)
        {
            aX.Value(1) = aParU[i];
            aX.Value(2) = aParV[i];

            if (!theFunc.Value(aX, aVal))
            {
                continue;
            }

            if (Math.Abs(theFunc.Root()) > aTol)
            {
                return false;
            }
        }

        return true;
    }

    /// <summary>
    /// Clears the containers and puts a dummy item at index 0 of the vectors, so that start points are indexed from 1.
    /// </summary>
    protected void Clear()
    {
        wd1.Clear();
        wd2.Clear();
        IntWalk_WalkingData aDummy;
        aDummy.etat = -10;
        aDummy.ustart = aDummy.vstart = 0.0;
        wd1.Append(aDummy);
        wd2.Append(aDummy);
        nbMultiplicities.Clear();
        nbMultiplicities.Append(-1);

        done = false;
        seqAjout.Clear();
        lines.Clear();
    }

    // ***************************************************************************
    //  etat1=12 not tangent, not passes
    //  etat1=11 tangent, not passes
    //  etat1=2  not tangent, passes
    //  etat1=1  tangent, passes
    //  after a point is processed its state becomes negative.
    // ***************************************************************************
    //  etat2=13  interior start point on closed line
    //  etat2=12  interior start point on open line
    //            (line initially closed -> la line s is open)
    //  after a point is processed (or if it is passed over during
    //  routing) its state becomes negative.
    // ****************************************************************************

    /// <summary>
    /// Searches the lines starting on a point of <paramref name="Pnts1"/> (open lines) or of <paramref name="Pnts2"/>
    /// (closed lines). Each point of a line verifies F(u, v) = 0.
    /// </summary>
    public void Perform(NCollection_Sequence<IntSurf_PathPoint> Pnts1, NCollection_Sequence<IntSurf_InteriorPoint> Pnts2,
                        TheIWFunction Func, Adaptor3d_Surface Caro, bool Reversed = false)
    {
        int I;
        bool Rajout = false;
        int nbPnts1 = Pnts1.Length();
        int nbPnts2 = Pnts2.Length();
        double U, V;

        Clear();
        reversed = Reversed;

        Um = Adaptor3d_HSurfaceTool.FirstUParameter(Caro);
        Vm = Adaptor3d_HSurfaceTool.FirstVParameter(Caro);
        UM = Adaptor3d_HSurfaceTool.LastUParameter(Caro);
        VM = Adaptor3d_HSurfaceTool.LastVParameter(Caro);

        if (UM < Um)
        {
            double utemp = UM;
            UM = Um;
            Um = utemp;
        }
        if (VM < Vm)
        {
            double vtemp = VM;
            VM = Vm;
            Vm = vtemp;
        }

        double aStepU = pas * (UM - Um), aStepV = pas * (VM - Vm);

        // Loading of etat1 and etat2  as well as  ustart and vstart.

        NCollection_Sequence<double> Umult = new();
        NCollection_Sequence<double> Vmult = new();

        int decal = 0;
        wd1.Reserve(nbPnts1 + decal);
        nbMultiplicities.Reserve(nbPnts1 + decal);
        for (I = 1; I <= nbPnts1 + decal; I++)
        {
            IntSurf_PathPoint PathPnt = Pnts1.Value(I - decal);
            IntWalk_WalkingData aWD1;
            aWD1.etat = 1;
            if (!IntSurf_PathPointTool.IsPassingPnt(PathPnt))
            {
                aWD1.etat = 11;
            }
            if (!IntSurf_PathPointTool.IsTangent(PathPnt))
            {
                ++aWD1.etat;
            }

            if (aWD1.etat == 2)
            {
                aWD1.etat = 11;
            }

            IntSurf_PathPointTool.Value2d(PathPnt, out aWD1.ustart, out aWD1.vstart);
            mySRangeU.Add(aWD1.ustart);
            mySRangeV.Add(aWD1.vstart);

            wd1.Append(aWD1);
            int aNbMult = IntSurf_PathPointTool.Multiplicity(PathPnt);
            nbMultiplicities.Append(aNbMult);

            for (int J = 1; J <= aNbMult; J++)
            {
                IntSurf_PathPointTool.Parameters(PathPnt, J, out U, out V);
                Umult.Append(U);
                Vmult.Append(V);
            }
        }

        wd2.Reserve(nbPnts2);
        for (I = 1; I <= nbPnts2; I++)
        {
            IntWalk_WalkingData aWD2;
            aWD2.etat = 1;
            IntSurf_InteriorPoint anIP = Pnts2.Value(I);
            IntSurf_InteriorPointTool.Value2d(anIP, out aWD2.ustart, out aWD2.vstart);
            mySRangeU.Add(aWD2.ustart);
            mySRangeV.Add(aWD2.vstart);

            if (!IsTangentExtCheck(Func, aWD2.ustart, aWD2.vstart, aStepU, aStepV, Um, UM, Vm, VM))
            {
                aWD2.etat = 13;
            }

            wd2.Append(aWD2);
        }

        tolerance[1] = Adaptor3d_HSurfaceTool.UResolution(Caro, global::Kerf.Engine.Foundation.Precision.Confusion());
        tolerance[2] = Adaptor3d_HSurfaceTool.VResolution(Caro, global::Kerf.Engine.Foundation.Precision.Confusion());

        Func.Set(Caro);

        if (mySRangeU.Delta() > Math.Max(tolerance[1], global::Kerf.Engine.Foundation.Precision.PConfusion()))
        {
            mySRangeU.Enlarge(mySRangeU.Delta());
            mySRangeU.Common(new BoundRange(Um, UM));
        }
        else
        {
            mySRangeU = new BoundRange(Um, UM);
        }

        if (mySRangeV.Delta() > Math.Max(tolerance[2], global::Kerf.Engine.Foundation.Precision.PConfusion()))
        {
            mySRangeV.Enlarge(mySRangeV.Delta());
            mySRangeV.Common(new BoundRange(Vm, VM));
        }
        else
        {
            mySRangeV = new BoundRange(Vm, VM);
        }

        // calculation of all open lines
        if (nbPnts1 != 0)
        {
            ComputeOpenLine(Umult, Vmult, Pnts1, Func, ref Rajout);
        }

        // calculation of all closed lines
        if (nbPnts2 != 0)
        {
            ComputeCloseLine(Umult, Vmult, Pnts1, Pnts2, Func, ref Rajout);
        }

        if (ToFillHoles)
        {
            int MaxNbIter = 10, nb_iter = 0;
            while (seqAlone.Length() > 1 && nb_iter < MaxNbIter)
            {
                nb_iter++;
                NCollection_Sequence<IntSurf_InteriorPoint> PntsInHoles = new();
                NCollection_Sequence<int> CopySeqAlone = new(seqAlone);
                FillPntsInHoles(Func, CopySeqAlone, PntsInHoles);
                wd2.Clear();
                IntWalk_WalkingData aDummy;
                aDummy.etat = -10;
                aDummy.ustart = aDummy.vstart = 0.0;
                wd2.Append(aDummy);
                int nbHoles = PntsInHoles.Length();
                wd2.Reserve(nbHoles);
                for (I = 1; I <= nbHoles; I++)
                {
                    IntWalk_WalkingData aWD2;
                    aWD2.etat = 13;
                    IntSurf_InteriorPoint anIP = PntsInHoles.Value(I);
                    IntSurf_InteriorPointTool.Value2d(anIP, out aWD2.ustart, out aWD2.vstart);
                    wd2.Append(aWD2);
                }
                ComputeCloseLine(Umult, Vmult, Pnts1, PntsInHoles, Func, ref Rajout);
            }
        }

        for (I = 1; I <= nbPnts1; I++)
        {
            if (wd1[I].etat > 0)
            {
                seqSingle.Append(Pnts1.Value(I));
            }
        }
        done = true;
    }

    /// <summary>Searches the lines starting on a point of <paramref name="Pnts1"/>.</summary>
    /// <remarks>
    /// As OCCT's, this overload doesn't call <see cref="Clear"/> (occt-issues.md, unit 5C): it keeps the lines of a
    /// previous Perform, and on a new object the vectors lack their dummy item 0, so the walking reads start point
    /// I + 1's data as point I's and, for the last one, past the vectors' end (out of bounds in OCCT; the port throws
    /// Standard_OutOfRange). No OCCT caller uses it (ImpPrm and Contap give interior points).
    /// </remarks>
    public void Perform(NCollection_Sequence<IntSurf_PathPoint> Pnts1, TheIWFunction Func, Adaptor3d_Surface Caro,
                        bool Reversed = false)
    {
        int I;
        bool Rajout = false;
        int nbPnts1 = Pnts1.Length();
        double U, V;

        // OCCT: no Clear() here, unlike the other overload (see the remarks).
        reversed = Reversed;

        // Loading of etat1 as well as ustart1 and vstart1.

        NCollection_Sequence<double> Umult = new();
        NCollection_Sequence<double> Vmult = new();

        wd1.Reserve(nbPnts1);
        for (I = 1; I <= nbPnts1; I++)
        {
            IntSurf_PathPoint PathPnt = Pnts1.Value(I);
            IntWalk_WalkingData aWD1;
            aWD1.etat = 1;
            if (!IntSurf_PathPointTool.IsPassingPnt(PathPnt))
            {
                aWD1.etat = 11;
            }
            if (!IntSurf_PathPointTool.IsTangent(PathPnt))
            {
                ++aWD1.etat;
            }
            IntSurf_PathPointTool.Value2d(PathPnt, out aWD1.ustart, out aWD1.vstart);
            wd1.Append(aWD1);
            int aNbMult = IntSurf_PathPointTool.Multiplicity(PathPnt);
            nbMultiplicities.Append(aNbMult);

            for (int J = 1; J <= aNbMult; J++)
            {
                IntSurf_PathPointTool.Parameters(PathPnt, J, out U, out V);
                Umult.Append(U);
                Vmult.Append(V);
            }
        }

        tolerance[1] = Adaptor3d_HSurfaceTool.UResolution(Caro, global::Kerf.Engine.Foundation.Precision.Confusion());
        tolerance[2] = Adaptor3d_HSurfaceTool.VResolution(Caro, global::Kerf.Engine.Foundation.Precision.Confusion());

        Um = Adaptor3d_HSurfaceTool.FirstUParameter(Caro);
        Vm = Adaptor3d_HSurfaceTool.FirstVParameter(Caro);
        UM = Adaptor3d_HSurfaceTool.LastUParameter(Caro);
        VM = Adaptor3d_HSurfaceTool.LastVParameter(Caro);

        if (UM < Um)
        {
            double utemp = UM;
            UM = Um;
            Um = utemp;
        }
        if (VM < Vm)
        {
            double vtemp = VM;
            VM = Vm;
            Vm = vtemp;
        }

        Func.Set(Caro);

        // calcul de toutes les lignes ouvertes
        if (nbPnts1 != 0)
        {
            ComputeOpenLine(Umult, Vmult, Pnts1, Func, ref Rajout);
        }

        for (I = 1; I <= nbPnts1; I++)
        {
            if (wd1[I].etat > 0)
            {
                seqSingle.Append(Pnts1.Value(I));
            }
        }
        done = true;
    }

    // IntWalk_IWalking_2.gxx

    private static void CutVectorByTolerances(ref Vector2D theVector, math_Vector theTolerance)
    {
        if (Math.Abs(theVector.X()) < theTolerance[1])
        {
            theVector.SetX(0.0);
        }
        if (Math.Abs(theVector.Y()) < theTolerance[2])
        {
            theVector.SetY(0.0);
        }
    }

    /// <summary>
    /// Locates the point (u, v) in the natural domain of the surface and updates (u, v) for the next point. If the
    /// approached point is out of the domain, it is put on the border in the best direction, the step is changed and a
    /// bound blocks one parameter for the next FunctionSetRoot. Returns true if the point was put on the border.
    /// </summary>
    protected bool Cadrage(math_Vector BornInf, math_Vector BornSup, math_Vector UVap, ref double Step, int StepSign)
    {
        // there are always :
        // BorInf(1) <= UVap(1) <= BornSup(1) et BorInf(2) <= UVap(2) <= BornSup(2)
        double Duvx = previousd2d.X();
        double Duvy = previousd2d.Y();

        if (!reversed)
        {
            previousPoint.ParametersOnS2(out UVap[1], out UVap[2]);
        }
        else
        {
            previousPoint.ParametersOnS1(out UVap[1], out UVap[2]);
        }

        double U1 = UVap[1] + Step * Duvx * StepSign;
        double V1 = UVap[2] + Step * Duvy * StepSign;

        bool infu = (U1 <= BornInf[1] + global::Kerf.Engine.Foundation.Precision.PConfusion());
        bool supu = (U1 >= BornSup[1] - global::Kerf.Engine.Foundation.Precision.PConfusion());
        bool infv = (V1 <= BornInf[2] + global::Kerf.Engine.Foundation.Precision.PConfusion());
        bool supv = (V1 >= BornSup[2] - global::Kerf.Engine.Foundation.Precision.PConfusion());

        double theStepU, theStepV;

        if (!infu && !supu && !infv && !supv)
        {
            UVap[1] = U1;
            UVap[2] = V1;
            return false;
        }

        if ((infu || supu) && (infv || supv))
        {
            if (infu)
            { // jag 940616
                if (Duvx != 0.0)
                {
                    theStepU = Math.Abs((BornInf[1] - UVap[1]) / Duvx); // iso U =BornInf(1)
                }
                else
                {
                    theStepU = Step;
                }
            }
            else
            {
                if (Duvx != 0.0)
                {
                    theStepU = Math.Abs((BornSup[1] - UVap[1]) / Duvx); // iso U =BornSup(1)
                }
                else
                {
                    theStepU = Step;
                }
            }
            if (infv)
            { // jag 940616
                if (Duvy != 0.0)
                {
                    theStepV = Math.Abs((BornInf[2] - UVap[2]) / Duvy); // iso V =BornInf(2)
                }
                else
                {
                    theStepV = Step;
                }
            }
            else
            {
                if (Duvy != 0.0)
                {
                    theStepV = Math.Abs((BornSup[2] - UVap[2]) / Duvy); // iso V =BornSup(2)
                }
                else
                {
                    theStepV = Step;
                }
            }

            if (theStepU <= theStepV)
            {
                Step = theStepU;
                if (infu)
                {
                    UVap[1] = BornInf[1];
                    BornSup[1] = BornInf[1];
                }
                else
                {
                    UVap[1] = BornSup[1];
                    BornInf[1] = BornSup[1];
                }
                UVap[2] += Step * Duvy * StepSign;
            }
            else
            {
                Step = theStepV;
                if (infv)
                {
                    UVap[2] = BornInf[2];
                    BornSup[2] = BornInf[2];
                }
                else
                {
                    UVap[2] = BornSup[2];
                    BornInf[2] = BornSup[2];
                }
                UVap[1] += Step * Duvx * StepSign;
            }
            return true;
        }

        else if (infu)
        { // jag 940616
            if (Duvx != 0.0)
            {
                double aStep = Math.Abs((BornInf[1] - UVap[1]) / Duvx); // iso U =BornInf(1)
                if (aStep < Step)
                {
                    Step = aStep;
                }
            }
            BornSup[1] = BornInf[1]; // limit the parameter
            UVap[1] = BornInf[1];
            UVap[2] += Step * Duvy * StepSign;
            return true;
        }
        else if (supu)
        { // jag 940616
            if (Duvx != 0.0)
            {
                double aStep = Math.Abs((BornSup[1] - UVap[1]) / Duvx); // iso U =BornSup(1)
                if (aStep < Step)
                {
                    Step = aStep;
                }
            }
            BornInf[1] = BornSup[1]; // limit the parameter
            UVap[1] = BornSup[1];
            UVap[2] += Step * Duvy * StepSign;
            return true;
        }
        else if (infv)
        { // jag 940616
            if (Duvy != 0.0)
            {
                double aStep = Math.Abs((BornInf[2] - UVap[2]) / Duvy); // iso V =BornInf(2)
                if (aStep < Step)
                {
                    Step = aStep;
                }
            }
            BornSup[2] = BornInf[2];
            UVap[1] += Step * Duvx * StepSign;
            UVap[2] = BornInf[2];
            return true;
        }
        else if (supv)
        { // jag 940616
            if (Duvy != 0.0)
            {
                double aStep = Math.Abs((BornSup[2] - UVap[2]) / Duvy); // iso V =BornSup(2)
                if (aStep < Step)
                {
                    Step = aStep;
                }
            }
            BornInf[2] = BornSup[2];
            UVap[1] += Step * Duvx * StepSign;
            UVap[2] = BornSup[2];
            return true;
        }
        return true;
    }

    /// <summary>
    /// Stop test for an OPEN line: crossing test on all interior points, then stop test on all start points. If a stop
    /// is detected, <paramref name="Irang"/> is the index of the stop point and <paramref name="UV"/> its parameters.
    /// </summary>
    protected bool TestArretPassage(NCollection_Sequence<double> Umult, NCollection_Sequence<double> Vmult,
                                    TheIWFunction sp, math_Vector UV, ref int Irang)
    {
        // Umult et Vmult : table of stop (or crossing) points on border,
        //         here only the crossing points are taken into account.
        // UV     : the current point.
        // Irang  : at exit : give index of the stop point in uvstart1 or 0.
        //          consider that there is no risk of crossing only if there is one crossing point.
        double Up, Vp, Du, Dv, Dup, Dvp, Utest, Vtest;
        int j, N, ind;
        double tolu = tolerance[1];
        double tolv = tolerance[2];
        double tolu2 = 10.0 * tolerance[1];
        double tolv2 = 10.0 * tolerance[2];

        bool Arrive = false;

        // crossing test  on point that can start a loop; mark
        // as processed if passes through an open line

        if (!reversed)
        {
            previousPoint.ParametersOnS2(out Up, out Vp);
        }
        else
        {
            previousPoint.ParametersOnS1(out Up, out Vp);
        }

        for (int i = 1; i < wd2.Size(); i++)
        {
            if (wd2[i].etat > 0)
            {
                // debug jag 05.04.94

                //      if ((Up-wd2[i].ustart)*(UV(1)-wd2[i].ustart) +
                //	  (Vp-wd2[i].vstart)*(UV(2)-wd2[i].vstart) <= 0)
                Utest = wd2[i].ustart;
                Vtest = wd2[i].vstart;

                Du = UV[1] - Utest;
                Dv = UV[2] - Vtest;
                Dup = Up - Utest;
                Dvp = Vp - Vtest;

                //-- lbr le 30 oct 97

                // IFV for OCC20285

                if ((Math.Abs(Du) < tolu2 && Math.Abs(Dv) < tolv2) || (Math.Abs(Dup) < tolu2 && Math.Abs(Dvp) < tolv2))
                {
                    wd2[i].etat = -wd2[i].etat;
                }
                else
                {
                    double DDu = (UV[1] - Up);
                    double DDv = (UV[2] - Vp);
                    double DDD = DDu * DDu + DDv * DDv;
                    double DD1 = Du * Du + Dv * Dv;
                    if (DD1 <= DDD)
                    {
                        double DD2 = Dup * Dup + Dvp * Dvp;
                        if (DD2 <= DDD && ((Du * Dup) + (Dv * Dvp * tolu / tolv) <= 0.0))
                        {
                            wd2[i].etat = -wd2[i].etat;
                        }
                    }
                }
            }
        }

        // stop test on point given at input and not yet processed

        //  Modified by Sergey KHROMOV - Tue Nov 20 10:55:01 2001 Begin
        // Check of all path points in the following order:
        //   * First check all not treated points;
        //   * After that check of already treated ones.
        int l;

        //// Modified by jgv, 28.07.2010 for OCC21914 ////
        // There are several path points between (Up,Vp) and UV
        // So several path points satisfy the condition
        // Dup*UV1mUtest + Dvp*UV2mVtest) < 0
        // We choose from them the path point with
        // minimum distance to (Up,Vp)
        NCollection_Sequence<int> i_candidates = new();
        NCollection_Sequence<double> SqDist_candidates = new();

        for (l = 1; l <= 2 && !Arrive; l++)
        {
            bool isToCheck;

            for (int i = 1; i < wd1.Size(); i++)
            {
                if (l == 1)
                {
                    isToCheck = (wd1[i].etat > 0);
                }
                else
                {
                    isToCheck = (wd1[i].etat < 0);
                }

                if (isToCheck)
                {
                    //  Modified by Sergey KHROMOV - Tue Nov 20 11:03:16 2001 End

                    // debug jag voir avec isg

                    Utest = wd1[i].ustart;
                    Vtest = wd1[i].vstart;
                    Dup = Up - Utest;
                    Dvp = Vp - Vtest;
                    if (Math.Abs(Dup) >= tolu || Math.Abs(Dvp) >= tolv)
                    {
                        double UV1mUtest = UV[1] - Utest;
                        double UV2mVtest = UV[2] - Vtest;
                        if (((Dup * UV1mUtest + Dvp * UV2mVtest) < 0)
                            || (Math.Abs(UV1mUtest) < tolu && Math.Abs(UV2mVtest) < tolv))
                        {
                            i_candidates.Append(i);
                            SqDist_candidates.Append(Dup * Dup + Dvp * Dvp);
                        }
                        else if (nbMultiplicities[i] > 0 && i_candidates.IsEmpty())
                        {
                            N = 0;
                            for (int k = 1; k < i; k++)
                            {
                                N += nbMultiplicities[k];
                            }
                            for (j = N + 1; j <= N + nbMultiplicities[i]; j++)
                            {
                                if (((Up - Umult.Value(j)) * (UV[1] - Umult.Value(j))
                                         + (Vp - Vmult.Value(j)) * (UV[2] - Vmult.Value(j))
                                     < 0)
                                    || (Math.Abs(UV[1] - Umult.Value(j)) < tolu && Math.Abs(UV[2] - Vmult.Value(j)) < tolv))
                                {
                                    Irang = i;
                                    Arrive = true;
                                    UV[1] = Utest;
                                    UV[2] = Vtest;
                                    break;
                                }
                            }
                        }
                        if (Arrive)
                        {
                            sp.Values(UV, myBidF, myBidD);
                            break;
                        }
                    }
                }
            } // end of for (i = 1; i < wd1.Size(); i++)
            if (!i_candidates.IsEmpty())
            {
                double MinSqDist = RealLast();
                for (ind = 1; ind <= i_candidates.Length(); ind++)
                {
                    if (SqDist_candidates.Value(ind) < MinSqDist)
                    {
                        MinSqDist = SqDist_candidates.Value(ind);
                        Irang = i_candidates.Value(ind);
                    }
                }
                Arrive = true;
                UV[1] = wd1[Irang].ustart;
                UV[2] = wd1[Irang].vstart;
            }
        } // end of for (l = 1; l <= 2 && !Arrive; l++)
        return Arrive;
    }

    /// <summary>
    /// Stop test for a CLOSED line started at the interior point <paramref name="Index"/>: crossing test on all
    /// interior points, then on all passing points (<paramref name="Irang"/>: the passing point crossed, or 0).
    /// </summary>
    protected bool TestArretPassage(NCollection_Sequence<double> Umult, NCollection_Sequence<double> Vmult,
                                    math_Vector UV, int Index, ref int Irang)
    {
        double Up, Vp, Scal;
        bool Arrive = false;
        int N, k, i;
        double Utest, Vtest;
        double tolu = tolerance[1];
        double tolv = tolerance[2];

        // tests of stop and of crossing on all interior points.

        if (!reversed)
        {
            previousPoint.ParametersOnS2(out Up, out Vp);
        }
        else
        {
            previousPoint.ParametersOnS1(out Up, out Vp);
        }

        double UV1 = UV[1];
        double UV2 = UV[2];

        // Normalizing factor. If it is less than 1.0 then the range will be expanded.
        // This is no good for computation. Therefore, it is limited.
        // Do not limit this factor in case of highly anisotropic parametrization
        //(parametric space is considerably larger in one direction than another).
        bool isHighlyAnisotropic = Math.Max(tolu, tolv) > 1000.0 * Math.Min(tolu, tolv);
        double deltau = mySRangeU.IsVoid
                            ? UM - Um
                            : (isHighlyAnisotropic ? mySRangeU.Delta() : Math.Max(mySRangeU.Delta(), 1.0));
        double deltav = mySRangeV.IsVoid
                            ? VM - Vm
                            : (isHighlyAnisotropic ? mySRangeV.Delta() : Math.Max(mySRangeV.Delta(), 1.0));

        Up /= deltau;
        UV1 /= deltau;
        Vp /= deltav;
        UV2 /= deltav;

        tolu /= deltau;
        tolv /= deltav;

        double tolu2 = tolu + tolu;
        double tolv2 = tolv + tolv;

        double dPreviousCurrent = (Up - UV1) * (Up - UV1) + (Vp - UV2) * (Vp - UV2);
        for (k = 1; k < wd2.Size(); k++)
        {
            if (wd2[k].etat > 0)
            {
                Utest = wd2[k].ustart;
                Vtest = wd2[k].vstart;

                Utest /= deltau;
                Vtest /= deltav;

                double UV1mUtest = UV1 - Utest;
                double UV2mVtest = UV2 - Vtest;
                if ((UV1mUtest < tolu2 && UV1mUtest > -tolu2) && (UV2mVtest < tolv2 && UV2mVtest > -tolv2))
                {
                    if (Index != k)
                    {
                        wd2[k].etat = -wd2[k].etat; //-- mark the point as a crossing point
                    }
                    else
                    { //-- Index == k
                        Arrive = true;
                    }
                }
                else
                {
                    double UpmUtest = (Up - Utest);
                    double VpmVtest = (Vp - Vtest);
                    double dPreviousStart = (UpmUtest) * (UpmUtest) + (VpmVtest) * (VpmVtest);
                    double dCurrentStart = UV1mUtest * UV1mUtest + UV2mVtest * UV2mVtest;

                    Scal = (UpmUtest) * (UV1mUtest) + (VpmVtest) * (UV2mVtest);
                    if ((Math.Abs(UpmUtest) < tolu && Math.Abs(VpmVtest) < tolv))
                    {
                        if (Index != k)
                        {
                            wd2[k].etat = -wd2[k].etat;
                        }
                    }
                    else if (Scal < 0 && (dPreviousStart + dCurrentStart < dPreviousCurrent))
                    {
                        if (Index == k)
                        { // on a boucle.
                            Arrive = true;
                        }
                        else
                        {
                            wd2[k].etat = -wd2[k].etat; // mark the point as a crossing point
                        }
                    }
                    else if (k != Index)
                    {
                        if (dPreviousStart < dPreviousCurrent * 0.25)
                        {
                            wd2[k].etat = -wd2[k].etat; // mark the point as a crossing point
                        }
                        else
                        {
                            if (dCurrentStart < dPreviousCurrent * 0.25)
                            {
                                wd2[k].etat = -wd2[k].etat; // mark the point as a crossing point
                            }
                            else
                            {
                                double UMidUtest = 0.5 * (UV1 + Up) - Utest;
                                double VMidVtest = 0.5 * (UV2 + Vp) - Vtest;
                                double dMiddleStart = UMidUtest * UMidUtest + VMidVtest * VMidVtest;

                                if (dMiddleStart < dPreviousCurrent * 0.5)
                                {
                                    wd2[k].etat = -wd2[k].etat; // mark the point as a crossing point
                                }
                            }
                        }
                    }
                }
            }
        }

        // crossing test on crossing points.

        Irang = 0;
        for (i = 1; i < wd1.Size(); i++)
        {
            if (wd1[i].etat > 0 && wd1[i].etat < 11)
            { // test of crossing points
                Utest = wd1[i].ustart;
                Vtest = wd1[i].vstart;
                Utest /= deltau;
                Vtest /= deltav;

                if (((Up - Utest) * (UV1 - Utest) + (Vp - Vtest) * (UV2 - Vtest) < 0)
                    || (Math.Abs(UV1 - Utest) < tolu && Math.Abs(UV2 - Vtest) < tolv))
                {
                    Irang = i;
                }
                else if (nbMultiplicities[i] > 0)
                {
                    N = 0;
                    for (k = 1; k < i; k++)
                    {
                        N = N + nbMultiplicities[k];
                    }
                    for (int j = N + 1; j <= N + nbMultiplicities[i]; j++)
                    {
                        double Umultj = Umult.Value(j) / deltau;
                        double Vmultj = Vmult.Value(j) / deltav;
                        if (((Up - Umultj) * (UV1 - Umultj) + (Vp - Vmultj) * (UV2 - Vmultj) < 0)
                            || (Math.Abs(UV1 - Umultj) < tolu && Math.Abs(UV2 - Vmultj) < tolv))
                        {
                            Irang = i;
                            break;
                        }
                    }
                }
            }
        }
        return Arrive;
    }

    /// <summary>
    /// Stop test on the added points (the points on the natural border that were not given at input): returns the
    /// added point in <paramref name="Psol"/>, its index among the added points in <paramref name="Irang"/> and its
    /// parameters in <paramref name="UV"/>.
    /// </summary>
    protected bool TestArretAjout(TheIWFunction sp, math_Vector UV, ref int Irang, ref IntSurf_PntOn2S Psol)
    {
        bool Arrive = false;
        double U1, V1;
        double Up, Vp;

        if (!reversed)
        {
            previousPoint.ParametersOnS2(out Up, out Vp);
        }
        else
        {
            previousPoint.ParametersOnS1(out Up, out Vp);
        }

        int nbAjout = seqAjout.Length();
        for (int i = 1; i <= nbAjout; i++)
        {
            Irang = seqAjout.Value(i);

            // add test std::abs(Irang) <= lines.Length() for the case when
            // a closed line is opened by adding a  1 point on this same
            // line. Anyway there is a huge problem as 2 points will be
            // added on this line...

            if (Math.Abs(Irang) <= lines.Length())
            {
                TheIWLine Line = lines.Value(Math.Abs(Irang));
                if (Irang > 0)
                {
                    Psol = Line.Value(Line.NbPoints());
                }
                else
                {
                    Psol = Line.Value(1);
                }
                if (!reversed)
                {
                    Psol.ParametersOnS2(out U1, out V1);
                }
                else
                {
                    Psol.ParametersOnS1(out U1, out V1);
                }
                if (((Up - U1) * (UV[1] - U1) + (Vp - V1) * (UV[2] - V1)) < 0
                    || (Math.Abs(UV[1] - U1) < tolerance[1] && Math.Abs(UV[2] - V1) < tolerance[2]))
                {
                    // jag 940615	Irang= -std::abs(Irang);
                    Arrive = true;
                    UV[1] = U1;
                    UV[2] = V1;
                    sp.Values(UV, myBidF, myBidD);
                    break;
                }
            }
        }
        return Arrive;
    }

    /// <summary>
    /// Finds interior start points in the holes between the lines' lone ends (<paramref name="CopySeqAlone"/>): the
    /// root of the function near the middle of the two nearest ends, when it is between them and on no line.
    /// </summary>
    protected void FillPntsInHoles(TheIWFunction sp, NCollection_Sequence<int> CopySeqAlone,
                                   NCollection_Sequence<IntSurf_InteriorPoint> PntsInHoles)
    {
        math_Vector BornInf = new(1, 2), BornSup = new(1, 2);
        BornInf[1] = Um;
        BornSup[1] = UM;
        BornInf[2] = Vm;
        BornSup[2] = VM;
        PointLineLine.Clear();
        NCollection_Sequence<int> SeqToRemove = new();
        NCollection_Map<int> BadSolutions = new();

        for (int i = 1; i < CopySeqAlone.Length(); i++)
        {
            int Irang1 = CopySeqAlone.Value(i);
            if (Irang1 == 0)
            {
                continue;
            }
            bool ToRemove = false;
            IntSurf_PntOn2S PointAlone1, PointAlone2 = new();
            TheIWLine Line1 = lines.Value(Math.Abs(Irang1));
            if (Irang1 > 0)
            {
                PointAlone1 = Line1.Value(Line1.NbPoints());
            }
            else
            {
                PointAlone1 = Line1.Value(1);
            }
            Point2D P2d1 = PointAlone1.ValueOnSurface(reversed), P2d2;
            double MinSqDist = RealLast();
            int MinRang = 0, MinIndex = 0;
            for (int j = i + 1; j <= CopySeqAlone.Length(); j++)
            {
                int Irang2 = CopySeqAlone.Value(j);
                if (Irang2 == 0 || BadSolutions.Contains(Irang2))
                {
                    continue;
                }
                TheIWLine Line2a = lines.Value(Math.Abs(Irang2));
                if (Irang2 > 0)
                {
                    PointAlone2 = Line2a.Value(Line2a.NbPoints());
                }
                else
                {
                    PointAlone2 = Line2a.Value(1);
                }
                P2d2 = PointAlone2.ValueOnSurface(reversed);
                double aSqDist = P2d1.SquareDistance(P2d2);
                if (aSqDist < MinSqDist)
                {
                    MinSqDist = aSqDist;
                    MinRang = Irang2;
                    MinIndex = j;
                }
            }
            // processing points from Irang1 and MinRang
            if (MinRang == 0)
            {
                SeqToRemove.Append(Irang1);
                BadSolutions.Clear();
                continue;
            }
            // Ends of same line
            if (Math.Abs(Irang1) == Math.Abs(MinRang) && lines.Value(Math.Abs(Irang1)).NbPoints() == 2)
            {
                SeqToRemove.Append(Irang1);
                SeqToRemove.Append(MinRang);
                CopySeqAlone[i] = 0;
                CopySeqAlone[MinIndex] = 0;
                BadSolutions.Clear();
                continue;
            }
            ///////////////////
            TheIWLine Line2 = lines.Value(Math.Abs(MinRang));
            if (MinRang > 0)
            {
                PointAlone2 = Line2.Value(Line2.NbPoints());
            }
            else
            {
                PointAlone2 = Line2.Value(1);
            }
            Point3D Pnt1 = PointAlone1.Value();
            Point3D Pnt2 = PointAlone2.Value();
            P2d2 = PointAlone2.ValueOnSurface(reversed);
            double MinSqDist3d = Pnt1.SquareDistance(Pnt2);
            if (MinSqDist3d <= epsilon
                || (Math.Abs(P2d1.X() - P2d2.X()) <= tolerance[1] && Math.Abs(P2d1.Y() - P2d2.Y()) <= tolerance[2])) // close points
            {
                ToRemove = true;
            }
            else // real curve
            {
                math_Vector UVap = new(1, 2), UV = new(1, 2);
                UVap[1] = (P2d1.X() + P2d2.X()) / 2;
                UVap[2] = (P2d1.Y() + P2d2.Y()) / 2;
                FunctionSetRoot Rsnld = new(sp, tolerance);
                Rsnld.Perform(sp, UVap, BornInf, BornSup);
                if (Rsnld.IsDone && Math.Abs(sp.Root()) <= sp.Tolerance() && !sp.IsTangent())
                {
                    Rsnld.Root(UV);
                    Point2D Pmid = new(UV[1], UV[2]);
                    Vector2D P1P2 = new(P2d1, P2d2);
                    Vector2D P1Pmid = new(P2d1, Pmid);
                    Vector2D P2Pmid = new(P2d2, Pmid);
                    double ScalProd1 = P1P2 * P1Pmid;
                    double ScalProd2 = P1P2 * P2Pmid;
                    bool IsPmidValid = (ScalProd1 > 0.0 && ScalProd2 < 0); // Pmid is in the middle
                    if (IsPmidValid)
                    {
                        for (int iline = 1; iline <= lines.Length(); iline++)
                        {
                            if (IsPointOnLine(Pmid, iline))
                            {
                                IsPmidValid = false;
                                break;
                            }
                        }
                    }
                    if (IsPmidValid)
                    {
                        IntSurf_InteriorPoint aPoint = new(sp.Point(), UV[1], UV[2], sp.Direction3d(),
                                                           new Vector2D(sp.Direction2d()));
                        PntsInHoles.Append(aPoint);
                        NCollection_List<int> LineLine = new();
                        LineLine.Append(Irang1);
                        LineLine.Append(MinRang);
                        PointLineLine.Bind(PntsInHoles.Length(), LineLine);
                    }
                    else
                    {
                        BadSolutions.Add(MinRang);
                        i--;
                        continue;
                    }
                }
                else
                {
                    BadSolutions.Add(MinRang);
                    i--;
                    continue;
                }
            }
            CopySeqAlone[i] = 0;
            CopySeqAlone[MinIndex] = 0;
            if (ToRemove)
            {
                SeqToRemove.Append(Irang1);
                SeqToRemove.Append(MinRang);
            }
            BadSolutions.Clear();
        }

        for (int i = 1; i <= SeqToRemove.Length(); i++)
        {
            for (int j = 1; j <= seqAlone.Length(); j++)
            {
                if (seqAlone.Value(j) == SeqToRemove.Value(i))
                {
                    seqAlone.Remove(j);
                    break;
                }
            }
        }
    }

    /// <summary>
    /// Stop test on a border: on all the start points already marked and on the whole current line, which is cut at
    /// the stop point if one is found. |Irang| is the stop point's index or 0; if Irang &lt; 0 the point must be added to
    /// the line (no cut). UV is the stop point's parameters.
    /// </summary>
    protected void TestArretCadre(NCollection_Sequence<double> Umult, NCollection_Sequence<double> Vmult, TheIWLine Line,
                                  TheIWFunction sp, math_Vector UV, ref int Irang)
    {
        double Scal, Up, Vp, Uc, Vc;
        int N;
        bool Found = false;

        Irang = 0;
        for (int i = 1; i < wd1.Size(); i++)
        {
            if (wd1[i].etat < 0)
            {
                N = 0; // range in UVMult.
                if (nbMultiplicities[i] > 0)
                {
                    for (int k = 1; k < i; k++)
                    {
                        N += nbMultiplicities[k];
                    }
                }
                if (!reversed)
                {
                    Line.Value(1).ParametersOnS2(out Up, out Vp);
                }
                else
                {
                    Line.Value(1).ParametersOnS1(out Up, out Vp);
                }
                int nbp = Line.NbPoints();
                Vector2D aVec1, aVec2;
                for (int j = 2; j <= nbp; j++)
                {
                    if (!reversed)
                    {
                        Line.Value(j).ParametersOnS2(out Uc, out Vc);
                    }
                    else
                    {
                        Line.Value(j).ParametersOnS1(out Uc, out Vc);
                    }

                    aVec1 = new Vector2D(Up - wd1[i].ustart, Vp - wd1[i].vstart);
                    aVec2 = new Vector2D(Uc - wd1[i].ustart, Vc - wd1[i].vstart);
                    CutVectorByTolerances(ref aVec1, tolerance);
                    CutVectorByTolerances(ref aVec2, tolerance);

                    Scal = aVec1 * aVec2;

                    // if a stop point is found: stop the line on this point.
                    if (Scal < 0)
                    {
                        Line.Cut(j);
                        nbp = Line.NbPoints();
                        Irang = i;
                        UV[1] = wd1[Irang].ustart;
                        UV[2] = wd1[Irang].vstart;
                        Found = true;
                    }
                    else if (Math.Abs(Uc - wd1[i].ustart) < tolerance[1] && Math.Abs(Vc - wd1[i].vstart) < tolerance[2])
                    {
                        Line.Cut(j);
                        nbp = Line.NbPoints();
                        Irang = i;
                        UV[1] = wd1[Irang].ustart;
                        UV[2] = wd1[Irang].vstart;
                        Found = true;
                    }
                    else if (nbMultiplicities[i] > 0)
                    {
                        for (int k = N + 1; k <= N + nbMultiplicities[i]; k++)
                        {
                            aVec1.SetCoord(Up - Umult.Value(k), Vp - Vmult.Value(k));
                            aVec2.SetCoord(Uc - Umult.Value(k), Vc - Vmult.Value(k));
                            CutVectorByTolerances(ref aVec1, tolerance);
                            CutVectorByTolerances(ref aVec2, tolerance);

                            Scal = aVec1 * aVec2;

                            if (Scal < 0)
                            {
                                Line.Cut(j);
                                nbp = Line.NbPoints();
                                Irang = i;
                                UV[1] = wd1[Irang].ustart;
                                UV[2] = wd1[Irang].vstart;
                                Found = true;
                                break;
                            }
                            else if (Math.Abs(Uc - Umult.Value(k)) < tolerance[1] && Math.Abs(Vc - Vmult.Value(k)) < tolerance[2])
                            {
                                Line.Cut(j);
                                nbp = Line.NbPoints();
                                Irang = i;
                                UV[1] = wd1[Irang].ustart;
                                UV[2] = wd1[Irang].vstart;
                                Found = true;
                                break;
                            }
                        }
                    }
                    if (Found)
                    {
                        sp.Values(UV, myBidF, myBidD);
                        int NBP = Line.NbPoints();
                        Line.TangentVector(out int Indextg);
                        if (Indextg > NBP)
                        {
                            if (j > 3 && j <= NBP + 1)
                            {
                                // OCCT computes and orients this direction, then sets the previous tangent (kept:
                                // Direction3d throws at a tangent point, as OCCT's).
                                Vector3D Dir3d = sp.Direction3d();
                                Vector3D Dir3d1 = new(Line.Value(j - 2).Value(), Line.Value(j - 1).Value());
                                double dot = Dir3d.Dot(Dir3d1);
                                if (dot < 0.0)
                                { // Normally this Function should not be used often !!!
                                    Dir3d.Reverse();
                                }
                                Line.SetTangentVector(previousd3d, j - 1);
                            }
                        }

                        return;
                    }
                    Up = Uc;
                    Vp = Vc;
                }

                // now the last point of the line and the last calculated point are compated.
                // there will be no need to "Cut"

                aVec1 = new Vector2D(Up - wd1[i].ustart, Vp - wd1[i].vstart);
                aVec2 = new Vector2D(UV[1] - wd1[i].ustart, UV[2] - wd1[i].vstart);
                CutVectorByTolerances(ref aVec1, tolerance);
                CutVectorByTolerances(ref aVec2, tolerance);

                Scal = aVec1 * aVec2;

                if (Scal < 0)
                {
                    Irang = i;
                    UV[1] = wd1[Irang].ustart;
                    UV[2] = wd1[Irang].vstart;
                    Found = true;
                }
                else if (Math.Abs(UV[1] - wd1[i].ustart) < tolerance[1] && Math.Abs(UV[2] - wd1[i].vstart) < tolerance[2])
                {
                    Irang = i;
                    UV[1] = wd1[Irang].ustart;
                    UV[2] = wd1[Irang].vstart;
                    Found = true;
                }
                else if (nbMultiplicities[i] > 0)
                {
                    for (int j = N + 1; j <= N + nbMultiplicities[i]; j++)
                    {
                        aVec1.SetCoord(Up - Umult.Value(j), Vp - Vmult.Value(j));
                        aVec2.SetCoord(UV[1] - Umult.Value(j), UV[2] - Vmult.Value(j));
                        CutVectorByTolerances(ref aVec1, tolerance);
                        CutVectorByTolerances(ref aVec2, tolerance);

                        Scal = aVec1 * aVec2;

                        if (Scal < 0)
                        {
                            Irang = i;
                            UV[1] = wd1[Irang].ustart;
                            UV[2] = wd1[Irang].vstart;
                            Found = true;
                            break;
                        }
                        else if (Math.Abs(UV[1] - Umult.Value(j)) < tolerance[1] && Math.Abs(UV[2] - Vmult.Value(j)) < tolerance[2])
                        {
                            Irang = i;
                            UV[1] = wd1[Irang].ustart;
                            UV[2] = wd1[Irang].vstart;
                            Found = true;
                            break;
                        }
                    }
                }
                if (Found)
                {
                    Irang = -Irang; // jag 941017
                    sp.Values(UV, myBidF, myBidD);
                    return;
                }
            }
        }
    }

    // IntWalk_IWalking_3.gxx

    /// <summary>
    /// Processing of the open lines: from each start point that is neither passing nor tangent and not yet processed,
    /// walk with a step from the deflection and the largest step, framing the approached point on the borders,
    /// dividing the step when no point is found, until a stop test passes (start point, added point, border).
    /// </summary>
    protected void ComputeOpenLine(NCollection_Sequence<double> Umult, NCollection_Sequence<double> Vmult,
                                   NCollection_Sequence<IntSurf_PathPoint> Pnts1, TheIWFunction Func, ref bool Rajout)
    {
        int I = 0, N = 0, SaveN = 0;
        math_Vector BornInf = new(1, 2), BornSup = new(1, 2), UVap = new(1, 2);
        double PasC = 0.0, PasCu = 0.0, PasCv = 0.0;
        bool Arrive = false; // shows if the line ends
        bool Cadre = false; // shows if one is on border of the domain
        bool ArretAjout = false; // shows if one is on added point
        IntSurf_PntOn2S Psol = new();
        TheIWLine CurrentLine; // line under construction
        bool Tgtend = false;

        IntWalk_StatusDeflection aStatus = IntWalk_OK, StatusPrecedent = IntWalk_OK;

        int NbDivision = 0;
        // number of divisions of step for each section

        int StepSign = 0;

        IntSurf_PathPoint PathPnt = new();

        BornInf[1] = Um;
        BornSup[1] = UM;
        BornInf[2] = Vm;
        BornSup[2] = VM;

        FunctionSetRoot Rsnld = new(Func, tolerance);
        int nbPath = Pnts1.Length();

        // modified by NIZHNY-MKK  Fri Oct 27 12:32:34 2000.BEGIN
        int[] movementdirectioninfo = new int[nbPath + 1];
        for (I = 0; I <= nbPath; I++)
        {
            movementdirectioninfo[I] = 0;
        }
        // modified by NIZHNY-MKK  Fri Oct 27 12:32:38 2000.END

        TheIWFunction aFuncForDuplicate = TheIWFunction.Copy(Func);

        for (I = 1; I <= nbPath; I++)
        {
            // start point of the progression
            //      if (wd1[I].etat > 11) {
            //  modified by NIZHNY-MKK  Fri Oct 27 12:33:37 2000.BEGIN
            if ((wd1[I].etat > 11) || ((wd1[I].etat < -11) && (movementdirectioninfo[I] != 0)))
            {
                // modified by NIZHNY-MKK  Fri Oct 27 12:33:43 2000.END
                PathPnt = Pnts1.Value(I);
                UVap[1] = wd1[I].ustart;
                UVap[2] = wd1[I].vstart;
                MakeWalkingPoint(11, UVap[1], UVap[2], Func, ref previousPoint);

                if (IsPointOnLine(previousPoint, BornInf, BornSup, Rsnld, aFuncForDuplicate))
                {
                    wd1[I].etat = -Math.Abs(wd1[I].etat); // mark point as processed
                    continue;
                }

                CurrentLine = new TheIWLine();
                CurrentLine.SetTangencyAtBegining(false);
                Tgtend = false;
                CurrentLine.AddStatusFirst(false, true, I, PathPnt);
                previousd3d = Func.Direction3d();
                previousd2d = Func.Direction2d();
                CurrentLine.AddPoint(previousPoint);
                // modified by NIZHNY-MKK  Fri Oct 27 12:34:32 2000.BEGIN
                if (movementdirectioninfo[I] != 0)
                {
                    if (movementdirectioninfo[I] < 0)
                    {
                        StepSign = -1;
                        CurrentLine.SetTangentVector(previousd3d.Reversed(), 1);
                    }
                    else
                    {
                        StepSign = 1;
                        CurrentLine.SetTangentVector(previousd3d, 1);
                    }
                }
                else
                {
                    double tyutuyt = IntSurf_PathPointTool.Direction3d(PathPnt) * previousd3d;
                    if (tyutuyt < 0)
                    {
                        StepSign = -1;
                        CurrentLine.SetTangentVector(previousd3d.Reversed(), 1);
                    }
                    else
                    {
                        StepSign = 1;
                        CurrentLine.SetTangentVector(previousd3d, 1);
                    }
                }
                // modified by NIZHNY-MKK  Fri Oct 27 12:34:37 2000.END

                //  Modified by Sergey KHROMOV - Tue Nov 20 10:41:45 2001 Begin
                wd1[I].etat = -Math.Abs(wd1[I].etat);
                movementdirectioninfo[I] = (movementdirectioninfo[I] == 0) ? StepSign : 0;
                //  Modified by Sergey KHROMOV - Tue Nov 20 10:41:56 2001 End
                // first step of advancement
                double d2dx = Math.Abs(previousd2d.X());
                double d2dy = Math.Abs(previousd2d.Y());
                if (d2dx < tolerance[1])
                {
                    PasC = pas * (VM - Vm) / d2dy;
                }
                else if (d2dy < tolerance[2])
                {
                    PasC = pas * (UM - Um) / d2dx;
                }
                else
                {
                    PasC = pas * Math.Min((UM - Um) / d2dx, (VM - Vm) / d2dy);
                }

                Arrive = false;
                ArretAjout = false;
                NbDivision = 0;
                StatusPrecedent = IntWalk_OK;
                // modified by NIZHNY-MKK  Fri Oct 27 12:39:37 2000
                int IndexOfPathPointDoNotCheck = 0;
                int aNbIter = 10;
                int aNbBadRootIter = 0; // consecutive iterations where Root > Tolerance
                while (!Arrive)
                {
                    Cadre = Cadrage(BornInf, BornSup, UVap, ref PasC, StepSign);
                    //  Border?

                    Rsnld.Perform(Func, UVap, BornInf, BornSup);

                    if (Cadre)
                    {
                        BornInf[1] = Um;
                        BornSup[1] = UM;
                        BornInf[2] = Vm;
                        BornSup[2] = VM;
                    }
                    if (Rsnld.IsDone)
                    {
                        if (Math.Abs(Func.Root()) > Func.Tolerance())
                        {
                            PasC = PasC / 2.0;
                            PasCu = Math.Abs(PasC * previousd2d.X());
                            PasCv = Math.Abs(PasC * previousd2d.Y());
                            ++aNbBadRootIter;
                            if ((PasCu <= tolerance[1] && PasCv <= tolerance[2]) || aNbBadRootIter > MaxDivision)
                            {
                                // Step collapsed below tolerance, or Cadrage keeps resetting
                                // the step (preventing convergence) after too many attempts.
                                if (CurrentLine.NbPoints() == 1)
                                {
                                    break;
                                }
                                Arrive = true;
                                CurrentLine.AddStatusLast(false);
                                Tgtend = true; // check
                                Rajout = true;
                                seqAlone.Append(lines.Length() + 1);
                                seqAjout.Append(lines.Length() + 1);
                            }
                        }
                        else
                        { // test stop
                            aNbBadRootIter = 0;
                            Rsnld.Root(UVap);
                            Arrive = TestArretPassage(Umult, Vmult, Func, UVap, ref N);
                            if (Arrive)
                            {
                                Cadre = false;
                                // in case if there is a frame and arrive at the same time
                            }
                            else
                            {
                                if (Rajout)
                                {
                                    ArretAjout = TestArretAjout(Func, UVap, ref N, ref Psol);
                                    SaveN = N;
                                    if (ArretAjout)
                                    {
                                        // jag 940615
                                        Tgtend = lines.Value(N).IsTangentAtEnd();
                                        N = -N;
                                    }
                                }
                                // modified by NIZHNY-MKK  Thu Nov  2 15:09:08 2000.BEGIN
                                if (!(Rajout && ArretAjout))
                                {
                                    double prevUp, prevVp;
                                    if (!reversed)
                                    {
                                        previousPoint.ParametersOnS2(out prevUp, out prevVp);
                                    }
                                    else
                                    {
                                        previousPoint.ParametersOnS1(out prevUp, out prevVp);
                                    }
                                    Arrive = TestPassedSolutionWithNegativeState(wd1, Umult, Vmult, prevUp, prevVp,
                                                                                 nbMultiplicities, tolerance, Func,
                                                                                 UVap, ref N, myBidF, myBidD);
                                    if (Arrive)
                                    {
                                        Cadre = false;
                                    }
                                }
                                // modified by NIZHNY-MKK  Thu Nov  2 15:09:13 2000.END
                                if (!ArretAjout && Cadre)
                                {
                                    if (CurrentLine.NbPoints() == 1)
                                    {
                                        break; // cancel the line
                                    }
                                    TestArretCadre(Umult, Vmult, CurrentLine, Func, UVap, ref N);
                                    //		if (N == 0) {
                                    if (N <= 0)
                                    { // jag 941017
                                        MakeWalkingPoint(2, UVap[1], UVap[2], Func, ref Psol);
                                        Tgtend = Func.IsTangent();
                                        N = -N;
                                    }
                                }
                            }
                            aStatus = TestDeflection(Func, Arrive, UVap, StatusPrecedent, ref NbDivision, ref PasC,
                                                     StepSign);
                            StatusPrecedent = aStatus;
                            if (aStatus == IntWalk_PasTropGrand)
                            {
                                Arrive = false;
                                ArretAjout = false;
                                Tgtend = false; // jag 940615
                                if (!reversed)
                                {
                                    previousPoint.ParametersOnS2(out UVap[1], out UVap[2]);
                                }
                                else
                                {
                                    previousPoint.ParametersOnS1(out UVap[1], out UVap[2]);
                                }
                            }
                            else if (ArretAjout || Cadre)
                            {
                                Arrive = true;
                                CurrentLine.AddStatusLast(false);
                                // if (aStatus != IntWalk_ArretSurPointPrecedent)
                                CurrentLine.AddPoint(Psol);
                                // Remove <SaveN> from <seqAlone>
                                for (int iseq = 1; iseq <= seqAlone.Length(); iseq++)
                                {
                                    if (seqAlone.Value(iseq) == SaveN)
                                    {
                                        seqAlone.Remove(iseq);
                                        break;
                                    }
                                }

                                if (Cadre && N == 0)
                                {
                                    Rajout = true;
                                    seqAjout.Append(lines.Length() + 1);
                                }
                            }
                            else if (aStatus == IntWalk_ArretSurPointPrecedent)
                            {
                                if (CurrentLine.NbPoints() == 1)
                                { // cancel the line
                                    Arrive = false;
                                    break;
                                }
                                Arrive = true;
                                Rajout = true;
                                // AddAlonePoint();
                                seqAlone.Append(lines.Length() + 1);
                                seqAjout.Append(lines.Length() + 1);
                                CurrentLine.AddStatusLast(false);
                                Tgtend = true; // check
                            }
                            else if (Arrive)
                            {
                                if (CurrentLine.NbPoints() == 1 && // cancel the line
                                    (N == I || aStatus == IntWalk_PointConfondu))
                                {
                                    // if N == I the main uv is probably lost
                                    // or the point is a point of accumulation
                                    // if point is confused the start data is bad
                                    Arrive = false;
                                    break;
                                }
                                // necessarily N > 0 jag 940617
                                // point of stop given at input
                                PathPnt = Pnts1.Value(N);

                                int etat1N = wd1[N].etat;
                                // modified by NIZHNY-MKK  Thu Nov  2 15:09:51 2000.BEGIN
                                // 	      if (etat1N < 11) { // passing point that is a stop
                                if (Math.Abs(etat1N) < 11)
                                { // passing point that is a stop
                                    // modified by NIZHNY-MKK  Thu Nov  2 15:12:11 2000.END
                                    if (aStatus == IntWalk_ArretSurPoint)
                                    {
                                        CurrentLine.AddStatusLast(false);
                                        Tgtend = true; // need check
                                    }
                                    else
                                    {
                                        Arrive = false;
                                    }
                                    CurrentLine.AddIndexPassing(N);
                                }
                                else
                                { // point of stop given at input
                                    if (etat1N == 11)
                                    {
                                        Tgtend = true;
                                    }
                                    CurrentLine.AddStatusLast(true, N, PathPnt);
                                }
                                AddPointInCurrentLine(N, PathPnt, CurrentLine);
                                if ((etat1N != 1 && etat1N != 11))
                                {
                                    // modified by NIZHNY-MKK  Fri Oct 27 12:43:05 2000.BEGIN
                                    // 		wd1[N].etat= - wd1[N].etat;
                                    wd1[N].etat = -Math.Abs(etat1N);
                                    movementdirectioninfo[N] = (movementdirectioninfo[N] == 0) ? StepSign : 0;
                                    if (Arrive && movementdirectioninfo[N] != 0)
                                    {
                                        IndexOfPathPointDoNotCheck = N;
                                    }

                                    if (Arrive)
                                    {
                                        Rajout = true;
                                        seqAjout.Append(lines.Length() + 1);
                                    }
                                    // modified by NIZHNY-MKK  Fri Oct 27 12:45:33 2000.END
                                }
                            }
                            else if (aStatus == IntWalk_ArretSurPoint)
                            {
                                Arrive = true;
                                CurrentLine.AddStatusLast(false);
                                Tgtend = true;
                                MakeWalkingPoint(1, UVap[1], UVap[2], Func, ref Psol);
                                CurrentLine.AddPoint(Psol);
                                Rajout = true;
                                seqAlone.Append(lines.Length() + 1);
                                seqAjout.Append(lines.Length() + 1);
                            }
                            else if (aStatus == IntWalk_OK)
                            {
                                MakeWalkingPoint(2, UVap[1], UVap[2], Func, ref previousPoint);
                                previousd3d = Func.Direction3d();
                                previousd2d = Func.Direction2d();
                                CurrentLine.AddPoint(previousPoint);
                            }
                            else if (aStatus == IntWalk_PointConfondu)
                            {
                                aNbIter--;
                            }
                        }
                    }
                    else
                    { // no numerical solution
                        PasC = PasC / 2.0;
                        PasCu = Math.Abs(PasC * previousd2d.X());
                        PasCv = Math.Abs(PasC * previousd2d.Y());
                        if (PasCu <= tolerance[1] && PasCv <= tolerance[2])
                        {
                            if (CurrentLine.NbPoints() == 1)
                            {
                                break;
                            }
                            Arrive = true;
                            CurrentLine.AddStatusLast(false);
                            Tgtend = true; // need check
                            Rajout = true;
                            seqAlone.Append(lines.Length() + 1);
                            seqAjout.Append(lines.Length() + 1);
                        }
                    }

                    if (aNbIter < 0)
                    {
                        break;
                    }
                } // end of started line

                if (Arrive)
                {
                    CurrentLine.SetTangencyAtEnd(Tgtend);
                    lines.Append(CurrentLine);
                    // modified by NIZHNY-MKK  Fri Oct 27 12:59:29 2000.BEGIN
                    movementdirectioninfo[I] = 0;
                    if (wd1[I].etat > 0)
                    {
                        // modified by NIZHNY-MKK  Fri Oct 27 12:59:42 2000.END
                        wd1[I].etat = -wd1[I].etat;
                    }

                    //-- lbr le 5 juin 97 (Pb ds Contap)
                    for (int av = 1; av <= nbPath; av++)
                    {
                        // modified by NIZHNY-MKK  Fri Oct 27 13:00:22 2000.BEGIN
                        // 	  if (wd1[av].etat > 11) {
                        if ((wd1[av].etat > 11)
                            || ((av != I) && (av != IndexOfPathPointDoNotCheck) && (wd1[av].etat < -11)
                                && (movementdirectioninfo[av] != 0)))
                        {
                            // modified by NIZHNY-MKK  Fri Oct 27 13:00:26 2000.END
                            double Uav = wd1[av].ustart;
                            double Vav = wd1[av].vstart;
                            double Uavp, Vavp;
                            IntSurf_PntOn2S avP = CurrentLine.Value(CurrentLine.NbPoints());
                            if (!reversed)
                            {
                                avP.ParametersOnS2(out Uavp, out Vavp);
                            }
                            else
                            {
                                avP.ParametersOnS1(out Uavp, out Vavp);
                            }
                            Uav -= Uavp;
                            Vav -= Vavp;
                            Uav *= 0.001;
                            Vav *= 0.001;
                            if (Math.Abs(Uav) < tolerance[1] && Math.Abs(Vav) < tolerance[2])
                            {
                                // modified by NIZHNY-MKK  Fri Oct 27 13:01:38 2000.BEGIN
                                // 	      wd1[av].etat=-wd1[av].etat;
                                if (wd1[av].etat < 0)
                                {
                                    movementdirectioninfo[av] = 0;
                                }
                                else
                                {
                                    wd1[av].etat = -wd1[av].etat;
                                    movementdirectioninfo[av] = StepSign;
                                }
                                // modified by NIZHNY-MKK  Fri Oct 27 13:01:42 2000.END
                                CurrentLine.AddStatusLast(true, av, Pnts1.Value(av));
                            }

                            IntSurf_PntOn2S avPP = CurrentLine.Value(1);
                            if (!reversed)
                            {
                                avPP.ParametersOnS2(out Uavp, out Vavp);
                            }
                            else
                            {
                                avPP.ParametersOnS1(out Uavp, out Vavp);
                            }
                            Uav = wd1[av].ustart;
                            Vav = wd1[av].vstart;
                            Uav -= Uavp;
                            Vav -= Vavp;
                            Uav *= 0.001;
                            Vav *= 0.001;
                            if (Math.Abs(Uav) < tolerance[1] && Math.Abs(Vav) < tolerance[2])
                            {
                                // modified by NIZHNY-MKK  Fri Oct 27 13:02:49 2000.BEGIN
                                // 	      wd1[av].etat=-wd1[av].etat;
                                if (wd1[av].etat < 0)
                                {
                                    movementdirectioninfo[av] = 0;
                                }
                                else
                                {
                                    wd1[av].etat = -wd1[av].etat;
                                    movementdirectioninfo[av] = -StepSign;
                                }
                                // modified by NIZHNY-MKK  Fri Oct 27 13:02:52 2000.END
                                CurrentLine.AddStatusFirst(false, true, av, Pnts1.Value(av));
                            }
                        }
                    }
                }
            } // end of point processing
        } // end of all points
    }

    // modified by NIZHNY-MKK  Thu Nov  2 15:07:53 2000.BEGIN
    private static bool TestPassedSolutionWithNegativeState(NCollection_LinearVector<IntWalk_WalkingData> wd,
                                                            NCollection_Sequence<double> Umult,
                                                            NCollection_Sequence<double> Vmult, double prevUp,
                                                            double prevVp, NCollection_LinearVector<int> nbMultiplicities,
                                                            math_Vector tolerance, TheIWFunction sp, math_Vector UV,
                                                            ref int Irang, math_Vector bidF, Matrix bidD)
    {
        bool Arrive = false;
        double Dup, Dvp, Utest, Vtest;
        double tolu = tolerance[1];
        double tolv = tolerance[2];
        int i, j, k, N;
        for (i = 1; i < wd.Size(); i++)
        {
            if (wd[i].etat < -11)
            {
                // debug jag see with isg

                Utest = wd[i].ustart;
                Vtest = wd[i].vstart;
                Dup = prevUp - Utest;
                Dvp = prevVp - Vtest;
                if (Math.Abs(Dup) >= tolu || Math.Abs(Dvp) >= tolv)
                {
                    double UV1mUtest = UV[1] - Utest;
                    double UV2mVtest = UV[2] - Vtest;
                    if (((Dup * UV1mUtest + Dvp * UV2mVtest) < 0)
                        || (Math.Abs(UV1mUtest) < tolu && Math.Abs(UV2mVtest) < tolv))
                    {
                        Irang = i;
                        Arrive = true;
                        UV[1] = Utest;
                        UV[2] = Vtest;
                    }
                    else if (nbMultiplicities[i] > 0)
                    {
                        N = 0;
                        for (k = 1; k < i; k++)
                        {
                            N += nbMultiplicities[k];
                        }
                        for (j = N + 1; j <= N + nbMultiplicities[i]; j++)
                        {
                            if (((prevUp - Umult.Value(j)) * (UV[1] - Umult.Value(j))
                                     + (prevVp - Vmult.Value(j)) * (UV[2] - Vmult.Value(j))
                                 < 0)
                                || (Math.Abs(UV[1] - Umult.Value(j)) < tolu && Math.Abs(UV[2] - Vmult.Value(j)) < tolv))
                            {
                                Irang = i;
                                Arrive = true;
                                UV[1] = Utest;
                                UV[2] = Vtest;
                                break;
                            }
                        }
                    }
                    if (Arrive)
                    {
                        sp.Values(UV, bidF, bidD);
                        break;
                    }
                }
            }
        }
        return Arrive;
    }
    // modified by NIZHNY-MKK  Thu Nov  2 15:07:58 2000.END

    // IntWalk_IWalking_4.gxx

    /// <summary>
    /// Processing of the closed lines: from each interior point not yet processed, walk until the line closes, or
    /// until it stops (then the line is opened and walked in the other direction).
    /// </summary>
    protected void ComputeCloseLine(NCollection_Sequence<double> Umult, NCollection_Sequence<double> Vmult,
                                    NCollection_Sequence<IntSurf_PathPoint> Pnts1,
                                    NCollection_Sequence<IntSurf_InteriorPoint> Pnts2, TheIWFunction Func,
                                    ref bool Rajout)
    {
        int I = 0, N = 0, SaveN = 0;
        math_Vector BornInf = new(1, 2), BornSup = new(1, 2);
        math_Vector Uvap = new(1, 2); // parameters of current approach
        double PasC = 0.0; // step of advancement on the tangent
        double PasCu = 0.0; // step of advancement current by U
        double PasCv = 0.0; // step of advancement current by V
        double PasSav = 0.0; // save first step of advancement
        bool Arrive = false; // show if line ends
        bool Cadre = false; // show if on border of the  domains
        bool ArretAjout = false; // show if on the added point
        IntSurf_PntOn2S Psol = new();
        TheIWLine CurrentLine; // line under construction
        IntSurf_PathPoint PathPnt = new();
        IntSurf_InteriorPoint LoopPnt = new();

        bool Tgtbeg = false, Tgtend = false;

        int StepSign = 0;

        IntWalk_StatusDeflection aStatus = IntWalk_OK, StatusPrecedent;
        int NbDivision = 0; // number of divisions of step
        // during calculation of  1 section

        int Ipass = 0;
        // index in the iterator of points on edge of point of passage

        BornInf[1] = Um;
        BornSup[1] = UM;
        BornInf[2] = Vm;
        BornSup[2] = VM;

        FunctionSetRoot Rsnld = new(Func, tolerance);
        int nbLoop = Pnts2.Length();

        // Check borders for degeneracy:
        Matrix D = new(1, 1, 1, 2);
        const int aNbSamplePnt = 10;
        Span<bool> isLeftDegeneratedBorder = [true, true];
        Span<bool> isRightDegeneratedBorder = [true, true];
        math_Vector aStep = new(1, 2);
        // OCCT: aStep = (BornSup - BornInf) / (aNbSamplePnt - 1), item by item.
        for (int anIdx = 1; anIdx <= 2; anIdx++)
        {
            aStep[anIdx] = (BornSup[anIdx] - BornInf[anIdx]) / (aNbSamplePnt - 1);
        }
        for (int aBorderIdx = 1; aBorderIdx <= 2; aBorderIdx++)
        {
            int aChangeIdx = aBorderIdx == 2 ? 1 : 2;
            math_Vector UV = new(1, 2);

            // Left border.
            UV[aBorderIdx] = BornInf[aBorderIdx];
            for (int aParamIdx = 0; aParamIdx < aNbSamplePnt; aParamIdx++)
            {
                double aParam = BornInf[aChangeIdx] + aParamIdx * aStep[aChangeIdx];
                UV[aChangeIdx] = aParam;
                Func.Derivatives(UV, D);
                if (Math.Abs(D[1, aChangeIdx]) > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    isLeftDegeneratedBorder[aBorderIdx - 1] = false;
                    break;
                }
            }

            // Right border.
            UV[aBorderIdx] = BornSup[aBorderIdx];
            for (int aParamIdx = 0; aParamIdx < aNbSamplePnt; aParamIdx++)
            {
                double aParam = BornInf[aChangeIdx] + aParamIdx * aStep[aChangeIdx];
                UV[aChangeIdx] = aParam;
                Func.Derivatives(UV, D);
                if (Math.Abs(D[1, aChangeIdx]) > global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    isRightDegeneratedBorder[aBorderIdx - 1] = false;
                    break;
                }
            }
        }

        TheIWFunction aFuncForDuplicate = TheIWFunction.Copy(Func);

        // OCCT's uvprev[2] and uv[2] of the degenerated-border fix, hoisted out of the loops.
        Span<double> uvprev = stackalloc double[2];
        Span<double> uv = stackalloc double[2];

        for (I = 1; I <= nbLoop; I++)
        {
            if (wd2[I].etat > 12)
            { // start point of closed line
                LoopPnt = Pnts2.Value(I);
                previousPoint.SetValue(IntSurf_InteriorPointTool.Value3d(LoopPnt), reversed, wd2[I].ustart,
                                       wd2[I].vstart);

                if (IsPointOnLine(previousPoint, BornInf, BornSup, Rsnld, aFuncForDuplicate))
                {
                    wd2[I].etat = -wd2[I].etat; // mark point as processed
                    continue;
                }

                previousd3d = IntSurf_InteriorPointTool.Direction3d(LoopPnt);
                previousd2d = IntSurf_InteriorPointTool.Direction2d(LoopPnt);

                CurrentLine = new TheIWLine();
                CurrentLine.AddPoint(previousPoint);
                CurrentLine.SetTangentVector(previousd3d, 1);
                Tgtbeg = false;
                Tgtend = false;
                Uvap[1] = wd2[I].ustart;
                Uvap[2] = wd2[I].vstart;

                StepSign = 1;

                // first step of advancement

                double d2dx = Math.Abs(previousd2d.X());
                double d2dy = Math.Abs(previousd2d.Y());
                if (d2dx < tolerance[1])
                {
                    PasC = pas * (VM - Vm) / d2dy;
                }
                else if (d2dy < tolerance[2])
                {
                    PasC = pas * (UM - Um) / d2dx;
                }
                else
                {
                    PasC = pas * Math.Min((UM - Um) / d2dx, (VM - Vm) / d2dy);
                }

                PasSav = PasC;

                Arrive = false;
                ArretAjout = false;
                NbDivision = 0;
                StatusPrecedent = IntWalk_OK;
                int aNbIter = 10;
                int aNbBadRootIter = 0; // consecutive iterations where Root > Tolerance
                while (!Arrive)
                { // as no test of stop is passed
                    Cadre = Cadrage(BornInf, BornSup, Uvap, ref PasC, StepSign); // border?

                    Rsnld.Perform(Func, Uvap, BornInf, BornSup);

                    bool isOnDegeneratedBorder = false;

                    if (Cadre)
                    { // update of limits.
                        BornInf[1] = Um;
                        BornSup[1] = UM;
                        BornInf[2] = Vm;
                        BornSup[2] = VM;
                    }
                    if (Rsnld.IsDone)
                    {
                        if (Math.Abs(Func.Root()) > Func.Tolerance())
                        { // no solution for the tolerance
                            PasC = PasC / 2.0;
                            PasCu = Math.Abs(PasC * previousd2d.X());
                            PasCv = Math.Abs(PasC * previousd2d.Y());
                            ++aNbBadRootIter;

                            if ((PasCu <= tolerance[1] && PasCv <= tolerance[2]) || aNbBadRootIter > MaxDivision)
                            {
                                if (CurrentLine.NbPoints() == 1)
                                {
                                    RemoveTwoEndPoints(I);
                                    break; // cancel the line
                                }
                                if (wd2[I].etat > 12)
                                { // the line should become open
                                    wd2[I].etat = 12; // declare it open
                                    ArretAjout = false;
                                    OpenLine(0, Psol, Pnts1, Func, CurrentLine);
                                    StepSign = -1;
                                    StatusPrecedent = IntWalk_OK;
                                    Arrive = false;
                                    PasC = PasSav;
                                    aNbBadRootIter = 0;
                                    Rajout = true;
                                    seqAlone.Append(-lines.Length() - 1);
                                    seqAjout.Append(-lines.Length() - 1);
                                }
                                else
                                { // line s is open
                                    Arrive = true;
                                    CurrentLine.AddStatusLast(false);
                                    Rajout = true;
                                    seqAlone.Append(lines.Length() + 1);
                                    seqAjout.Append(lines.Length() + 1);
                                    Tgtend = true;
                                }
                            }
                        }
                        else
                        { // there is a solution
                            aNbBadRootIter = 0;
                            Rsnld.Root(Uvap);

                            // Avoid uninitialized memory access.
                            if (CurrentLine.NbPoints() > 2)
                            {
                                for (int aCoordIdx = 1; aCoordIdx <= 2; aCoordIdx++)
                                {
                                    // Check degenerated cases and fix if possible.
                                    if ((isLeftDegeneratedBorder[aCoordIdx - 1]
                                         && Math.Abs(Uvap[aCoordIdx] - BornInf[aCoordIdx]) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                                        || (isRightDegeneratedBorder[aCoordIdx - 1]
                                            && Math.Abs(Uvap[aCoordIdx] - BornSup[aCoordIdx]) < global::Kerf.Engine.Foundation.Precision.PConfusion()))
                                    {
                                        if (!reversed)
                                        {
                                            CurrentLine.Value(CurrentLine.NbPoints() - 1).ParametersOnS2(out uvprev[0], out uvprev[1]);
                                            CurrentLine.Value(CurrentLine.NbPoints()).ParametersOnS2(out uv[0], out uv[1]);
                                        }
                                        else
                                        {
                                            CurrentLine.Value(CurrentLine.NbPoints() - 1).ParametersOnS1(out uvprev[0], out uvprev[1]);
                                            CurrentLine.Value(CurrentLine.NbPoints()).ParametersOnS1(out uv[0], out uv[1]);
                                        }

                                        double aScaleCoeff = 0.0;

                                        // Avoid finite cycle which lead to stop computing iline.
                                        if (aStatus != IntWalk_PasTropGrand)
                                        {
                                            // Make linear extrapolation.
                                            if (Math.Abs(uv[aCoordIdx - 1] - uvprev[aCoordIdx - 1]) > Constants.Resolution())
                                            {
                                                aScaleCoeff = Math.Abs((Uvap[aCoordIdx] - uv[aCoordIdx - 1])
                                                                       / (uv[aCoordIdx - 1] - uvprev[aCoordIdx - 1]));
                                            }
                                            int aFixIdx = aCoordIdx == 1 ? 2 : 1; // Fixing index;
                                            Uvap[aFixIdx] = uv[aFixIdx - 1] + (uv[aFixIdx - 1] - uvprev[aFixIdx - 1]) * aScaleCoeff;
                                            isOnDegeneratedBorder = true;
                                        }
                                    }
                                }
                            }

                            Arrive = TestArretPassage(Umult, Vmult, Uvap, I, ref Ipass);
                            if (Arrive)
                            { // reset proper parameter to test the arrow.
                                Psol = CurrentLine.Value(1);
                                if (!reversed)
                                {
                                    Psol.ParametersOnS2(out Uvap[1], out Uvap[2]);
                                }
                                else
                                {
                                    Psol.ParametersOnS1(out Uvap[1], out Uvap[2]);
                                }
                                Cadre = false;
                                // in case if there is a frame and arrival at the same time
                            }
                            else
                            { // modif jag 940615
                                if (Rajout)
                                { // test on added points
                                    ArretAjout = TestArretAjout(Func, Uvap, ref N, ref Psol);
                                    SaveN = N;
                                    if (ArretAjout)
                                    {
                                        if (N > 0)
                                        {
                                            Tgtend = lines.Value(N).IsTangentAtEnd();
                                            N = -N;
                                        }
                                        else
                                        {
                                            Tgtend = lines.Value(-N).IsTangentAtBegining();
                                        }
                                        Arrive = (wd2[I].etat == 12);
                                    }
                                }

                                if (!ArretAjout && Cadre)
                                { // test on already marked points
                                    if (CurrentLine.NbPoints() == 1)
                                    {
                                        RemoveTwoEndPoints(I);
                                        break; // cancel the line
                                    }
                                    TestArretCadre(Umult, Vmult, CurrentLine, Func, Uvap, ref N);
                                    SaveN = N;
                                    //		if (N==0) {
                                    if (N <= 0)
                                    { // jag 941017
                                        MakeWalkingPoint(2, Uvap[1], Uvap[2], Func, ref Psol);
                                        Tgtend = Func.IsTangent(); // jag 940616
                                        if (isOnDegeneratedBorder)
                                        {
                                            Tgtend = true;
                                        }
                                        N = -N;
                                    }
                                    Arrive = (wd2[I].etat == 12); // the line is open
                                }
                            }
                            aStatus = TestDeflection(Func, Arrive, Uvap, StatusPrecedent, ref NbDivision, ref PasC,
                                                     StepSign);

                            if (isOnDegeneratedBorder && Tgtend)
                            {
                                aStatus = IntWalk_ArretSurPoint;
                            }

                            StatusPrecedent = aStatus;
                            if (aStatus == IntWalk_PasTropGrand)
                            { // division of the step
                                Arrive = false;
                                ArretAjout = false;
                                Tgtend = false; // jag 940616
                                if (!reversed)
                                {
                                    previousPoint.ParametersOnS2(out Uvap[1], out Uvap[2]);
                                }
                                else
                                {
                                    previousPoint.ParametersOnS1(out Uvap[1], out Uvap[2]);
                                }
                            }
                            else if (ArretAjout || Cadre)
                            {
                                if (Arrive)
                                { // line s is open
                                    CurrentLine.AddStatusLast(false);
                                    // if (aStatus != IntWalk_ArretSurPointPrecedent)
                                    CurrentLine.AddPoint(Psol);

                                    // Remove <SaveN> from <seqAlone> and, if it is first found point,
                                    // from <seqAjout> too
                                    if (IsValidEndPoint(I, SaveN))
                                    {
                                        for (int iseq = 1; iseq <= seqAlone.Length(); iseq++)
                                        {
                                            if (seqAlone.Value(iseq) == SaveN)
                                            {
                                                seqAlone.Remove(iseq);
                                                break;
                                            }
                                        }
                                        if (CurrentLine.NbPoints() <= 3)
                                        {
                                            for (int iseq = 1; iseq <= seqAjout.Length(); iseq++)
                                            {
                                                if (seqAjout.Value(iseq) == SaveN)
                                                {
                                                    seqAjout.Remove(iseq);
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                    else
                                    {
                                        if (seqAlone.Last() == -lines.Length() - 1)
                                        {
                                            seqAlone.Remove(seqAlone.Length());
                                            seqAjout.Remove(seqAjout.Length());
                                        }
                                        RemoveTwoEndPoints(I);
                                        Arrive = false;
                                        break; // cancel the line
                                    }

                                    if (Cadre && N == 0)
                                    {
                                        Rajout = true;
                                        // seqAlone.Append(lines.Length()+1);
                                        seqAjout.Append(lines.Length() + 1);
                                    }
                                }
                                else
                                { // open
                                    wd2[I].etat = 12; // declare it open
                                    Tgtbeg = Tgtend;
                                    Tgtend = false;
                                    ArretAjout = false;
                                    StepSign = -1;
                                    StatusPrecedent = IntWalk_OK;
                                    PasC = PasSav;
                                    // Check if <Psol> has been really updated
                                    if (Arrive || Rajout || (!ArretAjout && Cadre && SaveN <= 0))
                                    {
                                        if (aStatus == IntWalk_ArretSurPointPrecedent)
                                        {
                                            CurrentLine.AddPoint(Psol);
                                            OpenLine(0, Psol, Pnts1, Func, CurrentLine);
                                        }
                                        else
                                        {
                                            OpenLine(-lines.Length() - 1, Psol, Pnts1, Func, CurrentLine);
                                        }
                                    }
                                    // Remove <SaveN> from <seqAlone> and, if it is first found point,
                                    // from <seqAjout> too
                                    if (IsValidEndPoint(I, SaveN))
                                    {
                                        for (int iseq = 1; iseq <= seqAlone.Length(); iseq++)
                                        {
                                            if (seqAlone.Value(iseq) == SaveN)
                                            {
                                                seqAlone.Remove(iseq);
                                                break;
                                            }
                                        }
                                        if (CurrentLine.NbPoints() <= 2)
                                        {
                                            for (int iseq = 1; iseq <= seqAjout.Length(); iseq++)
                                            {
                                                if (seqAjout.Value(iseq) == SaveN)
                                                {
                                                    seqAjout.Remove(iseq);
                                                    break;
                                                }
                                            }
                                        }
                                    }
                                    else
                                    {
                                        RemoveTwoEndPoints(I);
                                        break; // cancel the line
                                    }

                                    if (Cadre && N == 0)
                                    {
                                        Rajout = true;
                                        seqAjout.Append(-lines.Length() - 1);
                                    }
                                }
                            }
                            else if (aStatus == IntWalk_ArretSurPointPrecedent)
                            {
                                if (CurrentLine.NbPoints() == 1)
                                { // cancel the line
                                    Arrive = false;
                                    RemoveTwoEndPoints(I);
                                    break;
                                }
                                if (wd2[I].etat > 12)
                                { // the line should become open
                                    wd2[I].etat = 12; // declare it open
                                    ArretAjout = false;
                                    OpenLine(0, Psol, Pnts1, Func, CurrentLine);
                                    StepSign = -1;
                                    StatusPrecedent = IntWalk_OK;
                                    Arrive = false;
                                    PasC = PasSav;
                                    aNbBadRootIter = 0;
                                    Rajout = true;
                                    seqAlone.Append(-lines.Length() - 1);
                                    seqAjout.Append(-lines.Length() - 1);
                                }
                                else
                                { // line s is open
                                    Arrive = true;
                                    CurrentLine.AddStatusLast(false);
                                    Rajout = true;
                                    seqAlone.Append(lines.Length() + 1);
                                    seqAjout.Append(lines.Length() + 1);
                                }
                            }
                            else if (Arrive)
                            {
                                if (wd2[I].etat > 12)
                                { // line closed good case
                                    CurrentLine.AddStatusFirstLast(true, false, false);
                                    CurrentLine.AddPoint(CurrentLine.Value(1));
                                }
                                else if ((N > 0) && (Pnts1.Length() >= N))
                                {
                                    // point of stop given at input
                                    PathPnt = Pnts1.Value(N);
                                    CurrentLine.AddStatusLast(true, N, PathPnt);
                                    AddPointInCurrentLine(N, PathPnt, CurrentLine);
                                }
                            }
                            else if (aStatus == IntWalk_ArretSurPoint)
                            {
                                if (wd2[I].etat > 12)
                                { // line should become open
                                    wd2[I].etat = 12; // declare it open
                                    Tgtbeg = true;
                                    Tgtend = false;
                                    N = -lines.Length() - 1;
                                    Psol.SetValue(Func.Point(), reversed, Uvap[1], Uvap[2]);
                                    OpenLine(N, Psol, Pnts1, Func, CurrentLine);
                                    StepSign = -1;
                                    Rajout = true;
                                    seqAlone.Append(N);
                                    seqAjout.Append(N);
                                    StatusPrecedent = IntWalk_OK;
                                    Arrive = false;
                                    PasC = PasSav;
                                    aNbBadRootIter = 0;
                                }
                                else
                                {
                                    Arrive = true;
                                    if (Ipass != 0)
                                    { // point of passage, point of stop
                                        PathPnt = Pnts1.Value(Ipass);
                                        CurrentLine.AddStatusLast(true, Ipass, PathPnt);
                                        AddPointInCurrentLine(Ipass, PathPnt, CurrentLine);
                                    }
                                    else
                                    {
                                        CurrentLine.AddStatusLast(false);
                                        IntSurf_PntOn2S newP = new();
                                        newP.SetValue(Func.Point(), reversed, Uvap[1], Uvap[2]);
                                        CurrentLine.AddPoint(newP);
                                        Rajout = true;
                                        seqAlone.Append(lines.Length() + 1);
                                        seqAjout.Append(lines.Length() + 1);
                                    }
                                }
                            }
                            else if (aStatus == IntWalk_OK)
                            {
                                if (Ipass != 0)
                                {
                                    CurrentLine.AddIndexPassing(Ipass);
                                }
                                previousPoint.SetValue(Func.Point(), reversed, Uvap[1], Uvap[2]);
                                previousd3d = Func.Direction3d();
                                previousd2d = Func.Direction2d();
                                CurrentLine.AddPoint(previousPoint);
                            }
                            else if (aStatus == IntWalk_PointConfondu)
                            {
                                aNbIter--;
                            }
                        }
                    }
                    else
                    { // no numerical solution NotDone
                        PasC = PasC / 2.0;
                        PasCu = Math.Abs(PasC * previousd2d.X());
                        PasCv = Math.Abs(PasC * previousd2d.Y());

                        if (PasCu <= tolerance[1] && PasCv <= tolerance[2])
                        {
                            if (CurrentLine.NbPoints() == 1)
                            {
                                RemoveTwoEndPoints(I);
                                break; // cancel the line
                            }
                            if (wd2[I].etat > 12)
                            { // the line should become open
                                wd2[I].etat = 12; // declare it open
                                ArretAjout = false;
                                OpenLine(0, Psol, Pnts1, Func, CurrentLine);
                                StepSign = -1;
                                StatusPrecedent = IntWalk_OK;
                                Arrive = false;
                                PasC = PasSav;
                                aNbBadRootIter = 0;
                                Rajout = true;
                                seqAlone.Append(-lines.Length() - 1);
                                seqAjout.Append(-lines.Length() - 1);
                            }
                            else
                            { // line s is open
                                Arrive = true;
                                CurrentLine.AddStatusLast(false);
                                Tgtend = true;
                                Rajout = true;
                                seqAlone.Append(lines.Length() + 1);
                                seqAjout.Append(lines.Length() + 1);
                            }
                        }
                    }

                    if (aNbIter < 0)
                    {
                        break;
                    }
                } // end of started line
                if (Arrive)
                {
                    CurrentLine.SetTangencyAtBegining(Tgtbeg);
                    CurrentLine.SetTangencyAtEnd(Tgtend);

                    lines.Append(CurrentLine);
                    wd2[I].etat = -wd2[I].etat; // mark point as processed
                }
            } // end of processing of start point
        } // end of all start points
    }

    // IntWalk_IWalking_5.gxx

    /// <summary>
    /// Checks the step of advancement and computes the next one: confused points, 3D and 2D angles too great, tangency,
    /// change of side, then the step from the deflection, adjusted by the previous steps.
    /// </summary>
    protected IntWalk_StatusDeflection TestDeflection(TheIWFunction sp, bool Finished, math_Vector UV,
                                                      IntWalk_StatusDeflection StatusPrecedent, ref int NbDivision,
                                                      ref double Step, int StepSign)
    {
        IntWalk_StatusDeflection aStatus = TestDeflectionImpl(sp, Finished, UV, StatusPrecedent, ref NbDivision,
                                                              ref Step, StepSign);
        if ((StepTrace ?? ThreadStepTrace) is { } aTrace)
        {
            IntSurf_PntOn2S aPoint = new();
            aPoint.SetValue(sp.Point(), reversed, UV[1], UV[2]);
            aTrace(aStatus, aPoint);
        }
        return aStatus;
    }

    private IntWalk_StatusDeflection TestDeflectionImpl(TheIWFunction sp, bool Finished, math_Vector UV,
                                                        IntWalk_StatusDeflection StatusPrecedent, ref int NbDivision,
                                                        ref double Step, int StepSign)
    {
        // Check the step of advancement, AND recalculate this step :
        //
        // 1) test point confused
        //     if yes other tests are not done
        // 2) test angle 3d too great
        //     if yes divide the step and leave
        //     angle3d = angle ((previous point, calculated point),
        //                       previous tangent)
        // 3) check step of advancement in 2d
        // 4) test point confused
        // 5) test angle 2d too great
        // 6) test point of tangency
        //     if yes leave
        // 7) calculate the tangent by u,v of the section
        // 8) test angle 3d too great
        //     angle3d = angle ((previous point, calculated point),
        //                       new tangent)
        // 9) test angle 2d too great
        // 10) test change of side (pass the tangent point not knowing it)
        // 11) calculate the step of advancement depending on the vector
        // 12) adjust the step depending on the previous steps

        IntWalk_StatusDeflection aStatus = IntWalk_OK;

        //---------------------------------------------------------------------------------
        //-- lbr le 4 Avril 95 : it is possible that the status returns points confused
        //-- if epsilon is great enough (1e-11). In this case one loops
        //-- without ever changing the values sent to Rsnld.
        //---------------------------------------------------------------------------------
        double Paramu = 0.0, Paramv = 0.0;

        if (!reversed)
        {
            previousPoint.ParametersOnS2(out Paramu, out Paramv);
        }
        else
        {
            previousPoint.ParametersOnS1(out Paramu, out Paramv);
        }

        double Du = UV[1] - Paramu;
        double Dv = UV[2] - Paramv;
        double Duv = Du * Du + Dv * Dv;

        Vector3D Corde = new(previousPoint.Value(), sp.Point());

        double Norme = Corde.SquareMagnitude();

        if ((Norme <= 4.0 * global::Kerf.Engine.Foundation.Precision.SquareConfusion())
            && ((Duv <= global::Kerf.Engine.Foundation.Precision.SquarePConfusion()) || (StatusPrecedent != IntWalk_OK)))
        { // the square is already taken in the constructor
            aStatus = IntWalk_PointConfondu;
            if (StatusPrecedent == IntWalk_PasTropGrand)
            {
                return IntWalk_ArretSurPointPrecedent;
            }
        }
        else
        {
            double Cosi = Corde * previousd3d;
            double Cosi2 = 0.0;

            if (Cosi * StepSign >= 0.0)
            { // angle 3d <= pi/2 !!!!
                double aDiv = previousd3d.SquareMagnitude() * Norme;
                if (aDiv == 0)
                {
                    return aStatus;
                }
                Cosi2 = Cosi * Cosi / aDiv;
            }
            if (Cosi2 < CosRef3D)
            { // angle 3d too great
                Step = Step / 2.0;
                double StepU = Math.Abs(Step * previousd2d.X()), StepV = Math.Abs(Step * previousd2d.Y());
                if (StepU < tolerance[1] && StepV < tolerance[2])
                {
                    aStatus = IntWalk_ArretSurPointPrecedent;
                }
                else
                {
                    aStatus = IntWalk_PasTropGrand;
                }
                return aStatus;
            }
        }

        double aMinTolU = 0.1 * Math.Abs(Step * previousd2d.X()), aMinTolV = 0.1 * Math.Abs(Step * previousd2d.Y());
        double aTolU = (aMinTolU > 0.0) ? Math.Min(tolerance[1], aMinTolU) : tolerance[1],
               aTolV = (aMinTolV > 0.0) ? Math.Min(tolerance[2], aMinTolV) : tolerance[2];

        // If aMinTolU==0.0 then (std::abs(Du) < aMinTolU) is equivalent of (std::abs(Du) < 0.0).
        // It is impossible. Therefore, this case should be processed separately.
        // Analogically for aMinTolV.

        if ((Math.Abs(Du) < aTolU) && (Math.Abs(Dv) < aTolV))
        {
            // Thin shapes (for which Ulast-Ufirst or/and Vlast-Vfirst is quite small)
            // exists (see bug #25820). In this case, step is quite small too.
            // Nevertheless, it not always means that we mark time. Therefore, Du and Dv
            // must consider step (aMinTolU and aMinTolV parameters).

            return IntWalk_ArretSurPointPrecedent; // confused point 2d
        }

        {
            double Cosi = StepSign * (Du * previousd2d.X() + Dv * previousd2d.Y());

            if (Cosi < 0 && aStatus == IntWalk_PointConfondu)
            {
                return IntWalk_ArretSurPointPrecedent; // leave as step back with confused point
            }

            if (sp.IsTangent())
            {
                return IntWalk_ArretSurPoint;
            }

            // if during routing one has subdivided more than  MaxDivision for each
            // previous step, bug on the square; do nothing (experience U4)

            if ((NbDivision < MaxDivision) && (aStatus != IntWalk_PointConfondu)
                && (StatusPrecedent != IntWalk_PointConfondu))
            {
                double Cosi2 = Cosi * Cosi / Duv;
                if (Cosi2 < CosRef2D || Cosi < 0)
                {
                    Step = Step / 2.0;
                    double StepU = Math.Abs(Step * previousd2d.X()), StepV = Math.Abs(Step * previousd2d.Y());

                    if (StepU < tolerance[1] && StepV < tolerance[2])
                    {
                        aStatus = IntWalk_ArretSurPointPrecedent;
                    }
                    else
                    {
                        aStatus = IntWalk_PasTropGrand;
                    }
                    NbDivision = NbDivision + 1;
                    return aStatus;
                }

                Cosi = Corde * sp.Direction3d();
                Cosi2 = Cosi * Cosi / sp.Direction3d().SquareMagnitude() / Norme;
                if (Cosi2 < CosRef3D)
                { // angle 3d too great
                    Step = Step / 2.0;
                    double StepU = Math.Abs(Step * previousd2d.X()), StepV = Math.Abs(Step * previousd2d.Y());
                    if (StepU < tolerance[1] && StepV < tolerance[2])
                    {
                        aStatus = IntWalk_ArretSurPoint;
                    }
                    else
                    {
                        aStatus = IntWalk_PasTropGrand;
                    }
                    return aStatus;
                }
                Cosi = Du * sp.Direction2d().X() + Dv * sp.Direction2d().Y();
                Cosi2 = Cosi * Cosi / Duv;
                if (Cosi2 < CosRef2D || sp.Direction2d() * previousd2d < 0)
                {
                    // angle 2d too great or change the side
                    Step = Step / 2.0;
                    double StepU = Math.Abs(Step * previousd2d.X()), StepV = Math.Abs(Step * previousd2d.Y());
                    if (StepU < tolerance[1] && StepV < tolerance[2])
                    {
                        aStatus = IntWalk_ArretSurPointPrecedent;
                    }
                    else
                    {
                        aStatus = IntWalk_PasTropGrand;
                    }
                    return aStatus;
                }
            }
        }

        if (!Finished)
        {
            if (aStatus == IntWalk_PointConfondu)
            {
                double StepU = Math.Min(Math.Abs(1.5 * Du), pas * (UM - Um)),
                       StepV = Math.Min(Math.Abs(1.5 * Dv), pas * (VM - Vm));

                double d2dx = Math.Abs(previousd2d.X());
                double d2dy = Math.Abs(previousd2d.Y());

                if (d2dx < tolerance[1])
                {
                    Step = StepV / d2dy;
                }
                else if (d2dy < tolerance[2])
                {
                    Step = StepU / d2dx;
                }
                else
                {
                    Step = Math.Min(StepU / d2dx, StepV / d2dy);
                }
            }
            else
            {
                //   estimate the current vector.
                //   if vector/2<=current vector<= vector it is considered that the criterion
                //   is observed.
                //   otherwise adjust the step depending on the previous step

                // Direct calculation :
                // POnCurv=(((p1+p2)/2.+(p2+p3)/2.)/2. + ((p2+p3)/2.+(p3+P4)/2.)/2.)/2.
                // either POnCurv = p1/8. + 3.p2/8. + 3.p3/8. + p4/8.
                // Or p2 = p1 + lambda*d1 et p3 = p4 - lambda*d4
                // So POnCurv = (p1 + p4)/2. + 3.*(lambda d1 - lambda d4)/8.
                // Calculate the deviation with (p1+p4)/2. . So it is just necessary to calculate
                // the norm (square) of 3.*lambda (d1 - d4)/8.
                // either the norm of :
                //    3.*(std::sqrt(Norme)/3.)*StepSign*(d1-d4)/8.
                // which produces, taking the square :
                //         Norme * (d1-d4).SquareMagnitude()/64.

                double FlecheCourante =
                    (previousd3d.Normalized().XYZ() - sp.Direction3d().Normalized().XYZ()).SquareModulus() * Norme / 64.0;

                //      if (FlecheCourante <= 0.5*fleche) {
                if (FlecheCourante <= 0.25 * fleche * fleche)
                {
                    double d2dx = Math.Abs(sp.Direction2d().X());
                    double d2dy = Math.Abs(sp.Direction2d().Y());

                    double StepU = Math.Min(Math.Abs(1.5 * Du), pas * (UM - Um)),
                           StepV = Math.Min(Math.Abs(1.5 * Dv), pas * (VM - Vm));

                    if (d2dx < tolerance[1])
                    {
                        Step = StepV / d2dy;
                    }
                    else if (d2dy < tolerance[2])
                    {
                        Step = StepU / d2dx;
                    }
                    else
                    {
                        Step = Math.Min(StepU / d2dx, StepV / d2dy);
                    }
                }
                else
                {
                    // if (FlecheCourante > fleche) {  // step too great
                    if (FlecheCourante > fleche * fleche)
                    { // step too great
                        Step = Step / 2.0;
                        double StepU = Math.Abs(Step * previousd2d.X()), StepV = Math.Abs(Step * previousd2d.Y());

                        if (StepU < tolerance[1] && StepV < tolerance[2])
                        {
                            aStatus = IntWalk_ArretSurPointPrecedent;
                        }
                        else
                        {
                            aStatus = IntWalk_PasTropGrand;
                        }
                    }
                    else
                    {
                        double d2dx = Math.Abs(sp.Direction2d().X());
                        double d2dy = Math.Abs(sp.Direction2d().Y());

                        double StepU = Math.Min(Math.Abs(1.5 * Du), pas * (UM - Um)),
                               StepV = Math.Min(Math.Abs(1.5 * Dv), pas * (VM - Vm));

                        if (d2dx < tolerance[1])
                        {
                            Step = Math.Min(Step, StepV / d2dy);
                        }
                        else if (d2dy < tolerance[2])
                        {
                            Step = Math.Min(Step, StepU / d2dx);
                        }
                        else
                        {
                            Step = Math.Min(Step, Math.Min(StepU / d2dx, StepV / d2dy));
                        }
                    }
                }
            }
        }
        return aStatus;
    }

    // IntWalk_IWalking_6.gxx

    protected void AddPointInCurrentLine(int N, in IntSurf_PathPoint PathPnt, TheIWLine CurrentLine)
    {
        IntSurf_PntOn2S Psol = new();
        Psol.SetValue(IntSurf_PathPointTool.Value3d(PathPnt), reversed, wd1[N].ustart, wd1[N].vstart);
        CurrentLine.AddPoint(Psol);
    }

    /// <summary>
    /// Makes a walking point: Case 1 or 2 from the function's last point; Case 10 + i the same after evaluating the
    /// function at (U, V). Any other case throws Standard_ConstructionError.
    /// </summary>
    protected void MakeWalkingPoint(int Case, double U, double V, TheIWFunction sp, ref IntSurf_PntOn2S Psol)
    {
        // Case == 1      : make a WalkinkPoint.
        // Case == 2      : make a WalkinkPoint.
        //                  The computation of the tangency on is done
        // Case == 10 + i : make a WalkinkPoint according to i.
        //                  but F is updated according to U and V
        // Case == other  : the exception Standard_Failure is raised.

        if ((Case == 1) || (Case == 2))
        {
            Psol.SetValue(sp.Point(), reversed, U, V);
        }
        else if (Case == 11 || Case == 12)
        {
            math_Vector UV = new(1, 2);
            math_Vector FF = new(1, 1);
            Matrix DD = new(1, 1, 1, 2);
            UV[1] = U;
            UV[2] = V;
            sp.Values(UV, FF, DD);
            MakeWalkingPoint(Case - 10, U, V, sp, ref Psol);
        }
        else
        {
            throw new ConstructionErrorException();
        }
    }

    /// <summary>Opens the line and restarts it in the other direction.</summary>
    protected void OpenLine(int N, in IntSurf_PntOn2S Psol, NCollection_Sequence<IntSurf_PathPoint> Pnts1,
                            TheIWFunction sp, TheIWLine Line)
    {
        IntSurf_PathPoint PathPnt;

        math_Vector UV = new(1, 2);
        math_Vector FF = new(1, 1);
        Matrix DD = new(1, 1, 1, 2);

        previousPoint = Line.Value(1);
        if (!reversed)
        {
            previousPoint.ParametersOnS2(out UV[1], out UV[2]);
        }
        else
        {
            previousPoint.ParametersOnS1(out UV[1], out UV[2]);
        }
        sp.Values(UV, FF, DD);
        previousd3d = sp.Direction3d();
        previousd2d = sp.Direction2d();

        if (N > 0)
        { // departure point given at input
            PathPnt = Pnts1.Value(N);
            // mark the line as open with a given stop point
            Line.AddStatusFirst(false, true, N, PathPnt);

            AddPointInCurrentLine(N, PathPnt, Line);
        }
        else
        {
            if (N < 0)
            {
                Line.AddPoint(Psol);
            }
            Line.AddStatusFirst(false, false);
            // mark the line as open without given stop point
        }
        Line.Reverse(); // inverser la ligne
        Line.SetTangentVector(previousd3d.Reversed(), Line.NbPoints());
    }

    protected bool IsValidEndPoint(int IndOfPoint, int IndOfLine)
    {
        if (PointLineLine.IsEmpty())
        {
            return true;
        }

        NCollection_List<int> aList = PointLineLine.ChangeFind(IndOfPoint);
        NCollection_List<int>.Iterator itl = new(aList);
        for (; itl.More(); itl.Next())
        {
            if (itl.Value() == IndOfLine)
            {
                aList.Remove(itl);
                return true;
            }
        }
        return false;
    }

    protected void RemoveTwoEndPoints(int IndOfPoint)
    {
        if (PointLineLine.IsBound(IndOfPoint))
        {
            int Line1 = PointLineLine.Find(IndOfPoint).First();
            int Line2 = PointLineLine.Find(IndOfPoint).Last();
            for (int iseq = 1; iseq <= seqAlone.Length(); iseq++)
            {
                if (seqAlone.Value(iseq) == Line1 || seqAlone.Value(iseq) == Line2)
                {
                    seqAlone.Remove(iseq--);
                }
            }
        }
    }

    protected bool IsPointOnLine(in Point2D theP2d, int Irang)
    {
        TheIWLine aLine = lines.Value(Math.Abs(Irang));
        for (int i = 1; i <= aLine.NbPoints(); i++)
        {
            Point2D P2d1 = aLine.Value(i).ValueOnSurface(reversed);
            if (Math.Abs(P2d1.X() - theP2d.X()) <= tolerance[1] && Math.Abs(P2d1.Y() - theP2d.Y()) <= tolerance[2])
            {
                return true;
            }
            if (i < aLine.NbPoints())
            {
                Point2D P2d2 = aLine.Value(i + 1).ValueOnSurface(reversed);
                Vector2D PP1 = new(theP2d, P2d1);
                Vector2D PP2 = new(theP2d, P2d2);
                if (PP1 * PP2 < 0)
                {
                    return true;
                }
            }
        }
        return false;
    }

    /// <summary>
    /// True if <paramref name="thePOn2S"/> is on one of the lines already computed: it is projected on the nearest
    /// segment of each line, the projection refined with <paramref name="theSolver"/>, and compared.
    /// </summary>
    protected bool IsPointOnLine(in IntSurf_PntOn2S thePOn2S, math_Vector theInfBounds, math_Vector theSupBounds,
                                 FunctionSetRoot theSolver, TheIWFunction theFunc)
    {
        double eps = Epsilon(1.0);
        Point3D aP3d = thePOn2S.Value();

        for (int aLIdx = 1; aLIdx <= lines.Length(); aLIdx++)
        {
            IntSurf_LineOn2S aL = lines.Value(aLIdx).Line();

            if (aL.IsOutBox(aP3d))
            {
                continue;
            }

            // Look for the nearest segment
            double aUMin = 0.0, aVMin = 0.0;
            double aMinSqDist = RealLast();
            for (int aPtIdx = 1; aPtIdx < aL.NbPoints(); aPtIdx++)
            {
                Point3D aP1 = aL.Value(aPtIdx).Value();
                Point3D aP2 = aL.Value(aPtIdx + 1).Value();

                Xyz aP1P = aP3d.XYZ() - aP1.XYZ();
                Xyz aP1P2 = aP2.XYZ() - aP1.XYZ();

                double aSq12 = aP1P2.SquareModulus();

                if (aSq12 < Constants.Resolution())
                {
                    continue;
                }

                double aDP = aP1P.Dot(aP1P2);

                double aSqD = RealLast();
                if (aDP < 0.0)
                {
                    continue;
                }
                else if (aDP > aSq12)
                {
                    continue;
                }
                else
                {
                    aSqD = aP1P.CrossSquareMagnitude(aP1P2) / aSq12;
                }

                if (aSqD < aMinSqDist)
                {
                    aMinSqDist = aSqD;

                    double aL1 = aDP / aSq12;
                    double aL2 = 1.0 - aL1;

                    if (aL1 < eps || aL2 < eps)
                    {
                        return true;
                    }

                    aL.Value(aPtIdx).ParametersOnSurface(reversed, out double aU1, out double aV1);
                    aL.Value(aPtIdx + 1).ParametersOnSurface(reversed, out double aU2, out double aV2);

                    aUMin = aL1 * aU2 + aL2 * aU1;
                    aVMin = aL1 * aV2 + aL2 * aV1;
                }
            }

            if (aMinSqDist > global::Kerf.Engine.Foundation.Precision.Infinite())
            {
                continue;
            }

            math_Vector aVecPrms = new(1, 2);
            aVecPrms[1] = aUMin;
            aVecPrms[2] = aVMin;
            theSolver.Perform(theFunc, aVecPrms, theInfBounds, theSupBounds);
            if (!theSolver.IsDone)
            {
                continue;
            }

            theSolver.Root(aVecPrms);

            Point3D aPa = theFunc.PSurface().Value(aUMin, aVMin), aPb = theFunc.PSurface().Value(aVecPrms[1], aVecPrms[2]);
            double aSqD1 = aPb.SquareDistance(aP3d);
            double aSqD2 = aPa.SquareDistance(aPb);

            if (aSqD1 < 4.0 * aSqD2)
            {
                return true;
            }
        }

        return false;
    }
}
