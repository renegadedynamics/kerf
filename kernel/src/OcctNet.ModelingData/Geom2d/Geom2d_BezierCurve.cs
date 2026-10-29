// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_BezierCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_BezierCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// A rational or non-rational Bezier curve in the plane, defined by 2 to MaxDegree() + 1 poles and, if
/// rational, one positive weight per pole. The degree is the number of poles minus 1; the parameter range is
/// [0, 1]; the curve passes through its first and last poles, and is infinitely smooth.
/// </summary>
/// <remarks>
/// <para>
/// OCCT returns its pole, weight and knot arrays by const reference. The port returns them to callers as
/// <see cref="PackedList{T}"/> copies (<see cref="Poles()"/>, <see cref="Weights()"/>, ...), and gives ported
/// code the arrays themselves through internal <c>...Ref()</c> accessors, which must not be modified
/// (decision 701).
/// </para>
/// <para>
/// <see cref="Resolution"/> caches a value in the curve, as in OCCT; on a shared (frozen) curve the cache is
/// filled at most once per value and is safe to read from several threads.
/// </para>
/// </remarks>
public class Geom2d_BezierCurve : Geom2d_BoundedCurve
{
    private NCollection_Array1<gp_Pnt2d> myPoles = new NCollection_Array1<gp_Pnt2d>();
    private NCollection_Array1<double> myWeights = new NCollection_Array1<double>();
    private Geom2dEval_RepCurveDesc.Base? myEvalRep;
    private bool myRational = false;
    private bool myClosed = false;
    private double myMaxDerivInv = 0.0;
    private bool myMaxDerivInvOk = false;

    /// <summary>True if an evaluation representation is attached.</summary>
    public bool HasEvalRepresentation() => myEvalRep is not null;

    /// <summary>The attached evaluation representation, or null. Internal: descriptors are mutable (decision 507).</summary>
    internal Geom2dEval_RepCurveDesc.Base? EvalRepresentation() => myEvalRep;

    /// <summary>Attaches an evaluation representation (null detaches it).</summary>
    /// <exception cref="Standard_ProgramError">The descriptor is invalid.</exception>
    internal void SetEvalRepresentation(Geom2dEval_RepCurveDesc.Base? theDesc)
    {
        CheckMutable();
        Geom2dEval_RepUtils.ValidateCurveDesc(theDesc, this);
        myEvalRep = theDesc;
    }

    /// <summary>Detaches the evaluation representation.</summary>
    internal void ClearEvalRepresentation()
    {
        CheckMutable();
        myEvalRep = null;
    }

    private static bool Rational(NCollection_Array1<double> W)
    {
        int i, n = W.Length();
        bool rat = false;
        for (i = 1; i < n; i++)
        {
            rat = Math.Abs(W[i] - W[i + 1]) > gp.Resolution();
            if (rat)
            {
                break;
            }
        }
        return rat;
    }

    /// <summary>The non-rational Bezier curve of poles CurvePoles.</summary>
    /// <exception cref="Standard_ConstructionError">Fewer than 2 or more than MaxDegree() + 1 poles.</exception>
    public Geom2d_BezierCurve(NCollection_Array1<gp_Pnt2d> CurvePoles)
    {
        int nbpoles = CurvePoles.Length();
        if (nbpoles < 2 || nbpoles > (Geom2d_BezierCurve.MaxDegree() + 1))
        {
            throw new Standard_ConstructionError();
        }
        // init non rational
        init(CurvePoles, null);
    }

    /// <summary>
    /// The rational Bezier curve of poles CurvePoles and weights PoleWeights (non-rational if all the weights
    /// are equal).
    /// </summary>
    /// <exception cref="Standard_ConstructionError">
    /// Fewer than 2 or more than MaxDegree() + 1 poles, a weight count different from the pole count, or a
    /// weight not above gp::Resolution().
    /// </exception>
    public Geom2d_BezierCurve(NCollection_Array1<gp_Pnt2d> CurvePoles, NCollection_Array1<double> PoleWeights)
    {
        int nbpoles = CurvePoles.Length();
        if (nbpoles < 2 || nbpoles > (Geom2d_BezierCurve.MaxDegree() + 1))
        {
            throw new Standard_ConstructionError();
        }

        if (PoleWeights.Length() != nbpoles)
        {
            throw new Standard_ConstructionError();
        }

        for (int i = 1; i <= nbpoles; i++)
        {
            // OCCT: indexes the weights from 1 whatever their lower bound (occt-issues.md).
            if (PoleWeights[i] <= gp.Resolution())
            {
                throw new Standard_ConstructionError();
            }
        }

        // check really rational
        if (Rational(PoleWeights))
        {
            init(CurvePoles, PoleWeights);
        }
        else
        {
            init(CurvePoles, null);
        }
    }

    /// <summary>A copy of theOther (its evaluation representation deep-copied), not frozen.</summary>
    public Geom2d_BezierCurve(Geom2d_BezierCurve theOther)
    {
        myPoles = new NCollection_Array1<gp_Pnt2d>(theOther.myPoles);
        myWeights = theOther.myRational ? new NCollection_Array1<double>(theOther.myWeights)
                                        : BSplCLib.UnitWeights(theOther.myPoles.Length());
        myEvalRep = Geom2dEval_RepUtils.CloneCurveDesc(theOther.myEvalRep);
        myRational = theOther.myRational;
        myClosed = theOther.myClosed;
        myMaxDerivInvOk = false;
    }

    /// <summary>Raises the degree to Degree, keeping the shape of the curve.</summary>
    /// <exception cref="Standard_ConstructionError">Degree is below the current degree or above MaxDegree().</exception>
    internal void Increase(int Deg)
    {
        CheckMutable();
        if (Deg == Degree())
        {
            return;
        }

        Standard_ConstructionError_Raise_if(Deg < Degree() || Deg > Geom2d_BezierCurve.MaxDegree(),
                                            "Geom2d_BezierCurve::Increase");

        NCollection_Array1<gp_Pnt2d> npoles = new NCollection_Array1<gp_Pnt2d>(1, Deg + 1);
        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, 2);
        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, 2);

        if (IsRational())
        {
            NCollection_Array1<double> nweights = new NCollection_Array1<double>(1, Deg + 1);
            BSplCLib.IncreaseDegree(Degree(),
                                    Deg,
                                    false,
                                    myPoles,
                                    myWeights,
                                    KnotsRef(),
                                    MultiplicitiesRef(),
                                    npoles,
                                    nweights,
                                    nknots,
                                    nmults);
            init(npoles, nweights);
        }
        else
        {
            BSplCLib.IncreaseDegree(Degree(),
                                    Deg,
                                    false,
                                    myPoles,
                                    BSplCLib.NoWeights(),
                                    KnotsRef(),
                                    MultiplicitiesRef(),
                                    npoles,
                                    BSplCLib.NoWeights(),
                                    nknots,
                                    nmults);
            init(npoles, null);
        }
    }

    /// <summary>The maximum degree of a Bezier curve: 25.</summary>
    public static int MaxDegree() => BSplCLib.MaxDegree();

    /// <summary>Inserts the pole P (with the weight Weight) after the pole of index Index (0 inserts first).</summary>
    /// <exception cref="Standard_ConstructionError">
    /// The curve already has MaxDegree() poles, or Weight is not above gp::Resolution().
    /// </exception>
    /// <exception cref="Standard_OutOfRange">Index is outside [0, NbPoles()].</exception>
    internal void InsertPoleAfter(int Index, in gp_Pnt2d P, double Weight = 1.0)
    {
        CheckMutable();
        int nbpoles = NbPoles();

        Standard_ConstructionError_Raise_if(nbpoles >= Geom2d_BezierCurve.MaxDegree()
                                              || Weight <= gp.Resolution(),
                                            "Geom2d_BezierCurve::InsertPoleAfter");

        Standard_OutOfRange_Raise_if(Index < 0 || Index > nbpoles, "Geom2d_BezierCurve::InsertPoleAfter");

        int i;

        // Insert the pole
        NCollection_Array1<gp_Pnt2d> npoles = new NCollection_Array1<gp_Pnt2d>(1, nbpoles + 1);

        for (i = 1; i <= Index; i++)
        {
            npoles[i] = myPoles[i];
        }

        npoles[Index + 1] = P;

        for (i = Index + 1; i <= nbpoles; i++)
        {
            npoles[i + 1] = myPoles[i];
        }

        // Insert the weight
        bool rat = IsRational() || Math.Abs(Weight - 1.0) > gp.Resolution();

        if (rat)
        {
            NCollection_Array1<double> nweights = new NCollection_Array1<double>(1, nbpoles + 1);

            for (i = 1; i <= Index; i++)
            {
                if (IsRational())
                {
                    nweights[i] = myWeights[i];
                }
                else
                {
                    nweights[i] = 1.0;
                }
            }

            nweights[Index + 1] = Weight;

            for (i = Index + 1; i <= nbpoles; i++)
            {
                if (IsRational())
                {
                    nweights[i + 1] = myWeights[i];
                }
                else
                {
                    nweights[i + 1] = 1.0;
                }
            }

            init(npoles, nweights);
        }
        else
        {
            init(npoles, null);
        }
    }

    /// <summary>Inserts the pole P (with the weight Weight) before the pole of index Index.</summary>
    /// <exception cref="Standard_ConstructionError">
    /// The curve already has MaxDegree() poles, or Weight is not above gp::Resolution().
    /// </exception>
    /// <exception cref="Standard_OutOfRange">Index is outside [1, NbPoles() + 1].</exception>
    internal void InsertPoleBefore(int Index, in gp_Pnt2d P, double Weight = 1.0)
    {
        CheckMutable();
        InsertPoleAfter(Index - 1, P, Weight);
    }

    /// <summary>Removes the pole of index Index.</summary>
    /// <exception cref="Standard_ConstructionError">The curve has only 2 poles.</exception>
    /// <exception cref="Standard_OutOfRange">Index is outside [1, NbPoles()].</exception>
    internal void RemovePole(int Index)
    {
        CheckMutable();
        int nbpoles = NbPoles();

        Standard_ConstructionError_Raise_if(nbpoles <= 2, "Geom2d_BezierCurve::RemovePole");

        Standard_OutOfRange_Raise_if(Index < 1 || Index > nbpoles, "Geom2d_BezierCurve::RemovePole");

        int i;

        // Remove the pole
        NCollection_Array1<gp_Pnt2d> npoles = new NCollection_Array1<gp_Pnt2d>(1, nbpoles - 1);

        for (i = 1; i < Index; i++)
        {
            npoles[i] = myPoles[i];
        }

        for (i = Index + 1; i <= nbpoles; i++)
        {
            npoles[i - 1] = myPoles[i];
        }

        // Remove the weight
        if (IsRational())
        {
            NCollection_Array1<double> nweights = new NCollection_Array1<double>(1, nbpoles - 1);

            for (i = 1; i < Index; i++)
            {
                nweights[i] = myWeights[i];
            }

            for (i = Index + 1; i <= nbpoles; i++)
            {
                nweights[i - 1] = myWeights[i];
            }

            init(npoles, nweights);
        }
        else
        {
            init(npoles, null);
        }
    }

    /// <summary>Reverses the order of the poles (and weights).</summary>
    protected internal sealed override void Reverse()
    {
        CheckMutable();
        gp_Pnt2d P;
        int i, nbpoles = NbPoles();

        // reverse poles
        for (i = 1; i <= nbpoles / 2; i++)
        {
            P = myPoles[i];
            myPoles[i] = myPoles[nbpoles - i + 1];
            myPoles[nbpoles - i + 1] = P;
        }

        // reverse weights
        if (IsRational())
        {
            double w;
            for (i = 1; i <= nbpoles / 2; i++)
            {
                w = myWeights[i];
                myWeights[i] = myWeights[nbpoles - i + 1];
                myWeights[nbpoles - i + 1] = w;
            }
        }
        myMaxDerivInvOk = false;
        // OCCT: keeps the evaluation representation, as SetPole, SetWeight and Segment do; only init and Transform
        // drop it (occt-issues.md).
    }

    /// <summary>1 - U.</summary>
    public sealed override double ReversedParameter(double U) => (1.0 - U);

    /// <summary>
    /// Reparametrizes the portion of the curve between U1 and U2 onto [0, 1]. U1 &gt; U2 reverses the curve;
    /// U1 and U2 may lie outside [0, 1] (extrapolation).
    /// </summary>
    internal void Segment(double U1, double U2)
    {
        CheckMutable();
        myClosed = (Math.Abs(Value(U1).Distance(Value(U2))) <= gp.Resolution());

        NCollection_Array1<gp_Pnt2d> coeffs = new NCollection_Array1<gp_Pnt2d>(1, myPoles.Length());
        if (IsRational())
        {
            NCollection_Array1<double> wcoeffs = new NCollection_Array1<double>(1, myPoles.Length());
            BSplCLib.BuildCache(0.0,
                                1.0,
                                false,
                                Degree(),
                                KnotSequenceRef(),
                                myPoles,
                                myWeights,
                                coeffs,
                                wcoeffs);
            PLib.Trimming(U1, U2, coeffs, wcoeffs);
            PLib.CoefficientsPoles(coeffs, wcoeffs, myPoles, myWeights);
        }
        else
        {
            BSplCLib.BuildCache(0.0,
                                1.0,
                                false,
                                Degree(),
                                KnotSequenceRef(),
                                myPoles,
                                BSplCLib.NoWeights(),
                                coeffs,
                                BSplCLib.NoWeights());
            PLib.Trimming(U1, U2, coeffs, PLib.NoWeights());
            PLib.CoefficientsPoles(coeffs, PLib.NoWeights(), myPoles, PLib.NoWeights());
        }
        myMaxDerivInvOk = false;
    }

    /// <summary>Replaces the pole of index Index by P.</summary>
    /// <exception cref="Standard_OutOfRange">Index is outside [1, NbPoles()].</exception>
    internal void SetPole(int Index, in gp_Pnt2d P)
    {
        CheckMutable();
        Standard_OutOfRange_Raise_if(Index < 1 || Index > NbPoles(), "Geom2d_BezierCurve::SetPole");
        myPoles[Index] = P;

        if (Index == 1 || Index == myPoles.Length())
        {
            myClosed = (myPoles[1].Distance(myPoles[NbPoles()]) <= gp.Resolution());
        }
        myMaxDerivInvOk = false;
    }

    /// <summary>Replaces the pole of index Index by P, and its weight by Weight.</summary>
    /// <exception cref="Standard_OutOfRange">Index is outside [1, NbPoles()].</exception>
    /// <exception cref="Standard_ConstructionError">Weight is not above gp::Resolution().</exception>
    internal void SetPole(int Index, in gp_Pnt2d P, double Weight)
    {
        CheckMutable();
        SetPole(Index, P);
        SetWeight(Index, Weight);
    }

    /// <summary>Changes the weight of the pole of index Index; the curve may become rational or non-rational.</summary>
    /// <exception cref="Standard_OutOfRange">Index is outside [1, NbPoles()].</exception>
    /// <exception cref="Standard_ConstructionError">Weight is not above gp::Resolution().</exception>
    internal void SetWeight(int Index, double Weight)
    {
        CheckMutable();
        int nbpoles = NbPoles();

        Standard_OutOfRange_Raise_if(Index < 1 || Index > nbpoles, "Geom2d_BezierCurve::SetWeight");
        Standard_ConstructionError_Raise_if(Weight <= gp.Resolution(), "Geom2d_BezierCurve::SetWeight");

        // compute new rationality
        bool wasrat = IsRational();
        if (!wasrat)
        {
            // a weight of 1. does not turn to rational
            if (Math.Abs(Weight - 1.0) <= gp.Resolution())
            {
                return;
            }

            // Becoming rational: copy non-owning view to owned array.
            myWeights = new NCollection_Array1<double>(myWeights);
        }

        myWeights[Index] = Weight;

        // is it turning into non rational
        if (wasrat && !Rational(myWeights))
        {
            myRational = false;
            myWeights = BSplCLib.UnitWeights(nbpoles);
        }
        else
        {
            myRational = true;
        }

        myMaxDerivInvOk = false;
    }

    /// <summary>True if the first and last poles are within gp::Resolution().</summary>
    public sealed override bool IsClosed() => myClosed;

    /// <summary>True: a Bezier curve is CN.</summary>
    public sealed override bool IsCN(int N) => true;

    /// <summary>False.</summary>
    public sealed override bool IsPeriodic() => false;

    /// <summary>True if the weights are not all equal.</summary>
    public bool IsRational() => myRational;

    /// <summary>CN.</summary>
    public sealed override GeomAbs_Shape Continuity() => GeomAbs_CN;

    /// <summary>The degree: NbPoles() - 1.</summary>
    public int Degree() => myPoles.Length() - 1;

    public sealed override gp_Pnt2d EvalD0(double U)
    {
        gp_Pnt2d aEvalRepResult;
        if (Geom2dEval_RepUtils.TryEvalCurveD0(myEvalRep, U, out aEvalRepResult))
        {
            return aEvalRepResult;
        }
        gp_Pnt2d P;
        BSplCLib.D0(U, myPoles, WeightsRef(), out P);
        return P;
    }

    public sealed override ResD1 EvalD1(double U)
    {
        ResD1 aEvalRepResult;
        if (Geom2dEval_RepUtils.TryEvalCurveD1(myEvalRep, U, out aEvalRepResult))
        {
            return aEvalRepResult;
        }
        ResD1 aResult;
        BSplCLib.D1(U, myPoles, WeightsRef(), out aResult.Point, out aResult.D1);
        return aResult;
    }

    public sealed override ResD2 EvalD2(double U)
    {
        ResD2 aEvalRepResult;
        if (Geom2dEval_RepUtils.TryEvalCurveD2(myEvalRep, U, out aEvalRepResult))
        {
            return aEvalRepResult;
        }
        ResD2 aResult;
        BSplCLib.D2(U, myPoles, WeightsRef(), out aResult.Point, out aResult.D1, out aResult.D2);
        return aResult;
    }

    public sealed override ResD3 EvalD3(double U)
    {
        ResD3 aEvalRepResult;
        if (Geom2dEval_RepUtils.TryEvalCurveD3(myEvalRep, U, out aEvalRepResult))
        {
            return aEvalRepResult;
        }
        ResD3 aResult;
        BSplCLib.D3(U, myPoles, WeightsRef(), out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
        return aResult;
    }

    /// <exception cref="Geom2d_UndefinedDerivative">N &lt; 1.</exception>
    public sealed override gp_Vec2d EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Geom2d_UndefinedDerivative();
        }
        gp_Vec2d aEvalRepResult;
        if (Geom2dEval_RepUtils.TryEvalCurveDN(myEvalRep, U, N, out aEvalRepResult))
        {
            return aEvalRepResult;
        }
        gp_Vec2d V;
        BSplCLib.DN(U, N, 0, Degree(), false, myPoles, WeightsRef(), KnotsRef(), MultiplicitiesRef(), out V);
        return V;
    }

    /// <summary>The last pole.</summary>
    public sealed override gp_Pnt2d EndPoint() => myPoles[myPoles.Upper()];

    /// <summary>0.</summary>
    public sealed override double FirstParameter() => 0.0;

    /// <summary>1.</summary>
    public sealed override double LastParameter() => 1.0;

    /// <summary>The number of poles.</summary>
    public int NbPoles() => myPoles.Length();

    /// <summary>The pole of index Index.</summary>
    /// <exception cref="Standard_OutOfRange">Index is outside [1, NbPoles()].</exception>
    public gp_Pnt2d Pole(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > myPoles.Length(), "Geom2d_BezierCurve::Pole");
        return myPoles[Index];
    }

    /// <summary>Copies the poles into P (indexed as P is).</summary>
    /// <exception cref="Standard_DimensionError">P's length is not NbPoles().</exception>
    public void Poles(NCollection_Array1<gp_Pnt2d> P)
    {
        Standard_DimensionError_Raise_if(P.Length() != myPoles.Length(), "Geom2d_BezierCurve::Poles");
        P.Assign(myPoles);
    }

    /// <summary>The poles (a copy; OCCT returns a reference to its array).</summary>
    public PackedList<gp_Pnt2d> Poles() => myPoles.ToPackedList();

    /// <summary>The pole array itself, indexed from 1, for ported code: don't modify it (decision 701).</summary>
    internal NCollection_Array1<gp_Pnt2d> PolesRef() => myPoles;

    /// <summary>The first pole.</summary>
    public sealed override gp_Pnt2d StartPoint() => myPoles[1];

    /// <summary>The weight of the pole of index Index (1 if the curve is not rational).</summary>
    /// <exception cref="Standard_OutOfRange">Index is outside [1, NbPoles()].</exception>
    public double Weight(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > myPoles.Length(), "Geom2d_BezierCurve::Weight");
        if (IsRational())
        {
            return myWeights[Index];
        }
        else
        {
            return 1.0;
        }
    }

    /// <summary>Copies the weights into W (ones if the curve is not rational).</summary>
    /// <exception cref="Standard_DimensionError">W's length is not NbPoles().</exception>
    public void Weights(NCollection_Array1<double> W)
    {
        int nbpoles = NbPoles();
        Standard_DimensionError_Raise_if(W.Length() != nbpoles, "Geom2d_BezierCurve::Weights");
        if (IsRational())
        {
            W.Assign(myWeights);
        }
        else
        {
            for (int i = 1; i <= nbpoles; i++)
            {
                W[i] = 1.0;
            }
        }
    }

    /// <summary>The weights (a copy), or null if the curve is not rational (OCCT's <c>BSplCLib::NoWeights()</c>).</summary>
    public PackedList<double>? Weights() => myRational ? myWeights.ToPackedList() : null;

    /// <summary>The weight array for ported code, or null if the curve is not rational: don't modify it.</summary>
    internal NCollection_Array1<double>? WeightsRef() => myRational ? myWeights : BSplCLib.NoWeights();

    /// <summary>The weights (a copy), ones if the curve is not rational.</summary>
    public PackedList<double> WeightsArray() => myWeights.ToPackedList();

    /// <summary>The weight array (shared unit weights if not rational) for ported code: don't modify it.</summary>
    internal NCollection_Array1<double> WeightsArrayRef() => myWeights;

    /// <summary>Applies T to the poles; the evaluation representation is dropped.</summary>
    protected internal sealed override void Transform(in gp_Trsf2d T)
    {
        CheckMutable();
        int nbpoles = NbPoles();

        for (int i = 1; i <= nbpoles; i++)
        {
            myPoles[i].Transform(T);
        }

        ClearEvalRepresentation();
        myMaxDerivInvOk = false;
    }

    /// <summary>
    /// The parametric tolerance UTolerance such that |t1 - t0| &lt; UTolerance implies
    /// |C(t1) - C(t0)| &lt; ToleranceUV.
    /// </summary>
    public void Resolution(double ToleranceUV, out double UTolerance)
    {
        // OCCT.NET: a cache filled on a possibly shared curve; the flag is published after the value.
        if (!Volatile.Read(ref myMaxDerivInvOk))
        {
            double aMaxDerivInv;
            BSplCLib.Resolution(myPoles,
                                WeightsRef(),
                                myPoles.Length(),
                                KnotSequenceRef(),
                                Degree(),
                                1.0,
                                out aMaxDerivInv);
            myMaxDerivInv = aMaxDerivInv;
            Volatile.Write(ref myMaxDerivInvOk, true);
        }
        UTolerance = ToleranceUV * myMaxDerivInv;
    }

    /// <summary>A new, unfrozen curve equal to this one.</summary>
    public sealed override Geom2d_Geometry Copy() => new Geom2d_BezierCurve(this);

    /// <summary>The knots {0, 1} of the curve seen as a B-spline (a copy).</summary>
    public PackedList<double> Knots() => KnotsRef().ToPackedList();

    /// <summary>The multiplicities {Degree() + 1, Degree() + 1} of the curve seen as a B-spline (a copy).</summary>
    public PackedList<int> Multiplicities() => MultiplicitiesRef().ToPackedList();

    /// <summary>The flat knots of the curve seen as a B-spline: Degree() + 1 zeros then Degree() + 1 ones (a copy).</summary>
    public PackedList<double> KnotSequence() => KnotSequenceRef().ToPackedList();

    private static readonly NCollection_Array1<double> THE_KNOTS = NCollection_Array1<double>.From([0.0, 1.0]);

    private static readonly NCollection_Array1<int>[] THE_MULTS = buildMults();

    private static readonly NCollection_Array1<double>[] THE_FKNOTS = buildFlatKnots();

    private static NCollection_Array1<int>[] buildMults()
    {
        int THE_MAX_SIZE = BSplCLib.MaxDegree() + 1;
        NCollection_Array1<int>[] anArr = new NCollection_Array1<int>[THE_MAX_SIZE];
        for (int i = 0; i < THE_MAX_SIZE; ++i)
        {
            anArr[i] = NCollection_Array1<int>.From([i + 1, i + 1]);
        }
        return anArr;
    }

    private static NCollection_Array1<double>[] buildFlatKnots()
    {
        int THE_MAX_SIZE = BSplCLib.MaxDegree() + 1;
        NCollection_Array1<double>[] anArr = new NCollection_Array1<double>[THE_MAX_SIZE];
        anArr[0] = new NCollection_Array1<double>();
        for (int i = 1; i <= BSplCLib.MaxDegree(); ++i)
        {
            anArr[i] = NCollection_Array1<double>.From(BSplCLib.FlatBezierKnots(i));
        }
        return anArr;
    }

    /// <summary>The shared knot array {0, 1}, for ported code: don't modify it.</summary>
    internal NCollection_Array1<double> KnotsRef() => THE_KNOTS;

    /// <summary>The shared multiplicity array of this degree, for ported code: don't modify it.</summary>
    internal NCollection_Array1<int> MultiplicitiesRef()
    {
        Standard_ProgramError_Raise_if(myPoles.IsEmpty(), "Geom2d_BezierCurve: empty poles");
        return THE_MULTS[myPoles.Length() - 1];
    }

    /// <summary>The shared flat knot array of this degree, for ported code: don't modify it.</summary>
    internal NCollection_Array1<double> KnotSequenceRef()
    {
        Standard_ProgramError_Raise_if(myPoles.IsEmpty(), "Geom2d_BezierCurve: empty poles");
        return THE_FKNOTS[myPoles.Length() - 1];
    }

    /// <summary>Sets the poles and weights (null: non-rational), resetting the caches and the evaluation representation.</summary>
    protected void init(NCollection_Array1<gp_Pnt2d> thePoles, NCollection_Array1<double>? theWeights)
    {
        int nbpoles = thePoles.Length();
        // closed ?
        myClosed = thePoles[thePoles.Lower()].Distance(thePoles[thePoles.Upper()]) <= gp.Resolution();

        // set fields
        myPoles.Resize(1, nbpoles, false);
        myPoles.CopyValues(thePoles);

        if (theWeights is not null)
        {
            // OCCT.NET: never write into the shared unit weights (a resize to the same size keeps the view).
            if (!myWeights.IsDeletable())
            {
                myWeights = new NCollection_Array1<double>();
            }
            myWeights.Resize(1, nbpoles, false);
            myWeights.CopyValues(theWeights);
            myRational = Rational(myWeights);
            if (!myRational)
            {
                myWeights = BSplCLib.UnitWeights(nbpoles);
            }
        }
        else
        {
            myRational = false;
            myWeights = BSplCLib.UnitWeights(nbpoles);
        }

        myMaxDerivInv = 0.0;
        myMaxDerivInvOk = false;
        ClearEvalRepresentation();
    }

    // Non-mutating twins of the mutators (OCCT.NET)

    /// <summary>A copy with its degree raised to Deg.</summary>
    public Geom2d_BezierCurve Increased(int Deg)
    {
        Geom2d_BezierCurve aCopy = new Geom2d_BezierCurve(this);
        aCopy.Increase(Deg);
        return aCopy;
    }

    /// <summary>A copy with the pole P (weight Weight) inserted after the pole of index Index.</summary>
    public Geom2d_BezierCurve PoleInsertedAfter(int Index, in gp_Pnt2d P, double Weight = 1.0)
    {
        Geom2d_BezierCurve aCopy = new Geom2d_BezierCurve(this);
        aCopy.InsertPoleAfter(Index, P, Weight);
        return aCopy;
    }

    /// <summary>A copy with the pole P (weight Weight) inserted before the pole of index Index.</summary>
    public Geom2d_BezierCurve PoleInsertedBefore(int Index, in gp_Pnt2d P, double Weight = 1.0)
    {
        Geom2d_BezierCurve aCopy = new Geom2d_BezierCurve(this);
        aCopy.InsertPoleBefore(Index, P, Weight);
        return aCopy;
    }

    /// <summary>A copy without the pole of index Index.</summary>
    public Geom2d_BezierCurve PoleRemoved(int Index)
    {
        Geom2d_BezierCurve aCopy = new Geom2d_BezierCurve(this);
        aCopy.RemovePole(Index);
        return aCopy;
    }

    /// <summary>A copy reparametrized between U1 and U2 (see Segment).</summary>
    public Geom2d_BezierCurve Segmented(double U1, double U2)
    {
        Geom2d_BezierCurve aCopy = new Geom2d_BezierCurve(this);
        aCopy.Segment(U1, U2);
        return aCopy;
    }

    /// <summary>A copy with the pole of index Index replaced by P.</summary>
    public Geom2d_BezierCurve WithPole(int Index, in gp_Pnt2d P)
    {
        Geom2d_BezierCurve aCopy = new Geom2d_BezierCurve(this);
        aCopy.SetPole(Index, P);
        return aCopy;
    }

    /// <summary>A copy with the pole of index Index replaced by P, of weight Weight.</summary>
    public Geom2d_BezierCurve WithPole(int Index, in gp_Pnt2d P, double Weight)
    {
        Geom2d_BezierCurve aCopy = new Geom2d_BezierCurve(this);
        aCopy.SetPole(Index, P, Weight);
        return aCopy;
    }

    /// <summary>A copy with the weight of the pole of index Index changed to Weight.</summary>
    public Geom2d_BezierCurve WithWeight(int Index, double Weight)
    {
        Geom2d_BezierCurve aCopy = new Geom2d_BezierCurve(this);
        aCopy.SetWeight(Index, Weight);
        return aCopy;
    }

    /// <summary>A copy evaluated through theDesc, which is copied (so later changes to theDesc don't affect it).</summary>
    /// <exception cref="Standard_ProgramError">The descriptor is invalid.</exception>
    public Geom2d_BezierCurve WithEvalRepresentation(Geom2dEval_RepCurveDesc.Base? theDesc)
    {
        Geom2d_BezierCurve aCopy = new Geom2d_BezierCurve(this);
        Geom2dEval_RepUtils.ValidateCurveDesc(theDesc, this);
        aCopy.SetEvalRepresentation(Geom2dEval_RepUtils.CloneCurveDesc(theDesc));
        return aCopy;
    }

    /// <summary>A copy without an evaluation representation.</summary>
    public Geom2d_BezierCurve WithoutEvalRepresentation()
    {
        Geom2d_BezierCurve aCopy = new Geom2d_BezierCurve(this);
        aCopy.ClearEvalRepresentation();
        return aCopy;
    }
}
