// Port of: src/ModelingData/TKG3d/Geom/Geom_BezierCurve.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_BezierCurve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// A rational or non-rational Bezier curve: a table of poles (control points), indexed from 1, with weights when
/// rational. The degree is the number of poles minus 1, from 1 to <see cref="MaxDegree"/> (25). The parameter
/// range is [0, 1]; the first and last poles are the start and end points. A weight is zero when at most
/// gp::Resolution(), and two weights are equal when they differ by at most gp::Resolution(); a curve whose
/// weights are all equal is non-rational. A closed Bezier curve is never periodic.
/// </summary>
/// <remarks>
/// <para>
/// Arrays: <see cref="Poles()"/>, <see cref="Weights()"/>, <see cref="WeightsArray"/>, <see cref="Knots"/>,
/// <see cref="Multiplicities"/> and <see cref="KnotSequence"/> return immutable copies (OCCT returns references to
/// the curve's arrays); their index 0 is OCCT's index 1. Code inside OCCT.NET that passes the arrays on to
/// BSplCLib and the like uses the internal <c>Internal*</c> accessors, which return the stored arrays (shared:
/// don't modify).
/// </para>
/// <para>
/// The evaluation representation (<see cref="HasEvalRepresentation"/>) is set by <see cref="WithEvalRepresentation"/>,
/// which stores a copy of the descriptor.
/// </para>
/// </remarks>
public class Geom_BezierCurve : Geom_BoundedCurve
{
    private NCollection_Array1<Point3D> myPoles = new NCollection_Array1<Point3D>();
    private NCollection_Array1<double> myWeights = new NCollection_Array1<double>();
    private GeomEval_RepCurveDesc.Base? myEvalRep;
    private bool myRational = false;
    private bool myClosed = false;
    private double myMaxDerivInv = 0.0;
    private bool myMaxDerivInvOk = false;

    /// <summary>True if an evaluation representation is attached.</summary>
    public bool HasEvalRepresentation() => myEvalRep is not null;

    /// <summary>The evaluation representation descriptor (may be null). Stored, not copied: don't modify.</summary>
    internal GeomEval_RepCurveDesc.Base? EvalRepresentation() => myEvalRep;

    /// <summary>Sets a new evaluation representation, after checking it (no circular reference).</summary>
    internal void SetEvalRepresentation(GeomEval_RepCurveDesc.Base? theDesc)
    {
        CheckMutable();
        GeomEval_RepUtils.ValidateCurveDesc(theDesc, this);
        myEvalRep = theDesc;
    }

    /// <summary>Removes the evaluation representation.</summary>
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
            rat = Math.Abs(W[i] - W[i + 1]) > Constants.Resolution();
            if (rat)
            {
                break;
            }
        }
        return rat;
    }

    /// <summary>A copy of theOther, without validation (OCCT's copy constructor).</summary>
    public Geom_BezierCurve(Geom_BezierCurve theOther)
    {
        myPoles = new NCollection_Array1<Point3D>(theOther.myPoles);
        myWeights = theOther.myRational ? new NCollection_Array1<double>(theOther.myWeights)
                                        : global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(theOther.myPoles.Length());
        myEvalRep = GeomEval_RepUtils.CloneCurveDesc(theOther.myEvalRep);
        myRational = theOther.myRational;
        myClosed = theOther.myClosed;
        myMaxDerivInvOk = false;
    }

    /// <summary>A non-rational Bezier curve with the given poles.</summary>
    /// <exception cref="ConstructionErrorException">Fewer than 2 or more than MaxDegree() + 1 poles.</exception>
    public Geom_BezierCurve(NCollection_Array1<Point3D> Poles)
    {
        int nbpoles = Poles.Length();
        if (nbpoles < 2 || nbpoles > (Geom_BezierCurve.MaxDegree() + 1))
        {
            throw new ConstructionErrorException();
        }
        // init non rational
        init(Poles, null);
    }

    /// <summary>A rational Bezier curve (non-rational if all the weights are equal).</summary>
    /// <exception cref="ConstructionErrorException">
    /// Fewer than 2 or more than MaxDegree() + 1 poles, as many weights as poles, or a weight at most gp::Resolution().
    /// </exception>
    public Geom_BezierCurve(NCollection_Array1<Point3D> Poles, NCollection_Array1<double> Weights)
    {
        // copy the poles
        int nbpoles = Poles.Length();
        if (nbpoles < 2 || nbpoles > (Geom_BezierCurve.MaxDegree() + 1))
        {
            throw new ConstructionErrorException();
        }

        // check the weights
        if (Weights.Length() != nbpoles)
        {
            throw new ConstructionErrorException();
        }

        for (int i = 1; i <= nbpoles; i++)
        {
            // OCCT: indexes Weights from 1 whatever its lower bound, as Rational does (out of range otherwise)
            if (Weights[i] <= Constants.Resolution())
            {
                throw new ConstructionErrorException();
            }
        }

        // check really rational
        if (Rational(Weights))
        {
            init(Poles, Weights);
        }
        else
        {
            init(Poles, null);
        }
    }

    /// <summary>Raises the degree to Deg.</summary>
    /// <exception cref="ConstructionErrorException">Deg is less than the degree or more than MaxDegree().</exception>
    internal void Increase(int Deg)
    {
        CheckMutable();
        if (Deg == Degree())
        {
            return;
        }

        if (Deg < Degree() || Deg > Geom_BezierCurve.MaxDegree())
        {
            throw new ConstructionErrorException("Geom_BezierCurve::Increase");
        }

        int aDeg = myPoles.Length() - 1;
        NCollection_Array1<Point3D> npoles = new NCollection_Array1<Point3D>(1, Deg + 1);
        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, 2);
        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, 2);

        if (IsRational())
        {
            NCollection_Array1<double> nweights = new NCollection_Array1<double>(1, Deg + 1);
            global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(aDeg,
                                    Deg,
                                    false,
                                    myPoles,
                                    myWeights,
                                    InternalKnots(),
                                    InternalMultiplicities(),
                                    npoles,
                                    nweights,
                                    nknots,
                                    nmults);
            init(npoles, nweights);
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(aDeg,
                                    Deg,
                                    false,
                                    myPoles,
                                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                                    InternalKnots(),
                                    InternalMultiplicities(),
                                    npoles,
                                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                                    nknots,
                                    nmults);
            init(npoles, null);
        }
    }

    /// <summary>The maximum degree of a Bezier curve: 25.</summary>
    public static int MaxDegree() => global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree();

    /// <summary>Inserts P after the pole Index (weight 1 if the curve is rational).</summary>
    internal void InsertPoleAfter(int Index, in Point3D P)
    {
        CheckMutable();
        InsertPoleAfter(Index, P, 1.0);
    }

    /// <summary>Inserts P with its weight after the pole Index (0 inserts first).</summary>
    /// <exception cref="ConstructionErrorException">Too many poles, or Weight at most gp::Resolution().</exception>
    /// <exception cref="OutOfRangeException">Index is not in [0, NbPoles].</exception>
    internal void InsertPoleAfter(int Index, in Point3D P, double Weight)
    {
        CheckMutable();
        int nbpoles = NbPoles();

        // OCCT: refuses a 25-pole curve, though the constructors accept MaxDegree() + 1 = 26 poles
        if (nbpoles >= Geom_BezierCurve.MaxDegree() || Weight <= Constants.Resolution())
        {
            throw new ConstructionErrorException("Geom_BezierCurve::InsertPoleAfter");
        }

        if (Index < 0 || Index > nbpoles)
        {
            throw new OutOfRangeException("Geom_BezierCurve::InsertPoleAfter");
        }

        int i;

        // Insert the pole
        NCollection_Array1<Point3D> npoles = new NCollection_Array1<Point3D>(1, nbpoles + 1);

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
        bool rat = IsRational() || Math.Abs(Weight - 1.0) > Constants.Resolution();

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

    /// <summary>Inserts P before the pole Index (weight 1 if the curve is rational).</summary>
    internal void InsertPoleBefore(int Index, in Point3D P)
    {
        CheckMutable();
        InsertPoleAfter(Index - 1, P);
    }

    /// <summary>Inserts P with its weight before the pole Index.</summary>
    internal void InsertPoleBefore(int Index, in Point3D P, double Weight)
    {
        CheckMutable();
        InsertPoleAfter(Index - 1, P, Weight);
    }

    /// <summary>Removes the pole Index.</summary>
    /// <exception cref="ConstructionErrorException">The curve has only 2 poles.</exception>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbPoles].</exception>
    internal void RemovePole(int Index)
    {
        CheckMutable();
        int nbpoles = NbPoles();

        if (nbpoles <= 2)
        {
            throw new ConstructionErrorException("Geom_BezierCurve::RemovePole");
        }

        if (Index < 1 || Index > nbpoles)
        {
            throw new OutOfRangeException("Geom_BezierCurve::RemovePole");
        }

        int i;

        // Remove the pole
        NCollection_Array1<Point3D> npoles = new NCollection_Array1<Point3D>(1, nbpoles - 1);

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

    /// <summary>Reverses the parametrization: Value(NewU) = Value(1 - OldU).</summary>
    protected internal sealed override void Reverse()
    {
        CheckMutable();
        Point3D P;
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
    }

    /// <summary>1 - U.</summary>
    public sealed override double ReversedParameter(double U) => (1.0 - U);

    /// <summary>
    /// Keeps the part between U1 and U2 (which may be outside [0, 1]), oriented from U1 to U2, reparametrized on
    /// [0, 1].
    /// </summary>
    internal void Segment(double U1, double U2)
    {
        CheckMutable();
        myClosed = (Math.Abs(Value(U1).Distance(Value(U2))) <= global::Kerf.Engine.Foundation.Precision.Confusion());

        int aDeg = myPoles.Length() - 1;
        NCollection_Array1<Point3D> coeffs = new NCollection_Array1<Point3D>(1, myPoles.Length());
        if (IsRational())
        {
            NCollection_Array1<double> wcoeffs = new NCollection_Array1<double>(1, myPoles.Length());
            global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildCache(0.0,
                                1.0,
                                false,
                                aDeg,
                                InternalKnotSequence(),
                                myPoles,
                                myWeights,
                                coeffs,
                                wcoeffs);
            global::Kerf.Engine.Numerics.Polynomials.PLib.Trimming(U1, U2, coeffs, wcoeffs);
            global::Kerf.Engine.Numerics.Polynomials.PLib.CoefficientsPoles(coeffs, wcoeffs, myPoles, myWeights);
        }
        else
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.BuildCache(0.0,
                                1.0,
                                false,
                                aDeg,
                                InternalKnotSequence(),
                                myPoles,
                                global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                                coeffs,
                                global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights());
            global::Kerf.Engine.Numerics.Polynomials.PLib.Trimming(U1, U2, coeffs, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());
            global::Kerf.Engine.Numerics.Polynomials.PLib.CoefficientsPoles(coeffs, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights(), myPoles, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());
        }
        myMaxDerivInvOk = false;
    }

    /// <summary>Replaces the pole Index (its weight is kept).</summary>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbPoles].</exception>
    internal void SetPole(int Index, in Point3D P)
    {
        CheckMutable();
        if (Index < 1 || Index > NbPoles())
        {
            throw new OutOfRangeException("Geom_BezierCurve::SetPole");
        }

        myPoles[Index] = P;

        if (Index == 1 || Index == myPoles.Length())
        {
            myClosed = (myPoles[1].Distance(myPoles[NbPoles()]) <= global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        myMaxDerivInvOk = false;
    }

    /// <summary>Replaces the pole Index and its weight.</summary>
    internal void SetPole(int Index, in Point3D P, double Weight)
    {
        CheckMutable();
        SetPole(Index, P);
        SetWeight(Index, Weight);
    }

    /// <summary>Changes the weight of the pole Index; the curve may become rational or non-rational.</summary>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbPoles].</exception>
    /// <exception cref="ConstructionErrorException">Weight is at most gp::Resolution().</exception>
    internal void SetWeight(int Index, double Weight)
    {
        CheckMutable();
        int nbpoles = NbPoles();

        if (Index < 1 || Index > nbpoles)
        {
            throw new OutOfRangeException("Geom_BezierCurve::SetWeight");
        }
        if (Weight <= Constants.Resolution())
        {
            throw new ConstructionErrorException("Geom_BezierCurve::SetWeight");
        }

        // compute new rationality
        bool wasrat = IsRational();
        if (!wasrat)
        {
            // a weight of 1. does not turn to rational
            if (Math.Abs(Weight - 1.0) <= Constants.Resolution())
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
            myWeights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(nbpoles);
        }
        else
        {
            myRational = true;
        }
        myMaxDerivInvOk = false;
    }

    /// <summary>True if the first and last poles are within Precision::Confusion().</summary>
    public sealed override bool IsClosed() => myClosed;

    /// <summary>True: a Bezier curve is CN.</summary>
    public sealed override bool IsCN(int N) => true;

    /// <summary>False.</summary>
    public sealed override bool IsPeriodic() => false;

    /// <summary>False if all the weights are equal (within gp::Resolution()).</summary>
    public bool IsRational() => myRational;

    /// <summary>CN.</summary>
    public sealed override Continuity Continuity() => CN;

    /// <summary>The degree: the number of poles minus 1.</summary>
    public int Degree() => myPoles.Length() - 1;

    /// <summary>The point at U (U may be outside [0, 1]).</summary>
    public sealed override Point3D EvalD0(double U)
    {
        if (GeomEval_RepUtils.TryEvalCurveD0(myEvalRep, U, out Point3D aEvalRepResult))
        {
            return aEvalRepResult;
        }
        Point3D P;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(U, myPoles, InternalWeights(), out P);
        return P;
    }

    public sealed override ResD1 EvalD1(double U)
    {
        if (GeomEval_RepUtils.TryEvalCurveD1(myEvalRep, U, out ResD1 aEvalRepResult))
        {
            return aEvalRepResult;
        }
        ResD1 aResult = new ResD1();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(U, myPoles, InternalWeights(), out aResult.Point, out aResult.D1);
        return aResult;
    }

    public sealed override ResD2 EvalD2(double U)
    {
        if (GeomEval_RepUtils.TryEvalCurveD2(myEvalRep, U, out ResD2 aEvalRepResult))
        {
            return aEvalRepResult;
        }
        ResD2 aResult = new ResD2();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(U, myPoles, InternalWeights(), out aResult.Point, out aResult.D1, out aResult.D2);
        return aResult;
    }

    public sealed override ResD3 EvalD3(double U)
    {
        if (GeomEval_RepUtils.TryEvalCurveD3(myEvalRep, U, out ResD3 aEvalRepResult))
        {
            return aEvalRepResult;
        }
        ResD3 aResult = new ResD3();
        global::Kerf.Engine.Geometry.Splines.BSplCLib.D3(U, myPoles, InternalWeights(), out aResult.Point, out aResult.D1, out aResult.D2, out aResult.D3);
        return aResult;
    }

    /// <exception cref="Geom_UndefinedDerivative">N &lt; 1.</exception>
    public sealed override Vector3D EvalDN(double U, int N)
    {
        if (N < 1)
        {
            throw new Geom_UndefinedDerivative();
        }
        if (GeomEval_RepUtils.TryEvalCurveDN(myEvalRep, U, N, out Vector3D aEvalRepResult))
        {
            return aEvalRepResult;
        }
        Vector3D V;
        int aDeg = myPoles.Length() - 1;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.DN(U, N, 0, aDeg, false, myPoles, InternalWeights(), InternalKnots(), InternalMultiplicities(), out V);
        return V;
    }

    /// <summary>The first pole.</summary>
    public sealed override Point3D StartPoint() => myPoles[1];

    /// <summary>The last pole.</summary>
    public sealed override Point3D EndPoint() => myPoles[myPoles.Upper()];

    /// <summary>0.</summary>
    public sealed override double FirstParameter() => 0.0;

    /// <summary>1.</summary>
    public sealed override double LastParameter() => 1.0;

    public int NbPoles() => myPoles.Length();

    /// <summary>The pole Index.</summary>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbPoles].</exception>
    public Point3D Pole(int Index)
    {
        if (Index < 1 || Index > myPoles.Length())
        {
            throw new OutOfRangeException("Geom_BezierCurve::Pole");
        }
        return myPoles[Index];
    }

    /// <summary>Copies the poles into P (OCCT's deprecated overload).</summary>
    /// <exception cref="DimensionErrorException">P has not NbPoles elements.</exception>
    public void Poles(NCollection_Array1<Point3D> P)
    {
        if (P.Length() != myPoles.Length())
        {
            throw new DimensionErrorException("Geom_BezierCurve::Poles");
        }
        P.Assign(myPoles);
    }

    /// <summary>The poles (a copy; index 0 is pole 1).</summary>
    public PackedList<Point3D> Poles() => myPoles.ToPackedList();

    /// <summary>The weight of the pole Index (1 for a non-rational curve).</summary>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbPoles].</exception>
    public double Weight(int Index)
    {
        if (Index < 1 || Index > myPoles.Length())
        {
            throw new OutOfRangeException("Geom_BezierCurve::Weight");
        }
        if (IsRational())
        {
            return myWeights[Index];
        }
        else
        {
            return 1.0;
        }
    }

    /// <summary>Copies the weights into W (OCCT's deprecated overload; 1s for a non-rational curve).</summary>
    /// <exception cref="DimensionErrorException">W has not NbPoles elements.</exception>
    public void Weights(NCollection_Array1<double> W)
    {
        int nbpoles = NbPoles();
        if (W.Length() != nbpoles)
        {
            throw new DimensionErrorException("Geom_BezierCurve::Weights");
        }
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

    /// <summary>The weights (a copy), or null for a non-rational curve (OCCT returns a null pointer).</summary>
    public PackedList<double>? Weights() => myRational ? myWeights.ToPackedList() : null;

    /// <summary>The weights, always NbPoles of them (1s for a non-rational curve); a copy.</summary>
    public PackedList<double> WeightsArray() => myWeights.ToPackedList();

    /// <summary>Applies T to the poles.</summary>
    protected internal sealed override void Transform(in Transform3D T)
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
    /// |f(t1) - f(t0)| &lt; Tolerance3D.
    /// </summary>
    /// <remarks>Non-const in OCCT, because it caches a value; not a mutator, so it works on a frozen curve.</remarks>
    public void Resolution(double Tolerance3D, out double UTolerance)
    {
        // OCCT.NET: a cache filled on a possibly shared curve; the flag is published after the value.
        if (!Volatile.Read(ref myMaxDerivInvOk))
        {
            double aMaxDerivInv;
            int aDeg = myPoles.Length() - 1;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Resolution(myPoles,
                                InternalWeights(),
                                myPoles.Length(),
                                InternalKnotSequence(),
                                aDeg,
                                1.0,
                                out aMaxDerivInv);
            myMaxDerivInv = aMaxDerivInv;
            Volatile.Write(ref myMaxDerivInvOk, true);
        }
        UTolerance = Tolerance3D * myMaxDerivInv;
    }

    /// <summary>A new, unfrozen curve equal to this one.</summary>
    public sealed override Geom_Geometry Copy() => new Geom_BezierCurve(this);

    /// <summary>Sets poles and weights (null: non-rational); updates the rational and closed flags.</summary>
    protected void init(NCollection_Array1<Point3D> thePoles, NCollection_Array1<double>? theWeights)
    {
        int nbpoles = thePoles.Length();
        // closed ?
        myClosed = thePoles[thePoles.Lower()].Distance(thePoles[thePoles.Upper()]) <= global::Kerf.Engine.Foundation.Precision.Confusion();

        // set fields
        myPoles.Resize(1, nbpoles, false);
        myPoles.CopyValues(thePoles);

        if (theWeights is not null)
        {
            myWeights.Resize(1, nbpoles, false);
            myWeights.CopyValues(theWeights);
            myRational = Rational(myWeights);
            if (!myRational)
            {
                myWeights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(nbpoles);
            }
        }
        else
        {
            myRational = false;
            myWeights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(nbpoles);
        }

        myMaxDerivInv = 0.0;
        myMaxDerivInvOk = false;
        ClearEvalRepresentation();
    }

    // The Bezier knots and multiplicities, shared per degree as OCCT's static arrays.

    private static readonly NCollection_Array1<double> THE_KNOTS = NCollection_Array1<double>.From([0.0, 1.0]);

    private static readonly NCollection_Array1<int>[] THE_MULTS = CreateMults();

    private static readonly NCollection_Array1<double>?[] THE_FKNOTS = CreateFlatKnots();

    private static NCollection_Array1<int>[] CreateMults()
    {
        int THE_MAX_SIZE = global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree() + 1;
        NCollection_Array1<int>[] anArr = new NCollection_Array1<int>[THE_MAX_SIZE];
        for (int i = 0; i < THE_MAX_SIZE; ++i)
        {
            anArr[i] = NCollection_Array1<int>.From([i + 1, i + 1]);
        }
        return anArr;
    }

    private static NCollection_Array1<double>?[] CreateFlatKnots()
    {
        int THE_MAX_SIZE = global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree() + 1;
        NCollection_Array1<double>?[] anArr = new NCollection_Array1<double>?[THE_MAX_SIZE];
        for (int i = 1; i <= global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree(); ++i)
        {
            anArr[i] = NCollection_Array1<double>.From(global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatBezierKnots(i));
        }
        return anArr;
    }

    /// <summary>The Bezier knots {0, 1}: shared, don't modify.</summary>
    internal NCollection_Array1<double> InternalKnots() => THE_KNOTS;

    /// <summary>The Bezier multiplicities {Degree + 1, Degree + 1}: shared, don't modify.</summary>
    internal NCollection_Array1<int> InternalMultiplicities()
    {
        Standard_ProgramError_Raise_if(myPoles.IsEmpty(), "Geom_BezierCurve: empty poles");
        return THE_MULTS[myPoles.Length() - 1];
    }

    /// <summary>The Bezier flat knots (Degree + 1 zeros, Degree + 1 ones): shared, don't modify.</summary>
    internal NCollection_Array1<double> InternalKnotSequence()
    {
        Standard_ProgramError_Raise_if(myPoles.IsEmpty(), "Geom_BezierCurve: empty poles");
        return THE_FKNOTS[myPoles.Length() - 1]!;
    }

    /// <summary>The stored poles: shared, don't modify (OCCT's <c>Poles()</c> const reference).</summary>
    internal NCollection_Array1<Point3D> InternalPoles() => myPoles;

    /// <summary>The stored weights, or null for a non-rational curve (OCCT's <c>Weights()</c> pointer). Don't modify.</summary>
    internal NCollection_Array1<double>? InternalWeights() => myRational ? myWeights : global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights();

    /// <summary>The weights, always NbPoles of them (OCCT's <c>WeightsArray()</c>): shared, don't modify.</summary>
    internal NCollection_Array1<double> InternalWeightsArray() => myWeights;

    /// <summary>The Bezier knots {0, 1} (a copy).</summary>
    public PackedList<double> Knots() => THE_KNOTS.ToPackedList();

    /// <summary>The Bezier multiplicities {Degree + 1, Degree + 1} (a copy).</summary>
    public PackedList<int> Multiplicities() => InternalMultiplicities().ToPackedList();

    /// <summary>The Bezier flat knots (a copy).</summary>
    public PackedList<double> KnotSequence() => InternalKnotSequence().ToPackedList();

    // Non-mutating twins (OCCT.NET)

    /// <summary>A copy with the degree raised to Deg.</summary>
    public Geom_BezierCurve Increased(int Deg)
    {
        Geom_BezierCurve aCopy = (Geom_BezierCurve)Copy();
        aCopy.Increase(Deg);
        return aCopy;
    }

    /// <summary>A copy with P inserted after the pole Index.</summary>
    public Geom_BezierCurve PoleInsertedAfter(int Index, in Point3D P)
    {
        Geom_BezierCurve aCopy = (Geom_BezierCurve)Copy();
        aCopy.InsertPoleAfter(Index, P);
        return aCopy;
    }

    /// <summary>A copy with P and its weight inserted after the pole Index.</summary>
    public Geom_BezierCurve PoleInsertedAfter(int Index, in Point3D P, double Weight)
    {
        Geom_BezierCurve aCopy = (Geom_BezierCurve)Copy();
        aCopy.InsertPoleAfter(Index, P, Weight);
        return aCopy;
    }

    /// <summary>A copy with P inserted before the pole Index.</summary>
    public Geom_BezierCurve PoleInsertedBefore(int Index, in Point3D P)
    {
        Geom_BezierCurve aCopy = (Geom_BezierCurve)Copy();
        aCopy.InsertPoleBefore(Index, P);
        return aCopy;
    }

    /// <summary>A copy with P and its weight inserted before the pole Index.</summary>
    public Geom_BezierCurve PoleInsertedBefore(int Index, in Point3D P, double Weight)
    {
        Geom_BezierCurve aCopy = (Geom_BezierCurve)Copy();
        aCopy.InsertPoleBefore(Index, P, Weight);
        return aCopy;
    }

    /// <summary>A copy without the pole Index.</summary>
    public Geom_BezierCurve PoleRemoved(int Index)
    {
        Geom_BezierCurve aCopy = (Geom_BezierCurve)Copy();
        aCopy.RemovePole(Index);
        return aCopy;
    }

    /// <summary>A copy segmented between U1 and U2 (see <see cref="Segment"/>).</summary>
    public Geom_BezierCurve Segmented(double U1, double U2)
    {
        Geom_BezierCurve aCopy = (Geom_BezierCurve)Copy();
        aCopy.Segment(U1, U2);
        return aCopy;
    }

    public Geom_BezierCurve WithPole(int Index, in Point3D P)
    {
        Geom_BezierCurve aCopy = (Geom_BezierCurve)Copy();
        aCopy.SetPole(Index, P);
        return aCopy;
    }

    public Geom_BezierCurve WithPole(int Index, in Point3D P, double Weight)
    {
        Geom_BezierCurve aCopy = (Geom_BezierCurve)Copy();
        aCopy.SetPole(Index, P, Weight);
        return aCopy;
    }

    public Geom_BezierCurve WithWeight(int Index, double Weight)
    {
        Geom_BezierCurve aCopy = (Geom_BezierCurve)Copy();
        aCopy.SetWeight(Index, Weight);
        return aCopy;
    }

    /// <summary>A copy evaluated through a copy of theDesc (null: without a representation).</summary>
    public Geom_BezierCurve WithEvalRepresentation(GeomEval_RepCurveDesc.Base? theDesc)
    {
        GeomEval_RepUtils.ValidateCurveDesc(theDesc, this);
        Geom_BezierCurve aCopy = (Geom_BezierCurve)Copy();
        aCopy.SetEvalRepresentation(GeomEval_RepUtils.CloneCurveDesc(theDesc));
        return aCopy;
    }

    /// <summary>A copy without an evaluation representation (the twin of ClearEvalRepresentation).</summary>
    public Geom_BezierCurve WithoutEvalRepresentation()
    {
        Geom_BezierCurve aCopy = (Geom_BezierCurve)Copy();
        aCopy.ClearEvalRepresentation();
        return aCopy;
    }
}
