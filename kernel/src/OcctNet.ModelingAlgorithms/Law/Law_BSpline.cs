// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_BSpline.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_BSpline.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_BSplKnotDistribution;
using static Occt.GeomAbs_Shape;
using static Occt.BSplCLib_KnotDistribution;
using static Occt.BSplCLib_MultDistribution;

namespace Occt;

/// <summary>
/// A one-dimensional B-spline curve (a real function of one parameter), rational or not, periodic or not: the
/// basis of the B-spline evolution laws. Its definition and its knot operations are those of Geom2d_BSplineCurve
/// with real poles.
/// </summary>
/// <remarks>
/// <para>
/// Geometry (rules, section 8): the mutators are internal and check that the curve is not frozen; callers use
/// the twins (<see cref="DegreeIncreased"/>, <see cref="KnotInserted"/>, <see cref="WithPole(int, double)"/>,
/// ...), which change a copy. <see cref="Copy"/> returns an unfrozen curve. <see cref="Law_BSpFunc"/> freezes the
/// curve it keeps.
/// </para>
/// <para>
/// The arrays are returned to callers as <see cref="PackedList{T}"/> copies (<see cref="Poles()"/>,
/// <see cref="Knots()"/>, ...), and to ported code through the internal <c>Internal...()</c> accessors, which must
/// not be modified (decision 131). They are indexed from 1, whatever the bounds of the arrays given (OCCT 8 would
/// copy those bounds, occt-issues.md).
/// </para>
/// </remarks>
public sealed class Law_BSpline : Standard_Transient
{
    private bool rational;
    private bool periodic;
    private GeomAbs_BSplKnotDistribution knotSet;
    private GeomAbs_Shape smooth;
    private int deg;
    private NCollection_HArray1<double> poles;
    private NCollection_HArray1<double>? weights;
    private NCollection_HArray1<double> flatknots;
    private NCollection_HArray1<double> knots;
    private NCollection_HArray1<int> mults;

    private static void SetPoles(NCollection_Array1<double> Poles, NCollection_Array1<double> Weights,
                                 NCollection_Array1<double> FP)
    {
        int i, j = FP.Lower();
        for (i = Poles.Lower(); i <= Poles.Upper(); i++)
        {
            double w = Weights[i];
            FP[j] = Poles[i] * w;
            j++;
            FP[j] = w;
            j++;
        }
    }

    private static void GetPoles(NCollection_Array1<double> FP, NCollection_Array1<double> Poles,
                                 NCollection_Array1<double> Weights)
    {
        int i, j = FP.Lower();
        for (i = Poles.Lower(); i <= Poles.Upper(); i++)
        {
            double w = FP[j + 1];
            Weights[i] = w;
            Poles[i] = FP[j] / w;
            j += 2;
        }
    }

    private static void CheckCurveData(NCollection_Array1<double> CPoles, NCollection_Array1<double> CKnots,
                                       NCollection_Array1<int> CMults, int Degree, bool Periodic)
    {
        if (Degree < 1 || Degree > Law_BSpline.MaxDegree())
        {
            throw new Standard_ConstructionError();
        }

        if (CPoles.Length() < 2)
        {
            throw new Standard_ConstructionError();
        }
        if (CKnots.Length() != CMults.Length())
        {
            throw new Standard_ConstructionError();
        }

        for (int I = CKnots.Lower(); I < CKnots.Upper(); I++)
        {
            if (CKnots[I + 1] - CKnots[I] <= Epsilon(Math.Abs(CKnots[I])))
            {
                throw new Standard_ConstructionError();
            }
        }

        if (CPoles.Length() != BSplCLib.NbPoles(Degree, Periodic, CMults))
        {
            throw new Standard_ConstructionError();
        }
    }

    private static void KnotAnalysis(int Degree, bool Periodic, NCollection_Array1<double> CKnots,
                                     NCollection_Array1<int> CMults, out GeomAbs_BSplKnotDistribution KnotForm,
                                     out int MaxKnotMult)
    {
        KnotForm = GeomAbs_NonUniform;

        BSplCLib_KnotDistribution KSet = BSplCLib.KnotForm(CKnots, 1, CKnots.Length());

        if (KSet == BSplCLib_Uniform)
        {
            BSplCLib_MultDistribution MSet = BSplCLib.MultForm(CMults, 1, CMults.Length());
            switch (MSet)
            {
                case BSplCLib_NonConstant:
                    break;
                case BSplCLib_Constant:
                    if (CKnots.Length() == 2)
                    {
                        KnotForm = GeomAbs_PiecewiseBezier;
                    }
                    else
                    {
                        if (CMults[1] == 1)
                        {
                            KnotForm = GeomAbs_Uniform;
                        }
                    }
                    break;
                case BSplCLib_QuasiConstant:
                    if (CMults[1] == Degree + 1)
                    {
                        double M = CMults[2];
                        if (M == Degree)
                        {
                            KnotForm = GeomAbs_PiecewiseBezier;
                        }
                        else if (M == 1)
                        {
                            KnotForm = GeomAbs_QuasiUniform;
                        }
                    }
                    break;
            }
        }

        int FirstKM = Periodic ? CKnots.Lower() : BSplCLib.FirstUKnotIndex(Degree, CMults);
        int LastKM = Periodic ? CKnots.Upper() : BSplCLib.LastUKnotIndex(Degree, CMults);
        MaxKnotMult = 0;
        if (LastKM - FirstKM != 1)
        {
            int Multi;
            for (int i = FirstKM + 1; i < LastKM; i++)
            {
                Multi = CMults[i];
                MaxKnotMult = Max(MaxKnotMult, Multi);
            }
        }
    }

    /// <summary>Whether an array of weights is really rational.</summary>
    private static bool Rational(NCollection_Array1<double> W)
    {
        // OCCT reads W from index 1 whatever its lower bound
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

    /// <summary>An unfrozen copy of the curve.</summary>
    public Law_BSpline Copy()
    {
        Law_BSpline C;
        if (IsRational())
        {
            C = new Law_BSpline(poles.Array1(), weights!.Array1(), knots.Array1(), mults.Array1(), deg, periodic);
        }
        else
        {
            C = new Law_BSpline(poles.Array1(), knots.Array1(), mults.Array1(), deg, periodic);
        }
        return C;
    }

    /// <summary>A non-rational B-spline of the given poles, knots, multiplicities and degree.</summary>
    /// <exception cref="Standard_ConstructionError">The data don't define a B-spline (see Geom2d_BSplineCurve).</exception>
    public Law_BSpline(NCollection_Array1<double> Poles, NCollection_Array1<double> Knots, NCollection_Array1<int> Mults,
                       int Degree, bool Periodic = false)
    {
        rational = false;
        periodic = Periodic;
        deg = Degree;

        // check

        CheckCurveData(Poles, Knots, Mults, Degree, Periodic);

        // copy arrays (OCCT 8's array assignment would copy the arguments' bounds; the port keeps the arrays indexed
        // from 1, as OCCT allocates them and as before OCCT 8)

        poles = new NCollection_HArray1<double>(1, Poles.Length());
        poles.ChangeArray1().CopyValues(Poles);

        knots = new NCollection_HArray1<double>(1, Knots.Length());
        knots.ChangeArray1().CopyValues(Knots);

        mults = new NCollection_HArray1<int>(1, Mults.Length());
        mults.ChangeArray1().CopyValues(Mults);

        flatknots = knots;
        UpdateKnots();
    }

    /// <summary>A rational B-spline (non-rational if all the weights are equal).</summary>
    /// <exception cref="Standard_ConstructionError">The data don't define a B-spline, or a weight is not positive.</exception>
    public Law_BSpline(NCollection_Array1<double> Poles, NCollection_Array1<double> Weights,
                       NCollection_Array1<double> Knots, NCollection_Array1<int> Mults, int Degree,
                       bool Periodic = false)
    {
        rational = true;
        periodic = Periodic;
        deg = Degree;

        // check

        CheckCurveData(Poles, Knots, Mults, Degree, Periodic);

        if (Weights.Length() != Poles.Length())
        {
            throw new Standard_ConstructionError("Law_BSpline");
        }

        int i;
        for (i = Weights.Lower(); i <= Weights.Upper(); i++)
        {
            if (Weights[i] <= gp.Resolution())
            {
                throw new Standard_ConstructionError("Law_BSpline");
            }
        }

        // check really rational
        rational = Rational(Weights);

        // copy arrays (indexed from 1, as above)

        poles = new NCollection_HArray1<double>(1, Poles.Length());
        poles.ChangeArray1().CopyValues(Poles);
        if (rational)
        {
            weights = new NCollection_HArray1<double>(1, Weights.Length());
            weights.ChangeArray1().CopyValues(Weights);
        }

        knots = new NCollection_HArray1<double>(1, Knots.Length());
        knots.ChangeArray1().CopyValues(Knots);

        mults = new NCollection_HArray1<int>(1, Mults.Length());
        mults.ChangeArray1().CopyValues(Mults);

        flatknots = knots;
        UpdateKnots();
    }

    public static int MaxDegree() => BSplCLib.MaxDegree();

    /// <summary>Raises the degree to Degree (no change if it is the current degree).</summary>
    internal void IncreaseDegree(int Degree)
    {
        CheckMutable();
        if (Degree == deg)
        {
            return;
        }

        if (Degree < deg || Degree > Law_BSpline.MaxDegree())
        {
            throw new Standard_ConstructionError();
        }

        int FromK1 = FirstUKnotIndex();
        int ToK2 = LastUKnotIndex();

        int Step = Degree - deg;

        NCollection_HArray1<double> npoles = new NCollection_HArray1<double>(1, poles.Length() + Step * (ToK2 - FromK1));

        int nbknots = BSplCLib.IncreaseDegreeCountKnots(deg, Degree, periodic, mults.Array1());

        NCollection_HArray1<double> nknots = new NCollection_HArray1<double>(1, nbknots);

        NCollection_HArray1<int> nmults = new NCollection_HArray1<int>(1, nbknots);

        NCollection_HArray1<double>? nweights = null;

        if (IsRational())
        {
            nweights = new NCollection_HArray1<double>(1, npoles.Upper());
            NCollection_Array1<double> adimpol = new NCollection_Array1<double>(1, 2 * poles.Upper());
            SetPoles(poles.Array1(), weights!.Array1(), adimpol);
            NCollection_Array1<double> adimnpol = new NCollection_Array1<double>(1, 2 * npoles.Upper());
            BSplCLib.IncreaseDegree(deg, Degree, periodic, 2, adimpol, knots.Array1(), mults.Array1(), adimnpol,
                                    nknots.ChangeArray1(), nmults.ChangeArray1());
            GetPoles(adimnpol, npoles.ChangeArray1(), nweights.ChangeArray1());
        }
        else
        {
            BSplCLib.IncreaseDegree(deg, Degree, periodic, 1, poles.Array1(), knots.Array1(), mults.Array1(),
                                    npoles.ChangeArray1(), nknots.ChangeArray1(), nmults.ChangeArray1());
        }

        deg = Degree;
        poles = npoles;
        weights = nweights;
        knots = nknots;
        mults = nmults;
        UpdateKnots();
    }

    /// <summary>Raises the multiplicity of the knot Index to M.</summary>
    internal void IncreaseMultiplicity(int Index, int M)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(1, 1);
        k[1] = knots.Value(Index);
        NCollection_Array1<int> m = new NCollection_Array1<int>(1, 1);
        m[1] = M - mults.Value(Index);
        // OCCT inserts with Add = false (Geom's IncreaseMultiplicity passes true): the multiplicity becomes
        // Max(old, M - old), not M
        InsertKnots(k, m, Epsilon(1.0));
    }

    /// <summary>Raises the multiplicities of the knots I1 to I2 to M.</summary>
    internal void IncreaseMultiplicity(int I1, int I2, int M)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(knots.Array1(), I1, I1, I2);
        NCollection_Array1<int> m = new NCollection_Array1<int>(I1, I2);
        int i;
        for (i = I1; i <= I2; i++)
        {
            m[i] = M - mults.Value(i);
        }
        // OCCT inserts with Add = false (Geom passes true): the multiplicities become Max(old, M - old), not M
        InsertKnots(k, m, Epsilon(1.0));
    }

    /// <summary>Increments the multiplicities of the knots I1 to I2 by Step.</summary>
    internal void IncrementMultiplicity(int I1, int I2, int Step)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(knots.Array1(), I1, I1, I2);
        NCollection_Array1<int> m = new NCollection_Array1<int>(I1, I2);
        m.Init(Step);
        // OCCT inserts with Add = false (Geom passes true): the multiplicities become Max(old, Step), not old + Step
        InsertKnots(k, m, Epsilon(1.0));
    }

    /// <summary>Inserts the knot U with multiplicity M (adding to an existing knot's multiplicity if Add).</summary>
    internal void InsertKnot(double U, int M = 1, double ParametricTolerance = 0.0, bool Add = true)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(1, 1);
        k[1] = U;
        NCollection_Array1<int> m = new NCollection_Array1<int>(1, 1);
        m[1] = M;
        InsertKnots(k, m, ParametricTolerance, Add);
    }

    /// <summary>Inserts the knots Knots with the multiplicities Mults.</summary>
    /// <exception cref="Standard_ConstructionError">The knots can't be inserted.</exception>
    internal void InsertKnots(NCollection_Array1<double> Knots, NCollection_Array1<int> Mults, double Epsilon = 0.0,
                              bool Add = false)
    {
        CheckMutable();
        // Check and compute new sizes
        int nbpoles, nbknots;

        if (!BSplCLib.PrepareInsertKnots(deg, periodic, knots.Array1(), mults.Array1(), Knots, Mults, out nbpoles,
                                         out nbknots, Epsilon, Add))
        {
            throw new Standard_ConstructionError("Law_BSpline::InsertKnots");
        }

        if (nbpoles == poles.Length())
        {
            return;
        }

        NCollection_HArray1<double> npoles = new NCollection_HArray1<double>(1, nbpoles);
        NCollection_HArray1<double> nknots = knots;
        NCollection_HArray1<int> nmults = mults;

        if (nbknots != knots.Length())
        {
            nknots = new NCollection_HArray1<double>(1, nbknots);
            nmults = new NCollection_HArray1<int>(1, nbknots);
        }

        if (rational)
        {
            NCollection_HArray1<double> nweights = new NCollection_HArray1<double>(1, nbpoles);
            NCollection_Array1<double> adimpol = new NCollection_Array1<double>(1, 2 * poles.Upper());
            SetPoles(poles.Array1(), weights!.Array1(), adimpol);
            NCollection_Array1<double> adimnpol = new NCollection_Array1<double>(1, 2 * npoles.Upper());
            BSplCLib.InsertKnots(deg, periodic, 2, adimpol, knots.Array1(), mults.Array1(), Knots, Mults, adimnpol,
                                 nknots.ChangeArray1(), nmults.ChangeArray1(), Epsilon, Add);
            GetPoles(adimnpol, npoles.ChangeArray1(), nweights.ChangeArray1());
            weights = nweights;
        }
        else
        {
            BSplCLib.InsertKnots(deg, periodic, 1, poles.Array1(), knots.Array1(), mults.Array1(), Knots, Mults,
                                 npoles.ChangeArray1(), nknots.ChangeArray1(), nmults.ChangeArray1(), Epsilon, Add);
        }

        poles = npoles;
        knots = nknots;
        mults = nmults;
        UpdateKnots();
    }

    /// <summary>
    /// Lowers the multiplicity of the knot Index to M (0 removes it) if the curve stays within Tolerance;
    /// returns false otherwise.
    /// </summary>
    internal bool RemoveKnot(int Index, int M, double Tolerance)
    {
        CheckMutable();
        if (M < 0)
        {
            return true;
        }

        int I1 = FirstUKnotIndex();
        int I2 = LastUKnotIndex();

        if (!periodic && (Index <= I1 || Index >= I2))
        {
            throw new Standard_OutOfRange();
        }
        else if (periodic && (Index < I1 || Index > I2))
        {
            throw new Standard_OutOfRange();
        }

        NCollection_Array1<double> oldpoles = poles.Array1();
        int step = mults.Value(Index) - M;
        if (step <= 0)
        {
            return true;
        }

        NCollection_HArray1<double> npoles = new NCollection_HArray1<double>(1, oldpoles.Length() - step);

        NCollection_HArray1<double> nknots = knots;
        NCollection_HArray1<int> nmults = mults;

        if (M == 0)
        {
            nknots = new NCollection_HArray1<double>(1, knots.Length() - 1);
            nmults = new NCollection_HArray1<int>(1, knots.Length() - 1);
        }

        if (IsRational())
        {
            NCollection_HArray1<double> nweights = new NCollection_HArray1<double>(1, npoles.Length());
            NCollection_Array1<double> adimpol = new NCollection_Array1<double>(1, 2 * poles.Upper());
            SetPoles(poles.Array1(), weights!.Array1(), adimpol);
            NCollection_Array1<double> adimnpol = new NCollection_Array1<double>(1, 2 * npoles.Upper());
            if (!BSplCLib.RemoveKnot(Index, M, deg, periodic, 2, adimpol, knots.Array1(), mults.Array1(), adimnpol,
                                     nknots.ChangeArray1(), nmults.ChangeArray1(), Tolerance))
            {
                return false;
            }
            GetPoles(adimnpol, npoles.ChangeArray1(), nweights.ChangeArray1());
            weights = nweights;
        }
        else
        {
            if (!BSplCLib.RemoveKnot(Index, M, deg, periodic, 1, poles.Array1(), knots.Array1(), mults.Array1(),
                                     npoles.ChangeArray1(), nknots.ChangeArray1(), nmults.ChangeArray1(), Tolerance))
            {
                return false;
            }
        }

        poles = npoles;
        knots = nknots;
        mults = nmults;

        UpdateKnots();
        return true;
    }

    /// <summary>Reverses the parametrization (the knots, multiplicities, poles and weights).</summary>
    internal void Reverse()
    {
        CheckMutable();
        BSplCLib.Reverse(knots.ChangeArray1());
        BSplCLib.Reverse(mults.ChangeArray1());
        int last;
        if (periodic)
        {
            last = flatknots.Upper() - deg - 1;
        }
        else
        {
            last = poles.Upper();
        }
        BSplCLib.Reverse(poles.ChangeArray1(), last);
        if (rational)
        {
            BSplCLib.Reverse(weights!.ChangeArray1(), last);
        }
        UpdateKnots();
    }

    /// <summary>The parameter on the reversed curve of the point of parameter U.</summary>
    public double ReversedParameter(double U) => (FirstParameter() + LastParameter() - U);

    /// <summary>Limits the curve to [U1, U2] (a periodic curve becomes non-periodic).</summary>
    internal void Segment(double U1, double U2)
    {
        CheckMutable();
        Standard_DomainError_Raise_if(U2 < U1, "Law_BSpline::Segment");
        double Eps = Epsilon(Max(Math.Abs(U1), Math.Abs(U2)));
        double delta = U2 - U1;

        double NewU1, NewU2;
        double U;
        int index;

        NCollection_Array1<double> Knots = new NCollection_Array1<double>(1, 2);
        NCollection_Array1<int> Mults = new NCollection_Array1<int>(1, 2);

        BSplCLib.LocateParameter(deg, knots.Array1(), mults.Array1(), U1, periodic, knots.Lower(), knots.Upper(),
                                 out index, out NewU1);
        BSplCLib.LocateParameter(deg, knots.Array1(), mults.Array1(), U2, periodic, knots.Lower(), knots.Upper(),
                                 out index, out NewU2);
        Knots[1] = Min(NewU1, NewU2);
        Knots[2] = Max(NewU1, NewU2);
        Mults[1] = Mults[2] = deg;
        InsertKnots(Knots, Mults, Eps);

        if (periodic)
        { // set the origine at NewU1
            int index0;
            BSplCLib.LocateParameter(deg, knots.Array1(), mults.Array1(), U1, periodic, knots.Lower(), knots.Upper(),
                                     out index0, out U);
            if (Math.Abs(knots.Value(index0 + 1) - U) < Eps)
            {
                index0++;
            }
            SetOrigin(index0);
            SetNotPeriodic();
        }

        // compute index1 and index2 to set the new knots and mults
        int index1, index2;
        int FromU1 = knots.Lower();
        int ToU2 = knots.Upper();
        BSplCLib.LocateParameter(deg, knots.Array1(), mults.Array1(), NewU1, periodic, FromU1, ToU2, out index1, out U);
        // OCCT: NewU1 + delta can round one ulp below the knot inserted at U2, and the strict test against Eps
        // (Epsilon(max(|U1|, |U2|))) then misses it, so the segment ends at the previous knot
        BSplCLib.LocateParameter(deg, knots.Array1(), mults.Array1(), NewU1 + delta, periodic, FromU1, ToU2,
                                 out index2, out U);
        if (Math.Abs(knots.Value(index2 + 1) - U) < Eps)
        {
            index2++;
        }

        int nbknots = index2 - index1 + 1;

        NCollection_HArray1<double> nknots = new NCollection_HArray1<double>(1, nbknots);
        NCollection_HArray1<int> nmults = new NCollection_HArray1<int>(1, nbknots);

        int i, k = 1;
        for (i = index1; i <= index2; i++)
        {
            nknots.SetValue(k, knots.Value(i));
            nmults.SetValue(k, mults.Value(i));
            k++;
        }
        nmults.SetValue(1, deg + 1);
        nmults.SetValue(nbknots, deg + 1);

        // compute index1 and index2 to set the new poles and weights
        int pindex1 = BSplCLib.PoleIndex(deg, index1, periodic, mults.Array1());
        int pindex2 = BSplCLib.PoleIndex(deg, index2, periodic, mults.Array1());

        pindex1++;
        pindex2 = Min(pindex2 + 1, poles.Length());

        int nbpoles = pindex2 - pindex1 + 1;

        NCollection_HArray1<double> nweights = new NCollection_HArray1<double>(1, nbpoles);
        NCollection_HArray1<double> npoles = new NCollection_HArray1<double>(1, nbpoles);

        k = 1;
        if (rational)
        {
            nweights = new NCollection_HArray1<double>(1, nbpoles);
            for (i = pindex1; i <= pindex2; i++)
            {
                npoles.SetValue(k, poles.Value(i));
                nweights.SetValue(k, weights!.Value(i));
                k++;
            }
        }
        else
        {
            for (i = pindex1; i <= pindex2; i++)
            {
                npoles.SetValue(k, poles.Value(i));
                k++;
            }
        }

        knots = nknots;
        mults = nmults;
        poles = npoles;
        if (rational)
        {
            weights = nweights;
        }

        UpdateKnots();
    }

    /// <summary>Changes the value of the knot Index to K.</summary>
    /// <exception cref="Standard_ConstructionError">K is not between the neighbouring knots.</exception>
    internal void SetKnot(int Index, double K)
    {
        CheckMutable();
        if (Index < 1 || Index > knots.Length())
        {
            throw new Standard_OutOfRange();
        }
        double DK = Math.Abs(Epsilon(K));
        if (Index == 1)
        {
            if (K >= knots.Value(2) - DK)
            {
                throw new Standard_ConstructionError();
            }
        }
        else if (Index == knots.Length())
        {
            if (K <= knots.Value(knots.Length() - 1) + DK)
            {
                throw new Standard_ConstructionError();
            }
        }
        else
        {
            if (K <= knots.Value(Index - 1) + DK || K >= knots.Value(Index + 1) - DK)
            {
                throw new Standard_ConstructionError();
            }
        }
        if (K != knots.Value(Index))
        {
            knots.SetValue(Index, K);
            UpdateKnots();
        }
    }

    /// <summary>Changes all the knots.</summary>
    internal void SetKnots(NCollection_Array1<double> K)
    {
        CheckMutable();
        CheckCurveData(poles.Array1(), K, mults.Array1(), deg, periodic);
        knots.ChangeArray1().CopyValues(K);
        UpdateKnots();
    }

    /// <summary>Raises the multiplicity of the knot Index to M and changes its value to K.</summary>
    internal void SetKnot(int Index, double K, int M)
    {
        CheckMutable();
        IncreaseMultiplicity(Index, M);
        SetKnot(Index, K);
    }

    /// <summary>Brings U into the period of a periodic curve.</summary>
    public void PeriodicNormalization(ref double Parameter)
    {
        double Period;

        if (periodic)
        {
            Period = flatknots.Value(flatknots.Upper() - deg) - flatknots.Value(deg + 1);
            while (Parameter > flatknots.Value(flatknots.Upper() - deg))
            {
                Parameter -= Period;
            }
            while (Parameter < flatknots.Value((deg + 1)))
            {
                Parameter += Period;
            }
        }
    }

    /// <summary>Makes the curve periodic, keeping the knots from FirstUKnotIndex to LastUKnotIndex.</summary>
    internal void SetPeriodic()
    {
        CheckMutable();
        int first = FirstUKnotIndex();
        int last = LastUKnotIndex();

        // OCCT 8's array assignment copies the bounds, so OCCT's new knots and multiplicities are indexed from
        // first, which the 1-based reads that follow (KnotForm) overrun when first > 1; the port keeps them
        // indexed from 1 (CopyValues), as before OCCT 8
        NCollection_Array1<double> cknots = new NCollection_Array1<double>(knots.Array1(), first, first, last);
        knots = new NCollection_HArray1<double>(1, cknots.Length());
        knots.ChangeArray1().CopyValues(cknots);

        NCollection_Array1<int> cmults = new NCollection_Array1<int>(mults.Array1(), first, first, last);
        cmults[first] = cmults[last] = Max(cmults[first], cmults[last]);
        mults = new NCollection_HArray1<int>(1, cmults.Length());
        mults.ChangeArray1().CopyValues(cmults);

        // compute new number of poles;
        int nbp = BSplCLib.NbPoles(deg, true, cmults);

        NCollection_Array1<double> cpoles = new NCollection_Array1<double>(poles.Array1(), 1, 1, nbp);
        poles = new NCollection_HArray1<double>(1, nbp);
        poles.ChangeArray1().CopyValues(cpoles);

        if (rational)
        {
            NCollection_Array1<double> cweights = new NCollection_Array1<double>(weights!.Array1(), 1, 1, nbp);
            weights = new NCollection_HArray1<double>(1, nbp);
            weights.ChangeArray1().CopyValues(cweights);
        }

        periodic = true;

        UpdateKnots();
    }

    /// <summary>Sets the origin of a periodic curve at the knot Index.</summary>
    internal void SetOrigin(int Index)
    {
        CheckMutable();
        Standard_NoSuchObject_Raise_if(!periodic, "Law_BSpline::SetOrigin");
        int i, k;
        int first = FirstUKnotIndex();
        int last = LastUKnotIndex();

        Standard_DomainError_Raise_if((Index < first) || (Index > last), "Law_BSpline::SetOrigine");

        int nbknots = knots.Length();
        int nbpoles = poles.Length();

        NCollection_HArray1<double> nknots = new NCollection_HArray1<double>(1, nbknots);
        NCollection_Array1<double> newknots = nknots.ChangeArray1();

        NCollection_HArray1<int> nmults = new NCollection_HArray1<int>(1, nbknots);
        NCollection_Array1<int> newmults = nmults.ChangeArray1();

        // set the knots and mults
        double period = knots.Value(last) - knots.Value(first);
        k = 1;
        for (i = Index; i <= last; i++)
        {
            newknots[k] = knots.Value(i);
            newmults[k] = mults.Value(i);
            k++;
        }
        for (i = first + 1; i <= Index; i++)
        {
            newknots[k] = knots.Value(i) + period;
            newmults[k] = mults.Value(i);
            k++;
        }

        int index = 1;
        for (i = first + 1; i <= Index; i++)
        {
            index += mults.Value(i);
        }

        // set the poles and weights
        NCollection_HArray1<double> npoles = new NCollection_HArray1<double>(1, nbpoles);
        NCollection_HArray1<double> nweights = new NCollection_HArray1<double>(1, nbpoles);
        NCollection_Array1<double> newpoles = npoles.ChangeArray1();
        NCollection_Array1<double> newweights = nweights.ChangeArray1();
        first = poles.Lower();
        last = poles.Upper();
        if (rational)
        {
            k = 1;
            for (i = index; i <= last; i++)
            {
                newpoles[k] = poles.Value(i);
                newweights[k] = weights!.Value(i);
                k++;
            }
            for (i = first; i < index; i++)
            {
                newpoles[k] = poles.Value(i);
                newweights[k] = weights!.Value(i);
                k++;
            }
        }
        else
        {
            k = 1;
            for (i = index; i <= last; i++)
            {
                newpoles[k] = poles.Value(i);
                k++;
            }
            for (i = first; i < index; i++)
            {
                newpoles[k] = poles.Value(i);
                k++;
            }
        }

        poles = npoles;
        knots = nknots;
        mults = nmults;
        if (rational)
        {
            weights = nweights;
        }
        UpdateKnots();
    }

    /// <summary>Makes a periodic curve non-periodic (the same curve, with clamped ends).</summary>
    internal void SetNotPeriodic()
    {
        CheckMutable();
        if (periodic)
        {
            int NbKnots, NbPoles;
            BSplCLib.PrepareUnperiodize(deg, mults.Array1(), out NbKnots, out NbPoles);

            NCollection_HArray1<double> npoles = new NCollection_HArray1<double>(1, NbPoles);

            NCollection_HArray1<double> nknots = new NCollection_HArray1<double>(1, NbKnots);

            NCollection_HArray1<int> nmults = new NCollection_HArray1<int>(1, NbKnots);

            NCollection_HArray1<double>? nweights = null;

            if (IsRational())
            {
                nweights = new NCollection_HArray1<double>(1, NbPoles);

                NCollection_Array1<double> adimpol = new NCollection_Array1<double>(1, 2 * poles.Upper());
                SetPoles(poles.Array1(), weights!.Array1(), adimpol);
                NCollection_Array1<double> adimnpol = new NCollection_Array1<double>(1, 2 * npoles.Upper());
                BSplCLib.Unperiodize(deg, 1, mults.Array1(), knots.Array1(), adimpol, nmults.ChangeArray1(),
                                     nknots.ChangeArray1(), adimnpol);
                GetPoles(adimnpol, npoles.ChangeArray1(), nweights.ChangeArray1());
            }
            else
            {
                BSplCLib.Unperiodize(deg, 1, mults.Array1(), knots.Array1(), poles.Array1(), nmults.ChangeArray1(),
                                     nknots.ChangeArray1(), npoles.ChangeArray1());
            }
            poles = npoles;
            weights = nweights;
            mults = nmults;
            knots = nknots;
            periodic = false;

            UpdateKnots();
        }
    }

    /// <summary>Changes the pole Index to P.</summary>
    internal void SetPole(int Index, double P)
    {
        CheckMutable();
        if (Index < 1 || Index > poles.Length())
        {
            throw new Standard_OutOfRange();
        }
        poles.SetValue(Index, P);
    }

    /// <summary>Changes the pole Index to P and its weight to W.</summary>
    internal void SetPole(int Index, double P, double W)
    {
        CheckMutable();
        SetPole(Index, P);
        SetWeight(Index, W);
    }

    /// <summary>Changes the weight of the pole Index to W.</summary>
    internal void SetWeight(int Index, double W)
    {
        CheckMutable();
        if (Index < 1 || Index > poles.Length())
        {
            throw new Standard_OutOfRange();
        }

        if (W <= gp.Resolution())
        {
            throw new Standard_ConstructionError();
        }

        bool rat = IsRational() || (Math.Abs(W - 1.0) > gp.Resolution());

        if (rat)
        {
            if (rat && !IsRational())
            {
                weights = new NCollection_HArray1<double>(1, poles.Length(), 1.0);
            }

            weights!.SetValue(Index, W);

            if (IsRational())
            {
                rat = Rational(weights.Array1());
                if (!rat)
                {
                    weights = null;
                }
            }

            rational = weights is not null;
        }
    }

    private void UpdateKnots()
    {

        rational = weights is not null;

        KnotAnalysis(deg, periodic, knots.Array1(), mults.Array1(), out knotSet, out int MaxKnotMult);

        if (knotSet == GeomAbs_Uniform && !periodic)
        {
            flatknots = knots;
        }
        else
        {
            flatknots = new NCollection_HArray1<double>(1, BSplCLib.KnotSequenceLength(mults.Array1(), deg, periodic));

            BSplCLib.KnotSequence(knots.Array1(), mults.Array1(), deg, periodic, flatknots.ChangeArray1());
        }

        if (MaxKnotMult == 0)
        {
            smooth = GeomAbs_CN;
        }
        else
        {
            switch (deg - MaxKnotMult)
            {
                case 0:
                    smooth = GeomAbs_C0;
                    break;
                case 1:
                    smooth = GeomAbs_C1;
                    break;
                case 2:
                    smooth = GeomAbs_C2;
                    break;
                case 3:
                    smooth = GeomAbs_C3;
                    break;
                default:
                    smooth = GeomAbs_C3;
                    break;
            }
        }
    }

    /// <summary>Whether the curve is at least CN.</summary>
    public bool IsCN(int N)
    {
        Standard_RangeError_Raise_if(N < 0, "Law_BSpline::IsCN");

        switch (smooth)
        {
            case GeomAbs_CN:
                return true;
            case GeomAbs_C0:
                return N <= 0;
            case GeomAbs_G1:
                return N <= 0;
            case GeomAbs_C1:
                return N <= 1;
            case GeomAbs_G2:
                return N <= 1;
            case GeomAbs_C2:
                return N <= 2;
            case GeomAbs_C3:
                return N <= 3 ? true : N <= deg - BSplCLib.MaxKnotMult(mults.Array1(), mults.Lower() + 1, mults.Upper() - 1);
            default:
                return false;
        }
    }

    public bool IsClosed() => (Math.Abs(StartPoint() - EndPoint())) <= gp.Resolution();

    public bool IsPeriodic() => periodic;

    public GeomAbs_Shape Continuity() => smooth;

    public int Degree() => deg;

    public double Value(double U)
    {
        D0(U, out double P);
        return P;
    }

    public void D0(double U, out double P)
    {
        double NewU = U;
        PeriodicNormalization(ref NewU);
        if (rational)
        {
            BSplCLib.D0(NewU, 0, deg, periodic, poles, weights!.Array1(), flatknots, BSplCLib.NoMults(), out P);
        }
        else
        {
            BSplCLib.D0(NewU, 0, deg, periodic, poles, BSplCLib.NoWeights(), flatknots, BSplCLib.NoMults(), out P);
        }
    }

    public void D1(double U, out double P, out double V1)
    {
        double NewU = U;
        PeriodicNormalization(ref NewU);
        if (rational)
        {
            BSplCLib.D1(NewU, 0, deg, periodic, poles, weights!.Array1(), flatknots, BSplCLib.NoMults(), out P, out V1);
        }
        else
        {
            BSplCLib.D1(NewU, 0, deg, periodic, poles, BSplCLib.NoWeights(), flatknots, BSplCLib.NoMults(), out P,
                        out V1);
        }
    }

    public void D2(double U, out double P, out double V1, out double V2)
    {
        double NewU = U;
        PeriodicNormalization(ref NewU);
        if (rational)
        {
            BSplCLib.D2(NewU, 0, deg, periodic, poles, weights!.Array1(), flatknots, BSplCLib.NoMults(), out P, out V1,
                        out V2);
        }
        else
        {
            BSplCLib.D2(NewU, 0, deg, periodic, poles, BSplCLib.NoWeights(), flatknots, BSplCLib.NoMults(), out P,
                        out V1, out V2);
        }
    }

    public void D3(double U, out double P, out double V1, out double V2, out double V3)
    {
        double NewU = U;
        PeriodicNormalization(ref NewU);
        if (rational)
        {
            BSplCLib.D3(NewU, 0, deg, periodic, poles, weights!.Array1(), flatknots, BSplCLib.NoMults(), out P, out V1,
                        out V2, out V3);
        }
        else
        {
            BSplCLib.D3(NewU, 0, deg, periodic, poles, BSplCLib.NoWeights(), flatknots, BSplCLib.NoMults(), out P,
                        out V1, out V2, out V3);
        }
    }

    /// <summary>The derivative of order N at U.</summary>
    public double DN(double U, int N)
    {
        double V;
        if (rational)
        {
            BSplCLib.DN(U, N, 0, deg, periodic, poles, weights!.Array1(), flatknots, BSplCLib.NoMults(), out V);
        }
        else
        {
            BSplCLib.DN(U, N, 0, deg, periodic, poles, BSplCLib.NoWeights(), flatknots, BSplCLib.NoMults(), out V);
        }
        return V;
    }

    public double EndPoint()
    {
        if (mults.Value(knots.Upper()) == deg + 1)
        {
            return poles.Value(poles.Upper());
        }
        else
        {
            return Value(LastParameter());
        }
    }

    public int FirstUKnotIndex()
    {
        if (periodic)
        {
            return 1;
        }
        else
        {
            return BSplCLib.FirstUKnotIndex(deg, mults.Array1());
        }
    }

    public double FirstParameter() => flatknots.Value(deg + 1);

    public double Knot(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > knots.Length(), "Law_BSpline::Knot");
        return knots.Value(Index);
    }

    public GeomAbs_BSplKnotDistribution KnotDistribution() => knotSet;

    /// <summary>Copies the knots into K (OCCT's array assignment: K must have NbKnots() elements).</summary>
    public void Knots(NCollection_Array1<double> K)
    {
        Standard_DimensionError_Raise_if(K.Length() != knots.Length(), "Law_BSpline::Knots");
        K.Assign(knots.Array1());
    }

    /// <summary>The knots (a copy).</summary>
    public PackedList<double> Knots() => knots.ToPackedList();

    /// <summary>The knot array itself, for ported code: don't modify it (decision 131).</summary>
    internal NCollection_Array1<double> InternalKnots() => knots;

    /// <summary>Copies the flat knots into K (K must have as many elements).</summary>
    public void KnotSequence(NCollection_Array1<double> K)
    {
        Standard_DimensionError_Raise_if(K.Length() != flatknots.Length(), "Law_BSpline::KnotSequence");
        K.Assign(flatknots.Array1());
    }

    /// <summary>The flat knots, each knot repeated by its multiplicity (a copy).</summary>
    public PackedList<double> KnotSequence() => flatknots.ToPackedList();

    /// <summary>The flat knot array itself, for ported code: don't modify it (decision 131).</summary>
    internal NCollection_Array1<double> InternalKnotSequence() => flatknots;

    public int LastUKnotIndex()
    {
        if (periodic)
        {
            return knots.Length();
        }
        else
        {
            return BSplCLib.LastUKnotIndex(deg, mults.Array1());
        }
    }

    public double LastParameter() => flatknots.Value(flatknots.Upper() - deg);

    /// <summary>The value at U computed on the span between the knots FromK1 and ToK2.</summary>
    public double LocalValue(double U, int FromK1, int ToK2)
    {
        LocalD0(U, FromK1, ToK2, out double P);
        return P;
    }

    public void LocalD0(double U, int FromK1, int ToK2, out double P)
    {
        Standard_DomainError_Raise_if(FromK1 == ToK2, "Law_BSpline::LocalValue");
        BSplCLib.LocateParameter(deg, flatknots, U, periodic, FromK1, ToK2, out int index, out double u);
        index = BSplCLib.FlatIndex(deg, index, mults.Array1(), periodic);
        if (rational)
        {
            BSplCLib.D0(u, index, deg, periodic, poles, weights!.Array1(), flatknots, BSplCLib.NoMults(), out P);
        }
        else
        {
            BSplCLib.D0(u, index, deg, periodic, poles, BSplCLib.NoWeights(), flatknots, BSplCLib.NoMults(), out P);
        }
    }

    public void LocalD1(double U, int FromK1, int ToK2, out double P, out double V1)
    {
        Standard_DomainError_Raise_if(FromK1 == ToK2, "Law_BSpline::LocalD1");
        BSplCLib.LocateParameter(deg, flatknots, U, periodic, FromK1, ToK2, out int index, out double u);
        index = BSplCLib.FlatIndex(deg, index, mults.Array1(), periodic);
        if (rational)
        {
            BSplCLib.D1(u, index, deg, periodic, poles, weights!.Array1(), flatknots, BSplCLib.NoMults(), out P, out V1);
        }
        else
        {
            BSplCLib.D1(u, index, deg, periodic, poles, BSplCLib.NoWeights(), flatknots, BSplCLib.NoMults(), out P,
                        out V1);
        }
    }

    public void LocalD2(double U, int FromK1, int ToK2, out double P, out double V1, out double V2)
    {
        Standard_DomainError_Raise_if(FromK1 == ToK2, "Law_BSpline::LocalD2");
        BSplCLib.LocateParameter(deg, flatknots, U, periodic, FromK1, ToK2, out int index, out double u);
        index = BSplCLib.FlatIndex(deg, index, mults.Array1(), periodic);
        if (rational)
        {
            BSplCLib.D2(u, index, deg, periodic, poles, weights!.Array1(), flatknots, BSplCLib.NoMults(), out P, out V1,
                        out V2);
        }
        else
        {
            BSplCLib.D2(u, index, deg, periodic, poles, BSplCLib.NoWeights(), flatknots, BSplCLib.NoMults(), out P,
                        out V1, out V2);
        }
    }

    public void LocalD3(double U, int FromK1, int ToK2, out double P, out double V1, out double V2, out double V3)
    {
        Standard_DomainError_Raise_if(FromK1 == ToK2, "Law_BSpline::LocalD3");
        BSplCLib.LocateParameter(deg, flatknots, U, periodic, FromK1, ToK2, out int index, out double u);
        index = BSplCLib.FlatIndex(deg, index, mults.Array1(), periodic);
        if (rational)
        {
            BSplCLib.D3(u, index, deg, periodic, poles, weights!.Array1(), flatknots, BSplCLib.NoMults(), out P, out V1,
                        out V2, out V3);
        }
        else
        {
            BSplCLib.D3(u, index, deg, periodic, poles, BSplCLib.NoWeights(), flatknots, BSplCLib.NoMults(), out P,
                        out V1, out V2, out V3);
        }
    }

    public double LocalDN(double U, int FromK1, int ToK2, int N)
    {
        Standard_DomainError_Raise_if(FromK1 == ToK2, "Law_BSpline::LocalD3");
        BSplCLib.LocateParameter(deg, flatknots, U, periodic, FromK1, ToK2, out int index, out double u);
        index = BSplCLib.FlatIndex(deg, index, mults.Array1(), periodic);

        double V;
        if (rational)
        {
            BSplCLib.DN(u, N, index, deg, periodic, poles, weights!.Array1(), flatknots, BSplCLib.NoMults(), out V);
        }
        else
        {
            BSplCLib.DN(u, N, index, deg, periodic, poles, BSplCLib.NoWeights(), flatknots, BSplCLib.NoMults(), out V);
        }
        return V;
    }

    public int Multiplicity(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > mults.Length(), "Law_BSpline::Multiplicity");
        return mults.Value(Index);
    }

    /// <summary>Copies the multiplicities into M (M must have NbKnots() elements).</summary>
    public void Multiplicities(NCollection_Array1<int> M)
    {
        Standard_DimensionError_Raise_if(M.Length() != mults.Length(), "Law_BSpline::Multiplicities");
        M.Assign(mults.Array1());
    }

    /// <summary>The multiplicities (a copy).</summary>
    public PackedList<int> Multiplicities() => mults.ToPackedList();

    /// <summary>The multiplicity array itself, for ported code: don't modify it (decision 131).</summary>
    internal NCollection_Array1<int> InternalMultiplicities() => mults;

    public int NbKnots() => knots.Length();

    public int NbPoles() => poles.Length();

    public double Pole(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > poles.Length(), "Law_BSpline::Pole");
        return poles.Value(Index);
    }

    /// <summary>Copies the poles into P (P must have NbPoles() elements).</summary>
    public void Poles(NCollection_Array1<double> P)
    {
        Standard_DimensionError_Raise_if(P.Length() != poles.Length(), "Law_BSpline::Poles");
        P.Assign(poles.Array1());
    }

    /// <summary>The poles (a copy).</summary>
    public PackedList<double> Poles() => poles.ToPackedList();

    /// <summary>The pole array itself, for ported code: don't modify it (decision 131).</summary>
    internal NCollection_Array1<double> InternalPoles() => poles;

    public double StartPoint()
    {
        if (mults.Value(1) == deg + 1)
        {
            return poles.Value(1);
        }
        else
        {
            return Value(FirstParameter());
        }
    }

    public double Weight(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > poles.Length(), "Law_BSpline::Weight");
        if (IsRational())
        {
            return weights!.Value(Index);
        }
        else
        {
            return 1.0;
        }
    }

    /// <summary>Copies the weights into W (all 1 for a non-rational curve; W must have NbPoles() elements).</summary>
    public void Weights(NCollection_Array1<double> W)
    {
        Standard_DimensionError_Raise_if(W.Length() != poles.Length(), "Law_BSpline::Weights");
        if (IsRational())
        {
            W.Assign(weights!.Array1());
        }
        else
        {
            int i;
            for (i = W.Lower(); i <= W.Upper(); i++)
            {
                W[i] = 1.0;
            }
        }
    }

    /// <summary>The weights (a copy), or null for a non-rational curve.</summary>
    public PackedList<double>? Weights() => IsRational() ? weights!.ToPackedList() : null;

    /// <summary>The weight array itself (null if non-rational), for ported code: don't modify it (decision 131).</summary>
    internal NCollection_Array1<double>? InternalWeights() => weights;

    public bool IsRational() => weights is not null;

    /// <summary>
    /// Locates U in the knots (the flat knots if WithKnotRepetition): Knots(I1) &lt;= U &lt;= Knots(I2), with
    /// I1 = I2 when U is on a knot within ParametricTolerance.
    /// </summary>
    public void LocateU(double U, double ParametricTolerance, out int I1, out int I2, bool WithKnotRepetition = false)
    {
        double NewU = U;
        NCollection_HArray1<double> TheKnots;
        if (WithKnotRepetition)
        {
            TheKnots = flatknots;
        }
        else
        {
            TheKnots = knots;
        }

        PeriodicNormalization(ref NewU); // Attention a la periode

        NCollection_Array1<double> CKnots = TheKnots.Array1();
        double UFirst = CKnots[1];
        double ULast = CKnots[CKnots.Length()];
        if (Math.Abs(U - UFirst) <= Math.Abs(ParametricTolerance))
        {
            I1 = I2 = 1;
        }
        else if (Math.Abs(U - ULast) <= Math.Abs(ParametricTolerance))
        {
            I1 = I2 = CKnots.Length();
        }
        else if (NewU < UFirst - Math.Abs(ParametricTolerance))
        {
            I2 = 1;
            I1 = 0;
        }
        else if (NewU > ULast + Math.Abs(ParametricTolerance))
        {
            I1 = CKnots.Length();
            I2 = I1 + 1;
        }
        else
        {
            BSplCLib.Hunt(CKnots, NewU, out I1);
            I1 = Max(Min(I1, CKnots.Upper()), CKnots.Lower());
            while (I1 + 1 <= CKnots.Upper() && Math.Abs(CKnots[I1 + 1] - NewU) <= Math.Abs(ParametricTolerance))
            {
                I1++;
            }
            if (Math.Abs(CKnots[I1] - NewU) <= Math.Abs(ParametricTolerance))
            {
                I2 = I1;
            }
            else
            {
                I2 = I1 + 1;
            }
        }
    }

    /// <summary>
    /// Moves the value at U to NewValue and the derivative to Derivative, by changing the poles (see
    /// Geom2d_BSplineCurve.MovePointAndTangent); ErrorStatus is 0 on success, and the curve is unchanged otherwise.
    /// </summary>
    internal void MovePointAndTangent(double U, double P, double Tangent, double Tolerance, int StartingCondition,
                                     int EndingCondition, out int ErrorStatus)
    {
        CheckMutable();
        NCollection_Array1<double> new_poles = new NCollection_Array1<double>(1, poles.Length());
        double delta, delta_derivative;
        const int dimension = 1;
        D1(U, out delta, out delta_derivative);
        delta = P - delta;

        delta_derivative = Tangent - delta_derivative;
        ReadOnlySpan<double> aDelta = [delta];
        ReadOnlySpan<double> aDeltaDerivative = [delta_derivative];
        BSplCLib.MovePointAndTangent(U, dimension, aDelta, aDeltaDerivative, Tolerance, deg, StartingCondition,
                                     EndingCondition, poles.AsSpan(), rational ? weights!.Array1() : BSplCLib.NoWeights(),
                                     flatknots.Array1(), new_poles.AsSpan(), out ErrorStatus);
        if (ErrorStatus == 0)
        {
            poles.ChangeArray1().CopyValues(new_poles);
        }
    }

    /// <summary>The parametric tolerance UTolerance such that |t1 - t0| &lt; UTolerance implies |C(t1) - C(t0)| &lt; Tolerance3D.</summary>
    public void Resolution(double Tolerance3D, out double UTolerance)
    {
        if (rational)
        {
            BSplCLib.Resolution(poles.AsSpan(), 1, poles.Length(), weights!.Array1(), flatknots, deg, Tolerance3D,
                                out UTolerance);
        }
        else
        {
            BSplCLib.Resolution(poles.AsSpan(), 1, poles.Length(), BSplCLib.NoWeights(), flatknots, deg, Tolerance3D,
                                out UTolerance);
        }
    }

    // Non-mutating twins of the mutators (OCCT.NET)

    /// <summary>A copy with its degree raised to Degree.</summary>
    public Law_BSpline DegreeIncreased(int Degree)
    {
        Law_BSpline aCopy = Copy();
        aCopy.IncreaseDegree(Degree);
        return aCopy;
    }

    /// <summary>A copy with the multiplicity of the knot Index raised to M.</summary>
    public Law_BSpline MultiplicityIncreased(int Index, int M)
    {
        Law_BSpline aCopy = Copy();
        aCopy.IncreaseMultiplicity(Index, M);
        return aCopy;
    }

    /// <summary>A copy with the multiplicities of the knots I1 to I2 raised to M.</summary>
    public Law_BSpline MultiplicityIncreased(int I1, int I2, int M)
    {
        Law_BSpline aCopy = Copy();
        aCopy.IncreaseMultiplicity(I1, I2, M);
        return aCopy;
    }

    /// <summary>A copy with the multiplicities of the knots I1 to I2 incremented by Step.</summary>
    public Law_BSpline MultiplicityIncremented(int I1, int I2, int Step)
    {
        Law_BSpline aCopy = Copy();
        aCopy.IncrementMultiplicity(I1, I2, Step);
        return aCopy;
    }

    /// <summary>A copy with the knot U inserted (see InsertKnot; Add defaults to true, as in OCCT).</summary>
    public Law_BSpline KnotInserted(double U, int M = 1, double ParametricTolerance = 0.0, bool Add = true)
    {
        Law_BSpline aCopy = Copy();
        aCopy.InsertKnot(U, M, ParametricTolerance, Add);
        return aCopy;
    }

    /// <summary>A copy with the knots Knots inserted (Add defaults to false, as in OCCT).</summary>
    public Law_BSpline KnotsInserted(NCollection_Array1<double> Knots, NCollection_Array1<int> Mults,
                                     double ParametricTolerance = 0.0, bool Add = false)
    {
        Law_BSpline aCopy = Copy();
        aCopy.InsertKnots(Knots, Mults, ParametricTolerance, Add);
        return aCopy;
    }

    /// <summary>A copy with the multiplicity of the knot Index lowered to M, or null if that changes the curve by more than Tolerance.</summary>
    public Law_BSpline? KnotRemoved(int Index, int M, double Tolerance)
    {
        Law_BSpline aCopy = Copy();
        return aCopy.RemoveKnot(Index, M, Tolerance) ? aCopy : null;
    }

    /// <summary>A copy with the parametrization reversed.</summary>
    public Law_BSpline Reversed()
    {
        Law_BSpline aCopy = Copy();
        aCopy.Reverse();
        return aCopy;
    }

    /// <summary>A copy limited to [U1, U2] (see Segment).</summary>
    public Law_BSpline Segmented(double U1, double U2)
    {
        Law_BSpline aCopy = Copy();
        aCopy.Segment(U1, U2);
        return aCopy;
    }

    /// <summary>A copy with the knot Index changed to K.</summary>
    public Law_BSpline WithKnot(int Index, double K)
    {
        Law_BSpline aCopy = Copy();
        aCopy.SetKnot(Index, K);
        return aCopy;
    }

    /// <summary>A copy with the knots K.</summary>
    public Law_BSpline WithKnots(NCollection_Array1<double> K)
    {
        Law_BSpline aCopy = Copy();
        aCopy.SetKnots(K);
        return aCopy;
    }

    /// <summary>A copy with the knot Index raised to multiplicity M and changed to K.</summary>
    public Law_BSpline WithKnot(int Index, double K, int M)
    {
        Law_BSpline aCopy = Copy();
        aCopy.SetKnot(Index, K, M);
        return aCopy;
    }

    /// <summary>A periodic copy (see SetPeriodic).</summary>
    public Law_BSpline WithPeriodic()
    {
        Law_BSpline aCopy = Copy();
        aCopy.SetPeriodic();
        return aCopy;
    }

    /// <summary>A copy of this periodic curve with its origin at the knot Index.</summary>
    public Law_BSpline WithOrigin(int Index)
    {
        Law_BSpline aCopy = Copy();
        aCopy.SetOrigin(Index);
        return aCopy;
    }

    /// <summary>A non-periodic copy (see SetNotPeriodic).</summary>
    public Law_BSpline WithNotPeriodic()
    {
        Law_BSpline aCopy = Copy();
        aCopy.SetNotPeriodic();
        return aCopy;
    }

    /// <summary>A copy with the pole Index changed to P.</summary>
    public Law_BSpline WithPole(int Index, double P)
    {
        Law_BSpline aCopy = Copy();
        aCopy.SetPole(Index, P);
        return aCopy;
    }

    /// <summary>A copy with the pole Index changed to P, of weight W.</summary>
    public Law_BSpline WithPole(int Index, double P, double W)
    {
        Law_BSpline aCopy = Copy();
        aCopy.SetPole(Index, P, W);
        return aCopy;
    }

    /// <summary>A copy with the weight of the pole Index changed to W.</summary>
    public Law_BSpline WithWeight(int Index, double W)
    {
        Law_BSpline aCopy = Copy();
        aCopy.SetWeight(Index, W);
        return aCopy;
    }

    /// <summary>A copy whose value and derivative at U are moved (see MovePointAndTangent).</summary>
    public Law_BSpline PointAndTangentMoved(double U, double P, double Tangent, double Tolerance, int StartingCondition,
                                           int EndingCondition, out int ErrorStatus)
    {
        Law_BSpline aCopy = Copy();
        aCopy.MovePointAndTangent(U, P, Tangent, Tolerance, StartingCondition, EndingCondition, out ErrorStatus);
        return aCopy;
    }
}
