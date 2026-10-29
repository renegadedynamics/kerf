// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_BSplineCurve.hxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_BSplineCurve.cxx
// Port of: src/ModelingData/TKG2d/Geom2d/Geom2d_BSplineCurve_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_BSplKnotDistribution;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// A B-spline curve in the plane: rational or not, periodic or not, of degree 1 to MaxDegree(), defined by its
/// poles, (weights,) knots and their multiplicities. For a non-periodic curve the number of poles is the sum of
/// the multiplicities minus Degree - 1; for a periodic one, the sum of the multiplicities except the last.
/// </summary>
/// <remarks>
/// <para>
/// OCCT returns its pole, weight and knot arrays by const reference. The port returns them to callers as
/// <see cref="PackedList{T}"/> copies (<see cref="Poles()"/>, <see cref="Knots()"/>, ...), and gives ported code
/// the arrays themselves through internal <c>...Ref()</c> accessors, which must not be modified (decision 701).
/// The arrays are indexed from 1.
/// </para>
/// <para>
/// OCCT's Geom2d_BSplineCurve keeps an attached evaluation representation through every modification, where
/// Geom_BSplineCurve drops it (occt-issues.md); the port does the same as Geom2d.
/// </para>
/// </remarks>
public class Geom2d_BSplineCurve : Geom2d_BoundedCurve
{
    private NCollection_Array1<gp_Pnt2d> myPoles = new NCollection_Array1<gp_Pnt2d>();
    private NCollection_Array1<double> myWeights = new NCollection_Array1<double>();
    private NCollection_Array1<double> myKnots = new NCollection_Array1<double>();
    private NCollection_Array1<double> myFlatKnots = new NCollection_Array1<double>();
    private NCollection_Array1<int> myMults = new NCollection_Array1<int>();
    private Geom2dEval_RepCurveDesc.Base? myEvalRep;
    private int myDeg = 0;
    private bool myPeriodic = false;
    private bool myRational = false;
    private GeomAbs_BSplKnotDistribution myKnotSet = GeomAbs_NonUniform;
    private GeomAbs_Shape mySmooth = GeomAbs_C0;
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

    private static void CheckCurveData(NCollection_Array1<gp_Pnt2d> CPoles,
                                       NCollection_Array1<double> CKnots,
                                       NCollection_Array1<int> CMults,
                                       int Degree,
                                       bool Periodic)
    {
        if (Degree < 1 || Degree > Geom2d_BSplineCurve.MaxDegree())
        {
            throw new Standard_ConstructionError("BSpline curve: invalid degree");
        }

        if (CPoles.Length() < 2)
        {
            throw new Standard_ConstructionError("BSpline curve: at least 2 poles required");
        }

        if (CKnots.Length() != CMults.Length())
        {
            throw new Standard_ConstructionError("BSpline curve: Knot and Mult array size mismatch");
        }

        for (int I = CKnots.Lower(); I < CKnots.Upper(); I++)
        {
            if (CKnots[I + 1] - CKnots[I] <= Epsilon(Math.Abs(CKnots[I])))
            {
                throw new Standard_ConstructionError("BSpline curve: Knots interval values too close");
            }
        }

        if (CPoles.Length() != BSplCLib.NbPoles(Degree, Periodic, CMults))
        {
            throw new Standard_ConstructionError("BSpline curve: # Poles and degree mismatch");
        }
    }

    // Check rationality of an array of weights
    private static bool Rational(NCollection_Array1<double> theWeights)
    {
        for (int i = theWeights.Lower(); i < theWeights.Upper(); i++)
        {
            if (Math.Abs(theWeights[i] - theWeights[i + 1]) > gp.Resolution())
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>A new, unfrozen curve equal to this one.</summary>
    public sealed override Geom2d_Geometry Copy() => new Geom2d_BSplineCurve(this);

    /// <summary>A copy of theOther (its evaluation representation deep-copied), not frozen.</summary>
    public Geom2d_BSplineCurve(Geom2d_BSplineCurve theOther)
    {
        myPoles = new NCollection_Array1<gp_Pnt2d>(theOther.myPoles);
        myWeights = theOther.myRational ? new NCollection_Array1<double>(theOther.myWeights)
                                        : BSplCLib.UnitWeights(theOther.myPoles.Length());
        myKnots = new NCollection_Array1<double>(theOther.myKnots);
        myFlatKnots = new NCollection_Array1<double>(theOther.myFlatKnots);
        myMults = new NCollection_Array1<int>(theOther.myMults);
        myEvalRep = Geom2dEval_RepUtils.CloneCurveDesc(theOther.myEvalRep);
        myDeg = theOther.myDeg;
        myPeriodic = theOther.myPeriodic;
        myRational = theOther.myRational;
        myKnotSet = theOther.myKnotSet;
        mySmooth = theOther.mySmooth;
        myMaxDerivInv = theOther.myMaxDerivInv;
        myMaxDerivInvOk = false;
    }

    /// <summary>A non-rational B-spline curve.</summary>
    /// <exception cref="Standard_ConstructionError">
    /// Degree outside [1, MaxDegree()], fewer than 2 poles, knots and multiplicities of different lengths, knots
    /// not increasing (by more than Epsilon), or a pole count inconsistent with the degree and multiplicities.
    /// </exception>
    public Geom2d_BSplineCurve(NCollection_Array1<gp_Pnt2d> Poles,
                               NCollection_Array1<double> Knots,
                               NCollection_Array1<int> Multiplicities,
                               int Degree,
                               bool Periodic = false)
    {
        myRational = false;
        myMaxDerivInv = 0.0;
        myMaxDerivInvOk = false;
        myDeg = Degree;
        myPeriodic = Periodic;

        // check
        CheckCurveData(Poles, Knots, Multiplicities, Degree, Periodic);

        // copy arrays
        myPoles.Resize(1, Poles.Length(), false);
        myPoles.CopyValues(Poles);
        myWeights = BSplCLib.UnitWeights(Poles.Length());

        myKnots.Resize(1, Knots.Length(), false);
        myKnots.CopyValues(Knots);

        myMults.Resize(1, Multiplicities.Length(), false);
        myMults.CopyValues(Multiplicities);

        updateKnots();
    }

    /// <summary>A rational B-spline curve (non-rational if all the weights are equal).</summary>
    /// <exception cref="Standard_ConstructionError">
    /// As the non-rational constructor, or weights of a different length than the poles, or a weight not above
    /// gp::Resolution().
    /// </exception>
    public Geom2d_BSplineCurve(NCollection_Array1<gp_Pnt2d> Poles,
                               NCollection_Array1<double> Weights,
                               NCollection_Array1<double> Knots,
                               NCollection_Array1<int> Multiplicities,
                               int Degree,
                               bool Periodic = false)
    {
        myRational = true;
        myMaxDerivInv = 0.0;
        myMaxDerivInvOk = false;
        myDeg = Degree;
        myPeriodic = Periodic;

        // check
        CheckCurveData(Poles, Knots, Multiplicities, Degree, Periodic);

        if (Weights.Length() != Poles.Length())
        {
            throw new Standard_ConstructionError("Geom2d_BSplineCurve: Weights and Poles array size mismatch");
        }

        int i;
        for (i = Weights.Lower(); i <= Weights.Upper(); i++)
        {
            if (Weights[i] <= gp.Resolution())
            {
                throw new Standard_ConstructionError("Geom2d_BSplineCurve: Weights values too small");
            }
        }

        // check really rational
        myRational = Rational(Weights);

        // copy arrays
        myPoles.Resize(1, Poles.Length(), false);
        myPoles.CopyValues(Poles);
        if (myRational)
        {
            myWeights.Resize(1, Weights.Length(), false);
            myWeights.CopyValues(Weights);
        }
        else
        {
            myWeights = BSplCLib.UnitWeights(Poles.Length());
        }

        myKnots.Resize(1, Knots.Length(), false);
        myKnots.CopyValues(Knots);

        myMults.Resize(1, Multiplicities.Length(), false);
        myMults.CopyValues(Multiplicities);

        updateKnots();
    }

    /// <summary>The maximum degree of a B-spline curve: 25.</summary>
    public static int MaxDegree() => BSplCLib.MaxDegree();

    /// <summary>Raises the degree to Degree, keeping the shape of the curve.</summary>
    /// <exception cref="Standard_ConstructionError">Degree is below the current degree or above MaxDegree().</exception>
    internal void IncreaseDegree(int Degree)
    {
        CheckMutable();
        if (Degree == myDeg)
        {
            return;
        }

        if (Degree < myDeg || Degree > Geom2d_BSplineCurve.MaxDegree())
        {
            throw new Standard_ConstructionError("BSpline curve: IncreaseDegree: bad degree value");
        }

        int FromK1 = FirstUKnotIndex();
        int ToK2 = LastUKnotIndex();

        int Step = Degree - myDeg;

        // OCCT: counts only the knots of the parameter range; on a curve not clamped at its ends the new pole count
        // doesn't match the new multiplicities, or BSplCLib::IncreaseDegree writes past it (occt-issues.md).
        NCollection_Array1<gp_Pnt2d> npoles = new NCollection_Array1<gp_Pnt2d>(1, myPoles.Length() + Step * (ToK2 - FromK1));

        int nbknots = BSplCLib.IncreaseDegreeCountKnots(myDeg, Degree, myPeriodic, myMults);

        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, nbknots);

        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, nbknots);

        NCollection_Array1<double> nweights = new NCollection_Array1<double>();
        if (IsRational())
        {
            nweights.Resize(1, npoles.Upper(), false);
        }

        BSplCLib.IncreaseDegree(myDeg,
                                Degree,
                                myPeriodic,
                                myPoles,
                                WeightsRef(),
                                myKnots,
                                myMults,
                                npoles,
                                myRational ? nweights : BSplCLib.NoWeights(),
                                nknots,
                                nmults);

        myDeg = Degree;
        myPoles = npoles;
        if (IsRational())
        {
            myWeights = nweights;
        }
        else
        {
            myWeights = BSplCLib.UnitWeights(myPoles.Length());
        }
        myKnots = nknots;
        myMults = nmults;
        updateKnots();
    }

    /// <summary>Raises the multiplicity of the knot of index Index to M (does nothing if it is already M or more).</summary>
    /// <exception cref="Standard_OutOfRange">Index is outside the knot range.</exception>
    internal void IncreaseMultiplicity(int Index, int M)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(1, 1);
        k[1] = myKnots.Value(Index);
        NCollection_Array1<int> m = new NCollection_Array1<int>(1, 1);
        m[1] = M - myMults.Value(Index);
        InsertKnots(k, m, Epsilon(1.0), true);
    }

    /// <summary>Raises the multiplicities of the knots of indexes I1 to I2 to M.</summary>
    internal void IncreaseMultiplicity(int I1, int I2, int M)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(myKnots, I1, I1, I2);
        NCollection_Array1<int> m = new NCollection_Array1<int>(I1, I2);
        int i;
        for (i = I1; i <= I2; i++)
        {
            m[i] = M - myMults.Value(i);
        }
        InsertKnots(k, m, Epsilon(1.0), true);
    }

    /// <summary>Increments the multiplicities of the knots of indexes I1 to I2 by M.</summary>
    internal void IncrementMultiplicity(int I1, int I2, int Step)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(myKnots, I1, I1, I2);
        NCollection_Array1<int> m = new NCollection_Array1<int>(I1, I2);
        m.Init(Step);
        InsertKnots(k, m, Epsilon(1.0), true);
    }

    /// <summary>
    /// Inserts the knot U with the multiplicity M, or raises the multiplicity of an existing knot within
    /// ParametricTolerance of U to M.
    /// </summary>
    internal void InsertKnot(double U, int M = 1, double ParametricTolerance = 0.0)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(1, 1);
        k[1] = U;
        NCollection_Array1<int> m = new NCollection_Array1<int>(1, 1);
        m[1] = M;
        InsertKnots(k, m, ParametricTolerance);
    }

    /// <summary>
    /// Inserts the knots Knots with the multiplicities Mults. A knot within ParametricTolerance of an existing
    /// knot raises its multiplicity: to the given one if Add is false, by the given one if Add is true.
    /// </summary>
    /// <exception cref="Standard_ConstructionError">The insertion is not possible (BSplCLib::PrepareInsertKnots).</exception>
    internal void InsertKnots(NCollection_Array1<double> Knots,
                              NCollection_Array1<int> Mults,
                              double ParametricTolerance = 0.0,
                              bool Add = false)
    {
        CheckMutable();
        // Check and compute new sizes
        int nbpoles, nbknots;

        if (!BSplCLib.PrepareInsertKnots(myDeg,
                                         myPeriodic,
                                         myKnots,
                                         myMults,
                                         Knots,
                                         Mults,
                                         out nbpoles,
                                         out nbknots,
                                         ParametricTolerance,
                                         Add))
        {
            throw new Standard_ConstructionError("Geom2d_BSplineCurve::InsertKnots");
        }

        if (nbpoles == myPoles.Length())
        {
            return;
        }

        NCollection_Array1<gp_Pnt2d> npoles = new NCollection_Array1<gp_Pnt2d>(1, nbpoles);
        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, nbknots);
        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, nbknots);
        NCollection_Array1<double> nweights = new NCollection_Array1<double>();

        if (myRational)
        {
            nweights.Resize(1, nbpoles, false);
        }

        BSplCLib.InsertKnots(myDeg,
                             myPeriodic,
                             myPoles,
                             WeightsRef(),
                             myKnots,
                             myMults,
                             Knots,
                             Mults,
                             npoles,
                             myRational ? nweights : BSplCLib.NoWeights(),
                             nknots,
                             nmults,
                             ParametricTolerance,
                             Add);

        if (myRational)
        {
            myWeights = nweights;
        }
        else
        {
            myWeights = BSplCLib.UnitWeights(npoles.Length());
        }
        myPoles = npoles;
        myKnots = nknots;
        myMults = nmults;
        updateKnots();
    }

    /// <summary>
    /// Lowers the multiplicity of the knot of index Index to M (0 removes it) if the curve stays within
    /// Tolerance of itself; returns false, leaving the curve unchanged, if not.
    /// </summary>
    /// <exception cref="Standard_OutOfRange">Index is outside [FirstUKnotIndex(), LastUKnotIndex()].</exception>
    internal bool RemoveKnot(int Index, int M, double Tolerance)
    {
        CheckMutable();
        if (M < 0)
        {
            return true;
        }

        int I1 = FirstUKnotIndex();
        int I2 = LastUKnotIndex();

        if (Index < I1 || Index > I2)
        {
            throw new Standard_OutOfRange("BSpline curve: RemoveKnot: index out of range");
        }

        NCollection_Array1<gp_Pnt2d> oldpoles = myPoles;

        int step = myMults.Value(Index) - M;
        if (step <= 0)
        {
            return true;
        }

        NCollection_Array1<gp_Pnt2d> npoles = new NCollection_Array1<gp_Pnt2d>(1, oldpoles.Length() - step);

        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, myKnots.Length() - (M == 0 ? 1 : 0));
        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, myMults.Length() - (M == 0 ? 1 : 0));

        NCollection_Array1<double> nweights = new NCollection_Array1<double>();
        if (IsRational())
        {
            nweights.Resize(1, npoles.Length(), false);
        }

        if (!BSplCLib.RemoveKnot(Index,
                                 M,
                                 myDeg,
                                 myPeriodic,
                                 myPoles,
                                 WeightsRef(),
                                 myKnots,
                                 myMults,
                                 npoles,
                                 myRational ? nweights : BSplCLib.NoWeights(),
                                 nknots,
                                 nmults,
                                 Tolerance))
        {
            return false;
        }

        if (IsRational())
        {
            myWeights = nweights;
        }
        else
        {
            myWeights = BSplCLib.UnitWeights(npoles.Length());
        }
        myPoles = npoles;
        myKnots = nknots;
        myMults = nmults;

        updateKnots();
        myMaxDerivInvOk = false;
        return true;
    }

    /// <summary>
    /// Inserts the pole P (weight Weight) after the pole of index Index, adding a knot at the end of a uniform
    /// or quasi-uniform knot sequence.
    /// </summary>
    /// <exception cref="Standard_OutOfRange">Index is outside [0, NbPoles()].</exception>
    /// <exception cref="Standard_ConstructionError">
    /// Weight is not above gp::Resolution(), or the knots are non-uniform or piecewise Bezier.
    /// </exception>
    internal void InsertPoleAfter(int Index, in gp_Pnt2d P, double Weight = 1.0)
    {
        CheckMutable();
        if (Index < 0 || Index > myPoles.Length())
        {
            throw new Standard_OutOfRange("BSpline curve: InsertPoleAfter: Index and #pole mismatch");
        }

        if (Weight <= gp.Resolution())
        {
            throw new Standard_ConstructionError("BSpline curve: InsertPoleAfter: Weight too small");
        }

        if (myKnotSet == GeomAbs_NonUniform || myKnotSet == GeomAbs_PiecewiseBezier)
        {
            throw new Standard_ConstructionError("BSpline curve: InsertPoleAfter: bad knotSet type");
        }

        NCollection_Array1<double> cknots = myKnots;
        int nbknots = cknots.Length();

        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, nbknots + 1);

        int i;
        for (i = 1; i <= nbknots; i++)
        {
            nknots[i] = cknots[i];
        }

        nknots[nbknots + 1] = 2 * nknots[nbknots] - nknots[nbknots - 1];

        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, nbknots + 1);
        NCollection_Array1<int> cmults = myMults;

        for (i = 2; i <= nbknots; i++)
        {
            nmults[i] = 1;
        }
        nmults[1] = cmults[1];
        nmults[nbknots + 1] = cmults[nbknots];

        NCollection_Array1<gp_Pnt2d> cpoles = myPoles;
        int nbpoles = cpoles.Length();
        NCollection_Array1<gp_Pnt2d> npoles = new NCollection_Array1<gp_Pnt2d>(1, nbpoles + 1);

        // insert the pole

        for (i = 1; i <= Index; i++)
        {
            npoles[i] = cpoles[i];
        }

        npoles[Index + 1] = P;

        for (i = Index + 1; i <= nbpoles; i++)
        {
            npoles[i + 1] = cpoles[i];
        }

        // Insert the weight

        NCollection_Array1<double> nweights = new NCollection_Array1<double>();
        bool rat = IsRational() || Math.Abs(Weight - 1.0) > gp.Resolution();

        if (rat)
        {
            nweights.Resize(1, nbpoles + 1, false);

            for (i = 1; i <= Index; i++)
            {
                if (IsRational())
                {
                    nweights[i] = myWeights.Value(i);
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
                    nweights[i + 1] = myWeights.Value(i);
                }
                else
                {
                    nweights[i + 1] = 1.0;
                }
            }
        }

        myPoles = npoles;
        if (rat)
        {
            myWeights = nweights;
        }
        else
        {
            myWeights = BSplCLib.UnitWeights(myPoles.Length());
        }
        myRational = rat;
        myKnots = nknots;
        myMults = nmults;
        myMaxDerivInvOk = false;
        updateKnots();
    }

    /// <summary>Inserts the pole P (weight Weight) before the pole of index Index.</summary>
    internal void InsertPoleBefore(int Index, in gp_Pnt2d P, double Weight = 1.0)
    {
        CheckMutable();
        InsertPoleAfter(Index - 1, P, Weight);
    }

    /// <summary>Removes the pole of index Index, and the last knot (uniform or quasi-uniform knots only).</summary>
    /// <exception cref="Standard_OutOfRange">Index is outside [1, NbPoles()].</exception>
    /// <exception cref="Standard_ConstructionError">
    /// The curve has only 2 poles, or the knots are non-uniform or piecewise Bezier.
    /// </exception>
    internal void RemovePole(int Index)
    {
        CheckMutable();
        if (Index < 1 || Index > myPoles.Length())
        {
            throw new Standard_OutOfRange("BSpline curve: RemovePole: Index and #pole mismatch");
        }

        if (myPoles.Length() <= 2)
        {
            throw new Standard_ConstructionError("BSpline curve: RemovePole: #pole is already minimum");
        }

        if (myKnotSet == GeomAbs_NonUniform || myKnotSet == GeomAbs_PiecewiseBezier)
        {
            throw new Standard_ConstructionError("BSpline curve: RemovePole: bad knotSet type");
        }

        int i;
        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, myKnots.Length() - 1);

        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, myMults.Length() - 1);

        for (i = 1; i < nknots.Length(); i++)
        {
            nknots[i] = myKnots.Value(i);
            nmults[i] = 1;
        }
        nmults[1] = myMults.Value(1);
        nknots[nknots.Upper()] = myKnots.Value(myKnots.Upper());
        nmults[nmults.Upper()] = myMults.Value(myMults.Upper());

        NCollection_Array1<gp_Pnt2d> npoles = new NCollection_Array1<gp_Pnt2d>(1, myPoles.Upper() - 1);

        for (i = 1; i < Index; i++)
        {
            npoles[i] = myPoles.Value(i);
        }
        for (i = Index; i <= npoles.Length(); i++)
        {
            npoles[i] = myPoles.Value(i + 1);
        }

        NCollection_Array1<double> nweights = new NCollection_Array1<double>();
        if (IsRational())
        {
            nweights.Resize(1, npoles.Length(), false);
            for (i = 1; i < Index; i++)
            {
                nweights[i] = myWeights.Value(i);
            }
            for (i = Index; i <= nweights.Length(); i++)
            {
                nweights[i] = myWeights.Value(i + 1);
            }
        }

        myPoles = npoles;
        if (IsRational())
        {
            myWeights = nweights;
            myRational = Rational(myWeights);
            if (!myRational)
            {
                myWeights = BSplCLib.UnitWeights(myPoles.Length());
            }
        }
        else
        {
            myWeights = BSplCLib.UnitWeights(myPoles.Length());
        }
        myKnots = nknots;
        myMults = nmults;
        updateKnots();
    }

    /// <summary>Reverses the curve: knots, multiplicities, poles and weights; the parameter range is kept.</summary>
    protected internal sealed override void Reverse()
    {
        CheckMutable();
        BSplCLib.Reverse(myKnots);
        BSplCLib.Reverse(myMults);
        int last;
        if (myPeriodic)
        {
            last = myFlatKnots.Upper() - myDeg - 1;
        }
        else
        {
            last = myPoles.Upper();
        }
        BSplCLib.Reverse(myPoles, last);
        if (myRational)
        {
            BSplCLib.Reverse(myWeights, last);
        }
        updateKnots();
    }

    /// <summary>FirstParameter() + LastParameter() - U.</summary>
    public sealed override double ReversedParameter(double U) => (FirstParameter() + LastParameter() - U);

    /// <summary>
    /// Keeps the part of the curve between U1 and U2 (knots within theTolerance are merged). A periodic curve
    /// becomes non-periodic; U2 - U1 may not exceed its period.
    /// </summary>
    /// <exception cref="Standard_DomainError">U2 &lt; U1, or U2 - U1 exceeds the period of a periodic curve.</exception>
    internal void Segment(double aU1, double aU2, double theTolerance = 1.0e-9)
    {
        CheckMutable();
        if (aU2 < aU1)
        {
            throw new Standard_DomainError("Geom2d_BSplineCurve::Segment");
        }

        double NewU1, NewU2;
        double U, DU = 0, aDDU = 0;
        int i, k, index;
        bool wasPeriodic = myPeriodic;
        double U1 = aU1, U2 = aU2;

        NCollection_Array1<double> Knots = new NCollection_Array1<double>(1, 2);
        NCollection_Array1<int> Mults = new NCollection_Array1<int>(1, 2);

        // define param distance to keep (eap, Apr 18 2002, occ311)
        if (myPeriodic)
        {
            double Period = LastParameter() - FirstParameter();
            DU = U2 - U1;
            if (DU - Period > Precision.PConfusion())
            {
                throw new Standard_DomainError("Geom2d_BSplineCurve::Segment");
            }
            if (DU > Period)
            {
                DU = Period;
            }
            aDDU = DU;
        }

        index = 0;
        BSplCLib.LocateParameter(myDeg,
                                 myKnots,
                                 myMults,
                                 U1,
                                 myPeriodic,
                                 myKnots.Lower(),
                                 myKnots.Upper(),
                                 out index,
                                 out NewU1);
        index = 0;
        BSplCLib.LocateParameter(myDeg,
                                 myKnots,
                                 myMults,
                                 U2,
                                 myPeriodic,
                                 myKnots.Lower(),
                                 myKnots.Upper(),
                                 out index,
                                 out NewU2);

        //-- DBB
        double aNu2 = NewU2;
        //-- DBB

        double AbsUMax = Max(Math.Abs(NewU1), Math.Abs(NewU2));
        AbsUMax = Max(AbsUMax, Max(Math.Abs(FirstParameter()), Math.Abs(LastParameter())));

        double Eps = Max(Epsilon(AbsUMax), theTolerance);

        Knots[1] = Min(NewU1, NewU2);
        Knots[2] = Max(NewU1, NewU2);
        Mults[1] = Mults[2] = myDeg;
        InsertKnots(Knots, Mults, Eps, false);

        if (myPeriodic)
        { // set the origine at NewU1
            index = 0;
            BSplCLib.LocateParameter(myDeg,
                                     myKnots,
                                     myMults,
                                     U1,
                                     myPeriodic,
                                     myKnots.Lower(),
                                     myKnots.Upper(),
                                     out index,
                                     out U);
            if (Math.Abs(myKnots.Value(index + 1) - U) <= Eps)
            {
                index++;
            }
            SetOrigin(index);
            SetNotPeriodic();
            NewU2 = NewU1 + DU;
        }

        // compute index1 and index2 to set the new knots and mults
        int index1 = 0, index2 = 0;
        int FromU1 = myKnots.Lower();
        int ToU2 = myKnots.Upper();
        BSplCLib.LocateParameter(myDeg, myKnots, myMults, NewU1, myPeriodic, FromU1, ToU2, out index1, out U);
        if (Math.Abs(myKnots.Value(index1 + 1) - U) <= Eps)
        {
            index1++;
        }

        BSplCLib.LocateParameter(myDeg, myKnots, myMults, NewU2, myPeriodic, FromU1, ToU2, out index2, out U);
        if (Math.Abs(myKnots.Value(index2 + 1) - U) <= Eps || index2 == index1)
        {
            index2++;
        }

        int nbknots = index2 - index1 + 1;

        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, nbknots);
        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, nbknots);

        // to restore changed U1
        if (DU > 0)
        { // if was periodic
            DU = NewU1 - U1;
        }
        //
        k = 1;
        //
        for (i = index1; i <= index2; i++)
        {
            nknots.SetValue(k, myKnots.Value(i) - DU);
            nmults.SetValue(k, myMults.Value(i));
            k++;
        }
        nmults.SetValue(1, myDeg + 1);
        nmults.SetValue(nbknots, myDeg + 1);

        // compute index1 and index2 to set the new poles and weights
        int pindex1 = BSplCLib.PoleIndex(myDeg, index1, myPeriodic, myMults);
        int pindex2 = BSplCLib.PoleIndex(myDeg, index2, myPeriodic, myMults);

        pindex1++;
        pindex2 = Min(pindex2 + 1, myPoles.Length());

        int nbpoles = pindex2 - pindex1 + 1;

        NCollection_Array1<double> nweights = new NCollection_Array1<double>();
        NCollection_Array1<gp_Pnt2d> npoles = new NCollection_Array1<gp_Pnt2d>(1, nbpoles);

        k = 1;
        if (myRational)
        {
            nweights.Resize(1, nbpoles, false);
            for (i = pindex1; i <= pindex2; i++)
            {
                npoles.SetValue(k, myPoles.Value(i));
                nweights.SetValue(k, myWeights.Value(i));
                k++;
            }
        }
        else
        {
            for (i = pindex1; i <= pindex2; i++)
            {
                npoles.SetValue(k, myPoles.Value(i));
                k++;
            }
        }

        //-- DBB
        if (wasPeriodic)
        {
            nknots.ChangeValue(nknots.Lower()) = U1;
            if (aNu2 < U2)
            {
                nknots.ChangeValue(nknots.Upper()) = U1 + aDDU;
            }
        }
        //-- DBB

        myKnots = nknots;
        myMults = nmults;
        myPoles = npoles;
        if (myRational)
        {
            myWeights = nweights;
        }
        else
        {
            myWeights = BSplCLib.UnitWeights(myPoles.Length());
        }

        myMaxDerivInvOk = false;
        updateKnots();
    }

    /// <summary>Changes the value of the knot of index Index; the knots must stay increasing.</summary>
    /// <exception cref="Standard_OutOfRange">Index is outside [1, NbKnots()].</exception>
    /// <exception cref="Standard_ConstructionError">K is not strictly between the neighbouring knots.</exception>
    internal void SetKnot(int Index, double K)
    {
        CheckMutable();
        if (Index < 1 || Index > myKnots.Length())
        {
            throw new Standard_OutOfRange("BSpline curve: SetKnot: Index and #knots mismatch");
        }
        double DK = Math.Abs(Epsilon(K));
        if (Index == 1)
        {
            if (K >= myKnots.Value(2) - DK)
            {
                throw new Standard_ConstructionError("BSpline curve: SetKnot: K out of range");
            }
        }
        else if (Index == myKnots.Length())
        {
            if (K <= myKnots.Value(myKnots.Length() - 1) + DK)
            {
                throw new Standard_ConstructionError("BSpline curve: SetKnot: K out of range");
            }
        }
        else
        {
            if (K <= myKnots.Value(Index - 1) + DK || K >= myKnots.Value(Index + 1) - DK)
            {
                throw new Standard_ConstructionError("BSpline curve: SetKnot: K out of range");
            }
        }
        if (K != myKnots.Value(Index))
        {
            myKnots.SetValue(Index, K);
            myMaxDerivInvOk = false;
            updateKnots();
        }
    }

    /// <summary>Changes all the knot values.</summary>
    /// <exception cref="Standard_ConstructionError">The knots are not increasing or don't match the multiplicities.</exception>
    internal void SetKnots(NCollection_Array1<double> K)
    {
        CheckMutable();
        CheckCurveData(myPoles, K, myMults, myDeg, myPeriodic);
        myKnots.CopyValues(K);
        myMaxDerivInvOk = false;
        updateKnots();
    }

    /// <summary>Raises the multiplicity of the knot of index Index to M, then changes its value to K.</summary>
    internal void SetKnot(int Index, double K, int M)
    {
        CheckMutable();
        IncreaseMultiplicity(Index, M);
        SetKnot(Index, K);
    }

    /// <summary>
    /// Brings the parameter of a periodic curve into [FirstParameter(), LastParameter()] by adding or
    /// subtracting periods; does nothing on a non-periodic curve.
    /// </summary>
    public void PeriodicNormalization(ref double Parameter)
    {
        double Period;

        if (myPeriodic)
        {
            Period = myFlatKnots.Value(myFlatKnots.Upper() - myDeg) - myFlatKnots.Value(myDeg + 1);
            while (Parameter > myFlatKnots.Value(myFlatKnots.Upper() - myDeg))
            {
                Parameter -= Period;
            }
            while (Parameter < myFlatKnots.Value((myDeg + 1)))
            {
                Parameter += Period;
            }
        }
    }

    /// <summary>Makes the curve periodic, keeping the knots between FirstUKnotIndex() and LastUKnotIndex().</summary>
    internal void SetPeriodic()
    {
        CheckMutable();
        int first = FirstUKnotIndex();
        int last = LastUKnotIndex();

        NCollection_Array1<double> cknots = new NCollection_Array1<double>(1, last - first + 1);
        for (int k = first; k <= last; k++)
        {
            cknots[k - first + 1] = myKnots[k];
        }
        myKnots = cknots;

        NCollection_Array1<int> cmults = new NCollection_Array1<int>(1, last - first + 1);
        for (int k = first; k <= last; k++)
        {
            cmults[k - first + 1] = myMults[k];
        }
        cmults[1] = cmults[cmults.Upper()] = Min(myDeg, Max(cmults[1], cmults[cmults.Upper()]));
        myMults = cmults;

        // compute new number of poles;
        int nbp = BSplCLib.NbPoles(myDeg, true, myMults);

        myPoles.Resize(1, nbp, true);
        if (myRational)
        {
            myWeights.Resize(1, nbp, true);
        }
        else
        {
            myWeights = BSplCLib.UnitWeights(nbp);
        }

        myPeriodic = true;

        myMaxDerivInvOk = false;
        updateKnots();
    }

    /// <summary>Sets the origin of a periodic curve at the knot of index Index.</summary>
    /// <exception cref="Standard_NoSuchObject">The curve is not periodic.</exception>
    /// <exception cref="Standard_DomainError">Index is outside [FirstUKnotIndex(), LastUKnotIndex()].</exception>
    internal void SetOrigin(int Index)
    {
        CheckMutable();
        if (!myPeriodic)
        {
            throw new Standard_NoSuchObject("Geom2d_BSplineCurve::SetOrigin");
        }

        int i, k;
        int first = FirstUKnotIndex();
        int last = LastUKnotIndex();

        if ((Index < first) || (Index > last))
        {
            throw new Standard_DomainError("Geom2d_BSplineCurve::SetOrigin");
        }

        int nbknots = myKnots.Length();
        int nbpoles = myPoles.Length();

        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, nbknots);

        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, nbknots);

        // set the knots and mults
        double period = myKnots.Value(last) - myKnots.Value(first);
        k = 1;
        for (i = Index; i <= last; i++)
        {
            nknots[k] = myKnots.Value(i);
            nmults[k] = myMults.Value(i);
            k++;
        }
        for (i = first + 1; i <= Index; i++)
        {
            nknots[k] = myKnots.Value(i) + period;
            nmults[k] = myMults.Value(i);
            k++;
        }

        int index = 1;
        for (i = first + 1; i <= Index; i++)
        {
            index += myMults.Value(i);
        }

        // set the poles and weights
        NCollection_Array1<gp_Pnt2d> npoles = new NCollection_Array1<gp_Pnt2d>(1, nbpoles);
        NCollection_Array1<double> nweights = new NCollection_Array1<double>();
        first = myPoles.Lower();
        last = myPoles.Upper();
        if (myRational)
        {
            nweights.Resize(1, nbpoles, false);
            k = 1;
            for (i = index; i <= last; i++)
            {
                npoles[k] = myPoles.Value(i);
                nweights[k] = myWeights.Value(i);
                k++;
            }
            for (i = first; i < index; i++)
            {
                npoles[k] = myPoles.Value(i);
                nweights[k] = myWeights.Value(i);
                k++;
            }
        }
        else
        {
            k = 1;
            for (i = index; i <= last; i++)
            {
                npoles[k] = myPoles.Value(i);
                k++;
            }
            for (i = first; i < index; i++)
            {
                npoles[k] = myPoles.Value(i);
                k++;
            }
        }

        myPoles = npoles;
        myKnots = nknots;
        myMults = nmults;
        if (myRational)
        {
            myWeights = nweights;
        }
        else
        {
            myWeights = BSplCLib.UnitWeights(nbpoles);
        }
        myMaxDerivInvOk = false;
        updateKnots();
    }

    /// <summary>Makes a periodic curve non-periodic (with the same shape); does nothing on a non-periodic curve.</summary>
    internal void SetNotPeriodic()
    {
        CheckMutable();
        if (myPeriodic)
        {
            int NbKnots, NbPoles;
            BSplCLib.PrepareUnperiodize(myDeg, myMults, out NbKnots, out NbPoles);

            NCollection_Array1<gp_Pnt2d> npoles = new NCollection_Array1<gp_Pnt2d>(1, NbPoles);

            NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, NbKnots);

            NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, NbKnots);

            NCollection_Array1<double> nweights = new NCollection_Array1<double>();

            if (IsRational())
            {
                nweights.Resize(1, NbPoles, false);
            }

            BSplCLib.Unperiodize(myDeg,
                                 myMults,
                                 myKnots,
                                 myPoles,
                                 WeightsRef(),
                                 nmults,
                                 nknots,
                                 npoles,
                                 myRational ? nweights : BSplCLib.NoWeights());

            myPoles = npoles;
            if (IsRational())
            {
                myWeights = nweights;
            }
            else
            {
                myWeights = BSplCLib.UnitWeights(myPoles.Length());
            }
            myMults = nmults;
            myKnots = nknots;
            myPeriodic = false;
            myMaxDerivInvOk = false;

            updateKnots();
        }
    }

    /// <summary>Replaces the pole of index Index by P.</summary>
    /// <exception cref="Standard_OutOfRange">Index is outside [1, NbPoles()].</exception>
    internal void SetPole(int Index, in gp_Pnt2d P)
    {
        CheckMutable();
        if (Index < 1 || Index > myPoles.Length())
        {
            throw new Standard_OutOfRange("BSpline curve: SetPole: index and #pole mismatch");
        }
        myPoles.SetValue(Index, P);
        myMaxDerivInvOk = false;
    }

    /// <summary>Replaces the pole of index Index by P, and its weight by W.</summary>
    internal void SetPole(int Index, in gp_Pnt2d P, double W)
    {
        CheckMutable();
        SetPole(Index, P);
        SetWeight(Index, W);
    }

    /// <summary>Changes the weight of the pole of index Index; the curve may become rational or non-rational.</summary>
    /// <exception cref="Standard_OutOfRange">Index is outside [1, NbPoles()].</exception>
    /// <exception cref="Standard_ConstructionError">W is not above gp::Resolution().</exception>
    internal void SetWeight(int Index, double W)
    {
        CheckMutable();
        if (Index < 1 || Index > myPoles.Length())
        {
            throw new Standard_OutOfRange("BSpline curve: SetWeight: Index and #pole mismatch");
        }

        if (W <= gp.Resolution())
        {
            throw new Standard_ConstructionError("BSpline curve: SetWeight: Weight too small");
        }

        bool rat = IsRational() || (Math.Abs(W - 1.0) > gp.Resolution());

        if (rat)
        {
            // Becoming rational from non-rational: copy non-owning view to owned array.
            if (!IsRational())
            {
                myWeights = new NCollection_Array1<double>(myWeights);
            }

            myWeights.SetValue(Index, W);

            if (IsRational())
            {
                rat = Rational(myWeights);
                if (!rat)
                {
                    myWeights = BSplCLib.UnitWeights(myPoles.Length());
                }
            }

            myRational = rat;
        }

        myMaxDerivInvOk = false;
    }

    /// <summary>
    /// Moves the point of parameter U to P by changing the poles of indexes Index1 to Index2.
    /// FirstModifiedPole and LastModifiedPole are the poles actually changed; FirstModifiedPole is 0 if the
    /// point couldn't be moved.
    /// </summary>
    /// <exception cref="Standard_OutOfRange">Index1 or Index2 is outside [1, NbPoles()], or Index1 &gt; Index2.</exception>
    internal void MovePoint(double U,
                            in gp_Pnt2d P,
                            int Index1,
                            int Index2,
                            out int FirstModifiedPole,
                            out int LastmodifiedPole)
    {
        CheckMutable();
        if (Index1 < 1 || Index1 > myPoles.Length() || Index2 < 1 || Index2 > myPoles.Length()
            || Index1 > Index2)
        {
            throw new Standard_OutOfRange("BSpline curve: MovePoint: Index and #pole mismatch");
        }
        NCollection_Array1<gp_Pnt2d> npoles = new NCollection_Array1<gp_Pnt2d>(1, myPoles.Length());
        gp_Pnt2d P0;
        D0(U, out P0);
        gp_Vec2d Displ = new gp_Vec2d(P0, P);
        BSplCLib.MovePoint(U,
                           Displ,
                           Index1,
                           Index2,
                           myDeg,
                           myPoles,
                           WeightsRef(),
                           myFlatKnots,
                           out FirstModifiedPole,
                           out LastmodifiedPole,
                           npoles);
        if (FirstModifiedPole != 0)
        {
            myPoles = npoles;
            myMaxDerivInvOk = false;
        }
    }

    /// <summary>
    /// Moves the point of parameter U to P and its tangent to Tangent (a periodic curve becomes non-periodic).
    /// ErrorStatus is 0 on success; the curve is unchanged otherwise.
    /// </summary>
    internal void MovePointAndTangent(double U,
                                      in gp_Pnt2d P,
                                      in gp_Vec2d Tangent,
                                      double Tolerance,
                                      int StartingCondition,
                                      int EndingCondition,
                                      out int ErrorStatus)
    {
        CheckMutable();
        int ii;
        if (IsPeriodic())
        {
            //
            // for the time being do not deal with periodic curves
            //
            SetNotPeriodic();
        }
        NCollection_Array1<gp_Pnt2d> new_poles = new NCollection_Array1<gp_Pnt2d>(1, myPoles.Length());
        gp_Pnt2d P0;

        gp_Vec2d delta_derivative;
        D1(U, out P0, out delta_derivative);
        gp_Vec2d delta = new gp_Vec2d(P0, P);
        for (ii = 1; ii <= 2; ii++)
        {
            delta_derivative.SetCoord(ii, Tangent.Coord(ii) - delta_derivative.Coord(ii));
        }
        BSplCLib.MovePointAndTangent(U,
                                     delta,
                                     delta_derivative,
                                     Tolerance,
                                     myDeg,
                                     StartingCondition,
                                     EndingCondition,
                                     myPoles,
                                     WeightsRef(),
                                     myFlatKnots,
                                     new_poles,
                                     out ErrorStatus);
        if (ErrorStatus == 0)
        {
            myPoles = new_poles;
            myMaxDerivInvOk = false;
        }
    }

    /// <summary>Recomputes the knot distribution, flat knots and continuity after a change of knots.</summary>
    protected void updateKnots()
    {
        myMaxDerivInvOk = false;
        int MaxKnotMult = 0;
        BSplCLib.KnotAnalysis(myDeg, myPeriodic, myKnots, myMults, out myKnotSet, out MaxKnotMult);

        if (myKnotSet == GeomAbs_Uniform && !myPeriodic)
        {
            myFlatKnots.Resize(myKnots.Lower(), myKnots.Upper(), false);
            myFlatKnots.Assign(myKnots);
        }
        else
        {
            myFlatKnots.Resize(1, BSplCLib.KnotSequenceLength(myMults, myDeg, myPeriodic), false);

            BSplCLib.KnotSequence(myKnots, myMults, myDeg, myPeriodic, myFlatKnots);
        }

        if (MaxKnotMult == 0)
        {
            mySmooth = GeomAbs_CN;
        }
        else
        {
            switch (myDeg - MaxKnotMult)
            {
                case 0:
                    mySmooth = GeomAbs_C0;
                    break;
                case 1:
                    mySmooth = GeomAbs_C1;
                    break;
                case 2:
                    mySmooth = GeomAbs_C2;
                    break;
                case 3:
                    mySmooth = GeomAbs_C3;
                    break;
                default:
                    mySmooth = GeomAbs_C3;
                    break;
            }
        }
    }

    // Geom2d_BSplineCurve_1.cxx

    /// <summary>True if the curve is CN (from the multiplicities of its interior knots).</summary>
    /// <exception cref="Standard_RangeError">N &lt; 0.</exception>
    public sealed override bool IsCN(int N)
    {
        Standard_RangeError_Raise_if(N < 0, "Geom2d_BSplineCurve::IsCN");

        switch (mySmooth)
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
                return N <= 3
                         ? true
                         : N <= myDeg
                                  - BSplCLib.MaxKnotMult(myMults, myMults.Lower() + 1, myMults.Upper() - 1);
            default:
                return false;
        }
    }

    /// <summary>
    /// True if the curve is G1 on [theTf, theTl]: at every interior knot of multiplicity Degree or more, the
    /// left and right tangents are non-null and within theAngTol radians (and, for a periodic curve whose range
    /// meets the bounds, at the start/end point too).
    /// </summary>
    public bool IsG1(double theTf, double theTl, double theAngTol)
    {
        if (IsCN(1))
        {
            return true;
        }

        int start = FirstUKnotIndex() + 1, finish = LastUKnotIndex() - 1;
        int aDeg = Degree();
        for (int aNKnot = start; aNKnot <= finish; aNKnot++)
        {
            double aTpar = Knot(aNKnot);

            if (aTpar < theTf)
            {
                continue;
            }
            if (aTpar > theTl)
            {
                break;
            }

            int mult = Multiplicity(aNKnot);
            if (mult < aDeg)
            {
                continue;
            }

            gp_Pnt2d aP1, aP2;
            gp_Vec2d aV1, aV2;
            LocalD1(aTpar, aNKnot - 1, aNKnot, out aP1, out aV1);
            LocalD1(aTpar, aNKnot, aNKnot + 1, out aP2, out aV2);

            if ((aV1.SquareMagnitude() <= gp.Resolution()) || aV2.SquareMagnitude() <= gp.Resolution())
            {
                return false;
            }

            if (Math.Abs(aV1.Angle(aV2)) > theAngTol)
            {
                return false;
            }
        }

        if (!IsPeriodic())
        {
            return true;
        }

        double aFirstParam = FirstParameter(), aLastParam = LastParameter();

        if (((aFirstParam - theTf) * (theTl - aFirstParam) < 0.0)
            && ((aLastParam - theTf) * (theTl - aLastParam) < 0.0))
        {
            // Range [theTf, theTl] does not intersect curve boundaries
            return true;
        }

        // Curve is closed or periodic and range [theTf, theTl]
        // intersect curve boundary. Therefore, it is necessary to
        // check if curve is smooth in its first and last point.

        gp_Pnt2d aP;
        gp_Vec2d aV1b, aV2b;
        D1(Knot(FirstUKnotIndex()), out aP, out aV1b);
        D1(Knot(LastUKnotIndex()), out aP, out aV2b);

        if ((aV1b.SquareMagnitude() <= gp.Resolution()) || aV2b.SquareMagnitude() <= gp.Resolution())
        {
            return false;
        }

        if (Math.Abs(aV1b.Angle(aV2b)) > theAngTol)
        {
            return false;
        }

        return true;
    }

    /// <summary>True if the start and end points are within Precision::Computational() (squared distance).</summary>
    public sealed override bool IsClosed() => StartPoint().SquareDistance(EndPoint()) <= Precision.Computational();

    /// <summary>True if the curve is periodic.</summary>
    public sealed override bool IsPeriodic() => myPeriodic;

    /// <summary>The global continuity, from the highest interior knot multiplicity (C3 at most, or CN).</summary>
    public sealed override GeomAbs_Shape Continuity() => mySmooth;

    /// <summary>The degree.</summary>
    public int Degree() => myDeg;

    public sealed override gp_Pnt2d EvalD0(double U)
    {
        gp_Pnt2d aEvalRepResult;
        if (Geom2dEval_RepUtils.TryEvalCurveD0(myEvalRep, U, out aEvalRepResult))
        {
            return aEvalRepResult;
        }
        gp_Pnt2d P;
        int aSpanIndex = 0;
        double aNewU = U;
        PeriodicNormalization(ref aNewU);
        BSplCLib.LocateParameter(myDeg, myKnots, myMults, U, myPeriodic, ref aSpanIndex, out aNewU);
        if (aNewU < myKnots.Value(aSpanIndex))
        {
            aSpanIndex--;
        }

        BSplCLib.D0(aNewU, aSpanIndex, myDeg, myPeriodic, myPoles, WeightsRef(), myKnots, myMults, out P);
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
        int aSpanIndex = 0;
        double aNewU = U;
        PeriodicNormalization(ref aNewU);
        BSplCLib.LocateParameter(myDeg, myKnots, myMults, U, myPeriodic, ref aSpanIndex, out aNewU);
        if (aNewU < myKnots.Value(aSpanIndex))
        {
            aSpanIndex--;
        }
        BSplCLib.D1(aNewU,
                    aSpanIndex,
                    myDeg,
                    myPeriodic,
                    myPoles,
                    WeightsRef(),
                    myKnots,
                    myMults,
                    out aResult.Point,
                    out aResult.D1);
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
        int aSpanIndex = 0;
        double aNewU = U;
        PeriodicNormalization(ref aNewU);
        BSplCLib.LocateParameter(myDeg, myKnots, myMults, U, myPeriodic, ref aSpanIndex, out aNewU);
        if (aNewU < myKnots.Value(aSpanIndex))
        {
            aSpanIndex--;
        }
        BSplCLib.D2(aNewU,
                    aSpanIndex,
                    myDeg,
                    myPeriodic,
                    myPoles,
                    WeightsRef(),
                    myKnots,
                    myMults,
                    out aResult.Point,
                    out aResult.D1,
                    out aResult.D2);
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
        int aSpanIndex = 0;
        double aNewU = U;
        PeriodicNormalization(ref aNewU);
        BSplCLib.LocateParameter(myDeg, myKnots, myMults, U, myPeriodic, ref aSpanIndex, out aNewU);
        if (aNewU < myKnots.Value(aSpanIndex))
        {
            aSpanIndex--;
        }
        BSplCLib.D3(aNewU,
                    aSpanIndex,
                    myDeg,
                    myPeriodic,
                    myPoles,
                    WeightsRef(),
                    myKnots,
                    myMults,
                    out aResult.Point,
                    out aResult.D1,
                    out aResult.D2,
                    out aResult.D3);
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
        BSplCLib.DN(U, N, 0, myDeg, myPeriodic, myPoles, WeightsRef(), myFlatKnots, BSplCLib.NoMults(), out V);
        return V;
    }

    /// <summary>The last pole if the last knot has multiplicity Degree() + 1, the point at LastParameter() otherwise.</summary>
    public sealed override gp_Pnt2d EndPoint()
    {
        if (myMults.Value(myKnots.Upper()) == myDeg + 1)
        {
            return myPoles.Value(myPoles.Upper());
        }
        else
        {
            return Value(LastParameter());
        }
    }

    /// <summary>The index of the knot giving FirstParameter() (1 for a periodic curve).</summary>
    public int FirstUKnotIndex()
    {
        if (myPeriodic)
        {
            return 1;
        }
        else
        {
            return BSplCLib.FirstUKnotIndex(myDeg, myMults);
        }
    }

    /// <summary>The flat knot of index Degree() + 1.</summary>
    public sealed override double FirstParameter() => myFlatKnots.Value(myDeg + 1);

    /// <summary>The knot of index Index.</summary>
    /// <exception cref="Standard_OutOfRange">Index is outside [1, NbKnots()].</exception>
    public double Knot(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > myKnots.Length(), "Geom2d_BSplineCurve::Knot");
        return myKnots.Value(Index);
    }

    /// <summary>The knot distribution: NonUniform, Uniform, QuasiUniform or PiecewiseBezier.</summary>
    public GeomAbs_BSplKnotDistribution KnotDistribution() => myKnotSet;

    /// <summary>Copies the knots of K's index range into K.</summary>
    /// <exception cref="Standard_DomainError">K's range is outside [1, NbKnots()].</exception>
    public void Knots(NCollection_Array1<double> K)
    {
        Standard_DomainError_Raise_if(K.Lower() < myKnots.Lower() || K.Upper() > myKnots.Upper(),
                                      "Geom2d_BSplineCurve::Knots");
        for (int anIdx = K.Lower(); anIdx <= K.Upper(); anIdx++)
        {
            K[anIdx] = myKnots.Value(anIdx);
        }
    }

    /// <summary>The knots (a copy; OCCT returns a reference to its array).</summary>
    public PackedList<double> Knots() => myKnots.ToPackedList();

    /// <summary>The knot array itself, indexed from 1, for ported code: don't modify it (decision 701).</summary>
    internal NCollection_Array1<double> KnotsRef() => myKnots;

    /// <summary>Copies the flat knots of K's index range into K.</summary>
    /// <exception cref="Standard_DomainError">K's range is outside the flat knots' range.</exception>
    public void KnotSequence(NCollection_Array1<double> K)
    {
        Standard_DomainError_Raise_if(K.Lower() < myFlatKnots.Lower() || K.Upper() > myFlatKnots.Upper(),
                                      "Geom2d_BSplineCurve::KnotSequence");
        for (int anIdx = K.Lower(); anIdx <= K.Upper(); anIdx++)
        {
            K[anIdx] = myFlatKnots.Value(anIdx);
        }
    }

    /// <summary>The flat knots, each knot repeated by its multiplicity (a copy).</summary>
    public PackedList<double> KnotSequence() => myFlatKnots.ToPackedList();

    /// <summary>The flat knot array itself, for ported code: don't modify it (decision 701).</summary>
    internal NCollection_Array1<double> KnotSequenceRef() => myFlatKnots;

    /// <summary>The index of the knot giving LastParameter() (NbKnots() for a periodic curve).</summary>
    public int LastUKnotIndex()
    {
        if (myPeriodic)
        {
            return myKnots.Length();
        }
        else
        {
            return BSplCLib.LastUKnotIndex(myDeg, myMults);
        }
    }

    /// <summary>The flat knot of index NbFlatKnots - Degree().</summary>
    public sealed override double LastParameter() => myFlatKnots.Value(myFlatKnots.Upper() - myDeg);

    /// <summary>The point of parameter U, computed on the span between the knots FromK1 and ToK2.</summary>
    /// <exception cref="Standard_DomainError">FromK1 == ToK2.</exception>
    public gp_Pnt2d LocalValue(double U, int FromK1, int ToK2)
    {
        gp_Pnt2d P;
        LocalD0(U, FromK1, ToK2, out P);
        return P;
    }

    /// <summary>The point of parameter U, computed on the span between the knots FromK1 and ToK2.</summary>
    /// <exception cref="Standard_DomainError">FromK1 == ToK2.</exception>
    public void LocalD0(double U, int FromK1, int ToK2, out gp_Pnt2d P)
    {
        Standard_DomainError_Raise_if(FromK1 == ToK2, "Geom2d_BSplineCurve::LocalValue");

        double u = U;
        int index = 0;
        BSplCLib.LocateParameter(myDeg, myFlatKnots, U, myPeriodic, FromK1, ToK2, out index, out u);
        index = BSplCLib.FlatIndex(myDeg, index, myMults, myPeriodic);

        BSplCLib.D0(u,
                    index,
                    myDeg,
                    myPeriodic,
                    myPoles,
                    WeightsRef(),
                    myFlatKnots,
                    BSplCLib.NoMults(),
                    out P);
    }

    /// <summary>The point and first derivative at U, computed on the span between the knots FromK1 and ToK2.</summary>
    /// <exception cref="Standard_DomainError">FromK1 == ToK2.</exception>
    public void LocalD1(double U, int FromK1, int ToK2, out gp_Pnt2d P, out gp_Vec2d V1)
    {
        Standard_DomainError_Raise_if(FromK1 == ToK2, "Geom2d_BSplineCurve::LocalD1");

        double u = U;
        int index = 0;
        BSplCLib.LocateParameter(myDeg, myFlatKnots, U, myPeriodic, FromK1, ToK2, out index, out u);
        index = BSplCLib.FlatIndex(myDeg, index, myMults, myPeriodic);

        BSplCLib.D1(u,
                    index,
                    myDeg,
                    myPeriodic,
                    myPoles,
                    WeightsRef(),
                    myFlatKnots,
                    BSplCLib.NoMults(),
                    out P,
                    out V1);
    }

    /// <summary>The point and first two derivatives at U, computed on the span between the knots FromK1 and ToK2.</summary>
    /// <exception cref="Standard_DomainError">FromK1 == ToK2.</exception>
    public void LocalD2(double U, int FromK1, int ToK2, out gp_Pnt2d P, out gp_Vec2d V1, out gp_Vec2d V2)
    {
        Standard_DomainError_Raise_if(FromK1 == ToK2, "Geom2d_BSplineCurve::LocalD2");

        double u = U;
        int index = 0;
        BSplCLib.LocateParameter(myDeg, myFlatKnots, U, myPeriodic, FromK1, ToK2, out index, out u);
        index = BSplCLib.FlatIndex(myDeg, index, myMults, myPeriodic);

        BSplCLib.D2(u,
                    index,
                    myDeg,
                    myPeriodic,
                    myPoles,
                    WeightsRef(),
                    myFlatKnots,
                    BSplCLib.NoMults(),
                    out P,
                    out V1,
                    out V2);
    }

    /// <summary>The point and first three derivatives at U, computed on the span between the knots FromK1 and ToK2.</summary>
    /// <exception cref="Standard_DomainError">FromK1 == ToK2.</exception>
    public void LocalD3(double U,
                        int FromK1,
                        int ToK2,
                        out gp_Pnt2d P,
                        out gp_Vec2d V1,
                        out gp_Vec2d V2,
                        out gp_Vec2d V3)
    {
        Standard_DomainError_Raise_if(FromK1 == ToK2, "Geom2d_BSplineCurve::LocalD3");

        double u = U;
        int index = 0;
        BSplCLib.LocateParameter(myDeg, myFlatKnots, U, myPeriodic, FromK1, ToK2, out index, out u);
        index = BSplCLib.FlatIndex(myDeg, index, myMults, myPeriodic);

        BSplCLib.D3(u,
                    index,
                    myDeg,
                    myPeriodic,
                    myPoles,
                    WeightsRef(),
                    myFlatKnots,
                    BSplCLib.NoMults(),
                    out P,
                    out V1,
                    out V2,
                    out V3);
    }

    /// <summary>The Nth derivative at U, computed on the span between the knots FromK1 and ToK2.</summary>
    /// <exception cref="Standard_DomainError">FromK1 == ToK2.</exception>
    public gp_Vec2d LocalDN(double U, int FromK1, int ToK2, int N)
    {
        // OCCT: the message says LocalD3 (cosmetic).
        Standard_DomainError_Raise_if(FromK1 == ToK2, "Geom2d_BSplineCurve::LocalD3");

        double u = U;
        int index = 0;
        BSplCLib.LocateParameter(myDeg, myFlatKnots, U, myPeriodic, FromK1, ToK2, out index, out u);
        index = BSplCLib.FlatIndex(myDeg, index, myMults, myPeriodic);

        gp_Vec2d V;

        BSplCLib.DN(u,
                    N,
                    index,
                    myDeg,
                    myPeriodic,
                    myPoles,
                    WeightsRef(),
                    myFlatKnots,
                    BSplCLib.NoMults(),
                    out V);
        return V;
    }

    /// <summary>The multiplicity of the knot of index Index.</summary>
    /// <exception cref="Standard_OutOfRange">Index is outside [1, NbKnots()].</exception>
    public int Multiplicity(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > myMults.Length(),
                                     "Geom2d_BSplineCurve::Multiplicity");
        return myMults.Value(Index);
    }

    /// <summary>Copies the multiplicities into M.</summary>
    /// <exception cref="Standard_DimensionError">M's length is not NbKnots().</exception>
    public void Multiplicities(NCollection_Array1<int> M)
    {
        Standard_DimensionError_Raise_if(M.Length() != myMults.Length(),
                                         "Geom2d_BSplineCurve::Multiplicities");
        M.Assign(myMults);
    }

    /// <summary>The multiplicities (a copy).</summary>
    public PackedList<int> Multiplicities() => myMults.ToPackedList();

    /// <summary>The multiplicity array itself, for ported code: don't modify it (decision 701).</summary>
    internal NCollection_Array1<int> MultiplicitiesRef() => myMults;

    /// <summary>The number of knots.</summary>
    public int NbKnots() => myKnots.Length();

    /// <summary>The number of poles.</summary>
    public int NbPoles() => myPoles.Length();

    /// <summary>The pole of index Index.</summary>
    /// <exception cref="Standard_OutOfRange">Index is outside [1, NbPoles()].</exception>
    public gp_Pnt2d Pole(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > myPoles.Length(), "Geom2d_BSplineCurve::Pole");
        return myPoles.Value(Index);
    }

    /// <summary>Copies the poles into P.</summary>
    /// <exception cref="Standard_DimensionError">P's length is not NbPoles().</exception>
    public void Poles(NCollection_Array1<gp_Pnt2d> P)
    {
        Standard_DimensionError_Raise_if(P.Length() != myPoles.Length(), "Geom2d_BSplineCurve::Poles");
        P.Assign(myPoles);
    }

    /// <summary>The poles (a copy; OCCT returns a reference to its array).</summary>
    public PackedList<gp_Pnt2d> Poles() => myPoles.ToPackedList();

    /// <summary>The pole array itself, indexed from 1, for ported code: don't modify it (decision 701).</summary>
    internal NCollection_Array1<gp_Pnt2d> PolesRef() => myPoles;

    /// <summary>The first pole if the first knot has multiplicity Degree() + 1, the point at FirstParameter() otherwise.</summary>
    public sealed override gp_Pnt2d StartPoint()
    {
        if (myMults.Value(1) == myDeg + 1)
        {
            return myPoles.Value(1);
        }
        else
        {
            return Value(FirstParameter());
        }
    }

    /// <summary>The weight of the pole of index Index (1 if the curve is not rational).</summary>
    /// <exception cref="Standard_OutOfRange">Index is outside [1, NbPoles()].</exception>
    public double Weight(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > myPoles.Length(),
                                     "Geom2d_BSplineCurve::Weight");
        if (IsRational())
        {
            return myWeights.Value(Index);
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
        Standard_DimensionError_Raise_if(W.Length() != myPoles.Length(), "Geom2d_BSplineCurve::Weights");
        if (IsRational())
        {
            W.Assign(myWeights);
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

    /// <summary>The weights (a copy), or null if the curve is not rational (OCCT's <c>BSplCLib::NoWeights()</c>).</summary>
    public PackedList<double>? Weights() => IsRational() ? myWeights.ToPackedList() : null;

    /// <summary>The weight array for ported code, or null if the curve is not rational: don't modify it.</summary>
    internal NCollection_Array1<double>? WeightsRef()
    {
        if (IsRational())
        {
            return myWeights;
        }
        return BSplCLib.NoWeights();
    }

    /// <summary>The weights (a copy), ones if the curve is not rational.</summary>
    public PackedList<double> WeightsArray() => myWeights.ToPackedList();

    /// <summary>The weight array (shared unit weights if not rational) for ported code: don't modify it.</summary>
    internal NCollection_Array1<double> WeightsArrayRef() => myWeights;

    /// <summary>True if the weights are not all equal.</summary>
    public bool IsRational() => myRational;

    /// <summary>Applies T to the poles.</summary>
    protected internal sealed override void Transform(in gp_Trsf2d T)
    {
        CheckMutable();
        for (int I = 1; I <= myPoles.Length(); I++)
        {
            myPoles[I].Transform(T);
        }
        myMaxDerivInvOk = false;
        // OCCT: the evaluation representation is kept (Geom_BSplineCurve clears it), so the transformed curve still
        // evaluates through the untransformed representation (occt-issues.md). The same holds for every mutator here.
    }

    /// <summary>
    /// Locates U among the knots (the flat knots if WithKnotRepetition): I1 = I2 = the index of a knot within
    /// ParametricTolerance of U, or the indexes of the knots around U (0 and 1, or NbKnots and NbKnots + 1,
    /// outside the range).
    /// </summary>
    public void LocateU(double U,
                        double ParametricTolerance,
                        out int I1,
                        out int I2,
                        bool WithKnotRepetition = false)
    {
        double NewU = U;
        NCollection_Array1<double> CKnots = WithKnotRepetition ? myFlatKnots : myKnots;

        PeriodicNormalization(ref NewU); // Attention a la periode

        double UFirst = CKnots[1];
        double ULast = CKnots[CKnots.Length()];
        double PParametricTolerance = Math.Abs(ParametricTolerance);
        if (Math.Abs(NewU - UFirst) <= PParametricTolerance)
        {
            I1 = I2 = 1;
        }
        else if (Math.Abs(NewU - ULast) <= PParametricTolerance)
        {
            I1 = I2 = CKnots.Length();
        }
        else if (NewU < UFirst)
        {
            I2 = 1;
            I1 = 0;
        }
        else if (NewU > ULast)
        {
            I1 = CKnots.Length();
            I2 = I1 + 1;
        }
        else
        {
            I1 = 1;
            BSplCLib.Hunt(CKnots, NewU, out I1);
            I1 = Max(Min(I1, CKnots.Upper()), CKnots.Lower());
            while (I1 + 1 <= CKnots.Upper() && Math.Abs(CKnots[I1 + 1] - NewU) <= PParametricTolerance)
            {
                I1++;
            }
            if (Math.Abs(CKnots[I1] - NewU) <= PParametricTolerance)
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
    /// The parametric tolerance UTolerance such that |t1 - t0| &lt; UTolerance implies
    /// |C(t1) - C(t0)| &lt; ToleranceUV.
    /// </summary>
    public void Resolution(double ToleranceUV, out double UTolerance)
    {
        // OCCT.NET: a cache filled on a possibly shared curve; the flag is published after the value.
        if (!Volatile.Read(ref myMaxDerivInvOk))
        {
            double aMaxDerivInv;
            if (myPeriodic)
            {
                int NbKnots, NbPoles;
                BSplCLib.PrepareUnperiodize(myDeg, myMults, out NbKnots, out NbPoles);
                NCollection_Array1<gp_Pnt2d> new_poles = new NCollection_Array1<gp_Pnt2d>(1, NbPoles);
                NCollection_Array1<double> new_weights = new NCollection_Array1<double>(1, NbPoles);
                for (int ii = 1; ii <= NbPoles; ii++)
                {
                    new_poles[ii] = myPoles[((ii - 1) % myPoles.Length()) + 1];
                }
                if (myRational)
                {
                    for (int ii = 1; ii <= NbPoles; ii++)
                    {
                        new_weights[ii] = myWeights[((ii - 1) % myPoles.Length()) + 1];
                    }
                }
                BSplCLib.Resolution(new_poles,
                                    myRational ? new_weights : BSplCLib.NoWeights(),
                                    new_poles.Length(),
                                    myFlatKnots,
                                    myDeg,
                                    1.0,
                                    out aMaxDerivInv);
            }
            else
            {
                BSplCLib.Resolution(myPoles,
                                    WeightsRef(),
                                    myPoles.Length(),
                                    myFlatKnots,
                                    myDeg,
                                    1.0,
                                    out aMaxDerivInv);
            }
            myMaxDerivInv = aMaxDerivInv;
            Volatile.Write(ref myMaxDerivInvOk, true);
        }
        UTolerance = ToleranceUV * myMaxDerivInv;
    }

    // Non-mutating twins of the mutators (OCCT.NET)

    /// <summary>A copy with its degree raised to Degree.</summary>
    public Geom2d_BSplineCurve DegreeIncreased(int Degree)
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        aCopy.IncreaseDegree(Degree);
        return aCopy;
    }

    /// <summary>A copy with the multiplicity of the knot of index Index raised to M.</summary>
    public Geom2d_BSplineCurve MultiplicityIncreased(int Index, int M)
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        aCopy.IncreaseMultiplicity(Index, M);
        return aCopy;
    }

    /// <summary>A copy with the multiplicities of the knots of indexes I1 to I2 raised to M.</summary>
    public Geom2d_BSplineCurve MultiplicityIncreased(int I1, int I2, int M)
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        aCopy.IncreaseMultiplicity(I1, I2, M);
        return aCopy;
    }

    /// <summary>A copy with the multiplicities of the knots of indexes I1 to I2 incremented by Step.</summary>
    public Geom2d_BSplineCurve MultiplicityIncremented(int I1, int I2, int Step)
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        aCopy.IncrementMultiplicity(I1, I2, Step);
        return aCopy;
    }

    /// <summary>A copy with the knot U inserted (see InsertKnot).</summary>
    public Geom2d_BSplineCurve KnotInserted(double U, int M = 1, double ParametricTolerance = 0.0)
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        aCopy.InsertKnot(U, M, ParametricTolerance);
        return aCopy;
    }

    /// <summary>A copy with the knots Knots inserted (see InsertKnots; Add defaults to false, as in OCCT).</summary>
    public Geom2d_BSplineCurve KnotsInserted(NCollection_Array1<double> Knots,
                                             NCollection_Array1<int> Mults,
                                             double ParametricTolerance = 0.0,
                                             bool Add = false)
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        aCopy.InsertKnots(Knots, Mults, ParametricTolerance, Add);
        return aCopy;
    }

    /// <summary>A copy with the multiplicity of the knot of index Index lowered to M, or null if that changes the curve by more than Tolerance.</summary>
    public Geom2d_BSplineCurve? KnotRemoved(int Index, int M, double Tolerance)
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        return aCopy.RemoveKnot(Index, M, Tolerance) ? aCopy : null;
    }

    /// <summary>A copy with the pole P (weight Weight) inserted after the pole of index Index.</summary>
    public Geom2d_BSplineCurve PoleInsertedAfter(int Index, in gp_Pnt2d P, double Weight = 1.0)
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        aCopy.InsertPoleAfter(Index, P, Weight);
        return aCopy;
    }

    /// <summary>A copy with the pole P (weight Weight) inserted before the pole of index Index.</summary>
    public Geom2d_BSplineCurve PoleInsertedBefore(int Index, in gp_Pnt2d P, double Weight = 1.0)
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        aCopy.InsertPoleBefore(Index, P, Weight);
        return aCopy;
    }

    /// <summary>A copy without the pole of index Index.</summary>
    public Geom2d_BSplineCurve PoleRemoved(int Index)
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        aCopy.RemovePole(Index);
        return aCopy;
    }

    /// <summary>A copy limited to [U1, U2] (see Segment).</summary>
    public Geom2d_BSplineCurve Segmented(double U1, double U2, double theTolerance = 1.0e-9)
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        aCopy.Segment(U1, U2, theTolerance);
        return aCopy;
    }

    /// <summary>A copy with the knot of index Index changed to K.</summary>
    public Geom2d_BSplineCurve WithKnot(int Index, double K)
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        aCopy.SetKnot(Index, K);
        return aCopy;
    }

    /// <summary>A copy with the knots K.</summary>
    public Geom2d_BSplineCurve WithKnots(NCollection_Array1<double> K)
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        aCopy.SetKnots(K);
        return aCopy;
    }

    /// <summary>A copy with the knot of index Index raised to multiplicity M and changed to K.</summary>
    public Geom2d_BSplineCurve WithKnot(int Index, double K, int M)
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        aCopy.SetKnot(Index, K, M);
        return aCopy;
    }

    /// <summary>A periodic copy (see SetPeriodic).</summary>
    public Geom2d_BSplineCurve WithPeriodic()
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        aCopy.SetPeriodic();
        return aCopy;
    }

    /// <summary>A copy of this periodic curve with its origin at the knot of index Index.</summary>
    public Geom2d_BSplineCurve WithOrigin(int Index)
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        aCopy.SetOrigin(Index);
        return aCopy;
    }

    /// <summary>A non-periodic copy (see SetNotPeriodic).</summary>
    public Geom2d_BSplineCurve WithNotPeriodic()
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        aCopy.SetNotPeriodic();
        return aCopy;
    }

    /// <summary>A copy with the pole of index Index replaced by P.</summary>
    public Geom2d_BSplineCurve WithPole(int Index, in gp_Pnt2d P)
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        aCopy.SetPole(Index, P);
        return aCopy;
    }

    /// <summary>A copy with the pole of index Index replaced by P, of weight W.</summary>
    public Geom2d_BSplineCurve WithPole(int Index, in gp_Pnt2d P, double W)
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        aCopy.SetPole(Index, P, W);
        return aCopy;
    }

    /// <summary>A copy with the weight of the pole of index Index changed to W.</summary>
    public Geom2d_BSplineCurve WithWeight(int Index, double W)
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        aCopy.SetWeight(Index, W);
        return aCopy;
    }

    /// <summary>A copy whose point of parameter U is moved to P (see MovePoint).</summary>
    public Geom2d_BSplineCurve PointMoved(double U,
                                          in gp_Pnt2d P,
                                          int Index1,
                                          int Index2,
                                          out int FirstModifiedPole,
                                          out int LastModifiedPole)
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        aCopy.MovePoint(U, P, Index1, Index2, out FirstModifiedPole, out LastModifiedPole);
        return aCopy;
    }

    /// <summary>A copy whose point and tangent at U are moved (see MovePointAndTangent).</summary>
    public Geom2d_BSplineCurve PointAndTangentMoved(double U,
                                                    in gp_Pnt2d P,
                                                    in gp_Vec2d Tangent,
                                                    double Tolerance,
                                                    int StartingCondition,
                                                    int EndingCondition,
                                                    out int ErrorStatus)
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        aCopy.MovePointAndTangent(U, P, Tangent, Tolerance, StartingCondition, EndingCondition, out ErrorStatus);
        return aCopy;
    }

    /// <summary>A copy evaluated through theDesc, which is copied (so later changes to theDesc don't affect it).</summary>
    /// <exception cref="Standard_ProgramError">The descriptor is invalid.</exception>
    public Geom2d_BSplineCurve WithEvalRepresentation(Geom2dEval_RepCurveDesc.Base? theDesc)
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        Geom2dEval_RepUtils.ValidateCurveDesc(theDesc, this);
        aCopy.SetEvalRepresentation(Geom2dEval_RepUtils.CloneCurveDesc(theDesc));
        return aCopy;
    }

    /// <summary>A copy without an evaluation representation.</summary>
    public Geom2d_BSplineCurve WithoutEvalRepresentation()
    {
        Geom2d_BSplineCurve aCopy = new Geom2d_BSplineCurve(this);
        aCopy.ClearEvalRepresentation();
        return aCopy;
    }
}
