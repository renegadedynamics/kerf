// Port of: src/ModelingData/TKG3d/Geom/Geom_BSplineCurve.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_BSplineCurve.cxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_BSplineCurve_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Geometry;
using static Kerf.Engine.Geometry.BSplKnotDistribution;
using static Kerf.Geometry.Continuity;

namespace Occt;

/// <summary>
/// A B-spline curve: uniform or non-uniform, rational or non-rational, periodic or non-periodic. It is defined by
/// its degree (1 to <see cref="MaxDegree"/>, 25), its poles (with weights when rational) and its knots with their
/// multiplicities, all indexed from 1. The knots are strictly increasing; the continuity at an inner knot is
/// Degree - Mult. A non-periodic curve has NbPoles = Sum(Mults) - Degree - 1 poles, a periodic one
/// Sum(Mults) - Mult(last) poles (the first and last multiplicities being equal).
/// </summary>
/// <remarks>
/// <para>
/// Arrays: <see cref="Poles()"/>, <see cref="Weights()"/>, <see cref="WeightsArray"/>, <see cref="Knots()"/>,
/// <see cref="Multiplicities()"/> and <see cref="KnotSequence()"/> return immutable copies (OCCT returns
/// references to the curve's arrays); their index 0 is OCCT's index 1. Code inside OCCT.NET that passes the
/// arrays on to BSplCLib and the like uses the internal <c>Internal*</c> accessors, which return the stored
/// arrays (shared: don't modify).
/// </para>
/// <para>
/// The evaluation representation (<see cref="HasEvalRepresentation"/>) is set by <see cref="WithEvalRepresentation"/>,
/// which stores a copy of the descriptor. Modifications of the curve drop it, as in OCCT.
/// </para>
/// </remarks>
public class Geom_BSplineCurve : Geom_BoundedCurve
{
    private NCollection_Array1<Point3D> myPoles = new NCollection_Array1<Point3D>();
    private NCollection_Array1<double> myWeights = new NCollection_Array1<double>();
    private NCollection_Array1<double> myKnots = new NCollection_Array1<double>();
    private NCollection_Array1<double> myFlatKnots = new NCollection_Array1<double>();
    private NCollection_Array1<int> myMults = new NCollection_Array1<int>();
    private GeomEval_RepCurveDesc.Base? myEvalRep;
    private int myDeg = 0;
    private bool myPeriodic = false;
    private bool myRational = false;
    private BSplKnotDistribution myKnotSet = NonUniform;
    private Continuity mySmooth = C0;
    private double myMaxDerivInv = 0.0;
    private bool myMaxDerivInvOk = false;

    //=============================================================================================
    // Geom_BSplineCurve.cxx
    //=============================================================================================

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

    private static void CheckCurveData(NCollection_Array1<Point3D> CPoles,
                                       NCollection_Array1<double> CKnots,
                                       NCollection_Array1<int> CMults,
                                       int Degree,
                                       bool Periodic)
    {
        if (Degree < 1 || Degree > Geom_BSplineCurve.MaxDegree())
        {
            throw new ConstructionErrorException("BSpline curve: invalid degree");
        }

        if (CPoles.Length() < 2)
        {
            throw new ConstructionErrorException("BSpline curve: at least 2 poles required");
        }

        if (CKnots.Length() != CMults.Length())
        {
            throw new ConstructionErrorException("BSpline curve: Knot and Mult array size mismatch");
        }

        for (int I = CKnots.Lower(); I < CKnots.Upper(); I++)
        {
            if (CKnots[I + 1] - CKnots[I] <= Epsilon(Math.Abs(CKnots[I])))
            {
                throw new ConstructionErrorException("BSpline curve: Knots interval values too close");
            }
        }

        if (CPoles.Length() != global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(Degree, Periodic, CMults))
        {
            throw new ConstructionErrorException("BSpline curve: # Poles and degree mismatch");
        }
    }

    /// <summary>Check rationality of an array of weights.</summary>
    private static bool Rational(NCollection_Array1<double> theWeights)
    {
        for (int i = theWeights.Lower(); i < theWeights.Upper(); i++)
        {
            if (Math.Abs(theWeights[i] - theWeights[i + 1]) > Constants.Resolution())
            {
                return true;
            }
        }
        return false;
    }

    /// <summary>A new, unfrozen curve equal to this one.</summary>
    public sealed override Geom_Geometry Copy() => new Geom_BSplineCurve(this);

    /// <summary>A copy of theOther, without validation (OCCT's copy constructor).</summary>
    public Geom_BSplineCurve(Geom_BSplineCurve theOther)
    {
        myPoles = new NCollection_Array1<Point3D>(theOther.myPoles);
        myWeights = theOther.myRational ? new NCollection_Array1<double>(theOther.myWeights)
                                        : global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(theOther.myPoles.Length());
        myKnots = new NCollection_Array1<double>(theOther.myKnots);
        myFlatKnots = new NCollection_Array1<double>(theOther.myFlatKnots);
        myMults = new NCollection_Array1<int>(theOther.myMults);
        myEvalRep = GeomEval_RepUtils.CloneCurveDesc(theOther.myEvalRep);
        myDeg = theOther.myDeg;
        myPeriodic = theOther.myPeriodic;
        myRational = theOther.myRational;
        myKnotSet = theOther.myKnotSet;
        mySmooth = theOther.mySmooth;
        myMaxDerivInv = theOther.myMaxDerivInv;
        myMaxDerivInvOk = false;
    }

    /// <summary>A non-rational B-spline curve.</summary>
    /// <exception cref="ConstructionErrorException">
    /// Degree not in [1, MaxDegree()], fewer than 2 poles, knots and multiplicities of different lengths, knots
    /// not increasing (by more than Epsilon), or a number of poles that doesn't match the multiplicities.
    /// </exception>
    public Geom_BSplineCurve(NCollection_Array1<Point3D> Poles,
                             NCollection_Array1<double> Knots,
                             NCollection_Array1<int> Mults,
                             int Degree,
                             bool Periodic = false)
    {
        myRational = false;
        myMaxDerivInv = 0.0;
        myMaxDerivInvOk = false;
        myDeg = Degree;
        myPeriodic = Periodic;

        // check
        CheckCurveData(Poles, Knots, Mults, Degree, Periodic);

        // copy arrays
        myPoles.Resize(1, Poles.Length(), false);
        myPoles.CopyValues(Poles);
        myWeights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(Poles.Length());

        myKnots.Resize(1, Knots.Length(), false);
        myKnots.CopyValues(Knots);

        myMults.Resize(1, Mults.Length(), false);
        myMults.CopyValues(Mults);

        updateKnots();
    }

    /// <summary>
    /// A rational B-spline curve; with CheckRational, it is non-rational if all the weights are equal.
    /// </summary>
    /// <exception cref="ConstructionErrorException">
    /// As the non-rational constructor, or the weights are not as many as the poles, or a weight is at most
    /// gp::Resolution().
    /// </exception>
    public Geom_BSplineCurve(NCollection_Array1<Point3D> Poles,
                             NCollection_Array1<double> Weights,
                             NCollection_Array1<double> Knots,
                             NCollection_Array1<int> Mults,
                             int Degree,
                             bool Periodic = false,
                             bool CheckRational = true)
    {
        myRational = true;
        myMaxDerivInv = 0.0;
        myMaxDerivInvOk = false;
        myDeg = Degree;
        myPeriodic = Periodic;

        // check
        CheckCurveData(Poles, Knots, Mults, Degree, Periodic);

        if (Weights.Length() != Poles.Length())
        {
            throw new ConstructionErrorException("Geom_BSplineCurve: Weights and Poles array size mismatch");
        }

        int i;
        for (i = Weights.Lower(); i <= Weights.Upper(); i++)
        {
            if (Weights[i] <= Constants.Resolution())
            {
                throw new ConstructionErrorException("Geom_BSplineCurve: Weights values too small");
            }
        }

        // check really rational
        if (CheckRational)
        {
            myRational = Rational(Weights);
        }

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
            myWeights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(Poles.Length());
        }

        myKnots.Resize(1, Knots.Length(), false);
        myKnots.CopyValues(Knots);

        myMults.Resize(1, Mults.Length(), false);
        myMults.CopyValues(Mults);

        updateKnots();
    }

    /// <summary>The maximum degree of a B-spline curve: 25.</summary>
    public static int MaxDegree() => global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree();

    /// <summary>Raises the degree to Degree, keeping the curve (knot multiplicities grow by the same step).</summary>
    /// <exception cref="ConstructionErrorException">Degree is less than the degree or more than MaxDegree().</exception>
    internal void IncreaseDegree(int Degree)
    {
        CheckMutable();
        if (Degree == myDeg)
        {
            return;
        }

        ClearEvalRepresentation();

        if (Degree < myDeg || Degree > Geom_BSplineCurve.MaxDegree())
        {
            throw new ConstructionErrorException("BSpline curve: IncreaseDegree: bad degree value");
        }

        int FromK1 = FirstUKnotIndex();
        int ToK2 = LastUKnotIndex();

        int Step = Degree - myDeg;

        NCollection_Array1<Point3D> npoles = new NCollection_Array1<Point3D>(1, myPoles.Length() + Step * (ToK2 - FromK1));

        int nbknots = global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegreeCountKnots(myDeg, Degree, myPeriodic, myMults);

        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, nbknots);

        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, nbknots);

        NCollection_Array1<double> nweights = new NCollection_Array1<double>();

        if (IsRational())
        {
            nweights.Resize(1, npoles.Upper(), false);
        }

        global::Kerf.Engine.Geometry.Splines.BSplCLib.IncreaseDegree(myDeg,
                                Degree,
                                myPeriodic,
                                myPoles,
                                InternalWeights(),
                                myKnots,
                                myMults,
                                npoles,
                                myRational ? nweights : global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
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
            myWeights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(myPoles.Length());
        }
        myKnots = nknots;
        myMults = nmults;
        updateKnots();
    }

    /// <summary>Raises the multiplicity of the knot Index to M (unchanged if already at least M).</summary>
    internal void IncreaseMultiplicity(int Index, int M)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(1, 1);
        k[1] = myKnots.Value(Index);
        NCollection_Array1<int> m = new NCollection_Array1<int>(1, 1);
        m[1] = M - myMults.Value(Index);
        InsertKnots(k, m, Epsilon(1.0), true);
    }

    /// <summary>Raises the multiplicities of the knots I1 to I2 to M.</summary>
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

    /// <summary>Raises the multiplicities of the knots I1 to I2 by Step.</summary>
    internal void IncrementMultiplicity(int I1, int I2, int Step)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(myKnots, I1, I1, I2);
        NCollection_Array1<int> m = new NCollection_Array1<int>(I1, I2);
        m.Init(Step);
        InsertKnots(k, m, Epsilon(1.0), true);
    }

    /// <summary>
    /// Inserts the knot U with multiplicity M. A knot within ParametricTolerance of an existing one raises its
    /// multiplicity: by M if Add, to M otherwise.
    /// </summary>
    internal void InsertKnot(double U, int M = 1, double ParametricTolerance = 0.0, bool Add = true)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(1, 1);
        k[1] = U;
        NCollection_Array1<int> m = new NCollection_Array1<int>(1, 1);
        m[1] = M;
        InsertKnots(k, m, ParametricTolerance, Add);
    }

    /// <summary>
    /// Inserts the knots with their multiplicities (see <see cref="InsertKnot"/>). Note that Add defaults to false
    /// here, and to true in InsertKnot and BSplCLib.InsertKnots.
    /// </summary>
    /// <exception cref="ConstructionErrorException">The knots can't be inserted (BSplCLib::PrepareInsertKnots).</exception>
    internal void InsertKnots(NCollection_Array1<double> Knots,
                              NCollection_Array1<int> Mults,
                              double ParametricTolerance = 0.0,
                              bool Add = false)
    {
        CheckMutable();
        // Check and compute new sizes
        int nbpoles, nbknots;

        if (!global::Kerf.Engine.Geometry.Splines.BSplCLib.PrepareInsertKnots(myDeg,
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
            throw new ConstructionErrorException("Geom_BSplineCurve::InsertKnots");
        }

        if (nbpoles == myPoles.Length())
        {
            return;
        }

        ClearEvalRepresentation();

        NCollection_Array1<Point3D> npoles = new NCollection_Array1<Point3D>(1, nbpoles);
        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, nbknots);
        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, nbknots);
        NCollection_Array1<double> nweights = new NCollection_Array1<double>();

        if (myRational)
        {
            nweights.Resize(1, nbpoles, false);
        }

        global::Kerf.Engine.Geometry.Splines.BSplCLib.InsertKnots(myDeg,
                             myPeriodic,
                             myPoles,
                             InternalWeights(),
                             myKnots,
                             myMults,
                             Knots,
                             Mults,
                             npoles,
                             myRational ? nweights : global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
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
            myWeights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(npoles.Length());
        }

        myPoles = npoles;
        myKnots = nknots;
        myMults = nmults;
        updateKnots();
    }

    /// <summary>
    /// Lowers the multiplicity of the knot Index to M (0 removes it) if the curve stays within Tolerance;
    /// returns false, leaving the curve unchanged, otherwise.
    /// </summary>
    /// <exception cref="OutOfRangeException">Index is not an inner knot (non-periodic) or not in the knot range (periodic).</exception>
    internal bool RemoveKnot(int Index, int M, double Tolerance)
    {
        CheckMutable();
        if (M < 0)
        {
            return true;
        }

        int I1 = FirstUKnotIndex();
        int I2 = LastUKnotIndex();

        if (!myPeriodic && (Index <= I1 || Index >= I2))
        {
            throw new OutOfRangeException("BSpline curve: RemoveKnot: index out of range");
        }
        else if (myPeriodic && (Index < I1 || Index > I2))
        {
            throw new OutOfRangeException("BSpline curve: RemoveKnot: index out of range");
        }

        NCollection_Array1<Point3D> oldpoles = myPoles;

        int step = myMults.Value(Index) - M;
        if (step <= 0)
        {
            return true;
        }

        NCollection_Array1<Point3D> npoles = new NCollection_Array1<Point3D>(1, oldpoles.Length() - step);

        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, myKnots.Length() - (M == 0 ? 1 : 0));
        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, myMults.Length() - (M == 0 ? 1 : 0));

        NCollection_Array1<double> nweights = new NCollection_Array1<double>();

        if (IsRational())
        {
            nweights.Resize(1, npoles.Length(), false);
        }

        if (!global::Kerf.Engine.Geometry.Splines.BSplCLib.RemoveKnot(Index,
                                 M,
                                 myDeg,
                                 myPeriodic,
                                 myPoles,
                                 InternalWeights(),
                                 myKnots,
                                 myMults,
                                 npoles,
                                 myRational ? nweights : global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                                 nknots,
                                 nmults,
                                 Tolerance))
        {
            return false;
        }

        ClearEvalRepresentation();

        if (IsRational())
        {
            myWeights = nweights;
        }
        else
        {
            myWeights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(npoles.Length());
        }

        myPoles = npoles;
        myKnots = nknots;
        myMults = nmults;

        updateKnots();
        myMaxDerivInvOk = false;
        return true;
    }

    /// <summary>
    /// Reverses the parametrization; the knots K1, ..., Kn become -Kn, ..., -K1 shifted to keep the bounds.
    /// </summary>
    protected internal sealed override void Reverse()
    {
        CheckMutable();
        ClearEvalRepresentation();

        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reverse(myKnots);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reverse(myMults);
        int last;
        if (myPeriodic)
        {
            last = myFlatKnots.Upper() - myDeg - 1;
        }
        else
        {
            last = myPoles.Upper();
        }
        global::Kerf.Engine.Geometry.Splines.BSplCLib.Reverse(myPoles, last);
        if (myRational)
        {
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reverse(myWeights, last);
        }
        updateKnots();
    }

    /// <summary>FirstParameter() + LastParameter() - U.</summary>
    public sealed override double ReversedParameter(double U) => (FirstParameter() + LastParameter() - U);

    /// <summary>
    /// Keeps the part between U1 and U2 (U1 &lt;= U2); a periodic curve becomes non-periodic. theTolerance is the
    /// minimum distance at which a new knot is inserted.
    /// </summary>
    /// <exception cref="DomainErrorException">U2 &lt; U1, or U2 - U1 exceeds the period of a periodic curve.</exception>
    internal void Segment(double U1, double U2, double theTolerance = Precision_PConfusion)
    {
        CheckMutable();
        if (U2 < U1)
        {
            throw new DomainErrorException("Geom_BSplineCurve::Segment");
        }

        ClearEvalRepresentation();

        double NewU1, NewU2;
        double U, DU = 0, aDDU = 0;
        int index;
        bool wasPeriodic = myPeriodic;

        NCollection_Array1<double> Knots = new NCollection_Array1<double>(1, 2);
        NCollection_Array1<int> Mults = new NCollection_Array1<int>(1, 2);

        // define param distance to keep (eap, Apr 18 2002, occ311)
        if (myPeriodic)
        {
            double Period = LastParameter() - FirstParameter();
            DU = U2 - U1;
            if (DU - Period > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                throw new DomainErrorException("Geom_BSplineCurve::Segment");
            }
            if (DU > Period)
            {
                DU = Period;
            }
            aDDU = DU;
        }

        index = 0;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(myDeg,
                                 myKnots,
                                 myMults,
                                 U1,
                                 myPeriodic,
                                 myKnots.Lower(),
                                 myKnots.Upper(),
                                 out index,
                                 out NewU1);
        index = 0;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(myDeg,
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

        Knots[1] = Min(NewU1, NewU2);
        Knots[2] = Max(NewU1, NewU2);
        Mults[1] = Mults[2] = myDeg;

        double AbsUMax = Max(Math.Abs(NewU1), Math.Abs(NewU2));

        //  Modified by Sergey KHROMOV - Fri Apr 11 12:15:40 2003 Begin
        AbsUMax = Max(AbsUMax, Max(Math.Abs(FirstParameter()), Math.Abs(LastParameter())));
        //  Modified by Sergey KHROMOV - Fri Apr 11 12:15:40 2003 End

        double Eps = Max(Epsilon(AbsUMax), theTolerance);

        InsertKnots(Knots, Mults, Eps);

        if (myPeriodic)
        { // set the origine at NewU1
            index = 0;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(myDeg,
                                     myKnots,
                                     myMults,
                                     U1,
                                     myPeriodic,
                                     myKnots.Lower(),
                                     myKnots.Upper(),
                                     out index,
                                     out U);
            // Test if the insertion is OK, shift otherwise.
            if (Math.Abs(myKnots.Value(index + 1) - U) <= Eps)
            { // <= to be consistent with InsertKnots
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
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(myDeg, myKnots, myMults, NewU1, myPeriodic, FromU1, ToU2, out index1, out U);
        if (Math.Abs(myKnots.Value(index1 + 1) - U) <= Eps)
        {
            index1++;
        }

        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(myDeg, myKnots, myMults, NewU2, myPeriodic, FromU1, ToU2, out index2, out U);
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

        int i, k = 1;
        for (i = index1; i <= index2; i++)
        {
            nknots.SetValue(k, myKnots.Value(i) - DU);
            nmults.SetValue(k, myMults.Value(i));
            k++;
        }
        nmults.SetValue(1, myDeg + 1);
        nmults.SetValue(nbknots, myDeg + 1);

        // compute index1 and index2 to set the new poles and weights
        int pindex1 = global::Kerf.Engine.Geometry.Splines.BSplCLib.PoleIndex(myDeg, index1, myPeriodic, myMults);
        int pindex2 = global::Kerf.Engine.Geometry.Splines.BSplCLib.PoleIndex(myDeg, index2, myPeriodic, myMults);

        pindex1++;
        pindex2 = Min(pindex2 + 1, myPoles.Length());

        int nbpoles = pindex2 - pindex1 + 1;

        NCollection_Array1<double> nweights = new NCollection_Array1<double>();
        NCollection_Array1<Point3D> npoles = new NCollection_Array1<Point3D>(1, nbpoles);

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
            myWeights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(myPoles.Length());
        }

        myMaxDerivInvOk = false;
        updateKnots();
    }

    // Precision::PConfusion() as a constant, for the default argument of Segment.
    private const double Precision_PConfusion = 1.0e-7 * 0.01;

    /// <summary>Changes the value of the knot Index (it must stay strictly between its neighbours).</summary>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbKnots].</exception>
    /// <exception cref="ConstructionErrorException">K is not between the neighbouring knots.</exception>
    internal void SetKnot(int Index, double K)
    {
        CheckMutable();
        if (Index < 1 || Index > myKnots.Length())
        {
            throw new OutOfRangeException("BSpline curve: SetKnot: Index and #knots mismatch");
        }
        double DK = Math.Abs(Epsilon(K));
        if (Index == 1)
        {
            if (K >= myKnots.Value(2) - DK)
            {
                throw new ConstructionErrorException("BSpline curve: SetKnot: K out of range");
            }
        }
        else if (Index == myKnots.Length())
        {
            if (K <= myKnots.Value(myKnots.Length() - 1) + DK)
            {
                throw new ConstructionErrorException("BSpline curve: SetKnot: K out of range");
            }
        }
        else
        {
            if (K <= myKnots.Value(Index - 1) + DK || K >= myKnots.Value(Index + 1) - DK)
            {
                throw new ConstructionErrorException("BSpline curve: SetKnot: K out of range");
            }
        }
        if (K != myKnots.Value(Index))
        {
            ClearEvalRepresentation();
            myKnots.SetValue(Index, K);
            myMaxDerivInvOk = false;
            updateKnots();
        }
    }

    /// <summary>Changes all the knots.</summary>
    /// <exception cref="ConstructionErrorException">The knots are invalid for the curve.</exception>
    internal void SetKnots(NCollection_Array1<double> K)
    {
        CheckMutable();
        CheckCurveData(myPoles, K, myMults, myDeg, myPeriodic);
        ClearEvalRepresentation();
        myKnots.CopyValues(K);
        myMaxDerivInvOk = false;
        updateKnots();
    }

    /// <summary>Raises the multiplicity of the knot Index to M, then changes its value.</summary>
    internal void SetKnot(int Index, double K, int M)
    {
        CheckMutable();
        IncreaseMultiplicity(Index, M);
        SetKnot(Index, K);
    }

    /// <summary>
    /// Makes the curve periodic on its parameter range: the knots outside [FirstUKnotIndex, LastUKnotIndex] are
    /// dropped, the end multiplicities set to Min(Degree, Max(first, last)) and the poles truncated.
    /// </summary>
    internal void SetPeriodic()
    {
        CheckMutable();
        ClearEvalRepresentation();

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
        int nbp = global::Kerf.Engine.Geometry.Splines.BSplCLib.NbPoles(myDeg, true, myMults);

        myPoles.Resize(1, nbp, true);
        if (myRational)
        {
            myWeights.Resize(1, nbp, true);
        }
        else
        {
            myWeights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(nbp);
        }

        myPeriodic = true;

        myMaxDerivInvOk = false;
        updateKnots();
    }

    /// <summary>Makes the knot Index the origin of a periodic curve.</summary>
    /// <exception cref="NoSuchObjectException">The curve is not periodic.</exception>
    /// <exception cref="DomainErrorException">Index is not in [FirstUKnotIndex, LastUKnotIndex].</exception>
    internal void SetOrigin(int Index)
    {
        CheckMutable();
        if (!myPeriodic)
        {
            throw new NoSuchObjectException("Geom_BSplineCurve::SetOrigin");
        }

        ClearEvalRepresentation();

        int i, k;
        int first = FirstUKnotIndex();
        int last = LastUKnotIndex();

        if ((Index < first) || (Index > last))
        {
            throw new DomainErrorException("Geom_BSplineCurve::SetOrigin");
        }

        int nbknots = myKnots.Length();
        int nbpoles = myPoles.Length();

        NCollection_Array1<double> newknots = new NCollection_Array1<double>(1, nbknots);

        NCollection_Array1<int> newmults = new NCollection_Array1<int>(1, nbknots);

        // set the knots and mults
        double period = myKnots.Value(last) - myKnots.Value(first);
        k = 1;
        for (i = Index; i <= last; i++)
        {
            newknots[k] = myKnots.Value(i);
            newmults[k] = myMults.Value(i);
            k++;
        }
        for (i = first + 1; i <= Index; i++)
        {
            newknots[k] = myKnots.Value(i) + period;
            newmults[k] = myMults.Value(i);
            k++;
        }

        int index = 1;
        for (i = first + 1; i <= Index; i++)
        {
            index += myMults.Value(i);
        }

        // set the poles and weights
        NCollection_Array1<Point3D> newpoles = new NCollection_Array1<Point3D>(1, nbpoles);
        first = myPoles.Lower();
        last = myPoles.Upper();
        if (myRational)
        {
            NCollection_Array1<double> newweights = new NCollection_Array1<double>(1, nbpoles);
            k = 1;
            for (i = index; i <= last; i++)
            {
                newpoles[k] = myPoles.Value(i);
                newweights[k] = myWeights.Value(i);
                k++;
            }
            for (i = first; i < index; i++)
            {
                newpoles[k] = myPoles.Value(i);
                newweights[k] = myWeights.Value(i);
                k++;
            }
            myWeights = newweights;
        }
        else
        {
            k = 1;
            for (i = index; i <= last; i++)
            {
                newpoles[k] = myPoles.Value(i);
                k++;
            }
            for (i = first; i < index; i++)
            {
                newpoles[k] = myPoles.Value(i);
                k++;
            }
            myWeights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(nbpoles);
        }

        myPoles = newpoles;
        myKnots = newknots;
        myMults = newmults;
        myMaxDerivInvOk = false;
        updateKnots();
    }

    /// <summary>
    /// Makes the parameter U the origin of a periodic curve: the knots are shifted by whole periods so that U is
    /// in range, a knot is inserted at U unless one is within Tol, and that knot becomes the origin.
    /// </summary>
    /// <exception cref="NoSuchObjectException">The curve is not periodic.</exception>
    internal void SetOrigin(double U, double Tol)
    {
        CheckMutable();
        if (!myPeriodic)
        {
            throw new NoSuchObjectException("Geom_BSplineCurve::SetOrigin");
        }
        // Is U within the period?
        double uf = FirstParameter(), ul = LastParameter();
        double u = U, period = ul - uf;
        while (Tol < (uf - u))
        {
            u += period;
        }
        while (Tol > (ul - u))
        {
            u -= period;
        }
        if (Math.Abs(U - u) > Tol)
        { // Reparametrize the curve
            ClearEvalRepresentation();
            double delta = U - u;
            uf += delta;
            ul += delta;
            int fk = myKnots.Lower(), lk = myKnots.Upper();
            for (int i = fk; i <= lk; i++)
            {
                myKnots.ChangeValue(i) += delta;
            }
            updateKnots();
        }
        {
            // For periodic curve, uf and ul represent the same point
            if (Math.Abs(U - uf) < Tol || Math.Abs(U - ul) < Tol)
            {
                return;
            }

            int fk = myKnots.Lower(), lk = myKnots.Upper(), ik = 0;
            double delta = RealLast();
            for (int i = fk; i <= lk; i++)
            {
                double dki = myKnots.Value(i) - U;
                if (Math.Abs(dki) < Math.Abs(delta))
                {
                    ik = i;
                    delta = dki;
                }
            }
            if (Math.Abs(delta) > Tol)
            {
                InsertKnot(U);
                if (delta < 0.0)
                {
                    ik++;
                }
            }

            SetOrigin(ik);
        }
    }

    /// <summary>Makes a periodic curve non-periodic (the same curve, with clamped ends).</summary>
    internal void SetNotPeriodic()
    {
        CheckMutable();
        if (myPeriodic)
        {
            ClearEvalRepresentation();
            int NbKnots, NbPoles;
            global::Kerf.Engine.Geometry.Splines.BSplCLib.PrepareUnperiodize(myDeg, myMults, out NbKnots, out NbPoles);

            NCollection_Array1<Point3D> npoles = new NCollection_Array1<Point3D>(1, NbPoles);

            NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, NbKnots);

            NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, NbKnots);

            NCollection_Array1<double> nweights = new NCollection_Array1<double>();

            if (IsRational())
            {
                nweights.Resize(1, NbPoles, false);
            }

            global::Kerf.Engine.Geometry.Splines.BSplCLib.Unperiodize(myDeg,
                                 myMults,
                                 myKnots,
                                 myPoles,
                                 InternalWeights(),
                                 nmults,
                                 nknots,
                                 npoles,
                                 myRational ? nweights : global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights());
            myPoles = npoles;
            if (IsRational())
            {
                myWeights = nweights;
            }
            else
            {
                myWeights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(myPoles.Length());
            }
            myMults = nmults;
            myKnots = nknots;
            myPeriodic = false;

            myMaxDerivInvOk = false;
            updateKnots();
        }
    }

    /// <summary>Replaces the pole Index (its weight is kept).</summary>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbPoles].</exception>
    internal void SetPole(int Index, in Point3D P)
    {
        CheckMutable();
        if (Index < 1 || Index > myPoles.Length())
        {
            throw new OutOfRangeException("BSpline curve: SetPole: index and #pole mismatch");
        }
        ClearEvalRepresentation();
        myPoles.SetValue(Index, P);
        myMaxDerivInvOk = false;
    }

    /// <summary>Replaces the pole Index and its weight.</summary>
    internal void SetPole(int Index, in Point3D P, double W)
    {
        CheckMutable();
        SetPole(Index, P);
        SetWeight(Index, W);
    }

    /// <summary>Changes the weight of the pole Index; the curve may become rational or non-rational.</summary>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbPoles].</exception>
    /// <exception cref="ConstructionErrorException">W is at most gp::Resolution().</exception>
    internal void SetWeight(int Index, double W)
    {
        CheckMutable();
        if (Index < 1 || Index > myPoles.Length())
        {
            throw new OutOfRangeException("BSpline curve: SetWeight: Index and #pole mismatch");
        }

        if (W <= Constants.Resolution())
        {
            throw new ConstructionErrorException("BSpline curve: SetWeight: Weight too small");
        }

        ClearEvalRepresentation();

        bool rat = IsRational() || (Math.Abs(W - 1.0) > Constants.Resolution());

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
                    myWeights = global::Kerf.Engine.Geometry.Splines.BSplCLib.UnitWeights(myPoles.Length());
                }
            }

            myRational = rat;
        }

        myMaxDerivInvOk = false;
    }

    /// <summary>
    /// Moves the point of parameter U to P by changing the poles Index1 to Index2 (as few as possible);
    /// FirstModifiedPole and LastModifiedPole are 0 if nothing could be done.
    /// </summary>
    /// <exception cref="OutOfRangeException">Index1 or Index2 is not in [1, NbPoles], or Index1 &gt; Index2.</exception>
    internal void MovePoint(double U,
                            in Point3D P,
                            int Index1,
                            int Index2,
                            out int FirstModifiedPole,
                            out int LastmodifiedPole)
    {
        CheckMutable();
        if (Index1 < 1 || Index1 > myPoles.Length() || Index2 < 1 || Index2 > myPoles.Length()
            || Index1 > Index2)
        {
            throw new OutOfRangeException("BSpline curve: MovePoint: Index and #pole mismatch");
        }
        NCollection_Array1<Point3D> npoles = new NCollection_Array1<Point3D>(1, myPoles.Length());
        Point3D P0;
        D0(U, out P0);
        Vector3D Displ = new Vector3D(P0, P);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.MovePoint(U,
                           Displ,
                           Index1,
                           Index2,
                           myDeg,
                           myPoles,
                           InternalWeights(),
                           myFlatKnots,
                           out FirstModifiedPole,
                           out LastmodifiedPole,
                           npoles);
        if (FirstModifiedPole != 0)
        {
            ClearEvalRepresentation();
            myPoles = npoles;
            myMaxDerivInvOk = false;
        }
    }

    /// <summary>
    /// Moves the point of parameter U to P and its tangent to Tangent. A periodic curve is made non-periodic
    /// first. ErrorStatus is 0 on success, and the curve unchanged otherwise.
    /// </summary>
    internal void MovePointAndTangent(double U,
                                      in Point3D P,
                                      in Vector3D Tangent,
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
        NCollection_Array1<Point3D> new_poles = new NCollection_Array1<Point3D>(1, myPoles.Length());
        Point3D P0;

        Vector3D delta_derivative;
        D1(U, out P0, out delta_derivative);
        Vector3D delta = new Vector3D(P0, P);
        for (ii = 1; ii <= 3; ii++)
        {
            delta_derivative.SetCoord(ii, Tangent.Coord(ii) - delta_derivative.Coord(ii));
        }
        global::Kerf.Engine.Geometry.Splines.BSplCLib.MovePointAndTangent(U,
                                     delta,
                                     delta_derivative,
                                     Tolerance,
                                     myDeg,
                                     StartingCondition,
                                     EndingCondition,
                                     myPoles,
                                     InternalWeights(),
                                     myFlatKnots,
                                     new_poles,
                                     out ErrorStatus);
        if (ErrorStatus == 0)
        {
            ClearEvalRepresentation();
            myPoles = new_poles;
            myMaxDerivInvOk = false;
        }
    }

    /// <summary>Recomputes the flat knots, the knot distribution and the continuity.</summary>
    protected void updateKnots()
    {
        myMaxDerivInvOk = false;

        int MaxKnotMult = 0;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotAnalysis(myDeg, myPeriodic, myKnots, myMults, out myKnotSet, out MaxKnotMult);

        if (myKnotSet == Uniform && !myPeriodic)
        {
            myFlatKnots.Resize(myKnots.Lower(), myKnots.Upper(), false);
            myFlatKnots.Assign(myKnots);
        }
        else
        {
            myFlatKnots.Resize(1, global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequenceLength(myMults, myDeg, myPeriodic), false);

            global::Kerf.Engine.Geometry.Splines.BSplCLib.KnotSequence(myKnots, myMults, myDeg, myPeriodic, myFlatKnots);
        }

        if (MaxKnotMult == 0)
        {
            mySmooth = CN;
        }
        else
        {
            switch (myDeg - MaxKnotMult)
            {
                case 0:
                    mySmooth = C0;
                    break;
                case 1:
                    mySmooth = C1;
                    break;
                case 2:
                    mySmooth = C2;
                    break;
                case 3:
                    mySmooth = C3;
                    break;
                default:
                    mySmooth = C3;
                    break;
            }
        }
    }

    /// <summary>
    /// Brings theParameter into [FirstParameter, LastParameter] by whole periods when the curve is periodic
    /// (LastParameter itself is left unchanged); does nothing otherwise.
    /// </summary>
    public void PeriodicNormalization(ref double theParameter)
    {
        if (!myPeriodic)
        {
            return;
        }

        double aFirst = myFlatKnots.Value(myDeg + 1);
        double aLast = myFlatKnots.Value(myFlatKnots.Upper() - myDeg);

        // If the parameter is already within the valid range, no adjustment is needed.
        // Note: Strictly speaking, the parameter should be within [aFirst, aLast), since for
        // periodic curves first and last parameters represent the same point.
        // However, we allow the case theParameter == aLast to be valid as well just because
        // there is already a lot of code that relies on this particular behavior.
        if (theParameter >= aFirst && theParameter <= aLast)
        {
            return;
        }

        double aPeriod = aLast - aFirst;

        // Shift by the exact number of full periods in one step - O(1) regardless of distance.
        double aNbPeriods = Math.Floor((theParameter - aFirst) / aPeriod);
        if (aNbPeriods != 0.0)
        {
            theParameter -= aNbPeriods * aPeriod;
        }

        // One-step correction for floating-point residual overshoot after the bulk shift.
        if (theParameter > aLast)
        {
            double aNewParam = theParameter - aPeriod;
            theParameter = aNewParam == theParameter ? NextAfter(aNewParam, aFirst) : aNewParam;
        }
        else if (theParameter < aFirst)
        {
            double aNewParam = theParameter + aPeriod;
            theParameter = aNewParam == theParameter ? NextAfter(aNewParam, aLast) : aNewParam;
        }
    }

    //=============================================================================================
    // Geom_BSplineCurve_1.cxx
    //=============================================================================================

    /// <summary>True if the curve is at least CN.</summary>
    /// <exception cref="RangeErrorException">N &lt; 0.</exception>
    public sealed override bool IsCN(int N)
    {
        Standard_RangeError_Raise_if(N < 0, "Geom_BSplineCurve::IsCN");

        switch (mySmooth)
        {
            case CN:
                return true;
            case C0:
                return N <= 0;
            case G1:
                return N <= 0;
            case C1:
                return N <= 1;
            case G2:
                return N <= 1;
            case C2:
                return N <= 2;
            case C3:
                return N <= 3
                         ? true
                         : N <= myDeg
                                  - global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxKnotMult(myMults, myMults.Lower() + 1, myMults.Upper() - 1);
            default:
                return false;
        }
    }

    /// <summary>
    /// True if the curve is G1 on [theTf, theTl]: at every inner knot of multiplicity at least the degree in the
    /// range, the tangents on both sides make an angle at most theAngTol (and, for a periodic curve whose range
    /// includes its ends, also at the ends).
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

            Point3D aP1, aP2;
            Vector3D aV1, aV2;
            LocalD1(aTpar, aNKnot - 1, aNKnot, out aP1, out aV1);
            LocalD1(aTpar, aNKnot, aNKnot + 1, out aP2, out aV2);

            if ((aV1.SquareMagnitude() <= Constants.Resolution()) || aV2.SquareMagnitude() <= Constants.Resolution())
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

        Point3D aP;
        Vector3D aV1b, aV2b;
        D1(Knot(FirstUKnotIndex()), out aP, out aV1b);
        D1(Knot(LastUKnotIndex()), out aP, out aV2b);

        if ((aV1b.SquareMagnitude() <= Constants.Resolution()) || aV2b.SquareMagnitude() <= Constants.Resolution())
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
    public sealed override bool IsClosed() => StartPoint().SquareDistance(EndPoint()) <= global::Kerf.Engine.Foundation.Precision.Computational();

    public sealed override bool IsPeriodic() => myPeriodic;

    /// <summary>The global continuity: C0 to C3, or CN without inner knots.</summary>
    public sealed override Continuity Continuity() => mySmooth;

    public int Degree() => myDeg;

    public sealed override Point3D EvalD0(double U)
    {
        if (GeomEval_RepUtils.TryEvalCurveD0(myEvalRep, U, out Point3D aEvalRepResult))
        {
            return aEvalRepResult;
        }

        Point3D P;
        int aSpanIndex = 0;
        double aNewU = U;
        // OCCT: the normalized aNewU is overwritten by LocateParameter, which is given U (dead code, harmless)
        PeriodicNormalization(ref aNewU);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(myDeg, myKnots, myMults, U, myPeriodic, ref aSpanIndex, out aNewU);
        if (aNewU < myKnots[aSpanIndex])
        {
            aSpanIndex--;
        }

        global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(aNewU, aSpanIndex, myDeg, myPeriodic, myPoles, InternalWeights(), myKnots, myMults, out P);
        return P;
    }

    public sealed override ResD1 EvalD1(double U)
    {
        if (GeomEval_RepUtils.TryEvalCurveD1(myEvalRep, U, out ResD1 aEvalRepResult))
        {
            return aEvalRepResult;
        }

        ResD1 aResult = new ResD1();
        int aSpanIndex = 0;
        double aNewU = U;
        PeriodicNormalization(ref aNewU);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(myDeg, myKnots, myMults, U, myPeriodic, ref aSpanIndex, out aNewU);
        if (aNewU < myKnots[aSpanIndex])
        {
            aSpanIndex--;
        }

        global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(aNewU,
                    aSpanIndex,
                    myDeg,
                    myPeriodic,
                    myPoles,
                    InternalWeights(),
                    myKnots,
                    myMults,
                    out aResult.Point,
                    out aResult.D1);
        return aResult;
    }

    public sealed override ResD2 EvalD2(double U)
    {
        if (GeomEval_RepUtils.TryEvalCurveD2(myEvalRep, U, out ResD2 aEvalRepResult))
        {
            return aEvalRepResult;
        }

        ResD2 aResult = new ResD2();
        int aSpanIndex = 0;
        double aNewU = U;
        PeriodicNormalization(ref aNewU);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(myDeg, myKnots, myMults, U, myPeriodic, ref aSpanIndex, out aNewU);
        if (aNewU < myKnots[aSpanIndex])
        {
            aSpanIndex--;
        }

        global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(aNewU,
                    aSpanIndex,
                    myDeg,
                    myPeriodic,
                    myPoles,
                    InternalWeights(),
                    myKnots,
                    myMults,
                    out aResult.Point,
                    out aResult.D1,
                    out aResult.D2);
        return aResult;
    }

    public sealed override ResD3 EvalD3(double U)
    {
        if (GeomEval_RepUtils.TryEvalCurveD3(myEvalRep, U, out ResD3 aEvalRepResult))
        {
            return aEvalRepResult;
        }

        ResD3 aResult = new ResD3();
        int aSpanIndex = 0;
        double aNewU = U;
        PeriodicNormalization(ref aNewU);
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(myDeg, myKnots, myMults, U, myPeriodic, ref aSpanIndex, out aNewU);
        if (aNewU < myKnots[aSpanIndex])
        {
            aSpanIndex--;
        }

        global::Kerf.Engine.Geometry.Splines.BSplCLib.D3(aNewU,
                    aSpanIndex,
                    myDeg,
                    myPeriodic,
                    myPoles,
                    InternalWeights(),
                    myKnots,
                    myMults,
                    out aResult.Point,
                    out aResult.D1,
                    out aResult.D2,
                    out aResult.D3);
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
        global::Kerf.Engine.Geometry.Splines.BSplCLib.DN(U, N, 0, myDeg, myPeriodic, myPoles, InternalWeights(), myFlatKnots, global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(), out V);
        return V;
    }

    /// <summary>The end point: the last pole when the last multiplicity is Degree + 1.</summary>
    public sealed override Point3D EndPoint()
    {
        if (myMults[myKnots.Upper()] == myDeg + 1)
        {
            return myPoles[myPoles.Upper()];
        }
        else
        {
            return Value(LastParameter());
        }
    }

    /// <summary>The index of the knot giving the first parameter (1 for a periodic curve).</summary>
    public int FirstUKnotIndex()
    {
        if (myPeriodic)
        {
            return 1;
        }
        else
        {
            return global::Kerf.Engine.Geometry.Splines.BSplCLib.FirstUKnotIndex(myDeg, myMults);
        }
    }

    public sealed override double FirstParameter() => myFlatKnots[myDeg + 1];

    /// <summary>The knot Index.</summary>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbKnots].</exception>
    public double Knot(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > myKnots.Length(), "Geom_BSplineCurve::Knot");
        return myKnots[Index];
    }

    /// <summary>NonUniform, Uniform, QuasiUniform or PiecewiseBezier.</summary>
    public BSplKnotDistribution KnotDistribution() => myKnotSet;

    /// <summary>Copies the knots K.Lower() to K.Upper() into K (OCCT's deprecated overload).</summary>
    /// <exception cref="DomainErrorException">K's bounds are outside [1, NbKnots].</exception>
    public void Knots(NCollection_Array1<double> K)
    {
        Standard_DomainError_Raise_if(K.Lower() < myKnots.Lower() || K.Upper() > myKnots.Upper(),
                                      "Geom_BSplineCurve::Knots");
        for (int anIdx = K.Lower(); anIdx <= K.Upper(); anIdx++)
        {
            K[anIdx] = myKnots[anIdx];
        }
    }

    /// <summary>The knots (a copy; index 0 is knot 1).</summary>
    public PackedList<double> Knots() => myKnots.ToPackedList();

    /// <summary>Copies the flat knots K.Lower() to K.Upper() into K (OCCT's deprecated overload).</summary>
    /// <exception cref="DomainErrorException">K's bounds are outside the knot sequence's.</exception>
    public void KnotSequence(NCollection_Array1<double> K)
    {
        Standard_DomainError_Raise_if(K.Lower() < myFlatKnots.Lower() || K.Upper() > myFlatKnots.Upper(),
                                      "Geom_BSplineCurve::KnotSequence");
        for (int anIdx = K.Lower(); anIdx <= K.Upper(); anIdx++)
        {
            K[anIdx] = myFlatKnots[anIdx];
        }
    }

    /// <summary>The knots repeated by their multiplicities (a copy).</summary>
    public PackedList<double> KnotSequence() => myFlatKnots.ToPackedList();

    /// <summary>The index of the knot giving the last parameter (NbKnots for a periodic curve).</summary>
    public int LastUKnotIndex()
    {
        if (myPeriodic)
        {
            return myKnots.Length();
        }
        else
        {
            return global::Kerf.Engine.Geometry.Splines.BSplCLib.LastUKnotIndex(myDeg, myMults);
        }
    }

    public sealed override double LastParameter() => myFlatKnots[myFlatKnots.Upper() - myDeg];

    /// <summary>The point at U, computed on the span between the knots FromK1 and ToK2.</summary>
    public Point3D LocalValue(double U, int FromK1, int ToK2)
    {
        Point3D P;
        LocalD0(U, FromK1, ToK2, out P);
        return P;
    }

    /// <exception cref="DomainErrorException">FromK1 == ToK2.</exception>
    public void LocalD0(double U, int FromK1, int ToK2, out Point3D P)
    {
        Standard_DomainError_Raise_if(FromK1 == ToK2, "Geom_BSplineCurve::LocalValue");

        double u = U;
        int index = 0;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(myDeg, myFlatKnots, U, myPeriodic, FromK1, ToK2, out index, out u);
        index = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(myDeg, index, myMults, myPeriodic);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.D0(u,
                    index,
                    myDeg,
                    myPeriodic,
                    myPoles,
                    InternalWeights(),
                    myFlatKnots,
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    out P);
    }

    /// <exception cref="DomainErrorException">FromK1 == ToK2.</exception>
    public void LocalD1(double U, int FromK1, int ToK2, out Point3D P, out Vector3D V1)
    {
        Standard_DomainError_Raise_if(FromK1 == ToK2, "Geom_BSplineCurve::LocalD1");

        double u = U;
        int index = 0;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(myDeg, myFlatKnots, U, myPeriodic, FromK1, ToK2, out index, out u);
        index = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(myDeg, index, myMults, myPeriodic);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.D1(u,
                    index,
                    myDeg,
                    myPeriodic,
                    myPoles,
                    InternalWeights(),
                    myFlatKnots,
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    out P,
                    out V1);
    }

    /// <exception cref="DomainErrorException">FromK1 == ToK2.</exception>
    public void LocalD2(double U, int FromK1, int ToK2, out Point3D P, out Vector3D V1, out Vector3D V2)
    {
        Standard_DomainError_Raise_if(FromK1 == ToK2, "Geom_BSplineCurve::LocalD2");

        double u = U;
        int index = 0;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(myDeg, myFlatKnots, U, myPeriodic, FromK1, ToK2, out index, out u);
        index = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(myDeg, index, myMults, myPeriodic);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.D2(u,
                    index,
                    myDeg,
                    myPeriodic,
                    myPoles,
                    InternalWeights(),
                    myFlatKnots,
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    out P,
                    out V1,
                    out V2);
    }

    /// <exception cref="DomainErrorException">FromK1 == ToK2.</exception>
    public void LocalD3(double U, int FromK1, int ToK2, out Point3D P, out Vector3D V1, out Vector3D V2, out Vector3D V3)
    {
        Standard_DomainError_Raise_if(FromK1 == ToK2, "Geom_BSplineCurve::LocalD3");

        double u = U;
        int index = 0;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(myDeg, myFlatKnots, U, myPeriodic, FromK1, ToK2, out index, out u);
        index = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(myDeg, index, myMults, myPeriodic);

        global::Kerf.Engine.Geometry.Splines.BSplCLib.D3(u,
                    index,
                    myDeg,
                    myPeriodic,
                    myPoles,
                    InternalWeights(),
                    myFlatKnots,
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    out P,
                    out V1,
                    out V2,
                    out V3);
    }

    /// <exception cref="DomainErrorException">FromK1 == ToK2.</exception>
    public Vector3D LocalDN(double U, int FromK1, int ToK2, int N)
    {
        Standard_DomainError_Raise_if(FromK1 == ToK2, "Geom_BSplineCurve::LocalD3"); // OCCT: says LocalD3 in LocalDN

        double u = U;
        int index = 0;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.LocateParameter(myDeg, myFlatKnots, U, myPeriodic, FromK1, ToK2, out index, out u);
        index = global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatIndex(myDeg, index, myMults, myPeriodic);

        Vector3D V;
        global::Kerf.Engine.Geometry.Splines.BSplCLib.DN(u,
                    N,
                    index,
                    myDeg,
                    myPeriodic,
                    myPoles,
                    InternalWeights(),
                    myFlatKnots,
                    global::Kerf.Engine.Geometry.Splines.BSplCLib.NoMults(),
                    out V);
        return V;
    }

    /// <summary>The multiplicity of the knot Index.</summary>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbKnots].</exception>
    public int Multiplicity(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > myMults.Length(), "Geom_BSplineCurve::Multiplicity");
        return myMults[Index];
    }

    /// <summary>Copies the multiplicities into M (OCCT's deprecated overload).</summary>
    /// <exception cref="DimensionErrorException">M has not NbKnots elements.</exception>
    public void Multiplicities(NCollection_Array1<int> M)
    {
        Standard_DimensionError_Raise_if(M.Length() != myMults.Length(), "Geom_BSplineCurve::Multiplicities");
        M.Assign(myMults);
    }

    /// <summary>The multiplicities (a copy; index 0 is knot 1's).</summary>
    public PackedList<int> Multiplicities() => myMults.ToPackedList();

    public int NbKnots() => myKnots.Length();

    public int NbPoles() => myPoles.Length();

    /// <summary>The pole Index.</summary>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbPoles].</exception>
    public Point3D Pole(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > myPoles.Length(), "Geom_BSplineCurve::Pole");
        return myPoles[Index];
    }

    /// <summary>Copies the poles into P (OCCT's deprecated overload).</summary>
    /// <exception cref="DimensionErrorException">P has not NbPoles elements.</exception>
    public void Poles(NCollection_Array1<Point3D> P)
    {
        Standard_DimensionError_Raise_if(P.Length() != myPoles.Length(), "Geom_BSplineCurve::Poles");
        P.Assign(myPoles);
    }

    /// <summary>The poles (a copy; index 0 is pole 1).</summary>
    public PackedList<Point3D> Poles() => myPoles.ToPackedList();

    /// <summary>The start point: the first pole when the first multiplicity is Degree + 1.</summary>
    public sealed override Point3D StartPoint()
    {
        if (myMults[1] == myDeg + 1)
        {
            return myPoles[1];
        }
        else
        {
            return Value(FirstParameter());
        }
    }

    /// <summary>The weight of the pole Index (1 for a non-rational curve).</summary>
    /// <exception cref="OutOfRangeException">Index is not in [1, NbPoles].</exception>
    public double Weight(int Index)
    {
        Standard_OutOfRange_Raise_if(Index < 1 || Index > myPoles.Length(), "Geom_BSplineCurve::Weight");
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
        Standard_DimensionError_Raise_if(W.Length() != myPoles.Length(), "Geom_BSplineCurve::Weights");
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

    /// <summary>The weights (a copy), or null for a non-rational curve (OCCT returns a null pointer).</summary>
    public PackedList<double>? Weights() => IsRational() ? myWeights.ToPackedList() : null;

    /// <summary>The weights, always NbPoles of them (1s for a non-rational curve); a copy.</summary>
    public PackedList<double> WeightsArray() => myWeights.ToPackedList();

    public bool IsRational() => myRational;

    /// <summary>Applies T to the poles.</summary>
    protected internal sealed override void Transform(in Transform3D T)
    {
        CheckMutable();
        ClearEvalRepresentation();
        for (int I = 1; I <= myPoles.Length(); I++)
        {
            myPoles.ChangeValue(I).Transform(T);
        }
        myMaxDerivInvOk = false;
    }

    /// <summary>
    /// The knots around U within ParametricTolerance: I1 = I2 = the knot's index when U is on a knot, otherwise
    /// the knots I1 &lt; U &lt; I2 (I1 = 0 or I2 = NbKnots + 1 outside the range). With WithKnotRepetition the
    /// indexes are in the knot sequence.
    /// </summary>
    public void LocateU(double U,
                        double ParametricTolerance,
                        out int I1,
                        out int I2,
                        bool WithKnotRepetition = false)
    {
        double NewU = U;
        NCollection_Array1<double> CKnots = WithKnotRepetition ? myFlatKnots : myKnots;

        PeriodicNormalization(ref NewU); // Handle periodicity

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
            global::Kerf.Engine.Geometry.Splines.BSplCLib.Hunt(CKnots, NewU, out I1);
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
    /// |f(t1) - f(t0)| &lt; Tolerance3D.
    /// </summary>
    /// <remarks>Non-const in OCCT, because it caches a value; not a mutator, so it works on a frozen curve.</remarks>
    public void Resolution(double Tolerance3D, out double UTolerance)
    {
        // OCCT.NET: a cache filled on a possibly shared curve; the flag is published after the value.
        if (!Volatile.Read(ref myMaxDerivInvOk))
        {
            double aMaxDerivInv;
            if (myPeriodic)
            {
                int NbKnots, NbPoles;
                global::Kerf.Engine.Geometry.Splines.BSplCLib.PrepareUnperiodize(myDeg, myMults, out NbKnots, out NbPoles);
                NCollection_Array1<Point3D> new_poles = new NCollection_Array1<Point3D>(1, NbPoles);
                NCollection_Array1<double> new_weights = new NCollection_Array1<double>(1, NbPoles);
                for (int ii = 1; ii <= NbPoles; ii++)
                {
                    new_poles[ii] = myPoles[(ii - 1) % myPoles.Length() + 1];
                }
                if (myRational)
                {
                    for (int ii = 1; ii <= NbPoles; ii++)
                    {
                        new_weights[ii] = myWeights[(ii - 1) % myPoles.Length() + 1];
                    }
                }
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Resolution(new_poles,
                                    myRational ? new_weights : global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights(),
                                    new_poles.Length(),
                                    myFlatKnots,
                                    myDeg,
                                    1.0,
                                    out aMaxDerivInv);
            }
            else
            {
                global::Kerf.Engine.Geometry.Splines.BSplCLib.Resolution(myPoles,
                                    InternalWeights(),
                                    myPoles.Length(),
                                    myFlatKnots,
                                    myDeg,
                                    1.0,
                                    out aMaxDerivInv);
            }
            myMaxDerivInv = aMaxDerivInv;
            Volatile.Write(ref myMaxDerivInvOk, true);
        }
        UTolerance = Tolerance3D * myMaxDerivInv;
    }

    /// <summary>
    /// True if theOther has the same degree, rationality, knots (within Precision::Parametric(thePreci)),
    /// multiplicities, poles (each coordinate within thePreci) and weights (within Epsilon).
    /// </summary>
    public bool IsEqual(Geom_BSplineCurve theOther, double thePreci)
    {
        if (myKnots.IsEmpty() || myPoles.IsEmpty() || myMults.IsEmpty() || myDeg != theOther.Degree()
            || myRational != theOther.IsRational() || myKnots.Length() != theOther.NbKnots()
            || myPoles.Length() != theOther.NbPoles())
        {
            return false;
        }

        for (int aPoleIter = 1; aPoleIter <= myPoles.Length(); ++aPoleIter)
        {
            Point3D aPole1 = myPoles[aPoleIter];
            Point3D aPole2 = theOther.Pole(aPoleIter);
            if (Math.Abs(aPole1.X() - aPole2.X()) > thePreci || Math.Abs(aPole1.Y() - aPole2.Y()) > thePreci
                || Math.Abs(aPole1.Z() - aPole2.Z()) > thePreci)
            {
                return false;
            }
        }

        for (int aKnotIter = 1; aKnotIter <= myKnots.Length(); ++aKnotIter)
        {
            if (Math.Abs(myKnots[aKnotIter] - theOther.Knot(aKnotIter)) > global::Kerf.Engine.Foundation.Precision.Parametric(thePreci))
            {
                return false;
            }
        }

        for (int aMultIter = 1; aMultIter <= myMults.Length(); ++aMultIter)
        {
            if (myMults[aMultIter] != theOther.Multiplicity(aMultIter))
            {
                return false;
            }
        }

        if (!myRational)
        {
            return true;
        }

        for (int aWeightIter = 1; aWeightIter <= myWeights.Length(); ++aWeightIter)
        {
            if (Math.Abs((double)(myWeights[aWeightIter] - theOther.Weight(aWeightIter)))
                > Epsilon(myWeights[aWeightIter]))
            {
                return false;
            }
        }

        return true;
    }

    //=============================================================================================
    // Internal access to the stored arrays (OCCT.NET)
    //=============================================================================================

    /// <summary>The stored poles: shared, don't modify (OCCT's <c>Poles()</c> const reference).</summary>
    internal NCollection_Array1<Point3D> InternalPoles() => myPoles;

    /// <summary>The stored weights, or null for a non-rational curve (OCCT's <c>Weights()</c> pointer). Don't modify.</summary>
    internal NCollection_Array1<double>? InternalWeights() => IsRational() ? myWeights : global::Kerf.Engine.Geometry.Splines.BSplCLib.NoWeights();

    /// <summary>The weights, always NbPoles of them (OCCT's <c>WeightsArray()</c>): shared, don't modify.</summary>
    internal NCollection_Array1<double> InternalWeightsArray() => myWeights;

    /// <summary>The stored knots: shared, don't modify (OCCT's <c>Knots()</c> const reference).</summary>
    internal NCollection_Array1<double> InternalKnots() => myKnots;

    /// <summary>The stored multiplicities: shared, don't modify (OCCT's <c>Multiplicities()</c> const reference).</summary>
    internal NCollection_Array1<int> InternalMultiplicities() => myMults;

    /// <summary>The stored knot sequence: shared, don't modify (OCCT's <c>KnotSequence()</c> const reference).</summary>
    internal NCollection_Array1<double> InternalKnotSequence() => myFlatKnots;

    //=============================================================================================
    // Non-mutating twins (OCCT.NET)
    //=============================================================================================

    private Geom_BSplineCurve CopyBSpline() => (Geom_BSplineCurve)Copy();

    /// <summary>A copy with the degree raised to Degree.</summary>
    public Geom_BSplineCurve DegreeIncreased(int Degree)
    {
        Geom_BSplineCurve aCopy = CopyBSpline();
        aCopy.IncreaseDegree(Degree);
        return aCopy;
    }

    /// <summary>A copy with the multiplicity of the knot Index raised to M.</summary>
    public Geom_BSplineCurve MultiplicityIncreased(int Index, int M)
    {
        Geom_BSplineCurve aCopy = CopyBSpline();
        aCopy.IncreaseMultiplicity(Index, M);
        return aCopy;
    }

    /// <summary>A copy with the multiplicities of the knots I1 to I2 raised to M.</summary>
    public Geom_BSplineCurve MultiplicityIncreased(int I1, int I2, int M)
    {
        Geom_BSplineCurve aCopy = CopyBSpline();
        aCopy.IncreaseMultiplicity(I1, I2, M);
        return aCopy;
    }

    /// <summary>A copy with the multiplicities of the knots I1 to I2 raised by Step.</summary>
    public Geom_BSplineCurve MultiplicityIncremented(int I1, int I2, int Step)
    {
        Geom_BSplineCurve aCopy = CopyBSpline();
        aCopy.IncrementMultiplicity(I1, I2, Step);
        return aCopy;
    }

    /// <summary>A copy with the knot U inserted (see <see cref="InsertKnot"/>).</summary>
    public Geom_BSplineCurve KnotInserted(double U, int M = 1, double ParametricTolerance = 0.0, bool Add = true)
    {
        Geom_BSplineCurve aCopy = CopyBSpline();
        aCopy.InsertKnot(U, M, ParametricTolerance, Add);
        return aCopy;
    }

    /// <summary>A copy with the knots inserted (see <see cref="InsertKnots"/>; Add defaults to false).</summary>
    public Geom_BSplineCurve KnotsInserted(NCollection_Array1<double> Knots,
                                           NCollection_Array1<int> Mults,
                                           double ParametricTolerance = 0.0,
                                           bool Add = false)
    {
        Geom_BSplineCurve aCopy = CopyBSpline();
        aCopy.InsertKnots(Knots, Mults, ParametricTolerance, Add);
        return aCopy;
    }

    /// <summary>
    /// A copy with the multiplicity of the knot Index lowered to M (see <see cref="RemoveKnot"/>), or null if
    /// the curve would move by more than Tolerance.
    /// </summary>
    public Geom_BSplineCurve? KnotRemoved(int Index, int M, double Tolerance)
    {
        Geom_BSplineCurve aCopy = CopyBSpline();
        return aCopy.RemoveKnot(Index, M, Tolerance) ? aCopy : null;
    }

    /// <summary>A copy segmented between U1 and U2 (see <see cref="Segment"/>).</summary>
    public Geom_BSplineCurve Segmented(double U1, double U2, double theTolerance = Precision_PConfusion)
    {
        Geom_BSplineCurve aCopy = CopyBSpline();
        aCopy.Segment(U1, U2, theTolerance);
        return aCopy;
    }

    public Geom_BSplineCurve WithKnot(int Index, double K)
    {
        Geom_BSplineCurve aCopy = CopyBSpline();
        aCopy.SetKnot(Index, K);
        return aCopy;
    }

    public Geom_BSplineCurve WithKnots(NCollection_Array1<double> K)
    {
        Geom_BSplineCurve aCopy = CopyBSpline();
        aCopy.SetKnots(K);
        return aCopy;
    }

    public Geom_BSplineCurve WithKnot(int Index, double K, int M)
    {
        Geom_BSplineCurve aCopy = CopyBSpline();
        aCopy.SetKnot(Index, K, M);
        return aCopy;
    }

    /// <summary>A periodic copy (see <see cref="SetPeriodic"/>).</summary>
    public Geom_BSplineCurve WithPeriodic()
    {
        Geom_BSplineCurve aCopy = CopyBSpline();
        aCopy.SetPeriodic();
        return aCopy;
    }

    public Geom_BSplineCurve WithOrigin(int Index)
    {
        Geom_BSplineCurve aCopy = CopyBSpline();
        aCopy.SetOrigin(Index);
        return aCopy;
    }

    public Geom_BSplineCurve WithOrigin(double U, double Tol)
    {
        Geom_BSplineCurve aCopy = CopyBSpline();
        aCopy.SetOrigin(U, Tol);
        return aCopy;
    }

    /// <summary>A non-periodic copy (see <see cref="SetNotPeriodic"/>).</summary>
    public Geom_BSplineCurve WithNotPeriodic()
    {
        Geom_BSplineCurve aCopy = CopyBSpline();
        aCopy.SetNotPeriodic();
        return aCopy;
    }

    public Geom_BSplineCurve WithPole(int Index, in Point3D P)
    {
        Geom_BSplineCurve aCopy = CopyBSpline();
        aCopy.SetPole(Index, P);
        return aCopy;
    }

    public Geom_BSplineCurve WithPole(int Index, in Point3D P, double W)
    {
        Geom_BSplineCurve aCopy = CopyBSpline();
        aCopy.SetPole(Index, P, W);
        return aCopy;
    }

    public Geom_BSplineCurve WithWeight(int Index, double W)
    {
        Geom_BSplineCurve aCopy = CopyBSpline();
        aCopy.SetWeight(Index, W);
        return aCopy;
    }

    /// <summary>A copy whose point of parameter U is moved to P (see <see cref="MovePoint"/>).</summary>
    public Geom_BSplineCurve PointMoved(double U,
                                        in Point3D P,
                                        int Index1,
                                        int Index2,
                                        out int FirstModifiedPole,
                                        out int LastModifiedPole)
    {
        Geom_BSplineCurve aCopy = CopyBSpline();
        aCopy.MovePoint(U, P, Index1, Index2, out FirstModifiedPole, out LastModifiedPole);
        return aCopy;
    }

    /// <summary>A copy whose point and tangent at U are moved (see <see cref="MovePointAndTangent"/>).</summary>
    public Geom_BSplineCurve PointAndTangentMoved(double U,
                                                  in Point3D P,
                                                  in Vector3D Tangent,
                                                  double Tolerance,
                                                  int StartingCondition,
                                                  int EndingCondition,
                                                  out int ErrorStatus)
    {
        Geom_BSplineCurve aCopy = CopyBSpline();
        aCopy.MovePointAndTangent(U, P, Tangent, Tolerance, StartingCondition, EndingCondition, out ErrorStatus);
        return aCopy;
    }

    /// <summary>A copy evaluated through a copy of theDesc (null: without a representation).</summary>
    public Geom_BSplineCurve WithEvalRepresentation(GeomEval_RepCurveDesc.Base? theDesc)
    {
        GeomEval_RepUtils.ValidateCurveDesc(theDesc, this);
        Geom_BSplineCurve aCopy = CopyBSpline();
        aCopy.SetEvalRepresentation(GeomEval_RepUtils.CloneCurveDesc(theDesc));
        return aCopy;
    }

    /// <summary>A copy without an evaluation representation (the twin of ClearEvalRepresentation).</summary>
    public Geom_BSplineCurve WithoutEvalRepresentation()
    {
        Geom_BSplineCurve aCopy = CopyBSpline();
        aCopy.ClearEvalRepresentation();
        return aCopy;
    }
}
