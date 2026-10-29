// Port of: src/ModelingData/TKG3d/Geom/Geom_BSplineSurface.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_BSplineSurface.cxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_BSplineSurface_1.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_BSplKnotDistribution;
using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// A B-spline surface: uniform or non-uniform, rational or non-rational, periodic or non-periodic in each
/// parametric direction. It is defined by its degrees in U and V (1 to <see cref="MaxDegree"/>, 25), a
/// rectangular grid of poles (with weights when rational), and in each direction a strictly increasing knot
/// sequence with multiplicities, all indexed from 1. The rows of the pole grid follow U, the columns V.
/// </summary>
/// <remarks>
/// <para>
/// Arrays: <see cref="UKnots()"/>, <see cref="VKnots()"/>, <see cref="UMultiplicities()"/>,
/// <see cref="VMultiplicities()"/>, <see cref="UKnotSequence()"/> and <see cref="VKnotSequence()"/> return
/// immutable <see cref="PackedList{T}"/>s (index 0 is OCCT's index 1); <see cref="Poles()"/>,
/// <see cref="Weights()"/> and <see cref="WeightsArray"/> return new copies of the two-dimensional arrays (OCCT
/// returns references to the surface's arrays). Code inside OCCT.NET uses the internal <c>Internal*</c>
/// accessors, which return the stored arrays (shared: don't modify).
/// </para>
/// <para>
/// The evaluation representation (<see cref="HasEvalRepresentation"/>) is set by
/// <see cref="WithEvalRepresentation"/>, which stores a copy of the descriptor. Modifications drop it, as in OCCT.
/// </para>
/// </remarks>
public class Geom_BSplineSurface : Geom_BoundedSurface
{
    private NCollection_Array2<gp_Pnt> myPoles = new NCollection_Array2<gp_Pnt>();
    private NCollection_Array2<double> myWeights = new NCollection_Array2<double>();
    private NCollection_Array1<double> myUKnots = new NCollection_Array1<double>();
    private NCollection_Array1<double> myVKnots = new NCollection_Array1<double>();
    private NCollection_Array1<double> myUFlatKnots = new NCollection_Array1<double>();
    private NCollection_Array1<double> myVFlatKnots = new NCollection_Array1<double>();
    private NCollection_Array1<int> myUMults = new NCollection_Array1<int>();
    private NCollection_Array1<int> myVMults = new NCollection_Array1<int>();
    private GeomEval_RepSurfaceDesc.Base? myEvalRep;
    private int myUDeg = 0;
    private int myVDeg = 0;
    private bool myUPeriodic = false;
    private bool myVPeriodic = false;
    private bool myURational = false;
    private bool myVRational = false;
    private GeomAbs_BSplKnotDistribution myUKnotSet = GeomAbs_NonUniform;
    private GeomAbs_BSplKnotDistribution myVKnotSet = GeomAbs_NonUniform;
    private GeomAbs_Shape myUSmooth = GeomAbs_C0;
    private GeomAbs_Shape myVSmooth = GeomAbs_C0;
    private double myUMaxDerivInv = 0.0;
    private double myVMaxDerivInv = 0.0;
    private volatile bool myMaxDerivInvOk = false;

    private const double Precision_PConfusion = 1.0e-7 * 0.01;

    //=============================================================================================
    // Geom_BSplineSurface.cxx
    //=============================================================================================

    /// <summary>True if an evaluation representation is attached.</summary>
    public bool HasEvalRepresentation() => myEvalRep is not null;

    /// <summary>The evaluation representation descriptor (may be null). Stored, not copied: don't modify.</summary>
    internal GeomEval_RepSurfaceDesc.Base? EvalRepresentation() => myEvalRep;

    /// <summary>Sets a new evaluation representation, after checking it (no circular reference).</summary>
    internal void SetEvalRepresentation(GeomEval_RepSurfaceDesc.Base? theDesc)
    {
        CheckMutable();
        GeomEval_RepUtils.ValidateSurfaceDesc(theDesc, this);
        myEvalRep = theDesc;
    }

    /// <summary>Removes the evaluation representation.</summary>
    internal void ClearEvalRepresentation()
    {
        CheckMutable();
        myEvalRep = null;
    }

    private static void CheckSurfaceData(NCollection_Array2<gp_Pnt> SPoles,
                                         NCollection_Array1<double> SUKnots,
                                         NCollection_Array1<double> SVKnots,
                                         NCollection_Array1<int> SUMults,
                                         NCollection_Array1<int> SVMults,
                                         int UDegree,
                                         int VDegree,
                                         bool UPeriodic,
                                         bool VPeriodic)
    {
        if (UDegree < 1 || UDegree > Geom_BSplineSurface.MaxDegree() || VDegree < 1
            || VDegree > Geom_BSplineSurface.MaxDegree())
        {
            throw new Standard_ConstructionError("Geom_BSplineSurface: invalid degree");
        }

        if (SPoles.ColLength() < 2 || SPoles.RowLength() < 2)
        {
            throw new Standard_ConstructionError("Geom_BSplineSurface: at least 2 poles required");
        }

        if (SUKnots.Length() != SUMults.Length() || SVKnots.Length() != SVMults.Length())
        {
            throw new Standard_ConstructionError("Geom_BSplineSurface: Knot and Mult array size mismatch");
        }

        int i;
        for (i = SUKnots.Lower(); i < SUKnots.Upper(); i++)
        {
            if (SUKnots[i + 1] - SUKnots[i] <= Epsilon(Math.Abs(SUKnots[i])))
            {
                throw new Standard_ConstructionError("Geom_BSplineSurface: UKnots interval values too close");
            }
        }

        for (i = SVKnots.Lower(); i < SVKnots.Upper(); i++)
        {
            if (SVKnots[i + 1] - SVKnots[i] <= Epsilon(Math.Abs(SVKnots[i])))
            {
                throw new Standard_ConstructionError("Geom_BSplineSurface: VKnots interval values too close");
            }
        }

        if (SPoles.ColLength() != BSplCLib.NbPoles(UDegree, UPeriodic, SUMults))
        {
            throw new Standard_ConstructionError("Geom_BSplineSurface: # U Poles and degree mismatch");
        }

        if (SPoles.RowLength() != BSplCLib.NbPoles(VDegree, VPeriodic, SVMults))
        {
            throw new Standard_ConstructionError("Geom_BSplineSurface: # V Poles and degree mismatch");
        }
    }

    /// <summary>Checks the rationality of an array of weights in each direction.</summary>
    internal static void Rational(NCollection_Array2<double> Weights, out bool Urational, out bool Vrational)
    {
        int I, J;
        J = Weights.LowerCol();
        Vrational = false;
        while (!Vrational && J <= Weights.UpperCol())
        {
            I = Weights.LowerRow();
            while (!Vrational && I <= Weights.UpperRow() - 1)
            {
                Vrational = (Math.Abs(Weights[I, J] - Weights[I + 1, J]) > Epsilon(Math.Abs(Weights[I, J])));
                I++;
            }
            J++;
        }

        I = Weights.LowerRow();
        Urational = false;
        while (!Urational && I <= Weights.UpperRow())
        {
            J = Weights.LowerCol();
            while (!Urational && J <= Weights.UpperCol() - 1)
            {
                Urational = (Math.Abs(Weights[I, J] - Weights[I, J + 1]) > Epsilon(Math.Abs(Weights[I, J])));
                J++;
            }
            I++;
        }
    }

    /// <summary>A new, unfrozen surface equal to this one.</summary>
    public sealed override Geom_Geometry Copy() => new Geom_BSplineSurface(this);

    /// <summary>A copy of theOther, without validation (OCCT's copy constructor).</summary>
    public Geom_BSplineSurface(Geom_BSplineSurface theOther)
    {
        myPoles = new NCollection_Array2<gp_Pnt>(theOther.myPoles);
        myWeights = (theOther.myURational || theOther.myVRational)
                      ? new NCollection_Array2<double>(theOther.myWeights)
                      : BSplSLib.UnitWeights(theOther.myPoles.ColLength(), theOther.myPoles.RowLength());
        myUKnots = new NCollection_Array1<double>(theOther.myUKnots);
        myVKnots = new NCollection_Array1<double>(theOther.myVKnots);
        myUFlatKnots = new NCollection_Array1<double>(theOther.myUFlatKnots);
        myVFlatKnots = new NCollection_Array1<double>(theOther.myVFlatKnots);
        myUMults = new NCollection_Array1<int>(theOther.myUMults);
        myVMults = new NCollection_Array1<int>(theOther.myVMults);
        myEvalRep = GeomEval_RepUtils.CloneSurfaceDesc(theOther.myEvalRep);
        myUDeg = theOther.myUDeg;
        myVDeg = theOther.myVDeg;
        myUPeriodic = theOther.myUPeriodic;
        myVPeriodic = theOther.myVPeriodic;
        myURational = theOther.myURational;
        myVRational = theOther.myVRational;
        myUKnotSet = theOther.myUKnotSet;
        myVKnotSet = theOther.myVKnotSet;
        myUSmooth = theOther.myUSmooth;
        myVSmooth = theOther.myVSmooth;
        myUMaxDerivInv = theOther.myUMaxDerivInv;
        myVMaxDerivInv = theOther.myVMaxDerivInv;
        myMaxDerivInvOk = false;
    }

    /// <summary>A non-rational B-spline surface.</summary>
    /// <exception cref="Standard_ConstructionError">
    /// A degree not in [1, MaxDegree()], fewer than 2 poles in a direction, knots and multiplicities of different
    /// lengths, knots not increasing, or numbers of poles that don't match the multiplicities.
    /// </exception>
    public Geom_BSplineSurface(NCollection_Array2<gp_Pnt> Poles,
                               NCollection_Array1<double> UKnots,
                               NCollection_Array1<double> VKnots,
                               NCollection_Array1<int> UMults,
                               NCollection_Array1<int> VMults,
                               int UDegree,
                               int VDegree,
                               bool UPeriodic = false,
                               bool VPeriodic = false)
    {
        myURational = false;
        myVRational = false;
        myMaxDerivInvOk = false;

        // check
        CheckSurfaceData(Poles, UKnots, VKnots, UMults, VMults, UDegree, VDegree, UPeriodic, VPeriodic);

        // set scalar fields
        myUDeg = UDegree;
        myVDeg = VDegree;
        myUPeriodic = UPeriodic;
        myVPeriodic = VPeriodic;

        // copy arrays
        myPoles.Resize(1, Poles.ColLength(), 1, Poles.RowLength(), false);
        myPoles.CopyValues(Poles);
        myWeights = BSplSLib.UnitWeights(Poles.ColLength(), Poles.RowLength());

        myUKnots.Resize(1, UKnots.Length(), false);
        myUKnots.CopyValues(UKnots);
        myUMults.Resize(1, UMults.Length(), false);
        myUMults.CopyValues(UMults);
        myVKnots.Resize(1, VKnots.Length(), false);
        myVKnots.CopyValues(VKnots);
        myVMults.Resize(1, VMults.Length(), false);
        myVMults.CopyValues(VMults);

        updateUKnots();
        updateVKnots();
    }

    /// <summary>A rational B-spline surface (non-rational if the weights are all equal in each direction).</summary>
    /// <exception cref="Standard_ConstructionError">
    /// As the non-rational constructor, or weights of a different size from the poles, or a weight not greater
    /// than gp::Resolution().
    /// </exception>
    public Geom_BSplineSurface(NCollection_Array2<gp_Pnt> Poles,
                               NCollection_Array2<double> Weights,
                               NCollection_Array1<double> UKnots,
                               NCollection_Array1<double> VKnots,
                               NCollection_Array1<int> UMults,
                               NCollection_Array1<int> VMults,
                               int UDegree,
                               int VDegree,
                               bool UPeriodic = false,
                               bool VPeriodic = false)
    {
        myURational = false;
        myVRational = false;
        myMaxDerivInvOk = false;

        // check weights
        if (Weights.ColLength() != Poles.ColLength())
        {
            throw new Standard_ConstructionError("Geom_BSplineSurface: U Weights and Poles array size mismatch");
        }

        if (Weights.RowLength() != Poles.RowLength())
        {
            throw new Standard_ConstructionError("Geom_BSplineSurface: V Weights and Poles array size mismatch");
        }

        int i, j;
        for (i = Weights.LowerRow(); i <= Weights.UpperRow(); i++)
        {
            for (j = Weights.LowerCol(); j <= Weights.UpperCol(); j++)
            {
                if (Weights[i, j] <= gp.Resolution())
                {
                    throw new Standard_ConstructionError("Geom_BSplineSurface: Weights values too small");
                }
            }
        }

        // check really rational
        Rational(Weights, out myURational, out myVRational);

        // check
        CheckSurfaceData(Poles, UKnots, VKnots, UMults, VMults, UDegree, VDegree, UPeriodic, VPeriodic);

        // set scalar fields
        myUDeg = UDegree;
        myVDeg = VDegree;
        myUPeriodic = UPeriodic;
        myVPeriodic = VPeriodic;

        // copy arrays
        myPoles.Resize(1, Poles.ColLength(), 1, Poles.RowLength(), false);
        myPoles.CopyValues(Poles);

        if (myURational || myVRational)
        {
            myWeights.Resize(1, Poles.ColLength(), 1, Poles.RowLength(), false);
            myWeights.CopyValues(Weights);
        }
        else
        {
            myWeights = BSplSLib.UnitWeights(Poles.ColLength(), Poles.RowLength());
        }

        myUKnots.Resize(1, UKnots.Length(), false);
        myUKnots.CopyValues(UKnots);
        myUMults.Resize(1, UMults.Length(), false);
        myUMults.CopyValues(UMults);
        myVKnots.Resize(1, VKnots.Length(), false);
        myVKnots.CopyValues(VKnots);
        myVMults.Resize(1, VMults.Length(), false);
        myVMults.CopyValues(VMults);

        updateUKnots();
        updateVKnots();
    }

    /// <summary>Exchanges the U and V parametric directions; the poles grid is transposed.</summary>
    internal void ExchangeUV()
    {
        CheckMutable();
        ClearEvalRepresentation();
        int LC = myPoles.LowerCol();
        int UC = myPoles.UpperCol();
        int LR = myPoles.LowerRow();
        int UR = myPoles.UpperRow();

        NCollection_Array2<gp_Pnt> npoles = new NCollection_Array2<gp_Pnt>(LC, UC, LR, UR);
        NCollection_Array2<double> nweights = new NCollection_Array2<double>();
        if (myURational || myVRational)
        {
            nweights = new NCollection_Array2<double>(LC, UC, LR, UR);
        }

        for (int i = LC; i <= UC; i++)
        {
            for (int j = LR; j <= UR; j++)
            {
                npoles[i, j] = myPoles[j, i];
                if (myURational || myVRational)
                {
                    nweights.ChangeValue(i, j) = myWeights.Value(j, i);
                }
            }
        }

        myPoles = npoles;
        if (myURational || myVRational)
        {
            myWeights = nweights;
        }
        else
        {
            myWeights = BSplSLib.UnitWeights(myPoles.ColLength(), myPoles.RowLength());
        }

        (myURational, myVRational) = (myVRational, myURational);
        (myUPeriodic, myVPeriodic) = (myVPeriodic, myUPeriodic);
        (myUDeg, myVDeg) = (myVDeg, myUDeg);
        (myUKnots, myVKnots) = (myVKnots, myUKnots);
        (myUMults, myVMults) = (myVMults, myUMults);

        updateUKnots();
        updateVKnots();
    }

    /// <summary>Raises the degrees to UDegree and VDegree (no change where equal to the current degree).</summary>
    /// <exception cref="Standard_ConstructionError">A degree lower than the current one or above MaxDegree().</exception>
    internal void IncreaseDegree(int UDegree, int VDegree)
    {
        CheckMutable();
        if (UDegree != myUDeg)
        {
            if (UDegree < myUDeg || UDegree > Geom_BSplineSurface.MaxDegree())
            {
                throw new Standard_ConstructionError("Geom_BSplineSurface::IncreaseDegree: bad U degree value");
            }

            ClearEvalRepresentation();

            int FromK1 = FirstUKnotIndex();
            int ToK2 = LastUKnotIndex();

            int Step = UDegree - myUDeg;

            NCollection_Array2<gp_Pnt> npoles =
                new NCollection_Array2<gp_Pnt>(1, myPoles.ColLength() + Step * (ToK2 - FromK1), 1, myPoles.RowLength());

            int nbknots = BSplCLib.IncreaseDegreeCountKnots(myUDeg, UDegree, myUPeriodic, myUMults);

            NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, nbknots);

            NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, nbknots);

            if (myURational || myVRational)
            {
                NCollection_Array2<double> nweights =
                    new NCollection_Array2<double>(1, npoles.ColLength(), 1, npoles.RowLength());

                BSplSLib.IncreaseDegree(true,
                                        myUDeg,
                                        UDegree,
                                        myUPeriodic,
                                        myPoles,
                                        myWeights,
                                        myUKnots,
                                        myUMults,
                                        npoles,
                                        nweights,
                                        nknots,
                                        nmults);
                myWeights = nweights;
            }
            else
            {
                BSplSLib.IncreaseDegree(true,
                                        myUDeg,
                                        UDegree,
                                        myUPeriodic,
                                        myPoles,
                                        BSplSLib.NoWeights(),
                                        myUKnots,
                                        myUMults,
                                        npoles,
                                        BSplSLib.NoWeights(),
                                        nknots,
                                        nmults);
                myWeights = BSplSLib.UnitWeights(npoles.ColLength(), npoles.RowLength());
            }
            myUDeg = UDegree;
            myPoles = npoles;
            myUKnots = nknots;
            myUMults = nmults;
            updateUKnots();
        }

        if (VDegree != myVDeg)
        {
            if (VDegree < myVDeg || VDegree > Geom_BSplineSurface.MaxDegree())
            {
                throw new Standard_ConstructionError("Geom_BSplineSurface::IncreaseDegree: bad V degree value");
            }

            ClearEvalRepresentation();

            int FromK1 = FirstVKnotIndex();
            int ToK2 = LastVKnotIndex();

            int Step = VDegree - myVDeg;

            NCollection_Array2<gp_Pnt> npoles =
                new NCollection_Array2<gp_Pnt>(1, myPoles.ColLength(), 1, myPoles.RowLength() + Step * (ToK2 - FromK1));

            int nbknots = BSplCLib.IncreaseDegreeCountKnots(myVDeg, VDegree, myVPeriodic, myVMults);

            NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, nbknots);

            NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, nbknots);

            if (myURational || myVRational)
            {
                NCollection_Array2<double> nweights =
                    new NCollection_Array2<double>(1, npoles.ColLength(), 1, npoles.RowLength());

                BSplSLib.IncreaseDegree(false,
                                        myVDeg,
                                        VDegree,
                                        myVPeriodic,
                                        myPoles,
                                        myWeights,
                                        myVKnots,
                                        myVMults,
                                        npoles,
                                        nweights,
                                        nknots,
                                        nmults);
                myWeights = nweights;
            }
            else
            {
                BSplSLib.IncreaseDegree(false,
                                        myVDeg,
                                        VDegree,
                                        myVPeriodic,
                                        myPoles,
                                        BSplSLib.NoWeights(),
                                        myVKnots,
                                        myVMults,
                                        npoles,
                                        BSplSLib.NoWeights(),
                                        nknots,
                                        nmults);
                myWeights = BSplSLib.UnitWeights(npoles.ColLength(), npoles.RowLength());
            }
            myVDeg = VDegree;
            myPoles = npoles;
            myVKnots = nknots;
            myVMults = nmults;
            updateVKnots();
        }
    }

    /// <summary>Raises the multiplicity of the U knot UIndex to M.</summary>
    internal void IncreaseUMultiplicity(int UIndex, int M)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(1, 1);
        k[1] = myUKnots.Value(UIndex);
        NCollection_Array1<int> m = new NCollection_Array1<int>(1, 1);
        m[1] = M - myUMults.Value(UIndex);
        InsertUKnots(k, m, Epsilon(1.0), true);
    }

    /// <summary>Raises the multiplicities of the U knots FromI1 to ToI2 to M.</summary>
    internal void IncreaseUMultiplicity(int FromI1, int ToI2, int M)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(myUKnots, FromI1, FromI1, ToI2);
        NCollection_Array1<int> m = new NCollection_Array1<int>(FromI1, ToI2);
        for (int i = FromI1; i <= ToI2; i++)
        {
            m[i] = M - myUMults.Value(i);
        }
        InsertUKnots(k, m, Epsilon(1.0), true);
    }

    /// <summary>Raises the multiplicity of the V knot VIndex to M.</summary>
    internal void IncreaseVMultiplicity(int VIndex, int M)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(1, 1);
        k[1] = myVKnots.Value(VIndex);
        NCollection_Array1<int> m = new NCollection_Array1<int>(1, 1);
        m[1] = M - myVMults.Value(VIndex);
        InsertVKnots(k, m, Epsilon(1.0), true);
    }

    /// <summary>Raises the multiplicities of the V knots FromI1 to ToI2 to M.</summary>
    internal void IncreaseVMultiplicity(int FromI1, int ToI2, int M)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(myVKnots, FromI1, FromI1, ToI2);
        NCollection_Array1<int> m = new NCollection_Array1<int>(FromI1, ToI2);
        for (int i = FromI1; i <= ToI2; i++)
        {
            m[i] = M - myVMults.Value(i);
        }
        InsertVKnots(k, m, Epsilon(1.0), true);
    }

    /// <summary>The common part of <see cref="Segment"/> and <see cref="CheckAndSegment"/>.</summary>
    protected void segment(double U1,
                           double U2,
                           double V1,
                           double V2,
                           double EpsU,
                           double EpsV,
                           bool SegmentInU,
                           bool SegmentInV)
    {
        CheckMutable();
        ClearEvalRepresentation();
        double deltaU = U2 - U1;
        if (myUPeriodic)
        {
            double aUPeriod = myUKnots.Last() - myUKnots.First();
            if (deltaU - aUPeriod > Precision.PConfusion())
            {
                throw new Standard_DomainError("Geom_BSplineSurface::Segment");
            }
            if (deltaU > aUPeriod)
            {
                deltaU = aUPeriod;
            }
        }

        double deltaV = V2 - V1;
        if (myVPeriodic)
        {
            double aVPeriod = myVKnots.Last() - myVKnots.First();
            if (deltaV - aVPeriod > Precision.PConfusion())
            {
                throw new Standard_DomainError("Geom_BSplineSurface::Segment");
            }
            if (deltaV > aVPeriod)
            {
                deltaV = aVPeriod;
            }
        }

        double NewU1, NewU2, NewV1, NewV2;
        double U, V;
        int indexU, indexV;

        BSplCLib.LocateParameter(myUDeg, myUKnots, myUMults, U1, myUPeriodic, myUKnots.Lower(), myUKnots.Upper(),
                                 out indexU, out NewU1);
        BSplCLib.LocateParameter(myUDeg, myUKnots, myUMults, U2, myUPeriodic, myUKnots.Lower(), myUKnots.Upper(),
                                 out indexU, out NewU2);
        if (SegmentInU)
        {
            // inserting the UKnots
            NCollection_Array1<double> UKnots = new NCollection_Array1<double>(1, 2);
            NCollection_Array1<int> UMults = new NCollection_Array1<int>(1, 2);
            UKnots[1] = Min(NewU1, NewU2);
            UKnots[2] = Max(NewU1, NewU2);
            UMults[1] = UMults[2] = myUDeg;

            InsertUKnots(UKnots, UMults, EpsU);
        }

        BSplCLib.LocateParameter(myVDeg, myVKnots, myVMults, V1, myVPeriodic, myVKnots.Lower(), myVKnots.Upper(),
                                 out indexV, out NewV1);
        BSplCLib.LocateParameter(myVDeg, myVKnots, myVMults, V2, myVPeriodic, myVKnots.Lower(), myVKnots.Upper(),
                                 out indexV, out NewV2);
        if (SegmentInV)
        {
            // Inserting the VKnots
            NCollection_Array1<double> VKnots = new NCollection_Array1<double>(1, 2);
            NCollection_Array1<int> VMults = new NCollection_Array1<int>(1, 2);

            VKnots[1] = Min(NewV1, NewV2);
            VKnots[2] = Max(NewV1, NewV2);
            VMults[1] = VMults[2] = myVDeg;
            InsertVKnots(VKnots, VMults, EpsV);
        }

        if (myUPeriodic && SegmentInU)
        { // set the origine at NewU1
            int index;
            BSplCLib.LocateParameter(myUDeg, myUKnots, myUMults, U1, myUPeriodic, myUKnots.Lower(),
                                     myUKnots.Upper(), out index, out U);
            if (Math.Abs(myUKnots.Value(index + 1) - U) <= EpsU)
            {
                index++;
            }
            SetUOrigin(index);
            SetUNotPeriodic();
        }

        // compute index1 and index2 to set the new knots and mults
        int index1U, index2U;
        int FromU1 = myUKnots.Lower();
        int ToU2 = myUKnots.Upper();
        BSplCLib.LocateParameter(myUDeg, myUKnots, myUMults, NewU1, myUPeriodic, FromU1, ToU2, out index1U, out U);
        if (Math.Abs(myUKnots.Value(index1U + 1) - U) <= EpsU)
        {
            index1U++;
        }
        BSplCLib.LocateParameter(myUDeg, myUKnots, myUMults, NewU1 + deltaU, myUPeriodic, FromU1, ToU2,
                                 out index2U, out U);
        if (Math.Abs(myUKnots.Value(index2U + 1) - U) <= EpsU || index2U == index1U)
        {
            index2U++;
        }

        int nbuknots = index2U - index1U + 1;

        NCollection_Array1<double> nuknots = new NCollection_Array1<double>(1, nbuknots);
        NCollection_Array1<int> numults = new NCollection_Array1<int>(1, nbuknots);

        int i, k = 1;
        for (i = index1U; i <= index2U; i++)
        {
            nuknots.SetValue(k, myUKnots.Value(i));
            numults.SetValue(k, myUMults.Value(i));
            k++;
        }
        if (SegmentInU)
        {
            numults.SetValue(1, myUDeg + 1);
            numults.SetValue(nbuknots, myUDeg + 1);
        }

        if (myVPeriodic && SegmentInV)
        { // set the origine at NewV1
            int index;
            BSplCLib.LocateParameter(myVDeg, myVKnots, myVMults, V1, myVPeriodic, myVKnots.Lower(),
                                     myVKnots.Upper(), out index, out V);
            if (Math.Abs(myVKnots.Value(index + 1) - V) <= EpsV)
            {
                index++;
            }
            SetVOrigin(index);
            SetVNotPeriodic();
        }

        // compute index1 and index2 to set the new knots and mults
        int index1V, index2V;
        int FromV1 = myVKnots.Lower();
        int ToV2 = myVKnots.Upper();
        BSplCLib.LocateParameter(myVDeg, myVKnots, myVMults, NewV1, myVPeriodic, FromV1, ToV2, out index1V, out V);
        if (Math.Abs(myVKnots.Value(index1V + 1) - V) <= EpsV)
        {
            index1V++;
        }
        BSplCLib.LocateParameter(myVDeg, myVKnots, myVMults, NewV1 + deltaV, myVPeriodic, FromV1, ToV2,
                                 out index2V, out V);
        if (Math.Abs(myVKnots.Value(index2V + 1) - V) <= EpsV || index2V == index1V)
        {
            index2V++;
        }

        int nbvknots = index2V - index1V + 1;

        NCollection_Array1<double> nvknots = new NCollection_Array1<double>(1, nbvknots);
        NCollection_Array1<int> nvmults = new NCollection_Array1<int>(1, nbvknots);

        k = 1;
        for (i = index1V; i <= index2V; i++)
        {
            nvknots.SetValue(k, myVKnots.Value(i));
            nvmults.SetValue(k, myVMults.Value(i));
            k++;
        }
        if (SegmentInV)
        {
            nvmults.SetValue(1, myVDeg + 1);
            nvmults.SetValue(nbvknots, myVDeg + 1);
        }

        // compute index1 and index2 to set the new poles and weights
        int pindex1U = BSplCLib.PoleIndex(myUDeg, index1U, myUPeriodic, myUMults);
        int pindex2U = BSplCLib.PoleIndex(myUDeg, index2U, myUPeriodic, myUMults);

        pindex1U++;
        pindex2U = Min(pindex2U + 1, myPoles.ColLength());

        int nbupoles = pindex2U - pindex1U + 1;

        // compute index1 and index2 to set the new poles and weights
        int pindex1V = BSplCLib.PoleIndex(myVDeg, index1V, myVPeriodic, myVMults);
        int pindex2V = BSplCLib.PoleIndex(myVDeg, index2V, myVPeriodic, myVMults);

        pindex1V++;
        pindex2V = Min(pindex2V + 1, myPoles.RowLength());

        int nbvpoles = pindex2V - pindex1V + 1;

        NCollection_Array2<gp_Pnt> npoles = new NCollection_Array2<gp_Pnt>(1, nbupoles, 1, nbvpoles);

        k = 1;
        int j, l;
        if (myURational || myVRational)
        {
            NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, nbupoles, 1, nbvpoles);
            for (i = pindex1U; i <= pindex2U; i++)
            {
                l = 1;
                for (j = pindex1V; j <= pindex2V; j++)
                {
                    npoles.SetValue(k, l, myPoles.Value(i, j));
                    nweights.SetValue(k, l, myWeights.Value(i, j));
                    l++;
                }
                k++;
            }
            myWeights = nweights;
        }
        else
        {
            for (i = pindex1U; i <= pindex2U; i++)
            {
                l = 1;
                for (j = pindex1V; j <= pindex2V; j++)
                {
                    npoles.SetValue(k, l, myPoles.Value(i, j));
                    l++;
                }
                k++;
            }
            myWeights = BSplSLib.UnitWeights(nbupoles, nbvpoles);
        }

        myUKnots = nuknots;
        myUMults = numults;
        myVKnots = nvknots;
        myVMults = nvmults;
        myPoles = npoles;

        myMaxDerivInvOk = false;
        updateUKnots();
        updateVKnots();
    }

    /// <summary>
    /// Restricts the surface to the parametric rectangle [U1, U2] x [V1, V2], inserting knots of multiplicity
    /// Degree at the bounds; a periodic direction becomes non-periodic.
    /// </summary>
    /// <exception cref="Standard_DomainError">U2 &lt; U1 or V2 &lt; V1, or a segment longer than the period.</exception>
    internal void Segment(double U1,
                          double U2,
                          double V1,
                          double V2,
                          double theUTolerance = Precision_PConfusion,
                          double theVTolerance = Precision_PConfusion)
    {
        CheckMutable();
        if ((U2 < U1) || (V2 < V1))
        {
            throw new Standard_DomainError("Geom_BSplineSurface::Segment");
        }

        double aMaxU = Max(Math.Abs(U2), Math.Abs(U1));
        double EpsU = Max(Epsilon(aMaxU), theUTolerance);

        double aMaxV = Max(Math.Abs(V2), Math.Abs(V1));
        double EpsV = Max(Epsilon(aMaxV), theVTolerance);

        segment(U1, U2, V1, V2, EpsU, EpsV, true, true);
    }

    /// <summary>As <see cref="Segment"/>, but segments only the directions whose bounds change.</summary>
    internal void CheckAndSegment(double U1,
                                  double U2,
                                  double V1,
                                  double V2,
                                  double theUTolerance = Precision_PConfusion,
                                  double theVTolerance = Precision_PConfusion)
    {
        CheckMutable();
        if ((U2 < U1) || (V2 < V1))
        {
            throw new Standard_DomainError("Geom_BSplineSurface::CheckAndSegment");
        }

        double aMaxU = Max(Math.Abs(U2), Math.Abs(U1));
        double EpsU = Max(Epsilon(aMaxU), theUTolerance);

        double aMaxV = Max(Math.Abs(V2), Math.Abs(V1));
        double EpsV = Max(Epsilon(aMaxV), theVTolerance);

        bool segment_in_U = (Math.Abs(U1 - myUKnots.Value(myUKnots.Lower())) > EpsU)
                            || (Math.Abs(U2 - myUKnots.Value(myUKnots.Upper())) > EpsU);
        bool segment_in_V = (Math.Abs(V1 - myVKnots.Value(myVKnots.Lower())) > EpsV)
                            || (Math.Abs(V2 - myVKnots.Value(myVKnots.Upper())) > EpsV);

        segment(U1, U2, V1, V2, EpsU, EpsV, segment_in_U, segment_in_V);
    }

    /// <summary>Changes the value of the U knot UIndex.</summary>
    /// <exception cref="Standard_OutOfRange">UIndex out of [1, NbUKnots].</exception>
    /// <exception cref="Standard_ConstructionError">K not strictly between the neighbouring knots.</exception>
    internal void SetUKnot(int UIndex, double K)
    {
        CheckMutable();
        if (UIndex < 1 || UIndex > myUKnots.Length())
        {
            throw new Standard_OutOfRange("Geom_BSplineSurface::SetUKnot: Index and #knots mismatch");
        }

        int NewIndex = UIndex;
        double DU = Math.Abs(Epsilon(K));
        if (UIndex == 1)
        {
            if (K >= myUKnots.Value(2) - DU)
            {
                throw new Standard_ConstructionError("Geom_BSplineSurface::SetUKnot: K out of range");
            }
        }
        else if (UIndex == myUKnots.Length())
        {
            if (K <= myUKnots.Value(myUKnots.Length() - 1) + DU)
            {
                throw new Standard_ConstructionError("Geom_BSplineSurface::SetUKnot: K out of range");
            }
        }
        else
        {
            if (K <= myUKnots.Value(NewIndex - 1) + DU || K >= myUKnots.Value(NewIndex + 1) - DU)
            {
                throw new Standard_ConstructionError("Geom_BSplineSurface::SetUKnot: K out of range");
            }
        }

        if (K != myUKnots.Value(NewIndex))
        {
            ClearEvalRepresentation();
            myUKnots.SetValue(NewIndex, K);
            myMaxDerivInvOk = false;
            updateUKnots();
        }
    }

    /// <summary>Changes the U knots UK.Lower() to UK.Upper().</summary>
    internal void SetUKnots(NCollection_Array1<double> UK)
    {
        CheckMutable();
        int Lower = UK.Lower();
        int Upper = UK.Upper();
        if (Lower < 1 || Lower > myUKnots.Length() || Upper < 1 || Upper > myUKnots.Length())
        {
            throw new Standard_OutOfRange("Geom_BSplineSurface::SetUKnots: invalid array dimension");
        }
        if (Lower > 1)
        {
            if (Math.Abs(UK[Lower] - myUKnots.Value(Lower - 1)) <= gp.Resolution())
            {
                throw new Standard_ConstructionError("Geom_BSplineSurface::SetUKnots: invalid knot value");
            }
        }
        if (Upper < myUKnots.Length())
        {
            if (Math.Abs(UK[Upper] - myUKnots.Value(Upper + 1)) <= gp.Resolution())
            {
                throw new Standard_ConstructionError("Geom_BSplineSurface::SetUKnots: invalid knot value");
            }
        }
        double K1 = UK[Lower];
        ClearEvalRepresentation();
        for (int i = Lower; i <= Upper; i++)
        {
            myUKnots.SetValue(i, UK[i]);
            if (i != Lower)
            {
                if (Math.Abs(UK[i] - K1) <= gp.Resolution())
                {
                    throw new Standard_ConstructionError("Geom_BSplineSurface::SetUKnots: invalid knot value");
                }
                K1 = UK[i];
            }
        }

        myMaxDerivInvOk = false;
        updateUKnots();
    }

    /// <summary>Changes the value of the U knot UIndex and raises its multiplicity to M.</summary>
    internal void SetUKnot(int UIndex, double K, int M)
    {
        CheckMutable();
        IncreaseUMultiplicity(UIndex, M);
        SetUKnot(UIndex, K);
    }

    /// <summary>Changes the value of the V knot VIndex.</summary>
    /// <exception cref="Standard_OutOfRange">VIndex out of [1, NbVKnots].</exception>
    /// <exception cref="Standard_ConstructionError">K not strictly between the neighbouring knots.</exception>
    internal void SetVKnot(int VIndex, double K)
    {
        CheckMutable();
        if (VIndex < 1 || VIndex > myVKnots.Length())
        {
            throw new Standard_OutOfRange("Geom_BSplineSurface::SetVKnot: Index and #knots mismatch");
        }
        int NewIndex = VIndex + myVKnots.Lower() - 1;
        double DV = Math.Abs(Epsilon(K));
        if (VIndex == 1)
        {
            if (K >= myVKnots.Value(2) - DV)
            {
                throw new Standard_ConstructionError("Geom_BSplineSurface::SetVKnot: K out of range");
            }
        }
        else if (VIndex == myVKnots.Length())
        {
            if (K <= myVKnots.Value(myVKnots.Length() - 1) + DV)
            {
                throw new Standard_ConstructionError("Geom_BSplineSurface::SetVKnot: K out of range");
            }
        }
        else
        {
            if (K <= myVKnots.Value(NewIndex - 1) + DV || K >= myVKnots.Value(NewIndex + 1) - DV)
            {
                throw new Standard_ConstructionError("Geom_BSplineSurface::SetVKnot: K out of range");
            }
        }

        if (K != myVKnots.Value(NewIndex))
        {
            ClearEvalRepresentation();
            myVKnots.SetValue(NewIndex, K);
            myMaxDerivInvOk = false;
            updateVKnots();
        }
    }

    /// <summary>Changes the V knots VK.Lower() to VK.Upper().</summary>
    internal void SetVKnots(NCollection_Array1<double> VK)
    {
        CheckMutable();
        int Lower = VK.Lower();
        int Upper = VK.Upper();
        if (Lower < 1 || Lower > myVKnots.Length() || Upper < 1 || Upper > myVKnots.Length())
        {
            throw new Standard_OutOfRange("Geom_BSplineSurface::SetVKnots: invalid array dimension");
        }
        if (Lower > 1)
        {
            if (Math.Abs(VK[Lower] - myVKnots.Value(Lower - 1)) <= gp.Resolution())
            {
                throw new Standard_ConstructionError("Geom_BSplineSurface::SetVKnots: invalid knot value");
            }
        }
        if (Upper < myVKnots.Length())
        {
            if (Math.Abs(VK[Upper] - myVKnots.Value(Upper + 1)) <= gp.Resolution())
            {
                throw new Standard_ConstructionError("Geom_BSplineSurface::SetVKnots: invalid knot value");
            }
        }
        double K1 = VK[Lower];
        ClearEvalRepresentation();
        for (int i = Lower; i <= Upper; i++)
        {
            myVKnots.SetValue(i, VK[i]);
            if (i != Lower)
            {
                if (Math.Abs(VK[i] - K1) <= gp.Resolution())
                {
                    throw new Standard_ConstructionError("Geom_BSplineSurface::SetVKnots: invalid knot value");
                }
                K1 = VK[i];
            }
        }

        myMaxDerivInvOk = false;
        updateVKnots();
    }

    /// <summary>Changes the value of the V knot VIndex and raises its multiplicity to M.</summary>
    internal void SetVKnot(int VIndex, double K, int M)
    {
        CheckMutable();
        IncreaseVMultiplicity(VIndex, M);
        SetVKnot(VIndex, K);
    }

    /// <summary>Inserts the U knot U with multiplicity M (see <see cref="InsertUKnots"/>).</summary>
    internal void InsertUKnot(double U, int M, double ParametricTolerance, bool Add = true)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(1, 1);
        k[1] = U;
        NCollection_Array1<int> m = new NCollection_Array1<int>(1, 1);
        m[1] = M;
        InsertUKnots(k, m, ParametricTolerance, Add);
    }

    /// <summary>Inserts the V knot V with multiplicity M (see <see cref="InsertVKnots"/>).</summary>
    internal void InsertVKnot(double V, int M, double ParametricTolerance, bool Add = true)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(1, 1);
        k[1] = V;
        NCollection_Array1<int> m = new NCollection_Array1<int>(1, 1);
        m[1] = M;
        InsertVKnots(k, m, ParametricTolerance, Add);
    }

    /// <summary>Raises the multiplicities of the U knots FromI1 to ToI2 by Step.</summary>
    internal void IncrementUMultiplicity(int FromI1, int ToI2, int Step)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(myUKnots, FromI1, FromI1, ToI2);
        NCollection_Array1<int> m = new NCollection_Array1<int>(FromI1, ToI2);
        m.Init(Step);
        InsertUKnots(k, m, Epsilon(1.0));
    }

    /// <summary>Raises the multiplicities of the V knots FromI1 to ToI2 by Step.</summary>
    internal void IncrementVMultiplicity(int FromI1, int ToI2, int Step)
    {
        CheckMutable();
        NCollection_Array1<double> k = new NCollection_Array1<double>(myVKnots, FromI1, FromI1, ToI2);
        NCollection_Array1<int> m = new NCollection_Array1<int>(FromI1, ToI2);
        m.Init(Step);
        InsertVKnots(k, m, Epsilon(1.0));
    }

    /// <summary>Recomputes the U flat knots, the U knot distribution and the U continuity.</summary>
    protected void updateUKnots()
    {
        myMaxDerivInvOk = false;
        int MaxKnotMult;
        BSplCLib.KnotAnalysis(myUDeg, myUPeriodic, myUKnots, myUMults, out myUKnotSet, out MaxKnotMult);

        if (myUKnotSet == GeomAbs_Uniform && !myUPeriodic)
        {
            myUFlatKnots.Resize(myUKnots.Lower(), myUKnots.Upper(), false);
            myUFlatKnots.Assign(myUKnots);
        }
        else
        {
            myUFlatKnots.Resize(1, BSplCLib.KnotSequenceLength(myUMults, myUDeg, myUPeriodic), false);
            BSplCLib.KnotSequence(myUKnots, myUMults, myUDeg, myUPeriodic, myUFlatKnots);
        }

        if (MaxKnotMult == 0)
        {
            myUSmooth = GeomAbs_CN;
        }
        else
        {
            switch (myUDeg - MaxKnotMult)
            {
                case 0:
                    myUSmooth = GeomAbs_C0;
                    break;
                case 1:
                    myUSmooth = GeomAbs_C1;
                    break;
                case 2:
                    myUSmooth = GeomAbs_C2;
                    break;
                case 3:
                    myUSmooth = GeomAbs_C3;
                    break;
                default:
                    myUSmooth = GeomAbs_C3;
                    break;
            }
        }
    }

    /// <summary>Recomputes the V flat knots, the V knot distribution and the V continuity.</summary>
    protected void updateVKnots()
    {
        myMaxDerivInvOk = false;
        int MaxKnotMult;
        BSplCLib.KnotAnalysis(myVDeg, myVPeriodic, myVKnots, myVMults, out myVKnotSet, out MaxKnotMult);

        if (myVKnotSet == GeomAbs_Uniform && !myVPeriodic)
        {
            myVFlatKnots.Resize(myVKnots.Lower(), myVKnots.Upper(), false);
            myVFlatKnots.Assign(myVKnots);
        }
        else
        {
            myVFlatKnots.Resize(1, BSplCLib.KnotSequenceLength(myVMults, myVDeg, myVPeriodic), false);
            BSplCLib.KnotSequence(myVKnots, myVMults, myVDeg, myVPeriodic, myVFlatKnots);
        }

        if (MaxKnotMult == 0)
        {
            myVSmooth = GeomAbs_CN;
        }
        else
        {
            switch (myVDeg - MaxKnotMult)
            {
                case 0:
                    myVSmooth = GeomAbs_C0;
                    break;
                case 1:
                    myVSmooth = GeomAbs_C1;
                    break;
                case 2:
                    myVSmooth = GeomAbs_C2;
                    break;
                case 3:
                    myVSmooth = GeomAbs_C3;
                    break;
                default:
                    myVSmooth = GeomAbs_C3;
                    break;
            }
        }
    }

    /// <summary>
    /// Brings the parameters of a periodic direction into its period range by whole periods; the parameters of a
    /// non-periodic direction are unchanged.
    /// </summary>
    /// <exception cref="Standard_OutOfRange">The period is not greater than Epsilon of the parameter.</exception>
    public void PeriodicNormalization(ref double Uparameter, ref double Vparameter)
    {
        double Period, aMaxVal, aMinVal;

        if (myUPeriodic)
        {
            aMaxVal = myUFlatKnots.Value(myUFlatKnots.Upper() - myUDeg);
            aMinVal = myUFlatKnots.Value(myUDeg + 1);
            double eps = Math.Abs(Epsilon(Uparameter));
            Period = aMaxVal - aMinVal;

            if (Period <= eps)
            {
                throw new Standard_OutOfRange("Geom_BSplineSurface::PeriodicNormalization: Uparameter is too great number");
            }

            bool isLess, isGreater;
            isLess = aMinVal - Uparameter > 0;
            isGreater = Uparameter - aMaxVal > 0;
            if (isLess || isGreater)
            {
                double aDPar, aNbPer;
                aDPar = (isLess) ? (aMaxVal - Uparameter) : (aMinVal - Uparameter);
                aNbPer = Math.Truncate(aDPar / Period); // modf
                Uparameter += aNbPer * Period;
            }
        }
        if (myVPeriodic)
        {
            aMaxVal = myVFlatKnots.Value(myVFlatKnots.Upper() - myVDeg);
            aMinVal = myVFlatKnots.Value(myVDeg + 1);
            double eps = Math.Abs(Epsilon(Vparameter));
            Period = aMaxVal - aMinVal;

            if (Period <= eps)
            {
                throw new Standard_OutOfRange("Geom_BSplineSurface::PeriodicNormalization: Vparameter is too great number");
            }

            bool isLess, isGreater;
            isLess = aMinVal - Vparameter > 0;
            isGreater = Vparameter - aMaxVal > 0;
            if (isLess || isGreater)
            {
                double aDPar, aNbPer;
                aDPar = (isLess) ? (aMaxVal - Vparameter) : (aMinVal - Vparameter);
                aNbPer = Math.Truncate(aDPar / Period); // modf
                Vparameter += aNbPer * Period;
            }
        }
    }

    /// <summary>Changes the weight of the pole (UIndex, VIndex); the rationality flags are recomputed.</summary>
    /// <exception cref="Standard_ConstructionError">Weight not greater than gp::Resolution().</exception>
    /// <exception cref="Standard_OutOfRange">An index out of the pole grid.</exception>
    internal void SetWeight(int UIndex, int VIndex, double Weight)
    {
        CheckMutable();
        if (Weight <= gp.Resolution())
        {
            throw new Standard_ConstructionError("Geom_BSplineSurface::SetWeight: Weight too small");
        }

        if (UIndex < 1 || UIndex > myPoles.ColLength() || VIndex < 1 || VIndex > myPoles.RowLength())
        {
            throw new Standard_OutOfRange("Geom_BSplineSurface::SetWeight: Index and #pole mismatch");
        }

        ClearEvalRepresentation();
        if (!myURational && !myVRational)
        {
            // Make an owned copy of the unit weights view before modifying.
            myWeights = new NCollection_Array2<double>(myWeights);
        }
        myWeights[UIndex + myWeights.LowerRow() - 1, VIndex + myWeights.LowerCol() - 1] = Weight;
        Rational(myWeights, out myURational, out myVRational);
        if (!myURational && !myVRational)
        {
            myWeights = BSplSLib.UnitWeights(myPoles.ColLength(), myPoles.RowLength());
        }

        myMaxDerivInvOk = false;
    }

    /// <summary>Changes the weights of the column VIndex (poles CPoleWeights.Lower() to Upper() of it).</summary>
    internal void SetWeightCol(int VIndex, NCollection_Array1<double> CPoleWeights)
    {
        CheckMutable();
        if (VIndex < 1 || VIndex > myPoles.RowLength())
        {
            throw new Standard_OutOfRange("Geom_BSplineSurface::SetWeightCol: Index and #pole mismatch");
        }

        if (CPoleWeights.Lower() < 1 || CPoleWeights.Lower() > myPoles.ColLength() || CPoleWeights.Upper() < 1
            || CPoleWeights.Upper() > myPoles.ColLength())
        {
            throw new Standard_ConstructionError("Geom_BSplineSurface::SetWeightCol: invalid array dimension");
        }

        ClearEvalRepresentation();
        if (!myURational && !myVRational)
        {
            // Make an owned copy of the unit weights view before modifying.
            myWeights = new NCollection_Array2<double>(myWeights);
        }
        int I = CPoleWeights.Lower();
        while (I <= CPoleWeights.Upper())
        {
            if (CPoleWeights[I] <= gp.Resolution())
            {
                throw new Standard_ConstructionError("Geom_BSplineSurface::SetWeightCol: Weight too small");
            }
            myWeights[I + myWeights.LowerRow() - 1, VIndex + myWeights.LowerCol() - 1] = CPoleWeights[I];
            I++;
        }
        // Check if it is rational
        Rational(myWeights, out myURational, out myVRational);
        if (!myURational && !myVRational)
        {
            myWeights = BSplSLib.UnitWeights(myPoles.ColLength(), myPoles.RowLength());
        }

        myMaxDerivInvOk = false;
    }

    /// <summary>Changes the weights of the row UIndex (poles CPoleWeights.Lower() to Upper() of it).</summary>
    internal void SetWeightRow(int UIndex, NCollection_Array1<double> CPoleWeights)
    {
        CheckMutable();
        if (UIndex < 1 || UIndex > myPoles.ColLength())
        {
            throw new Standard_OutOfRange("Geom_BSplineSurface::SetWeightRow: Index and #pole mismatch");
        }

        if (CPoleWeights.Lower() < 1 || CPoleWeights.Lower() > myPoles.RowLength() || CPoleWeights.Upper() < 1
            || CPoleWeights.Upper() > myPoles.RowLength())
        {
            throw new Standard_ConstructionError("Geom_BSplineSurface::SetWeightRow: invalid array dimension");
        }

        ClearEvalRepresentation();
        if (!myURational && !myVRational)
        {
            // Make an owned copy of the unit weights view before modifying.
            myWeights = new NCollection_Array2<double>(myWeights);
        }
        int I = CPoleWeights.Lower();

        while (I <= CPoleWeights.Upper())
        {
            if (CPoleWeights[I] <= gp.Resolution())
            {
                throw new Standard_ConstructionError("Geom_BSplineSurface::SetWeightRow: Weight too small");
            }
            myWeights[UIndex + myWeights.LowerRow() - 1, I + myWeights.LowerCol() - 1] = CPoleWeights[I];
            I++;
        }
        // Check if it is rational
        Rational(myWeights, out myURational, out myVRational);
        if (!myURational && !myVRational)
        {
            myWeights = BSplSLib.UnitWeights(myPoles.ColLength(), myPoles.RowLength());
        }

        myMaxDerivInvOk = false;
    }

    //=============================================================================================
    // Geom_BSplineSurface_1.cxx
    //=============================================================================================

    /// <summary>True if the surface is at least CN in the U direction.</summary>
    /// <exception cref="Standard_RangeError">N &lt; 0.</exception>
    public sealed override bool IsCNu(int N)
    {
        Standard_RangeError_Raise_if(N < 0, " ");
        switch (myUSmooth)
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
                return N <= 3 ? true
                              : N <= myUDeg - BSplCLib.MaxKnotMult(myUMults, myUMults.Lower() + 1, myUMults.Upper() - 1);
            default:
                return false;
        }
    }

    /// <summary>True if the surface is at least CN in the V direction.</summary>
    /// <exception cref="Standard_RangeError">N &lt; 0.</exception>
    public sealed override bool IsCNv(int N)
    {
        Standard_RangeError_Raise_if(N < 0, " ");

        switch (myVSmooth)
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
                return N <= 3 ? true
                              : N <= myVDeg - BSplCLib.MaxKnotMult(myVMults, myVMults.Lower() + 1, myVMults.Upper() - 1);
            default:
                return false;
        }
    }

    /// <summary>The point of parameters (U, V); periodic parameters are normalised first.</summary>
    public sealed override gp_Pnt EvalD0(double U, double V)
    {
        gp_Pnt aEvalRepResult;
        if (GeomEval_RepUtils.TryEvalSurfaceD0(myEvalRep, U, V, out aEvalRepResult))
        {
            return aEvalRepResult;
        }

        double aNewU = U;
        double aNewV = V;
        PeriodicNormalization(ref aNewU, ref aNewV);

        gp_Pnt P;
        BSplSLib.D0(aNewU,
                    aNewV,
                    0,
                    0,
                    myPoles,
                    InternalWeights(),
                    myUFlatKnots,
                    myVFlatKnots,
                    BSplCLib.NoMults(),
                    BSplCLib.NoMults(),
                    myUDeg,
                    myVDeg,
                    myURational,
                    myVRational,
                    myUPeriodic,
                    myVPeriodic,
                    out P);
        return P;
    }

    /// <summary>The point and first derivatives at (U, V).</summary>
    public sealed override ResD1 EvalD1(double U, double V)
    {
        ResD1 aEvalRepResult;
        if (GeomEval_RepUtils.TryEvalSurfaceD1(myEvalRep, U, V, out aEvalRepResult))
        {
            return aEvalRepResult;
        }

        double aNewU = U;
        double aNewV = V;
        PeriodicNormalization(ref aNewU, ref aNewV);

        int uindex = 0, vindex = 0;

        BSplCLib.LocateParameter(myUDeg, myUKnots, myUMults, U, myUPeriodic, ref uindex, out aNewU);
        uindex = BSplCLib.FlatIndex(myUDeg, uindex, myUMults, myUPeriodic);

        BSplCLib.LocateParameter(myVDeg, myVKnots, myVMults, V, myVPeriodic, ref vindex, out aNewV);
        vindex = BSplCLib.FlatIndex(myVDeg, vindex, myVMults, myVPeriodic);

        ResD1 aResult;
        BSplSLib.D1(aNewU,
                    aNewV,
                    uindex,
                    vindex,
                    myPoles,
                    InternalWeights(),
                    myUFlatKnots,
                    myVFlatKnots,
                    BSplCLib.NoMults(),
                    BSplCLib.NoMults(),
                    myUDeg,
                    myVDeg,
                    myURational,
                    myVRational,
                    myUPeriodic,
                    myVPeriodic,
                    out aResult.Point,
                    out aResult.D1U,
                    out aResult.D1V);
        return aResult;
    }

    /// <summary>The point and derivatives up to the second order at (U, V).</summary>
    public sealed override ResD2 EvalD2(double U, double V)
    {
        ResD2 aEvalRepResult;
        if (GeomEval_RepUtils.TryEvalSurfaceD2(myEvalRep, U, V, out aEvalRepResult))
        {
            return aEvalRepResult;
        }

        double aNewU = U;
        double aNewV = V;
        PeriodicNormalization(ref aNewU, ref aNewV);

        int uindex = 0, vindex = 0;

        BSplCLib.LocateParameter(myUDeg, myUKnots, myUMults, U, myUPeriodic, ref uindex, out aNewU);
        uindex = BSplCLib.FlatIndex(myUDeg, uindex, myUMults, myUPeriodic);

        BSplCLib.LocateParameter(myVDeg, myVKnots, myVMults, V, myVPeriodic, ref vindex, out aNewV);
        vindex = BSplCLib.FlatIndex(myVDeg, vindex, myVMults, myVPeriodic);

        ResD2 aResult;
        BSplSLib.D2(aNewU,
                    aNewV,
                    uindex,
                    vindex,
                    myPoles,
                    InternalWeights(),
                    myUFlatKnots,
                    myVFlatKnots,
                    BSplCLib.NoMults(),
                    BSplCLib.NoMults(),
                    myUDeg,
                    myVDeg,
                    myURational,
                    myVRational,
                    myUPeriodic,
                    myVPeriodic,
                    out aResult.Point,
                    out aResult.D1U,
                    out aResult.D1V,
                    out aResult.D2U,
                    out aResult.D2V,
                    out aResult.D2UV);
        return aResult;
    }

    /// <summary>The point and derivatives up to the third order at (U, V).</summary>
    public sealed override ResD3 EvalD3(double U, double V)
    {
        ResD3 aEvalRepResult;
        if (GeomEval_RepUtils.TryEvalSurfaceD3(myEvalRep, U, V, out aEvalRepResult))
        {
            return aEvalRepResult;
        }

        ResD3 aResult;
        BSplSLib.D3(U,
                    V,
                    0,
                    0,
                    myPoles,
                    InternalWeights(),
                    myUFlatKnots,
                    myVFlatKnots,
                    BSplCLib.NoMults(),
                    BSplCLib.NoMults(),
                    myUDeg,
                    myVDeg,
                    myURational,
                    myVRational,
                    myUPeriodic,
                    myVPeriodic,
                    out aResult.Point,
                    out aResult.D1U,
                    out aResult.D1V,
                    out aResult.D2U,
                    out aResult.D2V,
                    out aResult.D2UV,
                    out aResult.D3U,
                    out aResult.D3V,
                    out aResult.D3UUV,
                    out aResult.D3UVV);
        return aResult;
    }

    /// <summary>The derivative of order Nu in U and Nv in V at (U, V).</summary>
    /// <exception cref="Geom_UndefinedDerivative">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    public sealed override gp_Vec EvalDN(double U, double V, int Nu, int Nv)
    {
        if (Nu + Nv < 1 || Nu < 0 || Nv < 0)
        {
            throw new Geom_UndefinedDerivative();
        }

        gp_Vec aEvalRepResult;
        if (GeomEval_RepUtils.TryEvalSurfaceDN(myEvalRep, U, V, Nu, Nv, out aEvalRepResult))
        {
            return aEvalRepResult;
        }

        gp_Vec Vn;
        BSplSLib.DN(U,
                    V,
                    Nu,
                    Nv,
                    0,
                    0,
                    myPoles,
                    InternalWeights(),
                    myUFlatKnots,
                    myVFlatKnots,
                    BSplCLib.NoMults(),
                    BSplCLib.NoMults(),
                    myUDeg,
                    myVDeg,
                    myURational,
                    myVRational,
                    myUPeriodic,
                    myVPeriodic,
                    out Vn);
        return Vn;
    }

    /// <summary>The point at (U, V), evaluated on the spans between the knots FromUK1-ToUK2 and FromVK1-ToVK2.</summary>
    public gp_Pnt LocalValue(double U, double V, int FromUK1, int ToUK2, int FromVK1, int ToVK2)
    {
        gp_Pnt P;
        LocalD0(U, V, FromUK1, ToUK2, FromVK1, ToVK2, out P);
        return P;
    }

    /// <summary>
    /// The point at (U, V), evaluated on the spans between the flat knots FromUK1 and ToUK2 in U, FromVK1 and
    /// ToVK2 in V (useful at a knot of a C0 surface).
    /// </summary>
    /// <exception cref="Standard_DomainError">FromUK1 == ToUK2 or FromVK1 == ToVK2.</exception>
    public void LocalD0(double U, double V, int FromUK1, int ToUK2, int FromVK1, int ToVK2, out gp_Pnt P)
    {
        Standard_DomainError_Raise_if(FromUK1 == ToUK2 || FromVK1 == ToVK2, "Geom_BSplineSurface::LocalD0");

        double u, v;
        int uindex, vindex;

        BSplCLib.LocateParameter(myUDeg, myUFlatKnots, U, myUPeriodic, FromUK1, ToUK2, out uindex, out u);
        uindex = BSplCLib.FlatIndex(myUDeg, uindex, myUMults, myUPeriodic);

        BSplCLib.LocateParameter(myVDeg, myVFlatKnots, V, myVPeriodic, FromVK1, ToVK2, out vindex, out v);
        vindex = BSplCLib.FlatIndex(myVDeg, vindex, myVMults, myVPeriodic);

        BSplSLib.D0(u,
                    v,
                    uindex,
                    vindex,
                    myPoles,
                    InternalWeights(),
                    myUFlatKnots,
                    myVFlatKnots,
                    BSplCLib.NoMults(),
                    BSplCLib.NoMults(),
                    myUDeg,
                    myVDeg,
                    myURational,
                    myVRational,
                    myUPeriodic,
                    myVPeriodic,
                    out P);
    }

    /// <summary>The point and first derivatives at (U, V), evaluated on the given spans (see <see cref="LocalD0"/>).</summary>
    public void LocalD1(double U,
                        double V,
                        int FromUK1,
                        int ToUK2,
                        int FromVK1,
                        int ToVK2,
                        out gp_Pnt P,
                        out gp_Vec D1U,
                        out gp_Vec D1V)
    {
        Standard_DomainError_Raise_if(FromUK1 == ToUK2 || FromVK1 == ToVK2, "Geom_BSplineSurface::LocalD1");

        double u, v;
        int uindex, vindex;

        BSplCLib.LocateParameter(myUDeg, myUFlatKnots, U, myUPeriodic, FromUK1, ToUK2, out uindex, out u);
        uindex = BSplCLib.FlatIndex(myUDeg, uindex, myUMults, myUPeriodic);

        BSplCLib.LocateParameter(myVDeg, myVFlatKnots, V, myVPeriodic, FromVK1, ToVK2, out vindex, out v);
        vindex = BSplCLib.FlatIndex(myVDeg, vindex, myVMults, myVPeriodic);

        BSplSLib.D1(u,
                    v,
                    uindex,
                    vindex,
                    myPoles,
                    InternalWeights(),
                    myUFlatKnots,
                    myVFlatKnots,
                    BSplCLib.NoMults(),
                    BSplCLib.NoMults(),
                    myUDeg,
                    myVDeg,
                    myURational,
                    myVRational,
                    myUPeriodic,
                    myVPeriodic,
                    out P,
                    out D1U,
                    out D1V);
    }

    /// <summary>The point and derivatives up to the second order, evaluated on the given spans.</summary>
    public void LocalD2(double U,
                        double V,
                        int FromUK1,
                        int ToUK2,
                        int FromVK1,
                        int ToVK2,
                        out gp_Pnt P,
                        out gp_Vec D1U,
                        out gp_Vec D1V,
                        out gp_Vec D2U,
                        out gp_Vec D2V,
                        out gp_Vec D2UV)
    {
        Standard_DomainError_Raise_if(FromUK1 == ToUK2 || FromVK1 == ToVK2, "Geom_BSplineSurface::LocalD2");

        double u, v;
        int uindex, vindex;

        BSplCLib.LocateParameter(myUDeg, myUFlatKnots, U, myUPeriodic, FromUK1, ToUK2, out uindex, out u);
        uindex = BSplCLib.FlatIndex(myUDeg, uindex, myUMults, myUPeriodic);

        BSplCLib.LocateParameter(myVDeg, myVFlatKnots, V, myVPeriodic, FromVK1, ToVK2, out vindex, out v);
        vindex = BSplCLib.FlatIndex(myVDeg, vindex, myVMults, myVPeriodic);

        BSplSLib.D2(u,
                    v,
                    uindex,
                    vindex,
                    myPoles,
                    InternalWeights(),
                    myUFlatKnots,
                    myVFlatKnots,
                    BSplCLib.NoMults(),
                    BSplCLib.NoMults(),
                    myUDeg,
                    myVDeg,
                    myURational,
                    myVRational,
                    myUPeriodic,
                    myVPeriodic,
                    out P,
                    out D1U,
                    out D1V,
                    out D2U,
                    out D2V,
                    out D2UV);
    }

    /// <summary>The point and derivatives up to the third order, evaluated on the given spans.</summary>
    public void LocalD3(double U,
                        double V,
                        int FromUK1,
                        int ToUK2,
                        int FromVK1,
                        int ToVK2,
                        out gp_Pnt P,
                        out gp_Vec D1U,
                        out gp_Vec D1V,
                        out gp_Vec D2U,
                        out gp_Vec D2V,
                        out gp_Vec D2UV,
                        out gp_Vec D3U,
                        out gp_Vec D3V,
                        out gp_Vec D3UUV,
                        out gp_Vec D3UVV)
    {
        Standard_DomainError_Raise_if(FromUK1 == ToUK2 || FromVK1 == ToVK2, "Geom_BSplineSurface::LocalD3");

        double u, v;
        int uindex, vindex;

        BSplCLib.LocateParameter(myUDeg, myUFlatKnots, U, myUPeriodic, FromUK1, ToUK2, out uindex, out u);
        uindex = BSplCLib.FlatIndex(myUDeg, uindex, myUMults, myUPeriodic);

        BSplCLib.LocateParameter(myVDeg, myVFlatKnots, V, myVPeriodic, FromVK1, ToVK2, out vindex, out v);
        vindex = BSplCLib.FlatIndex(myVDeg, vindex, myVMults, myVPeriodic);

        BSplSLib.D3(u,
                    v,
                    uindex,
                    vindex,
                    myPoles,
                    InternalWeights(),
                    myUFlatKnots,
                    myVFlatKnots,
                    BSplCLib.NoMults(),
                    BSplCLib.NoMults(),
                    myUDeg,
                    myVDeg,
                    myURational,
                    myVRational,
                    myUPeriodic,
                    myVPeriodic,
                    out P,
                    out D1U,
                    out D1V,
                    out D2U,
                    out D2V,
                    out D2UV,
                    out D3U,
                    out D3V,
                    out D3UUV,
                    out D3UVV);
    }

    /// <summary>The derivative of order Nu in U and Nv in V, evaluated on the given spans.</summary>
    public gp_Vec LocalDN(double U, double V, int FromUK1, int ToUK2, int FromVK1, int ToVK2, int Nu, int Nv)
    {
        Standard_DomainError_Raise_if(FromUK1 == ToUK2 || FromVK1 == ToVK2, "Geom_BSplineSurface::LocalDN");

        double u, v;
        int uindex, vindex;

        BSplCLib.LocateParameter(myUDeg, myUFlatKnots, U, myUPeriodic, FromUK1, ToUK2, out uindex, out u);
        uindex = BSplCLib.FlatIndex(myUDeg, uindex, myUMults, myUPeriodic);

        BSplCLib.LocateParameter(myVDeg, myVFlatKnots, V, myVPeriodic, FromVK1, ToVK2, out vindex, out v);
        vindex = BSplCLib.FlatIndex(myVDeg, vindex, myVMults, myVPeriodic);

        gp_Vec Vn;
        BSplSLib.DN(u,
                    v,
                    Nu,
                    Nv,
                    uindex,
                    vindex,
                    myPoles,
                    InternalWeights(),
                    myUFlatKnots,
                    myVFlatKnots,
                    BSplCLib.NoMults(),
                    BSplCLib.NoMults(),
                    myUDeg,
                    myVDeg,
                    myURational,
                    myVRational,
                    myUPeriodic,
                    myVPeriodic,
                    out Vn);
        return Vn;
    }

    /// <summary>The pole (UIndex, VIndex).</summary>
    /// <exception cref="Standard_OutOfRange">An index out of the pole grid.</exception>
    public gp_Pnt Pole(int UIndex, int VIndex)
    {
        Standard_OutOfRange_Raise_if(UIndex < 1 || UIndex > myPoles.ColLength() || VIndex < 1
                                         || VIndex > myPoles.RowLength(),
                                     " ");
        return myPoles.Value(UIndex, VIndex);
    }

    /// <summary>Copies the poles into P (deprecated in OCCT: use <see cref="Poles()"/>).</summary>
    /// <exception cref="Standard_DimensionError">P is not NbUPoles x NbVPoles.</exception>
    public void Poles(NCollection_Array2<gp_Pnt> P)
    {
        Standard_DimensionError_Raise_if(P.ColLength() != myPoles.ColLength() || P.RowLength() != myPoles.RowLength(),
                                         " ");
        P.Assign(myPoles);
    }

    /// <summary>A copy of the poles, NbUPoles rows by NbVPoles columns, indexed from 1.</summary>
    public NCollection_Array2<gp_Pnt> Poles() => new NCollection_Array2<gp_Pnt>(myPoles);

    /// <summary>The U iso-curve (a B-spline curve in V) at U.</summary>
    /// <remarks>Not available yet: needs the port of Geom_BSplineCurve (unit 1A). Throws Standard_NotImplemented.</remarks>
    public sealed override Geom_Curve UIso(double U)
    {
        NCollection_Array1<gp_Pnt> cpoles = new NCollection_Array1<gp_Pnt>(1, myPoles.RowLength());
        NCollection_Array1<double> cweights = new NCollection_Array1<double>(1, myPoles.RowLength());

        if (myURational || myVRational)
        {
            BSplSLib.Iso(U, true, myPoles, InternalWeights(), myUFlatKnots, BSplCLib.NoMults(), myUDeg, myUPeriodic,
                         cpoles, cweights);
            // TODO(1A): C = new Geom_BSplineCurve(cpoles, cweights, myVKnots, myVMults, myVDeg, myVPeriodic);
        }
        else
        {
            BSplSLib.Iso(U, true, myPoles, BSplSLib.NoWeights(), myUFlatKnots, BSplCLib.NoMults(), myUDeg,
                         myUPeriodic, cpoles, cweights);
            // TODO(1A): C = new Geom_BSplineCurve(cpoles, myVKnots, myVMults, myVDeg, myVPeriodic);
        }
        throw new Standard_NotImplemented("Geom_BSplineSurface::UIso: Geom_BSplineCurve is not ported yet");
    }

    /// <summary>
    /// The U iso-curve at U; with CheckRational, the curve is non-rational when its weights are all equal.
    /// </summary>
    /// <remarks>Not available yet: needs the port of Geom_BSplineCurve (unit 1A). Throws Standard_NotImplemented.</remarks>
    public Geom_Curve UIso(double U, bool CheckRational)
    {
        NCollection_Array1<gp_Pnt> cpoles = new NCollection_Array1<gp_Pnt>(1, myPoles.RowLength());
        NCollection_Array1<double> cweights = new NCollection_Array1<double>(1, myPoles.RowLength());

        if (myURational || myVRational)
        {
            BSplSLib.Iso(U, true, myPoles, InternalWeights(), myUFlatKnots, BSplCLib.NoMults(), myUDeg, myUPeriodic,
                         cpoles, cweights);
            // TODO(1A): C = new Geom_BSplineCurve(cpoles, cweights, myVKnots, myVMults, myVDeg, myVPeriodic,
            //                                     CheckRational);
        }
        else
        {
            BSplSLib.Iso(U, true, myPoles, BSplSLib.NoWeights(), myUFlatKnots, BSplCLib.NoMults(), myUDeg,
                         myUPeriodic, cpoles, cweights);
            // TODO(1A): C = new Geom_BSplineCurve(cpoles, myVKnots, myVMults, myVDeg, myVPeriodic);
        }
        _ = CheckRational;
        throw new Standard_NotImplemented("Geom_BSplineSurface::UIso: Geom_BSplineCurve is not ported yet");
    }

    /// <summary>The value of the U knot UIndex.</summary>
    /// <exception cref="Standard_OutOfRange">UIndex out of [1, NbUKnots].</exception>
    public double UKnot(int UIndex)
    {
        Standard_OutOfRange_Raise_if(UIndex < 1 || UIndex > myUKnots.Length(), " ");
        return myUKnots.Value(UIndex);
    }

    /// <summary>The value of the V knot VIndex.</summary>
    /// <exception cref="Standard_OutOfRange">VIndex out of [1, NbVKnots].</exception>
    public double VKnot(int VIndex)
    {
        Standard_OutOfRange_Raise_if(VIndex < 1 || VIndex > myVKnots.Length(), " ");
        return myVKnots.Value(VIndex);
    }

    /// <summary>Copies the U knots into Ku (deprecated in OCCT: use <see cref="UKnots()"/>).</summary>
    public void UKnots(NCollection_Array1<double> Ku)
    {
        Standard_DimensionError_Raise_if(Ku.Length() != myUKnots.Length(), " ");
        Ku.Assign(myUKnots);
    }

    /// <summary>The U knots (without repetition).</summary>
    public PackedList<double> UKnots() => myUKnots.ToPackedList();

    /// <summary>Copies the V knots into Kv (deprecated in OCCT: use <see cref="VKnots()"/>).</summary>
    public void VKnots(NCollection_Array1<double> Kv)
    {
        Standard_DimensionError_Raise_if(Kv.Length() != myVKnots.Length(), " ");
        Kv.Assign(myVKnots);
    }

    /// <summary>The V knots (without repetition).</summary>
    public PackedList<double> VKnots() => myVKnots.ToPackedList();

    /// <summary>Copies the U flat knots into Ku (deprecated in OCCT: use <see cref="UKnotSequence()"/>).</summary>
    public void UKnotSequence(NCollection_Array1<double> Ku)
    {
        Standard_DimensionError_Raise_if(Ku.Length() != myUFlatKnots.Length(), " ");
        Ku.Assign(myUFlatKnots);
    }

    /// <summary>The U knots repeated by their multiplicities (the flat knots).</summary>
    public PackedList<double> UKnotSequence() => myUFlatKnots.ToPackedList();

    /// <summary>Copies the V flat knots into Kv (deprecated in OCCT: use <see cref="VKnotSequence()"/>).</summary>
    public void VKnotSequence(NCollection_Array1<double> Kv)
    {
        Standard_DimensionError_Raise_if(Kv.Length() != myVFlatKnots.Length(), " ");
        Kv.Assign(myVFlatKnots);
    }

    /// <summary>The V knots repeated by their multiplicities (the flat knots).</summary>
    public PackedList<double> VKnotSequence() => myVFlatKnots.ToPackedList();

    /// <summary>The multiplicity of the U knot UIndex.</summary>
    public int UMultiplicity(int UIndex)
    {
        Standard_OutOfRange_Raise_if(UIndex < 1 || UIndex > myUMults.Length(), " ");
        return myUMults.Value(UIndex);
    }

    /// <summary>Copies the U multiplicities into Mu (deprecated in OCCT: use <see cref="UMultiplicities()"/>).</summary>
    public void UMultiplicities(NCollection_Array1<int> Mu)
    {
        Standard_DimensionError_Raise_if(Mu.Length() != myUMults.Length(), " ");
        Mu.Assign(myUMults);
    }

    /// <summary>The multiplicities of the U knots.</summary>
    public PackedList<int> UMultiplicities() => myUMults.ToPackedList();

    /// <summary>The V iso-curve (a B-spline curve in U) at V.</summary>
    /// <remarks>Not available yet: needs the port of Geom_BSplineCurve (unit 1A). Throws Standard_NotImplemented.</remarks>
    public sealed override Geom_Curve VIso(double V)
    {
        NCollection_Array1<gp_Pnt> cpoles = new NCollection_Array1<gp_Pnt>(1, myPoles.ColLength());
        NCollection_Array1<double> cweights = new NCollection_Array1<double>(1, myPoles.ColLength());

        if (myURational || myVRational)
        {
            BSplSLib.Iso(V, false, myPoles, InternalWeights(), myVFlatKnots, BSplCLib.NoMults(), myVDeg, myVPeriodic,
                         cpoles, cweights);
            // TODO(1A): C = new Geom_BSplineCurve(cpoles, cweights, myUKnots, myUMults, myUDeg, myUPeriodic);
        }
        else
        {
            BSplSLib.Iso(V, false, myPoles, BSplSLib.NoWeights(), myVFlatKnots, BSplCLib.NoMults(), myVDeg,
                         myVPeriodic, cpoles, cweights);
            // TODO(1A): C = new Geom_BSplineCurve(cpoles, myUKnots, myUMults, myUDeg, myUPeriodic);
        }
        throw new Standard_NotImplemented("Geom_BSplineSurface::VIso: Geom_BSplineCurve is not ported yet");
    }

    /// <summary>
    /// The V iso-curve at V; with CheckRational, the curve is non-rational when its weights are all equal.
    /// </summary>
    /// <remarks>Not available yet: needs the port of Geom_BSplineCurve (unit 1A). Throws Standard_NotImplemented.</remarks>
    public Geom_Curve VIso(double V, bool CheckRational)
    {
        NCollection_Array1<gp_Pnt> cpoles = new NCollection_Array1<gp_Pnt>(1, myPoles.ColLength());
        NCollection_Array1<double> cweights = new NCollection_Array1<double>(1, myPoles.ColLength());

        if (myURational || myVRational)
        {
            BSplSLib.Iso(V, false, myPoles, InternalWeights(), myVFlatKnots, BSplCLib.NoMults(), myVDeg, myVPeriodic,
                         cpoles, cweights);
            // TODO(1A): C = new Geom_BSplineCurve(cpoles, cweights, myUKnots, myUMults, myUDeg, myUPeriodic,
            //                                     CheckRational);
        }
        else
        {
            BSplSLib.Iso(V, false, myPoles, BSplSLib.NoWeights(), myVFlatKnots, BSplCLib.NoMults(), myVDeg,
                         myVPeriodic, cpoles, cweights);
            // TODO(1A): C = new Geom_BSplineCurve(cpoles, myUKnots, myUMults, myUDeg, myUPeriodic);
        }
        _ = CheckRational;
        throw new Standard_NotImplemented("Geom_BSplineSurface::VIso: Geom_BSplineCurve is not ported yet");
    }

    /// <summary>The multiplicity of the V knot VIndex.</summary>
    public int VMultiplicity(int VIndex)
    {
        Standard_OutOfRange_Raise_if(VIndex < 1 || VIndex > myVMults.Length(), " ");
        return myVMults.Value(VIndex);
    }

    /// <summary>Copies the V multiplicities into Mv (deprecated in OCCT: use <see cref="VMultiplicities()"/>).</summary>
    public void VMultiplicities(NCollection_Array1<int> Mv)
    {
        Standard_DimensionError_Raise_if(Mv.Length() != myVMults.Length(), " ");
        Mv.Assign(myVMults);
    }

    /// <summary>The multiplicities of the V knots.</summary>
    public PackedList<int> VMultiplicities() => myVMults.ToPackedList();

    /// <summary>The weight of the pole (UIndex, VIndex): 1 for a non-rational surface.</summary>
    public double Weight(int UIndex, int VIndex)
    {
        if (!(myURational || myVRational))
        {
            return 1.0;
        }
        Standard_OutOfRange_Raise_if(UIndex < 1 || UIndex > myWeights.ColLength() || VIndex < 1
                                         || VIndex > myWeights.RowLength(),
                                     " ");
        return myWeights.Value(UIndex, VIndex);
    }

    /// <summary>Copies the weights into W, all 1 for a non-rational surface (deprecated in OCCT).</summary>
    /// <exception cref="Standard_DimensionError">W is not NbUPoles x NbVPoles.</exception>
    public void Weights(NCollection_Array2<double> W)
    {
        if (!(myURational || myVRational))
        {
            Standard_DimensionError_Raise_if(W.ColLength() != myPoles.ColLength()
                                                 || W.RowLength() != myPoles.RowLength(),
                                             " ");
            W.Init(1.0);
            return;
        }
        Standard_DimensionError_Raise_if(W.ColLength() != myWeights.ColLength() || W.RowLength() != myWeights.RowLength(),
                                         " ");
        W.Assign(myWeights);
    }

    /// <summary>A copy of the weights, or null if the surface is non-rational in both directions.</summary>
    public NCollection_Array2<double>? Weights()
    {
        if (myURational || myVRational)
        {
            return new NCollection_Array2<double>(myWeights);
        }
        return null;
    }

    /// <summary>A copy of the weights, all 1 for a non-rational surface.</summary>
    public NCollection_Array2<double> WeightsArray() => new NCollection_Array2<double>(myWeights);

    /// <summary>Transforms the poles by T.</summary>
    protected internal sealed override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        ClearEvalRepresentation();
        for (int j = myPoles.LowerCol(); j <= myPoles.UpperCol(); j++)
        {
            for (int i = myPoles.LowerRow(); i <= myPoles.UpperRow(); i++)
            {
                myPoles[i, j].Transform(T);
            }
        }
        myMaxDerivInvOk = false;
    }

    /// <summary>
    /// Makes the surface periodic in U: the knots outside [FirstUKnotIndex, LastUKnotIndex] are dropped and the
    /// poles trimmed to the periodic count.
    /// </summary>
    internal void SetUPeriodic()
    {
        CheckMutable();
        ClearEvalRepresentation();
        int first = FirstUKnotIndex();
        int last = LastUKnotIndex();

        NCollection_Array1<double> cknots = new NCollection_Array1<double>(1, last - first + 1);
        for (int k = first; k <= last; k++)
        {
            cknots[k - first + 1] = myUKnots[k];
        }
        myUKnots = cknots;

        NCollection_Array1<int> cmults = new NCollection_Array1<int>(1, last - first + 1);
        for (int k = first; k <= last; k++)
        {
            cmults[k - first + 1] = myUMults[k];
        }
        cmults[1] = cmults[cmults.Upper()] = Min(myUDeg, Max(cmults[1], cmults[cmults.Upper()]));
        myUMults = cmults;

        // compute new number of poles;
        int nbp = BSplCLib.NbPoles(myUDeg, true, myUMults);

        myPoles.ResizeWithTrim(1, nbp, myPoles.LowerCol(), myPoles.UpperCol(), true);
        if (myURational || myVRational)
        {
            myWeights.ResizeWithTrim(1, nbp, myWeights.LowerCol(), myWeights.UpperCol(), true);
        }
        else
        {
            myWeights = BSplSLib.UnitWeights(nbp, myPoles.RowLength());
        }

        myUPeriodic = true;

        myMaxDerivInvOk = false;
        updateUKnots();
    }

    /// <summary>Makes the surface periodic in V (see <see cref="SetUPeriodic"/>).</summary>
    internal void SetVPeriodic()
    {
        CheckMutable();
        ClearEvalRepresentation();
        int first = FirstVKnotIndex();
        int last = LastVKnotIndex();

        NCollection_Array1<double> cknots = new NCollection_Array1<double>(1, last - first + 1);
        for (int k = first; k <= last; k++)
        {
            cknots[k - first + 1] = myVKnots[k];
        }
        myVKnots = cknots;

        NCollection_Array1<int> cmults = new NCollection_Array1<int>(1, last - first + 1);
        for (int k = first; k <= last; k++)
        {
            cmults[k - first + 1] = myVMults[k];
        }
        cmults[1] = cmults[cmults.Upper()] = Min(myVDeg, Max(cmults[1], cmults[cmults.Upper()]));
        myVMults = cmults;

        // compute new number of poles;
        int nbp = BSplCLib.NbPoles(myVDeg, true, myVMults);

        myPoles.ResizeWithTrim(myPoles.LowerRow(), myPoles.UpperRow(), 1, nbp, true);
        if (myURational || myVRational)
        {
            myWeights.ResizeWithTrim(myWeights.LowerRow(), myWeights.UpperRow(), 1, nbp, true);
        }
        else
        {
            myWeights = BSplSLib.UnitWeights(myPoles.ColLength(), nbp);
        }

        myVPeriodic = true;

        myMaxDerivInvOk = false;
        updateVKnots();
    }

    /// <summary>Makes the U knot Index the origin of a U-periodic surface.</summary>
    /// <exception cref="Standard_NoSuchObject">The surface is not U-periodic.</exception>
    /// <exception cref="Standard_DomainError">Index out of [FirstUKnotIndex, LastUKnotIndex].</exception>
    internal void SetUOrigin(int Index)
    {
        CheckMutable();
        if (!myUPeriodic)
        {
            throw new Standard_NoSuchObject("Geom_BSplineSurface::SetUOrigin: surface is not U periodic");
        }

        ClearEvalRepresentation();
        int i, j, k;
        int first = FirstUKnotIndex();
        int last = LastUKnotIndex();

        if ((Index < first) || (Index > last))
        {
            throw new Standard_DomainError("Geom_BSplineCurve::SetUOrigin: Index out of range");
        }

        int nbknots = myUKnots.Length();
        int nbpoles = myPoles.ColLength();

        NCollection_Array1<double> newknots = new NCollection_Array1<double>(1, nbknots);

        NCollection_Array1<int> newmults = new NCollection_Array1<int>(1, nbknots);

        // set the knots and mults
        double period = myUKnots.Value(last) - myUKnots.Value(first);
        k = 1;
        for (i = Index; i <= last; i++)
        {
            newknots[k] = myUKnots.Value(i);
            newmults[k] = myUMults.Value(i);
            k++;
        }
        for (i = first + 1; i <= Index; i++)
        {
            newknots[k] = myUKnots.Value(i) + period;
            newmults[k] = myUMults.Value(i);
            k++;
        }

        int index = 1;
        for (i = first + 1; i <= Index; i++)
        {
            index += myUMults.Value(i);
        }

        // set the poles and weights
        int nbvp = myPoles.RowLength();
        NCollection_Array2<gp_Pnt> newpoles = new NCollection_Array2<gp_Pnt>(1, nbpoles, 1, nbvp);
        first = myPoles.LowerRow();
        last = myPoles.UpperRow();
        if (myURational || myVRational)
        {
            NCollection_Array2<double> newweights = new NCollection_Array2<double>(1, nbpoles, 1, nbvp);
            k = 1;
            for (i = index; i <= last; i++)
            {
                for (j = 1; j <= nbvp; j++)
                {
                    newpoles[k, j] = myPoles.Value(i, j);
                    newweights[k, j] = myWeights.Value(i, j);
                }
                k++;
            }
            for (i = first; i < index; i++)
            {
                for (j = 1; j <= nbvp; j++)
                {
                    newpoles[k, j] = myPoles.Value(i, j);
                    newweights[k, j] = myWeights.Value(i, j);
                }
                k++;
            }
            myWeights = newweights;
        }
        else
        {
            k = 1;
            for (i = index; i <= last; i++)
            {
                for (j = 1; j <= nbvp; j++)
                {
                    newpoles[k, j] = myPoles.Value(i, j);
                }
                k++;
            }
            for (i = first; i < index; i++)
            {
                for (j = 1; j <= nbvp; j++)
                {
                    newpoles[k, j] = myPoles.Value(i, j);
                }
                k++;
            }
            myWeights = BSplSLib.UnitWeights(nbpoles, nbvp);
        }

        myPoles = newpoles;
        myUKnots = newknots;
        myUMults = newmults;
        updateUKnots();
    }

    /// <summary>Makes the V knot Index the origin of a V-periodic surface.</summary>
    /// <exception cref="Standard_NoSuchObject">The surface is not V-periodic.</exception>
    /// <exception cref="Standard_DomainError">Index out of [FirstVKnotIndex, LastVKnotIndex].</exception>
    internal void SetVOrigin(int Index)
    {
        CheckMutable();
        if (!myVPeriodic)
        {
            throw new Standard_NoSuchObject("Geom_BSplineSurface::SetVOrigin: surface is not V periodic");
        }

        ClearEvalRepresentation();
        int i, j, k;
        int first = FirstVKnotIndex();
        int last = LastVKnotIndex();

        if ((Index < first) || (Index > last))
        {
            throw new Standard_DomainError("Geom_BSplineCurve::SetVOrigin: Index out of range");
        }

        int nbknots = myVKnots.Length();
        int nbpoles = myPoles.RowLength();

        NCollection_Array1<double> newknots = new NCollection_Array1<double>(1, nbknots);

        NCollection_Array1<int> newmults = new NCollection_Array1<int>(1, nbknots);

        // set the knots and mults
        double period = myVKnots.Value(last) - myVKnots.Value(first);
        k = 1;
        for (i = Index; i <= last; i++)
        {
            newknots[k] = myVKnots.Value(i);
            newmults[k] = myVMults.Value(i);
            k++;
        }
        for (i = first + 1; i <= Index; i++)
        {
            newknots[k] = myVKnots.Value(i) + period;
            newmults[k] = myVMults.Value(i);
            k++;
        }

        int index = 1;
        for (i = first + 1; i <= Index; i++)
        {
            index += myVMults.Value(i);
        }

        // set the poles and weights
        int nbup = myPoles.ColLength();
        NCollection_Array2<gp_Pnt> newpoles = new NCollection_Array2<gp_Pnt>(1, nbup, 1, nbpoles);
        first = myPoles.LowerCol();
        last = myPoles.UpperCol();
        if (myURational || myVRational)
        {
            NCollection_Array2<double> newweights = new NCollection_Array2<double>(1, nbup, 1, nbpoles);
            k = 1;
            for (j = index; j <= last; j++)
            {
                for (i = 1; i <= nbup; i++)
                {
                    newpoles[i, k] = myPoles.Value(i, j);
                    newweights[i, k] = myWeights.Value(i, j);
                }
                k++;
            }
            for (j = first; j < index; j++)
            {
                for (i = 1; i <= nbup; i++)
                {
                    newpoles[i, k] = myPoles.Value(i, j);
                    newweights[i, k] = myWeights.Value(i, j);
                }
                k++;
            }
            myWeights = newweights;
        }
        else
        {
            k = 1;
            for (j = index; j <= last; j++)
            {
                for (i = 1; i <= nbup; i++)
                {
                    newpoles[i, k] = myPoles.Value(i, j);
                }
                k++;
            }
            for (j = first; j < index; j++)
            {
                for (i = 1; i <= nbup; i++)
                {
                    newpoles[i, k] = myPoles.Value(i, j);
                }
                k++;
            }
            myWeights = BSplSLib.UnitWeights(nbup, nbpoles);
        }

        myPoles = newpoles;
        myVKnots = newknots;
        myVMults = newmults;
        updateVKnots();
    }

    /// <summary>Makes a U-periodic surface non-periodic (the same geometry); does nothing otherwise.</summary>
    internal void SetUNotPeriodic()
    {
        CheckMutable();
        if (myUPeriodic)
        {
            ClearEvalRepresentation();
            int NbKnots, NbPoles;
            BSplCLib.PrepareUnperiodize(myUDeg, myUMults, out NbKnots, out NbPoles);

            NCollection_Array2<gp_Pnt> npoles = new NCollection_Array2<gp_Pnt>(1, NbPoles, 1, myPoles.RowLength());

            NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, NbKnots);

            NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, NbKnots);

            if (myURational || myVRational)
            {
                NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, NbPoles, 1, myPoles.RowLength());
                nweights.Init(0.0);

                BSplSLib.Unperiodize(true,
                                     myUDeg,
                                     myUMults,
                                     myUKnots,
                                     myPoles,
                                     InternalWeights(),
                                     nmults,
                                     nknots,
                                     npoles,
                                     nweights);
                myWeights = nweights;
            }
            else
            {
                BSplSLib.Unperiodize(true,
                                     myUDeg,
                                     myUMults,
                                     myUKnots,
                                     myPoles,
                                     BSplSLib.NoWeights(),
                                     nmults,
                                     nknots,
                                     npoles,
                                     BSplSLib.NoWeights());
                myWeights = BSplSLib.UnitWeights(npoles.ColLength(), npoles.RowLength());
            }
            myPoles = npoles;
            myUMults = nmults;
            myUKnots = nknots;
            myUPeriodic = false;

            myMaxDerivInvOk = false;
            updateUKnots();
        }
    }

    /// <summary>Makes a V-periodic surface non-periodic (the same geometry); does nothing otherwise.</summary>
    internal void SetVNotPeriodic()
    {
        CheckMutable();
        if (myVPeriodic)
        {
            ClearEvalRepresentation();
            int NbKnots, NbPoles;
            BSplCLib.PrepareUnperiodize(myVDeg, myVMults, out NbKnots, out NbPoles);

            NCollection_Array2<gp_Pnt> npoles = new NCollection_Array2<gp_Pnt>(1, myPoles.ColLength(), 1, NbPoles);

            NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, NbKnots);

            NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, NbKnots);

            if (myURational || myVRational)
            {
                NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, myPoles.ColLength(), 1, NbPoles);
                nweights.Init(0.0);

                BSplSLib.Unperiodize(false,
                                     myVDeg,
                                     myVMults,
                                     myVKnots,
                                     myPoles,
                                     InternalWeights(),
                                     nmults,
                                     nknots,
                                     npoles,
                                     nweights);
                myWeights = nweights;
            }
            else
            {
                BSplSLib.Unperiodize(false,
                                     myVDeg,
                                     myVMults,
                                     myVKnots,
                                     myPoles,
                                     BSplSLib.NoWeights(),
                                     nmults,
                                     nknots,
                                     npoles,
                                     BSplSLib.NoWeights());
                myWeights = BSplSLib.UnitWeights(npoles.ColLength(), npoles.RowLength());
            }
            myPoles = npoles;
            myVMults = nmults;
            myVKnots = nknots;
            myVPeriodic = false;

            myMaxDerivInvOk = false;
            updateVKnots();
        }
    }

    /// <summary>
    /// True if the surface is closed in U: periodic, or with equal first and last U iso-curves (within
    /// Precision::Confusion()).
    /// </summary>
    public sealed override bool IsUClosed()
    {
        if (myUPeriodic)
        {
            return true;
        }

        double aU1, aU2, aV1, aV2;
        Bounds(out aU1, out aU2, out aV1, out aV2);
        // TODO(1A): OCCT compares UIso(aU1) and UIso(aU2) with Geom_BSplineCurve::IsEqual(.., Confusion()).
        // Until Geom_BSplineCurve is ported, IsoCurvesEqual does the same on the iso-curves' data.
        return IsoCurvesEqual(true, aU1, aU2);
    }

    /// <summary>
    /// True if the surface is closed in V: periodic, or with equal first and last V iso-curves (within
    /// Precision::Confusion()).
    /// </summary>
    public sealed override bool IsVClosed()
    {
        if (myVPeriodic)
        {
            return true;
        }

        double aU1, aU2, aV1, aV2;
        Bounds(out aU1, out aU2, out aV1, out aV2);
        // TODO(1A): OCCT compares VIso(aV1) and VIso(aV2) with Geom_BSplineCurve::IsEqual(.., Confusion()).
        return IsoCurvesEqual(false, aV1, aV2);
    }

    /// <summary>
    /// What <c>UIso(P1)->IsEqual(UIso(P2), Precision::Confusion())</c> (or VIso) computes in OCCT, on the
    /// iso-curves' poles and weights: both curves share their knots, multiplicities and degree, and are rational
    /// when the surface is and their weights are not all equal (Geom_BSplineCurve's constructor check).
    /// </summary>
    private bool IsoCurvesEqual(bool theIsU, double theP1, double theP2)
    {
        int aNb = theIsU ? myPoles.RowLength() : myPoles.ColLength();
        NCollection_Array1<double> aFlatKnots = theIsU ? myUFlatKnots : myVFlatKnots;
        int aDeg = theIsU ? myUDeg : myVDeg;
        bool aPer = theIsU ? myUPeriodic : myVPeriodic;
        bool aRat = myURational || myVRational;
        NCollection_Array1<gp_Pnt> aPoles1 = new NCollection_Array1<gp_Pnt>(1, aNb);
        NCollection_Array1<gp_Pnt> aPoles2 = new NCollection_Array1<gp_Pnt>(1, aNb);
        NCollection_Array1<double> aWeights1 = new NCollection_Array1<double>(1, aNb);
        NCollection_Array1<double> aWeights2 = new NCollection_Array1<double>(1, aNb);
        NCollection_Array2<double>? aWeights = aRat ? InternalWeights() : BSplSLib.NoWeights();
        BSplSLib.Iso(theP1, theIsU, myPoles, aWeights, aFlatKnots, BSplCLib.NoMults(), aDeg, aPer, aPoles1, aWeights1);
        BSplSLib.Iso(theP2, theIsU, myPoles, aWeights, aFlatKnots, BSplCLib.NoMults(), aDeg, aPer, aPoles2, aWeights2);

        // Geom_BSplineCurve's constructor: rational only if the weights differ by more than gp::Resolution()
        bool aRat1 = aRat && IsoWeightsRational(aWeights1);
        bool aRat2 = aRat && IsoWeightsRational(aWeights2);
        if (aRat1 != aRat2)
        {
            return false;
        }

        double aPreci = Precision.Confusion();
        for (int aPoleIter = 1; aPoleIter <= aNb; ++aPoleIter)
        {
            gp_Pnt aPole1 = aPoles1[aPoleIter];
            gp_Pnt aPole2 = aPoles2[aPoleIter];
            if (Math.Abs(aPole1.X() - aPole2.X()) > aPreci || Math.Abs(aPole1.Y() - aPole2.Y()) > aPreci
                || Math.Abs(aPole1.Z() - aPole2.Z()) > aPreci)
            {
                return false;
            }
        }

        if (!aRat1)
        {
            return true;
        }

        for (int aWeightIter = 1; aWeightIter <= aNb; ++aWeightIter)
        {
            if (Math.Abs(aWeights1[aWeightIter] - aWeights2[aWeightIter]) > Epsilon(aWeights1[aWeightIter]))
            {
                return false;
            }
        }
        return true;
    }

    private static bool IsoWeightsRational(NCollection_Array1<double> theWeights)
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

    /// <summary>True if the surface is periodic in U.</summary>
    public sealed override bool IsUPeriodic() => myUPeriodic;

    /// <summary>True if the surface is periodic in V.</summary>
    public sealed override bool IsVPeriodic() => myVPeriodic;

    /// <summary>The index of the first U knot of the parametric range (1 when periodic).</summary>
    public int FirstUKnotIndex()
    {
        if (myUPeriodic)
        {
            return 1;
        }
        else
        {
            return BSplCLib.FirstUKnotIndex(myUDeg, myUMults);
        }
    }

    /// <summary>The index of the first V knot of the parametric range (1 when periodic).</summary>
    public int FirstVKnotIndex()
    {
        if (myVPeriodic)
        {
            return 1;
        }
        else
        {
            return BSplCLib.FirstUKnotIndex(myVDeg, myVMults);
        }
    }

    /// <summary>The index of the last U knot of the parametric range (NbUKnots when periodic).</summary>
    public int LastUKnotIndex()
    {
        if (myUPeriodic)
        {
            return myUKnots.Length();
        }
        else
        {
            return BSplCLib.LastUKnotIndex(myUDeg, myUMults);
        }
    }

    /// <summary>The index of the last V knot of the parametric range (NbVKnots when periodic).</summary>
    public int LastVKnotIndex()
    {
        if (myVPeriodic)
        {
            return myVKnots.Length();
        }
        else
        {
            return BSplCLib.LastUKnotIndex(myVDeg, myVMults);
        }
    }

    /// <summary>
    /// Locates U among the U knots (with repetition if WithKnotRepetition): I1 = I2 on a knot (within
    /// ParametricTolerance), else UKnots(I1) &lt; U &lt; UKnots(I2); I1 = 0 below the first knot, I2 = Length + 1
    /// above the last.
    /// </summary>
    public void LocateU(double U, double ParametricTolerance, out int I1, out int I2, bool WithKnotRepetition = false)
    {
        double NewU = U, vbid = myVKnots.Value(1);
        NCollection_Array1<double> TheKnots = WithKnotRepetition ? myUFlatKnots : myUKnots;

        PeriodicNormalization(ref NewU, ref vbid); // Handle periodicity

        double UFirst = TheKnots[1];
        double ULast = TheKnots[TheKnots.Length()];
        double PParametricTolerance = Math.Abs(ParametricTolerance);
        if (Math.Abs(NewU - UFirst) <= PParametricTolerance)
        {
            I1 = I2 = 1;
        }
        else if (Math.Abs(NewU - ULast) <= PParametricTolerance)
        {
            I1 = I2 = TheKnots.Length();
        }
        else if (NewU < UFirst)
        {
            I2 = 1;
            I1 = 0;
        }
        else if (NewU > ULast)
        {
            I1 = TheKnots.Length();
            I2 = I1 + 1;
        }
        else
        {
            BSplCLib.Hunt(TheKnots, NewU, out I1);
            I1 = Max(Min(I1, TheKnots.Upper()), TheKnots.Lower());
            while (I1 + 1 <= TheKnots.Upper() && Math.Abs(TheKnots[I1 + 1] - NewU) <= PParametricTolerance)
            {
                I1++;
            }
            if (Math.Abs(TheKnots[I1] - NewU) <= PParametricTolerance)
            {
                I2 = I1;
            }
            else
            {
                I2 = I1 + 1;
            }
        }
    }

    /// <summary>Locates V among the V knots (see <see cref="LocateU"/>).</summary>
    public void LocateV(double V, double ParametricTolerance, out int I1, out int I2, bool WithKnotRepetition = false)
    {
        double NewV = V, ubid = myUKnots.Value(1);
        NCollection_Array1<double> TheKnots = WithKnotRepetition ? myVFlatKnots : myVKnots;

        PeriodicNormalization(ref ubid, ref NewV); // Handle periodicity

        double VFirst = TheKnots[1];
        double VLast = TheKnots[TheKnots.Length()];
        double PParametricTolerance = Math.Abs(ParametricTolerance);
        if (Math.Abs(NewV - VFirst) <= PParametricTolerance)
        {
            I1 = I2 = 1;
        }
        else if (Math.Abs(NewV - VLast) <= PParametricTolerance)
        {
            I1 = I2 = TheKnots.Length();
        }
        else if (NewV < VFirst - PParametricTolerance)
        {
            I2 = 1;
            I1 = 0;
        }
        else if (NewV > VLast + PParametricTolerance)
        {
            I1 = TheKnots.Length();
            I2 = I1 + 1;
        }
        else
        {
            BSplCLib.Hunt(TheKnots, NewV, out I1);
            I1 = Max(Min(I1, TheKnots.Upper()), TheKnots.Lower());
            while (I1 + 1 <= TheKnots.Upper() && Math.Abs(TheKnots[I1 + 1] - NewV) <= PParametricTolerance)
            {
                I1++;
            }
            if (Math.Abs(TheKnots[I1] - NewV) <= PParametricTolerance)
            {
                I2 = I1;
            }
            else
            {
                I2 = I1 + 1;
            }
        }
    }

    /// <summary>Reverses the U direction; the U knots become UKnots(1) + UKnots(NbUKnots) - UKnots(i).</summary>
    protected internal sealed override void UReverse()
    {
        CheckMutable();
        ClearEvalRepresentation();
        BSplCLib.Reverse(myUMults);
        BSplCLib.Reverse(myUKnots);
        int last;
        if (myUPeriodic)
        {
            last = myUFlatKnots.Upper() - myUDeg - 1;
        }
        else
        {
            last = myPoles.UpperRow();
        }
        BSplSLib.Reverse(myPoles, last, true);
        if (myURational || myVRational)
        {
            BSplSLib.Reverse(myWeights, last, true);
        }
        updateUKnots();
    }

    /// <summary>UKnots(1) + UKnots(NbUKnots) - U.</summary>
    public sealed override double UReversedParameter(double U) => (myUKnots.Value(1) + myUKnots.Value(myUKnots.Length()) - U);

    /// <summary>Reverses the V direction; the V knots become VKnots(1) + VKnots(NbVKnots) - VKnots(i).</summary>
    protected internal sealed override void VReverse()
    {
        CheckMutable();
        ClearEvalRepresentation();
        BSplCLib.Reverse(myVMults);
        BSplCLib.Reverse(myVKnots);
        int last;
        if (myVPeriodic)
        {
            last = myVFlatKnots.Upper() - myVDeg - 1;
        }
        else
        {
            last = myPoles.UpperCol();
        }
        BSplSLib.Reverse(myPoles, last, false);
        if (myURational || myVRational)
        {
            BSplSLib.Reverse(myWeights, last, false);
        }
        updateVKnots();
    }

    /// <summary>VKnots(1) + VKnots(NbVKnots) - V.</summary>
    public sealed override double VReversedParameter(double V) => (myVKnots.Value(1) + myVKnots.Value(myVKnots.Length()) - V);

    /// <summary>Changes the poles of the column VIndex (rows CPoles.Lower() to Upper()).</summary>
    internal void SetPoleCol(int VIndex, NCollection_Array1<gp_Pnt> CPoles)
    {
        CheckMutable();
        if (VIndex < 1 || VIndex > myPoles.RowLength())
        {
            throw new Standard_OutOfRange("Geom_BSplineSurface::SetPoleCol: VIndex out of range");
        }
        if (CPoles.Lower() < 1 || CPoles.Lower() > myPoles.ColLength() || CPoles.Upper() < 1
            || CPoles.Upper() > myPoles.ColLength())
        {
            throw new Standard_ConstructionError("Geom_BSplineSurface::SetPoleCol: invalid array dimension");
        }

        ClearEvalRepresentation();
        for (int I = CPoles.Lower(); I <= CPoles.Upper(); I++)
        {
            myPoles[I + myPoles.LowerRow() - 1, VIndex + myPoles.LowerCol() - 1] = CPoles[I];
        }

        myMaxDerivInvOk = false;
    }

    /// <summary>Changes the poles and weights of the column VIndex.</summary>
    internal void SetPoleCol(int VIndex, NCollection_Array1<gp_Pnt> CPoles, NCollection_Array1<double> CPoleWeights)
    {
        CheckMutable();
        SetPoleCol(VIndex, CPoles);
        SetWeightCol(VIndex, CPoleWeights);
    }

    /// <summary>Changes the poles of the row UIndex (columns CPoles.Lower() to Upper()).</summary>
    internal void SetPoleRow(int UIndex, NCollection_Array1<gp_Pnt> CPoles)
    {
        CheckMutable();
        if (UIndex < 1 || UIndex > myPoles.ColLength())
        {
            throw new Standard_OutOfRange("Geom_BSplineSurface::SetPoleRow: UIndex out of range");
        }
        if (CPoles.Lower() < 1 || CPoles.Lower() > myPoles.RowLength() || CPoles.Upper() < 1
            || CPoles.Upper() > myPoles.RowLength())
        {
            throw new Standard_ConstructionError("Geom_BSplineSurface::SetPoleRow: invalid array dimension");
        }

        ClearEvalRepresentation();
        for (int I = CPoles.Lower(); I <= CPoles.Upper(); I++)
        {
            myPoles[UIndex + myPoles.LowerRow() - 1, I + myPoles.LowerCol() - 1] = CPoles[I];
        }

        myMaxDerivInvOk = false;
    }

    /// <summary>Changes the poles and weights of the row UIndex.</summary>
    internal void SetPoleRow(int UIndex, NCollection_Array1<gp_Pnt> CPoles, NCollection_Array1<double> CPoleWeights)
    {
        CheckMutable();
        SetPoleRow(UIndex, CPoles);
        SetWeightRow(UIndex, CPoleWeights);
    }

    /// <summary>Changes the pole (UIndex, VIndex).</summary>
    /// <remarks>OCCT checks no index (a bad one throws <see cref="IndexOutOfRangeException"/> here).</remarks>
    internal void SetPole(int UIndex, int VIndex, in gp_Pnt P)
    {
        CheckMutable();
        ClearEvalRepresentation();
        myPoles.SetValue(UIndex + myPoles.LowerRow() - 1, VIndex + myPoles.LowerCol() - 1, P);
        myMaxDerivInvOk = false;
    }

    /// <summary>Changes the pole (UIndex, VIndex) and its weight.</summary>
    internal void SetPole(int UIndex, int VIndex, in gp_Pnt P, double Weight)
    {
        CheckMutable();
        SetWeight(UIndex, VIndex, Weight);
        SetPole(UIndex, VIndex, P);
    }

    /// <summary>
    /// Moves the point at (U, V) to P by moving the poles UIndex1-UIndex2 x VIndex1-VIndex2; returns the range of
    /// poles actually moved (UFirstModifiedPole = 0 if none: P is already the point, or the indices don't
    /// influence it).
    /// </summary>
    /// <exception cref="Standard_OutOfRange">Indices out of the pole grid or not increasing.</exception>
    internal void MovePoint(double U,
                            double V,
                            in gp_Pnt P,
                            int UIndex1,
                            int UIndex2,
                            int VIndex1,
                            int VIndex2,
                            out int UFirstModifiedPole,
                            out int ULastmodifiedPole,
                            out int VFirstModifiedPole,
                            out int VLastmodifiedPole)
    {
        CheckMutable();
        if (UIndex1 < 1 || UIndex1 > myPoles.UpperRow() || UIndex2 < 1 || UIndex2 > myPoles.UpperRow()
            || UIndex1 > UIndex2 || VIndex1 < 1 || VIndex1 > myPoles.UpperCol() || VIndex2 < 1
            || VIndex2 > myPoles.UpperCol() || VIndex1 > VIndex2)
        {
            throw new Standard_OutOfRange("Geom_BSplineSurface::MovePoint: Index and #pole mismatch");
        }

        NCollection_Array2<gp_Pnt> npoles = new NCollection_Array2<gp_Pnt>(1, myPoles.UpperRow(), 1, myPoles.UpperCol());
        gp_Pnt P0;
        D0(U, V, out P0);
        gp_Vec Displ = new gp_Vec(P0, P);
        bool rational = (myURational || myVRational);
        BSplSLib.MovePoint(U,
                           V,
                           Displ,
                           UIndex1,
                           UIndex2,
                           VIndex1,
                           VIndex2,
                           myUDeg,
                           myVDeg,
                           rational,
                           myPoles,
                           myWeights,
                           myUFlatKnots,
                           myVFlatKnots,
                           out UFirstModifiedPole,
                           out ULastmodifiedPole,
                           out VFirstModifiedPole,
                           out VLastmodifiedPole,
                           npoles);
        if (UFirstModifiedPole != 0)
        {
            ClearEvalRepresentation();
            myPoles = npoles;
        }
        myMaxDerivInvOk = false;
    }

    /// <summary>The parametric bounds: the knots at the ends of the parametric range in each direction.</summary>
    public sealed override void Bounds(out double U1, out double U2, out double V1, out double V2)
    {
        U1 = myUFlatKnots.Value(myUDeg + 1);
        U2 = myUFlatKnots.Value(myUFlatKnots.Upper() - myUDeg);
        V1 = myVFlatKnots.Value(myVDeg + 1);
        V2 = myVFlatKnots.Value(myVFlatKnots.Upper() - myVDeg);
    }

    /// <summary>The maximum degree in each direction: 25.</summary>
    public static int MaxDegree() => BSplCLib.MaxDegree();

    /// <summary>True if the weights vary in the U direction.</summary>
    public bool IsURational() => myURational;

    /// <summary>True if the weights vary in the V direction.</summary>
    public bool IsVRational() => myVRational;

    /// <summary>The lower of the U and V continuities.</summary>
    public sealed override GeomAbs_Shape Continuity() => ((myUSmooth < myVSmooth) ? myUSmooth : myVSmooth);

    /// <summary>The number of U knots.</summary>
    public int NbUKnots() => myUKnots.Length();

    /// <summary>The number of poles in U (rows of the pole grid).</summary>
    public int NbUPoles() => myPoles.ColLength();

    /// <summary>The number of V knots.</summary>
    public int NbVKnots() => myVKnots.Length();

    /// <summary>The number of poles in V (columns of the pole grid).</summary>
    public int NbVPoles() => myPoles.RowLength();

    /// <summary>The degree in U.</summary>
    public int UDegree() => myUDeg;

    /// <summary>The degree in V.</summary>
    public int VDegree() => myVDeg;

    /// <summary>The distribution of the U knots (uniform, quasi-uniform, piecewise Bezier, non-uniform).</summary>
    public GeomAbs_BSplKnotDistribution UKnotDistribution() => myUKnotSet;

    /// <summary>The distribution of the V knots.</summary>
    public GeomAbs_BSplKnotDistribution VKnotDistribution() => myVKnotSet;

    /// <summary>
    /// Inserts the U knots with the multiplicities Mults (a knot within ParametricTolerance of an existing one
    /// raises its multiplicity: by Mults(i) when Add, to Mults(i) otherwise).
    /// </summary>
    /// <exception cref="Standard_ConstructionError">The insertion is invalid (BSplCLib::PrepareInsertKnots).</exception>
    internal void InsertUKnots(NCollection_Array1<double> Knots,
                               NCollection_Array1<int> Mults,
                               double ParametricTolerance = 0.0,
                               bool Add = true)
    {
        CheckMutable();
        // Check and compute new sizes
        int nbpoles, nbknots;

        if (!BSplCLib.PrepareInsertKnots(myUDeg,
                                         myUPeriodic,
                                         myUKnots,
                                         myUMults,
                                         Knots,
                                         Mults,
                                         out nbpoles,
                                         out nbknots,
                                         ParametricTolerance,
                                         Add))
        {
            throw new Standard_ConstructionError("Geom_BSplineSurface::InsertUKnots");
        }

        if (nbpoles == myPoles.ColLength())
        {
            return;
        }

        ClearEvalRepresentation();
        NCollection_Array2<gp_Pnt> npoles = new NCollection_Array2<gp_Pnt>(1, nbpoles, 1, myPoles.RowLength());
        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, nbknots);
        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, nbknots);

        if (myURational || myVRational)
        {
            NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, nbpoles, 1, myPoles.RowLength());
            BSplSLib.InsertKnots(true,
                                 myUDeg,
                                 myUPeriodic,
                                 myPoles,
                                 InternalWeights(),
                                 myUKnots,
                                 myUMults,
                                 Knots,
                                 Mults,
                                 npoles,
                                 nweights,
                                 nknots,
                                 nmults,
                                 ParametricTolerance,
                                 Add);
            myWeights = nweights;
        }
        else
        {
            BSplSLib.InsertKnots(true,
                                 myUDeg,
                                 myUPeriodic,
                                 myPoles,
                                 BSplSLib.NoWeights(),
                                 myUKnots,
                                 myUMults,
                                 Knots,
                                 Mults,
                                 npoles,
                                 BSplSLib.NoWeights(),
                                 nknots,
                                 nmults,
                                 ParametricTolerance,
                                 Add);
            myWeights = BSplSLib.UnitWeights(npoles.ColLength(), npoles.RowLength());
        }

        myPoles = npoles;
        myUKnots = nknots;
        myUMults = nmults;
        updateUKnots();
    }

    /// <summary>Inserts the V knots (see <see cref="InsertUKnots"/>).</summary>
    /// <exception cref="Standard_ConstructionError">The insertion is invalid (BSplCLib::PrepareInsertKnots).</exception>
    internal void InsertVKnots(NCollection_Array1<double> Knots,
                               NCollection_Array1<int> Mults,
                               double ParametricTolerance = 0.0,
                               bool Add = true)
    {
        CheckMutable();
        // Check and compute new sizes
        int nbpoles, nbknots;

        if (!BSplCLib.PrepareInsertKnots(myVDeg,
                                         myVPeriodic,
                                         myVKnots,
                                         myVMults,
                                         Knots,
                                         Mults,
                                         out nbpoles,
                                         out nbknots,
                                         ParametricTolerance,
                                         Add))
        {
            throw new Standard_ConstructionError("Geom_BSplineSurface::InsertVKnots");
        }

        if (nbpoles == myPoles.RowLength())
        {
            return;
        }

        ClearEvalRepresentation();
        NCollection_Array2<gp_Pnt> npoles = new NCollection_Array2<gp_Pnt>(1, myPoles.ColLength(), 1, nbpoles);
        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, nbknots);
        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, nbknots);

        if (myURational || myVRational)
        {
            NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, myPoles.ColLength(), 1, nbpoles);
            BSplSLib.InsertKnots(false,
                                 myVDeg,
                                 myVPeriodic,
                                 myPoles,
                                 InternalWeights(),
                                 myVKnots,
                                 myVMults,
                                 Knots,
                                 Mults,
                                 npoles,
                                 nweights,
                                 nknots,
                                 nmults,
                                 ParametricTolerance,
                                 Add);
            myWeights = nweights;
        }
        else
        {
            BSplSLib.InsertKnots(false,
                                 myVDeg,
                                 myVPeriodic,
                                 myPoles,
                                 BSplSLib.NoWeights(),
                                 myVKnots,
                                 myVMults,
                                 Knots,
                                 Mults,
                                 npoles,
                                 BSplSLib.NoWeights(),
                                 nknots,
                                 nmults,
                                 ParametricTolerance,
                                 Add);
            myWeights = BSplSLib.UnitWeights(npoles.ColLength(), npoles.RowLength());
        }

        myPoles = npoles;
        myVKnots = nknots;
        myVMults = nmults;
        updateVKnots();
    }

    /// <summary>
    /// Lowers the multiplicity of the U knot Index to M (0 removes it), if the surface moves by no more than
    /// Tolerance; returns false and leaves the surface unchanged otherwise.
    /// </summary>
    /// <exception cref="Standard_OutOfRange">Index is an end knot (non-periodic) or out of range.</exception>
    internal bool RemoveUKnot(int Index, int M, double Tolerance)
    {
        CheckMutable();
        if (M < 0)
        {
            return true;
        }

        int I1 = FirstUKnotIndex();
        int I2 = LastUKnotIndex();

        if (!myUPeriodic && (Index <= I1 || Index >= I2))
        {
            throw new Standard_OutOfRange("Geom_BSplineSurface::RemoveUKnot: invalid Index");
        }
        else if (myUPeriodic && (Index < I1 || Index > I2))
        {
            throw new Standard_OutOfRange("Geom_BSplineSurface::RemoveUKnot: invalid Index for periodic case");
        }

        int step = myUMults.Value(Index) - M;
        if (step <= 0)
        {
            return true;
        }

        NCollection_Array2<gp_Pnt> npoles = new NCollection_Array2<gp_Pnt>(1, myPoles.ColLength() - step, 1, myPoles.RowLength());
        int aNewUKnotLen = (M == 0) ? myUKnots.Length() - 1 : myUKnots.Length();
        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, aNewUKnotLen);
        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, aNewUKnotLen);

        if (myURational || myVRational)
        {
            NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, npoles.ColLength(), 1, npoles.RowLength());
            if (!BSplSLib.RemoveKnot(true,
                                     Index,
                                     M,
                                     myUDeg,
                                     myUPeriodic,
                                     myPoles,
                                     InternalWeights(),
                                     myUKnots,
                                     myUMults,
                                     npoles,
                                     nweights,
                                     nknots,
                                     nmults,
                                     Tolerance))
            {
                return false;
            }
            myWeights = nweights;
        }
        else
        {
            if (!BSplSLib.RemoveKnot(true,
                                     Index,
                                     M,
                                     myUDeg,
                                     myUPeriodic,
                                     myPoles,
                                     BSplSLib.NoWeights(),
                                     myUKnots,
                                     myUMults,
                                     npoles,
                                     BSplSLib.NoWeights(),
                                     nknots,
                                     nmults,
                                     Tolerance))
            {
                return false;
            }
            myWeights = BSplSLib.UnitWeights(npoles.ColLength(), npoles.RowLength());
        }

        ClearEvalRepresentation();
        myPoles = npoles;
        myUKnots = nknots;
        myUMults = nmults;

        myMaxDerivInvOk = false;
        updateUKnots();
        return true;
    }

    /// <summary>Lowers the multiplicity of the V knot Index to M (see <see cref="RemoveUKnot"/>).</summary>
    internal bool RemoveVKnot(int Index, int M, double Tolerance)
    {
        CheckMutable();
        if (M < 0)
        {
            return true;
        }

        int I1 = FirstVKnotIndex();
        int I2 = LastVKnotIndex();

        if (!myVPeriodic && (Index <= I1 || Index >= I2))
        {
            throw new Standard_OutOfRange("Geom_BSplineSurface::RemoveVKnot: invalid Index");
        }
        else if (myVPeriodic && (Index < I1 || Index > I2))
        {
            throw new Standard_OutOfRange("Geom_BSplineSurface::RemoveVKnot: invalid Index for periodic case");
        }

        int step = myVMults.Value(Index) - M;
        if (step <= 0)
        {
            return true;
        }

        NCollection_Array2<gp_Pnt> npoles = new NCollection_Array2<gp_Pnt>(1, myPoles.ColLength(), 1, myPoles.RowLength() - step);
        int aNewVKnotLen = (M == 0) ? myVKnots.Length() - 1 : myVKnots.Length();
        NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, aNewVKnotLen);
        NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, aNewVKnotLen);

        if (myURational || myVRational)
        {
            NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, npoles.ColLength(), 1, npoles.RowLength());
            if (!BSplSLib.RemoveKnot(false,
                                     Index,
                                     M,
                                     myVDeg,
                                     myVPeriodic,
                                     myPoles,
                                     InternalWeights(),
                                     myVKnots,
                                     myVMults,
                                     npoles,
                                     nweights,
                                     nknots,
                                     nmults,
                                     Tolerance))
            {
                return false;
            }
            myWeights = nweights;
        }
        else
        {
            if (!BSplSLib.RemoveKnot(false,
                                     Index,
                                     M,
                                     myVDeg,
                                     myVPeriodic,
                                     myPoles,
                                     BSplSLib.NoWeights(),
                                     myVKnots,
                                     myVMults,
                                     npoles,
                                     BSplSLib.NoWeights(),
                                     nknots,
                                     nmults,
                                     Tolerance))
            {
                return false;
            }
            myWeights = BSplSLib.UnitWeights(npoles.ColLength(), npoles.RowLength());
        }

        ClearEvalRepresentation();
        myPoles = npoles;
        myVKnots = nknots;
        myVMults = nmults;

        myMaxDerivInvOk = false;
        updateVKnots();
        return true;
    }

    /// <summary>
    /// The parametric tolerances in U and V matching the 3D tolerance Tolerance3D (computed once and cached, as in
    /// OCCT; the cache is not part of the surface's value, so this works on a frozen surface).
    /// </summary>
    public void Resolution(double Tolerance3D, out double UTolerance, out double VTolerance)
    {
        if (!myMaxDerivInvOk)
        {
            double aUMaxDerivInv, aVMaxDerivInv;
            BSplSLib.Resolution(myPoles,
                                InternalWeights(),
                                myUKnots,
                                myVKnots,
                                myUMults,
                                myVMults,
                                myUDeg,
                                myVDeg,
                                myURational,
                                myVRational,
                                myUPeriodic,
                                myVPeriodic,
                                1.0,
                                out aUMaxDerivInv,
                                out aVMaxDerivInv);
            myUMaxDerivInv = aUMaxDerivInv;
            myVMaxDerivInv = aVMaxDerivInv;
            myMaxDerivInvOk = true;
        }
        UTolerance = Tolerance3D * myUMaxDerivInv;
        VTolerance = Tolerance3D * myVMaxDerivInv;
    }

    //=============================================================================================
    // Internal access to the stored arrays (OCCT.NET)
    //=============================================================================================

    /// <summary>The stored poles: shared, don't modify (OCCT's <c>Poles()</c> const reference).</summary>
    internal NCollection_Array2<gp_Pnt> InternalPoles() => myPoles;

    /// <summary>The stored weights, or null for a non-rational surface (OCCT's <c>Weights()</c> pointer). Don't modify.</summary>
    internal NCollection_Array2<double>? InternalWeights() => (myURational || myVRational) ? myWeights : BSplSLib.NoWeights();

    /// <summary>The weights, NbUPoles x NbVPoles of them (OCCT's <c>WeightsArray()</c>): shared, don't modify.</summary>
    internal NCollection_Array2<double> InternalWeightsArray() => myWeights;

    /// <summary>The stored U knots: shared, don't modify.</summary>
    internal NCollection_Array1<double> InternalUKnots() => myUKnots;

    /// <summary>The stored V knots: shared, don't modify.</summary>
    internal NCollection_Array1<double> InternalVKnots() => myVKnots;

    /// <summary>The stored U multiplicities: shared, don't modify.</summary>
    internal NCollection_Array1<int> InternalUMultiplicities() => myUMults;

    /// <summary>The stored V multiplicities: shared, don't modify.</summary>
    internal NCollection_Array1<int> InternalVMultiplicities() => myVMults;

    /// <summary>The stored U flat knots: shared, don't modify.</summary>
    internal NCollection_Array1<double> InternalUKnotSequence() => myUFlatKnots;

    /// <summary>The stored V flat knots: shared, don't modify.</summary>
    internal NCollection_Array1<double> InternalVKnotSequence() => myVFlatKnots;

    //=============================================================================================
    // Non-mutating twins (OCCT.NET)
    //=============================================================================================

    private Geom_BSplineSurface CopyBSpline() => (Geom_BSplineSurface)Copy();

    /// <summary>A copy with U and V exchanged (see <see cref="ExchangeUV"/>).</summary>
    public Geom_BSplineSurface ExchangedUV()
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.ExchangeUV();
        return aCopy;
    }

    /// <summary>A copy made U-periodic (see <see cref="SetUPeriodic"/>).</summary>
    public Geom_BSplineSurface WithUPeriodic()
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.SetUPeriodic();
        return aCopy;
    }

    /// <summary>A copy made V-periodic (see <see cref="SetVPeriodic"/>).</summary>
    public Geom_BSplineSurface WithVPeriodic()
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.SetVPeriodic();
        return aCopy;
    }

    /// <summary>A copy whose U origin is the knot Index (see <see cref="SetUOrigin"/>).</summary>
    public Geom_BSplineSurface WithUOrigin(int Index)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.SetUOrigin(Index);
        return aCopy;
    }

    /// <summary>A copy whose V origin is the knot Index (see <see cref="SetVOrigin"/>).</summary>
    public Geom_BSplineSurface WithVOrigin(int Index)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.SetVOrigin(Index);
        return aCopy;
    }

    /// <summary>A copy made non-periodic in U (see <see cref="SetUNotPeriodic"/>).</summary>
    public Geom_BSplineSurface WithUNotPeriodic()
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.SetUNotPeriodic();
        return aCopy;
    }

    /// <summary>A copy made non-periodic in V (see <see cref="SetVNotPeriodic"/>).</summary>
    public Geom_BSplineSurface WithVNotPeriodic()
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.SetVNotPeriodic();
        return aCopy;
    }

    /// <summary>A copy with the degrees raised (see <see cref="IncreaseDegree"/>).</summary>
    public Geom_BSplineSurface DegreeIncreased(int UDegree, int VDegree)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.IncreaseDegree(UDegree, VDegree);
        return aCopy;
    }

    /// <summary>A copy with the U knots inserted (see <see cref="InsertUKnots"/>).</summary>
    public Geom_BSplineSurface UKnotsInserted(NCollection_Array1<double> Knots,
                                              NCollection_Array1<int> Mults,
                                              double ParametricTolerance = 0.0,
                                              bool Add = true)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.InsertUKnots(Knots, Mults, ParametricTolerance, Add);
        return aCopy;
    }

    /// <summary>A copy with the V knots inserted (see <see cref="InsertVKnots"/>).</summary>
    public Geom_BSplineSurface VKnotsInserted(NCollection_Array1<double> Knots,
                                              NCollection_Array1<int> Mults,
                                              double ParametricTolerance = 0.0,
                                              bool Add = true)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.InsertVKnots(Knots, Mults, ParametricTolerance, Add);
        return aCopy;
    }

    /// <summary>A copy with the U knot U inserted (see <see cref="InsertUKnot"/>).</summary>
    public Geom_BSplineSurface UKnotInserted(double U, int M, double ParametricTolerance, bool Add = true)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.InsertUKnot(U, M, ParametricTolerance, Add);
        return aCopy;
    }

    /// <summary>A copy with the V knot V inserted (see <see cref="InsertVKnot"/>).</summary>
    public Geom_BSplineSurface VKnotInserted(double V, int M, double ParametricTolerance, bool Add = true)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.InsertVKnot(V, M, ParametricTolerance, Add);
        return aCopy;
    }

    /// <summary>A copy with the U knot Index lowered to multiplicity M, or null if that fails (see <see cref="RemoveUKnot"/>).</summary>
    public Geom_BSplineSurface? UKnotRemoved(int Index, int M, double Tolerance)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        return aCopy.RemoveUKnot(Index, M, Tolerance) ? aCopy : null;
    }

    /// <summary>A copy with the V knot Index lowered to multiplicity M, or null if that fails (see <see cref="RemoveVKnot"/>).</summary>
    public Geom_BSplineSurface? VKnotRemoved(int Index, int M, double Tolerance)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        return aCopy.RemoveVKnot(Index, M, Tolerance) ? aCopy : null;
    }

    /// <summary>A copy with the multiplicity of the U knot UIndex raised to M.</summary>
    public Geom_BSplineSurface UMultiplicityIncreased(int UIndex, int M)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.IncreaseUMultiplicity(UIndex, M);
        return aCopy;
    }

    /// <summary>A copy with the multiplicities of the U knots FromI1 to ToI2 raised to M.</summary>
    public Geom_BSplineSurface UMultiplicityIncreased(int FromI1, int ToI2, int M)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.IncreaseUMultiplicity(FromI1, ToI2, M);
        return aCopy;
    }

    /// <summary>A copy with the multiplicities of the U knots FromI1 to ToI2 raised by Step.</summary>
    public Geom_BSplineSurface UMultiplicityIncremented(int FromI1, int ToI2, int Step)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.IncrementUMultiplicity(FromI1, ToI2, Step);
        return aCopy;
    }

    /// <summary>A copy with the multiplicity of the V knot VIndex raised to M.</summary>
    public Geom_BSplineSurface VMultiplicityIncreased(int VIndex, int M)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.IncreaseVMultiplicity(VIndex, M);
        return aCopy;
    }

    /// <summary>A copy with the multiplicities of the V knots FromI1 to ToI2 raised to M.</summary>
    public Geom_BSplineSurface VMultiplicityIncreased(int FromI1, int ToI2, int M)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.IncreaseVMultiplicity(FromI1, ToI2, M);
        return aCopy;
    }

    /// <summary>A copy with the multiplicities of the V knots FromI1 to ToI2 raised by Step.</summary>
    public Geom_BSplineSurface VMultiplicityIncremented(int FromI1, int ToI2, int Step)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.IncrementVMultiplicity(FromI1, ToI2, Step);
        return aCopy;
    }

    /// <summary>A copy restricted to [U1, U2] x [V1, V2] (see <see cref="Segment"/>).</summary>
    public Geom_BSplineSurface Segmented(double U1,
                                         double U2,
                                         double V1,
                                         double V2,
                                         double theUTolerance = Precision_PConfusion,
                                         double theVTolerance = Precision_PConfusion)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.Segment(U1, U2, V1, V2, theUTolerance, theVTolerance);
        return aCopy;
    }

    /// <summary>A copy restricted to [U1, U2] x [V1, V2] where the bounds change (see <see cref="CheckAndSegment"/>).</summary>
    public Geom_BSplineSurface CheckedAndSegmented(double U1,
                                                   double U2,
                                                   double V1,
                                                   double V2,
                                                   double theUTolerance = Precision_PConfusion,
                                                   double theVTolerance = Precision_PConfusion)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.CheckAndSegment(U1, U2, V1, V2, theUTolerance, theVTolerance);
        return aCopy;
    }

    public Geom_BSplineSurface WithUKnot(int UIndex, double K)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.SetUKnot(UIndex, K);
        return aCopy;
    }

    public Geom_BSplineSurface WithUKnots(NCollection_Array1<double> UK)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.SetUKnots(UK);
        return aCopy;
    }

    public Geom_BSplineSurface WithUKnot(int UIndex, double K, int M)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.SetUKnot(UIndex, K, M);
        return aCopy;
    }

    public Geom_BSplineSurface WithVKnot(int VIndex, double K)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.SetVKnot(VIndex, K);
        return aCopy;
    }

    public Geom_BSplineSurface WithVKnots(NCollection_Array1<double> VK)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.SetVKnots(VK);
        return aCopy;
    }

    public Geom_BSplineSurface WithVKnot(int VIndex, double K, int M)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.SetVKnot(VIndex, K, M);
        return aCopy;
    }

    public Geom_BSplineSurface WithPole(int UIndex, int VIndex, in gp_Pnt P)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.SetPole(UIndex, VIndex, P);
        return aCopy;
    }

    public Geom_BSplineSurface WithPole(int UIndex, int VIndex, in gp_Pnt P, double Weight)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.SetPole(UIndex, VIndex, P, Weight);
        return aCopy;
    }

    public Geom_BSplineSurface WithPoleCol(int VIndex, NCollection_Array1<gp_Pnt> CPoles)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.SetPoleCol(VIndex, CPoles);
        return aCopy;
    }

    public Geom_BSplineSurface WithPoleCol(int VIndex, NCollection_Array1<gp_Pnt> CPoles, NCollection_Array1<double> CPoleWeights)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.SetPoleCol(VIndex, CPoles, CPoleWeights);
        return aCopy;
    }

    public Geom_BSplineSurface WithPoleRow(int UIndex, NCollection_Array1<gp_Pnt> CPoles)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.SetPoleRow(UIndex, CPoles);
        return aCopy;
    }

    public Geom_BSplineSurface WithPoleRow(int UIndex, NCollection_Array1<gp_Pnt> CPoles, NCollection_Array1<double> CPoleWeights)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.SetPoleRow(UIndex, CPoles, CPoleWeights);
        return aCopy;
    }

    public Geom_BSplineSurface WithWeight(int UIndex, int VIndex, double Weight)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.SetWeight(UIndex, VIndex, Weight);
        return aCopy;
    }

    public Geom_BSplineSurface WithWeightCol(int VIndex, NCollection_Array1<double> CPoleWeights)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.SetWeightCol(VIndex, CPoleWeights);
        return aCopy;
    }

    public Geom_BSplineSurface WithWeightRow(int UIndex, NCollection_Array1<double> CPoleWeights)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.SetWeightRow(UIndex, CPoleWeights);
        return aCopy;
    }

    /// <summary>A copy with the point at (U, V) moved to P (see <see cref="MovePoint"/>).</summary>
    public Geom_BSplineSurface PointMoved(double U,
                                          double V,
                                          in gp_Pnt P,
                                          int UIndex1,
                                          int UIndex2,
                                          int VIndex1,
                                          int VIndex2,
                                          out int UFirstIndex,
                                          out int ULastIndex,
                                          out int VFirstIndex,
                                          out int VLastIndex)
    {
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.MovePoint(U, V, P, UIndex1, UIndex2, VIndex1, VIndex2,
                        out UFirstIndex, out ULastIndex, out VFirstIndex, out VLastIndex);
        return aCopy;
    }

    /// <summary>A copy evaluated through a copy of theDesc (null: without a representation).</summary>
    public Geom_BSplineSurface WithEvalRepresentation(GeomEval_RepSurfaceDesc.Base? theDesc)
    {
        GeomEval_RepUtils.ValidateSurfaceDesc(theDesc, this);
        Geom_BSplineSurface aCopy = CopyBSpline();
        aCopy.SetEvalRepresentation(GeomEval_RepUtils.CloneSurfaceDesc(theDesc));
        return aCopy;
    }
}
