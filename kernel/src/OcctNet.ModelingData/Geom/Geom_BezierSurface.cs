// Port of: src/ModelingData/TKG3d/Geom/Geom_BezierSurface.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_BezierSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// A rational or non-rational Bezier surface: a grid of NbUPoles x NbVPoles poles (with weights when rational),
/// of degree NbUPoles - 1 in U and NbVPoles - 1 in V (at most <see cref="MaxDegree"/>, 25), parametrised on
/// [0, 1] x [0, 1]. The rows of the pole grid follow U, the columns V; indices start at 1.
/// </summary>
/// <remarks>
/// <para>
/// Arrays: <see cref="Poles()"/>, <see cref="Weights()"/> and <see cref="WeightsArray"/> return new copies (OCCT
/// returns references to the surface's arrays); the knots, multiplicities and knot sequences of the equivalent
/// B-spline are immutable <see cref="PackedList{T}"/>s. Code inside OCCT.NET uses the internal <c>Internal*</c>
/// accessors, which return the stored (or shared static) arrays: don't modify them.
/// </para>
/// <para>
/// The evaluation representation (<see cref="HasEvalRepresentation"/>) is set by
/// <see cref="WithEvalRepresentation"/>, which stores a copy of the descriptor. As in OCCT, only Transform,
/// UReverse and VReverse drop it.
/// </para>
/// </remarks>
public class Geom_BezierSurface : Geom_BoundedSurface
{
    private NCollection_Array2<gp_Pnt> myPoles = new NCollection_Array2<gp_Pnt>();
    private NCollection_Array2<double> myWeights = new NCollection_Array2<double>();
    private GeomEval_RepSurfaceDesc.Base? myEvalRep;
    private bool myURational = false;
    private bool myVRational = false;
    private double myUMaxDerivInv = 0.0;
    private double myVMaxDerivInv = 0.0;
    private volatile bool myMaxDerivInvOk = false;

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

    /// <summary>Checks the rationality of an array of weights in each direction.</summary>
    private static void Rational(NCollection_Array2<double> Weights, out bool Urational, out bool Vrational)
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

    private static void AddPoleCol(NCollection_Array2<gp_Pnt> Poles,
                                   NCollection_Array1<gp_Pnt> PoleCol,
                                   int AfterIndex,
                                   NCollection_Array2<gp_Pnt> NewPoles)
    {
        int InsertIndex = AfterIndex + NewPoles.LowerCol();
        int Offset = NewPoles.LowerRow() - PoleCol.Lower();
        int ColIndex = NewPoles.LowerCol();
        int RowIndex;
        while (ColIndex < InsertIndex)
        {
            RowIndex = NewPoles.LowerRow();
            while (RowIndex <= NewPoles.UpperRow())
            {
                NewPoles[RowIndex, ColIndex] = Poles[RowIndex, ColIndex];
                RowIndex++;
            }
            ColIndex++;
        }
        RowIndex = NewPoles.LowerRow();
        while (RowIndex <= NewPoles.UpperRow())
        {
            NewPoles[RowIndex, ColIndex] = PoleCol[RowIndex - Offset];
            RowIndex++;
        }
        ColIndex++;
        while (ColIndex <= NewPoles.UpperCol())
        {
            RowIndex = NewPoles.LowerRow();
            while (RowIndex <= NewPoles.UpperRow())
            {
                NewPoles[RowIndex, ColIndex] = Poles[RowIndex, ColIndex - 1];
                RowIndex++;
            }
            ColIndex++;
        }
    }

    private static void AddRatPoleCol(NCollection_Array2<gp_Pnt> Poles,
                                      NCollection_Array2<double> Weights,
                                      NCollection_Array1<gp_Pnt> PoleCol,
                                      NCollection_Array1<double> PoleWeightCol,
                                      int AfterIndex,
                                      NCollection_Array2<gp_Pnt> NewPoles,
                                      NCollection_Array2<double> NewWeights)
    {
        int InsertIndex = AfterIndex + NewPoles.LowerCol();
        int OffsetPol = NewPoles.LowerRow() - PoleCol.Lower();
        int OffsetW = NewWeights.LowerRow() - PoleWeightCol.Lower();

        int ColIndex = NewPoles.LowerCol();
        int RowIndex;
        while (ColIndex < InsertIndex)
        {
            RowIndex = NewPoles.LowerRow();
            while (RowIndex <= NewPoles.UpperRow())
            {
                NewPoles[RowIndex, ColIndex] = Poles[RowIndex, ColIndex];
                NewWeights[RowIndex, ColIndex] = Weights[RowIndex, ColIndex];
                RowIndex++;
            }
            ColIndex++;
        }
        RowIndex = NewPoles.LowerRow();
        while (RowIndex <= NewPoles.UpperRow())
        {
            NewPoles[RowIndex, ColIndex] = PoleCol[RowIndex - OffsetPol];
            NewWeights[RowIndex, ColIndex] = PoleWeightCol[RowIndex - OffsetW];
            RowIndex++;
        }
        ColIndex++;
        while (ColIndex <= NewPoles.UpperCol())
        {
            RowIndex = NewPoles.LowerRow();
            while (RowIndex <= NewPoles.UpperRow())
            {
                NewPoles[RowIndex, ColIndex] = Poles[RowIndex, ColIndex - 1];
                NewWeights[RowIndex, ColIndex] = Weights[RowIndex, ColIndex - 1];
                RowIndex++;
            }
            ColIndex++;
        }
    }

    private static void AddPoleRow(NCollection_Array2<gp_Pnt> Poles,
                                   NCollection_Array1<gp_Pnt> PoleRow,
                                   int AfterIndex,
                                   NCollection_Array2<gp_Pnt> NewPoles)
    {
        int InsertIndex = AfterIndex + NewPoles.LowerRow();
        int Offset = NewPoles.LowerCol() - PoleRow.Lower();
        int RowIndex = NewPoles.LowerRow();
        int ColIndex;
        while (RowIndex < InsertIndex)
        {
            ColIndex = NewPoles.LowerCol();
            while (ColIndex <= NewPoles.UpperCol())
            {
                NewPoles[RowIndex, ColIndex] = Poles[RowIndex, ColIndex];
                ColIndex++;
            }
            RowIndex++;
        }
        ColIndex = NewPoles.LowerCol();
        while (ColIndex <= NewPoles.UpperCol())
        {
            NewPoles[RowIndex, ColIndex] = PoleRow[ColIndex - Offset];
            ColIndex++;
        }
        RowIndex++;
        while (RowIndex <= NewPoles.UpperRow())
        {
            ColIndex = NewPoles.LowerCol();
            while (ColIndex <= NewPoles.UpperCol())
            {
                NewPoles[RowIndex, ColIndex] = Poles[RowIndex - 1, ColIndex];
                ColIndex++;
            }
            RowIndex++;
        }
    }

    private static void AddRatPoleRow(NCollection_Array2<gp_Pnt> Poles,
                                      NCollection_Array2<double> Weights,
                                      NCollection_Array1<gp_Pnt> PoleRow,
                                      NCollection_Array1<double> PoleWeightRow,
                                      int AfterIndex,
                                      NCollection_Array2<gp_Pnt> NewPoles,
                                      NCollection_Array2<double> NewWeights)
    {
        int InsertIndex = AfterIndex + NewPoles.LowerRow();
        int OffsetPol = NewPoles.LowerCol() - PoleRow.Lower();
        int OffsetW = NewWeights.LowerCol() - PoleWeightRow.Lower();

        int ColIndex;
        int RowIndex = NewPoles.LowerRow();
        while (RowIndex < InsertIndex)
        {
            ColIndex = NewPoles.LowerCol();
            while (ColIndex <= NewPoles.UpperCol())
            {
                NewPoles[RowIndex, ColIndex] = Poles[RowIndex, ColIndex];
                NewWeights[RowIndex, ColIndex] = Weights[RowIndex, ColIndex];
                ColIndex++;
            }
            RowIndex++;
        }
        ColIndex = NewPoles.LowerCol();
        while (ColIndex <= NewPoles.UpperCol())
        {
            NewPoles[RowIndex, ColIndex] = PoleRow[ColIndex - OffsetPol];
            NewWeights[RowIndex, ColIndex] = PoleWeightRow[ColIndex - OffsetW];
            ColIndex++;
        }
        RowIndex++;
        while (RowIndex <= NewPoles.UpperRow())
        {
            ColIndex = NewPoles.LowerCol();
            while (ColIndex <= NewPoles.UpperCol())
            {
                NewPoles[RowIndex, ColIndex] = Poles[RowIndex - 1, ColIndex];
                NewWeights[RowIndex, ColIndex] = Weights[RowIndex - 1, ColIndex];
                ColIndex++;
            }
            RowIndex++;
        }
    }

    private static void DeletePoleCol(NCollection_Array2<gp_Pnt> Poles, int Index, NCollection_Array2<gp_Pnt> NewPoles)
    {
        int Offset = 0;
        int RowIndex;
        int ColIndex = NewPoles.LowerCol();
        while (ColIndex <= NewPoles.UpperCol())
        {
            RowIndex = NewPoles.LowerRow();
            if (ColIndex == Index)
            {
                Offset = 1;
            }
            while (RowIndex <= NewPoles.UpperRow())
            {
                NewPoles[RowIndex, ColIndex] = Poles[RowIndex, ColIndex + Offset];
                RowIndex++;
            }
            ColIndex++;
        }
    }

    private static void DeleteRatPoleCol(NCollection_Array2<gp_Pnt> Poles,
                                         NCollection_Array2<double> Weights,
                                         int Index,
                                         NCollection_Array2<gp_Pnt> NewPoles,
                                         NCollection_Array2<double> NewWeights)
    {
        int Offset = 0;
        int RowIndex;
        int ColIndex = NewPoles.LowerCol();
        while (ColIndex <= NewPoles.UpperCol())
        {
            RowIndex = NewPoles.LowerRow();
            if (ColIndex == Index)
            {
                Offset = 1;
            }
            while (RowIndex <= NewPoles.UpperRow())
            {
                NewPoles[RowIndex, ColIndex] = Poles[RowIndex, ColIndex + Offset];
                NewWeights[RowIndex, ColIndex] = Weights[RowIndex, ColIndex + Offset];
                RowIndex++;
            }
            ColIndex++;
        }
    }

    private static void DeletePoleRow(NCollection_Array2<gp_Pnt> Poles, int Index, NCollection_Array2<gp_Pnt> NewPoles)
    {
        int Offset = 0;
        int ColIndex;
        int RowIndex = NewPoles.LowerRow();
        while (RowIndex <= NewPoles.UpperRow())
        {
            ColIndex = NewPoles.LowerCol();
            if (RowIndex == Index)
            {
                Offset = 1;
            }
            while (ColIndex <= NewPoles.UpperCol())
            {
                NewPoles[RowIndex, ColIndex] = Poles[RowIndex + Offset, ColIndex];
                ColIndex++;
            }
            RowIndex++;
        }
    }

    private static void DeleteRatPoleRow(NCollection_Array2<gp_Pnt> Poles,
                                         NCollection_Array2<double> Weights,
                                         int Index,
                                         NCollection_Array2<gp_Pnt> NewPoles,
                                         NCollection_Array2<double> NewWeights)
    {
        int Offset = 0;
        int ColIndex;
        int RowIndex = NewPoles.LowerRow();
        while (RowIndex <= NewPoles.UpperRow())
        {
            ColIndex = NewPoles.LowerCol();
            if (RowIndex == Index)
            {
                Offset = 1;
            }
            while (ColIndex <= NewPoles.UpperCol())
            {
                NewPoles[RowIndex, ColIndex] = Poles[RowIndex + Offset, ColIndex];
                NewWeights[RowIndex, ColIndex] = Weights[RowIndex + Offset, ColIndex];
                ColIndex++;
            }
            RowIndex++;
        }
    }

    /// <summary>A copy of theOther (OCCT's copy constructor).</summary>
    public Geom_BezierSurface(Geom_BezierSurface theOther)
    {
        myPoles = new NCollection_Array2<gp_Pnt>(theOther.myPoles);
        myWeights = (theOther.myURational || theOther.myVRational)
                      ? new NCollection_Array2<double>(theOther.myWeights)
                      : BSplSLib.UnitWeights(theOther.myPoles.ColLength(), theOther.myPoles.RowLength());
        myEvalRep = GeomEval_RepUtils.CloneSurfaceDesc(theOther.myEvalRep);
        myURational = theOther.myURational;
        myVRational = theOther.myVRational;
        myUMaxDerivInv = theOther.myUMaxDerivInv;
        myVMaxDerivInv = theOther.myVMaxDerivInv;
        myMaxDerivInvOk = false;
    }

    /// <summary>A non-rational Bezier surface on the given poles.</summary>
    /// <exception cref="Standard_ConstructionError">Fewer than 2 or more than MaxDegree() + 1 poles in a direction.</exception>
    public Geom_BezierSurface(NCollection_Array2<gp_Pnt> SurfacePoles)
    {
        myMaxDerivInvOk = false;
        int NbUPoles = SurfacePoles.ColLength();
        int NbVPoles = SurfacePoles.RowLength();
        if (NbUPoles < 2 || NbUPoles > MaxDegree() + 1 || NbVPoles < 2 || NbVPoles > MaxDegree() + 1)
        {
            throw new Standard_ConstructionError();
        }

        myURational = false;
        myVRational = false;

        // init non rational
        init(SurfacePoles, null);
    }

    /// <summary>A rational Bezier surface (non-rational if the weights are all equal in each direction).</summary>
    /// <exception cref="Standard_ConstructionError">
    /// Fewer than 2 or more than MaxDegree() + 1 poles in a direction, weights of a different size, or a weight not
    /// greater than gp::Resolution().
    /// </exception>
    public Geom_BezierSurface(NCollection_Array2<gp_Pnt> SurfacePoles, NCollection_Array2<double> PoleWeights)
    {
        myMaxDerivInvOk = false;
        int NbUPoles = SurfacePoles.ColLength();
        int NbVPoles = SurfacePoles.RowLength();
        if (NbUPoles < 2 || NbUPoles > MaxDegree() + 1 || NbVPoles < 2 || NbVPoles > MaxDegree() + 1
            || NbVPoles != PoleWeights.RowLength() || NbUPoles != PoleWeights.ColLength())
        {
            throw new Standard_ConstructionError();
        }

        int Col = PoleWeights.LowerCol();
        while (Col <= PoleWeights.UpperCol())
        {
            int Row = PoleWeights.LowerRow();
            while (Row <= PoleWeights.UpperRow())
            {
                if (PoleWeights[Row, Col] <= gp.Resolution())
                {
                    throw new Standard_ConstructionError();
                }
                Row++;
            }
            Col++;
        }

        // Rational(PoleWeights, myURational, myVRational) inlined in OCCT
        int I, J;
        myURational = false;
        myVRational = false;
        J = PoleWeights.LowerCol();
        while (!myVRational && J <= PoleWeights.UpperCol())
        {
            I = PoleWeights.LowerRow();
            while (!myVRational && I <= PoleWeights.UpperRow() - 1)
            {
                myVRational = (Math.Abs(PoleWeights[I, J] - PoleWeights[I + 1, J])
                               > Epsilon(Math.Abs(PoleWeights[I, J])));
                I++;
            }
            J++;
        }
        I = PoleWeights.LowerRow();
        while (!myURational && I <= PoleWeights.UpperRow())
        {
            J = PoleWeights.LowerCol();
            while (!myURational && J <= PoleWeights.UpperCol() - 1)
            {
                myURational = (Math.Abs(PoleWeights[I, J] - PoleWeights[I, J + 1])
                               > Epsilon(Math.Abs(PoleWeights[I, J])));
                J++;
            }
            I++;
        }

        if (myURational || myVRational)
        {
            init(SurfacePoles, PoleWeights);
        }
        else
        {
            init(SurfacePoles, null);
        }
    }

    /// <summary>The maximum degree in each direction: 25.</summary>
    public static int MaxDegree() => BSplCLib.MaxDegree();

    /// <summary>Exchanges the U and V parametric directions; the poles grid is transposed.</summary>
    internal void ExchangeUV()
    {
        CheckMutable();
        int LR = myPoles.LowerRow();
        int UR = myPoles.UpperRow();
        int LC = myPoles.LowerCol();
        int UC = myPoles.UpperCol();

        NCollection_Array2<gp_Pnt> npoles = new NCollection_Array2<gp_Pnt>(LC, UC, LR, UR);
        NCollection_Array2<double> nweights = new NCollection_Array2<double>();
        bool rat = (myURational || myVRational);
        if (rat)
        {
            nweights.Resize(LC, UC, LR, UR, false);
        }

        for (int i = LC; i <= UC; i++)
        {
            for (int j = LR; j <= UR; j++)
            {
                npoles[i, j] = myPoles[j, i];
                if (rat)
                {
                    nweights[i, j] = myWeights[j, i];
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
            myWeights = BSplSLib.UnitWeights(UC - LC + 1, UR - LR + 1);
        }
        (myURational, myVRational) = (myVRational, myURational);
        myMaxDerivInvOk = false;
    }

    /// <summary>Raises the degrees to UDeg and VDeg.</summary>
    /// <exception cref="Standard_ConstructionError">A degree lower than the current one or above MaxDegree().</exception>
    internal void Increase(int UDeg, int VDeg)
    {
        CheckMutable();
        if (UDeg < UDegree() || UDeg > Geom_BezierSurface.MaxDegree() || VDeg < VDegree()
            || VDeg > Geom_BezierSurface.MaxDegree())
        {
            throw new Standard_ConstructionError();
        }

        int oldUDeg = UDegree();
        int oldVDeg = VDegree();
        int IncUDeg = UDeg - oldUDeg;
        int IncVDeg = VDeg - oldVDeg;
        if (IncUDeg == 0 && IncVDeg == 0)
        {
            return;
        }

        if (IncUDeg > 0)
        {
            NCollection_Array2<gp_Pnt> npoles = new NCollection_Array2<gp_Pnt>(1, UDeg + 1, 1, oldVDeg + 1);
            NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, 2);
            NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, 2);
            if (myURational || myVRational)
            {
                NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, UDeg + 1, 1, VDegree() + 1);

                BSplSLib.IncreaseDegree(true,
                                        oldUDeg,
                                        UDeg,
                                        false,
                                        myPoles,
                                        myWeights,
                                        InternalUKnots(),
                                        InternalUMultiplicities(),
                                        npoles,
                                        nweights,
                                        nknots,
                                        nmults);
                myWeights = nweights;
            }
            else
            {
                BSplSLib.IncreaseDegree(true,
                                        oldUDeg,
                                        UDeg,
                                        false,
                                        myPoles,
                                        BSplSLib.NoWeights(),
                                        InternalUKnots(),
                                        InternalUMultiplicities(),
                                        npoles,
                                        BSplSLib.NoWeights(),
                                        nknots,
                                        nmults);
            }
            myPoles = npoles;
        }

        if (IncVDeg > 0)
        {
            NCollection_Array2<gp_Pnt> npoles = new NCollection_Array2<gp_Pnt>(1, UDeg + 1, 1, VDeg + 1);
            NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, 2);
            NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, 2);
            // OCCT passes UKnots() as the V knots: the Bezier knots are (0, 1) in both directions.
            if (myURational || myVRational)
            {
                NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, UDeg + 1, 1, VDeg + 1);

                BSplSLib.IncreaseDegree(false,
                                        oldVDeg,
                                        VDeg,
                                        false,
                                        myPoles,
                                        myWeights,
                                        InternalUKnots(),
                                        InternalVMultiplicities(),
                                        npoles,
                                        nweights,
                                        nknots,
                                        nmults);
                myWeights = nweights;
            }
            else
            {
                BSplSLib.IncreaseDegree(false,
                                        oldVDeg,
                                        VDeg,
                                        false,
                                        myPoles,
                                        BSplSLib.NoWeights(),
                                        InternalUKnots(),
                                        InternalVMultiplicities(),
                                        npoles,
                                        BSplSLib.NoWeights(),
                                        nknots,
                                        nmults);
            }
            myPoles = npoles;
        }

        if (myURational || myVRational)
        {
            Rational(myWeights, out myURational, out myVRational);
            if (!(myURational || myVRational))
            {
                myWeights = BSplSLib.UnitWeights(myPoles.ColLength(), myPoles.RowLength());
            }
        }
        else
        {
            myWeights = BSplSLib.UnitWeights(myPoles.ColLength(), myPoles.RowLength());
        }
        myMaxDerivInvOk = false;
    }

    /// <summary>Inserts a column of poles after the column VIndex (weight 1 if the surface is rational).</summary>
    /// <exception cref="Standard_OutOfRange">VIndex out of [1, NbVPoles].</exception>
    /// <exception cref="Standard_ConstructionError">CPoles.Length() != NbUPoles.</exception>
    internal void InsertPoleColAfter(int VIndex, NCollection_Array1<gp_Pnt> CPoles)
    {
        CheckMutable();
        if (VIndex < 1 || VIndex > myPoles.RowLength())
        {
            throw new Standard_OutOfRange();
        }
        if (CPoles.Length() != myPoles.ColLength())
        {
            throw new Standard_ConstructionError();
        }

        int NbUPoles = myPoles.ColLength();
        int NbVPoles = myPoles.RowLength();

        NCollection_Array2<gp_Pnt> npoles = new NCollection_Array2<gp_Pnt>(1, NbUPoles, 1, NbVPoles + 1);

        if (myURational || myVRational)
        {
            NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, NbUPoles, 1, NbVPoles + 1);
            NCollection_Array1<double> CWeights = new NCollection_Array1<double>(1, NbUPoles);
            CWeights.Init(1.0);
            AddRatPoleCol(myPoles, myWeights, CPoles, CWeights, VIndex, npoles, nweights);
            myWeights = nweights;
        }
        else
        {
            AddPoleCol(myPoles, CPoles, VIndex, npoles);
            myWeights = BSplSLib.UnitWeights(NbUPoles, NbVPoles + 1);
        }
        myPoles = npoles;
        myMaxDerivInvOk = false;
    }

    /// <summary>Inserts a column of weighted poles after the column VIndex.</summary>
    /// <exception cref="Standard_OutOfRange">VIndex out of [1, NbVPoles].</exception>
    /// <exception cref="Standard_ConstructionError">Bad lengths, or a weight not greater than gp::Resolution().</exception>
    internal void InsertPoleColAfter(int VIndex, NCollection_Array1<gp_Pnt> CPoles, NCollection_Array1<double> CPoleWeights)
    {
        CheckMutable();
        if (VIndex < 1 || VIndex > myPoles.RowLength())
        {
            throw new Standard_OutOfRange();
        }
        if (CPoles.Length() != myPoles.ColLength() || CPoleWeights.Length() != CPoles.Length())
        {
            throw new Standard_ConstructionError();
        }
        int Index = CPoleWeights.Lower();
        while (Index <= CPoleWeights.Upper())
        {
            if (CPoleWeights[Index] <= gp.Resolution())
            {
                throw new Standard_ConstructionError();
            }
            Index++;
        }

        int NbUPoles = myPoles.ColLength();
        int NbVPoles = myPoles.RowLength();

        // Ensure weights are an owned copy for rational insertion
        if (!(myURational || myVRational))
        {
            myWeights = new NCollection_Array2<double>(myWeights);
        }

        NCollection_Array2<gp_Pnt> npoles = new NCollection_Array2<gp_Pnt>(1, NbUPoles, 1, NbVPoles + 1);
        NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, NbUPoles, 1, NbVPoles + 1);

        AddRatPoleCol(myPoles, myWeights, CPoles, CPoleWeights, VIndex, npoles, nweights);
        myPoles = npoles;
        myWeights = nweights;
        Rational(myWeights, out myURational, out myVRational);
        if (!(myURational || myVRational))
        {
            myWeights = BSplSLib.UnitWeights(myPoles.ColLength(), myPoles.RowLength());
        }
        myMaxDerivInvOk = false;
    }

    /// <summary>Inserts a column of poles before the column VIndex.</summary>
    internal void InsertPoleColBefore(int VIndex, NCollection_Array1<gp_Pnt> CPoles)
    {
        CheckMutable();
        InsertPoleColAfter(VIndex - 1, CPoles);
    }

    /// <summary>Inserts a column of weighted poles before the column VIndex.</summary>
    internal void InsertPoleColBefore(int VIndex, NCollection_Array1<gp_Pnt> CPoles, NCollection_Array1<double> CPoleWeights)
    {
        CheckMutable();
        InsertPoleColAfter(VIndex - 1, CPoles, CPoleWeights);
    }

    /// <summary>Inserts a row of poles after the row UIndex (weight 1 if the surface is rational).</summary>
    /// <exception cref="Standard_OutOfRange">UIndex out of [1, NbUPoles].</exception>
    /// <exception cref="Standard_ConstructionError">CPoles.Length() != NbVPoles.</exception>
    internal void InsertPoleRowAfter(int UIndex, NCollection_Array1<gp_Pnt> CPoles)
    {
        CheckMutable();
        if (UIndex < 1 || UIndex > myPoles.ColLength())
        {
            throw new Standard_OutOfRange();
        }
        if (CPoles.Length() != myPoles.RowLength())
        {
            throw new Standard_ConstructionError();
        }

        int NbUPoles = myPoles.ColLength();
        int NbVPoles = myPoles.RowLength();

        NCollection_Array2<gp_Pnt> npoles = new NCollection_Array2<gp_Pnt>(1, NbUPoles + 1, 1, NbVPoles);

        if (myURational || myVRational)
        {
            NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, NbUPoles + 1, 1, NbVPoles);
            NCollection_Array1<double> CWeights = new NCollection_Array1<double>(1, NbVPoles);
            CWeights.Init(1.0);
            AddRatPoleRow(myPoles, myWeights, CPoles, CWeights, UIndex, npoles, nweights);
            myWeights = nweights;
        }
        else
        {
            AddPoleRow(myPoles, CPoles, UIndex, npoles);
            myWeights = BSplSLib.UnitWeights(NbUPoles + 1, NbVPoles);
        }
        myPoles = npoles;
        myMaxDerivInvOk = false;
    }

    /// <summary>Inserts a row of weighted poles after the row UIndex.</summary>
    /// <exception cref="Standard_OutOfRange">UIndex out of [1, NbUPoles].</exception>
    /// <exception cref="Standard_ConstructionError">Bad lengths, or a weight not greater than gp::Resolution().</exception>
    internal void InsertPoleRowAfter(int UIndex, NCollection_Array1<gp_Pnt> CPoles, NCollection_Array1<double> CPoleWeights)
    {
        CheckMutable();
        if (UIndex < 1 || UIndex > myPoles.ColLength())
        {
            throw new Standard_OutOfRange();
        }
        if (CPoles.Length() != myPoles.RowLength() || CPoleWeights.Length() != CPoles.Length())
        {
            throw new Standard_ConstructionError();
        }
        int Index = CPoleWeights.Lower();
        while (Index <= CPoleWeights.Upper())
        {
            if (CPoleWeights[Index] <= gp.Resolution())
            {
                throw new Standard_ConstructionError();
            }
            Index++;
        }

        int NbUPoles = myPoles.ColLength();
        int NbVPoles = myPoles.RowLength();

        // Ensure weights are an owned copy for rational insertion
        if (!(myURational || myVRational))
        {
            myWeights = new NCollection_Array2<double>(myWeights);
        }

        NCollection_Array2<gp_Pnt> npoles = new NCollection_Array2<gp_Pnt>(1, NbUPoles + 1, 1, NbVPoles);
        NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, NbUPoles + 1, 1, NbVPoles);

        AddRatPoleRow(myPoles, myWeights, CPoles, CPoleWeights, UIndex, npoles, nweights);
        myPoles = npoles;
        myWeights = nweights;
        Rational(myWeights, out myURational, out myVRational);
        if (!(myURational || myVRational))
        {
            myWeights = BSplSLib.UnitWeights(myPoles.ColLength(), myPoles.RowLength());
        }
        myMaxDerivInvOk = false;
    }

    /// <summary>Inserts a row of poles before the row UIndex.</summary>
    internal void InsertPoleRowBefore(int UIndex, NCollection_Array1<gp_Pnt> CPoles)
    {
        CheckMutable();
        InsertPoleRowAfter(UIndex - 1, CPoles);
    }

    /// <summary>Inserts a row of weighted poles before the row UIndex.</summary>
    internal void InsertPoleRowBefore(int UIndex, NCollection_Array1<gp_Pnt> CPoles, NCollection_Array1<double> CPoleWeights)
    {
        CheckMutable();
        InsertPoleRowAfter(UIndex - 1, CPoles, CPoleWeights);
    }

    /// <summary>Removes the column of poles VIndex.</summary>
    /// <exception cref="Standard_OutOfRange">VIndex out of [1, NbVPoles].</exception>
    /// <exception cref="Standard_ConstructionError">Only 2 columns are left.</exception>
    internal void RemovePoleCol(int VIndex)
    {
        CheckMutable();
        if (VIndex < 1 || VIndex > myPoles.RowLength())
        {
            throw new Standard_OutOfRange();
        }
        if (myPoles.RowLength() <= 2)
        {
            throw new Standard_ConstructionError();
        }

        int NbUPoles = myPoles.ColLength();
        int NbVPoles = myPoles.RowLength();

        NCollection_Array2<gp_Pnt> npoles = new NCollection_Array2<gp_Pnt>(1, NbUPoles, 1, NbVPoles - 1);

        if (myURational || myVRational)
        {
            NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, NbUPoles, 1, NbVPoles - 1);
            DeleteRatPoleCol(myPoles, myWeights, VIndex, npoles, nweights);
            Rational(nweights, out myURational, out myVRational);
            if (myURational || myVRational)
            {
                myWeights = nweights;
            }
            else
            {
                myWeights = BSplSLib.UnitWeights(NbUPoles, NbVPoles - 1);
            }
        }
        else
        {
            DeletePoleCol(myPoles, VIndex, npoles);
            myWeights = BSplSLib.UnitWeights(NbUPoles, NbVPoles - 1);
        }
        myPoles = npoles;
        myMaxDerivInvOk = false;
    }

    /// <summary>Removes the row of poles UIndex.</summary>
    /// <exception cref="Standard_OutOfRange">UIndex out of [1, NbUPoles].</exception>
    /// <exception cref="Standard_ConstructionError">Only 2 rows are left.</exception>
    internal void RemovePoleRow(int UIndex)
    {
        CheckMutable();
        if (UIndex < 1 || UIndex > myPoles.ColLength())
        {
            throw new Standard_OutOfRange();
        }
        if (myPoles.ColLength() <= 2)
        {
            throw new Standard_ConstructionError();
        }

        int NbUPoles = myPoles.ColLength();
        int NbVPoles = myPoles.RowLength();

        NCollection_Array2<gp_Pnt> npoles = new NCollection_Array2<gp_Pnt>(1, NbUPoles - 1, 1, NbVPoles);

        if (myURational || myVRational)
        {
            NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, NbUPoles - 1, 1, NbVPoles);
            DeleteRatPoleRow(myPoles, myWeights, UIndex, npoles, nweights);
            Rational(nweights, out myURational, out myVRational);
            if (myURational || myVRational)
            {
                myWeights = nweights;
            }
            else
            {
                myWeights = BSplSLib.UnitWeights(NbUPoles - 1, NbVPoles);
            }
        }
        else
        {
            DeletePoleRow(myPoles, UIndex, npoles);
            myWeights = BSplSLib.UnitWeights(NbUPoles - 1, NbVPoles);
        }
        myPoles = npoles;
        myMaxDerivInvOk = false;
    }

    /// <summary>
    /// Restricts the surface to [U1, U2] x [V1, V2], reparametrised on [0, 1] x [0, 1] (U1 &gt; U2 reverses the
    /// U direction, likewise in V).
    /// </summary>
    internal void Segment(double U1, double U2, double V1, double V2)
    {
        CheckMutable();
        bool rat = (myURational || myVRational);

        int aMinDegree = UDegree() <= VDegree() ? UDegree() : VDegree();
        int aMaxDegree = UDegree() > VDegree() ? UDegree() : VDegree();

        NCollection_Array2<gp_Pnt> aCoefs = new NCollection_Array2<gp_Pnt>(1, aMaxDegree + 1, 1, aMinDegree + 1);
        NCollection_Array2<double> aWCoefs = new NCollection_Array2<double>();
        if (rat)
        {
            aWCoefs.Resize(1, aMaxDegree + 1, 1, aMinDegree + 1, false);
        }

        const double uparameter_11 = 0.5;
        const double uspanlenght_11 = 0.5;
        const double vparameter_11 = 0.5;
        const double vspanlenght_11 = 0.5;

        if (rat)
        {
            BSplSLib.BuildCache(uparameter_11,
                                vparameter_11,
                                uspanlenght_11,
                                vspanlenght_11,
                                false,
                                false,
                                UDegree(),
                                VDegree(),
                                0,
                                0,
                                InternalUKnotSequence(),
                                InternalVKnotSequence(),
                                myPoles,
                                myWeights,
                                aCoefs,
                                aWCoefs);
        }
        else
        {
            BSplSLib.BuildCache(uparameter_11,
                                vparameter_11,
                                uspanlenght_11,
                                vspanlenght_11,
                                false,
                                false,
                                UDegree(),
                                VDegree(),
                                0,
                                0,
                                InternalUKnotSequence(),
                                InternalVKnotSequence(),
                                myPoles,
                                BSplSLib.NoWeights(),
                                aCoefs,
                                BSplSLib.NoWeights());
        }

        // When udeg <= vdeg, u and v are interchanged in the coefficients,
        // so everything needs to be transposed.
        if (UDegree() <= VDegree())
        {
            NCollection_Array2<gp_Pnt> aTransposed = new NCollection_Array2<gp_Pnt>(1, UDegree() + 1, 1, VDegree() + 1);
            for (int ii = 1; ii <= UDegree() + 1; ii++)
            {
                for (int jj = 1; jj <= VDegree() + 1; jj++)
                {
                    aTransposed[ii, jj] = aCoefs[jj, ii];
                }
            }
            aCoefs = aTransposed;

            if (rat)
            {
                NCollection_Array2<double> aWTransposed = new NCollection_Array2<double>(1, UDegree() + 1, 1, VDegree() + 1);
                for (int ii = 1; ii <= UDegree() + 1; ii++)
                {
                    for (int jj = 1; jj <= VDegree() + 1; jj++)
                    {
                        aWTransposed[ii, jj] = aWCoefs[jj, ii];
                    }
                }
                aWCoefs = aWTransposed;
            }
        }

        // Trim in the canonical basis and update Poles and Coefficients.
        double ufirst = 2 * (U1 - 0.5), ulast = 2 * (U2 - 0.5), vfirst = 2 * (V1 - 0.5), vlast = 2 * (V2 - 0.5);
        if (rat)
        {
            PLib.UTrimming(ufirst, ulast, aCoefs, aWCoefs);
            PLib.VTrimming(vfirst, vlast, aCoefs, aWCoefs);
            PLib.CoefficientsPoles(aCoefs, aWCoefs, myPoles, myWeights);
        }
        else
        {
            PLib.UTrimming(ufirst, ulast, aCoefs, PLib.NoWeights2());
            PLib.VTrimming(vfirst, vlast, aCoefs, PLib.NoWeights2());
            PLib.CoefficientsPoles(aCoefs, PLib.NoWeights2(), myPoles, PLib.NoWeights2());
        }

        myMaxDerivInvOk = false;
    }

    /// <summary>Changes the pole (UIndex, VIndex).</summary>
    /// <exception cref="Standard_OutOfRange">An index out of the pole grid.</exception>
    internal void SetPole(int UIndex, int VIndex, in gp_Pnt P)
    {
        CheckMutable();
        if (UIndex < 1 || UIndex > myPoles.ColLength() || VIndex < 1 || VIndex > myPoles.RowLength())
        {
            throw new Standard_OutOfRange();
        }
        // OCCT: keeps the evaluation representation, as every edit but Transform, UReverse and VReverse does
        // (Geom_BSplineSurface clears it): the surface goes on evaluating the old shape (occt-issues.md)
        myPoles[UIndex, VIndex] = P;
        myMaxDerivInvOk = false;
    }

    /// <summary>Changes the pole (UIndex, VIndex) and its weight.</summary>
    /// <exception cref="Standard_ConstructionError">Weight not greater than gp::Resolution().</exception>
    /// <exception cref="Standard_OutOfRange">An index out of the pole grid.</exception>
    internal void SetPole(int UIndex, int VIndex, in gp_Pnt P, double Weight)
    {
        CheckMutable();
        if (Weight <= gp.Resolution())
        {
            throw new Standard_ConstructionError("Geom_BezierSurface::SetPole");
        }
        if (UIndex < 1 || UIndex > myPoles.ColLength() || VIndex < 1 || VIndex > myPoles.RowLength())
        {
            throw new Standard_OutOfRange("Geom_BezierSurface::SetPole");
        }

        myPoles[UIndex, VIndex] = P;

        SetWeight(UIndex, VIndex, Weight);
    }

    /// <summary>Changes the poles and weights of the column VIndex.</summary>
    internal void SetPoleCol(int VIndex, NCollection_Array1<gp_Pnt> CPoles, NCollection_Array1<double> CPoleWeights)
    {
        CheckMutable();
        if (VIndex < 1 || VIndex > myPoles.RowLength())
        {
            throw new Standard_OutOfRange();
        }

        if (CPoles.Lower() < 1 || CPoles.Lower() > myPoles.ColLength() || CPoles.Upper() < 1
            || CPoles.Upper() > myPoles.ColLength() || CPoleWeights.Lower() != CPoles.Lower()
            || CPoleWeights.Upper() != CPoles.Upper())
        {
            throw new Standard_ConstructionError();
        }

        int I;
        for (I = CPoles.Lower(); I <= CPoles.Upper(); I++)
        {
            myPoles[I, VIndex] = CPoles[I];
        }
        SetWeightCol(VIndex, CPoleWeights);
    }

    /// <summary>Changes the poles of the column VIndex (rows CPoles.Lower() to Upper()).</summary>
    internal void SetPoleCol(int VIndex, NCollection_Array1<gp_Pnt> CPoles)
    {
        CheckMutable();
        if (VIndex < 1 || VIndex > myPoles.RowLength())
        {
            throw new Standard_OutOfRange();
        }

        if (CPoles.Lower() < 1 || CPoles.Lower() > myPoles.ColLength() || CPoles.Upper() < 1
            || CPoles.Upper() > myPoles.ColLength())
        {
            throw new Standard_ConstructionError();
        }

        for (int I = CPoles.Lower(); I <= CPoles.Upper(); I++)
        {
            myPoles[I, VIndex] = CPoles[I];
        }
        myMaxDerivInvOk = false;
    }

    /// <summary>Changes the poles of the row UIndex (columns CPoles.Lower() to Upper()).</summary>
    internal void SetPoleRow(int UIndex, NCollection_Array1<gp_Pnt> CPoles)
    {
        CheckMutable();
        if (UIndex < 1 || UIndex > myPoles.ColLength())
        {
            throw new Standard_OutOfRange();
        }
        if (CPoles.Lower() < 1 || CPoles.Lower() > myPoles.RowLength() || CPoles.Upper() < 1
            || CPoles.Upper() > myPoles.RowLength())
        {
            throw new Standard_ConstructionError();
        }

        for (int I = CPoles.Lower(); I <= CPoles.Upper(); I++)
        {
            myPoles[UIndex, I] = CPoles[I];
        }
        myMaxDerivInvOk = false;
    }

    /// <summary>Changes the poles and weights of the row UIndex.</summary>
    internal void SetPoleRow(int UIndex, NCollection_Array1<gp_Pnt> CPoles, NCollection_Array1<double> CPoleWeights)
    {
        CheckMutable();
        if (UIndex < 1 || UIndex > myPoles.ColLength())
        {
            throw new Standard_OutOfRange();
        }
        if (CPoles.Lower() < 1 || CPoles.Lower() > myPoles.RowLength() || CPoles.Upper() < 1
            || CPoles.Upper() > myPoles.RowLength() || CPoleWeights.Lower() != CPoles.Lower()
            || CPoleWeights.Upper() != CPoles.Upper())
        {
            throw new Standard_ConstructionError();
        }

        int I;

        for (I = CPoles.Lower(); I <= CPoles.Upper(); I++)
        {
            myPoles[UIndex, I] = CPoles[I];
        }

        SetWeightRow(UIndex, CPoleWeights);
    }

    /// <summary>Changes the weight of the pole (UIndex, VIndex); the rationality flags are recomputed.</summary>
    /// <exception cref="Standard_ConstructionError">Weight not greater than gp::Resolution().</exception>
    /// <exception cref="Standard_OutOfRange">An index out of the pole grid.</exception>
    internal void SetWeight(int UIndex, int VIndex, double Weight)
    {
        CheckMutable();
        if (Weight <= gp.Resolution())
        {
            throw new Standard_ConstructionError("Geom_BezierSurface::SetWeight");
        }

        if (UIndex < 1 || UIndex > myPoles.ColLength() || VIndex < 1 || VIndex > myPoles.RowLength())
        {
            throw new Standard_OutOfRange();
        }

        // compute new rationality
        bool wasrat = (myURational || myVRational);
        if (!wasrat)
        {
            // a weight of 1. does not turn to rational
            if (Math.Abs(Weight - 1.0) <= gp.Resolution())
            {
                return;
            }

            // owned copy from non-owning view
            myWeights = new NCollection_Array2<double>(myWeights);
        }

        if (Math.Abs(Weight - myWeights[UIndex, VIndex]) > gp.Resolution())
        {
            myWeights[UIndex, VIndex] = Weight;
            Rational(myWeights, out myURational, out myVRational);
            if (!(myURational || myVRational))
            {
                myWeights = BSplSLib.UnitWeights(myPoles.ColLength(), myPoles.RowLength());
            }
        }

        myMaxDerivInvOk = false;
    }

    /// <summary>Changes the weights of the column VIndex.</summary>
    internal void SetWeightCol(int VIndex, NCollection_Array1<double> CPoleWeights)
    {
        CheckMutable();
        if (VIndex < 1 || VIndex > myPoles.RowLength())
        {
            throw new Standard_OutOfRange();
        }

        if (CPoleWeights.Length() != myPoles.ColLength())
        {
            throw new Standard_ConstructionError("Geom_BezierSurface::SetWeightCol");
        }

        int I;

        // compute new rationality
        bool wasrat = (myURational || myVRational);
        if (!wasrat)
        {
            // owned copy from non-owning view
            myWeights = new NCollection_Array2<double>(myWeights);
        }

        I = CPoleWeights.Lower();
        while (I <= CPoleWeights.Upper())
        {
            if (CPoleWeights[I] <= gp.Resolution())
            {
                throw new Standard_ConstructionError();
            }
            myWeights[I, VIndex] = CPoleWeights[I];
            I++;
        }

        Rational(myWeights, out myURational, out myVRational);
        if (!(myURational || myVRational))
        {
            myWeights = BSplSLib.UnitWeights(myPoles.ColLength(), myPoles.RowLength());
        }

        myMaxDerivInvOk = false;
    }

    /// <summary>Changes the weights of the row UIndex.</summary>
    internal void SetWeightRow(int UIndex, NCollection_Array1<double> CPoleWeights)
    {
        CheckMutable();
        if (UIndex < 1 || UIndex > myPoles.ColLength())
        {
            throw new Standard_OutOfRange("Geom_BezierSurface::SetWeightRow");
        }
        if (CPoleWeights.Lower() < 1 || CPoleWeights.Lower() > myPoles.RowLength() || CPoleWeights.Upper() < 1
            || CPoleWeights.Upper() > myPoles.RowLength())
        {
            throw new Standard_ConstructionError("Geom_BezierSurface::SetWeightRow");
        }

        int I;

        // compute new rationality
        bool wasrat = (myURational || myVRational);
        if (!wasrat)
        {
            // owned copy from non-owning view
            myWeights = new NCollection_Array2<double>(myWeights);
        }

        I = CPoleWeights.Lower();
        while (I <= CPoleWeights.Upper())
        {
            if (CPoleWeights[I] <= gp.Resolution())
            {
                throw new Standard_ConstructionError();
            }
            myWeights[UIndex, I] = CPoleWeights[I];
            I++;
        }

        Rational(myWeights, out myURational, out myVRational);
        if (!(myURational || myVRational))
        {
            myWeights = BSplSLib.UnitWeights(myPoles.ColLength(), myPoles.RowLength());
        }

        myMaxDerivInvOk = false;
    }

    /// <summary>Reverses the U direction (the rows of the pole grid).</summary>
    protected internal sealed override void UReverse()
    {
        CheckMutable();
        gp_Pnt Pol;
        int Row, Col;
        int NbUPoles = myPoles.ColLength();
        int NbVPoles = myPoles.RowLength();

        if (myURational || myVRational)
        {
            double W;
            for (Col = 1; Col <= NbVPoles; Col++)
            {
                for (Row = 1; Row <= NbUPoles / 2; Row++)
                {
                    W = myWeights[Row, Col];
                    myWeights[Row, Col] = myWeights[NbUPoles - Row + 1, Col];
                    myWeights[NbUPoles - Row + 1, Col] = W;
                    Pol = myPoles[Row, Col];
                    myPoles[Row, Col] = myPoles[NbUPoles - Row + 1, Col];
                    myPoles[NbUPoles - Row + 1, Col] = Pol;
                }
            }
        }
        else
        {
            for (Col = 1; Col <= NbVPoles; Col++)
            {
                for (Row = 1; Row <= NbUPoles / 2; Row++)
                {
                    Pol = myPoles[Row, Col];
                    myPoles[Row, Col] = myPoles[NbUPoles - Row + 1, Col];
                    myPoles[NbUPoles - Row + 1, Col] = Pol;
                }
            }
        }
        ClearEvalRepresentation();
        myMaxDerivInvOk = false;
    }

    /// <summary>1 - U.</summary>
    public sealed override double UReversedParameter(double U) => (1.0 - U);

    /// <summary>Reverses the V direction (the columns of the pole grid).</summary>
    protected internal sealed override void VReverse()
    {
        CheckMutable();
        gp_Pnt Pol;
        int Row, Col;
        int NbUPoles = myPoles.ColLength();
        int NbVPoles = myPoles.RowLength();

        if (myURational || myVRational)
        {
            double W;
            for (Row = 1; Row <= NbUPoles; Row++)
            {
                for (Col = 1; Col <= NbVPoles / 2; Col++)
                {
                    W = myWeights[Row, Col];
                    myWeights[Row, Col] = myWeights[Row, NbVPoles - Col + 1];
                    myWeights[Row, NbVPoles - Col + 1] = W;
                    Pol = myPoles[Row, Col];
                    myPoles[Row, Col] = myPoles[Row, NbVPoles - Col + 1];
                    myPoles[Row, NbVPoles - Col + 1] = Pol;
                }
            }
        }
        else
        {
            for (Row = 1; Row <= NbUPoles; Row++)
            {
                for (Col = 1; Col <= NbVPoles / 2; Col++)
                {
                    Pol = myPoles[Row, Col];
                    myPoles[Row, Col] = myPoles[Row, NbVPoles - Col + 1];
                    myPoles[Row, NbVPoles - Col + 1] = Pol;
                }
            }
        }
        ClearEvalRepresentation();
        myMaxDerivInvOk = false;
    }

    /// <summary>1 - V.</summary>
    public sealed override double VReversedParameter(double V) => (1.0 - V);

    /// <summary>The parametric bounds: [0, 1] x [0, 1].</summary>
    public sealed override void Bounds(out double U1, out double U2, out double V1, out double V2)
    {
        U1 = 0.0;
        U2 = 1.0;
        V1 = 0.0;
        V2 = 1.0;
    }

    /// <summary>CN.</summary>
    public sealed override GeomAbs_Shape Continuity() => GeomAbs_CN;

    /// <summary>The point of parameters (U, V).</summary>
    public sealed override gp_Pnt EvalD0(double U, double V)
    {
        gp_Pnt aEvalRepResult;
        if (GeomEval_RepUtils.TryEvalSurfaceD0(myEvalRep, U, V, out aEvalRepResult))
        {
            return aEvalRepResult;
        }

        gp_Pnt P;
        if (myURational || myVRational)
        {
            BSplSLib.D0(U,
                        V,
                        1,
                        1,
                        myPoles,
                        myWeights,
                        InternalUKnots(),
                        InternalUKnots(),
                        InternalUMultiplicities(),
                        InternalVMultiplicities(),
                        (myPoles.ColLength() - 1),
                        (myPoles.RowLength() - 1),
                        myURational,
                        myVRational,
                        false,
                        false,
                        out P);
        }
        else
        {
            BSplSLib.D0(U,
                        V,
                        1,
                        1,
                        myPoles,
                        BSplSLib.NoWeights(),
                        InternalUKnots(),
                        InternalUKnots(),
                        InternalUMultiplicities(),
                        InternalVMultiplicities(),
                        (myPoles.ColLength() - 1),
                        (myPoles.RowLength() - 1),
                        myURational,
                        myVRational,
                        false,
                        false,
                        out P);
        }
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

        ResD1 aResult;
        if (myURational || myVRational)
        {
            BSplSLib.D1(U,
                        V,
                        1,
                        1,
                        myPoles,
                        myWeights,
                        InternalUKnots(),
                        InternalUKnots(),
                        InternalUMultiplicities(),
                        InternalVMultiplicities(),
                        (myPoles.ColLength() - 1),
                        (myPoles.RowLength() - 1),
                        myURational,
                        myVRational,
                        false,
                        false,
                        out aResult.Point,
                        out aResult.D1U,
                        out aResult.D1V);
        }
        else
        {
            BSplSLib.D1(U,
                        V,
                        1,
                        1,
                        myPoles,
                        BSplSLib.NoWeights(),
                        InternalUKnots(),
                        InternalUKnots(),
                        InternalUMultiplicities(),
                        InternalVMultiplicities(),
                        (myPoles.ColLength() - 1),
                        (myPoles.RowLength() - 1),
                        myURational,
                        myVRational,
                        false,
                        false,
                        out aResult.Point,
                        out aResult.D1U,
                        out aResult.D1V);
        }
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

        ResD2 aResult;
        if (myURational || myVRational)
        {
            //-- WARNING: pay attention to the ORDER of arguments in BSPLSLIB
            BSplSLib.D2(U,
                        V,
                        1,
                        1,
                        myPoles,
                        myWeights,
                        InternalUKnots(),
                        InternalUKnots(),
                        InternalUMultiplicities(),
                        InternalVMultiplicities(),
                        (myPoles.ColLength() - 1),
                        (myPoles.RowLength() - 1),
                        myURational,
                        myVRational,
                        false,
                        false,
                        out aResult.Point,
                        out aResult.D1U,
                        out aResult.D1V,
                        out aResult.D2U,
                        out aResult.D2V,
                        out aResult.D2UV);
        }
        else
        {
            //-- WARNING: pay attention to the ORDER of arguments in BSPLSLIB
            BSplSLib.D2(U,
                        V,
                        1,
                        1,
                        myPoles,
                        BSplSLib.NoWeights(),
                        InternalUKnots(),
                        InternalUKnots(),
                        InternalUMultiplicities(),
                        InternalVMultiplicities(),
                        (myPoles.ColLength() - 1),
                        (myPoles.RowLength() - 1),
                        myURational,
                        myVRational,
                        false,
                        false,
                        out aResult.Point,
                        out aResult.D1U,
                        out aResult.D1V,
                        out aResult.D2U,
                        out aResult.D2V,
                        out aResult.D2UV);
        }
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
        if (myURational || myVRational)
        {
            BSplSLib.D3(U,
                        V,
                        0,
                        0,
                        myPoles,
                        myWeights,
                        InternalUKnots(),
                        InternalUKnots(),
                        InternalUMultiplicities(),
                        InternalVMultiplicities(),
                        (myPoles.ColLength() - 1),
                        (myPoles.RowLength() - 1),
                        myURational,
                        myVRational,
                        false,
                        false,
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
        }
        else
        {
            BSplSLib.D3(U,
                        V,
                        0,
                        0,
                        myPoles,
                        BSplSLib.NoWeights(),
                        InternalUKnots(),
                        InternalUKnots(),
                        InternalUMultiplicities(),
                        InternalVMultiplicities(),
                        (myPoles.ColLength() - 1),
                        (myPoles.RowLength() - 1),
                        myURational,
                        myVRational,
                        false,
                        false,
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
        }
        return aResult;
    }

    /// <summary>The derivative of order Nu in U and Nv in V at (U, V).</summary>
    /// <exception cref="Geom_UndefinedDerivative">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    public sealed override gp_Vec EvalDN(double U, double V, int Nu, int Nv)
    {
        gp_Vec aEvalRepResult;
        if (GeomEval_RepUtils.TryEvalSurfaceDN(myEvalRep, U, V, Nu, Nv, out aEvalRepResult))
        {
            return aEvalRepResult;
        }

        if (Nu + Nv < 1 || Nu < 0 || Nv < 0)
        {
            throw new Geom_UndefinedDerivative();
        }

        gp_Vec Derivative;
        if (myURational || myVRational)
        {
            BSplSLib.DN(U,
                        V,
                        Nu,
                        Nv,
                        0,
                        0,
                        myPoles,
                        myWeights,
                        InternalUKnots(),
                        InternalUKnots(),
                        InternalUMultiplicities(),
                        InternalVMultiplicities(),
                        (myPoles.ColLength() - 1),
                        (myPoles.RowLength() - 1),
                        myURational,
                        myVRational,
                        false,
                        false,
                        out Derivative);
        }
        else
        {
            BSplSLib.DN(U,
                        V,
                        Nu,
                        Nv,
                        0,
                        0,
                        myPoles,
                        BSplSLib.NoWeights(),
                        InternalUKnots(),
                        InternalUKnots(),
                        InternalUMultiplicities(),
                        InternalVMultiplicities(),
                        (myPoles.ColLength() - 1),
                        (myPoles.RowLength() - 1),
                        myURational,
                        myVRational,
                        false,
                        false,
                        out Derivative);
        }
        return Derivative;
    }

    /// <summary>The number of poles in U (rows of the pole grid).</summary>
    public int NbUPoles() => myPoles.ColLength();

    /// <summary>The number of poles in V (columns of the pole grid).</summary>
    public int NbVPoles() => myPoles.RowLength();

    /// <summary>The pole (UIndex, VIndex).</summary>
    /// <exception cref="Standard_OutOfRange">An index out of the pole grid.</exception>
    public gp_Pnt Pole(int UIndex, int VIndex)
    {
        Standard_OutOfRange_Raise_if(UIndex < 1 || UIndex > myPoles.ColLength() || VIndex < 1
                                         || VIndex > myPoles.RowLength(),
                                     " ");
        return myPoles[UIndex, VIndex];
    }

    /// <summary>Copies the poles into P (deprecated in OCCT: use <see cref="Poles()"/>).</summary>
    /// <exception cref="Standard_DimensionError">P is not NbUPoles x NbVPoles.</exception>
    public void Poles(NCollection_Array2<gp_Pnt> P)
    {
        Standard_DimensionError_Raise_if(P.RowLength() != myPoles.RowLength() || P.ColLength() != myPoles.ColLength(),
                                         " ");
        P.Assign(myPoles);
    }

    /// <summary>A copy of the poles, NbUPoles rows by NbVPoles columns, indexed from 1.</summary>
    public NCollection_Array2<gp_Pnt> Poles() => new NCollection_Array2<gp_Pnt>(myPoles);

    /// <summary>The degree in U: NbUPoles - 1.</summary>
    public int UDegree() => (myPoles.ColLength() - 1);

    /// <summary>The U iso-curve (a Bezier curve in V) at U.</summary>
    /// <remarks>Not available yet: needs the port of Geom_BezierCurve (unit 1A). Throws Standard_NotImplemented.</remarks>
    public sealed override Geom_Curve UIso(double U)
    {
        NCollection_Array1<gp_Pnt> VCurvePoles = new NCollection_Array1<gp_Pnt>(1, myPoles.RowLength());
        if (myURational || myVRational)
        {
            NCollection_Array1<double> VCurveWeights = new NCollection_Array1<double>(1, myPoles.RowLength());
            BSplSLib.Iso(U,
                         true,
                         myPoles,
                         myWeights,
                         InternalUKnots(),
                         InternalUMultiplicities(),
                         (myPoles.ColLength() - 1),
                         false,
                         VCurvePoles,
                         VCurveWeights);
            // TODO(1A): if (myURational) UIsoCurve = new Geom_BezierCurve(VCurvePoles, VCurveWeights);
            //           else UIsoCurve = new Geom_BezierCurve(VCurvePoles);
        }
        else
        {
            BSplSLib.Iso(U,
                         true,
                         myPoles,
                         BSplSLib.NoWeights(),
                         InternalUKnots(),
                         InternalUMultiplicities(),
                         (myPoles.ColLength() - 1),
                         false,
                         VCurvePoles,
                         PLib.NoWeights());
            // TODO(1A): UIsoCurve = new Geom_BezierCurve(VCurvePoles);
        }
        throw new Standard_NotImplemented("Geom_BezierSurface::UIso: Geom_BezierCurve is not ported yet");
    }

    /// <summary>The degree in V: NbVPoles - 1.</summary>
    public int VDegree() => (myPoles.RowLength() - 1);

    /// <summary>The V iso-curve (a Bezier curve in U) at V.</summary>
    /// <remarks>Not available yet: needs the port of Geom_BezierCurve (unit 1A). Throws Standard_NotImplemented.</remarks>
    public sealed override Geom_Curve VIso(double V)
    {
        NCollection_Array1<gp_Pnt> VCurvePoles = new NCollection_Array1<gp_Pnt>(1, myPoles.ColLength());
        if (myVRational || myURational)
        {
            NCollection_Array1<double> VCurveWeights = new NCollection_Array1<double>(1, myPoles.ColLength());
            BSplSLib.Iso(V,
                         false,
                         myPoles,
                         myWeights,
                         InternalUKnots(),
                         InternalVMultiplicities(),
                         (myPoles.RowLength() - 1),
                         false,
                         VCurvePoles,
                         VCurveWeights);
            // TODO(1A): if (myVRational) VIsoCurve = new Geom_BezierCurve(VCurvePoles, VCurveWeights);
            //           else VIsoCurve = new Geom_BezierCurve(VCurvePoles);
        }
        else
        {
            BSplSLib.Iso(V,
                         false,
                         myPoles,
                         BSplSLib.NoWeights(),
                         InternalUKnots(),
                         InternalVMultiplicities(),
                         (myPoles.RowLength() - 1),
                         false,
                         VCurvePoles,
                         PLib.NoWeights());
            // TODO(1A): VIsoCurve = new Geom_BezierCurve(VCurvePoles);
        }
        throw new Standard_NotImplemented("Geom_BezierSurface::VIso: Geom_BezierCurve is not ported yet");
    }

    /// <summary>The weight of the pole (UIndex, VIndex): 1 for a non-rational surface.</summary>
    /// <exception cref="Standard_OutOfRange">An index out of the pole grid.</exception>
    public double Weight(int UIndex, int VIndex)
    {
        Standard_OutOfRange_Raise_if(UIndex < 1 || UIndex > myPoles.ColLength() || VIndex < 1
                                         || VIndex > myPoles.RowLength(),
                                     " ");
        if (myURational || myVRational)
        {
            return myWeights[UIndex, VIndex];
        }
        else
        {
            return 1;
        }
    }

    /// <summary>Copies the weights into W, all 1 for a non-rational surface (deprecated in OCCT).</summary>
    /// <exception cref="Standard_DimensionError">W is not NbUPoles x NbVPoles.</exception>
    public void Weights(NCollection_Array2<double> W)
    {
        Standard_DimensionError_Raise_if(W.RowLength() != myPoles.RowLength() || W.ColLength() != myPoles.ColLength(),
                                         " ");
        if (myURational || myVRational)
        {
            W.Assign(myWeights);
        }
        else
        {
            W.Init(1.0);
        }
    }

    /// <summary>A copy of the weights, or null if the surface is non-rational in both directions.</summary>
    public NCollection_Array2<double>? Weights() =>
        (myURational || myVRational) ? new NCollection_Array2<double>(myWeights) : null;

    /// <summary>A copy of the weights, all 1 for a non-rational surface.</summary>
    public NCollection_Array2<double> WeightsArray() => new NCollection_Array2<double>(myWeights);

    /// <summary>True.</summary>
    public sealed override bool IsCNu(int N) => true;

    /// <summary>True.</summary>
    public sealed override bool IsCNv(int N) => true;

    /// <summary>True if the weights vary in the U direction.</summary>
    public bool IsURational() => myURational;

    /// <summary>True if the weights vary in the V direction.</summary>
    public bool IsVRational() => myVRational;

    /// <summary>Transforms the poles by T.</summary>
    protected internal sealed override void Transform(in gp_Trsf T)
    {
        CheckMutable();
        for (int I = 1; I <= myPoles.ColLength(); I++)
        {
            for (int J = 1; J <= myPoles.RowLength(); J++)
            {
                myPoles[I, J].Transform(T);
            }
        }
        ClearEvalRepresentation();
        myMaxDerivInvOk = false;
    }

    /// <summary>True if the first and last rows of poles coincide (within Precision::Confusion()).</summary>
    public sealed override bool IsUClosed()
    {
        bool Closed = true;
        int Lower = myPoles.LowerRow();
        int Upper = myPoles.UpperRow();
        int j = myPoles.LowerCol();
        while (Closed && j <= myPoles.UpperCol())
        {
            Closed = (myPoles[Lower, j].Distance(myPoles[Upper, j]) <= Precision.Confusion());
            j++;
        }
        return Closed;
    }

    /// <summary>True if the first and last columns of poles coincide (within Precision::Confusion()).</summary>
    public sealed override bool IsVClosed()
    {
        bool Closed = true;
        int Lower = myPoles.LowerCol();
        int Upper = myPoles.UpperCol();
        int i = myPoles.LowerRow();
        while (Closed && i <= myPoles.UpperRow())
        {
            Closed = (myPoles[i, Lower].Distance(myPoles[i, Upper]) <= Precision.Confusion());
            i++;
        }
        return Closed;
    }

    /// <summary>False.</summary>
    public sealed override bool IsUPeriodic() => false;

    /// <summary>False.</summary>
    public sealed override bool IsVPeriodic() => false;

    /// <summary>
    /// The parametric tolerances in U and V matching the 3D tolerance Tolerance3D (computed once and cached, as in
    /// OCCT; the cache is not part of the surface's value, so this works on a frozen surface).
    /// </summary>
    public void Resolution(double Tolerance3D, out double UTolerance, out double VTolerance)
    {
        if (!myMaxDerivInvOk)
        {
            double aUMaxDerivInv, aVMaxDerivInv;
            if (myURational || myVRational)
            {
                BSplSLib.Resolution(myPoles,
                                    myWeights,
                                    InternalUKnots(),
                                    InternalUKnots(),
                                    InternalUMultiplicities(),
                                    InternalVMultiplicities(),
                                    (myPoles.ColLength() - 1),
                                    (myPoles.RowLength() - 1),
                                    myURational,
                                    myVRational,
                                    false,
                                    false,
                                    1.0,
                                    out aUMaxDerivInv,
                                    out aVMaxDerivInv);
            }
            else
            {
                BSplSLib.Resolution(myPoles,
                                    BSplSLib.NoWeights(),
                                    InternalUKnots(),
                                    InternalUKnots(),
                                    InternalUMultiplicities(),
                                    InternalVMultiplicities(),
                                    (myPoles.ColLength() - 1),
                                    (myPoles.RowLength() - 1),
                                    myURational,
                                    myVRational,
                                    false,
                                    false,
                                    1.0,
                                    out aUMaxDerivInv,
                                    out aVMaxDerivInv);
            }
            myUMaxDerivInv = aUMaxDerivInv;
            myVMaxDerivInv = aVMaxDerivInv;
            myMaxDerivInvOk = true;
        }
        UTolerance = Tolerance3D * myUMaxDerivInv;
        VTolerance = Tolerance3D * myVMaxDerivInv;
    }

    /// <summary>A new, unfrozen surface equal to this one.</summary>
    public sealed override Geom_Geometry Copy() => new Geom_BezierSurface(this);

    /// <summary>Sets the poles and weights (null: non-rational) of a new surface.</summary>
    protected void init(NCollection_Array2<gp_Pnt> thePoles, NCollection_Array2<double>? theWeights)
    {
        CheckMutable();
        int NbUPoles = thePoles.ColLength();
        int NbVPoles = thePoles.RowLength();

        myPoles.Resize(1, NbUPoles, 1, NbVPoles, false);
        myPoles.CopyValues(thePoles);

        if (theWeights != null)
        {
            myWeights.Resize(1, NbUPoles, 1, NbVPoles, false);
            myWeights.CopyValues(theWeights);
            Rational(myWeights, out myURational, out myVRational);
            if (!(myURational || myVRational))
            {
                myWeights = BSplSLib.UnitWeights(NbUPoles, NbVPoles);
            }
        }
        else
        {
            myURational = false;
            myVRational = false;
            myWeights = BSplSLib.UnitWeights(NbUPoles, NbVPoles);
        }

        myMaxDerivInvOk = false;
        ClearEvalRepresentation();
    }

    //=============================================================================================
    // The knots of the equivalent B-spline surface: OCCT's static arrays, shared per degree
    //=============================================================================================

    private static readonly NCollection_Array1<double> THE_KNOTS = NCollection_Array1<double>.From([0.0, 1.0]);

    private static readonly NCollection_Array1<int>[] THE_MULTS = CreateMults();

    private static readonly NCollection_Array1<double>?[] THE_FKNOTS = CreateFlatKnots();

    private static NCollection_Array1<int>[] CreateMults()
    {
        int aMaxSize = BSplCLib.MaxDegree() + 1;
        NCollection_Array1<int>[] anArr = new NCollection_Array1<int>[aMaxSize];
        for (int i = 0; i < aMaxSize; ++i)
        {
            anArr[i] = NCollection_Array1<int>.From([i + 1, i + 1]);
        }
        return anArr;
    }

    private static NCollection_Array1<double>?[] CreateFlatKnots()
    {
        int aMaxSize = BSplCLib.MaxDegree() + 1;
        NCollection_Array1<double>?[] anArr = new NCollection_Array1<double>?[aMaxSize];
        for (int i = 1; i <= BSplCLib.MaxDegree(); ++i)
        {
            anArr[i] = NCollection_Array1<double>.From(BSplCLib.FlatBezierKnots(i));
        }
        return anArr;
    }

    /// <summary>The U knots (0, 1): a shared static array, don't modify.</summary>
    internal NCollection_Array1<double> InternalUKnots() => THE_KNOTS;

    /// <summary>The V knots (0, 1): a shared static array, don't modify.</summary>
    internal NCollection_Array1<double> InternalVKnots() => InternalUKnots();

    /// <summary>The U multiplicities (UDegree + 1 twice): a shared static array, don't modify.</summary>
    internal NCollection_Array1<int> InternalUMultiplicities()
    {
        Standard_ProgramError_Raise_if(myPoles.IsEmpty(), "Geom_BezierSurface: empty poles");
        return THE_MULTS[myPoles.ColLength() - 1];
    }

    /// <summary>The V multiplicities (VDegree + 1 twice): a shared static array, don't modify.</summary>
    internal NCollection_Array1<int> InternalVMultiplicities()
    {
        Standard_ProgramError_Raise_if(myPoles.IsEmpty(), "Geom_BezierSurface: empty poles");
        return THE_MULTS[myPoles.RowLength() - 1];
    }

    /// <summary>The U flat knots (UDegree + 1 zeros and ones): a shared static array, don't modify.</summary>
    internal NCollection_Array1<double> InternalUKnotSequence()
    {
        Standard_ProgramError_Raise_if(myPoles.IsEmpty(), "Geom_BezierSurface: empty poles");
        return THE_FKNOTS[myPoles.ColLength() - 1]!;
    }

    /// <summary>The V flat knots (VDegree + 1 zeros and ones): a shared static array, don't modify.</summary>
    internal NCollection_Array1<double> InternalVKnotSequence()
    {
        Standard_ProgramError_Raise_if(myPoles.IsEmpty(), "Geom_BezierSurface: empty poles");
        return THE_FKNOTS[myPoles.RowLength() - 1]!;
    }

    /// <summary>The U knots of the equivalent B-spline surface: (0, 1).</summary>
    public PackedList<double> UKnots() => InternalUKnots().ToPackedList();

    /// <summary>The V knots of the equivalent B-spline surface: (0, 1).</summary>
    public PackedList<double> VKnots() => InternalVKnots().ToPackedList();

    /// <summary>The U multiplicities of the equivalent B-spline surface: (UDegree + 1, UDegree + 1).</summary>
    public PackedList<int> UMultiplicities() => InternalUMultiplicities().ToPackedList();

    /// <summary>The V multiplicities of the equivalent B-spline surface: (VDegree + 1, VDegree + 1).</summary>
    public PackedList<int> VMultiplicities() => InternalVMultiplicities().ToPackedList();

    /// <summary>The U flat knots of the equivalent B-spline surface.</summary>
    public PackedList<double> UKnotSequence() => InternalUKnotSequence().ToPackedList();

    /// <summary>The V flat knots of the equivalent B-spline surface.</summary>
    public PackedList<double> VKnotSequence() => InternalVKnotSequence().ToPackedList();

    //=============================================================================================
    // Internal access to the stored arrays and non-mutating twins (OCCT.NET)
    //=============================================================================================

    /// <summary>The stored poles: shared, don't modify (OCCT's <c>Poles()</c> const reference).</summary>
    internal NCollection_Array2<gp_Pnt> InternalPoles() => myPoles;

    /// <summary>The stored weights, or null for a non-rational surface (OCCT's <c>Weights()</c> pointer). Don't modify.</summary>
    internal NCollection_Array2<double>? InternalWeights() => (myURational || myVRational) ? myWeights : BSplSLib.NoWeights();

    /// <summary>The weights, NbUPoles x NbVPoles of them (OCCT's <c>WeightsArray()</c>): shared, don't modify.</summary>
    internal NCollection_Array2<double> InternalWeightsArray() => myWeights;

    private Geom_BezierSurface CopyBezier() => (Geom_BezierSurface)Copy();

    /// <summary>A copy with U and V exchanged.</summary>
    public Geom_BezierSurface ExchangedUV()
    {
        Geom_BezierSurface aCopy = CopyBezier();
        aCopy.ExchangeUV();
        return aCopy;
    }

    /// <summary>A copy with the degrees raised to UDeg and VDeg (see <see cref="Increase"/>).</summary>
    public Geom_BezierSurface Increased(int UDeg, int VDeg)
    {
        Geom_BezierSurface aCopy = CopyBezier();
        aCopy.Increase(UDeg, VDeg);
        return aCopy;
    }

    /// <summary>A copy with a column of poles inserted after VIndex.</summary>
    public Geom_BezierSurface PoleColInsertedAfter(int VIndex, NCollection_Array1<gp_Pnt> CPoles)
    {
        Geom_BezierSurface aCopy = CopyBezier();
        aCopy.InsertPoleColAfter(VIndex, CPoles);
        return aCopy;
    }

    /// <summary>A copy with a column of weighted poles inserted after VIndex.</summary>
    public Geom_BezierSurface PoleColInsertedAfter(int VIndex, NCollection_Array1<gp_Pnt> CPoles, NCollection_Array1<double> CPoleWeights)
    {
        Geom_BezierSurface aCopy = CopyBezier();
        aCopy.InsertPoleColAfter(VIndex, CPoles, CPoleWeights);
        return aCopy;
    }

    /// <summary>A copy with a column of poles inserted before VIndex.</summary>
    public Geom_BezierSurface PoleColInsertedBefore(int VIndex, NCollection_Array1<gp_Pnt> CPoles)
    {
        Geom_BezierSurface aCopy = CopyBezier();
        aCopy.InsertPoleColBefore(VIndex, CPoles);
        return aCopy;
    }

    /// <summary>A copy with a column of weighted poles inserted before VIndex.</summary>
    public Geom_BezierSurface PoleColInsertedBefore(int VIndex, NCollection_Array1<gp_Pnt> CPoles, NCollection_Array1<double> CPoleWeights)
    {
        Geom_BezierSurface aCopy = CopyBezier();
        aCopy.InsertPoleColBefore(VIndex, CPoles, CPoleWeights);
        return aCopy;
    }

    /// <summary>A copy with a row of poles inserted after UIndex.</summary>
    public Geom_BezierSurface PoleRowInsertedAfter(int UIndex, NCollection_Array1<gp_Pnt> CPoles)
    {
        Geom_BezierSurface aCopy = CopyBezier();
        aCopy.InsertPoleRowAfter(UIndex, CPoles);
        return aCopy;
    }

    /// <summary>A copy with a row of weighted poles inserted after UIndex.</summary>
    public Geom_BezierSurface PoleRowInsertedAfter(int UIndex, NCollection_Array1<gp_Pnt> CPoles, NCollection_Array1<double> CPoleWeights)
    {
        Geom_BezierSurface aCopy = CopyBezier();
        aCopy.InsertPoleRowAfter(UIndex, CPoles, CPoleWeights);
        return aCopy;
    }

    /// <summary>A copy with a row of poles inserted before UIndex.</summary>
    public Geom_BezierSurface PoleRowInsertedBefore(int UIndex, NCollection_Array1<gp_Pnt> CPoles)
    {
        Geom_BezierSurface aCopy = CopyBezier();
        aCopy.InsertPoleRowBefore(UIndex, CPoles);
        return aCopy;
    }

    /// <summary>A copy with a row of weighted poles inserted before UIndex.</summary>
    public Geom_BezierSurface PoleRowInsertedBefore(int UIndex, NCollection_Array1<gp_Pnt> CPoles, NCollection_Array1<double> CPoleWeights)
    {
        Geom_BezierSurface aCopy = CopyBezier();
        aCopy.InsertPoleRowBefore(UIndex, CPoles, CPoleWeights);
        return aCopy;
    }

    /// <summary>A copy without the column of poles VIndex.</summary>
    public Geom_BezierSurface PoleColRemoved(int VIndex)
    {
        Geom_BezierSurface aCopy = CopyBezier();
        aCopy.RemovePoleCol(VIndex);
        return aCopy;
    }

    /// <summary>A copy without the row of poles UIndex.</summary>
    public Geom_BezierSurface PoleRowRemoved(int UIndex)
    {
        Geom_BezierSurface aCopy = CopyBezier();
        aCopy.RemovePoleRow(UIndex);
        return aCopy;
    }

    /// <summary>A copy restricted to [U1, U2] x [V1, V2] (see <see cref="Segment"/>).</summary>
    public Geom_BezierSurface Segmented(double U1, double U2, double V1, double V2)
    {
        Geom_BezierSurface aCopy = CopyBezier();
        aCopy.Segment(U1, U2, V1, V2);
        return aCopy;
    }

    public Geom_BezierSurface WithPole(int UIndex, int VIndex, in gp_Pnt P)
    {
        Geom_BezierSurface aCopy = CopyBezier();
        aCopy.SetPole(UIndex, VIndex, P);
        return aCopy;
    }

    public Geom_BezierSurface WithPole(int UIndex, int VIndex, in gp_Pnt P, double Weight)
    {
        Geom_BezierSurface aCopy = CopyBezier();
        aCopy.SetPole(UIndex, VIndex, P, Weight);
        return aCopy;
    }

    public Geom_BezierSurface WithPoleCol(int VIndex, NCollection_Array1<gp_Pnt> CPoles)
    {
        Geom_BezierSurface aCopy = CopyBezier();
        aCopy.SetPoleCol(VIndex, CPoles);
        return aCopy;
    }

    public Geom_BezierSurface WithPoleCol(int VIndex, NCollection_Array1<gp_Pnt> CPoles, NCollection_Array1<double> CPoleWeights)
    {
        Geom_BezierSurface aCopy = CopyBezier();
        aCopy.SetPoleCol(VIndex, CPoles, CPoleWeights);
        return aCopy;
    }

    public Geom_BezierSurface WithPoleRow(int UIndex, NCollection_Array1<gp_Pnt> CPoles)
    {
        Geom_BezierSurface aCopy = CopyBezier();
        aCopy.SetPoleRow(UIndex, CPoles);
        return aCopy;
    }

    public Geom_BezierSurface WithPoleRow(int UIndex, NCollection_Array1<gp_Pnt> CPoles, NCollection_Array1<double> CPoleWeights)
    {
        Geom_BezierSurface aCopy = CopyBezier();
        aCopy.SetPoleRow(UIndex, CPoles, CPoleWeights);
        return aCopy;
    }

    public Geom_BezierSurface WithWeight(int UIndex, int VIndex, double Weight)
    {
        Geom_BezierSurface aCopy = CopyBezier();
        aCopy.SetWeight(UIndex, VIndex, Weight);
        return aCopy;
    }

    public Geom_BezierSurface WithWeightCol(int VIndex, NCollection_Array1<double> CPoleWeights)
    {
        Geom_BezierSurface aCopy = CopyBezier();
        aCopy.SetWeightCol(VIndex, CPoleWeights);
        return aCopy;
    }

    public Geom_BezierSurface WithWeightRow(int UIndex, NCollection_Array1<double> CPoleWeights)
    {
        Geom_BezierSurface aCopy = CopyBezier();
        aCopy.SetWeightRow(UIndex, CPoleWeights);
        return aCopy;
    }

    /// <summary>A copy evaluated through a copy of theDesc (null: without a representation).</summary>
    public Geom_BezierSurface WithEvalRepresentation(GeomEval_RepSurfaceDesc.Base? theDesc)
    {
        GeomEval_RepUtils.ValidateSurfaceDesc(theDesc, this);
        Geom_BezierSurface aCopy = CopyBezier();
        aCopy.SetEvalRepresentation(GeomEval_RepUtils.CloneSurfaceDesc(theDesc));
        return aCopy;
    }
}
