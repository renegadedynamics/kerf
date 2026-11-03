// Port of: src/ModelingData/TKG3d/Geom/Geom_BezierSurface.hxx
// Port of: src/ModelingData/TKG3d/Geom/Geom_BezierSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Evaluation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Geometry.Splines;
using Kerf.Geometry;
using Occt;
using static Kerf.Geometry.Continuity;

namespace Kerf.Geometry;

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
public class BezierSurface : BoundedSurface
{
    private NCollection_Array2<Point3D> _poles = new NCollection_Array2<Point3D>();
    private NCollection_Array2<double> _weights = new NCollection_Array2<double>();
    private RepSurfaceDesc.Base? _evalRep;
    private bool _uRational = false;
    private bool _vRational = false;
    private double _uMaxDerivInv = 0.0;
    private double _vMaxDerivInv = 0.0;
    private volatile bool _maxDerivInvOk = false;

    /// <summary>True if an evaluation representation is attached.</summary>
    public bool HasEvalRepresentation => _evalRep is not null;

    /// <summary>The evaluation representation descriptor (may be null). Stored, not copied: don't modify.</summary>
    internal RepSurfaceDesc.Base? EvalRepresentation() => _evalRep;

    /// <summary>Sets a new evaluation representation, after checking it (no circular reference).</summary>
    internal void SetEvalRepresentation(RepSurfaceDesc.Base? desc)
    {
        CheckMutable();
        RepUtils.ValidateSurfaceDesc(desc, this);
        _evalRep = desc;
    }

    /// <summary>Removes the evaluation representation.</summary>
    internal void ClearEvalRepresentation()
    {
        CheckMutable();
        _evalRep = null;
    }

    /// <summary>Checks the rationality of an array of weights in each direction.</summary>
    private static void Rational(NCollection_Array2<double> weights, out bool urational, out bool vrational)
    {
        int i, j;
        j = weights.LowerCol();
        vrational = false;
        while (!vrational && j <= weights.UpperCol())
        {
            i = weights.LowerRow();
            while (!vrational && i <= weights.UpperRow() - 1)
            {
                vrational = (Math.Abs(weights[i, j] - weights[i + 1, j]) > Epsilon(Math.Abs(weights[i, j])));
                i++;
            }
            j++;
        }

        i = weights.LowerRow();
        urational = false;
        while (!urational && i <= weights.UpperRow())
        {
            j = weights.LowerCol();
            while (!urational && j <= weights.UpperCol() - 1)
            {
                urational = (Math.Abs(weights[i, j] - weights[i, j + 1]) > Epsilon(Math.Abs(weights[i, j])));
                j++;
            }
            i++;
        }
    }

    private static void AddPoleCol(NCollection_Array2<Point3D> poles,
                                   NCollection_Array1<Point3D> poleCol,
                                   int afterIndex,
                                   NCollection_Array2<Point3D> newPoles)
    {
        int insertIndex = afterIndex + newPoles.LowerCol();
        int offset = newPoles.LowerRow() - poleCol.Lower();
        int colIndex = newPoles.LowerCol();
        int rowIndex;
        while (colIndex < insertIndex)
        {
            rowIndex = newPoles.LowerRow();
            while (rowIndex <= newPoles.UpperRow())
            {
                newPoles[rowIndex, colIndex] = poles[rowIndex, colIndex];
                rowIndex++;
            }
            colIndex++;
        }
        rowIndex = newPoles.LowerRow();
        while (rowIndex <= newPoles.UpperRow())
        {
            newPoles[rowIndex, colIndex] = poleCol[rowIndex - offset];
            rowIndex++;
        }
        colIndex++;
        while (colIndex <= newPoles.UpperCol())
        {
            rowIndex = newPoles.LowerRow();
            while (rowIndex <= newPoles.UpperRow())
            {
                newPoles[rowIndex, colIndex] = poles[rowIndex, colIndex - 1];
                rowIndex++;
            }
            colIndex++;
        }
    }

    private static void AddRatPoleCol(NCollection_Array2<Point3D> poles,
                                      NCollection_Array2<double> weights,
                                      NCollection_Array1<Point3D> poleCol,
                                      NCollection_Array1<double> poleWeightCol,
                                      int afterIndex,
                                      NCollection_Array2<Point3D> newPoles,
                                      NCollection_Array2<double> newWeights)
    {
        int insertIndex = afterIndex + newPoles.LowerCol();
        int offsetPol = newPoles.LowerRow() - poleCol.Lower();
        int offsetW = newWeights.LowerRow() - poleWeightCol.Lower();

        int colIndex = newPoles.LowerCol();
        int rowIndex;
        while (colIndex < insertIndex)
        {
            rowIndex = newPoles.LowerRow();
            while (rowIndex <= newPoles.UpperRow())
            {
                newPoles[rowIndex, colIndex] = poles[rowIndex, colIndex];
                newWeights[rowIndex, colIndex] = weights[rowIndex, colIndex];
                rowIndex++;
            }
            colIndex++;
        }
        rowIndex = newPoles.LowerRow();
        while (rowIndex <= newPoles.UpperRow())
        {
            newPoles[rowIndex, colIndex] = poleCol[rowIndex - offsetPol];
            newWeights[rowIndex, colIndex] = poleWeightCol[rowIndex - offsetW];
            rowIndex++;
        }
        colIndex++;
        while (colIndex <= newPoles.UpperCol())
        {
            rowIndex = newPoles.LowerRow();
            while (rowIndex <= newPoles.UpperRow())
            {
                newPoles[rowIndex, colIndex] = poles[rowIndex, colIndex - 1];
                newWeights[rowIndex, colIndex] = weights[rowIndex, colIndex - 1];
                rowIndex++;
            }
            colIndex++;
        }
    }

    private static void AddPoleRow(NCollection_Array2<Point3D> poles,
                                   NCollection_Array1<Point3D> poleRow,
                                   int afterIndex,
                                   NCollection_Array2<Point3D> newPoles)
    {
        int insertIndex = afterIndex + newPoles.LowerRow();
        int offset = newPoles.LowerCol() - poleRow.Lower();
        int rowIndex = newPoles.LowerRow();
        int colIndex;
        while (rowIndex < insertIndex)
        {
            colIndex = newPoles.LowerCol();
            while (colIndex <= newPoles.UpperCol())
            {
                newPoles[rowIndex, colIndex] = poles[rowIndex, colIndex];
                colIndex++;
            }
            rowIndex++;
        }
        colIndex = newPoles.LowerCol();
        while (colIndex <= newPoles.UpperCol())
        {
            newPoles[rowIndex, colIndex] = poleRow[colIndex - offset];
            colIndex++;
        }
        rowIndex++;
        while (rowIndex <= newPoles.UpperRow())
        {
            colIndex = newPoles.LowerCol();
            while (colIndex <= newPoles.UpperCol())
            {
                newPoles[rowIndex, colIndex] = poles[rowIndex - 1, colIndex];
                colIndex++;
            }
            rowIndex++;
        }
    }

    private static void AddRatPoleRow(NCollection_Array2<Point3D> poles,
                                      NCollection_Array2<double> weights,
                                      NCollection_Array1<Point3D> poleRow,
                                      NCollection_Array1<double> poleWeightRow,
                                      int afterIndex,
                                      NCollection_Array2<Point3D> newPoles,
                                      NCollection_Array2<double> newWeights)
    {
        int insertIndex = afterIndex + newPoles.LowerRow();
        int offsetPol = newPoles.LowerCol() - poleRow.Lower();
        int offsetW = newWeights.LowerCol() - poleWeightRow.Lower();

        int colIndex;
        int rowIndex = newPoles.LowerRow();
        while (rowIndex < insertIndex)
        {
            colIndex = newPoles.LowerCol();
            while (colIndex <= newPoles.UpperCol())
            {
                newPoles[rowIndex, colIndex] = poles[rowIndex, colIndex];
                newWeights[rowIndex, colIndex] = weights[rowIndex, colIndex];
                colIndex++;
            }
            rowIndex++;
        }
        colIndex = newPoles.LowerCol();
        while (colIndex <= newPoles.UpperCol())
        {
            newPoles[rowIndex, colIndex] = poleRow[colIndex - offsetPol];
            newWeights[rowIndex, colIndex] = poleWeightRow[colIndex - offsetW];
            colIndex++;
        }
        rowIndex++;
        while (rowIndex <= newPoles.UpperRow())
        {
            colIndex = newPoles.LowerCol();
            while (colIndex <= newPoles.UpperCol())
            {
                newPoles[rowIndex, colIndex] = poles[rowIndex - 1, colIndex];
                newWeights[rowIndex, colIndex] = weights[rowIndex - 1, colIndex];
                colIndex++;
            }
            rowIndex++;
        }
    }

    private static void DeletePoleCol(NCollection_Array2<Point3D> poles, int index, NCollection_Array2<Point3D> newPoles)
    {
        int offset = 0;
        int rowIndex;
        int colIndex = newPoles.LowerCol();
        while (colIndex <= newPoles.UpperCol())
        {
            rowIndex = newPoles.LowerRow();
            if (colIndex == index)
            {
                offset = 1;
            }
            while (rowIndex <= newPoles.UpperRow())
            {
                newPoles[rowIndex, colIndex] = poles[rowIndex, colIndex + offset];
                rowIndex++;
            }
            colIndex++;
        }
    }

    private static void DeleteRatPoleCol(NCollection_Array2<Point3D> poles,
                                         NCollection_Array2<double> weights,
                                         int index,
                                         NCollection_Array2<Point3D> newPoles,
                                         NCollection_Array2<double> newWeights)
    {
        int offset = 0;
        int rowIndex;
        int colIndex = newPoles.LowerCol();
        while (colIndex <= newPoles.UpperCol())
        {
            rowIndex = newPoles.LowerRow();
            if (colIndex == index)
            {
                offset = 1;
            }
            while (rowIndex <= newPoles.UpperRow())
            {
                newPoles[rowIndex, colIndex] = poles[rowIndex, colIndex + offset];
                newWeights[rowIndex, colIndex] = weights[rowIndex, colIndex + offset];
                rowIndex++;
            }
            colIndex++;
        }
    }

    private static void DeletePoleRow(NCollection_Array2<Point3D> poles, int index, NCollection_Array2<Point3D> newPoles)
    {
        int offset = 0;
        int colIndex;
        int rowIndex = newPoles.LowerRow();
        while (rowIndex <= newPoles.UpperRow())
        {
            colIndex = newPoles.LowerCol();
            if (rowIndex == index)
            {
                offset = 1;
            }
            while (colIndex <= newPoles.UpperCol())
            {
                newPoles[rowIndex, colIndex] = poles[rowIndex + offset, colIndex];
                colIndex++;
            }
            rowIndex++;
        }
    }

    private static void DeleteRatPoleRow(NCollection_Array2<Point3D> poles,
                                         NCollection_Array2<double> weights,
                                         int index,
                                         NCollection_Array2<Point3D> newPoles,
                                         NCollection_Array2<double> newWeights)
    {
        int offset = 0;
        int colIndex;
        int rowIndex = newPoles.LowerRow();
        while (rowIndex <= newPoles.UpperRow())
        {
            colIndex = newPoles.LowerCol();
            if (rowIndex == index)
            {
                offset = 1;
            }
            while (colIndex <= newPoles.UpperCol())
            {
                newPoles[rowIndex, colIndex] = poles[rowIndex + offset, colIndex];
                newWeights[rowIndex, colIndex] = weights[rowIndex + offset, colIndex];
                colIndex++;
            }
            rowIndex++;
        }
    }

    /// <summary>A copy of theOther (OCCT's copy constructor).</summary>
    public BezierSurface(BezierSurface other)
    {
        _poles = new NCollection_Array2<Point3D>(other._poles);
        _weights = (other._uRational || other._vRational)
                      ? new NCollection_Array2<double>(other._weights)
                      : SurfaceBSplSLib.UnitWeights(other._poles.ColLength(), other._poles.RowLength());
        _evalRep = RepUtils.CloneSurfaceDesc(other._evalRep);
        _uRational = other._uRational;
        _vRational = other._vRational;
        _uMaxDerivInv = other._uMaxDerivInv;
        _vMaxDerivInv = other._vMaxDerivInv;
        _maxDerivInvOk = false;
    }

    /// <summary>A non-rational Bezier surface on the given poles.</summary>
    /// <exception cref="ConstructionErrorException">Fewer than 2 or more than MaxDegree() + 1 poles in a direction.</exception>
    public BezierSurface(NCollection_Array2<Point3D> surfacePoles)
    {
        _maxDerivInvOk = false;
        int nbUPoles = surfacePoles.ColLength();
        int nbVPoles = surfacePoles.RowLength();
        if (nbUPoles < 2 || nbUPoles > MaxDegree() + 1 || nbVPoles < 2 || nbVPoles > MaxDegree() + 1)
        {
            throw new ConstructionErrorException();
        }

        _uRational = false;
        _vRational = false;

        // init non rational
        Init(surfacePoles, null);
    }

    /// <summary>A rational Bezier surface (non-rational if the weights are all equal in each direction).</summary>
    /// <exception cref="ConstructionErrorException">
    /// Fewer than 2 or more than MaxDegree() + 1 poles in a direction, weights of a different size, or a weight not
    /// greater than gp::Resolution().
    /// </exception>
    public BezierSurface(NCollection_Array2<Point3D> surfacePoles, NCollection_Array2<double> poleWeights)
    {
        _maxDerivInvOk = false;
        int nbUPoles = surfacePoles.ColLength();
        int nbVPoles = surfacePoles.RowLength();
        if (nbUPoles < 2 || nbUPoles > MaxDegree() + 1 || nbVPoles < 2 || nbVPoles > MaxDegree() + 1
            || nbVPoles != poleWeights.RowLength() || nbUPoles != poleWeights.ColLength())
        {
            throw new ConstructionErrorException();
        }

        int col = poleWeights.LowerCol();
        while (col <= poleWeights.UpperCol())
        {
            int row = poleWeights.LowerRow();
            while (row <= poleWeights.UpperRow())
            {
                if (poleWeights[row, col] <= Constants.Resolution())
                {
                    throw new ConstructionErrorException();
                }
                row++;
            }
            col++;
        }

        // Rational(PoleWeights, myURational, myVRational) inlined in OCCT
        int i, j;
        _uRational = false;
        _vRational = false;
        j = poleWeights.LowerCol();
        while (!_vRational && j <= poleWeights.UpperCol())
        {
            i = poleWeights.LowerRow();
            while (!_vRational && i <= poleWeights.UpperRow() - 1)
            {
                _vRational = (Math.Abs(poleWeights[i, j] - poleWeights[i + 1, j])
                               > Epsilon(Math.Abs(poleWeights[i, j])));
                i++;
            }
            j++;
        }
        i = poleWeights.LowerRow();
        while (!_uRational && i <= poleWeights.UpperRow())
        {
            j = poleWeights.LowerCol();
            while (!_uRational && j <= poleWeights.UpperCol() - 1)
            {
                _uRational = (Math.Abs(poleWeights[i, j] - poleWeights[i, j + 1])
                               > Epsilon(Math.Abs(poleWeights[i, j])));
                j++;
            }
            i++;
        }

        if (_uRational || _vRational)
        {
            Init(surfacePoles, poleWeights);
        }
        else
        {
            Init(surfacePoles, null);
        }
    }

    /// <summary>The maximum degree in each direction: 25.</summary>
    public static int MaxDegree() => global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree();

    /// <summary>Exchanges the U and V parametric directions; the poles grid is transposed.</summary>
    internal void ExchangeUV()
    {
        CheckMutable();
        int lr = _poles.LowerRow();
        int ur = _poles.UpperRow();
        int lc = _poles.LowerCol();
        int uc = _poles.UpperCol();

        NCollection_Array2<Point3D> npoles = new NCollection_Array2<Point3D>(lc, uc, lr, ur);
        NCollection_Array2<double> nweights = new NCollection_Array2<double>();
        bool rat = (_uRational || _vRational);
        if (rat)
        {
            nweights.Resize(lc, uc, lr, ur, false);
        }

        for (int i = lc; i <= uc; i++)
        {
            for (int j = lr; j <= ur; j++)
            {
                npoles[i, j] = _poles[j, i];
                if (rat)
                {
                    nweights[i, j] = _weights[j, i];
                }
            }
        }

        _poles = npoles;
        if (rat)
        {
            _weights = nweights;
        }
        else
        {
            _weights = SurfaceBSplSLib.UnitWeights(uc - lc + 1, ur - lr + 1);
        }
        (_uRational, _vRational) = (_vRational, _uRational);
        _maxDerivInvOk = false;
    }

    /// <summary>Raises the degrees to UDeg and VDeg.</summary>
    /// <exception cref="ConstructionErrorException">A degree lower than the current one or above MaxDegree().</exception>
    internal void Increase(int uDeg, int vDeg)
    {
        CheckMutable();
        if (uDeg < UDegree() || uDeg > BezierSurface.MaxDegree() || vDeg < VDegree()
            || vDeg > BezierSurface.MaxDegree())
        {
            throw new ConstructionErrorException();
        }

        int oldUDeg = UDegree();
        int oldVDeg = VDegree();
        int incUDeg = uDeg - oldUDeg;
        int incVDeg = vDeg - oldVDeg;
        if (incUDeg == 0 && incVDeg == 0)
        {
            return;
        }

        if (incUDeg > 0)
        {
            NCollection_Array2<Point3D> npoles = new NCollection_Array2<Point3D>(1, uDeg + 1, 1, oldVDeg + 1);
            NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, 2);
            NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, 2);
            if (_uRational || _vRational)
            {
                NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, uDeg + 1, 1, VDegree() + 1);

                SurfaceBSplSLib.IncreaseDegree(true,
                                        oldUDeg,
                                        uDeg,
                                        false,
                                        _poles,
                                        _weights,
                                        InternalUKnots(),
                                        InternalUMultiplicities(),
                                        npoles,
                                        nweights,
                                        nknots,
                                        nmults);
                _weights = nweights;
            }
            else
            {
                SurfaceBSplSLib.IncreaseDegree(true,
                                        oldUDeg,
                                        uDeg,
                                        false,
                                        _poles,
                                        SurfaceBSplSLib.NoWeights(),
                                        InternalUKnots(),
                                        InternalUMultiplicities(),
                                        npoles,
                                        SurfaceBSplSLib.NoWeights(),
                                        nknots,
                                        nmults);
            }
            _poles = npoles;
        }

        if (incVDeg > 0)
        {
            NCollection_Array2<Point3D> npoles = new NCollection_Array2<Point3D>(1, uDeg + 1, 1, vDeg + 1);
            NCollection_Array1<double> nknots = new NCollection_Array1<double>(1, 2);
            NCollection_Array1<int> nmults = new NCollection_Array1<int>(1, 2);
            // OCCT passes UKnots() as the V knots: the Bezier knots are (0, 1) in both directions.
            if (_uRational || _vRational)
            {
                NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, uDeg + 1, 1, vDeg + 1);

                SurfaceBSplSLib.IncreaseDegree(false,
                                        oldVDeg,
                                        vDeg,
                                        false,
                                        _poles,
                                        _weights,
                                        InternalUKnots(),
                                        InternalVMultiplicities(),
                                        npoles,
                                        nweights,
                                        nknots,
                                        nmults);
                _weights = nweights;
            }
            else
            {
                SurfaceBSplSLib.IncreaseDegree(false,
                                        oldVDeg,
                                        vDeg,
                                        false,
                                        _poles,
                                        SurfaceBSplSLib.NoWeights(),
                                        InternalUKnots(),
                                        InternalVMultiplicities(),
                                        npoles,
                                        SurfaceBSplSLib.NoWeights(),
                                        nknots,
                                        nmults);
            }
            _poles = npoles;
        }

        if (_uRational || _vRational)
        {
            Rational(_weights, out _uRational, out _vRational);
            if (!(_uRational || _vRational))
            {
                _weights = SurfaceBSplSLib.UnitWeights(_poles.ColLength(), _poles.RowLength());
            }
        }
        else
        {
            _weights = SurfaceBSplSLib.UnitWeights(_poles.ColLength(), _poles.RowLength());
        }
        _maxDerivInvOk = false;
    }

    /// <summary>Inserts a column of poles after the column VIndex (weight 1 if the surface is rational).</summary>
    /// <exception cref="OutOfRangeException">VIndex out of [1, NbVPoles].</exception>
    /// <exception cref="ConstructionErrorException">CPoles.Length() != NbUPoles.</exception>
    internal void InsertPoleColAfter(int vIndex, NCollection_Array1<Point3D> cPoles)
    {
        CheckMutable();
        if (vIndex < 1 || vIndex > _poles.RowLength())
        {
            throw new OutOfRangeException();
        }
        if (cPoles.Length() != _poles.ColLength())
        {
            throw new ConstructionErrorException();
        }

        int nbUPoles = _poles.ColLength();
        int nbVPoles = _poles.RowLength();

        NCollection_Array2<Point3D> npoles = new NCollection_Array2<Point3D>(1, nbUPoles, 1, nbVPoles + 1);

        if (_uRational || _vRational)
        {
            NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, nbUPoles, 1, nbVPoles + 1);
            NCollection_Array1<double> cWeights = new NCollection_Array1<double>(1, nbUPoles);
            cWeights.Init(1.0);
            AddRatPoleCol(_poles, _weights, cPoles, cWeights, vIndex, npoles, nweights);
            _weights = nweights;
        }
        else
        {
            AddPoleCol(_poles, cPoles, vIndex, npoles);
            _weights = SurfaceBSplSLib.UnitWeights(nbUPoles, nbVPoles + 1);
        }
        _poles = npoles;
        _maxDerivInvOk = false;
    }

    /// <summary>Inserts a column of weighted poles after the column VIndex.</summary>
    /// <exception cref="OutOfRangeException">VIndex out of [1, NbVPoles].</exception>
    /// <exception cref="ConstructionErrorException">Bad lengths, or a weight not greater than gp::Resolution().</exception>
    internal void InsertPoleColAfter(int vIndex, NCollection_Array1<Point3D> cPoles, NCollection_Array1<double> cPoleWeights)
    {
        CheckMutable();
        if (vIndex < 1 || vIndex > _poles.RowLength())
        {
            throw new OutOfRangeException();
        }
        if (cPoles.Length() != _poles.ColLength() || cPoleWeights.Length() != cPoles.Length())
        {
            throw new ConstructionErrorException();
        }
        int index = cPoleWeights.Lower();
        while (index <= cPoleWeights.Upper())
        {
            if (cPoleWeights[index] <= Constants.Resolution())
            {
                throw new ConstructionErrorException();
            }
            index++;
        }

        int nbUPoles = _poles.ColLength();
        int nbVPoles = _poles.RowLength();

        // Ensure weights are an owned copy for rational insertion
        if (!(_uRational || _vRational))
        {
            _weights = new NCollection_Array2<double>(_weights);
        }

        NCollection_Array2<Point3D> npoles = new NCollection_Array2<Point3D>(1, nbUPoles, 1, nbVPoles + 1);
        NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, nbUPoles, 1, nbVPoles + 1);

        AddRatPoleCol(_poles, _weights, cPoles, cPoleWeights, vIndex, npoles, nweights);
        _poles = npoles;
        _weights = nweights;
        Rational(_weights, out _uRational, out _vRational);
        if (!(_uRational || _vRational))
        {
            _weights = SurfaceBSplSLib.UnitWeights(_poles.ColLength(), _poles.RowLength());
        }
        _maxDerivInvOk = false;
    }

    /// <summary>Inserts a column of poles before the column VIndex.</summary>
    internal void InsertPoleColBefore(int vIndex, NCollection_Array1<Point3D> cPoles)
    {
        CheckMutable();
        InsertPoleColAfter(vIndex - 1, cPoles);
    }

    /// <summary>Inserts a column of weighted poles before the column VIndex.</summary>
    internal void InsertPoleColBefore(int vIndex, NCollection_Array1<Point3D> cPoles, NCollection_Array1<double> cPoleWeights)
    {
        CheckMutable();
        InsertPoleColAfter(vIndex - 1, cPoles, cPoleWeights);
    }

    /// <summary>Inserts a row of poles after the row UIndex (weight 1 if the surface is rational).</summary>
    /// <exception cref="OutOfRangeException">UIndex out of [1, NbUPoles].</exception>
    /// <exception cref="ConstructionErrorException">CPoles.Length() != NbVPoles.</exception>
    internal void InsertPoleRowAfter(int uIndex, NCollection_Array1<Point3D> cPoles)
    {
        CheckMutable();
        if (uIndex < 1 || uIndex > _poles.ColLength())
        {
            throw new OutOfRangeException();
        }
        if (cPoles.Length() != _poles.RowLength())
        {
            throw new ConstructionErrorException();
        }

        int nbUPoles = _poles.ColLength();
        int nbVPoles = _poles.RowLength();

        NCollection_Array2<Point3D> npoles = new NCollection_Array2<Point3D>(1, nbUPoles + 1, 1, nbVPoles);

        if (_uRational || _vRational)
        {
            NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, nbUPoles + 1, 1, nbVPoles);
            NCollection_Array1<double> cWeights = new NCollection_Array1<double>(1, nbVPoles);
            cWeights.Init(1.0);
            AddRatPoleRow(_poles, _weights, cPoles, cWeights, uIndex, npoles, nweights);
            _weights = nweights;
        }
        else
        {
            AddPoleRow(_poles, cPoles, uIndex, npoles);
            _weights = SurfaceBSplSLib.UnitWeights(nbUPoles + 1, nbVPoles);
        }
        _poles = npoles;
        _maxDerivInvOk = false;
    }

    /// <summary>Inserts a row of weighted poles after the row UIndex.</summary>
    /// <exception cref="OutOfRangeException">UIndex out of [1, NbUPoles].</exception>
    /// <exception cref="ConstructionErrorException">Bad lengths, or a weight not greater than gp::Resolution().</exception>
    internal void InsertPoleRowAfter(int uIndex, NCollection_Array1<Point3D> cPoles, NCollection_Array1<double> cPoleWeights)
    {
        CheckMutable();
        if (uIndex < 1 || uIndex > _poles.ColLength())
        {
            throw new OutOfRangeException();
        }
        if (cPoles.Length() != _poles.RowLength() || cPoleWeights.Length() != cPoles.Length())
        {
            throw new ConstructionErrorException();
        }
        int index = cPoleWeights.Lower();
        while (index <= cPoleWeights.Upper())
        {
            if (cPoleWeights[index] <= Constants.Resolution())
            {
                throw new ConstructionErrorException();
            }
            index++;
        }

        int nbUPoles = _poles.ColLength();
        int nbVPoles = _poles.RowLength();

        // Ensure weights are an owned copy for rational insertion
        if (!(_uRational || _vRational))
        {
            _weights = new NCollection_Array2<double>(_weights);
        }

        NCollection_Array2<Point3D> npoles = new NCollection_Array2<Point3D>(1, nbUPoles + 1, 1, nbVPoles);
        NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, nbUPoles + 1, 1, nbVPoles);

        AddRatPoleRow(_poles, _weights, cPoles, cPoleWeights, uIndex, npoles, nweights);
        _poles = npoles;
        _weights = nweights;
        Rational(_weights, out _uRational, out _vRational);
        if (!(_uRational || _vRational))
        {
            _weights = SurfaceBSplSLib.UnitWeights(_poles.ColLength(), _poles.RowLength());
        }
        _maxDerivInvOk = false;
    }

    /// <summary>Inserts a row of poles before the row UIndex.</summary>
    internal void InsertPoleRowBefore(int uIndex, NCollection_Array1<Point3D> cPoles)
    {
        CheckMutable();
        InsertPoleRowAfter(uIndex - 1, cPoles);
    }

    /// <summary>Inserts a row of weighted poles before the row UIndex.</summary>
    internal void InsertPoleRowBefore(int uIndex, NCollection_Array1<Point3D> cPoles, NCollection_Array1<double> cPoleWeights)
    {
        CheckMutable();
        InsertPoleRowAfter(uIndex - 1, cPoles, cPoleWeights);
    }

    /// <summary>Removes the column of poles VIndex.</summary>
    /// <exception cref="OutOfRangeException">VIndex out of [1, NbVPoles].</exception>
    /// <exception cref="ConstructionErrorException">Only 2 columns are left.</exception>
    internal void RemovePoleCol(int vIndex)
    {
        CheckMutable();
        if (vIndex < 1 || vIndex > _poles.RowLength())
        {
            throw new OutOfRangeException();
        }
        if (_poles.RowLength() <= 2)
        {
            throw new ConstructionErrorException();
        }

        int nbUPoles = _poles.ColLength();
        int nbVPoles = _poles.RowLength();

        NCollection_Array2<Point3D> npoles = new NCollection_Array2<Point3D>(1, nbUPoles, 1, nbVPoles - 1);

        if (_uRational || _vRational)
        {
            NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, nbUPoles, 1, nbVPoles - 1);
            DeleteRatPoleCol(_poles, _weights, vIndex, npoles, nweights);
            Rational(nweights, out _uRational, out _vRational);
            if (_uRational || _vRational)
            {
                _weights = nweights;
            }
            else
            {
                _weights = SurfaceBSplSLib.UnitWeights(nbUPoles, nbVPoles - 1);
            }
        }
        else
        {
            DeletePoleCol(_poles, vIndex, npoles);
            _weights = SurfaceBSplSLib.UnitWeights(nbUPoles, nbVPoles - 1);
        }
        _poles = npoles;
        _maxDerivInvOk = false;
    }

    /// <summary>Removes the row of poles UIndex.</summary>
    /// <exception cref="OutOfRangeException">UIndex out of [1, NbUPoles].</exception>
    /// <exception cref="ConstructionErrorException">Only 2 rows are left.</exception>
    internal void RemovePoleRow(int uIndex)
    {
        CheckMutable();
        if (uIndex < 1 || uIndex > _poles.ColLength())
        {
            throw new OutOfRangeException();
        }
        if (_poles.ColLength() <= 2)
        {
            throw new ConstructionErrorException();
        }

        int nbUPoles = _poles.ColLength();
        int nbVPoles = _poles.RowLength();

        NCollection_Array2<Point3D> npoles = new NCollection_Array2<Point3D>(1, nbUPoles - 1, 1, nbVPoles);

        if (_uRational || _vRational)
        {
            NCollection_Array2<double> nweights = new NCollection_Array2<double>(1, nbUPoles - 1, 1, nbVPoles);
            DeleteRatPoleRow(_poles, _weights, uIndex, npoles, nweights);
            Rational(nweights, out _uRational, out _vRational);
            if (_uRational || _vRational)
            {
                _weights = nweights;
            }
            else
            {
                _weights = SurfaceBSplSLib.UnitWeights(nbUPoles - 1, nbVPoles);
            }
        }
        else
        {
            DeletePoleRow(_poles, uIndex, npoles);
            _weights = SurfaceBSplSLib.UnitWeights(nbUPoles - 1, nbVPoles);
        }
        _poles = npoles;
        _maxDerivInvOk = false;
    }

    /// <summary>
    /// Restricts the surface to [U1, U2] x [V1, V2], reparametrised on [0, 1] x [0, 1] (U1 &gt; U2 reverses the
    /// U direction, likewise in V).
    /// </summary>
    internal void Segment(double u1, double u2, double v1, double v2)
    {
        CheckMutable();
        bool rat = (_uRational || _vRational);

        int minDegree = UDegree() <= VDegree() ? UDegree() : VDegree();
        int maxDegree = UDegree() > VDegree() ? UDegree() : VDegree();

        NCollection_Array2<Point3D> coefs = new NCollection_Array2<Point3D>(1, maxDegree + 1, 1, minDegree + 1);
        NCollection_Array2<double> wCoefs = new NCollection_Array2<double>();
        if (rat)
        {
            wCoefs.Resize(1, maxDegree + 1, 1, minDegree + 1, false);
        }

        const double uparameter_11 = 0.5;
        const double uspanlenght_11 = 0.5;
        const double vparameter_11 = 0.5;
        const double vspanlenght_11 = 0.5;

        if (rat)
        {
            SurfaceBSplSLib.BuildCache(uparameter_11,
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
                                _poles,
                                _weights,
                                coefs,
                                wCoefs);
        }
        else
        {
            SurfaceBSplSLib.BuildCache(uparameter_11,
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
                                _poles,
                                SurfaceBSplSLib.NoWeights(),
                                coefs,
                                SurfaceBSplSLib.NoWeights());
        }

        // When udeg <= vdeg, u and v are interchanged in the coefficients,
        // so everything needs to be transposed.
        if (UDegree() <= VDegree())
        {
            NCollection_Array2<Point3D> transposed = new NCollection_Array2<Point3D>(1, UDegree() + 1, 1, VDegree() + 1);
            for (int ii = 1; ii <= UDegree() + 1; ii++)
            {
                for (int jj = 1; jj <= VDegree() + 1; jj++)
                {
                    transposed[ii, jj] = coefs[jj, ii];
                }
            }
            coefs = transposed;

            if (rat)
            {
                NCollection_Array2<double> wTransposed = new NCollection_Array2<double>(1, UDegree() + 1, 1, VDegree() + 1);
                for (int ii = 1; ii <= UDegree() + 1; ii++)
                {
                    for (int jj = 1; jj <= VDegree() + 1; jj++)
                    {
                        wTransposed[ii, jj] = wCoefs[jj, ii];
                    }
                }
                wCoefs = wTransposed;
            }
        }

        // Trim in the canonical basis and update Poles and Coefficients.
        double ufirst = 2 * (u1 - 0.5), ulast = 2 * (u2 - 0.5), vfirst = 2 * (v1 - 0.5), vlast = 2 * (v2 - 0.5);
        if (rat)
        {
            global::Kerf.Engine.Numerics.Polynomials.PLib.UTrimming(ufirst, ulast, coefs, wCoefs);
            global::Kerf.Engine.Numerics.Polynomials.PLib.VTrimming(vfirst, vlast, coefs, wCoefs);
            global::Kerf.Engine.Numerics.Polynomials.PLib.CoefficientsPoles(coefs, wCoefs, _poles, _weights);
        }
        else
        {
            global::Kerf.Engine.Numerics.Polynomials.PLib.UTrimming(ufirst, ulast, coefs, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights2());
            global::Kerf.Engine.Numerics.Polynomials.PLib.VTrimming(vfirst, vlast, coefs, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights2());
            global::Kerf.Engine.Numerics.Polynomials.PLib.CoefficientsPoles(coefs, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights2(), _poles, global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights2());
        }

        _maxDerivInvOk = false;
    }

    /// <summary>Changes the pole (UIndex, VIndex).</summary>
    /// <exception cref="OutOfRangeException">An index out of the pole grid.</exception>
    internal void SetPole(int uIndex, int vIndex, in Point3D p)
    {
        CheckMutable();
        if (uIndex < 1 || uIndex > _poles.ColLength() || vIndex < 1 || vIndex > _poles.RowLength())
        {
            throw new OutOfRangeException();
        }
        // OCCT: keeps the evaluation representation, as every edit but Transform, UReverse and VReverse does
        // (Geom_BSplineSurface clears it): the surface goes on evaluating the old shape (occt-issues.md)
        _poles[uIndex, vIndex] = p;
        _maxDerivInvOk = false;
    }

    /// <summary>Changes the pole (UIndex, VIndex) and its weight.</summary>
    /// <exception cref="ConstructionErrorException">Weight not greater than gp::Resolution().</exception>
    /// <exception cref="OutOfRangeException">An index out of the pole grid.</exception>
    internal void SetPole(int uIndex, int vIndex, in Point3D p, double weight)
    {
        CheckMutable();
        if (weight <= Constants.Resolution())
        {
            throw new ConstructionErrorException("Geom_BezierSurface::SetPole");
        }
        if (uIndex < 1 || uIndex > _poles.ColLength() || vIndex < 1 || vIndex > _poles.RowLength())
        {
            throw new OutOfRangeException("Geom_BezierSurface::SetPole");
        }

        _poles[uIndex, vIndex] = p;

        SetWeight(uIndex, vIndex, weight);
    }

    /// <summary>Changes the poles and weights of the column VIndex.</summary>
    internal void SetPoleCol(int vIndex, NCollection_Array1<Point3D> cPoles, NCollection_Array1<double> cPoleWeights)
    {
        CheckMutable();
        if (vIndex < 1 || vIndex > _poles.RowLength())
        {
            throw new OutOfRangeException();
        }

        if (cPoles.Lower() < 1 || cPoles.Lower() > _poles.ColLength() || cPoles.Upper() < 1
            || cPoles.Upper() > _poles.ColLength() || cPoleWeights.Lower() != cPoles.Lower()
            || cPoleWeights.Upper() != cPoles.Upper())
        {
            throw new ConstructionErrorException();
        }

        int i;
        for (i = cPoles.Lower(); i <= cPoles.Upper(); i++)
        {
            _poles[i, vIndex] = cPoles[i];
        }
        SetWeightCol(vIndex, cPoleWeights);
    }

    /// <summary>Changes the poles of the column VIndex (rows CPoles.Lower() to Upper()).</summary>
    internal void SetPoleCol(int vIndex, NCollection_Array1<Point3D> cPoles)
    {
        CheckMutable();
        if (vIndex < 1 || vIndex > _poles.RowLength())
        {
            throw new OutOfRangeException();
        }

        if (cPoles.Lower() < 1 || cPoles.Lower() > _poles.ColLength() || cPoles.Upper() < 1
            || cPoles.Upper() > _poles.ColLength())
        {
            throw new ConstructionErrorException();
        }

        for (int i = cPoles.Lower(); i <= cPoles.Upper(); i++)
        {
            _poles[i, vIndex] = cPoles[i];
        }
        _maxDerivInvOk = false;
    }

    /// <summary>Changes the poles of the row UIndex (columns CPoles.Lower() to Upper()).</summary>
    internal void SetPoleRow(int uIndex, NCollection_Array1<Point3D> cPoles)
    {
        CheckMutable();
        if (uIndex < 1 || uIndex > _poles.ColLength())
        {
            throw new OutOfRangeException();
        }
        if (cPoles.Lower() < 1 || cPoles.Lower() > _poles.RowLength() || cPoles.Upper() < 1
            || cPoles.Upper() > _poles.RowLength())
        {
            throw new ConstructionErrorException();
        }

        for (int i = cPoles.Lower(); i <= cPoles.Upper(); i++)
        {
            _poles[uIndex, i] = cPoles[i];
        }
        _maxDerivInvOk = false;
    }

    /// <summary>Changes the poles and weights of the row UIndex.</summary>
    internal void SetPoleRow(int uIndex, NCollection_Array1<Point3D> cPoles, NCollection_Array1<double> cPoleWeights)
    {
        CheckMutable();
        if (uIndex < 1 || uIndex > _poles.ColLength())
        {
            throw new OutOfRangeException();
        }
        if (cPoles.Lower() < 1 || cPoles.Lower() > _poles.RowLength() || cPoles.Upper() < 1
            || cPoles.Upper() > _poles.RowLength() || cPoleWeights.Lower() != cPoles.Lower()
            || cPoleWeights.Upper() != cPoles.Upper())
        {
            throw new ConstructionErrorException();
        }

        int i;

        for (i = cPoles.Lower(); i <= cPoles.Upper(); i++)
        {
            _poles[uIndex, i] = cPoles[i];
        }

        SetWeightRow(uIndex, cPoleWeights);
    }

    /// <summary>Changes the weight of the pole (UIndex, VIndex); the rationality flags are recomputed.</summary>
    /// <exception cref="ConstructionErrorException">Weight not greater than gp::Resolution().</exception>
    /// <exception cref="OutOfRangeException">An index out of the pole grid.</exception>
    internal void SetWeight(int uIndex, int vIndex, double weight)
    {
        CheckMutable();
        if (weight <= Constants.Resolution())
        {
            throw new ConstructionErrorException("Geom_BezierSurface::SetWeight");
        }

        if (uIndex < 1 || uIndex > _poles.ColLength() || vIndex < 1 || vIndex > _poles.RowLength())
        {
            throw new OutOfRangeException();
        }

        // compute new rationality
        bool wasrat = (_uRational || _vRational);
        if (!wasrat)
        {
            // a weight of 1. does not turn to rational
            if (Math.Abs(weight - 1.0) <= Constants.Resolution())
            {
                return;
            }

            // owned copy from non-owning view
            _weights = new NCollection_Array2<double>(_weights);
        }

        if (Math.Abs(weight - _weights[uIndex, vIndex]) > Constants.Resolution())
        {
            _weights[uIndex, vIndex] = weight;
            Rational(_weights, out _uRational, out _vRational);
            if (!(_uRational || _vRational))
            {
                _weights = SurfaceBSplSLib.UnitWeights(_poles.ColLength(), _poles.RowLength());
            }
        }

        _maxDerivInvOk = false;
    }

    /// <summary>Changes the weights of the column VIndex.</summary>
    internal void SetWeightCol(int vIndex, NCollection_Array1<double> cPoleWeights)
    {
        CheckMutable();
        if (vIndex < 1 || vIndex > _poles.RowLength())
        {
            throw new OutOfRangeException();
        }

        if (cPoleWeights.Length() != _poles.ColLength())
        {
            throw new ConstructionErrorException("Geom_BezierSurface::SetWeightCol");
        }

        int i;

        // compute new rationality
        bool wasrat = (_uRational || _vRational);
        if (!wasrat)
        {
            // owned copy from non-owning view
            _weights = new NCollection_Array2<double>(_weights);
        }

        i = cPoleWeights.Lower();
        while (i <= cPoleWeights.Upper())
        {
            if (cPoleWeights[i] <= Constants.Resolution())
            {
                throw new ConstructionErrorException();
            }
            _weights[i, vIndex] = cPoleWeights[i];
            i++;
        }

        Rational(_weights, out _uRational, out _vRational);
        if (!(_uRational || _vRational))
        {
            _weights = SurfaceBSplSLib.UnitWeights(_poles.ColLength(), _poles.RowLength());
        }

        _maxDerivInvOk = false;
    }

    /// <summary>Changes the weights of the row UIndex.</summary>
    internal void SetWeightRow(int uIndex, NCollection_Array1<double> cPoleWeights)
    {
        CheckMutable();
        if (uIndex < 1 || uIndex > _poles.ColLength())
        {
            throw new OutOfRangeException("Geom_BezierSurface::SetWeightRow");
        }
        if (cPoleWeights.Lower() < 1 || cPoleWeights.Lower() > _poles.RowLength() || cPoleWeights.Upper() < 1
            || cPoleWeights.Upper() > _poles.RowLength())
        {
            throw new ConstructionErrorException("Geom_BezierSurface::SetWeightRow");
        }

        int i;

        // compute new rationality
        bool wasrat = (_uRational || _vRational);
        if (!wasrat)
        {
            // owned copy from non-owning view
            _weights = new NCollection_Array2<double>(_weights);
        }

        i = cPoleWeights.Lower();
        while (i <= cPoleWeights.Upper())
        {
            if (cPoleWeights[i] <= Constants.Resolution())
            {
                throw new ConstructionErrorException();
            }
            _weights[uIndex, i] = cPoleWeights[i];
            i++;
        }

        Rational(_weights, out _uRational, out _vRational);
        if (!(_uRational || _vRational))
        {
            _weights = SurfaceBSplSLib.UnitWeights(_poles.ColLength(), _poles.RowLength());
        }

        _maxDerivInvOk = false;
    }

    /// <summary>Reverses the U direction (the rows of the pole grid).</summary>
    protected internal sealed override void UReverse()
    {
        CheckMutable();
        Point3D pol;
        int row, col;
        int nbUPoles = _poles.ColLength();
        int nbVPoles = _poles.RowLength();

        if (_uRational || _vRational)
        {
            double w;
            for (col = 1; col <= nbVPoles; col++)
            {
                for (row = 1; row <= nbUPoles / 2; row++)
                {
                    w = _weights[row, col];
                    _weights[row, col] = _weights[nbUPoles - row + 1, col];
                    _weights[nbUPoles - row + 1, col] = w;
                    pol = _poles[row, col];
                    _poles[row, col] = _poles[nbUPoles - row + 1, col];
                    _poles[nbUPoles - row + 1, col] = pol;
                }
            }
        }
        else
        {
            for (col = 1; col <= nbVPoles; col++)
            {
                for (row = 1; row <= nbUPoles / 2; row++)
                {
                    pol = _poles[row, col];
                    _poles[row, col] = _poles[nbUPoles - row + 1, col];
                    _poles[nbUPoles - row + 1, col] = pol;
                }
            }
        }
        ClearEvalRepresentation();
        _maxDerivInvOk = false;
    }

    /// <summary>1 - U.</summary>
    public sealed override double UReversedParameter(double u) => (1.0 - u);

    /// <summary>Reverses the V direction (the columns of the pole grid).</summary>
    protected internal sealed override void VReverse()
    {
        CheckMutable();
        Point3D pol;
        int row, col;
        int nbUPoles = _poles.ColLength();
        int nbVPoles = _poles.RowLength();

        if (_uRational || _vRational)
        {
            double w;
            for (row = 1; row <= nbUPoles; row++)
            {
                for (col = 1; col <= nbVPoles / 2; col++)
                {
                    w = _weights[row, col];
                    _weights[row, col] = _weights[row, nbVPoles - col + 1];
                    _weights[row, nbVPoles - col + 1] = w;
                    pol = _poles[row, col];
                    _poles[row, col] = _poles[row, nbVPoles - col + 1];
                    _poles[row, nbVPoles - col + 1] = pol;
                }
            }
        }
        else
        {
            for (row = 1; row <= nbUPoles; row++)
            {
                for (col = 1; col <= nbVPoles / 2; col++)
                {
                    pol = _poles[row, col];
                    _poles[row, col] = _poles[row, nbVPoles - col + 1];
                    _poles[row, nbVPoles - col + 1] = pol;
                }
            }
        }
        ClearEvalRepresentation();
        _maxDerivInvOk = false;
    }

    /// <summary>1 - V.</summary>
    public sealed override double VReversedParameter(double v) => (1.0 - v);

    /// <summary>The parametric bounds: [0, 1] x [0, 1].</summary>
    public sealed override void Bounds(out double u1, out double u2, out double v1, out double v2)
    {
        u1 = 0.0;
        u2 = 1.0;
        v1 = 0.0;
        v2 = 1.0;
    }

    /// <summary>CN.</summary>
    public sealed override Continuity Continuity() => CN;

    /// <summary>The point of parameters (U, V).</summary>
    public sealed override Point3D EvalD0(double u, double v)
    {
        Point3D evalRepResult;
        if (RepUtils.TryEvalSurfaceD0(_evalRep, u, v, out evalRepResult))
        {
            return evalRepResult;
        }

        Point3D p;
        if (_uRational || _vRational)
        {
            SurfaceBSplSLib.D0(u,
                        v,
                        1,
                        1,
                        _poles,
                        _weights,
                        InternalUKnots(),
                        InternalUKnots(),
                        InternalUMultiplicities(),
                        InternalVMultiplicities(),
                        (_poles.ColLength() - 1),
                        (_poles.RowLength() - 1),
                        _uRational,
                        _vRational,
                        false,
                        false,
                        out p);
        }
        else
        {
            SurfaceBSplSLib.D0(u,
                        v,
                        1,
                        1,
                        _poles,
                        SurfaceBSplSLib.NoWeights(),
                        InternalUKnots(),
                        InternalUKnots(),
                        InternalUMultiplicities(),
                        InternalVMultiplicities(),
                        (_poles.ColLength() - 1),
                        (_poles.RowLength() - 1),
                        _uRational,
                        _vRational,
                        false,
                        false,
                        out p);
        }
        return p;
    }

    /// <summary>The point and first derivatives at (U, V).</summary>
    public sealed override ResD1 EvalD1(double u, double v)
    {
        ResD1 evalRepResult;
        if (RepUtils.TryEvalSurfaceD1(_evalRep, u, v, out evalRepResult))
        {
            return evalRepResult;
        }

        ResD1 result;
        if (_uRational || _vRational)
        {
            SurfaceBSplSLib.D1(u,
                        v,
                        1,
                        1,
                        _poles,
                        _weights,
                        InternalUKnots(),
                        InternalUKnots(),
                        InternalUMultiplicities(),
                        InternalVMultiplicities(),
                        (_poles.ColLength() - 1),
                        (_poles.RowLength() - 1),
                        _uRational,
                        _vRational,
                        false,
                        false,
                        out result.Point,
                        out result.D1U,
                        out result.D1V);
        }
        else
        {
            SurfaceBSplSLib.D1(u,
                        v,
                        1,
                        1,
                        _poles,
                        SurfaceBSplSLib.NoWeights(),
                        InternalUKnots(),
                        InternalUKnots(),
                        InternalUMultiplicities(),
                        InternalVMultiplicities(),
                        (_poles.ColLength() - 1),
                        (_poles.RowLength() - 1),
                        _uRational,
                        _vRational,
                        false,
                        false,
                        out result.Point,
                        out result.D1U,
                        out result.D1V);
        }
        return result;
    }

    /// <summary>The point and derivatives up to the second order at (U, V).</summary>
    public sealed override ResD2 EvalD2(double u, double v)
    {
        ResD2 evalRepResult;
        if (RepUtils.TryEvalSurfaceD2(_evalRep, u, v, out evalRepResult))
        {
            return evalRepResult;
        }

        ResD2 result;
        if (_uRational || _vRational)
        {
            //-- WARNING: pay attention to the ORDER of arguments in BSPLSLIB
            SurfaceBSplSLib.D2(u,
                        v,
                        1,
                        1,
                        _poles,
                        _weights,
                        InternalUKnots(),
                        InternalUKnots(),
                        InternalUMultiplicities(),
                        InternalVMultiplicities(),
                        (_poles.ColLength() - 1),
                        (_poles.RowLength() - 1),
                        _uRational,
                        _vRational,
                        false,
                        false,
                        out result.Point,
                        out result.D1U,
                        out result.D1V,
                        out result.D2U,
                        out result.D2V,
                        out result.D2UV);
        }
        else
        {
            //-- WARNING: pay attention to the ORDER of arguments in BSPLSLIB
            SurfaceBSplSLib.D2(u,
                        v,
                        1,
                        1,
                        _poles,
                        SurfaceBSplSLib.NoWeights(),
                        InternalUKnots(),
                        InternalUKnots(),
                        InternalUMultiplicities(),
                        InternalVMultiplicities(),
                        (_poles.ColLength() - 1),
                        (_poles.RowLength() - 1),
                        _uRational,
                        _vRational,
                        false,
                        false,
                        out result.Point,
                        out result.D1U,
                        out result.D1V,
                        out result.D2U,
                        out result.D2V,
                        out result.D2UV);
        }
        return result;
    }

    /// <summary>The point and derivatives up to the third order at (U, V).</summary>
    public sealed override ResD3 EvalD3(double u, double v)
    {
        ResD3 evalRepResult;
        if (RepUtils.TryEvalSurfaceD3(_evalRep, u, v, out evalRepResult))
        {
            return evalRepResult;
        }

        ResD3 result;
        if (_uRational || _vRational)
        {
            SurfaceBSplSLib.D3(u,
                        v,
                        0,
                        0,
                        _poles,
                        _weights,
                        InternalUKnots(),
                        InternalUKnots(),
                        InternalUMultiplicities(),
                        InternalVMultiplicities(),
                        (_poles.ColLength() - 1),
                        (_poles.RowLength() - 1),
                        _uRational,
                        _vRational,
                        false,
                        false,
                        out result.Point,
                        out result.D1U,
                        out result.D1V,
                        out result.D2U,
                        out result.D2V,
                        out result.D2UV,
                        out result.D3U,
                        out result.D3V,
                        out result.D3UUV,
                        out result.D3UVV);
        }
        else
        {
            SurfaceBSplSLib.D3(u,
                        v,
                        0,
                        0,
                        _poles,
                        SurfaceBSplSLib.NoWeights(),
                        InternalUKnots(),
                        InternalUKnots(),
                        InternalUMultiplicities(),
                        InternalVMultiplicities(),
                        (_poles.ColLength() - 1),
                        (_poles.RowLength() - 1),
                        _uRational,
                        _vRational,
                        false,
                        false,
                        out result.Point,
                        out result.D1U,
                        out result.D1V,
                        out result.D2U,
                        out result.D2V,
                        out result.D2UV,
                        out result.D3U,
                        out result.D3V,
                        out result.D3UUV,
                        out result.D3UVV);
        }
        return result;
    }

    /// <summary>The derivative of order Nu in U and Nv in V at (U, V).</summary>
    /// <exception cref="UndefinedDerivative3DException">Nu + Nv &lt; 1, Nu &lt; 0 or Nv &lt; 0.</exception>
    public sealed override Vector3D EvalDN(double u, double v, int nu, int nv)
    {
        Vector3D evalRepResult;
        if (RepUtils.TryEvalSurfaceDN(_evalRep, u, v, nu, nv, out evalRepResult))
        {
            return evalRepResult;
        }

        if (nu + nv < 1 || nu < 0 || nv < 0)
        {
            throw new UndefinedDerivative3DException();
        }

        Vector3D derivative;
        if (_uRational || _vRational)
        {
            SurfaceBSplSLib.DN(u,
                        v,
                        nu,
                        nv,
                        0,
                        0,
                        _poles,
                        _weights,
                        InternalUKnots(),
                        InternalUKnots(),
                        InternalUMultiplicities(),
                        InternalVMultiplicities(),
                        (_poles.ColLength() - 1),
                        (_poles.RowLength() - 1),
                        _uRational,
                        _vRational,
                        false,
                        false,
                        out derivative);
        }
        else
        {
            SurfaceBSplSLib.DN(u,
                        v,
                        nu,
                        nv,
                        0,
                        0,
                        _poles,
                        SurfaceBSplSLib.NoWeights(),
                        InternalUKnots(),
                        InternalUKnots(),
                        InternalUMultiplicities(),
                        InternalVMultiplicities(),
                        (_poles.ColLength() - 1),
                        (_poles.RowLength() - 1),
                        _uRational,
                        _vRational,
                        false,
                        false,
                        out derivative);
        }
        return derivative;
    }

    /// <summary>The number of poles in U (rows of the pole grid).</summary>
    public int UPoleCount => _poles.ColLength();

    /// <summary>The number of poles in V (columns of the pole grid).</summary>
    public int VPoleCount => _poles.RowLength();

    /// <summary>The pole (UIndex, VIndex).</summary>
    /// <exception cref="OutOfRangeException">An index out of the pole grid.</exception>
    public Point3D Pole(int uIndex, int vIndex)
    {
        Standard_OutOfRange_Raise_if(uIndex < 1 || uIndex > _poles.ColLength() || vIndex < 1
                                         || vIndex > _poles.RowLength(),
                                     " ");
        return _poles[uIndex, vIndex];
    }

    /// <summary>Copies the poles into P (deprecated in OCCT: use <see cref="Poles()"/>).</summary>
    /// <exception cref="DimensionErrorException">P is not NbUPoles x NbVPoles.</exception>
    internal void Poles(NCollection_Array2<Point3D> p)
    {
        Standard_DimensionError_Raise_if(p.RowLength() != _poles.RowLength() || p.ColLength() != _poles.ColLength(),
                                         " ");
        p.Assign(_poles);
    }

    /// <summary>The poles, NbUPoles rows by NbVPoles columns, indexed from 0 (a copy).</summary>
    public Grid<Point3D> Poles() => _poles.ToGrid();

    /// <summary>The degree in U: NbUPoles - 1.</summary>
    public int UDegree() => (_poles.ColLength() - 1);

    /// <summary>The U iso-curve (a Bezier curve in V) at U.</summary>
    public sealed override Curve3D UIso(double u)
    {
        BezierCurve3D uIsoCurve;
        NCollection_Array1<Point3D> vCurvePoles = new NCollection_Array1<Point3D>(1, _poles.RowLength());
        if (_uRational || _vRational)
        {
            NCollection_Array1<double> vCurveWeights = new NCollection_Array1<double>(1, _poles.RowLength());
            SurfaceBSplSLib.Iso(u,
                         true,
                         _poles,
                         _weights,
                         InternalUKnots(),
                         InternalUMultiplicities(),
                         (_poles.ColLength() - 1),
                         false,
                         vCurvePoles,
                         vCurveWeights);
            if (_uRational)
            {
                uIsoCurve = new BezierCurve3D(vCurvePoles, vCurveWeights);
            }
            else
            {
                uIsoCurve = new BezierCurve3D(vCurvePoles);
            }
        }
        else
        {
            SurfaceBSplSLib.Iso(u,
                         true,
                         _poles,
                         SurfaceBSplSLib.NoWeights(),
                         InternalUKnots(),
                         InternalUMultiplicities(),
                         (_poles.ColLength() - 1),
                         false,
                         vCurvePoles,
                         global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());
            uIsoCurve = new BezierCurve3D(vCurvePoles);
        }
        return uIsoCurve;
    }

    /// <summary>The degree in V: NbVPoles - 1.</summary>
    public int VDegree() => (_poles.RowLength() - 1);

    /// <summary>The V iso-curve (a Bezier curve in U) at V.</summary>
    public sealed override Curve3D VIso(double v)
    {
        BezierCurve3D vIsoCurve;
        // OCCT passes UKnots() as the V knots: the Bezier knots are (0, 1) in both directions.
        NCollection_Array1<Point3D> vCurvePoles = new NCollection_Array1<Point3D>(1, _poles.ColLength());
        if (_vRational || _uRational)
        {
            NCollection_Array1<double> vCurveWeights = new NCollection_Array1<double>(1, _poles.ColLength());
            SurfaceBSplSLib.Iso(v,
                         false,
                         _poles,
                         _weights,
                         InternalUKnots(),
                         InternalVMultiplicities(),
                         (_poles.RowLength() - 1),
                         false,
                         vCurvePoles,
                         vCurveWeights);
            if (_vRational)
            {
                vIsoCurve = new BezierCurve3D(vCurvePoles, vCurveWeights);
            }
            else
            {
                vIsoCurve = new BezierCurve3D(vCurvePoles);
            }
        }
        else
        {
            SurfaceBSplSLib.Iso(v,
                         false,
                         _poles,
                         SurfaceBSplSLib.NoWeights(),
                         InternalUKnots(),
                         InternalVMultiplicities(),
                         (_poles.RowLength() - 1),
                         false,
                         vCurvePoles,
                         global::Kerf.Engine.Numerics.Polynomials.PLib.NoWeights());
            vIsoCurve = new BezierCurve3D(vCurvePoles);
        }
        return vIsoCurve;
    }

    /// <summary>The weight of the pole (UIndex, VIndex): 1 for a non-rational surface.</summary>
    /// <exception cref="OutOfRangeException">An index out of the pole grid.</exception>
    public double Weight(int uIndex, int vIndex)
    {
        Standard_OutOfRange_Raise_if(uIndex < 1 || uIndex > _poles.ColLength() || vIndex < 1
                                         || vIndex > _poles.RowLength(),
                                     " ");
        if (_uRational || _vRational)
        {
            return _weights[uIndex, vIndex];
        }
        else
        {
            return 1;
        }
    }

    /// <summary>Copies the weights into W, all 1 for a non-rational surface (deprecated in OCCT).</summary>
    /// <exception cref="DimensionErrorException">W is not NbUPoles x NbVPoles.</exception>
    internal void Weights(NCollection_Array2<double> w)
    {
        Standard_DimensionError_Raise_if(w.RowLength() != _poles.RowLength() || w.ColLength() != _poles.ColLength(),
                                         " ");
        if (_uRational || _vRational)
        {
            w.Assign(_weights);
        }
        else
        {
            w.Init(1.0);
        }
    }

    /// <summary>The weights, indexed from 0 as <see cref="Poles()"/> (a copy), or null if the surface is non-rational
    /// in both directions.</summary>
    public Grid<double>? Weights() => (_uRational || _vRational) ? _weights.ToGrid() : null;

    /// <summary>The weights, all 1 for a non-rational surface, indexed from 0 as <see cref="Poles()"/> (a copy).</summary>
    public Grid<double> WeightsArray() => _weights.ToGrid();

    /// <summary>True.</summary>
    public sealed override bool IsCNu(int n) => true;

    /// <summary>True.</summary>
    public sealed override bool IsCNv(int n) => true;

    /// <summary>True if the weights vary in the U direction.</summary>
    public bool IsURational => _uRational;

    /// <summary>True if the weights vary in the V direction.</summary>
    public bool IsVRational => _vRational;

    /// <summary>Transforms the poles by T.</summary>
    protected internal sealed override void Transform(in Transform3D t)
    {
        CheckMutable();
        for (int i = 1; i <= _poles.ColLength(); i++)
        {
            for (int j = 1; j <= _poles.RowLength(); j++)
            {
                _poles[i, j] = _poles[i, j].Transformed(t);
            }
        }
        ClearEvalRepresentation();
        _maxDerivInvOk = false;
    }

    /// <summary>True if the first and last rows of poles coincide (within Precision::Confusion()).</summary>
    public sealed override bool IsUClosed
    {
        get
        {
            bool closed = true;
            int lower = _poles.LowerRow();
            int upper = _poles.UpperRow();
            int j = _poles.LowerCol();
            while (closed && j <= _poles.UpperCol())
            {
                closed = (_poles[lower, j].Distance(_poles[upper, j]) <= global::Kerf.Engine.Foundation.Precision.Confusion());
                j++;
            }
            return closed;
        }
    }

    /// <summary>True if the first and last columns of poles coincide (within Precision::Confusion()).</summary>
    public sealed override bool IsVClosed
    {
        get
        {
            bool closed = true;
            int lower = _poles.LowerCol();
            int upper = _poles.UpperCol();
            int i = _poles.LowerRow();
            while (closed && i <= _poles.UpperRow())
            {
                closed = (_poles[i, lower].Distance(_poles[i, upper]) <= global::Kerf.Engine.Foundation.Precision.Confusion());
                i++;
            }
            return closed;
        }
    }

    /// <summary>False.</summary>
    public sealed override bool IsUPeriodic => false;

    /// <summary>False.</summary>
    public sealed override bool IsVPeriodic => false;

    /// <summary>
    /// The parametric tolerances in U and V matching the 3D tolerance Tolerance3D (computed once and cached, as in
    /// OCCT; the cache is not part of the surface's value, so this works on a frozen surface).
    /// </summary>
    public void Resolution(double tolerance3D, out double uTolerance, out double vTolerance)
    {
        if (!_maxDerivInvOk)
        {
            double uMaxDerivInv, vMaxDerivInv;
            if (_uRational || _vRational)
            {
                SurfaceBSplSLib.Resolution(_poles,
                                    _weights,
                                    InternalUKnots(),
                                    InternalUKnots(),
                                    InternalUMultiplicities(),
                                    InternalVMultiplicities(),
                                    (_poles.ColLength() - 1),
                                    (_poles.RowLength() - 1),
                                    _uRational,
                                    _vRational,
                                    false,
                                    false,
                                    1.0,
                                    out uMaxDerivInv,
                                    out vMaxDerivInv);
            }
            else
            {
                SurfaceBSplSLib.Resolution(_poles,
                                    SurfaceBSplSLib.NoWeights(),
                                    InternalUKnots(),
                                    InternalUKnots(),
                                    InternalUMultiplicities(),
                                    InternalVMultiplicities(),
                                    (_poles.ColLength() - 1),
                                    (_poles.RowLength() - 1),
                                    _uRational,
                                    _vRational,
                                    false,
                                    false,
                                    1.0,
                                    out uMaxDerivInv,
                                    out vMaxDerivInv);
            }
            _uMaxDerivInv = uMaxDerivInv;
            _vMaxDerivInv = vMaxDerivInv;
            _maxDerivInvOk = true;
        }
        uTolerance = tolerance3D * _uMaxDerivInv;
        vTolerance = tolerance3D * _vMaxDerivInv;
    }

    /// <summary>A new, unfrozen surface equal to this one.</summary>
    public sealed override Geometry3D Copy() => new BezierSurface(this);

    /// <summary>Sets the poles and weights (null: non-rational) of a new surface.</summary>
    protected void Init(NCollection_Array2<Point3D> poles, NCollection_Array2<double>? weights)
    {
        CheckMutable();
        int nbUPoles = poles.ColLength();
        int nbVPoles = poles.RowLength();

        _poles.Resize(1, nbUPoles, 1, nbVPoles, false);
        _poles.CopyValues(poles);

        if (weights != null)
        {
            _weights.Resize(1, nbUPoles, 1, nbVPoles, false);
            _weights.CopyValues(weights);
            Rational(_weights, out _uRational, out _vRational);
            if (!(_uRational || _vRational))
            {
                _weights = SurfaceBSplSLib.UnitWeights(nbUPoles, nbVPoles);
            }
        }
        else
        {
            _uRational = false;
            _vRational = false;
            _weights = SurfaceBSplSLib.UnitWeights(nbUPoles, nbVPoles);
        }

        _maxDerivInvOk = false;
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
        int maxSize = global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree() + 1;
        NCollection_Array1<int>[] arr = new NCollection_Array1<int>[maxSize];
        for (int i = 0; i < maxSize; ++i)
        {
            arr[i] = NCollection_Array1<int>.From([i + 1, i + 1]);
        }
        return arr;
    }

    private static NCollection_Array1<double>?[] CreateFlatKnots()
    {
        int maxSize = global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree() + 1;
        NCollection_Array1<double>?[] arr = new NCollection_Array1<double>?[maxSize];
        for (int i = 1; i <= global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxDegree(); ++i)
        {
            arr[i] = NCollection_Array1<double>.From(global::Kerf.Engine.Geometry.Splines.BSplCLib.FlatBezierKnots(i));
        }
        return arr;
    }

    /// <summary>The U knots (0, 1): a shared static array, don't modify.</summary>
    internal NCollection_Array1<double> InternalUKnots() => THE_KNOTS;

    /// <summary>The V knots (0, 1): a shared static array, don't modify.</summary>
    internal NCollection_Array1<double> InternalVKnots() => InternalUKnots();

    /// <summary>The U multiplicities (UDegree + 1 twice): a shared static array, don't modify.</summary>
    internal NCollection_Array1<int> InternalUMultiplicities()
    {
        Standard_ProgramError_Raise_if(_poles.IsEmpty(), "Geom_BezierSurface: empty poles");
        return THE_MULTS[_poles.ColLength() - 1];
    }

    /// <summary>The V multiplicities (VDegree + 1 twice): a shared static array, don't modify.</summary>
    internal NCollection_Array1<int> InternalVMultiplicities()
    {
        Standard_ProgramError_Raise_if(_poles.IsEmpty(), "Geom_BezierSurface: empty poles");
        return THE_MULTS[_poles.RowLength() - 1];
    }

    /// <summary>The U flat knots (UDegree + 1 zeros and ones): a shared static array, don't modify.</summary>
    internal NCollection_Array1<double> InternalUKnotSequence()
    {
        Standard_ProgramError_Raise_if(_poles.IsEmpty(), "Geom_BezierSurface: empty poles");
        return THE_FKNOTS[_poles.ColLength() - 1]!;
    }

    /// <summary>The V flat knots (VDegree + 1 zeros and ones): a shared static array, don't modify.</summary>
    internal NCollection_Array1<double> InternalVKnotSequence()
    {
        Standard_ProgramError_Raise_if(_poles.IsEmpty(), "Geom_BezierSurface: empty poles");
        return THE_FKNOTS[_poles.RowLength() - 1]!;
    }

    /// <summary>The U knots of the equivalent B-spline surface: (0, 1).</summary>
    public IReadOnlyList<double> UKnots() => InternalUKnots().ToPackedList();

    /// <summary>The V knots of the equivalent B-spline surface: (0, 1).</summary>
    public IReadOnlyList<double> VKnots() => InternalVKnots().ToPackedList();

    /// <summary>The U multiplicities of the equivalent B-spline surface: (UDegree + 1, UDegree + 1).</summary>
    public IReadOnlyList<int> UMultiplicities() => InternalUMultiplicities().ToPackedList();

    /// <summary>The V multiplicities of the equivalent B-spline surface: (VDegree + 1, VDegree + 1).</summary>
    public IReadOnlyList<int> VMultiplicities() => InternalVMultiplicities().ToPackedList();

    /// <summary>The U flat knots of the equivalent B-spline surface.</summary>
    public IReadOnlyList<double> UKnotSequence() => InternalUKnotSequence().ToPackedList();

    /// <summary>The V flat knots of the equivalent B-spline surface.</summary>
    public IReadOnlyList<double> VKnotSequence() => InternalVKnotSequence().ToPackedList();

    //=============================================================================================
    // Internal access to the stored arrays and non-mutating twins (OCCT.NET)
    //=============================================================================================

    /// <summary>The stored poles: shared, don't modify (OCCT's <c>Poles()</c> const reference).</summary>
    internal NCollection_Array2<Point3D> InternalPoles() => _poles;

    /// <summary>The stored weights, or null for a non-rational surface (OCCT's <c>Weights()</c> pointer). Don't modify.</summary>
    internal NCollection_Array2<double>? InternalWeights() => (_uRational || _vRational) ? _weights : SurfaceBSplSLib.NoWeights();

    /// <summary>The weights, NbUPoles x NbVPoles of them (OCCT's <c>WeightsArray()</c>): shared, don't modify.</summary>
    internal NCollection_Array2<double> InternalWeightsArray() => _weights;

    private BezierSurface CopyBezier() => (BezierSurface)Copy();

    /// <summary>A copy with U and V exchanged.</summary>
    public BezierSurface ExchangedUV()
    {
        BezierSurface copy = CopyBezier();
        copy.ExchangeUV();
        return copy;
    }

    /// <summary>A copy with the degrees raised to UDeg and VDeg (see <see cref="Increase"/>).</summary>
    public BezierSurface Increased(int uDeg, int vDeg)
    {
        BezierSurface copy = CopyBezier();
        copy.Increase(uDeg, vDeg);
        return copy;
    }

    /// <summary>A copy with a column of poles inserted after VIndex.</summary>
    public BezierSurface PoleColInsertedAfter(int vIndex, NCollection_Array1<Point3D> cPoles)
    {
        BezierSurface copy = CopyBezier();
        copy.InsertPoleColAfter(vIndex, cPoles);
        return copy;
    }

    /// <summary>A copy with a column of weighted poles inserted after VIndex.</summary>
    public BezierSurface PoleColInsertedAfter(int vIndex, NCollection_Array1<Point3D> cPoles, NCollection_Array1<double> cPoleWeights)
    {
        BezierSurface copy = CopyBezier();
        copy.InsertPoleColAfter(vIndex, cPoles, cPoleWeights);
        return copy;
    }

    /// <summary>A copy with a column of poles inserted before VIndex.</summary>
    public BezierSurface PoleColInsertedBefore(int vIndex, NCollection_Array1<Point3D> cPoles)
    {
        BezierSurface copy = CopyBezier();
        copy.InsertPoleColBefore(vIndex, cPoles);
        return copy;
    }

    /// <summary>A copy with a column of weighted poles inserted before VIndex.</summary>
    public BezierSurface PoleColInsertedBefore(int vIndex, NCollection_Array1<Point3D> cPoles, NCollection_Array1<double> cPoleWeights)
    {
        BezierSurface copy = CopyBezier();
        copy.InsertPoleColBefore(vIndex, cPoles, cPoleWeights);
        return copy;
    }

    /// <summary>A copy with a row of poles inserted after UIndex.</summary>
    public BezierSurface PoleRowInsertedAfter(int uIndex, NCollection_Array1<Point3D> cPoles)
    {
        BezierSurface copy = CopyBezier();
        copy.InsertPoleRowAfter(uIndex, cPoles);
        return copy;
    }

    /// <summary>A copy with a row of weighted poles inserted after UIndex.</summary>
    public BezierSurface PoleRowInsertedAfter(int uIndex, NCollection_Array1<Point3D> cPoles, NCollection_Array1<double> cPoleWeights)
    {
        BezierSurface copy = CopyBezier();
        copy.InsertPoleRowAfter(uIndex, cPoles, cPoleWeights);
        return copy;
    }

    /// <summary>A copy with a row of poles inserted before UIndex.</summary>
    public BezierSurface PoleRowInsertedBefore(int uIndex, NCollection_Array1<Point3D> cPoles)
    {
        BezierSurface copy = CopyBezier();
        copy.InsertPoleRowBefore(uIndex, cPoles);
        return copy;
    }

    /// <summary>A copy with a row of weighted poles inserted before UIndex.</summary>
    public BezierSurface PoleRowInsertedBefore(int uIndex, NCollection_Array1<Point3D> cPoles, NCollection_Array1<double> cPoleWeights)
    {
        BezierSurface copy = CopyBezier();
        copy.InsertPoleRowBefore(uIndex, cPoles, cPoleWeights);
        return copy;
    }

    /// <summary>A copy without the column of poles VIndex.</summary>
    public BezierSurface PoleColRemoved(int vIndex)
    {
        BezierSurface copy = CopyBezier();
        copy.RemovePoleCol(vIndex);
        return copy;
    }

    /// <summary>A copy without the row of poles UIndex.</summary>
    public BezierSurface PoleRowRemoved(int uIndex)
    {
        BezierSurface copy = CopyBezier();
        copy.RemovePoleRow(uIndex);
        return copy;
    }

    /// <summary>A copy restricted to [U1, U2] x [V1, V2] (see <see cref="Segment"/>).</summary>
    public BezierSurface Segmented(double u1, double u2, double v1, double v2)
    {
        BezierSurface copy = CopyBezier();
        copy.Segment(u1, u2, v1, v2);
        return copy;
    }

    public BezierSurface WithPole(int uIndex, int vIndex, in Point3D p)
    {
        BezierSurface copy = CopyBezier();
        copy.SetPole(uIndex, vIndex, p);
        return copy;
    }

    public BezierSurface WithPole(int uIndex, int vIndex, in Point3D p, double weight)
    {
        BezierSurface copy = CopyBezier();
        copy.SetPole(uIndex, vIndex, p, weight);
        return copy;
    }

    public BezierSurface WithPoleCol(int vIndex, NCollection_Array1<Point3D> cPoles)
    {
        BezierSurface copy = CopyBezier();
        copy.SetPoleCol(vIndex, cPoles);
        return copy;
    }

    public BezierSurface WithPoleCol(int vIndex, NCollection_Array1<Point3D> cPoles, NCollection_Array1<double> cPoleWeights)
    {
        BezierSurface copy = CopyBezier();
        copy.SetPoleCol(vIndex, cPoles, cPoleWeights);
        return copy;
    }

    public BezierSurface WithPoleRow(int uIndex, NCollection_Array1<Point3D> cPoles)
    {
        BezierSurface copy = CopyBezier();
        copy.SetPoleRow(uIndex, cPoles);
        return copy;
    }

    public BezierSurface WithPoleRow(int uIndex, NCollection_Array1<Point3D> cPoles, NCollection_Array1<double> cPoleWeights)
    {
        BezierSurface copy = CopyBezier();
        copy.SetPoleRow(uIndex, cPoles, cPoleWeights);
        return copy;
    }

    public BezierSurface WithWeight(int uIndex, int vIndex, double weight)
    {
        BezierSurface copy = CopyBezier();
        copy.SetWeight(uIndex, vIndex, weight);
        return copy;
    }

    public BezierSurface WithWeightCol(int vIndex, NCollection_Array1<double> cPoleWeights)
    {
        BezierSurface copy = CopyBezier();
        copy.SetWeightCol(vIndex, cPoleWeights);
        return copy;
    }

    public BezierSurface WithWeightRow(int uIndex, NCollection_Array1<double> cPoleWeights)
    {
        BezierSurface copy = CopyBezier();
        copy.SetWeightRow(uIndex, cPoleWeights);
        return copy;
    }

    /// <summary>A copy evaluated through a copy of theDesc (null: without a representation).</summary>
    public BezierSurface WithEvalRepresentation(RepSurfaceDesc.Base? desc)
    {
        RepUtils.ValidateSurfaceDesc(desc, this);
        BezierSurface copy = CopyBezier();
        copy.SetEvalRepresentation(RepUtils.CloneSurfaceDesc(desc));
        return copy;
    }
}
