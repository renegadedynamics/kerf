// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_BSplineSurfaceKnotSplitting.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_BSplineSurfaceKnotSplitting.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// Computes the knots of a B-spline surface at which it must be split to obtain patches of a given continuity
/// in each parametric direction. The split values are knot indices, the first and last ones included.
/// </summary>
internal sealed class BSplineSurfaceKnotSplitting
{
    private NCollection_HArray1<int>? _usplitIndexes;
    private NCollection_HArray1<int>? _vsplitIndexes;

    /// <summary>
    /// Determines the U and V knots at which BasisSurface is less than UContinuityRange (resp.
    /// VContinuityRange) times continuously differentiable. A range of 0 keeps only the end knots.
    /// </summary>
    /// <exception cref="RangeErrorException">A negative continuity range.</exception>
    /// <remarks>
    /// OCCT: when the surface is already of continuity VContinuityRange in V, the V result is written into the U
    /// split indices (see occt-issues.md); NbVSplits, VSplitValue and Splitting then throw.
    /// </remarks>
    public BSplineSurfaceKnotSplitting(BSplineSurface basisSurface,
                                                   int uContinuityRange,
                                                   int vContinuityRange)
    {
        if (uContinuityRange < 0 || vContinuityRange < 0)
        {
            throw new RangeErrorException();
        }

        int firstUIndex = basisSurface.FirstUKnotIndex();
        int lastUIndex = basisSurface.LastUKnotIndex();
        int firstVIndex = basisSurface.FirstVKnotIndex();
        int lastVIndex = basisSurface.LastVKnotIndex();
        int uDegree = basisSurface.UDegree;
        int vDegree = basisSurface.VDegree;
        int i;

        if (uContinuityRange == 0)
        {
            _usplitIndexes = new NCollection_HArray1<int>(1, 2);
            _usplitIndexes.SetValue(1, firstUIndex);
            _usplitIndexes.SetValue(2, lastUIndex);
        }
        else
        {
            NCollection_Array1<int> uMults = basisSurface.InternalUMultiplicities();
            int mmax = global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxKnotMult(uMults, firstUIndex, lastUIndex);
            if (uDegree - mmax >= uContinuityRange)
            {
                _usplitIndexes = new NCollection_HArray1<int>(1, 2);
                _usplitIndexes.SetValue(1, firstUIndex);
                _usplitIndexes.SetValue(2, lastUIndex);
            }
            else
            {
                var uSplit = new NCollection_Array1<int>(1, lastUIndex - firstUIndex + 1);
                int nbUSplit = 1;
                int uIndex = firstUIndex;
                uSplit[nbUSplit] = uIndex;
                uIndex++;
                nbUSplit++;
                while (uIndex < lastUIndex)
                {
                    if (uDegree - uMults[uIndex] < uContinuityRange)
                    {
                        uSplit[nbUSplit] = uIndex;
                        nbUSplit++;
                    }

                    uIndex++;
                }

                uSplit[nbUSplit] = uIndex;
                _usplitIndexes = new NCollection_HArray1<int>(1, nbUSplit);
                for (i = 1; i <= nbUSplit; i++)
                {
                    _usplitIndexes.SetValue(i, uSplit[i]);
                }
            }
        }

        if (vContinuityRange == 0)
        {
            _vsplitIndexes = new NCollection_HArray1<int>(1, 2);
            _vsplitIndexes.SetValue(1, firstVIndex);
            _vsplitIndexes.SetValue(2, lastVIndex);
        }
        else
        {
            NCollection_Array1<int> vMults = basisSurface.InternalVMultiplicities();
            int mmax = global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxKnotMult(vMults, firstVIndex, lastVIndex);
            if (vDegree - mmax >= vContinuityRange)
            {
                // OCCT writes the V end indices into the U split indices; the V ones stay null
                _usplitIndexes = new NCollection_HArray1<int>(1, 2);
                _usplitIndexes.SetValue(1, firstVIndex);
                _usplitIndexes.SetValue(2, lastVIndex);
            }
            else
            {
                var vSplit = new NCollection_Array1<int>(1, lastVIndex - firstVIndex + 1);
                int nbVSplit = 1;
                int vIndex = firstVIndex;
                vSplit[nbVSplit] = vIndex;
                vIndex++;
                nbVSplit++;
                while (vIndex < lastVIndex)
                {
                    if (vDegree - vMults[vIndex] < vContinuityRange)
                    {
                        vSplit[nbVSplit] = vIndex;
                        nbVSplit++;
                    }

                    vIndex++;
                }

                vSplit[nbVSplit] = vIndex;
                _vsplitIndexes = new NCollection_HArray1<int>(1, nbVSplit);
                for (i = 1; i <= nbVSplit; i++)
                {
                    _vsplitIndexes.SetValue(i, vSplit[i]);
                }
            }
        }
    }

    /// <summary>The number of U split knots (at least 2: the first and last knots).</summary>
    public int USplitCount => _usplitIndexes!.Length();

    /// <summary>The number of V split knots.</summary>
    /// <exception cref="NullReferenceException">After OCCT's error described on the constructor.</exception>
    public int VSplitCount => _vsplitIndexes!.Length(); // OCCT: null when the V branch wrote into usplitIndexes

    /// <summary>Copies the U and V split knot indices into USplit and VSplit (from index 1).</summary>
    public void Splitting(NCollection_Array1<int> uSplit, NCollection_Array1<int> vSplit)
    {
        int i;
        for (i = 1; i <= _usplitIndexes!.Length(); i++)
        {
            uSplit[i] = _usplitIndexes.Value(i);
        }

        for (i = 1; i <= _vsplitIndexes!.Length(); i++)
        {
            vSplit[i] = _vsplitIndexes.Value(i);
        }
    }

    /// <summary>The knot index of the U split UIndex.</summary>
    /// <exception cref="RangeErrorException">UIndex out of [1, NbUSplits].</exception>
    public int USplitValue(int uIndex)
    {
        Standard_RangeError_Raise_if(uIndex < 1 || uIndex > _usplitIndexes!.Length(), " ");
        return _usplitIndexes!.Value(uIndex);
    }

    /// <summary>The knot index of the V split VIndex.</summary>
    /// <exception cref="RangeErrorException">VIndex out of [1, NbVSplits].</exception>
    public int VSplitValue(int vIndex)
    {
        Standard_RangeError_Raise_if(vIndex < 1 || vIndex > _vsplitIndexes!.Length(), " ");
        return _vsplitIndexes!.Value(vIndex);
    }
}
