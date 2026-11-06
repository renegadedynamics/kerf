// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitSurfaceArea.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitSurfaceArea.lxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeUpgrade/ShapeUpgrade_SplitSurfaceArea.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;

namespace Kerf.Engine.Healing.Upgrade;

/// <summary>
/// Splits a surface into a number of parts of about the same area: into strips along its longer direction, or, in
/// the "splitting into squares" mode, into parts close to squares in 2D (or into fixed numbers of U and V parts).
/// </summary>
/// <remarks>A working object (decision 21). OCCT's modifiable reference <see cref="NbParts"/> returns <c>ref</c>.</remarks>
internal class SplitSurfaceArea : SplitSurface
{
    private int _nbParts;
    private int _unbSplit;
    private int _vnbSplit;
    private bool _isSplittingIntoSquares;

    /// <summary>Empty constructor (one part).</summary>
    public SplitSurfaceArea()
    {
        _nbParts = 1;
        _unbSplit = _vnbSplit = -1;
        _isSplittingIntoSquares = false;
    }

    /// <summary>The number of parts to split into.</summary>
    public ref int NbParts() => ref _nbParts;

    /// <summary>
    /// Sets the splitting mode: "splitting into squares" splits the face into about <see cref="NbParts"/> parts close
    /// to squares in 2D.
    /// </summary>
    public void SetSplittingIntoSquares(bool isSplittingIntoSquares)
    {
        _isSplittingIntoSquares = isSplittingIntoSquares;
    }

    /// <summary>Sets fixed numbers of splits in U and V (only in the "splitting into squares" mode).</summary>
    public void SetNumbersUVSplits(int nbUsplits, int nbVsplits)
    {
        _unbSplit = nbUsplits;
        _vnbSplit = nbVsplits;
        if (_unbSplit > 0 && _vnbSplit > 0)
        {
            _nbParts = _unbSplit * _vnbSplit;
        }
    }

    /// <summary>Computes the U and V splitting values.</summary>
    public override void Compute(bool segment = true)
    {
        if (_nbParts <= 1)
        {
            return;
        }

        var ads = new SurfaceAdaptor(_surface!,
                                          _uSplitValues!.Value(1),
                                          _uSplitValues.Value(2),
                                          _vSplitValues!.Value(1),
                                          _vSplitValues.Value(2));
        double koefU = ads.UResolution(1.0);
        double koefV = ads.VResolution(1.0);
        if (koefU == 0)
        {
            koefU = 1.0;
        }
        if (koefV == 0)
        {
            koefV = 1.0;
        }
        double uSize = Math.Abs(_uSplitValues.Value(2) - _uSplitValues.Value(1)) / koefU;
        double vSize = Math.Abs(_vSplitValues.Value(2) - _vSplitValues.Value(1)) / koefV;
        double nbUV = uSize / vSize;
        NCollection_HSequence<double> firstSplit = (nbUV < 1.0 ? _vSplitValues : _uSplitValues);
        NCollection_HSequence<double> secondSplit = (nbUV < 1.0 ? _uSplitValues : _vSplitValues);
        bool isUFirst = (nbUV > 1.0);
        if (nbUV < 1)
        {
            nbUV = 1.0 / nbUV;
        }

        bool isFixedUVnbSplits = (_unbSplit > 0 && _vnbSplit > 0);
        int nbSplitF, nbSplitS;
        if (_isSplittingIntoSquares && _nbParts > 0)
        {
            if (!isFixedUVnbSplits) //(myUnbSplit <= 0 || myVnbSplit <= 0)
            {
                double squareSize = Math.Sqrt(_area / _nbParts);
                _unbSplit = (int)(_usize / squareSize);
                _vnbSplit = (int)(_vsize / squareSize);
                if (_unbSplit == 0)
                {
                    _unbSplit = 1;
                }
                if (_vnbSplit == 0)
                {
                    _vnbSplit = 1;
                }
            }

            if (isUFirst)
            {
                nbSplitF = _unbSplit;
                nbSplitS = _vnbSplit;
            }
            else
            {
                nbSplitF = _vnbSplit;
                nbSplitS = _unbSplit;
            }
        }
        else
        {
            nbSplitF = (nbUV >= _nbParts ? _nbParts : RealToInt(Math.Ceiling(Math.Sqrt(_nbParts * Math.Ceiling(nbUV)))));
            nbSplitS = (nbUV >= _nbParts ? 0 : RealToInt(Math.Ceiling((double)_nbParts / (double)nbSplitF)));
        }
        if (nbSplitS == 1 && !isFixedUVnbSplits)
        {
            nbSplitS++;
        }
        if (nbSplitF == 0)
        {
            return;
        }
        double step = (firstSplit.Value(2) - firstSplit.Value(1)) / nbSplitF;
        double prevPar = firstSplit.Value(1);
        int i = 1;
        for (; i < nbSplitF; i++)
        {
            double nextPar = prevPar + step;
            firstSplit.InsertBefore(i + 1, nextPar);
            prevPar = nextPar;
        }

        if (nbSplitS != 0)
        {
            step = (secondSplit.Value(2) - secondSplit.Value(1)) / nbSplitS;
            prevPar = secondSplit.Value(1);
            for (i = 1; i < nbSplitS; i++)
            {
                double nextPar = prevPar + step;
                secondSplit.InsertBefore(i + 1, nextPar);
                prevPar = nextPar;
            }
        }
    }
}
