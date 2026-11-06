// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_BSplineKnotSplitting.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_BSplineKnotSplitting.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Kerf.Engine.Construction.Laws;

/// <summary>
/// The indexes of the knots where a 1D B-spline should be split to get arcs of a given continuity (at a knot the
/// continuity is Degree - Mult(Index)). The splits are in [FirstUKnotIndex, LastUKnotIndex].
/// </summary>
internal sealed class BSplineKnotSplitting
{
    private readonly NCollection_HArray1<int> _splitIndexes;

    /// <summary>Locates the knots that split the curve into arcs of continuity ContinuityRange.</summary>
    /// <exception cref="RangeErrorException">ContinuityRange is negative.</exception>
    public BSplineKnotSplitting(BSpline basisCurve, int continuityRange)
    {
        if (continuityRange < 0)
        {
            throw new RangeErrorException();
        }

        int firstIndex = basisCurve.FirstUKnotIndex();
        int lastIndex = basisCurve.LastUKnotIndex();
        int degree = basisCurve.Degree();

        if (continuityRange == 0)
        {
            _splitIndexes = new NCollection_HArray1<int>(1, 2);
            _splitIndexes.SetValue(1, firstIndex);
            _splitIndexes.SetValue(2, lastIndex);
        }
        else
        {
            int nbKnots = basisCurve.KnotCount;
            NCollection_Array1<int> mults = new NCollection_Array1<int>(1, nbKnots);
            basisCurve.Multiplicities(mults);
            int mmax = global::Kerf.Engine.Geometry.Splines.BSplCLib.MaxKnotMult(mults, firstIndex, lastIndex);
            if (degree - mmax >= continuityRange)
            {
                _splitIndexes = new NCollection_HArray1<int>(1, 2);
                _splitIndexes.SetValue(1, firstIndex);
                _splitIndexes.SetValue(2, lastIndex);
            }
            else
            {
                NCollection_Array1<int> split = new NCollection_Array1<int>(1, lastIndex - firstIndex + 1);
                int nbSplit = 1;
                int index = firstIndex;
                split[nbSplit] = index;
                index++;
                nbSplit++;
                while (index < lastIndex)
                {
                    if (degree - mults[index] < continuityRange)
                    {
                        split[nbSplit] = index;
                        nbSplit++;
                    }
                    index++;
                }
                split[nbSplit] = index;
                _splitIndexes = new NCollection_HArray1<int>(1, nbSplit);
                for (int i = 1; i <= nbSplit; i++)
                {
                    _splitIndexes.SetValue(i, split[i]);
                }
            }
        }
    }

    /// <summary>The number of knots of the splitting.</summary>
    public int SplitCount => _splitIndexes.Length();

    /// <summary>Copies the knot indexes of the splitting into SplitValues (from index 1).</summary>
    public void Splitting(NCollection_Array1<int> splitValues)
    {
        for (int i = 1; i <= _splitIndexes.Length(); i++)
        {
            splitValues[i] = _splitIndexes.Value(i);
        }
    }

    /// <summary>The knot index of the split Index.</summary>
    /// <exception cref="RangeErrorException">Index is outside [1, NbSplits()].</exception>
    public int SplitValue(int index)
    {
        Standard_RangeError_Raise_if(index < 1 || index > _splitIndexes.Length(), " ");
        return _splitIndexes.Value(index);
    }
}
