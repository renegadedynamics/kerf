// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_BSplineKnotSplitting.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Law/Law_BSplineKnotSplitting.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The indexes of the knots where a 1D B-spline should be split to get arcs of a given continuity (at a knot the
/// continuity is Degree - Mult(Index)). The splits are in [FirstUKnotIndex, LastUKnotIndex].
/// </summary>
public sealed class Law_BSplineKnotSplitting
{
    private readonly NCollection_HArray1<int> splitIndexes;

    /// <summary>Locates the knots that split the curve into arcs of continuity ContinuityRange.</summary>
    /// <exception cref="Standard_RangeError">ContinuityRange is negative.</exception>
    public Law_BSplineKnotSplitting(Law_BSpline BasisCurve, int ContinuityRange)
    {
        if (ContinuityRange < 0)
        {
            throw new Standard_RangeError();
        }

        int FirstIndex = BasisCurve.FirstUKnotIndex();
        int LastIndex = BasisCurve.LastUKnotIndex();
        int Degree = BasisCurve.Degree();

        if (ContinuityRange == 0)
        {
            splitIndexes = new NCollection_HArray1<int>(1, 2);
            splitIndexes.SetValue(1, FirstIndex);
            splitIndexes.SetValue(2, LastIndex);
        }
        else
        {
            int NbKnots = BasisCurve.NbKnots();
            NCollection_Array1<int> Mults = new NCollection_Array1<int>(1, NbKnots);
            BasisCurve.Multiplicities(Mults);
            int Mmax = BSplCLib.MaxKnotMult(Mults, FirstIndex, LastIndex);
            if (Degree - Mmax >= ContinuityRange)
            {
                splitIndexes = new NCollection_HArray1<int>(1, 2);
                splitIndexes.SetValue(1, FirstIndex);
                splitIndexes.SetValue(2, LastIndex);
            }
            else
            {
                NCollection_Array1<int> Split = new NCollection_Array1<int>(1, LastIndex - FirstIndex + 1);
                int NbSplit = 1;
                int Index = FirstIndex;
                Split[NbSplit] = Index;
                Index++;
                NbSplit++;
                while (Index < LastIndex)
                {
                    if (Degree - Mults[Index] < ContinuityRange)
                    {
                        Split[NbSplit] = Index;
                        NbSplit++;
                    }
                    Index++;
                }
                Split[NbSplit] = Index;
                splitIndexes = new NCollection_HArray1<int>(1, NbSplit);
                for (int i = 1; i <= NbSplit; i++)
                {
                    splitIndexes.SetValue(i, Split[i]);
                }
            }
        }
    }

    /// <summary>The number of knots of the splitting.</summary>
    public int NbSplits() => splitIndexes.Length();

    /// <summary>Copies the knot indexes of the splitting into SplitValues (from index 1).</summary>
    public void Splitting(NCollection_Array1<int> SplitValues)
    {
        for (int i = 1; i <= splitIndexes.Length(); i++)
        {
            SplitValues[i] = splitIndexes.Value(i);
        }
    }

    /// <summary>The knot index of the split Index.</summary>
    /// <exception cref="Standard_RangeError">Index is outside [1, NbSplits()].</exception>
    public int SplitValue(int Index)
    {
        Standard_RangeError_Raise_if(Index < 1 || Index > splitIndexes.Length(), " ");
        return splitIndexes.Value(Index);
    }
}
