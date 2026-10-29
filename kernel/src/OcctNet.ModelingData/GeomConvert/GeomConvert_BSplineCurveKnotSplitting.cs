// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_BSplineCurveKnotSplitting.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_BSplineCurveKnotSplitting.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The knots at which a 3D B-spline curve must be split to obtain arcs of a given continuity. At a knot, the
/// continuity of a B-spline is its degree minus the knot's multiplicity. The split points are knot indices, the
/// first and last knots of the curve included; GeomConvert.SplitBSplineCurve splits the curve between two of them.
/// </summary>
public sealed class GeomConvert_BSplineCurveKnotSplitting
{
    private readonly NCollection_HArray1<int> splitIndexes;

    /// <summary>The knots at which BasisCurve must be split to obtain arcs of continuity ContinuityRange.</summary>
    /// <exception cref="Standard_RangeError">ContinuityRange is negative.</exception>
    public GeomConvert_BSplineCurveKnotSplitting(Geom_BSplineCurve BasisCurve, int ContinuityRange)
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
            NCollection_Array1<int> Mults = BasisCurve.InternalMultiplicities();
            int Mmax = BSplCLib.MaxKnotMult(Mults, FirstIndex, LastIndex);
            if (Degree - Mmax >= ContinuityRange)
            {
                splitIndexes = new NCollection_HArray1<int>(1, 2);
                splitIndexes.SetValue(1, FirstIndex);
                splitIndexes.SetValue(2, LastIndex);
            }
            else
            {
                NCollection_Array1<int> Split = new(1, LastIndex - FirstIndex + 1);
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

    /// <summary>The number of split points, the first and last knots of the curve included.</summary>
    public int NbSplits() => splitIndexes.Length();

    /// <summary>
    /// Copies the split knot indices, in ascending order, into SplitValues (indexed from 1 to NbSplits(); OCCT
    /// documents a Standard_DimensionError for other bounds but doesn't check them).
    /// </summary>
    public void Splitting(NCollection_Array1<int> SplitValues)
    {
        for (int i = 1; i <= splitIndexes.Length(); i++)
        {
            SplitValues[i] = splitIndexes.Value(i);
        }
    }

    /// <summary>The knot index of split point Index (1 to NbSplits()).</summary>
    /// <exception cref="Standard_RangeError">Index is out of range.</exception>
    public int SplitValue(int Index)
    {
        Standard_RangeError_Raise_if(Index < 1 || Index > splitIndexes.Length(), " ");
        return splitIndexes.Value(Index);
    }
}
