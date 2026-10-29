// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_BSplineSurfaceKnotSplitting.hxx
// Port of: src/ModelingData/TKGeomBase/GeomConvert/GeomConvert_BSplineSurfaceKnotSplitting.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Computes the knots of a B-spline surface at which it must be split to obtain patches of a given continuity
/// in each parametric direction. The split values are knot indices, the first and last ones included.
/// </summary>
public sealed class GeomConvert_BSplineSurfaceKnotSplitting
{
    private NCollection_HArray1<int>? usplitIndexes;
    private NCollection_HArray1<int>? vsplitIndexes;

    /// <summary>
    /// Determines the U and V knots at which BasisSurface is less than UContinuityRange (resp.
    /// VContinuityRange) times continuously differentiable. A range of 0 keeps only the end knots.
    /// </summary>
    /// <exception cref="Standard_RangeError">A negative continuity range.</exception>
    /// <remarks>
    /// OCCT: when the surface is already of continuity VContinuityRange in V, the V result is written into the U
    /// split indices (see occt-issues.md); NbVSplits, VSplitValue and Splitting then throw.
    /// </remarks>
    public GeomConvert_BSplineSurfaceKnotSplitting(Geom_BSplineSurface BasisSurface,
                                                   int UContinuityRange,
                                                   int VContinuityRange)
    {
        if (UContinuityRange < 0 || VContinuityRange < 0)
        {
            throw new Standard_RangeError();
        }

        int FirstUIndex = BasisSurface.FirstUKnotIndex();
        int LastUIndex = BasisSurface.LastUKnotIndex();
        int FirstVIndex = BasisSurface.FirstVKnotIndex();
        int LastVIndex = BasisSurface.LastVKnotIndex();
        int UDegree = BasisSurface.UDegree();
        int VDegree = BasisSurface.VDegree();
        int i;

        if (UContinuityRange == 0)
        {
            usplitIndexes = new NCollection_HArray1<int>(1, 2);
            usplitIndexes.SetValue(1, FirstUIndex);
            usplitIndexes.SetValue(2, LastUIndex);
        }
        else
        {
            NCollection_Array1<int> UMults = BasisSurface.InternalUMultiplicities();
            int Mmax = BSplCLib.MaxKnotMult(UMults, FirstUIndex, LastUIndex);
            if (UDegree - Mmax >= UContinuityRange)
            {
                usplitIndexes = new NCollection_HArray1<int>(1, 2);
                usplitIndexes.SetValue(1, FirstUIndex);
                usplitIndexes.SetValue(2, LastUIndex);
            }
            else
            {
                var USplit = new NCollection_Array1<int>(1, LastUIndex - FirstUIndex + 1);
                int NbUSplit = 1;
                int UIndex = FirstUIndex;
                USplit[NbUSplit] = UIndex;
                UIndex++;
                NbUSplit++;
                while (UIndex < LastUIndex)
                {
                    if (UDegree - UMults[UIndex] < UContinuityRange)
                    {
                        USplit[NbUSplit] = UIndex;
                        NbUSplit++;
                    }

                    UIndex++;
                }

                USplit[NbUSplit] = UIndex;
                usplitIndexes = new NCollection_HArray1<int>(1, NbUSplit);
                for (i = 1; i <= NbUSplit; i++)
                {
                    usplitIndexes.SetValue(i, USplit[i]);
                }
            }
        }

        if (VContinuityRange == 0)
        {
            vsplitIndexes = new NCollection_HArray1<int>(1, 2);
            vsplitIndexes.SetValue(1, FirstVIndex);
            vsplitIndexes.SetValue(2, LastVIndex);
        }
        else
        {
            NCollection_Array1<int> VMults = BasisSurface.InternalVMultiplicities();
            int Mmax = BSplCLib.MaxKnotMult(VMults, FirstVIndex, LastVIndex);
            if (VDegree - Mmax >= VContinuityRange)
            {
                // OCCT writes the V end indices into the U split indices; the V ones stay null
                usplitIndexes = new NCollection_HArray1<int>(1, 2);
                usplitIndexes.SetValue(1, FirstVIndex);
                usplitIndexes.SetValue(2, LastVIndex);
            }
            else
            {
                var VSplit = new NCollection_Array1<int>(1, LastVIndex - FirstVIndex + 1);
                int NbVSplit = 1;
                int VIndex = FirstVIndex;
                VSplit[NbVSplit] = VIndex;
                VIndex++;
                NbVSplit++;
                while (VIndex < LastVIndex)
                {
                    if (VDegree - VMults[VIndex] < VContinuityRange)
                    {
                        VSplit[NbVSplit] = VIndex;
                        NbVSplit++;
                    }

                    VIndex++;
                }

                VSplit[NbVSplit] = VIndex;
                vsplitIndexes = new NCollection_HArray1<int>(1, NbVSplit);
                for (i = 1; i <= NbVSplit; i++)
                {
                    vsplitIndexes.SetValue(i, VSplit[i]);
                }
            }
        }
    }

    /// <summary>The number of U split knots (at least 2: the first and last knots).</summary>
    public int NbUSplits() => usplitIndexes!.Length();

    /// <summary>The number of V split knots.</summary>
    /// <exception cref="NullReferenceException">After OCCT's error described on the constructor.</exception>
    public int NbVSplits() => vsplitIndexes!.Length(); // OCCT: null when the V branch wrote into usplitIndexes

    /// <summary>Copies the U and V split knot indices into USplit and VSplit (from index 1).</summary>
    public void Splitting(NCollection_Array1<int> USplit, NCollection_Array1<int> VSplit)
    {
        int i;
        for (i = 1; i <= usplitIndexes!.Length(); i++)
        {
            USplit[i] = usplitIndexes.Value(i);
        }

        for (i = 1; i <= vsplitIndexes!.Length(); i++)
        {
            VSplit[i] = vsplitIndexes.Value(i);
        }
    }

    /// <summary>The knot index of the U split UIndex.</summary>
    /// <exception cref="Standard_RangeError">UIndex out of [1, NbUSplits].</exception>
    public int USplitValue(int UIndex)
    {
        Standard_RangeError_Raise_if(UIndex < 1 || UIndex > usplitIndexes!.Length(), " ");
        return usplitIndexes!.Value(UIndex);
    }

    /// <summary>The knot index of the V split VIndex.</summary>
    /// <exception cref="Standard_RangeError">VIndex out of [1, NbVSplits].</exception>
    public int VSplitValue(int VIndex)
    {
        Standard_RangeError_Raise_if(VIndex < 1 || VIndex > vsplitIndexes!.Length(), " ");
        return vsplitIndexes!.Value(VIndex);
    }
}
