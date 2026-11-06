// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyShellData.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAlgo/HLRAlgo_PolyShellData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Bounds;

namespace Kerf.Engine.HiddenLines.Algorithm;

/// <summary>The face data of a shell for the polygonal hidden-line removal, its hiding faces and its segments.</summary>
/// <remarks>A working object (a transient), as OCCT's.</remarks>
internal class PolyShellData : Transient
{
    /// <summary>The shell's min-max indices (plain data).</summary>
    public struct ShellIndices
    {
        public int Min;
        public int Max;
    }

    private ShellIndices _indices;
    private readonly NCollection_Array1<PolyData?> _polyg;
    private NCollection_Array1<PolyData?> _hPolHi;
    private readonly NCollection_List<AlgoBiPoint> _segList = new();

    public PolyShellData(int nbFace)
    {
        _polyg = new NCollection_Array1<PolyData?>(1, nbFace);
        _hPolHi = new NCollection_Array1<PolyData?>();
    }

    /// <summary>Adds the segments' projected points and the faces' hiding nodes to <paramref name="box"/>.</summary>
    public void UpdateGlobalMinMax(ref Box box)
    {
        for (NCollection_List<AlgoBiPoint>.Iterator it = new(_segList); it.More(); it.Next())
        {
            AlgoBiPoint.PointsT points = it.Value().PointsValue();
            box.Update(points.PntP1.X(), points.PntP1.Y(), points.PntP1.Z());
            box.Update(points.PntP2.X(), points.PntP2.Y(), points.PntP2.Z());
        }
        for (int i = _polyg.Lower(); i <= _polyg.Upper(); i++)
        {
            PolyData pd = _polyg[i]!;
            pd.UpdateGlobalMinMax(ref box);
        }
    }

    /// <summary>Sizes the array of hiding faces to <paramref name="nbHiding"/> (empty if none).</summary>
    public void UpdateHiding(int nbHiding)
    {
        if (nbHiding > 0)
        {
            _hPolHi.Resize(1, nbHiding, false);
        }
        else
        {
            _hPolHi = new NCollection_Array1<PolyData?>();
        }
    }

    public bool Hiding() => !_hPolHi.IsEmpty();

    public NCollection_Array1<PolyData?> PolyData() => _polyg;

    public NCollection_Array1<PolyData?> HidingPolyData() => _hPolHi;

    public NCollection_List<AlgoBiPoint> Edges() => _segList;

    public ref ShellIndices Indices() => ref _indices;
}
