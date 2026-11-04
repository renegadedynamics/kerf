// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Curve.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_Curve.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Intersection.Tools;
using Occt;

namespace Kerf.Engine.Booleans.Data;

/// <summary>
/// An intersection curve of two faces: the curve, its box, its pave blocks, its technological vertices and its
/// tolerance.
/// </summary>
/// <remarks>
/// A class, a working object held in <see cref="InterfFF.ChangeCurves"/> and changed in place (decision
/// 6151). OCCT's by-value copies are the copy constructor, which copies the lists (sharing the pave blocks, as the
/// C++ copy shares the handles).
/// </remarks>
internal class SectionCurve
{
    protected IntersectionCurve _curve = new();
    protected NCollection_List<PaveBlock> _paveBlocks = new();
    protected NCollection_List<int> _technoVertices = new();
    protected Box _box = new();
    protected double _tolerance;

    public SectionCurve()
    {
        _tolerance = 0.0;
    }

    /// <summary>A copy of <paramref name="other"/> (OCCT's implicit copy constructor).</summary>
    public SectionCurve(SectionCurve other)
    {
        _curve = other._curve;
        _paveBlocks = new NCollection_List<PaveBlock>(other._paveBlocks);
        _technoVertices = new NCollection_List<int>(other._technoVertices);
        _box = other._box;
        _tolerance = other._tolerance;
    }

    internal void SetCurve(in IntersectionCurve curve) => _curve = curve;

    public IntersectionCurve Curve() => _curve;

    internal void SetBox(in Box box) => _box = box;

    public Box Box() => _box;

    internal ref Box ChangeBox() => ref _box;

    /// <summary>Replaces the pave blocks by those of the list.</summary>
    internal void SetPaveBlocks(NCollection_List<PaveBlock> lpb)
    {
        _paveBlocks.Clear();
        for (NCollection_List<PaveBlock>.Iterator it = new(lpb); it.More(); it.Next())
        {
            _paveBlocks.Append(it.Value());
        }
    }

    /// <summary>The pave blocks of the curve (the curve's own list; read only for callers).</summary>
    public NCollection_List<PaveBlock> PaveBlocks() => _paveBlocks;

    internal NCollection_List<PaveBlock> ChangePaveBlocks() => _paveBlocks;

    /// <summary>Creates the initial pave block of the curve, if it has none.</summary>
    internal void InitPaveBlock1()
    {
        if (_paveBlocks.Extent() == 0)
        {
            PaveBlock pb = new();
            _paveBlocks.Append(pb);
        }
    }

    /// <summary>The initial pave block, as a reference into the list (OCCT returns a reference to the handle).</summary>
    internal ref PaveBlock ChangePaveBlock1() => ref _paveBlocks.ChangeFirst();

    /// <summary>The indices of the technological vertices of the curve (read only for callers).</summary>
    public NCollection_List<int> TechnoVertices() => _technoVertices;

    internal NCollection_List<int> ChangeTechnoVertices() => _technoVertices;

    /// <summary>True if a pave block of the curve has an edge.</summary>
    public bool HasEdge()
    {
        bool bFlag;
        //
        bFlag = false;
        for (NCollection_List<PaveBlock>.Iterator itPB = new(_paveBlocks); itPB.More(); itPB.Next())
        {
            PaveBlock pb = itPB.Value();
            bFlag = pb.HasEdge();
            if (bFlag)
            {
                break;
            }
        }
        return bFlag;
    }

    internal void SetTolerance(double tol) => _tolerance = tol;

    public double Tolerance() => _tolerance;

    /// <summary>The tangential tolerance of the curve.</summary>
    public double TangentialTolerance() => _curve.TangentialTolerance();
}
