// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_FaceInfo.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_FaceInfo.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Occt;

namespace Kerf.Engine.Booleans.Data;

/// <summary>
/// The state of a face in the data structure: the pave blocks and vertices In (inside), On (on the boundary) and
/// Sc (on the section curves) of the face.
/// </summary>
/// <remarks>
/// A class, a working object held in the data structure's pool and changed in place (decision 631). The pave-block
/// maps compare by reference and iterate in insertion order (decision 580). The vertex maps are
/// <see cref="NCollection_IntMap"/>s, which iterate in the order of OCCT's hash buckets as native's do (decision 784).
/// As in OCCT, <see cref="Clear"/> leaves the Sc maps.
/// </remarks>
public class FaceInfo
{
    protected int _index;
    protected NCollection_IndexedMap<PaveBlock> _paveBlocksIn = new();
    protected NCollection_IntMap _verticesIn = new();
    protected NCollection_IndexedMap<PaveBlock> _paveBlocksOn = new();
    protected NCollection_IntMap _verticesOn = new();
    protected NCollection_IndexedMap<PaveBlock> _paveBlocksSc = new();
    protected NCollection_IntMap _verticesSc = new();

    public FaceInfo()
    {
        _index = -1;
    }

    /// <summary>Clears the In and On maps (OCCT leaves the Sc maps).</summary>
    internal void Clear()
    {
        _paveBlocksIn.Clear();
        _verticesIn.Clear();
        _paveBlocksOn.Clear();
        _verticesOn.Clear();
    }

    internal void SetIndex(int i) => _index = i;

    /// <summary>The index of the face.</summary>
    public int Index() => _index;

    /// <summary>The pave blocks In the face (the face information's own map; read only for callers).</summary>
    public NCollection_IndexedMap<PaveBlock> PaveBlocksIn() => _paveBlocksIn;

    internal NCollection_IndexedMap<PaveBlock> ChangePaveBlocksIn() => _paveBlocksIn;

    public NCollection_IntMap VerticesIn() => _verticesIn;

    internal NCollection_IntMap ChangeVerticesIn() => _verticesIn;

    /// <summary>The pave blocks On the face (read only for callers).</summary>
    public NCollection_IndexedMap<PaveBlock> PaveBlocksOn() => _paveBlocksOn;

    internal NCollection_IndexedMap<PaveBlock> ChangePaveBlocksOn() => _paveBlocksOn;

    public NCollection_IntMap VerticesOn() => _verticesOn;

    internal NCollection_IntMap ChangeVerticesOn() => _verticesOn;

    /// <summary>The pave blocks of the section curves on the face (read only for callers).</summary>
    public NCollection_IndexedMap<PaveBlock> PaveBlocksSc() => _paveBlocksSc;

    internal NCollection_IndexedMap<PaveBlock> ChangePaveBlocksSc() => _paveBlocksSc;

    public NCollection_IntMap VerticesSc() => _verticesSc;

    internal NCollection_IntMap ChangeVerticesSc() => _verticesSc;
}
