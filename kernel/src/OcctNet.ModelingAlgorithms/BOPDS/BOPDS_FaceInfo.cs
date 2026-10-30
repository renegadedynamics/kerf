// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_FaceInfo.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_FaceInfo.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The state of a face in the data structure: the pave blocks and vertices In (inside), On (on the boundary) and
/// Sc (on the section curves) of the face.
/// </summary>
/// <remarks>
/// A class, a working object held in the data structure's pool and changed in place (decision 631). The pave-block
/// maps compare by reference and iterate in insertion order (decision 580). The vertex maps (<c>NCollection_Map
/// &lt;int&gt;</c>) iterate in insertion order too, where OCCT's iterate in the order of their hash buckets (decision
/// 6153). As in OCCT, <see cref="Clear"/> leaves the Sc maps.
/// </remarks>
public class BOPDS_FaceInfo
{
    protected int myIndex;
    protected NCollection_IndexedMap<BOPDS_PaveBlock> myPaveBlocksIn = new();
    protected NCollection_Map<int> myVerticesIn = new();
    protected NCollection_IndexedMap<BOPDS_PaveBlock> myPaveBlocksOn = new();
    protected NCollection_Map<int> myVerticesOn = new();
    protected NCollection_IndexedMap<BOPDS_PaveBlock> myPaveBlocksSc = new();
    protected NCollection_Map<int> myVerticesSc = new();

    public BOPDS_FaceInfo()
    {
        myIndex = -1;
    }

    /// <summary>Clears the In and On maps (OCCT leaves the Sc maps).</summary>
    internal void Clear()
    {
        myPaveBlocksIn.Clear();
        myVerticesIn.Clear();
        myPaveBlocksOn.Clear();
        myVerticesOn.Clear();
    }

    internal void SetIndex(int theI) => myIndex = theI;

    /// <summary>The index of the face.</summary>
    public int Index() => myIndex;

    /// <summary>The pave blocks In the face (the face information's own map; read only for callers).</summary>
    public NCollection_IndexedMap<BOPDS_PaveBlock> PaveBlocksIn() => myPaveBlocksIn;

    internal NCollection_IndexedMap<BOPDS_PaveBlock> ChangePaveBlocksIn() => myPaveBlocksIn;

    public NCollection_Map<int> VerticesIn() => myVerticesIn;

    internal NCollection_Map<int> ChangeVerticesIn() => myVerticesIn;

    /// <summary>The pave blocks On the face (read only for callers).</summary>
    public NCollection_IndexedMap<BOPDS_PaveBlock> PaveBlocksOn() => myPaveBlocksOn;

    internal NCollection_IndexedMap<BOPDS_PaveBlock> ChangePaveBlocksOn() => myPaveBlocksOn;

    public NCollection_Map<int> VerticesOn() => myVerticesOn;

    internal NCollection_Map<int> ChangeVerticesOn() => myVerticesOn;

    /// <summary>The pave blocks of the section curves on the face (read only for callers).</summary>
    public NCollection_IndexedMap<BOPDS_PaveBlock> PaveBlocksSc() => myPaveBlocksSc;

    internal NCollection_IndexedMap<BOPDS_PaveBlock> ChangePaveBlocksSc() => myPaveBlocksSc;

    public NCollection_Map<int> VerticesSc() => myVerticesSc;

    internal NCollection_Map<int> ChangeVerticesSc() => myVerticesSc;
}
