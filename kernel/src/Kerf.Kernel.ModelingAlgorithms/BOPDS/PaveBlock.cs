// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_PaveBlock.hxx
// Port of: src/ModelingAlgorithms/TKBO/BOPDS/BOPDS_PaveBlock.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Foundation.Std;

using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine;

namespace Kerf.Engine.Booleans.Data;

/// <summary>
/// A pave block: the part of an edge between two adjacent paves, with the extra paves found on it, the index of
/// the edge built for it (or the original edge) and its shrunk data.
/// </summary>
/// <remarks>
/// A working object (plan section 6): maps and lists of pave blocks compare them by reference and iterate in
/// insertion order (decision 580). The sort of the paves in <see cref="Update"/> keeps MSVC's order of paves with
/// equal parameters (<see cref="Sorting"/>).
/// </remarks>
internal class PaveBlock : Transient
{
    protected int _edge;
    protected int _originalEdge;
    protected Pave _pave1 = new();
    protected Pave _pave2 = new();
    protected NCollection_List<Pave> _extPaves = new();
    protected double _ts1;
    protected double _ts2;
    protected Box _shrunkBox = new();
    protected NCollection_IntMap _mFence = new();
    protected bool _isSplittable;

    /// <summary>An empty pave block.</summary>
    public PaveBlock()
    {
        _edge = -1;
        _originalEdge = -1;
        _ts1 = -99.0;
        _ts2 = _ts1;
        _isSplittable = false;
    }

    internal void SetEdge(int edge) => _edge = edge;

    /// <summary>The index of the edge of the pave block.</summary>
    public int Edge() => _edge;

    public bool HasEdge() => _edge >= 0;

    public bool HasEdge(out int edge)
    {
        edge = _edge;
        return _edge >= 0;
    }

    internal void SetOriginalEdge(int edge) => _originalEdge = edge;

    /// <summary>The index of the original edge.</summary>
    public int OriginalEdge() => _originalEdge;

    /// <summary>True if the edge differs from the original edge.</summary>
    public bool IsSplitEdge => _edge != _originalEdge;

    internal void SetPave1(in Pave pave) => _pave1 = pave;

    public Pave Pave1() => _pave1;

    internal void SetPave2(in Pave pave) => _pave2 = pave;

    public Pave Pave2() => _pave2;

    /// <summary>The parameters of the two paves.</summary>
    public void Range(out double t1, out double t2)
    {
        t1 = _pave1.Parameter();
        t2 = _pave2.Parameter();
    }

    /// <summary>The vertex indices of the two paves.</summary>
    public void Indices(out int index1, out int index2)
    {
        index1 = _pave1.Index();
        index2 = _pave2.Index();
    }

    /// <summary>True if the other pave block's vertex indices are this one's, in either order.</summary>
    public bool HasSameBounds(PaveBlock other)
    {
        bool bFlag1, bFlag2;
        int n11, n12, n21, n22;
        //
        Indices(out n11, out n12);
        other.Indices(out n21, out n22);
        //
        bFlag1 = (n11 == n21) && (n12 == n22);
        bFlag2 = (n11 == n22) && (n12 == n21);
        //
        return bFlag1 || bFlag2;
    }

    /// <summary>Appends an extra pave, unless one with the same vertex was appended already.</summary>
    internal void AppendExtPave(in Pave pave)
    {
        if (_mFence.Add(pave.Index()))
        {
            _extPaves.Append(pave);
        }
    }

    /// <summary>Appends an extra pave unconditionally.</summary>
    internal void AppendExtPave1(in Pave pave) => _extPaves.Append(pave);

    /// <summary>Removes the extra paves of the vertex.</summary>
    internal void RemoveExtPave(int vertNum)
    {
        if (_mFence.Contains(vertNum))
        {
            NCollection_List<Pave>.Iterator itPaves = new(_extPaves);
            while (itPaves.More())
            {
                if (itPaves.Value().Index() == vertNum)
                {
                    _extPaves.Remove(itPaves);
                }
                else
                {
                    itPaves.Next();
                }
            }
            _mFence.Remove(vertNum);
        }
    }

    /// <summary>The extra paves (the pave block's own list; read only for callers).</summary>
    public NCollection_List<Pave> ExtPaves() => _extPaves;

    internal NCollection_List<Pave> ChangeExtPaves() => _extPaves;

    /// <summary>True if the pave block has extra paves.</summary>
    public bool IsToUpdate => !_extPaves.IsEmpty();

    /// <summary>True if an extra pave lies within <paramref name="tol"/> of the parameter; its index then (otherwise <paramref name="ind"/> is left as it was).</summary>
    public bool ContainsParameter(double t, double tol, ref int ind)
    {
        bool bRet;
        //
        bRet = false;
        for (NCollection_List<Pave>.Iterator it = new(_extPaves); it.More(); it.Next())
        {
            Pave pave = it.Value();
            bRet = Math.Abs(pave.Parameter() - t) < tol;
            if (bRet)
            {
                ind = pave.Index();
                break;
            }
        }
        return bRet;
    }

    /// <summary>
    /// Splits the pave block at its extra paves (and its two paves if <paramref name="flag"/>) into new pave
    /// blocks appended to <paramref name="lpb"/>, sorted by parameter; the extra paves are cleared.
    /// </summary>
    internal void Update(NCollection_List<PaveBlock> lpb, bool flag = true)
    {
        int i, nb;
        Pave pave1 = new(), pave2;
        PaveBlock pb;
        //
        nb = _extPaves.Extent();
        if (flag)
        {
            nb = nb + 2;
        }
        //
        if (nb <= 1)
        {
            _extPaves.Clear();
            _mFence.Clear();
            return;
        }
        //
        Pave[] pPaves = new Pave[nb];
        //
        i = 0;
        if (flag)
        {
            pPaves[i] = _pave1;
            ++i;
            pPaves[i] = _pave2;
            ++i;
        }
        //
        for (NCollection_List<Pave>.Iterator it = new(_extPaves); it.More(); it.Next())
        {
            pPaves[i] = it.Value();
            ++i;
        }
        _extPaves.Clear();
        _mFence.Clear();
        //
        Sorting.Sort(pPaves.AsSpan(), new Pave.LessThan());
        //
        for (i = 0; i < nb; ++i)
        {
            Pave pave = pPaves[i];
            if (i == 0)
            {
                pave1 = pave;
                continue;
            }
            //
            pave2 = pave;
            pb = new PaveBlock();
            pb.SetOriginalEdge(_originalEdge);
            pb.SetPave1(pave1);
            pb.SetPave2(pave2);
            //
            lpb.Append(pb);
            //
            pave1 = pave2;
        }
    }

    /// <summary>True if the pave block has shrunk data (a non-void shrunk box).</summary>
    public bool HasShrunkData => !_shrunkBox.IsVoid;

    /// <summary>Sets the shrunk range, its box and whether the edge can be split.</summary>
    internal void SetShrunkData(double t1, double t2, in Box box, bool isSplittable)
    {
        _ts1 = t1;
        _ts2 = t2;
        _shrunkBox = box;
        _isSplittable = isSplittable;
    }

    /// <summary>The shrunk range, its box and whether the edge can be split.</summary>
    public void ShrunkData(out double t1, out double t2, out Box box, out bool isSplittable)
    {
        t1 = _ts1;
        t2 = _ts2;
        box = _shrunkBox;
        isSplittable = _isSplittable;
    }

    /// <summary>False if the shrunk range is too short for the edge to be split.</summary>
    public bool IsSplittable => _isSplittable;

    /// <summary>Writes the pave block as OCCT's <c>Dump</c> prints it.</summary>
    public void Dump(TextWriter os)
    {
        os.Write(" PB:{ E:" + _edge + " orE:" + _originalEdge);
        os.Write(" Pave1:");
        _pave1.Dump(os);
        os.Write(" Pave2:");
        _pave2.Dump(os);
        os.Write(" }");
    }
}
