// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Shell.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeAnalysis/ShapeAnalysis_Shell.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Analysis;

/// <summary>
/// Checks the orientation of the faces of shells: edges met twice with the same orientation (bad edges), edges met
/// once (free edges), and whether the shells are connected.
/// </summary>
/// <remarks>
/// A public working object (decision 21; OCCT uses it by value). It only reads the shapes it is given, so it works on
/// frozen shapes. <see cref="BadEdges"/> and <see cref="FreeEdges"/> build compounds of the edges found and freeze them
/// (docs/phase3-design.md, section 4.2), which freezes the edges too; ported code that goes on writing into those
/// edges (ShapeFix_Shell, on its own copy) uses <see cref="InternalBadEdges"/> and <see cref="InternalFreeEdges"/>.
/// The maps are indexed, so the edges come in the order OCCT finds them.
/// </remarks>
public sealed class ShellAnalysis
{
    private readonly IndexedSet<Shape> _shells = new(ShapeHasher.Instance);
    private readonly IndexedSet<Shape> _bad = new(ShapeHasher.Instance);
    private readonly IndexedSet<Shape> _free = new(ShapeHasher.Instance);
    private bool _conex;

    /// <summary>An empty analyser.</summary>
    public ShellAnalysis()
    {
        _conex = false;
    }

    /// <summary>Clears the loaded shells and the results.</summary>
    public void Clear()
    {
        _shells.Clear();
        _bad.Clear();
        _free.Clear();
        _conex = false;
    }

    /// <summary>Loads the shape if it is a shell, else its shells.</summary>
    public void LoadShells(in Shape shape)
    {
        if (shape.IsNull)
        {
            return;
        }

        if (shape.ShapeType() == ShapeType.Shell)
        {
            _shells.Add(shape); // szv#4:S4163:12Mar99 i =
        }
        else
        {
            for (var exs = new Explorer(shape, ShapeType.Shell); exs.More(); exs.Next())
            {
                Shape sh = exs.Current();
                _shells.Add(sh); // szv#4:S4163:12Mar99 i =
            }
        }
    }

    //  CheckOrientedShells : alimente BadEdges et FreeEdges
    //  BadEdges : edges presentes plus d une fois dans une meme orientation
    //  FreeEdges : edges presentes une seule fois
    //  On utilise pour cela une fonction auxiliaire : CheckEdges
    //    Qui alimente 2 maps auxiliaires : les edges directes et les inverses

    private static bool CheckEdges(in Shape shape, IndexedSet<Shape> bads,
                                   IndexedSet<Shape> dirs, IndexedSet<Shape> revs,
                                   IndexedSet<Shape> ints)
    {
        bool res = false;

        if (shape.ShapeType() != ShapeType.Edge)
        {
            for (var it = new Iterator(shape); it.More(); it.Next())
            {
                if (CheckEdges(it.Value(), bads, dirs, revs, ints))
                {
                    res = true;
                }
            }
        }
        else
        {
            Edge e = ShapeCasts.Edge(shape);
            if (Tool.Degenerated(e))
            {
                return false;
            }

            if (shape.Orientation() == Forward)
            {
                // szv#4:S4163:12Mar99 optimized
                if (!dirs.Contains(shape))
                {
                    dirs.Add(shape);
                }
                else
                {
                    bads.Add(shape);
                    res = true;
                }
            }
            if (shape.Orientation() == Reversed)
            {
                // szv#4:S4163:12Mar99 optimized
                if (!revs.Contains(shape))
                {
                    revs.Add(shape);
                }
                else
                {
                    bads.Add(shape);
                    res = true;
                }
            }
            if (shape.Orientation() == Internal)
            {
                if (!ints.Contains(shape))
                {
                    ints.Add(shape);
                }
                // else { bads.Add (shape); res = true; }
            }
        }

        return res;
    }

    /// <summary>
    /// Checks the shells of the shape: an edge met twice with the same orientation is bad, and its shell is loaded.
    /// True if a new badly oriented shell was found. With <paramref name="alsofree"/>, the free edges (met in one
    /// orientation only) are found too, not counting the INTERNAL ones as free with
    /// <paramref name="checkinternaledges"/>.
    /// </summary>
    public bool CheckOrientedShells(in Shape shape, bool alsofree = false, bool checkinternaledges = false)
    {
        _conex = false;
        if (shape.IsNull)
        {
            return false;
        }
        bool res = false;

        var dirs = new IndexedSet<Shape>(ShapeHasher.Instance);
        var revs = new IndexedSet<Shape>(ShapeHasher.Instance);
        var ints = new IndexedSet<Shape>(ShapeHasher.Instance);
        for (var exs = new Explorer(shape, ShapeType.Shell); exs.More(); exs.Next())
        {
            Shape sh = exs.Current();
            // szv#4:S4163:12Mar99 optimized
            if (CheckEdges(sh, _bad, dirs, revs, ints))
            {
                if (_shells.Add(sh) + 1 != 0) // an indexed map's Add returns the index (never 0)
                {
                    res = true;
                }
            }
        }

        //  Resteraient a faire les FreeEdges
        if (!alsofree)
        {
            return res;
        }

        //  Free Edges. Ce sont les edges d une map pas dans l autre
        //  et lycee de Versailles  (les maps dirs et revs)
        int nb = dirs.Count;
        int i; // svv Jan11 2000 : porting on DEC
        for (i = 1; i <= nb; i++)
        {
            Shape sh = dirs[i - 1];
            if (!_bad.Contains(sh))
            {
                if (!revs.Contains(sh))
                {
                    if (checkinternaledges)
                    {
                        if (!ints.Contains(sh))
                        {
                            _free.Add(sh);
                        }
                        else
                        {
                            _conex = true;
                        }
                    }
                    else
                    {
                        _free.Add(sh);
                    }
                }
                else
                {
                    _conex = true;
                }
            }
            else
            {
                _conex = true;
            }
        }

        nb = revs.Count;
        for (i = 1; i <= nb; i++)
        {
            Shape sh = revs[i - 1];
            if (!_bad.Contains(sh))
            {
                if (!dirs.Contains(sh))
                {
                    if (checkinternaledges)
                    {
                        if (!ints.Contains(sh))
                        {
                            _free.Add(sh);
                        }
                        else
                        {
                            _conex = true;
                        }
                    }
                    else
                    {
                        _free.Add(sh);
                    }
                }
                else
                {
                    _conex = true;
                }
            }
            else
            {
                _conex = true;
            }
        }

        return res;
    }

    /// <summary>True if the shape is a loaded shell.</summary>
    public bool IsLoaded(in Shape shape)
    {
        if (shape.IsNull)
        {
            return false;
        }
        return _shells.Contains(shape);
    }

    /// <summary>The number of loaded shells.</summary>
    public int LoadedCount => _shells.Count;

    /// <summary>The loaded shell of the given rank (from 1), as given.</summary>
    public Shape Loaded(int num) => _shells[num - 1];

    /// <summary>True if bad edges were found by <see cref="CheckOrientedShells"/>.</summary>
    public bool HasBadEdges => (_bad.Count > 0);

    /// <summary>A compound of the bad edges. Frozen.</summary>
    public Compound BadEdges() => Published(InternalBadEdges());

    /// <summary>A compound of the bad edges, as <see cref="BadEdges"/>, unfrozen (ported code).</summary>
    internal Compound InternalBadEdges()
    {
        Compound c = new();
        BrepBuilder b = new();
        b.MakeCompound(ref c);
        int n = _bad.Count;
        for (int i = 0; i < n; i++)
        {
            b.Add(c, _bad[i]);
        }
        return c;
    }

    /// <summary>True if free edges were found by <see cref="CheckOrientedShells"/>.</summary>
    public bool HasFreeEdges => (_free.Count > 0);

    /// <summary>A compound of the free edges. Frozen.</summary>
    public Compound FreeEdges() => Published(InternalFreeEdges());

    /// <summary>A compound of the free edges, as <see cref="FreeEdges"/>, unfrozen (ported code).</summary>
    internal Compound InternalFreeEdges()
    {
        Compound c = new();
        BrepBuilder b = new();
        b.MakeCompound(ref c);
        int n = _free.Count;
        for (int i = 0; i < n; i++)
        {
            b.Add(c, _free[i]);
        }
        return c;
    }

    /// <summary>True if edges shared by two faces (in opposite orientations) were found.</summary>
    public bool HasConnectedEdges => _conex;

    private static Compound Published(in Compound shape)
    {
        shape.TShape()?.FreezeGraph();
        return shape;
    }
}
