// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_C2DF.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_C2DF.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;

using Kerf.Topology;

namespace Kerf.Engine.Booleans.Legacy.Tools;

/// <summary>A pcurve with its range and tolerance, and the face it lies on.</summary>
/// <remarks>A working object of the legacy Booleans' pcurve cache (<see cref="Legacy2D"/>).</remarks>
internal sealed class C2DF
{
    private Curve2D? _pc;
    private double _myf2d;
    private double _myl2d;
    private double _mytol;
    private Face _face = new();

    /// <summary>An empty pcurve.</summary>
    public C2DF()
    {
    }

    /// <summary>The pcurve <paramref name="pc"/> on <paramref name="f"/>.</summary>
    public C2DF(Curve2D? pc, double f2d, double l2d, double tol, in Face f)
    {
        _pc = pc;
        _myf2d = f2d;
        _myl2d = l2d;
        _mytol = tol;
        _face = f;
    }

    internal void SetPC(Curve2D? pc, double f2d, double l2d, double tol)
    {
        _pc = pc;
        _myf2d = f2d;
        _myl2d = l2d;
        _mytol = tol;
    }

    internal void SetFace(in Face f)
    {
        _face = f;
    }

    /// <summary>The pcurve, its range and its tolerance.</summary>
    public Curve2D? PC(out double f2d, out double l2d, out double tol)
    {
        f2d = _myf2d;
        l2d = _myl2d;
        tol = _mytol;
        return _pc;
    }

    /// <summary>The face.</summary>
    public Face Face() => _face;

    /// <summary>Whether the pcurve is <paramref name="pc"/> (the same object).</summary>
    public bool IsPC(Curve2D? pc)
    {
        bool b = ReferenceEquals(pc, _pc);
        return b;
    }

    /// <summary>Whether the face is equal to <paramref name="f"/> (IsEqual).</summary>
    public bool IsFace(in Face f)
    {
        bool b = f.IsEqual(_face);
        return b;
    }
}
