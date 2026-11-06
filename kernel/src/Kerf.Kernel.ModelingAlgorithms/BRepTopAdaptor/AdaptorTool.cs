// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepTopAdaptor/BRepTopAdaptor_Tool.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepTopAdaptor/BRepTopAdaptor_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;

using Kerf.Topology;

namespace Kerf.Engine.Topology.Adaptors;

/// <summary>A face's surface adaptor with its topological tool.</summary>
/// <remarks>A working object (decision 21). OCCT's empty Destroy is left out.</remarks>
internal class AdaptorTool
{
    private bool _myloaded;
    private BrepTopolTool _topolTool;
    private SurfaceAdaptorBase? _hSurface;

    /// <summary>An empty tool.</summary>
    public AdaptorTool()
    {
        _topolTool = new BrepTopolTool();
        _myloaded = false;
    }

    /// <summary>The tool of the face <paramref name="f"/> (restricted to its UV bounds); the tolerance is unused.</summary>
    public AdaptorTool(in Face f, double tol2d)
    {
        _topolTool = new BrepTopolTool();

        var surface = new FaceSurfaceAdaptor();
        surface.Initialize(f, true);
        _topolTool.Initialize((SurfaceAdaptorBase)surface);
        _hSurface = surface;
        _myloaded = true;
    }

    /// <summary>The tool of the surface <paramref name="surface"/> (a BRepAdaptor_Surface); the tolerance is unused.</summary>
    public AdaptorTool(SurfaceAdaptorBase surface, double tol2d)
    {
        _topolTool = new BrepTopolTool();
        _topolTool.Initialize(surface);
        _hSurface = surface;
        _myloaded = true;
    }

    /// <summary>Loads the face <paramref name="f"/>; the tolerance is unused.</summary>
    public void Init(in Face f, double tol2d)
    {
        var surface = new FaceSurfaceAdaptor();
        surface.Initialize(f);
        _topolTool.Initialize((SurfaceAdaptorBase)surface);
        _hSurface = surface;
        _myloaded = true;
    }

    /// <summary>Loads the surface <paramref name="surface"/> (a BRepAdaptor_Surface); the tolerance is unused.</summary>
    public void Init(SurfaceAdaptorBase surface, double tol2d)
    {
        _topolTool.Initialize(surface);
        _hSurface = surface;
        _myloaded = true;
    }

    /// <summary>The topological tool (also when nothing is loaded, as in OCCT).</summary>
    public BrepTopolTool GetTopolTool() => _topolTool;

    /// <summary>Replaces the topological tool.</summary>
    public void SetTopolTool(BrepTopolTool tt) => _topolTool = tt;

    /// <summary>The surface (null if none was loaded, as in OCCT).</summary>
    public SurfaceAdaptorBase? GetSurface() => _hSurface;

    /// <summary>True once a face or a surface is loaded (OCCT reads it only to print an error in debug builds).</summary>
    internal bool IsLoaded => _myloaded;
}
