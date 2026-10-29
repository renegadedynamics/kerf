// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepTopAdaptor/BRepTopAdaptor_Tool.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepTopAdaptor/BRepTopAdaptor_Tool.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A face's surface adaptor with its topological tool.</summary>
/// <remarks>A working object (decision 21). OCCT's empty Destroy is left out.</remarks>
public class BRepTopAdaptor_Tool
{
    private bool myloaded;
    private BRepTopAdaptor_TopolTool myTopolTool;
    private Adaptor3d_Surface? myHSurface;

    /// <summary>An empty tool.</summary>
    public BRepTopAdaptor_Tool()
    {
        myTopolTool = new BRepTopAdaptor_TopolTool();
        myloaded = false;
    }

    /// <summary>The tool of the face <paramref name="F"/> (restricted to its UV bounds); the tolerance is unused.</summary>
    public BRepTopAdaptor_Tool(in TopoDS_Face F, double Tol2d)
    {
        myTopolTool = new BRepTopAdaptor_TopolTool();

        var surface = new BRepAdaptor_Surface();
        surface.Initialize(F, true);
        myTopolTool.Initialize((Adaptor3d_Surface)surface);
        myHSurface = surface;
        myloaded = true;
    }

    /// <summary>The tool of the surface <paramref name="surface"/> (a BRepAdaptor_Surface); the tolerance is unused.</summary>
    public BRepTopAdaptor_Tool(Adaptor3d_Surface surface, double Tol2d)
    {
        myTopolTool = new BRepTopAdaptor_TopolTool();
        myTopolTool.Initialize(surface);
        myHSurface = surface;
        myloaded = true;
    }

    /// <summary>Loads the face <paramref name="F"/>; the tolerance is unused.</summary>
    public void Init(in TopoDS_Face F, double Tol2d)
    {
        var surface = new BRepAdaptor_Surface();
        surface.Initialize(F);
        myTopolTool.Initialize((Adaptor3d_Surface)surface);
        myHSurface = surface;
        myloaded = true;
    }

    /// <summary>Loads the surface <paramref name="surface"/> (a BRepAdaptor_Surface); the tolerance is unused.</summary>
    public void Init(Adaptor3d_Surface surface, double Tol2d)
    {
        myTopolTool.Initialize(surface);
        myHSurface = surface;
        myloaded = true;
    }

    /// <summary>The topological tool (also when nothing is loaded, as in OCCT).</summary>
    public BRepTopAdaptor_TopolTool GetTopolTool() => myTopolTool;

    /// <summary>Replaces the topological tool.</summary>
    public void SetTopolTool(BRepTopAdaptor_TopolTool TT) => myTopolTool = TT;

    /// <summary>The surface (null if none was loaded, as in OCCT).</summary>
    public Adaptor3d_Surface? GetSurface() => myHSurface;

    /// <summary>True once a face or a surface is loaded (OCCT reads it only to print an error in debug builds).</summary>
    internal bool IsLoaded() => myloaded;
}
