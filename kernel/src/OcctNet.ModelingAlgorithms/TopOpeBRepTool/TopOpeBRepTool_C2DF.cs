// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_C2DF.hxx
// Port of: src/ModelingAlgorithms/TKBool/TopOpeBRepTool/TopOpeBRepTool_C2DF.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A pcurve with its range and tolerance, and the face it lies on.</summary>
/// <remarks>A working object of the legacy Booleans' pcurve cache (<see cref="TopOpeBRepTool_2d"/>).</remarks>
public sealed class TopOpeBRepTool_C2DF
{
    private Geom2d_Curve? myPC;
    private double myf2d;
    private double myl2d;
    private double mytol;
    private TopoDS_Face myFace = new();

    /// <summary>An empty pcurve.</summary>
    public TopOpeBRepTool_C2DF()
    {
    }

    /// <summary>The pcurve <paramref name="PC"/> on <paramref name="F"/>.</summary>
    public TopOpeBRepTool_C2DF(Geom2d_Curve? PC, double f2d, double l2d, double tol, in TopoDS_Face F)
    {
        myPC = PC;
        myf2d = f2d;
        myl2d = l2d;
        mytol = tol;
        myFace = F;
    }

    internal void SetPC(Geom2d_Curve? PC, double f2d, double l2d, double tol)
    {
        myPC = PC;
        myf2d = f2d;
        myl2d = l2d;
        mytol = tol;
    }

    internal void SetFace(in TopoDS_Face F)
    {
        myFace = F;
    }

    /// <summary>The pcurve, its range and its tolerance.</summary>
    public Geom2d_Curve? PC(out double f2d, out double l2d, out double tol)
    {
        f2d = myf2d;
        l2d = myl2d;
        tol = mytol;
        return myPC;
    }

    /// <summary>The face.</summary>
    public TopoDS_Face Face() => myFace;

    /// <summary>Whether the pcurve is <paramref name="PC"/> (the same object).</summary>
    public bool IsPC(Geom2d_Curve? PC)
    {
        bool b = ReferenceEquals(PC, myPC);
        return b;
    }

    /// <summary>Whether the face is equal to <paramref name="F"/> (IsEqual).</summary>
    public bool IsFace(in TopoDS_Face F)
    {
        bool b = F.IsEqual(myFace);
        return b;
    }
}
