// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_SolidClassifier.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_SolidClassifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.TopAbs_State;

namespace Occt;

/// <summary>Classifies points in a solid: IN, OUT or ON its boundary.</summary>
/// <remarks>
/// A working object (decision 21): load a shape once, then classify any number of points. It only reads the shape.
/// OCCT's statistics (LBRCOMPT) and its MARCHEPASSIUNESEULEFACE variants are compiled out, as in OCCT.
/// </remarks>
public class BRepClass3d_SolidClassifier : BRepClass3d_SClassifier
{
    private bool aSolidLoaded;
    private readonly BRepClass3d_SolidExplorer explorer;
    private bool isaholeinspace;

    /// <summary>An empty classifier.</summary>
    public BRepClass3d_SolidClassifier()
    {
        explorer = new BRepClass3d_SolidExplorer();
        aSolidLoaded = isaholeinspace = false;
    }

    /// <summary>Loads the shape <paramref name="S"/>.</summary>
    public void Load(in TopoDS_Shape S)
    {
        if (aSolidLoaded)
        {
            explorer.Destroy();
        }

        explorer.InitShape(S);
        aSolidLoaded = true;
    }

    /// <summary>A classifier of the shape <paramref name="S"/>.</summary>
    public BRepClass3d_SolidClassifier(in TopoDS_Shape S)
    {
        aSolidLoaded = true;
        explorer = new BRepClass3d_SolidExplorer(S);
        isaholeinspace = false;
    }

    /// <summary>Classifies the point <paramref name="P"/> with the tolerance <paramref name="Tol"/> in the shape <paramref name="S"/>.</summary>
    public BRepClass3d_SolidClassifier(in TopoDS_Shape S, in gp_Pnt P, double Tol)
    {
        explorer = new BRepClass3d_SolidExplorer(S);
        aSolidLoaded = true;
        Perform(P, Tol);
    }

    /// <summary>Classifies the point <paramref name="P"/> with the tolerance <paramref name="Tol"/>.</summary>
    public void Perform(in gp_Pnt P, double Tol)
    {
        Perform(explorer, P, Tol);
    }

    /// <summary>
    /// Classifies a point at infinity with the tolerance <paramref name="Tol"/>: OUT for a finite solid, IN for a hole
    /// in space (useful to find a solid's orientation).
    /// </summary>
    public void PerformInfinitePoint(double Tol)
    {
        if (aSolidLoaded)
        {
            PerformInfinitePoint(explorer, Tol);
            isaholeinspace = State() != TopAbs_OUT;
        }
    }

    /// <summary>Unloads the shape (OCCT's Destroy, also called by its destructor).</summary>
    public void Destroy()
    {
        if (aSolidLoaded)
        {
            explorer.Destroy();
            aSolidLoaded = false;
        }
    }

    /// <summary>True if the last <see cref="PerformInfinitePoint(double)"/> found a hole in space (OCCT keeps it, unread).</summary>
    internal bool IsAHoleInSpace() => isaholeinspace;
}
