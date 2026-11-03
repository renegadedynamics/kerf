// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_SolidClassifier.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepClass3d/BRepClass3d_SolidClassifier.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Classification;

/// <summary>Classifies points in a solid: IN, OUT or ON its boundary.</summary>
/// <remarks>
/// A working object (decision 21): load a shape once, then classify any number of points. It only reads the shape.
/// OCCT's statistics (LBRCOMPT) and its MARCHEPASSIUNESEULEFACE variants are compiled out, as in OCCT.
/// </remarks>
public class SolidClassifier : SClassifier
{
    private bool _aSolidLoaded;
    private readonly SolidExplorer _explorer;
    private bool _isaholeinspace;

    /// <summary>An empty classifier.</summary>
    public SolidClassifier()
    {
        _explorer = new SolidExplorer();
        _aSolidLoaded = _isaholeinspace = false;
    }

    /// <summary>Loads the shape <paramref name="s"/>.</summary>
    public void Load(in Shape s)
    {
        if (_aSolidLoaded)
        {
            _explorer.Destroy();
        }

        _explorer.InitShape(s);
        _aSolidLoaded = true;
    }

    /// <summary>A classifier of the shape <paramref name="s"/>.</summary>
    public SolidClassifier(in Shape s)
    {
        _aSolidLoaded = true;
        _explorer = new SolidExplorer(s);
        _isaholeinspace = false;
    }

    /// <summary>Classifies the point <paramref name="p"/> with the tolerance <paramref name="tol"/> in the shape <paramref name="s"/>.</summary>
    public SolidClassifier(in Shape s, in Point3D p, double tol)
    {
        _explorer = new SolidExplorer(s);
        _aSolidLoaded = true;
        Perform(p, tol);
    }

    /// <summary>Classifies the point <paramref name="p"/> with the tolerance <paramref name="tol"/>.</summary>
    public void Perform(in Point3D p, double tol)
    {
        Perform(_explorer, p, tol);
    }

    /// <summary>
    /// Classifies a point at infinity with the tolerance <paramref name="tol"/>: OUT for a finite solid, IN for a hole
    /// in space (useful to find a solid's orientation).
    /// </summary>
    public void PerformInfinitePoint(double tol)
    {
        if (_aSolidLoaded)
        {
            PerformInfinitePoint(_explorer, tol);
            _isaholeinspace = State() != Out;
        }
    }

    /// <summary>Unloads the shape (OCCT's Destroy, also called by its destructor).</summary>
    public void Destroy()
    {
        if (_aSolidLoaded)
        {
            _explorer.Destroy();
            _aSolidLoaded = false;
        }
    }

    /// <summary>True if the last <see cref="PerformInfinitePoint(double)"/> found a hole in space (OCCT keeps it, unread).</summary>
    internal bool IsAHoleInSpace => _isaholeinspace;
}
