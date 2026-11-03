// Port of: src/ModelingAlgorithms/TKHLR/HLRAppli/HLRAppli_ReflectLines.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAppli/HLRAppli_ReflectLines.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2001-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Library;
using Kerf.Engine.HiddenLines.Algorithm;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.HiddenLines.TypeOfResultingEdge;

namespace Kerf.Engine.HiddenLines;

/// <summary>
/// The reflect lines of a shape for a view: its outlines (and its other edges, by type) seen from a direction, as 3D
/// edges on the shape or as 2D edges in the view, by the exact hidden-line removal.
/// </summary>
/// <remarks>
/// A working object, as OCCT's. The shape is not written into (HLRBRep_Algo copies a frozen shape at the entry); the
/// compounds are published frozen, after <c>BRepLib::SameParameter</c> as OCCT does.
/// </remarks>
public class ReflectLines
{
    private AlgorithmProjector _projector = new();
    private HiddenLineAlgo? _hlrAlgo;
    private readonly Shape _shape;

    /// <summary>Constructor.</summary>
    public ReflectLines(in Shape aShape)
    {
        _shape = aShape;
    }

    /// <summary>
    /// Sets the normal to the plane of visualisation (Nx, Ny, Nz), the coordinates of the view point (XAt, YAt, ZAt)
    /// and of the vertical direction on the plane of visualisation (XUp, YUp, ZUp).
    /// </summary>
    public void SetAxes(double nx, double ny, double nz, double xAt, double yAt, double zAt, double xUp, double yUp,
                        double zUp)
    {
        bool isPerspective = false;
        double focus = 1;
        // Prs3d_Projector aPrs3dProjector(IsPerspective, aFocus, Nx, Ny, Nz, XAt, YAt, ZAt, XUp, YUp, ZUp);

        Point3D at = new(xAt, yAt, zAt);
        Direction3D zpers = new(nx, ny, nz);
        Direction3D ypers = new(xUp, yUp, zUp);
        Direction3D xpers = ypers.Crossed(zpers);
        CoordinateSystem axe = new(at, zpers, xpers);
        Transform3D t = new();
        t.SetTransformation(axe);

        // myProjector = aPrs3dProjector.Projector();
        _projector = new AlgorithmProjector(t, isPerspective, focus);
    }

    /// <summary>Runs the hidden-line removal.</summary>
    public void Perform()
    {
        _hlrAlgo = new HiddenLineAlgo();
        _hlrAlgo.Add(_shape, 0);
        _hlrAlgo.Projector(_projector);
        _hlrAlgo.Update();
        _hlrAlgo.Hide();

        // OCCT's commented-out OutLineVCompound3d and SameParameter left out
    }

    /// <summary>The visible outlines, in 3D.</summary>
    public Shape GetResult() => GetCompoundOf3dEdges(TypeOfResultingEdge.OutLine, true, true);

    /// <summary>The visible or hidden edges of a type, in 3D (on the shape) or in 2D (in the view).</summary>
    public Shape GetCompoundOf3dEdges(TypeOfResultingEdge type, bool visible, bool in3d)
    {
        HlrToShape hlrToShape = new(_hlrAlgo!);

        Shape theCompound = hlrToShape.InternalCompoundOfEdges(new Shape(), type, visible, in3d);

        BrepLibrary.SameParameter(theCompound, global::Kerf.Engine.Foundation.Precision.PConfusion(), false);

        theCompound.TShape()?.FreezeGraph();
        return theCompound;
    }
}
