// Port of: src/ModelingAlgorithms/TKHLR/HLRAppli/HLRAppli_ReflectLines.hxx
// Port of: src/ModelingAlgorithms/TKHLR/HLRAppli/HLRAppli_ReflectLines.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2001-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Geometry;
using Kerf.Topology;
using static Occt.HLRBRep_TypeOfResultingEdge;

namespace Occt;

/// <summary>
/// The reflect lines of a shape for a view: its outlines (and its other edges, by type) seen from a direction, as 3D
/// edges on the shape or as 2D edges in the view, by the exact hidden-line removal.
/// </summary>
/// <remarks>
/// A working object, as OCCT's. The shape is not written into (HLRBRep_Algo copies a frozen shape at the entry); the
/// compounds are published frozen, after <c>BRepLib::SameParameter</c> as OCCT does.
/// </remarks>
public class HLRAppli_ReflectLines
{
    private HLRAlgo_Projector myProjector = new();
    private HLRBRep_Algo? myHLRAlgo;
    private readonly Shape myShape;

    /// <summary>Constructor.</summary>
    public HLRAppli_ReflectLines(in Shape aShape)
    {
        myShape = aShape;
    }

    /// <summary>
    /// Sets the normal to the plane of visualisation (Nx, Ny, Nz), the coordinates of the view point (XAt, YAt, ZAt)
    /// and of the vertical direction on the plane of visualisation (XUp, YUp, ZUp).
    /// </summary>
    public void SetAxes(double Nx, double Ny, double Nz, double XAt, double YAt, double ZAt, double XUp, double YUp,
                        double ZUp)
    {
        bool IsPerspective = false;
        double aFocus = 1;
        // Prs3d_Projector aPrs3dProjector(IsPerspective, aFocus, Nx, Ny, Nz, XAt, YAt, ZAt, XUp, YUp, ZUp);

        Point3D At = new(XAt, YAt, ZAt);
        Direction3D Zpers = new(Nx, Ny, Nz);
        Direction3D Ypers = new(XUp, YUp, ZUp);
        Direction3D Xpers = Ypers.Crossed(Zpers);
        CoordinateSystem Axe = new(At, Zpers, Xpers);
        Transform3D T = new();
        T.SetTransformation(Axe);

        // myProjector = aPrs3dProjector.Projector();
        myProjector = new HLRAlgo_Projector(T, IsPerspective, aFocus);
    }

    /// <summary>Runs the hidden-line removal.</summary>
    public void Perform()
    {
        myHLRAlgo = new HLRBRep_Algo();
        myHLRAlgo.Add(myShape, 0);
        myHLRAlgo.Projector(myProjector);
        myHLRAlgo.Update();
        myHLRAlgo.Hide();

        // OCCT's commented-out OutLineVCompound3d and SameParameter left out
    }

    /// <summary>The visible outlines, in 3D.</summary>
    public Shape GetResult() => GetCompoundOf3dEdges(HLRBRep_OutLine, true, true);

    /// <summary>The visible or hidden edges of a type, in 3D (on the shape) or in 2D (in the view).</summary>
    public Shape GetCompoundOf3dEdges(HLRBRep_TypeOfResultingEdge type, bool visible, bool In3d)
    {
        HLRBRep_HLRToShape aHLRToShape = new(myHLRAlgo!);

        Shape theCompound = aHLRToShape.InternalCompoundOfEdges(new Shape(), type, visible, In3d);

        BRepLib.SameParameter(theCompound, global::Kerf.Engine.Foundation.Precision.PConfusion(), false);

        theCompound.TShape()?.FreezeGraph();
        return theCompound;
    }
}
