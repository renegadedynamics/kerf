// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_Bisec.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_Bisec.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// A bisecting curve, the result of the GccAna bisector constructions: a line, circle, ellipse, parabola, hyperbola
/// or point, as ArcType() says, read through the accessor of that kind.
/// </summary>
/// <remarks>Immutable: each kind is set by its constructor only.</remarks>
public abstract class GccInt_Bisec : Standard_Transient
{
    /// <summary>The kind of the bisecting curve.</summary>
    public abstract GccInt_IType ArcType();

    /// <summary>The bisecting point when ArcType() is GccInt_Pnt; throws Standard_NotImplemented otherwise.</summary>
    public virtual gp_Pnt2d Point()
    {
        throw new Standard_NotImplemented();
    }

    /// <summary>The bisecting line when ArcType() is GccInt_Lin; throws Standard_NotImplemented otherwise.</summary>
    public virtual gp_Lin2d Line()
    {
        throw new Standard_NotImplemented();
    }

    /// <summary>The bisecting circle when ArcType() is GccInt_Cir; throws Standard_NotImplemented otherwise.</summary>
    public virtual gp_Circ2d Circle()
    {
        throw new Standard_NotImplemented();
    }

    /// <summary>The bisecting hyperbola when ArcType() is GccInt_Hpr; throws Standard_NotImplemented otherwise.</summary>
    public virtual gp_Hypr2d Hyperbola()
    {
        throw new Standard_NotImplemented();
    }

    /// <summary>The bisecting parabola when ArcType() is GccInt_Par; throws Standard_NotImplemented otherwise.</summary>
    public virtual gp_Parab2d Parabola()
    {
        throw new Standard_NotImplemented();
    }

    /// <summary>The bisecting ellipse when ArcType() is GccInt_Ell; throws Standard_NotImplemented otherwise.</summary>
    public virtual gp_Elips2d Ellipse()
    {
        throw new Standard_NotImplemented();
    }
}
