// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_Bisec.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/GccInt/GccInt_Bisec.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Constraints;

/// <summary>
/// A bisecting curve, the result of the GccAna bisector constructions: a line, circle, ellipse, parabola, hyperbola
/// or point, as ArcType() says, read through the accessor of that kind.
/// </summary>
/// <remarks>Immutable: each kind is set by its constructor only.</remarks>
internal abstract class ConstraintBisec : Transient
{
    /// <summary>The kind of the bisecting curve.</summary>
    public abstract ConstraintCurveType ArcType();

    /// <summary>The bisecting point when ArcType() is GccInt_Pnt; throws Standard_NotImplemented otherwise.</summary>
    public virtual Point2D Point()
    {
        throw new MethodNotImplementedException();
    }

    /// <summary>The bisecting line when ArcType() is GccInt_Lin; throws Standard_NotImplemented otherwise.</summary>
    public virtual Line2D Line()
    {
        throw new MethodNotImplementedException();
    }

    /// <summary>The bisecting circle when ArcType() is GccInt_Cir; throws Standard_NotImplemented otherwise.</summary>
    public virtual Circle2D Circle()
    {
        throw new MethodNotImplementedException();
    }

    /// <summary>The bisecting hyperbola when ArcType() is GccInt_Hpr; throws Standard_NotImplemented otherwise.</summary>
    public virtual Hyperbola2D Hyperbola()
    {
        throw new MethodNotImplementedException();
    }

    /// <summary>The bisecting parabola when ArcType() is GccInt_Par; throws Standard_NotImplemented otherwise.</summary>
    public virtual Parabola2D Parabola()
    {
        throw new MethodNotImplementedException();
    }

    /// <summary>The bisecting ellipse when ArcType() is GccInt_Ell; throws Standard_NotImplemented otherwise.</summary>
    public virtual Ellipse2D Ellipse()
    {
        throw new MethodNotImplementedException();
    }
}
