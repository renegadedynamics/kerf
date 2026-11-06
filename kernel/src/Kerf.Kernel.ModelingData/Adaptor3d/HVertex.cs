// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_HVertex.hxx
// Port of: src/ModelingData/TKG3d/Adaptor3d/Adaptor3d_HVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Engine.Geometry.Adaptors;

/// <summary>
/// A vertex of the parametric domain of a surface, as Adaptor3d_TopolTool sees it: a 2D point, an orientation
/// and a tolerance. BRepTopAdaptor_HVertex redefines it for topological vertices.
/// </summary>
internal class HVertex : Transient
{
    private Point2D _pnt;
    private double _tol;
    private Orientation _ori;

    /// <summary>A vertex at the origin with a zero tolerance.</summary>
    /// <remarks>OCCT leaves the orientation uninitialised; here it is FORWARD.</remarks>
    public HVertex()
    {
        // OCCT: myOri is left uninitialised (docs/occt-issues.md).
        _tol = 0.0;
    }

    public HVertex(in Point2D p, Orientation or, double resolution)
    {
        _pnt = p;
        _tol = resolution;
        _ori = or;
    }

    public virtual Point2D Value() => _pnt;

    /// <summary>The parameter of the vertex on C, which must be a line.</summary>
    public virtual double Parameter(Curve2DAdaptorBase c) => global::Kerf.Engine.Geometry.ElCLib.Parameter(c.Line(), _pnt);

    /// <summary>The parametric (2D) resolution.</summary>
    public virtual double Resolution(Curve2DAdaptorBase c) => _tol;

    public virtual Orientation Orientation() => _ori;

    /// <summary>True if the two vertices are within <c>Precision::Confusion()</c>.</summary>
    public virtual bool IsSame(HVertex other) => (_pnt.DistanceTo(other.Value()) <= global::Kerf.Engine.Foundation.Precision.Confusion());
}
