// Port of: src/ModelingAlgorithms/TKOffset/BiTgte/BiTgte_CurveOnVertex.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BiTgte/BiTgte_CurveOnVertex.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Offsets.Blend;

/// <summary>
/// The degenerate curve of the rolling ball's centre on a vertex: the vertex's point over the range of the edge EonF.
/// Only the parameters, D0 (<see cref="EvalD0"/>) and the type are defined; the other members throw
/// <see cref="MethodNotImplementedException"/> (the base class's), and <see cref="Circle"/> Standard_NoSuchObject, as in OCCT.
/// </summary>
internal sealed class CurveOnVertex : CurveAdaptorBase
{
    private double _first;
    private double _last;
    private Point3D _pnt = new();

    public CurveOnVertex()
    {
        _first = 0.0;
        _last = 0.0;
    }

    public CurveOnVertex(in Edge eonF, in Vertex vertex)
    {
        _first = 0.0;
        _last = 0.0;
        Init(eonF, vertex);
    }

    public void Init(in Edge eonF, in Vertex v)
    {
        Tool.Range(eonF, out _first, out _last);
        _pnt = Tool.Pnt(v);
    }

    public override double FirstParameter() => _first;

    public override double LastParameter() => _last;

    public override Point3D EvalD0(double u) => _pnt;

    public override CurveKind GetType() => OtherCurve;

    /// <exception cref="NoSuchObjectException">Always.</exception>
    public override Circle3D Circle() => throw new NoSuchObjectException("BiTgte_CurveOnVertex::Circle");
}
