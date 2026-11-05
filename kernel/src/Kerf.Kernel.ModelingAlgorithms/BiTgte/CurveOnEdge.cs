// Port of: src/ModelingAlgorithms/TKOffset/BiTgte/BiTgte_CurveOnEdge.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BiTgte/BiTgte_CurveOnEdge.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Api;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.CurveKind;

namespace Kerf.Engine.Offsets.Blend;

/// <summary>
/// The curve of the rolling ball's centre on an edge: a point of the edge EonF of the offset face projected on the
/// original edge. Only the parameters, D0 (<see cref="EvalD0"/>), the type and a zero-radius circle are defined; the
/// other members throw <see cref="MethodNotImplementedException"/>, as in OCCT (the base class's).
/// </summary>
internal sealed class CurveOnEdge : CurveAdaptorBase
{
    private Edge _edge = new();
    private Edge _eonF = new();
    private Curve3D? _curv;
    private Curve3D? _conF;
    private CurveKind _type;
    private Circle3D _circ = new();

    public CurveOnEdge()
    {
        _type = OtherCurve;
    }

    public CurveOnEdge(in Edge eonF, in Edge edge)
    {
        _edge = edge;
        _eonF = eonF;
        _type = OtherCurve;
        Init(eonF, edge);
    }

    public override CurveAdaptorBase ShallowCopy()
    {
        CurveOnEdge copy = new()
        {
            _edge = _edge,
            _eonF = _eonF,
            _curv = _curv,
            _conF = _conF,
            _type = _type,
            _circ = _circ
        };

        return copy;
    }

    public void Init(in Edge eonF, in Edge edge)
    {
        double f, l;

        _edge = edge;
        _curv = Tool.Curve(_edge, out f, out l);
        _curv = new TrimmedCurve3D(_curv!, f, l);

        _eonF = eonF;
        _conF = Tool.Curve(_eonF, out f, out l);
        _conF = new TrimmedCurve3D(_conF!, f, l);

        // peut on generer un cercle de rayon nul
        CurveAdaptor curv = new(_curv);
        CurveAdaptor conF = new(_conF);

        _type = OtherCurve;
        if (curv.GetType() == global::Kerf.Geometry.CurveKind.Line && conF.GetType() == global::Kerf.Geometry.CurveKind.Circle)
        {
            Axis3D a1 = curv.Line().Position;
            Axis3D a2 = conF.Circle().Axis;
            if (a1.IsCoaxial(a2, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion()))
            {
                _type = global::Kerf.Geometry.CurveKind.Circle;
                _circ = new Circle3D(conF.Circle().Position, 0.0);
            }
        }
    }

    public override double FirstParameter() => _conF!.FirstParameter;

    public override double LastParameter() => _conF!.LastParameter;

    public override Point3D EvalD0(double u)
    {
        ProjectPointOnCurve projector = new();
        Point3D p = _conF!.Value(u);
        projector.Init(p, _curv!);
        return projector.NearestPoint();
    }

    public override CurveKind GetType() => _type;

    /// <exception cref="NoSuchObjectException">The curve isn't a circle.</exception>
    public override Circle3D Circle()
    {
        if (_type != global::Kerf.Geometry.CurveKind.Circle)
        {
            throw new NoSuchObjectException("BiTgte_CurveOnEdge::Circle");
        }

        return _circ;
    }
}
