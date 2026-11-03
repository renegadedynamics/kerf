// Port of: src/ModelingData/TKBRep/BRepAdaptor/BRepAdaptor_Curve.hxx
// Port of: src/ModelingData/TKBRep/BRepAdaptor/BRepAdaptor_Curve.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;

using Kerf.Topology;
using Occt;

namespace Kerf.Engine.Topology.Adaptors;

/// <summary>
/// An edge of the BRep topology seen as a 3D curve, placed by the edge's location. The edge's 3D curve is used if it
/// has one, otherwise its first curve on a surface; a curve on a given face can be asked for explicitly.
/// </summary>
/// <remarks>
/// Most methods come from <see cref="TransformedCurve"/>, which applies the location. A working object
/// (decision 214).
/// </remarks>
public class EdgeCurveAdaptor : TransformedCurve
{
    private Edge _edge = new();

    /// <summary>An undefined curve, with no edge loaded.</summary>
    public EdgeCurveAdaptor()
    {
    }

    /// <summary>The geometry of the edge.</summary>
    /// <exception cref="NullObjectException">The edge has neither a 3D curve nor a curve on a surface.</exception>
    public EdgeCurveAdaptor(in Edge e)
    {
        Initialize(e);
    }

    /// <summary>The geometry of the edge, computed from its pcurve on the face.</summary>
    public EdgeCurveAdaptor(in Edge e, in Face f)
    {
        Initialize(e, f);
    }

    /// <summary>A shallow copy of the adaptor (with shallow copies of the inner adaptors).</summary>
    public override CurveAdaptorBase ShallowCopy()
    {
        EdgeCurveAdaptor copy = new EdgeCurveAdaptor();

        CurveAdaptorBase curve = _curve.ShallowCopy();
        CurveAdaptor geomCurve = (CurveAdaptor)curve;
        copy._curve = geomCurve;

        if (_conSurf is not null)
        {
            copy._conSurf = (CurveOnSurface)_conSurf.ShallowCopy();
        }
        copy._trsf = _trsf;
        copy._edge = _edge;

        return copy;
    }

    /// <summary>Unloads the edge.</summary>
    public void Reset()
    {
        _curve.Reset();
        _conSurf = null;
        _edge.Nullify();
        _trsf = new Transform3D();
    }

    /// <summary>Loads the geometry of the edge.</summary>
    /// <exception cref="NullObjectException">The edge has neither a 3D curve nor a curve on a surface.</exception>
    public void Initialize(in Edge e)
    {
        _conSurf = null;
        _edge = e;

        Curve3D? c = Tool.Curve(e, out Placement l, out double pf, out double pl);

        if (c is not null)
        {
            _curve.Load(c, pf, pl);
        }
        else
        {
            Tool.CurveOnSurface(e, out Curve2D? pc, out Surface? s, out l, out pf, out pl);
            if (pc is not null)
            {
                SurfaceAdaptor hs = new SurfaceAdaptor();
                hs.Load(s!);
                CurveAdaptor2D hc = new CurveAdaptor2D();
                hc.Load(pc, pf, pl);
                _conSurf = new CurveOnSurface();
                _conSurf.Load(hc, hs);
            }
            else
            {
                throw new NullObjectException("BRepAdaptor_Curve::No geometry");
            }
        }
        _trsf = l.Transformation();
    }

    /// <summary>Loads the geometry of the edge, computed from its pcurve on the face.</summary>
    public void Initialize(in Edge e, in Face f)
    {
        _conSurf = null;

        _edge = e;
        Surface? s = Tool.Surface(f, out Placement l);
        Curve2D? pc = Tool.CurveOnSurface(e, f, out double pf, out double pl);

        SurfaceAdaptor hs = new SurfaceAdaptor();
        hs.Load(s!);
        CurveAdaptor2D hc = new CurveAdaptor2D();
        hc.Load(pc!, pf, pl);
        _conSurf = new CurveOnSurface();
        _conSurf.Load(hc, hs);

        _trsf = l.Transformation();
    }

    /// <summary>The edge.</summary>
    public Edge Edge() => _edge;

    /// <summary>The edge's tolerance.</summary>
    public double Tolerance() => Tool.Tolerance(_edge);

    /// <summary>The curve between First and Last, keeping the edge and the transformation.</summary>
    public override CurveAdaptorBase Trim(double first, double last, double tol)
    {
        // Make a copy of this to keep the transformation.
        // OCCT loads the range into its own adaptor, copies itself and restores the range; the port builds the copy
        // directly, with its own adaptors (the copy shares no cache, decision 214).
        EdgeCurveAdaptor res = new EdgeCurveAdaptor();
        if (_conSurf is null)
        {
            Curve3D? c = _curve.Curve();
            res._curve.Load(c!, first, last);
        }
        else
        {
            res._conSurf = (CurveOnSurface)_conSurf.Trim(first, last, tol);
        }
        res._trsf = _trsf;
        res._edge = _edge;
        return res;
    }
}
