// Port of: src/ModelingData/TKBRep/BRepAdaptor/BRepAdaptor_Curve2d.hxx
// Port of: src/ModelingData/TKBRep/BRepAdaptor/BRepAdaptor_Curve2d.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;

using Kerf.Engine.Topology.Brep;

using Kerf.Geometry;

using Kerf.Topology;

using Occt;

namespace Kerf.Engine.Topology.Adaptors;

/// <summary>The pcurve of an edge on a face, seen as a 2D curve.</summary>
/// <remarks>A working object (decision 214).</remarks>
internal class EdgeCurve2DAdaptor : CurveAdaptor2D
{
    private Edge _edge = new();
    private Face _face = new();

    /// <summary>An uninitialised curve.</summary>
    public EdgeCurve2DAdaptor()
    {
    }

    /// <summary>The pcurve of the edge on the face.</summary>
    public EdgeCurve2DAdaptor(in Edge e, in Face f)
    {
        Initialize(e, f);
    }

    /// <summary>A shallow copy: the curve is shared, the evaluation caches are not.</summary>
    public override Curve2DAdaptorBase ShallowCopy()
    {
        EdgeCurve2DAdaptor copy = new EdgeCurve2DAdaptor();

        // OCCT copies the curve, its type, range and evaluation data by hand ("cache will be rebuilt on demand");
        // the port does it with Geom2dAdaptor_Curve's own shallow copy
        ShallowCopyTo(copy);
        copy._edge = _edge;
        copy._face = _face;

        return copy;
    }

    /// <summary>Loads the pcurve of the edge on the face (nothing is loaded if there is none).</summary>
    public void Initialize(in Edge e, in Face f)
    {
        _edge = e;
        _face = f;
        Curve2D? pCurve = Tool.CurveOnSurface(e, f, out double firs, out double last);
        if (pCurve is not null)
        {
            Load(pCurve, firs, last);
        }
    }

    /// <summary>The edge.</summary>
    public Edge Edge() => _edge;

    /// <summary>The face.</summary>
    public Face Face() => _face;
}
