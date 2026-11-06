// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;

namespace Kerf.Engine.Fillets;

/// <summary>
/// Puts faces on the surfaces the fillet extended (<c>DsDataStructure.NewSurface</c>), as
/// <c>TopOpeBRepBuild_Builder::BuildFaces</c> does with <c>UpdateSurface</c> (decision 1721). An extended surface keeps
/// the old one's parameters, so each edge's pcurve moves across as it is. OCCT writes into the edges; the kept edges
/// here are the frozen input's, so <see cref="Modifier"/> copies what it has to, neighbouring faces included.
/// </summary>
internal sealed class NewSurfaceModification(Dictionary<Shape, Surface> theSurfaces) : Modification
{
    /// <inheritdoc/>
    public override bool NewSurface(in Face f, out Surface? s, out Placement l, out double tol, out bool revWires, out bool revFace)
    {
        revWires = revFace = false;
        tol = Tool.Tolerance(f);
        _ = Tool.Surface(f, out l);
        return theSurfaces.TryGetValue(f, out s);
    }

    /// <inheritdoc/>
    public override bool NewCurve(in Edge e, out Curve3D? c, out Placement l, out double tol)
    {
        c = null;
        l = new Placement();
        tol = 0.0;
        return false;
    }

    /// <inheritdoc/>
    public override bool NewPoint(in Vertex v, out Point3D p, out double tol)
    {
        p = default;
        tol = 0.0;
        return false;
    }

    /// <inheritdoc/>
    public override bool NewCurve2d(in Edge e, in Face f, in Edge newE, in Face newF, out Curve2D? c, out double tol)
    {
        tol = Tool.Tolerance(e);
        if (!theSurfaces.ContainsKey(f))
        {
            c = null;
            return false;
        }
        c = Tool.CurveOnSurface(e, f, out _, out _);
        return c is not null;
    }

    /// <inheritdoc/>
    public override bool NewParameter(in Vertex v, in Edge e, ref double p, ref double tol) => false;

    /// <inheritdoc/>
    public override Continuity Continuity(in Edge e, in Face f1, in Face f2, in Edge newE, in Face newF1, in Face newF2) =>
        Tool.Continuity(e, f1, f2);
}
