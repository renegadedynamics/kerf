// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_ShapeTolerance.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_ShapeTolerance.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// 25.12.98 pdn: adding empty constructor

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Healing.Fixes;

/// <summary>Limits or sets the tolerances of the vertices, edges and faces of a shape.</summary>
/// <remarks>
/// A stateless working object. OCCT's <c>LimitTolerance</c> and <c>SetTolerance</c> write the tolerances in place,
/// so they are internal (ported code on shapes it owns). Callers use the twins <see cref="ToleranceLimited"/> and
/// <see cref="WithTolerance"/>, which run them in a working overlay (Kerf unit 10H1): the caller's frozen shape isn't
/// written, the result shares its unchanged sub-shapes and is frozen.
/// </remarks>
public sealed class ShapeToleranceFixer
{
    /// <summary>Empty constructor.</summary>
    public ShapeToleranceFixer()
    {
    }

    /// <summary>
    /// Limits the tolerances of the sub-shapes of type <paramref name="styp"/> (VERTEX, EDGE or FACE; WIRE for edges
    /// and their vertices; any other for all three) to [tmin, tmax] (no maximum if tmax &lt; tmin). Returns true if a
    /// tolerance changed. Writes into the shape.
    /// </summary>
    internal bool LimitTolerance(in Shape shape, double tmin, double tmax = 0.0, ShapeType styp = ShapeType.Shape)
    {
        if (shape.IsNull || tmin < 0)
        {
            return false;
        }
        bool   iamax = (tmax >= tmin);
        double prec;
        bool   fait = false;
        if (styp == ShapeType.Vertex || styp == ShapeType.Edge || styp == ShapeType.Face)
        {
            for (var ex = new Explorer(shape, styp); ex.More(); ex.Next())
            {
                Shape sh     = ex.Current();
                int          newtol = 0;
                if (styp == ShapeType.Vertex)
                {
                    Vertex v = ShapeCasts.Vertex(sh);
                    prec = Tool.Tolerance(v);
                    if (iamax && prec > tmax)
                    {
                        newtol = 1;
                    }
                    else if (prec < tmin)
                    {
                        newtol = -1;
                    }
                    if (newtol != 0)
                    {
                        var tv = (BrepTVertex)v.TShape()!;
                        tv.Tolerance((newtol > 0 ? tmax : tmin));
                        fait = true;
                    }
                }
                else if (styp == ShapeType.Edge)
                {
                    Edge e = ShapeCasts.Edge(sh);
                    prec = Tool.Tolerance(e);
                    if (iamax && prec > tmax)
                    {
                        newtol = 1;
                    }
                    else if (prec < tmin)
                    {
                        newtol = -1;
                    }
                    if (newtol != 0)
                    {
                        var te = (BrepTEdge)e.TShape()!;
                        te.Tolerance((newtol > 0 ? tmax : tmin));
                        fait = true;
                    }
                }
                else if (styp == ShapeType.Face)
                {
                    Face f = ShapeCasts.Face(sh);
                    prec = Tool.Tolerance(f);
                    if (iamax && prec > tmax)
                    {
                        newtol = 1;
                    }
                    else if (prec < tmin)
                    {
                        newtol = -1;
                    }
                    if (newtol != 0)
                    {
                        var tf = (BrepTFace)f.TShape()!;
                        tf.Tolerance((newtol > 0 ? tmax : tmin));
                        fait = true;
                    }
                }
            }
        }
        else if (styp == ShapeType.Wire)
        {
            for (var ex = new Explorer(shape, ShapeType.Edge); ex.More(); ex.Next())
            {
                Shape sh = ex.Current();
                Edge  e  = ShapeCasts.Edge(sh);
                // OCCT: the edges' own result isn't added to fait (only their vertices'): a change of edge
                // tolerances alone returns false
                LimitTolerance(e, tmin, tmax, ShapeType.Edge);
                Exploration.Vertices(e, out Vertex v1, out Vertex v2);
                if (!v1.IsNull)
                {
                    fait |= LimitTolerance(v1, tmin, tmax, ShapeType.Vertex);
                }
                if (!v2.IsNull)
                {
                    fait |= LimitTolerance(v2, tmin, tmax, ShapeType.Vertex);
                }
            }
        }
        else
        {
            fait |= LimitTolerance(shape, tmin, tmax, ShapeType.Vertex);
            fait |= LimitTolerance(shape, tmin, tmax, ShapeType.Edge);
            fait |= LimitTolerance(shape, tmin, tmax, ShapeType.Face);
        }
        return fait;
    }

    /// <summary>
    /// Sets the tolerances of the sub-shapes of type <paramref name="styp"/> (as <see cref="LimitTolerance"/>) to
    /// <paramref name="preci"/> (nothing if it isn't positive). Writes into the shape.
    /// </summary>
    internal void SetTolerance(in Shape shape, double preci, ShapeType styp = ShapeType.Shape)
    {
        //   VERTEX ou EDGE ou FACE : ces types seulement
        //   WIRE : EDGE + VERTEX
        //   Autres : TOUT (donc == WIRE + FACE)
        if (shape.IsNull || preci <= 0)
        {
            return;
        }
        if (styp == ShapeType.Vertex || styp == ShapeType.Edge || styp == ShapeType.Face)
        {
            for (var ex = new Explorer(shape, styp); ex.More(); ex.Next())
            {
                Shape sh = ex.Current();
                if (styp == ShapeType.Vertex)
                {
                    Vertex v = ShapeCasts.Vertex(sh);
                    //	B.UpdateVertex (V,preci);
                    var tv = (BrepTVertex)v.TShape()!;
                    tv.Tolerance(preci);
                }
                else if (styp == ShapeType.Edge)
                {
                    Edge e = ShapeCasts.Edge(sh);
                    //	B.UpdateEdge   (E,preci);
                    var te = (BrepTEdge)e.TShape()!;
                    te.Tolerance(preci);
                }
                else if (styp == ShapeType.Face)
                {
                    Face f = ShapeCasts.Face(sh);
                    //	B.UpdateFace   (F,preci);
                    var tf = (BrepTFace)f.TShape()!;
                    tf.Tolerance(preci);
                }
            }
        }
        else if (styp == ShapeType.Wire)
        {
            for (var ex = new Explorer(shape, ShapeType.Edge); ex.More(); ex.Next())
            {
                Shape sh = ex.Current();
                Edge  e  = ShapeCasts.Edge(sh);
                //      B.UpdateEdge   (E,preci);
                var te = (BrepTEdge)e.TShape()!;
                te.Tolerance(preci);
                Exploration.Vertices(e, out Vertex v1, out Vertex v2);
                if (!v1.IsNull)
                {
                    //	B.UpdateVertex (V1,preci);
                    var tv = (BrepTVertex)v1.TShape()!;
                    tv.Tolerance(preci);
                }
                if (!v2.IsNull)
                {
                    //	B.UpdateVertex (V2,preci);
                    var tv = (BrepTVertex)v2.TShape()!;
                    tv.Tolerance(preci);
                }
            }
        }
        else
        {
            SetTolerance(shape, preci, ShapeType.Vertex);
            SetTolerance(shape, preci, ShapeType.Edge);
            SetTolerance(shape, preci, ShapeType.Face);
        }
    }

    // ---- OCCT.NET: the functional twins (design 4.6, decision 583) ----

    /// <summary>
    /// The shape with the tolerances limited as <see cref="LimitTolerance"/> does, frozen: the sub-shapes left unchanged
    /// are the caller's own; a shape needing no change is returned itself.
    /// <paramref name="isDone"/> is <c>LimitTolerance</c>'s result (true if a tolerance changed).
    /// </summary>
    public Shape ToleranceLimited(in Shape shape, double tmin, out bool isDone, double tmax = 0.0,
                                         ShapeType styp = ShapeType.Shape)
    {
        Shape work = WorkingOverlay.WithUnfrozenCopied(shape, out Dictionary<TShape, TShape>? copies);
        using var overlay = WorkingOverlay.Begin(work);
        isDone = LimitTolerance(work, tmin, tmax, styp);
        Shape result = WorkingOverlay.UncopiedWhereUnchanged(overlay.Finish(work), copies);
        result.TShape()?.FreezeGraph();
        return result;
    }

    /// <summary>
    /// The shape with the tolerances set as <see cref="SetTolerance"/> does, frozen; see
    /// <see cref="ToleranceLimited"/>.
    /// </summary>
    public Shape WithTolerance(in Shape shape, double preci, ShapeType styp = ShapeType.Shape)
    {
        Shape work = WorkingOverlay.WithUnfrozenCopied(shape, out Dictionary<TShape, TShape>? copies);
        using var overlay = WorkingOverlay.Begin(work);
        SetTolerance(work, preci, styp);
        Shape result = WorkingOverlay.UncopiedWhereUnchanged(overlay.Finish(work), copies);
        result.TShape()?.FreezeGraph();
        return result;
    }
}
