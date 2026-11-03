// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_ShapeTolerance.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_ShapeTolerance.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// 25.12.98 pdn: adding empty constructor

using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Topology;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>Limits or sets the tolerances of the vertices, edges and faces of a shape.</summary>
/// <remarks>
/// A stateless working object. OCCT's <c>LimitTolerance</c> and <c>SetTolerance</c> write the tolerances in place,
/// so they are internal (ported code on shapes it owns). Callers use the twins <see cref="ToleranceLimited"/> and
/// <see cref="WithTolerance"/>, which work on a copy of a frozen shape, give back its unchanged sub-shapes and freeze
/// the result (decision 583).
/// </remarks>
public sealed class ShapeFix_ShapeTolerance
{
    /// <summary>Empty constructor.</summary>
    public ShapeFix_ShapeTolerance()
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
                    Vertex V = ShapeCasts.Vertex(sh);
                    prec = Tool.Tolerance(V);
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
                        var TV = (BrepTVertex)V.TShape()!;
                        TV.Tolerance((newtol > 0 ? tmax : tmin));
                        fait = true;
                    }
                }
                else if (styp == ShapeType.Edge)
                {
                    Edge E = ShapeCasts.Edge(sh);
                    prec = Tool.Tolerance(E);
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
                        var TE = (BrepTEdge)E.TShape()!;
                        TE.Tolerance((newtol > 0 ? tmax : tmin));
                        fait = true;
                    }
                }
                else if (styp == ShapeType.Face)
                {
                    Face F = ShapeCasts.Face(sh);
                    prec = Tool.Tolerance(F);
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
                        var TF = (BrepTFace)F.TShape()!;
                        TF.Tolerance((newtol > 0 ? tmax : tmin));
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
                Edge  E  = ShapeCasts.Edge(sh);
                // OCCT: the edges' own result isn't added to fait (only their vertices'): a change of edge
                // tolerances alone returns false
                LimitTolerance(E, tmin, tmax, ShapeType.Edge);
                Exploration.Vertices(E, out Vertex V1, out Vertex V2);
                if (!V1.IsNull)
                {
                    fait |= LimitTolerance(V1, tmin, tmax, ShapeType.Vertex);
                }
                if (!V2.IsNull)
                {
                    fait |= LimitTolerance(V2, tmin, tmax, ShapeType.Vertex);
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
                    Vertex V = ShapeCasts.Vertex(sh);
                    //	B.UpdateVertex (V,preci);
                    var TV = (BrepTVertex)V.TShape()!;
                    TV.Tolerance(preci);
                }
                else if (styp == ShapeType.Edge)
                {
                    Edge E = ShapeCasts.Edge(sh);
                    //	B.UpdateEdge   (E,preci);
                    var TE = (BrepTEdge)E.TShape()!;
                    TE.Tolerance(preci);
                }
                else if (styp == ShapeType.Face)
                {
                    Face F = ShapeCasts.Face(sh);
                    //	B.UpdateFace   (F,preci);
                    var TF = (BrepTFace)F.TShape()!;
                    TF.Tolerance(preci);
                }
            }
        }
        else if (styp == ShapeType.Wire)
        {
            for (var ex = new Explorer(shape, ShapeType.Edge); ex.More(); ex.Next())
            {
                Shape sh = ex.Current();
                Edge  E  = ShapeCasts.Edge(sh);
                //      B.UpdateEdge   (E,preci);
                var TE = (BrepTEdge)E.TShape()!;
                TE.Tolerance(preci);
                Exploration.Vertices(E, out Vertex V1, out Vertex V2);
                if (!V1.IsNull)
                {
                    //	B.UpdateVertex (V1,preci);
                    var TV = (BrepTVertex)V1.TShape()!;
                    TV.Tolerance(preci);
                }
                if (!V2.IsNull)
                {
                    //	B.UpdateVertex (V2,preci);
                    var TV = (BrepTVertex)V2.TShape()!;
                    TV.Tolerance(preci);
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
    /// The shape with the tolerances limited as <see cref="LimitTolerance"/> does, frozen: a frozen shape is copied
    /// first, and the sub-shapes left unchanged are the caller's own; a shape needing no change is returned itself.
    /// <paramref name="theIsDone"/> is <c>LimitTolerance</c>'s result (true if a tolerance changed).
    /// </summary>
    public Shape ToleranceLimited(in Shape shape, double tmin, out bool theIsDone, double tmax = 0.0,
                                         ShapeType styp = ShapeType.Shape)
    {
        var anEntry = new CopyAtEntry(shape);
        theIsDone = LimitTolerance(anEntry.Shape(), tmin, tmax, styp);
        Shape aResult = anEntry.Finish(anEntry.Shape());
        aResult.TShape()?.FreezeGraph();
        return aResult;
    }

    /// <summary>
    /// The shape with the tolerances set as <see cref="SetTolerance"/> does, frozen; see
    /// <see cref="ToleranceLimited"/>.
    /// </summary>
    public Shape WithTolerance(in Shape shape, double preci, ShapeType styp = ShapeType.Shape)
    {
        var anEntry = new CopyAtEntry(shape);
        SetTolerance(anEntry.Shape(), preci, styp);
        Shape aResult = anEntry.Finish(anEntry.Shape());
        aResult.TShape()?.FreezeGraph();
        return aResult;
    }
}
