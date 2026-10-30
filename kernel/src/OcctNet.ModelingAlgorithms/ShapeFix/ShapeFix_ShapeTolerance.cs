// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_ShapeTolerance.hxx
// Port of: src/ModelingAlgorithms/TKShHealing/ShapeFix/ShapeFix_ShapeTolerance.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

// 25.12.98 pdn: adding empty constructor

using static Occt.TopAbs_ShapeEnum;

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
    internal bool LimitTolerance(in TopoDS_Shape shape, double tmin, double tmax = 0.0, TopAbs_ShapeEnum styp = TopAbs_SHAPE)
    {
        if (shape.IsNull() || tmin < 0)
        {
            return false;
        }
        bool   iamax = (tmax >= tmin);
        double prec;
        bool   fait = false;
        if (styp == TopAbs_VERTEX || styp == TopAbs_EDGE || styp == TopAbs_FACE)
        {
            for (var ex = new TopExp_Explorer(shape, styp); ex.More(); ex.Next())
            {
                TopoDS_Shape sh     = ex.Current();
                int          newtol = 0;
                if (styp == TopAbs_VERTEX)
                {
                    TopoDS_Vertex V = TopoDS.Vertex(sh);
                    prec = BRep_Tool.Tolerance(V);
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
                        var TV = (BRep_TVertex)V.TShape()!;
                        TV.Tolerance((newtol > 0 ? tmax : tmin));
                        fait = true;
                    }
                }
                else if (styp == TopAbs_EDGE)
                {
                    TopoDS_Edge E = TopoDS.Edge(sh);
                    prec = BRep_Tool.Tolerance(E);
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
                        var TE = (BRep_TEdge)E.TShape()!;
                        TE.Tolerance((newtol > 0 ? tmax : tmin));
                        fait = true;
                    }
                }
                else if (styp == TopAbs_FACE)
                {
                    TopoDS_Face F = TopoDS.Face(sh);
                    prec = BRep_Tool.Tolerance(F);
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
                        var TF = (BRep_TFace)F.TShape()!;
                        TF.Tolerance((newtol > 0 ? tmax : tmin));
                        fait = true;
                    }
                }
            }
        }
        else if (styp == TopAbs_WIRE)
        {
            for (var ex = new TopExp_Explorer(shape, TopAbs_EDGE); ex.More(); ex.Next())
            {
                TopoDS_Shape sh = ex.Current();
                TopoDS_Edge  E  = TopoDS.Edge(sh);
                // OCCT: the edges' own result isn't added to fait (only their vertices'): a change of edge
                // tolerances alone returns false
                LimitTolerance(E, tmin, tmax, TopAbs_EDGE);
                TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                if (!V1.IsNull())
                {
                    fait |= LimitTolerance(V1, tmin, tmax, TopAbs_VERTEX);
                }
                if (!V2.IsNull())
                {
                    fait |= LimitTolerance(V2, tmin, tmax, TopAbs_VERTEX);
                }
            }
        }
        else
        {
            fait |= LimitTolerance(shape, tmin, tmax, TopAbs_VERTEX);
            fait |= LimitTolerance(shape, tmin, tmax, TopAbs_EDGE);
            fait |= LimitTolerance(shape, tmin, tmax, TopAbs_FACE);
        }
        return fait;
    }

    /// <summary>
    /// Sets the tolerances of the sub-shapes of type <paramref name="styp"/> (as <see cref="LimitTolerance"/>) to
    /// <paramref name="preci"/> (nothing if it isn't positive). Writes into the shape.
    /// </summary>
    internal void SetTolerance(in TopoDS_Shape shape, double preci, TopAbs_ShapeEnum styp = TopAbs_SHAPE)
    {
        //   VERTEX ou EDGE ou FACE : ces types seulement
        //   WIRE : EDGE + VERTEX
        //   Autres : TOUT (donc == WIRE + FACE)
        if (shape.IsNull() || preci <= 0)
        {
            return;
        }
        if (styp == TopAbs_VERTEX || styp == TopAbs_EDGE || styp == TopAbs_FACE)
        {
            for (var ex = new TopExp_Explorer(shape, styp); ex.More(); ex.Next())
            {
                TopoDS_Shape sh = ex.Current();
                if (styp == TopAbs_VERTEX)
                {
                    TopoDS_Vertex V = TopoDS.Vertex(sh);
                    //	B.UpdateVertex (V,preci);
                    var TV = (BRep_TVertex)V.TShape()!;
                    TV.Tolerance(preci);
                }
                else if (styp == TopAbs_EDGE)
                {
                    TopoDS_Edge E = TopoDS.Edge(sh);
                    //	B.UpdateEdge   (E,preci);
                    var TE = (BRep_TEdge)E.TShape()!;
                    TE.Tolerance(preci);
                }
                else if (styp == TopAbs_FACE)
                {
                    TopoDS_Face F = TopoDS.Face(sh);
                    //	B.UpdateFace   (F,preci);
                    var TF = (BRep_TFace)F.TShape()!;
                    TF.Tolerance(preci);
                }
            }
        }
        else if (styp == TopAbs_WIRE)
        {
            for (var ex = new TopExp_Explorer(shape, TopAbs_EDGE); ex.More(); ex.Next())
            {
                TopoDS_Shape sh = ex.Current();
                TopoDS_Edge  E  = TopoDS.Edge(sh);
                //      B.UpdateEdge   (E,preci);
                var TE = (BRep_TEdge)E.TShape()!;
                TE.Tolerance(preci);
                TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                if (!V1.IsNull())
                {
                    //	B.UpdateVertex (V1,preci);
                    var TV = (BRep_TVertex)V1.TShape()!;
                    TV.Tolerance(preci);
                }
                if (!V2.IsNull())
                {
                    //	B.UpdateVertex (V2,preci);
                    var TV = (BRep_TVertex)V2.TShape()!;
                    TV.Tolerance(preci);
                }
            }
        }
        else
        {
            SetTolerance(shape, preci, TopAbs_VERTEX);
            SetTolerance(shape, preci, TopAbs_EDGE);
            SetTolerance(shape, preci, TopAbs_FACE);
        }
    }

    // ---- OCCT.NET: the functional twins (design 4.6, decision 583) ----

    /// <summary>
    /// The shape with the tolerances limited as <see cref="LimitTolerance"/> does, frozen: a frozen shape is copied
    /// first, and the sub-shapes left unchanged are the caller's own; a shape needing no change is returned itself.
    /// <paramref name="theIsDone"/> is <c>LimitTolerance</c>'s result (true if a tolerance changed).
    /// </summary>
    public TopoDS_Shape ToleranceLimited(in TopoDS_Shape shape, double tmin, out bool theIsDone, double tmax = 0.0,
                                         TopAbs_ShapeEnum styp = TopAbs_SHAPE)
    {
        var anEntry = new BRepTools_CopyAtEntry(shape);
        theIsDone = LimitTolerance(anEntry.Shape(), tmin, tmax, styp);
        TopoDS_Shape aResult = anEntry.Finish(anEntry.Shape());
        aResult.TShape()?.FreezeGraph();
        return aResult;
    }

    /// <summary>
    /// The shape with the tolerances set as <see cref="SetTolerance"/> does, frozen; see
    /// <see cref="ToleranceLimited"/>.
    /// </summary>
    public TopoDS_Shape WithTolerance(in TopoDS_Shape shape, double preci, TopAbs_ShapeEnum styp = TopAbs_SHAPE)
    {
        var anEntry = new BRepTools_CopyAtEntry(shape);
        SetTolerance(anEntry.Shape(), preci, styp);
        TopoDS_Shape aResult = anEntry.Finish(anEntry.Shape());
        aResult.TShape()?.FreezeGraph();
        return aResult;
    }
}
