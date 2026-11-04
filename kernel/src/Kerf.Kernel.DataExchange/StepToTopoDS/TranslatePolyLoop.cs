// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslatePolyLoop.hxx
// Port of: src/DataExchange/TKDESTEP/StepToTopoDS/StepToTopoDS_TranslatePolyLoop.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Framework.Transfer;
using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Exchange.Step.Shapes;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Healing.Analysis;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Exchange.Step.Import.TranslatePolyLoopError;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Exchange.Step.Import;

/// <summary>Translates a poly_loop (of a faceted brep) into a wire of line edges on the given planar face.</summary>
/// <remarks><see cref="Value"/> freezes the result; the translators use <c>InternalValue</c> (decision 1247).</remarks>
internal sealed class TranslatePolyLoop : ImportRoot
{
    private TranslatePolyLoopError _error;
    private Shape _result = new Shape();

    public TranslatePolyLoop()
    {
        _error = TranslatePolyLoopError.TranslatePolyLoopOther;
        _done = false;
    }

    public TranslatePolyLoop(StepPolyLoop pl, ImportTool t, Surface s, in Face f,
                                          in Factors localFactors)
    {
        Init(pl, t, s, f, localFactors);
    }

    public TranslatePolyLoop(StepPolyLoop pl, ImportTool t, Surface s, in Face f)
        : this(pl, t, s, f, new Factors())
    {
    }

    /// <remarks>Writes the pcurves of the edges into <paramref name="topoFace"/>, which must be the reader's own.</remarks>
    public void Init(StepPolyLoop pl, ImportTool aTool, Surface geomSurf, in Face topoFace,
                     in Factors localFactors)
    {
        if (!aTool.IsBound(pl))
        {
            var b = new BrepBuilder();
            TransientProcess tp = aTool.TransientProcess();

            //: S4136    double preci = BRepAPI::Precision();
            int i;
            StepCartesianPoint p1, p2;
            CartesianPoint gp1, gp2;
            Vertex v1 = new Vertex(), v2 = new Vertex();
            Edge e = new Edge();
            Wire w = new Wire();
            LineCurve3D l;
            LineCurve2D l2d;
            Vector3D v;
            Vector2D v2d;
            double magn;
            PlaneSurface sp = geomSurf as PlaneSurface;
            if (sp is null)
            {
                tp.AddFail(pl, "Surface not planar in a FacetedBRep !");
            }

            var stsu = new SurfaceAnalysis(geomSurf);
            int nb = pl.NbPolygon();
            var poly = new NCollection_HArray1<StepCartesianPoint>(1, nb + 1);

            for (i = 1; i <= nb; i++)
            {
                poly.SetValue(i, pl.PolygonValue(i));
            }

            nb++;
            poly.SetValue(nb, pl.PolygonValue(1));
            p1 = poly.Value(1);
            gp1 = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeCartesianPoint(p1, localFactors);
            if (aTool.IsVertexBound(p1))
            {
                v1 = aTool.FindVertex(p1);
            }
            else
            {
                b.MakeVertex(ref v1, gp1.Pnt(), Kerf.Engine.Foundation.Precision.Confusion()); //: S4136: preci
                aTool.BindVertex(p1, v1);
            }

            b.MakeWire(ref w);
            for (i = 2; i <= nb; i++)
            {
                p2 = poly.Value(i);
                if (p1 == p2)
                {
                    continue; // peut arriver (KK)  CKY 9-DEC-1997
                }

                var pp = new PointPair(p1, p2);
                gp2 = global::Kerf.Engine.Exchange.Step.Import.Geometry.StepToGeom.MakeCartesianPoint(p2, localFactors);
                Shape boundEdge;
                bool isbound = aTool.IsEdgeBound(pp);
                if (!isbound)
                {
                    if (aTool.IsVertexBound(p2))
                    {
                        v2 = aTool.FindVertex(p2);
                    }
                    else
                    {
                        b.MakeVertex(ref v2, gp2.Pnt(), Kerf.Engine.Foundation.Precision.Confusion()); //: S4136: preci
                        aTool.BindVertex(p2, v2);
                    }

                    v = new Vector3D(gp1.Pnt(), gp2.Pnt());
                    l = new LineCurve3D(gp1.Pnt(), new Direction3D(v));
                    b.MakeEdge(ref e, l, Kerf.Engine.Foundation.Precision.Confusion()); //: S4136: preci
                    v1.Orient(Forward);
                    v2.Orient(Reversed);
                    b.Add(e, v1);
                    b.Add(e, v2);
                    magn = v.Magnitude;
                    b.UpdateVertex(v1, 0.0, e, 0.0);  //: S4136: preci
                    b.UpdateVertex(v2, magn, e, 0.0); //: S4136: preci
                }
                else
                {
                    boundEdge = aTool.FindEdge(pp);
                    e = ShapeCasts.Edge(boundEdge);
                    //  Il faut qu en finale l edge soit vue
                    //  - via sa premiere face, orientation combinee = celle de cette premiere face
                    //  - via sa deuxieme face, orientation combinee INVERSE de la precedente
                    if (topoFace.Orientation == Forward)
                    {
                        e.Reverse();
                    }

                    v2 = aTool.FindVertex(p2);
                }

                Point2D v2p1 = stsu.ValueOfUV(gp1.Pnt(), Precision());
                Point2D v2p2 = stsu.ValueOfUV(gp2.Pnt(), Precision());
                if (e.Orientation == Forward)
                {
                    v2d = new Vector2D(v2p1, v2p2);
                    l2d = new LineCurve2D(v2p1, new Direction2D(v2d));
                }
                else
                {
                    v2d = new Vector2D(v2p2, v2p1);
                    l2d = new LineCurve2D(v2p2, new Direction2D(v2d));
                }

                b.UpdateEdge(e, l2d, topoFace, 0.0);
                Edge eb = e; // pour le binding : cumul des orientations !
                eb.Orient(topoFace.Orientation);
                if (!isbound)
                {
                    aTool.BindEdge(pp, eb);
                }

                if (!e.IsNull)
                {
                    b.Add(w, e);
                }

                p1 = p2;
                gp1 = gp2;
                v1 = v2;
            }

            w.Closed(Tool.IsClosed(w));
            aTool.Bind(pl, w);
            _result = w;
            _error = TranslatePolyLoopError.TranslatePolyLoopDone;
            _done = true;
        }
        else
        {
            _result = ShapeCasts.Wire(aTool.Find(pl));
            _error = TranslatePolyLoopError.TranslatePolyLoopDone;
            _done = true;
        }
    }

    public void Init(StepPolyLoop pl, ImportTool aTool, Surface geomSurf, in Face topoFace)
        => Init(pl, aTool, geomSurf, topoFace, new Factors());

    /// <summary>The wire, frozen.</summary>
    /// <exception cref="NotDoneException">The translation wasn't done.</exception>
    public Shape Value()
    {
        Shape shape = InternalValue();
        shape.TShape()?.FreezeGraph();
        return shape;
    }

    /// <summary>As <see cref="Value"/>, without freezing: the reader's working shape.</summary>
    internal Shape InternalValue()
    {
        StdFail_NotDone_Raise_if(!_done, "StepToTopoDS_TranslatePolyLoop::Value() - no result");
        return _result;
    }

    public TranslatePolyLoopError Error() => _error;
}
