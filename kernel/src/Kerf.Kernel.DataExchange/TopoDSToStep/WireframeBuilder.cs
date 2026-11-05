// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_WireframeBuilder.hxx
// Port of: src/DataExchange/TKDESTEP/TopoDSToStep/TopoDSToStep_WireframeBuilder.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

#nullable disable warnings

using Kerf.Engine.Exchange.Step.Data;
using Kerf.Engine.Exchange.Step.Export.Geometry;
using Kerf.Engine.Exchange.Step.Geometry;
using Kerf.Engine.Foundation;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Exchange.Step.Geometry.StepTrimmingPreference;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Engine.Exchange.Step.Export.ExportBuilderError;

namespace Kerf.Engine.Exchange.Step.Export;

/// <summary>Builds the trimmed curves of the edges of a shape (an edge, wire, face, shell, solid or compound).</summary>
internal sealed class WireframeBuilder : ExportRoot
{
    private const int Nbpt = 23;

    private NCollection_HSequence<Transient> _result;
    private ExportBuilderError _error;

    public WireframeBuilder()
    {
        _error = ExportBuilderError.BuilderOther;
        _done = false;
    }

    public WireframeBuilder(in Shape aShape, ExportTool aTool, in Factors localFactors)
    {
        _done = false;
        Init(aShape, aTool, localFactors);
    }

    public WireframeBuilder(in Shape aShape, ExportTool aTool) : this(aShape, aTool, new Factors())
    {
    }

    public void Init(in Shape aShape, ExportTool t, in Factors localFactors)
    {
        _ = t;
        var itemList = new NCollection_HSequence<Transient>();
        var pmsMap = new Dictionary<Shape, Transient>(ShapeHasher.Instance);
        _done = GetTrimmedCurveFromShape(aShape, pmsMap, ref itemList, localFactors);
        _result = itemList;
    }

    public void Init(in Shape aShape, ExportTool t) => Init(aShape, t, new Factors());

    public ExportBuilderError Error() => _error;

    /// <summary>The curves made.</summary>
    /// <exception cref="NotDoneException">The build wasn't done.</exception>
    public NCollection_HSequence<Transient> Value()
    {
        StdFail_NotDone_Raise_if(!_done, "TopoDSToStep_WireframeBuilder::Value() - no result");
        return _result;
    }

    //: S4134: abv 10 Mar 99: the methods below moved from package TopoDSToGBWire

    private static StepTrimmedCurve MakeTrimmedCurve(StepCurve c, StepCartesianPoint p1, StepCartesianPoint p2,
                                                          double trim1, double trim2, bool sense)
    {
        var sts1 = new NCollection_HArray1<StepTrimmingSelect>(1, 2);
        var tSel = new StepTrimmingSelect();
        tSel.SetValue(p1);
        sts1.SetValue(1, new StepTrimmingSelect(tSel)); // C++ stores a copy
        tSel.SetParameterValue(trim1);
        sts1.SetValue(2, new StepTrimmingSelect(tSel));

        var sts2 = new NCollection_HArray1<StepTrimmingSelect>(1, 2);
        tSel.SetValue(p2);
        sts2.SetValue(1, new StepTrimmingSelect(tSel));
        tSel.SetParameterValue(trim2);
        sts2.SetValue(2, new StepTrimmingSelect(tSel));

        var empty = new TCollection_HAsciiString("");
        var pmsTC = new StepTrimmedCurve();
        pmsTC.Init(empty, c, sts1, sts2, sense, StepTrimmingPreference.TpParameter);
        return pmsTC;
    }

    /// <summary>Extracts the trimmed curve of an edge (on a face, which may be null), into the list.</summary>
    public bool GetTrimmedCurveFromEdge(in Edge edge, in Face face, Dictionary<Shape, Transient> map,
                                        ref NCollection_HSequence<Transient> curveList, in Factors localFactors)
    {
        if (edge.Orientation == Internal || edge.Orientation == External)
        {
            // OCCT_DEBUG output left out
            return false;
        }

        // szv#4:S4163:12Mar99 SGI warns
        Shape sh = edge.Oriented(Forward);
        Edge anEdge = ShapeCasts.Edge(sh);

        // resulting curve
        StepCurve sgc = null;
        if (map.TryGetValue(anEdge, out Transient transient))
        {
            sgc = transient as StepCurve; // OCCT: found, then made again
        }

        var ca = new EdgeCurveAdaptor();
        try
        {
            ca.Initialize(anEdge);
        }
        catch (Exception caught) when (Refusal.Of(caught) is NullObjectException)
        {
            return false;
        }

        Vertex vFirst = new Vertex(), vLast = new Vertex();
        StepCartesianPoint sgcp1 = null, sgcp2 = null;
        for (var exp = new Explorer(anEdge, ShapeType.Vertex); exp.More(); exp.Next())
        {
            Vertex vertex = ShapeCasts.Vertex(exp.Value());
            Point3D gpP = Tool.Pnt(vertex);
            if (vertex.Orientation == Forward)
            {
                vFirst = vertex;
                // 1.point for trimming
                var gtsmcp = new MakeCartesianPoint(gpP, localFactors.LengthFactor());
                sgcp1 = gtsmcp.Value();
            }

            if (vertex.Orientation == Reversed)
            {
                vLast = vertex;
                // 2.point for trimming
                var gtsmcp = new MakeCartesianPoint(gpP, localFactors.LengthFactor());
                sgcp2 = gtsmcp.Value();
            }
        }

        Curve3D c = Tool.Curve(anEdge, out double first, out double last);

        if (c is not null)
        {
            if (c is TrimmedCurve3D trimmed)
            {
                c = trimmed.BasisCurve;
            }

            var gtsmc = new MakeCurve(c, localFactors);
            if (!gtsmc.IsDone)
            {
                return false;
            }

            StepCurve pmsc = gtsmc.Value();

            // trim the curve
            double trim1 = ca.FirstParameter();
            double trim2 = ca.LastParameter();

            if (vFirst.IsNull && vLast.IsNull && global::Kerf.Engine.Foundation.Precision.IsInfinite(first) && global::Kerf.Engine.Foundation.Precision.IsInfinite(last))
            {
                var curveMaker = new MakeCurve(c, localFactors);
                if (curveMaker.IsDone)
                {
                    sgc = curveMaker.Value();
                    curveList.Append(sgc);
                    return true;
                }

                return false;
            }

            if (vFirst.IsNull)
            {
                var gtsmcp = new MakeCartesianPoint(ca.Value(first), localFactors.LengthFactor());
                sgcp1 = gtsmcp.Value();
            }

            if (vLast.IsNull)
            {
                var gtsmcp = new MakeCartesianPoint(ca.Value(last), localFactors.LengthFactor());
                sgcp2 = gtsmcp.Value();
            }

            sgc = MakeTrimmedCurve(pmsc, sgcp1, sgcp2, trim1, trim2, true);
        }
        else
        {
            // -------------------------
            // a 3D Curve is constructed
            // -------------------------
            bool iPlan = false;
            if (!face.IsNull)
            {
                Curve2D c2d = Tool.CurveOnSurface(anEdge, face, out double cf, out double cl);
                Surface s = Tool.Surface(face);
                if (s is PlaneSurface && c2d is LineCurve2D)
                {
                    iPlan = true;
                }
            }

            // to be modified : cf and cl are the topological trimming parameter
            // these are computed after ! (U1 and U2) -> cf and cl instead
            if (iPlan)
            {
                Point3D pnt1 = ca.Value(ca.FirstParameter()), pnt2 = ca.Value(ca.LastParameter());
                Vector3D v = new Vector3D(pnt1, pnt2);
                double length = v.Magnitude;
                if (length >= global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    var l = new LineCurve3D(pnt1, new Direction3D(v));
                    var gtsml = new GeometryMakeLine(l, localFactors);
                    sgc = gtsml.Value();
                    sgc = MakeTrimmedCurve(gtsml.Value(), sgcp1, sgcp2, 0, length, true);
                }
                // OCCT_DEBUG output left out
            }
            else
            {
                var points = new NCollection_Array1<Point3D>(1, Nbpt);
                var knots = new NCollection_Array1<double>(1, Nbpt);
                var mult = new NCollection_Array1<int>(1, Nbpt);
                double u1 = ca.FirstParameter();
                double u2 = ca.LastParameter();
                for (int i = 1; i <= Nbpt; i++)
                {
                    double u = u1 + (i - 1) * (u2 - u1) / (Nbpt - 1);
                    Point3D p = ca.Value(u);
                    points.SetValue(i, p);
                    knots.SetValue(i, u);
                    mult.SetValue(i, 1);
                }

                points.SetValue(1, Tool.Pnt(vFirst));
                points.SetValue(Nbpt, Tool.Pnt(vLast));
                mult.SetValue(1, 2);
                mult.SetValue(Nbpt, 2);
                Curve3D bsCurve = new BSplineCurve3D(points, knots, mult, 1);
                var gtsmc = new MakeCurve(bsCurve, localFactors);
                sgc = gtsmc.Value();
            }
        }

        if (sgc is null)
        {
            return false;
        }

        map[anEdge] = sgc;
        curveList.Append(sgc);
        return true;
    }

    /// <summary>Extracts the trimmed curves of a face's edges, into the list.</summary>
    public bool GetTrimmedCurveFromFace(in Face aFace, Dictionary<Shape, Transient> aMap,
                                        ref NCollection_HSequence<Transient> aCurveList, in Factors localFactors)
    {
        Shape curShape;
        Edge curEdge;
        var exp = new Explorer();
        bool result = false; // szv#4:S4163:12Mar99 `done` hid one from this, initialisation needed

        for (exp.Init(aFace, ShapeType.Edge); exp.More(); exp.Next())
        {
            curShape = exp.Current();
            curEdge = ShapeCasts.Edge(curShape);
            if (GetTrimmedCurveFromEdge(curEdge, aFace, aMap, ref aCurveList, localFactors))
            {
                result = true;
            }
        }

        return result;
    }

    /// <summary>Extracts the trimmed curves of a shape's edges, into the list.</summary>
    public bool GetTrimmedCurveFromShape(in Shape aShape, Dictionary<Shape, Transient> aMap,
                                         ref NCollection_HSequence<Transient> aCurveList, in Factors localFactors)
    {
        var it = new Iterator();
        bool result = false; // szv#4:S4163:12Mar99 `done` hid one from this, initialisation needed

        // szv#4:S4163:12Mar99 optimized
        switch (aShape.Type)
        {
            case ShapeType.Edge:
            {
                Edge curEdge = ShapeCasts.Edge(aShape);
                Face nulFace = new Face();
                result = GetTrimmedCurveFromEdge(curEdge, nulFace, aMap, ref aCurveList, localFactors);
                break;
            }
            case ShapeType.Wire:
            {
                Face nulFace = new Face();
                Shape curShape;
                Edge curEdge;
                var exp = new Explorer();

                for (exp.Init(aShape, ShapeType.Edge); exp.More(); exp.Next())
                {
                    curShape = exp.Current();
                    curEdge = ShapeCasts.Edge(curShape);
                    if (GetTrimmedCurveFromEdge(curEdge, nulFace, aMap, ref aCurveList, localFactors))
                    {
                        result = true;
                    }
                }

                break;
            }
            case ShapeType.Face:
            {
                Face curFace = ShapeCasts.Face(aShape);
                result = GetTrimmedCurveFromFace(curFace, aMap, ref aCurveList, localFactors);
                break;
            }
            case ShapeType.Shell:
            {
                Shell sh = ShapeCasts.Shell(aShape);
                it.Initialize(sh);
                for (; it.More(); it.Next())
                {
                    Face curFace = ShapeCasts.Face(it.Value());
                    if (GetTrimmedCurveFromFace(curFace, aMap, ref aCurveList, localFactors))
                    {
                        result = true;
                    }
                }

                break;
            }

            case ShapeType.Solid:
            {
                it.Initialize(aShape);
                for (; it.More(); it.Next())
                {
                    if (it.Value().Type == ShapeType.Shell)
                    {
                        if (GetTrimmedCurveFromShape(it.Value(), aMap, ref aCurveList, localFactors))
                        {
                            result = true;
                        }
                    }
                }

                break;
            }

            case ShapeType.Compound:
            {
                it.Initialize(aShape);
                for (; it.More(); it.Next())
                {
                    if (GetTrimmedCurveFromShape(it.Value(), aMap, ref aCurveList, localFactors))
                    {
                        result = true;
                    }
                }

                break;
            }
            default:
                break;
        }

        return result;
    }
}
