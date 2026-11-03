// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_SectionPlacement.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_SectionPlacement.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Filling;

/// <summary>
/// The placement of a section on a location law: the transformation taking the section to the law's frame at the
/// point of the path nearest to it (or at a given vertex of the path), and that point's curvilinear abscissa.
/// </summary>
/// <remarks>
/// An algorithm (decision 837): the constructor computes, the accessors read. It reads the section's curves and calls
/// <c>SetLocation</c>/<c>Perform</c> on a GeomFill_SectionPlacement over the law's own elementary laws, which it
/// evaluates; <see cref="AbscissaOnPath"/> may cache the path's lengths in the law (laws are working objects).
/// </remarks>
public sealed class SectionPlacement
{
    private readonly LocationLaw _law;
    private readonly Shape _section;
    private Transform3D _trsf = new();
    private double _param;
    private int _index;

    //=================================================================================================

    private static double SearchParam(LocationLaw law, int ind, in Vertex theV)
    {
        double t;
        Edge e;
        e = law.Edge(ind);
        t = Tool.Parameter(theV, e);
        if (e.Orientation == Reversed)
        {
            double lf, ll;
            Tool.Curve(e, out double f, out double l);
            lf = law.Law(ind).GetCurve()!.FirstParameter();
            ll = law.Law(ind).GetCurve()!.LastParameter();
            t = ll - (t - f) * (ll - lf) / (l - f);
        }
        return t;
    }

    //=================================================================================================

    /// <summary>Automatic placement of <paramref name="section"/> on <paramref name="law"/>.</summary>
    /// <exception cref="ConstructionErrorException">The section's nearest point isn't on the path.</exception>
    public SectionPlacement(LocationLaw law, in Shape section, bool withContact = false,
                                     bool withCorrection = false)
    {
        _law = law;
        _section = section;
        Vertex vNull = new();
        vNull.Nullify();
        Perform(withContact, withCorrection, vNull);
    }

    //=================================================================================================

    /// <summary>Placement of <paramref name="section"/> on <paramref name="law"/> at the path's vertex <paramref name="vertex"/>.</summary>
    /// <exception cref="ConstructionErrorException">The section's nearest point isn't on the path.</exception>
    public SectionPlacement(LocationLaw law, in Shape section, in Shape vertex,
                                     bool withContact = false, bool withCorrection = false)
    {
        _law = law;
        _section = section;
        Perform(withContact, withCorrection, vertex);
    }

    //=================================================================================================

    private void Perform(bool withContact, bool withCorrection, in Shape vertex)
    {
        double edgeStartParam = 0.0;
        double edgeEndParam = 0.0;
        Curve3D? curve = null;

        // Here we are simply looking for the first valid curve in the section.
        Explorer edgeExplorer = new(_section, ShapeType.Edge);
        for (; edgeExplorer.More(); edgeExplorer.Next())
        {
            Edge edge = ShapeCasts.Edge(edgeExplorer.Current());
            if (edge.IsNull || Tool.Degenerated(edge))
            {
                continue;
            }

            curve = Tool.Curve(edge, out edgeStartParam, out edgeEndParam);
            if (curve is null)
            {
                continue;
            }

            break;
        }

        Geometry3D section;
        if (curve is null)
        {
            // No edge found : the section is a vertex
            Explorer vertexExplorer = new(_section, global::Kerf.Topology.ShapeType.Vertex);
            Vertex firstVertex = ShapeCasts.Vertex(vertexExplorer.Current());
            Point3D point = Tool.Pnt(firstVertex);
            section = new CartesianPoint(point);
        }
        else
        {
            TrimmedCurve3D trimmedCurve = new(curve, edgeStartParam, edgeEndParam);
            edgeExplorer.Next();

            if (edgeExplorer.More())
            {
                double precisionTolerance = global::Kerf.Engine.Foundation.Precision.Confusion();
                CompCurveToBSplineCurve bSplineConverter = new(trimmedCurve);
                for (; edgeExplorer.More(); edgeExplorer.Next())
                {
                    Edge edge = ShapeCasts.Edge(edgeExplorer.Current());
                    // avoid null, degenerated edges
                    if (edge.IsNull || Tool.Degenerated(edge))
                    {
                        continue;
                    }

                    curve = Tool.Curve(edge, out edgeStartParam, out edgeEndParam);
                    if (curve is null)
                    {
                        continue;
                    }

                    Exploration.Vertices(edge, out Vertex firstVertex, out Vertex lastVertex);
                    double vertexTolerance =
                        Max(Tool.Tolerance(firstVertex), Tool.Tolerance(lastVertex));

                    trimmedCurve = new TrimmedCurve3D(curve, edgeStartParam, edgeEndParam);
                    if (!bSplineConverter.Add(trimmedCurve, Min(precisionTolerance, vertexTolerance)))
                    {
                        bSplineConverter.Add(trimmedCurve, Max(precisionTolerance, vertexTolerance));
                    }
                }
                curve = bSplineConverter.InternalBSplineCurve();
            }
            else
            {
                curve = trimmedCurve;
            }

            section = curve!;
        }

        SurfaceSectionPlacement sectionPlacement = new(_law.Law(1), section);
        WireCurveAdaptor wireAdaptor = new(_law.Wire());
        sectionPlacement.Perform(wireAdaptor, global::Kerf.Engine.Foundation.Precision.Confusion());

        double sectionParam = sectionPlacement.ParameterOnPath();
        double paramConfusion = global::Kerf.Engine.Foundation.Precision.PConfusion();

        int lawIndex1 = 0;
        int lawIndex2 = 0;
        // In the general case : Localisation via concatenation of the spine
        bool isIntervalFound = false;
        for (int lawIndex = 1; lawIndex <= _law.LawCount && !isIntervalFound; ++lawIndex)
        {
            double currKnotParam = lawIndex - 1;
            double nextKnotParam = lawIndex;

            // Check if the section parameter is in the interval [aCurrKnotParam, aNextKnotParam]
            isIntervalFound = (currKnotParam - paramConfusion <= sectionParam)
                                && (nextKnotParam + paramConfusion >= sectionParam);
            if (!isIntervalFound)
            {
                continue;
            }

            lawIndex1 = lawIndex;
            if ((Math.Abs(sectionParam - currKnotParam) < paramConfusion) && (lawIndex > 1))
            {
                lawIndex2 = lawIndex - 1;
            }
            else if ((Math.Abs(sectionParam - nextKnotParam) < paramConfusion)
                     && (lawIndex < _law.LawCount))
            {
                lawIndex2 = lawIndex + 1;
            }
        }

        if (!isIntervalFound)
        {
            throw new ConstructionErrorException("Interval is not found");
        }

        // Search of the <Ind1> by vertex <TheV>
        bool isVertexOnLaw = false;
        Vertex aVertex = ShapeCasts.Vertex(vertex);
        if (!aVertex.IsNull)
        {
            for (int currentLawIndex = 1; currentLawIndex <= _law.LawCount; ++currentLawIndex)
            {
                Edge edge = _law.Edge(currentLawIndex);
                Exploration.Vertices(edge, out Vertex v1, out Vertex v2);
                if (v1.IsSame(aVertex) || v2.IsSame(aVertex))
                {
                    isVertexOnLaw = true;
                    lawIndex1 = currentLawIndex;
                    lawIndex2 = 0;
                    break;
                }
            }
        }
        ////////////////////

        // Positioning on the localized edge (or 2 Edges)

        sectionPlacement.SetLocation(_law.Law(lawIndex1));
        if (!isVertexOnLaw)
        {
            sectionPlacement.Perform(global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        else
        {
            sectionPlacement.Perform(SearchParam(_law, lawIndex1, aVertex), global::Kerf.Engine.Foundation.Precision.Confusion());
        }

        _trsf = sectionPlacement.Transformation(withContact, withCorrection);
        _index = lawIndex1;
        _param = sectionPlacement.ParameterOnPath();
        double angle = sectionPlacement.Angle();

        if (lawIndex2 != 0)
        {
            sectionPlacement.SetLocation(_law.Law(lawIndex2));
            if (!isVertexOnLaw)
            {
                sectionPlacement.Perform(global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            else
            {
                if (lawIndex1 == lawIndex2)
                {
                    aVertex.Reverse();
                }
                sectionPlacement.Perform(SearchParam(_law, lawIndex2, aVertex), global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            if (sectionPlacement.Angle() > angle)
            {
                _trsf = sectionPlacement.Transformation(withContact, withCorrection);
                _index = lawIndex2;
                _param = sectionPlacement.ParameterOnPath();
            }
        }
    }

    //=================================================================================================

    /// <summary>The transformation placing the section on the law.</summary>
    public Transform3D Transformation() => _trsf;

    //=================================================================================================

    /// <summary>The curvilinear abscissa on the path of the section's placement.</summary>
    public double AbscissaOnPath() => _law.Abscissa(_index, _param);
}
