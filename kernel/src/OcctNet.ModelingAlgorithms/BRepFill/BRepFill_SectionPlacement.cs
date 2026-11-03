// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_SectionPlacement.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_SectionPlacement.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Geometry.Objects;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// The placement of a section on a location law: the transformation taking the section to the law's frame at the
/// point of the path nearest to it (or at a given vertex of the path), and that point's curvilinear abscissa.
/// </summary>
/// <remarks>
/// An algorithm (decision 837): the constructor computes, the accessors read. It reads the section's curves and calls
/// <c>SetLocation</c>/<c>Perform</c> on a GeomFill_SectionPlacement over the law's own elementary laws, which it
/// evaluates; <see cref="AbscissaOnPath"/> may cache the path's lengths in the law (laws are working objects).
/// </remarks>
public sealed class BRepFill_SectionPlacement
{
    private readonly BRepFill_LocationLaw myLaw;
    private readonly Shape mySection;
    private Transform3D myTrsf = new();
    private double myParam;
    private int myIndex;

    //=================================================================================================

    private static double SearchParam(BRepFill_LocationLaw Law, int Ind, in Vertex TheV)
    {
        double t;
        Edge E;
        E = Law.Edge(Ind);
        t = Tool.Parameter(TheV, E);
        if (E.Orientation() == Reversed)
        {
            double Lf, Ll;
            Tool.Curve(E, out double f, out double l);
            Lf = Law.Law(Ind).GetCurve()!.FirstParameter();
            Ll = Law.Law(Ind).GetCurve()!.LastParameter();
            t = Ll - (t - f) * (Ll - Lf) / (l - f);
        }
        return t;
    }

    //=================================================================================================

    /// <summary>Automatic placement of <paramref name="Section"/> on <paramref name="Law"/>.</summary>
    /// <exception cref="ConstructionErrorException">The section's nearest point isn't on the path.</exception>
    public BRepFill_SectionPlacement(BRepFill_LocationLaw Law, in Shape Section, bool WithContact = false,
                                     bool WithCorrection = false)
    {
        myLaw = Law;
        mySection = Section;
        Vertex VNull = new();
        VNull.Nullify();
        Perform(WithContact, WithCorrection, VNull);
    }

    //=================================================================================================

    /// <summary>Placement of <paramref name="Section"/> on <paramref name="Law"/> at the path's vertex <paramref name="Vertex"/>.</summary>
    /// <exception cref="ConstructionErrorException">The section's nearest point isn't on the path.</exception>
    public BRepFill_SectionPlacement(BRepFill_LocationLaw Law, in Shape Section, in Shape Vertex,
                                     bool WithContact = false, bool WithCorrection = false)
    {
        myLaw = Law;
        mySection = Section;
        Perform(WithContact, WithCorrection, Vertex);
    }

    //=================================================================================================

    private void Perform(bool WithContact, bool WithCorrection, in Shape Vertex)
    {
        double anEdgeStartParam = 0.0;
        double anEdgeEndParam = 0.0;
        Curve3D? aCurve = null;

        // Here we are simply looking for the first valid curve in the section.
        Explorer anEdgeExplorer = new(mySection, ShapeType.Edge);
        for (; anEdgeExplorer.More(); anEdgeExplorer.Next())
        {
            Edge anEdge = ShapeCasts.Edge(anEdgeExplorer.Current());
            if (anEdge.IsNull || Tool.Degenerated(anEdge))
            {
                continue;
            }

            aCurve = Tool.Curve(anEdge, out anEdgeStartParam, out anEdgeEndParam);
            if (aCurve is null)
            {
                continue;
            }

            break;
        }

        Geometry3D aSection;
        if (aCurve is null)
        {
            // No edge found : the section is a vertex
            Explorer aVertexExplorer = new(mySection, global::Kerf.Topology.ShapeType.Vertex);
            Vertex aFirstVertex = ShapeCasts.Vertex(aVertexExplorer.Current());
            Point3D aPoint = Tool.Pnt(aFirstVertex);
            aSection = new CartesianPoint(aPoint);
        }
        else
        {
            TrimmedCurve3D aTrimmedCurve = new(aCurve, anEdgeStartParam, anEdgeEndParam);
            anEdgeExplorer.Next();

            if (anEdgeExplorer.More())
            {
                double aPrecisionTolerance = global::Kerf.Engine.Foundation.Precision.Confusion();
                CompCurveToBSplineCurve aBSplineConverter = new(aTrimmedCurve);
                for (; anEdgeExplorer.More(); anEdgeExplorer.Next())
                {
                    Edge anEdge = ShapeCasts.Edge(anEdgeExplorer.Current());
                    // avoid null, degenerated edges
                    if (anEdge.IsNull || Tool.Degenerated(anEdge))
                    {
                        continue;
                    }

                    aCurve = Tool.Curve(anEdge, out anEdgeStartParam, out anEdgeEndParam);
                    if (aCurve is null)
                    {
                        continue;
                    }

                    Exploration.Vertices(anEdge, out Vertex aFirstVertex, out Vertex aLastVertex);
                    double aVertexTolerance =
                        Max(Tool.Tolerance(aFirstVertex), Tool.Tolerance(aLastVertex));

                    aTrimmedCurve = new TrimmedCurve3D(aCurve, anEdgeStartParam, anEdgeEndParam);
                    if (!aBSplineConverter.Add(aTrimmedCurve, Min(aPrecisionTolerance, aVertexTolerance)))
                    {
                        aBSplineConverter.Add(aTrimmedCurve, Max(aPrecisionTolerance, aVertexTolerance));
                    }
                }
                aCurve = aBSplineConverter.InternalBSplineCurve();
            }
            else
            {
                aCurve = aTrimmedCurve;
            }

            aSection = aCurve!;
        }

        GeomFill_SectionPlacement aSectionPlacement = new(myLaw.Law(1), aSection);
        WireCurveAdaptor aWireAdaptor = new(myLaw.Wire());
        aSectionPlacement.Perform(aWireAdaptor, global::Kerf.Engine.Foundation.Precision.Confusion());

        double aSectionParam = aSectionPlacement.ParameterOnPath();
        double aParamConfusion = global::Kerf.Engine.Foundation.Precision.PConfusion();

        int aLawIndex1 = 0;
        int aLawIndex2 = 0;
        // In the general case : Localisation via concatenation of the spine
        bool anIsIntervalFound = false;
        for (int aLawIndex = 1; aLawIndex <= myLaw.NbLaw() && !anIsIntervalFound; ++aLawIndex)
        {
            double aCurrKnotParam = aLawIndex - 1;
            double aNextKnotParam = aLawIndex;

            // Check if the section parameter is in the interval [aCurrKnotParam, aNextKnotParam]
            anIsIntervalFound = (aCurrKnotParam - aParamConfusion <= aSectionParam)
                                && (aNextKnotParam + aParamConfusion >= aSectionParam);
            if (!anIsIntervalFound)
            {
                continue;
            }

            aLawIndex1 = aLawIndex;
            if ((Math.Abs(aSectionParam - aCurrKnotParam) < aParamConfusion) && (aLawIndex > 1))
            {
                aLawIndex2 = aLawIndex - 1;
            }
            else if ((Math.Abs(aSectionParam - aNextKnotParam) < aParamConfusion)
                     && (aLawIndex < myLaw.NbLaw()))
            {
                aLawIndex2 = aLawIndex + 1;
            }
        }

        if (!anIsIntervalFound)
        {
            throw new ConstructionErrorException("Interval is not found");
        }

        // Search of the <Ind1> by vertex <TheV>
        bool anIsVertexOnLaw = false;
        Vertex aVertex = ShapeCasts.Vertex(Vertex);
        if (!aVertex.IsNull)
        {
            for (int aCurrentLawIndex = 1; aCurrentLawIndex <= myLaw.NbLaw(); ++aCurrentLawIndex)
            {
                Edge anEdge = myLaw.Edge(aCurrentLawIndex);
                Exploration.Vertices(anEdge, out Vertex V1, out Vertex V2);
                if (V1.IsSame(aVertex) || V2.IsSame(aVertex))
                {
                    anIsVertexOnLaw = true;
                    aLawIndex1 = aCurrentLawIndex;
                    aLawIndex2 = 0;
                    break;
                }
            }
        }
        ////////////////////

        // Positioning on the localized edge (or 2 Edges)

        aSectionPlacement.SetLocation(myLaw.Law(aLawIndex1));
        if (!anIsVertexOnLaw)
        {
            aSectionPlacement.Perform(global::Kerf.Engine.Foundation.Precision.Confusion());
        }
        else
        {
            aSectionPlacement.Perform(SearchParam(myLaw, aLawIndex1, aVertex), global::Kerf.Engine.Foundation.Precision.Confusion());
        }

        myTrsf = aSectionPlacement.Transformation(WithContact, WithCorrection);
        myIndex = aLawIndex1;
        myParam = aSectionPlacement.ParameterOnPath();
        double Angle = aSectionPlacement.Angle();

        if (aLawIndex2 != 0)
        {
            aSectionPlacement.SetLocation(myLaw.Law(aLawIndex2));
            if (!anIsVertexOnLaw)
            {
                aSectionPlacement.Perform(global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            else
            {
                if (aLawIndex1 == aLawIndex2)
                {
                    aVertex.Reverse();
                }
                aSectionPlacement.Perform(SearchParam(myLaw, aLawIndex2, aVertex), global::Kerf.Engine.Foundation.Precision.Confusion());
            }
            if (aSectionPlacement.Angle() > Angle)
            {
                myTrsf = aSectionPlacement.Transformation(WithContact, WithCorrection);
                myIndex = aLawIndex2;
                myParam = aSectionPlacement.ParameterOnPath();
            }
        }
    }

    //=================================================================================================

    /// <summary>The transformation placing the section on the law.</summary>
    public Transform3D Transformation() => myTrsf;

    //=================================================================================================

    /// <summary>The curvilinear abscissa on the path of the section's placement.</summary>
    public double AbscissaOnPath() => myLaw.Abscissa(myIndex, myParam);
}
