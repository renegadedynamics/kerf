// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_ShapeLaw.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_ShapeLaw.lxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_ShapeLaw.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.Continuity;
using static Kerf.Topology.Orientation;

namespace Occt;

/// <summary>
/// A section law built on a vertex or a wire: constant (a GeomFill_UniformSection per edge) or scaled along the path by
/// a law function (a GeomFill_EvolvedSection per edge).
/// </summary>
/// <remarks>
/// The section's edges are the caller's, read only: their curves are reversed into new curves and trimmed into new
/// trimmed curves, as in OCCT. <see cref="D0"/> gives the caller's wire itself for a constant law and a frozen scaled
/// copy otherwise; <see cref="Vertex"/> the caller's vertex or a frozen scaled copy (decision 901).
/// </remarks>
public class BRepFill_ShapeLaw : BRepFill_SectionLaw
{
    protected bool vertex;
    private readonly Shape myShape;
    private NCollection_HArray1<Shape>? myEdges;
    private readonly Law_Function? TheLaw;

    //=======================================================================
    // function : Create
    // purpose  : Process the case of Vertex by constructing a line
    //           with the vertex in the origin
    //=======================================================================

    /// <summary>A constant law on the vertex <paramref name="V"/> (a tiny segment along X through it).</summary>
    public BRepFill_ShapeLaw(in Vertex V, bool Build = true)
    {
        vertex = true;
        myShape = V;
        TheLaw = null;
        uclosed = false;
        vclosed = true; // constant law
        myEdges = new NCollection_HArray1<Shape>(1, 1);
        myEdges.SetValue(1, V);

        if (Build)
        {
            myLaws = new NCollection_HArray1<GeomFill_SectionLaw>(1, 1);
            //    gp_Pnt Origine;
            Direction3D D = new(Direction3D.D.X); // Following the normal
            LineCurve3D L = new(Tool.Pnt(V), D);
            double Last = 2 * Tool.Tolerance(V) + global::Kerf.Engine.Foundation.Precision.PConfusion();
            TrimmedCurve3D TC = new(L, 0, Last);

            myLaws.ChangeValue(1) = new GeomFill_UniformSection(TC);
        }
        myDone = true;
    }

    //=================================================================================================

    /// <summary>A constant law on the wire <paramref name="W"/>.</summary>
    public BRepFill_ShapeLaw(in Wire W, bool Build = true)
    {
        vertex = false;
        myShape = W;
        TheLaw = null;
        Init(Build);
        myDone = true;
    }

    //=================================================================================================

    /// <summary>A law on the wire <paramref name="W"/> scaled about the origin by <paramref name="L"/>.</summary>
    public BRepFill_ShapeLaw(in Wire W, Law_Function L, bool Build = true)
    {
        vertex = false;
        myShape = W;
        TheLaw = L;
        Init(Build);
        myDone = true;
    }

    //=======================================================================
    // function : Init
    // purpose  : Case of the wire : Create a table of GeomFill_SectionLaw
    //=======================================================================
    private void Init(bool Build)
    {
        vclosed = true;
        WireExplorer wexp = new();
        Edge E;
        int NbEdge, ii;
        double First, Last;
        Wire W;
        W = ShapeCasts.Wire(myShape);

        for (NbEdge = 0, wexp.Init(W); wexp.More(); wexp.Next())
        {
            E = wexp.Current();
            if (!E.IsNull && !Tool.Degenerated(E))
            {
                Curve3D? C = Tool.Curve(E, out First, out Last);
                if (C is not null)
                {
                    NbEdge++;
                }
            }
        }

        myLaws = new NCollection_HArray1<GeomFill_SectionLaw>(1, NbEdge);
        myEdges = new NCollection_HArray1<Shape>(1, NbEdge);

        ii = 1;

        for (wexp.Init(W); wexp.More(); wexp.Next())
        {
            E = wexp.Current();
            if (!E.IsNull && !Tool.Degenerated(wexp.Current()))
            {
                Curve3D? C = Tool.Curve(E, out First, out Last);
                if (C is not null)
                {
                    myEdges.SetValue(ii, E);
                    myIndices.Bind(E, ii);
                    if (Build)
                    {
                        // occ::handle<Geom_Curve> C = BRep_Tool::Curve(E,First,Last);
                        if (E.Orientation() == Reversed)
                        {
                            double aux;
                            Curve3D CBis;
                            CBis = C.Reversed(); // To avoid the deterioration of the topology
                            aux = C.ReversedParameter(First);
                            First = C.ReversedParameter(Last);
                            Last = aux;
                            C = CBis;
                        }

                        bool IsClosed = Tool.IsClosed(E);
                        if (IsClosed && Math.Abs(C.FirstParameter() - First) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            IsClosed = false; // trimmed curve differs
                        }

                        if ((ii > 1) || !IsClosed)
                        { // Trim C
                            TrimmedCurve3D TC = new(C, First, Last);
                            C = TC;
                        }
                        // otherwise preserve the integrity of the curve
                        if (TheLaw is null)
                        {
                            myLaws.ChangeValue(ii) = new GeomFill_UniformSection(C);
                        }
                        else
                        {
                            myLaws.ChangeValue(ii) = new GeomFill_EvolvedSection(C, TheLaw);
                        }
                    }
                    ii++;
                }
            }
        }

        //  std::cout << "new law" << std::endl;

        //  Is the law closed by U ?
        uclosed = W.Closed();
        if (!uclosed)
        {
            // if not sure about the flag, make check
            Edge Edge1, Edge2;
            Vertex V1, V2;
            Edge1 = ShapeCasts.Edge(myEdges.Value(myEdges.Length()));
            Edge2 = ShapeCasts.Edge(myEdges.Value(1));

            if (Edge1.Orientation() == Reversed)
            {
                V1 = Exploration.FirstVertex(Edge1);
            }
            else
            {
                V1 = Exploration.LastVertex(Edge1);
            }

            if (Edge2.Orientation() == Reversed)
            {
                V2 = Exploration.LastVertex(Edge2);
            }
            else
            {
                V2 = Exploration.FirstVertex(Edge2);
            }
            if (V1.IsSame(V2))
            {
                uclosed = true;
            }
            else
            {
                EdgeCurveAdaptor Curve1 = new(Edge1);
                EdgeCurveAdaptor Curve2 = new(Edge2);
                double U1 = Tool.Parameter(V1, Edge1);
                double U2 = Tool.Parameter(V2, Edge2);
                double Eps = Tool.Tolerance(V2) + Tool.Tolerance(V1);

                uclosed = Curve1.Value(U1).IsEqual(Curve2.Value(U2), Eps);
            }
        }
    }

    //=================================================================================================

    /// <summary>True if the input shape is a vertex.</summary>
    public override bool IsVertex() => vertex;

    //=================================================================================================

    /// <summary>True if the law is constant (no law function).</summary>
    public override bool IsConstant() => TheLaw is null;

    //=================================================================================================

    /// <summary>
    /// The section's vertex of rank <paramref name="Index"/> (in [1, NbLaw() + 1]): the caller's vertex for a constant
    /// law, a frozen copy scaled by the law at <paramref name="Param"/> otherwise.
    /// </summary>
    public override Vertex Vertex(int Index, double Param)
    {
        Edge E;
        Vertex V = new();
        if (Index <= myEdges!.Length())
        {
            E = ShapeCasts.Edge(myEdges.Value(Index));
            if (E.Orientation() == Reversed)
            {
                V = Exploration.LastVertex(E);
            }
            else
            {
                V = Exploration.FirstVertex(E);
            }
        }
        else if (Index == myEdges.Length() + 1)
        {
            E = ShapeCasts.Edge(myEdges.Value(Index - 1));
            if (E.Orientation() == Reversed)
            {
                V = Exploration.FirstVertex(E);
            }
            else
            {
                V = Exploration.LastVertex(E);
            }
        }

        if (TheLaw is not null)
        {
            Transform3D T = new();
            T.SetScale(new Point3D(0, 0, 0), TheLaw.Value(Param));
            // TopLoc_Location L(T);
            // V.Move(L);
            V = ShapeCasts.Vertex(new BRepBuilderAPI_Transform(V, T).Shape());
        }
        return V;
    }

    //=======================================================================
    // function : VertexTol
    // purpose  : Evaluate the hole between 2 edges of the section
    //=======================================================================

    /// <summary>The gap between the end of the edge of rank <paramref name="Index"/> and the start of the next one.</summary>
    public override double VertexTol(int Index, double Param)
    {
        double Tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        int I1, I2;
        if ((Index == 0) || (Index == myEdges!.Length()))
        {
            if (!uclosed)
            {
                return Tol; // The least possible error
            }
            I1 = myEdges!.Length();
            I2 = 1;
        }
        else
        {
            I1 = Index;
            I2 = I1 + 1;
        }

        GeomFill_SectionLaw Loi;
        int NbPoles, NbKnots, Degree;
        NCollection_HArray1<Point3D> Poles;
        NCollection_HArray1<double> Knots, Weigth;
        NCollection_HArray1<int> Mults;
        BSplineCurve3D BS;
        Point3D PFirst;

        Loi = myLaws!.Value(I1);
        Loi.SectionShape(out NbPoles, out NbKnots, out Degree);
        Poles = new NCollection_HArray1<Point3D>(1, NbPoles);
        Weigth = new NCollection_HArray1<double>(1, NbPoles);
        Loi.D0(Param, Poles.ChangeArray1(), Weigth.ChangeArray1());
        Knots = new NCollection_HArray1<double>(1, NbKnots);
        Loi.Knots(Knots.ChangeArray1());
        Mults = new NCollection_HArray1<int>(1, NbKnots);
        Loi.Mults(Mults.ChangeArray1());
        BS = new BSplineCurve3D(Poles.Array1(),
                                   Weigth.Array1(),
                                   Knots.Array1(),
                                   Mults.Array1(),
                                   Degree,
                                   Loi.IsUPeriodic());
        PFirst = BS.Value(Knots.Value(Knots.Length()));

        Loi = myLaws.Value(I2);
        Loi.SectionShape(out NbPoles, out NbKnots, out Degree);
        Poles = new NCollection_HArray1<Point3D>(1, NbPoles);
        Weigth = new NCollection_HArray1<double>(1, NbPoles);
        Loi.D0(Param, Poles.ChangeArray1(), Weigth.ChangeArray1());
        Knots = new NCollection_HArray1<double>(1, NbKnots);
        Loi.Knots(Knots.ChangeArray1());
        Mults = new NCollection_HArray1<int>(1, NbKnots);
        Loi.Mults(Mults.ChangeArray1());
        BS = new BSplineCurve3D(Poles.Array1(),
                                   Weigth.Array1(),
                                   Knots.Array1(),
                                   Mults.Array1(),
                                   Degree,
                                   Loi.IsUPeriodic());
        Tol += PFirst.Distance(BS.Value(Knots.Value(1)));
        return Tol;
    }

    //=================================================================================================

    /// <summary>
    /// The law on the section's edges concatenated into one B-spline (the law's own elementary law when there is one
    /// edge; null for a vertex law of several edges, which can't happen).
    /// </summary>
    public override GeomFill_SectionLaw? ConcatenedLaw()
    {
        GeomFill_SectionLaw? Law = null;
        if (myLaws!.Length() == 1)
        {
            return myLaws.Value(1);
        }
        else
        {
            Wire W;
            Vertex V = new();
            W = ShapeCasts.Wire(myShape);
            if (!W.IsNull)
            {
                //  Concatenation of edges
                int ii;
                double epsV;
                bool Bof;
                Curve3D Composite;
                TrimmedCurve3D TC;
                Composite = Tool.Curve(Edge(1), out double f, out double l)!;
                TC = new TrimmedCurve3D(Composite, f, l);
                CompCurveToBSplineCurve Concat = new(TC);

                for (ii = 2, Bof = true; ii <= myEdges!.Length() && Bof; ii++)
                {
                    Composite = Tool.Curve(Edge(ii), out f, out l)!;
                    TC = new TrimmedCurve3D(Composite, f, l);
                    Bof = Exploration.CommonVertex(Edge(ii - 1), Edge(ii), ref V);
                    if (Bof)
                    {
                        epsV = Tool.Tolerance(V);
                    }
                    else
                    {
                        epsV = 10 * global::Kerf.Engine.Foundation.Precision.PConfusion();
                    }
                    Bof = Concat.Add(TC, epsV, true, false, 20);
                    if (!Bof)
                    {
                        Bof = Concat.Add(TC, 200 * epsV, true, false, 20);
                    }
                }
                Composite = Concat.InternalBSplineCurve()!;

                if (TheLaw is null)
                {
                    Law = new GeomFill_UniformSection(Composite);
                }
                else
                {
                    Law = new GeomFill_EvolvedSection(Composite, TheLaw);
                }
            }
        }
        return Law;
    }

    //=================================================================================================

    /// <summary>The continuity of the section at the vertex between its edges of ranks <paramref name="Index"/> and next.</summary>
    public override Continuity Continuity(int Index, double TolAngular)
    {

        Edge Edge1, Edge2;
        if ((Index == 0) || (Index == myEdges!.Length()))
        {
            if (!uclosed)
            {
                return C0; // The least possible error
            }

            Edge1 = ShapeCasts.Edge(myEdges!.Value(myEdges.Length()));
            Edge2 = ShapeCasts.Edge(myEdges.Value(1));
        }
        else
        {
            Edge1 = ShapeCasts.Edge(myEdges.Value(Index));
            Edge2 = ShapeCasts.Edge(myEdges.Value(Index + 1));
        }

        Vertex V1, V2; // common vertex
        Exploration.Vertices(Edge1, out Vertex vv1, out Vertex vv2);
        Exploration.Vertices(Edge2, out Vertex vv3, out Vertex vv4);
        if (vv1.IsSame(vv3))
        {
            V1 = vv1;
            V2 = vv3;
        }
        else if (vv1.IsSame(vv4))
        {
            V1 = vv1;
            V2 = vv4;
        }
        else if (vv2.IsSame(vv3))
        {
            V1 = vv2;
            V2 = vv3;
        }
        else
        {
            V1 = vv2;
            V2 = vv4;
        }

        double U1 = Tool.Parameter(V1, Edge1);
        double U2 = Tool.Parameter(V2, Edge2);
        EdgeCurveAdaptor Curve1 = new(Edge1);
        EdgeCurveAdaptor Curve2 = new(Edge2);
        double Eps = Tool.Tolerance(V2) + Tool.Tolerance(V1);
        Continuity cont;
        cont = global::Kerf.Engine.Topology.Properties.BRepLProp.Continuity(Curve1, Curve2, U1, U2, Eps, TolAngular);

        return cont;
    }

    //=================================================================================================

    /// <summary>
    /// The section at the law's parameter <paramref name="U"/>: the caller's shape for a constant law, a frozen copy
    /// scaled about the origin otherwise.
    /// </summary>
    public override void D0(double U, ref Shape S)
    {
        S = myShape;
        if (TheLaw is not null)
        {
            Transform3D T = new();
            T.SetScale(new Point3D(0, 0, 0), TheLaw.Value(U));
            // TopLoc_Location L(T);
            // S.Move(L);
            S = new BRepBuilderAPI_Transform(S, T).Shape();
        }
    }

    /// <summary>The section's edge of rank <paramref name="Index"/>.</summary>
    public Edge Edge(int Index) => ShapeCasts.Edge(myEdges!.Value(Index));
}
