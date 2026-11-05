// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Generator.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Generator.lxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Generator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Collections;
using Kerf.Engine.Construction.Library;
using Kerf.Engine.Construction.Surfaces;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using Kerf.Engine;
using static Kerf.Engine.Construction.Filling.ThruSectionErrorStatus;
using static Kerf.Geometry.CurveKind;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Construction.Filling;

/// <summary>
/// Computes a ruled shell between consecutive wires (sections) with the same number of edges: one ruled face between
/// each pair of corresponding edges; planes, cylinders and cones where two edges allow them. A first or last section
/// may be a point (a wire of degenerated edges).
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). As in OCCT, the faces are bounded by the sections' own edges: by default
/// (<see cref="IsMutableInput"/> true) the algorithm gives them pcurves on the new faces, ranges, flags and tolerances
/// in place. <see cref="Perform"/> runs in a working overlay (unit 10H3): the writes into the caller's frozen wires go
/// to working copies, and the shell holds the caller's sub-shapes wherever the construction left them unchanged; the
/// history answers for the caller's shapes directly. Inside an operation's own overlay (ThruSections',
/// BRepProj_Projection's) the writes are that operation's, as in OCCT.
/// </para>
/// <para>
/// <see cref="Shell"/>, <see cref="Generated"/>, <see cref="GeneratedShapes"/> and <see cref="ResultShape"/> hand out
/// frozen shapes; ported callers use the <c>Internal</c> forms.
/// </para>
/// </remarks>
internal sealed class Generator
{
    private readonly NCollection_Sequence<Shape> _wires = new();
    private Shell _shell = new();
    private readonly Dictionary<Shape, NCollection_List<Shape>> _map = new(ShapeHasher.Instance);
    private readonly Dictionary<Shape, Shape> _oldNewShapes = new(ShapeHasher.Instance);
    private readonly ReShape _reshaper = new();
    private bool _mutableInput;
    private ThruSectionErrorStatus _status;
    private WorkingOverlay? _overlay; // Kerf: the overlay the sections were written in, finished (unit 10H3)

    //=================================================================================================

    /// <summary>
    /// The kind of surface two edges allow: 0 none (a ruled surface), 1 a cylinder, 2 a cone, -2 a cone whose apex is
    /// the first (degenerated) edge, 4 a plane; -1 when an edge has no 3D curve.
    /// </summary>
    internal static int DetectKPart(in Edge edge1, in Edge edge2)
    {
        // initializations
        // !Note if IType set as -1 it means that occurs error with null 3d curve for the edge
        int iType = 0;

        // characteristics of the first edge
        double first1 = 0.0, last1 = 0.0, first2, last2, ff, ll;
        Placement loc;
        Vertex v1, v2;
        Curve3D? curv1, curv;
        CurveAdaptor adC1 = new();
        bool degen1 = Tool.Degenerated(edge1);

        // find the particular case
        Point3D pos1 = new(), pos;
        double dist;
        double dist1 = 0.0;
        Axis3D axe1 = new(), axe;

        if (degen1)
        {
            iType = -2;
            Exploration.Vertices(edge1, out v1, out v2);
            pos1 = Tool.Pnt(v1);
        }
        else
        {
            curv1 = Tool.Curve(edge1, out loc, out first1, out last1);
            if (curv1 is null)
            {
                return -1;
            }
            curv1 = (Curve3D)curv1.Transformed(loc.Transformation);
            ff = first1;
            ll = last1;
            if (edge1.Orientation == Reversed)
            {
                curv1.Reverse();
                first1 = curv1.ReversedParameter(ll);
                last1 = curv1.ReversedParameter(ff);
            }
            adC1.Load(curv1);
            if (adC1.GetType() == Circle)
            {
                // first circular section
                iType = 1;
                pos1 = adC1.Circle().Location;
                dist1 = adC1.Circle().Radius;
                axe1 = adC1.Circle().Axis;
            }
            else if (adC1.GetType() == Line)
            {
                // first straight line section
                iType = 4;
                pos1 = adC1.Line().Location;
                dist1 = adC1.Value(first1).DistanceTo(adC1.Value(last1));
                Vector3D vec = new(adC1.Value(first1), adC1.Value(last1));
                Direction3D dir = new(vec);
                axe1 = new Axis3D(adC1.Value(first1), dir);
            }
            else
            {
                // first section of any type
                iType = 0;
            }
        }

        if (iType != 0)
        {

            bool degen2 = Tool.Degenerated(edge2);
            if (degen2)
            {
                Exploration.Vertices(edge2, out v1, out v2);
                pos = Tool.Pnt(v1);
                if (iType == 1)
                {
                    // the only particular case with degenerated edge at end : the cone
                    if (pos.IsEqual(pos1, global::Kerf.Engine.Foundation.Precision.Confusion()))
                    {
                        // the top is mixed with the center of the circle
                        iType = 0;
                    }
                    else
                    {
                        Vector3D vec = new(pos1, pos);
                        Direction3D dir = new(vec);
                        axe = new Axis3D(pos1, dir);
                        if (axe.IsParallel(axe1, global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            // the top is on the axis of the circle
                            iType = 2;
                        }
                        else
                        {
                            // incorrect top --> no particular case
                            iType = 0;
                        }
                    }
                }
                else if (iType != 4)
                { // not a plane
                    // no particular case
                    iType = 0;
                }
            }
            else
            {
                curv = Tool.Curve(edge2, out loc, out first2, out last2);
                if (curv is null)
                {
                    return -1;
                }
                curv = (Curve3D)curv.Transformed(loc.Transformation);
                ff = first2;
                ll = last2;
                if (edge2.Orientation == Reversed)
                {
                    curv.Reverse();
                    first2 = curv.ReversedParameter(ll);
                    last2 = curv.ReversedParameter(ff);
                }
                CurveAdaptor adC = new(curv);

                if (iType > 0 && iType < 4)
                {
                    if (adC.GetType() != Circle)
                    {
                        // section not circular --> no particular case
                        iType = 0;
                    }
                    else
                    {
                        if (adC.Circle().Axis.IsCoaxial(axe1, global::Kerf.Engine.Foundation.Precision.Angular(), global::Kerf.Engine.Foundation.Precision.Confusion()))
                        {
                            // same axis
                            if (Math.Abs(adC.Circle().Radius - dist1) < global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                // possibility of cylinder or a piece of cylinder
                                double h1 = Math.Abs(last1 - first1), h2 = Math.Abs(last2 - first2);
                                bool same, sameParametricLength = (Math.Abs(h1 - h2) < global::Kerf.Engine.Foundation.Precision.PConfusion());
                                double m1 = (first1 + last1) / 2.0, m2 = (first2 + last2) / 2.0;
                                adC1.D1(m1, out Point3D p1, out Vector3D du);
                                Point3D p2 = adC.Value(m2);
                                same = sameParametricLength && (new Vector3D(p1, p2).IsNormal(du, global::Kerf.Engine.Foundation.Precision.Angular()));
                                if (same)
                                {
                                    // cylinder or piece of cylinder
                                    iType = 1;
                                }
                                else
                                {
                                    // the interval of definition is not correct
                                    iType = 0;
                                }
                            }
                            else
                            {
                                // possibility of cone truncation
                                double h1 = Math.Abs(last1 - first1), h2 = Math.Abs(last2 - first2);
                                bool same, sameParametricLength = (Math.Abs(h1 - h2) < global::Kerf.Engine.Foundation.Precision.PConfusion());
                                double m1 = (first1 + last1) / 2.0, m2 = (first2 + last2) / 2.0;
                                adC1.D1(m1, out Point3D p1, out Vector3D du);
                                Point3D p2 = adC.Value(m2);
                                same = sameParametricLength && (new Vector3D(p1, p2).IsNormal(du, global::Kerf.Engine.Foundation.Precision.Angular()));
                                if (same)
                                {
                                    // truncation of cone
                                    iType = 2;
                                }
                                else
                                {
                                    // the interval of definition is not correct
                                    iType = 0;
                                }
                            }
                            if (adC.Circle().Location.IsEqual(pos1, global::Kerf.Engine.Foundation.Precision.Confusion()))
                            {
                                // the centers are mixed
                                iType = 0;
                            }
                        }
                        else
                        {
                            // different axis
                            if (adC.Circle().Radius == dist1)
                            {
                                // torus ?
                                iType = 3;
                            }
                            else
                            {
                                // different radius --> no particular case
                                iType = 0;
                            }
                        }
                    }
                }
                else if (iType >= 4)
                {
                    if (adC.GetType() != Line)
                    {
                        // not a straight line section --> no particular case
                        iType = 0;
                    }
                    else
                    {
                        pos = adC.Line().Location;
                        dist = adC.Value(first2).DistanceTo(adC.Value(last2));
                        Vector3D vec = new(adC.Value(first2), adC.Value(last2));
                        Direction3D aDir = new(vec);
                        axe = new Axis3D(adC.Value(first2), aDir);
                        if (axe.IsParallel(axe1, global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            // parallel straight line
                            if (Math.Abs(dist - dist1) < global::Kerf.Engine.Foundation.Precision.Confusion())
                            {
                                Direction3D dir = new(new Vector3D(adC1.Value(first1), adC.Value(first2)));
                                if (dir.IsNormal(new Direction3D(vec), global::Kerf.Engine.Foundation.Precision.Angular()))
                                {
                                    // plane
                                    iType = 4;
                                }
                                else
                                {
                                    // extrusion ?
                                    iType = 5;
                                }
                            }
                            else
                            {
                                // different length --> no particular case
                                iType = 0;
                            }
                        }
                        else
                        {
                            // not parallel straight line --> no particular case
                            iType = 0;
                        }
                    }
                }
                else if (iType == -2)
                {
                    if (adC.GetType() == Line)
                    {
                        iType = 4; // plane
                    }
                    else if (adC.GetType() == Circle)
                    {
                        // the only particular case with degenerated edge at the beginning the cone
                        pos = adC.Circle().Location;
                        axe = adC.Circle().Axis;
                        if (pos1.IsEqual(pos, global::Kerf.Engine.Foundation.Precision.Confusion()))
                        {
                            // the top is mixed with the center of the circle
                            iType = 0;
                        }
                        else
                        {
                            Vector3D vec = new(pos1, pos);
                            Direction3D dir = new(vec);
                            axe1 = new Axis3D(pos1, dir);
                            if (axe.IsParallel(axe1, global::Kerf.Engine.Foundation.Precision.Angular()))
                            {
                                // the top is on the axis of the circle
                                iType = -2;
                            }
                            else
                            {
                                // incorrect top --> no particular case
                                iType = 0;
                            }
                        }
                    }
                    else
                    {
                        iType = 0;
                    }
                }
            }
        }
        // torus and extrusion are not particular cases.
        if (iType == 3 || iType == 5)
        {
            iType = 0;
        }
        return iType;
    }

    //=======================================================================
    // function : CreateKPart
    // purpose  : Returns true if there is no errors occur
    //=======================================================================

    /// <summary>The surface of the particular case <paramref name="iType"/> between two edges (null for none).</summary>
    internal static bool CreateKPart(in Edge edge1,
                                     in Edge edge2,
                                     int iType,
                                     out Surface? surf)
    {
        surf = null;
        // find the dimension
        Placement loc;
        double a1 = 0.0, b1 = 0.0, aa = 0.0, bb = 0.0;
        Vertex v1f, v1l, v2f, v2l;

        bool isDone = true;

        // find characteristics of the first edge
        Curve3D? C1 = null;
        bool degen1 = Tool.Degenerated(edge1);
        if (degen1)
        {
            // cone with degenerated edge at the top
            Exploration.Vertices(edge1, out v1f, out v1l);
        }
        else
        {
            C1 = Tool.Curve(edge1, out loc, out a1, out b1);
            if (C1 is null)
            {
                return false;
            }
            C1 = (Curve3D)C1.Transformed(loc.Transformation);
            aa = a1;
            bb = b1;
            if (edge1.Orientation == Reversed)
            {
                C1.Reverse();
                aa = C1.ReversedParameter(b1);
                bb = C1.ReversedParameter(a1);
                Exploration.Vertices(edge1, out v1l, out v1f);
            }
            else
            {
                Exploration.Vertices(edge1, out v1f, out v1l);
            }
        }

        // find characteristics of the second edge
        Curve3D? C2 = null;
        bool degen2 = Tool.Degenerated(edge2);
        if (degen2)
        {
            // cone with degenerated edge at the top
            Exploration.Vertices(edge2, out v2f, out v2l);
        }
        else
        {
            C2 = Tool.Curve(edge2, out loc, out a1, out b1);
            if (C2 is null)
            {
                return false;
            }
            C2 = (Curve3D)C2.Transformed(loc.Transformation);
            if (edge2.Orientation == Reversed)
            {
                C2.Reverse();
                if (degen1)
                {
                    aa = a1;
                    bb = b1;
                }
                Exploration.Vertices(edge2, out v2l, out v2f);
            }
            else
            {
                if (degen1)
                {
                    aa = a1; // C2->ReversedParameter(b1);
                    bb = b1; // C2->ReversedParameter(a1);
                }
                Exploration.Vertices(edge2, out v2f, out v2l);
            }
        }

        // create the new surface (OCCT's unused face, wires and edges left out)
        CurveAdaptor c1Adaptor = new();
        if (C1 is not null)
        {
            c1Adaptor.Load(C1);
        }
        CurveAdaptor c2Adaptor = new();
        if (C2 is not null)
        {
            c2Adaptor.Load(C2);
        }

        // calculate the surface
        Surface? surface = null;
        double v, rad;
        if (iType == 1)
        {
            // cylindrical surface
            Circle3D c1 = c1Adaptor.Circle();
            Circle3D c2 = c2Adaptor.Circle();
            CoordinateSystem ac1 = new(c1.Position);
            v = new Vector3D(c1.Location, c2.Location).Dot(new Vector3D(ac1.Direction));
            if (v < 0.0)
            {
                ac1 = ac1.ZReversed();
                v = -v;
            }
            CylindricalSurface cyl = new(ac1, c1.Radius);
            surface = new TrimmedSurface(cyl, aa, bb, Min(0.0, v), Max(0.0, v));
        }
        else if (iType == 2)
        {
            // conical surface
            Circle3D k1 = c1Adaptor.Circle();
            CoordinateSystem ak1 = new(k1.Position);
            if (degen2)
            {
                v = new Vector3D(k1.Location, Tool.Pnt(v2f)).Dot(new Vector3D(ak1.Direction));
                rad = -k1.Radius;
            }
            else
            {
                Circle3D k2 = c2Adaptor.Circle();
                v = new Vector3D(k1.Location, k2.Location).Dot(new Vector3D(ak1.Direction));
                rad = k2.Radius - k1.Radius;
            }

            if (v < 0.0)
            {
                ak1 = ak1.ZReversed();
                v = -v;
            }
            double ang = Math.Atan(rad / v);
            ConicalSurface cone = new(ak1, ang, k1.Radius);
            v /= Math.Cos(ang);
            surface = new TrimmedSurface(cone, aa, bb, Min(0.0, v), Max(0.0, v));
        }
        else if (iType == -2)
        {
            // conical surface with the top at the beginning (degen1 is true)
            Circle3D k2 = c2Adaptor.Circle();
            CoordinateSystem ak2 = new(k2.Position);
            ak2 = ak2.WithLocation(Tool.Pnt(v1f));
            v = new Vector3D(Tool.Pnt(v1f), k2.Location).Dot(new Vector3D(ak2.Direction));
            rad = k2.Radius; // - k2.Radius();
            if (v < 0.0)
            {
                ak2 = ak2.ZReversed();
                v = -v;
            }
            double ang = Math.Atan(rad / v);
            ConicalSurface cone = new(ak2, ang, 0.0);
            v /= Math.Cos(ang);
            surface = new TrimmedSurface(cone, aa, bb, Min(0.0, v), Max(0.0, v));
        }
        else if (iType == 3)
        {
            // torus surface ?
        }
        else if (iType == 4)
        {
            // surface plane
            Line3D l1 = new(), l2 = new(), line = new();
            if (!degen1)
            {
                l1 = c1Adaptor.Line();
                line = l1;
            }
            if (!degen2)
            {
                l2 = c2Adaptor.Line();
                line = l2;
            }

            Point3D p1 = (degen1) ? Tool.Pnt(v1f) : l1.Location;
            Point3D p2 = (degen2) ? Tool.Pnt(v2f) : l2.Location;

            Vector3D p1P2 = new(p1, p2);
            Direction3D d1 = line.Direction;
            CoordinateSystem ax = new(line.Location, d1.Crossed(new Direction3D(p1P2)), d1);
            PlaneSurface plan = new(ax);
            v = p1P2.Dot(new Vector3D(ax.YDirection));
            surface = plan;
            // surface = new Geom_RectangularTrimmedSurface
            //( Plan, aa, bb, std::min(0.,V), std::max(0.,V) );
        }
        else if (iType == 5)
        {
            // surface of extrusion ?
        }
        else
        {
            // IType incorrect
        }
        surf = surface;
        return isDone;
    }

    //=================================================================================================

    private static Edge CreateNewEdge(in Edge edge,
                                             Dictionary<Shape, Shape> copiedEdges,
                                             in Wire wire,
                                             IndexedSet<Shape> modifWires)
    {
        BrepBuilder builder = new();
        Edge newEdge;
        newEdge = ShapeCasts.Edge(edge.EmptyCopied());
        Iterator iterator = new(edge);
        for (; iterator.More(); iterator.Next())
        {
            builder.Add(newEdge, iterator.Value());
        }
        copiedEdges[edge] = newEdge;

        if (!modifWires.Contains(wire))
        {
            modifWires.Add(wire);
        }
        return newEdge;
    }

    //=================================================================================================

    /// <summary>An empty generator (mutable input).</summary>
    public Generator()
    {
        _mutableInput = true;
        _status = ThruSectionErrorStatus.ThruSectionErrorStatus_NotDone;
    }

    //=================================================================================================

    /// <summary>Adds a section: a wire, or a point as a wire of degenerated edges (first or last only).</summary>
    public void AddWire(in Wire wire)
    {
        _wires.Append(wire);
    }

    //=================================================================================================

    /// <summary>Computes the shell.</summary>
    public void Perform()
    {
        // Kerf: the sections a caller holds are written into: in a working overlay (unit 10H3), unless the generator
        // runs inside an operation's own (ThruSections'), whose writes these are and which reads them back
        _overlay = null;
        WorkingOverlay? overlay = WorkingOverlay.BeginUnlessWriting(new Shape());
        try
        {
            PerformCore();
        }
        finally
        {
            if (overlay is not null)
            {
                // the shell holds the caller's sub-shapes the construction left unchanged
                _shell = ShapeCasts.Shell(overlay.Finish(_shell));
                _overlay = overlay;
                overlay.Dispose();
            }
        }
    }

    private void PerformCore()
    {
        _status = ThruSectionErrorStatus.ThruSectionErrorStatus_Done;

        Face face = new();
        Edge edge1, edge2, edge3 = new(), edge4 = new();

        BrepBuilder b = new();
        b.MakeShell(ref _shell);

        int nb = _wires.Length();
        IndexedSet<Shape> modifWires = new(ShapeHasher.Instance); // indexed map for debugging

        WireExplorer ex1 = new(), ex2 = new();

        bool firstWire = true;
        bool wPoint1, wPoint2, uClosed = false, degenFirst = false, degenLast = false;

        for (int i = 1; i <= nb - 1; i++)
        {

            Wire wire1 = ShapeCasts.Wire(_wires.Value(i));
            Wire wire2 = ShapeCasts.Wire(_wires.Value(i + 1));

            wPoint1 = false;
            if (i == 1)
            {
                wPoint1 = true;
                for (ex1.Init(wire1); ex1.More(); ex1.Next())
                {
                    wPoint1 = wPoint1 && (Tool.Degenerated(ex1.Current()));
                }
                degenFirst = wPoint1;

                Exploration.Vertices(wire1, out Vertex v1, out Vertex v2);
                uClosed = v1.IsSame(v2);
            }

            wPoint2 = false;
            if (i == nb - 1)
            {
                wPoint2 = true;
                for (ex2.Init(wire2); ex2.More(); ex2.Next())
                {
                    wPoint2 = wPoint2 && (Tool.Degenerated(ex2.Current()));
                }
                degenLast = wPoint2;
            }

            ex1.Init(wire1);
            ex2.Init(wire2);

            Dictionary<Shape, Shape> map = new(ShapeHasher.Instance);

            bool tantque = ex1.More() && ex2.More();

            while (tantque)
            {
                Vertex v1f, v1l, v2f, v2l, Vf_toMap, Vl_toMap;
                Edge orEdge1 = ShapeCasts.Edge(ex1.Current());
                Edge orEdge2 = ShapeCasts.Edge(ex2.Current());

                bool degen1 = Tool.Degenerated(orEdge1);
                bool degen2 = Tool.Degenerated(orEdge2);

                if (degen1)
                {
                    Shape localShape = orEdge1.EmptyCopied();
                    edge1 = ShapeCasts.Edge(localShape);
                    Exploration.Vertices(orEdge1, out v1f, out v1l);
                    v1f.Orient(Forward);
                    b.Add(edge1, v1f);
                    v1l.Orient(Reversed);
                    b.Add(edge1, v1l);
                    b.Range(edge1, 0, 1);
                    _oldNewShapes[orEdge1] = edge1;
                }
                else
                {
                    edge1 = orEdge1;
                }

                if (degen2)
                {
                    Shape localShape = orEdge2.EmptyCopied();
                    edge2 = ShapeCasts.Edge(localShape);
                    Exploration.Vertices(orEdge2, out v2f, out v2l);
                    v2f.Orient(Forward);
                    b.Add(edge2, v2f);
                    v2l.Orient(Reversed);
                    b.Add(edge2, v2l);
                    b.Range(edge2, 0, 1);
                    _oldNewShapes[orEdge2] = edge2;
                }
                else
                {
                    edge2 = orEdge2;
                }

                bool periodic =
                  (Tool.IsClosed(edge1) || degen1) && (Tool.IsClosed(edge2) || degen2);
                // ATTENTION : a non-punctual wire should not
                //             contain a punctual edge
                if (!wPoint1)
                {
                    ex1.Next();
                    // OCCT: anOrEdge1 is a reference to the explorer's current edge, so from here on it is the next
                    // edge of the wire (a null edge after the last): myMap below is keyed by it (occt-issues.md)
                    orEdge1 = ShapeCasts.Edge(ex1.Current());
                }
                if (!wPoint2)
                {
                    ex2.Next();
                    orEdge2 = ShapeCasts.Edge(ex2.Current());
                }

                // initialization of vertices
                Surface? surf;
                double f1 = 0, l1 = 1, f2 = 0, l2 = 1;
                if (edge1.Orientation == Reversed)
                {
                    Exploration.Vertices(edge1, out v1l, out v1f);
                }
                else
                {
                    Exploration.Vertices(edge1, out v1f, out v1l);
                }
                if (edge2.Orientation == Reversed)
                {
                    Exploration.Vertices(edge2, out v2l, out v2f);
                }
                else
                {
                    Exploration.Vertices(edge2, out v2f, out v2l);
                }
                if (degen1)
                {
                    Vf_toMap = v2f;
                    Vl_toMap = v2l;
                }
                else
                {
                    Vf_toMap = v1f;
                    Vl_toMap = v1l;
                }

                // processing of KPart
                int iType = DetectKPart(edge1, edge2);
                if (iType == -1)
                {
                    _status = ThruSectionErrorStatus.ThruSectionErrorStatus_Null3DCurve;
                    return;
                }

                if (iType == 0)
                {
                    // no part cases
                    Curve3D? c1, c2;
                    NCollection_Array1<Point3D> extremities = new(1, 2);
                    Placement L1 = new(), L2 = new();

                    if (degen1)
                    {
                        extremities[1] = Tool.Pnt(v1f);
                        extremities[2] = Tool.Pnt(v1l);
                        c1 = new BezierCurve3D(extremities);
                    }
                    else
                    {
                        c1 = Tool.Curve(edge1, out L1, out f1, out l1);
                        if (c1 is null)
                        {
                            _status = ThruSectionErrorStatus.ThruSectionErrorStatus_Null3DCurve;
                            return;
                        }
                    }
                    if (degen2)
                    {
                        extremities[1] = Tool.Pnt(v2l);
                        extremities[2] = Tool.Pnt(v2f);
                        c2 = new BezierCurve3D(extremities);
                    }
                    else
                    {
                        c2 = Tool.Curve(edge2, out L2, out f2, out l2);
                        if (c2 is null)
                        {
                            _status = ThruSectionErrorStatus.ThruSectionErrorStatus_Null3DCurve;
                            return;
                        }
                    }

                    // compute the location
                    bool sameLoc = false;

                    // transform and trim the curves

                    if (Math.Abs(f1 - c1.FirstParameter) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                        || Math.Abs(l1 - c1.LastParameter) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        c1 = new TrimmedCurve3D(c1, f1, l1);
                    }
                    else
                    {
                        c1 = (Curve3D)c1.Copy();
                    }
                    if (!sameLoc)
                    {
                        c1.Transform(L1.Transformation);
                    }
                    if (edge1.Orientation == Reversed)
                    {
                        c1.Reverse();
                    }

                    if (Math.Abs(f2 - c2.FirstParameter) > global::Kerf.Engine.Foundation.Precision.PConfusion()
                        || Math.Abs(l2 - c2.LastParameter) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                    {
                        c2 = new TrimmedCurve3D(c2, f2, l2);
                    }
                    else
                    {
                        c2 = (Curve3D)c2.Copy();
                    }
                    if (!sameLoc)
                    {
                        c2.Transform(L2.Transformation);
                    }
                    if (edge2.Orientation == Reversed)
                    {
                        c2.Reverse();
                    }

                    SurfaceGenerator generator = new();
                    generator.AddCurve(c1);
                    generator.AddCurve(c2);
                    generator.Perform(global::Kerf.Engine.Foundation.Precision.PConfusion());

                    surf = generator.InternalSurface();
                    b.MakeFace(ref face, surf, global::Kerf.Engine.Foundation.Precision.Confusion());
                }
                else
                {
                    // particular case
                    if (!CreateKPart(edge1, edge2, iType, out surf))
                    {
                        _status = ThruSectionErrorStatus.ThruSectionErrorStatus_Null3DCurve;
                        return;
                    }
                    b.MakeFace(ref face, surf, global::Kerf.Engine.Foundation.Precision.Confusion());
                }

                // make the missing edges
                double first, last;
                surf!.ParameterBounds(out f1, out l1, out f2, out l2);
                if (iType == 0)
                {
                    first = f2;
                    last = l2;
                }
                else
                {
                    first = 0.0;
                    last = 1.0;
                }

                if (map.ContainsKey(Vf_toMap))
                {
                    Shape localShape = map.GetValue(Vf_toMap).Reversed();
                    edge3 = ShapeCasts.Edge(localShape);
                }
                else
                {
                    if (v1f.IsSame(v2f))
                    {
                        b.MakeEdge(ref edge3);
                        b.Degenerated(edge3, true);
                    }
                    else
                    {
                        Curve3D cc;
                        NCollection_Array1<Point3D> extremities = new(1, 2);
                        if (iType == 0)
                        {
                            // general case : Edge3 corresponds to iso U=f1
                            cc = surf.UIso(f1);
                        }
                        else
                        {
                            // particular case : it is required to calculate the curve 3d
                            extremities[1] = Tool.Pnt(v1f);
                            extremities[2] = Tool.Pnt(v2f);
                            cc = new BezierCurve3D(extremities);
                        }
                        b.MakeEdge(ref edge3, cc, global::Kerf.Engine.Foundation.Precision.Confusion());
                    }
                    v1f.Orient(Forward);
                    b.Add(edge3, v1f);
                    v2f.Orient(Reversed);
                    b.Add(edge3, v2f);
                    b.Range(edge3, first, last);
                    edge3.Reverse();
                    map[Vf_toMap] = edge3;
                }

                bool commonEdge = false;
                if (map.ContainsKey(Vl_toMap))
                {
                    Shape localShape = map.GetValue(Vl_toMap).Reversed();
                    Edge commonE = ShapeCasts.Edge(localShape);
                    Exploration.Vertices(commonE, out Vertex v1, out Vertex v2);
                    commonEdge = v1.IsSame(v1l) && v2.IsSame(v2l);
                }
                if (commonEdge)
                {
                    Shape localShape = map.GetValue(Vl_toMap).Reversed();
                    edge4 = ShapeCasts.Edge(localShape);
                }
                else
                {
                    if (v1l.IsSame(v2l))
                    {
                        b.MakeEdge(ref edge4);
                        b.Degenerated(edge4, true);
                    }
                    else
                    {
                        Curve3D cc;
                        NCollection_Array1<Point3D> extremities = new(1, 2);
                        if (iType == 0)
                        {
                            // general case : Edge4 corresponds to iso U=l1
                            cc = surf.UIso(l1);
                        }
                        else
                        {
                            // particular case : it is required to calculate the curve 3d
                            extremities[1] = Tool.Pnt(v1l);
                            extremities[2] = Tool.Pnt(v2l);
                            cc = new BezierCurve3D(extremities);
                        }
                        b.MakeEdge(ref edge4, cc, global::Kerf.Engine.Foundation.Precision.Confusion());
                    }
                    v1l.Orient(Forward);
                    b.Add(edge4, v1l);
                    v2l.Orient(Reversed);
                    b.Add(edge4, v2l);
                    b.Range(edge4, first, last);
                    map[Vl_toMap] = edge4;
                }

                if (!_mutableInput)
                {
                    if (!degen1) // if true=>already empty-copied
                    {
                        if (_oldNewShapes.TryGetValue(edge1, out Shape newEd1))
                        {
                            edge1 = ShapeCasts.Edge(newEd1);
                        }
                        else if (firstWire
                                 && (iType != 4 || Tool.SameParameter(edge1)
                                     || Tool.SameRange(edge1)))
                        {
                            // if such expression is true and mutableInput is false => pre-copy the edge to prevent
                            // a following modifying (see code below)
                            edge1 = CreateNewEdge(edge1, _oldNewShapes, wire1, modifWires);
                        }
                    }
                    if (!degen2)
                    {
                        if (_oldNewShapes.TryGetValue(edge2, out Shape newEd2))
                        {
                            edge2 = ShapeCasts.Edge(newEd2);
                        }
                        else if (iType != 4 || Tool.SameParameter(edge2) || Tool.SameRange(edge2))
                        {
                            edge2 = CreateNewEdge(edge2, _oldNewShapes, wire2, modifWires);
                        }
                    }
                }

                // set the pcurves
                double t = global::Kerf.Engine.Foundation.Precision.Confusion();

                if (iType != 4) // not plane
                {
                    if (edge1.Orientation == Reversed)
                    {
                        b.UpdateEdge(edge1, new LineCurve2D(new Point2D(0, f2), new Direction2D(Direction2D.D.NX)), face, t);
                        b.Range(edge1, face, -l1, -f1);
                    }
                    else
                    {
                        b.UpdateEdge(edge1, new LineCurve2D(new Point2D(0, f2), new Direction2D(Direction2D.D.X)), face, t);
                        b.Range(edge1, face, f1, l1);
                    }

                    if (edge2.Orientation == Reversed)
                    {
                        b.UpdateEdge(edge2, new LineCurve2D(new Point2D(0, l2), new Direction2D(Direction2D.D.NX)), face, t);
                        b.Range(edge2, face, -l1, -f1);
                    }
                    else
                    {
                        b.UpdateEdge(edge2, new LineCurve2D(new Point2D(0, l2), new Direction2D(Direction2D.D.X)), face, t);
                        b.Range(edge2, face, f1, l1);
                    }
                }

                if (iType == 0)
                {
                    if (periodic)
                    {
                        b.UpdateEdge(edge3,
                                     new LineCurve2D(new Point2D(l1, 0), new Direction2D(Direction2D.D.Y)),
                                     new LineCurve2D(new Point2D(f1, 0), new Direction2D(Direction2D.D.Y)),
                                     face,
                                     t);
                    }
                    else
                    {
                        b.UpdateEdge(edge3, new LineCurve2D(new Point2D(f1, 0), new Direction2D(Direction2D.D.Y)), face, t);
                        b.UpdateEdge(edge4, new LineCurve2D(new Point2D(l1, 0), new Direction2D(Direction2D.D.Y)), face, t);
                    }
                }
                else
                {
                    // KPart
                    if (periodic)
                    {
                        NCollection_Array1<Point2D> extrem1 = new(1, 2);
                        extrem1[1] = extrem1[1].WithCoord(l1, f2);
                        extrem1[2] = extrem1[2].WithCoord(l1, l2);
                        NCollection_Array1<Point2D> extrem2 = new(1, 2);
                        extrem2[1] = extrem2[1].WithCoord(f1, f2);
                        extrem2[2] = extrem2[2].WithCoord(f1, l2);
                        b.UpdateEdge(edge3,
                                     new BezierCurve2D(extrem1),
                                     new BezierCurve2D(extrem2),
                                     face,
                                     t);
                    }
                    else if (iType != 4)
                    { // not plane
                        NCollection_Array1<Point2D> extrem2 = new(1, 2);
                        extrem2[1] = extrem2[1].WithCoord(f1, f2);
                        extrem2[2] = extrem2[2].WithCoord(f1, l2);
                        b.UpdateEdge(edge3, new BezierCurve2D(extrem2), face, t);
                        NCollection_Array1<Point2D> extrem1 = new(1, 2);
                        extrem1[1] = extrem1[1].WithCoord(l1, f2);
                        extrem1[2] = extrem1[2].WithCoord(l1, l2);
                        b.UpdateEdge(edge4, new BezierCurve2D(extrem1), face, t);
                    }
                }
                // Set the non parameter flag;
                b.SameParameter(edge1, false);
                b.SameParameter(edge2, false);
                b.SameParameter(edge3, false);
                b.SameParameter(edge4, false);
                b.SameRange(edge1, false);
                b.SameRange(edge2, false);
                b.SameRange(edge3, false);
                b.SameRange(edge4, false);

                // make the wire
                Wire wire = new();
                b.MakeWire(ref wire);

                if (!degen1 || iType != 4)
                {
                    b.Add(wire, edge1);
                }
                b.Add(wire, edge4);
                if (!degen2 || iType != 4)
                {
                    b.Add(wire, edge2.Reversed());
                }
                b.Add(wire, edge3);

                b.Add(face, wire);
                b.Add(_shell, face);

                // complete myMap for edge1
                if (!degen1 || iType != 4)
                {
                    Edge rEd = orEdge1;
                    if (degen1)
                    {
                        rEd = edge1;
                    }

                    if (!_map.ContainsKey(rEd))
                    {
                        NCollection_List<Shape> empty = new();
                        _map[rEd] = empty;
                    }
                    _map.GetValue(rEd).Append(face);
                }

                tantque = ex1.More() && ex2.More();
                if (wPoint1)
                {
                    tantque = ex2.More();
                }
                if (wPoint2)
                {
                    tantque = ex1.More();
                }
            }
            firstWire = false;
        }

        // all vertices from myShell are the part of orig. section wires
        if (_mutableInput)
        {
            BrepLibrary.SameParameter(_shell);
        }
        else
        {
            foreach (KeyValuePair<Shape, Shape> mapIt in _oldNewShapes)
            {
                Shape k = mapIt.Key;
                Shape val = mapIt.Value;
                _reshaper.Replace(k, val);
            }
            BrepLibrary.SameParameter(_shell, _reshaper);
            _shell = ShapeCasts.Shell(_reshaper.InternalApply(_shell));
        }

        if (uClosed && degenFirst && degenLast)
        {
            _shell.Closed(true);
        }

        // update wire's history
        Iterator itEdge = new();
        for (int i = 0; i < modifWires.Count; i++)
        {
            Shape curWire = modifWires[i];
            Wire newWire = new();
            b.MakeWire(ref newWire);

            itEdge.Initialize(curWire);
            for (; itEdge.More(); itEdge.Next())
            {
                Shape curEdge = itEdge.Value(); // edges only
                Shape newEdge = ResultShapeCore(curEdge);
                b.Add(newWire, newEdge);
            }

            newWire.Free(curWire.Free());
            newWire.Modified(curWire.Modified());
            newWire.Checked(curWire.Checked());
            newWire.Orientable(curWire.OrientableFlag);
            newWire.Closed(curWire.ClosedFlag);
            newWire.Infinite(curWire.InfiniteFlag);
            newWire.Convex(curWire.ConvexFlag);

            _oldNewShapes[curWire] = newWire;
        }
    }

    //=================================================================================================

    /// <summary>The shell (frozen).</summary>
    public Shell Shell()
    {
        _shell.TShape()?.FreezeGraph();
        return _shell;
    }

    /// <summary>The shell, not frozen, for ported callers.</summary>
    internal Shell InternalShell() => _shell;

    //=================================================================================================

    /// <summary>
    /// The faces created from the edge <paramref name="sSection"/> of a section, frozen (empty if none). As natively,
    /// each face is keyed by the edge that follows the one it was built on (see docs/occt-issues.md).
    /// </summary>
    public IReadOnlyList<Shape> GeneratedShapes(in Shape sSection) => Published(InternalGeneratedShapes(sSection));

    private static readonly NCollection_List<Shape> Empty = new();

    /// <summary>The algorithm's own list behind <see cref="GeneratedShapes"/>, for ported callers.</summary>
    internal NCollection_List<Shape> InternalGeneratedShapes(in Shape sSection)
    {
        if (_map.ContainsKey(sSection))
        {
            return _map.GetValue(sSection);
        }
        else
        {
            return Empty;
        }
    }

    /// <summary>All the shapes created, keyed by the sections' edges (frozen), in the order they were built.</summary>
    public IReadOnlyList<KeyValuePair<Shape, IReadOnlyList<Shape>>> Generated()
    {
        var pairs = new KeyValuePair<Shape, IReadOnlyList<Shape>>[_map.Count];
        int i = 0;
        foreach (KeyValuePair<Shape, NCollection_List<Shape>> it in _map)
        {
            Shape key = it.Key;
            key.TShape()?.FreezeGraph();
            pairs[i++] = new KeyValuePair<Shape, IReadOnlyList<Shape>>(key, Published(it.Value));
        }
        return ImmutableArrays.Own(pairs);
    }

    /// <summary>The algorithm's own map behind <see cref="Generated"/>.</summary>
    internal Dictionary<Shape, NCollection_List<Shape>> InternalGenerated() => _map;

    private static IReadOnlyList<Shape> Published(NCollection_List<Shape> list)
    {
        if (list.IsEmpty())
        {
            return Array.Empty<Shape>();
        }
        var shapes = new Shape[list.Extent()];
        int i = 0;
        for (NCollection_List<Shape>.Iterator it = new(list); it.More(); it.Next())
        {
            shapes[i] = it.Value();
            shapes[i].TShape()?.FreezeGraph();
            i++;
        }
        return ImmutableArrays.Own(shapes);
    }

    //=================================================================================================

    /// <summary>
    /// The shape standing for <paramref name="shape"/> (a section, or one of its sub-shapes) in the shell: itself
    /// when the operation didn't replace it. Frozen.
    /// </summary>
    public Shape ResultShape(in Shape shape)
    {
        Shape aShape = InternalResultShape(shape);
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary><see cref="ResultShape"/> without freezing, for ported callers.</summary>
    internal Shape InternalResultShape(in Shape shape)
    {
        Shape result = ResultShapeCore(shape);
        return _overlay is null ? result : _overlay.Final(result);
    }

    private Shape ResultShapeCore(in Shape shape)
    {
        Shape newShape = _oldNewShapes.TryGetValue(shape, out Shape aShape) ? aShape : shape;
        Shape prevShape;
        do
        {
            prevShape = newShape;
            newShape = _reshaper.InternalValue(newShape);
        } while (newShape != prevShape);
        return newShape;
    }

    //=================================================================================================

    /// <summary>
    /// Sets the mutable input state: when true (the default), the sections' edges are given the pcurves and ranges
    /// on the new faces in place (in a working overlay for a caller's frozen sections); when false, on copies of them.
    /// </summary>
    public void SetMutableInput(bool isMutableInput)
    {
        _mutableInput = isMutableInput;
    }

    /// <summary>The mutable input state.</summary>
    public bool IsMutableInput
    {
        get
        {
            return _mutableInput;
        }
    }

    /// <summary>The status of the operation.</summary>
    public ThruSectionErrorStatus GetStatus() => _status;
}
