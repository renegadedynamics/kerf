// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Generator.hxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Generator.lxx
// Port of: src/ModelingAlgorithms/TKBool/BRepFill/BRepFill_Generator.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1994-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Schematiq.Collections;
using static Occt.BRepFill_ThruSectionErrorStatus;
using static Occt.GeomAbs_CurveType;
using static Occt.TopAbs_Orientation;

namespace Occt;

/// <summary>
/// Computes a ruled shell between consecutive wires (sections) with the same number of edges: one ruled face between
/// each pair of corresponding edges; planes, cylinders and cones where two edges allow them. A first or last section
/// may be a point (a wire of degenerated edges).
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21). As in OCCT, the faces are bounded by the sections' own edges: by default
/// (<see cref="IsMutableInput"/> true) the algorithm gives them pcurves on the new faces, ranges, flags and tolerances
/// in place. The wires a caller gives (whose TShapes are frozen) are therefore copied at <see cref="Perform"/>, all at
/// once so that shared sub-shapes stay shared, and the caller's sub-shapes the construction left unchanged are given
/// back in the shell (decision 936); the history answers for the caller's shapes. Unfrozen wires (ported callers:
/// ThruSections, BRepProj_Projection) are used in place, as in OCCT.
/// </para>
/// <para>
/// <see cref="Shell"/>, <see cref="Generated"/>, <see cref="GeneratedShapes"/> and <see cref="ResultShape"/> hand out
/// frozen shapes; ported callers use the <c>Internal</c> forms.
/// </para>
/// </remarks>
public sealed class BRepFill_Generator
{
    private readonly NCollection_Sequence<TopoDS_Shape> myWires = new();
    private TopoDS_Shell myShell = new();
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myMap = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> myOldNewShapes = new(TopTools_ShapeMapHasher.Instance);
    private readonly BRepTools_ReShape myReshaper = new();
    private bool myMutableInput;
    private BRepFill_ThruSectionErrorStatus myStatus;
    private BRepTools_CopyAtEntry? myCopy; // OCCT.NET: the copy of frozen sections (decision 936)

    //=================================================================================================

    /// <summary>
    /// The kind of surface two edges allow: 0 none (a ruled surface), 1 a cylinder, 2 a cone, -2 a cone whose apex is
    /// the first (degenerated) edge, 4 a plane; -1 when an edge has no 3D curve.
    /// </summary>
    internal static int DetectKPart(in TopoDS_Edge Edge1, in TopoDS_Edge Edge2)
    {
        // initializations
        // !Note if IType set as -1 it means that occurs error with null 3d curve for the edge
        int IType = 0;

        // characteristics of the first edge
        double first1 = 0.0, last1 = 0.0, first2, last2, ff, ll;
        TopLoc_Location loc;
        TopoDS_Vertex V1, V2;
        Geom_Curve? curv1, curv;
        GeomAdaptor_Curve AdC1 = new();
        bool degen1 = BRep_Tool.Degenerated(Edge1);

        // find the particular case
        gp_Pnt pos1 = new(), pos;
        double dist;
        double dist1 = 0.0;
        gp_Ax1 axe1 = new(), axe;

        if (degen1)
        {
            IType = -2;
            TopExp.Vertices(Edge1, out V1, out V2);
            pos1 = BRep_Tool.Pnt(V1);
        }
        else
        {
            curv1 = BRep_Tool.Curve(Edge1, out loc, out first1, out last1);
            if (curv1 is null)
            {
                return -1;
            }
            curv1 = (Geom_Curve)curv1.Transformed(loc.Transformation());
            ff = first1;
            ll = last1;
            if (Edge1.Orientation() == TopAbs_REVERSED)
            {
                curv1.Reverse();
                first1 = curv1.ReversedParameter(ll);
                last1 = curv1.ReversedParameter(ff);
            }
            AdC1.Load(curv1);
            if (AdC1.GetType() == GeomAbs_Circle)
            {
                // first circular section
                IType = 1;
                pos1 = AdC1.Circle().Location();
                dist1 = AdC1.Circle().Radius();
                axe1 = AdC1.Circle().Axis();
            }
            else if (AdC1.GetType() == GeomAbs_Line)
            {
                // first straight line section
                IType = 4;
                pos1 = AdC1.Line().Location();
                dist1 = AdC1.Value(first1).Distance(AdC1.Value(last1));
                gp_Vec vec = new(AdC1.Value(first1), AdC1.Value(last1));
                gp_Dir dir = new(vec);
                axe1 = new gp_Ax1(AdC1.Value(first1), dir);
            }
            else
            {
                // first section of any type
                IType = 0;
            }
        }

        if (IType != 0)
        {

            bool degen2 = BRep_Tool.Degenerated(Edge2);
            if (degen2)
            {
                TopExp.Vertices(Edge2, out V1, out V2);
                pos = BRep_Tool.Pnt(V1);
                if (IType == 1)
                {
                    // the only particular case with degenerated edge at end : the cone
                    if (pos.IsEqual(pos1, Precision.Confusion()))
                    {
                        // the top is mixed with the center of the circle
                        IType = 0;
                    }
                    else
                    {
                        gp_Vec vec = new(pos1, pos);
                        gp_Dir dir = new(vec);
                        axe = new gp_Ax1(pos1, dir);
                        if (axe.IsParallel(axe1, Precision.Angular()))
                        {
                            // the top is on the axis of the circle
                            IType = 2;
                        }
                        else
                        {
                            // incorrect top --> no particular case
                            IType = 0;
                        }
                    }
                }
                else if (IType != 4)
                { // not a plane
                    // no particular case
                    IType = 0;
                }
            }
            else
            {
                curv = BRep_Tool.Curve(Edge2, out loc, out first2, out last2);
                if (curv is null)
                {
                    return -1;
                }
                curv = (Geom_Curve)curv.Transformed(loc.Transformation());
                ff = first2;
                ll = last2;
                if (Edge2.Orientation() == TopAbs_REVERSED)
                {
                    curv.Reverse();
                    first2 = curv.ReversedParameter(ll);
                    last2 = curv.ReversedParameter(ff);
                }
                GeomAdaptor_Curve AdC = new(curv);

                if (IType > 0 && IType < 4)
                {
                    if (AdC.GetType() != GeomAbs_Circle)
                    {
                        // section not circular --> no particular case
                        IType = 0;
                    }
                    else
                    {
                        if (AdC.Circle().Axis().IsCoaxial(axe1, Precision.Angular(), Precision.Confusion()))
                        {
                            // same axis
                            if (Math.Abs(AdC.Circle().Radius() - dist1) < Precision.Confusion())
                            {
                                // possibility of cylinder or a piece of cylinder
                                double h1 = Math.Abs(last1 - first1), h2 = Math.Abs(last2 - first2);
                                bool Same, SameParametricLength = (Math.Abs(h1 - h2) < Precision.PConfusion());
                                double m1 = (first1 + last1) / 2.0, m2 = (first2 + last2) / 2.0;
                                AdC1.D1(m1, out gp_Pnt P1, out gp_Vec DU);
                                gp_Pnt P2 = AdC.Value(m2);
                                Same = SameParametricLength && (new gp_Vec(P1, P2).IsNormal(DU, Precision.Angular()));
                                if (Same)
                                {
                                    // cylinder or piece of cylinder
                                    IType = 1;
                                }
                                else
                                {
                                    // the interval of definition is not correct
                                    IType = 0;
                                }
                            }
                            else
                            {
                                // possibility of cone truncation
                                double h1 = Math.Abs(last1 - first1), h2 = Math.Abs(last2 - first2);
                                bool Same, SameParametricLength = (Math.Abs(h1 - h2) < Precision.PConfusion());
                                double m1 = (first1 + last1) / 2.0, m2 = (first2 + last2) / 2.0;
                                AdC1.D1(m1, out gp_Pnt P1, out gp_Vec DU);
                                gp_Pnt P2 = AdC.Value(m2);
                                Same = SameParametricLength && (new gp_Vec(P1, P2).IsNormal(DU, Precision.Angular()));
                                if (Same)
                                {
                                    // truncation of cone
                                    IType = 2;
                                }
                                else
                                {
                                    // the interval of definition is not correct
                                    IType = 0;
                                }
                            }
                            if (AdC.Circle().Location().IsEqual(pos1, Precision.Confusion()))
                            {
                                // the centers are mixed
                                IType = 0;
                            }
                        }
                        else
                        {
                            // different axis
                            if (AdC.Circle().Radius() == dist1)
                            {
                                // torus ?
                                IType = 3;
                            }
                            else
                            {
                                // different radius --> no particular case
                                IType = 0;
                            }
                        }
                    }
                }
                else if (IType >= 4)
                {
                    if (AdC.GetType() != GeomAbs_Line)
                    {
                        // not a straight line section --> no particular case
                        IType = 0;
                    }
                    else
                    {
                        pos = AdC.Line().Location();
                        dist = AdC.Value(first2).Distance(AdC.Value(last2));
                        gp_Vec vec = new(AdC.Value(first2), AdC.Value(last2));
                        gp_Dir aDir = new(vec);
                        axe = new gp_Ax1(AdC.Value(first2), aDir);
                        if (axe.IsParallel(axe1, Precision.Angular()))
                        {
                            // parallel straight line
                            if (Math.Abs(dist - dist1) < Precision.Confusion())
                            {
                                gp_Dir dir = new(new gp_Vec(AdC1.Value(first1), AdC.Value(first2)));
                                if (dir.IsNormal(new gp_Dir(vec), Precision.Angular()))
                                {
                                    // plane
                                    IType = 4;
                                }
                                else
                                {
                                    // extrusion ?
                                    IType = 5;
                                }
                            }
                            else
                            {
                                // different length --> no particular case
                                IType = 0;
                            }
                        }
                        else
                        {
                            // not parallel straight line --> no particular case
                            IType = 0;
                        }
                    }
                }
                else if (IType == -2)
                {
                    if (AdC.GetType() == GeomAbs_Line)
                    {
                        IType = 4; // plane
                    }
                    else if (AdC.GetType() == GeomAbs_Circle)
                    {
                        // the only particular case with degenerated edge at the beginning the cone
                        pos = AdC.Circle().Location();
                        axe = AdC.Circle().Axis();
                        if (pos1.IsEqual(pos, Precision.Confusion()))
                        {
                            // the top is mixed with the center of the circle
                            IType = 0;
                        }
                        else
                        {
                            gp_Vec vec = new(pos1, pos);
                            gp_Dir dir = new(vec);
                            axe1 = new gp_Ax1(pos1, dir);
                            if (axe.IsParallel(axe1, Precision.Angular()))
                            {
                                // the top is on the axis of the circle
                                IType = -2;
                            }
                            else
                            {
                                // incorrect top --> no particular case
                                IType = 0;
                            }
                        }
                    }
                    else
                    {
                        IType = 0;
                    }
                }
            }
        }
        // torus and extrusion are not particular cases.
        if (IType == 3 || IType == 5)
        {
            IType = 0;
        }
        return IType;
    }

    //=======================================================================
    // function : CreateKPart
    // purpose  : Returns true if there is no errors occur
    //=======================================================================

    /// <summary>The surface of the particular case <paramref name="IType"/> between two edges (null for none).</summary>
    internal static bool CreateKPart(in TopoDS_Edge Edge1,
                                     in TopoDS_Edge Edge2,
                                     int IType,
                                     out Geom_Surface? Surf)
    {
        Surf = null;
        // find the dimension
        TopLoc_Location loc;
        double a1 = 0.0, b1 = 0.0, aa = 0.0, bb = 0.0;
        TopoDS_Vertex v1f, v1l, v2f, v2l;

        bool isDone = true;

        // find characteristics of the first edge
        Geom_Curve? C1 = null;
        bool degen1 = BRep_Tool.Degenerated(Edge1);
        if (degen1)
        {
            // cone with degenerated edge at the top
            TopExp.Vertices(Edge1, out v1f, out v1l);
        }
        else
        {
            C1 = BRep_Tool.Curve(Edge1, out loc, out a1, out b1);
            if (C1 is null)
            {
                return false;
            }
            C1 = (Geom_Curve)C1.Transformed(loc.Transformation());
            aa = a1;
            bb = b1;
            if (Edge1.Orientation() == TopAbs_REVERSED)
            {
                C1.Reverse();
                aa = C1.ReversedParameter(b1);
                bb = C1.ReversedParameter(a1);
                TopExp.Vertices(Edge1, out v1l, out v1f);
            }
            else
            {
                TopExp.Vertices(Edge1, out v1f, out v1l);
            }
        }

        // find characteristics of the second edge
        Geom_Curve? C2 = null;
        bool degen2 = BRep_Tool.Degenerated(Edge2);
        if (degen2)
        {
            // cone with degenerated edge at the top
            TopExp.Vertices(Edge2, out v2f, out v2l);
        }
        else
        {
            C2 = BRep_Tool.Curve(Edge2, out loc, out a1, out b1);
            if (C2 is null)
            {
                return false;
            }
            C2 = (Geom_Curve)C2.Transformed(loc.Transformation());
            if (Edge2.Orientation() == TopAbs_REVERSED)
            {
                C2.Reverse();
                if (degen1)
                {
                    aa = a1;
                    bb = b1;
                }
                TopExp.Vertices(Edge2, out v2l, out v2f);
            }
            else
            {
                if (degen1)
                {
                    aa = a1; // C2->ReversedParameter(b1);
                    bb = b1; // C2->ReversedParameter(a1);
                }
                TopExp.Vertices(Edge2, out v2f, out v2l);
            }
        }

        // create the new surface (OCCT's unused face, wires and edges left out)
        GeomAdaptor_Curve aC1Adaptor = new();
        if (C1 is not null)
        {
            aC1Adaptor.Load(C1);
        }
        GeomAdaptor_Curve aC2Adaptor = new();
        if (C2 is not null)
        {
            aC2Adaptor.Load(C2);
        }

        // calculate the surface
        Geom_Surface? surface = null;
        double V, Rad;
        if (IType == 1)
        {
            // cylindrical surface
            gp_Circ c1 = aC1Adaptor.Circle();
            gp_Circ c2 = aC2Adaptor.Circle();
            gp_Ax3 Ac1 = new(c1.Position());
            V = new gp_Vec(c1.Location(), c2.Location()).Dot(new gp_Vec(Ac1.Direction()));
            if (V < 0.0)
            {
                Ac1.ZReverse();
                V = -V;
            }
            Geom_CylindricalSurface Cyl = new(Ac1, c1.Radius());
            surface = new Geom_RectangularTrimmedSurface(Cyl, aa, bb, Min(0.0, V), Max(0.0, V));
        }
        else if (IType == 2)
        {
            // conical surface
            gp_Circ k1 = aC1Adaptor.Circle();
            gp_Ax3 Ak1 = new(k1.Position());
            if (degen2)
            {
                V = new gp_Vec(k1.Location(), BRep_Tool.Pnt(v2f)).Dot(new gp_Vec(Ak1.Direction()));
                Rad = -k1.Radius();
            }
            else
            {
                gp_Circ k2 = aC2Adaptor.Circle();
                V = new gp_Vec(k1.Location(), k2.Location()).Dot(new gp_Vec(Ak1.Direction()));
                Rad = k2.Radius() - k1.Radius();
            }

            if (V < 0.0)
            {
                Ak1.ZReverse();
                V = -V;
            }
            double Ang = Math.Atan(Rad / V);
            Geom_ConicalSurface Cone = new(Ak1, Ang, k1.Radius());
            V /= Math.Cos(Ang);
            surface = new Geom_RectangularTrimmedSurface(Cone, aa, bb, Min(0.0, V), Max(0.0, V));
        }
        else if (IType == -2)
        {
            // conical surface with the top at the beginning (degen1 is true)
            gp_Circ k2 = aC2Adaptor.Circle();
            gp_Ax3 Ak2 = new(k2.Position());
            Ak2.SetLocation(BRep_Tool.Pnt(v1f));
            V = new gp_Vec(BRep_Tool.Pnt(v1f), k2.Location()).Dot(new gp_Vec(Ak2.Direction()));
            Rad = k2.Radius(); // - k2.Radius();
            if (V < 0.0)
            {
                Ak2.ZReverse();
                V = -V;
            }
            double Ang = Math.Atan(Rad / V);
            Geom_ConicalSurface Cone = new(Ak2, Ang, 0.0);
            V /= Math.Cos(Ang);
            surface = new Geom_RectangularTrimmedSurface(Cone, aa, bb, Min(0.0, V), Max(0.0, V));
        }
        else if (IType == 3)
        {
            // torus surface ?
        }
        else if (IType == 4)
        {
            // surface plane
            gp_Lin L1 = new(), L2 = new(), aLine = new();
            if (!degen1)
            {
                L1 = aC1Adaptor.Line();
                aLine = L1;
            }
            if (!degen2)
            {
                L2 = aC2Adaptor.Line();
                aLine = L2;
            }

            gp_Pnt P1 = (degen1) ? BRep_Tool.Pnt(v1f) : L1.Location();
            gp_Pnt P2 = (degen2) ? BRep_Tool.Pnt(v2f) : L2.Location();

            gp_Vec P1P2 = new(P1, P2);
            gp_Dir D1 = aLine.Direction();
            gp_Ax3 Ax = new(aLine.Location(), D1.Crossed(new gp_Dir(P1P2)), D1);
            Geom_Plane Plan = new(Ax);
            V = P1P2.Dot(new gp_Vec(Ax.YDirection()));
            surface = Plan;
            // surface = new Geom_RectangularTrimmedSurface
            //( Plan, aa, bb, std::min(0.,V), std::max(0.,V) );
        }
        else if (IType == 5)
        {
            // surface of extrusion ?
        }
        else
        {
            // IType incorrect
        }
        Surf = surface;
        return isDone;
    }

    //=================================================================================================

    private static TopoDS_Edge CreateNewEdge(in TopoDS_Edge theEdge,
                                             NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> theCopiedEdges,
                                             in TopoDS_Wire theWire,
                                             NCollection_IndexedMap<TopoDS_Shape> theModifWires)
    {
        BRep_Builder aBuilder = new();
        TopoDS_Edge aNewEdge;
        aNewEdge = TopoDS.Edge(theEdge.EmptyCopied());
        TopoDS_Iterator anIterator = new(theEdge);
        for (; anIterator.More(); anIterator.Next())
        {
            aBuilder.Add(aNewEdge, anIterator.Value());
        }
        theCopiedEdges.Bind(theEdge, aNewEdge);

        if (!theModifWires.Contains(theWire))
        {
            theModifWires.Add(theWire);
        }
        return aNewEdge;
    }

    //=================================================================================================

    /// <summary>An empty generator (mutable input).</summary>
    public BRepFill_Generator()
    {
        myMutableInput = true;
        myStatus = BRepFill_ThruSectionErrorStatus_NotDone;
    }

    //=================================================================================================

    /// <summary>Adds a section: a wire, or a point as a wire of degenerated edges (first or last only).</summary>
    public void AddWire(in TopoDS_Wire Wire)
    {
        myWires.Append(Wire);
    }

    //=================================================================================================

    /// <summary>Computes the shell.</summary>
    public void Perform()
    {
        // OCCT.NET: the sections a caller holds are written into: copy them at once (decision 936)
        myCopy = null;
        bool isFrozen = false;
        for (int i = 1; i <= myWires.Length() && !isFrozen; i++)
        {
            isFrozen = BRepTools.HoldsFrozen(myWires.Value(i));
        }
        NCollection_Sequence<TopoDS_Shape> aCallerWires = myWires;
        if (isFrozen)
        {
            BRep_Builder aB = new();
            TopoDS_Compound aHolder = new();
            aB.MakeCompound(ref aHolder);
            for (int i = 1; i <= myWires.Length(); i++)
            {
                aB.Add(aHolder, myWires.Value(i));
            }
            myCopy = new BRepTools_CopyAtEntry(aHolder);
            aCallerWires = new NCollection_Sequence<TopoDS_Shape>(myWires);
            int k = 1;
            for (TopoDS_Iterator anIt = new(myCopy.Shape(), false, false); anIt.More(); anIt.Next(), k++)
            {
                myWires.ChangeValue(k) = anIt.Value();
            }
        }
        try
        {
            perform();
        }
        finally
        {
            if (myCopy is not null)
            {
                // the sections are the caller's again; the shell holds its unchanged sub-shapes
                myShell = TopoDS.Shell(myCopy.Finish(myShell));
                myWires.Assign(aCallerWires);
            }
        }
    }

    private void perform()
    {
        myStatus = BRepFill_ThruSectionErrorStatus_Done;

        TopoDS_Face Face = new();
        TopoDS_Edge Edge1, Edge2, Edge3 = new(), Edge4 = new();

        BRep_Builder B = new();
        B.MakeShell(ref myShell);

        int Nb = myWires.Length();
        NCollection_IndexedMap<TopoDS_Shape> aModifWires = new(TopTools_ShapeMapHasher.Instance); // indexed map for debugging

        BRepTools_WireExplorer ex1 = new(), ex2 = new();

        bool aFirstWire = true;
        bool wPoint1, wPoint2, uClosed = false, DegenFirst = false, DegenLast = false;

        for (int i = 1; i <= Nb - 1; i++)
        {

            TopoDS_Wire Wire1 = TopoDS.Wire(myWires.Value(i));
            TopoDS_Wire Wire2 = TopoDS.Wire(myWires.Value(i + 1));

            wPoint1 = false;
            if (i == 1)
            {
                wPoint1 = true;
                for (ex1.Init(Wire1); ex1.More(); ex1.Next())
                {
                    wPoint1 = wPoint1 && (BRep_Tool.Degenerated(ex1.Current()));
                }
                DegenFirst = wPoint1;

                TopExp.Vertices(Wire1, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                uClosed = V1.IsSame(V2);
            }

            wPoint2 = false;
            if (i == Nb - 1)
            {
                wPoint2 = true;
                for (ex2.Init(Wire2); ex2.More(); ex2.Next())
                {
                    wPoint2 = wPoint2 && (BRep_Tool.Degenerated(ex2.Current()));
                }
                DegenLast = wPoint2;
            }

            ex1.Init(Wire1);
            ex2.Init(Wire2);

            NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> Map = new(TopTools_ShapeMapHasher.Instance);

            bool tantque = ex1.More() && ex2.More();

            while (tantque)
            {
                TopoDS_Vertex V1f, V1l, V2f, V2l, Vf_toMap, Vl_toMap;
                TopoDS_Edge anOrEdge1 = TopoDS.Edge(ex1.Current());
                TopoDS_Edge anOrEdge2 = TopoDS.Edge(ex2.Current());

                bool degen1 = BRep_Tool.Degenerated(anOrEdge1);
                bool degen2 = BRep_Tool.Degenerated(anOrEdge2);

                if (degen1)
                {
                    TopoDS_Shape aLocalShape = anOrEdge1.EmptyCopied();
                    Edge1 = TopoDS.Edge(aLocalShape);
                    TopExp.Vertices(anOrEdge1, out V1f, out V1l);
                    V1f.Orientation(TopAbs_FORWARD);
                    B.Add(Edge1, V1f);
                    V1l.Orientation(TopAbs_REVERSED);
                    B.Add(Edge1, V1l);
                    B.Range(Edge1, 0, 1);
                    myOldNewShapes.Bind(anOrEdge1, Edge1);
                }
                else
                {
                    Edge1 = anOrEdge1;
                }

                if (degen2)
                {
                    TopoDS_Shape aLocalShape = anOrEdge2.EmptyCopied();
                    Edge2 = TopoDS.Edge(aLocalShape);
                    TopExp.Vertices(anOrEdge2, out V2f, out V2l);
                    V2f.Orientation(TopAbs_FORWARD);
                    B.Add(Edge2, V2f);
                    V2l.Orientation(TopAbs_REVERSED);
                    B.Add(Edge2, V2l);
                    B.Range(Edge2, 0, 1);
                    myOldNewShapes.Bind(anOrEdge2, Edge2);
                }
                else
                {
                    Edge2 = anOrEdge2;
                }

                bool Periodic =
                  (BRep_Tool.IsClosed(Edge1) || degen1) && (BRep_Tool.IsClosed(Edge2) || degen2);
                // ATTENTION : a non-punctual wire should not
                //             contain a punctual edge
                if (!wPoint1)
                {
                    ex1.Next();
                    // OCCT: anOrEdge1 is a reference to the explorer's current edge, so from here on it is the next
                    // edge of the wire (a null edge after the last): myMap below is keyed by it (occt-issues.md)
                    anOrEdge1 = TopoDS.Edge(ex1.Current());
                }
                if (!wPoint2)
                {
                    ex2.Next();
                    anOrEdge2 = TopoDS.Edge(ex2.Current());
                }

                // initialization of vertices
                Geom_Surface? Surf;
                double f1 = 0, l1 = 1, f2 = 0, l2 = 1;
                if (Edge1.Orientation() == TopAbs_REVERSED)
                {
                    TopExp.Vertices(Edge1, out V1l, out V1f);
                }
                else
                {
                    TopExp.Vertices(Edge1, out V1f, out V1l);
                }
                if (Edge2.Orientation() == TopAbs_REVERSED)
                {
                    TopExp.Vertices(Edge2, out V2l, out V2f);
                }
                else
                {
                    TopExp.Vertices(Edge2, out V2f, out V2l);
                }
                if (degen1)
                {
                    Vf_toMap = V2f;
                    Vl_toMap = V2l;
                }
                else
                {
                    Vf_toMap = V1f;
                    Vl_toMap = V1l;
                }

                // processing of KPart
                int IType = DetectKPart(Edge1, Edge2);
                if (IType == -1)
                {
                    myStatus = BRepFill_ThruSectionErrorStatus_Null3DCurve;
                    return;
                }

                if (IType == 0)
                {
                    // no part cases
                    Geom_Curve? C1, C2;
                    NCollection_Array1<gp_Pnt> Extremities = new(1, 2);
                    TopLoc_Location L1 = new(), L2 = new();

                    if (degen1)
                    {
                        Extremities[1] = BRep_Tool.Pnt(V1f);
                        Extremities[2] = BRep_Tool.Pnt(V1l);
                        C1 = new Geom_BezierCurve(Extremities);
                    }
                    else
                    {
                        C1 = BRep_Tool.Curve(Edge1, out L1, out f1, out l1);
                        if (C1 is null)
                        {
                            myStatus = BRepFill_ThruSectionErrorStatus_Null3DCurve;
                            return;
                        }
                    }
                    if (degen2)
                    {
                        Extremities[1] = BRep_Tool.Pnt(V2l);
                        Extremities[2] = BRep_Tool.Pnt(V2f);
                        C2 = new Geom_BezierCurve(Extremities);
                    }
                    else
                    {
                        C2 = BRep_Tool.Curve(Edge2, out L2, out f2, out l2);
                        if (C2 is null)
                        {
                            myStatus = BRepFill_ThruSectionErrorStatus_Null3DCurve;
                            return;
                        }
                    }

                    // compute the location
                    bool SameLoc = false;

                    // transform and trim the curves

                    if (Math.Abs(f1 - C1.FirstParameter()) > Precision.PConfusion()
                        || Math.Abs(l1 - C1.LastParameter()) > Precision.PConfusion())
                    {
                        C1 = new Geom_TrimmedCurve(C1, f1, l1);
                    }
                    else
                    {
                        C1 = (Geom_Curve)C1.Copy();
                    }
                    if (!SameLoc)
                    {
                        C1.Transform(L1.Transformation());
                    }
                    if (Edge1.Orientation() == TopAbs_REVERSED)
                    {
                        C1.Reverse();
                    }

                    if (Math.Abs(f2 - C2.FirstParameter()) > Precision.PConfusion()
                        || Math.Abs(l2 - C2.LastParameter()) > Precision.PConfusion())
                    {
                        C2 = new Geom_TrimmedCurve(C2, f2, l2);
                    }
                    else
                    {
                        C2 = (Geom_Curve)C2.Copy();
                    }
                    if (!SameLoc)
                    {
                        C2.Transform(L2.Transformation());
                    }
                    if (Edge2.Orientation() == TopAbs_REVERSED)
                    {
                        C2.Reverse();
                    }

                    GeomFill_Generator Generator = new();
                    Generator.AddCurve(C1);
                    Generator.AddCurve(C2);
                    Generator.Perform(Precision.PConfusion());

                    Surf = Generator.InternalSurface();
                    B.MakeFace(ref Face, Surf, Precision.Confusion());
                }
                else
                {
                    // particular case
                    if (!CreateKPart(Edge1, Edge2, IType, out Surf))
                    {
                        myStatus = BRepFill_ThruSectionErrorStatus_Null3DCurve;
                        return;
                    }
                    B.MakeFace(ref Face, Surf, Precision.Confusion());
                }

                // make the missing edges
                double first, last;
                Surf!.Bounds(out f1, out l1, out f2, out l2);
                if (IType == 0)
                {
                    first = f2;
                    last = l2;
                }
                else
                {
                    first = 0.0;
                    last = 1.0;
                }

                if (Map.IsBound(Vf_toMap))
                {
                    TopoDS_Shape aLocalShape = Map.Find(Vf_toMap).Reversed();
                    Edge3 = TopoDS.Edge(aLocalShape);
                }
                else
                {
                    if (V1f.IsSame(V2f))
                    {
                        B.MakeEdge(ref Edge3);
                        B.Degenerated(Edge3, true);
                    }
                    else
                    {
                        Geom_Curve CC;
                        NCollection_Array1<gp_Pnt> Extremities = new(1, 2);
                        if (IType == 0)
                        {
                            // general case : Edge3 corresponds to iso U=f1
                            CC = Surf.UIso(f1);
                        }
                        else
                        {
                            // particular case : it is required to calculate the curve 3d
                            Extremities[1] = BRep_Tool.Pnt(V1f);
                            Extremities[2] = BRep_Tool.Pnt(V2f);
                            CC = new Geom_BezierCurve(Extremities);
                        }
                        B.MakeEdge(ref Edge3, CC, Precision.Confusion());
                    }
                    V1f.Orientation(TopAbs_FORWARD);
                    B.Add(Edge3, V1f);
                    V2f.Orientation(TopAbs_REVERSED);
                    B.Add(Edge3, V2f);
                    B.Range(Edge3, first, last);
                    Edge3.Reverse();
                    Map.Bind(Vf_toMap, Edge3);
                }

                bool CommonEdge = false;
                if (Map.IsBound(Vl_toMap))
                {
                    TopoDS_Shape aLocalShape = Map.Find(Vl_toMap).Reversed();
                    TopoDS_Edge CommonE = TopoDS.Edge(aLocalShape);
                    TopExp.Vertices(CommonE, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                    CommonEdge = V1.IsSame(V1l) && V2.IsSame(V2l);
                }
                if (CommonEdge)
                {
                    TopoDS_Shape aLocalShape = Map.Find(Vl_toMap).Reversed();
                    Edge4 = TopoDS.Edge(aLocalShape);
                }
                else
                {
                    if (V1l.IsSame(V2l))
                    {
                        B.MakeEdge(ref Edge4);
                        B.Degenerated(Edge4, true);
                    }
                    else
                    {
                        Geom_Curve CC;
                        NCollection_Array1<gp_Pnt> Extremities = new(1, 2);
                        if (IType == 0)
                        {
                            // general case : Edge4 corresponds to iso U=l1
                            CC = Surf.UIso(l1);
                        }
                        else
                        {
                            // particular case : it is required to calculate the curve 3d
                            Extremities[1] = BRep_Tool.Pnt(V1l);
                            Extremities[2] = BRep_Tool.Pnt(V2l);
                            CC = new Geom_BezierCurve(Extremities);
                        }
                        B.MakeEdge(ref Edge4, CC, Precision.Confusion());
                    }
                    V1l.Orientation(TopAbs_FORWARD);
                    B.Add(Edge4, V1l);
                    V2l.Orientation(TopAbs_REVERSED);
                    B.Add(Edge4, V2l);
                    B.Range(Edge4, first, last);
                    Map.Bind(Vl_toMap, Edge4);
                }

                if (!myMutableInput)
                {
                    if (!degen1) // if true=>already empty-copied
                    {
                        if (myOldNewShapes.Find(Edge1, out TopoDS_Shape aNewEd1))
                        {
                            Edge1 = TopoDS.Edge(aNewEd1);
                        }
                        else if (aFirstWire
                                 && (IType != 4 || BRep_Tool.SameParameter(Edge1)
                                     || BRep_Tool.SameRange(Edge1)))
                        {
                            // if such expression is true and mutableInput is false => pre-copy the edge to prevent
                            // a following modifying (see code below)
                            Edge1 = CreateNewEdge(Edge1, myOldNewShapes, Wire1, aModifWires);
                        }
                    }
                    if (!degen2)
                    {
                        if (myOldNewShapes.Find(Edge2, out TopoDS_Shape aNewEd2))
                        {
                            Edge2 = TopoDS.Edge(aNewEd2);
                        }
                        else if (IType != 4 || BRep_Tool.SameParameter(Edge2) || BRep_Tool.SameRange(Edge2))
                        {
                            Edge2 = CreateNewEdge(Edge2, myOldNewShapes, Wire2, aModifWires);
                        }
                    }
                }

                // set the pcurves
                double T = Precision.Confusion();

                if (IType != 4) // not plane
                {
                    if (Edge1.Orientation() == TopAbs_REVERSED)
                    {
                        B.UpdateEdge(Edge1, new Geom2d_Line(new gp_Pnt2d(0, f2), new gp_Dir2d(gp_Dir2d.D.NX)), Face, T);
                        B.Range(Edge1, Face, -l1, -f1);
                    }
                    else
                    {
                        B.UpdateEdge(Edge1, new Geom2d_Line(new gp_Pnt2d(0, f2), new gp_Dir2d(gp_Dir2d.D.X)), Face, T);
                        B.Range(Edge1, Face, f1, l1);
                    }

                    if (Edge2.Orientation() == TopAbs_REVERSED)
                    {
                        B.UpdateEdge(Edge2, new Geom2d_Line(new gp_Pnt2d(0, l2), new gp_Dir2d(gp_Dir2d.D.NX)), Face, T);
                        B.Range(Edge2, Face, -l1, -f1);
                    }
                    else
                    {
                        B.UpdateEdge(Edge2, new Geom2d_Line(new gp_Pnt2d(0, l2), new gp_Dir2d(gp_Dir2d.D.X)), Face, T);
                        B.Range(Edge2, Face, f1, l1);
                    }
                }

                if (IType == 0)
                {
                    if (Periodic)
                    {
                        B.UpdateEdge(Edge3,
                                     new Geom2d_Line(new gp_Pnt2d(l1, 0), new gp_Dir2d(gp_Dir2d.D.Y)),
                                     new Geom2d_Line(new gp_Pnt2d(f1, 0), new gp_Dir2d(gp_Dir2d.D.Y)),
                                     Face,
                                     T);
                    }
                    else
                    {
                        B.UpdateEdge(Edge3, new Geom2d_Line(new gp_Pnt2d(f1, 0), new gp_Dir2d(gp_Dir2d.D.Y)), Face, T);
                        B.UpdateEdge(Edge4, new Geom2d_Line(new gp_Pnt2d(l1, 0), new gp_Dir2d(gp_Dir2d.D.Y)), Face, T);
                    }
                }
                else
                {
                    // KPart
                    if (Periodic)
                    {
                        NCollection_Array1<gp_Pnt2d> Extrem1 = new(1, 2);
                        Extrem1[1].SetCoord(l1, f2);
                        Extrem1[2].SetCoord(l1, l2);
                        NCollection_Array1<gp_Pnt2d> Extrem2 = new(1, 2);
                        Extrem2[1].SetCoord(f1, f2);
                        Extrem2[2].SetCoord(f1, l2);
                        B.UpdateEdge(Edge3,
                                     new Geom2d_BezierCurve(Extrem1),
                                     new Geom2d_BezierCurve(Extrem2),
                                     Face,
                                     T);
                    }
                    else if (IType != 4)
                    { // not plane
                        NCollection_Array1<gp_Pnt2d> Extrem2 = new(1, 2);
                        Extrem2[1].SetCoord(f1, f2);
                        Extrem2[2].SetCoord(f1, l2);
                        B.UpdateEdge(Edge3, new Geom2d_BezierCurve(Extrem2), Face, T);
                        NCollection_Array1<gp_Pnt2d> Extrem1 = new(1, 2);
                        Extrem1[1].SetCoord(l1, f2);
                        Extrem1[2].SetCoord(l1, l2);
                        B.UpdateEdge(Edge4, new Geom2d_BezierCurve(Extrem1), Face, T);
                    }
                }
                // Set the non parameter flag;
                B.SameParameter(Edge1, false);
                B.SameParameter(Edge2, false);
                B.SameParameter(Edge3, false);
                B.SameParameter(Edge4, false);
                B.SameRange(Edge1, false);
                B.SameRange(Edge2, false);
                B.SameRange(Edge3, false);
                B.SameRange(Edge4, false);

                // make the wire
                TopoDS_Wire aWire = new();
                B.MakeWire(ref aWire);

                if (!degen1 || IType != 4)
                {
                    B.Add(aWire, Edge1);
                }
                B.Add(aWire, Edge4);
                if (!degen2 || IType != 4)
                {
                    B.Add(aWire, Edge2.Reversed());
                }
                B.Add(aWire, Edge3);

                B.Add(Face, aWire);
                B.Add(myShell, Face);

                // complete myMap for edge1
                if (!degen1 || IType != 4)
                {
                    TopoDS_Edge aREd = anOrEdge1;
                    if (degen1)
                    {
                        aREd = Edge1;
                    }

                    if (!myMap.IsBound(aREd))
                    {
                        NCollection_List<TopoDS_Shape> Empty = new();
                        myMap.Bind(aREd, Empty);
                    }
                    myMap.ChangeFind(aREd).Append(Face);
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
            aFirstWire = false;
        }

        // all vertices from myShell are the part of orig. section wires
        if (myMutableInput)
        {
            BRepLib.SameParameter(myShell);
        }
        else
        {
            foreach (KeyValuePair<TopoDS_Shape, TopoDS_Shape> aMapIt in myOldNewShapes)
            {
                TopoDS_Shape aK = aMapIt.Key;
                TopoDS_Shape aVal = aMapIt.Value;
                myReshaper.Replace(aK, aVal);
            }
            BRepLib.SameParameter(myShell, myReshaper);
            myShell = TopoDS.Shell(myReshaper.InternalApply(myShell));
        }

        if (uClosed && DegenFirst && DegenLast)
        {
            myShell.Closed(true);
        }

        // update wire's history
        TopoDS_Iterator anItEdge = new();
        for (int i = 1; i <= aModifWires.Extent(); i++)
        {
            TopoDS_Shape aCurWire = aModifWires.FindKey(i);
            TopoDS_Wire aNewWire = new();
            B.MakeWire(ref aNewWire);

            anItEdge.Initialize(aCurWire);
            for (; anItEdge.More(); anItEdge.Next())
            {
                TopoDS_Shape aCurEdge = anItEdge.Value(); // edges only
                TopoDS_Shape aNewEdge = resultShape(aCurEdge);
                B.Add(aNewWire, aNewEdge);
            }

            aNewWire.Free(aCurWire.Free());
            aNewWire.Modified(aCurWire.Modified());
            aNewWire.Checked(aCurWire.Checked());
            aNewWire.Orientable(aCurWire.Orientable());
            aNewWire.Closed(aCurWire.Closed());
            aNewWire.Infinite(aCurWire.Infinite());
            aNewWire.Convex(aCurWire.Convex());

            myOldNewShapes.Bind(aCurWire, aNewWire);
        }
    }

    //=================================================================================================

    /// <summary>The shell (frozen).</summary>
    public TopoDS_Shell Shell()
    {
        myShell.TShape()?.FreezeGraph();
        return myShell;
    }

    /// <summary>The shell, not frozen, for ported callers.</summary>
    internal TopoDS_Shell InternalShell() => myShell;

    //=================================================================================================

    /// <summary>
    /// The faces created from the edge <paramref name="SSection"/> of a section, frozen (empty if none). As natively,
    /// each face is keyed by the edge that follows the one it was built on (see docs/occt-issues.md).
    /// </summary>
    public ISchematiqList<TopoDS_Shape> GeneratedShapes(in TopoDS_Shape SSection) => published(InternalGeneratedShapes(SSection));

    private static readonly NCollection_List<TopoDS_Shape> theEmpty = new();

    /// <summary>The algorithm's own list behind <see cref="GeneratedShapes"/>, for ported callers.</summary>
    internal NCollection_List<TopoDS_Shape> InternalGeneratedShapes(in TopoDS_Shape SSection)
    {
        TopoDS_Shape aKey = myCopy is null ? SSection : myCopy.Copied(SSection);
        if (myMap.IsBound(aKey))
        {
            return myMap.Find(aKey);
        }
        else
        {
            return theEmpty;
        }
    }

    /// <summary>All the shapes created, keyed by the sections' edges (frozen), in the order they were built.</summary>
    public ISchematiqList<KeyValuePair<TopoDS_Shape, ISchematiqList<TopoDS_Shape>>> Generated()
    {
        var aPairs = new KeyValuePair<TopoDS_Shape, ISchematiqList<TopoDS_Shape>>[myMap.Extent()];
        int i = 0;
        foreach (KeyValuePair<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anIt in myMap)
        {
            TopoDS_Shape aKey = myCopy is null ? anIt.Key : myCopy.Original(anIt.Key);
            aKey.TShape()?.FreezeGraph();
            aPairs[i++] = new KeyValuePair<TopoDS_Shape, ISchematiqList<TopoDS_Shape>>(aKey, published(anIt.Value));
        }
        return PackedList.Own(aPairs);
    }

    /// <summary>The algorithm's own map behind <see cref="Generated"/> (on the copies when the sections were copied).</summary>
    internal NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> InternalGenerated() => myMap;

    private static ISchematiqList<TopoDS_Shape> published(NCollection_List<TopoDS_Shape> theList)
    {
        if (theList.IsEmpty())
        {
            return PackedList<TopoDS_Shape>.Empty;
        }
        var aShapes = new TopoDS_Shape[theList.Extent()];
        int i = 0;
        for (NCollection_List<TopoDS_Shape>.Iterator anIt = new(theList); anIt.More(); anIt.Next())
        {
            aShapes[i] = anIt.Value();
            aShapes[i].TShape()?.FreezeGraph();
            i++;
        }
        return PackedList.Own(aShapes);
    }

    //=================================================================================================

    /// <summary>
    /// The shape standing for <paramref name="theShape"/> (a section, or one of its sub-shapes) in the shell: itself
    /// when the operation didn't replace it. Frozen.
    /// </summary>
    public TopoDS_Shape ResultShape(in TopoDS_Shape theShape)
    {
        TopoDS_Shape aShape = InternalResultShape(theShape);
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    /// <summary><see cref="ResultShape"/> without freezing, for ported callers.</summary>
    internal TopoDS_Shape InternalResultShape(in TopoDS_Shape theShape)
    {
        if (myCopy is null)
        {
            return resultShape(theShape);
        }
        return myCopy.Final(resultShape(myCopy.Copied(theShape)));
    }

    private TopoDS_Shape resultShape(in TopoDS_Shape theShape)
    {
        TopoDS_Shape aNewShape = myOldNewShapes.Find(theShape, out TopoDS_Shape aShape) ? aShape : theShape;
        TopoDS_Shape aPrevShape;
        do
        {
            aPrevShape = aNewShape;
            aNewShape = myReshaper.InternalValue(aNewShape);
        } while (aNewShape != aPrevShape);
        return aNewShape;
    }

    //=================================================================================================

    /// <summary>
    /// Sets the mutable input state: when true (the default), the sections' edges are given the pcurves and ranges
    /// on the new faces in place (on copies of a caller's frozen sections); when false, on copies of them.
    /// </summary>
    public void SetMutableInput(bool theIsMutableInput)
    {
        myMutableInput = theIsMutableInput;
    }

    /// <summary>The mutable input state.</summary>
    public bool IsMutableInput()
    {
        return myMutableInput;
    }

    /// <summary>The status of the operation.</summary>
    public BRepFill_ThruSectionErrorStatus GetStatus() => myStatus;
}
