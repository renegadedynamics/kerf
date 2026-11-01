// Port of: src/ModelingAlgorithms/TKOffset/BiTgte/BiTgte_Blend.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BiTgte/BiTgte_Blend.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.BiTgte_ContactType;
using static Occt.BRepOffset_Status;
using static Occt.ChFiDS_TypeOfConcavity;
using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_Shape;
using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;
using static Occt.TopAbs_State;

namespace Occt;

// OCCT_DEBUG blocks (chronometers and traces) are left out; so is the file-scope t_mkcurve they fill.

/// <summary>
/// A rolling-ball blend of a shape: a ball of the given radius rolled between faces (or along edges), giving tubes
/// along the edges and spheres at the vertices where three tubes meet; optionally the shape rebuilt with them.
/// </summary>
/// <remarks>
/// <para>
/// A working object (decision 21), Draw's <c>rollingball</c>. It works with the offset's tools (BRepOffset_Inter3d,
/// Inter2d, MakeLoops, Offset) on its own working shapes, and OCCT writes into the shape it is given (3D curves,
/// same-parameter, parameters of the tube's vertices on the edges). So <see cref="Perform"/> copies a frozen shape at
/// the entry (<see cref="BRepTools_CopyAtEntry"/>, decision 1005's rule), maps the faces and edges given onto the copy,
/// and gives back the caller's unchanged faces in the result (decision 9403).
/// </para>
/// <para>
/// <see cref="Shape"/>, <see cref="Face(int)"/> and <see cref="Face(in TopoDS_Shape)"/> freeze what they return;
/// <see cref="SupportShape1"/> and <see cref="SupportShape2"/> return the caller's shapes where they are unchanged.
/// </para>
/// </remarks>
public sealed class BiTgte_Blend
{
    private double myRadius;
    private double myTol;
    private bool myNubs;
    private TopoDS_Shape myShape = new();
    private TopoDS_Shape myResult = new();
    private bool myBuildShape;
    private readonly NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myAncestors =
        new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>>
        myCreated = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> myCutEdges =
        new(TopTools_ShapeMapHasher.Instance);
    private NCollection_IndexedMap<TopoDS_Shape> myFaces = new(TopTools_ShapeMapHasher.Instance);
    private NCollection_IndexedMap<TopoDS_Shape> myEdges = new(TopTools_ShapeMapHasher.Instance);
    private NCollection_Map<TopoDS_Shape> myStopFaces = new(TopTools_ShapeMapHasher.Instance);
    private readonly BRepOffset_Analyse myAnalyse = new();
    private NCollection_IndexedMap<TopoDS_Shape> myCenters = new(TopTools_ShapeMapHasher.Instance);
    private readonly NCollection_DataMap<TopoDS_Shape, BRepOffset_Offset> myMapSF = new(TopTools_ShapeMapHasher.Instance);
    private readonly BRepAlgo_Image myInitOffsetFace = new();
    private readonly BRepAlgo_Image myImage = new();
    private readonly BRepAlgo_Image myImageOffset = new();
    private readonly BRepAlgo_AsDes myAsDes;
    private int myNbBranches;
    private NCollection_HArray1<int>? myIndices;
    private bool myDone;

    // OCCT.NET: the copy of the caller's shape the blend works on (decision 9403).
    private BRepTools_CopyAtEntry? myEntry;

    //======================== START STATIC FUNCTIONS ============

    //=================================================================================================

    private static bool IsOnRestriction(in TopoDS_Vertex V, in TopoDS_Edge CurE, in TopoDS_Face F, out TopoDS_Edge E)
    {
        // find if Vertex V of CurE is on a restriction of F.
        // if yes, store this restriction in E.

        // dub - 03 01 97
        // Method somewhat brutal : possible to really optimize by a
        // direct call the SD of intersections -> See LBR

        E = new TopoDS_Edge();
        Geom2d_Curve CurC = BRep_Tool.CurveOnSurface(CurE, F, out double f, out double l)!;
        double U = BRep_Tool.Parameter(V, CurE, F);
        gp_Pnt2d P = CurC.Value(U);

        Geom2dAPI_ProjectPointOnCurve Proj = new();

        // The tolerance is exaggerated : it is better to construct too many
        // tubes than to miss intersections.
        // double Tol = 100 * BRep_Tool::Tolerance(V);
        double Tol = BRep_Tool.Tolerance(V);
        TopExp_Explorer exp = new(F, TopAbs_EDGE);
        for (; exp.More(); exp.Next())
        {
            E = TopoDS.Edge(exp.Current());
            Geom2d_Curve PC = BRep_Tool.CurveOnSurface(E, F, out f, out l)!;
            Proj.Init(P, PC, f, l);
            if (Proj.NbPoints() > 0)
            {
                if (Proj.LowerDistance() < Tol)
                {
                    return true;
                }
            }
        }
        return false;
    }

    //=================================================================================================

    // If WarningSurBordLibre = TRUE, no propagation if the edge is open.
    private static void Add(in TopoDS_Edge E,
                            NCollection_IndexedMap<TopoDS_Shape> Map,
                            in TopoDS_Shape S,
                            BRepOffset_Offset OF,
                            BRepOffset_Analyse Analyse,
                            bool WarningSurBordLibre)
    {
        TopAbs_ShapeEnum Type = S.ShapeType();

        if (Type == TopAbs_FACE)
        {
            TopExp_Explorer exp = new(S, TopAbs_EDGE);
            for (; exp.More(); exp.Next())
            {
                TopoDS_Edge OriE = TopoDS.Edge(exp.Current());
                TopoDS_Shape aLocalShape = OF.InternalGenerated(OriE);
                TopoDS_Edge IE = TopoDS.Edge(aLocalShape);
                //      const TopoDS_Edge& IE   = TopoDS::Edge(OF.Generated(OriE));
                if (E.IsEqual(IE))
                {
                    if (WarningSurBordLibre)
                    {
                        // It is checked that the border is not free.
                        NCollection_List<TopoDS_Shape> L = Analyse.Ancestors(OriE);
                        if (L.Extent() == 1)
                        {
                            break; // Nothing is done.
                        }
                    }
                    Map.Add(exp.Current());
                    break;
                }
            }
        }
        else if (Type == TopAbs_EDGE)
        {
            TopExp_Explorer exp = new(S, TopAbs_VERTEX);
            for (; exp.More(); exp.Next())
            {
                TopoDS_Shape aLocalShape = OF.InternalGenerated(exp.Current());
                TopoDS_Edge IE = TopoDS.Edge(aLocalShape);
                //      const TopoDS_Edge& IE = TopoDS::Edge(OF.Generated(exp.Current()));
                if (E.IsEqual(IE))
                {
                    NCollection_List<TopoDS_Shape> L = Analyse.Ancestors(exp.Current());
                    NCollection_List<TopoDS_Shape>.Iterator it = new(L);
                    for (; it.More(); it.Next())
                    {
                        Map.Add(it.Value());
                    }
                    break;
                }
            }
        }
    }

    //=================================================================================================

    private static bool IsInFace(in TopoDS_Edge E, in TopoDS_Face F)
    {
        TopExp_Explorer exp = new(F, TopAbs_EDGE);
        for (; exp.More(); exp.Next())
        {
            if (E.IsSame(exp.Current()))
            {
                return true;
            }
        }
        return false;
    }

    //=================================================================================================

    private static void KPartCurve3d(in TopoDS_Edge Edge, Geom2d_Curve Curve, Geom_Surface Surf)
    {
        // try to find the particular case
        // if not found call BRepLib::BuildCurve3d

        TopLoc_Location Loc = new();
        double Tol = Precision.Confusion();

        // Search only isos on analytical surfaces.
        Geom2dAdaptor_Curve C = new(Curve);
        GeomAdaptor_Surface S = new(Surf);
        GeomAbs_CurveType CTy = C.GetType();
        GeomAbs_SurfaceType STy = S.GetType();
        BRep_Builder TheBuilder = new();

        if (STy != GeomAbs_Plane)
        { // if plane buildcurve3d manage KPart
            if (CTy == GeomAbs_Line)
            {
                gp_Dir2d D = C.Line().Direction();
                if (D.IsParallel(gp.DX2d(), Precision.Angular()))
                { // Iso V.
                    if (STy == GeomAbs_Sphere)
                    {
                        gp_Pnt2d P = C.Line().Location();
                        if (Math.Abs(Math.Abs(P.Y()) - M_PI / 2.0) < Precision.PConfusion())
                        {
                            TheBuilder.Degenerated(Edge, true);
                        }
                        else
                        {
                            gp_Sphere Sph = S.Sphere();
                            gp_Ax3 Axis = Sph.Position();
                            gp_Circ Ci = ElSLib.SphereVIso(Axis, Sph.Radius(), P.Y());
                            gp_Dir DRev = Axis.XDirection().Crossed(Axis.YDirection());
                            gp_Ax1 AxeRev = new(Axis.Location(), DRev);
                            Ci.Rotate(AxeRev, P.X());
                            Geom_Circle Circle = new(Ci);
                            if (D.IsOpposite(gp.DX2d(), Precision.Angular()))
                            {
                                Circle.Reverse();
                            }
                            TheBuilder.UpdateEdge(Edge, Circle, Loc, Tol);
                        }
                    }
                    else if (STy == GeomAbs_Cylinder)
                    {
                        gp_Cylinder Cyl = S.Cylinder();
                        gp_Pnt2d P = C.Line().Location();
                        gp_Ax3 Axis = Cyl.Position();
                        gp_Circ Ci = ElSLib.CylinderVIso(Axis, Cyl.Radius(), P.Y());
                        gp_Dir DRev = Axis.XDirection().Crossed(Axis.YDirection());
                        gp_Ax1 AxeRev = new(Axis.Location(), DRev);
                        Ci.Rotate(AxeRev, P.X());
                        Geom_Circle Circle = new(Ci);
                        if (D.IsOpposite(gp.DX2d(), Precision.Angular()))
                        {
                            Circle.Reverse();
                        }
                        TheBuilder.UpdateEdge(Edge, Circle, Loc, Tol);
                    }
                    else if (STy == GeomAbs_Cone)
                    {
                        gp_Cone Cone = S.Cone();
                        gp_Pnt2d P = C.Line().Location();
                        gp_Ax3 Axis = Cone.Position();
                        gp_Circ Ci = ElSLib.ConeVIso(Axis, Cone.RefRadius(), Cone.SemiAngle(), P.Y());
                        gp_Dir DRev = Axis.XDirection().Crossed(Axis.YDirection());
                        gp_Ax1 AxeRev = new(Axis.Location(), DRev);
                        Ci.Rotate(AxeRev, P.X());
                        Geom_Circle Circle = new(Ci);
                        if (D.IsOpposite(gp.DX2d(), Precision.Angular()))
                        {
                            Circle.Reverse();
                        }
                        TheBuilder.UpdateEdge(Edge, Circle, Loc, Tol);
                    }
                    else if (STy == GeomAbs_Torus)
                    {
                        gp_Torus Tore = S.Torus();
                        gp_Pnt2d P = C.Line().Location();
                        gp_Ax3 Axis = Tore.Position();
                        gp_Circ Ci = ElSLib.TorusVIso(Axis, Tore.MajorRadius(), Tore.MinorRadius(), P.Y());
                        gp_Dir DRev = Axis.XDirection().Crossed(Axis.YDirection());
                        gp_Ax1 AxeRev = new(Axis.Location(), DRev);
                        Ci.Rotate(AxeRev, P.X());
                        Geom_Circle Circle = new(Ci);
                        if (D.IsOpposite(gp.DX2d(), Precision.Angular()))
                        {
                            Circle.Reverse();
                        }
                        TheBuilder.UpdateEdge(Edge, Circle, Loc, Tol);
                    }
                }
                else if (D.IsParallel(gp.DY2d(), Precision.Angular()))
                { // Iso U.
                    if (STy == GeomAbs_Sphere)
                    {
                        gp_Sphere Sph = S.Sphere();
                        gp_Pnt2d P = C.Line().Location();
                        gp_Ax3 Axis = Sph.Position();
                        // calculate iso 0.
                        gp_Circ Ci = ElSLib.SphereUIso(Axis, Sph.Radius(), 0.0);

                        // set to sameparameter (rotation of the circle - offset from Y)
                        gp_Dir DRev = Axis.XDirection().Crossed(Axis.Direction());
                        gp_Ax1 AxeRev = new(Axis.Location(), DRev);
                        Ci.Rotate(AxeRev, P.Y());

                        // transformation by iso U ( = P.X())
                        DRev = Axis.XDirection().Crossed(Axis.YDirection());
                        AxeRev = new gp_Ax1(Axis.Location(), DRev);
                        Ci.Rotate(AxeRev, P.X());
                        Geom_Circle Circle = new(Ci);

                        if (D.IsOpposite(gp.DY2d(), Precision.Angular()))
                        {
                            Circle.Reverse();
                        }
                        TheBuilder.UpdateEdge(Edge, Circle, Loc, Tol);
                    }
                    else if (STy == GeomAbs_Cylinder)
                    {
                        gp_Cylinder Cyl = S.Cylinder();
                        gp_Pnt2d P = C.Line().Location();
                        gp_Lin L = ElSLib.CylinderUIso(Cyl.Position(), Cyl.Radius(), P.X());
                        gp_Vec Tr = new(L.Direction());
                        Tr.Multiply(P.Y());
                        L.Translate(Tr);
                        Geom_Line Line = new(L);
                        if (D.IsOpposite(gp.DY2d(), Precision.Angular()))
                        {
                            Line.Reverse();
                        }
                        TheBuilder.UpdateEdge(Edge, Line, Loc, Tol);
                    }
                    else if (STy == GeomAbs_Cone)
                    {
                        gp_Cone Cone = S.Cone();
                        gp_Pnt2d P = C.Line().Location();
                        gp_Lin L = ElSLib.ConeUIso(Cone.Position(), Cone.RefRadius(), Cone.SemiAngle(), P.X());
                        gp_Vec Tr = new(L.Direction());
                        Tr.Multiply(P.Y());
                        L.Translate(Tr);
                        Geom_Line Line = new(L);
                        if (D.IsOpposite(gp.DY2d(), Precision.Angular()))
                        {
                            Line.Reverse();
                        }
                        TheBuilder.UpdateEdge(Edge, Line, Loc, Tol);
                    }
                    else if (STy == GeomAbs_Torus)
                    {
                    }
                }
            }
        }
        else
        { // Case Plane
            Geom_Curve C3d = GeomAPI.To3d(Curve, S.Plane());
            TheBuilder.UpdateEdge(Edge, C3d, Loc, Tol);
        }
    }

    //=================================================================================================

    private sealed class MakeCurve_Function : AppCont_Function
    {
        private readonly BiTgte_CurveOnEdge myCurve;

        // OCCT copies the curve into the function; the port's function only reads it.
        public MakeCurve_Function(BiTgte_CurveOnEdge C)
        {
            myCurve = C;
            myNbPnt = 1;
            myNbPnt2d = 0;
        }

        public override double FirstParameter() => myCurve.FirstParameter();

        public override double LastParameter() => myCurve.LastParameter();

        public override bool Value(double theT, NCollection_Array1<gp_Pnt2d> thePnt2d, NCollection_Array1<gp_Pnt> thePnt)
        {
            thePnt[1] = myCurve.Value(theT);
            return true;
        }

        public override bool D1(double theT, NCollection_Array1<gp_Vec2d> theVec2d, NCollection_Array1<gp_Vec> theVec)
        {
            return false;
        }
    }

    private static Geom_Curve MakeCurve(BiTgte_CurveOnEdge HC)
    {
        Geom_Curve C;

        if (HC.GetType() == GeomAbs_Circle)
        {
            C = new Geom_Circle(HC.Circle());
            C = new Geom_TrimmedCurve(C, HC.FirstParameter(), HC.LastParameter());
        }
        else
        { // the approximation is done
            MakeCurve_Function F = new(HC);
            int Deg1, Deg2;
            Deg1 = Deg2 = 8;
            double Tol = Precision.Approximation();
            Approx_FitAndDivide Fit = new(F, Deg1, Deg2, Tol, Tol, true);
            int i;
            int NbCurves = Fit.NbMultiCurves();
            // it is attempted to make the curve at least C1
            Convert_CompBezierCurvesToBSplineCurve Conv = new();

            for (i = 1; i <= NbCurves; i++)
            {
                AppParCurves_MultiCurve MC = Fit.Value(i); // Load the Ith Curve
                NCollection_Array1<gp_Pnt> Poles = new(1, MC.Degree() + 1); // Return poles
                MC.Curve(1, Poles);

                Conv.AddCurve(Poles);
            }

            Conv.Perform();

            int NbPoles = Conv.NbPoles();
            int NbKnots = Conv.NbKnots();
            NCollection_Array1<gp_Pnt> NewPoles = new(1, NbPoles);
            NCollection_Array1<double> NewKnots = new(1, NbKnots);
            NCollection_Array1<int> NewMults = new(1, NbKnots);

            Conv.KnotsAndMults(NewKnots, NewMults);
            Conv.Poles(NewPoles);

            BSplCLib.Reparametrize(HC.FirstParameter(), HC.LastParameter(), NewKnots);

            C = new Geom_BSplineCurve(NewPoles, NewKnots, NewMults, Conv.Degree());
        }

        return C;
    }

    //=======================================================================
    // function : Touched
    // purpose  : Only the faces connected with caps are given
    //=======================================================================

    private static void Touched(BRepOffset_Analyse Analyse,
                                NCollection_Map<TopoDS_Shape> StopFaces,
                                in TopoDS_Shape Shape,
                                NCollection_Map<TopoDS_Shape> TouchedByCork)
    {
        // currently nothing is done !!
    }

    //=================================================================================================

    private static TopoDS_Vertex FindVertex(in gp_Pnt P, NCollection_Map<TopoDS_Shape> Map, double Tol)
    {
        BRep_Builder B = new();
        // Find in <Map> a vertex which represent the point <P>.
        double Tol2, Dist;
        TopoDS_Vertex V = new();
        TopoDS_Vertex[] VV = new TopoDS_Vertex[2];
        double TolCarre = Tol * Tol;
        // OCCT iterates the map by the edges' addresses; the port in insertion order.
        foreach (TopoDS_Shape aKey in Map)
        {
            TopoDS_Edge E = TopoDS.Edge(aKey);
            if (!E.IsNull())
            {
                TopExp.Vertices(E, out VV[0], out VV[1]);

                for (int i = 0; i < 2; i++)
                {
                    // if OK la Tolerance du Vertex
                    Tol2 = BRep_Tool.Tolerance(VV[i]);
                    Tol2 *= Tol2;
                    gp_Pnt P1 = BRep_Tool.Pnt(VV[i]);
                    Dist = P.SquareDistance(P1);
                    if (Dist <= Tol2)
                    {
                        return VV[i];
                    }
                    // otherwise with the required tolerance.
                    if (TolCarre > Tol2)
                    {
                        if (Dist <= TolCarre)
                        {
                            // so it is necessary to update the tolerance of Vertex.
                            B.UpdateVertex(VV[i], Tol);
                            return VV[i];
                        }
                    }
                }
            }
        }

        return V;
    }

    //=================================================================================================

    private static TopoDS_Edge MakeDegeneratedEdge(Geom_Curve CC, in TopoDS_Vertex VfOnE)
    {
        BRep_Builder B = new();
        double Tol = Precision.Confusion();
        // kill trimmed curves
        Geom_Curve C = CC;
        Geom_TrimmedCurve? CT = C as Geom_TrimmedCurve;
        while (CT is not null)
        {
            C = CT.BasisCurve();
            CT = C as Geom_TrimmedCurve;
        }

        TopoDS_Vertex V1 = new(), V2;
        if (VfOnE.IsNull())
        {
            gp_Pnt P = C.Value(C.FirstParameter());
            B.MakeVertex(ref V1, P, Tol);
            V2 = V1;
        }
        else
        {
            V1 = V2 = VfOnE;
        }
        V1.Orientation(TopAbs_FORWARD);
        V2.Orientation(TopAbs_REVERSED);

        TopoDS_Edge E = new();
        B.MakeEdge(ref E, C, Tol);
        B.Add(E, V1);
        B.Add(E, V2);
        //  B.UpdateVertex(V1,C->FirstParameter(),E,Tol);
        //  B.UpdateVertex(V2,C->LastParameter(),E,Tol);
        B.Range(E, CC.FirstParameter(), CC.LastParameter());
        return E;
    }

    //=================================================================================================

    private static TopAbs_Orientation Orientation(in TopoDS_Edge E, in TopoDS_Face F, NCollection_List<TopoDS_Shape> L)
    {
        TopAbs_Orientation Orien = TopAbs_FORWARD;
        NCollection_List<TopoDS_Shape>.Iterator itld = new();
        for (itld.Initialize(L); itld.More(); itld.Next())
        {
            if (itld.Value().IsSame(E))
            {
                Orien = itld.Value().Orientation();
                break;
            }
        }
        if (F.Orientation() == TopAbs_REVERSED)
        {
            Orien = TopAbs.Reverse(Orien);
        }

        return Orien;
    }

    //=================================================================================================

    private static TopoDS_Edge FindCreatedEdge(in TopoDS_Vertex V1,
                                               in TopoDS_Edge E,
                                               NCollection_DataMap<TopoDS_Shape, BRepOffset_Offset> MapSF,
                                               NCollection_Map<TopoDS_Shape> MapOnV,
                                               BRepOffset_Analyse CenterAnalyse,
                                               double Radius,
                                               double Tol)
    {
        TopoDS_Edge E1 = new();
        if (!CenterAnalyse.HasAncestor(V1))
        {
            return E1; // return a Null Shape.
        }

        NCollection_List<TopoDS_Shape> TangE = new();
        CenterAnalyse.TangentEdges(E, V1, TangE);

        NCollection_List<TopoDS_Shape>.Iterator itl = new(TangE);
        bool Find = false;
        for (; itl.More() && !Find; itl.Next())
        {
            TopoDS_Edge ET = TopoDS.Edge(itl.Value());
            if (MapSF.IsBound(ET))
            {
                TopoDS_Shape aLocalShape = MapSF.Find(ET).InternalGenerated(V1);
                E1 = TopoDS.Edge(aLocalShape);
                //      E1 = TopoDS::Edge(MapSF(ET).Generated(V1));
                MapOnV.Add(E1);
                Find = true;
            }
            else
            {
                // Find the sharing of vertices in case of tangent consecutive 3 edges
                // the second of which is the edge that degenerates the tube.
                Geom_Curve CET = BRep_Tool.Curve(ET, out TopLoc_Location CLoc, out double ff, out double ll)!;
                if (CET.GetType() == typeof(Geom_TrimmedCurve))
                {
                    CET = ((Geom_TrimmedCurve)CET).BasisCurve();
                }
                if (CET is not Geom_Circle Circ)
                {
                    continue;
                }
                if (Math.Abs(Circ.Radius() - Math.Abs(Radius)) > Tol)
                {
                    continue;
                }

                TopExp.Vertices(ET, out TopoDS_Vertex U1, out TopoDS_Vertex U2);
                if (U1.IsSame(V1))
                {
                    U1 = U2;
                }
                NCollection_List<TopoDS_Shape> Tang2 = new();
                CenterAnalyse.TangentEdges(ET, U1, Tang2);
                NCollection_List<TopoDS_Shape>.Iterator it2 = new(Tang2);
                for (; it2.More(); it2.Next())
                {
                    TopoDS_Edge ET2 = TopoDS.Edge(it2.Value());
                    if (MapSF.IsBound(ET2))
                    {
                        TopoDS_Shape aLocalShape = MapSF.Find(ET2).InternalGenerated(U1);
                        MapOnV.Add(TopoDS.Edge(aLocalShape));
                        //	  MapOnV.Add(TopoDS::Edge(MapSF(ET2).Generated(U1)));
                    }
                }
            }
        }
        if (!Find)
        {
            TangE.Clear();
            //	CenterAnalyse.Edges(V1f, OT, TangE);
            if (CenterAnalyse.HasAncestor(V1))
            {
                // OCCT assigns (copies) the ancestors' list.
                TangE = new NCollection_List<TopoDS_Shape>(CenterAnalyse.Ancestors(V1));
                itl.Initialize(TangE);
                for (; itl.More() && !Find; itl.Next())
                {
                    if (MapSF.IsBound(itl.Value()))
                    {
                        MapOnV.Add(MapSF.Find(itl.Value()).InternalGenerated(V1));
                    }
                }
            }
        }

        return E1;
    }

    //======================== END OF STATIC FUNCTIONS ============

    //=================================================================================================

    /// <summary>An empty blend; <see cref="Init"/> gives the shape.</summary>
    public BiTgte_Blend()
    {
        myAsDes = new BRepAlgo_AsDes();
        myNbBranches = -1;
    }

    //=================================================================================================

    /// <summary>
    /// The blend of <paramref name="S"/> with the radius <paramref name="Radius"/>; <paramref name="Tol"/> is the 3D
    /// tolerance of the approximations, <paramref name="NUBS"/> true makes only B-spline surfaces.
    /// </summary>
    public BiTgte_Blend(in TopoDS_Shape S, double Radius, double Tol, bool NUBS)
    {
        myAsDes = new BRepAlgo_AsDes();
        Init(S, Radius, Tol, NUBS);
    }

    //=================================================================================================

    /// <summary>Sets the shape, radius, tolerance and surface kind, and clears the rest.</summary>
    public void Init(in TopoDS_Shape S, double Radius, double Tol, bool NUBS)
    {
        Clear();
        myShape = S;
        myTol = Tol;
        myNubs = NUBS;
        myRadius = Radius;
        myNbBranches = -1;
        //  TopExp::MapShapesAndAncestors(S,TopAbs_EDGE,TopAbs_FACE,myAncestors);
    }

    //=================================================================================================

    /// <summary>Clears all the fields.</summary>
    public void Clear()
    {
        myInitOffsetFace.Clear();
        myImage.Clear();
        myImageOffset.Clear();
        myStopFaces.Clear();
        myAnalyse.Clear();
        myAsDes.Clear();
        myNbBranches = -1;
        myDone = false;
        myEntry = null;
    }

    //=================================================================================================

    /// <summary>Sets a face of the shape on which the fillet must stop.</summary>
    public void SetStoppingFace(in TopoDS_Face Face)
    {
        myStopFaces.Add(Face);
        //-------------
        // MAJ SD. -> To end loop, set faces of edges
        //-------------
        //  myInitOffsetFace.SetRoot(Face);
        //  myInitOffsetFace.Bind   (Face,Face);
        //  myImageOffset.SetRoot   (Face);
    }

    //=================================================================================================

    /// <summary>Sets two faces of the shape between which the ball must roll.</summary>
    public void SetFaces(in TopoDS_Face F1, in TopoDS_Face F2)
    {
        myFaces.Add(F1);
        myFaces.Add(F2);
    }

    //=================================================================================================

    /// <summary>Sets an edge of the shape to be rounded.</summary>
    public void SetEdge(in TopoDS_Edge Edge)
    {
        myEdges.Add(Edge);
    }

    //=================================================================================================

    /// <summary>
    /// Computes the blending surfaces, and the resulting shape if <paramref name="BuildShape"/>.
    /// </summary>
    /// <exception cref="Standard_Failure">The sewing of the shape failed.</exception>
    public void Perform(bool BuildShape = true)
    {
        myBuildShape = BuildShape;

        // OCCT.NET: the work is done on a copy of a frozen shape, the faces and edges given mapped onto it (decision
        // 9403).
        myEntry = new BRepTools_CopyAtEntry(myShape);
        if (myEntry.IsCopied())
        {
            myShape = myEntry.Shape();
            NCollection_IndexedMap<TopoDS_Shape> aFaces = new(TopTools_ShapeMapHasher.Instance);
            for (int k = 1; k <= myFaces.Extent(); k++)
            {
                aFaces.Add(myEntry.Copied(myFaces[k]));
            }
            myFaces = aFaces;
            NCollection_IndexedMap<TopoDS_Shape> anEdges = new(TopTools_ShapeMapHasher.Instance);
            for (int k = 1; k <= myEdges.Extent(); k++)
            {
                anEdges.Add(myEntry.Copied(myEdges[k]));
            }
            myEdges = anEdges;
            NCollection_Map<TopoDS_Shape> aStops = new(TopTools_ShapeMapHasher.Instance);
            foreach (TopoDS_Shape aStop in myStopFaces)
            {
                aStops.Add(myEntry.Copied(aStop));
            }
            myStopFaces = aStops;
        }

        // Try cutting to avoid tubes on free borders
        // that are not actually free.
        BRepBuilderAPI_Sewing Sew = new(myTol);
        BRepLib.BuildCurves3d(myShape);
        TopExp_Explorer expf = new(myShape, TopAbs_FACE);
        for (; expf.More(); expf.Next())
        {
            Sew.Add(expf.Current());
        }
        Sew.Perform();
        TopoDS_Shape SewedShape = Sew.InternalSewedShape();
        if (SewedShape.IsNull())
        {
            throw new Standard_Failure("Sewing aux fraises");
        }

        // Check if the sewing modified the orientation.
        expf.Init(myShape, TopAbs_FACE);
        TopoDS_Face FaceRef = TopoDS.Face(expf.Current());
        TopAbs_Orientation OriRef = FaceRef.Orientation();
        if (Sew.IsModified(FaceRef))
        {
            FaceRef = TopoDS.Face(Sew.InternalModified(FaceRef));
        }
        expf.Init(SewedShape, TopAbs_FACE);
        for (; expf.More(); expf.Next())
        {
            TopoDS_Face FF = TopoDS.Face(expf.Current());
            if (FaceRef.IsSame(FF) && (FF.Orientation() != OriRef))
            {
                SewedShape.Reverse();
                break;
            }
        }

        // Make SameParameter if Sew does not do it (Detect that edges
        // are not sameparameter but it does nothing.)
        expf.Init(SewedShape, TopAbs_EDGE);
        for (; expf.More(); expf.Next())
        {
            TopoDS_Edge sec = TopoDS.Edge(expf.Current());
            BRepLib.SameParameter(sec, BRep_Tool.Tolerance(sec));
        }

        TopExp.MapShapesAndAncestors(SewedShape, TopAbs_EDGE, TopAbs_FACE, myAncestors);

        // Extend myFaces with the faces of the sewed shape.
        expf.Init(myShape, TopAbs_FACE);
        for (; expf.More(); expf.Next())
        {
            TopoDS_Shape F = expf.Current();
            if (myFaces.Contains(F) && Sew.IsModified(F))
            {
                myFaces.RemoveKey(F);
                myFaces.Add(Sew.InternalModified(F));
            }
        }

        myShape = SewedShape;
        // end Sewing for false free borders.

        // ----------------------------------------------------------------
        // place faces with the proper orientation in the initial shape
        // ----------------------------------------------------------------
        TopExp_Explorer exp = new(myShape, TopAbs_FACE);
        for (; exp.More(); exp.Next())
        {
            TopoDS_Shape F = exp.Current();
            if (myFaces.Contains(F))
            {
                myFaces.RemoveKey(F);
                myFaces.Add(F);
            }
            else if (myStopFaces.Contains(F))
            {
                myStopFaces.Remove(F);
                myStopFaces.Add(F);
            }
        }

        // ----------------------------------------------
        // Calculate lines of centers and of surfaces
        // ----------------------------------------------
        ComputeCenters();

        // -----------------------------
        // Calculate connection Surfaces
        // -----------------------------
        ComputeSurfaces();

        // ----------------------------------
        // Calculate the generated shape if required
        // ----------------------------------
        if (myBuildShape)
        {
            ComputeShape();
        }

        // Finally construct curves 3d from edges to be transferred
        // since the partition is provided ( A Priori);
        BRepLib.BuildCurves3d(myResult, Precision.Confusion());

        // OCCT.NET: the caller's faces the blend left alone stand in the result again.
        myResult = myEntry.Finish(myResult);

        myDone = true;
    }

    //=================================================================================================

    /// <summary>True if <see cref="Perform"/> was run.</summary>
    public bool IsDone() => myDone;

    //=================================================================================================

    /// <summary>The result (frozen).</summary>
    public TopoDS_Shape Shape()
    {
        myResult.TShape()?.FreezeGraph();
        return myResult;
    }

    /// <summary><see cref="Shape"/> without freezing, for ported code.</summary>
    internal TopoDS_Shape InternalShape() => myResult;

    //=================================================================================================

    /// <summary>The number of blending surfaces.</summary>
    public int NbSurfaces() => myCenters.Extent();

    //=================================================================================================

    /// <summary>The blending surface of index <paramref name="Index"/>.</summary>
    public Geom_Surface? Surface(int Index) => Surface(myCenters[Index]);

    //=================================================================================================

    /// <summary>The blending face of index <paramref name="Index"/> (frozen).</summary>
    public TopoDS_Face Face(int Index) => Face(myCenters[Index]);

    //=================================================================================================

    /// <summary>Sets in <paramref name="LC"/> all the center lines.</summary>
    public void CenterLines(NCollection_List<TopoDS_Shape> LC)
    {
        LC.Clear();
        int Nb = NbSurfaces();
        for (int i = 1; i <= Nb; i++)
        {
            LC.Append(myCenters[i]);
        }
    }

    //=================================================================================================

    /// <summary>
    /// The surface generated by the center line <paramref name="CenterLine"/>: a pipe for an edge, a sphere for a
    /// vertex.
    /// </summary>
    public Geom_Surface? Surface(in TopoDS_Shape CenterLine)
    {
        TopoDS_Face F = myMapSF.Find(CenterLine).InternalFace();
        Geom_Surface? S = BRep_Tool.Surface(F);
        S?.Freeze();
        return S;
    }

    //=================================================================================================

    /// <summary>The face generated by the center line <paramref name="CenterLine"/> (frozen).</summary>
    /// <exception cref="Standard_DomainError">The center line generated no face.</exception>
    public TopoDS_Face Face(in TopoDS_Shape CenterLine)
    {
        if (!myMapSF.IsBound(CenterLine))
        {
            throw new Standard_DomainError("BiTgte_Blend::Face");
        }

        TopoDS_Face aFace = myMapSF.Find(CenterLine).InternalFace();
        aFace.TShape()?.FreezeGraph();
        return aFace;
    }

    //=================================================================================================

    /// <summary>The type of contact of the blending surface of index <paramref name="Index"/>.</summary>
    public BiTgte_ContactType ContactType(int Index)
    {
        TopoDS_Shape S1 = SupportShape1(Index);
        TopoDS_Shape S2 = SupportShape2(Index);

        TopAbs_ShapeEnum Type1 = S1.ShapeType();
        TopAbs_ShapeEnum Type2 = S2.ShapeType();

        if (Type2 < Type1)
        {
            TopAbs_ShapeEnum Dummy = Type1;
            Type1 = Type2;
            Type2 = Dummy;
        }
        BiTgte_ContactType Type = BiTgte_VertexVertex;

        switch (Type1)
        {
            case TopAbs_VERTEX:
                switch (Type2)
                {
                    case TopAbs_VERTEX:
                        Type = BiTgte_VertexVertex;
                        break;
                    case TopAbs_EDGE:
                        Type = BiTgte_EdgeVertex;
                        break;
                    case TopAbs_FACE:
                        Type = BiTgte_FaceVertex;
                        break;
                    default:
                        break;
                }
                break;

            case TopAbs_EDGE:
                switch (Type2)
                {
                    case TopAbs_EDGE:
                        Type = BiTgte_EdgeEdge;
                        break;
                    case TopAbs_FACE:
                        Type = BiTgte_FaceEdge;
                        break;
                    default:
                        break;
                }
                break;

            case TopAbs_FACE:
                switch (Type2)
                {
                    case TopAbs_FACE:
                        Type = BiTgte_FaceEdge; // OCCT: FaceFace is never returned
                        break;
                    default:
                        break;
                }
                break;

            default:
                break;
        }

        return Type;
    }

    //=================================================================================================

    /// <summary>The first support shape of the blending surface of index <paramref name="Index"/>.</summary>
    public TopoDS_Shape SupportShape1(int Index)
    {
        TopoDS_Edge CurE = TopoDS.Edge(myCenters[Index]);

        NCollection_List<TopoDS_Shape> L = myAsDes.Ascendant(CurE);

        // --------------------------------------------------------------
        // F1 and F2 = 2 parallel faces intersecting at CurE.
        // --------------------------------------------------------------
        TopoDS_Face F1 = TopoDS.Face(L.First());
        TopoDS_Shape Or1 = myInitOffsetFace.ImageFrom(F1);
        return published(Or1);
    }

    //=================================================================================================

    /// <summary>The second support shape of the blending surface of index <paramref name="Index"/>.</summary>
    public TopoDS_Shape SupportShape2(int Index)
    {
        TopoDS_Edge CurE = TopoDS.Edge(myCenters[Index]);

        NCollection_List<TopoDS_Shape> L = myAsDes.Ascendant(CurE);

        // --------------------------------------------------------------
        // F1 and F2 = 2 parallel faces intersecting at CurE.
        // --------------------------------------------------------------
        TopoDS_Face F2 = TopoDS.Face(L.Last());
        TopoDS_Shape Or2 = myInitOffsetFace.ImageFrom(F2);
        return published(Or2);
    }

    // OCCT.NET: a shape of the working copy as the caller sees it: its own where unchanged, frozen.
    private TopoDS_Shape published(in TopoDS_Shape theShape)
    {
        TopoDS_Shape aShape = myEntry is not null && myDone ? myEntry.Final(theShape) : theShape;
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    //=================================================================================================

    /// <summary>The 3D curve of the blending surface of index <paramref name="Index"/> on its first support.</summary>
    public Geom_Curve? CurveOnShape1(int Index)
    {
        TopoDS_Edge CurE = TopoDS.Edge(myCenters[Index]);
        TopoDS_Shape F = myMapSF.Find(CurE).InternalFace();

        // somewhat brutal method based ONLY on the construction of the fillet:
        // the first edge of the tube is exactly the edge on Shape1.

        TopExp_Explorer exp = new(F, TopAbs_EDGE);
        TopoDS_Edge E = TopoDS.Edge(exp.Current());
        Geom_Curve? C = null;
        if (!BRep_Tool.Degenerated(E))
        {
            C = BRep_Tool.Curve(E, out double f, out double l);
            C = new Geom_TrimmedCurve(C!, f, l);
        }
        return C;
    }

    //=================================================================================================

    /// <summary>The 3D curve of the blending surface of index <paramref name="Index"/> on its second support.</summary>
    public Geom_Curve? CurveOnShape2(int Index)
    {
        TopoDS_Edge CurE = TopoDS.Edge(myCenters[Index]);
        TopoDS_Shape F = myMapSF.Find(CurE).InternalFace();

        // somewhat brutal method based ONLY on the construction of the fillet:
        // the first edge of the tube is exactly the edge on Shape2.

        TopExp_Explorer exp = new(F, TopAbs_EDGE);
        exp.Next();
        TopoDS_Edge E = TopoDS.Edge(exp.Current());
        Geom_Curve? C = null;
        if (!BRep_Tool.Degenerated(E))
        {
            C = BRep_Tool.Curve(E, out double f, out double l);
            C = new Geom_TrimmedCurve(C!, f, l);
        }
        return C;
    }

    //=================================================================================================

    /// <summary>Always null in OCCT.</summary>
    public Geom2d_Curve? PCurveOnFace1(int Index) => null;

    /// <summary>Always null in OCCT.</summary>
    public Geom2d_Curve? PCurve1OnFillet(int Index) => null;

    /// <summary>Always null in OCCT.</summary>
    public Geom2d_Curve? PCurveOnFace2(int Index) => null;

    /// <summary>Always null in OCCT.</summary>
    public Geom2d_Curve? PCurve2OnFillet(int Index) => null;

    //=================================================================================================

    /// <summary>The number of branches (sets of connected blending faces); reorders the center lines by branch.</summary>
    public int NbBranches()
    {
        if (myNbBranches != -1)
        {
            return myNbBranches;
        }

        // else, compute the Branches.
        BRepTools_Quilt Glue = new();

        int NbFaces = myCenters.Extent();

        if (NbFaces == 0)
        {
            return 0;
        }

        int i;
        for (i = 1; i <= NbFaces; i++)
        {
            TopoDS_Shape CenterLine = myCenters[i];
            Glue.Add(myMapSF.Find(CenterLine).InternalFace());
        }

        TopoDS_Shape Shells = Glue.Shells();

        // Reorder Map myCenters.
        // The method is brutal and unpolished,
        // it is possible to refine it.
        myNbBranches = 0;
        NCollection_IndexedMap<TopoDS_Shape> tmpMap = new(TopTools_ShapeMapHasher.Instance);

        TopExp_Explorer exp = new(Shells, TopAbs_SHELL);
        for (; exp.More(); exp.Next())
        {
            myNbBranches++;
        }

        myIndices = new NCollection_HArray1<int>(1, myNbBranches + 1);

        myIndices.SetValue(1, 0);
        int Count = 0;
        int Index = 2;

        exp.Init(Shells, TopAbs_SHELL);
        for (; exp.More(); exp.Next())
        {
            // CurS = the current Shell.
            TopoDS_Shape CurS = exp.Current();

            TopExp_Explorer exp2 = new(CurS, TopAbs_FACE);
            for (; exp2.More(); exp2.Next())
            {
                // CurF = the current face of the current Shell.
                TopoDS_Shape CurF = exp2.Current();

                for (i = 1; i <= NbFaces; i++)
                {
                    TopoDS_Shape Center = myCenters[i];
                    TopoDS_Shape Rakk = myMapSF.Find(Center).InternalFace();
                    // Rakk = the ith generated connection face
                    if (CurF.IsEqual(Rakk))
                    {
                        tmpMap.Add(Center);
                        Count++;
                        break;
                    }
                }
            }
            myIndices.SetValue(Index, Count);
            Index++;
        }

        myCenters = tmpMap;
        return myNbBranches;
    }

    //=================================================================================================

    /// <summary>
    /// The indices of the faces of the branch <paramref name="Index"/>: Face(From) to Face(To) (after
    /// <see cref="NbBranches"/>).
    /// </summary>
    public void IndicesOfBranche(int Index, out int From, out int To)
    {
        // Attention to the ranking in myIndices:
        // If the branches are  1-4 5-9 10-12, it is ranked in myIndices:
        //                      0 4   9    12
        From = myIndices!.Value(Index) + 1;
        To = myIndices.Value(Index + 1);
    }

    //=================================================================================================

    // OCCT binds copies of an empty map and an empty list into myCreated; the port binds new ones.
    private NCollection_List<TopoDS_Shape> created(in TopoDS_Shape theKey1, in TopoDS_Shape theKey2) =>
        myCreated.Find(theKey1).Find(theKey2);

    private void bindCreated(in TopoDS_Shape theKey) =>
        myCreated.Bind(theKey, new NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>>(TopTools_ShapeMapHasher.Instance));

    private void bindCreated(in TopoDS_Shape theKey1, in TopoDS_Shape theKey2) =>
        myCreated.Find(theKey1).Bind(theKey2, new NCollection_List<TopoDS_Shape>());

    //=================================================================================================

    /// <summary>Computes the center lines.</summary>
    public void ComputeCenters()
    {
        // ------------
        // Preanalyze.
        // ------------
        double TolAngle = 2 * Math.Asin(myTol / Math.Abs(myRadius * 0.5));
        myAnalyse.Perform(myShape, TolAngle);

        // ------------------------------------------
        // calculate faces touched by caps
        // ------------------------------------------
        NCollection_Map<TopoDS_Shape> TouchedByCork = new(TopTools_ShapeMapHasher.Instance);
        Touched(myAnalyse, myStopFaces, myShape, TouchedByCork);

        // -----------------------
        // init of the intersector
        // -----------------------
        TopAbs_State Side = TopAbs_IN;
        if (myRadius < 0.0)
        {
            Side = TopAbs_OUT;
        }
        BRepOffset_Inter3d Inter = new(myAsDes, Side, myTol);

        NCollection_DataMap<TopoDS_Shape, Bnd_Box> MapSBox = new(TopTools_ShapeMapHasher.Instance);
        NCollection_Map<TopoDS_Shape> Done = new(TopTools_ShapeMapHasher.Instance);

        BRep_Builder B = new();
        TopoDS_Compound Co = new(); // to only know on which edges the tubes are made
        B.MakeCompound(ref Co);

        // ----------------------------------------
        // Calculate Sections Face/Face + Propagation
        // ----------------------------------------
        bool JenRajoute = true;
        int i;

        while (JenRajoute)
        {
            JenRajoute = false;

            bool Fini = false;

            NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> EdgeTgt = new(TopTools_ShapeMapHasher.Instance);

            while (!Fini)
            {
                // -------------------------------------------------
                // locate in myFaces the Faces connected to myEdges.
                // -------------------------------------------------
                Fini = true;
                // for (it.Initialize(myEdges); it.More(); it.Next()) {
                for (i = 1; i <= myEdges.Extent(); i++)
                {
                    TopoDS_Edge E = TopoDS.Edge(myEdges[i]);
                    if (BRep_Tool.Degenerated(E))
                    {
                        continue;
                    }

                    NCollection_List<TopoDS_Shape> L = myAncestors.FindFromKey(E);
                    if (L.Extent() == 1)
                    {
                        // So this is a free border onwhich the ball should roll.
                        myFaces.Add(E);

                        // set in myStopFaces to not propagate the tube on free border.
                        myStopFaces.Add(E);
                    }
                    else
                    {
                        NCollection_List<TopoDS_Shape>.Iterator itl = new();
                        for (itl.Initialize(L); itl.More(); itl.Next())
                        {
                            TopoDS_Shape Sh = itl.Value();
                            if (!myStopFaces.Contains(Sh))
                            {
                                myFaces.Add(itl.Value());
                            }
                        }
                    }
                }
                myEdges.Clear();

                // --------------------------------------------
                // Construction of Offsets of all faces.
                // --------------------------------------------
                // for (it.Initialize(myFaces); it.More(); it.Next()) {
                for (i = 1; i <= myFaces.Extent(); i++)
                {
                    TopoDS_Shape AS = myFaces[i];
                    if (myMapSF.IsBound(AS))
                    {
                        continue;
                    }

                    BRepOffset_Offset OF1 = new();
                    TopoDS_Face BigF = new();

                    if (AS.ShapeType() == TopAbs_FACE)
                    {
                        TopoDS_Face F = TopoDS.Face(myFaces[i]);
                        if (TouchedByCork.Contains(F))
                        {
                            BRepOffset_Tool.EnLargeFace(F, ref BigF, true);
                            OF1.Init(BigF, myRadius, EdgeTgt);
                        }
                        else
                        {
                            OF1.Init(F, myRadius, EdgeTgt);
                        }
                    }
                    else
                    { // So this is a Free Border edge on which the ball rolls.
                        OF1.Init(TopoDS.Edge(AS), myRadius);
                    }

                    // ------------------------------------
                    // Increment the map of created tangents
                    // ------------------------------------
                    NCollection_List<TopoDS_Shape> Let = new();
                    if (AS.ShapeType() == TopAbs_FACE)
                    {
                        myAnalyse.Edges(TopoDS.Face(AS), ChFiDS_Tangential, Let);
                    }
                    NCollection_List<TopoDS_Shape>.Iterator itlet = new(Let);

                    for (; itlet.More(); itlet.Next())
                    {
                        TopoDS_Edge Cur = TopoDS.Edge(itlet.Value());
                        if (!EdgeTgt.IsBound(Cur))
                        {
                            TopoDS_Shape aLocalShape = OF1.InternalGenerated(Cur);
                            TopoDS_Edge OTE = TopoDS.Edge(aLocalShape);
                            //	    const TopoDS_Edge& OTE = TopoDS::Edge(OF1.Generated(Cur));
                            EdgeTgt.Bind(Cur, OF1.InternalGenerated(Cur));
                            TopExp.Vertices(Cur, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                            TopExp.Vertices(OTE, out TopoDS_Vertex OV1, out TopoDS_Vertex OV2);
                            NCollection_List<TopoDS_Shape> LE = new();
                            if (!EdgeTgt.IsBound(V1))
                            {
                                myAnalyse.Edges(V1, ChFiDS_Tangential, LE);
                                NCollection_List<TopoDS_Shape> LA = myAnalyse.Ancestors(V1);
                                if (LE.Extent() == LA.Extent())
                                {
                                    EdgeTgt.Bind(V1, OV1);
                                }
                            }
                            if (!EdgeTgt.IsBound(V2))
                            {
                                LE.Clear();
                                myAnalyse.Edges(V2, ChFiDS_Tangential, LE);
                                NCollection_List<TopoDS_Shape> LA = myAnalyse.Ancestors(V2);
                                if (LE.Extent() == LA.Extent())
                                {
                                    EdgeTgt.Bind(V2, OV2);
                                }
                            }
                        }
                    }
                    // end of map created tangent

                    if (OF1.Status() == BRepOffset_Reversed || OF1.Status() == BRepOffset_Degenerated)
                    {
                        continue;
                    }

                    TopoDS_Face F1 = OF1.InternalFace();

                    // increment S D
                    myInitOffsetFace.SetRoot(AS);
                    myInitOffsetFace.Bind(AS, F1);

                    Bnd_Box Box1 = new();
                    BRepBndLib.Add(F1, ref Box1);
                    MapSBox.Bind(F1, Box1);

                    // ---------------------------------------------
                    // intersection with all already created faces.
                    // ---------------------------------------------
                    Fini = !Intersect(AS, F1, MapSBox, OF1, Inter);

                    if (AS.ShapeType() == TopAbs_FACE)
                    {
                        B.Add(Co, AS);
                    }

                    myMapSF.Bind(AS, OF1);
                }
            } // end of : while ( !Fini)

            //--------------------------------------------------------
            // so the offsets were created and intersected.
            // now the tubes are constructed.
            //--------------------------------------------------------
            // Construction of tubes on edge.
            //--------------------------------------------------------
            ChFiDS_TypeOfConcavity OT1 = ChFiDS_Convex;
            if (myRadius < 0.0)
            {
                OT1 = ChFiDS_Concave;
            }

            NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> Map =
                new(TopTools_ShapeMapHasher.Instance);
            TopExp.MapShapesAndAncestors(Co, TopAbs_EDGE, TopAbs_FACE, Map);
            TopExp.MapShapesAndAncestors(Co, TopAbs_VERTEX, TopAbs_EDGE, Map);

            TopExp_Explorer exp = new(Co, TopAbs_EDGE);
            for (; exp.More(); exp.Next())
            {
                TopoDS_Edge E = TopoDS.Edge(exp.Current());
                if (myMapSF.IsBound(E))
                {
                    continue;
                }

                NCollection_List<TopoDS_Shape> Anc = Map.FindFromKey(E);
                if (Anc.Extent() == 2)
                {
                    NCollection_List<BRepOffset_Interval> L = myAnalyse.Type(E);
                    if (!L.IsEmpty() && L.First().Type() == OT1)
                    {
                        TopoDS_Shape aLocalShapeGen = myMapSF.Find(Anc.First()).InternalGenerated(E);
                        TopoDS_Edge EOn1 = TopoDS.Edge(aLocalShapeGen);
                        aLocalShapeGen = myMapSF.Find(Anc.Last()).InternalGenerated(E);
                        TopoDS_Edge EOn2 = TopoDS.Edge(aLocalShapeGen);
                        //	  TopoDS_Edge EOn1 = TopoDS::Edge(myMapSF(Anc.First()).Generated(E));
                        //	  TopoDS_Edge EOn2 = TopoDS::Edge(myMapSF(Anc.Last()) .Generated(E));
                        // find if exits tangent edges in the original shape
                        TopoDS_Edge E1f = new(), E1l = new();
                        TopExp.Vertices(E, out TopoDS_Vertex V1f, out TopoDS_Vertex V1l);
                        NCollection_List<TopoDS_Shape> TangE = new();
                        myAnalyse.TangentEdges(E, V1f, TangE);
                        // find if the pipe on the tangent edges are soon created.
                        NCollection_List<TopoDS_Shape>.Iterator itl = new(TangE);
                        bool Find = false;
                        for (; itl.More() && !Find; itl.Next())
                        {
                            if (myMapSF.IsBound(itl.Value()))
                            {
                                TopoDS_Shape aLocalShape = myMapSF.Find(itl.Value()).InternalGenerated(V1f);
                                E1f = TopoDS.Edge(aLocalShape);
                                //	      E1f  = TopoDS::Edge(myMapSF(itl.Value()).Generated(V1f));
                                Find = true;
                            }
                        }
                        TangE.Clear();
                        myAnalyse.TangentEdges(E, V1l, TangE);
                        // find if the pipe on the tangent edges are soon created.
                        itl.Initialize(TangE);
                        Find = false;
                        for (; itl.More() && !Find; itl.Next())
                        {
                            if (myMapSF.IsBound(itl.Value()))
                            {
                                TopoDS_Shape aLocalShape = myMapSF.Find(itl.Value()).InternalGenerated(V1l);
                                E1l = TopoDS.Edge(aLocalShape);
                                //	      E1l  = TopoDS::Edge(myMapSF(itl.Value()).Generated(V1l));
                                Find = true;
                            }
                        }
                        BRepOffset_Offset OF1 = new(E, EOn1, EOn2, myRadius, E1f, E1l);
                        TopoDS_Face F1 = OF1.InternalFace();

                        // maj S D
                        myInitOffsetFace.SetRoot(E);
                        myInitOffsetFace.Bind(E, F1);

                        Bnd_Box Box1 = new();
                        BRepBndLib.Add(F1, ref Box1);
                        MapSBox.Bind(F1, Box1);

                        // ---------------------------------------------
                        // intersection with all already created faces.
                        // ---------------------------------------------
                        bool IsOnRest = Intersect(E, F1, MapSBox, OF1, Inter);
                        JenRajoute = JenRajoute || IsOnRest;

                        myMapSF.Bind(E, OF1);
                    }
                }
            }
        } // end while JenRajoute

        myEdges.Clear();
        myEdges = new NCollection_IndexedMap<TopoDS_Shape>(Inter.NewEdges());

        // -------------------------------------------------------------------
        // now it is necessary to limit edges on the neighbors (otherwise one
        // will go too far and will not be able to construct faces).
        // -------------------------------------------------------------------

        // Proceed with MakeLoops
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> aDMVV =
            new(TopTools_ShapeMapHasher.Instance);
        ChFiDS_TypeOfConcavity OT = ChFiDS_Concave;
        if (myRadius < 0.0)
        {
            OT = ChFiDS_Convex;
        }

        NCollection_List<TopoDS_Shape> LOF = new();
        // it.Initialize(myFaces);
        for (i = 1; i <= myFaces.Extent(); i++)
        {
            TopoDS_Shape CurS = myFaces[i];

            // tube on free border, it is undesirable.
            if (myStopFaces.Contains(CurS))
            {
                continue;
            }

            if (!myMapSF.IsBound(CurS))
            {
                continue; // inverted or degenerated
            }

            TopoDS_Face CurOF = myMapSF.Find(CurS).InternalFace();
            LOF.Append(CurOF);

            if (CurS.ShapeType() == TopAbs_FACE)
            {
                TopoDS_Face CurF = TopoDS.Face(CurS);
                TopExp_Explorer expe = new(CurF.Oriented(TopAbs_FORWARD), TopAbs_EDGE);
                for (; expe.More(); expe.Next())
                {
                    // --------------------------------------------------------------
                    // set in myAsDes the edges generated by limitations of the
                    // initial square if the type is correct (The edges that will
                    // disappear are not set)
                    // --------------------------------------------------------------
                    TopoDS_Edge CurE = TopoDS.Edge(expe.Current());
                    NCollection_List<BRepOffset_Interval> L = myAnalyse.Type(CurE);
                    if (!L.IsEmpty() && L.First().Type() != OT)
                    {
                        // a priori doe s not disappear, so it is set
                        TopoDS_Shape aLocalShape = myMapSF.Find(CurF).InternalGenerated(CurE);
                        TopoDS_Edge CurOE = TopoDS.Edge(aLocalShape);
                        //	  const TopoDS_Edge& CurOE =
                        //	    TopoDS::Edge(myMapSF(CurF).Generated(CurE));
                        myAsDes.Add(CurOF, CurOE.Oriented(CurE.Orientation()));
                    }
                    else
                    {
                        NCollection_List<TopoDS_Shape> Lanc = myAnalyse.Ancestors(CurE);
                        if (!myFaces.Contains(Lanc.First()) || !myFaces.Contains(Lanc.Last())
                            || myStopFaces.Contains(Lanc.First()) || myStopFaces.Contains(Lanc.Last()))
                        {
                            TopoDS_Shape aLocalShape = myMapSF.Find(CurF).InternalGenerated(CurE);
                            TopoDS_Edge CurOE = TopoDS.Edge(aLocalShape);
                            //	    const TopoDS_Edge& CurOE =
                            //	      TopoDS::Edge(myMapSF(CurF).Generated(CurE));
                            myAsDes.Add(CurOF, CurOE.Oriented(CurE.Orientation()));
                        }
                    }
                }
                NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anEmptyMap =
                    new(TopTools_ShapeMapHasher.Instance);
                BRepOffset_Inter2d.Compute(myAsDes, CurOF, myEdges, myTol, anEmptyMap, aDMVV, new Message_ProgressRange());
            }
        }

        // ----------------------------------------------------------------
        // It is also required to make 2D intersections with generated tubes
        // (Useful for unwinding)
        // ----------------------------------------------------------------
        // OCCT iterates myMapSF by the shapes' addresses; the port in insertion order.
        foreach (KeyValuePair<TopoDS_Shape, BRepOffset_Offset> anEntry in myMapSF)
        {
            TopoDS_Shape CurS = anEntry.Key;
            if (CurS.ShapeType() == TopAbs_FACE)
            {
                continue;
            }

            TopoDS_Face CurOF = anEntry.Value.InternalFace();

            // no unwinding by tubes on free border.
            if (myStopFaces.Contains(CurS))
            {
                continue;
            }

            LOF.Append(CurOF);

            // --------------------------------------------------------------
            // set in myAsDes the edge restrictions of the square
            // --------------------------------------------------------------
            TopExp_Explorer expe = new(CurOF.Oriented(TopAbs_FORWARD), TopAbs_EDGE);
            for (; expe.More(); expe.Next())
            {
                TopoDS_Edge CurOE = TopoDS.Edge(expe.Current());
                myAsDes.Add(CurOF, CurOE);
            }

            NCollection_DataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> anEmptyMap =
                new(TopTools_ShapeMapHasher.Instance);
            BRepOffset_Inter2d.Compute(myAsDes, CurOF, myEdges, myTol, anEmptyMap, aDMVV, new Message_ProgressRange());
        }
        //
        // fuse vertices on edges stored in AsDes
        BRepAlgo_Image anEmptyImage = new();
        BRepOffset_Inter2d.FuseVertices(aDMVV, myAsDes, anEmptyImage);
        // ------------
        // unwinding
        // ------------
        BRepOffset_MakeLoops MakeLoops = new();
        MakeLoops.Build(LOF, myAsDes, myImageOffset, anEmptyImage, new Message_ProgressRange());

        // ------------------------------------------------------------
        // It is possible to unwind edges at least one ancestor which of
        // is a face of the initial shape, so:
        // the edges generated by intersection tube-tube are missing
        // ------------------------------------------------------------

        // --------------------------------------------------------------
        // Currently set the unwinded surfaces in <myResult>
        // --------------------------------------------------------------
        TopoDS_Compound aResult = new();
        B.MakeCompound(ref aResult);
        myResult = aResult;
        NCollection_List<TopoDS_Shape>.Iterator itLOF = new(LOF);
        for (; itLOF.More(); itLOF.Next())
        {
            TopoDS_Shape CurLOF = itLOF.Value();

            if (!myImageOffset.HasImage(CurLOF))
            {
                continue;
            }

            NCollection_List<TopoDS_Shape> Lim = new();
            myImageOffset.LastImage(CurLOF, Lim);
            NCollection_List<TopoDS_Shape>.Iterator itLim = new(Lim);
            for (; itLim.More(); itLim.Next())
            {
                // If a face is its own image, it is not set
                TopoDS_Shape CurLIM = itLim.Value();
                if (CurLIM.IsSame(CurLOF))
                {
                    break;
                }

                B.Add(myResult, CurLIM);
            }
        }
    }

    //=================================================================================================

    private void ComputeSurfaces()
    {
        // set in myFaces, the faces actually implied in the connection
        myFaces.Clear();

        // construct
        // 1 - Tubes (True Fillets)
        // 2 - Spheres.

        Geom_Surface? GS1 = null, GS2 = null;
        Geom_Curve? GC1 = null, GC2 = null;

        double TolAngle = 2 * Math.Asin(myTol / Math.Abs(myRadius * 0.5));
        BRepOffset_Analyse CenterAnalyse = new(myResult, TolAngle);

        // -----------------------------------------------------
        // Construction of tubes in myResult
        // -----------------------------------------------------
        BRep_Builder B = new();
        TopoDS_Compound aResult = new();
        B.MakeCompound(ref aResult);
        myResult = aResult;

        // --------------------------------------------------------------------
        // Dummy: for construction of spheres:
        // Set in Co the center line, then it there are at least 3
        // center lines sharing the same vertex, Sphere on this vertex.
        // --------------------------------------------------------------------
        TopoDS_Compound Co = new();
        B.MakeCompound(ref Co);

        // --------------------------------------------------------------------
        // Iteration on the edges lines of center
        // and their valid valid part is taken after cut and tube construction.
        // --------------------------------------------------------------------

        // NCollection_Map<TopoDS_Shape, TopTools_ShapeMapHasher>::Iterator ic(myEdges);
        int i;
        for (i = 1; i <= myEdges.Extent(); i++)
        {
            TopoDS_Edge CurE = TopoDS.Edge(myEdges[i]);

            NCollection_List<TopoDS_Shape> L = myAsDes.Ascendant(CurE);
            if (L.Extent() != 2)
            {
                continue;
            }

            // --------------------------------------------------------------
            // F1 and F2 = 2 parallel faces intersecting in CurE.
            // --------------------------------------------------------------
            TopoDS_Face F1 = TopoDS.Face(L.First());
            TopoDS_Face F2 = TopoDS.Face(L.Last());

            // -----------------------------------------------------
            // find the orientation of edges of intersection
            // in the initial faces.
            // -----------------------------------------------------
            NCollection_List<TopoDS_Shape> LD1 = myAsDes.Descendant(F1);
            NCollection_List<TopoDS_Shape> LD2 = myAsDes.Descendant(F2);

            TopAbs_Orientation Orien1 = Orientation(CurE, F1, LD1);
            TopAbs_Orientation Orien2 = Orientation(CurE, F2, LD2);

            // ---------------------------------------------------------
            // Or1 and Or2 : the shapes generators of parallel faces
            // ---------------------------------------------------------
            TopoDS_Shape Or1 = myInitOffsetFace.ImageFrom(F1);
            TopoDS_Shape Or2 = myInitOffsetFace.ImageFrom(F2);

            myFaces.Add(Or1);
            myFaces.Add(Or2);

            TopoDS_Edge OE1 = new(), OE2 = new();
            TopoDS_Face OF1 = new(), OF2 = new();
            TopLoc_Location Loc;
            double f1, l1, f2, l2;

            bool OF1isEdge = false;

            if (Or1.ShapeType() == TopAbs_EDGE)
            {
                OF1isEdge = true;
                OE1 = TopoDS.Edge(Or1);
                GC1 = BRep_Tool.Curve(OE1, out Loc, out f1, out l1);
                GC1 = (Geom_Curve)GC1!.Transformed(Loc.Transformation());
            }
            else if (Or1.ShapeType() == TopAbs_FACE)
            {
                OF1 = TopoDS.Face(Or1);
                GS1 = BRep_Tool.Surface(OF1);
            }

            // ----------------------------------------------------------------
            // If a vertex is used in contact, currently nothing is done
            // and the vertexes are not managed (Intersections with sphere);
            // ----------------------------------------------------------------
            if (OF1.IsNull() && OE1.IsNull())
            {
                continue;
            }

            bool OF2isEdge = false;

            if (Or2.ShapeType() == TopAbs_EDGE)
            {
                OF2isEdge = true;
                OE2 = TopoDS.Edge(Or2);
                GC2 = BRep_Tool.Curve(OE2, out Loc, out f2, out l2);
                GC2 = (Geom_Curve)GC2!.Transformed(Loc.Transformation());
            }
            else if (Or2.ShapeType() == TopAbs_FACE)
            {
                OF2 = TopoDS.Face(Or2);
                GS2 = BRep_Tool.Surface(OF2);
            }
            // ----------------------------------------------------------------
            // If a vertex is used in contact, currently nothing is done
            // and the vertexes are not managed (Intersections with sphere);
            // ----------------------------------------------------------------
            if (OF2.IsNull() && OE2.IsNull())
            {
                continue;
            }

            NCollection_List<TopoDS_Shape> CurL = new();

            if (!myImageOffset.HasImage(CurE))
            { // the tubes are not unwinded
                if (OF1isEdge && OF2isEdge)
                { // if I don't have the image, possibly
                    CurL.Append(CurE); // I'm on intersection tube-tube
                } // See comment on the call to
                else
                { // MakeLoops
                    continue;
                }
            }
            else
            {
                myImageOffset.LastImage(CurE, CurL);
            }

            // ---------------------------------------------------------------
            // CurL = List of edges descending from CurE ( = Cuts of CurE)
            // ---------------------------------------------------------------
            NCollection_List<TopoDS_Shape>.Iterator itl = new(CurL);
            for (; itl.More(); itl.Next())
            {
                TopoDS_Edge CurCutE = TopoDS.Edge(itl.Value());

                Geom2d_Curve? PC1 = BRep_Tool.CurveOnSurface(CurCutE, F1, out f1, out l1);
                Geom2d_Curve? PC2 = BRep_Tool.CurveOnSurface(CurCutE, F2, out f2, out l2);
                if (PC1 is null || PC2 is null)
                {
                    continue;
                }

                TopoDS_Edge E1f, E1l;
                TopoDS_Vertex VfOnE1, VlOnE1, VfOnE2, VlOnE2;
                NCollection_Map<TopoDS_Shape> MapOnV1f = new(TopTools_ShapeMapHasher.Instance);
                NCollection_Map<TopoDS_Shape> MapOnV1l = new(TopTools_ShapeMapHasher.Instance);

                TopExp.Vertices(CurCutE, out TopoDS_Vertex V1f, out TopoDS_Vertex V1l);

                // find if the pipe on the tangent edges are soon created.
                // edges generated by V1f and V1l + Maj MapOnV1f/l
                E1f = FindCreatedEdge(V1f, CurCutE, myMapSF, MapOnV1f, CenterAnalyse, myRadius, myTol);

                E1l = FindCreatedEdge(V1l, CurCutE, myMapSF, MapOnV1l, CenterAnalyse, myRadius, myTol);

                TopoDS_Edge E1, E2;
                if (OF1isEdge)
                {
                    BiTgte_CurveOnEdge ConE = new(CurCutE, OE1);
                    Geom_Curve C = MakeCurve(ConE);
                    gp_Pnt P1 = C.Value(C.FirstParameter());
                    gp_Pnt P2 = C.Value(C.LastParameter());
                    VfOnE1 = FindVertex(P1, MapOnV1f, myTol);
                    if (VfOnE1.IsNull())
                    {
                        VfOnE1 = FindVertex(P1, MapOnV1l, myTol);
                    }
                    VlOnE1 = FindVertex(P2, MapOnV1l, myTol);
                    if (VlOnE1.IsNull())
                    {
                        VlOnE1 = FindVertex(P2, MapOnV1f, myTol);
                    }
                    if (P1.SquareDistance(P2) < myTol * myTol)
                    {
                        // BRepOffset_Offset manages degenerated KPart
                        // It is REQUIRED that C should be a circle with ZERO radius
                        E1 = MakeDegeneratedEdge(C, VfOnE1);
                    }
                    else
                    {
                        E1 = new BRepLib_MakeEdge(C, VfOnE1, VlOnE1).InternalEdge();
                    }
                }
                else
                {
                    gp_Pnt2d P2d;
                    P2d = PC1.Value(f1);
                    gp_Pnt P1 = GS1!.Value(P2d.X(), P2d.Y());
                    P2d = PC1.Value(l1);
                    gp_Pnt P2 = GS1.Value(P2d.X(), P2d.Y());
                    VfOnE1 = FindVertex(P1, MapOnV1f, myTol);
                    VlOnE1 = FindVertex(P2, MapOnV1l, myTol);
                    BRepLib_MakeEdge MKE = new(PC1, GS1, VfOnE1, VlOnE1, f1, l1);
                    if (MKE.IsDone())
                    {
                        E1 = MKE.InternalEdge();
                    }
                    else
                    {
                        // OCCT writes "Edge Not Done" to std::cout here.
                        E1 = MKE.InternalEdge();
                    }

                    KPartCurve3d(E1, PC1, GS1);
                }

                if (OF2isEdge)
                {
                    BiTgte_CurveOnEdge ConE = new(CurCutE, OE2);
                    Geom_Curve C = MakeCurve(ConE);
                    gp_Pnt P1 = C.Value(C.FirstParameter());
                    gp_Pnt P2 = C.Value(C.LastParameter());
                    VfOnE2 = FindVertex(P1, MapOnV1f, myTol);
                    if (VfOnE2.IsNull())
                    {
                        VfOnE2 = FindVertex(P1, MapOnV1l, myTol);
                    }
                    VlOnE2 = FindVertex(P2, MapOnV1l, myTol);
                    if (VlOnE2.IsNull())
                    {
                        VlOnE2 = FindVertex(P2, MapOnV1f, myTol);
                    }
                    if (P1.SquareDistance(P2) < myTol * myTol)
                    {
                        // BRepOffset_Offset manages degenerated KParts
                        // It is REQUIRED that C should be a circle with ZERO radius
                        E2 = MakeDegeneratedEdge(C, VfOnE2);
                    }
                    else
                    {
                        E2 = new BRepLib_MakeEdge(C, VfOnE2, VlOnE2).InternalEdge();
                    }
                }
                else
                {
                    gp_Pnt2d P2d;
                    P2d = PC2.Value(f2);
                    gp_Pnt P1 = GS2!.Value(P2d.X(), P2d.Y());
                    P2d = PC2.Value(l2);
                    gp_Pnt P2 = GS2.Value(P2d.X(), P2d.Y());
                    VfOnE2 = FindVertex(P1, MapOnV1f, myTol);
                    VlOnE2 = FindVertex(P2, MapOnV1l, myTol);
                    BRepLib_MakeEdge MKE = new(PC2, GS2, VfOnE2, VlOnE2, f2, l2);
                    if (MKE.IsDone())
                    {
                        E2 = MKE.InternalEdge();
                    }
                    else
                    {
                        // OCCT writes "edge not Done" to std::cout here.
                        E2 = MKE.InternalEdge();
                    }
                    KPartCurve3d(E2, PC2, GS2);
                }
                // Increment of the Map of Created if reconstruction of the Shape is required
                if (myBuildShape)
                {
                    bindCreated(CurCutE);

                    bindCreated(CurCutE, Or1);
                    created(CurCutE, Or1).Append(E1);

                    bindCreated(CurCutE, Or2);
                    created(CurCutE, Or2).Append(E2);
                }

                // ----------------------------------------------------------
                // try to init E1f, E1l, if not found with Analysis.
                // Should happen only if the THEORETICALLY tangent edges
                // are not actually tangent ( Cf: Approximation of lines
                // of intersection that add noise.)
                // ----------------------------------------------------------
                TopoDS_Vertex aVertex1, aVertex2;
                if (E1f.IsNull() && !VfOnE1.IsNull() && !VfOnE2.IsNull())
                {
                    // OCCT iterates the map by the edges' addresses; the port in insertion order.
                    foreach (TopoDS_Shape aKey in MapOnV1f)
                    {
                        TopoDS_Edge E = TopoDS.Edge(aKey);
                        if (!E.IsNull())
                        {
                            TopExp.Vertices(E, out aVertex1, out aVertex2);
                            if ((aVertex1.IsSame(VfOnE1) && aVertex2.IsSame(VfOnE2))
                                || (aVertex2.IsSame(VfOnE1) && aVertex1.IsSame(VfOnE2)))
                            {
                                E1f = E;
                                break;
                            }
                        }
                    }
                }
                if (E1l.IsNull() && !VlOnE1.IsNull() && !VlOnE2.IsNull())
                {
                    foreach (TopoDS_Shape aKey in MapOnV1l)
                    {
                        TopoDS_Edge E = TopoDS.Edge(aKey);
                        if (!E.IsNull())
                        {
                            TopExp.Vertices(E, out aVertex1, out aVertex2);
                            if ((aVertex1.IsSame(VlOnE1) && aVertex2.IsSame(VlOnE2))
                                || (aVertex2.IsSame(VlOnE1) && aVertex1.IsSame(VlOnE2)))
                            {
                                E1l = E;
                                break;
                            }
                        }
                    }
                }

                E1.Orientation(Orien1);
                E2.Orientation(Orien2);

                BRepOffset_Offset AnOffset = new(CurCutE, E1, E2, -myRadius, E1f, E1l, myNubs, myTol, GeomAbs_C2);
                myMapSF.Bind(CurCutE, AnOffset);
                myCenters.Add(CurCutE);
                B.Add(Co, CurCutE);

                TopoDS_Face Tuyo = AnOffset.InternalFace();
                B.Add(myResult, Tuyo);

                if (myBuildShape)
                {
                    // method based ONLY on the construction of fillet:
                    // the first edge of the tube is exactly on Shape1.
                    GeomAPI_ProjectPointOnCurve Projector = new();
                    TopExp_Explorer exp = new(Tuyo, TopAbs_EDGE);
                    TopoDS_Vertex V1, V2;
                    if (OF1isEdge)
                    { // Update CutEdges.
                        TopoDS_Edge EOnF1 = TopoDS.Edge(exp.Current());
                        TopExp.Vertices(EOnF1, out V1, out V2);

                        gp_Pnt P1 = BRep_Tool.Pnt(V1);
                        Projector.Init(P1, GC1!);
                        double U1 = Projector.LowerDistanceParameter();

                        gp_Pnt P2 = BRep_Tool.Pnt(V2);
                        Projector.Init(P2, GC1!);
                        double U2 = Projector.LowerDistanceParameter();

                        TopoDS_Shape aLocalShape = V1.Oriented(TopAbs_INTERNAL);
                        B.UpdateVertex(TopoDS.Vertex(aLocalShape), U1, TopoDS.Edge(Or1), myTol);
                        aLocalShape = V2.Oriented(TopAbs_INTERNAL);
                        B.UpdateVertex(TopoDS.Vertex(aLocalShape), U2, TopoDS.Edge(Or1), myTol);
                        //	  B.UpdateVertex(TopoDS::Vertex(V1.Oriented(TopAbs_INTERNAL)),U1,
                        //			 TopoDS::Edge(Or1),myTol);
                        //	  B.UpdateVertex(TopoDS::Vertex(V2.Oriented(TopAbs_INTERNAL)),U2,
                        //			 TopoDS::Edge(Or1),myTol);

                        if (!myCutEdges.IsBound(Or1))
                        {
                            myCutEdges.Bind(Or1, new NCollection_List<TopoDS_Shape>());
                        }
                        NCollection_List<TopoDS_Shape> L1 = myCutEdges.Find(Or1);
                        L1.Append(V1);
                        L1.Append(V2);
                    }
                    if (OF2isEdge)
                    { // Update CutEdges.
                        exp.Next();
                        TopoDS_Edge EOnF2 = TopoDS.Edge(exp.Current());
                        TopExp.Vertices(EOnF2, out V1, out V2);

                        gp_Pnt P1 = BRep_Tool.Pnt(V1);
                        Projector.Init(P1, GC2!);
                        double U1 = Projector.LowerDistanceParameter();

                        gp_Pnt P2 = BRep_Tool.Pnt(V2);
                        Projector.Init(P2, GC2!);
                        double U2 = Projector.LowerDistanceParameter();

                        TopoDS_Shape aLocalShape = V1.Oriented(TopAbs_INTERNAL);
                        B.UpdateVertex(TopoDS.Vertex(aLocalShape), U1, TopoDS.Edge(Or2), myTol);
                        aLocalShape = V2.Oriented(TopAbs_INTERNAL);
                        B.UpdateVertex(TopoDS.Vertex(aLocalShape), U2, TopoDS.Edge(Or2), myTol);
                        //	  B.UpdateVertex(TopoDS::Vertex(V1.Oriented(TopAbs_INTERNAL)),U1,
                        //			 TopoDS::Edge(Or2),myTol);
                        //	  B.UpdateVertex(TopoDS::Vertex(V2.Oriented(TopAbs_INTERNAL)),U2,
                        //			 TopoDS::Edge(Or2),myTol);

                        if (!myCutEdges.IsBound(Or2))
                        {
                            myCutEdges.Bind(Or2, new NCollection_List<TopoDS_Shape>());
                        }
                        NCollection_List<TopoDS_Shape> L2 = myCutEdges.Find(Or2);
                        L2.Append(V1);
                        L2.Append(V2);
                    }
                }
            }
        }

        // ---------------------------------------------------
        // Construction of spheres,
        // if enough tubes arrive at the vertex
        // ---------------------------------------------------
        NCollection_IndexedDataMap<TopoDS_Shape, NCollection_List<TopoDS_Shape>> Map = new(TopTools_ShapeMapHasher.Instance);
        TopExp.MapShapesAndAncestors(Co, TopAbs_VERTEX, TopAbs_EDGE, Map);

        for (int j = 1; j <= Map.Extent(); j++)
        {
            TopoDS_Vertex V = TopoDS.Vertex(Map.FindKey(j));
            if (Map[j].Extent() != 3)
            {
                continue;
            }

            NCollection_List<TopoDS_Shape> LOE = new();
            NCollection_List<TopoDS_Shape>.Iterator it = new();

            for (it.Initialize(Map[j]); it.More(); it.Next())
            {
                bool Reverse = true;
                if (Reverse)
                {
                    LOE.Append(myMapSF.Find(it.Value()).InternalGenerated(V).Reversed());
                }
                else
                {
                    LOE.Append(myMapSF.Find(it.Value()).InternalGenerated(V));
                }
            }

            BRepOffset_Offset OFT = new(V, LOE, -myRadius, myNubs, myTol, GeomAbs_C2);
            myMapSF.Bind(V, OFT);
            myCenters.Add(V);

            B.Add(myResult, OFT.InternalFace());
        }
    }

    //=================================================================================================

    private void ComputeShape()
    {
        // Find in the initial Shapel:
        //  - untouched Faces
        //  - generated tubes
        //  - the faces neighbors of tubes that should be reconstructed preserving sharing.

        // For Debug : Visualize edges of the initial shape that should be reconstructed.
        // end debug

        BRep_Builder B = new();

        // Maj of the Map of created.
        // Update edges that do not change in the resulting shape
        // i.e. invariant edges in the unwinding.
        TopExp_Explorer exp = new(myShape, TopAbs_FACE);
        // int nbe = 1;
        for (; exp.More(); exp.Next())
        {
            TopoDS_Face CurF = TopoDS.Face(exp.Current());

            if (!myFaces.Contains(CurF))
            {
                continue; // so the face is not touched
            }

            // so the faces are unwinded
            if (!myMapSF.IsBound(CurF))
            {
                continue; // inverted or degenerated
            }

            BRepOffset_Offset Offset = myMapSF.Find(CurF);
            TopoDS_Face CurOF = myMapSF.Find(CurF).InternalFace();

            if (!myImageOffset.HasImage(CurOF))
            { // face disappears in unwinding
                continue;
            }

            TopExp_Explorer exp2 = new(CurF, TopAbs_EDGE);
            for (; exp2.More(); exp2.Next())
            {
                TopoDS_Edge CurE = TopoDS.Edge(exp2.Current());
                TopoDS_Shape aLocalShape = Offset.InternalGenerated(CurE);
                TopoDS_Edge CurOE = TopoDS.Edge(aLocalShape);
                //      const TopoDS_Edge& CurOE = TopoDS::Edge(Offset.Generated(CurE));

                if (!myImageOffset.HasImage(CurOE))
                {
                    continue;
                }
                // CurOE disappears

                TopoDS_Edge ImE = TopoDS.Edge(myImageOffset.Image(CurOE).First());
                if (ImE.IsSame(CurOE))
                {
                    bindCreated(CurOE);
                    bindCreated(CurOE, CurF);
                    created(CurOE, CurF).Append(CurE);
                }
            }
        }

        // The connected faces are already in myResult.
        // So it is necessary to add faces:
        //    - non-touched (so not in myFaces)
        //    - issuing from the unwinding (non degenerated, non inverted, non disappeared)
        exp.Init(myShape, TopAbs_FACE);
        for (; exp.More(); exp.Next())
        {
            TopoDS_Face CurF = TopoDS.Face(exp.Current());

            if (!myFaces.Contains(CurF))
            {
                // so the face is not touched
                B.Add(myResult, CurF);
            }
            else
            { // so the faces are unwindeds

                if (!myMapSF.IsBound(CurF))
                {
                    continue; // inverted or degenerated
                }

                TopoDS_Face CurOF = myMapSF.Find(CurF).InternalFace();

                if (!myImageOffset.HasImage(CurOF))
                { // face disappears in unwinding
                    continue;
                }

                // List of faces generated by a face in the unwinding
                NCollection_List<TopoDS_Shape> Lim = new();
                myImageOffset.LastImage(CurOF, Lim);
                NCollection_List<TopoDS_Shape>.Iterator itLim = new(Lim);
                for (; itLim.More(); itLim.Next())
                {
                    // DeboucFace = offset Face unwinded in "Debouc".
                    TopoDS_Face DeboucFace = TopoDS.Face(itLim.Value());

                    Geom_Surface S = BRep_Tool.Surface(CurF, out TopLoc_Location L)!;

                    TopoDS_Face NewF = new();
                    B.MakeFace(ref NewF);
                    B.UpdateFace(NewF, S, L, BRep_Tool.Tolerance(CurF));

                    NCollection_DataMap<TopoDS_Shape, TopoDS_Shape> MapSS = new(TopTools_ShapeMapHasher.Instance);

                    TopoDS_Shape aLocalShape = DeboucFace.Oriented(TopAbs_FORWARD);
                    TopoDS_Face Face = TopoDS.Face(aLocalShape);
                    //	const TopoDS_Face& Face =
                    //	  TopoDS::Face(DeboucFace.Oriented(TopAbs_FORWARD));
                    TopExp_Explorer exp2 = new(Face, TopAbs_EDGE);
                    for (; exp2.More(); exp2.Next())
                    {
                        TopoDS_Edge E = TopoDS.Edge(exp2.Current());
                        TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                        if (myCreated.IsBound(E))
                        {
                            if (myCreated.Find(E).IsBound(CurF))
                            {
                                TopoDS_Edge OE = TopoDS.Edge(created(E, CurF).First());
                                TopExp.Vertices(OE, out TopoDS_Vertex OV1, out TopoDS_Vertex OV2);
                                if (!myCreated.IsBound(V1))
                                {
                                    bindCreated(V1);
                                }
                                if (!myCreated.IsBound(V2))
                                {
                                    bindCreated(V2);
                                }
                                if (!myCreated.Find(V1).IsBound(CurF))
                                {
                                    bindCreated(V1, CurF);
                                    created(V1, CurF).Append(OV1);
                                }
                                if (!myCreated.Find(V2).IsBound(CurF))
                                {
                                    bindCreated(V2, CurF);
                                    created(V2, CurF).Append(OV2);
                                }
                            }
                        }
                    }

                    TopExp_Explorer expw = new(Face, TopAbs_WIRE);
                    for (; expw.More(); expw.Next())
                    {
                        TopoDS_Wire W = TopoDS.Wire(expw.Current());
                        TopExp_Explorer expe = new(W.Oriented(TopAbs_FORWARD), TopAbs_EDGE);
                        TopoDS_Wire OW = new();
                        B.MakeWire(ref OW);

                        for (; expe.More(); expe.Next())
                        {
                            TopoDS_Edge E = TopoDS.Edge(expe.Current());
                            Geom2d_Curve? C2d = BRep_Tool.CurveOnSurface(E, Face, out double f, out double l);
                            TopoDS_Edge OE = new();
                            if (MapSS.IsBound(E))
                            { // this is an edge of cutting
                                OE = TopoDS.Edge(MapSS.Find(E));
                                TopoDS_Shape aLocalShapeReversedE = E.Reversed();
                                Geom2d_Curve? C2d_1 =
                                    BRep_Tool.CurveOnSurface(TopoDS.Edge(aLocalShapeReversedE), Face, out f, out l);
                                //	      occ::handle<Geom2d_Curve> C2d_1 =
                                //		BRep_Tool::CurveOnSurface(TopoDS::Edge(E.Reversed()),
                                //					  Face,f,l);
                                if (E.Orientation() == TopAbs_FORWARD)
                                {
                                    B.UpdateEdge(OE, C2d, C2d_1, NewF, BRep_Tool.Tolerance(E));
                                }
                                else
                                {
                                    B.UpdateEdge(OE, C2d_1, C2d, NewF, BRep_Tool.Tolerance(E));
                                }
                                B.Range(OE, f, l);
                            }
                            else
                            {
                                // Is there an image in the Map of Created ?
                                if (myCreated.IsBound(E))
                                {
                                    if (myCreated.Find(E).IsBound(CurF))
                                    {
                                        OE = TopoDS.Edge(created(E, CurF).First());
                                    }
                                }
                                else
                                {
                                    B.MakeEdge(ref OE);
                                    TopoDS_Vertex OV1 = new(), OV2 = new();
                                    TopExp.Vertices(E, out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                                    if (myCreated.IsBound(V1) && myCreated.Find(V1).IsBound(CurF))
                                    {
                                        OV1 = TopoDS.Vertex(created(V1, CurF).First());
                                    }
                                    else
                                    {
                                        B.MakeVertex(ref OV1);
                                        gp_Pnt2d P2d = C2d!.Value(BRep_Tool.Parameter(V1, E, Face));
                                        S.D0(P2d.X(), P2d.Y(), out gp_Pnt P);
                                        P.Transform(L.Transformation());
                                        B.UpdateVertex(OV1, P, BRep_Tool.Tolerance(V1));
                                        bindCreated(V1);
                                        bindCreated(V1, CurF);
                                        created(V1, CurF).Append(OV1);
                                    }
                                    if (myCreated.IsBound(V2) && myCreated.Find(V2).IsBound(CurF))
                                    {
                                        OV2 = TopoDS.Vertex(created(V2, CurF).First());
                                    }
                                    else
                                    {
                                        B.MakeVertex(ref OV2);
                                        gp_Pnt2d P2d = C2d!.Value(BRep_Tool.Parameter(V2, E, Face));
                                        S.D0(P2d.X(), P2d.Y(), out gp_Pnt P);
                                        P.Transform(L.Transformation());
                                        B.UpdateVertex(OV2, P, BRep_Tool.Tolerance(V2));
                                        bindCreated(V2);
                                        bindCreated(V2, CurF);
                                        created(V2, CurF).Append(OV2);
                                    }
                                    B.Add(OE, OV1.Oriented(V1.Orientation()));
                                    B.Add(OE, OV2.Oriented(V2.Orientation()));
                                }
                                B.UpdateEdge(OE, C2d, NewF, BRep_Tool.Tolerance(E));
                                B.Range(OE, f, l);
                                //	      ComputeCurve3d(OE,C2d,TheSurf,L,BRep_Tool::Tolerance(E));
                                MapSS.Bind(E, OE);
                            }
                            B.Add(OW, OE.Oriented(E.Orientation()));
                        }
                        B.Add(NewF, OW.Oriented(W.Orientation()));
                    }

                    NewF.Orientation(DeboucFace.Orientation());

                    BRepTools.Update(NewF);
                    B.Add(myResult, NewF);
                }
            }
        }

        // non-regarding the cause, there always remain greeb borders on this Shape, so it is sewn.
        BRepBuilderAPI_Sewing Sew = new(myTol);

        BRepLib.BuildCurves3d(myResult);

        exp.Init(myResult, TopAbs_FACE);
        for (; exp.More(); exp.Next())
        {
            Sew.Add(exp.Current());
        }

        Sew.Perform();

        // SameParameter is done in case Sew does not do it (Detect that the edges
        // are not sameparameter but does nothing.)

        TopoDS_Shape SewedShape = Sew.InternalSewedShape();
        if (!SewedShape.IsNull())
        {
            exp.Init(Sew.InternalSewedShape(), TopAbs_EDGE);
            for (; exp.More(); exp.Next())
            {
                TopoDS_Edge sec = TopoDS.Edge(exp.Current());
                BRepLib.SameParameter(sec, BRep_Tool.Tolerance(sec));
            }
            myResult = SewedShape;
        }
    }

    //=================================================================================================

    private bool Intersect(in TopoDS_Shape Init,
                           in TopoDS_Face Face,
                           NCollection_DataMap<TopoDS_Shape, Bnd_Box> MapSBox,
                           BRepOffset_Offset OF1,
                           BRepOffset_Inter3d Inter)
    {
        bool JenRajoute = false;

        Bnd_Box Box1 = MapSBox.Find(Face);

        // -----------------------------------------------
        // intersection with all already created faces.
        // -----------------------------------------------
        TopoDS_Shape InitShape1 = OF1.InitialShape();
        bool F1surBordLibre = InitShape1.ShapeType() == TopAbs_EDGE && myStopFaces.Contains(InitShape1);

        NCollection_Map<TopoDS_Shape> Done = new(TopTools_ShapeMapHasher.Instance);
        // OCCT iterates myMapSF by the shapes' addresses; the port in insertion order.
        foreach (KeyValuePair<TopoDS_Shape, BRepOffset_Offset> It in myMapSF)
        {
            BRepOffset_Offset OF2 = It.Value;
            TopoDS_Face F2 = OF2.InternalFace();

            if (Box1.IsOut(MapSBox.Find(F2)))
            {
                continue;
            }

            if (Inter.IsDone(Face, F2))
            {
                continue;
            }

            // 2 tubes created on free border are not intersected.
            TopoDS_Shape InitShape2 = OF2.InitialShape();
            bool F2surBordLibre = InitShape2.ShapeType() == TopAbs_EDGE && myStopFaces.Contains(InitShape2);

            if (F1surBordLibre && F2surBordLibre)
            {
                continue;
            }

            // -------------------------------------------------------
            // Tubes are not intersected with neighbor faces.
            // -------------------------------------------------------
            TopoDS_Shape ItKey = It.Key;

            if (Init.ShapeType() == TopAbs_EDGE)
            {
                if (ItKey.ShapeType() == TopAbs_FACE && IsInFace(TopoDS.Edge(Init), TopoDS.Face(ItKey)))
                {
                    continue;
                }
            }

            Inter.FaceInter(Face, F2, myInitOffsetFace);

            // ------------------------------------------
            // an edge of F1 or F2 has been touched ?
            // if yes, add faces in myFaces
            //   ==> JenRajoute = True
            // ------------------------------------------
            NCollection_List<TopoDS_Shape> LInt = new();
            Done.Clear();
            if (myAsDes.HasCommonDescendant(Face, F2, LInt))
            {
                NCollection_List<TopoDS_Shape>.Iterator itl2 = new();
                for (itl2.Initialize(LInt); itl2.More(); itl2.Next())
                {
                    TopoDS_Edge CurE = TopoDS.Edge(itl2.Value());
                    TopExp.Vertices(CurE, out TopoDS_Vertex V1, out TopoDS_Vertex V2);

                    if (Done.Add(V1))
                    {
                        bool IsOnR1 = IsOnRestriction(V1, CurE, Face, out TopoDS_Edge E1);
                        bool IsOnR2 = IsOnRestriction(V1, CurE, F2, out TopoDS_Edge E2);
                        if (IsOnR1)
                        {
                            if (!myStopFaces.Contains(Init))
                            {
                                Add(E1, myEdges, Init, OF1, myAnalyse, IsOnR1 && IsOnR2);
                                JenRajoute = true;
                            }
                        }
                        if (IsOnR2)
                        {
                            if (!myStopFaces.Contains(ItKey))
                            {
                                Add(E2, myEdges, ItKey, OF2, myAnalyse, IsOnR1 && IsOnR2);
                                JenRajoute = true;
                            }
                        }
                    }

                    if (Done.Add(V2))
                    {
                        bool IsOnR1 = IsOnRestriction(V2, CurE, Face, out TopoDS_Edge E1);
                        bool IsOnR2 = IsOnRestriction(V2, CurE, F2, out TopoDS_Edge E2);

                        // If IsOnR1 && IsOnR2,
                        // Leave in the same tps on 2 faces, propagate only on
                        // free borders.
                        // A priori, only facet is closed.
                        if (IsOnR1)
                        {
                            if (!myStopFaces.Contains(Init))
                            {
                                Add(E1, myEdges, Init, OF1, myAnalyse, IsOnR1 && IsOnR2);
                                JenRajoute = true;
                            }
                        }
                        if (IsOnR2)
                        {
                            if (!myStopFaces.Contains(ItKey))
                            {
                                Add(E2, myEdges, ItKey, OF2, myAnalyse, IsOnR1 && IsOnR2);
                                JenRajoute = true;
                            }
                        }
                    }
                }
            }
        }

        return JenRajoute;
    }
}
