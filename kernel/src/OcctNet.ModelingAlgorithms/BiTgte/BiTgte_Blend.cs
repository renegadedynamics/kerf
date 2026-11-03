// Port of: src/ModelingAlgorithms/TKOffset/BiTgte/BiTgte_Blend.hxx
// Port of: src/ModelingAlgorithms/TKOffset/BiTgte/BiTgte_Blend.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Approximation;
using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Conversion;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Occt.BiTgte_ContactType;
using static Occt.BRepOffset_Status;
using static Occt.ChFiDS_TypeOfConcavity;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.Continuity;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;
using static Kerf.Topology.Containment;

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
/// the entry (<see cref="CopyAtEntry"/>, decision 1005's rule), maps the faces and edges given onto the copy,
/// and gives back the caller's unchanged faces in the result (decision 1046).
/// </para>
/// <para>
/// <see cref="Shape"/>, <see cref="Face(int)"/> and <see cref="Face(in Shape)"/> freeze what they return;
/// <see cref="SupportShape1"/> and <see cref="SupportShape2"/> return the caller's shapes where they are unchanged.
/// </para>
/// </remarks>
public sealed class BiTgte_Blend
{
    private double myRadius;
    private double myTol;
    private bool myNubs;
    private Shape myShape = new();
    private Shape myResult = new();
    private bool myBuildShape;
    private readonly NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> myAncestors =
        new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, NCollection_DataMap<Shape, NCollection_List<Shape>>>
        myCreated = new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, NCollection_List<Shape>> myCutEdges =
        new(ShapeHasher.Instance);
    private NCollection_IndexedMap<Shape> myFaces = new(ShapeHasher.Instance);
    private NCollection_IndexedMap<Shape> myEdges = new(ShapeHasher.Instance);
    private NCollection_Map<Shape> myStopFaces = new(ShapeHasher.Instance);
    private readonly BRepOffset_Analyse myAnalyse = new();
    private NCollection_IndexedMap<Shape> myCenters = new(ShapeHasher.Instance);
    private readonly NCollection_DataMap<Shape, BRepOffset_Offset> myMapSF = new(ShapeHasher.Instance);
    private readonly BRepAlgo_Image myInitOffsetFace = new();
    private readonly BRepAlgo_Image myImage = new();
    private readonly BRepAlgo_Image myImageOffset = new();
    private readonly BRepAlgo_AsDes myAsDes;
    private int myNbBranches;
    private NCollection_HArray1<int>? myIndices;
    private bool myDone;

    // OCCT.NET: the copy of the caller's shape the blend works on (decision 1046).
    private CopyAtEntry? myEntry;

    //======================== START STATIC FUNCTIONS ============

    //=================================================================================================

    private static bool IsOnRestriction(in Vertex V, in Edge CurE, in Face F, out Edge E)
    {
        // find if Vertex V of CurE is on a restriction of F.
        // if yes, store this restriction in E.

        // dub - 03 01 97
        // Method somewhat brutal : possible to really optimize by a
        // direct call the SD of intersections -> See LBR

        E = new Edge();
        Curve2D CurC = Tool.CurveOnSurface(CurE, F, out double f, out double l)!;
        double U = Tool.Parameter(V, CurE, F);
        Point2D P = CurC.Value(U);

        Geom2dAPI_ProjectPointOnCurve Proj = new();

        // The tolerance is exaggerated : it is better to construct too many
        // tubes than to miss intersections.
        // double Tol = 100 * BRep_Tool::Tolerance(V);
        double Tol = Tool.Tolerance(V);
        Explorer exp = new(F, ShapeType.Edge);
        for (; exp.More(); exp.Next())
        {
            E = ShapeCasts.Edge(exp.Current());
            Curve2D PC = Tool.CurveOnSurface(E, F, out f, out l)!;
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
    private static void Add(in Edge E,
                            NCollection_IndexedMap<Shape> Map,
                            in Shape S,
                            BRepOffset_Offset OF,
                            BRepOffset_Analyse Analyse,
                            bool WarningSurBordLibre)
    {
        ShapeType Type = S.ShapeType();

        if (Type == global::Kerf.Topology.ShapeType.Face)
        {
            Explorer exp = new(S, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Edge OriE = ShapeCasts.Edge(exp.Current());
                Shape aLocalShape = OF.InternalGenerated(OriE);
                Edge IE = ShapeCasts.Edge(aLocalShape);
                //      const TopoDS_Edge& IE   = TopoDS::Edge(OF.Generated(OriE));
                if (E.IsEqual(IE))
                {
                    if (WarningSurBordLibre)
                    {
                        // It is checked that the border is not free.
                        NCollection_List<Shape> L = Analyse.Ancestors(OriE);
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
        else if (Type == ShapeType.Edge)
        {
            Explorer exp = new(S, ShapeType.Vertex);
            for (; exp.More(); exp.Next())
            {
                Shape aLocalShape = OF.InternalGenerated(exp.Current());
                Edge IE = ShapeCasts.Edge(aLocalShape);
                //      const TopoDS_Edge& IE = TopoDS::Edge(OF.Generated(exp.Current()));
                if (E.IsEqual(IE))
                {
                    NCollection_List<Shape> L = Analyse.Ancestors(exp.Current());
                    NCollection_List<Shape>.Iterator it = new(L);
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

    private static bool IsInFace(in Edge E, in Face F)
    {
        Explorer exp = new(F, ShapeType.Edge);
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

    private static void KPartCurve3d(in Edge Edge, Curve2D Curve, Surface Surf)
    {
        // try to find the particular case
        // if not found call BRepLib::BuildCurve3d

        Placement Loc = new();
        double Tol = global::Kerf.Engine.Foundation.Precision.Confusion();

        // Search only isos on analytical surfaces.
        CurveAdaptor2D C = new(Curve);
        SurfaceAdaptor S = new(Surf);
        CurveKind CTy = C.GetType();
        SurfaceKind STy = S.GetType();
        BrepBuilder TheBuilder = new();

        if (STy != Plane)
        { // if plane buildcurve3d manage KPart
            if (CTy == Line)
            {
                Direction2D D = C.Line().Direction();
                if (D.IsParallel(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                { // Iso V.
                    if (STy == SurfaceKind.Sphere)
                    {
                        Point2D P = C.Line().Location();
                        if (Math.Abs(Math.Abs(P.Y()) - M_PI / 2.0) < global::Kerf.Engine.Foundation.Precision.PConfusion())
                        {
                            TheBuilder.Degenerated(Edge, true);
                        }
                        else
                        {
                            Sphere Sph = S.Sphere();
                            CoordinateSystem Axis = Sph.Position();
                            Circle3D Ci = global::Kerf.Engine.Geometry.ElSLib.SphereVIso(Axis, Sph.Radius(), P.Y());
                            Direction3D DRev = Axis.XDirection().Crossed(Axis.YDirection());
                            Axis3D AxeRev = new(Axis.Location(), DRev);
                            Ci.Rotate(AxeRev, P.X());
                            CircleCurve3D Circle = new(Ci);
                            if (D.IsOpposite(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                            {
                                Circle.Reverse();
                            }
                            TheBuilder.UpdateEdge(Edge, Circle, Loc, Tol);
                        }
                    }
                    else if (STy == SurfaceKind.Cylinder)
                    {
                        Cylinder Cyl = S.Cylinder();
                        Point2D P = C.Line().Location();
                        CoordinateSystem Axis = Cyl.Position();
                        Circle3D Ci = global::Kerf.Engine.Geometry.ElSLib.CylinderVIso(Axis, Cyl.Radius(), P.Y());
                        Direction3D DRev = Axis.XDirection().Crossed(Axis.YDirection());
                        Axis3D AxeRev = new(Axis.Location(), DRev);
                        Ci.Rotate(AxeRev, P.X());
                        CircleCurve3D Circle = new(Ci);
                        if (D.IsOpposite(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            Circle.Reverse();
                        }
                        TheBuilder.UpdateEdge(Edge, Circle, Loc, Tol);
                    }
                    else if (STy == SurfaceKind.Cone)
                    {
                        Cone Cone = S.Cone();
                        Point2D P = C.Line().Location();
                        CoordinateSystem Axis = Cone.Position();
                        Circle3D Ci = global::Kerf.Engine.Geometry.ElSLib.ConeVIso(Axis, Cone.RefRadius(), Cone.SemiAngle(), P.Y());
                        Direction3D DRev = Axis.XDirection().Crossed(Axis.YDirection());
                        Axis3D AxeRev = new(Axis.Location(), DRev);
                        Ci.Rotate(AxeRev, P.X());
                        CircleCurve3D Circle = new(Ci);
                        if (D.IsOpposite(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            Circle.Reverse();
                        }
                        TheBuilder.UpdateEdge(Edge, Circle, Loc, Tol);
                    }
                    else if (STy == SurfaceKind.Torus)
                    {
                        Torus Tore = S.Torus();
                        Point2D P = C.Line().Location();
                        CoordinateSystem Axis = Tore.Position();
                        Circle3D Ci = global::Kerf.Engine.Geometry.ElSLib.TorusVIso(Axis, Tore.MajorRadius(), Tore.MinorRadius(), P.Y());
                        Direction3D DRev = Axis.XDirection().Crossed(Axis.YDirection());
                        Axis3D AxeRev = new(Axis.Location(), DRev);
                        Ci.Rotate(AxeRev, P.X());
                        CircleCurve3D Circle = new(Ci);
                        if (D.IsOpposite(Constants.DX2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            Circle.Reverse();
                        }
                        TheBuilder.UpdateEdge(Edge, Circle, Loc, Tol);
                    }
                }
                else if (D.IsParallel(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                { // Iso U.
                    if (STy == SurfaceKind.Sphere)
                    {
                        Sphere Sph = S.Sphere();
                        Point2D P = C.Line().Location();
                        CoordinateSystem Axis = Sph.Position();
                        // calculate iso 0.
                        Circle3D Ci = global::Kerf.Engine.Geometry.ElSLib.SphereUIso(Axis, Sph.Radius(), 0.0);

                        // set to sameparameter (rotation of the circle - offset from Y)
                        Direction3D DRev = Axis.XDirection().Crossed(Axis.Direction());
                        Axis3D AxeRev = new(Axis.Location(), DRev);
                        Ci.Rotate(AxeRev, P.Y());

                        // transformation by iso U ( = P.X())
                        DRev = Axis.XDirection().Crossed(Axis.YDirection());
                        AxeRev = new Axis3D(Axis.Location(), DRev);
                        Ci.Rotate(AxeRev, P.X());
                        CircleCurve3D Circle = new(Ci);

                        if (D.IsOpposite(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            Circle.Reverse();
                        }
                        TheBuilder.UpdateEdge(Edge, Circle, Loc, Tol);
                    }
                    else if (STy == SurfaceKind.Cylinder)
                    {
                        Cylinder Cyl = S.Cylinder();
                        Point2D P = C.Line().Location();
                        Line3D L = global::Kerf.Engine.Geometry.ElSLib.CylinderUIso(Cyl.Position(), Cyl.Radius(), P.X());
                        Vector3D Tr = new(L.Direction());
                        Tr.Multiply(P.Y());
                        L.Translate(Tr);
                        LineCurve3D Line = new(L);
                        if (D.IsOpposite(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            Line.Reverse();
                        }
                        TheBuilder.UpdateEdge(Edge, Line, Loc, Tol);
                    }
                    else if (STy == SurfaceKind.Cone)
                    {
                        Cone Cone = S.Cone();
                        Point2D P = C.Line().Location();
                        Line3D L = global::Kerf.Engine.Geometry.ElSLib.ConeUIso(Cone.Position(), Cone.RefRadius(), Cone.SemiAngle(), P.X());
                        Vector3D Tr = new(L.Direction());
                        Tr.Multiply(P.Y());
                        L.Translate(Tr);
                        LineCurve3D Line = new(L);
                        if (D.IsOpposite(Constants.DY2d(), global::Kerf.Engine.Foundation.Precision.Angular()))
                        {
                            Line.Reverse();
                        }
                        TheBuilder.UpdateEdge(Edge, Line, Loc, Tol);
                    }
                    else if (STy == SurfaceKind.Torus)
                    {
                    }
                }
            }
        }
        else
        { // Case Plane
            Curve3D C3d = GeomAPI.To3d(Curve, S.Plane());
            TheBuilder.UpdateEdge(Edge, C3d, Loc, Tol);
        }
    }

    //=================================================================================================

    private sealed class MakeCurve_Function : Function
    {
        private readonly BiTgte_CurveOnEdge myCurve;

        // OCCT copies the curve into the function; the port's function only reads it.
        public MakeCurve_Function(BiTgte_CurveOnEdge C)
        {
            myCurve = C;
            _nbPnt = 1;
            _nbPnt2d = 0;
        }

        public override double FirstParameter() => myCurve.FirstParameter();

        public override double LastParameter() => myCurve.LastParameter();

        public override bool Value(double theT, NCollection_Array1<Point2D> thePnt2d, NCollection_Array1<Point3D> thePnt)
        {
            thePnt[1] = myCurve.Value(theT);
            return true;
        }

        public override bool D1(double theT, NCollection_Array1<Vector2D> theVec2d, NCollection_Array1<Vector3D> theVec)
        {
            return false;
        }
    }

    private static Curve3D MakeCurve(BiTgte_CurveOnEdge HC)
    {
        Curve3D C;

        if (HC.GetType() == Circle)
        {
            C = new CircleCurve3D(HC.Circle());
            C = new TrimmedCurve3D(C, HC.FirstParameter(), HC.LastParameter());
        }
        else
        { // the approximation is done
            MakeCurve_Function F = new(HC);
            int Deg1, Deg2;
            Deg1 = Deg2 = 8;
            double Tol = global::Kerf.Engine.Foundation.Precision.Approximation();
            FitAndDivide Fit = new(F, Deg1, Deg2, Tol, Tol, true);
            int i;
            int NbCurves = Fit.MultiCurveCount;
            // it is attempted to make the curve at least C1
            CompBezierCurvesToBSplineCurve Conv = new();

            for (i = 1; i <= NbCurves; i++)
            {
                MultiCurve MC = Fit.Value(i); // Load the Ith Curve
                NCollection_Array1<Point3D> Poles = new(1, MC.Degree() + 1); // Return poles
                MC.Curve(1, Poles);

                Conv.AddCurve(Poles);
            }

            Conv.Perform();

            int NbPoles = Conv.PoleCount;
            int NbKnots = Conv.KnotCount;
            NCollection_Array1<Point3D> NewPoles = new(1, NbPoles);
            NCollection_Array1<double> NewKnots = new(1, NbKnots);
            NCollection_Array1<int> NewMults = new(1, NbKnots);

            Conv.KnotsAndMults(NewKnots, NewMults);
            Conv.Poles(NewPoles);

            global::Kerf.Engine.Geometry.Splines.BSplCLib.Reparametrize(HC.FirstParameter(), HC.LastParameter(), NewKnots);

            C = new BSplineCurve3D(NewPoles, NewKnots, NewMults, Conv.Degree());
        }

        return C;
    }

    //=======================================================================
    // function : Touched
    // purpose  : Only the faces connected with caps are given
    //=======================================================================

    private static void Touched(BRepOffset_Analyse Analyse,
                                NCollection_Map<Shape> StopFaces,
                                in Shape Shape,
                                NCollection_Map<Shape> TouchedByCork)
    {
        // currently nothing is done !!
    }

    //=================================================================================================

    private static Vertex FindVertex(in Point3D P, NCollection_Map<Shape> Map, double Tol)
    {
        BrepBuilder B = new();
        // Find in <Map> a vertex which represent the point <P>.
        double Tol2, Dist;
        Vertex V = new();
        Vertex[] VV = new Vertex[2];
        double TolCarre = Tol * Tol;
        // OCCT iterates the map by the edges' addresses; the port in insertion order.
        foreach (Shape aKey in Map)
        {
            Edge E = ShapeCasts.Edge(aKey);
            if (!E.IsNull)
            {
                Exploration.Vertices(E, out VV[0], out VV[1]);

                for (int i = 0; i < 2; i++)
                {
                    // if OK la Tolerance du Vertex
                    Tol2 = Tool.Tolerance(VV[i]);
                    Tol2 *= Tol2;
                    Point3D P1 = Tool.Pnt(VV[i]);
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

    private static Edge MakeDegeneratedEdge(Curve3D CC, in Vertex VfOnE)
    {
        BrepBuilder B = new();
        double Tol = global::Kerf.Engine.Foundation.Precision.Confusion();
        // kill trimmed curves
        Curve3D C = CC;
        TrimmedCurve3D? CT = C as TrimmedCurve3D;
        while (CT is not null)
        {
            C = CT.BasisCurve();
            CT = C as TrimmedCurve3D;
        }

        Vertex V1 = new(), V2;
        if (VfOnE.IsNull)
        {
            Point3D P = C.Value(C.FirstParameter());
            B.MakeVertex(ref V1, P, Tol);
            V2 = V1;
        }
        else
        {
            V1 = V2 = VfOnE;
        }
        V1.Orientation(Forward);
        V2.Orientation(Reversed);

        Edge E = new();
        B.MakeEdge(ref E, C, Tol);
        B.Add(E, V1);
        B.Add(E, V2);
        //  B.UpdateVertex(V1,C->FirstParameter(),E,Tol);
        //  B.UpdateVertex(V2,C->LastParameter(),E,Tol);
        B.Range(E, CC.FirstParameter(), CC.LastParameter());
        return E;
    }

    //=================================================================================================

    private static Orientation Orientation(in Edge E, in Face F, NCollection_List<Shape> L)
    {
        Orientation Orien = Forward;
        NCollection_List<Shape>.Iterator itld = new();
        for (itld.Initialize(L); itld.More(); itld.Next())
        {
            if (itld.Value().IsSame(E))
            {
                Orien = itld.Value().Orientation();
                break;
            }
        }
        if (F.Orientation() == Reversed)
        {
            Orien = global::Kerf.Engine.Topology.TopAbs.Reverse(Orien);
        }

        return Orien;
    }

    //=================================================================================================

    private static Edge FindCreatedEdge(in Vertex V1,
                                               in Edge E,
                                               NCollection_DataMap<Shape, BRepOffset_Offset> MapSF,
                                               NCollection_Map<Shape> MapOnV,
                                               BRepOffset_Analyse CenterAnalyse,
                                               double Radius,
                                               double Tol)
    {
        Edge E1 = new();
        if (!CenterAnalyse.HasAncestor(V1))
        {
            return E1; // return a Null Shape.
        }

        NCollection_List<Shape> TangE = new();
        CenterAnalyse.TangentEdges(E, V1, TangE);

        NCollection_List<Shape>.Iterator itl = new(TangE);
        bool Find = false;
        for (; itl.More() && !Find; itl.Next())
        {
            Edge ET = ShapeCasts.Edge(itl.Value());
            if (MapSF.IsBound(ET))
            {
                Shape aLocalShape = MapSF.Find(ET).InternalGenerated(V1);
                E1 = ShapeCasts.Edge(aLocalShape);
                //      E1 = TopoDS::Edge(MapSF(ET).Generated(V1));
                MapOnV.Add(E1);
                Find = true;
            }
            else
            {
                // Find the sharing of vertices in case of tangent consecutive 3 edges
                // the second of which is the edge that degenerates the tube.
                Curve3D CET = Tool.Curve(ET, out Placement CLoc, out double ff, out double ll)!;
                if (CET.GetType() == typeof(TrimmedCurve3D))
                {
                    CET = ((TrimmedCurve3D)CET).BasisCurve();
                }
                if (CET is not CircleCurve3D Circ)
                {
                    continue;
                }
                if (Math.Abs(Circ.Radius() - Math.Abs(Radius)) > Tol)
                {
                    continue;
                }

                Exploration.Vertices(ET, out Vertex U1, out Vertex U2);
                if (U1.IsSame(V1))
                {
                    U1 = U2;
                }
                NCollection_List<Shape> Tang2 = new();
                CenterAnalyse.TangentEdges(ET, U1, Tang2);
                NCollection_List<Shape>.Iterator it2 = new(Tang2);
                for (; it2.More(); it2.Next())
                {
                    Edge ET2 = ShapeCasts.Edge(it2.Value());
                    if (MapSF.IsBound(ET2))
                    {
                        Shape aLocalShape = MapSF.Find(ET2).InternalGenerated(U1);
                        MapOnV.Add(ShapeCasts.Edge(aLocalShape));
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
                TangE = new NCollection_List<Shape>(CenterAnalyse.Ancestors(V1));
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
    public BiTgte_Blend(in Shape S, double Radius, double Tol, bool NUBS)
    {
        myAsDes = new BRepAlgo_AsDes();
        Init(S, Radius, Tol, NUBS);
    }

    //=================================================================================================

    /// <summary>Sets the shape, radius, tolerance and surface kind, and clears the rest.</summary>
    public void Init(in Shape S, double Radius, double Tol, bool NUBS)
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
    public void SetStoppingFace(in Face Face)
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
    public void SetFaces(in Face F1, in Face F2)
    {
        myFaces.Add(F1);
        myFaces.Add(F2);
    }

    //=================================================================================================

    /// <summary>Sets an edge of the shape to be rounded.</summary>
    public void SetEdge(in Edge Edge)
    {
        myEdges.Add(Edge);
    }

    //=================================================================================================

    /// <summary>
    /// Computes the blending surfaces, and the resulting shape if <paramref name="BuildShape"/>.
    /// </summary>
    /// <exception cref="FailureException">The sewing of the shape failed.</exception>
    public void Perform(bool BuildShape = true)
    {
        myBuildShape = BuildShape;

        // OCCT.NET: the work is done on a copy of a frozen shape, the faces and edges given mapped onto it (decision
        // 9403).
        myEntry = new CopyAtEntry(myShape);
        if (myEntry.IsCopied)
        {
            myShape = myEntry.Shape();
            NCollection_IndexedMap<Shape> aFaces = new(ShapeHasher.Instance);
            for (int k = 1; k <= myFaces.Extent(); k++)
            {
                aFaces.Add(myEntry.Copied(myFaces[k]));
            }
            myFaces = aFaces;
            NCollection_IndexedMap<Shape> anEdges = new(ShapeHasher.Instance);
            for (int k = 1; k <= myEdges.Extent(); k++)
            {
                anEdges.Add(myEntry.Copied(myEdges[k]));
            }
            myEdges = anEdges;
            NCollection_Map<Shape> aStops = new(ShapeHasher.Instance);
            foreach (Shape aStop in myStopFaces)
            {
                aStops.Add(myEntry.Copied(aStop));
            }
            myStopFaces = aStops;
        }

        // Try cutting to avoid tubes on free borders
        // that are not actually free.
        BRepBuilderAPI_Sewing Sew = new(myTol);
        BRepLib.BuildCurves3d(myShape);
        Explorer expf = new(myShape, global::Kerf.Topology.ShapeType.Face);
        for (; expf.More(); expf.Next())
        {
            Sew.Add(expf.Current());
        }
        Sew.Perform();
        Shape SewedShape = Sew.InternalSewedShape();
        if (SewedShape.IsNull)
        {
            throw new FailureException("Sewing aux fraises");
        }

        // Check if the sewing modified the orientation.
        expf.Init(myShape, global::Kerf.Topology.ShapeType.Face);
        Face FaceRef = ShapeCasts.Face(expf.Current());
        Orientation OriRef = FaceRef.Orientation();
        if (Sew.IsModified(FaceRef))
        {
            FaceRef = ShapeCasts.Face(Sew.InternalModified(FaceRef));
        }
        expf.Init(SewedShape, global::Kerf.Topology.ShapeType.Face);
        for (; expf.More(); expf.Next())
        {
            Face FF = ShapeCasts.Face(expf.Current());
            if (FaceRef.IsSame(FF) && (FF.Orientation() != OriRef))
            {
                SewedShape.Reverse();
                break;
            }
        }

        // Make SameParameter if Sew does not do it (Detect that edges
        // are not sameparameter but it does nothing.)
        expf.Init(SewedShape, ShapeType.Edge);
        for (; expf.More(); expf.Next())
        {
            Edge sec = ShapeCasts.Edge(expf.Current());
            BRepLib.SameParameter(sec, Tool.Tolerance(sec));
        }

        Exploration.MapShapesAndAncestors(SewedShape, ShapeType.Edge, global::Kerf.Topology.ShapeType.Face, myAncestors);

        // Extend myFaces with the faces of the sewed shape.
        expf.Init(myShape, global::Kerf.Topology.ShapeType.Face);
        for (; expf.More(); expf.Next())
        {
            Shape F = expf.Current();
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
        Explorer exp = new(myShape, global::Kerf.Topology.ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            Shape F = exp.Current();
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
        BRepLib.BuildCurves3d(myResult, global::Kerf.Engine.Foundation.Precision.Confusion());

        // OCCT.NET: the caller's faces the blend left alone stand in the result again.
        myResult = myEntry.Finish(myResult);

        myDone = true;
    }

    //=================================================================================================

    /// <summary>True if <see cref="Perform"/> was run.</summary>
    public bool IsDone() => myDone;

    //=================================================================================================

    /// <summary>The result (frozen).</summary>
    public Shape Shape()
    {
        myResult.TShape()?.FreezeGraph();
        return myResult;
    }

    /// <summary><see cref="Shape"/> without freezing, for ported code.</summary>
    internal Shape InternalShape() => myResult;

    //=================================================================================================

    /// <summary>The number of blending surfaces.</summary>
    public int NbSurfaces() => myCenters.Extent();

    //=================================================================================================

    /// <summary>The blending surface of index <paramref name="Index"/>.</summary>
    public Surface? Surface(int Index) => Surface(myCenters[Index]);

    //=================================================================================================

    /// <summary>The blending face of index <paramref name="Index"/> (frozen).</summary>
    public Face Face(int Index) => Face(myCenters[Index]);

    //=================================================================================================

    /// <summary>Sets in <paramref name="LC"/> all the center lines.</summary>
    public void CenterLines(NCollection_List<Shape> LC)
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
    public Surface? Surface(in Shape CenterLine)
    {
        Face F = myMapSF.Find(CenterLine).InternalFace();
        Surface? S = Tool.Surface(F);
        S?.Freeze();
        return S;
    }

    //=================================================================================================

    /// <summary>The face generated by the center line <paramref name="CenterLine"/> (frozen).</summary>
    /// <exception cref="DomainErrorException">The center line generated no face.</exception>
    public Face Face(in Shape CenterLine)
    {
        if (!myMapSF.IsBound(CenterLine))
        {
            throw new DomainErrorException("BiTgte_Blend::Face");
        }

        Face aFace = myMapSF.Find(CenterLine).InternalFace();
        aFace.TShape()?.FreezeGraph();
        return aFace;
    }

    //=================================================================================================

    /// <summary>The type of contact of the blending surface of index <paramref name="Index"/>.</summary>
    public BiTgte_ContactType ContactType(int Index)
    {
        Shape S1 = SupportShape1(Index);
        Shape S2 = SupportShape2(Index);

        ShapeType Type1 = S1.ShapeType();
        ShapeType Type2 = S2.ShapeType();

        if (Type2 < Type1)
        {
            ShapeType Dummy = Type1;
            Type1 = Type2;
            Type2 = Dummy;
        }
        BiTgte_ContactType Type = BiTgte_VertexVertex;

        switch (Type1)
        {
            case ShapeType.Vertex:
                switch (Type2)
                {
                    case ShapeType.Vertex:
                        Type = BiTgte_VertexVertex;
                        break;
                    case ShapeType.Edge:
                        Type = BiTgte_EdgeVertex;
                        break;
                    case global::Kerf.Topology.ShapeType.Face:
                        Type = BiTgte_FaceVertex;
                        break;
                    default:
                        break;
                }
                break;

            case ShapeType.Edge:
                switch (Type2)
                {
                    case ShapeType.Edge:
                        Type = BiTgte_EdgeEdge;
                        break;
                    case global::Kerf.Topology.ShapeType.Face:
                        Type = BiTgte_FaceEdge;
                        break;
                    default:
                        break;
                }
                break;

            case global::Kerf.Topology.ShapeType.Face:
                switch (Type2)
                {
                    case global::Kerf.Topology.ShapeType.Face:
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
    public Shape SupportShape1(int Index)
    {
        Edge CurE = ShapeCasts.Edge(myCenters[Index]);

        NCollection_List<Shape> L = myAsDes.Ascendant(CurE);

        // --------------------------------------------------------------
        // F1 and F2 = 2 parallel faces intersecting at CurE.
        // --------------------------------------------------------------
        Face F1 = ShapeCasts.Face(L.First());
        Shape Or1 = myInitOffsetFace.ImageFrom(F1);
        return published(Or1);
    }

    //=================================================================================================

    /// <summary>The second support shape of the blending surface of index <paramref name="Index"/>.</summary>
    public Shape SupportShape2(int Index)
    {
        Edge CurE = ShapeCasts.Edge(myCenters[Index]);

        NCollection_List<Shape> L = myAsDes.Ascendant(CurE);

        // --------------------------------------------------------------
        // F1 and F2 = 2 parallel faces intersecting at CurE.
        // --------------------------------------------------------------
        Face F2 = ShapeCasts.Face(L.Last());
        Shape Or2 = myInitOffsetFace.ImageFrom(F2);
        return published(Or2);
    }

    // OCCT.NET: a shape of the working copy as the caller sees it: its own where unchanged, frozen.
    private Shape published(in Shape theShape)
    {
        Shape aShape = myEntry is not null && myDone ? myEntry.Final(theShape) : theShape;
        aShape.TShape()?.FreezeGraph();
        return aShape;
    }

    //=================================================================================================

    /// <summary>The 3D curve of the blending surface of index <paramref name="Index"/> on its first support.</summary>
    public Curve3D? CurveOnShape1(int Index)
    {
        Edge CurE = ShapeCasts.Edge(myCenters[Index]);
        Shape F = myMapSF.Find(CurE).InternalFace();

        // somewhat brutal method based ONLY on the construction of the fillet:
        // the first edge of the tube is exactly the edge on Shape1.

        Explorer exp = new(F, ShapeType.Edge);
        Edge E = ShapeCasts.Edge(exp.Current());
        Curve3D? C = null;
        if (!Tool.Degenerated(E))
        {
            C = Tool.Curve(E, out double f, out double l);
            C = new TrimmedCurve3D(C!, f, l);
        }
        return C;
    }

    //=================================================================================================

    /// <summary>The 3D curve of the blending surface of index <paramref name="Index"/> on its second support.</summary>
    public Curve3D? CurveOnShape2(int Index)
    {
        Edge CurE = ShapeCasts.Edge(myCenters[Index]);
        Shape F = myMapSF.Find(CurE).InternalFace();

        // somewhat brutal method based ONLY on the construction of the fillet:
        // the first edge of the tube is exactly the edge on Shape2.

        Explorer exp = new(F, ShapeType.Edge);
        exp.Next();
        Edge E = ShapeCasts.Edge(exp.Current());
        Curve3D? C = null;
        if (!Tool.Degenerated(E))
        {
            C = Tool.Curve(E, out double f, out double l);
            C = new TrimmedCurve3D(C!, f, l);
        }
        return C;
    }

    //=================================================================================================

    /// <summary>Always null in OCCT.</summary>
    public Curve2D? PCurveOnFace1(int Index) => null;

    /// <summary>Always null in OCCT.</summary>
    public Curve2D? PCurve1OnFillet(int Index) => null;

    /// <summary>Always null in OCCT.</summary>
    public Curve2D? PCurveOnFace2(int Index) => null;

    /// <summary>Always null in OCCT.</summary>
    public Curve2D? PCurve2OnFillet(int Index) => null;

    //=================================================================================================

    /// <summary>The number of branches (sets of connected blending faces); reorders the center lines by branch.</summary>
    public int NbBranches()
    {
        if (myNbBranches != -1)
        {
            return myNbBranches;
        }

        // else, compute the Branches.
        Quilt Glue = new();

        int NbFaces = myCenters.Extent();

        if (NbFaces == 0)
        {
            return 0;
        }

        int i;
        for (i = 1; i <= NbFaces; i++)
        {
            Shape CenterLine = myCenters[i];
            Glue.Add(myMapSF.Find(CenterLine).InternalFace());
        }

        Shape Shells = Glue.Shells();

        // Reorder Map myCenters.
        // The method is brutal and unpolished,
        // it is possible to refine it.
        myNbBranches = 0;
        NCollection_IndexedMap<Shape> tmpMap = new(ShapeHasher.Instance);

        Explorer exp = new(Shells, ShapeType.Shell);
        for (; exp.More(); exp.Next())
        {
            myNbBranches++;
        }

        myIndices = new NCollection_HArray1<int>(1, myNbBranches + 1);

        myIndices.SetValue(1, 0);
        int Count = 0;
        int Index = 2;

        exp.Init(Shells, ShapeType.Shell);
        for (; exp.More(); exp.Next())
        {
            // CurS = the current Shell.
            Shape CurS = exp.Current();

            Explorer exp2 = new(CurS, global::Kerf.Topology.ShapeType.Face);
            for (; exp2.More(); exp2.Next())
            {
                // CurF = the current face of the current Shell.
                Shape CurF = exp2.Current();

                for (i = 1; i <= NbFaces; i++)
                {
                    Shape Center = myCenters[i];
                    Shape Rakk = myMapSF.Find(Center).InternalFace();
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
    private NCollection_List<Shape> created(in Shape theKey1, in Shape theKey2) =>
        myCreated.Find(theKey1).Find(theKey2);

    private void bindCreated(in Shape theKey) =>
        myCreated.Bind(theKey, new NCollection_DataMap<Shape, NCollection_List<Shape>>(ShapeHasher.Instance));

    private void bindCreated(in Shape theKey1, in Shape theKey2) =>
        myCreated.Find(theKey1).Bind(theKey2, new NCollection_List<Shape>());

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
        NCollection_Map<Shape> TouchedByCork = new(ShapeHasher.Instance);
        Touched(myAnalyse, myStopFaces, myShape, TouchedByCork);

        // -----------------------
        // init of the intersector
        // -----------------------
        Containment Side = In;
        if (myRadius < 0.0)
        {
            Side = Out;
        }
        BRepOffset_Inter3d Inter = new(myAsDes, Side, myTol);

        NCollection_DataMap<Shape, Box> MapSBox = new(ShapeHasher.Instance);
        NCollection_Map<Shape> Done = new(ShapeHasher.Instance);

        BrepBuilder B = new();
        Compound Co = new(); // to only know on which edges the tubes are made
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

            NCollection_DataMap<Shape, Shape> EdgeTgt = new(ShapeHasher.Instance);

            while (!Fini)
            {
                // -------------------------------------------------
                // locate in myFaces the Faces connected to myEdges.
                // -------------------------------------------------
                Fini = true;
                // for (it.Initialize(myEdges); it.More(); it.Next()) {
                for (i = 1; i <= myEdges.Extent(); i++)
                {
                    Edge E = ShapeCasts.Edge(myEdges[i]);
                    if (Tool.Degenerated(E))
                    {
                        continue;
                    }

                    NCollection_List<Shape> L = myAncestors.FindFromKey(E);
                    if (L.Extent() == 1)
                    {
                        // So this is a free border onwhich the ball should roll.
                        myFaces.Add(E);

                        // set in myStopFaces to not propagate the tube on free border.
                        myStopFaces.Add(E);
                    }
                    else
                    {
                        NCollection_List<Shape>.Iterator itl = new();
                        for (itl.Initialize(L); itl.More(); itl.Next())
                        {
                            Shape Sh = itl.Value();
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
                    Shape AS = myFaces[i];
                    if (myMapSF.IsBound(AS))
                    {
                        continue;
                    }

                    BRepOffset_Offset OF1 = new();
                    Face BigF = new();

                    if (AS.ShapeType() == global::Kerf.Topology.ShapeType.Face)
                    {
                        Face F = ShapeCasts.Face(myFaces[i]);
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
                        OF1.Init(ShapeCasts.Edge(AS), myRadius);
                    }

                    // ------------------------------------
                    // Increment the map of created tangents
                    // ------------------------------------
                    NCollection_List<Shape> Let = new();
                    if (AS.ShapeType() == global::Kerf.Topology.ShapeType.Face)
                    {
                        myAnalyse.Edges(ShapeCasts.Face(AS), ChFiDS_Tangential, Let);
                    }
                    NCollection_List<Shape>.Iterator itlet = new(Let);

                    for (; itlet.More(); itlet.Next())
                    {
                        Edge Cur = ShapeCasts.Edge(itlet.Value());
                        if (!EdgeTgt.IsBound(Cur))
                        {
                            Shape aLocalShape = OF1.InternalGenerated(Cur);
                            Edge OTE = ShapeCasts.Edge(aLocalShape);
                            //	    const TopoDS_Edge& OTE = TopoDS::Edge(OF1.Generated(Cur));
                            EdgeTgt.Bind(Cur, OF1.InternalGenerated(Cur));
                            Exploration.Vertices(Cur, out Vertex V1, out Vertex V2);
                            Exploration.Vertices(OTE, out Vertex OV1, out Vertex OV2);
                            NCollection_List<Shape> LE = new();
                            if (!EdgeTgt.IsBound(V1))
                            {
                                myAnalyse.Edges(V1, ChFiDS_Tangential, LE);
                                NCollection_List<Shape> LA = myAnalyse.Ancestors(V1);
                                if (LE.Extent() == LA.Extent())
                                {
                                    EdgeTgt.Bind(V1, OV1);
                                }
                            }
                            if (!EdgeTgt.IsBound(V2))
                            {
                                LE.Clear();
                                myAnalyse.Edges(V2, ChFiDS_Tangential, LE);
                                NCollection_List<Shape> LA = myAnalyse.Ancestors(V2);
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

                    Face F1 = OF1.InternalFace();

                    // increment S D
                    myInitOffsetFace.SetRoot(AS);
                    myInitOffsetFace.Bind(AS, F1);

                    Box Box1 = new();
                    BRepBndLib.Add(F1, ref Box1);
                    MapSBox.Bind(F1, Box1);

                    // ---------------------------------------------
                    // intersection with all already created faces.
                    // ---------------------------------------------
                    Fini = !Intersect(AS, F1, MapSBox, OF1, Inter);

                    if (AS.ShapeType() == global::Kerf.Topology.ShapeType.Face)
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

            NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> Map =
                new(ShapeHasher.Instance);
            Exploration.MapShapesAndAncestors(Co, ShapeType.Edge, global::Kerf.Topology.ShapeType.Face, Map);
            Exploration.MapShapesAndAncestors(Co, ShapeType.Vertex, ShapeType.Edge, Map);

            Explorer exp = new(Co, ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Edge E = ShapeCasts.Edge(exp.Current());
                if (myMapSF.IsBound(E))
                {
                    continue;
                }

                NCollection_List<Shape> Anc = Map.FindFromKey(E);
                if (Anc.Extent() == 2)
                {
                    NCollection_List<BRepOffset_Interval> L = myAnalyse.Type(E);
                    if (!L.IsEmpty() && L.First().Type() == OT1)
                    {
                        Shape aLocalShapeGen = myMapSF.Find(Anc.First()).InternalGenerated(E);
                        Edge EOn1 = ShapeCasts.Edge(aLocalShapeGen);
                        aLocalShapeGen = myMapSF.Find(Anc.Last()).InternalGenerated(E);
                        Edge EOn2 = ShapeCasts.Edge(aLocalShapeGen);
                        //	  TopoDS_Edge EOn1 = TopoDS::Edge(myMapSF(Anc.First()).Generated(E));
                        //	  TopoDS_Edge EOn2 = TopoDS::Edge(myMapSF(Anc.Last()) .Generated(E));
                        // find if exits tangent edges in the original shape
                        Edge E1f = new(), E1l = new();
                        Exploration.Vertices(E, out Vertex V1f, out Vertex V1l);
                        NCollection_List<Shape> TangE = new();
                        myAnalyse.TangentEdges(E, V1f, TangE);
                        // find if the pipe on the tangent edges are soon created.
                        NCollection_List<Shape>.Iterator itl = new(TangE);
                        bool Find = false;
                        for (; itl.More() && !Find; itl.Next())
                        {
                            if (myMapSF.IsBound(itl.Value()))
                            {
                                Shape aLocalShape = myMapSF.Find(itl.Value()).InternalGenerated(V1f);
                                E1f = ShapeCasts.Edge(aLocalShape);
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
                                Shape aLocalShape = myMapSF.Find(itl.Value()).InternalGenerated(V1l);
                                E1l = ShapeCasts.Edge(aLocalShape);
                                //	      E1l  = TopoDS::Edge(myMapSF(itl.Value()).Generated(V1l));
                                Find = true;
                            }
                        }
                        BRepOffset_Offset OF1 = new(E, EOn1, EOn2, myRadius, E1f, E1l);
                        Face F1 = OF1.InternalFace();

                        // maj S D
                        myInitOffsetFace.SetRoot(E);
                        myInitOffsetFace.Bind(E, F1);

                        Box Box1 = new();
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
        myEdges = new NCollection_IndexedMap<Shape>(Inter.NewEdges());

        // -------------------------------------------------------------------
        // now it is necessary to limit edges on the neighbors (otherwise one
        // will go too far and will not be able to construct faces).
        // -------------------------------------------------------------------

        // Proceed with MakeLoops
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> aDMVV =
            new(ShapeHasher.Instance);
        ChFiDS_TypeOfConcavity OT = ChFiDS_Concave;
        if (myRadius < 0.0)
        {
            OT = ChFiDS_Convex;
        }

        NCollection_List<Shape> LOF = new();
        // it.Initialize(myFaces);
        for (i = 1; i <= myFaces.Extent(); i++)
        {
            Shape CurS = myFaces[i];

            // tube on free border, it is undesirable.
            if (myStopFaces.Contains(CurS))
            {
                continue;
            }

            if (!myMapSF.IsBound(CurS))
            {
                continue; // inverted or degenerated
            }

            Face CurOF = myMapSF.Find(CurS).InternalFace();
            LOF.Append(CurOF);

            if (CurS.ShapeType() == global::Kerf.Topology.ShapeType.Face)
            {
                Face CurF = ShapeCasts.Face(CurS);
                Explorer expe = new(CurF.Oriented(Forward), ShapeType.Edge);
                for (; expe.More(); expe.Next())
                {
                    // --------------------------------------------------------------
                    // set in myAsDes the edges generated by limitations of the
                    // initial square if the type is correct (The edges that will
                    // disappear are not set)
                    // --------------------------------------------------------------
                    Edge CurE = ShapeCasts.Edge(expe.Current());
                    NCollection_List<BRepOffset_Interval> L = myAnalyse.Type(CurE);
                    if (!L.IsEmpty() && L.First().Type() != OT)
                    {
                        // a priori doe s not disappear, so it is set
                        Shape aLocalShape = myMapSF.Find(CurF).InternalGenerated(CurE);
                        Edge CurOE = ShapeCasts.Edge(aLocalShape);
                        //	  const TopoDS_Edge& CurOE =
                        //	    TopoDS::Edge(myMapSF(CurF).Generated(CurE));
                        myAsDes.Add(CurOF, CurOE.Oriented(CurE.Orientation()));
                    }
                    else
                    {
                        NCollection_List<Shape> Lanc = myAnalyse.Ancestors(CurE);
                        if (!myFaces.Contains(Lanc.First()) || !myFaces.Contains(Lanc.Last())
                            || myStopFaces.Contains(Lanc.First()) || myStopFaces.Contains(Lanc.Last()))
                        {
                            Shape aLocalShape = myMapSF.Find(CurF).InternalGenerated(CurE);
                            Edge CurOE = ShapeCasts.Edge(aLocalShape);
                            //	    const TopoDS_Edge& CurOE =
                            //	      TopoDS::Edge(myMapSF(CurF).Generated(CurE));
                            myAsDes.Add(CurOF, CurOE.Oriented(CurE.Orientation()));
                        }
                    }
                }
                NCollection_DataMap<Shape, NCollection_List<Shape>> anEmptyMap =
                    new(ShapeHasher.Instance);
                BRepOffset_Inter2d.Compute(myAsDes, CurOF, myEdges, myTol, anEmptyMap, aDMVV, new ProgressRange());
            }
        }

        // ----------------------------------------------------------------
        // It is also required to make 2D intersections with generated tubes
        // (Useful for unwinding)
        // ----------------------------------------------------------------
        // OCCT iterates myMapSF by the shapes' addresses; the port in insertion order.
        foreach (KeyValuePair<Shape, BRepOffset_Offset> anEntry in myMapSF)
        {
            Shape CurS = anEntry.Key;
            if (CurS.ShapeType() == global::Kerf.Topology.ShapeType.Face)
            {
                continue;
            }

            Face CurOF = anEntry.Value.InternalFace();

            // no unwinding by tubes on free border.
            if (myStopFaces.Contains(CurS))
            {
                continue;
            }

            LOF.Append(CurOF);

            // --------------------------------------------------------------
            // set in myAsDes the edge restrictions of the square
            // --------------------------------------------------------------
            Explorer expe = new(CurOF.Oriented(Forward), ShapeType.Edge);
            for (; expe.More(); expe.Next())
            {
                Edge CurOE = ShapeCasts.Edge(expe.Current());
                myAsDes.Add(CurOF, CurOE);
            }

            NCollection_DataMap<Shape, NCollection_List<Shape>> anEmptyMap =
                new(ShapeHasher.Instance);
            BRepOffset_Inter2d.Compute(myAsDes, CurOF, myEdges, myTol, anEmptyMap, aDMVV, new ProgressRange());
        }
        //
        // fuse vertices on edges stored in AsDes
        BRepAlgo_Image anEmptyImage = new();
        BRepOffset_Inter2d.FuseVertices(aDMVV, myAsDes, anEmptyImage);
        // ------------
        // unwinding
        // ------------
        BRepOffset_MakeLoops MakeLoops = new();
        MakeLoops.Build(LOF, myAsDes, myImageOffset, anEmptyImage, new ProgressRange());

        // ------------------------------------------------------------
        // It is possible to unwind edges at least one ancestor which of
        // is a face of the initial shape, so:
        // the edges generated by intersection tube-tube are missing
        // ------------------------------------------------------------

        // --------------------------------------------------------------
        // Currently set the unwinded surfaces in <myResult>
        // --------------------------------------------------------------
        Compound aResult = new();
        B.MakeCompound(ref aResult);
        myResult = aResult;
        NCollection_List<Shape>.Iterator itLOF = new(LOF);
        for (; itLOF.More(); itLOF.Next())
        {
            Shape CurLOF = itLOF.Value();

            if (!myImageOffset.HasImage(CurLOF))
            {
                continue;
            }

            NCollection_List<Shape> Lim = new();
            myImageOffset.LastImage(CurLOF, Lim);
            NCollection_List<Shape>.Iterator itLim = new(Lim);
            for (; itLim.More(); itLim.Next())
            {
                // If a face is its own image, it is not set
                Shape CurLIM = itLim.Value();
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

        Surface? GS1 = null, GS2 = null;
        Curve3D? GC1 = null, GC2 = null;

        double TolAngle = 2 * Math.Asin(myTol / Math.Abs(myRadius * 0.5));
        BRepOffset_Analyse CenterAnalyse = new(myResult, TolAngle);

        // -----------------------------------------------------
        // Construction of tubes in myResult
        // -----------------------------------------------------
        BrepBuilder B = new();
        Compound aResult = new();
        B.MakeCompound(ref aResult);
        myResult = aResult;

        // --------------------------------------------------------------------
        // Dummy: for construction of spheres:
        // Set in Co the center line, then it there are at least 3
        // center lines sharing the same vertex, Sphere on this vertex.
        // --------------------------------------------------------------------
        Compound Co = new();
        B.MakeCompound(ref Co);

        // --------------------------------------------------------------------
        // Iteration on the edges lines of center
        // and their valid valid part is taken after cut and tube construction.
        // --------------------------------------------------------------------

        // NCollection_Map<TopoDS_Shape, TopTools_ShapeMapHasher>::Iterator ic(myEdges);
        int i;
        for (i = 1; i <= myEdges.Extent(); i++)
        {
            Edge CurE = ShapeCasts.Edge(myEdges[i]);

            NCollection_List<Shape> L = myAsDes.Ascendant(CurE);
            if (L.Extent() != 2)
            {
                continue;
            }

            // --------------------------------------------------------------
            // F1 and F2 = 2 parallel faces intersecting in CurE.
            // --------------------------------------------------------------
            Face F1 = ShapeCasts.Face(L.First());
            Face F2 = ShapeCasts.Face(L.Last());

            // -----------------------------------------------------
            // find the orientation of edges of intersection
            // in the initial faces.
            // -----------------------------------------------------
            NCollection_List<Shape> LD1 = myAsDes.Descendant(F1);
            NCollection_List<Shape> LD2 = myAsDes.Descendant(F2);

            Orientation Orien1 = Orientation(CurE, F1, LD1);
            Orientation Orien2 = Orientation(CurE, F2, LD2);

            // ---------------------------------------------------------
            // Or1 and Or2 : the shapes generators of parallel faces
            // ---------------------------------------------------------
            Shape Or1 = myInitOffsetFace.ImageFrom(F1);
            Shape Or2 = myInitOffsetFace.ImageFrom(F2);

            myFaces.Add(Or1);
            myFaces.Add(Or2);

            Edge OE1 = new(), OE2 = new();
            Face OF1 = new(), OF2 = new();
            Placement Loc;
            double f1, l1, f2, l2;

            bool OF1isEdge = false;

            if (Or1.ShapeType() == ShapeType.Edge)
            {
                OF1isEdge = true;
                OE1 = ShapeCasts.Edge(Or1);
                GC1 = Tool.Curve(OE1, out Loc, out f1, out l1);
                GC1 = (Curve3D)GC1!.Transformed(Loc.Transformation());
            }
            else if (Or1.ShapeType() == global::Kerf.Topology.ShapeType.Face)
            {
                OF1 = ShapeCasts.Face(Or1);
                GS1 = Tool.Surface(OF1);
            }

            // ----------------------------------------------------------------
            // If a vertex is used in contact, currently nothing is done
            // and the vertexes are not managed (Intersections with sphere);
            // ----------------------------------------------------------------
            if (OF1.IsNull && OE1.IsNull)
            {
                continue;
            }

            bool OF2isEdge = false;

            if (Or2.ShapeType() == ShapeType.Edge)
            {
                OF2isEdge = true;
                OE2 = ShapeCasts.Edge(Or2);
                GC2 = Tool.Curve(OE2, out Loc, out f2, out l2);
                GC2 = (Curve3D)GC2!.Transformed(Loc.Transformation());
            }
            else if (Or2.ShapeType() == global::Kerf.Topology.ShapeType.Face)
            {
                OF2 = ShapeCasts.Face(Or2);
                GS2 = Tool.Surface(OF2);
            }
            // ----------------------------------------------------------------
            // If a vertex is used in contact, currently nothing is done
            // and the vertexes are not managed (Intersections with sphere);
            // ----------------------------------------------------------------
            if (OF2.IsNull && OE2.IsNull)
            {
                continue;
            }

            NCollection_List<Shape> CurL = new();

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
            NCollection_List<Shape>.Iterator itl = new(CurL);
            for (; itl.More(); itl.Next())
            {
                Edge CurCutE = ShapeCasts.Edge(itl.Value());

                Curve2D? PC1 = Tool.CurveOnSurface(CurCutE, F1, out f1, out l1);
                Curve2D? PC2 = Tool.CurveOnSurface(CurCutE, F2, out f2, out l2);
                if (PC1 is null || PC2 is null)
                {
                    continue;
                }

                Edge E1f, E1l;
                Vertex VfOnE1, VlOnE1, VfOnE2, VlOnE2;
                NCollection_Map<Shape> MapOnV1f = new(ShapeHasher.Instance);
                NCollection_Map<Shape> MapOnV1l = new(ShapeHasher.Instance);

                Exploration.Vertices(CurCutE, out Vertex V1f, out Vertex V1l);

                // find if the pipe on the tangent edges are soon created.
                // edges generated by V1f and V1l + Maj MapOnV1f/l
                E1f = FindCreatedEdge(V1f, CurCutE, myMapSF, MapOnV1f, CenterAnalyse, myRadius, myTol);

                E1l = FindCreatedEdge(V1l, CurCutE, myMapSF, MapOnV1l, CenterAnalyse, myRadius, myTol);

                Edge E1, E2;
                if (OF1isEdge)
                {
                    BiTgte_CurveOnEdge ConE = new(CurCutE, OE1);
                    Curve3D C = MakeCurve(ConE);
                    Point3D P1 = C.Value(C.FirstParameter());
                    Point3D P2 = C.Value(C.LastParameter());
                    VfOnE1 = FindVertex(P1, MapOnV1f, myTol);
                    if (VfOnE1.IsNull)
                    {
                        VfOnE1 = FindVertex(P1, MapOnV1l, myTol);
                    }
                    VlOnE1 = FindVertex(P2, MapOnV1l, myTol);
                    if (VlOnE1.IsNull)
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
                    Point2D P2d;
                    P2d = PC1.Value(f1);
                    Point3D P1 = GS1!.Value(P2d.X(), P2d.Y());
                    P2d = PC1.Value(l1);
                    Point3D P2 = GS1.Value(P2d.X(), P2d.Y());
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
                    Curve3D C = MakeCurve(ConE);
                    Point3D P1 = C.Value(C.FirstParameter());
                    Point3D P2 = C.Value(C.LastParameter());
                    VfOnE2 = FindVertex(P1, MapOnV1f, myTol);
                    if (VfOnE2.IsNull)
                    {
                        VfOnE2 = FindVertex(P1, MapOnV1l, myTol);
                    }
                    VlOnE2 = FindVertex(P2, MapOnV1l, myTol);
                    if (VlOnE2.IsNull)
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
                    Point2D P2d;
                    P2d = PC2.Value(f2);
                    Point3D P1 = GS2!.Value(P2d.X(), P2d.Y());
                    P2d = PC2.Value(l2);
                    Point3D P2 = GS2.Value(P2d.X(), P2d.Y());
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
                Vertex aVertex1, aVertex2;
                if (E1f.IsNull && !VfOnE1.IsNull && !VfOnE2.IsNull)
                {
                    // OCCT iterates the map by the edges' addresses; the port in insertion order.
                    foreach (Shape aKey in MapOnV1f)
                    {
                        Edge E = ShapeCasts.Edge(aKey);
                        if (!E.IsNull)
                        {
                            Exploration.Vertices(E, out aVertex1, out aVertex2);
                            if ((aVertex1.IsSame(VfOnE1) && aVertex2.IsSame(VfOnE2))
                                || (aVertex2.IsSame(VfOnE1) && aVertex1.IsSame(VfOnE2)))
                            {
                                E1f = E;
                                break;
                            }
                        }
                    }
                }
                if (E1l.IsNull && !VlOnE1.IsNull && !VlOnE2.IsNull)
                {
                    foreach (Shape aKey in MapOnV1l)
                    {
                        Edge E = ShapeCasts.Edge(aKey);
                        if (!E.IsNull)
                        {
                            Exploration.Vertices(E, out aVertex1, out aVertex2);
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

                BRepOffset_Offset AnOffset = new(CurCutE, E1, E2, -myRadius, E1f, E1l, myNubs, myTol, C2);
                myMapSF.Bind(CurCutE, AnOffset);
                myCenters.Add(CurCutE);
                B.Add(Co, CurCutE);

                Face Tuyo = AnOffset.InternalFace();
                B.Add(myResult, Tuyo);

                if (myBuildShape)
                {
                    // method based ONLY on the construction of fillet:
                    // the first edge of the tube is exactly on Shape1.
                    GeomAPI_ProjectPointOnCurve Projector = new();
                    Explorer exp = new(Tuyo, ShapeType.Edge);
                    Vertex V1, V2;
                    if (OF1isEdge)
                    { // Update CutEdges.
                        Edge EOnF1 = ShapeCasts.Edge(exp.Current());
                        Exploration.Vertices(EOnF1, out V1, out V2);

                        Point3D P1 = Tool.Pnt(V1);
                        Projector.Init(P1, GC1!);
                        double U1 = Projector.LowerDistanceParameter();

                        Point3D P2 = Tool.Pnt(V2);
                        Projector.Init(P2, GC1!);
                        double U2 = Projector.LowerDistanceParameter();

                        Shape aLocalShape = V1.Oriented(Internal);
                        B.UpdateVertex(ShapeCasts.Vertex(aLocalShape), U1, ShapeCasts.Edge(Or1), myTol);
                        aLocalShape = V2.Oriented(Internal);
                        B.UpdateVertex(ShapeCasts.Vertex(aLocalShape), U2, ShapeCasts.Edge(Or1), myTol);
                        //	  B.UpdateVertex(TopoDS::Vertex(V1.Oriented(TopAbs_INTERNAL)),U1,
                        //			 TopoDS::Edge(Or1),myTol);
                        //	  B.UpdateVertex(TopoDS::Vertex(V2.Oriented(TopAbs_INTERNAL)),U2,
                        //			 TopoDS::Edge(Or1),myTol);

                        if (!myCutEdges.IsBound(Or1))
                        {
                            myCutEdges.Bind(Or1, new NCollection_List<Shape>());
                        }
                        NCollection_List<Shape> L1 = myCutEdges.Find(Or1);
                        L1.Append(V1);
                        L1.Append(V2);
                    }
                    if (OF2isEdge)
                    { // Update CutEdges.
                        exp.Next();
                        Edge EOnF2 = ShapeCasts.Edge(exp.Current());
                        Exploration.Vertices(EOnF2, out V1, out V2);

                        Point3D P1 = Tool.Pnt(V1);
                        Projector.Init(P1, GC2!);
                        double U1 = Projector.LowerDistanceParameter();

                        Point3D P2 = Tool.Pnt(V2);
                        Projector.Init(P2, GC2!);
                        double U2 = Projector.LowerDistanceParameter();

                        Shape aLocalShape = V1.Oriented(Internal);
                        B.UpdateVertex(ShapeCasts.Vertex(aLocalShape), U1, ShapeCasts.Edge(Or2), myTol);
                        aLocalShape = V2.Oriented(Internal);
                        B.UpdateVertex(ShapeCasts.Vertex(aLocalShape), U2, ShapeCasts.Edge(Or2), myTol);
                        //	  B.UpdateVertex(TopoDS::Vertex(V1.Oriented(TopAbs_INTERNAL)),U1,
                        //			 TopoDS::Edge(Or2),myTol);
                        //	  B.UpdateVertex(TopoDS::Vertex(V2.Oriented(TopAbs_INTERNAL)),U2,
                        //			 TopoDS::Edge(Or2),myTol);

                        if (!myCutEdges.IsBound(Or2))
                        {
                            myCutEdges.Bind(Or2, new NCollection_List<Shape>());
                        }
                        NCollection_List<Shape> L2 = myCutEdges.Find(Or2);
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
        NCollection_IndexedDataMap<Shape, NCollection_List<Shape>> Map = new(ShapeHasher.Instance);
        Exploration.MapShapesAndAncestors(Co, ShapeType.Vertex, ShapeType.Edge, Map);

        for (int j = 1; j <= Map.Extent(); j++)
        {
            Vertex V = ShapeCasts.Vertex(Map.FindKey(j));
            if (Map[j].Extent() != 3)
            {
                continue;
            }

            NCollection_List<Shape> LOE = new();
            NCollection_List<Shape>.Iterator it = new();

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

            BRepOffset_Offset OFT = new(V, LOE, -myRadius, myNubs, myTol, C2);
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

        BrepBuilder B = new();

        // Maj of the Map of created.
        // Update edges that do not change in the resulting shape
        // i.e. invariant edges in the unwinding.
        Explorer exp = new(myShape, global::Kerf.Topology.ShapeType.Face);
        // int nbe = 1;
        for (; exp.More(); exp.Next())
        {
            Face CurF = ShapeCasts.Face(exp.Current());

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
            Face CurOF = myMapSF.Find(CurF).InternalFace();

            if (!myImageOffset.HasImage(CurOF))
            { // face disappears in unwinding
                continue;
            }

            Explorer exp2 = new(CurF, ShapeType.Edge);
            for (; exp2.More(); exp2.Next())
            {
                Edge CurE = ShapeCasts.Edge(exp2.Current());
                Shape aLocalShape = Offset.InternalGenerated(CurE);
                Edge CurOE = ShapeCasts.Edge(aLocalShape);
                //      const TopoDS_Edge& CurOE = TopoDS::Edge(Offset.Generated(CurE));

                if (!myImageOffset.HasImage(CurOE))
                {
                    continue;
                }
                // CurOE disappears

                Edge ImE = ShapeCasts.Edge(myImageOffset.Image(CurOE).First());
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
        exp.Init(myShape, global::Kerf.Topology.ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            Face CurF = ShapeCasts.Face(exp.Current());

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

                Face CurOF = myMapSF.Find(CurF).InternalFace();

                if (!myImageOffset.HasImage(CurOF))
                { // face disappears in unwinding
                    continue;
                }

                // List of faces generated by a face in the unwinding
                NCollection_List<Shape> Lim = new();
                myImageOffset.LastImage(CurOF, Lim);
                NCollection_List<Shape>.Iterator itLim = new(Lim);
                for (; itLim.More(); itLim.Next())
                {
                    // DeboucFace = offset Face unwinded in "Debouc".
                    Face DeboucFace = ShapeCasts.Face(itLim.Value());

                    Surface S = Tool.Surface(CurF, out Placement L)!;

                    Face NewF = new();
                    B.MakeFace(ref NewF);
                    B.UpdateFace(NewF, S, L, Tool.Tolerance(CurF));

                    NCollection_DataMap<Shape, Shape> MapSS = new(ShapeHasher.Instance);

                    Shape aLocalShape = DeboucFace.Oriented(Forward);
                    Face Face = ShapeCasts.Face(aLocalShape);
                    //	const TopoDS_Face& Face =
                    //	  TopoDS::Face(DeboucFace.Oriented(TopAbs_FORWARD));
                    Explorer exp2 = new(Face, ShapeType.Edge);
                    for (; exp2.More(); exp2.Next())
                    {
                        Edge E = ShapeCasts.Edge(exp2.Current());
                        Exploration.Vertices(E, out Vertex V1, out Vertex V2);
                        if (myCreated.IsBound(E))
                        {
                            if (myCreated.Find(E).IsBound(CurF))
                            {
                                Edge OE = ShapeCasts.Edge(created(E, CurF).First());
                                Exploration.Vertices(OE, out Vertex OV1, out Vertex OV2);
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

                    Explorer expw = new(Face, ShapeType.Wire);
                    for (; expw.More(); expw.Next())
                    {
                        Wire W = ShapeCasts.Wire(expw.Current());
                        Explorer expe = new(W.Oriented(Forward), ShapeType.Edge);
                        Wire OW = new();
                        B.MakeWire(ref OW);

                        for (; expe.More(); expe.Next())
                        {
                            Edge E = ShapeCasts.Edge(expe.Current());
                            Curve2D? C2d = Tool.CurveOnSurface(E, Face, out double f, out double l);
                            Edge OE = new();
                            if (MapSS.IsBound(E))
                            { // this is an edge of cutting
                                OE = ShapeCasts.Edge(MapSS.Find(E));
                                Shape aLocalShapeReversedE = E.Reversed();
                                Curve2D? C2d_1 =
                                    Tool.CurveOnSurface(ShapeCasts.Edge(aLocalShapeReversedE), Face, out f, out l);
                                //	      occ::handle<Geom2d_Curve> C2d_1 =
                                //		BRep_Tool::CurveOnSurface(TopoDS::Edge(E.Reversed()),
                                //					  Face,f,l);
                                if (E.Orientation() == Forward)
                                {
                                    B.UpdateEdge(OE, C2d, C2d_1, NewF, Tool.Tolerance(E));
                                }
                                else
                                {
                                    B.UpdateEdge(OE, C2d_1, C2d, NewF, Tool.Tolerance(E));
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
                                        OE = ShapeCasts.Edge(created(E, CurF).First());
                                    }
                                }
                                else
                                {
                                    B.MakeEdge(ref OE);
                                    Vertex OV1 = new(), OV2 = new();
                                    Exploration.Vertices(E, out Vertex V1, out Vertex V2);
                                    if (myCreated.IsBound(V1) && myCreated.Find(V1).IsBound(CurF))
                                    {
                                        OV1 = ShapeCasts.Vertex(created(V1, CurF).First());
                                    }
                                    else
                                    {
                                        B.MakeVertex(ref OV1);
                                        Point2D P2d = C2d!.Value(Tool.Parameter(V1, E, Face));
                                        S.D0(P2d.X(), P2d.Y(), out Point3D P);
                                        P.Transform(L.Transformation());
                                        B.UpdateVertex(OV1, P, Tool.Tolerance(V1));
                                        bindCreated(V1);
                                        bindCreated(V1, CurF);
                                        created(V1, CurF).Append(OV1);
                                    }
                                    if (myCreated.IsBound(V2) && myCreated.Find(V2).IsBound(CurF))
                                    {
                                        OV2 = ShapeCasts.Vertex(created(V2, CurF).First());
                                    }
                                    else
                                    {
                                        B.MakeVertex(ref OV2);
                                        Point2D P2d = C2d!.Value(Tool.Parameter(V2, E, Face));
                                        S.D0(P2d.X(), P2d.Y(), out Point3D P);
                                        P.Transform(L.Transformation());
                                        B.UpdateVertex(OV2, P, Tool.Tolerance(V2));
                                        bindCreated(V2);
                                        bindCreated(V2, CurF);
                                        created(V2, CurF).Append(OV2);
                                    }
                                    B.Add(OE, OV1.Oriented(V1.Orientation()));
                                    B.Add(OE, OV2.Oriented(V2.Orientation()));
                                }
                                B.UpdateEdge(OE, C2d, NewF, Tool.Tolerance(E));
                                B.Range(OE, f, l);
                                //	      ComputeCurve3d(OE,C2d,TheSurf,L,BRep_Tool::Tolerance(E));
                                MapSS.Bind(E, OE);
                            }
                            B.Add(OW, OE.Oriented(E.Orientation()));
                        }
                        B.Add(NewF, OW.Oriented(W.Orientation()));
                    }

                    NewF.Orientation(DeboucFace.Orientation());

                    BrepTools.Update(NewF);
                    B.Add(myResult, NewF);
                }
            }
        }

        // non-regarding the cause, there always remain greeb borders on this Shape, so it is sewn.
        BRepBuilderAPI_Sewing Sew = new(myTol);

        BRepLib.BuildCurves3d(myResult);

        exp.Init(myResult, global::Kerf.Topology.ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            Sew.Add(exp.Current());
        }

        Sew.Perform();

        // SameParameter is done in case Sew does not do it (Detect that the edges
        // are not sameparameter but does nothing.)

        Shape SewedShape = Sew.InternalSewedShape();
        if (!SewedShape.IsNull)
        {
            exp.Init(Sew.InternalSewedShape(), ShapeType.Edge);
            for (; exp.More(); exp.Next())
            {
                Edge sec = ShapeCasts.Edge(exp.Current());
                BRepLib.SameParameter(sec, Tool.Tolerance(sec));
            }
            myResult = SewedShape;
        }
    }

    //=================================================================================================

    private bool Intersect(in Shape Init,
                           in Face Face,
                           NCollection_DataMap<Shape, Box> MapSBox,
                           BRepOffset_Offset OF1,
                           BRepOffset_Inter3d Inter)
    {
        bool JenRajoute = false;

        Box Box1 = MapSBox.Find(Face);

        // -----------------------------------------------
        // intersection with all already created faces.
        // -----------------------------------------------
        Shape InitShape1 = OF1.InitialShape();
        bool F1surBordLibre = InitShape1.ShapeType() == ShapeType.Edge && myStopFaces.Contains(InitShape1);

        NCollection_Map<Shape> Done = new(ShapeHasher.Instance);
        // OCCT iterates myMapSF by the shapes' addresses; the port in insertion order.
        foreach (KeyValuePair<Shape, BRepOffset_Offset> It in myMapSF)
        {
            BRepOffset_Offset OF2 = It.Value;
            Face F2 = OF2.InternalFace();

            if (Box1.IsOut(MapSBox.Find(F2)))
            {
                continue;
            }

            if (Inter.IsDone(Face, F2))
            {
                continue;
            }

            // 2 tubes created on free border are not intersected.
            Shape InitShape2 = OF2.InitialShape();
            bool F2surBordLibre = InitShape2.ShapeType() == ShapeType.Edge && myStopFaces.Contains(InitShape2);

            if (F1surBordLibre && F2surBordLibre)
            {
                continue;
            }

            // -------------------------------------------------------
            // Tubes are not intersected with neighbor faces.
            // -------------------------------------------------------
            Shape ItKey = It.Key;

            if (Init.ShapeType() == ShapeType.Edge)
            {
                if (ItKey.ShapeType() == global::Kerf.Topology.ShapeType.Face && IsInFace(ShapeCasts.Edge(Init), ShapeCasts.Face(ItKey)))
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
            NCollection_List<Shape> LInt = new();
            Done.Clear();
            if (myAsDes.HasCommonDescendant(Face, F2, LInt))
            {
                NCollection_List<Shape>.Iterator itl2 = new();
                for (itl2.Initialize(LInt); itl2.More(); itl2.Next())
                {
                    Edge CurE = ShapeCasts.Edge(itl2.Value());
                    Exploration.Vertices(CurE, out Vertex V1, out Vertex V2);

                    if (Done.Add(V1))
                    {
                        bool IsOnR1 = IsOnRestriction(V1, CurE, Face, out Edge E1);
                        bool IsOnR2 = IsOnRestriction(V1, CurE, F2, out Edge E2);
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
                        bool IsOnR1 = IsOnRestriction(V2, CurE, Face, out Edge E1);
                        bool IsOnR2 = IsOnRestriction(V2, CurE, F2, out Edge E2);

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
