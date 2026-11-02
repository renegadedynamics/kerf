// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Rotation.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Rotation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_CurveType;
using static Occt.GeomAbs_SurfaceType;
using static Occt.TopAbs_Orientation;
using static Occt.TopAbs_ShapeEnum;

namespace Occt;

/// <summary>
/// The geometry of a rotational sweep (a revolution): circles, surfaces of revolution (canonized to planes, cylinders,
/// cones, spheres and tori), rotated caps; shapes on the axis are invariant.
/// </summary>
internal sealed class BRepSweep_Rotation : BRepSweep_Trsf
{
    private readonly double myAng;
    private readonly gp_Ax1 myAxe;

    private static double ComputeTolerance(in TopoDS_Edge E, in TopoDS_Face F, Geom2d_Curve C)
    {
        if (BRep_Tool.Degenerated(E))
        {
            return BRep_Tool.Tolerance(E);
        }

        Geom_Surface surf = BRep_Tool.Surface(F)!;
        Geom_Curve c3d = BRep_Tool.Curve(E, out double first, out double last)!;

        double d2 = 0.0;
        int nn = 23;
        double unsurnn = 1.0 / nn;
        for (int i = 0; i <= nn; i++)
        {
            double t = unsurnn * i;
            double u = first * (1.0 - t) + last * t;
            gp_Pnt Pc3d = c3d.Value(u);
            gp_Pnt2d UV = C.Value(u);
            gp_Pnt Pcons = surf.Value(UV.X(), UV.Y());
            if (Precision.IsInfinite(Pcons.X()) || Precision.IsInfinite(Pcons.Y()) || Precision.IsInfinite(Pcons.Z()))
            {
                d2 = Precision.Infinite();
                break;
            }
            double temp = Pc3d.SquareDistance(Pcons);
            if (temp > d2)
            {
                d2 = temp;
            }
        }
        d2 = 1.5 * Math.Sqrt(d2);
        if (d2 < 1.0e-7)
        {
            d2 = 1.0e-7;
        }
        return d2;
    }

    private static void SetThePCurve(BRep_Builder B, in TopoDS_Edge E, in TopoDS_Face F, TopAbs_Orientation O, Geom2d_Curve C)
    {
        // check if there is already a pcurve
        Geom2d_Curve? OC = null;
        var GP = BRep_Tool.Surface(F, out TopLoc_Location _) as Geom_Plane;
        if (GP is null)
        {
            OC = BRep_Tool.CurveOnSurface(E, F, out _, out _);
        }
        if (OC is null)
        {
            B.UpdateEdge(E, C, F, ComputeTolerance(E, F, C));
        }
        else
        {
            if (O == TopAbs_REVERSED)
            {
                B.UpdateEdge(E, OC, C, F, ComputeTolerance(E, F, C));
            }
            else
            {
                B.UpdateEdge(E, C, OC, F, ComputeTolerance(E, F, C));
            }
        }
    }

    /// <summary>The rotation of <paramref name="S"/> around <paramref name="A"/> by the angle <paramref name="D"/>.</summary>
    public BRepSweep_Rotation(in TopoDS_Shape S, in Sweep_NumShape N, in TopLoc_Location L, in gp_Ax1 A, double D, bool C)
        : base(new BRep_Builder(), S, N, L, C)
    {
        myAng = D;
        myAxe = A;
        Standard_ConstructionError_Raise_if(D < Precision.Angular(), "BRepSweep_Rotation::Constructor");
        Init();
    }

    public override TopoDS_Shape MakeEmptyVertex(in TopoDS_Shape aGenV, in Sweep_NumShape aDirV)
    {
        // call only in construction mode with copy.
        Standard_ConstructionError_Raise_if(!myCopy, "BRepSweep_Rotation::MakeEmptyVertex");
        gp_Pnt P = BRep_Tool.Pnt(TopoDS.Vertex(aGenV));
        TopoDS_Vertex V = new();
        if (aDirV.Index() == 2)
        {
            P.Transform(myLocation.Transformation());
        }
        ////// modified by jgv, 1.10.01, for buc61005 //////
        // myBuilder.Builder().MakeVertex(V,P,Precision::Confusion());
        myBuilder.Builder().MakeVertex(ref V, P, BRep_Tool.Tolerance(TopoDS.Vertex(aGenV)));
        ////////////////////////////////////////////////////
        if (aDirV.Index() == 1 && IsInvariant(aGenV) && myDirShapeTool.NbShapes() == 3)
        {
            myBuiltShapes[myGenShapeTool.Index(aGenV), 3] = true;
            myShapes[myGenShapeTool.Index(aGenV), 3] = V;
        }
        return V;
    }

    public override TopoDS_Shape MakeEmptyDirectingEdge(in TopoDS_Shape aGenV, in Sweep_NumShape aDirE)
    {
        TopoDS_Edge E = new();
        gp_Pnt P = BRep_Tool.Pnt(TopoDS.Vertex(aGenV));
        gp_Dir Dirz = myAxe.Direction();
        var V = new gp_Vec(Dirz);
        gp_Pnt O = myAxe.Location();
        O.Translate(V.Dot(new gp_Vec(O, P)) * V);
        if (O.IsEqual(P, Precision.Confusion()))
        {
            // make a degenerated edge
            // temporary make 3D curve null so that
            // parameters should be registered.
            // myBuilder.Builder().MakeEdge(E);
            var Axis = new gp_Ax2(O, Dirz);
            var GC = new Geom_Circle(Axis, 0.0);
            myBuilder.Builder().MakeEdge(ref E, GC, BRep_Tool.Tolerance(TopoDS.Vertex(aGenV)));
            myBuilder.Builder().Degenerated(E, true);
        }
        else
        {
            var Axis = new gp_Ax2(O, Dirz, new gp_Dir(new gp_Vec(O, P)));
            var GC = new Geom_Circle(Axis, O.Distance(P));
            double tol = BRep_Tool.Tolerance(TopoDS.Vertex(aGenV));
            myBuilder.Builder().MakeEdge(ref E, GC, tol);
        }
        return E;
    }

    public override TopoDS_Shape MakeEmptyGeneratingEdge(in TopoDS_Shape aGenE, in Sweep_NumShape aDirV)
    {
        // call in case of construction with copy, or only when meridian touches myaxe.
        TopoDS_Edge E = new();
        if (BRep_Tool.Degenerated(TopoDS.Edge(aGenE)))
        {
            myBuilder.Builder().MakeEdge(ref E);
            myBuilder.Builder().UpdateEdge(E, BRep_Tool.Tolerance(TopoDS.Edge(aGenE)));
            myBuilder.Builder().Degenerated(E, true);
        }
        else
        {
            // OCCT dereferences the curve (Copy()) before testing it for null: an edge without a 3D curve crashes there
            Geom_Curve? C = (Geom_Curve)BRep_Tool.Curve(TopoDS.Edge(aGenE), out TopLoc_Location Loc, out _, out _)!.Copy();
            if (C is not null)
            {
                C.Transform(Loc.Transformation());
                if (aDirV.Index() == 2)
                {
                    C.Transform(myLocation.Transformation());
                }
            }
            myBuilder.Builder().MakeEdge(ref E, C, BRep_Tool.Tolerance(TopoDS.Edge(aGenE)));
        }
        if (aDirV.Index() == 1 && IsInvariant(aGenE) && myDirShapeTool.NbShapes() == 3)
        {
            myBuiltShapes[myGenShapeTool.Index(aGenE), 3] = true;
            myShapes[myGenShapeTool.Index(aGenE), 3] = E;
        }
        return E;
    }

    public override void SetParameters(in TopoDS_Shape aNewFace, ref TopoDS_Shape aNewVertex, in TopoDS_Shape aGenF,
                                       in TopoDS_Shape aGenV, in Sweep_NumShape aDirV)
    {
        // Glue the parameter of vertices directly included in cap faces.
        gp_Pnt2d pnt2d = BRep_Tool.Parameters(TopoDS.Vertex(aGenV), TopoDS.Face(aGenF));
        myBuilder.Builder().UpdateVertex(TopoDS.Vertex(aNewVertex), pnt2d.X(), pnt2d.Y(), TopoDS.Face(aNewFace),
                                         Precision.PConfusion());
    }

    public override void SetDirectingParameter(in TopoDS_Shape aNewEdge, ref TopoDS_Shape aNewVertex, in TopoDS_Shape aGenV,
                                               in Sweep_NumShape aDirE, in Sweep_NumShape aDirV)
    {
        double param = 0;
        TopAbs_Orientation ori = TopAbs_FORWARD;
        if (aDirV.Index() == 2)
        {
            param = myAng;
            ori = TopAbs_REVERSED;
        }
        TopoDS_Vertex V_wnt = TopoDS.Vertex(aNewVertex);
        V_wnt.Orientation(ori);
        // OCCT.NET: frozen. Built on demand by the history after the result is published, the edge holds its vertices
        myBuilder.Builder().UpdateVertexOfFrozen(V_wnt, param, TopoDS.Edge(aNewEdge), Precision.PConfusion());
    }

    public override void SetGeneratingParameter(in TopoDS_Shape aNewEdge, ref TopoDS_Shape aNewVertex, in TopoDS_Shape aGenE,
                                                in TopoDS_Shape aGenV, in Sweep_NumShape aDirV)
    {
        TopoDS_Vertex vbid = TopoDS.Vertex(aNewVertex);
        vbid.Orientation(aGenV.Orientation());
        myBuilder.Builder().UpdateVertex(vbid, BRep_Tool.Parameter(TopoDS.Vertex(aGenV), TopoDS.Edge(aGenE)),
                                         TopoDS.Edge(aNewEdge), Precision.PConfusion());
    }

    public override TopoDS_Shape MakeEmptyFace(in TopoDS_Shape aGenS, in Sweep_NumShape aDirS)
    {
        double toler;
        TopoDS_Face F = new();
        Geom_Surface S;
        if (aGenS.ShapeType() == TopAbs_EDGE)
        {
            Geom_Curve C = BRep_Tool.Curve(TopoDS.Edge(aGenS), out TopLoc_Location L, out double First, out double Last)!;
            toler = BRep_Tool.Tolerance(TopoDS.Edge(aGenS));
            gp_Trsf Tr = L.Transformation();
            C = (Geom_Curve)C.Copy();
            //// modified by jgv, 9.12.03 ////
            C = new Geom_TrimmedCurve(C, First, Last);
            //////////////////////////////////
            C.Transform(Tr);

            var HC = new GeomAdaptor_Curve();
            HC.Load(C, First, Last);
            var AS = new GeomAdaptor_SurfaceOfRevolution(HC, myAxe);
            switch (AS.GetType())
            {
                case GeomAbs_Plane:
                {
                    var Pl = new Geom_Plane(AS.Plane());
                    S = Pl;
                }
                break;
                case GeomAbs_Cylinder:
                {
                    var Cy = new Geom_CylindricalSurface(AS.Cylinder());
                    S = Cy;
                }
                break;
                case GeomAbs_Sphere:
                {
                    var Sp = new Geom_SphericalSurface(AS.Sphere());
                    S = Sp;
                }
                break;
                case GeomAbs_Cone:
                {
                    var Co = new Geom_ConicalSurface(AS.Cone());
                    S = Co;
                }
                break;
                case GeomAbs_Torus:
                {
                    var To = new Geom_ToroidalSurface(AS.Torus());
                    S = To;
                }
                break;
                default:
                {
                    var Se = new Geom_SurfaceOfRevolution(C, myAxe);
                    S = Se;
                }
                break;
            }
        }
        else
        {
            S = BRep_Tool.Surface(TopoDS.Face(aGenS), out TopLoc_Location L)!;
            toler = BRep_Tool.Tolerance(TopoDS.Face(aGenS));
            gp_Trsf Tr = L.Transformation();
            S = (Geom_Surface)S.Copy();
            S.Transform(Tr);
            if (aDirS.Index() == 2)
            {
                S.Transform(myLocation.Transformation());
            }
        }
        myBuilder.Builder().MakeFace(ref F, S, toler);
        return F;
    }

    public override void SetPCurve(in TopoDS_Shape aNewFace, ref TopoDS_Shape aNewEdge, in TopoDS_Shape aGenF,
                                   in TopoDS_Shape aGenE, in Sweep_NumShape aDirV, TopAbs_Orientation orien)
    {
        // Set on edges of cap faces the same pcurves as
        // on edges of the generator face.
        SetThePCurve(myBuilder.Builder(), TopoDS.Edge(aNewEdge), TopoDS.Face(aNewFace), orien,
                     BRep_Tool.CurveOnSurface(TopoDS.Edge(aGenE), TopoDS.Face(aGenF), out _, out _)!);
    }

    public override void SetGeneratingPCurve(in TopoDS_Shape aNewFace, ref TopoDS_Shape aNewEdge, in TopoDS_Shape aGenE,
                                             in Sweep_NumShape aDirE, in Sweep_NumShape aDirV, TopAbs_Orientation orien)
    {
        var AS = new GeomAdaptor_Surface(BRep_Tool.Surface(TopoDS.Face(aNewFace), out TopLoc_Location Loc)!);
        double u, v;
        gp_Pnt point;
        var pnt2d = new gp_Pnt2d();
        var dir2d = new gp_Dir2d();
        var L = new gp_Lin2d();
        if (AS.GetType() == GeomAbs_Plane)
        {
            gp_Pln pln = AS.Plane();
            gp_Ax3 ax3 = pln.Position();
            Geom_Curve? aC = BRep_Tool.Curve(TopoDS.Edge(aNewEdge), out Loc, out _, out _);
            Geom_Line? GL = aC as Geom_Line;
            if (GL is null)
            {
                if (aC is Geom_TrimmedCurve aTrimmedCurve)
                {
                    GL = aTrimmedCurve.BasisCurve() as Geom_Line;
                    if (GL is null)
                    {
                        throw new Standard_ConstructionError("BRepSweep_Rotation::SetGeneratingPCurve");
                    }
                }
            }
            gp_Lin gl = GL!.Lin();
            gl.Transform(Loc.Transformation());
            point = gl.Location();
            gp_Dir dir = gl.Direction();
            ElSLib.PlaneParameters(ax3, point, out u, out v);
            pnt2d.SetCoord(u, v);
            dir2d.SetCoord(dir.Dot(ax3.XDirection()), dir.Dot(ax3.YDirection()));
            L.SetLocation(pnt2d);
            L.SetDirection(dir2d);
        }
        else if (AS.GetType() == GeomAbs_Torus)
        {
            gp_Torus tor = AS.Torus();
            var BC = new BRepAdaptor_Curve(TopoDS.Edge(aNewEdge));
            double U = BC.FirstParameter();
            point = BC.Value(U);
            if (point.Distance(tor.Location()) < Precision.Confusion())
            {
                v = Math.PI;
                //  modified by NIZHNY-EAP Wed Mar  1 17:49:29 2000 ___BEGIN___
                u = 0.0;
            }
            else
            {
                ElSLib.TorusParameters(tor.Position(), tor.MajorRadius(), tor.MinorRadius(), point, out u, out v);
            }
            //    u = 0.;
            v = ElCLib.InPeriod(v, 0.0, 2 * Math.PI);
            if ((2 * Math.PI - v) <= Precision.PConfusion())
            {
                v -= 2 * Math.PI;
            }
            if (aDirV.Index() == 2)
            {
                double uLeft = u - myAng;
                ElCLib.AdjustPeriodic(-Math.PI, Math.PI, Precision.PConfusion(), ref uLeft, ref u);
            }
            else
            {
                double uRight = u + myAng;
                ElCLib.AdjustPeriodic(-Math.PI, Math.PI, Precision.PConfusion(), ref u, ref uRight);
            }
            //  modified by NIZHNY-EAP Wed Mar  1 17:49:32 2000 ___END___
            pnt2d.SetCoord(u, v - U);
            L.SetLocation(pnt2d);
            L.SetDirection(gp.DY2d());
        }
        else if (AS.GetType() == GeomAbs_Sphere)
        {
            gp_Sphere sph = AS.Sphere();
            var BC = new BRepAdaptor_Curve(TopoDS.Edge(aNewEdge));
            double U = BC.FirstParameter();
            point = BC.Value(U);
            ElSLib.SphereParameters(sph.Position(), sph.Radius(), point, out u, out v);
            u = 0.0;
            if (aDirV.Index() == 2)
            {
                u = myAng;
            }
            pnt2d.SetCoord(u, v - U);
            L.SetLocation(pnt2d);
            L.SetDirection(gp.DY2d());
        }
        else
        {
            double anAngleTemp = 0;
            if (aDirV.Index() == 2)
            {
                anAngleTemp = myAng;
            }
            L.SetLocation(new gp_Pnt2d(anAngleTemp, 0));
            L.SetDirection(gp.DY2d());
        }
        var GL2 = new Geom2d_Line(L);
        SetThePCurve(myBuilder.Builder(), TopoDS.Edge(aNewEdge), TopoDS.Face(aNewFace), orien, GL2);
    }

    public override void SetDirectingPCurve(in TopoDS_Shape aNewFace, ref TopoDS_Shape aNewEdge, in TopoDS_Shape aGenE,
                                            in TopoDS_Shape aGenV, in Sweep_NumShape aDirE, TopAbs_Orientation orien)
    {
        var AS = new GeomAdaptor_Surface(BRep_Tool.Surface(TopoDS.Face(aNewFace), out TopLoc_Location _)!);
        double par = BRep_Tool.Parameter(TopoDS.Vertex(aGenV), TopoDS.Edge(aGenE));
        gp_Pnt p2 = BRep_Tool.Pnt(TopoDS.Vertex(aGenV));
        var p22d = new gp_Pnt2d();
        double u, v;
        Geom2d_Curve thePCurve;

        switch (AS.GetType())
        {
            case GeomAbs_Plane:
            {
                gp_Pln pln = AS.Plane();
                gp_Ax3 ax3 = pln.Position();
                gp_Pnt p1 = pln.Location();
                double R = p1.Distance(p2);
                ElSLib.PlaneParameters(ax3, p2, out u, out v);
                var dx2d = new gp_Dir2d(u, v);
                var axe = new gp_Ax22d(gp.Origin2d(), dx2d, gp.DY2d());
                var C = new gp_Circ2d(axe, R);
                var GC = new Geom2d_Circle(C);
                thePCurve = GC;
            }
            break;
            case GeomAbs_Cone:
            {
                gp_Cone cone = AS.Cone();
                ElSLib.ConeParameters(cone.Position(), cone.RefRadius(), cone.SemiAngle(), p2, out u, out v);
                p22d.SetCoord(0.0, v);
                var L = new gp_Lin2d(p22d, gp.DX2d());
                var GL = new Geom2d_Line(L);
                thePCurve = GL;
            }
            break;
            case GeomAbs_Sphere:
            {
                gp_Sphere sph = AS.Sphere();
                ElSLib.SphereParameters(sph.Position(), sph.Radius(), p2, out u, out v);
                p22d.SetCoord(0.0, v);
                var L = new gp_Lin2d(p22d, gp.DX2d());
                var GL = new Geom2d_Line(L);
                thePCurve = GL;
            }
            break;
            case GeomAbs_Torus:
            {
                gp_Pnt p1;
                double u1, u2, v1, v2;
                gp_Torus tor = AS.Torus();
                var BC = new BRepAdaptor_Curve(TopoDS.Edge(aGenE));
                p1 = BC.Value(BC.FirstParameter());
                if (p1.Distance(tor.Location()) < Precision.Confusion())
                {
                    v1 = Math.PI;
                    //  modified by NIZHNY-EAP Thu Mar  2 09:43:26 2000 ___BEGIN___
                    u1 = 0.0;
                    //  modified by NIZHNY-EAP Thu Mar  2 15:28:59 2000 ___END___
                }
                else
                {
                    ElSLib.TorusParameters(tor.Position(), tor.MajorRadius(), tor.MinorRadius(), p1, out u1, out v1);
                }
                p2 = BC.Value(BC.LastParameter());
                if (p2.Distance(tor.Location()) < Precision.Confusion())
                {
                    v2 = Math.PI;
                }
                else
                {
                    ElSLib.TorusParameters(tor.Position(), tor.MajorRadius(), tor.MinorRadius(), p2, out u2, out v2);
                }
                ElCLib.AdjustPeriodic(0.0, 2 * Math.PI, Precision.PConfusion(), ref v1, ref v2);
                //  modified by NIZHNY-EAP Thu Mar  2 15:29:04 2000 ___BEGIN___
                u2 = u1 + myAng;
                ElCLib.AdjustPeriodic(-Math.PI, Math.PI, Precision.PConfusion(), ref u1, ref u2);
                if (aGenV.Orientation() == TopAbs_FORWARD)
                {
                    p22d.SetCoord(u1, v1);
                }
                else
                {
                    p22d.SetCoord(u1, v2);
                    //  modified by NIZHNY-EAP Thu Mar  2 09:43:32 2000 ___END___
                }
                var L = new gp_Lin2d(p22d, gp.DX2d());
                var GL = new Geom2d_Line(L);
                thePCurve = GL;
            }
            break;
            default:
            {
                p22d.SetCoord(0.0, par);
                var L = new gp_Lin2d(p22d, gp.DX2d());
                var GL = new Geom2d_Line(L);
                thePCurve = GL;
            }
            break;
        }
        SetThePCurve(myBuilder.Builder(), TopoDS.Edge(aNewEdge), TopoDS.Face(aNewFace), orien, thePCurve);
    }

    // modified by NIZNHY-PKV Tue Jun 14 08:33:55 2011f
    public override TopAbs_Orientation DirectSolid(in TopoDS_Shape aGenS, in Sweep_NumShape aDirS)
    { // compare the face normal and the direction
        double aU1, aU2, aV1, aV2, aUx, aVx, aX, aMV2, aTol2, aTx;
        TopAbs_Orientation aOr;
        var surf = new BRepAdaptor_Surface(TopoDS.Face(aGenS));
        //
        aTol2 = Precision.Confusion();
        aTol2 = aTol2 * aTol2;
        //
        gp_Pnt aPAxeLoc = myAxe.Location();
        gp_Dir aPAxeDir = myAxe.Direction();
        //
        aU1 = surf.FirstUParameter();
        aU2 = surf.LastUParameter();
        aV1 = surf.FirstVParameter();
        aV2 = surf.LastVParameter();
        //
        aTx = 0.5;
        aUx = aTx * (aU1 + aU2);
        aVx = aTx * (aV1 + aV2);
        surf.D1(aUx, aVx, out gp_Pnt aP, out gp_Vec du, out gp_Vec dv);
        //
        var aV = new gp_Vec(aPAxeLoc, aP);
        aV.Cross(new gp_Vec(aPAxeDir));
        aMV2 = aV.SquareMagnitude();
        if (aMV2 < aTol2)
        {
            aTx = 0.43213918;
            aUx = aU1 * (1.0 - aTx) + aU2 * aTx;
            aVx = aV1 * (1.0 - aTx) + aV2 * aTx;
            surf.D1(aUx, aVx, out aP, out du, out dv);
            aV.SetXYZ(aP.XYZ() - aPAxeLoc.XYZ());
            aV.Cross(new gp_Vec(aPAxeDir));
        }
        //
        aX = aV.DotCross(du, dv);
        aOr = (aX > 0.0) ? TopAbs_FORWARD : TopAbs_REVERSED;
        return aOr;
    }
    // modified by NIZNHY-PKV Tue Jun 14 08:33:59 2011t

    public override bool GGDShapeIsToAdd(in TopoDS_Shape aNewShape, in TopoDS_Shape aNewSubShape, in TopoDS_Shape aGenS,
                                         in TopoDS_Shape aSubGenS, in Sweep_NumShape aDirS)
    {
        bool aRes = true;
        if (aNewShape.ShapeType() == TopAbs_FACE && aNewSubShape.ShapeType() == TopAbs_EDGE
            && aGenS.ShapeType() == TopAbs_EDGE && aSubGenS.ShapeType() == TopAbs_VERTEX
            && aDirS.Type() == TopAbs_EDGE)
        {
            var AS = new GeomAdaptor_Surface(BRep_Tool.Surface(TopoDS.Face(aNewShape), out TopLoc_Location _)!);
            if (AS.GetType() == GeomAbs_Plane)
            {
                return (!IsInvariant(aSubGenS));
            }
            else
            {
                return aRes;
            }
        }
        else
        {
            return aRes;
        }
    }

    public override bool GDDShapeIsToAdd(in TopoDS_Shape aNewShape, in TopoDS_Shape aNewSubShape, in TopoDS_Shape aGenS,
                                         in Sweep_NumShape aDirS, in Sweep_NumShape aSubDirS)
    {
        if (aNewShape.ShapeType() == TopAbs_SOLID && aNewSubShape.ShapeType() == TopAbs_FACE
            && aGenS.ShapeType() == TopAbs_FACE && aDirS.Type() == TopAbs_EDGE && aSubDirS.Type() == TopAbs_VERTEX)
        {
            return (Math.Abs(myAng - 2 * Math.PI) > Precision.Angular());
        }
        else if (aNewShape.ShapeType() == TopAbs_FACE && aNewSubShape.ShapeType() == TopAbs_EDGE
                 && aGenS.ShapeType() == TopAbs_EDGE && aDirS.Type() == TopAbs_EDGE && aSubDirS.Type() == TopAbs_VERTEX)
        {
            var AS = new GeomAdaptor_Surface(BRep_Tool.Surface(TopoDS.Face(aNewShape), out TopLoc_Location _)!);
            if (AS.GetType() == GeomAbs_Plane)
            {
                return (Math.Abs(myAng - 2 * Math.PI) > Precision.Angular());
            }
            else
            {
                return true;
            }
        }
        else
        {
            return true;
        }
    }

    public override bool SeparatedWires(in TopoDS_Shape aNewShape, in TopoDS_Shape aNewSubShape, in TopoDS_Shape aGenS,
                                        in TopoDS_Shape aSubGenS, in Sweep_NumShape aDirS)
    {
        if (aNewShape.ShapeType() == TopAbs_FACE && aNewSubShape.ShapeType() == TopAbs_EDGE
            && aGenS.ShapeType() == TopAbs_EDGE && aSubGenS.ShapeType() == TopAbs_VERTEX
            && aDirS.Type() == TopAbs_EDGE)
        {
            var AS = new GeomAdaptor_Surface(BRep_Tool.Surface(TopoDS.Face(aNewShape), out TopLoc_Location _)!);
            if (AS.GetType() == GeomAbs_Plane)
            {
                return (Math.Abs(myAng - 2 * Math.PI) <= Precision.Angular());
            }
            else
            {
                return false;
            }
        }
        else
        {
            return false;
        }
    }

    public override TopoDS_Shape SplitShell(in TopoDS_Shape aNewShape)
    {
        var Q = new BRepTools_Quilt();
        Q.Add(aNewShape);
        // OCCT.NET: the quilt's shells are read without freezing them (the sweep goes on building on them)
        return Q.InternalShells();
    }

    public override bool HasShape(in TopoDS_Shape aGenS, in Sweep_NumShape aDirS)
    {
        if (aDirS.Type() == TopAbs_EDGE && aGenS.ShapeType() == TopAbs_EDGE)
        {
            // Verify that the edge has entrails
            TopoDS_Edge anEdge = TopoDS.Edge(aGenS);
            //
            if (BRep_Tool.Degenerated(anEdge))
            {
                return false;
            }

            Geom_Curve? aCurve = BRep_Tool.Curve(anEdge, out TopLoc_Location _, out _, out _);
            if (aCurve is null)
            {
                return false;
            }

            if (IsInvariant(aGenS))
            {
                return false;
            }

            // Check seem edge
            var FaceExp = new TopExp_Explorer(myGenShape, TopAbs_FACE);
            for (; FaceExp.More(); FaceExp.Next())
            {
                TopoDS_Face F = TopoDS.Face(FaceExp.Current());
                if (BRepTools.IsReallyClosed(anEdge, F))
                {
                    return false;
                }
            }

            return true;
        }
        else
        {
            return true;
        }
    }

    public override bool IsInvariant(in TopoDS_Shape aGenS)
    {
        if (aGenS.ShapeType() == TopAbs_EDGE)
        {
            var aC = new BRepAdaptor_Curve(TopoDS.Edge(aGenS));
            if (aC.GetType() == GeomAbs_Line || aC.GetType() == GeomAbs_BSplineCurve || aC.GetType() == GeomAbs_BezierCurve)
            {
                TopExp.Vertices(TopoDS.Edge(aGenS), out TopoDS_Vertex V1, out TopoDS_Vertex V2);
                if (IsInvariant(V1) && IsInvariant(V2))
                {
                    if (aC.GetType() == GeomAbs_Line)
                    {
                        return true;
                    }

                    double aTol = Max(BRep_Tool.Tolerance(V1), BRep_Tool.Tolerance(V2));
                    var Lin = new gp_Lin(myAxe.Location(), myAxe.Direction());
                    if (aC.GetType() == GeomAbs_BSplineCurve)
                    {
                        Geom_BSplineCurve aBS = aC.BSpline()!;
                        for (int i = 1; i <= aBS.NbPoles(); i++)
                        {
                            if (Lin.Distance(aBS.Pole(i)) > aTol)
                            {
                                return false;
                            }
                        }
                    }
                    else
                    {
                        Geom_BezierCurve aBz = aC.Bezier()!;
                        for (int i = 1; i <= aBz.NbPoles(); i++)
                        {
                            if (Lin.Distance(aBz.Pole(i)) > aTol)
                            {
                                return false;
                            }
                        }
                    }
                    return true;
                }
            }
        }
        else if (aGenS.ShapeType() == TopAbs_VERTEX)
        {
            gp_Pnt P = BRep_Tool.Pnt(TopoDS.Vertex(aGenS));
            var Lin = new gp_Lin(myAxe.Location(), myAxe.Direction());
            return (Lin.Distance(P) <= BRep_Tool.Tolerance(TopoDS.Vertex(aGenS)));
        }
        return false;
    }

    /// <summary>The angle of the rotation.</summary>
    public double Angle() => myAng;

    /// <summary>The axis of the rotation.</summary>
    public gp_Ax1 Axe() => myAxe;
}
