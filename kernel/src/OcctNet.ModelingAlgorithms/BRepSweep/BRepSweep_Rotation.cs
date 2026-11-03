// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Rotation.hxx
// Port of: src/ModelingAlgorithms/TKPrim/BRepSweep/BRepSweep_Rotation.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Engine.Topology.Brep;
using Kerf.Engine.Topology.Tools;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Occt;

/// <summary>
/// The geometry of a rotational sweep (a revolution): circles, surfaces of revolution (canonized to planes, cylinders,
/// cones, spheres and tori), rotated caps; shapes on the axis are invariant.
/// </summary>
internal sealed class BRepSweep_Rotation : BRepSweep_Trsf
{
    private readonly double myAng;
    private readonly Axis3D myAxe;

    private static double ComputeTolerance(in Edge E, in Face F, Curve2D C)
    {
        if (Tool.Degenerated(E))
        {
            return Tool.Tolerance(E);
        }

        Surface surf = Tool.Surface(F)!;
        Curve3D c3d = Tool.Curve(E, out double first, out double last)!;

        double d2 = 0.0;
        int nn = 23;
        double unsurnn = 1.0 / nn;
        for (int i = 0; i <= nn; i++)
        {
            double t = unsurnn * i;
            double u = first * (1.0 - t) + last * t;
            Point3D Pc3d = c3d.Value(u);
            Point2D UV = C.Value(u);
            Point3D Pcons = surf.Value(UV.X(), UV.Y());
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(Pcons.X()) || global::Kerf.Engine.Foundation.Precision.IsInfinite(Pcons.Y()) || global::Kerf.Engine.Foundation.Precision.IsInfinite(Pcons.Z()))
            {
                d2 = global::Kerf.Engine.Foundation.Precision.Infinite();
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

    private static void SetThePCurve(BrepBuilder B, in Edge E, in Face F, Orientation O, Curve2D C)
    {
        // check if there is already a pcurve
        Curve2D? OC = null;
        var GP = Tool.Surface(F, out Placement _) as PlaneSurface;
        if (GP is null)
        {
            OC = Tool.CurveOnSurface(E, F, out _, out _);
        }
        if (OC is null)
        {
            B.UpdateEdge(E, C, F, ComputeTolerance(E, F, C));
        }
        else
        {
            if (O == Reversed)
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
    public BRepSweep_Rotation(in Shape S, in Sweep_NumShape N, in Placement L, in Axis3D A, double D, bool C)
        : base(new BrepBuilder(), S, N, L, C)
    {
        myAng = D;
        myAxe = A;
        Standard_ConstructionError_Raise_if(D < global::Kerf.Engine.Foundation.Precision.Angular(), "BRepSweep_Rotation::Constructor");
        Init();
    }

    public override Shape MakeEmptyVertex(in Shape aGenV, in Sweep_NumShape aDirV)
    {
        // call only in construction mode with copy.
        Standard_ConstructionError_Raise_if(!myCopy, "BRepSweep_Rotation::MakeEmptyVertex");
        Point3D P = Tool.Pnt(ShapeCasts.Vertex(aGenV));
        Vertex V = new();
        if (aDirV.Index() == 2)
        {
            P.Transform(myLocation.Transformation());
        }
        ////// modified by jgv, 1.10.01, for buc61005 //////
        // myBuilder.Builder().MakeVertex(V,P,Precision::Confusion());
        myBuilder.Builder().MakeVertex(ref V, P, Tool.Tolerance(ShapeCasts.Vertex(aGenV)));
        ////////////////////////////////////////////////////
        if (aDirV.Index() == 1 && IsInvariant(aGenV) && myDirShapeTool.NbShapes() == 3)
        {
            myBuiltShapes[myGenShapeTool.Index(aGenV), 3] = true;
            myShapes[myGenShapeTool.Index(aGenV), 3] = V;
        }
        return V;
    }

    public override Shape MakeEmptyDirectingEdge(in Shape aGenV, in Sweep_NumShape aDirE)
    {
        Edge E = new();
        Point3D P = Tool.Pnt(ShapeCasts.Vertex(aGenV));
        Direction3D Dirz = myAxe.Direction();
        var V = new Vector3D(Dirz);
        Point3D O = myAxe.Location();
        O.Translate(V.Dot(new Vector3D(O, P)) * V);
        if (O.IsEqual(P, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            // make a degenerated edge
            // temporary make 3D curve null so that
            // parameters should be registered.
            // myBuilder.Builder().MakeEdge(E);
            var Axis = new Frame3D(O, Dirz);
            var GC = new CircleCurve3D(Axis, 0.0);
            myBuilder.Builder().MakeEdge(ref E, GC, Tool.Tolerance(ShapeCasts.Vertex(aGenV)));
            myBuilder.Builder().Degenerated(E, true);
        }
        else
        {
            var Axis = new Frame3D(O, Dirz, new Direction3D(new Vector3D(O, P)));
            var GC = new CircleCurve3D(Axis, O.Distance(P));
            double tol = Tool.Tolerance(ShapeCasts.Vertex(aGenV));
            myBuilder.Builder().MakeEdge(ref E, GC, tol);
        }
        return E;
    }

    public override Shape MakeEmptyGeneratingEdge(in Shape aGenE, in Sweep_NumShape aDirV)
    {
        // call in case of construction with copy, or only when meridian touches myaxe.
        Edge E = new();
        if (Tool.Degenerated(ShapeCasts.Edge(aGenE)))
        {
            myBuilder.Builder().MakeEdge(ref E);
            myBuilder.Builder().UpdateEdge(E, Tool.Tolerance(ShapeCasts.Edge(aGenE)));
            myBuilder.Builder().Degenerated(E, true);
        }
        else
        {
            // OCCT dereferences the curve (Copy()) before testing it for null: an edge without a 3D curve crashes there
            Curve3D? C = (Curve3D)Tool.Curve(ShapeCasts.Edge(aGenE), out Placement Loc, out _, out _)!.Copy();
            if (C is not null)
            {
                C.Transform(Loc.Transformation());
                if (aDirV.Index() == 2)
                {
                    C.Transform(myLocation.Transformation());
                }
            }
            myBuilder.Builder().MakeEdge(ref E, C, Tool.Tolerance(ShapeCasts.Edge(aGenE)));
        }
        if (aDirV.Index() == 1 && IsInvariant(aGenE) && myDirShapeTool.NbShapes() == 3)
        {
            myBuiltShapes[myGenShapeTool.Index(aGenE), 3] = true;
            myShapes[myGenShapeTool.Index(aGenE), 3] = E;
        }
        return E;
    }

    public override void SetParameters(in Shape aNewFace, ref Shape aNewVertex, in Shape aGenF,
                                       in Shape aGenV, in Sweep_NumShape aDirV)
    {
        // Glue the parameter of vertices directly included in cap faces.
        Point2D pnt2d = Tool.Parameters(ShapeCasts.Vertex(aGenV), ShapeCasts.Face(aGenF));
        myBuilder.Builder().UpdateVertex(ShapeCasts.Vertex(aNewVertex), pnt2d.X(), pnt2d.Y(), ShapeCasts.Face(aNewFace),
                                         global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    public override void SetDirectingParameter(in Shape aNewEdge, ref Shape aNewVertex, in Shape aGenV,
                                               in Sweep_NumShape aDirE, in Sweep_NumShape aDirV)
    {
        double param = 0;
        Orientation ori = Forward;
        if (aDirV.Index() == 2)
        {
            param = myAng;
            ori = Reversed;
        }
        Vertex V_wnt = ShapeCasts.Vertex(aNewVertex);
        V_wnt.Orientation(ori);
        // OCCT.NET: frozen. Built on demand by the history after the result is published, the edge holds its vertices
        myBuilder.Builder().UpdateVertexOfFrozen(V_wnt, param, ShapeCasts.Edge(aNewEdge), global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    public override void SetGeneratingParameter(in Shape aNewEdge, ref Shape aNewVertex, in Shape aGenE,
                                                in Shape aGenV, in Sweep_NumShape aDirV)
    {
        Vertex vbid = ShapeCasts.Vertex(aNewVertex);
        vbid.Orientation(aGenV.Orientation());
        myBuilder.Builder().UpdateVertex(vbid, Tool.Parameter(ShapeCasts.Vertex(aGenV), ShapeCasts.Edge(aGenE)),
                                         ShapeCasts.Edge(aNewEdge), global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    public override Shape MakeEmptyFace(in Shape aGenS, in Sweep_NumShape aDirS)
    {
        double toler;
        Face F = new();
        Surface S;
        if (aGenS.ShapeType() == ShapeType.Edge)
        {
            Curve3D C = Tool.Curve(ShapeCasts.Edge(aGenS), out Placement L, out double First, out double Last)!;
            toler = Tool.Tolerance(ShapeCasts.Edge(aGenS));
            Transform3D Tr = L.Transformation();
            C = (Curve3D)C.Copy();
            //// modified by jgv, 9.12.03 ////
            C = new TrimmedCurve3D(C, First, Last);
            //////////////////////////////////
            C.Transform(Tr);

            var HC = new CurveAdaptor();
            HC.Load(C, First, Last);
            var AS = new RevolutionSurfaceAdaptor(HC, myAxe);
            switch (AS.GetType())
            {
                case Plane:
                {
                    var Pl = new PlaneSurface(AS.Plane());
                    S = Pl;
                }
                break;
                case SurfaceKind.Cylinder:
                {
                    var Cy = new CylindricalSurface(AS.Cylinder());
                    S = Cy;
                }
                break;
                case SurfaceKind.Sphere:
                {
                    var Sp = new SphericalSurface(AS.Sphere());
                    S = Sp;
                }
                break;
                case SurfaceKind.Cone:
                {
                    var Co = new ConicalSurface(AS.Cone());
                    S = Co;
                }
                break;
                case SurfaceKind.Torus:
                {
                    var To = new ToroidalSurface(AS.Torus());
                    S = To;
                }
                break;
                default:
                {
                    var Se = new RevolutionSurface(C, myAxe);
                    S = Se;
                }
                break;
            }
        }
        else
        {
            S = Tool.Surface(ShapeCasts.Face(aGenS), out Placement L)!;
            toler = Tool.Tolerance(ShapeCasts.Face(aGenS));
            Transform3D Tr = L.Transformation();
            S = (Surface)S.Copy();
            S.Transform(Tr);
            if (aDirS.Index() == 2)
            {
                S.Transform(myLocation.Transformation());
            }
        }
        myBuilder.Builder().MakeFace(ref F, S, toler);
        return F;
    }

    public override void SetPCurve(in Shape aNewFace, ref Shape aNewEdge, in Shape aGenF,
                                   in Shape aGenE, in Sweep_NumShape aDirV, Orientation orien)
    {
        // Set on edges of cap faces the same pcurves as
        // on edges of the generator face.
        SetThePCurve(myBuilder.Builder(), ShapeCasts.Edge(aNewEdge), ShapeCasts.Face(aNewFace), orien,
                     Tool.CurveOnSurface(ShapeCasts.Edge(aGenE), ShapeCasts.Face(aGenF), out _, out _)!);
    }

    public override void SetGeneratingPCurve(in Shape aNewFace, ref Shape aNewEdge, in Shape aGenE,
                                             in Sweep_NumShape aDirE, in Sweep_NumShape aDirV, Orientation orien)
    {
        var AS = new SurfaceAdaptor(Tool.Surface(ShapeCasts.Face(aNewFace), out Placement Loc)!);
        double u, v;
        Point3D point;
        var pnt2d = new Point2D();
        var dir2d = new Direction2D();
        var L = new Line2D();
        if (AS.GetType() == Plane)
        {
            Plane3D pln = AS.Plane();
            CoordinateSystem ax3 = pln.Position();
            Curve3D? aC = Tool.Curve(ShapeCasts.Edge(aNewEdge), out Loc, out _, out _);
            LineCurve3D? GL = aC as LineCurve3D;
            if (GL is null)
            {
                if (aC is TrimmedCurve3D aTrimmedCurve)
                {
                    GL = aTrimmedCurve.BasisCurve() as LineCurve3D;
                    if (GL is null)
                    {
                        throw new ConstructionErrorException("BRepSweep_Rotation::SetGeneratingPCurve");
                    }
                }
            }
            Line3D gl = GL!.Lin();
            gl.Transform(Loc.Transformation());
            point = gl.Location();
            Direction3D dir = gl.Direction();
            global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(ax3, point, out u, out v);
            pnt2d.SetCoord(u, v);
            dir2d.SetCoord(dir.Dot(ax3.XDirection()), dir.Dot(ax3.YDirection()));
            L.SetLocation(pnt2d);
            L.SetDirection(dir2d);
        }
        else if (AS.GetType() == SurfaceKind.Torus)
        {
            Torus tor = AS.Torus();
            var BC = new EdgeCurveAdaptor(ShapeCasts.Edge(aNewEdge));
            double U = BC.FirstParameter();
            point = BC.Value(U);
            if (point.Distance(tor.Location()) < global::Kerf.Engine.Foundation.Precision.Confusion())
            {
                v = Math.PI;
                //  modified by NIZHNY-EAP Wed Mar  1 17:49:29 2000 ___BEGIN___
                u = 0.0;
            }
            else
            {
                global::Kerf.Engine.Geometry.ElSLib.TorusParameters(tor.Position(), tor.MajorRadius(), tor.MinorRadius(), point, out u, out v);
            }
            //    u = 0.;
            v = global::Kerf.Engine.Geometry.ElCLib.InPeriod(v, 0.0, 2 * Math.PI);
            if ((2 * Math.PI - v) <= global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                v -= 2 * Math.PI;
            }
            if (aDirV.Index() == 2)
            {
                double uLeft = u - myAng;
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(-Math.PI, Math.PI, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref uLeft, ref u);
            }
            else
            {
                double uRight = u + myAng;
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(-Math.PI, Math.PI, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref u, ref uRight);
            }
            //  modified by NIZHNY-EAP Wed Mar  1 17:49:32 2000 ___END___
            pnt2d.SetCoord(u, v - U);
            L.SetLocation(pnt2d);
            L.SetDirection(Constants.DY2d());
        }
        else if (AS.GetType() == SurfaceKind.Sphere)
        {
            Sphere sph = AS.Sphere();
            var BC = new EdgeCurveAdaptor(ShapeCasts.Edge(aNewEdge));
            double U = BC.FirstParameter();
            point = BC.Value(U);
            global::Kerf.Engine.Geometry.ElSLib.SphereParameters(sph.Position(), sph.Radius(), point, out u, out v);
            u = 0.0;
            if (aDirV.Index() == 2)
            {
                u = myAng;
            }
            pnt2d.SetCoord(u, v - U);
            L.SetLocation(pnt2d);
            L.SetDirection(Constants.DY2d());
        }
        else
        {
            double anAngleTemp = 0;
            if (aDirV.Index() == 2)
            {
                anAngleTemp = myAng;
            }
            L.SetLocation(new Point2D(anAngleTemp, 0));
            L.SetDirection(Constants.DY2d());
        }
        var GL2 = new LineCurve2D(L);
        SetThePCurve(myBuilder.Builder(), ShapeCasts.Edge(aNewEdge), ShapeCasts.Face(aNewFace), orien, GL2);
    }

    public override void SetDirectingPCurve(in Shape aNewFace, ref Shape aNewEdge, in Shape aGenE,
                                            in Shape aGenV, in Sweep_NumShape aDirE, Orientation orien)
    {
        var AS = new SurfaceAdaptor(Tool.Surface(ShapeCasts.Face(aNewFace), out Placement _)!);
        double par = Tool.Parameter(ShapeCasts.Vertex(aGenV), ShapeCasts.Edge(aGenE));
        Point3D p2 = Tool.Pnt(ShapeCasts.Vertex(aGenV));
        var p22d = new Point2D();
        double u, v;
        Curve2D thePCurve;

        switch (AS.GetType())
        {
            case Plane:
            {
                Plane3D pln = AS.Plane();
                CoordinateSystem ax3 = pln.Position();
                Point3D p1 = pln.Location();
                double R = p1.Distance(p2);
                global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(ax3, p2, out u, out v);
                var dx2d = new Direction2D(u, v);
                var axe = new Frame2D(Constants.Origin2d(), dx2d, Constants.DY2d());
                var C = new Circle2D(axe, R);
                var GC = new CircleCurve2D(C);
                thePCurve = GC;
            }
            break;
            case SurfaceKind.Cone:
            {
                Cone cone = AS.Cone();
                global::Kerf.Engine.Geometry.ElSLib.ConeParameters(cone.Position(), cone.RefRadius(), cone.SemiAngle(), p2, out u, out v);
                p22d.SetCoord(0.0, v);
                var L = new Line2D(p22d, Constants.DX2d());
                var GL = new LineCurve2D(L);
                thePCurve = GL;
            }
            break;
            case SurfaceKind.Sphere:
            {
                Sphere sph = AS.Sphere();
                global::Kerf.Engine.Geometry.ElSLib.SphereParameters(sph.Position(), sph.Radius(), p2, out u, out v);
                p22d.SetCoord(0.0, v);
                var L = new Line2D(p22d, Constants.DX2d());
                var GL = new LineCurve2D(L);
                thePCurve = GL;
            }
            break;
            case SurfaceKind.Torus:
            {
                Point3D p1;
                double u1, u2, v1, v2;
                Torus tor = AS.Torus();
                var BC = new EdgeCurveAdaptor(ShapeCasts.Edge(aGenE));
                p1 = BC.Value(BC.FirstParameter());
                if (p1.Distance(tor.Location()) < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    v1 = Math.PI;
                    //  modified by NIZHNY-EAP Thu Mar  2 09:43:26 2000 ___BEGIN___
                    u1 = 0.0;
                    //  modified by NIZHNY-EAP Thu Mar  2 15:28:59 2000 ___END___
                }
                else
                {
                    global::Kerf.Engine.Geometry.ElSLib.TorusParameters(tor.Position(), tor.MajorRadius(), tor.MinorRadius(), p1, out u1, out v1);
                }
                p2 = BC.Value(BC.LastParameter());
                if (p2.Distance(tor.Location()) < global::Kerf.Engine.Foundation.Precision.Confusion())
                {
                    v2 = Math.PI;
                }
                else
                {
                    global::Kerf.Engine.Geometry.ElSLib.TorusParameters(tor.Position(), tor.MajorRadius(), tor.MinorRadius(), p2, out u2, out v2);
                }
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(0.0, 2 * Math.PI, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref v1, ref v2);
                //  modified by NIZHNY-EAP Thu Mar  2 15:29:04 2000 ___BEGIN___
                u2 = u1 + myAng;
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(-Math.PI, Math.PI, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref u1, ref u2);
                if (aGenV.Orientation() == Forward)
                {
                    p22d.SetCoord(u1, v1);
                }
                else
                {
                    p22d.SetCoord(u1, v2);
                    //  modified by NIZHNY-EAP Thu Mar  2 09:43:32 2000 ___END___
                }
                var L = new Line2D(p22d, Constants.DX2d());
                var GL = new LineCurve2D(L);
                thePCurve = GL;
            }
            break;
            default:
            {
                p22d.SetCoord(0.0, par);
                var L = new Line2D(p22d, Constants.DX2d());
                var GL = new LineCurve2D(L);
                thePCurve = GL;
            }
            break;
        }
        SetThePCurve(myBuilder.Builder(), ShapeCasts.Edge(aNewEdge), ShapeCasts.Face(aNewFace), orien, thePCurve);
    }

    // modified by NIZNHY-PKV Tue Jun 14 08:33:55 2011f
    public override Orientation DirectSolid(in Shape aGenS, in Sweep_NumShape aDirS)
    { // compare the face normal and the direction
        double aU1, aU2, aV1, aV2, aUx, aVx, aX, aMV2, aTol2, aTx;
        Orientation aOr;
        var surf = new FaceSurfaceAdaptor(ShapeCasts.Face(aGenS));
        //
        aTol2 = global::Kerf.Engine.Foundation.Precision.Confusion();
        aTol2 = aTol2 * aTol2;
        //
        Point3D aPAxeLoc = myAxe.Location();
        Direction3D aPAxeDir = myAxe.Direction();
        //
        aU1 = surf.FirstUParameter();
        aU2 = surf.LastUParameter();
        aV1 = surf.FirstVParameter();
        aV2 = surf.LastVParameter();
        //
        aTx = 0.5;
        aUx = aTx * (aU1 + aU2);
        aVx = aTx * (aV1 + aV2);
        surf.D1(aUx, aVx, out Point3D aP, out Vector3D du, out Vector3D dv);
        //
        var aV = new Vector3D(aPAxeLoc, aP);
        aV.Cross(new Vector3D(aPAxeDir));
        aMV2 = aV.SquareMagnitude();
        if (aMV2 < aTol2)
        {
            aTx = 0.43213918;
            aUx = aU1 * (1.0 - aTx) + aU2 * aTx;
            aVx = aV1 * (1.0 - aTx) + aV2 * aTx;
            surf.D1(aUx, aVx, out aP, out du, out dv);
            aV.SetXYZ(aP.XYZ() - aPAxeLoc.XYZ());
            aV.Cross(new Vector3D(aPAxeDir));
        }
        //
        aX = aV.DotCross(du, dv);
        aOr = (aX > 0.0) ? Forward : Reversed;
        return aOr;
    }
    // modified by NIZNHY-PKV Tue Jun 14 08:33:59 2011t

    public override bool GGDShapeIsToAdd(in Shape aNewShape, in Shape aNewSubShape, in Shape aGenS,
                                         in Shape aSubGenS, in Sweep_NumShape aDirS)
    {
        bool aRes = true;
        if (aNewShape.ShapeType() == ShapeType.Face && aNewSubShape.ShapeType() == ShapeType.Edge
            && aGenS.ShapeType() == ShapeType.Edge && aSubGenS.ShapeType() == ShapeType.Vertex
            && aDirS.Type() == ShapeType.Edge)
        {
            var AS = new SurfaceAdaptor(Tool.Surface(ShapeCasts.Face(aNewShape), out Placement _)!);
            if (AS.GetType() == Plane)
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

    public override bool GDDShapeIsToAdd(in Shape aNewShape, in Shape aNewSubShape, in Shape aGenS,
                                         in Sweep_NumShape aDirS, in Sweep_NumShape aSubDirS)
    {
        if (aNewShape.ShapeType() == ShapeType.Solid && aNewSubShape.ShapeType() == ShapeType.Face
            && aGenS.ShapeType() == ShapeType.Face && aDirS.Type() == ShapeType.Edge && aSubDirS.Type() == ShapeType.Vertex)
        {
            return (Math.Abs(myAng - 2 * Math.PI) > global::Kerf.Engine.Foundation.Precision.Angular());
        }
        else if (aNewShape.ShapeType() == ShapeType.Face && aNewSubShape.ShapeType() == ShapeType.Edge
                 && aGenS.ShapeType() == ShapeType.Edge && aDirS.Type() == ShapeType.Edge && aSubDirS.Type() == ShapeType.Vertex)
        {
            var AS = new SurfaceAdaptor(Tool.Surface(ShapeCasts.Face(aNewShape), out Placement _)!);
            if (AS.GetType() == Plane)
            {
                return (Math.Abs(myAng - 2 * Math.PI) > global::Kerf.Engine.Foundation.Precision.Angular());
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

    public override bool SeparatedWires(in Shape aNewShape, in Shape aNewSubShape, in Shape aGenS,
                                        in Shape aSubGenS, in Sweep_NumShape aDirS)
    {
        if (aNewShape.ShapeType() == ShapeType.Face && aNewSubShape.ShapeType() == ShapeType.Edge
            && aGenS.ShapeType() == ShapeType.Edge && aSubGenS.ShapeType() == ShapeType.Vertex
            && aDirS.Type() == ShapeType.Edge)
        {
            var AS = new SurfaceAdaptor(Tool.Surface(ShapeCasts.Face(aNewShape), out Placement _)!);
            if (AS.GetType() == Plane)
            {
                return (Math.Abs(myAng - 2 * Math.PI) <= global::Kerf.Engine.Foundation.Precision.Angular());
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

    public override Shape SplitShell(in Shape aNewShape)
    {
        var Q = new Quilt();
        Q.Add(aNewShape);
        // OCCT.NET: the quilt's shells are read without freezing them (the sweep goes on building on them)
        return Q.InternalShells();
    }

    public override bool HasShape(in Shape aGenS, in Sweep_NumShape aDirS)
    {
        if (aDirS.Type() == ShapeType.Edge && aGenS.ShapeType() == ShapeType.Edge)
        {
            // Verify that the edge has entrails
            Edge anEdge = ShapeCasts.Edge(aGenS);
            //
            if (Tool.Degenerated(anEdge))
            {
                return false;
            }

            Curve3D? aCurve = Tool.Curve(anEdge, out Placement _, out _, out _);
            if (aCurve is null)
            {
                return false;
            }

            if (IsInvariant(aGenS))
            {
                return false;
            }

            // Check seem edge
            var FaceExp = new Explorer(myGenShape, ShapeType.Face);
            for (; FaceExp.More(); FaceExp.Next())
            {
                Face F = ShapeCasts.Face(FaceExp.Current());
                if (BrepTools.IsReallyClosed(anEdge, F))
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

    public override bool IsInvariant(in Shape aGenS)
    {
        if (aGenS.ShapeType() == ShapeType.Edge)
        {
            var aC = new EdgeCurveAdaptor(ShapeCasts.Edge(aGenS));
            if (aC.GetType() == Line || aC.GetType() == BSplineCurve || aC.GetType() == BezierCurve)
            {
                Exploration.Vertices(ShapeCasts.Edge(aGenS), out Vertex V1, out Vertex V2);
                if (IsInvariant(V1) && IsInvariant(V2))
                {
                    if (aC.GetType() == Line)
                    {
                        return true;
                    }

                    double aTol = Max(Tool.Tolerance(V1), Tool.Tolerance(V2));
                    var Lin = new Line3D(myAxe.Location(), myAxe.Direction());
                    if (aC.GetType() == BSplineCurve)
                    {
                        BSplineCurve3D aBS = aC.BSpline()!;
                        for (int i = 1; i <= aBS.PoleCount; i++)
                        {
                            if (Lin.Distance(aBS.Pole(i)) > aTol)
                            {
                                return false;
                            }
                        }
                    }
                    else
                    {
                        BezierCurve3D aBz = aC.Bezier()!;
                        for (int i = 1; i <= aBz.PoleCount; i++)
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
        else if (aGenS.ShapeType() == ShapeType.Vertex)
        {
            Point3D P = Tool.Pnt(ShapeCasts.Vertex(aGenS));
            var Lin = new Line3D(myAxe.Location(), myAxe.Direction());
            return (Lin.Distance(P) <= Tool.Tolerance(ShapeCasts.Vertex(aGenS)));
        }
        return false;
    }

    /// <summary>The angle of the rotation.</summary>
    public double Angle() => myAng;

    /// <summary>The axis of the rotation.</summary>
    public Axis3D Axe() => myAxe;
}
