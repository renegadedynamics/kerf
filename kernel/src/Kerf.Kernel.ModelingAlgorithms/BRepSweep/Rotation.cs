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
using Occt;
using static Kerf.Geometry.CurveKind;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Construction.Sweeping;

/// <summary>
/// The geometry of a rotational sweep (a revolution): circles, surfaces of revolution (canonized to planes, cylinders,
/// cones, spheres and tori), rotated caps; shapes on the axis are invariant.
/// </summary>
internal sealed class Rotation : Trsf
{
    private readonly double _ang;
    private readonly Axis3D _axe;

    private static double ComputeTolerance(in Edge e, in Face f, Curve2D c)
    {
        if (Tool.Degenerated(e))
        {
            return Tool.Tolerance(e);
        }

        Surface surf = Tool.Surface(f)!;
        Curve3D c3d = Tool.Curve(e, out double first, out double last)!;

        double d2 = 0.0;
        int nn = 23;
        double unsurnn = 1.0 / nn;
        for (int i = 0; i <= nn; i++)
        {
            double t = unsurnn * i;
            double u = first * (1.0 - t) + last * t;
            Point3D pc3d = c3d.Value(u);
            Point2D uv = c.Value(u);
            Point3D pcons = surf.Value(uv.X(), uv.Y());
            if (global::Kerf.Engine.Foundation.Precision.IsInfinite(pcons.X()) || global::Kerf.Engine.Foundation.Precision.IsInfinite(pcons.Y()) || global::Kerf.Engine.Foundation.Precision.IsInfinite(pcons.Z()))
            {
                d2 = global::Kerf.Engine.Foundation.Precision.Infinite();
                break;
            }
            double temp = pc3d.SquareDistance(pcons);
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

    private static void SetThePCurve(BrepBuilder b, in Edge e, in Face f, Orientation o, Curve2D c)
    {
        // check if there is already a pcurve
        Curve2D? oc = null;
        var gp = Tool.Surface(f, out Placement _) as PlaneSurface;
        if (gp is null)
        {
            oc = Tool.CurveOnSurface(e, f, out _, out _);
        }
        if (oc is null)
        {
            b.UpdateEdge(e, c, f, ComputeTolerance(e, f, c));
        }
        else
        {
            if (o == Reversed)
            {
                b.UpdateEdge(e, oc, c, f, ComputeTolerance(e, f, c));
            }
            else
            {
                b.UpdateEdge(e, c, oc, f, ComputeTolerance(e, f, c));
            }
        }
    }

    /// <summary>The rotation of <paramref name="s"/> around <paramref name="a"/> by the angle <paramref name="d"/>.</summary>
    public Rotation(in Shape s, in NumShape n, in Placement l, in Axis3D a, double d, bool c)
        : base(new BrepBuilder(), s, n, l, c)
    {
        _ang = d;
        _axe = a;
        Standard_ConstructionError_Raise_if(d < global::Kerf.Engine.Foundation.Precision.Angular(), "BRepSweep_Rotation::Constructor");
        Init();
    }

    public override Shape MakeEmptyVertex(in Shape aGenV, in NumShape aDirV)
    {
        // call only in construction mode with copy.
        Standard_ConstructionError_Raise_if(!_copy, "BRepSweep_Rotation::MakeEmptyVertex");
        Point3D p = Tool.Pnt(ShapeCasts.Vertex(aGenV));
        Vertex v = new();
        if (aDirV.Index() == 2)
        {
            p = p.Transformed(_location.Transformation());
        }
        ////// modified by jgv, 1.10.01, for buc61005 //////
        // myBuilder.Builder().MakeVertex(V,P,Precision::Confusion());
        _builder.Builder().MakeVertex(ref v, p, Tool.Tolerance(ShapeCasts.Vertex(aGenV)));
        ////////////////////////////////////////////////////
        if (aDirV.Index() == 1 && IsInvariant(aGenV) && _dirShapeTool.NbShapes() == 3)
        {
            _builtShapes[_genShapeTool.Index(aGenV), 3] = true;
            _shapes[_genShapeTool.Index(aGenV), 3] = v;
        }
        return v;
    }

    public override Shape MakeEmptyDirectingEdge(in Shape aGenV, in NumShape aDirE)
    {
        Edge e = new();
        Point3D p = Tool.Pnt(ShapeCasts.Vertex(aGenV));
        Direction3D dirz = _axe.Direction();
        var v = new Vector3D(dirz);
        Point3D o = _axe.Location();
        o = o.Translated(v.Dot(new Vector3D(o, p)) * v);
        if (o.IsEqual(p, global::Kerf.Engine.Foundation.Precision.Confusion()))
        {
            // make a degenerated edge
            // temporary make 3D curve null so that
            // parameters should be registered.
            // myBuilder.Builder().MakeEdge(E);
            var axis = new Frame3D(o, dirz);
            var gc = new CircleCurve3D(axis, 0.0);
            _builder.Builder().MakeEdge(ref e, gc, Tool.Tolerance(ShapeCasts.Vertex(aGenV)));
            _builder.Builder().Degenerated(e, true);
        }
        else
        {
            var axis = new Frame3D(o, dirz, new Direction3D(new Vector3D(o, p)));
            var gc = new CircleCurve3D(axis, o.Distance(p));
            double tol = Tool.Tolerance(ShapeCasts.Vertex(aGenV));
            _builder.Builder().MakeEdge(ref e, gc, tol);
        }
        return e;
    }

    public override Shape MakeEmptyGeneratingEdge(in Shape aGenE, in NumShape aDirV)
    {
        // call in case of construction with copy, or only when meridian touches myaxe.
        Edge e = new();
        if (Tool.Degenerated(ShapeCasts.Edge(aGenE)))
        {
            _builder.Builder().MakeEdge(ref e);
            _builder.Builder().UpdateEdge(e, Tool.Tolerance(ShapeCasts.Edge(aGenE)));
            _builder.Builder().Degenerated(e, true);
        }
        else
        {
            // OCCT dereferences the curve (Copy()) before testing it for null: an edge without a 3D curve crashes there
            Curve3D? c = (Curve3D)Tool.Curve(ShapeCasts.Edge(aGenE), out Placement loc, out _, out _)!.Copy();
            if (c is not null)
            {
                c.Transform(loc.Transformation());
                if (aDirV.Index() == 2)
                {
                    c.Transform(_location.Transformation());
                }
            }
            _builder.Builder().MakeEdge(ref e, c, Tool.Tolerance(ShapeCasts.Edge(aGenE)));
        }
        if (aDirV.Index() == 1 && IsInvariant(aGenE) && _dirShapeTool.NbShapes() == 3)
        {
            _builtShapes[_genShapeTool.Index(aGenE), 3] = true;
            _shapes[_genShapeTool.Index(aGenE), 3] = e;
        }
        return e;
    }

    public override void SetParameters(in Shape aNewFace, ref Shape aNewVertex, in Shape aGenF,
                                       in Shape aGenV, in NumShape aDirV)
    {
        // Glue the parameter of vertices directly included in cap faces.
        Point2D pnt2d = Tool.Parameters(ShapeCasts.Vertex(aGenV), ShapeCasts.Face(aGenF));
        _builder.Builder().UpdateVertex(ShapeCasts.Vertex(aNewVertex), pnt2d.X(), pnt2d.Y(), ShapeCasts.Face(aNewFace),
                                         global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    public override void SetDirectingParameter(in Shape aNewEdge, ref Shape aNewVertex, in Shape aGenV,
                                               in NumShape aDirE, in NumShape aDirV)
    {
        double param = 0;
        Orientation ori = Forward;
        if (aDirV.Index() == 2)
        {
            param = _ang;
            ori = Reversed;
        }
        Vertex V_wnt = ShapeCasts.Vertex(aNewVertex);
        V_wnt.Orientation(ori);
        // OCCT.NET: frozen. Built on demand by the history after the result is published, the edge holds its vertices
        _builder.Builder().UpdateVertexOfFrozen(V_wnt, param, ShapeCasts.Edge(aNewEdge), global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    public override void SetGeneratingParameter(in Shape aNewEdge, ref Shape aNewVertex, in Shape aGenE,
                                                in Shape aGenV, in NumShape aDirV)
    {
        Vertex vbid = ShapeCasts.Vertex(aNewVertex);
        vbid.Orientation(aGenV.Orientation());
        _builder.Builder().UpdateVertex(vbid, Tool.Parameter(ShapeCasts.Vertex(aGenV), ShapeCasts.Edge(aGenE)),
                                         ShapeCasts.Edge(aNewEdge), global::Kerf.Engine.Foundation.Precision.PConfusion());
    }

    public override Shape MakeEmptyFace(in Shape aGenS, in NumShape aDirS)
    {
        double toler;
        Face f = new();
        Surface s;
        if (aGenS.ShapeType() == ShapeType.Edge)
        {
            Curve3D c = Tool.Curve(ShapeCasts.Edge(aGenS), out Placement l, out double first, out double last)!;
            toler = Tool.Tolerance(ShapeCasts.Edge(aGenS));
            Transform3D tr = l.Transformation();
            c = (Curve3D)c.Copy();
            //// modified by jgv, 9.12.03 ////
            c = new TrimmedCurve3D(c, first, last);
            //////////////////////////////////
            c.Transform(tr);

            var hc = new CurveAdaptor();
            hc.Load(c, first, last);
            var AS = new RevolutionSurfaceAdaptor(hc, _axe);
            switch (AS.GetType())
            {
                case Plane:
                {
                    var pl = new PlaneSurface(AS.Plane());
                    s = pl;
                }
                break;
                case SurfaceKind.Cylinder:
                {
                    var cy = new CylindricalSurface(AS.Cylinder());
                    s = cy;
                }
                break;
                case SurfaceKind.Sphere:
                {
                    var sp = new SphericalSurface(AS.Sphere());
                    s = sp;
                }
                break;
                case SurfaceKind.Cone:
                {
                    var co = new ConicalSurface(AS.Cone());
                    s = co;
                }
                break;
                case SurfaceKind.Torus:
                {
                    var to = new ToroidalSurface(AS.Torus());
                    s = to;
                }
                break;
                default:
                {
                    var se = new RevolutionSurface(c, _axe);
                    s = se;
                }
                break;
            }
        }
        else
        {
            s = Tool.Surface(ShapeCasts.Face(aGenS), out Placement l)!;
            toler = Tool.Tolerance(ShapeCasts.Face(aGenS));
            Transform3D tr = l.Transformation();
            s = (Surface)s.Copy();
            s.Transform(tr);
            if (aDirS.Index() == 2)
            {
                s.Transform(_location.Transformation());
            }
        }
        _builder.Builder().MakeFace(ref f, s, toler);
        return f;
    }

    public override void SetPCurve(in Shape aNewFace, ref Shape aNewEdge, in Shape aGenF,
                                   in Shape aGenE, in NumShape aDirV, Orientation orien)
    {
        // Set on edges of cap faces the same pcurves as
        // on edges of the generator face.
        SetThePCurve(_builder.Builder(), ShapeCasts.Edge(aNewEdge), ShapeCasts.Face(aNewFace), orien,
                     Tool.CurveOnSurface(ShapeCasts.Edge(aGenE), ShapeCasts.Face(aGenF), out _, out _)!);
    }

    public override void SetGeneratingPCurve(in Shape aNewFace, ref Shape aNewEdge, in Shape aGenE,
                                             in NumShape aDirE, in NumShape aDirV, Orientation orien)
    {
        var AS = new SurfaceAdaptor(Tool.Surface(ShapeCasts.Face(aNewFace), out Placement loc)!);
        double u, v;
        Point3D point;
        var pnt2d = new Point2D();
        var dir2d = new Direction2D();
        var l = new Line2D();
        if (AS.GetType() == Plane)
        {
            Plane3D pln = AS.Plane();
            CoordinateSystem ax3 = pln.Position();
            Curve3D? c = Tool.Curve(ShapeCasts.Edge(aNewEdge), out loc, out _, out _);
            LineCurve3D? GL = c as LineCurve3D;
            if (GL is null)
            {
                if (c is TrimmedCurve3D trimmedCurve)
                {
                    GL = trimmedCurve.BasisCurve() as LineCurve3D;
                    if (GL is null)
                    {
                        throw new ConstructionErrorException("BRepSweep_Rotation::SetGeneratingPCurve");
                    }
                }
            }
            Line3D gl = GL!.Lin();
            gl = gl.Transformed(loc.Transformation());
            point = gl.Location();
            Direction3D dir = gl.Direction();
            global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(ax3, point, out u, out v);
            pnt2d = pnt2d.WithCoord(u, v);
            dir2d = dir2d.WithCoord(dir.Dot(ax3.XDirection()), dir.Dot(ax3.YDirection()));
            l = l.WithLocation(pnt2d);
            l = l.WithDirection(dir2d);
        }
        else if (AS.GetType() == SurfaceKind.Torus)
        {
            Torus tor = AS.Torus();
            var bc = new EdgeCurveAdaptor(ShapeCasts.Edge(aNewEdge));
            double U = bc.FirstParameter();
            point = bc.Value(U);
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
                double uLeft = u - _ang;
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(-Math.PI, Math.PI, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref uLeft, ref u);
            }
            else
            {
                double uRight = u + _ang;
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(-Math.PI, Math.PI, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref u, ref uRight);
            }
            //  modified by NIZHNY-EAP Wed Mar  1 17:49:32 2000 ___END___
            pnt2d = pnt2d.WithCoord(u, v - U);
            l = l.WithLocation(pnt2d);
            l = l.WithDirection(Constants.DY2d());
        }
        else if (AS.GetType() == SurfaceKind.Sphere)
        {
            Sphere sph = AS.Sphere();
            var bc = new EdgeCurveAdaptor(ShapeCasts.Edge(aNewEdge));
            double U = bc.FirstParameter();
            point = bc.Value(U);
            global::Kerf.Engine.Geometry.ElSLib.SphereParameters(sph.Position(), sph.Radius(), point, out u, out v);
            u = 0.0;
            if (aDirV.Index() == 2)
            {
                u = _ang;
            }
            pnt2d = pnt2d.WithCoord(u, v - U);
            l = l.WithLocation(pnt2d);
            l = l.WithDirection(Constants.DY2d());
        }
        else
        {
            double angleTemp = 0;
            if (aDirV.Index() == 2)
            {
                angleTemp = _ang;
            }
            l = l.WithLocation(new Point2D(angleTemp, 0));
            l = l.WithDirection(Constants.DY2d());
        }
        var gl2 = new LineCurve2D(l);
        SetThePCurve(_builder.Builder(), ShapeCasts.Edge(aNewEdge), ShapeCasts.Face(aNewFace), orien, gl2);
    }

    public override void SetDirectingPCurve(in Shape aNewFace, ref Shape aNewEdge, in Shape aGenE,
                                            in Shape aGenV, in NumShape aDirE, Orientation orien)
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
                double r = p1.Distance(p2);
                global::Kerf.Engine.Geometry.ElSLib.PlaneParameters(ax3, p2, out u, out v);
                var dx2d = new Direction2D(u, v);
                var axe = new Frame2D(Constants.Origin2d(), dx2d, Constants.DY2d());
                var c = new Circle2D(axe, r);
                var gc = new CircleCurve2D(c);
                thePCurve = gc;
            }
            break;
            case SurfaceKind.Cone:
            {
                Cone cone = AS.Cone();
                global::Kerf.Engine.Geometry.ElSLib.ConeParameters(cone.Position(), cone.RefRadius(), cone.SemiAngle(), p2, out u, out v);
                p22d = p22d.WithCoord(0.0, v);
                var l = new Line2D(p22d, Constants.DX2d());
                var gl = new LineCurve2D(l);
                thePCurve = gl;
            }
            break;
            case SurfaceKind.Sphere:
            {
                Sphere sph = AS.Sphere();
                global::Kerf.Engine.Geometry.ElSLib.SphereParameters(sph.Position(), sph.Radius(), p2, out u, out v);
                p22d = p22d.WithCoord(0.0, v);
                var l = new Line2D(p22d, Constants.DX2d());
                var gl = new LineCurve2D(l);
                thePCurve = gl;
            }
            break;
            case SurfaceKind.Torus:
            {
                Point3D p1;
                double u1, u2, v1, v2;
                Torus tor = AS.Torus();
                var bc = new EdgeCurveAdaptor(ShapeCasts.Edge(aGenE));
                p1 = bc.Value(bc.FirstParameter());
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
                p2 = bc.Value(bc.LastParameter());
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
                u2 = u1 + _ang;
                global::Kerf.Engine.Geometry.ElCLib.AdjustPeriodic(-Math.PI, Math.PI, global::Kerf.Engine.Foundation.Precision.PConfusion(), ref u1, ref u2);
                if (aGenV.Orientation() == Forward)
                {
                    p22d = p22d.WithCoord(u1, v1);
                }
                else
                {
                    p22d = p22d.WithCoord(u1, v2);
                    //  modified by NIZHNY-EAP Thu Mar  2 09:43:32 2000 ___END___
                }
                var l = new Line2D(p22d, Constants.DX2d());
                var gl = new LineCurve2D(l);
                thePCurve = gl;
            }
            break;
            default:
            {
                p22d = p22d.WithCoord(0.0, par);
                var l = new Line2D(p22d, Constants.DX2d());
                var gl = new LineCurve2D(l);
                thePCurve = gl;
            }
            break;
        }
        SetThePCurve(_builder.Builder(), ShapeCasts.Edge(aNewEdge), ShapeCasts.Face(aNewFace), orien, thePCurve);
    }

    // modified by NIZNHY-PKV Tue Jun 14 08:33:55 2011f
    public override Orientation DirectSolid(in Shape aGenS, in NumShape aDirS)
    { // compare the face normal and the direction
        double u1, u2, v1, v2, ux, vx, x, mv2, tol2, tx;
        Orientation or;
        var surf = new FaceSurfaceAdaptor(ShapeCasts.Face(aGenS));
        //
        tol2 = global::Kerf.Engine.Foundation.Precision.Confusion();
        tol2 = tol2 * tol2;
        //
        Point3D pAxeLoc = _axe.Location();
        Direction3D pAxeDir = _axe.Direction();
        //
        u1 = surf.FirstUParameter();
        u2 = surf.LastUParameter();
        v1 = surf.FirstVParameter();
        v2 = surf.LastVParameter();
        //
        tx = 0.5;
        ux = tx * (u1 + u2);
        vx = tx * (v1 + v2);
        surf.D1(ux, vx, out Point3D p, out Vector3D du, out Vector3D dv);
        //
        var v = new Vector3D(pAxeLoc, p);
        v = v.Crossed(new Vector3D(pAxeDir));
        mv2 = v.SquareMagnitude();
        if (mv2 < tol2)
        {
            tx = 0.43213918;
            ux = u1 * (1.0 - tx) + u2 * tx;
            vx = v1 * (1.0 - tx) + v2 * tx;
            surf.D1(ux, vx, out p, out du, out dv);
            v = v.WithXYZ(p.XYZ() - pAxeLoc.XYZ());
            v = v.Crossed(new Vector3D(pAxeDir));
        }
        //
        x = v.DotCross(du, dv);
        or = (x > 0.0) ? Forward : Reversed;
        return or;
    }
    // modified by NIZNHY-PKV Tue Jun 14 08:33:59 2011t

    public override bool GGDShapeIsToAdd(in Shape aNewShape, in Shape aNewSubShape, in Shape aGenS,
                                         in Shape aSubGenS, in NumShape aDirS)
    {
        bool res = true;
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
                return res;
            }
        }
        else
        {
            return res;
        }
    }

    public override bool GDDShapeIsToAdd(in Shape aNewShape, in Shape aNewSubShape, in Shape aGenS,
                                         in NumShape aDirS, in NumShape aSubDirS)
    {
        if (aNewShape.ShapeType() == ShapeType.Solid && aNewSubShape.ShapeType() == ShapeType.Face
            && aGenS.ShapeType() == ShapeType.Face && aDirS.Type() == ShapeType.Edge && aSubDirS.Type() == ShapeType.Vertex)
        {
            return (Math.Abs(_ang - 2 * Math.PI) > global::Kerf.Engine.Foundation.Precision.Angular());
        }
        else if (aNewShape.ShapeType() == ShapeType.Face && aNewSubShape.ShapeType() == ShapeType.Edge
                 && aGenS.ShapeType() == ShapeType.Edge && aDirS.Type() == ShapeType.Edge && aSubDirS.Type() == ShapeType.Vertex)
        {
            var AS = new SurfaceAdaptor(Tool.Surface(ShapeCasts.Face(aNewShape), out Placement _)!);
            if (AS.GetType() == Plane)
            {
                return (Math.Abs(_ang - 2 * Math.PI) > global::Kerf.Engine.Foundation.Precision.Angular());
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
                                        in Shape aSubGenS, in NumShape aDirS)
    {
        if (aNewShape.ShapeType() == ShapeType.Face && aNewSubShape.ShapeType() == ShapeType.Edge
            && aGenS.ShapeType() == ShapeType.Edge && aSubGenS.ShapeType() == ShapeType.Vertex
            && aDirS.Type() == ShapeType.Edge)
        {
            var AS = new SurfaceAdaptor(Tool.Surface(ShapeCasts.Face(aNewShape), out Placement _)!);
            if (AS.GetType() == Plane)
            {
                return (Math.Abs(_ang - 2 * Math.PI) <= global::Kerf.Engine.Foundation.Precision.Angular());
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
        var q = new Quilt();
        q.Add(aNewShape);
        // OCCT.NET: the quilt's shells are read without freezing them (the sweep goes on building on them)
        return q.InternalShells();
    }

    public override bool HasShape(in Shape aGenS, in NumShape aDirS)
    {
        if (aDirS.Type() == ShapeType.Edge && aGenS.ShapeType() == ShapeType.Edge)
        {
            // Verify that the edge has entrails
            Edge edge = ShapeCasts.Edge(aGenS);
            //
            if (Tool.Degenerated(edge))
            {
                return false;
            }

            Curve3D? curve = Tool.Curve(edge, out Placement _, out _, out _);
            if (curve is null)
            {
                return false;
            }

            if (IsInvariant(aGenS))
            {
                return false;
            }

            // Check seem edge
            var faceExp = new Explorer(_genShape, ShapeType.Face);
            for (; faceExp.More(); faceExp.Next())
            {
                Face f = ShapeCasts.Face(faceExp.Current());
                if (BrepTools.IsReallyClosed(edge, f))
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
            var c = new EdgeCurveAdaptor(ShapeCasts.Edge(aGenS));
            if (c.GetType() == Line || c.GetType() == BSplineCurve || c.GetType() == BezierCurve)
            {
                Exploration.Vertices(ShapeCasts.Edge(aGenS), out Vertex v1, out Vertex v2);
                if (IsInvariant(v1) && IsInvariant(v2))
                {
                    if (c.GetType() == Line)
                    {
                        return true;
                    }

                    double tol = Max(Tool.Tolerance(v1), Tool.Tolerance(v2));
                    var lin = new Line3D(_axe.Location(), _axe.Direction());
                    if (c.GetType() == BSplineCurve)
                    {
                        BSplineCurve3D bs = c.BSpline()!;
                        for (int i = 1; i <= bs.PoleCount; i++)
                        {
                            if (lin.Distance(bs.Pole(i)) > tol)
                            {
                                return false;
                            }
                        }
                    }
                    else
                    {
                        BezierCurve3D bz = c.Bezier()!;
                        for (int i = 1; i <= bz.PoleCount; i++)
                        {
                            if (lin.Distance(bz.Pole(i)) > tol)
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
            Point3D p = Tool.Pnt(ShapeCasts.Vertex(aGenS));
            var lin = new Line3D(_axe.Location(), _axe.Direction());
            return (lin.Distance(p) <= Tool.Tolerance(ShapeCasts.Vertex(aGenS)));
        }
        return false;
    }

    /// <summary>The angle of the rotation.</summary>
    public double Angle() => _ang;

    /// <summary>The axis of the rotation.</summary>
    public Axis3D Axe() => _axe;
}
