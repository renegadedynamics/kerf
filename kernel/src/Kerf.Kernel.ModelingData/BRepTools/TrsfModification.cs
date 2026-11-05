// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_TrsfModification.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_TrsfModification.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1995-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Geometry.Library;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Geometry.SurfaceKind;
using static Kerf.Geometry.TransformForm;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Topology.Tools;

/// <summary>
/// The modification that applies a transformation (gp_Trsf) to the geometry of a shape: surfaces, curves, points,
/// pcurves (reparametrised where the transformation changes the parametrisation) and, if <see cref="IsCopyMesh"/>,
/// triangulations and polygons.
/// </summary>
internal class TrsfModification : Modification
{
    private Transform3D _trsf;
    private bool _copyMesh;

    /// <summary>The modification applying <paramref name="t"/>; meshes are not copied.</summary>
    public TrsfModification(in Transform3D t)
    {
        _trsf = t;
        _copyMesh = false;
    }

    /// <summary>The transformation, for reading and writing (a working object).</summary>
    public ref Transform3D Trsf() => ref _trsf;

    /// <summary>Whether triangulations and polygons are transformed too, for reading and writing.</summary>
    public ref bool IsCopyMesh() => ref _copyMesh;

    /// <summary>The face's surface transformed (in the frame of its location); false if the face has no surface.</summary>
    public override bool NewSurface(in Face f,
                                    out Surface? s,
                                    out Placement l,
                                    out double tol,
                                    out bool revWires,
                                    out bool revFace)
    {
        tol = 0.0;
        revWires = revFace = false;
        s = Tool.Surface(f, out l);
        if (s is null)
        {
            // processing cases when there is no geometry
            return false;
        }

        tol = Tool.Tolerance(f);
        tol *= Math.Abs(_trsf.ScaleFactor);
        revWires = false;
        revFace = _trsf.IsNegative;

        Transform3D lt = l.Transformation;
        lt = lt.Inverted();
        lt = lt.Multiplied(_trsf);
        lt = lt.Multiplied(l.Transformation);

        s = (Surface)s.Transformed(lt);

        return true;
    }

    /// <summary>The face's triangulation transformed, if meshes are copied.</summary>
    public override bool NewTriangulation(in Face face, out Triangulation? triangulation)
    {
        triangulation = null;
        if (!_copyMesh)
        {
            return false;
        }

        triangulation = Tool.Triangulation(face, out Placement loc);

        if (triangulation is null)
        {
            return false;
        }

        Transform3D trsf = _trsf;
        if (!loc.IsIdentity)
        {
            trsf = loc.Transformation.Inverted() * trsf * loc.Transformation;
        }

        triangulation = triangulation.Copy();
        triangulation.SetCachedMinMax(new Box()); // clear bounding box
        triangulation.Deflection(triangulation.Deflection() * Math.Abs(_trsf.ScaleFactor));
        // apply transformation to 3D nodes
        for (int ind = 1; ind <= triangulation.NodeCount; ++ind)
        {
            Point3D p = triangulation.Node(ind);
            p = p.Transformed(trsf);
            triangulation.SetNode(ind, p);
        }
        // modify 2D nodes
        Surface? surf = Tool.Surface(face, out loc);
        if (triangulation.HasUVNodes && surf is not null)
        {
            for (int ind = 1; ind <= triangulation.NodeCount; ++ind)
            {
                Point2D p2d = triangulation.UVNode(ind);
                double u = p2d.X, v = p2d.Y;
                surf.TransformParameters(ref u, ref v, _trsf);
                p2d = p2d.WithCoord(u, v);
                triangulation.SetUVNode(ind, p2d);
            }
        }
        // modify triangles orientation in case of mirror transformation
        if (_trsf.ScaleFactor < 0.0)
        {
            for (int ind = 1; ind <= triangulation.TriangleCount; ++ind)
            {
                Triangle1 tria = triangulation.Triangle(ind);
                tria.Get(out int n1, out int n2, out int n3);
                tria.Set(n1, n3, n2);
                triangulation.SetTriangle(ind, tria);
            }
        }
        // modify normals
        if (triangulation.HasNormals)
        {
            for (int ind = 1; ind <= triangulation.NodeCount; ++ind)
            {
                Direction3D normal = triangulation.Normal(ind);
                normal = normal.Transformed(trsf);
                triangulation.SetNormal(ind, normal);
            }
        }

        return true;
    }

    /// <summary>The edge's 3D polygon transformed (its parameters too), if meshes are copied.</summary>
    public override bool NewPolygon(in Edge e, out Polygon3D? p)
    {
        p = null;
        if (!_copyMesh)
        {
            return false;
        }

        p = Tool.Polygon3D(e, out Placement loc);
        if (p is null)
        {
            return false;
        }

        Transform3D trsf = _trsf;
        if (!loc.IsIdentity)
        {
            trsf = loc.Transformation.Inverted() * trsf * loc.Transformation;
        }

        p = p.Copy();
        p.Deflection(p.Deflection() * Math.Abs(_trsf.ScaleFactor));
        NCollection_Array1<Point3D> nodesArray = p.ChangeNodes();
        for (int id = nodesArray.Lower(); id <= nodesArray.Upper(); ++id)
        {
            // Applying the transformation to each node of polygon
            nodesArray[id] = nodesArray[id].Transformed(trsf);
        }
        // transform the parametrization
        if (p.HasParameters)
        {
            Curve3D? curve = Tool.Curve(e, out _, out _, out _);
            if (curve is not null)
            {
                double reparametrization = curve.ParametricTransformation(trsf);
                if (Math.Abs(reparametrization - 1.0) > global::Kerf.Engine.Foundation.Precision.PConfusion())
                {
                    NCollection_Array1<double> aParams = p.ChangeParameters();
                    for (int ind = aParams.Lower(); ind <= aParams.Upper(); ++ind)
                    {
                        aParams[ind] *= reparametrization;
                    }
                }
            }
        }
        return true;
    }

    /// <summary>The edge's polygon on the face's triangulation, its parameters mapped to the new pcurve, if meshes are copied.</summary>
    public override bool NewPolygonOnTriangulation(in Edge e, in Face f, out PolygonOnTriangulation? p)
    {
        p = null;
        if (!_copyMesh)
        {
            return false;
        }

        Triangulation? t = Tool.Triangulation(f, out Placement loc);
        if (t is null)
        {
            p = null;
            return false;
        }

        p = Tool.PolygonOnTriangulation(e, t, loc);
        if (p is null)
        {
            return false;
        }
        p = p.Copy();
        p.Deflection(p.Deflection() * Math.Abs(_trsf.ScaleFactor));

        // transform the parametrization
        Surface? surf = Tool.Surface(f, out loc);
        Curve2D? c2d = Tool.CurveOnSurface(e, f, out _, out _);
        if (surf is not null && c2d is not null
            && Math.Abs(Math.Abs(_trsf.ScaleFactor) - 1.0) > Placement.ScalePrec())
        {
            GeneralTransform2D gTrsf = surf.ParametricTransformation(_trsf);
            if (gTrsf.Form != Identity)
            {
                Curve2D? newC2d = global::Kerf.Engine.Geometry.Library.GeomLib.GTransform(c2d, gTrsf);
                for (int ind = 1; ind <= p.NodeCount; ++ind)
                {
                    double param = p.Parameter(ind);
                    Point2D p2d = c2d.Value(param);
                    Xy xy = p2d.XY();
                    gTrsf.Transforms(ref xy);
                    p2d = p2d.WithXY(xy);
                    GeometryLibraryTool.Parameter(newC2d, p2d, p.Deflection(), out param);
                    p.SetParameter(ind, param);
                }
            }
        }

        return true;
    }

    /// <summary>The edge's 3D curve transformed (in the frame of its location); always true.</summary>
    public override bool NewCurve(in Edge e, out Curve3D? c, out Placement l, out double tol)
    {
        c = Tool.Curve(e, out l, out _, out _);

        tol = Tool.Tolerance(e);
        tol *= Math.Abs(_trsf.ScaleFactor);

        Transform3D lt = l.Transformation;
        lt = lt.Inverted();
        lt = lt.Multiplied(_trsf);
        lt = lt.Multiplied(l.Transformation);

        if (c is not null)
        {
            c = (Curve3D)c.Transformed(lt);
        }

        return true;
    }

    /// <summary>The vertex's point transformed; always true.</summary>
    public override bool NewPoint(in Vertex v, out Point3D p, out double tol)
    {
        p = Tool.Pnt(v);
        tol = Tool.Tolerance(v);
        tol *= Math.Abs(_trsf.ScaleFactor);
        p = p.Transformed(_trsf);

        return true;
    }

    /// <summary>
    /// The edge's pcurve on the face, transformed in the parametric space when the transformation scales, with the
    /// range of the new vertex parameters; false on a plane or a face without surface.
    /// </summary>
    public override bool NewCurve2d(in Edge e,
                                    in Face F,
                                    in Edge newE,
                                    in Face newF,
                                    out Curve2D? c,
                                    out double tol)
    {
        c = null;
        tol = Tool.Tolerance(e);
        double scale = _trsf.ScaleFactor;
        tol *= Math.Abs(scale);
        Surface? s = Tool.Surface(F, out _);

        if (s is null)
        {
            // processing the case when the surface (geometry) is deleted
            return false;
        }
        SurfaceAdaptor gAsurf = new SurfaceAdaptor(s);
        if (gAsurf.GetType() == Plane)
        {
            return false;
        }

        Curve2D? newC = Tool.CurveOnSurface(e, F, out double f, out double l);
        if (newC is null)
        {
            return false;
        }

        double newf, newl;

        if (newC.GetType() == typeof(TrimmedCurve2D))
        {
            TrimmedCurve2D tc = (TrimmedCurve2D)newC;
            newC = tc.BasisCurve;
        }

        double fc = newC.FirstParameter, lc = newC.LastParameter;

        if (!newC.IsPeriodic)
        {
            if (fc - f > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                f = fc;
            }
            if (l - lc > global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                l = lc;
            }
            if (Math.Abs(l - f) < global::Kerf.Engine.Foundation.Precision.PConfusion())
            {
                if (Math.Abs(f - fc) < global::Kerf.Engine.Foundation.Precision.PConfusion() && !global::Kerf.Engine.Foundation.Precision.IsInfinite(lc))
                {
                    l = lc;
                }
                else if (!global::Kerf.Engine.Foundation.Precision.IsInfinite(fc))
                {
                    f = fc;
                }
            }
        }

        newf = f;
        newl = l;
        if (Math.Abs(scale) != 1.0)
        {
            newC = new TrimmedCurve2D(newC, f, l);
            GeneralTransform2D gtrsf = s.ParametricTransformation(_trsf);

            if (gtrsf.Form != Identity)
            {
                newC = global::Kerf.Engine.Geometry.Library.GeomLib.GTransform(newC, gtrsf);
                if (newC is null)
                {
                    throw new DomainErrorException("TrsfModification:Error in NewCurve2d");
                }
                newf = newC.FirstParameter;
                newl = newC.LastParameter;
            }
        }
        // sometimes need to readjust the 3D / 2D ranges
        Exploration.Vertices(e, out Vertex v1, out Vertex v2);
        Shape initEFOR = e.Oriented(Forward);                           // skl
        Edge efor = ShapeCasts.Edge(initEFOR /*E.Oriented(TopAbs_FORWARD)*/); // skl
        double tolV = 0.0;
        NewParameter(v1, efor, ref f, ref tolV);
        NewParameter(v2, efor, ref l, ref tolV);
        global::Kerf.Engine.Geometry.Library.GeomLib.SameRange(global::Kerf.Engine.Foundation.Precision.PConfusion(), newC, newf, newl, f, l, ref c);

        return true;
    }

    /// <summary>The vertex's parameter on the edge, transformed with the edge's curve; false for a null vertex.</summary>
    public override bool NewParameter(in Vertex v, in Edge e, ref double p, ref double tol)
    {
        if (v.IsNull)
        {
            return false; // infinite edge may have Null vertex
        }

        tol = Tool.Tolerance(v);
        tol *= Math.Abs(_trsf.ScaleFactor);
        p = Tool.Parameter(v, e);

        Curve3D? c = Tool.Curve(e, out _, out _, out _);
        if (c is not null)
        {
            p = c.TransformedParameter(p, _trsf);
        }

        return true;
    }

    /// <summary>The edge's continuity between the original faces.</summary>
    public override Continuity Continuity(in Edge e,
                                             in Face f1,
                                             in Face f2,
                                             in Edge newE,
                                             in Face newF1,
                                             in Face newF2)
    {
        return Tool.Continuity(e, f1, f2);
    }
}
