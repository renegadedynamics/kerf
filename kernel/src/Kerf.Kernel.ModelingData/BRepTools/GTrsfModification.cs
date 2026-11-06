// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_GTrsfModification.hxx
// Port of: src/ModelingData/TKBRep/BRepTools/BRepTools_GTrsfModification.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Engine.Meshing;
using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;
using Kerf.Topology;
using static Kerf.Geometry.TransformForm;

namespace Kerf.Engine.Topology.Tools;

/// <summary>
/// The modification that applies a general transformation (gp_GTrsf, affine) to a shape whose surfaces and curves are
/// all B-splines or Beziers, by transforming their poles. Triangulations and polygons are transformed too.
/// </summary>
/// <remarks>
/// Any other surface or curve throws <see cref="NoSuchObjectException"/>, as in OCCT (BRepBuilderAPI_GTransform
/// converts the shape to NURBS first).
/// </remarks>
internal class GTrsfModification : Modification
{
    private GeneralTransform3D _gTrsf;
    private readonly double _gScale;

    /// <summary>The modification applying <paramref name="t"/>; tolerances grow by the sup norm of its matrix.</summary>
    public GTrsfModification(in GeneralTransform3D t)
    {
        _gTrsf = t;
        // use the sup norm as the maximum dilation for the tolerance
        double loc1, loc2, loc3, loc4;

        loc1 = Max(Math.Abs(t.Value(1, 1)), Math.Abs(t.Value(1, 2)));
        loc2 = Max(Math.Abs(t.Value(2, 1)), Math.Abs(t.Value(2, 2)));
        loc3 = Max(Math.Abs(t.Value(3, 1)), Math.Abs(t.Value(3, 2)));
        loc4 = Max(Math.Abs(t.Value(1, 3)), Math.Abs(t.Value(2, 3)));

        loc1 = Max(loc1, loc2);
        loc2 = Max(loc3, loc4);

        loc1 = Max(loc1, loc2);

        _gScale = Max(loc1, Math.Abs(t.Value(3, 3)));
    }

    /// <summary>The transformation, for reading and writing (a working object).</summary>
    public ref GeneralTransform3D GTrsf() => ref _gTrsf;

    /// <summary>
    /// The face's B-spline or Bezier surface placed and transformed, at the identity location; false if the face has
    /// no surface.
    /// </summary>
    /// <exception cref="NoSuchObjectException">The surface is neither a B-spline nor a Bezier surface.</exception>
    public override bool NewSurface(in Face f,
                                    out Surface? s,
                                    out Placement l,
                                    out double tol,
                                    out bool revWires,
                                    out bool revFace)
    {
        tol = 0.0;
        revWires = revFace = false;
        GeneralTransform3D gtrsf = new GeneralTransform3D().WithVectorialPart(_gTrsf.VectorialPart);
        gtrsf = gtrsf.WithTranslationPart(_gTrsf.TranslationXyz());
        s = Tool.Surface(f, out l);
        if (s is null)
        {
            // processing the case when there is no geometry
            return false;
        }
        s = (Surface)s.Copy();

        tol = Tool.Tolerance(f);
        tol *= _gScale;
        revWires = false;
        revFace = _gTrsf.IsNegative;
        s = (Surface)s.Transformed(l.Transformation);

        if (s.GetType() == typeof(BSplineSurface))
        {
            BSplineSurface s2 = (BSplineSurface)s;
            for (int i = 1; i <= s2.UPoleCount; i++)
            {
                for (int j = 1; j <= s2.VPoleCount; j++)
                {
                    Xyz coor = s2.Pole(i, j).Coord();
                    gtrsf.Transforms(ref coor);
                    Point3D p = new Point3D(coor);
                    s2.SetPole(i, j, p);
                }
            }
        }
        else if (s.GetType() == typeof(BezierSurface))
        {
            BezierSurface s2 = (BezierSurface)s;
            for (int i = 1; i <= s2.UPoleCount; i++)
            {
                for (int j = 1; j <= s2.VPoleCount; j++)
                {
                    Xyz coor = s2.Pole(i, j).Coord();
                    gtrsf.Transforms(ref coor);
                    Point3D p = new Point3D(coor);
                    s2.SetPole(i, j, p);
                }
            }
        }
        else
        {
            throw new NoSuchObjectException("BRepTools_GTrsfModification : Pb no BSpline/Bezier Type Surface");
        }

        l = new Placement();
        return true;
    }

    /// <summary>
    /// The edge's B-spline or Bezier curve placed, transformed and trimmed to the edge's range, at the identity
    /// location; false if the edge has no 3D curve.
    /// </summary>
    /// <exception cref="NoSuchObjectException">The curve is neither a B-spline nor a Bezier curve.</exception>
    public override bool NewCurve(in Edge e, out Curve3D? c, out Placement L, out double tol)
    {
        GeneralTransform3D gtrsf = new GeneralTransform3D().WithVectorialPart(_gTrsf.VectorialPart);
        gtrsf = gtrsf.WithTranslationPart(_gTrsf.TranslationXyz());
        tol = Tool.Tolerance(e) * _gScale;
        c = Tool.Curve(e, out L, out double f, out double l);

        if (c is not null)
        {
            c = (Curve3D)c.Copy().Transformed(L.Transformation);
            if (c.GetType() == typeof(BSplineCurve3D))
            {
                BSplineCurve3D c2 = (BSplineCurve3D)c;
                for (int i = 1; i <= c2.PoleCount; i++)
                {
                    Xyz coor = c2.Pole(i).Coord();
                    gtrsf.Transforms(ref coor);
                    Point3D p = new Point3D(coor);
                    c2.SetPole(i, p);
                }
            }
            else if (c.GetType() == typeof(BezierCurve3D))
            {
                BezierCurve3D c2 = (BezierCurve3D)c;
                for (int i = 1; i <= c2.PoleCount; i++)
                {
                    Xyz coor = c2.Pole(i).Coord();
                    gtrsf.Transforms(ref coor);
                    Point3D p = new Point3D(coor);
                    c2.SetPole(i, p);
                }
            }
            else
            {
                throw new NoSuchObjectException("BRepTools_GTrsfModification : Pb no BSpline/Bezier Type Curve");
            }
            c = new TrimmedCurve3D(c, f, l);
        }
        L = new Placement();
        return c is not null;
    }

    /// <summary>The vertex's point transformed; always true.</summary>
    public override bool NewPoint(in Vertex v, out Point3D p, out double tol)
    {
        Point3D pnt = Tool.Pnt(v);
        tol = Tool.Tolerance(v);
        tol *= _gScale;
        Xyz coor = pnt.Coord();
        _gTrsf.Transforms(ref coor);
        p = new Point3D();
        p = p.WithXYZ(coor);

        return true;
    }

    /// <summary>The edge's pcurve on the face, trimmed to its range; false if it has none.</summary>
    public override bool NewCurve2d(in Edge e,
                                    in Face F,
                                    in Edge newE,
                                    in Face newF,
                                    out Curve2D? c,
                                    out double tol)
    {
        tol = Tool.Tolerance(e);
        tol *= _gScale;
        c = Tool.CurveOnSurface(e, F, out double f, out double l);
        if (c is null)
        {
            // processing the case when there is no geometry
            return false;
        }
        c = new TrimmedCurve2D(c, f, l);
        return true;
    }

    /// <summary>The vertex's parameter on the edge, unchanged; always true.</summary>
    public override bool NewParameter(in Vertex v, in Edge e, ref double p, ref double tol)
    {
        tol = Tool.Tolerance(v);
        tol *= _gScale;
        p = Tool.Parameter(v, e);
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

    /// <summary>The face's triangulation transformed; false if it has none.</summary>
    public override bool NewTriangulation(in Face face, out Triangulation? triangulation)
    {
        triangulation = Tool.Triangulation(face, out Placement loc);
        if (triangulation is null)
        {
            return false;
        }

        GeneralTransform3D gTrsf = new GeneralTransform3D().WithVectorialPart(_gTrsf.VectorialPart);
        gTrsf = gTrsf.WithTranslationPart(_gTrsf.TranslationXyz());
        gTrsf = gTrsf.Multiplied(new GeneralTransform3D(loc.Transformation));

        triangulation = triangulation.Copy();
        triangulation.SetCachedMinMax(new Box()); // clear bounding box
        triangulation.Deflection(triangulation.Deflection() * Math.Abs(_gScale));
        // apply transformation to 3D nodes
        for (int ind = 1; ind <= triangulation.NodeCount; ++ind)
        {
            Point3D p = triangulation.Node(ind);
            Xyz xyz = p.XYZ();
            gTrsf.Transforms(ref xyz);
            p = p.WithXYZ(xyz);
            triangulation.SetNode(ind, p);
        }
        // modify triangles orientation in case of mirror transformation
        if (_gScale < 0.0)
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
                Matrix3x3 mat = gTrsf.VectorialPart;
                mat = mat.WithDiagonal(1.0, 1.0, 1.0);
                Transform3D trsf = new Transform3D(1.0, Rotation, mat, new Xyz(0.0, 0.0, 0.0));
                normal = normal.Transformed(trsf);
                triangulation.SetNormal(ind, normal);
            }
        }

        return true;
    }

    /// <summary>The edge's 3D polygon transformed; false if it has none.</summary>
    public override bool NewPolygon(in Edge edge, out Polygon3D? poly)
    {
        poly = Tool.Polygon3D(edge, out Placement loc);
        if (poly is null)
        {
            return false;
        }

        GeneralTransform3D gTrsf = new GeneralTransform3D().WithVectorialPart(_gTrsf.VectorialPart);
        gTrsf = gTrsf.WithTranslationPart(_gTrsf.TranslationXyz());
        gTrsf = gTrsf.Multiplied(new GeneralTransform3D(loc.Transformation));

        poly = poly.Copy();
        poly.Deflection(poly.Deflection() * Math.Abs(_gScale));
        // transform nodes
        NCollection_Array1<Point3D> nodesArray = poly.ChangeNodes();
        for (int id = nodesArray.Lower(); id <= nodesArray.Upper(); ++id)
        {
            ref Point3D p = ref nodesArray[id];
            Xyz xyz = p.XYZ();
            gTrsf.Transforms(ref xyz);
            p = p.WithXYZ(xyz);
        }
        return true;
    }

    /// <summary>
    /// The edge's polygon on the face's triangulation, copied (null if it has none); false if the face has no
    /// triangulation.
    /// </summary>
    public override bool NewPolygonOnTriangulation(in Edge edge, in Face face, out PolygonOnTriangulation? poly)
    {
        poly = null;
        Triangulation? t = Tool.Triangulation(face, out Placement loc);
        if (t is null)
        {
            return false;
        }

        poly = Tool.PolygonOnTriangulation(edge, t, loc);
        if (poly is not null)
        {
            poly = poly.Copy();
        }
        return true;
    }
}
