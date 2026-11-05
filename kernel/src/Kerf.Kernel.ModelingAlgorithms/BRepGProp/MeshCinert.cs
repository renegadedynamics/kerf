// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_MeshCinert.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_MeshCinert.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;

using Kerf.Engine.Numerics;

using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;

using Kerf.Topology;
using Kerf.Engine;

namespace Kerf.Engine.Measurement;

/// <summary>
/// The global properties of a polyline given by its points: for an edge with no exact geometry (no 3D curve or pcurve)
/// but with a polygon.
/// </summary>
internal class MeshCinert : Properties
{
    /// <summary>An empty system.</summary>
    public MeshCinert()
    {
    }

    /// <summary>Sets the reference point.</summary>
    public void SetLocation(in Point3D cLocation) => _loc = cLocation;

    /// <summary>Computes the global properties of the polyline through <paramref name="nodes"/>.</summary>
    public void Perform(NCollection_Array1<Point3D> nodes)
    {
        double ix, iy, iz, ixx, iyy, izz, ixy, ixz, iyz;
        _dim = ix = iy = iz = ixx = iyy = izz = ixy = ixz = iyz = 0.0;

        int order = 2;
        double ds;
        double ur, um, u;
        double x, y, z;
        double xloc, yloc, zloc;
        double upper;
        Xyz p = new(), d;

        var gaussP = new math_Vector(1, order);
        var gaussW = new math_Vector(1, order);

        NumericTables.GaussPoints(order, gaussP);
        NumericTables.GaussWeights(order, gaussW);

        int nIndex = 0;
        for (nIndex = 1; nIndex < nodes.Length(); nIndex++)
        {
            Xyz p1 = nodes[nIndex].XYZ();
            Xyz p2 = nodes[nIndex + 1].XYZ();

            double dimLocal, ixLocal, iyLocal, izLocal, ixxLocal, iyyLocal, izzLocal, ixyLocal, ixzLocal, iyzLocal;
            dimLocal = ixLocal = iyLocal = izLocal = ixxLocal = iyyLocal = izzLocal = ixyLocal = ixzLocal = iyzLocal = 0.0;

            _loc.Coord(out xloc, out yloc, out zloc);

            int i;
            upper = (p2 - p1).Modulus();
            if (upper < Constants.Resolution())
            {
                continue;
            }

            um = 0.5 * upper;
            ur = um;
            d = (p2 - p1) / upper;
            for (i = 1; i <= order; i++)
            {
                u = um + ur * gaussP[i];
                p = p1 + u * d;
                p.Coord(out x, out y, out z);
                x -= xloc;
                y -= yloc;
                z -= zloc;
                ds = gaussW[i];
                dimLocal += ds;
                ixLocal += x * ds;
                iyLocal += y * ds;
                izLocal += z * ds;
                ixyLocal += x * y * ds;
                iyzLocal += y * z * ds;
                ixzLocal += x * z * ds;
                x *= x;
                y *= y;
                z *= z;
                ixxLocal += (y + z) * ds;
                iyyLocal += (x + z) * ds;
                izzLocal += (x + y) * ds;
            }

            dimLocal *= ur;
            ixLocal *= ur;
            iyLocal *= ur;
            izLocal *= ur;
            ixxLocal *= ur;
            iyyLocal *= ur;
            izzLocal *= ur;
            ixyLocal *= ur;
            ixzLocal *= ur;
            iyzLocal *= ur;

            _dim += dimLocal;
            ix += ixLocal;
            iy += iyLocal;
            iz += izLocal;
            ixx += ixxLocal;
            iyy += iyyLocal;
            izz += izzLocal;
            ixy += ixyLocal;
            ixz += ixzLocal;
            iyz += iyzLocal;
        }

        _inertia = new Matrix3x3(new Xyz(ixx, -ixy, -ixz), new Xyz(-ixy, iyy, -iyz), new Xyz(-ixz, -iyz, izz));

        if (Math.Abs(_dim) < Constants.Resolution())
        {
            _g = new Point3D(p);
        }
        else
        {
            _g = _g.WithCoord(ix / _dim, iy / _dim, iz / _dim);
        }
    }

    /// <summary>
    /// The 3D points of a polygon of the edge: its 3D polygon, else its polygon on a triangulation, else its 2D polygon
    /// on a surface (evaluated on the surface), placed by the location; null if the edge has none.
    /// </summary>
    public static NCollection_HArray1<Point3D>? PreparePolygon(in Edge e)
    {
        NCollection_HArray1<Point3D>? thePolyg = null;
        Polygon3D? polyg = Tool.Polygon3D(e, out Placement loc);
        if (polyg is not null)
        {
            NCollection_Array1<Point3D> nodes = polyg.Nodes();
            thePolyg = new NCollection_HArray1<Point3D>(1, nodes.Length());
            int i;
            if (loc.IsIdentity)
            {
                for (i = 1; i <= nodes.Length(); ++i)
                {
                    thePolyg.SetValue(i, nodes[i]);
                }
            }
            else
            {
                Transform3D tr = loc.Transformation;
                for (i = 1; i <= nodes.Length(); ++i)
                {
                    thePolyg.SetValue(i, nodes.Value(i).Transformed(tr));
                }
            }

            return thePolyg;
        }

        // Try to get PolygonOnTriangulation
        Tool.PolygonOnTriangulation(e, out PolygonOnTriangulation? pOnTri, out Triangulation? tri, out loc);
        if (pOnTri is not null)
        {
            int nbNodes = pOnTri.NodeCount;
            thePolyg = new NCollection_HArray1<Point3D>(1, nbNodes);
            NCollection_Array1<int> nodeInds = pOnTri.Nodes();
            int i;
            if (loc.IsIdentity)
            {
                for (i = 1; i <= nbNodes; ++i)
                {
                    thePolyg.SetValue(i, tri!.Node(nodeInds[i]));
                }
            }
            else
            {
                Transform3D tr = loc.Transformation;
                for (i = 1; i <= nbNodes; ++i)
                {
                    thePolyg.SetValue(i, tri!.Node(nodeInds[i]).Transformed(tr));
                }
            }

            return thePolyg;
        }

        // Try to get Polygon2D on Surface
        Tool.PolygonOnSurface(e, out Polygon2D? polyg2D, out Surface? s, out loc);
        if (polyg2D is not null)
        {
            int nbNodes = polyg2D.NodeCount;
            thePolyg = new NCollection_HArray1<Point3D>(1, nbNodes);
            NCollection_Array1<Point2D> nodes2D = polyg2D.Nodes();
            int i;
            if (loc.IsIdentity)
            {
                for (i = 1; i <= nbNodes; ++i)
                {
                    Point2D p2d = nodes2D[i];
                    Point3D p = s!.Value(p2d.X, p2d.Y);
                    thePolyg.SetValue(i, p);
                }
            }
            else
            {
                Transform3D tr = loc.Transformation;
                for (i = 1; i <= nbNodes; ++i)
                {
                    Point2D p2d = nodes2D[i];
                    Point3D p = s!.Value(p2d.X, p2d.Y);
                    p = p.Transformed(tr);
                    thePolyg.SetValue(i, p);
                }
            }
        }

        return thePolyg;
    }
}
