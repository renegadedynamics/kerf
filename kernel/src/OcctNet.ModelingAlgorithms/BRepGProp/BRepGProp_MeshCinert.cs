// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_MeshCinert.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_MeshCinert.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Measurement;
using Kerf.Engine.Meshing;

using Kerf.Engine.Numerics;

using Kerf.Engine.Topology.Brep;
using Kerf.Geometry;

using Kerf.Topology;

namespace Occt;

/// <summary>
/// The global properties of a polyline given by its points: for an edge with no exact geometry (no 3D curve or pcurve)
/// but with a polygon.
/// </summary>
public class BRepGProp_MeshCinert : Properties
{
    /// <summary>An empty system.</summary>
    public BRepGProp_MeshCinert()
    {
    }

    /// <summary>Sets the reference point.</summary>
    public void SetLocation(in Point3D CLocation) => _loc = CLocation;

    /// <summary>Computes the global properties of the polyline through <paramref name="theNodes"/>.</summary>
    public void Perform(NCollection_Array1<Point3D> theNodes)
    {
        double Ix, Iy, Iz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
        _dim = Ix = Iy = Iz = Ixx = Iyy = Izz = Ixy = Ixz = Iyz = 0.0;

        int Order = 2;
        double ds;
        double ur, um, u;
        double x, y, z;
        double xloc, yloc, zloc;
        double Upper;
        Xyz P = new(), D;

        var GaussP = new math_Vector(1, Order);
        var GaussW = new math_Vector(1, Order);

        NumericTables.GaussPoints(Order, GaussP);
        NumericTables.GaussWeights(Order, GaussW);

        int nIndex = 0;
        for (nIndex = 1; nIndex < theNodes.Length(); nIndex++)
        {
            Xyz aP1 = theNodes[nIndex].XYZ();
            Xyz aP2 = theNodes[nIndex + 1].XYZ();

            double dimLocal, IxLocal, IyLocal, IzLocal, IxxLocal, IyyLocal, IzzLocal, IxyLocal, IxzLocal, IyzLocal;
            dimLocal = IxLocal = IyLocal = IzLocal = IxxLocal = IyyLocal = IzzLocal = IxyLocal = IxzLocal = IyzLocal = 0.0;

            _loc.Coord(out xloc, out yloc, out zloc);

            int i;
            Upper = (aP2 - aP1).Modulus();
            if (Upper < Constants.Resolution())
            {
                continue;
            }

            um = 0.5 * Upper;
            ur = um;
            D = (aP2 - aP1) / Upper;
            for (i = 1; i <= Order; i++)
            {
                u = um + ur * GaussP[i];
                P = aP1 + u * D;
                P.Coord(out x, out y, out z);
                x -= xloc;
                y -= yloc;
                z -= zloc;
                ds = GaussW[i];
                dimLocal += ds;
                IxLocal += x * ds;
                IyLocal += y * ds;
                IzLocal += z * ds;
                IxyLocal += x * y * ds;
                IyzLocal += y * z * ds;
                IxzLocal += x * z * ds;
                x *= x;
                y *= y;
                z *= z;
                IxxLocal += (y + z) * ds;
                IyyLocal += (x + z) * ds;
                IzzLocal += (x + y) * ds;
            }

            dimLocal *= ur;
            IxLocal *= ur;
            IyLocal *= ur;
            IzLocal *= ur;
            IxxLocal *= ur;
            IyyLocal *= ur;
            IzzLocal *= ur;
            IxyLocal *= ur;
            IxzLocal *= ur;
            IyzLocal *= ur;

            _dim += dimLocal;
            Ix += IxLocal;
            Iy += IyLocal;
            Iz += IzLocal;
            Ixx += IxxLocal;
            Iyy += IyyLocal;
            Izz += IzzLocal;
            Ixy += IxyLocal;
            Ixz += IxzLocal;
            Iyz += IyzLocal;
        }

        _inertia = new Matrix3x3(new Xyz(Ixx, -Ixy, -Ixz), new Xyz(-Ixy, Iyy, -Iyz), new Xyz(-Ixz, -Iyz, Izz));

        if (Math.Abs(_dim) < Constants.Resolution())
        {
            _g = new Point3D(P);
        }
        else
        {
            _g.SetCoord(Ix / _dim, Iy / _dim, Iz / _dim);
        }
    }

    /// <summary>
    /// The 3D points of a polygon of the edge: its 3D polygon, else its polygon on a triangulation, else its 2D polygon
    /// on a surface (evaluated on the surface), placed by the location; null if the edge has none.
    /// </summary>
    public static NCollection_HArray1<Point3D>? PreparePolygon(in Edge theE)
    {
        NCollection_HArray1<Point3D>? thePolyg = null;
        Polygon3D? aPolyg = Tool.Polygon3D(theE, out Placement aLoc);
        if (aPolyg is not null)
        {
            NCollection_Array1<Point3D> aNodes = aPolyg.Nodes();
            thePolyg = new NCollection_HArray1<Point3D>(1, aNodes.Length());
            int i;
            if (aLoc.IsIdentity)
            {
                for (i = 1; i <= aNodes.Length(); ++i)
                {
                    thePolyg.SetValue(i, aNodes[i]);
                }
            }
            else
            {
                Transform3D aTr = aLoc.Transformation();
                for (i = 1; i <= aNodes.Length(); ++i)
                {
                    thePolyg.SetValue(i, aNodes.Value(i).Transformed(aTr));
                }
            }

            return thePolyg;
        }

        // Try to get PolygonOnTriangulation
        Tool.PolygonOnTriangulation(theE, out PolygonOnTriangulation? aPOnTri, out Triangulation? aTri, out aLoc);
        if (aPOnTri is not null)
        {
            int aNbNodes = aPOnTri.NodeCount;
            thePolyg = new NCollection_HArray1<Point3D>(1, aNbNodes);
            NCollection_Array1<int> aNodeInds = aPOnTri.Nodes();
            int i;
            if (aLoc.IsIdentity)
            {
                for (i = 1; i <= aNbNodes; ++i)
                {
                    thePolyg.SetValue(i, aTri!.Node(aNodeInds[i]));
                }
            }
            else
            {
                Transform3D aTr = aLoc.Transformation();
                for (i = 1; i <= aNbNodes; ++i)
                {
                    thePolyg.SetValue(i, aTri!.Node(aNodeInds[i]).Transformed(aTr));
                }
            }

            return thePolyg;
        }

        // Try to get Polygon2D on Surface
        Tool.PolygonOnSurface(theE, out Polygon2D? aPolyg2D, out Surface? aS, out aLoc);
        if (aPolyg2D is not null)
        {
            int aNbNodes = aPolyg2D.NodeCount;
            thePolyg = new NCollection_HArray1<Point3D>(1, aNbNodes);
            NCollection_Array1<Point2D> aNodes2D = aPolyg2D.Nodes();
            int i;
            if (aLoc.IsIdentity)
            {
                for (i = 1; i <= aNbNodes; ++i)
                {
                    Point2D aP2d = aNodes2D[i];
                    Point3D aP = aS!.Value(aP2d.X(), aP2d.Y());
                    thePolyg.SetValue(i, aP);
                }
            }
            else
            {
                Transform3D aTr = aLoc.Transformation();
                for (i = 1; i <= aNbNodes; ++i)
                {
                    Point2D aP2d = aNodes2D[i];
                    Point3D aP = aS!.Value(aP2d.X(), aP2d.Y());
                    aP.Transform(aTr);
                    thePolyg.SetValue(i, aP);
                }
            }
        }

        return thePolyg;
    }
}
