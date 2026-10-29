// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_MeshCinert.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_MeshCinert.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The global properties of a polyline given by its points: for an edge with no exact geometry (no 3D curve or pcurve)
/// but with a polygon.
/// </summary>
public class BRepGProp_MeshCinert : GProp_GProps
{
    /// <summary>An empty system.</summary>
    public BRepGProp_MeshCinert()
    {
    }

    /// <summary>Sets the reference point.</summary>
    public void SetLocation(in gp_Pnt CLocation) => loc = CLocation;

    /// <summary>Computes the global properties of the polyline through <paramref name="theNodes"/>.</summary>
    public void Perform(NCollection_Array1<gp_Pnt> theNodes)
    {
        double Ix, Iy, Iz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
        dim = Ix = Iy = Iz = Ixx = Iyy = Izz = Ixy = Ixz = Iyz = 0.0;

        int Order = 2;
        double ds;
        double ur, um, u;
        double x, y, z;
        double xloc, yloc, zloc;
        double Upper;
        gp_XYZ P = new(), D;

        var GaussP = new math_Vector(1, Order);
        var GaussW = new math_Vector(1, Order);

        math.GaussPoints(Order, GaussP);
        math.GaussWeights(Order, GaussW);

        int nIndex = 0;
        for (nIndex = 1; nIndex < theNodes.Length(); nIndex++)
        {
            gp_XYZ aP1 = theNodes[nIndex].XYZ();
            gp_XYZ aP2 = theNodes[nIndex + 1].XYZ();

            double dimLocal, IxLocal, IyLocal, IzLocal, IxxLocal, IyyLocal, IzzLocal, IxyLocal, IxzLocal, IyzLocal;
            dimLocal = IxLocal = IyLocal = IzLocal = IxxLocal = IyyLocal = IzzLocal = IxyLocal = IxzLocal = IyzLocal = 0.0;

            loc.Coord(out xloc, out yloc, out zloc);

            int i;
            Upper = (aP2 - aP1).Modulus();
            if (Upper < gp.Resolution())
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

            dim += dimLocal;
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

        inertia = new gp_Mat(new gp_XYZ(Ixx, -Ixy, -Ixz), new gp_XYZ(-Ixy, Iyy, -Iyz), new gp_XYZ(-Ixz, -Iyz, Izz));

        if (Math.Abs(dim) < gp.Resolution())
        {
            g = new gp_Pnt(P);
        }
        else
        {
            g.SetCoord(Ix / dim, Iy / dim, Iz / dim);
        }
    }

    /// <summary>
    /// The 3D points of a polygon of the edge: its 3D polygon, else its polygon on a triangulation, else its 2D polygon
    /// on a surface (evaluated on the surface), placed by the location; null if the edge has none.
    /// </summary>
    public static NCollection_HArray1<gp_Pnt>? PreparePolygon(in TopoDS_Edge theE)
    {
        NCollection_HArray1<gp_Pnt>? thePolyg = null;
        Poly_Polygon3D? aPolyg = BRep_Tool.Polygon3D(theE, out TopLoc_Location aLoc);
        if (aPolyg is not null)
        {
            NCollection_Array1<gp_Pnt> aNodes = aPolyg.Nodes();
            thePolyg = new NCollection_HArray1<gp_Pnt>(1, aNodes.Length());
            int i;
            if (aLoc.IsIdentity())
            {
                for (i = 1; i <= aNodes.Length(); ++i)
                {
                    thePolyg.SetValue(i, aNodes[i]);
                }
            }
            else
            {
                gp_Trsf aTr = aLoc.Transformation();
                for (i = 1; i <= aNodes.Length(); ++i)
                {
                    thePolyg.SetValue(i, aNodes.Value(i).Transformed(aTr));
                }
            }

            return thePolyg;
        }

        // Try to get PolygonOnTriangulation
        BRep_Tool.PolygonOnTriangulation(theE, out Poly_PolygonOnTriangulation? aPOnTri, out Poly_Triangulation? aTri, out aLoc);
        if (aPOnTri is not null)
        {
            int aNbNodes = aPOnTri.NbNodes();
            thePolyg = new NCollection_HArray1<gp_Pnt>(1, aNbNodes);
            NCollection_Array1<int> aNodeInds = aPOnTri.Nodes();
            int i;
            if (aLoc.IsIdentity())
            {
                for (i = 1; i <= aNbNodes; ++i)
                {
                    thePolyg.SetValue(i, aTri!.Node(aNodeInds[i]));
                }
            }
            else
            {
                gp_Trsf aTr = aLoc.Transformation();
                for (i = 1; i <= aNbNodes; ++i)
                {
                    thePolyg.SetValue(i, aTri!.Node(aNodeInds[i]).Transformed(aTr));
                }
            }

            return thePolyg;
        }

        // Try to get Polygon2D on Surface
        BRep_Tool.PolygonOnSurface(theE, out Poly_Polygon2D? aPolyg2D, out Geom_Surface? aS, out aLoc);
        if (aPolyg2D is not null)
        {
            int aNbNodes = aPolyg2D.NbNodes();
            thePolyg = new NCollection_HArray1<gp_Pnt>(1, aNbNodes);
            NCollection_Array1<gp_Pnt2d> aNodes2D = aPolyg2D.Nodes();
            int i;
            if (aLoc.IsIdentity())
            {
                for (i = 1; i <= aNbNodes; ++i)
                {
                    gp_Pnt2d aP2d = aNodes2D[i];
                    gp_Pnt aP = aS!.Value(aP2d.X(), aP2d.Y());
                    thePolyg.SetValue(i, aP);
                }
            }
            else
            {
                gp_Trsf aTr = aLoc.Transformation();
                for (i = 1; i <= aNbNodes; ++i)
                {
                    gp_Pnt2d aP2d = aNodes2D[i];
                    gp_Pnt aP = aS!.Value(aP2d.X(), aP2d.Y());
                    aP.Transform(aTr);
                    thePolyg.SetValue(i, aP);
                }
            }
        }

        return thePolyg;
    }
}
