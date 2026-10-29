// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Cinert.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Cinert.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.GeomAbs_Shape;

namespace Occt;

/// <summary>
/// The global properties of a bounded curve in 3D space (the curve of an edge), by Gauss integration over each interval
/// of continuity. The curve must be at least C1.
/// </summary>
/// <remarks>
/// OCCT takes a BRepAdaptor_Curve; the port takes its base class, <see cref="GeomAdaptor_TransformedCurve"/>, until
/// unit 3B ports BRepAdaptor_Curve (TODO(3B)).
/// </remarks>
public class BRepGProp_Cinert : GProp_GProps
{
    /// <summary>An empty system.</summary>
    public BRepGProp_Cinert()
    {
    }

    /// <summary>The global properties of <paramref name="C"/>, inertia at <paramref name="CLocation"/>.</summary>
    public BRepGProp_Cinert(GeomAdaptor_TransformedCurve C, in gp_Pnt CLocation)
    {
        SetLocation(CLocation);
        Perform(C);
    }

    /// <summary>Sets the reference point.</summary>
    public void SetLocation(in gp_Pnt CLocation) => loc = CLocation;

    /// <summary>Computes the global properties of <paramref name="C"/>.</summary>
    public void Perform(GeomAdaptor_TransformedCurve C)
    {
        double Ix, Iy, Iz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz;
        dim = Ix = Iy = Iz = Ixx = Iyy = Izz = Ixy = Ixz = Iyz = 0.0;

        double Lower = BRepGProp_EdgeTool.FirstParameter(C);
        double Upper = BRepGProp_EdgeTool.LastParameter(C);
        int Order = Min(BRepGProp_EdgeTool.IntegrationOrder(C), math.GaussPointsMax());

        gp_Pnt P = new(); // value on the curve
        gp_Vec V1;        // first derivative on the curve
        double ds;        // curvilign abscissae
        double ur, um, u;
        double x, y, z;
        double xloc, yloc, zloc;

        var GaussP = new math_Vector(1, Order);
        var GaussW = new math_Vector(1, Order);

        // Recuperation des points de Gauss dans le fichier GaussPoints.
        math.GaussPoints(Order, GaussP);
        math.GaussWeights(Order, GaussW);

        // modified by NIZHNY-MKK  Thu Jun  9 12:13:21 2005.BEGIN
        int nbIntervals = BRepGProp_EdgeTool.NbIntervals(C, GeomAbs_CN);
        bool bHasIntervals = (nbIntervals > 1);
        var TI = new NCollection_Array1<double>(1, nbIntervals + 1);

        if (bHasIntervals)
        {
            BRepGProp_EdgeTool.Intervals(C, TI, GeomAbs_CN);
        }
        else
        {
            nbIntervals = 1;
        }

        int nIndex = 0;
        double UU1 = Min(Lower, Upper);
        double UU2 = Max(Lower, Upper);

        for (nIndex = 1; nIndex <= nbIntervals; nIndex++)
        {
            if (bHasIntervals)
            {
                Lower = Max(TI[nIndex], UU1);
                Upper = Min(TI[nIndex + 1], UU2);
            }
            else
            {
                Lower = UU1;
                Upper = UU2;
            }

            double dimLocal, IxLocal, IyLocal, IzLocal, IxxLocal, IyyLocal, IzzLocal, IxyLocal, IxzLocal, IyzLocal;
            dimLocal = IxLocal = IyLocal = IzLocal = IxxLocal = IyyLocal = IzzLocal = IxyLocal = IxzLocal = IyzLocal = 0.0;
            // modified by NIZHNY-MKK  Thu Jun  9 12:13:32 2005.END

            loc.Coord(out xloc, out yloc, out zloc);

            int i;

            // Calcul des integrales aux points de gauss :
            um = 0.5 * (Upper + Lower);
            ur = 0.5 * (Upper - Lower);

            for (i = 1; i <= Order; i++)
            {
                u = um + ur * GaussP[i];
                BRepGProp_EdgeTool.D1(C, u, out P, out V1);
                ds = V1.Magnitude();
                P.Coord(out x, out y, out z);
                x -= xloc;
                y -= yloc;
                z -= zloc;
                ds *= GaussW[i];
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

            // modified by NIZHNY-MKK  Thu Jun  9 12:13:47 2005.BEGIN
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
        // modified by NIZHNY-MKK  Thu Jun  9 12:13:55 2005.END

        inertia = new gp_Mat(new gp_XYZ(Ixx, -Ixy, -Ixz), new gp_XYZ(-Ixy, Iyy, -Iyz), new gp_XYZ(-Ixz, -Iyz, Izz));

        if (Math.Abs(dim) < gp.Resolution())
        {
            g = P;
        }
        else
        {
            g.SetCoord(Ix / dim, Iy / dim, Iz / dim);
        }
    }
}
