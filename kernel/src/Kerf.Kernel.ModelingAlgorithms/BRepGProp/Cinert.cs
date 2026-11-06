// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Cinert.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/BRepGProp/BRepGProp_Cinert.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;
using Kerf.Engine.Numerics;
using Kerf.Engine.Topology.Adaptors;
using Kerf.Geometry;
using static Kerf.Geometry.Continuity;

namespace Kerf.Engine.Measurement;

/// <summary>
/// The global properties of a bounded curve in 3D space (the curve of an edge), by Gauss integration over each interval
/// of continuity. The curve must be at least C1.
/// </summary>
internal class Cinert : Properties
{
    /// <summary>An empty system.</summary>
    public Cinert()
    {
    }

    /// <summary>The global properties of <paramref name="c"/>, inertia at <paramref name="cLocation"/>.</summary>
    public Cinert(EdgeCurveAdaptor c, in Point3D cLocation, bool settled = false)
    {
        _settled = settled;
        SetLocation(cLocation);
        Perform(c);
    }

    // Halve the spans until the length settles (decision 1768). The engine's own callers keep OCCT's fixed points, so
    // that what they decide from a wire's centre doesn't move
    private readonly bool _settled;

    /// <summary>Sets the reference point.</summary>
    public void SetLocation(in Point3D cLocation) => _loc = cLocation;

    /// <summary>Computes the global properties of <paramref name="c"/>.</summary>
    public void Perform(EdgeCurveAdaptor c)
    {
        double ix, iy, iz, ixx, iyy, izz, ixy, ixz, iyz;
        _dim = ix = iy = iz = ixx = iyy = izz = ixy = ixz = iyz = 0.0;

        double lower = EdgeTool.FirstParameter(c);
        double upper = EdgeTool.LastParameter(c);
        int order = Min(EdgeTool.IntegrationOrder(c), NumericTables.GaussPointsMax());

        Point3D p = new(); // value on the curve
        Vector3D v1;        // first derivative on the curve
        double ds;        // curvilign abscissae
        double ur, um, u;
        double x, y, z;
        double xloc, yloc, zloc;

        var gaussP = new math_Vector(1, order);
        var gaussW = new math_Vector(1, order);

        // Recuperation des points de Gauss dans le fichier GaussPoints.
        NumericTables.GaussPoints(order, gaussP);
        NumericTables.GaussWeights(order, gaussW);

        // modified by NIZHNY-MKK  Thu Jun  9 12:13:21 2005.BEGIN
        int nbIntervals = EdgeTool.NbIntervals(c, CN);
        bool bHasIntervals = (nbIntervals > 1);
        var ti = new NCollection_Array1<double>(1, nbIntervals + 1);

        if (bHasIntervals)
        {
            EdgeTool.Intervals(c, ti, CN);
        }
        else
        {
            nbIntervals = 1;
        }

        // the speed along these is constant, so that their first Gauss points are exact
        bool constantSpeed = c.GetType() is Kerf.Geometry.CurveKind.Line or Kerf.Geometry.CurveKind.Circle;

        int nIndex = 0;
        double uu1 = Min(lower, upper);
        double uu2 = Max(lower, upper);

        for (nIndex = 1; nIndex <= nbIntervals; nIndex++)
        {
            if (bHasIntervals)
            {
                lower = Max(ti[nIndex], uu1);
                upper = Min(ti[nIndex + 1], uu2);
            }
            else
            {
                lower = uu1;
                upper = uu2;
            }

            double dimLocal, ixLocal, iyLocal, izLocal, ixxLocal, iyyLocal, izzLocal, ixyLocal, ixzLocal, iyzLocal;
            dimLocal = ixLocal = iyLocal = izLocal = ixxLocal = iyyLocal = izzLocal = ixyLocal = ixzLocal = iyzLocal = 0.0;
            // modified by NIZHNY-MKK  Thu Jun  9 12:13:32 2005.END

            _loc.Coord(out xloc, out yloc, out zloc);

            int i;

            // Calcul des integrales aux points de gauss :
            // OCCT stops at one span of a few points: 1.5% out on the length of an ellipse of ten to one
            int pieces = constantSpeed || !_settled ? 1 : PiecesForLength(c, lower, upper, order, gaussP, gaussW);
            double piece = (upper - lower) / pieces;
            ur = 0.5 * piece;

            for (int j = 0; j < pieces; j++)
            for (i = 1; i <= order; i++)
            {
                um = pieces == 1 ? 0.5 * (upper + lower) : lower + (j + 0.5) * piece;
                u = um + ur * gaussP[i];
                EdgeTool.D1(c, u, out p, out v1);
                ds = v1.Magnitude;
                p.Coord(out x, out y, out z);
                x -= xloc;
                y -= yloc;
                z -= zloc;
                ds *= gaussW[i];
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

            // modified by NIZHNY-MKK  Thu Jun  9 12:13:47 2005.BEGIN
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
        // modified by NIZHNY-MKK  Thu Jun  9 12:13:55 2005.END

        _inertia = new Matrix3x3(new Xyz(ixx, -ixy, -ixz), new Xyz(-ixy, iyy, -iyz), new Xyz(-ixz, -iyz, izz));

        if (Math.Abs(_dim) < Constants.Resolution())
        {
            _g = p;
        }
        else
        {
            _g = _g.WithCoord(ix / _dim, iy / _dim, iz / _dim);
        }
    }

    // How many equal pieces of [lower, upper] give the length to one part in 1e9: one if it has already settled
    private static int PiecesForLength(EdgeCurveAdaptor c, double lower, double upper, int order, math_Vector gaussP, math_Vector gaussW)
    {
        const int mostPieces = 1 << 12;
        double length = LengthOver(c, lower, upper, 1, order, gaussP, gaussW);
        for (int pieces = 2; pieces <= mostPieces; pieces *= 2)
        {
            double finer = LengthOver(c, lower, upper, pieces, order, gaussP, gaussW);
            if (Math.Abs(finer - length) <= 1e-9 * Math.Abs(finer))
            {
                return pieces / 2;
            }

            length = finer;
        }

        return mostPieces;
    }

    private static double LengthOver(EdgeCurveAdaptor c, double lower, double upper, int pieces, int order, math_Vector gaussP, math_Vector gaussW)
    {
        double ur = 0.5 * (upper - lower) / pieces, length = 0;
        for (int j = 0; j < pieces; j++)
        {
            double um = lower + (j + 0.5) * (upper - lower) / pieces;
            for (int i = 1; i <= order; i++)
            {
                EdgeTool.D1(c, um + ur * gaussP[i], out _, out Vector3D v1);
                length += v1.Magnitude * gaussW[i];
            }
        }

        return length * ur;
    }
}
