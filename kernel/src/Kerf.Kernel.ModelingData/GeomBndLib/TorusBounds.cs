// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Torus.hxx
// Port of: src/ModelingData/TKGeomBase/GeomBndLib/GeomBndLib_Torus.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Geometry.Bounds;
using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Geometry.Bounds.Computation;

/// <summary>
/// Computes bounding box for a toroidal surface (Geom_ToroidalSurface) through BndLib; the precise box from where each
/// coordinate can be extreme (OCCT searches for them: PSO + Powell, GeomBndLib_OtherSurface).
/// </summary>
internal sealed class TorusBounds : SurfaceBounds.ISurfaceEvaluator
{
    private readonly ToroidalSurface _geom;

    public TorusBounds(ToroidalSurface surf)
    {
        _geom = surf;
    }

    public ToroidalSurface Geometry() => _geom;

    /// <summary>Compute bounding box for full torus.</summary>
    public Box Box(double tol)
    {
        Box box = new();
        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(_geom.Torus, tol, ref box);
        return box;
    }

    /// <summary>Compute bounding box for torus patch [theUMin, theUMax] x [theVMin, theVMax].</summary>
    public Box Box(double uMin, double uMax, double vMin, double vMax, double tol)
    {
        Box box = new();
        global::Kerf.Engine.Geometry.Bounds.BndLib.Add(_geom.Torus, uMin, uMax, vMin, vMax, tol, ref box);
        return box;
    }

    /// <summary>
    /// Compute precise bounding box for torus patch, from the points where a coordinate can be extreme. OCCT searches for
    /// them (PSO and Powell through GeomBndLib_OtherSurface) at about 10 ms a patch; here they are worked out (unit 11E).
    /// As OCCT's does, the box is enlarged by the tolerance, at least Precision::Confusion.
    /// </summary>
    public Box BoxOptimal(double uMin, double uMax, double vMin, double vMax, double tol)
    {
        Torus3D torus = _geom.Torus;
        Box box = new();
        Span<double> low = [0.0, 0.0, 0.0];
        Span<double> high = [0.0, 0.0, 0.0];
        Xyz location = torus.Location.XYZ();
        Xyz x = torus.Position.XDirection.XYZ();
        Xyz y = torus.Position.YDirection.XYZ();
        Xyz z = torus.Position.Direction.XYZ();
        for (int k = 1; k <= 3; k++)
        {
            ExtremesOfCoordinate(location.Coord(k), x.Coord(k), y.Coord(k), z.Coord(k), torus.MajorRadius, torus.MinorRadius,
                                 uMin, uMax, vMin, vMax, out low[k - 1], out high[k - 1]);
        }
        box.Add(new Point3D(low[0], low[1], low[2]));
        box.Add(new Point3D(high[0], high[1], high[2]));
        box.Enlarge(Max(tol, global::Kerf.Engine.Foundation.Precision.Confusion()));
        return box;
    }

    /// <summary>Compute precise bounding box for full torus.</summary>
    public Box BoxOptimal(double tol)
    {
        _geom.ParameterBounds(out double uMin, out double uMax, out double vMin, out double vMax);
        return BoxOptimal(uMin, uMax, vMin, vMax, tol);
    }

    /// <summary>
    /// The least and greatest of one coordinate over the patch. With a, b, c that coordinate of the torus's X, Y and main
    /// directions, it is f(u, v) = o + (R + r cos v)(a cos u + b sin u) + r c sin v. On each side of the patch f is a
    /// constant plus a sinusoid, so its extremes are the side's ends and the sinusoid's peaks. Inside the patch both partial
    /// derivatives vanish, which needs u = atan2(b, a) or that plus pi, with v at a peak of the sinusoid in v there.
    /// </summary>
    private static void ExtremesOfCoordinate(double o, double a, double b, double c, double major, double minor,
                                             double uMin, double uMax, double vMin, double vMax,
                                             out double low, out double high)
    {
        low = RealLast();
        high = -RealLast();
        double uPeak = Math.Atan2(b, a);
        Span<double> us = [uMin, uMax, uPeak, uPeak + M_PI];
        foreach (double u0 in us)
        {
            if (!InRange(u0, uMin, uMax, out double u))
            {
                continue;
            }
            double m = a * Math.Cos(u) + b * Math.Sin(u);
            double vPeak = Math.Atan2(c, m);
            Span<double> vs = [vMin, vMax, vPeak, vPeak + M_PI];
            foreach (double v0 in vs)
            {
                if (InRange(v0, vMin, vMax, out double v))
                {
                    double f = o + (major + minor * Math.Cos(v)) * m + minor * c * Math.Sin(v);
                    low = Min(low, f);
                    high = Max(high, f);
                }
            }
        }
    }

    /// <summary>Moves an angle by whole turns into [first, first + 2 pi), and says whether that is within [first, last].</summary>
    private static bool InRange(double angle, double first, double last, out double turned)
    {
        turned = angle == first || angle == last ? angle : global::Kerf.Engine.Geometry.ElCLib.InPeriod(angle, first, first + 2.0 * M_PI);
        return turned >= first && turned <= last;
    }
}
