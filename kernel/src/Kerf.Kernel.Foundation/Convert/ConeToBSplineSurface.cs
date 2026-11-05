// Port of: src/FoundationClasses/TKMath/Convert/Convert_ConeToBSplineSurface.hxx
// Port of: src/FoundationClasses/TKMath/Convert/Convert_ConeToBSplineSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

using Occt;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// Converts a bounded cone into a rational B-spline surface. The cone's parametrisation is
/// P(U, V) = Loc + V * ZDir + (R + V * tan(Ang)) * (cos(U) * XDir + sin(U) * YDir), U in [0, 2 Pi], Ang the
/// semi-angle.
/// </summary>
internal class ConeToBSplineSurface : ElementarySurfaceToBSplineSurface
{
    private const int TheUDegree = 2;
    private const int TheVDegree = 1;
    private const int TheNbUKnots = 5;
    private const int TheNbVKnots = 2;
    private const int TheNbUPoles = 9;
    private const int TheNbVPoles = 2;

    private static void ComputePoles(double r,
                                     double a,
                                     double u1,
                                     double u2,
                                     double v1,
                                     double v2,
                                     NCollection_Array2<Point3D> poles)
    {
        double deltaU = u2 - u1;

        int i;

        // Number of spans : maximum opening = 150 degrees ( = PI / 1.2 rds)
        int nbUSpans = (int)Math.Truncate(1.2 * deltaU / M_PI) + 1;
        double alfaU = deltaU / (nbUSpans * 2);

        Span<double> x = stackalloc double[TheNbVPoles];
        Span<double> z = stackalloc double[TheNbVPoles];

        x[0] = r + v1 * Math.Sin(a);
        z[0] = v1 * Math.Cos(a);
        x[1] = r + v2 * Math.Sin(a);
        z[1] = v2 * Math.Cos(a);

        double uStart = u1;
        poles[1, 1] = new Point3D(x[0] * Math.Cos(uStart), x[0] * Math.Sin(uStart), z[0]);
        poles[1, 2] = new Point3D(x[1] * Math.Cos(uStart), x[1] * Math.Sin(uStart), z[1]);

        for (i = 1; i <= nbUSpans; i++)
        {
            poles[2 * i, 1] = new Point3D(x[0] * Math.Cos(uStart + alfaU) / Math.Cos(alfaU),
                                         x[0] * Math.Sin(uStart + alfaU) / Math.Cos(alfaU),
                                         z[0]);
            poles[2 * i, 2] = new Point3D(x[1] * Math.Cos(uStart + alfaU) / Math.Cos(alfaU),
                                         x[1] * Math.Sin(uStart + alfaU) / Math.Cos(alfaU),
                                         z[1]);
            poles[2 * i + 1, 1] =
              new Point3D(x[0] * Math.Cos(uStart + 2 * alfaU), x[0] * Math.Sin(uStart + 2 * alfaU), z[0]);
            poles[2 * i + 1, 2] =
              new Point3D(x[1] * Math.Cos(uStart + 2 * alfaU), x[1] * Math.Sin(uStart + 2 * alfaU), z[1]);
            uStart += 2 * alfaU;
        }
    }

    /// <summary>
    /// The B-spline surface equivalent to the cone between the parameters U1, U2 and V1, V2, with the cone's
    /// orientation. Throws <see cref="DomainErrorException"/> if V1 = V2 or U2 - U1 is outside [0, 2 Pi].
    /// </summary>
    public ConeToBSplineSurface(in Cone3D c, double u1, double u2, double v1, double v2)
        : base(TheNbUPoles, TheNbVPoles, TheNbUKnots, TheNbVKnots, TheUDegree, TheVDegree)
    {
        double deltaU = u2 - u1;
        Standard_DomainError_Raise_if((Math.Abs(v2 - v1) <= Math.Abs(Epsilon(v1))) || (deltaU > 2 * M_PI)
                                        || (deltaU < 0.0),
                                      "Convert_ConeToBSplineSurface");

        _isUPeriodic = false;
        _isVPeriodic = false;

        int i, j;
        // construction of cone in the reference mark xOy.

        // Number of spans : maximum opening = 150 degrees ( = PI / 1.2 rds)
        int nbUSpans = (int)Math.Truncate(1.2 * deltaU / M_PI) + 1;
        double alfaU = deltaU / (nbUSpans * 2);

        _nbUPoles = 2 * nbUSpans + 1;
        _nbUKnots = nbUSpans + 1;

        _nbVPoles = 2;
        _nbVKnots = 2;

        double r = c.RefRadius;
        double a = c.SemiAngle;

        ComputePoles(r, a, u1, u2, v1, v2, _poles);

        for (i = 1; i <= _nbUKnots; i++)
        {
            _uKnots[i] = u1 + (i - 1) * 2 * alfaU;
            _uMults[i] = 2;
        }
        _uMults[1]++;
        _uMults[_nbUKnots]++;
        _vKnots[1] = v1;
        _vMults[1] = 2;
        _vKnots[2] = v2;
        _vMults[2] = 2;

        // Replace the bspline in the mark of the sphere.
        // and calculate the weight of the bspline.
        double w1;
        Transform3D trsf = new Transform3D().WithTransformation(c.Position, new CoordinateSystem(Constants.XOY()));

        for (i = 1; i <= _nbUPoles; i++)
        {
            if (i % 2 == 0)
            {
                w1 = Math.Cos(alfaU);
            }
            else
            {
                w1 = 1.0;
            }

            for (j = 1; j <= _nbVPoles; j++)
            {
                _weights[i, j] = w1;
                _poles[i, j] = _poles[i, j].Transformed(trsf);
            }
        }
        FinalizeArrays();
    }

    /// <summary>
    /// The B-spline surface equivalent to the whole cone between the parameters V1 and V2 (periodic in U), with
    /// the cone's orientation. Throws <see cref="DomainErrorException"/> if V1 = V2.
    /// </summary>
    public ConeToBSplineSurface(in Cone3D c, double v1, double v2)
        : base(TheNbUPoles, TheNbVPoles, TheNbUKnots, TheNbVKnots, TheUDegree, TheVDegree)
    {
        Standard_DomainError_Raise_if(Math.Abs(v2 - v1) <= Math.Abs(Epsilon(v1)),
                                      "Convert_ConeToBSplineSurface");

        int i, j;

        _isUPeriodic = true;
        _isVPeriodic = false;

        // construction of the cone in the reference mark xOy.

        double r = c.RefRadius;
        double a = c.SemiAngle;

        ComputePoles(r, a, 0.0, 2.0 * M_PI, v1, v2, _poles);

        _nbUPoles = 6;
        _nbUKnots = 4;
        _nbVPoles = 2;
        _nbVKnots = 2;

        for (i = 1; i <= _nbUKnots; i++)
        {
            _uKnots[i] = (i - 1) * 2.0 * M_PI / 3.0;
            _uMults[i] = 2;
        }
        _vKnots[1] = v1;
        _vMults[1] = 2;
        _vKnots[2] = v2;
        _vMults[2] = 2;

        // replace bspline in the mark of the cone.
        // and calculate the weight of bspline.
        double w;
        Transform3D trsf = new Transform3D().WithTransformation(c.Position, new CoordinateSystem(Constants.XOY()));

        for (i = 1; i <= _nbUPoles; i++)
        {
            if (i % 2 == 0)
            {
                w = 0.5; // = std::cos(pi /3)
            }
            else
            {
                w = 1.0;
            }

            for (j = 1; j <= _nbVPoles; j++)
            {
                _weights[i, j] = w;
                _poles[i, j] = _poles[i, j].Transformed(trsf);
            }
        }
        FinalizeArrays();
    }
}
