// Port of: src/FoundationClasses/TKMath/Convert/Convert_SphereToBSplineSurface.hxx
// Port of: src/FoundationClasses/TKMath/Convert/Convert_SphereToBSplineSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;

using Occt;

namespace Kerf.Engine.Geometry.Conversion;

/// <summary>
/// Converts a bounded sphere into a rational B-spline surface. The sphere's parametrisation is
/// P(U, V) = Loc + Radius * sin(V) * ZDir + Radius * cos(V) * (cos(U) * XDir + sin(U) * YDir), U in [0, 2 Pi],
/// V in [-Pi / 2, Pi / 2].
/// </summary>
internal class SphereToBSplineSurface : ElementarySurfaceToBSplineSurface
{
    private const int TheUDegree = 2;
    private const int TheVDegree = 2;
    private const int MaxNbUKnots = 4;
    private const int MaxNbVKnots = 3;
    private const int MaxNbUPoles = 7;
    private const int MaxNbVPoles = 5;

    private static void ComputePoles(double r,
                                     double u1,
                                     double u2,
                                     double v1,
                                     double v2,
                                     NCollection_Array2<Point3D> poles)
    {
        double deltaU = u2 - u1;
        double deltaV = v2 - v1;

        int i, j;

        // Number of spans : maximum opening = 150 degrees ( = PI / 1.2 rds)
        int nbUSpans = (int)Math.Truncate(1.2 * deltaU / M_PI) + 1;
        int nbVSpans = (int)Math.Truncate(1.2 * deltaV / M_PI) + 1;
        double alfaU = deltaU / (nbUSpans * 2);
        double alfaV = deltaV / (nbVSpans * 2);

        int nbVP = 2 * nbVSpans + 1;

        Span<double> x = stackalloc double[MaxNbVPoles];
        Span<double> z = stackalloc double[MaxNbVPoles];

        x[0] = r * Math.Cos(v1);
        z[0] = r * Math.Sin(v1);

        double vStart = v1;
        for (i = 1; i <= nbVSpans; i++)
        {
            x[2 * i - 1] = r * Math.Cos(vStart + alfaV) / Math.Cos(alfaV);
            z[2 * i - 1] = r * Math.Sin(vStart + alfaV) / Math.Cos(alfaV);
            x[2 * i] = r * Math.Cos(vStart + 2 * alfaV);
            z[2 * i] = r * Math.Sin(vStart + 2 * alfaV);
            vStart += 2 * alfaV;
        }

        double uStart = u1;
        for (j = 0; j <= nbVP - 1; j++)
        {
            poles[1, j + 1] = new Point3D(x[j] * Math.Cos(uStart), x[j] * Math.Sin(uStart), z[j]);
        }

        for (i = 1; i <= nbUSpans; i++)
        {
            for (j = 0; j <= nbVP - 1; j++)
            {
                poles[2 * i, j + 1] = new Point3D(x[j] * Math.Cos(uStart + alfaU) / Math.Cos(alfaU),
                                                 x[j] * Math.Sin(uStart + alfaU) / Math.Cos(alfaU),
                                                 z[j]);
                poles[2 * i + 1, j + 1] =
                  new Point3D(x[j] * Math.Cos(uStart + 2 * alfaU), x[j] * Math.Sin(uStart + 2 * alfaU), z[j]);
            }
            uStart += 2 * alfaU;
        }
    }

    /// <summary>
    /// The B-spline surface equivalent to the sphere between the parameters U1, U2 and V1, V2, with the sphere's
    /// orientation. Throws <see cref="DomainErrorException"/> if U2 - U1 is outside [0, 2 Pi] or V1, V2 outside
    /// [-Pi / 2, Pi / 2].
    /// </summary>
    public SphereToBSplineSurface(in Sphere3D sph, double u1, double u2, double v1, double v2)
        : base(MaxNbUPoles, MaxNbVPoles, MaxNbUKnots, MaxNbVKnots, TheUDegree, TheVDegree)
    {
        double deltaU = u2 - u1;
        double deltaV = v2 - v1;
        Standard_DomainError_Raise_if((deltaU > 2 * M_PI) || (deltaU < 0.0) || (v1 < -M_PI / 2.0)
                                        || (v2 > M_PI / 2),
                                      "Convert_SphereToBSplineSurface");

        _isUPeriodic = false;
        _isVPeriodic = false;

        int i, j;
        // construction of the sphere in the reference mark xOy.

        // Number of spans : maximum opening = 150 degrees ( = PI / 1.2 rds)
        int nbUSpans = (int)Math.Truncate(1.2 * deltaU / M_PI) + 1;
        int nbVSpans = (int)Math.Truncate(1.2 * deltaV / M_PI) + 1;
        double alfaU = deltaU / (nbUSpans * 2);
        double alfaV = deltaV / (nbVSpans * 2);

        _nbUPoles = 2 * nbUSpans + 1;
        _nbVPoles = 2 * nbVSpans + 1;
        _nbUKnots = nbUSpans + 1;
        _nbVKnots = nbVSpans + 1;

        double r = sph.Radius;

        ComputePoles(r, u1, u2, v1, v2, _poles);

        for (i = 1; i <= _nbUKnots; i++)
        {
            _uKnots[i] = u1 + (i - 1) * 2 * alfaU;
            _uMults[i] = 2;
        }
        _uMults[1]++;
        _uMults[_nbUKnots]++;
        for (i = 1; i <= _nbVKnots; i++)
        {
            _vKnots[i] = v1 + (i - 1) * 2 * alfaV;
            _vMults[i] = 2;
        }
        _vMults[1]++;
        _vMults[_nbVKnots]++;

        // Replace the bspline in the reference of the sphere.
        // and calculate the weight of the bspline.
        double w1, w2;
        Transform3D trsf = new Transform3D().WithTransformation(sph.Position, new CoordinateSystem(Constants.XOY()));

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
                if (j % 2 == 0)
                {
                    w2 = Math.Cos(alfaV);
                }
                else
                {
                    w2 = 1.0;
                }

                _weights[i, j] = w1 * w2;
                _poles[i, j] = _poles[i, j].Transformed(trsf);
            }
        }
        FinalizeArrays();
    }

    /// <summary>
    /// The B-spline surface equivalent to the sphere between <paramref name="param1"/> and
    /// <paramref name="param2"/> in U (<paramref name="uTrim"/>; whole in V) or in V (whole and periodic in U),
    /// with the sphere's orientation. Throws <see cref="DomainErrorException"/> if Param2 - Param1 is outside
    /// [0, 2 Pi].
    /// </summary>
    public SphereToBSplineSurface(in Sphere3D sph, double param1, double param2, bool uTrim = true)
        : base(MaxNbUPoles, MaxNbVPoles, MaxNbUKnots, MaxNbVKnots, TheUDegree, TheVDegree)
    {
        double delta = param2 - param1;
        Standard_DomainError_Raise_if((delta > 2 * M_PI) || (delta < 0.0),
                                      "Convert_SphereToBSplineSurface");

        int i, j;
        double deltaU, deltaV;

        _isUPeriodic = !uTrim;
        _isVPeriodic = false;

        double r = sph.Radius;

        double w1, w2, cosU, cosV;

        if (_isUPeriodic)
        {
            ComputePoles(r, 0.0, 2.0 * M_PI, param1, param2, _poles);

            _nbUPoles = 6;
            _nbUKnots = 4;

            deltaV = param2 - param1;
            int nbVSpans = (int)Math.Truncate(1.2 * deltaV / M_PI) + 1;
            double alfaV = deltaV / (nbVSpans * 2);
            _nbVPoles = 2 * nbVSpans + 1;
            _nbVKnots = nbVSpans + 1;

            for (i = 1; i <= _nbUKnots; i++)
            {
                _uKnots[i] = (i - 1) * 2.0 * M_PI / 3.0;
                _uMults[i] = 2;
            }
            for (i = 1; i <= _nbVKnots; i++)
            {
                _vKnots[i] = param1 + (i - 1) * 2 * alfaV;
                _vMults[i] = 2;
            }
            _vMults[1]++;
            _vMults[_nbVKnots]++;

            cosU = 0.5; // = std::cos(pi /3)
            cosV = Math.Cos(alfaV);
        }
        else
        {
            ComputePoles(r, param1, param2, -M_PI / 2.0, M_PI / 2.0, _poles);

            _nbVPoles = 5;
            _nbVKnots = 3;

            deltaU = param2 - param1;
            int nbUSpans = (int)Math.Truncate(1.2 * deltaU / M_PI) + 1;
            double alfaU = deltaU / (nbUSpans * 2);
            _nbUPoles = 2 * nbUSpans + 1;
            _nbUKnots = nbUSpans + 1;

            _vKnots[1] = -M_PI / 2.0;
            _vMults[1] = 3;
            _vKnots[2] = 0.0;
            _vMults[2] = 2;
            _vKnots[3] = M_PI / 2.0;
            _vMults[3] = 3;
            for (i = 1; i <= _nbUKnots; i++)
            {
                _uKnots[i] = param1 + (i - 1) * 2 * alfaU;
                _uMults[i] = 2;
            }
            _uMults[1]++;
            _uMults[_nbUKnots]++;

            cosV = 0.5; // = std::cos(pi /3)
            cosU = Math.Cos(alfaU);
        }

        // Replace the bspline in the mark of the sphere.
        // and calculate the weight of bspline.
        Transform3D trsf = new Transform3D().WithTransformation(sph.Position, new CoordinateSystem(Constants.XOY()));

        for (i = 1; i <= _nbUPoles; i++)
        {
            if (i % 2 == 0)
            {
                w1 = cosU;
            }
            else
            {
                w1 = 1.0;
            }

            for (j = 1; j <= _nbVPoles; j++)
            {
                if (j % 2 == 0)
                {
                    w2 = cosV;
                }
                else
                {
                    w2 = 1.0;
                }

                _weights[i, j] = w1 * w2;
                _poles[i, j] = _poles[i, j].Transformed(trsf);
            }
        }
        FinalizeArrays();
    }

    /// <summary>The B-spline surface equivalent to the whole sphere (periodic in U), with the sphere's orientation.</summary>
    public SphereToBSplineSurface(in Sphere3D sph)
        : base(MaxNbUPoles, MaxNbVPoles, MaxNbUKnots, MaxNbVKnots, TheUDegree, TheVDegree)
    {
        _isUPeriodic = true;
        _isVPeriodic = false;

        double w1, w2;
        int i, j;

        _nbUPoles = 6;
        _nbVPoles = 5;
        _nbUKnots = 4;
        _nbVKnots = 3;

        // Construction of the sphere in the reference mark xOy.

        double r = sph.Radius;

        ComputePoles(r, 0.0, 2.0 * M_PI, -M_PI / 2.0, M_PI / 2.0, _poles);

        _uKnots[1] = 0.0;
        _uKnots[2] = 2.0 * M_PI / 3.0;
        _uKnots[3] = 4.0 * M_PI / 3.0;
        _uKnots[4] = 2.0 * M_PI;
        _vKnots[1] = -M_PI / 2.0;
        _vKnots[2] = 0.0;
        _vKnots[3] = M_PI / 2.0;
        for (i = 1; i <= 4; i++)
        {
            _uMults[i] = 2;
        }
        _vMults[1] = _vMults[3] = 3;
        _vMults[2] = 2;

        // Replace the bspline in the mark of the sphere.
        // and calculate the weight of the bspline.
        Transform3D trsf = new Transform3D().WithTransformation(sph.Position, new CoordinateSystem(Constants.XOY()));

        for (i = 1; i <= _nbUPoles; i++)
        {
            if (i % 2 == 0)
            {
                w1 = 0.5;
            }
            else
            {
                w1 = 1.0;
            }

            for (j = 1; j <= _nbVPoles; j++)
            {
                if (j % 2 == 0)
                {
                    w2 = Math.Sqrt(2.0) / 2.0;
                }
                else
                {
                    w2 = 1.0;
                }

                _weights[i, j] = w1 * w2;
                _poles[i, j] = _poles[i, j].Transformed(trsf);
            }
        }
        FinalizeArrays();
    }
}
