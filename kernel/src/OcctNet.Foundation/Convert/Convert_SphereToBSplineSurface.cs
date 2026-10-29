// Port of: src/FoundationClasses/TKMath/Convert/Convert_SphereToBSplineSurface.hxx
// Port of: src/FoundationClasses/TKMath/Convert/Convert_SphereToBSplineSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Converts a bounded sphere into a rational B-spline surface. The sphere's parametrisation is
/// P(U, V) = Loc + Radius * sin(V) * ZDir + Radius * cos(V) * (cos(U) * XDir + sin(U) * YDir), U in [0, 2 Pi],
/// V in [-Pi / 2, Pi / 2].
/// </summary>
public class Convert_SphereToBSplineSurface : Convert_ElementarySurfaceToBSplineSurface
{
    private const int TheUDegree = 2;
    private const int TheVDegree = 2;
    private const int MaxNbUKnots = 4;
    private const int MaxNbVKnots = 3;
    private const int MaxNbUPoles = 7;
    private const int MaxNbVPoles = 5;

    private static void ComputePoles(double R,
                                     double U1,
                                     double U2,
                                     double V1,
                                     double V2,
                                     NCollection_Array2<gp_Pnt> Poles)
    {
        double deltaU = U2 - U1;
        double deltaV = V2 - V1;

        int i, j;

        // Number of spans : maximum opening = 150 degrees ( = PI / 1.2 rds)
        int nbUSpans = (int)Math.Truncate(1.2 * deltaU / M_PI) + 1;
        int nbVSpans = (int)Math.Truncate(1.2 * deltaV / M_PI) + 1;
        double AlfaU = deltaU / (nbUSpans * 2);
        double AlfaV = deltaV / (nbVSpans * 2);

        int nbVP = 2 * nbVSpans + 1;

        Span<double> x = stackalloc double[MaxNbVPoles];
        Span<double> z = stackalloc double[MaxNbVPoles];

        x[0] = R * Math.Cos(V1);
        z[0] = R * Math.Sin(V1);

        double VStart = V1;
        for (i = 1; i <= nbVSpans; i++)
        {
            x[2 * i - 1] = R * Math.Cos(VStart + AlfaV) / Math.Cos(AlfaV);
            z[2 * i - 1] = R * Math.Sin(VStart + AlfaV) / Math.Cos(AlfaV);
            x[2 * i] = R * Math.Cos(VStart + 2 * AlfaV);
            z[2 * i] = R * Math.Sin(VStart + 2 * AlfaV);
            VStart += 2 * AlfaV;
        }

        double UStart = U1;
        for (j = 0; j <= nbVP - 1; j++)
        {
            Poles[1, j + 1] = new gp_Pnt(x[j] * Math.Cos(UStart), x[j] * Math.Sin(UStart), z[j]);
        }

        for (i = 1; i <= nbUSpans; i++)
        {
            for (j = 0; j <= nbVP - 1; j++)
            {
                Poles[2 * i, j + 1] = new gp_Pnt(x[j] * Math.Cos(UStart + AlfaU) / Math.Cos(AlfaU),
                                                 x[j] * Math.Sin(UStart + AlfaU) / Math.Cos(AlfaU),
                                                 z[j]);
                Poles[2 * i + 1, j + 1] =
                  new gp_Pnt(x[j] * Math.Cos(UStart + 2 * AlfaU), x[j] * Math.Sin(UStart + 2 * AlfaU), z[j]);
            }
            UStart += 2 * AlfaU;
        }
    }

    /// <summary>
    /// The B-spline surface equivalent to the sphere between the parameters U1, U2 and V1, V2, with the sphere's
    /// orientation. Throws <see cref="Standard_DomainError"/> if U2 - U1 is outside [0, 2 Pi] or V1, V2 outside
    /// [-Pi / 2, Pi / 2].
    /// </summary>
    public Convert_SphereToBSplineSurface(in gp_Sphere Sph, double U1, double U2, double V1, double V2)
        : base(MaxNbUPoles, MaxNbVPoles, MaxNbUKnots, MaxNbVKnots, TheUDegree, TheVDegree)
    {
        double deltaU = U2 - U1;
        double deltaV = V2 - V1;
        Standard_DomainError_Raise_if((deltaU > 2 * M_PI) || (deltaU < 0.0) || (V1 < -M_PI / 2.0)
                                        || (V2 > M_PI / 2),
                                      "Convert_SphereToBSplineSurface");

        myIsUPeriodic = false;
        myIsVPeriodic = false;

        int i, j;
        // construction of the sphere in the reference mark xOy.

        // Number of spans : maximum opening = 150 degrees ( = PI / 1.2 rds)
        int nbUSpans = (int)Math.Truncate(1.2 * deltaU / M_PI) + 1;
        int nbVSpans = (int)Math.Truncate(1.2 * deltaV / M_PI) + 1;
        double AlfaU = deltaU / (nbUSpans * 2);
        double AlfaV = deltaV / (nbVSpans * 2);

        myNbUPoles = 2 * nbUSpans + 1;
        myNbVPoles = 2 * nbVSpans + 1;
        myNbUKnots = nbUSpans + 1;
        myNbVKnots = nbVSpans + 1;

        double R = Sph.Radius();

        ComputePoles(R, U1, U2, V1, V2, myPoles);

        for (i = 1; i <= myNbUKnots; i++)
        {
            myUKnots[i] = U1 + (i - 1) * 2 * AlfaU;
            myUMults[i] = 2;
        }
        myUMults[1]++;
        myUMults[myNbUKnots]++;
        for (i = 1; i <= myNbVKnots; i++)
        {
            myVKnots[i] = V1 + (i - 1) * 2 * AlfaV;
            myVMults[i] = 2;
        }
        myVMults[1]++;
        myVMults[myNbVKnots]++;

        // Replace the bspline in the reference of the sphere.
        // and calculate the weight of the bspline.
        double W1, W2;
        gp_Trsf Trsf = new();
        Trsf.SetTransformation(Sph.Position(), new gp_Ax3(gp.XOY()));

        for (i = 1; i <= myNbUPoles; i++)
        {
            if (i % 2 == 0)
            {
                W1 = Math.Cos(AlfaU);
            }
            else
            {
                W1 = 1.0;
            }

            for (j = 1; j <= myNbVPoles; j++)
            {
                if (j % 2 == 0)
                {
                    W2 = Math.Cos(AlfaV);
                }
                else
                {
                    W2 = 1.0;
                }

                myWeights[i, j] = W1 * W2;
                myPoles[i, j].Transform(Trsf);
            }
        }
        FinalizeArrays();
    }

    /// <summary>
    /// The B-spline surface equivalent to the sphere between <paramref name="Param1"/> and
    /// <paramref name="Param2"/> in U (<paramref name="UTrim"/>; whole in V) or in V (whole and periodic in U),
    /// with the sphere's orientation. Throws <see cref="Standard_DomainError"/> if Param2 - Param1 is outside
    /// [0, 2 Pi].
    /// </summary>
    public Convert_SphereToBSplineSurface(in gp_Sphere Sph, double Param1, double Param2, bool UTrim = true)
        : base(MaxNbUPoles, MaxNbVPoles, MaxNbUKnots, MaxNbVKnots, TheUDegree, TheVDegree)
    {
        double delta = Param2 - Param1;
        Standard_DomainError_Raise_if((delta > 2 * M_PI) || (delta < 0.0),
                                      "Convert_SphereToBSplineSurface");

        int i, j;
        double deltaU, deltaV;

        myIsUPeriodic = !UTrim;
        myIsVPeriodic = false;

        double R = Sph.Radius();

        double W1, W2, CosU, CosV;

        if (myIsUPeriodic)
        {
            ComputePoles(R, 0.0, 2.0 * M_PI, Param1, Param2, myPoles);

            myNbUPoles = 6;
            myNbUKnots = 4;

            deltaV = Param2 - Param1;
            int nbVSpans = (int)Math.Truncate(1.2 * deltaV / M_PI) + 1;
            double AlfaV = deltaV / (nbVSpans * 2);
            myNbVPoles = 2 * nbVSpans + 1;
            myNbVKnots = nbVSpans + 1;

            for (i = 1; i <= myNbUKnots; i++)
            {
                myUKnots[i] = (i - 1) * 2.0 * M_PI / 3.0;
                myUMults[i] = 2;
            }
            for (i = 1; i <= myNbVKnots; i++)
            {
                myVKnots[i] = Param1 + (i - 1) * 2 * AlfaV;
                myVMults[i] = 2;
            }
            myVMults[1]++;
            myVMults[myNbVKnots]++;

            CosU = 0.5; // = std::cos(pi /3)
            CosV = Math.Cos(AlfaV);
        }
        else
        {
            ComputePoles(R, Param1, Param2, -M_PI / 2.0, M_PI / 2.0, myPoles);

            myNbVPoles = 5;
            myNbVKnots = 3;

            deltaU = Param2 - Param1;
            int nbUSpans = (int)Math.Truncate(1.2 * deltaU / M_PI) + 1;
            double AlfaU = deltaU / (nbUSpans * 2);
            myNbUPoles = 2 * nbUSpans + 1;
            myNbUKnots = nbUSpans + 1;

            myVKnots[1] = -M_PI / 2.0;
            myVMults[1] = 3;
            myVKnots[2] = 0.0;
            myVMults[2] = 2;
            myVKnots[3] = M_PI / 2.0;
            myVMults[3] = 3;
            for (i = 1; i <= myNbUKnots; i++)
            {
                myUKnots[i] = Param1 + (i - 1) * 2 * AlfaU;
                myUMults[i] = 2;
            }
            myUMults[1]++;
            myUMults[myNbUKnots]++;

            CosV = 0.5; // = std::cos(pi /3)
            CosU = Math.Cos(AlfaU);
        }

        // Replace the bspline in the mark of the sphere.
        // and calculate the weight of bspline.
        gp_Trsf Trsf = new();
        Trsf.SetTransformation(Sph.Position(), new gp_Ax3(gp.XOY()));

        for (i = 1; i <= myNbUPoles; i++)
        {
            if (i % 2 == 0)
            {
                W1 = CosU;
            }
            else
            {
                W1 = 1.0;
            }

            for (j = 1; j <= myNbVPoles; j++)
            {
                if (j % 2 == 0)
                {
                    W2 = CosV;
                }
                else
                {
                    W2 = 1.0;
                }

                myWeights[i, j] = W1 * W2;
                myPoles[i, j].Transform(Trsf);
            }
        }
        FinalizeArrays();
    }

    /// <summary>The B-spline surface equivalent to the whole sphere (periodic in U), with the sphere's orientation.</summary>
    public Convert_SphereToBSplineSurface(in gp_Sphere Sph)
        : base(MaxNbUPoles, MaxNbVPoles, MaxNbUKnots, MaxNbVKnots, TheUDegree, TheVDegree)
    {
        myIsUPeriodic = true;
        myIsVPeriodic = false;

        double W1, W2;
        int i, j;

        myNbUPoles = 6;
        myNbVPoles = 5;
        myNbUKnots = 4;
        myNbVKnots = 3;

        // Construction of the sphere in the reference mark xOy.

        double R = Sph.Radius();

        ComputePoles(R, 0.0, 2.0 * M_PI, -M_PI / 2.0, M_PI / 2.0, myPoles);

        myUKnots[1] = 0.0;
        myUKnots[2] = 2.0 * M_PI / 3.0;
        myUKnots[3] = 4.0 * M_PI / 3.0;
        myUKnots[4] = 2.0 * M_PI;
        myVKnots[1] = -M_PI / 2.0;
        myVKnots[2] = 0.0;
        myVKnots[3] = M_PI / 2.0;
        for (i = 1; i <= 4; i++)
        {
            myUMults[i] = 2;
        }
        myVMults[1] = myVMults[3] = 3;
        myVMults[2] = 2;

        // Replace the bspline in the mark of the sphere.
        // and calculate the weight of the bspline.
        gp_Trsf Trsf = new();
        Trsf.SetTransformation(Sph.Position(), new gp_Ax3(gp.XOY()));

        for (i = 1; i <= myNbUPoles; i++)
        {
            if (i % 2 == 0)
            {
                W1 = 0.5;
            }
            else
            {
                W1 = 1.0;
            }

            for (j = 1; j <= myNbVPoles; j++)
            {
                if (j % 2 == 0)
                {
                    W2 = Math.Sqrt(2.0) / 2.0;
                }
                else
                {
                    W2 = 1.0;
                }

                myWeights[i, j] = W1 * W2;
                myPoles[i, j].Transform(Trsf);
            }
        }
        FinalizeArrays();
    }
}
