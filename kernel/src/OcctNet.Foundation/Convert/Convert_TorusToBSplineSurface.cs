// Port of: src/FoundationClasses/TKMath/Convert/Convert_TorusToBSplineSurface.hxx
// Port of: src/FoundationClasses/TKMath/Convert/Convert_TorusToBSplineSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Converts a bounded torus into a rational B-spline surface. The torus's parametrisation is
/// P(U, V) = Loc + MinorRadius * sin(V) * ZDir + (MajorRadius + MinorRadius * cos(V)) * (cos(U) * XDir +
/// sin(U) * YDir), U and V in [0, 2 Pi].
/// </summary>
public class Convert_TorusToBSplineSurface : Convert_ElementarySurfaceToBSplineSurface
{
    private const int TheUDegree = 2;
    private const int TheVDegree = 2;
    private const int MaxNbUKnots = 4;
    private const int MaxNbVKnots = 4;
    private const int MaxNbUPoles = 7;
    private const int MaxNbVPoles = 7;

    private static void ComputePoles(double R,
                                     double r,
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

        x[0] = R + r * Math.Cos(V1);
        z[0] = r * Math.Sin(V1);

        double VStart = V1;
        for (i = 1; i <= nbVSpans; i++)
        {
            x[2 * i - 1] = R + r * Math.Cos(VStart + AlfaV) / Math.Cos(AlfaV);
            z[2 * i - 1] = r * Math.Sin(VStart + AlfaV) / Math.Cos(AlfaV);
            x[2 * i] = R + r * Math.Cos(VStart + 2 * AlfaV);
            z[2 * i] = r * Math.Sin(VStart + 2 * AlfaV);
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
    /// The B-spline surface equivalent to the torus between the parameters U1, U2 and V1, V2, with the torus's
    /// orientation. Throws <see cref="Standard_DomainError"/> if U2 - U1 or V2 - V1 is outside [0, 2 Pi].
    /// </summary>
    public Convert_TorusToBSplineSurface(in gp_Torus T, double U1, double U2, double V1, double V2)
        : base(MaxNbUPoles, MaxNbVPoles, MaxNbUKnots, MaxNbVKnots, TheUDegree, TheVDegree)
    {
        double deltaU = U2 - U1;
        double deltaV = V2 - V1;
        Standard_DomainError_Raise_if((deltaU > 2 * M_PI) || (deltaU < 0.0) || (deltaV > 2 * M_PI)
                                        || (deltaV < 0.0),
                                      "Convert_TorusToBSplineSurface");

        myIsUPeriodic = false;
        myIsVPeriodic = false;

        int i, j;
        // construction of the torus in the reference mark xOy.

        // Number of spans : maximum opening = 150 degrees ( = PI / 1.2 rds)
        int nbUSpans = (int)Math.Truncate(1.2 * deltaU / M_PI) + 1;
        int nbVSpans = (int)Math.Truncate(1.2 * deltaV / M_PI) + 1;
        double AlfaU = deltaU / (nbUSpans * 2);
        double AlfaV = deltaV / (nbVSpans * 2);

        myNbUPoles = 2 * nbUSpans + 1;
        myNbVPoles = 2 * nbVSpans + 1;
        myNbUKnots = nbUSpans + 1;
        myNbVKnots = nbVSpans + 1;

        double R = T.MajorRadius();
        double r = T.MinorRadius();

        ComputePoles(R, r, U1, U2, V1, V2, myPoles);

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

        // Replace the bspline in the reference of the torus.
        // and calculate the weight of the bspline.
        double W1, W2;
        gp_Trsf Trsf = new();
        Trsf.SetTransformation(T.Position(), new gp_Ax3(gp.XOY()));

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
    /// The B-spline surface equivalent to the torus between <paramref name="Param1"/> and <paramref name="Param2"/>
    /// in U (<paramref name="UTrim"/>; whole and periodic in V) or in V (whole and periodic in U), with the torus's
    /// orientation. Throws <see cref="Standard_DomainError"/> if Param2 - Param1 is outside [0, 2 Pi].
    /// </summary>
    public Convert_TorusToBSplineSurface(in gp_Torus T, double Param1, double Param2, bool UTrim = true)
        : base(MaxNbUPoles, MaxNbVPoles, MaxNbUKnots, MaxNbVKnots, TheUDegree, TheVDegree)
    {
        double delta = Param2 - Param1;
        Standard_DomainError_Raise_if((delta > 2 * M_PI) || (delta < 0.0),
                                      "Convert_TorusToBSplineSurface");

        int i, j;
        double deltaU, deltaV;

        myIsUPeriodic = !UTrim;
        myIsVPeriodic = UTrim;

        double R = T.MajorRadius();
        double r = T.MinorRadius();

        double W1, W2, CosU, CosV;

        if (myIsUPeriodic)
        {
            ComputePoles(R, r, 0, 2.0 * M_PI, Param1, Param2, myPoles);

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
            ComputePoles(R, r, Param1, Param2, 0.0, 2.0 * M_PI, myPoles);

            myNbVPoles = 6;
            myNbVKnots = 4;

            deltaU = Param2 - Param1;
            int nbUSpans = (int)Math.Truncate(1.2 * deltaU / M_PI) + 1;
            double AlfaU = deltaU / (nbUSpans * 2);
            myNbUPoles = 2 * nbUSpans + 1;
            myNbUKnots = nbUSpans + 1;

            for (i = 1; i <= myNbVKnots; i++)
            {
                myVKnots[i] = (i - 1) * 2.0 * M_PI / 3.0;
                myVMults[i] = 2;
            }
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

        // Replace the bspline in the reference of the torus.
        // and calculate the weight of the bspline.
        gp_Trsf Trsf = new();
        Trsf.SetTransformation(T.Position(), new gp_Ax3(gp.XOY()));

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

    /// <summary>The B-spline surface equivalent to the whole torus (periodic in U and V), with the torus's orientation.</summary>
    public Convert_TorusToBSplineSurface(in gp_Torus T)
        : base(MaxNbUPoles, MaxNbVPoles, MaxNbUKnots, MaxNbVKnots, TheUDegree, TheVDegree)
    {
        myIsUPeriodic = true;
        myIsVPeriodic = true;

        double W1, W2;
        int i, j;

        myNbUPoles = 6;
        myNbVPoles = 6;
        myNbUKnots = 4;
        myNbVKnots = 4;

        // Construction of the Torus in the reference mark xOy.

        double R = T.MajorRadius();
        double r = T.MinorRadius();

        ComputePoles(R, r, 0.0, 2.0 * M_PI, 0.0, 2.0 * M_PI, myPoles);

        myUKnots[1] = myVKnots[1] = 0.0;
        myUKnots[2] = myVKnots[2] = 2.0 * M_PI / 3.0;
        myUKnots[3] = myVKnots[3] = 4.0 * M_PI / 3.0;
        myUKnots[4] = myVKnots[4] = 2.0 * M_PI;
        for (i = 1; i <= 4; i++)
        {
            myUMults[i] = myVMults[i] = 2;
        }

        // Replace the bspline in the mark of the torus.
        // and calculate the weight of the bspline.
        gp_Trsf Trsf = new();
        Trsf.SetTransformation(T.Position(), new gp_Ax3(gp.XOY()));

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
                    W2 = 0.5;
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
