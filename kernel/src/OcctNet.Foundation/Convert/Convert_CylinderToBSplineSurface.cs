// Port of: src/FoundationClasses/TKMath/Convert/Convert_CylinderToBSplineSurface.hxx
// Port of: src/FoundationClasses/TKMath/Convert/Convert_CylinderToBSplineSurface.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// Converts a bounded cylinder into a rational B-spline surface. The cylinder's parametrisation is
/// P(U, V) = Loc + V * ZDir + Radius * (XDir * cos(U) + YDir * sin(U)), U in [0, 2 Pi].
/// </summary>
public class Convert_CylinderToBSplineSurface : Convert_ElementarySurfaceToBSplineSurface
{
    private const int TheUDegree = 2;
    private const int TheVDegree = 1;
    private const int TheNbUKnots = 5;
    private const int TheNbVKnots = 2;
    private const int TheNbUPoles = 9;
    private const int TheNbVPoles = 2;

    private static void ComputePoles(double R,
                                     double U1,
                                     double U2,
                                     double V1,
                                     double V2,
                                     NCollection_Array2<gp_Pnt> Poles)
    {
        double deltaU = U2 - U1;

        int i;

        // Number of spans : maximum opening = 150 degrees ( = PI / 1.2 rds)
        int nbUSpans = (int)Math.Truncate(1.2 * deltaU / M_PI) + 1;
        double AlfaU = deltaU / (nbUSpans * 2);

        double UStart = U1;
        Poles[1, 1] = new gp_Pnt(R * Math.Cos(UStart), R * Math.Sin(UStart), V1);
        Poles[1, 2] = new gp_Pnt(R * Math.Cos(UStart), R * Math.Sin(UStart), V2);

        for (i = 1; i <= nbUSpans; i++)
        {
            Poles[2 * i, 1] = new gp_Pnt(R * Math.Cos(UStart + AlfaU) / Math.Cos(AlfaU),
                                         R * Math.Sin(UStart + AlfaU) / Math.Cos(AlfaU),
                                         V1);
            Poles[2 * i, 2] = new gp_Pnt(R * Math.Cos(UStart + AlfaU) / Math.Cos(AlfaU),
                                         R * Math.Sin(UStart + AlfaU) / Math.Cos(AlfaU),
                                         V2);
            Poles[2 * i + 1, 1] =
              new gp_Pnt(R * Math.Cos(UStart + 2 * AlfaU), R * Math.Sin(UStart + 2 * AlfaU), V1);
            Poles[2 * i + 1, 2] =
              new gp_Pnt(R * Math.Cos(UStart + 2 * AlfaU), R * Math.Sin(UStart + 2 * AlfaU), V2);
            UStart += 2 * AlfaU;
        }
    }

    /// <summary>
    /// The B-spline surface equivalent to the cylinder between the parameters U1, U2 and V1, V2, with the
    /// cylinder's orientation. Throws <see cref="Standard_DomainError"/> if V1 = V2 or U2 - U1 is outside [0, 2 Pi].
    /// </summary>
    public Convert_CylinderToBSplineSurface(in gp_Cylinder Cyl, double U1, double U2, double V1, double V2)
        : base(TheNbUPoles, TheNbVPoles, TheNbUKnots, TheNbVKnots, TheUDegree, TheVDegree)
    {
        double deltaU = U2 - U1;
        Standard_DomainError_Raise_if((Math.Abs(V2 - V1) <= Math.Abs(Epsilon(V1))) || (deltaU > 2 * M_PI)
                                        || (deltaU < 0.0),
                                      "Convert_CylinderToBSplineSurface");

        myIsUPeriodic = false;
        myIsVPeriodic = false;

        int i, j;
        // construction of the cylinder in the reference mark xOy.

        // Number of spans : maximum opening = 150 degrees ( = PI / 1.2 rds)
        int nbUSpans = (int)Math.Truncate(1.2 * deltaU / M_PI) + 1;
        double AlfaU = deltaU / (nbUSpans * 2);

        myNbUPoles = 2 * nbUSpans + 1;
        myNbUKnots = nbUSpans + 1;

        myNbVPoles = 2;
        myNbVKnots = 2;

        double R = Cyl.Radius();

        ComputePoles(R, U1, U2, V1, V2, myPoles);

        for (i = 1; i <= myNbUKnots; i++)
        {
            myUKnots[i] = U1 + (i - 1) * 2 * AlfaU;
            myUMults[i] = 2;
        }
        myUMults[1]++;
        myUMults[myNbUKnots]++;
        myVKnots[1] = V1;
        myVMults[1] = 2;
        myVKnots[2] = V2;
        myVMults[2] = 2;

        // Replace bspline in the mark of the sphere.
        // and calculate the weight of the bspline.
        double W1;
        gp_Trsf Trsf = new();
        Trsf.SetTransformation(Cyl.Position(), new gp_Ax3(gp.XOY()));

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
                myWeights[i, j] = W1;
                myPoles[i, j].Transform(Trsf);
            }
        }
        FinalizeArrays();
    }

    /// <summary>
    /// The B-spline surface equivalent to the whole cylinder between the parameters V1 and V2 (periodic in U),
    /// with the cylinder's orientation. Throws <see cref="Standard_DomainError"/> if V1 = V2.
    /// </summary>
    public Convert_CylinderToBSplineSurface(in gp_Cylinder Cyl, double V1, double V2)
        : base(TheNbUPoles, TheNbVPoles, TheNbUKnots, TheNbVKnots, TheUDegree, TheVDegree)
    {
        Standard_DomainError_Raise_if(Math.Abs(V2 - V1) <= Math.Abs(Epsilon(V1)),
                                      "Convert_CylinderToBSplineSurface");

        int i, j;

        myIsUPeriodic = true;
        myIsVPeriodic = false;

        // construction of the cylinder in the reference mark xOy.

        double R = Cyl.Radius();

        ComputePoles(R, 0.0, 2.0 * M_PI, V1, V2, myPoles);

        myNbUPoles = 6;
        myNbUKnots = 4;
        myNbVPoles = 2;
        myNbVKnots = 2;

        for (i = 1; i <= myNbUKnots; i++)
        {
            myUKnots[i] = (i - 1) * 2.0 * M_PI / 3.0;
            myUMults[i] = 2;
        }
        myVKnots[1] = V1;
        myVMults[1] = 2;
        myVKnots[2] = V2;
        myVMults[2] = 2;

        // Replace the bspline inn the mark of the cone.
        // and calculate the weight of the bspline.
        double W;
        gp_Trsf Trsf = new();
        Trsf.SetTransformation(Cyl.Position(), new gp_Ax3(gp.XOY()));

        for (i = 1; i <= myNbUPoles; i++)
        {
            if (i % 2 == 0)
            {
                W = 0.5; // = std::cos(pi /3)
            }
            else
            {
                W = 1.0;
            }

            for (j = 1; j <= myNbVPoles; j++)
            {
                myWeights[i, j] = W;
                myPoles[i, j].Transform(Trsf);
            }
        }
        FinalizeArrays();
    }
}
