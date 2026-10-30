// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_DistributionOfTension.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_DistributionOfTension.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_DistributionOfTension.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The tension distribution of a batten: its local energy of stretching and its derivatives.</summary>
public class FairCurve_DistributionOfTension : FairCurve_DistributionOfEnergy
{
    private double MyLengthSliding;
    private readonly FairCurve_BattenLaw MyLaw;
    private readonly double MyHeight;

    /// <summary>
    /// The tension of the curve of order <paramref name="BSplOrder"/> over <paramref name="FlatKnots"/> and
    /// <paramref name="Poles"/> (kept, not copied). <paramref name="Uniform"/> uses the law's height at the middle
    /// everywhere (MinimalVariation uses it to avoid parametrisation problems).
    /// </summary>
    public FairCurve_DistributionOfTension(int BSplOrder,
                                           NCollection_HArray1<double> FlatKnots,
                                           NCollection_HArray1<gp_Pnt2d> Poles,
                                           int DerivativeOrder,
                                           double LengthSliding,
                                           FairCurve_BattenLaw Law,
                                           int NbValAux = 0,
                                           bool Uniform = false)
        : base(BSplOrder, FlatKnots, Poles, DerivativeOrder, NbValAux)
    {
        MyLengthSliding = LengthSliding;
        // a copy of the law, as OCCT's by-value member (decision 5891)
        MyLaw = new FairCurve_BattenLaw(Law);
        if (Uniform)
        {
            MyLaw.Value(0.5, out MyHeight);
        } // it used in MVC to avoid Parametrization Problemes
        else
        {
            MyHeight = 0;
        }
    }

    /// <summary>Changes the length sliding.</summary>
    public void SetLengthSliding(double LengthSliding)
    {
        MyLengthSliding = LengthSliding;
    }

    /// <summary>Computes the values <paramref name="FTension"/> at <paramref name="TParam"/>; false if they could not be computed.</summary>
    public override bool Value(math_Vector TParam, math_Vector FTension)
    {
        bool Ok = true;
        int ier, ii, jj, kk;
        gp_XY CPrim = new(0.0, 0.0);
        int LastGradientIndex, FirstNonZero, LastZero;

        // (0.0) initialisations generales
        FTension.Init(0.0);
        math_Matrix Base = new(1,
                               3,
                               1,
                               MyBSplOrder); // On shouhaite utiliser la derive premieres
                                             // Dans EvalBsplineBasis C' <=> DerivOrder = 2
                                             // et il faut ajouter 1 rang dans la matrice Base => 3 rang

        ier = BSplCLib.EvalBsplineBasis(1,
                                        MyBSplOrder,
                                        MyFlatKnots.Array1(),
                                        TParam[TParam.Lower()],
                                        out FirstNonZero,
                                        Base);
        if (ier != 0)
        {
            return false;
        }
        LastZero = FirstNonZero - 1;
        FirstNonZero = 2 * LastZero + 1;

        // (0.1) evaluation de CPrim
        for (ii = 1; ii <= MyBSplOrder; ii++)
        {
            CPrim += Base[2, ii] * MyPoles.Value(ii + LastZero).Coord();
        }

        // (1) Evaluation de la tension locale --------------------------------
        double NormeCPrim = CPrim.Modulus();
        double Hauteur, Difference;

        if (MyHeight > 0)
        {
            Hauteur = MyHeight;
        } // it used in MVC to avoid Parametrization Problemes
        else
        {
            Ok = MyLaw.Value(TParam[TParam.Lower()], out Hauteur);
            if (!Ok)
            {
                return Ok;
            }
        }
        Difference = NormeCPrim - MyLengthSliding;

        FTension[FTension.Lower()] = Hauteur * Math.Pow(Difference, 2) / MyLengthSliding;

        if (MyDerivativeOrder >= 1)
        {
            // (2) Evaluation du gradient de la tension locale ----------------------
            math_Vector GradDifference = new(1, 2 * MyBSplOrder + MyNbValAux);
            double Xaux, Yaux, Facteur;

            Xaux = CPrim.X() / NormeCPrim;
            Yaux = CPrim.Y() / NormeCPrim;
            Facteur = 2 * Hauteur * Difference / MyLengthSliding;

            kk = FTension.Lower() + FirstNonZero;
            jj = 1;
            for (ii = 1; ii <= MyBSplOrder; ii++)
            {
                GradDifference[jj] = Base[2, ii] * Xaux;
                FTension[kk] = Facteur * GradDifference[jj];
                jj += 1;
                GradDifference[jj] = Base[2, ii] * Yaux;
                FTension[kk + 1] = Facteur * GradDifference[jj];
                jj += 1;
                kk += 2;
            }
            if (MyNbValAux == 1)
            {
                LastGradientIndex = FTension.Lower() + 2 * MyPoles.Length() + 1;
                GradDifference[GradDifference.Upper()] = (1 - Math.Pow(NormeCPrim / MyLengthSliding, 2));
                FTension[LastGradientIndex] = Hauteur * GradDifference[GradDifference.Upper()];
            }

            else
            {
                LastGradientIndex = FTension.Lower() + 2 * MyPoles.Length();
            }

            if (MyDerivativeOrder >= 2)
            {

                // (3) Evaluation du Hessien de la tension locale ----------------------

                double FacteurX = Difference * (1 - Math.Pow(Xaux, 2)) / NormeCPrim;
                double FacteurY = Difference * (1 - Math.Pow(Yaux, 2)) / NormeCPrim;
                double FacteurXY = -Difference * Xaux * Yaux / NormeCPrim;
                double Produit;
                int k1, k2;

                Facteur = 2 * Hauteur / MyLengthSliding;

                kk = FirstNonZero;
                k2 = LastGradientIndex + (kk - 1) * kk / 2;

                for (ii = 2; ii <= 2 * MyBSplOrder; ii += 2)
                {
                    k1 = k2 + FirstNonZero;
                    k2 = k1 + kk;
                    kk += 2;
                    for (jj = 2; jj < ii; jj += 2)
                    {
                        Produit = Base[2, ii / 2] * Base[2, jj / 2];

                        FTension[k1] = Facteur
                                       * (GradDifference[ii - 1] * GradDifference[jj - 1]
                                          + FacteurX * Produit); // derivation en XiXj
                        k1++;
                        FTension[k1] = Facteur
                                       * (GradDifference[ii] * GradDifference[jj - 1]
                                          + FacteurXY * Produit); // derivation en YiXj
                        k1++;
                        FTension[k2] = Facteur
                                       * (GradDifference[ii - 1] * GradDifference[jj]
                                          + FacteurXY * Produit); // derivation en XiYj
                        k2++;
                        FTension[k2] =
                          Facteur
                          * (GradDifference[ii] * GradDifference[jj] + FacteurY * Produit); // derivation en YiYj
                        k2++;
                    }
                    // case where jj = ii: triangular fill
                    Produit = Math.Pow(Base[2, ii / 2], 2);

                    FTension[k1] = Facteur
                                   * (GradDifference[ii - 1] * GradDifference[ii - 1]
                                      + FacteurX * Produit); // derivation en XiXi
                    FTension[k2] = Facteur
                                   * (GradDifference[ii] * GradDifference[ii - 1]
                                      + FacteurXY * Produit); // derivation en XiYi
                    k2++;
                    FTension[k2] =
                      Facteur
                      * (GradDifference[ii] * GradDifference[ii] + FacteurY * Produit); // derivation en YiYi
                }
                if (MyNbValAux == 1)
                {
                    FacteurX = -2 * CPrim.X() * Hauteur / Math.Pow(MyLengthSliding, 2);
                    FacteurY = -2 * CPrim.Y() * Hauteur / Math.Pow(MyLengthSliding, 2);

                    ii = LastGradientIndex - FTension.Lower();
                    kk = LastGradientIndex + (ii - 1) * ii / 2 + FirstNonZero;
                    for (ii = 1; ii <= MyBSplOrder; ii++)
                    {
                        FTension[kk] = FacteurX * Base[2, ii];
                        kk++;
                        FTension[kk] = FacteurY * Base[2, ii];
                        kk++;
                    }
                    FTension[FTension.Upper()] =
                      2 * Hauteur * Math.Pow(NormeCPrim / MyLengthSliding, 2) / MyLengthSliding;
                }
            }
        }

        // sortie standard
        return Ok;
    }
}
