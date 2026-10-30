// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_DistributionOfJerk.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_DistributionOfJerk.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The "jerk" distribution of a curve: the local energy of its variation of curvature and its derivatives.</summary>
public class FairCurve_DistributionOfJerk : FairCurve_DistributionOfEnergy
{
    private readonly FairCurve_BattenLaw MyLaw;

    /// <summary>
    /// The jerk of the curve of order <paramref name="BSplOrder"/> over <paramref name="FlatKnots"/> and
    /// <paramref name="Poles"/> (kept, not copied).
    /// </summary>
    public FairCurve_DistributionOfJerk(int BSplOrder,
                                        NCollection_HArray1<double> FlatKnots,
                                        NCollection_HArray1<gp_Pnt2d> Poles,
                                        int DerivativeOrder,
                                        FairCurve_BattenLaw Law,
                                        int NbValAux = 0)
        : base(BSplOrder, FlatKnots, Poles, DerivativeOrder, NbValAux)
    {
        // a copy of the law, as OCCT's by-value member (decision 5891)
        MyLaw = new FairCurve_BattenLaw(Law);
    }

    /// <summary>Computes the values <paramref name="Jerk"/> at <paramref name="TParam"/>; false if they could not be computed.</summary>
    public override bool Value(math_Vector TParam, math_Vector Jerk)
    {
        bool Ok = true;
        int ier, ii, jj, kk;
        gp_XY CPrim = new(0.0, 0.0), CSecn = new(0.0, 0.0), CTroi = new(0.0, 0.0);
        int LastGradientIndex, FirstNonZero, LastZero;

        // (0.0) initialisations generales
        Jerk.Init(0.0);
        math_Matrix Base = new(1,
                               5,
                               1,
                               MyBSplOrder); // On shouhaite utiliser la derive troisieme
                                             // Dans EvalBsplineBasis C"' <=> DerivOrder = 4
                                             // et il faut ajouter 1 rang dans la matrice Base => 5 rangs

        ier = BSplCLib.EvalBsplineBasis(3,
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

        // (0.1) evaluation de CPrim et CScn
        for (ii = 1; ii <= MyBSplOrder; ii++)
        {
            CPrim += Base[2, ii] * MyPoles.Value(ii + LastZero).Coord();
            CSecn += Base[3, ii] * MyPoles.Value(ii + LastZero).Coord();
            CTroi += Base[4, ii] * MyPoles.Value(ii + LastZero).Coord();
        }

        // (1) Evaluation de la secousse locale = W*W
        double NormeCPrim = CPrim.Modulus();
        double InvNormeCPrim = 1 / NormeCPrim;
        double InvNormeCPrim2 = InvNormeCPrim * InvNormeCPrim;
        double ProduitC1C2 = CPrim * CSecn;
        double DeriveNormeCPrim = ProduitC1C2 * InvNormeCPrim2;

        double Hauteur, WVal, Mesure;
        double NumRho = CPrim ^ CSecn;
        double Numerateur = (CPrim ^ CTroi) - 3 * NumRho * DeriveNormeCPrim;
        double Denominateur = Math.Pow(NormeCPrim, 2.5);

        Ok = MyLaw.Value(TParam[TParam.Lower()], out Hauteur);
        if (!Ok)
        {
            return Ok;
        }

        Mesure = Math.Pow(Hauteur, 3) / 12;
        WVal = Numerateur / Denominateur;
        Jerk[Jerk.Lower()] = Mesure * Math.Pow(WVal, 2);

        if (MyDerivativeOrder >= 1)
        {
            // (2) Evaluation du gradient de la secousse locale.

            math_Vector WGrad = new(1, 2 * MyBSplOrder + MyNbValAux), NumGrad = new(1, 2 * MyBSplOrder),
              NGrad1 = new(1, 2 * MyBSplOrder), NGrad2 = new(1, 2 * MyBSplOrder), GradNormeCPrim = new(1, 2 * MyBSplOrder),
              NGDNCPrim = new(1, 2 * MyBSplOrder), GradDeriveNormeCPrim = new(1, 2 * MyBSplOrder),
              GradNumRho = new(1, 2 * MyBSplOrder), NumduGrad = new(1, 2 * MyBSplOrder);
            double Facteur;
            double XPrim = CPrim.X();
            double YPrim = CPrim.Y();
            double XSecn = CSecn.X();
            double YSecn = CSecn.Y();
            double XTroi = CTroi.X();
            double YTroi = CTroi.Y();
            double InvDenominateur = 1 / Denominateur;
            double Aux, AuxBis;

            Facteur = 2 * Mesure * WVal;
            Aux = 2.5 * Numerateur * InvNormeCPrim;
            AuxBis = 2 * ProduitC1C2 * InvNormeCPrim;
            kk = Jerk.Lower() + FirstNonZero;

            jj = 1;
            for (ii = 1; ii <= MyBSplOrder; ii++)
            {

                //     (2.1) Derivation en X
                GradNormeCPrim[jj] = XPrim * Base[2, ii] * InvNormeCPrim;
                NGDNCPrim[jj] = (XPrim * Base[3, ii] + XSecn * Base[2, ii]) - AuxBis * GradNormeCPrim[jj];
                GradDeriveNormeCPrim[jj] = NGDNCPrim[jj] * InvNormeCPrim2;
                GradNumRho[jj] = YSecn * Base[2, ii] - YPrim * Base[3, ii];
                NGrad1[jj] = YTroi * Base[2, ii] - YPrim * Base[4, ii];
                NGrad2[jj] = -3 * (NumRho * GradDeriveNormeCPrim[jj] + GradNumRho[jj] * DeriveNormeCPrim);
                NumGrad[jj] = NGrad1[jj] + NGrad2[jj];
                NumduGrad[jj] = NumGrad[jj] - Aux * GradNormeCPrim[jj];
                WGrad[jj] = InvDenominateur * NumduGrad[jj];
                Jerk[kk] = Facteur * WGrad[jj];

                jj += 1;

                //     (2.2) Derivation en Y

                GradNormeCPrim[jj] = YPrim * Base[2, ii] * InvNormeCPrim;
                NGDNCPrim[jj] = (YPrim * Base[3, ii] + YSecn * Base[2, ii]) - AuxBis * GradNormeCPrim[jj];
                GradDeriveNormeCPrim[jj] = NGDNCPrim[jj] * InvNormeCPrim2;
                GradNumRho[jj] = -XSecn * Base[2, ii] + XPrim * Base[3, ii];
                NGrad1[jj] = -XTroi * Base[2, ii] + XPrim * Base[4, ii];
                NGrad2[jj] = -3 * (NumRho * GradDeriveNormeCPrim[jj] + GradNumRho[jj] * DeriveNormeCPrim);
                NumGrad[jj] = NGrad1[jj] + NGrad2[jj];
                NumduGrad[jj] = NumGrad[jj] - Aux * GradNormeCPrim[jj];
                WGrad[jj] = InvDenominateur * NumduGrad[jj];
                Jerk[kk + 1] = Facteur * WGrad[jj];

                jj += 1;
                kk += 2;
            }
            if (MyNbValAux == 1)
            {
                //    (2.3) Gestion de la variable de glissement
                LastGradientIndex = Jerk.Lower() + 2 * MyPoles.Length() + 1;
                WGrad[WGrad.Upper()] = 0.0;
                Jerk[LastGradientIndex] = 0.0;
            }

            else
            {
                LastGradientIndex = Jerk.Lower() + 2 * MyPoles.Length();
            }

            if (MyDerivativeOrder >= 2)
            {

                // (3) Evaluation du Hessien de la tension locale ----------------------

                double FacteurX = (1 - Math.Pow(XPrim * InvNormeCPrim, 2)) * InvNormeCPrim;
                double FacteurY = (1 - Math.Pow(YPrim * InvNormeCPrim, 2)) * InvNormeCPrim;
                double FacteurXY = -(XPrim * InvNormeCPrim) * (YPrim * InvNormeCPrim) * InvNormeCPrim;
                double FacteurW = WVal * InvNormeCPrim;

                double Produit, Produit2, ProduitV, HNumRho, DSeconde, NSeconde, HDeriveNormeCPrim, Aux1,
                  DeriveAuxBis;
                double VIntermed;
                int k1, k2, II, JJ;

                Facteur = 2 * Mesure;

                kk = FirstNonZero;
                k2 = LastGradientIndex + (kk - 1) * kk / 2;

                for (ii = 2; ii <= 2 * MyBSplOrder; ii += 2)
                {
                    II = ii / 2;
                    k1 = k2 + FirstNonZero;
                    k2 = k1 + kk;
                    kk += 2;
                    for (jj = 2; jj < ii; jj += 2)
                    {
                        JJ = jj / 2;
                        Produit = Base[2, II] * Base[2, JJ];
                        Produit2 = Base[2, II] * Base[3, JJ] + Base[3, II] * Base[2, JJ];
                        ProduitV = Base[2, II] * Base[4, JJ] - Base[4, II] * Base[2, JJ];
                        HNumRho = Base[2, II] * Base[3, JJ] - Base[3, II] * Base[2, JJ];

                        // derivation en XiXj
                        Aux1 = InvNormeCPrim2 * GradNormeCPrim[jj - 1];
                        DeriveAuxBis =
                          2 * ((XPrim * Base[3, JJ] + XSecn * Base[2, JJ]) * InvNormeCPrim - ProduitC1C2 * Aux1);
                        HDeriveNormeCPrim = (Produit2 - DeriveAuxBis * GradNormeCPrim[ii - 1]
                                             - AuxBis * (Produit * InvNormeCPrim - XPrim * Base[2, II] * Aux1))
                                              * InvNormeCPrim2
                                            - 2 * NGDNCPrim[ii - 1] * Aux1 * InvNormeCPrim;
                        NSeconde =
                          -3
                          * (GradNumRho[ii - 1] * GradDeriveNormeCPrim[jj - 1] + NumRho * HDeriveNormeCPrim
                             + GradNumRho[jj - 1] * GradDeriveNormeCPrim[ii - 1]);

                        DSeconde = FacteurX * Produit;
                        Aux = NormeCPrim * NSeconde + NumGrad[ii - 1] * GradNormeCPrim[jj - 1]
                              - 2.5 * (NumGrad[jj - 1] * GradNormeCPrim[ii - 1] + DSeconde * Numerateur);
                        VIntermed = InvDenominateur * (Aux - 3.5 * GradNormeCPrim[jj - 1] * NumduGrad[ii - 1]);

                        Jerk[k1] = Facteur * (WGrad[ii - 1] * WGrad[jj - 1] + FacteurW * VIntermed);
                        k1++;

                        // derivation en XiYj
                        Aux1 = InvNormeCPrim2 * GradNormeCPrim[jj];
                        DeriveAuxBis =
                          2 * ((YPrim * Base[3, JJ] + YSecn * Base[2, JJ]) * InvNormeCPrim - ProduitC1C2 * Aux1);
                        HDeriveNormeCPrim =
                          (-DeriveAuxBis * GradNormeCPrim[ii - 1] + AuxBis * XPrim * Base[2, II] * Aux1)
                            * InvNormeCPrim2
                          - 2 * NGDNCPrim[ii - 1] * Aux1 * InvNormeCPrim;
                        NSeconde =
                          ProduitV
                          - 3
                              * (GradNumRho[ii - 1] * GradDeriveNormeCPrim[jj] + NumRho * HDeriveNormeCPrim
                                 + GradNumRho[jj] * GradDeriveNormeCPrim[ii - 1] + HNumRho * DeriveNormeCPrim);
                        DSeconde = FacteurXY * Produit;
                        Aux = NormeCPrim * NSeconde + NumGrad[ii - 1] * GradNormeCPrim[jj]
                              - 2.5 * (NumGrad[jj] * GradNormeCPrim[ii - 1] + DSeconde * Numerateur);
                        VIntermed = InvDenominateur * (Aux - 3.5 * GradNormeCPrim[jj] * NumduGrad[ii - 1]);
                        Jerk[k1] = Facteur * (WGrad[ii - 1] * WGrad[jj] + FacteurW * VIntermed);
                        k1++;

                        // derivation en YiXj
                        // DSeconde calcule ci-dessus
                        Aux1 = InvNormeCPrim2 * GradNormeCPrim[jj - 1];
                        DeriveAuxBis =
                          2 * ((XPrim * Base[3, JJ] + XSecn * Base[2, JJ]) * InvNormeCPrim - ProduitC1C2 * Aux1);
                        HDeriveNormeCPrim =
                          (-DeriveAuxBis * GradNormeCPrim[ii] + AuxBis * YPrim * Base[2, II] * Aux1)
                            * InvNormeCPrim2
                          - 2 * NGDNCPrim[ii] * Aux1 * InvNormeCPrim;
                        NSeconde =
                          -ProduitV
                          - 3
                              * (GradNumRho[ii] * GradDeriveNormeCPrim[jj - 1] + NumRho * HDeriveNormeCPrim
                                 + GradNumRho[jj - 1] * GradDeriveNormeCPrim[ii] - HNumRho * DeriveNormeCPrim);
                        Aux = NormeCPrim * NSeconde + NumGrad[ii] * GradNormeCPrim[jj - 1]
                              - 2.5 * (NumGrad[jj - 1] * GradNormeCPrim[ii] + DSeconde * Numerateur);
                        VIntermed = InvDenominateur * (Aux - 3.5 * GradNormeCPrim[jj - 1] * NumduGrad[ii]);

                        Jerk[k2] = Facteur * (WGrad[ii] * WGrad[jj - 1] + FacteurW * VIntermed);
                        k2++;

                        // derivation en YiYj
                        Aux1 = InvNormeCPrim2 * GradNormeCPrim[jj];
                        DeriveAuxBis =
                          2 * ((YPrim * Base[3, JJ] + YSecn * Base[2, JJ]) * InvNormeCPrim - ProduitC1C2 * Aux1);
                        HDeriveNormeCPrim = (Produit2 - DeriveAuxBis * GradNormeCPrim[ii]
                                             - AuxBis * (Produit * InvNormeCPrim - YPrim * Base[2, II] * Aux1))
                                              * InvNormeCPrim2
                                            - 2 * NGDNCPrim[ii] * Aux1 * InvNormeCPrim;
                        NSeconde = -3
                                   * (GradNumRho[ii] * GradDeriveNormeCPrim[jj] + NumRho * HDeriveNormeCPrim
                                      + GradNumRho[jj] * GradDeriveNormeCPrim[ii]);

                        DSeconde = FacteurY * Produit;
                        Aux = NSeconde * NormeCPrim + NumGrad[ii] * GradNormeCPrim[jj]
                              - 2.5 * (NumGrad[jj] * GradNormeCPrim[ii] + DSeconde * Numerateur);
                        VIntermed = InvDenominateur * (Aux - 3.5 * GradNormeCPrim[jj] * NumduGrad[ii]);
                        Jerk[k2] = Facteur * (WGrad[ii] * WGrad[jj] + FacteurW * VIntermed);
                        k2++;
                    }

                    // case where jj = ii: triangular fill
                    Produit = Math.Pow(Base[2, II], 2);
                    Produit2 = 2 * Base[2, II] * Base[3, II];
                    //           ProduitV2 = Base

                    // derivation en XiXi
                    Aux1 = InvNormeCPrim2 * GradNormeCPrim[ii - 1];
                    DeriveAuxBis =
                      2 * ((XPrim * Base[3, II] + XSecn * Base[2, II]) * InvNormeCPrim - ProduitC1C2 * Aux1);
                    HDeriveNormeCPrim = (Produit2 - DeriveAuxBis * GradNormeCPrim[ii - 1]
                                         - AuxBis * (Produit * InvNormeCPrim - XPrim * Base[2, II] * Aux1))
                                          * InvNormeCPrim2
                                        - 2 * NGDNCPrim[ii - 1] * Aux1 * InvNormeCPrim;
                    NSeconde =
                      -3 * (2 * GradNumRho[ii - 1] * GradDeriveNormeCPrim[ii - 1] + NumRho * HDeriveNormeCPrim);
                    DSeconde = FacteurX * Produit;
                    Aux = NSeconde * NormeCPrim - 1.5 * NumGrad[ii - 1] * GradNormeCPrim[ii - 1]
                          - 2.5 * DSeconde * Numerateur;
                    VIntermed = InvDenominateur * (Aux - 3.5 * GradNormeCPrim[ii - 1] * NumduGrad[ii - 1]);
                    Jerk[k1] = Facteur * (WGrad[ii - 1] * WGrad[ii - 1] + FacteurW * VIntermed);
                    // derivation en XiYi
                    Aux1 = InvNormeCPrim2 * GradNormeCPrim[ii];
                    DeriveAuxBis =
                      2 * ((YPrim * Base[3, II] + YSecn * Base[2, II]) * InvNormeCPrim - ProduitC1C2 * Aux1);
                    HDeriveNormeCPrim =
                      (-DeriveAuxBis * GradNormeCPrim[ii - 1] + AuxBis * XPrim * Base[2, II] * Aux1)
                        * InvNormeCPrim2
                      - 2 * NGDNCPrim[ii - 1] * Aux1 * InvNormeCPrim;
                    NSeconde = -3
                               * (GradNumRho[ii - 1] * GradDeriveNormeCPrim[ii] + NumRho * HDeriveNormeCPrim
                                  + GradNumRho[ii] * GradDeriveNormeCPrim[ii - 1]);
                    DSeconde = FacteurXY * Produit;
                    Aux = NSeconde * NormeCPrim + NumGrad[ii] * GradNormeCPrim[ii - 1]
                          - 2.5 * (NumGrad[ii - 1] * GradNormeCPrim[ii] + DSeconde * Numerateur);
                    VIntermed = InvDenominateur * (Aux - 3.5 * GradNormeCPrim[ii - 1] * NumduGrad[ii]);
                    Jerk[k2] = Facteur * (WGrad[ii] * WGrad[ii - 1] + FacteurW * VIntermed);
                    k2++;

                    // derivation en YiYi
                    // Aux1 et DeriveAuxBis calcules au pas precedent ...
                    HDeriveNormeCPrim = (Produit2 - DeriveAuxBis * GradNormeCPrim[ii]
                                         - AuxBis * (Produit * InvNormeCPrim - YPrim * Base[2, II] * Aux1))
                                          * InvNormeCPrim2
                                        - 2 * NGDNCPrim[ii] * Aux1 * InvNormeCPrim;
                    NSeconde =
                      -3 * (2 * GradNumRho[ii] * GradDeriveNormeCPrim[ii] + NumRho * HDeriveNormeCPrim);
                    DSeconde = FacteurY * Produit;
                    Aux = NSeconde * NormeCPrim - 1.5 * NumGrad[ii] * GradNormeCPrim[ii]
                          - 2.5 * DSeconde * Numerateur;
                    VIntermed = InvDenominateur * (Aux - 3.5 * GradNormeCPrim[ii] * NumduGrad[ii]);
                    Jerk[k2] = Facteur * (WGrad[ii] * WGrad[ii] + FacteurW * VIntermed);
                }
            }
        }

        // sortie standard
        return Ok;
    }
}
