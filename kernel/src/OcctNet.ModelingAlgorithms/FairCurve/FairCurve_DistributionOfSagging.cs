// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_DistributionOfSagging.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_DistributionOfSagging.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>The sagging (flexion) distribution of a batten: its local bending energy and its derivatives.</summary>
public class FairCurve_DistributionOfSagging : FairCurve_DistributionOfEnergy
{
    private readonly FairCurve_BattenLaw MyLaw;

    /// <summary>
    /// The sagging of the curve of order <paramref name="BSplOrder"/> over <paramref name="FlatKnots"/> and
    /// <paramref name="Poles"/> (kept, not copied).
    /// </summary>
    public FairCurve_DistributionOfSagging(int BSplOrder,
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

    /// <summary>Computes the values <paramref name="Flexion"/> at <paramref name="TParam"/>; false if they could not be computed.</summary>
    public override bool Value(math_Vector TParam, math_Vector Flexion)
    {
        bool Ok = true;
        int ier, ii, jj, kk;
        gp_XY CPrim = new(0.0, 0.0), CSecn = new(0.0, 0.0);
        int LastGradientIndex, FirstNonZero, LastZero;

        // (0.0) initialisations generales
        Flexion.Init(0.0);
        math_Matrix Base = new(1,
                               4,
                               1,
                               MyBSplOrder); // On shouhaite utiliser la derive premieres
                                             // Dans EvalBsplineBasis C" <=> DerivOrder = 3
                                             // et il faut ajouter 1 rang dans la matrice Base => 4 rang

        ier = BSplCLib.EvalBsplineBasis(2,
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
        }

        // (1) Evaluation de la flexion locale = W*W
        double NormeCPrim = CPrim.Modulus();
        double InvNormeCPrim = 1 / NormeCPrim;
        double Hauteur, WVal, Mesure;
        double Numerateur = CPrim ^ CSecn;
        double Denominateur = Math.Pow(NormeCPrim, 2.5);

        Ok = MyLaw.Value(TParam[TParam.Lower()], out Hauteur);
        if (!Ok)
        {
            return Ok;
        }

        Mesure = Math.Pow(Hauteur, 3) / 12;
        WVal = Numerateur / Denominateur;
        Flexion[Flexion.Lower()] = Mesure * Math.Pow(WVal, 2);

        if (MyDerivativeOrder >= 1)
        {
            // (2) Evaluation du gradient de la flexion locale.

            math_Vector WGrad = new(1, 2 * MyBSplOrder + MyNbValAux), NumGrad = new(1, 2 * MyBSplOrder + MyNbValAux),
              GradNormeCPrim = new(1, 2 * MyBSplOrder + MyNbValAux), NumduGrad = new(1, 2 * MyBSplOrder + MyNbValAux);
            double Facteur;
            double XPrim = CPrim.X();
            double YPrim = CPrim.Y();
            double XSecn = CSecn.X();
            double YSecn = CSecn.Y();
            double InvDenominateur = 1 / Denominateur;
            double Aux;

            Facteur = 2 * Mesure * WVal;
            Aux = 2.5 * Numerateur * InvNormeCPrim;
            kk = Flexion.Lower() + FirstNonZero;

            jj = 1;
            for (ii = 1; ii <= MyBSplOrder; ii++)
            {

                //     (2.1) Derivation en X
                NumGrad[jj] = YSecn * Base[2, ii] - YPrim * Base[3, ii];
                GradNormeCPrim[jj] = XPrim * Base[2, ii] * InvNormeCPrim;
                NumduGrad[jj] = NumGrad[jj] - Aux * GradNormeCPrim[jj];
                WGrad[jj] = InvDenominateur * NumduGrad[jj];
                Flexion[kk] = Facteur * WGrad[jj];
                jj += 1;

                //     (2.2) Derivation en Y
                NumGrad[jj] = -XSecn * Base[2, ii] + XPrim * Base[3, ii];
                GradNormeCPrim[jj] = YPrim * Base[2, ii] * InvNormeCPrim;
                NumduGrad[jj] = NumGrad[jj] - Aux * GradNormeCPrim[jj];
                WGrad[jj] = InvDenominateur * NumduGrad[jj];
                Flexion[kk + 1] = Facteur * WGrad[jj];
                jj += 1;
                kk += 2;
            }
            if (MyNbValAux == 1)
            {
                //    (2.3) Gestion de la variable de glissement
                LastGradientIndex = Flexion.Lower() + 2 * MyPoles.Length() + 1;
                WGrad[WGrad.Upper()] = 0.0;
            }

            else
            {
                LastGradientIndex = Flexion.Lower() + 2 * MyPoles.Length();
            }

            if (MyDerivativeOrder >= 2)
            {

                // (3) Evaluation du Hessien de la tension locale ----------------------

                double FacteurX = (1 - Math.Pow(XPrim * InvNormeCPrim, 2)) * InvNormeCPrim;
                double FacteurY = (1 - Math.Pow(YPrim * InvNormeCPrim, 2)) * InvNormeCPrim;
                double FacteurXY = -(XPrim * InvNormeCPrim) * (YPrim * InvNormeCPrim) * InvNormeCPrim;
                double FacteurW = WVal * InvNormeCPrim;

                double Produit, DSeconde, NSeconde;
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
                        NSeconde = Base[2, II] * Base[3, JJ] - Base[3, II] * Base[2, JJ];

                        // derivation en XiXj
                        DSeconde = FacteurX * Produit;
                        Aux = NumGrad[ii - 1] * GradNormeCPrim[jj - 1]
                              - 2.5 * (NumGrad[jj - 1] * GradNormeCPrim[ii - 1] + DSeconde * Numerateur);
                        VIntermed = InvDenominateur * (Aux - 3.5 * GradNormeCPrim[jj - 1] * NumduGrad[ii - 1]);

                        Flexion[k1] = Facteur * (WGrad[ii - 1] * WGrad[jj - 1] + FacteurW * VIntermed);
                        k1++;

                        // derivation en XiYj
                        DSeconde = FacteurXY * Produit;
                        Aux = NormeCPrim * NSeconde + NumGrad[ii - 1] * GradNormeCPrim[jj]
                              - 2.5 * (NumGrad[jj] * GradNormeCPrim[ii - 1] + DSeconde * Numerateur);
                        VIntermed = InvDenominateur * (Aux - 3.5 * GradNormeCPrim[jj] * NumduGrad[ii - 1]);
                        Flexion[k1] = Facteur * (WGrad[ii - 1] * WGrad[jj] + FacteurW * VIntermed);
                        k1++;

                        // derivation en YiXj
                        // DSeconde calcule ci-dessus
                        Aux = -NormeCPrim * NSeconde + NumGrad[ii] * GradNormeCPrim[jj - 1]
                              - 2.5 * (NumGrad[jj - 1] * GradNormeCPrim[ii] + DSeconde * Numerateur);
                        VIntermed = InvDenominateur * (Aux - 3.5 * GradNormeCPrim[jj - 1] * NumduGrad[ii]);

                        Flexion[k2] = Facteur * (WGrad[ii] * WGrad[jj - 1] + FacteurW * VIntermed);
                        k2++;

                        // derivation en YiYj
                        DSeconde = FacteurY * Produit;
                        Aux = NumGrad[ii] * GradNormeCPrim[jj]
                              - 2.5 * (NumGrad[jj] * GradNormeCPrim[ii] + DSeconde * Numerateur);
                        VIntermed = InvDenominateur * (Aux - 3.5 * GradNormeCPrim[jj] * NumduGrad[ii]);
                        Flexion[k2] = Facteur * (WGrad[ii] * WGrad[jj] + FacteurW * VIntermed);
                        k2++;
                    }

                    // case where jj = ii: triangular fill
                    Produit = Math.Pow(Base[2, II], 2);

                    // derivation en XiXi
                    DSeconde = FacteurX * Produit;
                    Aux = -1.5 * NumGrad[ii - 1] * GradNormeCPrim[ii - 1] - 2.5 * DSeconde * Numerateur;
                    VIntermed = InvDenominateur * (Aux - 3.5 * GradNormeCPrim[ii - 1] * NumduGrad[ii - 1]);
                    Flexion[k1] = Facteur * (WGrad[ii - 1] * WGrad[ii - 1] + FacteurW * VIntermed);
                    // derivation en XiYi
                    DSeconde = FacteurXY * Produit;
                    Aux = NumGrad[ii - 1] * GradNormeCPrim[ii]
                          - 2.5 * (NumGrad[ii] * GradNormeCPrim[ii - 1] + DSeconde * Numerateur);
                    VIntermed = InvDenominateur * (Aux - 3.5 * GradNormeCPrim[ii] * NumduGrad[ii - 1]);
                    Flexion[k2] = Facteur * (WGrad[ii] * WGrad[ii - 1] + FacteurW * VIntermed);
                    k2++;

                    // derivation en YiYi
                    DSeconde = FacteurY * Produit;
                    Aux = -1.5 * NumGrad[ii] * GradNormeCPrim[ii] - 2.5 * DSeconde * Numerateur;
                    VIntermed = InvDenominateur * (Aux - 3.5 * GradNormeCPrim[ii] * NumduGrad[ii]);
                    Flexion[k2] = Facteur * (WGrad[ii] * WGrad[ii] + FacteurW * VIntermed);
                }
            }
        }

        // sortie standard
        return Ok;
    }
}
