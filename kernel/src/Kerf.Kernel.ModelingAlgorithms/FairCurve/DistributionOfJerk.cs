// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_DistributionOfJerk.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_DistributionOfJerk.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Fairing;

/// <summary>The "jerk" distribution of a curve: the local energy of its variation of curvature and its derivatives.</summary>
internal class DistributionOfJerk : DistributionOfEnergy
{
    private readonly BattenLaw _myLaw;

    /// <summary>
    /// The jerk of the curve of order <paramref name="bSplOrder"/> over <paramref name="flatKnots"/> and
    /// <paramref name="poles"/> (kept, not copied).
    /// </summary>
    public DistributionOfJerk(int bSplOrder,
                                        NCollection_HArray1<double> flatKnots,
                                        NCollection_HArray1<Point2D> poles,
                                        int derivativeOrder,
                                        BattenLaw law,
                                        int nbValAux = 0)
        : base(bSplOrder, flatKnots, poles, derivativeOrder, nbValAux)
    {
        // a copy of the law, as OCCT's by-value member (decision 858)
        _myLaw = new BattenLaw(law);
    }

    /// <summary>Computes the values <paramref name="jerk"/> at <paramref name="tParam"/>; false if they could not be computed.</summary>
    public override bool Value(math_Vector tParam, math_Vector jerk)
    {
        bool ok = true;
        int ier, ii, jj, kk;
        Xy cPrim = new(0.0, 0.0), cSecn = new(0.0, 0.0), cTroi = new(0.0, 0.0);
        int lastGradientIndex, firstNonZero, lastZero;

        // (0.0) initialisations generales
        jerk.Init(0.0);
        Matrix Base = new(1,
                               5,
                               1,
                               _myBSplOrder); // On shouhaite utiliser la derive troisieme
                                             // Dans EvalBsplineBasis C"' <=> DerivOrder = 4
                                             // et il faut ajouter 1 rang dans la matrice Base => 5 rangs

        ier = global::Kerf.Engine.Geometry.Splines.BSplCLib.EvalBsplineBasis(3,
                                        _myBSplOrder,
                                        _myFlatKnots.Array1(),
                                        tParam[tParam.Lower()],
                                        out firstNonZero,
                                        Base);
        if (ier != 0)
        {
            return false;
        }
        lastZero = firstNonZero - 1;
        firstNonZero = 2 * lastZero + 1;

        // (0.1) evaluation de CPrim et CScn
        for (ii = 1; ii <= _myBSplOrder; ii++)
        {
            cPrim += Base[2, ii] * _myPoles.Value(ii + lastZero).Coord();
            cSecn += Base[3, ii] * _myPoles.Value(ii + lastZero).Coord();
            cTroi += Base[4, ii] * _myPoles.Value(ii + lastZero).Coord();
        }

        // (1) Evaluation de la secousse locale = W*W
        double normeCPrim = cPrim.Modulus();
        double invNormeCPrim = 1 / normeCPrim;
        double invNormeCPrim2 = invNormeCPrim * invNormeCPrim;
        double produitC1C2 = cPrim * cSecn;
        double deriveNormeCPrim = produitC1C2 * invNormeCPrim2;

        double hauteur, wVal, mesure;
        double numRho = cPrim ^ cSecn;
        double numerateur = (cPrim ^ cTroi) - 3 * numRho * deriveNormeCPrim;
        double denominateur = Math.Pow(normeCPrim, 2.5);

        ok = _myLaw.Value(tParam[tParam.Lower()], out hauteur);
        if (!ok)
        {
            return ok;
        }

        mesure = Math.Pow(hauteur, 3) / 12;
        wVal = numerateur / denominateur;
        jerk[jerk.Lower()] = mesure * Math.Pow(wVal, 2);

        if (_myDerivativeOrder >= 1)
        {
            // (2) Evaluation du gradient de la secousse locale.

            math_Vector wGrad = new(1, 2 * _myBSplOrder + _myNbValAux), numGrad = new(1, 2 * _myBSplOrder),
              nGrad1 = new(1, 2 * _myBSplOrder), nGrad2 = new(1, 2 * _myBSplOrder), gradNormeCPrim = new(1, 2 * _myBSplOrder),
              ngdncPrim = new(1, 2 * _myBSplOrder), gradDeriveNormeCPrim = new(1, 2 * _myBSplOrder),
              gradNumRho = new(1, 2 * _myBSplOrder), numduGrad = new(1, 2 * _myBSplOrder);
            double facteur;
            double xPrim = cPrim.X();
            double yPrim = cPrim.Y();
            double xSecn = cSecn.X();
            double ySecn = cSecn.Y();
            double xTroi = cTroi.X();
            double yTroi = cTroi.Y();
            double invDenominateur = 1 / denominateur;
            double aux, auxBis;

            facteur = 2 * mesure * wVal;
            aux = 2.5 * numerateur * invNormeCPrim;
            auxBis = 2 * produitC1C2 * invNormeCPrim;
            kk = jerk.Lower() + firstNonZero;

            jj = 1;
            for (ii = 1; ii <= _myBSplOrder; ii++)
            {

                //     (2.1) Derivation en X
                gradNormeCPrim[jj] = xPrim * Base[2, ii] * invNormeCPrim;
                ngdncPrim[jj] = (xPrim * Base[3, ii] + xSecn * Base[2, ii]) - auxBis * gradNormeCPrim[jj];
                gradDeriveNormeCPrim[jj] = ngdncPrim[jj] * invNormeCPrim2;
                gradNumRho[jj] = ySecn * Base[2, ii] - yPrim * Base[3, ii];
                nGrad1[jj] = yTroi * Base[2, ii] - yPrim * Base[4, ii];
                nGrad2[jj] = -3 * (numRho * gradDeriveNormeCPrim[jj] + gradNumRho[jj] * deriveNormeCPrim);
                numGrad[jj] = nGrad1[jj] + nGrad2[jj];
                numduGrad[jj] = numGrad[jj] - aux * gradNormeCPrim[jj];
                wGrad[jj] = invDenominateur * numduGrad[jj];
                jerk[kk] = facteur * wGrad[jj];

                jj += 1;

                //     (2.2) Derivation en Y

                gradNormeCPrim[jj] = yPrim * Base[2, ii] * invNormeCPrim;
                ngdncPrim[jj] = (yPrim * Base[3, ii] + ySecn * Base[2, ii]) - auxBis * gradNormeCPrim[jj];
                gradDeriveNormeCPrim[jj] = ngdncPrim[jj] * invNormeCPrim2;
                gradNumRho[jj] = -xSecn * Base[2, ii] + xPrim * Base[3, ii];
                nGrad1[jj] = -xTroi * Base[2, ii] + xPrim * Base[4, ii];
                nGrad2[jj] = -3 * (numRho * gradDeriveNormeCPrim[jj] + gradNumRho[jj] * deriveNormeCPrim);
                numGrad[jj] = nGrad1[jj] + nGrad2[jj];
                numduGrad[jj] = numGrad[jj] - aux * gradNormeCPrim[jj];
                wGrad[jj] = invDenominateur * numduGrad[jj];
                jerk[kk + 1] = facteur * wGrad[jj];

                jj += 1;
                kk += 2;
            }
            if (_myNbValAux == 1)
            {
                //    (2.3) Gestion de la variable de glissement
                lastGradientIndex = jerk.Lower() + 2 * _myPoles.Length() + 1;
                wGrad[wGrad.Upper()] = 0.0;
                jerk[lastGradientIndex] = 0.0;
            }

            else
            {
                lastGradientIndex = jerk.Lower() + 2 * _myPoles.Length();
            }

            if (_myDerivativeOrder >= 2)
            {

                // (3) Evaluation du Hessien de la tension locale ----------------------

                double facteurX = (1 - Math.Pow(xPrim * invNormeCPrim, 2)) * invNormeCPrim;
                double facteurY = (1 - Math.Pow(yPrim * invNormeCPrim, 2)) * invNormeCPrim;
                double facteurXY = -(xPrim * invNormeCPrim) * (yPrim * invNormeCPrim) * invNormeCPrim;
                double facteurW = wVal * invNormeCPrim;

                double produit, produit2, produitV, hNumRho, dSeconde, nSeconde, hDeriveNormeCPrim, aux1,
                  deriveAuxBis;
                double vIntermed;
                int k1, k2, II, JJ;

                facteur = 2 * mesure;

                kk = firstNonZero;
                k2 = lastGradientIndex + (kk - 1) * kk / 2;

                for (ii = 2; ii <= 2 * _myBSplOrder; ii += 2)
                {
                    II = ii / 2;
                    k1 = k2 + firstNonZero;
                    k2 = k1 + kk;
                    kk += 2;
                    for (jj = 2; jj < ii; jj += 2)
                    {
                        JJ = jj / 2;
                        produit = Base[2, II] * Base[2, JJ];
                        produit2 = Base[2, II] * Base[3, JJ] + Base[3, II] * Base[2, JJ];
                        produitV = Base[2, II] * Base[4, JJ] - Base[4, II] * Base[2, JJ];
                        hNumRho = Base[2, II] * Base[3, JJ] - Base[3, II] * Base[2, JJ];

                        // derivation en XiXj
                        aux1 = invNormeCPrim2 * gradNormeCPrim[jj - 1];
                        deriveAuxBis =
                          2 * ((xPrim * Base[3, JJ] + xSecn * Base[2, JJ]) * invNormeCPrim - produitC1C2 * aux1);
                        hDeriveNormeCPrim = (produit2 - deriveAuxBis * gradNormeCPrim[ii - 1]
                                             - auxBis * (produit * invNormeCPrim - xPrim * Base[2, II] * aux1))
                                              * invNormeCPrim2
                                            - 2 * ngdncPrim[ii - 1] * aux1 * invNormeCPrim;
                        nSeconde =
                          -3
                          * (gradNumRho[ii - 1] * gradDeriveNormeCPrim[jj - 1] + numRho * hDeriveNormeCPrim
                             + gradNumRho[jj - 1] * gradDeriveNormeCPrim[ii - 1]);

                        dSeconde = facteurX * produit;
                        aux = normeCPrim * nSeconde + numGrad[ii - 1] * gradNormeCPrim[jj - 1]
                              - 2.5 * (numGrad[jj - 1] * gradNormeCPrim[ii - 1] + dSeconde * numerateur);
                        vIntermed = invDenominateur * (aux - 3.5 * gradNormeCPrim[jj - 1] * numduGrad[ii - 1]);

                        jerk[k1] = facteur * (wGrad[ii - 1] * wGrad[jj - 1] + facteurW * vIntermed);
                        k1++;

                        // derivation en XiYj
                        aux1 = invNormeCPrim2 * gradNormeCPrim[jj];
                        deriveAuxBis =
                          2 * ((yPrim * Base[3, JJ] + ySecn * Base[2, JJ]) * invNormeCPrim - produitC1C2 * aux1);
                        hDeriveNormeCPrim =
                          (-deriveAuxBis * gradNormeCPrim[ii - 1] + auxBis * xPrim * Base[2, II] * aux1)
                            * invNormeCPrim2
                          - 2 * ngdncPrim[ii - 1] * aux1 * invNormeCPrim;
                        nSeconde =
                          produitV
                          - 3
                              * (gradNumRho[ii - 1] * gradDeriveNormeCPrim[jj] + numRho * hDeriveNormeCPrim
                                 + gradNumRho[jj] * gradDeriveNormeCPrim[ii - 1] + hNumRho * deriveNormeCPrim);
                        dSeconde = facteurXY * produit;
                        aux = normeCPrim * nSeconde + numGrad[ii - 1] * gradNormeCPrim[jj]
                              - 2.5 * (numGrad[jj] * gradNormeCPrim[ii - 1] + dSeconde * numerateur);
                        vIntermed = invDenominateur * (aux - 3.5 * gradNormeCPrim[jj] * numduGrad[ii - 1]);
                        jerk[k1] = facteur * (wGrad[ii - 1] * wGrad[jj] + facteurW * vIntermed);
                        k1++;

                        // derivation en YiXj
                        // DSeconde calcule ci-dessus
                        aux1 = invNormeCPrim2 * gradNormeCPrim[jj - 1];
                        deriveAuxBis =
                          2 * ((xPrim * Base[3, JJ] + xSecn * Base[2, JJ]) * invNormeCPrim - produitC1C2 * aux1);
                        hDeriveNormeCPrim =
                          (-deriveAuxBis * gradNormeCPrim[ii] + auxBis * yPrim * Base[2, II] * aux1)
                            * invNormeCPrim2
                          - 2 * ngdncPrim[ii] * aux1 * invNormeCPrim;
                        nSeconde =
                          -produitV
                          - 3
                              * (gradNumRho[ii] * gradDeriveNormeCPrim[jj - 1] + numRho * hDeriveNormeCPrim
                                 + gradNumRho[jj - 1] * gradDeriveNormeCPrim[ii] - hNumRho * deriveNormeCPrim);
                        aux = normeCPrim * nSeconde + numGrad[ii] * gradNormeCPrim[jj - 1]
                              - 2.5 * (numGrad[jj - 1] * gradNormeCPrim[ii] + dSeconde * numerateur);
                        vIntermed = invDenominateur * (aux - 3.5 * gradNormeCPrim[jj - 1] * numduGrad[ii]);

                        jerk[k2] = facteur * (wGrad[ii] * wGrad[jj - 1] + facteurW * vIntermed);
                        k2++;

                        // derivation en YiYj
                        aux1 = invNormeCPrim2 * gradNormeCPrim[jj];
                        deriveAuxBis =
                          2 * ((yPrim * Base[3, JJ] + ySecn * Base[2, JJ]) * invNormeCPrim - produitC1C2 * aux1);
                        hDeriveNormeCPrim = (produit2 - deriveAuxBis * gradNormeCPrim[ii]
                                             - auxBis * (produit * invNormeCPrim - yPrim * Base[2, II] * aux1))
                                              * invNormeCPrim2
                                            - 2 * ngdncPrim[ii] * aux1 * invNormeCPrim;
                        nSeconde = -3
                                   * (gradNumRho[ii] * gradDeriveNormeCPrim[jj] + numRho * hDeriveNormeCPrim
                                      + gradNumRho[jj] * gradDeriveNormeCPrim[ii]);

                        dSeconde = facteurY * produit;
                        aux = nSeconde * normeCPrim + numGrad[ii] * gradNormeCPrim[jj]
                              - 2.5 * (numGrad[jj] * gradNormeCPrim[ii] + dSeconde * numerateur);
                        vIntermed = invDenominateur * (aux - 3.5 * gradNormeCPrim[jj] * numduGrad[ii]);
                        jerk[k2] = facteur * (wGrad[ii] * wGrad[jj] + facteurW * vIntermed);
                        k2++;
                    }

                    // case where jj = ii: triangular fill
                    produit = Math.Pow(Base[2, II], 2);
                    produit2 = 2 * Base[2, II] * Base[3, II];
                    //           ProduitV2 = Base

                    // derivation en XiXi
                    aux1 = invNormeCPrim2 * gradNormeCPrim[ii - 1];
                    deriveAuxBis =
                      2 * ((xPrim * Base[3, II] + xSecn * Base[2, II]) * invNormeCPrim - produitC1C2 * aux1);
                    hDeriveNormeCPrim = (produit2 - deriveAuxBis * gradNormeCPrim[ii - 1]
                                         - auxBis * (produit * invNormeCPrim - xPrim * Base[2, II] * aux1))
                                          * invNormeCPrim2
                                        - 2 * ngdncPrim[ii - 1] * aux1 * invNormeCPrim;
                    nSeconde =
                      -3 * (2 * gradNumRho[ii - 1] * gradDeriveNormeCPrim[ii - 1] + numRho * hDeriveNormeCPrim);
                    dSeconde = facteurX * produit;
                    aux = nSeconde * normeCPrim - 1.5 * numGrad[ii - 1] * gradNormeCPrim[ii - 1]
                          - 2.5 * dSeconde * numerateur;
                    vIntermed = invDenominateur * (aux - 3.5 * gradNormeCPrim[ii - 1] * numduGrad[ii - 1]);
                    jerk[k1] = facteur * (wGrad[ii - 1] * wGrad[ii - 1] + facteurW * vIntermed);
                    // derivation en XiYi
                    aux1 = invNormeCPrim2 * gradNormeCPrim[ii];
                    deriveAuxBis =
                      2 * ((yPrim * Base[3, II] + ySecn * Base[2, II]) * invNormeCPrim - produitC1C2 * aux1);
                    hDeriveNormeCPrim =
                      (-deriveAuxBis * gradNormeCPrim[ii - 1] + auxBis * xPrim * Base[2, II] * aux1)
                        * invNormeCPrim2
                      - 2 * ngdncPrim[ii - 1] * aux1 * invNormeCPrim;
                    nSeconde = -3
                               * (gradNumRho[ii - 1] * gradDeriveNormeCPrim[ii] + numRho * hDeriveNormeCPrim
                                  + gradNumRho[ii] * gradDeriveNormeCPrim[ii - 1]);
                    dSeconde = facteurXY * produit;
                    aux = nSeconde * normeCPrim + numGrad[ii] * gradNormeCPrim[ii - 1]
                          - 2.5 * (numGrad[ii - 1] * gradNormeCPrim[ii] + dSeconde * numerateur);
                    vIntermed = invDenominateur * (aux - 3.5 * gradNormeCPrim[ii - 1] * numduGrad[ii]);
                    jerk[k2] = facteur * (wGrad[ii] * wGrad[ii - 1] + facteurW * vIntermed);
                    k2++;

                    // derivation en YiYi
                    // Aux1 et DeriveAuxBis calcules au pas precedent ...
                    hDeriveNormeCPrim = (produit2 - deriveAuxBis * gradNormeCPrim[ii]
                                         - auxBis * (produit * invNormeCPrim - yPrim * Base[2, II] * aux1))
                                          * invNormeCPrim2
                                        - 2 * ngdncPrim[ii] * aux1 * invNormeCPrim;
                    nSeconde =
                      -3 * (2 * gradNumRho[ii] * gradDeriveNormeCPrim[ii] + numRho * hDeriveNormeCPrim);
                    dSeconde = facteurY * produit;
                    aux = nSeconde * normeCPrim - 1.5 * numGrad[ii] * gradNormeCPrim[ii]
                          - 2.5 * dSeconde * numerateur;
                    vIntermed = invDenominateur * (aux - 3.5 * gradNormeCPrim[ii] * numduGrad[ii]);
                    jerk[k2] = facteur * (wGrad[ii] * wGrad[ii] + facteurW * vIntermed);
                }
            }
        }

        // sortie standard
        return ok;
    }
}
