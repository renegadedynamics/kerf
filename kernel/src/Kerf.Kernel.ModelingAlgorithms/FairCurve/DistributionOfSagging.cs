// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_DistributionOfSagging.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_DistributionOfSagging.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;

namespace Kerf.Engine.Construction.Fairing;

/// <summary>The sagging (flexion) distribution of a batten: its local bending energy and its derivatives.</summary>
internal class DistributionOfSagging : DistributionOfEnergy
{
    private readonly BattenLaw _myLaw;

    /// <summary>
    /// The sagging of the curve of order <paramref name="bSplOrder"/> over <paramref name="flatKnots"/> and
    /// <paramref name="poles"/> (kept, not copied).
    /// </summary>
    public DistributionOfSagging(int bSplOrder,
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

    /// <summary>Computes the values <paramref name="flexion"/> at <paramref name="tParam"/>; false if they could not be computed.</summary>
    public override bool Value(math_Vector tParam, math_Vector flexion)
    {
        bool ok = true;
        int ier, ii, jj, kk;
        Xy cPrim = new(0.0, 0.0), cSecn = new(0.0, 0.0);
        int lastGradientIndex, firstNonZero, lastZero;

        // (0.0) initialisations generales
        flexion.Init(0.0);
        Matrix Base = new(1,
                               4,
                               1,
                               _myBSplOrder); // On shouhaite utiliser la derive premieres
                                             // Dans EvalBsplineBasis C" <=> DerivOrder = 3
                                             // et il faut ajouter 1 rang dans la matrice Base => 4 rang

        ier = global::Kerf.Engine.Geometry.Splines.BSplCLib.EvalBsplineBasis(2,
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
        }

        // (1) Evaluation de la flexion locale = W*W
        double normeCPrim = cPrim.Modulus();
        double invNormeCPrim = 1 / normeCPrim;
        double hauteur, wVal, mesure;
        double numerateur = cPrim ^ cSecn;
        double denominateur = Math.Pow(normeCPrim, 2.5);

        ok = _myLaw.Value(tParam[tParam.Lower()], out hauteur);
        if (!ok)
        {
            return ok;
        }

        mesure = Math.Pow(hauteur, 3) / 12;
        wVal = numerateur / denominateur;
        flexion[flexion.Lower()] = mesure * Math.Pow(wVal, 2);

        if (_myDerivativeOrder >= 1)
        {
            // (2) Evaluation du gradient de la flexion locale.

            math_Vector wGrad = new(1, 2 * _myBSplOrder + _myNbValAux), numGrad = new(1, 2 * _myBSplOrder + _myNbValAux),
              gradNormeCPrim = new(1, 2 * _myBSplOrder + _myNbValAux), numduGrad = new(1, 2 * _myBSplOrder + _myNbValAux);
            double facteur;
            double xPrim = cPrim.X();
            double yPrim = cPrim.Y();
            double xSecn = cSecn.X();
            double ySecn = cSecn.Y();
            double invDenominateur = 1 / denominateur;
            double aux;

            facteur = 2 * mesure * wVal;
            aux = 2.5 * numerateur * invNormeCPrim;
            kk = flexion.Lower() + firstNonZero;

            jj = 1;
            for (ii = 1; ii <= _myBSplOrder; ii++)
            {

                //     (2.1) Derivation en X
                numGrad[jj] = ySecn * Base[2, ii] - yPrim * Base[3, ii];
                gradNormeCPrim[jj] = xPrim * Base[2, ii] * invNormeCPrim;
                numduGrad[jj] = numGrad[jj] - aux * gradNormeCPrim[jj];
                wGrad[jj] = invDenominateur * numduGrad[jj];
                flexion[kk] = facteur * wGrad[jj];
                jj += 1;

                //     (2.2) Derivation en Y
                numGrad[jj] = -xSecn * Base[2, ii] + xPrim * Base[3, ii];
                gradNormeCPrim[jj] = yPrim * Base[2, ii] * invNormeCPrim;
                numduGrad[jj] = numGrad[jj] - aux * gradNormeCPrim[jj];
                wGrad[jj] = invDenominateur * numduGrad[jj];
                flexion[kk + 1] = facteur * wGrad[jj];
                jj += 1;
                kk += 2;
            }
            if (_myNbValAux == 1)
            {
                //    (2.3) Gestion de la variable de glissement
                lastGradientIndex = flexion.Lower() + 2 * _myPoles.Length() + 1;
                wGrad[wGrad.Upper()] = 0.0;
            }

            else
            {
                lastGradientIndex = flexion.Lower() + 2 * _myPoles.Length();
            }

            if (_myDerivativeOrder >= 2)
            {

                // (3) Evaluation du Hessien de la tension locale ----------------------

                double facteurX = (1 - Math.Pow(xPrim * invNormeCPrim, 2)) * invNormeCPrim;
                double facteurY = (1 - Math.Pow(yPrim * invNormeCPrim, 2)) * invNormeCPrim;
                double facteurXY = -(xPrim * invNormeCPrim) * (yPrim * invNormeCPrim) * invNormeCPrim;
                double facteurW = wVal * invNormeCPrim;

                double produit, dSeconde, nSeconde;
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
                        nSeconde = Base[2, II] * Base[3, JJ] - Base[3, II] * Base[2, JJ];

                        // derivation en XiXj
                        dSeconde = facteurX * produit;
                        aux = numGrad[ii - 1] * gradNormeCPrim[jj - 1]
                              - 2.5 * (numGrad[jj - 1] * gradNormeCPrim[ii - 1] + dSeconde * numerateur);
                        vIntermed = invDenominateur * (aux - 3.5 * gradNormeCPrim[jj - 1] * numduGrad[ii - 1]);

                        flexion[k1] = facteur * (wGrad[ii - 1] * wGrad[jj - 1] + facteurW * vIntermed);
                        k1++;

                        // derivation en XiYj
                        dSeconde = facteurXY * produit;
                        aux = normeCPrim * nSeconde + numGrad[ii - 1] * gradNormeCPrim[jj]
                              - 2.5 * (numGrad[jj] * gradNormeCPrim[ii - 1] + dSeconde * numerateur);
                        vIntermed = invDenominateur * (aux - 3.5 * gradNormeCPrim[jj] * numduGrad[ii - 1]);
                        flexion[k1] = facteur * (wGrad[ii - 1] * wGrad[jj] + facteurW * vIntermed);
                        k1++;

                        // derivation en YiXj
                        // DSeconde calcule ci-dessus
                        aux = -normeCPrim * nSeconde + numGrad[ii] * gradNormeCPrim[jj - 1]
                              - 2.5 * (numGrad[jj - 1] * gradNormeCPrim[ii] + dSeconde * numerateur);
                        vIntermed = invDenominateur * (aux - 3.5 * gradNormeCPrim[jj - 1] * numduGrad[ii]);

                        flexion[k2] = facteur * (wGrad[ii] * wGrad[jj - 1] + facteurW * vIntermed);
                        k2++;

                        // derivation en YiYj
                        dSeconde = facteurY * produit;
                        aux = numGrad[ii] * gradNormeCPrim[jj]
                              - 2.5 * (numGrad[jj] * gradNormeCPrim[ii] + dSeconde * numerateur);
                        vIntermed = invDenominateur * (aux - 3.5 * gradNormeCPrim[jj] * numduGrad[ii]);
                        flexion[k2] = facteur * (wGrad[ii] * wGrad[jj] + facteurW * vIntermed);
                        k2++;
                    }

                    // case where jj = ii: triangular fill
                    produit = Math.Pow(Base[2, II], 2);

                    // derivation en XiXi
                    dSeconde = facteurX * produit;
                    aux = -1.5 * numGrad[ii - 1] * gradNormeCPrim[ii - 1] - 2.5 * dSeconde * numerateur;
                    vIntermed = invDenominateur * (aux - 3.5 * gradNormeCPrim[ii - 1] * numduGrad[ii - 1]);
                    flexion[k1] = facteur * (wGrad[ii - 1] * wGrad[ii - 1] + facteurW * vIntermed);
                    // derivation en XiYi
                    dSeconde = facteurXY * produit;
                    aux = numGrad[ii - 1] * gradNormeCPrim[ii]
                          - 2.5 * (numGrad[ii] * gradNormeCPrim[ii - 1] + dSeconde * numerateur);
                    vIntermed = invDenominateur * (aux - 3.5 * gradNormeCPrim[ii] * numduGrad[ii - 1]);
                    flexion[k2] = facteur * (wGrad[ii] * wGrad[ii - 1] + facteurW * vIntermed);
                    k2++;

                    // derivation en YiYi
                    dSeconde = facteurY * produit;
                    aux = -1.5 * numGrad[ii] * gradNormeCPrim[ii] - 2.5 * dSeconde * numerateur;
                    vIntermed = invDenominateur * (aux - 3.5 * gradNormeCPrim[ii] * numduGrad[ii]);
                    flexion[k2] = facteur * (wGrad[ii] * wGrad[ii] + facteurW * vIntermed);
                }
            }
        }

        // sortie standard
        return ok;
    }
}
