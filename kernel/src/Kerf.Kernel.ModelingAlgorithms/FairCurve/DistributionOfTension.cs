// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_DistributionOfTension.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_DistributionOfTension.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_DistributionOfTension.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry;

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Construction.Fairing;

/// <summary>The tension distribution of a batten: its local energy of stretching and its derivatives.</summary>
internal class DistributionOfTension : DistributionOfEnergy
{
    private double _myLengthSliding;
    private readonly BattenLaw _myLaw;
    private readonly double _myHeight;

    /// <summary>
    /// The tension of the curve of order <paramref name="bSplOrder"/> over <paramref name="flatKnots"/> and
    /// <paramref name="poles"/> (kept, not copied). <paramref name="uniform"/> uses the law's height at the middle
    /// everywhere (MinimalVariation uses it to avoid parametrisation problems).
    /// </summary>
    public DistributionOfTension(int bSplOrder,
                                           NCollection_HArray1<double> flatKnots,
                                           NCollection_HArray1<Point2D> poles,
                                           int derivativeOrder,
                                           double lengthSliding,
                                           BattenLaw law,
                                           int nbValAux = 0,
                                           bool uniform = false)
        : base(bSplOrder, flatKnots, poles, derivativeOrder, nbValAux)
    {
        _myLengthSliding = lengthSliding;
        // a copy of the law, as OCCT's by-value member (decision 858)
        _myLaw = new BattenLaw(law);
        if (uniform)
        {
            _myLaw.Value(0.5, out _myHeight);
        } // it used in MVC to avoid Parametrization Problemes
        else
        {
            _myHeight = 0;
        }
    }

    /// <summary>Changes the length sliding.</summary>
    public void SetLengthSliding(double lengthSliding)
    {
        _myLengthSliding = lengthSliding;
    }

    /// <summary>Computes the values <paramref name="fTension"/> at <paramref name="tParam"/>; false if they could not be computed.</summary>
    public override bool Value(math_Vector tParam, math_Vector fTension)
    {
        bool ok = true;
        int ier, ii, jj, kk;
        Xy cPrim = new(0.0, 0.0);
        int lastGradientIndex, firstNonZero, lastZero;

        // (0.0) initialisations generales
        fTension.Init(0.0);
        Matrix Base = new(1,
                               3,
                               1,
                               _myBSplOrder); // On shouhaite utiliser la derive premieres
                                             // Dans EvalBsplineBasis C' <=> DerivOrder = 2
                                             // et il faut ajouter 1 rang dans la matrice Base => 3 rang

        ier = global::Kerf.Engine.Geometry.Splines.BSplCLib.EvalBsplineBasis(1,
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

        // (0.1) evaluation de CPrim
        for (ii = 1; ii <= _myBSplOrder; ii++)
        {
            cPrim += Base[2, ii] * _myPoles.Value(ii + lastZero).Coord();
        }

        // (1) Evaluation de la tension locale --------------------------------
        double normeCPrim = cPrim.Modulus();
        double hauteur, difference;

        if (_myHeight > 0)
        {
            hauteur = _myHeight;
        } // it used in MVC to avoid Parametrization Problemes
        else
        {
            ok = _myLaw.Value(tParam[tParam.Lower()], out hauteur);
            if (!ok)
            {
                return ok;
            }
        }
        difference = normeCPrim - _myLengthSliding;

        fTension[fTension.Lower()] = hauteur * Math.Pow(difference, 2) / _myLengthSliding;

        if (_myDerivativeOrder >= 1)
        {
            // (2) Evaluation du gradient de la tension locale ----------------------
            math_Vector gradDifference = new(1, 2 * _myBSplOrder + _myNbValAux);
            double xaux, yaux, facteur;

            xaux = cPrim.X() / normeCPrim;
            yaux = cPrim.Y() / normeCPrim;
            facteur = 2 * hauteur * difference / _myLengthSliding;

            kk = fTension.Lower() + firstNonZero;
            jj = 1;
            for (ii = 1; ii <= _myBSplOrder; ii++)
            {
                gradDifference[jj] = Base[2, ii] * xaux;
                fTension[kk] = facteur * gradDifference[jj];
                jj += 1;
                gradDifference[jj] = Base[2, ii] * yaux;
                fTension[kk + 1] = facteur * gradDifference[jj];
                jj += 1;
                kk += 2;
            }
            if (_myNbValAux == 1)
            {
                lastGradientIndex = fTension.Lower() + 2 * _myPoles.Length() + 1;
                gradDifference[gradDifference.Upper()] = (1 - Math.Pow(normeCPrim / _myLengthSliding, 2));
                fTension[lastGradientIndex] = hauteur * gradDifference[gradDifference.Upper()];
            }

            else
            {
                lastGradientIndex = fTension.Lower() + 2 * _myPoles.Length();
            }

            if (_myDerivativeOrder >= 2)
            {

                // (3) Evaluation du Hessien de la tension locale ----------------------

                double facteurX = difference * (1 - Math.Pow(xaux, 2)) / normeCPrim;
                double facteurY = difference * (1 - Math.Pow(yaux, 2)) / normeCPrim;
                double facteurXY = -difference * xaux * yaux / normeCPrim;
                double produit;
                int k1, k2;

                facteur = 2 * hauteur / _myLengthSliding;

                kk = firstNonZero;
                k2 = lastGradientIndex + (kk - 1) * kk / 2;

                for (ii = 2; ii <= 2 * _myBSplOrder; ii += 2)
                {
                    k1 = k2 + firstNonZero;
                    k2 = k1 + kk;
                    kk += 2;
                    for (jj = 2; jj < ii; jj += 2)
                    {
                        produit = Base[2, ii / 2] * Base[2, jj / 2];

                        fTension[k1] = facteur
                                       * (gradDifference[ii - 1] * gradDifference[jj - 1]
                                          + facteurX * produit); // derivation en XiXj
                        k1++;
                        fTension[k1] = facteur
                                       * (gradDifference[ii] * gradDifference[jj - 1]
                                          + facteurXY * produit); // derivation en YiXj
                        k1++;
                        fTension[k2] = facteur
                                       * (gradDifference[ii - 1] * gradDifference[jj]
                                          + facteurXY * produit); // derivation en XiYj
                        k2++;
                        fTension[k2] =
                          facteur
                          * (gradDifference[ii] * gradDifference[jj] + facteurY * produit); // derivation en YiYj
                        k2++;
                    }
                    // case where jj = ii: triangular fill
                    produit = Math.Pow(Base[2, ii / 2], 2);

                    fTension[k1] = facteur
                                   * (gradDifference[ii - 1] * gradDifference[ii - 1]
                                      + facteurX * produit); // derivation en XiXi
                    fTension[k2] = facteur
                                   * (gradDifference[ii] * gradDifference[ii - 1]
                                      + facteurXY * produit); // derivation en XiYi
                    k2++;
                    fTension[k2] =
                      facteur
                      * (gradDifference[ii] * gradDifference[ii] + facteurY * produit); // derivation en YiYi
                }
                if (_myNbValAux == 1)
                {
                    facteurX = -2 * cPrim.X() * hauteur / Math.Pow(_myLengthSliding, 2);
                    facteurY = -2 * cPrim.Y() * hauteur / Math.Pow(_myLengthSliding, 2);

                    ii = lastGradientIndex - fTension.Lower();
                    kk = lastGradientIndex + (ii - 1) * ii / 2 + firstNonZero;
                    for (ii = 1; ii <= _myBSplOrder; ii++)
                    {
                        fTension[kk] = facteurX * Base[2, ii];
                        kk++;
                        fTension[kk] = facteurY * Base[2, ii];
                        kk++;
                    }
                    fTension[fTension.Upper()] =
                      2 * hauteur * Math.Pow(normeCPrim / _myLengthSliding, 2) / _myLengthSliding;
                }
            }
        }

        // sortie standard
        return ok;
    }
}
