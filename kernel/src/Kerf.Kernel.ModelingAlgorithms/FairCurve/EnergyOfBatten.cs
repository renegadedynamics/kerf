// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_EnergyOfBatten.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_EnergyOfBatten.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_EnergyOfBatten.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.Construction.Fairing.AnalysisCode;

namespace Kerf.Engine.Construction.Fairing;

/// <summary>The energy criterion that FairCurve_Batten minimises: the batten's tension and sagging.</summary>
public class EnergyOfBatten : Energy
{
    private double _myLengthSliding;
    private readonly double _originalSliding;
    private readonly BattenLaw _myBattenLaw;
    private readonly DistributionOfTension _myTension;
    private readonly DistributionOfSagging _mySagging;
    private AnalysisCode _myStatus;

    /// <summary>
    /// The energy of the curve of order <paramref name="bSplOrder"/> over <paramref name="flatKnots"/> and
    /// <paramref name="poles"/> (kept, and written by each evaluation). Angles are measured from the Ox axis.
    /// </summary>
    public EnergyOfBatten(int bSplOrder,
                                    NCollection_HArray1<double> flatKnots,
                                    NCollection_HArray1<Point2D> poles,
                                    int contrOrder1,
                                    int contrOrder2,
                                    BattenLaw law,
                                    double lengthSliding,
                                    bool freeSliding = true,
                                    double angle1 = 0,
                                    double angle2 = 0)
        : base(poles, contrOrder1, contrOrder2, freeSliding, angle1, angle2)
    {
        _myLengthSliding = lengthSliding;
        _originalSliding = lengthSliding;
        // by-value members in OCCT: each keeps its own copy of the law (decision 858)
        _myBattenLaw = new BattenLaw(law);
        _myTension = new DistributionOfTension(bSplOrder, flatKnots, poles, 1, lengthSliding, law, freeSliding ? 1 : 0);
        _mySagging = new DistributionOfSagging(bSplOrder, flatKnots, poles, 1, law, freeSliding ? 1 : 0);
        _myStatus = AnalysisCode.OK;
    }

    /// <summary>The length sliding (P1P2 + Sliding).</summary>
    public double LengthSliding()
    {
        return _myLengthSliding;
    }

    /// <summary>The status of the latest evaluation.</summary>
    public AnalysisCode Status()
    {
        return _myStatus;
    }

    /// <summary>Computes the variables <paramref name="x"/> that correspond to the poles (and the sliding).</summary>
    public override bool Variable(math_Vector x)
    {
        bool ok;
        ok = base.Variable(x);
        if (_myWithAuxValue)
        {
            x[x.Upper()] = _myLengthSliding;
        }
        return ok;
    }

    protected override void ComputePoles(math_Vector x)
    {
        base.ComputePoles(x);
        if (_myWithAuxValue)
        {
            _myLengthSliding = x[x.Upper()];
        }
    }

    protected override bool Compute(int derivativeOrder, math_Vector result)
    {
        math_Vector debut = new(1, 1, 0.0), fin = new(1, 1, 1.0);
        math_IntegerVector myOrder = new(1, 1, 24);
        bool ok = false;

        // Blindage contre les longueur de glissement trop exotique
        _myStatus = AnalysisCode.OK;
        if (_myLengthSliding > 10 * _originalSliding)
        {
            _myStatus = AnalysisCode.InfiniteSliding;
            return false;
        }

        if (_myLengthSliding < _originalSliding / 100)
        {
            _myLengthSliding = _originalSliding / 100;
        }

        // Mise a jour des objets sous-fonction
        _myTension.SetDerivativeOrder(derivativeOrder);
        _myTension.SetLengthSliding(_myLengthSliding);
        _mySagging.SetDerivativeOrder(derivativeOrder);
        // OCCT: updates this copy of the law only, which nothing reads; the distributions keep the initial sliding
        _myBattenLaw.SetSliding(_myLengthSliding);

        //  Integrations

        // on decoupe afin d'avoir au moins 2 points d'integration par poles
        // 24 points de Gauss => 12 poles maximum.

        int nbInterv = (_myPoles.Length() - 1) / 12 + 1, ii;
        double delta = 1.0 / nbInterv;
        result.Init(0);

        for (ii = 1; ii <= nbInterv; ii++)
        {
            debut[1] = (ii - 1) * delta;
            fin[1] = ii * delta;

            GaussSetIntegration sumTension = new(_myTension, debut, fin, myOrder);
            ok = sumTension.IsDone;
            if (!ok)
            {
                return ok;
            }

            GaussSetIntegration sumSagging = new(_mySagging, debut, fin, myOrder);
            ok = sumSagging.IsDone;

            // OCCT: a failed sagging integration goes on to the next interval, whose result is returned
            if (ok)
            {
                result += sumSagging.Value() + sumTension.Value();
            }
        }

        return ok;
    }
}
