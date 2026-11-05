// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_DistributionOfEnergy.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/FairCurve/FairCurve_DistributionOfEnergy.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Kerf.Engine;

namespace Kerf.Engine.Construction.Fairing;

/// <summary>
/// The base of the energy distributions of a fair curve: functions of the curve's parameter whose values are the
/// local energy, then (by derivative order) its gradient and hessian with respect to the poles.
/// </summary>
/// <remarks>
/// A working object (decision 21). It keeps the caller's knot and pole arrays as OCCT keeps the handles, and reads the
/// poles as the energy that owns it moves them.
/// </remarks>
internal abstract class DistributionOfEnergy : FunctionSet
{
    protected int _myBSplOrder;
    protected NCollection_HArray1<double> _myFlatKnots;
    protected NCollection_HArray1<Point2D> _myPoles;
    protected int _myDerivativeOrder;
    protected int _myNbVar;
    protected int _myNbEqua;
    protected int _myNbValAux;

    /// <summary>The number of variables of the function.</summary>
    public override int NbVariables()
    {
        return _myNbVar;
    }

    /// <summary>The number of equations of the function.</summary>
    public override int NbEquations()
    {
        return _myNbEqua;
    }

    protected DistributionOfEnergy(int bSplOrder,
                                             NCollection_HArray1<double> flatKnots,
                                             NCollection_HArray1<Point2D> poles,
                                             int derivativeOrder,
                                             int nbValAux = 0)
    {
        _myBSplOrder = bSplOrder;
        _myFlatKnots = flatKnots;
        _myPoles = poles;
        _myDerivativeOrder = derivativeOrder;
        _myNbValAux = nbValAux;
        _myNbVar = 1;
        SetDerivativeOrder(derivativeOrder);
    }

    /// <summary>Sets the order of the derivatives computed (0: the energy; 1: and its gradient; 2: and its hessian).</summary>
    public void SetDerivativeOrder(int derivativeOrder)
    {
        _myNbEqua = 1;
        if (derivativeOrder >= 1)
        {
            _myNbEqua += 2 * _myPoles.Length() + _myNbValAux;
        }
        if (derivativeOrder >= 2)
        {
            _myNbEqua = _myNbEqua + (_myNbEqua - 1) * _myNbEqua / 2;
        }
        _myDerivativeOrder = derivativeOrder;
    }
}
