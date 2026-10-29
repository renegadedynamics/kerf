// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_MyGaussFunction.hxx
// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_MyGaussFunction.lxx
// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_MyGaussFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>For the Gauss integration: computes the values of a <see cref="CPnts_RealFunction"/> on client data.</summary>
public sealed class CPnts_MyGaussFunction : math_Function
{
    private CPnts_RealFunction? myFunction;
    private object? myData;

    public CPnts_MyGaussFunction()
    {
        myFunction = null;
        myData = null;
    }

    /// <summary>F is the function, D the client data; each value is computed as F(X, D).</summary>
    public void Init(CPnts_RealFunction F, object? D)
    {
        myFunction = F;
        myData = D;
    }

    public override bool Value(double X, out double F)
    {
        F = myFunction!(X, myData);
        return true;
    }
}
