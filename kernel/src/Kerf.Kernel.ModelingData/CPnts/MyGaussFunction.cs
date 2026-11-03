// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_MyGaussFunction.hxx
// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_MyGaussFunction.lxx
// Port of: src/ModelingData/TKGeomBase/CPnts/CPnts_MyGaussFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;
using Occt;

namespace Kerf.Engine.Measurement.Points;

/// <summary>For the Gauss integration: computes the values of a <see cref="RealFunction"/> on client data.</summary>
public sealed class MyGaussFunction : Function
{
    private RealFunction? _function;
    private object? _data;

    public MyGaussFunction()
    {
        _function = null;
        _data = null;
    }

    /// <summary>F is the function, D the client data; each value is computed as F(X, D).</summary>
    public void Init(RealFunction f, object? d)
    {
        _function = f;
        _data = d;
    }

    public override bool Value(double x, out double f)
    {
        f = _function!(x, _data);
        return true;
    }
}
