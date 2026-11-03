// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_LogSample.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_LogSample.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Numerics;
using Occt;

namespace Kerf.Engine.Geometry.Library;

/// <summary>A sampling of [A, B] in N points spaced logarithmically: A - 1 + exp(Index * log(B - A) / N).</summary>
public sealed class LogSample : FunctionSample
{
    private readonly double _f;
    private readonly double _myexp;

    public LogSample(double a, double b, int n)
        : base(a, b, n)
    {
        _f = a - 1;
        _myexp = Math.Log(b - a) / n;
    }

    /// <summary>The parameter of the point Index (A for 1, B for NbPoints). Raises Standard_OutOfRange outside.</summary>
    public override double GetParameter(int index)
    {
        int n = PointCount;
        if ((index >= n) || (index <= 1))
        {
            double a, b;
            Bounds(out a, out b);
            if (index == 1)
            {
                return a;
            }
            else if (index == n)
            {
                return b;
            }
            else
            {
                throw new OutOfRangeException("GeomLib_LogSample::GetParameter");
            }
        }
        double v = _f + Math.Exp(_myexp * index);
        return v;
    }
}
