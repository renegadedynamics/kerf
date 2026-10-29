// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_LogSample.hxx
// Port of: src/ModelingData/TKGeomBase/GeomLib/GeomLib_LogSample.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>A sampling of [A, B] in N points spaced logarithmically: A - 1 + exp(Index * log(B - A) / N).</summary>
public sealed class GeomLib_LogSample : math_FunctionSample
{
    private readonly double myF;
    private readonly double myexp;

    public GeomLib_LogSample(double A, double B, int N)
        : base(A, B, N)
    {
        myF = A - 1;
        myexp = Math.Log(B - A) / N;
    }

    /// <summary>The parameter of the point Index (A for 1, B for NbPoints). Raises Standard_OutOfRange outside.</summary>
    public override double GetParameter(int Index)
    {
        int aN = NbPoints();
        if ((Index >= aN) || (Index <= 1))
        {
            double aA, aB;
            Bounds(out aA, out aB);
            if (Index == 1)
            {
                return aA;
            }
            else if (Index == aN)
            {
                return aB;
            }
            else
            {
                throw new Standard_OutOfRange("GeomLib_LogSample::GetParameter");
            }
        }
        double v = myF + Math.Exp(myexp * Index);
        return v;
    }
}
