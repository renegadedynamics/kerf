// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_SampledCurveConstraint.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_SampledCurveConstraint.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/Plate/Plate_SampledCurveConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>
/// A constraint that n evenly spread hat-function averages of the plate's deformation over a sampled curve match the
/// same averages of the imposed values.
/// </summary>
/// <remarks>A class: OCCT's copies share the linear constraint's arrays (see <see cref="Plate_LinearXYZConstraint"/>).</remarks>
public class Plate_SampledCurveConstraint
{
    private readonly Plate_LinearXYZConstraint myLXYZC;

    private static double B0(double t)
    {
        double s = t;
        if (s < 0.0)
        {
            s = -s;
        }
        s = 1.0 - s;
        if (s < 0.0)
        {
            s = 0;
        }
        return s;
    }

    public Plate_SampledCurveConstraint(NCollection_Sequence<Plate_PinpointConstraint> SOPPC, int n)
    {
        myLXYZC = new Plate_LinearXYZConstraint(n, SOPPC.Length());
        int m = SOPPC.Length();

        if (n > m)
        {
            throw new DimensionMismatchException();
        }
        for (int index = 1; index <= m; index++)
        {
            myLXYZC.SetPPC(index, SOPPC.Value(index));
        }

        double ratio = (double)(n + 1) / (double)(m + 1);
        for (int i = 1; i <= n; i++)
        {
            for (int j = 1; j <= m; j++)
            {
                myLXYZC.SetCoeff(i, j, B0(ratio * j - i));
            }
        }
    }

    public Plate_LinearXYZConstraint LXYZC() => myLXYZC;
}
