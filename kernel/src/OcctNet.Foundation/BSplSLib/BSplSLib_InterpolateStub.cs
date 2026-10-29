// TEMPORARY stub so that the Convert branch builds before BSplSLib is merged. Drop on merge.

namespace Occt;

public static partial class BSplSLib
{
    public static void Interpolate(int UDegree, int VDegree, NCollection_Array1<double> UFlatKnots,
                                   NCollection_Array1<double> VFlatKnots, NCollection_Array1<double> UParameters,
                                   NCollection_Array1<double> VParameters, NCollection_Array2<gp_Pnt> Poles,
                                   out int InversionProblem)
        => throw new NotImplementedException("BSplSLib.Interpolate stub");
}
