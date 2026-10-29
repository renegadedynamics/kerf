// TEMPORARY stub so that the Convert branch builds before BSplSLib is merged. Drop on merge.

namespace Occt;

public static partial class BSplSLib
{
    public static void Interpolate(int UDegree, int VDegree, NCollection_Array1<double> UFlatKnots,
                                   NCollection_Array1<double> VFlatKnots, NCollection_Array1<double> UParameters,
                                   NCollection_Array1<double> VParameters, NCollection_Array2<gp_Pnt> Poles,
                                   out int InversionProblem)
        => throw new NotImplementedException("BSplSLib.Interpolate stub");

    public static void D0(double U, double V, int UIndex, int VIndex, NCollection_Array2<gp_Pnt> Poles,
                          NCollection_Array2<double>? Weights, NCollection_Array1<double> UKnots,
                          NCollection_Array1<double> VKnots, NCollection_Array1<int>? UMults,
                          NCollection_Array1<int>? VMults, int UDegree, int VDegree, bool URat, bool VRat,
                          bool UPer, bool VPer, out gp_Pnt P)
        => throw new NotImplementedException("BSplSLib.D0 stub");
}
