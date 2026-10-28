// STUB: signatures only, replaced by the port
namespace Occt;

public static class PLib
{
    public static NCollection_Array1<double>? NoWeights() => null;
    public static NCollection_Array2<double>? NoWeights2() => null;
    public static void SetPoles(NCollection_Array1<gp_Pnt> Poles, NCollection_Array1<double> FP) => throw new NotImplementedException();
    public static void SetPoles(NCollection_Array1<gp_Pnt> Poles, NCollection_Array1<double> Weights, NCollection_Array1<double> FP) => throw new NotImplementedException();
    public static void GetPoles(NCollection_Array1<double> FP, NCollection_Array1<gp_Pnt> Poles) => throw new NotImplementedException();
    public static void GetPoles(NCollection_Array1<double> FP, NCollection_Array1<gp_Pnt> Poles, NCollection_Array1<double> Weights) => throw new NotImplementedException();
    public static void SetPoles(NCollection_Array1<gp_Pnt2d> Poles, NCollection_Array1<double> FP) => throw new NotImplementedException();
    public static void SetPoles(NCollection_Array1<gp_Pnt2d> Poles, NCollection_Array1<double> Weights, NCollection_Array1<double> FP) => throw new NotImplementedException();
    public static void GetPoles(NCollection_Array1<double> FP, NCollection_Array1<gp_Pnt2d> Poles) => throw new NotImplementedException();
    public static void GetPoles(NCollection_Array1<double> FP, NCollection_Array1<gp_Pnt2d> Poles, NCollection_Array1<double> Weights) => throw new NotImplementedException();
    public static double Bin(int N, int P) => throw new NotImplementedException();
    public static void RationalDerivative(int Degree, int N, int Dimension, ReadOnlySpan<double> Ders, Span<double> RDers, bool All = true) => throw new NotImplementedException();
    public static void RationalDerivatives(int DerivativesRequest, int Dimension, ReadOnlySpan<double> PolesDerivatives, ReadOnlySpan<double> WeightsDerivatives, Span<double> RationalDerivates) => throw new NotImplementedException();
    public static void EvalPolynomial(double U, int DerivativeOrder, int Degree, int Dimension, ReadOnlySpan<double> PolynomialCoeff, Span<double> Results) => throw new NotImplementedException();
    public static void NoDerivativeEvalPolynomial(double U, int Degree, int Dimension, int DegreeDimension, ReadOnlySpan<double> PolynomialCoeff, Span<double> Results) => throw new NotImplementedException();
    public static void EvalPoly2Var(double U, double V, int UDerivativeOrder, int VDerivativeOrder, int UDegree, int VDegree, int Dimension, ReadOnlySpan<double> PolynomialCoeff, Span<double> Results) => throw new NotImplementedException();
    public static int EvalLagrange(double U, int DerivativeOrder, int Degree, int Dimension, ReadOnlySpan<double> ValueArray, ReadOnlySpan<double> ParameterArray, Span<double> Results) => throw new NotImplementedException();
    public static int EvalCubicHermite(double U, int DerivativeOrder, int Dimension, ReadOnlySpan<double> ValueArray, ReadOnlySpan<double> DerivativeArray, ReadOnlySpan<double> ParameterArray, Span<double> Results) => throw new NotImplementedException();
    public static bool HermiteCoefficients(double FirstParameter, double LastParameter, int FirstOrder, int LastOrder, math_Matrix MatrixCoefs) => throw new NotImplementedException();
    public static void CoefficientsPoles(NCollection_Array1<gp_Pnt> Coefs, NCollection_Array1<double>? WCoefs, NCollection_Array1<gp_Pnt> Poles, NCollection_Array1<double>? WPoles) => throw new NotImplementedException();
    public static void CoefficientsPoles(NCollection_Array1<gp_Pnt2d> Coefs, NCollection_Array1<double>? WCoefs, NCollection_Array1<gp_Pnt2d> Poles, NCollection_Array1<double>? WPoles) => throw new NotImplementedException();
    public static void CoefficientsPoles(NCollection_Array1<double> Coefs, NCollection_Array1<double>? WCoefs, NCollection_Array1<double> Poles, NCollection_Array1<double>? WPoles) => throw new NotImplementedException();
    public static void CoefficientsPoles(int dim, NCollection_Array1<double> Coefs, NCollection_Array1<double>? WCoefs, NCollection_Array1<double> Poles, NCollection_Array1<double>? WPoles) => throw new NotImplementedException();
    public static void Trimming(double U1, double U2, NCollection_Array1<gp_Pnt> Coeffs, NCollection_Array1<double>? WCoeffs) => throw new NotImplementedException();
    public static void Trimming(double U1, double U2, NCollection_Array1<gp_Pnt2d> Coeffs, NCollection_Array1<double>? WCoeffs) => throw new NotImplementedException();
    public static void Trimming(double U1, double U2, NCollection_Array1<double> Coeffs, NCollection_Array1<double>? WCoeffs) => throw new NotImplementedException();
    public static void Trimming(double U1, double U2, int dim, NCollection_Array1<double> Coeffs, NCollection_Array1<double>? WCoeffs) => throw new NotImplementedException();
    public static void CoefficientsPoles(NCollection_Array2<gp_Pnt> Coefs, NCollection_Array2<double>? WCoefs, NCollection_Array2<gp_Pnt> Poles, NCollection_Array2<double>? WPoles) => throw new NotImplementedException();
    public static void UTrimming(double U1, double U2, NCollection_Array2<gp_Pnt> Coeffs, NCollection_Array2<double>? WCoeffs) => throw new NotImplementedException();
    public static void VTrimming(double V1, double V2, NCollection_Array2<gp_Pnt> Coeffs, NCollection_Array2<double>? WCoeffs) => throw new NotImplementedException();
    public static bool HermiteInterpolate(int Dimension, double FirstParameter, double LastParameter, int FirstOrder, int LastOrder, NCollection_Array2<double> FirstConstr, NCollection_Array2<double> LastConstr, NCollection_Array1<double> Coefficients) => throw new NotImplementedException();
    public static void JacobiParameters(GeomAbs_Shape ConstraintOrder, int MaxDegree, int Code, out int NbGaussPoints, out int WorkDegree) => throw new NotImplementedException();
    public static int NivConstr(GeomAbs_Shape ConstraintOrder) => throw new NotImplementedException();
    public static GeomAbs_Shape ConstraintOrder(int NivConstr) => throw new NotImplementedException();
    public static void EvalLength(int Degree, int Dimension, ReadOnlySpan<double> PolynomialCoeff, double U1, double U2, out double Length) => throw new NotImplementedException();
    public static void EvalLength(int Degree, int Dimension, ReadOnlySpan<double> PolynomialCoeff, double U1, double U2, double Tol, out double Length, out double Error) => throw new NotImplementedException();
}
