// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GlobOptFuncCS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_GlobOptFuncCS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 2014-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The square distance between a curve and a surface,
/// F(cu, su, sv) = |C(cu) - S(su, sv)|^2, with its gradient and hessian (each halved), for global optimisation.
/// </summary>
/// <remarks>
/// The curve and the surface are kept by reference and must outlive the function, as in OCCT. Outside the parameter
/// ranges the functions return false and write 0 to their <c>out</c> value (decision 221).
/// </remarks>
public class Extrema_GlobOptFuncCS : math_MultipleVarFunctionWithHessian
{
    private readonly Adaptor3d_Curve myC;
    private readonly Adaptor3d_Surface myS;

    // F(cu, su, sv)=(C^{(x)}(cu)-S^{(x)}(su,sv))^{2}+
    //               (C^{(y)}(cu)-S^{(y)}(su,sv))^{2}+
    //               (C^{(z)}(cu)-S^{(z)}(su,sv))^{2}

    private void value(double cu, double su, double sv, out double F)
    {
        F = myC.Value(cu).SquareDistance(myS.Value(su, sv));
    }

    private void gradient(double cu, double su, double sv, math_Vector G)
    {
        myC.D1(cu, out gp_Pnt CD0, out gp_Vec CD1);
        myS.D1(su, sv, out gp_Pnt SD0, out gp_Vec SD1U, out gp_Vec SD1V);

        G[1] = +(CD0.X() - SD0.X()) * CD1.X() + (CD0.Y() - SD0.Y()) * CD1.Y() + (CD0.Z() - SD0.Z()) * CD1.Z();
        G[2] = -(CD0.X() - SD0.X()) * SD1U.X() - (CD0.Y() - SD0.Y()) * SD1U.Y()
               - (CD0.Z() - SD0.Z()) * SD1U.Z();
        G[3] = -(CD0.X() - SD0.X()) * SD1V.X() - (CD0.Y() - SD0.Y()) * SD1V.Y()
               - (CD0.Z() - SD0.Z()) * SD1V.Z();
    }

    private void hessian(double cu, double su, double sv, math_Matrix H)
    {
        myC.D2(cu, out gp_Pnt CD0, out gp_Vec CD1, out gp_Vec CD2);
        myS.D2(su, sv, out gp_Pnt SD0, out gp_Vec SD1U, out gp_Vec SD1V, out gp_Vec SD2UU, out gp_Vec SD2VV,
            out gp_Vec SD2UV);

        H[1, 1] = +CD1.X() * CD1.X() + CD1.Y() * CD1.Y() + CD1.Z() * CD1.Z()
                  + (CD0.X() - SD0.X()) * CD2.X() + (CD0.Y() - SD0.Y()) * CD2.Y()
                  + (CD0.Z() - SD0.Z()) * CD2.Z();

        H[1, 2] = -CD1.X() * SD1U.X() - CD1.Y() * SD1U.Y() - CD1.Z() * SD1U.Z();

        H[1, 3] = -CD1.X() * SD1V.X() - CD1.Y() * SD1V.Y() - CD1.Z() * SD1V.Z();

        H[2, 1] = H[1, 2];

        H[2, 2] = +SD1U.X() * SD1U.X() + SD1U.Y() * SD1U.Y() + SD1U.Z() * SD1U.Z()
                  - (CD0.X() - SD0.X()) * SD2UU.X() - (CD0.Y() - SD0.Y()) * SD2UU.Y()
                  - (CD0.Z() - SD0.Z()) * SD2UU.Z();

        H[2, 3] = +SD1U.X() * SD1V.X() + SD1U.Y() * SD1V.Y() + SD1U.Z() * SD1V.Z()
                  - (CD0.X() - SD0.X()) * SD2UV.X() - (CD0.Y() - SD0.Y()) * SD2UV.Y()
                  - (CD0.Z() - SD0.Z()) * SD2UV.Z();

        H[3, 1] = H[1, 3];

        H[3, 2] = H[2, 3];

        H[3, 3] = +SD1V.X() * SD1V.X() + SD1V.Y() * SD1V.Y() + SD1V.Z() * SD1V.Z()
                  - (CD0.X() - SD0.X()) * SD2VV.X() - (CD0.Y() - SD0.Y()) * SD2VV.Y()
                  - (CD0.Z() - SD0.Z()) * SD2VV.Z();
    }

    private bool checkInputData(math_Vector X, out double cu, out double su, out double sv)
    {
        int aStartIndex = X.Lower();
        cu = X[aStartIndex];
        su = X[aStartIndex + 1];
        sv = X[aStartIndex + 2];

        return cu >= myC.FirstParameter() && cu <= myC.LastParameter() && su >= myS.FirstUParameter()
               && su <= myS.LastUParameter() && sv >= myS.FirstVParameter()
               && sv <= myS.LastVParameter();
    }

    /// <summary>The function of a curve and a surface, which must outlive it.</summary>
    public Extrema_GlobOptFuncCS(Adaptor3d_Curve C, Adaptor3d_Surface S)
    {
        myC = C;
        myS = S;
    }

    public override int NbVariables() => 3;

    public override bool Value(math_Vector X, out double F)
    {
        if (!checkInputData(X, out double cu, out double su, out double sv))
        {
            F = 0.0; // OCCT leaves F unchanged (decision 221)
            return false;
        }

        value(cu, su, sv, out F);
        return true;
    }

    public override bool Gradient(math_Vector X, math_Vector G)
    {
        if (!checkInputData(X, out double cu, out double su, out double sv))
        {
            return false;
        }

        gradient(cu, su, sv, G);
        return true;
    }

    public override bool Values(math_Vector X, out double F, math_Vector G)
    {
        if (!checkInputData(X, out double cu, out double su, out double sv))
        {
            F = 0.0; // OCCT leaves F unchanged (decision 221)
            return false;
        }

        value(cu, su, sv, out F);
        gradient(cu, su, sv, G);
        return true;
    }

    public override bool Values(math_Vector X, out double F, math_Vector G, math_Matrix H)
    {
        if (!checkInputData(X, out double cu, out double su, out double sv))
        {
            F = 0.0; // OCCT leaves F unchanged (decision 221)
            return false;
        }

        value(cu, su, sv, out F);
        gradient(cu, su, sv, G);
        hessian(cu, su, sv, H);
        return true;
    }
}
