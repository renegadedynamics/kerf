// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_SurfFunction.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_SurfFunction.lxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_SurfFunction.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using static Occt.Contap_TFunction;

namespace Occt;

/// <summary>
/// The contour function on a surface: F(u, v) = N(u, v) . D / mean, the normal's component along the view direction
/// (or towards the eye, or less the draft angle's cosine), scaled by the mean normal length; its zeros are the
/// contour. The function of the contour's walking and interior search.
/// </summary>
/// <remarks>
/// A working object. It implements <see cref="IIntWalk_IWFunction{TSelf}"/> (the walking's function) and so
/// <see cref="IIntStart_SurfFunction{ThePSurface}"/>. With <see cref="Contap_DraftPrs"/> OCCT computes nothing and
/// leaves the outputs as they were, as the port does.
/// </remarks>
public sealed class Contap_SurfFunction : math_FunctionSetWithDerivatives, IIntWalk_IWFunction<Contap_SurfFunction>
{
    private Adaptor3d_Surface? mySurf;
    private double myMean;
    private Contap_TFunction myType;
    private gp_Dir myDir;
    private gp_Pnt myEye;
    private double myAng;
    private double myCosAng;
    private double tol;
    private gp_Pnt solpt;
    private double valf;
    private double Usol;
    private double Vsol;
    private double Fpu;
    private double Fpv;
    private gp_Dir2d d2d;
    private gp_Vec d3d;
    private bool tangent;
    private bool computed;
    private bool derived;

    public Contap_SurfFunction()
    {
        myMean = 1.0;
        myType = Contap_ContourStd;
        myDir = new gp_Dir(gp_Dir.D.Z);
        myEye = new gp_Pnt();
        myAng = 0.0;
        myCosAng = 0.0; // PI/2 - Angle de depouille
        tol = 1.0e-6;
        solpt = new gp_Pnt();
        valf = 0.0;
        Usol = 0.0;
        Vsol = 0.0;
        Fpu = 0.0;
        Fpv = 0.0;
        d2d = new gp_Dir2d();
        d3d = new gp_Vec();
        tangent = false;
        computed = false;
        derived = false;
    }

    /// <summary>A copy of <paramref name="theOther"/> (OCCT's implicit copy constructor; decision 686).</summary>
    public Contap_SurfFunction(Contap_SurfFunction theOther)
    {
        mySurf = theOther.mySurf;
        myMean = theOther.myMean;
        myType = theOther.myType;
        myDir = theOther.myDir;
        myEye = theOther.myEye;
        myAng = theOther.myAng;
        myCosAng = theOther.myCosAng;
        tol = theOther.tol;
        solpt = theOther.solpt;
        valf = theOther.valf;
        Usol = theOther.Usol;
        Vsol = theOther.Vsol;
        Fpu = theOther.Fpu;
        Fpv = theOther.Fpv;
        d2d = theOther.d2d;
        d3d = theOther.d3d;
        tangent = theOther.tangent;
        computed = theOther.computed;
        derived = theOther.derived;
    }

    static Contap_SurfFunction IIntWalk_IWFunction<Contap_SurfFunction>.Copy(Contap_SurfFunction theOther) =>
        new(theOther);

    /// <summary>Sets the surface, and the mean length of its normal at its sample points.</summary>
    public void Set(Adaptor3d_Surface S)
    {
        mySurf = S;
        int i;
        int nbs = Contap_HContTool.NbSamplePoints(S);
        double U, V;
        gp_Vec norm;
        if (nbs > 0)
        {
            myMean = 0.0;
            for (i = 1; i <= nbs; i++)
            {
                Contap_HContTool.SamplePoint(S, i, out U, out V);
                //      Adaptor3d_HSurfaceTool::D1(S,U,V,solpt,d1u,d1v);
                //      myMean = myMean + d1u.Crossed(d1v).Magnitude();
                Contap_SurfProps.Normale(S, U, V, out solpt, out norm);
                myMean = myMean + norm.Magnitude();
            }
            myMean = myMean / ((double)nbs);
        }
        computed = false;
        derived = false;
    }

    /// <summary>A silhouette for the eye <paramref name="Eye"/>.</summary>
    public void Set(in gp_Pnt Eye)
    {
        myType = Contap_ContourPrs; // pers
        myEye = Eye;
        myAng = 0.0;
    }

    /// <summary>A silhouette for the view direction <paramref name="Direction"/>.</summary>
    public void Set(in gp_Dir Direction)
    {
        myType = Contap_ContourStd; // Contour app
        myDir = Direction;
        myAng = 0.0;
    }

    /// <summary>A draft contour of angle <paramref name="Angle"/> for the direction <paramref name="Direction"/>.</summary>
    public void Set(in gp_Dir Direction, double Angle)
    {
        myType = Contap_DraftStd; // Contour vu
        myDir = Direction;
        myAng = Angle;
        myCosAng = Math.Cos(M_PI / 2.0 + Angle);
    }

    /// <summary>A draft contour of angle <paramref name="Angle"/> for the eye <paramref name="Eye"/>.</summary>
    public void Set(in gp_Pnt Eye, double Angle)
    {
        myType = Contap_DraftPrs; // Contour vu "conique"...
        myEye = Eye;
        myAng = Angle;
        myCosAng = Math.Cos(M_PI / 2.0 + Angle);
    }

    /// <summary>Sets the tolerance (at least 1e-12).</summary>
    public void Set(double Tolerance)
    {
        tol = Max(Tolerance, 1.0e-12);
    }

    public override int NbVariables() => 2;

    public override int NbEquations() => 1;

    public override bool Value(math_Vector X, math_Vector F)
    {
        Usol = X[1];
        Vsol = X[2];
        //  Adaptor3d_HSurfaceTool::D1(mySurf,Usol,Vsol,solpt,d1u,d1v);
        //  gp_Vec norm(d1u.Crossed(d1v));
        Contap_SurfProps.Normale(mySurf!, Usol, Vsol, out solpt, out gp_Vec norm);
        switch (myType)
        {
            case Contap_ContourStd:
                F[1] = valf = (norm.Dot(new gp_Vec(myDir))) / myMean;
                break;
            case Contap_ContourPrs:
                F[1] = valf = (norm.Dot(new gp_Vec(myEye, solpt))) / myMean;
                break;
            case Contap_DraftStd:
                F[1] = valf = (norm.Dot(new gp_Vec(myDir)) - myCosAng * norm.Magnitude()) / myMean;
                break;
            default:
                break;
        }
        computed = false;
        derived = false;
        return true;
    }

    public override bool Derivatives(math_Vector X, math_Matrix Grad)
    {
        //  gp_Vec d2u,d2v,d2uv;
        Usol = X[1];
        Vsol = X[2];
        //  Adaptor3d_HSurfaceTool::D2(mySurf,Usol,Vsol,solpt,d1u,d1v,d2u,d2v,d2uv);

        Contap_SurfProps.NormAndDn(mySurf!, Usol, Vsol, out solpt, out gp_Vec norm, out gp_Vec dnu, out gp_Vec dnv);

        switch (myType)
        {
            case Contap_ContourStd:
                Grad[1, 1] = (dnu.Dot(new gp_Vec(myDir))) / myMean;
                Grad[1, 2] = (dnv.Dot(new gp_Vec(myDir))) / myMean;
                break;
            case Contap_ContourPrs:
            {
                gp_Vec Ep = new(myEye, solpt);
                Grad[1, 1] = (dnu.Dot(Ep)) / myMean;
                Grad[1, 2] = (dnv.Dot(Ep)) / myMean;
            }
            break;
            case Contap_DraftStd:
                norm.Normalize();
                Grad[1, 1] = (dnu.Dot(new gp_Vec(myDir)) - myCosAng * dnu.Dot(norm)) / myMean;
                Grad[1, 2] = (dnv.Dot(new gp_Vec(myDir)) - myCosAng * dnv.Dot(norm)) / myMean;
                break;
            case Contap_DraftPrs:
            default:
                break;
        }
        Fpu = Grad[1, 1];
        Fpv = Grad[1, 2];
        computed = false;
        derived = true;
        return true;
    }

    public override bool Values(math_Vector X, math_Vector F, math_Matrix Grad)
    {
        //  gp_Vec d2u,d2v,d2uv;

        Usol = X[1];
        Vsol = X[2];
        //  Adaptor3d_HSurfaceTool::D2(mySurf,Usol,Vsol,solpt,d1u,d1v,d2u,d2v,d2uv);
        //  gp_Vec norm(d1u.Crossed(d1v));
        Contap_SurfProps.NormAndDn(mySurf!, Usol, Vsol, out solpt, out gp_Vec norm, out gp_Vec dnu, out gp_Vec dnv);

        switch (myType)
        {
            case Contap_ContourStd:
                F[1] = (norm.Dot(new gp_Vec(myDir))) / myMean;
                Grad[1, 1] = (dnu.Dot(new gp_Vec(myDir))) / myMean;
                Grad[1, 2] = (dnv.Dot(new gp_Vec(myDir))) / myMean;
                break;
            case Contap_ContourPrs:
            {
                gp_Vec Ep = new(myEye, solpt);
                F[1] = (norm.Dot(Ep)) / myMean;
                Grad[1, 1] = (dnu.Dot(Ep)) / myMean;
                Grad[1, 2] = (dnv.Dot(Ep)) / myMean;
            }
            break;
            case Contap_DraftStd:
                F[1] = (norm.Dot(new gp_Vec(myDir)) - myCosAng * norm.Magnitude()) / myMean;
                norm.Normalize();
                Grad[1, 1] = (dnu.Dot(new gp_Vec(myDir)) - myCosAng * dnu.Dot(norm)) / myMean;
                Grad[1, 2] = (dnv.Dot(new gp_Vec(myDir)) - myCosAng * dnv.Dot(norm)) / myMean;
                break;
            case Contap_DraftPrs:
            default:
                break;
        }
        valf = F[1];
        Fpu = Grad[1, 1];
        Fpv = Grad[1, 2];
        computed = false;
        derived = true;
        return true;
    }

    /// <summary>The value of the function at the last evaluated point.</summary>
    public double Root() => valf;

    /// <summary>The tolerance below which the function is null.</summary>
    public double Tolerance() => tol;

    /// <summary>The last evaluated point.</summary>
    public gp_Pnt Point() => solpt;

    /// <summary>
    /// True if the contour is singular (the gradient vanishes) at the last evaluated point; otherwise computes its
    /// tangent (<see cref="Direction3d"/>, <see cref="Direction2d"/>).
    /// </summary>
    public bool IsTangent()
    {
        if (!computed)
        {
            computed = true;
            if (!derived)
            {
                //      gp_Vec d2u,d2v,d2uv;
                //      Adaptor3d_HSurfaceTool::D2(mySurf, Usol, Vsol, solpt, d1u, d1v, d2u, d2v, d2uv);
                Contap_SurfProps.NormAndDn(mySurf!, Usol, Vsol, out solpt, out gp_Vec norm, out gp_Vec dnu,
                                           out gp_Vec dnv);

                switch (myType)
                {
                    case Contap_ContourStd:
                        Fpu = (dnu.Dot(new gp_Vec(myDir))) / myMean;
                        Fpv = (dnv.Dot(new gp_Vec(myDir))) / myMean;
                        break;
                    case Contap_ContourPrs:
                    {
                        gp_Vec Ep = new(myEye, solpt);
                        Fpu = (dnu.Dot(Ep)) / myMean;
                        Fpv = (dnv.Dot(Ep)) / myMean;
                    }
                    break;
                    case Contap_DraftStd:
                        norm.Normalize();
                        Fpu = (dnu.Dot(new gp_Vec(myDir)) - myCosAng * dnu.Dot(norm)) / myMean;
                        Fpv = (dnv.Dot(new gp_Vec(myDir)) - myCosAng * dnv.Dot(norm)) / myMean;
                        break;
                    case Contap_DraftPrs:
                    default:
                        break;
                }
                derived = true;
            }
            tangent = false;
            double D = Math.Sqrt(Fpu * Fpu + Fpv * Fpv);

            if (D <= gp.Resolution())
            {
                tangent = true;
            }
            else
            {
                d2d = new gp_Dir2d(-Fpv, Fpu);
                Adaptor3d_HSurfaceTool.D1(mySurf!, Usol, Vsol, out solpt, out gp_Vec d1u, out gp_Vec d1v); // ajout jag 02.95

                gp_XYZ d3dxyz = -Fpv * d1u.XYZ();
                d3dxyz.Add(Fpu * d1v.XYZ());
                d3d.SetXYZ(d3dxyz);

                // jag 940616    if (d3d.Magnitude() <= Tolpetit) {
                if (d3d.Magnitude() <= tol)
                {
                    tangent = true;
                }
            }
        }
        return tangent;
    }

    /// <summary>The tangent of the contour in 3D. Throws StdFail_UndefinedDerivative at a singular point.</summary>
    public gp_Vec Direction3d()
    {
        if (IsTangent())
        {
            throw new StdFail_UndefinedDerivative();
        }
        return d3d;
    }

    /// <summary>The tangent in the parametric space. Throws StdFail_UndefinedDerivative at a singular point.</summary>
    public gp_Dir2d Direction2d()
    {
        if (IsTangent())
        {
            throw new StdFail_UndefinedDerivative();
        }
        return d2d;
    }

    public Contap_TFunction FunctionType() => myType;

    public gp_Pnt Eye() => myEye;

    public gp_Dir Direction() => myDir;

    public double Angle() => myAng;

    public Adaptor3d_Surface Surface() => mySurf!;

    /// <summary>The surface (the walking's name for it).</summary>
    public Adaptor3d_Surface PSurface() => Surface();
}
