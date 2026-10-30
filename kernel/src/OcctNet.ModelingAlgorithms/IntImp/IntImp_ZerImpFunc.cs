// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImp/IntImp_ZerImpFunc.gxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/IntImp/IntImp_ZerImpFunc.lxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1992-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

namespace Occt;

/// <summary>
/// The function whose zeros are the points of a parametric surface S(u, v) on an implicit surface (a quadric):
/// F(u, v) = Q(S(u, v)), with its gradient, for math_FunctionSetRoot; and, at a root, the tangent of the
/// intersection line in 3D and in the parametric space.
/// </summary>
/// <remarks>
/// OCCT's generic file IntImp_ZerImpFunc.gxx, ported once as a generic class over the parametric surface and its tool
/// (decisions 202, 337 and 607). OCCT's macro TheISurface is IntSurf_Quadric and TheISurfaceTool IntSurf_QuadricTool
/// in every instantiation (IntPatch_TheSurfFunction, GeomInt's and BRepApprox's TheZerImpFunc...), so the body names
/// them. OCCT's instantiations are sealed subclasses with the constructors forwarded.
/// <para>
/// OCCT keeps pointers to the caller's surface handle and quadric. The port keeps the surface (an adaptor, by
/// reference, as the handle) and a copy of the quadric, a struct: every OCCT caller builds the quadric before
/// <see cref="SetImplicitSurface"/> and never changes it afterwards (decision 608).
/// </para>
/// </remarks>
public class IntImp_ZerImpFunc<ThePSurface, ThePSurfaceTool> : math_FunctionSetWithDerivatives
    where ThePSurfaceTool : IIntImp_ZerCSParPSurfaceTool<ThePSurface>
{
    private const double EpsAng = 1.0e-8;
    private const double EpsAng2 = 1.0e-16;
    private const double Tolpetit = 1.0e-16;

    private ThePSurface surf;
    private IntSurf_Quadric func;
    private double u;
    private double v;
    private double tol;
    private gp_Pnt pntsol;
    private double valf;
    private bool computed;
    private bool tangent;
    private double tgdu;
    private double tgdv;
    private gp_Vec gradient;
    private bool derived;
    private gp_Vec d1u;
    private gp_Vec d1v;
    private gp_Vec d3d;
    private gp_Dir2d d2d;

    /// <summary>An empty function; <see cref="Set(ThePSurface)"/> and <see cref="SetImplicitSurface"/> load it.</summary>
    public IntImp_ZerImpFunc()
    {
        surf = default!;
        func = new IntSurf_Quadric();
        u = 0.0;
        v = 0.0;
        tol = 0.0;
        pntsol = new gp_Pnt();
        valf = 0.0;
        computed = false;
        tangent = false;
        tgdu = 0.0;
        tgdv = 0.0;
        gradient = new gp_Vec();
        derived = false;
        d1u = new gp_Vec();
        d1v = new gp_Vec();
        d3d = new gp_Vec();
        d2d = new gp_Dir2d();
    }

    public IntImp_ZerImpFunc(ThePSurface PS, in IntSurf_Quadric IS)
        : this()
    {
        surf = PS;
        func = IS;
    }

    public IntImp_ZerImpFunc(in IntSurf_Quadric IS)
        : this()
    {
        func = IS;
    }

    /// <summary>
    /// A copy of <paramref name="theOther"/>, as OCCT's implicit copy constructor: the same surface (a reference) and
    /// a copy of the quadric, with the state of the last evaluation (IntWalk_IWalking's duplicate of its function).
    /// </summary>
    protected IntImp_ZerImpFunc(IntImp_ZerImpFunc<ThePSurface, ThePSurfaceTool> theOther)
    {
        surf = theOther.surf;
        func = theOther.func;
        u = theOther.u;
        v = theOther.v;
        tol = theOther.tol;
        pntsol = theOther.pntsol;
        valf = theOther.valf;
        computed = theOther.computed;
        tangent = theOther.tangent;
        tgdu = theOther.tgdu;
        tgdv = theOther.tgdv;
        gradient = theOther.gradient;
        derived = theOther.derived;
        d1u = theOther.d1u;
        d1v = theOther.d1v;
        d3d = theOther.d3d;
        d2d = theOther.d2d;
    }

    public void Set(ThePSurface PS) => surf = PS;

    public void SetImplicitSurface(in IntSurf_Quadric IS) => func = IS;

    public void Set(double Tol) => tol = Tol;

    public override int NbVariables() => 2;

    public override int NbEquations() => 1;

    public override bool Value(math_Vector X, math_Vector F)
    {
        u = X[1];
        v = X[2];
        pntsol = ThePSurfaceTool.Value(surf, u, v);
        valf = IntSurf_QuadricTool.Value(func, pntsol.X(), pntsol.Y(), pntsol.Z());
        F[1] = valf;
        computed = false;
        derived = false;
        return true;
    }

    public override bool Derivatives(math_Vector X, math_Matrix D)
    {
        u = X[1];
        v = X[2];
        ThePSurfaceTool.D1(surf, u, v, out pntsol, out d1u, out d1v);
        IntSurf_QuadricTool.Gradient(func, pntsol.X(), pntsol.Y(), pntsol.Z(), out gradient);
        D[1, 1] = d1u.Dot(gradient);
        D[1, 2] = d1v.Dot(gradient);
        computed = false;
        derived = true;
        return true;
    }

    public override bool Values(math_Vector X, math_Vector F, math_Matrix D)
    {
        u = X[1];
        v = X[2];
        ThePSurfaceTool.D1(surf, u, v, out pntsol, out d1u, out d1v);
        IntSurf_QuadricTool.ValueAndGradient(func, pntsol.X(), pntsol.Y(), pntsol.Z(), ref valf, ref gradient);
        F[1] = valf;
        D[1, 1] = d1u.Dot(gradient);
        D[1, 2] = d1v.Dot(gradient);
        computed = false;
        derived = true;
        return true;
    }

    /// <summary>The value of the last evaluation.</summary>
    public double Root() => valf;

    /// <summary>The value Tol such that the function is null where |Root()| &lt; Tol.</summary>
    public double Tolerance() => tol;

    /// <summary>The point of the last evaluation.</summary>
    public gp_Pnt Point() => pntsol;

    /// <summary>
    /// True if the surfaces are tangent at the last evaluated point; otherwise computes the tangent of the
    /// intersection (<see cref="Direction3d"/>, <see cref="Direction2d"/>).
    /// </summary>
    public bool IsTangent()
    {
        if (!computed)
        {
            computed = true;
            if (!derived)
            {
                ThePSurfaceTool.D1(surf, u, v, out pntsol, out d1u, out d1v);
                derived = true;
            }

            tgdu = gradient.Dot(d1v);
            tgdv = -gradient.Dot(d1u);
            double N2grad = gradient.SquareMagnitude();
            double N2grad_EpsAng2 = N2grad * EpsAng2;
            double N2d1u = d1u.SquareMagnitude();
            double N2d1v = d1v.SquareMagnitude();
            tangent = (tgdu * tgdu <= N2grad_EpsAng2 * N2d1v) && (tgdv * tgdv <= N2grad_EpsAng2 * N2d1u);
            if (!tangent)
            {
                d3d.SetLinearForm(tgdu, d1u, tgdv, d1v);
                d2d = new gp_Dir2d(tgdu, tgdv);
                if (d3d.Magnitude() <= Tolpetit)
                { // jag
                    tangent = true;
                }
            }
        }
        return tangent;
    }

    /// <summary>The tangent of the intersection in 3D. Throws StdFail_UndefinedDerivative at a tangent point.</summary>
    public gp_Vec Direction3d()
    {
        if (IsTangent())
        {
            throw new StdFail_UndefinedDerivative();
        }
        return d3d;
    }

    /// <summary>The tangent in the parametric space. Throws StdFail_UndefinedDerivative at a tangent point.</summary>
    public gp_Dir2d Direction2d()
    {
        if (IsTangent())
        {
            throw new StdFail_UndefinedDerivative();
        }
        return d2d;
    }

    /// <summary>The parametric surface.</summary>
    public ThePSurface PSurface() => surf;

    /// <summary>The implicit surface (the function's copy).</summary>
    public IntSurf_Quadric ISurface() => func;
}
