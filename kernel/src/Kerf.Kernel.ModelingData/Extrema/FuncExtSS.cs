// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_FuncExtSS.hxx
// Port of: src/ModelingData/TKGeomBase/Extrema/Extrema_FuncExtSS.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Extrema;

/// <summary>
/// The system whose roots are the extrema of the distance between two surfaces S1 and S2:
/// (S1(u1, v1) - S2(u2, v2)) dotted with S1u, S1v, S2u and S2v, with its derivatives, for math_FunctionSetRoot.
/// Every root found (GetStateNumber) is stored with its square distance.
/// </summary>
/// <remarks>
/// OCCT keeps pointers to the surfaces; the port keeps the adaptors, which must outlive the function.
/// </remarks>
public class FuncExtSS : FunctionSetWithDerivatives
{
    private SurfaceAdaptorBase? _s1;
    private SurfaceAdaptorBase? _s2;
    private Point3D _p1 = new();
    private Point3D _p2 = new();
    private double _u1;
    private double _v1;
    private double _u2;
    private double _v2;
    private readonly NCollection_Sequence<double> _sqDist = new();
    private readonly NCollection_Sequence<POnSurf> _point1 = new();
    private readonly NCollection_Sequence<POnSurf> _point2 = new();
    private bool _s1init;
    private bool _s2init;

    public FuncExtSS()
    {
        _s1 = null;
        _s2 = null;
        _u1 = 0.0;
        _v1 = 0.0;
        _u2 = 0.0;
        _v2 = 0.0;
        _s1init = false;
        _s2init = false;
    }

    public FuncExtSS(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2)
    {
        _u1 = 0.0;
        _v1 = 0.0;
        _u2 = 0.0;
        _v2 = 0.0;
        _s1 = s1;
        _s2 = s2;
        _s1init = true;
        _s2init = true;
    }

    /// <summary>Sets the surfaces, and forgets the extrema found.</summary>
    public void Initialize(SurfaceAdaptorBase s1, SurfaceAdaptorBase s2)
    {
        _s1 = s1;
        _s2 = s2;
        _s1init = true;
        _s2init = true;
        _point1.Clear();
        _point2.Clear();
        _sqDist.Clear();
    }

    public override int NbVariables() => 4;

    public override int NbEquations() => 4;

    /// <summary>Calculates Fi(U1, V1, U2, V2).</summary>
    public override bool Value(math_Vector uv, math_Vector f)
    {
        if (!_s1init || !_s2init)
        {
            throw new TypeMismatchException();
        }
        _u1 = uv[1];
        _v1 = uv[2];
        _u2 = uv[3];
        _v2 = uv[4];
        _s1!.D1(_u1, _v1, out _p1, out Vector3D du1s1, out Vector3D dv1s1);
        _s2!.D1(_u2, _v2, out _p2, out Vector3D du2s2, out Vector3D dv2s2);

        Vector3D p1P2 = new(_p2, _p1);

        f[1] = p1P2.Dot(du1s1);
        f[2] = p1P2.Dot(dv1s1);
        f[3] = p1P2.Dot(du2s2);
        f[4] = p1P2.Dot(dv2s2);

        return true;
    }

    /// <summary>Calculates Fi'(U1, V1, U2, V2).</summary>
    public override bool Derivatives(math_Vector uv, Matrix df)
    {
        math_Vector f = new(1, 4);
        return Values(uv, f, df);
    }

    /// <summary>Calculates Fi(U1, V1, U2, V2) and Fi'(U1, V1, U2, V2).</summary>
    public override bool Values(math_Vector uv, math_Vector f, Matrix df)
    {
        if (!_s1init || !_s2init)
        {
            throw new TypeMismatchException();
        }
        _u1 = uv[1];
        _v1 = uv[2];
        _u2 = uv[3];
        _v2 = uv[4];
        _s1!.D2(_u1, _v1, out _p1, out Vector3D du1s1, out Vector3D dv1s1, out Vector3D du1u1s1, out Vector3D dv1v1s1,
            out Vector3D du1v1s1);
        _s2!.D2(_u2, _v2, out _p2, out Vector3D du2s2, out Vector3D dv2s2, out Vector3D du2u2s2, out Vector3D dv2v2s2,
            out Vector3D du2v2s2);

        Vector3D p1P2 = new(_p2, _p1);

        f[1] = p1P2.Dot(du1s1);
        f[2] = p1P2.Dot(dv1s1);
        f[3] = p1P2.Dot(du2s2);
        f[4] = p1P2.Dot(dv2s2);

        df[1, 1] = du1s1.SquareMagnitude() + p1P2.Dot(du1u1s1);
        df[1, 2] = dv1s1.Dot(du1s1) + p1P2.Dot(du1v1s1);
        df[1, 3] = -du2s2.Dot(du1s1);
        df[1, 4] = -dv2s2.Dot(du1s1);

        df[2, 1] = df[1, 2]; // Du1s1.Dot(Dv1s1) + P1P2.Dot(Du1v1s1);
        df[2, 2] = dv1s1.SquareMagnitude() + p1P2.Dot(dv1v1s1);
        df[2, 3] = -du2s2.Dot(dv1s1);
        df[2, 4] = -dv2s2.Dot(dv1s1);

        df[3, 1] = -df[1, 3]; // Du1s1.Dot(Du2s2);
        df[3, 2] = -df[2, 3]; // Dv1s1.Dot(Du2s2);
        df[3, 3] = -du2s2.SquareMagnitude() + p1P2.Dot(du2u2s2);
        df[3, 4] = -dv2s2.Dot(du2s2) + p1P2.Dot(du2v2s2);

        df[4, 1] = -df[1, 4]; // Du1s1.Dot(Dv2s2);
        df[4, 2] = -df[2, 4]; // Dv1s1.Dot(Dv2s2);
        df[4, 3] = df[3, 4]; // -Du2s2.Dot(Dv2s2) + P1P2.Dot(Du2v2s2);
        df[4, 4] = -dv2s2.SquareMagnitude() + p1P2.Dot(dv2v2s2);

        return true;
    }

    /// <summary>Saves the extremum found.</summary>
    public override int GetStateNumber()
    {
        if (!_s1init || !_s2init)
        {
            throw new TypeMismatchException();
        }

        _sqDist.Append(_p1.SquareDistance(_p2));
        _point1.Append(new POnSurf(_u1, _v1, _p1));
        _point2.Append(new POnSurf(_u2, _v2, _p2));
        return 0;
    }

    /// <summary>The number of extrema found.</summary>
    public int ExtCount => _sqDist.Length();

    /// <summary>The square distance of the Nth extremum.</summary>
    public double SquareDistance(int n)
    {
        if (!_s1init || !_s2init)
        {
            throw new TypeMismatchException();
        }
        return _sqDist.Value(n);
    }

    /// <summary>The Nth extremum on S1.</summary>
    public POnSurf PointOnS1(int n)
    {
        if (!_s1init || !_s2init)
        {
            throw new TypeMismatchException();
        }
        return _point1.Value(n);
    }

    /// <summary>The Nth extremum on S2.</summary>
    public POnSurf PointOnS2(int n)
    {
        if (!_s1init || !_s2init)
        {
            throw new TypeMismatchException();
        }
        return _point2.Value(n);
    }
}
