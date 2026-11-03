// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_SearchSing.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFi3d/ChFi3d_SearchSing.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1997-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Numerics;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Fillets;

/// <summary>
/// The function whose roots are the parameters where the isos of two curves meet: F(t) = (C2(t) - C1(t)) · (C2'(t) -
/// C1'(t)), with its derivative. ChFi3d_FilBuilder's <c>SplitSurf</c> searches the singular sections of a fillet with it.
/// </summary>
internal sealed class SearchSing : FunctionWithDerivative
{
    private readonly Curve3D _c1;
    private readonly Curve3D _c2;

    public SearchSing(Curve3D c1, Curve3D c2)
    {
        _c1 = c1;
        _c2 = c2;
    }

    public override bool Value(double x, out double f)
    {
        _c1.D1(x, out Point3D p1, out Vector3D v1);
        _c2.D1(x, out Point3D p2, out Vector3D v2);
        Vector3D v = new(p1, p2);
        f = v * (v2 - v1);
        return true;
    }

    public override bool Derivative(double x, out double d)
    {
        _c1.D2(x, out Point3D p1, out Vector3D v1, out Vector3D w1);
        _c2.D2(x, out Point3D p2, out Vector3D v2, out Vector3D w2);
        Vector3D v = new(p1, p2), vPrim;
        vPrim = v2 - v1;
        d = vPrim.SquareMagnitude + (v * (w2 - w1));
        return true;
    }

    public override bool Values(double x, out double f, out double d)
    {
        _c1.D2(x, out Point3D p1, out Vector3D v1, out Vector3D w1);
        _c2.D2(x, out Point3D p2, out Vector3D v2, out Vector3D w2);
        Vector3D v = new(p1, p2), vPrim;
        vPrim = v2 - v1;
        f = v * vPrim;
        d = vPrim.SquareMagnitude + (v * (w2 - w1));
        return true;
    }
}
