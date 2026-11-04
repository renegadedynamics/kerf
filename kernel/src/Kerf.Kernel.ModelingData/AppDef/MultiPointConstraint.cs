// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_MultiPointConstraint.hxx
// Port of: src/ModelingData/TKGeomBase/AppDef/AppDef_MultiPointConstraint.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

using Kerf.Geometry;
using Occt;

namespace Kerf.Engine.Approximation;

/// <summary>
/// A MultiPoint of a MultiLine with optional tangency and curvature constraints: several 2d or 3d points (the 3d
/// points first) that share one constraint and one parameter, approximated in parallel. A tangency point is also a
/// passing point.
/// </summary>
/// <remarks>
/// A class, as its base (decisions 141 and 201): a C++ copy shares the point, tangent and curvature arrays, and so
/// does a C# reference. The setters stay public: callers build MultiLines from these.
/// </remarks>
internal class MultiPointConstraint : MultiPoint
{
    private NCollection_HArray1<Vector3D>? _tabTang;
    private NCollection_HArray1<Vector3D>? _tabCurv;
    private NCollection_HArray1<Vector2D>? _tabTang2d;
    private NCollection_HArray1<Vector2D>? _tabCurv2d;

    /// <summary>An undefined MultiPointConstraint.</summary>
    public MultiPointConstraint() { }

    /// <summary>
    /// <paramref name="nbPoles"/> 3d points and <paramref name="nbPoles2d"/> 2d points, to be set with
    /// <see cref="MultiPoint.SetPoint"/> and <see cref="MultiPoint.SetPoint2d"/>.
    /// </summary>
    public MultiPointConstraint(int nbPoles, int nbPoles2d) : base(nbPoles, nbPoles2d) { }

    /// <summary>A MultiPoint of 3d points only.</summary>
    public MultiPointConstraint(NCollection_Array1<Point3D> tabP) : base(tabP) { }

    /// <summary>A MultiPoint of 2d points only.</summary>
    public MultiPointConstraint(NCollection_Array1<Point2D> tabP2d) : base(tabP2d) { }

    /// <summary>A MultiPoint of 3d and 2d points.</summary>
    public MultiPointConstraint(NCollection_Array1<Point3D> tabP, NCollection_Array1<Point2D> tabP2d)
        : base(tabP, tabP2d) { }

    /// <summary>
    /// A MultiPoint with tangency and curvature constraints. Throws Standard_ConstructionError unless there are as
    /// many tangents and curvatures as points, in each dimension.
    /// </summary>
    public MultiPointConstraint(NCollection_Array1<Point3D> tabP, NCollection_Array1<Point2D> tabP2d,
                                       NCollection_Array1<Vector3D> tabVec, NCollection_Array1<Vector2D> tabVec2d,
                                       NCollection_Array1<Vector3D> tabCur, NCollection_Array1<Vector2D> tabCur2d)
        : base(tabP, tabP2d)
    {
        if ((tabP.Length() != tabVec.Length()) || (tabP2d.Length() != tabVec2d.Length())
            || (tabCur.Length() != tabP.Length()) || (tabCur2d.Length() != tabP2d.Length()))
        {
            throw new ConstructionErrorException();
        }

        _tabTang = new NCollection_HArray1<Vector3D>(1, tabVec.Length());
        _tabTang2d = new NCollection_HArray1<Vector2D>(1, tabVec2d.Length());

        int i, lower = tabVec.Lower();
        for (i = 1; i <= tabVec.Length(); i++)
        {
            _tabTang.SetValue(i, tabVec.Value(lower + i - 1));
        }
        lower = tabVec2d.Lower();
        for (i = 1; i <= tabVec2d.Length(); i++)
        {
            _tabTang2d.SetValue(i, tabVec2d.Value(lower + i - 1));
        }

        _tabCurv = new NCollection_HArray1<Vector3D>(1, tabCur.Length());
        _tabCurv2d = new NCollection_HArray1<Vector2D>(1, tabCur2d.Length());

        lower = tabCur.Lower();
        for (i = 1; i <= tabVec.Length(); i++)
        {
            _tabCurv.SetValue(i, tabCur.Value(lower + i - 1));
        }
        lower = tabCur2d.Lower();
        for (i = 1; i <= tabCur2d.Length(); i++)
        {
            _tabCurv2d.SetValue(i, tabCur2d.Value(lower + i - 1));
        }
    }

    /// <summary>
    /// A MultiPoint with tangency constraints. Throws Standard_ConstructionError unless there are as many
    /// tangents as points, in each dimension.
    /// </summary>
    public MultiPointConstraint(NCollection_Array1<Point3D> tabP, NCollection_Array1<Point2D> tabP2d,
                                       NCollection_Array1<Vector3D> tabVec, NCollection_Array1<Vector2D> tabVec2d)
        : base(tabP, tabP2d)
    {
        if ((tabP.Length() != tabVec.Length()) || (tabP2d.Length() != tabVec2d.Length()))
        {
            throw new ConstructionErrorException();
        }

        _tabTang = new NCollection_HArray1<Vector3D>(1, tabVec.Length());
        _tabTang2d = new NCollection_HArray1<Vector2D>(1, tabVec2d.Length());

        int i, lower = tabVec.Lower();
        for (i = 1; i <= tabVec.Length(); i++)
        {
            _tabTang.SetValue(i, tabVec.Value(lower + i - 1));
        }
        lower = tabVec2d.Lower();
        for (i = 1; i <= tabVec2d.Length(); i++)
        {
            _tabTang2d.SetValue(i, tabVec2d.Value(lower + i - 1));
        }
    }

    /// <summary>
    /// A MultiPoint of 3d points with tangency constraints. Throws Standard_ConstructionError unless there are as
    /// many tangents as points.
    /// </summary>
    public MultiPointConstraint(NCollection_Array1<Point3D> tabP, NCollection_Array1<Vector3D> tabVec)
        : base(tabP)
    {
        if (tabP.Length() != tabVec.Length())
        {
            throw new ConstructionErrorException();
        }

        _tabTang = new NCollection_HArray1<Vector3D>(1, tabVec.Length());

        int i, lower = tabVec.Lower();
        for (i = 1; i <= tabVec.Length(); i++)
        {
            _tabTang.SetValue(i, tabVec.Value(lower + i - 1));
        }
    }

    /// <summary>
    /// A MultiPoint of 3d points with tangency and curvature constraints. Throws Standard_ConstructionError unless
    /// there are as many tangents and curvatures as points.
    /// </summary>
    public MultiPointConstraint(NCollection_Array1<Point3D> tabP, NCollection_Array1<Vector3D> tabVec,
                                       NCollection_Array1<Vector3D> tabCur)
        : base(tabP)
    {
        if ((tabP.Length() != tabVec.Length()) || (tabP.Length() != tabCur.Length()))
        {
            throw new ConstructionErrorException();
        }

        _tabTang = new NCollection_HArray1<Vector3D>(1, tabVec.Length());
        int i, lower = tabVec.Lower();
        for (i = 1; i <= tabVec.Length(); i++)
        {
            _tabTang.SetValue(i, tabVec.Value(lower + i - 1));
        }

        _tabCurv = new NCollection_HArray1<Vector3D>(1, tabCur.Length());
        lower = tabCur.Lower();
        for (i = 1; i <= tabCur.Length(); i++)
        {
            _tabCurv.SetValue(i, tabCur.Value(lower + i - 1));
        }
    }

    /// <summary>
    /// A MultiPoint of 2d points with tangency constraints. Throws Standard_ConstructionError unless there are as
    /// many tangents as points.
    /// </summary>
    public MultiPointConstraint(NCollection_Array1<Point2D> tabP2d, NCollection_Array1<Vector2D> tabVec2d)
        : base(tabP2d)
    {
        if (tabP2d.Length() != tabVec2d.Length())
        {
            throw new ConstructionErrorException();
        }

        _tabTang2d = new NCollection_HArray1<Vector2D>(1, tabVec2d.Length());
        int i, lower = tabVec2d.Lower();
        for (i = 1; i <= tabVec2d.Length(); i++)
        {
            _tabTang2d.SetValue(i, tabVec2d.Value(lower + i - 1));
        }
    }

    /// <summary>
    /// A MultiPoint of 2d points with tangency and curvature constraints. Throws Standard_ConstructionError unless
    /// there are as many tangents and curvatures as points.
    /// </summary>
    public MultiPointConstraint(NCollection_Array1<Point2D> tabP2d, NCollection_Array1<Vector2D> tabVec2d,
                                       NCollection_Array1<Vector2D> tabCur2d)
        : base(tabP2d)
    {
        if ((tabP2d.Length() != tabVec2d.Length()) || (tabCur2d.Length() != tabP2d.Length()))
        {
            throw new ConstructionErrorException();
        }

        _tabTang2d = new NCollection_HArray1<Vector2D>(1, tabVec2d.Length());
        int i, lower = tabVec2d.Lower();
        for (i = 1; i <= tabVec2d.Length(); i++)
        {
            _tabTang2d.SetValue(i, tabVec2d.Value(lower + i - 1));
        }

        _tabCurv2d = new NCollection_HArray1<Vector2D>(1, tabCur2d.Length());
        lower = tabCur2d.Lower();
        for (i = 1; i <= tabCur2d.Length(); i++)
        {
            _tabCurv2d.SetValue(i, tabCur2d.Value(lower + i - 1));
        }
    }

    /// <summary>Sets the tangent of the 3d point of range <paramref name="index"/> (1 to NbPoints).</summary>
    public void SetTang(int index, in Vector3D tang)
    {
        _tabTang ??= new NCollection_HArray1<Vector3D>(1, _nbP);
        if ((index <= 0) || (index > _nbP))
        {
            throw new OutOfRangeException();
        }
        _tabTang.SetValue(index, tang);
    }

    /// <summary>The tangent of the 3d point of range <paramref name="index"/> (1 to NbPoints).</summary>
    public Vector3D Tang(int index)
    {
        if ((index <= 0) || (index > _nbP))
        {
            throw new OutOfRangeException();
        }
        return _tabTang!.Value(index);
    }

    /// <summary>Sets the tangent of the 2d point of range <paramref name="index"/> (NbPoints + 1 to NbPoints + NbPoints2d).</summary>
    public void SetTang2d(int index, in Vector2D tang2d)
    {
        _tabTang2d ??= new NCollection_HArray1<Vector2D>(1, _nbP2d);

        if ((index <= _nbP) || (index > _nbP + _nbP2d))
        {
            throw new OutOfRangeException();
        }
        _tabTang2d.SetValue(index - _nbP, tang2d);
    }

    /// <summary>The tangent of the 2d point of range <paramref name="index"/> (NbPoints + 1 to NbPoints + NbPoints2d).</summary>
    public Vector2D Tang2d(int index)
    {
        if ((index <= _nbP) || (index > _nbP + _nbP2d))
        {
            throw new OutOfRangeException();
        }
        return _tabTang2d!.Value(index - _nbP);
    }

    /// <summary>
    /// Sets the normal vector of the 3d point of range <paramref name="index"/> (1 to NbPoints), whose norm is the
    /// normal curvature.
    /// </summary>
    public void SetCurv(int index, in Vector3D curv)
    {
        _tabCurv ??= new NCollection_HArray1<Vector3D>(1, _nbP);
        if ((index <= 0) || (index > _nbP))
        {
            throw new OutOfRangeException();
        }
        _tabCurv.SetValue(index, curv);
    }

    /// <summary>The normal vector of the 3d point of range <paramref name="index"/> (1 to NbPoints).</summary>
    public Vector3D Curv(int index)
    {
        if ((index <= 0) || (index > _nbP))
        {
            throw new OutOfRangeException();
        }
        return _tabCurv!.Value(index);
    }

    /// <summary>
    /// Sets the normal vector of the 2d point of range <paramref name="index"/> (NbPoints + 1 to NbPoints +
    /// NbPoints2d), whose norm is the normal curvature.
    /// </summary>
    public void SetCurv2d(int index, in Vector2D curv2d)
    {
        _tabCurv2d ??= new NCollection_HArray1<Vector2D>(1, _nbP2d);
        if ((index <= _nbP) || (index > _nbP + _nbP2d))
        {
            throw new OutOfRangeException();
        }
        _tabCurv2d.SetValue(index - _nbP, curv2d);
    }

    /// <summary>The normal vector of the 2d point of range <paramref name="index"/> (NbPoints + 1 to NbPoints + NbPoints2d).</summary>
    public Vector2D Curv2d(int index)
    {
        if ((index <= _nbP) || (index > _nbP + _nbP2d))
        {
            throw new OutOfRangeException();
        }
        return _tabCurv2d!.Value(index - _nbP);
    }

    /// <summary>True if the MultiPoint has tangents.</summary>
    public bool IsTangencyPoint => !(_tabTang is null && _tabTang2d is null);

    /// <summary>True if the MultiPoint has curvatures.</summary>
    public bool IsCurvaturePoint => !(_tabCurv is null && _tabCurv2d is null);

    /// <summary>Writes a one-line description.</summary>
    public override void Dump(TextWriter o)
    {
        o.Write("AppDef_MultiPointConstraint dump:" + '\n');
    }
}
