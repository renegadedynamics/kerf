// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_SurfData.hxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_SurfData.lxx
// Port of: src/ModelingAlgorithms/TKFillet/ChFiDS/ChFiDS_SurfData.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.Orientation;

namespace Kerf.Engine.Fillets.Data;

/// <summary>
/// The data of one fillet surface and its two faces: the indices of the faces, the fillet surface and the curves in
/// the data structure, the interferences with the faces, the four corner points and the spine parameters.
/// </summary>
/// <remarks>
/// A working object of the fillet builders (decision 21): they fill it while they build the stripes, through the
/// internal mutators and the <c>Change*</c> accessors that return the stored values by reference. The public
/// members only read.
/// </remarks>
public sealed class SurfData : Transient
{
    private CommonPoint _pfirstOnS1 = new();
    private CommonPoint _plastOnS1 = new();
    private CommonPoint _pfirstOnS2 = new();
    private CommonPoint _plastOnS2 = new();
    private FaceInterference _intf1 = new();
    private FaceInterference _intf2 = new();
    private Point2D _p2df1 = new();
    private Point2D _p2dl1 = new();
    private Point2D _p2df2 = new();
    private Point2D _p2dl2 = new();
    private double _ufspine;
    private double _ulspine;
    private double _myfirstextend;
    private double _mylastextend;
    private object? _simul; // OCCT: a Standard_Transient handle; the port's NCollection_HArray1 is not one (unit 7N2)
    private int _indexOfS1;
    private int _indexOfC1;
    private int _indexOfS2;
    private int _indexOfC2;
    private int _indexOfConge;
    private bool _isoncurv1;
    private bool _isoncurv2;
    private bool _twistons1;
    private bool _twistons2;
    private Orientation _orientation;

    public SurfData()
    {
        _ufspine = 0.0;
        _ulspine = 0.0;
        _myfirstextend = 0.0;
        _mylastextend = 0.0;
        _indexOfS1 = 0;
        _indexOfC1 = 0;
        _indexOfS2 = 0;
        _indexOfC2 = 0;
        _indexOfConge = 0;
        _isoncurv1 = false;
        _isoncurv2 = false;
        _twistons1 = false;
        _twistons2 = false;
        _orientation = Forward;
    }

    /// <summary>
    /// Copies <paramref name="other"/>'s data into this one, except (as OCCT) the indices of the curves and the
    /// flags that the interferences are on curves.
    /// </summary>
    internal void Copy(SurfData other)
    {
        _indexOfS1 = other._indexOfS1;
        _indexOfS2 = other._indexOfS2;
        _indexOfConge = other._indexOfConge;
        _orientation = other._orientation;
        _intf1 = other._intf1;
        _intf2 = other._intf2;
        _pfirstOnS1 = other._pfirstOnS1;
        _plastOnS1 = other._plastOnS1;
        _pfirstOnS2 = other._pfirstOnS2;
        _plastOnS2 = other._plastOnS2;

        _ufspine = other._ufspine;
        _ulspine = other._ulspine;

        _simul = other._simul;
        _p2df1 = other._p2df1;
        _p2dl1 = other._p2dl1;
        _p2df2 = other._p2df2;
        _p2dl2 = other._p2dl2;

        _myfirstextend = other._myfirstextend;
        _mylastextend = other._mylastextend;

        _twistons1 = other._twistons1;
        _twistons2 = other._twistons2;
    }

    public int IndexOfS1()
    {
        return _indexOfS1;
    }

    public int IndexOfS2()
    {
        return _indexOfS2;
    }

    public bool IsOnCurve1
    {
        get
        {
            return _isoncurv1;
        }
    }

    public bool IsOnCurve2
    {
        get
        {
            return _isoncurv2;
        }
    }

    /// <summary>The index of the curve on the first face; throws <see cref="FailureException"/> if not on a curve.</summary>
    public int IndexOfC1()
    {
        if (!_isoncurv1)
        {
            throw new FailureException("Interference pas sur courbe");
        }

        return _indexOfC1;
    }

    /// <summary>The index of the curve on the second face; throws <see cref="FailureException"/> if not on a curve.</summary>
    public int IndexOfC2()
    {
        if (!_isoncurv2)
        {
            throw new FailureException("Interference pas sur courbe");
        }

        return _indexOfC2;
    }

    /// <summary>The index of the fillet surface in the data structure.</summary>
    public int Surf()
    {
        return _indexOfConge;
    }

    public Orientation Orientation()
    {
        return _orientation;
    }

    public FaceInterference InterferenceOnS1()
    {
        return _intf1;
    }

    public FaceInterference InterferenceOnS2()
    {
        return _intf2;
    }

    public CommonPoint VertexFirstOnS1()
    {
        return _pfirstOnS1;
    }

    public CommonPoint VertexFirstOnS2()
    {
        return _pfirstOnS2;
    }

    public CommonPoint VertexLastOnS1()
    {
        return _plastOnS1;
    }

    public CommonPoint VertexLastOnS2()
    {
        return _plastOnS2;
    }

    internal void ChangeIndexOfS1(int index)
    {
        _indexOfS1 = index;
    }

    internal void ChangeIndexOfS2(int index)
    {
        _indexOfS2 = index;
    }

    internal void ChangeSurf(int index)
    {
        _indexOfConge = index;
    }

    internal void SetIndexOfC1(int index)
    {
        _indexOfC1 = index;
        _isoncurv1 = (index != 0);
    }

    internal void SetIndexOfC2(int index)
    {
        _indexOfC2 = index;
        _isoncurv2 = (index != 0);
    }

    internal ref Orientation ChangeOrientation() => ref _orientation;

    internal ref FaceInterference ChangeInterferenceOnS1() => ref _intf1;

    internal ref FaceInterference ChangeInterferenceOnS2() => ref _intf2;

    internal ref CommonPoint ChangeVertexFirstOnS1() => ref _pfirstOnS1;

    internal ref CommonPoint ChangeVertexFirstOnS2() => ref _pfirstOnS2;

    internal ref CommonPoint ChangeVertexLastOnS1() => ref _plastOnS1;

    internal ref CommonPoint ChangeVertexLastOnS2() => ref _plastOnS2;

    /// <summary>The interference on face <paramref name="onS"/> (1 or 2).</summary>
    public FaceInterference Interference(int onS)
    {
        if (onS == 1)
        {
            return _intf1;
        }
        else
        {
            return _intf2;
        }
    }

    internal ref FaceInterference ChangeInterference(int onS)
    {
        if (onS == 1)
        {
            return ref _intf1;
        }
        else
        {
            return ref _intf2;
        }
    }

    /// <summary>The index of face <paramref name="ofS"/> (1 or 2).</summary>
    public int Index(int ofS)
    {
        if (ofS == 1)
        {
            return _indexOfS1;
        }
        else
        {
            return _indexOfS2;
        }
    }

    /// <summary>The first or last point on face <paramref name="onS"/>.</summary>
    public CommonPoint Vertex(bool first, int onS)
    {
        if (first && onS == 1)
        {
            return _pfirstOnS1;
        }
        else if (first && onS == 2)
        {
            return _pfirstOnS2;
        }
        else if (!first && onS == 1)
        {
            return _plastOnS1;
        }
        else
        {
            return _plastOnS2;
        }
    }

    internal ref CommonPoint ChangeVertex(bool first, int onS)
    {
        if (first && onS == 1)
        {
            return ref _pfirstOnS1;
        }
        else if (first && onS == 2)
        {
            return ref _pfirstOnS2;
        }
        else if (!first && onS == 1)
        {
            return ref _plastOnS1;
        }
        else
        {
            return ref _plastOnS2;
        }
    }

    public bool IsOnCurve(int onS)
    {
        if (onS == 1)
        {
            return _isoncurv1;
        }

        return _isoncurv2;
    }

    public int IndexOfC(int onS)
    {
        if (onS == 1)
        {
            if (!_isoncurv1)
            {
                throw new FailureException("Interference pas sur courbe");
            }

            return _indexOfC1;
        }

        if (!_isoncurv2)
        {
            throw new FailureException("Interference pas sur courbe");
        }

        return _indexOfC2;
    }

    public double FirstSpineParam()
    {
        return _ufspine;
    }

    public double LastSpineParam()
    {
        return _ulspine;
    }

    internal void FirstSpineParam(double par)
    {
        _ufspine = par;
    }

    internal void LastSpineParam(double par)
    {
        _ulspine = par;
    }

    public double FirstExtensionValue()
    {
        return _myfirstextend;
    }

    public double LastExtensionValue()
    {
        return _mylastextend;
    }

    internal void FirstExtensionValue(double extend)
    {
        _myfirstextend = extend;
    }

    internal void LastExtensionValue(double extend)
    {
        _mylastextend = extend;
    }

    /// <summary>The simulated sections of the fillet (ChFiDS_SecHArray1, set by the simulation), or null.</summary>
    public object? Simul()
    {
        return _simul;
    }

    internal void SetSimul(object? s)
    {
        _simul = s;
    }

    internal void ResetSimul()
    {
        _simul = null;
    }

    /// <summary>The first or last 2d point on face <paramref name="onS"/>.</summary>
    public Point2D Get2dPoints(bool first, int onS)
    {
        if (first && onS == 1)
        {
            return _p2df1;
        }
        else if (!first && onS == 1)
        {
            return _p2dl1;
        }
        else if (first && onS == 2)
        {
            return _p2df2;
        }

        return _p2dl2;
    }

    public void Get2dPoints(out Point2D p2df1, out Point2D p2dl1, out Point2D p2df2, out Point2D p2dl2)
    {
        p2df1 = _p2df1;
        p2dl1 = _p2dl1;
        p2df2 = _p2df2;
        p2dl2 = _p2dl2;
    }

    internal void Set2dPoints(in Point2D p2df1, in Point2D p2dl1, in Point2D p2df2, in Point2D p2dl2)
    {
        _p2df1 = p2df1;
        _p2dl1 = p2dl1;
        _p2df2 = p2df2;
        _p2dl2 = p2dl2;
    }

    public bool TwistOnS1()
    {
        return _twistons1;
    }

    public bool TwistOnS2()
    {
        return _twistons2;
    }

    internal void TwistOnS1(bool t)
    {
        _twistons1 = t;
    }

    internal void TwistOnS2(bool t)
    {
        _twistons2 = t;
    }
}
