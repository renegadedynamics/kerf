// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_Line.hxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_Line.lxx
// Port of: src/ModelingAlgorithms/TKHLR/Contap/Contap_Line.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.Walking;
using Kerf.Geometry;
using Occt;
using static Kerf.Engine.HiddenLines.Contours.ContourLineType;
using static Kerf.Engine.Intersection.Walking.WalkingTypeTrans;

namespace Kerf.Engine.HiddenLines.Contours;

/// <summary>
/// A contour line: a line, a circle, a walking line (a polyline on the surface) or a part of a restriction, with its
/// vertices sorted by their parameter on the line, and its transition on the surface.
/// </summary>
/// <remarks>
/// OCCT's class is used by value but holds its polyline and its vertices through handles, which its copies share. The
/// port makes it a class with OCCT's (shallow) copy as the copy constructor; ported code copies where C++ copies
/// (decision 1077). <see cref="Vertex"/> returns a reference to the stored vertex, as OCCT's. As in OCCT, the default
/// constructor leaves the type unset (<see cref="Lin"/> here).
/// </remarks>
internal sealed class ContourLine
{
    private WalkingTypeTrans _trans;
    private LineOn2S? _curv;
    private NCollection_HSequence<ContourPoint> _svtx;
    private Curve2DAdaptorBase? _thearc;
    private ContourLineType _typL;
    private Point3D _pt = new();
    private Direction3D _dir1 = new();
    private Direction3D _dir2 = new();
    private double _rad;

    public ContourLine()
    {
        _svtx = new NCollection_HSequence<ContourPoint>();
        _trans = WalkingTypeTrans.Undecided;
    }

    /// <summary>OCCT's implicit copy: the polyline and the vertices are shared, as OCCT's handles.</summary>
    public ContourLine(ContourLine other)
    {
        _trans = other._trans;
        _curv = other._curv;
        _svtx = other._svtx;
        _thearc = other._thearc;
        _typL = other._typL;
        _pt = other._pt;
        _dir1 = other._dir1;
        _dir2 = other._dir2;
        _rad = other._rad;
    }

    /// <summary>Makes the line a walking line on the polyline <paramref name="l"/>.</summary>
    public void SetLineOn2S(LineOn2S l)
    {
        _curv = l;
        _typL = ContourLineType.Walking;
    }

    /// <summary>Clears the polyline (shared with the line's copies, as in OCCT) and the vertices; a walking line.</summary>
    public void Clear()
    {
        if (_curv is not null)
        {
            _curv.Clear();
        }
        _svtx = new NCollection_HSequence<ContourPoint>();
        _typL = ContourLineType.Walking;
    }

    /// <summary>The polyline of a walking line.</summary>
    public LineOn2S LineOn2S() => _curv!;

    /// <summary>Starts a new sequence of vertices (the copies keep the old one).</summary>
    public void ResetSeqOfVertex()
    {
        _svtx = new NCollection_HSequence<ContourPoint>();
    }

    /// <summary>Adds a point to the polyline.</summary>
    public void Add(in PntOn2S pOn2S)
    {
        _curv!.Add(pOn2S);
    }

    /// <summary>Makes the line the straight line <paramref name="l"/>.</summary>
    public void SetValue(in Line3D l)
    {
        _pt = l.Location;
        _dir1 = l.Direction;
        _typL = ContourLineType.Lin;
    }

    /// <summary>Makes the line the circle <paramref name="c"/>.</summary>
    public void SetValue(in Circle3D c)
    {
        _pt = c.Location;
        _dir1 = c.Position.Direction;
        _dir2 = c.Position.XDirection;
        _rad = c.Radius;
        _typL = ContourLineType.Circle;
    }

    /// <summary>Makes the line the restriction <paramref name="a"/>.</summary>
    public void SetValue(Curve2DAdaptorBase a)
    {
        _thearc = a;
        _typL = ContourLineType.Restriction;
    }

    /// <summary>Adds a vertex, keeping the vertices sorted by their parameter on the line.</summary>
    public void Add(in ContourPoint p)
    {
        int n = _svtx.Length();
        if (n == 0)
        {
            _svtx.Append(p);
        }
        else
        {
            double prm = p.ParameterOnLine();
            if (prm > _svtx.Value(n).ParameterOnLine())
            {
                _svtx.Append(p);
            }
            else
            {
                for (int i = n - 1; i > 0; i--)
                {
                    if (prm > _svtx.Value(i).ParameterOnLine())
                    {
                        _svtx.InsertBefore(i + 1, p);
                        return;
                    }
                }
                _svtx.Prepend(p);
            }
        }
    }

    public int VertexCount => _svtx.Length();

    /// <summary>The vertex <paramref name="index"/>, by reference (OCCT's non-const reference).</summary>
    public ref ContourPoint Vertex(int index) => ref _svtx.ChangeSequence()[index];

    /// <summary>Contap_Lin, Contap_Circle, Contap_Walking or Contap_Restriction.</summary>
    public ContourLineType TypeContour() => _typL;

    /// <summary>The number of points of a walking line; throws Standard_DomainError on another type.</summary>
    public int PntCount
    {
        get
        {
            if (_typL != ContourLineType.Walking)
            {
                throw new DomainErrorException();
            }
            return (_curv!.PointCount);
        }
    }

    /// <summary>The point <paramref name="index"/> of a walking line; throws Standard_DomainError on another type.</summary>
    public PntOn2S Point(int index)
    {
        if (_typL != ContourLineType.Walking)
        {
            throw new DomainErrorException();
        }
        return (_curv!.Value(index));
    }

    /// <summary>The straight line; throws Standard_DomainError on another type.</summary>
    public Line3D Line()
    {
        if (_typL != ContourLineType.Lin)
        {
            throw new DomainErrorException();
        }
        return new Line3D(_pt, _dir1);
    }

    /// <summary>The circle; throws Standard_DomainError on another type.</summary>
    public Circle3D Circle()
    {
        if (_typL != ContourLineType.Circle)
        {
            throw new DomainErrorException();
        }
        return new Circle3D(new Frame3D(_pt, _dir1, _dir2), _rad);
    }

    /// <summary>The restriction; throws Standard_DomainError on another type.</summary>
    public Curve2DAdaptorBase Arc()
    {
        if (_typL != ContourLineType.Restriction)
        {
            throw new DomainErrorException();
        }
        return _thearc!;
    }

    /// <summary>Sets the transition of the line.</summary>
    public void SetTransitionOnS(WalkingTypeTrans t)
    {
        _trans = t;
    }

    /// <summary>IN if, at the left of the line, the surface's normal is oriented towards the observer.</summary>
    public WalkingTypeTrans TransitionOnS() => (_trans);
}
