// Port of: src/ModelingAlgorithms/TKGeomAlgo/HatchGen/HatchGen_PointOnElement.hxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/HatchGen/HatchGen_PointOnElement.lxx
// Port of: src/ModelingAlgorithms/TKGeomAlgo/HatchGen/HatchGen_PointOnElement.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1993-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Intersection.TwoD;
using Occt;
using static Kerf.Engine.Classification.Hatching.IntersectionType;
using static Kerf.Engine.Intersection.TwoD.Position2D;
using static Kerf.Engine.Intersection.TwoD.Situation;
using static Kerf.Engine.Intersection.TwoD.TypeTrans;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.Containment;

namespace Kerf.Engine.Classification.Hatching;

/// <summary>An intersection point on an element of a hatcher, with the type of the intersection there.</summary>
/// <remarks>A value class of OCCT, copied where OCCT copies it (decision 664); its setters are internal.</remarks>
internal sealed class PointOnElement : IntersectionPoint
{
    private IntersectionType _type;

    /// <summary>An empty point on element.</summary>
    public PointOnElement()
    {
        _type = global::Kerf.Engine.Classification.Hatching.IntersectionType.Undetermined;
    }

    /// <summary>A copy of <paramref name="other"/>.</summary>
    public PointOnElement(PointOnElement other)
        : base(other)
    {
        _type = other._type;
    }

    /// <summary>
    /// The point on the element (the second curve of <paramref name="point"/>), with the states before and after it
    /// along the hatching (the first curve).
    /// </summary>
    public PointOnElement(in IntersectionPoint2D point)
    {
        _type = global::Kerf.Engine.Classification.Hatching.IntersectionType.Undetermined;
        Transition trsH = point.TransitionOfFirst();
        Transition trsE = point.TransitionOfSecond();

        _index = 0;

        _param = point.ParamOnSecond();

        switch (trsE.PositionOnCurve())
        {
            case Position2D.Head:
                _posit = Forward;
                break;
            case Position2D.Middle:
                _posit = Internal;
                break;
            case Position2D.End:
                _posit = Reversed;
                break;
        }

        switch (trsH.TransitionType())
        {
            case TypeTrans.In:
            {
                _before = global::Kerf.Topology.Containment.Out;
                _after = global::Kerf.Topology.Containment.In;
                _type = (_posit == Internal) ? global::Kerf.Engine.Classification.Hatching.IntersectionType.True : global::Kerf.Engine.Classification.Hatching.IntersectionType.Touch;
                break;
            }
            case TypeTrans.Out:
            {
                _before = global::Kerf.Topology.Containment.In;
                _after = global::Kerf.Topology.Containment.Out;
                _type = (_posit == Internal) ? global::Kerf.Engine.Classification.Hatching.IntersectionType.True : global::Kerf.Engine.Classification.Hatching.IntersectionType.Touch;
                break;
            }
            //  Modified by Sergey KHROMOV - Fri Jan  5 12:07:34 2001 Begin
            case TypeTrans.Touch:
            {
                switch (trsH.Situation())
                {
                    case Situation.Inside:
                    {
                        _type = global::Kerf.Engine.Classification.Hatching.IntersectionType.Tangent;
                        switch (_posit)
                        {
                            case Forward:
                            {
                                if (trsE.IsOpposite)
                                {
                                    _before = global::Kerf.Topology.Containment.In;
                                    _after = global::Kerf.Topology.Containment.Out;
                                }
                                else
                                {
                                    _before = global::Kerf.Topology.Containment.Out;
                                    _after = global::Kerf.Topology.Containment.In;
                                }
                                break;
                            }
                            case Internal:
                            {
                                _before = global::Kerf.Topology.Containment.In;
                                _after = global::Kerf.Topology.Containment.In;
                                break;
                            }
                            case Reversed:
                            {
                                if (trsE.IsOpposite)
                                {
                                    _before = global::Kerf.Topology.Containment.Out;
                                    _after = global::Kerf.Topology.Containment.In;
                                }
                                else
                                {
                                    _before = global::Kerf.Topology.Containment.In;
                                    _after = global::Kerf.Topology.Containment.Out;
                                }
                                break;
                            }
                            case External:
                                break;
                        }
                        break;
                    }
                    case Situation.Outside:
                    {
                        _type = global::Kerf.Engine.Classification.Hatching.IntersectionType.Tangent;
                        switch (_posit)
                        {
                            case Forward:
                            {
                                if (trsE.IsOpposite)
                                {
                                    _before = global::Kerf.Topology.Containment.Out;
                                    _after = global::Kerf.Topology.Containment.In;
                                }
                                else
                                {
                                    _before = global::Kerf.Topology.Containment.In;
                                    _after = global::Kerf.Topology.Containment.Out;
                                }
                                break;
                            }
                            case Internal:
                            {
                                _before = global::Kerf.Topology.Containment.Out;
                                _after = global::Kerf.Topology.Containment.Out;
                                break;
                            }
                            case Reversed:
                            {
                                if (trsE.IsOpposite)
                                {
                                    _before = global::Kerf.Topology.Containment.In;
                                    _after = global::Kerf.Topology.Containment.Out;
                                }
                                else
                                {
                                    _before = global::Kerf.Topology.Containment.Out;
                                    _after = global::Kerf.Topology.Containment.In;
                                }
                                break;
                            }
                            case External:
                                break;
                        }
                        break;
                    }
                    case Situation.Unknown:
                    {
                        _before = global::Kerf.Topology.Containment.Unknown;
                        _after = global::Kerf.Topology.Containment.Unknown;
                        _type = global::Kerf.Engine.Classification.Hatching.IntersectionType.Tangent;
                        break;
                    }
                }
                break;
            }
            //  Modified by Sergey KHROMOV - Fri Jan  5 12:07:46 2001 End
            case TypeTrans.Undecided:
            {
                _before = global::Kerf.Topology.Containment.Unknown;
                _after = global::Kerf.Topology.Containment.Unknown;
                _type = global::Kerf.Engine.Classification.Hatching.IntersectionType.Undetermined;
                break;
            }
        }
        _segBeg = false;
        _segEnd = false;
    }

    /// <summary>Sets the intersection type at this point.</summary>
    internal void SetIntersectionType(IntersectionType type)
    {
        _type = type;
    }

    /// <summary>The intersection type at this point.</summary>
    public IntersectionType IntersectionType() => _type;

    /// <summary>
    /// True if the point is identical to <paramref name="point"/>: the same index, position, type, states and segment
    /// flags, and parameters within <paramref name="confusion"/>.
    /// </summary>
    public bool IsIdentical(PointOnElement point, double confusion)
    {
        double delta = Math.Abs(_param - point._param);
        return ((delta <= confusion) && (_index == point._index) && (_posit == point._posit)
                && (_type == point._type) && (_before == point._before) && (_after == point._after)
                && (_segBeg == point._segBeg) && (_segEnd == point._segEnd));
    }

    /// <summary>True if the point is different from <paramref name="point"/> (not <see cref="IsIdentical"/>).</summary>
    public bool IsDifferent(PointOnElement point, double confusion)
    {
        double delta = Math.Abs(_param - point._param);
        return ((delta > confusion) || (_index != point._index) || (_posit != point._posit)
                || (_type != point._type) || (_before != point._before) || (_after != point._after)
                || (_segBeg != point._segBeg) || (_segEnd != point._segEnd));
    }

    /// <summary>Writes the point to <paramref name="os"/>, as OCCT writes it to std::cout.</summary>
    public override void Dump(TextWriter os, int index = 0)
    {
        os.Write("    --- Point on element ");
        if (index > 0)
        {
            os.Write($"# {index,3} ");
        }
        else
        {
            os.Write("------");
        }
        os.Write("---------------\n");
        os.Write($"        Index of the element = {_index}\n");
        os.Write($"        Parameter on element = {Number(_param)}\n");
        os.Write($"        Position  on element = {PositionName(_posit)}\n");
        os.Write("        Intersection Type    = ");
        switch (_type)
        {
            case global::Kerf.Engine.Classification.Hatching.IntersectionType.True:
                os.Write("TRUE");
                break;
            case global::Kerf.Engine.Classification.Hatching.IntersectionType.Touch:
                os.Write("TOUCH");
                break;
            case global::Kerf.Engine.Classification.Hatching.IntersectionType.Tangent:
                os.Write("TANGENT");
                break;
            case global::Kerf.Engine.Classification.Hatching.IntersectionType.Undetermined:
                os.Write("UNDETERMINED");
                break;
        }
        os.Write('\n');
        os.Write($"        State Before         = {StateName(_before)}\n");
        os.Write($"        State After          = {StateName(_after)}\n");
        os.Write($"        Beginning of segment = {(_segBeg ? "TRUE" : "FALSE")}\n");
        os.Write($"        End       of segment = {(_segEnd ? "TRUE" : "FALSE")}\n");
        os.Write("    ------------------------------------------\n");
    }
}
