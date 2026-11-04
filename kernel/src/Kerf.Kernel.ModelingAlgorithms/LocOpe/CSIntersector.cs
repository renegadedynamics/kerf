// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_CSIntersector.hxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_CSIntersector.lxx
// Port of: src/ModelingAlgorithms/TKFeat/LocOpe/LocOpe_CSIntersector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1996-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Intersection.CurveSurface;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Engine.Intersection.CurveSurface.TransitionOnCurve;
using static Kerf.Topology.Orientation;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Features.Local;

/// <summary>
/// The intersection between a set of axes, circles or curves and the faces of a shape. The intersection points are
/// sorted in increasing parameter along each element.
/// </summary>
/// <remarks>A working object (decision 21): it only reads the shape; the points' faces are the shape's own.</remarks>
internal sealed class CSIntersector
{
    private bool _done;
    private Shape _shape = new();
    private NCollection_Sequence<PntFace>[]? _points;
    private int _nbelem;

    /// <summary>Empty constructor.</summary>
    public CSIntersector()
    {
        _done = false;
        _points = null;
        _nbelem = 0;
    }

    /// <summary>Creates the intersection with the shape <paramref name="s"/>.</summary>
    public CSIntersector(in Shape s)
    {
        _done = false;
        _shape = s;
        _points = null;
        _nbelem = 0;
    }

    /// <summary>Initializes the intersection with the shape <paramref name="s"/>.</summary>
    public void Init(in Shape s)
    {
        _done = false;
        _shape = s;
        _points = null;
        _nbelem = 0;
    }

    private void NewPoints()
    {
        _points = new NCollection_Sequence<PntFace>[_nbelem];
        for (int i = 0; i < _nbelem; i++)
        {
            _points[i] = new NCollection_Sequence<PntFace>();
        }
    }

    public void Perform(NCollection_Sequence<Line3D> slin)
    {
        if (_shape.IsNull || slin.Length() <= 0)
        {
            throw new ConstructionErrorException();
        }
        _done = false;

        _nbelem = slin.Length();
        NewPoints();

        double binf = RealFirst();
        double bsup = RealLast();
        Explorer exp = new(_shape, ShapeType.Face);
        for (; exp.More(); exp.Next())
        {
            Face theface = ShapeCasts.Face(exp.Current());
            CurveSurfaceIntersector theInt = new(theface, global::Kerf.Engine.Foundation.Precision.PConfusion());
            for (int i = 1; i <= _nbelem; i++)
            {
                theInt.Perform(slin[i], binf, bsup);
                if (theInt.IsDone)
                {
                    AddPoints(theInt, _points![i - 1], theface);
                }
            }
        }
        _done = true;
    }

    public void Perform(NCollection_Sequence<Circle3D> scir)
    {
        if (_shape.IsNull || scir.Length() <= 0)
        {
            throw new ConstructionErrorException();
        }
        _done = false;

        _nbelem = scir.Length();
        NewPoints();

        Explorer exp = new(_shape, ShapeType.Face);
        CurveAdaptor hc = new();
        double binf = 0.0;
        double bsup = 2.0 * Math.PI;

        for (; exp.More(); exp.Next())
        {
            Face theface = ShapeCasts.Face(exp.Current());
            CurveSurfaceIntersector theInt = new(theface, 0.0);
            for (int i = 1; i <= _nbelem; i++)
            {
                hc.Load(new CircleCurve3D(scir[i]));
                theInt.Perform(hc, binf, bsup);
                if (theInt.IsDone)
                {
                    AddPoints(theInt, _points![i - 1], theface);
                }
            }
        }
        _done = true;
    }

    public void Perform(NCollection_Sequence<Curve3D?> scur)
    {
        if (_shape.IsNull || scur.Length() <= 0)
        {
            throw new ConstructionErrorException();
        }
        _done = false;

        _nbelem = scur.Length();
        NewPoints();

        Explorer exp = new(_shape, ShapeType.Face);
        CurveAdaptor hc = new();
        for (; exp.More(); exp.Next())
        {
            Face theface = ShapeCasts.Face(exp.Current());
            CurveSurfaceIntersector theInt = new(theface, 0.0);
            for (int i = 1; i <= _nbelem; i++)
            {
                Curve3D? cur = scur[i];
                if (cur is null)
                {
                    continue;
                }
                hc.Load(cur);
                double binf = hc.FirstParameter();
                double bsup = hc.LastParameter();
                theInt.Perform(hc, binf, bsup);
                if (theInt.IsDone)
                {
                    AddPoints(theInt, _points![i - 1], theface);
                }
            }
        }
        _done = true;
    }

    /// <summary>Returns true if the intersection has been done.</summary>
    public bool IsDone => _done;

    /// <summary>Returns the number of intersection points on the element of range <paramref name="i"/>.</summary>
    public int NbPoints(int i)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (i <= 0 || i > _nbelem)
        {
            throw new OutOfRangeException();
        }
        return _points![i - 1].Length();
    }

    /// <summary>
    /// Returns the intersection point of range <paramref name="index"/> on the element of range <paramref name="i"/>,
    /// in increasing order of parameter along the element.
    /// </summary>
    public PntFace Point(int i, int index)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (i <= 0 || i > _nbelem)
        {
            throw new OutOfRangeException();
        }
        return _points![i - 1].Value(index);
    }

    public void Destroy()
    {
        _points = null;
    }

    /// <summary>
    /// On the element of range <paramref name="i"/>, searches the first intersection point located after the parameter
    /// <paramref name="From"/> whose orientation is not EXTERNAL. If found, returns true; <paramref name="or"/> is the
    /// orientation of the point, <paramref name="indFrom"/> and <paramref name="indTo"/> the interval of indices of the
    /// points it gathers. <paramref name="tol"/> decides when two parameters are equal. The outputs are left unchanged
    /// where OCCT leaves them.
    /// </summary>
    public bool LocalizeAfter(int i, double From, double tol, ref Orientation or, ref int indFrom, ref int indTo)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (i <= 0 || i > _nbelem)
        {
            throw new OutOfRangeException();
        }
        return LocAfter(_points![i - 1], From, tol, ref or, ref indFrom, ref indTo);
    }

    /// <summary>
    /// On the element of range <paramref name="i"/>, searches the first intersection point located before the
    /// parameter <paramref name="From"/> whose orientation is not EXTERNAL (see <see cref="LocalizeAfter(int, double, double, ref Orientation, ref int, ref int)"/>).
    /// </summary>
    public bool LocalizeBefore(int i, double From, double tol, ref Orientation or, ref int indFrom, ref int indTo)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (i <= 0 || i > _nbelem)
        {
            throw new OutOfRangeException();
        }
        return LocBefore(_points![i - 1], From, tol, ref or, ref indFrom, ref indTo);
    }

    /// <summary>
    /// On the element of range <paramref name="i"/>, searches the first intersection point after the index
    /// <paramref name="fromInd"/> whose orientation is not EXTERNAL.
    /// </summary>
    public bool LocalizeAfter(int i, int fromInd, double tol, ref Orientation or, ref int indFrom, ref int indTo)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (i <= 0 || i > _nbelem)
        {
            throw new OutOfRangeException();
        }
        return LocAfter(_points![i - 1], fromInd, tol, ref or, ref indFrom, ref indTo);
    }

    /// <summary>
    /// On the element of range <paramref name="i"/>, searches the first intersection point before the index
    /// <paramref name="fromInd"/> whose orientation is not EXTERNAL. As in OCCT, the index is taken as a parameter
    /// (occt-issues.md).
    /// </summary>
    public bool LocalizeBefore(int i, int fromInd, double tol, ref Orientation or, ref int indFrom, ref int indTo)
    {
        if (!_done)
        {
            throw new NotDoneException();
        }
        if (i <= 0 || i > _nbelem)
        {
            throw new OutOfRangeException();
        }
        // OCCT: there is no LocBefore by index; the int converts to the parameter From
        return LocBefore(_points![i - 1], (double)fromInd, tol, ref or, ref indFrom, ref indTo);
    }

    private static bool LocAfter(NCollection_Sequence<PntFace> spt, double From, double tol,
                                 ref Orientation or, ref int indFrom, ref int indTo)
    {
        double param, fmeps = From - tol;
        int i, ifirst, nbpoints = spt.Length();
        for (ifirst = 1; ifirst <= nbpoints; ifirst++)
        {
            if (spt.Value(ifirst).Parameter() >= fmeps)
            {
                break;
            }
        }
        bool retVal = false;
        if (ifirst <= nbpoints)
        {
            i = ifirst;
            indFrom = ifirst;
            bool found = false;
            while (!found)
            {
                or = spt.Value(i).Orientation();
                param = spt.Value(i).Parameter();
                i = i + 1;
                while (i <= nbpoints)
                {
                    if (spt.Value(i).Parameter() - param <= tol)
                    {
                        if (or != External && or != spt.Value(i).Orientation())
                        {
                            or = External;
                        }
                        i++;
                    }
                    else
                    {
                        break;
                    }
                }
                if (or == External)
                {
                    found = (i > nbpoints);
                    indFrom = i;
                }
                else
                { // on a une intersection franche
                    indTo = i - 1;
                    found = true;
                    retVal = true;
                }
            }
        }

        return retVal;
    }

    private static bool LocBefore(NCollection_Sequence<PntFace> spt, double From, double tol,
                                  ref Orientation or, ref int indFrom, ref int indTo)
    {
        double param, fpeps = From + tol;
        int i, ifirst, nbpoints = spt.Length();
        for (ifirst = nbpoints; ifirst >= 1; ifirst--)
        {
            if (spt.Value(ifirst).Parameter() <= fpeps)
            {
                break;
            }
        }
        bool retVal = false;
        if (ifirst >= 1)
        {
            i = ifirst;
            indTo = ifirst;
            bool found = false;
            while (!found)
            {
                or = spt.Value(i).Orientation();
                param = spt.Value(i).Parameter();
                i = i - 1;
                while (i >= 1)
                {
                    if (param - spt.Value(i).Parameter() <= tol)
                    {
                        if (or != External && or != spt.Value(i).Orientation())
                        {
                            or = External;
                        }
                        i--;
                    }
                    else
                    {
                        break;
                    }
                }
                if (or == External)
                {
                    found = (i < 1);
                    indTo = i;
                }
                else
                { // on a une intersection franche
                    indFrom = i + 1;
                    found = true;
                    retVal = true;
                }
            }
        }

        return retVal;
    }

    private static bool LocAfter(NCollection_Sequence<PntFace> spt, int fromInd, double tol,
                                 ref Orientation or, ref int indFrom, ref int indTo)
    {
        int nbpoints = spt.Length();
        if (fromInd >= nbpoints)
        {
            return false;
        }

        double param, fmeps;
        int i, ifirst;
        if (fromInd >= 1)
        {
            fmeps = spt.Value(fromInd).Parameter() - tol;
            for (ifirst = fromInd + 1; ifirst <= nbpoints; ifirst++)
            {
                if (spt.Value(ifirst).Parameter() >= fmeps)
                {
                    break;
                }
            }
        }
        else
        {
            ifirst = 1;
        }

        bool retVal = false;
        if (ifirst <= nbpoints)
        {
            i = ifirst;
            indFrom = ifirst;
            bool found = false;
            while (!found)
            {
                or = spt.Value(i).Orientation();
                param = spt.Value(i).Parameter();
                i = i + 1;
                while (i <= nbpoints)
                {
                    if (spt.Value(i).Parameter() - param <= tol)
                    {
                        if (or != External && or != spt.Value(i).Orientation())
                        {
                            or = External;
                        }
                        i++;
                    }
                    else
                    {
                        break;
                    }
                }
                if (or == External)
                {
                    found = (i > nbpoints);
                    indFrom = i;
                }
                else
                { // on a une intersection franche
                    indTo = i - 1;
                    found = true;
                    retVal = true;
                }
            }
        }
        return retVal;
    }

    /// <summary>
    /// Adds the points of <paramref name="theInt"/> on <paramref name="theface"/> to <paramref name="seq"/>, sorted
    /// by parameter (shared with LocOpe_CurveShapeIntersector, as in OCCT's two files).
    /// </summary>
    internal static void AddPoints(CurveSurfaceIntersector theInt, NCollection_Sequence<PntFace> seq, in Face theface)
    {
        int nbpoints = seq.Length();
        int newpnt = theInt.PntCount;
        double param, paramu, paramv;
        for (int j = 1; j <= newpnt; j++)
        {
            Point3D thept = theInt.Pnt(j);
            param = theInt.WParameter(j);
            paramu = theInt.UParameter(j);
            paramv = theInt.VParameter(j);

            Orientation theor = Forward;

            switch (theInt.Transition(j))
            {
                case TransitionOnCurve.In:
                    theor = Forward;
                    break;
                case TransitionOnCurve.Out:
                    theor = Reversed;
                    break;
                case TransitionOnCurve.Tangent:
                    theor = External;
                    break;
            }
            PntFace newpt = new(thept, theface, theor, param, paramu, paramv);
            int k;
            for (k = 1; k <= nbpoints; k++)
            {
                if (seq.Value(k).Parameter() - param > 0.0)
                {
                    break;
                }
            }
            if (k <= nbpoints)
            {
                seq.InsertBefore(k, newpt);
            }
            else
            {
                seq.Append(newpt);
            }
            nbpoints++;
        }
    }
}
