// Port of: src/ModelingAlgorithms/TKTopAlgo/IntCurvesFace/IntCurvesFace_ShapeIntersector.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/IntCurvesFace/IntCurvesFace_ShapeIntersector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1998-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Geometry.Adaptors;
using Kerf.Engine.Topology;
using Kerf.Geometry;
using Kerf.Topology;
using Occt;
using static Kerf.Topology.ShapeType;

namespace Kerf.Engine.Intersection.CurveSurface;

/// <summary>
/// The intersection of a line or a curve with the faces of a shape: the points IN or ON the faces, sorted along the
/// curve.
/// </summary>
/// <remarks>
/// A working object (decision 21): load a shape, then intersect any number of curves. As in OCCT, a second
/// <see cref="Load"/> adds the new shape's faces to those already loaded. It only reads the shape.
/// </remarks>
internal class ShapeIntersector
{
    private bool _isDone;
    private int _nbFaces;
    private NCollection_Array1<int> _ptrNums = new();
    private NCollection_Array1<int> _ptrIndexNums = new();
    private readonly NCollection_Sequence<CurveSurfaceIntersector> _intersector = new();
    private readonly NCollection_Sequence<int> _indexPt = new();
    private readonly NCollection_Sequence<int> _indexFace = new();
    private readonly NCollection_Sequence<int> _indexIntPnt = new();
    private readonly NCollection_Sequence<double> _indexPar = new();

    /// <summary>An empty intersector.</summary>
    public ShapeIntersector()
    {
        _isDone = false;
        _nbFaces = 0;
    }

    /// <summary>Loads the faces of <paramref name="shape"/>, with the tolerance <paramref name="tol"/> (see <see cref="CurveSurfaceIntersector"/>).</summary>
    public void Load(in Shape shape, double tol)
    {
        // OCCT doesn't clear the intersectors of a shape loaded before: they stay first in the sequence, and myNbFaces
        // counts only the new faces, so a second Load intersects the old shape's faces (occt-issues.md)
        var ex = new Explorer();
        int i;
        for (_nbFaces = 0, i = 0, ex.Init(shape, global::Kerf.Topology.ShapeType.Face); ex.More(); ++i, ex.Next())
        {
            ++_nbFaces;
            Face currentFace = ShapeCasts.Face(ex.Current());
            _intersector.Append(new CurveSurfaceIntersector(currentFace, tol));
        }
    }

    /// <summary>
    /// Intersects the segment [<paramref name="parMin"/>, <paramref name="parMax"/>] of the line
    /// <paramref name="l"/> (they may be ∓RealLast) with the loaded faces.
    /// </summary>
    public void Perform(in Line3D l, double parMin, double parMax)
    {
        _isDone = false;
        for (int i = 1; i <= _nbFaces; ++i)
        {
            _intersector[i].Perform(l, parMin, parMax);
        }

        SortResult();
    }

    /// <summary>
    /// As <see cref="Perform(in Line3D, double, double)"/>, but each face is intersected only up to the nearest
    /// point found so far; the faces met first are tried first at the next call.
    /// </summary>
    public void PerformNearest(in Line3D l, double parMin, double parMax)
    {
        int i = 0;
        if (_nbFaces > 2)
        {
            if (_ptrNums.IsEmpty())
            {
                _ptrNums = new NCollection_Array1<int>(0, _nbFaces - 1);
                _ptrIndexNums = new NCollection_Array1<int>(0, _nbFaces - 1);
                for (; i < _nbFaces; ++i)
                {
                    _ptrNums[i] = 0;
                    _ptrIndexNums[i] = i + 1;
                }
            }
        }

        int indexFace = -1;
        double aParMax = parMax;
        _isDone = false;
        for (int ii = 1; ii <= _nbFaces; ++ii)
        {
            if (!_ptrNums.IsEmpty())
            {
                i = _ptrIndexNums.Value(ii - 1);
            }
            else
            {
                i = ii;
            }

            CurveSurfaceIntersector intersector = _intersector[i];
            // OCCT: a face skipped here keeps the points of its previous Perform, which SortResult then reports
            // (occt-issues.md)
            if (parMin < aParMax)
            {
                intersector.Perform(l, parMin, aParMax);
                if (intersector.IsDone)
                {
                    int n = intersector.PntCount;
                    for (int j = 1; j <= n; ++j)
                    {
                        double w = intersector.WParameter(j);
                        if (w < aParMax)
                        {
                            aParMax = w;
                            indexFace = ii - 1;
                        }
                    }
                }
                else
                {
                    _isDone = false;
                    return;
                }
            }
        }

        if (!_ptrNums.IsEmpty() && indexFace >= 0)
        {
            _ptrNums[indexFace] += 1;
            int im1;
            for (im1 = indexFace - 1, i = indexFace;
                 i >= 1 && _ptrNums.Value(i) > _ptrNums.Value(im1);
                 --i, --im1)
            {
                (_ptrIndexNums[i], _ptrIndexNums[im1]) = (_ptrIndexNums[im1], _ptrIndexNums[i]);
                (_ptrNums[i], _ptrNums[im1]) = (_ptrNums[im1], _ptrNums[i]);
            }
        }

        SortResult();
    }

    /// <summary>
    /// Intersects the part [<paramref name="parMin"/>, <paramref name="parMax"/>] of the curve
    /// <paramref name="hCurve"/> with the loaded faces.
    /// </summary>
    public void Perform(CurveAdaptorBase hCurve, double parMin, double parMax)
    {
        _isDone = false;
        for (int i = 1; i <= _nbFaces; ++i)
        {
            CurveSurfaceIntersector intersector = _intersector[i];
            intersector.Perform(hCurve, parMin, parMax);
        }

        SortResult();
    }

    /// <summary>True when the intersection has been computed.</summary>
    public bool IsDone => _isDone;

    /// <summary>The number of intersection points.</summary>
    public int PntCount => _indexPt.Length();

    /// <summary>The U parameter of the <paramref name="i"/>-th point on its face's surface.</summary>
    public double UParameter(int i)
    {
        CurveSurfaceIntersector intAdaptor = _intersector[_indexFace[_indexPt[i]]];
        return intAdaptor.UParameter(_indexIntPnt[_indexPt[i]]);
    }

    /// <summary>The V parameter of the <paramref name="i"/>-th point on its face's surface.</summary>
    public double VParameter(int i)
    {
        CurveSurfaceIntersector intAdaptor = _intersector[_indexFace[_indexPt[i]]];
        return intAdaptor.VParameter(_indexIntPnt[_indexPt[i]]);
    }

    /// <summary>The parameter of the <paramref name="i"/>-th point on the line or curve.</summary>
    public double WParameter(int i)
    {
        CurveSurfaceIntersector intAdaptor = _intersector[_indexFace[_indexPt[i]]];
        return intAdaptor.WParameter(_indexIntPnt[_indexPt[i]]);
    }

    /// <summary>The <paramref name="i"/>-th point.</summary>
    public Point3D Pnt(int i)
    {
        CurveSurfaceIntersector intAdaptor = _intersector[_indexFace[_indexPt[i]]];
        return intAdaptor.Pnt(_indexIntPnt[_indexPt[i]]);
    }

    /// <summary>The transition of the line or curve on the surface at the <paramref name="i"/>-th point.</summary>
    public TransitionOnCurve Transition(int i)
    {
        CurveSurfaceIntersector intAdaptor = _intersector[_indexFace[_indexPt[i]]];
        return intAdaptor.Transition(_indexIntPnt[_indexPt[i]]);
    }

    /// <summary>The state of the <paramref name="i"/>-th point in its face: IN, or ON its boundary.</summary>
    public Containment State(int i)
    {
        CurveSurfaceIntersector intAdaptor = _intersector[_indexFace[_indexPt[i]]];
        return intAdaptor.State(_indexIntPnt[_indexPt[i]]);
    }

    /// <summary>The face of the <paramref name="i"/>-th point.</summary>
    public Face Face(int i)
    {
        CurveSurfaceIntersector intAdaptor = _intersector[_indexFace[_indexPt[i]]];
        return intAdaptor.Face();
    }

    //-- ================================================================================
    //-- myIntersector   : Sequence of the addresses
    //-- myIndexPt          : 1 2 3 .... n  Points before the sorting
    //-- myNumberFace       : Number of the face (of the intersector) of the point myIndexPt(i)
    //-- myNumberIntPnt     : Number of the point  myIndexPt(i) of the intersection
    // myNumberFace(myIndexPt(i))
    //-- myIndexPar         : Parameter W of point myIndexPt(i)
    //--
    //-- To sum up, for each point index of K = myIndexPt(i) on a
    //--      * the face to which it belongs                               : myNumberFace(K)
    //--      * the number of the point in the intersection for FaceCurve  : myNumberIntPnt(K)
    //--      * the parameter W of the point on the curve                  : myIndexPar(K)
    //--
    //-- SortResult Sorts the points in ascending order of W
    //-- (updating the index table TabPt(.))
    //--
    //-- ================================================================================

    /// <summary>Internal: collects the faces' points and sorts them along the curve.</summary>
    public void SortResult()
    {
        _isDone = true;
        int nbPnt = 0;
        _indexPt.Clear();
        _indexFace.Clear();
        _indexIntPnt.Clear();
        _indexPar.Clear();

        // Retrieval of the results
        for (int f = 1; f <= _nbFaces; ++f)
        {
            CurveSurfaceIntersector intersector = _intersector[f];
            if (intersector.IsDone)
            {
                int n = intersector.PntCount;
                for (int j = 1; j <= n; ++j)
                {
                    _indexPt.Append(++nbPnt);
                    _indexFace.Append(f);
                    _indexIntPnt.Append(j);
                    _indexPar.Append(intersector.WParameter(j));
                }
            }
            else
            {
                _isDone = false;
                return;
            }
        }

        // Sort according to parameter  w
        bool isOK;
        do
        {
            isOK = true;
            for (int ind0 = 1; ind0 < nbPnt; ind0++)
            {
                int ind = _indexPt[ind0];
                int indp1 = _indexPt[ind0 + 1];
                if (_indexPar[ind] > _indexPar[indp1])
                {
                    _indexPt[ind0] = indp1;
                    _indexPt[ind0 + 1] = ind;
                    isOK = false;
                }
            }
        } while (!isOK);
    }
}
