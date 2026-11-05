// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Bisector.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Bisector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The Kerf Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;
using Kerf.Engine;

namespace Kerf.Engine.Offsets.MedialAxis;

/// <summary>A bisector in MAT2d_Mat2d's algorithm: its number, its two edges, its issue and end points, its vectors,
/// sense and parameters, and the list of the bisectors it was built from.</summary>
/// <remarks>A working object of MAT2d_Mat2d (decision 21): the setters are internal. OCCT leaves theindexnumber,
/// theissuepoint, theendpoint, the vectors, the sense and distissuepoint uninitialised; they are 0 in the port.
/// Dump (to std::cout) is left out.</remarks>
internal sealed class MedialBisector : Transient
{
    private int _thebisectornumber;
    private int _theindexnumber;
    private MedialEdge? _thefirstedge;
    private MedialEdge? _thesecondedge;
    private readonly MedialListOfBisector _thelistofbisectors;
    private int _theissuepoint;
    private int _theendpoint;
    private int _thefirstvector;
    private int _thesecondvector;
    private double _thesense;
    private double _thefirstparameter;
    private double _thesecondparameter;
    private double _distissuepoint;

    public MedialBisector()
    {
        _thebisectornumber = -1;
        _thefirstparameter = global::Kerf.Engine.Foundation.Precision.Infinite();
        _thesecondparameter = global::Kerf.Engine.Foundation.Precision.Infinite();
        _thelistofbisectors = new MedialListOfBisector();
    }

    internal void AddBisector(MedialBisector abisector) => _thelistofbisectors.BackAdd(abisector);

    public MedialListOfBisector List() => _thelistofbisectors;

    public MedialBisector FirstBisector() => _thelistofbisectors.FirstItem();

    public MedialBisector LastBisector() => _thelistofbisectors.LastItem();

    internal void BisectorNumber(int anumber) => _thebisectornumber = anumber;

    internal void IndexNumber(int anumber) => _theindexnumber = anumber;

    internal void FirstEdge(MedialEdge? anedge) => _thefirstedge = anedge;

    internal void SecondEdge(MedialEdge? anedge) => _thesecondedge = anedge;

    internal void IssuePoint(int apoint) => _theissuepoint = apoint;

    internal void EndPoint(int apoint) => _theendpoint = apoint;

    internal void DistIssuePoint(double areal) => _distissuepoint = areal;

    internal void FirstVector(int avector) => _thefirstvector = avector;

    internal void SecondVector(int avector) => _thesecondvector = avector;

    internal void Sense(double asense) => _thesense = asense;

    internal void FirstParameter(double aparameter) => _thefirstparameter = aparameter;

    internal void SecondParameter(double aparameter) => _thesecondparameter = aparameter;

    public int BisectorNumber() => _thebisectornumber;

    public int IndexNumber() => _theindexnumber;

    public MedialEdge FirstEdge() => _thefirstedge!;

    public MedialEdge SecondEdge() => _thesecondedge!;

    public int IssuePoint() => _theissuepoint;

    public int EndPoint() => _theendpoint;

    public double DistIssuePoint() => _distissuepoint;

    public int FirstVector() => _thefirstvector;

    public int SecondVector() => _thesecondvector;

    public double Sense() => _thesense;

    public double FirstParameter() => _thefirstparameter;

    public double SecondParameter() => _thesecondparameter;
}
