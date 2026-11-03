// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Bisector.hxx
// Port of: src/ModelingAlgorithms/TKTopAlgo/MAT/MAT_Bisector.cxx
// Open CASCADE Technology V8_0_1 (b8f597c6). Copyright (c) 1991-1999 Matra Datavision; 1999-2026 OPEN CASCADE SAS.
// Copyright (c) 2026 The OCCT.NET Contributors. LGPL 2.1 with the Open CASCADE exception (see LICENSE).

using Kerf.Engine.Foundation;

namespace Occt;

/// <summary>A bisector in MAT2d_Mat2d's algorithm: its number, its two edges, its issue and end points, its vectors,
/// sense and parameters, and the list of the bisectors it was built from.</summary>
/// <remarks>A working object of MAT2d_Mat2d (decision 21): the setters are internal. OCCT leaves theindexnumber,
/// theissuepoint, theendpoint, the vectors, the sense and distissuepoint uninitialised; they are 0 in the port.
/// Dump (to std::cout) is left out.</remarks>
public sealed class MAT_Bisector : Transient
{
    private int thebisectornumber;
    private int theindexnumber;
    private MAT_Edge? thefirstedge;
    private MAT_Edge? thesecondedge;
    private readonly MAT_ListOfBisector thelistofbisectors;
    private int theissuepoint;
    private int theendpoint;
    private int thefirstvector;
    private int thesecondvector;
    private double thesense;
    private double thefirstparameter;
    private double thesecondparameter;
    private double distissuepoint;

    public MAT_Bisector()
    {
        thebisectornumber = -1;
        thefirstparameter = global::Kerf.Engine.Foundation.Precision.Infinite();
        thesecondparameter = global::Kerf.Engine.Foundation.Precision.Infinite();
        thelistofbisectors = new MAT_ListOfBisector();
    }

    internal void AddBisector(MAT_Bisector abisector) => thelistofbisectors.BackAdd(abisector);

    public MAT_ListOfBisector List() => thelistofbisectors;

    public MAT_Bisector FirstBisector() => thelistofbisectors.FirstItem();

    public MAT_Bisector LastBisector() => thelistofbisectors.LastItem();

    internal void BisectorNumber(int anumber) => thebisectornumber = anumber;

    internal void IndexNumber(int anumber) => theindexnumber = anumber;

    internal void FirstEdge(MAT_Edge? anedge) => thefirstedge = anedge;

    internal void SecondEdge(MAT_Edge? anedge) => thesecondedge = anedge;

    internal void IssuePoint(int apoint) => theissuepoint = apoint;

    internal void EndPoint(int apoint) => theendpoint = apoint;

    internal void DistIssuePoint(double areal) => distissuepoint = areal;

    internal void FirstVector(int avector) => thefirstvector = avector;

    internal void SecondVector(int avector) => thesecondvector = avector;

    internal void Sense(double asense) => thesense = asense;

    internal void FirstParameter(double aparameter) => thefirstparameter = aparameter;

    internal void SecondParameter(double aparameter) => thesecondparameter = aparameter;

    public int BisectorNumber() => thebisectornumber;

    public int IndexNumber() => theindexnumber;

    public MAT_Edge FirstEdge() => thefirstedge!;

    public MAT_Edge SecondEdge() => thesecondedge!;

    public int IssuePoint() => theissuepoint;

    public int EndPoint() => theendpoint;

    public double DistIssuePoint() => distissuepoint;

    public int FirstVector() => thefirstvector;

    public int SecondVector() => thesecondvector;

    public double Sense() => thesense;

    public double FirstParameter() => thefirstparameter;

    public double SecondParameter() => thesecondparameter;
}
